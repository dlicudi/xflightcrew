//! The loop: `spawn_autoland`, whole.

use super::*;

/// The fastest an altitude hold returns to its altitude and is still holding
/// it, fpm (a 172's ALT came back at 900 after the setup's dive).
const LEVEL_RETURN_MAX_FPM: f64 = 1_000.0;

/// The plant's reads (`theta`, `alpha`, `q`, `vy_true`, `elev_total`, `tas`,
/// `ail_total`, `rud_total`, `beta` on the autoland tick), in this order:
/// logged, and the attitude law's measurements.
const PITCH_PLANT: [&str; 9] = [
    sim::datarefs::THETA_DEG,
    sim::datarefs::ALPHA_DEG,
    sim::datarefs::PITCH_RATE_DEG_S,
    sim::datarefs::LOCAL_VY_MS,
    sim::datarefs::TOTAL_PITCH_RATIO,
    sim::datarefs::TRUE_AIRSPEED_MS,
    // The lateral plant (2026-10-12): the ailerons and rudder the
    // autopilot or our hands give, and the sideslip, so the roll response
    // can be read while the autopilot flies and a rudder law designed on
    // data (the tapes have none airborne).
    sim::datarefs::TOTAL_ROLL_RATIO,
    sim::datarefs::TOTAL_HEADING_RATIO,
    sim::datarefs::SIDESLIP_DEG,
];

pub(super) fn spawn_autoland(launch: AutolandLaunch) {
    let AutolandLaunch {
        base,
        apt_dat,
        cfg,
        approach_ias,
        voice,
        forced_rwy,
        parents,
        hand_down,
    } = launch;
    // The last approach's tracking goes before the flag rises (it is
    // unreadable once ACTIVE is set again, not at the first tick).
    TRACKING_ERROR.clear();
    // The flight this run lands: its writes to the flight's facts are
    // stamped with it, so a run stood down by a new flight writes nothing
    // that flight reads.
    let flight = crate::flight_state::flight_gen();
    // Taken on the caller's thread: a stand-down landing before the thread
    // runs cancels THIS run. The run's end clears the panel line and the
    // landing configuration, only while it still owns the slot.
    let lease = RUN.take_over().with_cleanup(&LANDING_CONFIGURED, || {
        set_panel("");
        ON_FINAL.store(false, Ordering::SeqCst);
    });
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        let lease = &lease;
        rt.block_on(async move {
            // Kept for a re-arm on a different runway (see the clearance
            // watch in the tick): the PARENTS only, never this run's own
            // token, which the re-arm's take-over cancels.
            let base_url = base.clone();
            let xp = XPlaneClient::new(base)
                .with_parents(parents.clone())
                .with_cancel(lease.cancel.clone());
            if xp.cancelled() {
                return;
            }
            // The flare and the rollout fly the plugin's yoke channels.
            crate::yoke::arm(&xp, "autoland").await;
            LANDING_CONFIGURED.store(false, Ordering::SeqCst);
            ON_FINAL.store(false, Ordering::SeqCst);
            let announce = |line: String, sev: Priority| {
                tracing::info!(target: "xflightcrew::autoland", spoke = %line, "copilot");
                if let Some(v) = &voice { v.alert(sev, line); }
            };
            // The approach speed is a number somebody handed us; the airframe's
            // stall floor is a fact. A Cessna's 65 kt on an RV-10 whose clean
            // stall is 65 flew the speed schedule into a stall (2026-09-11).
            // The LANDING configuration's floor, not the clean one: the 737's
            // Vref 145 floored to the clean 167 overflew LFLL 35L twice
            // (2026-09-17; `aircraft::approach_floor_kts`).
            // The approach speed for what the aircraft weighs NOW, on an
            // airframe that declares its Vref table (`weight_speeds:`): Vref
            // with the landing flap plus the additive, as a crew reads it
            // off the card. The Zibo at 45.7 t flew a fixed 145 where Vref40
            // was 116 and touched nosewheel first (LEMG 30, 2026-10-07).
            let approach_ias = match crate::aircraft::approach_kts_now(&xp).await {
                Some((kts, kg)) => {
                    // The weight just read is the floor's too: on a fresh
                    // load no monitor tick had published one yet, and the
                    // heavy declared floor took a weighed 117 back up to 137
                    // (LEMG 30 reposition, 2026-10-07, #232).
                    crate::aircraft::publish_weight_now(Some(kg));
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "approach_ias_weighed", "declared": approach_ias,
                            "kts": kts, "weight_kg": kg.round(),
                            "floor": crate::aircraft::approach_floor_kts()}),
                    );
                    kts
                }
                None => approach_ias,
            };
            let approach_ias = {
                let floored = crate::aircraft::floored_target(approach_ias, crate::aircraft::approach_floor_kts());
                if floored > approach_ias {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "approach_ias_floored", "asked": approach_ias, "floor": floored}),
                    );
                    announce(
                        format!("Approach speed {approach_ias:.0} is below this airframe's {floored:.0} knot floor. Using {floored:.0}."),
                        Priority::Caution,
                    );
                }
                floored
            };

            // This approach's path angle: a steeper one handed down for a
            // visual segment (LSZS), else 3.
            let gp_deg = arm_glidepath(hand_down.glidepath);
            // The stabilised gate this final is judged at (#199): a
            // circuit's is visual, every other the instrument gate.
            let gate = hand_down.gate;
            if (gp_deg - 3.0).abs() > 1e-9 {
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "glidepath", "deg": gp_deg}),
                );
            }
            let ArmedRunway {
                runway: rwy,
                ils,
                coupled_ok,
                lands,
            } = match arm_runway(&xp, &apt_dat, forced_rwy.as_ref(), gp_deg, &voice).await {
                Some(armed) => armed,
                // The run ends with the thread (the lease drops the flag).
                None => return,
            };
            // No angle handed down: the runway's own, from its PAPI where it
            // has one (`nav::papi_for`), the published visual path at a
            // field with no ILS and no procedure (LXGB's 3.00, both ends).
            let gp_deg = if hand_down.glidepath.is_none() {
                let (path, icao) = (apt_dat.clone(), rwy.icao.clone());
                let (lat, lon, crs) = (rwy.thr_lat, rwy.thr_lon, rwy.course_deg);
                let papi = tokio::task::spawn_blocking(move || {
                    crate::nav::find_airport(&path, &icao)
                        .ok()
                        .flatten()
                        .and_then(|a| crate::nav::papi_for(&a.papis, lat, lon, crs))
                })
                .await
                .ok()
                .flatten();
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "papi", "rwy": rwy.id, "angle": papi.map(|p| p.angle_deg)}),
                );
                match papi {
                    Some(p) => arm_glidepath(Some((p.angle_deg, true))),
                    None => gp_deg,
                }
            } else {
                gp_deg
            };
            // THE ENERGY PLAN (`approach_energy`): the airframe's own idle
            // deceleration, planned back from this final's gate. `None` on an
            // airframe that declares none: the distance schedule flies.
            let energy_model = crate::aircraft::live_profile()
                .and_then(|p| crate::approach_energy::EnergyModel::from_profile(&p, gp_deg));
            // What the gate judges the speed against (the stable sample's).
            let energy_vapp = approach_target_kts(
                approach_ias,
                1.0,
                &crate::aircraft::flap_limits(),
                crate::aircraft::config_floor_kts(1.0),
            );
            // A planned final never accelerates: the speed it asked for is a
            // ceiling for the rest of it (the curve is a MAXIMUM, and a
            // governor handed a maximum flies it as a target).
            let mut energy_cap: Option<f64> = None;
            if let Some(m) = &energy_model {
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "energy_model", "vapp": energy_vapp, "gp_deg": gp_deg,
                        "path_decel": m.detents.iter().map(|d| (d.ratio, m.path_decel(d.ratio))).collect::<Vec<_>>()}),
                );
            }

            // Engage the AP (FD up first for airframes that need it), and
            // CONFIRM it: the tick re-presses until the servos read on
            // (`ApEngage`), holding the aircraft level until then.
            let engage_t0 = std::time::Instant::now();
            let mut engage = ApEngage::default();
            let mut engage_pitch = LevelHold::with_gains(engage_hold_gains());
            let mut engage_held = false;
            {
                let e = engage.arm(0.0);
                let ok = crate::ap_engage::press(&xp, &cfg.fd_commands).await;
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "ap_engage", "try": e.try_n, "sent": ok,
                        "at_s": 0.0, "servos": null, "ias": null}),
                );
            }
            // The altitude dial OUT OF THE WAY. Autoland flies the vertical
            // through the VS dial and never touched the altitude dial, so it
            // sat wherever the follower or the pilot left it: at LEAX the
            // plan's 2,600 ft constraint, 130 ft under the arm, and the AP's
            // altitude capture was already arresting the descent as the arm
            // came. The VS dial's -1,200 was ignored for six seconds, flaps
            // ballooned it to +300 fpm, and only the divergence watchdog's
            // re-press of VS mode freed it ("not holding the descent",
            // 2026-09-15). Parked well above and VS re-selected so no capture
            // can start; a go-around levels there at the latest, which the
            // per-tick VS re-select then takes down again.
            // The airframe's window rule (`autoflight::WindowRule`): above
            // (the LEAX rule), or on an MCP that flies a V/S only toward its
            // window (the 737) at the field, since an approach descends.
            let window_rule = autoflight::panel().window;
            {
                let (dial_r, alt_r) = tokio::join!(
                    autoflight::read_dial(&xp, Dial::Altitude),
                    xp.get_float(ALT_IND_FT)
                );
                if let (Ok(dial), Ok(alt_ind)) = (dial_r, alt_r) {
                    if let Some(parked) = parked_window_ft(window_rule, dial, alt_ind, rwy.elev_ft) {
                        let _ = autoflight::set_dial(&xp, Dial::Altitude, parked).await;
                        // Read before it is fired: on a toggle MCP (the Zibo)
                        // pressing a lit V/S turns it OFF, which this press
                        // did at LEMG 30's arm (2026-10-07). A failed read
                        // presses nothing; the tick's reconcile brings it.
                        let vs_on = autoflight::read_status(&xp, autoflight::names::VS_STATUS).await;
                        if vs_on.is_ok_and(|v| v <= 0.5) {
                            autoflight::press_logged(&xp, Button::VerticalSpeed, "autoland_arm").await;
                        }
                        crate::blackbox::record("autoland", serde_json::json!({
                            "event": "alt_dial_parked", "was": dial, "now": parked, "alt_ft": alt_ind,
                            "rule": window_rule,
                        }));
                    }
                }
            }

            TRACKING_ERROR.clear();
            let mut last_phase: Option<ApproachPhase> = None;
            // The heading knob, set as a pilot sets it (`bug_knob`, issue #93).
            let mut bug_hold = crate::heading_bug::BugHold::default();
            let knob_t0 = std::time::Instant::now();
            // The stabilised-approach rule (`stable_approach`, #199): the
            // breaches held so far, the clock they are timed on, the balk
            // word and spoken reason of a go-around it ordered, and whether
            // this approach's gate crossing has been recorded.
            // Watch the tower's go-around orders from here: an order shouted
            // at somebody else's approach half an hour ago must not balk this
            // one on its first tick. A WATCH, never the main loop's `take_*`
            // flag: sharing that flag, the main loop's FSM sync consumed every
            // order before this loop's tick saw it (LEMG 31, 2026-09-28).
            // The pilot's order rides the same watch (`go_around`, #77).
            // The crew's own watch on the landing clearance (#155).
            // The final report owed on a SECOND approach. The circuit or the
            // arrival makes the first one, which is what earns the landing
            // clearance; a missed approach then flies away, and until this
            // existed nothing called again — eleven minutes of missed
            // approach at Malaga with the tower waiting on "report downwind",
            // then a landing without a clearance (2026-08-05).
            // The "going around" transmission: said once per balked landing,
            // with the callsign read from the aircraft the first time it's
            // needed (an HTTP read, so not on every tick).
            // The balk being flown until it climbs (then the call) or the
            // wheels take it (then no call: see [`go_around_call_due`]), and
            // whether the intercom said "going around" for it.
            let mut runtime = ApproachRuntime::arm();
            // The current missed approach's (or pattern's) progress, judged
            // by the ground it makes (`reversal::MissedWatch`), on a clock of
            // this loop's own; and the speed the pattern began at, which
            // sizes its track to go.
            let mut missed_watch: Option<crate::reversal::MissedWatch> = None;
            let watch_epoch = std::time::Instant::now();
            let mut pattern_gs_kts = 0.0_f64;
            let mut gear_down = false;
            let mut landing_config_done = false;
            // The flap the wing is flying on at the arm: the schedule's first
            // write must know what it would take away (`flap_kept_flying`).
            // A failed read is no measurement: unknown, as before.
            let mut last_flap: Option<f64> = xp
                .get_float(FLAP_RATIO)
                .await
                .ok()
                .filter(|f| f.is_finite() && (0.0..=1.0).contains(f));
            let mut vs_diverged_s: f64 = 0.0;
            let mut prev_vs_err: Option<f64> = None;
            // Warned once per approach when the VSI cannot be read.
            let mut vvi_read_failed = false;
            // The vertical speed one tick ago, for the arrest's trend term.
            let mut prev_vvi: Option<(f64, std::time::Instant)> = None;
            // The arrest by attitude, judged by height (`pilot::attitude`).
            let mut attitude_flare = crate::pilot::attitude::AttitudeFlare::default();
            let mut energy_throttle = crate::pilot::energy::EnergyThrottle::default();
            let mut decel_brake = DecelBrake::default();
            let mut path_nudge = super::path_nudge::PathNudge::default();
            let path_clock = std::time::Instant::now();
            // The lever the loop last commanded, for the tick record (the
            // flare's table had the elevator and the sink but not the power).
            let mut last_lever: Option<f64> = None;
            // Last tick was the flare: a balloon stays in it (`hold_flare`).
            let mut flaring = false;
            // This flare was committed (`commit_flare_latched`).
            let mut flare_committed = false;
            // The roundout gate's go/no-go for this approach (`landing_call`).
            let mut landing_called: Option<LandingCall> = None;
            let mut pitch_yielded = false;
            // The hand-flown segment has the servos, from minimums to the
            // wheels (`arrest_hands_step`); the roundout has begun under it
            // (`hold_roundout` keeps it); the ailerons on the bank
            // (`pilot::bank`, its roll response measured as it flies).
            let mut arrest_hands = false;
            let mut roundout_taken = false;
            // This tick's arrest is only `hold_roundout`'s (a balloon over
            // the band), for the blackbox's `roundout_held` edges.
            let mut roundout_held = false;
            let mut bank_hold = crate::pilot::bank::BankHold::default();
            // What the hand-flown segment flies on: this airframe's measured
            // responses (`xfc identify`, `aircraft::learned`), else the
            // fleet's priors; the throttle's energy; the nose's geometry.
            let pilot_airframe = crate::pilot::airframe();
            crate::blackbox::record(
                "autoland",
                serde_json::json!({"event": "pilot_airframe", "measured": pilot_airframe.measured,
                    "pitch_b": pilot_airframe.pitch.b, "roll_b": pilot_airframe.roll.b,
                    "yaw_b": pilot_airframe.yaw.b, "k_thr2ste": pilot_airframe.tecs.fpm_per_unit,
                    "spool_s": pilot_airframe.tecs.spool_s, "theta_max": pilot_airframe.limits.max_deg,
                    "theta_min_td": pilot_airframe.limits.touchdown_min_deg}),
            );
            // The centreline's period (NPFG): bounded by the bank loop's own
            // answer on the roll response measured.
            let line_period_s = |ias: f64, tas: f64, dt: f64| {
                let g = crate::pilot::response::design(&pilot_airframe.roll.at(ias, tas), dt);
                crate::pilot::line::period_s(crate::pilot::response::attitude_tc_s(&g))
            };
            // The rudder in the hand-flown segment (`pilot::yaw`): the slip
            // to zero, then the decrab; the pilot's feet take it back.
            let mut yaw_hold = crate::pilot::yaw::YawHold::default();
            let mut steer_yielded = false;
            // The final's heading above minimums, the autopilot's standing
            // error learnt as it flies (`FinalHeading`).
            let mut final_heading = FinalHeading::default();
            let mut arrest_rate = crate::autotakeoff::XtkRate::default();
            let mut arrest_prev_xtk: Option<f64> = None;
            let mut roll_yielded = false;
            // Airspeed history for the throttle governor's derivative term — the
            // anti-stall reflex needs the TREND (kts/sec), not just the error.
            let mut prev_ias: Option<f64> = None;
            let mut prev_ias_t = std::time::Instant::now();
            // Cross-track history for the intercept damping term.
            let mut prev_xtk: Option<(f64, std::time::Instant)> = None;
            let mut rollout_rate = crate::autotakeoff::XtkRate::default();
            let mut xtk_rate: f64 = 0.0;
            // Wind crab, low-passed — see [`CrabFilter`] for the hunt it ends.
            let mut crab_filter = CrabFilter::new();
            // Rollout: latched at touchdown, then brakes down — to a full stop,
            // or to taxi pace when a vacate drive follows. The takeoff roll law
            // (P + trim, tested) holds the centreline through the plugin steer
            // channel while it brakes.
            let mut rolling_out = false;
            let mut wheels_on: Option<f64> = None;
            // When the last live tick read the wheels (the wheels-on clock).
            let mut wheels_read_t: Option<std::time::Instant> = None;
            let mut vacate_follows = false;
            let mut roll_law = crate::autotakeoff::RollSteer::default();
            // A castoring nosewheel steers the rollout on its brakes too
            // (`RolloutBrakeSteer`, issue #166).
            let rollout_brake_steer = rollout_brake_steers();
            // The tiller below the hand-off, where the airframe has one
            // (`crate::tiller`): the 737's pedals turn the nose 7 degrees.
            let mut rollout_tiller = crate::tiller::Tiller::for_airframe();
            rollout_tiller.arm(&xp).await;
            let mut rollout_brakes = RolloutBrakeSteer::for_airframe();
            // A steered nosewheel that declares its own rollout law
            // (`RolloutSteer`, issue #197); else the takeoff roll's default.
            let mut rollout_steer = crate::aircraft::rollout_steer().map(RolloutSteer::new);
            // Reverse on the rollout, where the airframe declares it
            // (`rollout_reverse`, #222): its engine count, and whether this
            // landing has selected it (once; the watcher stows it).
            let rollout_reverse_engines = crate::aircraft::live_profile()
                .filter(|p| p.reverse_thrust && p.rollout_reverse)
                .map(|p| p.engines);
            let mut reverse_selected = false;
            // The wheels off again after the rollout latched (a bounce).
            let mut bounce_hold = BounceHold::default();
            // The landing readback last sent, so an owed one goes out once
            // (the party line keeps it owed until the tower hears it).
            let mut readback_sent: Option<String> = None;
            // The cleared ident already looked up against apt.dat, so the
            // (seconds-long) lookup of one that is not at this field runs once.
            let mut clearance_checked: Option<String> = None;
            // The course reversal in progress, if any (see [`crate::reversal`]):
            // how this loop gets round to a final it is on the wrong side of.
            let mut reversal: Option<crate::reversal::Reversal> = None;
            let mut reversals_flown: u32 = 0;
            // Where this pattern's downwind stops for no-go airspace while
            // high, and the hold short of it (`reversal::limit_step`).
            let mut reversal_limit: Option<crate::reversal::AirspaceLimit> = None;
            let mut hold_back = false;
            let mut limit_said = false;
            // A published missed approach asked for once: one that cannot be
            // flown falls through to our own pattern, never to "Your aircraft"
            // with nobody there (LEMG 13 SR22 circuit 2026-09-29).
            let mut missed_procedure_tried = false;
            // The pattern's plan over the probed ground (#182), probed beside
            // the loop (the plugin answers in seconds; the aircraft flies on),
            // and when it began. Begun in the go-around's climb, so it is
            // ready when the reposition wants it.
            let mut reversal_plan: Option<(
                tokio::task::JoinHandle<crate::reversal::PatternPlan>,
                std::time::Instant,
            )> = None;
            // Straight ahead at a held altitude: while that plan is probed,
            // or because no pattern here clears the ground (#182).
            let mut hold: Option<TerrainHold> = None;
            // The live floor over the ground under and ahead on every leg
            // but the final (#182, the circuit's `live_floor_ft`): the floor
            // and when it tripped, when it was last probed, and whether the
            // spell has been said.
            let mut live_floor: Option<(f64, std::time::Instant)> = None;
            let mut live_floor_probed: Option<std::time::Instant> = None;
            let mut live_floor_said = false;
            // How far out this approach has legitimately been: the arm
            // distance, pushed out by any pattern flown since.
            let mut recede_ref_nm = rwy.dist_nm;
            // A give-up happened: the speed is handed to the flight governor
            // once the loop has ended (see below).
            let mut gave_up = false;
            // The confirmed engage's pitch authority is checked from the
            // next tick with a climb read (#181).
            let mut authority_owed = false;
            let mut ils_xcheck = crate::ils::XCheck::default();
            // The tick's writes, sent in one round (`sim::state::Writes`),
            // and the lever last written, for `lever_took`.
            let mut writes = sim::state::Writes::new();
            let mut last_thr_written: Option<f64> = None;
            let mut coupling = crate::ils::Coupling::Off;
            let mut coupling_armed_s = 0.0_f64;
            // The autopilot lands it (`ap_lands`, where declared): who flies
            // the landing, the A/T on the speed window, the window last
            // confirmed, the rollout still the autopilot's, on its clock.
            let mut ap_landing = crate::ap_lands::Landing::default();
            let mut at_speed = crate::ap_lands::AtSpeed::default();
            let mut at_window_last: Option<f64> = None;
            let mut ap_rollout = false;
            let ap_t0 = std::time::Instant::now();
            // The thrust is this loop's (`autoflight::ThrustWatch`): an
            // ENGAGED autothrottle is taken out to ARM, said once. The A/T
            // in SPEED 230 from a manual recovery overrode every lever
            // written from 13:02 to 13:09 (LEMG 13, 2026-10-05).
            let mut thrust_watch = autoflight::ThrustWatch::default();
            let thrust_t0 = std::time::Instant::now();
            loop {
                if xp.cancelled() || !lease.current() {
                    break;
                }
                // The tick runs to its period (`tick_ms`), not sleep + work.
                let pacer = sim::state::Pacer::start();
                // Frozen sim (replay or pause): a replay's positions are a
                // recording and not ours to fly; a pause is real but frozen,
                // and ticking the throttle governor against a frozen airspeed
                // walks the lever to its clamp for a surge on unpause. Freeze
                // (no writes, no state, no blackbox) and resume when it's live.
                //
                // ONE read a tick (`sim::state`): the freeze flags, position,
                // height, groundspeed and heading as ONE frame saw them (#157:
                // read separately, latitude and longitude paired two frames,
                // metres of cross-track for a tick at speed), and with them
                // everything the tick used to ask for later: the VSI, the
                // lever (the profile's own beside the frame), the servos, the
                // mode lights, the bug, the ILS needles and the yields. Every
                // round of requests is a sim frame: the freeze check, this
                // read, the VSI and lever, the ILS, the knob and yields, the
                // lever write and its read back made Final 0.56 s (`power` dt
                // median) against a 200 ms sleep and the Flare 0.39 s against
                // 100 ms on the iMac at 20 fps (2026-09-30), where a MacBook
                // ran near 0.3. An older plugin falls back to separate reads,
                // still one round.
                let thr_name = throttle_dataref();
                // Beside the frame, in its round: the lever, the autopilot's
                // mode bitfield and the autothrottle (`crate::autoflight`:
                // the modes' one truth, and a thrust this loop owns).
                // The panel's reads are the owner's (`Panel::tick_names`):
                // the stock bitfield and autothrottle, or a declared MCP's
                // own lamps (the Zibo's), in the same round.
                let af_panel = autoflight::panel();
                let mut extra_names: Vec<String> = vec![thr_name.clone()];
                extra_names.extend(af_panel.tick_names());
                // The pitch plant, every tick: the attitude flare's
                // measurements, and the true vertical speed the power is
                // flown on (`energy_throttle`).
                let panel_n = extra_names.len() - 1;
                extra_names.extend(PITCH_PLANT.iter().map(|n| n.to_string()));
                let extra_refs: Vec<&str> = extra_names.iter().map(String::as_str).collect();
                // ...and on a landing the autopilot flies (`ap_lands`), its
                // FMA, second channel and speed window, in the same round.
                let ((st, extra), lands_read) = tokio::join!(
                    async {
                        if ils.is_some() {
                            sim::state::read_with(&xp, sim::state::Want(AUTOLAND_WANT_ILS), &extra_refs).await
                        } else {
                            sim::state::read_with(&xp, sim::state::Want(AUTOLAND_WANT), &extra_refs).await
                        }
                    },
                    async {
                        match &lands {
                            Some(l) => Some(crate::ap_lands::read(&xp, l, &af_panel).await),
                            None => None,
                        }
                    },
                );
                if st.frozen() {
                    tokio::time::sleep(std::time::Duration::from_millis(400)).await;
                    continue;
                }
                let mut extra = extra.into_iter();
                let mut unread = || Err(anyhow::anyhow!("unread"));
                let mut throttle_read = Some(extra.next().unwrap_or_else(&mut unread));
                let mut panel_reads: Vec<Option<f64>> = extra.map(|r| r.ok()).collect();
                let plant = panel_reads.split_off(panel_n.min(panel_reads.len()));
                let plant_at = |i: usize| plant.get(i).copied().flatten();
                let panel_tick = af_panel.tick_from(&panel_reads);
                // The FMA's captures beside the lamps (`ap_lands`): the
                // Zibo's APP lamp is lit armed or captured alike.
                let fma = lands
                    .as_ref()
                    .zip(lands_read.as_ref())
                    .map(|(l, r)| crate::ap_lands::Fma::decode(r, &l.codes));
                let ap_modes = match &fma {
                    Some(f) => crate::ap_lands::modes_with_fma(panel_tick.modes, f),
                    None => panel_tick.modes,
                };
                // The two channels' lamps, and whether the autopilot flies
                // this landing (as the last tick decided it).
                let cmd_a_lit = lands
                    .as_ref()
                    .and(panel_tick.status(&st, F::Servos).ok())
                    .map(|v| v > 0.5);
                let cmd_b_lit = lands_read.as_ref().and_then(|r| r.cmd_b).map(|v| v >= 0.5);
                let ap_flies = ap_landing.ap_flies();
                // An A/T this path engaged is the airframe's, not a fight.
                let at_engaged = if at_speed.quiets_thrust_watch() {
                    None
                } else {
                    panel_tick.at_engaged
                };
                // Before the lever is written: an engaged autothrottle fights it.
                {
                    let step = thrust_watch.step(thrust_t0.elapsed().as_secs_f64(), at_engaged);
                    if let Some(line) = autoflight::act_thrust(&xp, step, "autoland").await {
                        announce(line.to_string(), Priority::Caution);
                    }
                }
                let pos = st.position();
                let (var_r, ias_r, hpath_r, ground_r, phi_r) = (
                    st.get(F::MagVar),
                    st.get(F::Ias),
                    st.get(F::Hpath),
                    st.get(F::OnGround),
                    st.get(F::Phi),
                );
                let (lat_r, lon_r) = sim::position::lat_lon(&pos);
                let elev_r = sim::position::field(&pos, |p| p.elev_m);
                let agl_r = sim::position::field(&pos, |p| p.agl_m);
                let gs_r = sim::position::field(&pos, |p| p.gs_ms);
                let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
                // The flight model's vertical speed, from the frame the height
                // came from: the gauge (`vvi`) lags it by about a second, so
                // the touchdown is scored on this one (#198). Logged only
                // until each airframe's flare is refitted to it.
                let vs_fm = sim::position::field(&pos, |p| p.vs_fpm).ok();
                let pos_source = sim::position::source_word(&pos);
                let (Ok(lat), Ok(lon), Ok(elev_m), Ok(agl_m), Ok(gs_ms), Ok(var), Ok(ias), Ok(on_ground)) =
                    (lat_r, lon_r, elev_r, agl_r, gs_r, var_r, ias_r, ground_r)
                else { tokio::time::sleep(std::time::Duration::from_millis(300)).await; continue };
                let alt = elev_m * M_TO_FT;
                let radio_ft = agl_m * M_TO_FT;
                let gs_kts = gs_ms * MS_TO_KTS;

                // THE TOWER'S WORD DURING THE APPROACH. The clearance was read
                // once, at arm time; the final report that EARNS it goes out
                // after that, so the answer was never seen: "negative, runway
                // zero niner, I say again runway zero niner ... cleared to
                // land" came back five seconds after the pick, and autoland
                // flew 27 into a 21 kt tailwind (LXGB, 2026-09-14). Now the
                // landing clearance is read back (the runway-critical echo
                // the tower waits for, the crew's job while a loop flies),
                // and a clearance for ANOTHER end of THIS field re-arms the
                // approach on it: a fresh loop, forced onto the cleared
                // runway, repositioning from wherever we are. A departure
                // state's runway is the departure's; a stale one is never
                // ours to land on.
                if on_ground < 0.5 && !rolling_out {
                    let cleared = crate::party_line::clearance_runway();
                    match crate::party_line::readback_owed() {
                        None => readback_sent = None,
                        Some(owed) if landing_readback(&owed) && readback_sent.as_deref() != Some(&owed) => {
                            readback_sent = Some(owed.clone());
                            if runtime.going_around_callsign.is_none() {
                                runtime.going_around_callsign = Some(crate::crew_radio::crew_callsign(&xp).await);
                            }
                            let cs = runtime.going_around_callsign.clone().flatten();
                            let rwy_words = cleared
                                .as_deref()
                                .map(|r| format!(" runway {}", crate::crew_radio::spell_runway(r)))
                                .unwrap_or_default();
                            crate::crew_radio::transmit(crate::crew_radio::radio_call(
                                &cs,
                                &format!("cleared to land{rwy_words}"),
                            ));
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "landing_readback", "rwy": cleared}),
                            );
                        }
                        Some(_) => {}
                    }
                    // A re-sequence the go-around order named is taken once
                    // the climb away is flown (#182).
                    let expected = if runtime.go_around.climbing() {
                        None
                    } else {
                        runtime.orders.expected_runway()
                    };
                    if let Some(c) = clearance_switch(
                        &rwy.id,
                        crate::party_line::clearance_state(),
                        cleared.as_deref(),
                        clearance_checked.as_deref(),
                        runtime.go_around.active(),
                        expected.as_deref(),
                    ) {
                        clearance_checked = Some(c.clone());
                        let resequenced = runtime.go_around.active();
                        match runway_by_id(&apt_dat, lat, lon, &c, 40.0, Some(rwy.icao.as_str())) {
                            Ok(Some(r)) if r.icao == rwy.icao => {
                                crate::blackbox::record(
                                    "autoland",
                                    serde_json::json!({"event": "runway_from_clearance",
                                        "planned": rwy.id, "cleared": r.id, "in_flight": true,
                                        "resequenced": resequenced,
                                        "radio": radio_ft, "dist": r.dist_nm}),
                                );
                                announce(
                                    if resequenced {
                                        format!(
                                            "Tower expects us on runway {}. Setting up for runway {}.",
                                            crate::crew_radio::spell_runway(&r.id),
                                            crate::crew_radio::spell_runway(&r.id)
                                        )
                                    } else {
                                        format!(
                                            "Tower cleared us for runway {}, not {}. Setting up for runway {}.",
                                            crate::crew_radio::spell_runway(&r.id),
                                            crate::crew_radio::spell_runway(&rwy.id),
                                            crate::crew_radio::spell_runway(&r.id)
                                        )
                                    },
                                    Priority::Caution,
                                );
                                // The mission's ONE arrival runway moves
                                // with us, before the new loop arms: its
                                // ILS tune, the crew's calls, the vacate
                                // and the card read the cleared end (#64).
                                crate::full_auto::retarget_arrival_runway(
                                    &r.id,
                                    if resequenced { "resequenced" } else { "clearance" },
                                );
                                // A new loop owns the aircraft from its first
                                // tick; this one is taken over (cancelled)
                                // and leaves ACTIVE to it. Our parents are
                                // its parents.
                                run_autoland_scoped(
                                    base_url.clone(),
                                    apt_dat.clone(),
                                    cfg.clone(),
                                    approach_ias,
                                    voice.clone(),
                                    None,
                                    Some(PlannedRunway::at(&r.icao, &r.id)),
                                    parents.clone(),
                                    HandDown::default(),
                                );
                                break;
                            }
                            other => {
                                crate::blackbox::record(
                                    "autoland",
                                    serde_json::json!({"event": "clearance_runway_elsewhere",
                                        "flying": format!("{} {}", rwy.icao, rwy.id), "cleared": c,
                                        "found": other.ok().flatten().map(|r| format!("{} {}", r.icao, r.id))}),
                                );
                            }
                        }
                    }
                }

                // ROLLOUT TO A STOP. Once on the ground and slow, cut the power
                // and brake to a halt before handing off — the old touchdown
                // path just idled the throttle at 40 kt and said "you have the
                // aircraft", leaving it coasting down the runway with nobody on
                // the brakes (the circuit's landings rolled on unmanaged,
                // 2026-07-23). Latched, so the approach guidance below never
                // re-fires once we're down. The nose is held to the centreline
                // through the plugin steer channel with the takeoff roll law.
                //
                // Commit on GROUNDSPEED *or* airspeed-below-approach: the bare
                // gs < 45 gate is a 172 number, and a heavier airframe landing
                // into a TAILWIND touches down well above it — a Twin Otter
                // planted at 70 kt gs (56 KIAS, 6 kt tailwind on 27) never
                // latched, so nothing steered the centreline OR braked; the
                // approach guidance floated past the aimpoint, flipped
                // Flare->Intercept, and the go-around latch firewalled it off
                // the end into the bay (LXGB, 2026-07-26). Airspeed below the
                // approach speed can identify a committed landing only after
                // weight-on-wheels. Without that gate a normal flare below five
                // feet could latch rollout in the air, cut power, and arm the
                // brakes before touchdown.
                let wheels_t = std::time::Instant::now();
                let wheels_dt = wheels_read_t
                    .map_or(0.0, |t0| wheels_t.duration_since(t0).as_secs_f64());
                wheels_read_t = Some(wheels_t);
                wheels_on = wheels_on_time(wheels_on, on_ground > 0.5, radio_ft, wheels_dt);
                if should_start_rollout(
                    rolling_out,
                    on_ground > 0.5,
                    radio_ft,
                    gs_kts,
                    ias,
                    approach_ias,
                    wheels_on,
                ) {
                    // Consumed once, at rollout entry: a stale flag from a
                    // mission that never landed must not turn a later
                    // standalone landing into a rolling handoff to nobody.
                    if !rolling_out {
                        rolling_out = true;
                        // The arrest's last aileron rides the roll channel's
                        // 1 s heartbeat onto the wheels unless it is
                        // centred: full right at the handover banked a 737
                        // 9.9 degrees on the mains and slid it 6.1 m right
                        // (EGKK 08R 1790687142, 2026-09-29).
                        if arrest_hands {
                            let _ = xp.set_float(FLARE_ROLL, 0.0).await;
                        }
                        // A landing the autopilot flew is its rollout too
                        // (`ap_lands`): disconnected at taxi speed, or
                        // sooner off the line (`rollout_hand_back`).
                        if ap_flies {
                            ap_rollout = true;
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "ap_rollout", "taken": true, "gs": gs_kts,
                                    "radio": radio_ft, "cmd_a": cmd_a_lit, "cmd_b": cmd_b_lit,
                                    "rollout_fma": fma.and_then(|f| f.rollout)}),
                            );
                        } else {
                            disconnect_autopilot(&xp).await;
                        }
                        vacate_follows = VACATE_FOLLOWS.swap(false, Ordering::SeqCst);
                        // Wheels on this runway: the fact the vacate needs.
                        LANDED_RUNWAY.set_in(flight, rwy.id.clone());
                        // A balk the wheels took is a LANDING (LFBD 11,
                        // 2026-09-27): the call waited for a climb that
                        // never came, so the tower still holds our landing
                        // clearance; the intercom's "going around" is
                        // corrected.
                        if runtime.go_around.active() {
                            let along_m = along_track_nm(lat, lon, &rwy) * M_PER_NM;
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "balk_landed",
                                    "why": runtime.balk_watch.and_then(|w| w.why),
                                    "balk_radio": runtime.balk_watch.map(|w| w.balk_radio_ft),
                                    "radio": radio_ft, "gs": gs_kts,
                                    "remaining_m": (rwy.length_m + along_m).max(0.0),
                                    "called": runtime.said_going_around}),
                            );
                            if runtime.spoke_going_around {
                                announce(
                                    "Correction, we're down. Landing, not going around.".into(),
                                    Priority::Caution,
                                );
                            }
                        }
                    }
                    let xtk_nm = cross_track_nm(lat, lon, &rwy);
                    let xtk_m = xtk_nm * M_PER_NM;
                    // The autopilot's rollout, handed back at taxi speed or
                    // off the line (`ap_lands::rollout_hand_back`): from
                    // here our steering law, begun fresh.
                    if let (true, Some(l)) = (ap_rollout, lands.as_ref()) {
                        let ap_in = match (cmd_a_lit, cmd_b_lit) {
                            (Some(true), _) | (_, Some(true)) => Some(true),
                            (Some(false), Some(false)) => Some(false),
                            _ => None,
                        };
                        if let Some(why) = crate::ap_lands::rollout_hand_back(
                            gs_kts,
                            xtk_m,
                            fma.and_then(|f| f.rollout),
                            wheels_on.unwrap_or(0.0),
                            ap_in,
                            l,
                        ) {
                            ap_rollout = false;
                            let off = disconnect_autopilot(&xp).await;
                            roll_law = crate::autotakeoff::RollSteer::default();
                            rollout_steer = crate::aircraft::rollout_steer().map(RolloutSteer::new);
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "ap_rollout", "taken": false, "why": why,
                                    "gs": gs_kts, "xtk_m": xtk_m, "disconnected": off,
                                    "wheels_on_s": wheels_on}),
                            );
                            if let Some(line) = crate::ap_lands::rollout_line(why) {
                                announce(line.into(), Priority::Caution);
                            }
                        }
                    }
                    if let (Some(l), Some(r), Some(f)) = (lands.as_ref(), lands_read.as_ref(), fma.as_ref()) {
                        crate::blackbox::record(
                            "autoland",
                            crate::ap_lands::tick_record(
                                l, r, f, &ap_landing, &at_speed, coupling, cmd_a_lit, radio_ft, ias, None,
                                on_ground > 0.5, ap_rollout,
                            ),
                        );
                    }
                    // The roll's own closure rate and its own tick time. The
                    // approach path below never runs again once we are rolling
                    // out, so `prev_xtk` is ours from here; its last value is
                    // the final's, which is the continuity we want at the
                    // wheels. The steering law used to be handed a HARDCODED
                    // 0.4 s while this loop actually ticks at 300 ms plus two
                    // round trips, so its lead term and its slew limit were
                    // both computed against a clock that did not exist.
                    let t_roll = std::time::Instant::now();
                    let roll_dt = prev_xtk
                        .map(|(_, t0)| t_roll.duration_since(t0).as_secs_f64())
                        .filter(|d| (0.05..=3.0).contains(d))
                        .unwrap_or(0.4);
                    let xtk_rate = rollout_rate.update(
                        prev_xtk
                            .filter(|_| roll_dt > 0.0)
                            .map_or(0.0, |(x0, _)| (xtk_m - x0 * M_PER_NM) / roll_dt),
                        roll_dt,
                    );
                    prev_xtk = Some((xtk_nm, t_roll));
                    let target = crate::autotakeoff::roll_target_heading(
                        rwy.course_deg,
                        xtk_m,
                        xtk_rate,
                        gs_kts,
                    );
                    let psi = psi_r.as_ref().copied().unwrap_or(rwy.course_deg);
                    let err = crate::angles::signed_diff_deg(target, psi);
                    // OFF THE WHEELS after the latch: a bounce (`BounceHold`).
                    // The latch takes the first tick the wheels read on, and
                    // at LEMG 13 (the SR22, 2026-09-29) that tick was a
                    // bounce both times: 1790689843 flew 3.5 to 3.8 ft for
                    // six seconds after it, 1790694457 up to 9 ft for four
                    // and a half, the brakes split and the rudder steering an
                    // aircraft in the air with the roll channel let go.
                    // 1790694457 drifted from 4.1 m to 10.3 m right at 1.1 to
                    // 1.8 m/s and landed with 0.39 of left brake already on,
                    // which slewed the nose from 134 to 127 in a second on a
                    // 131.5 track (the scorer's 3.8 degrees of drift at the
                    // touchdown). In the air: no wheel brake, no rudder, the
                    // arrest's wings closing the track on the centreline.
                    let airborne = on_ground < 0.5;
                    let bounce = bounce_hold.tick(airborne);
                    if matches!(bounce, BounceTick::Left | BounceTick::Landed) {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "bounce",
                                "off": bounce == BounceTick::Left,
                                "spell": bounce_hold.spells(), "radio": radio_ft,
                                "gs": gs_kts, "xtk_m": xtk_m, "psi": psi}),
                        );
                    }
                    if bounce == BounceTick::Landed {
                        // Down again: the split starts from the wheels, never
                        // from what it learnt steering the air.
                        rollout_brakes = RolloutBrakeSteer::for_airframe();
                    }
                    // A declared rollout steer damps on the sim's yaw rate
                    // (`RolloutSteer`, #197): carried by the tick's one read
                    // (`sim::state`, `F::R`), no round of its own.
                    let yaw_r = match rollout_steer.as_ref() {
                        Some(_) if !airborne => st.opt(F::R),
                        _ => None,
                    };
                    let steer = if airborne {
                        if let Some(law) = rollout_steer.as_mut() {
                            law.in_air();
                        }
                        0.0
                    } else if let Some(law) = rollout_steer.as_mut() {
                        law.step(err, roll_dt, gs_kts, yaw_r)
                    } else {
                        roll_law.step(err, roll_dt, gs_kts, 0.0)
                    };
                    let track_roll = hpath_r.as_ref().ok().copied();
                    let side_roll = crate::pilot::line::side_velocity_m_s(track_roll, rwy.course_deg, gs_kts)
                        .unwrap_or(xtk_rate);
                    let phi_roll = phi_r.as_ref().ok().copied();
                    let tas_roll = plant_at(5).map_or(ias, |v| v * 1.943_844);
                    let air_bank_tgt = phi_roll
                        .map(|_| crate::pilot::line::bank_for_line(xtk_m, side_roll, phi_roll.unwrap_or(0.0), radio_ft, line_period_s(ias, tas_roll, roll_dt), 0.0));
                    let air_ail = match (airborne && arrest_hands && !roll_yielded, phi_roll, air_bank_tgt) {
                        (true, Some(phi), Some(tgt)) => {
                            Some(bank_hold.step(phi, tgt, roll_dt, &pilot_airframe.roll, ias, tas_roll, plant_at(8).unwrap_or(0.0)))
                        }
                        _ => None,
                    };
                    // Queued: the rollout's writes go out in one round below.
                    if let Some(a) = air_ail {
                        // Ours, not a hand: unmarked, the scorer read this
                        // bounce's bank as a pilot's yoke (LEMG 13 SR22
                        // 1790697159, yoke_under_ap yr -0.18, 2026-09-29).
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_ROLL, a);
                    } else if bounce == BounceTick::Landed && arrest_hands {
                        // The rollout entry's centring, again at the wheels.
                        writes.set(FLARE_ROLL, 0.0);
                    }
                    // Reverse once the wheels have settled (`reverse::select`,
                    // the pilot's own door): the watcher stows it at taxi pace
                    // or on a bounce, and while it is in the levers are its,
                    // so the idle write below stands aside (#222).
                    if crate::tools::reverse::rollout_reverse_due(
                        rollout_reverse_engines.is_some(),
                        reverse_selected,
                        airborne,
                        wheels_on,
                        gs_kts,
                    ) {
                        reverse_selected = true;
                        let took = crate::tools::reverse::select(
                            &xp,
                            base_url.clone(),
                            rollout_reverse_engines.unwrap_or(1),
                            "autoland",
                        )
                        .await;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "reverse", "took": took,
                                "gs": gs_kts, "wheels_on_s": wheels_on}),
                        );
                        if took == Some(false) {
                            announce("Reverse didn't take.".into(), Priority::Caution);
                        }
                    }
                    let reverse_in = crate::tools::reverse::ACTIVE.load(Ordering::SeqCst);
                    // Brake by wanted deceleration, not a fixed clamp: a
                    // comfortable rate when the runway allows it, harder
                    // only when stopping in what remains demands it; on a
                    // castoring nosewheel, split between the wheels to steer
                    // (`RolloutBrakeSteer`; an unread heading steers nothing).
                    let along_m = along_track_nm(lat, lon, &rwy) * M_PER_NM;
                    let remaining_m = (rwy.length_m + along_m).max(0.0);
                    let target_kts = if vacate_follows { VACATE_HANDOFF_KTS } else { 0.0 };
                    // Braked on the deceleration it measures, lightly, the
                    // drag doing the early work (`DecelBrake`).
                    if airborne {
                        decel_brake = DecelBrake::default();
                    }
                    let brake =
                        decel_brake.step(gs_kts, roll_dt, rollout_required_decel(gs_kts, target_kts, remaining_m));
                    let brakes = if airborne {
                        RolloutBrakes {
                            sym: 0.0,
                            left: 0.0,
                            right: 0.0,
                        }
                    } else if rollout_brake_steer && !ap_rollout {
                        rollout_brakes.step_with(
                            if psi_r.is_ok() { err } else { f64::NAN },
                            psi_r.as_ref().ok().copied(),
                            track_roll,
                            steer,
                            roll_dt,
                            gs_kts,
                            brake,
                        )
                    } else {
                        RolloutBrakes {
                            sym: brake,
                            left: 0.0,
                            right: 0.0,
                        }
                    };
                    // The rollout kept no record of its own: the EGLF and LFPB
                    // weaves had to be reconstructed from the 1 Hz `state`
                    // samples, which is no way to read a control loop.
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({
                            "event": "rollout", "gs": gs_kts, "xtk_m": xtk_m,
                            "xtk_rate": xtk_rate, "err": err, "steer": steer,
                            "trim": roll_law.trim(), "psi": psi, "dt": roll_dt,
                            // The sim's yaw rate a declared rollout steer
                            // damps on (#197); null when not read.
                            "yaw_r": yaw_r,
                            "pos": pos_source,
                            // The brakes as written: the symmetric channel and
                            // each wheel's steering channel (issue #166).
                            "brake": brakes.sym, "bl": brakes.left, "br": brakes.right,
                            // The free nose's learnt standing yaw, deg/s^2
                            // (`ground_steer`; 0 on the shared split), and its
                            // tick: the yaw rate asked and read.
                            "btrim": rollout_brakes.trim(),
                            "gsteer": rollout_brakes.last().map(|s| s.json()),
                            // The wheels and the height: a latch on a bounce
                            // flew six seconds of "rollout" in the air, and
                            // only the 3 s `state` samples said so (#166).
                            "ground": !airborne, "radio": radio_ft,
                            "phi": phi_roll, "trk": track_roll,
                            "air_ail": air_ail,
                            "bank_tgt": airborne.then_some(air_bank_tgt).flatten(),
                            // Reverse in (#222): the idle write stood aside.
                            "rev": reverse_in,
                        }),
                    );
                    if !reverse_in {
                        writes.set(&thr_name, 0.0);
                    }
                    // The autopilot's ROLLOUT steers (`ap_lands`): ours
                    // stands aside until it is handed back.
                    if !ap_rollout {
                        writes.set(TAXI_STEER, steer);
                        if let Some(c) = rollout_tiller.command(steer, gs_kts) {
                            writes.set(crate::tiller::TILLER, c);
                        }
                    }
                    // A vacate follows: hand off ROLLING at taxi pace — the
                    // completion of a landing is clearing the runway, and a
                    // dead stop on the strip followed by a re-acceleration was
                    // neither procedure nor comfortable (EGHI, 2026-08-02).
                    // The vacate drive arms the governor from the current
                    // position, which takes a rolling aircraft like any join.
                    if vacate_follows && gs_kts <= VACATE_HANDOFF_KTS {
                        if ap_rollout {
                            disconnect_autopilot(&xp).await;
                        }
                        // Stowed by now (25 kt); never handed to the drive in.
                        crate::tools::reverse::stow();
                        writes.flush(&xp).await;
                        let _ = xp.set_float(TAXI_BRAKE, 0.0).await;
                        let _ = xp.set_float(TAXI_STEER, 0.0).await;
                        rollout_tiller.release(&xp).await;
                        if rollout_brake_steer {
                            let _ = tokio::join!(
                                xp.set_float(crate::autotaxi::LEFT_BRAKE, 0.0),
                                xp.set_float(crate::autotaxi::RIGHT_BRAKE, 0.0),
                            );
                        }
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({ "event": "rolling_handoff", "gs": gs_kts }),
                        );
                        break;
                    }
                    if gs_kts > 2.0 {
                        // The steering pair every tick with the stop: the
                        // plugin's channels are one-second heartbeats.
                        // With the lever, the rudder and the column: the
                        // rollout's ONE write round (it was two or three,
                        // 0.48 s a tick against a 300 ms sleep, 2026-09-30).
                        writes.set(TAXI_BRAKE, brakes.sym);
                        if rollout_brake_steer && !ap_rollout {
                            writes
                                .set(crate::autotaxi::LEFT_BRAKE, brakes.left)
                                .set(crate::autotaxi::RIGHT_BRAKE, brakes.right);
                        }
                        writes.flush(&xp).await;
                        set_panel(&format!("AUTOLAND {} {} ROLLOUT", rwy.icao, rwy.id));
                        pacer.wait(std::time::Duration::from_millis(ROLLOUT_TICK_MS)).await;
                        continue;
                    }
                    // Stopped: squeeze the brakes, free the nosewheel, then
                    // hand the hold to the PARKING brake through the
                    // valve-aware seam, and say what the read-back says. The
                    // squeeze is the plugin's one-second heartbeat: on its own
                    // it let go the moment this loop ended, and at EGSS the
                    // 737 sat on the runway after "Brakes on" with
                    // `parking_brake_ratio` at 0.03 (2026-09-22).
                    crate::tools::reverse::stow();
                    writes.flush(&xp).await;
                    let _ = xp.set_float(TAXI_BRAKE, ROLLOUT_BRAKE_HOLD).await;
                    let _ = xp.set_float(TAXI_STEER, 0.0).await;
                    rollout_tiller.release(&xp).await;
                    if rollout_brake_steer {
                        let _ = tokio::join!(
                            xp.set_float(crate::autotaxi::LEFT_BRAKE, 0.0),
                            xp.set_float(crate::autotaxi::RIGHT_BRAKE, 0.0),
                        );
                    }
                    // The one stop-and-park (`hand_back_hold`): to under a
                    // knot on the wheel brakes, the held command's tail, then
                    // the parking brake read back. It set the parking brake
                    // at 2 kt with no tail wait (refactor audit 2026-10-04,
                    // duplicates #8). Fail closed: we are on the runway.
                    if ap_rollout {
                        disconnect_autopilot(&xp).await;
                    }
                    let valve = crate::aircraft::live_profile().and_then(|p| p.park_brake_valve_set);
                    let held = crate::tools::controls::hand_back_hold(
                        &xp,
                        valve,
                        crate::tools::controls::HoldFor::by("autoland").fail_closed(),
                    )
                    .await
                    .unwrap_or(false);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({
                            "event": "stopped_brakes", "held": held, "valve_set": valve,
                        }),
                    );
                    announce(stopped_line(held).into(), Priority::Warning);
                    break;
                }
                // Cross-track closure rate (nm/min, positive = drifting right),
                // low-pass filtered — the damping term that turns the P-only
                // centreline chase into a crisp intercept-and-rollout. P alone
                // converged asymptotically (the banana join) and S-ed under
                // wind (2026-07-24).
                let xtk_now = cross_track_nm(lat, lon, &rwy);
                let t_now = std::time::Instant::now();
                let dt_s = prev_xtk.map_or(0.0, |(_, t0)| t_now.duration_since(t0).as_secs_f64());
                let raw_rate = match prev_xtk {
                    Some((x0, _)) if (0.1..=3.0).contains(&dt_s) => (xtk_now - x0) / dt_s * 60.0,
                    _ => 0.0,
                };
                prev_xtk = Some((xtk_now, t_now));
                // Low-passed on TIME (`XTK_RATE_TAU_S`), not half per tick.
                let a = 1.0 - (-dt_s.clamp(0.05, 3.0) / XTK_RATE_TAU_S).exp();
                xtk_rate += (raw_rate.clamp(-3.0, 3.0) - xtk_rate) * a;

                // Wind crab, from the reference recipe (heading vs ground track):
                // command a heading whose TRACK aligns with the runway, not the
                // heading itself. P-only tracking left a standing 0.10 nm
                // cross-track under the LXGB crosswind — touchdown in the grass
                // (2026-07-22). LOW-PASSED: the raw read fed straight back to
                // the bug hunted the LEMG 13 final into a balk (2026-09-12,
                // see [`CrabFilter`]). Estimated every tick, applied on final.
                let crab_raw = CrabFilter::raw(psi_r.as_ref().ok().copied(), hpath_r.as_ref().ok().copied());
                let crab = crab_filter.update(crab_raw, dt_s.clamp(0.0, 3.0));

                let raw = guidance_on(
                    lat, lon, alt, radio_ft, gs_kts, ias, &rwy, approach_ias, 25.0, xtk_rate,
                    runway_width_m(&rwy),
                );
                let raw = hold_flare(raw, flaring, radio_ft, gs_kts, 25.0);
                // A missed approach called low is flown by the latch (full power,
                // straight ahead) until clear of the ground — the raw intercept
                // guidance turns back to the FAP immediately, which is a killer
                // at 50 ft (see [`GoAroundLatch`]).
                let along_nm = along_track_nm(lat, lon, &rwy);
                // A roundout begun keeps the arrest (`hold_roundout`): a
                // roundout balloon over the band is still the landing, or a
                // balked one, never the autopilot's descent again.
                let held = hold_roundout(
                    raw,
                    roundout_taken,
                    radio_ft,
                    gs_kts,
                    rwy.length_m + along_nm.min(0.0) * M_PER_NM,
                    crate::aircraft::landing_distance_m(),
                );
                let held_now = held.arresting && !raw.arresting;
                let balked_now = held.balk.is_some() && raw.balk.is_none();
                if held_now != roundout_held || balked_now {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "roundout_held", "held": held_now,
                            "balk": held.balk, "radio": radio_ft,
                            "band": arrest_gates_ft(25.0, gs_kts).1}),
                    );
                }
                roundout_held = held_now;
                let raw = held;
                // THE GO/NO-GO, once, at the roundout gate, on an airframe
                // that declares its landing roll (`landing_call`): the
                // touchdown the energy predicts leaves the room to stop, and
                // the landing is committed; or it does not, and the
                // go-around is flown from here with height in hand.
                let roll_m = crate::aircraft::landing_roll_m();
                let book_m = crate::aircraft::landing_distance_m();
                if landing_called.is_none()
                    && roll_m.is_some()
                    && raw.arresting
                    && raw.phase == ApproachPhase::Final
                    && raw.balk.is_none()
                    && !runtime.go_around.active()
                    && on_ground < 0.5
                {
                    let call = landing_call(
                        along_nm * M_PER_NM,
                        rwy.length_m,
                        radio_ft,
                        gs_kts,
                        Some(ias),
                        approach_ias,
                        wheels_need_m(book_m, roll_m),
                    );
                    if let Some(call) = call {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "landing_call", "land": call.land,
                                "touchdown_m": call.touchdown_m, "remaining_m": call.remaining_m,
                                "need_m": call.need_m, "air_s": call.air_s, "radio": radio_ft,
                                "ias": ias, "gs": gs_kts, "length_m": rwy.length_m}),
                        );
                        if !call.land && runtime.go_around.order(false) {
                            announce(
                                "Long landing. Going around.".to_string(),
                                Priority::Caution,
                            );
                        }
                        landing_called = Some(call);
                    }
                }
                let committed_need = (flare_committed || landing_called.is_some_and(|c| c.land))
                    .then(|| wheels_need_m(book_m, roll_m));
                let committed = commit_flare_latched(
                    raw,
                    flaring,
                    committed_need,
                    radio_ft,
                    gs_kts,
                    25.0,
                    rwy.length_m + along_nm.min(0.0) * M_PER_NM,
                    crate::aircraft::landing_distance_m(),
                    rwy.course_deg,
                );
                let committed_now = committed.phase != raw.phase;
                if committed_now && !flare_committed {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "committed_flare", "radio": radio_ft,
                                           "past_m": (-along_nm * M_PER_NM).max(0.0),
                                           "remaining_m": rwy.length_m + along_nm.min(0.0) * M_PER_NM}),
                    );
                }
                // A commitment the room no longer covers is a go-around.
                if committed_need.is_some()
                    && matches!(raw.balk, Some("overflew_the_runway" | "balloon"))
                    && !committed_now
                {
                    let remaining_m = rwy.length_m + along_nm.min(0.0) * M_PER_NM;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "commit_released", "why": raw.balk,
                                           "radio": radio_ft, "remaining_m": remaining_m,
                                           "need_m": committed_need}),
                    );
                }
                let (raw, refused) =
                    wheels_hold_the_landing(committed, on_ground > 0.5, flaring, rwy.course_deg);
                if let Some(why) = refused {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "balk_refused_on_ground", "why": why,
                                           "radio": radio_ft, "ias": ias, "gs": gs_kts}),
                    );
                }
                // A controller's order beats anything the approach thinks of
                // itself. Watched here because this is the thing that can ACT
                // on it; the watch is armed with the approach above, so only
                // an order given during this approach can trip it.
                // The pilot's order is flown the same way and answered, so
                // the `go_around` tool reads back what the aircraft does.
                if let Some(by) = runtime.orders.take() {
                    let already = runtime.go_around.active();
                    let flown = runtime.go_around.order(on_ground > 0.5);
                    if flown {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "ordered_go_around", "radio": radio_ft,
                                               "dist": raw.dist_nm, "by": by.label()}),
                        );
                    }
                    // Answered for the tower's too: a pilot's order the
                    // same tick is folded into it.
                    use crate::go_around::Answer;
                    crate::go_around::answer(if flown {
                        Answer::Flying
                    } else if already {
                        Answer::Already
                    } else {
                        Answer::OnGround
                    });
                }
                // A final the tower is working but has not cleared: ask
                // again at 1,000 ft, go around at 300 (`UnclearedWatch`).
                // The tower's hold is judged by the monitor on its own; this
                // is the crew never landing uncleared (#155).
                let book = crate::party_line::clearance_state();
                match runtime.uncleared.step(
                    book == Some(crate::tower::State::Inbound),
                    book == Some(crate::tower::State::TakeoffCleared),
                    radio_ft,
                    on_ground > 0.5,
                    runtime.go_around.active(),
                    super::established_on_final(),
                ) {
                    Some(UnclearedStep::Ask) => {
                        if runtime.going_around_callsign.is_none() {
                            runtime.going_around_callsign = Some(crate::crew_radio::crew_callsign(&xp).await);
                        }
                        let cs = runtime.going_around_callsign.clone().flatten();
                        crate::crew_radio::transmit(crate::crew_radio::radio_call(
                            &cs,
                            &format!(
                                "final runway {}, request landing clearance",
                                crate::crew_radio::spell_runway(&rwy.id)
                            ),
                        ));
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "landing_clearance_asked", "rwy": rwy.id,
                                               "radio": radio_ft, "dist": raw.dist_nm,
                                               "held": crate::party_line::landing_hold_armed()}),
                        );
                    }
                    Some(UnclearedStep::GoAround) => {
                        if runtime.go_around.order(on_ground > 0.5) {
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "uncleared_go_around", "rwy": rwy.id,
                                                   "radio": radio_ft, "dist": raw.dist_nm,
                                                   "held": crate::party_line::landing_hold_armed()}),
                            );
                            announce(
                                "No landing clearance. Going around.".into(),
                                Priority::Caution,
                            );
                        }
                    }
                    None => {}
                }
                // Unstabilised at the thousand-foot gate is a go-around, not a
                // quiet re-intercept: the latch only arms itself below the
                // terrain floor, so it is ordered.
                // So is one found unstabilised below the floor on the FIRST
                // tick (a handoff that arrives that way, LEMG 13 2026-09-28):
                // with no final before it, the latch's own transition never
                // sees it.
                if matches!(raw.balk, Some("unstabilised_at_gate" | "unstabilised_below_floor")) {
                    runtime.go_around.order(on_ground > 0.5);
                }
                // THE STABILISED APPROACH (`stable_approach`, #199): by the
                // gate the landing configuration, on speed, not sinking hard
                // and on the path, or a go-around THEN, with height in hand.
                // The lateral half is the guidance's own gate above. EGNX 09
                // (2026-09-30): 255 kt clean and 1,400 fpm down from 1,000 ft
                // to 121 ft radio, on the centreline, so nothing balked.
                let height_aal = alt - rwy.elev_ft;
                let judging = raw.phase == ApproachPhase::Final
                    && !raw.arresting
                    && !runtime.go_around.active()
                    && on_ground < 0.5
                    && along_nm > STABILISED_MIN_ALONG_NM
                    && crate::stable_approach::judged(gate, height_aal);
                let flap_limits_now = crate::aircraft::flap_limits();
                let stable_sample = crate::stable_approach::Sample {
                    height_aal_ft: height_aal,
                    dist_to_aim_nm: along_nm.max(0.0) + TOUCHDOWN_AIM_NM,
                    ias_kts: ias,
                    gs_kts,
                    // What the governor holds in the landing configuration.
                    target_kts: approach_target_kts(
                        approach_ias,
                        1.0,
                        &flap_limits_now,
                        crate::aircraft::config_floor_kts(1.0),
                    ),
                    vvi_fpm: prev_vvi.map(|(v, _)| v),
                    above_path_ft: alt - raw.glidepath_alt_ft,
                    path_ft_per_nm: gp_ft_per_nm(),
                    gear_down,
                    landing_flap: last_flap.is_some_and(|f| f >= LANDING_FLAP_RATIO),
                };
                let now_breaches = if judging {
                    crate::stable_approach::breaches(gate, &stable_sample)
                } else {
                    Vec::new()
                };
                if judging && !runtime.gate_recorded {
                    runtime.gate_recorded = true;
                    crate::blackbox::record(
                        "autoland",
                        stable_record("stable_gate", gate, &stable_sample, &now_breaches, None),
                    );
                }
                if let Some(breach) =
                    runtime.stable_watch.step(runtime.stable_clock.elapsed().as_secs_f64(), &now_breaches)
                {
                    runtime.stable_watch.reset();
                    if runtime.go_around.order_over_gate(on_ground > 0.5, gate.gate_ft()) {
                        runtime.stable_balk = Some(breach.word());
                        runtime.stable_said = Some(breach.spoken());
                        crate::blackbox::record(
                            "autoland",
                            stable_record(
                                "unstabilised",
                                gate,
                                &stable_sample,
                                &now_breaches,
                                Some(breach),
                            ),
                        );
                    }
                }
                if runtime.go_around.active() {
                    runtime.stable_watch.reset();
                    runtime.gate_recorded = false;
                }
                runtime.go_around.note_height_aal(alt - rwy.elev_ft);
                let mut missed = runtime.go_around.update(
                    last_phase,
                    raw.phase,
                    radio_ft,
                    raw.dist_nm,
                    along_nm,
                    join_point_nm(alt, rwy.elev_ft),
                    xtk_now,
                );
                flaring = raw.phase == ApproachPhase::Flare && missed.is_none();
                if missed.is_some() {
                    landing_called = None;
                }
                // The latch lives as long as the flare it committed.
                flare_committed = flaring && (flare_committed || committed_now);
                // THE COURSE REVERSAL. Started by the missed approach's
                // reposition stage, or by an ordinary approach that finds
                // itself steering at a centreline point it would reach flying
                // away from the runway (or called "final" while doing so).
                let track_true = hpath_r
                    .as_ref()
                    .ok()
                    .or(psi_r.as_ref().ok())
                    .copied()
                    .unwrap_or(rwy.course_deg);
                // Set on the tick a pattern begins: its own line says what
                // happens, so the phase change says nothing more.
                let mut reversal_began = false;
                // A refused pattern is not asked for again: its hold flies.
                if reversal.is_none() && !hold.as_ref().is_some_and(|h| h.refused) {
                    // The guidance's own aim: a join point behind only when
                    // the height cannot come off on the way in (#131).
                    let aim_along_nm = if aims_at_join_point(
                        alt - raw.glidepath_alt_ft,
                        along_nm,
                        xtk_now,
                        gs_kts,
                    ) {
                        join_point_nm(alt, rwy.elev_ft)
                    } else {
                        FAP_NM
                    };
                    // A steep, visual-segment approach (LSZS) is never
                    // repositioned by a pattern of our own: that is a valley.
                    // A go-around there flies the PUBLISHED missed approach.
                    // PUBLISHED steep only: a circuit's path steepened over
                    // the ground (#182) is an ordinary field, its reversal
                    // terrain-probed (LEMG 13 SR22 2026-09-29: 3.26 degrees
                    // took the valley branch, no pattern when high on the
                    // join, then no missed approach to fly after the balk).
                    let steep = published_steep() && !missed_procedure_tried;
                    let wanted = match missed {
                        Some(MissedStage::Reposition) => true,
                        Some(MissedStage::Climb) => false,
                        None if steep => false,
                        None => {
                            let needed = raw.phase != ApproachPhase::Flare
                                && radio_ft >= TERRAIN_FLOOR_AGL_FT
                                && crate::reversal::needed(
                                    &rwy,
                                    lat,
                                    lon,
                                    track_true,
                                    aim_along_nm,
                                    raw.phase == ApproachPhase::Final,
                                );
                            // Inside the FAP and pointing down the runway, a
                            // pattern is not the answer to "too high": a crew
                            // GOES AROUND, straight ahead, and the latch's own
                            // reposition follows the climb. Armed at 2.6 nm
                            // 800 ft high, the 737 turned straight onto a
                            // downwind from 1,600 ft at LEMG 13 (2026-09-26).
                            if needed
                                && late_go_around(along_nm, crate::angles::abs_diff_deg(track_true, rwy.course_deg))
                                && runtime.go_around.order(on_ground > 0.5)
                            {
                                crate::blackbox::record(
                                    "autoland",
                                    serde_json::json!({"event": "late_go_around", "dist": raw.dist_nm,
                                        "radio": radio_ft, "above_path_ft": alt - raw.glidepath_alt_ft}),
                                );
                                announce(
                                    "Too high to land from here. Going around.".to_string(),
                                    Priority::Caution,
                                );
                                false
                            } else {
                                needed
                            }
                        }
                    };
                    if wanted && steep {
                        missed_procedure_tried = true;
                        let missed_legs = crate::procedure::take_missed_legs();
                        let vy = crate::aircraft::live_profile()
                            .and_then(|p| p.reference.as_ref().and_then(|r| r.vy_kts))
                            .unwrap_or(approach_ias * 1.3);
                        let flying = match missed_legs {
                            Some(legs) => crate::procedure::start_scoped(
                                base_url.clone(),
                                voice.clone(),
                                crate::procedure::Plan {
                                    name: "the missed approach".to_string(),
                                    legs,
                                    end: crate::procedure::EndAction::Level,
                                    vy_kts: vy,
                                    cruise_ias_kts: None,
                                    transition_altitude_ft: None,
                                    climb_minimums: Vec::new(),
                                    arrival_constraints: Vec::new(),
                                    visual_deg: None,
                                    missed_legs: Vec::new(),
                                    cruise_schedule: Default::default(),
                                    arrival_handoff_kts: None,
                                },
                                Vec::new(),
                            ),
                            None => false,
                        };
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "missed_procedure", "flying": flying, "radio": radio_ft}),
                        );
                        if flying {
                            gave_up = true;
                            announce(
                                "Flying the published missed approach.".to_string(),
                                Priority::Caution,
                            );
                            break;
                        }
                        // No published missed approach to fly: our own
                        // terrain-probed pattern below (`reversal::plan`,
                        // which climbs straight ahead and HOLDS a height
                        // clear of the ground where no pattern fits). The
                        // old "climbing straight ahead" hand-back left
                        // an unattended SR22 at 805 ft with nobody flying
                        // (LEMG 13 2026-09-29).
                    }
                    if wanted && reversals_flown >= crate::reversal::MAX_REVERSALS {
                        gave_up = true;
                        let thr_dref = throttle_dataref();
                        let _ = tokio::join!(
                            xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                            autoflight::set_dial(&xp, Dial::Vs, 0.0),
                        );
                        announce(
                            format!(
                                "I can't get us established for runway {}. Your aircraft. \
                                 The autopilot is holding heading and altitude{}.",
                                crate::crew_radio::spell_runway(&rwy.id),
                                if gear_down { ", gear is down" } else { "" }
                            ),
                            Priority::Warning,
                        );
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "give_up", "reason": "max_reversals",
                                               "flown": reversals_flown}),
                        );
                        break;
                    }
                    // The pattern is planned over the probed ground (#182,
                    // `reversal::plan`), beside the loop: begun in the
                    // go-around's climb (the reposition will want it), or the
                    // tick it is wanted; a plan over a minute old is stale.
                    let default_platform = rwy.elev_ft + REPOSITION_MIN_AGL_FT;
                    if reversal_plan
                        .as_ref()
                        .is_some_and(|(_, t)| t.elapsed().as_secs_f64() > REVERSAL_PLAN_FRESH_S)
                    {
                        reversal_plan = None;
                    }
                    if (wanted || missed == Some(MissedStage::Climb)) && reversal_plan.is_none() {
                        let task = crate::reversal::plan(
                            XPlaneClient::new(base_url.clone()).with_parents(xp.cancellation()),
                            rwy.clone(),
                            along_nm,
                            xtk_now,
                            gs_kts,
                            default_platform,
                            (lat, lon, track_true, alt),
                        );
                        reversal_plan = Some((tokio::spawn(task), std::time::Instant::now()));
                    }
                    // Waiting on the plan: straight ahead, never lower than
                    // the platform the old rule would have flown.
                    if wanted && hold.is_none() {
                        hold = Some(TerrainHold::waiting(track_true, alt.max(default_platform)));
                    }
                    if !wanted && hold.as_ref().is_some_and(|h| !h.refused) {
                        hold = None;
                    }
                    let ready = wanted
                        && reversal_plan.as_ref().is_some_and(|(h, _)| h.is_finished());
                    let planned = if ready {
                        match reversal_plan.take() {
                            Some((h, _)) => h.await.ok(),
                            None => None,
                        }
                    } else {
                        None
                    };
                    if let Some(plan) = planned {
                        let above_path_ft = alt - raw.glidepath_alt_ft;
                        let mut record = serde_json::json!({"event": "reversal", "side": plan.side,
                            "platform_ft": plan.platform_ft,
                            "xtk_m": xtk_now * M_PER_NM, "dist": raw.dist_nm,
                            "track": track_true, "missed": missed.is_some(),
                            "final_called": raw.phase == ApproachPhase::Final,
                            "why": crate::reversal::why(
                                missed.is_some(),
                                raw.phase == ApproachPhase::Final,
                                above_path_ft,
                            ),
                            "above_path_ft": above_path_ft, "aim_nm": aim_along_nm,
                            "along": along_nm, "ias": ias, "gs": gs_kts});
                        if let (Some(r), Some(p)) = (record.as_object_mut(), plan.record.as_object()) {
                            r.extend(p.clone());
                        }
                        crate::blackbox::record("autoland", record);
                        match (plan.refused.as_deref(), plan.hold_ft) {
                            // No pattern here clears the ground: climb straight
                            // ahead to a height clear of it and hold it on the
                            // autopilot, said with the numbers; the aircraft is
                            // handed over only once it is there.
                            (Some(why), hold_ft) => {
                                let to = hold_ft.unwrap_or_else(|| alt.max(default_platform));
                                hold = Some(TerrainHold::refused(track_true, to));
                                announce(
                                    format!(
                                        "No pattern for runway {} clears the ground: {why}. \
                                         Climbing straight ahead to {:.0} feet and holding it.",
                                        crate::crew_radio::spell_runway(&rwy.id),
                                        to
                                    ),
                                    Priority::Warning,
                                );
                            }
                            (None, _) => {
                                hold = None;
                                reversals_flown += 1;
                                reversal = Some(crate::reversal::Reversal {
                                    side: plan.side,
                                    leg: crate::reversal::Leg::Outbound,
                                    platform_ft: plan.platform_ft,
                                    high: false,
                                });
                                if missed == Some(MissedStage::Reposition) {
                                    runtime.go_around.hold_reposition();
                                }
                                reversal_began = true;
                                pattern_gs_kts = gs_kts;
                                reversal_limit = plan.limit.clone();
                                hold_back = false;
                                limit_said = false;
                                if matches!(
                                    plan.why,
                                    "switched_side_for_terrain" | "raised_for_terrain"
                                ) {
                                    crate::blackbox::record(
                                        "autoland",
                                        serde_json::json!({"event": "reversal_terrain",
                                            "why": plan.why, "platform_ft": plan.platform_ft}),
                                    );
                                }
                                if let Some(name) = plan.airspace.as_deref() {
                                    crate::blackbox::record(
                                        "autoland",
                                        serde_json::json!({"event": "reversal_airspace",
                                            "why": plan.why, "airspace": name,
                                            "platform_ft": plan.platform_ft}),
                                    );
                                }
                                announce(
                                    crate::reversal::reposition_line(
                                        &crate::crew_radio::spell_runway(&rwy.id),
                                        plan.side,
                                        plan.platform_ft,
                                        plan.why,
                                        plan.platform_ft > default_platform + 50.0,
                                        plan.airspace.as_deref(),
                                    ),
                                    Priority::Advisory,
                                );
                            }
                        }
                    }
                }
                // Fly it: the pattern owns the heading, the platform and the
                // speed until the aircraft is inbound and the ordinary
                // guidance calls a final it is actually pointing down.
                let mut pattern: Option<(f64, crate::reversal::Leg, f64)> = None;
                // An inbound leg that went over the field is over: left
                // alone it flies the runway heading away for ever.
                if reversal.as_ref().is_some_and(|r| crate::reversal::overflown(r, along_nm)) {
                    reversal = None;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "reversal_overflown", "alt": alt,
                            "radio": radio_ft, "dist": raw.dist_nm, "flown": reversals_flown}),
                    );
                }
                // An inbound leg the glidepath has come down to is an
                // approach: the pattern is done, the path takes the descent.
                if reversal.as_ref().is_some_and(|r| {
                    crate::reversal::path_joined(
                        r,
                        along_nm,
                        xtk_now,
                        track_true,
                        rwy.course_deg,
                        alt - raw.glidepath_alt_ft,
                    )
                }) {
                    reversal = None;
                    runtime.go_around.finish_reposition();
                    missed = None;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "reversal_done", "why": "path_joined",
                            "dist": raw.dist_nm, "xtk_m": xtk_now * M_PER_NM,
                            "above_path_ft": alt - raw.glidepath_alt_ft}),
                    );
                }
                if let Some(rev) = reversal.as_mut() {
                    recede_ref_nm = recede_ref_nm.max(raw.dist_nm);
                    let join = join_point_nm(rev.platform_ft, rwy.elev_ft);
                    // Height first: a downwind begun far above the platform
                    // extends until the rest of the pattern can lose it.
                    let too_high = rev.leg == crate::reversal::Leg::Outbound
                        && crate::reversal::too_high_to_turn(
                            alt,
                            rev.platform_ft,
                            along_nm,
                            join,
                            gs_kts,
                            crate::procedure::descent_fpm(),
                        );
                    // ...but never INTO no-go airspace: short of it the height
                    // is lost in a hold, else the downwind turns in high.
                    let was_back = hold_back;
                    let lim = crate::reversal::limit_step(
                        too_high,
                        along_nm,
                        reversal_limit.as_ref(),
                        &mut hold_back,
                    );
                    let high = lim.high;
                    if high && !rev.high {
                        let above = ((alt - rev.platform_ft) / 100.0).round() * 100.0;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "downwind_extended", "above_ft": above,
                                "along_nm": along_nm, "alt": alt,
                                "limit_nm": reversal_limit.as_ref().map(|l| l.extend_to_nm),
                                "airspace": reversal_limit.as_ref().map(|l| &l.name),
                                "hold": reversal_limit.as_ref().map(|l| l.hold)}),
                        );
                        announce(
                            crate::reversal::extend_line(above, reversal_limit.as_ref()),
                            Priority::Advisory,
                        );
                    }
                    if lim.back && !was_back {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "reversal_hold", "along_nm": along_nm,
                                "alt": alt, "above_ft": alt - rev.platform_ft,
                                "limit_nm": reversal_limit.as_ref().map(|l| l.extend_to_nm)}),
                        );
                    }
                    if lim.capped && !limit_said {
                        limit_said = true;
                        let above = ((alt - rev.platform_ft) / 100.0).round() * 100.0;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "extension_capped", "along_nm": along_nm,
                                "alt": alt, "above_ft": above,
                                "airspace": reversal_limit.as_ref().map(|l| &l.name)}),
                        );
                        if let Some(l) = reversal_limit.as_ref() {
                            announce(
                                crate::reversal::capped_line(above, &l.name),
                                Priority::Advisory,
                            );
                        }
                    }
                    rev.high = high;
                    let stepped = crate::reversal::step(
                        rev,
                        &rwy,
                        lat,
                        lon,
                        along_nm,
                        xtk_now,
                        track_true,
                        gs_kts,
                        join,
                        raw.phase == ApproachPhase::Final,
                    );
                    // The hold's way back toward the field, outside the downwind.
                    let stepped = if lim.back {
                        Some(crate::reversal::hold_heading(
                            &rwy, rev.side, lat, lon, along_nm, gs_kts,
                        ))
                    } else {
                        stepped
                    };
                    match stepped {
                        Some(h) => pattern = Some((h, rev.leg, rev.platform_ft)),
                        None => {
                            reversal = None;
                            runtime.go_around.finish_reposition();
                            missed = None;
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "reversal_done",
                                    "dist": raw.dist_nm, "xtk_m": xtk_now * M_PER_NM}),
                            );
                        }
                    }
                }
                let ga = missed.is_some();
                // The tower is owed this the moment it happens: it is holding a
                // landing clearance for an aircraft that is no longer landing,
                // and it works the traffic underneath us. Luton 2026-07-28: the
                // autoland balked, climbed away and repositioned over the field
                // for four minutes, and nothing was ever said — the FSM sat in
                // ClearedToLand throughout. Announced ONCE, once the go-around
                // is real: a balk low in the flare can still land (LFBD 11,
                // 2026-09-27), and then it is a landing, cleared, and nothing
                // goes out (see [`go_around_call_due`], `balk_landed`).
                if missed.is_none() {
                    runtime.balk_watch = None;
                } else if runtime.balk_watch.is_none() {
                    // The stabilised gate's word when it ordered this one.
                    runtime.balk_watch = Some(BalkWatch::new(raw.balk.or(runtime.stable_balk.take()), radio_ft));
                }
                if let Some(w) = runtime.balk_watch.as_mut() {
                    w.note(radio_ft, on_ground > 0.5);
                }
                if go_around_call_due(
                    missed.is_some(),
                    runtime.said_going_around,
                    on_ground > 0.5,
                    runtime.balk_watch.as_ref(),
                    radio_ft,
                    prev_vvi.map(|(v, _)| v),
                ) {
                    runtime.said_going_around = true;
                    // The go-around VOIDS the landing clearance, so this
                    // approach owes the tower a fresh final report before it
                    // can be cleared again — the call that earns it.
                    runtime.owe_final_report = true;
                    if runtime.going_around_callsign.is_none() {
                        runtime.going_around_callsign = Some(crate::crew_radio::crew_callsign(&xp).await);
                    }
                    let cs = runtime.going_around_callsign.clone().flatten();
                    crate::crew_radio::transmit(crate::crew_radio::radio_call(&cs, "going around"));
                    // `radio` and `why` are the BALK's (the debrief's
                    // "GO-AROUND at N ft radio: why"); `called_radio` is
                    // where the climb made it real.
                    let w = runtime.balk_watch.unwrap_or(BalkWatch::new(raw.balk, radio_ft));
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "going_around", "radio": w.balk_radio_ft,
                                           "why": w.why, "called_radio": radio_ft,
                                           "low_ft": w.low_ft, "touched": w.touched,
                                           "xtk_m": xtk_now * M_PER_NM,
                                           "dist": raw.dist_nm}),
                    );
                }
                // Back on final after a missed approach: report it. The first
                // approach is reported by whoever set this one up (the
                // circuit's own "final runway ...", or full auto's arrival);
                // the second is nobody's until here, and a tower that never
                // hears from us again holds nothing to clear us with. Said on
                // the way IN — established on final, inside the FAP — because
                // "final" is a position report and it has to be true.
                if runtime.owe_final_report
                    && missed.is_none()
                    && raw.phase == ApproachPhase::Final
                    && raw.dist_nm <= FAP_NM
                {
                    runtime.owe_final_report = false;
                    if runtime.going_around_callsign.is_none() {
                        runtime.going_around_callsign = Some(crate::crew_radio::crew_callsign(&xp).await);
                    }
                    let cs = runtime.going_around_callsign.clone().flatten();
                    crate::crew_radio::transmit(crate::crew_radio::radio_call(
                        &cs,
                        &format!("final runway {}", crate::crew_radio::spell_runway(&rwy.id)),
                    ));
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "final_report", "rwy": rwy.id,
                                           "dist": raw.dist_nm}),
                    );
                }
                // Cleared to try again: the next balked landing is a new call.
                if missed.is_none() {
                    runtime.said_going_around = false;
                    runtime.spoke_going_around = false;
                }
                // A missed approach (or a pattern) that stops getting anywhere
                // is its own emergency, judged by the ground it makes, never a
                // flat clock (`reversal::MissedWatch`): the stage's measure
                // (feet to climb, the pattern's track to go, an extended
                // downwind's height) stepping at a rate for the airframe's
                // speed, legs sequenced, inside an outer bound from the plan.
                // The flat 600 s gave up on an SR22 on the line, 90 ft under
                // the path it was about to join, on LEMG 13's terrain-raised
                // pattern (iMac 2026-09-30, sortie 1790775809-approach).
                let watched = if reversal.is_some() || hold.is_some() || missed.is_some() {
                    let progress = match (reversal.as_ref(), hold.as_ref()) {
                        (Some(rev), _) => {
                            let join = join_point_nm(rev.platform_ft, rwy.elev_ft);
                            let end = crate::reversal::pattern_end_nm(
                                rev.platform_ft,
                                rwy.elev_ft,
                                gp_ft_per_nm(),
                                join,
                            );
                            let to_go = crate::reversal::to_go_nm(
                                rev,
                                along_nm,
                                xtk_now,
                                pattern_gs_kts,
                                join,
                                end,
                            );
                            let above = alt - rev.platform_ft;
                            crate::reversal::Progress::Pattern {
                                n: reversals_flown,
                                leg: rev.leg,
                                high: rev.high,
                                to_go_nm: to_go,
                                above_ft: above,
                                budget_s: crate::reversal::pattern_budget_s(
                                    to_go,
                                    pattern_gs_kts,
                                    above,
                                    crate::procedure::descent_fpm(),
                                ),
                                gs_kts: pattern_gs_kts,
                            }
                        }
                        (None, Some(h)) if h.refused => {
                            crate::reversal::Progress::Hold { to_go_ft: h.alt_ft - alt }
                        }
                        _ if missed == Some(MissedStage::Climb) => crate::reversal::Progress::Climb {
                            to_go_ft: GO_AROUND_TURN_AGL_FT - radio_ft,
                        },
                        _ => crate::reversal::Progress::Waiting,
                    };
                    let t = watch_epoch.elapsed().as_secs_f64();
                    let w = missed_watch.get_or_insert_with(|| crate::reversal::MissedWatch::start(t));
                    if let Some(stage) = w.note(t, progress) {
                        let (to_go_nm, leg) = match progress {
                            crate::reversal::Progress::Pattern { to_go_nm, leg, .. } => {
                                (Some(to_go_nm), Some(leg.word()))
                            }
                            _ => (None, None),
                        };
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "missed_progress", "stage": stage,
                                "missed": missed.is_some(), "to_go_nm": to_go_nm, "leg": leg,
                                "gs": gs_kts, "pattern_gs": pattern_gs_kts,
                                "budget_s": w.budget_left_s(t), "alt": alt, "radio": radio_ft,
                                "dist": raw.dist_nm}),
                        );
                    }
                    w.verdict(t).map(|v| (v, progress, w.stalled_s(t), w.budget_left_s(t)))
                } else {
                    missed_watch = None;
                    None
                };
                if let Some((verdict, progress, stalled_s, budget_left_s)) = watched {
                    // A give-up must hand back a FLYABLE aircraft. The balked
                    // landing left gear down and landing flap out, and a flat
                    // 0.5 throttle cannot hold level flight dirty: the Baron
                    // decayed 110 to 80 kt at -900 fpm in the thirty seconds
                    // after the old handback (EGHI, 2026-08-02). Climb power,
                    // the VS dial zeroed (the AP is already holding heading),
                    // and the line says what is actually held.
                    gave_up = true;
                    let thr_dref = throttle_dataref();
                    let _ = tokio::join!(
                        xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                        autoflight::set_dial(&xp, Dial::Vs, 0.0),
                    );
                    announce(
                        format!(
                            "I can't get us back round for another approach. Your aircraft. \
                             The autopilot is holding heading and altitude{}.",
                            if gear_down { ", gear is down" } else { "" }
                        ),
                        Priority::Warning,
                    );
                    let to_go_nm = match progress {
                        crate::reversal::Progress::Pattern { to_go_nm, .. } => Some(to_go_nm),
                        _ => None,
                    };
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": verdict.reason(),
                            "stage": progress.word(), "to_go_nm": to_go_nm,
                            "stalled_s": stalled_s, "budget_left_s": budget_left_s,
                            "missed": missed.is_some(), "leg": reversal.map(|r| r.leg.word()),
                            "alt": alt, "radio": radio_ft, "dist": raw.dist_nm, "gs": gs_kts}),
                    );
                    break;
                }
                // Runaway backstop: an approach that keeps missing (arrives too
                // fast/high to ever capture) would loop go-arounds forever,
                // accelerating toward Vne. After MAX_GO_AROUNDS, stop trying:
                // level the wings, throttle to a safe climb, and give the
                // aircraft back — a real crew would divert, not keep diving at
                // a runway they can't make (LXGB Twin Otter, 2026-07-26).
                // Too many go-arounds: give up, but only once the climb-away
                // is flown. The third LFLL balk released the aircraft at 95 ft
                // with landing flaps and climb power, "holding altitude": it
                // accelerated at 300 ft AGL to 236 kt through the flap limit
                // (2026-09-17). The latch's Climb stage runs to
                // GO_AROUND_TURN_AGL_FT under full power with the go-around
                // flap; the handback happens there, flaps retracted.
                if runtime.go_around.attempts() > MAX_GO_AROUNDS && missed != Some(MissedStage::Climb) {
                    gave_up = true;
                    let thr_dref = throttle_dataref();
                    let _ = tokio::join!(
                        xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                        autoflight::set_dial(&xp, Dial::Vs, 0.0),
                        crate::tools::flaps::set_handle(&xp, 0.0),
                    );
                    announce(
                        format!(
                            "Unable to land, we've gone around too many times. Climbed away to \
                             {} feet. Your aircraft. The autopilot is holding heading and \
                             altitude, flaps up, gear is down.",
                            (radio_ft / 100.0).round() * 100.0
                        ),
                        Priority::Warning,
                    );
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "max_go_arounds", "attempts": runtime.go_around.attempts()}),
                    );
                    break;
                }
                let g = match missed {
                    Some(stage) => missed_approach_guidance(
                        &rwy,
                        stage,
                        lat,
                        lon,
                        alt,
                        radio_ft,
                        raw.dist_nm,
                        raw.glidepath_alt_ft,
                        ias,
                        approach_ias,
                        // The speed's trend since the last tick's power
                        // law read it (the same pair; nothing written).
                        prev_ias.map_or(0.0, |p| {
                            (ias - p) / prev_ias_t.elapsed().as_secs_f64().clamp(0.1, 2.0)
                        }),
                        crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0)),
                    ),
                    None => raw,
                };
                // The pattern's own command over whichever of those stood.
                let g = match pattern {
                    Some((hdg, leg, platform_ft)) => {
                        let slow = approach_ias - ias > SPEED_FLOOR_MARGIN_KTS;
                        let inbound = leg == crate::reversal::Leg::Inbound;
                        Guidance {
                            target_heading_deg: hdg,
                            target_vs_fpm: crate::reversal::platform_vs_fpm(
                                platform_ft,
                                alt,
                                slow,
                                max_climb_fpm(),
                                crate::procedure::descent_fpm(),
                            ),
                            // The inbound never accelerates back past the
                            // pattern's own speed: at 14.5 nm the schedule
                            // read "clean" and took the 737 from 177 to 222 kt
                            // level before the path (LEMG 13, 2026-10-05).
                            target_ias_kts: if inbound {
                                scheduled_ias(g.dist_nm, approach_ias)
                                    .min(pattern_ias(approach_ias, &crate::aircraft::flap_limits()))
                            } else {
                                pattern_ias(approach_ias, &crate::aircraft::flap_limits())
                            },
                            phase: ApproachPhase::Intercept,
                            balk: None,
                            arresting: false,
                            ..g
                        }
                    }
                    None => g,
                };
                // ...or the hold's (#182): straight ahead at its altitude,
                // while the pattern is planned or because none clears the
                // ground. A refusal hands over only once it is there, the
                // autopilot holding heading and altitude and the governor the
                // speed (`gave_up`): never "your aircraft" low over the hills.
                let slow = approach_ias - ias > SPEED_FLOOR_MARGIN_KTS;
                if pattern.is_some() {
                    hold = None;
                }
                let hold_done = hold.as_mut().is_some_and(|h| h.level_now(alt));
                if hold_done {
                    let h = hold.take().expect("a hold that is done exists");
                    gave_up = true;
                    let _ = autoflight::set_dial(&xp, Dial::Vs, 0.0).await;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "terrain_refused",
                                           "hold_ft": h.alt_ft, "alt": alt, "radio": radio_ft}),
                    );
                    announce(
                        format!(
                            "Level at {:.0} feet, straight ahead, the autopilot holding heading and \
                             altitude{}. Your aircraft.",
                            h.alt_ft,
                            if gear_down { ", gear is down" } else { "" }
                        ),
                        Priority::Warning,
                    );
                    break;
                }
                let g = match (pattern, hold) {
                    (None, Some(h)) => Guidance {
                        target_heading_deg: h.hdg_true,
                        target_vs_fpm: crate::reversal::platform_vs_fpm(
                            h.alt_ft,
                            alt,
                            slow,
                            max_climb_fpm(),
                            crate::procedure::descent_fpm(),
                        ),
                        target_ias_kts: pattern_ias(approach_ias, &crate::aircraft::flap_limits()),
                        phase: ApproachPhase::Intercept,
                        balk: None,
                        arresting: false,
                        ..g
                    },
                    _ => g,
                };
                // The live floor (#182): every leg but the final keeps
                // `circuit::live_floor_ft` over the ground under and 1.5 nm
                // ahead, probed every 2 s: the level legs climb 800 ft clear
                // of it, the base and an intercept only keep the floor. The
                // go-around's climb is its own; the final is the landing's.
                // A steep (visual segment, mountain) approach's intercept is
                // a valley's, flown on its published path: its walls ahead
                // are not ours to climb away from.
                let floor_leg = live_floor_leg(
                    g.phase,
                    missed,
                    pattern.map(|(_, leg, _)| leg),
                    hold.is_some(),
                )
                .filter(|_| {
                    pattern.is_some()
                        || hold.is_some()
                        || missed.is_some()
                        || glidepath_deg() <= 3.0 + 1e-9
                });
                let mut g = g;
                if let Some(leg) = floor_leg {
                    if live_floor_probed.is_none_or(|t| t.elapsed().as_secs_f64() >= LIVE_FLOOR_EVERY_S) {
                        live_floor_probed = Some(std::time::Instant::now());
                        let mut pts = vec![(lat, lon)];
                        pts.extend(crate::terrain::look_ahead_points(
                            lat,
                            lon,
                            track_true,
                            crate::circuit::LIVE_LOOK_NM,
                            crate::circuit::LIVE_LOOK_POINTS,
                        ));
                        if let Some(e) = crate::terrain::max_elevation_ft(&xp, &pts).await {
                            if let Some(floor) = crate::circuit::live_floor_ft(leg, alt, e) {
                                let raised = live_floor.is_none_or(|(f, _)| floor > f);
                                live_floor = Some((
                                    live_floor.map_or(floor, |(f, _)| f.max(floor)),
                                    std::time::Instant::now(),
                                ));
                                if raised {
                                    crate::blackbox::record(
                                        "autoland",
                                        serde_json::json!({"event": "terrain_floor",
                                            "leg": format!("{leg:?}"),
                                            "pattern_leg": pattern.map(|(_, l, _)| l.word()),
                                            "hold": hold.is_some(), "ground_ft": e, "alt": alt,
                                            "radio": radio_ft, "floor_ft": floor}),
                                    );
                                }
                                if !live_floor_said {
                                    live_floor_said = true;
                                    announce(
                                        format!(
                                            "Terrain ahead. Holding {:.0} feet.",
                                            (floor / 50.0).ceil() * 50.0
                                        ),
                                        Priority::Warning,
                                    );
                                }
                            }
                        }
                    }
                } else {
                    live_floor = None;
                }
                // A spell ends with its floor: the next one is said again.
                live_floor = live_floor.filter(|(_, t)| t.elapsed().as_secs_f64() < LIVE_FLOOR_HOLD_S);
                if live_floor.is_none() {
                    live_floor_said = false;
                }
                if let Some((floor, _)) = live_floor {
                    g.target_vs_fpm = g.target_vs_fpm.max(crate::reversal::platform_vs_fpm(
                        floor,
                        alt,
                        slow,
                        max_climb_fpm(),
                        crate::procedure::descent_fpm(),
                    ));
                }
                // The plan's ask (`approach_energy::ask`): level, slow to the
                // speed the path is joined at; on the path, no faster than
                // its own curve. Descent first: the vertical law is not
                // touched. Not in a go-around, a hold or the flare.
                let gate_nm = gate.gate_ft() / gp_ft_per_nm() - TOUCHDOWN_AIM_NM;
                let energy = energy_model
                    .as_ref()
                    .filter(|_| missed.is_none() && hold.is_none() && g.phase != ApproachPhase::Flare)
                    .map(|m| {
                        let (s_nm, path_nm) = energy_geometry(
                            reversal.as_ref().filter(|_| pattern.is_some()).map(|rev| {
                                let join = join_point_nm(rev.platform_ft, rwy.elev_ft);
                                let end = crate::reversal::pattern_end_nm(
                                    rev.platform_ft,
                                    rwy.elev_ft,
                                    gp_ft_per_nm(),
                                    join,
                                );
                                let to_go = crate::reversal::to_go_nm(
                                    rev,
                                    along_nm,
                                    xtk_now,
                                    pattern_gs_kts,
                                    join,
                                    end,
                                );
                                (to_go, end)
                            }),
                            g.dist_nm,
                            alt - g.glidepath_alt_ft,
                            gp_ft_per_nm(),
                            gate_nm,
                        );
                        let ask = crate::approach_energy::ask(m, energy_vapp, s_nm, path_nm, gs_kts);
                        let gate_kts =
                            crate::approach_energy::predicted_gate_kts(m, energy_vapp, ias, s_nm, path_nm);
                        (ask, s_nm, path_nm, gate_kts)
                    });
                let mut g = g;
                // Below the terrain floor the guidance asks for climb energy
                // over the schedule (`CLIMB_ENERGY_BONUS_KTS`): never capped.
                let floor_bonus =
                    pattern.is_none() && g.target_ias_kts > scheduled_ias(g.dist_nm, approach_ias) + 0.5;
                // A new pattern is a new final: planned afresh.
                if reversal_began {
                    energy_cap = None;
                }
                match energy.filter(|_| !floor_bonus) {
                    Some((ask, ..)) => {
                        let kts = g.target_ias_kts.min(ask.kts).max(approach_ias);
                        let kts = energy_cap.map_or(kts, |c| kts.min(c));
                        energy_cap = Some(kts);
                        g.target_ias_kts = kts;
                    }
                    // A go-around, a hold, the flare: the next final plans
                    // afresh.
                    None => energy_cap = None,
                }
                // The final's lateral law stands aside while the airframe's
                // own localiser has the final on a landing its autopilot
                // flies (`ap_lands`): the law is that path's own-law
                // fallback, never a second lateral.
                let ap_has_loc = lands.is_some()
                    && matches!(
                        coupling,
                        crate::ils::Coupling::LocCaptured | crate::ils::Coupling::Coupled
                    );
                let g = g;
                // MINIMUMS (2026-10-12): above them the autopilot flies the
                // final in the modes built for it; from them it is hand flown
                // to the wheels as ONE segment, servos out (`crate::pilot`).
                // Once taken it is held to the wheels or a go-around.
                let minimums = minimums_ft(
                    matches!(coupling, crate::ils::Coupling::LocCaptured | crate::ils::Coupling::Coupled),
                    crate::aircraft::live_profile().and_then(|p| p.ils_decouple_agl_ft),
                );
                let hand_flown = !ap_flies
                    && (g.arresting
                        || (g.phase == ApproachPhase::Final
                            && pattern.is_none()
                            && !ga
                            && (arrest_hands || (radio_ft > 0.0 && radio_ft < minimums))));
                let outbound = hold.is_some()
                    || pattern.is_some_and(|(_, leg, _)| leg != crate::reversal::Leg::Inbound);
                let phase_word = match g.phase {
                    ApproachPhase::Intercept => "INTERCEPT",
                    ApproachPhase::Final => "FINAL",
                    ApproachPhase::Flare => "FLARE",
                };
                set_panel(&format!(
                    "AUTOLAND {} {} {:.1}nm {}",
                    rwy.icao, rwy.id, g.dist_nm, phase_word
                ));
                tracing::info!(
                    target: "xflightcrew::autoland",
                    dist = g.dist_nm, alt, gp_alt = g.glidepath_alt_ft, vs = g.target_vs_fpm,
                    hdg = g.target_heading_deg, ias, radio = radio_ft, phase = ?g.phase,
                    "autoland"
                );
                // The vertical speed actually flown: the watchdog below reads
                // it, and so does the landing arrest, whose dial leads the
                // target through the autopilot's lag (`vs_dial_fpm`). The
                // blackbox keeps both columns: `cmd_vs` is what the law
                // wants, `vs_dial` what the autopilot was told.
                // A failed read is NO measurement, never the command in
                // disguise: the dial gets the bare target that tick, the
                // watchdog does not count it, and the blackbox keeps `null`
                // so the debrief can see the gauge was missing.
                // From the tick's one read (`st`), no round of its own.
                let vvi_read = st.get(F::Vvi);
                let throttle_read = throttle_read
                    .take()
                    .unwrap_or_else(|| Err(anyhow::anyhow!("unread")));
                let servos_read = if engage.confirmed() { None } else { panel_tick.status(&st, F::Servos).ok() };
                // The engage watch (see `ApEngage`): re-press until the
                // servos read on with the sim live, each try on the tape.
                let engage_now = engage_t0.elapsed().as_secs_f64();
                // Airborne only: a servo pressed on the wheels winds the
                // yoke over (`disconnect_autopilot`).
                let es = if on_ground < 0.5 && !rolling_out {
                    engage.step(engage_now, servos_read.filter(|v| v.is_finite()), ias)
                } else {
                    EngageStep::default()
                };
                if es.fire {
                    let ok = crate::ap_engage::press(&xp, &cfg.fd_commands).await;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "ap_engage", "try": es.try_n, "sent": ok,
                            "at_s": engage_now, "servos": servos_read, "ias": ias,
                            "radio": radio_ft}),
                    );
                }
                if es.confirmed_now {
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "ap_engaged", "tries": es.try_n,
                            "at_s": engage_now, "held": engage_held, "radio": radio_ft}),
                    );
                    authority_owed = true;
                }
                if es.give_up {
                    gave_up = true;
                    let _ = xp.set_float(&throttle_dataref(), GIVE_UP_THROTTLE).await;
                    announce(ENGAGE_GIVE_UP_LINE.into(), Priority::Warning);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "ap_not_engaged",
                            "tries": es.try_n, "at_s": engage_now, "radio": radio_ft}),
                    );
                    break;
                }
                let thr_now = throttle_read.ok().filter(|v| v.is_finite());
                let vvi = match vvi_read {
                    Ok(v) => Some(v),
                    Err(e) => {
                        if !vvi_read_failed {
                            vvi_read_failed = true;
                            tracing::warn!(target: "xflightcrew::autoland", error = %e, "VSI read failed; flying the bare VS target");
                        }
                        None
                    }
                };
                // The sink's rate of change over the last tick (fpm per
                // second), the arrest's damping term on a slow autopilot.
                let tick_dt = prev_vvi
                    .map(|(_, t0)| t_now.duration_since(t0).as_secs_f64())
                    .filter(|d| (0.02..=3.0).contains(d))
                    .unwrap_or(0.2);
                let vvi_trend = match (vvi, prev_vvi) {
                    (Some(v), Some((p, t0))) => {
                        let dt = t_now.duration_since(t0).as_secs_f64();
                        if (0.1..=3.0).contains(&dt) { (v - p) / dt } else { 0.0 }
                    }
                    _ => 0.0,
                };
                prev_vvi = vvi.map(|v| (v, t_now));
                // The final on its own path: the VS set once to the path's
                // rate and nudged, as a pilot flies it (`path_nudge`).
                // Not while the autopilot flies a captured glidepath: the dial is
                // not its input then, and nudged against our own path it walked to
                // -50 under a GPS path; handed back at 200 ft, the 172 levelled
                // over the runway (LEMG 12, 1791807406). Handed back, it is set
                // afresh to the path's rate.
                let nudged = (g.phase == ApproachPhase::Final
                    && !hand_flown
                    && !ap_modes.is_some_and(|m| m.gs()))
                .then(|| {
                    let path_fpm = -(gs_kts / 60.0) * gp_ft_per_nm();
                    path_nudge.step(path_fpm, alt - raw.glidepath_alt_ft, path_clock.elapsed().as_secs_f64())
                });
                if nudged.is_none() {
                    path_nudge.reset();
                }
                // The dial is the target: the arrest that led it through a VS
                // mode's lag (`arrest_lead`) is hand flown since 2026-10-12.
                let vs_dial = nudged.unwrap_or(g.target_vs_fpm);
                // The servos' pitch AUTHORITY (#181): the hand-flown segment
                // takes the servos out, so the check ends there; before it, a
                // dial the autopilot's elevator does not fly is hand flown
                // level by the recovery and the approach abandoned.
                if hand_flown || !engage.confirmed() || es.hold {
                    if hand_flown {
                        crate::climb_watch::disarm_authority();
                        authority_owed = false;
                    }
                } else if authority_owed {
                    if let Some(v) = vvi {
                        crate::climb_watch::arm_authority("autoland", v, vs_dial);
                        authority_owed = false;
                    }
                } else if let Some(crate::climb_watch::Authority::NoAuthority(why)) =
                    crate::climb_watch::feed_authority(vvi, Some(vs_dial))
                {
                    let started = crate::tools::recover::take_over(
                        base_url.clone(),
                        voice.clone(),
                        crate::tools::recover::TakeOver {
                            why: format!("autoland_{why}"),
                            min_safe_ias: crate::aircraft::config_floor_kts(
                                last_flap.unwrap_or(0.0),
                            )
                            .map(|f| f + crate::circuit::RECOVERY_MARGIN_KTS),
                            hold_kts: None,
                            ap_no_pitch: true,
                            hold_ft: None,
                        },
                    );
                    announce("Abandoning the approach.".into(), Priority::Warning);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "ap_no_pitch",
                            "why": why, "recovery_started": started, "radio": radio_ft}),
                    );
                    break;
                }
                // THE HAND-FLOWN SEGMENT FLIES THE ELEVATOR, from minimums: the
                // glidepath held by attitude, then the flare from it (`pilot::
                // attitude`), only with its measurements in hand. Above
                // minimums the autopilot flies it. The channel is a 1 s
                // expiring heartbeat: ceasing to write it hands the pitch back.
                // A landing the autopilot flies (`ap_lands`) is its flare:
                // our column never arrests it (the plugin releases our yoke
                // channels while CMD is lit anyway).
                let mut att_out: Option<crate::pilot::attitude::Out> = None;
                let flare_elev = match (hand_flown, vvi, pitch_yielded) {
                    // The arrest by attitude: the path from the attitude, no
                    // VSI. No attitude read is no decision (the channel
                    // expires by itself).
                    (true, _, false) => {
                        match (plant_at(0), plant_at(1), plant_at(2), plant_at(5)) {
                            (Some(theta_deg), Some(alpha_deg), Some(q_dps), Some(tas_ms)) => {
                                let o = attitude_flare.step(
                                    crate::pilot::attitude::In {
                                        radio_ft,
                                        theta_deg,
                                        alpha_deg,
                                        q_dps,
                                        ias_kts: ias,
                                        tas_kts: tas_ms * 1.943_844,
                                        dt: tick_dt,
                                        path: (g.phase == ApproachPhase::Final).then(|| crate::pilot::attitude::Path {
                                            vs_fpm: -(gs_kts / 60.0) * gp_ft_per_nm(),
                                            above_ft: alt - g.glidepath_alt_ft,
                                        }),
                                    },
                                    &pilot_airframe.pitch,
                                    pilot_airframe.limits,
                                );
                                att_out = Some(o);
                                Some(o.elev)
                            }
                            _ => None,
                        }
                    }
                    // NOBODY IS FLYING: the autopilot is not confirmed and
                    // its servos read off. Level on the elevator until it
                    // is (`ApEngage`, `engage_hold_gains`); the same
                    // channel, the same single write below.
                    (false, Some(v), false) if es.hold => {
                        engage_held = true;
                        Some(engage_pitch.step(0.0, v, vvi_trend, tick_dt))
                    }
                    _ => {
                        attitude_flare.reset();
                        None
                    }
                };
                // THE ARREST TAKES THE SERVOS. The pitch channel writes the
                // yoke under the joystick override and the autopilot's pitch
                // servo writes it back the same frame: LFPG 09R (2026-09-22)
                // held 0.35 of elevator for three seconds and touched down at
                // 0.1 degrees of pitch and -672 fpm, the sink following the
                // VS dial's own slow shape exactly as it had before the
                // elevator law existed (every 737 landing since it shipped
                // shows the same). The channel is proven with the servos
                // OFF - the takeoff rotates on it - so minimums are where the
                // autopilot comes off: pitch on the elevator channel, the
                // centreline on the roll channel, the power on the energy
                // throttle. Only a go-around hands the servos back and
                // re-engages the modes (a balloon over the band stays the
                // arrest, `hold_roundout`); the rollout latch finds them off.
                // ABOVE MINIMUMS the final's heading is SET as a pilot sets it
                // (`FinalHeading`, whole degrees, `AUTOLAND_FINAL_DEADBAND`):
                // the knob had moved a quarter degree every second
                // (`FinalLateral`) and rocked the Evolution +-14 degrees of
                // bank (1791826866).
                let track_now = hpath_r.as_ref().ok().copied();
                let crabbed_true = match g.phase {
                    ApproachPhase::Final if !hand_flown && ap_has_loc => {
                        (rwy.course_deg + crab).rem_euclid(360.0)
                    }
                    ApproachPhase::Final if !hand_flown => {
                        final_heading.heading_true(xtk_now * M_PER_NM, gs_kts, rwy.course_deg, crab)
                    }
                    ApproachPhase::Final | ApproachPhase::Flare => {
                        (g.target_heading_deg + crab).rem_euclid(360.0)
                    }
                    ApproachPhase::Intercept => g.target_heading_deg,
                };
                match arrest_hands_step(arrest_hands, hand_flown) {
                    ArrestHands::Take => {
                        let ok = autoflight::press(&xp, Button::Disconnect).await.is_ok();
                        arrest_hands = true;
                        bank_hold = crate::pilot::bank::BankHold::default();
                        yaw_hold = crate::pilot::yaw::YawHold::default();
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({
                                "event": "arrest_hands", "taken": true, "ok": ok, "radio": radio_ft,
                                "minimums": minimums, "arresting": g.arresting,
                            }),
                        );
                    }
                    ArrestHands::Return => {
                        // The servos back through the one engage door (FD
                        // first, `ap_engage` on the tape). Not a fresh
                        // `ApEngage` watch: its level hold would fight the
                        // balk's climb and its give-up would hand a go-around
                        // over; the VS watchdog below judges what comes back.
                        let ok = crate::ap_engage::press_logged(
                            &xp, &cfg.fd_commands, "autoland", "arrest_return", 1,
                        )
                        .await;
                        autoflight::apply(
                            &xp,
                            &[Button::Heading, Button::VerticalSpeed],
                            "autoland_arrest_return",
                        )
                        .await;
                        let _ = autoflight::set_dial(&xp, Dial::Vs, vs_dial).await;
                        arrest_hands = false;
                        roundout_taken = false;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "arrest_hands", "taken": false, "ok": ok, "radio": radio_ft}),
                        );
                    }
                    ArrestHands::Off | ArrestHands::Keep => {}
                }
                roundout_taken |= arrest_hands && g.arresting;
                // The wings while the segment has the servos: the bank that
                // closes the painted centreline by the TRACK (`pilot::line`;
                // the coupled 172 held its localiser 8 to 23 m off it at
                // minimums), the sim's ground track against the course, the
                // smoothed cross-track rate when no track was read, flown on
                // the ailerons (`pilot::bank`). No bank read is no command.
                let xtk_m_now = xtk_now * M_PER_NM;
                let arrest_rate_now = arrest_rate.update(
                    arrest_prev_xtk.map_or(0.0, |p: f64| (xtk_m_now - p) / tick_dt.max(0.05)),
                    tick_dt.max(0.05),
                );
                arrest_prev_xtk = Some(xtk_m_now);
                let side_now = crate::pilot::line::side_velocity_m_s(track_now, rwy.course_deg, gs_kts)
                    .unwrap_or(arrest_rate_now);
                let tas_now = plant_at(5).map_or(ias, |v| v * 1.943_844);
                // The rudder while the segment has the servos (`pilot::yaw`):
                // coordinated, then the decrab once the wheels are the crab's
                // turn and the yaw loop's settling away (the flare's own plan
                // of the time to them), held from then. No heading, slip, yaw
                // rate or attitude read is no command (the channel expires by
                // itself).
                let mut yaw_mode = None;
                let mut slip_deg = plant_at(8).unwrap_or(0.0);
                let flare_rud = match (
                    arrest_hands && !steer_yielded,
                    psi_r.as_ref().ok().copied(),
                    plant_at(8),
                    st.opt(F::R),
                    phi_r.as_ref().ok().copied(),
                    plant_at(0),
                ) {
                    (true, Some(psi_deg), Some(beta_deg), Some(r_dps), Some(phi_deg), Some(theta_deg)) => {
                        let yi = crate::pilot::yaw::In {
                            psi_deg,
                            beta_deg,
                            r_dps,
                            phi_deg,
                            theta_deg,
                            ias_kts: ias,
                            tas_kts: tas_now,
                            dt: tick_dt,
                            course_deg: rwy.course_deg,
                        };
                        let mode = yaw_hold.mode(
                            att_out.filter(|o| o.flaring).map(|o| o.to_wheels_s),
                            &yi,
                            &pilot_airframe.yaw,
                        );
                        yaw_mode = Some(mode);
                        slip_deg = crate::pilot::yaw::slip_planned_deg(&yi, mode);
                        Some(yaw_hold.step(yi, mode, &pilot_airframe.yaw))
                    }
                    _ => None,
                };
                // The bank: the centreline's (NPFG), with the wing down for
                // the slip the rudder holds (or will hold, in the decrab).
                let line_period = line_period_s(ias, tas_now, tick_dt);
                let bank_tgt = crate::pilot::line::bank_for_line(
                    xtk_m_now,
                    side_now,
                    phi_r.as_ref().ok().copied().unwrap_or(0.0),
                    radio_ft,
                    line_period,
                    crate::pilot::line::slip_accel_m_s2(slip_deg, pilot_airframe.yaw.at(ias, tas_now).path, tas_now),
                );
                let flare_ail = match (arrest_hands, phi_r.as_ref().ok().copied(), roll_yielded) {
                    (true, Some(phi), false) => Some(bank_hold.step(
                        phi,
                        bank_tgt,
                        tick_dt,
                        &pilot_airframe.roll,
                        ias,
                        tas_now,
                        plant_at(8).unwrap_or(0.0),
                    )),
                    _ => None,
                };
                TRACKING_ERROR.set_in(flight, xtk_now * M_PER_NM);
                crate::blackbox::record("autoland", serde_json::json!({
                    "rwy": format!("{} {}", rwy.icao, rwy.id), "dist": g.dist_nm,
                    "alt": alt, "gp_alt": g.glidepath_alt_ft, "cmd_vs": g.target_vs_fpm,
                    // The energy plan (`approach_energy`): its speed, its
                    // flap, the join speed, the speed it predicts at the
                    // gate from here at idle, the route to the gate and the
                    // path's share of it.
                    "tgt_ias": g.target_ias_kts,
                    "e_kts": energy.map(|(a, ..)| a.kts), "e_flap": energy.and_then(|(a, ..)| a.flap),
                    "e_join": energy.map(|(a, ..)| a.join_kts), "e_gate": energy.map(|(.., k)| k),
                    "e_s": energy.map(|(_, s, ..)| s), "e_path": energy.map(|(_, _, p, _)| p),
                    "vs_dial": vs_dial, "vvi": vvi, "vs_fm": vs_fm, "vvi_trend": vvi_trend, "arresting": g.arresting,
                    "theta": plant_at(0), "alpha": plant_at(1), "q": plant_at(2),
                    "vy_true": plant_at(3).map(|v| v * 196.850_394), "elev_total": plant_at(4),
                    "tas": plant_at(5).map(|v| v * 1.943_844),
                    "ail_total": plant_at(6), "rud_total": plant_at(7), "beta": plant_at(8),
                    "att_gate": att_out.map(|o| o.gate_ft), "att_vs_t": att_out.map(|o| o.vs_target_fpm),
                    "att_gamma": att_out.map(|o| o.gamma_deg), "att_gamma_cmd": att_out.map(|o| o.gamma_cmd_deg),
                    "att_theta_cmd": att_out.map(|o| o.theta_cmd_deg),
                    "cmd_hdg": g.target_heading_deg, "ias": ias, "radio": radio_ft,
                    // The wheels, so a first contact the rollout latch never
                    // saw (a bounce) is on the tape: the scorecard judges it.
                    "ground": on_ground > 0.5,
                    "phase": format!("{:?}", g.phase), "ga": ga, "crab": crab, "crab_raw": crab_raw,
                    "xtk_m": xtk_now * M_PER_NM, "minimums": minimums, "hdg_bias": final_heading.bias_deg(),
                    // The roll response the segment has measured (`pilot::bank`).
                    "kp": arrest_hands.then(|| bank_hold.kp()).flatten(),
                    // The rudder (`pilot::yaw`), the decrab's time, the
                    // centreline's period and the attitude law's gain.
                    "flare_rud": flare_rud, "yaw_mode": yaw_mode.map(|m| format!("{m:?}")),
                    "line_period_s": arrest_hands.then_some(line_period),
                    "att_k": att_out.map(|o| o.k_theta), "att_to_wheels_s": att_out.map(|o| o.to_wheels_s),
                    // "plugin": one frame's position; "split": an older
                    // plugin's separate reads (#157).
                    "pos": pos_source,
                    "flare_elev": flare_elev, "flare_ail": flare_ail, "hands": arrest_hands,
                    // The autopilot not yet confirmed and read off: the
                    // elevator above is the level hold's (`ApEngage`).
                    "ap_hold": es.hold,
                    "lever": last_lever,
                    "phi": phi_r.as_ref().ok().copied(),
                    "psi": psi_r.as_ref().ok().copied(), "bug_true": crabbed_true,
                    // The arrest's lateral: the track it closes on, the
                    // sideways velocity and the bank it asked for.
                    "trk": track_now, "side_m_s": side_now,
                    "bank_tgt": arrest_hands.then_some(bank_tgt),
                    // Which leg of a reversal is flying: every pattern tick
                    // reports phase Intercept, so without this the outbound,
                    // the base and the inbound are one undifferentiated block
                    // in the log and an overshoot cannot be read back to the
                    // leg that flew it.
                    "leg": pattern.map(|(_, leg, _)| leg.word()),
                    // The hold's altitude (#182): planning, or refused.
                    "hold_ft": hold.map(|h| h.alt_ft),
                    "floor_ft": live_floor.map(|(f, _)| f),
                }));

                // Lateral: heading bug in MAGNETIC (guidance is TRUE — see
                // [`true_to_mag`] for the variation sign trap). On final and in
                // the flare, add the wind crab so the commanded heading flies a
                // runway-aligned TRACK; intercept vectors stay coarse.
                let mag_hdg = true_to_mag(crabbed_true, var);
                // Knob presses, not a raw write — registers on glass (G1000) too.
                // Slew-capped: each press is an HTTP round trip, and the LXGB go-
                // around's 178° flip stalled this loop ~5 s per tick (2026-07-14) —
                // no VS/throttle updates the whole time. 30°/tick keeps a full
                // reversal turning while the loop stays live.
                // The bug, the VS dial and BOTH mode checks in one burst. They
                // were four sequential round trips on the hot path, and at
                // 40-70 ms each that is a quarter of a second of tick time
                // spent waiting — on top of the sleep. Nothing here depends on
                // anything else here: the mode reads decide whether to re-engage
                // AFTER the fact, and a dial written while its mode is off is
                // exactly what the re-engage below is for.
                // The elevator rides in the SAME burst. Written every tick it
                // is wanted (the plugin channel is a 1 s heartbeat, so a
                // change-gate would drop it exactly where it sits still), and
                // simply not written otherwise — which hands the pitch back
                // inside a second with nothing to remember.
                // The ILS: its needles against our path, and the coupled
                // approach where the airframe allows it.
                let on_final = g.phase == ApproachPhase::Final && pattern.is_none() && !ga;
                ON_FINAL.store(
                    final_report_due(
                        g.phase == ApproachPhase::Final,
                        pattern.is_some() || reversal.is_some() || hold.is_some(),
                        ga,
                        track_true,
                        rwy.course_deg,
                    ),
                    Ordering::SeqCst,
                );
                // APR pressed this tick: the tick's mode read is from before it.
                let mut appr_pressed = false;
                let needles = ils.as_ref().map(|_| {
                    // From the tick's one read.
                    let (h, v, lv, gv) = (st.get(F::Hdef), st.get(F::Vdef), st.get(F::LocValid), st.get(F::GsValid));
                    crate::ils::Needles {
                        hdef: h.ok(),
                        vdef: v.ok(),
                        loc_valid: lv.is_ok_and(|x| x > 0.5),
                        gs_valid: gv.is_ok_and(|x| x > 0.5),
                    }
                });
                let cpl = match needles {
                    Some(n) => {
                        if let Some(line) = ils_xcheck.step(n, on_final, radio_ft, tick_dt) {
                            crate::blackbox::record(
                                "ils",
                                serde_json::json!({"event": "xcheck", "line": line, "hdef": n.hdef,
                                    "vdef": n.vdef, "xtk_m": xtk_now * M_PER_NM, "radio": radio_ft}),
                            );
                            announce(line, Priority::Caution);
                        }
                        // The airframe's own capture, armed on an intercept
                        // to the localizer from below the beam: the final, a
                        // straight-in heading for it, or a pattern's inbound.
                        let inbound_leg = pattern.is_none_or(|(_, leg, _)| leg == crate::reversal::Leg::Inbound);
                        let toward_loc = ils.as_ref().zip(psi_r.as_ref().ok()).is_some_and(|(i, psi)| {
                            crate::ils::arm_verdict(lat, lon, true_to_mag(*psi, var), i)
                                == crate::ils::ArmVerdict::Now
                        });
                        let before = coupling;
                        let out = coupling.step(
                            crate::ils::CouplingIn {
                                allowed: coupled_ok,
                                inbound: !ga && hold.is_none() && inbound_leg && (on_final || toward_loc),
                                arresting: g.arresting || hand_flown,
                                leaving: ga || hold.is_some() || !inbound_leg,
                                dist_nm: g.dist_nm,
                                needles: n,
                                modes: ap_modes,
                                radio_ft,
                                // A landing the autopilot flies keeps it to
                                // the wheels (`ap_lands`).
                                decouple_below_ft: if lands.is_some() {
                                    None
                                } else {
                                    crate::aircraft::live_profile().and_then(|p| p.ils_decouple_agl_ft)
                                },
                            },
                            &mut coupling_armed_s,
                            tick_dt,
                        );
                        appr_pressed = out.press_appr;
                        if out.press_appr {
                            if let Some(ok) = crate::ils::course_source_to_nav1(&xp).await {
                                crate::blackbox::record(
                                    "ils",
                                    serde_json::json!({"event": "course_source", "to": "nav1", "ok": ok}),
                                );
                            }
                        }
                        if out.press_appr || out.disarm {
                            autoflight::press_logged(
                                &xp,
                                Button::Approach,
                                if out.disarm { "autoland_decouple" } else { "autoland_coupling" },
                            )
                            .await;
                        }
                        // A decouple is an INTENT: the mode housekeeping
                        // below presses HDG and V/S back once, from the
                        // tick's read (pressed here too, a toggle went twice).
                        if coupling != before {
                            crate::blackbox::record(
                                "ils",
                                serde_json::json!({"event": "coupling",
                                    "modes": ap_modes.map(|m| m.words()),
                                    "from": format!("{before:?}"),
                                    "to": format!("{coupling:?}"), "hdef": n.hdef, "vdef": n.vdef,
                                    "radio": radio_ft, "dist": g.dist_nm}),
                            );
                            match coupling {
                                crate::ils::Coupling::Coupled => announce(
                                    "Localizer and glideslope captured. The autopilot has the ILS.".into(),
                                    Priority::Advisory,
                                ),
                                crate::ils::Coupling::Abandoned if out.decouple => announce(
                                    "Off the ILS coupling. I'm flying the approach myself.".into(),
                                    Priority::Caution,
                                ),
                                _ => {}
                            }
                        }
                        out
                    }
                    None => crate::ils::CouplingOut {
                        press_appr: false,
                        decouple: false,
                        disarm: false,
                        own_lateral: true,
                        own_vertical: true,
                    },
                };
                // THE AUTOPILOT LANDS IT (`ap_lands`), where declared: the
                // second channel after the glideslope, the landing armed by
                // its gate, else taken back loudly; the A/T in SPEED on our
                // approach target. Every FMA column on the tape.
                let mut cpl = cpl;
                if let (Some(l), Some(r), Some(f)) = (lands.as_ref(), lands_read.as_ref(), fma.as_ref()) {
                    let now_s = ap_t0.elapsed().as_secs_f64();
                    let before = ap_landing;
                    let lo = ap_landing.step(
                        crate::ap_lands::LandIn {
                            coupling,
                            radio_ft,
                            airborne: on_ground < 0.5,
                            going_around: ga,
                            cmd_a: cmd_a_lit,
                            cmd_b: cmd_b_lit,
                            fma: *f,
                            now_s,
                        },
                        l,
                    );
                    if lo.press_second {
                        autoflight::press_logged(&xp, Button::EngageSecond, "ap_lands").await;
                    }
                    if lo.press_toga {
                        autoflight::press_logged(&xp, Button::Toga, "ap_lands").await;
                    }
                    if let Some(fb) = lo.fallback {
                        announce(crate::ap_lands::fallback_line(fb, radio_ft, l), Priority::Caution);
                        match fb {
                            crate::ap_lands::Fallback::OwnLaw(_) => {
                                // Our law from this tick: the bug, the wheel
                                // and (low) the arrest are ours again.
                                if let Some(o) = coupling.abandon_now(ap_modes) {
                                    if o.disarm {
                                        autoflight::press_logged(&xp, Button::Approach, "ap_lands_decouple").await;
                                    }
                                    cpl.own_lateral = true;
                                    cpl.own_vertical = true;
                                }
                            }
                            crate::ap_lands::Fallback::GoAround(_) => {
                                runtime.go_around.order(on_ground > 0.5);
                            }
                        }
                    }
                    if let Some(line) = lo.armed_line {
                        announce(line.into(), Priority::Advisory);
                    }
                    if ap_landing != before {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "ap_landing", "from": before.word(),
                                "to": ap_landing.word(), "why": lo.fallback.map(|f| f.why()),
                                "radio": radio_ft, "dist": g.dist_nm, "cmd_a": cmd_a_lit,
                                "cmd_b": cmd_b_lit, "toga": lo.press_toga}),
                        );
                    }
                    // The A/T holds what our governor would: the approach
                    // target for the flap out (`approach_target_kts`).
                    let flap_now = last_flap.unwrap_or(0.0);
                    let at_want = approach_target_kts(
                        g.target_ias_kts,
                        flap_now,
                        &crate::aircraft::flap_limits(),
                        crate::aircraft::config_floor_kts(flap_now),
                    );
                    let at_before = at_speed;
                    let ao = at_speed.step(crate::ap_lands::AtIn {
                        wanted: l.autothrottle
                            && ap_landing.live()
                            && coupling != crate::ils::Coupling::Off
                            && on_ground < 0.5
                            && !ga,
                        engaged: crate::ap_lands::at_engaged(f, r.speed_lamp),
                        window_kts: r.speed_window,
                        want_kts: at_want,
                        now_s,
                    });
                    if ao.arm_switch {
                        let armed = autoflight::ensure_switch(
                            &xp,
                            autoflight::Switch::AutothrottleArm,
                            true,
                            "ap_lands",
                        )
                        .await;
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "at_arm", "armed": armed}),
                        );
                    }
                    if let Some(w) = ao.set_window {
                        let _ = autoflight::set_dial_confirmed(
                            "autoland",
                            &xp,
                            Dial::Airspeed,
                            w,
                            &mut at_window_last,
                            1.0,
                        )
                        .await;
                    }
                    if ao.press_speed {
                        autoflight::press_logged(&xp, Button::Speed, "ap_lands").await;
                    }
                    if let Some(line) = ao.say {
                        announce(line.into(), Priority::Advisory);
                    }
                    if at_speed != at_before || ao.event.is_some() {
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "ap_at", "what": ao.event,
                                "from": at_before.word(), "to": at_speed.word(), "want": at_want,
                                "window": r.speed_window, "speed_lamp": r.speed_lamp,
                                "thrust_fma": r.thrust, "radio": radio_ft}),
                        );
                    }
                    crate::blackbox::record(
                        "autoland",
                        crate::ap_lands::tick_record(
                            l,
                            r,
                            f,
                            &ap_landing,
                            &at_speed,
                            coupling,
                            cmd_a_lit,
                            radio_ft,
                            ias,
                            Some(at_want),
                            on_ground > 0.5,
                            ap_rollout,
                        ),
                    );
                }
                let cpl = cpl;
                // The knob: once per phase or pattern leg and corrected past a
                // deadband (`bug_knob`), the final's in whole degrees down to
                // minimums; hand flown below them, the bug is left alone.
                let knob = bug_knob(g.phase, g.arresting || hand_flown, pattern.map(|(_, l, _)| l), missed);
                let knob_set = match (cpl.own_lateral, knob) {
                    (false, _) => {
                        bug_hold.reset();
                        None
                    }
                    (true, Some((key, db))) => {
                        // The pattern's, the reversal's, the join's and the
                        // missed approach's turns go through the one
                        // turn-safe steer (`heading_bug::bug_toward`, audit
                        // #6): never a bug on the far side of a turn in
                        // progress, a big turn led from level by at most
                        // 25 degrees. They had no reversal guard. The knob
                        // drive below does the stepping (30 a tick), so the
                        // steer's own slew is not used.
                        let want = match psi_r.as_ref().ok().copied() {
                            Some(psi)
                                if g.phase == ApproachPhase::Intercept
                                    || pattern.is_some()
                                    || missed.is_some() =>
                            {
                                let hdg_mag = true_to_mag(psi, var);
                                crate::heading_bug::bug_toward(
                                    bug_hold.set().unwrap_or(hdg_mag),
                                    mag_hdg,
                                    hdg_mag,
                                    phi_r.as_ref().ok().copied().unwrap_or(0.0),
                                    360.0,
                                )
                            }
                            // The final's heading in whole degrees, as set.
                            _ if g.phase == ApproachPhase::Final => mag_hdg.round(),
                            _ => mag_hdg,
                        };
                        bug_hold.decide(key, want, knob_t0.elapsed().as_secs_f64(), db)
                    }
                    (true, None) => {
                        bug_hold.reset();
                        None
                    }
                };
                // What the autopilot holds off the final's setting, once it
                // has settled on it (`FinalHeading`).
                if cpl.own_lateral && matches!(knob, Some((2, _))) {
                    final_heading.learn(
                        psi_r.as_ref().ok().copied(),
                        bug_hold.set().map(|m| mag_to_true(m, var)),
                        bug_hold
                            .since_write_s(knob_t0.elapsed().as_secs_f64())
                            .is_some_and(|s| s >= FINAL_SETTLE_S),
                        tick_dt,
                    );
                }
                // The mode lights and the yields come from the tick's one
                // read; the VS dial, the column, the ailerons and a fine bug
                // correction are QUEUED and go out with the lever in the
                // tick's one write round below. A yield is read as it always
                // was, only on a tick that writes its channel; the latch now
                // comes from the tick's read, a frame before this tick's
                // write, and the plugin drops the override on its own frame
                // either way (the write that follows a yield is ignored).
                let hdg_st = panel_tick.status(&st, F::HdgStatus);
                let vs_st = panel_tick.status(&st, F::VsStatus);
                // An ILS is intercepted LEVEL: with the approach armed and a live
                // glideslope above us, ALT holds and the glideslope captures
                // from below. No VS wheel (LEMG 13, 2026-10-11: it was rewritten
                // 270 times in 8 minutes, +524 fpm climbing to our own path
                // under the beam). A dead glideslope or a beam below us keeps
                // the path's VS.
                let level_for_gs = matches!(
                    coupling,
                    crate::ils::Coupling::Armed | crate::ils::Coupling::LocCaptured
                ) && !ap_modes.is_some_and(|m| m.gs())
                    && needles.is_some_and(|n| n.gs_valid && n.vdef.is_some_and(|v| v <= 0.0));
                if cpl.own_vertical && !level_for_gs && !arrest_hands {
                    autoflight::stage_dial(&mut writes, Dial::Vs, vs_dial);
                }
                let yield_r = match flare_elev {
                    Some(e) => {
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_PITCH, e);
                        st.get(F::PitchYield)
                    }
                    None => Ok(0.0),
                };
                let roll_yield_r = match flare_ail {
                    Some(a) => {
                        crate::monitor::note_crew_yoke();
                        writes.set(FLARE_ROLL, a);
                        st.get(F::RollYield)
                    }
                    None => Ok(0.0),
                };
                // The rudder rides the pedal channel the rollout steers on
                // (one writer: the segment ends at the wheels, where the
                // rollout takes it).
                let steer_yield_r = match flare_rud {
                    Some(r) => {
                        writes.set(TAXI_STEER, r);
                        st.get(F::SteerYield)
                    }
                    None => Ok(0.0),
                };
                if steer_yield_r.is_ok_and(|v| v > 0.5) && !steer_yielded {
                    steer_yielded = true;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "steer_yield", "radio": radio_ft}),
                    );
                }
                // A keyed setting (once per phase or leg) is driven and READ
                // BACK beside the write round.
                let bug_drive = knob.and(knob_set);
                // The pilot pulled or pushed: the plugin has already dropped
                // the override this frame. Stop commanding and let the dial
                // fly what is left — the VS path is still running underneath.
                if yield_r.is_ok_and(|v| v > 0.5) {
                    if !pitch_yielded {
                        pitch_yielded = true;
                        crate::flight::note_pilot_takeover("autoland: yoke");
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "pitch_yield", "radio": radio_ft}),
                        );
                        announce(
                            "You have the pitch. I'm holding the wings and the power.".into(),
                            Priority::Caution,
                        );
                    }
                }
                // Likewise the roll: the plugin dropped that override the
                // frame the hand moved, and the elevator keeps flying.
                if roll_yield_r.is_ok_and(|v| v > 0.5) && !roll_yielded {
                    roll_yielded = true;
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "roll_yield", "radio": radio_ft}),
                    );
                    announce(
                        "You have the roll. I'm on the elevator and the power.".into(),
                        Priority::Caution,
                    );
                }
                // Mode housekeeping is the autopilot's while it flies. With
                // the arrest on the channels a mode press is at best nothing
                // and at worst (an MCP that engages the master with the
                // mode) the servo back on the yoke mid-flare.
                // A coupled ILS's captured modes are the autopilot's:
                // re-engaging HDG or VS would drop the localizer or the
                // glideslope it just captured.
                // The owner turns the intent into presses (`autoflight::
                // reconcile`): HDG and V/S on a difference only, nothing on a
                // failed read, and never over what a coupled ILS hands the
                // airframe (#81: the old presses here dropped the 737's
                // captured localiser and glideslope at the next tick).
                // Never on the tick APR was pressed: its read is from before the
                // press. With the localiser needle alive the GFC 700's APR
                // captures at once, and HDG pressed on the stale read replaced
                // the capture, leaving the glideslope armed alone (LEMG 13,
                // 1791736342 and 1791741953; captured parked on the runway,
                // nav_status 2 the moment APR was pressed).
                if !arrest_hands && !appr_pressed {
                    use autoflight::{Lateral, ModeRead, Vertical, Want};
                    let coupled = matches!(
                        coupling,
                        crate::ils::Coupling::Armed
                            | crate::ils::Coupling::LocCaptured
                            | crate::ils::Coupling::Coupled
                    );
                    let want = Want {
                        lateral: Some(if coupled { Lateral::Coupled } else { Lateral::Heading }),
                        vertical: Some(if level_for_gs {
                            Vertical::Level
                        } else if coupled {
                            Vertical::Coupled
                        } else {
                            Vertical::Vs
                        }),
                    };
                    let presses = autoflight::reconcile(
                        want,
                        ModeRead {
                            hdg_status: hdg_st.as_ref().ok().copied(),
                            vs_status: vs_st.as_ref().ok().copied(),
                            modes: ap_modes,
                        },
                    );
                    autoflight::apply(&xp, &presses, "autoland").await;
                    if presses.contains(&Button::VerticalSpeed) {
                        // The engage syncs the dial to the CURRENT rate, so the
                        // rate we want has to be written again after it.
                        let _ = autoflight::set_dial(&xp, Dial::Vs, vs_dial).await;
                    }
                }

                // Speed: hold the approach speed with the SAME governor the copilot
                // uses for speed hold (speedhold::throttle_adjust) — a P+D law whose
                // derivative term adds power while the airspeed is still UNWINDING,
                // before the error grows, with extra authority on the slow side. The
                // old loop here was P-only (0.004/kt, no trend), so it only reacted
                // after speed was already gone and dribbled power in too slowly —
                // the nose came up holding level, speed bled to a stall, the AP quit
                // ("lose speed, lose AP", seen live at LERL/XLE000N). Profile-led flare power.
                let now = std::time::Instant::now();
                let dt_s = (now - prev_ias_t).as_secs_f64().clamp(0.1, 2.0);
                let trend_kts_s = prev_ias.map_or(0.0, |p| (ias - p) / dt_s);
                prev_ias = Some(ias);
                prev_ias_t = now;
                if missed.is_some() {
                    energy_throttle.reset();
                }
                let mut tecs_out: Option<crate::pilot::energy::Out> = None;
                let thr = if missed == Some(MissedStage::Climb) {
                    // The balked landing itself is firewall, not a governor
                    // holding approach speed — the LXGB miss flew its last 20
                    // seconds at the governor's idea of "on speed" with the
                    // ground coming up. ONLY the climb: full power belongs to
                    // the seconds of getting away from the runway, not to the
                    // minutes of repositioning that follow. Left firewalled
                    // across the whole missed approach (the first flight of the
                    // sticky version, Luton 2026-07-28) the MD-82 accelerated to
                    // 284 KIAS on the outbound leg and was still gaining.
                    // ...and only until the climb is ESTABLISHED: then it
                    // flies the go-around speed (`missed_approach_throttle`).
                    // EGNX 09 (2026-09-30): firewalled from 150 ft to the
                    // turn at 800 ft radio, the clean 737 went 250 to 309 kt,
                    // and 355 before the lever came back (#199).
                    let limits = crate::aircraft::flap_limits();
                    let flap = last_flap.unwrap_or(0.0);
                    let target = missed_approach_speed_kts(
                        approach_ias,
                        flap,
                        &limits,
                        crate::aircraft::config_floor_kts(GO_AROUND_FLAP),
                        crate::aircraft::config_floor_kts(flap),
                        alt,
                    );
                    Some(match thr_now {
                        Some(t) => missed_approach_throttle(
                            t,
                            ias,
                            trend_kts_s,
                            target,
                            radio_ft,
                            vvi,
                            crate::aircraft::approach_thrust_min(),
                        ),
                        None => 1.0,
                    })
                } else if g.phase == ApproachPhase::Flare && att_out.is_none() {
                    // A flare nobody hand-flies (the plant unread, the
                    // yoke the pilot's): idle, as it always was.
                    Some(0.0)
                } else {
                    // The target is only as safe as the flaps actually out
                    // (`config_floor_kts`), and the lever has a floor of its
                    // own on final and a ceiling-only rule low in a
                    // reposition (`approach_power_floor`).
                    let floor = crate::aircraft::config_floor_kts(last_flap.unwrap_or(0.0));
                    let target = approach_target_kts(
                        g.target_ias_kts,
                        last_flap.unwrap_or(0.0),
                        &crate::aircraft::flap_limits(),
                        floor,
                    );
                    // The power on total energy (TECS's throttle half,
                    // `pilot::energy`): the shared governor chased the
                    // phugoid in ALT hold (LEMG 13, 2026-10-11). Hand flown,
                    // the path the attitude law asks and, below its gate,
                    // the flare: the lever only back, at idle with the
                    // engine spooled down as the wheels arrive. No true
                    // vertical speed read: the lever stays.
                    let flaring = att_out.is_some_and(|o| o.flaring);
                    let out = match (thr_now, plant_at(3)) {
                        (Some(t), Some(vy_ms)) => Some(energy_throttle.step(
                            crate::pilot::energy::In {
                                ias_kts: ias,
                                accel_kts_s: trend_kts_s,
                                vs_fpm: vy_ms * 196.850_394,
                                target_kts: target,
                                floor_kts: floor,
                                // The path paid for is the one the autopilot
                                // FLIES, by its own mode: a captured glidepath
                                // at the path's rate, else our dial. The
                                // guidance's VS read 0 under a -500 dial (the
                                // RV-10 held 98 kt against 80, 1791803684); the
                                // dial read -150 under a captured -400 path (the
                                // 172 ran to 80 kt against 65, 1791806643).
                                // Hand flown, the path the attitude law asks.
                                path_vs_fpm: if let Some(o) = att_out {
                                    o.vs_target_fpm
                                } else if level_for_gs {
                                    0.0
                                } else if ap_modes.is_some_and(|m| m.gs()) {
                                    -(gs_kts / 60.0) * gp_ft_per_nm()
                                } else {
                                    vs_dial
                                },
                                thr_now: t,
                                dt: dt_s,
                                flare_to_wheels_s: att_out.filter(|o| o.flaring).map(|o| o.to_wheels_s),
                            },
                            pilot_airframe.tecs,
                        )),
                        _ => None,
                    };
                    tecs_out = out;
                    let min_thr = if flaring {
                        0.0
                    } else {
                        approach_power_floor(
                            g.phase,
                            radio_ft,
                            target - ias,
                            crate::aircraft::approach_thrust_min(),
                            last_flap.is_some_and(|f| f >= LANDING_FLAP_RATIO),
                        )
                        .max(climb_power_floor(
                            g.phase,
                            g.target_vs_fpm,
                            crate::aircraft::approach_thrust_min(),
                        ))
                    };
                    // In the autopilot's roundout the lever only ever comes
                    // back: the speed law read the roundout's own
                    // deceleration as "five knots slow" and walked the 737's
                    // lever from 0.32 to 0.55 at 30 ft over EGCC 23L, and it
                    // climbed away from two feet at 155 kt (2026-09-19).
                    match (out, thr_now) {
                        (Some(o), Some(t)) if g.arresting && att_out.is_none() => Some(o.lever.min(t).clamp(min_thr, 1.0)),
                        (Some(o), _) => Some(o.lever.clamp(min_thr, 1.0)),
                        (None, t) => t.map(|t| t.clamp(min_thr, 1.0)),
                    }
                };
                // The A/T holds the speed and retards (`ap_lands`): our
                // lever stands aside.
                let thr = if at_speed.flies() { None } else { thr };
                if thr.is_some() {
                    last_lever = thr;
                }
                // The tick's ONE write round: the lever, the VS dial, the
                // column, the ailerons and the bug, together (they were up to
                // four rounds, 2026-09-30), a keyed bug drive beside it.
                if let Some(throttle) = thr {
                    writes.set(&thr_name, throttle);
                }
                let (done, knob_read) = tokio::join!(writes.flush(&xp), async {
                    match bug_drive {
                        Some(h) => {
                            Some(crate::tools::autopilot::drive_heading_bug_step(&xp, h, 30.0).await)
                        }
                        None => None,
                    }
                });
                if let (Some((key, _)), Some(h)) = (knob, knob_set) {
                    bug_hold.took(knob_read);
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "heading_set", "key": key, "hdg": h,
                            "read": knob_read, "phase": format!("{:?}", g.phase),
                            "radio": radio_ft}),
                    );
                }
                let throttle_written = thr.map(|_| done.ok(&thr_name) == Some(true));
                // Whether the lever TOOK the last tick's write (an add-on
                // re-driving it is caught this way): this tick's read against
                // that command. It was a second round trip after the write
                // (`throttle_after`), skipped in the flare; the next tick's
                // read says the same thing a tick later, for no round.
                let lever_took = last_thr_written
                    .zip(thr_now)
                    .map(|(c, r)| (c - r).abs() < LEVER_TOOK_TOL);
                last_thr_written = if throttle_written == Some(true) { thr } else { None };
                crate::blackbox::record("autoland", serde_json::json!({
                    "event": "power", "phase": format!("{:?}", g.phase),
                    "throttle_before": thr_now, "throttle_command": thr,
                    "lever_took": lever_took, "write_ok": throttle_written, "dt": dt_s,
                    // TECS's state (`pilot::energy`): the flare's retard,
                    // the underspeed and bad-descent protections.
                    "tecs_flare": tecs_out.map(|o| o.flare),
                    "tecs_underspeed": tecs_out.map(|o| o.underspeed),
                    "tecs_bad_descent": tecs_out.map(|o| o.bad_descent),
                    "vvi": vvi, "radio": radio_ft, "ias": ias,
                }));

                // Config: gear down inside 6 nm; flaps by distance AND vertical
                // state (full only once on the path — see [`flap_schedule`]).
                // Both wait for the speed the placard allows (the profile's
                // VLO and per-detent Vfe): a fast handoff slows first and
                // configures as the limits open up. Written only when the
                // target changes, so the setting isn't re-asserted against the
                // pilot every tick.
                // ...or with the plan's half flap: its deceleration was fitted
                // with the gear down from there (`approach_energy`).
                let plan_flap = energy.and_then(|(a, ..)| a.flap);
                if !gear_down
                    && !outbound
                    && (g.dist_nm < 6.0 || plan_flap.is_some_and(|f| f >= 0.5 - 1e-6))
                    && gear_may_extend(ias, crate::aircraft::gear_limit_kts())
                {
                    let _ = crate::tools::gear::set_handle(&xp, false).await;
                    gear_down = true;
                }
                // The landing's own checklist items, with the gear: ground
                // spoilers armed and the autobrake, where the airframe has
                // them. Asked again while the descent still has the
                // speedbrake out. (The Monitor list named both on every A330
                // flight; nothing had ever set them.)
                if !landing_config_done && g.dist_nm < 6.0 && !ga {
                    let cfg = crate::takeoff_config::set_for_landing(&xp).await;
                    landing_config_done = !cfg.deferred;
                    if landing_config_done {
                        LANDING_CONFIGURED.store(true, Ordering::SeqCst);
                    }
                    if let Some(line) = crate::takeoff_config::line("Landing config", &cfg) {
                        announce(line, Priority::Advisory);
                    }
                }
                // Go-around flap: half, whatever the schedule says — full flap
                // won't climb, and clean this low gives the sink away.
                // A CLOSE downwind is flown at half flap too. `outbound` also
                // names a distant reversal/reposition, which stays clean until
                // it enters the normal six-mile configuration area (#211).
                // ...once the speed flies it (`go_around_flap_due`): the
                // half flap's floor, not the full flap's.
                let wanted = if ga
                    && last_flap.is_some_and(|f| f > 0.5 + 1e-6)
                    && !go_around_flap_due(ias, crate::aircraft::config_floor_kts(0.5))
                {
                    last_flap.unwrap_or(0.5)
                } else if ga {
                    0.5
                } else {
                    let sched = approach_leg_flap_schedule(
                        g.dist_nm,
                        alt - g.glidepath_alt_ft,
                        outbound,
                    );
                    // The plan's configuration for its speed, never less
                    // than the schedule's; the landing flap only on or above
                    // the path (the LERL energy trap, `flap_schedule`).
                    let sched = plan_flap.map_or(sched, |f| {
                        plan_flap_wanted(f, alt - g.glidepath_alt_ft).max(sched)
                    });
                    approach_flap_wanted(sched, last_flap)
                };
                let limits = crate::aircraft::flap_limits();
                let flap = flap_kept_flying(
                    flap_held_within_limits(wanted, ias, &limits, last_flap),
                    last_flap,
                    ias,
                    &limits,
                    ga,
                    retract_speed_kts,
                );
                if last_flap != Some(flap) {
                    last_flap = Some(flap);
                    let _ = crate::tools::flaps::set_handle(&xp, flap).await;
                }

                // Watchdog: the AP not FLYING the commanded vertical speed is how
                // the LEMI approach dove 2,300 fpm into terrain with vs_cmd=0 and
                // no callout. If actual VS diverges hard for ~5 s, re-arm the VS
                // mode and tell the pilot — this is exactly when a hand on the
                // yoke matters.
                // Level under a live glideslope is held in ALT: judged against
                // level, and taken back to ALT, never to a VS the intercept
                // does not want (the first level intercept had the two pressing
                // against each other, 2026-10-11).
                // ALT returning to its altitude climbs or descends to it (+900
                // fpm after the setup's dive, 2026-10-11): that is holding, and
                // only a rate past `LEVEL_RETURN_MAX_FPM` is a divergence.
                let held_vs = if level_for_gs { 0.0 } else { g.target_vs_fpm };
                let vs_err = vvi.map(|v| {
                    if level_for_gs && v.abs() <= LEVEL_RETURN_MAX_FPM { 0.0 } else { v - held_vs }
                });
                // ...and never inside the arrest, where the autopilot has
                // no vertical to hold: a watchdog re-engaging the servos at
                // fifty feet would be the fight this change removes.
                // ...nor before the engage is confirmed: `ApEngage` owns the
                // press until then, and the aircraft is held level meanwhile.
                let not_held = g.phase != ApproachPhase::Flare
                    && !hand_flown
                    && engage.confirmed()
                    // A captured glideslope is the airframe's vertical, not our dial.
                    && cpl.own_vertical
                    && vs_err.is_some_and(|e| vs_not_held(e, prev_vs_err, tick_dt));
                prev_vs_err = vs_err;
                if not_held {
                    let was_s = vs_diverged_s;
                    vs_diverged_s += tick_dt;
                    if was_s < VS_DIVERGED_WARN_S && vs_diverged_s >= VS_DIVERGED_WARN_S {
                        // The mode AND the master: an AP kicked off by the
                        // airframe's own protection (the A330's high-speed
                        // law at Vmo) answers to neither a dial nor a mode
                        // command until the servos are back on.
                        // FD first (`ap_engage`): the watchdog's own clock
                        // judges whether it held.
                        crate::ap_engage::press_logged(
                            &xp, &cfg.fd_commands, "autoland", "vs_watchdog", 1,
                        )
                        .await;
                        // ALT pressed while holding toggles it off and the
                        // re-engage takes a new altitude (2,950 to 2,870 ft,
                        // 2026-10-11): an engaged ALT is only warned about.
                        if !level_for_gs {
                            autoflight::press_logged(&xp, Button::VerticalSpeed, "autoland_vs_watchdog").await;
                        } else if !ap_modes.is_some_and(|m| m.alt_hold()) {
                            autoflight::press_logged(&xp, Button::AltitudeHold, "autoland_vs_watchdog").await;
                        }
                        announce(vs_not_held_line(held_vs).into(), Priority::Warning);
                    }
                    // Said once and then flown for twenty-two minutes: the
                    // A330 climbed from 1,900 ft to FL350 on a -1,200 fpm
                    // dial with the loop still "on approach" and the aircraft
                    // 90 nm from the runway (EGPH 06, 2026-09-20). A warning
                    // that changes nothing is not a watchdog. Give up with a
                    // flyable aircraft and say what is held.
                    if vs_diverged_s >= VS_DIVERGED_GIVE_UP_S {
                        gave_up = true;
                        let thr_dref = throttle_dataref();
                        let _ = tokio::join!(
                            xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                            autoflight::set_dial(&xp, Dial::Vs, 0.0),
                        );
                        announce(
                            "Autopilot is still not holding the descent. Abandoning the approach. \
                             Your aircraft — the vertical speed dial is zero and the heading bug is set."
                                .into(),
                            Priority::Warning,
                        );
                        crate::blackbox::record(
                            "autoland",
                            serde_json::json!({"event": "give_up", "reason": "vs_divergence",
                                               "diverged_s": vs_diverged_s, "vvi": vvi, "cmd_vs": g.target_vs_fpm}),
                        );
                        break;
                    }
                } else {
                    vs_diverged_s = 0.0;
                }
                // Flying AWAY from the runway is not an approach. The same
                // sortie: an arm 3.6 nm from the threshold, then 90 nm out and
                // opening, phase still Intercept. A missed approach flies
                // outbound by design; anything else past the arm distance plus
                // a margin is handed back before it becomes a diversion.
                // Measured from as far out as this approach has legitimately
                // been, never while a pattern is flying its downwind: armed
                // over the runway in a go-around (0.9 nm) the limit was 5.9 nm,
                // a 737's turn back needs 6.5, and the guard fired on the tick
                // the turn inbound began (EGTE, 2026-09-21).
                if missed.is_none()
                    && reversal.is_none()
                    && hold.is_none()
                    && g.dist_nm > recede_ref_nm + RECEDING_GIVE_UP_NM
                {
                    gave_up = true;
                    let thr_dref = throttle_dataref();
                    let _ = tokio::join!(
                        xp.set_float(&thr_dref, GIVE_UP_THROTTLE),
                        autoflight::set_dial(&xp, Dial::Vs, 0.0),
                    );
                    announce(
                        format!(
                            "We're {:.0} miles from runway {} and opening. Abandoning the approach. \
                             Your aircraft — the vertical speed dial is zero.",
                            g.dist_nm,
                            crate::crew_radio::spell_runway(&rwy.id)
                        ),
                        Priority::Warning,
                    );
                    crate::blackbox::record(
                        "autoland",
                        serde_json::json!({"event": "give_up", "reason": "receding",
                                           "dist_nm": g.dist_nm, "armed_at_nm": rwy.dist_nm,
                                           "ref_nm": recede_ref_nm}),
                    );
                    break;
                }

                if last_phase != Some(g.phase) {
                    let prev = last_phase;
                    last_phase = Some(g.phase);
                    // The landing light for the landing, at the final: the
                    // exterior-lights rule lights it lined up and puts it out
                    // at the vacate, and nothing lit it for the ARRIVAL. The
                    // BEFORE LANDING cross-check read it off on the LEMG 31
                    // final (2026-09-12).
                    if g.phase == ApproachPhase::Final {
                        let _ = crate::tools::lights::landing(&xp, true).await;
                    }
                    let line = match g.phase {
                        // Dropping back to Intercept from Final/Flare is a
                        // go-around, not a fresh intercept — say so. Low it's
                        // the latch flying (straight ahead); high it's the
                        // normal turn back to re-intercept.
                        ApproachPhase::Intercept
                            if matches!(prev, Some(ApproachPhase::Final | ApproachPhase::Flare)) =>
                        {
                            if ga {
                                // Said at once: the power is coming and the
                                // pilot must know why. Corrected if the
                                // wheels take it (`balk_landed`).
                                runtime.spoke_going_around = true;
                                "Going around. Full power, climbing straight ahead."
                            } else {
                                dropped_final_line(
                                    along_nm,
                                    reversal_began,
                                    alt - g.glidepath_alt_ft,
                                )
                                .unwrap_or("")
                            }
                        }
                        ApproachPhase::Intercept => "Intercepting the final approach.",
                        ApproachPhase::Final => "On final. Gear and flaps as the speed allows.",
                        ApproachPhase::Flare => "Over the threshold, flaring, easing the power back.",
                    };
                    // A go-around the stabilised gate ordered says why, and
                    // is a hazard (the EFB's caution); the rest announce.
                    let (line, unstable) = match runtime.stable_said.take() {
                        Some(why) if ga && line.starts_with("Going around") => {
                            (format!("Not stabilised, {why}. Going around."), true)
                        }
                        _ => (line.to_string(), false),
                    };
                    if unstable {
                        tracing::info!(target: "xflightcrew::autoland", spoke = %line, "copilot");
                        if let Some(v) = &voice {
                            v.hazard(crate::monitor::Severity::Caution, line);
                        }
                    } else if !line.is_empty() {
                        announce(line, Priority::Caution);
                    }
                }

                // (Touchdown + rollout-to-stop is handled at the top of the loop,
                // latched by `rolling_out`, before the approach guidance.)
                // The flare's tick from minimums: the segment is hand flown.
                pacer.wait(std::time::Duration::from_millis(tick_ms(last_phase, g.arresting || arrest_hands))).await;
            }
            // A newer loop (the clearance re-arm above, or any later arm) has
            // already set ACTIVE for itself; clearing it here would report
            // "landed or gone around" to whoever waits on it.
            // "YOUR AIRCRAFT" WITH A SPEED HELD. Every give-up wrote a flat
            // 0.75 lever for the pilot to find: right for a piston dragging
            // gear and flap, and on a level 737 it is an acceleration with no
            // end (152 kt at the hand-back, 240 forty seconds later, 346 and
            // ten overspeed warnings before anyone had the picture, EGTE
            // 2026-09-21). The lever write stays as the first second's power;
            // the governor then holds a speed the configuration allows, and
            // "my controls" or a spoken speed takes it from there.
            if gave_up && lease.current() && !xp.cancelled() {
                let hold = hand_back_speed_kts(
                    approach_ias,
                    last_flap.unwrap_or(0.0),
                    &crate::aircraft::flap_limits(),
                );
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "hand_back", "speed_kts": hold, "flap": last_flap}),
                );
                announce(format!("I'm holding {hold:.0} knots for you."), Priority::Advisory);
                crate::tools::speedhold::arm_flight(base_url.clone(), voice.clone(), hold);
            }
            // The flag, the panel and the landing configuration go with the
            // run's lease when the thread ends (a newer loop's are its own).
        });
    });
}
