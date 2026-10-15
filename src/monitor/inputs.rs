//! The tick's inputs: the fuel tanks, the checks built for the airframe, the
//! reads, the crew's yoke and brake, the interventions and the world's gates.

use super::*;

/// The tank picture for the centre-pump check: fuel and pumps every tick
/// (batched), the lateral positions once per airframe. All or nothing; a
/// declared tank the geometry says is a main tank is logged, never used.
pub(super) async fn read_fuel_tanks(
    xp: &XPlaneClient,
    declared: &[usize],
    lateral: &mut Option<Vec<f64>>,
) -> Option<crate::tools::fuel_pumps::Tanks> {
    use crate::tools::fuel_pumps as fp;
    if lateral.is_none() {
        let x = xp
            .get_float_array(fp::TANK_LATERAL_M)
            .await
            .ok()
            .filter(|v| !v.is_empty())?;
        let bad = fp::rejected(declared, &x);
        if !bad.is_empty() {
            tracing::warn!(?bad, "fuel pumps: declared tanks are not centre tanks; ignored");
            crate::blackbox::record(
                "fuel_pumps",
                serde_json::json!({"event": "rejected", "tanks": bad, "lateral_m": &x}),
            );
        }
        *lateral = Some(x);
    }
    let (kg, pumps) = tokio::join!(
        xp.get_float_array(fp::TANK_FUEL_KG),
        xp.get_float_array(fp::TANK_PUMP_ON),
    );
    Some(fp::Tanks {
        kg: kg.ok().filter(|v| !v.is_empty())?,
        pumps: pumps.ok().filter(|v| !v.is_empty())?,
        lateral_m: lateral.clone()?,
    })
}

pub(super) fn build_checks(profile: &AircraftProfile) -> Vec<Box<dyn Check>> {
    let mut checks: Vec<Box<dyn Check>> = vec![
        // Universal: every airframe can roll with the brake set.
        Box::new(RollingOnParkBrake),
        // Universal terrain guard: any airframe can dive into the ground.
        Box::new(SinkRate),
        // Descent advisor — armed only when a flight plan gave us a destination
        // (the checks read `Snapshot::dest_*`, `None` without one).
        Box::new(TodAhead::default()),
        Box::new(TodPassed),
        Box::new(HighOnProfile),
        // Universal: every altimeter can be mis-set, and every automation here
        // flies the indicated altitude off it.
        Box::new(AltimeterSetting {
            transition_altitude_ft: profile.transition_altitude_ft,
            crossing: false,
        }),
        // ...and AT the transition, crossed in the direction flown (#156).
        Box::new(AltimeterSetting {
            transition_altitude_ft: profile.transition_altitude_ft,
            crossing: true,
        }),
        // Universal: the altimeter itself against the true height (#94).
        Box::new(AltimeterTruthCheck {
            transition_altitude_ft: profile.transition_altitude_ft,
        }),
        // Universal: the tool finds what anti-ice is fitted (#100).
        Box::new(Icing),
    ];
    // Centre tank pumps: only where the profile declares which tanks.
    if !profile.fuel_tank_pumps_off_when_empty.is_empty() {
        checks.push(Box::new(CentreTankDry {
            declared: profile.fuel_tank_pumps_off_when_empty.clone(),
        }));
    }
    // Step climb: only on an airframe whose profile declares an optimum
    // altitude, which is jets. A piston goes as high as it can and stays.
    if let Some(r) = profile.reference.clone() {
        if r.optimum_alt_ft.is_some() {
            checks.push(Box::new(StepClimb { reference: r }));
        }
    }

    // Phase-expectation advisor rows from the profile's SOP table.
    for (index, item) in profile.advisor.iter().enumerate() {
        let phases: Vec<Phase> = item.phases.iter().filter_map(|p| parse_phase(p)).collect();
        if phases.len() != item.phases.len() {
            tracing::warn!(dataref = %item.dataref, "advisor item has unknown phase names; skipping");
            continue;
        }
        checks.push(Box::new(PhaseExpectation {
            // Leaked once per profile build (a handful of tiny strings) so the
            // id can be `&'static str` like every other check's.
            id: Box::leak(format!("advisor_{index}_{}", profile.id).into_boxed_str()),
            index,
            expect_on: item.expect_on,
            phases,
            say: item.say.clone(),
            command: item.command.clone(),
        }));
    }
    // Altitude capture watchdog — needs an AP that flies a dialed altitude.
    if profile.autopilot {
        checks.push(Box::new(AltitudeBust));
    }

    // Hypoxia — applies to every airframe (cabin altitude tracks real altitude when
    // unpressurized). Disjoint bands so only one fires at a time.
    checks.push(Box::new(CabinAltitude {
        id: "cabin_alt_caution",
        severity: Severity::Caution,
        band_ft: (10_000.0, Some(12_500.0)),
        message: cabin_message(profile.pressurised, false),
        min_dur: Duration::from_secs(10),
        above_low: Default::default(),
    }));
    checks.push(Box::new(CabinAltitude {
        id: "cabin_alt_warning",
        severity: Severity::Warning,
        band_ft: (12_500.0, None),
        message: cabin_message(profile.pressurised, true),
        min_dur: Duration::from_secs(4),
        above_low: Default::default(),
    }));
    if !profile.electrical.is_empty() {
        // Charging source is a "generator" on a turbine (King Air PT6), an
        // "alternator" on a piston — a King Air has no alternator to reference.
        let noun = if profile.turboprop_start.is_some() || profile.turbine_start {
            "Generator"
        } else {
            "Alternator"
        };
        checks.push(Box::new(AlternatorOffline { noun }));
        // Switch-on-but-not-charging, caught via the ammeter.
        checks.push(Box::new(AlternatorNotCharging { noun }));
    }
    if profile.directional_gyro {
        // The vacuum system drives the DG/AI, so an airframe with a gyro has
        // one. (Gyro DRIFT is no longer a check: the monitor loop syncs the
        // DG itself and announces it — a copilot's hand, not a nag.)
        checks.push(Box::new(LowVacuum));
    }
    if profile.carb_heat {
        checks.push(Box::new(CarbIce));
    }
    if profile.engine {
        // Engine instruments, calibrated to the 172's gauges (verified live):
        // OIL TEMP green to 245°F redline; OIL PRESS 0–115 PSI; CHT to 500°F. Oil
        // temp and CHT datarefs read °F despite their "_deg_C" names. min_dur skips
        // post-start build-up and cold-start spikes.
        let secs = Duration::from_secs;
        // Oil pressure (PSI): idle legitimately sits well below the 50–90 green
        // band, so only the genuinely-dangerous extremes alarm — below the 20 PSI
        // redline (warning), and above the ~115 PSI redline (caution then warning).
        checks.push(Box::new(EngineHealth {
            id: "oil_press_low",
            severity: Severity::Warning,
            param: EngineParam::OilPressure,
            low_side: true,
            trip: 20.0,
            clear: None,
            min_dur: secs(4),
        }));
        checks.push(Box::new(EngineHealth {
            id: "oil_press_high_caution",
            severity: Severity::Caution,
            param: EngineParam::OilPressure,
            low_side: false,
            trip: 105.0,
            clear: Some(115.0),
            min_dur: secs(4),
        }));
        checks.push(Box::new(EngineHealth {
            id: "oil_press_high_warn",
            severity: Severity::Warning,
            param: EngineParam::OilPressure,
            low_side: false,
            trip: 115.0,
            clear: None,
            min_dur: secs(3),
        }));
        // Oil temperature (°F): caution band 240–245°F, warning at/over the 245°F redline.
        checks.push(Box::new(EngineHealth {
            id: "oil_temp_caution",
            severity: Severity::Caution,
            param: EngineParam::OilTemp,
            low_side: false,
            trip: 240.0,
            clear: Some(245.0),
            min_dur: secs(5),
        }));
        checks.push(Box::new(EngineHealth {
            id: "oil_temp_warn",
            severity: Severity::Warning,
            param: EngineParam::OilTemp,
            low_side: false,
            trip: 245.0,
            clear: None,
            min_dur: secs(3),
        }));
        // CHT (°F): caution from ~450°F, warning at the 500°F redline.
        checks.push(Box::new(EngineHealth {
            id: "cht_caution",
            severity: Severity::Caution,
            param: EngineParam::Cht,
            low_side: false,
            trip: 450.0,
            clear: Some(500.0),
            min_dur: secs(5),
        }));
        checks.push(Box::new(EngineHealth {
            id: "cht_warn",
            severity: Severity::Warning,
            param: EngineParam::Cht,
            low_side: false,
            trip: 500.0,
            clear: None,
            min_dur: secs(3),
        }));
    }
    if let Some(vs) = profile.vs_clean_kts {
        // Wide caution band (≤1.2×Vs) so it speaks with lead time, urgent prompt
        // below ~1.1×Vs. Disjoint bands so only one speaks; the factors are
        // applied to the *flap-adjusted* stall speed at eval time, so a slow but
        // safe flaps-down approach doesn't trip it.
        let vs0 = profile.reference.as_ref().and_then(|r| r.vs0_kts);
        let detent_stall = crate::aircraft::detent_stall_points(profile.flaps.as_ref());
        let stall_ref_weight_kg = profile.reference.as_ref().and_then(|r| r.stall_ref_weight_kg);
        checks.push(Box::new(StallSpeed {
            id: "airspeed_caution",
            severity: Severity::Caution,
            vs_clean_kts: vs,
            vs0_kts: vs0,
            detent_stall: detent_stall.clone(),
            stall_ref_weight_kg,
            at_or_below_factor: crate::aircraft::CAUTION_HAND_RATIO,
            above_factor: Some(crate::aircraft::STALL_WARNING_RATIO),
        }));
        checks.push(Box::new(StallSpeed {
            id: "airspeed_warning",
            severity: Severity::Warning,
            vs_clean_kts: vs,
            vs0_kts: vs0,
            detent_stall,
            stall_ref_weight_kg,
            at_or_below_factor: crate::aircraft::STALL_WARNING_RATIO,
            above_factor: None,
        }));
    }
    if let Some(vne) = profile.vne_kts {
        // Caution from ~0.90×Vne up to redline (leaves margin below in normal fast
        // cruise/descent), urgent prompt at or above Vne. Disjoint bands so only
        // one speaks.
        checks.push(Box::new(Overspeed {
            id: "overspeed_caution",
            severity: Severity::Caution,
            vne_kts: vne,
            vmo_high: profile.vmo_high,
            at_or_above_factor: OVERSPEED_CAUTION_FACTOR,
            below_factor: Some(1.0),
        }));
        checks.push(Box::new(Overspeed {
            id: "overspeed_warning",
            severity: Severity::Warning,
            vne_kts: vne,
            vmo_high: profile.vmo_high,
            at_or_above_factor: 1.0,
            below_factor: None,
        }));
    }
    if let Some(fuel) = &profile.fuel {
        // Two banded checks share one struct: caution between the thresholds,
        // warning below the lower one. Disjoint bands so only one speaks at a
        // time, and each gets its own fixed severity (the Check trait's model).
        checks.push(Box::new(FuelEndurance {
            id: "fuel_caution",
            severity: Severity::Caution,
            at_or_below_min: fuel.reserve_caution_min,
            above_min: Some(fuel.reserve_warning_min),
            cruise_burn_kg_hr: Some(fuel.cruise_burn_kg_hr).filter(|b| *b > 0.0),
            entered: Default::default(),
        }));
        checks.push(Box::new(FuelEndurance {
            id: "fuel_warning",
            severity: Severity::Warning,
            at_or_below_min: fuel.reserve_warning_min,
            above_min: None,
            cruise_burn_kg_hr: Some(fuel.cruise_burn_kg_hr).filter(|b| *b > 0.0),
            entered: Default::default(),
        }));
        checks.push(Box::new(FuelPreTakeoff {
            caution_min: fuel.reserve_caution_min,
            cruise_burn_kg_hr: fuel.cruise_burn_kg_hr,
        }));
    }
    // Selector-equipped airframes only (the GA singles driving the sim's
    // fuel_tank_selector) — a custom fuel system (cycle selectors, xlua-managed)
    // can legitimately leave the sim selector at 0.
    if profile.fuel_selector.is_some() {
        checks.push(Box::new(FuelSelectorOff));
    }
    if profile.pitch_trim {
        checks.push(Box::new(StabTrim));
    }
    if let Some(flaps) = &profile.flaps {
        let limits: Vec<(f64, Option<f64>)> = flaps
            .detents
            .iter()
            .filter(|d| d.vfe_kts.is_some())
            .map(|d| (d.ratio, d.vfe_kts))
            .collect();
        if !limits.is_empty() {
            checks.push(Box::new(FlapOverspeed { limits }));
        }
    }
    checks
}

/// Run the monitor loop until the process exits. Spawned on its own thread +
/// runtime with its own X-Plane client; shares the [`Voice`] arbiter.
/// The plugin panel's "mute monitor callouts" setting, refreshed each tick from
/// its dataref. Checked by every callout alongside the copilot-is-flying gate.
pub(super) static PANEL_MUTE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// The live flight strip shown on the plugin panel: the active emergency line
/// (if any), then cabin altitude airborne, then fuel endurance. " | " separators
/// (ASCII-safe for the panel font). Empty hides the row.
/// Round to 0.1 for compact black-box records.
/// The pilot's controls, the levers and the conditions, for the `state`
/// record's `in` object. A failed read is `None`, never a number.
#[derive(Debug, Clone, Default)]
pub(super) struct Inputs {
    pub(super) throttle: Option<f64>,
    pub(super) beta: Option<f64>,
    pub(super) brake_l: Option<f64>,
    pub(super) brake_r: Option<f64>,
    pub(super) yoke_pitch: Option<f64>,
    pub(super) yoke_roll: Option<f64>,
    pub(super) yoke_yaw: Option<f64>,
    pub(super) ap_on: Option<f64>,
    pub(super) ap_state: Option<f64>,
    pub(super) wind_dir: Option<f64>,
    pub(super) wind_kts: Option<f64>,
    pub(super) sim_speed: Option<f64>,
    pub(super) trq: Option<f64>,
    pub(super) steer_yield: Option<f64>,
    pub(super) pitch_yield: Option<f64>,
    pub(super) brake_yield: Option<f64>,
    /// A crew loop's own brake command is in flight (held, or inside its
    /// expiry and decay tail, `tools::brake_cmd_quiet`): the brake the
    /// plugin sees and the parking-brake ratio are OURS this tick.
    pub(super) crew_brake: bool,
    /// The pitch channel to the tail, read airborne while the servos or our
    /// own column fly it (#181: the servo moved nothing and nothing logged
    /// it); `None` = not read this tick.
    pub(super) pitch: Option<PitchInputs>,
}

/// The `state` record's `in.pitch`: the autopilot's servo, the elevator
/// SURFACE (degrees, trailing edge down positive), the summed column, the
/// trim, the dials the autopilot flies and who overrides the pitch. A failed
/// read is `None`.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct PitchInputs {
    pub(super) servo: Option<f64>,
    pub(super) elev_deg: Option<f64>,
    pub(super) total: Option<f64>,
    pub(super) trim: Option<f64>,
    pub(super) vs_dial: Option<f64>,
    pub(super) alt_hold_ft: Option<f64>,
    pub(super) ovr_pitch: Option<f64>,
    pub(super) ovr_fd_pitch: Option<f64>,
}

impl PitchInputs {
    pub(super) fn json(&self) -> serde_json::Value {
        let r3 = |v: Option<f64>| v.map(|x| (x * 1000.0).round() / 1000.0);
        serde_json::json!({
            "servo": r3(self.servo), "elev": self.elev_deg.map(|x| (x * 100.0).round() / 100.0),
            "tot": r3(self.total), "trim": r3(self.trim),
            "vs_dial": self.vs_dial.map(r1), "alt_hold": self.alt_hold_ft.map(r1),
            "ovr": self.ovr_pitch.map(|v| v > 0.5), "ovr_fd": self.ovr_fd_pitch.map(|v| v > 0.5),
        })
    }

    pub(super) fn telemetry(&self) -> crate::climb_watch::PitchTelemetry {
        crate::climb_watch::PitchTelemetry {
            servo: self.servo,
            elev_deg: self.elev_deg,
            total: self.total,
            ours: self.ovr_pitch.is_some_and(|v| v > 0.5),
        }
    }
}

/// Is the pitch channel worth its eight reads this tick? Airborne, with the
/// autopilot's servos in or our own column fresh: the Web API's load is the
/// sim's frame time. Pure.
pub(super) fn pitch_reads_due(on_ground: bool, servos_on: bool, crew_yoke: bool) -> bool {
    !on_ground && (servos_on || crew_yoke)
}

impl Inputs {
    pub(super) fn json(&self) -> serde_json::Value {
        let r2 = |v: Option<f64>| v.map(|x| (x * 100.0).round() / 100.0);
        serde_json::json!({
            "thr": r2(self.throttle), "beta": r2(self.beta),
            "bl": r2(self.brake_l), "br": r2(self.brake_r),
            "yp": r2(self.yoke_pitch), "yr": r2(self.yoke_roll), "yy": r2(self.yoke_yaw),
            "ap": self.ap_on.map(|v| v > 0.5), "ap_state": self.ap_state.map(|v| v as i64),
            "wind": [self.wind_dir.map(r1), self.wind_kts.map(r1)],
            "sim_speed": self.sim_speed, "trq": self.trq.map(r1),
            "yield": [self.steer_yield.map(|v| v > 0.5), self.pitch_yield.map(|v| v > 0.5),
                      self.brake_yield.map(|v| v > 0.5)],
            "crew_brake": self.crew_brake,
            "pitch": self.pitch.map(|p| p.json()),
        })
    }
}

/// One join for the pitch channel's eight reads.
pub(super) async fn read_pitch(xp: &crate::xplane::XPlaneClient) -> PitchInputs {
    use crate::datarefs as d;
    let ok = |r: anyhow::Result<f64>| r.ok().filter(|v| v.is_finite());
    let (servo, elev, total, trim, vs_dial, alt_hold, ovr, ovr_fd) = tokio::join!(
        xp.get_float(d::SERVO_PITCH_RATIO),
        xp.get_float_array(d::ELEVATOR1_DEG),
        xp.get_float(d::TOTAL_PITCH_RATIO),
        xp.get_float(d::ELEVATOR_TRIM),
        xp.get_float(d::VVI_DIAL_FPM),
        xp.get_float(d::ALTITUDE_HOLD_FT),
        xp.get_float(d::OVERRIDE_JOYSTICK_PITCH),
        xp.get_float(d::OVERRIDE_FD_PITCH),
    );
    PitchInputs {
        servo: ok(servo),
        elev_deg: elev.ok().and_then(|v| d::elevator_surface_deg(&v)),
        total: ok(total),
        trim: ok(trim),
        vs_dial: ok(vs_dial),
        alt_hold_ft: ok(alt_hold),
        ovr_pitch: ok(ovr),
        ovr_fd_pitch: ok(ovr_fd),
    }
}

pub(super) async fn read_inputs(xp: &crate::xplane::XPlaneClient, pitch_due: bool) -> Inputs {
    let ok = |r: anyhow::Result<f64>| r.ok().filter(|v| v.is_finite());
    let (throttle, beta, brake_l, brake_r, yoke_pitch, yoke_roll, yoke_yaw, ap_on) = tokio::join!(
        xp.get_float("sim/cockpit2/engine/actuators/throttle_ratio_all"),
        xp.get_float("sim/cockpit2/engine/actuators/throttle_beta_rev_ratio_all"),
        xp.get_float("sim/cockpit2/controls/left_brake_ratio"),
        xp.get_float("sim/cockpit2/controls/right_brake_ratio"),
        xp.get_float(crate::datarefs::YOKE_PITCH_RATIO),
        xp.get_float(crate::datarefs::YOKE_ROLL_RATIO),
        xp.get_float("sim/joystick/yoke_heading_ratio"),
        xp.get_float(autoflight::names::AP_ON),
    );
    // The pitch channel rides alongside the second batch, never after it.
    let (
        (ap_state, wind_dir, wind_ms, sim_speed, trq, steer_yield, pitch_yield, brake_yield),
        pitch,
    ) = tokio::join!(
        async {
            tokio::join!(
                crate::autoflight::read_ap_state(&*xp),
                xp.get_float(crate::datarefs::WIND_DIR_DEG_TRUE),
                xp.get_float(crate::datarefs::WIND_SPEED_MSC),
                xp.get_float("sim/time/sim_speed"),
                xp.get_float_array("sim/flightmodel/engine/ENGN_TRQ"),
                xp.get_float("xcopilot/steer_yield"),
                xp.get_float("xcopilot/pitch_yield"),
                xp.get_float("xcopilot/brake_yield"),
            )
        },
        async {
            if pitch_due {
                Some(read_pitch(xp).await)
            } else {
                None
            }
        },
    );
    Inputs {
        throttle: ok(throttle),
        beta: ok(beta),
        brake_l: ok(brake_l),
        brake_r: ok(brake_r),
        yoke_pitch: ok(yoke_pitch),
        yoke_roll: ok(yoke_roll),
        yoke_yaw: ok(yoke_yaw),
        ap_on: ok(ap_on),
        ap_state: ok(ap_state),
        wind_dir: ok(wind_dir),
        wind_kts: ok(wind_ms).map(|m| m * 1.943_844),
        sim_speed: ok(sim_speed),
        trq: trq.ok().and_then(|v| v.first().copied()).filter(|v| v.is_finite()),
        steer_yield: ok(steer_yield),
        pitch_yield: ok(pitch_yield),
        brake_yield: ok(brake_yield),
        crew_brake: !crate::tools::brake_cmd_quiet(),
        pitch,
    }
}

/// Past this deflection a yoke under an autopilot is a hand on it.
pub(super) const YOKE_HANDS: f64 = 0.15;

/// How long a write to the crew's own yoke channels (`xcopilot/pitch`,
/// `xcopilot/roll`) owns the yoke reading: the plugin applies a write for
/// 1 s, the monitor reads the yoke some ticks later.
pub(super) const CREW_YOKE_FRESH_S: f64 = 1.5;

pub(super) fn process_secs() -> f64 {
    static EPOCH: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    EPOCH.get_or_init(std::time::Instant::now).elapsed().as_secs_f64()
}

/// Millis since the process epoch of the last crew yoke write, 0 = never.
pub(super) static CREW_YOKE_MS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// A loop wrote the elevator or aileron channel: for the next
/// [`CREW_YOKE_FRESH_S`] the yoke ratio is OURS (the plugin overrides the
/// joystick and writes the ratio itself), not a hand. Called by every writer
/// of `xcopilot/pitch` / `xcopilot/roll` in the air (the takeoff's rotation
/// and hand-flown climb, the autoland's arrest and flare).
pub fn note_crew_yoke() {
    let ms = (process_secs() * 1000.0) as u64 + 1;
    CREW_YOKE_MS.store(ms, std::sync::atomic::Ordering::Relaxed);
}

pub(crate) fn crew_yoke_fresh() -> bool {
    let ms = CREW_YOKE_MS.load(std::sync::atomic::Ordering::Relaxed);
    ms > 0 && process_secs() - (ms - 1) as f64 / 1000.0 <= CREW_YOKE_FRESH_S
}

/// Millis since the process epoch of the pilot's last "my controls" that
/// stopped a flying loop, 0 = never.
pub(super) static PILOT_RELEASE_MS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
/// How long a pilot's own "my controls" answers the automation-stopped
/// watch: the crew already said "You have the aircraft."
pub(super) const PILOT_RELEASE_QUIET_S: f64 = 15.0;

/// The pilot took the aircraft from a flying loop ("my controls",
/// `tools::autopilot::AutopilotOff`): the stop is not news.
pub fn note_pilot_release() {
    let ms = (process_secs() * 1000.0) as u64 + 1;
    PILOT_RELEASE_MS.store(ms, std::sync::atomic::Ordering::Relaxed);
}

pub(crate) fn pilot_released_recently() -> bool {
    let ms = PILOT_RELEASE_MS.load(std::sync::atomic::Ordering::Relaxed);
    ms > 0 && process_secs() - (ms - 1) as f64 / 1000.0 <= PILOT_RELEASE_QUIET_S
}

/// Which interventions the tick sees, pure: `[steer_yield, pitch_yield,
/// brake_yield, yoke_under_ap]`. The yoke is a hand only when WE are not the
/// one moving it: under our own pitch/roll channel the ratio is the loop's
/// output (1790441293: `yp` 0.21 to 0.32 in the 737's arrest and flare,
/// -0.37 at the takeoff's AP engage, the channel's exact value, four
/// "interventions" and a failed sortie), and a pilot's hand on the yoke
/// while the channel is applied is the plugin's `pitch_yield` latch to see.
/// Likewise the brake latch is not a hand when it is the crew's
/// (`crew_brake_latch`, from [`CrewBrakeLatch`]).
pub(super) fn interventions_now(
    i: &Inputs,
    control: crate::flight::Control,
    crew_yoke: bool,
    crew_brake_latch: bool,
) -> [bool; 4] {
    let automation = matches!(control, crate::flight::Control::Automation(_));
    let hands = automation
        && !crew_yoke
        && i.ap_on.is_some_and(|v| v > 0.5)
        && [i.yoke_pitch, i.yoke_roll].iter().any(|v| v.is_some_and(|x| x.abs() > YOKE_HANDS));
    [
        i.steer_yield.is_some_and(|v| v > 0.5),
        i.pitch_yield.is_some_and(|v| v > 0.5),
        i.brake_yield.is_some_and(|v| v > 0.5) && !crew_brake_latch,
        hands,
    ]
}

/// Whose `xcopilot/brake_yield` latch this is. The plugin latches it on ANY
/// begin of the sim's brake commands while the steer channel holds the toe
/// brakes for steering alone, and it cannot tell the pilot's key from our
/// own Web API command: a rejected takeoff fires `brakes_regular` and then
/// `brakes_max` while the roll law is still steering, and the latch rose
/// 0.4 to 0.9 s after every reject at EGKK (1790524628, 1790525414,
/// 1790525500, 2026-09-27), an "intervention" that made each sortie
/// attended. A latch that ROSE while a crew brake command was in flight
/// (`tools::brake_cmd_quiet` false) is the crew's for its whole episode (it
/// stays up until the steer channel goes stale); one that rose with our
/// brakes quiet is a hand. A pilot pressing the key during our own stop is
/// invisible either way: the latch is one bit, and it is already up.
#[derive(Debug, Default, Clone, Copy)]
pub(super) struct CrewBrakeLatch {
    pub(super) prev: bool,
    pub(super) crews: bool,
}

impl CrewBrakeLatch {
    /// The latch's raw reading and whether a crew brake is in flight; true
    /// while the latch is up and it is the crew's.
    pub(super) fn step(&mut self, latched: bool, crew_brake: bool) -> bool {
        if latched && !self.prev {
            self.crews = crew_brake;
        }
        if !latched {
            self.crews = false;
        }
        self.prev = latched;
        latched && self.crews
    }
}

/// Interventions the tick can see: a plugin yield latch rising (feet on the
/// pedals, hands on the yoke, a toe brake during a steered drive), or the yoke
/// deflected while the autopilot flies for an automation and no loop of ours
/// is writing it. Each is blackboxed `intervention` once per episode; "my
/// controls" and the loops' own yield stand-downs record through
/// `flight::note_pilot_takeover`. Returns whether the pilot's hand is on the
/// yoke under the automation this tick (`yoke_under_ap`).
pub(super) fn note_interventions(i: &Inputs, control: crate::flight::Control) -> bool {
    static PREV: std::sync::Mutex<[bool; 4]> = std::sync::Mutex::new([false; 4]);
    static BRAKE_LATCH: std::sync::Mutex<CrewBrakeLatch> =
        std::sync::Mutex::new(CrewBrakeLatch { prev: false, crews: false });
    let crews = BRAKE_LATCH
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .step(i.brake_yield.is_some_and(|v| v > 0.5), i.crew_brake);
    let now = interventions_now(i, control, crew_yoke_fresh(), crews);
    let names = ["steer_yield", "pitch_yield", "brake_yield", "yoke_under_ap"];
    let mut prev = PREV.lock().unwrap_or_else(|e| e.into_inner());
    for k in 0..4 {
        if now[k] && !prev[k] {
            crate::blackbox::record(
                "intervention",
                serde_json::json!({ "what": names[k], "ctl": format!("{control:?}"),
                    "yp": i.yoke_pitch, "yr": i.yoke_roll }),
            );
        }
    }
    *prev = now;
    now[3]
}

/// A hand on the yoke under the automation within this long makes the
/// flying the pilot's, for who landed it.
pub(super) const PILOT_HANDS_RECENT_S: f64 = 30.0;

/// Whether the pilot's hand was on the yoke lately (`yoke_under_ap`, seconds
/// ago). 2026-10-15: the pilot held the 737's yoke through the autoland's
/// flare and the landing was booked as the automation's. Pure.
pub(super) fn pilot_hands_recent(hands_s_ago: Option<f64>) -> bool {
    hands_s_ago.is_some_and(|s| s <= PILOT_HANDS_RECENT_S)
}

pub(crate) fn r1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

/// Who in the world block may act this tick. `quiet` mutes the world's
/// narration while the flight deck is busy (a recovery, an autoland, the
/// panel's mute, a wreck). The cabin and the compliance watch are not
/// narration: they are state machines that must SEE the flight, the autoland
/// included (the cabin missed its own landing, Liege 2026-09-22; the
/// compliance watch called an uncleared landing 26.6 s late, 2026-09-26,
/// issue #63). They speak unless a recovery, the panel's mute or a wreck
/// silences them; the compliance watch TICKS on every live tick but a wreck,
/// so its air/ground edges never go stale.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct WorldGates {
    pub(crate) quiet: bool,
    pub(crate) cabin_quiet: bool,
    pub(crate) compliance_ticks: bool,
    pub(crate) compliance_speaks: bool,
    pub(crate) autoland: bool,
}

pub(super) fn world_gates(recovering: bool, autoland: bool, panel_mute: bool, crashed: bool) -> WorldGates {
    let silenced = recovering || panel_mute || crashed;
    WorldGates {
        quiet: silenced || autoland,
        cabin_quiet: silenced,
        compliance_ticks: !crashed,
        compliance_speaks: !silenced,
        autoland,
    }
}

/// Whether the radio has power, from the airframe's avionics readback and the
/// value read off it. Fails OPEN twice over: no readback dataref (the airframe
/// models no avionics master) and a failed read both mean powered.
///
/// This is a decision, not a publish — the caller writes the result EVERY tick.
/// The flag is process-wide and outlives an airframe switch, so simply going
/// quiet latches whatever the last aircraft said: a cold-and-dark 172 published
/// false, the pilot swapped to a Twin Otter (no avionics master, nothing left
/// to publish), and the stale false silently binned every crew radio call for
/// the rest of the session (Heathrow, 2026-07-27).
pub(super) fn radio_powered_from(readback: Option<&str>, read: Option<f64>) -> bool {
    match (readback, read) {
        (Some(_), Some(v)) => v > 0.5,
        _ => true,
    }
}

/// The avionics-master readback dataref for an airframe, when it models one —
/// it gates whether the pilot can HEAR the radio (station audio). `None` when
/// there's no avionics system, or none with a readback: then the radio stays
/// powered (fail open), so an airframe whose avionics we can't read never goes
/// mysteriously silent. Keyed on the `avionics` system specifically, not the
/// battery/`master` — it's the avionics/audio bus that powers reception.
pub(super) fn avionics_readback_dref(p: &AircraftProfile) -> Option<String> {
    p.electrical
        .iter()
        .find(|s| s.key == "avionics")
        .and_then(|s| s.readback.first().cloned())
}

pub(super) fn compose_strip(s: &Snapshot) -> String {
    let mut parts: Vec<String> = Vec::new();
    if let Ok(e) = crate::autoland::PANEL.lock() {
        if !e.is_empty() {
            parts.push(e.clone());
        }
    }
    if !s.on_ground {
        parts.push(format!("CABIN {:.0}ft", s.cabin_altitude_ft));
    }
    if let Some(m) = s.endurance_min {
        parts.push(format!(
            "FUEL {}h{:02}m",
            (m / 60.0) as i64,
            (m % 60.0) as i64
        ));
    }
    parts.join(" | ")
}
