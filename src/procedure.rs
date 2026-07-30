//! Fly a published terminal procedure — a SID, STAR, or instrument approach —
//! read from X-Plane's CIFP instead of an invented pattern.
//!
//! This is the terrain-safe backbone the naive circuit vectoring never was: the
//! fixes of a real departure or approach define a corridor the airport authority
//! already threaded through the terrain and airspace. The follower is an
//! ORCHESTRATOR in the same mould as [`crate::circuit`] — it flies the AP's
//! heading bug and altitude dial, it does not invent a new flight law:
//!
//! 1. A leg list comes in already parsed and resolved to positions
//!    ([`crate::nav::ProcLeg`] with `fix` filled), plus an [`EndAction`].
//! 2. Each tick the loop steers the great-circle track to the current leg's fix
//!    (wind-crabbed, exactly like the circuit's downwind), descends/climbs to
//!    the leg's altitude constraint, and sequences to the next leg on capture.
//!    To-altitude legs (CA/VA/FA — a missed approach's "climb to 3000", a SID's
//!    initial climb) hold the leg course until the altitude is made.
//! 3. On an APPROACH it hands the FAF-inbound segment to [`crate::autoland`]
//!    (the proven glideslope + flare + rollout); on a DEPARTURE it levels at the
//!    last constraint and hands back to the pilot.
//!
//! Trust model matches the circuit: every actuation is the same AP the copilot
//! tools drive, and "my controls" / "cancel" hands back at any point. It only
//! flies what the data says — a field with no published procedure (Gibraltar)
//! gets an honest refusal at the tool, never a guessed path into the Rock.
//! EXPERIMENTAL and coarse (fix flyby, not RF arcs or holds); fly it VMC with a
//! hand ready and mine `logs/blackbox.jsonl` (`"kind":"procedure"`) to tune.

use std::sync::atomic::{AtomicBool, Ordering};

use crate::autoland::true_to_mag;
use crate::nav::ProcLeg;
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const HPATH: &str = "sim/flightmodel/position/hpath";
const AGL_M: &str = "sim/flightmodel/position/y_agl";
const ELEV_M: &str = "sim/flightmodel/position/elevation";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
const IAS: &str = "sim/flightmodel/position/indicated_airspeed";
// AP surface (mirror of autopilot.rs / autoland.rs / circuit.rs).
const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";
const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
const HDG_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
const HDG_STATUS: &str = "sim/cockpit2/autopilot/heading_status";

const M_TO_FT: f64 = 3.280_84;

/// A leg's fix counts as reached inside this radius (a flyby simplification —
/// we don't fly RF arcs or exact fly-over turns yet). 0.8 nm keeps the corridor
/// tight without a light aircraft overshooting a sharp turn.
const CAPTURE_NM: f64 = 0.8;
/// A to-altitude leg (CA/VA/FA) is satisfied within this of its target.
const ALT_MARGIN_FT: f64 = 150.0;

/// True while the procedure follower runs.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// Set true to cancel ("cancel" / "my controls").
pub static ABORT: AtomicBool = AtomicBool::new(false);

/// One-line status for the dashboard's "managing" panel.
pub fn status() -> Option<String> {
    ACTIVE
        .load(Ordering::Relaxed)
        .then(|| "flying a procedure".to_string())
}

/// Cancel an active procedure and stand down whatever sub-loop is flying;
/// returns whether one was running. Autoland (if the approach handed off) is
/// aborted too — the pilot gets the aircraft.
pub fn abort() -> bool {
    ABORT.store(true, Ordering::SeqCst);
    crate::autoland::abort();
    ACTIVE.swap(false, Ordering::SeqCst)
}

/// What to do when the last flyable leg is reached.
#[derive(Clone)]
pub enum EndAction {
    /// Instrument approach: hand the FAF-inbound segment to autoland.
    Approach {
        apt_dat: String,
        cfg: ApConfig,
        approach_ias: f64,
    },
    /// Departure/arrival that ends in the enroute structure: level at the last
    /// constraint and hand back to the pilot.
    Level,
    /// Enroute segment flown as one phase of a larger flight (the full-auto
    /// SimBrief mission): complete QUIETLY — no "your aircraft", the flight
    /// orchestrator is watching `ACTIVE` and owns what happens next.
    Enroute,
}

/// Everything the follower needs, resolved by the arming tool.
pub struct Plan {
    /// Spoken name of the procedure, e.g. "the ILS approach runway two seven".
    pub name: String,
    /// Legs, already resolved (`fix` filled where a fix exists). Unresolved fix
    /// legs are skipped in flight.
    pub legs: Vec<ProcLeg>,
    pub end: EndAction,
    /// Best-climb speed reference for the SPEED-PROTECTED climb law; 0 = no
    /// protection (a descent-only approach plan). VS mode holds RATE whatever
    /// the airspeed: an unprotected 700 fpm enroute climb bled 86 knots down
    /// to a 53-knot stall departure at 4,600 ft (LEJR, 2026-07-24).
    pub vy_kts: f64,
    /// Indicated cruise speed for the cruise-power governor. `Some` on an enroute
    /// flight arms the speed-hold (throttle finds the power to hold it) for the
    /// whole leg, released at handoff; `None` leaves the throttle alone (the
    /// climb-out setting rides through — approaches/SIDs, which manage their own).
    pub cruise_ias_kts: Option<f64>,
}

/// The true great-circle bearing to a leg's fix, or `None` for a to-altitude
/// leg (no fix to steer at). Pure — the lateral-steering decision, testable
/// without a sim.
pub fn bearing_to_fix(lat: f64, lon: f64, leg: &ProcLeg) -> Option<f64> {
    leg.fix
        .as_ref()
        .map(|f| crate::nav::bearing_deg(lat, lon, f.lat, f.lon))
}

/// Has the current leg been flown? A fix leg completes inside [`CAPTURE_NM`]; a
/// to-altitude leg completes when the altitude constraint is satisfied. Pure and
/// position-based, so sequencing is immune to how slow the AP's turns are (the
/// same lesson that drove the circuit off timers).
pub fn leg_captured(leg: &ProcLeg, dist_to_fix_nm: Option<f64>, alt_ft: f64) -> bool {
    match dist_to_fix_nm {
        Some(d) => d <= CAPTURE_NM,
        // No fix: a to-altitude leg. '+' climb to at-or-above, '-' descend to
        // at-or-below, otherwise cross at the altitude.
        None => match leg.alt1_ft {
            Some(target) => match leg.alt_desc {
                '+' => alt_ft >= target - ALT_MARGIN_FT,
                '-' => alt_ft <= target + ALT_MARGIN_FT,
                _ => (alt_ft - target).abs() <= ALT_MARGIN_FT,
            },
            // A heading leg with no altitude (rare, FM/VM manual) — nothing to
            // wait for; treat as instantly done so we don't stall the sequence.
            None => true,
        },
    }
}

/// The altitude-dial target for a leg, feet MSL: its own constraint when it has
/// one, else `carry` (hold the last commanded altitude across an unconstrained
/// fix). The AP's ALT hold flies toward it — climb or descent falls out of
/// whether the target is above or below.
pub fn commanded_alt_ft(leg: &ProcLeg, carry: f64) -> f64 {
    leg.alt1_ft.unwrap_or(carry)
}

/// The leg to hand off to autoland on an approach: the FAF if the data flags one,
/// else the last fix leg before the runway threshold, else the last leg. `None`
/// only when there are no legs. Handing off AT the FAF lets autoland own the
/// glideslope from platform to touchdown, exactly where it's proven.
pub fn handoff_index(legs: &[ProcLeg]) -> Option<usize> {
    if legs.is_empty() {
        return None;
    }
    if let Some(i) = legs.iter().position(|l| l.is_faf) {
        return Some(i);
    }
    // The leg just before the first runway-threshold ("RW..") leg.
    if let Some(rw) = legs.iter().position(|l| l.fix_ident.starts_with("RW")) {
        return Some(rw.saturating_sub(1));
    }
    Some(legs.len() - 1)
}

/// A human word for the approach type from its ident's leading letter.
pub(crate) fn approach_kind(ident: &str) -> &'static str {
    match ident.chars().next() {
        Some('I') => "ILS",
        Some('X') | Some('R') | Some('Q') => "RNAV",
        Some('L') => "localizer",
        Some('D') | Some('S') | Some('V') => "VOR",
        _ => "instrument",
    }
}

/// Build an APPROACH plan for `rwy_id` at `icao`: pick the best published
/// approach, load its final segment, resolve every fix against the nav data.
/// `Err` carries an honest refusal (no published approach / unreadable legs) for
/// the caller to speak. Shared by the `fly_procedure` tool and the circuit's
/// arrival handoff so both build the arrival the same way.
#[allow(clippy::too_many_arguments)]
pub fn approach_plan_for(
    cifp: &str,
    fix_path: &str,
    nav_path: &str,
    airport: &crate::nav::AirportInfo,
    icao: &str,
    rwy_id: &str,
    lat: f64,
    lon: f64,
    end: EndAction,
) -> Result<Plan, String> {
    let spoken = crate::circuit::spell_runway(rwy_id);
    // Prefer a full CIFP approach; fall back to a straight-in reconstructed from
    // the runway's published DME fixes (the way a pilot flies the DME approach
    // when there's no coded procedure — Gibraltar's whole story).
    let (legs, name) = match crate::nav::best_approach(cifp, icao, rwy_id) {
        Some(appch) => {
            let mut legs =
                crate::nav::load_proc_legs(cifp, icao, crate::nav::ProcKind::Appch, &appch, None);
            crate::nav::resolve_proc_fixes(fix_path, nav_path, airport, icao, &mut legs, lat, lon);
            (
                legs,
                format!("the {} approach runway {spoken}", approach_kind(&appch)),
            )
        }
        None => (Vec::new(), String::new()),
    };
    let (legs, name) = if legs.is_empty() {
        let recon = crate::nav::reconstruct_approach(fix_path, airport, icao, rwy_id);
        if recon.is_empty() {
            return Err(format!(
                "{icao} has no published approach to runway {spoken}. No coded procedure and no \
                 DME approach fixes to reconstruct one from."
            ));
        }
        (recon, format!("a straight-in approach to runway {spoken}"))
    } else {
        (legs, name)
    };
    // Approaches descend; the climb protection reference stays off unless the
    // caller sets one (e.g. the enroute follower).
    Ok(Plan {
        name,
        legs,
        end,
        vy_kts: 0.0,
        cruise_ias_kts: None, // an approach/SID manages its own speed
    })
}

/// Build the ENROUTE legs of a SimBrief plan: direct-to legs over the named
/// fixes, with the plan's own crossing altitudes as constraints. Pure, so the
/// altitude logic battle-tests without a fetch.
///
/// - The origin airport (prepended by the fetch) and the destination airport
///   (the arrival phase owns that) are dropped — this is the middle of the
///   flight only.
/// - SimBrief's per-fix altitudes carry the whole profile: rising through the
///   climb, cruise in the middle, falling after TOD. Flying the CLIMB fixes at
///   their charted crossing altitudes would stair-step the departure (level at
///   3,000 over a fix mid-climb, then climb again), so every fix before the
///   first at-cruise fix commands cruise directly — one continuous climb — and
///   from there the plan's own numbers stand, which is what flies the descent.
/// - A fix with no plausible altitude (0 in the navlog) commands cruise.
pub fn enroute_legs(
    waypoints: &[crate::simbrief::Waypoint],
    origin: &str,
    destination: &str,
    cruise_alt_ft: f64,
) -> Vec<ProcLeg> {
    let mid: Vec<&crate::simbrief::Waypoint> = waypoints
        .iter()
        .enumerate()
        .filter(|(i, w)| {
            (*i != 0 || !w.ident.eq_ignore_ascii_case(origin))
                && (*i != waypoints.len() - 1 || !w.ident.eq_ignore_ascii_case(destination))
        })
        .map(|(_, w)| w)
        .collect();
    let plausible = |alt: f64| (500.0..=60_000.0).contains(&alt);
    let raw: Vec<f64> = mid
        .iter()
        .map(|w| {
            if plausible(w.alt) {
                w.alt
            } else {
                cruise_alt_ft
            }
        })
        .collect();
    let peak = raw.iter().copied().fold(cruise_alt_ft, f64::max);
    let first_at_peak = raw.iter().position(|a| *a >= peak - 100.0).unwrap_or(0);
    mid.iter()
        .zip(raw.iter())
        .enumerate()
        .map(|(i, (w, alt))| ProcLeg {
            seq: i as u32,
            transition: String::new(),
            leg_type: "TF".to_string(),
            fix_ident: w.ident.clone(),
            fix_region: String::new(),
            fix: Some(crate::nav::Fix {
                ident: w.ident.clone(),
                lat: w.lat,
                lon: w.lon,
            }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: Some(if i < first_at_peak { peak } else { *alt }),
            alt2_ft: None,
            is_faf: false,
        })
        .collect()
}

/// Whether the aircraft is positioned to actually FLY this approach — outside
/// the final approach fix and at/above its crossing altitude. A tight circuit at
/// pattern height INSIDE the FAF is not (arming there would command a climb back
/// to platform); those stay with autoland. An enroute arrival at altitude is.
/// The seam that lets the circuit's arrival prefer the real approach only when
/// it makes sense.
pub fn approach_flyable_from(plan: &Plan, lat: f64, lon: f64, alt_ft: f64) -> bool {
    let Some(hi) = handoff_index(&plan.legs) else {
        return false;
    };
    let leg = &plan.legs[hi];
    let Some(fix) = &leg.fix else {
        return false;
    };
    let dist = crate::nav::haversine_nm(lat, lon, fix.lat, fix.lon);
    let faf_alt = leg.alt1_ft.unwrap_or(0.0);
    dist >= 2.0 && alt_ft >= faf_alt - 500.0
}

/// Spawn the follower on its own thread + runtime + client, the circuit's shape.
pub fn start(base_url: String, voice: Option<Voice>, plan: Plan) {
    if ACTIVE.swap(true, Ordering::SeqCst) {
        return;
    }
    ABORT.store(false, Ordering::SeqCst);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            ACTIVE.store(false, Ordering::SeqCst);
            return;
        };
        rt.block_on(fly(
            XPlaneClient::new(base_url.clone()),
            base_url,
            voice,
            plan,
        ));
        ACTIVE.store(false, Ordering::SeqCst);
    });
}

fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    tracing::info!(target: "xvoicelayer::procedure", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(priority, line.to_string());
    }
}

fn stand_down(voice: &Option<Voice>, line: &str, reason: &str) {
    ACTIVE.store(false, Ordering::SeqCst);
    // Drop any cruise-power hold so nothing keeps working the throttle after we
    // hand back (no-op if it was never armed).
    crate::tools::speedhold::release();
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "stand_down", "reason": reason}),
    );
    alert(voice, Priority::Caution, line);
}

async fn fly(xp: XPlaneClient, base_url: String, voice: Option<Voice>, plan: Plan) {
    let Plan {
        name,
        legs,
        end,
        vy_kts,
        cruise_ias_kts,
    } = plan;
    let handoff = handoff_index(&legs);
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "armed", "name": name, "legs": legs.len(), "approach": matches!(end, EndAction::Approach{..})}),
    );
    alert(&voice, Priority::Advisory, &format!("Flying {name}."));

    // Engage the AP: servos + heading; vertical is VS-mode + dial (the
    // takeoff's proven recipe — `altitude_hold` holds the CURRENT altitude in
    // X-Plane, not the dial, and never flew a constraint; the circuit's
    // pattern flew flat at handoff height for the same reason, 2026-07-24).
    let _ = xp.run_command(CMD_SERVOS_ON).await;
    if xp.get_float(HDG_STATUS).await.unwrap_or(0.0) <= 0.5 {
        let _ = xp.run_command(CMD_HDG_MODE).await;
    }
    let mag_var = xp.get_float(MAG_VAR).await.unwrap_or(0.0);
    // The heading bug is SLEWED, never jumped: a single write across the
    // 0/360 seam reads as a long-way dial spin to the sim's AP (the circuit's
    // 270-degree wrong-way turn, 2026-07-24). And the slew must START AT THE
    // AIRCRAFT'S NOSE, never at the dial's read: seeding from a failed read
    // (unwrap_or(0.0)) walked the dial clockwise from zero to a target 22°
    // to our LEFT, the AP honored the spin direction, and the aircraft flew
    // a full 340° right orbit over Jerez before converging (16:59Z,
    // 2026-07-24). Snapping the dial to psi first is safe whatever the jump
    // (the AP is already pointing there, so the inferred direction moves it
    // nowhere), and every subsequent step is unambiguous.
    let psi_seed = xp.get_float(PSI_TRUE).await.unwrap_or(0.0);
    let mut bug_mag = true_to_mag(psi_seed, mag_var);
    let _ = xp.set_float(HDG_BUG, bug_mag).await;
    // Carry starts at the current altitude so an unconstrained first leg holds.
    let mut carry_alt = xp.get_float(ELEV_M).await.unwrap_or(0.0) * M_TO_FT;
    let mut last_vs_cmd: Option<f64> = None;
    // The altitude PRESELECT is driven to the leg target too: a stale dial
    // with capture armed (1,122 ft on the Jerez departure, from nobody's
    // loop) would otherwise level the climb wherever it happened to sit —
    // and driven to the real target, the capture works FOR us: it owns the
    // level-off at the constraint more cleanly than the VS chase.
    let mut last_alt_dial: Option<f64> = None;

    // Cruise power: on an enroute flight, arm the flight speed-hold at the
    // planned indicated cruise speed. The throttle law then FINDS the power —
    // full while slow (the climb still commands full throttle), settling to
    // cruise power once at speed, robust to weight/altitude/temp. Pitch (the VS
    // law below) owns altitude; the governor owns speed. Released at every exit
    // so autoland/the arrival own the throttle alone.
    if matches!(end, EndAction::Enroute) {
        if let Some(kts) = cruise_ias_kts {
            crate::tools::speedhold::arm_flight(base_url.clone(), voice.clone(), kts);
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "cruise_power", "target_ias": kts}),
            );
        }
    }

    let mut idx = 0usize;
    while idx < legs.len() {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if ABORT.load(Ordering::SeqCst) {
            stand_down(&voice, "Cancelled. Your aircraft.", "abort");
            return;
        }
        // A frozen sim (pause/replay) suspends the sequence; the AP holds.
        if crate::xplane::sim_frozen(&xp).await {
            continue;
        }

        let (lat, lon, elev_m, psi_r, hpath_r, agl_m, vs_st, hdg_st, ias_r) = tokio::join!(
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(ELEV_M),
            xp.get_float(PSI_TRUE),
            xp.get_float(HPATH),
            xp.get_float(AGL_M),
            xp.get_float(VS_STATUS),
            xp.get_float(HDG_STATUS),
            xp.get_float(IAS),
        );
        let (Ok(lat), Ok(lon)) = (lat, lon) else {
            continue;
        };
        let alt_ft = elev_m.unwrap_or(0.0) * M_TO_FT;
        let agl_ft = agl_m.unwrap_or(0.0) * M_TO_FT;

        let leg = &legs[idx];

        // Skip a leg whose fix never resolved — flying at a phantom (0,0) fix
        // would drag the aircraft off the planet. Log it and move on.
        if leg.fix.is_none() && !leg.fix_ident.is_empty() {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "skip_unresolved", "idx": idx, "fix": leg.fix_ident}),
            );
            idx += 1;
            continue;
        }

        // --- Lateral: steer the great-circle track to the fix, wind-crabbed so
        // the AP holds the GROUND track (the circuit's downwind lesson). A
        // to-altitude leg holds its own magnetic course.
        let psi_v = psi_r.ok();
        let crab = match (psi_v, hpath_r.ok()) {
            (Some(psi), Some(hp)) => {
                (((psi - hp + 540.0).rem_euclid(360.0)) - 180.0).clamp(-25.0, 25.0)
            }
            _ => 0.0,
        };
        let dist_nm = bearing_to_fix(lat, lon, leg).map(|_| {
            let f = leg.fix.as_ref().unwrap();
            crate::nav::haversine_nm(lat, lon, f.lat, f.lon)
        });
        let target_mag = if let Some(brg_true) = bearing_to_fix(lat, lon, leg) {
            Some(true_to_mag(brg_true + crab, mag_var))
        } else {
            // course is already magnetic; add crab to hold it as a track.
            leg.course_mag.map(|c| (c + crab).rem_euclid(360.0))
        };
        if let Some(t) = target_mag {
            bug_mag = crate::circuit::slew_bug(bug_mag, t, 40.0);
            let _ = xp.set_float(HDG_BUG, bug_mag).await;
        }
        // The lateral mode must actually be ON for the bug to mean anything —
        // the circuit's guard, which this follower LACKED: an AP disconnect
        // mid-climb left the servos off with only the VS re-engage watching,
        // and the aircraft spiralled at 20°/s while the loop slewed a bug
        // nobody was following (LEJR enroute, 2026-07-24).
        if hdg_st.unwrap_or(1.0) <= 0.5 {
            let _ = xp.run_command(CMD_SERVOS_ON).await;
            let _ = xp.run_command(CMD_HDG_MODE).await;
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "reengage_lateral", "idx": idx}),
            );
        }

        // --- Vertical: fly toward the leg's constraint (carried across
        // unconstrained legs) with the VS law — proportional rate, clamped,
        // mode re-engaged when dropped (the takeoff's recipe) — and the
        // preselect dialled to the same target so the sim's armed capture
        // levels off AT the constraint instead of a stale value.
        let want_alt = commanded_alt_ft(leg, carry_alt);
        carry_alt = want_alt;
        if last_alt_dial.is_none_or(|v| (v - want_alt).abs() > 50.0) {
            let _ = xp.set_float(ALT_DIAL, want_alt).await;
            last_alt_dial = Some(want_alt);
        }
        // Speed-protected: VS mode holds RATE whatever the airspeed, so the
        // climb rate backs off as IAS sags below Vy (the takeoff's own law,
        // floored positive) — the unprotected 700 fpm stalled the LEJR climb.
        let mut vs_cmd = ((want_alt - alt_ft) * 3.0).clamp(-700.0, 700.0);
        if vs_cmd > 0.0 && vy_kts > 0.0 {
            if let Ok(ias) = ias_r {
                vs_cmd = vs_cmd.min(crate::autotakeoff::climb_vs_fpm(ias, vy_kts, vs_cmd));
            }
        }
        if vs_st.unwrap_or(1.0) <= 0.5 {
            // VS mode gone NEAR the target means the preselect CAPTURED —
            // its hold is the clean level-off, leave it flying. Far from the
            // target it's a dropped mode: re-engage and rewrite the dial.
            if (want_alt - alt_ft).abs() > 100.0 {
                let _ = xp.run_command(CMD_VS_MODE).await;
                last_vs_cmd = None; // engage synced the dial; force a rewrite
            }
        }
        if last_vs_cmd.is_none_or(|v| (v - vs_cmd).abs() > 40.0) {
            let _ = xp.set_float(VS_DIAL, vs_cmd).await;
            last_vs_cmd = Some(vs_cmd);
        }

        crate::blackbox::record(
            "procedure",
            serde_json::json!({
                "event": "leg", "idx": idx, "type": leg.leg_type, "fix": leg.fix_ident,
                "dist_nm": dist_nm, "alt_ft": alt_ft, "want_alt": want_alt, "agl_ft": agl_ft, "crab": crab,
                "psi": psi_v, "bug": bug_mag, "tgt": target_mag,
            }),
        );

        // --- Approach handoff: at the FAF leg, once captured, autoland owns it.
        if let (
            EndAction::Approach {
                apt_dat,
                cfg,
                approach_ias,
            },
            Some(hi),
        ) = (&end, handoff)
        {
            if idx >= hi && leg_captured(leg, dist_nm, alt_ft) {
                crate::blackbox::record(
                    "procedure",
                    serde_json::json!({"event": "handoff_autoland", "idx": idx, "agl_ft": agl_ft}),
                );
                alert(&voice, Priority::Advisory, "Established. Landing us.");
                crate::autoland::ABORT.store(false, Ordering::SeqCst);
                crate::autoland::run_autoland(
                    base_url.clone(),
                    apt_dat.clone(),
                    cfg.clone(),
                    *approach_ias,
                    voice.clone(),
                    None,
                    None,
                );
                crate::tools::speedhold::release();
                ACTIVE.store(false, Ordering::SeqCst);
                return;
            }
        }

        if leg_captured(leg, dist_nm, alt_ft) {
            idx += 1;
            if idx < legs.len() {
                let next = &legs[idx];
                let label = if next.fix_ident.is_empty() {
                    "next leg".to_string()
                } else {
                    next.fix_ident.clone()
                };
                alert(&voice, Priority::Advisory, &format!("Next, {label}."));
            }
        }
    }

    // Ran off the end of the legs (a departure, or an approach with no handoff).
    // Drop any cruise-power hold before handing back (no-op if never armed).
    crate::tools::speedhold::release();
    match end {
        EndAction::Level => {
            crate::blackbox::record("procedure", serde_json::json!({"event": "complete_level"}));
            alert(
                &voice,
                Priority::Advisory,
                "Procedure complete. Level, the autopilot is holding. Your aircraft.",
            );
        }
        EndAction::Enroute => {
            // The flight orchestrator is polling ACTIVE; it announces the
            // arrival itself. The AP keeps holding heading + altitude.
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "complete_enroute"}),
            );
        }
        EndAction::Approach { .. } => {
            crate::blackbox::record(
                "procedure",
                serde_json::json!({"event": "complete_no_handoff"}),
            );
            alert(
                &voice,
                Priority::Caution,
                "Ran out of approach legs without a handoff. Your aircraft.",
            );
        }
    }
    ACTIVE.store(false, Ordering::SeqCst);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav::Fix;

    fn leg(fix_ident: &str, lat: f64, lon: f64) -> ProcLeg {
        ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "TF".to_string(),
            fix_ident: fix_ident.to_string(),
            fix_region: "LE".to_string(),
            fix: Some(Fix {
                ident: fix_ident.to_string(),
                lat,
                lon,
            }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: None,
            alt2_ft: None,
            is_faf: false,
        }
    }

    fn to_alt(desc: char, alt: f64) -> ProcLeg {
        ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "CA".to_string(),
            fix_ident: String::new(),
            fix_region: String::new(),
            fix: None,
            course_mag: Some(315.0),
            dist_nm: None,
            turn: None,
            alt_desc: desc,
            alt1_ft: Some(alt),
            alt2_ft: None,
            is_faf: false,
        }
    }

    #[test]
    fn steers_the_great_circle_bearing_to_the_fix() {
        // A fix due east: bearing ~90.
        let l = leg("EAST", 36.0, -4.0);
        let brg = bearing_to_fix(36.0, -5.0, &l).unwrap();
        assert!((brg - 90.0).abs() < 1.0, "{brg}");
        // A to-altitude leg has no fix to steer at.
        assert!(bearing_to_fix(36.0, -5.0, &to_alt('+', 3000.0)).is_none());
    }

    #[test]
    fn a_fix_leg_captures_inside_the_radius() {
        let l = leg("F", 36.0, -5.0);
        assert!(!leg_captured(&l, Some(1.5), 2000.0), "far: not captured");
        assert!(
            leg_captured(&l, Some(CAPTURE_NM - 0.1), 2000.0),
            "inside: captured"
        );
    }

    #[test]
    fn a_climb_leg_captures_at_or_above_the_altitude() {
        let climb = to_alt('+', 3000.0);
        assert!(!leg_captured(&climb, None, 2000.0), "below: keep climbing");
        assert!(leg_captured(&climb, None, 3000.0), "made it");
        assert!(leg_captured(&climb, None, 2900.0), "within margin counts");
    }

    #[test]
    fn a_descend_leg_captures_at_or_below_the_altitude() {
        let down = to_alt('-', 3000.0);
        assert!(!leg_captured(&down, None, 5000.0), "above: keep descending");
        assert!(leg_captured(&down, None, 3000.0), "made it");
    }

    #[test]
    fn altitude_carries_across_unconstrained_legs() {
        // A fix leg with no altitude keeps the last dialled value.
        let plain = leg("F", 36.0, -5.0);
        assert_eq!(commanded_alt_ft(&plain, 4500.0), 4500.0);
        // A constrained leg dials its own.
        let mut constrained = leg("G", 36.0, -5.0);
        constrained.alt1_ft = Some(2500.0);
        assert_eq!(commanded_alt_ft(&constrained, 4500.0), 2500.0);
    }

    #[test]
    fn handoff_is_the_faf_when_flagged() {
        let mut legs = vec![
            leg("IF", 36.0, -5.0),
            leg("FAF", 36.1, -5.0),
            leg("RW27", 36.2, -5.0),
        ];
        legs[1].is_faf = true;
        assert_eq!(handoff_index(&legs), Some(1));
    }

    #[test]
    fn handoff_falls_back_to_the_leg_before_the_runway() {
        // No FAF flag: hand off the leg before the RW threshold.
        let legs = vec![
            leg("IF", 36.0, -5.0),
            leg("STEP", 36.1, -5.0),
            leg("RW27", 36.2, -5.0),
        ];
        assert_eq!(handoff_index(&legs), Some(1));
        assert_eq!(handoff_index(&[]), None);
    }

    fn wp(ident: &str, lat: f64, lon: f64, alt: f64) -> crate::simbrief::Waypoint {
        crate::simbrief::Waypoint {
            ident: ident.to_string(),
            lat,
            lon,
            alt,
        }
    }

    #[test]
    fn enroute_legs_drop_the_airports_and_keep_the_middle() {
        // The Jerez–Gibraltar shape: origin + two fixes + destination.
        let wps = vec![
            wp("LEJR", 36.74, -6.06, 0.0),
            wp("VJF", 36.24, -5.98, 5000.0),
            wp("LINTO", 35.83, -5.95, 5000.0),
            wp("LXGB", 36.15, -5.35, 2500.0),
        ];
        let legs = enroute_legs(&wps, "LEJR", "LXGB", 5000.0);
        assert_eq!(legs.len(), 2);
        assert_eq!(legs[0].fix_ident, "VJF");
        assert_eq!(legs[1].fix_ident, "LINTO");
        assert!(legs.iter().all(|l| l.fix.is_some() && l.leg_type == "TF"));
        assert_eq!(legs[0].alt1_ft, Some(5000.0));
    }

    #[test]
    fn climb_fixes_command_cruise_but_descent_fixes_stand() {
        // A mid-climb fix at 3,000 must not stair-step the departure; the
        // post-TOD 4,000 must be honored (that's the descent).
        let wps = vec![
            wp("AAA", 36.0, -5.0, 3000.0),
            wp("BBB", 36.2, -5.0, 5000.0),
            wp("CCC", 36.4, -5.0, 5000.0),
            wp("DDD", 36.6, -5.0, 4000.0),
        ];
        let legs = enroute_legs(&wps, "LEJR", "LXGB", 5000.0);
        let alts: Vec<f64> = legs.iter().map(|l| l.alt1_ft.unwrap()).collect();
        assert_eq!(alts, vec![5000.0, 5000.0, 5000.0, 4000.0]);
    }

    #[test]
    fn a_zero_altitude_fix_commands_cruise() {
        let wps = vec![wp("AAA", 36.0, -5.0, 0.0), wp("BBB", 36.2, -5.0, 0.0)];
        let legs = enroute_legs(&wps, "LEJR", "LXGB", 4500.0);
        assert!(legs.iter().all(|l| l.alt1_ft == Some(4500.0)));
    }

    #[test]
    fn a_plan_of_just_airports_yields_no_enroute_legs() {
        let wps = vec![
            wp("LEJR", 36.74, -6.06, 0.0),
            wp("LXGB", 36.15, -5.35, 2500.0),
        ];
        assert!(enroute_legs(&wps, "LEJR", "LXGB", 5000.0).is_empty());
    }

    #[test]
    fn a_tight_circuit_is_not_flyable_but_an_arrival_is() {
        // FAF at (36.0,-5.0), crossing 2500 ft.
        let mut faf = leg("FAF", 36.0, -5.0);
        faf.is_faf = true;
        faf.alt1_ft = Some(2500.0);
        let plan = Plan {
            name: "test".into(),
            legs: vec![faf],
            end: EndAction::Level,
            vy_kts: 0.0,
            cruise_ias_kts: None,
        };
        // Tight circuit: 0.3 nm from the FAF at 700 ft — NOT flyable (autoland).
        let (near_lat, near_lon) = crate::nav::project_latlon(36.0, -5.0, 0.0, 0.3);
        assert!(!approach_flyable_from(&plan, near_lat, near_lon, 700.0));
        // Arrival: 6 nm out at 3000 ft — flyable.
        let (far_lat, far_lon) = crate::nav::project_latlon(36.0, -5.0, 90.0, 6.0);
        assert!(approach_flyable_from(&plan, far_lat, far_lon, 3000.0));
        // Far out but too low (below platform − 500): not flyable.
        assert!(!approach_flyable_from(&plan, far_lat, far_lon, 1500.0));
    }
}
