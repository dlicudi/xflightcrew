//! Background monitor — the proactive half of the copilot.
//!
//! A dedicated task samples a handful of datarefs a few times a second, estimates
//! the flight phase, and runs a set of [`Check`]s. A check that finds a problem
//! produces a spoken callout, queued (non-blocking) through the speech arbiter.
//!
//! Discipline lives in [`FireState`]: callouts are **edge-triggered** (spoken
//! when the problem first appears, not every tick) and Warnings **repeat** on an
//! interval until the condition clears — so the copilot speaks up once, reminds
//! you if it's serious, and otherwise stays quiet.
//!
//! This is the proactive sibling of the tool safety gates: same safety logic,
//! running continuously and *speaking* instead of refusing. It never touches the
//! controls. See `docs/monitoring.md`.

use crate::autoflight::{self, Dial};
use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::aircraft::AircraftProfile;
use crate::config::Config;
use crate::plugin_status::StatusSink;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

/// A taxi clearance this soon after full auto ran answers the mission's own
/// request: the pilot's short-runway advice leaves it alone.
const MISSION_CLEARANCE_SECS: f64 = 15.0;
const MS_TO_KTS: f64 = 1.943_844;
const M_TO_FT: f64 = 3.280_84;

/// How often a Warning re-announces while the condition persists.
/// Minimum gap between two announcements of the *same* alert — whether it
/// persisted (Warnings re-announce on this interval) or cleared and recurred
/// (so cycling a switch off/on/off doesn't re-nag each time).
const RENOTIFY_COOLDOWN: Duration = Duration::from_secs(30);

/// Directional-gyro precession past this many degrees (vs the magnetic compass)
/// is worth calling out. The vacuum DG drifts a few degrees every ~15 min in a
/// real 172; ~10° is the "genuinely off, sync it" amount and matches the usual
/// DG-vs-compass tolerance — small enough to be useful, large enough not to
/// nag on normal precession. (`align_gyro` itself treats <2° as a good sync,
/// deliberately tighter than this alert.)
const GYRO_DRIFT_ADVISORY_DEG: f64 = 10.0;
/// Grace after an approach→tower handoff before the landing-clearance coach
/// nudges: long enough for a prompt pilot to report final and be cleared
/// (which clears the handoff and cancels the nudge), short enough to still
/// leave room to act before the threshold.
const LANDING_COACH_GRACE_SECS: u64 = 12;
/// Knob time after "radar services terminated, squawk seven thousand,
/// contact X Tower" before the copilot checks the code and the frequency.
const REGION_COACH_GRACE_SECS: u64 = 20;

/// Where this flight is going — the context that turns the monitor from a
/// systems watchdog into a copilot that can *anticipate*. Set by the main loop
/// whenever a flight plan is fetched or loaded to the FMS; the descent advisor
/// checks read it every tick. `None` = no plan known, advisors stay silent.
#[derive(Debug, Clone)]
pub struct Destination {
    pub lat: f64,
    pub lon: f64,
    /// Field elevation, feet (the plan's destination waypoint altitude when
    /// plausible, else 0 — a sea-level assumption only makes TOD *earlier*).
    pub elev_ft: f64,
    /// Planned top-of-descent distance from the destination, nm. SimBrief's
    /// is ALONG THE ROUTE (the navlog's legs after TOD), so it is compared
    /// with [`route_remaining_nm`] over `route`, never the straight line.
    pub tod_ring_nm: f64,
    /// The plan's points, origin to destination, `(lat, lon)`: the legs the
    /// ring is measured along. Empty = no route known (straight line then).
    pub route: Vec<(f64, f64)>,
}

/// Miles still to fly to the end of `route` from here: straight to the end
/// of the leg we are nearest (by the distance to each leg, clamped to it),
/// then the rest of the plan's legs. `None` with fewer than two points.
///
/// The plan's TOD ring is along-route: LSZS-LILO (SR22, 2026-09-24, #109)
/// flies 63 nm of valley legs to a field 23 nm away in a straight line, so a
/// 25 nm ring compared with the straight line was "past" from the runway and
/// "coming up" at every point of the route's northern loop. Pure.
pub fn route_remaining_nm(route: &[(f64, f64)], lat: f64, lon: f64) -> Option<f64> {
    if route.len() < 2 {
        return None;
    }
    // Local flat miles about the aircraft: good enough to pick the leg.
    let k = lat.to_radians().cos();
    let xy = |(a, b): (f64, f64)| ((b - lon) * 60.0 * k, (a - lat) * 60.0);
    let mut best: Option<(f64, usize)> = None;
    for i in 0..route.len() - 1 {
        let (ax, ay) = xy(route[i]);
        let (bx, by) = xy(route[i + 1]);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        let t = if len2 > 1e-12 {
            ((-ax * dx - ay * dy) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (px, py) = (ax + t * dx, ay + t * dy);
        let d = (px * px + py * py).sqrt();
        // At a vertex both legs are equally near: the later one is the leg
        // being flown once the fix is passed, and costs the same either way.
        if best.is_none_or(|(b, _)| d <= b + 1e-9) {
            best = Some((d, i));
        }
    }
    let (_, i) = best?;
    let next = route[i + 1];
    let mut sum = crate::nav::haversine_nm(lat, lon, next.0, next.1);
    for w in route[i + 1..].windows(2) {
        sum += crate::nav::haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1);
    }
    Some(sum)
}

pub static DESTINATION: std::sync::Mutex<Option<Destination>> = std::sync::Mutex::new(None);

/// A new flight: the last plan's destination no longer drives the TOD ring
/// or the answers until a plan read publishes it again (`plan::ensure_plan`).
pub(crate) fn forget_destination() {
    *DESTINATION.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// Coarse flight phase, estimated from motion and height. Used to arm/disarm
/// checks (e.g. "flaps up on final" only matters on Approach).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Phase {
    Ground,
    Takeoff,
    Climb,
    Cruise,
    Approach,
    Landing,
}

/// A point-in-time read of the systems/state the checks care about.
pub struct Snapshot {
    pub on_ground: bool,
    pub groundspeed_kts: f64,
    /// Indicated airspeed, knots — drives the stall/airspeed callout.
    pub ias_kts: f64,
    /// Rate of change of IAS, knots per second (negative = decelerating).
    /// Computed by the poll loop across ticks; lets the airspeed callout fire on
    /// a fast washout *before* the speed actually reaches the threshold.
    pub ias_trend_kts_per_s: f64,
    /// Flap deflection 0.0–1.0 — stall speed drops with flaps, so the airspeed
    /// callout's threshold scales with it.
    pub flap_ratio: f64,
    pub agl_ft: f64,
    /// The pilot's VSI (`vvi_fpm_pilot`): what the pilot sees, a second late.
    pub vsi_fpm: f64,
    /// The flight model's vertical speed (`vh_ind_fpm`), `None` when the read
    /// failed: what the flight model and the touchdown are judged on (#198).
    pub vs_fpm: Option<f64>,
    /// Elevator/stabilizer trim, -1 (full nose down) .. +1 (full nose up). Used to
    /// catch trim pinned at a limit (out of authority / possible runaway).
    pub elevator_trim: f64,
    /// Seatbelt sign lit (`sim/cockpit2/annunciators/fasten_seatbelt` — the
    /// annunciator, not the switch, so auto modes count). `None` = read
    /// failed; the cabin watch invents no edge from a gap.
    pub seatbelt_sign: Option<bool>,
    /// Pushback tug attached (`sim/aircraft/overflow/pushback_attached`).
    pub tug_attached: bool,
    /// Pushback/tow actively underway (`sim/flightmodel2/misc/tow_in_progress`).
    pub pushing_back: bool,
    pub engine_running: bool,
    pub alternator_on: bool,
    /// Highest per-source charging output, amps (`generator_amps`, all sources).
    /// Positive = at least one alternator/generator is demonstrably producing.
    /// Multi-battery airframes (MD-82) idle with a small standing battery-bus
    /// discharge even while both generators charge, so the battery ammeter alone
    /// can't distinguish "generator failed" from "battery charger cycling".
    pub generator_amps_max: f64,
    /// Directional-gyro precession vs the magnetic compass, degrees, folded into
    /// −180..180 (so a small *negative* drift reads as e.g. −4°, not 356°).
    pub gyro_drift_deg: f64,
    /// Minutes of fuel remaining at the current burn (total fuel ÷ fuel flow).
    /// `None` when it can't be computed (no fuel/flow read, or flow ≈ 0).
    pub endurance_min: Option<f64>,
    /// The region's wind profile, re-read on its own slow clock. The step-climb
    /// advisory names the wind at the level it suggests: whether the climb is
    /// free is the whole decision, and on EIDW-BIKF it was (one knot).
    pub winds_aloft: Option<crate::weather::WindsAloft>,
    /// All-up weight, kg (`None` if the read failed). The step-climb advisory
    /// reads it: the altitude a jet flies best at is a function of weight, so
    /// without it there is no optimum to compare against.
    pub all_up_kg: Option<f64>,
    /// Total fuel aboard, kg. `None` if the read failed. Used by the ground
    /// check, which estimates endurance from a nominal cruise burn instead of
    /// the (near-zero) idle burn.
    pub fuel_kg: Option<f64>,
    /// Total fuel flow, all engines, kg/s (`None` when unread).
    pub fuel_flow_kg_sec: Option<f64>,
    /// Engine oil pressure, PSI (`None` if the read failed). Normal 50–90 (POH).
    pub oil_pressure_psi: Option<f64>,
    /// Engine oil temperature, °F (`None` if the read failed). NOTE: the
    /// `oil_temperature_deg_C` dataref is misnamed — it reads °F (verified live:
    /// ~205°F running, 59°F = ambient cold). Normal 100–245°F (POH).
    pub oil_temp_f: Option<f64>,
    /// Cylinder head temperature, °F (`None` if the read failed). The `CHT_deg_C`
    /// dataref is likewise misnamed — it reads °F (verified live: ~354°F cruise).
    pub cht_f: Option<f64>,
    /// Outside air temp, °C — for the carb-ice advisory.
    pub oat_c: f64,
    /// Dewpoint, °C — a humidity proxy (small OAT−dewpoint spread = moist air).
    pub dewpoint_c: f64,
    /// Carburettor heat applied (ratio > 0.5).
    pub carb_heat_on: bool,
    /// Engine RPM — low RPM (descent/idle, throttle closed) is the worst regime
    /// for carburettor ice.
    pub rpm: f64,
    /// Low-vacuum annunciator — the vacuum-driven gyros (AI/DG) may be unreliable.
    pub low_vacuum: bool,
    /// Battery ammeter, amps. Positive = charging, negative = discharging.
    pub battery_amps: f64,
    /// Parking-brake ratio (0 released .. 1 set) — for the rolling-with-the-brake
    /// check. The C90 doesn't animate its handle from this, so the panel can look
    /// released while the sim-level brake drags (seen live: 48 kt -> stopped).
    pub park_brake: f64,
    /// A crew loop's own brake command is in flight (`tools::brake_cmd_quiet`
    /// false). X-Plane shows a HELD brake command on `parking_brake_ratio`:
    /// `brakes_regular` ramps it to about 0.36, `brakes_max` to 1.0, and it
    /// decays within a second or two of the command's expiry. So while this
    /// is true the ratio is our brake, not a parking brake anyone set.
    pub crew_brake: bool,
    /// Cabin altitude, feet (`.../pressurization/indicators/cabin_altitude_ft`). On
    /// an unpressurized airframe this tracks the actual altitude, which is exactly
    /// what the hypoxia check wants; on a pressurized one it's the real cabin.
    pub cabin_altitude_ft: f64,
    /// Fuel tank selector position (`sim/cockpit2/fuel/fuel_tank_selector`,
    /// 0 = OFF). `None` if the read failed.
    pub fuel_selector: Option<f64>,
    /// Indicated altitude, feet (`sim/flightmodel/misc/h_ind`).
    pub alt_ft: f64,
    /// Autopilot altitude preselector (the dial the pilot spins), feet.
    /// `None` if the read failed. NOT `altitude_hold_ft`, which is the last
    /// *captured* altitude and goes stale the moment a new target is dialed.
    pub ap_alt_dial_ft: Option<f64>,
    /// Autopilot servos engaged — someone (or something) is flying on the dial.
    pub ap_servos_on: bool,
    /// Gear handle down (fixed-gear airframes read permanently down).
    pub gear_down: bool,
    /// Values of the profile's advisor datarefs, index-aligned with
    /// [`AircraftProfile::advisor`]. `None` = that read failed this tick.
    pub advisor_values: Vec<Option<f64>>,
    /// Position (`sim/flightmodel/position/latitude`/`longitude`).
    pub lat: f64,
    pub lon: f64,
    /// Which read the position came from (`None` = it failed): the plugin's
    /// one frame, or an older plugin's separate reads (#157).
    pub pos_source: Option<sim::position::Source>,
    /// Ground track, degrees true (`sim/flightmodel/position/hpath`). `None` =
    /// the read failed; consumers treat it as "track unknown", never 0.
    pub track_deg: Option<f64>,
    /// True heading (`psi`) and bank (`phi`), degrees, best-effort (0 on a
    /// failed read): the instructor's scorer and the blackbox state record.
    pub heading_deg: f64,
    pub bank_deg: f64,
    /// Sim frame rate (`sim/time/framerate_period`, inverted). `None` = read
    /// failed or the sim is paused (period 0). Feeds the fps-dip watchdog.
    pub fps: Option<f64>,
    /// Descent picture vs the planned destination, when one is known
    /// ([`DESTINATION`]): great-circle distance to the field, its elevation,
    /// and the planned TOD ring. All-or-nothing — `None` without a plan.
    pub dest_dist_nm: Option<f64>,
    pub dest_elev_ft: Option<f64>,
    pub tod_ring_nm: Option<f64>,
    /// Miles still to fly along the plan's legs ([`route_remaining_nm`]):
    /// what the along-route ring is compared with. `None` = no route, and
    /// [`ring_dist_nm`] falls back to `dest_dist_nm`.
    pub dest_route_nm: Option<f64>,
    /// The procedure follower's OWN top of descent while one flies
    /// ([`crate::procedure::planned_tod`]). Outranks the ring above for the
    /// descent advisors: the point the hands will use is the only one worth
    /// announcing.
    pub follower_tod: Option<crate::procedure::PlannedTod>,
    /// A procedure follower is flying the plan ([`crate::procedure::ACTIVE`]).
    /// THE RING IS NEVER CONSULTED WHILE THIS IS TRUE, even on a leg where
    /// the follower has no top of descent to publish: `follower_tod` is
    /// `None` on a leg with no fix or whose constraint is not below us, and
    /// falling through to the destination ring there let the two advisors
    /// take turns. EGLF-LFPB (2026-09-22) heard "top of descent in about a
    /// minute" from the follower, then "12 miles ahead" from the ring, then
    /// the ring's "we're past the descent point" as a Caution, then the
    /// follower's "7 miles ahead" again - four calls, two geometries, one
    /// descent that was being flown correctly the whole time.
    pub follower_flying: bool,
    /// The flight's highest altitude so far (the flight model's memory):
    /// a top of descent is the one that leaves it.
    pub flight_peak_ft: Option<f64>,
    /// The altimeter knob (inHg) and the sim's local QNH (Pa), best-effort,
    /// for the altimeter cross-check. `None` = that read failed this tick.
    pub baro_inhg: Option<f64>,
    pub qnh_pa: Option<f64>,
    /// An automation is flying the altitude (the follower, full auto, the
    /// circuit or autoland): the altimeter is then the crew's knob to set,
    /// not the pilot's to be advised about.
    pub automation_flying: bool,
    /// The tanks (fuel, pump switch, lateral position), read only on an
    /// airframe that declares `fuel_tank_pumps_off_when_empty`. `None` = not
    /// declared or a read failed this tick: no measurement, no callout.
    pub fuel_tanks: Option<crate::tools::fuel_pumps::Tanks>,
    /// Where the aircraft IS, feet above sea level (`position/elevation`):
    /// the altimeter cross-check's truth (#94). `None` = no read.
    pub alt_true_ft: Option<f64>,
    /// The ice picture and the anti-ice switches (#100), read by the poll
    /// loop on a slow clock in cold air. `None` = warm air or not read.
    pub icing: Option<crate::tools::anti_ice::IcingRead>,
    pub phase: Phase,
}

impl Snapshot {
    /// Read the snapshot from X-Plane. `None` if the sim is unreachable (the
    /// caller just skips the tick and tries again). `advisor_drefs` are the
    /// profile's phase-expectation switches, read best-effort per tick.
    async fn read(xp: &XPlaneClient, advisor_drefs: &[String]) -> Option<Snapshot> {
        let on_ground = xp
            .get_float(crate::datarefs::ON_GROUND_ANY)
            .await
            .ok()?
            > 0.5;
        let groundspeed_kts = xp
            .get_float(crate::datarefs::GROUNDSPEED_MS)
            .await
            .ok()?
            * MS_TO_KTS;
        let ias_kts = xp
            .get_float(crate::datarefs::AIRSPEED_KTS)
            .await
            .ok()?;
        let flap_ratio = xp
            .get_float(crate::datarefs::FLAP_RATIO)
            .await
            .unwrap_or(0.0);
        let agl_ft = xp.get_float(crate::datarefs::AGL_METERS).await.ok()? * M_TO_FT;
        let vsi_fpm = xp
            .get_float("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
            .await
            .ok()?;
        let vs_fpm = xp.get_float(sim::datarefs::VS_FPM_FLIGHT_MODEL).await.ok();
        // Best-effort (a missing read must not kill the snapshot): the altitude
        // picture for the bust/sink-rate guards.
        let alt_ft = xp
            .get_float("sim/flightmodel/misc/h_ind")
            .await
            .unwrap_or(0.0);
        let ap_alt_dial_ft = autoflight::read_dial(xp, Dial::Altitude)
            .await
            .ok();
        let baro_inhg = xp.get_float(BARO_KNOB).await.ok();
        let qnh_pa = xp.get_float(crate::datarefs::QNH_PASCALS).await.ok();
        let alt_true_ft = xp
            .get_float(sim::datarefs::ELEVATION_M)
            .await
            .ok()
            .map(|m| m * M_TO_FT);
        // The one "an automation is flying" (`flight_state::crew_flying`):
        // the takeoff, the helicopter's loops and a recovery count too, so
        // under them the checks fix rather than advise (refactor audit
        // 2026-10-04, duplicates #1).
        let automation_flying = crate::flight_state::crew_flying().is_some();
        let ap_servos_on = crate::autoflight::read_status(&*xp, autoflight::names::SERVOS_ON)
            .await
            .unwrap_or(0.0)
            > 0.5;
        let gear_down = xp
            .get_float("sim/cockpit/switches/gear_handle_status")
            .await
            .unwrap_or(1.0)
            > 0.5;
        let mut advisor_values = Vec::with_capacity(advisor_drefs.len());
        for dref in advisor_drefs {
            advisor_values.push(xp.get_float(dref).await.ok());
        }
        // Position + the descent picture vs the planned destination (if any).
        // One frame's latitude and longitude (#157): read separately they
        // paired two frames, metres apart at speed, and this is the position
        // the taxi guide projects and the `state` record the scorer judges.
        let (lat, lon, pos_source) = match sim::position::read_lat_lon(xp).await {
            Ok((lat, lon, src)) => (lat, lon, Some(src)),
            Err(_) => (0.0, 0.0, None),
        };
        let track_deg = xp.get_float(crate::datarefs::TRACK_DEG_TRUE).await.ok();
        let heading_deg = xp
            .get_float(crate::datarefs::TRUE_HEADING_DEG)
            .await
            .unwrap_or(0.0);
        let bank_deg = xp
            .get_float(crate::datarefs::PHI_DEG)
            .await
            .unwrap_or(0.0);
        let fps = xp
            .get_float("sim/time/framerate_period")
            .await
            .ok()
            .filter(|p| *p > 0.0)
            .map(|p| 1.0 / p);
        let dest = DESTINATION
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone();
        let (dest_dist_nm, dest_elev_ft, tod_ring_nm, dest_route_nm) = match dest {
            Some(d) => (
                Some(crate::nav::haversine_nm(lat, lon, d.lat, d.lon)),
                Some(d.elev_ft),
                Some(d.tod_ring_nm),
                route_remaining_nm(&d.route, lat, lon),
            ),
            None => (None, None, None, None),
        };
        // Best-effort (a missing read must not kill the snapshot): the
        // seatbelt sign, for the cabin crew's watch.
        let seatbelt_sign = xp
            .get_float("sim/cockpit2/annunciators/fasten_seatbelt")
            .await
            .ok()
            .map(|v| v > 0.5);
        // Best-effort (a missing read must not kill the snapshot): pushback state.
        let tug_attached = xp
            .get_float(crate::datarefs::PUSHBACK_ATTACHED)
            .await
            .unwrap_or(0.0)
            > 0.5;
        // Being pushed = hooked to a tug AND moving. NOT `tow_in_progress`:
        // probed live during a real push, that dataref stayed 0 from hook-up
        // to release (2026-07-27), so the callouts keyed on it ("Pushing
        // back", "Pushback complete, tug clear") could never speak and the
        // only line the pilot ever heard was the tug connecting. Read anyway,
        // as extra evidence in case some airframe does raise it.
        let towing = xp
            .get_float("sim/flightmodel2/misc/tow_in_progress")
            .await
            .unwrap_or(0.0)
            > 0.5;
        let pushing_back = towing || (tug_attached && groundspeed_kts > 0.3);
        let engine_running = xp
            .get_float_array(crate::datarefs::ENGINE_RUNNING)
            .await
            .ok()?
            .first()
            .copied()
            .unwrap_or(0.0)
            > 0.5;
        let alternator_on = xp
            .get_float_array("sim/cockpit/electrical/generator_on")
            .await
            .ok()?
            .first()
            .copied()
            .unwrap_or(0.0)
            > 0.5;
        // Best-effort (a missing read must not kill the snapshot): highest
        // charging output across all sources, for the not-charging check.
        let generator_amps_max = xp
            .get_float_array("sim/cockpit2/electrical/generator_amps")
            .await
            .ok()
            .and_then(|v| v.into_iter().reduce(f64::max))
            .unwrap_or(0.0);
        // Best-effort: a missing/!unavailable drift dataref must not kill the
        // whole snapshot (which would silence every other check), so default to
        // 0.0 rather than propagating None. Fold into −180..180, same as the
        // `align_gyro` tool, so a small negative drift doesn't read as ~360.
        let gyro_drift_raw = xp
            .get_float("sim/cockpit/gyros/dg_drift_vac_deg")
            .await
            .unwrap_or(0.0);
        let gyro_drift_deg = fold_drift_deg(gyro_drift_raw);

        // Fuel endurance = total fuel ÷ current burn. Both best-effort (a missing
        // read must not kill the snapshot); endurance is None when either is
        // absent or the burn is ~0 (idle/shutdown), where "minutes remaining"
        // is meaningless or infinite.
        let fuel_kg = xp
            .get_float("sim/flightmodel/weight/m_fuel_total")
            .await
            .ok();
        let all_up_kg = xp
            .get_float(sim::datarefs::TOTAL_WEIGHT_KG)
            .await
            .ok()
            .filter(|w| w.is_finite() && *w > 0.0);
        // Cached inside the reader on its own slow clock — see `winds_aloft`.
        let winds_aloft = crate::weather::winds_aloft(xp).await;
        let fuel_flow_kg_sec = xp
            .get_float_array(crate::datarefs::FUEL_FLOW_KG_SEC)
            .await
            .ok()
            .map(|v| v.iter().sum::<f64>());
        let endurance_min = match (fuel_kg, fuel_flow_kg_sec) {
            (Some(kg), Some(flow)) if flow > 1e-4 => Some(kg / flow / 60.0),
            _ => None,
        };

        // Engine instruments, best-effort (a missing read must not kill the
        // snapshot). Per-engine arrays; element 0. Names verified against the
        // stock 172's cockpitdecks mapping.
        let engine0 = |v: Vec<f64>| v.first().copied();
        let oil_pressure_psi = xp
            .get_float_array("sim/cockpit2/engine/indicators/oil_pressure_psi")
            .await
            .ok()
            .and_then(engine0);
        // NOTE: despite the "_deg_C"/"deg_C" names, both of these read °F in XP12
        // (verified live: ~205°F oil, ~354°F CHT at cruise). Stored as °F.
        let oil_temp_f = xp
            .get_float_array("sim/cockpit2/engine/indicators/oil_temperature_deg_C")
            .await
            .ok()
            .and_then(engine0);
        let cht_f = xp
            .get_float_array("sim/cockpit2/engine/indicators/CHT_deg_C")
            .await
            .ok()
            .and_then(engine0);

        // Carb-ice / vacuum / electrical inputs, best-effort. Defaults chosen so a
        // failed read can't raise a false advisory (mild OAT, dry, no discharge).
        let oat_c = xp
            .get_float("sim/cockpit2/temperature/outside_air_temp_degc")
            .await
            .unwrap_or(15.0);
        // The dewpoint AT the aircraft, not the lowest of the sim's levels
        // (`anti_ice::dewpoint_at`; the wrong level kept the anti-ice on for
        // a whole flight, #156). Unread is dry: no false advisory.
        let dewpoint_c = {
            let (dew, levels) = tokio::join!(
                xp.get_float_array(crate::tools::anti_ice::DEWPOINT_LEVELS),
                xp.get_float_array(crate::tools::anti_ice::ATMOSPHERE_LEVELS_M),
            );
            let alt_m = alt_true_ft.unwrap_or(alt_ft) / M_TO_FT;
            match (dew, levels) {
                (Ok(d), Ok(l)) => crate::tools::anti_ice::dewpoint_at(&l, &d, alt_m),
                _ => None,
            }
            .unwrap_or(-30.0)
        };
        let carb_heat_on = xp
            .get_float_array("sim/cockpit2/engine/actuators/carb_heat_ratio")
            .await
            .ok()
            .and_then(engine0)
            .unwrap_or(0.0)
            > 0.5;
        let rpm = xp
            .get_float_array(crate::datarefs::PROP_SPEED_RPM)
            .await
            .ok()
            .and_then(engine0)
            .unwrap_or(0.0);
        let elevator_trim = xp
            .get_float(crate::datarefs::ELEVATOR_TRIM)
            .await
            .unwrap_or(0.0);
        let low_vacuum = xp
            .get_float("sim/cockpit/warnings/annunciators/low_vacuum")
            .await
            .unwrap_or(0.0)
            > 0.5;
        let battery_amps = xp
            .get_float_array("sim/cockpit2/electrical/battery_amps")
            .await
            .ok()
            .and_then(engine0)
            .unwrap_or(0.0);

        // Parking brake for the rolling-with-brake check (best-effort).
        let park_brake = xp
            .get_float("sim/cockpit2/controls/parking_brake_ratio")
            .await
            .unwrap_or(0.0);
        // Cabin altitude for the hypoxia check. Best-effort; an unpressurized
        // airframe reports its actual altitude here, which is what we want.
        let cabin_altitude_ft = xp
            .get_float("sim/cockpit2/pressurization/indicators/cabin_altitude_ft")
            .await
            .unwrap_or(0.0);
        // Fuel selector for the selector-off check (best-effort; None stays silent).
        let fuel_selector = xp
            .get_float("sim/cockpit2/fuel/fuel_tank_selector")
            .await
            .ok();

        let s = Snapshot {
            on_ground,
            groundspeed_kts,
            ias_kts,
            ias_trend_kts_per_s: 0.0, // filled in by the poll loop (needs history)
            flap_ratio,
            agl_ft,
            vsi_fpm,
            vs_fpm,
            elevator_trim,
            seatbelt_sign,
            tug_attached,
            pushing_back,
            engine_running,
            alternator_on,
            generator_amps_max,
            gyro_drift_deg,
            endurance_min,
            fuel_kg,
            all_up_kg,
            winds_aloft,
            fuel_flow_kg_sec,
            oil_pressure_psi,
            oil_temp_f,
            cht_f,
            oat_c,
            dewpoint_c,
            carb_heat_on,
            rpm,
            low_vacuum,
            battery_amps,
            park_brake,
            crew_brake: !crate::tools::brake_cmd_quiet(),
            cabin_altitude_ft,
            fuel_selector,
            alt_ft,
            ap_alt_dial_ft,
            ap_servos_on,
            gear_down,
            advisor_values,
            lat,
            lon,
            pos_source,
            track_deg,
            heading_deg,
            bank_deg,
            fps,
            dest_dist_nm,
            dest_elev_ft,
            tod_ring_nm,
            dest_route_nm,
            follower_tod: crate::procedure::planned_tod(),
            follower_flying: follower_flying(
                crate::flight_state::Owner::Procedure.active(),
                crate::flight_state::Owner::HeliRoute.active(),
            ),
            flight_peak_ft: crate::flight::published().map(|f| f.peak_alt_ft).filter(|p| *p > 0.0),
            baro_inhg,
            qnh_pa,
            automation_flying,
            fuel_tanks: None, // filled in by the poll loop (per airframe)
            alt_true_ft,
            icing: None, // filled in by the poll loop (slow clock)
            phase: Phase::Ground,
        };
        // The coarse phase is the flight model's, set by the monitor tick
        // once the model has seen this snapshot (`coarse_phase`).
        Some(s)
    }
}

/// The coarse phase the checks are written against, from the flight
/// model (`flight.rs`): its memory and sustained edges, where the old
/// estimate read one snapshot (a gust at cruise height was an approach).
/// The old boundaries stand: a descent is an approach under 3,000 ft AGL,
/// an approach a landing under 1,500.
pub fn coarse_phase(p: crate::flight::Phase, agl_ft: f64) -> Phase {
    use crate::flight::Phase as F;
    match p {
        F::Parked
        | F::EngineStart
        | F::Ready
        | F::Pushback
        | F::TaxiOut
        | F::Holding
        | F::LinedUp
        | F::TaxiIn => Phase::Ground,
        F::TakeoffRoll | F::RejectedTakeoff | F::Rollout => Phase::Takeoff,
        F::InitialClimb | F::Climb | F::GoAround => Phase::Climb,
        F::Cruise | F::Hover => Phase::Cruise,
        F::Descent if agl_ft < 3000.0 => Phase::Approach,
        F::Descent => Phase::Cruise,
        F::Approach if agl_ft < 1500.0 => Phase::Landing,
        F::Approach => Phase::Approach,
        F::Landing => Phase::Landing,
    }
}

/// The loop with its hands on the aircraft now, by name: the innermost
/// (full auto flies through autoland, the takeoff, the taxi; it is named
/// itself only between them, an engine start). A checklist flow is not
/// flying anything.
/// A recovery is on it too ("recovery"): it was missing, so "you have
/// control" during one was answered by taking the aircraft a second time.
pub(crate) fn flying_loop() -> Option<&'static str> {
    crate::flight_state::crew_has_aircraft().map(crate::flight_state::Owner::name)
}

/// The outermost mission armed: what the flight is FOR, the missions first
/// (they own every loop under them). A lesson is a mission with its hands
/// off. `None` = the pilot's own flight.
/// A recovery is not a mission (it is a hand, not a purpose): with only a
/// recovery flying the flight is still the pilot's.
fn mission() -> Option<&'static str> {
    use crate::flight_state::Owner;
    let outer = crate::flight_state::crew_mission().filter(|o| *o != Owner::Recover);
    match outer {
        Some(o @ (Owner::FullAuto | Owner::HeliRoute | Owner::Circuit | Owner::HeliCircuit)) => {
            Some(o.name())
        }
        _ if crate::instructor::active() => Some("lesson"),
        o => o.map(Owner::name),
    }
}

/// The crew's side of the situation, from the process-wide facts that
/// already carry it (the shelf, the follower's leg, the party line).
fn situation() -> crate::flight::Situation {
    use crate::flight::{ClearanceFact, PlanFact, Situation};
    let plan = crate::simbrief::loaded_plan().map(|p| PlanFact {
        source: match p.source {
            crate::simbrief::PlanSource::SimBrief => "simbrief",
            crate::simbrief::PlanSource::Local => "local",
        },
        origin: p.origin,
        destination: p.destination,
        cruise_alt_ft: p.cruise_alt_ft,
    });
    let clearance = crate::party_line::clearance_state().map(|st| ClearanceFact {
        state: format!("{st:?}"),
        runway: crate::party_line::clearance_runway(),
        squawk: crate::party_line::assigned_squawk(),
        readback_owed: crate::party_line::readback_owed().is_some(),
    });
    Situation {
        mission: mission(),
        plan,
        active_fix: crate::procedure::active_leg().map(|l| l.fix),
        clearance,
        pilot_takeover: crate::flight::pilot_takeover(),
        towered: Some(crate::mode::atc_enabled() && crate::party_line::station_reaches()),
        at_hold: Some(
            !crate::autotaxi::driving()
                && crate::autotaxi::LAST_ARRIVED.load(std::sync::atomic::Ordering::Relaxed)
                && !crate::party_line::taxi_to_stand(),
        ),
    }
}

/// The flight model's sample from this tick: the `state` record's numbers
/// plus the evidence only the live tick has. What the monitor does not read
/// stays `None` (the model never enters a phase on a guess).
fn flight_sample(s: &Snapshot, t: f64, crashed: bool) -> crate::flight::Sample {
    crate::flight::Sample {
        t,
        on_ground: s.on_ground,
        gs_kts: s.groundspeed_kts,
        ias_kts: s.ias_kts,
        vsi_fpm: s.vs_fpm.unwrap_or(s.vsi_fpm),
        agl_ft: s.agl_ft,
        alt_ft: s.alt_ft,
        engine_running: s.engine_running,
        fuel_kg: s.fuel_kg,
        lat: Some(s.lat),
        lon: Some(s.lon),
        tug_attached: Some(s.tug_attached),
        on_runway: if s.on_ground {
            crate::party_line::on_runway(s.lat, s.lon)
        } else {
            None
        },
        approach: Some(crate::flight_state::Owner::Autoland.active()),
        automation: flying_loop(),
        autopilot: Some(s.ap_servos_on),
        crashed: Some(crashed),
        contact: crate::flight::first_contact(),
        situation: Some(situation()),
        ..crate::flight::Sample::default()
    }
}

/// Severity of a callout. Maps onto the speech arbiter's priority.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Severity {
    Advisory,
    Caution,
    Warning,
}

impl From<Severity> for Priority {
    fn from(s: Severity) -> Priority {
        match s {
            Severity::Advisory => Priority::Advisory,
            Severity::Caution => Priority::Caution,
            Severity::Warning => Priority::Warning,
        }
    }
}

/// One monitored condition. `eval` returns the callout text when the condition
/// is currently bad, `None` when it's fine.
pub trait Check: Send {
    fn id(&self) -> &'static str;
    fn severity(&self) -> Severity;
    fn eval(&self, s: &Snapshot) -> Option<String>;
    /// How long the condition must hold continuously before we speak. Defaults
    /// to zero (fire on the rising edge). Override for conditions that are
    /// briefly true during normal operation — e.g. the alternator is offline for
    /// a few seconds during every engine start — so a transient doesn't nag.
    fn min_duration(&self) -> Duration {
        Duration::ZERO
    }
    /// Higher values permit a fresh callout when a persistent condition worsens.
    fn repeat_level(&self, _s: &Snapshot) -> Option<i64> {
        None
    }
    /// A command the copilot should FIRE (not just advise) to bring the state to
    /// what the SOP expects — e.g. the strobe/landing-light switches. Fired once
    /// per episode, the moment the advisory speaks, so the copilot *acts* instead
    /// of nagging. `None` (the default) = advisory only, the pilot's switch.
    fn fix_action(&self) -> Option<&str> {
        None
    }
    /// A dataref the copilot should WRITE (a knob, not a switch) when the
    /// callout speaks: `(dataref, value)`. Same edge-triggered, once-per-episode
    /// discipline as `fix_action`. `None` (the default) = advisory only.
    fn fix_write(&self, _s: &Snapshot) -> Option<(&'static str, f64)> {
        None
    }
    /// The crew's own job that a landing does NOT do for itself, so the
    /// landing's mute ([`check_muted`]) lets it through. `false` (the
    /// default): pilot-directed advice, backwards while the copilot lands.
    fn acts_while_crew_lands(&self, _s: &Snapshot) -> bool {
        false
    }
}

/// Whether a check's line (and the fix it carries) is held this tick.
/// `taking` the loop taking every lever ([`crate::flight_state::
/// crew_taking_every_lever`]): a recovery mutes everything, a landing all
/// but the checks that are the crew's own job ([`Check::acts_while_crew_lands`]);
/// the panel's mute, everything. Pure.
///
/// LEGR-LXGB, 2026-10-06 (sortie 1791275171-full_auto): the follower
/// levelled at 12,700 above LXGB's 7,000 ft transition level, autoland flew
/// the arrival down from there, and the landing's mute swallowed both
/// altimeter checks with their resets: the whole approach on 29.92 against
/// a 30.06 QNH, the field at minus 108 ft on the ground, and the first word
/// about it ("Altimeter check, QNH is three zero zero six") after landing.
fn check_muted(
    taking: Option<crate::flight_state::Owner>,
    panel_mute: bool,
    acts_while_crew_lands: bool,
) -> bool {
    use crate::flight_state::Owner;
    panel_mute
        || match taking {
            None => false,
            Some(Owner::Autoland) => !acts_while_crew_lands,
            Some(o) => o.takes_every_lever(),
        }
}

/// Per-check firing state: enforces edge-triggering and Warning repeats.
struct FireState {
    /// The CURRENT episode of the condition has already been announced. Set
    /// only when the callout actually speaks — never on the sustain-wait
    /// ticks before it — and reset by [`clear`](Self::clear). (The old
    /// every-tick "firing" flag consumed the rising edge while `min_duration`
    /// was still counting, so a recurred Caution could never speak twice in
    /// one flight: a stalling 172's second "getting slow" was eaten exactly
    /// there, four minutes before terrain, 2026-07-13.)
    spoke_this_episode: bool,
    last_level: Option<i64>,
    last_spoken: Option<Instant>,
    /// When the condition most recently became true (rising edge). Used to
    /// enforce a check's `min_duration` sustain before the first callout.
    since: Option<Instant>,
}

impl FireState {
    fn new() -> Self {
        FireState {
            spoke_this_episode: false,
            last_level: None,
            last_spoken: None,
            since: None,
        }
    }

    fn observe_level(&mut self, level: Option<i64>) {
        if let Some(level) = level {
            if self.last_level.is_some_and(|old| level > old) {
                self.spoke_this_episode = false;
            }
            self.last_level = Some(self.last_level.map_or(level, |old| old.max(level)));
        }
    }

    /// Should we speak right now? Never more than once per `RENOTIFY_COOLDOWN` —
    /// that single rule covers both cases: a persisting Warning re-announcing on
    /// the interval, and a cleared-then-recurred alert (so fiddling a switch
    /// doesn't re-nag). Once the cooldown has elapsed, a fresh occurrence (rising
    /// edge) always speaks; a still-firing one only repeats if it's a Warning.
    fn should_speak(&self, sev: Severity, now: Instant) -> bool {
        match self.last_spoken {
            None => true, // never announced
            Some(t) => {
                if now.duration_since(t) < RENOTIFY_COOLDOWN {
                    return false; // within cooldown — stay quiet
                }
                !self.spoke_this_episode || sev == Severity::Warning
            }
        }
    }

    /// Has the condition held long enough to be worth speaking? Records the
    /// rising edge on first call and returns whether `min_duration` has elapsed
    /// since — so a transient that [`clear`](Self::clear)s before then never
    /// reaches the speak path.
    fn sustained(&mut self, min_duration: Duration, now: Instant) -> bool {
        let since = *self.since.get_or_insert(now);
        now.duration_since(since) >= min_duration
    }

    /// Condition is currently bad. Speak per [`should_speak`], but only once it
    /// has held continuously for `min_duration`. Marks firing either way.
    /// Returns whether it SPOKE this call — the caller fires any `fix_action`
    /// command on that rising edge, so the copilot acts exactly once per episode.
    #[allow(clippy::too_many_arguments)]
    fn active(
        &mut self,
        id: &str,
        msg: String,
        sev: Severity,
        min_duration: Duration,
        voice: &Voice,
        status: Option<&StatusSink>,
        now: Instant,
        acts_while_crew_lands: bool,
    ) -> bool {
        // While the copilot is flying the aircraft (an active recovery/autoland),
        // stay quiet: it announces its own actions, and pilot-directed advice
        // ("lower the nose, add power") is backwards when the copilot is the one
        // doing it. The panel's "mute monitor" setting silences callouts too.
        // A landing's own crew job (the altimeter) is not advice
        // ([`check_muted`]).
        let flying = check_muted(
            crate::flight_state::crew_taking_every_lever(),
            PANEL_MUTE.load(std::sync::atomic::Ordering::Relaxed),
            acts_while_crew_lands,
        );
        if !flying
            && self.sustained(min_duration, now)
            && self.should_speak(
                if id.starts_with("cabin_alt_") || id == "fuel_warning" {
                    Severity::Caution
                } else {
                    sev
                },
                now,
            )
        {
            // Mirror OR action, never both: `voice.alert` below already posts the
            // spoken line to the panel transcript, so a separate `s.alert` here
            // wrote every callout twice (a coloured line AND a green one). The
            // one door all callouts pass through — the whole class was doubling.
            let _ = status;
            // Black box: the callout is part of the flight record (the LEAL–LEMD
            // CFIT forensics had to be reconstructed without them), and it's what
            // the post-flight distiller counts.
            crate::blackbox::record(
                "callout",
                serde_json::json!({ "id": id, "sev": format!("{sev:?}"), "msg": &msg }),
            );
            voice.hazard(sev, msg);
            self.last_spoken = Some(now);
            self.spoke_this_episode = true;
            return true;
        }
        false
    }

    /// Condition cleared. Keep `last_spoken` so a quick recurrence is still
    /// inside the cooldown and won't re-nag; drop the episode flag and reset
    /// the sustain timer so the next occurrence must hold for `min_duration`
    /// again.
    fn clear(&mut self) {
        self.last_level = None;
        self.spoke_this_episode = false;
        self.since = None;
    }
}

// The checks, the inputs and the loop in their own files, whole and
// unchanged (refactor phase 4); every path stays `crate::monitor::...`
// through these re-exports.
mod checks;
pub use checks::*;
mod inputs;
pub use inputs::*;
mod run;
pub use run::*;

/// What the pilot still owes the region handoff, as one spoken line, or
/// `None` when both the code and the frequency already comply. `com1` is
/// COM1 in kHz; `code` the transponder code, read only when seven thousand
/// was ordered.
pub fn region_coach_line(
    h: &crate::party_line::RegionHandoff,
    com1: Option<i64>,
    code: Option<i64>,
) -> Option<String> {
    let mut owed = Vec::new();
    if h.squawk_vfr && code != Some(7000) {
        owed.push("squawk seven thousand".to_string());
    }
    if com1.is_none_or(|k| (k - h.khz).abs() > 5) {
        owed.push(format!("{} on {}", h.to, crate::atis::spoken_mhz(h.khz)));
    }
    if owed.is_empty() {
        return None;
    }
    // An IFR flight is handed on, not let go: no service ended.
    let lead = if h.ifr_code.is_some() {
        "We've been handed over"
    } else {
        "Radar's done with us"
    };
    Some(format!("{lead}: {}.", owed.join(", and ")))
}

#[cfg(test)]
mod region_coach_tests;

/// A distance the way it is said: "under a mile", "a mile", "12 miles".
/// "1 miles ahead" was spoken (EGKK-EGBB, 2026-09-27, #127).
pub(crate) fn spoken_miles(nm: f64) -> String {
    match nm.round() as i64 {
        n if n < 1 => "under a mile".to_string(),
        1 => "a mile".to_string(),
        n => format!("{n} miles"),
    }
}

/// Under this many seconds a time to a point is "under a minute", never
/// rounded up to "about a minute".
const UNDER_A_MINUTE_S: f64 = 45.0;

/// "in under a minute, a mile ahead", "in about 3 minutes, 24 miles ahead":
/// a top of descent's time and distance as ONE phrase, the time derived
/// from the distance at this groundspeed so the two always agree; the
/// distance alone ("12 miles ahead") without a usable groundspeed.
///
/// EGKK-EGBB, 2026-09-27 (sortie 1790541304-full_auto +1604): "Top of
/// descent in about a minute, 1 miles ahead" at 300 kt, where a mile is
/// 12 seconds: the minutes were rounded to the nearest and floored at one,
/// so anything under 90 seconds was "about a minute" (#127). ("about 1
/// minutes" was the same rounding's older bug, 2026-09-21.)
pub(crate) fn tod_ahead_words(nm: f64, gs_kts: f64) -> String {
    let miles = spoken_miles(nm);
    if gs_kts <= 60.0 {
        return format!("{miles} ahead");
    }
    let secs = nm.max(0.0) / gs_kts * 3600.0;
    let time = if secs < UNDER_A_MINUTE_S {
        "in under a minute".to_string()
    } else {
        match (secs / 60.0).round().max(1.0) as i64 {
            1 => "in about a minute".to_string(),
            n => format!("in about {n} minutes"),
        }
    };
    format!("{time}, {miles} ahead")
}

/// Hand the active field's SURFACE wind (the sim's real-weather METAR) down
/// to the station, once a minute, off the tick. The tower picked its runway
/// by the wind datarefs, which are the air the AIRCRAFT is in: for an arrival
/// at 2,700 ft that was 350 at 8 while the field reported 190 at 3, and crew
/// and tower chose opposite ends of Exeter (2026-09-21). Above the `quiet`
/// gate on purpose: an autoland in progress is exactly when the tower's wind
/// matters.
/// Set when a callsign setting changes, so the monitor republishes the
/// resolved callsign on its next tick rather than within 5 s.
static CALLSIGN_STALE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// A callsign setting changed (`settings`: `callsign`, `airline_telephony`).
pub(crate) fn callsign_changed() {
    CALLSIGN_STALE.store(true, std::sync::atomic::Ordering::Relaxed);
}

fn feed_surface_wind() {
    // Once a minute, and AT ONCE when the station changes: the wind held
    // until then is the last field's, filtered out as not this one's, and
    // the tower answers from the winds aloft at the aircraft. Forli took the
    // station two seconds before the inbound call, read 053 at 12 at 5,000
    // ft over Ferrara, and gave runway 12 against its own 24004KT
    // (LIPE-LIPK, 2026-09-25).
    static LAST: std::sync::Mutex<Option<(std::time::Instant, String)>> = std::sync::Mutex::new(None);
    let Some((icao, lat, lon)) = crate::party_line::active_icao() else {
        return;
    };
    {
        let mut last = LAST.lock().unwrap_or_else(|e| e.into_inner());
        if last
            .as_ref()
            .is_some_and(|(t, i)| *i == icao && t.elapsed().as_secs() < 60)
        {
            return;
        }
        *last = Some((std::time::Instant::now(), icao.clone()));
    }
    tokio::task::spawn_blocking(move || {
        let wind = crate::weather::field_wind(&crate::config::xplane_root(), &icao);
        crate::party_line::set_surface_wind(lat, lon, wind);
    });
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod endurance_minutes_tests;
