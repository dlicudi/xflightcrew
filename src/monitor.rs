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
        let ap_servos_on = xp
            .get_float(autoflight::names::SERVOS_ON)
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
                crate::procedure::ACTIVE.load(std::sync::atomic::Ordering::Relaxed),
                crate::heli_route::ACTIVE.load(std::sync::atomic::Ordering::Relaxed),
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
        approach: Some(crate::autoland::ACTIVE.load(std::sync::atomic::Ordering::Relaxed)),
        automation: flying_loop(),
        autopilot: Some(s.ap_servos_on),
        crashed: Some(crashed),
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
    ) -> bool {
        // While the copilot is flying the aircraft (an active recovery/autoland),
        // stay quiet: it announces its own actions, and pilot-directed advice
        // ("lower the nose, add power") is backwards when the copilot is the one
        // doing it. The panel's "mute monitor" setting silences callouts too.
        let flying = crate::flight_state::crew_landing_or_recovering()
            || PANEL_MUTE.load(std::sync::atomic::Ordering::Relaxed);
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
            voice.alert(sev.into(), msg);
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

/// Build the checks enabled for this airframe.
/// Below this AGL the aircraft is in the landing flare, where slow-and-low is
/// *intended* — so the airspeed callout is suppressed there to avoid nagging on
/// every normal touchdown.
const FLARE_AGL_FT: f64 = 50.0;

/// Approach-to-stall airspeed callout: the copilot's "Airspeed!" — it warns and
/// prompts the recovery, it never touches the controls (intent is the pilot's;
/// auto-recovery would fight intentional slow flight). Airborne and above the
/// flare only, sustained so a momentary dip in a turn doesn't trip it. Banded
/// like the fuel check: a caution as you approach the stall, an urgent prompt
/// below it. The threshold scales with flaps (`aircraft::stall_speed_in_config`)
/// so a normal slow-but-safe flaps-down approach doesn't trip it.

/// How far ahead (seconds) to project a decaying airspeed. At a 3 kt/s washout
/// this fires the caution ~12 kt early — enough lead to call it well before a
/// mushing autopilot reaches its envelope limit and disconnects. Long enough to
/// get ahead of the decay, short enough that a brief, intentional power
/// reduction settles before it nags.
const STALL_LOOKAHEAD_S: f64 = 4.0;

/// How far ahead (seconds) to project a *building* airspeed for the overspeed
/// callout — so a descent accelerating toward redline is called with lead time,
/// not at Vne. Only a rising trend counts (see `Overspeed::eval`).
const OVERSPEED_LOOKAHEAD_S: f64 = 4.0;
/// The overspeed caution's floor, a fraction of Vne; the follower schedules
/// its cruise under it (`procedure::cruise_cap_kts`).
pub(crate) const OVERSPEED_CAUTION_FACTOR: f64 = crate::aircraft::OVERSPEED_CAUTION_FACTOR;

struct StallSpeed {
    id: &'static str,
    severity: Severity,
    /// Clean stall speed (kts) the bands scale off.
    vs_clean_kts: f64,
    /// Landing-configuration stall speed, when the profile declares one. The
    /// full-flap stall is then THIS number and the flap-adjusted speed runs
    /// straight between the two; without it the 17% trainer reduction
    /// applies, which on the A330 (Vs 175, Vso 125) put a stabilised 150 kt
    /// final inside the WARNING band the whole way down (2026-09-20).
    vs0_kts: Option<f64>,
    /// The detents that declare their own stall speed, `(ratio, kts)`
    /// (`aircraft::detent_stall_points`): a slatted wing is not a straight
    /// line (the A330's CONF 2 is 139, not 150; EGPF 2026-09-21).
    detent_stall: Vec<(f64, f64)>,
    /// Fires when IAS ≤ this × the flap-adjusted stall speed …
    at_or_below_factor: f64,
    /// … and (if set) still above this × it — the more urgent band owns below.
    above_factor: Option<f64>,
}

/// Whether a route automation owns the vertical profile, so the ring's TOD
/// and profile advisories (and the step-climb advice) keep silent: the
/// procedure follower, or the helicopter route, which flies its own approach
/// down to its gate. LEMG-LXGB (S-76, iMac 2026-09-29, #191) heard "High on
/// the profile" and "past the descent point, about 1600 feet a minute" while
/// the route was flying its descent. Pure.
pub fn follower_flying(procedure: bool, heli_route: bool) -> bool {
    procedure || heli_route
}

/// With an automation flying, the caution band's top: the margin under its
/// own target, not the hand-flown 1.2 Vs.
const AUTO_CAUTION_FACTOR: f64 = crate::aircraft::CAUTION_AUTO_RATIO;

/// The stall speed at this flap setting: a straight line from the clean
/// stall to the landing-configuration stall when the profile declares one,
/// else the trainer's fixed reduction. Never above clean, never below Vso.
#[cfg(test)]
pub fn stall_speed_at_flap(vs_clean_kts: f64, vs0_kts: Option<f64>, flap_ratio: f64) -> f64 {
    crate::aircraft::stall_speed_in_config(vs_clean_kts, vs0_kts, &[], flap_ratio)
}

impl Check for StallSpeed {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        self.severity
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        // Airborne and above the flare. Engine state is irrelevant — a glide can
        // stall too.
        if s.on_ground || s.agl_ft < FLARE_AGL_FT {
            return None;
        }
        // Stall speed for the current flap setting.
        let vs = crate::aircraft::stall_speed_in_config(
            self.vs_clean_kts,
            self.vs0_kts,
            &self.detent_stall,
            s.flap_ratio,
        );
        // Anticipate: when the speed is washing off, project it forward so the
        // band trips while there's still margin to recover. Only a *decaying*
        // trend counts (`.min(0.0)`), so this can lower the effective speed but
        // never raises it — it never masks a genuine low reading.
        //
        // With an AUTOMATION flying, the deceleration is its own, toward a
        // target it chose: the trend is not a warning and the caution band
        // is the margin under that target, not the 1.2 Vs a hand-flown
        // approach wants. A 737's Vref is 1.23 Vso, so the hand band called
        // "you're getting slow" through every normal slowdown to it, to a
        // pilot who was not flying (LEMG circuit, 2026-09-26).
        let auto = s.automation_flying;
        let caution = self.severity == Severity::Caution;
        let projected = if auto && caution {
            s.ias_kts
        } else {
            s.ias_kts + s.ias_trend_kts_per_s.min(0.0) * STALL_LOOKAHEAD_S
        };
        let at_or_below = if auto && caution { AUTO_CAUTION_FACTOR } else { self.at_or_below_factor };
        let in_band = projected <= at_or_below * vs
            && self.above_factor.is_none_or(|a| projected > a * vs);
        if !in_band {
            return None;
        }
        // Who is flying decides the words: never "you" to a pilot who is
        // not, never an order to the pilot's hands.
        Some(match (self.severity, auto) {
            (Severity::Warning, true) => "Airspeed low!".to_string(),
            (Severity::Warning, false) => "Airspeed! Lower the nose, add power.".to_string(),
            (_, true) => "Speed's getting low.".to_string(),
            (_, false) => "Airspeed, you're getting slow.".to_string(),
        })
    }
    fn min_duration(&self) -> Duration {
        // Speak quickly — a decaying airspeed needs lead time, not a long
        // confirm. Just enough to ignore a single-frame glitch (~1–2 samples at
        // the monitor's poll rate); the urgent band reacts fastest.
        match self.severity {
            Severity::Warning => Duration::from_millis(300),
            _ => Duration::from_millis(700),
        }
    }
}

/// Overspeed callout: the copilot's "Airspeed!" on the *high* side — it warns as
/// the speed builds toward Vne (e.g. a descent left at cruise power) and prompts
/// the recovery; like the stall check it never touches the controls. Airborne
/// only; banded — a caution approaching redline, an urgent prompt at or above it.
struct Overspeed {
    id: &'static str,
    severity: Severity,
    /// Never-exceed speed (kts) the bands scale off: the LOW one on an
    /// airframe with a `vmo_high:`.
    vne_kts: f64,
    /// The higher Vmo above an altitude (the Citation X's 350 over 8,000
    /// ft): the bands scale off the limit at the indicated altitude
    /// (`aircraft::vne_at`). `None` = `vne_kts` everywhere.
    vmo_high: Option<crate::aircraft::VmoHigh>,
    /// Fires when IAS ≥ this × Vne …
    at_or_above_factor: f64,
    /// … and (if set) still below this × Vne — the more urgent band owns above.
    below_factor: Option<f64>,
}

/// The overspeed warning's words, by what the aircraft is doing. "Ease off
/// the descent" was said five times to an A330 climbing at 2,600 fpm (EGPH,
/// 2026-09-20): the pilot can see the nose is up, and an instruction that is
/// visibly wrong takes the whole sentence with it.
pub fn overspeed_line(vsi_fpm: f64) -> &'static str {
    if vsi_fpm < -300.0 {
        "Overspeed! Reduce power, ease off the descent."
    } else {
        "Overspeed! Reduce power."
    }
}

impl Check for Overspeed {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        self.severity
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground {
            return None; // can't meaningfully overspeed on the ground
        }
        // Anticipate: project only a *rising* trend forward (`.max(0.0)`), so the
        // band trips with margin before redline. A decaying trend never raises the
        // projected speed, so it can't mask a falling-back-to-safe reading.
        let projected = s.ias_kts + s.ias_trend_kts_per_s.max(0.0) * OVERSPEED_LOOKAHEAD_S;
        let vne = crate::aircraft::vne_at(Some(self.vne_kts), self.vmo_high, s.alt_ft)
            .unwrap_or(self.vne_kts);
        let in_band = projected >= self.at_or_above_factor * vne
            && self.below_factor.is_none_or(|b| projected < b * vne);
        if !in_band {
            return None;
        }
        Some(match self.severity {
            Severity::Warning => overspeed_line(s.vsi_fpm).to_string(),
            _ => "Airspeed — you're getting fast.".to_string(),
        })
    }
    fn min_duration(&self) -> Duration {
        match self.severity {
            Severity::Warning => Duration::from_millis(300),
            _ => Duration::from_millis(700),
        }
    }
}

/// Which engine instrument an [`EngineHealth`] check watches.
#[derive(Clone, Copy)]
enum EngineParam {
    OilPressure,
    OilTemp,
    Cht,
}

impl EngineParam {
    fn value(self, s: &Snapshot) -> Option<f64> {
        match self {
            EngineParam::OilPressure => s.oil_pressure_psi,
            EngineParam::OilTemp => s.oil_temp_f,
            EngineParam::Cht => s.cht_f,
        }
    }
}

/// An engine instrument out of its normal range. One struct, instantiated per
/// parameter and band, mirroring the stall/overspeed pattern. `trip` is the
/// threshold to cross; `clear` (toward the more-severe side) keeps bands disjoint
/// so only one severity speaks. Thresholds are in the dataref's native unit (PSI,
/// or °C for temps); temperatures are spoken in °F to match the gauge and the POH.
struct EngineHealth {
    id: &'static str,
    severity: Severity,
    param: EngineParam,
    /// True for a low-side alarm (value below `trip`); false for high-side.
    low_side: bool,
    trip: f64,
    /// Boundary toward the next (more severe) band, so bands don't overlap.
    clear: Option<f64>,
    min_dur: Duration,
}

impl Check for EngineHealth {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        self.severity
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        // Only meaningful with the engine running and the instrument reading.
        if !s.engine_running {
            return None;
        }
        let v = self.param.value(s)?;
        let in_band = if self.low_side {
            v < self.trip && self.clear.is_none_or(|c| v >= c)
        } else {
            v > self.trip && self.clear.is_none_or(|c| v <= c)
        };
        if !in_band {
            return None;
        }
        // oil_temp_f / cht_f already read in °F; no conversion.
        let (name, unit, shown) = match self.param {
            EngineParam::OilPressure => ("Oil pressure", "pounds", v),
            EngineParam::OilTemp => ("Oil temperature", "degrees", v),
            EngineParam::Cht => ("Cylinder head temperature", "degrees", v),
        };
        let dir = if self.low_side { "low" } else { "high" };
        Some(match self.severity {
            Severity::Warning => format!("{name} {dir} — {shown:.0} {unit}. Check it now."),
            _ => format!("{name} {dir}, {shown:.0} {unit}."),
        })
    }
    fn min_duration(&self) -> Duration {
        self.min_dur
    }
}

/// Carburettor-ice advisory (predictive). The stock 172 is carburetted, and the
/// classic trap is ice forming at low power in a descent — throttle closed, moist
/// air — quietly starving the engine. Fires when conditions are ripe and carb heat
/// is off, so the copilot prompts before it bites. Advisory, not alarm.
struct CarbIce;

impl Check for CarbIce {
    fn id(&self) -> &'static str {
        "carb_ice"
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if !s.engine_running || s.carb_heat_on || s.on_ground {
            return None;
        }
        // Ice-prone: moist air (small temp–dewpoint spread) in the −10..+27 °C band,
        // at low power (throttled back for descent/idle), where icing is worst.
        let prone_temp = (-10.0..=27.0).contains(&s.oat_c);
        let humid = (s.oat_c - s.dewpoint_c) < 13.0;
        let low_power = s.rpm > 100.0 && s.rpm < 2000.0;
        (prone_temp && humid && low_power)
            .then(|| "Conditions are right for carburettor ice — recommend carb heat.".to_string())
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(10)
    }
}

/// Low vacuum: the engine-driven pump spins the attitude and heading gyros; if
/// suction drops those instruments lie. Uses the sim's low-vacuum annunciator.
/// AIRBORNE ONLY: at ground idle the pump legitimately makes too little
/// suction and the light is normal (the POH checks vacuum at run-up RPM) —
/// cautioning about it while taxiing is noise ("pointless", the pilot said,
/// 2026-07-09). In flight the RPM is up, so a lit annunciator is a real
/// failure — critical in IMC, a cross-check reminder in VMC.
struct LowVacuum;

impl Check for LowVacuum {
    fn id(&self) -> &'static str {
        "low_vacuum"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        (s.engine_running && s.low_vacuum && !s.on_ground).then(|| {
            "Low vacuum — the attitude and heading indicators may be unreliable. \
             Cross-check the compass and turn coordinator."
                .to_string()
        })
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(5)
    }
}

/// Alternator not charging: the switch is on but the ammeter shows the battery
/// discharging — a failed or declutched charging source the on/off check can't
/// see. Pairs with [`AlternatorOffline`] (switch off). Caution: shed load before
/// the battery dies. `noun` is the airframe's charging source ("alternator" on a
/// piston, "generator" on a turbine — a King Air has no alternator).
struct AlternatorNotCharging {
    noun: &'static str,
}

impl Check for AlternatorNotCharging {
    fn id(&self) -> &'static str {
        "alt_not_charging"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if !s.engine_running || !s.alternator_on {
            return None;
        }
        // A visibly producing generator overrules the battery ammeter: on
        // multi-battery airframes (MD-82) the battery bus idles at a small
        // standing discharge (~-7 A, charger cycling) with both generators
        // online, which the threshold alone reads as a failure. Verified live
        // on the Laminar MD-82: generator_amps 39/16, battery_amps -7.
        if s.generator_amps_max > 1.0 {
            return None;
        }
        (s.battery_amps < -5.0).then(|| {
            format!(
                "{noun}'s not charging — battery's draining at {amps:.0} amps. \
                 Check the {lower} and shed electrical load.",
                noun = self.noun,
                lower = self.noun.to_lowercase(),
                amps = s.battery_amps.abs()
            )
        })
    }
    /// Same start-transient sustain as [`AlternatorOffline`]: a turbine start pulls
    /// 200+ amps from the battery through the starter-generator (in *starter*
    /// mode), which the 5-second window read as a failed alternator. Only warn once
    /// the discharge persists past a normal start-to-generator-online window.
    fn min_duration(&self) -> Duration {
        ALTERNATOR_SUSTAIN
    }
}

/// Phase-expectation advisor: one profile-declared switch that should be on (or
/// off) in the listed phases — the declarative SOP table, one check per row.
/// Advisory severity: it's the copilot saying "aren't you forgetting something",
/// not an alarm. The value is read into `Snapshot::advisor_values` by index.
struct PhaseExpectation {
    /// Leaked once per profile build — check ids must be `&'static str`.
    id: &'static str,
    index: usize,
    expect_on: bool,
    phases: Vec<Phase>,
    say: String,
    /// When set, the copilot FIRES this command to fix the state itself rather
    /// than advising — the strobe/landing-light SOP rows the pilot wants done,
    /// not asked about.
    command: Option<String>,
}

impl Check for PhaseExpectation {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if !self.phases.contains(&s.phase) {
            return None;
        }
        // On the ground a switch is only owed ON with an engine running: the
        // 172's "taxi light on" row lit the lamp before the start and again
        // six seconds after the shutdown at the LIPK stand (2026-09-25).
        if self.expect_on && s.phase == Phase::Ground && !s.engine_running {
            return None;
        }
        let on = (*s.advisor_values.get(self.index)?)? > 0.5;
        (on != self.expect_on).then(|| self.say.clone())
    }
    fn fix_action(&self) -> Option<&str> {
        self.command.as_deref()
    }
    /// Generous sustain: SOP items get configured *around* phase changes, so
    /// give the pilot half a minute before speaking up.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(30)
    }
}

/// Parse a profile's lowercase phase name. Unknown names are a profile bug —
/// dropped with a log line rather than a panic.
fn parse_phase(name: &str) -> Option<Phase> {
    Some(match name {
        "ground" => Phase::Ground,
        "takeoff" => Phase::Takeoff,
        "climb" => Phase::Climb,
        "cruise" => Phase::Cruise,
        "approach" => Phase::Approach,
        "landing" => Phase::Landing,
        _ => return None,
    })
}

/// Above this the aircraft is climbing, not a level cruise with a descent
/// ahead or behind it.
const CLIMBING_FPM: f64 = 300.0;

/// Below this height a top of descent is never called: the initial climb's
/// own height (`flight::INITIAL_CLIMB_AGL_FT`). A level-off low after a
/// departure is a pattern or a terrain level, not a cruise to leave.
const TOD_MIN_AGL_FT: f64 = crate::flight::INITIAL_CLIMB_AGL_FT;

/// The top-of-descent family (`tod_ahead`, `tod_passed`) speaks only in the
/// CRUISE, by the flight model (`coarse_phase`: a level cruise, or a descent
/// still above 3,000 ft AGL) and clear of the ground. LSZS-LILO (SR22,
/// 2026-09-24): "we're past the descent point" at 51 ft AGL, five seconds
/// after liftoff. The plan's ring (SimBrief's along-route TOD, a winding
/// valley route) was longer than the 23.6 nm straight line from Samedan to
/// Sondrio, so the ring was already behind the aircraft on the runway; the
/// vertical-speed gate alone let any level moment in the climb-out speak.
fn tod_in_cruise(s: &Snapshot) -> bool {
    s.phase == Phase::Cruise && s.agl_ft >= TOD_MIN_AGL_FT
}

/// The distance the plan's ring is compared with: miles still to fly along
/// the plan's legs, since the ring is along-route (#109); the straight line
/// only when no route is known. Everything measured against `tod_ring_nm`
/// reads this, never `dest_dist_nm` directly.
fn ring_dist_nm(s: &Snapshot) -> Option<f64> {
    s.dest_route_nm.or(s.dest_dist_nm)
}

/// Descent advisor, part 1: top of descent coming up. The anticipatory version
/// of the "when is TOD" question — the copilot volunteers it while there's
/// still time to act, instead of answering after being asked. Fires once as
/// the aircraft enters the 12-nm run-up to the planned TOD, level at altitude.
/// Within this of the flight's peak the aircraft is still at its cruise.
const CRUISE_BAND_FT: f64 = 1000.0;

struct TodAhead;

impl Check for TodAhead {
    fn id(&self) -> &'static str {
        "tod_ahead"
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        // Only when level: a climb or descent has its own plan. A CLIMB is
        // never announced a top of descent (09-24: "top of descent 11 miles
        // ahead" three times at 1,000 ft climbing out of a short hop).
        if s.on_ground || s.vsi_fpm.abs() > 500.0 || s.vsi_fpm > CLIMBING_FPM {
            return None;
        }
        // A cruise, by the flight model, and clear of the ground.
        if !tod_in_cruise(s) {
            return None;
        }
        // A follower flying: ITS point, to ITS altitude (LEJR-LXGB,
        // 2026-09-14: the ring said "past", the hands waited 3 more minutes).
        if let Some(p) = &s.follower_tod {
            if p.started || !(0.0..=12.0).contains(&p.to_tod_nm) {
                return None;
            }
            // THE top of descent is the one that leaves the cruise. The
            // follower publishes a point for every stepped leg, and each was
            // called: seven "Descent coming up" down one arrival, from
            // 34,400 to 8,800 feet (LIBP-LEMG, 2026-09-26). Below the
            // cruise the steps are flown, not announced.
            if s.flight_peak_ft.is_some_and(|peak| s.alt_ft < peak - CRUISE_BAND_FT) {
                return None;
            }
            let to_tod = p.to_tod_nm;
            let alt = crate::auto_status::commas((p.target_ft / 100.0).round() as i64 * 100);
            // The pilot's altitude holds the follower: the plan's descent,
            // said once as the plan's, with who holds the level.
            if let Some(h) = p.held_ft {
                let held = crate::auto_status::commas((h / 100.0).round() as i64 * 100);
                return Some(format!(
                    "The plan starts down to {alt} feet for {}, {} ahead. You've got {held} held, give me an altitude when you want to go down.",
                    p.fix,
                    spoken_miles(to_tod)
                ));
            }
            return Some(format!(
                "Descent coming up. Top of descent {}, down to {alt} feet.",
                tod_ahead_words(to_tod, s.groundspeed_kts)
            ));
        }
        if s.follower_flying {
            return None; // the follower's point or nothing — see `follower_flying`
        }
        let (dist, elev, ring) = (ring_dist_nm(s)?, s.dest_elev_ft?, s.tod_ring_nm?);
        if s.alt_ft < elev + 6000.0 {
            return None; // already low; TOD math is meaningless
        }
        let to_tod = dist - ring;
        if !(0.0..=12.0).contains(&to_tod) {
            return None;
        }
        Some(format!(
            "Descent coming up. Top of descent {}.",
            tod_ahead_words(to_tod, s.groundspeed_kts)
        ))
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(3)
    }
}

/// Descent advisor, part 2: past the planned TOD and still level — the exact
/// failure of the LEAL–LEMD flight (TOD slipped past unannounced; the descent
/// started late, steep, and fast). Says what rate recovers the profile, so the
/// fix is in the same breath as the problem.
struct TodPassed;

impl Check for TodPassed {
    fn id(&self) -> &'static str {
        "tod_passed"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        // Descending: moot. Climbing: "still level" is false, and on a short
        // hop the ring can be behind before the climb ends (09-24: "past the
        // descent point" at 51 ft AGL, climbing out).
        if s.on_ground || s.vsi_fpm < -400.0 || s.vsi_fpm > CLIMBING_FPM {
            return None;
        }
        // And only in the cruise, clear of the ground (`tod_in_cruise`): the
        // same flight's ring was behind it from the runway.
        if !tod_in_cruise(s) {
            return None;
        }
        // A follower flying is judged against ITS OWN point: a miss is the
        // follower a mile past it with the descent still uncommanded, never
        // the plan's ring passing while the hands are correctly waiting.
        // Nor a level the PILOT is holding it at (`held_ft`): that is a
        // choice, reminded once by `TodAhead`, not a missed descent.
        if let Some(p) = &s.follower_tod {
            if p.started || p.held_ft.is_some() || p.to_tod_nm > -1.0 || s.groundspeed_kts < 60.0 {
                return None;
            }
            let height = s.alt_ft - p.target_ft;
            let mins = (p.dist_nm - 1.0).max(2.0) / s.groundspeed_kts * 60.0;
            let fpm = ((height / mins) / 100.0).round() * 100.0;
            let alt = crate::auto_status::commas((p.target_ft / 100.0).round() as i64 * 100);
            return Some(format!(
                "We're past the descent point and still level. About {fpm:.0} feet a minute makes {alt} feet at the fix."
            ));
        }
        if s.follower_flying {
            return None; // the follower's point or nothing — see `follower_flying`
        }
        let (dist, elev, ring) = (ring_dist_nm(s)?, s.dest_elev_ft?, s.tod_ring_nm?);
        if s.alt_ft < elev + 4000.0 || dist >= ring - 1.0 || s.groundspeed_kts < 60.0 {
            return None;
        }
        // Rate that reaches ~2,000 ft over the field with 5 nm to spare.
        let height = s.alt_ft - (elev + 2000.0);
        let mins = (dist - 5.0).max(2.0) / s.groundspeed_kts * 60.0;
        let fpm = ((height / mins) / 100.0).round() * 100.0;
        Some(format!(
            "We're past the descent point. About {fpm:.0} feet a minute makes the profile."
        ))
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(5)
    }
}

/// Descent advisor, part 3: descending but above the slope. Uses a 300 ft/nm
/// profile to a point 8 nm out (deceleration allowance) — deliberately a touch
/// above the TOD ring's slope so a normal just-started descent never trips it,
/// plus a 1,500 ft margin. High *and* fast is when the boards call gets added.
struct HighOnProfile;

impl Check for HighOnProfile {
    fn id(&self) -> &'static str {
        "high_on_profile"
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || s.vsi_fpm > -400.0 {
            return None; // only mid-descent
        }
        // A descent from the cruise or on the arrival, clear of the ground:
        // never the takeoff, the climb or the flare (`coarse_phase`).
        let descending_phase = matches!(s.phase, Phase::Cruise | Phase::Approach);
        if !descending_phase || s.agl_ft < TOD_MIN_AGL_FT {
            return None;
        }
        // The follower's profile is its own (700 fpm to the next fix, not
        // 300 ft/nm to the field): judging it against the ring's slope would
        // call a correct descent high.
        if s.follower_tod.is_some() || s.follower_flying {
            return None;
        }
        let (dist, elev, ring) = (ring_dist_nm(s)?, s.dest_elev_ft?, s.tod_ring_nm?);
        if dist > ring + 5.0 || dist < 4.0 {
            return None; // not on the arrival yet / inside the final where GPWS owns it
        }
        let profile_alt = elev + 2000.0 + (dist - 8.0).max(0.0) * 300.0;
        let excess = s.alt_ft - profile_alt;
        if excess < 1500.0 {
            return None;
        }
        let rounded = (excess / 100.0).round() * 100.0;
        Some(if s.ias_kts > 240.0 {
            format!(
                "High on the profile, about {rounded:.0} feet above the slope, and fast. Boards would help."
            )
        } else {
            format!("High on the profile, about {rounded:.0} feet above the slope.")
        })
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(8)
    }
}

/// GPWS-style sink-rate guard (mode 1, simplified): excessive descent rate close
/// to the ground. The allowance scales with height, so a normal 700-fpm final
/// never trips it but a 2,000-fpm descent through 500 AGL does. Born of an
/// LEAL–LEMD arrival that flew a 3,400-fpm dive into terrain with no callout:
/// the threshold at 300 AGL is 1,800 fpm, which would have fired ~20 seconds out.
struct SinkRate;

impl Check for SinkRate {
    fn id(&self) -> &'static str {
        "sink_rate"
    }
    fn severity(&self) -> Severity {
        Severity::Warning
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || s.agl_ft > 2500.0 {
            return None;
        }
        let limit = 1500.0 + s.agl_ft; // fpm of sink allowed at this height
        (s.vsi_fpm < -limit).then(|| {
            format!(
                "Sink rate! {vsi:.0} feet a minute at {agl:.0} feet. Pull up.",
                vsi = s.vsi_fpm.abs(),
                agl = s.agl_ft
            )
        })
    }
    /// React fast — two ticks at 3 Hz, enough to skip a single glitched read.
    fn min_duration(&self) -> Duration {
        Duration::from_millis(600)
    }
}

/// Altitude bust: the autopilot is flying *through* the dialed altitude instead
/// of capturing it. On airframes whose AP never arms the preselector (the
/// Laminar MD-82's dial is display-only to the sim AP), a dialed-in 5,000 simply
/// doesn't level the descent — this is the check that turns that silent
/// non-capture into a callout. Gated off in landing configuration (gear down or
/// big flap): on final the dial legitimately holds the missed-approach altitude
/// while the aircraft descends below it.
struct AltitudeBust;

impl Check for AltitudeBust {
    fn id(&self) -> &'static str {
        "altitude_bust"
    }
    fn severity(&self) -> Severity {
        Severity::Warning
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || !s.ap_servos_on || s.gear_down || s.flap_ratio > 0.6 {
            return None;
        }
        let dial = s.ap_alt_dial_ft?;
        let diff = s.alt_ft - dial;
        let busted = (s.vsi_fpm < -400.0 && diff < -200.0) // descending through it
            || (s.vsi_fpm > 400.0 && diff > 200.0); // climbing through it
        busted.then(|| {
            format!(
                "Through the selected altitude, {dial:.0} set, passing {alt:.0}. Check altitude capture.",
                alt = s.alt_ft
            )
        })
    }
    /// A couple of seconds: enough to ride out the normal capture overshoot,
    /// short enough to matter with terrain coming up.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(3)
    }
}

/// Hypoxia guard: cabin altitude climbing toward the level where the pilot greys
/// out (the blackout you get unpressurized/without oxygen up high). Banded so the
/// caution and warning don't both fire — a caution as it passes ~10,000 ft, then a
/// warning past 12,500 (the regulatory oxygen line, and X-Plane's own annunciator
/// threshold). Airborne only; on an unpressurized airframe the cabin altitude just
/// tracks the real altitude, so this covers the GA singles too.
fn cabin_message(pressurised: Option<bool>, warning: bool) -> &'static str {
    match (pressurised, warning) {
        (Some(false), false) => "We're unpressurised and above ten thousand feet. Check oxygen or descend when terrain permits.",
        (Some(false), true) => "Unpressurised cabin above twelve thousand five hundred. Oxygen now, or descend when terrain permits.",
        (Some(true), false) => "Cabin altitude above ten thousand. Check pressurisation and oxygen.",
        (_, true) => "Cabin altitude above twelve thousand five hundred. Oxygen now, or descend when terrain permits.",
        (None, false) => "Cabin altitude above ten thousand. Check oxygen or descend when terrain permits.",
    }
}

struct CabinAltitude {
    id: &'static str,
    severity: Severity,
    /// [low, high) band, feet. `high = None` means "and above".
    band_ft: (f64, Option<f64>),
    message: &'static str,
    min_dur: Duration,
}

impl Check for CabinAltitude {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        self.severity
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        let (low, high) = self.band_ft;
        let in_band = s.cabin_altitude_ft >= low && high.is_none_or(|h| s.cabin_altitude_ft < h);
        (!s.on_ground && in_band).then(|| self.message.to_string())
    }
    fn repeat_level(&self, s: &Snapshot) -> Option<i64> {
        s.cabin_altitude_ft
            .is_finite()
            .then(|| (s.cabin_altitude_ft / 1000.0).floor() as i64)
    }
    fn min_duration(&self) -> Duration {
        self.min_dur
    }
}

/// Rolling with the parking brake set — dragging the brakes on a taxi/takeoff
/// roll. Caught twice live on the C90B, where the 3D handle doesn't reflect the
/// sim-level brake (a stray 'b' key sets it invisibly). Groundspeed floor keeps
/// it quiet at a standstill and during brake-on run-ups.
struct RollingOnParkBrake;

impl Check for RollingOnParkBrake {
    fn id(&self) -> &'static str {
        "rolling_park_brake"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        // Not when the copilot set it: the taxi-in arrests a rolling handoff
        // on the parking brake and says so itself (see `taxi_in::arrest_holding`).
        // Nor while a crew brake command is in flight: a rejected takeoff's
        // `brakes_max` reads 0.7 to 1.0 on this ratio at 60 to 90 kt and
        // decays to zero once it expires (1790524628: 1.0 at the stop, 0.4
        // two seconds later, 0.0 by four); the caution said "release it"
        // mid-reject at EGKK three times (2026-09-27). This check is for the
        // brake a PILOT forgot, which reads set with our brakes quiet.
        (s.on_ground
            && s.groundspeed_kts > 5.0
            && s.park_brake > 0.5
            && !s.crew_brake
            && !crate::taxi_in::arrest_holding())
        .then(|| "Parking brake is set and we're rolling — release it.".to_string())
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(2)
    }
}

/// Fuel selector OFF with the engine running — it's burning what's left in the
/// lines. Seen live on the SR22 after a plugin reload (the xlua re-init snaps the
/// selector to OFF mid-flight), and it's the forgotten-tank failure mode generally
/// on selector-only airframes (the SR22 has no BOTH). Warning: minutes from a
/// quiet engine.
struct FuelSelectorOff;

impl Check for FuelSelectorOff {
    fn id(&self) -> &'static str {
        "fuel_selector_off"
    }
    fn severity(&self) -> Severity {
        Severity::Warning
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        let sel = s.fuel_selector?;
        (s.engine_running && sel < 0.5).then(|| {
            "Fuel selector is off with the engine running — select a tank now.".to_string()
        })
    }
    /// Short sustain so a selector transiting through OFF between tanks doesn't
    /// trip it, but the callout still lands well before the lines run dry.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(3)
    }
}

const BARO_KNOB: &str = "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot";

/// The altimeter against what it should read (local QNH below the transition
/// altitude, standard above; `instruments::correct_setting_inhg`). Every
/// automation flies the INDICATED altitude (LINTO, 2026-09-14), so a wrong
/// setting is a wrong altitude: with an automation flying the copilot RESETS
/// the knob and says so; with the pilot flying it is the pilot's knob, said
/// once per episode and never re-asserted (the cockpit_auto rule). Real
/// weather walks the QNH a hundredth at a time; the tolerance and the sustain
/// keep that from becoming chatter.
///
/// Two instances: the drift check (`crossing: false`, 20 s) and the
/// transition itself (`crossing: true`, [`ALTIMETER_CROSSING_SUSTAIN_MS`]):
/// crossed in the direction flown, the setting is owed AT the transition
/// (`instruments::transition_crossed`, #156), and the drift check leaves
/// that case to it so one knob is never said twice.
struct AltimeterSetting {
    transition_altitude_ft: f64,
    crossing: bool,
}

/// The transition crossing's sustain: a single-sample altimeter glitch is
/// not a crossing; two seconds at 2,000 fpm is under 70 ft past it.
const ALTIMETER_CROSSING_SUSTAIN_MS: u64 = 2_000;

impl AltimeterSetting {
    fn wanted(&self, s: &Snapshot) -> Option<(f64, f64)> {
        let ta = crate::tools::instruments::transition_in_force_ft(self.transition_altitude_ft);
        altimeter_wanted(s, ta, self.crossing)
    }
}

/// (knob, wanted) when the altimeter should be reset against transition
/// `ta` (the one in force), for the drift check or the crossing one. Pure.
fn altimeter_wanted(s: &Snapshot, ta: f64, crossing: bool) -> Option<(f64, f64)> {
    use crate::tools::instruments as ins;
    if !s.engine_running {
        return None;
    }
    let (baro, pa) = (s.baro_inhg?, s.qnh_pa?);
    if pa <= 0.0 {
        return None;
    }
    let crossed = ins::transition_crossed(s.alt_ft, s.vsi_fpm, ta, baro, pa);
    if crossing {
        return crossed.map(|want| (baro, want));
    }
    if crossed.is_some() {
        return None;
    }
    let want = ins::correct_setting_inhg(s.alt_ft, ta, pa);
    let tol = ins::ALTIMETER_TOLERANCE_INHG;
    let qnh = (pa / ins::PA_PER_INHG).clamp(28.0, 31.5);
    let std = ins::STANDARD_INHG;
    // Near the transition either setting is right: hovering at FL180 on a
    // FL180 transition reset the knob QNH, standard, QNH inside two
    // minutes (EDLP-LOWI, 2026-09-24). Inside the band, a knob on either
    // one is left alone; the change is made clear of it.
    if in_transition_band(s.alt_ft, ta)
        && ((baro - qnh).abs() < tol || (baro - std).abs() < tol)
    {
        return None;
    }
    // Just through it the right way, the knob's own jump can put the
    // altimeter back across (standard set at 6,000 on a 30.50 QNH reads
    // 5,440): the setting the direction flown owes is left alone within
    // the knob's jump of the transition.
    let jump_ft = (qnh - std).abs() * ins::FT_PER_INHG + 100.0;
    let near = (s.alt_ft - ta).abs() < jump_ft.max(TRANSITION_BAND_FT);
    let climbing = s.vsi_fpm >= ins::TRANSITION_CROSSING_FPM;
    let descending = s.vsi_fpm <= -ins::TRANSITION_CROSSING_FPM;
    if near
        && ((climbing && (baro - std).abs() < tol) || (descending && (baro - qnh).abs() < tol))
    {
        return None;
    }
    ((baro - want).abs() >= tol).then_some((baro, want))
}

/// Feet either side of the transition where either setting stands: more than
/// the ~260 ft the knob change itself moves the indicated altitude, which is
/// what made it flip.
const TRANSITION_BAND_FT: f64 = 500.0;

fn in_transition_band(alt_ft: f64, transition_ft: f64) -> bool {
    (alt_ft - transition_ft).abs() < TRANSITION_BAND_FT
}

impl Check for AltimeterSetting {
    fn id(&self) -> &'static str {
        if self.crossing {
            "altimeter_transition"
        } else {
            "altimeter_setting"
        }
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        let (baro, want) = self.wanted(s)?;
        let spoken = crate::tools::instruments::spoken_setting;
        Some(if s.automation_flying {
            format!("Altimeter reset, {}.", spoken(want))
        } else {
            let what = if (want - crate::tools::instruments::STANDARD_INHG).abs() < 0.005 {
                "standard"
            } else {
                "QNH"
            };
            format!(
                "Altimeter check, {what} is {}, you have {}.",
                spoken(want),
                spoken(baro)
            )
        })
    }
    /// The pilot turning the knob passes through wrong values on the way to
    /// the right one; a weather edition moves the QNH under a set knob. The
    /// transition itself is owed at once.
    fn min_duration(&self) -> Duration {
        if self.crossing {
            Duration::from_millis(ALTIMETER_CROSSING_SUSTAIN_MS)
        } else {
            Duration::from_secs(20)
        }
    }
    fn fix_write(&self, s: &Snapshot) -> Option<(&'static str, f64)> {
        if !s.automation_flying {
            return None;
        }
        self.wanted(s).map(|(_, want)| (BARO_KNOB, want))
    }
}

/// A centre tank empty with its pumps still on and an engine running
/// (`tools::fuel_pumps::pumps_to_switch_off`, the one decision). The stock
/// 737's scripted start switches the CTR pumps on over an empty centre, and
/// the load sheet fills the wings first, so a short sector flew with both
/// CTR LOW PRESSURE lights lit for the whole flight (2026-09-27). The
/// altimeter's shape: with an automation flying the crew switches them off
/// (`fix_write`, through `fuel_pumps::switch_pumps`, read back); with the
/// pilot flying it is the pilot's fuel panel, said once per episode.
pub(crate) const CENTRE_TANK_DRY: &str = "centre_tank_dry";

struct CentreTankDry {
    declared: Vec<usize>,
}

impl CentreTankDry {
    fn tanks(&self, s: &Snapshot) -> Vec<usize> {
        crate::tools::fuel_pumps::pumps_to_switch_off(
            &self.declared,
            s.fuel_tanks.as_ref(),
            s.engine_running,
        )
    }
}

impl Check for CentreTankDry {
    fn id(&self) -> &'static str {
        CENTRE_TANK_DRY
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if self.tanks(s).is_empty() {
            return None;
        }
        Some(if s.automation_flying {
            "Centre tank's empty, centre pumps off.".to_string()
        } else {
            "Centre tank's empty, centre pumps are still on.".to_string()
        })
    }
    /// A few seconds, so a pump the pilot is reaching for, or an engine just
    /// lit, is not a callout.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(CENTRE_DRY_SUSTAIN_S)
    }
    fn fix_write(&self, s: &Snapshot) -> Option<(&'static str, f64)> {
        if !s.automation_flying || self.tanks(s).is_empty() {
            return None;
        }
        Some((crate::tools::fuel_pumps::TANK_PUMP_ON, 0.0))
    }
}

/// Seconds an empty centre with its pumps on must hold before it is said.
const CENTRE_DRY_SUSTAIN_S: u64 = 5;

/// The altimeter against where the aircraft IS (#94). EGSS (2026-09-22) flew
/// 4,675 ft indicated at 16,884 true; every automation flies the altimeter,
/// so only a cross-check against the true height sees the altimeter itself
/// is wrong. The knob's part is `AltimeterSetting`'s; this speaks only what
/// the knob and the air's temperature do NOT explain (a static port, a
/// lying dial), airborne, sustained. Advisory: the pilot's instruments.
pub(crate) fn altimeter_truth_of(
    s: &Snapshot,
    transition_altitude_ft: f64,
) -> Option<crate::tools::instruments::AltimeterTruth> {
    use crate::tools::instruments as ins;
    let (baro, pa, truth) = (s.baro_inhg?, s.qnh_pa?, s.alt_true_ft?);
    if pa <= 0.0 || baro <= 0.0 {
        return None;
    }
    let ta = ins::transition_in_force_ft(transition_altitude_ft);
    let reference = ins::correct_setting_inhg(s.alt_ft, ta, pa);
    Some(ins::altimeter_truth(
        s.alt_ft,
        truth,
        baro,
        pa / ins::PA_PER_INHG,
        reference,
        s.oat_c,
    ))
}

struct AltimeterTruthCheck {
    transition_altitude_ft: f64,
}

impl Check for AltimeterTruthCheck {
    fn id(&self) -> &'static str {
        "altimeter_disagrees"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground {
            return None;
        }
        let t = altimeter_truth_of(s, self.transition_altitude_ft)?;
        // A gross setting is the knob, said and fixed by its own check.
        if t.setting_gross() || !t.instrument_suspect() {
            return None;
        }
        let off = (t.residual_ft.abs() / 100.0).round() * 100.0;
        let way = if t.residual_ft > 0.0 { "higher" } else { "lower" };
        Some(format!(
            "Altimeter disagrees with GPS altitude, we're about {off:.0} feet {way} than it says. \
             Cross-check the standby."
        ))
    }
    /// A climb or descent rate lags the static system for a few seconds.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(30)
    }
}

/// Icing conditions and the anti-ice switches (#100,
/// `tools::anti_ice::practice_wants`, the one decision): probes, windows,
/// engines and props on in visible moisture at +10 C or colder, the wing
/// once ice accretes. The altimeter's shape: with an automation flying the
/// crew switches them (`fix_write` sentinel, through `anti_ice::monitor_fix`,
/// read back); with the pilot flying it is said once per episode.
pub(crate) const ICING: &str = "icing";

struct Icing;

impl Check for Icing {
    fn id(&self) -> &'static str {
        ICING
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || !s.engine_running {
            return None;
        }
        let r = s.icing.as_ref()?;
        let wants = crate::tools::anti_ice::practice_wants(r);
        if wants.is_empty() {
            return None;
        }
        Some(crate::tools::anti_ice::practice_line(
            r,
            &wants,
            s.automation_flying && crate::mode::crew_actuates(),
        ))
    }
    /// A cloud passed through in seconds is not icing conditions.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(20)
    }
    fn fix_write(&self, s: &Snapshot) -> Option<(&'static str, f64)> {
        if !s.automation_flying {
            return None;
        }
        let r = s.icing.as_ref()?;
        (!crate::tools::anti_ice::practice_wants(r).is_empty())
            .then_some((crate::tools::anti_ice::MONITOR_FIX, 1.0))
    }
}

/// Seconds clear of icing before the crew switches off what it switched on.
const ICING_CLEAR_S: u64 = 300;

/// Seconds between ice reads (a slow clock: the API is frame time).
const ICING_READ_S: u64 = 10;

/// Air this cold or colder is worth reading the ice picture in.
const ICING_READ_OAT_C: f64 = 12.0;

/// Cruising further below the optimum altitude than a step is worth saying.
///
/// The altitude a jet flies best at rises as fuel burns off — roughly a level
/// an hour on a narrowbody — and the plan picks ONE level before departure and
/// holds it to the destination. On a long sector that is a slow, silent fuel
/// leak: EIDW-BIKF (2026-09-20) planned FL340, and by mid-Atlantic at 62.9 t
/// the optimum was 38,600 — 4,600 ft up, with the jet stream core so flat
/// across FL300-FL390 that the climb would have cost one knot of headwind.
///
/// Advisory, and advisory ONLY. A level change belongs to ATC: the copilot
/// says what it would ask for and the pilot asks. There is no `fix_write`
/// here on purpose, and it is `Advisory` not `Caution` because nothing is
/// wrong — this is money, not safety.
struct StepClimb {
    reference: crate::aircraft::ReferenceProfile,
}

impl StepClimb {
    /// The level to ask for, and the optimum it is chasing. `None` unless we
    /// are actually IN level cruise on an airframe that declares an optimum:
    /// a climb or descent is already going somewhere, and advising a step in
    /// the middle of one is noise.
    fn wanted(&self, s: &Snapshot) -> Option<(f64, f64)> {
        // A LEVEL THE PLAN HANDED DOWN IS NOT A LEVEL TO ARGUE WITH. The
        // follower publishes its own top of descent for the leg it is
        // flying; while that exists the aircraft is going DOWN shortly, on
        // the plan's own constraint, and money saved by climbing first is
        // money nobody can spend. EGLF-LFPB (2026-09-22) said "sixteen
        // thousand feet below optimum, flight level two five zero would burn
        // less" - twice - at FL230, which the plan had just brought us down
        // to from FL250, with the descent advisory already given.
        //
        // And the plan's level is the plan's on EVERY leg, not only one
        // with a descent published: `follower_tod` is `None` on a leg whose
        // constraint is not below us, so EGBB-EGKK (2026-09-27) heard the
        // advice at +1405 s with the follower holding the planned 15,000 ft
        // and its descent 9 minutes off.
        if s.follower_tod.is_some() || s.follower_flying {
            return None;
        }
        if s.phase != Phase::Cruise || s.on_ground || s.vsi_fpm.abs() > CRUISE_WANDER_FPM {
            return None;
        }
        // A climb pays back only over the cruise left to fly at the new
        // level; minutes before the top of descent it is fuel spent going
        // up to come straight down.
        if step_cruise_left_min(s).is_some_and(|m| m < STEP_CLIMB_MIN_CRUISE_MIN) {
            return None;
        }
        let optimum = crate::aircraft::optimum_altitude_ft(&self.reference, s.all_up_kg?)?;
        crate::aircraft::step_climb_ft(s.alt_ft, optimum).map(|step| (step, optimum))
    }
}

/// Whether the crew's region note ("Closest field is Benson, tower is on
/// ...") is spoken. It is VOLUNTEERED: nobody asked, and the station swaps
/// behind it either way. On the ground it is the relocation note (you loaded
/// the flight at a new field) and always said. Airborne it is worth a word
/// when something is wrong, since the closest field is then the question,
/// and otherwise only to a chatty copilot (`persona` gates what is
/// volunteered, never what is owed). EGBB-EGKK (2026-09-27) heard it four
/// times in nine minutes, Benson, Heathrow, Farnborough, Gatwick, over a
/// normal descent with the follower flying. Pure.
fn region_note_said(on_ground: bool, abnormal: bool, chatter: crate::persona::Chatter) -> bool {
    on_ground || abnormal || chatter == crate::persona::Chatter::Chatty
}

/// Minutes of cruise left before the top of descent under which a step climb
/// is not worth asking for. A 2,000 ft step costs a couple of minutes at climb
/// power and saves a percent or two of the cruise burn after it: over half an
/// hour that is money, over the ten minutes EGBB-EGKK had left it is not.
const STEP_CLIMB_MIN_CRUISE_MIN: f64 = 30.0;

/// Minutes to the top of descent at this groundspeed: to the plan's ring when
/// one is published, else to 3 nm per 1,000 ft above the field. `None`
/// without a destination or a groundspeed (then nothing is gated on it).
/// Pure.
fn step_cruise_left_min(s: &Snapshot) -> Option<f64> {
    let dist = ring_dist_nm(s)?;
    if s.groundspeed_kts <= 60.0 {
        return None;
    }
    let ring = s.tod_ring_nm.unwrap_or_else(|| {
        (s.alt_ft - s.dest_elev_ft.unwrap_or(0.0)).max(0.0) / 1000.0 * 3.0
    });
    Some((dist - ring).max(0.0) / s.groundspeed_kts * 60.0)
}

/// A level cruise under an autopilot wanders, and every crossing of this gate
/// ENDS the advisory's episode and re-arms it. At 300 fpm the 737's own
/// level hold tripped it twice in six minutes and said the same thing three
/// times (EGLF-LFPB, 2026-09-22). The phase gate above already decides what
/// counts as cruise; this only has to ignore the wobble inside it.
const CRUISE_WANDER_FPM: f64 = 600.0;

impl Check for StepClimb {
    fn id(&self) -> &'static str {
        "step_climb"
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        let (step, optimum) = self.wanted(s)?;
        // What the wind does up there is the whole decision, so say it when we
        // know: a step into a stronger headwind can cost more than the
        // altitude saves, and a step into the SAME wind is free money. On
        // EIDW-BIKF the jet stream was flat across the band and the climb was
        // worth one knot, which nobody had because nobody looked.
        let wind = s.winds_aloft.as_ref().and_then(|w| {
            let (here, there) = (w.at_ft(s.alt_ft)?, w.at_ft(step)?);
            let change = there.1 - here.1;
            Some(if change.abs() < 5.0 {
                " The wind up there is much the same.".to_string()
            } else if change < 0.0 {
                format!(" The wind eases {:.0} knots up there.", -change)
            } else {
                format!(" It's {:.0} knots windier up there.", change)
            })
        });
        Some(format!(
            "We're {} feet below optimum at this weight. Flight level {} would burn less if \
             you can get it.{}",
            crate::auto_status::commas(((optimum - s.alt_ft) / 100.0).round() as i64 * 100),
            crate::circuit::spell_runway(&format!("{:03}", (step / 100.0).round() as i64)),
            wind.unwrap_or_default(),
        ))
    }
    /// Cruise altitude wanders a little under the AP and the weight falls
    /// steadily; a long sustain keeps this to one considered word rather than
    /// a nag at the threshold.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(120)
    }
    /// ...and when it does speak again it must have something NEW to say.
    /// Keyed on the level it would ask for, so burning off the weight into a
    /// higher step is worth a word and holding the same cruise is not.
    fn repeat_level(&self, s: &Snapshot) -> Option<i64> {
        self.wanted(s)
            .map(|(step, _)| (step / 1000.0).round() as i64)
    }
}

/// The tank picture for the centre-pump check: fuel and pumps every tick
/// (batched), the lateral positions once per airframe. All or nothing; a
/// declared tank the geometry says is a main tank is logged, never used.
async fn read_fuel_tanks(
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

fn build_checks(profile: &AircraftProfile) -> Vec<Box<dyn Check>> {
    let mut checks: Vec<Box<dyn Check>> = vec![
        // Universal: every airframe can roll with the brake set.
        Box::new(RollingOnParkBrake),
        // Universal terrain guard: any airframe can dive into the ground.
        Box::new(SinkRate),
        // Descent advisor — armed only when a flight plan gave us a destination
        // (the checks read `Snapshot::dest_*`, `None` without one).
        Box::new(TodAhead),
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
    }));
    checks.push(Box::new(CabinAltitude {
        id: "cabin_alt_warning",
        severity: Severity::Warning,
        band_ft: (12_500.0, None),
        message: cabin_message(profile.pressurised, true),
        min_dur: Duration::from_secs(4),
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
        checks.push(Box::new(StallSpeed {
            id: "airspeed_caution",
            severity: Severity::Caution,
            vs_clean_kts: vs,
            vs0_kts: vs0,
            detent_stall: detent_stall.clone(),
            at_or_below_factor: crate::aircraft::CAUTION_HAND_RATIO,
            above_factor: Some(crate::aircraft::STALL_WARNING_RATIO),
        }));
        checks.push(Box::new(StallSpeed {
            id: "airspeed_warning",
            severity: Severity::Warning,
            vs_clean_kts: vs,
            vs0_kts: vs0,
            detent_stall,
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
static PANEL_MUTE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// The live flight strip shown on the plugin panel: the active emergency line
/// (if any), then cabin altitude airborne, then fuel endurance. " | " separators
/// (ASCII-safe for the panel font). Empty hides the row.
/// Round to 0.1 for compact black-box records.
/// The pilot's controls, the levers and the conditions, for the `state`
/// record's `in` object. A failed read is `None`, never a number.
#[derive(Debug, Clone, Default)]
struct Inputs {
    throttle: Option<f64>,
    beta: Option<f64>,
    brake_l: Option<f64>,
    brake_r: Option<f64>,
    yoke_pitch: Option<f64>,
    yoke_roll: Option<f64>,
    yoke_yaw: Option<f64>,
    ap_on: Option<f64>,
    ap_state: Option<f64>,
    wind_dir: Option<f64>,
    wind_kts: Option<f64>,
    sim_speed: Option<f64>,
    trq: Option<f64>,
    steer_yield: Option<f64>,
    pitch_yield: Option<f64>,
    brake_yield: Option<f64>,
    /// A crew loop's own brake command is in flight (held, or inside its
    /// expiry and decay tail, `tools::brake_cmd_quiet`): the brake the
    /// plugin sees and the parking-brake ratio are OURS this tick.
    crew_brake: bool,
    /// The pitch channel to the tail, read airborne while the servos or our
    /// own column fly it (#181: the servo moved nothing and nothing logged
    /// it); `None` = not read this tick.
    pitch: Option<PitchInputs>,
}

/// The `state` record's `in.pitch`: the autopilot's servo, the elevator
/// SURFACE (degrees, trailing edge down positive), the summed column, the
/// trim, the dials the autopilot flies and who overrides the pitch. A failed
/// read is `None`.
#[derive(Debug, Clone, Copy, Default)]
struct PitchInputs {
    servo: Option<f64>,
    elev_deg: Option<f64>,
    total: Option<f64>,
    trim: Option<f64>,
    vs_dial: Option<f64>,
    alt_hold_ft: Option<f64>,
    ovr_pitch: Option<f64>,
    ovr_fd_pitch: Option<f64>,
}

impl PitchInputs {
    fn json(&self) -> serde_json::Value {
        let r3 = |v: Option<f64>| v.map(|x| (x * 1000.0).round() / 1000.0);
        serde_json::json!({
            "servo": r3(self.servo), "elev": self.elev_deg.map(|x| (x * 100.0).round() / 100.0),
            "tot": r3(self.total), "trim": r3(self.trim),
            "vs_dial": self.vs_dial.map(r1), "alt_hold": self.alt_hold_ft.map(r1),
            "ovr": self.ovr_pitch.map(|v| v > 0.5), "ovr_fd": self.ovr_fd_pitch.map(|v| v > 0.5),
        })
    }

    fn telemetry(&self) -> crate::climb_watch::PitchTelemetry {
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
fn pitch_reads_due(on_ground: bool, servos_on: bool, crew_yoke: bool) -> bool {
    !on_ground && (servos_on || crew_yoke)
}

impl Inputs {
    fn json(&self) -> serde_json::Value {
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
async fn read_pitch(xp: &crate::xplane::XPlaneClient) -> PitchInputs {
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

async fn read_inputs(xp: &crate::xplane::XPlaneClient, pitch_due: bool) -> Inputs {
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
                xp.get_float(autoflight::names::AP_STATE),
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
const YOKE_HANDS: f64 = 0.15;

/// How long a write to the crew's own yoke channels (`xcopilot/pitch`,
/// `xcopilot/roll`) owns the yoke reading: the plugin applies a write for
/// 1 s, the monitor reads the yoke some ticks later.
const CREW_YOKE_FRESH_S: f64 = 1.5;

fn process_secs() -> f64 {
    static EPOCH: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    EPOCH.get_or_init(std::time::Instant::now).elapsed().as_secs_f64()
}

/// Millis since the process epoch of the last crew yoke write, 0 = never.
static CREW_YOKE_MS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

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
static PILOT_RELEASE_MS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
/// How long a pilot's own "my controls" answers the automation-stopped
/// watch: the crew already said "You have the aircraft."
const PILOT_RELEASE_QUIET_S: f64 = 15.0;

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
fn interventions_now(
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
struct CrewBrakeLatch {
    prev: bool,
    crews: bool,
}

impl CrewBrakeLatch {
    /// The latch's raw reading and whether a crew brake is in flight; true
    /// while the latch is up and it is the crew's.
    fn step(&mut self, latched: bool, crew_brake: bool) -> bool {
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
/// `flight::note_pilot_takeover`.
fn note_interventions(i: &Inputs, control: crate::flight::Control) {
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
}

fn r1(v: f64) -> f64 {
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
struct WorldGates {
    quiet: bool,
    cabin_quiet: bool,
    compliance_ticks: bool,
    compliance_speaks: bool,
    autoland: bool,
}

fn world_gates(recovering: bool, autoland: bool, panel_mute: bool, crashed: bool) -> WorldGates {
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
fn radio_powered_from(readback: Option<&str>, read: Option<f64>) -> bool {
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
fn avionics_readback_dref(p: &AircraftProfile) -> Option<String> {
    p.electrical
        .iter()
        .find(|s| s.key == "avionics")
        .and_then(|s| s.readback.first().cloned())
}

fn compose_strip(s: &Snapshot) -> String {
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

pub async fn run(
    config: Config,
    live: std::sync::Arc<crate::LiveProfile>,
    voice: Voice,
    status: Option<StatusSink>,
    cabin: std::sync::Arc<std::sync::Mutex<crate::cabin::Cabin>>,
) {
    // Checks are built from the active profile and rebuilt when the aircraft
    // switches (the generation counter changes), so a jet doesn't keep a 172's
    // stall/overspeed limits after a profile swap.
    let advisor_drefs = |p: &AircraftProfile| -> Vec<String> {
        p.advisor.iter().map(|a| a.dataref.clone()).collect()
    };
    let mut gen = live.generation();
    let mut checks = build_checks(&live.get());
    let mut drefs = advisor_drefs(&live.get());
    // The tanks the centre-pump check reads, and their lateral positions
    // (static per airframe, read once per swap).
    let mut pump_tanks = live.get().fuel_tank_pumps_off_when_empty.clone();
    let mut tank_lateral: Option<Vec<f64>> = None;
    // The ice picture (#100), read on its own slow clock in cold air, and
    // since when the air has been clear of icing (the crew then switches off
    // what it switched on).
    let mut icing_read: Option<(Instant, crate::tools::anti_ice::IcingRead)> = None;
    let mut icing_clear_since: Option<Instant> = None;
    // The last icing verdict blackboxed (`icing` `conditions`), so a change
    // is one record with the numbers it was judged on.
    let mut icing_logged: Option<bool> = None;
    let mut avionics_readback = avionics_readback_dref(&live.get());
    // The cockpit duties the copilot performs unasked, per airframe: interior
    // lights flagged `auto_at_night`, and a GPS welcome page to acknowledge.
    // Collected at rebuild so an airframe declaring NEITHER never pays a read.
    let cockpit_duties =
        |p: &AircraftProfile| -> (Vec<crate::aircraft::LightProfile>, Option<String>) {
            (
                p.lights
                    .iter()
                    .filter(|l| l.auto_at_night)
                    .cloned()
                    .collect(),
                p.gps_ack_command.clone(),
            )
        };
    let (mut night_lights, mut gps_ack) = cockpit_duties(&live.get());
    // Whether THIS airframe carries a crew, rebuilt on aircraft switch; the
    // live staffing setting is read at the watch itself (cabin::aboard), so
    // a panel flip applies mid-flight without a profile swap.
    let mut cabin_capable = live.get().cabin_crew;
    tracing::info!(
        checks = checks.len(),
        hz = config.monitor_hz,
        "monitor started"
    );
    // Stamp the record with the airframe so the distiller can key memory by it.
    crate::blackbox::record(
        "flight_meta",
        serde_json::json!({ "aircraft": live.get().id }),
    );

    let xp = XPlaneClient::new(config.xplane_base_url.clone());
    let period = Duration::from_secs_f32(1.0 / config.monitor_hz.max(0.1));
    let mut states: HashMap<&'static str, FireState> =
        checks.iter().map(|c| (c.id(), FireState::new())).collect();

    // IAS history for the rate-of-decay term. `trend` is an EMA (kts/s) so a
    // single noisy sample doesn't read as a sharp washout.
    let mut prev_ias: Option<(f64, Instant)> = None;
    let mut trend = 0.0_f64;
    // The flight model (`flight::Flight`): the one state every consumer reads.
    let mut flight = crate::flight::Flight::new();
    // The duty engine's last answer, logged on change (`duty.rs`).
    let mut duty_trail = crate::duty::DutyTrail::default();
    // The State tab's last text, and its resend countdown.
    let mut last_state_text = String::new();
    let mut state_resend: u8 = 0;
    // Pushback progress is a sequence of *events*, not a sustained condition, so
    // it's called out on transitions here rather than as Checks.
    let mut prev_attached = false;
    let mut prev_pushing = false;
    let mut push_called = false;
    // Last strip pushed to the panel — only re-send on change.
    let mut last_strip = String::new();
    let mut last_card = String::new();
    // Last automation feed pushed, plus a tick countdown that forces a
    // periodic resend: a plugin reload zeroes its buffers, and a purely
    // change-gated publish would leave the Flight tab blank until the next
    // real change (the aircraft-chip heartbeat is the same lesson).
    let mut last_auto = String::new();
    let mut last_sheet = String::new();
    let mut auto_resend = 0u8;
    // When the Settings tab's resolved callsign was last published.
    let mut callsign_at: Option<Instant> = None;

    // Talk-down pacing: when the last callout went out, and whether we've
    // already handed over at the threshold (say it once, then shut up).
    let mut guide_last: Option<Instant> = None;
    let mut guide_handed_over = false;
    // Fps-dip watchdog: rolling window of recent fps so a dip is judged
    // against THIS flight's own baseline (60 locked vs 35 in weather), and a
    // rate limit so a sustained slump logs one event, not one per tick.
    let mut fps_window: std::collections::VecDeque<f64> = std::collections::VecDeque::new();
    let mut fps_dip_last: Option<Instant> = None;
    // Approach traffic advisories while being vectored, rate-limited: a real
    // controller calls a factor once and moves on, not every sweep.
    let mut traffic_last: Option<Instant> = None;
    // Near-miss cadence: repeats every 15 s while the conflict persists,
    // independent of the advisory clock above — a safety alert must not wait
    // out a 45 s narration window.
    let mut near_miss_last: Option<Instant> = None;
    // Gyro auto-sync: cooldown, the readback owed after a fire, and the latch
    // for an airframe whose sync does nothing (reset on every swap below).
    let mut gyro_sync = GyroSync::default();
    // The unasked cockpit duties (night lights, GPS welcome page) and their own
    // slower clock. The monitor ticks at 3 Hz and neither input justifies that:
    // the sun moves a quarter of a degree a minute, and a battery master thrown
    // two seconds ago is still "just now" to the pilot. At 3 Hz this pair would
    // have been 6 reads/s, the same order as the replay/pause flags the load
    // histogram singled out as the obvious thing to slow down.
    let mut cockpit_auto = crate::cockpit_auto::CockpitAuto::default();
    let mut cockpit_auto_last: Option<Instant> = None;
    /// How often the cockpit duties look at the sun and the bus.
    const COCKPIT_AUTO_EVERY: Duration = Duration::from_secs(2);
    // Party-line cadence, and whether an advisory is waiting on its "clear of
    // traffic" closure.
    let mut chatter_last: Option<Instant> = None;
    // The taxi clearance whose runway the crew last judged for a pilot
    // taxiing (full auto judges its own): said once per clearance.
    let mut cleared_runway_judged: Option<String> = None;
    // The last ATIS edition logged to the panel ("London Heathrow information
    // Alpha") — the loop speaks every repeat, the transcript records once.
    let mut atis_logged: Option<String> = None;
    let mut traffic_open = false;
    let mut traffic_called: Option<crate::station::TrafficFactor> = None;
    // Runway-incursion shout cadence: it repeats while the conflict persists
    // (a safety shout may nag), but not every tick.
    let mut incursion_last: Option<Instant> = None;
    // Landing-clearance coach: when the approach hands us to the tower we owe
    // a "final" report before "cleared to land". `seen` starts the grace so a
    // prompt reporter is never nagged; `spoken` fires the nudge once.
    let mut landing_coach_seen: Option<Instant> = None;
    let mut landing_coach_spoken = false;
    // Region-handoff coach: with the pilot flying, the enroute controller's
    // "squawk seven thousand, contact X Tower on Y" is theirs to obey; after
    // a grace the copilot says once whatever is still undone.
    let mut region_coach: Option<(crate::party_line::RegionHandoff, Instant)> = None;
    // The copilot's progressive taxi guide, rebuilt per published clearance
    // route: the pilot monitoring with the airport diagram out.
    let mut taxi_guide: Option<crate::taxi_guide::Guide> = None;
    // Whether the last tick saw the sim in replay / paused, for transition logging.
    let mut was_replay = false;
    let mut was_paused = false;
    let mut lost_since: Option<Instant> = None;
    let mut seen_sim = false;
    let mut disconnected = false;
    let mut had_flying_control = false;
    let mut control_idle_since: Option<Instant> = None;
    let mut airspeed_recovery = crate::interaction::Recovery::default();
    let mut cabin_recovery = crate::interaction::Recovery::default();
    let mut landing_summary = crate::interaction::Landing::default();
    // The copilot as a person who was there: the flight's memory, the word
    // after the landing, the offers, the asides held for a quiet moment.
    let mut rapport = crate::rapport::Rapport::new();
    let mut rapport_airframe = crate::rapport::Airframe::from_profile(&live.get());
    // The checklist flow's phase-boundary detector: climb, cruise, descent,
    // before landing, after landing, each once per flight.
    let mut flow_tracker = crate::flow::Tracker::default();

    loop {
        // Settings that this tick reads (the taxi guide, the party line) are
        // taken fresh from the live store, so a Settings-screen change applies
        // on the next tick rather than the next boot. Boot-time values used
        // above (the sample rate, the base URL) deliberately stay as they were.
        let config = crate::settings::live();
        // Aircraft switched → rebuild the checks for the new airframe.
        if live.generation() != gen {
            gen = live.generation();
            checks = build_checks(&live.get());
            drefs = advisor_drefs(&live.get());
            pump_tanks = live.get().fuel_tank_pumps_off_when_empty.clone();
            tank_lateral = None;
            icing_read = None;
            icing_clear_since = None;
            icing_logged = None;
            avionics_readback = avionics_readback_dref(&live.get());
            (night_lights, gps_ack) = cockpit_duties(&live.get());
            // A new airframe is a new cockpit: whatever we set on the last one
            // says nothing about this one's switches, and its GPS has its own
            // welcome page. Same rule as the radio-power latch — a process-wide
            // fact that outlives its airframe aims the next aircraft's writes
            // at something it may not have.
            cockpit_auto = crate::cockpit_auto::CockpitAuto::default();
            // "The sync does nothing here" is a fact about THAT airframe.
            gyro_sync = GyroSync::default();
            flow_tracker = crate::flow::Tracker::default();
            landing_summary = crate::interaction::Landing::default();
            rapport = crate::rapport::Rapport::new();
            rapport_airframe = crate::rapport::Airframe::from_profile(&live.get());
            airspeed_recovery = crate::interaction::Recovery::default();
            cabin_recovery = crate::interaction::Recovery::default();
            had_flying_control = false;
            cabin_capable = live.get().cabin_crew;
            states = checks.iter().map(|c| (c.id(), FireState::new())).collect();
            tracing::info!(
                checks = checks.len(),
                "monitor: reconfigured for new aircraft"
            );
            crate::blackbox::record(
                "flight_meta",
                serde_json::json!({ "aircraft": live.get().id }),
            );
        }
        // Replay or pause: neither is the aircraft flying NOW — a replay is a
        // recording, a pause is real but frozen — so nothing here may react.
        // One gate covers the whole reactive tick: checks and callouts, the
        // ATC compliance / incursion / region watches, and crash recording.
        // A replayed crash landing earned "you landed on runway two seven
        // without clearance" and a second crash event (2026-07-14). Fail
        // open: a failed read means live. State (FireStates) simply freezes
        // and resumes on live.
        let in_replay = xp
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5);
        let paused = xp
            .get_float(crate::datarefs::PAUSED)
            .await
            .is_ok_and(|v| v > 0.5);
        if in_replay != was_replay {
            was_replay = in_replay;
            tracing::info!(in_replay, "monitor: replay mode changed");
            crate::blackbox::record("replay", serde_json::json!({ "active": in_replay }));
        }
        if paused != was_paused {
            was_paused = paused;
            tracing::info!(paused, "monitor: pause changed");
            crate::blackbox::record("pause", serde_json::json!({ "active": paused }));
        }
        // Published to the arbiter and the command loop's transmit gates.
        // Replay drops station audio (cutting a live transmission — the film
        // has no radio); pause only blocks NEW radio traffic, so a controller
        // caught mid-word finishes the sentence.
        voice.set_replay(in_replay);
        voice.set_paused(paused);
        if in_replay || paused {
            tokio::time::sleep(period).await;
            continue;
        }
        if let Some(mut snap) = Snapshot::read(&xp, &drefs).await {
            if !pump_tanks.is_empty() {
                snap.fuel_tanks = read_fuel_tanks(&xp, &pump_tanks, &mut tank_lateral).await;
            }
            let now = Instant::now();
            // The altimeter cross-check, published for a follower that wants
            // to know whether the altimeter it flies can be believed (#94).
            let truth = altimeter_truth_of(&snap, live.get().transition_altitude_ft);
            crate::tools::instruments::publish_altimeter_truth(truth);
            // The ice picture (#100): cold air only, every ICING_READ_S.
            if snap.on_ground || snap.oat_c > ICING_READ_OAT_C {
                icing_read = None;
            } else if icing_read
                .as_ref()
                .is_none_or(|(at, _)| now.duration_since(*at) >= Duration::from_secs(ICING_READ_S))
            {
                let systems = crate::tools::anti_ice::resolved_live();
                let r = crate::tools::anti_ice::read_icing(&xp, snap.oat_c, snap.dewpoint_c, &systems)
                    .await;
                icing_read = Some((now, r));
            }
            snap.icing = icing_read.as_ref().map(|(_, r)| r.clone());
            // Clear of icing long enough: the crew switches off only what it
            // switched on itself, and only while an automation flies.
            let icing_now = snap.icing.as_ref().is_some_and(|r| {
                crate::tools::anti_ice::icing_conditions(r) || crate::tools::anti_ice::accreting(r)
            });
            if snap.icing.is_some() && icing_logged != Some(icing_now) {
                icing_logged = Some(icing_now);
                crate::blackbox::record(
                    "icing",
                    serde_json::json!({"event": "conditions", "icing": icing_now,
                        "oat_c": snap.oat_c, "dewpoint_c": snap.dewpoint_c, "alt_ft": snap.alt_ft,
                        "claimed": crate::tools::anti_ice::claimed().len()}),
                );
            }
            if icing_now {
                icing_clear_since = None;
            } else {
                let since = *icing_clear_since.get_or_insert(now);
                let claimed = crate::tools::anti_ice::claimed();
                if !claimed.is_empty()
                    && !snap.on_ground
                    && snap.automation_flying
                    && crate::mode::crew_actuates()
                    && now.duration_since(since) >= Duration::from_secs(ICING_CLEAR_S)
                {
                    let failed = crate::tools::anti_ice::monitor_fix(&xp, &claimed, false).await;
                    crate::blackbox::record(
                        "icing",
                        serde_json::json!({"event": "clear_off", "systems": claimed.len(),
                            "took": failed.is_none(), "oat_c": snap.oat_c, "dewpoint_c": snap.dewpoint_c}),
                    );
                    voice.alert(
                        Priority::Advisory,
                        failed.unwrap_or_else(|| "Clear of icing, anti-ice off.".to_string()),
                    );
                    icing_clear_since = Some(now);
                }
            }
            // The flight model FIRST, every tick whatever is quiet: the
            // consumers below read its phase, memory and this tick's events
            // (`flight.rs`). Its events are blackboxed `flight`, its phase
            // rides the state record as `fphase`.
            let wall_t = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_secs_f64())
                .unwrap_or(0.0);
            // The world's reach: a station whose field we are not at (on the
            // ground) neither hears nor answers, and an uncontrolled
            // departure plans its own taxi.
            crate::station::note_aircraft(snap.lat, snap.lon, snap.on_ground);
            let crashed = voice.crashed();
            let flight_events = flight.tick(&flight_sample(&snap, wall_t, crashed));
            snap.phase = coarse_phase(flight.phase(), snap.agl_ft);
            let flight_status = flight.status();
            // The duty engine, in SHADOW: what the flight owes next, logged
            // when it changes beside whether the running automation agrees.
            // It acts on nothing (`duty.rs`).
            if let Some(st) = &flight_status {
                let landed = flight.memory().landings > 0;
                let (d, why) = crate::duty::next(st, landed, wall_t);
                // The State tab, change-gated with a resend every 20 ticks
                // (a plugin reload zeroes its buffer).
                if let Some(sink) = status.as_ref() {
                    let text = crate::flight::state_lines(
                        st,
                        flight.memory(),
                        flight.in_phase_secs(wall_t),
                        (d.name(), why, crate::duty::agrees(d, st.control)),
                    );
                    state_resend = state_resend.saturating_sub(1);
                    if text != last_state_text || state_resend == 0 {
                        sink.state(&text);
                        last_state_text = text;
                        state_resend = 20;
                    }
                }
                // Written once the verdict has settled (`duty::SETTLE_S`): a
                // hand-over between loops is not a disagreement.
                let phase = format!("{:?}", st.phase);
                if let Some(r) = duty_trail.step(d, why, &phase, st.control, wall_t) {
                    crate::blackbox::record(
                        "duty",
                        serde_json::json!({
                            "duty": r.duty.name(), "why": r.why,
                            "phase": r.phase, "began_t": r.began_t,
                            "agrees": r.agrees,
                        }),
                    );
                }
            }
            crate::flight::publish(flight_status);
            for ev in &flight_events {
                crate::blackbox::record("flight", ev.to_json());
            }
            seen_sim = true;
            lost_since = None;
            if disconnected {
                disconnected = false;
                landing_summary = crate::interaction::Landing::default();
                landing_summary.note_interruption();
                airspeed_recovery = crate::interaction::Recovery::default();
                cabin_recovery = crate::interaction::Recovery::default();
                if crate::mode::copilot_enabled() {
                    voice.alert(Priority::Caution, "Connected again. I'm monitoring only; you have control. Check the autopilot. Say resume the flight plan when ready.".to_string());
                }
            }

            // Fps-dip watchdog: pure diagnosis, no callout. A dip well below
            // the rolling median lands in the flight log + blackbox with a
            // timestamp, so it correlates against what else was happening
            // (llm answer, tts synth, station transmission — all logged).
            if let Some(fps) = snap.fps {
                let median = {
                    let mut v: Vec<f64> = fps_window.iter().copied().collect();
                    v.sort_by(|a, b| a.total_cmp(b));
                    v.get(v.len() / 2).copied()
                };
                if let Some(median) = median.filter(|m| fps_window.len() >= 30 && *m >= 20.0) {
                    let rate_ok =
                        fps_dip_last.is_none_or(|t| now.duration_since(t).as_secs() >= 10);
                    if fps < median * 0.7 && rate_ok {
                        fps_dip_last = Some(now);
                        tracing::warn!(fps = fps.round(), median = median.round(), "fps dip");
                        crate::blackbox::record(
                            "fps_dip",
                            serde_json::json!({ "fps": r1(fps), "median": r1(median) }),
                        );
                    }
                }
                fps_window.push_back(fps);
                if fps_window.len() > 90 {
                    fps_window.pop_front();
                }
            }

            // Crew-side watches stand down entirely in ATC-only mode: no
            // copilot and no callouts — the pilot kept the world and sent
            // the crew home. World ticks are inert without the station's
            // published state, so most need no gate of their own;
            // region_tick is the exception (its index cache survives).
            let crew_on = crate::mode::copilot_enabled();

            // The sim moved the aircraft (a new flight loaded, a reposition):
            // a list of the flight that was ends with it. Through the door,
            // which knows a jump our own setup or the runner's start made
            // (their doors already ran) and keeps a replay's jump light.
            if flight_events
                .iter()
                .any(|e| matches!(e, crate::flight::Event::Reposition { .. }))
            {
                crate::flight_state::flight_begins(crate::flight_state::NewFlight::Reposition);
            }
            // The checklist at its phase: crossing a boundary runs the
            // matching list as the flying copilot's cross-check when an
            // automation has the aircraft, or proposes it to the pilot
            // flying ("Climb checklist when you're ready?"). The tracker is
            // sustained-condition and one-shot per flight; the flow module
            // decides who is flying and never acts on a proposal by itself.
            if crew_on {
                let sample = crate::flow::Sample {
                    agl_ft: snap.agl_ft,
                    vsi_fpm: snap.vsi_fpm,
                    automation: crate::flow::automation_flying(),
                    landing_config: crate::flow::landing_config_for(
                        // The helicopter's loops land without autoland, so
                        // its landing items are not theirs to wait for.
                        crate::flight_state::crew_flying().is_some_and(|o| {
                            !matches!(
                                o,
                                crate::flight_state::Owner::HeliRoute
                                    | crate::flight_state::Owner::HeliCircuit
                                    | crate::flight_state::Owner::Hover
                            )
                        }),
                        snap.gear_down,
                        snap.flap_ratio,
                        !crate::aircraft::flap_limits().is_empty(),
                        crate::autoland::landing_configured(),
                    ),
                    climb_config: crate::flow::climb_config(
                        snap.flap_ratio,
                        !crate::aircraft::flap_limits().is_empty(),
                    ),
                    on_runway: if snap.on_ground {
                        crate::party_line::on_runway(snap.lat, snap.lon)
                    } else {
                        None
                    },
                };
                if let Some(section) = flow_tracker.tick(&sample, &flight, &flight_events, now) {
                    crate::flow::phase_boundary(section, &config, &voice);
                }
            }

            // The instructor's lesson: every tick feeds the scorer (and the
            // brief's picture of the aircraft); narration comes back only
            // while a lesson flies. Advisory, so the radio is never talked
            // over for a heading nudge.
            // The aircraft against the cleared taxi route (the guide's turn
            // card), for the lesson's centreline and the state record (#88).
            let taxi_line = (snap.on_ground
                && crate::party_line::clearance_state() == Some(crate::tower::State::TaxiCleared))
            .then(|| taxi_guide.as_ref().and_then(|g| g.turn_card(snap.lat, snap.lon)))
            .flatten()
            .map(|c| crate::instructor::TaxiLine::from_card(&c));
            if crew_on {
                let sample = crate::instructor::Sample {
                    t_s: crate::instructor::clock_s(),
                    alt_ft: snap.alt_ft,
                    agl_ft: snap.agl_ft,
                    hdg_deg: snap.heading_deg,
                    ias_kts: snap.ias_kts,
                    bank_deg: snap.bank_deg,
                    vsi_fpm: snap.vsi_fpm,
                    on_ground: snap.on_ground,
                    track_deg: snap.track_deg,
                    gs_kts: snap.groundspeed_kts,
                    engine_running: snap.engine_running,
                    taxi: taxi_line,
                };
                let mut io = crate::instructor::FlowIo {
                    config: &config,
                    voice: &voice,
                };
                for line in crate::instructor::tick_with(sample, &mut io) {
                    voice.alert(Priority::Advisory, line);
                }
            }

            // Pushback callouts (on the ground): tug hooks up → pushing → clear.
            if crew_on && snap.on_ground {
                // Mirror OR action, never both: `voice.alert` already mirrors the
                // spoken line to the panel transcript (2026-07-24), so an extra
                // `s.alert` here wrote every callout twice — a green mirror line
                // AND a blue Advisory line, same timestamp (2026-07-26).
                let say = |line: &str| {
                    voice.alert(Priority::Advisory, line.to_string());
                };
                // Casual ground chatter, not phraseology — so it varies flight to
                // flight (the value is nothing to standardise here, unlike a callout).
                if snap.tug_attached && !prev_attached && !snap.pushing_back {
                    say(crate::readback::one_of(&[
                        "Tug connected, ready for pushback.",
                        "Tug's hooked up, ready to push.",
                        "Ground's connected, ready when you are.",
                    ]));
                }
                // Once per hook-up: `pushing_back` is motion over 0.3 kt with
                // the bar on, and a tug nudging at the edge of it flickered
                // "Pushing back" three times in five seconds (LIPK 2026-09-26).
                if snap.pushing_back && !prev_pushing && !push_called {
                    push_called = true;
                    say(crate::readback::one_of(&[
                        "Pushing back.",
                        "Here we go, pushing back.",
                        "And we're pushing.",
                    ]));
                }
                // Clear is the BAR coming off, not the rolling stopping: now
                // that `pushing_back` is motion-based, a pause mid-push would
                // otherwise announce a tug that is still hooked to the nose.
                if !snap.tug_attached {
                    push_called = false;
                }
                if !snap.tug_attached && prev_attached {
                    say(crate::readback::one_of(&[
                        "Pushback complete, tug clear.",
                        "Pushback done, tug's clear.",
                        "That's us off the gate, tug clear.",
                    ]));
                }
            }
            prev_attached = snap.tug_attached;
            prev_pushing = snap.pushing_back;

            // The wreck flag: X-Plane raises has_crashed the instant the
            // airframe is destroyed; a flight reset drops it, and the world
            // resumes. Published to the arbiter (receive + transmit gates) —
            // the avionics bus stays hot on the crash screen, so the power
            // gate alone let a mayday from the wreck raise the tower
            // (2026-07-13). Fail open: a failed read changes nothing.
            if let Ok(v) = xp.get_float(crate::datarefs::HAS_CRASHED).await {
                let is_crashed = v > 0.5;
                if is_crashed && !voice.crashed() {
                    crate::blackbox::record(
                        "crash",
                        serde_json::json!({
                            "lat": snap.lat, "lon": snap.lon, "alt": r1(snap.alt_ft),
                        }),
                    );
                }
                voice.set_crashed(is_crashed);
            }
            // The cockpit duties a crew member performs unasked: the
            // interior lights once the aircraft is alive and the sun is
            // down, and the GNS430's welcome page. Crew-gated for the same
            // reason as the gyro sync above — this is the copilot's HAND on
            // the panel, and in ATC-only nobody is aboard to reach it.
            // Skipped entirely on an airframe that declares neither duty,
            // so nothing pays for a capability it doesn't have.
            if crew_on
                && crate::mode::crew_actuates()
                && (!night_lights.is_empty() || gps_ack.is_some())
                && cockpit_auto_last.is_none_or(|t| now.duration_since(t) >= COCKPIT_AUTO_EVERY)
            {
                cockpit_auto_last = Some(now);
                // The pilot switched one of these themselves since our last
                // pass: their switch now, not ours.
                if crate::cockpit_auto::take_pilot_moved_lights() {
                    cockpit_auto.pilot_took_the_lights();
                }
                // Both best-effort: a failed read must not act. Defaulting
                // either way would be a guess with a switch on the end of
                // it — a missing sun angle reading as night would light the
                // cockpit at noon.
                let sun = xp.get_float(crate::datarefs::SUN_PITCH_DEG).await;
                let volts = xp
                    .get_float_array(crate::datarefs::BUS_VOLTS)
                    .await
                    .ok()
                    .and_then(|v| v.first().copied());
                if let (Ok(sun), Some(volts)) = (sun, volts) {
                    for action in cockpit_auto.tick(sun, volts) {
                        use crate::cockpit_auto::Action;
                        match action {
                            Action::LightsOn | Action::LightsOff => {
                                let on = action == Action::LightsOn;
                                // Through the SAME writer the `set_lights`
                                // tool uses, so every profile mechanism and
                                // the readback behave identically whether a
                                // pilot asked or the copilot decided.
                                let mut done = Vec::new();
                                for l in &night_lights {
                                    if crate::tools::lights::apply(&xp, l, on)
                                        .await
                                        .is_ok_and(|actual| actual == on)
                                    {
                                        done.push(l.spoken.clone());
                                    }
                                }
                                crate::blackbox::record(
                                    "cockpit_auto",
                                    serde_json::json!({
                                        "event": if on { "lights_on" } else { "lights_off" },
                                        "sun_pitch_deg": r1(sun), "bus_volts": r1(volts),
                                        "lights": done,
                                    }),
                                );
                                // Speak only what actually moved: a switch
                                // that refused is not an announcement, it
                                // is the ground-power lie in a new coat.
                                if let Some(what) = done.first() {
                                    voice.alert(
                                        Priority::Advisory,
                                        if on {
                                            format!("Sun's down, {what} on.")
                                        } else {
                                            format!("Daylight, {what} off.")
                                        },
                                    );
                                }
                            }
                            Action::AckGps => {
                                // Twice: the welcome page and the database
                                // page behind it. Silent — clearing a
                                // startup screen is not worth a word, and
                                // the pilot sees the unit come up.
                                if let Some(cmd) = &gps_ack {
                                    let a = xp.run_command(cmd).await;
                                    let b = xp.run_command(cmd).await;
                                    crate::blackbox::record(
                                        "cockpit_auto",
                                        serde_json::json!({
                                            "event": "gps_ack", "command": cmd,
                                            "ok": a.is_ok() && b.is_ok(),
                                        }),
                                    );
                                }
                            }
                        }
                    }
                }
            }
            // Approach vectors: armed by the ATC station when the pilot requests
            // vectors on the approach frequency. Advisory narration only; muted
            // under the same gates as the visual-approach talk-down. A crashed
            // airframe silences the whole world block: no controller works a
            // wreck, and the frozen flight model reads as garbage anyway.
            let gates = world_gates(
                crate::flight_state::Owner::Recover.active(),
                crate::flight_state::Owner::Autoland.active(),
                PANEL_MUTE.load(std::sync::atomic::Ordering::Relaxed),
                voice.crashed(),
            );
            let quiet = gates.quiet;
            // The CABIN is not part of that. `quiet` mutes the world's
            // narration while the flight deck is busy, and an autoland is
            // exactly when it is - but the cabin is a state machine that has
            // to SEE the flight, not a commentator. Gated with the rest it was
            // never called from the autoland arm to the rollout handoff, so it
            // missed its own landing every time an automation flew the
            // approach (Liege 2026-09-22, "cabin is secure for departure"
            // during the taxi in). A recovery, the panel's mute and a wreck
            // still silence it; a normal landing does not.
            let cabin_quiet = gates.cabin_quiet;
            feed_surface_wind();
            // The crew's oxygen masks: on in the cabin-altitude warning band,
            // off again below it, never over the pilot (crew hands only;
            // #159: they used to follow the master and breathed all flight).
            if crate::mode::crew_actuates() {
                if let Some((urgent, line)) =
                    crate::tools::cabin_air::masks_tick(&xp, snap.on_ground).await
                {
                    let p = if urgent { Priority::Warning } else { Priority::Advisory };
                    voice.alert(p, line);
                }
            }
            // Cabin crew watches (crew-side, airframes with a cabin crew
            // only): the owed "cabin secure" report after an acknowledged
            // prepare call, and the seatbelt-sign interphone call. The
            // sterile-cockpit rule lives in the cabin module — safety
            // reports come through, flavor stays above 10,000 ft.
            if crew_on && !cabin_quiet && crate::cabin::aboard(cabin_capable) {
                let (own, belt_line, tick_line) = {
                    let mut cab = cabin.lock().unwrap_or_else(|e| e.into_inner());
                    (
                        // The cabin's own initiative first: what it
                        // starts here, `tick` reports when due.
                        cab.watch(
                            now,
                            crate::cabin::View {
                                agl_ft: snap.agl_ft,
                                on_ground: snap.on_ground,
                                groundspeed_kts: snap.groundspeed_kts,
                                vsi_fpm: snap.vsi_fpm,
                                engine_running: snap.engine_running,
                                pushing_back: snap.pushing_back || snap.tug_attached,
                                flight: Some(crate::cabin::FlightFacts {
                                    landed: flight.memory().landings > 0
                                        && !flight.phase().airborne(),
                                    repositioned: flight_events.iter().any(|e| {
                                        matches!(e, crate::flight::Event::Reposition { .. })
                                    }),
                                }),
                            },
                        ),
                        snap.seatbelt_sign
                            .and_then(|on| cab.seatbelt(on, snap.agl_ft)),
                        cab.tick(now, snap.agl_ft, snap.on_ground),
                    )
                };
                let own_line = own.and_then(|i| {
                    if i.line.is_none() {
                        crate::blackbox::record(
                            "cabin",
                            serde_json::json!({ "route": "watch", "event": i.event }),
                        );
                    }
                    i.line
                });
                for line in [own_line, belt_line, tick_line].into_iter().flatten() {
                    if let Some(s) = status.as_ref() {
                        s.action(&format!("CAB: {line}"));
                    }
                    crate::blackbox::record(
                        "cabin",
                        serde_json::json!({ "route": "watch", "line": &line }),
                    );
                    voice.alert_cabin(Priority::Advisory, line);
                }
            }
            // One COM1 read serves the compliance watch, the incursion
            // shout, the ATIS loop and the traffic advisories below, read
            // whenever the compliance watch ticks (a wreck reads nothing).
            let com1: Option<i64> = if gates.compliance_ticks {
                xp.get_float(crate::datarefs::COM1_FREQ_KHZ)
                    .await
                    .ok()
                    .map(|v| v.round() as i64)
            } else {
                None
            };
            // Which CONTROLLER the radio actually hears, resolved once per
            // tick: every controller call below speaks in THIS voice (or
            // falls to the crew when None). A berate hardcoded to the
            // Tower while the pilot sat on the approach frequency put two
            // controllers on one radio (2026-07-12).
            let tuned_speaker = com1.and_then(crate::station::tuned_controller);
            // Compliance watch: the tower holds the pilot to the clearances
            // it actually issued (uncleared runway entry, takeoff, landing),
            // and challenges zone traffic it has never talked to. Tuned to
            // the station the controller speaks; otherwise the copilot flags
            // it as crew: a controller can't berate a radio that isn't
            // listening.
            //
            // NOT under `quiet`: that gate includes `autoland::ACTIVE`, and
            // the landing clearance is exactly what this watch guards. Gated
            // with the chatter it was never ticked from the autoland arm to
            // the rollout handoff, so its air/ground edge went stale and an
            // uncleared landing was called on the first tick after the
            // handoff, 26.6 s after the touchdown, at a stopped aircraft
            // (2026-09-26, issue #63). It TICKS on every live tick but a
            // wreck, so its edges are always current, and SPEAKS unless a
            // recovery, the panel's mute or a wreck silences the world (the
            // cabin's gate). A muted call is consumed, never said late.
            if gates.compliance_ticks {
                if let Some(call) = crate::station::compliance_tick_facing(
                    snap.lat,
                    snap.lon,
                    snap.alt_ft,
                    crate::hover::tower_on_ground(snap.on_ground, snap.agl_ft),
                    snap.groundspeed_kts,
                    tuned_speaker.is_some(),
                    now,
                    Some(snap.heading_deg).filter(|h| *h != 0.0),
                ) {
                    crate::blackbox::record(
                        "atc",
                        serde_json::json!({
                            "route": "compliance", "crew": call.crew, "reply": &call.line,
                            "autoland": gates.autoland, "said": gates.compliance_speaks,
                        }),
                    );
                    if !gates.compliance_speaks {
                        // A recovery, the panel's mute: consumed, unsaid.
                    } else if call.crew {
                        voice.alert(Priority::Caution, call.line);
                    } else {
                        // The controller you're actually TUNED to delivers
                        // it (approach relays a deviation and the phone
                        // number exactly like the real thing), never the
                        // tower's voice on someone else's frequency.
                        let who = tuned_speaker.unwrap_or(crate::voice::StationSpeaker::Tower);
                        if let Some(s) = status.as_ref() {
                            s.action(&format!("{}: {}", who.tag(), call.line));
                        }
                        voice.alert_station(Priority::Warning, who, call.line);
                    }
                }
            }
            // The landing's runway watch: the go-around order (a ghost on the
            // runway that will still be there when we arrive) and the release
            // of a withheld landing clearance. NOT under `quiet`, for the
            // compliance watch's reason: `quiet` includes `autoland::ACTIVE`,
            // and an autoland final is exactly the approach this watch guards.
            // Gated with the chatter, "continue approach, expect late landing
            // clearance" at 7 nm was never released and never judged into a
            // go-around, and autoland landed uncleared (EGKK-EGBB 15, sortie
            // 1790601822-full_auto, 2026-09-28, issue #155). Ticks on every
            // live tick but a wreck; speaks unless a recovery, the panel's
            // mute or a wreck silences the world (a muted call is consumed).
            // The TCAS read is shared with the traffic watches below.
            let landing_held = crate::party_line::landing_hold_armed();
            let cleared_to_land = crate::party_line::clearance_state()
                == Some(crate::tower::State::ClearedToLand);
            let mut runway_watch_targets: Option<Vec<crate::station::TcasTarget>> = None;
            if gates.compliance_ticks
                && (landing_held || cleared_to_land)
                && crate::station::active_field().is_some()
            {
                let targets = crate::station::read_tcas_targets(xp.view()).await;
                let tower_tuned =
                    matches!(tuned_speaker, Some(crate::voice::StationSpeaker::Tower));
                // Safety outranks flavor: the go-around order first.
                if let Some(call) = crate::station::go_around_tick(
                    &targets,
                    snap.agl_ft,
                    snap.on_ground,
                    tuned_speaker.is_some(),
                    now,
                ) {
                    crate::blackbox::record(
                        "atc",
                        serde_json::json!({
                            "route": "go_around_order", "crew": call.crew,
                            "reply": &call.line, "autoland": gates.autoland,
                            "said": gates.compliance_speaks,
                        }),
                    );
                    if !gates.compliance_speaks {
                        // A recovery, the panel's mute: consumed, unsaid.
                    } else if call.crew {
                        voice.alert(Priority::Warning, call.line);
                    } else {
                        // Whoever you're tuned to gives the order: a
                        // go-around shout is any controller's to deliver,
                        // in their own voice.
                        let who = tuned_speaker.unwrap_or(crate::voice::StationSpeaker::Tower);
                        if let Some(s) = status.as_ref() {
                            s.action(&format!("{}: {}", who.tag(), call.line));
                        }
                        voice.alert_station(Priority::Warning, who, call.line);
                    }
                } else if landing_held {
                    if let Some(line) =
                        crate::station::landing_hold_tick(&targets, snap.on_ground, now)
                    {
                        crate::blackbox::record(
                            "atc",
                            serde_json::json!({
                                "route": "landing_release", "reply": &line,
                                "autoland": gates.autoland, "said": gates.compliance_speaks,
                            }),
                        );
                        // The tower's clearance on the tower frequency only,
                        // like the takeoff release.
                        if tower_tuned && gates.compliance_speaks {
                            if let Some(s) = status.as_ref() {
                                s.action(&format!("TWR: {line}"));
                            }
                            voice.alert_station(
                                Priority::Advisory,
                                crate::voice::StationSpeaker::Tower,
                                line,
                            );
                        }
                    }
                }
                runway_watch_targets = Some(targets);
            }
            if !quiet {
                // Tell the speech arbiter what COM1 hears (controllers AND
                // the ATIS loop): it cuts a live station transmission the
                // instant we tune off its frequency, and lets a stepped-on
                // tail die with a retune instead of following us to the new
                // frequency.
                voice.set_com1(com1, com1.and_then(crate::station::tuned_station));
                // ...and how FAR that station is: the radio's signal-quality
                // model fades a distant tower into hiss instead of playing
                // every transmission studio-clean at 60 nm.
                voice
                    .set_station_range(crate::station::active_station_range_nm(snap.lat, snap.lon));
                // Radio audio needs a powered receiver: with the avionics
                // master OFF the pilot can't hear ATC/ATIS/traffic. Confident
                // reads only — a read failure or an airframe with no avionics
                // readback keeps the radio powered (fail open, never a
                // mysteriously silent controller).
                // Fail open must be RESTORED, not merely defaulted: the flag is
                // process-wide and survives an airframe switch, so "stop
                // publishing" latches whatever the LAST aircraft said. A
                // cold-and-dark 172 published false, the pilot swapped to a Twin
                // Otter (no avionics master — nothing left to publish), and the
                // stale false silently binned every crew radio call for the rest
                // of the session: full auto's clearance and taxi requests never
                // left the aircraft and it blamed the tower ("no taxi clearance
                // came back", Heathrow 2026-07-27). A failed read is the same
                // trap one tick wide.
                let read = match &avionics_readback {
                    Some(dref) => xp.get_float(dref).await.ok(),
                    None => None,
                };
                voice.set_radio_powered(radio_powered_from(avionics_readback.as_deref(), read));
                // Runway incursion: sitting (or slow-taxiing) ON a runway with
                // live traffic in THAT strip's final corridor — the one
                // conflict geometry where seconds matter. Repeats every 15 s
                // while it persists; escalates inside 1.5 nm. The tower shouts
                // when the pilot is on its frequency, the copilot calls it as
                // crew otherwise — a safety net must not depend on the radio
                // being tuned.
                // Rolling above ~30 kt is a departing aircraft, not an
                // incursion. LiveTraffic / AI aircraft feed the TCAS picture.
                // ...unless the tower ITSELF put us there: a standing takeoff
                // clearance or "line up and wait" means the runway is ours,
                // and "exit the runway, expedite" 42 seconds after "cleared
                // for takeoff" is the tower contradicting its own clearance
                // (Malaga runway 12, 2026-07-12). A genuine conflict against
                // a cleared departure is the release/cancel machinery's
                // problem, never this shout.
                // So too the LANDING it cleared, until the vacated report, and
                // a runway start or a moving cleared crossing: EGKK 26L
                // (2026-09-27) heard "exit the runway immediately" from the
                // touchdown to the vacate, eight times, cleared to land.
                let runway_is_ours = crate::party_line::runway_occupancy_authorised(
                    snap.lat,
                    snap.lon,
                    snap.groundspeed_kts,
                );
                if snap.on_ground && snap.groundspeed_kts < 30.0 && !runway_is_ours {
                    // Named by the end we point down, as the compliance
                    // watch names it: rolling out on 15 past midfield is
                    // not "runway three three" (EGKK-EGBB, #155).
                    if let Some(rwy) = crate::station::on_active_runway_facing(
                        snap.lat,
                        snap.lon,
                        Some(snap.heading_deg).filter(|h| *h != 0.0),
                    ) {
                        let due =
                            incursion_last.is_none_or(|t| now.duration_since(t).as_secs() >= 15);
                        if due {
                            if let Some(nm) =
                                crate::station::inbound_traffic_for_strip(xp.view(), &rwy).await
                            {
                                incursion_last = Some(now);
                                let urgent = nm < 1.5;
                                crate::blackbox::record(
                                    "runway_incursion",
                                    serde_json::json!({ "runway": &rwy, "traffic_nm": r1(nm) }),
                                );
                                if let Some(who) = tuned_speaker {
                                    // Addressed by callsign, like every other
                                    // controller call — an unaddressed "exit
                                    // the runway" reads as talking to nobody
                                    // (pilot feedback, 2026-07-10).
                                    let callsign = crate::circuit::crew_callsign(&xp)
                                        .await
                                        .unwrap_or_else(|| "aircraft on the runway".to_string());
                                    let line = if urgent {
                                        format!(
                                            "{callsign}, traffic on short final, exit the runway immediately."
                                        )
                                    } else {
                                        format!(
                                            "{callsign}, traffic on {nm:.0} mile final, exit the runway, expedite."
                                        )
                                    };
                                    if let Some(s) = status.as_ref() {
                                        s.action(&format!("{}: {line}", who.tag()));
                                    }
                                    voice.alert_station(Priority::Warning, who, line);
                                } else {
                                    let line = if urgent {
                                        "Traffic short final and we're on the runway. Clear the runway NOW.".to_string()
                                    } else {
                                        format!(
                                            "Traffic on {nm:.0} mile final and we're still on the runway. Expedite."
                                        )
                                    };
                                    voice.alert(Priority::Warning, line);
                                }
                            }
                        }
                    }
                }
                // Gyro auto-sync: a drifted DG gets re-aligned by the copilot's
                // own hand, then announced — not a "recommend syncing" nag
                // (pilot feedback, 2026-07-09). Airborne only (on the ground
                // you align at the run-up). Gated on MEASURED drift, not the
                // profile's directional_gyro flag: the sim drifts a DG under
                // "glass" airframes too (the SR22 flew 166° off while the
                // profile swore there was nothing to sync, 2026-07-12) — and
                // a true glass panel reads ~0 here, so it never fires there.
                // Crew-gated: this is the copilot's HAND on the panel, not
                // just its voice — in ATC-only nobody re-aligns your DG.
                //
                // An ACCEPTED command is not a sync: the drift is read back a
                // beat later and the line is said only if it fell. On the stock
                // 737 the command did nothing and the copilot claimed a resync
                // every 120 s with the drift climbing 11.8 to 15.6 (LEBL-LFBD
                // 2026-09-27); a sync that does not take is silent and latches
                // off for the airframe. Not gated on the systems survey's
                // glass_hsi: the readback is the guard.
                if gyro_sync.verify_due(now) {
                    // A failed read is NO measurement (the snapshot's 0.0
                    // default would read as a perfect sync).
                    let after = xp
                        .get_float("sim/cockpit/gyros/dg_drift_vac_deg")
                        .await
                        .ok()
                        .map(fold_drift_deg);
                    match gyro_sync.settle(after, now) {
                        Some(GyroSyncOutcome::Took { before, after }) => {
                            crate::blackbox::record(
                                "gyro_sync",
                                serde_json::json!({
                                    "took": true,
                                    "drift_deg": r1(before),
                                    "after_deg": r1(after),
                                }),
                            );
                            // `voice.alert` mirrors to the panel; an explicit
                            // `s.action(&line)` here too double-wrote the line
                            // to the chat (2026-07-24).
                            voice.alert(Priority::Advisory, gyro_sync_line(before));
                        }
                        Some(GyroSyncOutcome::DidNotTake { before, after }) => {
                            crate::blackbox::record(
                                "gyro_sync",
                                serde_json::json!({
                                    "took": false,
                                    "drift_deg": r1(before),
                                    "after_deg": r1(after),
                                    "latched": true,
                                }),
                            );
                        }
                        Some(GyroSyncOutcome::Unmeasured { before }) => {
                            crate::blackbox::record(
                                "gyro_sync",
                                serde_json::json!({
                                    "took": serde_json::Value::Null,
                                    "drift_deg": r1(before),
                                    "after_deg": serde_json::Value::Null,
                                }),
                            );
                        }
                        None => {}
                    }
                }
                if crew_on && !snap.on_ground && gyro_sync.due(snap.gyro_drift_deg, now) {
                    gyro_sync.last = Some(now);
                    if xp.run_command("sim/instruments/DG_sync_mag").await.is_ok() {
                        gyro_sync.fired(snap.gyro_drift_deg, now);
                    }
                }
                // The ATIS loop: while COM1 sits on the ATIS frequency the
                // broadcast repeats, cutting a new information letter when the
                // weather genuinely moves. Cheap gate first — tuned AND due —
                // so the weather set is read once per broadcast, not per tick.
                if let Some(com1) = com1 {
                    // The ATC tab tunes ANY in-range field's frequencies:
                    // follow COM1 to that field's ATIS instead of staying
                    // silent because it wasn't the active station's.
                    crate::station::atis_retune(com1, snap.lat, snap.lon, snap.alt_ft);
                    if crate::station::atis_due(com1, std::time::Instant::now()) {
                        let wx = crate::station::AtisWx {
                            zulu_sec: xp
                                .get_float(crate::datarefs::ZULU_TIME_SEC)
                                .await
                                .unwrap_or(0.0),
                            wind_from_deg: xp
                                .get_float(crate::datarefs::WIND_DIR_DEG_TRUE)
                                .await
                                .unwrap_or(0.0),
                            wind_kts: xp
                                .get_float(crate::datarefs::WIND_SPEED_MSC)
                                .await
                                .unwrap_or(0.0)
                                * 1.943844,
                            visibility_sm: xp
                                .get_float(crate::datarefs::VISIBILITY_SM)
                                .await
                                .unwrap_or(10.0),
                            qnh_hpa: xp
                                .get_float(crate::datarefs::QNH_PASCALS)
                                .await
                                .unwrap_or(101300.0)
                                / 100.0,
                            oat_c: xp
                                .get_float(crate::datarefs::OAT_DEG_C)
                                .await
                                .unwrap_or(15.0),
                        };
                        if let Some(report) =
                            crate::station::atis_tick(com1, &wx, std::time::Instant::now())
                        {
                            // The broadcast LOOPS on the radio, but the log
                            // is a transcript: one entry per edition (field
                            // or letter change), not one per repetition —
                            // the loop was flooding the panel (2026-07-10).
                            let edition = report.split('.').next().unwrap_or("").to_string();
                            if atis_logged.as_deref() != Some(edition.as_str()) {
                                atis_logged = Some(edition);
                                if let Some(s) = status.as_ref() {
                                    s.action(&format!("ATIS: {report}"));
                                }
                                crate::blackbox::record(
                                    "atis",
                                    serde_json::json!({ "msg": &report }),
                                );
                            }
                            voice.alert_station(
                                Priority::Advisory,
                                crate::voice::StationSpeaker::Atis,
                                report,
                            );
                        }
                    }
                }
                // After-takeoff handoff: armed by the tower's takeoff
                // clearance, spoken once airborne and climbing ("contact
                // Gibraltar Departure on one three zero decimal four"). It is
                // the TOWER talking on the tower frequency: tuned anywhere
                // else it goes into the void — missed radio is gone, never
                // replayed on another frequency (two controllers on one
                // radio, 2026-07-12).
                if let Some(line) = crate::station::handoff_tick(snap.alt_ft, snap.on_ground) {
                    crate::blackbox::record(
                        "atc_handoff",
                        serde_json::json!({ "alt": r1(snap.alt_ft), "msg": &line }),
                    );
                    if matches!(tuned_speaker, Some(crate::voice::StationSpeaker::Tower)) {
                        if let Some(s) = status.as_ref() {
                            s.action(&format!("TWR: {line}"));
                        }
                        voice.alert_station(
                            Priority::Advisory,
                            crate::voice::StationSpeaker::Tower,
                            line,
                        );
                    }
                }
                // Radar watch on an assigned squawk: "radar contact" once the
                // sim shows the code replying, "confirm squawk" nudges while
                // it doesn't. Transponder reads gated on an armed watch, so
                // flights without a radar service cost nothing. AIRBORNE
                // only: a departure code is set on the ground with the
                // transponder correctly in standby — nudging there is wrong,
                // and "radar contact" belongs after liftoff.
                // The watch belongs to the ASSIGNER (approach), but radio
                // physics belong to COM1: nothing is heard off frequency,
                // and on someone else's frequency the message arrives in
                // THAT controller's voice — a tower relaying radar's
                // "confirm squawk", exactly like the real thing (approach
                // nagged the pilot ACROSS the tower frequency, 2026-07-12).
                // Untuned, the watch just waits; the transponder reads cost
                // nothing while nobody could speak anyway.
                let squawk_ears = com1.and_then(crate::station::tuned_controller);
                if !snap.on_ground && crate::station::squawk_watch_active() {
                    if let Some(speaker) = squawk_ears {
                        let code = xp
                            .get_float(crate::datarefs::TRANSPONDER_CODE)
                            .await
                            .ok()
                            .map(|v| v.round() as i64);
                        let replying = xp
                            .get_float(crate::datarefs::TRANSPONDER_MODE)
                            .await
                            .is_ok_and(|v| v >= 2.0);
                        if let Some((line, _assigner)) =
                            crate::station::squawk_tick(code, replying, speaker, now)
                        {
                            if let Some(s) = status.as_ref() {
                                s.action(&format!("{}: {line}", speaker.tag()));
                            }
                            crate::blackbox::record(
                                "atc",
                                serde_json::json!({
                                    "route": "squawk_watch", "code": code, "replying": replying,
                                    "reply": &line,
                                }),
                            );
                            voice.alert_station(Priority::Advisory, speaker, line);
                        }
                    }
                }
                // The controller's squawk nudge is an INSTRUCTION to the
                // crew: recycle, or dial the code, and say so on both
                // channels (EGPH 2026-09-20: two "recycle transponder" calls,
                // nobody answered). Crew hands only; on its own task, since a
                // stepped knob takes a few hundred milliseconds.
                if let Some(nudge) = crate::party_line::take_squawk_nudge() {
                    if crate::mode::crew_actuates() {
                        let base = config.xplane_base_url.clone();
                        let v = voice.clone();
                        tokio::spawn(async move {
                            let xp = XPlaneClient::new(base);
                            let (check, said, readback) =
                                crate::tools::radio::answer_nudge(&xp, nudge.code, nudge.recycle)
                                    .await;
                            let mut rec = check.json();
                            rec["route"] = serde_json::json!("squawk_answer");
                            rec["recycle"] = serde_json::json!(nudge.recycle);
                            rec["ok"] = serde_json::json!(check.ok());
                            crate::blackbox::record("atc", rec);
                            let prio = if check.ok() {
                                Priority::Advisory
                            } else {
                                Priority::Caution
                            };
                            v.alert(prio, &said);
                            if let Some(words) = readback {
                                let cs = crate::circuit::crew_callsign(&xp).await;
                                crate::circuit::transmit(crate::circuit::radio_call(&cs, &words));
                            }
                        });
                    }
                }
                // A taxi clearance to a runway this airframe cannot use
                // (ETND 2026-09-24: 446 m of grass for an SF50 needing 780):
                // with a pilot taxiing, the copilot says both numbers and the
                // request to make, once per clearance. Full auto judges and
                // asks for itself (`full_auto::judge_cleared_runway`).
                {
                    let taxi_to = (crate::party_line::clearance_state()
                        == Some(crate::tower::State::TaxiCleared))
                    .then(crate::party_line::clearance_runway)
                    .flatten();
                    if taxi_to != cleared_runway_judged {
                        cleared_runway_judged = taxi_to.clone();
                        if let Some(rwy) =
                            taxi_to.filter(|_| crew_on && snap.on_ground && !crate::full_auto::running())
                        {
                            let (roll, mtow) = crate::aircraft::profile_takeoff_roll(&live.get());
                            let needed =
                                crate::aircraft::takeoff_needed_now_m(&xp, roll, mtow, 500.0).await;
                            if let crate::full_auto::ClearedFit::Short {
                                cleared,
                                length_m,
                                needed_m,
                                ask,
                            } = crate::full_auto::cleared_runway_fit(
                                &crate::party_line::field_runways(),
                                &rwy,
                                needed,
                            ) {
                                let line = match &ask {
                                    Some(a) => format!(
                                        "Runway {} is {:.0} metres, we need {:.0}. Ask the tower: {}.",
                                        crate::circuit::spell_runway(&cleared),
                                        length_m,
                                        needed_m,
                                        crate::full_auto::runway_request_call(&cleared, a)
                                    ),
                                    None => crate::full_auto::short_runway_line(
                                        length_m, needed_m, &cleared, None,
                                    ),
                                };
                                crate::blackbox::record(
                                    "atc",
                                    serde_json::json!({
                                        "route": "cleared_runway_short", "runway": cleared,
                                        "length_m": length_m.round(), "needed_m": needed_m.round(),
                                        "ask": ask, "reply": &line,
                                    }),
                                );
                                voice.alert(Priority::Caution, line);
                            }
                        }
                    }
                }
                // Landing-clearance coach. The approach clears you for the
                // APPROACH and hands you to the tower; the tower must still
                // say "cleared to land". Landing uncleared earns a phone
                // number (the compliance watch above) — so a beat after the
                // handoff, if no landing clearance has come and we're still
                // flying, nudge the pilot to report final. The world clears
                // the handoff on the landing clearance or touchdown, which
                // resets the latch here; a prompt reporter is never nagged.
                match crate::party_line::landing_handoff_runway().filter(|_| crew_on) {
                    Some(rwy) if !snap.on_ground => {
                        let seen = *landing_coach_seen.get_or_insert(now);
                        if !landing_coach_spoken
                            && now.duration_since(seen).as_secs() >= LANDING_COACH_GRACE_SECS
                        {
                            landing_coach_spoken = true;
                            let line = format!(
                                "That's the approach clearance, not a landing clearance. \
                                 Call the tower and report final, runway {rwy} — \
                                 or we touch down uncleared."
                            );
                            voice.alert(Priority::Advisory, line);
                        }
                    }
                    _ => {
                        landing_coach_seen = None;
                        landing_coach_spoken = false;
                    }
                }
                // Region-handoff coach. Full auto consumes the handoff itself
                // (readback, seven thousand, retune); with anyone else flying
                // the copilot takes it and, after a grace, names what the
                // pilot has not done yet — once, never re-asserted.
                if !crate::full_auto::ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
                    if let Some(h) = crate::party_line::take_region_handoff() {
                        region_coach = Some((h, now));
                    }
                }
                if let Some((h, seen)) = region_coach.as_ref() {
                    if !crew_on || snap.on_ground {
                        region_coach = None;
                    } else if now.duration_since(*seen).as_secs() >= REGION_COACH_GRACE_SECS {
                        let code = if h.squawk_vfr {
                            xp.get_float(crate::datarefs::TRANSPONDER_CODE)
                                .await
                                .ok()
                                .map(|v| v.round() as i64)
                        } else {
                            None
                        };
                        if let Some(line) = region_coach_line(h, com1, code) {
                            crate::blackbox::record(
                                "atc",
                                serde_json::json!({
                                    "route": "region_coach", "khz": h.khz, "com1": com1,
                                    "squawk_vfr": h.squawk_vfr, "code": code, "reply": &line,
                                }),
                            );
                            voice.alert(Priority::Advisory, line);
                        }
                        region_coach = None;
                    }
                }
                // The copilot's progressive taxi guide — the crew half of
                // the taxi clearance. The tower said the route once; the
                // pilot monitoring calls the turns just-in-time ("next
                // left, onto Whiskey"), the runway crossings, and the
                // holding point, in the copilot's own voice at Advisory.
                // Off-route it goes quiet: the compliance watch above owns
                // that conversation, and two voices saying it would nag.
                if crew_on && config.taxi_guide && snap.on_ground {
                    let taxiing = crate::party_line::clearance_state()
                        == Some(crate::tower::State::TaxiCleared);
                    if taxiing {
                        crate::party_line::with_taxi_route(|route| match route {
                            Some(watch)
                                if !taxi_guide.as_ref().is_some_and(|g| g.is_for(&watch.path)) =>
                            {
                                taxi_guide = crate::taxi_guide::Guide::build(watch);
                            }
                            Some(_) => {}
                            None => taxi_guide = None,
                        });
                        if let Some(cue) = taxi_guide
                            .as_mut()
                            .and_then(|g| g.tick(snap.lat, snap.lon, snap.groundspeed_kts, now))
                        {
                            if let Some(s) = status.as_ref() {
                                s.action(&format!("Taxi guide: {cue}"));
                            }
                            crate::blackbox::record(
                                "taxi_guide",
                                serde_json::json!({ "cue": &cue }),
                            );
                            // PERISHABLE: a cue that can't speak within ~8 s
                            // (60 m of taxi) is disinformation — the G11
                            // "next left" queued behind a party-line exchange
                            // and played 17 s late, 130 m past the junction
                            // (2026-07-13). Dropped cues are covered by the
                            // guide's missed-turn nudge and the panel strip.
                            // The copilot driving says only the safety
                            // calls; its own steering is on the panel.
                            let copilot_driving = crate::autotaxi::driving();
                            if !copilot_driving
                                || taxi_guide.as_ref().is_some_and(|g| g.last_cue_is_safety())
                            {
                                voice.alert_stale_by(
                                    Priority::Advisory,
                                    cue,
                                    now + std::time::Duration::from_secs(8),
                                );
                            }
                        }
                    } else {
                        taxi_guide = None;
                    }
                }
                // Live-traffic watches off one TCAS target read: the
                // go-around order (cancel the landing clearance a ghost on
                // the runway is about to contradict), the held takeoff
                // clearance release, and the party line (clearances narrated
                // for the ghosts themselves — LiveTraffic replays real
                // flights, so the tower "clears" what reality is about to do
                // anyway). Reads gated: nothing armed and chatter off means
                // no dataref traffic.
                {
                    let tower_tuned =
                        matches!(tuned_speaker, Some(crate::voice::StationSpeaker::Tower));
                    // A held release is spent once we ROLL on the runway: the
                    // tower saw the takeoff it was holding. Left armed it
                    // outlived a reject and replayed "traffic clear, cleared
                    // for takeoff" into the next mission (EGKK 2026-09-27).
                    if snap.on_ground
                        && crate::station::void_takeoff_hold_by_roll(snap.groundspeed_kts)
                    {
                        crate::blackbox::record(
                            "atc",
                            serde_json::json!({
                                "event": "hold_void", "why": "rolled",
                                "gs_kts": snap.groundspeed_kts,
                            }),
                        );
                    }
                    let hold_armed = crate::party_line::takeoff_hold_armed();
                    // A landing clearance withheld for traffic on the runway
                    // ("continue approach") or given: the runway watch above
                    // judged it this tick; the flavour below waits for it.
                    // Party-line flavor stands down while the taxi guide is
                    // working AND the aircraft is rolling: its cues are
                    // perishable, and a three-line Ryanair landing exchange
                    // cost the pilot the G11 turn (cue spoken 17 s late,
                    // 2026-07-13). But guide-active is NOT rolling — the
                    // guide rides the whole taxi clearance, and gating on it
                    // alone killed the party line for a 20-minute Gatwick
                    // taxi, most of it stationary (same day). Stopped, no cue
                    // is imminent; chatter is welcome. Calls addressed to US
                    // (compliance, hold releases) are unaffected either way.
                    let chatter_on = config.atc_chatter
                        && tower_tuned
                        && (taxi_guide.is_none() || snap.groundspeed_kts < 3.0);
                    // The wake watch needs the TCAS picture while the pilot
                    // is on the tower frequency on the ground — a heavy
                    // rotating ahead of you starts a clock whether or not
                    // the party line is narrating it.
                    let wake_watch = tower_tuned && snap.on_ground;
                    if let Some((_, flat, flon, felev)) = crate::station::active_field() {
                        if chatter_on || hold_armed || landing_held || cleared_to_land || wake_watch
                        {
                            // One read per tick: the runway watch's, when it
                            // made one.
                            let targets = match runway_watch_targets.take() {
                                Some(t) => t,
                                None => crate::station::read_tcas_targets(xp.view()).await,
                            };
                            crate::station::wake_note_targets(&targets, now);
                            // A withheld landing clearance is the runway
                            // watch's (above, whoever flies); the flavour
                            // waits for it.
                            if landing_held {
                            } else if hold_armed {
                                if let Some(line) = crate::station::takeoff_hold_tick_from(
                                    &targets,
                                    felev,
                                    snap.on_ground,
                                    Some((snap.lat, snap.lon)),
                                    now,
                                ) {
                                    crate::blackbox::record(
                                        "atc",
                                        serde_json::json!({
                                            "route": "hold_release", "reply": &line,
                                        }),
                                    );
                                    // The tower's clearance on the tower
                                    // frequency: tuned elsewhere it's said
                                    // into the void (ask again and the tower
                                    // re-states it), never replayed on another
                                    // controller's frequency.
                                    if tower_tuned {
                                        if let Some(s) = status.as_ref() {
                                            s.action(&format!("TWR: {line}"));
                                        }
                                        voice.alert_station(
                                            Priority::Advisory,
                                            crate::voice::StationSpeaker::Tower,
                                            line,
                                        );
                                    }
                                }
                            } else if chatter_on
                                && chatter_last
                                    .is_none_or(|t| now.duration_since(t).as_secs() >= 45)
                            {
                                let fname = crate::station::active_field()
                                    .map(|(n, _, _, _)| n)
                                    .unwrap_or_default();
                                // The strip the PLAYER holds (stands on, or
                                // landed on and not yet reported vacated): a
                                // ghost landing it gets "continue", not a
                                // narrated clearance over the player's head.
                                let blocked = crate::station::player_blocked_strip(
                                    snap.on_ground,
                                    snap.lat,
                                    snap.lon,
                                );
                                if let Some(exchange) = crate::station::chatter_tick(
                                    &targets,
                                    &fname,
                                    flat,
                                    flon,
                                    felev,
                                    blocked.as_deref(),
                                    now,
                                ) {
                                    chatter_last = Some(now);
                                    let transcript: Vec<&str> =
                                        exchange.lines.iter().map(|l| l.text.as_str()).collect();
                                    crate::blackbox::record(
                                        "atc",
                                        serde_json::json!({
                                            "route": "party_line",
                                            "reply": transcript.join(" / "),
                                        }),
                                    );
                                    // Pilot and tower lines queue in order
                                    // through the one arbiter — the exchange
                                    // plays as a dialogue, two voices. The
                                    // '~' marks AMBIENCE for the panel: an
                                    // overheard exchange, not a call to this
                                    // aircraft — the log dims it so calls
                                    // with our name on them stay bright.
                                    for l in exchange.lines {
                                        let who = if l.ghost {
                                            crate::voice::StationSpeaker::Ghost {
                                                seed: exchange.ghost_seed,
                                            }
                                        } else {
                                            crate::voice::StationSpeaker::Tower
                                        };
                                        if let Some(s) = status.as_ref() {
                                            s.action(&format!("~{}: {}", who.tag(), l.text));
                                        }
                                        voice.alert_station_overheard(
                                            Priority::Advisory,
                                            who,
                                            l.text,
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
                // Region switch: airborne it's a controller handoff; on the
                // ground (loaded the flight at a new field) it's the copilot
                // noting the local tower. Either way the main loop rebuilds
                // the station around the new airport. Gated by mode — the
                // region index survives an ATC-off toggle (cheap re-enable),
                // so this is the one world tick that would otherwise keep
                // running against a torn-down station.
                if let Some(sw) = (crate::mode::atc_enabled())
                    .then(|| crate::station::region_tick(snap.lat, snap.lon, snap.on_ground, now))
                    .flatten()
                {
                    let abnormal = crate::flight::published()
                        .is_some_and(|f| f.abnormal != crate::flight::Abnormal::Normal);
                    let said = !sw.crew
                        || region_note_said(snap.on_ground, abnormal, crate::persona::chatter());
                    crate::blackbox::record(
                        "atc_region",
                        serde_json::json!({
                            "lat": r1(snap.lat), "lon": r1(snap.lon),
                            "crew": sw.crew, "said": said, "msg": &sw.line
                        }),
                    );
                    if sw.crew && said {
                        // `voice.alert` mirrors to the panel; an explicit
                        // `s.action(&sw.line)` here too double-wrote the line
                        // to the chat (the Jerez relocation note, 2026-07-24).
                        voice.alert(Priority::Advisory, sw.line);
                    } else if sw.crew {
                        // Unsaid, still on the panel for a glance: mirror OR
                        // speak, never both.
                        if let Some(s) = status.as_ref() {
                            s.action(&sw.line);
                        }
                    } else {
                        if let Some(s) = status.as_ref() {
                            s.action(&format!("DEP: {}", sw.line));
                        }
                        voice.alert_station(
                            Priority::Advisory,
                            crate::voice::StationSpeaker::Departure,
                            sw.line,
                        );
                    }
                }
                if let Some(line) = crate::station::vector_tick(
                    snap.lat,
                    snap.lon,
                    snap.alt_ft,
                    snap.track_deg,
                    snap.on_ground,
                    std::time::Instant::now(),
                ) {
                    // Radio physics: the vectors are heard ONLY on the
                    // approach frequency. Off it, the controller still talks
                    // (the plan advances, the blackbox records the missed
                    // call) but the cockpit hears nothing — tuning away
                    // mid-vector loses the service, it doesn't mute-proof it
                    // ("ATC still speaking after I tuned another station",
                    // 2026-07-12).
                    let on_freq = com1.and_then(crate::station::tuned_controller)
                        == Some(crate::voice::StationSpeaker::Approach);
                    if on_freq {
                        if let Some(s) = status.as_ref() {
                            // The prefix names the speaking position on the panel.
                            s.action(&format!("APP: {line}"));
                        }
                    }
                    crate::blackbox::record(
                        "vectors",
                        serde_json::json!({
                            "lat": r1(snap.lat), "lon": r1(snap.lon),
                            "alt": r1(snap.alt_ft), "heard": on_freq, "msg": &line
                        }),
                    );
                    if on_freq {
                        voice.alert_station(
                            Priority::Advisory,
                            crate::voice::StationSpeaker::Approach,
                            line,
                        );
                    }
                }
                // Near miss: inside half a mile and 500 ft — the one air
                // picture where calm narration stops being enough. The tuned
                // controller issues the safety alert with an avoidance
                // suggestion; the CREW shouts it when no frequency is tuned
                // (a safety net must not depend on the radio, like the
                // incursion watch). Above 500 ft AGL only: real TCAS inhibits
                // alerts low, where the ghost lifting off behind a departure
                // would false-fire. Words only — the copilot never maneuvers
                // uncommanded.
                if !snap.on_ground && snap.agl_ft > 500.0 {
                    let due = near_miss_last.is_none_or(|t| now.duration_since(t).as_secs() >= 15);
                    if due {
                        if let Some(f) = crate::station::nearest_traffic_factor(xp.view())
                            .await
                            .filter(crate::station::is_near_miss)
                        {
                            near_miss_last = Some(now);
                            crate::blackbox::record(
                                "near_miss",
                                serde_json::json!({
                                    "nm": r1(f.nm), "clock": f.clock, "ft": f.ft
                                }),
                            );
                            let speaker = com1.and_then(crate::station::tuned_controller);
                            if let Some(who) = speaker {
                                let callsign = crate::circuit::crew_callsign(&xp)
                                    .await
                                    .unwrap_or_else(|| "traffic alert".to_string());
                                let line = format!(
                                    "{callsign}, {}",
                                    crate::station::near_miss_line(&f, false)
                                );
                                if let Some(s) = status.as_ref() {
                                    s.action(&format!("{}: {line}", who.tag()));
                                }
                                voice.alert_station(Priority::Warning, who, line);
                            } else {
                                let line = crate::station::near_miss_line(&f, true);
                                voice.alert(Priority::Warning, line);
                            }
                        }
                    }
                }
                // Traffic advisories: the controller narrates the nearest
                // factor off the TCAS picture (LiveTraffic or AI aircraft)
                // whenever one is watching — vectored to final, or simply on
                // any of the station's frequencies while airborne. At most
                // one call per 45 s. Narration only — separation stays the
                // pilot's job.
                let vectored = crate::party_line::vectored_callsign();
                let controller = if vectored.is_some() {
                    Some(crate::voice::StationSpeaker::Approach)
                } else if !snap.on_ground {
                    com1.and_then(crate::station::tuned_controller)
                } else {
                    None
                };
                if let Some(speaker) = controller {
                    // Never two voices on one conflict: while the near-miss
                    // alert is speaking (15 s cadence), the calm narration of
                    // the same target stands down.
                    let due = traffic_last.is_none_or(|t| now.duration_since(t).as_secs() >= 45)
                        && near_miss_last.is_none_or(|t| now.duration_since(t).as_secs() >= 45);
                    if due {
                        // Called once, and again only when it is more of a
                        // factor (`traffic_worth_recalling`).
                        let factor = crate::station::nearest_traffic_factor(xp.view()).await;
                        let fresh = factor.as_ref().filter(|f| {
                            traffic_called.as_ref().is_none_or(|c| crate::station::traffic_worth_recalling(c, f))
                        });
                        let advisory = fresh.map(crate::station::advisory_line);
                        if let Some(f) = fresh {
                            traffic_called = Some(f.clone());
                        }
                        // Close the loop: the last factor is gone.
                        if factor.is_none() && traffic_open {
                            traffic_open = false;
                            traffic_called = None;
                            traffic_last = Some(now);
                            let line = "clear of traffic.".to_string();
                            if let Some(s) = status.as_ref() {
                                s.action(&format!("{}: {line}", speaker.tag()));
                            }
                            voice.alert_station(Priority::Advisory, speaker, line);
                        }
                        if let Some(call) = advisory {
                            traffic_open = true;
                            traffic_last = Some(now);
                            // Address the flight when vectoring gave us the
                            // callsign; otherwise read the tail number.
                            let callsign = match &vectored {
                                Some(c) => c.clone(),
                                None => crate::circuit::crew_callsign(&xp)
                                    .await
                                    .unwrap_or_else(|| "traffic alert".to_string()),
                            };
                            let line = format!("{callsign}, {call}");
                            if let Some(s) = status.as_ref() {
                                s.action(&format!("{}: {line}", speaker.tag()));
                            }
                            crate::blackbox::record(
                                "traffic_advisory",
                                serde_json::json!({ "msg": &line }),
                            );
                            voice.alert_station(Priority::Advisory, speaker, line);
                        }
                    }
                }
            } else if snap.on_ground {
                let _ = crate::station::vector_tick(
                    snap.lat,
                    snap.lon,
                    snap.alt_ft,
                    snap.track_deg,
                    snap.on_ground,
                    std::time::Instant::now(),
                );
            }

            // Talk-down: PAR-style callouts down the synthesized approach path,
            // armed by the visual_approach tool. Advisory narration only — the
            // pilot flies. Muted while the copilot itself is flying (recover /
            // autoland) or the panel mute is on, same as every other callout.
            // Crew-gated: the guide IS the copilot talking.
            if let Some(path) = crate::approach_guide::armed().filter(|_| crew_on) {
                if snap.on_ground {
                    // Touchdown completes the guidance — but only if we actually
                    // guided in the air. Armed on the ground (pre-takeoff, or
                    // testing on the ramp) it stays armed for the flight instead
                    // of instantly "completing".
                    if guide_last.is_some() || guide_handed_over {
                        crate::approach_guide::cancel();
                        if !quiet {
                            voice.alert(Priority::Advisory, "Down. Guidance complete.".to_string());
                        }
                        (guide_last, guide_handed_over) = (None, false);
                    }
                } else {
                    let d =
                        crate::approach_guide::deviations(&path, snap.lat, snap.lon, snap.alt_ft);
                    if d.dist_nm > 35.0 {
                        crate::approach_guide::cancel();
                        if !quiet {
                            voice.alert(
                                Priority::Advisory,
                                "We've left the approach area, guidance dropped.".to_string(),
                            );
                        }
                        (guide_last, guide_handed_over) = (None, false);
                    } else if !quiet && !guide_handed_over {
                        if let Some(line) = crate::approach_guide::callout(&d) {
                            let due = guide_last.is_none_or(|t| {
                                t.elapsed().as_secs_f64()
                                    >= crate::approach_guide::cadence_secs(d.dist_nm)
                            });
                            if due {
                                if line.starts_with("Over the threshold") {
                                    guide_handed_over = true; // once — the flare is theirs
                                }
                                crate::blackbox::record(
                                    "talkdown",
                                    serde_json::json!({
                                        "apt": path.icao, "rwy": path.runway_id, "dist": r1(d.dist_nm),
                                        "xtk": r1(d.xtk_nm), "vdev": r1(d.vdev_ft), "msg": &line
                                    }),
                                );
                                voice.alert(Priority::Advisory, line);
                                guide_last = Some(now);
                            }
                        }
                    }
                }
            } else if guide_last.is_some() || guide_handed_over {
                (guide_last, guide_handed_over) = (None, false); // cancelled by voice
            }
            if let Some((last, last_t)) = prev_ias {
                let dt = now.duration_since(last_t).as_secs_f64();
                // Ignore a stretched dt (a skipped/stalled tick) — the rate would
                // be meaningless. Only update inside a sane sampling window.
                if (0.1..=1.0).contains(&dt) {
                    let inst = (snap.ias_kts - last) / dt;
                    trend = 0.6 * trend + 0.4 * inst;
                }
            }
            prev_ias = Some((snap.ias_kts, now));
            snap.ias_trend_kts_per_s = trend;

            // Black box: one state record per tick, all from the snapshot.
            // Bank and track ride along for the offline ACS grade
            // (`instructor::grade_flight`: steep turns, touchdown drift).
            let (lat, lon, alt) = (snap.lat, snap.lon, snap.alt_ft);
            // The hands and the levers, batched in one join: what the pilot
            // and the loops were doing, so a sortie can prove nobody touched
            // it and a complaint ("jerky braking") has a number.
            let pitch_due = pitch_reads_due(snap.on_ground, snap.ap_servos_on, crew_yoke_fresh());
            let inputs = read_inputs(&xp, pitch_due).await;
            if let Some(p) = &inputs.pitch {
                crate::climb_watch::publish_telemetry(p.telemetry());
            }
            crate::climb_watch::publish_sim_speed(inputs.sim_speed);
            note_interventions(&inputs, flight.control());
            crate::blackbox::record(
                "state",
                serde_json::json!({
                    "lat": lat, "lon": lon, "hdg": r1(snap.heading_deg), "alt": r1(alt),
                    "pos": snap.pos_source.map(|s| s.as_str()),
                    "bank": r1(snap.bank_deg), "track": snap.track_deg.map(r1),
                    "ias": r1(snap.ias_kts), "gs": r1(snap.groundspeed_kts),
                    "vsi": r1(snap.vsi_fpm), "vs": snap.vs_fpm.map(r1), "agl": r1(snap.agl_ft),
                    "ground": snap.on_ground, "flap": snap.flap_ratio,
                    "park": r1(snap.park_brake), "cabin": r1(snap.cabin_altitude_ft),
                    "fuel_kg": snap.fuel_kg, "eng": snap.engine_running,
                    "phase": format!("{:?}", snap.phase),
                    "fphase": format!("{:?}", flight.phase()), "crashed": crashed,
                    "ctl": format!("{:?}", flight.control()),
                    "taxi": taxi_line.map(|l| l.json()),
                    "in": inputs.json(),
                }),
            );

            // Panel settings + the live flight strip. The mute switch comes from
            // the plugin's settings popup; unresolvable (no plugin) = unmuted.
            let mute = xp
                .get_float("xcopilot/opt/monitor_mute")
                .await
                .is_ok_and(|v| v > 0.5);
            PANEL_MUTE.store(mute, std::sync::atomic::Ordering::Relaxed);
            if let Some(s) = status.as_ref() {
                let mut strip = compose_strip(&snap);
                // The glanceable next-turn line: on a busy frequency the
                // spoken cue can be late or dropped (perishable) — the strip
                // is the fallback the pilot can always read.
                if let Some(hint) = taxi_guide
                    .as_ref()
                    .and_then(|g| g.strip_hint(snap.lat, snap.lon))
                {
                    if strip.is_empty() {
                        strip = hint;
                    } else {
                        strip = format!("{strip} | {hint}");
                    }
                }
                if strip != last_strip {
                    s.strip(&strip);
                    last_strip = strip;
                }
                // The turn card behind the panel's arrow: the same geometry
                // as the strip's hint, structured, and empty (card hidden)
                // whenever there is no guide. It rides the periodic resend
                // below so a plugin reload gets it back.
                let card = taxi_guide
                    .as_ref()
                    .and_then(|g| g.turn_card(snap.lat, snap.lon))
                    .map(|c| c.payload())
                    .unwrap_or_default();
                if card != last_card || auto_resend == 0 {
                    s.taxi(&card);
                    last_card = card;
                }
                // The Flight tab's automation feed, published here rather than
                // the 1 Hz radio poller because that thread only exists when
                // ATC is configured — auto status must flow with ATC off.
                let auto = crate::auto_status::compose();
                // The Route tab's load sheet rides the same gate and resend.
                let sheet =
                    crate::simbrief::loadsheet_payload(crate::simbrief::loaded_plan().as_ref());
                if sheet != last_sheet || auto_resend == 0 {
                    s.load_sheet(&sheet);
                    last_sheet = sheet;
                }
                // The Settings tab's resolved callsign under its field
                // ("spoken|source"), through the one resolver: every 5 s
                // (resent, since a plugin reload zeroes it) and at once after
                // a callsign setting changed. A failed read sends nothing.
                let due = CALLSIGN_STALE.swap(false, std::sync::atomic::Ordering::Relaxed)
                    || callsign_at.map_or(true, |t| t.elapsed() >= Duration::from_secs(5));
                if due {
                    callsign_at = Some(Instant::now());
                    if let Some(r) = crate::callsign::live(xp.view()).await {
                        s.callsign(&format!("{}|{}", r.spoken, r.source.as_str()));
                    }
                }
                if auto != last_auto || auto_resend == 0 {
                    s.auto_status(&auto);
                    last_auto = auto;
                    auto_resend = 5;
                } else {
                    auto_resend -= 1;
                }
            }

            // How busy the flight deck is, for the voice's pace, the reply's
            // pause and the asides below (`tempo::busy_now`).
            let busy = crate::tempo::busy_now(&crate::tempo::Load {
                on_ground: snap.on_ground,
                groundspeed_kts: snap.groundspeed_kts,
                agl_ft: snap.agl_ft,
                critical_loop: crate::flight_state::crew_in_critical_phase(),
                crew_busy: crate::flow::ACTIVE.load(std::sync::atomic::Ordering::Relaxed)
                    || crate::instructor::active(),
            });
            crate::tempo::set_workload(busy);
            // The rapport SEES every tick, a wreck included (it drops the
            // landing word for one), and speaks only what its tick returns.
            // Its wheels are the flight model's (this tick's events, on the
            // model's clock), never a second touchdown detector (#85).
            if crew_on {
                let view = crate::rapport::View {
                    t_s: wall_t,
                    on_ground: snap.on_ground,
                    groundspeed_kts: snap.groundspeed_kts,
                    agl_ft: snap.agl_ft,
                    vsi_fpm: snap.vsi_fpm,
                    ias_kts: snap.ias_kts,
                    lat: snap.lat,
                    lon: snap.lon,
                    flap_ratio: snap.flap_ratio,
                    gear_down: snap.gear_down,
                    automation_flying: snap.automation_flying,
                    dest_dist_nm: snap.dest_dist_nm,
                    crashed: voice.crashed(),
                    instructing: crate::mode::instructing(),
                    busy,
                    quiet,
                };
                let persona = crate::persona::Persona::live();
                for line in
                    rapport.tick(&view, &flight, &flight_events, &rapport_airframe, &persona)
                {
                    crate::blackbox::record(
                        "rapport",
                        serde_json::json!({ "event": "said", "line": line.text() }),
                    );
                    voice.alert(Priority::Advisory, line.text().to_string());
                }
            }

            // A wreck gets no advice: the flight model freezes at the impact
            // values (the 2026-07-13 crash read 106 kts and -1,463 fpm
            // forever), so the checks would evaluate a flight that already
            // ended. They stand down until the flight resets. And with no
            // crew aboard (ATC-only mode) there is nobody to speak them.
            if crew_on && !voice.crashed() {
                let flying = snap.automation_flying;
                if flying {
                    had_flying_control = true;
                    control_idle_since = None;
                } else if had_flying_control && !snap.on_ground {
                    let since = *control_idle_since.get_or_insert(now);
                    if now.duration_since(since) >= Duration::from_secs(3) {
                        had_flying_control = false;
                        landing_summary.note_interruption();
                        // The pilot's own "my controls" was answered already;
                        // said again it landed on the next "you have control"
                        // (LEMG 13 1790690743, 2026-09-29, #54).
                        if !pilot_released_recently() {
                        voice.alert(Priority::Caution, "Flight automation has stopped. I'm monitoring only; you have control. Check the autopilot.".to_string());
                        }
                    }
                } else if snap.on_ground {
                    had_flying_control = false;
                }
                let xtk = crate::autoland::last_tracking_error();
                if let Some(line) = landing_summary.tick(
                    now,
                    snap.on_ground,
                    snap.agl_ft,
                    snap.vsi_fpm,
                    snap.groundspeed_kts,
                    crate::autoland::ACTIVE.load(std::sync::atomic::Ordering::Relaxed),
                    xtk,
                ) {
                    // The record for the debrief; the SPOKEN word about the
                    // landing is the rapport's, said the way a person would
                    // ("Nice one, about 180 feet a minute"), not a report.
                    crate::blackbox::record("landing_summary", serde_json::json!({"msg": &line}));
                }
                let mut speed_bad = false;
                let mut speed_spoke = false;
                let mut cabin_bad = false;
                let mut cabin_spoke = false;
                for check in &checks {
                    let state = states.get_mut(check.id()).expect("state per check");
                    match check.eval(&snap) {
                        Some(msg) => {
                            let speed = check.id().starts_with("airspeed_")
                                || check.id().starts_with("overspeed_");
                            let cabin = check.id().starts_with("cabin_alt_");
                            speed_bad |= speed;
                            cabin_bad |= cabin;
                            state.observe_level(check.repeat_level(&snap));
                            let spoke = state.active(
                                check.id(),
                                msg,
                                check.severity(),
                                check.min_duration(),
                                &voice,
                                status.as_ref(),
                                now,
                            );
                            speed_spoke |= speed && spoke;
                            cabin_spoke |= cabin && spoke;
                            // The pilot's fuel panel: said, never written.
                            if spoke
                                && check.id() == CENTRE_TANK_DRY
                                && check.fix_write(&snap).is_none()
                            {
                                if let Some(t) = snap.fuel_tanks.as_ref() {
                                    for i in crate::tools::fuel_pumps::pumps_to_switch_off(
                                        &pump_tanks,
                                        Some(t),
                                        snap.engine_running,
                                    ) {
                                        crate::blackbox::record(
                                            "fuel_pumps",
                                            serde_json::json!({
                                                "event": "advised", "source": "monitor",
                                                "tank": i, "kg": t.kg.get(i),
                                                "before": t.pumps.get(i),
                                            }),
                                        );
                                    }
                                }
                            }
                            // On the rising edge, the copilot ACTS on SOP rows that
                            // carry a fix (strobes, landing lights) instead of only
                            // advising — crew business, so it's already behind the
                            // `crew_on` gate above, and the tick never runs in
                            // replay/pause. Fire-and-forget: the switch flips and
                            // the check clears next tick.
                            if spoke && crate::mode::crew_actuates() {
                                if let Some(cmd) = check.fix_action() {
                                    let _ = xp.run_command(cmd).await;
                                }
                                // A cabin climbing through ten thousand: the
                                // crew's own air first (bleed, the oxygen
                                // supply; the masks are `masks_tick`'s), and
                                // past the warning band a follower that is
                                // flying goes DOWN (its terrain floor still
                                // rules). Two "cabin altitude" callouts went by
                                // unanswered on EDLP-LOWI and the pilot blacked
                                // out at FL260 (2026-09-24).
                                if cabin {
                                    let air = crate::tools::cabin_air::ensure_on(&xp).await;
                                    if let Some(line) = air.line() {
                                        voice.alert(Priority::Warning, line);
                                    }
                                    if check.id() == "cabin_alt_warning"
                                        && snap.alt_ft > crate::tools::cabin_air::HYPOXIA_DESCENT_FT + 500.0
                                        && crate::procedure::take_altitude(
                                            crate::tools::cabin_air::HYPOXIA_DESCENT_FT,
                                        )
                                    {
                                        voice.alert(
                                            Priority::Warning,
                                            "Descending to ten thousand for the cabin. \
                                             Ask for a new level once it holds."
                                                .to_string(),
                                        );
                                    }
                                }
                                if let Some((dref, value)) = check.fix_write(&snap) {
                                    // The altimeter goes through the proving
                                    // setter: on the 737 a dial write is a
                                    // silent no-op every 20 s otherwise.
                                    if dref == BARO_KNOB {
                                        // The line above promised a reset; a
                                        // knob that did not get there is said.
                                        if let Ok(now) =
                                            crate::tools::instruments::set_altimeter(&xp, value)
                                                .await
                                        {
                                            let sync = crate::tools::instruments::AltimeterSync {
                                                was: now,
                                                now,
                                                want: value.clamp(28.0, 31.5),
                                            };
                                            crate::blackbox::record(
                                                "callout",
                                                serde_json::json!({"id": "altimeter_reset", "now": now, "want": sync.want, "took": sync.took()}),
                                            );
                                            if !sync.took() {
                                                voice.alert(Priority::Warning, sync.line());
                                            }
                                        }
                                    } else if dref == crate::tools::fuel_pumps::TANK_PUMP_ON {
                                        // An indexed switch per tank, through
                                        // its one writer, read back: the tanks
                                        // are the decision's own.
                                        let tanks = crate::tools::fuel_pumps::pumps_to_switch_off(
                                            &pump_tanks,
                                            snap.fuel_tanks.as_ref(),
                                            snap.engine_running,
                                        );
                                        let done = crate::tools::fuel_pumps::switch_pumps(
                                            &xp,
                                            &tanks,
                                            value > 0.5,
                                        )
                                        .await;
                                        crate::tools::fuel_pumps::record("switched", "monitor", &done);
                                        if let Some(line) = crate::tools::fuel_pumps::failed_line(&done) {
                                            voice.alert(Priority::Caution, line);
                                        }
                                    } else if dref == crate::tools::anti_ice::MONITOR_FIX {
                                        // A set of switches, each read back
                                        // through the tool's one writer.
                                        let wants = snap
                                            .icing
                                            .as_ref()
                                            .map(crate::tools::anti_ice::practice_wants)
                                            .unwrap_or_default();
                                        if let Some(line) = crate::tools::anti_ice::monitor_fix(
                                            &xp,
                                            &wants,
                                            value > 0.5,
                                        )
                                        .await
                                        {
                                            voice.alert(Priority::Caution, line);
                                        }
                                    } else {
                                        let _ = xp.set_float(dref, value).await;
                                    }
                                }
                            }
                        }
                        None => state.clear(),
                    }
                }
                let speed_clear = airspeed_recovery.tick(speed_bad, speed_spoke, now);
                let cabin_clear = cabin_recovery.tick(cabin_bad, cabin_spoke, now);
                if !quiet && !snap.on_ground {
                    if speed_clear {
                        voice.alert(Priority::Advisory, "Airspeed recovered.".to_string());
                    }
                    if cabin_clear {
                        voice.alert(
                            Priority::Advisory,
                            "Cabin altitude back below the caution level.".to_string(),
                        );
                    }
                }
            }
        } else if xp.get_float(crate::datarefs::ON_GROUND_ANY).await.is_err() {
            let since = *lost_since.get_or_insert_with(Instant::now);
            if seen_sim && !disconnected && since.elapsed() >= Duration::from_secs(3) {
                disconnected = true;
                had_flying_control = false;
                crate::flight_state::stand_down();
                crate::blackbox::record(
                    "control_ownership",
                    serde_json::json!({"event":"connection_lost"}),
                );
                if crate::mode::copilot_enabled() {
                    voice.alert(Priority::Warning, "Connection to X-Plane lost. Flight automation is stopped. You'll have control when the sim returns.".to_string());
                }
            }
        } else {
            lost_since = None;
        }
        tokio::time::sleep(period).await;
    }
}

/// The alternator is normally offline for a few seconds during every engine
/// start (the engine catches mid-crank, before the ALT switch goes on), so we
/// only warn once it has stayed offline this long while running — a genuine
/// "you forgot the alternator", not the start transient.
const ALTERNATOR_SUSTAIN: Duration = Duration::from_secs(60);

/// Engine running but the alternator switch is off → the bus is being drained
/// with nothing charging it. A classic config slip a human copilot catches.
struct AlternatorOffline {
    noun: &'static str,
}

impl Check for AlternatorOffline {
    fn id(&self) -> &'static str {
        "alternator_offline"
    }
    fn severity(&self) -> Severity {
        Severity::Warning
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        (s.engine_running && !s.alternator_on)
            .then(|| format!("{}'s offline — we're running on the battery.", self.noun))
    }
    fn min_duration(&self) -> Duration {
        ALTERNATOR_SUSTAIN
    }
}

/// The directional gyro has precessed away from the magnetic compass. Vacuum DGs
/// drift slowly, and on a long leg it's easy to forget to realign — but a
/// right-seat pilot doesn't nag about it, they reach over and fix it ("the
/// copilot should adjust it and say so", 2026-07-09). The monitor loop performs
/// the sync itself (the same DG_sync_mag command as the align_gyro tool) and
/// announces it with this line, once the drift READ BACK shows it took
/// (`GyroSync`). Pure so it unit-tests; rounds to 5 degrees so the figure
/// sounds like an estimate, not false precision.
fn gyro_sync_line(drift_deg: f64) -> String {
    let approx = (drift_deg.abs() / 5.0).round() * 5.0;
    format!("Heading indicator had drifted about {approx:.0} degrees, re-synced to the compass.")
}

/// Fold a DG drift into -180..180, the same as the `align_gyro` tool, so a
/// small negative drift does not read as ~360.
fn fold_drift_deg(raw: f64) -> f64 {
    (raw + 180.0).rem_euclid(360.0) - 180.0
}

/// Time between auto-sync attempts.
const GYRO_SYNC_COOLDOWN: Duration = Duration::from_secs(120);
/// The sync is read back this long after the command (the sim applies a
/// command on its next frame; a beat leaves room for a slow one).
const GYRO_VERIFY_AFTER: Duration = Duration::from_secs(1);
/// Past this with no successful read the readback gives up: unmeasured, silent,
/// no latch (a failed read is no measurement either way).
const GYRO_VERIFY_GIVE_UP: Duration = Duration::from_secs(3);

/// Did the sync take? The drift must have at least halved: a real
/// DG_sync_mag zeroes it, and an airframe whose heading is not a vacuum DG
/// (the stock 737: IRS and glass) keeps drifting as if nothing was pressed.
fn gyro_sync_took(before_deg: f64, after_deg: f64) -> bool {
    after_deg.abs() <= before_deg.abs() / 2.0
}

/// What a readback found.
#[derive(Debug, Clone, Copy, PartialEq)]
enum GyroSyncOutcome {
    /// The drift fell: say so.
    Took { before: f64, after: f64 },
    /// The command was accepted and did nothing: silent, and latched off.
    DidNotTake { before: f64, after: f64 },
    /// No read landed in time: silent, not latched.
    Unmeasured { before: f64 },
}

/// The monitor's DG auto-sync, per airframe (the loop resets it on a swap):
/// the cooldown, the readback owed after a fire, and the latch that stops an
/// airframe whose sync does nothing from being "re-synced" every 120 s.
#[derive(Debug, Default)]
struct GyroSync {
    /// When the command was last tried (the cooldown's clock).
    last: Option<Instant>,
    /// Drift at the fire and when, while its readback is owed.
    pending: Option<(f64, Instant)>,
    /// The sync was read back and did nothing on this airframe.
    ineffective: bool,
}

impl GyroSync {
    /// A drift worth syncing, off cooldown, no readback owed, not latched.
    fn due(&self, drift_deg: f64, now: Instant) -> bool {
        !self.ineffective
            && self.pending.is_none()
            && drift_deg.abs() >= GYRO_DRIFT_ADVISORY_DEG
            && self
                .last
                .is_none_or(|t| now.duration_since(t) >= GYRO_SYNC_COOLDOWN)
    }

    /// The command was accepted: a readback is owed.
    fn fired(&mut self, before_deg: f64, now: Instant) {
        self.pending = Some((before_deg, now));
    }

    /// Time to read the drift back.
    fn verify_due(&self, now: Instant) -> bool {
        self.pending
            .is_some_and(|(_, at)| now.duration_since(at) >= GYRO_VERIFY_AFTER)
    }

    /// Judge a readback (`None` = the read failed). A failed read inside the
    /// give-up window keeps the readback owed and says nothing yet.
    fn settle(&mut self, after: Option<f64>, now: Instant) -> Option<GyroSyncOutcome> {
        let (before, at) = self.pending?;
        let outcome = match after {
            Some(after) if gyro_sync_took(before, after) => GyroSyncOutcome::Took { before, after },
            Some(after) => {
                self.ineffective = true;
                GyroSyncOutcome::DidNotTake { before, after }
            }
            None if now.duration_since(at) >= GYRO_VERIFY_GIVE_UP => {
                GyroSyncOutcome::Unmeasured { before }
            }
            None => return None,
        };
        self.pending = None;
        Some(outcome)
    }
}

/// Flaps extended above their speed limit (Vfe) — the per-detent limits from the
/// profile. The applicable limit is the lowest Vfe of any detent at or below the
/// current flap position (Vfe drops as the flaps extend further). Airborne only.
struct FlapOverspeed {
    /// (detent ratio, Vfe knots) for detents that carry a limit.
    limits: Vec<(f64, Option<f64>)>,
}

/// The flap overspeed caution speaks this far over the placard.
const FLAP_OVERSPEED_MARGIN_KTS: f64 = 5.0;
impl Check for FlapOverspeed {
    fn id(&self) -> &'static str {
        "flap_overspeed"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || s.flap_ratio < 0.02 {
            return None;
        }
        // Lowest Vfe among detents we've reached: the one placard lookup.
        let vfe = crate::aircraft::flap_placard_kts(s.flap_ratio, &self.limits)
            .unwrap_or(f64::INFINITY);
        if vfe.is_finite() && s.ias_kts > vfe + FLAP_OVERSPEED_MARGIN_KTS {
            Some(format!("Flap overspeed, {:.0} knots over the {vfe:.0} flap limit. Slow down or bring the flaps up.", s.ias_kts))
        } else {
            None
        }
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(2)
    }
}

/// Elevator/stabilizer trim pinned at (or essentially at) its travel limit while
/// airborne — out of trim authority, the signature of a mistrim or a runaway. Set
/// high (0.98) so normal trim, including the MD-80's nose-down Mach trim at speed,
/// doesn't trip it; only a trim that's actually hit the stop does.
const TRIM_LIMIT: f64 = 0.98;

struct StabTrim;
impl Check for StabTrim {
    fn id(&self) -> &'static str {
        "stab_trim_limit"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || s.elevator_trim.abs() < TRIM_LIMIT {
            return None;
        }
        let dir = if s.elevator_trim < 0.0 {
            "nose down"
        } else {
            "nose up"
        };
        Some(format!(
            "Stabilizer trim's at the {dir} limit, check the trim."
        ))
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(5)
    }
}

/// Fuel endurance (minutes remaining at the current burn) has dropped into a
/// reserve band. Tracking *time*, not gallons, is what a real copilot does — it
/// folds in the power setting. Two instances run: a Caution band between the
/// thresholds and a Warning band below the lower one; the bands are disjoint so
/// only one speaks. Airborne + engine-running only — on the ground, idle burn
/// makes "minutes remaining" misleadingly huge (a separate pre-takeoff
/// quantity check would be the way to cover that case).
struct FuelEndurance {
    id: &'static str,
    severity: Severity,
    /// Fires when endurance ≤ this many minutes …
    at_or_below_min: f64,
    /// … and (if set) still above this — the next, more urgent band owns below it.
    above_min: Option<f64>,
    /// The profile's cruise burn, kg/hr: the most the estimate will ever
    /// assume the aircraft keeps burning (see [`endurance_minutes`]).
    cruise_burn_kg_hr: Option<f64>,
    /// The band has been entered this flight. The estimate rides the live
    /// flow, and a governor working the lever walks it back and forth over
    /// the band's edge: every wobble was a cleared-and-recurred episode, and
    /// the 737 heard the same "about 60 minutes" every 31 s for four minutes
    /// (EGSS, 2026-09-22). Once in, the band holds until the estimate is
    /// [`FUEL_BAND_HYSTERESIS_MIN`] clear of it.
    entered: std::sync::atomic::AtomicBool,
}

const FUEL_BAND_HYSTERESIS_MIN: f64 = 10.0;
/// A fuel callout is said again for each of these lost, never on a clock.
const FUEL_REPEAT_STEP_MIN: f64 = 10.0;

/// Minutes of fuel at the burn the flight will actually average: the live
/// flow, capped at the profile's cruise burn. Takeoff and climb thrust are
/// transient by definition, and dividing the tanks by them is how the 737
/// with a correct 6,672 kg block heard "fuel's getting low, about 45
/// minutes" on every departure (a 9,000 kg/hr takeoff flow against a
/// 2,500 kg/hr cruise; LFLL, 2026-09-17), then again as the estimate swung
/// through the 60-minute band in the climb. Below the cruise burn the live
/// flow stands, optimistic side and all (a profile with no burn keeps the
/// old arithmetic). `None` when nothing is burning.
pub fn endurance_minutes(
    fuel_kg: Option<f64>,
    flow_kg_sec: Option<f64>,
    cruise_burn_kg_hr: Option<f64>,
) -> Option<f64> {
    let (kg, flow) = (fuel_kg?, flow_kg_sec?);
    let burn = match cruise_burn_kg_hr {
        Some(c) if c > 0.0 => flow.min(c / 3600.0),
        _ => flow,
    };
    (burn > 1e-4).then(|| kg / burn / 60.0)
}

impl Check for FuelEndurance {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        self.severity
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || !s.engine_running {
            return None;
        }
        let min = endurance_minutes(s.fuel_kg, s.fuel_flow_kg_sec, self.cruise_burn_kg_hr)
            .or(s.endurance_min)?;
        use std::sync::atomic::Ordering::Relaxed;
        let above_floor = self.above_min.is_none_or(|a| min > a);
        let in_band = min <= self.at_or_below_min && above_floor;
        let held = self.entered.load(Relaxed)
            && min <= self.at_or_below_min + FUEL_BAND_HYSTERESIS_MIN
            && above_floor;
        self.entered.store(in_band || held, Relaxed);
        if !(in_band || held) {
            return None;
        }
        // Round to the nearest 5 minutes: it's an estimate, not a stopwatch.
        let approx = (min / 5.0).round() * 5.0;
        // The first line of a band introduces it; the later ones are a crew
        // keeping count, each said once as another ten minutes goes.
        let first = min > self.at_or_below_min - FUEL_REPEAT_STEP_MIN;
        Some(match (self.severity, first) {
            (Severity::Warning, true) => {
                format!("Low fuel, about {approx:.0} minutes at this burn. Plan to land.")
            }
            (Severity::Warning, false) if approx <= 10.0 => {
                format!("About {approx:.0} minutes of fuel. We land now.")
            }
            (Severity::Warning, false) => {
                format!("Fuel is down to about {approx:.0} minutes. We need to be landing.")
            }
            (_, true) => format!("Fuel's getting low, about {approx:.0} minutes remaining."),
            (_, false) => format!("Fuel check, about {approx:.0} minutes now."),
        })
    }
    /// Each ten minutes lost is a new level, and a new level is the only
    /// thing that says a fuel line twice (see `FireState::should_speak`).
    fn repeat_level(&self, s: &Snapshot) -> Option<i64> {
        let min = endurance_minutes(s.fuel_kg, s.fuel_flow_kg_sec, self.cruise_burn_kg_hr)
            .or(s.endurance_min)?;
        Some(((self.at_or_below_min - min) / FUEL_REPEAT_STEP_MIN).floor() as i64)
    }
}

/// On the ground with the engine running and not enough fuel for a sensible
/// flight. The in-flight [`FuelEndurance`] check can't help here — idle burn
/// makes its "minutes remaining" near-infinite — so this estimates endurance
/// from a nominal *cruise* burn and cautions (once) below the reserve. Catches
/// "you're about to depart low on fuel" before you're airborne. Advisory-grade
/// Caution: a one-time nudge to go refuel, not a repeating nag.
struct FuelPreTakeoff {
    caution_min: f64,
    cruise_burn_kg_hr: f64,
}

impl Check for FuelPreTakeoff {
    fn id(&self) -> &'static str {
        "fuel_pretakeoff"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if !s.on_ground || !s.engine_running || self.cruise_burn_kg_hr <= 0.0 {
            return None;
        }
        let fuel = s.fuel_kg?;
        let endurance_min = fuel / self.cruise_burn_kg_hr * 60.0;
        if endurance_min >= self.caution_min {
            return None;
        }
        let approx = (endurance_min / 5.0).round() * 5.0;
        Some(format!(
            "Low fuel for departure, about {approx:.0} minutes at cruise burn. Consider refueling."
        ))
    }
}

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
    let Some((icao, lat, lon)) = crate::station::active_icao() else {
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
        let wind = crate::weather::surface_wind(&crate::tools::autoland::xplane_root(), &icao);
        crate::station::set_surface_wind(lat, lon, wind);
    });
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod endurance_minutes_tests;
