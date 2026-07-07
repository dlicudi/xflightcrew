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

use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::aircraft::AircraftProfile;
use crate::config::Config;
use crate::plugin_status::{AlertLevel, StatusSink};
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
    /// Planned top-of-descent distance from the destination, nm.
    pub tod_ring_nm: f64,
}

pub static DESTINATION: std::sync::Mutex<Option<Destination>> = std::sync::Mutex::new(None);

/// Coarse flight phase, estimated from motion and height. Used to arm/disarm
/// checks (e.g. "flaps up on final" only matters on Approach).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[allow(dead_code)] // not every phase is consumed yet (more checks land in M3)
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
    pub vsi_fpm: f64,
    /// Elevator/stabilizer trim, -1 (full nose down) .. +1 (full nose up). Used to
    /// catch trim pinned at a limit (out of authority / possible runaway).
    pub elevator_trim: f64,
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
    /// Total fuel aboard, kg. `None` if the read failed. Used by the ground
    /// check, which estimates endurance from a nominal cruise burn instead of
    /// the (near-zero) idle burn.
    pub fuel_kg: Option<f64>,
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
    /// Descent picture vs the planned destination, when one is known
    /// ([`DESTINATION`]): great-circle distance to the field, its elevation,
    /// and the planned TOD ring. All-or-nothing — `None` without a plan.
    pub dest_dist_nm: Option<f64>,
    pub dest_elev_ft: Option<f64>,
    pub tod_ring_nm: Option<f64>,
    pub phase: Phase,
}

impl Snapshot {
    /// Read the snapshot from X-Plane. `None` if the sim is unreachable (the
    /// caller just skips the tick and tries again). `advisor_drefs` are the
    /// profile's phase-expectation switches, read best-effort per tick.
    async fn read(xp: &XPlaneClient, advisor_drefs: &[String]) -> Option<Snapshot> {
        let on_ground = xp.get_float("sim/flightmodel/failures/onground_any").await.ok()? > 0.5;
        let groundspeed_kts =
            xp.get_float("sim/flightmodel/position/groundspeed").await.ok()? * MS_TO_KTS;
        let ias_kts = xp
            .get_float("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")
            .await
            .ok()?;
        let flap_ratio = xp.get_float("sim/cockpit2/controls/flap_ratio").await.unwrap_or(0.0);
        let agl_ft = xp.get_float("sim/flightmodel/position/y_agl").await.ok()? * M_TO_FT;
        let vsi_fpm = xp
            .get_float("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
            .await
            .ok()?;
        // Best-effort (a missing read must not kill the snapshot): the altitude
        // picture for the bust/sink-rate guards.
        let alt_ft = xp.get_float("sim/flightmodel/misc/h_ind").await.unwrap_or(0.0);
        let ap_alt_dial_ft =
            xp.get_float("sim/cockpit2/autopilot/altitude_dial_ft").await.ok();
        let ap_servos_on =
            xp.get_float("sim/cockpit2/autopilot/servos_on").await.unwrap_or(0.0) > 0.5;
        let gear_down =
            xp.get_float("sim/cockpit/switches/gear_handle_status").await.unwrap_or(1.0) > 0.5;
        let mut advisor_values = Vec::with_capacity(advisor_drefs.len());
        for dref in advisor_drefs {
            advisor_values.push(xp.get_float(dref).await.ok());
        }
        // Position + the descent picture vs the planned destination (if any).
        let lat = xp.get_float("sim/flightmodel/position/latitude").await.unwrap_or(0.0);
        let lon = xp.get_float("sim/flightmodel/position/longitude").await.unwrap_or(0.0);
        let dest = DESTINATION.lock().unwrap_or_else(|e| e.into_inner()).clone();
        let (dest_dist_nm, dest_elev_ft, tod_ring_nm) = match dest {
            Some(d) => (
                Some(crate::haversine_nm(lat, lon, d.lat, d.lon)),
                Some(d.elev_ft),
                Some(d.tod_ring_nm),
            ),
            None => (None, None, None),
        };
        // Best-effort (a missing read must not kill the snapshot): pushback state.
        let tug_attached =
            xp.get_float("sim/aircraft/overflow/pushback_attached").await.unwrap_or(0.0) > 0.5;
        let pushing_back =
            xp.get_float("sim/flightmodel2/misc/tow_in_progress").await.unwrap_or(0.0) > 0.5;
        let engine_running = xp
            .get_float_array("sim/flightmodel/engine/ENGN_running")
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
        let gyro_drift_raw = xp.get_float("sim/cockpit/gyros/dg_drift_vac_deg").await.unwrap_or(0.0);
        let gyro_drift_deg = (gyro_drift_raw + 180.0).rem_euclid(360.0) - 180.0;

        // Fuel endurance = total fuel ÷ current burn. Both best-effort (a missing
        // read must not kill the snapshot); endurance is None when either is
        // absent or the burn is ~0 (idle/shutdown), where "minutes remaining"
        // is meaningless or infinite.
        let fuel_kg = xp.get_float("sim/flightmodel/weight/m_fuel_total").await.ok();
        let fuel_flow_kg_sec = xp
            .get_float_array("sim/cockpit2/engine/indicators/fuel_flow_kg_sec")
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
        let oat_c = xp.get_float("sim/cockpit2/temperature/outside_air_temp_degc").await.unwrap_or(15.0);
        let dewpoint_c = xp
            .get_float_array("sim/weather/aircraft/dewpoint_deg_c")
            .await
            .ok()
            .and_then(engine0)
            .unwrap_or(-30.0);
        let carb_heat_on = xp
            .get_float_array("sim/cockpit2/engine/actuators/carb_heat_ratio")
            .await
            .ok()
            .and_then(engine0)
            .unwrap_or(0.0)
            > 0.5;
        let rpm = xp
            .get_float_array("sim/cockpit2/engine/indicators/prop_speed_rpm")
            .await
            .ok()
            .and_then(engine0)
            .unwrap_or(0.0);
        let elevator_trim = xp.get_float("sim/cockpit2/controls/elevator_trim").await.unwrap_or(0.0);
        let low_vacuum = xp.get_float("sim/cockpit/warnings/annunciators/low_vacuum").await.unwrap_or(0.0) > 0.5;
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
        let fuel_selector = xp.get_float("sim/cockpit2/fuel/fuel_tank_selector").await.ok();

        let mut s = Snapshot {
            on_ground,
            groundspeed_kts,
            ias_kts,
            ias_trend_kts_per_s: 0.0, // filled in by the poll loop (needs history)
            flap_ratio,
            agl_ft,
            vsi_fpm,
            elevator_trim,
            tug_attached,
            pushing_back,
            engine_running,
            alternator_on,
            generator_amps_max,
            gyro_drift_deg,
            endurance_min,
            fuel_kg,
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
            cabin_altitude_ft,
            fuel_selector,
            alt_ft,
            ap_alt_dial_ft,
            ap_servos_on,
            gear_down,
            advisor_values,
            lat,
            lon,
            dest_dist_nm,
            dest_elev_ft,
            tod_ring_nm,
            phase: Phase::Ground,
        };
        s.phase = estimate_phase(&s);
        Some(s)
    }
}

/// Rough flight-phase estimate. Deliberately simple; refined as more checks need
/// finer distinctions.
fn estimate_phase(s: &Snapshot) -> Phase {
    if s.on_ground {
        // Rolling fast enough to be a takeoff/landing roll vs sitting/taxiing.
        if s.groundspeed_kts > 35.0 {
            Phase::Takeoff
        } else {
            Phase::Ground
        }
    } else if s.vsi_fpm > 300.0 {
        Phase::Climb
    } else if s.agl_ft < 1500.0 && s.vsi_fpm < -100.0 {
        Phase::Landing
    } else if s.agl_ft < 3000.0 && s.vsi_fpm < -300.0 {
        Phase::Approach
    } else {
        Phase::Cruise
    }
}

/// Severity of a callout. Maps onto the speech arbiter's priority.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[allow(dead_code)] // Advisory/Caution arrive with the M3 check set
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
}

/// Per-check firing state: enforces edge-triggering and Warning repeats.
struct FireState {
    firing: bool,
    last_spoken: Option<Instant>,
    /// When the condition most recently became true (rising edge). Used to
    /// enforce a check's `min_duration` sustain before the first callout.
    since: Option<Instant>,
}

impl FireState {
    fn new() -> Self {
        FireState { firing: false, last_spoken: None, since: None }
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
                !self.firing || sev == Severity::Warning
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
    fn active(
        &mut self,
        id: &str,
        msg: String,
        sev: Severity,
        min_duration: Duration,
        voice: &Voice,
        status: Option<&StatusSink>,
        now: Instant,
    ) {
        // While the copilot is flying the aircraft (an active recovery/autoland),
        // stay quiet: it announces its own actions, and pilot-directed advice
        // ("lower the nose, add power") is backwards when the copilot is the one
        // doing it. The panel's "mute monitor" setting silences callouts too.
        let flying = crate::tools::recover::ACTIVE.load(std::sync::atomic::Ordering::Relaxed)
            || crate::autoland::ACTIVE.load(std::sync::atomic::Ordering::Relaxed)
            || PANEL_MUTE.load(std::sync::atomic::Ordering::Relaxed);
        if !flying && self.sustained(min_duration, now) && self.should_speak(sev, now) {
            // Mirror the callout to the in-sim panel (monitor speaks via Voice,
            // which otherwise bypasses the panel's Reporter), coloured by severity.
            if let Some(s) = status {
                let level = match sev {
                    Severity::Advisory => AlertLevel::Advisory,
                    Severity::Caution => AlertLevel::Caution,
                    Severity::Warning => AlertLevel::Warning,
                };
                s.alert(level, &msg);
            }
            // Black box: the callout is part of the flight record (the LEAL–LEMD
            // CFIT forensics had to be reconstructed without them), and it's what
            // the post-flight distiller counts.
            crate::blackbox::record(
                "callout",
                serde_json::json!({ "id": id, "sev": format!("{sev:?}"), "msg": &msg }),
            );
            voice.alert(sev.into(), msg);
            self.last_spoken = Some(now);
        }
        self.firing = true;
    }

    /// Condition cleared. Keep `last_spoken` so a quick recurrence is still
    /// inside the cooldown and won't re-nag; drop the firing flag and reset the
    /// sustain timer so the next occurrence must hold for `min_duration` again.
    fn clear(&mut self) {
        self.firing = false;
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
/// below it.
/// Flaps lower the stall speed by up to this fraction at full extension (172:
/// clean Vs ≈ 48, full-flap Vs0 ≈ 40 → ~17%). The threshold scales with flaps so
/// a normal slow-but-safe flaps-down approach doesn't trip it.
const FLAP_STALL_REDUCTION: f64 = 0.17;

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

struct StallSpeed {
    id: &'static str,
    severity: Severity,
    /// Clean stall speed (kts) the bands scale off.
    vs_clean_kts: f64,
    /// Fires when IAS ≤ this × the flap-adjusted stall speed …
    at_or_below_factor: f64,
    /// … and (if set) still above this × it — the more urgent band owns below.
    above_factor: Option<f64>,
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
        let vs = self.vs_clean_kts * (1.0 - FLAP_STALL_REDUCTION * s.flap_ratio.clamp(0.0, 1.0));
        // Anticipate: when the speed is washing off, project it forward so the
        // band trips while there's still margin to recover. Only a *decaying*
        // trend counts (`.min(0.0)`), so this can lower the effective speed but
        // never raises it — it never masks a genuine low reading.
        let projected = s.ias_kts + s.ias_trend_kts_per_s.min(0.0) * STALL_LOOKAHEAD_S;
        let in_band = projected <= self.at_or_below_factor * vs
            && self.above_factor.is_none_or(|a| projected > a * vs);
        if !in_band {
            return None;
        }
        Some(match self.severity {
            Severity::Warning => "Airspeed! Lower the nose, add power.".to_string(),
            _ => "Airspeed — you're getting slow.".to_string(),
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
    /// Never-exceed speed (kts) the bands scale off.
    vne_kts: f64,
    /// Fires when IAS ≥ this × Vne …
    at_or_above_factor: f64,
    /// … and (if set) still below this × Vne — the more urgent band owns above.
    below_factor: Option<f64>,
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
        let in_band = projected >= self.at_or_above_factor * self.vne_kts
            && self.below_factor.is_none_or(|b| projected < b * self.vne_kts);
        if !in_band {
            return None;
        }
        Some(match self.severity {
            Severity::Warning => "Overspeed! Reduce power, ease off the descent.".to_string(),
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
/// Caution — critical in IMC, a cross-check reminder in VMC.
struct LowVacuum;

impl Check for LowVacuum {
    fn id(&self) -> &'static str {
        "low_vacuum"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        (s.engine_running && s.low_vacuum).then(|| {
            "Low vacuum — the attitude and heading indicators may be unreliable. \
             Cross-check the compass and turn coordinator."
                .to_string()
        })
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(3)
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
        let on = (*s.advisor_values.get(self.index)?)? > 0.5;
        (on != self.expect_on).then(|| self.say.clone())
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

/// Descent advisor, part 1: top of descent coming up. The anticipatory version
/// of the "when is TOD" question — the copilot volunteers it while there's
/// still time to act, instead of answering after being asked. Fires once as
/// the aircraft enters the 12-nm run-up to the planned TOD, level at altitude.
struct TodAhead;

impl Check for TodAhead {
    fn id(&self) -> &'static str {
        "tod_ahead"
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || s.vsi_fpm.abs() > 500.0 {
            return None; // only when level — a climb/descent has its own plan
        }
        let (dist, elev, ring) = (s.dest_dist_nm?, s.dest_elev_ft?, s.tod_ring_nm?);
        if s.alt_ft < elev + 6000.0 {
            return None; // already low; TOD math is meaningless
        }
        let to_tod = dist - ring;
        if !(0.0..=12.0).contains(&to_tod) {
            return None;
        }
        let mins = (s.groundspeed_kts > 60.0).then(|| to_tod / s.groundspeed_kts * 60.0);
        Some(match mins {
            Some(m) => format!(
                "Descent coming up. Top of descent in about {m:.0} minutes, {to_tod:.0} miles ahead."
            ),
            None => format!("Descent coming up. Top of descent {to_tod:.0} miles ahead."),
        })
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
        if s.on_ground || s.vsi_fpm < -400.0 {
            return None; // descending → the message is moot
        }
        let (dist, elev, ring) = (s.dest_dist_nm?, s.dest_elev_ft?, s.tod_ring_nm?);
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
        let (dist, elev, ring) = (s.dest_dist_nm?, s.dest_elev_ft?, s.tod_ring_nm?);
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
        let in_band = s.cabin_altitude_ft >= low && high.map_or(true, |h| s.cabin_altitude_ft < h);
        (!s.on_ground && in_band).then(|| self.message.to_string())
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
        (s.on_ground && s.groundspeed_kts > 5.0 && s.park_brake > 0.5)
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
        (s.engine_running && sel < 0.5)
            .then(|| "Fuel selector is off with the engine running — select a tank now.".to_string())
    }
    /// Short sustain so a selector transiting through OFF between tanks doesn't
    /// trip it, but the callout still lands well before the lines run dry.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(3)
    }
}

fn build_checks(profile: &AircraftProfile) -> Vec<Box<dyn Check>> {
    let mut checks: Vec<Box<dyn Check>> = Vec::new();

    // Universal: every airframe can roll with the brake set.
    checks.push(Box::new(RollingOnParkBrake));

    // Universal terrain guard: any airframe can dive into the ground.
    checks.push(Box::new(SinkRate));

    // Descent advisor — armed only when a flight plan gave us a destination
    // (the checks read `Snapshot::dest_*`, `None` without one).
    checks.push(Box::new(TodAhead));
    checks.push(Box::new(TodPassed));
    checks.push(Box::new(HighOnProfile));

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
        message: "Cabin altitude above ten thousand. Check the pressurization or get on oxygen.",
        min_dur: Duration::from_secs(10),
    }));
    checks.push(Box::new(CabinAltitude {
        id: "cabin_alt_warning",
        severity: Severity::Warning,
        band_ft: (12_500.0, None),
        message: "Cabin altitude above twelve thousand five hundred. Oxygen now, or start down.",
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
        checks.push(Box::new(GyroDrift));
        // The vacuum system drives the DG/AI, so an airframe with a gyro has one.
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
            id: "oil_press_low", severity: Severity::Warning, param: EngineParam::OilPressure,
            low_side: true, trip: 20.0, clear: None, min_dur: secs(4),
        }));
        checks.push(Box::new(EngineHealth {
            id: "oil_press_high_caution", severity: Severity::Caution, param: EngineParam::OilPressure,
            low_side: false, trip: 105.0, clear: Some(115.0), min_dur: secs(4),
        }));
        checks.push(Box::new(EngineHealth {
            id: "oil_press_high_warn", severity: Severity::Warning, param: EngineParam::OilPressure,
            low_side: false, trip: 115.0, clear: None, min_dur: secs(3),
        }));
        // Oil temperature (°F): caution band 240–245°F, warning at/over the 245°F redline.
        checks.push(Box::new(EngineHealth {
            id: "oil_temp_caution", severity: Severity::Caution, param: EngineParam::OilTemp,
            low_side: false, trip: 240.0, clear: Some(245.0), min_dur: secs(5),
        }));
        checks.push(Box::new(EngineHealth {
            id: "oil_temp_warn", severity: Severity::Warning, param: EngineParam::OilTemp,
            low_side: false, trip: 245.0, clear: None, min_dur: secs(3),
        }));
        // CHT (°F): caution from ~450°F, warning at the 500°F redline.
        checks.push(Box::new(EngineHealth {
            id: "cht_caution", severity: Severity::Caution, param: EngineParam::Cht,
            low_side: false, trip: 450.0, clear: Some(500.0), min_dur: secs(5),
        }));
        checks.push(Box::new(EngineHealth {
            id: "cht_warn", severity: Severity::Warning, param: EngineParam::Cht,
            low_side: false, trip: 500.0, clear: None, min_dur: secs(3),
        }));
    }
    if let Some(vs) = profile.vs_clean_kts {
        // Wide caution band (≤1.2×Vs) so it speaks with lead time, urgent prompt
        // below ~1.1×Vs. Disjoint bands so only one speaks; the factors are
        // applied to the *flap-adjusted* stall speed at eval time, so a slow but
        // safe flaps-down approach doesn't trip it.
        checks.push(Box::new(StallSpeed {
            id: "airspeed_caution",
            severity: Severity::Caution,
            vs_clean_kts: vs,
            at_or_below_factor: 1.20,
            above_factor: Some(1.10),
        }));
        checks.push(Box::new(StallSpeed {
            id: "airspeed_warning",
            severity: Severity::Warning,
            vs_clean_kts: vs,
            at_or_below_factor: 1.10,
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
            at_or_above_factor: 0.90,
            below_factor: Some(1.0),
        }));
        checks.push(Box::new(Overspeed {
            id: "overspeed_warning",
            severity: Severity::Warning,
            vne_kts: vne,
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
        }));
        checks.push(Box::new(FuelEndurance {
            id: "fuel_warning",
            severity: Severity::Warning,
            at_or_below_min: fuel.reserve_warning_min,
            above_min: None,
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
        let limits: Vec<(f64, f64)> =
            flaps.detents.iter().filter_map(|d| d.vfe_kts.map(|v| (d.ratio, v))).collect();
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
fn r1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
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
        parts.push(format!("FUEL {}h{:02}m", (m / 60.0) as i64, (m % 60.0) as i64));
    }
    parts.join(" | ")
}

pub async fn run(config: Config, live: std::sync::Arc<crate::LiveProfile>, voice: Voice, status: Option<StatusSink>) {
    // Checks are built from the active profile and rebuilt when the aircraft
    // switches (the generation counter changes), so a jet doesn't keep a 172's
    // stall/overspeed limits after a profile swap.
    let advisor_drefs = |p: &AircraftProfile| -> Vec<String> {
        p.advisor.iter().map(|a| a.dataref.clone()).collect()
    };
    let mut gen = live.generation();
    let mut checks = build_checks(&live.get());
    let mut drefs = advisor_drefs(&live.get());
    tracing::info!(checks = checks.len(), hz = config.monitor_hz, "monitor started");
    // Stamp the record with the airframe so the distiller can key memory by it.
    crate::blackbox::record("flight_meta", serde_json::json!({ "aircraft": live.get().id }));

    let xp = XPlaneClient::new(config.xplane_base_url.clone());
    let period = Duration::from_secs_f32(1.0 / config.monitor_hz.max(0.1));
    let mut states: HashMap<&'static str, FireState> =
        checks.iter().map(|c| (c.id(), FireState::new())).collect();

    // IAS history for the rate-of-decay term. `trend` is an EMA (kts/s) so a
    // single noisy sample doesn't read as a sharp washout.
    let mut prev_ias: Option<(f64, Instant)> = None;
    let mut trend = 0.0_f64;
    // Pushback progress is a sequence of *events*, not a sustained condition, so
    // it's called out on transitions here rather than as Checks.
    let mut prev_attached = false;
    let mut prev_pushing = false;
    // Last strip pushed to the panel — only re-send on change.
    let mut last_strip = String::new();

    // Long-term memory: brief from the carry-forward on startup, and remember
    // the flight after shutdown. `was_airborne` arms the debrief on liftoff.
    let memory = crate::debrief::memory_dir();
    if let Some(brief) = crate::debrief::CarryForward::briefing_once_today(&memory, &live.get().id) {
        voice.alert(Priority::Advisory, brief);
    }
    let mut was_airborne = false;
    // Talk-down pacing: when the last callout went out, and whether we've
    // already handed over at the threshold (say it once, then shut up).
    let mut guide_last: Option<Instant> = None;
    let mut guide_handed_over = false;

    loop {
        // Aircraft switched → rebuild the checks for the new airframe.
        if live.generation() != gen {
            gen = live.generation();
            checks = build_checks(&live.get());
            drefs = advisor_drefs(&live.get());
            states = checks.iter().map(|c| (c.id(), FireState::new())).collect();
            tracing::info!(checks = checks.len(), "monitor: reconfigured for new aircraft");
            crate::blackbox::record("flight_meta", serde_json::json!({ "aircraft": live.get().id }));
            if let Some(brief) = crate::debrief::CarryForward::briefing_once_today(&memory, &live.get().id) {
                voice.alert(Priority::Advisory, brief);
            }
        }
        if let Some(mut snap) = Snapshot::read(&xp, &drefs).await {
            let now = Instant::now();

            // Pushback callouts (on the ground): tug hooks up → pushing → clear.
            if snap.on_ground {
                let say = |line: &str| {
                    if let Some(s) = status.as_ref() {
                        s.alert(AlertLevel::Advisory, line);
                    }
                    voice.alert(Priority::Advisory, line.to_string());
                };
                if snap.tug_attached && !prev_attached && !snap.pushing_back {
                    say("Tug connected, ready for pushback.");
                }
                if snap.pushing_back && !prev_pushing {
                    say("Pushing back.");
                } else if !snap.pushing_back && prev_pushing {
                    say("Pushback complete, tug clear.");
                }
            }
            prev_attached = snap.tug_attached;
            prev_pushing = snap.pushing_back;

            // Flight ended (was airborne, now parked with the engines off) →
            // distill the black box into memory and speak the debrief. Once per
            // flight: the arm resets only on the next liftoff.
            if !snap.on_ground {
                was_airborne = true;
            } else if was_airborne && !snap.engine_running && snap.groundspeed_kts < 5.0 {
                was_airborne = false;
                let mem = memory.clone();
                let done = tokio::task::spawn_blocking(move || {
                    crate::debrief::remember(std::path::Path::new("logs/blackbox.jsonl"), &mem)
                })
                .await;
                if let Ok(Some(summary)) = done {
                    tracing::info!(aircraft = %summary.aircraft, "flight distilled to memory");
                    if let Some(s) = status.as_ref() {
                        s.alert(AlertLevel::Advisory, &summary.spoken());
                    }
                    voice.alert(Priority::Advisory, summary.spoken());
                }
            }

            // Talk-down: PAR-style callouts down the synthesized approach path,
            // armed by the visual_approach tool. Advisory narration only — the
            // pilot flies. Muted while the copilot itself is flying (recover /
            // autoland) or the panel mute is on, same as every other callout.
            if let Some(path) = crate::approach_guide::armed() {
                let quiet = crate::tools::recover::ACTIVE.load(std::sync::atomic::Ordering::Relaxed)
                    || crate::autoland::ACTIVE.load(std::sync::atomic::Ordering::Relaxed)
                    || PANEL_MUTE.load(std::sync::atomic::Ordering::Relaxed);
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
                    let d = crate::approach_guide::deviations(&path, snap.lat, snap.lon, snap.alt_ft);
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
                                if let Some(s) = status.as_ref() {
                                    s.alert(AlertLevel::Advisory, &line);
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

            // Black box: one state record per tick (position/altitude come from
            // the snapshot; only the heading needs an extra read).
            let (lat, lon, alt) = (snap.lat, snap.lon, snap.alt_ft);
            let hdg = xp.get_float("sim/flightmodel/position/psi").await.unwrap_or(0.0);
            crate::blackbox::record("state", serde_json::json!({
                "lat": lat, "lon": lon, "hdg": r1(hdg), "alt": r1(alt),
                "ias": r1(snap.ias_kts), "gs": r1(snap.groundspeed_kts),
                "vsi": r1(snap.vsi_fpm), "agl": r1(snap.agl_ft),
                "ground": snap.on_ground, "flap": snap.flap_ratio,
                "park": r1(snap.park_brake), "cabin": r1(snap.cabin_altitude_ft),
                "fuel_kg": snap.fuel_kg, "eng": snap.engine_running,
                "phase": format!("{:?}", snap.phase),
            }));

            // Panel settings + the live flight strip. The mute switch comes from
            // the plugin's settings popup; unresolvable (no plugin) = unmuted.
            let mute = xp.get_float("xcopilot/opt/monitor_mute").await.map_or(false, |v| v > 0.5);
            PANEL_MUTE.store(mute, std::sync::atomic::Ordering::Relaxed);
            if let Some(s) = status.as_ref() {
                let strip = compose_strip(&snap);
                if strip != last_strip {
                    s.strip(&strip);
                    last_strip = strip;
                }
            }

            for check in &checks {
                let state = states.get_mut(check.id()).expect("state per check");
                match check.eval(&snap) {
                    Some(msg) => state.active(check.id(), msg, check.severity(), check.min_duration(), &voice, status.as_ref(), now),
                    None => state.clear(),
                }
            }
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
/// drift slowly, and on a long leg it's easy to forget to realign — a right-seat
/// pilot catches it. Advisory only: the wet compass is still valid, so this is a
/// flat advisory ("recommend syncing the gyro"), spoken once per occurrence, not a nag.
/// Airborne-only — on the ground you just align during the run-up.
struct GyroDrift;

impl Check for GyroDrift {
    fn id(&self) -> &'static str {
        "gyro_drift"
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || s.gyro_drift_deg.abs() < GYRO_DRIFT_ADVISORY_DEG {
            return None;
        }
        // Round to the nearest 5° so the spoken figure sounds like an estimate
        // ("about ten degrees"), not false precision.
        let approx = (s.gyro_drift_deg.abs() / 5.0).round() * 5.0;
        Some(format!(
            "Heading indicator's drifted about {approx:.0} degrees — recommend syncing the gyro."
        ))
    }
}

/// Flaps extended above their speed limit (Vfe) — the per-detent limits from the
/// profile. The applicable limit is the lowest Vfe of any detent at or below the
/// current flap position (Vfe drops as the flaps extend further). Airborne only.
struct FlapOverspeed {
    /// (detent ratio, Vfe knots) for detents that carry a limit.
    limits: Vec<(f64, f64)>,
}
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
        // Lowest Vfe among detents we've reached (ratio <= current, small tolerance).
        let vfe = self
            .limits
            .iter()
            .filter(|(r, _)| *r <= s.flap_ratio + 0.05)
            .map(|(_, v)| *v)
            .fold(f64::INFINITY, f64::min);
        if vfe.is_finite() && s.ias_kts > vfe + 5.0 {
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
        let dir = if s.elevator_trim < 0.0 { "nose down" } else { "nose up" };
        Some(format!("Stabilizer trim's at the {dir} limit, check the trim."))
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
        let min = s.endurance_min?;
        let in_band = min <= self.at_or_below_min && self.above_min.is_none_or(|a| min > a);
        if !in_band {
            return None;
        }
        // Round to the nearest 5 minutes — it's an estimate, not a stopwatch.
        let approx = (min / 5.0).round() * 5.0;
        Some(match self.severity {
            Severity::Warning => {
                format!("Low fuel — about {approx:.0} minutes at this burn. Plan to land.")
            }
            _ => format!("Fuel's getting low — about {approx:.0} minutes remaining."),
        })
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
            "Low fuel for departure — about {approx:.0} minutes at cruise burn. Consider refueling."
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snap(engine_running: bool, alternator_on: bool) -> Snapshot {
        Snapshot {
            on_ground: true,
            groundspeed_kts: 0.0,
            ias_kts: 0.0,
            ias_trend_kts_per_s: 0.0,
            flap_ratio: 0.0,
            agl_ft: 0.0,
            vsi_fpm: 0.0,
            elevator_trim: 0.0,
            tug_attached: false,
            pushing_back: false,
            engine_running,
            alternator_on,
            generator_amps_max: 0.0,
            gyro_drift_deg: 0.0,
            endurance_min: None,
            fuel_kg: None,
            oil_pressure_psi: None,
            oil_temp_f: None,
            cht_f: None,
            oat_c: 15.0,
            dewpoint_c: -30.0,
            carb_heat_on: false,
            rpm: 0.0,
            low_vacuum: false,
            battery_amps: 0.0,
            park_brake: 0.0,
            cabin_altitude_ft: 0.0,
            fuel_selector: None,
            alt_ft: 0.0,
            ap_alt_dial_ft: None,
            ap_servos_on: false,
            gear_down: false,
            advisor_values: vec![],
            lat: 0.0,
            lon: 0.0,
            dest_dist_nm: None,
            dest_elev_ft: None,
            tod_ring_nm: None,
            phase: Phase::Ground,
        }
    }

    /// Airborne at cruise with a known destination — the descent-advisor base.
    /// Roughly today's LEAL–LEMD picture: 14,000 ft, Madrid at ~2,000 ft,
    /// planned TOD ring 42 nm out.
    fn cruise_with_dest(dist_nm: f64) -> Snapshot {
        let mut s = snap(true, true);
        s.on_ground = false;
        s.alt_ft = 14_000.0;
        s.vsi_fpm = 0.0;
        s.groundspeed_kts = 300.0;
        s.ias_kts = 240.0;
        s.dest_dist_nm = Some(dist_nm);
        s.dest_elev_ft = Some(2_000.0);
        s.tod_ring_nm = Some(42.0);
        s
    }

    #[test]
    fn tod_ahead_speaks_in_the_runup_band() {
        let c = TodAhead;
        assert!(c.eval(&cruise_with_dest(80.0)).is_none(), "far out → quiet");
        let msg = c.eval(&cruise_with_dest(50.0)).expect("8 nm before TOD → advise");
        assert!(msg.contains("Top of descent"), "{msg}");
        assert!(c.eval(&cruise_with_dest(40.0)).is_none(), "past TOD → this check yields");
        // No destination known → silent.
        let mut s = cruise_with_dest(50.0);
        s.dest_dist_nm = None;
        assert!(c.eval(&s).is_none());
    }

    #[test]
    fn tod_passed_names_the_recovery_rate() {
        let c = TodPassed;
        assert!(c.eval(&cruise_with_dest(45.0)).is_none(), "before TOD → quiet");
        // Today's miss: level at 14,000, 38 nm out, TOD at 42. Height to lose
        // 10,000 ft over (38-5)/300kt = 6.6 min → ~1,500 fpm.
        let msg = c.eval(&cruise_with_dest(38.0)).expect("past TOD and level → caution");
        assert!(msg.contains("1500 feet a minute"), "{msg}");
        // Already descending → moot, quiet.
        let mut s = cruise_with_dest(38.0);
        s.vsi_fpm = -2000.0;
        assert!(c.eval(&s).is_none());
    }

    #[test]
    fn high_on_profile_advises_mid_descent() {
        let c = HighOnProfile;
        // On the slope: 30 nm out, profile ≈ 2000+2000+22*300 = 10,600.
        let mut s = cruise_with_dest(30.0);
        s.vsi_fpm = -1800.0;
        s.alt_ft = 10_600.0;
        assert!(c.eval(&s).is_none(), "on profile → quiet");
        // 3,400 ft above it, fast → boards call.
        s.alt_ft = 14_000.0;
        s.ias_kts = 258.0;
        let msg = c.eval(&s).expect("well above the slope");
        assert!(msg.contains("Boards"), "{msg}");
        // Level flight is TodPassed's problem, not this check's.
        s.vsi_fpm = 0.0;
        assert!(c.eval(&s).is_none());
    }

    #[test]
    fn phase_expectation_advises_on_the_delta() {
        let c = PhaseExpectation {
            id: "advisor_test",
            index: 0,
            expect_on: true,
            phases: vec![Phase::Climb, Phase::Cruise],
            say: "Yaw damper's still off.".into(),
        };
        let mut s = snap(true, true);
        // Wrong phase → silent even when the switch is off.
        s.phase = Phase::Ground;
        s.advisor_values = vec![Some(0.0)];
        assert!(c.eval(&s).is_none());
        // Right phase, switch off → advise.
        s.phase = Phase::Cruise;
        assert_eq!(c.eval(&s).as_deref(), Some("Yaw damper's still off."));
        // Switch on → satisfied.
        s.advisor_values = vec![Some(1.0)];
        assert!(c.eval(&s).is_none());
        // Failed read → silent, never advise off stale air.
        s.advisor_values = vec![None];
        assert!(c.eval(&s).is_none());
    }

    #[test]
    fn advisor_rows_build_from_the_profile() {
        let p = crate::aircraft::load("md82").expect("md82 profile");
        assert!(!p.advisor.is_empty(), "md82 declares an SOP table");
        let checks = build_checks(&p);
        let advisors = checks.iter().filter(|c| c.id().starts_with("advisor_")).count();
        assert_eq!(advisors, p.advisor.len());
    }

    #[test]
    fn sink_rate_scales_with_height() {
        let c = SinkRate;
        let mut s = snap(true, true);
        s.on_ground = false;
        // Normal 700-fpm final at 400 AGL: silent.
        s.agl_ft = 400.0;
        s.vsi_fpm = -700.0;
        assert!(c.eval(&s).is_none());
        // The LEAL-LEMD dive: ~1,960 fpm at 187 AGL → fires.
        s.agl_ft = 187.0;
        s.vsi_fpm = -1958.0;
        assert!(c.eval(&s).is_some());
        // Same rate up high is a normal descent.
        s.agl_ft = 3000.0;
        assert!(c.eval(&s).is_none());
        s.on_ground = true;
        s.agl_ft = 0.0;
        assert!(c.eval(&s).is_none(), "on the ground → silent");
    }

    #[test]
    fn altitude_bust_fires_through_the_dial_only() {
        let c = AltitudeBust;
        let mut s = snap(true, true);
        s.on_ground = false;
        s.ap_servos_on = true;
        s.ap_alt_dial_ft = Some(5000.0);
        // Descending toward the dial from above: normal, silent.
        s.alt_ft = 8000.0;
        s.vsi_fpm = -2000.0;
        assert!(c.eval(&s).is_none());
        // Blown through it: fires.
        s.alt_ft = 4500.0;
        assert!(c.eval(&s).is_some());
        // Landing configuration (gear down = dial holds the missed-approach
        // altitude while descending below it): silent.
        s.gear_down = true;
        assert!(c.eval(&s).is_none());
        s.gear_down = false;
        // Hand-flying (servos off): the AP isn't going to capture anything.
        s.ap_servos_on = false;
        assert!(c.eval(&s).is_none());
        s.ap_servos_on = true;
        // Climbing through the dial fires too.
        s.alt_ft = 5600.0;
        s.vsi_fpm = 1500.0;
        assert!(c.eval(&s).is_some());
    }

    #[test]
    fn fuel_selector_off_gating() {
        let c = FuelSelectorOff;
        let mut s = snap(true, true);
        assert!(c.eval(&s).is_none(), "no reading → silent");
        s.fuel_selector = Some(0.0);
        assert!(c.eval(&s).is_some(), "selector OFF, engine running → warn");
        s.fuel_selector = Some(1.0);
        assert!(c.eval(&s).is_none(), "tank selected → silent");
        s.fuel_selector = Some(0.0);
        s.engine_running = false;
        assert!(c.eval(&s).is_none(), "engine off (parked) → silent");
    }

    #[test]
    fn not_charging_trusts_a_producing_generator() {
        let c = AlternatorNotCharging { noun: "Generator" };
        let mut s = snap(true, true);
        s.battery_amps = -7.0;
        assert!(c.eval(&s).is_some(), "discharge, nothing producing → warn");
        // MD-82 case: standing battery-bus discharge while both generators charge.
        s.generator_amps_max = 39.0;
        assert!(c.eval(&s).is_none(), "a producing generator overrules the ammeter");
        s.generator_amps_max = 0.0;
        s.engine_running = false;
        assert!(c.eval(&s).is_none(), "engine off → silent");
    }

    #[test]
    fn engine_health_gating_and_disjoint_bands() {
        let z = Duration::ZERO;
        // Oil pressure (PSI): warn below the 20 PSI redline; idle lows (e.g. 40)
        // are NOT alarmed (that idle nuisance was the original false positive).
        let low = EngineHealth { id: "l", severity: Severity::Warning, param: EngineParam::OilPressure, low_side: true, trip: 20.0, clear: None, min_dur: z };

        let mut s = snap(true, true);
        assert!(low.eval(&s).is_none(), "no reading → silent");
        s.oil_pressure_psi = Some(40.0);
        assert!(low.eval(&s).is_none(), "idle-low 40 PSI is fine, no alarm");
        s.oil_pressure_psi = Some(15.0);
        assert!(low.eval(&s).is_some(), "below 20 PSI redline → warn");
        s.engine_running = false;
        assert!(low.eval(&s).is_none(), "engine off suppresses");

        // Oil-temp bands are disjoint (°F thresholds): caution 240–245, warn >245.
        let caution = EngineHealth { id: "c", severity: Severity::Caution, param: EngineParam::OilTemp, low_side: false, trip: 240.0, clear: Some(245.0), min_dur: z };
        let warn = EngineHealth { id: "w", severity: Severity::Warning, param: EngineParam::OilTemp, low_side: false, trip: 245.0, clear: None, min_dur: z };
        let mut s = snap(true, true);
        s.oil_temp_f = Some(205.0);
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none(), "205°F cruise → silent");
        s.oil_temp_f = Some(242.0);
        assert!(caution.eval(&s).is_some() && warn.eval(&s).is_none(), "caution band only");
        s.oil_temp_f = Some(250.0);
        assert!(warn.eval(&s).is_some() && caution.eval(&s).is_none(), "warning band only");
    }

    #[test]
    fn carb_ice_advisory_conditions() {
        let c = CarbIce;
        let mut s = snap(true, true);
        s.on_ground = false;
        s.oat_c = 10.0;
        s.dewpoint_c = 5.0; // spread 5 → humid
        s.rpm = 1500.0; // throttled back
        assert!(c.eval(&s).is_some(), "ripe conditions → advise carb heat");
        s.carb_heat_on = true;
        assert!(c.eval(&s).is_none(), "carb heat already on → silent");
        s.carb_heat_on = false;
        s.dewpoint_c = -30.0; // dry
        assert!(c.eval(&s).is_none(), "dry air → silent");
        s.dewpoint_c = 5.0;
        s.rpm = 2400.0; // cruise power
        assert!(c.eval(&s).is_none(), "high power → silent");
    }

    #[test]
    fn low_vacuum_and_alternator_ammeter() {
        let v = LowVacuum;
        let mut s = snap(true, true);
        assert!(v.eval(&s).is_none(), "vacuum ok → silent");
        s.low_vacuum = true;
        assert!(v.eval(&s).is_some(), "low vacuum → warn unreliable gyros");
        s.engine_running = false;
        assert!(v.eval(&s).is_none(), "engine off → no vacuum check");

        let a = AlternatorNotCharging { noun: "Alternator" };
        let mut s = snap(true, true); // alternator switch on
        s.battery_amps = 8.0; // charging
        assert!(a.eval(&s).is_none(), "charging → silent");
        s.battery_amps = -10.0; // discharging with alternator on
        assert!(a.eval(&s).is_some(), "not charging → caution");
        let mut s = snap(true, false); // alternator off — AlternatorOffline's job
        s.battery_amps = -10.0;
        assert!(a.eval(&s).is_none(), "switch off → handled elsewhere");
    }

    #[test]
    fn airspeed_bands_and_flare_and_ground_suppression() {
        // Vs=48 clean → caution ≤57.6 (and >52.8), warning ≤52.8.
        let caution = StallSpeed { id: "c", severity: Severity::Caution, vs_clean_kts: 48.0, at_or_below_factor: 1.20, above_factor: Some(1.10) };
        let warn = StallSpeed { id: "w", severity: Severity::Warning, vs_clean_kts: 48.0, at_or_below_factor: 1.10, above_factor: None };
        let mut s = snap(true, true);
        s.on_ground = false;
        s.agl_ft = 500.0; // airborne, above the flare

        s.ias_kts = 65.0; // normal approach — silent
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
        s.ias_kts = 55.0; // getting slow — caution only
        assert!(caution.eval(&s).is_some() && warn.eval(&s).is_none());
        s.ias_kts = 50.0; // near stall — warning only (bands disjoint)
        assert!(warn.eval(&s).is_some() && caution.eval(&s).is_none());

        // Rate term: a fast washout trips the caution while the speed is still
        // well above the line. 67 kt is silent steady, but decaying at 3 kt/s it
        // projects to 67 − 3×4.0 = 55 → caution.
        s.ias_kts = 67.0;
        s.ias_trend_kts_per_s = 0.0;
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
        s.ias_trend_kts_per_s = -3.0;
        assert!(caution.eval(&s).is_some() && warn.eval(&s).is_none());
        // Same 67 kt accelerating away (recovering) stays silent — projection
        // only ever lowers the effective speed, never raises it.
        s.ias_trend_kts_per_s = 3.0;
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
        s.ias_trend_kts_per_s = 0.0;

        // Flaps lower the stall speed, so the same 55 kt is now safe and silent;
        // the band only fires once you're slow *for that configuration*.
        s.flap_ratio = 1.0; // full flaps → Vs ≈ 39.8, caution ≤47.8
        s.ias_kts = 55.0;
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
        s.ias_kts = 46.0; // slow for full flaps — caution
        assert!(caution.eval(&s).is_some() && warn.eval(&s).is_none());
        s.flap_ratio = 0.0;

        // Slow but in the flare → suppressed (slow-and-low is intended there).
        s.ias_kts = 50.0;
        s.agl_ft = 20.0;
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
        // On the ground → suppressed.
        s.agl_ft = 500.0;
        s.on_ground = true;
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
    }

    /// The 172's fuel bands: caution ≤ 45 min, warning ≤ 30 min.
    fn fuel_caution() -> FuelEndurance {
        FuelEndurance { id: "fuel_caution", severity: Severity::Caution, at_or_below_min: 45.0, above_min: Some(30.0) }
    }
    fn fuel_warning() -> FuelEndurance {
        FuelEndurance { id: "fuel_warning", severity: Severity::Warning, at_or_below_min: 30.0, above_min: None }
    }

    #[test]
    fn rolling_on_park_brake_fires_only_when_moving_on_ground() {
        let c = RollingOnParkBrake;
        let mut s = snap(true, true);
        // Parked with the brake set (a run-up): silent.
        s.park_brake = 1.0;
        s.groundspeed_kts = 0.0;
        assert!(c.eval(&s).is_none(), "stationary is fine");
        // Rolling with it set: caution.
        s.groundspeed_kts = 20.0;
        assert!(c.eval(&s).is_some(), "rolling with the brake must fire");
        // Rolling with it released: silent.
        s.park_brake = 0.0;
        assert!(c.eval(&s).is_none());
        // Airborne (stale ratio): silent.
        s.park_brake = 1.0;
        s.on_ground = false;
        assert!(c.eval(&s).is_none(), "not a thing in the air");
    }

    #[test]
    fn cabin_altitude_bands_are_disjoint_and_airborne_only() {
        let caution = CabinAltitude {
            id: "c", severity: Severity::Caution, band_ft: (10_000.0, Some(12_500.0)),
            message: "caution", min_dur: Duration::from_secs(10),
        };
        let warning = CabinAltitude {
            id: "w", severity: Severity::Warning, band_ft: (12_500.0, None),
            message: "warning", min_dur: Duration::from_secs(4),
        };
        let mut s = snap(true, true);
        s.on_ground = false;
        // Below 10k: nothing.
        s.cabin_altitude_ft = 8_000.0;
        assert!(caution.eval(&s).is_none() && warning.eval(&s).is_none());
        // 10k–12.5k: caution only (disjoint bands).
        s.cabin_altitude_ft = 11_000.0;
        assert!(caution.eval(&s).is_some() && warning.eval(&s).is_none());
        // Above 12.5k: warning only.
        s.cabin_altitude_ft = 14_000.0;
        assert!(caution.eval(&s).is_none() && warning.eval(&s).is_some());
        // On the ground it never fires, however high the reading.
        s.on_ground = true;
        assert!(caution.eval(&s).is_none() && warning.eval(&s).is_none());
    }

    #[test]
    fn alternator_fires_only_when_running_without_charge() {
        let c = AlternatorOffline { noun: "Alternator" };
        assert!(c.eval(&snap(true, false)).is_some()); // running, alt off → alert
        assert!(c.eval(&snap(true, true)).is_none()); // running, alt on → ok
        assert!(c.eval(&snap(false, false)).is_none()); // shut down → ok
        assert_eq!(c.min_duration(), ALTERNATOR_SUSTAIN); // sustained before warning
    }

    #[test]
    fn sustain_suppresses_transient_then_fires_when_held() {
        let t0 = Instant::now();
        let d = ALTERNATOR_SUSTAIN;
        let mut st = FireState::new();

        // The start transient: alternator offline for a few seconds, then the
        // ALT switch goes on and the condition clears — must never reach speak.
        assert!(!st.sustained(d, t0)); // rising edge: timer starts, not yet due
        assert!(!st.sustained(d, t0 + Duration::from_secs(3)));
        st.clear(); // alternator came online

        // A genuinely forgotten alternator: held continuously past the sustain.
        let t1 = t0 + Duration::from_secs(10);
        assert!(!st.sustained(d, t1)); // new rising edge — times from here
        assert!(!st.sustained(d, t1 + d - Duration::from_secs(1))); // still short
        assert!(st.sustained(d, t1 + d)); // held a full minute → now warn
    }

    #[test]
    fn fuel_bands_are_disjoint_and_airborne_only() {
        let caution = fuel_caution();
        let warning = fuel_warning();
        let mut s = snap(true, true);
        s.on_ground = false;

        // Plenty of fuel: nothing fires.
        s.endurance_min = Some(90.0);
        assert!(caution.eval(&s).is_none() && warning.eval(&s).is_none());

        // Caution band (30 < min ≤ 45): only caution.
        s.endurance_min = Some(40.0);
        assert!(caution.eval(&s).is_some());
        assert!(warning.eval(&s).is_none());

        // Warning band (≤ 30): only warning, not caution (disjoint handoff).
        s.endurance_min = Some(25.0);
        assert!(caution.eval(&s).is_none());
        assert!(warning.eval(&s).is_some());

        // On the ground: silent even when low (idle burn makes endurance moot).
        s.on_ground = true;
        assert!(caution.eval(&s).is_none() && warning.eval(&s).is_none());

        // Engine off, or endurance uncomputable: silent.
        s.on_ground = false;
        s.engine_running = false;
        assert!(warning.eval(&s).is_none());
        s.engine_running = true;
        s.endurance_min = None;
        assert!(warning.eval(&s).is_none());
    }

    #[test]
    fn fuel_pretakeoff_cautions_on_ground_only_when_low() {
        // 45 min caution; 24 kg/hr cruise → 18 kg ≈ 45 min, 14 kg ≈ 35 min.
        let c = FuelPreTakeoff { caution_min: 45.0, cruise_burn_kg_hr: 24.0 };
        let mut s = snap(true, true); // on ground, engine running

        // Plenty for a flight: silent.
        s.fuel_kg = Some(40.0); // 100 min at cruise
        assert!(c.eval(&s).is_none());

        // Low for departure: cautions.
        s.fuel_kg = Some(14.0); // ~35 min at cruise
        assert!(c.eval(&s).is_some());

        // Airborne: this check is ground-only (FuelEndurance owns the air).
        s.on_ground = false;
        assert!(c.eval(&s).is_none());

        // Engine off, or no fuel read: silent.
        s.on_ground = true;
        s.engine_running = false;
        assert!(c.eval(&s).is_none());
        s.engine_running = true;
        s.fuel_kg = None;
        assert!(c.eval(&s).is_none());
    }

    #[test]
    fn gyro_drift_fires_only_airborne_past_threshold() {
        let c = GyroDrift;
        let mut s = snap(true, true);

        // On the ground: never fires, even with big drift (sync at run-up).
        s.on_ground = true;
        s.gyro_drift_deg = 20.0;
        assert!(c.eval(&s).is_none());

        // Airborne, small precession: under threshold, stays quiet.
        s.on_ground = false;
        s.gyro_drift_deg = 6.0;
        assert!(c.eval(&s).is_none());

        // Airborne, drifted past threshold: fires (either sign).
        s.gyro_drift_deg = 12.0;
        assert!(c.eval(&s).is_some());
        s.gyro_drift_deg = -14.0;
        assert!(c.eval(&s).is_some());
    }

    #[test]
    fn gyro_drift_callout_rounds_to_nearest_five() {
        let c = GyroDrift;
        let mut s = snap(true, true);
        s.on_ground = false;
        s.gyro_drift_deg = -13.0; // |13| → "about 15 degrees"
        let msg = c.eval(&s).expect("should fire");
        assert!(msg.contains("15 degrees"), "got: {msg}");
    }

    #[test]
    fn warning_speaks_on_edge_then_repeats() {
        let mut st = FireState::new();
        let t0 = Instant::now();
        assert!(st.should_speak(Severity::Warning, t0)); // rising edge
        st.firing = true;
        st.last_spoken = Some(t0);
        assert!(!st.should_speak(Severity::Warning, t0 + Duration::from_secs(1))); // too soon
        assert!(st.should_speak(Severity::Warning, t0 + RENOTIFY_COOLDOWN + Duration::from_secs(1))); // due
    }

    #[test]
    fn advisory_speaks_once_and_does_not_repeat() {
        let mut st = FireState::new();
        let t0 = Instant::now();
        assert!(st.should_speak(Severity::Advisory, t0)); // rising edge
        st.firing = true;
        st.last_spoken = Some(t0);
        // Even long after, a still-firing Advisory doesn't nag.
        assert!(!st.should_speak(Severity::Advisory, t0 + RENOTIFY_COOLDOWN * 3));
    }

    #[test]
    fn fiddling_within_cooldown_does_not_renag() {
        let mut st = FireState::new();
        let t0 = Instant::now();
        // First occurrence speaks.
        assert!(st.should_speak(Severity::Warning, t0));
        st.firing = true;
        st.last_spoken = Some(t0);
        // Pilot fixes it.
        st.clear();
        assert!(!st.firing);
        // Recurs almost immediately → suppressed (still within cooldown).
        assert!(!st.should_speak(Severity::Warning, t0 + Duration::from_secs(3)));
        // Recurs well after the cooldown → speaks again.
        assert!(st.should_speak(Severity::Warning, t0 + RENOTIFY_COOLDOWN + Duration::from_secs(1)));
    }

    #[test]
    fn phase_estimate_basic() {
        let mut s = snap(true, true);
        s.on_ground = true;
        s.groundspeed_kts = 0.0;
        assert_eq!(estimate_phase(&s), Phase::Ground);
        s.groundspeed_kts = 60.0;
        assert_eq!(estimate_phase(&s), Phase::Takeoff);
        s.on_ground = false;
        s.groundspeed_kts = 0.0;
        s.vsi_fpm = 700.0;
        assert_eq!(estimate_phase(&s), Phase::Climb);
    }
}
