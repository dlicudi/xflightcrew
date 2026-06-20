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
    pub agl_ft: f64,
    pub vsi_fpm: f64,
    pub engine_running: bool,
    pub alternator_on: bool,
    /// Directional-gyro precession vs the magnetic compass, degrees, folded into
    /// −180..180 (so a small *negative* drift reads as e.g. −4°, not 356°).
    pub gyro_drift_deg: f64,
    /// Minutes of fuel remaining at the current burn (total fuel ÷ fuel flow).
    /// `None` when it can't be computed (no fuel/flow read, or flow ≈ 0).
    pub endurance_min: Option<f64>,
    pub phase: Phase,
}

impl Snapshot {
    /// Read the snapshot from X-Plane. `None` if the sim is unreachable (the
    /// caller just skips the tick and tries again).
    async fn read(xp: &XPlaneClient) -> Option<Snapshot> {
        let on_ground = xp.get_float("sim/flightmodel/failures/onground_any").await.ok()? > 0.5;
        let groundspeed_kts =
            xp.get_float("sim/flightmodel/position/groundspeed").await.ok()? * MS_TO_KTS;
        let agl_ft = xp.get_float("sim/flightmodel/position/y_agl").await.ok()? * M_TO_FT;
        let vsi_fpm = xp
            .get_float("sim/cockpit2/gauges/indicators/vvi_fpm_pilot")
            .await
            .ok()?;
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

        let mut s = Snapshot {
            on_ground,
            groundspeed_kts,
            agl_ft,
            vsi_fpm,
            engine_running,
            alternator_on,
            gyro_drift_deg,
            endurance_min,
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
}

/// Per-check firing state: enforces edge-triggering and Warning repeats.
struct FireState {
    firing: bool,
    last_spoken: Option<Instant>,
}

impl FireState {
    fn new() -> Self {
        FireState { firing: false, last_spoken: None }
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

    /// Condition is currently bad. Speak per [`should_speak`] and mark firing.
    fn active(&mut self, msg: String, sev: Severity, voice: &Voice, now: Instant) {
        if self.should_speak(sev, now) {
            voice.alert(sev.into(), msg);
            self.last_spoken = Some(now);
        }
        self.firing = true;
    }

    /// Condition cleared. Keep `last_spoken` so a quick recurrence is still
    /// inside the cooldown and won't re-nag; only drop the firing flag.
    fn clear(&mut self) {
        self.firing = false;
    }
}

/// Build the checks enabled for this airframe.
fn build_checks(profile: &AircraftProfile) -> Vec<Box<dyn Check>> {
    let mut checks: Vec<Box<dyn Check>> = Vec::new();
    if profile.electrical {
        checks.push(Box::new(AlternatorOffline));
    }
    if profile.directional_gyro {
        checks.push(Box::new(GyroDrift));
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
    }
    checks
}

/// Run the monitor loop until the process exits. Spawned on its own thread +
/// runtime with its own X-Plane client; shares the [`Voice`] arbiter.
pub async fn run(config: Config, profile: AircraftProfile, voice: Voice) {
    let checks = build_checks(&profile);
    if checks.is_empty() {
        tracing::info!("monitor: no checks for this airframe; not starting");
        return;
    }
    tracing::info!(checks = checks.len(), hz = config.monitor_hz, "monitor started");

    let xp = XPlaneClient::new(config.xplane_base_url.clone());
    let period = Duration::from_secs_f32(1.0 / config.monitor_hz.max(0.1));
    let mut states: HashMap<&'static str, FireState> =
        checks.iter().map(|c| (c.id(), FireState::new())).collect();

    loop {
        if let Some(snap) = Snapshot::read(&xp).await {
            let now = Instant::now();
            for check in &checks {
                let state = states.get_mut(check.id()).expect("state per check");
                match check.eval(&snap) {
                    Some(msg) => state.active(msg, check.severity(), &voice, now),
                    None => state.clear(),
                }
            }
        }
        tokio::time::sleep(period).await;
    }
}

/// Engine running but the alternator switch is off → the bus is being drained
/// with nothing charging it. A classic config slip a human copilot catches.
struct AlternatorOffline;

impl Check for AlternatorOffline {
    fn id(&self) -> &'static str {
        "alternator_offline"
    }
    fn severity(&self) -> Severity {
        Severity::Warning
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        (s.engine_running && !s.alternator_on)
            .then(|| "Alternator's offline — we're running on the battery.".to_string())
    }
}

/// The directional gyro has precessed away from the magnetic compass. Vacuum DGs
/// drift slowly, and on a long leg it's easy to forget to realign — a right-seat
/// pilot catches it. Advisory only: the wet compass is still valid, so this is a
/// nudge ("you might want to sync it"), spoken once per occurrence, not a nag.
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
            "Heading indicator's drifted about {approx:.0} degrees — you might want to sync it."
        ))
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
        let in_band = min <= self.at_or_below_min && self.above_min.map_or(true, |a| min > a);
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

#[cfg(test)]
mod tests {
    use super::*;

    fn snap(engine_running: bool, alternator_on: bool) -> Snapshot {
        Snapshot {
            on_ground: true,
            groundspeed_kts: 0.0,
            agl_ft: 0.0,
            vsi_fpm: 0.0,
            engine_running,
            alternator_on,
            gyro_drift_deg: 0.0,
            endurance_min: None,
            phase: Phase::Ground,
        }
    }

    /// The 172's fuel bands: caution ≤ 45 min, warning ≤ 30 min.
    fn fuel_caution() -> FuelEndurance {
        FuelEndurance { id: "fuel_caution", severity: Severity::Caution, at_or_below_min: 45.0, above_min: Some(30.0) }
    }
    fn fuel_warning() -> FuelEndurance {
        FuelEndurance { id: "fuel_warning", severity: Severity::Warning, at_or_below_min: 30.0, above_min: None }
    }

    #[test]
    fn alternator_fires_only_when_running_without_charge() {
        let c = AlternatorOffline;
        assert!(c.eval(&snap(true, false)).is_some()); // running, alt off → alert
        assert!(c.eval(&snap(true, true)).is_none()); // running, alt on → ok
        assert!(c.eval(&snap(false, false)).is_none()); // shut down → ok
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
