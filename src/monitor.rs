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
    pub phase: Phase,
}

impl Snapshot {
    /// Read the snapshot from X-Plane. `None` if the sim is unreachable (the
    /// caller just skips the tick and tries again).
    async fn read(xp: &XPlaneClient) -> Option<Snapshot> {
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

fn build_checks(profile: &AircraftProfile) -> Vec<Box<dyn Check>> {
    let mut checks: Vec<Box<dyn Check>> = Vec::new();

    // Universal: every airframe can roll with the brake set.
    checks.push(Box::new(RollingOnParkBrake));

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
    let mut gen = live.generation();
    let mut checks = build_checks(&live.get());
    tracing::info!(checks = checks.len(), hz = config.monitor_hz, "monitor started");

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

    loop {
        // Aircraft switched → rebuild the checks for the new airframe.
        if live.generation() != gen {
            gen = live.generation();
            checks = build_checks(&live.get());
            states = checks.iter().map(|c| (c.id(), FireState::new())).collect();
            tracing::info!(checks = checks.len(), "monitor: reconfigured for new aircraft");
        }
        if let Some(mut snap) = Snapshot::read(&xp).await {
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
                    Some(msg) => state.active(msg, check.severity(), check.min_duration(), &voice, status.as_ref(), now),
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
            phase: Phase::Ground,
        }
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
