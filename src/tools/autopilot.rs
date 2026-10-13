//! `set_heading` — command the autopilot to fly a magnetic heading.
//!
//! "Autopilot heading 270" is three actions: dial the heading bug (a dataref
//! write), engage the servos, and engage HDG mode (two commands). The readback
//! re-reads the bug and the servo state and confirms what actually happened.
//!
//! Datarefs/commands are standard X-Plane (confirmed against the rustdecks
//! profiles), so the tool owns them; the aircraft profile only declares that an
//! autopilot exists.


use crate::autoflight::{self, Button, Dial};
use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
#[cfg(test)]
use super::CMD_SERVOS_ON;
use crate::voice::{Priority, Voice};
use crate::aircraft::throttle_dataref;
use crate::xplane::XPlaneClient;

/// Pilot-side heading bug, degrees magnetic (writable).
#[cfg(test)]
const HEADING_BUG: &str = Dial::HeadingBug.read_name();
/// Selected altitude, feet (writable).
#[cfg(test)]
const ALT_DIAL: &str = Dial::Altitude.read_name();
/// Selected vertical speed, feet per minute (writable). Use the classic
/// `vertical_velocity` dial, not cockpit2's `vvi_dial_fpm`: on the SR22's
/// G1000/Perspective AP the latter isn't the dataref the autopilot follows —
/// verified against the rustdecks SR22 deck, which reads/writes VS here.
#[cfg(test)]
const VS_DIAL: &str = Dial::Vs.read_name();
/// Servo engagement status (read-only; 1 = autopilot flying).
const SERVOS_ON: &str = autoflight::names::SERVOS_ON;
/// Command: disconnect the autopilot. The stock 172's S-TEC-style AP has no
/// master switch and does not register `sim/autopilot/servos_off`. X-Plane
/// exposes `servos_off_any` ("disco servos, whichever side is active") and the
/// yoke red button `ap_disc_trim_interrupt`. We use `servos_off_any` — a clean
/// servos disconnect — rather than the yoke button, which also interrupts the
/// pitch-trim servo (an unwanted side effect for a voice "autopilot off").
/// Command: engage heading-select (HDG) mode.
#[cfg(test)]
const CMD_HEADING_MODE: &str = autoflight::cmd::HEADING;
/// GPSS (LNAV) engaged status.
const GPSS_STATUS: &str = autoflight::names::GPSS_STATUS;
/// Command: select GPS as the CDI/HSI source (so NAV follows the GPS flight plan).
/// NAV-mode status (> 0 = armed/engaged).
const NAV_STATUS: &str = autoflight::names::NAV_STATUS;
/// Command: engage altitude select/hold (ALT) mode.
#[cfg(test)]
const CMD_ALT_HOLD: &str = autoflight::cmd::ALTITUDE_HOLD;
/// Command: wings-level / roll hold — for "auto level" / "straight and level".
/// Current indicated altitude (feet), read to capture "hold this altitude".
const CURRENT_ALT: &str = crate::datarefs::ALTITUDE_FT;
/// Autopilot altitude targets below this (feet MSL) are refused — a 172's AP is
/// never commanded this low, so it's almost always a misheard/empty value (e.g.
/// "auto level" misread as altitude 0). The gate catches it before the AP flies
/// the aircraft into the ground.
const MIN_AP_ALTITUDE_FT: f64 = 500.0;
/// Command: engage vertical-speed (VS) mode.
#[cfg(test)]
const CMD_VS_MODE: &str = autoflight::cmd::VERTICAL_SPEED;
/// Throttle (all engines), 0..1. A commanded climb sets this to full — the AP
/// only controls pitch, so without power a 172 trades airspeed for altitude.
/// Throttle restored once a climb captures its target / on level-off (~cruise
/// power for a 172). Not a precise %, just "off the climb stop" so the aircraft
/// settles at the altitude instead of accelerating at full power.
const CRUISE_THROTTLE: f64 = 0.65;
/// The capture watcher only backs off the throttle *it* firewalled to full. If
/// the pilot has since taken manual throttle (set_throttle writes the same
/// dataref), it will read below this floor — and the watcher stands down rather
/// than clobbering the pilot's setting back to cruise. The firewall sets 1.0 and
/// cruise is 0.65, so a value below 0.9 means a human moved it.
const FIREWALL_HANDBACK_FLOOR: f64 = 0.9;
/// Mode status datarefs (0 = off, >0 = armed/engaged). Read before pressing a
/// mode command, because the `sim/autopilot/*` mode commands are TOGGLES: firing
/// one while its mode is already active switches it back OFF.
const HEADING_STATUS: &str = autoflight::names::HDG_STATUS;
const ALT_HOLD_STATUS: &str = autoflight::names::ALT_HOLD_STATUS;
const VS_STATUS: &str = autoflight::names::VS_STATUS;
/// Descents steeper than this (fpm) are confirmed — anything shallower, and all
/// climbs, execute immediately. A 172 descends ~500–1000 fpm routinely.
const VS_CONFIRM_DESCENT_FPM: f64 = -1500.0;
/// A set_climb rate magnitude above this (fpm) is implausible for a 172 (it climbs
/// ~700, descends ~1000) and likely a mishear, so it still confirms; normal,
/// sensible rates execute directly — the unsafe-target gate is the real safety net.
const CLIMB_CONFIRM_RATE_FPM: f64 = 1500.0;

/// Per-airframe autopilot engagement prerequisites (from the profile's
/// `autopilot_prep`). Empty/`false` for the GA singles, which just need servos-on.
#[derive(Clone)]
pub struct ApConfig {
    /// Commands to raise the flight director before engaging the servos.
    pub fd_commands: Vec<String>,
    /// The AP is AC-powered (needs an inverter) — drives the honest failure hint.
    pub ac_powered: bool,
    /// Service ceiling (ft MSL) — altitude targets above it are refused. From the
    /// airframe's reference figures; falls back to the GA default.
    pub service_ceiling_ft: f64,
    /// The airframe's climb numbers for a pilot's "climb to X".
    pub climb: ClimbPerf,
}

impl Default for ApConfig {
    fn default() -> Self {
        Self {
            fd_commands: Vec::new(),
            ac_powered: false,
            service_ceiling_ft: DEFAULT_SERVICE_CEILING_FT,
            climb: ClimbPerf::default(),
        }
    }
}

/// The airframe's own numbers for a pilot-commanded climb (refactor audit
/// 2026-10-04, finding #3). The tool flew a Cessna's climb on every
/// airframe: 700 fpm, eased only under a flat 60 kt and handed back to a
/// flat 0.65 lever, so on a 737 or an A330 (clean stall 139 to 175 kt) the
/// protection never fired, which is the #181 failure (166 kt to 73 at 9,600
/// fpm) by another door. The defaults are the old GA figures, for an
/// airframe that declares nothing.
#[derive(Debug, Clone, PartialEq)]
pub struct ClimbPerf {
    /// The most an auto-scaled climb asks, fpm: the profile's
    /// `enroute_climb_fpm` (the follower's rate), else the GA 700.
    pub max_climb_fpm: f64,
    /// The speed the climb is flown at and gets its full rate at, kt: Vy.
    pub vy_kts: Option<f64>,
    /// The speed the climb fades to nothing at, kt: the circuit's climb
    /// floor (`aircraft::climb_floor`: blue line, else the 1.2 Vs
    /// automation floor). 0 = undeclared.
    pub floor_kts: f64,
    /// The speed a level-off hands the governor, kt: the profile's cruise.
    pub cruise_ias_kts: Option<f64>,
}

impl Default for ClimbPerf {
    fn default() -> Self {
        Self {
            max_climb_fpm: MAX_CLIMB_FPM,
            vy_kts: None,
            floor_kts: 0.0,
            cruise_ias_kts: None,
        }
    }
}

impl ClimbPerf {
    /// From the profile: the same numbers the circuit and the follower fly.
    pub fn from_profile(p: &crate::aircraft::AircraftProfile) -> Self {
        let r = p.reference.as_ref();
        Self {
            max_climb_fpm: r
                .and_then(|r| r.enroute_climb_fpm)
                .filter(|f| f.is_finite() && *f > 0.0)
                .unwrap_or(MAX_CLIMB_FPM),
            vy_kts: r.and_then(|r| r.vy_kts).filter(|v| v.is_finite() && *v > 0.0),
            floor_kts: crate::aircraft::climb_floor(p),
            cruise_ias_kts: r
                .and_then(|r| r.cruise_ias_kts)
                .filter(|v| v.is_finite() && *v > 0.0),
        }
    }

    /// The floor the climb protection fades to: this airframe's, the live
    /// automation floor (`aircraft::speed_floor_kts`) when it is higher,
    /// and only with neither known the old GA 60 kt.
    fn floor(&self, live_floor: Option<f64>) -> f64 {
        let f = self.floor_kts.max(live_floor.unwrap_or(0.0));
        if f > 0.0 {
            f
        } else {
            CLIMB_MIN_KTS
        }
    }
}

/// The vertical speed a pilot's climb is flown at this tick: the circuit's
/// own climb limiter (`aircraft::climb_limit_fpm`, a projected airspeed
/// fading the rate from the climb speed to the floor) over the commanded
/// rate, rounded to the 100 fpm a VS wheel steps in. The full-rate speed is
/// Vy, capped by the governor's target when one holds the speed
/// (`aircraft::climb_full_rate_kts`); with no Vy it is the floor plus the old
/// 8 kt restore band. A 172 (floor 58, Vy 74) climbs at full rate down to
/// Vy and at none at 58; it used to hold the rate to 60. Pure.
fn protected_climb_fpm(
    commanded_fpm: f64,
    ias_kts: f64,
    ias_trend_kts_s: f64,
    perf: &ClimbPerf,
    live_floor: Option<f64>,
    governor_kts: Option<f64>,
) -> f64 {
    if commanded_fpm <= 0.0 {
        return commanded_fpm;
    }
    let floor = perf.floor(live_floor);
    let vy = perf.vy_kts.unwrap_or(floor + CLIMB_RESTORE_BAND_KTS);
    let full =
        crate::aircraft::climb_full_rate_kts(governor_kts.unwrap_or(f64::INFINITY), vy, floor);
    let fpm =
        crate::aircraft::climb_limit_fpm(commanded_fpm, ias_kts, ias_trend_kts_s, full, floor);
    ((fpm / VS_TRIM_FPM).round() * VS_TRIM_FPM).clamp(0.0, commanded_fpm)
}

/// What a level-off does with the power. Pure; the watcher and the level-off
/// tool act on it.
#[derive(Debug, Clone, Copy, PartialEq)]
enum LevelPower {
    /// The governor already holds the speed: it owns the throttle.
    Governor,
    /// Hand the speed to the governor at the airframe's cruise speed (the
    /// follower's and the takeoff handoff's way of settling).
    ArmCruise(f64),
    /// An airframe declaring no cruise speed: the old GA lever.
    Lever(f64),
}

/// The level-off's power: never a flat 172 lever on an airframe that
/// declares its cruise. The cruise speed is held under 250 kt below 10,000
/// and over the automation floor (`speedhold::hold_for_pilot` floors it).
/// Pure.
fn level_power(governor_kts: Option<f64>, cruise_ias_kts: Option<f64>, alt_ft: f64) -> LevelPower {
    if governor_kts.is_some() {
        return LevelPower::Governor;
    }
    match cruise_ias_kts {
        Some(kts) => LevelPower::ArmCruise(crate::tools::speedhold::airspace_limited(alt_ft, kts)),
        None => LevelPower::Lever(CRUISE_THROTTLE),
    }
}

/// Standard X-Plane inverter status (array; any element > 0 = AC power available).
const INVERTER_ON: &str = "sim/cockpit2/electrical/inverter_on";

/// Command: turn the heading bug one degree (knob presses).
#[cfg(test)]
const CMD_HDG_UP: &str = autoflight::cmd::HEADING_UP;

// The heading knob is the panel's owner's (`autoflight::drive_heading_bug`).
pub use crate::autoflight::{drive_heading_bug, drive_heading_bug_step};

/// Read the servo status as a bool ("is the autopilot actually flying").
async fn servos_engaged(ctx: &ToolCtx<'_>) -> Result<bool> {
    Ok(crate::autoflight::read_status(&*ctx.xplane, SERVOS_ON).await? > 0.5)
}

/// Raise the flight director (if this airframe needs it) then engage the servos.
/// The C90's `fd_monitor` drops the servos unless the FD switch is up, so the FD
/// commands must fire first.
async fn engage_ap(ctx: &ToolCtx<'_>, cfg: &ApConfig) -> Result<()> {
    for c in &cfg.fd_commands {
        ctx.xplane.run_command(c).await?;
    }
    autoflight::press(ctx.xplane, Button::Engage).await?;
    Ok(())
}

/// The honest reason the servos didn't latch, for the failure readback. On an
/// AC-powered AP with no inverter on, that's the real cause; otherwise fall back to
/// the generic AP-master check.
async fn no_engage_reason(ctx: &ToolCtx<'_>, cfg: &ApConfig) -> &'static str {
    if cfg.ac_powered {
        let any_inverter = ctx
            .xplane
            .get_float_array(INVERTER_ON)
            .await
            .map(|v| v.iter().any(|&x| x > 0.5))
            .unwrap_or(false);
        if !any_inverter {
            return "the autopilot has no AC power, turn the inverter on";
        }
    }
    "check the AP master"
}

/// Read a mode's 0/1-ish status dataref ("is this mode armed/engaged").
async fn mode_engaged(ctx: &ToolCtx<'_>, status: &str) -> bool {
    crate::autoflight::read_status(ctx.xplane, status).await.unwrap_or(0.0) > 0.5
}

/// Engage an autopilot mode *idempotently*. The `sim/autopilot/*` mode commands
/// are toggles, so pressing one whose mode is already active turns it back OFF —
/// which is why a second "set vertical speed" (or heading, or altitude) in a row
/// would silently drop the mode. Check the status first; only press when off.
async fn engage_mode(ctx: &ToolCtx<'_>, status: &str, button: Button) -> Result<()> {
    if !mode_engaged(ctx, status).await {
        autoflight::press(ctx.xplane, button).await?;
    }
    Ok(())
}

/// Default climb/descent rate (fpm) for an altitude change with no rate given.
/// Gentle on purpose: a 172 climbs ~500-700 fpm flat-out, and the copilot isn't
/// managing power, so a high commanded VS just bleeds airspeed. 300 is safe for
/// both climb and descent; say an explicit rate ("climb to X at 700") for more.
const DEFAULT_VS_FPM: f64 = 300.0;
/// Max climb rate (fpm) the 172 sustains at full power without bleeding airspeed —
/// the cap when auto-scaling the rate to the altitude change, for an airframe
/// that declares no `enroute_climb_fpm` ([`ClimbPerf::max_climb_fpm`]).
const MAX_CLIMB_FPM: f64 = 700.0;
/// Max auto descent rate (fpm) commanded for an altitude change — gentle enough
/// not to build speed toward Vne without active power management. (Per-airframe.)
const MAX_DESCENT_FPM: f64 = 1000.0;
/// The climb protection's floor (kt) for an airframe that declares neither a
/// clean stall nor a blue line ([`ClimbPerf::floor`]): ~12 kt above the 172's
/// clean stall, the figure every airframe was held to before.
const CLIMB_MIN_KTS: f64 = 60.0;
/// With no Vy declared, the climb gets its full rate this far over the floor
/// (the old restore threshold, 60 + 8).
const CLIMB_RESTORE_BAND_KTS: f64 = 8.0;
/// The step the climb protection's VS command moves in, fpm.
const VS_TRIM_FPM: f64 = 100.0;
/// Default service ceiling (ft MSL) when the airframe declares none — the 172's.
/// Real airframes override it via their reference `service_ceiling_ft` (the C90B is
/// 28,100), threaded in through [`ApConfig`].
const DEFAULT_SERVICE_CEILING_FT: f64 = 14_000.0;
/// Minimum height above the terrain below (ft) an altitude target may be. Replaces
/// a fixed MSL floor, which is meaningless at a high-elevation airport.
const MIN_TARGET_AGL_FT: f64 = 500.0;
/// Feet per meter — `y_agl` is metres.
const METERS_TO_FEET: f64 = 3.280_84;
/// Within this many feet of the target, treat it as "hold here" (no climb), and
/// the band the capture watcher uses to detect a pilot re-command (dial moved).
const ALT_CAPTURE_BAND_FT: f64 = 100.0;
/// The watcher levels off once the aircraft *reaches* the target in its direction
/// of travel (climb: at/above; descent: at/below), minus this small lead so the
/// roll-out settles on the target rather than the band edge. Reliable because the
/// watcher polls fast near the target (see `capture_at_altitude`); tune against the
/// "capture" log line if the AP consistently settles above/below the target.
const CAPTURE_LEAD_FT: f64 = 0.0;
/// Inside this distance from the target, the watcher polls fast so the level-off
/// isn't skipped between ticks even at a brisk rate.
const CAPTURE_APPROACH_FT: f64 = 400.0;

/// Vertical speed (fpm, magnitude) for an altitude change of `delta_ft`: scaled so
/// any change captures in roughly three minutes, floored at the gentle default so
/// small steps aren't sluggish, and capped at the airframe's sustainable rate for
/// the direction so a commanded climb never outruns the available power.
fn auto_vs_fpm(delta_ft: f64, climb: bool, max_climb_fpm: f64) -> f64 {
    let cap = if climb {
        max_climb_fpm.max(DEFAULT_VS_FPM)
    } else {
        MAX_DESCENT_FPM
    };
    (delta_ft.abs() / 3.0).clamp(DEFAULT_VS_FPM, cap)
}

/// The lowest safe target altitude (ft MSL): the terrain below plus a margin.
/// `agl_m` is height above ground in metres, `current_msl` the current altitude.
fn terrain_floor_ft(current_msl: f64, agl_m: f64) -> f64 {
    let ground = current_msl - agl_m * METERS_TO_FEET;
    ground + MIN_TARGET_AGL_FT
}

/// Reject an altitude target that's above the ceiling or into the terrain. Returns
/// `Some(refusal)` to speak back, or `None` if the target is safe to fly to. Fails
/// safe: if the terrain read is unavailable, falls back to the fixed MSL floor.
async fn reject_unsafe_target(ctx: &ToolCtx<'_>, target: f64, ceiling: f64) -> Option<String> {
    if target > ceiling {
        return Some(format!(
            "Negative — {target:.0} feet is above this aircraft's service ceiling. \
             Say something at or below {ceiling:.0}."
        ));
    }
    // The ground AHEAD, when a follower is flying and probing it. The
    // under-the-wheels test alone cleared 4,000 ft over ground at 1,700 with a
    // 3,861 ft ridge five miles on the nose (2026-09-13): a target is only
    // safe against terrain it is about to fly over, not terrain already passed.
    if let Some(floor) = crate::terrain::floor_ahead_ft() {
        if target < floor {
            let ground = floor - crate::terrain::CLEARANCE_FT;
            return Some(format!(
                "Negative, {target:.0} feet is into the terrain ahead; the high ground on our \
                 track is about {ground:.0} feet. Say {floor:.0} or higher."
            ));
        }
    }
    let agl = ctx.xplane.get_float(crate::datarefs::AGL_METERS).await.ok();
    let current = ctx.xplane.get_float(CURRENT_ALT).await.ok();
    match (agl, current) {
        (Some(agl_m), Some(current_msl)) => {
            let floor = terrain_floor_ft(current_msl, agl_m);
            if target < floor {
                let ground = current_msl - agl_m * METERS_TO_FEET;
                return Some(format!(
                    "Negative — {target:.0} feet is into the terrain; the ground here is about \
                     {ground:.0} feet. Say a higher altitude, or 'level off' to hold this one."
                ));
            }
            None
        }
        // Terrain unknown — fall back to the fixed floor so we never fly blind low.
        _ if target < MIN_AP_ALTITUDE_FT => Some(format!(
            "Negative — {target:.0} feet is too low for an altitude hold. Say a target like \
             five thousand, or 'level off' to hold your current altitude."
        )),
        _ => None,
    }
}

/// A CLIMB commanded to an altitude that is still into the ground ahead is
/// flown to the terrain floor instead of refused: the pilot asking to go up is
/// already reacting to the terrain, and "negative" leaves the aircraft exactly
/// where it was. Returns the floor to fly when the target must be raised
/// (`None` for a descent, which [`reject_unsafe_target`] refuses, or with no
/// terrain answer). The same standing rule as the speed floor: raise the
/// number, say so, log it.
async fn terrain_raised_target(ctx: &ToolCtx<'_>, target: f64) -> Option<f64> {
    let floor = crate::terrain::floor_ahead_ft()?;
    if target >= floor {
        return None;
    }
    let current = ctx.xplane.get_float(CURRENT_ALT).await.ok()?;
    (target > current + ALT_CAPTURE_BAND_FT).then_some((floor / 100.0).ceil() * 100.0)
}

/// Fly the autopilot to `altitude` at `rate` fpm and let it capture/level off.
/// On this airframe ALT hold alone won't *change* altitude — you must arm the
/// target AND command a vertical speed, so this sets both. Direction is inferred
/// from current altitude. Returns ("Climbing to" | "Descending to" | "Holding at",
/// commanded_fpm_magnitude, servos_engaged).
/// `rate` is the magnitude in fpm; `None` auto-scales it to the altitude change.
/// The fourth element says whether the climb throttle was firewalled, so the
/// readback only promises full power when full power was actually commanded.
async fn fly_to(
    ctx: &ToolCtx<'_>,
    altitude: f64,
    rate: Option<f64>,
    cfg: &ApConfig,
) -> Result<(&'static str, f64, bool, bool)> {
    // A procedure follower rewrites the altitude dial EVERY tick, so writing it
    // here would be undone within 500 ms, which is exactly what happened when
    // a pilot twice ordered a climb away from rising ground and watched the
    // aircraft stay on the plan's 2,700 ft (2026-09-13). Hand the follower the
    // number instead: it stays the one writer, its VS law flies the change, and
    // the pilot's altitude outranks the plan's until they say another one.
    if crate::pilot_altitude::take(altitude) {
        let current = ctx.xplane.get_float(CURRENT_ALT).await.unwrap_or(altitude);
        let diff = altitude - current;
        let movement = if diff.abs() <= ALT_CAPTURE_BAND_FT {
            "Holding at"
        } else if diff > 0.0 {
            "Climbing to"
        } else {
            "Descending to"
        };
        // The follower's own clamp is the rate that will be flown, whatever was
        // asked for; and the governor owns the throttle, so nothing firewalls.
        let fpm = rate
            .map(f64::abs)
            .unwrap_or_else(|| auto_vs_fpm(diff, diff > 0.0, cfg.climb.max_climb_fpm))
            .min(crate::pilot_altitude::follower_rate_fpm());
        tracing::info!(
            target: "xvoicelayer::autopilot",
            movement,
            target_alt = altitude,
            "handed the altitude to the follower"
        );
        return Ok((movement, fpm, servos_engaged(ctx).await?, false));
    }
    // Raise the flight director first (some airframes' AP won't hold without it);
    // arm_altitude then engages the servos.
    for c in &cfg.fd_commands {
        ctx.xplane.run_command(c).await?;
    }
    let governor = crate::tools::speedhold::flight_target();
    let (movement, signed, capture, firewalled) =
        arm_altitude(ctx, altitude, rate, cfg.climb.max_climb_fpm, governor).await?;
    // A climb/descent needs the watcher to level us off (and settle a climb's
    // power); a hold is already done. The watcher calls out the level-off when
    // it reaches the target, so hand it a voice clone.
    if let Some(was_climb) = capture {
        capture_at_altitude(
            ctx.config.xplane_base_url.clone(),
            altitude,
            was_climb,
            ctx.voice.cloned(),
            ctx.status.cloned(),
            cfg.climb.clone(),
        );
    }

    // Diagnostic: dump what the AP actually latched so a flight test shows the
    // truth (did VS engage? is the dial holding our rate? did ALT hold sneak on?).
    let vs_dial = autoflight::read_dial(ctx.xplane, Dial::Vs).await.unwrap_or(f64::NAN);
    let vs_mode = mode_engaged(ctx, VS_STATUS).await;
    let alt_hold_mode = mode_engaged(ctx, ALT_HOLD_STATUS).await;
    tracing::info!(
        target: "xvoicelayer::autopilot",
        movement,
        commanded_fpm = signed,
        vs_dial,
        vs_mode,
        alt_hold_mode,
        target_alt = altitude,
        "fly_to latched"
    );
    Ok((movement, signed.abs(), servos_engaged(ctx).await?, firewalled))
}

/// The synchronous part of [`fly_to`]: read the current altitude, infer the
/// direction, and arm the autopilot (altitude window + VS mode + rate, plus the
/// climb power firewall). Returns the movement phrase, the *signed* commanded fpm,
/// `Some(was_climb)` if a capture watcher should be spawned (climb/descent) or
/// `None` for a hold, and whether the throttle was firewalled. Split out from the
/// threaded watcher so the trust-critical writes are unit-testable against a fake
/// sim. `governor` is the speed governor's target when it holds the speed: it
/// owns the throttle, so the climb leaves it alone (one writer per dataref).
async fn arm_altitude(
    ctx: &ToolCtx<'_>,
    altitude: f64,
    rate: Option<f64>,
    max_climb_fpm: f64,
    governor: Option<f64>,
) -> Result<(&'static str, f64, Option<bool>, bool)> {
    let current = ctx.xplane.get_float(CURRENT_ALT).await.unwrap_or(altitude);
    let diff = altitude - current;
    autoflight::press(ctx.xplane, Button::Engage).await?;

    if diff.abs() <= ALT_CAPTURE_BAND_FT {
        // HOLD: we're already at it. Capture the current altitude now — this is the
        // one case where `altitude_hold` is right (hold where you are).
        let hold = (current / 100.0).round() * 100.0;
        engage_mode(ctx, ALT_HOLD_STATUS, Button::AltitudeHold).await?;
        autoflight::set_dial(ctx.xplane, Dial::Altitude, hold).await?;
        return Ok(("Holding at", 0.0, None, false));
    }

    // CLIMB or DESCENT. Arm the target in the altitude window and fly to it in VS
    // mode — VS is the only *active* pitch mode. Do NOT engage `altitude_hold` now:
    // at the current altitude it would hold here and cancel the climb. Set the VS
    // dial LAST — engaging VS snaps the dial to the live rate, so the target must be
    // written after. The stock 172 AP does NOT reliably capture the preselected
    // altitude from VS mode, so the watcher levels us off when we reach it.
    let climb = diff > 0.0;
    // Explicit rate (set_climb) is used as given; an absent rate (set_altitude)
    // scales to the change so a big climb isn't a 30-minute crawl at 300 fpm.
    let mag = rate
        .map(f64::abs)
        .unwrap_or_else(|| auto_vs_fpm(diff, climb, max_climb_fpm));
    let signed = if climb { mag } else { -mag };
    autoflight::set_dial(ctx.xplane, Dial::Altitude, altitude).await?;
    engage_mode(ctx, VS_STATUS, Button::VerticalSpeed).await?;
    autoflight::set_dial(ctx.xplane, Dial::Vs, signed).await?;
    // A climb needs power — the AP pitches up to hold VS, and on cruise power a 172
    // just bleeds airspeed. Full throttle for the climb; the watcher settles it at
    // capture. A running speed governor already owns the throttle and adds the
    // power the climb asks for, so it is left to it.
    let firewall = climb && governor.is_none();
    if firewall {
        let _ = ctx.xplane.set_float(&throttle_dataref(), 1.0).await;
    }
    Ok((
        if climb {
            "Climbing to"
        } else {
            "Descending to"
        },
        signed,
        Some(climb),
        firewall,
    ))
}

/// What the capture watcher should do on a tick, given the selected-altitude dial
/// it's watching, its `target`, and the current altitude. Pure, so the capture
/// logic is testable without the polling thread.
#[derive(Debug, PartialEq, Eq)]
enum Capture {
    /// The dial moved off our target — the pilot re-commanded; stop watching.
    StandDown,
    /// Not there yet — keep watching.
    Wait,
    /// Within the band — level off (engage ALT hold) and finish.
    LevelOff,
}

fn capture_action(dial: f64, target: f64, current_alt: f64, climb: bool) -> Capture {
    if (dial - target).abs() > ALT_CAPTURE_BAND_FT {
        return Capture::StandDown; // dial moved off our target — pilot re-commanded
    }
    // Level off once we've arrived at the target in the direction of travel (with a
    // small lead), so we settle on it rather than at the band edge. Directional so
    // a climb doesn't trip on the way up to a higher re-command, etc.
    let reached = if climb {
        current_alt >= target - CAPTURE_LEAD_FT
    } else {
        current_alt <= target + CAPTURE_LEAD_FT
    };
    if reached {
        Capture::LevelOff
    } else {
        Capture::Wait
    }
}

/// The capture watcher's run. Each climb/descent takes it over: the watcher
/// it replaces is cancelled (its client refuses every write) and stands down
/// at its next look, so at most one is ever live no matter how many altitude
/// changes are commanded in quick succession (the `ALT_DIAL` stand-down only
/// catches a *different* target; two commands to the same altitude, or a
/// re-command, would otherwise stack independent watchers). It writes the
/// altitude dial, fires ALT hold and sets the throttle for up to 20 minutes,
/// so a swap, a new flight or instructor entry cancels it
/// (`flight_state::LOOPS`).
pub(crate) static CAPTURE: sim::run::Slot = sim::run::Slot::new();

/// Fly the AP to a selected altitude in VS mode, then **level off** there.
///
/// The stock 172 AP does not reliably capture the preselected altitude from VS
/// mode on its own, so the copilot does it: a one-shot watcher polls until the
/// aircraft reaches `target`, then engages ALT hold (idempotently — a no-op if the
/// AP already captured) so it holds instead of flying straight through. For a
/// `was_climb` it also settles the firewalled throttle ([`level_power`]) and,
/// on the way up, protects the airspeed with the airframe's climb limiter
/// ([`protected_climb_fpm`], `perf`).
///
/// Own thread + runtime + client (like the dashboard poller): the session runs a
/// current-thread runtime blocked on the audio listener between commands, so a
/// `tokio::spawn` task would be starved — the watcher must poll independently.
/// Self-terminating and guarded: it stands down without touching anything if a
/// newer command supersedes it (generation), if the selector dial moves off
/// `target` (pilot re-commanded), and it won't override a throttle the pilot has
/// taken manually. Fire-and-forget; a dropped poll just retries next tick.
fn capture_at_altitude(
    base: String,
    target: f64,
    was_climb: bool,
    voice: Option<Voice>,
    status: Option<crate::plugin_status::StatusSink>,
    perf: ClimbPerf,
) {
    // Taken on the caller's thread: a stand-down landing before the thread
    // starts cancels THIS watcher, not a predecessor.
    let lease = CAPTURE.take_over();
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async move {
            // Every write is scoped to this watcher's run: once superseded or
            // stood down, a write still in flight is refused.
            let xp = XPlaneClient::new(base.clone()).with_cancel(lease.cancel.clone());
            let mut last_ias: Option<(std::time::Instant, f64)> = None;
            let mut ias_trend = 0.0_f64;
            // Wall-clock budget: even a long, slow change finishes inside 20 min; if
            // it somehow doesn't, we stop watching.
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20 * 60);
            // The commanded climb rate — the ceiling the airspeed protection won't
            // exceed when it restores the climb after backing off.
            let initial_vs = autoflight::read_dial(&xp, Dial::Vs).await.unwrap_or(0.0);
            let mut eased_for_speed = false;
            while std::time::Instant::now() < deadline {
                // A newer command was issued (it owns the level-off now), or a
                // stand-down landed.
                if !lease.live() {
                    return;
                }
                // Need both the selected dial (did the pilot re-command?) and the
                // current altitude (are we there yet?) to decide; a dropped read just
                // retries next tick.
                let (Ok(dial), Ok(alt)) = (
                    autoflight::read_dial(&xp, Dial::Altitude).await,
                    xp.get_float(CURRENT_ALT).await,
                ) else {
                    tokio::time::sleep(std::time::Duration::from_secs(3)).await;
                    continue;
                };
                // The reads awaited: a command or a stand-down may have
                // landed meanwhile, and the level-off writes are not ours.
                if !lease.live() {
                    return;
                }
                match capture_action(dial, target, alt, was_climb) {
                    Capture::StandDown => return,
                    Capture::LevelOff => {
                        // Diagnostic: where did we engage vs the target? If the AP
                        // settles consistently off-target, tune CAPTURE_LEAD_FT from
                        // (alt_at_capture - target) and the settled altitude.
                        tracing::info!(
                            target: "xvoicelayer::autopilot",
                            target_alt = target,
                            alt_at_capture = alt,
                            was_climb,
                            "capture: levelling off"
                        );
                        // Re-affirm the exact target in the window, then engage ALT
                        // hold so the AP holds here instead of flying through in VS.
                        // Idempotent (the command is a toggle): if the AP already
                        // captured on its own, leave it engaged.
                        let _ = autoflight::set_dial(&xp, Dial::Altitude, target).await;
                        let held = crate::autoflight::read_status(&xp, ALT_HOLD_STATUS)
                            .await
                            .map(|v| v > 0.5)
                            .unwrap_or(false);
                        if !held {
                            let _ = autoflight::press(&xp, Button::AltitudeHold).await;
                        }
                        // Settle a firewalled climb's power, but only if the
                        // throttle is still where we left it (full). If the pilot took
                        // manual throttle mid-climb (which doesn't move the alt dial,
                        // so the stand-down never fired), it now reads below the floor —
                        // leave it rather than overriding their setting. A running
                        // governor owns the throttle; else the airframe's cruise speed
                        // is handed to it, and only an airframe declaring none gets
                        // the old GA lever (`level_power`).
                        let mut settled = None;
                        if was_climb {
                            let governor = crate::tools::speedhold::flight_target();
                            match level_power(governor, perf.cruise_ias_kts, alt) {
                                LevelPower::Governor => {}
                                power => match xp.get_float(&throttle_dataref()).await {
                                    Ok(thr) if thr >= FIREWALL_HANDBACK_FLOOR => {
                                        match power {
                                            LevelPower::ArmCruise(kts) => {
                                                crate::tools::speedhold::hold_for_pilot(
                                                    base.clone(),
                                                    voice.clone(),
                                                    kts,
                                                );
                                            }
                                            LevelPower::Lever(ratio) => {
                                                let _ =
                                                    xp.set_float(&throttle_dataref(), ratio).await;
                                            }
                                            LevelPower::Governor => {}
                                        }
                                        settled = Some(power);
                                    }
                                    _ => {} // pilot took manual throttle: don't touch it
                                },
                            }
                            crate::blackbox::record(
                                "autopilot",
                                serde_json::json!({"event": "climb_level_power",
                                    "target": target, "alt_ft": alt,
                                    "governor": governor,
                                    "settled": format!("{settled:?}")}),
                            );
                        }
                        // Call out the capture. This lands minutes after the command,
                        // so it's the copilot telling you it actually levelled off.
                        // Spoken via Voice and mirrored to the panel (which the
                        // Reporter never sees for this background announcement).
                        let line = match settled {
                            Some(LevelPower::ArmCruise(kts)) => format!(
                                "Reached {target:.0} feet, levelling off, holding {kts:.0} knots."
                            ),
                            Some(LevelPower::Lever(_)) => format!(
                                "Reached {target:.0} feet, easing the power back, levelling off."
                            ),
                            _ => format!("Reached {target:.0} feet, levelling off."),
                        };
                        if let Some(s) = &status {
                            s.action(&line);
                        }
                        if let Some(v) = &voice {
                            v.say(Priority::Readback, line);
                        }
                        return;
                    }
                    Capture::Wait => {
                        // Airspeed-protected climb: the circuit's climb limiter
                        // over the airframe's own climb speed and floor
                        // (`protected_climb_fpm`), never above what was
                        // commanded. The dial is written only when the command
                        // moves (a knob is set, not rewritten).
                        if was_climb && initial_vs > 0.0 {
                            if let Ok(ias) = xp.get_float(crate::datarefs::AIRSPEED_KTS).await {
                                let now = std::time::Instant::now();
                                if let Some((t0, ias0)) = last_ias {
                                    let dt = now.duration_since(t0).as_secs_f64();
                                    if dt > 0.2 {
                                        let raw = (ias - ias0) / dt;
                                        ias_trend += (raw - ias_trend) * 0.5;
                                    }
                                }
                                last_ias = Some((now, ias));
                                let want = protected_climb_fpm(
                                    initial_vs,
                                    ias,
                                    ias_trend,
                                    &perf,
                                    crate::aircraft::speed_floor_kts(),
                                    crate::tools::speedhold::flight_target(),
                                );
                                let cur = autoflight::read_dial(&xp, Dial::Vs).await.unwrap_or(initial_vs);
                                if (want - cur).abs() >= VS_TRIM_FPM - 1.0 {
                                    let _ = autoflight::set_dial(&xp, Dial::Vs, want).await;
                                    crate::blackbox::record(
                                        "autopilot",
                                        serde_json::json!({"event": "climb_protect",
                                            "ias": ias, "trend": ias_trend,
                                            "was_fpm": cur, "fpm": want,
                                            "commanded_fpm": initial_vs,
                                            "floor": perf.floor(crate::aircraft::speed_floor_kts()),
                                            "vy": perf.vy_kts}),
                                    );
                                    if want < cur && !eased_for_speed {
                                        eased_for_speed = true;
                                        let line =
                                            "Easing the climb to hold airspeed.".to_string();
                                        if let Some(s) = &status {
                                            s.alert(
                                                crate::plugin_status::AlertLevel::Caution,
                                                &line,
                                            );
                                        }
                                        if let Some(v) = &voice {
                                            v.alert(Priority::Caution, line);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                // Poll fast near the target (tight level-off trigger) and during a
                // climb (so airspeed protection reacts promptly); slower otherwise.
                let secs = if (alt - target).abs() <= CAPTURE_APPROACH_FT {
                    1
                } else if was_climb {
                    2
                } else {
                    3
                };
                tokio::time::sleep(std::time::Duration::from_secs(secs)).await;
            }
        });
    });
}

pub struct SetHeading {
    pub cfg: ApConfig,
}

#[derive(Deserialize)]
struct Args {
    /// Target magnetic heading in degrees. Omitted = hold the current one.
    heading: Option<f64>,
    /// "Keep current heading": bug whatever we're flying right now.
    #[serde(default)]
    current: bool,
}

#[async_trait]
impl Tool for SetHeading {
    fn name(&self) -> &str {
        "set_heading"
    }

    fn description(&self) -> &str {
        "Command the autopilot to turn to and hold a magnetic heading. Engages \
         the autopilot and heading mode, then flies the given heading in degrees. \
         Use for 'autopilot heading 270', 'fly heading 090', 'turn to heading 180', \
         'steer 270', 'come left to heading 200', 'heading bug 360', 'fly two \
         seven zero', 'select heading 120'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "heading": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 360,
                    "description": "Magnetic heading in degrees (0-359)."
                },
                "current": {
                    "type": "boolean",
                    "description": "Hold the CURRENT heading ('keep current heading') — no heading number needed."
                }
            }
        })
    }

    // No confirmation: a misheard heading just turns the wrong way and is
    // trivially corrected — unlike altitude/VS (terrain) or engine cutoff.

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        let heading = match args.heading {
            Some(h) => {
                // A heading well past 360 is a mishear, not a wrap: "turn heading
                // 1018" (QNH bleeding into the phrase, seen live) silently flew 298.
                // Refuse it; only a small overshoot ("370") wraps.
                if h > 400.0 {
                    return Ok(format!(
                        "Say again? I heard heading {h:.0}, and headings run zero to three sixty."
                    ));
                }
                // Normalize into 0..360 (so "370" → 10, "-10" → 350).
                h.rem_euclid(360.0)
            }
            // "Keep current heading" (beeped live, 2026-07-12): bug what
            // we're flying right now — no number to hear, nothing to mishear.
            None if args.current => ctx
                .xplane
                .get_float(crate::datarefs::MAG_PSI)
                .await?
                .rem_euclid(360.0),
            None => return Ok("Say again? Which heading?".to_string()),
        };

        // Engage the servos LAST, once the bug already sits on the target — so
        // the autopilot flies to a STATIONARY bug. The old order (servos on,
        // then walk the bug one knob-press per degree over several seconds) left
        // the AP chasing a crawling bug: on any large turn the SR22 "just kept
        // turning," following the bug around as it moved, and only small turns
        // (a press or two, sub-second) captured cleanly (live, 2026-07-12).
        // Order within the pre-engage steps still matters: the FD prep leads
        // (the C90's servos drop unless the FD is up first), and HDG mode is
        // armed before the bug so a mode-engage dial-snap can't clobber the
        // target. Then drive the bug (knob presses, dataref fallback), servos
        // on, and confirm.
        for c in &self.cfg.fd_commands {
            ctx.xplane.run_command(c).await?;
        }
        engage_mode(ctx, HEADING_STATUS, Button::Heading).await?;
        let actual = drive_heading_bug(ctx.xplane, heading).await;
        autoflight::press(ctx.xplane, Button::Engage).await?;
        if servos_engaged(ctx).await? {
            Ok(format!(
                "Roger — autopilot engaged, heading mode, bug to {}.",
                spoken_heading(actual)
            ))
        } else {
            Ok(format!(
                "Heading bug set to {}, but the autopilot didn't engage — {}.",
                spoken_heading(actual),
                no_engage_reason(ctx, &self.cfg).await
            ))
        }
    }
}

/// Format a heading for readback: rounded to the degree, three digits, and
/// NORTH spoken as 360 — a heading bug reads "three six zero", never
/// "zero zero zero" ("turn heading north" read back "bug to 000", 2026-07-12).
pub(crate) fn spoken_heading(deg: f64) -> String {
    let d = (deg.round() as i64).rem_euclid(360);
    format!("{:03}", if d == 0 { 360 } else { d })
}

pub struct Navigate {
    pub cfg: ApConfig,
}

#[async_trait]
impl Tool for Navigate {
    fn name(&self) -> &str {
        "navigate"
    }

    fn description(&self) -> &str {
        "Couple the autopilot to the GPS flight plan and follow it: selects GPS as \
         the nav source and engages NAV mode. Use for 'navigate', 'follow the flight \
         plan', 'follow the route', 'track the flight plan', 'fly the route', \
         'engage nav', 'nav mode', 'GPS nav'. Load the route first with load_route."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // GPS as the CDI source, servos on, then LNAV. Following a *GPS flight plan*
        // is GPSS (lateral nav) — NAV mode is VOR/LOC capture, a different thing.
        // The MD-82 (and other study FMS aircraft) only fly the loaded route under
        // GPSS. Engage GPSS first; fall back to NAV for any AP that tracks the GPS
        // under NAV instead (keeps the simpler GA setups working).
        autoflight::press(ctx.xplane, Button::HsiGps).await?;
        engage_ap(ctx, &self.cfg).await?;
        engage_mode(ctx, GPSS_STATUS, Button::Gpss).await?;
        let mut lnav = mode_engaged(ctx, GPSS_STATUS).await;
        if !lnav {
            engage_mode(ctx, NAV_STATUS, Button::Nav).await?;
            lnav = mode_engaged(ctx, NAV_STATUS).await;
        }

        match (servos_engaged(ctx).await?, lnav) {
            (true, true) => Ok("Roger, following the flight plan, LNAV engaged.".to_string()),
            (true, false) => Ok(
                "GPS selected and servos on, but LNAV didn't engage — check the autopilot."
                    .to_string(),
            ),
            (false, _) => Ok(format!(
                "Set GPS nav, but the autopilot didn't engage — {}.",
                no_engage_reason(ctx, &self.cfg).await
            )),
        }
    }
}

pub struct SetAltitude {
    pub cfg: ApConfig,
}

#[derive(Deserialize)]
struct AltArgs {
    /// Target altitude in feet.
    altitude: f64,
}

#[async_trait]
impl Tool for SetAltitude {
    fn name(&self) -> &str {
        "set_altitude"
    }

    fn description(&self) -> &str {
        "Set the autopilot target altitude in feet and engage altitude hold. Use \
         for 'autopilot altitude 5500', 'climb and maintain 7000', 'set altitude \
         3000', 'maintain 5500', 'level off at 6000', 'altitude to 4000', \
         'select 7000 feet'. Pair with set_vertical_speed to start a climb or \
         descent."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "altitude": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 60000,
                    "description": "Target altitude in feet MSL."
                }
            },
            "required": ["altitude"]
        })
    }

    // No confirm: an altitude assignment is routine ("climb and maintain X"), and the
    // *dangerous* mishears are already gated — `reject_unsafe_target` refuses anything
    // into the terrain or above the service ceiling, so a misheard "climb 17000" or a
    // descent into the ground can't execute even on the LLM-free fast-path. The
    // readback ("descending to 2000 feet") is the catch for an in-envelope mishear,
    // undone instantly with "level off". A confirm on every level change is friction
    // without payoff; the steep case worth gating — a fast descent rate — still
    // confirms in `set_vertical_speed`/`set_climb`.

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: AltArgs = serde_json::from_value(args.clone())?;
        let mut altitude = args.altitude.max(0.0);

        // A climb that still doesn't clear the ground ahead is flown to the
        // floor rather than refused: the pilot is already climbing away from
        // something, and "negative" would leave them in it.
        let asked = altitude;
        let raised = terrain_raised_target(ctx, altitude).await;
        if let Some(floor) = raised {
            altitude = floor;
        }

        // Safety gate: refuse a target into the terrain or above the ceiling rather
        // than fly the aircraft toward the ground. "auto level" with no number lands
        // here as 0 feet — refused, and pointed at the right tool.
        if let Some(refusal) =
            reject_unsafe_target(ctx, altitude, self.cfg.service_ceiling_ft).await
        {
            return Ok(refusal);
        }

        // Fly to it: arm the altitude AND command a vertical speed (ALT hold alone
        // won't change altitude on this AP), at a rate scaled to the change, then
        // capture and level off.
        let (movement, _fpm, engaged, firewalled) = fly_to(ctx, altitude, None, &self.cfg).await?;
        let power = if firewalled { "full power, " } else { "" };
        let terrain = match raised {
            Some(_) => format!(
                " {asked:.0} feet is into the terrain ahead, so I'm taking us to {altitude:.0}."
            ),
            None => String::new(),
        };
        if engaged {
            Ok(format!(
                "Roger — {power}{} {altitude:.0} feet.{terrain}{}",
                movement.to_lowercase(),
                crate::interaction::route_continuation(
                    crate::flight_state::Owner::Procedure.active()
                )
            ))
        } else {
            Ok(format!(
                "Set to {} {altitude:.0} feet, but the autopilot didn't engage — {}.",
                movement.to_lowercase(),
                no_engage_reason(ctx, &self.cfg).await
            ))
        }
    }
}

/// `level_off` — capture the current altitude, level the wings, and hold. The
/// "auto level / straight and level" command, which has no target (it holds
/// where you are now), so it can't be expressed as `set_altitude`.
pub struct LevelOff {
    pub cfg: ApConfig,
}

#[async_trait]
impl Tool for LevelOff {
    fn name(&self) -> &str {
        "level_off"
    }

    fn description(&self) -> &str {
        "Level the aircraft and hold the CURRENT altitude — engage the autopilot \
         in altitude hold at the altitude you're at now, wings level. Takes no \
         target. Use for 'auto level', 'level off', 'level the aircraft', 'wings \
         level', 'straight and level', 'hold this altitude', 'hold present \
         altitude', 'stop the climb', 'stop descending', 'level here'."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Capture where we are now and hold it (rounded to the nearest 100 ft,
        // the way an ALT-button capture reads out).
        let current = ctx.xplane.get_float(CURRENT_ALT).await?;
        let target = (current / 100.0).round() * 100.0;

        // "Stop the descent" while a follower flies: hand it the altitude the
        // same way a commanded one is handed over. Wings and throttle are its
        // own (the lateral law and the governor): taking those here would put
        // two writers on each.
        if crate::pilot_altitude::take(target) {
            return Ok(format!(
                "Roger, holding {target:.0} feet. You have the altitude; I'm still flying the track."
            ));
        }

        engage_ap(ctx, &self.cfg).await?;
        autoflight::press(ctx.xplane, Button::WingLeveler).await?;
        engage_mode(ctx, ALT_HOLD_STATUS, Button::AltitudeHold).await?;
        autoflight::set_dial(ctx.xplane, Dial::Altitude, target).await?;
        // Settle to cruise power. Intentional even though it overrides the pilot's
        // throttle: the only power that *holds* an altitude on a 172 is ~cruise, and
        // the pre-level-off setting is by definition not a hold power (full from a
        // climb would accelerate; idle from a descent would decay toward a stall).
        // Converging to cruise from either direction is the safe default until
        // airspeed-aware power management lands — making it conditional (leave the
        // pilot's setting) would risk that decay when levelling from a descent.
        // "Cruise" is the airframe's (`level_power`): a running governor keeps
        // the throttle, a declared cruise speed is handed to it, and only an
        // airframe declaring none gets the old GA lever.
        let power = match level_power(
            crate::tools::speedhold::flight_target(),
            self.cfg.climb.cruise_ias_kts,
            current,
        ) {
            LevelPower::Governor => String::new(),
            LevelPower::ArmCruise(kts) => {
                crate::tools::speedhold::hold_for_pilot(
                    ctx.config.xplane_base_url.clone(),
                    ctx.voice.cloned(),
                    kts,
                );
                format!("{kts:.0} knots, ")
            }
            LevelPower::Lever(ratio) => {
                let _ = ctx.xplane.set_float(&throttle_dataref(), ratio).await;
                "cruise power, ".to_string()
            }
        };

        if servos_engaged(ctx).await? {
            Ok(format!(
                "Roger, wings level, {power}holding {target:.0} feet."
            ))
        } else {
            Ok(format!(
                "Set to hold {target:.0} feet, but the autopilot didn't engage — {}.",
                no_engage_reason(ctx, &self.cfg).await
            ))
        }
    }
}

pub struct SetVerticalSpeed {
    pub cfg: ApConfig,
}

#[derive(Deserialize)]
struct VsArgs {
    /// Vertical speed in feet per minute; positive climbs, negative descends.
    fpm: f64,
}

#[async_trait]
impl Tool for SetVerticalSpeed {
    fn name(&self) -> &str {
        "set_vertical_speed"
    }

    fn description(&self) -> &str {
        "Set the autopilot vertical speed in feet per minute and engage vertical \
         speed mode. Positive climbs, negative descends. Use for 'vertical speed \
         500 up', 'descend at 700', 'VS 1000', 'vertical speed down 500', \
         'start a descent', 'climb at 500', 'rate of descent 700', 'ease down \
         at 300'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "fpm": {
                    "type": "integer",
                    "minimum": -2000,
                    "maximum": 2000,
                    "description": "Vertical speed, feet per minute. Positive = climb, negative = descend."
                }
            },
            "required": ["fpm"]
        })
    }

    fn confirm_prompt(&self, args: &Value) -> Option<String> {
        let a: VsArgs = serde_json::from_value(args.clone()).ok()?;
        let fpm = a.fpm.clamp(-2000.0, 2000.0);
        // Only an extreme descent is gated — climbs can't hit terrain, and a
        // normal descent (up to ~1500 fpm) is routine for a 172.
        if fpm <= VS_CONFIRM_DESCENT_FPM {
            Some(describe_vs(fpm))
        } else {
            None
        }
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: VsArgs = serde_json::from_value(args.clone())?;
        // Clamp to a sane light-GA envelope so a misheard magnitude can't
        // command an extreme dive/climb.
        let fpm = args.fpm.clamp(-2000.0, 2000.0);

        // Engage VS mode FIRST — when it engages, `sim/autopilot/vertical_speed`
        // snaps the dial to the live climb rate, so writing our target LAST is
        // what makes the aircraft fly the value we confirmed. The command is a
        // toggle, so guard it: a repeat VS command must not switch the mode back
        // off (which would leave the dial holding a value the AP isn't flying).
        engage_ap(ctx, &self.cfg).await?;
        engage_mode(ctx, VS_STATUS, Button::VerticalSpeed).await?;
        autoflight::set_dial(ctx.xplane, Dial::Vs, fpm).await?;

        // Confirm the actual state, not the intent: both that the servos are
        // flying and that VS mode actually latched.
        let actual = autoflight::read_dial(ctx.xplane, Dial::Vs).await?;
        let phrase = describe_vs(actual);
        match (
            servos_engaged(ctx).await?,
            mode_engaged(ctx, VS_STATUS).await,
        ) {
            (true, true) => Ok(format!(
                "Roger — autopilot engaged, vertical speed mode, {phrase}."
            )),
            (true, false) => Ok(format!(
                "{phrase} dialed, but vertical speed mode didn't latch — check the AP."
            )),
            (false, _) => Ok(format!(
                "{phrase} set, but the autopilot didn't engage — {}.",
                no_engage_reason(ctx, &self.cfg).await
            )),
        }
    }
}

/// `set_climb` — climb or descend to a target altitude at a chosen rate, then
/// level off. Sets the AP target altitude AND vertical speed and lets the
/// autopilot capture the altitude (it levels off on its own — no copilot polling
/// loop, no overshoot). Direction is inferred from target vs current altitude.
pub struct SetClimb {
    pub cfg: ApConfig,
}

#[derive(Deserialize)]
struct ClimbArgs {
    /// Target altitude in feet MSL.
    altitude: f64,
    /// Climb/descent rate in feet per minute (magnitude; sign is inferred).
    fpm: f64,
}

#[async_trait]
impl Tool for SetClimb {
    fn name(&self) -> &str {
        "set_climb"
    }

    fn description(&self) -> &str {
        "Climb or descend to a target altitude at a given rate, then level off \
         automatically (the autopilot captures the altitude). One command for \
         'climb to 7000 at 500', 'descend to 2000 at a thousand', 'take us up to \
         5500 at 700', 'bring us down to 3000 at 500 feet per minute'. Direction \
         is inferred from the target vs current altitude; fpm is the rate."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "altitude": {
                    "type": "integer", "minimum": 0, "maximum": 60000,
                    "description": "Target altitude in feet MSL."
                },
                "fpm": {
                    "type": "integer", "minimum": 0, "maximum": 2000,
                    "description": "Climb/descent rate in feet per minute (magnitude)."
                }
            },
            "required": ["altitude", "fpm"]
        })
    }

    fn confirm_prompt(&self, args: &Value) -> Option<String> {
        let a: ClimbArgs = serde_json::from_value(args.clone()).ok()?;
        // A sensible climb/descent executes directly — execute()'s unsafe-target
        // gate (terrain floor / service ceiling) is the real safety net. Only a
        // grossly abnormal rate (a likely mishear) still confirms.
        if a.fpm.abs() > CLIMB_CONFIRM_RATE_FPM {
            Some(format!(
                "altitude {:.0} feet at {:.0} feet per minute",
                a.altitude.max(0.0),
                a.fpm.abs()
            ))
        } else {
            None
        }
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: ClimbArgs = serde_json::from_value(args.clone())?;
        let mut altitude = args.altitude.max(0.0);
        let asked = altitude;
        let raised = terrain_raised_target(ctx, altitude).await;
        if let Some(floor) = raised {
            altitude = floor;
        }
        if let Some(refusal) =
            reject_unsafe_target(ctx, altitude, self.cfg.service_ceiling_ft).await
        {
            return Ok(refusal);
        }
        // A zero (or near-zero) rate would arm the target but command VS 0 and skip
        // the climb power firewall — a silent no-op that still reads back "climbing."
        // The schema permits fpm 0, so treat anything under 1 fpm as "unspecified"
        // and auto-scale the rate, the same as a plain set_altitude.
        let rate = (args.fpm.abs() >= 1.0).then(|| args.fpm.abs().min(2000.0));
        let (movement, fpm, engaged, firewalled) = fly_to(ctx, altitude, rate, &self.cfg).await?;
        let power = if firewalled { "full power, " } else { "" };
        let terrain = match raised {
            Some(_) => format!(
                " {asked:.0} feet is into the terrain ahead, so I'm taking us to {altitude:.0}."
            ),
            None => String::new(),
        };
        if engaged {
            let following = crate::flight_state::Owner::Procedure.active();
            let rate_line = if following {
                ", with descent or climb rate managed by the route follower".to_string()
            } else {
                format!(" at {fpm:.0} feet per minute")
            };
            Ok(format!(
                "Roger — {power}{} {altitude:.0} feet{rate_line}, leveling off there.{terrain}{}",
                movement.to_lowercase(),
                crate::interaction::route_continuation(following)
            ))
        } else {
            Ok(format!(
                "Set to {} {altitude:.0} feet, but the autopilot didn't engage — {}.",
                movement.to_lowercase(),
                no_engage_reason(ctx, &self.cfg).await
            ))
        }
    }
}

pub struct AutopilotOff;

#[async_trait]
impl Tool for AutopilotOff {
    fn name(&self) -> &str {
        "autopilot_off"
    }

    fn description(&self) -> &str {
        "Disengage the autopilot — disconnect the servos so the pilot hand-flies. \
         Use for 'autopilot off', 'disconnect the autopilot', 'disengage AP', \
         'kill the autopilot', 'I have the airplane', 'my airplane', 'I'll hand \
         fly', 'drop the autopilot', 'take me off autopilot'."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Also release an active emergency recovery or autoland — "my controls"
        // hands the aircraft fully back to the pilot. If one was actually flying,
        // acknowledge *that* rather than the AP-off wording.
        let was_flying = crate::flight_state::release_recovery_and_landing();
        if was_flying {
            crate::monitor::note_pilot_release();
            return Ok("You have the aircraft.".to_string());
        }
        if !servos_engaged(ctx).await? {
            return Ok("Autopilot's already off.".to_string());
        }
        autoflight::press(ctx.xplane, Button::Disconnect).await?;
        if servos_engaged(ctx).await? {
            Ok("The autopilot didn't disengage — disconnect it manually.".to_string())
        } else {
            Ok("Roger, autopilot off. You have the airplane.".to_string())
        }
    }
}

/// Turn a signed fpm value into a spoken phrase.
fn describe_vs(fpm: f64) -> String {
    let mag = fpm.abs();
    if mag < 1.0 {
        "vertical speed level".to_string()
    } else if fpm > 0.0 {
        format!("climb {mag:.0} feet per minute")
    } else {
        format!("descend {mag:.0} feet per minute")
    }
}

// ---- airliner autoflight (jets): speed select + autothrottle -----------------

/// The AP airspeed select (standard X-Plane AP dataref).
/// MD-82 autothrottle: a custom on/off switch (arm) driven by a toggle command;
/// engaging into the detent needs an EPR thrust rating selected then EPR_lim, which
/// clamps autothrottle_enabled to a holding mode (needs the flight director on).
const AT_SWITCH: &str = "laminar/md82/autopilot/autothrottle_switch";
const AT_ENABLED: &str = autoflight::names::AT_MODE; // 0 arm, 1 speed, 2 EPR
const CMD_AT_TOGGLE: &str = "laminar/md82cmd/autopilot/autothrottle_switch";
const CMD_EPR_CLIMB: &str = "laminar/md82cmd/autopilot/EPR_CL"; // select climb thrust rating
const CMD_EPR_ENGAGE: &str = "laminar/md82cmd/autopilot/EPR_lim"; // engage EPR mode (clamp)

pub struct SetSpeed {
    /// Final-approach speed from the profile's reference card (`approach_kts`),
    /// for "set speed for landing" — `None` if the airframe doesn't declare one.
    approach_kts: Option<f64>,
}

impl SetSpeed {
    pub fn new(approach_kts: Option<f64>) -> Self {
        Self { approach_kts }
    }
}

#[derive(Deserialize)]
struct SpeedArgs {
    /// Target indicated airspeed, knots.
    #[serde(default)]
    knots: Option<f64>,
    /// Change relative to the current bug, knots ("slow down 20" = -20).
    #[serde(default)]
    delta: Option<f64>,
    /// Named target: "approach" = the profile's final-approach speed.
    #[serde(default)]
    preset: Option<String>,
}

#[async_trait]
impl Tool for SetSpeed {
    fn name(&self) -> &str {
        "set_speed"
    }
    fn description(&self) -> &str {
        "Set the autopilot target airspeed (the speed bug). Use for 'set speed \
         250', 'speed 280', 'select 300 knots', 'set the bug to 240', 'airspeed \
         250'. Relative: 'reduce speed by 20', 'slow down' (delta, negative = \
         slower). 'Set speed for landing' / 'approach speed' = preset 'approach'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "knots": {"type": "number", "description": "target IAS, knots"},
                "delta": {"type": "number", "description": "change vs the current bug, knots (negative = slower)"},
                "preset": {"type": "string", "enum": ["approach"], "description": "named target: the profile's final-approach speed"}
            }
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: SpeedArgs = serde_json::from_value(args.clone())?;
        let (kts, why) = if let Some(k) = a.knots {
            (k, String::new())
        } else if a.preset.as_deref() == Some("approach") {
            match self.approach_kts {
                Some(k) => (k, " for the approach".to_string()),
                None => {
                    return Ok(
                        "No approach speed on file for this airframe — give me a number."
                            .to_string(),
                    )
                }
            }
        } else if let Some(d) = a.delta {
            let current = autoflight::read_dial(ctx.xplane, Dial::AirspeedKts).await?;
            (current + d, String::new())
        } else {
            return Ok("Say a speed in knots, or 'speed for landing'.".to_string());
        };
        let kts = kts.clamp(80.0, 400.0);
        autoflight::set_dial(ctx.xplane, Dial::AirspeedKts, kts).await?;
        let actual = autoflight::read_dial(ctx.xplane, Dial::AirspeedKts).await.unwrap_or(kts);
        Ok(format!("Speed bug {actual:.0} knots{why}."))
    }
}

pub struct Autothrottle;

#[derive(Deserialize)]
struct AtArgs {
    /// true = engage, false = disengage.
    on: bool,
}

#[async_trait]
impl Tool for Autothrottle {
    fn name(&self) -> &str {
        "autothrottle"
    }
    fn description(&self) -> &str {
        "Engage or disengage the autothrottle. Use for 'autothrottle on', 'engage \
         the autothrottle', 'autothrottle off', 'disconnect the autothrottle', \
         'auto throttle arm'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": { "on": {"type": "boolean", "description": "true = engage, false = disengage"} },
            "required": ["on"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: AtArgs = serde_json::from_value(args.clone())?;
        let armed = ctx.xplane.get_float(AT_SWITCH).await.unwrap_or(0.0) > 0.5;
        if !a.on {
            if armed {
                ctx.xplane.run_command(CMD_AT_TOGGLE).await?; // also clears the mode
            }
            return Ok("Autothrottle off.".to_string());
        }
        // Arm, select a thrust rating (climb), then engage into the detent.
        if !armed {
            ctx.xplane.run_command(CMD_AT_TOGGLE).await?;
        }
        ctx.xplane.run_command(CMD_EPR_CLIMB).await.ok();
        ctx.xplane.run_command(CMD_EPR_ENGAGE).await?;
        // Engaged only if it clamped into a holding mode (needs the flight director).
        if ctx.xplane.get_float(AT_ENABLED).await.unwrap_or(0.0) >= 0.5 {
            Ok("Autothrottle engaged, climb thrust.".to_string())
        } else {
            Ok(
                "Autothrottle armed, but it didn't engage. Check the flight director is on."
                    .to_string(),
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_rate_scales_with_the_change_and_is_bounded() {
        // Small step → the gentle default floor, not sluggish.
        assert_eq!(auto_vs_fpm(300.0, true, MAX_CLIMB_FPM), DEFAULT_VS_FPM);
        // Mid change → scaled (≈ delta / 3).
        assert_eq!(auto_vs_fpm(1500.0, true, MAX_CLIMB_FPM), 500.0);
        // Big climb → capped at the airframe's sustainable climb rate, so it can't
        // outrun the available power.
        assert_eq!(auto_vs_fpm(9000.0, true, MAX_CLIMB_FPM), MAX_CLIMB_FPM);
        // Descents get their own (higher) cap.
        assert_eq!(auto_vs_fpm(9000.0, false, MAX_CLIMB_FPM), MAX_DESCENT_FPM);
        // Direction is by magnitude — sign of the delta doesn't matter here.
        assert_eq!(auto_vs_fpm(-1500.0, false, MAX_CLIMB_FPM), 500.0);
        // A jet's cap is its own enroute climb (refactor audit #3).
        assert_eq!(auto_vs_fpm(9000.0, true, 2000.0), 2000.0);
    }

    fn perf(id: &str) -> ClimbPerf {
        ClimbPerf::from_profile(&crate::aircraft::load(id).expect(id))
    }

    /// The climb numbers are the airframe's: the 172 keeps the GA figures
    /// it always flew (700 fpm, its cruise 108), the jets get their own.
    #[test]
    fn a_pilot_climb_flies_the_airframes_numbers() {
        let c172 = perf("cessna_172sp");
        assert_eq!(c172.max_climb_fpm, 700.0);
        assert_eq!(c172.vy_kts, Some(74.0));
        assert!((c172.floor_kts - 57.6).abs() < 0.01, "{}", c172.floor_kts);
        assert_eq!(c172.cruise_ias_kts, Some(108.0));
        let sr22 = perf("cirrus_sr22");
        assert_eq!(sr22.max_climb_fpm, 700.0);
        assert_eq!(sr22.cruise_ias_kts, None, "no cruise declared: the GA lever");
        let b738 = perf("boeing_737_800");
        assert_eq!(b738.max_climb_fpm, 2000.0);
        assert!((b738.floor_kts - 166.8).abs() < 0.01, "{}", b738.floor_kts);
        let a330 = perf("airbus_a330_300");
        assert_eq!(a330.max_climb_fpm, 2000.0);
        assert!((a330.floor_kts - 210.0).abs() < 0.01, "{}", a330.floor_kts);
        // Nothing declared: the old 60 kt floor and 700 fpm.
        let none = ClimbPerf::default();
        assert_eq!(none.floor(None), CLIMB_MIN_KTS);
        assert_eq!(none.max_climb_fpm, MAX_CLIMB_FPM);
    }

    /// The #181 door: a 737 climbing on a pilot's command at 166 kt and
    /// falling. The old 60 kt protection never fired; the airframe's floor
    /// takes the whole climb away.
    #[test]
    fn a_jet_climb_bleeding_toward_its_floor_is_eased() {
        let b738 = perf("boeing_737_800");
        let floor = Some(167.0);
        assert_eq!(protected_climb_fpm(2000.0, 230.0, 0.0, &b738, floor, None), 2000.0);
        assert_eq!(protected_climb_fpm(2000.0, 166.0, -1.0, &b738, floor, None), 0.0);
        let mid = protected_climb_fpm(2000.0, 190.0, 0.0, &b738, floor, None);
        assert!(mid > 0.0 && mid < 2000.0, "{mid}");
        assert_eq!(mid % VS_TRIM_FPM, 0.0, "steps a VS wheel can take");
        // A governor holding under Vy lowers the full-rate speed with it.
        let held = protected_climb_fpm(2000.0, 195.0, 0.0, &b738, floor, Some(195.0));
        assert_eq!(held, 2000.0);
    }

    /// The 172 stays close to what it flew: full rate at Vy and above,
    /// eased below it, nothing left at its floor (it used to keep the whole
    /// rate to 60 kt, two knots over this floor).
    #[test]
    fn a_172_climb_keeps_its_rate_at_vy() {
        let c172 = perf("cessna_172sp");
        let floor = Some(58.0);
        assert_eq!(protected_climb_fpm(700.0, 80.0, 0.0, &c172, floor, None), 700.0);
        assert_eq!(protected_climb_fpm(700.0, 74.0, 0.0, &c172, floor, None), 700.0);
        assert_eq!(protected_climb_fpm(700.0, 58.0, 0.0, &c172, floor, None), 0.0);
        let eased = protected_climb_fpm(700.0, 66.0, 0.0, &c172, floor, None);
        assert!(eased > 0.0 && eased < 700.0, "{eased}");
        // A descent is never touched.
        assert_eq!(protected_climb_fpm(-500.0, 50.0, 0.0, &c172, floor, None), -500.0);
    }

    /// The level-off's power is the airframe's: a governor keeps it, a
    /// declared cruise is handed to the governor (250 below 10,000), and
    /// only an airframe declaring none gets the GA lever.
    #[test]
    fn a_level_off_settles_on_the_airframes_cruise() {
        assert_eq!(level_power(Some(200.0), Some(255.0), 5000.0), LevelPower::Governor);
        assert_eq!(level_power(None, Some(255.0), 5000.0), LevelPower::ArmCruise(250.0));
        assert_eq!(level_power(None, Some(285.0), 24_000.0), LevelPower::ArmCruise(285.0));
        assert_eq!(level_power(None, Some(108.0), 5000.0), LevelPower::ArmCruise(108.0));
        assert_eq!(level_power(None, None, 5000.0), LevelPower::Lever(CRUISE_THROTTLE));
    }

    #[test]
    fn north_reads_back_as_360_never_000() {
        // "turn heading north" (0°) read back "bug to 000", 2026-07-12 — a
        // heading bug is spoken "three six zero", never "zero zero zero".
        assert_eq!(spoken_heading(0.0), "360");
        assert_eq!(spoken_heading(360.0), "360");
        assert_eq!(spoken_heading(0.3), "360"); // rounds to north
        assert_eq!(spoken_heading(359.7), "360");
        // Everything else is just three-digit degrees.
        assert_eq!(spoken_heading(90.0), "090");
        assert_eq!(spoken_heading(270.0), "270");
        assert_eq!(spoken_heading(5.0), "005");
    }

    #[test]
    fn terrain_floor_is_ground_relative() {
        // At a sea-level field (current 0, agl 0) the floor is just the margin.
        assert_eq!(terrain_floor_ft(0.0, 0.0), MIN_TARGET_AGL_FT);
        // Flying at 8000 ft MSL with 3000 ft AGL → ground ≈ 5000 ft, floor ≈ 5500.
        let floor = terrain_floor_ft(8000.0, 3000.0 / METERS_TO_FEET);
        assert!(
            (floor - (5000.0 + MIN_TARGET_AGL_FT)).abs() < 1.0,
            "floor was {floor}"
        );
    }

    #[test]
    fn capture_action_levels_off_stands_down_and_waits() {
        // Climb: level off once we reach the target, not before.
        assert_eq!(
            capture_action(7000.0, 7000.0, 7000.0, true),
            Capture::LevelOff
        );
        assert_eq!(capture_action(7000.0, 7000.0, 6900.0, true), Capture::Wait);
        // Descent: level off once we come down to the target, not before.
        assert_eq!(
            capture_action(3000.0, 3000.0, 3000.0, false),
            Capture::LevelOff
        );
        assert_eq!(capture_action(3000.0, 3000.0, 3100.0, false), Capture::Wait);
        // The selected dial moved off our target (pilot re-commanded) → stand down,
        // even if we happen to be near the old target's altitude.
        assert_eq!(
            capture_action(3000.0, 7000.0, 6950.0, true),
            Capture::StandDown
        );
    }

    // --- arm_altitude / reject_unsafe_target: the trust-critical writes, against a
    // fake sim so they're tested without a running X-Plane. ---

    use crate::tools::ToolCtx;
    use crate::xplane::FakeXPlane;

    fn ctx<'a>(fake: &'a FakeXPlane, cfg: &'a crate::config::Config) -> ToolCtx<'a> {
        ToolCtx {
            xplane: fake,
            config: cfg,
            voice: None,
            status: None,
            reminders: None,
        }
    }

    #[tokio::test]
    async fn set_heading_engages_servos_after_the_bug_is_on_target() {
        // The "kept turning" fix: the servos must come on LAST, once the bug
        // already sits on the target, so the AP flies to a STATIONARY bug
        // instead of chasing it around as it crawls one knob-press per degree.
        // Bug starts 20 deg off the target so the knob actually turns.
        let fake = FakeXPlane::with_reads(&[
            (HEADING_STATUS, 0.0), // HDG off → engage_mode presses it
            (HEADING_BUG, 250.0),  // 20 deg from the 270 target
            (SERVOS_ON, 1.0),      // engaged when the readback checks
        ]);
        let cfg = crate::config::Config::default();
        let out = SetHeading {
            cfg: ApConfig::default(),
        }
        .execute(&serde_json::json!({ "heading": 270 }), &ctx(&fake, &cfg))
        .await
        .unwrap();
        assert!(out.contains("engaged"), "reports engaged: {out:?}");

        let servos = fake.command_index(CMD_SERVOS_ON).expect("servos engaged");
        let mode = fake
            .command_index(CMD_HEADING_MODE)
            .expect("HDG mode pressed");
        let bug = fake.command_index(CMD_HDG_UP).expect("bug knob turned");
        assert!(mode < bug, "HDG mode is armed before the bug is driven");
        assert!(
            servos > bug,
            "servos engage AFTER the bug reaches the target, not before"
        );
        assert!(servos > mode, "servos engage last");
    }

    #[tokio::test]
    async fn arm_altitude_climb_arms_target_vs_and_firewalls_power() {
        let fake = FakeXPlane::with_reads(&[(CURRENT_ALT, 2000.0)]);
        let cfg = crate::config::Config::default();
        let (movement, signed, capture, firewalled) =
            arm_altitude(&ctx(&fake, &cfg), 7000.0, None, MAX_CLIMB_FPM, None)
                .await
                .unwrap();

        assert_eq!(movement, "Climbing to");
        assert!(signed > 0.0, "climb VS should be positive, was {signed}");
        assert_eq!(
            capture,
            Some(true),
            "a climb needs a capture watcher (was_climb)"
        );
        assert_eq!(
            fake.last_write(ALT_DIAL),
            Some(7000.0),
            "arms the target altitude"
        );
        assert!(fake.ran(CMD_VS_MODE), "engages VS mode");
        assert_eq!(
            fake.last_write(VS_DIAL),
            Some(signed),
            "dials the commanded VS"
        );
        assert_eq!(
            fake.last_write(&throttle_dataref()),
            Some(1.0),
            "firewalls the throttle for the climb"
        );
        assert!(firewalled, "and says so");
    }

    /// A running speed governor owns the throttle: the pilot's climb leaves
    /// it to it (one writer per dataref) and promises no full power.
    #[tokio::test]
    async fn arm_altitude_climb_under_a_governor_leaves_the_throttle() {
        let fake = FakeXPlane::with_reads(&[(CURRENT_ALT, 2000.0)]);
        let cfg = crate::config::Config::default();
        let (movement, signed, _, firewalled) =
            arm_altitude(&ctx(&fake, &cfg), 12_000.0, None, 2000.0, Some(250.0))
                .await
                .unwrap();
        assert_eq!(movement, "Climbing to");
        assert_eq!(signed, 2000.0, "the jet's own climb cap");
        assert!(!firewalled);
        assert!(!fake.wrote(&throttle_dataref()), "the governor's throttle");
    }

    #[tokio::test]
    async fn arm_altitude_descent_does_not_touch_the_throttle() {
        let fake = FakeXPlane::with_reads(&[(CURRENT_ALT, 9000.0)]);
        let cfg = crate::config::Config::default();
        let (movement, signed, capture, firewalled) =
            arm_altitude(&ctx(&fake, &cfg), 3000.0, None, MAX_CLIMB_FPM, None)
                .await
                .unwrap();

        assert_eq!(movement, "Descending to");
        assert!(!firewalled, "a descent never firewalls");
        assert!(signed < 0.0, "descent VS should be negative, was {signed}");
        assert_eq!(
            capture,
            Some(false),
            "a descent still needs the level-off watcher"
        );
        assert_eq!(fake.last_write(VS_DIAL), Some(signed));
        assert!(
            !fake.wrote(&throttle_dataref()),
            "descent must not firewall (or touch) the throttle"
        );
    }

    #[tokio::test]
    async fn arm_altitude_at_target_holds() {
        // Within the capture band of the current altitude → hold, no climb/descent.
        let fake = FakeXPlane::with_reads(&[(CURRENT_ALT, 5020.0)]);
        let cfg = crate::config::Config::default();
        let (movement, signed, capture, firewalled) =
            arm_altitude(&ctx(&fake, &cfg), 5000.0, None, MAX_CLIMB_FPM, None)
                .await
                .unwrap();

        assert_eq!(movement, "Holding at");
        assert_eq!(signed, 0.0);
        assert!(!firewalled);
        assert_eq!(capture, None, "a hold needs no capture watcher");
        assert!(fake.ran(CMD_ALT_HOLD), "engages altitude hold");
        assert!(
            !fake.wrote(VS_DIAL),
            "hold doesn't command a vertical speed"
        );
    }

    #[tokio::test]
    async fn reject_target_into_the_terrain() {
        // Ground ≈ 4000 ft (5000 MSL − 1000 ft AGL); floor ≈ 4500. A 3000 ft target
        // is into the terrain → refused.
        let fake = FakeXPlane::with_reads(&[
            (CURRENT_ALT, 5000.0),
            (crate::datarefs::AGL_METERS, 1000.0 / METERS_TO_FEET),
        ]);
        let cfg = crate::config::Config::default();
        let refusal =
            reject_unsafe_target(&ctx(&fake, &cfg), 3000.0, DEFAULT_SERVICE_CEILING_FT).await;
        assert!(refusal.is_some(), "target into terrain must be refused");
        // A target well above the floor is accepted.
        assert!(
            reject_unsafe_target(&ctx(&fake, &cfg), 8000.0, DEFAULT_SERVICE_CEILING_FT)
                .await
                .is_none()
        );
    }

    #[tokio::test]
    async fn reject_target_above_ceiling() {
        let fake = FakeXPlane::new();
        let cfg = crate::config::Config::default();
        assert!(reject_unsafe_target(
            &ctx(&fake, &cfg),
            DEFAULT_SERVICE_CEILING_FT + 1000.0,
            DEFAULT_SERVICE_CEILING_FT
        )
        .await
        .is_some());
    }

    #[tokio::test]
    async fn reject_falls_back_to_msl_floor_when_terrain_unknown() {
        // No AGL preset → terrain read fails → fall back to the fixed MSL floor.
        let fake = FakeXPlane::with_reads(&[(CURRENT_ALT, 600.0)]);
        let cfg = crate::config::Config::default();
        assert!(
            reject_unsafe_target(&ctx(&fake, &cfg), 300.0, DEFAULT_SERVICE_CEILING_FT)
                .await
                .is_some(),
            "below MSL floor"
        );
        assert!(
            reject_unsafe_target(&ctx(&fake, &cfg), 5000.0, DEFAULT_SERVICE_CEILING_FT)
                .await
                .is_none(),
            "above MSL floor"
        );
    }
}
