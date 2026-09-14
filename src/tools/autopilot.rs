//! `set_heading` — command the autopilot to fly a magnetic heading.
//!
//! "Autopilot heading 270" is three actions: dial the heading bug (a dataref
//! write), engage the servos, and engage HDG mode (two commands). The readback
//! re-reads the bug and the servo state and confirms what actually happened.
//!
//! Datarefs/commands are standard X-Plane (confirmed against the rustdecks
//! profiles), so the tool owns them; the aircraft profile only declares that an
//! autopilot exists.

use std::sync::atomic::{AtomicU64, Ordering};

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx, CMD_SERVOS_ON};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

/// Pilot-side heading bug, degrees magnetic (writable).
const HEADING_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
/// Selected altitude, feet (writable).
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
/// Selected vertical speed, feet per minute (writable). Use the classic
/// `vertical_velocity` dial, not cockpit2's `vvi_dial_fpm`: on the SR22's
/// G1000/Perspective AP the latter isn't the dataref the autopilot follows —
/// verified against the rustdecks SR22 deck, which reads/writes VS here.
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
/// Servo engagement status (read-only; 1 = autopilot flying).
const SERVOS_ON: &str = crate::datarefs::SERVOS_ON;
/// Command: disconnect the autopilot. The stock 172's S-TEC-style AP has no
/// master switch and does not register `sim/autopilot/servos_off`. X-Plane
/// exposes `servos_off_any` ("disco servos, whichever side is active") and the
/// yoke red button `ap_disc_trim_interrupt`. We use `servos_off_any` — a clean
/// servos disconnect — rather than the yoke button, which also interrupts the
/// pitch-trim servo (an unwanted side effect for a voice "autopilot off").
const CMD_SERVOS_OFF: &str = "sim/autopilot/servos_off_any";
/// Command: engage heading-select (HDG) mode.
const CMD_HEADING_MODE: &str = "sim/autopilot/heading";
/// Command: engage NAV mode (follow the active nav source).
const CMD_NAV_MODE: &str = "sim/autopilot/NAV";
/// Command: engage GPSS (lateral nav / LNAV) — fly the *GPS flight plan*. This is
/// the correct "track the magenta line" mode; `NAV` arms VOR/LOC capture instead.
/// Study airliners like the MD-82 fly the FMS route only under GPSS, not NAV.
const CMD_GPSS: &str = "sim/autopilot/gpss";
/// GPSS (LNAV) engaged status.
const GPSS_STATUS: &str = "sim/cockpit2/autopilot/gpss_status";
/// Command: select GPS as the CDI/HSI source (so NAV follows the GPS flight plan).
const CMD_HSI_GPS: &str = "sim/autopilot/hsi_select_gps";
/// NAV-mode status (> 0 = armed/engaged).
const NAV_STATUS: &str = crate::datarefs::NAV_STATUS;
/// Command: engage altitude select/hold (ALT) mode.
const CMD_ALT_HOLD: &str = "sim/autopilot/altitude_hold";
/// Command: wings-level / roll hold — for "auto level" / "straight and level".
const CMD_WING_LEVELER: &str = "sim/autopilot/wing_leveler";
/// Current indicated altitude (feet), read to capture "hold this altitude".
const CURRENT_ALT: &str = crate::datarefs::ALTITUDE_FT;
/// Autopilot altitude targets below this (feet MSL) are refused — a 172's AP is
/// never commanded this low, so it's almost always a misheard/empty value (e.g.
/// "auto level" misread as altitude 0). The gate catches it before the AP flies
/// the aircraft into the ground.
const MIN_AP_ALTITUDE_FT: f64 = 500.0;
/// Command: engage vertical-speed (VS) mode.
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
/// Throttle (all engines), 0..1. A commanded climb sets this to full — the AP
/// only controls pitch, so without power a 172 trades airspeed for altitude.
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}
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
const HEADING_STATUS: &str = crate::datarefs::HEADING_STATUS;
const ALT_HOLD_STATUS: &str = "sim/cockpit2/autopilot/altitude_hold_status";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
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
}

impl Default for ApConfig {
    fn default() -> Self {
        Self {
            fd_commands: Vec::new(),
            ac_powered: false,
            service_ceiling_ft: DEFAULT_SERVICE_CEILING_FT,
        }
    }
}

/// Standard X-Plane inverter status (array; any element > 0 = AC power available).
const INVERTER_ON: &str = "sim/cockpit2/electrical/inverter_on";

/// Command: turn the heading bug one degree (knob presses).
const CMD_HDG_UP: &str = "sim/autopilot/heading_up";
const CMD_HDG_DN: &str = "sim/autopilot/heading_down";

/// Drive the heading BUG to `target` the way a human does — by turning the knob
/// (`heading_up`/`heading_down` presses). Glass avionics that ignore raw dataref
/// writes (the SR22's G1000) register the commands, and the knob animates. Falls
/// back to a direct dataref write when the commands don't move the bug (checked a
/// few presses in). Returns the bug's actual value afterwards.
pub async fn drive_heading_bug(xp: &dyn crate::xplane::XPlane, target: f64) -> f64 {
    drive_heading_bug_step(xp, target, 360.0).await
}

/// Like [`drive_heading_bug`] but turns the knob at most `max_step_deg` toward
/// the target per call — for control loops that drive the bug every tick. Each
/// press is an HTTP round trip, so an unbounded slew stalls the caller: the
/// LXGB go-around's 178° reversal froze the autoland loop ~5 s per tick with
/// no VS or throttle updates the whole time (2026-07-14). A capped call gets
/// there over successive ticks; the direct-write fallback (knob unbound) still
/// jumps straight to the full target.
pub async fn drive_heading_bug_step(
    xp: &dyn crate::xplane::XPlane,
    target: f64,
    max_step_deg: f64,
) -> f64 {
    let target = target.rem_euclid(360.0);
    let Ok(start) = xp.get_float(HEADING_BUG).await else {
        let _ = xp.set_float(HEADING_BUG, target).await;
        return target;
    };
    let full = ((target - start) % 360.0 + 540.0) % 360.0 - 180.0;
    let delta = full.clamp(-max_step_deg.abs(), max_step_deg.abs());
    let leg = (start + delta).rem_euclid(360.0);
    let presses = delta.round().abs() as i64;
    let cmd = if delta >= 0.0 { CMD_HDG_UP } else { CMD_HDG_DN };
    for i in 0..presses {
        if xp.run_command(cmd).await.is_err() {
            break;
        }
        // A few presses in, confirm the knob is actually bound on this airframe.
        if i == 4 {
            let now = xp.get_float(HEADING_BUG).await.unwrap_or(start);
            if (now - start).abs() < 0.5 {
                break;
            }
        }
    }
    let now = xp.get_float(HEADING_BUG).await.unwrap_or(start);
    let err = (((leg - now) % 360.0 + 540.0) % 360.0 - 180.0).abs();
    if err > 1.5 {
        // Knob path didn't get there — direct write as the fallback (straight
        // to the FULL target: a dataref write is one round trip, no cap needed).
        let _ = xp.set_float(HEADING_BUG, target).await;
    }
    xp.get_float(HEADING_BUG).await.unwrap_or(target)
}

/// Read the servo status as a bool ("is the autopilot actually flying").
async fn servos_engaged(ctx: &ToolCtx<'_>) -> Result<bool> {
    Ok(ctx.xplane.get_float(SERVOS_ON).await? > 0.5)
}

/// Raise the flight director (if this airframe needs it) then engage the servos.
/// The C90's `fd_monitor` drops the servos unless the FD switch is up, so the FD
/// commands must fire first.
async fn engage_ap(ctx: &ToolCtx<'_>, cfg: &ApConfig) -> Result<()> {
    for c in &cfg.fd_commands {
        ctx.xplane.run_command(c).await?;
    }
    ctx.xplane.run_command(CMD_SERVOS_ON).await?;
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
    ctx.xplane.get_float(status).await.unwrap_or(0.0) > 0.5
}

/// Engage an autopilot mode *idempotently*. The `sim/autopilot/*` mode commands
/// are toggles, so pressing one whose mode is already active turns it back OFF —
/// which is why a second "set vertical speed" (or heading, or altitude) in a row
/// would silently drop the mode. Check the status first; only press when off.
async fn engage_mode(ctx: &ToolCtx<'_>, status: &str, command: &str) -> Result<()> {
    if !mode_engaged(ctx, status).await {
        ctx.xplane.run_command(command).await?;
    }
    Ok(())
}

/// Default climb/descent rate (fpm) for an altitude change with no rate given.
/// Gentle on purpose: a 172 climbs ~500-700 fpm flat-out, and the copilot isn't
/// managing power, so a high commanded VS just bleeds airspeed. 300 is safe for
/// both climb and descent; say an explicit rate ("climb to X at 700") for more.
const DEFAULT_VS_FPM: f64 = 300.0;
/// Max climb rate (fpm) the 172 sustains at full power without bleeding airspeed —
/// the cap when auto-scaling the rate to the altitude change. (Per-airframe later.)
const MAX_CLIMB_FPM: f64 = 700.0;
/// Max auto descent rate (fpm) commanded for an altitude change — gentle enough
/// not to build speed toward Vne without active power management. (Per-airframe.)
const MAX_DESCENT_FPM: f64 = 1000.0;
/// During a climb the watcher protects airspeed: if IAS bleeds below this (kt),
/// it reduces the commanded vertical speed (pitch down) so a too-steep climb can't
/// stall — power's already at the firewall, so climb rate is the lever to trade.
/// ~12 kt above the 172's clean stall; normal full-power climbs sit well above it.
const CLIMB_MIN_KTS: f64 = 60.0;
/// How much to ease (or restore) the climb rate per check, fpm.
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
fn auto_vs_fpm(delta_ft: f64, climb: bool) -> f64 {
    let cap = if climb {
        MAX_CLIMB_FPM
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
    if let Some(floor) = crate::procedure::terrain_floor_ahead_ft() {
        if target < floor {
            let ground = floor - crate::procedure::TERRAIN_CLEARANCE_FT;
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
    let floor = crate::procedure::terrain_floor_ahead_ft()?;
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
    if crate::procedure::take_altitude(altitude) {
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
            .unwrap_or_else(|| auto_vs_fpm(diff, diff > 0.0))
            .min(crate::procedure::vs_clamp_fpm());
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
    let (movement, signed, capture) = arm_altitude(ctx, altitude, rate).await?;
    // A climb/descent needs the watcher to level us off (and ease a climb throttle);
    // a hold is already done. The watcher calls out the level-off when it reaches
    // the target, so hand it a voice clone.
    if let Some(was_climb) = capture {
        capture_at_altitude(
            ctx.config.xplane_base_url.clone(),
            altitude,
            was_climb,
            ctx.voice.cloned(),
            ctx.status.cloned(),
        );
    }

    // Diagnostic: dump what the AP actually latched so a flight test shows the
    // truth (did VS engage? is the dial holding our rate? did ALT hold sneak on?).
    let vs_dial = ctx.xplane.get_float(VS_DIAL).await.unwrap_or(f64::NAN);
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
    Ok((
        movement,
        signed.abs(),
        servos_engaged(ctx).await?,
        movement.starts_with("Climbing"),
    ))
}

/// The synchronous part of [`fly_to`]: read the current altitude, infer the
/// direction, and arm the autopilot (altitude window + VS mode + rate, plus the
/// climb power firewall). Returns the movement phrase, the *signed* commanded fpm,
/// and `Some(was_climb)` if a capture watcher should be spawned (climb/descent) or
/// `None` for a hold. Split out from the threaded watcher so the trust-critical
/// writes are unit-testable against a fake sim.
async fn arm_altitude(
    ctx: &ToolCtx<'_>,
    altitude: f64,
    rate: Option<f64>,
) -> Result<(&'static str, f64, Option<bool>)> {
    let current = ctx.xplane.get_float(CURRENT_ALT).await.unwrap_or(altitude);
    let diff = altitude - current;
    ctx.xplane.run_command(CMD_SERVOS_ON).await?;

    if diff.abs() <= ALT_CAPTURE_BAND_FT {
        // HOLD: we're already at it. Capture the current altitude now — this is the
        // one case where `altitude_hold` is right (hold where you are).
        let hold = (current / 100.0).round() * 100.0;
        engage_mode(ctx, ALT_HOLD_STATUS, CMD_ALT_HOLD).await?;
        ctx.xplane.set_float(ALT_DIAL, hold).await?;
        return Ok(("Holding at", 0.0, None));
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
        .unwrap_or_else(|| auto_vs_fpm(diff, climb));
    let signed = if climb { mag } else { -mag };
    ctx.xplane.set_float(ALT_DIAL, altitude).await?;
    engage_mode(ctx, VS_STATUS, CMD_VS_MODE).await?;
    ctx.xplane.set_float(VS_DIAL, signed).await?;
    // A climb needs power — the AP pitches up to hold VS, and on cruise power a 172
    // just bleeds airspeed. Full throttle for the climb; the watcher eases it back
    // to cruise at capture. (Autothrottle will replace this later.)
    if climb {
        let _ = ctx.xplane.set_float(&throttle_dref(), 1.0).await;
    }
    Ok((
        if climb {
            "Climbing to"
        } else {
            "Descending to"
        },
        signed,
        Some(climb),
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

/// Generation stamp for capture watchers. Each climb/descent bumps this; a watcher
/// stands down as soon as it sees a newer generation, so at most one is ever live
/// no matter how many altitude changes are commanded in quick succession (the
/// `ALT_DIAL` stand-down only catches a *different* target — two commands to the
/// same altitude, or a re-command, would otherwise stack independent watchers).
static CLIMB_WATCHER_GEN: AtomicU64 = AtomicU64::new(0);

/// Fly the AP to a selected altitude in VS mode, then **level off** there.
///
/// The stock 172 AP does not reliably capture the preselected altitude from VS
/// mode on its own, so the copilot does it: a one-shot watcher polls until the
/// aircraft reaches `target`, then engages ALT hold (idempotently — a no-op if the
/// AP already captured) so it holds instead of flying straight through. For a
/// `was_climb` it also eases the firewalled throttle back to cruise.
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
) {
    let my_gen = CLIMB_WATCHER_GEN.fetch_add(1, Ordering::SeqCst) + 1;
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async move {
            let xp = XPlaneClient::new(base);
            // Wall-clock budget: even a long, slow change finishes inside 20 min; if
            // it somehow doesn't, we stop watching.
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20 * 60);
            // The commanded climb rate — the ceiling the airspeed protection won't
            // exceed when it restores the climb after backing off.
            let initial_vs = xp.get_float(VS_DIAL).await.unwrap_or(0.0);
            let mut eased_for_speed = false;
            while std::time::Instant::now() < deadline {
                // A newer command was issued — it owns the level-off now.
                if CLIMB_WATCHER_GEN.load(Ordering::SeqCst) != my_gen {
                    return;
                }
                // Need both the selected dial (did the pilot re-command?) and the
                // current altitude (are we there yet?) to decide; a dropped read just
                // retries next tick.
                let (Ok(dial), Ok(alt)) = (
                    xp.get_float(ALT_DIAL).await,
                    xp.get_float(CURRENT_ALT).await,
                ) else {
                    tokio::time::sleep(std::time::Duration::from_secs(3)).await;
                    continue;
                };
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
                        let _ = xp.set_float(ALT_DIAL, target).await;
                        let held = xp
                            .get_float(ALT_HOLD_STATUS)
                            .await
                            .map(|v| v > 0.5)
                            .unwrap_or(false);
                        if !held {
                            let _ = xp.run_command(CMD_ALT_HOLD).await;
                        }
                        // Ease a firewalled climb back to cruise — but only if the
                        // throttle is still where we left it (full). If the pilot took
                        // manual throttle mid-climb (which doesn't move the alt dial,
                        // so the stand-down never fired), it now reads below the floor —
                        // leave it rather than overriding their setting.
                        let mut eased = false;
                        if was_climb {
                            match xp.get_float(&throttle_dref()).await {
                                Ok(thr) if thr >= FIREWALL_HANDBACK_FLOOR => {
                                    let _ = xp.set_float(&throttle_dref(), CRUISE_THROTTLE).await;
                                    eased = true;
                                }
                                _ => {} // pilot took manual throttle — don't touch it
                            }
                        }
                        // Call out the capture. This lands minutes after the command,
                        // so it's the copilot telling you it actually levelled off.
                        // Spoken via Voice and mirrored to the panel (which the
                        // Reporter never sees for this background announcement).
                        let line = if eased {
                            format!(
                                "Reached {target:.0} feet, easing the power back, levelling off."
                            )
                        } else {
                            format!("Reached {target:.0} feet, levelling off.")
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
                        // Airspeed-protected climb: hold the fastest climb that keeps
                        // IAS above the floor. Below it, ease the vertical speed down
                        // (pitch down for speed); once speed's healthy again, restore
                        // the climb toward what was commanded. Power stays firewalled.
                        if was_climb && initial_vs > 0.0 {
                            if let Ok(ias) = xp.get_float(crate::datarefs::AIRSPEED_KTS).await {
                                let cur = xp.get_float(VS_DIAL).await.unwrap_or(initial_vs);
                                if ias < CLIMB_MIN_KTS {
                                    let new = (cur - VS_TRIM_FPM).max(0.0);
                                    if new < cur {
                                        let _ = xp.set_float(VS_DIAL, new).await;
                                        if !eased_for_speed {
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
                                } else if ias > CLIMB_MIN_KTS + 8.0 && cur < initial_vs {
                                    let new = (cur + VS_TRIM_FPM).min(initial_vs);
                                    let _ = xp.set_float(VS_DIAL, new).await;
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
        engage_mode(ctx, HEADING_STATUS, CMD_HEADING_MODE).await?;
        let actual = drive_heading_bug(ctx.xplane, heading).await;
        ctx.xplane.run_command(CMD_SERVOS_ON).await?;
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
fn spoken_heading(deg: f64) -> String {
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
        ctx.xplane.run_command(CMD_HSI_GPS).await?;
        engage_ap(ctx, &self.cfg).await?;
        engage_mode(ctx, GPSS_STATUS, CMD_GPSS).await?;
        let mut lnav = mode_engaged(ctx, GPSS_STATUS).await;
        if !lnav {
            engage_mode(ctx, NAV_STATUS, CMD_NAV_MODE).await?;
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
                "Roger — {power}{} {altitude:.0} feet.{terrain}",
                movement.to_lowercase()
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
        if crate::procedure::take_altitude(target) {
            return Ok(format!(
                "Roger, holding {target:.0} feet. You have the altitude; I'm still flying the track."
            ));
        }

        engage_ap(ctx, &self.cfg).await?;
        ctx.xplane.run_command(CMD_WING_LEVELER).await?;
        engage_mode(ctx, ALT_HOLD_STATUS, CMD_ALT_HOLD).await?;
        ctx.xplane.set_float(ALT_DIAL, target).await?;
        // Settle to cruise power. Intentional even though it overrides the pilot's
        // throttle: the only power that *holds* an altitude on a 172 is ~cruise, and
        // the pre-level-off setting is by definition not a hold power (full from a
        // climb would accelerate; idle from a descent would decay toward a stall).
        // Converging to cruise from either direction is the safe default until
        // airspeed-aware power management lands — making it conditional (leave the
        // pilot's setting) would risk that decay when levelling from a descent.
        let _ = ctx
            .xplane
            .set_float(&throttle_dref(), CRUISE_THROTTLE)
            .await;

        if servos_engaged(ctx).await? {
            Ok(format!(
                "Roger, wings level, cruise power, holding {target:.0} feet."
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
        engage_mode(ctx, VS_STATUS, CMD_VS_MODE).await?;
        ctx.xplane.set_float(VS_DIAL, fpm).await?;

        // Confirm the actual state, not the intent: both that the servos are
        // flying and that VS mode actually latched.
        let actual = ctx.xplane.get_float(VS_DIAL).await?;
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
            Ok(format!(
                "Roger — {power}{} {altitude:.0} feet at {fpm:.0} feet per minute, leveling off there.{terrain}",
                movement.to_lowercase()
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
        let was_flying = crate::tools::recover::abort() | crate::autoland::abort();
        if was_flying {
            return Ok("You have the aircraft.".to_string());
        }
        if !servos_engaged(ctx).await? {
            return Ok("Autopilot's already off.".to_string());
        }
        ctx.xplane.run_command(CMD_SERVOS_OFF).await?;
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
const AIRSPEED_DIAL: &str = "sim/cockpit2/autopilot/airspeed_dial_kts";
/// MD-82 autothrottle: a custom on/off switch (arm) driven by a toggle command;
/// engaging into the detent needs an EPR thrust rating selected then EPR_lim, which
/// clamps autothrottle_enabled to a holding mode (needs the flight director on).
const AT_SWITCH: &str = "laminar/md82/autopilot/autothrottle_switch";
const AT_ENABLED: &str = "sim/cockpit2/autopilot/autothrottle_enabled"; // 0 arm, 1 speed, 2 EPR
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
            let current = ctx.xplane.get_float(AIRSPEED_DIAL).await?;
            (current + d, String::new())
        } else {
            return Ok("Say a speed in knots, or 'speed for landing'.".to_string());
        };
        let kts = kts.clamp(80.0, 400.0);
        ctx.xplane.set_float(AIRSPEED_DIAL, kts).await?;
        let actual = ctx.xplane.get_float(AIRSPEED_DIAL).await.unwrap_or(kts);
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
        assert_eq!(auto_vs_fpm(300.0, true), DEFAULT_VS_FPM);
        // Mid change → scaled (≈ delta / 3).
        assert_eq!(auto_vs_fpm(1500.0, true), 500.0);
        // Big climb → capped at the airframe's sustainable climb rate, so it can't
        // outrun the available power.
        assert_eq!(auto_vs_fpm(9000.0, true), MAX_CLIMB_FPM);
        // Descents get their own (higher) cap.
        assert_eq!(auto_vs_fpm(9000.0, false), MAX_DESCENT_FPM);
        // Direction is by magnitude — sign of the delta doesn't matter here.
        assert_eq!(auto_vs_fpm(-1500.0, false), 500.0);
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
        let (movement, signed, capture) =
            arm_altitude(&ctx(&fake, &cfg), 7000.0, None).await.unwrap();

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
            fake.last_write(&throttle_dref()),
            Some(1.0),
            "firewalls the throttle for the climb"
        );
    }

    #[tokio::test]
    async fn arm_altitude_descent_does_not_touch_the_throttle() {
        let fake = FakeXPlane::with_reads(&[(CURRENT_ALT, 9000.0)]);
        let cfg = crate::config::Config::default();
        let (movement, signed, capture) =
            arm_altitude(&ctx(&fake, &cfg), 3000.0, None).await.unwrap();

        assert_eq!(movement, "Descending to");
        assert!(signed < 0.0, "descent VS should be negative, was {signed}");
        assert_eq!(
            capture,
            Some(false),
            "a descent still needs the level-off watcher"
        );
        assert_eq!(fake.last_write(VS_DIAL), Some(signed));
        assert!(
            !fake.wrote(&throttle_dref()),
            "descent must not firewall (or touch) the throttle"
        );
    }

    #[tokio::test]
    async fn arm_altitude_at_target_holds() {
        // Within the capture band of the current altitude → hold, no climb/descent.
        let fake = FakeXPlane::with_reads(&[(CURRENT_ALT, 5020.0)]);
        let cfg = crate::config::Config::default();
        let (movement, signed, capture) =
            arm_altitude(&ctx(&fake, &cfg), 5000.0, None).await.unwrap();

        assert_eq!(movement, "Holding at");
        assert_eq!(signed, 0.0);
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
