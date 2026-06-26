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
const SERVOS_ON: &str = "sim/cockpit2/autopilot/servos_on";
/// Command: disconnect the autopilot. The stock 172's S-TEC-style AP has no
/// master switch and does not register `sim/autopilot/servos_off`. X-Plane
/// exposes `servos_off_any` ("disco servos, whichever side is active") and the
/// yoke red button `ap_disc_trim_interrupt`. We use `servos_off_any` — a clean
/// servos disconnect — rather than the yoke button, which also interrupts the
/// pitch-trim servo (an unwanted side effect for a voice "autopilot off").
const CMD_SERVOS_OFF: &str = "sim/autopilot/servos_off_any";
/// Command: engage heading-select (HDG) mode.
const CMD_HEADING_MODE: &str = "sim/autopilot/heading";
/// Command: engage altitude select/hold (ALT) mode.
const CMD_ALT_HOLD: &str = "sim/autopilot/altitude_hold";
/// Command: wings-level / roll hold — for "auto level" / "straight and level".
const CMD_WING_LEVELER: &str = "sim/autopilot/wing_leveler";
/// Current indicated altitude (feet), read to capture "hold this altitude".
const CURRENT_ALT: &str = "sim/cockpit2/gauges/indicators/altitude_ft_pilot";
/// Autopilot altitude targets below this (feet MSL) are refused — a 172's AP is
/// never commanded this low, so it's almost always a misheard/empty value (e.g.
/// "auto level" misread as altitude 0). The gate catches it before the AP flies
/// the aircraft into the ground.
const MIN_AP_ALTITUDE_FT: f64 = 500.0;
/// Command: engage vertical-speed (VS) mode.
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
/// Throttle (all engines), 0..1. A commanded climb sets this to full — the AP
/// only controls pitch, so without power a 172 trades airspeed for altitude.
const THROTTLE_ALL: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";
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
const HEADING_STATUS: &str = "sim/cockpit2/autopilot/heading_status";
const ALT_HOLD_STATUS: &str = "sim/cockpit2/autopilot/altitude_hold_status";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
/// Descents steeper than this (fpm) are confirmed — anything shallower, and all
/// climbs, execute immediately. A 172 descends ~500–1000 fpm routinely.
const VS_CONFIRM_DESCENT_FPM: f64 = -1500.0;

/// Read the servo status as a bool ("is the autopilot actually flying").
async fn servos_engaged(ctx: &ToolCtx<'_>) -> Result<bool> {
    Ok(ctx.xplane.get_float(SERVOS_ON).await? > 0.5)
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
/// Within this many feet of the target, treat it as "hold here" (no climb).
const ALT_CAPTURE_BAND_FT: f64 = 100.0;

/// Fly the autopilot to `altitude` at `rate` fpm and let it capture/level off.
/// On this airframe ALT hold alone won't *change* altitude — you must arm the
/// target AND command a vertical speed, so this sets both. Direction is inferred
/// from current altitude. Returns ("Climbing to" | "Descending to" | "Holding at",
/// servos_engaged).
async fn fly_to(ctx: &ToolCtx<'_>, altitude: f64, rate: f64) -> Result<(&'static str, bool)> {
    let current = ctx.xplane.get_float(CURRENT_ALT).await.unwrap_or(altitude);
    let diff = altitude - current;
    ctx.xplane.run_command(CMD_SERVOS_ON).await?;

    let (movement, signed) = if diff.abs() <= ALT_CAPTURE_BAND_FT {
        // HOLD: we're already at it. Capture the current altitude now — this is the
        // one case where `altitude_hold` is right (hold where you are).
        let hold = (current / 100.0).round() * 100.0;
        engage_mode(ctx, ALT_HOLD_STATUS, CMD_ALT_HOLD).await?;
        ctx.xplane.set_float(ALT_DIAL, hold).await?;
        ("Holding at", 0.0)
    } else {
        // CLIMB or DESCENT. Arm the target in the altitude window and fly to it in
        // VS mode — VS is the only *active* pitch mode; the window arms ALT capture
        // on its own and levels off when reached. Do NOT engage `altitude_hold`
        // here: that holds the CURRENT altitude and cancels the climb (the bug that
        // left the aircraft accelerating in level flight at full power). Set the VS
        // dial LAST — engaging VS snaps the dial to the live rate, so our target
        // must be written after.
        let signed = if diff > 0.0 { rate.abs() } else { -rate.abs() };
        ctx.xplane.set_float(ALT_DIAL, altitude).await?;
        engage_mode(ctx, VS_STATUS, CMD_VS_MODE).await?;
        ctx.xplane.set_float(VS_DIAL, signed).await?;
        // A climb needs power — the AP pitches up to hold VS, and on cruise power a
        // 172 just bleeds airspeed. Full throttle for the climb, then a watcher
        // eases it back to cruise once we level off. (Autothrottle will replace
        // this once we've confirmed the climb itself works.)
        if signed > 0.0 {
            let _ = ctx.xplane.set_float(THROTTLE_ALL, 1.0).await;
            ease_power_at_capture(ctx.config.xplane_base_url.clone(), altitude);
        }
        (if diff > 0.0 { "Climbing to" } else { "Descending to" }, signed)
    };

    // Diagnostic: dump what the AP actually latched so a flight test shows the
    // truth (did VS engage? is the dial holding our rate? did ALT hold sneak on?).
    let vs_dial = ctx.xplane.get_float(VS_DIAL).await.unwrap_or(f64::NAN);
    let vs_mode = mode_engaged(ctx, VS_STATUS).await;
    let alt_hold_mode = mode_engaged(ctx, ALT_HOLD_STATUS).await;
    tracing::info!(
        target: "x_copilot::autopilot",
        movement,
        commanded_fpm = signed,
        vs_dial,
        vs_mode,
        alt_hold_mode,
        current_alt = current,
        target_alt = altitude,
        "fly_to latched"
    );
    Ok((movement, servos_engaged(ctx).await?))
}

/// Generation stamp for climb watchers. Each new climb bumps this; a watcher
/// stands down as soon as it sees a newer generation, so at most one watcher is
/// ever live no matter how many climbs are commanded in quick succession (the
/// `ALT_DIAL` stand-down only catches a *different* target — two climbs to the
/// same altitude, or a re-command, would otherwise stack independent watchers).
static CLIMB_WATCHER_GEN: AtomicU64 = AtomicU64::new(0);

/// After a full-power climb the AP levels at the target on its own but leaves the
/// throttle on the stop — so the aircraft accelerates instead of settling. Spawn
/// a one-shot watcher (own thread + runtime + client, like the dashboard poller —
/// the session runs a current-thread runtime that's blocked on the audio listener
/// between commands, so a `tokio::spawn` task would be starved; the watcher needs
/// to poll independently) that waits for the capture, then eases the throttle to
/// cruise — what a copilot does once established. Self-terminating and guarded: it
/// stands down without touching the throttle if a newer climb supersedes it, if
/// the selector dial moves off `target`, or if the pilot has taken manual
/// throttle. Fire-and-forget; a dropped poll just means the next tick retries.
fn ease_power_at_capture(base: String, target: f64) {
    let my_gen = CLIMB_WATCHER_GEN.fetch_add(1, Ordering::SeqCst) + 1;
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else { return };
        rt.block_on(async move {
            let xp = XPlaneClient::new(base);
            // 400 ticks × 3 s ≈ 20 min — even a long climb at 300 fpm finishes
            // well inside this; if it doesn't, we simply stop watching.
            for _ in 0..400 {
                tokio::time::sleep(std::time::Duration::from_secs(3)).await;
                // A newer climb was commanded — it owns the throttle now. Stand
                // down so we don't ease back a climb that isn't ours.
                if CLIMB_WATCHER_GEN.load(Ordering::SeqCst) != my_gen {
                    return;
                }
                // Pilot changed plans? The selected altitude is no longer ours —
                // stand down so we don't throttle back the new climb.
                match xp.get_float(ALT_DIAL).await {
                    Ok(dial) if (dial - target).abs() > ALT_CAPTURE_BAND_FT => return,
                    Ok(_) => {}
                    Err(_) => continue, // transient; try again next tick
                }
                // Captured? Ease to cruise and we're done — but only if the throttle
                // is still where we left it (full). If the pilot has taken manual
                // throttle mid-climb (e.g. "throttle 80 percent", which doesn't move
                // the altitude dial so the stand-down above never fires), it now
                // reads below the firewall floor — leave it alone rather than
                // silently overriding their setting back to cruise.
                if let Ok(alt) = xp.get_float(CURRENT_ALT).await {
                    if (alt - target).abs() <= ALT_CAPTURE_BAND_FT {
                        match xp.get_float(THROTTLE_ALL).await {
                            Ok(thr) if thr >= FIREWALL_HANDBACK_FLOOR => {
                                let _ = xp.set_float(THROTTLE_ALL, CRUISE_THROTTLE).await;
                            }
                            _ => {} // pilot took manual throttle — don't touch it
                        }
                        return;
                    }
                }
            }
        });
    });
}

pub struct SetHeading;

#[derive(Deserialize)]
struct Args {
    /// Target magnetic heading in degrees.
    heading: f64,
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
                }
            },
            "required": ["heading"]
        })
    }

    // No confirmation: a misheard heading just turns the wrong way and is
    // trivially corrected — unlike altitude/VS (terrain) or engine cutoff.

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        // Normalize into 0..360 (so "370" → 10, "-10" → 350).
        let heading = args.heading.rem_euclid(360.0);

        // Engage the autopilot and mode FIRST, then write the bug LAST —
        // engaging a mode can snap the dial to a current value, so the target
        // must be the final write or it gets clobbered.
        ctx.xplane.run_command(CMD_SERVOS_ON).await?;
        engage_mode(ctx, HEADING_STATUS, CMD_HEADING_MODE).await?;
        ctx.xplane.set_float(HEADING_BUG, heading).await?;

        // Confirm the actual state, not the intent.
        let actual = ctx.xplane.get_float(HEADING_BUG).await?;
        if servos_engaged(ctx).await? {
            Ok(format!("Roger — autopilot engaged, heading mode, bug to {:03.0}.", actual))
        } else {
            Ok(format!(
                "Heading bug set to {:03.0}, but the autopilot didn't engage — check the AP master.",
                actual
            ))
        }
    }
}

pub struct SetAltitude;

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

    // No confirm: an altitude assignment is routine ("climb and maintain X"), the
    // change is gentle (default 300 fpm) and the AP captures, the MIN_AP_ALTITUDE
    // gate already refuses absurd/empty targets, and the readback ("descending to
    // 2000 feet") *is* the confirmation — a mishear is undone instantly with "level
    // off". A confirm on every level change is friction without payoff. The steep
    // case that *is* worth gating — a fast descent rate — still confirms in
    // `set_vertical_speed`/`set_climb`.

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: AltArgs = serde_json::from_value(args.clone())?;
        let altitude = args.altitude.max(0.0);

        // Safety gate: refuse an absurdly-low target rather than fly the aircraft
        // toward the ground. "auto level" with no number lands here as 0 feet —
        // refuse and point at the right tool.
        if altitude < MIN_AP_ALTITUDE_FT {
            return Ok(format!(
                "Negative — {altitude:.0} feet is too low for an altitude hold. Say a target like \
                 five thousand, or 'level off' to hold your current altitude."
            ));
        }

        // Fly to it: arm the altitude AND command a vertical speed (ALT hold alone
        // won't change altitude on this AP), then let the AP capture and level off.
        let (movement, engaged) = fly_to(ctx, altitude, DEFAULT_VS_FPM).await?;
        // fly_to firewalls the throttle on a climb; say so.
        let power = if movement.starts_with("Climbing") { "full power, " } else { "" };
        if engaged {
            Ok(format!("Roger — {power}{} {altitude:.0} feet.", movement.to_lowercase()))
        } else {
            Ok(format!(
                "Set to {} {altitude:.0} feet, but the autopilot didn't engage — check the AP master.",
                movement.to_lowercase()
            ))
        }
    }
}

/// `level_off` — capture the current altitude, level the wings, and hold. The
/// "auto level / straight and level" command, which has no target (it holds
/// where you are now), so it can't be expressed as `set_altitude`.
pub struct LevelOff;

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

        ctx.xplane.run_command(CMD_SERVOS_ON).await?;
        ctx.xplane.run_command(CMD_WING_LEVELER).await?;
        engage_mode(ctx, ALT_HOLD_STATUS, CMD_ALT_HOLD).await?;
        ctx.xplane.set_float(ALT_DIAL, target).await?;
        // Stopping a climb means coming off full power — ease to cruise so the
        // aircraft settles at this altitude instead of accelerating.
        let _ = ctx.xplane.set_float(THROTTLE_ALL, CRUISE_THROTTLE).await;

        if servos_engaged(ctx).await? {
            Ok(format!("Roger, wings level, cruise power, holding {target:.0} feet."))
        } else {
            Ok(format!(
                "Set to hold {target:.0} feet, but the autopilot didn't engage — check the AP master."
            ))
        }
    }
}

pub struct SetVerticalSpeed;

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
        ctx.xplane.run_command(CMD_SERVOS_ON).await?;
        engage_mode(ctx, VS_STATUS, CMD_VS_MODE).await?;
        ctx.xplane.set_float(VS_DIAL, fpm).await?;

        // Confirm the actual state, not the intent: both that the servos are
        // flying and that VS mode actually latched.
        let actual = ctx.xplane.get_float(VS_DIAL).await?;
        let phrase = describe_vs(actual);
        match (servos_engaged(ctx).await?, mode_engaged(ctx, VS_STATUS).await) {
            (true, true) => Ok(format!("Roger — autopilot engaged, vertical speed mode, {phrase}.")),
            (true, false) => Ok(format!("{phrase} dialed, but vertical speed mode didn't latch — check the AP.")),
            (false, _) => Ok(format!("{phrase} set, but the autopilot didn't engage — check the AP master.")),
        }
    }
}

/// `set_climb` — climb or descend to a target altitude at a chosen rate, then
/// level off. Sets the AP target altitude AND vertical speed and lets the
/// autopilot capture the altitude (it levels off on its own — no copilot polling
/// loop, no overshoot). Direction is inferred from target vs current altitude.
pub struct SetClimb;

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
        Some(format!("altitude {:.0} feet at {:.0} feet per minute", a.altitude.max(0.0), a.fpm.abs()))
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: ClimbArgs = serde_json::from_value(args.clone())?;
        let altitude = args.altitude.max(0.0);
        if altitude < MIN_AP_ALTITUDE_FT {
            return Ok(format!(
                "Negative — {altitude:.0} feet is too low for an altitude hold. Say a target like \
                 five thousand."
            ));
        }
        // A zero (or near-zero) rate would arm the target but command VS 0 and skip
        // the climb power firewall — a silent no-op that still reads back "climbing."
        // The schema permits fpm 0, so treat anything under 1 fpm as "unspecified"
        // and use the default rate, the same as a plain set_altitude.
        let rate = match args.fpm.abs() {
            r if r >= 1.0 => r.min(2000.0),
            _ => DEFAULT_VS_FPM,
        };
        let (movement, engaged) = fly_to(ctx, altitude, rate).await?;
        let power = if movement.starts_with("Climbing") { "full power, " } else { "" };
        if engaged {
            Ok(format!(
                "Roger — {power}{} {altitude:.0} feet at {rate:.0} feet per minute, leveling off there.",
                movement.to_lowercase()
            ))
        } else {
            Ok(format!(
                "Set to {} {altitude:.0} feet, but the autopilot didn't engage — check the AP master.",
                movement.to_lowercase()
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
