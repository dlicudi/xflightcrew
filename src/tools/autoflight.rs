//! The autopilot's own modes as tools (issue #92): the stock autothrottle,
//! level change (FLC / FLCH) and GPS steering (GPSS / LNAV), each engaged and
//! READ BACK against the intent, never assumed.
//!
//! The follower still flies the heading bug, the VS dial and its throttle law
//! (EGPF A330 2026-09-21: the stock autothrust sat unused). Nothing here is
//! wired into the follower or autoland: those files belong to other work.
//! This is the seam they adopt: [`crate::systems::ap_modes`] says whether an
//! airframe HAS a mode (a readback that proved it, else the profile's
//! `autoflight:` block, else the autopilot type from the systems survey), and
//! [`engage`] engages one and says whether it took. A follower that wants the
//! autothrottle asks `ap_modes().autothrust != ModeCap::No`, calls
//! `engage(xp, ApMode::Autothrust, true)`, and keeps its own throttle law
//! only when that returns `Engaged::No`.
//!
//! Level change and GPSS are TOGGLE commands: the status is read first and
//! the command fired only when it differs (a toggle fired on a mode already
//! in state turns it off). The autothrottle has real on/off commands. Every
//! button is the panel owner's ([`crate::autoflight::Button`]): this tool
//! decides, the owner presses.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::systems::{ApMode, ModeCap};

use crate::autoflight::{names, Button};

/// Reads 1 while the stock autothrottle is actually driving the levers.
const AT_ON: &str = names::AT_ON;
/// Speed on the elevator: 0 off, 1 armed, 2 captured. Level change engages
/// it (pitch for speed toward the altitude dial).
const SPEED_STATUS: &str = names::SPEED_STATUS;
/// GPS steering: 0 off, 1 armed, 2 captured.
const GPSS_STATUS: &str = names::GPSS_STATUS;
/// The servos: a pitch or roll mode needs the autopilot flying to mean
/// anything, and a failed readback without them proves nothing.
const SERVOS_ON: &str = names::SERVOS_ON;
#[cfg(test)]
const CMD_AT_ON: &str = crate::autoflight::cmd::AT_ON;
#[cfg(test)]
const CMD_AT_OFF: &str = crate::autoflight::cmd::AT_OFF;
#[cfg(test)]
const CMD_LEVEL_CHANGE: &str = crate::autoflight::cmd::LEVEL_CHANGE;
#[cfg(test)]
const CMD_GPSS: &str = crate::autoflight::cmd::GPSS;

/// One mode's switch: how it is read and pressed.
struct Switch {
    status: &'static str,
    on: Button,
    /// `None`: `on` is a toggle.
    off: Option<Button>,
    /// Needs the autopilot's servos in.
    needs_servos: bool,
}

fn switch(mode: ApMode) -> Switch {
    match mode {
        ApMode::Autothrust => Switch {
            status: AT_ON,
            on: Button::AutothrottleOn,
            off: Some(Button::AutothrottleOff),
            needs_servos: false,
        },
        ApMode::LevelChange => Switch {
            status: SPEED_STATUS,
            on: Button::LevelChange,
            off: None,
            needs_servos: true,
        },
        ApMode::Lnav => Switch {
            status: GPSS_STATUS,
            on: Button::Gpss,
            off: None,
            needs_servos: true,
        },
    }
}

/// What a crew calls it.
pub fn spoken(mode: ApMode) -> &'static str {
    match mode {
        ApMode::Autothrust => "autothrottle",
        ApMode::LevelChange => "level change",
        ApMode::Lnav => "GPS steering",
    }
}

/// Armed counts: a GPSS armed waiting for its course is the mode taken.
fn reads_on(v: f64) -> bool {
    v >= 0.5
}

/// The verdict of one engage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Engaged {
    /// Read back in the state asked.
    Yes,
    /// It was already there; nothing was fired.
    Already,
    /// Fired, and the readback disagrees.
    No,
    /// The status could not be read: no measurement.
    Unread,
    /// The mode needs the autopilot flying and it is not.
    NoServos,
}

/// How long the sim takes to show a mode after its command (a frame or two;
/// an add-on's mirror a frame behind that).
const SETTLE_MS: u64 = 400;

/// Engage (or drop) `mode` and prove it. Read first; fire only on a
/// difference; read back. A mode proved on is remembered for the airframe
/// (`systems::note_mode`); a failure is blackboxed and never unlearns one
/// (a failure is as often the levers or the dial as the airframe).
/// Blackbox `autoflight` `mode`.
pub async fn engage(xp: &dyn crate::xplane::XPlane, mode: ApMode, on: bool) -> Engaged {
    let sw = switch(mode);
    let Ok(before) = xp.get_float(sw.status).await else {
        return Engaged::Unread;
    };
    if reads_on(before) == on {
        return Engaged::Already;
    }
    if on && sw.needs_servos {
        match crate::autoflight::read_status(&*xp, SERVOS_ON).await {
            Ok(s) if s >= 0.5 => {}
            Ok(_) => return Engaged::NoServos,
            Err(_) => return Engaged::Unread,
        }
    }
    let button = if on { sw.on } else { sw.off.unwrap_or(sw.on) };
    let cmd = crate::autoflight::panel().command(button).to_string();
    if crate::autoflight::press(xp, button).await.is_err() {
        return Engaged::Unread;
    }
    tokio::time::sleep(std::time::Duration::from_millis(SETTLE_MS)).await;
    let after = xp.get_float(sw.status).await.ok();
    let verdict = match after {
        None => Engaged::Unread,
        Some(v) if reads_on(v) == on => Engaged::Yes,
        Some(_) => Engaged::No,
    };
    crate::blackbox::record(
        "autoflight",
        json!({
            "event": "mode", "mode": mode, "want": on, "cmd": cmd,
            "before": before, "after": after, "took": verdict == Engaged::Yes,
        }),
    );
    if on && verdict == Engaged::Yes {
        crate::systems::note_mode(mode, true);
    }
    verdict
}

/// The autothrottle refused while one of our loops flies the levers (it
/// would take it straight back out, `autoflight::ThrustWatch`). Pure.
pub fn autothrust_refused_line(owner: &str) -> String {
    format!("The {owner} is flying the thrust, so no autothrottle. Say my controls first.")
}

/// The spoken readback for one engage. Pure.
pub fn line(mode: ApMode, on: bool, verdict: Engaged) -> String {
    let name = spoken(mode);
    let cap = {
        let mut c = name.chars();
        c.next()
            .map(|f| f.to_uppercase().collect::<String>() + c.as_str())
            .unwrap_or_default()
    };
    let state = if on { "on" } else { "off" };
    match verdict {
        Engaged::Yes => format!("{cap} {state}."),
        Engaged::Already => format!("{cap} already {state}."),
        Engaged::No if on => format!(
            "{cap} didn't engage. {}",
            match mode {
                ApMode::Autothrust => "Check the autothrottle arm switch.",
                ApMode::LevelChange => "Set a new altitude on the dial first.",
                ApMode::Lnav => "Check there's a GPS course to follow.",
            }
        ),
        Engaged::No => format!("{cap} is still on."),
        Engaged::Unread => format!("I can't read the {name}."),
        Engaged::NoServos => format!("The autopilot isn't engaged, so no {name}."),
    }
}

pub struct ApModeTool;

#[derive(Deserialize)]
struct Args {
    /// autothrust | level_change | lnav
    mode: String,
    on: bool,
}

fn parse_mode(s: &str) -> Option<ApMode> {
    match s {
        "autothrust" => Some(ApMode::Autothrust),
        "level_change" => Some(ApMode::LevelChange),
        "lnav" => Some(ApMode::Lnav),
        _ => None,
    }
}

/// The tool's body with the capability given. Pure over the sim.
pub async fn run(xp: &dyn crate::xplane::XPlane, mode: ApMode, on: bool, cap: ModeCap) -> String {
    if on && cap == ModeCap::No {
        return format!("This autopilot has no {}.", spoken(mode));
    }
    line(mode, on, engage(xp, mode, on).await)
}

#[async_trait]
impl Tool for ApModeTool {
    fn name(&self) -> &str {
        "ap_mode"
    }

    fn description(&self) -> &str {
        "The autopilot's own modes: 'autothrottle on/off' (autothrust), 'level \
         change' / 'FLCH' / 'flight level change' (speed on the elevator to the \
         altitude dial), 'GPS steering' / 'GPSS' / 'LNAV' (follow the GPS course). \
         Read back; refused where this autopilot has no such mode."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "mode": {"type": "string", "enum": ["autothrust", "level_change", "lnav"]},
                "on": {"type": "boolean"}
            },
            "required": ["mode", "on"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: Args = serde_json::from_value(args.clone())?;
        let Some(mode) = parse_mode(&a.mode) else {
            return Ok("Autothrottle, level change or GPS steering?".to_string());
        };
        let modes = crate::systems::ap_modes();
        let cap = match mode {
            ApMode::Autothrust => modes.autothrust,
            ApMode::LevelChange => modes.level_change,
            ApMode::Lnav => modes.lnav,
        };
        // The thrust has one owner (`autoflight::ThrustWatch`): while one of
        // our loops flies the levers it would take this straight back out.
        if a.on && mode == ApMode::Autothrust {
            if let Some(owner) = crate::flight_state::crew_flying() {
                return Ok(autothrust_refused_line(owner.name()));
            }
        }
        let out = run(ctx.xplane, mode, a.on, cap).await;
        // Given to the autothrottle, the speed governor stands down: two
        // owners of one lever fight, and the governor would read the A/T's
        // lever as the pilot's hand.
        if a.on
            && mode == ApMode::Autothrust
            && out.ends_with(" on.")
            && crate::flight_state::release_speed_hold()
        {
            crate::blackbox::record(
                "autoflight",
                json!({"event": "governor_released", "for": "autothrottle"}),
            );
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xplane::FakeXPlane;

    /// The A330's stock autothrust (EGPF 2026-09-21, never used): on by its
    /// own command, read back on `autothrottle_on`, remembered as proved.
    #[tokio::test]
    async fn the_autothrottle_is_engaged_and_read_back() {
        let fake = FakeXPlane::with_reads(&[(AT_ON, 0.0)]);
        fake.set_on_command(CMD_AT_ON, AT_ON, 1.0);
        let out = run(&fake, ApMode::Autothrust, true, ModeCap::Yes).await;
        assert_eq!(out, "Autothrottle on.");
        assert!(fake.ran(CMD_AT_ON));
        // Off by its own command, never a toggle.
        fake.set_on_command(CMD_AT_OFF, AT_ON, 0.0);
        let out = run(&fake, ApMode::Autothrust, false, ModeCap::Yes).await;
        assert_eq!(out, "Autothrottle off.");
        assert!(fake.ran(CMD_AT_OFF));
    }

    /// A silently failed engage is never "Roger": the readback names it.
    #[tokio::test]
    async fn an_autothrottle_that_does_not_engage_is_said() {
        let fake = FakeXPlane::with_reads(&[(AT_ON, 0.0)]);
        let out = run(&fake, ApMode::Autothrust, true, ModeCap::Unknown).await;
        assert!(out.starts_with("Autothrottle didn't engage"), "{out}");
    }

    /// Level change and GPSS are toggles: a mode in state is confirmed and
    /// the toggle never fired (it would turn it off).
    #[tokio::test]
    async fn a_toggle_mode_is_read_before_it_is_fired() {
        let fake = FakeXPlane::with_reads(&[(SPEED_STATUS, 2.0), (SERVOS_ON, 1.0)]);
        let out = run(&fake, ApMode::LevelChange, true, ModeCap::Yes).await;
        assert_eq!(out, "Level change already on.");
        assert!(!fake.ran(CMD_LEVEL_CHANGE));
        let fake = FakeXPlane::with_reads(&[(GPSS_STATUS, 0.0), (SERVOS_ON, 1.0)]);
        fake.set_on_command(CMD_GPSS, GPSS_STATUS, 2.0);
        let out = run(&fake, ApMode::Lnav, true, ModeCap::Yes).await;
        assert_eq!(out, "GPS steering on.");
        assert_eq!(fake.ran_count(CMD_GPSS), 1);
    }

    /// A pitch or roll mode without the servos in is said, and nothing fired.
    #[tokio::test]
    async fn a_mode_without_the_autopilot_is_refused() {
        let fake = FakeXPlane::with_reads(&[(SPEED_STATUS, 0.0), (SERVOS_ON, 0.0)]);
        let out = run(&fake, ApMode::LevelChange, true, ModeCap::Yes).await;
        assert_eq!(out, "The autopilot isn't engaged, so no level change.");
        assert!(!fake.ran(CMD_LEVEL_CHANGE));
    }

    /// The KAP 140 has no level change: refused by the capability map
    /// before anything is pressed. A failed read is no measurement.
    #[tokio::test]
    async fn a_mode_the_autopilot_lacks_is_refused_and_unread_is_said() {
        let fake = FakeXPlane::with_reads(&[(SPEED_STATUS, 0.0), (SERVOS_ON, 1.0)]);
        let out = run(&fake, ApMode::LevelChange, true, ModeCap::No).await;
        assert_eq!(out, "This autopilot has no level change.");
        assert!(!fake.ran(CMD_LEVEL_CHANGE));
        let fake = FakeXPlane::new();
        assert_eq!(engage(&fake, ApMode::Lnav, true).await, Engaged::Unread);
    }
}
