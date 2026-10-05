//! Radio tuning: `set_com1` (COM1 active frequency).
//!
//! X-Plane stores the frequency as an integer in kHz (8.33-capable), e.g.
//! 118075 = 118.075 MHz. We take MHz from the pilot and convert.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

const COM1: &str = "sim/cockpit2/radios/actuators/com1_frequency_hz_833";
const COM2: &str = "sim/cockpit2/radios/actuators/com2_frequency_hz_833";
const XPDR_CODE: &str = "sim/cockpit2/radios/actuators/transponder_code";
const XPDR_MODE: &str = crate::datarefs::TRANSPONDER_MODE;
/// NAV1 active frequency, integer in units of 10 kHz (10950 = 109.50 MHz).
const NAV1: &str = "sim/cockpit2/radios/actuators/nav1_frequency_hz";
const NAV1_OBS: &str = "sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot";
const CMD_COM1_FLIP: &str = "sim/radios/com1_standy_flip";
const CMD_COM2_FLIP: &str = "sim/radios/com2_standy_flip";
const CMD_XPDR_IDENT: &str = "sim/transponder/transponder_ident";

pub struct SetCom1;

#[derive(Deserialize)]
struct Args {
    /// Frequency in MHz (e.g. 118.5, 121.9).
    mhz: f64,
}

#[async_trait]
impl Tool for SetCom1 {
    fn name(&self) -> &str {
        "set_com1"
    }

    fn description(&self) -> &str {
        "Tune the COM1 radio to a frequency in megahertz (VHF comm band, \
         118.0–136.975). Use for 'com one 118.5', 'set com one to 121.9', \
         'tune com one one two one decimal niner'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "mhz": {
                    "type": "number",
                    "minimum": 118.0,
                    "maximum": 136.975,
                    "description": "COM frequency in MHz, e.g. 118.5"
                }
            },
            "required": ["mhz"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        let mhz = args.mhz.clamp(118.0, 136.975);
        let khz = (mhz * 1000.0).round() as i64;
        ctx.xplane.set_int(COM1, khz).await?;
        let actual_mhz = ctx.xplane.get_float(COM1).await? / 1000.0;
        Ok(format!("Roger, com one {actual_mhz:.3}."))
    }
}

pub struct SetTransponder;

#[derive(Deserialize)]
struct XpdrArgs {
    /// 4-digit octal squawk code (each digit 0–7), e.g. 1200.
    code: i64,
}

#[async_trait]
impl Tool for SetTransponder {
    fn name(&self) -> &str {
        "set_transponder"
    }

    fn description(&self) -> &str {
        "Set the transponder squawk code (4 digits, each 0-7). Use for 'squawk \
         1200', 'squawk 7000', 'set transponder to 4521', 'squawk VFR' (1200), \
         'set squawk 0021'. Common codes: VFR = 1200, emergency = 7700, lost \
         comms = 7600, hijack = 7500."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "code": {"type": "integer", "minimum": 0, "maximum": 7777,
                         "description": "4-digit octal squawk code, e.g. 1200"}
            },
            "required": ["code"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: XpdrArgs = serde_json::from_value(args.clone())?;
        let code = args.code;
        // Squawk codes are octal: 0000–7777, no digit above 7.
        let valid = (0..=7777).contains(&code)
            && format!("{:04}", code)
                .bytes()
                .all(|b| (b'0'..=b'7').contains(&b));
        if !valid {
            return Ok(format!(
                "{code:04} isn't a valid squawk — digits must be zero through seven."
            ));
        }
        ctx.xplane.set_int(XPDR_CODE, code).await?;
        let actual = ctx.xplane.get_float(XPDR_CODE).await? as i64;
        Ok(format!("Roger, squawking {actual:04}."))
    }
}

pub struct SetCom2;

#[async_trait]
impl Tool for SetCom2 {
    fn name(&self) -> &str {
        "set_com2"
    }

    fn description(&self) -> &str {
        "Tune the COM2 radio to a frequency in megahertz (118.0–136.975). Use for \
         'com two 121.9', 'set com two to 119.1'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "mhz": {"type": "number", "minimum": 118.0, "maximum": 136.975,
                        "description": "COM frequency in MHz, e.g. 121.9"}
            },
            "required": ["mhz"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        let mhz = args.mhz.clamp(118.0, 136.975);
        let khz = (mhz * 1000.0).round() as i64;
        ctx.xplane.set_int(COM2, khz).await?;
        let actual_mhz = ctx.xplane.get_float(COM2).await? / 1000.0;
        Ok(format!("Roger, com two {actual_mhz:.3}."))
    }
}

pub struct SwapRadio;

#[derive(Deserialize)]
struct SwapArgs {
    /// Which radio to flip active/standby: "com1" or "com2".
    radio: String,
}

#[async_trait]
impl Tool for SwapRadio {
    fn name(&self) -> &str {
        "swap_radio"
    }

    fn description(&self) -> &str {
        "Swap a COM radio's active and standby frequencies. Use for 'swap com one', \
         'flip com two', 'swap com one frequencies', 'make standby active on com \
         one', 'flip the frequencies', 'swap com radios'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "radio": {"type": "string", "enum": ["com1", "com2"],
                          "description": "Which COM radio to swap."}
            },
            "required": ["radio"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: SwapArgs = serde_json::from_value(args.clone())?;
        let (cmd, spoken) = match a.radio.to_ascii_lowercase().as_str() {
            "com1" => (CMD_COM1_FLIP, "com one"),
            "com2" => (CMD_COM2_FLIP, "com two"),
            other => {
                return Ok(format!(
                    "I didn't catch which radio '{other}'. Say com one or com two."
                ))
            }
        };
        ctx.xplane.run_command(cmd).await?;
        Ok(format!("Roger, swapped {spoken}."))
    }
}

pub struct SetNav1;

#[derive(Deserialize)]
struct NavArgs {
    /// Frequency in MHz (e.g. 109.50, 111.30).
    mhz: f64,
}

#[async_trait]
impl Tool for SetNav1 {
    fn name(&self) -> &str {
        "set_nav1"
    }

    fn description(&self) -> &str {
        "Tune the NAV1 radio (VOR/ILS) to a frequency in megahertz \
         (108.0–117.95). Use for 'nav one 109.5', 'set nav one to 111.30', \
         'tune nav one one zero eight decimal one'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "mhz": {"type": "number", "minimum": 108.0, "maximum": 117.95,
                        "description": "NAV frequency in MHz, e.g. 109.50"}
            },
            "required": ["mhz"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: NavArgs = serde_json::from_value(args.clone())?;
        let mhz = a.mhz.clamp(108.0, 117.95);
        // NAV freq dataref is in units of 10 kHz: 109.50 MHz -> 10950.
        let units = (mhz * 100.0).round() as i64;
        ctx.xplane.set_int(NAV1, units).await?;
        let actual = ctx.xplane.get_float(NAV1).await? / 100.0;
        Ok(format!("Roger, nav one {actual:.2}."))
    }
}

pub struct SetObs;

#[derive(Deserialize)]
struct ObsArgs {
    /// Course/radial in degrees (0–359).
    course: f64,
}

#[async_trait]
impl Tool for SetObs {
    fn name(&self) -> &str {
        "set_obs"
    }

    fn description(&self) -> &str {
        "Set the NAV1 OBS course (the selected VOR radial / course pointer) in \
         degrees. Use for 'set OBS 240', 'course 090', 'set the OBS to two four \
         zero', 'select the 180 radial', 'set the radial', 'course pointer to \
         090', 'OBS to 360'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "course": {"type": "integer", "minimum": 0, "maximum": 360,
                           "description": "Course in degrees magnetic (0-359)."}
            },
            "required": ["course"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: ObsArgs = serde_json::from_value(args.clone())?;
        let course = a.course.rem_euclid(360.0);
        ctx.xplane.set_float(NAV1_OBS, course).await?;
        let actual = ctx.xplane.get_float(NAV1_OBS).await?;
        Ok(format!("Roger, OBS {actual:03.0}."))
    }
}

/// A transponder mode the copilot switches to, decoupled from any one
/// airframe's encoding: [`set_mode`] maps it to the stock actuator AND, where
/// the airframe's box is an add-on's, to that add-on's own knob.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XpdrMode {
    Off,
    Standby,
    On,
    Altitude,
}

impl XpdrMode {
    /// The stock `transponder_mode` encoding.
    fn stock(self) -> i64 {
        match self {
            XpdrMode::Off => 0,
            XpdrMode::Standby => 1,
            XpdrMode::On => 2,
            XpdrMode::Altitude => 3,
        }
    }
    /// ON and ALTITUDE reply to radar; OFF and STANDBY do not.
    fn transmits(self) -> bool {
        matches!(self, XpdrMode::On | XpdrMode::Altitude)
    }
    fn spoken(self) -> &'static str {
        match self {
            XpdrMode::Off => "off",
            XpdrMode::Standby => "standby",
            XpdrMode::On => "on",
            XpdrMode::Altitude => "altitude",
        }
    }
    fn knob(self, k: &crate::aircraft::TransponderModeKnob) -> i64 {
        match self {
            XpdrMode::Off => k.off,
            XpdrMode::Standby => k.standby,
            XpdrMode::On => k.on,
            XpdrMode::Altitude => k.altitude,
        }
    }
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s.to_ascii_lowercase().as_str() {
            "off" => XpdrMode::Off,
            "standby" => XpdrMode::Standby,
            "on" => XpdrMode::On,
            "altitude" => XpdrMode::Altitude,
            _ => return None,
        })
    }
}

/// The ACTIVE airframe's mode selector, if its transponder is an add-on's own
/// (see [`crate::aircraft::TransponderModeKnob`]).
///
/// Process-wide, so it is republished on EVERY profile swap — including to
/// `None` — never merely set when an airframe happens to have one. A
/// published-once flag that outlives an airframe change is how a cold-and-dark
/// 172 silently killed a Twin Otter's radio for a whole session (2026-07-27);
/// the same trap here would aim every mode write at a knob the new aircraft
/// doesn't have. The snapshot's `mode_knob`.
///
/// Publish (or clear) the active airframe's transponder selector alone.
#[cfg(test)]
pub fn publish_mode_knob(knob: Option<crate::aircraft::TransponderModeKnob>) {
    crate::aircraft::update_airframe_facts(|a| a.mode_knob = knob);
}

/// Set the transponder mode on THIS airframe, and confirm it from what the
/// box actually reads back.
///
/// The stock actuator is always written: it is the real input on nearly every
/// aircraft, and where it isn't, writing it is harmless. Where the profile
/// names an add-on knob, that knob is written too and is what the readback
/// believes — on the Twin Otter the stock dataref is an OUTPUT the add-on
/// rewrites from the knob every frame, so reading it back confirms nothing but
/// the add-on's own opinion (and its ALT is stock mode 2, not 3).
///
/// Returns whether the box now reads what we asked for.
pub async fn set_mode(xp: &dyn crate::xplane::XPlane, mode: XpdrMode) -> bool {
    let knob = crate::aircraft::airframe_facts().mode_knob.clone();
    let stock_ok = xp.set_int(XPDR_MODE, mode.stock()).await.is_ok();
    let Some(k) = knob else {
        return stock_ok
            && xp
                .get_float(XPDR_MODE)
                .await
                .is_ok_and(|v| v.round() as i64 == mode.stock());
    };
    let want = mode.knob(&k);
    let knob_ok = if let (Some(up), Some(down)) = (&k.up_command, &k.down_command) {
        step_knob(xp, &k.dataref, want, up, down).await
    } else if xp.set_int(&k.dataref, want).await.is_err() {
        false
    } else {
        xp.get_float(&k.dataref)
            .await
            .is_ok_and(|v| v.round() as i64 == want)
    };
    if !knob_ok || k.also.is_empty() || !mode.transmits() {
        return knob_ok;
    }
    // The box has more knobs than the one that names the mode. Turn each,
    // then believe the STOCK mode: here it is the add-on's own verdict over
    // all of them, and a knob that reads right on a box in standby is the
    // readback that confirms the half it looked at.
    for c in &k.also {
        if !step_knob(xp, &c.dataref, c.want, &c.up_command, &c.down_command).await {
            return false;
        }
    }
    tokio::time::sleep(std::time::Duration::from_millis(150)).await;
    xp.get_float(XPDR_MODE)
        .await
        .is_ok_and(|v| stock_transmits(v.round() as i64))
}

/// What the one transponder check found and did: the code the tower
/// assigned (if any), whether we dialled it, whether the box READS it back,
/// and whether the mode is ALT by the airframe's own verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XpdrCheck {
    pub code: Option<i64>,
    pub code_dialled: bool,
    pub code_ok: bool,
    pub mode_ok: bool,
}

impl XpdrCheck {
    pub fn ok(&self) -> bool {
        self.code_ok && self.mode_ok
    }

    /// What the crew says when the check did not take, else `None`.
    pub fn failure_line(&self) -> Option<String> {
        match (self.code_ok, self.mode_ok) {
            (true, true) => None,
            (false, true) => Some(format!(
                "The transponder isn't showing squawk {}. Check the code.",
                self.code.map(|c| format!("{c:04}")).unwrap_or_default()
            )),
            (true, false) => {
                Some("The transponder didn't take altitude mode. Check the knob.".to_string())
            }
            (false, false) => Some(
                "The transponder didn't take the code or altitude mode. Check the box."
                    .to_string(),
            ),
        }
    }

    pub fn json(&self) -> Value {
        json!({
            "code": self.code, "code_dialled": self.code_dialled,
            "code_ok": self.code_ok, "mode_ok": self.mode_ok,
        })
    }
}

/// Dial `code` into the transponder and confirm it from what the box reads.
pub async fn set_code(xp: &dyn crate::xplane::XPlane, code: i64) -> bool {
    if xp.set_int(XPDR_CODE, code).await.is_err() {
        return false;
    }
    xp.get_float(XPDR_CODE)
        .await
        .is_ok_and(|v| v.round() as i64 == code)
}

/// The ONE transponder check before a departure call and at the roll: the
/// assigned code dialled where the box does not already show it and READ
/// BACK, then ALT through the airframe's own selector ([`set_mode`], never a
/// raw write). Two "negative radar contact, recycle transponder" calls went
/// unanswered on the EGPH sortie (2026-09-20): the spoken takeoff set the
/// mode at liftoff only and never looked at the code, the circuit likewise,
/// and only full auto dialled the squawk. With nothing assigned the code is
/// the pilot's and is left alone.
pub async fn departure_check(xp: &dyn crate::xplane::XPlane, assigned: Option<i64>) -> XpdrCheck {
    let (code_dialled, code_ok) = match assigned {
        None => (false, true),
        Some(code) => match xp.get_float(XPDR_CODE).await {
            Ok(v) if v.round() as i64 == code => (false, true),
            // A failed read is no measurement: dial it, and believe the
            // readback only.
            _ => (true, set_code(xp, code).await),
        },
    };
    let mode_ok = set_mode(xp, XpdrMode::Altitude).await;
    XpdrCheck {
        code: assigned,
        code_dialled,
        code_ok,
        mode_ok,
    }
}

/// "Recycle transponder": standby, the code dialled again, back to ALT, each
/// through the one writer and the result read back.
pub async fn recycle(xp: &dyn crate::xplane::XPlane, code: i64) -> XpdrCheck {
    let _ = set_mode(xp, XpdrMode::Standby).await;
    let code_ok = set_code(xp, code).await;
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    let mode_ok = set_mode(xp, XpdrMode::Altitude).await;
    XpdrCheck {
        code: Some(code),
        code_dialled: true,
        code_ok,
        mode_ok,
    }
}

/// The crew's answer to a controller's squawk nudge: "recycle transponder"
/// is recycled, "confirm squawk X" re-dialled (and the box to ALT), both
/// through the one check. Returns what was done, the flight deck's line and,
/// when the box took it, the radio readback ("squawking four five zero
/// three", a readback the station hears in silence; the watch's next tick
/// says "radar contact" from what the box now shows).
pub async fn answer_nudge(
    xp: &dyn crate::xplane::XPlane,
    code: i64,
    recycle_asked: bool,
) -> (XpdrCheck, String, Option<String>) {
    let check = if recycle_asked {
        recycle(xp, code).await
    } else {
        departure_check(xp, Some(code)).await
    };
    let digits = crate::atis::spoken_digits(&format!("{code:04}"));
    if let Some(line) = check.failure_line() {
        return (check, line, None);
    }
    let said = if recycle_asked {
        format!("Recycled the transponder, squawk {digits}.")
    } else {
        format!("Squawk {digits}, set.")
    };
    (check, said, Some(format!("squawking {digits}")))
}

/// Whether a stock `transponder_mode` is a box that replies: 2 ON, 3 ALT, 5
/// ground mode S, 6 TA, 7 TA/RA. 0 off, 1 standby and 4 test are not.
pub fn stock_transmits(mode: i64) -> bool {
    matches!(mode, 2 | 3 | 5 | 6 | 7)
}

/// The most a stepped knob is turned for one request. No selector here has
/// more positions than this, and a knob that ignores its commands must not
/// be hammered.
const KNOB_MAX_STEPS: usize = 6;

/// Which way a stepped knob turns next: `Some(true)` up, `Some(false)` down,
/// `None` when it already reads what was asked.
pub fn knob_step_up(at: i64, want: i64) -> Option<bool> {
    (at != want).then_some(at < want)
}

/// Turn a READ-ONLY knob by its commands: read, step once toward `want`, let
/// the add-on move it, read again. A failed read ends it (no measurement, no
/// step); the answer is what the knob READS at the end, never the intent.
pub(crate) async fn step_knob(
    xp: &dyn crate::xplane::XPlane,
    dataref: &str,
    want: i64,
    up: &str,
    down: &str,
) -> bool {
    for _ in 0..KNOB_MAX_STEPS {
        let Ok(at) = xp.get_float(dataref).await else {
            return false;
        };
        let Some(go_up) = knob_step_up(at.round() as i64, want) else {
            return true;
        };
        if xp.run_command(if go_up { up } else { down }).await.is_err() {
            return false;
        }
        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
    }
    xp.get_float(dataref)
        .await
        .is_ok_and(|v| v.round() as i64 == want)
}

pub struct SetTransponderMode;

#[derive(Deserialize)]
struct ModeArgs {
    /// off, standby, on, altitude, or ident.
    mode: String,
}

#[async_trait]
impl Tool for SetTransponderMode {
    fn name(&self) -> &str {
        "set_transponder_mode"
    }

    fn description(&self) -> &str {
        "Set the transponder mode (off, standby, on, altitude) or send ident. Use \
         for 'transponder standby', 'squawk altitude', 'squawk mode charlie', \
         'transponder on', 'ident', 'squawk ident', 'transponder off', \
         'go to standby', 'mode C on'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "mode": {"type": "string", "enum": ["off", "standby", "on", "altitude", "ident"],
                         "description": "Transponder mode, or 'ident' to send a squawk ident."}
            },
            "required": ["mode"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: ModeArgs = serde_json::from_value(args.clone())?;
        // 'ident' is momentary — it presses the ident button, leaving the mode as-is.
        if a.mode.eq_ignore_ascii_case("ident") {
            ctx.xplane.run_command(CMD_XPDR_IDENT).await?;
            return Ok("Roger, ident.".to_string());
        }
        let Some(mode) = XpdrMode::parse(&a.mode) else {
            return Ok(format!("I didn't catch the transponder mode '{}'.", a.mode));
        };
        // Confirm the ACTION, not the intent: on an airframe whose box is an
        // add-on's, the write can land on a dataref nothing reads.
        Ok(if set_mode(ctx.xplane, mode).await {
            format!("Roger, transponder {}.", mode.spoken())
        } else {
            format!(
                "I set the transponder to {}, but it's not reading back. Check the box.",
                mode.spoken()
            )
        })
    }
}

#[cfg(test)]
mod tests {
    // The serialising lock below is deliberately held across awaits.
    #![allow(clippy::await_holding_lock)]

    use super::*;
    use crate::xplane::FakeXPlane;

    /// The published selector is process-wide (one aircraft is loaded at a
    /// time), so these tests take turns with it. Held across the awaits on
    /// purpose — that is the point of the lock, and each test drives its own
    /// current-thread runtime, so nothing can deadlock behind it.
    static KNOB_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// The Twin Otter's KT73 is the add-on's own box: master.lua rewrites the
    /// stock `transponder_mode` from `TO/radios/KT73SelKnob` every frame, so a
    /// stock write is a no-op that snaps back and the copilot's standby/ALT
    /// switching did nothing at all (2026-07-27). The mode write must turn the
    /// KNOB, and the readback must believe the knob — the airframe's ALT is
    /// knob 5 driving stock mode 2, so a stock readback would call the right
    /// answer wrong.
    #[tokio::test]
    async fn an_add_on_transponder_gets_its_own_knob_turned() {
        let _g = KNOB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let p = crate::aircraft::load("dhc6_twin_otter").expect("twotter profile loads");
        let knob = p
            .transponder_mode_knob
            .clone()
            .expect("the KT73 selector is wired in the profile");
        publish_mode_knob(Some(knob.clone()));

        // The box comes back reading ALT (knob 5).
        let fake = FakeXPlane::with_reads(&[(knob.dataref.as_str(), 5.0)]);
        assert!(set_mode(&fake, XpdrMode::Altitude).await);
        assert_eq!(fake.last_write(&knob.dataref), Some(5.0), "the knob turned");
        // The stock actuator is still written — harmless here, and it is the
        // real input on every airframe without an add-on box.
        assert_eq!(fake.last_write(XPDR_MODE), Some(3.0));

        // A knob that doesn't move is reported, not assumed: the copilot
        // confirms the ACTION, not the intent.
        let stuck = FakeXPlane::with_reads(&[(knob.dataref.as_str(), 1.0)]);
        assert!(!set_mode(&stuck, XpdrMode::Altitude).await);

        publish_mode_knob(None);
    }

    /// EGPH 2026-09-20: two "negative radar contact, recycle transponder"
    /// went unanswered, and only full auto ever looked at the code. The one
    /// check dials an assigned code the box does not show and believes only
    /// the readback; a code already showing is left alone; nothing assigned
    /// leaves the pilot's code alone; the mode goes to ALT every time.
    #[tokio::test]
    async fn the_departure_check_sets_the_code_and_altitude_and_reads_them_back() {
        let _g = KNOB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish_mode_knob(None);

        // Already squawking the assigned code, box in standby: ALT only.
        let set = FakeXPlane::with_reads(&[(XPDR_CODE, 4503.0), (XPDR_MODE, 3.0)]);
        let c = departure_check(&set, Some(4503)).await;
        assert!(c.ok() && !c.code_dialled, "{c:?}");
        assert!(!set.wrote(XPDR_CODE), "a code that shows is never rewritten");
        assert_eq!(set.last_write(XPDR_MODE), Some(3.0));
        assert_eq!(c.failure_line(), None);

        // Still on 7000 and the box never takes the new code: dialled, and
        // the failure is said with the code.
        let stuck = FakeXPlane::with_reads(&[(XPDR_CODE, 7000.0), (XPDR_MODE, 3.0)]);
        let c = departure_check(&stuck, Some(4503)).await;
        assert!(c.code_dialled && !c.code_ok && c.mode_ok, "{c:?}");
        assert_eq!(stuck.last_write(XPDR_CODE), Some(4503.0));
        let line = c.failure_line().expect("said");
        assert!(line.contains("4503"), "{line}");

        // Nothing assigned: the pilot's code stands, the mode still goes up.
        let vfr = FakeXPlane::with_reads(&[(XPDR_CODE, 7000.0), (XPDR_MODE, 1.0)]);
        let c = departure_check(&vfr, None).await;
        assert!(!vfr.wrote(XPDR_CODE));
        assert!(c.code_ok && !c.mode_ok, "a box that stays in standby is reported: {c:?}");
        assert!(c.failure_line().unwrap().contains("altitude mode"));
    }

    /// "Recycle transponder" is recycled: standby, the code again, ALT, read
    /// back, and the words for both channels. The radio line is a READBACK
    /// the station hears in silence (never a "say again"); the watch then
    /// sees the code replying and says "radar contact".
    #[tokio::test]
    async fn the_crew_recycles_when_the_controller_asks() {
        let _g = KNOB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish_mode_knob(None);
        let fake = FakeXPlane::with_reads(&[(XPDR_CODE, 4503.0), (XPDR_MODE, 3.0)]);
        let (c, said, radio) = answer_nudge(&fake, 4503, true).await;
        assert!(c.ok(), "{c:?}");
        assert_eq!(fake.writes_to(XPDR_MODE), vec![1.0, 3.0], "standby, then altitude");
        assert_eq!(fake.writes_to(XPDR_CODE), vec![4503.0], "the code dialled again");
        assert!(said.starts_with("Recycled the transponder"), "{said}");
        let radio = radio.expect("read back on the radio");
        assert_eq!(radio, "squawking four five zero three");
        assert!(crate::tower::is_readback(&radio));

        // "Confirm squawk" re-dials; a box that does not take it is said on
        // the flight deck and never read back as set.
        let stuck = FakeXPlane::with_reads(&[(XPDR_CODE, 7000.0), (XPDR_MODE, 3.0)]);
        let (c, said, radio) = answer_nudge(&stuck, 4503, false).await;
        assert!(!c.code_ok);
        assert!(said.contains("isn't showing squawk 4503"), "{said}");
        assert!(radio.is_none());
    }

    /// The A330's knob dataref is read-only (probed live, 2026-09-21): the
    /// knob is turned by its commands, read before every step, and the
    /// dataref is never written. AUTO is the SOP position and the profile's
    /// "altitude".
    #[tokio::test]
    async fn a_read_only_knob_is_stepped_by_its_commands() {
        let _g = KNOB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let p = crate::aircraft::load("airbus_a330_300").expect("A330 profile loads");
        let knob = p
            .transponder_mode_knob
            .clone()
            .expect("the knob is declared");
        let up = knob.up_command.clone().expect("stepped by command");
        let down = knob.down_command.clone().expect("stepped by command");
        publish_mode_knob(Some(knob.clone()));

        assert_eq!(knob_step_up(-1, 0), Some(true));
        assert_eq!(knob_step_up(1, 0), Some(false));
        assert_eq!(knob_step_up(0, 0), None);

        // Already at AUTO, its companions set, the box replying: confirmed,
        // nothing fired.
        let (alt_rpt, ta_ra) = (&knob.also[0], &knob.also[1]);
        let set = FakeXPlane::with_reads(&[
            (knob.dataref.as_str(), 0.0),
            (alt_rpt.dataref.as_str(), 1.0),
            (ta_ra.dataref.as_str(), 2.0),
            (XPDR_MODE, 7.0),
        ]);
        assert!(set_mode(&set, XpdrMode::Altitude).await);
        assert!(!set.ran(&ta_ra.up_command) && !set.ran(&alt_rpt.up_command));
        assert!(!set.ran(&up) && !set.ran(&down));
        assert!(
            !set.wrote(&knob.dataref),
            "a read-only dataref is never written"
        );

        // In standby, and a knob that never moves: stepped UP, bounded, and
        // reported as not set.
        let stuck = FakeXPlane::with_reads(&[(knob.dataref.as_str(), -1.0)]);
        assert!(!set_mode(&stuck, XpdrMode::Altitude).await);
        assert_eq!(stuck.ran_count(&up), KNOB_MAX_STEPS);
        assert!(!stuck.ran(&down));
        assert!(!stuck.wrote(&knob.dataref));

        publish_mode_knob(None);
    }

    /// EGPF, 2026-09-21, read live in Glasgow's zone: mode knob AUTO, the TCAS
    /// knob still in STBY where the aircraft loads, stock mode 1. The takeoff
    /// had blackboxed `transponder_alt ok: true` from the one knob it read.
    /// The companions are turned, and the verdict is the box's own.
    #[tokio::test]
    async fn auto_with_tcas_in_standby_is_not_altitude() {
        let _g = KNOB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let p = crate::aircraft::load("airbus_a330_300").expect("A330 profile loads");
        let knob = p
            .transponder_mode_knob
            .clone()
            .expect("the knob is declared");
        let ta_ra = knob
            .also
            .iter()
            .find(|c| c.dataref.ends_with("ta_ra_knob_pos"))
            .expect("the TCAS knob is a companion");
        assert_eq!(ta_ra.want, 2, "TA/RA");
        publish_mode_knob(Some(knob.clone()));

        // As found: the TCAS knob is asked for, and while the box still reads
        // standby the answer is no.
        let found = FakeXPlane::with_reads(&[
            (knob.dataref.as_str(), 0.0),
            (knob.also[0].dataref.as_str(), 1.0),
            (ta_ra.dataref.as_str(), 0.0),
            (XPDR_MODE, 1.0),
        ]);
        assert!(!set_mode(&found, XpdrMode::Altitude).await);
        assert!(found.ran(&ta_ra.up_command), "the TCAS knob is turned");
        assert!(!found.wrote(&ta_ra.dataref), "read-only, never written");

        // Every knob right and the box still in standby (no AC power): no.
        let dead = FakeXPlane::with_reads(&[
            (knob.dataref.as_str(), 0.0),
            (knob.also[0].dataref.as_str(), 1.0),
            (ta_ra.dataref.as_str(), 2.0),
            (XPDR_MODE, 1.0),
        ]);
        assert!(!set_mode(&dead, XpdrMode::Altitude).await);

        // Standby is the mode knob alone: the companions stay where they are.
        let stby = FakeXPlane::with_reads(&[(knob.dataref.as_str(), -1.0)]);
        assert!(set_mode(&stby, XpdrMode::Standby).await);
        assert!(!stby.ran(&ta_ra.up_command) && !stby.ran(&ta_ra.down_command));

        assert!(stock_transmits(7) && stock_transmits(5) && stock_transmits(3));
        assert!(!stock_transmits(0) && !stock_transmits(1) && !stock_transmits(4));
        publish_mode_knob(None);
    }

    /// With no add-on box the stock actuator IS the input, and nothing else is
    /// touched.
    #[tokio::test]
    async fn a_stock_transponder_is_written_directly() {
        let _g = KNOB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish_mode_knob(None);
        let fake = FakeXPlane::with_reads(&[(XPDR_MODE, 1.0)]);
        assert!(set_mode(&fake, XpdrMode::Standby).await);
        assert_eq!(fake.last_write(XPDR_MODE), Some(1.0));
        assert!(!fake.wrote("TO/radios/KT73SelKnob"));
    }

    /// The selector is republished on every profile swap, so it can never
    /// outlive its airframe: swapping off the Twin Otter must not leave the
    /// next aircraft's mode writes aimed at a knob it doesn't have (the shape
    /// of the radio-power flag that latched across an airframe change,
    /// 2026-07-27).
    #[tokio::test]
    async fn the_selector_cannot_outlive_its_airframe() {
        let _g = KNOB_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let p = crate::aircraft::load("dhc6_twin_otter").expect("twotter profile loads");
        publish_mode_knob(p.transponder_mode_knob.clone());
        // ...and now a 172, which has no add-on box.
        let c172 = crate::aircraft::load("cessna_172sp").expect("172 profile loads");
        assert!(c172.transponder_mode_knob.is_none());
        publish_mode_knob(c172.transponder_mode_knob.clone());

        let fake = FakeXPlane::with_reads(&[(XPDR_MODE, 3.0)]);
        assert!(set_mode(&fake, XpdrMode::Altitude).await);
        assert!(
            !fake.wrote("TO/radios/KT73SelKnob"),
            "the Otter's knob must not follow us to the next aircraft"
        );
    }
}
