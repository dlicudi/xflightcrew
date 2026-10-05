//! The ONE owner of the airframe's autoflight panel: the servos (CMD), the
//! lateral and vertical modes (HDG, VS, ALT HOLD, LVL CHG, APP, NAV, GPSS),
//! the dials (heading bug, altitude window, VS wheel, speed window), the
//! autothrottle and the flight director. Nothing else in the tree names an
//! autopilot command or writes an autopilot dial (`every_mcp_write_goes_through_the_owner`
//! scans the source).
//!
//! # Why (LEMG 737, 2026-10-05, sortie 1791187965-full_auto)
//!
//! Eleven files pressed `sim/autopilot/*` themselves, each with its own idea
//! of the panel: autoland assumed the autothrottle was off and the window
//! harmless, and left both to whoever had touched them last. The A/T sat in
//! SPEED 230 from a manual recovery and overrode every lever autoland wrote
//! (13:02 to 13:09, `ap_state` 51: the 1 bit). The window parked 1,000 ft
//! ABOVE held the 737 level against a -570 dial (12:25): this MCP flies a
//! V/S only toward its window. And the coupled ILS (#81) failed 43 of 46
//! times because autoland's own housekeeping pressed HDG and VS the moment
//! the 737's APP captured: `ap_state` went 50 (HDG, VS, ALT arm) to 1584 or
//! 2560 (LOC captured, GS armed or captured) within a second of the press and
//! back to 50 or 1074 at the next tick (blackbox 2026-09-26 to 09-28), while
//! `approach_status`, the read the coupling trusted, still said 0.
//!
//! # The model
//!
//! - **One truth for the modes**: X-Plane's `autopilot_state` bitfield
//!   ([`Modes`]), which every stock autopilot and the Laminar 737's xlua MCP
//!   (its `*_status` lights are computed FROM the stock modes) keep. The
//!   per-mode `*_status` reads stay for the loops' frames; the bitfield is
//!   what a coupling, a takeover and the blackbox believe.
//! - **The panel is the airframe's** ([`Panel`], from the profile's
//!   `autoflight:` block, else the stock commands): the 737's own CMD A,
//!   HDG SEL, V/S, ALT HLD and APP buttons, which keep its lamps and FMA
//!   honest. CMD B is never pressed (it dropped CMD A live, 2026-10-05).
//! - **Intent, not buttons** ([`Want`], [`reconcile`]): a loop says what it
//!   wants the autopilot to fly (its heading, its VS, the airframe's own
//!   capture); the owner turns the modes READ into the presses needed, only on
//!   a difference, and never over a capture the intent hands to the airframe
//!   (no HDG over an armed or captured localiser, no VS over a captured
//!   glideslope). A failed read is no measurement: nothing is pressed on it.
//! - **The window is parked where no capture can take the wheel**
//!   ([`WindowRule`]): above the aircraft everywhere (an altitude capture
//!   below would take a descent), and on an MCP that flies a V/S only TOWARD
//!   its window (`vs_toward_window`, the 737) at the field, which is toward
//!   every descent of an approach and under every level it flies; its
//!   go-around climbs flew away from it (2026-10-05). Subsumes 89fe7a6's
//!   `alt_dial_park_field`.
//! - **The thrust has one owner** ([`ThrustWatch`], [`take_thrust`]): a loop
//!   that flies the levers (autoland, the speed governor, the takeoff) takes
//!   an ENGAGED autothrottle out to ARM (`autothrottle_off`: "all modes off,
//!   remain armed"), reads it back, retries a bounded number of times, says
//!   it once, and says so if it will not come out. Armed is never touched:
//!   that is the takeoff configuration's.
//! - Every press, take and window move is blackboxed `autoflight`.
//!
//! # Primitives for other loops (the safe-hold and recovery work)
//!
//! [`engage_confirmed`] (servos in and READ in), [`hold_level`] (window at
//! the level, ALT HOLD, read back) and [`hold_heading`] (bug, HDG SEL, read
//! back): the airframe's own buttons, read before and after, bounded.

use std::time::Duration;

use serde_json::json;

use crate::xplane::XPlane;

/// Status and state names: READS. Anyone may read them; the dials are read
/// through [`Dial::read_name`] and written only here.
pub mod names {
    /// X-Plane's mode bitfield ([`super::Modes`]).
    pub const AP_STATE: &str = "sim/cockpit/autopilot/autopilot_state";
    pub const SERVOS_ON: &str = crate::datarefs::SERVOS_ON;
    /// The autopilot master as the sim has it (power and failures included).
    pub const AP_ON: &str = "sim/cockpit2/autopilot/autopilot_on";
    pub const AP_POWER: &str = "sim/cockpit2/autopilot/autopilot_has_power";
    pub const HDG_STATUS: &str = crate::datarefs::HEADING_STATUS;
    pub const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
    pub const ALT_HOLD_STATUS: &str = "sim/cockpit2/autopilot/altitude_hold_status";
    pub const NAV_STATUS: &str = crate::datarefs::NAV_STATUS;
    pub const GPSS_STATUS: &str = "sim/cockpit2/autopilot/gpss_status";
    /// Speed on the elevator: 0 off, 1 armed, 2 captured.
    pub const SPEED_STATUS: &str = "sim/cockpit2/autopilot/speed_status";
    pub const ALT_MODE: &str = "sim/cockpit2/autopilot/altitude_mode";
    /// 1 while the autothrottle is really driving the levers.
    pub const AT_ON: &str = "sim/cockpit2/autopilot/autothrottle_on";
    /// -1 hard off, 0 armed, 1 speed, 2 N1, 3 retard.
    pub const AT_MODE: &str = "sim/cockpit2/autopilot/autothrottle_enabled";
    pub const FD_MODE: &str = "sim/cockpit2/autopilot/flight_director_mode";
    pub const HOVER_STATUS: &str = "sim/cockpit2/autopilot/hover_status";
}

/// The stock commands, the panel's defaults. Crate-visible for the tests
/// that assert what was pressed; a press goes through [`press`].
pub(crate) mod cmd {
    pub const SERVOS_ON: &str = "sim/autopilot/servos_on";
    pub const SERVOS_OFF_ANY: &str = "sim/autopilot/servos_off_any";
    pub const SERVOS_FDIR_OFF: &str = "sim/autopilot/servos_fdir_off";
    pub const HEADING: &str = "sim/autopilot/heading";
    pub const VERTICAL_SPEED: &str = "sim/autopilot/vertical_speed";
    pub const ALTITUDE_HOLD: &str = "sim/autopilot/altitude_hold";
    pub const LEVEL_CHANGE: &str = "sim/autopilot/level_change";
    pub const APPROACH: &str = "sim/autopilot/approach";
    pub const NAV: &str = "sim/autopilot/NAV";
    pub const GPSS: &str = "sim/autopilot/gpss";
    pub const WING_LEVELER: &str = "sim/autopilot/wing_leveler";
    pub const HSI_NAV1: &str = "sim/autopilot/hsi_select_nav_1";
    pub const HSI_GPS: &str = "sim/autopilot/hsi_select_gps";
    pub const HEADING_UP: &str = "sim/autopilot/heading_up";
    pub const HEADING_DOWN: &str = "sim/autopilot/heading_down";
    pub const HOVER: &str = "sim/autopilot/hover";
    pub const FD_ON: &str = "sim/autopilot/fdir_on";
    pub const AT_ON: &str = "sim/autopilot/autothrottle_on";
    pub const AT_OFF: &str = "sim/autopilot/autothrottle_off";
    pub const AT_ARM: &str = "sim/autopilot/autothrottle_arm";
}

/// A button on the panel. Its command is the airframe's ([`Panel`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Button {
    /// CMD A / the servos. Not a toggle.
    Engage,
    /// Servos out, whichever side is active.
    Disconnect,
    /// Servos and flight director out (the wheels' disconnect, the hover's).
    DisconnectFd,
    /// HDG SEL. A toggle on most panels: pressed only on a difference.
    Heading,
    /// V/S, at the current rate (the dial is written after it).
    VerticalSpeed,
    AltitudeHold,
    LevelChange,
    /// APP: localiser and glideslope armed. A toggle.
    Approach,
    /// VOR/LOC.
    Nav,
    Gpss,
    WingLeveler,
    HsiNav1,
    HsiGps,
    HeadingUp,
    HeadingDown,
    Hover,
    FlightDirectorOn,
    AutothrottleOn,
    /// All autothrottle modes off, armed kept.
    AutothrottleOff,
    AutothrottleArm,
}

/// A dial: written only through [`set_dial`] / [`stage_dial`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Dial {
    /// The pilot's heading bug, degrees magnetic.
    HeadingBug,
    /// The altitude window, feet.
    Altitude,
    /// The VS wheel as the autopilot follows it (`vertical_velocity`; the
    /// SR22's Perspective ignores cockpit2's `vvi_dial_fpm`).
    Vs,
    /// cockpit2's VS dial (the helicopter route writes it).
    VviFpm,
    /// The speed window, knots or Mach.
    Airspeed,
    /// The speed window in knots.
    AirspeedKts,
    /// The hover mode's groundspeed vector (an array).
    HoverVector,
}

impl Dial {
    const fn name(self) -> &'static str {
        match self {
            Dial::HeadingBug => "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot",
            Dial::Altitude => "sim/cockpit2/autopilot/altitude_dial_ft",
            Dial::Vs => "sim/cockpit/autopilot/vertical_velocity",
            Dial::VviFpm => "sim/cockpit2/autopilot/vvi_dial_fpm",
            Dial::Airspeed => "sim/cockpit2/autopilot/airspeed_dial_kts_mach",
            Dial::AirspeedKts => "sim/cockpit2/autopilot/airspeed_dial_kts",
            Dial::HoverVector => "sim/cockpit2/autopilot/hover_vector",
        }
    }

    /// The dial's name for a READ (a tick's batched read, a fake's preset).
    /// Never for a write: those are [`set_dial`]'s.
    pub const fn read_name(self) -> &'static str {
        self.name()
    }
}

/// The dial a dataref name is, if it is one (the undo's reverts carry
/// names).
pub fn dial_named(name: &str) -> Option<Dial> {
    [
        Dial::HeadingBug,
        Dial::Altitude,
        Dial::Vs,
        Dial::VviFpm,
        Dial::Airspeed,
        Dial::AirspeedKts,
        Dial::HoverVector,
    ]
    .into_iter()
    .find(|d| d.name() == name)
}

/// Write a dial.
pub async fn set_dial(xp: &dyn XPlane, d: Dial, v: f64) -> anyhow::Result<()> {
    note(OurChange::Dial(d, v));
    xp.set_float(d.name(), v).await
}

/// Write one element of an array dial (the hover vector).
pub async fn set_dial_elem(xp: &dyn XPlane, d: Dial, index: usize, v: f64) -> anyhow::Result<()> {
    note(OurChange::Dial(d, v));
    xp.set_float_array_elem(d.name(), index, v).await
}

/// Read a dial.
pub async fn read_dial(xp: &dyn XPlane, d: Dial) -> anyhow::Result<f64> {
    xp.get_float(d.name()).await
}

/// A dial written and read back, only on a change past `change`
/// ([`crate::control::set_confirmed`]: a later write is suppressed only once
/// the sim has confirmed this one).
pub async fn set_dial_confirmed(
    kind: &str,
    xp: &dyn XPlane,
    d: Dial,
    want: f64,
    last: &mut Option<f64>,
    change: f64,
) -> bool {
    note(OurChange::Dial(d, want));
    crate::control::set_confirmed(kind, xp, d.name(), want, last, change).await
}

/// Read an array dial (the hover vector).
pub async fn read_dial_array(xp: &dyn XPlane, d: Dial) -> anyhow::Result<Vec<f64>> {
    xp.get_float_array(d.name()).await
}

/// Put a dial write in a tick's write round (`sim::state::Writes`).
pub fn stage_dial(writes: &mut sim::state::Writes, d: Dial, v: f64) {
    note(OurChange::Dial(d, v));
    writes.set(d.name(), v);
}

// ---------------------------------------------------------------------------
// The heading knob.
// ---------------------------------------------------------------------------

/// Drive the heading BUG to `target` the way a human does: by turning the knob
/// (`heading_up`/`heading_down` presses). Glass avionics that ignore raw dataref
/// writes (the SR22's G1000) register the commands, and the knob animates. Falls
/// back to a direct dataref write when the commands don't move the bug (checked a
/// few presses in). Returns the bug's actual value afterwards.
pub async fn drive_heading_bug(xp: &dyn crate::xplane::XPlane, target: f64) -> f64 {
    drive_heading_bug_step(xp, target, 360.0).await
}

/// Like [`drive_heading_bug`] but turns the knob at most `max_step_deg` toward
/// the target per call, for control loops that drive the bug every tick. Each
/// press is an HTTP round trip, so an unbounded slew stalls the caller: the
/// LXGB go-around's 178° reversal froze the autoland loop ~5 s per tick with
/// no VS or throttle updates the whole time (2026-07-14). A capped call gets
/// there over successive ticks; the direct-write fallback (knob unbound) still
/// jumps straight to the full target.
/// Below this a heading change is SET, to a tenth of a degree, not pressed.
const FINE_BUG_DEG: f64 = 3.0;
/// ...and below this it is left alone.
const FINE_BUG_MIN_DEG: f64 = 0.2;

/// What [`drive_heading_bug_step`] would do from a bug already read.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BugStep {
    /// Within a fifth of a degree: the knob is left alone.
    Leave,
    /// Set to this value (a fine correction, or the target when the bug
    /// was unread): one write, which can ride the tick's write round.
    Set(f64),
    /// A change past [`FINE_BUG_DEG`]: pressed, through
    /// [`drive_heading_bug_step`] itself.
    Drive,
}

/// [`drive_heading_bug_step`]'s first decision from the bug a tick has
/// ALREADY read (`sim::state` carries it), so a loop that steers the knob
/// every tick pays no read of its own and its fine correction goes out with
/// the tick's other writes: the short final and the flare set the bug every
/// tick, three rounds of requests (read, write, read back) where one write
/// does (iMac 2026-09-30, a round is a frame, 50 ms at 20 fps). Pure.
pub fn bug_step_from(start: Option<f64>, target: f64) -> BugStep {
    let target = target.rem_euclid(360.0);
    let Some(start) = start.filter(|v| v.is_finite()) else {
        return BugStep::Set(target);
    };
    let full = crate::angles::signed_diff_deg(target, start);
    if full.abs() < FINE_BUG_DEG {
        if full.abs() >= FINE_BUG_MIN_DEG {
            let fine = ((start + full) * 10.0).round() / 10.0;
            return BugStep::Set(fine.rem_euclid(360.0));
        }
        return BugStep::Leave;
    }
    BugStep::Drive
}

pub async fn drive_heading_bug_step(
    xp: &dyn crate::xplane::XPlane,
    target: f64,
    max_step_deg: f64,
) -> f64 {
    let target = target.rem_euclid(360.0);
    let Ok(start) = read_dial(xp, Dial::HeadingBug).await else {
        let _ = set_dial(xp, Dial::HeadingBug, target).await;
        return target;
    };
    let full = crate::angles::signed_diff_deg(target, start);
    // A small correction is set to a tenth of a degree, not pressed: a press
    // is a whole degree and a change under half of one is no press at all,
    // so autoland's final command hovering at 131.5 flipped the bug between
    // 131 and 132 and the 737 wove 10 to 15 m either side of the centreline
    // all the way down (LEMG 13, 2026-09-26). Under a fifth of a degree the
    // knob is left alone.
    if full.abs() < FINE_BUG_DEG {
        if full.abs() >= FINE_BUG_MIN_DEG {
            let fine = ((start + full) * 10.0).round() / 10.0;
            let _ = set_dial(xp, Dial::HeadingBug, fine.rem_euclid(360.0)).await;
            return read_dial(xp, Dial::HeadingBug).await.unwrap_or(fine);
        }
        return start;
    }
    let delta = full.clamp(-max_step_deg.abs(), max_step_deg.abs());
    let leg = (start + delta).rem_euclid(360.0);
    let presses = delta.round().abs() as i64;
    let knob = if delta >= 0.0 {
        Button::HeadingUp
    } else {
        Button::HeadingDown
    };
    for i in 0..presses {
        if press(xp, knob).await.is_err() {
            break;
        }
        // A few presses in, confirm the knob is actually bound on this airframe.
        if i == 4 {
            let now = read_dial(xp, Dial::HeadingBug).await.unwrap_or(start);
            if (now - start).abs() < 0.5 {
                break;
            }
        }
    }
    let now = read_dial(xp, Dial::HeadingBug).await.unwrap_or(start);
    let err = crate::angles::abs_diff_deg(leg, now);
    if err > 1.5 {
        // Knob path did not get there: direct write as the fallback (straight
        // to the FULL target: a dataref write is one round trip, no cap needed).
        let _ = set_dial(xp, Dial::HeadingBug, target).await;
    }
    read_dial(xp, Dial::HeadingBug).await.unwrap_or(target)
}

// ---------------------------------------------------------------------------
// The ledger: what WE pressed and wrote, for whoever must tell a pilot's
// hand from ours (the safe hold's `mcp_moved`).
// ---------------------------------------------------------------------------

/// One change of ours to the panel.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum OurChange {
    Press(Button),
    Dial(Dial, f64),
}

/// How long the ledger remembers, s.
pub const LEDGER_S: f64 = 300.0;

static LEDGER: std::sync::Mutex<std::collections::VecDeque<(std::time::Instant, OurChange)>> =
    std::sync::Mutex::new(std::collections::VecDeque::new());

fn note(c: OurChange) {
    let now = std::time::Instant::now();
    let mut l = LEDGER.lock().unwrap_or_else(|e| e.into_inner());
    while l
        .front()
        .is_some_and(|(t, _)| now.duration_since(*t).as_secs_f64() > LEDGER_S)
    {
        l.pop_front();
    }
    l.push_back((now, c));
}

/// Every change of ours since `since` (oldest first). The one question a
/// watcher of the panel asks: was this move ours? Every press and dial write
/// in the tree goes through this module (`every_mcp_write_goes_through_the_owner`),
/// so a change this ledger does not hold was someone else's.
pub fn ours_since(since: std::time::Instant) -> Vec<(std::time::Instant, OurChange)> {
    LEDGER
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .filter(|(t, _)| *t >= since)
        .copied()
        .collect()
}

/// What a set of our changes can explain. Pure.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Ours(pub Vec<OurChange>);

impl Ours {
    /// We pressed `b`.
    pub fn pressed(&self, b: Button) -> bool {
        self.0.contains(&OurChange::Press(b))
    }
    /// We moved the servos in or out.
    pub fn servos(&self) -> bool {
        [Button::Engage, Button::Disconnect, Button::DisconnectFd]
            .iter()
            .any(|b| self.pressed(*b))
    }
    /// We wrote `d` to within `tol` of `value`.
    pub fn wrote(&self, d: Dial, value: f64, tol: f64) -> bool {
        let same = |x: Dial| {
            x == d || matches!((x, d), (Dial::Vs, Dial::VviFpm) | (Dial::VviFpm, Dial::Vs))
        };
        self.0
            .iter()
            .any(|c| matches!(c, OurChange::Dial(x, v) if same(*x) && (v - value).abs() <= tol))
    }
    /// The `autopilot_state` bits our presses can have brought in.
    pub fn mode_bits(&self) -> u32 {
        self.0
            .iter()
            .map(|c| match c {
                OurChange::Press(b) => button_bits(*b),
                OurChange::Dial(..) => 0,
            })
            .fold(0, |a, b| a | b)
    }
}

/// The modes a button can bring in (an engage brings the airframe's
/// default modes with it). Pure.
pub fn button_bits(b: Button) -> u32 {
    match b {
        Button::Engage => {
            Modes::HEADING | Modes::VS | Modes::ALT_HOLD | Modes::WING_LEVELER | Modes::PITCH_SYNC
        }
        Button::Heading => Modes::HEADING,
        Button::VerticalSpeed => Modes::VS | Modes::ALT_ARMED,
        Button::AltitudeHold => Modes::ALT_HOLD,
        Button::LevelChange => Modes::LEVEL_CHANGE | Modes::SPEED_ON_PITCH,
        Button::Approach => Modes::LOC_ARMED | Modes::LOC | Modes::GS_ARMED | Modes::GS,
        Button::Nav => Modes::LOC_ARMED | Modes::LOC,
        Button::Gpss => Modes::FMS_ARMED | Modes::FMS | Modes::LOC_ARMED | Modes::LOC,
        Button::WingLeveler => Modes::WING_LEVELER,
        Button::AutothrottleOn => Modes::AUTOTHROTTLE,
        _ => 0,
    }
}

/// X-Plane's `autopilot_state` flags: the modes as the autopilot has them.
/// Pure decoding.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Modes(pub u32);

impl Modes {
    pub const AUTOTHROTTLE: u32 = 1;
    pub const HEADING: u32 = 2;
    pub const WING_LEVELER: u32 = 4;
    pub const SPEED_ON_PITCH: u32 = 8;
    pub const VS: u32 = 16;
    pub const ALT_ARMED: u32 = 32;
    pub const LEVEL_CHANGE: u32 = 64;
    pub const PITCH_SYNC: u32 = 128;
    pub const LOC_ARMED: u32 = 256;
    pub const LOC: u32 = 512;
    pub const GS_ARMED: u32 = 1024;
    pub const GS: u32 = 2048;
    pub const FMS_ARMED: u32 = 4096;
    pub const FMS: u32 = 8192;
    pub const ALT_HOLD: u32 = 16384;
    pub const TOGA_LATERAL: u32 = 32768;
    pub const TOGA_VERTICAL: u32 = 65536;
    pub const VNAV_ARMED: u32 = 131072;
    pub const VNAV: u32 = 262144;

    /// From a read (a float on the wire).
    pub fn from_read(v: f64) -> Option<Self> {
        (v.is_finite() && v >= 0.0).then(|| Self(v.round() as u32))
    }
    fn has(self, bit: u32) -> bool {
        self.0 & bit != 0
    }
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn autothrottle(self) -> bool {
        self.has(Self::AUTOTHROTTLE)
    }
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn heading(self) -> bool {
        self.has(Self::HEADING)
    }
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn vs(self) -> bool {
        self.has(Self::VS)
    }
    pub fn loc_armed(self) -> bool {
        self.has(Self::LOC_ARMED)
    }
    pub fn loc(self) -> bool {
        self.has(Self::LOC)
    }
    pub fn gs_armed(self) -> bool {
        self.has(Self::GS_ARMED)
    }
    pub fn gs(self) -> bool {
        self.has(Self::GS)
    }
    /// The approach mode is in: either half armed or captured.
    pub fn approach_any(self) -> bool {
        self.has(Self::LOC_ARMED | Self::LOC | Self::GS_ARMED | Self::GS)
    }
    /// The words for the tape.
    pub fn words(self) -> Vec<&'static str> {
        const ALL: [(u32, &str); 17] = [
            (Modes::AUTOTHROTTLE, "at"),
            (Modes::HEADING, "hdg"),
            (Modes::WING_LEVELER, "wlv"),
            (Modes::SPEED_ON_PITCH, "spd_pitch"),
            (Modes::VS, "vs"),
            (Modes::ALT_ARMED, "alt_arm"),
            (Modes::LEVEL_CHANGE, "flch"),
            (Modes::PITCH_SYNC, "pitch_sync"),
            (Modes::LOC_ARMED, "loc_arm"),
            (Modes::LOC, "loc"),
            (Modes::GS_ARMED, "gs_arm"),
            (Modes::GS, "gs"),
            (Modes::FMS_ARMED, "fms_arm"),
            (Modes::FMS, "fms"),
            (Modes::ALT_HOLD, "alt_hold"),
            (Modes::TOGA_LATERAL, "toga_lat"),
            (Modes::TOGA_VERTICAL, "toga_vert"),
        ];
        ALL.iter()
            .filter(|(b, _)| self.has(*b))
            .map(|(_, w)| *w)
            .collect()
    }
}

/// Where the altitude window is parked while a loop flies the VS wheel to
/// the runway.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum WindowRule {
    /// Above the aircraft, out of the way: an altitude capture would take a
    /// descent (the LEAX rule).
    ParkAbove,
    /// At the field: this MCP flies a V/S only TOWARD its window (the 737:
    /// parked at 5,600 over a 3,400 ft inbound, -570 held level, LEMG 13
    /// 2026-10-05 12:25), and a window anywhere between the aircraft and the
    /// field would capture the descent. Its go-around climbs flew away from
    /// a window at 0 the same day.
    AtField,
}

/// A window this far over the aircraft is already out of a climb's way.
pub const WINDOW_CLEAR_FT: f64 = 500.0;
/// Parked this far above (rounded up to the hundred).
pub const WINDOW_PARK_ABOVE_FT: f64 = 1000.0;

/// Where the window should go for a loop that flies the VS wheel to the
/// runway at `alt_ft` (indicated) over a field at `field_ft`, or `None`
/// when it already serves. Pure.
pub fn window_for(rule: WindowRule, dial_ft: f64, alt_ft: f64, field_ft: f64) -> Option<f64> {
    match rule {
        WindowRule::ParkAbove => (dial_ft < alt_ft + WINDOW_CLEAR_FT)
            .then(|| ((alt_ft + WINDOW_PARK_ABOVE_FT) / 100.0).ceil() * 100.0),
        WindowRule::AtField => {
            let field = (field_ft.max(0.0) / 100.0).floor() * 100.0;
            (dial_ft > field + 1.0).then_some(field)
        }
    }
}

/// The airframe's panel: which command each button is, and how its MCP
/// behaves. From the live profile at each call (published per swap by
/// `aircraft::publish_live_profile`, so it never outlives its airframe).
#[derive(Debug, Clone, PartialEq)]
pub struct Panel {
    pub engage: String,
    pub heading: String,
    pub vertical_speed: String,
    pub altitude_hold: String,
    pub approach: String,
    pub autothrottle_off: String,
    pub window: WindowRule,
    /// The profile's `ils_coupled`: autoland flies the airframe's APP.
    pub coupled_ils: bool,
}

impl Default for Panel {
    fn default() -> Self {
        Self {
            engage: cmd::SERVOS_ON.into(),
            heading: cmd::HEADING.into(),
            vertical_speed: cmd::VERTICAL_SPEED.into(),
            altitude_hold: cmd::ALTITUDE_HOLD.into(),
            approach: cmd::APPROACH.into(),
            autothrottle_off: cmd::AT_OFF.into(),
            window: WindowRule::ParkAbove,
            coupled_ils: false,
        }
    }
}

impl Panel {
    /// The panel a profile declares. Pure.
    pub fn from_profile(p: &crate::aircraft::AircraftProfile) -> Self {
        let d = Self::default();
        let a = p.autoflight.as_ref();
        let pick = |v: Option<&String>, dflt: String| v.cloned().unwrap_or(dflt);
        Self {
            engage: pick(a.and_then(|a| a.engage_command.as_ref()), d.engage),
            heading: pick(a.and_then(|a| a.heading_command.as_ref()), d.heading),
            vertical_speed: pick(a.and_then(|a| a.vs_command.as_ref()), d.vertical_speed),
            altitude_hold: pick(
                a.and_then(|a| a.altitude_hold_command.as_ref()),
                d.altitude_hold,
            ),
            approach: pick(p.approach_command.as_ref(), d.approach),
            autothrottle_off: pick(
                a.and_then(|a| a.autothrottle_off_command.as_ref()),
                d.autothrottle_off,
            ),
            window: if a.is_some_and(|a| a.vs_toward_window) {
                WindowRule::AtField
            } else {
                WindowRule::ParkAbove
            },
            coupled_ils: p.ils_coupled,
        }
    }

    /// The command a button is on this panel.
    pub fn command(&self, b: Button) -> &str {
        match b {
            Button::Engage => &self.engage,
            Button::Disconnect => cmd::SERVOS_OFF_ANY,
            Button::DisconnectFd => cmd::SERVOS_FDIR_OFF,
            Button::Heading => &self.heading,
            Button::VerticalSpeed => &self.vertical_speed,
            Button::AltitudeHold => &self.altitude_hold,
            Button::LevelChange => cmd::LEVEL_CHANGE,
            Button::Approach => &self.approach,
            Button::Nav => cmd::NAV,
            Button::Gpss => cmd::GPSS,
            Button::WingLeveler => cmd::WING_LEVELER,
            Button::HsiNav1 => cmd::HSI_NAV1,
            Button::HsiGps => cmd::HSI_GPS,
            Button::HeadingUp => cmd::HEADING_UP,
            Button::HeadingDown => cmd::HEADING_DOWN,
            Button::Hover => cmd::HOVER,
            Button::FlightDirectorOn => cmd::FD_ON,
            Button::AutothrottleOn => cmd::AT_ON,
            Button::AutothrottleOff => &self.autothrottle_off,
            Button::AutothrottleArm => cmd::AT_ARM,
        }
    }
}

/// The live airframe's panel.
pub fn panel() -> Panel {
    crate::aircraft::live_profile()
        .map(|p| Panel::from_profile(&p))
        .unwrap_or_default()
}

/// Press a button: the airframe's command for it. Routine presses (the
/// knob's steps, a loop's re-engage every tick) are not taped here; the
/// sites tape their own decisions, and the mode presses below are taped by
/// [`press_logged`].
pub async fn press(xp: &dyn XPlane, b: Button) -> anyhow::Result<()> {
    note(OurChange::Press(b));
    let p = panel();
    xp.run_command(p.command(b)).await
}

/// [`press`], on the tape: blackbox `autoflight` `press` with the site and
/// the command the panel resolved.
pub async fn press_logged(xp: &dyn XPlane, b: Button, site: &str) -> bool {
    note(OurChange::Press(b));
    let p = panel();
    let c = p.command(b).to_string();
    let sent = xp.run_command(&c).await.is_ok();
    crate::blackbox::record(
        "autoflight",
        json!({"event": "press", "button": b, "cmd": c, "site": site, "sent": sent}),
    );
    sent
}

/// What a loop wants the autopilot to fly laterally.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Lateral {
    /// The loop's heading bug, in HDG SEL.
    Heading,
    /// The airframe's own localiser capture (APP): never HDG over it while it
    /// is armed or captured.
    Coupled,
}

/// What a loop wants the autopilot to fly vertically.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Vertical {
    /// The loop's VS wheel, in V/S.
    Vs,
    /// The airframe's glideslope: the VS wheel until it captures, then never
    /// V/S over it.
    Coupled,
}

/// A loop's intent for this tick. `None` on an axis: the loop does not want
/// the autopilot on it (the arrest has the servos out, the hover flies its
/// own).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Want {
    pub lateral: Option<Lateral>,
    pub vertical: Option<Vertical>,
}

/// The modes as one tick read them: the per-mode statuses of the loop's
/// frame and, where it read it, the bitfield.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ModeRead {
    pub hdg_status: Option<f64>,
    pub vs_status: Option<f64>,
    pub modes: Option<Modes>,
}

/// The presses that bring the modes READ to the intent: only on a
/// difference, never over a capture the intent gives the airframe, nothing
/// on a failed read. Pure.
pub fn reconcile(want: Want, read: ModeRead) -> Vec<Button> {
    let mut out = Vec::new();
    let m = read.modes;
    match want.lateral {
        Some(Lateral::Heading) => {
            if read.hdg_status.is_some_and(|s| s <= 0.5) {
                out.push(Button::Heading);
            }
        }
        Some(Lateral::Coupled) => {
            // The localiser armed or captured owns the lateral; with neither
            // HDG is what flies until it arms. An unread bitfield is no
            // measurement: nothing is pressed on it.
            let loc_owned = m.is_some_and(|m| m.loc_armed() || m.loc());
            if !loc_owned && m.is_some() && read.hdg_status.is_some_and(|s| s <= 0.5) {
                out.push(Button::Heading);
            }
        }
        None => {}
    }
    match want.vertical {
        Some(Vertical::Vs) => {
            if read.vs_status.is_some_and(|s| s <= 0.5) {
                out.push(Button::VerticalSpeed);
            }
        }
        Some(Vertical::Coupled) => {
            // V/S until the glideslope captures: armed is still V/S flying
            // down (or level) to meet it from below.
            let gs_owned = m.is_some_and(|m| m.gs());
            if !gs_owned && m.is_some() && read.vs_status.is_some_and(|s| s <= 0.5) {
                out.push(Button::VerticalSpeed);
            }
        }
        None => {}
    }
    out
}

/// Press what [`reconcile`] asks, in order, each on the tape. `false` when
/// any press did not go out (every one is still tried).
pub async fn apply(xp: &dyn XPlane, presses: &[Button], site: &str) -> bool {
    let mut all = true;
    for b in presses {
        all &= press_logged(xp, *b, site).await;
    }
    all
}

/// What [`bring`] pressed, and whether every press went out.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Applied {
    pub presses: Vec<Button>,
    pub sent: bool,
}

impl Applied {
    /// `b` was pressed (sent or not).
    pub fn pressed(&self, b: Button) -> bool {
        self.presses.contains(&b)
    }
}

/// [`reconcile`] the intent against the modes read, then [`apply`] it: the
/// one door a loop's mode presses go through.
pub async fn bring(xp: &dyn XPlane, want: Want, read: ModeRead, site: &str) -> Applied {
    let presses = reconcile(want, read);
    let sent = apply(xp, &presses, site).await;
    Applied { presses, sent }
}

/// [`bring`] HDG SEL alone, from its status read (`None`: unread, nothing
/// pressed).
pub async fn bring_heading(xp: &dyn XPlane, hdg_status: Option<f64>, site: &str) -> Applied {
    let want = Want {
        lateral: Some(Lateral::Heading),
        vertical: None,
    };
    let read = ModeRead {
        hdg_status,
        ..Default::default()
    };
    bring(xp, want, read, site).await
}

/// [`bring`] V/S alone, from its status read (`None`: unread, nothing
/// pressed).
pub async fn bring_vs(xp: &dyn XPlane, vs_status: Option<f64>, site: &str) -> Applied {
    let want = Want {
        lateral: None,
        vertical: Some(Vertical::Vs),
    };
    let read = ModeRead {
        vs_status,
        ..Default::default()
    };
    bring(xp, want, read, site).await
}

// ---------------------------------------------------------------------------
// The thrust.
// ---------------------------------------------------------------------------

/// An autothrottle read: on (really driving) and its mode.
pub fn autothrottle_engaged(at_on: Option<f64>, at_mode: Option<f64>) -> Option<bool> {
    let on = at_on?;
    Some(on >= 0.5 && at_mode.is_none_or(|m| m >= 0.5))
}

/// Presses per episode before the take is given up and said.
pub const THRUST_TRIES: u32 = 3;
/// Seconds between presses while it still reads engaged.
pub const THRUST_RETRY_S: f64 = 1.0;
/// Episodes (a take that worked and the autothrottle came back) before the
/// watch stops fighting and says who has the thrust.
pub const THRUST_EPISODES: u32 = 3;

/// What the thrust watch wants this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThrustStep {
    Nothing,
    /// Press the airframe's autothrottle-off (try `n` of the episode);
    /// `say` on the first take of the watch.
    Take {
        n: u32,
        say: bool,
    },
    /// It will not come out (or keeps coming back): say it, once.
    GiveUp,
}

/// A loop that flies the levers watches the autothrottle every tick and
/// takes an engaged one out to ARM: pressed every [`THRUST_RETRY_S`] while
/// it reads engaged, at most [`THRUST_TRIES`] an episode and
/// [`THRUST_EPISODES`] episodes, then given up and said. A failed read is no
/// measurement. Pure.
#[derive(Debug, Clone, Default)]
pub struct ThrustWatch {
    tries: u32,
    episodes: u32,
    last_fire_s: Option<f64>,
    said: bool,
    gave_up: bool,
}

impl ThrustWatch {
    pub fn step(&mut self, now_s: f64, engaged: Option<bool>) -> ThrustStep {
        if self.gave_up {
            return ThrustStep::Nothing;
        }
        match engaged {
            None => ThrustStep::Nothing,
            Some(false) => {
                if self.tries > 0 {
                    self.tries = 0;
                }
                ThrustStep::Nothing
            }
            Some(true) => {
                if self.tries == 0 {
                    self.episodes += 1;
                    if self.episodes > THRUST_EPISODES {
                        self.gave_up = true;
                        return ThrustStep::GiveUp;
                    }
                }
                let due = self
                    .last_fire_s
                    .is_none_or(|t| now_s - t >= THRUST_RETRY_S - 1e-9);
                if !due {
                    return ThrustStep::Nothing;
                }
                if self.tries >= THRUST_TRIES {
                    self.gave_up = true;
                    return ThrustStep::GiveUp;
                }
                self.tries += 1;
                self.last_fire_s = Some(now_s);
                let say = !self.said;
                self.said = true;
                ThrustStep::Take { n: self.tries, say }
            }
        }
    }
}

/// The words when a loop takes the thrust.
pub const THRUST_TAKEN_LINE: &str = "Autothrottle off, I have the thrust.";
/// ...and when it will not let go.
pub const THRUST_STUCK_LINE: &str =
    "The autothrottle won't stay off and it's fighting my thrust. Disconnect it.";

/// Act on a [`ThrustStep`]: press the airframe's autothrottle-off, on the
/// tape. Returns the line to say, if any.
pub async fn act_thrust(xp: &dyn XPlane, step: ThrustStep, owner: &str) -> Option<&'static str> {
    match step {
        ThrustStep::Nothing => None,
        ThrustStep::Take { n, say } => {
            let p = panel();
            let sent = {
                note(OurChange::Press(Button::AutothrottleOff));
                xp.run_command(&p.autothrottle_off).await
            }
            .is_ok();
            crate::blackbox::record(
                "autoflight",
                json!({"event": "thrust_taken", "owner": owner, "try": n, "cmd": p.autothrottle_off, "sent": sent}),
            );
            say.then_some(THRUST_TAKEN_LINE)
        }
        ThrustStep::GiveUp => {
            crate::blackbox::record(
                "autoflight",
                json!({"event": "thrust_not_taken", "owner": owner}),
            );
            Some(THRUST_STUCK_LINE)
        }
    }
}

/// How long a mode takes to show after its command (a frame or two, an
/// add-on's mirror a frame behind).
pub const SETTLE_MS: u64 = 400;

/// The one-shot take at a loop's arm: read, press only if engaged, read back,
/// up to [`THRUST_TRIES`]. `Some(true)` taken, `Some(false)` it would not
/// come out, `None` it was not engaged (or unread: no measurement). Taped.
#[cfg_attr(not(test), allow(dead_code))] // offered to the other loops
pub async fn take_thrust(xp: &dyn XPlane, owner: &str) -> Option<bool> {
    let read = || async {
        let (on, mode) = tokio::join!(xp.get_float(names::AT_ON), xp.get_float(names::AT_MODE));
        autothrottle_engaged(on.ok(), mode.ok())
    };
    if read().await != Some(true) {
        return None;
    }
    let p = panel();
    for n in 1..=THRUST_TRIES {
        let _ = {
            note(OurChange::Press(Button::AutothrottleOff));
            xp.run_command(&p.autothrottle_off).await
        };
        tokio::time::sleep(Duration::from_millis(SETTLE_MS)).await;
        let after = read().await;
        crate::blackbox::record(
            "autoflight",
            json!({"event": "thrust_taken", "owner": owner, "try": n, "cmd": p.autothrottle_off,
                   "engaged_after": after}),
        );
        if after == Some(false) {
            return Some(true);
        }
    }
    Some(false)
}

/// The take without the wait, for a loop that cannot spare the settle (the
/// upset recovery: its first writes are the controls): read, press the
/// airframe's autothrottle-off only when engaged, on the tape. `Some(true)`
/// pressed, `None` not engaged or unread.
pub async fn drop_thrust_now(xp: &dyn XPlane, owner: &str) -> Option<bool> {
    let (on, mode) = tokio::join!(xp.get_float(names::AT_ON), xp.get_float(names::AT_MODE));
    if autothrottle_engaged(on.ok(), mode.ok()) != Some(true) {
        return None;
    }
    let p = panel();
    let sent = {
        note(OurChange::Press(Button::AutothrottleOff));
        xp.run_command(&p.autothrottle_off).await
    }
    .is_ok();
    crate::blackbox::record(
        "autoflight",
        json!({"event": "thrust_taken", "owner": owner, "try": 1, "cmd": p.autothrottle_off, "sent": sent}),
    );
    Some(sent)
}

// ---------------------------------------------------------------------------
// The approach mode.
// ---------------------------------------------------------------------------

/// Arm the airframe's APP: pressed only when neither half reads armed or
/// captured (a toggle), read back on the bitfield. `Some(true)` armed (or
/// already), `Some(false)` pressed and not armed, `None` unread. Taped.
#[cfg_attr(not(test), allow(dead_code))] // offered to the other loops
pub async fn arm_approach(xp: &dyn XPlane, site: &str) -> Option<bool> {
    let before = Modes::from_read(xp.get_float(names::AP_STATE).await.ok()?)?;
    if before.approach_any() {
        return Some(true);
    }
    press_logged(xp, Button::Approach, site).await;
    tokio::time::sleep(Duration::from_millis(SETTLE_MS)).await;
    let after = xp
        .get_float(names::AP_STATE)
        .await
        .ok()
        .and_then(Modes::from_read);
    crate::blackbox::record(
        "autoflight",
        json!({"event": "approach_arm", "site": site, "before": before.words(),
               "after": after.map(|m| m.words())}),
    );
    after.map(|m| m.approach_any())
}

// ---------------------------------------------------------------------------
// Primitives: engage, hold level, hold heading.
// ---------------------------------------------------------------------------

/// The servos in and READ in ([`crate::ap_engage::engage_confirmed`]: the
/// flight director commands first, then the panel's engage, pressed until
/// the servos read in on two live reads), at most `max_s`.
pub async fn engage_confirmed(xp: &dyn XPlane, site: &str, max_s: f64) -> bool {
    let fd = crate::ap_engage::fd_commands();
    crate::ap_engage::engage_confirmed(xp, &fd, "autoflight", site, max_s).await
}

/// Make sure a toggled mode reads on: read its status first, press only
/// when off, read back, up to `tries`. `true` when it reads on.
pub async fn ensure_on(xp: &dyn XPlane, b: Button, status: &str, site: &str, tries: u32) -> bool {
    for _ in 0..tries.max(1) {
        match xp.get_float(status).await {
            Ok(v) if v >= 0.5 => return true,
            Ok(_) => {
                press_logged(xp, b, site).await;
                tokio::time::sleep(Duration::from_millis(SETTLE_MS)).await;
            }
            Err(_) => return false,
        }
    }
    xp.get_float(status).await.is_ok_and(|v| v >= 0.5)
}

/// Hold this level: servos in, the window AT the level (rounded to the
/// hundred), ALT HOLD on and read back. A primitive for the safe-hold and
/// recovery loops. `true` when the hold reads on.
#[cfg_attr(not(test), allow(dead_code))] // offered to the other loops
pub async fn hold_level(xp: &dyn XPlane, alt_ft: f64, site: &str) -> bool {
    if !writer_may_act(site) || !engage_confirmed(xp, site, 6.0).await {
        return false;
    }
    let level = (alt_ft / 100.0).round() * 100.0;
    let _ = set_dial(xp, Dial::Altitude, level).await;
    let ok = ensure_on(xp, Button::AltitudeHold, names::ALT_HOLD_STATUS, site, 3).await;
    crate::blackbox::record(
        "autoflight",
        json!({"event": "hold_level", "site": site, "level": level, "held": ok}),
    );
    ok
}

/// Hold this heading: servos in, the bug turned to it (the knob, which glass
/// that ignores a dial write registers), HDG SEL on and read back. A
/// primitive for the safe-hold and recovery loops.
pub async fn hold_heading(xp: &dyn XPlane, hdg_mag: f64, site: &str) -> bool {
    if !writer_may_act(site) || !engage_confirmed(xp, site, 6.0).await {
        return false;
    }
    drive_heading_bug(xp, hdg_mag.rem_euclid(360.0)).await;
    let ok = ensure_on(xp, Button::Heading, names::HDG_STATUS, site, 3).await;
    crate::blackbox::record(
        "autoflight",
        json!({"event": "hold_heading", "site": site, "hdg": hdg_mag, "held": ok}),
    );
    ok
}

/// The safe hold's site: a legitimate writer only while no loop of ours has
/// the aircraft (a loop owns its panel; the hold is for an aircraft nobody
/// flies).
pub const SAFE_HOLD: &str = "safe_hold";

/// May `site` act on the panel now? The safe hold only while no loop of
/// ours has the aircraft; every other site is its loop's own. A refusal is
/// on the tape.
pub fn writer_may_act(site: &str) -> bool {
    if site != SAFE_HOLD {
        return true;
    }
    match crate::flight_state::crew_has_aircraft() {
        None => true,
        Some(owner) => {
            crate::blackbox::record(
                "autoflight",
                json!({"event": "refused", "site": site, "owner": owner.name()}),
            );
            false
        }
    }
}

/// Climb or descend to `alt_ft` (the altimeter) at `fpm` and capture it:
/// servos in, the window AT the level (the capture's, and toward the VS on
/// an MCP that flies one only toward its window), V/S on, the wheel set
/// after the engage (an engage syncs it away), the window read back. A
/// primitive for the safe hold. `true` when the window and V/S read back.
pub async fn climb_to(xp: &dyn XPlane, alt_ft: f64, fpm: f64, site: &str) -> bool {
    if !writer_may_act(site) || !engage_confirmed(xp, site, 6.0).await {
        return false;
    }
    let level = (alt_ft / 100.0).round() * 100.0;
    let _ = set_dial(xp, Dial::Altitude, level).await;
    let vs_on = ensure_on(xp, Button::VerticalSpeed, names::VS_STATUS, site, 3).await;
    let _ = set_dial(xp, Dial::Vs, fpm).await;
    let window = read_dial(xp, Dial::Altitude)
        .await
        .is_ok_and(|w| (w - level).abs() <= 50.0);
    crate::blackbox::record(
        "autoflight",
        json!({"event": "climb_to", "site": site, "level": level, "fpm": fpm, "vs_on": vs_on, "window": window}),
    );
    vs_on && window
}

/// Every name the owner relies on for a profile that declares its panel,
/// for the profile probe (`aircraft::profile_references`): the stock
/// reads and commands behind the intent API.
#[cfg_attr(not(test), allow(dead_code))] // offered to the other loops
pub fn stock_references() -> &'static [&'static str] {
    &[
        names::AP_STATE,
        names::AT_ON,
        names::AT_MODE,
        names::SERVOS_ON,
        names::HDG_STATUS,
        names::VS_STATUS,
        names::ALT_HOLD_STATUS,
        cmd::AT_OFF,
        "sim/cockpit2/autopilot/altitude_dial_ft",
        "sim/cockpit/autopilot/vertical_velocity",
        "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot",
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xplane::FakeXPlane;

    /// ONE owner: outside this file no production code names an autopilot
    /// command or dataref, presses the owner's stock commands itself, or
    /// writes a dial (a dial alias is any const bound to `read_name()`). The
    /// one named exemption is `aircraft.rs`'s probe lists (the takeoff
    /// configuration's and `AutoflightProfile::STOCK`: the lib crate cannot
    /// see this module); outside it they are never used to act.
    #[test]
    fn every_mcp_write_goes_through_the_owner() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut bad = Vec::new();
        let mut scanned = Vec::new();
        for f in crate::test_support::rust_files(&root) {
            if crate::test_support::is_test_file(&f) || f.ends_with("autoflight.rs") {
                continue;
            }
            let rel = f.strip_prefix(&root).unwrap().to_string_lossy().to_string();
            scanned.push(rel.clone());
            let text = std::fs::read_to_string(&f).unwrap();
            let production = text.split("#[cfg(test)]\nmod ").next().unwrap();
            // cfg(test) items (the aliases tests assert on) are not production.
            let mut lines: Vec<&str> = Vec::new();
            let mut skip_next = false;
            for l in production.lines() {
                if skip_next {
                    skip_next = false;
                    continue;
                }
                if l.trim() == "#[cfg(test)]" {
                    skip_next = true;
                    continue;
                }
                lines.push(l);
            }
            let aliases: Vec<String> = lines
                .iter()
                .filter(|l| l.contains("read_name()") && l.trim_start().starts_with("const "))
                .filter_map(|l| {
                    l.trim_start()
                        .strip_prefix("const ")?
                        .split(':')
                        .next()
                        .map(str::to_string)
                })
                .collect();
            for (i, l) in lines.iter().enumerate() {
                let code = l.split("//").next().unwrap_or("");
                let lit = [
                    "\"sim/autopilot/",
                    "\"sim/cockpit2/autopilot/",
                    "\"sim/cockpit/autopilot/",
                    "\"laminar/B738/autopilot/",
                ]
                .iter()
                .any(|p| code.contains(p));
                if lit && rel != "aircraft.rs" {
                    bad.push(format!("{rel}:{}: literal: {}", i + 1, l.trim()));
                }
                if code.contains("autoflight::cmd::") {
                    bad.push(format!(
                        "{rel}:{}: stock command outside the owner: {}",
                        i + 1,
                        l.trim()
                    ));
                }
                if rel != "aircraft.rs"
                    && [
                        "TakeoffConfigProfile::CMD_",
                        "TakeoffConfigProfile::SPEED_WINDOW",
                        "AutoflightProfile::STOCK",
                    ]
                    .iter()
                    .any(|p| code.contains(p))
                {
                    bad.push(format!(
                        "{rel}:{}: probe name used to act: {}",
                        i + 1,
                        l.trim()
                    ));
                }
                let writes = [
                    "set_float(",
                    "set_int(",
                    "set_float_array_elem(",
                    "set_float_index(",
                    ".set(",
                ]
                .iter()
                .any(|w| code.contains(w));
                if writes
                    && (code.contains("read_name()")
                        || aliases.iter().any(|a| code.contains(a.as_str())))
                {
                    bad.push(format!(
                        "{rel}:{}: dial written outside the owner: {}",
                        i + 1,
                        l.trim()
                    ));
                }
            }
        }
        // The safe hold and the recovery planner are writers like any other.
        for f in ["safe_hold.rs", "recovery_plan.rs", "autoland.rs", "tools/full_auto.rs"] {
            assert!(scanned.iter().any(|s| s == f), "{f} not scanned");
        }
        assert!(
            bad.is_empty(),
            "MCP access outside crate::autoflight:\n{}",
            bad.join("\n")
        );
    }

    /// The 737's panel is its own buttons, probed by the swap's profile
    /// check; CMD B is never one of them.
    #[test]
    fn the_737_panel_is_its_own_buttons_and_probed() {
        let p = crate::aircraft::load("boeing_737_800").expect("737 profile");
        let panel = Panel::from_profile(&p);
        assert_eq!(panel.engage, "laminar/B738/autopilot/cmd_a_press");
        assert_eq!(panel.heading, "laminar/B738/autopilot/hdg_sel_press");
        assert_eq!(panel.vertical_speed, "laminar/B738/autopilot/vs_press");
        assert_eq!(panel.altitude_hold, "laminar/B738/autopilot/alt_hld_press");
        assert_eq!(panel.approach, "laminar/B738/autopilot/app_press");
        assert_eq!(panel.window, WindowRule::AtField);
        assert_eq!(panel.autothrottle_off, cmd::AT_OFF);
        assert_eq!(
            panel.command(Button::LevelChange),
            cmd::LEVEL_CHANGE,
            "stock: the 737's own engages the A/T"
        );
        let names = crate::aircraft::profile_references(&p);
        // The owner's stock names ride the probe of a declared panel, the
        // lib's copy kept equal to this module's.
        let mut stock: Vec<&str> = stock_references().to_vec();
        let mut lib: Vec<&str> = crate::aircraft::AutoflightProfile::STOCK.to_vec();
        stock.sort_unstable();
        lib.sort_unstable();
        assert_eq!(stock, lib);
        for n in stock_references() {
            assert!(names.iter().any(|x| x == n), "{n} not probed");
        }
        for n in [
            &panel.engage,
            &panel.heading,
            &panel.vertical_speed,
            &panel.altitude_hold,
            &panel.approach,
        ] {
            assert!(names.iter().any(|x| x == n), "{n} not probed");
        }
        assert!(
            !names.iter().any(|x| x.contains("cmd_b")),
            "CMD B is never named"
        );
        // An airframe that declares nothing flies the stock panel.
        let c172 = crate::aircraft::load("cessna_172sp").expect("172 profile");
        assert_eq!(Panel::from_profile(&c172), Panel::default());
    }

    /// Every name the owner and the 737 panel use exists in the installed
    /// sim (DataRefs.txt, Commands.txt) or the 737's own xlua.
    /// `cargo test -- --ignored real_install_autoflight_names`
    #[test]
    #[ignore]
    fn real_install_autoflight_names() {
        let home = std::env::var("HOME").unwrap();
        let xp = format!("{home}/X-Plane 12");
        let read = |p: &str| std::fs::read_to_string(p).unwrap_or_default();
        let first = |t: String| -> std::collections::HashSet<String> {
            t.lines()
                .filter_map(|l| l.split_whitespace().next().map(str::to_string))
                .collect()
        };
        let drefs = first(read(&format!("{xp}/Resources/plugins/DataRefs.txt")));
        let cmds = first(read(&format!("{xp}/Resources/plugins/Commands.txt")));
        let lua = read(&format!(
            "{xp}/Aircraft/Laminar Research/Boeing 737-800/plugins/xlua/scripts/B738.glareshield/B738.glareshield.lua"
        ));
        assert!(
            !drefs.is_empty() && !cmds.is_empty() && !lua.is_empty(),
            "install not found under {xp}"
        );
        let p = crate::aircraft::load("boeing_737_800").expect("737 profile");
        let panel = Panel::from_profile(&p);
        let mut names: Vec<String> = stock_references().iter().map(|s| s.to_string()).collect();
        names.extend(
            [
                names::AP_ON,
                names::AP_POWER,
                names::NAV_STATUS,
                names::GPSS_STATUS,
                names::SPEED_STATUS,
                names::ALT_MODE,
                names::FD_MODE,
                names::HOVER_STATUS,
            ]
            .iter()
            .map(|s| s.to_string()),
        );
        for d in [
            Dial::HeadingBug,
            Dial::Altitude,
            Dial::Vs,
            Dial::VviFpm,
            Dial::Airspeed,
            Dial::AirspeedKts,
            Dial::HoverVector,
        ] {
            names.push(d.name().to_string());
        }
        for b in [
            Button::Engage,
            Button::Disconnect,
            Button::DisconnectFd,
            Button::Heading,
            Button::VerticalSpeed,
            Button::AltitudeHold,
            Button::LevelChange,
            Button::Approach,
            Button::Nav,
            Button::Gpss,
            Button::WingLeveler,
            Button::HsiNav1,
            Button::HsiGps,
            Button::HeadingUp,
            Button::HeadingDown,
            Button::Hover,
            Button::FlightDirectorOn,
            Button::AutothrottleOn,
            Button::AutothrottleOff,
            Button::AutothrottleArm,
        ] {
            names.push(Panel::default().command(b).to_string());
            names.push(panel.command(b).to_string());
        }
        let missing: Vec<&String> = names
            .iter()
            .filter(|n| {
                !drefs.contains(*n) && !cmds.contains(*n) && !lua.contains(&format!("\"{n}\""))
            })
            .collect();
        assert!(missing.is_empty(), "not in the install: {missing:?}");
    }

    /// The bitfield as the 2026-09-26 to 09-28 coupling attempts recorded
    /// it: 50 is HDG + VS + ALT arm; 1584 the localiser captured with the
    /// glideslope armed; 2560 both captured; 51 the 2026-10-05 A/T in SPEED.
    #[test]
    fn the_bitfield_reads_as_the_recorded_flights_had_it() {
        let m = Modes(50);
        assert!(m.heading() && m.vs() && !m.approach_any() && !m.autothrottle());
        let m = Modes(1584);
        assert!(m.loc() && m.gs_armed() && m.vs() && !m.heading());
        let m = Modes(2560);
        assert!(m.loc() && m.gs() && !m.vs());
        assert!(Modes(51).autothrottle());
        assert_eq!(
            Modes(17921).words(),
            vec!["at", "loc", "gs_arm", "alt_hold"]
        );
        assert_eq!(Modes::from_read(-1.0), None);
    }

    /// The #81 failure, replayed through the owner: with the APP captured
    /// (1584, then 2560) and the HDG and VS statuses reading OFF, the old
    /// housekeeping pressed both and dropped the capture. Coupled intent
    /// presses nothing; plain heading intent still keeps HDG in.
    #[test]
    fn a_coupled_intent_never_presses_over_the_airframes_capture() {
        let coupled = Want {
            lateral: Some(Lateral::Coupled),
            vertical: Some(Vertical::Coupled),
        };
        // The statuses as the sim showed them with those bits: HDG off under a
        // captured localiser, V/S on until the glideslope captures.
        let read = |m: u32| {
            let modes = Modes(m);
            ModeRead {
                hdg_status: Some(if modes.heading() { 2.0 } else { 0.0 }),
                vs_status: Some(if modes.vs() { 2.0 } else { 0.0 }),
                modes: Some(modes),
            }
        };
        assert!(
            reconcile(coupled, read(1584)).is_empty(),
            "loc captured, gs armed: no HDG over it"
        );
        assert!(
            reconcile(coupled, read(2560)).is_empty(),
            "both captured: neither"
        );
        // What the old housekeeping did with the same reads: both pressed.
        let own = Want {
            lateral: Some(Lateral::Heading),
            vertical: Some(Vertical::Vs),
        };
        assert_eq!(
            reconcile(own, read(2560)),
            vec![Button::Heading, Button::VerticalSpeed]
        );
        // Armed only (1330: LOC and GS armed, HDG and VS flying): nothing to do.
        let armed = ModeRead {
            hdg_status: Some(2.0),
            vs_status: Some(2.0),
            modes: Some(Modes(1330)),
        };
        assert!(reconcile(coupled, armed).is_empty());
        // The capture lost (nothing in, the statuses off): back to HDG and VS.
        assert_eq!(
            reconcile(coupled, read(0)),
            vec![Button::Heading, Button::VerticalSpeed]
        );
        // An unread bitfield is no measurement under a coupled intent.
        let unread = ModeRead {
            hdg_status: Some(0.0),
            vs_status: Some(0.0),
            modes: None,
        };
        assert!(reconcile(coupled, unread).is_empty());
        // The loop's own heading and VS: pressed on a difference only.
        assert_eq!(
            reconcile(own, read(0)),
            vec![Button::Heading, Button::VerticalSpeed]
        );
        let on = ModeRead {
            hdg_status: Some(2.0),
            vs_status: Some(2.0),
            modes: None,
        };
        assert!(reconcile(own, on).is_empty());
        let unread = ModeRead::default();
        assert!(
            reconcile(own, unread).is_empty(),
            "a failed read presses nothing"
        );
    }

    /// GS armed but not captured is still the loop's V/S: the aircraft flies
    /// level or down to meet the beam from below, so a dropped VS comes back.
    #[test]
    fn the_wheel_is_ours_until_the_glideslope_captures() {
        let want = Want {
            lateral: None,
            vertical: Some(Vertical::Coupled),
        };
        let gs_armed = ModeRead {
            hdg_status: None,
            vs_status: Some(0.0),
            modes: Some(Modes(Modes::GS_ARMED)),
        };
        assert_eq!(reconcile(want, gs_armed), vec![Button::VerticalSpeed]);
    }

    /// LEMG 13, 2026-10-05: the 737 held level at 3,400 ft against a -570
    /// dial with the window parked at 5,600. On a V/S-toward-window MCP the
    /// window goes to the field (rounded down), whatever lies between; the
    /// LEAX rule parks it above everywhere else.
    #[test]
    fn the_window_is_parked_where_no_capture_takes_the_wheel() {
        let r = WindowRule::AtField;
        assert_eq!(window_for(r, 5600.0, 3400.0, 52.0), Some(0.0));
        assert_eq!(
            window_for(r, 0.0, 3400.0, 52.0),
            None,
            "already at the field"
        );
        assert_eq!(
            window_for(r, 2000.0, 3400.0, 52.0),
            Some(0.0),
            "below us is a capture"
        );
        assert_eq!(
            window_for(r, 9000.0, 7000.0, 5432.0),
            Some(5400.0),
            "a mountain field"
        );
        let a = WindowRule::ParkAbove;
        assert_eq!(window_for(a, 3000.0, 4579.0, 52.0), Some(5600.0));
        assert_eq!(window_for(a, 9500.0, 4579.0, 52.0), None);
    }

    /// 13:02:41 on 2026-10-05: autoland armed with the A/T in SPEED 230
    /// (`ap_state` 51) and nothing took it; it overrode every lever written
    /// until 13:09:34. The watch takes it on the first tick, retries a
    /// bounded number of times, and gives up loudly.
    #[test]
    fn the_thrust_watch_takes_an_engaged_autothrottle_and_is_bounded() {
        let mut w = ThrustWatch::default();
        assert_eq!(
            w.step(0.0, Some(true)),
            ThrustStep::Take { n: 1, say: true }
        );
        assert_eq!(
            w.step(0.5, Some(true)),
            ThrustStep::Nothing,
            "not before the retry"
        );
        assert_eq!(
            w.step(1.0, Some(true)),
            ThrustStep::Take { n: 2, say: false }
        );
        assert_eq!(
            w.step(2.0, Some(true)),
            ThrustStep::Take { n: 3, say: false }
        );
        assert_eq!(w.step(3.0, Some(true)), ThrustStep::GiveUp);
        assert_eq!(w.step(4.0, Some(true)), ThrustStep::Nothing, "said once");
        // A take that works, and a pilot who keeps re-engaging it.
        let mut w = ThrustWatch::default();
        for e in 0..THRUST_EPISODES {
            let t = e as f64 * 10.0;
            assert!(matches!(
                w.step(t, Some(true)),
                ThrustStep::Take { n: 1, .. }
            ));
            assert_eq!(w.step(t + 1.0, Some(false)), ThrustStep::Nothing);
        }
        assert_eq!(w.step(100.0, Some(true)), ThrustStep::GiveUp);
        // Unread is no measurement; armed (mode 0) is not engaged.
        let mut w = ThrustWatch::default();
        assert_eq!(w.step(0.0, None), ThrustStep::Nothing);
        assert_eq!(autothrottle_engaged(Some(1.0), Some(0.0)), Some(false));
        assert_eq!(autothrottle_engaged(Some(1.0), Some(1.0)), Some(true));
        assert_eq!(autothrottle_engaged(Some(0.0), Some(1.0)), Some(false));
        assert_eq!(autothrottle_engaged(None, Some(1.0)), None);
    }

    /// The 2026-10-05 LEMG 13 approach, replayed from its own record
    /// (`bench/approach/lemg13_737_2026-10-05.jsonl`): autoland armed at
    /// 13:02:39 with the A/T in SPEED 230 (the bitfield's 1 bit, left by the
    /// manual recovery) and it stayed engaged to 13:09:34, the lever read
    /// off autoland's command on nearly half those ticks while the speed sat
    /// on 230 to 240 through a descent that wanted it lower. The watch on
    /// the same ticks takes it on the FIRST one, says it once, and with a
    /// take that holds presses nothing more.
    #[test]
    fn the_lemg_737_arm_takes_the_autothrottle_on_its_first_tick() {
        let src = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/bench/approach/lemg13_737_2026-10-05.jsonl"
        ));
        let ticks: Vec<serde_json::Value> = src
            .lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
            .filter(|r| r.get("ias").is_some() && r["ph"] != "pilot_app")
            .collect();
        let engaged = |r: &serde_json::Value| {
            r["ap"]
                .as_f64()
                .and_then(Modes::from_read)
                .map(Modes::autothrottle)
        };
        let arm_t = ticks[0]["t"].as_f64().unwrap();
        let held: Vec<&serde_json::Value> =
            ticks.iter().filter(|r| engaged(r) == Some(true)).collect();
        let last_held = held.last().unwrap()["t"].as_f64().unwrap();
        assert!(
            last_held - arm_t > 400.0,
            "engaged {:.0} s after the arm",
            last_held - arm_t
        );
        let fought = held
            .iter()
            .filter(|r| match (r["lever"].as_f64(), r["thr"].as_f64()) {
                (Some(cmd), Some(read)) => (cmd - read).abs() > 0.04,
                _ => false,
            })
            .count();
        assert!(
            fought * 3 > held.len(),
            "the lever fought on {fought} of {} ticks",
            held.len()
        );
        // The watch on the recorded ticks: the first take at the first tick.
        let mut w = ThrustWatch::default();
        let first = ticks
            .iter()
            .find_map(|r| {
                let t = r["t"].as_f64().unwrap() - arm_t;
                matches!(w.step(t, engaged(r)), ThrustStep::Take { n: 1, say: true }).then_some(t)
            })
            .expect("taken");
        assert!(first < 1.0, "taken {first:.1} s after the arm");
        // A take that holds (the A/T reads armed from the next tick): nothing
        // more is pressed for the rest of the approach.
        let presses = ticks
            .iter()
            .skip(1)
            .filter(|r| {
                let t = r["t"].as_f64().unwrap() - arm_t;
                matches!(
                    w.step(t, Some(false).filter(|_| engaged(r).is_some())),
                    ThrustStep::Take { .. }
                )
            })
            .count();
        assert_eq!(presses, 0);
    }

    #[tokio::test]
    async fn a_take_presses_the_panels_autothrottle_off_and_says_it_once() {
        let fake = FakeXPlane::new();
        assert_eq!(
            act_thrust(&fake, ThrustStep::Take { n: 1, say: true }, "test").await,
            Some(THRUST_TAKEN_LINE)
        );
        assert_eq!(
            act_thrust(&fake, ThrustStep::Take { n: 2, say: false }, "test").await,
            None
        );
        assert_eq!(fake.ran_count(cmd::AT_OFF), 2);
        assert_eq!(
            act_thrust(&fake, ThrustStep::GiveUp, "test").await,
            Some(THRUST_STUCK_LINE)
        );
        assert_eq!(act_thrust(&fake, ThrustStep::Nothing, "test").await, None);
        assert_eq!(fake.ran_count(cmd::AT_OFF), 2);
        // The no-wait take: only when engaged.
        let fake = FakeXPlane::with_reads(&[(names::AT_ON, 1.0), (names::AT_MODE, 2.0)]);
        assert_eq!(drop_thrust_now(&fake, "test").await, Some(true));
        let fake = FakeXPlane::with_reads(&[(names::AT_ON, 0.0), (names::AT_MODE, 0.0)]);
        assert_eq!(drop_thrust_now(&fake, "test").await, None);
        assert!(!fake.ran(cmd::AT_OFF));
    }

    #[tokio::test]
    async fn the_one_shot_take_reads_presses_and_reads_back() {
        let fake = FakeXPlane::with_reads(&[(names::AT_ON, 1.0), (names::AT_MODE, 1.0)]);
        fake.set_on_command(cmd::AT_OFF, names::AT_MODE, 0.0);
        assert_eq!(take_thrust(&fake, "test").await, Some(true));
        assert_eq!(fake.ran_count(cmd::AT_OFF), 1);
        // Armed only: nothing pressed.
        let fake = FakeXPlane::with_reads(&[(names::AT_ON, 1.0), (names::AT_MODE, 0.0)]);
        assert_eq!(take_thrust(&fake, "test").await, None);
        assert!(!fake.ran(cmd::AT_OFF));
        // Stuck: three presses, then false.
        let fake = FakeXPlane::with_reads(&[(names::AT_ON, 1.0), (names::AT_MODE, 1.0)]);
        assert_eq!(take_thrust(&fake, "test").await, Some(false));
        assert_eq!(fake.ran_count(cmd::AT_OFF), THRUST_TRIES as usize);
    }

    /// APP is a toggle: armed (either half) is never pressed again.
    #[tokio::test]
    async fn the_approach_is_armed_only_from_off() {
        let fake = FakeXPlane::with_reads(&[(names::AP_STATE, 1330.0)]);
        assert_eq!(arm_approach(&fake, "test").await, Some(true));
        assert!(!fake.ran(cmd::APPROACH));
        let fake = FakeXPlane::with_reads(&[(names::AP_STATE, 50.0)]);
        fake.set_on_command(cmd::APPROACH, names::AP_STATE, 1330.0);
        assert_eq!(arm_approach(&fake, "test").await, Some(true));
        assert_eq!(fake.ran_count(cmd::APPROACH), 1);
        let fake = FakeXPlane::new();
        assert_eq!(arm_approach(&fake, "test").await, None);
        assert!(!fake.ran(cmd::APPROACH), "unread: nothing pressed");
    }

    /// The ledger holds what WE did, from when: a press and a dial write
    /// through the owner are on it, nothing before `since` is.
    #[tokio::test]
    async fn the_ledger_holds_our_presses_and_writes() {
        let fake = FakeXPlane::new();
        let t0 = std::time::Instant::now();
        let _ = press(&fake, Button::AltitudeHold).await;
        let _ = set_dial(&fake, Dial::Altitude, 4_321.0).await;
        let ours = Ours(ours_since(t0).into_iter().map(|(_, c)| c).collect());
        assert!(ours.pressed(Button::AltitudeHold));
        assert!(ours.wrote(Dial::Altitude, 4_321.0, 1.0));
        assert!(ours.mode_bits() & Modes::ALT_HOLD != 0);
        let later = std::time::Instant::now();
        assert!(!ours_since(later)
            .iter()
            .any(|(_, c)| *c == OurChange::Dial(Dial::Altitude, 4_321.0)));
    }

    /// The climb primitive: the window at the level, V/S on, the wheel after.
    #[tokio::test]
    async fn climb_to_sets_the_window_vs_and_wheel() {
        let fake = FakeXPlane::with_reads(&[
            (names::SERVOS_ON, 1.0),
            (names::VS_STATUS, 0.0),
            (crate::datarefs::AIRSPEED_KTS, 200.0),
            (Dial::Altitude.read_name(), 6_000.0),
        ]);
        fake.set_on_command(cmd::VERTICAL_SPEED, names::VS_STATUS, 2.0);
        fake.set_on_command(cmd::VERTICAL_SPEED, Dial::Vs.read_name(), 0.0);
        assert!(climb_to(&fake, 6_049.0, 1_500.0, "test").await);
        assert_eq!(fake.last_write(Dial::Altitude.read_name()), Some(6_000.0));
        assert_eq!(fake.last_write(Dial::Vs.read_name()), Some(1_500.0));
    }

    #[tokio::test]
    async fn hold_level_engages_sets_the_window_and_reads_the_hold_back() {
        let fake = FakeXPlane::with_reads(&[
            (names::SERVOS_ON, 0.0),
            (names::ALT_HOLD_STATUS, 0.0),
            (crate::datarefs::AIRSPEED_KTS, 200.0),
        ]);
        fake.set_on_command(cmd::SERVOS_ON, names::SERVOS_ON, 1.0);
        fake.set_on_command(cmd::ALTITUDE_HOLD, names::ALT_HOLD_STATUS, 2.0);
        assert!(hold_level(&fake, 9049.0, "test").await);
        assert_eq!(fake.last_write(Dial::Altitude.read_name()), Some(9000.0));
        assert_eq!(fake.ran_count(cmd::ALTITUDE_HOLD), 1);
        let fake = FakeXPlane::with_reads(&[
            (names::SERVOS_ON, 1.0),
            (names::HDG_STATUS, 2.0),
            (crate::datarefs::AIRSPEED_KTS, 200.0),
        ]);
        assert!(hold_heading(&fake, 365.0, "test").await);
        assert_eq!(fake.last_write(Dial::HeadingBug.read_name()), Some(5.0));
        assert!(!fake.ran(cmd::HEADING), "HDG already in: never toggled");
    }
}
