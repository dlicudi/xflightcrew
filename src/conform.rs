//! The ground conformance check, `xvl conform [--only <area>[,<area>]]`: on a
//! PARKED aircraft (on the ground, stopped, park brake set; refused otherwise,
//! with the reading that refused it) every control channel the live profile
//! declares is moved through the code's OWN seam, read back, and put back,
//! and each one gets a row: PASS, FAIL or SKIPPED with the numbers read. The
//! rows print as a markdown table and each is one blackbox `conform` record.
//!
//! # Why (the Zibo 737, 2026-10-07)
//!
//! Nine flights found what a two minute probe on the ground each would have:
//! the stock yoke ratios overwritten by the airframe (`yoke_input`), a generic
//! takeoff trim of 8.35 units (`takeoff_trim`), levers snapping to the
//! hardware throttle (`throttle_hardware_axis`), a stock beta reverse write
//! undone (`reverse_hold_command`), MCP buttons that are TOGGLES with lamps
//! that ramp in, a heading window that must be written and a V/S window with
//! a quantum. Each was a channel that did not do what the profile said, and
//! each read so on the ramp.
//!
//! # Rules
//!
//! - **The seam that flies is the seam tested**: the yoke through the plugin's
//!   channels with the airframe's route armed (`yoke::arm`), the flaps through
//!   `tools::flaps::set_handle_with`, the speedbrake through
//!   `tools::controls::speedbrake_set_with`, the windows through
//!   `autoflight::set_dial_on`, the modes through the panel owner's presses
//!   and `read_status_on`, the switches through `ensure_switch_on`, the park
//!   brake through `drive_park_brake`, the tiller through `tiller::Tiller`.
//! - **Every probe restores what it changed**: read first, put back, read
//!   back. A probe that could not put it back says so in its row.
//! - **Nothing moves the aircraft**: thrust only from idle and only to a
//!   small lever, reverse only a short hold, the park brake released only
//!   with the plugin's toe brakes held, and the groundspeed read between
//!   probes (a moving aircraft stops the check and sets the brake).
//! - **A failed read is a FAIL with "no reading"**, never a guess.
//!
//! The verdicts are pure functions; the probes run behind `dyn XPlane` so
//! they test against `FakeXPlane`.

use std::time::Duration;

use serde_json::{json, Value};

use crate::aircraft::{AircraftProfile, FlapProfile, SpeedbrakeLever, TakeoffTrimProfile};
use crate::autoflight::{self, Button, Dial, Panel, Switch, Window};
use crate::xplane::XPlane;

/// The plugin's pitch and roll channels (-1..1, aft and right positive), 1 s
/// heartbeats.
const PITCH: &str = "xcopilot/pitch";
const ROLL: &str = "xcopilot/roll";
/// The plugin's pedal channel (-1..1, right positive), a 1 s heartbeat.
const TAXI_STEER: &str = "xcopilot/taxi_steer";
/// The plugin's toe-brake squeeze (0..1), a 1 s heartbeat.
const TAXI_BRAKE: &str = "xcopilot/taxi_brake";

const ELEVATOR_DEG: &str = crate::datarefs::ELEVATOR1_DEG;
const AILERON_DEG: &str = "sim/flightmodel2/wing/aileron1_deg";
const TIRE_STEER_DEG: &str = "sim/flightmodel2/gear/tire_steer_command_deg";
const REVERSER_DEPLOY: &str = "sim/flightmodel2/engines/thrust_reverser_deploy_ratio";
const BETA_REV: &str = "sim/cockpit2/engine/actuators/throttle_beta_rev_ratio";
const ELEV_TRIM: &str = crate::datarefs::ELEVATOR_TRIM;
const ON_GROUND: &str = crate::datarefs::ON_GROUND_ANY;
const GS_MS: &str = crate::datarefs::GROUNDSPEED_MS;
const ENGINE_RUNNING: &str = crate::datarefs::ENGINE_RUNNING;
const TOTAL_WEIGHT_KG: &str = crate::datarefs::TOTAL_WEIGHT_KG;

/// A yoke or pedal channel command: half deflection each way.
const CHANNEL_CMD: f64 = 0.5;
/// How long a channel command is held before the surface is read, s.
const HOLD_S: f64 = 1.2;
/// The heartbeat of a held channel, s (the plugin expires it after 1 s).
const BEAT_S: f64 = 0.1;
/// How long a released channel is left to expire and centre, s.
const RELEASE_S: f64 = 1.5;
/// A surface moved when it moved this far from rest, degrees.
const SURFACE_MIN_DEG: f64 = 2.0;
/// A nosewheel moved when it moved this far from rest, degrees.
const STEER_MIN_DEG: f64 = 1.0;
/// A tiller with no declared range must turn the nose this far at half
/// deflection, degrees (more than the pedals give an airliner).
const TILLER_MIN_DEG: f64 = 5.0;
/// A released surface is back when within this of where it rested, degrees.
const CENTRED_DEG: f64 = 1.0;

/// A thrust lever at or under this is at idle.
const IDLE_MAX: f64 = 0.05;
/// The thrust probe's lever: a crack off idle, against the park brake.
const PROBE_THRUST: f64 = 0.08;
/// A lever reads where it was sent within this.
const LEVER_TOL: f64 = 0.03;

/// The reverse hold, s: short (the Zibo ran N1 19 to 70 % in about 3 s).
const REVERSE_HOLD_S: f64 = 1.0;
/// The reverse is read this long into the hold, s.
const REVERSE_READ_S: f64 = 0.8;
/// The stow is read this long after the hold ended, s.
const STOW_WAIT_S: f64 = 2.5;
/// The stock reverse probe's lever: just into reverse (-1..-2).
const STOCK_REVERSE: f64 = -1.2;

/// A mode lamp is waited on this long after a press (lamps ramp in), s.
const MODE_SETTLE_S: f64 = 2.0;
/// And polled this often, s.
const MODE_POLL_S: f64 = 0.2;
/// A window is read back this long after it is set, s.
const WINDOW_SETTLE_S: f64 = 0.5;

/// The trim switch's blip, s.
const TRIM_BLIP_S: f64 = 1.0;
/// A trim blip moved the stabilizer when it moved this far, units.
const TRIM_MIN_MOVE: f64 = 0.02;
/// Holds the trim restore takes at most.
const TRIM_RESTORE_HOLDS: usize = 4;
/// The stock trim probe's step (ratio, -1..1) and its readback tolerance.
const STOCK_TRIM_STEP: f64 = 0.1;
const STOCK_TRIM_TOL: f64 = 0.02;

/// A flap handle reads at its detent within this.
const FLAP_TOL: f64 = crate::aircraft::FLAP_DETENT_TOLERANCE;
/// The speedbrake in the stock sense reads where it was sent within this.
const SPEEDBRAKE_TOL: f64 = 0.05;

/// What a channel's check came to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Verdict {
    Pass,
    Fail,
    Skipped,
}

impl Verdict {
    pub fn as_str(self) -> &'static str {
        match self {
            Verdict::Pass => "PASS",
            Verdict::Fail => "FAIL",
            Verdict::Skipped => "SKIPPED",
        }
    }
}

/// One channel's row: the verdict, a line with the numbers, and the reads.
#[derive(Debug, Clone, PartialEq)]
pub struct Row {
    pub channel: String,
    pub verdict: Verdict,
    pub detail: String,
    pub reads: Value,
}

impl Row {
    fn new(channel: impl Into<String>, (verdict, detail): (Verdict, String), reads: Value) -> Self {
        Self {
            channel: channel.into(),
            verdict,
            detail,
            reads,
        }
    }

    fn fail(channel: impl Into<String>, detail: impl Into<String>) -> Self {
        Self::new(channel, (Verdict::Fail, detail.into()), Value::Null)
    }

    fn skipped(channel: impl Into<String>, detail: impl Into<String>) -> Self {
        Self::new(channel, (Verdict::Skipped, detail.into()), Value::Null)
    }

    /// The markdown table's line. Pure.
    pub fn table_line(&self) -> String {
        format!(
            "| {} | {} | {} |",
            self.channel,
            self.verdict.as_str(),
            self.detail.replace('|', "/")
        )
    }
}

/// The table's header lines.
pub const TABLE_HEAD: [&str; 2] = ["| Channel | Verdict | Detail |", "|---|---|---|"];

// ---------------------------------------------------------------------------
// The areas and the plan.
// ---------------------------------------------------------------------------

/// What `--only` picks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Area {
    Profile,
    Mcp,
    Flaps,
    Speedbrake,
    Trim,
    Yoke,
    Steer,
    Throttle,
    Reverse,
    Brake,
}

impl Area {
    pub const ALL: [Area; 10] = [
        Area::Profile,
        Area::Mcp,
        Area::Flaps,
        Area::Speedbrake,
        Area::Trim,
        Area::Yoke,
        Area::Steer,
        Area::Throttle,
        Area::Reverse,
        Area::Brake,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Area::Profile => "profile",
            Area::Mcp => "mcp",
            Area::Flaps => "flaps",
            Area::Speedbrake => "speedbrake",
            Area::Trim => "trim",
            Area::Yoke => "yoke",
            Area::Steer => "steer",
            Area::Throttle => "throttle",
            Area::Reverse => "reverse",
            Area::Brake => "brake",
        }
    }

    pub fn parse(s: &str) -> Option<Area> {
        let s = s.trim().to_ascii_lowercase();
        Self::ALL
            .into_iter()
            .find(|a| a.name() == s)
            .or(match s.as_str() {
                "autopilot" | "ap" => Some(Area::Mcp),
                "tiller" => Some(Area::Steer),
                "park_brake" | "parking_brake" => Some(Area::Brake),
                _ => None,
            })
    }

    /// `--only a,b`: the areas named, or an error naming the one not known.
    pub fn parse_list(s: &str) -> Result<Vec<Area>, String> {
        s.split(',')
            .filter(|t| !t.trim().is_empty())
            .map(|t| {
                Area::parse(t).ok_or_else(|| {
                    format!(
                        "unknown area '{t}' (one of: {})",
                        Self::ALL.map(Area::name).join(", ")
                    )
                })
            })
            .collect()
    }
}

/// How a mode's state is read: a stock status as this panel has it
/// (`autoflight::read_status_on`), or a declared lamp.
#[derive(Debug, Clone, PartialEq)]
pub enum ModeRead {
    Stock(&'static str),
    Lamp(String),
}

/// One mode button and how its state is read.
#[derive(Debug, Clone, PartialEq)]
pub struct ModeProbe {
    pub button: Button,
    pub label: &'static str,
    pub read: ModeRead,
}

/// The mode buttons a panel can be checked on: a declared lamp panel's lit
/// lamps that have a button, else the stock statuses the owner reads. The
/// CMD engage is not a mode and is never pressed here. Pure.
pub fn mode_probes(p: &Panel) -> Vec<ModeProbe> {
    use autoflight::{names, Lamp};
    if p.lamps.is_some() {
        return p
            .lamp_names()
            .into_iter()
            .filter_map(|(lamp, name)| {
                let (button, label) = match lamp {
                    Lamp::Heading => (Button::Heading, "heading"),
                    Lamp::Vs => (Button::VerticalSpeed, "vs"),
                    Lamp::AltitudeHold => (Button::AltitudeHold, "altitude_hold"),
                    Lamp::LevelChange => (Button::LevelChange, "level_change"),
                    Lamp::Approach => (Button::Approach, "approach"),
                    Lamp::Lnav => (Button::Lnav, "lnav"),
                    _ => return None,
                };
                Some(ModeProbe {
                    button,
                    label,
                    read: ModeRead::Lamp(name),
                })
            })
            .collect();
    }
    let mut out = vec![
        ModeProbe {
            button: Button::Heading,
            label: "heading",
            read: ModeRead::Stock(names::HDG_STATUS),
        },
        ModeProbe {
            button: Button::VerticalSpeed,
            label: "vs",
            read: ModeRead::Stock(names::VS_STATUS),
        },
        ModeProbe {
            button: Button::AltitudeHold,
            label: "altitude_hold",
            read: ModeRead::Stock(names::ALT_HOLD_STATUS),
        },
    ];
    if let (true, Some(lamp)) = (p.lnav_declared(), p.lnav_status.clone()) {
        out.push(ModeProbe {
            button: Button::Lnav,
            label: "lnav",
            read: ModeRead::Lamp(lamp),
        });
    }
    out
}

/// The MCP windows checked, with their row names.
pub const WINDOWS: [(Dial, &str); 4] = [
    (Dial::HeadingBug, "heading"),
    (Dial::Altitude, "altitude"),
    (Dial::Vs, "vs"),
    (Dial::Airspeed, "airspeed"),
];

/// The panel switches checked, with their row names.
pub const SWITCHES: [(Switch, &str); 3] = [
    (Switch::FlightDirector, "flight_director"),
    (Switch::FlightDirectorFo, "flight_director_fo"),
    (Switch::AutothrottleArm, "autothrottle_arm"),
];

/// One step of the check.
#[derive(Debug, Clone, PartialEq)]
pub enum Step {
    Profile,
    Window(Dial, &'static str),
    Mode(ModeProbe),
    Switch(Switch, &'static str),
    Flaps,
    Speedbrake,
    Trim,
    Yoke {
        pitch: bool,
    },
    Steer,
    Tiller,
    Throttle,
    Reverse,
    ParkBrake,
    /// A channel the profile does not declare: a SKIPPED row.
    Skip {
        channel: String,
        why: String,
    },
}

/// The steps for a profile, in the order they run (nothing that moves a
/// lever before what only reads; the thrust and the brake last), limited to
/// `only` when it names anything. Pure.
pub fn plan(p: &AircraftProfile, only: &[Area]) -> Vec<Step> {
    let want = |a: Area| only.is_empty() || only.contains(&a);
    let skip = |channel: &str, why: &str| Step::Skip {
        channel: channel.to_string(),
        why: why.to_string(),
    };
    let mut out = Vec::new();
    if want(Area::Profile) {
        out.push(Step::Profile);
    }
    if want(Area::Mcp) {
        if p.autopilot {
            let panel = Panel::from_profile(p);
            for (d, name) in WINDOWS {
                out.push(Step::Window(d, name));
            }
            out.extend(mode_probes(&panel).into_iter().map(Step::Mode));
            for (s, name) in SWITCHES {
                if panel.switch(s).is_some() {
                    out.push(Step::Switch(s, name));
                }
            }
        } else {
            out.push(skip("mcp", "no autopilot declared"));
        }
    }
    if want(Area::Flaps) {
        out.push(if p.flaps.as_ref().is_some_and(|f| !f.detents.is_empty()) {
            Step::Flaps
        } else {
            skip("flaps", "no flap detents declared")
        });
    }
    if want(Area::Speedbrake) {
        out.push(if p.speedbrake {
            Step::Speedbrake
        } else {
            skip("speedbrake", "no speedbrake declared")
        });
    }
    if want(Area::Trim) {
        out.push(if p.takeoff_trim.is_some() || p.pitch_trim {
            Step::Trim
        } else {
            skip("trim", "no pitch trim declared")
        });
    }
    if want(Area::Yoke) {
        out.push(Step::Yoke { pitch: true });
        out.push(Step::Yoke { pitch: false });
    }
    if want(Area::Steer) {
        out.push(Step::Steer);
        out.push(if p.taxi.as_ref().is_some_and(|t| t.tiller.is_some()) {
            Step::Tiller
        } else {
            skip("tiller", "no tiller declared")
        });
    }
    if want(Area::Throttle) {
        out.push(if p.throttle {
            Step::Throttle
        } else {
            skip("throttle", "no throttle declared")
        });
    }
    if want(Area::Reverse) {
        out.push(if p.reverse_thrust {
            Step::Reverse
        } else {
            skip("reverse", "no reverse declared")
        });
    }
    if want(Area::Brake) {
        out.push(if p.parking_brake {
            Step::ParkBrake
        } else {
            skip("park_brake", "no parking brake declared")
        });
    }
    out
}

// ---------------------------------------------------------------------------
// Pure verdicts.
// ---------------------------------------------------------------------------

/// A number for the table: whole above 100, one place above 10, else two.
pub fn num(v: f64) -> String {
    if !v.is_finite() {
        return format!("{v}");
    }
    if v.abs() >= 100.0 {
        format!("{v:.0}")
    } else if v.abs() >= 10.0 {
        format!("{v:.1}")
    } else {
        format!("{v:.2}")
    }
}

fn read_str(v: Option<f64>) -> String {
    v.map_or_else(|| "no reading".to_string(), num)
}

/// How far apart two readings are; headings the short way round. Pure.
pub fn gap(a: f64, b: f64, wrap: bool) -> f64 {
    if wrap {
        let d = (a - b).rem_euclid(360.0);
        d.min(360.0 - d)
    } else {
        (a - b).abs()
    }
}

/// A setting sent and put back: `got` must read `want` and `back` read
/// `was`, each within `tol`. Pure.
pub fn set_restore_verdict(
    want: f64,
    got: Option<f64>,
    was: f64,
    back: Option<f64>,
    tol: f64,
    wrap: bool,
) -> (Verdict, String) {
    let Some(got) = got else {
        return (
            Verdict::Fail,
            format!("no reading after setting {} (was {})", num(want), num(was)),
        );
    };
    if gap(got, want, wrap) > tol {
        let restored = back.is_some_and(|b| gap(b, was, wrap) <= tol);
        return (
            Verdict::Fail,
            format!(
                "did not take: sent {}, read {} (was {}; {})",
                num(want),
                num(got),
                num(was),
                if restored { "restored" } else { "NOT restored" }
            ),
        );
    }
    match back {
        None => (
            Verdict::Fail,
            format!(
                "took {} but no reading after restoring {}",
                num(got),
                num(was)
            ),
        ),
        Some(b) if gap(b, was, wrap) > tol => (
            Verdict::Fail,
            format!(
                "took {} but NOT restored: sent {}, read {}",
                num(got),
                num(was),
                num(b)
            ),
        ),
        Some(b) => (
            Verdict::Pass,
            format!("{} -> {} -> {}", num(was), num(got), num(b)),
        ),
    }
}

/// The element that moved furthest between two readings of a per-wing or
/// per-gear array, and how far (signed). `None` without two readings. Pure.
pub fn largest_delta(base: &[f64], now: &[f64]) -> Option<(usize, f64)> {
    base.iter()
        .zip(now)
        .enumerate()
        .filter(|(_, (b, n))| b.is_finite() && n.is_finite())
        .map(|(i, (b, n))| (i, n - b))
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
}

/// One element's change between two readings. Pure.
pub fn delta_at(base: &[f64], now: &[f64], i: usize) -> Option<f64> {
    let d = now.get(i)? - base.get(i)?;
    d.is_finite().then_some(d)
}

/// A channel commanded one way then the other, then released: the surface
/// (the element `plus` found moving furthest) must move at least `min_deg`
/// each way, the two ways opposite, `sense` (the sign the positive command
/// must give, where one is known) obeyed, and come back within
/// [`CENTRED_DEG`] of rest. `plus`, `minus` and `back` are the changes from
/// rest. Pure.
pub fn surface_verdict(
    plus: Option<(usize, f64)>,
    minus: Option<f64>,
    back: Option<f64>,
    min_deg: f64,
    sense: Option<f64>,
) -> (Verdict, String) {
    let Some((i, p)) = plus else {
        return (Verdict::Fail, "no reading with the channel held".into());
    };
    let Some(m) = minus else {
        return (
            Verdict::Fail,
            "no reading with the channel held the other way".into(),
        );
    };
    let moved = format!(
        "+{CHANNEL_CMD} gave {:+.1} deg, -{CHANNEL_CMD} gave {m:+.1} deg [{i}]",
        p
    );
    if p.abs() < min_deg || m.abs() < min_deg {
        return (
            Verdict::Fail,
            format!("did not move {min_deg} deg each way: {moved}"),
        );
    }
    if p.signum() == m.signum() {
        return (
            Verdict::Fail,
            format!("moved the same way both ways: {moved}"),
        );
    }
    if let Some(s) = sense {
        if p.signum() != s.signum() {
            return (Verdict::Fail, format!("REVERSED sense: {moved}"));
        }
    }
    match back {
        None => (Verdict::Fail, format!("{moved}; no reading after release")),
        Some(b) if b.abs() > CENTRED_DEG => (
            Verdict::Fail,
            format!("{moved}; did not come back on release ({b:+.1} deg from rest)"),
        ),
        Some(_) => (Verdict::Pass, format!("{moved}; back on release")),
    }
}

/// A button pressed twice: the state must change on the first press and
/// come back on the second (a toggle, left where it started). Pure.
pub fn toggle_verdict(
    before: Option<bool>,
    first: Option<bool>,
    second: Option<bool>,
) -> (Verdict, String) {
    let s = |v: bool| if v { "on" } else { "off" };
    let Some(b) = before else {
        return (Verdict::Fail, "no reading before the press".into());
    };
    let Some(f) = first else {
        return (
            Verdict::Fail,
            format!("{} before; no reading after the press", s(b)),
        );
    };
    if f == b {
        return (
            Verdict::Fail,
            format!("stayed {} after a press (did not take)", s(b)),
        );
    }
    match second {
        None => (
            Verdict::Fail,
            format!("{} -> {}; no reading after the second press", s(b), s(f)),
        ),
        Some(x) if x != b => (
            Verdict::Fail,
            format!(
                "{} -> {} -> {}: not a toggle, left {}",
                s(b),
                s(f),
                s(x),
                s(x)
            ),
        ),
        Some(x) => (
            Verdict::Pass,
            format!("toggle: {} -> {} -> {}", s(b), s(f), s(x)),
        ),
    }
}

/// A lever sent off idle, read early and late (a lever an add-on snaps back
/// reads right once and wrong a second later), then put back. Pure.
pub fn lever_held_verdict(
    want: f64,
    early: Option<f64>,
    late: Option<f64>,
    was: f64,
    back: Option<f64>,
) -> (Verdict, String) {
    let (Some(e), Some(l)) = (early, late) else {
        return (
            Verdict::Fail,
            format!(
                "no reading with the lever at {}: early {}, late {}",
                num(want),
                read_str(early),
                read_str(late)
            ),
        );
    };
    if (e - want).abs() > LEVER_TOL {
        return set_restore_verdict(want, Some(e), was, back, LEVER_TOL, false);
    }
    if (l - want).abs() > LEVER_TOL {
        let restored = back.is_some_and(|b| (b - was).abs() <= LEVER_TOL);
        return (
            Verdict::Fail,
            format!(
                "took {} then went to {} (snapped back?); {}",
                num(e),
                num(l),
                if restored { "restored" } else { "NOT restored" }
            ),
        );
    }
    set_restore_verdict(want, Some(l), was, back, LEVER_TOL, false)
}

/// A reverse hold: deployed while held (deploy ratio from 0.5 or the lever
/// in the reverse range), stowed after. Not deployed with no engine running
/// is SKIPPED: a reverser needs the engines' hydraulics. Pure.
pub fn reverse_verdict(
    deploy: Option<f64>,
    beta: Option<f64>,
    stow_deploy: Option<f64>,
    stow_beta: Option<f64>,
    base_beta: f64,
    running: Option<bool>,
) -> (Verdict, String) {
    if deploy.is_none() && beta.is_none() {
        return (
            Verdict::Fail,
            "no reading of the reversers or the lever while held".into(),
        );
    }
    let held = format!(
        "held: deploy {}, lever {}",
        read_str(deploy),
        read_str(beta)
    );
    let deployed = deploy.is_some_and(|d| d >= 0.5) || beta.is_some_and(|b| b <= -1.0);
    if !deployed {
        if running == Some(false) {
            return (
                Verdict::Skipped,
                format!("no engine running, so no hydraulics to deploy: {held}"),
            );
        }
        return (Verdict::Fail, format!("did not deploy: {held}"));
    }
    if stow_deploy.is_none() && stow_beta.is_none() {
        return (Verdict::Fail, format!("{held}; no reading after the stow"));
    }
    let stowed = stow_deploy.map_or(true, |d| d <= 0.1)
        && stow_beta.map_or(true, |b| b >= -0.1 || (b - base_beta).abs() <= 0.1);
    let after = format!(
        "after: deploy {}, lever {}",
        read_str(stow_deploy),
        read_str(stow_beta)
    );
    if stowed {
        (Verdict::Pass, format!("{held}; {after}"))
    } else {
        (Verdict::Fail, format!("{held}; did NOT stow, {after}"))
    }
}

/// A trim switch blipped one way (`sense` +1 nose up) and the stabilizer put
/// back: it must move at least [`TRIM_MIN_MOVE`] that way and come back
/// within `tol`. Pure.
pub fn trim_blip_verdict(
    was: f64,
    after: Option<f64>,
    back: Option<f64>,
    sense: f64,
    tol: f64,
) -> (Verdict, String) {
    let way = if sense > 0.0 { "nose up" } else { "nose down" };
    let Some(a) = after else {
        return (Verdict::Fail, format!("no reading after the {way} blip"));
    };
    let moved = (a - was) * sense.signum();
    if moved < TRIM_MIN_MOVE {
        return (
            Verdict::Fail,
            format!("{way} blip moved {} -> {} (did not take)", num(was), num(a)),
        );
    }
    match back {
        None => (
            Verdict::Fail,
            format!(
                "{way} {} -> {}; no reading after restoring",
                num(was),
                num(a)
            ),
        ),
        Some(b) if (b - was).abs() > tol => (
            Verdict::Fail,
            format!(
                "{way} {} -> {}; NOT restored, read {}",
                num(was),
                num(a),
                num(b)
            ),
        ),
        Some(b) => (
            Verdict::Pass,
            format!("{way} {} -> {} -> {}", num(was), num(a), num(b)),
        ),
    }
}

/// The check may run: on the ground, stopped, the park brake set. `Err` is
/// the refusal with the reading that refused it. Pure.
pub fn parked(
    on_ground: Option<f64>,
    gs_mps: Option<f64>,
    park: Option<f64>,
) -> Result<String, String> {
    let Some(g) = on_ground else {
        return Err("no reading of on-ground".into());
    };
    if !sim::ground::on_ground(g) {
        return Err("not on the ground".into());
    }
    let Some(gs) = gs_mps.filter(|v| v.is_finite()) else {
        return Err("no reading of the groundspeed".into());
    };
    let kts = gs.abs() * sim::ground::MS_TO_KTS;
    if !sim::ground::stopped_kts(kts) {
        return Err(format!("moving at {} kt", num(kts)));
    }
    let Some(b) = park.filter(|v| v.is_finite()) else {
        return Err("no reading of the park brake".into());
    };
    if b < 0.5 {
        return Err(format!("park brake not set (reads {})", num(b)));
    }
    Ok(format!(
        "on the ground, {} kt, park brake {}",
        num(kts),
        num(b)
    ))
}

/// The value a window is tested at: away from where it reads, inside what
/// any MCP shows. Pure.
pub fn window_test_value(d: Dial, was: f64) -> f64 {
    match d {
        Dial::HeadingBug => (was + 20.0).rem_euclid(360.0),
        Dial::Altitude => {
            if was > 30_000.0 {
                was - 1_000.0
            } else {
                was + 1_000.0
            }
        }
        Dial::Vs | Dial::VviFpm => {
            if was > 2_000.0 {
                was - 500.0
            } else {
                was + 500.0
            }
        }
        Dial::Airspeed | Dial::AirspeedKts => {
            if was < 1.0 {
                // A Mach window: a hundredth.
                was + 0.01
            } else if was > 300.0 {
                was - 10.0
            } else {
                was + 10.0
            }
        }
        Dial::HoverVector => was,
    }
}

/// How close a window must read: its knob's step, a written window's
/// quantum, else a unit; a Mach window (under 1) a thousandth. Pure.
pub fn window_tol(w: Option<&Window>, d: Dial, was: f64) -> f64 {
    if matches!(d, Dial::Airspeed | Dial::AirspeedKts) && was < 1.0 {
        return 0.001;
    }
    match w {
        Some(Window::Knob { step, .. }) => step.max(1.0),
        Some(Window::Write {
            quantum: Some(q), ..
        }) => (q / 2.0).max(1.0),
        _ => 1.0,
    }
}

// ---------------------------------------------------------------------------
// Reads and holds.
// ---------------------------------------------------------------------------

async fn sleep_s(s: f64) {
    tokio::time::sleep(Duration::from_secs_f64(s)).await;
}

async fn read(xp: &dyn XPlane, name: &str) -> Option<f64> {
    xp.get_float(name).await.ok().filter(|v| v.is_finite())
}

async fn read_array(xp: &dyn XPlane, name: &str) -> Option<Vec<f64>> {
    xp.get_float_array(name)
        .await
        .ok()
        .filter(|v| !v.is_empty() && v.iter().all(|x| x.is_finite()))
}

/// A channel the probe holds: a plugin heartbeat, or the airframe's tiller.
enum Channel<'a> {
    Plain(&'static str),
    Tiller(&'a crate::tiller::Tiller),
}

impl Channel<'_> {
    async fn write(&self, xp: &dyn XPlane, v: f64) {
        match self {
            Channel::Plain(name) => {
                let _ = xp.set_float(name, v).await;
            }
            Channel::Tiller(t) => t.write(xp, v, 0.0).await,
        }
    }

    async fn release(&self, xp: &dyn XPlane) {
        match self {
            Channel::Plain(name) => {
                let _ = xp.set_float(name, 0.0).await;
            }
            Channel::Tiller(t) => t.release(xp).await,
        }
    }
}

/// Hold `v` on the channel for [`HOLD_S`] and read `surface` while it is
/// still held.
async fn hold_and_read(
    xp: &dyn XPlane,
    ch: &Channel<'_>,
    v: f64,
    surface: &str,
) -> Option<Vec<f64>> {
    let beats = (HOLD_S / BEAT_S).round() as usize;
    for _ in 0..beats {
        ch.write(xp, v).await;
        sleep_s(BEAT_S).await;
    }
    ch.write(xp, v).await;
    read_array(xp, surface).await
}

/// A channel held each way and released, `surface` read at rest, held and
/// after: the changes [`surface_verdict`] judges, and the reads.
async fn swing(
    xp: &dyn XPlane,
    ch: &Channel<'_>,
    surface: &str,
) -> (Option<(usize, f64)>, Option<f64>, Option<f64>, Value) {
    let Some(base) = read_array(xp, surface).await else {
        return (None, None, None, json!({"surface": surface, "base": null}));
    };
    let plus = hold_and_read(xp, ch, CHANNEL_CMD, surface).await;
    let minus = hold_and_read(xp, ch, -CHANNEL_CMD, surface).await;
    ch.release(xp).await;
    sleep_s(RELEASE_S).await;
    let after = read_array(xp, surface).await;
    let p = plus.as_ref().and_then(|v| largest_delta(&base, v));
    let i = p.map(|(i, _)| i);
    let at = |v: &Option<Vec<f64>>| i.and_then(|i| v.as_ref().and_then(|v| delta_at(&base, v, i)));
    let reads = json!({"surface": surface, "base": base, "plus": plus, "minus": minus,
                       "after": after, "element": i});
    (p, at(&minus), at(&after), reads)
}

// ---------------------------------------------------------------------------
// The probes.
// ---------------------------------------------------------------------------

/// The yoke on the plugin's channel (the route already armed): the elevator
/// for pitch, aft must give trailing edge UP (negative), the ailerons for
/// roll.
pub async fn probe_yoke(xp: &dyn XPlane, pitch: bool) -> Row {
    let (channel, surface, label, sense) = if pitch {
        (PITCH, ELEVATOR_DEG, "yoke_pitch", Some(-1.0))
    } else {
        (ROLL, AILERON_DEG, "yoke_roll", None)
    };
    let (p, m, b, reads) = swing(xp, &Channel::Plain(channel), surface).await;
    if reads["base"].is_null() {
        return Row::new(
            label,
            (Verdict::Fail, format!("no reading of {surface}")),
            reads,
        );
    }
    Row::new(
        label,
        surface_verdict(p, m, b, SURFACE_MIN_DEG, sense),
        reads,
    )
}

/// The pedal channel, read on the nosewheel's steering command.
pub async fn probe_steer(xp: &dyn XPlane) -> Row {
    let (p, m, b, reads) = swing(xp, &Channel::Plain(TAXI_STEER), TIRE_STEER_DEG).await;
    if reads["base"].is_null() {
        return Row::new(
            "taxi_steer",
            (Verdict::Fail, format!("no reading of {TIRE_STEER_DEG}")),
            reads,
        );
    }
    Row::new(
        "taxi_steer",
        surface_verdict(p, m, b, STEER_MIN_DEG, None),
        reads,
    )
}

/// The airframe's tiller (armed), read on the nosewheel: half deflection
/// must turn it at least half its declared range's half, else
/// [`TILLER_MIN_DEG`].
pub async fn probe_tiller(
    xp: &dyn XPlane,
    tiller: &crate::tiller::Tiller,
    max_deg: Option<f64>,
) -> Row {
    let (p, m, b, reads) = swing(xp, &Channel::Tiller(tiller), TIRE_STEER_DEG).await;
    if reads["base"].is_null() {
        return Row::new(
            "tiller",
            (Verdict::Fail, format!("no reading of {TIRE_STEER_DEG}")),
            reads,
        );
    }
    let min = max_deg
        .filter(|d| d.is_finite() && *d > 0.0)
        .map_or(TILLER_MIN_DEG, |d| d * CHANNEL_CMD * 0.5);
    Row::new("tiller", surface_verdict(p, m, b, min, None), reads)
}

/// The thrust lever: from idle only, to [`PROBE_THRUST`] and back. The
/// hardware axis, where declared, is read for the row.
pub async fn probe_throttle(xp: &dyn XPlane, lever: &str, hardware: Option<&str>) -> Row {
    let Some(was) = read(xp, lever).await else {
        return Row::fail("throttle", format!("no reading of {lever}"));
    };
    if was > IDLE_MAX {
        return Row::skipped(
            "throttle",
            format!("levers not at idle ({}): not moved", num(was)),
        );
    }
    let hw = match hardware {
        Some(h) => read(xp, h).await,
        None => None,
    };
    let _ = xp.set_float(lever, PROBE_THRUST).await;
    sleep_s(0.4).await;
    let early = read(xp, lever).await;
    sleep_s(1.1).await;
    let late = read(xp, lever).await;
    let _ = xp.set_float(lever, was).await;
    sleep_s(0.4).await;
    let back = read(xp, lever).await;
    let (v, mut d) = lever_held_verdict(PROBE_THRUST, early, late, was, back);
    if hardware.is_some() {
        d.push_str(&format!("; hardware axis {}", read_str(hw)));
    }
    Row::new(
        "throttle",
        (v, d),
        json!({"lever": lever, "was": was, "want": PROBE_THRUST, "early": early, "late": late,
               "back": back, "hardware": hardware, "hardware_read": hw}),
    )
}

/// The reverse: from idle only, a short hold of the airframe's reverse
/// command, or the stock lever just into reverse and put back; read while
/// held and after the stow.
pub async fn probe_reverse(
    xp: &dyn XPlane,
    lever: &str,
    hold: Option<&str>,
    engines: usize,
) -> Row {
    let engines = engines.max(1);
    match read(xp, lever).await {
        None => return Row::fail("reverse", format!("no reading of {lever}")),
        Some(t) if t > IDLE_MAX => {
            return Row::skipped(
                "reverse",
                format!("levers not at idle ({}): not moved", num(t)),
            )
        }
        Some(_) => {}
    }
    let Some(base) = read_array(xp, BETA_REV).await else {
        return Row::fail("reverse", format!("no reading of {BETA_REV}"));
    };
    let running = read_array(xp, ENGINE_RUNNING)
        .await
        .map(|r| r.iter().take(engines).any(|v| *v > 0.5));
    let first = |v: Option<Vec<f64>>| v.and_then(|v| v.first().copied());
    let (deploy, beta) = match hold {
        Some(cmd) => {
            let _ = xp.run_command_for(cmd, REVERSE_HOLD_S).await;
            sleep_s(REVERSE_READ_S).await;
            let d = first(read_array(xp, REVERSER_DEPLOY).await);
            let b = first(read_array(xp, BETA_REV).await);
            sleep_s((REVERSE_HOLD_S - REVERSE_READ_S).max(0.0)).await;
            (d, b)
        }
        None => {
            for i in 0..engines.min(base.len()) {
                let _ = xp.set_float_array_elem(BETA_REV, i, STOCK_REVERSE).await;
            }
            sleep_s(REVERSE_READ_S).await;
            let d = first(read_array(xp, REVERSER_DEPLOY).await);
            let b = first(read_array(xp, BETA_REV).await);
            for (i, v) in base.iter().enumerate().take(engines) {
                let _ = xp.set_float_array_elem(BETA_REV, i, *v).await;
            }
            (d, b)
        }
    };
    sleep_s(STOW_WAIT_S).await;
    let stow_d = first(read_array(xp, REVERSER_DEPLOY).await);
    let stow_b = first(read_array(xp, BETA_REV).await);
    let verdict = reverse_verdict(deploy, beta, stow_d, stow_b, base[0], running);
    Row::new(
        "reverse",
        verdict,
        json!({"hold": hold, "base_lever": base, "deploy": deploy, "lever": beta,
               "stow_deploy": stow_d, "stow_lever": stow_b, "engines_running": running}),
    )
}

/// The flap handle through its seam: the next detent and back.
pub async fn probe_flaps(xp: &dyn XPlane, flaps: &FlapProfile) -> Row {
    use crate::tools::flaps::{read_handle_with, set_handle_with};
    let by = flaps.by_command.as_ref();
    let Some(was) = read_handle_with(xp, by)
        .await
        .ok()
        .filter(|v| v.is_finite())
    else {
        return Row::fail("flaps", "no reading of the flap handle");
    };
    let mut ratios: Vec<f64> = flaps.detents.iter().map(|d| d.ratio).collect();
    ratios.sort_by(f64::total_cmp);
    let want = ratios
        .iter()
        .copied()
        .find(|r| *r > was + FLAP_TOL)
        .or_else(|| ratios.iter().copied().rev().find(|r| *r < was - FLAP_TOL));
    let Some(want) = want else {
        return Row::skipped("flaps", "a single detent: nothing to move to");
    };
    let sent = set_handle_with(xp, by, want).await;
    sleep_s(0.3).await;
    let got = read_handle_with(xp, by)
        .await
        .ok()
        .filter(|v| v.is_finite());
    let restored = set_handle_with(xp, by, was).await;
    sleep_s(0.3).await;
    let back = read_handle_with(xp, by)
        .await
        .ok()
        .filter(|v| v.is_finite());
    let (v, mut d) = set_restore_verdict(want, got, was, back, FLAP_TOL, false);
    for (what, r) in [("set", &sent), ("restore", &restored)] {
        if let Err(e) = r {
            d.push_str(&format!("; {what}: {e:#}"));
        }
    }
    Row::new(
        "flaps",
        (v, d),
        json!({"by_command": by.is_some(), "was": was, "want": want, "got": got, "back": back}),
    )
}

/// The speedbrake through its seam, in the stock sense: the flight detent
/// (or down, from there) and back.
pub async fn probe_speedbrake(xp: &dyn XPlane, lever: Option<&SpeedbrakeLever>) -> Row {
    use crate::tools::controls::{speedbrake_read_with, speedbrake_set_with};
    let Some(was) = speedbrake_read_with(xp, lever)
        .await
        .ok()
        .filter(|v| v.is_finite())
    else {
        return Row::fail("speedbrake", "no reading of the speedbrake");
    };
    let want = if (was - 0.5).abs() <= 0.1 { 0.0 } else { 0.5 };
    let sent = speedbrake_set_with(xp, lever, want).await;
    sleep_s(0.3).await;
    let got = speedbrake_read_with(xp, lever)
        .await
        .ok()
        .filter(|v| v.is_finite());
    let restored = speedbrake_set_with(xp, lever, was).await;
    sleep_s(0.3).await;
    let back = speedbrake_read_with(xp, lever)
        .await
        .ok()
        .filter(|v| v.is_finite());
    let (v, mut d) = set_restore_verdict(want, got, was, back, SPEEDBRAKE_TOL, false);
    for (what, r) in [("set", &sent), ("restore", &restored)] {
        if let Err(e) = r {
            d.push_str(&format!("; {what}: {e:#}"));
        }
    }
    Row::new(
        "speedbrake",
        (v, d),
        json!({"lever": lever.map(|l| l.readback.clone()), "was": was, "want": want, "got": got,
               "back": back}),
    )
}

/// The takeoff trim: on a declared table, the switch blipped and the
/// stabilizer stepped back on the switches (`takeoff_trim::next_hold`), the
/// table's figure for this weight and CG beside the gauge; else the stock
/// trim ratio written a step and back.
pub async fn probe_trim(xp: &dyn XPlane, table: Option<&TakeoffTrimProfile>) -> Row {
    let Some(t) = table else {
        let Some(was) = read(xp, ELEV_TRIM).await else {
            return Row::fail("trim", format!("no reading of {ELEV_TRIM}"));
        };
        let want = if was > 0.8 {
            was - STOCK_TRIM_STEP
        } else {
            was + STOCK_TRIM_STEP
        };
        let _ = xp.set_float(ELEV_TRIM, want).await;
        sleep_s(0.3).await;
        let got = read(xp, ELEV_TRIM).await;
        let _ = xp.set_float(ELEV_TRIM, was).await;
        sleep_s(0.3).await;
        let back = read(xp, ELEV_TRIM).await;
        let (v, mut d) = set_restore_verdict(want, got, was, back, STOCK_TRIM_TOL, false);
        if crate::tools::controls::trim_needs_reset(was) {
            d.push_str("; outside the takeoff band");
        }
        return Row::new(
            "trim",
            (v, d),
            json!({"trim": ELEV_TRIM, "was": was, "want": want, "got": got, "back": back}),
        );
    };
    let Some(was) = read(xp, &t.units).await else {
        return Row::fail("trim", format!("no reading of {}", t.units));
    };
    let (cg, kg) = (read(xp, &t.cg_mac).await, read(xp, TOTAL_WEIGHT_KG).await);
    let figure = match (cg, kg) {
        (Some(c), Some(k)) => crate::takeoff_trim::table_units(t, c, k / 1000.0),
        _ => None,
    };
    // Blipped away from the nearer stop.
    let sense = if was > t.max_units - 0.5 { -1.0 } else { 1.0 };
    let (cmd, _) = crate::takeoff_trim::next_hold(t, sense, 1.0);
    let _ = xp.run_command_for(cmd, TRIM_BLIP_S).await;
    sleep_s(TRIM_BLIP_S + 0.3).await;
    let after = read(xp, &t.units).await;
    let mut now = after;
    let mut rate = after.map_or(0.09, |a| ((a - was).abs() / TRIM_BLIP_S).max(0.01));
    for _ in 0..TRIM_RESTORE_HOLDS {
        let Some(n) = now else { break };
        if (was - n).abs() <= crate::takeoff_trim::TOLERANCE_UNITS {
            break;
        }
        let (cmd, secs) = crate::takeoff_trim::next_hold(t, was - n, rate);
        if xp.run_command_for(cmd, secs).await.is_err() {
            break;
        }
        sleep_s(secs + 0.3).await;
        now = read(xp, &t.units).await;
        if let Some(m) = now {
            if (m - n).abs() > 0.01 {
                rate = (m - n).abs() / secs;
            }
        }
    }
    let (v, mut d) =
        trim_blip_verdict(was, after, now, sense, crate::takeoff_trim::TOLERANCE_UNITS);
    d.push_str(&format!(
        "; table {} units at CG {} %, {} t",
        read_str(figure),
        read_str(cg),
        read_str(kg.map(|k| k / 1000.0))
    ));
    Row::new(
        "trim",
        (v, d),
        json!({"units": t.units, "was": was, "blip": after, "back": now, "table_units": figure,
               "cg_mac": cg, "weight_kg": kg}),
    )
}

/// One window through the panel owner: away and back.
pub async fn probe_window(xp: &dyn XPlane, p: &Panel, d: Dial, label: &str) -> Row {
    let channel = format!("mcp_window_{label}");
    let w = p.window(d);
    let declared = w.is_some();
    let Some(was) = autoflight::read_dial_on(xp, p, d)
        .await
        .ok()
        .filter(|v| v.is_finite())
    else {
        return Row::fail(channel, "no reading of the window");
    };
    let want = window_test_value(d, was);
    let want = w.as_ref().map_or(want, |w| w.snap(want));
    let sent = autoflight::set_dial_on(xp, p, d, want).await;
    sleep_s(WINDOW_SETTLE_S).await;
    let got = autoflight::read_dial_on(xp, p, d)
        .await
        .ok()
        .filter(|v| v.is_finite());
    let restored = autoflight::set_dial_on(xp, p, d, was).await;
    sleep_s(WINDOW_SETTLE_S).await;
    let back = autoflight::read_dial_on(xp, p, d)
        .await
        .ok()
        .filter(|v| v.is_finite());
    let tol = window_tol(w.as_ref(), d, was);
    let wrap = d == Dial::HeadingBug;
    let (mut v, mut detail) = set_restore_verdict(want, got, was, back, tol, wrap);
    // A V/S window blanks with V/S off (the 737's reads 0): no measurement.
    if v == Verdict::Fail && d == Dial::Vs && got == Some(0.0) && want != 0.0 {
        let vs_on = autoflight::read_status_on(xp, p, autoflight::names::VS_STATUS)
            .await
            .ok()
            .map(|s| s >= 0.5);
        if vs_on == Some(false) {
            v = Verdict::Skipped;
            detail = format!("window reads 0 with V/S off (blank): {detail}");
        }
    }
    for (what, r) in [("set", &sent), ("restore", &restored)] {
        if let Err(e) = r {
            detail.push_str(&format!("; {what}: {e:#}"));
        }
    }
    Row::new(
        channel,
        (v, detail),
        json!({"dial": d, "declared": declared, "was": was, "want": want, "got": got,
               "back": back, "tol": tol}),
    )
}

async fn mode_on(xp: &dyn XPlane, p: &Panel, r: &ModeRead) -> Option<bool> {
    let v = match r {
        ModeRead::Stock(stock) => autoflight::read_status_on(xp, p, stock).await.ok(),
        ModeRead::Lamp(name) => xp.get_float(name).await.ok(),
    };
    v.filter(|v| v.is_finite()).map(|v| v >= 0.5)
}

/// A press through the owner, the lamp waited on until it changes from
/// `from` (lamps ramp in), or [`MODE_SETTLE_S`].
async fn press_and_wait(xp: &dyn XPlane, p: &Panel, m: &ModeProbe, from: bool) -> Option<bool> {
    autoflight::press_logged_on(xp, p, m.button, "conform").await;
    let polls = (MODE_SETTLE_S / MODE_POLL_S).round() as usize;
    let mut last = None;
    for _ in 0..polls {
        sleep_s(MODE_POLL_S).await;
        last = mode_on(xp, p, &m.read).await;
        if last.is_some_and(|v| v != from) {
            // Read again after a beat: a lamp that ramps in reads steady.
            sleep_s(MODE_POLL_S).await;
            return mode_on(xp, p, &m.read).await.or(last);
        }
    }
    last
}

/// Every mode's state, in order.
async fn snapshot(xp: &dyn XPlane, p: &Panel, all: &[ModeProbe]) -> Vec<Option<bool>> {
    let mut v = Vec::new();
    for o in all {
        v.push(mode_on(xp, p, &o.read).await);
    }
    v
}

/// One mode button pressed twice through the owner, its lamp read; any
/// other mode the presses moved is named.
pub async fn probe_mode(xp: &dyn XPlane, p: &Panel, m: &ModeProbe, all: &[ModeProbe]) -> Row {
    let channel = format!("mcp_mode_{}", m.label);
    let others_before = snapshot(xp, p, all).await;
    let Some(before) = mode_on(xp, p, &m.read).await else {
        return Row::fail(channel, "no reading of its lamp");
    };
    let first = press_and_wait(xp, p, m, before).await;
    // Pressed again only when the first press moved it: a press that did
    // nothing is not repeated.
    let second = match first {
        Some(f) if f != before => press_and_wait(xp, p, m, f).await,
        _ => first,
    };
    let (v, mut d) = toggle_verdict(Some(before), first, second);
    if first == Some(before) {
        // The Zibo's V/S would not engage parked (2026-10-07): a FAIL here
        // is the ground's as often as the channel's.
        d.push_str("; some modes will not engage on the ground");
    }
    let others_after = snapshot(xp, p, all).await;
    let moved: Vec<String> = all
        .iter()
        .zip(others_before.iter().zip(&others_after))
        .filter(|(o, (b, a))| o.label != m.label && b.is_some() && a.is_some() && b != a)
        .map(|(o, (b, a))| {
            let s = |x: &Option<bool>| if *x == Some(true) { "on" } else { "off" };
            format!("{} {} -> {}", o.label, s(b), s(a))
        })
        .collect();
    if !moved.is_empty() {
        d.push_str(&format!("; also moved: {}", moved.join(", ")));
    }
    Row::new(
        channel,
        (v, d),
        json!({"button": m.button, "before": before, "first": first, "second": second,
               "others_moved": moved}),
    )
}

/// A declared panel switch through the owner (`ensure_switch_on`): flipped
/// and put back, its position read.
pub async fn probe_switch(xp: &dyn XPlane, p: &Panel, s: Switch, label: &str) -> Row {
    let channel = format!("mcp_switch_{label}");
    let Some(sw) = p.switch(s).cloned() else {
        return Row::skipped(channel, "not declared");
    };
    let Some(before) = read(xp, &sw.position).await.map(|v| v >= 0.5) else {
        return Row::fail(channel, format!("no reading of {}", sw.position));
    };
    let _ = autoflight::ensure_switch_on(xp, p, s, !before, "conform").await;
    let first = read(xp, &sw.position).await.map(|v| v >= 0.5);
    let second = if first.is_some_and(|f| f != before) {
        let _ = autoflight::ensure_switch_on(xp, p, s, before, "conform").await;
        read(xp, &sw.position).await.map(|v| v >= 0.5)
    } else {
        first
    };
    Row::new(
        channel,
        toggle_verdict(Some(before), first, second),
        json!({"switch": s, "position": sw.position, "before": before, "first": first,
               "second": second}),
    )
}

/// The park brake position this airframe is read on: the add-on's own where
/// it declares a toggle, else the stock ratio.
pub fn park_brake_name(p: &AircraftProfile) -> String {
    p.park_brake_toggle.as_ref().map_or_else(
        || crate::tools::PARK_BRAKE.to_string(),
        |t| t.position.clone(),
    )
}

/// Run `f` with the plugin's toe brakes held (a heartbeat beside it).
async fn braked<T>(xp: &dyn XPlane, f: impl std::future::Future<Output = T>) -> T {
    let beat = async {
        loop {
            let _ = xp.set_float(TAXI_BRAKE, 1.0).await;
            sleep_s(0.2).await;
        }
    };
    tokio::select! {
        v = f => v,
        _ = beat => unreachable!("the heartbeat never ends"),
    }
}

/// The park brake through its seam (`drive_park_brake`): released with the
/// toe brakes held, read, set again (`set_park_brake_confirmed`), read.
pub async fn probe_park_brake(xp: &dyn XPlane, position: &str, valve_set: Option<f64>) -> Row {
    use crate::tools::controls::{drive_park_brake, set_park_brake_confirmed};
    let Some(was) = read(xp, position).await else {
        return Row::fail("park_brake", format!("no reading of {position}"));
    };
    let (release, released, set) = braked(xp, async {
        let r = drive_park_brake(xp, false, valve_set).await;
        let released = read(xp, position).await;
        let set = set_park_brake_confirmed(xp, valve_set, 3).await;
        (r.map_err(|e| format!("{e:#}")), released, set)
    })
    .await;
    let back = read(xp, position).await;
    Row::new(
        "park_brake",
        park_brake_verdict(was, release.clone(), released, set.held, back),
        json!({"position": position, "was": was, "release": release.ok(), "released": released,
               "held": set.held, "tries": set.tries, "back": back, "err": set.err}),
    )
}

/// The park brake's row: it must read released after the release and set
/// after it is set again. Pure.
pub fn park_brake_verdict(
    was: f64,
    release: Result<bool, String>,
    released: Option<f64>,
    held: bool,
    back: Option<f64>,
) -> (Verdict, String) {
    let set_again = match back {
        None => "no reading after setting it again".to_string(),
        Some(b) if !held || b < 0.5 => format!("NOT set again, reads {}", num(b)),
        Some(b) => format!("set again, {}", num(b)),
    };
    let set_ok = held && back.is_some_and(|b| b >= 0.5);
    if let Err(e) = release {
        return (Verdict::Fail, format!("release failed: {e}; {set_again}"));
    }
    let Some(r) = released else {
        return (
            Verdict::Fail,
            format!("no reading after the release; {set_again}"),
        );
    };
    if r >= 0.5 {
        return (
            Verdict::Fail,
            format!("did not release ({} -> {}); {set_again}", num(was), num(r)),
        );
    }
    if !set_ok {
        return (
            Verdict::Fail,
            format!("released to {}; {set_again}", num(r)),
        );
    }
    (
        Verdict::Pass,
        format!("{} -> {} -> {}", num(was), num(r), read_str(back)),
    )
}

/// The profile row's verdict from the names the sim lacks. Pure.
pub fn profile_verdict(missing: &[String], total: usize) -> (Verdict, String) {
    if missing.is_empty() {
        return (
            Verdict::Pass,
            format!("all {total} names the profile relies on exist"),
        );
    }
    let shown: Vec<&str> = missing.iter().take(6).map(String::as_str).collect();
    let more = if missing.len() > shown.len() {
        format!(" and {} more", missing.len() - shown.len())
    } else {
        String::new()
    };
    (
        Verdict::Fail,
        format!(
            "{} of {total} names missing: {}{more}",
            missing.len(),
            shown.join(", ")
        ),
    )
}

// ---------------------------------------------------------------------------
// The runner.
// ---------------------------------------------------------------------------

/// The profile probe (`aircraft::profile_references` against the sim's
/// indexes, `detect::missing_profile_names`), the first row.
pub async fn profile_row(xp: &crate::xplane::XPlaneClient, p: &AircraftProfile) -> Row {
    let names = crate::aircraft::profile_references(p);
    match crate::detect::missing_profile_names(xp, &names).await {
        Err(e) => Row::fail(
            "profile",
            format!("no reading of the sim's name index: {e:#}"),
        ),
        Ok(missing) => Row::new(
            "profile",
            profile_verdict(&missing, names.len()),
            json!({"names": names.len(), "missing": missing}),
        ),
    }
}

/// One row on the tape: blackbox `conform`.
pub fn record_row(profile: &str, r: &Row) {
    crate::blackbox::record(
        "conform",
        json!({"profile": profile, "channel": r.channel, "verdict": r.verdict,
               "detail": r.detail, "reads": r.reads}),
    );
}

/// Still stopped between probes: `Err` with the reading that says not.
async fn still_parked(xp: &dyn XPlane) -> Result<(), String> {
    match read(xp, GS_MS).await {
        None => Err("no reading of the groundspeed".into()),
        Some(v) => {
            let kts = v.abs() * sim::ground::MS_TO_KTS;
            if sim::ground::stopped_kts(kts) {
                Ok(())
            } else {
                Err(format!("the aircraft moved ({} kt)", num(kts)))
            }
        }
    }
}

/// The yoke route's name for the row.
fn route_name(p: &AircraftProfile) -> String {
    match p.yoke_input {
        None => "stock".to_string(),
        Some(y) => serde_json::to_value(y)
            .ok()
            .and_then(|v| v.as_str().map(str::to_string))
            .unwrap_or_else(|| format!("{y:?}")),
    }
}

/// Run the steps (all but [`Step::Profile`], which needs the client's
/// indexes and is [`profile_row`]'s), each row recorded and handed to
/// `emit` as it completes. A moving aircraft stops the check and has its
/// park brake set.
pub async fn run_steps(
    xp: &dyn XPlane,
    p: &AircraftProfile,
    steps: &[Step],
    emit: &mut dyn FnMut(&Row),
) -> Vec<Row> {
    let panel = Panel::from_profile(p);
    let modes: Vec<ModeProbe> = steps
        .iter()
        .filter_map(|s| match s {
            Step::Mode(m) => Some(m.clone()),
            _ => None,
        })
        .collect();
    let mut yoke_armed = false;
    let mut rows = Vec::new();
    for step in steps {
        if !matches!(step, Step::Skip { .. } | Step::Profile) {
            if let Err(why) = still_parked(xp).await {
                let set = braked(
                    xp,
                    crate::tools::controls::set_park_brake_confirmed(xp, p.park_brake_valve_set, 3),
                )
                .await;
                let row = Row::fail(
                    "parked",
                    format!("check stopped: {why}; park brake set again: {}", set.held),
                );
                record_row(&p.id, &row);
                emit(&row);
                rows.push(row);
                break;
            }
        }
        let row = match step {
            Step::Profile => continue,
            Step::Skip { channel, why } => Row::skipped(channel.clone(), why.clone()),
            Step::Window(d, label) => probe_window(xp, &panel, *d, label).await,
            Step::Mode(m) => probe_mode(xp, &panel, m, &modes).await,
            Step::Switch(s, label) => probe_switch(xp, &panel, *s, label).await,
            Step::Flaps => match p.flaps.as_ref() {
                Some(f) => probe_flaps(xp, f).await,
                None => Row::skipped("flaps", "no flaps declared"),
            },
            Step::Speedbrake => probe_speedbrake(xp, p.speedbrake_lever.as_ref()).await,
            Step::Trim => probe_trim(xp, p.takeoff_trim.as_ref()).await,
            Step::Yoke { pitch } => {
                if !yoke_armed {
                    crate::yoke::arm(xp, "conform").await;
                    yoke_armed = true;
                }
                let mut r = probe_yoke(xp, *pitch).await;
                r.detail = format!("route {}: {}", route_name(p), r.detail);
                r
            }
            Step::Steer => probe_steer(xp).await,
            Step::Tiller => {
                let mut t = crate::tiller::Tiller::for_airframe();
                t.arm(xp).await;
                if t.declared() {
                    let max = p
                        .taxi
                        .as_ref()
                        .and_then(|t| t.tiller.as_ref())
                        .and_then(|t| t.max_deg);
                    probe_tiller(xp, &t, max).await
                } else {
                    Row::fail("tiller", "declared, but the plugin took no tiller method")
                }
            }
            Step::Throttle => {
                let lever = crate::aircraft::throttle_dataref();
                probe_throttle(xp, &lever, p.throttle_hardware_axis.as_deref()).await
            }
            Step::Reverse => {
                let lever = crate::aircraft::throttle_dataref();
                probe_reverse(xp, &lever, p.reverse_hold_command.as_deref(), p.engines).await
            }
            Step::ParkBrake => {
                probe_park_brake(xp, &park_brake_name(p), p.park_brake_valve_set).await
            }
        };
        record_row(&p.id, &row);
        emit(&row);
        rows.push(row);
    }
    rows
}

/// The counts line under the table. Pure.
pub fn summary(rows: &[Row]) -> String {
    let n = |v: Verdict| rows.iter().filter(|r| r.verdict == v).count();
    format!(
        "{} PASS, {} FAIL, {} SKIPPED",
        n(Verdict::Pass),
        n(Verdict::Fail),
        n(Verdict::Skipped)
    )
}

const USAGE: &str = "usage: xvl conform [--only <area>[,<area>...]]\n\
areas: profile, mcp, flaps, speedbrake, trim, yoke, steer, throttle, reverse, brake\n\
Runs only on a parked aircraft: on the ground, stopped, park brake set.";

/// The live profile as the app resolves it: detected from the loaded model
/// when configured `auto`, else the configured id.
async fn resolve_profile(
    xp: &crate::xplane::XPlaneClient,
    configured: &str,
) -> anyhow::Result<AircraftProfile> {
    if !configured.eq_ignore_ascii_case("auto") {
        return crate::aircraft::load(configured)
            .ok_or_else(|| anyhow::anyhow!("unknown aircraft profile: {configured}"));
    }
    let who = crate::detect::read_identity(xp).await?;
    let picked = crate::detect::pick_profile(&who);
    if let Some(line) = &picked.line {
        println!("{line}\n");
    }
    let id = picked.id.clone();
    picked
        .profile
        .or_else(|| crate::aircraft::load(&id))
        .ok_or_else(|| anyhow::anyhow!("no profile for {id}"))
}

/// `xvl conform [--only <area>]`: `Ok(true)` when nothing failed.
pub async fn main_cmd(args: &[String], base_url: &str, configured: &str) -> anyhow::Result<bool> {
    let mut only = Vec::new();
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--only" => {
                let list = it
                    .next()
                    .ok_or_else(|| anyhow::anyhow!("--only needs an area\n{USAGE}"))?;
                only.extend(Area::parse_list(list).map_err(|e| anyhow::anyhow!("{e}\n{USAGE}"))?);
            }
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(true);
            }
            other => anyhow::bail!("unknown argument '{other}'\n{USAGE}"),
        }
    }
    let xp = crate::xplane::XPlaneClient::new(base_url.to_string());
    let p = resolve_profile(&xp, configured).await?;
    // The seams read the live airframe's facts (the throttle, the levers,
    // the panel, the yoke route, the tiller): this process installs them.
    crate::flight_state::install_for_check(&p);
    println!("# Conformance: {} ({})\n", p.display_name, p.id);
    let park_name = park_brake_name(&p);
    let (g, gs, park) = tokio::join!(
        read(&xp, ON_GROUND),
        read(&xp, GS_MS),
        read(&xp, &park_name),
    );
    match parked(g, gs, park) {
        Err(why) => {
            println!(
                "Refused: {why}. The check runs only on a parked aircraft: on the ground, stopped, park brake set."
            );
            record_row(&p.id, &Row::fail("parked", format!("refused: {why}")));
            return Ok(false);
        }
        Ok(line) => println!("Parked: {line}.\n"),
    }
    for l in TABLE_HEAD {
        println!("{l}");
    }
    let steps = plan(&p, &only);
    let mut emit = |r: &Row| println!("{}", r.table_line());
    let mut rows = Vec::new();
    if steps.contains(&Step::Profile) {
        let r = profile_row(&xp, &p).await;
        record_row(&p.id, &r);
        emit(&r);
        rows.push(r);
    }
    rows.extend(run_steps(&xp, &p, &steps, &mut emit).await);
    println!("\n{}", summary(&rows));
    Ok(rows.iter().all(|r| r.verdict != Verdict::Fail))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft::{McpWindow, ModeLamps};
    use crate::xplane::FakeXPlane;

    fn zibo() -> AircraftProfile {
        let yaml = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/profiles/zibo_738.yaml"
        ))
        .unwrap();
        serde_yaml::from_str(&yaml).unwrap()
    }

    // ---- the refusal -------------------------------------------------------

    #[test]
    fn it_runs_only_on_a_parked_aircraft() {
        assert!(parked(Some(1.0), Some(0.0), Some(1.0)).is_ok());
        assert_eq!(
            parked(None, Some(0.0), Some(1.0)).unwrap_err(),
            "no reading of on-ground"
        );
        assert_eq!(
            parked(Some(0.0), Some(0.0), Some(1.0)).unwrap_err(),
            "not on the ground"
        );
        assert!(parked(Some(1.0), Some(3.0), Some(1.0))
            .unwrap_err()
            .starts_with("moving at 5.83"));
        assert_eq!(
            parked(Some(1.0), None, Some(1.0)).unwrap_err(),
            "no reading of the groundspeed"
        );
        assert_eq!(
            parked(Some(1.0), Some(0.0), Some(0.0)).unwrap_err(),
            "park brake not set (reads 0.00)"
        );
        assert_eq!(
            parked(Some(1.0), Some(0.0), None).unwrap_err(),
            "no reading of the park brake"
        );
    }

    // ---- the verdicts ------------------------------------------------------

    #[test]
    fn a_setting_must_take_and_be_put_back() {
        let (v, d) = set_restore_verdict(5000.0, Some(5000.0), 4000.0, Some(4000.0), 1.0, false);
        assert_eq!((v, d.as_str()), (Verdict::Pass, "4000 -> 5000 -> 4000"));
        let (v, d) = set_restore_verdict(150.0, Some(100.0), 100.0, Some(100.0), 1.0, false);
        assert_eq!(v, Verdict::Fail);
        assert_eq!(d, "did not take: sent 150, read 100 (was 100; restored)");
        let (v, d) = set_restore_verdict(5000.0, Some(5000.0), 4000.0, Some(5000.0), 1.0, false);
        assert_eq!(v, Verdict::Fail);
        assert!(d.contains("NOT restored"), "{d}");
        let (v, d) = set_restore_verdict(5000.0, None, 4000.0, Some(4000.0), 1.0, false);
        assert_eq!(v, Verdict::Fail);
        assert!(d.starts_with("no reading"), "{d}");
        // A heading the short way round.
        let (v, _) = set_restore_verdict(15.0, Some(14.6), 355.0, Some(355.2), 1.0, true);
        assert_eq!(v, Verdict::Pass);
        assert!((gap(359.5, 0.5, true) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn a_surface_moves_both_ways_in_its_sense_and_comes_back() {
        assert_eq!(largest_delta(&[0.0, 1.0], &[-8.4, 1.5]), Some((0, -8.4)));
        assert_eq!(delta_at(&[0.0, 1.0], &[0.0, 3.0], 1), Some(2.0));
        assert_eq!(delta_at(&[0.0], &[0.0, 3.0], 1), None);
        // The Zibo's measured elevator: aft (positive) is trailing edge up.
        let (v, _) = surface_verdict(Some((0, -8.4)), Some(7.9), Some(0.2), 2.0, Some(-1.0));
        assert_eq!(v, Verdict::Pass);
        let (v, d) = surface_verdict(Some((0, 8.4)), Some(-7.9), Some(0.2), 2.0, Some(-1.0));
        assert_eq!(v, Verdict::Fail);
        assert!(d.starts_with("REVERSED"), "{d}");
        // The stock ratios rewritten by the airframe: nothing moves.
        let (v, d) = surface_verdict(Some((0, 0.0)), Some(0.0), Some(0.0), 2.0, Some(-1.0));
        assert_eq!(v, Verdict::Fail);
        assert!(d.starts_with("did not move"), "{d}");
        let (v, d) = surface_verdict(Some((1, 5.0)), Some(4.0), Some(0.0), 2.0, None);
        assert_eq!(v, Verdict::Fail);
        assert!(d.starts_with("moved the same way"), "{d}");
        let (v, d) = surface_verdict(Some((1, 5.0)), Some(-4.0), Some(3.0), 2.0, None);
        assert_eq!(v, Verdict::Fail);
        assert!(d.contains("did not come back"), "{d}");
        let (v, d) = surface_verdict(None, Some(-4.0), Some(0.0), 2.0, None);
        assert_eq!(v, Verdict::Fail);
        assert!(d.starts_with("no reading"), "{d}");
    }

    #[test]
    fn a_button_is_a_toggle_left_where_it_started() {
        assert_eq!(
            toggle_verdict(Some(false), Some(true), Some(false)).0,
            Verdict::Pass
        );
        let (v, d) = toggle_verdict(Some(false), Some(false), Some(false));
        assert_eq!(v, Verdict::Fail);
        assert_eq!(d, "stayed off after a press (did not take)");
        let (v, d) = toggle_verdict(Some(false), Some(true), Some(true));
        assert_eq!(v, Verdict::Fail);
        assert!(d.contains("not a toggle, left on"), "{d}");
        assert_eq!(
            toggle_verdict(None, Some(true), Some(false)).0,
            Verdict::Fail
        );
    }

    #[test]
    fn a_lever_that_snaps_back_to_the_hardware_fails() {
        let (v, _) = lever_held_verdict(0.08, Some(0.08), Some(0.08), 0.0, Some(0.0));
        assert_eq!(v, Verdict::Pass);
        // The Zibo's levers at LEMG: the write read, then the hardware's.
        let (v, d) = lever_held_verdict(0.08, Some(0.08), Some(0.28), 0.0, Some(0.0));
        assert_eq!(v, Verdict::Fail);
        assert!(d.starts_with("took 0.08 then went to 0.28"), "{d}");
        let (v, d) = lever_held_verdict(0.08, None, Some(0.08), 0.0, Some(0.0));
        assert_eq!(v, Verdict::Fail);
        assert!(d.starts_with("no reading"), "{d}");
    }

    #[test]
    fn a_reverser_deploys_on_the_hold_and_stows_after() {
        let (v, _) = reverse_verdict(
            Some(0.69),
            Some(-1.4),
            Some(0.0),
            Some(0.0),
            0.0,
            Some(true),
        );
        assert_eq!(v, Verdict::Pass);
        // The Zibo's stock beta write: read straight back, undone.
        let (v, d) = reverse_verdict(Some(0.0), Some(0.0), Some(0.0), Some(0.0), 0.0, Some(true));
        assert_eq!(v, Verdict::Fail);
        assert!(d.starts_with("did not deploy"), "{d}");
        let (v, _) = reverse_verdict(Some(0.0), Some(0.0), Some(0.0), Some(0.0), 0.0, Some(false));
        assert_eq!(v, Verdict::Skipped);
        let (v, d) = reverse_verdict(
            Some(1.0),
            Some(-2.0),
            Some(1.0),
            Some(-2.0),
            0.0,
            Some(true),
        );
        assert_eq!(v, Verdict::Fail);
        assert!(d.contains("did NOT stow"), "{d}");
        assert_eq!(
            reverse_verdict(None, None, None, None, 0.0, None).0,
            Verdict::Fail
        );
    }

    #[test]
    fn a_trim_blip_moves_its_way_and_is_wound_back() {
        let (v, _) = trim_blip_verdict(4.4, Some(4.49), Some(4.42), 1.0, 0.1);
        assert_eq!(v, Verdict::Pass);
        let (v, d) = trim_blip_verdict(4.4, Some(4.4), Some(4.4), 1.0, 0.1);
        assert_eq!(v, Verdict::Fail);
        assert!(d.contains("did not take"), "{d}");
        // Moved, but the wrong way for the switch held.
        assert_eq!(
            trim_blip_verdict(4.4, Some(4.3), Some(4.4), 1.0, 0.1).0,
            Verdict::Fail
        );
        let (v, d) = trim_blip_verdict(4.4, Some(4.5), Some(4.8), 1.0, 0.1);
        assert_eq!(v, Verdict::Fail);
        assert!(d.contains("NOT restored"), "{d}");
    }

    #[test]
    fn the_park_brake_must_release_and_set_again() {
        assert_eq!(
            park_brake_verdict(1.0, Ok(true), Some(0.0), true, Some(1.0)).0,
            Verdict::Pass
        );
        let (v, d) = park_brake_verdict(1.0, Ok(false), Some(1.0), true, Some(1.0));
        assert_eq!(v, Verdict::Fail);
        assert!(d.starts_with("did not release"), "{d}");
        let (v, d) = park_brake_verdict(1.0, Ok(true), Some(0.0), false, Some(0.0));
        assert_eq!(v, Verdict::Fail);
        assert!(d.contains("NOT set again"), "{d}");
        assert_eq!(
            park_brake_verdict(1.0, Err("x".into()), None, true, Some(1.0)).0,
            Verdict::Fail
        );
    }

    #[test]
    fn the_profile_row_names_the_missing() {
        assert_eq!(profile_verdict(&[], 120).0, Verdict::Pass);
        let missing: Vec<String> = (0..8).map(|i| format!("a/b{i}")).collect();
        let (v, d) = profile_verdict(&missing, 120);
        assert_eq!(v, Verdict::Fail);
        assert!(d.starts_with("8 of 120 names missing: a/b0"), "{d}");
        assert!(d.ends_with("and 2 more"), "{d}");
    }

    #[test]
    fn the_table_line_keeps_its_columns() {
        let r = Row::fail("x", "a | b");
        assert_eq!(r.table_line(), "| x | FAIL | a / b |");
        assert_eq!(
            summary(&[r, Row::skipped("y", "z")]),
            "0 PASS, 1 FAIL, 1 SKIPPED"
        );
    }

    // ---- the plan ----------------------------------------------------------

    #[test]
    fn the_zibo_plan_covers_every_channel_it_declares() {
        let steps = plan(&zibo(), &[]);
        assert_eq!(steps.first(), Some(&Step::Profile));
        assert_eq!(steps.last(), Some(&Step::ParkBrake));
        assert!(
            !steps.iter().any(|s| matches!(s, Step::Skip { .. })),
            "{steps:?}"
        );
        for (d, _) in WINDOWS {
            assert!(steps
                .iter()
                .any(|s| matches!(s, Step::Window(x, _) if *x == d)));
        }
        let modes: Vec<&str> = steps
            .iter()
            .filter_map(|s| match s {
                Step::Mode(m) => Some(m.label),
                _ => None,
            })
            .collect();
        assert_eq!(
            modes,
            [
                "heading",
                "vs",
                "altitude_hold",
                "level_change",
                "approach",
                "lnav"
            ]
        );
        let switches = steps
            .iter()
            .filter(|s| matches!(s, Step::Switch(..)))
            .count();
        assert_eq!(switches, 3);
        for want in [
            Step::Flaps,
            Step::Speedbrake,
            Step::Trim,
            Step::Yoke { pitch: true },
            Step::Yoke { pitch: false },
            Step::Steer,
            Step::Tiller,
            Step::Throttle,
            Step::Reverse,
        ] {
            assert!(steps.contains(&want), "{want:?}");
        }
        // Thrust last but the brake: nothing moves a lever before the reads.
        let at = |w: &Step| steps.iter().position(|s| s == w).unwrap();
        assert!(at(&Step::Throttle) > at(&Step::Steer));
        assert!(at(&Step::Reverse) > at(&Step::Throttle));
    }

    #[test]
    fn an_undeclared_channel_is_a_skipped_row() {
        let c172 = crate::aircraft::load("cessna_172sp").unwrap();
        let steps = plan(&c172, &[]);
        for ch in ["reverse", "tiller"] {
            assert!(
                steps
                    .iter()
                    .any(|s| matches!(s, Step::Skip { channel, .. } if channel == ch)),
                "{ch}: {steps:?}"
            );
        }
    }

    #[test]
    fn only_runs_the_areas_named() {
        let steps = plan(&zibo(), &Area::parse_list("mcp").unwrap());
        assert!(!steps.is_empty());
        assert!(steps
            .iter()
            .all(|s| matches!(s, Step::Window(..) | Step::Mode(_) | Step::Switch(..))));
        let steps = plan(&zibo(), &Area::parse_list("yoke,brake").unwrap());
        assert_eq!(
            steps,
            [
                Step::Yoke { pitch: true },
                Step::Yoke { pitch: false },
                Step::ParkBrake
            ]
        );
        assert!(Area::parse_list("flapz")
            .unwrap_err()
            .contains("unknown area 'flapz'"));
        assert_eq!(Area::parse("Tiller"), Some(Area::Steer));
    }

    // ---- the probes, against the fake -------------------------------------

    #[tokio::test(start_paused = true)]
    async fn a_lever_off_idle_is_never_moved() {
        let xp = FakeXPlane::with_reads(&[("lever", 0.5)]);
        let r = probe_throttle(&xp, "lever", None).await;
        assert_eq!(r.verdict, Verdict::Skipped);
        assert!(xp.writes_to("lever").is_empty());
    }

    #[tokio::test(start_paused = true)]
    async fn a_thrust_write_that_does_not_land_fails_and_is_put_back() {
        let xp = FakeXPlane::with_reads(&[("lever", 0.0), ("hw", 0.28)]);
        let r = probe_throttle(&xp, "lever", Some("hw")).await;
        assert_eq!(r.verdict, Verdict::Fail);
        assert!(
            r.detail.starts_with("did not take: sent 0.08, read 0.00"),
            "{}",
            r.detail
        );
        assert!(r.detail.ends_with("hardware axis 0.28"), "{}", r.detail);
        assert_eq!(xp.writes_to("lever"), [PROBE_THRUST, 0.0]);
    }

    #[tokio::test(start_paused = true)]
    async fn a_knob_window_is_stepped_there_and_back() {
        let mut p = Panel::default();
        p.windows.airspeed = Some(McpWindow {
            up: Some("spd_up".into()),
            down: Some("spd_dn".into()),
            step: Some(1.0),
            readback: Some("spd".into()),
            ..Default::default()
        });
        let xp = FakeXPlane::with_reads(&[("spd", 150.0)]);
        xp.step_on_command("spd_up", "spd", 1.0);
        xp.step_on_command("spd_dn", "spd", -1.0);
        let r = probe_window(&xp, &p, Dial::Airspeed, "airspeed").await;
        assert_eq!(r.verdict, Verdict::Pass, "{}", r.detail);
        assert_eq!(r.detail, "150 -> 160 -> 150");
        assert_eq!(xp.ran_count("spd_up"), 10);
        assert_eq!(xp.ran_count("spd_dn"), 10);
    }

    #[tokio::test(start_paused = true)]
    async fn a_window_write_that_does_not_stand_fails_and_is_put_back() {
        let mut p = Panel::default();
        p.windows.heading = Some(McpWindow {
            write: Some("hdg".into()),
            ..Default::default()
        });
        let xp = FakeXPlane::with_reads(&[("hdg", 355.0)]);
        let r = probe_window(&xp, &p, Dial::HeadingBug, "heading").await;
        assert_eq!(r.verdict, Verdict::Fail);
        assert!(
            r.detail.starts_with("did not take: sent 15.0, read 355"),
            "{}",
            r.detail
        );
        assert_eq!(xp.writes_to("hdg"), [15.0, 355.0]);
    }

    #[tokio::test(start_paused = true)]
    async fn a_vs_window_blank_with_vs_off_is_skipped() {
        let mut p = Panel::default();
        p.windows.vs = Some(McpWindow {
            write: Some("vvi".into()),
            step: Some(50.0),
            ..Default::default()
        });
        p.lamps = Some(ModeLamps {
            vs: Some("vs_lamp".into()),
            ..Default::default()
        });
        let xp = FakeXPlane::with_reads(&[("vvi", 0.0), ("vs_lamp", 0.0)]);
        let r = probe_window(&xp, &p, Dial::Vs, "vs").await;
        assert_eq!(r.verdict, Verdict::Skipped, "{}", r.detail);
        assert!(
            r.detail.starts_with("window reads 0 with V/S off"),
            "{}",
            r.detail
        );
    }

    fn heading_panel() -> (Panel, ModeProbe) {
        let mut p = Panel::default();
        p.heading = "hdg_press".into();
        p.lamps = Some(ModeLamps {
            heading: Some("hdg_lamp".into()),
            ..Default::default()
        });
        let m = mode_probes(&p).remove(0);
        assert_eq!(m.read, ModeRead::Lamp("hdg_lamp".into()));
        (p, m)
    }

    #[tokio::test(start_paused = true)]
    async fn a_press_that_lights_nothing_fails_and_is_not_repeated() {
        let (p, m) = heading_panel();
        let xp = FakeXPlane::with_reads(&[("hdg_lamp", 0.0)]);
        let r = probe_mode(&xp, &p, &m, std::slice::from_ref(&m)).await;
        assert_eq!(r.verdict, Verdict::Fail);
        assert!(
            r.detail
                .starts_with("stayed off after a press (did not take)"),
            "{}",
            r.detail
        );
        assert_eq!(xp.ran_count("hdg_press"), 1);
    }

    #[tokio::test(start_paused = true)]
    async fn a_button_that_only_sets_is_not_a_toggle() {
        let (p, m) = heading_panel();
        let xp = FakeXPlane::with_reads(&[("hdg_lamp", 0.0)]);
        xp.set_on_command("hdg_press", "hdg_lamp", 1.0);
        let r = probe_mode(&xp, &p, &m, std::slice::from_ref(&m)).await;
        assert_eq!(r.verdict, Verdict::Fail);
        assert!(r.detail.contains("not a toggle, left on"), "{}", r.detail);
        assert_eq!(xp.ran_count("hdg_press"), 2);
    }

    #[tokio::test(start_paused = true)]
    async fn a_yoke_that_reads_nothing_is_no_measurement() {
        let xp = FakeXPlane::new();
        let r = probe_yoke(&xp, true).await;
        assert_eq!(r.verdict, Verdict::Fail);
        assert!(r.detail.starts_with("no reading"), "{}", r.detail);
        assert!(xp.writes_to(PITCH).is_empty());
    }

    #[tokio::test(start_paused = true)]
    async fn a_yoke_whose_surface_never_moves_fails_and_lets_go() {
        let xp = FakeXPlane::new();
        xp.set_read_array(ELEVATOR_DEG, &[0.0, 0.0]);
        let r = probe_yoke(&xp, true).await;
        assert_eq!(r.verdict, Verdict::Fail);
        assert!(r.detail.starts_with("did not move"), "{}", r.detail);
        let w = xp.writes_to(PITCH);
        assert!(w.contains(&CHANNEL_CMD) && w.contains(&-CHANNEL_CMD));
        assert_eq!(w.last(), Some(&0.0));
    }

    #[tokio::test(start_paused = true)]
    async fn a_stepped_flap_lever_goes_a_detent_and_back() {
        let flaps: FlapProfile = serde_yaml::from_str(
            "by_command: {more: fd, less: fu, readback: lever}\n\
             detents:\n\
             - {label: up, spoken: up, ratio: 0.0}\n\
             - {label: '1', spoken: one, ratio: 0.125}\n\
             - {label: '2', spoken: two, ratio: 0.25}\n",
        )
        .unwrap();
        let xp = FakeXPlane::with_reads(&[("lever", 0.0)]);
        xp.step_on_command("fd", "lever", 0.125);
        xp.step_on_command("fu", "lever", -0.125);
        let r = probe_flaps(&xp, &flaps).await;
        assert_eq!(r.verdict, Verdict::Pass, "{}", r.detail);
        assert_eq!((xp.ran_count("fd"), xp.ran_count("fu")), (1, 1));
    }

    #[tokio::test(start_paused = true)]
    async fn the_park_brake_is_released_under_the_toe_brakes_and_set_again() {
        let xp = FakeXPlane::with_reads(&[(crate::tools::PARK_BRAKE, 1.0)]);
        xp.set_on_command(
            "sim/flight_controls/park_brake_release",
            crate::tools::PARK_BRAKE,
            0.0,
        );
        xp.set_on_command(
            "sim/flight_controls/park_brake_set",
            crate::tools::PARK_BRAKE,
            1.0,
        );
        let r = probe_park_brake(&xp, crate::tools::PARK_BRAKE, None).await;
        assert_eq!(r.verdict, Verdict::Pass, "{}", r.detail);
        assert!(xp.writes_to(TAXI_BRAKE).iter().all(|v| *v == 1.0));
        assert!(!xp.writes_to(TAXI_BRAKE).is_empty());
        assert_eq!(xp.read_value(crate::tools::PARK_BRAKE), Some(1.0));
    }

    #[tokio::test(start_paused = true)]
    async fn a_moving_aircraft_stops_the_check_and_is_braked() {
        let xp = FakeXPlane::with_reads(&[(GS_MS, 2.0), (crate::tools::PARK_BRAKE, 0.0)]);
        xp.set_on_command(
            "sim/flight_controls/park_brake_set",
            crate::tools::PARK_BRAKE,
            1.0,
        );
        let p = crate::aircraft::load("cessna_172sp").unwrap();
        let steps = plan(&p, &[Area::Yoke]);
        let mut seen = Vec::new();
        let rows = run_steps(&xp, &p, &steps, &mut |r: &Row| seen.push(r.channel.clone())).await;
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].channel, "parked");
        assert_eq!(rows[0].verdict, Verdict::Fail);
        assert!(
            rows[0].detail.contains("the aircraft moved"),
            "{}",
            rows[0].detail
        );
        assert!(xp.writes_to(PITCH).is_empty());
        assert_eq!(seen, ["parked"]);
        assert_eq!(xp.read_value(crate::tools::PARK_BRAKE), Some(1.0));
    }
}
