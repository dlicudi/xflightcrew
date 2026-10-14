//! The autopilot LANDS the airframe, the crew monitors: an ILS landing flown
//! the way a 737 crew flies it, on an airframe that declares it
//! (`autopilot_lands:` in its profile, [`AutopilotLands`]).
//!
//! Until 2026-10-07 every landing on the Zibo 737 was autoland's own law:
//! our heading bug and V/S wheel down the final and our flare on the yoke.
//! The Zibo's heading window takes whole degrees only, so its lateral weave
//! sent four LEMG approaches around 28 to 34 m off the centreline, while the
//! airframe has its own APP, dual-channel autoland, flare, retard and
//! rollout. The pilot's direction: use the aircraft's own systems.
//!
//! The flow (autoland's tick drives it; every decision here is pure):
//!
//! - **Set up**: NAV1 tuned and the course set on the MCP's own course
//!   window (`ils::tune`, `windows.course`); with `nav2`, NAV2 and the first
//!   officer's course too ([`prepare`]).
//! - **Arm and capture**: APP is armed by `ils::Coupling` from below the
//!   beam (only from off, a toggle); its captures are the FMA's
//!   ([`modes_with_fma`]: VOR/LOC or ROLLOUT on the roll column is the
//!   localiser, G/S or FLARE on the pitch column the glideslope), since the
//!   Zibo's APP lamp is lit armed or captured alike.
//! - **Dual** ([`Landing`]): after the glideslope captures, the second
//!   channel (CMD B) is pressed, read on its lamp, at most twice, each press
//!   left [`crate::autoflight::PRESS_SETTLE_S`] to answer; both lamps lit
//!   by `dual_by_agl_ft`, the landing armed (FLARE armed or LAND 2/3) by
//!   `land_by_agl_ft`.
//! - **The airframe lands** ([`Landing::ap_flies`]): our arrest, flare
//!   column and roll channel stand aside (the plugin releases our yoke
//!   channels while CMD is lit anyway); with `autothrottle` the A/T holds
//!   our approach target on the MCP window in SPEED and retards itself
//!   ([`AtSpeed`]), our lever standing aside and the thrust watch quiet;
//!   else our thrust law and its retard fly the speed as before.
//! - **Monitoring stays ours**: the stabilised-approach rule, the landing
//!   call, the tower's and the pilot's go-around, the callouts. A go-around
//!   while the autopilot flies presses TO/GA, then the missed approach is
//!   flown as any other.
//! - **Rollout** ([`rollout_hand_back`]): reverse and brakes are ours
//!   (`tools::reverse`, the brake law); the steering is the autopilot's
//!   ROLLOUT until taxi speed (`disconnect_kts`), a cross-track past
//!   `rollout_xtk_limit_m`, no ROLLOUT annunciated, or the autopilot out:
//!   then it is disconnected and our rollout law steers.
//! - **Fallbacks are loud** ([`Fallback`]): not captured or not dual by the
//!   dual gate, our own law flies the approach (said); not armed to land by
//!   the land gate, or the autopilot lost above `own_flare_below_agl_ft`, a
//!   go-around with height in hand (said); lost below it, our flare takes
//!   the landing (said). The A/T not taking SPEED, not holding it, or a
//!   speed window that will not set: our thrust law (said).
//!
//! Blackbox `autoland` `coupled_tick` every tick on this path carries every
//! FMA column raw beside the decisions, so a probe flight decodes the codes.

use crate::aircraft::{AutopilotLands, FmaCodes};
use crate::autoflight::{Dial, Modes};
use crate::ils::Coupling;
use serde_json::json;

/// The landing path the live airframe declares, given an ILS to fly.
pub fn declared() -> Option<AutopilotLands> {
    crate::aircraft::live_profile().and_then(|p| p.autopilot_lands.clone())
}

// ---------------------------------------------------------------------------
// The reads.
// ---------------------------------------------------------------------------

/// One tick's reads on this path, each `None` unread.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Reads {
    pub roll: Option<f64>,
    pub roll_armed: Option<f64>,
    pub pitch: Option<f64>,
    pub pitch_armed: Option<f64>,
    pub thrust: Option<f64>,
    pub land: Option<f64>,
    /// The second channel's lamp.
    pub cmd_b: Option<f64>,
    /// The A/T SPEED lamp (`lamps.speed`).
    pub speed_lamp: Option<f64>,
    /// The MCP speed window as it reads back.
    pub speed_window: Option<f64>,
    /// `fma.log`, in order.
    pub log: Vec<Option<f64>>,
}

/// One name read: an element as `name[i]` (the Zibo's FMA columns are
/// captain/first officer arrays), else the scalar. `None` unread.
async fn read_one(xp: &dyn crate::xplane::XPlane, name: &str) -> Option<f64> {
    let v = match crate::tools::lights::split_index(name) {
        (base, Some(i)) => xp.get_float_array(base).await.ok()?.get(i).copied()?,
        (n, None) => xp.get_float(n).await.ok()?,
    };
    v.is_finite().then_some(v)
}

async fn read_opt(xp: &dyn crate::xplane::XPlane, name: Option<&str>) -> Option<f64> {
    match name {
        Some(n) => read_one(xp, n).await,
        None => None,
    }
}

/// This path's reads, in one round beside the tick's frame.
pub async fn read(
    xp: &dyn crate::xplane::XPlane,
    cfg: &AutopilotLands,
    panel: &crate::autoflight::Panel,
) -> Reads {
    let f = &cfg.fma;
    let speed_lamp = panel.lamps.as_ref().and_then(|l| l.speed.clone());
    let window = panel
        .window(Dial::Airspeed)
        .map(|w| w.readback().to_string());
    let (roll, roll_armed, pitch, pitch_armed, thrust, land, cmd_b, speed_lamp, speed_window, log) = tokio::join!(
        read_one(xp, &f.roll),
        read_opt(xp, f.roll_armed.as_deref()),
        read_one(xp, &f.pitch),
        read_opt(xp, f.pitch_armed.as_deref()),
        read_opt(xp, f.thrust.as_deref()),
        read_opt(xp, f.land.as_deref()),
        read_one(xp, &cfg.second_cmd.position),
        read_opt(xp, speed_lamp.as_deref()),
        read_opt(xp, window.as_deref()),
        futures::future::join_all(f.log.iter().map(|n| read_one(xp, n))),
    );
    Reads {
        roll,
        roll_armed,
        pitch,
        pitch_armed,
        thrust,
        land,
        cmd_b,
        speed_lamp,
        speed_window,
        log,
    }
}

/// NAV2 tuned and the first officer's course set to the ILS, read back
/// (`nav2`): a dual-channel approach reads both receivers. Blackboxed `ils`
/// `nav2_tuned`. `true` when both read right.
pub async fn prepare(
    xp: &dyn crate::xplane::XPlane,
    cfg: &AutopilotLands,
    ils: &crate::nav::Ils,
) -> bool {
    if !cfg.nav2 {
        return true;
    }
    const NAV2_FREQ: &str = "sim/cockpit2/radios/actuators/nav2_frequency_hz";
    let freq = xp.get_float(NAV2_FREQ).await;
    if !freq.is_ok_and(|f| f.round() as i64 == ils.freq_10khz) {
        let _ = xp.set_int(NAV2_FREQ, ils.freq_10khz).await;
    }
    let course = ils.course_mag.round().rem_euclid(360.0);
    let on = |c: anyhow::Result<f64>| {
        c.is_ok_and(|c| crate::angles::abs_diff_deg(c, ils.course_mag) < 1.0)
    };
    if !on(crate::autoflight::read_dial(xp, Dial::CourseFo).await) {
        let _ = crate::autoflight::set_dial(xp, Dial::CourseFo, course).await;
    }
    let (freq, crs) = tokio::join!(
        xp.get_float(NAV2_FREQ),
        crate::autoflight::read_dial(xp, Dial::CourseFo)
    );
    let freq = freq.ok();
    let crs = crs.ok();
    let ok = freq.is_some_and(|f| f.round() as i64 == ils.freq_10khz)
        && on(crs.ok_or_else(|| anyhow::anyhow!("unread")));
    crate::blackbox::record(
        "ils",
        json!({"event": "nav2_tuned", "freq_mhz": ils.freq_mhz(), "course": ils.course_mag,
               "freq_read": freq, "course_read": crs, "ok": ok}),
    );
    ok
}

// ---------------------------------------------------------------------------
// The FMA, decoded.
// ---------------------------------------------------------------------------

/// The FMA as modes: each `None` when its column did not read (or declares
/// no code for it), so nothing is decided on it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Fma {
    pub loc: Option<bool>,
    pub rollout: Option<bool>,
    pub gs: Option<bool>,
    pub flare: Option<bool>,
    pub toga: Option<bool>,
    pub flare_armed: Option<bool>,
    pub speed: Option<bool>,
    pub retard: Option<bool>,
    pub land: Option<bool>,
}

fn shows(v: Option<f64>, codes: &[f64]) -> Option<bool> {
    if codes.is_empty() {
        return None;
    }
    let v = v.filter(|v| v.is_finite())?;
    Some(codes.iter().any(|c| (c - v).abs() < 0.5))
}

impl Fma {
    /// Pure.
    pub fn decode(r: &Reads, c: &FmaCodes) -> Self {
        Self {
            loc: shows(r.roll, &c.loc),
            rollout: shows(r.roll, &c.rollout),
            gs: shows(r.pitch, &c.gs),
            flare: shows(r.pitch, &c.flare),
            toga: shows(r.pitch, &c.toga),
            flare_armed: shows(r.pitch_armed, &c.flare_armed),
            speed: shows(r.thrust, &c.speed),
            retard: shows(r.thrust, &c.retard),
            land: shows(r.land, &c.land),
        }
    }

    /// The localiser is the autopilot's: VOR/LOC, or ROLLOUT after it.
    /// `None` when the roll column did not read.
    pub fn lateral(&self) -> Option<bool> {
        either(self.loc, self.rollout)
    }

    /// The glideslope is the autopilot's: G/S, or FLARE after it.
    pub fn vertical(&self) -> Option<bool> {
        either(self.gs, self.flare)
    }

    /// The landing is armed or flying: FLARE armed or engaged, LAND 2/3.
    pub fn landing(&self) -> Option<bool> {
        either(either(self.flare_armed, self.flare), self.land)
    }
}

fn either(a: Option<bool>, b: Option<bool>) -> Option<bool> {
    match (a, b) {
        (Some(true), _) | (_, Some(true)) => Some(true),
        (None, None) => None,
        _ => Some(false),
    }
}

/// The panel's modes with the FMA's captures in them: a lamp panel's APP
/// lamp is lit armed or captured alike (`autoflight::modes_from_lamps`
/// reads it armed), so the captures are the FMA's. Unread lamps are no
/// measurement, and stay none. Pure.
pub fn modes_with_fma(lamps: Option<Modes>, fma: &Fma) -> Option<Modes> {
    let mut m = lamps?;
    if fma.lateral() == Some(true) {
        m.0 |= Modes::LOC;
    }
    if fma.vertical() == Some(true) {
        m.0 |= Modes::GS;
    }
    Some(m)
}

// ---------------------------------------------------------------------------
// The landing: who flies it.
// ---------------------------------------------------------------------------

/// A landing the autopilot was to fly, taken back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fallback {
    /// Our own law flies it from here: the approach (decoupled, said), or
    /// low, our flare.
    OwnLaw(&'static str),
    /// A go-around, with height in hand.
    GoAround(&'static str),
}

impl Fallback {
    pub fn why(self) -> &'static str {
        match self {
            Fallback::OwnLaw(w) | Fallback::GoAround(w) => w,
        }
    }
}

/// Where the landing stands.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Landing {
    /// APP armed or arming: waiting for both captures (`Coupling::Coupled`).
    #[default]
    Approach,
    /// Both captured: the second channel pressed and read.
    SecondCmd {
        pressed_s: Option<f64>,
        presses: u32,
    },
    /// Both channels in: waiting for the landing to arm.
    Dual,
    /// FLARE armed (or LAND 2/3): the airframe lands.
    Land,
    /// Taken back for this approach.
    Fallen(Fallback),
}

/// The second channel is pressed at most this many times.
pub const SECOND_CMD_PRESSES: u32 = 2;

/// One tick's facts for [`Landing::step`].
#[derive(Debug, Clone, Copy)]
pub struct LandIn {
    pub coupling: Coupling,
    /// Radio height, feet (0 unread).
    pub radio_ft: f64,
    pub airborne: bool,
    /// A go-around is being flown (anyone's: the tower's, the pilot's, the
    /// stabilised-approach rule, the landing call).
    pub going_around: bool,
    /// The CMD A and CMD B lamps.
    pub cmd_a: Option<bool>,
    pub cmd_b: Option<bool>,
    pub fma: Fma,
    pub now_s: f64,
}

/// What the tick does about it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LandOut {
    pub press_second: bool,
    pub press_toga: bool,
    /// Taken back this tick (said by the caller, [`fallback_line`]).
    pub fallback: Option<Fallback>,
    /// The landing armed this tick: said once.
    pub armed_line: Option<&'static str>,
}

impl Landing {
    /// The airframe flies the final and the landing: our arrest, flare
    /// column and roll channel stand aside.
    pub fn ap_flies(&self) -> bool {
        matches!(
            self,
            Landing::SecondCmd { .. } | Landing::Dual | Landing::Land
        )
    }

    /// Still the autopilot's approach (not taken back).
    pub fn live(&self) -> bool {
        !matches!(self, Landing::Fallen(_))
    }

    pub fn word(&self) -> &'static str {
        match self {
            Landing::Approach => "approach",
            Landing::SecondCmd { .. } => "second_cmd",
            Landing::Dual => "dual",
            Landing::Land => "land",
            Landing::Fallen(Fallback::OwnLaw(_)) => "own_law",
            Landing::Fallen(Fallback::GoAround(_)) => "go_around",
        }
    }

    /// One tick. Pure.
    pub fn step(&mut self, i: LandIn, cfg: &AutopilotLands) -> LandOut {
        let mut out = LandOut::default();
        if matches!(self, Landing::Fallen(_)) {
            return out;
        }
        let flying = self.ap_flies();
        // A go-around from anyone: TO/GA where the autopilot was flying it,
        // then the missed approach is flown as any other.
        if i.going_around {
            *self = Landing::Fallen(Fallback::GoAround("go_around"));
            out.press_toga = flying;
            return out;
        }
        // The rollout is decided on the ground ([`rollout_hand_back`]).
        if !i.airborne {
            return out;
        }
        let below = |gate: f64| i.radio_ft > 0.0 && i.radio_ft < gate;
        // Taken back at a height: our law above the land gate (there is
        // room to fly an approach), a go-around below it, our flare under
        // the flare floor (never a go-around from the wheels).
        let by_height = |why: &'static str| {
            if i.radio_ft <= 0.0 || i.radio_ft >= cfg.land_by_agl_ft {
                Fallback::OwnLaw(why)
            } else if i.radio_ft >= cfg.own_flare_below_agl_ft {
                Fallback::GoAround(why)
            } else {
                Fallback::OwnLaw(why)
            }
        };
        let fall = |s: &mut Landing, out: &mut LandOut, f: Fallback| {
            *s = Landing::Fallen(f);
            out.fallback = Some(f);
            out.press_toga = flying && matches!(f, Fallback::GoAround(_));
        };
        match *self {
            Landing::Approach => match i.coupling {
                Coupling::Abandoned => fall(self, &mut out, Fallback::OwnLaw("coupling_lost")),
                Coupling::Coupled => {
                    *self = Landing::SecondCmd {
                        pressed_s: None,
                        presses: 0,
                    };
                    return self.step(i, cfg);
                }
                _ if below(cfg.dual_by_agl_ft) => {
                    fall(self, &mut out, Fallback::OwnLaw("not_captured"))
                }
                _ => {}
            },
            Landing::SecondCmd { pressed_s, presses } => {
                if i.coupling != Coupling::Coupled {
                    fall(self, &mut out, by_height("coupling_lost"));
                } else if i.cmd_a == Some(true) && i.cmd_b == Some(true) {
                    *self = Landing::Dual;
                    return self.step(i, cfg);
                } else if below(cfg.dual_by_agl_ft) {
                    fall(self, &mut out, Fallback::OwnLaw("no_dual"));
                } else if i.cmd_b == Some(false)
                    && presses < SECOND_CMD_PRESSES
                    && pressed_s.is_none_or(|t| i.now_s - t >= crate::autoflight::PRESS_SETTLE_S)
                {
                    out.press_second = true;
                    *self = Landing::SecondCmd {
                        pressed_s: Some(i.now_s),
                        presses: presses + 1,
                    };
                }
            }
            Landing::Dual => {
                if i.coupling != Coupling::Coupled
                    || i.cmd_a == Some(false)
                    || i.cmd_b == Some(false)
                {
                    fall(self, &mut out, by_height("dual_lost"));
                } else if i.fma.landing() == Some(true) {
                    *self = Landing::Land;
                    out.armed_line = Some(if i.fma.flare_armed == Some(true) {
                        "Flare armed."
                    } else {
                        "Autoland armed."
                    });
                } else if below(cfg.land_by_agl_ft) {
                    fall(self, &mut out, by_height("no_land"));
                }
            }
            Landing::Land => {
                let ap_gone = i.cmd_a == Some(false) && i.cmd_b == Some(false);
                if ap_gone {
                    fall(self, &mut out, by_height("ap_lost"));
                } else if i.fma.vertical() == Some(false) || i.fma.lateral() == Some(false) {
                    fall(self, &mut out, by_height("modes_lost"));
                }
            }
            Landing::Fallen(_) => {}
        }
        out
    }
}

/// What the copilot says when it takes a landing back. Pure.
pub fn fallback_line(f: Fallback, radio_ft: f64, cfg: &AutopilotLands) -> String {
    let why = match f.why() {
        "not_captured" => "the ILS didn't capture",
        "no_dual" => "the second autopilot didn't engage",
        "coupling_lost" => "the ILS coupling dropped",
        "dual_lost" => "we lost the dual autopilot",
        "no_land" => "no flare armed",
        "ap_lost" => "the autopilot dropped out",
        "modes_lost" => "the autopilot left the approach modes",
        _ => "the autopilot isn't landing",
    };
    match f {
        Fallback::GoAround(_) => format!("No autoland, {why}. Going around."),
        Fallback::OwnLaw(_) if radio_ft > 0.0 && radio_ft < cfg.own_flare_below_agl_ft => {
            format!("Autopilot's out, {why}. I have the landing.")
        }
        Fallback::OwnLaw(_) => format!("No autoland, {why}. I'm flying the approach myself."),
    }
}

// ---------------------------------------------------------------------------
// The thrust: the A/T in SPEED on the MCP window.
// ---------------------------------------------------------------------------

/// The speed window must read within this of the target (knots).
pub const WINDOW_TOL_KTS: f64 = 5.0;
/// ...and is given this long to (seconds) before the A/T is let go.
pub const WINDOW_WRONG_S: f64 = 6.0;
/// The SPEED button is pressed at most this many times.
pub const SPEED_PRESSES: u32 = 2;
/// A speed the A/T does not hold ([`AtIn::astray`]) this long (seconds) is
/// let go to our thrust law: long enough for an airliner to accelerate
/// through a new target at climb thrust, short of a climb bleeding away.
pub const NOT_HOLDING_S: f64 = 30.0;

/// The autothrottle holding a speed: the Zibo's approach (`ap_lands`), and
/// the speed governor's airborne hold on a lever-driven A/THR
/// (`speedhold` `athr`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum AtSpeed {
    /// Not declared, or not begun.
    #[default]
    Off,
    /// The arm switch, the window and the SPEED button, read back.
    Engaging {
        since_s: f64,
        pressed_s: Option<f64>,
        presses: u32,
    },
    /// The A/T holds the window: our lever stands aside.
    Ours {
        off_since: Option<f64>,
        wrong_since: Option<f64>,
        astray_since: Option<f64>,
    },
    /// Our thrust law's for the rest of this approach (or this hold).
    Released,
}

/// One tick's facts for [`AtSpeed::step`].
#[derive(Debug, Clone, Copy)]
pub struct AtIn {
    /// The A/T is wanted: declared, the approach the autopilot's, airborne,
    /// no go-around.
    pub wanted: bool,
    /// The A/T in a speed mode: SPEED lit, or the FMA's MCP SPD or RETARD.
    /// `None` unread.
    pub engaged: Option<bool>,
    /// The speed window as it reads back.
    pub window_kts: Option<f64>,
    /// The approach target, knots.
    pub want_kts: f64,
    pub now_s: f64,
    /// The speed is not being held this tick (`speedhold::athr_astray`);
    /// `false` where nobody judges it (the Zibo's approach).
    pub astray: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AtOut {
    /// Bring the A/T arm switch on (`autoflight::ensure_switch`), once.
    pub arm_switch: bool,
    /// The window wanted (written on a change only, by the caller).
    pub set_window: Option<f64>,
    pub press_speed: bool,
    pub say: Option<&'static str>,
    /// For the tape.
    pub event: Option<&'static str>,
}

impl AtSpeed {
    /// The A/T holds the speed: our lever and the thrust watch stand aside.
    pub fn flies(&self) -> bool {
        matches!(self, AtSpeed::Ours { .. })
    }

    /// The thrust watch (`autoflight::ThrustWatch`) leaves an engaged A/T
    /// alone: this path is engaging it, or it holds the speed.
    pub fn quiets_thrust_watch(&self) -> bool {
        matches!(self, AtSpeed::Engaging { .. } | AtSpeed::Ours { .. })
    }

    pub fn word(&self) -> &'static str {
        match self {
            AtSpeed::Off => "off",
            AtSpeed::Engaging { .. } => "engaging",
            AtSpeed::Ours { .. } => "ours",
            AtSpeed::Released => "released",
        }
    }

    /// One tick. Pure.
    pub fn step(&mut self, i: AtIn) -> AtOut {
        let mut out = AtOut::default();
        let want = i.want_kts.round();
        let window_ok = i
            .window_kts
            .is_some_and(|w| (w - want).abs() <= WINDOW_TOL_KTS);
        match *self {
            AtSpeed::Off => {
                if i.wanted {
                    *self = AtSpeed::Engaging {
                        since_s: i.now_s,
                        pressed_s: None,
                        presses: 0,
                    };
                    out.arm_switch = true;
                    out.set_window = Some(want);
                    out.event = Some("at_engaging");
                }
            }
            AtSpeed::Engaging {
                since_s,
                pressed_s,
                presses,
            } => {
                if !i.wanted {
                    *self = AtSpeed::Released;
                    out.event = Some("at_released");
                    return out;
                }
                out.set_window = Some(want);
                let settled =
                    pressed_s.is_none_or(|t| i.now_s - t >= crate::autoflight::PRESS_SETTLE_S);
                if i.engaged == Some(true) && window_ok {
                    *self = AtSpeed::Ours {
                        off_since: None,
                        wrong_since: None,
                        astray_since: None,
                    };
                    out.say = Some("Autothrottle in speed.");
                    out.event = Some("at_speed");
                } else if i.engaged == Some(true) && i.now_s - since_s >= WINDOW_WRONG_S {
                    *self = AtSpeed::Released;
                    out.say = Some("The speed window won't set. I have the thrust.");
                    out.event = Some("at_window_wrong");
                } else if i.engaged == Some(false) && settled {
                    if presses < SPEED_PRESSES {
                        out.press_speed = true;
                        *self = AtSpeed::Engaging {
                            since_s,
                            pressed_s: Some(i.now_s),
                            presses: presses + 1,
                        };
                    } else {
                        *self = AtSpeed::Released;
                        out.say =
                            Some("The autothrottle didn't take the speed. I have the thrust.");
                        out.event = Some("at_not_engaged");
                    }
                }
            }
            AtSpeed::Ours {
                off_since,
                wrong_since,
                astray_since,
            } => {
                if !i.wanted {
                    *self = AtSpeed::Released;
                    out.event = Some("at_released");
                    return out;
                }
                out.set_window = Some(want);
                let off_since = match i.engaged {
                    Some(false) => Some(off_since.unwrap_or(i.now_s)),
                    _ => None,
                };
                let wrong_since = match (i.window_kts, window_ok) {
                    (Some(_), false) => Some(wrong_since.unwrap_or(i.now_s)),
                    _ => None,
                };
                let astray_since = i.astray.then(|| astray_since.unwrap_or(i.now_s));
                if off_since.is_some_and(|t| i.now_s - t >= crate::autoflight::PRESS_SETTLE_S) {
                    *self = AtSpeed::Released;
                    out.say = Some("Autothrottle's off. I have the thrust.");
                    out.event = Some("at_dropped");
                } else if wrong_since.is_some_and(|t| i.now_s - t >= WINDOW_WRONG_S) {
                    *self = AtSpeed::Released;
                    out.say = Some("The speed window won't set. I have the thrust.");
                    out.event = Some("at_window_wrong");
                } else if astray_since.is_some_and(|t| i.now_s - t >= NOT_HOLDING_S) {
                    *self = AtSpeed::Released;
                    out.say = Some("The autothrottle isn't holding the speed. I have the thrust.");
                    out.event = Some("at_not_holding");
                } else {
                    *self = AtSpeed::Ours {
                        off_since,
                        wrong_since,
                        astray_since,
                    };
                }
            }
            AtSpeed::Released => {}
        }
        out
    }
}

/// The A/T in a speed mode, from the FMA's MCP SPD or RETARD, else the
/// SPEED lamp. `None` when neither read. Pure.
pub fn at_engaged(fma: &Fma, speed_lamp: Option<f64>) -> Option<bool> {
    let lamp = speed_lamp.filter(|v| v.is_finite()).map(|v| v >= 0.5);
    either(either(fma.speed, fma.retard), lamp)
}

// ---------------------------------------------------------------------------
// The rollout.
// ---------------------------------------------------------------------------

/// ROLLOUT not annunciated this long on the wheels: the steering is ours.
pub const ROLLOUT_FMA_GRACE_S: f64 = 3.0;

/// Whether the autopilot's rollout is handed back this tick, and why:
/// the autopilot out, past the cross-track limit, no ROLLOUT annunciated
/// after [`ROLLOUT_FMA_GRACE_S`] on the wheels, or taxi speed. Pure.
pub fn rollout_hand_back(
    gs_kts: f64,
    xtk_m: f64,
    rollout_fma: Option<bool>,
    wheels_s: f64,
    ap_in: Option<bool>,
    cfg: &AutopilotLands,
) -> Option<&'static str> {
    if ap_in == Some(false) {
        return Some("ap_off");
    }
    if xtk_m.is_finite() && xtk_m.abs() > cfg.rollout_xtk_limit_m {
        return Some("off_centreline");
    }
    if rollout_fma == Some(false) && wheels_s > ROLLOUT_FMA_GRACE_S {
        return Some("no_rollout");
    }
    if gs_kts <= cfg.disconnect_kts {
        return Some("taxi_speed");
    }
    None
}

/// The line for a rollout handed back, if any is said. Pure.
pub fn rollout_line(why: &str) -> Option<&'static str> {
    match why {
        "off_centreline" => Some("Off the centreline. Autopilot off, I have the rollout."),
        "no_rollout" => Some("No rollout mode. Autopilot off, I have the rollout."),
        "ap_off" => Some("Autopilot's out. I have the rollout."),
        _ => None,
    }
}

/// The tick's record on this path (`autoland` `coupled_tick`): every FMA
/// column raw, beside the decisions.
#[allow(clippy::too_many_arguments)]
pub fn tick_record(
    cfg: &AutopilotLands,
    r: &Reads,
    fma: &Fma,
    landing: &Landing,
    at: &AtSpeed,
    coupling: Coupling,
    cmd_a: Option<bool>,
    radio_ft: f64,
    ias: f64,
    want_kts: Option<f64>,
    on_ground: bool,
    rollout_ap: bool,
) -> serde_json::Value {
    let log: serde_json::Map<String, serde_json::Value> = cfg
        .fma
        .log
        .iter()
        .zip(r.log.iter())
        .map(|(n, v)| (n.rsplit('/').next().unwrap_or(n).to_string(), json!(v)))
        .collect();
    json!({
        "event": "coupled_tick",
        "roll": r.roll, "roll_arm": r.roll_armed, "pitch": r.pitch, "pitch_arm": r.pitch_armed,
        "thrust": r.thrust, "land": r.land, "cmd_a": cmd_a, "cmd_b": r.cmd_b,
        "speed_lamp": r.speed_lamp, "spd_window": r.speed_window, "spd_want": want_kts,
        "loc": fma.lateral(), "gs": fma.vertical(), "flare_armed": fma.flare_armed,
        "flare": fma.flare, "rollout": fma.rollout, "retard": fma.retard,
        "landing": landing.word(), "at": at.word(), "coupling": format!("{coupling:?}"),
        "radio": radio_ft, "ias": ias, "ground": on_ground, "rollout_ap": rollout_ap,
        "log": log,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The Zibo's own declaration, as the profile carries it.
    fn zibo() -> AutopilotLands {
        crate::aircraft::load("zibo_738")
            .expect("zibo profile")
            .autopilot_lands
            .expect("the Zibo lands on its autopilot")
    }

    fn reads(roll: f64, pitch: f64, pitch_arm: f64) -> Reads {
        Reads {
            roll: Some(roll),
            roll_armed: Some(0.0),
            pitch: Some(pitch),
            pitch_armed: Some(pitch_arm),
            thrust: Some(3.0),
            land: Some(0.0),
            ..Default::default()
        }
    }

    fn at(
        coupling: Coupling,
        radio: f64,
        cmd_a: bool,
        cmd_b: bool,
        fma: Fma,
        now_s: f64,
    ) -> LandIn {
        LandIn {
            coupling,
            radio_ft: radio,
            airborne: true,
            going_around: false,
            cmd_a: Some(cmd_a),
            cmd_b: Some(cmd_b),
            fma,
            now_s,
        }
    }

    /// The codes are the Zibo's own PFD images (b738.acf `SHOW_EQUAL`):
    /// roll 2 VOR/LOC, 4 ROLLOUT; pitch 5 G/S, 6 FLARE, 11 TO/GA; pitch
    /// armed 3 FLARE; thrust 3 MCP SPD, 7 RETARD; land 1 LAND 2, 2 LAND 3.
    #[test]
    fn the_zibo_fma_reads_as_its_pfd_draws_it() {
        let c = zibo();
        let f = Fma::decode(&reads(2.0, 5.0, 3.0), &c.codes);
        assert_eq!(
            (f.lateral(), f.vertical(), f.flare_armed),
            (Some(true), Some(true), Some(true))
        );
        assert_eq!(f.landing(), Some(true));
        let f = Fma::decode(&reads(4.0, 6.0, 0.0), &c.codes);
        assert_eq!(
            (f.rollout, f.flare, f.lateral(), f.vertical()),
            (Some(true), Some(true), Some(true), Some(true))
        );
        // HDG SEL and V/S: neither captured.
        let f = Fma::decode(&reads(1.0, 1.0, 1.0), &c.codes);
        assert_eq!(
            (f.lateral(), f.vertical(), f.landing()),
            (Some(false), Some(false), Some(false))
        );
        // An unread column decides nothing.
        let f = Fma::decode(&Reads::default(), &c.codes);
        assert_eq!((f.lateral(), f.vertical(), f.landing()), (None, None, None));
        let mut r = reads(2.0, 5.0, 0.0);
        r.thrust = Some(7.0);
        assert_eq!(
            at_engaged(&Fma::decode(&r, &c.codes), Some(0.0)),
            Some(true),
            "RETARD is the A/T's"
        );
    }

    /// The APP lamp is lit armed or captured alike: the captures are the
    /// FMA's, so the coupling can hand the bug and the wheel over.
    #[test]
    fn the_captures_are_the_fmas() {
        let c = zibo();
        let armed = Modes(Modes::LOC_ARMED | Modes::GS_ARMED | Modes::HEADING);
        let m = modes_with_fma(Some(armed), &Fma::decode(&reads(2.0, 1.0, 1.0), &c.codes)).unwrap();
        assert!(m.loc() && !m.gs());
        let m = modes_with_fma(Some(armed), &Fma::decode(&reads(2.0, 5.0, 0.0), &c.codes)).unwrap();
        assert!(m.loc() && m.gs());
        // The flare and the rollout keep the coupling (it abandons on a
        // dropped LOC or G/S).
        let m = modes_with_fma(
            Some(Modes(0)),
            &Fma::decode(&reads(4.0, 6.0, 0.0), &c.codes),
        )
        .unwrap();
        assert!(m.loc() && m.gs());
        assert_eq!(
            modes_with_fma(None, &Fma::decode(&reads(2.0, 5.0, 0.0), &c.codes)),
            None
        );
    }

    /// The whole landing as a crew flies it: captured, CMD B pressed once
    /// and read, the flare armed, the autopilot flying it to the wheels.
    #[test]
    fn captured_dual_and_armed_is_the_autopilots_landing() {
        let c = zibo();
        let gs = Fma::decode(&reads(2.0, 5.0, 0.0), &c.codes);
        let armed = Fma::decode(&reads(2.0, 5.0, 3.0), &c.codes);
        let mut l = Landing::default();
        assert_eq!(
            l.step(at(Coupling::Armed, 2500.0, true, false, gs, 0.0), &c),
            LandOut::default()
        );
        assert!(!l.ap_flies());
        let o = l.step(at(Coupling::Coupled, 2300.0, true, false, gs, 1.0), &c);
        assert!(o.press_second && l.ap_flies());
        // Read back dark within the settle: never pressed again yet (a toggle).
        assert!(
            !l.step(at(Coupling::Coupled, 2280.0, true, false, gs, 2.0), &c)
                .press_second
        );
        assert_eq!(
            l.step(at(Coupling::Coupled, 2260.0, true, true, gs, 3.5), &c),
            LandOut::default()
        );
        assert_eq!(l, Landing::Dual);
        let o = l.step(at(Coupling::Coupled, 1450.0, true, true, armed, 30.0), &c);
        assert_eq!((l, o.armed_line), (Landing::Land, Some("Flare armed.")));
        // Down to the wheels in FLARE: still the autopilot's.
        let flare = Fma::decode(&reads(2.0, 6.0, 0.0), &c.codes);
        assert_eq!(
            l.step(at(Coupling::Coupled, 30.0, true, true, flare, 60.0), &c),
            LandOut::default()
        );
        assert!(l.ap_flies());
    }

    /// The second channel is pressed again only after the settle, at most
    /// twice; not in by the dual gate, our own law flies the approach.
    #[test]
    fn no_dual_by_the_gate_is_our_own_approach() {
        let c = zibo();
        let gs = Fma::decode(&reads(2.0, 5.0, 0.0), &c.codes);
        let mut l = Landing::default();
        let mut presses = 0;
        for (k, radio) in [2400.0, 2300.0, 2200.0, 2100.0, 2000.0, 1900.0]
            .iter()
            .enumerate()
        {
            presses += l
                .step(
                    at(Coupling::Coupled, *radio, true, false, gs, k as f64 * 1.5),
                    &c,
                )
                .press_second as u32;
        }
        assert_eq!(presses, SECOND_CMD_PRESSES);
        let o = l.step(at(Coupling::Coupled, 1490.0, true, false, gs, 20.0), &c);
        assert_eq!(o.fallback, Some(Fallback::OwnLaw("no_dual")));
        assert!(!o.press_toga && !l.ap_flies());
        assert!(
            fallback_line(o.fallback.unwrap(), 1490.0, &c).contains("flying the approach myself")
        );
        // Not captured by the gate: the same.
        let mut l = Landing::default();
        let o = l.step(at(Coupling::LocCaptured, 1400.0, true, false, gs, 0.0), &c);
        assert_eq!(o.fallback, Some(Fallback::OwnLaw("not_captured")));
    }

    /// No landing armed by the land gate is a go-around with height in
    /// hand, TO/GA pressed (the autopilot was flying); lost low is our
    /// flare, never a go-around from the wheels.
    #[test]
    fn a_landing_not_armed_goes_around_and_a_low_loss_is_our_flare() {
        let c = zibo();
        let gs = Fma::decode(&reads(2.0, 5.0, 0.0), &c.codes);
        let mut l = Landing::Dual;
        let o = l.step(at(Coupling::Coupled, 990.0, true, true, gs, 0.0), &c);
        assert_eq!(o.fallback, Some(Fallback::GoAround("no_land")));
        assert!(o.press_toga);
        let mut l = Landing::Land;
        let o = l.step(at(Coupling::Coupled, 400.0, false, false, gs, 0.0), &c);
        assert_eq!(o.fallback, Some(Fallback::GoAround("ap_lost")));
        let mut l = Landing::Land;
        let o = l.step(at(Coupling::Coupled, 80.0, false, false, gs, 0.0), &c);
        assert_eq!(o.fallback, Some(Fallback::OwnLaw("ap_lost")));
        assert!(!o.press_toga);
        assert!(fallback_line(o.fallback.unwrap(), 80.0, &c).contains("I have the landing"));
        // Out of G/S (V/S on the pitch column) in the landing: taken back.
        let mut l = Landing::Land;
        let vs = Fma::decode(&reads(2.0, 1.0, 0.0), &c.codes);
        assert_eq!(
            l.step(at(Coupling::Coupled, 600.0, true, true, vs, 0.0), &c)
                .fallback,
            Some(Fallback::GoAround("modes_lost"))
        );
        // An unread FMA is no measurement: nothing taken back on it.
        let mut l = Landing::Land;
        assert_eq!(
            l.step(
                at(Coupling::Coupled, 600.0, true, true, Fma::default(), 0.0),
                &c
            ),
            LandOut::default()
        );
    }

    /// Any go-around (the tower's, the pilot's, the stabilised-approach
    /// rule's) while the autopilot flies presses TO/GA once; the ground
    /// decides nothing here.
    #[test]
    fn a_go_around_presses_toga_once() {
        let c = zibo();
        let mut l = Landing::Land;
        let mut i = at(Coupling::Coupled, 300.0, true, true, Fma::default(), 0.0);
        i.going_around = true;
        assert!(l.step(i, &c).press_toga);
        assert!(!l.step(i, &c).press_toga);
        let mut l = Landing::Approach;
        assert!(
            !l.step(i, &c).press_toga,
            "nothing of the autopilot's to go around with"
        );
        let mut l = Landing::Land;
        let mut i = at(Coupling::Coupled, 0.0, false, false, Fma::default(), 0.0);
        i.airborne = false;
        assert_eq!(l.step(i, &c), LandOut::default());
        assert_eq!(l, Landing::Land);
    }

    /// The A/T: arm switch and window first, SPEED pressed (read before,
    /// settled), ours once lit on the window; not lit after two presses,
    /// or a window that will not set, is our thrust again, said.
    #[test]
    fn the_autothrottle_takes_the_speed_or_hands_it_back() {
        let i = |engaged, window, now_s| AtIn {
            wanted: true,
            engaged,
            window_kts: window,
            want_kts: 140.4,
            now_s,
            astray: false,
        };
        let mut a = AtSpeed::default();
        let o = a.step(i(Some(false), Some(100.0), 0.0));
        assert!(o.arm_switch && o.set_window == Some(140.0) && !o.press_speed);
        assert!(a.step(i(Some(false), Some(140.0), 0.5)).press_speed);
        assert!(
            !a.step(i(Some(false), Some(140.0), 1.0)).press_speed,
            "settling"
        );
        let o = a.step(i(Some(true), Some(140.0), 3.0));
        assert!(a.flies() && o.say.is_some());
        // Lit through the retard: the FMA's RETARD counts as engaged.
        assert!(a.step(i(Some(true), Some(140.0), 10.0)).say.is_none() && a.flies());
        // Dark for the settle: ours again, said.
        a.step(i(Some(false), Some(140.0), 11.0));
        let o = a.step(i(Some(false), Some(140.0), 13.5));
        assert_eq!((a, o.event), (AtSpeed::Released, Some("at_dropped")));

        let mut a = AtSpeed::default();
        let mut presses = 0;
        let mut last = AtOut::default();
        for k in 0..10 {
            last = a.step(i(Some(false), Some(140.0), k as f64 * 2.5));
            presses += last.press_speed as u32;
        }
        assert_eq!((presses, a), (SPEED_PRESSES, AtSpeed::Released));
        let _ = last;

        let mut a = AtSpeed::default();
        a.step(i(Some(false), Some(100.0), 0.0));
        a.step(i(Some(true), Some(100.0), 1.0));
        let o = a.step(i(Some(true), Some(100.0), 7.0));
        assert_eq!(o.event, Some("at_window_wrong"));
        assert!(!a.flies());
        // Not wanted (a fallback, the wheels): let go quietly.
        let mut a = AtSpeed::Ours {
            off_since: None,
            wrong_since: None,
            astray_since: None,
        };
        let mut n = i(Some(true), Some(140.0), 0.0);
        n.wanted = false;
        assert!(a.step(n).say.is_none() && a == AtSpeed::Released);
    }

    /// Engaged on the window but not holding the speed: let go to our
    /// thrust only once astray for [`NOT_HOLDING_S`] together, said; a
    /// tick back in hand starts the clock again.
    #[test]
    fn an_autothrottle_not_holding_the_speed_is_let_go_after_its_time() {
        let i = |astray, now_s| AtIn {
            wanted: true,
            engaged: Some(true),
            window_kts: Some(250.0),
            want_kts: 250.0,
            now_s,
            astray,
        };
        let mut a = AtSpeed::default();
        a.step(i(false, 0.0));
        a.step(i(false, 0.5));
        assert!(a.flies());
        a.step(i(true, 1.0));
        a.step(i(false, 20.0));
        assert!(a.step(i(true, 25.0)).event.is_none());
        assert!(a.step(i(true, 25.0 + NOT_HOLDING_S - 1.0)).event.is_none() && a.flies());
        let o = a.step(i(true, 25.0 + NOT_HOLDING_S));
        assert_eq!((a, o.event), (AtSpeed::Released, Some("at_not_holding")));
        assert!(o.say.is_some());
    }

    /// The rollout is the autopilot's until taxi speed, a cross-track past
    /// the limit, no ROLLOUT after the grace, or the autopilot out.
    #[test]
    fn the_rollout_is_handed_back_at_taxi_speed_or_off_the_line() {
        let c = zibo();
        assert_eq!(
            rollout_hand_back(110.0, 2.0, Some(true), 1.0, Some(true), &c),
            None
        );
        assert_eq!(
            rollout_hand_back(110.0, 2.0, Some(false), 1.0, Some(true), &c),
            None,
            "grace"
        );
        assert_eq!(
            rollout_hand_back(110.0, 2.0, Some(false), 4.0, Some(true), &c),
            Some("no_rollout")
        );
        assert_eq!(
            rollout_hand_back(110.0, -12.0, Some(true), 4.0, Some(true), &c),
            Some("off_centreline")
        );
        assert_eq!(
            rollout_hand_back(29.0, 2.0, Some(true), 9.0, Some(true), &c),
            Some("taxi_speed")
        );
        assert_eq!(
            rollout_hand_back(90.0, 2.0, None, 9.0, Some(false), &c),
            Some("ap_off")
        );
        assert_eq!(
            rollout_hand_back(90.0, 2.0, None, 9.0, None, &c),
            None,
            "unread decides nothing"
        );
        assert!(rollout_line("taxi_speed").is_none() && rollout_line("off_centreline").is_some());
    }

    /// Declared on the Zibo only; every other profile flies autoland's own
    /// law as before.
    #[test]
    fn only_the_zibo_declares_it() {
        let c = zibo();
        assert!(c.nav2 && c.autothrottle);
        assert!(
            (c.dual_by_agl_ft, c.land_by_agl_ft, c.own_flare_below_agl_ft)
                == (1500.0, 1000.0, 200.0)
        );
        for entry in std::fs::read_dir("profiles")
            .expect("profiles dir")
            .flatten()
        {
            let path = entry.path();
            let Some(id) = path.file_stem().and_then(|s| s.to_str()) else {
                continue;
            };
            if path.extension().and_then(|e| e.to_str()) != Some("yaml") || id == "zibo_738" {
                continue;
            }
            let p = crate::aircraft::load(id).expect("profile");
            assert!(p.autopilot_lands.is_none(), "{id}");
        }
        let names =
            crate::aircraft::profile_references(&crate::aircraft::load("zibo_738").unwrap());
        for n in [
            "laminar/B738/autopilot/cmd_b_press",
            "laminar/B738/autopilot/cmd_b_status",
            "laminar/B738/autopilot/pfd_hdg_mode",
            "laminar/B738/autopilot/pfd_alt_mode",
            "laminar/B738/autopilot/pfd_spd_mode",
            "laminar/B738/autopilot/course_pilot",
            "laminar/B738/autopilot/course_copilot",
            "laminar/B738/autopilot/speed_press",
        ] {
            assert!(names.iter().any(|x| x == n), "{n} not probed");
        }
    }
}
