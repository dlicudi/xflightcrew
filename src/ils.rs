//! The ILS as a crew uses it: tuned for every landing that has one, its
//! needles cross-checked against the path the copilot flies, and, on an
//! airframe whose autopilot has been proven on it, flown coupled.
//!
//! Until 2026-09-25 full auto "flew the ILS approach runway one two" at LIPE
//! without ever tuning NAV1: the approach was the copilot's own heading bug
//! and VS dial down a glidepath computed from the runway, the needles dead.
//! A real crew tunes, identifies and watches the raw data whoever flies.
//!
//! - [`tune`]: NAV1 to the runway's ILS frequency and course, read back, at
//!   the approach's setup and at autoland's arm (every landing path).
//! - [`XCheck`]: pure. On final, below [`XCHECK_BELOW_AGL_FT`], a needle
//!   past [`XCHECK_DOTS`] held [`XCHECK_HOLD_S`] is said once per episode:
//!   the copilot's path and the ILS disagree, and the pilot should know.
//! - [`Coupling`]: pure. With the profile's `ils_coupled`, the approach mode
//!   is armed on an intercept to the localizer (the final, a straight-in or
//!   a pattern's inbound heading for it inside its coverage) from below the
//!   beam, pressed only when neither half reads in; once the localizer
//!   (then the glideslope) is CAPTURED the copilot stops writing the heading
//!   bug (then the VS dial), and the panel's owner never presses HDG or V/S
//!   over them (`autoflight::reconcile`: those presses dropped 34 captures
//!   on the 737, #81). The modes are the `autopilot_state` bitfield's: the
//!   status datarefs read 0 through captures. The roundout gate's arrest
//!   takes the servos as always; a go-around or any leaving of the final
//!   decouples back to HDG/VS and presses APP once more to take out what is
//!   still in. Off by default: a profile turns it on on the evidence of
//!   captures flown on the type.

use crate::autoflight::{self, Button, Dial};
use serde_json::json;

pub const NAV1_FREQ: &str = "sim/cockpit2/radios/actuators/nav1_frequency_hz";
pub const NAV1_OBS: &str = "sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot";
pub const HDEF: &str = "sim/cockpit2/radios/indicators/nav1_hdef_dots_pilot";
pub const VDEF: &str = "sim/cockpit2/radios/indicators/nav1_vdef_dots_pilot";
pub const LOC_VALID: &str = "sim/cockpit2/radios/indicators/nav1_display_horizontal";
pub const GS_VALID: &str = "sim/cockpit2/radios/indicators/nav1_display_vertical";

/// How far an ILS may be from the runway's field and still be its own.
const FIND_MAX_NM: f64 = 40.0;

/// Tune NAV1 to `icao` runway `rwy`'s ILS and set its course, reading both
/// back. `None` when the runway has no ILS in the data (a visual or RNAV
/// runway: nothing to tune, nothing said). Idempotent: an already-tuned
/// radio is left alone.
pub async fn tune(
    xp: &dyn crate::xplane::XPlane,
    icao: &str,
    rwy: &str,
    lat: f64,
    lon: f64,
) -> Option<crate::nav::Ils> {
    let nav_path = crate::settings::live().nav_data_path.clone();
    tune_from(xp, &nav_path, icao, rwy, lat, lon).await
}

/// [`tune`] against a named `earth_nav.dat` (tests hand it a fixture).
pub async fn tune_from(
    xp: &dyn crate::xplane::XPlane,
    nav_path: &str,
    icao: &str,
    rwy: &str,
    lat: f64,
    lon: f64,
) -> Option<crate::nav::Ils> {
    let nav_path = nav_path.to_string();
    let (want_rwy, want_icao) = (rwy.to_string(), icao.to_string());
    let ils = tokio::task::spawn_blocking(move || {
        crate::nav::find_ils(&nav_path, &want_rwy, lat, lon, FIND_MAX_NM).ok().flatten()
    })
    .await
    .ok()
    .flatten()
    .filter(|i| i.airport.eq_ignore_ascii_case(&want_icao) && i.runway.eq_ignore_ascii_case(rwy))?;
    // A declared MCP course window is the OBS's source (the Zibo's
    // `course_pilot`: its script writes NAV1's OBS from it until the
    // localiser captures, so a raw OBS write is undone): the course is set
    // and read on the window, through the panel's owner, in whole degrees.
    let course_window = autoflight::panel().window(Dial::Course).is_some();
    let read_course = || async {
        if course_window {
            autoflight::read_dial(xp, Dial::Course).await
        } else {
            xp.get_float(NAV1_OBS).await
        }
    };
    let (freq, obs) = tokio::join!(xp.get_float(NAV1_FREQ), read_course());
    let tuned = freq.is_ok_and(|f| f.round() as i64 == ils.freq_10khz);
    let on_course = obs.is_ok_and(|o| crate::angles::abs_diff_deg(o, ils.course_mag) < 1.0);
    if !tuned {
        let _ = xp.set_int(NAV1_FREQ, ils.freq_10khz).await;
    }
    if !on_course {
        if course_window {
            let _ = autoflight::set_dial(xp, Dial::Course, ils.course_mag.round().rem_euclid(360.0)).await;
        } else {
            let _ = xp.set_float(NAV1_OBS, ils.course_mag).await;
        }
    }
    let (freq, obs) = tokio::join!(xp.get_float(NAV1_FREQ), read_course());
    let ok = freq.is_ok_and(|f| f.round() as i64 == ils.freq_10khz)
        && obs.is_ok_and(|o| crate::angles::abs_diff_deg(o, ils.course_mag) < 1.0);
    crate::blackbox::record(
        "ils",
        json!({ "event": "tuned", "icao": icao, "rwy": rwy, "freq_mhz": ils.freq_mhz(),
                "course": ils.course_mag, "already": tuned && on_course, "ok": ok }),
    );
    ok.then_some(ils)
}

/// A runway's glideslope antenna as `earth_nav.dat` places it (row code 6):
/// where the beam the needles read starts, its site elevation and angle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Glideslope {
    pub lat: f64,
    pub lon: f64,
    pub elev_ft: f64,
    pub angle_deg: f64,
}

/// The glideslope row of `icao` runway `rwy` in `earth_nav.dat` text. The
/// bearing field is `angle x 100000 + course` with the angle in hundredths
/// (300145.887 is 3.00 degrees on 145.887). `None` without one. Pure over
/// the text.
pub fn parse_glideslope<'a>(lines: impl Iterator<Item = &'a str>, icao: &str, rwy: &str) -> Option<Glideslope> {
    lines.filter(|l| l.trim_start().starts_with("6 ")).find_map(|l| {
        let f: Vec<&str> = l.split_whitespace().collect();
        if f.len() < 11 || !f[8].eq_ignore_ascii_case(icao) || !f[10].eq_ignore_ascii_case(rwy) {
            return None;
        }
        let num = |i: usize| f[i].parse::<f64>().ok().filter(|v| v.is_finite());
        let angle_deg = (num(6)? / 1_000.0).floor() / 100.0;
        (1.5..=8.0).contains(&angle_deg).then_some(Glideslope {
            lat: num(1)?,
            lon: num(2)?,
            elev_ft: num(3)?,
            angle_deg,
        })
    })
}

/// [`parse_glideslope`] from the nav file on disk.
pub fn find_glideslope(nav_path: &str, icao: &str, rwy: &str) -> Option<Glideslope> {
    let text = std::fs::read_to_string(nav_path).ok()?;
    parse_glideslope(text.lines(), icao, rwy)
}

/// The aim elevation, ft, that puts a glidepath of `gp_deg` aimed
/// `aim_nm` past the threshold ON the glideslope `gs` (the needles'
/// path: the antenna's elevation, its angle, from its point abeam on the
/// runway `course_true` from the threshold). `None` when the angles
/// differ by more than a tenth of a degree (the path could only cross the
/// beam, never lie on it). Pure.
pub fn glideslope_aim_ft(
    gs: &Glideslope,
    thr_lat: f64,
    thr_lon: f64,
    course_true: f64,
    aim_nm: f64,
    gp_deg: f64,
) -> Option<f64> {
    if (gs.angle_deg - gp_deg).abs() > 0.1 {
        return None;
    }
    let d = crate::nav::haversine_nm(thr_lat, thr_lon, gs.lat, gs.lon);
    let brg = crate::nav::bearing_deg(thr_lat, thr_lon, gs.lat, gs.lon);
    let beyond_nm = d * (brg - course_true).to_radians().cos();
    let ft_per_nm = 6076.12 * gs.angle_deg.to_radians().tan();
    Some(gs.elev_ft + (beyond_nm - aim_nm) * ft_per_nm)
}

/// A needle past this many dots...
pub const XCHECK_DOTS: f64 = 1.0;
/// ...held this long...
pub const XCHECK_HOLD_S: f64 = 8.0;
/// ...below this height on final is said.
pub const XCHECK_BELOW_AGL_FT: f64 = 3000.0;
/// No glideslope call under a decision height's worth of radio height: the
/// crew is landing on the pavement by then, and the beam's origin is an
/// antenna beside the touchdown zone, not the aim (#175: "1.5 dots" at
/// 142 ft on autoland's own path).
pub const XCHECK_GS_FLOOR_AGL_FT: f64 = 200.0;

/// Which side of the glideslope we are, from `nav1_vdef_dots_pilot`.
/// POSITIVE is the aircraft ABOVE the beam (the diamond sits below
/// centre, fly down), verified against earth_nav.dat's antennas on every
/// recorded call (#175: 47 of 47, e.g. EGBB 15 +1.37 at 221 ft above the
/// beam, LEMG 13 -2.50 at 213 ft below it). DataRefs.txt only says "CDI
/// vertical deflection in dots".
pub fn above_glideslope(vdef_dots: f64) -> bool {
    vdef_dots > 0.0
}

/// The raw-data cross-check. Pure.
#[derive(Debug, Default)]
pub struct XCheck {
    loc_out_s: f64,
    gs_out_s: f64,
    loc_said: bool,
    gs_said: bool,
}

/// What one tick of needles reads.
#[derive(Debug, Clone, Copy, Default)]
pub struct Needles {
    pub hdef: Option<f64>,
    pub vdef: Option<f64>,
    pub loc_valid: bool,
    pub gs_valid: bool,
}

fn side(dots: f64, pos: &'static str, neg: &'static str) -> &'static str {
    if dots > 0.0 {
        pos
    } else {
        neg
    }
}

impl XCheck {
    /// One tick: a line to say when a needle has stood out long enough.
    /// `on_final` gates it (an intercept is SUPPOSED to be off the needles).
    pub fn step(&mut self, n: Needles, on_final: bool, agl_ft: f64, dt: f64) -> Option<String> {
        let live = on_final && agl_ft < XCHECK_BELOW_AGL_FT;
        let loc = n.hdef.filter(|_| live && n.loc_valid);
        let gs_live = live && agl_ft >= XCHECK_GS_FLOOR_AGL_FT;
        let gs = n.vdef.filter(|_| gs_live && n.gs_valid);
        let mut out = None;
        match loc {
            Some(h) if h.abs() > XCHECK_DOTS => {
                self.loc_out_s += dt;
                if self.loc_out_s >= XCHECK_HOLD_S && !self.loc_said {
                    self.loc_said = true;
                    out = Some(format!(
                        "Localizer shows {:.1} dots {}, the ILS disagrees with our path.",
                        h.abs(),
                        side(h, "right", "left")
                    ));
                }
            }
            _ => {
                self.loc_out_s = 0.0;
                if loc.is_some_and(|h| h.abs() < XCHECK_DOTS * 0.5) {
                    self.loc_said = false;
                }
            }
        }
        match gs {
            Some(v) if v.abs() > XCHECK_DOTS => {
                self.gs_out_s += dt;
                if self.gs_out_s >= XCHECK_HOLD_S && !self.gs_said && out.is_none() {
                    self.gs_said = true;
                    let high = above_glideslope(v);
                    out = Some(format!(
                        "Glideslope {:.1} dots {}, we're {} the ILS path.",
                        v.abs(),
                        if high { "high" } else { "low" },
                        if high { "above" } else { "below" }
                    ));
                }
            }
            _ => {
                self.gs_out_s = 0.0;
                if gs.is_some_and(|v| v.abs() < XCHECK_DOTS * 0.5) {
                    self.gs_said = false;
                }
            }
        }
        out
    }
}

/// The coupled approach's state. Pure; see the module docs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Coupling {
    #[default]
    Off,
    /// APP pressed (or found armed), waiting for the localizer to capture.
    Armed,
    /// The localizer is the autopilot's; the glideslope maybe not yet.
    LocCaptured,
    /// Both captured: the autopilot flies the ILS until the roundout gate.
    Coupled,
    /// Given up for this approach (decoupled, or APP would not arm).
    Abandoned,
}

/// What the tick does about it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CouplingOut {
    /// Press the approach button this tick (only ever from off: a toggle).
    pub press_appr: bool,
    /// Hand the lateral and the vertical back this tick.
    pub decouple: bool,
    /// ...and press APP once more to take out what is still armed or
    /// captured, so nothing captures behind our own heading and VS.
    pub disarm: bool,
    /// The copilot still writes the heading bug and keeps HDG engaged.
    pub own_lateral: bool,
    /// The copilot still writes the VS dial and keeps VS engaged.
    pub own_vertical: bool,
}

/// The aircraft may be at most this far ABOVE the glideslope to arm
/// (dots): the capture is flown from below, as the airframe's own capture
/// was on 2026-10-05 (ALT HOLD at 4,000 under the beam, captured 13:25:45).
pub const ARM_MAX_ABOVE_GS_DOTS: f64 = 0.5;
/// ...and the runway this close (nm).
pub const ARM_MAX_NM: f64 = 15.0;
/// APP pressed and neither half reads armed after this long: abandoned.
pub const ARM_TIMEOUT_S: f64 = 6.0;

/// Inputs to one coupling step.
#[derive(Debug, Clone, Copy, Default)]
pub struct CouplingIn {
    pub allowed: bool,
    /// On an intercept to this localizer: the final, or a straight-in's or a
    /// pattern's inbound heading for it inside its coverage
    /// ([`arm_verdict`] `Now`).
    pub inbound: bool,
    pub arresting: bool,
    /// Leaving the approach: a go-around, a pattern's outbound legs.
    pub leaving: bool,
    pub dist_nm: f64,
    pub needles: Needles,
    /// The autopilot's modes (`autoflight::Modes`, the bitfield): the one
    /// truth. `approach_status` read 0 through captures on the 737 (#81).
    pub modes: Option<crate::autoflight::Modes>,
    /// Radio height, feet (0 when unread).
    pub radio_ft: f64,
    /// The airframe's `ils_decouple_agl_ft`: below it the coupling hands the
    /// final back to autoland's own VS law, so the arrest and flare start from
    /// the path it was tuned on, never from a trimmed coupled autopilot.
    pub decouple_below_ft: Option<f64>,
}

impl Coupling {
    /// One tick. `armed_s` counts the time in `Armed` (the caller's clock).
    pub fn step(&mut self, i: CouplingIn, armed_s: &mut f64, dt: f64) -> CouplingOut {
        let mut out = CouplingOut {
            press_appr: false,
            decouple: false,
            disarm: false,
            own_lateral: true,
            own_vertical: true,
        };
        let engaged = matches!(self, Coupling::Armed | Coupling::LocCaptured | Coupling::Coupled);
        let any = i.modes.is_some_and(|m| m.approach_any());
        let abandon = |s: &mut Coupling, out: &mut CouplingOut| {
            *s = Coupling::Abandoned;
            out.decouple = true;
            out.disarm = any;
        };
        // Leaving the final (a go-around, an outbound leg) hands HDG/VS back
        // at once. The arrest is not a leaving: it takes the servos itself.
        if engaged && i.leaving && !i.arresting {
            abandon(self, &mut out);
            return out;
        }
        // The declared hand-back height: LEMD 18L 2026-10-05 16:57, the
        // arrest took the servos at 80 ft out of the 737's glideslope mode,
        // trimmed for the path, and ballooned from 43 ft to 100 ft (-975 to
        // +980 fpm) into a balk. Below this the final is ours again.
        if engaged
            && !i.arresting
            && i.radio_ft > 0.0
            && i.decouple_below_ft.is_some_and(|h| i.radio_ft < h)
        {
            abandon(self, &mut out);
            return out;
        }
        match *self {
            Coupling::Off => {
                let below_gs = !i.needles.gs_valid
                    || i.needles.vdef.is_some_and(|v| v <= ARM_MAX_ABOVE_GS_DOTS);
                if i.allowed
                    && i.inbound
                    && !i.arresting
                    && !i.leaving
                    && i.needles.loc_valid
                    && below_gs
                    && i.dist_nm < ARM_MAX_NM
                    && i.modes.is_some()
                {
                    // A button that toggles: pressed only when neither half
                    // reads armed or captured.
                    out.press_appr = !any;
                    *self = Coupling::Armed;
                    *armed_s = 0.0;
                }
            }
            Coupling::Armed => {
                *armed_s += dt;
                match i.modes {
                    Some(m) if m.loc() && m.gs() => *self = Coupling::Coupled,
                    Some(m) if m.loc() => *self = Coupling::LocCaptured,
                    Some(m) if !m.approach_any() && *armed_s > ARM_TIMEOUT_S => {
                        abandon(self, &mut out);
                        return out;
                    }
                    _ => {}
                }
            }
            Coupling::LocCaptured => match i.modes {
                Some(m) if !(m.loc() || m.loc_armed()) && !i.arresting => {
                    abandon(self, &mut out);
                    return out;
                }
                Some(m) if m.gs() => *self = Coupling::Coupled,
                _ => {}
            },
            Coupling::Coupled => {
                // The glideslope or the localizer dropped out: back to us.
                // The arrest takes the servos and the modes with them.
                if !i.arresting && i.modes.is_some_and(|m| !m.gs() || !m.loc()) {
                    abandon(self, &mut out);
                    return out;
                }
            }
            Coupling::Abandoned => {}
        }
        out.own_lateral = !matches!(self, Coupling::LocCaptured | Coupling::Coupled);
        out.own_vertical = !matches!(self, Coupling::Coupled);
        out
    }
}

impl Coupling {
    /// Given up from outside the step: a landing the autopilot was to fly
    /// that did not confirm by its gate (`ap_lands::Fallback::OwnLaw`). The
    /// lateral and vertical are handed back as any abandon hands them, APP
    /// pressed once more when anything still reads in. `None` when it was
    /// not engaged (nothing to hand back).
    pub fn abandon_now(&mut self, modes: Option<crate::autoflight::Modes>) -> Option<CouplingOut> {
        if !matches!(self, Coupling::Armed | Coupling::LocCaptured | Coupling::Coupled) {
            return None;
        }
        *self = Coupling::Abandoned;
        Some(CouplingOut {
            press_appr: false,
            decouple: true,
            disarm: modes.is_some_and(|m| m.approach_any()),
            own_lateral: true,
            own_vertical: true,
        })
    }
}

/// A mode as the old status datarefs encoded it: 2 captured, 1 armed, 0 off.
fn status_of(captured: bool, armed: bool) -> f64 {
    if captured {
        2.0
    } else if armed {
        1.0
    } else {
        0.0
    }
}

/// The course source to NAV1 before the approach mode is armed: the mode
/// follows the HSI, and a panel left on GPS tracks the GPS whatever NAV1
/// says. The G1000 172 armed APR with the source on GPS, sat on the
/// localiser at -0.10 dots and never captured it (LEMG 13, 2026-10-11).
/// `None` when it already reads NAV1 or does not read (nothing pressed);
/// else whether the press read back.
pub async fn course_source_to_nav1(xp: &dyn crate::xplane::XPlane) -> Option<bool> {
    let src = xp.get_float(HSI_SOURCE).await.ok()?;
    if src.round() as i64 == 0 {
        return None;
    }
    let _ = autoflight::press(xp, Button::HsiNav1).await;
    Some(xp.get_float(HSI_SOURCE).await.is_ok_and(|v| v.round() as i64 == 0))
}

/// Whether the live airframe may fly an ILS coupled: as its profile says,
/// else wherever its surveyed autopilot has an approach mode (2026-10-13:
/// the final is flown in the modes built for it, down to minimums; our own
/// heading and path, `FinalHeading`, only where the airframe cannot).
pub fn coupled_allowed() -> bool {
    let declared = crate::aircraft::live_profile().and_then(|p| p.ils_coupled);
    declared.unwrap_or_else(|| crate::systems::current().ap_type.is_some_and(crate::systems::ApType::has_approach))
}

// ---------------------------------------------------------------------------
// The pilot-flying ILS setup: what a pilot monitoring does after "set up the
// ILS" (2026-09-25). The tool ([`crate::tools::approach::PrepareIls`]) tunes,
// sets the course source and the minimums; the watcher below arms APPR when
// the geometry allows it (never blind: APPR is a toggle), reads it back, and
// calls the needles alive and the modes captured, setting the missed approach
// altitude at glideslope capture.
// ---------------------------------------------------------------------------

pub const SERVOS: &str = autoflight::names::SERVOS_ON;
pub const AP_POWER: &str = autoflight::names::AP_POWER;
pub const HSI_SOURCE: &str = "sim/cockpit2/radios/actuators/HSI_source_select_pilot";
pub const RA_DH: &str = "sim/cockpit2/gauges/actuators/radio_altimeter_bug_ft_pilot";

/// A localizer's coverage: 35 degrees either side to 17 nm, 10 degrees to
/// 25 nm (ICAO Annex 10). Outside it the needle means nothing.
const LOC_WIDE_DEG: f64 = 35.0;
const LOC_WIDE_NM: f64 = 17.0;
const LOC_NARROW_DEG: f64 = 10.0;
const LOC_NARROW_NM: f64 = 25.0;
/// The widest intercept the approach mode is asked to capture.
pub const MAX_INTERCEPT_DEG: f64 = 90.0;
/// A needle this far in is "alive" (full scale is 2.5 dots).
const ALIVE_DOTS: f64 = 2.4;
/// The whole watch, however slow the arrival.
pub const WATCH_MAX_S: f64 = 25.0 * 60.0;

/// May the approach mode be armed from here?
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArmVerdict {
    Now,
    /// Not yet; the reason, said once.
    Wait(&'static str),
    /// Not from this side of the runway at all.
    Never(&'static str),
}

/// Where the aircraft is against the localizer: inside its coverage on the
/// approach side, heading within [`MAX_INTERCEPT_DEG`] of the course. Pure.
pub fn arm_verdict(lat: f64, lon: f64, heading_mag: f64, ils: &crate::nav::Ils) -> ArmVerdict {
    let d = crate::nav::haversine_nm(ils.lat, ils.lon, lat, lon);
    // The approach side lies along the reciprocal of the course from the
    // antenna (which stands past the far end of the runway).
    let from_loc = crate::nav::bearing_deg(ils.lat, ils.lon, lat, lon);
    let off = crate::angles::wrap180(from_loc - (ils.course_true + 180.0)).abs();
    if off > 90.0 && d > 1.0 {
        return ArmVerdict::Never("we're past the localiser, on the wrong side of the runway");
    }
    let covered = (off <= LOC_WIDE_DEG && d <= LOC_WIDE_NM) || (off <= LOC_NARROW_DEG && d <= LOC_NARROW_NM);
    if !covered {
        return ArmVerdict::Wait("we're outside the localiser's coverage");
    }
    if crate::angles::wrap180(heading_mag - ils.course_mag).abs() > MAX_INTERCEPT_DEG {
        return ArmVerdict::Wait("we're not heading for the localiser");
    }
    ArmVerdict::Now
}

/// One tick of what the watcher reads.
#[derive(Debug, Clone, Copy)]
pub struct SetupIn {
    pub needles: Needles,
    pub appr: Option<f64>,
    pub nav: Option<f64>,
    pub gs: Option<f64>,
    pub servos: bool,
    /// An automation is flying ([`crate::flight_state::crew_flying`]): the
    /// approach is its to fly, and [`Coupling`] owns APPR.
    pub automation: bool,
    pub verdict: ArmVerdict,
    pub agl_ft: f64,
}

#[derive(Debug, Default, PartialEq)]
pub struct SetupOut {
    pub say: Option<String>,
    pub press_appr: bool,
    pub set_dial: Option<f64>,
    pub done: bool,
}

/// The pilot monitoring's ILS watch. Pure.
#[derive(Debug, Default)]
pub struct SetupWatch {
    /// The approach mode is to be armed when the geometry allows.
    pub arm_wanted: bool,
    /// Seen armed (or captured) at least once.
    pub armed_seen: bool,
    pub missed_alt_ft: Option<f64>,
    /// The autopilot flies the glideslope (`None` = unknown).
    pub flies_gs: Option<bool>,
    t: f64,
    pressed_at: Option<f64>,
    loc_alive: bool,
    loc_cap: bool,
    gs_alive: bool,
    gs_cap: bool,
}

impl SetupWatch {
    pub fn new(arm_wanted: bool, armed_now: bool, missed_alt_ft: Option<f64>, flies_gs: Option<bool>) -> Self {
        Self { arm_wanted: arm_wanted && !armed_now, armed_seen: armed_now, missed_alt_ft, flies_gs, ..Self::default() }
    }

    pub fn step(&mut self, i: SetupIn, dt: f64) -> SetupOut {
        self.t += dt;
        let mut out = SetupOut::default();
        if i.automation || i.agl_ft < 300.0 || self.t > WATCH_MAX_S {
            out.done = true;
            return out;
        }
        let mut lines: Vec<String> = Vec::new();
        let appr_on = i.appr.is_some_and(|a| a >= 0.5);
        let n = i.needles;

        // Arming: only ever from OFF (APPR is a toggle), only with the
        // servos in, only inside the coverage, and read back.
        if self.arm_wanted {
            if appr_on {
                self.arm_wanted = false;
                self.armed_seen = true;
                if self.pressed_at.is_some() {
                    lines.push("Approach armed.".to_string());
                }
            } else if let Some(at) = self.pressed_at {
                if self.t - at > ARM_TIMEOUT_S {
                    self.arm_wanted = false;
                    lines.push("Approach mode didn't arm. It's on raw data, the needles are yours.".to_string());
                }
            } else if !i.servos {
                self.arm_wanted = false;
                lines.push("Autopilot's off, so the approach is yours. I'll call the needles.".to_string());
            } else {
                match i.verdict {
                    ArmVerdict::Now => {
                        out.press_appr = true;
                        self.pressed_at = Some(self.t);
                    }
                    ArmVerdict::Wait(_) => {}
                    ArmVerdict::Never(why) => {
                        self.arm_wanted = false;
                        lines.push(format!("I won't arm the approach, {why}."));
                    }
                }
            }
        } else if self.armed_seen && !appr_on && !self.gs_cap && i.appr.is_some() {
            // Disarmed after arming, not by a capture: the pilot's choice
            // (or the autopilot let go). Never re-armed.
            self.armed_seen = false;
        }

        // The needles, as a pilot monitoring calls them.
        if !self.loc_alive && n.loc_valid && n.hdef.is_some_and(|h| h.abs() < ALIVE_DOTS) {
            self.loc_alive = true;
            lines.push("Localiser alive.".to_string());
        }
        if !self.loc_cap && i.servos && (i.nav.is_some_and(|v| v >= 1.5) || i.appr.is_some_and(|v| v >= 1.5)) {
            self.loc_cap = true;
            self.loc_alive = true;
            lines.push("Localiser captured.".to_string());
        }
        if self.loc_alive && !self.gs_alive && n.gs_valid && n.vdef.is_some_and(|v| v.abs() < ALIVE_DOTS) {
            self.gs_alive = true;
            lines.push("Glideslope alive.".to_string());
        }
        if !self.gs_cap && i.servos && i.gs.is_some_and(|v| v >= 1.5) {
            self.gs_cap = true;
            self.gs_alive = true;
            match self.missed_alt_ft {
                Some(m) => {
                    out.set_dial = Some(m);
                    lines.push(format!(
                        "Glideslope captured. Missed approach altitude {} set.",
                        crate::answers::with_commas(m)
                    ));
                }
                None => lines.push("Glideslope captured.".to_string()),
            }
        }

        // Done once there is nothing left to call: the glideslope captured,
        // or, hand flown (or on an autopilot that cannot fly it), both
        // needles alive.
        let coupled_vertical = i.servos && self.flies_gs != Some(false) && (self.armed_seen || self.arm_wanted);
        out.done = self.gs_cap || (self.loc_alive && self.gs_alive && !coupled_vertical);
        if !lines.is_empty() {
            out.say = Some(lines.join(" "));
        }
        out
    }
}

/// The running setup watch: a new "set up the ILS" takes it over, a stand
/// down (`flight_state::LOOPS`: swap, instructor entry) cancels it.
pub(crate) static SETUP: sim::run::Slot = sim::run::Slot::new();

/// Run a [`SetupWatch`] on its own thread and runtime (the command loop's
/// runtime is blocked on the listener between commands, as for the altitude
/// capture watcher). Its writes are the APPR press, only from off, and the
/// missed approach altitude at capture.
pub fn spawn_setup_watch(base: String, ils: crate::nav::Ils, mut watch: SetupWatch, voice: Option<crate::voice::Voice>) {
    // Taken on the caller's thread: a stand-down landing before the thread
    // starts cancels THIS watch, not a predecessor.
    let lease = SETUP.take_over();
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            return;
        };
        rt.block_on(async move {
            // Writes scoped to this run: once superseded or stood down, a
            // write still in flight is refused.
            let xp = crate::xplane::XPlaneClient::new(base).with_cancel(lease.cancel.clone());
            let dt = 0.5;
            loop {
                if !lease.live() {
                    return;
                }
                if crate::xplane::sim_frozen(&xp).await {
                    tokio::time::sleep(std::time::Duration::from_secs_f64(dt)).await;
                    continue;
                }
                // The modes from the bitfield (`autoflight::Modes`): the
                // 737's approach_status read 0 through captures (#81).
                let (hdef, vdef, locv, gsv, ap_state, servos, lat, lon, hdg, agl) = tokio::join!(
                    xp.get_float(HDEF),
                    xp.get_float(VDEF),
                    xp.get_float(LOC_VALID),
                    xp.get_float(GS_VALID),
                    crate::autoflight::read_ap_state(&xp),
                    xp.get_float(SERVOS),
                    xp.get_float(crate::datarefs::LATITUDE),
                    xp.get_float(crate::datarefs::LONGITUDE),
                    xp.get_float(crate::datarefs::COMPASS_HEADING_DEG),
                    xp.get_float(crate::datarefs::AGL_METERS),
                );
                let modes = ap_state.ok().and_then(autoflight::Modes::from_read);
                // A failed position read is no measurement: skip the tick.
                let (Ok(lat), Ok(lon), Ok(hdg), Ok(agl)) = (lat, lon, hdg, agl) else {
                    tokio::time::sleep(std::time::Duration::from_secs_f64(dt)).await;
                    continue;
                };
                let i = SetupIn {
                    needles: Needles {
                        hdef: hdef.ok(),
                        vdef: vdef.ok(),
                        loc_valid: locv.is_ok_and(|v| v > 0.5),
                        gs_valid: gsv.is_ok_and(|v| v > 0.5),
                    },
                    appr: modes.map(|m| status_of(m.loc() || m.gs(), m.approach_any())),
                    nav: modes.map(|m| status_of(m.loc(), m.loc_armed())),
                    gs: modes.map(|m| status_of(m.gs(), m.gs_armed())),
                    servos: servos.is_ok_and(|v| v > 0.5),
                    automation: crate::flight_state::crew_flying().is_some(),
                    verdict: arm_verdict(lat, lon, hdg, &ils),
                    agl_ft: agl * 3.28084,
                };
                let out = watch.step(i, dt);
                if out.press_appr {
                    autoflight::press_logged(&xp, Button::Approach, "ils_setup").await;
                    crate::blackbox::record("ils", json!({ "event": "appr_pressed", "rwy": ils.runway, "icao": ils.airport }));
                }
                if let Some(m) = out.set_dial {
                    let _ = autoflight::set_dial(&xp, Dial::Altitude, m).await;
                    crate::blackbox::record("ils", json!({ "event": "missed_alt_set", "alt": m }));
                }
                if let Some(line) = out.say {
                    crate::blackbox::record("ils", json!({ "event": "watch", "said": line }));
                    if let Some(v) = voice.as_ref() {
                        v.alert(crate::voice::Priority::Advisory, line);
                    }
                }
                if out.done {
                    crate::blackbox::record("ils", json!({ "event": "watch_done", "rwy": ils.runway }));
                    lease.record(sim::run::Outcome::Completed);
                    return;
                }
                tokio::time::sleep(std::time::Duration::from_secs_f64(dt)).await;
            }
        });
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    /// #156, EGBB 15 (2026-09-28, sortie 1790601822-full_auto): the probe aimed
    /// the path at 297 ft (header 327) under a glideslope whose antenna
    /// earth_nav.dat puts at 348 ft, 0.34 nm past the threshold: the needle
    /// read 1.2 dots low at 497 ft. With the ILS tuned the aim moves up
    /// toward the beam as far as a threshold crossing of 80 ft over the
    /// pavement: 327.
    #[test]
    fn the_path_lies_toward_the_glideslope_when_the_ils_is_tuned() {
        let rows = " 4  52.442050000   -1.734913889      348    11010    18  52345.887 IBIR EGBB EG 15 ILS-cat-III\n \
                     6  52.460202778   -1.757769444      348    11010    18 300145.887 IBIR EGBB EG 15 GS\n \
                     6  52.447877778   -1.743875000      364    11010    18 300325.887  IBM EGBB EG 33 GS\n";
        let gs = parse_glideslope(rows.lines(), "EGBB", "15").expect("the 15 glideslope");
        assert_eq!((gs.elev_ft, gs.angle_deg), (348.0, 3.0));
        assert_eq!(parse_glideslope(rows.lines(), "EGBB", "33").unwrap().elev_ft, 364.0);
        assert_eq!(parse_glideslope(rows.lines(), "EGBB", "06"), None);
        // apt.dat's 15 and 33 thresholds: the course true.
        let (t15, t33) = ((52.4656497, -1.7611282), (52.4428974, -1.7358498));
        let course = crate::nav::bearing_deg(t15.0, t15.1, t33.0, t33.1);
        let aim_nm = crate::autoland::TOUCHDOWN_AIM_NM;
        let on_beam = glideslope_aim_ft(&gs, t15.0, t15.1, course, aim_nm, 3.0).expect("same angle");
        assert!((390.0..420.0).contains(&on_beam), "{on_beam}");
        // Capped by the threshold crossing: 297 + 30 = 327 (the header).
        let aim = crate::autoland::aim_with_glideslope(297.0, Some(on_beam));
        assert!((aim - 327.0).abs() < 0.01, "{aim}");
        let tch = aim + aim_nm * 6076.12 * 3f64.to_radians().tan() - 297.0;
        assert!((75.0..85.0).contains(&tch), "{tch}");
        // Nearer the beam by those 30 ft: at 1.4 nm from the threshold.
        let beam_at = |x: f64| on_beam + (x + aim_nm) * 318.4;
        let path_at = |a: f64, x: f64| a + (x + aim_nm) * 318.4;
        assert!((beam_at(1.4) - path_at(aim, 1.4)) < (beam_at(1.4) - path_at(297.0, 1.4)) - 29.0);
        // No glideslope, or a path at another angle: the pavement's aim.
        assert_eq!(crate::autoland::aim_with_glideslope(297.0, None), 297.0);
        assert_eq!(glideslope_aim_ft(&gs, t15.0, t15.1, course, aim_nm, 4.5), None);
        // A beam under the pavement's aim moves it at most 15 ft down.
        assert_eq!(crate::autoland::aim_with_glideslope(300.0, Some(250.0)), 285.0);
    }

    /// EGPH 24's real localiser (Custom Data): antenna past the 06 end,
    /// true 238.824, magnetic 239.
    fn egph24() -> crate::nav::Ils {
        crate::nav::Ils {
            freq_10khz: 10890,
            course_mag: 239.0,
            course_true: 238.824,
            lat: 55.941450,
            lon: -3.397650,
            airport: "EGPH".into(),
            runway: "24".into(),
            distance_nm: 0.0,
        }
    }

    /// A point `nm` from the antenna on true bearing `brg`.
    fn from_loc(nm: f64, brg: f64) -> (f64, f64) {
        let i = egph24();
        let r = brg.to_radians();
        (i.lat + nm * r.cos() / 60.0, i.lon + nm * r.sin() / (60.0 * i.lat.to_radians().cos()))
    }

    #[test]
    fn the_approach_is_armed_only_inside_the_coverage_on_an_intercept() {
        let ils = egph24();
        let back = 238.824 - 180.0;
        let (la, lo) = from_loc(12.0, back);
        assert_eq!(arm_verdict(la, lo, 239.0, &ils), ArmVerdict::Now, "on the centreline, inbound");
        assert_eq!(arm_verdict(la, lo, 270.0, &ils), ArmVerdict::Now, "a 31 degree intercept");
        assert!(matches!(arm_verdict(la, lo, 59.0, &ils), ArmVerdict::Wait(_)), "flying away");
        let (la, lo) = from_loc(22.0, back);
        assert_eq!(arm_verdict(la, lo, 239.0, &ils), ArmVerdict::Now, "the narrow sector reaches 25 nm");
        let (la, lo) = from_loc(22.0, back + 25.0);
        assert!(matches!(arm_verdict(la, lo, 239.0, &ils), ArmVerdict::Wait(w) if w.contains("coverage")));
        let (la, lo) = from_loc(6.0, 238.824);
        assert!(matches!(arm_verdict(la, lo, 239.0, &ils), ArmVerdict::Never(_)), "west of the field, past the antenna");
    }

    fn tick(appr: f64, nav: f64, gs: f64, n: Needles, servos: bool, verdict: ArmVerdict) -> SetupIn {
        SetupIn { needles: n, appr: Some(appr), nav: Some(nav), gs: Some(gs), servos, automation: false, verdict, agl_ft: 3000.0 }
    }

    fn dead() -> Needles {
        Needles { hdef: Some(2.5), vdef: Some(2.5), loc_valid: false, gs_valid: false }
    }

    #[test]
    fn the_approach_mode_is_pressed_once_from_off_and_read_back() {
        let mut w = SetupWatch::new(true, false, Some(3000.0), Some(true));
        let o = w.step(tick(0.0, 0.0, 0.0, dead(), true, ArmVerdict::Wait("x")), 0.5);
        assert!(!o.press_appr, "not before the intercept");
        let o = w.step(tick(0.0, 0.0, 0.0, dead(), true, ArmVerdict::Now), 0.5);
        assert!(o.press_appr);
        let o = w.step(tick(0.0, 0.0, 0.0, dead(), true, ArmVerdict::Now), 0.5);
        assert!(!o.press_appr, "APPR is a toggle: never pressed twice");
        let o = w.step(tick(1.0, 1.0, 1.0, dead(), true, ArmVerdict::Now), 0.5);
        assert_eq!(o.say.as_deref(), Some("Approach armed."));
        // The pilot presses it off: never re-armed.
        for _ in 0..20 {
            assert!(!w.step(tick(0.0, 0.0, 0.0, dead(), true, ArmVerdict::Now), 0.5).press_appr);
        }
    }

    #[test]
    fn an_approach_mode_that_does_not_arm_is_said_and_not_retried() {
        let mut w = SetupWatch::new(true, false, None, None);
        assert!(w.step(tick(0.0, 0.0, 0.0, dead(), true, ArmVerdict::Now), 0.5).press_appr);
        let mut said = Vec::new();
        for _ in 0..20 {
            let o = w.step(tick(0.0, 0.0, 0.0, dead(), true, ArmVerdict::Now), 0.5);
            assert!(!o.press_appr);
            said.extend(o.say);
        }
        assert_eq!(said.len(), 1, "{said:?}");
        assert!(said[0].contains("didn't arm"));
    }

    #[test]
    fn the_needles_and_captures_are_called_and_the_missed_altitude_set() {
        let mut w = SetupWatch::new(false, true, Some(3000.0), Some(true));
        let loc = Needles { hdef: Some(2.0), vdef: Some(2.5), loc_valid: true, gs_valid: false };
        assert_eq!(w.step(tick(1.0, 1.0, 1.0, loc, true, ArmVerdict::Now), 0.5).say.as_deref(), Some("Localiser alive."));
        assert_eq!(w.step(tick(2.0, 2.0, 1.0, loc, true, ArmVerdict::Now), 0.5).say.as_deref(), Some("Localiser captured."));
        let gs = Needles { hdef: Some(0.0), vdef: Some(2.0), loc_valid: true, gs_valid: true };
        let o = w.step(tick(2.0, 2.0, 1.0, gs, true, ArmVerdict::Now), 0.5);
        assert_eq!(o.say.as_deref(), Some("Glideslope alive."));
        assert!(!o.done);
        let o = w.step(tick(2.0, 2.0, 2.0, gs, true, ArmVerdict::Now), 0.5);
        assert_eq!(o.set_dial, Some(3000.0));
        assert_eq!(o.say.as_deref(), Some("Glideslope captured. Missed approach altitude 3,000 set."));
        assert!(o.done);
    }

    #[test]
    fn hand_flown_the_watch_calls_the_needles_and_never_touches_the_autopilot() {
        let mut w = SetupWatch::new(false, false, Some(3000.0), None);
        let both = Needles { hdef: Some(1.0), vdef: Some(1.0), loc_valid: true, gs_valid: true };
        let o = w.step(tick(0.0, 0.0, 0.0, both, false, ArmVerdict::Now), 0.5);
        assert_eq!(o.say.as_deref(), Some("Localiser alive. Glideslope alive."));
        assert!(!o.press_appr && o.set_dial.is_none());
        assert!(o.done);
    }

    #[test]
    fn an_automation_taking_the_approach_ends_the_watch_silently() {
        let mut w = SetupWatch::new(true, false, None, None);
        let mut i = tick(0.0, 0.0, 0.0, dead(), true, ArmVerdict::Now);
        i.automation = true;
        let o = w.step(i, 0.5);
        assert!(o.done && !o.press_appr && o.say.is_none());
    }

    #[test]
    fn a_pilot_who_disconnects_before_the_intercept_is_told_the_approach_is_theirs() {
        let mut w = SetupWatch::new(true, false, None, None);
        let o = w.step(tick(0.0, 0.0, 0.0, dead(), false, ArmVerdict::Now), 0.5);
        assert!(!o.press_appr);
        assert!(o.say.unwrap().contains("yours"));
    }

    fn needles(h: f64, v: f64) -> Needles {
        Needles { hdef: Some(h), vdef: Some(v), loc_valid: true, gs_valid: true }
    }

    #[test]
    fn a_standing_needle_is_said_once_and_only_on_final() {
        let mut x = XCheck::default();
        for _ in 0..40 {
            assert!(x.step(needles(1.6, 0.0), false, 1500.0, 0.5).is_none(), "intercepting");
        }
        let mut said = Vec::new();
        for _ in 0..40 {
            said.extend(x.step(needles(1.6, 0.0), true, 1500.0, 0.5));
        }
        assert_eq!(said.len(), 1, "{said:?}");
        assert!(said[0].contains("1.6 dots right"), "{}", said[0]);
        // Back on it, then out again: a new episode.
        for _ in 0..4 {
            x.step(needles(0.1, 0.0), true, 1500.0, 0.5);
        }
        let again: Vec<String> = (0..40).filter_map(|_| x.step(needles(-1.5, 0.0), true, 1500.0, 0.5)).collect();
        assert_eq!(again.len(), 1);
        assert!(again[0].contains("left"));
        // High above the band: nothing.
        let mut hi = XCheck::default();
        assert!((0..40).all(|_| hi.step(needles(2.0, 2.0), true, 5000.0, 0.5).is_none()));
    }

    /// #175, EGBB 15 (sortie 1790624015-full_auto, 2026-09-28). At +2683.8 the
    /// needle read +1.37 at 1,859 ft radio with the aircraft 221 ft above the
    /// beam by earth_nav.dat's antenna (autoland 279 ft over its own path) and
    /// was called "below"; at +2795.1 it read -1.52 at 142 ft on autoland's path
    /// (67 ft under the beam) and was called "above". Other sorties agree:
    /// LEMG 12 +1.49 at 191 ft above, LEMG 13 -2.50 at 213 ft below.
    #[test]
    fn a_positive_glideslope_needle_is_above_the_beam() {
        let said = |v: f64, radio: f64| -> Vec<String> {
            let mut x = XCheck::default();
            (0..40).filter_map(|_| x.step(needles(-0.32, v), true, radio, 0.3)).collect()
        };
        let high = said(1.3744881, 1859.0);
        assert_eq!(high.len(), 1, "{high:?}");
        assert!(high[0].contains("1.4 dots high, we're above"), "{}", high[0]);
        let lemg12 = said(1.4870062, 1819.0);
        assert!(lemg12[0].contains("above"), "{}", lemg12[0]);
        let low = said(-2.4999995, 518.0);
        assert!(low[0].contains("2.5 dots low, we're below"), "{}", low[0]);
        // The flare's needle: under the floor, nothing.
        assert!(said(-1.519844, 142.0).is_empty());
        assert!(said(-1.5, XCHECK_GS_FLOOR_AGL_FT - 1.0).is_empty());
        // The localizer is still watched there.
        let mut x = XCheck::default();
        let loc: Vec<String> =
            (0..40).filter_map(|_| x.step(needles(1.6, -1.5), true, 142.0, 0.3)).collect();
        assert_eq!(loc.len(), 1);
        assert!(loc[0].starts_with("Localizer"), "{}", loc[0]);
    }

    /// Below the airframe's declared height the coupling hands the final
    /// back (decouple, APP pressed once more to clear it); above it, and
    /// with no height declared, it stays coupled (LEMD 18L 2026-10-05).
    #[test]
    fn a_coupled_final_is_handed_back_at_the_declared_height() {
        let mut t = 0.0;
        let coupled = || {
            let mut c = Coupling::Coupled;
            let _ = &mut c;
            c
        };
        let mut c = coupled();
        let o = c.step(CouplingIn { radio_ft: 650.0, decouple_below_ft: Some(500.0), ..inp(2560) }, &mut t, 0.3);
        assert!(!o.decouple && c == Coupling::Coupled);
        let o = c.step(CouplingIn { radio_ft: 480.0, decouple_below_ft: Some(500.0), ..inp(2560) }, &mut t, 0.3);
        assert!(o.decouple && o.disarm && c == Coupling::Abandoned);
        assert!(o.own_vertical && o.own_lateral);
        let mut d = coupled();
        let o = d.step(CouplingIn { radio_ft: 80.0, decouple_below_ft: None, ..inp(2560) }, &mut t, 0.3);
        assert!(!o.decouple, "no height declared: coupled to the arrest");
        let mut e = coupled();
        let o = e.step(CouplingIn { radio_ft: 0.0, decouple_below_ft: Some(500.0), ..inp(2560) }, &mut t, 0.3);
        assert!(!o.decouple, "an unread radio height decides nothing");
    }

    fn inp(modes: u32) -> CouplingIn {
        CouplingIn {
            allowed: true,
            inbound: true,
            dist_nm: 9.0,
            needles: needles(0.3, -0.5),
            modes: Some(crate::autoflight::Modes(modes)),
            ..CouplingIn::default()
        }
    }

    /// The 737's own sequence on 2026-10-05 (13:22:39 to 13:25:45): 50
    /// (HDG, VS, ALT arm) when pressed, 1330 armed, 1586 the localizer
    /// captured with the glideslope armed, 2560 both captured.
    #[test]
    fn a_coupled_ils_hands_over_lateral_then_vertical() {
        let (mut c, mut t) = (Coupling::Off, 0.0);
        let o = c.step(inp(50), &mut t, 0.3);
        assert!(o.press_appr && o.own_lateral && o.own_vertical);
        let o = c.step(inp(1330), &mut t, 0.3);
        assert!(!o.press_appr && o.own_lateral, "armed: still ours");
        let o = c.step(inp(1586), &mut t, 0.3);
        assert_eq!(c, Coupling::LocCaptured);
        assert!(!o.own_lateral && o.own_vertical, "the localizer is the AP's, the vertical ours");
        let o = c.step(inp(2560), &mut t, 0.3);
        assert_eq!(c, Coupling::Coupled);
        assert!(!o.own_lateral && !o.own_vertical);
        // The roundout gate: the arrest takes the servos; no decouple, even
        // as the modes go with them.
        let o = c.step(CouplingIn { arresting: true, leaving: true, ..inp(132) }, &mut t, 0.3);
        assert!(!o.decouple);
    }

    /// Found armed (the pilot pressed it, or a capture already in): never
    /// pressed again.
    #[test]
    fn an_approach_already_armed_is_never_pressed() {
        let (mut c, mut t) = (Coupling::Off, 0.0);
        let o = c.step(inp(1330), &mut t, 0.3);
        assert!(!o.press_appr && c == Coupling::Armed);
        let o = c.step(inp(2560), &mut t, 0.3);
        assert_eq!(c, Coupling::Coupled, "captured both at once: {o:?}");
    }

    #[test]
    fn a_go_around_or_a_lost_capture_decouples_disarms_and_never_rearms() {
        let (mut c, mut t) = (Coupling::Coupled, 0.0);
        let o = c.step(CouplingIn { leaving: true, ..inp(2560) }, &mut t, 0.3);
        assert!(o.decouple && o.disarm && o.own_lateral && o.own_vertical);
        assert_eq!(c, Coupling::Abandoned);
        assert!(!c.step(inp(50), &mut t, 0.3).press_appr, "abandoned stays abandoned");
        let mut c = Coupling::Coupled;
        let o = c.step(inp(512), &mut t, 0.3);
        assert!(o.decouple && o.disarm, "glideslope lost: {o:?}");
        // APP that never arms is abandoned without a fight, nothing left to
        // disarm.
        let (mut c, mut t) = (Coupling::Off, 0.0);
        c.step(inp(50), &mut t, 0.3);
        let mut last = None;
        for _ in 0..30 {
            last = Some(c.step(inp(50), &mut t, 0.3));
        }
        assert_eq!(c, Coupling::Abandoned);
        assert!(!last.unwrap().press_appr);
    }

    #[test]
    fn nothing_couples_unless_allowed_inbound_below_the_beam_and_read() {
        let (mut c, mut t) = (Coupling::Off, 0.0);
        let o = c.step(CouplingIn { allowed: false, ..inp(50) }, &mut t, 0.3);
        assert!(!o.press_appr && c == Coupling::Off);
        let o = c.step(CouplingIn { inbound: false, ..inp(50) }, &mut t, 0.3);
        assert!(!o.press_appr, "not on an intercept");
        let o = c.step(CouplingIn { dist_nm: 20.0, ..inp(50) }, &mut t, 0.3);
        assert!(!o.press_appr, "too far");
        let o = c.step(CouplingIn { needles: needles(0.3, 1.2), ..inp(50) }, &mut t, 0.3);
        assert!(!o.press_appr, "above the glideslope: the capture is from below");
        let o = c.step(CouplingIn { modes: None, ..inp(50) }, &mut t, 0.3);
        assert!(!o.press_appr && c == Coupling::Off, "an unread bitfield is no measurement");
        // A localizer far off is still armed on an intercept (the 10-05
        // press was two minutes before the capture).
        let o = c.step(CouplingIn { needles: needles(2.5, -2.5), ..inp(50) }, &mut t, 0.3);
        assert!(o.press_appr);
    }

    /// The 737's own APP on 2026-10-05 (13:22:39 to 13:29:49, the fixture's
    /// `pilot_app` half), replayed through the coupling and the owner: found
    /// armed and never pressed again; the localizer handed over at 13:24:37,
    /// the glideslope at 13:25:45; nothing pressed over either to the
    /// roundout; the landing's servos-out is the arrest's, no decouple.
    #[test]
    fn the_737s_own_app_of_2026_10_05_couples_and_is_left_alone() {
        use crate::autoflight::{reconcile, Lateral, ModeRead, Modes, Vertical, Want};
        let src = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/approach/lemg13_737_2026-10-05.jsonl"));
        let ticks: Vec<serde_json::Value> = src
            .lines()
            .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
            .filter(|r| r["ph"] == "pilot_app" && r["t"].as_f64().unwrap() >= 1_791_199_359.0)
            .collect();
        let (mut c, mut t) = (Coupling::Off, 0.0);
        let mut loc_at = None;
        let mut gs_at = None;
        for r in &ticks {
            let m = Modes(r["ap"].as_u64().unwrap() as u32);
            let radio = r["radio"].as_f64().unwrap();
            let o = c.step(
                CouplingIn {
                    allowed: true,
                    inbound: true,
                    arresting: radio < 60.0,
                    dist_nm: 9.0,
                    needles: Needles { hdef: Some(0.0), vdef: None, loc_valid: true, gs_valid: false },
                    modes: Some(m),
                    ..CouplingIn::default()
                },
                &mut t,
                2.0,
            );
            assert!(!o.press_appr, "found armed: never pressed ({:?})", m.words());
            assert!(!o.decouple, "decoupled at {} on {:?}", r["t"], m.words());
            if !o.own_lateral && loc_at.is_none() {
                loc_at = r["t"].as_f64();
            }
            if !o.own_vertical && gs_at.is_none() {
                gs_at = r["t"].as_f64();
            }
            // The owner's housekeeping under the coupled intent, from the
            // statuses those bits show: nothing pressed while either half
            // of the approach is in.
            let read = ModeRead {
                hdg_status: Some(if m.heading() { 2.0 } else { 0.0 }),
                vs_status: Some(if m.vs() { 2.0 } else { 0.0 }),
                modes: Some(m),
            };
            let want = Want { lateral: Some(Lateral::Coupled), vertical: Some(Vertical::Coupled) };
            if m.approach_any() {
                let p = reconcile(want, read);
                assert!(
                    p.iter().all(|b| *b == crate::autoflight::Button::VerticalSpeed) && !(m.gs() && !p.is_empty()),
                    "{p:?} over {:?}",
                    m.words()
                );
            }
        }
        assert_eq!(c, Coupling::Coupled);
        let (loc, gs) = (loc_at.unwrap(), gs_at.unwrap());
        assert!((loc - 1_791_199_477.0).abs() < 3.0, "localizer at {loc}");
        assert!((gs - 1_791_199_545.0).abs() < 3.0, "glideslope at {gs}");
    }
}
