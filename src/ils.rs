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
//!   is armed on final, and once the localizer (then the glideslope) is
//!   CAPTURED the copilot stops writing the heading bug (then the VS dial)
//!   and stops re-engaging HDG/VS (re-engaging VS mode drops a captured
//!   glideslope). The roundout gate's arrest takes the servos as always; a
//!   go-around or any leaving of the final decouples back to HDG/VS. Off by
//!   default: each autopilot captures differently, and a profile turns it on
//!   only after a flown capture.

use serde_json::json;

pub const NAV1_FREQ: &str = "sim/cockpit2/radios/actuators/nav1_frequency_hz";
pub const NAV1_OBS: &str = "sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot";
pub const HDEF: &str = "sim/cockpit2/radios/indicators/nav1_hdef_dots_pilot";
pub const VDEF: &str = "sim/cockpit2/radios/indicators/nav1_vdef_dots_pilot";
pub const LOC_VALID: &str = "sim/cockpit2/radios/indicators/nav1_display_horizontal";
pub const GS_VALID: &str = "sim/cockpit2/radios/indicators/nav1_display_vertical";
pub const APPR_STATUS: &str = "sim/cockpit2/autopilot/approach_status";
pub const GS_STATUS: &str = "sim/cockpit2/autopilot/glideslope_status";
pub const CMD_APPROACH: &str = "sim/autopilot/approach";

/// How far an ILS may be from the runway's field and still be its own.
const FIND_MAX_NM: f64 = 40.0;

/// Tune NAV1 to `icao` runway `rwy`'s ILS and set its course, reading both
/// back. `None` when the runway has no ILS in the data (a visual or RNAV
/// runway: nothing to tune, nothing said). Idempotent: an already-tuned
/// radio is left alone.
pub async fn tune(
    xp: &crate::xplane::XPlaneClient,
    icao: &str,
    rwy: &str,
    lat: f64,
    lon: f64,
) -> Option<crate::nav::Ils> {
    let nav_path = crate::settings::live().nav_data_path.clone();
    let (want_rwy, want_icao) = (rwy.to_string(), icao.to_string());
    let ils = tokio::task::spawn_blocking(move || {
        crate::nav::find_ils(&nav_path, &want_rwy, lat, lon, FIND_MAX_NM).ok().flatten()
    })
    .await
    .ok()
    .flatten()
    .filter(|i| i.airport.eq_ignore_ascii_case(&want_icao) && i.runway.eq_ignore_ascii_case(rwy))?;
    let (freq, obs) = tokio::join!(xp.get_float(NAV1_FREQ), xp.get_float(NAV1_OBS));
    let tuned = freq.is_ok_and(|f| f.round() as i64 == ils.freq_10khz);
    let on_course = obs.is_ok_and(|o| ((o - ils.course_mag + 540.0).rem_euclid(360.0) - 180.0).abs() < 1.0);
    if !tuned {
        let _ = xp.set_int(NAV1_FREQ, ils.freq_10khz).await;
    }
    if !on_course {
        let _ = xp.set_float(NAV1_OBS, ils.course_mag).await;
    }
    let (freq, obs) = tokio::join!(xp.get_float(NAV1_FREQ), xp.get_float(NAV1_OBS));
    let ok = freq.is_ok_and(|f| f.round() as i64 == ils.freq_10khz)
        && obs.is_ok_and(|o| ((o - ils.course_mag + 540.0).rem_euclid(360.0) - 180.0).abs() < 1.0);
    crate::blackbox::record(
        "ils",
        json!({ "event": "tuned", "icao": icao, "rwy": rwy, "freq_mhz": ils.freq_mhz(),
                "course": ils.course_mag, "already": tuned && on_course, "ok": ok }),
    );
    ok.then_some(ils)
}

/// A needle past this many dots...
pub const XCHECK_DOTS: f64 = 1.0;
/// ...held this long...
pub const XCHECK_HOLD_S: f64 = 8.0;
/// ...below this height on final is said.
pub const XCHECK_BELOW_AGL_FT: f64 = 3000.0;

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
        let gs = n.vdef.filter(|_| live && n.gs_valid);
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
                    // The needle shows where the glideslope IS: above us
                    // (positive) means we are low.
                    out = Some(format!(
                        "Glideslope {:.1} dots {}, we're {} the ILS path.",
                        v.abs(),
                        side(v, "up", "down"),
                        side(v, "below", "above")
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
    /// APPR pressed, waiting for the localizer to capture.
    Armed,
    /// The localizer is the autopilot's; the glideslope maybe not yet.
    LocCaptured,
    /// Both captured: the autopilot flies the ILS until the roundout gate.
    Coupled,
    /// Given up for this approach (decoupled, or APPR would not arm).
    Abandoned,
}

/// What the tick does about it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CouplingOut {
    /// Press the approach button this tick.
    pub press_appr: bool,
    /// Put HDG and VS back this tick (decoupling).
    pub decouple: bool,
    /// The copilot still writes the heading bug and keeps HDG engaged.
    pub own_lateral: bool,
    /// The copilot still writes the VS dial and keeps VS engaged.
    pub own_vertical: bool,
}

/// The localizer must be this close to arm (dots)...
pub const ARM_MAX_DOTS: f64 = 2.0;
/// ...and the runway this close (nm).
pub const ARM_MAX_NM: f64 = 15.0;
/// APPR pressed and still off after this long: abandoned.
pub const ARM_TIMEOUT_S: f64 = 6.0;

/// Inputs to one coupling step.
#[derive(Debug, Clone, Copy, Default)]
pub struct CouplingIn {
    pub allowed: bool,
    pub on_final: bool,
    pub arresting: bool,
    pub go_around: bool,
    pub dist_nm: f64,
    pub needles: Needles,
    /// `approach_status`: 0 off, 1 armed, 2 captured.
    pub appr: Option<f64>,
    /// `glideslope_status`: 0 off, 1 armed, 2 captured.
    pub gs: Option<f64>,
}

impl Coupling {
    /// One tick. `armed_s` counts the time in `Armed` (the caller's clock).
    pub fn step(&mut self, i: CouplingIn, armed_s: &mut f64, dt: f64) -> CouplingOut {
        let mut out = CouplingOut { press_appr: false, decouple: false, own_lateral: true, own_vertical: true };
        let engaged = matches!(self, Coupling::Armed | Coupling::LocCaptured | Coupling::Coupled);
        // Leaving the final (a pattern, a go-around) hands HDG/VS back at
        // once. The arrest is not a leaving: it takes the servos itself.
        if engaged && (i.go_around || (!i.on_final && !i.arresting)) {
            *self = Coupling::Abandoned;
            out.decouple = true;
            return out;
        }
        let appr = i.appr.unwrap_or(0.0);
        let gs = i.gs.unwrap_or(0.0);
        match *self {
            Coupling::Off => {
                let h = i.needles.hdef.unwrap_or(f64::MAX).abs();
                if i.allowed
                    && i.on_final
                    && !i.arresting
                    && !i.go_around
                    && i.needles.loc_valid
                    && h < ARM_MAX_DOTS
                    && i.dist_nm < ARM_MAX_NM
                {
                    // A button that toggles: pressed only when it reads off.
                    out.press_appr = appr < 0.5;
                    *self = Coupling::Armed;
                    *armed_s = 0.0;
                }
            }
            Coupling::Armed => {
                *armed_s += dt;
                if appr > 1.5 {
                    *self = Coupling::LocCaptured;
                } else if appr < 0.5 && *armed_s > ARM_TIMEOUT_S {
                    *self = Coupling::Abandoned;
                }
            }
            Coupling::LocCaptured => {
                if appr < 0.5 {
                    *self = Coupling::Abandoned;
                    out.decouple = true;
                    return out;
                }
                if gs > 1.5 {
                    *self = Coupling::Coupled;
                }
            }
            Coupling::Coupled => {
                // The glideslope dropped out: the vertical comes back to us.
                if gs < 1.5 || appr < 0.5 {
                    *self = Coupling::Abandoned;
                    out.decouple = true;
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

/// Whether the live airframe may fly an ILS coupled.
pub fn coupled_allowed() -> bool {
    crate::aircraft::live_profile().is_some_and(|p| p.ils_coupled)
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn a_glideslope_needle_up_means_we_are_low() {
        let mut x = XCheck::default();
        let said: Vec<String> = (0..40).filter_map(|_| x.step(needles(0.0, 1.4), true, 1200.0, 0.5)).collect();
        assert_eq!(said.len(), 1);
        assert!(said[0].contains("below"), "{}", said[0]);
    }

    fn inp(appr: f64, gs: f64) -> CouplingIn {
        CouplingIn {
            allowed: true,
            on_final: true,
            dist_nm: 9.0,
            needles: needles(0.3, 0.5),
            appr: Some(appr),
            gs: Some(gs),
            ..CouplingIn::default()
        }
    }

    #[test]
    fn a_coupled_ils_hands_over_lateral_then_vertical() {
        let (mut c, mut t) = (Coupling::Off, 0.0);
        let o = c.step(inp(0.0, 0.0), &mut t, 0.3);
        assert!(o.press_appr && o.own_lateral && o.own_vertical);
        let o = c.step(inp(1.0, 1.0), &mut t, 0.3);
        assert!(!o.press_appr && o.own_lateral, "armed: still ours");
        let o = c.step(inp(2.0, 1.0), &mut t, 0.3);
        assert_eq!(c, Coupling::LocCaptured);
        assert!(!o.own_lateral && o.own_vertical, "the localizer is the AP's, the vertical ours");
        let o = c.step(inp(2.0, 2.0), &mut t, 0.3);
        assert_eq!(c, Coupling::Coupled);
        assert!(!o.own_lateral && !o.own_vertical);
        // The roundout gate: the arrest takes the servos; no decouple.
        let o = c.step(CouplingIn { on_final: false, arresting: true, ..inp(2.0, 2.0) }, &mut t, 0.3);
        assert!(!o.decouple);
    }

    #[test]
    fn a_go_around_or_a_lost_capture_decouples_and_never_rearms() {
        let (mut c, mut t) = (Coupling::Coupled, 0.0);
        let o = c.step(CouplingIn { go_around: true, ..inp(2.0, 2.0) }, &mut t, 0.3);
        assert!(o.decouple && o.own_lateral && o.own_vertical);
        assert_eq!(c, Coupling::Abandoned);
        assert!(!c.step(inp(0.0, 0.0), &mut t, 0.3).press_appr, "abandoned stays abandoned");
        let mut c = Coupling::Coupled;
        assert!(c.step(inp(2.0, 0.0), &mut t, 0.3).decouple, "glideslope lost");
        // APPR that never arms is abandoned without a fight.
        let (mut c, mut t) = (Coupling::Off, 0.0);
        c.step(inp(0.0, 0.0), &mut t, 0.3);
        for _ in 0..30 {
            c.step(inp(0.0, 0.0), &mut t, 0.3);
        }
        assert_eq!(c, Coupling::Abandoned);
    }

    #[test]
    fn nothing_couples_unless_the_airframe_allows_it_and_the_needle_is_close() {
        let (mut c, mut t) = (Coupling::Off, 0.0);
        let o = c.step(CouplingIn { allowed: false, ..inp(0.0, 0.0) }, &mut t, 0.3);
        assert!(!o.press_appr && c == Coupling::Off);
        let o = c.step(CouplingIn { needles: needles(2.5, 0.0), ..inp(0.0, 0.0) }, &mut t, 0.3);
        assert!(!o.press_appr, "full-scale localizer: not yet");
        let o = c.step(CouplingIn { dist_nm: 20.0, ..inp(0.0, 0.0) }, &mut t, 0.3);
        assert!(!o.press_appr, "too far");
    }
}
