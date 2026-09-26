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

/// The live airframe's approach button (its own where it has one).
pub fn approach_command() -> String {
    crate::aircraft::live_profile()
        .and_then(|p| p.approach_command.clone())
        .unwrap_or_else(|| CMD_APPROACH.to_string())
}

/// The live airframe's approach status dataref (its own where it has one).
pub fn approach_status_dref() -> String {
    crate::aircraft::live_profile()
        .and_then(|p| p.approach_status.clone())
        .unwrap_or_else(|| APPR_STATUS.to_string())
}

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

// ---------------------------------------------------------------------------
// The pilot-flying ILS setup: what a pilot monitoring does after "set up the
// ILS" (2026-09-25). The tool ([`crate::tools::approach::PrepareIls`]) tunes,
// sets the course source and the minimums; the watcher below arms APPR when
// the geometry allows it (never blind: APPR is a toggle), reads it back, and
// calls the needles alive and the modes captured, setting the missed approach
// altitude at glideslope capture.
// ---------------------------------------------------------------------------

pub const NAV_STATUS: &str = "sim/cockpit2/autopilot/nav_status";
pub const SERVOS: &str = "sim/cockpit2/autopilot/servos_on";
pub const AP_POWER: &str = "sim/cockpit2/autopilot/autopilot_has_power";
pub const HSI_SOURCE: &str = "sim/cockpit2/radios/actuators/HSI_source_select_pilot";
pub const CMD_HSI_NAV1: &str = "sim/autopilot/hsi_select_nav_1";
pub const RA_DH: &str = "sim/cockpit2/gauges/actuators/radio_altimeter_bug_ft_pilot";
pub const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";

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

fn wrap180(a: f64) -> f64 {
    (a + 540.0).rem_euclid(360.0) - 180.0
}

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
    let off = wrap180(from_loc - (ils.course_true + 180.0)).abs();
    if off > 90.0 && d > 1.0 {
        return ArmVerdict::Never("we're past the localiser, on the wrong side of the runway");
    }
    let covered = (off <= LOC_WIDE_DEG && d <= LOC_WIDE_NM) || (off <= LOC_NARROW_DEG && d <= LOC_NARROW_NM);
    if !covered {
        return ArmVerdict::Wait("we're outside the localiser's coverage");
    }
    if wrap180(heading_mag - ils.course_mag).abs() > MAX_INTERCEPT_DEG {
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
    /// An automation is flying (autoland, the follower, a circuit, full
    /// auto): the approach is its to fly, and [`Coupling`] owns APPR.
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

/// Generation of the running setup watch: a new "set up the ILS" or a stand
/// down retires the old one.
pub static SETUP_GEN: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Retire any running setup watch (airframe swap, instructor entry).
pub fn stop_setup_watch() {
    SETUP_GEN.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
}

fn automation_flying() -> bool {
    use std::sync::atomic::Ordering::Relaxed;
    crate::full_auto::ACTIVE.load(Relaxed)
        || crate::procedure::ACTIVE.load(Relaxed)
        || crate::circuit::ACTIVE.load(Relaxed)
        || crate::autoland::ACTIVE.load(Relaxed)
}

/// Run a [`SetupWatch`] on its own thread and runtime (the command loop's
/// runtime is blocked on the listener between commands, as for the altitude
/// capture watcher). Its writes are the APPR press, only from off, and the
/// missed approach altitude at capture.
pub fn spawn_setup_watch(base: String, ils: crate::nav::Ils, mut watch: SetupWatch, voice: Option<crate::voice::Voice>) {
    use std::sync::atomic::Ordering::SeqCst;
    let my_gen = SETUP_GEN.fetch_add(1, SeqCst) + 1;
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            return;
        };
        rt.block_on(async move {
            let xp = crate::xplane::XPlaneClient::new(base);
            let dt = 0.5;
            loop {
                if SETUP_GEN.load(SeqCst) != my_gen {
                    return;
                }
                if crate::xplane::sim_frozen(&xp).await {
                    tokio::time::sleep(std::time::Duration::from_secs_f64(dt)).await;
                    continue;
                }
                let appr_dref = approach_status_dref();
                let (hdef, vdef, locv, gsv, appr, nav, gs, servos, lat, lon, hdg, agl) = tokio::join!(
                    xp.get_float(HDEF),
                    xp.get_float(VDEF),
                    xp.get_float(LOC_VALID),
                    xp.get_float(GS_VALID),
                    xp.get_float(&appr_dref),
                    xp.get_float(NAV_STATUS),
                    xp.get_float(GS_STATUS),
                    xp.get_float(SERVOS),
                    xp.get_float(crate::datarefs::LATITUDE),
                    xp.get_float(crate::datarefs::LONGITUDE),
                    xp.get_float(crate::datarefs::COMPASS_HEADING_DEG),
                    xp.get_float(crate::datarefs::AGL_METERS),
                );
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
                    appr: appr.ok(),
                    nav: nav.ok(),
                    gs: gs.ok(),
                    servos: servos.is_ok_and(|v| v > 0.5),
                    automation: automation_flying(),
                    verdict: arm_verdict(lat, lon, hdg, &ils),
                    agl_ft: agl * 3.28084,
                };
                let out = watch.step(i, dt);
                if out.press_appr {
                    let _ = xp.run_command(&approach_command()).await;
                    crate::blackbox::record("ils", json!({ "event": "appr_pressed", "rwy": ils.runway, "icao": ils.airport }));
                }
                if let Some(m) = out.set_dial {
                    let _ = xp.set_float(ALT_DIAL, m).await;
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
