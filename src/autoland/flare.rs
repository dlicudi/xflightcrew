//! The arrest and flare: climb limits, gates and targets, the aim, the retard
//! power, the elevator's level hold until the autopilot is in.

use super::*;

/// Only climb with this much surplus over the approach speed. Climbing slow in a
/// marginal airframe bleeds energy to a stall (the reason this loop was
/// descent-only) — so a low-and-slow aircraft holds level and lets the throttle
/// governor rebuild speed FIRST, then the climb engages once the surplus exists.
pub(super) const CLIMB_SPEED_MARGIN_KTS: f64 = 5.0;
/// Ceiling on the safe-altitude climb (fpm). Was 500 — "gentle and energy-
/// preserving" read as anemic against real terrain (the Rock closed at an
/// effective 5,000+ fpm, 2026-07-22). The climb gate already requires
/// surplus airspeed, and speed protection still owns the slow case.
pub(super) const MAX_CLIMB_FPM: f64 = 800.0;

/// ...and the AIRFRAME's climb when it has more, because 800 fpm is a light
/// single's answer to terrain and an airliner's idea of level flight. It was
/// raised from 500 once already for being anemic; on a 737 it is anemic
/// again — the EGPH go-around (2026-09-20) climbed away from a balked
/// approach at exactly this number. Never LESS than the GA figure, so
/// nothing that flew before gets weaker.
pub(crate) fn max_climb_fpm() -> f64 {
    crate::procedure::climb_fpm().max(MAX_CLIMB_FPM)
}
/// While below the terrain floor, command the throttle governor to this much over
/// the approach speed, so it builds the surplus the climb gate needs.
pub(super) const CLIMB_ENERGY_BONUS_KTS: f64 = 12.0;

/// Flare only inside this distance to the threshold — beyond it, a low radio
/// altitude is terrain under the approach, not the runway. (25 ft on a 3° path
/// is ~0.08 nm out; 0.35 leaves margin for displaced thresholds.)
pub(super) const FLARE_MAX_DIST_NM: f64 = 0.35;
/// The roundout band: below this multiple of the flare height (inside the same
/// runway gates as the flare itself), the vertical target eases to
/// [`ROUNDOUT_VS_FPM`] so the flare is entered from a gentle descent rather
/// than from the full path rate or a level float.
pub(super) const ROUNDOUT_MULT: f64 = 2.0;
pub(super) const ROUNDOUT_VS_FPM: f64 = -250.0;
pub(super) const FLARE_VS_FPM: f64 = -100.0;

/// The arrest gates are SECONDS of sink, not feet: 25 ft is three seconds
/// of a 172's path sink and two of a 737's, and the 737 through this
/// autopilot's lag needs every one of them (EGCC 23L, 2026-09-19: the
/// roundout opened at 50 ft with the aircraft sinking 712 fpm, 4 s from the
/// pavement, and the arrest could only balloon or arrive firm). Both gates
/// are the greater of the fixed feet and the seconds at the NOMINAL path
/// sink for this groundspeed, so every airframe up to a 172's speeds keeps
/// exactly its old feet (50 and 25 at 75 kt) and a jet's open higher: at
/// 132 kt over the ground, 76 and 35.
pub(super) const ROUNDOUT_LEAD_S: f64 = 6.5;
pub(super) const FLARE_LEAD_S: f64 = 3.0;
/// The gates for this approach: (flare height, roundout band top), feet.
pub fn arrest_gates_ft(flare_radio_ft: f64, gs_kts: f64) -> (f64, f64) {
    let sink_fps = (gs_kts.max(0.0) / 60.0) * gp_ft_per_nm() / 60.0;
    let flare = flare_radio_ft.max(sink_fps * FLARE_LEAD_S);
    let band = (flare * ROUNDOUT_MULT).max(sink_fps * ROUNDOUT_LEAD_S);
    (flare, band)
}
/// The arrest's targets scale with the path sink the same way: the 172's
/// -250 then -100 are 60% and 25% of its 400 fpm path, and a 737 asked to
/// hold -100 from 35 ft floats a kilometre at 140 kt before it touches.
/// (roundout, flare), fpm; never shallower than the fixed figures.
pub fn arrest_targets_fpm(nominal_vs_fpm: f64) -> (f64, f64) {
    (
        ROUNDOUT_VS_FPM.min(0.6 * nominal_vs_fpm),
        FLARE_VS_FPM.min(0.25 * nominal_vs_fpm),
    )
}


/// The altitude dial must never be within reach BELOW the aircraft while
/// autoland flies: an armed capture takes the vertical away from the VS dial
/// (see the arm sequence in [`run_autoland`]). The LEAX rule
/// ([`autoflight::WindowRule::ParkAbove`]: 1,000 ft above, rounded up to the
/// hundred, unless already 500 ft clear). Pure.
pub fn parked_altitude_dial_ft(dial_ft: f64, alt_ft: f64) -> Option<f64> {
    autoflight::window_for(autoflight::WindowRule::ParkAbove, dial_ft, alt_ft, 0.0)
}

/// The window the arm parks for the airframe's rule
/// ([`autoflight::window_for`]): above, or at the field on an MCP
/// that flies a V/S only toward its window (the 737, LEMG 13 2026-10-05).
/// Pure.
pub fn parked_window_ft(
    rule: autoflight::WindowRule,
    dial_ft: f64,
    alt_ft: f64,
    field_ft: f64,
) -> Option<f64> {
    autoflight::window_for(rule, dial_ft, alt_ft, field_ft)
}

/// A probed threshold elevation within this of the header is the header (the
/// mesh and the file agree; no reason to move the aim by a few feet).
pub const THRESHOLD_ELEV_TRUST_FT: f64 = 20.0;
/// ...and one further than this from it is not believed (a probe off the
/// pavement, a broken mesh): the header stays, the number is still reported.
pub const THRESHOLD_ELEV_MAX_DIFF_FT: f64 = 300.0;
/// The glidepath's aim elevation: the HIGHER of the probed threshold and
/// touchdown-zone points when they disagree with the apt.dat header by more
/// than [`THRESHOLD_ELEV_TRUST_FT`] and less than [`THRESHOLD_ELEV_MAX_DIFF_FT`];
/// the header otherwise (and always without a probe). Returns (aim, probed).
pub fn aim_elevation_ft(header_ft: f64, probed_ft: Option<&[f64]>) -> (f64, Option<f64>) {
    let Some(p) = probed_ft.filter(|v| !v.is_empty()) else {
        return (header_ft, None);
    };
    let probed = p.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if !probed.is_finite() {
        return (header_ft, None);
    }
    let diff = (probed - header_ft).abs();
    if diff <= THRESHOLD_ELEV_TRUST_FT || diff > THRESHOLD_ELEV_MAX_DIFF_FT {
        return (header_ft, Some(probed));
    }
    (probed, Some(probed))
}

/// How far a tuned glideslope may move the aim from the pavement's, ft:
/// the path's threshold crossing (51.6 ft at 3 degrees aimed 300 m in)
/// stays inside a book's 35 to 80 ft.
pub const GS_AIM_BELOW_FT: f64 = 15.0;
pub const GS_AIM_ABOVE_FT: f64 = 30.0;

/// The aim elevation with the runway's ILS glideslope tuned: the path that
/// lies ON the glideslope (`ils::glideslope_aim_ft`), as far as a book's
/// threshold crossing over the pavement (`pavement_aim_ft`, from
/// [`aim_elevation_ft`]) allows; the pavement's aim without one. Pure.
///
/// EGBB 15, 2026-09-28 (sortie 1790601822-full_auto, #156): the probe put
/// the aim at 297 ft (the header 327) while the glideslope's antenna stands
/// at 348 ft, 0.34 nm in: the needle read 1.2 dots low at 497 ft on a path
/// autoland was flying as it meant to. Aimed at 327 the path lies
/// 30 ft nearer the beam with the wheels still crossing the numbers at 81 ft.
pub fn aim_with_glideslope(pavement_aim_ft: f64, gs_aim_ft: Option<f64>) -> f64 {
    match gs_aim_ft.filter(|g| g.is_finite()) {
        Some(g) => pavement_aim_ft + (g - pavement_aim_ft).clamp(-GS_AIM_BELOW_FT, GS_AIM_ABOVE_FT),
        None => pavement_aim_ft,
    }
}

/// A flare is CLIMBING (a balloon) only past this, fpm. A near-level float
/// reads a few fpm either side of zero (EGBB 15: -2, +1, +2, 0 at 22.9 ft),
/// and +1 is not a climb: it eased the held column 0.28 to 0.20 and would
/// put the approach thrust back on a lever the float is taking off.
pub const BALLOON_CLIMB_FPM: f64 = 25.0;

/// What to dial into the VS mode so the aircraft actually FLIES `target_fpm`.
/// The plugin's direct elevator channel, and the pilot's take-back latch —
/// the SAME pair the takeoff rotates on. `override_joystick_pitch` bypasses
/// the autopilot's pitch servo while leaving it the roll, so the flare can be
/// flown on the elevator with the AP still holding the bug.
pub(super) const FLARE_PITCH: &str = "xcopilot/pitch";
/// The roll channel the takeoff's climb-out flies, and its take-back latch:
/// with the servos off for the arrest, the runway track is held here.

pub(super) const FLARE_ROLL: &str = "xcopilot/roll";

/// The elevator that holds a wanted sink rate, flown DIRECTLY instead of
/// asked of the autopilot's vertical-speed mode: the level hold while the
/// autopilot is not yet confirmed (`ApEngage`, [`engage_hold_gains`]).
///
/// P on the sink-rate error plus D on its trend, in units of elevator, and
/// an integrator that finds the standing pull the aircraft needs. A measured
/// CLIMB (past [`BALLOON_CLIMB_FPM`]) never pushes: the sink a push rebuilds
/// is worse than the climb (the SF50's roundout wrote -0.05 at +62 and +214
/// fpm, LEMG 12, 2026-09-28).
#[derive(Debug, Default, Clone, Copy)]
pub struct LevelHold {
    pub(super) trim: f64,
    pub(super) gains: LevelHoldGains,
}

/// The level hold's numbers ([`engage_hold_gains`]).
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct LevelHoldGains {
    /// Elevator per 100 fpm of sink error.
    pub p: f64,
    /// Elevator per 100 fpm per second of sink trend.
    pub d: f64,
    /// The integrator's rate, elevator per 100 fpm of error per second.
    pub trim_rate: f64,
    /// Most pull, and most push.
    pub max: f64,
    pub min: f64,
}

impl LevelHold {
    /// `vvi_trend` is fpm per second, positive = sink easing.
    pub fn step(&mut self, target_fpm: f64, vvi_fpm: f64, vvi_trend: f64, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(0.02, 0.5);
        let g = self.gains;
        // Positive error = sinking FASTER than wanted = pull.
        let err = (target_fpm - vvi_fpm) / 100.0;
        self.trim = (self.trim + err * g.trim_rate * dt).clamp(g.min, g.max);
        let want = (err * g.p - (vvi_trend / 100.0) * g.d + self.trim).clamp(g.min, g.max);
        if vvi_fpm > BALLOON_CLIMB_FPM {
            want.max(0.0)
        } else {
            want
        }
    }

    /// The law with these numbers.
    pub fn with_gains(gains: LevelHoldGains) -> Self {
        Self {
            gains,
            ..Self::default()
        }
    }
}
