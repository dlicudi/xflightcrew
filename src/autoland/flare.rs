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

/// The VS mode is a slow servo, and the landing arrest is the one place the
/// dial's lag is measured in feet of runway. LXGB 27, 2026-09-14 (the 172's
/// third firm arrival in three days, same shape each time): the roundout
/// dialled -250 at 49 ft radio and the aircraft was still sinking 386 fpm
/// four seconds later; the flare dialled -100 at 23 ft and the vertical
/// speed never moved off -260 before the wheels touched at -309 (the
/// blackbox's `cmd_vs` column changed on time, the `vsi` column never
/// followed). Fitted to that tick log the autopilot follows the dial as a
/// first-order lag of about 4.5 s with a tick of dead time, which from the
/// flare gate at approach sink is longer than the flare itself. So in the
/// roundout and the flare the DIAL leads the target by the error the
/// autopilot still owes (`vs_dial_fpm`): dial = target + gain x (target -
/// actual). Gain 2 makes the closed lag ~1.5 s, comfortably inside the
/// stability margin a 1 s dead time leaves (about 7). The dial may read a
/// CLIMB for a moment - that is the flare, the yoke coming back - capped at
/// [`VS_LEAD_MAX_CLIMB_FPM`] so a balloon can never be commanded, and the
/// glidepath above the band is untouched (its own P law tracks a ramp with
/// `path_descent_fpm`, and every approach test stays byte-identical).
/// The 172's numbers live in [`crate::aircraft::ArrestLeadProfile::default`]
/// (gain 2, cap +300, no trend term); a jet declares its own in its profile.
#[cfg(test)]
pub(super) const VS_LEAD_GAIN: f64 = 2.0;
#[cfg(test)]
pub(super) const VS_LEAD_MAX_CLIMB_FPM: f64 = 300.0;

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

/// The gentlest retard below the retard height, lever per second.
pub(super) const RETARD_MIN_PER_S: f64 = 0.03;
/// The retard waits while the airspeed is this far under the approach speed
/// (kt), unless the profile says otherwise (`retard_slow_kts`).
pub(super) const RETARD_SLOW_KTS: f64 = 5.0;
/// Time to the wheels is taken at no less than this sink (fpm): a float is
/// not an infinite time, it is a slow retard.
pub(super) const RETARD_SINK_FLOOR_FPM: f64 = 100.0;
/// A flare sinking less than this (fpm) is FLOATING: the arrest is done and
/// the thrust is what holds it up, so the retard starts, whatever the height
/// and the speed. EGBB 15 (737, 2026-09-28, sortie 1790601822): levelled at
/// 22.7 ft on 0.30 of lever, held by the height key over 20 ft, then by the
/// time key (a float is 13 s from the wheels), then by the speed key once
/// the float had bled 7 kt; the lever never moved and the float ran 4 s.
pub const RETARD_FLOAT_FPM: f64 = 100.0;
/// ...or will be, this many seconds ahead on the sink's trend: the float is
/// met where the arrest is heading, not where it arrives. Thrust acts through
/// the speed and the column through the 737's 1.5 s dead time, so a retard
/// begun at the float itself is late: at EGBB 15 the sink came from -300 to
/// -111 in 2.1 s. Replayed through that flare's fitted plant (6,300 fpm per
/// unit, 1.7 s) and its air, a lead of 1 s left 3.6 s of float, 1.5 s none.
pub const RETARD_FLOAT_LEAD_S: f64 = 1.5;
/// The sink's trend for the float ahead is smoothed over this, seconds.
pub(super) const RETARD_TREND_TAU_S: f64 = 0.5;
/// A flare is CLIMBING (a balloon) only past this, fpm. A near-level float
/// reads a few fpm either side of zero (EGBB 15: -2, +1, +2, 0 at 22.9 ft),
/// and +1 is not a climb: it eased the held column 0.28 to 0.20 and would
/// put the approach thrust back on a lever the float is taking off.
pub const BALLOON_CLIMB_FPM: f64 = 25.0;

/// Why the retard did what it did this tick (the `power` tick's `retard`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetardKey {
    /// The first flare tick: the approach's power is where it starts.
    Entry,
    /// Above the retard height.
    Height,
    /// More time to the wheels than `retard_at_s`.
    Time,
    /// Slower than the approach speed less the margin, the retard not yet
    /// begun and the aircraft still sinking: the thrust stays.
    Slow,
    /// Floating (sink under [`RETARD_FLOAT_FPM`], now or
    /// [`RETARD_FLOAT_LEAD_S`] ahead on its trend): walking off at the
    /// airframe's quickest retard, at any height and any speed.
    Float,
    /// Walking to idle, timed to the wheels.
    Timed,
}

impl RetardKey {
    pub fn word(self) -> &'static str {
        match self {
            RetardKey::Entry => "entry",
            RetardKey::Height => "height",
            RetardKey::Time => "time",
            RetardKey::Slow => "slow",
            RetardKey::Float => "float",
            RetardKey::Timed => "timed",
        }
    }
}

/// THE FLARE RETARD, keyed on what it is for: HEIGHT (nothing above the
/// retard height), TIME to the wheels (the lever reaches idle as they
/// arrive: the rate is what is left over the seconds left, radio over
/// sink), the FLOAT (an arrested sink held up by thrust, or one the trend
/// will arrest within [`RETARD_FLOAT_LEAD_S`]: off it comes at any height,
/// at the airframe's quickest) and SPEED (a flare slower than the approach speed less
/// [`RETARD_SLOW_KTS`] may DEFER the retard while it is still sinking; it
/// never holds power in a float and never stops a retard that has begun).
/// Never on the sink the arrest is removing: the old law held the lever
/// while the arrest worked and let it go in the float the arrest had made.
/// It "fired latest exactly when it should already be finished" (EGLF 06
/// 2026-09-22; LEMG 13 2026-09-26 and 2026-09-28; issue #76). The float is
/// the other half: height, time and speed each held 0.30 through a 4 s
/// float at 22.7 ft (EGBB 15, 2026-09-28; issue #146).
///
/// Per approach: a go-around resets it, so a second flare starts from its
/// own power. Never adds power or impersonates an airspeed governor in the
/// flare; a lever still up at the wheels is the rollout's to close.
#[derive(Default)]
pub(super) struct FlarePower {
    pub(super) last_command: Option<f64>,
    pub(super) last_key: Option<RetardKey>,
    /// The last sink read and its smoothed rate of change (fpm per second,
    /// positive = easing), for the float ahead.
    pub(super) prev_vvi: Option<f64>,
    pub(super) trend_fpm_s: f64,
    /// The time to the wheels has come inside `retard_at_s`, or the flare
    /// has floated (latched).
    pub(super) due: bool,
    /// The lever has started off (latched): the speed key no longer holds.
    pub(super) begun: bool,
}

impl FlarePower {
    pub(super) fn reset(&mut self) {
        *self = Self::default();
    }

    /// The lever this tick; `None` when a measurement is missing (no guessed
    /// power and no ramp advance). An unread airspeed is no speed key.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn step(
        &mut self,
        cfg: crate::aircraft::FlarePowerProfile,
        measured: f64,
        vvi: f64,
        dt: f64,
        radio_ft: f64,
        ias: Option<f64>,
        approach_ias: Option<f64>,
    ) -> Option<f64> {
        if !measured.is_finite()
            || !vvi.is_finite()
            || !radio_ft.is_finite()
            || !dt.is_finite()
            || dt <= 0.0
            || !cfg.reduction_per_second.is_finite()
            || cfg.reduction_per_second <= 0.0
        {
            return None;
        }
        // First flare tick preserves the observed approach power. Readback may
        // lag a write, but it must never restart the ramp or increase power.
        let current = measured
            .clamp(0.0, 1.0)
            .min(self.last_command.unwrap_or(1.0));
        let slow_kts = cfg
            .retard_slow_kts
            .filter(|k| k.is_finite() && *k >= 0.0)
            .unwrap_or(RETARD_SLOW_KTS);
        let slow = match (ias, approach_ias) {
            (Some(v), Some(a)) if v.is_finite() && a.is_finite() => v < a - slow_kts,
            _ => false,
        };
        // The sink's trend, smoothed over RETARD_TREND_TAU_S: a float the
        // arrest will reach a lever's lag from now is a float now.
        if let Some(p) = self.prev_vvi {
            let raw = (vvi - p) / dt;
            self.trend_fpm_s += (raw - self.trend_fpm_s) * (dt / RETARD_TREND_TAU_S).min(1.0);
        }
        self.prev_vvi = Some(vvi);
        let floating = vvi + self.trend_fpm_s.max(0.0) * RETARD_FLOAT_LEAD_S > -RETARD_FLOAT_FPM;
        let above = cfg
            .retard_below_ft
            .filter(|h| h.is_finite() && *h > 0.0)
            .is_some_and(|h| radio_ft > h);
        let sink = (-vvi).max(RETARD_SINK_FLOOR_FPM);
        let to_touch_s = (radio_ft.max(0.0) / sink * 60.0).max(0.5);
        let fast = match (ias, approach_ias, cfg.retard_fast_kts) {
            (Some(v), Some(a), Some(k)) if v.is_finite() && a.is_finite() && k.is_finite() => {
                v > a + k
            }
            _ => false,
        };
        self.due |= floating
            || (!above
                && (fast
                    || cfg
                        .retard_at_s
                        .filter(|s| s.is_finite() && *s > 0.0)
                        .is_none_or(|s| to_touch_s <= s)));
        let key = if self.last_command.is_none() {
            RetardKey::Entry
        } else if floating {
            RetardKey::Float
        } else if !self.due {
            if above {
                RetardKey::Height
            } else {
                RetardKey::Time
            }
        } else if slow && !self.begun {
            RetardKey::Slow
        } else {
            RetardKey::Timed
        };
        let fastest = cfg
            .retard_max_per_s
            .filter(|r| r.is_finite() && *r > 0.0)
            .unwrap_or(cfg.reduction_per_second)
            .max(RETARD_MIN_PER_S);
        let rate = match key {
            // A float has no time to the wheels: the thrust holding it up
            // is what comes off, as quickly as the airframe allows.
            RetardKey::Float => Some(
                cfg.retard_float_per_s
                    .filter(|r| r.is_finite() && *r > 0.0)
                    .unwrap_or(fastest),
            ),
            RetardKey::Timed => Some((current / to_touch_s).clamp(RETARD_MIN_PER_S, fastest)),
            _ => None,
        };
        let next = match rate {
            Some(rate) => {
                self.begun = true;
                (current - rate * dt.min(1.0)).max(0.0)
            }
            None => current,
        };
        self.last_command = Some(next);
        self.last_key = Some(key);
        Some(next)
    }
}

/// The lever in a flare that is CLIMBING (past [`BALLOON_CLIMB_FPM`]): the
/// approach's own thrust, never more; `None` otherwise (the flare's usual
/// idle or progressive law). A near-level float is not a climb: its thrust
/// is the retard's to take off. Pure.
/// `above_band` holds it too: a committed balloon still above the roundout
/// band comes back down on approach power, not at idle.
pub fn balloon_power(
    vvi_fpm: Option<f64>,
    approach_thr: Option<f64>,
    above_band: bool,
    progressive_flare: bool,
) -> Option<f64> {
    // An airframe with a measured progressive retard owns its energy through
    // the whole flare. Its FlarePower law is monotonic even in a balloon;
    // restoring approach thrust here bypassed that guarantee and floated the
    // C172 at LFOS 06 from 23 to 26 ft (2026-10-03). The legacy/no-profile
    // path still uses approach thrust to bring a committed high balloon back.
    if progressive_flare {
        return None;
    }
    match (vvi_fpm, approach_thr) {
        (Some(v), Some(t)) if (v > BALLOON_CLIMB_FPM || above_band) && t.is_finite() => {
            Some(t.clamp(0.0, 1.0))
        }
        _ => None,
    }
}

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
