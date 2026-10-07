//! The arrest and flare: climb limits, gates and targets, the aim, the retard
//! power, the flare pitch and its model, the final's air.

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

/// The flare's sink target at this height: an EXPONENTIAL flare, as
/// autoland systems fly it. The touchdown sink (`flare_vs`) plus the height
/// over [`FLARE_TAU_S`], never steeper than the approach path. The flare
/// used to hold `flare_vs` flat from its gate: a 737 asked for -175 fpm from
/// 35 ft takes twelve seconds to arrive, and floated LEMG 13 from 36 to 19 ft
/// over five of them before the power came off and it dropped in at -859
/// fpm (2026-09-26). Proportional to the height it keeps descending and
/// arrives at `flare_vs` in about five. Pure.
pub fn flare_target_fpm(radio_ft: f64, flare_vs: f64, nominal_vs: f64) -> f64 {
    flare_target_with(radio_ft, flare_vs, nominal_vs, flare_tau_s())
}

/// [`flare_target_fpm`] with the time constant given: `None` is the flat
/// flare every airframe but a declared one keeps. The 172's flare is flown
/// through its VS mode's 4.5 s lag, and a target that moves with the height
/// is faster than that lag can follow (the sortie test lands it at -298
/// fpm); the 737's is flown on the elevator with the servos out. Pure.
pub fn flare_target_with(radio_ft: f64, flare_vs: f64, nominal_vs: f64, tau_s: Option<f64>) -> f64 {
    match tau_s.filter(|t| t.is_finite() && *t > 0.5) {
        Some(tau) => (flare_vs - radio_ft.max(0.0) * 60.0 / tau).max(nominal_vs.min(flare_vs)),
        None => flare_vs,
    }
}

/// The live airframe's exponential flare time constant, if it declares one
/// (`flare_elevator.tau_s`).
pub(super) fn flare_tau_s() -> Option<f64> {
    crate::aircraft::live_profile()
        .and_then(|p| p.flare_elevator)
        .and_then(|f| f.tau_s)
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

/// Elevator for a wanted sink rate, flown DIRECTLY instead of asked of the
/// autopilot's vertical-speed mode.
///
/// The whole `vs_dial_fpm` lead exists to compensate for a lag we chose to
/// fly through: the 737's VS mode is two 1.5 s lags with 1.5 s of dead time,
/// and the arrest is over in three seconds. Commanding the elevator removes
/// the lag rather than predicting it — which is what the TAKEOFF already
/// does, for the same reason, in its own comment: "the AP's VS mode is too
/// slow for a rotation". EGPH (2026-09-20) touched down at 977 fpm against a
/// 175 fpm target with the arrest asking the dial nicely.
///
/// P on the sink-rate error plus D on its trend, in units of elevator. Small
/// gains and a tight clamp on purpose: this is three seconds of flight a few
/// feet up, and a flare that over-rotates is a tail strike. The trim term is
/// the standing pull the aircraft needs to hold the attitude, found by the
/// integrator rather than assumed.
///
/// IN THE FLARE THE COLUMN ONLY COMES BACK, never forward (`ratchet`). A
/// tracker that has BEATEN its target relaxes, and forward column a few feet
/// up is a nosewheel-first landing. EGLF 06 (2026-09-22): the arrest pulled
/// the 737 from -511 to -102 fpm against a -197 target, the error went
/// negative and the elevator came back 0.35 -> 0.185 at thirteen feet, while
/// [`FlarePower`] - unlocked by that very same arrest, sink now inside its
/// `hold_sink_fpm` - walked the lever 0.29 to idle over the last 3.4 s. The
/// thrust left DURING the arrest instead of before it, the sink rebuilt to
/// -358 and worsening, and by then the law was saturated at
/// [`FLARE_PITCH_MAX`] with nothing left to give. The cap still bounds the
/// attitude, so the ratchet cannot buy a tail strike; a float it can buy is
/// the overflight guard's to judge, and the guard already knows the recited
/// landing distance.
///
/// The ROUNDOUT above the flare is ordinary flying and stays free both ways:
/// the ratchet is the last few feet, where the only question left is how
/// hard the wheels arrive.
#[derive(Debug, Default, Clone, Copy)]
pub struct FlarePitch {
    pub(super) trim: f64,
    pub(super) held: f64,
    pub(super) gains: FlareGains,
    /// The gentlest sink the flare has reached (fpm, negative), since it
    /// began: the target never asks for more sink than that again.
    pub(super) best_vvi: Option<f64>,
}

/// The flare law's numbers for THIS airframe (the profile's
/// `flare_elevator:`, [`crate::aircraft::FlareElevatorProfile`]). The default
/// is the law as the 172 and the 737 have flown it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlareGains {
    /// Elevator per 100 fpm of sink error.
    pub p: f64,
    /// Elevator per 100 fpm per second of sink trend.
    pub d: f64,
    /// The integrator's rate, elevator per 100 fpm of error per second.
    pub trim_rate: f64,
    /// Most pull, and most push.
    pub max: f64,
    pub min: f64,
    /// In a balloon the held pull eases toward the free law at most this
    /// fast (elevator per second); `None` lets go at once.
    pub balloon_ease_per_s: Option<f64>,
}

impl Default for FlareGains {
    fn default() -> Self {
        Self {
            p: FLARE_P_PER_100FPM,
            d: FLARE_D_PER_100FPM_S,
            trim_rate: FLARE_TRIM_PER_100FPM_S,
            max: FLARE_PITCH_MAX,
            min: FLARE_PITCH_MIN,
            balloon_ease_per_s: None,
        }
    }
}

impl FlareGains {
    /// The airframe's declared numbers over the default law.
    pub fn from_profile(p: Option<crate::aircraft::FlareElevatorProfile>) -> Self {
        let mut g = Self::default();
        if let Some(p) = p.and_then(|p| p.valid()) {
            g.p *= p.gain;
            g.d *= p.gain;
            g.trim_rate *= p.trim_gain;
            g.max = p.max;
            g.min = -p.max_push;
            g.balloon_ease_per_s = p.balloon_ease_per_s;
        }
        g
    }
}

/// Elevator per 100 fpm of sink error. A flare corrects hundreds of fpm with
/// a few percent of travel.
pub(super) const FLARE_P_PER_100FPM: f64 = 0.060;
/// Damping on the sink TREND, per 100 fpm per second — the term that stops
/// the pull becoming a balloon.
pub(super) const FLARE_D_PER_100FPM_S: f64 = 0.050;
/// The integrator's rate, per 100 fpm of standing error per second. Small:
/// it is there to find the standing pull, not to fly the arrest — wound
/// harder it carried the 737 through its target into a 66 fpm float.
pub(super) const FLARE_TRIM_PER_100FPM_S: f64 = 0.015;
/// The flare holds the gentlest sink it has reached, but never asks for
/// less than this (fpm): a float is not a landing either.
pub(super) const FLARE_HOLD_MIN_SINK_FPM: f64 = 150.0;
/// Total elevator the flare may command. Up is positive.
pub(super) const FLARE_PITCH_MAX: f64 = 0.35;
/// ...and the push, which is deliberately far smaller: the ground is there.
pub(super) const FLARE_PITCH_MIN: f64 = -0.10;
/// The ratchet lets go this far ahead of a balloon: the sink PREDICTED this
/// many seconds out (the trend) above zero counts as climbing.
pub(super) const BALLOON_LEAD_S: f64 = 0.5;

impl FlarePitch {
    /// `vvi_trend` is fpm per second, positive = sink easing (the same sign
    /// the VS lead uses). `None` for either measurement holds what we had.
    ///
    /// `ratchet` is the flare proper (as against the roundout above it): see
    /// [`FlarePitch`] for why the column only ever comes further back there.
    pub fn step(
        &mut self,
        target_fpm: f64,
        vvi_fpm: f64,
        vvi_trend: f64,
        dt_s: f64,
        ratchet: bool,
    ) -> f64 {
        let dt = dt_s.clamp(0.02, 0.5);
        let g = self.gains;
        // In the flare the sink never grows again: the target is held to the
        // gentlest sink reached (no gentler than FLARE_HOLD_MIN_SINK). The
        // roundout had the 737 at -300 by 29 ft; the exponential target was
        // still -660 there, the law saw nothing to pull for, and the sink
        // built back to -587 at the wheels (LEMG 13, 2026-09-26).
        let target_fpm = if ratchet {
            let best = self.best_vvi.map_or(vvi_fpm, |b| b.max(vvi_fpm));
            self.best_vvi = Some(best);
            target_fpm.max(best.min(-FLARE_HOLD_MIN_SINK_FPM))
        } else {
            self.best_vvi = None;
            target_fpm
        };
        // Positive error = sinking FASTER than wanted = pull.
        let err = (target_fpm - vvi_fpm) / 100.0;
        self.trim = (self.trim + err * g.trim_rate * dt).clamp(g.min, g.max);
        let want = (err * g.p - (vvi_trend / 100.0) * g.d + self.trim).clamp(g.min, g.max);
        if !ratchet {
            // The roundout is still ordinary flying: the law is free both
            // ways. But the pull it ends on is where the flare STARTS: reset
            // to nothing, the ratchet began from the flare's first ask, and
            // the 737 went from 0.29 of elevator to 0.016 in one tick at the
            // hand-over (38 ft), the nose dropped, and it met the runway at
            // -773 fpm and bounced to 11 ft (LEMG circuit, 2026-09-26).
            // ...but a measured CLIMB (past BALLOON_CLIMB_FPM) is a
            // balloon here too, and a balloon never pushes: the SF50's
            // roundout wrote -0.05 at +62 and +214 fpm (LEMG 12,
            // 2026-09-28) and the sink it rebuilt met the runway.
            let want = if vvi_fpm > BALLOON_CLIMB_FPM {
                want.max(0.0)
            } else {
                want
            };
            self.held = want.max(0.0);
            return want;
        }
        if vvi_fpm > 0.0 || vvi_fpm + vvi_trend * BALLOON_LEAD_S > 0.0 {
            // A BALLOON (or one BALLOON_LEAD_S away: the SF50's 0.3 was held
            // while the sink vanished at +400 fpm a second, and let go only
            // once it was climbing, which was already a balloon to 52 ft;
            // EDLP 24, 2026-09-24): the pull has beaten the ground and the aircraft is
            // climbing. The ratchet exists for a sink that is being arrested,
            // and held through a climb it kept the Citation going up, 28 to
            // 58 ft at +545 fpm with the speed bleeding away (EDDV 27L,
            // 2026-09-22). The pull eases toward neutral and never becomes a
            // push, so the aircraft settles; the ratchet takes over again
            // from there as soon as it sinks.
            let eased = want.max(0.0);
            self.held = match g.balloon_ease_per_s {
                // Eased, not dropped: neutral is the TRIMMED approach
                // attitude, and at a flare's speed that sinks at once. The
                // SF50's 0.35 went to 0 in one tick at +190 fpm and the sink
                // was back at -590 two seconds later (EDLP 24, 2026-09-24).
                Some(rate) => eased.max(self.held - rate * dt),
                None => eased,
            };
            return self.held;
        }
        self.held = self.held.max(want);
        self.held
    }

    /// The law with this airframe's numbers.
    pub fn with_gains(gains: FlareGains) -> Self {
        Self {
            gains,
            ..Self::default()
        }
    }

    /// Let go — the channel expires on its own within a second, but neither
    /// the integrator nor the ratchet may carry a standing pull into the
    /// next approach. The airframe's numbers stay.
    pub fn reset(&mut self) {
        *self = Self::with_gains(self.gains);
    }
}

/// The flare flown on a MODEL of the airframe's elevator, for an airframe
/// that declares one (`flare_elevator.model`,
/// [`crate::aircraft::FlareModelProfile`]).
///
/// The P+D tracker above treats the sink as something the elevator
/// accelerates: pull and it keeps arresting. The 737's LEMG flares of
/// 2026-09-26 say a held pull HOLDS a sink, arriving after a dead time, and
/// that the aircraft pitches down by itself over the last forty feet (the
/// tail's ground effect): at LEMG 12 the sink sat at -300 from 37 to 16 ft
/// with 0.02 to 0.07 of pull and grew to -416 at the wheels while the pull
/// crept to 0.13, too late by the dead time.
///
/// So the law is a prediction, not a correction. Each tick it runs the
/// model forward `horizon_s` from now: the elevator already written arrives
/// on schedule, the ground term grows as the predicted height falls, and a
/// single new elevator is held from the moment it can arrive. It writes the
/// elevator that puts the MODEL's sink, at the horizon, on the flare's
/// target for the height the aircraft will have then. What the model does
/// not explain (the trim the autopilot left, the thrust, the speed, a gust)
/// is measured every tick as the sink less the model's own answer
/// (`unexplained`) and carried through the prediction unchanged, so a
/// wrong trim at the gate (the LEMG 12 approach arrived at -774 on a trim
/// that held -470) is learned a dead time later instead of assumed. The
/// dead time is met with LEAD, the future written into the ask, never with
/// gain. Never a push past the airframe's `max_push`, never a pull past its
/// `max`.
///
/// IN ROUGH AIR THE COLUMN ONLY COMES BACK in the flare (the Flare phase;
/// the roundout above it stays free both ways): the prediction's ask is the
/// least pull written, never less than the pull already held. Free all the
/// way down, the model chased the air mass: LEMG 13, 2026-09-28 (sortie
/// 1790589309, the final's sink straying 72 fpm rms from the one asked for
/// against 10 to 23 on the ten calm finals before it), a downdraft of about
/// 130 fpm over 30 to 20 ft had it pull 0.39 at 25 ft, the air came back,
/// the unexplained sink swung up with it, and the prediction eased the
/// column 0.39 to 0.08 by 16 ft while still sinking 415 to 246; the ease
/// arrived a dead time later in a -69 fpm FLOAT at 15 ft, the sink rebuilt
/// faster than full pull could come back, and the wheels met the runway at
/// -422 fpm. The gust and a strong elevator look the same for the seconds
/// that matter, and in SMOOTH air the free prediction is what lands a strong
/// one: through the 17:25 flare of 2026-09-26 (6,429 fpm per unit) in still
/// air, the column held from the flare gate ballooned four of the eight
/// recorded gates and dropped the worst in at 506, where the prediction
/// landed all eight under 305. So smooth air
/// keeps the prediction and holds only a near-level float
/// ([`FLARE_HOLD_MIN_SINK_FPM`]); rough air ([`FinalAir`]) holds from the
/// flare gate. Either way a float is not a balloon: only a measured CLIMB
/// (past [`BALLOON_CLIMB_FPM`], never a +1 fpm float) lets the held pull go, eased at the airframe's `balloon_ease_per_s`
/// toward the ask, never a push, and more pull comes whenever the prediction
/// of the sink at the horizon asks for it.
#[derive(Debug, Clone)]
pub struct FlareModel {
    pub(super) cfg: crate::aircraft::FlareModelProfile,
    pub(super) max: f64,
    pub(super) min: f64,
    pub(super) tau_s: f64,
    /// In a climb the held pull eases toward the ask at most this fast
    /// (elevator per second); `None` lets go at once.
    pub(super) balloon_ease_per_s: Option<f64>,
    /// The pull the flare holds (the ratchet), `None` above the flare.
    pub(super) held: Option<f64>,
    /// Model time, seconds, and every elevator written with its time, for
    /// the dead time.
    pub(super) t: f64,
    pub(super) written: std::collections::VecDeque<(f64, f64)>,
    /// The model's answer to the elevator and the ground (two lag states).
    pub(super) b1: f64,
    pub(super) b2: f64,
    /// The elevator the model believes is on the channel.
    pub(super) el: f64,
    /// The sink the model does not explain, smoothed (a RATE plant: the
    /// change of sink it does not explain, fpm per second).
    pub(super) unexplained: Option<f64>,
    /// A rate plant's last measured sink and the model's own rate over the
    /// last advance, for the rate the model does not explain.
    pub(super) last_vvi: Option<f64>,
    pub(super) model_rate: f64,
    /// Seconds since the last write (a failed read coasts).
    pub(super) coasting_s: f64,
    /// The airspeed last read and its smoothed trend (kt/s), for a model
    /// that carries the speed (`fpm_per_kt`).
    pub(super) last_ias: Option<f64>,
    pub(super) ias_trend: f64,
    /// A climb past this eases the held pull ([`BALLOON_CLIMB_FPM`]; a
    /// replay of the law as flown before issue #146 sets it to zero).
    pub(super) climb_fpm: f64,
}

/// One tick of [`FlareModel`], for the blackbox as much as the channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlareModelOut {
    /// The elevator to write.
    pub elev: f64,
    /// The sink the law asked for at the horizon, fpm.
    pub target_fpm: f64,
    /// The height predicted at the horizon, ft.
    pub ahead_ft: f64,
    /// The sink the model does not explain, fpm (a rate plant: fpm per
    /// second).
    pub unexplained_fpm: f64,
    /// The elevator the prediction asked for, before the flare's ratchet.
    pub ask: f64,
}

/// The model's own integration step, seconds.
pub(super) const FLARE_MODEL_STEP_S: f64 = 0.02;
/// The prediction's step, seconds.
pub(super) const FLARE_PREDICT_STEP_S: f64 = 0.05;
/// The airspeed trend a speed-carrying model leads with is smoothed over
/// this, seconds.
pub(super) const FLARE_SPEED_TREND_TAU_S: f64 = 1.0;
/// The pitch channel expires this long after its last write.
pub(super) const FLARE_CHANNEL_EXPIRY_S: f64 = 1.0;

impl FlareModel {
    /// The law for this airframe's model, pull cap, push cap (negative) and
    /// flare time constant.
    pub fn new(cfg: crate::aircraft::FlareModelProfile, max: f64, min: f64, tau_s: f64) -> Self {
        // A STANDING plant never takes the column forward of neutral in the
        // roundout or the flare: the arrest leaves the sink shallower than
        // the exponential asks, the model pushed to catch it up, and the
        // nose-down it stored arrived after its own lag, at the wheels.
        // King Air LEMG 13 (iMac 2026-09-30): no push, 93 kt and 299 fpm
        // (1790783744); pushed from 48 and 61 ft at -0.05, 103 and 109 kt,
        // 349 and 367 fpm, both skipped (1790783945, 1790784149). The
        // Evolution at LILB pushed from 52 ft at 361 fpm, the sink built to
        // 576 and it landed at 570 (1790781114, #196). A float is the
        // retard's (`FlarePower`) and the commit's, never the column's. Only
        // a RATE plant, whose neutral HOLDS a sink, pushes (`step_rate`).
        let min = if cfg.rate { min } else { min.max(0.0) };
        Self {
            cfg,
            max,
            min,
            tau_s: if tau_s.is_finite() && tau_s > 0.5 {
                tau_s
            } else {
                5.0
            },
            balloon_ease_per_s: None,
            held: None,
            t: 0.0,
            written: std::collections::VecDeque::new(),
            b1: 0.0,
            b2: 0.0,
            el: 0.0,
            unexplained: None,
            last_vvi: None,
            model_rate: 0.0,
            coasting_s: 0.0,
            last_ias: None,
            ias_trend: 0.0,
            climb_fpm: BALLOON_CLIMB_FPM,
        }
    }

    /// A climb in the flare eases the held pull at most this fast (elevator
    /// per second; the profile's `balloon_ease_per_s`).
    pub fn with_balloon_ease(mut self, per_s: Option<f64>) -> Self {
        self.balloon_ease_per_s = per_s.filter(|r| r.is_finite() && *r > 0.0);
        self
    }

    /// From the airframe's `flare_elevator:`; `None` unless it declares a
    /// model (every other airframe keeps [`FlarePitch`]).
    pub fn from_profile(p: Option<crate::aircraft::FlareElevatorProfile>) -> Option<Self> {
        let p = p.and_then(|p| p.valid())?;
        let gains = FlareGains::from_profile(Some(p));
        p.model.map(|m| {
            Self::new(m, gains.max, gains.min, p.tau_s.unwrap_or(5.0))
                .with_balloon_ease(gains.balloon_ease_per_s)
        })
    }

    /// A new approach: nothing written, nothing owed, nothing held.
    pub fn reset(&mut self) {
        let climb_fpm = self.climb_fpm;
        *self = Self::new(self.cfg, self.max, self.min, self.tau_s)
            .with_balloon_ease(self.balloon_ease_per_s);
        self.climb_fpm = climb_fpm;
    }

    /// The law as flown before issue #146: any climb, +1 fpm included,
    /// eased the held pull. Replays only.
    #[cfg(test)]
    pub(super) fn easing_on_any_climb(mut self) -> Self {
        self.climb_fpm = 0.0;
        self
    }

    pub(super) fn lag2(&self) -> f64 {
        self.cfg.lag2_s.unwrap_or(self.cfg.lag_s)
    }

    pub(super) fn horizon(&self) -> f64 {
        self.cfg.horizon_s.unwrap_or(self.cfg.dead_s + 1.0)
    }

    /// The sink the ground adds at this height, fpm (positive).
    pub(super) fn ground(&self, h: f64) -> f64 {
        self.cfg.ground_fpm * (1.0 - h.max(0.0) / self.cfg.ground_ft).max(0.0)
    }

    /// The elevator arriving at the aircraft at model time `t`: the last
    /// one written a dead time before it.
    pub(super) fn arriving(&self, t: f64) -> f64 {
        let due = t - self.cfg.dead_s;
        self.written
            .iter()
            .take_while(|(tw, _)| *tw <= due + 1e-9)
            .last()
            .map_or(0.0, |(_, e)| *e)
    }

    /// Advance the model `dt` at height `h`.
    pub(super) fn advance(&mut self, dt: f64, h: f64) {
        let n = (dt / FLARE_MODEL_STEP_S).round().max(1.0) as usize;
        let step = dt / n as f64;
        let (k, l1, l2) = (self.cfg.fpm_per_elevator, self.cfg.lag_s, self.lag2());
        let ground = self.ground(h);
        let rate = self.cfg.rate;
        let mut rate_sum = 0.0;
        for _ in 0..n {
            self.t += step;
            self.written.push_back((self.t, self.el));
            // Keep one entry at or before the dead-time boundary.
            let due = self.t - self.cfg.dead_s;
            while self.written.len() > 1 && self.written[1].0 <= due {
                self.written.pop_front();
            }
            let arriving = self.arriving(self.t);
            if rate {
                // The lags carry the RATE the column asks; the ground's
                // own rate is taken off it, not lagged.
                self.b1 += (k * arriving - self.b1) * step / l1;
                self.b2 += (self.b1 - self.b2) * step / l2;
                rate_sum += self.b2 - ground;
            } else {
                self.b1 += (k * arriving - ground - self.b1) * step / l1;
                self.b2 += (self.b1 - self.b2) * step / l2;
            }
        }
        if rate {
            self.model_rate = rate_sum / n as f64;
        }
    }

    /// The airspeed this tick, for a model that carries the speed
    /// (`fpm_per_kt`): its trend is smoothed over [`FLARE_SPEED_TREND_TAU_S`].
    /// Nothing else reads it.
    pub fn note_speed(&mut self, ias_kts: f64, dt_s: f64) {
        if !ias_kts.is_finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            return;
        }
        if let Some(p) = self.last_ias {
            let raw = ((ias_kts - p) / dt_s).clamp(-6.0, 3.0);
            self.ias_trend += (raw - self.ias_trend) * (dt_s / FLARE_SPEED_TREND_TAU_S).min(1.0);
        }
        self.last_ias = Some(ias_kts);
    }

    /// The sink the speed's trend adds over the next `ahead_s` (fpm,
    /// negative while it bleeds); nothing without `fpm_per_kt`.
    pub(super) fn speed_sink_ahead(&self, ahead_s: f64) -> f64 {
        self.cfg
            .fpm_per_kt
            .map_or(0.0, |k| k * self.ias_trend.min(0.0) * ahead_s.max(0.0))
    }

    /// The model's sink and height at the horizon if `el_new` were held
    /// from now, the elevator already written arriving on schedule.
    pub(super) fn predict(&self, el_new: f64, radio_ft: f64, vvi_fpm: f64, unexplained: f64) -> (f64, f64) {
        let horizon = self.horizon();
        let n = (horizon / FLARE_PREDICT_STEP_S).round().max(1.0) as usize;
        let dt = horizon / n as f64;
        let (k, l1, l2) = (self.cfg.fpm_per_elevator, self.cfg.lag_s, self.lag2());
        let (mut b1, mut b2, mut t, mut h, mut v) = (self.b1, self.b2, self.t, radio_ft, vvi_fpm);
        for _ in 0..n {
            t += dt;
            let arriving = if t - self.cfg.dead_s <= self.t {
                self.arriving(t)
            } else {
                el_new
            };
            if self.cfg.rate {
                // From the sink measured now: the column's rate, less the
                // ground's, plus the rate nothing explains.
                b1 += (k * arriving - b1) * dt / l1;
                b2 += (b1 - b2) * dt / l2;
                v += (b2 - self.ground(h) + unexplained) * dt;
            } else {
                b1 += (k * arriving - self.ground(h) - b1) * dt / l1;
                b2 += (b1 - b2) * dt / l2;
                // The speed still to bleed, as sink, from now to then.
                v = unexplained + b2 + self.speed_sink_ahead(t - self.t);
            }
            h = (h + v / 60.0 * dt).max(0.0);
        }
        (v, h)
    }

    /// No measurement this tick: nothing is decided, the model keeps time
    /// with what is on the channel, and the channel itself runs out after
    /// [`FLARE_CHANNEL_EXPIRY_S`].
    pub fn coast(&mut self, dt_s: f64, radio_ft: f64) {
        let dt = dt_s.clamp(0.02, 3.0);
        self.advance(dt, radio_ft);
        self.coasting_s += dt;
        // A rate plant's next change of sink spans the gap: no measurement.
        self.last_vvi = None;
        if self.coasting_s >= FLARE_CHANNEL_EXPIRY_S {
            self.el = 0.0;
        }
    }

    /// The flare's target at a height and groundspeed: the exponential
    /// flare to the touchdown sink, never steeper than the path. Pure.
    pub fn target_fpm(&self, radio_ft: f64, gs_kts: f64) -> f64 {
        let nominal = -(gs_kts.max(0.0) / 60.0) * gp_ft_per_nm();
        let (_, arrest) = arrest_targets_fpm(nominal);
        let touchdown = self.cfg.touchdown_fpm.map_or(arrest, |t| -t);
        // A HOLD-OFF levels at its height, then settles: the same
        // exponential, measured from the hold-off height, not the runway.
        let above = match self.cfg.hold_off_ft {
            Some(hold) => (radio_ft - hold).max(0.0),
            None => radio_ft,
        };
        flare_target_with(above, touchdown, nominal, Some(self.tau_s))
    }

    /// The airframe flies a light single's hold-off (`hold_off_ft`).
    pub fn holds_off(&self) -> bool {
        self.cfg.hold_off_ft.is_some()
    }

    /// One tick on a RATE plant (`FlareModelProfile::rate`, the A330's
    /// sidestick): the elevator whose rate, held from when it can arrive,
    /// puts the sink at the horizon on the flare's target for the height it
    /// will have then. The rate the model does not explain (the speed
    /// bleeding off at idle, the trim the autopilot left) is measured from
    /// the sink's own change and carried through the prediction. Neutral
    /// HOLDS a sink on this plant, so there is no ratchet and no "never
    /// push a climb": the column comes forward to end a float or a balloon,
    /// as far as the airframe's `max_push`. The model's pipe carries what
    /// was written, as on the standing plant.
    pub(super) fn step_rate(&mut self, radio_ft: f64, gs_kts: f64, vvi_fpm: f64, dt: f64) -> FlareModelOut {
        let unexplained = match self.last_vvi {
            Some(prev) => {
                let now = (vvi_fpm - prev) / dt - self.model_rate;
                match self.unexplained {
                    Some(u) => u + (now - u) * (dt / self.cfg.observer_s).min(1.0),
                    None => now,
                }
            }
            None => self.unexplained.unwrap_or(0.0),
        };
        self.last_vvi = Some(vvi_fpm);
        self.unexplained = Some(unexplained);
        let (v0, ahead) = self.predict(0.0, radio_ft, vvi_fpm, unexplained);
        let (v1, _) = self.predict(1.0, radio_ft, vvi_fpm, unexplained);
        let target = self.target_fpm(ahead, gs_kts);
        let per_unit = v1 - v0;
        let ask = if per_unit > 1e-6 {
            ((target - v0) / per_unit).clamp(self.min, self.max)
        } else {
            0.0
        };
        self.held = None;
        self.el = ask;
        FlareModelOut {
            elev: ask,
            target_fpm: target,
            ahead_ft: ahead,
            unexplained_fpm: unexplained,
            ask,
        }
    }

    /// One tick: the elevator for this measured sink. `flare` is the flare
    /// proper (as against the roundout above it, which stays free both
    /// ways); `rough` is the final's air ([`FinalAir::rough`]). See
    /// [`FlareModel`] for what each holds.
    pub fn step(
        &mut self,
        radio_ft: f64,
        gs_kts: f64,
        vvi_fpm: f64,
        dt_s: f64,
        flare: bool,
        rough: bool,
    ) -> FlareModelOut {
        let dt = dt_s.clamp(0.02, 3.0);
        self.advance(dt, radio_ft);
        self.coasting_s = 0.0;
        if self.cfg.rate {
            return self.step_rate(radio_ft, gs_kts, vvi_fpm, dt);
        }
        let now = vvi_fpm - self.b2;
        let unexplained = match self.unexplained {
            Some(u) => u + (now - u) * (dt / self.cfg.observer_s).min(1.0),
            None => now,
        };
        self.unexplained = Some(unexplained);
        // Linear in the new elevator: two predictions give the answer.
        let (v0, ahead) = self.predict(0.0, radio_ft, vvi_fpm, unexplained);
        let (v1, _) = self.predict(1.0, radio_ft, vvi_fpm, unexplained);
        let target = self.target_fpm(ahead, gs_kts);
        let per_unit = v1 - v0;
        let ask = if per_unit > 1e-6 {
            ((target - v0) / per_unit).clamp(self.min, self.max)
        } else {
            0.0
        };
        let elev = if flare {
            // The roundout's last ask is where the flare starts.
            let held = self.held.unwrap_or(ask);
            let next = if vvi_fpm > self.climb_fpm {
                // A BALLOON, measured: the pull has beaten the ground and the
                // aircraft climbs. Eased toward the ask, never a push. A
                // near-level float is not one: +1 fpm at 22.9 ft eased the
                // held 0.28 to 0.20 at EGBB 15 (2026-09-28), a dead time
                // before the sink the retard would rebuild.
                let eased = ask.max(0.0);
                match self.balloon_ease_per_s {
                    Some(rate) => eased.max(held - rate * dt),
                    None => eased,
                }
            } else if (rough && self.cfg.rough_hold != Some(false))
                || vvi_fpm > -FLARE_HOLD_MIN_SINK_FPM
            {
                // Rough air, or a near-level float: the pull stays, and more
                // comes only when the prediction asks for it.
                ask.max(held)
            } else {
                // Smooth air and a real sink: the prediction flies it.
                ask
            };
            self.held = Some(next);
            next
        } else {
            self.held = None;
            // The roundout is free both ways, except that a measured climb
            // is a balloon and a balloon never pushes (LEMG 12, the SF50,
            // 2026-09-28: -0.05 at +214 fpm).
            if vvi_fpm > self.climb_fpm {
                ask.max(0.0)
            } else {
                ask
            }
        };
        // The model's pipe carries what was WRITTEN, the held pull included.
        self.el = elev;
        FlareModelOut {
            elev,
            target_fpm: target,
            ahead_ft: ahead,
            unexplained_fpm: unexplained,
            ask,
        }
    }
}

/// The final's air, as the autopilot felt it: how far the sink strays from
/// the one asked for (fpm rms about its own mean, so a steady tracking
/// offset is not turbulence), from [`FINAL_AIR_FROM_FT`] down to the arrest
/// gate, only while the autopilot is confirmed. The ten calm LEMG 13 finals
/// of 2026-09-28 strayed 10 to 23 fpm; the three gusty ones after them 72,
/// 83 and 105, and the 2026-09-26 finals in 8 to 12 kt of swinging wind 29
/// to 110. Too few samples (a close-in reposition, failed reads) is no
/// measurement, and no measurement is smooth air: the flare the calm
/// landings flew.
#[derive(Debug, Default, Clone, Copy)]
pub struct FinalAir {
    pub(super) n: u32,
    pub(super) sum: f64,
    pub(super) sum_sq: f64,
}

/// The final's air is measured from this radio height down, ft.
pub const FINAL_AIR_FROM_FT: f64 = 400.0;
/// ...needs this many samples (about five seconds of ticks)...
pub(super) const FINAL_AIR_MIN_SAMPLES: u32 = 15;
/// ...and is rough past this, fpm rms.
pub const ROUGH_AIR_FPM: f64 = 40.0;

impl FinalAir {
    /// One tick's sink against the one asked for; a read that failed is no
    /// sample.
    pub fn note(&mut self, vvi_fpm: f64, asked_fpm: f64) {
        let e = vvi_fpm - asked_fpm;
        if e.is_finite() {
            self.n += 1;
            self.sum += e;
            self.sum_sq += e * e;
        }
    }

    /// The stray, fpm rms about its mean; `None` with too few samples.
    pub fn rms_fpm(&self) -> Option<f64> {
        if self.n < FINAL_AIR_MIN_SAMPLES {
            return None;
        }
        let n = f64::from(self.n);
        let mean = self.sum / n;
        Some((self.sum_sq / n - mean * mean).max(0.0).sqrt())
    }

    pub fn rough(&self) -> bool {
        self.rms_fpm().is_some_and(|r| r > ROUGH_AIR_FPM)
    }
}
