//! The ground-track laws the takeoff roll, autoland's rollout and arrest
//! share: the centreline cut (`roll_target_heading`, `XtkRate`, `RollCut`,
//! `DriftFilter`), the roll steering law (`RollSteer`) and the wings-level
//! ailerons (`WingsLevel`). Moved out of `autotakeoff.rs` verbatim (refactor
//! audit 2026-10-04, phase 2); `autotakeoff` re-exports it, so every
//! `autotakeoff::` path still compiles. Items the takeoff or its tests
//! still reach are `pub(crate)`.

use crate::autotakeoff::{
    AIL_FULL_BANK_DEG,
    AIL_LEAD_S,
    AIL_MAX,
    AIL_SLEW_PER_S,
    LOW_SPEED_FULL_KTS,
    LOW_SPEED_STEER_CAP,
    MAX_ROLL_STEER,
    POWER_LEAD_FADE_KTS,
    POWER_LEAD_MAX,
    ROLL_FULL_DEG,
    RUDDER_CLOSE_PER_S,
    RUDDER_TRIM_MAX,
    RUDDER_TRIM_PER_S,
    STEER_AUTH_FLOOR,
    STEER_LEAD_S,
    STEER_SLEW_PER_S,
    STEER_TRIM_ERR_CLAMP_DEG,
    STEER_TRIM_MAX,
    STEER_TRIM_RATE,
    TRIM_MIN_GS_KTS,
    TRIM_SCALE_FROM_KTS,
    XTK_GAIN_DEG_PER_M,
    XTK_GAIN_REF_KTS,
    XTK_LEAD_S,
    XTK_MAX_DEG,
};

/// The heading to hold on the roll: runway course with a bounded cut back
/// toward the centreline (`xtk_m` right-positive, same convention as
/// autoland's final; `xtk_rate_m_s` right-positive too).
///
/// THE CUT IS SIZED FOR A CLOSURE TIME, NOT AN ANGLE PER METRE. A fixed
/// 0.4 deg/m closes an offset in `1 / (V * gain)` seconds, so the faster the
/// aircraft the tighter the outer loop winds - at 40 kt that is 7 seconds and
/// the loop is placid, at 150 kt it is under 2 and sits on top of the inner
/// loop's own lag. The pair then oscillates, and it did at both ends of the
/// same sortie (EGLF 24 / LFPB 03, 2026-09-22): the takeoff wandered
/// +1.7, -4.6, +10.1 m and LIFTED OFF ten metres right of the centreline,
/// and the landing rollout wove +3.1 / -5.7 m through four degrees of heading
/// before it damped. Scaling the gain by `ref/gs` above [`XTK_GAIN_REF_KTS`]
/// holds the closure time at the placid figure for every speed; below that
/// pace nothing changes, so the low-speed behaviour the takeoff was tuned
/// around (the sortie-three pirouette) is untouched.
///
/// [`XTK_LEAD_S`] is the other half: the cut is taken against where the
/// offset WILL be, not where it is. Autoland's final learned this years
/// earlier ("P alone converged asymptotically and S-ed under wind") and the
/// roll never got it.
pub fn roll_target_heading(course_deg: f64, xtk_m: f64, xtk_rate_m_s: f64, gs_kts: f64) -> f64 {
    (course_deg - roll_cut_deg(xtk_m, xtk_rate_m_s, gs_kts)).rem_euclid(360.0)
}

/// The cut [`roll_target_heading`] takes off the course, degrees (right of
/// the centreline positive, so the target moves left).
pub(crate) fn roll_cut_deg(xtk_m: f64, xtk_rate_m_s: f64, gs_kts: f64) -> f64 {
    let offset = if xtk_m.is_finite() { xtk_m } else { 0.0 };
    let rate = if xtk_rate_m_s.is_finite() {
        xtk_rate_m_s
    } else {
        0.0
    };
    // A missing or absurd groundspeed reads as the reference pace, which is
    // the gain this has always had.
    let pace = if gs_kts.is_finite() {
        gs_kts.abs().max(XTK_GAIN_REF_KTS)
    } else {
        XTK_GAIN_REF_KTS
    };
    let gain = XTK_GAIN_DEG_PER_M * (XTK_GAIN_REF_KTS / pace);
    ((offset + rate * XTK_LEAD_S) * gain).clamp(-XTK_MAX_DEG, XTK_MAX_DEG)
}

/// The cross-track RATE the centreline law leads with, smoothed. Raw, it is
/// a position differenced across a jittery tick: 0.8, 4.1, 5.4 and 7.4 m/s in
/// neighbouring ticks of a 737 roll whose real drift was about 1 m/s (LEMG,
/// 2026-09-26), and `XTK_LEAD_S` of lead turns each spike into degrees of
/// target heading, so the rudder flicked side to side down the runway. A
/// reading past [`XTK_RATE_MAX_M_S`] is noise, not a movement; the rest is
/// averaged over [`XTK_RATE_TAU_S`].
#[derive(Debug, Default, Clone, Copy)]
pub struct XtkRate {
    value: Option<f64>,
}

impl XtkRate {
    pub fn update(&mut self, raw_m_s: f64, dt_s: f64) -> f64 {
        if !raw_m_s.is_finite() || !dt_s.is_finite() || dt_s <= 0.0 {
            return self.value.unwrap_or(0.0);
        }
        let raw = raw_m_s.clamp(-XTK_RATE_MAX_M_S, XTK_RATE_MAX_M_S);
        let a = (dt_s / (XTK_RATE_TAU_S + dt_s)).clamp(0.0, 1.0);
        let v = match self.value {
            Some(v) => v + (raw - v) * a,
            None => raw * a,
        };
        self.value = Some(v);
        v
    }
}

/// The aircraft's own DRIFT on the roll: its track across the runway less
/// its heading, the angle a heading held dead on course still carries it
/// sideways by. The cut in [`roll_target_heading`] is proportional, and its
/// gain falls as 1/v to hold the closure time, so a steady drift is held off
/// only by a standing offset that grows with speed: drift / gain, 1.6 m at
/// 40 kt and 6 m at 150 for two thirds of a degree.
///
/// EGBB 33 (1790534902, 2026-09-27): about 3 kt of crosswind from the left,
/// the scheduled rudder law holding the heading within 0.3 degrees of its
/// target the whole roll, and the aircraft drifting right at 0.35 to 0.45
/// m/s from 95 kt: track less heading a steady 0.6 to 0.7 degrees, 0.9
/// once rotated. The cut had reached 0.5 degrees at 4 m; the wheels left
/// the runway 5.7 m right of the centreline.
///
/// Fed forward, like the final's wind crab (`autoland::CrabFilter`): the
/// heading target less this puts the TRACK where the cut asks. Read from
/// the cross-track rate the cut already uses (`asin(rate / v)` is the track
/// across the runway) less the heading off the runway smoothed by the SAME
/// [`XTK_RATE_TAU_S`], so a nose swing reaches both sides together and is
/// not read as drift, then low-passed over [`DRIFT_TAU_S`] and bounded.
/// Nothing below [`DRIFT_FROM_KTS`], where a centimetre of position jitter
/// is a degree of track.
///
/// Where the tyres make the track follow the heading late (`roll_rudder:`
/// `track_lag_s`, [`DriftFilter::with_track_lag`]) the heading is delayed by
/// that lag as well: the track read is the heading that much late, and
/// against the heading smoothed by the rate's second alone a swing of the
/// nose read as drift the other way. The Zibo's LEMG 30 roll 1791388851
/// (2026-10-07) swung 2.4 degrees right from 106 kt, the track behind it,
/// and the drift read -0.6: the target held 0.6 degrees right while the
/// aircraft ran right across the line into the rotation.
#[derive(Debug, Default, Clone, Copy)]
pub struct DriftFilter {
    /// Heading less course, smoothed like the cross-track rate.
    heading: Option<f64>,
    est: Option<f64>,
    /// The track's lag behind the heading, seconds (0: none declared).
    track_lag_s: f64,
}

/// The drift is read from this groundspeed (the cut's own reference pace).
pub const DRIFT_FROM_KTS: f64 = 40.0;
/// Its smoothing, seconds: three times the rate's, so the rate's spikes
/// (1.3 m of position in one EGBB tick at 50 kt) come through at a tenth.
pub const DRIFT_TAU_S: f64 = 4.0;
/// A raw reading past this is a swerve, not a drift.
pub(crate) const DRIFT_RAW_MAX_DEG: f64 = 3.0;
/// The most drift the target may carry.
pub const DRIFT_MAX_DEG: f64 = 2.0;

impl DriftFilter {
    /// The filter for an airframe whose track lags its heading by `lag_s`.
    pub fn with_track_lag(lag_s: f64) -> Self {
        Self {
            track_lag_s: if lag_s.is_finite() { lag_s.max(0.0) } else { 0.0 },
            ..Self::default()
        }
    }

    /// One tick: heading less runway course (degrees, right positive), the
    /// smoothed cross-track rate ([`XtkRate`], m/s right positive),
    /// groundspeed and the tick. Returns the drift to take off the target,
    /// degrees (track right of heading positive).
    pub fn update(&mut self, psi_off_deg: f64, rate_m_s: f64, gs_kts: f64, dt_s: f64) -> f64 {
        if !psi_off_deg.is_finite() || !rate_m_s.is_finite() || !gs_kts.is_finite() {
            return self.value();
        }
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return self.value();
        }
        let a = (dt_s / (XTK_RATE_TAU_S + self.track_lag_s + dt_s)).clamp(0.0, 1.0);
        let heading = match self.heading {
            Some(h) => h + (psi_off_deg - h) * a,
            None => psi_off_deg,
        };
        self.heading = Some(heading);
        if gs_kts < DRIFT_FROM_KTS {
            return self.value();
        }
        let v = gs_kts * 0.514_444;
        let track = (rate_m_s / v).clamp(-1.0, 1.0).asin().to_degrees();
        let raw = (track - heading).clamp(-DRIFT_RAW_MAX_DEG, DRIFT_RAW_MAX_DEG);
        let b = (dt_s / (DRIFT_TAU_S + dt_s)).clamp(0.0, 1.0);
        let e = self.est.unwrap_or(0.0);
        self.est = Some((e + (raw - e) * b).clamp(-DRIFT_MAX_DEG, DRIFT_MAX_DEG));
        self.value()
    }

    /// The current estimate; zero before it has read anything.
    pub fn value(&self) -> f64 {
        self.est.unwrap_or(0.0)
    }
}

/// The fastest a cross-track reading may move and still be believed, m/s.
pub const XTK_RATE_MAX_M_S: f64 = 5.0;
/// The smoothing of the cross-track rate, seconds.
pub const XTK_RATE_TAU_S: f64 = 1.0;

/// On a scheduled roll (`roll_rudder:`) at the declared rudder's speed and
/// over, the cut is this many times [`roll_target_heading`]'s, blended in
/// from [`XTK_GAIN_REF_KTS`] (see [`roll_fast_blend`]): a lateral closure of
/// about 3.5 s where the cut alone holds 7.
pub const ROLL_CUT_FAST_MULT: f64 = 2.0;
/// The cross-track trim: degrees of cut per metre-second of offset at
/// [`XTK_GAIN_REF_KTS`], scaled by ref/gs like the cut, from
/// [`DRIFT_FROM_KTS`]...
pub const XTK_TRIM_DEG_PER_M_S: f64 = 0.06;
/// ...and the most it may cut at that pace, degrees (less as the speed rises).
pub const XTK_TRIM_MAX_DEG: f64 = 1.0;
/// Where the airframe declares its track lag, the cut's lead goes over from
/// the rate read to the heading's own rate over this many knots from
/// [`DRIFT_FROM_KTS`], where the drift that keeps the heading's rate true
/// begins to be read.
pub(crate) const HEADING_LEAD_BLEND_KTS: f64 = 20.0;
/// Ceiling on the scheduled law's gain from the declared rudder's speed:
/// never more than full steer per this many degrees (the default law's
/// [`ROLL_FULL_DEG`] below [`XTK_GAIN_REF_KTS`], blended between).
pub(crate) const RUDDER_FULL_DEG_FAST: f64 = 8.0;

/// How far a scheduled roll is into holding the line harder: 0 below
/// [`XTK_GAIN_REF_KTS`] (the pace the cut is quoted at, and nothing below it
/// changes), 1 from the speed the airframe's rudder was measured at
/// (`roll_rudder:` `at_kts`, 80 on the 737), linear between.
pub(crate) fn roll_fast_blend(gs_kts: f64, rudder_at_kts: f64) -> f64 {
    if !gs_kts.is_finite() {
        return 0.0;
    }
    let span = rudder_at_kts - XTK_GAIN_REF_KTS;
    if span.is_nan() || span <= 1.0 {
        return if gs_kts >= XTK_GAIN_REF_KTS { 1.0 } else { 0.0 };
    }
    ((gs_kts - XTK_GAIN_REF_KTS) / span).clamp(0.0, 1.0)
}

/// One tick of the roll's centreline cut, as [`RollCut::update`] computes it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CutTick {
    /// The cross-track the cut was taken on, metres right (the read, or on a
    /// scheduled roll the median of the last three reads).
    pub xtk_m: f64,
    /// The smoothed cross-track rate read, m/s right.
    pub rate_m_s: f64,
    /// The rate the cut leads with, m/s right: the rate read, or where the
    /// airframe declares its track lag the rate its heading is making
    /// ([`RollCut`]).
    pub lead_m_s: f64,
    /// The drift fed forward ([`DriftFilter`]), degrees; zero on the default law.
    pub drift_deg: f64,
    /// The proportional cut, degrees (right of the line positive).
    pub cut_deg: f64,
    /// The cross-track trim's cut, degrees; zero on the default law.
    pub cut_i_deg: f64,
    /// The heading target less the runway course, degrees.
    pub target_off_deg: f64,
}

/// The centreline half of the takeoff roll: from the cross-track read and
/// the heading off the runway to the heading target, the one path the roll
/// loop and its replays share.
///
/// Every airframe gets the cut it flew ([`roll_target_heading`], plus the
/// [`DriftFilter`] on a `roll_rudder:` airframe). A SCHEDULED roll adds
/// three things, each read off the 737 rolls of 2026-09-28 (EGKK 08R lifted
/// off 5.4 m left, LEMG 13 1790591256 9.0 m left at 150 kt):
///
/// - The cross-track is the MEDIAN of the last three reads. Latitude and
///   longitude arrive in separate requests, so a read can pair two frames:
///   on LEMG's diagonal runway that is up to 2.9 m of cross-track for one
///   tick above 120 kt (0.7 at EGKK's east-west one), and each spike is
///   metres of target and a flick of rudder. A single-tick spike never
///   passes a median; a real movement is late by one tick. The source is
///   fixed since (#157: the plugin's `xcopilot/position`, one frame in one
///   request); the median stays as the defence for an older plugin's split
///   reads and any other single-tick glitch.
/// - From [`XTK_GAIN_REF_KTS`] the cut grows to [`ROLL_CUT_FAST_MULT`] times
///   the closure-time cut: the heading loop under it now holds the line, and
///   a 7 s lateral closure let a yaw gust at 100 kt carry the aircraft 4 m.
/// - A slow trim on the cross-track itself ([`XTK_TRIM_DEG_PER_M_S`], capped
///   at [`XTK_TRIM_MAX_DEG`]): whatever the drift feed and the heading trim
///   leave standing, it takes out by rotation.
///
/// Where the airframe declares how far its track lags its heading on the
/// wheels (`roll_rudder:` `track_lag_s`), the cut leads on the rate the
/// HEADING is making, the heading plus the drift (blended in over
/// [`HEADING_LEAD_BLEND_KTS`] from [`DRIFT_FROM_KTS`]), not on the
/// cross-track rate read: that rate is the heading's answer that lag late,
/// so a cut led on it sees its own correction late, holds the nose across
/// the line and crosses it still moving. The drift (read slowly from the
/// same rate) keeps the lead true over seconds; the heading makes it early.
/// The Zibo's track follows its heading through 0.5 to 1.0 s at 70 to 110
/// KIAS, 1 to 2 s toward Vr and 1.5 s or more once the nose is up (the five
/// LEMG 30 rolls of 2026-10-07): the four that drifted crossed the
/// centreline at the rotation 0.9 to 2.2 m/s right, nothing after the
/// rotation turned the track back in time, and they lifted off 5.3 to 11.1 m
/// right.
#[derive(Debug, Default, Clone)]
pub struct RollCut {
    /// The declared rudder's speed on a scheduled roll; `None` on the default law.
    rudder_at_kts: Option<f64>,
    /// The declared track lag (`roll_rudder:` `track_lag_s`): the cut then
    /// leads on the heading's rate.
    track_lag_s: Option<f64>,
    /// The cut as flown on 2026-09-28, for the replays that pin the change.
    #[cfg(test)]
    as_flown: bool,
    recent: std::collections::VecDeque<f64>,
    prev_xtk: Option<f64>,
    rate: XtkRate,
    drift: DriftFilter,
    /// Metre-seconds of cross-track, for the trim.
    integral: f64,
}

impl RollCut {
    /// The cut for a roll on the law scheduled on a rudder measured at
    /// `rudder_at_kts`, or (`None`) on the default law.
    pub fn new(rudder_at_kts: Option<f64>) -> Self {
        Self {
            rudder_at_kts,
            ..Self::default()
        }
    }

    /// The cut for the roll this steering law flies.
    pub fn for_law(law: &RollSteer) -> Self {
        let cut = Self::new(law.rudder.map(|r| r.at_kts));
        match law.rudder.and_then(|r| r.track_lag_s) {
            Some(lag) => cut.with_track_lag(lag),
            None => cut,
        }
    }

    /// The cut for an airframe whose track lags its heading by `lag_s` on
    /// the wheels (`roll_rudder:` `track_lag_s`).
    pub fn with_track_lag(mut self, lag_s: f64) -> Self {
        self.track_lag_s = Some(lag_s);
        self.drift = DriftFilter::with_track_lag(lag_s);
        self
    }

    #[cfg(test)]
    pub(crate) fn as_flown(mut self) -> Self {
        self.as_flown = true;
        self
    }

    fn improved(&self) -> bool {
        #[cfg(test)]
        if self.as_flown {
            return false;
        }
        self.rudder_at_kts.is_some()
    }

    /// One tick: the cross-track read (metres right), the heading less the
    /// runway course (degrees right), groundspeed and the tick.
    pub fn update(&mut self, xtk_read_m: f64, psi_off_deg: f64, gs_kts: f64, dt_s: f64) -> CutTick {
        let improved = self.improved();
        let xtk = if improved && xtk_read_m.is_finite() {
            self.recent.push_back(xtk_read_m);
            while self.recent.len() > 3 {
                self.recent.pop_front();
            }
            if self.recent.len() == 3 {
                let mut v: Vec<f64> = self.recent.iter().copied().collect();
                v.sort_by(|a, b| a.total_cmp(b));
                v[1]
            } else {
                xtk_read_m
            }
        } else {
            xtk_read_m
        };
        let raw = match self.prev_xtk {
            Some(p) if dt_s > 0.0 => (xtk - p) / dt_s,
            _ => 0.0,
        };
        self.prev_xtk = Some(xtk);
        let rate = self.rate.update(raw, dt_s);
        let drift = if self.rudder_at_kts.is_some() {
            self.drift.update(psi_off_deg, rate, gs_kts, dt_s)
        } else {
            0.0
        };
        let lead = match self.track_lag_s {
            Some(_) if improved && psi_off_deg.is_finite() && gs_kts.is_finite() => {
                let v = gs_kts.abs() * 0.514_444;
                let heading_rate = v * (psi_off_deg + drift).to_radians().sin();
                let share = ((gs_kts - DRIFT_FROM_KTS) / HEADING_LEAD_BLEND_KTS).clamp(0.0, 1.0);
                rate + (heading_rate - rate) * share
            }
            _ => rate,
        };
        let mut cut = roll_cut_deg(xtk, lead, gs_kts);
        let mut cut_i = 0.0;
        if improved {
            let at = self.rudder_at_kts.unwrap_or(XTK_GAIN_REF_KTS);
            let mult = 1.0 + (ROLL_CUT_FAST_MULT - 1.0) * roll_fast_blend(gs_kts, at);
            cut = (cut * mult).clamp(-XTK_MAX_DEG, XTK_MAX_DEG);
            if gs_kts >= DRIFT_FROM_KTS && xtk.is_finite() && dt_s.is_finite() && dt_s > 0.0 {
                let lim = XTK_TRIM_MAX_DEG / XTK_TRIM_DEG_PER_M_S;
                self.integral = (self.integral + xtk * dt_s).clamp(-lim, lim);
            }
            let pace = if gs_kts.is_finite() {
                gs_kts.abs().max(XTK_GAIN_REF_KTS)
            } else {
                XTK_GAIN_REF_KTS
            };
            cut_i = XTK_TRIM_DEG_PER_M_S * self.integral * (XTK_GAIN_REF_KTS / pace);
        }
        CutTick {
            xtk_m: xtk,
            rate_m_s: rate,
            lead_m_s: lead,
            drift_deg: drift,
            cut_deg: cut,
            cut_i_deg: cut_i,
            target_off_deg: -(cut + cut_i) - drift,
        }
    }
}

/// Slew limit: move from `prev` toward `want` by at most `max_step`.
pub fn slew(prev: f64, want: f64, max_step: f64) -> f64 {
    prev + (want - prev).clamp(-max_step, max_step)
}

/// The roll steering law: P on the error, a lead term on its rate (damps
/// the counter-steer oscillation of stale observations), and a slow
/// integrator that LEARNS the standing rudder the yaw moment demands —
/// P-factor at full power needs rudder held with the nose already
/// straight, which no proportional term can do without a standing error.
/// Output is capped, speed-tapered (gently — the taper's old floor
/// starved the law at rotate speed), and slew-limited.
#[derive(Default)]
pub struct RollSteer {
    pub(crate) trim: f64,
    pub(crate) prev_err: Option<f64>,
    /// Groundspeed at the last tick, for the trim's speed scaling.
    pub(crate) prev_gs: Option<f64>,
    pub(crate) out: f64,
    /// No propeller, no P-factor to lead: a jet takes no power lead
    /// (`for_airframe`). The default is the propeller law.
    pub(crate) jet: bool,
    /// The airframe's measured rudder, when its profile declares one: the
    /// law is then scheduled on it (see [`RollSteer::step_with_yaw`]).
    pub(crate) rudder: Option<crate::aircraft::RollRudderProfile>,
    /// The engine torque last read, ft-lb, for a lead declared on it
    /// (`roll_rudder:` `power_steer_per_kftlb`); `None` leads on the lever.
    pub(crate) torque_ftlb: Option<f64>,
    /// The steer written inside a declared dead time (`roll_rudder:`
    /// `dead_s`), oldest first: (age s, steer, seconds it was held).
    pub(crate) pipe: std::collections::VecDeque<(f64, f64, f64)>,
    /// The newest write past the dead time...
    pub(crate) arrived: f64,
    /// ...and the steer the yaw read answers, that one through the lag.
    pub(crate) seen: f64,
    /// The schedule as flown on 2026-09-28, for the replays that pin the change.
    #[cfg(test)]
    pub(crate) as_flown: bool,
    /// The dead-time predictor as flown until 2026-10-07: the declared rudder
    /// extrapolated past its speed.
    #[cfg(test)]
    pub(crate) predictor_past_fit: bool,
}

/// The rudder led in WITH the power: proportional to the throttle
/// actually commanded, fading partially as airflow builds.
pub fn power_lead(thr: f64, gs_kts: f64) -> f64 {
    POWER_LEAD_MAX * thr * (1.0 - 0.6 * (gs_kts / POWER_LEAD_FADE_KTS).clamp(0.0, 1.0))
}

impl RollSteer {
    /// The law for the live airframe: a JET gets no power lead. The lead is
    /// right rudder against a propeller's left-turning tendency, and fed
    /// into the SF50's roll it WAS the swing: 0.1 to 0.25 of right rudder at
    /// full power, the heading error building to 18 degrees against a
    /// saturated command and the aircraft 60 m right of the centreline by
    /// 60 kt (LOWI 26, 2026-09-24).
    pub fn for_airframe() -> Self {
        let jet = crate::aircraft::live_profile().is_some_and(|p| {
            p.reference
                .as_ref()
                .and_then(|r| r.kind.as_deref())
                .is_some_and(|k| k.to_ascii_lowercase().contains("jet"))
        });
        Self {
            jet,
            rudder: crate::aircraft::roll_rudder(),
            ..Self::default()
        }
    }

    /// The law scheduled on a measured rudder (tests and replays).
    #[cfg(test)]
    pub fn with_rudder(jet: bool, rudder: crate::aircraft::RollRudderProfile) -> Self {
        Self {
            jet,
            rudder: rudder.valid(),
            ..Self::default()
        }
    }

    /// The torque read this tick (ft-lb; `None` on a failed read), for the
    /// standing rudder a turboprop declares on it. A failed read leads on
    /// the lever, as `power_steer` does.
    pub fn set_torque(&mut self, torque_ftlb: Option<f64>) {
        self.torque_ftlb = torque_ftlb.filter(|t| t.is_finite());
    }

    /// Whether this law is scheduled on a declared rudder.
    pub fn scheduled(&self) -> bool {
        self.rudder.is_some()
    }

    /// The schedule as flown on 2026-09-28 (the declared v² extrapolated to
    /// Vr, [`ROLL_FULL_DEG`] at every speed).
    #[cfg(test)]
    pub(crate) fn as_flown(mut self) -> Self {
        self.as_flown = true;
        self
    }

    /// The groundspeed the declared rudder is read at, and the degrees of
    /// error full steer is never asked for under, at this groundspeed.
    fn schedule(&self, rudder: crate::aircraft::RollRudderProfile, gs_kts: f64) -> (f64, f64) {
        #[cfg(test)]
        if self.as_flown {
            return (gs_kts, ROLL_FULL_DEG);
        }
        let blend = roll_fast_blend(gs_kts, rudder.at_kts);
        let full = ROLL_FULL_DEG + (RUDDER_FULL_DEG_FAST - ROLL_FULL_DEG) * blend;
        (gs_kts.min(rudder.at_kts), full)
    }

    /// One control tick: heading error (deg, right positive), the time
    /// since the last tick, groundspeed, and the throttle being
    /// commanded (for the power feed-forward). Returns the deflection.
    pub fn step(&mut self, err_deg: f64, dt_s: f64, gs_kts: f64, thr: f64) -> f64 {
        self.step_with_yaw(err_deg, dt_s, gs_kts, thr, None)
    }

    /// [`RollSteer::step`] with the sim's yaw rate (deg/s, nose right
    /// positive, `sim/flightmodel/position/R`) for the damping. The default
    /// law ignores it and keeps its differenced error, as flown.
    ///
    /// With a declared rudder the law is SCHEDULED on it. EGKK 26L
    /// (2026-09-27, twice): an 8 kt crosswind from the left needed about 0.23
    /// of right rudder from 40 kt on, and the default law gave it 0.03 of
    /// steer per degree of error over 50 kt (its authority floor, set for the
    /// 172's lively rudder) plus a trim learning 0.01 per degree-second that
    /// the 1/v² scaling bled away nearly as fast as it learned. The nose sat
    /// 5 degrees left for ten seconds and both rolls were rejected 10 m left
    /// of the centreline at 82 and 87 kt. Here every gain is a yaw rate
    /// divided by the yaw rate a unit of steer holds at this speed, so the
    /// heading closes at [`RUDDER_CLOSE_PER_S`] whatever the speed; the
    /// same lead is taken on the MEASURED yaw rate, not a differenced heading
    /// (the error's own difference carries every step of the target); the
    /// trim learns at [`RUDDER_TRIM_PER_S`] of the same gain and shrinks as 1/v
    /// with the speed, which is how a crosswind's steer falls (its moment
    /// grows as v, the rudder's as v²). Never more per degree than the
    /// default law's full authority, so the slow roll is the one it was.
    pub fn step_with_yaw(
        &mut self,
        err_deg: f64,
        dt_s: f64,
        gs_kts: f64,
        thr: f64,
        yaw_dps: Option<f64>,
    ) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        let rate = self.prev_err.map_or(0.0, |p| (err_deg - p) / dt);
        self.prev_err = Some(err_deg);
        if let Some(rudder) = self.rudder {
            // A failed read is no measurement: the error's own rate stands in.
            let yaw = yaw_dps.filter(|v| v.is_finite()).unwrap_or(-rate);
            return self.step_scheduled(rudder, err_deg, dt, gs_kts, thr, yaw);
        }
        let auth = 1.0 - (1.0 - STEER_AUTH_FLOOR) * ((gs_kts - 15.0) / 35.0).clamp(0.0, 1.0);
        let pd = ((err_deg + STEER_LEAD_S * rate) / ROLL_FULL_DEG).clamp(-1.0, 1.0) * auth;
        // Prop wash makes the tail HYPER-effective under power at low
        // speed: the cap is tiny parked and grows with the roll, so an
        // engage-time misalignment is walked out over the first hundred
        // meters instead of spot-turned through (the sortie-three pirouette).
        let cap = LOW_SPEED_STEER_CAP
            + (MAX_ROLL_STEER - LOW_SPEED_STEER_CAP)
                * (gs_kts / LOW_SPEED_FULL_KTS).clamp(0.0, 1.0);
        let lead = if self.jet { 0.0 } else { power_lead(thr, gs_kts) };
        let want = (pd + self.trim + lead).clamp(-cap, cap);
        // The trim integrates unconditionally ON THE ROLL — its own small
        // cap bounds wind-up, and a saturated command with a persisting
        // error is exactly when the standing rudder must build. Below
        // taxi pace it stays frozen: it exists for P-factor at speed, not
        // for doubling every low-speed overshoot.
        if gs_kts > TRIM_MIN_GS_KTS {
            let e = err_deg.clamp(-STEER_TRIM_ERR_CLAMP_DEG, STEER_TRIM_ERR_CLAMP_DEG);
            self.trim =
                (self.trim + e * dt * STEER_TRIM_RATE).clamp(-STEER_TRIM_MAX, STEER_TRIM_MAX);
        }
        // The trim is a yawing MOMENT held as a deflection, and the rudder's
        // moment grows with the square of the airspeed: the same deflection
        // that just held the line at 70 kt swings the aircraft at 110.
        // LIBP 04 (2026-09-26, 737): the trim built to 0.2 of right rudder
        // drifting left through 50-90 kt, the rudder came alive at 95 and
        // swung it through the centreline to 9.5 m right at 124 kt with the
        // heading error asking LEFT and the trim still outvoting it; the
        // excursion reject followed. So the trim shrinks as the speed rises.
        // (A fast bleed when the error turns against it was tried against
        // the plants in the tests and made every strong-disturbance roll
        // worse: the scaling alone is the fix.)
        if let Some(pg) = self.prev_gs.filter(|pg| *pg > TRIM_SCALE_FROM_KTS && gs_kts > *pg) {
            self.trim *= (pg / gs_kts).powi(2);
        }
        self.prev_gs = Some(gs_kts);
        self.out = slew(self.out, want, STEER_SLEW_PER_S * dt);
        self.out
    }

    fn step_scheduled(
        &mut self,
        rudder: crate::aircraft::RollRudderProfile,
        err_deg: f64,
        dt: f64,
        gs_kts: f64,
        thr: f64,
        yaw_dps: f64,
    ) -> f64 {
        // The declared rudder is believed up to the speed it was measured at
        // and not past it. It was fitted from 30 to 86 kt (EGKK 26L) as
        // growing with v²; the 737 rolls of 2026-09-28, fitted to Vr the same
        // way (three-second heading predictions from the logged steer, 0.3 s
        // dead time, 0.47 s lag), hold 4 to 6 deg/s per unit from 45 to
        // 110 kt and 8.5 at 132, and the 27th's rolls refitted to Vr 7 to
        // 12.6: about v^1.25, never v². Extrapolated, the schedule read 31
        // at 132 kt and cut the gain to a third: the nose sat a degree off
        // its target for thirty seconds from 100 kt at EGKK 08R, steer
        // creeping 0.03 to 0.09, and the aircraft lifted off 5.4 m left.
        // Held at the declared figure above its speed the heading closes in
        // about a second at Vr on either day's rudder.
        let (at_kts, full_deg) = self.schedule(rudder, gs_kts);
        let authority = rudder.yaw_dps_at(at_kts).max(1e-3);
        // Where the rudder is still dead the gain would be enormous: never
        // more per degree than the default law's full authority at a walk,
        // [`RUDDER_FULL_DEG_FAST`] once the roll is under way (from 60 kt the
        // declared rudder is already stronger than either day's measured one,
        // so the old ceiling was the gain there).
        //
        // The closure and the trim's corner are the airframe's where it
        // declares them (`roll_rudder:` `close_per_s`, `trim_per_s`): a need
        // the lead and the trim did not predict is held off at need / kp
        // degrees. The SR22's second LEMG 13 roll (1790682305, 2026-09-29)
        // needed 0.5 at 45 to 75 kt in 3 kt of crosswind and 0.2 by 88 kt;
        // at 0.032 per degree the nose sat 4 degrees left, then swung 5
        // right with the steer still right, 4.1 m left to 7.5 m right on
        // the wheels. Its lag is shorter than the 737's (0.3 s fitted), and
        // 1.5 per second with a corner of 0.45 holds both of its rolls.
        //
        // Where the airframe declares a dead time it can be flown round
        // (`roll_rudder:` `dead_s`), the loop steers on the heading the steer
        // already written will make once it has all arrived
        // ([`RollSteer::predicted`]); the trim still learns on the error read.
        let (err_now, yaw_now) = match rudder.dead_s {
            Some(dead) => self.predicted(rudder, err_deg, yaw_dps, gs_kts, dt, dead),
            None => (err_deg, yaw_dps),
        };
        let close = rudder.close_per_s.unwrap_or(RUDDER_CLOSE_PER_S);
        let kp = (close / authority).min(1.0 / full_deg);
        let pd = (kp * (err_now - STEER_LEAD_S * yaw_now)).clamp(-1.0, 1.0);
        let cap = LOW_SPEED_STEER_CAP
            + (MAX_ROLL_STEER - LOW_SPEED_STEER_CAP)
                * (gs_kts / LOW_SPEED_FULL_KTS).clamp(0.0, 1.0);
        let lead = if self.jet {
            0.0
        } else if let (Some(per), Some(tq)) = (rudder.power_steer_per_kftlb, self.torque_ftlb) {
            // On the torque, not the lever: the Evolution's torque limiter
            // cycled the lever 0.55 to 0.76 while the need followed the
            // torque, 0.51 per 1,000 ft-lb, flat to 78 kt; led on the lever
            // and faded past 60 kt it gave 0.21 at 78 kt against 0.65 needed
            // and the nose sat 5 to 8 degrees left (LILB 03 1790736651).
            (per * tq.max(0.0) / 1000.0 * rudder.power_steer_share(gs_kts)).min(MAX_ROLL_STEER)
        } else if let Some(ps) = rudder.power_steer {
            // Believed to the speed it was measured at, then falling as v^2:
            // held flat, 0.35 met a need of 0.2 at 88 kt.
            ps * thr.clamp(0.0, 1.0) * rudder.power_steer_share(gs_kts)
        } else {
            power_lead(thr, gs_kts)
        };
        let want = (pd + self.trim + lead).clamp(-cap, cap);
        if gs_kts > TRIM_MIN_GS_KTS {
            let e = err_deg.clamp(-STEER_TRIM_ERR_CLAMP_DEG, STEER_TRIM_ERR_CLAMP_DEG);
            let corner = rudder.trim_per_s.unwrap_or(RUDDER_TRIM_PER_S);
            self.trim =
                (self.trim + corner * kp * e * dt).clamp(-RUDDER_TRIM_MAX, RUDDER_TRIM_MAX);
        }
        if let Some(pg) = self
            .prev_gs
            .filter(|pg| *pg > TRIM_SCALE_FROM_KTS && gs_kts > *pg)
        {
            self.trim *= pg / gs_kts;
        }
        self.prev_gs = Some(gs_kts);
        self.out = slew(self.out, want, STEER_SLEW_PER_S * dt);
        if rudder.dead_s.is_some() {
            self.pipe.push_back((0.0, self.out, 0.0));
        }
        self.out
    }

    /// The heading error `dead` seconds from now and the yaw rate to lead
    /// on, from the steer already written. The heading: the yaw read holds
    /// for the dead time, and the steer it is still catching up with (the
    /// declared `lag_s`) and each write still in transit add the declared
    /// rudder's yaw for their difference from it, for as long as each was
    /// held. Only differences: a standing steer that a crosswind balances
    /// predicts nothing (the steer integrated whole would read a crosswind's
    /// rudder as a turn to come). The yaw: the reading plus the lag still to
    /// come on the steer that has ARRIVED; the writes in transit are in the
    /// heading already, and led on as well the law answered its own last
    /// write within the tick and alternated at the slew limit on the fitted
    /// plant past 90 kt.
    ///
    /// The Zibo's pedals reach the yaw through 1.0 s of dead time and a
    /// 0.2 s lag (LEMG 30, 2026-10-07). Steered on the reading, the error a
    /// second old, the scheduled law rang through the fitted plant 14.6 m off
    /// the centreline (the default law flown there wove 4 m right to 9.5 m
    /// left, and two rolls of three were rejected).
    fn predicted(
        &mut self,
        rudder: crate::aircraft::RollRudderProfile,
        err_deg: f64,
        yaw_dps: f64,
        gs_kts: f64,
        dt: f64,
        dead: f64,
    ) -> (f64, f64) {
        // The yaw read has answered the steer that was arriving through the
        // tick just ended, not one arriving now.
        let lag = rudder.lag_s.unwrap_or(0.0);
        self.seen = if lag > 0.0 {
            self.seen + (self.arrived - self.seen) * (1.0 - (-dt / lag).exp())
        } else {
            self.arrived
        };
        if let Some(last) = self.pipe.back_mut() {
            last.2 = dt;
        }
        for e in self.pipe.iter_mut() {
            e.0 += dt;
        }
        while self.pipe.front().is_some_and(|e| e.0 >= dead - 1e-9) {
            if let Some((_, s, _)) = self.pipe.pop_front() {
                self.arrived = s;
            }
        }
        // Believed to the speed it was measured at and not past it, as the
        // gain is (`schedule`). Extrapolated, the Zibo's v^0.75 read 17 to
        // 21 deg/s a unit from 110 kt to the liftoff, where its five LEMG 30
        // rolls of 2026-10-07 answered about its 80 kt figure until the
        // rotation and a third of it after: the law credited each write with
        // a turn it did not make, wrote 0.1 against 2 degrees of error and,
        // once rotated, alternated -0.5 and +0.3 (1791374430).
        #[cfg(test)]
        let k = if self.predictor_past_fit {
            rudder.yaw_dps_at(gs_kts)
        } else {
            rudder.yaw_dps_at(gs_kts.min(rudder.at_kts))
        };
        #[cfg(not(test))]
        let k = rudder.yaw_dps_at(gs_kts.min(rudder.at_kts));
        let in_transit: f64 = self
            .pipe
            .iter()
            .map(|(_, s, held)| (s - self.arrived) * held)
            .sum();
        let to_come = k * ((self.arrived - self.seen) * lag + in_transit);
        (
            err_deg - yaw_dps * dead - to_come,
            yaw_dps + k * (self.arrived - self.seen),
        )
    }

    /// The learned standing-rudder trim, for telemetry.
    pub fn trim(&self) -> f64 {
        self.trim
    }
}

/// The climb-out ailerons: P on bank error with a rate lead, slew-limited.
/// Bank positive = right wing down; aileron positive = roll right — the
/// same convention, so the error maps straight through.
#[derive(Default)]
pub struct WingsLevel {
    out: f64,
    prev_bank: Option<f64>,
}

impl WingsLevel {
    pub fn step(&mut self, bank_deg: f64, target_deg: f64, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(0.05, 1.0);
        let rate = self.prev_bank.map_or(0.0, |p| (bank_deg - p) / dt);
        self.prev_bank = Some(bank_deg);
        let err = target_deg - bank_deg - AIL_LEAD_S * rate;
        let want = (err / AIL_FULL_BANK_DEG).clamp(-1.0, 1.0) * AIL_MAX;
        self.out = slew(self.out, want, AIL_SLEW_PER_S * dt);
        self.out
    }
}
