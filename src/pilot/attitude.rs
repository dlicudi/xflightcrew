//! The path and the flare flown by ATTITUDE, judged by HEIGHT
//! (docs/tasks/autonomy-layers, phase 2). Handed a [`Path`] it holds that
//! path from wherever the hand flying begins (minimums, 2026-10-12),
//! rejoining it in proportion, and flares from it: one law from minimums
//! to the wheels.
//!
//! The G1000 172 on LEMG 13 (2026-10-11, 1791739518) gave the measurements
//! it rests on:
//! - the flight path is the nose's attitude less the angle of attack, read
//!   with no lag: 0.9 deg nose down at 1.8 deg of alpha is a 2.7 deg path,
//!   -300 fpm at 63 kt, where the true vertical velocity read -296 and the
//!   VSI still -177;
//! - the nose answers the elevator in 0.2 to 0.4 s;
//! - the old law flew the VSI, a second late: it pulled the 172 into a
//!   balloon, let the nose fall from 4.3 to -1 deg and touched at 463 fpm.
//!
//! So: hold the path the aircraft is on until the flare height, then shrink
//! the sink in proportion to the height left down to the touchdown sink
//! (the flare TIME, ArduPlane's LAND_FLARE_SEC, PX4's FW_LND_FL_TIME:
//! [`RESPONSE_S`]). The wanted path becomes a nose attitude (the path plus a
//! slowly filtered angle of attack), limited by the airframe's GEOMETRY
//! ([`PitchLimits`]: the tail strike, the mains first), and the elevator
//! holds that attitude through the pitch response MEASURED for this airframe
//! ([`crate::pilot::response`], scaled by the air): its gains by the
//! crossover rule, the command's rate fed forward, on top of what a
//! disturbance observer reads holding the nose takes (ground effect, the
//! bleeding speed).

use super::response::{design, Response};

/// Seconds the aircraft needs to change its flight path: the nose's answer
/// (0.2 to 0.4 s measured) plus the path following it (0.7 to 1.5 s), with
/// the margin a weak elevator needs (2 s landed a steep arrival at 300 fpm
/// on half the authority, `attitude_flare_tests`).
pub const RESPONSE_S: f64 = 3.0;
/// The hold-off: the sink the flare eases to by [`HOLDOFF_FT`] and holds
/// to the wheels, fpm. The flare is a round-out THEN a hold-off: the wheels
/// kept just off, the power at idle, the nose rising as the speed bleeds
/// until the aircraft settles on its mains. Easing to -100 at the wheels, the
/// 172 touched flat (1.5 to 1.7 deg nose up) at 62 kt off a 66.5 kt final:
/// a flare that looked weak (LEMG 13, 1791755237, 1791755825).
pub const HOLDOFF_VS_FPM: f64 = -70.0;
/// The height the round-out ends and the hold-off begins, ft.
pub const HOLDOFF_FT: f64 = 2.0;
/// Never flare lower than this, ft (a very shallow approach).
const MIN_GATE_FT: f64 = 6.0;
/// The disturbance observer (the elevator the aircraft needs to hold its
/// nose: ground effect, the bleeding speed, read from what it holds and how
/// the nose answers) is this many times slower than the attitude loop's
/// crossover. Closed through the loop it is an attitude gain of its own,
/// `(ff + k_rate) / tau` a degree against the loop's designed `angle_w (ff
/// + k_rate)`: at this separation never more than half of it. A fixed 1 s
/// was faster than the 737-800's attitude loop (0.88 rad/s through its
/// 0.175 s), doubled its gain unseen by the design, and the flare rang
/// stop to stop (EGBB 15, 1792092651); on the light aircraft's 1.1 to 1.7
/// rad/s the rule asks 1.2 to 1.7 s, near the 1 s they landed on.
const DOB_SEPARATION: f64 = 2.0;
/// The elevator, at most either way: all of it. The NOSE is limited
/// ([`PitchLimits`]), never the column.
const ELEV_MAX: f64 = 1.0;

/// The pitch attitudes the hand-flown segment may ask for, degrees: the
/// airframe's geometry, not its elevator (`aircraft::GroundGeometry`, from
/// the `.acf`'s gear and fuselage). The 737's declared elevator limit of no
/// push held its nose up from minimums, the sink fell 950 to 410 fpm and the
/// gate sent it round high (LEMG 13, 1791838756).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PitchLimits {
    /// Never above this anywhere in the segment: the tail strike's pitch
    /// on the extended mains, less a margin.
    pub max_deg: f64,
    /// Never below this once the round-out has begun: the pitch at which
    /// the nosewheel would meet the runway with the mains, plus a margin
    /// (the mains first). `None`: a tailwheel.
    pub touchdown_min_deg: Option<f64>,
    /// The elevator's full travel, stop to stop, in seconds (the `.acf`'s
    /// `_elev_def_time`; zero: it follows at once): the column is never
    /// asked to move faster than the surface can. The 737-800's flare asked
    /// its 0.3 s surface for steps of up to 0.98 a tick (EGBB 15,
    /// 1792092651): a surface lagging its column is lag the loop was not
    /// designed for.
    pub elev_full_travel_s: f64,
}

/// The elevator's full travel time when the `.acf` was not read, s: the
/// slower of the stock fleet's declared surfaces (the 737-800's and the
/// King Air's), so an unread airframe's column never outruns them.
pub const ELEV_FULL_TRAVEL_PRIOR_S: f64 = 0.3;

impl Default for PitchLimits {
    /// Unread geometry: no tail strike below 10 degrees on anything flown
    /// here (the 737-800's is 11 on extended struts, its `.acf` says), and
    /// the mains first at a level nose.
    fn default() -> Self {
        PitchLimits { max_deg: 10.0, touchdown_min_deg: Some(0.0), elev_full_travel_s: ELEV_FULL_TRAVEL_PRIOR_S }
    }
}

/// The mains-first floor's slope above the hold-off, degrees a foot.
const FLOOR_DEG_PER_FT: f64 = 1.0;
/// The command's own rate fed forward, at most, deg/s: a nose that turns
/// faster than this in a flare is a step, not a flare.
const THETA_RATE_MAX_DPS: f64 = 5.0;

/// The angle of attack is filtered this slowly, seconds: the nose's own
/// short-period swing in alpha must not feed back into the attitude asked.
const ALPHA_TAU_S: f64 = 1.5;
/// The path error is trimmed out of the attitude this fast, deg per second
/// per deg of error: settled inside a third of [`RESPONSE_S`]. At 0.5 the
/// trim, a type-1 loop facing the ramp of a bleeding speed and the power
/// coming off (alpha rising 0.4 to 1.0 deg/s in the last 10 ft on five
/// airframes' fitted plants, 2026-10-12), left the path 0.8 to 2 deg under
/// its command: the windy evening's 202 to 274 fpm.
const GAMMA_TRIM_PER_S: f64 = 1.0;
/// ...and only while the nose is within this of its command, degrees (the
/// bank hold's trim band, `pilot::bank`): on the 737-800's slow path the
/// trim wound up through the round-out on a nose still catching up and
/// ballooned the flare (EGBB 15, 1792092651).
const TRIM_BAND_DEG: f64 = 1.0;
/// Knots to feet a minute.
const FPM_PER_KT: f64 = 101.268_591;

/// One tick's measurements. Attitudes in degrees, the pitch rate in deg/s.
#[derive(Debug, Clone, Copy)]
pub struct In {
    pub radio_ft: f64,
    pub theta_deg: f64,
    pub alpha_deg: f64,
    pub q_dps: f64,
    pub ias_kts: f64,
    pub tas_kts: f64,
    pub dt: f64,
    /// The path to hold above the flare, when there is one (the final's
    /// glidepath); `None` holds the path the first tick finds.
    pub path: Option<Path>,
}

/// A path to hold: its own sink at the groundspeed (fpm, negative down) and
/// how far above it the aircraft is (ft).
#[derive(Debug, Clone, Copy)]
pub struct Path {
    pub vs_fpm: f64,
    pub above_ft: f64,
}

/// A height off the path is closed over this long, s: a pilot's glidepath
/// correction, a dot in fifteen seconds, never a dive.
const REJOIN_S: f64 = 15.0;
/// ...asking at most this much off the path's own sink, fpm.
const REJOIN_MAX_FPM: f64 = 300.0;
/// ...and none of it below this height, ft (all of it from twice this): a
/// pilot stops chasing the glidepath for the flare, and past the threshold
/// the glidepath's height means nothing.
const REJOIN_TO_FT: f64 = 50.0;

/// The sink wanted above the flare on a path: its own, plus the height off
/// it closed over [`REJOIN_S`], faded out by [`REJOIN_TO_FT`], never a
/// climb. Pure.
pub fn path_vs_fpm(path: Path, radio_ft: f64) -> f64 {
    let fade = ((radio_ft - REJOIN_TO_FT) / REJOIN_TO_FT).clamp(0.0, 1.0);
    let rejoin = (-path.above_ft * 60.0 / REJOIN_S).clamp(-REJOIN_MAX_FPM, REJOIN_MAX_FPM);
    (path.vs_fpm + fade * rejoin).min(HOLDOFF_VS_FPM)
}

/// What the law asked for this tick (the blackbox's `att_*`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Out {
    pub elev: f64,
    pub gate_ft: f64,
    pub vs_target_fpm: f64,
    pub gamma_deg: f64,
    pub gamma_cmd_deg: f64,
    pub theta_cmd_deg: f64,
    /// The seconds to the wheels the flare plans from here: the height over
    /// the sink wanted or flown, whichever is quicker (the TECS flare's
    /// retard and the decrab are timed on it).
    pub to_wheels_s: f64,
    /// Below the flare gate: the round-out and the hold-off.
    pub flaring: bool,
    /// Elevator per degree of attitude error, from the measured response
    /// (`att_k`).
    pub k_theta: f64,
}

/// The law's memory for one approach: reset by a go-around.
#[derive(Debug, Clone, Copy, Default)]
pub struct AttitudeFlare {
    entry: Option<Entry>,
    alpha_f: f64,
    gamma_trim: f64,
    integ: f64,
    last_elev: f64,
    /// What the last command asked beyond what the surface could give
    /// (its rate, its stops): positive, nose up undelivered.
    short: f64,
    last_theta_cmd: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
struct Entry {
    vs_fpm: f64,
    gate_ft: f64,
}

/// The flare height for a sink, ft: the sink times the response time. Pure.
pub fn gate_ft(vs_fpm: f64, response_s: f64) -> f64 {
    (vs_fpm.abs() / 60.0 * response_s).max(MIN_GATE_FT)
}

/// The sink wanted at this height: the entry's path above the gate, then in
/// proportion to the height left down to the touchdown sink. Pure.
///
/// A SMOOTHSTEP in height, not a line: its slope is zero at the gate and at
/// the hold-off, so the rate fed forward never steps. As a line, the slope
/// cut to zero at 2 ft, the pull dropped 0.30 to 0.19 in one tick, the nose
/// pitched down at 1.7 deg/s and the full 0.35 could not lift it again
/// before the wheels: touched at 3.2 deg nose up against a command of 3.9
/// (LEMG 13, 1791759560).
pub fn vs_target_fpm(radio_ft: f64, entry_vs_fpm: f64, gate_ft: f64) -> f64 {
    let entry = entry_vs_fpm.min(HOLDOFF_VS_FPM);
    HOLDOFF_VS_FPM + (entry - HOLDOFF_VS_FPM) * smoothstep(rounding_out_k(radio_ft, gate_ft))
}

/// How far up the round-out the height is: 0 at the hold-off, 1 at the gate.
fn rounding_out_k(radio_ft: f64, gate_ft: f64) -> f64 {
    ((radio_ft - HOLDOFF_FT) / (gate_ft - HOLDOFF_FT).max(1.0)).clamp(0.0, 1.0)
}

/// The seconds to the wheels the flare PLANS from `radio_ft`: the path's
/// sink down to the gate, then the round-out and hold-off's own profile
/// ([`vs_target_fpm`]), integrated. The height over the sink flown is a
/// third of it at the gate (the flare stretches the time), and a decrab or
/// retard timed on it began 8 s early on the plants. Pure.
pub fn planned_s(radio_ft: f64, path_vs_fpm: f64, entry_vs_fpm: f64, gate_ft: f64) -> f64 {
    let h = radio_ft.max(0.0);
    let above = (h - gate_ft).max(0.0) / (-path_vs_fpm).max(60.0) * 60.0;
    let top = h.min(gate_ft);
    const N: usize = 40;
    let dh = top / N as f64;
    let flare: f64 = (0..N)
        .map(|k| {
            let hm = (k as f64 + 0.5) * dh;
            dh / (-vs_target_fpm(hm, entry_vs_fpm, gate_ft)).max(30.0) * 60.0
        })
        .sum();
    above + flare
}

fn smoothstep(k: f64) -> f64 {
    k * k * (3.0 - 2.0 * k)
}

impl AttitudeFlare {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// One tick: the elevator (yoke ratio, positive aft), flown on the
    /// pitch response `resp` (the airframe's, any airspeed: scaled here),
    /// the nose within `limits`. The first tick takes the path it finds as
    /// the one to hold.
    pub fn step(&mut self, i: In, resp: &Response, limits: PitchLimits) -> Out {
        let dt = i.dt.clamp(0.02, 0.5);
        let r = resp.at(i.ias_kts, i.tas_kts);
        let g = design(&r, dt);
        let v_fpm = i.tas_kts.max(30.0) * FPM_PER_KT;
        let gamma = i.theta_deg - i.alpha_deg;
        let mut entry = *self.entry.get_or_insert_with(|| {
            self.alpha_f = i.alpha_deg;
            let vs = i.path.map_or(v_fpm * gamma.to_radians().sin(), |p| p.vs_fpm);
            Entry { vs_fpm: vs, gate_ft: gate_ft(vs, RESPONSE_S) }
        });
        // On a path the flare starts from the path's sink as it is when the
        // gate is reached (the speed moves it on the way down).
        if let Some(p) = i.path.filter(|_| i.radio_ft > entry.gate_ft) {
            entry = Entry { vs_fpm: p.vs_fpm, gate_ft: gate_ft(p.vs_fpm, RESPONSE_S) };
            self.entry = Some(entry);
        }
        let flaring = i.radio_ft <= entry.gate_ft;
        let vs_t = match i.path {
            Some(p) if !flaring => path_vs_fpm(p, i.radio_ft),
            _ => vs_target_fpm(i.radio_ft, entry.vs_fpm, entry.gate_ft),
        };
        let gamma_cmd = (vs_t / v_fpm).clamp(-1.0, 1.0).asin().to_degrees();
        // The target's slope in height (the smoothstep's, zero at both
        // ends) times the height's own rate: led, not chased (a lag chain
        // tracking a ramp arrives late by its own lag).
        let k = rounding_out_k(i.radio_ft, entry.gate_ft);
        let dvs_dh = (entry.vs_fpm.min(HOLDOFF_VS_FPM) - HOLDOFF_VS_FPM) * 6.0 * k * (1.0 - k)
            / (entry.gate_ft - HOLDOFF_FT).max(1.0);
        let gamma_rate = (dvs_dh * (v_fpm * gamma.to_radians().sin() / 60.0) / v_fpm).to_degrees();
        // The path follows the nose over 1 / the measured path answer: the
        // lead is the angle of attack that bends the path at that rate.
        let path_lag_s = if r.path > 0.1 { (1.0 / r.path).min(2.5) } else { 1.0 };
        let lead = path_lag_s * gamma_rate;
        // The filter is for the alpha that HOLDS the path, so the lead's own
        // alpha is taken out of what it reads: read whole, the round-out's
        // alpha came back through the filter as a second lead, and on the
        // 737-800's 1.5 s path (its filter no slower than the path) the
        // nose ran 1.4 to 3 degrees past its command (EGPH 24, 1792158539).
        self.alpha_f += (i.alpha_deg - lead - self.alpha_f) * (dt / ALPHA_TAU_S).min(1.0);
        // The filtered alpha holds the path on the attitude; what the filter
        // misses in a flare (the speed bleeding, the power coming off) only
        // ever asks for more nose, so the trim never goes below zero: a gust
        // that shallowed the path wound it down, and its unwinding pushed
        // the SR22 to -7.8 deg and the runway at 441 fpm (LEMG 13,
        // 1791813026), the Evolution's -0.71 deg into its sag (1791815852).
        // It integrates only while the nose holds its command: a path still
        // following a nose that is still turning is not a path error. Nor
        // while the elevator could not give what it was asked the same way:
        // more trim is more column the surface cannot deliver.
        // Never faster than the path answers: an integrator quicker than the
        // lag it closes through winds up on the path still following. The
        // light aircraft's paths answer at 1.2 to 2.8 /s and keep the 1 /s;
        // the 737-800's 0.66 /s, trimmed at 1, wound 1.6 degrees into the
        // round-out and ballooned (EGPH 24, 1792158539).
        let trim_per_s = GAMMA_TRIM_PER_S.min(1.0 / path_lag_s);
        let err = gamma_cmd - gamma;
        if self.last_theta_cmd.is_none_or(|c| (c - i.theta_deg).abs() < TRIM_BAND_DEG) && err * self.short <= 0.0 {
            self.gamma_trim = (self.gamma_trim + err * trim_per_s * dt).clamp(0.0, 3.0);
        }
        // The mains-first floor is for the wheels: it rises toward them a
        // degree a foot from the hold-off, never a step into the round-out
        // (a level floor met at the gate stepped the Evolution's nose 4.6
        // degrees and rang it on its plant).
        let floor = match limits.touchdown_min_deg {
            Some(m) if flaring => m - (i.radio_ft - HOLDOFF_FT).max(0.0) * FLOOR_DEG_PER_FT,
            _ => f64::MIN,
        };
        let theta_cmd = (gamma_cmd + self.alpha_f + self.gamma_trim + lead)
            .min(limits.max_deg)
            .max(floor.min(limits.max_deg));
        let e_theta = theta_cmd - i.theta_deg;
        // The command's own rate (the speed bleeding, alpha rising, the
        // path easing): fed forward as the elevator that turns the nose at
        // that rate, so it rises WITH the command. Chased, it lagged 0.8 deg
        // below 8 ft and the 172 landed at 185 to 199 fpm on a command of
        // -110 (LEMG 13, 1791755237, 1791755825).
        let theta_rate = self
            .last_theta_cmd
            .map_or(0.0, |p| (theta_cmd - p) / dt)
            .clamp(-THETA_RATE_MAX_DPS, THETA_RATE_MAX_DPS);
        self.last_theta_cmd = Some(theta_cmd);
        // What holding the nose takes: the elevator held (as delivered:
        // `last_elev` is what the surface could give), less what the
        // nose's rate says it is doing beyond holding (at the measured rate
        // a unit holds). Ground effect and the bleeding speed show up here
        // within [`DOB_SEPARATION`] attitude loops (1.2 to 2.3 s on the
        // fleet); a slow trim could not find them in the flare's three (226
        // to 287 fpm, LEMG 2026-10-11).
        let need = self.last_elev - i.q_dps * g.ff;
        let tau = DOB_SEPARATION / g.angle_w;
        self.integ = (self.integ + (need - self.integ) * (dt / tau).min(1.0)).clamp(-ELEV_MAX, ELEV_MAX);
        // The rate wanted: the attitude error at the attitude loop's
        // crossover, plus the command's own rate; flown by its feed-forward
        // and the rate loop's gain.
        let q_c = g.angle_w * e_theta + theta_rate;
        // Never faster than the surface moves: a column it cannot follow
        // is lag the loop was not designed for.
        let asked = self.integ + q_c * g.ff + g.k_rate * (q_c - i.q_dps);
        let slew = if limits.elev_full_travel_s > 0.0 { 2.0 / limits.elev_full_travel_s * dt } else { f64::INFINITY };
        let elev = asked.clamp(self.last_elev - slew, self.last_elev + slew).clamp(-ELEV_MAX, ELEV_MAX);
        self.short = asked - elev;
        self.last_elev = elev;
        let path_sink = i.path.map_or(entry.vs_fpm, |p| p.vs_fpm);
        Out {
            elev,
            gate_ft: entry.gate_ft,
            vs_target_fpm: vs_t,
            gamma_deg: gamma,
            gamma_cmd_deg: gamma_cmd,
            theta_cmd_deg: theta_cmd,
            to_wheels_s: planned_s(i.radio_ft, path_sink, entry.vs_fpm, entry.gate_ft),
            flaring,
            k_theta: g.angle_w * (g.ff + g.k_rate),
        }
    }
}
