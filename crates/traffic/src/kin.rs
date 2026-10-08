//! Kinematics for traffic: a point that moves the way an aircraft looks
//! like it moves. Pure, deterministic, flat earth around the aircraft; NOT
//! the crew's flight laws (those fly a real aeroplane through the sim; this
//! IS the aeroplane). Heading equals track in v1 (no wind triangle).

/// Latitude, longitude in degrees.
pub type LatLon = (f64, f64);

/// Metres in a degree of latitude: the same flat earth nav uses.
pub const M_PER_DEG_LAT: f64 = 111_320.0;

/// Knots to metres a second.
pub const KT_TO_MS: f64 = 1852.0 / 3600.0;

const G_MS2: f64 = 9.81;

/// Metres in a degree of longitude at `lat`.
pub fn m_per_deg_lon(lat: f64) -> f64 {
    M_PER_DEG_LAT * lat.to_radians().cos()
}

/// `to` relative to `from`, metres (east, north), flat earth at `from`.
pub fn offset_m(from: LatLon, to: LatLon) -> (f64, f64) {
    (
        (to.1 - from.1) * m_per_deg_lon(from.0),
        (to.0 - from.0) * M_PER_DEG_LAT,
    )
}

/// `p` moved `east`, `north` metres.
pub fn shift(p: LatLon, east: f64, north: f64) -> LatLon {
    (p.0 + north / M_PER_DEG_LAT, p.1 + east / m_per_deg_lon(p.0))
}

/// `p` moved `dist_m` along true bearing `brg_deg`.
pub fn project_m(p: LatLon, brg_deg: f64, dist_m: f64) -> LatLon {
    let (s, c) = brg_deg.to_radians().sin_cos();
    shift(p, dist_m * s, dist_m * c)
}

/// Flat-earth distance, metres.
pub fn dist_m(a: LatLon, b: LatLon) -> f64 {
    let (e, n) = offset_m(a, b);
    e.hypot(n)
}

/// Flat-earth bearing from `a` to `b`, degrees true 0..360.
pub fn bearing(a: LatLon, b: LatLon) -> f64 {
    let (e, n) = offset_m(a, b);
    e.atan2(n).to_degrees().rem_euclid(360.0)
}

/// `a - b` folded into -180..180.
pub fn signed_diff_deg(a: f64, b: f64) -> f64 {
    (a - b + 540.0).rem_euclid(360.0) - 180.0
}

/// The aircraft's exterior lights.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Lights {
    pub nav: bool,
    pub beacon: bool,
    pub strobe: bool,
    pub taxi: bool,
    pub landing: bool,
}

/// Where one traffic aircraft is and what it looks like.
#[derive(Debug, Clone, PartialEq)]
pub struct Kin {
    pub lat: f64,
    pub lon: f64,
    /// Feet MSL.
    pub alt_ft: f64,
    pub on_ground: bool,
    pub gs_kts: f64,
    /// Degrees true.
    pub hdg_deg: f64,
    /// Degrees true; equal to the heading in v1.
    pub track_deg: f64,
    pub vs_fpm: f64,
    /// Degrees, nose up positive.
    pub pitch: f64,
    /// Degrees, right wing down positive.
    pub roll: f64,
    /// 0 up, 1 down.
    pub gear: f64,
    /// 0 clean, 1 full.
    pub flaps: f64,
    pub lights: Lights,
}

impl Kin {
    /// Stopped on the ground at `pos`, facing `hdg_deg`, gear down.
    pub fn parked(pos: LatLon, elev_ft: f64, hdg_deg: f64) -> Kin {
        Kin {
            lat: pos.0,
            lon: pos.1,
            alt_ft: elev_ft,
            on_ground: true,
            gs_kts: 0.0,
            hdg_deg: hdg_deg.rem_euclid(360.0),
            track_deg: hdg_deg.rem_euclid(360.0),
            vs_fpm: 0.0,
            pitch: 0.0,
            roll: 0.0,
            gear: 1.0,
            flaps: 0.0,
            lights: Lights::default(),
        }
    }

    pub fn pos(&self) -> LatLon {
        (self.lat, self.lon)
    }

    /// One substep of motion: turn at `turn_dps`, move at the groundspeed,
    /// climb at the vertical speed. The attitude follows for the picture.
    pub fn integrate(&mut self, turn_dps: f64, dt: f64) {
        self.hdg_deg = (self.hdg_deg + turn_dps * dt).rem_euclid(360.0);
        self.track_deg = self.hdg_deg;
        let v = self.gs_kts.max(0.0) * KT_TO_MS;
        let p = project_m(self.pos(), self.hdg_deg, v * dt);
        self.lat = p.0;
        self.lon = p.1;
        if !self.on_ground {
            self.alt_ft += self.vs_fpm / 60.0 * dt;
        }
        if self.on_ground {
            self.roll = 0.0;
        } else {
            self.roll = (v * turn_dps.to_radians() / G_MS2).atan().to_degrees();
            let gamma = if v > 1.0 {
                (self.vs_fpm * 0.00508 / v).atan().to_degrees()
            } else {
                0.0
            };
            self.pitch = gamma + 3.0;
        }
    }

    /// Bring the groundspeed toward `kts` at no more than the given rates.
    pub fn speed_toward(&mut self, kts: f64, accel_kts_s: f64, decel_kts_s: f64, dt: f64) {
        let dv = (kts - self.gs_kts).clamp(-decel_kts_s * dt, accel_kts_s * dt);
        self.gs_kts = (self.gs_kts + dv).max(0.0);
    }

    /// Bring the vertical speed toward what reaches `target_ft`, clamped to
    /// `climb_fpm` up and `descent_fpm` down. Aims to close the gap in
    /// [`VS_TAU_S`]; the vertical speed itself changes at most
    /// [`VS_ACCEL_FPM_S`].
    pub fn vertical_toward(&mut self, target_ft: f64, climb_fpm: f64, descent_fpm: f64, dt: f64) {
        let want = ((target_ft - self.alt_ft) * 60.0 / VS_TAU_S).clamp(-descent_fpm, climb_fpm);
        self.vs_toward(want, dt);
    }

    /// Bring the vertical speed toward `fpm` at [`VS_ACCEL_FPM_S`].
    pub fn vs_toward(&mut self, fpm: f64, dt: f64) {
        let d = (fpm - self.vs_fpm).clamp(-VS_ACCEL_FPM_S * dt, VS_ACCEL_FPM_S * dt);
        self.vs_fpm += d;
    }
}

/// The vertical toward a target closes its gap over this many seconds.
pub const VS_TAU_S: f64 = 10.0;

/// The most the vertical speed changes in a second.
pub const VS_ACCEL_FPM_S: f64 = 400.0;

/// How [`follow`] drives along a path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowSpec {
    /// The speed to hold on a straight.
    pub kts: f64,
    pub max_turn_dps: f64,
    pub accel_kts_s: f64,
    pub decel_kts_s: f64,
    /// Stop at the path's last point (taxi), or fly on past it (air).
    pub stop_at_end: bool,
    /// Slow for the corners ahead (taxi). In the air a corner is a turn at
    /// speed.
    pub slow_for_corners: bool,
    /// The shortest lookahead, metres.
    pub min_look_m: f64,
}

impl FollowSpec {
    /// Taxi at `kts`, turning at most `turn_dps`, stopping at the end.
    pub fn taxi(kts: f64, turn_dps: f64) -> FollowSpec {
        FollowSpec {
            kts,
            max_turn_dps: turn_dps,
            accel_kts_s: 1.5,
            decel_kts_s: 2.0,
            stop_at_end: true,
            slow_for_corners: true,
            min_look_m: 5.0,
        }
    }

    /// Fly at `kts`, turning at most `turn_dps`, never stopping.
    pub fn air(kts: f64, turn_dps: f64) -> FollowSpec {
        FollowSpec {
            kts,
            max_turn_dps: turn_dps,
            accel_kts_s: 2.0,
            decel_kts_s: 2.0,
            stop_at_end: false,
            slow_for_corners: false,
            min_look_m: 50.0,
        }
    }
}

/// Progress along one path: the segment being flown. Belongs to the phase
/// that owns the path; a new path starts a new `Follow`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Follow {
    pub seg: usize,
    pub done: bool,
}

/// The slowest a taxi corner is taken, as a fraction of the straight pace.
const CORNER_MIN_FRACTION: f64 = 0.35;

/// The slowest anything taxis while it is still moving, knots.
const CREEP_KTS: f64 = 2.0;

/// How a point sits against a segment: the fraction along it, the signed
/// cross-track (right positive, metres) and the segment's length.
fn project(a: LatLon, b: LatLon, p: LatLon) -> (f64, f64, f64) {
    let (bx, by) = offset_m(a, b);
    let (px, py) = offset_m(a, p);
    let len2 = bx * bx + by * by;
    if len2 < 1e-6 {
        return (1.0, px.hypot(py), 0.0);
    }
    let len = len2.sqrt();
    let t = (px * bx + py * by) / len2;
    let cross = (px * by - py * bx) / len;
    (t, cross, len)
}

/// The cross-track from `p` to the path's NEAREST segment, metres,
/// unsigned. For tests and scorers.
pub fn cross_track_m(path: &[LatLon], p: LatLon) -> f64 {
    path.windows(2)
        .map(|w| {
            let (t, cross, len) = project(w[0], w[1], p);
            if (0.0..=1.0).contains(&t) {
                cross.abs()
            } else {
                let (a, b) = (dist_m(w[0], p), dist_m(w[1], p));
                if len == 0.0 {
                    a
                } else {
                    a.min(b)
                }
            }
        })
        .fold(f64::INFINITY, f64::min)
}

/// One substep of pure pursuit along `path`: steer at a point a lookahead
/// ahead on the path, hold `spec.kts` (less for the corners ahead and the
/// stop at the end), turn no faster than `spec.max_turn_dps`. Returns true
/// once the path is finished: stopped on its last point (`stop_at_end`), or
/// past it.
///
/// The lookahead is never shorter than the turn radius at the speed flown,
/// so a turn begins one radius before a corner and ends one after it: the
/// fillet a pilot flies, and no overshoot from a saturated turn.
pub fn follow(path: &[LatLon], at: &mut Follow, kin: &mut Kin, spec: &FollowSpec, dt: f64) -> bool {
    if at.done && spec.stop_at_end {
        kin.gs_kts = 0.0;
        kin.integrate(0.0, dt);
        return true;
    }
    let here = kin.pos();
    if path.len() < 2 {
        kin.speed_toward(0.0, spec.accel_kts_s, spec.decel_kts_s, dt);
        kin.integrate(0.0, dt);
        at.done = !spec.stop_at_end || kin.gs_kts == 0.0;
        return at.done;
    }
    let last_seg = path.len() - 2;
    at.seg = at.seg.min(last_seg);
    let v_ms = kin.gs_kts.max(0.0) * KT_TO_MS;
    let omega = spec.max_turn_dps.to_radians().max(1e-6);
    let radius = v_ms / omega;
    // Move on to the next segment once past this one, or once nearer it
    // (cutting a corner). In the air a corner is FLOWN BY: the turn begins
    // where an arc at the full rate meets both legs, `radius * tan(turn/2)`
    // before the corner, or it starts late and overshoots the new leg.
    loop {
        if at.seg >= last_seg {
            break;
        }
        let (t, cross, len) = project(path[at.seg], path[at.seg + 1], here);
        let (t2, cross2, _) = project(path[at.seg + 1], path[at.seg + 2], here);
        let d_here = if t > 1.0 {
            dist_m(path[at.seg + 1], here)
        } else {
            cross.abs()
        };
        let d_next = if t2 < 0.0 {
            dist_m(path[at.seg + 1], here)
        } else {
            cross2.abs()
        };
        let fly_by = !spec.slow_for_corners && {
            let turn = signed_diff_deg(
                bearing(path[at.seg + 1], path[at.seg + 2]),
                bearing(path[at.seg], path[at.seg + 1]),
            )
            .abs()
            .min(150.0);
            (1.0 - t) * len <= radius * (turn.to_radians() / 2.0).tan()
        };
        if t >= 1.0 || fly_by || (t2 >= 0.0 && d_next < d_here) {
            at.seg += 1;
        } else {
            break;
        }
    }
    let (a, b) = (path[at.seg], path[at.seg + 1]);
    let (t, _, seg_len) = project(a, b, here);
    let tc = t.clamp(0.0, 1.0);
    let foot_left = (1.0 - tc) * seg_len;
    let to_end: f64 = foot_left
        + path[at.seg + 1..]
            .windows(2)
            .map(|w| dist_m(w[0], w[1]))
            .sum::<f64>();

    let look_radius = if spec.slow_for_corners {
        radius
    } else {
        0.5 * radius
    };
    let look = spec.min_look_m.max(look_radius).max(v_ms * 1.5);

    // The lookahead point: `look` metres on from the foot of the
    // perpendicular, along the path (or straight on past its end).
    let foot = {
        let (bx, by) = offset_m(a, b);
        shift(a, bx * tc, by * tc)
    };
    let aim = {
        let mut left = look;
        let mut from = foot;
        let mut k = at.seg + 1;
        loop {
            let d = dist_m(from, path[k]);
            if d >= left {
                let f = left / d.max(1e-9);
                let (ex, ey) = offset_m(from, path[k]);
                break shift(from, ex * f, ey * f);
            }
            left -= d;
            from = path[k];
            if k == path.len() - 1 {
                if spec.stop_at_end {
                    break path[k];
                }
                let brg = bearing(path[k - 1], path[k]);
                break project_m(path[k], brg, left);
            }
            k += 1;
        }
    };

    // Speed: the straight pace, less what the corners ahead and the stop
    // allow at the decel rate.
    let decel_ms2 = spec.decel_kts_s * KT_TO_MS;
    let mut v_allow = spec.kts * KT_TO_MS;
    if spec.slow_for_corners {
        let mut d = foot_left;
        for k in (at.seg + 1)..path.len().saturating_sub(1) {
            let turn =
                signed_diff_deg(bearing(path[k], path[k + 1]), bearing(path[k - 1], path[k])).abs();
            let frac = (1.0 - turn / 120.0).clamp(CORNER_MIN_FRACTION, 1.0);
            let v_corner = spec.kts * frac * KT_TO_MS;
            v_allow = v_allow.min((v_corner * v_corner + 2.0 * decel_ms2 * d).sqrt());
            d += dist_m(path[k], path[k + 1]);
            if d > 200.0 {
                break;
            }
        }
    }
    if spec.stop_at_end {
        v_allow = v_allow.min((2.0 * decel_ms2 * (to_end - 0.3).max(0.0)).sqrt());
    }

    // Steering: pure pursuit, saturated at the turn rate. A point behind is
    // a full-rate turn toward it, and a slow one on the ground.
    //
    // The gain is twice the classic arc's (2 sin(alpha) / L): with the
    // heading as the only state, the classic gain is damped at 0.7 and a
    // saturated intercept rings; this one is critically damped.
    let alpha = signed_diff_deg(bearing(here, aim), kin.hdg_deg);
    let mut turn_dps = if alpha.abs() > 90.0 {
        spec.max_turn_dps * alpha.signum()
    } else {
        let kappa = 4.0 * alpha.to_radians().sin() / look;
        (kappa * v_ms).to_degrees()
    };
    turn_dps = turn_dps.clamp(-spec.max_turn_dps, spec.max_turn_dps);
    // In the air, the LEAD TURN: closing on the leg with no more room
    // than a full-rate arc needs to roll out on it, turn at the full rate.
    // The pursuit alone eases off near the end of an arc and overshoots.
    if !spec.slow_for_corners {
        let course = bearing(a, b);
        let ce = signed_diff_deg(course, kin.hdg_deg);
        let (_, xte, _) = project(a, b, here);
        let closing = (kin.hdg_deg - course).to_radians().sin() * xte < 0.0;
        if closing
            && ce.abs() > 2.0
            && xte.abs() <= 1.1 * radius * (1.0 - ce.to_radians().cos()) + 0.5
        {
            turn_dps = spec.max_turn_dps * ce.signum();
        }
    }
    if spec.slow_for_corners && alpha.abs() > 45.0 {
        let frac = CORNER_MIN_FRACTION.max(1.0 - alpha.abs() / 120.0);
        v_allow = v_allow.min(spec.kts * frac * KT_TO_MS);
    }

    let mut want_kts = v_allow / KT_TO_MS;
    if spec.stop_at_end && to_end > 0.5 && spec.slow_for_corners {
        want_kts = want_kts.max(CREEP_KTS.min(spec.kts));
    }
    kin.speed_toward(want_kts, spec.accel_kts_s, spec.decel_kts_s, dt);
    if spec.stop_at_end && to_end <= 0.5 {
        kin.gs_kts = 0.0;
    }
    kin.integrate(turn_dps, dt);

    let (t_after, _, _) = project(path[last_seg], path[last_seg + 1], kin.pos());
    let past_end = at.seg == last_seg && t_after >= 1.0;
    if spec.stop_at_end {
        if to_end <= 0.5 || past_end {
            kin.gs_kts = 0.0;
            at.done = true;
        }
    } else if past_end {
        at.done = true;
    }
    at.done
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::XorShift64;

    const ORIGIN: LatLon = (36.67, -4.50);

    fn ll(east: f64, north: f64) -> LatLon {
        shift(ORIGIN, east, north)
    }

    /// Run `follow` until done or `max_s`, returning every substep's kin.
    fn drive(path: &[LatLon], mut kin: Kin, spec: &FollowSpec, max_s: f64) -> Vec<Kin> {
        let mut at = Follow::default();
        let mut out = vec![kin.clone()];
        let dt = crate::clock::SUBSTEP_S;
        let mut t = 0.0;
        while t < max_s {
            let done = follow(path, &mut at, &mut kin, spec, dt);
            out.push(kin.clone());
            t += dt;
            if done {
                break;
            }
        }
        out
    }

    #[test]
    fn a_taxi_converges_onto_a_straight_and_stops_at_its_end() {
        let path = vec![ll(0.0, 0.0), ll(0.0, 600.0)];
        // 12 m right of the line, nose 30 degrees off.
        let start = Kin::parked(ll(12.0, 0.0), 50.0, 330.0);
        let spec = FollowSpec::taxi(10.0, 20.0);
        let trace = drive(&path, start, &spec, 400.0);
        let end = trace.last().unwrap();
        assert_eq!(end.gs_kts, 0.0);
        assert!(
            dist_m(end.pos(), path[1]) < 1.5,
            "{:.2} m from the end",
            dist_m(end.pos(), path[1])
        );
        let late = trace.iter().filter(|k| offset_m(ORIGIN, k.pos()).1 > 150.0);
        let worst = late
            .map(|k| cross_track_m(&path, k.pos()))
            .fold(0.0, f64::max);
        assert!(worst < 0.5, "on the line after 150 m, worst {worst:.2} m");
    }

    #[test]
    fn a_taxi_rides_an_l_path_round_its_corner() {
        let path = vec![ll(0.0, 0.0), ll(0.0, 300.0), ll(300.0, 300.0)];
        let start = Kin::parked(path[0], 50.0, 0.0);
        let spec = FollowSpec::taxi(10.0, 20.0);
        let trace = drive(&path, start, &spec, 600.0);
        let worst = trace
            .iter()
            .map(|k| cross_track_m(&path, k.pos()))
            .fold(0.0, f64::max);
        assert!(worst < 4.0, "corner cut {worst:.2} m");
        let end = trace.last().unwrap();
        assert!(dist_m(end.pos(), path[2]) < 1.5);
        assert!(
            signed_diff_deg(end.hdg_deg, 90.0).abs() < 3.0,
            "ends facing east: {}",
            end.hdg_deg
        );
    }

    #[test]
    fn an_aircraft_flies_an_l_path_and_settles_on_the_second_leg() {
        let path = vec![ll(0.0, 0.0), ll(0.0, 4000.0), ll(8000.0, 4000.0)];
        let mut start = Kin::parked(ll(300.0, 0.0), 1000.0, 20.0);
        start.on_ground = false;
        start.gs_kts = 85.0;
        let spec = FollowSpec::air(85.0, 3.0);
        let trace = drive(&path, start, &spec, 600.0);
        let settled = trace
            .iter()
            .filter(|k| offset_m(ORIGIN, k.pos()).0 > 3000.0);
        let worst = settled
            .map(|k| cross_track_m(&path, k.pos()))
            .fold(0.0, f64::max);
        assert!(worst < 5.0, "settled on the second leg, worst {worst:.1} m");
    }

    #[test]
    fn the_turn_rate_is_never_exceeded() {
        let path = vec![
            ll(0.0, 0.0),
            ll(0.0, 200.0),
            ll(-200.0, 200.0),
            ll(-200.0, 0.0),
        ];
        let start = Kin::parked(ll(20.0, -20.0), 50.0, 180.0);
        let spec = FollowSpec::taxi(10.0, 20.0);
        let trace = drive(&path, start, &spec, 600.0);
        let dt = crate::clock::SUBSTEP_S;
        for w in trace.windows(2) {
            let rate = signed_diff_deg(w[1].hdg_deg, w[0].hdg_deg).abs() / dt;
            assert!(rate <= 20.0 + 1e-9, "turned at {rate} deg/s");
        }
    }

    fn seeded_trace(seed: u64) -> Vec<u64> {
        let mut r = XorShift64::new(seed);
        let path = vec![ll(0.0, 0.0), ll(0.0, 300.0), ll(300.0, 300.0)];
        let start = Kin::parked(
            ll(r.range_f64(-20.0, 20.0), 0.0),
            50.0,
            r.range_f64(0.0, 360.0),
        );
        let spec = FollowSpec::taxi(r.range_f64(8.0, 12.0), 20.0);
        drive(&path, start, &spec, 300.0)
            .iter()
            .flat_map(|k| {
                [
                    k.lat.to_bits(),
                    k.lon.to_bits(),
                    k.hdg_deg.to_bits(),
                    k.gs_kts.to_bits(),
                ]
            })
            .collect()
    }

    #[test]
    fn the_same_seed_and_inputs_give_a_bit_identical_trace() {
        assert_eq!(seeded_trace(7), seeded_trace(7));
        assert_ne!(seeded_trace(7), seeded_trace(8));
    }

    #[test]
    fn the_vertical_reaches_its_target_within_its_rates() {
        let mut k = Kin::parked(ORIGIN, 0.0, 0.0);
        k.on_ground = false;
        k.gs_kts = 74.0;
        let dt = crate::clock::SUBSTEP_S;
        for _ in 0..3000 {
            k.vertical_toward(1000.0, 700.0, 500.0, dt);
            assert!(k.vs_fpm <= 700.0 + 1e-9);
            k.integrate(0.0, dt);
        }
        assert!((k.alt_ft - 1000.0).abs() < 1.0, "{}", k.alt_ft);
    }
}
