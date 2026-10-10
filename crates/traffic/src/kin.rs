//! Kinematics for traffic: a point that moves the way an aircraft looks
//! like it moves. Pure, deterministic, flat earth around the aircraft; NOT
//! the crew's flight laws (those fly a real aeroplane through the sim; this
//! IS the aeroplane). Heading equals track in v1 (no wind triangle).
//!
//! On the ground a taxi is a BICYCLE ([`Gear`], [`follow_on_gear`]): the
//! main gear is the point that cannot slip sideways, so the aircraft yaws
//! about it, and the nose gear, a wheelbase ahead, is the point steered
//! along the line, as a pilot keeps the nosewheel on the paint. The mains
//! then cut inside a corner (what taxiway fillets are paved for). The
//! published position stays the one reference point the plugin draws the
//! model at, mid-fuselage, the point the spacing measures half a length to
//! the nose and the tail from, and the point that stops on a stand or a
//! hold as before.

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

/// An aircraft's taxi geometry: where its wheels are against the published
/// point ([`Kin`]'s lat/lon, the model's mid-fuselage reference).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gear {
    /// Nose gear to main gear, metres.
    pub wheelbase_m: f64,
    /// The main gear's distance behind the published point, metres
    /// (negative: ahead of it).
    pub mains_aft_m: f64,
    /// The straight taxi pace the corners are sized for, knots: a corner's
    /// arc is fixed by it, so the curve the nose follows does not move as
    /// the speed asked for changes (the spacing slows a drive every
    /// substep).
    pub pace_kts: f64,
}

/// The most the nosewheel is steered, degrees: an airliner's tiller gives
/// 70 to 78.
const MAX_NOSEWHEEL_DEG: f64 = 70.0;

impl Gear {
    /// How far the nose gear is ahead of the published point, metres.
    pub fn nose_ahead_m(&self) -> f64 {
        self.wheelbase_m - self.mains_aft_m
    }

    /// The main gear's position.
    pub fn mains(&self, kin: &Kin) -> LatLon {
        project_m(kin.pos(), kin.hdg_deg, -self.mains_aft_m)
    }

    /// The nose gear's position.
    pub fn nose(&self, kin: &Kin) -> LatLon {
        project_m(kin.pos(), kin.hdg_deg, self.nose_ahead_m())
    }

    fn wheelbase(&self) -> f64 {
        self.wheelbase_m.max(0.1)
    }

    /// The radius of the arc the nose is steered round at a corner taken at
    /// `v_ms`, turning at most `omega` radians a second: the nose's own
    /// circle (the mains' radius and the wheelbase at right angles), never
    /// inside the nosewheel's lock, with a tenth in hand so the turn does
    /// not saturate on it.
    fn fillet_radius_m(&self, v_ms: f64, omega: f64) -> f64 {
        let l = self.wheelbase();
        let lock = l / MAX_NOSEWHEEL_DEG.to_radians().sin();
        1.1 * (v_ms / omega).hypot(l).max(lock)
    }
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

    /// One substep of motion on the ground as a bicycle: the MAIN GEAR moves
    /// along the heading at the groundspeed (no sideslip there) while the
    /// heading turns at `turn_dps`, and the published point is carried with
    /// it, `gear.mains_aft_m` ahead of the mains. Airborne it is
    /// [`Kin::integrate`].
    pub fn integrate_on_gear(&mut self, turn_dps: f64, gear: &Gear, dt: f64) {
        if !self.on_ground {
            self.integrate(turn_dps, dt);
            return;
        }
        let mains = gear.mains(self);
        self.hdg_deg = (self.hdg_deg + turn_dps * dt).rem_euclid(360.0);
        self.track_deg = self.hdg_deg;
        let v = self.gs_kts.max(0.0) * KT_TO_MS;
        let mains = project_m(mains, self.hdg_deg, v * dt);
        let p = project_m(mains, self.hdg_deg, gear.mains_aft_m);
        self.lat = p.0;
        self.lon = p.1;
        self.roll = 0.0;
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
    /// The segment the published point is on.
    pub seg: usize,
    pub done: bool,
    /// The segment the nose gear is on ([`follow_on_gear`]): ahead of
    /// `seg` by up to a wheelbase, and only ever moving on, so a path that
    /// doubles back (a backtrack's turn round) is never read backwards.
    pub nose_seg: usize,
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
    follow_with(path, at, kin, spec, None, dt)
}

/// [`follow`] for a taxi on its wheels: a bicycle about the main gear with
/// the NOSE GEAR steered along the path (pure pursuit from the nose, the
/// nosewheel pointed at the aim), the path's corners taken as arcs the
/// nose can hold at the speed planned for them, and that speed reached
/// before the nose enters the arc. The published point still stops on the
/// path's last point, so a stand or a hold is reached as before. Anything
/// not a taxi on the ground (`slow_for_corners` off, or airborne) is
/// [`follow`] unchanged.
pub fn follow_on_gear(
    path: &[LatLon],
    at: &mut Follow,
    kin: &mut Kin,
    spec: &FollowSpec,
    gear: &Gear,
    dt: f64,
) -> bool {
    let gear = Some(gear).filter(|_| spec.slow_for_corners && kin.on_ground);
    follow_with(path, at, kin, spec, gear, dt)
}

fn follow_with(
    path: &[LatLon],
    at: &mut Follow,
    kin: &mut Kin,
    spec: &FollowSpec,
    gear: Option<&Gear>,
    dt: f64,
) -> bool {
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
            let (v_corner, d_turn) = match gear {
                None => {
                    let frac = (1.0 - turn / 120.0).clamp(CORNER_MIN_FRACTION, 1.0);
                    (spec.kts * frac * KT_TO_MS, d)
                }
                // On its wheels the corner's speed is due where the NOSE
                // enters the corner's arc: a wheelbase and the arc's
                // tangent before the published point reaches the corner.
                Some(g) => {
                    let len_in = dist_m(path[k - 1], path[k]);
                    let len_out = dist_m(path[k], path[k + 1]);
                    let c = gear_corner(g, spec, omega, turn, len_in, len_out);
                    (c.v_ms, (d - c.tangent_m - g.nose_ahead_m()).max(0.0))
                }
            };
            v_allow = v_allow.min((v_corner * v_corner + 2.0 * decel_ms2 * d_turn).sqrt());
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
    if let Some(g) = gear {
        let (turn_dps, delta_want, nose_at_end) = nose_steer(path, at, kin, spec, g, v_ms, omega);
        // The nosewheel pointed well off the nose is a slow turn, as the
        // pursuit's own alpha is below.
        if delta_want.abs() > 45.0 {
            let frac = CORNER_MIN_FRACTION.max(1.0 - delta_want.abs() / 120.0);
            v_allow = v_allow.min(spec.kts * frac * KT_TO_MS);
        }
        let mut want_kts = v_allow / KT_TO_MS;
        if spec.stop_at_end && to_end > 0.5 {
            want_kts = want_kts.max(CREEP_KTS.min(spec.kts));
        }
        kin.speed_toward(want_kts, spec.accel_kts_s, spec.decel_kts_s, dt);
        if spec.stop_at_end && to_end <= 0.5 {
            kin.gs_kts = 0.0;
        }
        kin.integrate_on_gear(turn_dps, g, dt);
        let (t_after, _, _) = project(path[last_seg], path[last_seg + 1], kin.pos());
        let past_end = at.seg == last_seg && t_after >= 1.0;
        if (spec.stop_at_end && to_end <= 0.5) || past_end || nose_at_end {
            if spec.stop_at_end {
                kin.gs_kts = 0.0;
            }
            at.done = true;
        }
        return at.done;
    }
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

/// How a taxi on its wheels takes one corner of its path.
struct GearCorner {
    /// The speed it is taken at, metres a second.
    v_ms: f64,
    /// How far before (and after) the corner the nose's arc begins.
    tangent_m: f64,
    /// The arc's radius.
    radius_m: f64,
}

/// One corner of `turn_deg` between legs `len_in` and `len_out` long: the
/// old corner pace (a fraction of the asked pace by the turn); and the arc
/// the nose is steered round, sized for that speed, its tangent at most
/// half of either leg so the arcs of two corners never overlap. A curve
/// DRAWN as short straights is not slowed for as a whole: capping its
/// speed to what the mains can turn at the rate cap (a high speed exit, a
/// jet at 18 kt on a 57 m curve) held the runway longer and cost the hour
/// sweep's ten worst fields 15 of 67 completed and 33 more go-arounds
/// (2026-10-10); the nose runs a little wide there instead.
fn gear_corner(
    g: &Gear,
    spec: &FollowSpec,
    omega: f64,
    turn_deg: f64,
    len_in: f64,
    len_out: f64,
) -> GearCorner {
    let frac = (1.0 - turn_deg / 120.0).clamp(CORNER_MIN_FRACTION, 1.0);
    let half_turn = (turn_deg.to_radians() / 2.0).max(1e-6);
    let v_ms = spec.kts * frac * KT_TO_MS;
    let r = g.fillet_radius_m(v_ms, omega);
    let tangent_m = (r * half_turn.tan()).min(0.5 * len_in).min(0.5 * len_out);
    GearCorner {
        v_ms,
        tangent_m,
        radius_m: tangent_m / half_turn.tan(),
    }
}

/// The path near the nose with its corners rounded into the arcs the nose
/// is steered round, in metres east and north of the nose.
struct NoseWindow {
    pts: Vec<(f64, f64)>,
    /// Distance along the window at each point.
    s: Vec<f64>,
    /// Along the window: the middle of the corner the nose's segment
    /// starts at, the one it ends at, and the one after.
    mids: [f64; 3],
}

impl NoseWindow {
    /// The point `s_m` along the window (its end past the last).
    fn at(&self, s_m: f64) -> (f64, f64) {
        for i in 1..self.pts.len() {
            if self.s[i] >= s_m {
                let span = (self.s[i] - self.s[i - 1]).max(1e-9);
                let f = ((s_m - self.s[i - 1]) / span).clamp(0.0, 1.0);
                let (a, b) = (self.pts[i - 1], self.pts[i]);
                return (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f);
            }
        }
        *self.pts.last().unwrap_or(&(0.0, 0.0))
    }

    /// Where the nose (the origin) is along the window between `lo` and
    /// `hi`: the nearest point, a stretch running against the heading
    /// counted 10 m further off, so where the path doubles back on itself
    /// the nose is read on the stretch it is driving, never the other.
    fn locate(&self, lo: f64, hi: f64, hdg_deg: f64) -> f64 {
        let (hs, hc) = hdg_deg.to_radians().sin_cos();
        let mut best = (f64::INFINITY, lo);
        for i in 1..self.pts.len() {
            let (s0, s1) = (self.s[i - 1], self.s[i]);
            if s1 < lo || s0 > hi || s1 - s0 < 1e-9 {
                continue;
            }
            let (a, b) = (self.pts[i - 1], self.pts[i]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let len = s1 - s0;
            let f_lo = ((lo - s0) / len).max(0.0);
            let f_hi = ((hi - s0) / len).min(1.0);
            let f = (-(a.0 * dx + a.1 * dy) / (len * len)).clamp(f_lo, f_hi);
            let (qx, qy) = (a.0 + dx * f, a.1 + dy * f);
            let against = (1.0 - (dx * hs + dy * hc) / len) / 2.0;
            let score = qx.hypot(qy) + 10.0 * against;
            if score < best.0 {
                best = (score, s0 + f * len);
            }
        }
        best.1
    }
}

/// The window of `path` from the corner before the nose's segment `seg` to
/// `reach_m` past its start, the path's end carried on `ext_m` straight
/// (so the published point, a wheelbase behind the nose, stops ON the
/// last point aligned with the last leg).
#[allow(clippy::too_many_arguments)]
fn nose_window(
    path: &[LatLon],
    seg: usize,
    nose: LatLon,
    g: &Gear,
    spec: &FollowSpec,
    omega: f64,
    ext_m: f64,
    reach_m: f64,
) -> NoseWindow {
    let i0 = seg.saturating_sub(1);
    let mut raw: Vec<(f64, f64)> = Vec::new();
    let mut ahead = 0.0;
    let mut reached_end = false;
    for (i, p) in path.iter().enumerate().skip(i0) {
        if i > seg {
            ahead += dist_m(path[i - 1], *p);
        }
        raw.push(offset_m(nose, *p));
        if i == path.len() - 1 {
            reached_end = true;
        } else if i > seg + 1 && ahead > reach_m {
            break;
        }
    }
    if reached_end && ext_m > 0.01 && raw.len() >= 2 {
        let (a, b) = (raw[raw.len() - 2], raw[raw.len() - 1]);
        let len = (b.0 - a.0).hypot(b.1 - a.1).max(1e-9);
        raw.push((
            b.0 + (b.0 - a.0) / len * ext_m,
            b.1 + (b.1 - a.1) / len * ext_m,
        ));
    }
    let mut w = NoseWindow {
        pts: Vec::with_capacity(raw.len() * 4),
        s: Vec::with_capacity(raw.len() * 4),
        mids: [0.0; 3],
    };
    let push = |w: &mut NoseWindow, p: (f64, f64)| {
        let s = match w.pts.last() {
            Some(q) => w.s[w.s.len() - 1] + (p.0 - q.0).hypot(p.1 - q.1),
            None => 0.0,
        };
        w.pts.push(p);
        w.s.push(s);
    };
    let mut mids = vec![0.0; raw.len()];
    push(&mut w, raw[0]);
    for j in 1..raw.len().saturating_sub(1) {
        let (p0, p1, p2) = (raw[j - 1], raw[j], raw[j + 1]);
        let (ax, ay) = (p1.0 - p0.0, p1.1 - p0.1);
        let (bx, by) = (p2.0 - p1.0, p2.1 - p1.1);
        let (la, lb) = (ax.hypot(ay), bx.hypot(by));
        // Counterclockwise positive, east and north.
        let theta = (ax * by - ay * bx).atan2(ax * bx + ay * by);
        let turn_deg = theta.abs().to_degrees();
        if la < 1e-6 || lb < 1e-6 || turn_deg < 0.5 {
            push(&mut w, p1);
            mids[j] = w.s[w.s.len() - 1];
            continue;
        }
        let c = gear_corner(g, spec, omega, turn_deg, la, lb);
        let (ua, ub) = ((ax / la, ay / la), (bx / lb, by / lb));
        let start = (p1.0 - ua.0 * c.tangent_m, p1.1 - ua.1 * c.tangent_m);
        let side = theta.signum();
        let centre = (
            start.0 - ua.1 * c.radius_m * side,
            start.1 + ua.0 * c.radius_m * side,
        );
        push(&mut w, start);
        let s_start = w.s[w.s.len() - 1];
        let n = (turn_deg / 5.0).ceil().max(1.0) as usize;
        let (rx, ry) = (start.0 - centre.0, start.1 - centre.1);
        for i in 1..n {
            let (sn, cs) = (theta * i as f64 / n as f64).sin_cos();
            push(
                &mut w,
                (centre.0 + rx * cs - ry * sn, centre.1 + rx * sn + ry * cs),
            );
        }
        push(
            &mut w,
            (p1.0 + ub.0 * c.tangent_m, p1.1 + ub.1 * c.tangent_m),
        );
        mids[j] = (s_start + w.s[w.s.len() - 1]) / 2.0;
    }
    if raw.len() >= 2 {
        push(&mut w, raw[raw.len() - 1]);
    }
    let last = raw.len() - 1;
    mids[last] = w.s[w.s.len() - 1];
    let js = seg - i0;
    let mid = |j: usize| mids[j.min(last)];
    w.mids = [
        if js == 0 { 0.0 } else { mid(js) },
        mid(js + 1),
        mid(js + 2),
    ];
    w
}

/// The nose gear's pure pursuit: the nosewheel pointed at a point a
/// lookahead on along the rounded path from where the nose is, the
/// heading turning as a bicycle's does about its mains (`v tan(delta) /
/// wheelbase`), at no more than the turn rate. Returns the turn rate and
/// the nosewheel angle wanted (before its lock).
///
/// Pointing the nosewheel at the aim sets the nose's own direction of
/// travel, so its distance off the line closes as a first-order lag over
/// the lookahead: no overshoot, and a short lookahead keeps the nose on a
/// curve (it cuts an arc of radius R by about look^2 / 2R).
fn nose_steer(
    path: &[LatLon],
    at: &mut Follow,
    kin: &Kin,
    spec: &FollowSpec,
    g: &Gear,
    v_ms: f64,
    omega: f64,
) -> (f64, f64, bool) {
    let last_seg = path.len() - 2;
    let nose = g.nose(kin);
    let look = spec.min_look_m.max(1.5 * v_ms);
    let ext_m = if spec.stop_at_end {
        g.nose_ahead_m()
    } else {
        look
    };
    let reach_m = look + 3.0 * g.fillet_radius_m(spec.kts.min(g.pace_kts) * KT_TO_MS, omega) + 20.0;
    at.nose_seg = at.nose_seg.max(at.seg).min(last_seg);
    let (w, s_nose) = loop {
        let w = nose_window(path, at.nose_seg, nose, g, spec, omega, ext_m, reach_m);
        let s_nose = w.locate(w.mids[0], w.mids[2], kin.hdg_deg);
        if at.nose_seg < last_seg && s_nose > w.mids[1] {
            at.nose_seg += 1;
            continue;
        }
        break (w, s_nose);
    };
    let aim = w.at(s_nose + look);
    let delta_want = if aim.0.hypot(aim.1) < 1e-6 {
        0.0
    } else {
        signed_diff_deg(aim.0.atan2(aim.1).to_degrees(), kin.hdg_deg)
    };
    let delta = delta_want
        .clamp(-MAX_NOSEWHEEL_DEG, MAX_NOSEWHEEL_DEG)
        .to_radians();
    let turn_dps = (v_ms * delta.tan() / g.wheelbase()).to_degrees();
    // The nose at the end of its carried-on line: the drive is over even
    // when the published point is not quite on the last point. An end
    // inside the turning circle (a vacate's short last leg) is otherwise
    // circled for good (ESNU, the hour sweep, 2026-10-10: a Lufthansa
    // vacate drove round its end at 6 kt for forty minutes).
    let at_end = spec.stop_at_end
        && at.nose_seg == last_seg
        && s_nose >= w.s.last().copied().unwrap_or(0.0) - 0.5;
    (
        turn_dps.clamp(-spec.max_turn_dps, spec.max_turn_dps),
        delta_want,
        at_end,
    )
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

    /// A 737-800's taxi geometry at a medium jet's pace.
    const B738: Gear = Gear {
        wheelbase_m: 15.6,
        mains_aft_m: 0.0,
        pace_kts: 18.0,
    };

    /// The 172's: the mains 1.3 m ahead of the published point.
    const C172: Gear = Gear {
        wheelbase_m: 1.65,
        mains_aft_m: -1.3,
        pace_kts: 10.0,
    };

    fn drive_on_gear(
        path: &[LatLon],
        mut kin: Kin,
        spec: &FollowSpec,
        gear: &Gear,
        max_s: f64,
    ) -> Vec<Kin> {
        let mut at = Follow::default();
        let mut out = vec![kin.clone()];
        let dt = crate::clock::SUBSTEP_S;
        let mut t = 0.0;
        while t < max_s {
            let done = follow_on_gear(path, &mut at, &mut kin, spec, gear, dt);
            out.push(kin.clone());
            t += dt;
            if done {
                break;
            }
        }
        out
    }

    /// The nose's distance off `path` carried on past its end by the nose's
    /// lead: stopped with the published point on the last point, the nose
    /// stands a wheelbase further on, on the last leg's line.
    fn nose_off(path: &[LatLon], gear: &Gear, nose: LatLon) -> f64 {
        let n = path.len();
        let brg = bearing(path[n - 2], path[n - 1]);
        let mut ext = path.to_vec();
        ext.push(project_m(path[n - 1], brg, gear.nose_ahead_m() + 1.0));
        cross_track_m(&ext, nose)
    }

    /// A 90 degree taxiway curve of radius `r`, drawn as apt.dat draws one:
    /// short straights every 5 degrees, between two long legs.
    fn curve_path(r: f64) -> Vec<LatLon> {
        let mut path = vec![ll(0.0, 0.0)];
        for i in 0..=18 {
            let a = (i as f64 * 5.0).to_radians();
            path.push(ll(r - r * a.cos(), 200.0 + r * a.sin()));
        }
        path.push(ll(r + 200.0, 200.0 + r));
        path
    }

    #[test]
    fn a_taxi_on_its_gear_rides_a_straight_and_stops_its_point_at_the_end() {
        let path = vec![ll(0.0, 0.0), ll(0.0, 600.0)];
        for gear in [B738, C172] {
            let start = Kin::parked(ll(12.0, 0.0), 50.0, 330.0);
            let spec = FollowSpec::taxi(gear.pace_kts, 10.0);
            let trace = drive_on_gear(&path, start, &spec, &gear, 600.0);
            let end = trace.last().unwrap();
            assert_eq!(end.gs_kts, 0.0);
            assert!(
                dist_m(end.pos(), path[1]) < 1.5,
                "{:.2} m from the end",
                dist_m(end.pos(), path[1])
            );
            let worst = trace
                .iter()
                .filter(|k| offset_m(ORIGIN, k.pos()).1 > 200.0)
                .map(|k| nose_off(&path, &gear, gear.nose(k)))
                .fold(0.0, f64::max);
            assert!(worst < 0.5, "nose on the line, worst {worst:.2} m");
            assert!(signed_diff_deg(end.hdg_deg, 0.0).abs() < 1.0);
        }
    }

    /// The nose gear stays on a 90 degree curve's paint while the mains
    /// cut inside it, by about wheelbase^2 / 2R (a 737 on a 40 m curve:
    /// 3 m); the published point between them. The pivot never slips
    /// sideways, and the heading never turns faster than the cap.
    #[test]
    fn round_a_curve_the_nose_holds_the_line_and_the_mains_cut_inside() {
        let r = 40.0;
        let path = curve_path(r);
        let centre = ll(r, 200.0);
        let start = Kin::parked(path[0], 50.0, 0.0);
        let spec = FollowSpec::taxi(10.0, 10.0);
        let trace = drive_on_gear(&path, start, &spec, &B738, 600.0);
        let (mut nose_worst, mut mains_in) = (0.0f64, 0.0f64);
        for k in &trace {
            nose_worst = nose_worst.max(nose_off(&path, &B738, B738.nose(k)));
            let m = B738.mains(k);
            let (e, n) = offset_m(ORIGIN, m);
            if n > 200.0 && e < r {
                mains_in = mains_in.max(r - dist_m(centre, m));
            }
        }
        println!("nose worst {nose_worst:.2} m, mains inside by {mains_in:.2} m");
        assert!(nose_worst < 1.0, "nose off the line {nose_worst:.2} m");
        assert!(mains_in > 2.0, "the mains cut inside: {mains_in:.2} m");
        let dt = crate::clock::SUBSTEP_S;
        for w in trace.windows(2) {
            let rate = signed_diff_deg(w[1].hdg_deg, w[0].hdg_deg).abs() / dt;
            assert!(rate <= 10.0 + 1e-9, "turned at {rate} deg/s");
            // No sideslip at the mains: they move along the heading.
            let (m0, m1) = (B738.mains(&w[0]), B738.mains(&w[1]));
            if dist_m(m0, m1) > 0.05 {
                let slip = signed_diff_deg(bearing(m0, m1), w[1].hdg_deg).abs();
                assert!(slip < 0.01, "the mains slipped {slip} deg");
            }
        }
        let end = trace.last().unwrap();
        assert!(dist_m(end.pos(), *path.last().unwrap()) < 1.5);
    }

    /// The old point model swung the nose out of the same curve: the gain.
    #[test]
    fn the_point_model_swung_the_nose_wide_where_the_gear_holds_it() {
        let path = curve_path(40.0);
        let start = Kin::parked(path[0], 50.0, 0.0);
        let spec = FollowSpec::taxi(10.0, 10.0);
        let old = drive(&path, start.clone(), &spec, 600.0)
            .iter()
            .map(|k| nose_off(&path, &B738, project_m(k.pos(), k.hdg_deg, 15.6)))
            .fold(0.0, f64::max);
        let new = drive_on_gear(&path, start, &spec, &B738, 600.0)
            .iter()
            .map(|k| nose_off(&path, &B738, B738.nose(k)))
            .fold(0.0, f64::max);
        println!("nose off the curve: point model {old:.2} m, gear {new:.2} m");
        assert!(new < old, "{new:.2} vs {old:.2}");
    }

    /// A sharp corner of the network (no curve drawn) is still ridden round,
    /// within the old corner's cut, and a 180 degree turn round on a
    /// backtrack is driven round and on, never read backwards.
    #[test]
    fn a_sharp_corner_and_a_turn_round_are_driven_on_gear() {
        for gear in [B738, C172] {
            let path = vec![ll(0.0, 0.0), ll(0.0, 300.0), ll(300.0, 300.0)];
            let start = Kin::parked(path[0], 50.0, 0.0);
            let spec = FollowSpec::taxi(gear.pace_kts, 10.0);
            let trace = drive_on_gear(&path, start, &spec, &gear, 600.0);
            let worst = trace
                .iter()
                .map(|k| nose_off(&path, &gear, gear.nose(k)))
                .fold(0.0, f64::max);
            let end = trace.last().unwrap();
            assert!(worst < 12.0, "corner cut {worst:.2} m");
            assert!(dist_m(end.pos(), path[2]) < 1.5);
            assert!(signed_diff_deg(end.hdg_deg, 90.0).abs() < 3.0);

            let path = vec![ll(0.0, 0.0), ll(0.0, 400.0), ll(0.0, 100.0)];
            let start = Kin::parked(path[0], 50.0, 0.0);
            let trace = drive_on_gear(&path, start, &spec, &gear, 900.0);
            let end = trace.last().unwrap();
            assert!(
                dist_m(end.pos(), path[2]) < 1.5,
                "turned round and on: {:.1} m off",
                dist_m(end.pos(), path[2])
            );
            assert!(signed_diff_deg(end.hdg_deg, 180.0).abs() < 5.0);
        }
    }

    #[test]
    fn the_same_inputs_give_a_bit_identical_trace_on_gear() {
        let run = || {
            drive_on_gear(
                &curve_path(40.0),
                Kin::parked(ll(3.0, 0.0), 50.0, 10.0),
                &FollowSpec::taxi(12.0, 10.0),
                &B738,
                300.0,
            )
            .iter()
            .flat_map(|k| [k.lat.to_bits(), k.lon.to_bits(), k.hdg_deg.to_bits()])
            .collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }
}
