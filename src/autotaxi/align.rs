//! The alignment on the strip (`LineUpAlign`), the stop straighten and the
//! join: its fix, turn and capture.

use super::*;

/// What the line-up's alignment wants this tick ([`LineUpAlign::step`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlignStep {
    /// The differential brakes, and whether the governor creeps (`true`) or
    /// stops (`false`, its symmetric brake then outweighs these).
    Align { bl: f64, br: f64, creep: bool },
    /// Stopped with the nose within [`ALIGNED_DEG`] of the course.
    Done { err_deg: f64 },
    /// Out of time or of runway rolled: stop, `err_deg` off.
    GiveUp { err_deg: f64, why: &'static str },
}

/// A line-up does not end until the nose points down the runway, on a
/// castoring nose: its position can be right and its heading not. LEMG 12
/// SR22 (1790690995, 2026-09-29): the drive ended 5.0 degrees off its final
/// bearing, still yawing left at 1.7 deg/s, and stopped at 107 true on a
/// 117.7 runway: the roll began 10.8 off (alignment FAIL). LILB 03 Evolution
/// (1790712974): the backtrack's turn-round ended 11.8 off (13 at the
/// stop) and the roll began 24.6 off. Runway STARTS, set on the numbers,
/// roll 0.3 to 0.5 off. So after the arrival the nose is brought round on
/// the brakes ([`align_pivot_brakes`]) at a creep, stopped, and judged
/// stopped; again if the stop swung it; bounded by [`ALIGN_MAX_S`] and
/// the runway it can roll ([`align_roll_limit_m`]). Pure: the drive feeds it.
///
/// It spends as little runway as it can. LILB 03 Evolution (1790721973,
/// 2026-09-30): the pivot took 13 degrees off in the first 2 m, then crept
/// 38 m more with the nose held 2.0 to 2.2 off by a standing yaw the 0.02
/// of brake its soft law gave could not beat, the idle thrust running the
/// creep from 2 to 5.7 kt, and stopped with 474.8 m ahead of a 500 m fit
/// (516.1 at the turn-round's stop). So: a slow trim holds the standing
/// yaw, a creep over its speed is stopped, and the roll is bounded by the
/// runway the fit can spare, the stop anticipated; with little to spare the
/// creep is halved and the nose pivots nearly in place.
#[derive(Debug, Clone)]
pub struct LineUpAlign {
    pub course_deg: f64,
    pub(super) t0: Option<f64>,
    pub(super) creeping: bool,
    /// How many times the governor was asked to creep (a stop that swung
    /// the nose is a second).
    pub creeps: u32,
    /// Ground it may roll, m ([`align_roll_limit_m`]).
    pub roll_limit_m: f64,
    /// The creep it asks the governor for.
    pub creep_kts: f64,
    /// The standing-yaw trim, signed brake (+ right).
    pub trim: f64,
    pub(super) over: bool,
    pub(super) last_t: Option<f64>,
    /// Closing a lateral past [`ALIGN_LATERAL_M`] ([`Self::step_on_line`]).
    pub closing: bool,
    /// Whether a lateral closure ever began (the time bound grows by
    /// [`ALIGN_LATERAL_EXTRA_S`]).
    pub closed_lateral: bool,
    /// A free castoring nose's one ground law (`ground_steer`): its brakes
    /// replace the aligning PD and its trim ([`Self::with_ground`]).
    pub(super) ground: Option<crate::ground_steer::GroundSteer>,
    /// ...and the rudder it owns, for the drive to write.
    pub rudder: f64,
}

impl LineUpAlign {
    #[cfg(test)]
    pub fn new(course_deg: f64) -> Self {
        Self::with_spare(course_deg, None)
    }

    /// An alignment that may roll only the runway `spare_m` the takeoff fit
    /// does not need.
    pub fn with_spare(course_deg: f64, spare_m: Option<f64>) -> Self {
        let roll_limit_m = align_roll_limit_m(spare_m);
        let creep_kts = if roll_limit_m < ALIGN_TIGHT_ROLL_M {
            ALIGN_TIGHT_CREEP_KTS
        } else {
            ALIGN_CREEP_KTS
        };
        LineUpAlign {
            course_deg,
            t0: None,
            creeping: false,
            creeps: 0,
            roll_limit_m,
            creep_kts,
            trim: 0.0,
            over: false,
            last_t: None,
            closing: false,
            closed_lateral: false,
            ground: None,
            rudder: 0.0,
        }
    }

    /// The alignment flown by the free nose's one ground law on `plant`:
    /// the heading (or the lateral's intercept) held by the plant inverted,
    /// the standing yaw its observer's, never faster than
    /// [`ALIGN_RATE_MAX_DPS`]. The creep, the stops and the bounds are this
    /// judge's, as before.
    pub fn with_ground(mut self, plant: crate::ground_steer::GroundPlant) -> Self {
        self.ground = Some(crate::ground_steer::GroundSteer::new(plant));
        self
    }

    /// Whether the one ground law flies it.
    pub fn free(&self) -> bool {
        self.ground.is_some()
    }

    /// Signed error, positive = the runway is right of the nose.
    pub fn err_deg(&self, psi_true: f64) -> f64 {
        crate::angles::signed_diff_deg(self.course_deg, psi_true)
    }

    #[cfg(test)]
    pub fn step(
        &mut self,
        t_s: f64,
        psi_true: f64,
        gs_kts: f64,
        yaw_dps: f64,
        rolled_m: f64,
    ) -> AlignStep {
        self.step_on_line(t_s, psi_true, gs_kts, yaw_dps, rolled_m, None)
    }

    /// [`Self::step`] knowing the lateral off the runway's centreline
    /// (`lat_m`, right positive): past [`ALIGN_LATERAL_M`] the creep flies
    /// an intercept back onto the line ([`align_intercept_deg`]) to
    /// [`ALIGN_LATERAL_DONE_M`] and then straightens, while the runway it
    /// may roll and the time keep [`ALIGN_LATERAL_RESERVE_M`] and
    /// [`ALIGN_LATERAL_RESERVE_S`] back to straighten in; past them it only
    /// aligns, as before. LILB 03 Evolution (1790740292, 2026-09-30): the
    /// turn-round's leg two stopped 1.9 m left and 11.7 degrees off, the
    /// alignment straightened the nose in 8 m and the roll began 2.3 m off
    /// (the alignment stage FAIL at 2), 61.9 m of runway to spare. A
    /// lateral inside [`ALIGN_LATERAL_M`] is left alone: the law is the
    /// heading's alone, as before.
    pub fn step_on_line(
        &mut self,
        t_s: f64,
        psi_true: f64,
        gs_kts: f64,
        yaw_dps: f64,
        rolled_m: f64,
        lat_m: Option<f64>,
    ) -> AlignStep {
        let t0 = *self.t0.get_or_insert(t_s);
        let dt = self.last_t.map_or(0.0, |l| (t_s - l).clamp(0.0, 1.0));
        self.last_t = Some(t_s);
        let course_err = self.err_deg(psi_true);
        // The stop this speed still needs, counted against the runway.
        let v = gs_kts / MS_TO_KTS;
        let stop_m = v * ALIGN_STOP_LATENCY_S + v * v / (2.0 * ALIGN_STOP_DECEL_MS2);
        let max_s = if self.closed_lateral {
            ALIGN_MAX_S + ALIGN_LATERAL_EXTRA_S
        } else {
            ALIGN_MAX_S
        };
        // A lateral closure while the runway and the time still leave room
        // to straighten after it.
        let room = rolled_m + stop_m + ALIGN_LATERAL_RESERVE_M < self.roll_limit_m
            && t_s - t0 + ALIGN_LATERAL_RESERVE_S < ALIGN_MAX_S + ALIGN_LATERAL_EXTRA_S;
        self.closing = match lat_m.filter(|l| l.is_finite()) {
            Some(l) if room => {
                if self.closing {
                    l.abs() > ALIGN_LATERAL_DONE_M
                } else {
                    l.abs() > ALIGN_LATERAL_M
                }
            }
            _ => false,
        };
        self.closed_lateral |= self.closing;
        // Closing, the heading wanted is the intercept; else the course.
        let err = if self.closing {
            course_err + align_intercept_deg(lat_m.unwrap_or(0.0))
        } else {
            course_err
        };
        let led = err - yaw_dps * ALIGN_LEAD_S;
        let settled = gs_kts < ALIGN_SETTLED_KTS && yaw_dps.abs() < ALIGN_SETTLED_DPS;
        if settled && !self.closing && err.abs() <= ALIGNED_DEG {
            self.creeping = false;
            return AlignStep::Done { err_deg: err };
        }
        let out_of_bounds = if t_s - t0 > max_s {
            Some("time")
        } else if rolled_m + stop_m >= self.roll_limit_m {
            Some("rolled")
        } else {
            None
        };
        if let Some(why) = out_of_bounds {
            // Out of bounds but inside what the roll's own hold straightens
            // (`autotakeoff::RollAlign`) and the alignment stage passes: done,
            // not a give-up (LILB 03 Evolution 1790720549, 2026-09-30: the
            // turn-round settled 2.2 off in 24.6 s and the mission gave up).
            self.creeping = false;
            if err.abs() <= ALIGN_ACCEPT_DEG {
                return AlignStep::Done { err_deg: err };
            }
            return AlignStep::GiveUp { err_deg: err, why };
        }
        // Creep while the nose still has to come round; stop once it is
        // close and not carried past (half the band, with hysteresis).
        let off = err.abs() > ALIGNED_DEG || led.abs() > ALIGNED_DEG;
        let close = err.abs() <= ALIGNED_DEG * 0.5 && led.abs() <= ALIGNED_DEG;
        let want = self.closing || if self.creeping { !close } else { off };
        // ...never faster than the creep: over it, stopped until back under.
        if gs_kts > self.creep_kts + ALIGN_CREEP_OVER_KTS {
            self.over = true;
        } else if gs_kts <= self.creep_kts {
            self.over = false;
        }
        if want && !self.creeping {
            self.creeps += 1;
        }
        self.creeping = want;
        let creep = want && !self.over;
        // The standing yaw (the propeller, a slope, a crosswind) is what a
        // soft law settles against: trimmed out while rolling and not
        // already coming round.
        let (bl, br) = match self.ground.as_mut() {
            Some(g) => {
                let s = g.step(crate::ground_steer::heading_demand(
                    dt,
                    gs_kts / MS_TO_KTS,
                    crate::ground_steer::Yaw::Now(yaw_dps),
                    err,
                    crate::ground_steer::Rudder::Own { slew_per_s: ALIGN_RUDDER_SLEW_PER_S },
                    PIVOT_BRAKE_MAX,
                    ALIGN_RATE_MAX_DPS,
                ));
                self.rudder = s.rudder;
                s.wheels()
            }
            None => {
                if want {
                    self.trim = align_trim_step(self.trim, err, yaw_dps, gs_kts, dt);
                }
                with_align_trim(align_pivot_brakes(err, yaw_dps), self.trim, yaw_dps)
            }
        };
        AlignStep::Align { bl, br, creep }
    }
}

/// Steering held while still rolling, even after the distance fade has run
/// out. Enough for the yaw damper to arrest a rotation, far short of the
/// full deflection that carves.
pub(super) const STRAIGHTEN_FLOOR: f64 = 0.35;
/// Below this groundspeed the aircraft has genuinely stopped and the fade may
/// go to zero. Matches the arrival test's own 1.5 kt so the two agree.
pub(super) const STRAIGHTEN_HOLD_KTS: f64 = 1.5;

/// Straighten the nosewheel as a stop closes: braking shifts weight onto
/// the nosewheel, and a deflected one carves under it — every stop at a
/// bendy route end turned into a swerve-and-lurch ("when you brake, the
/// aircraft steers", 2026-07-20). Full steering beyond 8 m to go, faded
/// across the last meters.
///
/// The fade may NOT reach zero while the aircraft is still moving.
/// `remaining_m` SATURATES at `-STOP_SHORT_M` the instant the projection
/// reaches the end of the polyline (`remaining = total - STOP_SHORT_M -
/// progress`, and progress cannot exceed total), so the distance term pins at
/// -5 and the fade hits zero with 2-3 kt still on the clock. Zeroing the
/// command also kills the yaw-rate damping *inside* it, so a residual rotation
/// goes unarrested through the last metres.
///
/// EGHI 2026-07-30, at the HOLDING POINT and repeatably: steer pinned at 0.00,
/// yaw −2 deg/s unarrested, cross-track growing 2.45 → 5.08 m while coasting
/// 2.7 → 1.6 kt. The aircraft stopped several metres off the paint and visibly
/// crooked. The same signature is in all three stops of that sortie.
///
/// The carve this exists to prevent needs SPEED; by the arrival threshold
/// there is none left to carve with, so a floor below it is free.
pub fn stop_straighten(steer: f64, remaining_m: f64, gs_kts: f64) -> f64 {
    let fade = ((remaining_m - 1.0) / 7.0).clamp(0.0, 1.0);
    let floor = if gs_kts > STRAIGHTEN_HOLD_KTS {
        STRAIGHTEN_FLOOR
    } else {
        0.0
    };
    steer * fade.max(floor)
}

/// The lateral allowance for an engage at `lateral` meters off the route
/// with `err_deg` of heading error: zero for an on-route, along-route start
/// (the tight established guard applies from the first tick), otherwise the
/// engage distance plus turning-circle room. The guard while joining is
/// [`OFF_ROUTE_STOP_M`] plus this.
pub fn join_allowance_m(lateral: f64, err_deg: f64) -> f64 {
    if lateral > OFF_ROUTE_STOP_M || err_deg.abs() > JOIN_ALLOW_ERR_DEG {
        lateral.max(OFF_ROUTE_STOP_M) + JOIN_GROW_M
    } else {
        0.0
    }
}

/// A JUMP IN A PROJECTION IS NEVER A MOVEMENT, tick to tick. While joining,
/// the facing pick (`taxi_guide::project_signed_facing`) can move to another
/// leg between two ticks as the nose swings; an offset that grew by more than
/// the aircraft moved (plus [`PROJECTION_JUMP_SLACK_M`]) is that jump, and
/// the plain nearest paint stands for the tick. LEMG 13 (sortie
/// 1790646817-full_auto, 2026-09-29, #178): a turn-round join 8.3 m off read
/// 149.4 m the next tick at 5.6 kt, and the off-route guard stopped the
/// taxi. `(fix, jumped)`. Pure.
pub(crate) fn join_fix(
    facing: Option<(f64, f64)>,
    plain: Option<(f64, f64)>,
    prev_lateral: Option<f64>,
    moved_m: f64,
) -> (Option<(f64, f64)>, bool) {
    match (facing, plain, prev_lateral) {
        (Some(f), Some(p), Some(prev))
            if f.1.abs() > prev + moved_m.max(0.0) + PROJECTION_JUMP_SLACK_M
                && p.1.abs() < f.1.abs() =>
        {
            (Some(p), true)
        }
        _ => (facing, false),
    }
}

/// A JOIN THE NOSE CANNOT POINT AT IS A PLANNED TURN, or no join at all.
/// Pure pursuit joins a route by aiming a few metres down it, and with the
/// route behind the wing line that aim is behind the aircraft: a jet pivots
/// toward it on the apron. LEMG (iMac 2026-09-29, sortie 1790712526, a
/// Citation X): the tug let go 51.5 m down the stand's lane with the nose
/// still on the stand (327.8), the cleared route's first leg was that lane
/// (155.3, the stand to the taxiway) and the taxiway beyond it left on 222.2.
/// No leg ran within 90 degrees of the nose, so the join aimed 16 m down
/// the lane at a point 149 degrees left (`join` err -149, lateral 6.9 m),
/// swung 188 degrees left through the aim and 27 m off the route before it
/// turned back (#190). A heavy airframe that does not pivot (neither
/// brake-steered nor castoring, over [`JOIN_TURN_LIGHT_KG`]) joining with
/// more than [`JOIN_TURN_MIN_DEG`] to turn, off a runway, flies a turn of
/// [`join_turn_radius_m`] from where it stands onto the route AHEAD of it:
/// the arc, then a straight meeting the route within [`JOIN_INTERCEPT_DEG`],
/// the shortest such turn (never the longest cut across an apron), every
/// sample on the drawn
/// pavement (`None` from the field is no verdict). No such turn is a
/// refusal, said. Pure.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct JoinTurn {
    /// Lat/lon from the aircraft round the arc and down the straight to the
    /// join point on the route.
    pub lead: Vec<(f64, f64)>,
    /// +1 a right turn, -1 left.
    pub dir: f64,
    /// The arc, degrees.
    pub turn_deg: f64,
    /// Along the route to the join point, m.
    pub join_m: f64,
    /// The arc plus the straight, m.
    pub lead_m: f64,
    /// The straight's angle onto the route, degrees.
    pub intercept_deg: f64,
}

/// A join needing more than this in heading is planned ([`JoinTurn`]).
pub(crate) const JOIN_TURN_MIN_DEG: f64 = 90.0;
/// At or under this `acf_m_max` an airframe turns on its brakes and nose
/// (the tug's line, `pushback::tug_for_weight`): the pursuit's own join.
pub(crate) const JOIN_TURN_LIGHT_KG: f64 = 5_700.0;
/// The straight meets the route at no more than this.
pub(super) const JOIN_INTERCEPT_DEG: f64 = 45.0;
/// ...after at least this much straight to settle on.
pub(super) const JOIN_STRAIGHT_MIN_M: f64 = 8.0;
/// The join point is sought this far along the route past the aircraft.
pub(super) const JOIN_TURN_REACH_M: f64 = 250.0;
/// An arc past this is a loop, not a turn onto a route.
pub(super) const JOIN_TURN_MAX_ARC_DEG: f64 = 240.0;
/// Pavement is sampled this often along the planned turn.
pub(super) const JOIN_SAMPLE_M: f64 = 3.0;
/// No planned turn fits: said, and the drive stands down holding.
pub(super) const JOIN_REFUSED_LINE: &str = "The route runs behind us and there's no room to turn round \
     onto it from here. Holding. Your aircraft.";

/// The planned join's radius by `acf_m_max`, m: the Citation X turned 27
/// deg/s at 8 kt in that LEMG join (8.7 m) and the 737 turns about 20 deg/s
/// at full lock under 7 kt (about 9 m at 6 kt; the 737 profile's taxi note),
/// so 15 m is a taxi turn with lock to spare; a widebody gets 25 m. Unread
/// reads as the medium jet.
pub(crate) fn join_turn_radius_m(mtow_kg: Option<f64>) -> f64 {
    match mtow_kg {
        Some(m) if m > 100_000.0 => 25.0,
        _ => 15.0,
    }
}

/// Does a join this far off the nose get a planned turn ([`JoinTurn`])?
/// Heavy (or unread), not pivoting, off a runway, a drive that is not a
/// line-up or turn-round (those plan their own geometry). Pure.
pub(crate) fn join_turn_applies(
    err_deg: f64,
    pivots: bool,
    mtow_kg: Option<f64>,
    on_runway: Option<bool>,
    own_geometry: bool,
) -> bool {
    err_deg.abs() > JOIN_TURN_MIN_DEG
        && !pivots
        && mtow_kg.is_none_or(|m| m > JOIN_TURN_LIGHT_KG)
        && on_runway != Some(true)
        && !own_geometry
}

pub(crate) fn join_turn(
    route: &[(f64, f64)],
    lat: f64,
    lon: f64,
    psi: f64,
    radius_m: f64,
    on_pavement: impl Fn(f64, f64) -> Option<bool>,
) -> Option<JoinTurn> {
    if route.len() < 2 || radius_m <= 0.0 {
        return None;
    }
    let geo = taxi_guide::Geo::new(lat);
    let here = (lat, lon);
    let pts: Vec<(f64, f64)> = route.iter().map(|&p| geo.to_m(here, p)).collect();
    let mut cum = vec![0.0_f64];
    for w in pts.windows(2) {
        cum.push(cum.last().unwrap() + taxi_guide::dist(w[0], w[1]));
    }
    let total = *cum.last().unwrap();
    // Where the aircraft projects onto the route (the nearest paint).
    let mut s0 = 0.0;
    let mut best_d = f64::INFINITY;
    for (i, w) in pts.windows(2).enumerate() {
        let (a, b) = (w[0], w[1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx * dx + dy * dy;
        let t = if len2 < 1e-9 {
            0.0
        } else {
            ((-a.0 * dx - a.1 * dy) / len2).clamp(0.0, 1.0)
        };
        let c = (a.0 + t * dx, a.1 + t * dy);
        let d = (c.0 * c.0 + c.1 * c.1).sqrt();
        if d < best_d {
            best_d = d;
            s0 = cum[i] + t * len2.sqrt();
        }
    }
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    let to_ll = |p: (f64, f64)| (lat + p.1 / 111_320.0, lon + p.0 / m_per_deg_lon);
    let (sin_p, cos_p) = (psi.to_radians().sin(), psi.to_radians().cos());
    let r = radius_m;
    let mut best: Option<(f64, JoinTurn)> = None;
    let mut s = s0 + JOIN_STRAIGHT_MIN_M;
    while s <= (s0 + JOIN_TURN_REACH_M).min(total - 1.0) {
        let here_s = s;
        s += 2.0;
        let q = taxi_guide::point_at(&pts, &cum, here_s);
        let i = (1..pts.len()).find(|&i| here_s <= cum[i]).unwrap_or(pts.len() - 1);
        let (a, b) = (pts[i - 1], pts[i]);
        if taxi_guide::dist(a, b) < 0.5 {
            continue;
        }
        let leg = taxi_guide::bearing(a, b);
        for dir in [-1.0_f64, 1.0] {
            // The turn's centre, abeam the nose on the turning side.
            let c = (dir * r * cos_p, -dir * r * sin_p);
            let d = taxi_guide::dist(c, q);
            if d < r * 1.2 {
                continue;
            }
            let bq = taxi_guide::bearing(c, q);
            let alpha = (r / d).acos().to_degrees();
            let b0 = taxi_guide::bearing(c, (0.0, 0.0));
            // Clockwise round the centre the heading is the bearing from it
            // plus 90; anticlockwise, less 90.
            let (arc, hs) = if dir > 0.0 {
                let beta = bq - alpha;
                ((beta - b0).rem_euclid(360.0), beta + 90.0)
            } else {
                let beta = bq + alpha;
                ((b0 - beta).rem_euclid(360.0), beta - 90.0)
            };
            let arc = if arc > 359.0 { 0.0 } else { arc };
            if arc > JOIN_TURN_MAX_ARC_DEG {
                continue;
            }
            let straight = (d * d - r * r).max(0.0).sqrt();
            if straight < JOIN_STRAIGHT_MIN_M {
                continue;
            }
            let intercept = crate::angles::abs_diff_deg(hs.rem_euclid(360.0), leg);
            if intercept > JOIN_INTERCEPT_DEG {
                continue;
            }
            let arc_m = r * arc.to_radians();
            let cost = arc_m + straight;
            if best.as_ref().is_some_and(|(c0, _)| *c0 <= cost) {
                continue;
            }
            // The turn, sampled: round the arc, then down the straight.
            let mut lead = vec![(0.0, 0.0)];
            let n_arc = (arc_m / JOIN_SAMPLE_M).ceil().max(1.0) as usize;
            for k in 1..=n_arc {
                let th = (b0 + dir * arc * k as f64 / n_arc as f64).to_radians();
                lead.push((c.0 + r * th.sin(), c.1 + r * th.cos()));
            }
            let t = *lead.last().unwrap();
            let n_st = (straight / JOIN_SAMPLE_M).ceil().max(1.0) as usize;
            for k in 1..=n_st {
                let f = k as f64 / n_st as f64;
                lead.push((t.0 + (q.0 - t.0) * f, t.1 + (q.1 - t.1) * f));
            }
            if lead.iter().any(|&p| {
                let (plat, plon) = to_ll(p);
                on_pavement(plat, plon) == Some(false)
            }) {
                continue;
            }
            let mut lead_ll: Vec<(f64, f64)> = lead.iter().map(|&p| to_ll(p)).collect();
            lead_ll[0] = here;
            best = Some((
                cost,
                JoinTurn {
                    lead: lead_ll,
                    dir,
                    turn_deg: arc,
                    join_m: here_s,
                    lead_m: arc_m + straight,
                    intercept_deg: intercept,
                },
            ));
        }
    }
    best.map(|(_, j)| j)
}

/// Measurement noise a join's offset may grow by, beyond the distance
/// actually moved, in one tick.
pub(super) const PROJECTION_JUMP_SLACK_M: f64 = 5.0;

/// True when a join has captured the route: on the line and pointed along it.
pub fn join_done(lateral: f64, err_deg: f64) -> bool {
    lateral < JOIN_DONE_LAT_M && err_deg.abs() < JOIN_DONE_ERR_DEG
}

/// The join's capture latch: [`join_done`], or a capture that has SETTLED
/// (aligned, inside the old 15 m, and no longer closing). Pure; `t_s` is any
/// monotonic clock.
#[derive(Debug, Default, Clone, Copy)]
pub struct JoinCapture {
    pub(super) best: Option<(f64, f64)>,
}

impl JoinCapture {
    pub fn step(&mut self, lateral: f64, err_deg: f64, t_s: f64) -> bool {
        if join_done(lateral, err_deg) {
            return true;
        }
        match self.best {
            Some((b, _)) if lateral > b - JOIN_PROGRESS_M => {}
            _ => self.best = Some((lateral, t_s)),
        }
        let since = self.best.map_or(t_s, |(_, t)| t);
        lateral < JOIN_SETTLE_MAX_LAT_M
            && err_deg.abs() < JOIN_DONE_ERR_DEG
            && t_s - since >= JOIN_SETTLE_S
    }
}
