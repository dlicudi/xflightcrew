//! The Zibo 737's rotation (LEMG 30, 2026-10-07): the elevator fitted to the
//! two rotations the column reached (1791371247 at 45.7 t, Vr 145 as then
//! declared; 1791374430 at 43.7 t, Vr 108 weighed), and the rotation law
//! flown against that fit.
//!
//! Both rotations lagged the 2.5 deg/s ramp by 4 to 5 degrees the whole way,
//! the column at 0.1 to 0.4 where the nose wanted more: rotate 108.5,
//! liftoff 153 (1791374430); rotate 146.7, liftoff 173 (1791371247).

use super::*;

const ROTATIONS: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/bench/takeoffs/zibo738_rotations_lemg30_2026-10-07.jsonl"
));

/// One tick of a recorded rotation: a `roll` record from the one after the
/// `rotate`, then `climb_manual`. Time is the sum of the logged `dt` (the
/// record's own `t` is rounded to a tenth).
#[derive(Debug, Clone, Copy)]
struct Tick {
    t: f64,
    ias: f64,
    theta: f64,
    q: f64,
    /// The column as the sim held it this tick (`yoke`): the last one the
    /// law wrote, as zibomod's joystick input took it.
    yoke: f64,
    wheels: bool,
    /// Height, feet: logged on the climb ticks only.
    agl: Option<f64>,
}

/// The two recorded rotations: (sortie, weight in tonnes, Vr flown, ticks).
fn rotations() -> Vec<(&'static str, f64, f64, Vec<Tick>)> {
    let recs: Vec<serde_json::Value> = ROTATIONS
        .lines()
        .filter_map(|l| serde_json::from_str(l).ok())
        .collect();
    let f = |r: &serde_json::Value, k: &str| r[k].as_f64();
    [
        ("1791371247-full_auto", 45.7, 145.0),
        ("1791374430-full_auto", 43.7, 108.0),
    ]
    .into_iter()
    .map(|(sid, w, vr)| {
        let rs: Vec<&serde_json::Value> = recs.iter().filter(|r| r["sortie"] == sid).collect();
        let rotate_t = rs
            .iter()
            .find(|r| r["event"] == "rotate")
            .and_then(|r| f(r, "t"))
            .expect("rotate");
        let mut t = 0.0;
        let ticks = rs
            .iter()
            .filter(|r| {
                (r["event"] == "roll" && f(r, "t").unwrap() > rotate_t)
                    || r["event"] == "climb_manual"
            })
            .enumerate()
            .map(|(i, r)| {
                if i > 0 {
                    t += f(r, "dt").unwrap();
                }
                Tick {
                    t,
                    ias: f(r, "ias").unwrap(),
                    theta: f(r, "theta").unwrap(),
                    q: f(r, "q").unwrap(),
                    yoke: f(r, "yoke").unwrap(),
                    wheels: r["event"] == "roll" && r["wheels"] == true,
                    agl: f(r, "agl").filter(|_| r["event"] == "climb_manual"),
                }
            })
            .collect();
        (sid, w, vr, ticks)
    })
    .collect()
}

/// The Zibo's pitch through the rotation, fitted to those two by output
/// error: the column read back on each tick (zibomod's joystick input as
/// the sim holds it) driven through the plant from the attitude at the
/// rotate over the recorded airspeed, against the attitude on every tick and
/// the height on every climb tick, to 300 ft.
///
/// A rate plant, as the SR22's: the nose turns
/// `k x ((v/ref)^2 x (u - per_deg x (alpha - rest)) - base)` deg/s through a
/// lag. On the wheels alpha is the attitude, the base the breakout, the lag
/// `TAU_S`: 17.0 deg/s per unit at 130 kt, breakout 0.033, 0.077 per degree,
/// 0.59 s (0.10 degree rms on the wheels alone). Flying, alpha is the
/// attitude less the TRUE flight path (the lift below flies it), and the
/// spring and the base walk from the wheels' to the air's by the ground
/// effect's height ([`Air`]). The lift is `lift x v^2 x (alpha - zero)` per
/// tonne, `ground_lift` more on the runway fading by 10 ft; the wheels leave
/// where it passes the weight. Both rotations: 0.15 degree rms to 300 ft
/// (0.31 worst), the height 3.0 and 1.2 ft rms, the wheels off 0.3 and 0.8 s
/// before `onground_any` let go.
///
/// The flight model's vertical speed (`vh_ind_fpm`, what the law reads)
/// trails the height's own rate by about 1.3 s (`VS_LAG_S`): 432 fpm read at
/// 13 ft where the height was climbing 1,145.
#[derive(Debug, Clone, Copy)]
struct ZiboRotation {
    theta: f64,
    q: f64,
    v: f64,
    /// Height, feet; true vertical speed, ft/s; the one the law reads, fpm.
    h: f64,
    vs: f64,
    vs_read: f64,
    wheels: bool,
    weight_t: f64,
    /// The roll attitude, degrees: the nose rests here on the wheels.
    rest: f64,
    air: Air,
    /// Multipliers on the fitted elevator, everywhere, and on the wheels'
    /// spring (robustness).
    k_mult: f64,
    spring_mult: f64,
    /// The thrust as a share of the levers at 1.0 (104 % N1): 1.0 is every
    /// flown roll here.
    thrust: f64,
}

/// The flying half of the fit. Its spring and base are found together (the
/// climb holds its angle of attack near 4 to 8 degrees, so the two trade
/// along a line): [`Air::FIT`] is the best, [`Air::NEXT`] the next family
/// member the search settled on, 0.19 and 0.20 degree rms.
#[derive(Debug, Clone, Copy)]
struct Air {
    per_deg: f64,
    base: f64,
    ground_effect_ft: f64,
    tau_s: f64,
    lift: f64,
    lift_zero_deg: f64,
    ground_lift: f64,
}

impl Air {
    const FIT: Air = Air {
        per_deg: 0.225,
        base: -1.501,
        ground_effect_ft: 23.6,
        tau_s: 0.427,
        lift: 3.13e-4,
        lift_zero_deg: -3.93,
        ground_lift: 0.196,
    };
    const NEXT: Air = Air {
        per_deg: 0.196,
        base: -1.324,
        ground_effect_ft: 36.3,
        tau_s: 0.308,
        lift: 2.974e-4,
        lift_zero_deg: -4.36,
        ground_lift: 0.209,
    };
}

impl ZiboRotation {
    const REF_KTS: f64 = 130.0;
    const K: f64 = 17.04;
    const BREAKOUT: f64 = 0.033;
    const PER_DEG: f64 = 0.077;
    const TAU_S: f64 = 0.594;
    const REST_DEG: f64 = -0.7;
    const VS_LAG_S: f64 = 1.3;
    /// The write's delay past the law's tick, seconds (the fit put the
    /// whole of the lag in `TAU_S`; this is margin).
    const DELAY_S: f64 = 0.1;
    /// Drag and friction's share of the full-thrust acceleration at 43.7 t,
    /// kt/s: the levers at 1.0 are about 29K a side (the 27K table's 100.8
    /// at 15 C on the field, as N1^2.3, to 104), 0.49 g at 43.7 t through
    /// the lapse to 150 kt, against the 7.2 kt/s (0.38 g) the roll made
    /// there. The flown climbs check it: at 168 kt and 48 t this plant
    /// climbs 4,500 fpm gaining a knot a second, the engages 4,600 to 4,700.
    const DRAG_KTS_S: f64 = 2.2;

    fn new(v: f64, weight_t: f64) -> Self {
        ZiboRotation {
            theta: Self::REST_DEG,
            q: 0.0,
            v,
            h: 0.0,
            vs: 0.0,
            vs_read: 0.0,
            wheels: true,
            weight_t,
            rest: Self::REST_DEG,
            air: Air::FIT,
            k_mult: 1.0,
            spring_mult: 1.0,
            thrust: 1.0,
        }
    }

    fn path_deg(&self) -> f64 {
        if self.wheels {
            0.0
        } else {
            self.vs.atan2(self.v * 1.687_81).to_degrees()
        }
    }

    /// zibomod's joystick input as read back: a column under 0.05 is held
    /// at 0.283 of itself (0.036 read 0.010, 0.025 read 0.007), above it
    /// exactly.
    fn as_held(u: f64) -> f64 {
        if u.abs() < 0.05 {
            0.283 * u
        } else {
            u
        }
    }

    /// Takeoff thrust's acceleration: 8.4 kt/s at 110 kt (1791374430's roll),
    /// 7.2 at 150 (1791371247's), less the climb's share of the weight.
    /// Heavier, it gains as the weight would have it.
    /// Less thrust (`thrust` under 1.0) gives up its share of the thrust
    /// alone, never the drag's: [`Self::DRAG_KTS_S`] of the 8.4 is drag and
    /// rolling friction.
    fn accel_kts_s(&self) -> f64 {
        let full = 8.4 - 0.03 * (self.v - 110.0);
        (self.thrust * (full + Self::DRAG_KTS_S) - Self::DRAG_KTS_S) * 43.7 / self.weight_t
            - 19.06 * self.path_deg().to_radians().sin()
    }

    /// `u` is the column as held, `v` the airspeed when the caller gives it
    /// (the replay); `None` flies the plant's own.
    fn advance(&mut self, u: f64, dur_s: f64, v: Option<f64>) {
        let a = self.air;
        let step_s = 0.005_f64;
        let mut left = dur_s;
        while left > 1e-9 {
            let step = step_s.min(left);
            let dq = (self.v / Self::REF_KTS).powi(2);
            let path = self.path_deg();
            let alpha = self.theta - path;
            let (want, tau) = if self.wheels {
                let per = Self::PER_DEG * self.spring_mult;
                (
                    Self::K * (dq * (u - per * (alpha - self.rest)) - Self::BREAKOUT),
                    Self::TAU_S,
                )
            } else {
                let eff = (1.0 - self.h / a.ground_effect_ft).max(0.0);
                let wheels_per = Self::PER_DEG * self.spring_mult;
                let per = a.per_deg + (wheels_per - a.per_deg) * eff;
                let base = a.base + (Self::BREAKOUT - a.base) * eff;
                (
                    Self::K * (dq * (u - per * (alpha - self.rest)) - base),
                    a.tau_s,
                )
            };
            self.q += (want * self.k_mult - self.q) / tau * step;
            self.theta += self.q * step;
            if self.wheels && self.theta < self.rest {
                self.theta = self.rest;
                self.q = self.q.max(0.0);
            }
            let ground = (1.0 - self.h / 10.0).max(0.0);
            let n = a.lift
                * self.v.powi(2)
                * (alpha - a.lift_zero_deg)
                * (1.0 + a.ground_lift * ground)
                / self.weight_t;
            if self.wheels && n >= 1.0 {
                self.wheels = false;
            }
            if !self.wheels {
                self.vs += 32.174 * (n - path.to_radians().cos()) * step;
                self.h += self.vs * step;
                if self.h <= 0.0 && self.vs < 0.0 {
                    (self.h, self.vs, self.wheels) = (0.0, 0.0, true);
                }
            }
            self.vs_read += (self.vs * 60.0 - self.vs_read) * step / Self::VS_LAG_S;
            match v {
                Some(v) => self.v = v,
                None => self.v += self.accel_kts_s() * step,
            }
            left -= step;
        }
    }
}

/// Drives the plant with each rotation's recorded column (the next tick's
/// read-back) over its recorded airspeed: (attitude errors, height errors,
/// plant wheels-off time less the recorded).
fn replay_plant(weight_t: f64, air: Air, ticks: &[Tick]) -> (Vec<f64>, Vec<f64>, f64) {
    let mut p = ZiboRotation {
        air,
        ..ZiboRotation::new(ticks[0].ias, weight_t)
    };
    (p.theta, p.rest, p.q) = (ticks[0].theta, ticks[0].theta, ticks[0].q);
    let (mut e_th, mut e_h) = (Vec::new(), Vec::new());
    let mut off = f64::NAN;
    let mut t = ticks[0].t;
    for w in ticks.windows(2) {
        let (k, next) = (w[0], w[1]);
        while t < next.t - 1e-9 {
            let step = 0.02_f64.min(next.t - t);
            let v = k.ias + (next.ias - k.ias) * ((t - k.t) / (next.t - k.t));
            p.advance(next.yoke, step, Some(v));
            t += step;
            if !p.wheels && off.is_nan() {
                off = t;
            }
        }
        e_th.push(p.theta - next.theta);
        if let Some(h) = next.agl {
            e_h.push(p.h - h);
        }
    }
    let rec_off = ticks.iter().find(|k| !k.wheels).map_or(f64::NAN, |k| k.t);
    (e_th, e_h, off - rec_off)
}

fn rms(e: &[f64]) -> f64 {
    (e.iter().map(|x| x * x).sum::<f64>() / e.len() as f64).sqrt()
}

/// The plant is the flown rotation: driven by the column each flight held,
/// from the attitude at the rotate, the fit keeps the recorded attitude
/// within 0.2 degree rms and half a degree on every tick to 300 ft, the
/// height within 3.5 ft rms, and lifts the wheels within a second of the
/// record; the next fit within 0.25 rms and 0.6.
#[test]
fn the_zibo_plant_is_the_flown_rotation() {
    for (air, rms_max, worst_max) in [(Air::FIT, 0.2, 0.5), (Air::NEXT, 0.25, 0.6)] {
        for (sid, w, _, ticks) in rotations() {
            let (e_th, e_h, off) = replay_plant(w, air, &ticks);
            assert!(e_th.len() > 25 && e_h.len() > 15, "{sid}");
            let worst = e_th.iter().fold(0.0_f64, |a, e| a.max(e.abs()));
            assert!(rms(&e_th) < rms_max, "{sid}: {:.2} degree rms", rms(&e_th));
            assert!(worst < worst_max, "{sid}: {worst:.2} degrees");
            assert!(rms(&e_h) < 3.5, "{sid}: {:.1} ft rms", rms(&e_h));
            assert!(
                (-1.0..=0.2).contains(&off),
                "{sid}: wheels off {off:.2} s from the record"
            );
        }
    }
}

/// What a closed-loop rotation did.
#[derive(Debug, Clone, Copy)]
struct Flown {
    /// Airspeed where the wheels left and at 8 ft (the takeoff's liftoff).
    wheels_off_kts: f64,
    liftoff_kts: f64,
    /// Worst attitude behind and ahead of the commanded target from the
    /// rotate to 8 ft (the rotation), and from there to 300 ft (the climb).
    rotate_lag: f64,
    rotate_lead: f64,
    climb_lag: f64,
    climb_lead: f64,
    /// Highest attitude on the wheels, and anywhere.
    max_theta_wheels: f64,
    max_theta: f64,
    /// Turns of the attitude, up to down or back, of more than 0.3 degree,
    /// from 8 ft on: one is the nose stopping at the climb attitude.
    turns: usize,
    /// At 300 ft, the autopilot's engage: the climb the flight model reads
    /// (what `ClimbVs` hands the autopilot), fpm, and the airspeed.
    engage_fpm: f64,
    engage_kts: f64,
}

/// Flies `law` against the plant from the rotate at its airspeed, at `tick`
/// seconds with a jitter and the runway's rate noise, the law reading the
/// wheels, the airspeed, the height and the lagged vertical speed as the
/// loop does, to 300 ft.
fn fly(mut law: RotatePitch, mut plant: ZiboRotation, tick: f64, seed: u32) -> Flown {
    let mut seed = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
    let mut noise = move || {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        f64::from(seed >> 8) / f64::from(1_u32 << 23) - 1.0
    };
    let mut pending: Vec<(f64, f64)> = Vec::new();
    let mut applied = 0.0;
    let mut t = 0.0_f64;
    let (mut off, mut liftoff) = (f64::NAN, f64::NAN);
    let (mut rotate_lag, mut rotate_lead) = (f64::MIN, f64::MIN);
    let (mut climb_lag, mut climb_lead) = (f64::MIN, f64::MIN);
    let (mut max_wheels, mut max_theta) = (f64::MIN, f64::MIN);
    // The attitude's last turn and which way it has gone since.
    let (mut extreme, mut rising, mut turns) = (f64::NAN, true, 0);
    let mut dt = tick;
    while plant.h < 300.0 && t < 25.0 {
        let on = plant.wheels;
        let q_read = plant.q + noise() * if on { 0.3 } else { 0.05 };
        let theta_read = plant.theta + 0.03 * noise();
        law.on_wheels(on);
        law.climb_fpm(Some(plant.vs_read));
        // `dt` is the time since the last tick, as the loop measures it.
        let cmd = law.step_climb(theta_read, Some(q_read), dt, Some((plant.v, plant.h)));
        // The target this tick's ramp ends on; the nose is judged against
        // the one it was asked for on the last.
        let asked = law.target_deg() - 2.5 * dt;
        let (lag, lead) = (asked - plant.theta, plant.theta - law.target_deg());
        if liftoff.is_nan() {
            (rotate_lag, rotate_lead) = (rotate_lag.max(lag), rotate_lead.max(lead));
        } else {
            (climb_lag, climb_lead) = (climb_lag.max(lag), climb_lead.max(lead));
            if extreme.is_nan() {
                extreme = plant.theta;
            } else if rising == (plant.theta >= extreme) {
                extreme = plant.theta;
            } else if (plant.theta - extreme).abs() > 0.3 {
                (rising, extreme, turns) = (!rising, plant.theta, turns + 1);
            }
        }
        pending.push((t + ZiboRotation::DELAY_S, ZiboRotation::as_held(cmd)));
        dt = tick + 0.2 * tick * noise();
        let end = t + dt;
        while t < end - 1e-9 {
            let next = pending
                .iter()
                .map(|&(at, _)| at)
                .filter(|&at| at > t && at < end)
                .fold(end, f64::min)
                .min(t + 0.01);
            plant.advance(applied, next - t, None);
            t = next;
            while let Some(i) = pending.iter().position(|&(at, _)| at <= t) {
                applied = pending.remove(i).1;
            }
            if !plant.wheels && off.is_nan() {
                off = plant.v;
            }
            if plant.h > 8.0 && liftoff.is_nan() {
                liftoff = plant.v;
            }
            if plant.wheels {
                max_wheels = max_wheels.max(plant.theta);
            }
            max_theta = max_theta.max(plant.theta);
        }
    }
    Flown {
        wheels_off_kts: off,
        liftoff_kts: liftoff,
        rotate_lag,
        rotate_lead,
        climb_lag,
        climb_lead,
        max_theta_wheels: max_wheels,
        max_theta,
        turns,
        engage_fpm: plant.vs_read,
        engage_kts: plant.v,
    }
}

/// The plant with the nose flown exactly on a 2.5 deg/s ramp from the roll
/// attitude to the 9 degree ceiling: the airspeed at 8 ft a rotation at the
/// airline rate earns.
fn ideal_liftoff_kts(vr: f64, weight_t: f64) -> f64 {
    let mut p = ZiboRotation::new(vr, weight_t);
    let mut t = 0.0;
    while p.h < 8.0 {
        let theta = (ZiboRotation::REST_DEG + 2.5 * t).min(9.0);
        (p.theta, p.q) = (theta, 0.0);
        p.advance(0.0, 0.01, None);
        p.theta = theta;
        t += 0.01;
    }
    p.v
}

/// The law as it flew both rotations: the 737's tail and push, the hold at
/// V2 + 15, no model.
fn law_as_flown(hold_kts: f64) -> RotatePitch {
    RotatePitch::with_authority(Some(11.0), -0.5).with_climb_hold(Some(ClimbHold {
        hold_kts,
        pitch_max_deg: 18.0,
    }))
}

/// The Zibo's declared rotation model.
fn zibo_model() -> crate::aircraft::RotateElevatorProfile {
    let text = std::fs::read_to_string("profiles/zibo_738.yaml").unwrap();
    let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
    p.rotate_elevator
        .and_then(|m| m.valid())
        .expect("the Zibo declares its rotation")
}

/// The law as the Zibo flies it now.
fn law_declared(hold_kts: f64) -> RotatePitch {
    law_as_flown(hold_kts).with_rotate_model(Some(zibo_model()))
}

/// The two weights flown, each at the Vr and V2 its card gives (43.7 t:
/// Vr 108, V2 124, 1791374430's; 45.7 t: Vr 111, V2 127, what 1791371247
/// would rotate at now).
const CASES: [(f64, f64, f64); 2] = [(43.7, 108.0, 124.0), (45.7, 111.0, 127.0)];

/// Against the plant, the law as flown is the flight: 1791374430 (43.7 t,
/// Vr 108) wheels off 144.5, 8 ft 153; 1791371247 (45.7 t, Vr 145) wheels
/// off 162.6, 8 ft 173, the nose 4 to 6 degrees behind the ramp both times.
#[test]
fn the_flown_law_rotates_late_on_the_plant() {
    for (w, vr, hold, off_flown, liftoff_flown) in [
        (43.7, 108.0, 139.0, 144.5, 153.4),
        (45.7, 145.0, 165.0, 162.6, 172.9),
    ] {
        for seed in 0..4 {
            let f = fly(law_as_flown(hold), ZiboRotation::new(vr, w), 0.25, seed);
            let case = format!("{w} t, seed {seed}: {f:?}");
            assert!((f.wheels_off_kts - off_flown).abs() < 8.0, "{case}");
            assert!((f.liftoff_kts - liftoff_flown).abs() < 4.0, "{case}");
            assert!(f.rotate_lag > 4.0, "{case}");
        }
    }
}

/// The fix: fed the declared model, the nose rides the 2.5 deg/s ramp
/// within 1.5 degrees from the rotate to 8 ft and lifts off within 10 kt of
/// a rotation flown exactly on the ramp (8 ft at 136 and 138 kt where the
/// law as flown reached 151 and 153), the tail at least 5 degrees clear on
/// the wheels, at 0.2 to 0.3 s ticks.
/// Climbing, the nose stays within 1.5 degrees of the hold's walk and goes
/// no higher than the law as flown takes it over the 18 degree ceiling (the
/// ceiling guard's, on this plant 19.0 as flown and 18.8 now; 18.5 in
/// 1791374430), turning at most twice.
#[test]
fn the_zibo_rotates_on_its_ramp() {
    for (w, vr, v2) in CASES {
        let ideal = ideal_liftoff_kts(vr, w);
        for tick in [0.2, 0.25, 0.3] {
            for seed in 0..6 {
                let plant = || ZiboRotation::new(vr, w);
                let old = fly(law_as_flown(v2 + 15.0), plant(), tick, seed);
                let f = fly(law_declared(v2 + 15.0), plant(), tick, seed);
                let case = format!("{w} t, tick {tick}, seed {seed}: {f:?}, as flown {old:?}");
                assert!(f.rotate_lag < 1.5 && f.rotate_lead < 1.5, "{case}");
                assert!(f.liftoff_kts < ideal + 10.0, "{case} (ideal {ideal:.1})");
                assert!(f.liftoff_kts < old.liftoff_kts - 10.0, "{case}");
                assert!(f.max_theta_wheels < 6.0, "{case}");
                assert!(f.climb_lag < 1.5 && f.climb_lead < 1.5, "{case}");
                assert!(f.max_theta < old.max_theta && f.max_theta < 19.0, "{case}");
                assert!(f.turns <= 2, "{case}");
            }
        }
    }
}

/// The model is a hypothesis about the plant: on an elevator 0.75 to 1.35
/// times the fit, the wheels' spring 0.75 to 1.35 times, and the next air
/// fit, the rotation still rides its ramp, lifts off within 12 kt of the
/// ideal and 8 kt before the law as flown, the tail clear, the climb no
/// higher than the law as flown takes it. The nose is within 1.5 degrees of
/// the ramp but on the stiffer spring, 2.0 behind, and 2.5 where that spring
/// meets the weaker elevator (the column it needs is 1.8 times the model's;
/// the law as flown lagged 6.6 there).
#[test]
fn the_zibo_rotation_survives_a_plant_not_as_fitted() {
    for (w, vr, v2) in CASES {
        let ideal = ideal_liftoff_kts(vr, w);
        for k_mult in [0.75, 1.0, 1.35] {
            for spring_mult in [0.75, 1.0, 1.35] {
                for air in [Air::FIT, Air::NEXT] {
                    let plant = || ZiboRotation {
                        k_mult,
                        spring_mult,
                        air,
                        ..ZiboRotation::new(vr, w)
                    };
                    let old = fly(law_as_flown(v2 + 15.0), plant(), 0.25, 1);
                    let f = fly(law_declared(v2 + 15.0), plant(), 0.25, 1);
                    let case = format!(
                        "{w} t, elevator x{k_mult}, spring x{spring_mult}, {air:?}: {f:?}, as flown {old:?}"
                    );
                    assert!(f.rotate_lag < 2.75 && f.rotate_lead < 1.5, "{case}");
                    assert!(f.climb_lag < 2.5 && f.climb_lead < 1.5, "{case}");
                    assert!(f.liftoff_kts < ideal + 12.0, "{case} (ideal {ideal:.1})");
                    assert!(f.liftoff_kts < old.liftoff_kts - 8.0, "{case}");
                    assert!(f.max_theta_wheels < 6.0, "{case}");
                    assert!(f.max_theta < old.max_theta + 0.2, "{case}");
                    assert!(f.turns <= 3, "{case}");
                }
            }
        }
    }
}

/// The thrust's share of the levers at 1.0 for an N1, the CFM56's thrust
/// going as N1^2.3 near the top (the 27K and 22K tables' 100.8 and 91.6 at
/// 15 C on the field are 27/22 thrust).
fn thrust_at_n1(n1: f64) -> f64 {
    (n1 / 104.0).powf(2.3)
}

/// "It still felt steep": the levers at 1.0 (104 % N1) and the 18 degree
/// climb attitude handed the light Zibo's autopilot 4,500 to 4,700 fpm at
/// its 300 ft engage (LEMG, 2026-10-07; this plant 4,100 to 4,200 at 18).
/// On the declared law (`takeoff_thrust:`, `climb_pitch_max_deg`) a 44 to
/// 46 t takeoff from LEMG's 3,200 m at 15 C is 22K at an assumed 60 C, N1
/// 86.1 (two thirds of the levers' thrust): the same rotation and V2 + 15
/// hold hand the autopilot 2,000 to 3,000 fpm (on this plant 2,400 and
/// 2,260) near V2 + 15, at least 900 fpm under the full levers at the same
/// attitude. A runway with no room for the assumed temperature flies the
/// 22K's own N1 91.6 into 3,000 fpm or less (2,790 and 2,740).
#[test]
fn a_derated_takeoff_hands_the_autopilot_a_sane_climb() {
    let p: crate::aircraft::AircraftProfile =
        serde_yaml::from_str(&std::fs::read_to_string("profiles/zibo_738.yaml").unwrap()).unwrap();
    let law = p.takeoff_thrust.clone().expect("the Zibo declares its takeoff thrust");
    let ceiling = p.climb_pitch_max_deg.expect("the Zibo declares its climb attitude");
    // LEMG 13 and what the 26K needs at these weights (the profile's
    // 5,500 ft roll at its weight, times 1.4), and a runway with room for
    // the 22K alone.
    let lemg = crate::takeoff_thrust::RunwayFit { needed_full_m: 1310.0, available_m: 3200.0 };
    let short = crate::takeoff_thrust::RunwayFit { available_m: 1900.0, ..lemg };
    for (w, vr, v2) in [(44.0, 108.0, 124.0), (45.7, 111.0, 127.0)] {
        let fly_at = |n1: f64| {
            let plant = ZiboRotation {
                thrust: thrust_at_n1(n1),
                ..ZiboRotation::new(vr, w)
            };
            let law = RotatePitch::with_authority(Some(11.0), -0.5)
                .with_climb_hold(Some(ClimbHold {
                    hold_kts: v2 + 15.0,
                    pitch_max_deg: ceiling,
                }))
                .with_rotate_model(Some(zibo_model()));
            fly(law, plant, 0.25, 1)
        };
        let plan = crate::takeoff_thrust::plan(&law, w, 15.0, 50.0, 1000.0, Some(lemg)).unwrap();
        assert_eq!((plan.rating.as_str(), plan.assumed_c), ("22K", Some(60.0)), "{plan:?}");
        let rated = crate::takeoff_thrust::plan(&law, w, 15.0, 50.0, 1000.0, Some(short)).unwrap();
        assert_eq!((rated.rating.as_str(), rated.assumed_c), ("22K", None), "{rated:?}");
        let full = fly_at(104.0);
        let atm = fly_at(plan.to_n1);
        let own = fly_at(rated.to_n1);
        let case = format!(
            "{w} t at {ceiling} deg: full {:.0} fpm at {:.0} kt; 22K assumed N1 {:.1}: {:.0} fpm at {:.0} kt; \
             22K N1 {:.1}: {:.0} fpm at {:.0} kt",
            full.engage_fpm, full.engage_kts, plan.to_n1, atm.engage_fpm, atm.engage_kts,
            rated.to_n1, own.engage_fpm, own.engage_kts
        );
        assert!((2000.0..=3000.0).contains(&atm.engage_fpm), "{case}");
        assert!(atm.engage_fpm < full.engage_fpm - 900.0, "{case}");
        assert!((atm.engage_kts - (v2 + 15.0)).abs() < 10.0, "{case}");
        assert!((2000.0..=3000.0).contains(&own.engage_fpm), "{case}");
        assert!((own.engage_kts - (v2 + 15.0)).abs() < 10.0, "{case}");
    }
}

/// The declaration: the fit's wheels as they are, the air's spring and base
/// the fit's, the path from the height. The SR22, the only other model,
/// declares neither new field and stays as it was.
#[test]
fn the_zibo_declares_its_fitted_rotation() {
    let m = zibo_model();
    assert_eq!((m.ref_kts, m.breakout, m.per_deg), (130.0, 0.033, 0.077));
    assert!((m.deg_s_per_unit - ZiboRotation::K).abs() < 0.1, "{m:?}");
    assert_eq!(m.air_per_deg, Some(Air::FIT.per_deg));
    assert!((m.air_base - Air::FIT.base).abs() < 0.01, "{m:?}");
    assert!(
        (m.ground_effect_ft - Air::FIT.ground_effect_ft).abs() < 0.5,
        "{m:?}"
    );
    assert!(m.path_from_height);
    for entry in std::fs::read_dir("profiles").unwrap() {
        let path = entry.unwrap().path();
        let text = std::fs::read_to_string(&path).unwrap();
        let p: crate::aircraft::AircraftProfile = serde_yaml::from_str(&text).unwrap();
        let Some(m) = p.rotate_elevator else { continue };
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        if name != "zibo_738.yaml" {
            assert_eq!(name, "cirrus_sr22.yaml");
            assert_eq!((m.air_per_deg, m.path_from_height), (None, false), "{name}");
        }
    }
}
