//! The final's lateral law on heading modes FITTED to seven airframes'
//! recorded finals (2026-10-12). One shape fits all seven: the autopilot
//! banks `k` degrees per degree of the bug less heading, low-passed over
//! `tf`, plus a standing bias `b`, through a first-order roll lag `tau`;
//! the track turns at g tan(bank) / TAS; the heading carries a yaw the
//! track does not. Fitted by least squares on the bank over 40-tick windows
//! of the intercept and final, the recorded knob (`heading_set`) as input:
//! the 172 on 27 finals (206 windows, LEMG 12 and 13, 1791739518 to
//! 1791814879), the SR22 on 14 (428, 1791791921 to 1791814508), the King
//! Air C90B on 3 (61, 1791799361 to 1791815553), the Evolution on 6 (154,
//! 1791801818 to 1791818389), the Baron 58 on 4 (91, 1791802802 to
//! 1791816184), the SF50 on 3 (77, 1791805150 to 1791809604) and the Zibo
//! 737 on its LEMG 30 finals (`bench/approach/lemg30_zibo_2026-10-07.jsonl`,
//! a 1 s delay and a whole-degree window). The yaw is the heading less
//! track about its 15 s mean on the same finals, rms and correlation time.

use super::*;
use crate::autoland::{bug_knob, ApproachPhase};

const G: f64 = 9.81;
const KT: f64 = 0.514_444;

/// One airframe's heading mode as fitted.
#[derive(Clone, Copy, Debug)]
struct Plant {
    name: &'static str,
    /// Bank per degree of heading error, the error's low-pass (s), the roll
    /// lag (s), the standing bank bias (deg).
    k: f64,
    tf: f64,
    tau: f64,
    b: f64,
    /// The knob answered this late (s), in steps of this (deg, 0: as set).
    delay_s: f64,
    window_deg: f64,
    /// The approach's true airspeed, kt.
    tas_kts: f64,
    /// The yaw on the heading: rms (deg) and correlation time (s).
    yaw_deg: f64,
    yaw_s: f64,
    /// The response time a 2.4 degree knob step gives on this plant
    /// (the analysis' integral time), s.
    t_r_step: f64,
}

const PLANTS: [Plant; 7] = [
    Plant {
        name: "C172",
        k: 0.568,
        tf: 0.366,
        tau: 1.43,
        b: -0.095,
        delay_s: 0.0,
        window_deg: 0.0,
        tas_kts: 65.0,
        yaw_deg: 0.37,
        yaw_s: 1.8,
        t_r_step: 6.0,
    },
    Plant {
        name: "SR22",
        k: 0.764,
        tf: 0.948,
        tau: 1.255,
        b: -0.202,
        delay_s: 0.0,
        window_deg: 0.0,
        tas_kts: 78.0,
        yaw_deg: 0.37,
        yaw_s: 1.2,
        t_r_step: 5.3,
    },
    Plant {
        name: "King Air C90B",
        k: 2.501,
        tf: 0.351,
        tau: 1.69,
        b: -0.606,
        delay_s: 0.0,
        window_deg: 0.0,
        tas_kts: 105.0,
        yaw_deg: 0.22,
        yaw_s: 1.1,
        t_r_step: 2.2,
    },
    Plant {
        name: "Evolution",
        k: 2.087,
        tf: 0.526,
        tau: 1.137,
        b: -0.527,
        delay_s: 0.0,
        window_deg: 0.0,
        tas_kts: 95.0,
        yaw_deg: 1.52,
        yaw_s: 1.8,
        t_r_step: 2.4,
    },
    Plant {
        name: "Baron 58",
        k: 1.002,
        tf: 0.609,
        tau: 0.921,
        b: -0.243,
        delay_s: 0.0,
        window_deg: 0.0,
        tas_kts: 98.0,
        yaw_deg: 0.60,
        yaw_s: 1.3,
        t_r_step: 5.1,
    },
    Plant {
        name: "SF50",
        k: 2.044,
        tf: 0.645,
        tau: 1.193,
        b: -0.558,
        delay_s: 0.0,
        window_deg: 0.0,
        tas_kts: 85.0,
        yaw_deg: 1.08,
        yaw_s: 1.4,
        t_r_step: 2.1,
    },
    Plant {
        name: "Zibo 737",
        k: 1.08,
        tf: 1.26,
        tau: 4.59,
        b: 0.10,
        delay_s: 1.0,
        window_deg: 1.0,
        tas_kts: 140.0,
        yaw_deg: 0.49,
        yaw_s: 1.5,
        t_r_step: 7.5,
    },
];

/// A deterministic Gaussian source (xorshift64*, Box-Muller).
struct Noise(u64);

impl Noise {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn gauss(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()
    }
}

/// The air: a crosswind from the right (kt) from the final's start to its
/// end, linearly between.
#[derive(Clone, Copy, Debug)]
struct Air {
    xwind_start_kts: f64,
    xwind_end_kts: f64,
}

const CALM: Air = Air {
    xwind_start_kts: 0.0,
    xwind_end_kts: 0.0,
};
const XWIND_15: Air = Air {
    xwind_start_kts: 15.0,
    xwind_end_kts: 15.0,
};
const SHEAR_15_0: Air = Air {
    xwind_start_kts: 15.0,
    xwind_end_kts: 0.0,
};

/// The heading mode under the loop: the track, bank and offset from the
/// course 0, the autopilot's error filter, the knob as the window takes it
/// (late, rounded) and the yaw.
struct Flight {
    p: Plant,
    trk: f64,
    phi: f64,
    x: f64,
    ef: f64,
    yaw: f64,
    knobs: std::collections::VecDeque<(f64, f64)>,
    t: f64,
    noise: Noise,
}

impl Flight {
    fn new(p: Plant, x0: f64, crab: f64, knob: f64, seed: u64) -> Self {
        let mut knobs = std::collections::VecDeque::new();
        knobs.push_back((-100.0, knob));
        // Trimmed on its bug: the bias held by the standing error.
        let ef = -p.b / p.k;
        Self {
            p,
            trk: knob - crab - ef,
            phi: 0.0,
            x: x0,
            ef,
            yaw: 0.0,
            knobs,
            t: 0.0,
            noise: Noise(seed),
        }
    }

    /// The knob the autopilot answers now: the last set `delay_s` ago, in
    /// the window's steps.
    fn seen(&self) -> f64 {
        let mut v = self.knobs[0].1;
        for &(at, k) in &self.knobs {
            if at <= self.t - self.p.delay_s + 1e-9 {
                v = k;
            } else {
                break;
            }
        }
        if self.p.window_deg > 0.0 {
            (v / self.p.window_deg).round() * self.p.window_deg
        } else {
            v
        }
    }

    fn set_knob(&mut self, knob: f64) {
        self.knobs.push_back((self.t, knob));
        while self.knobs.len() > 2 && self.knobs[1].0 <= self.t - self.p.delay_s - 1.0 {
            self.knobs.pop_front();
        }
    }

    /// The heading read: the nose on the crab plus the yaw.
    fn psi(&self, crab: f64) -> f64 {
        self.trk + crab + self.yaw
    }

    /// `dt` of flight on the crab `crab` (deg) turning at `crab_rate`
    /// (deg/s, the shear) and the groundspeed `gs_kts`.
    fn fly(&mut self, dt: f64, crab: f64, crab_rate: f64, gs_kts: f64) {
        let n = (dt / 0.1).round().max(1.0) as usize;
        let h = dt / n as f64;
        let v = self.p.tas_kts * KT;
        let a = (h / self.p.yaw_s).min(1.0);
        for _ in 0..n {
            self.yaw += -a * self.yaw + self.p.yaw_deg * (2.0 * a).sqrt() * self.noise.gauss();
            let e = self.seen() - self.psi(crab);
            self.ef += (e - self.ef) * (h / self.p.tf).min(1.0);
            let c = (self.p.k * self.ef + self.p.b).clamp(-25.0, 25.0);
            self.phi += (c - self.phi) * h / self.p.tau;
            // The wind dying with the heading held turns the track downwind.
            self.trk += ((G * self.phi.to_radians().tan() / v).to_degrees() - crab_rate) * h;
            self.x += gs_kts * KT * self.trk.to_radians().sin() * h;
            self.t += h;
        }
    }
}

/// What a final came to: the offset at 50 ft over the threshold, the
/// knob's moves (time, size) and the response time the loop measured.
#[derive(Debug)]
struct Flown {
    x50: f64,
    worst_last_mile: f64,
    moves: Vec<(f64, f64)>,
    t_r: f64,
    measured: bool,
}

const TICK_S: f64 = 0.33;
/// The intercept's tick, s (`tick_ms`).
const INTERCEPT_TICK_S: f64 = 0.54;

/// The response time as the loop arrives at the final with it: measured on
/// an intercept, the bug turned 8 degrees in, back onto the course, 4 in
/// and back, 15 s apart (the turn onto the course and its corrections).
fn intercept(p: Plant, seed: u64) -> ResponseTime {
    let mut f = Flight::new(p, 0.0, 0.0, 0.0, seed);
    let mut tr = ResponseTime::default();
    let mut knob = 0.0;
    for (i, k) in [-8.0, 0.0, -4.0, 0.0].into_iter().enumerate() {
        while f.t < 15.0 * (i + 1) as f64 {
            tr.feed(
                Some(knob),
                Some(f.psi(0.0)),
                Some(f.trk),
                INTERCEPT_TICK_S,
                true,
            );
            f.fly(INTERCEPT_TICK_S, 0.0, 0.0, p.tas_kts);
        }
        knob = k;
        f.set_knob(knob);
    }
    tr
}

/// The final flown as the loop flies it, from `start_nm` and `x0` metres
/// right of the centreline on the runway course, to the threshold.
fn fly(p: Plant, air: Air, x0: f64, start_nm: f64, seed: u64, yaw: bool) -> Flown {
    let p = if yaw { p } else { Plant { yaw_deg: 0.0, ..p } };
    let course = 0.0;
    let crab_of = |xw: f64| (xw / p.tas_kts).asin().to_degrees();
    let gs_of = |xw: f64| (p.tas_kts * p.tas_kts - xw * xw).sqrt();
    let dur = start_nm * crate::autoland::M_PER_NM
        / (gs_of(air.xwind_start_kts.max(air.xwind_end_kts)) * KT);
    let xw_at = |t: f64| {
        air.xwind_start_kts + (air.xwind_end_kts - air.xwind_start_kts) * (t / dur).clamp(0.0, 1.0)
    };
    let crab0 = crab_of(xw_at(0.0));
    let mut f = Flight::new(p, x0, crab0, course + crab0, seed);
    let mut crab_filter = crate::autoland::CrabFilter::new();
    crab_filter.update(Some(crab0), 0.0);
    let mut law = FinalLateral::default();
    let mut tr = intercept(p, seed + 100);
    let mut hold = crate::heading_bug::BugHold::default();
    let (key, db) =
        bug_knob(ApproachPhase::Final, false, None, None).expect("the final's knob is keyed");
    hold.decide(key, course + crab0, 0.0, db);
    let mut moves = Vec::new();
    let mut knob = course + crab0;
    let mut along = start_nm * crate::autoland::M_PER_NM;
    let mut worst_last_mile: f64 = 0.0;
    while along > 0.0 {
        let xw = xw_at(f.t);
        let crab = crab_of(xw);
        let gs = gs_of(xw);
        let psi = f.psi(crab);
        let c = crab_filter.update(
            crate::autoland::CrabFilter::raw(Some(psi), Some(f.trk)),
            TICK_S,
        );
        let side = gs * KT * f.trk.to_radians().sin();
        let radio_ft = along / crate::autoland::M_PER_NM * 318.0 + 50.0;
        tr.feed(
            Some(knob),
            Some(psi),
            Some(f.trk),
            TICK_S,
            f.t <= TR_FINAL_WINDOW_S && radio_ft > TR_MIN_RADIO_FT,
        );
        let want = law.bug_true(f.x, side, gs, course, c, tr.t_r(), TICK_S);
        let want = crate::angles::signed_diff_deg(want, 0.0);
        if let Some(w) = hold.decide(key, want, f.t, db) {
            let w = crate::angles::signed_diff_deg(w, 0.0);
            if (w - knob).abs() > 1e-9 {
                moves.push((f.t, w - knob));
                knob = w;
                f.set_knob(knob);
            }
        }
        if along < crate::autoland::M_PER_NM {
            worst_last_mile = worst_last_mile.max(f.x.abs());
        }
        let rate = (crab_of(xw_at(f.t + TICK_S)) - crab) / TICK_S;
        f.fly(TICK_S, crab, rate, gs);
        along -= gs * KT * f.trk.to_radians().cos() * TICK_S;
    }
    Flown {
        x50: f.x,
        worst_last_mile,
        moves,
        t_r: tr.t_r(),
        measured: tr.measured(),
    }
}

/// Every fitted airframe handed the final at 5 nm 40 m right, 80 m left
/// and on the line, each twice (another seed of yaw where it is flown),
/// the response time measured on its intercept: within `within_m` at 50 ft
/// over the threshold.
fn every_airframe_lands_on_the_line_in(air: Air, what: &str, yaw: bool, within_m: f64) {
    let mut bad = Vec::new();
    for p in PLANTS {
        for (x0, seed) in [
            (40.0, 1u64),
            (-80.0, 2),
            (40.0, 3),
            (-80.0, 4),
            (0.0, 5),
            (0.0, 6),
        ] {
            let o = fly(p, air, x0, 5.0, seed, yaw);
            eprintln!(
                "{what} {:<14} x0 {x0:>6.0}: x50 {:>5.1} m, worst last mile {:>5.1}, t_r {:.1} ({}), {} moves",
                p.name,
                o.x50,
                o.worst_last_mile,
                o.t_r,
                if o.measured { "measured" } else { "default" },
                o.moves.len()
            );
            if o.x50.abs() > within_m {
                bad.push(format!(
                    "{} from {x0} m (seed {seed}): {:.1} m",
                    p.name, o.x50
                ));
            }
        }
    }
    assert!(bad.is_empty(), "{what}: off the line at 50 ft: {bad:?}");
}

#[test]
fn every_fitted_airframe_lands_on_the_line_in_calm_air() {
    every_airframe_lands_on_the_line_in(CALM, "calm", false, 5.0);
}

#[test]
fn every_fitted_airframe_lands_on_the_line_in_a_15_kt_crosswind() {
    every_airframe_lands_on_the_line_in(XWIND_15, "xwind 15", false, 5.0);
}

#[test]
fn every_fitted_airframe_lands_on_the_line_in_a_15_to_0_kt_shear() {
    every_airframe_lands_on_the_line_in(SHEAR_15_0, "shear 15-0", false, 5.0);
}

/// With the yaw the heading carried on the recorded finals (the Evolution
/// 1.5 degrees rms, the SF50 1.1), which the autopilot banks against and
/// the estimate reads as error never answered: within 8 m (the Evolution
/// 6.7 m at worst, the Zibo 5.8; the analysis' replays of the recorded
/// finals, their own yaw included, kept 58 of 60 within 5 m).
#[test]
fn every_fitted_airframe_lands_near_the_line_with_its_recorded_yaw() {
    every_airframe_lands_on_the_line_in(CALM, "calm, yaw", true, 8.0);
}

/// The knob is set as a pilot sets it: no oftener than every second and
/// by a quarter degree or more (a move of five degrees or more is a turn,
/// set at once: `BugHold`).
#[test]
fn the_knob_moves_no_oftener_than_a_second_and_never_by_less_than_a_quarter_degree() {
    for p in PLANTS {
        for air in [CALM, XWIND_15, SHEAR_15_0] {
            let o = fly(p, air, -120.0, 5.0, 7, true);
            for w in o.moves.windows(2) {
                let ((t0, _), (t1, d)) = (w[0], w[1]);
                assert!(
                    d.abs() >= crate::heading_bug::BUG_STEER_DEG || t1 - t0 >= 1.0 - 1e-9,
                    "{}: knob moved {d:.2} deg {:.2} s after the last",
                    p.name,
                    t1 - t0
                );
            }
            for &(t, d) in &o.moves {
                assert!(
                    d.abs() >= 0.25 - 1e-9,
                    "{}: a {d:.3} deg move at {t:.1} s",
                    p.name
                );
            }
        }
    }
}

/// The response time measured in flight, on the intercept and the first
/// 90 s of the final, recovers the plant's: within the estimate's bounds and
/// within what the yaw lends it (the heading's yaw reads as error the track
/// never answers: in flight 3.9 s on the King Air against its 2.2).
#[test]
fn the_response_time_measured_in_flight_recovers_each_plants() {
    for p in PLANTS {
        let o = fly(p, CALM, -120.0, 5.0, 11, false);
        assert!(o.measured, "{}: no estimate", p.name);
        let want = p.t_r_step.clamp(TR_MIN_S, TR_MAX_S);
        eprintln!(
            "{:<14} t_r measured {:.2} s, the step's {:.1} (bounded {want:.1})",
            p.name, o.t_r, p.t_r_step
        );
        assert!((TR_MIN_S..=TR_MAX_S).contains(&o.t_r));
        assert!(
            o.t_r >= want * 0.8 && o.t_r <= want * 1.6,
            "{}: measured {:.2} s against {want:.1}",
            p.name,
            o.t_r
        );
    }
}

/// The step response the plants' `t_r_step` is: the integral of what is
/// left of a 2.4 degree knob step over 40 s (`step.py`).
#[test]
fn the_plants_step_response_times_are_the_analysis() {
    for p in PLANTS {
        let mut f = Flight::new(
            Plant {
                b: 0.0,
                yaw_deg: 0.0,
                window_deg: 0.0,
                ..p
            },
            0.0,
            0.0,
            0.0,
            1,
        );
        f.ef = 0.0;
        f.trk = 0.0;
        f.set_knob(2.4);
        let mut ti = 0.0;
        let h = 0.02;
        for _ in 0..(40.0 / h) as usize {
            ti += (1.0 - f.trk / 2.4) * h;
            f.fly(h, 0.0, 0.0, p.tas_kts);
        }
        assert!(
            (ti - p.t_r_step).abs() < 0.25,
            "{}: {ti:.2} s against {}",
            p.name,
            p.t_r_step
        );
    }
}

#[test]
fn the_law_closes_left_of_a_right_offset_and_crabs_into_the_wind() {
    let mut l = FinalLateral::default();
    let b = l.bug_true(30.0, 0.0, 100.0, 90.0, 0.0, 5.0, 0.0);
    assert!(b < 90.0, "right of the line steers left: {b}");
    let mut l = FinalLateral::default();
    let b = l.bug_true(0.0, 0.0, 100.0, 90.0, 4.0, 5.0, 0.0);
    assert!(
        (b - 94.0).abs() < 1e-9,
        "on the line: the course plus the crab: {b}"
    );
    let mut l = FinalLateral::default();
    let b = l.bug_true(0.0, -2.0, 100.0, 90.0, 0.0, 5.0, 0.0);
    assert!(
        b > 90.0,
        "drifting left steers right before the offset shows: {b}"
    );
    let mut l = FinalLateral::default();
    let b = l.bug_true(5_000.0, 0.0, 100.0, 90.0, 0.0, 5.0, 0.0);
    assert!((b - 60.0).abs() < 1e-9, "the cut is capped at 30: {b}");
}

#[test]
fn the_integral_is_bounded() {
    let mut l = FinalLateral::default();
    for _ in 0..10_000 {
        l.bug_true(50.0, 0.0, 100.0, 0.0, 0.0, 5.0, 0.33);
    }
    assert_eq!(l.i_m(), LATERAL_I_MAX_M);
    l.reset();
    assert_eq!(l.i_m(), 0.0);
}

#[test]
fn no_evidence_is_the_default_response_time() {
    let mut r = ResponseTime::default();
    assert_eq!(r.t_r(), TR_DEFAULT_S);
    for i in 0..30 {
        r.feed(Some(2.0), Some(0.0), Some(i as f64 * 0.1), 0.33, true);
    }
    assert!(!r.measured(), "under {TR_MIN_SAMPLES} ticks is no estimate");
    assert_eq!(r.t_r(), TR_DEFAULT_S);
    // A track that turns away from the knob is no response at all.
    let mut r = ResponseTime::default();
    for i in 0..100 {
        r.feed(Some(2.0), Some(0.0), Some(-(i as f64) * 0.1), 0.33, true);
    }
    assert_eq!(r.t_r(), TR_DEFAULT_S);
}
