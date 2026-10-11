use super::*;

/// A 172 on approach: thrust against drag and the path's slope, the
/// autopilot holding an altitude or a path through a 1.5 s lag on the
/// flight path, the engine 0.5 s behind the lever, the law ticking at 0.33 s
/// (the approach's tick). Level at 65 kt wants about half power.
struct Plant {
    v_fps: f64,
    h_ft: f64,
    gamma: f64,
    thrust: f64,
    path_vs_fpm: f64,
    /// The path is level until this time, then `path_vs_fpm` (a capture).
    path_from_s: f64,
    /// Drag multiplied by this from `flap_at_s` on (full flap out).
    flap_drag: f64,
    flap_at_s: f64,
}

const A_T: f64 = 6.0; // ft/s^2 per unit of throttle
const DRAG_C: f64 = 3.0 / (65.0 * FPS_PER_KT * 65.0 * FPS_PER_KT);

impl Plant {
    fn new(kts: f64, path_vs_fpm: f64) -> Self {
        Plant { v_fps: kts * FPS_PER_KT, h_ft: 2_000.0, gamma: 0.0, thrust: 0.5, path_vs_fpm, path_from_s: 0.0, flap_drag: 1.0, flap_at_s: f64::INFINITY }
    }
    fn vs_fpm(&self) -> f64 {
        self.v_fps * self.gamma.sin() * 60.0
    }
    /// The autopilot: the path's rate plus the height error closed in 20 s,
    /// within 1,000 fpm, the path following through 1.5 s.
    fn path_at(&self, t: f64) -> f64 {
        if t < self.path_from_s { 0.0 } else { self.path_vs_fpm }
    }
    fn step(&mut self, thr: f64, h_ref: f64, path_vs: f64, t: f64, dt: f64) {
        self.thrust += (thr - self.thrust) * dt / 0.5;
        let vs_cmd = (path_vs + (h_ref - self.h_ft) * 3.0).clamp(-1_000.0, 1_000.0);
        let gamma_cmd = (vs_cmd / 60.0 / self.v_fps).clamp(-0.3, 0.3).asin();
        self.gamma += (gamma_cmd - self.gamma) * dt / 1.5;
        let drag = if t >= self.flap_at_s { self.flap_drag } else { 1.0 };
        let accel = A_T * self.thrust - drag * DRAG_C * self.v_fps * self.v_fps - G_FPS2 * self.gamma.sin();
        self.v_fps += accel * dt;
        self.h_ft += self.v_fps * self.gamma.sin() * dt;
    }
}

struct Run {
    kts: Vec<f64>,
    vs: Vec<f64>,
    thr: Vec<f64>,
}

fn fly(mut p: Plant, target_kts: f64, secs: f64) -> Run {
    let (dt, tick) = (0.02, 0.33);
    let mut law = EnergyThrottle::default();
    let (mut thr, mut t, mut next, mut prev_kts) = (0.5, 0.0, 0.0, p.v_fps / FPS_PER_KT);
    let mut run = Run { kts: vec![], vs: vec![], thr: vec![] };
    while t < secs {
        if t >= next {
            let kts = p.v_fps / FPS_PER_KT;
            thr = law.step(In {
                ias_kts: kts,
                accel_kts_s: (kts - prev_kts) / tick,
                vs_fpm: p.vs_fpm(),
                target_kts,
                path_vs_fpm: p.path_at(t),
                thr_now: thr,
                dt: tick,
            });
            prev_kts = kts;
            next += tick;
            run.kts.push(kts);
            run.vs.push(p.vs_fpm());
            run.thr.push(thr);
        }
        let h_ref = 2_000.0 + p.path_vs_fpm / 60.0 * (t - p.path_from_s).max(0.0);
        p.step(thr, h_ref, p.path_at(t), t, dt);
        t += dt;
    }
    run
}

/// A phugoid swing holds its energy: climbing at 600 fpm while losing the
/// speed that pays for it is no reason to move the lever. The shared
/// governor read the same swing as an error coming and moved it.
#[test]
fn a_phugoid_swing_does_not_move_the_lever() {
    let (kts, vs) = (65.0, 600.0);
    let accel_kts_s = -(vs / 60.0) * G_FPS2 / (kts * FPS_PER_KT) / FPS_PER_KT;
    assert!(energy_rate_fpm(kts, accel_kts_s, vs).abs() < 1.0);
    let mut law = EnergyThrottle::default();
    let mut thr = 0.5;
    for _ in 0..30 {
        thr = law.step(In { ias_kts: kts, accel_kts_s, vs_fpm: vs, target_kts: kts, path_vs_fpm: 0.0, thr_now: thr, dt: 0.33 });
    }
    assert!((thr - 0.5).abs() < 1e-9, "lever {thr}");
    let old = crate::tools::speedhold::throttle_adjust(0.0, accel_kts_s);
    assert!(old > 0.01, "the shared governor pumps it: {old}");
}

/// The setup's slow start (2026-10-11, 21 kt at release, 50 here) in ALT
/// hold: the speed comes back without overshooting by more than a few knots,
/// the height is held without a swing, and the lever moves like a hand.
#[test]
fn a_slow_start_in_alt_hold_recovers_without_a_swing() {
    let r = fly(Plant::new(50.0, 0.0), 65.0, 90.0);
    let late = |v: &[f64]| v[v.len() * 2 / 3..].to_vec();
    let max_kts = r.kts.iter().cloned().fold(f64::MIN, f64::max);
    assert!(max_kts < 69.0, "overshoot to {max_kts:.1}");
    assert!(late(&r.kts).iter().all(|k| (k - 65.0).abs() < 2.5), "settles: {:?}", late(&r.kts).last());
    assert!(late(&r.vs).iter().all(|v| v.abs() < 250.0), "no swing late");
    let max_rate = r.thr.windows(2).map(|w| (w[1] - w[0]).abs() / 0.33).fold(0.0, f64::max);
    // Slow by 15 kt the hand moves at the slow side's rate, never faster.
    assert!(max_rate <= SLEW_SLOW_PER_S + 1e-9, "the hand's rate: {max_rate}");
}

/// The glideslope captured from level at 10 s: the power comes off as the
/// descent begins, the speed holds and the lever settles.
#[test]
fn on_the_glideslope_the_speed_holds() {
    let r = fly(Plant { path_from_s: 10.0, ..Plant::new(65.0, -350.0) }, 65.0, 90.0);
    let peak = r.kts.iter().cloned().fold(f64::MIN, f64::max);
    assert!(peak < 68.0, "the capture's surge {peak:.1}");
    let late = &r.kts[r.kts.len() / 2..];
    assert!(late.iter().all(|k| (k - 65.0).abs() < 2.0), "speed {:?}", late.last());
    let thr_late = &r.thr[r.thr.len() / 2..];
    let spread = thr_late.iter().cloned().fold(f64::MIN, f64::max) - thr_late.iter().cloned().fold(f64::MAX, f64::min);
    assert!(spread < 0.05, "lever wandered {spread:.3}");
}

/// Full flap on the glideslope (the drag up a third, all at once): the
/// G1000 172 bled from 55 to 45 kt into the stall on a 20 s energy close
/// (LEMG 13, 1791757554). The slow side closes in 6 s with a faster hand:
/// the speed never falls more than 6 kt under the target.
#[test]
fn full_flap_on_the_glideslope_never_bleeds_to_the_stall() {
    let r = fly(Plant { path_from_s: 0.0, flap_drag: 1.35, flap_at_s: 20.0, ..Plant::new(65.0, -350.0) }, 65.0, 90.0);
    let min = r.kts.iter().cloned().fold(f64::MAX, f64::min);
    assert!(min > 59.0, "bled to {min:.1}");
    let late = &r.kts[r.kts.len() * 2 / 3..];
    assert!(late.iter().all(|k| (k - 65.0).abs() < 3.0), "settles: {:?}", late.last());
}
