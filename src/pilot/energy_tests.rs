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
/// What a unit of lever buys this plant at 65 kt, by the book's rule
/// (twice the 172's 730 fpm best-rate climb: 1,460 against the plant's
/// 1,230), the engine's 0.5 s.
const PLANT_TECS: Tecs = Tecs { fpm_per_unit: 1_460.0, spool_s: 0.5 };

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
            thr = law
                .step(
                    In {
                        ias_kts: kts,
                        accel_kts_s: (kts - prev_kts) / tick,
                        vs_fpm: p.vs_fpm(),
                        target_kts,
                        floor_kts: None,
                        path_vs_fpm: p.path_at(t),
                        thr_now: thr,
                        dt: tick,
                        flare_to_wheels_s: None,
                    },
                    PLANT_TECS,
                )
                .lever;
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
        thr = law
            .step(
                In { ias_kts: kts, accel_kts_s, vs_fpm: vs, target_kts: kts, floor_kts: None, path_vs_fpm: 0.0, thr_now: thr, dt: 0.33, flare_to_wheels_s: None },
                PLANT_TECS,
            )
            .lever;
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
    assert!(max_rate <= SLEW_PROTECT_PER_S + 1e-9, "the hand's rate: {max_rate}");
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

/// The 737 on a 3 degree path as sortie 1791979030-full_auto flew it
/// (EGKK 26L, 2026-10-14): the lever's energy the book's (twice the
/// 3,000 fpm climb: 5.9 kt/s a unit at 190 kt) through the jet's spool, and
/// the idle deceleration that leaves the slow-down recorded, 0.5 kt/s on
/// the path at 0.08 of lever over idle with flaps 10 and 15 and the gear:
/// 2.0 kt/s level (the profile's lower quartile is 1.2). Flaps 40 under
/// its placard less the margin, scaled alike (the profile's 2.2 to 3.5).
/// The law's own 0.35 s tick. Returns the speed and the lever.
fn fly_737_final(from_kts: f64, target_kts: f64, thr0: f64, secs: f64) -> Run {
    const S1: f64 = 5.94;
    let tecs = Tecs::from_book(Some(3_000.0), EngineClass::Jet);
    let gp = 3f64.to_radians();
    let (dt, tick) = (0.05, 0.35);
    let mut law = EnergyThrottle::default();
    let (mut v, mut thrust, mut thr, mut t, mut next, mut prev) = (from_kts, thr0, thr0, 0.0, 0.0, from_kts);
    let mut run = Run { kts: vec![], vs: vec![], thr: vec![] };
    while t < secs {
        let path_vs = -v * FPS_PER_KT * 60.0 * gp.sin();
        if t >= next {
            thr = law
                .step(
                    In {
                        ias_kts: v,
                        accel_kts_s: (v - prev) / tick,
                        vs_fpm: path_vs,
                        target_kts,
                        floor_kts: None,
                        path_vs_fpm: path_vs,
                        thr_now: thr,
                        dt: tick,
                        flare_to_wheels_s: None,
                    },
                    tecs,
                )
                .lever;
            prev = v;
            next += tick;
            run.kts.push(v);
            run.thr.push(thr);
        }
        let idle = if v <= 157.0 { 2.2 } else { 2.0 };
        thrust += (thr - thrust) * dt / tecs.spool_s;
        v += (-idle + S1 * thrust + 32.174 / FPS_PER_KT * gp.sin()) * dt;
        t += dt;
    }
    run
}

/// EGKK 26L, 2026-10-14 (sortie 1791979030-full_auto): the 737 took the
/// final at 210 kt against 145, and the speed term's bound held a 0.5 kt/s
/// slow-down on 0.12 to 0.29 of lever for 55 s, 38 kt fast at the gate.
/// Fast by more than the band closes, the lever is at idle within its
/// hand's walk and stays there; the trim kept, the speed then arrives
/// without falling out of the stable band under the target.
#[test]
fn a_737_far_fast_on_the_path_flies_it_at_idle() {
    let r = fly_737_final(200.0, 145.0, 0.25, 360.0);
    let tick = 0.35;
    let mut idle_after = None;
    for (i, (k, l)) in r.kts.iter().zip(&r.thr).enumerate() {
        let at = i as f64 * tick;
        if *k > 145.0 + 15.0 && at > 0.25 / SLEW_PER_S + 1.0 {
            assert!(*l < 0.05, "{k:.0} kt at {at:.0} s: lever {l:.2}, not idle");
            idle_after.get_or_insert(at);
        }
    }
    assert!(idle_after.is_some(), "never far fast");
    let min = r.kts.iter().cloned().fold(f64::MAX, f64::min);
    assert!(min > 145.0 - 7.0, "bled to {min:.1}");
    let late = &r.kts[r.kts.len() * 5 / 6..];
    assert!(late.iter().all(|k| (k - 145.0).abs() < 4.0), "settles: {:?}", late.last());
}
