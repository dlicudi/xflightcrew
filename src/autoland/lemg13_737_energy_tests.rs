//! The LEMG 13 737 final of 2026-10-05, 13:13 to 13:16, replayed through the
//! energy plan (`approach_energy`), from its own record
//! (`bench/approach/lemg13_737_2026-10-05.jsonl`).
//!
//! Three steps, each pinned:
//! 1. The plan's deceleration is the airframe's: the flight's own idle
//!    windows, made level, are at or above the profile's figures.
//! 2. The model is conservative against the flight: integrated at idle
//!    along the RECORDED vertical speed and flaps from the join (13:14:21,
//!    207.7 kt) it never predicts the aircraft slower than it flew, and
//!    stays within 15 kt over the two minutes to 13:16:21 (the real 737 slowed
//!    more: its gear was down from 5.9 nm with flaps 5, and flaps 15 and 25
//!    showed 1.67 and 1.42 kt/s against the declared 1.2 and 1.3).
//! 3. Flown closed-loop from the inbound at 13:13:00 (14.5 nm, 3,480 ft,
//!    177 kt, flaps 5, level) through the 737's VS plant (two 1.5 s lags
//!    after 2 s of dead time, the profile's `arrest_lead` fit) to the
//!    1,000 ft gate: the old schedule arrives fast and unconfigured (as the
//!    flight did, going around high and fast at 3.8 nm); the plan arrives
//!    configured, on the path and on speed (`stable_approach::breaches`
//!    empty).

use super::*;
use crate::approach_energy::{self, EnergyModel, G_KTS_PER_S};

const SRC: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/bench/approach/lemg13_737_2026-10-05.jsonl"
));
/// LEMG 13's aim elevation (`threshold_elev` aim_ft, 13:02:39).
const AIM_ELEV_FT: f64 = 48.0;
/// The surface wind on the final (METAR 130/08): a headwind on 13.
const RECORDED_HEADWIND: f64 = 8.0;

#[derive(Debug, Clone)]
struct Tick {
    t: f64,
    ias: f64,
    alt: f64,
    dist: f64,
    vvi: f64,
    thr: Option<f64>,
    flap: f64,
}

fn ticks() -> Vec<Tick> {
    SRC.lines()
        .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
        .filter(|r| r.get("gp").is_some() && r.get("ias").is_some() && r["ph"] != "pilot_app")
        .map(|r| Tick {
            t: r["t"].as_f64().unwrap(),
            ias: r["ias"].as_f64().unwrap(),
            alt: r["alt"].as_f64().unwrap(),
            dist: r["dist"].as_f64().unwrap(),
            vvi: r["vvi"].as_f64().unwrap(),
            thr: r["thr"].as_f64(),
            flap: r["flap"].as_f64().unwrap_or(0.0),
        })
        .collect()
}

fn b738() -> crate::aircraft::AircraftProfile {
    crate::aircraft::load("boeing_737_800").expect("737 profile")
}

fn model() -> EnergyModel {
    EnergyModel::from_profile(&b738(), 3.0).expect("declared")
}

fn tas(ias: f64, alt: f64) -> f64 {
    ias * (1.0 + 0.02 * alt / 1000.0)
}

/// The deceleration a tick pair shows, made level (kt/s, slowing positive).
fn level_decel(a: &Tick, b: &Tick) -> f64 {
    let dt = b.t - a.t;
    let dv = (b.ias - a.ias) / dt;
    let vs = (a.vvi + b.vvi) / 2.0;
    let sin_g = (vs / 60.0) / (tas((a.ias + b.ias) / 2.0, a.alt) * 1.687_81);
    -dv - G_KTS_PER_S * sin_g
}

#[test]
fn the_plans_deceleration_is_the_flights_own() {
    let t = ticks();
    let m = model();
    // Every idle window of 10 s or more at one flap setting, made level.
    let mut by_flap: Vec<(f64, f64)> = Vec::new();
    let mut i = 0;
    while i < t.len() {
        let mut j = i;
        while j + 1 < t.len()
            && t[j + 1].thr.is_some_and(|x| x <= 0.12)
            && t[i].thr.is_some_and(|x| x <= 0.12)
            && (t[j + 1].flap - t[i].flap).abs() < 0.01
            && t[j + 1].t - t[j].t < 2.5
        {
            j += 1;
        }
        if j > i && t[j].t - t[i].t >= 10.0 {
            by_flap.push((t[i].flap, level_decel(&t[i], &t[j])));
        }
        i = j + 1;
    }
    assert!(by_flap.len() >= 5, "{by_flap:?}");
    for (flap, a) in &by_flap {
        // Conservative: never more than the airframe showed, less a margin
        // for the window's own noise.
        assert!(
            m.level_decel(*flap) <= a + 0.35,
            "flap {flap}: declared {} vs flown {a:.2}",
            m.level_decel(*flap)
        );
    }
}

#[test]
fn the_model_is_conservative_against_the_recorded_final() {
    let t = ticks();
    let m = model();
    // 13:14:21 to 13:16:21: the path joined at 207.7 kt, idle throughout.
    let i0 = t.iter().position(|k| k.t >= 1_791_198_861.0).unwrap();
    let i1 = t.iter().position(|k| k.t >= 1_791_198_981.0).unwrap();
    let mut v = t[i0].ias;
    for w in t[i0..=i1].windows(2) {
        let (a, b) = (&w[0], &w[1]);
        let dt = b.t - a.t;
        let sin_g = (a.vvi / 60.0) / (tas(v, a.alt) * 1.687_81);
        let lever_up = a.thr.is_some_and(|x| x > 0.15);
        let accel = if lever_up {
            0.0
        } else {
            -m.level_decel(a.flap)
        } - G_KTS_PER_S * sin_g;
        v += accel * dt;
    }
    let recorded = t[i1].ias;
    assert!(
        v >= recorded - 2.0 && v - recorded <= 15.0,
        "predicted {v:.1} against the recorded {recorded:.1}"
    );
}

/// The 737's VS mode: two 1.5 s lags after 2 s of dead time.
struct VsPlant {
    queue: std::collections::VecDeque<f64>,
    x1: f64,
    x2: f64,
}

impl VsPlant {
    fn new(vs: f64, dt: f64) -> Self {
        Self {
            queue: std::iter::repeat_n(vs, (2.0 / dt) as usize).collect(),
            x1: vs,
            x2: vs,
        }
    }
    fn step(&mut self, cmd: f64, dt: f64) -> f64 {
        self.queue.push_back(cmd);
        let u = self.queue.pop_front().unwrap();
        self.x1 += (u - self.x1) * dt / 1.5;
        self.x2 += (self.x1 - self.x2) * dt / 1.5;
        self.x2
    }
}

#[derive(Debug)]
struct AtGate {
    ias: f64,
    flap: f64,
    gear: bool,
    above_path_ft: f64,
    vvi: f64,
    landing_flap_s: f64,
}

/// Fly the final from 13:13:00 to the gate, the old schedule (`plan`
/// false) or the energy plan, the aircraft slowing `drag` times what the
/// plan believes (1.0: as declared; the flight showed more).
fn fly(plan: bool, drag: f64) -> AtGate {
    let p = b738();
    let m = model();
    let limits: Vec<(f64, Option<f64>)> = p
        .flaps
        .as_ref()
        .unwrap()
        .detents
        .iter()
        .map(|d| (d.ratio, d.vfe_kts))
        .collect();
    let floor = |flap: f64| Some((1.2 * (139.0 + (114.0 - 139.0) * flap)).round());
    let approach_ias = 145.0;
    let vapp = approach_target_kts(approach_ias, 1.0, &limits, floor(1.0));
    let t0 = ticks()
        .into_iter()
        .find(|k| k.t >= 1_791_198_780.0)
        .unwrap();
    assert!(
        (t0.dist - 14.5).abs() < 0.2 && (t0.ias - 177.0).abs() < 2.0,
        "{t0:?}"
    );
    let gpft = GLIDEPATH_FT_PER_NM;
    let path_alt = |d: f64| AIM_ELEV_FT + (d + TOUCHDOWN_AIM_NM) * gpft;
    // Where the 3,480 ft platform meets the path: the pattern's end.
    let end_nm = (t0.alt - AIM_ELEV_FT) / gpft - TOUCHDOWN_AIM_NM;
    let gate_nm = 1000.0 / gpft - TOUCHDOWN_AIM_NM;
    let dt = 0.25;
    let (mut d, mut alt, mut v, mut flap) = (t0.dist, t0.alt, t0.ias, t0.flap);
    let mut gear = false;
    let mut vs = 0.0;
    let mut plant = VsPlant::new(0.0, dt);
    let mut cap: Option<f64> = None;
    let mut joined = false;
    let mut last_flap_cmd = Some(flap);
    let mut landing_at: Option<f64> = None;
    let mut time = 0.0;
    while alt - AIM_ELEV_FT > 1000.0 {
        assert!(
            time < 900.0,
            "never reached the gate: {d:.2} nm {alt:.0} ft"
        );
        let gs = tas(v, alt) - RECORDED_HEADWIND;
        let err = alt - path_alt(d);
        joined = joined || err >= -10.0;
        // The vertical: the platform until the path comes down to it, then
        // the path law (feed-forward plus the error, its own cap).
        let nominal = -(gs / 60.0) * gpft;
        let cmd_vs = if joined {
            (nominal - err * glidepath_gain_fpm_per_ft(gs))
                .clamp(path_descent_cap_fpm(nominal), 0.0)
        } else {
            0.0
        };
        // The speed: the inbound leg's and the final's targets.
        let mut target = if joined {
            scheduled_ias(d, approach_ias)
        } else if plan {
            scheduled_ias(d, approach_ias).min(pattern_ias(approach_ias, &limits))
        } else {
            scheduled_ias(d, approach_ias)
        };
        let mut plan_flap = None;
        if plan {
            let pattern = (!joined).then_some(((d - end_nm).max(0.0), end_nm));
            let (s, path) = energy_geometry(pattern, d, err, gpft, gate_nm);
            let a = approach_energy::ask(&m, vapp, s, path, gs);
            let kts = target.min(a.kts).max(approach_ias);
            let kts = cap.map_or(kts, |c| kts.min(c));
            cap = Some(kts);
            target = kts;
            plan_flap = a.flap;
        }
        // The configuration.
        let sched = approach_leg_flap_schedule(d, err, false);
        let sched = plan_flap.map_or(sched, |f| plan_flap_wanted(f, err).max(sched));
        let wanted = approach_flap_wanted(sched, last_flap_cmd);
        let cmd = flap_held_within_limits(wanted, v, &limits, last_flap_cmd);
        last_flap_cmd = Some(cmd);
        if cmd >= LANDING_FLAP_RATIO && landing_at.is_none() {
            landing_at = Some(time);
        }
        flap += (cmd - flap).clamp(-0.05 * dt, 0.05 * dt);
        if !gear && (d < 6.0 || plan_flap.is_some_and(|f| f >= 0.5)) && v <= 235.0 {
            gear = true;
        }
        let target = approach_target_kts(target, flap, &limits, floor(flap));
        // The aircraft.
        vs = plant.step(cmd_vs, dt);
        let sin_g = (vs / 60.0) / (tas(v, alt) * 1.687_81);
        let idle = -m.level_decel(flap) * drag - G_KTS_PER_S * sin_g;
        let dv = if v > target {
            idle
        } else {
            ((target - v) / 3.0).min(1.0).max(idle)
        };
        v += dv * dt;
        alt += vs / 60.0 * dt;
        d -= gs / 3600.0 * dt;
        time += dt;
    }
    AtGate {
        ias: v,
        flap,
        gear,
        above_path_ft: alt - path_alt(d),
        vvi: vs,
        landing_flap_s: landing_at.map_or(0.0, |t| time - t),
    }
}

fn breaches(g: &AtGate) -> Vec<crate::stable_approach::Breach> {
    let sample = crate::stable_approach::Sample {
        height_aal_ft: 1000.0,
        dist_to_aim_nm: 1000.0 / GLIDEPATH_FT_PER_NM,
        ias_kts: g.ias,
        gs_kts: tas(g.ias, 1000.0) - RECORDED_HEADWIND,
        target_kts: 145.0,
        vvi_fpm: Some(g.vvi),
        above_path_ft: g.above_path_ft,
        path_ft_per_nm: GLIDEPATH_FT_PER_NM,
        gear_down: g.gear,
        landing_flap: g.flap >= LANDING_FLAP_RATIO,
    };
    crate::stable_approach::breaches(crate::stable_approach::GateKind::Instrument, &sample)
}

#[test]
fn the_lemg_737_final_is_configured_and_on_speed_at_the_gate_on_the_plan() {
    let old = fly(false, 1.0);
    let new = fly(true, 1.0);
    // The schedule: fast and short of the landing flap at the gate, the
    // flight's own story (177 kt at 3.9 nm with flaps 25, around).
    let b = breaches(&old);
    assert!(!b.is_empty(), "the old schedule made the gate? {old:?}");
    assert!(old.ias > 160.0, "{old:?}");
    // The plan: configured, on the path, on speed.
    let b = breaches(&new);
    assert!(b.is_empty(), "{b:?} {new:?}");
    assert!((140.0..=152.0).contains(&new.ias), "{new:?}");
    assert!(new.flap >= LANDING_FLAP_RATIO && new.gear, "{new:?}");
    assert!(new.above_path_ft.abs() < 60.0, "{new:?}");
    // The landing flap out early enough for its transient (12 s of a
    // stalled V/S on 2026-10-05) and the speed's settle.
    assert!(
        new.landing_flap_s >= approach_energy::GATE_MARGIN_S,
        "{new:?}"
    );
    // An aircraft with more drag than declared (the flight's own windows)
    // or a little less still makes the gate on the plan; the schedule
    // fails either way.
    for drag in [0.9, 1.3] {
        let g = fly(true, drag);
        assert!(
            breaches(&g).is_empty(),
            "drag x{drag}: {g:?} {:?}",
            breaches(&g)
        );
        assert!(!breaches(&fly(false, drag)).is_empty(), "drag x{drag}");
    }
}

/// The plan's geometry: a pattern is level to its end and the path from
/// there; a straight-in below the path is level until the path comes down
/// to it; on or above it, all path.
#[test]
fn the_route_to_the_gate_is_level_then_path() {
    let gate = 1000.0 / GLIDEPATH_FT_PER_NM - TOUCHDOWN_AIM_NM;
    // The 13:13 inbound: 3.9 nm to the pattern's end at 10.6.
    let (s, path) = energy_geometry(Some((3.9, 10.6)), 14.5, -1200.0, GLIDEPATH_FT_PER_NM, gate);
    assert!((path - (10.6 - gate)).abs() < 1e-9 && (s - (3.9 + path)).abs() < 1e-9);
    // 318 ft below the path at 12 nm: it comes down to us in a mile.
    let (s, path) = energy_geometry(None, 12.0, -318.0, GLIDEPATH_FT_PER_NM, gate);
    assert!(
        (s - (12.0 - gate)).abs() < 1e-9 && (path - (11.0 - gate)).abs() < 1e-6,
        "{s} {path}"
    );
    let (s, path) = energy_geometry(None, 8.0, 40.0, GLIDEPATH_FT_PER_NM, gate);
    assert_eq!(s, path);
    // The landing flap never goes out level under the path.
    assert_eq!(plan_flap_wanted(1.0, -300.0), 0.5);
    assert_eq!(plan_flap_wanted(1.0, -20.0), 1.0);
    assert_eq!(plan_flap_wanted(0.625, -300.0), 0.625);
}

/// 13:14:21 to 13:15:21 on 2026-10-05: on the centreline at 10.7 to 7.1 nm,
/// on or above the path, phase Intercept (steering for the FAP). The
/// reposition's soft floor keyed on radio height over the hills cut the
/// path's command to -210 fpm at 1,695 ft radio; established on the final
/// path the final's own floor holds, and the command is the path's.
#[test]
fn an_established_final_path_beyond_the_fap_descends_on_the_path() {
    use ApproachPhase::*;
    assert!(established_on_final_path(
        Intercept, None, 9.0, 0.02, 0.1, 148.0
    ));
    assert!(
        !established_on_final_path(Intercept, None, 4.0, 0.02, 0.1, 148.0),
        "inside the FAP: the final's"
    );
    assert!(
        !established_on_final_path(Intercept, None, 9.0, 0.6, 0.1, 148.0),
        "off the line"
    );
    assert!(
        !established_on_final_path(Intercept, None, 9.0, 0.02, 2.0, 148.0),
        "crossing it"
    );
    assert!(
        !established_on_final_path(Intercept, None, 9.0, 0.02, 0.1, -400.0),
        "below the path"
    );
    assert!(!established_on_final_path(
        Intercept,
        Some("overflew_the_runway"),
        9.0,
        0.02,
        0.1,
        0.0
    ));
    assert!(!established_on_final_path(Final, None, 9.0, 0.02, 0.1, 0.0));
    // The recorded tick (13:14:48): 148 ft high at 9.0 nm, 1,695 ft radio,
    // 212 kt. The fade asked -210; the path asks its own sink and more.
    let gs = 220.0;
    let nominal = -(gs / 60.0) * GLIDEPATH_FT_PER_NM;
    let fade = -((1695.0 - REPOSITION_MIN_AGL_FT) * 2.0).clamp(0.0, -path_descent_cap_fpm(nominal));
    let path =
        (nominal - 148.0 * glidepath_gain_fpm_per_ft(gs)).clamp(path_descent_cap_fpm(nominal), 0.0);
    assert!(
        fade > -250.0 && path < -1100.0,
        "fade {fade:.0} path {path:.0}"
    );
}
