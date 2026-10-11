use super::*;
use crate::aircraft::{FlareElevatorProfile, FlareModelProfile, FlarePowerProfile};
use std::collections::VecDeque;

/// The six arrests autoland flew on the Zibo at LEMG 30 on 2026-10-07 (the
/// final from 400 ft, the arrest to the wheels, the rollout's first tick
/// and the flight model's touchdown): sorties 1791363565 (450 fpm, the
/// fixed 145 kt), 1791371247 (203), the "land here" at 1791383430 (209),
/// sortie 1791385153 (259), the reposition at 1791388663 (326) and sortie
/// 1791392327 (475).
const BENCH: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/bench/landings/zibo_lemg30_flares.jsonl"
));

/// The loop's tick through these arrests, seconds.
const TICK_S: f64 = 0.2;
/// The radio altitude on the mains: the ticks on the wheels read 0.4 to
/// 1.3 ft.
const WHEELS_FT: f64 = 1.0;

#[derive(Debug, Clone, Copy)]
struct Tick {
    t: f64,
    radio: f64,
    vvi: f64,
    trend: f64,
    ias: f64,
    tgt_ias: f64,
    elev: Option<f64>,
    lever: f64,
    cmd: f64,
    flare: bool,
    arresting: bool,
    ground: bool,
    dist: f64,
}

#[derive(Clone)]
struct Flare {
    id: String,
    ticks: Vec<Tick>,
    rough: bool,
    touchdown_fpm: f64,
    /// The rollout's first tick: metres past the threshold.
    rollout_m: f64,
}

fn flares() -> Vec<Flare> {
    let mut out: Vec<Flare> = Vec::new();
    for line in BENCH.lines() {
        let v: serde_json::Value = serde_json::from_str(line).expect("a bench line");
        let id = v["flare"].as_str().expect("an id").to_string();
        if out.last().is_none_or(|f| f.id != id) {
            out.push(Flare {
                id: id.clone(),
                ticks: Vec::new(),
                rough: false,
                touchdown_fpm: f64::NAN,
                rollout_m: f64::NAN,
            });
        }
        let f = out.last_mut().expect("pushed");
        let n = |k: &str| v[k].as_f64().unwrap_or(f64::NAN);
        match v["kind"].as_str() {
            Some("tick") => f.ticks.push(Tick {
                t: n("t"),
                radio: n("radio"),
                vvi: n("vvi"),
                trend: n("vvi_trend"),
                ias: n("ias"),
                tgt_ias: n("tgt_ias"),
                elev: v["flare_elev"].as_f64(),
                lever: n("lever"),
                cmd: n("cmd_vs"),
                flare: v["phase"] == "Flare",
                arresting: v["arresting"] == true,
                ground: v["ground"] == true,
                dist: n("dist"),
            }),
            Some("arrest_hands") if v["taken"] == true => f.rough = v["rough"] == true,
            Some("touchdown") => f.touchdown_fpm = n("sink"),
            Some("rollout") => {
                let last = f.ticks.last().map_or(f64::NAN, |k| k.dist);
                f.rollout_m = last * M_PER_NM;
            }
            _ => {}
        }
    }
    assert_eq!(out.len(), 6);
    out
}

impl Flare {
    /// The arrest: the servos out and our column written, to the first tick
    /// on the wheels.
    fn arrest(&self) -> Vec<Tick> {
        let mut a = Vec::new();
        for k in self
            .ticks
            .iter()
            .filter(|k| k.arresting && k.elev.is_some())
        {
            a.push(*k);
            if k.ground || k.radio < 1.5 {
                break;
            }
        }
        a
    }
    /// The groundspeed through the arrest, kt (the threshold distance's
    /// travel, which is unsigned either side of the threshold).
    fn gs_kts(&self) -> f64 {
        let a = self.arrest();
        let nm: f64 = a.windows(2).map(|w| (w[1].dist - w[0].dist).abs()).sum();
        nm / (a[a.len() - 1].t - a[0].t) * 3600.0
    }
    /// The gate's distance short of the threshold, metres.
    fn gate_short_m(&self) -> f64 {
        self.arrest()[0].dist * M_PER_NM
    }
}

/// The Zibo's elevator with the servos out, fitted to every arrest tick of
/// the six (the recorded column and lever written, the recorded airspeed):
/// a standing sink per unit of column through two lags after a dead time,
/// the lever pitching the nose as `LEVER_ELEV` of column does (its thrust
/// line under the wing: a retard is a push), a sink that grows below
/// `GROUND_FT` (the tail's ground effect) and with every knot lost.
const LAG1_S: f64 = 0.416;
const LAG2_S: f64 = 6.458;
const GROUND_SINK: f64 = 650.4;
const GROUND_FT: f64 = 19.03;
const FPM_PER_KT: f64 = 48.87;
const LEVER_ELEV: f64 = 0.2046;

/// One plant: the column's standing sink (fpm per unit) and its dead time.
#[derive(Debug, Clone, Copy)]
struct Fit {
    k: f64,
    dead_s: f64,
}

/// The six together (17 fpm rms over every tick, a trimmed sink each), then
/// each arrest's own gain and dead time on the shared lags (6.5 to 22 fpm
/// rms). The calm three ask 6,833 to 9,767 fpm and 1.28 to 1.59 s; the
/// rough 209 and 326 ask 4,620 and 3,870 (their gusts are in the fit), the
/// rough 475 a gain at the search's bound. The still-air family is the
/// joint fit, the calm three, and the corners past them (6,000 at 1.75 s,
/// 10,000 at 1.25 s); the rough fits fly only with their own air.
const JOINT: Fit = Fit {
    k: 7918.0,
    dead_s: 1.44,
};
const PLANTS: [Fit; 6] = [
    JOINT,
    Fit {
        k: 9116.0,
        dead_s: 1.28,
    },
    Fit {
        k: 6833.0,
        dead_s: 1.59,
    },
    Fit {
        k: 6000.0,
        dead_s: 1.75,
    },
    Fit {
        k: 9767.0,
        dead_s: 1.55,
    },
    Fit {
        k: 10000.0,
        dead_s: 1.25,
    },
];
/// Each arrest's own fit, in the bench's order, and the sink its trim held
/// with the column at neutral: (plant, trimmed fpm) for its own replay.
const OWN: [(Fit, f64); 6] = [
    (
        Fit {
            k: 9116.0,
            dead_s: 1.28,
        },
        -668.0,
    ),
    (
        Fit {
            k: 6833.0,
            dead_s: 1.59,
        },
        -801.0,
    ),
    (
        Fit {
            k: 4620.0,
            dead_s: 1.37,
        },
        -442.0,
    ),
    (
        Fit {
            k: 9767.0,
            dead_s: 1.55,
        },
        -1128.0,
    ),
    (
        Fit {
            k: 3870.0,
            dead_s: 1.77,
        },
        -465.0,
    ),
    (JOINT, -1272.0),
];
/// The joint fit's trimmed sink per arrest, for any plant from that gate.
const JOINT_TRIM: [f64; 6] = [-580.0, -980.0, -1165.0, -855.0, -1126.0, -1272.0];

/// The trimmed sink for a plant at a gate: the column the autopilot held
/// there (the joint fit's) is the gate's, whatever the plant.
fn trim_for(f: &Flare, gi: usize, fit: Fit) -> f64 {
    let v0 = f.arrest()[0].vvi;
    let ap_column = (v0 - JOINT_TRIM[gi]) / JOINT.k;
    v0 - fit.k * ap_column
}

/// The airspeed, kt/s: the lever as it was 1.2 s before (the spool), the
/// column, the sink (least squares over the six arrests, 0.21 kt/s rms).
const SPOOL_S: f64 = 1.2;
fn speed_model(lever_then: f64, elev: f64, vvi: f64) -> f64 {
    -1.146 + 2.580 * lever_then + 0.194 * elev - 0.449 * vvi / 1000.0
}

struct Plant {
    fit: Fit,
    trimmed: f64,
    lever0: f64,
    x1: f64,
    x2: f64,
    t: f64,
    /// (time written, column with the lever's pitch) and (time, lever).
    written: VecDeque<(f64, f64)>,
    levers: VecDeque<(f64, f64)>,
    elev: f64,
    ias0: f64,
    ias: f64,
    vvi: f64,
    air_fpm: f64,
    air_kts_s: f64,
}

impl Plant {
    /// At the gate, steady: the autopilot's column is what holds the sink.
    fn at(g: Tick, fit: Fit, trimmed: f64) -> Self {
        let x = g.vvi - trimmed;
        let mut written = VecDeque::new();
        written.push_back((f64::NEG_INFINITY, x / fit.k));
        let mut levers = VecDeque::new();
        levers.push_back((f64::NEG_INFINITY, g.lever));
        Plant {
            fit,
            trimmed,
            lever0: g.lever,
            x1: x,
            x2: x,
            t: 0.0,
            written,
            levers,
            elev: 0.0,
            ias0: g.ias,
            ias: g.ias,
            vvi: g.vvi,
            air_fpm: 0.0,
            air_kts_s: 0.0,
        }
    }
    fn write(&mut self, elev: f64, lever: f64) {
        self.elev = elev;
        self.written
            .push_back((self.t, elev + LEVER_ELEV * (lever - self.lever0)));
        self.levers.push_back((self.t, lever));
    }
    fn at_time(q: &mut VecDeque<(f64, f64)>, t: f64) -> f64 {
        while q.len() > 1 && q[1].0 <= t + 1e-9 {
            q.pop_front();
        }
        q[0].1
    }
    /// `ias` Some: the recorded airspeed (a fit); None: the speed model.
    fn step(&mut self, h: f64, ias: Option<f64>, dt: f64) {
        let e = Self::at_time(&mut self.written, self.t - self.fit.dead_s);
        let ground = GROUND_SINK * (1.0 - h.max(0.0) / GROUND_FT).max(0.0);
        self.x1 += (self.fit.k * e - ground - self.x1) * dt / LAG1_S;
        self.x2 += (self.x1 - self.x2) * dt / LAG2_S;
        self.ias = match ias {
            Some(v) => v,
            None => {
                let then = Self::at_time(&mut self.levers, self.t - SPOOL_S);
                self.ias + (speed_model(then, self.elev, self.vvi) + self.air_kts_s) * dt
            }
        };
        self.vvi = self.trimmed + self.x2 + FPM_PER_KT * (self.ias - self.ias0) + self.air_fpm;
        self.t += dt;
    }
}

/// What the plant does not explain along a recording: the recorded column,
/// lever and airspeed flown open loop, the sink (fpm, at each tick) and the
/// speed change (kt/s, over the interval after it) left over.
fn air_of(a: &[Tick], fit: Fit, trimmed: f64) -> (Vec<f64>, Vec<f64>, f64) {
    let mut p = Plant::at(a[0], fit, trimmed);
    let (mut air, mut speed, mut sq) = (vec![0.0], Vec::new(), 0.0);
    for w in a.windows(2) {
        let (t0, t1) = (w[0].t - a[0].t, w[1].t - a[0].t);
        p.write(w[0].elev.unwrap_or(0.0), w[0].lever);
        let (mut acc, mut n) = (0.0, 0.0);
        while p.t + 1e-9 < t1 {
            let then = Plant::at_time(&mut p.levers.clone(), p.t - SPOOL_S);
            acc += speed_model(then, p.elev, w[0].vvi);
            n += 1.0;
            let f = (p.t - t0) / (t1 - t0);
            p.step(w[0].radio, Some(w[0].ias + (w[1].ias - w[0].ias) * f), 0.01);
        }
        speed.push((w[1].ias - w[0].ias) / (t1 - t0) - acc / f64::max(n, 1.0));
        air.push(w[1].vvi - p.vvi);
        sq += (w[1].vvi - p.vvi).powi(2);
    }
    let rms = (sq / (a.len() - 1) as f64).sqrt();
    (air, speed, rms)
}

/// Rough air on a recording's tick times: a vertical gust (fpm), an
/// Ornstein-Uhlenbeck sink of `rms` with a 1.5 s time constant, seeded.
fn gusts(a: &[Tick], seed: u64, rms: f64) -> (Vec<f64>, Vec<f64>, f64) {
    let mut rng = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
    let mut gauss = || {
        let mut u = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            ((rng >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        };
        let (x, y) = (u(), u());
        (-2.0 * x.ln()).sqrt() * (std::f64::consts::TAU * y).cos()
    };
    const TC: f64 = 1.5;
    let mut g = 0.0;
    let mut sink = vec![0.0];
    for w in a.windows(2) {
        let dt = w[1].t - w[0].t;
        g += -g / TC * dt + rms * (2.0 * dt / TC).sqrt() * gauss();
        sink.push(g);
    }
    (sink, vec![0.0; a.len()], 0.0)
}

/// The column law flown.
enum Law {
    /// The P+D tracker (no `model:`), with its exponential's time constant.
    Tracker(FlarePitch, Option<f64>, Option<f64>),
    Model(FlareModel),
}

impl Law {
    fn of(p: FlareElevatorProfile) -> Self {
        match FlareModel::from_profile(Some(p)) {
            Some(m) => Law::Model(m),
            None => Law::Tracker(
                FlarePitch::with_gains(FlareGains::from_profile(Some(p))),
                p.tau_s,
                None,
            ),
        }
    }
    #[allow(clippy::too_many_arguments)]
    fn step(
        &mut self,
        h: f64,
        gs: f64,
        v: f64,
        dt: f64,
        flare: bool,
        rough: bool,
        ias: f64,
    ) -> f64 {
        match self {
            Law::Model(m) => {
                m.note_speed(ias, dt);
                m.step(h, gs, v, dt, flare, rough).elev
            }
            Law::Tracker(p, tau, prev) => {
                let trend = prev.map_or(0.0, |p| (v - p) / dt);
                *prev = Some(v);
                let nominal = -(gs / 60.0) * gp_ft_per_nm();
                let (roundout, flare_vs) = arrest_targets_fpm(nominal);
                let target = if flare {
                    flare_target_with(h, flare_vs, nominal, *tau)
                } else {
                    roundout
                };
                p.step(target, v, trend, dt, flare)
            }
        }
    }
}

/// The Zibo's `flare_elevator:` and `flare_power:` as flown on 2026-10-07:
/// the stock 737's tracker settings, no model.
fn as_flown_elevator() -> FlareElevatorProfile {
    FlareElevatorProfile {
        gain: 0.6,
        trim_gain: 1.0,
        max: 0.45,
        max_push: 0.0,
        balloon_ease_per_s: Some(0.15),
        tau_s: Some(5.5),
        model: None,
        attitude: false,
    }
}
fn as_flown_power() -> FlarePowerProfile {
    FlarePowerProfile {
        reduction_per_second: 0.08,
        retard_below_ft: Some(20.0),
        retard_max_per_s: Some(0.04),
        retard_slow_kts: Some(5.0),
        retard_at_s: Some(2.0),
        retard_fast_kts: Some(0.0),
        hold_from_s: None,
        retard_float_per_s: Some(0.08),
    }
}
fn zibo() -> crate::aircraft::AircraftProfile {
    crate::aircraft::load("zibo_738").expect("the Zibo profile loads")
}
fn declared_elevator() -> FlareElevatorProfile {
    zibo().flare_elevator.expect("the Zibo declares its flare")
}
fn declared_power() -> FlarePowerProfile {
    zibo().flare_power.expect("the Zibo declares its retard")
}

#[derive(Debug, Clone, Copy)]
struct Landed {
    sink_fpm: f64,
    /// Metres past the threshold.
    touchdown_m: f64,
    /// Seconds in the flare sinking under `RETARD_FLOAT_FPM`.
    float_s: f64,
    climbed: bool,
    /// The steepest climb in the flare after its first three seconds, fpm.
    climb_fpm: f64,
    max_elev: f64,
    lever_at_wheels: f64,
    ias_at_wheels: f64,
}

/// Fly one recorded gate to the wheels: the loop's tick, the flare from
/// the loop's own gate, the recorded lever in the roundout (with the
/// roundout's float retard), the airframe's retard below the flare gate,
/// the column through the plant; `air` the recording's unexplained sink
/// and speed (else still air).
fn fly(
    f: &Flare,
    fit: Fit,
    trimmed: f64,
    mut law: Law,
    power: Option<FlarePowerProfile>,
    air: Option<&(Vec<f64>, Vec<f64>, f64)>,
) -> Landed {
    let a = f.arrest();
    let gs = f.gs_kts();
    let times: Vec<f64> = a.iter().map(|k| k.t - a[0].t).collect();
    let mut p = Plant::at(a[0], fit, trimmed);
    let mut h = a[0].radio;
    let mut lever = a[0].lever;
    let mut fp = FlarePower::default();
    let mut was_flaring = false;
    let (mut flaring, mut next, mut i) = (false, 0.0, 0usize);
    let (mut prev_v, mut trend) = (a[0].vvi, a[0].trend);
    let mut out = Landed {
        sink_fpm: f64::NAN,
        touchdown_m: f64::NAN,
        float_s: 0.0,
        climbed: false,
        climb_fpm: f64::MIN,
        max_elev: 0.0,
        lever_at_wheels: f64::NAN,
        ias_at_wheels: f64::NAN,
    };
    let rough = f.rough;
    while h > WHEELS_FT && p.t < 30.0 {
        if p.t + 1e-9 >= next {
            let v = p.vvi;
            if p.t > 0.0 {
                trend = (v - prev_v) / TICK_S;
            }
            prev_v = v;
            let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
            flaring = h <= flare_ft || (flaring && h <= band_ft);
            let e = law.step(h, gs, v, TICK_S, flaring, rough, p.ias);
            out.max_elev = out.max_elev.max(e);
            while i + 1 < times.len() && times[i + 1] <= p.t + 1e-9 {
                i += 1;
            }
            let recorded = a[i].lever;
            lever = match power {
                // The lever as it was written, tick for tick.
                None => recorded,
                // The flare's first tick takes the lever as it reads
                // (the roundout's floor may have put it back up).
                Some(cfg) if flaring => {
                    let measured = if was_flaring { lever } else { recorded };
                    fp.step(cfg, measured, v, TICK_S, h, Some(p.ias), Some(a[0].tgt_ias))
                        .unwrap_or(measured)
                }
                Some(cfg) => {
                    let rate = cfg
                        .retard_float_per_s
                        .or(cfg.retard_max_per_s)
                        .unwrap_or(cfg.reduction_per_second);
                    roundout_float_lever(lever, v, trend, TICK_S, rate)
                        .map_or(recorded, |l| l.min(recorded))
                }
            };
            was_flaring = flaring;
            p.write(e, lever);
            next += TICK_S;
        }
        if let Some((sink, speed, _)) = air {
            let j = times.partition_point(|t| *t <= p.t).saturating_sub(1);
            p.air_fpm = match (sink.get(j), sink.get(j + 1), times.get(j + 1)) {
                (Some(a0), Some(a1), Some(t1)) => {
                    a0 + (a1 - a0) * ((p.t - times[j]) / (t1 - times[j])).clamp(0.0, 1.0)
                }
                _ => *sink.last().unwrap_or(&0.0),
            };
            p.air_kts_s = speed.get(j).copied().unwrap_or(0.0);
        }
        let v = p.vvi;
        p.step(h, None, 0.01);
        h += v / 60.0 * 0.01;
        if flaring {
            out.climbed |= p.t > 3.0 && p.vvi > BALLOON_CLIMB_FPM;
            if p.t > 3.0 {
                out.climb_fpm = out.climb_fpm.max(p.vvi);
            }
            if p.vvi > -RETARD_FLOAT_FPM {
                out.float_s += 0.01;
            }
        }
    }
    out.sink_fpm = -p.vvi;
    out.touchdown_m = gs * KTS_TO_MPS * p.t - f.gate_short_m();
    out.lever_at_wheels = lever;
    out.ias_at_wheels = p.ias;
    out
}

/// Each arrest's own fit replays its record: the column and lever as
/// written, the airspeed as read, under 25 fpm rms (the 475, in 102 fpm rms
/// of air on its final, under 30).
#[test]
fn the_zibo_plant_replays_every_arrest() {
    for (f, (fit, trim)) in flares().iter().zip(OWN) {
        let (_, _, rms) = air_of(&f.arrest(), fit, trim);
        let limit = if f.touchdown_fpm > 400.0 && f.rough {
            30.0
        } else {
            25.0
        };
        assert!(rms < limit, "{}: {rms:.1} fpm rms", f.id);
    }
    // ...and the joint fit is one aircraft: every arrest on it, its trim
    // its own, under 30 fpm rms.
    for (gi, f) in flares().iter().enumerate() {
        let (_, _, rms) = air_of(&f.arrest(), JOINT, JOINT_TRIM[gi]);
        assert!(rms < 30.0, "{} on the joint fit: {rms:.1} fpm rms", f.id);
    }
}

/// The law that flew is the law in the harness: the tracker the Zibo
/// declared on 2026-10-07, fed each record's sink, writes the column that
/// was written.
#[test]
fn the_tracker_in_the_harness_is_the_one_that_flew() {
    for f in flares() {
        let a = f.arrest();
        let mut law = FlarePitch::with_gains(FlareGains::from_profile(Some(as_flown_elevator())));
        let mut worst: f64 = 0.0;
        for (n, k) in a.iter().enumerate() {
            let dt = if n == 0 { TICK_S } else { k.t - a[n - 1].t };
            let e = law.step(k.cmd, k.vvi, k.trend, dt, k.flare);
            worst = worst.max((e - k.elev.unwrap_or(0.0)).abs());
        }
        assert!(worst < 0.02, "{}: {worst:.3} off the record", f.id);
    }
}

/// Closed loop, the tracker as flown and the lever as written, through
/// each arrest's own fit and the air it carried: each lands within 70 fpm
/// of its recorded touchdown and 150 m of where, and the scatter is the
/// day's (a plant that cannot put a 475 next to a 209 cannot judge a fix).
#[test]
fn the_tracker_as_flown_lands_where_it_did() {
    let mut sinks = Vec::new();
    for (f, (fit, trim)) in flares().iter().zip(OWN) {
        let air = air_of(&f.arrest(), fit, trim);
        let l = fly(f, fit, trim, Law::of(as_flown_elevator()), None, Some(&air));
        assert!(
            (l.sink_fpm - f.touchdown_fpm).abs() < 70.0,
            "{}: {:.0} against the recorded {:.0}",
            f.id,
            l.sink_fpm,
            f.touchdown_fpm
        );
        // ...and where it did: inside 150 m of the rollout's first tick.
        assert!(
            (l.touchdown_m - f.rollout_m).abs() < 150.0,
            "{}: {:.0} m against the rollout's {:.0}",
            f.id,
            l.touchdown_m,
            f.rollout_m
        );
        sinks.push(l.sink_fpm);
    }
    let spread =
        sinks.iter().copied().fold(0.0, f64::max) - sinks.iter().copied().fold(f64::MAX, f64::min);
    assert!(spread > 200.0, "the day's scatter: {spread:.0}");
}

/// Every recorded gate on every plant in still air (36 landings), the
/// declared retard: the declared model lands all under 300 fpm and at least
/// 33 in the 100 to 250 a crew aims for, none climbing, the wheels down
/// inside 1,000 m of the threshold (LEMG 30's 2,751 m leaves the roll its
/// 792 from there). The tracker the Zibo flew lands 5 of them in the band
/// and 23 past 300.
#[test]
fn the_declared_flare_lands_every_gate_on_every_plant() {
    let (el, pw) = (declared_elevator(), declared_power());
    let (mut band, mut flown_band, mut flown_hard) = (0, 0, 0);
    for (gi, f) in flares().iter().enumerate() {
        for fit in PLANTS {
            let trim = trim_for(f, gi, fit);
            let l = fly(f, fit, trim, Law::of(el), Some(pw), None);
            assert!(
                l.sink_fpm > 60.0 && l.sink_fpm < 300.0,
                "gate {} on {fit:?}: {:.0} fpm",
                f.id,
                l.sink_fpm
            );
            assert!(!l.climbed, "gate {} on {fit:?}: ballooned", f.id);
            assert!(
                l.touchdown_m < 1000.0,
                "gate {} on {fit:?}: {:.0} m",
                f.id,
                l.touchdown_m
            );
            assert!(l.max_elev <= el.max + 1e-9);
            band += usize::from((100.0..=250.0).contains(&l.sink_fpm));
            let t = fly(
                f,
                fit,
                trim,
                Law::of(as_flown_elevator()),
                Some(as_flown_power()),
                None,
            );
            flown_band += usize::from((100.0..=250.0).contains(&t.sink_fpm));
            flown_hard += usize::from(t.sink_fpm > 300.0);
        }
    }
    assert!(band >= 33, "{band} of 36 in the band");
    assert!(
        flown_band <= 6 && flown_hard >= 20,
        "as flown: {flown_band} in the band, {flown_hard} past 300"
    );
}

/// The six recorded arrests replayed through their own fits and the air
/// each carried (the gusts of the 209, 326 and 475 included), now on the
/// declared flare and retard, the roundout's lever as written: every one
/// at 100 to 250 fpm (flown: 450, 203, 209, 259, 326, 475), in the first
/// 900 m.
#[test]
fn the_recorded_flares_replay_through_the_declared_flare() {
    let (el, pw) = (declared_elevator(), declared_power());
    for (f, (fit, trim)) in flares().iter().zip(OWN) {
        let air = air_of(&f.arrest(), fit, trim);
        let l = fly(f, fit, trim, Law::of(el), Some(pw), Some(&air));
        assert!(
            (100.0..=250.0).contains(&l.sink_fpm),
            "{} (flown {:.0}): {:.0} fpm",
            f.id,
            f.touchdown_fpm,
            l.sink_fpm
        );
        assert!(
            !l.climbed && l.touchdown_m < 900.0,
            "{}: {:.0} m",
            f.id,
            l.touchdown_m
        );
    }
}

/// Rough air: every gate on three plants (the joint fit and the two slow
/// and fast corners), five seeds of 40 fpm rms gusts each, the final
/// called rough (90 landings): the declared flare at 250 fpm mean or
/// under, no more than 2 past 400 and none past 480, no climb past 200 fpm
/// (a gust's own, 160 at most; the tracker's 101); the tracker as flown past
/// 330 mean with a third of them past 400.
#[test]
fn in_rough_air_the_declared_flare_holds_its_spread() {
    let (el, pw) = (declared_elevator(), declared_power());
    let (mut sum, mut hard, mut worst, mut n) = (0.0, 0, 0.0f64, 0.0);
    let (mut flown_sum, mut flown_hard, mut climb, mut flown_climb) = (0.0, 0, f64::MIN, f64::MIN);
    for (gi, f) in flares().iter().enumerate() {
        let f = Flare {
            rough: true,
            ..f.clone()
        };
        for fit in [PLANTS[0], PLANTS[3], PLANTS[5]] {
            let trim = trim_for(&f, gi, fit);
            for seed in 1..=5u64 {
                let air = gusts(&f.arrest(), seed + 10 * gi as u64, 40.0);
                let l = fly(&f, fit, trim, Law::of(el), Some(pw), Some(&air));
                climb = climb.max(l.climb_fpm);
                sum += l.sink_fpm;
                hard += usize::from(l.sink_fpm > 400.0);
                worst = worst.max(l.sink_fpm);
                n += 1.0;
                let t = fly(
                    &f,
                    fit,
                    trim,
                    Law::of(as_flown_elevator()),
                    Some(as_flown_power()),
                    Some(&air),
                );
                flown_sum += t.sink_fpm;
                flown_hard += usize::from(t.sink_fpm > 400.0);
                flown_climb = flown_climb.max(t.climb_fpm);
            }
        }
    }
    assert!(
        sum / n <= 250.0 && hard <= 2 && worst < 480.0 && climb < 200.0,
        "{:.0} mean, {hard} past 400, {worst:.0} worst, climbing {climb:.0} (as flown {flown_climb:.0})",
        sum / n
    );
    assert!(
        flown_sum / n > 330.0 && flown_hard >= 25,
        "as flown: {:.0} mean, {flown_hard} past 400",
        flown_sum / n
    );
}

/// The go/no-go at the roundout gate (`landing_call`) on the declared
/// flare (6.5 s to 180 fpm): LAND from each recorded gate at LEMG 30, the
/// touchdown predicted 596 to 898 m past the threshold, 1,850 m or more
/// left against the roll's 792 (the day's calls on the 5.5 s exponential:
/// 557 to 926; the approach's excess speed is most of it).
#[test]
fn the_landing_call_still_lands_every_recorded_gate() {
    let el = declared_elevator();
    let p = zibo();
    let need = wheels_need_m(
        p.reference
            .as_ref()
            .and_then(|r| r.landing_distance_ft)
            .map(|f| f * M_PER_FT),
        p.reference
            .as_ref()
            .and_then(|r| r.landing_roll_ft)
            .map(|f| f * M_PER_FT),
    );
    for f in flares() {
        let g = f.arrest()[0];
        let call = landing_call(
            f.gate_short_m(),
            2751.46,
            g.radio,
            f.gs_kts(),
            Some(g.ias),
            g.tgt_ias,
            el.tau_s,
            el.model.and_then(|m| m.touchdown_fpm),
            need,
        )
        .expect("a call");
        assert!(call.land && call.touchdown_m < 950.0, "{}: {call:?}", f.id);
    }
}

/// The declared block is a model the law flies (every number in range).
#[test]
fn the_zibo_flies_its_declared_model() {
    let el = declared_elevator();
    let m = el.valid().and_then(|v| v.model).expect("the model stands");
    assert!(!m.rate && m.rough_hold == Some(false) && m.fpm_per_kt.is_some());
    assert!(FlareModel::from_profile(Some(el)).is_some());
    assert_eq!(el.max_push, 0.0, "pull-only");
}

/// Sweep (ignored): the declared law, or a ZIBO_GRID of variations on it,
/// through every gate and plant, still and rough air, and the six own
/// replays; ZIBO_TABLE prints each still-air landing.
#[test]
#[ignore]
fn sweep_the_zibo_flare() {
    let fl = flares();
    let report = |name: &str, el: FlareElevatorProfile, pw: FlarePowerProfile| {
        let mut sinks = Vec::new();
        let (mut worst_m, mut climbs, mut floats) = (0.0f64, 0, 0.0f64);
        for (gi, f) in fl.iter().enumerate() {
            for fit in PLANTS {
                let l = fly(f, fit, trim_for(f, gi, fit), Law::of(el), Some(pw), None);
                if std::env::var("ZIBO_TABLE").is_ok() {
                    eprintln!(
                        "  gate {gi} plant {:.0}/{:.2}: {:.0} fpm, {:.0} m, max {:.2}, lever {:.2}, float {:.1}",
                        fit.k, fit.dead_s, l.sink_fpm, l.touchdown_m, l.max_elev, l.lever_at_wheels, l.float_s
                    );
                }
                sinks.push(l.sink_fpm);
                worst_m = worst_m.max(l.touchdown_m);
                climbs += usize::from(l.climbed);
                floats = floats.max(l.float_s);
            }
        }
        let mut rough = Vec::new();
        let mut rough_climb = f64::MIN;
        for (gi, f) in fl.iter().enumerate() {
            let f = Flare {
                rough: true,
                ..f.clone()
            };
            for fit in [PLANTS[0], PLANTS[3], PLANTS[5]] {
                for seed in 1..=5u64 {
                    let air = gusts(&f.arrest(), seed + 10 * gi as u64, 40.0);
                    let l = fly(
                        &f,
                        fit,
                        trim_for(&f, gi, fit),
                        Law::of(el),
                        Some(pw),
                        Some(&air),
                    );
                    rough.push(l.sink_fpm);
                    rough_climb = rough_climb.max(l.climb_fpm);
                }
            }
        }
        let rough_worst = rough.iter().copied().fold(0.0, f64::max);
        let rough_mean = rough.iter().sum::<f64>() / rough.len() as f64;
        let rough_hard = rough.iter().filter(|s| **s > 400.0).count();
        let rough_soft = rough.iter().filter(|s| **s < 60.0).count();
        let mut own = Vec::new();
        for (f, (fit, trim)) in fl.iter().zip(OWN) {
            let air = air_of(&f.arrest(), fit, trim);
            let l = fly(f, fit, trim, Law::of(el), Some(pw), Some(&air));
            own.push(format!("{:.0}/{:.0}m", l.sink_fpm, l.touchdown_m));
        }
        let min = sinks.iter().copied().fold(f64::MAX, f64::min);
        let max = sinks.iter().copied().fold(0.0, f64::max);
        let mean = sinks.iter().sum::<f64>() / sinks.len() as f64;
        let band = sinks
            .iter()
            .filter(|s| (100.0..=250.0).contains(*s))
            .count();
        let over = sinks.iter().filter(|s| **s > 300.0).count();
        eprintln!(
            "{name}: still {min:.0}..{max:.0} mean {mean:.0} band {band}/{} >300 {over}, far {worst_m:.0} m, climbs {climbs}, float {floats:.1} s | rough mean {rough_mean:.0} worst {rough_worst:.0} climb {rough_climb:.0} >400 {rough_hard} <60 {rough_soft}/{} | own+air {}",
            sinks.len(),
            rough.len(),
            own.join(" ")
        );
    };
    report("as flown", as_flown_elevator(), as_flown_power());
    report("declared", declared_elevator(), declared_power());
    for (name, e, p) in grid_combos() {
        report(&name, e, p);
    }
}

/// ZIBO_GRID="k=4800,5200;dead=1.6,1.8;..." over the declared numbers (keys
/// below; `rough` 1 holds the pull in rough air, `kv` 0 drops the speed).
fn grid_combos() -> Vec<(String, FlareElevatorProfile, FlarePowerProfile)> {
    let grid = std::env::var("ZIBO_GRID").unwrap_or_default();
    if grid.is_empty() {
        return Vec::new();
    }
    let (de, dp) = (declared_elevator(), declared_power());
    let dm = de.model.expect("a declared model");
    let get = |key: &str, dflt: f64| -> Vec<f64> {
        grid.split(';')
            .find_map(|kv| kv.strip_prefix(&format!("{key}=")))
            .map(|v| v.split(',').map(|x| x.parse().unwrap()).collect())
            .unwrap_or_else(|| vec![dflt])
    };
    let keys = [
        ("k", dm.fpm_per_elevator),
        ("lag", dm.lag_s),
        ("lag2", dm.lag2_s.unwrap_or(dm.lag_s)),
        ("dead", dm.dead_s),
        ("gfpm", dm.ground_fpm),
        ("gft", dm.ground_ft),
        ("hz", dm.horizon_s.unwrap_or(dm.dead_s + 1.0)),
        ("td", dm.touchdown_fpm.unwrap_or(150.0)),
        ("obs", dm.observer_s),
        ("rough", f64::from(u8::from(dm.rough_hold != Some(false)))),
        ("kv", dm.fpm_per_kt.unwrap_or(0.0)),
        ("tau", de.tau_s.unwrap_or(5.5)),
        ("max", de.max),
        ("ease", de.balloon_ease_per_s.unwrap_or(0.15)),
        ("float", dp.retard_float_per_s.unwrap_or(0.08)),
        ("rmax", dp.retard_max_per_s.unwrap_or(0.04)),
        ("below", dp.retard_below_ft.unwrap_or(20.0)),
        ("at", dp.retard_at_s.unwrap_or(2.0)),
        ("fast", dp.retard_fast_kts.unwrap_or(0.0)),
        ("slow", dp.retard_slow_kts.unwrap_or(5.0)),
    ];
    let mut combos: Vec<Vec<(&str, f64)>> = vec![vec![]];
    for (key, dflt) in keys {
        let vals = get(key, dflt);
        combos = combos
            .into_iter()
            .flat_map(|c| {
                vals.iter().map(move |v| {
                    let mut c = c.clone();
                    c.push((key, *v));
                    c
                })
            })
            .collect();
    }
    combos
        .into_iter()
        .map(|c| {
            let g = |k: &str| c.iter().find(|(n, _)| *n == k).map(|(_, v)| *v).unwrap();
            let m = FlareModelProfile {
                fpm_per_elevator: g("k"),
                lag_s: g("lag"),
                lag2_s: Some(g("lag2")),
                dead_s: g("dead"),
                ground_fpm: g("gfpm"),
                ground_ft: g("gft"),
                horizon_s: Some(g("hz")),
                touchdown_fpm: Some(g("td")),
                observer_s: g("obs"),
                rough_hold: Some(g("rough") > 0.5),
                fpm_per_kt: (g("kv") > 0.0).then(|| g("kv")),
                ..dm
            };
            let pw = FlarePowerProfile {
                retard_float_per_s: Some(g("float")),
                retard_max_per_s: Some(g("rmax")),
                retard_below_ft: Some(g("below")),
                retard_at_s: Some(g("at")),
                retard_fast_kts: Some(g("fast")),
                retard_slow_kts: Some(g("slow")),
                ..dp
            };
            let e = FlareElevatorProfile {
                max: g("max"),
                tau_s: Some(g("tau")),
                balloon_ease_per_s: Some(g("ease")),
                model: Some(m),
                ..de
            };
            let name: Vec<String> = c
                .iter()
                .filter(|(n, _)| get(n, f64::NAN).len() > 1)
                .map(|(n, v)| format!("{n} {v}"))
                .collect();
            (name.join(" "), e, pw)
        })
        .collect()
}
