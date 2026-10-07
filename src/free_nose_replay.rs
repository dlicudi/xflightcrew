//! Every recorded free-nose manoeuvre on file, replayed against the
//! identified plant (`ground_steer::GroundPlant`, and its brake at 0.7 and
//! 1.3), the law that flew it (as it stood on main at 776a820) against the
//! one ground law. `cargo test --release -- --ignored free_nose_replay
//! --nocapture` over `$XVL_BLACKBOX` (default the repository's own
//! `logs/blackbox.jsonl`): one row per manoeuvre, the scorer's measures.
//!
//! - A taxi drive (a corner-by-corner taxi, a backtrack's legs): its route
//!   rebuilt from the drive's own aim points (the pursuit aim IS a point of
//!   the route: the aircraft's dead-reckoned position, anchored on the 1 Hz
//!   `state` fixes, plus the logged aim distance along the logged heading
//!   error), checked against the logged `lat_m`; flown from the tick the
//!   scorer starts judging, the tick times as logged, the speed as logged
//!   less the brake drag the law adds or saves (6 m/s^2 per unit, the
//!   governor taking it back over 4 s), judged by `scorecard::LineJudge`.
//!   At LEMG the route is also planned again on the install's own network
//!   from where the drive began, and flown instead where it gives the
//!   logged lateral back better (the rebuild's feet carry the heading's
//!   error into knots and jogs no taxiway has; the `source` column).
//!   `FNR_TRACE=<t0>` prints the new law's ticks on one drive (its `t0`
//!   column), `FNR_DEBUG` the planned routes it turned down.
//! - A line-up alignment: from its first tick, the governor's creep and
//!   stop modelled, the judge's own bounds.
//! - A roll's start: to 25 kt, the recorded rudder and speed, the heading
//!   target the roll's own centreline law.
//! - A rollout: from the wheels to 5 kt, the recorded speed and stopping
//!   brake, the rollout loop's 0.4 s of delay, the weathervane and the track.
//!
//! The standing yaw (the propeller's) of each manoeuvre is fitted to its own
//! record first (open loop, the recorded inputs through the plant).
#![cfg(test)]

use crate::ground_steer::{self, Demand, GroundPlant, GroundSteer, Rudder, Yaw};
use serde_json::Value;

const KT: f64 = 1.943_844;
const DRAG_MS2: f64 = 6.0;

fn f(v: &Value, k: &str) -> Option<f64> {
    v.get(k).and_then(Value::as_f64)
}

/// The plant with its weathervane and track, the command reaching it
/// `io` s after it is written.
#[derive(Clone)]
struct Sim {
    p: GroundPlant,
    w: f64,
    io: f64,
    r: f64,
    psi: f64,
    trk: f64,
    t: f64,
    x: f64,
    y: f64,
    sent: Vec<(f64, f64, f64)>,
}

impl Sim {
    fn new(p: GroundPlant, w: f64, io: f64, psi: f64, trk: f64, r: f64) -> Self {
        Sim { p, w, io, r, psi, trk, t: 0.0, x: 0.0, y: 0.0, sent: vec![(-10.0, 0.0, 0.0)] }
    }

    /// Write (u, rudder), fly `dt` at `v`; the mean yaw rate.
    fn run(&mut self, u: f64, rud: f64, v: f64, dt: f64) -> f64 {
        self.sent.push((self.t, u, rud));
        let n = (dt / 0.01).round().max(1.0) as usize;
        let h = dt / n as f64;
        let psi0 = self.psi;
        let (kb, d, kr, kv, kt) = (self.p.kb(v), self.p.d(v), self.p.kr(v), self.p.kv(v), self.p.kt(v));
        for _ in 0..n {
            let (_, u_d, r_d) = *self.sent.iter().rev().find(|s| s.0 <= self.t - self.io + 1e-9).unwrap_or(&(0.0, 0.0, 0.0));
            let w = if v > 0.3 { self.w } else { 0.0 };
            let slip = crate::angles::wrap180(self.psi - self.trk);
            let rd = kb * u_d + kr * r_d - d * self.r - kv * slip + w;
            let r1 = self.r + rd * h;
            self.psi += (self.r + r1) / 2.0 * h;
            self.r = r1;
            // The track follows the nose; at a walk it is the nose.
            self.trk += (kt * slip * h).clamp(-slip.abs(), slip.abs());
            let trk = self.trk.to_radians();
            self.x += v * trk.sin() * h;
            self.y += v * trk.cos() * h;
            self.t += h;
        }
        if self.sent.len() > 64 {
            self.sent.drain(..32);
        }
        (self.psi - psi0) / dt
    }
}

/// The standing yaw of a record: the plant driven by the recorded inputs in
/// 3 s windows from the recorded rate and heading, the `w` whose heading
/// comes closest.
fn fit_w(p: GroundPlant, io: f64, ticks: &[(f64, f64, f64, f64, f64, f64)]) -> f64 {
    // (t, v m/s, u, rudder, psi, r)
    let mut best = (f64::INFINITY, 0.0);
    let mut w = -4.0;
    while w <= 4.0 + 1e-9 {
        let mut se = 0.0;
        let mut n = 0;
        let mut k = 0;
        while k + 1 < ticks.len() {
            let mut sim = Sim::new(p, w, io, ticks[k].4, ticks[k].4, ticks[k].5);
            let t0 = ticks[k].0;
            let mut j = k;
            while j + 1 < ticks.len() && ticks[j + 1].0 - t0 <= 3.0 {
                let dt = (ticks[j + 1].0 - ticks[j].0).max(0.05);
                sim.run(ticks[j].2, ticks[j].3, ticks[j].1, dt);
                se += crate::angles::wrap180(sim.psi - ticks[j + 1].4).powi(2);
                n += 1;
                j += 1;
            }
            k = j.max(k + 1);
        }
        let rms = (se / n.max(1) as f64).sqrt();
        if rms < best.0 {
            best = (rms, w);
        }
        w += 0.25;
    }
    best.1
}

/// The records of the free-nose airframes, in order, with their airframe.
fn load() -> Vec<(String, Value)> {
    let path = std::env::var("XVL_BLACKBOX").unwrap_or_else(|_| {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("logs/blackbox.jsonl").to_string_lossy().into_owned()
    });
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut ac = String::new();
    let mut out = Vec::new();
    for line in text.lines() {
        let Ok(v) = serde_json::from_str::<Value>(line) else { continue };
        let kind = v.get("kind").and_then(Value::as_str).unwrap_or("");
        if kind == "flight_meta" {
            ac = v.get("aircraft").and_then(Value::as_str).unwrap_or("").to_string();
            continue;
        }
        if !matches!(ac.as_str(), "cirrus_sr22" | "lancair_evolution" | "vans_rv10") {
            continue;
        }
        if matches!(kind, "autotaxi" | "autotakeoff" | "autoland" | "state") {
            out.push((ac.clone(), v));
        }
    }
    out
}

fn plant_of(ac: &str) -> GroundPlant {
    let p = crate::aircraft::load(ac).and_then(|p| p.ground_plant).and_then(crate::aircraft::GroundPlantProfile::valid);
    p.map_or(GroundPlant::SR22, |p| GroundPlant {
        brake_dps2: p.brake_dps2,
        brake_from_ms: p.brake_from_ms,
        brake_per_ms: p.brake_per_ms,
        damp: p.damp,
        damp_per_ms: p.damp_per_ms,
        rudder: p.rudder_per_ms2,
        vane_dps2: p.vane_dps2,
        side_ms2: p.side_ms2,
    })
}

fn event(v: &Value) -> &str {
    v.get("event").and_then(Value::as_str).unwrap_or("")
}

fn kind(v: &Value) -> &str {
    v.get("kind").and_then(Value::as_str).unwrap_or("")
}

/// Runs of consecutive records of (kind, event), split at gaps over 2 s.
fn runs<'a>(recs: &'a [(String, Value)], k: &str, e: &str) -> Vec<Vec<&'a (String, Value)>> {
    let mut out: Vec<Vec<&(String, Value)>> = Vec::new();
    let mut last: Option<f64> = None;
    for r in recs {
        if kind(&r.1) != k || event(&r.1) != e {
            continue;
        }
        let t = f(&r.1, "t").unwrap_or(0.0);
        if last.is_none_or(|l| t - l > 2.0) {
            out.push(Vec::new());
        }
        out.last_mut().unwrap().push(r);
        last = Some(t);
    }
    out
}

// ---------------------------------------------------------------------------
// The laws as they stood on main at 776a820.
// ---------------------------------------------------------------------------

mod legacy {
    use crate::autotaxi::{pivot_brake_split, pivot_due, slew_steer, smooth_toward, PIVOT_RATE_DPS};

    const DEADBAND: f64 = 0.05;
    const MAX: f64 = 0.25;
    const SAT: f64 = 0.6;
    const SLEW: f64 = 0.4;
    const TAU: f64 = 0.4;
    const FREE_MIN: f64 = 1.5;
    const FREE_FULL: f64 = 4.0;
    const LEAD_S: f64 = 1.2;
    const RATE_SPAN: f64 = 15.0;
    const SWING_DPS: f64 = 8.0;
    const SWING_SPAN: f64 = 30.0;
    const SWING_MAX: f64 = 0.2;
    const TURN_ERR: f64 = 25.0;
    const TURN_S: f64 = 2.0;
    const TURN_MAX_DPS: f64 = 15.0;
    const TURN_FULL: f64 = 15.0;
    const TURN_MAX: f64 = 0.12;
    const WRONG_DPS: f64 = 1.5;
    const WRONG_SPAN: f64 = 4.0;
    const WRONG_MAX: f64 = 0.3;

    fn walk(gs: f64) -> f64 {
        ((gs - 0.5) / 1.0).clamp(0.0, 1.0)
    }

    fn split_free(steer: f64, gs: f64) -> (f64, f64) {
        let a = ((gs - FREE_MIN) / (FREE_FULL - FREE_MIN)).clamp(0.0, 1.0);
        let e = ((steer.abs() - DEADBAND) / (SAT - DEADBAND)).clamp(0.0, 1.0);
        let b = e * MAX * a;
        if steer > 0.0 {
            (0.0, b)
        } else {
            (b, 0.0)
        }
    }

    /// `BrakeSteer::step` on a free nose (the swing catch, the turn
    /// builder, the led ceiling, the wrong-way pivot brake, the instant
    /// release).
    #[derive(Default)]
    pub struct BrakeSteer {
        smoothed: f64,
        prev: (f64, f64),
        prev_yaw: Option<f64>,
    }

    impl BrakeSteer {
        pub fn step(&mut self, steer: f64, err: f64, gs: f64, yaw: f64, joining: bool, dt: f64) -> (f64, f64) {
            let pivot = pivot_due(err, gs, joining);
            self.smoothed = smooth_toward(self.smoothed, steer, TAU, dt);
            let (wl, wr) = if pivot { pivot_brake_split(steer, gs) } else { split_free(self.smoothed, gs) };
            let turning = if pivot { steer } else { self.smoothed };
            let led = match self.prev_yaw {
                Some(p) => {
                    let acc = (yaw - p) / dt.max(0.1);
                    if acc * yaw > 0.0 {
                        yaw + acc * LEAD_S
                    } else {
                        yaw
                    }
                }
                None => yaw,
            };
            let k = if turning * led <= 0.0 { 1.0 } else { (1.0 - (led.abs() - PIVOT_RATE_DPS) / RATE_SPAN).clamp(0.0, 1.0) };
            self.prev_yaw = Some(yaw);
            let (mut wl, mut wr) = (wl * k, wr * k);
            if pivot && steer * yaw < 0.0 {
                let b = ((yaw.abs() - WRONG_DPS) / WRONG_SPAN).clamp(0.0, 1.0) * WRONG_MAX;
                if steer > 0.0 {
                    (wl, wr) = (0.0, wr.max(b));
                } else {
                    (wl, wr) = (wl.max(b), 0.0);
                }
            }
            if steer * yaw < 0.0 && yaw.abs() > SWING_DPS {
                let b = ((yaw.abs() - SWING_DPS) / SWING_SPAN).clamp(0.0, 1.0) * SWING_MAX * walk(gs);
                if yaw < 0.0 {
                    (wl, wr) = (0.0, wr.max(b));
                } else {
                    (wl, wr) = (wl.max(b), 0.0);
                }
            }
            if err.abs() >= TURN_ERR {
                let want = (err.abs() / TURN_S).min(TURN_MAX_DPS);
                let toward = yaw * err.signum();
                let b = ((want - toward) / TURN_FULL).clamp(0.0, 1.0) * TURN_MAX * walk(gs);
                if b > 0.0 {
                    if err > 0.0 {
                        (wl, wr) = (0.0, wr.max(b));
                    } else {
                        (wl, wr) = (wl.max(b), 0.0);
                    }
                }
            }
            let ease = |p: f64, w: f64| if w < p { w } else { slew_steer(p, w, Some(SLEW), dt) };
            self.prev = (ease(self.prev.0, wl), ease(self.prev.1, wr));
            self.prev
        }
    }

    /// The SR22's declared rollout split (`rollout_brakes:`, LEMG 13
    /// 1790694457): 0.02 per degree at 40 kt (0.4 to 1.5 of it with the
    /// speed), 0.8 s of lead on a rate smoothed over 0.25 s, a trim of 0.6
    /// per second to 0.15.
    #[derive(Default)]
    pub struct Sr22Split {
        prev: Option<f64>,
        rate: f64,
        trim: f64,
    }

    impl Sr22Split {
        /// The wheel difference, right positive.
        pub fn step(&mut self, err: f64, dt: f64, gs: f64) -> f64 {
            if let Some(p) = self.prev {
                let raw = (err - p) / dt.clamp(0.05, 3.0);
                let d = dt.clamp(0.05, 3.0);
                self.rate += (raw - self.rate) * d / (0.25 + d);
            }
            self.prev = Some(err);
            if gs < 4.0 {
                return 0.0;
            }
            let per = 0.02 * (gs / 40.0).clamp(0.4, 1.5);
            self.trim = (self.trim + per * 0.6 * err * dt.clamp(0.05, 1.0)).clamp(-0.15, 0.15);
            (per * (err + 0.8 * self.rate) + self.trim).clamp(-0.3, 0.3)
        }
    }
}

// ---------------------------------------------------------------------------
// The taxi drives.
// ---------------------------------------------------------------------------

struct DriveTick {
    t: f64,
    gs: f64,
    err: f64,
    ema: f64,
    raw: f64,
    steer: f64,
    u: f64,
    lat: f64,
    rem: f64,
    look: f64,
    turn: f64,
}

struct Geo {
    lat0: f64,
    lon0: f64,
    m_lon: f64,
}

impl Geo {
    fn new(lat0: f64, lon0: f64) -> Self {
        Geo { lat0, lon0, m_lon: 111_320.0 * lat0.to_radians().cos() }
    }
    fn xy(&self, lat: f64, lon: f64) -> (f64, f64) {
        ((lon - self.lon0) * self.m_lon, (lat - self.lat0) * 111_320.0)
    }
    fn ll(&self, x: f64, y: f64) -> (f64, f64) {
        (self.lat0 + y / 111_320.0, self.lon0 + x / self.m_lon)
    }
}

fn interp(ts: &[f64], vs: &[f64], t: f64) -> f64 {
    if t <= ts[0] {
        return vs[0];
    }
    for i in 1..ts.len() {
        if t <= ts[i] {
            let a = (t - ts[i - 1]) / (ts[i] - ts[i - 1]).max(1e-9);
            return vs[i - 1] + a * (vs[i] - vs[i - 1]);
        }
    }
    *vs.last().unwrap()
}

/// Ramer-Douglas-Peucker on a polyline.
fn rdp(pts: &[(f64, f64)], tol: f64) -> Vec<(f64, f64)> {
    if pts.len() < 3 {
        return pts.to_vec();
    }
    let (a, b) = (pts[0], pts[pts.len() - 1]);
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len = (dx * dx + dy * dy).sqrt().max(1e-9);
    let (mut worst, mut at) = (0.0, 0);
    for (i, p) in pts.iter().enumerate().skip(1).take(pts.len() - 2) {
        let d = ((p.0 - a.0) * dy - (p.1 - a.1) * dx).abs() / len;
        if d > worst {
            worst = d;
            at = i;
        }
    }
    if worst <= tol {
        return vec![a, b];
    }
    let mut left = rdp(&pts[..=at], tol);
    let right = rdp(&pts[at..], tol);
    left.pop();
    left.extend(right);
    left
}

/// A drive's route rebuilt from its aim points, its ticks' positions and
/// headings, and how well the rebuilt route gives back the logged lateral.
struct Rebuilt {
    route: crate::autotaxi::Route,
    geo: Geo,
    pos: Vec<(f64, f64)>,
    psi: Vec<f64>,
    fit_rms: f64,
    corners: Vec<f64>,
}

/// The `state` record is stamped after its reads: its fixes are `offset`
/// s older than its stamp (about 2 s on the iMac; [`best_rebuild`] finds it
/// per drive by the rebuilt route's fit).
fn rebuild(ticks: &[DriveTick], states: &[(f64, f64, f64, f64)], pace: crate::autotaxi::Pace, offset: f64) -> Option<Rebuilt> {
    if states.len() < 5 || ticks.len() < 20 {
        return None;
    }
    let geo = Geo::new(states[0].1, states[0].2);
    // The heading: the drive's own rate integrated, anchored on `state`.
    let mut integ = vec![0.0; ticks.len()];
    for k in 1..ticks.len() {
        integ[k] = integ[k - 1] + ticks[k].raw * (ticks[k].t - ticks[k - 1].t);
    }
    let tt: Vec<f64> = ticks.iter().map(|k| k.t).collect();
    let st: Vec<f64> = states.iter().map(|s| s.0 + offset).collect();
    let mut hdg = Vec::new();
    let mut prev: Option<f64> = None;
    for s in states {
        let h = match prev {
            Some(p) => p + crate::angles::wrap180(s.3 - p),
            None => s.3,
        };
        hdg.push(h);
        prev = Some(h);
    }
    let corr: Vec<f64> = states.iter().zip(&hdg).map(|(s, h)| h - interp(&tt, &integ, s.0 + offset)).collect();
    let psi: Vec<f64> = ticks.iter().zip(&integ).map(|(k, i)| i + interp(&st, &corr, k.t)).collect();
    // The position: dead-reckoned, anchored on `state`.
    let (mut dx, mut dy) = (vec![0.0; ticks.len()], vec![0.0; ticks.len()]);
    for k in 1..ticks.len() {
        let dt = ticks[k].t - ticks[k - 1].t;
        let v = (ticks[k].gs + ticks[k - 1].gs) / 2.0 / KT;
        let h = ((psi[k] + psi[k - 1]) / 2.0).to_radians();
        dx[k] = dx[k - 1] + v * h.sin() * dt;
        dy[k] = dy[k - 1] + v * h.cos() * dt;
    }
    let (cx, cy): (Vec<f64>, Vec<f64>) = states
        .iter()
        .map(|s| {
            let (x, y) = geo.xy(s.1, s.2);
            (x - interp(&tt, &dx, s.0 + offset), y - interp(&tt, &dy, s.0 + offset))
        })
        .unzip();
    let pos: Vec<(f64, f64)> = ticks
        .iter()
        .enumerate()
        .map(|(k, tk)| (dx[k] + interp(&st, &cx, tk.t), dy[k] + interp(&st, &cy, tk.t)))
        .collect();
    // The route under the aircraft: its position less the logged lateral
    // (the feet, while near the line; the normal off the heading, which
    // at 2 m off and 20 degrees costs 0.1 m), and the aim points past the
    // last foot (the route's end) and wherever the feet leave a gap.
    let mut aims: Vec<(f64, (f64, f64))> = Vec::new();
    for (k, tk) in ticks.iter().enumerate() {
        if tk.lat.abs() <= 3.0 {
            let h = psi[k].to_radians();
            aims.push((-tk.rem, (pos[k].0 - tk.lat * h.cos(), pos[k].1 + tk.lat * h.sin())));
        }
    }
    // The route's last stretch, which no foot reaches: the last aims.
    let last_foot = aims.iter().map(|a| a.0).fold(f64::NEG_INFINITY, f64::max);
    for (k, tk) in ticks.iter().enumerate() {
        if tk.rem > 9.0 && tk.look > 0.0 && -tk.rem + tk.look > last_foot + 1.0 {
            let b = (psi[k] + tk.err).to_radians();
            aims.push((-tk.rem + tk.look, (pos[k].0 + tk.look * b.sin(), pos[k].1 + tk.look * b.cos())));
        }
    }
    // The start: the foot of the first tick.
    let h0 = psi[0].to_radians();
    let foot0 = (pos[0].0 - ticks[0].lat * h0.cos(), pos[0].1 + ticks[0].lat * h0.sin());
    aims.push((-ticks[0].rem, foot0));
    aims.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    // One point per metre of arc: the median of the aims there.
    let mut pts: Vec<(f64, f64)> = Vec::new();
    let mut i = 0;
    while i < aims.len() {
        let bin = aims[i].0.floor();
        let mut xs = Vec::new();
        let mut ys = Vec::new();
        while i < aims.len() && aims[i].0.floor() == bin {
            xs.push(aims[i].1 .0);
            ys.push(aims[i].1 .1);
            i += 1;
        }
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
        pts.push((xs[xs.len() / 2], ys[ys.len() / 2]));
    }
    // The aims scatter about half a metre (the heading's error over the
    // aim): vertices only where the route leaves a line by more.
    let mut pts = rdp(&pts, 0.6);
    // On past the last aim to the route's end (the stop is 5 m short).
    if pts.len() >= 2 {
        let (a, b) = (pts[pts.len() - 2], pts[pts.len() - 1]);
        let l = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt().max(1e-6);
        pts.push((b.0 + (b.0 - a.0) / l * 20.0, b.1 + (b.1 - a.1) / l * 20.0));
    }
    let path: Vec<(f64, f64)> = pts.iter().map(|&(x, y)| geo.ll(x, y)).collect();
    let watch = crate::party_line::TaxiWatch {
        seg: vec![(String::new(), false); path.len().saturating_sub(1)],
        path,
        via: String::new(),
        holding_point: String::new(),
        lead_on: Vec::new(),
    };
    let mut route = crate::autotaxi::Route::build(&watch)?;
    route.pace = pace;
    // How well it gives back the logged lateral.
    // How well it holds the drive's own aim points (independent of the
    // lateral the feet were built from): each aim is a point of the route.
    let (mut se, mut n) = (0.0, 0);
    for (k, tk) in ticks.iter().enumerate() {
        if tk.rem <= 9.0 || tk.look <= 0.0 || tk.lat.abs() > 3.0 {
            continue;
        }
        let b = (psi[k] + tk.err).to_radians();
        let aim = (pos[k].0 + tk.look * b.sin(), pos[k].1 + tk.look * b.cos());
        let (la, lo) = geo.ll(aim.0, aim.1);
        if let Some((_, off)) = route.fix_signed(la, lo) {
            // An aim is `look` ahead along the heading error: its angle is
            // what the law flew on, so the error is read as an angle's worth.
            se += off.powi(2);
            n += 1;
        }
    }
    // The corners of the rebuilt route (vertices of 30 degrees and more),
    // along-route.
    let mut corners = Vec::new();
    let mut cum = 0.0;
    for w in pts.windows(3) {
        cum += ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
        let b1 = (w[1].0 - w[0].0).atan2(w[1].1 - w[0].1).to_degrees();
        let b2 = (w[2].0 - w[1].0).atan2(w[2].1 - w[1].1).to_degrees();
        if crate::angles::wrap180(b2 - b1).abs() >= 30.0 {
            corners.push(cum);
        }
    }
    Some(Rebuilt { route, geo, pos, psi, fit_rms: (se / n.max(1) as f64).sqrt(), corners })
}

/// [`rebuild`] at the state offset whose route fits the logged lateral best.
fn best_rebuild(ticks: &[DriveTick], states: &[(f64, f64, f64, f64)], pace: crate::autotaxi::Pace) -> Option<(Rebuilt, f64)> {
    let mut best: Option<(Rebuilt, f64)> = None;
    for k in 0..=8 {
        let off = -3.0 + 0.25 * k as f64;
        if let Some(rb) = rebuild(ticks, states, pace, off) {
            if best.as_ref().is_none_or(|b| rb.fit_rms < b.0.fit_rms) {
                best = Some((rb, off));
            }
        }
    }
    best
}

/// The LEMG taxi network and runways from the local install, once.
fn lemg() -> Option<&'static (atc::nav::TaxiNet, Vec<atc::nav::Runway>)> {
    static NET: std::sync::OnceLock<Option<(atc::nav::TaxiNet, Vec<atc::nav::Runway>)>> = std::sync::OnceLock::new();
    NET.get_or_init(|| {
        let apt =
            crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
        let net = atc::nav::find_taxi_net(&apt, "LEMG").ok()??;
        let rwys = atc::nav::find_runways(&apt, "LEMG").ok()?;
        Some((net, rwys))
    })
    .as_ref()
}

/// The logged lateral given back by `route` over the drive's ticks on the
/// line: (rms of the difference, rms of the remaining's difference).
fn lateral_fit(ticks: &[DriveTick], rb: &Rebuilt, route: &crate::autotaxi::Route) -> (f64, f64) {
    let (mut se, mut sr, mut n) = (0.0, 0.0, 0);
    for (k, tk) in ticks.iter().enumerate() {
        if tk.lat.abs() > 3.0 {
            continue;
        }
        let (la, lo) = rb.geo.ll(rb.pos[k].0, rb.pos[k].1);
        if let Some((p, off)) = route.fix_signed(la, lo) {
            se += (off - tk.lat).powi(2);
            sr += (route.remaining(p) - tk.rem).powi(2);
            n += 1;
        }
    }
    let n = n.max(1) as f64;
    ((se / n).sqrt(), (sr / n).sqrt())
}

/// A route's corners: along-route positions where it turns 30 degrees or
/// more within 15 m (a drawn fillet as one corner), metres.
fn corners_of(pts: &[(f64, f64)]) -> Vec<f64> {
    let mut cum = vec![0.0];
    for w in pts.windows(2) {
        cum.push(cum.last().unwrap() + ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt());
    }
    let brg = |i: usize| (pts[i + 1].0 - pts[i].0).atan2(pts[i + 1].1 - pts[i].1).to_degrees();
    let mut out: Vec<f64> = Vec::new();
    for i in 0..pts.len().saturating_sub(1) {
        if cum[i + 1] - cum[i] < 0.3 {
            continue;
        }
        let mut turn = 0.0_f64;
        let mut j = i + 1;
        while j + 1 < pts.len() && cum[j] - cum[i + 1] <= 15.0 {
            if cum[j + 1] - cum[j] >= 0.3 {
                turn = turn.max(crate::angles::wrap180(brg(j) - brg(i)).abs());
            }
            j += 1;
        }
        if turn >= 30.0 && out.last().is_none_or(|&c| cum[i + 1] - c > 20.0) {
            out.push(cum[i + 1]);
        }
    }
    out
}

/// The drive's route planned again from the install's LEMG network, from
/// where the rebuild puts its first tick, to whichever runway gives back the
/// logged lateral and remaining best; with its fit.
fn planned(ticks: &[DriveTick], rb: &Rebuilt, pace: crate::autotaxi::Pace) -> Option<(crate::autotaxi::Route, Vec<f64>, (f64, f64))> {
    let (net, rwys) = lemg()?;
    let (la, lo) = rb.geo.ll(rb.pos[0].0, rb.pos[0].1);
    // Only a drive on the field.
    if (la - 36.675).abs() > 0.03 || (lo + 4.49).abs() > 0.04 {
        return None;
    }
    let mut best: Option<(crate::autotaxi::Route, Vec<f64>, (f64, f64))> = None;
    for strip in rwys {
        for (tlat, tlon) in [(strip.lat_a, strip.lon_a), (strip.lat_b, strip.lon_b)] {
            let standoff = (strip.width_m / 2.0 + 30.0).max(atc::nav::HOLD_SHORT_STANDOFF_M);
            let zone = [strip.ident_a.clone(), strip.ident_b.clone()];
            let Some(plan) = net.plan_to_runway(la, lo, tlat, tlon, standoff, &zone, Some(strip)) else { continue };
            if plan.path.len() < 2 {
                continue;
            }
            let pts: Vec<(f64, f64)> = plan.path.iter().map(|&(a, b)| rb.geo.xy(a, b)).collect();
            let watch = crate::party_line::TaxiWatch {
                seg: plan.seg.clone(),
                path: plan.path.clone(),
                via: String::new(),
                holding_point: plan.holding_point.clone(),
                lead_on: Vec::new(),
            };
            let Some(mut route) = crate::autotaxi::Route::build(&watch) else { continue };
            route.pace = pace;
            let fit = lateral_fit(ticks, rb, &route);
            if best.as_ref().is_none_or(|b| fit.0 + fit.1 / 10.0 < b.2 .0 + b.2 .1 / 10.0) {
                best = Some((route, corners_of(&pts), fit));
            }
        }
    }
    best
}

#[derive(Clone, Copy, PartialEq)]
enum TaxiLaw {
    Old,
    New,
}

/// The scorer's verdict on one flown drive: (most off the line, most cut
/// inside a vertex, per-corner worst off the line).
struct Verdict {
    lat: f64,
    cut: f64,
    corners: Vec<f64>,
}

impl Verdict {
    fn scale(&self) -> f64 {
        use crate::scorecard::{TAXI_MAX_CUT_M, TAXI_MAX_LAT_M};
        self.lat.max(self.cut - (TAXI_MAX_CUT_M - TAXI_MAX_LAT_M))
    }
}

fn corner_worst(samples: &[(f64, f64)], corners: &[f64]) -> Vec<f64> {
    corners
        .iter()
        .map(|&c| samples.iter().filter(|(s, _)| (s - c).abs() <= 15.0).map(|(_, l)| l.abs()).fold(0.0, f64::max))
        .collect()
}

fn judge_flown(ticks: &[DriveTick], k0: usize, rb: &Rebuilt) -> Verdict {
    let mut judge = crate::scorecard::LineJudge::default();
    let mut samples = Vec::new();
    for (k, tk) in ticks.iter().enumerate().skip(k0) {
        if tk.rem < 0.0 {
            break;
        }
        let asked = (tk.turn.abs() >= crate::scorecard::CORNER_TURN_DPS).then(|| tk.turn.signum());
        let cut = judge.corner(tk.t, tk.gs, tk.lat, Some(tk.rem), asked, tk.ema);
        judge.judge(tk.lat, cut);
        let (la, lo) = rb.geo.ll(rb.pos[k].0, rb.pos[k].1);
        if let Some((progress, _)) = rb.route.fix_signed(la, lo) {
            samples.push((progress, tk.lat));
        }
    }
    judge.flush();
    Verdict { lat: judge.max_lat, cut: judge.max_cut, corners: corner_worst(&samples, &rb.corners) }
}

fn fly_drive(ticks: &[DriveTick], k0: usize, rb: &Rebuilt, plant: GroundPlant, law_plant: GroundPlant, w: f64, law: TaxiLaw) -> Verdict {
    use crate::autotaxi::*;
    let route = &rb.route;
    let pace = route.pace;
    let (x0, y0) = rb.pos[k0];
    let mut sim = Sim::new(plant, w, ground_steer::IO_DELAY_S, rb.psi[k0], rb.psi[k0], ticks[k0].raw);
    sim.x = x0;
    sim.y = y0;
    let ref_path = route.ref_path();
    let mut ground = GroundSteer::new(law_plant);
    let mut legacy = legacy::BrakeSteer::default();
    let mut trim = StraightTrim::default();
    let (mut dir, mut ref_dir) = (None, None);
    let (mut ema, mut prev_psi, mut prev_steer) = (ticks[k0].ema, sim.psi, ticks[k0].steer);
    let mut dv = 0.0;
    let mut judge = crate::scorecard::LineJudge::default();
    let mut samples = Vec::new();
    let max_yaw = pace.max_yaw_dps.unwrap_or(ground_steer::DEFAULT_MAX_YAW_DPS);
    for k in k0..ticks.len() - 1 {
        let tk = &ticks[k];
        let dt = (ticks[k + 1].t - tk.t).clamp(0.05, 1.5);
        let v = (tk.gs / KT + dv).max(0.0);
        let gs = v * KT;
        let (lat, lon) = rb.geo.ll(sim.x, sim.y);
        let psi = sim.psi.rem_euclid(360.0);
        let Some((progress, offset)) = route.fix_signed(lat, lon) else { break };
        let remaining = route.remaining(progress);
        if remaining < 0.0 {
            break;
        }
        let look = steer_look_m(
            &pace,
            gs,
            route.route_turn_rate_dps(progress, BEND_PROBE_M, gs),
            remaining,
            true,
        );
        let err = if remaining <= 8.0 {
            route.stop_heading_err_deg(lat, lon, psi, progress, look)
        } else {
            route.heading_err_deg(lat, lon, psi, progress, look)
        };
        let route_turn = if remaining <= 8.0 { 0.0 } else { route.route_turn_rate_dps(progress, look, gs) };
        let raw = crate::angles::wrap180(psi - prev_psi) / dt.max(0.05);
        ema = 0.5 * ema + 0.5 * raw.clamp(-90.0, 90.0);
        let (u, rud) = match law {
            TaxiLaw::Old => {
                // The drive as it stood: the 12 m corner aim on a free nose.
                let old_look = lookahead_m_with(gs, pace.lookahead_s)
                    .min(bend_lookahead_cap_floor_m(gs, route.route_turn_rate_dps(progress, BEND_PROBE_M, gs), 12.0))
                    .min((remaining * 0.6).max(6.0));
                let e = if remaining <= 8.0 {
                    route.stop_heading_err_deg(lat, lon, psi, progress, old_look)
                } else {
                    route.heading_err_deg(lat, lon, psi, progress, old_look)
                };
                let e = committed_err(e, &mut dir);
                let rt = if remaining <= 8.0 { 0.0 } else { route.route_turn_rate_dps(progress, old_look, gs) };
                let tr = trim.step(offset, rt, gs, false, remaining, dt);
                let want = stop_straighten(
                    (steer_cmd_with(damped_err_deg(e, ema, rt, pace.steer_lead_s), pace.steer_full_deg) + tr).clamp(-1.0, 1.0)
                        * steer_authority(gs),
                    remaining,
                    gs,
                );
                let steer = slew_steer(prev_steer, want, pace.steer_slew_per_s, dt);
                prev_steer = steer;
                let (bl, br) = legacy.step(steer, e, gs, ema, false, dt);
                (br - bl, steer)
            }
            TaxiLaw::New => {
                let (e, ff) = if remaining <= 8.0 {
                    (err, 0.0)
                } else {
                    ground_steer::path_demand(&ref_path, progress, route.local_m(lat, lon), psi, v, &ground, max_yaw)
                };
                let e = committed_err(e, &mut ref_dir);
                let s = ground.step(Demand {
                    dt,
                    v_ms: v,
                    yaw: Yaw::Now(sim.r),
                    err_deg: e,
                    ff_dps: ff,
                    rudder: Rudder::Own { slew_per_s: pace.steer_slew_per_s.unwrap_or(0.8) },
                    u_max: PIVOT_BRAKE_MAX,
                    r_max_dps: ground.taxi_rate_limit(v, PIVOT_BRAKE_MAX, max_yaw),
                    io_delay_s: ground_steer::IO_DELAY_S,
                });
                if std::env::var("FNR_TRACE").is_ok_and(|d| d == format!("{}", ticks[0].t as i64)) && plant == law_plant {
                    eprintln!(
                        "trace k {k} prog {progress:7.1} off {offset:6.2} rec_lat {:6.2} gs {gs:5.1} rec_gs {:5.1} psi {psi:6.1} e {e:6.1} ff {ff:6.1} r {:6.1} rref {:6.1} u {:5.2} rud {:5.2} w {:5.2}",
                        tk.lat, tk.gs, sim.r, s.r_ref, s.diff, s.rudder, s.w_hat
                    );
                }
                (s.diff, s.rudder)
            }
        };
        let asked = (route_turn.abs() >= crate::scorecard::CORNER_TURN_DPS).then(|| route_turn.signum());
        let cut = judge.corner(tk.t, gs, offset, Some(remaining), asked, ema);
        judge.judge(offset, cut);
        samples.push((progress, offset));
        prev_psi = psi;
        sim.run(u, rud, v, dt);
        // The governor takes back the drag the law adds or saves.
        dv = (dv + (-DRAG_MS2 * (u.abs() - tk.u.abs()) - dv / 4.0) * dt).clamp(-1.5, 0.5);
    }
    judge.flush();
    Verdict { lat: judge.max_lat, cut: judge.max_cut, corners: corner_worst(&samples, &rb.corners) }
}

fn drive_ticks(run: &[&(String, Value)]) -> Vec<DriveTick> {
    let mut out: Vec<DriveTick> = Vec::new();
    for (_, v) in run {
        let ema = f(v, "yaw_dps").unwrap_or(0.0);
        let raw = out.last().map_or(ema, |p: &DriveTick| 2.0 * ema - p.ema);
        out.push(DriveTick {
            t: f(v, "t").unwrap_or(0.0),
            gs: f(v, "gs").unwrap_or(0.0),
            err: f(v, "err").unwrap_or(0.0),
            ema,
            raw,
            steer: f(v, "steer").unwrap_or(0.0),
            u: f(v, "br").unwrap_or(0.0) - f(v, "bl").unwrap_or(0.0),
            lat: f(v, "lat_m").unwrap_or(0.0),
            rem: f(v, "remaining_m").unwrap_or(0.0),
            look: f(v, "look_m").unwrap_or(0.0),
            turn: f(v, "turn_dps").unwrap_or(0.0),
        });
    }
    out
}

// ---------------------------------------------------------------------------
// The replay.
// ---------------------------------------------------------------------------

const SCALES: [f64; 3] = [0.7, 1.0, 1.3];

fn fmt3(v: [f64; 3]) -> String {
    format!("{:5.1} {:5.1} {:5.1}", v[0], v[1], v[2])
}

#[test]
#[ignore]
fn free_nose_replay() {
    let recs = load();
    println!("free-nose replay over {} records", recs.len());
    println!("worst lateral, m (the scorer's scale: off the line, or a vertex cut past its own 5 m); plant brake x0.7 x1.0 x1.3");

    // --- Taxi drives -----------------------------------------------------
    println!("\n== taxi drives (corners, backtracks, line-up drives) ==");
    println!("{:<24} {:>5} {:>6} {:>6} | {:>7} | {:>17} | {:>17} | corners >3 m flown/old/new", "sortie", "ac", "ticks", "fit_m", "flown", "old x0.7/1/1.3", "new x0.7/1/1.3");
    let states: Vec<(f64, f64, f64, f64)> = recs
        .iter()
        .filter(|r| kind(&r.1) == "state")
        .filter_map(|r| Some((f(&r.1, "t")?, f(&r.1, "lat")?, f(&r.1, "lon")?, f(&r.1, "hdg")?)))
        .collect();
    let mut all_corners = (0usize, 0usize, 0usize, 0usize);
    for run in runs(&recs, "autotaxi", "drive") {
        if run.len() < 40 {
            continue;
        }
        let ac = run[0].0.clone();
        let sortie = run[0].1.get("sortie").and_then(Value::as_str).unwrap_or("?").to_string();
        let mut ticks = drive_ticks(&run);
        // A planned turn onto the route rebuilds it (`join_turn`): the
        // route flown is the one after the last jump up in `remaining_m`.
        if let Some(j) = (1..ticks.len()).rev().find(|&k| ticks[k].rem > ticks[k - 1].rem + 5.0) {
            ticks.drain(..j);
        }
        if ticks.len() < 40 {
            continue;
        }
        let (t0, t1) = (ticks[0].t, ticks[ticks.len() - 1].t);
        let st: Vec<(f64, f64, f64, f64)> = states.iter().copied().filter(|s| s.0 >= t0 - 0.5 && s.0 <= t1 + 3.5).collect();
        let prof = crate::aircraft::load(&ac);
        let pace = crate::autotaxi::Pace::from_profile(prof.as_ref().and_then(|p| p.taxi.as_ref()));
        let Some((mut rb, _off)) = best_rebuild(&ticks, &st, pace) else {
            println!("{sortie:<24} {ac:>5.5}: no route");
            continue;
        };
        // The route planned again from the install's network where it gives
        // the logged lateral back better than the rebuild (the rebuild's
        // feet carry the heading's error into jogs a law can chase).
        let rebuilt_fit = lateral_fit(&ticks, &rb, &rb.route);
        let mut source = "rebuilt";
        if let Some((route, corners, fit)) = planned(&ticks, &rb, pace) {
            if fit.0 < rebuilt_fit.0.max(0.8) && fit.1 < 25.0 {
                rb.route = route;
                rb.corners = corners;
                rb.fit_rms = fit.0;
                source = "planned";
            } else if std::env::var("FNR_DEBUG").is_ok() {
                println!("  planned fit {:.2}/{:.1} vs rebuilt {:.2}/{:.1}", fit.0, fit.1, rebuilt_fit.0, rebuilt_fit.1);
            }
        }
        if source == "rebuilt" {
            rb.fit_rms = rebuilt_fit.0;
        }
        // Judged from the first tick on the line after the start (a join).
        let k0 = ticks.iter().position(|k| k.t - t0 >= 2.0 && k.lat.abs() <= 3.0).unwrap_or(0);
        let plant = plant_of(&ac);
        let io_ticks: Vec<(f64, f64, f64, f64, f64, f64)> =
            ticks.iter().zip(&rb.psi).map(|(k, p)| (k.t, k.gs / KT, k.u, k.steer, *p, k.raw)).collect();
        let w = fit_w(plant, ground_steer::IO_DELAY_S, &io_ticks);
        let flown = judge_flown(&ticks, k0, &rb);
        let mut old = [0.0; 3];
        let mut new = [0.0; 3];
        let mut c_old = 0;
        let mut c_new = 0;
        for (i, s) in SCALES.iter().enumerate() {
            let p = plant.scaled(*s, 1.0);
            let o = fly_drive(&ticks, k0, &rb, p, plant, w, TaxiLaw::Old);
            let n = fly_drive(&ticks, k0, &rb, p, plant, w, TaxiLaw::New);
            old[i] = o.scale();
            new[i] = n.scale();
            if *s == 1.0 {
                c_old = o.corners.iter().filter(|c| **c > 3.0).count();
                c_new = n.corners.iter().filter(|c| **c > 3.0).count();
            }
        }
        let c_flown = flown.corners.iter().filter(|c| **c > 3.0).count();
        all_corners.0 += rb.corners.len();
        all_corners.1 += c_flown;
        all_corners.2 += c_old;
        all_corners.3 += c_new;
        println!(
            "{sortie:<24} {:>5} {:>6} {:>6.2} | {:>7.1} | {} | {} | {}/{} corners: {c_flown}/{c_old}/{c_new}  (w {w:+.2}) {source} t0 {}",
            &ac[..5.min(ac.len())],
            ticks.len() - k0,
            rb.fit_rms,
            flown.scale(),
            fmt3(old),
            fmt3(new),
            rb.corners.len(),
            rb.corners.len(),
            t0 as i64,
        );
    }
    println!("corners in all: {}, over 3 m flown {}, old {}, new {}", all_corners.0, all_corners.1, all_corners.2, all_corners.3);

    // --- Line-up alignments ---------------------------------------------
    println!("\n== line-up alignments (the aligning pivot) ==");
    println!("{:<24} {:>5} | {:>28} | {:>28}", "sortie", "ac", "old: max|lat| end_err secs", "new: max|lat| end_err secs");
    for run in runs(&recs, "autotaxi", "line_up_align") {
        if run.len() < 5 {
            continue;
        }
        let ac = run[0].0.clone();
        let sortie = run[0].1.get("sortie").and_then(Value::as_str).unwrap_or("?").to_string();
        let t_end = f(&run[run.len() - 1].1, "t").unwrap_or(0.0);
        let course = recs
            .iter()
            .find(|r| event(&r.1) == "line_up_aligned" && f(&r.1, "t").is_some_and(|t| t >= t_end && t - t_end < 5.0))
            .and_then(|r| f(&r.1, "course"));
        let Some(course) = course else { continue };
        let spare = recs
            .iter()
            .rev()
            .find(|r| event(&r.1) == "line_up_align_budget" && f(&r.1, "t").is_some_and(|t| t <= f(&run[0].1, "t").unwrap_or(0.0) + 1.0))
            .and_then(|r| f(&r.1, "spare_m"));
        let first = &run[0].1;
        let (err0, lat0, gs0, yaw0) = (f(first, "err").unwrap_or(0.0), f(first, "lat_m"), f(first, "gs").unwrap_or(0.0), f(first, "yaw_dps").unwrap_or(0.0));
        let plant = plant_of(&ac);
        let mut rows = Vec::new();
        for law in [TaxiLaw::Old, TaxiLaw::New] {
            let mut judge = crate::autotaxi::LineUpAlign::with_spare(course, spare);
            if law == TaxiLaw::New {
                judge = judge.with_ground(plant);
            }
            let mut sim = Sim::new(plant, 0.0, ground_steer::IO_DELAY_S, course - err0, course - err0, yaw0);
            let (mut y, mut v, mut t, mut rolled) = (lat0.unwrap_or(0.0), gs0 / KT, 0.0, 0.0);
            let mut creep;
            let mut worst = y.abs();
            let mut outcome = String::from("time");
            while t < 90.0 {
                let step = judge.step_on_line(t, sim.psi.rem_euclid(360.0), v * KT, sim.r, rolled, lat0.map(|_| y));
                let (bl, br) = match step {
                    crate::autotaxi::AlignStep::Align { bl, br, creep: c } => {
                        creep = c;
                        (bl, br)
                    }
                    crate::autotaxi::AlignStep::Done { err_deg } => {
                        outcome = format!("{err_deg:+.1} {t:4.0}s ok");
                        break;
                    }
                    crate::autotaxi::AlignStep::GiveUp { err_deg, why } => {
                        outcome = format!("{err_deg:+.1} {t:4.0}s {why}");
                        break;
                    }
                };
                let dt = 0.35;
                let target = if creep { judge.creep_kts / KT } else { 0.0 };
                sim.run(br - bl, 0.0, v, dt);
                let brg = crate::angles::wrap180(sim.psi - course).to_radians();
                y += v * brg.sin() * dt;
                rolled += v * dt;
                // The governor: a creep's pull, its brake over the creep.
                v = (v + ((target - v).clamp(-1.0, 0.3) - DRAG_MS2 * (bl + br)) * dt).max(0.0);
                worst = worst.max(y.abs());
                t += dt;
            }
            rows.push(format!("{worst:5.2} m {outcome:>18}"));
        }
        println!("{sortie:<24} {:>5} | {:>28} | {:>28}", &ac[..5.min(ac.len())], rows[0], rows[1]);
    }

    // --- Roll starts ---------------------------------------------------
    println!("\n== roll starts (to 25 kt) ==");
    println!("{:<24} {:>5} | {:>7} | {:>17} | {:>17}", "sortie", "ac", "flown", "old x0.7/1/1.3", "new x0.7/1/1.3");
    for run in runs(&recs, "autotakeoff", "roll") {
        let ticks: Vec<&Value> = run.iter().map(|r| &r.1).take_while(|v| f(v, "gs").unwrap_or(0.0) <= 25.0).collect();
        if ticks.len() < 8 {
            continue;
        }
        let ac = run[0].0.clone();
        let sortie = ticks[0].get("sortie").and_then(Value::as_str).unwrap_or("?").to_string();
        let mut courses: Vec<f64> = ticks.iter().filter_map(|v| Some(f(v, "tgt")? + f(v, "cut")?)).collect();
        courses.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let course = courses[courses.len() / 2];
        let flown = ticks.iter().filter_map(|v| f(v, "xtk_m")).map(f64::abs).fold(0.0, f64::max);
        let plant = plant_of(&ac);
        let io_ticks: Vec<(f64, f64, f64, f64, f64, f64)> = ticks
            .iter()
            .map(|v| {
                (
                    f(v, "t").unwrap_or(0.0),
                    f(v, "gs").unwrap_or(0.0) / KT,
                    f(v, "br").unwrap_or(0.0) - f(v, "bl").unwrap_or(0.0),
                    f(v, "steer").unwrap_or(0.0),
                    f(v, "psi").unwrap_or(0.0),
                    f(v, "r").unwrap_or(0.0),
                )
            })
            .collect();
        let w = fit_w(plant, ground_steer::IO_DELAY_S, &io_ticks);
        let mut res = [[0.0; 3]; 2];
        for (li, law) in [TaxiLaw::Old, TaxiLaw::New].into_iter().enumerate() {
            for (si, s) in SCALES.iter().enumerate() {
                let p = plant.scaled(*s, 1.0);
                let first = ticks[0];
                let psi0 = f(first, "psi").unwrap_or(course);
                let mut sim = Sim::new(p, w, ground_steer::IO_DELAY_S, psi0, psi0, f(first, "r").unwrap_or(0.0));
                let mut x = f(first, "xtk_m").unwrap_or(0.0);
                let mut rate = crate::autotakeoff::XtkRate::default();
                let mut prev_x: Option<f64> = None;
                let mut align = crate::autotakeoff::RollAlign::default();
                let mut hold = true;
                let mut law_g = GroundSteer::new(plant);
                let mut worst = x.abs();
                for k in 0..ticks.len() - 1 {
                    let v = ticks[k];
                    let dt = (f(ticks[k + 1], "t").unwrap_or(0.0) - f(v, "t").unwrap_or(0.0)).clamp(0.05, 1.5);
                    let gs = f(v, "gs").unwrap_or(0.0);
                    let steer = f(v, "steer").unwrap_or(0.0);
                    let r = rate.update(prev_x.map_or(0.0, |q| (x - q) / dt), dt);
                    prev_x = Some(x);
                    let tgt = crate::autotakeoff::roll_target_heading(course, x, r, gs);
                    let err = crate::angles::wrap180(tgt - sim.psi);
                    if hold {
                        hold = matches!(align.step(err, sim.r, gs, dt), crate::autotakeoff::RollAlignState::Hold);
                    }
                    let (bl, br) = match law {
                        TaxiLaw::Old => {
                            let split = crate::autotakeoff::roll_brake_split_aligning(steer, gs, err, sim.r);
                            if hold {
                                align.trimmed(split, err, sim.r, gs, dt)
                            } else {
                                split
                            }
                        }
                        TaxiLaw::New => crate::autotakeoff::free_roll_brakes(&mut law_g, err, Yaw::Now(sim.r), steer, gs, dt, hold),
                    };
                    sim.run(br - bl, steer, gs / KT, dt);
                    let brg = crate::angles::wrap180(sim.trk - course).to_radians();
                    x += gs / KT * brg.sin() * dt;
                    worst = worst.max(x.abs());
                }
                res[li][si] = worst;
            }
        }
        println!("{sortie:<24} {:>5} | {:>7.1} | {} | {}  (w {w:+.2})", &ac[..5.min(ac.len())], flown, fmt3(res[0]), fmt3(res[1]));
    }

    // --- Rollouts ------------------------------------------------------
    println!("\n== rollouts (the wheels to 5 kt) ==");
    println!("{:<24} {:>5} | {:>7} | {:>17} | {:>17}", "sortie", "ac", "flown", "old x0.7/1/1.3", "new x0.7/1/1.3");
    for run in runs(&recs, "autoland", "rollout") {
        let ticks: Vec<&Value> = run
            .iter()
            .map(|r| &r.1)
            .skip_while(|v| v.get("ground").and_then(Value::as_bool) == Some(false))
            .take_while(|v| f(v, "gs").unwrap_or(0.0) >= 5.0)
            .filter(|v| f(v, "psi").is_some() && f(v, "xtk_m").is_some())
            .collect();
        if ticks.len() < 8 {
            continue;
        }
        let ac = run[0].0.clone();
        let sortie = ticks[0].get("sortie").and_then(Value::as_str).unwrap_or("?").to_string();
        // The course: the target less the roll law's cut, the heading plus its error.
        let mut courses: Vec<f64> = ticks
            .iter()
            .filter_map(|v| {
                let tgt0 = crate::autotakeoff::roll_target_heading(0.0, f(v, "xtk_m")?, f(v, "xtk_rate").unwrap_or(0.0), f(v, "gs")?);
                Some((f(v, "psi")? + f(v, "err")? - tgt0).rem_euclid(360.0))
            })
            .collect();
        courses.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let course = courses[courses.len() / 2];
        let flown = ticks.iter().filter_map(|v| f(v, "xtk_m")).map(f64::abs).fold(0.0, f64::max);
        let plant = plant_of(&ac);
        // The stopping brake asked: the channel and the inside wheel.
        let stop = |v: &Value| {
            let (sym, bl, br) = (f(v, "brake").unwrap_or(0.0), f(v, "bl").unwrap_or(0.0), f(v, "br").unwrap_or(0.0));
            let inside = bl.max(br);
            if inside > 0.0 {
                (sym + inside) / 2.0
            } else {
                sym
            }
        };
        let applied = |v: &Value| {
            let (sym, bl, br) = (f(v, "brake").unwrap_or(0.0), f(v, "bl").unwrap_or(0.0), f(v, "br").unwrap_or(0.0));
            sym.max(br) - sym.max(bl)
        };
        let io_ticks: Vec<(f64, f64, f64, f64, f64, f64)> = {
            let mut out = Vec::new();
            for (k, v) in ticks.iter().enumerate() {
                let r = if k == 0 { 0.0 } else { crate::angles::wrap180(f(v, "psi").unwrap() - f(ticks[k - 1], "psi").unwrap()) / f(v, "dt").unwrap_or(0.4).max(0.05) };
                out.push((f(v, "t").unwrap_or(0.0), f(v, "gs").unwrap_or(0.0) / KT, applied(v), 0.0, f(v, "psi").unwrap(), r));
            }
            out
        };
        let w = fit_w(plant, ground_steer::ROLLOUT_IO_DELAY_S, &io_ticks);
        let sr22 = ac == "cirrus_sr22";
        let mut res = [[0.0; 3]; 2];
        for (li, law) in [TaxiLaw::Old, TaxiLaw::New].into_iter().enumerate() {
            for (si, s) in SCALES.iter().enumerate() {
                let p = plant.scaled(*s, 1.0);
                let first = ticks[0];
                let psi0 = f(first, "psi").unwrap();
                let trk0 = f(first, "trk").unwrap_or(psi0);
                let mut sim = Sim::new(p, w, ground_steer::ROLLOUT_IO_DELAY_S, psi0, trk0, io_ticks.get(1).map_or(0.0, |x| x.5));
                let mut x = f(first, "xtk_m").unwrap();
                let mut rate = crate::autotakeoff::XtkRate::default();
                let mut prev_x: Option<f64> = None;
                let mut shared = crate::autoland::RolloutBrakeSteer::default();
                let mut split = legacy::Sr22Split::default();
                let mut free = crate::autoland::RolloutBrakeSteer::free(plant);
                let mut roll_law = crate::autotakeoff::RollSteer::default();
                let mut worst = x.abs();
                for k in 0..ticks.len() - 1 {
                    let v = ticks[k];
                    let dt = f(v, "dt").unwrap_or(0.45).clamp(0.05, 1.5);
                    let gs = f(v, "gs").unwrap_or(0.0);
                    let r = rate.update(prev_x.map_or(0.0, |q| (x - q) / dt), dt);
                    prev_x = Some(x);
                    let tgt = crate::autotakeoff::roll_target_heading(course, x, r, gs);
                    let err = crate::angles::wrap180(tgt - sim.psi);
                    let steer = roll_law.step(err, dt, gs, 0.0);
                    let brake = stop(v);
                    let diff = match law {
                        TaxiLaw::Old if sr22 => {
                            let d = split.step(err, dt, gs);
                            let b = crate::ground_steer::stop_wheels(brake, d);
                            b.2.max(b.0) - b.1.max(b.0)
                        }
                        TaxiLaw::Old => {
                            let b = shared.step(err, dt, gs, brake);
                            b.sym.max(b.right) - b.sym.max(b.left)
                        }
                        TaxiLaw::New => {
                            let b = free.step_with(err, Some(sim.psi), Some(sim.trk), steer, dt, gs, brake);
                            b.sym.max(b.right) - b.sym.max(b.left)
                        }
                    };
                    sim.run(diff, steer, gs / KT, dt);
                    let brg = crate::angles::wrap180(sim.trk - course).to_radians();
                    x += gs / KT * brg.sin() * dt;
                    worst = worst.max(x.abs());
                }
                res[li][si] = worst;
            }
        }
        println!("{sortie:<24} {:>5} | {:>7.1} | {} | {}  (w {w:+.2})", &ac[..5.min(ac.len())], flown, fmt3(res[0]), fmt3(res[1]));
    }
}
