//! The sortie scorecard: blackbox ticks in, one `stage_result` per stage out.
//!
//! Unattended testing needs a verdict per stage, not a pilot watching (asked
//! 2026-09-25). Pure: [`Scorer::feed`] takes each blackbox record in order and
//! returns the records it concludes, so the SAME code runs live (a blackbox tap,
//! `stage_result` written beside the ticks) and offline (`xvl debrief` over any
//! file). Every pass line is a number with a limit, because the complaints that
//! started it ("jerky braking", "lined up at an angle", "it wobbles on the
//! centreline") were seen and never measured. The limits are first guesses,
//! named here so they can be argued with.

use serde_json::{json, Value};

/// Taxi: most off the centreline.
pub const TAXI_MAX_LAT_M: f64 = 3.0;
/// Taxi: fastest the brake command may move (ratio per second) outside a stop.
pub const TAXI_MAX_BRAKE_RATE: f64 = 1.5;
/// A takeoff starts aligned or it is a line-up failure.
pub const LINEUP_MAX_ERR_DEG: f64 = 3.0;
pub const LINEUP_MAX_XTK_M: f64 = 2.0;
/// Takeoff roll: most off the centreline.
pub const ROLL_MAX_XTK_M: f64 = 5.0;
/// Touchdown: firmer than this fails; crabbed more than this fails.
pub const TOUCHDOWN_MAX_SINK_FPM: f64 = 400.0;
pub const TOUCHDOWN_MAX_DRIFT_DEG: f64 = 5.0;
/// Rollout: most off the centreline.
pub const ROLLOUT_MAX_XTK_M: f64 = 5.0;
/// A stream quiet this long has ended.
const GAP_S: f64 = 5.0;

fn f(r: &Value, k: &str) -> Option<f64> {
    r.get(k).and_then(Value::as_f64)
}

fn s<'a>(r: &'a Value, k: &str) -> &'a str {
    r.get(k).and_then(Value::as_str).unwrap_or("")
}

fn r1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

fn r2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

/// One taxi (or line-up) drive, from `autotaxi` `drive` ticks.
#[derive(Debug, Default, Clone)]
struct Taxi {
    line_up: bool,
    t0: f64,
    t_last: f64,
    ticks: usize,
    max_gs: f64,
    max_lat: f64,
    brake_rate: f64,
    brake_reversals: usize,
    yaw_swings: usize,
    last: Option<(f64, f64, f64, f64)>, // t, bl, br, yaw
    last_err: f64,
    last_lat: f64,
    ended_by: String,
}

/// One takeoff roll, from `autotakeoff` `roll` ticks.
#[derive(Debug, Default, Clone)]
struct Roll {
    t0: f64,
    start_err: Option<f64>,
    start_xtk: Option<f64>,
    max_xtk: f64,
    max_err_fast: f64,
    max_brake: f64,
    last_xtk: f64,
    last_ias: f64,
    ticks: usize,
    t_last: f64,
}

/// The landing, from `state` (touchdown) and `autoland` `rollout` ticks.
#[derive(Debug, Default, Clone)]
struct Landing {
    t_td: f64,
    sink: f64,
    drift: f64,
    max_xtk: Option<f64>,
    t_last: f64,
}

#[derive(Debug, Default)]
pub struct Scorer {
    taxi: Option<Taxi>,
    line_up_next: bool,
    roll: Option<Roll>,
    landing: Option<Landing>,
    /// Recent airborne state samples (t, vsi, hdg, track) for the touchdown.
    air: Vec<(f64, f64, f64, Option<f64>)>,
    airborne_since: Option<f64>,
    interventions: usize,
    results: Vec<(String, bool)>,
}

fn angle(a: f64) -> f64 {
    (a + 540.0).rem_euclid(360.0) - 180.0
}

impl Scorer {
    pub fn new() -> Self {
        Self::default()
    }

    /// One record in, the conclusions it completes out (`stage_result`,
    /// `sortie_result`). Never fed its own output.
    pub fn feed(&mut self, r: &Value) -> Vec<(String, Value)> {
        let mut out = Vec::new();
        let t = f(r, "t").unwrap_or(0.0);
        // A stream that went quiet has ended, whatever record says so.
        if self.taxi.as_ref().is_some_and(|x| t - x.t_last > GAP_S) {
            self.close_taxi("gap", &mut out);
        }
        if self.landing.as_ref().is_some_and(|x| t - x.t_last > GAP_S) {
            self.close_landing(&mut out);
        }
        let (kind, event) = (s(r, "kind"), s(r, "event"));
        match (kind, event) {
            ("autotaxi", "line_up_path") => self.line_up_next = true,
            ("autotaxi", "drive") => self.taxi_tick(r, t),
            ("autotaxi", "holding_point" | "arrived" | "stopped" | "disengaged" | "gave_up") => {
                self.close_taxi(event, &mut out)
            }
            ("autotakeoff", "roll") => self.roll_tick(r, t),
            ("autotakeoff", "climb" | "climb_manual") => self.close_roll(None, &mut out),
            ("autotakeoff", e)
                if self.roll.is_some()
                    && !matches!(e, "takeoff_config" | "engaged" | "transponder_alt" | "") =>
            {
                self.close_roll(Some(e), &mut out)
            }
            ("autoland", "rollout") => {
                if let Some(l) = self.landing.as_mut() {
                    if let Some(x) = f(r, "xtk_m") {
                        l.max_xtk = Some(l.max_xtk.unwrap_or(0.0).max(x.abs()));
                    }
                    l.t_last = t;
                }
            }
            ("state", _) => self.state_tick(r, t, &mut out),
            ("full_auto", _) if s(r, "stage") == "give_up" => {
                if self.roll.is_some() {
                    self.close_roll(Some("give_up"), &mut out);
                }
                self.close_taxi("give_up", &mut out);
            }
            ("intervention", _) => self.interventions += 1,
            ("sortie", "begin") => {
                self.results.clear();
                self.interventions = 0;
            }
            ("sortie", "end") => {
                self.close_taxi("sortie_end", &mut out);
                self.close_landing(&mut out);
                let failed: Vec<&str> =
                    self.results.iter().filter(|(_, ok)| !ok).map(|(n, _)| n.as_str()).collect();
                out.push((
                    "sortie_result".into(),
                    json!({
                        "outcome": s(r, "outcome"),
                        "stages": self.results.len(),
                        "failed": failed,
                        "interventions": self.interventions,
                        "unattended": self.interventions == 0,
                        "pass": failed.is_empty() && self.interventions == 0
                            && s(r, "outcome") != "gave_up",
                    }),
                ));
            }
            _ => {}
        }
        out
    }

    fn taxi_tick(&mut self, r: &Value, t: f64) {
        let line_up = std::mem::take(&mut self.line_up_next);
        let x = self.taxi.get_or_insert_with(|| Taxi { t0: t, ..Taxi::default() });
        x.line_up |= line_up;
        x.ticks += 1;
        x.t_last = t;
        let (gs, lat) = (f(r, "gs").unwrap_or(0.0), f(r, "lat_m").unwrap_or(0.0));
        let (bl, br, yaw) = (
            f(r, "bl").unwrap_or(0.0),
            f(r, "br").unwrap_or(0.0),
            f(r, "yaw_dps").unwrap_or(0.0),
        );
        x.max_gs = x.max_gs.max(gs);
        x.max_lat = x.max_lat.max(lat.abs());
        x.last_err = f(r, "err").unwrap_or(0.0);
        x.last_lat = lat;
        if let Some((t1, bl1, br1, yaw1)) = x.last {
            let dt = t - t1;
            // A stop is allowed to squeeze; judged only while still moving.
            if dt > 0.05 && gs > 2.0 {
                x.brake_rate = x.brake_rate.max(((bl - bl1).abs().max((br - br1).abs())) / dt);
            }
            let (d1, d) = (bl1 - br1, bl - br);
            if d1.abs() > 0.05 && d.abs() > 0.05 && d1.signum() != d.signum() {
                x.brake_reversals += 1;
            }
            if yaw1.abs() > 2.0 && yaw.abs() > 2.0 && yaw1.signum() != yaw.signum() {
                x.yaw_swings += 1;
            }
        }
        x.last = Some((t, bl, br, yaw));
    }

    fn close_taxi(&mut self, by: &str, out: &mut Vec<(String, Value)>) {
        let Some(mut x) = self.taxi.take() else { return };
        if x.ticks < 5 {
            return;
        }
        x.ended_by = by.to_string();
        let mut fails = Vec::new();
        if x.max_lat > TAXI_MAX_LAT_M {
            fails.push(format!("{:.1} m off the line", x.max_lat));
        }
        if x.brake_rate > TAXI_MAX_BRAKE_RATE {
            fails.push(format!("brake moved {:.1}/s", x.brake_rate));
        }
        let stage = if x.line_up { "line_up" } else { "taxi" };
        self.push(stage, fails, json!({
            "secs": r1(x.t_last - x.t0), "max_gs": r1(x.max_gs), "max_lat_m": r1(x.max_lat),
            "brake_rate": r2(x.brake_rate), "brake_reversals": x.brake_reversals,
            "yaw_swings": x.yaw_swings, "end_err_deg": r1(x.last_err),
            "end_lat_m": r1(x.last_lat), "ended_by": x.ended_by,
        }), out);
    }

    fn roll_tick(&mut self, r: &Value, t: f64) {
        let x = self.roll.get_or_insert_with(|| Roll { t0: t, ..Roll::default() });
        x.ticks += 1;
        x.t_last = t;
        let (err, xtk, gs) = (f(r, "err"), f(r, "xtk_m"), f(r, "gs").unwrap_or(0.0));
        if x.start_err.is_none() {
            x.start_err = err;
            x.start_xtk = xtk;
        }
        if let Some(v) = xtk {
            x.max_xtk = x.max_xtk.max(v.abs());
            x.last_xtk = v;
        }
        if gs > 20.0 {
            x.max_err_fast = x.max_err_fast.max(err.unwrap_or(0.0).abs());
        }
        x.max_brake = x.max_brake.max(f(r, "bl").unwrap_or(0.0)).max(f(r, "br").unwrap_or(0.0));
        x.last_ias = f(r, "ias").unwrap_or(0.0);
    }

    fn close_roll(&mut self, ended: Option<&str>, out: &mut Vec<(String, Value)>) {
        let Some(x) = self.roll.take() else { return };
        let (e0, x0) = (x.start_err.unwrap_or(0.0).abs(), x.start_xtk.unwrap_or(0.0).abs());
        // The line-up is judged where it hands over: the roll's first tick.
        let mut lu = Vec::new();
        if e0 > LINEUP_MAX_ERR_DEG {
            lu.push(format!("{e0:.0} degrees off the runway"));
        }
        if x0 > LINEUP_MAX_XTK_M {
            lu.push(format!("{x0:.1} m off the centreline"));
        }
        self.push("alignment", lu, json!({ "err_deg": r1(e0), "xtk_m": r1(x0) }), out);
        let mut fails = Vec::new();
        if let Some(e) = ended {
            fails.push(format!("ended by {e}"));
        }
        if x.max_xtk > ROLL_MAX_XTK_M {
            fails.push(format!("{:.1} m off the centreline", x.max_xtk));
        }
        self.push("takeoff", fails, json!({
            "ticks": x.ticks, "max_xtk_m": r1(x.max_xtk), "liftoff_xtk_m": r1(x.last_xtk),
            "max_err_fast_deg": r1(x.max_err_fast), "max_brake": r2(x.max_brake),
            "last_ias": r1(x.last_ias), "ended_by": ended.unwrap_or("climb"),
            "secs": r1(x.t_last - x.t0),
        }), out);
    }

    fn state_tick(&mut self, r: &Value, t: f64, out: &mut Vec<(String, Value)>) {
        let ground = r.get("ground").and_then(Value::as_bool).unwrap_or(true);
        let crashed = r.get("crashed").and_then(Value::as_bool).unwrap_or(false);
        if !ground {
            self.airborne_since.get_or_insert(t);
            self.air.push((
                t,
                f(r, "vsi").unwrap_or(0.0),
                f(r, "hdg").unwrap_or(0.0),
                f(r, "track"),
            ));
            self.air.retain(|(ta, ..)| t - ta <= 3.0);
            return;
        }
        let Some(since) = self.airborne_since.take() else { return };
        // A hop is not a flight: 30 s airborne before a touchdown counts.
        if t - since < 30.0 {
            self.air.clear();
            return;
        }
        let sink = self
            .air
            .iter()
            .map(|(_, v, ..)| *v)
            .chain(f(r, "vsi"))
            .fold(0.0_f64, f64::min)
            .abs();
        let drift = self
            .air
            .last()
            .and_then(|(_, _, h, tr)| tr.map(|tr| angle(h - tr).abs()))
            .unwrap_or(0.0);
        self.air.clear();
        self.close_landing(out);
        self.landing = Some(Landing { t_td: t, sink, drift, max_xtk: None, t_last: t });
        if crashed {
            self.close_landing(out);
        }
    }

    fn close_landing(&mut self, out: &mut Vec<(String, Value)>) {
        let Some(l) = self.landing.take() else { return };
        let mut fails = Vec::new();
        if l.sink > TOUCHDOWN_MAX_SINK_FPM {
            fails.push(format!("touched down at {:.0} fpm", l.sink));
        }
        if l.drift > TOUCHDOWN_MAX_DRIFT_DEG {
            fails.push(format!("{:.0} degrees of drift", l.drift));
        }
        if l.max_xtk.is_some_and(|x| x > ROLLOUT_MAX_XTK_M) {
            fails.push(format!("{:.1} m off the centreline rolling out", l.max_xtk.unwrap_or(0.0)));
        }
        self.push("landing", fails, json!({
            "sink_fpm": l.sink.round(), "drift_deg": r1(l.drift),
            "rollout_max_xtk_m": l.max_xtk.map(r1), "t_touchdown": l.t_td,
        }), out);
    }

    fn push(&mut self, stage: &str, fails: Vec<String>, mut metrics: Value, out: &mut Vec<(String, Value)>) {
        let ok = fails.is_empty();
        if let Some(o) = metrics.as_object_mut() {
            o.insert("stage".into(), stage.into());
            o.insert("pass".into(), ok.into());
            o.insert("why".into(), fails.into());
        }
        self.results.push((stage.to_string(), ok));
        out.push(("stage_result".into(), metrics));
    }
}

static LIVE: std::sync::Mutex<Option<Scorer>> = std::sync::Mutex::new(None);

/// The blackbox tap: every record through the live scorer.
pub fn tap(r: &Value) -> Vec<(String, Value)> {
    let mut g = LIVE.lock().unwrap_or_else(|e| e.into_inner());
    g.get_or_insert_with(Scorer::new).feed(r)
}

/// Score a recorded stream: every conclusion with its time, in order.
pub fn score_all<'a>(records: impl Iterator<Item = &'a Value>) -> Vec<Value> {
    let mut sc = Scorer::new();
    let mut out = Vec::new();
    for r in records {
        // Recorded conclusions are re-derived, never trusted twice.
        if matches!(s(r, "kind"), "stage_result" | "sortie_result") {
            continue;
        }
        for (k, mut v) in sc.feed(r) {
            if let Some(o) = v.as_object_mut() {
                o.insert("kind".into(), k.into());
                o.insert("t".into(), r.get("t").cloned().unwrap_or(Value::Null));
            }
            out.push(v);
        }
    }
    out
}

/// One line per conclusion for the debrief.
pub fn line(v: &Value) -> String {
    let why: Vec<&str> = v
        .get("why")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    if s(v, "kind") == "sortie_result" {
        return format!(
            "SORTIE {}: {} stages, failed {:?}, {} interventions, outcome {}",
            if v.get("pass").and_then(Value::as_bool) == Some(true) { "PASS" } else { "FAIL" },
            v.get("stages").and_then(Value::as_u64).unwrap_or(0),
            v.get("failed").cloned().unwrap_or_default(),
            v.get("interventions").and_then(Value::as_u64).unwrap_or(0),
            s(v, "outcome"),
        );
    }
    format!(
        "{:<9} {}{}",
        s(v, "stage"),
        if v.get("pass").and_then(Value::as_bool) == Some(true) { "pass" } else { "FAIL" },
        if why.is_empty() { String::new() } else { format!(": {}", why.join(", ")) }
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn drive(t: f64, gs: f64, lat: f64, bl: f64, br: f64, yaw: f64, err: f64) -> Value {
        json!({"t": t, "kind": "autotaxi", "event": "drive", "gs": gs, "lat_m": lat,
               "bl": bl, "br": br, "yaw_dps": yaw, "err": err})
    }

    fn roll(t: f64, gs: f64, err: f64, xtk: f64) -> Value {
        json!({"t": t, "kind": "autotakeoff", "event": "roll", "gs": gs, "err": err,
               "xtk_m": xtk, "bl": 0.0, "br": 0.0, "ias": gs})
    }

    /// LIME 28, 2026-09-25: lined up 21 degrees off, the roll swung further,
    /// the pilot's pedals stood it down. Every piece of that is a FAIL.
    #[test]
    fn the_lime_departure_fails_where_it_failed() {
        let mut recs = vec![json!({"t": 0.0, "kind": "sortie", "event": "begin"})];
        recs.push(json!({"t": 1.0, "kind": "autotaxi", "event": "line_up_path"}));
        for i in 0..20 {
            let t = 2.0 + i as f64 * 0.25;
            recs.push(drive(t, 6.0, -0.1 * i as f64, 0.0, 0.01, -2.0 - 0.1 * i as f64, i as f64));
        }
        recs.push(json!({"t": 8.0, "kind": "autotaxi", "event": "holding_point"}));
        for i in 0..16 {
            recs.push(roll(15.0 + i as f64 * 0.25, 1.0 + i as f64 * 0.3, 21.0 + 0.5 * i as f64, -1.4));
        }
        recs.push(json!({"t": 19.5, "kind": "autotakeoff", "event": "pedal_yield"}));
        recs.push(json!({"t": 19.6, "kind": "intervention", "what": "pedal_yield"}));
        recs.push(json!({"t": 20.0, "kind": "full_auto", "stage": "give_up"}));
        recs.push(json!({"t": 21.0, "kind": "sortie", "event": "end", "outcome": "gave_up"}));
        let out = score_all(recs.iter());
        let lines: Vec<String> = out.iter().map(line).collect();
        println!("{}", lines.join("\n"));
        let stage = |n: &str| out.iter().find(|v| s(v, "stage") == n).cloned().unwrap();
        assert_eq!(stage("line_up")["pass"], true, "the drive itself stayed on its line");
        assert_eq!(stage("alignment")["pass"], false);
        assert_eq!(stage("takeoff")["pass"], false);
        assert_eq!(stage("takeoff")["ended_by"], "pedal_yield");
        let sortie = out.last().unwrap();
        assert_eq!(sortie["pass"], false);
        assert_eq!(sortie["interventions"], 1);
    }

    #[test]
    fn jerky_brakes_and_a_wander_fail_the_taxi() {
        let mut recs = Vec::new();
        for i in 0..20 {
            let b = if i % 2 == 0 { 0.0 } else { 0.6 };
            recs.push(drive(i as f64 * 0.2, 8.0, if i == 10 { 4.0 } else { 0.5 }, b, 0.0, 0.0, 0.0));
        }
        recs.push(json!({"t": 10.0, "kind": "autotaxi", "event": "holding_point"}));
        let out = score_all(recs.iter());
        assert_eq!(out[0]["stage"], "taxi");
        assert_eq!(out[0]["pass"], false);
        assert_eq!(out[0]["why"].as_array().unwrap().len(), 2, "{}", out[0]);
    }

    #[test]
    fn a_touchdown_is_judged_on_its_sink_and_drift() {
        let mut recs = Vec::new();
        for i in 0..40 {
            recs.push(json!({"t": i as f64, "kind": "state", "ground": false,
                "vsi": if i > 37 { -650.0 } else { -500.0 }, "hdg": 90.0, "track": 88.0}));
        }
        recs.push(json!({"t": 40.0, "kind": "state", "ground": true, "vsi": -100.0, "hdg": 90.0, "track": 90.0}));
        recs.push(json!({"t": 41.0, "kind": "autoland", "event": "rollout", "xtk_m": 1.2}));
        recs.push(json!({"t": 50.0, "kind": "state", "ground": true, "vsi": 0.0, "hdg": 90.0, "track": 90.0}));
        let out = score_all(recs.iter());
        let l = out.iter().find(|v| v["stage"] == "landing").unwrap();
        assert_eq!(l["pass"], false, "{l}");
        assert_eq!(l["sink_fpm"], 650.0);
        assert_eq!(l["rollout_max_xtk_m"], 1.2);
    }
}
