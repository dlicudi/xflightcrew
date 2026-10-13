//! The 737's scripted start, WATCHED: milestone calls while the aircraft's
//! own sequence runs, and an abort caught in half a minute.
//!
//! The stock 737-800's xlua (`B738.Z.autostart.lua`) replaces
//! `sim/operation/auto_start` with its own two-minute sequence
//! (`jet_start::ScriptedAutoStart`), and for those two minutes the copilot
//! used to say nothing. Live on the parked aircraft (2026-10-06, 2 s
//! samples from the command): in progress at 0 s; APU N1 0 to 100 from 26
//! to 66 s; APU bleed on at 66 s; starter 2 at 78 s with N2 rising about 2%
//! a second; fuel in on engine 2 at 92 s (N2 about 29); starter 2 cut out
//! and starter 1 in at 99 s (about 56% N2); engine 1 fuel in at 113 s; both
//! starters off at 120 s; the sequence's flag cleared at 125 s; both N2 63.7
//! by 139 s.
//!
//! The FAILURE (2026-10-06 09:11Z): starter 2 engaged, N2 never passed 25%,
//! the script's own step timed out after 25 s and ABORTED, and never cleared
//! `auto_start_in_progress`, so every later `auto_start` was ignored until
//! the aircraft was reloaded. Full auto waited five minutes for "The engine
//! didn't catch.", and a retry answered "The start sequence is already
//! running." [`Watcher`] is the pure judge of both: fed one [`Sample`] a
//! tick, it says the milestones and calls the abort (a starter that does not
//! spool N2, or a sequence latched with nothing moving). [`run`] feeds it
//! from the sim on the start's own thread; full auto reads its verdict
//! ([`state`]) instead of waiting out the class's deadline.

use std::sync::Mutex;
use std::time::{Duration, Instant};

use serde_json::json;
use sim::xplane::XPlane;

use crate::crew_pace::StartCalls;
use crate::voice::{Priority, Voice};

/// The aircraft's own "a start is running" flag (the 737's script sets it
/// and, on an abort, never clears it).
pub const AUTO_START_IN_PROGRESS: &str = "sim/flightmodel2/misc/auto_start_in_progress";
const APU_N1: &str = crate::datarefs::APU_N1_PERCENT;
const APU_BLEED: &str = crate::datarefs::APU_BLEED;
const STARTER: &str = crate::datarefs::STARTER_IS_RUNNING;
const N2: &str = crate::datarefs::N2_PERCENT;
const RUNNING: &str = crate::datarefs::ENGINE_RUNNING;

/// Starter engaged this long with N2 still under [`LIGHT_N2`]: the starter
/// is not turning it (the script's own step gives up at 25 s).
pub const NO_SPOOL_SECS: f64 = 30.0;
/// The N2 a starter has to reach before fuel goes in.
pub const LIGHT_N2: f64 = 25.0;
/// Starter engaged, no light-off, and N2 has not risen a percent in this
/// long: stalled. A healthy start rises about 2% a second.
pub const STALL_SECS: f64 = 20.0;
/// The N2 a starter cuts out at or above on a good start (56% live); a
/// starter dropping below it with the engine not lit is the start ending.
pub const CUTOUT_MIN_N2: f64 = 40.0;
/// Self-sustaining: running, starter out, N2 at or above this (idle 63.7).
pub const IDLE_N2: f64 = 50.0;
/// In progress with NOTHING moving for this long: latched from an abort.
/// A healthy start's quietest stretch is its first 26 s (battery and fire
/// tests before the APU turns).
pub const LATCH_SECS: f64 = 45.0;
/// The flag cleared with the engines not running for this long: the
/// sequence ended without them.
pub const ENDED_SECS: f64 = 15.0;
/// No flag and nothing moving this long after the command: it never began.
pub const NEVER_BEGAN_SECS: f64 = 20.0;
/// How long [`run`] watches before letting the class deadline decide.
pub const MAX_WATCH_SECS: f64 = 300.0;
const POLL_MS: u64 = 1_000;

/// One tick's readings. The APU's are optional (a failed read of those is
/// not a reason to skip the engines); the rest are the tick or nothing.
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    pub t_s: f64,
    pub in_progress: bool,
    pub apu_n1: Option<f64>,
    pub apu_bleed: Option<bool>,
    pub starter: Vec<bool>,
    pub n2: Vec<f64>,
    pub running: Vec<bool>,
}

impl Sample {
    fn starter(&self, i: usize) -> bool {
        self.starter.get(i).copied().unwrap_or(false)
    }
    fn running(&self, i: usize) -> bool {
        self.running.get(i).copied().unwrap_or(false)
    }
    fn n2(&self, i: usize) -> f64 {
        self.n2.get(i).copied().unwrap_or(0.0)
    }
}

/// Why the start was called off.
#[derive(Debug, Clone, PartialEq)]
pub enum Abort {
    /// Starter engaged, N2 under [`LIGHT_N2`] for [`NO_SPOOL_SECS`].
    NoSpool { engine: usize, n2: f64 },
    /// Starter engaged, unlit, N2 flat for [`STALL_SECS`].
    Stalled { engine: usize, n2: f64 },
    /// The starter came off short of light-off.
    StarterDropped { engine: usize, n2: f64 },
    /// In progress, nothing moving for [`LATCH_SECS`].
    Latched,
    /// The flag cleared with the engines not running.
    Ended,
    /// The command never started anything.
    NeverBegan,
}

impl Abort {
    pub fn kind(&self) -> &'static str {
        match self {
            Abort::NoSpool { .. } => "no_spool",
            Abort::Stalled { .. } => "stalled",
            Abort::StarterDropped { .. } => "starter_dropped",
            Abort::Latched => "latched",
            Abort::Ended => "ended",
            Abort::NeverBegan => "never_began",
        }
    }
}

/// A call the watch makes.
#[derive(Debug, Clone, PartialEq)]
pub enum Call {
    ApuStarting,
    ApuAvailable,
    ApuBleedOn,
    Starting(usize),
    N2Rising(usize),
    FuelIn(usize),
    StarterCutout(usize),
    Stable(usize),
    AllRunning,
    Aborted(Abort),
}

/// "one", "two", ... for the engine at index `i`.
pub fn engine_word(i: usize) -> String {
    match i {
        0 => "one".into(),
        1 => "two".into(),
        2 => "three".into(),
        3 => "four".into(),
        n => (n + 1).to_string(),
    }
}

/// The line for a sequence that has stopped and will not restart.
pub const RELOAD_LINE: &str =
    "The aircraft's start sequence has stopped and won't restart. It needs the aircraft reloaded.";

impl Call {
    /// Said in milestones mode too (the rest only when every item is called).
    pub fn milestone(&self) -> bool {
        !matches!(
            self,
            Call::ApuBleedOn | Call::N2Rising(_) | Call::FuelIn(_) | Call::Stable(_)
        )
    }

    pub fn spoken(&self, calls: StartCalls) -> bool {
        self.milestone() || calls.items_spoken()
    }

    pub fn kind(&self) -> &'static str {
        match self {
            Call::ApuStarting => "apu_starting",
            Call::ApuAvailable => "apu_available",
            Call::ApuBleedOn => "apu_bleed_on",
            Call::Starting(_) => "starting",
            Call::N2Rising(_) => "n2_rising",
            Call::FuelIn(_) => "fuel_in",
            Call::StarterCutout(_) => "starter_cutout",
            Call::Stable(_) => "stable",
            Call::AllRunning => "all_running",
            Call::Aborted(_) => "aborted",
        }
    }

    pub fn line(&self, engines: usize) -> String {
        match self {
            Call::ApuStarting => "APU starting.".into(),
            Call::ApuAvailable => "APU available.".into(),
            Call::ApuBleedOn => "APU bleed on.".into(),
            Call::Starting(i) => format!("Starting engine {}.", engine_word(*i)),
            Call::N2Rising(_) => "N2 rising.".into(),
            Call::FuelIn(i) => format!("Fuel in, engine {}.", engine_word(*i)),
            Call::StarterCutout(i) => format!("Starter cutout, engine {}.", engine_word(*i)),
            Call::Stable(i) => format!("Engine {} stable.", engine_word(*i)),
            Call::AllRunning => match engines {
                0 | 1 => "Engine running.".into(),
                2 => "Both engines running.".into(),
                _ => "All engines running.".into(),
            },
            Call::Aborted(a) => match a {
                Abort::NoSpool { engine, .. }
                | Abort::Stalled { engine, .. }
                | Abort::StarterDropped { engine, .. } => format!(
                    "Engine {} isn't spooling up. The aircraft's start sequence has stopped; \
                     it needs the aircraft reloaded.",
                    engine_word(*engine)
                ),
                Abort::Latched => RELOAD_LINE.into(),
                Abort::Ended => if engines > 1 {
                    "The aircraft's start sequence ended without the engines running."
                } else {
                    "The aircraft's start sequence ended without the engine running."
                }
                .into(),
                Abort::NeverBegan => "The aircraft's start sequence didn't begin.".into(),
            },
        }
    }
}

/// How the watch ended.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    Running,
    Aborted(Abort),
}

#[derive(Debug, Clone, Default)]
struct EngineWatch {
    starter_since: Option<f64>,
    cranked: bool,
    peak_n2: f64,
    peak_t: f64,
    rising_said: bool,
    fuel_said: bool,
    cutout_said: bool,
    stable_said: bool,
}

/// The pure judge of a scripted start. Feed it every tick's [`Sample`].
#[derive(Debug, Clone)]
pub struct Watcher {
    engines: usize,
    prev: Option<Sample>,
    t_first: f64,
    seen_in_progress: bool,
    last_move_t: f64,
    apu_starting: bool,
    apu_available: bool,
    apu_bleed: bool,
    eng: Vec<EngineWatch>,
    ended_since: Option<f64>,
    finished: Option<Outcome>,
}

impl Watcher {
    pub fn new(engines: usize) -> Self {
        let engines = engines.max(1);
        Self {
            engines,
            prev: None,
            t_first: 0.0,
            seen_in_progress: false,
            last_move_t: 0.0,
            apu_starting: false,
            apu_available: false,
            apu_bleed: false,
            eng: vec![EngineWatch::default(); engines],
            ended_since: None,
            finished: None,
        }
    }

    pub fn finished(&self) -> Option<&Outcome> {
        self.finished.as_ref()
    }

    /// Something moved between two samples: the APU, the flag, a starter, a
    /// light-off, or any N2 by more than half a percent.
    pub fn moved(a: &Sample, b: &Sample, engines: usize) -> bool {
        let apu = match (a.apu_n1, b.apu_n1) {
            (Some(x), Some(y)) => (x - y).abs() > 0.5,
            _ => false,
        };
        apu || a.in_progress != b.in_progress
            || (a.apu_bleed.is_some() && b.apu_bleed.is_some() && a.apu_bleed != b.apu_bleed)
            || (0..engines.max(1)).any(|i| {
                a.starter(i) != b.starter(i)
                    || a.running(i) != b.running(i)
                    || (a.n2(i) - b.n2(i)).abs() > 0.5
            })
    }

    fn all_running(&self, s: &Sample) -> bool {
        (0..self.engines).all(|i| s.running(i) && !s.starter(i) && s.n2(i) >= IDLE_N2)
    }

    fn abort(&mut self, out: &mut Vec<Call>, a: Abort) {
        out.push(Call::Aborted(a.clone()));
        self.finished = Some(Outcome::Aborted(a));
    }

    /// One tick. Returns the calls due, in order; nothing once finished.
    pub fn step(&mut self, s: &Sample) -> Vec<Call> {
        let mut out = Vec::new();
        if self.finished.is_some() {
            return out;
        }
        let t = s.t_s;
        if s.in_progress {
            self.seen_in_progress = true;
            self.ended_since = None;
        }
        let Some(p) = self.prev.clone() else {
            // The baseline: whatever is already true when the watch joins is
            // not news (a start joined midway is not re-announced).
            self.t_first = t;
            self.last_move_t = t;
            let apu = s.apu_n1.unwrap_or(0.0);
            self.apu_starting = apu > 3.0;
            self.apu_available = apu >= 95.0;
            self.apu_bleed = s.apu_bleed == Some(true);
            for i in 0..self.engines {
                let e = &mut self.eng[i];
                if s.starter(i) && !s.running(i) {
                    e.starter_since = Some(t);
                    e.cranked = true;
                    e.peak_n2 = s.n2(i);
                    e.peak_t = t;
                    e.rising_said = s.n2(i) >= 10.0;
                }
                if s.running(i) {
                    e.rising_said = true;
                    e.fuel_said = true;
                    e.cutout_said = !s.starter(i);
                    e.cranked = s.starter(i);
                    e.stable_said = !s.starter(i) && s.n2(i) >= IDLE_N2;
                }
            }
            if self.all_running(s) {
                self.finished = Some(Outcome::Running);
            }
            self.prev = Some(s.clone());
            return out;
        };
        if Self::moved(&p, s, self.engines) {
            self.last_move_t = t;
        }

        // The APU.
        if let Some(apu) = s.apu_n1 {
            if !self.apu_starting && apu > 3.0 && apu < 95.0 {
                self.apu_starting = true;
                out.push(Call::ApuStarting);
            }
            if !self.apu_available && apu >= 95.0 {
                self.apu_starting = true;
                self.apu_available = true;
                out.push(Call::ApuAvailable);
            }
        }
        if !self.apu_bleed && s.apu_bleed == Some(true) && p.apu_bleed != Some(true) {
            self.apu_bleed = true;
            out.push(Call::ApuBleedOn);
        }

        // Each engine, in index order; a new start is called after the
        // cutout it follows (the 737's starter 2 cuts out in the same tick
        // starter 1 engages).
        let mut starts = Vec::new();
        for i in 0..self.engines {
            let (st, pst, run, prun) = (s.starter(i), p.starter(i), s.running(i), p.running(i));
            let (n2, pn2) = (s.n2(i), p.n2(i));
            if st && !pst && !run {
                let e = &mut self.eng[i];
                e.starter_since = Some(t);
                e.cranked = true;
                e.peak_n2 = n2;
                e.peak_t = t;
                starts.push(Call::Starting(i));
            }
            if st && !run {
                if let Some(since) = self.eng[i].starter_since {
                    let e = &mut self.eng[i];
                    if n2 > e.peak_n2 + 1.0 {
                        e.peak_n2 = n2;
                        e.peak_t = t;
                    }
                    if !e.rising_said && n2 >= 10.0 && n2 > pn2 + 0.5 {
                        e.rising_said = true;
                        out.push(Call::N2Rising(i));
                    }
                    let peak_t = e.peak_t;
                    if t - since >= NO_SPOOL_SECS && n2 < LIGHT_N2 {
                        self.abort(&mut out, Abort::NoSpool { engine: i, n2 });
                        return self.done_tick(s, out);
                    }
                    if t - peak_t >= STALL_SECS {
                        self.abort(&mut out, Abort::Stalled { engine: i, n2 });
                        return self.done_tick(s, out);
                    }
                }
            }
            if run && !prun && !self.eng[i].fuel_said {
                self.eng[i].fuel_said = true;
                out.push(Call::FuelIn(i));
            }
            if !st && pst && self.eng[i].cranked && !self.eng[i].cutout_said {
                self.eng[i].starter_since = None;
                if run || n2 >= CUTOUT_MIN_N2 {
                    self.eng[i].cutout_said = true;
                    out.push(Call::StarterCutout(i));
                } else {
                    self.abort(&mut out, Abort::StarterDropped { engine: i, n2 });
                    return self.done_tick(s, out);
                }
            }
            if run && !st && !self.eng[i].stable_said && n2 >= IDLE_N2 && (n2 - pn2).abs() < 0.5 {
                self.eng[i].stable_said = true;
                out.push(Call::Stable(i));
            }
        }

        out.extend(starts);
        if self.all_running(s) {
            out.push(Call::AllRunning);
            self.finished = Some(Outcome::Running);
            return self.done_tick(s, out);
        }

        // The sequence itself.
        if s.in_progress && t - self.last_move_t >= LATCH_SECS {
            self.abort(&mut out, Abort::Latched);
        } else if self.seen_in_progress && !s.in_progress {
            let since = *self.ended_since.get_or_insert(t);
            if t - since >= ENDED_SECS {
                self.abort(&mut out, Abort::Ended);
            }
        } else if !self.seen_in_progress
            && t - self.t_first >= NEVER_BEGAN_SECS
            && t - self.last_move_t >= NEVER_BEGAN_SECS
        {
            self.abort(&mut out, Abort::NeverBegan);
        }
        self.done_tick(s, out)
    }

    fn done_tick(&mut self, s: &Sample, out: Vec<Call>) -> Vec<Call> {
        self.prev = Some(s.clone());
        out
    }
}

// ---- The verdict full auto reads ------------------------------------------

/// Where the scripted start's watch stands, for full auto's engine wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchState {
    /// No watch (none begun, timed out or stood down): the engines decide.
    Idle,
    Watching,
    Running,
    /// Aborted, or found latched: nothing more will start on this load.
    Aborted,
}

/// The watch's verdict (lifetime A: an abort is this airframe load's; a
/// new airframe starts Idle, and a new start replaces it).
static STATE: Mutex<WatchState> = Mutex::new(WatchState::Idle);

pub fn state() -> WatchState {
    *STATE.lock().unwrap_or_else(|e| e.into_inner())
}

pub(crate) fn publish(s: WatchState) {
    *STATE.lock().unwrap_or_else(|e| e.into_inner()) = s;
}

/// A new airframe knows nothing of the last one's start.
pub(crate) fn forget() {
    publish(WatchState::Idle);
}

/// One tick's readings, batched. `None` when the flag, starters, N2 or
/// running flags did not read: no measurement, the tick is skipped.
pub async fn read_sample(xp: &dyn XPlane, t_s: f64) -> Option<Sample> {
    let (flag, apu, bleed, starter, n2, running) = tokio::join!(
        xp.get_float(AUTO_START_IN_PROGRESS),
        xp.get_float(APU_N1),
        xp.get_float(APU_BLEED),
        xp.get_float_array(STARTER),
        xp.get_float_array(N2),
        xp.get_float_array(RUNNING),
    );
    Some(Sample {
        t_s,
        in_progress: flag.ok()? > 0.5,
        apu_n1: apu.ok().filter(|v| v.is_finite()),
        apu_bleed: bleed.ok().map(|v| v > 0.5),
        starter: starter.ok()?.iter().map(|v| *v > 0.5).collect(),
        n2: n2.ok()?,
        running: running.ok()?.iter().map(|v| *v > 0.5).collect(),
    })
}

fn sample_json(s: &Sample, engines: usize) -> serde_json::Value {
    json!({
        "t_s": (s.t_s * 10.0).round() / 10.0,
        "in_progress": s.in_progress,
        "apu_n1": s.apu_n1,
        "apu_bleed": s.apu_bleed,
        "starter": s.starter.iter().take(engines).collect::<Vec<_>>(),
        "n2": s.n2.iter().take(engines).map(|v| (v * 10.0).round() / 10.0).collect::<Vec<_>>(),
        "running": s.running.iter().take(engines).collect::<Vec<_>>(),
    })
}

/// Watch the aircraft's start to its end, calling it as it goes, and
/// publish the verdict. `live` is the run's keep-going test (its lease).
/// `joined` when the watch began on a sequence already in progress.
pub async fn run(
    xp: &dyn XPlane,
    engines: usize,
    voice: Option<&Voice>,
    joined: bool,
    live: impl Fn() -> bool,
) -> Option<Outcome> {
    let engines = engines.max(1);
    let t0 = Instant::now();
    let mut w = Watcher::new(engines);
    crate::blackbox::record(
        "scripted_start",
        json!({ "event": "watch_begins", "engines": engines, "joined": joined }),
    );
    loop {
        if !live() {
            // The verdict is left to the caller: a watch taken over by a new
            // start must not overwrite the new one's `Watching`.
            crate::blackbox::record("scripted_start", json!({ "event": "stood_down" }));
            return None;
        }
        let t = t0.elapsed().as_secs_f64();
        if t > MAX_WATCH_SECS {
            publish(WatchState::Idle);
            crate::blackbox::record("scripted_start", json!({ "event": "watch_timeout" }));
            return None;
        }
        if let Some(s) = read_sample(xp, t).await {
            let calls = crate::crew_pace::start_calls();
            for c in w.step(&s) {
                let line = c.line(engines);
                let spoken = c.spoken(calls);
                let mut rec = json!({
                    "event": c.kind(),
                    "line": line,
                    "spoken": spoken,
                    "sample": sample_json(&s, engines),
                });
                if let Call::Aborted(a) = &c {
                    rec["abort"] = json!(a.kind());
                }
                crate::blackbox::record("scripted_start", rec);
                tracing::info!(target: "xflightcrew::scripted_start", call = c.kind(), %line, spoken);
                if spoken {
                    if let Some(v) = voice {
                        // `alert` mirrors to the panel itself; an aborted
                        // start is a hazard the EFB shows as its caution.
                        if matches!(c, Call::Aborted(_)) {
                            v.hazard(crate::monitor::Severity::Caution, line);
                        } else {
                            v.alert(Priority::Advisory, line);
                        }
                    }
                }
            }
            if let Some(o) = w.finished().cloned() {
                publish(match o {
                    Outcome::Running => WatchState::Running,
                    Outcome::Aborted(_) => WatchState::Aborted,
                });
                return Some(o);
            }
        }
        tokio::time::sleep(Duration::from_millis(POLL_MS)).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(
        t: f64,
        flag: bool,
        apu: f64,
        bleed: bool,
        st: [bool; 2],
        n2: [f64; 2],
        run: [bool; 2],
    ) -> Sample {
        Sample {
            t_s: t,
            in_progress: flag,
            apu_n1: Some(apu),
            apu_bleed: Some(bleed),
            starter: st.to_vec(),
            n2: n2.to_vec(),
            running: run.to_vec(),
        }
    }

    /// The healthy 737 start as sampled live on 2026-10-06, every 2 s.
    fn healthy() -> Vec<Sample> {
        let mut v = Vec::new();
        let mut t: f64 = 0.0;
        while t <= 160.0 {
            let flag = t < 125.0;
            let apu = if t < 26.0 {
                0.0
            } else if t < 66.0 {
                (t - 26.0) * 2.5
            } else if t < 150.0 {
                100.0
            } else {
                60.0
            };
            let bleed = (66.0..125.0).contains(&t);
            // Engine 2 (index 1): starter 78..99, N2 2%/s from 78, lit 92.
            let n2_2 = if t < 78.0 {
                0.0
            } else {
                ((t - 78.0) * 2.0).min(63.7)
            };
            let st2 = (78.0..99.0).contains(&t);
            let run2 = t >= 92.0;
            // Engine 1: starter 99..120, lit 113.
            let n2_1 = if t < 99.0 {
                0.0
            } else {
                ((t - 99.0) * 2.0).min(63.7)
            };
            let st1 = (99.0..120.0).contains(&t);
            let run1 = t >= 113.0;
            v.push(s(
                t,
                flag,
                apu,
                bleed,
                [st1, st2],
                [n2_1, n2_2],
                [run1, run2],
            ));
            t += 2.0;
        }
        v
    }

    fn feed(samples: &[Sample]) -> (Vec<Call>, Watcher) {
        let mut w = Watcher::new(2);
        let mut calls = Vec::new();
        for x in samples {
            calls.extend(w.step(x));
        }
        (calls, w)
    }

    fn lines(calls: &[Call], mode: StartCalls) -> Vec<String> {
        calls
            .iter()
            .filter(|c| c.spoken(mode))
            .map(|c| c.line(2))
            .collect()
    }

    #[test]
    fn a_healthy_737_start_is_called_like_a_crew() {
        let (calls, w) = feed(&healthy());
        assert_eq!(w.finished(), Some(&Outcome::Running));
        assert_eq!(
            lines(&calls, StartCalls::Milestones),
            vec![
                "APU starting.",
                "APU available.",
                "Starting engine two.",
                "Starter cutout, engine two.",
                "Starting engine one.",
                "Starter cutout, engine one.",
                "Both engines running.",
            ]
        );
        let every = lines(&calls, StartCalls::EveryItem);
        for want in [
            "APU bleed on.",
            "N2 rising.",
            "Fuel in, engine two.",
            "Fuel in, engine one.",
        ] {
            assert!(every.iter().any(|l| l == want), "{want} in {every:?}");
        }
        // In order: engine two fuelled before its cutout, then engine one.
        let pos = |l: &str| every.iter().position(|x| x == l).unwrap();
        assert!(pos("Fuel in, engine two.") < pos("Starter cutout, engine two."));
        assert!(pos("Starter cutout, engine two.") < pos("Starting engine one."));
        assert_eq!(
            every.last().map(String::as_str),
            Some("Both engines running.")
        );
        assert!(every.len() <= 14, "{every:?}");
        // No line carries a dash the TTS chokes on.
        for l in &every {
            assert!(!l.contains('\u{2013}') && !l.contains('\u{2014}'), "{l}");
        }
    }

    /// The 09:11Z failure: starter 2 engaged, N2 crawls to 20% and stays
    /// there, the script gives up and leaves its flag set.
    #[test]
    fn a_starter_that_does_not_spool_n2_is_called_off_in_half_a_minute() {
        let mut v: Vec<Sample> = healthy().into_iter().filter(|x| x.t_s < 78.0).collect();
        let mut t: f64 = 78.0;
        while t <= 200.0 {
            let n2 = ((t - 78.0) * 1.0).min(20.0);
            v.push(s(
                t,
                true,
                100.0,
                true,
                [false, t < 103.0],
                [0.0, n2],
                [false, false],
            ));
            t += 2.0;
        }
        let mut w = Watcher::new(2);
        let mut abort_t = None;
        let mut calls = Vec::new();
        for x in &v {
            let c = w.step(x);
            if c.iter().any(|c| matches!(c, Call::Aborted(_))) && abort_t.is_none() {
                abort_t = Some(x.t_s);
            }
            calls.extend(c);
        }
        let abort_t = abort_t.expect("aborted");
        assert!(abort_t - 78.0 <= 32.0, "called at {abort_t}");
        let last = calls.last().unwrap();
        assert!(matches!(last, Call::Aborted(_)), "{last:?}");
        assert_eq!(
            last.line(2),
            "Engine two isn't spooling up. The aircraft's start sequence has stopped; it needs the aircraft reloaded."
        );
        assert!(matches!(w.finished(), Some(Outcome::Aborted(_))));
        assert!(last.spoken(StartCalls::Milestones));
    }

    /// Starter engaged, N2 never rising at all, the starter held: the no
    /// spool rule (or the stall) calls it within thirty seconds.
    #[test]
    fn a_starter_held_with_n2_dead_is_called_off() {
        let mut w = Watcher::new(2);
        let mut got = None;
        let mut t: f64 = 0.0;
        while t <= 60.0 {
            let st = t >= 4.0;
            for c in w.step(&s(
                t,
                true,
                100.0,
                true,
                [false, st],
                [0.0, 0.0],
                [false, false],
            )) {
                if let Call::Aborted(a) = c {
                    got.get_or_insert((t, a));
                }
            }
            t += 2.0;
        }
        let (t, a) = got.expect("aborted");
        assert!(t - 4.0 <= NO_SPOOL_SECS + 2.0, "{t}");
        assert!(matches!(
            a,
            Abort::NoSpool { engine: 1, .. } | Abort::Stalled { engine: 1, .. }
        ));
    }

    /// The starter came off with the engine unlit and N2 low: the start is
    /// over, and it is said at once.
    #[test]
    fn a_starter_dropping_before_light_off_ends_the_start() {
        let mut w = Watcher::new(2);
        w.step(&s(
            0.0,
            true,
            100.0,
            true,
            [false, false],
            [0.0, 0.0],
            [false, false],
        ));
        w.step(&s(
            2.0,
            true,
            100.0,
            true,
            [false, true],
            [0.0, 2.0],
            [false, false],
        ));
        w.step(&s(
            4.0,
            true,
            100.0,
            true,
            [false, true],
            [0.0, 6.0],
            [false, false],
        ));
        let c = w.step(&s(
            6.0,
            true,
            100.0,
            true,
            [false, false],
            [0.0, 7.0],
            [false, false],
        ));
        assert!(matches!(
            c.last(),
            Some(Call::Aborted(Abort::StarterDropped { engine: 1, .. }))
        ));
    }

    /// Found in progress at the arm with nothing moving: latched from an
    /// abort, said as a reload, never "already running".
    #[test]
    fn a_flag_latched_with_nothing_moving_needs_a_reload() {
        let mut w = Watcher::new(2);
        let mut got = None;
        let mut t: f64 = 0.0;
        while t <= 90.0 {
            let x = s(
                t,
                true,
                100.0,
                false,
                [false, false],
                [0.0, 0.0],
                [false, false],
            );
            for c in w.step(&x) {
                got.get_or_insert((t, c));
            }
            t += 2.0;
        }
        let (t, c) = got.expect("called");
        assert_eq!(c, Call::Aborted(Abort::Latched));
        assert!((LATCH_SECS..=LATCH_SECS + 2.0).contains(&t), "{t}");
        assert!(c.line(2).contains("reloaded"));
        assert!(!c.line(2).contains("already running"));
        // Two samples a moment apart: nothing moved.
        let a = s(
            0.0,
            true,
            100.0,
            false,
            [false, false],
            [0.0, 0.0],
            [false, false],
        );
        let b = s(
            2.0,
            true,
            100.0,
            false,
            [false, false],
            [0.0, 0.0],
            [false, false],
        );
        assert!(!Watcher::moved(&a, &b, 2));
    }

    /// A healthy start's quiet first 26 s, and its 12 s between the APU and
    /// the first starter, are never mistaken for a latch.
    #[test]
    fn a_healthy_start_is_never_latched() {
        let (calls, _) = feed(&healthy());
        assert!(
            !calls.iter().any(|c| matches!(c, Call::Aborted(_))),
            "{calls:?}"
        );
        let h = healthy();
        assert!(
            Watcher::moved(&h[20], &h[21], 2),
            "the APU spooling is movement"
        );
        assert!(Watcher::moved(&h[40], &h[41], 2), "N2 rising is movement");
    }

    /// A watch that joins a start under way does not re-announce what is
    /// already done, and still calls the rest.
    #[test]
    fn a_watch_joined_midway_calls_only_what_is_left() {
        let h: Vec<Sample> = healthy().into_iter().filter(|x| x.t_s >= 84.0).collect();
        let (calls, w) = feed(&h);
        assert_eq!(w.finished(), Some(&Outcome::Running));
        let m = lines(&calls, StartCalls::Milestones);
        assert_eq!(
            m,
            vec![
                "Starter cutout, engine two.",
                "Starting engine one.",
                "Starter cutout, engine one.",
                "Both engines running.",
            ]
        );
    }

    /// The flag cleared with the engines not running ends the watch.
    #[test]
    fn a_sequence_ending_without_engines_is_called() {
        let mut w = Watcher::new(2);
        let mut got = None;
        let mut t: f64 = 0.0;
        while t <= 60.0 {
            let flag = t < 10.0;
            for c in w.step(&s(
                t,
                flag,
                (t * 2.0).min(30.0),
                false,
                [false; 2],
                [0.0; 2],
                [false; 2],
            )) {
                if let Call::Aborted(a) = c {
                    got.get_or_insert(a);
                }
            }
            t += 2.0;
        }
        assert_eq!(got, Some(Abort::Ended));
    }

    #[test]
    fn engines_are_named_as_a_crew_names_them() {
        assert_eq!(Call::Starting(1).line(2), "Starting engine two.");
        assert_eq!(Call::AllRunning.line(2), "Both engines running.");
        assert_eq!(Call::AllRunning.line(4), "All engines running.");
        assert!(Call::Starting(0).milestone());
        assert!(!Call::FuelIn(0).milestone());
    }
}
