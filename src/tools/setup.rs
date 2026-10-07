//! "Set up the flight": the flight plan's aircraft, at a stand that suits it at
//! the plan's origin, cold and dark, then the GPS and the load sheet from the
//! plan. One ask instead of X-Plane's flight configuration screen, the plan
//! page and the load sheet, and the same path the unattended sortie runner
//! takes to load an aircraft at a field.
//!
//! Only the plugin can start a flight (`XPLMInitFlight`, on the main thread):
//! the app writes the flight-initialisation JSON to `xcopilot/sim/init_flight`
//! and waits on `xcopilot/sim/init_seq`/`init_result`.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::voice::{Priority, Voice};

const INIT_JSON: &str = "xcopilot/sim/init_flight";
const INIT_RESULT: &str = "xcopilot/sim/init_result";
pub(crate) const INIT_SEQ: &str = "xcopilot/sim/init_seq";
const TYPED: &str = "xcopilot/typed";

// ---- A new flight in progress, and one someone else started ---------------
//
// Two facts about the flight-initialisation mailbox the rest of the crew
// must respect (#134, LEMG 2026-09-28):
// - While the app's OWN new flight loads, nothing arms on the old one: every
//   read is the sim mid-reload. "Fly a circuit" armed 0.6 s after a queued
//   "put us on a 5 mile final" (sortie 1790584476-full_auto), judged the
//   load's transient position (runway 12's midpoint, 36.68532005 N
//   4.49416975 W, AGL -84 ft) as "on 12, 1,376 m left of 2,019 needed", and
//   flew on under the approach that replaced the flight.
// - A new flight posted by ANOTHER writer (the sortie runner's runway and
//   ramp starts go straight to the plugin) is a new flight all the same: the
//   loops flying the old one stand down and its clearances are retired. The
//   runner put the 737 on 13's threshold while the last case's zone
//   clearance (31, squawk 4501) was still held, and full auto taxied to 31.

/// New flights being started or taken on (a setup tool committed to one, or
/// the watch resolving one someone else started): counted up before the
/// follow-up is spawned, down once it has armed or failed.
static NEW_FLIGHT_LOADING: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
/// Mailbox runs this process has in flight ([`run_init`]).
static OWN_INITS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
/// The highest init sequence number this process has accounted for (its own
/// answered runs, and every run the watch has seen): the watch acts only
/// past it. `i64::MIN` until the first read.
static KNOWN_INIT_SEQ: std::sync::atomic::AtomicI64 =
    std::sync::atomic::AtomicI64::new(i64::MIN);

/// Held by a setup's follow-up: the flag stays up until it drops.
pub(crate) struct Loading;

impl Loading {
    pub(crate) fn begin() -> Loading {
        NEW_FLIGHT_LOADING.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        Loading
    }
}

impl Drop for Loading {
    fn drop(&mut self) {
        NEW_FLIGHT_LOADING.fetch_sub(1, std::sync::atomic::Ordering::SeqCst);
    }
}

/// A new flight is loading through this app's mailbox, or one someone else
/// started is still being taken on (its station resolved).
pub fn new_flight_loading() -> bool {
    NEW_FLIGHT_LOADING.load(std::sync::atomic::Ordering::SeqCst) > 0
        || OWN_INITS.load(std::sync::atomic::Ordering::SeqCst) > 0
}

/// The words a mission's arm answers with while a new flight loads.
pub fn new_flight_loading_refusal() -> Option<String> {
    new_flight_loading()
        .then(|| "A new flight is still loading. Ask again once it's up.".to_string())
}

/// What the watch makes of one read of the mailbox's counter and result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InitSeen {
    /// Nothing new (or the first read: the baseline).
    Nothing,
    /// A run nobody here posted has finished; `new_flight` when it made one
    /// (result 0).
    Foreign { new_flight: bool },
}

/// Judge one read against what this process has accounted for: a counter
/// past `known` with an answer (`result >= 0`) is someone else's run.
/// Returns the verdict and the new `known`. Pure.
pub fn judge_init_seq(known: i64, seq: i64, result: i64) -> (InitSeen, i64) {
    if known == i64::MIN {
        return (InitSeen::Nothing, seq);
    }
    if seq <= known || result < 0 {
        return (InitSeen::Nothing, known);
    }
    (InitSeen::Foreign { new_flight: result == 0 }, seq)
}

/// Watch the flight-initialisation mailbox for runs this app did not post
/// (the sortie runner's starts): each new flight stands every loop down and
/// retires the last flight's clearances, and the station for the field
/// under the aircraft's NEW position is rebuilt, as the app's own setups do
/// (blackbox `setup` `foreign_new_flight`). A slow poll: the runner waits for
/// the aircraft to sit still and settles fifteen seconds before its command.
pub fn spawn_new_flight_watch(base: String) {
    spawn_off_loop(async move {
        use std::sync::atomic::Ordering::SeqCst;
        let xp = crate::xplane::XPlaneClient::new(base);
        loop {
            tokio::time::sleep(std::time::Duration::from_secs(2)).await;
            // Our own run accounts for its own number.
            if new_flight_loading() {
                continue;
            }
            let (seq, res) = tokio::join!(xp.get_float(INIT_SEQ), xp.get_float(INIT_RESULT));
            let (Ok(seq), Ok(res)) = (seq, res) else {
                continue;
            };
            let (seen, known) =
                judge_init_seq(KNOWN_INIT_SEQ.load(SeqCst), seq as i64, res as i64);
            KNOWN_INIT_SEQ.fetch_max(known, SeqCst);
            if seen == (InitSeen::Foreign { new_flight: true }) && !new_flight_loading() {
                foreign_new_flight(&xp, seq as i64).await;
            }
        }
    });
}

/// Someone else started a new flight: nothing keeps flying the old one, the
/// world forgets it, and the station is the field the aircraft is now at.
/// The arm waits while this runs (`new_flight_loading`).
async fn foreign_new_flight(xp: &crate::xplane::XPlaneClient, seq: i64) {
    let _loading = Loading::begin();
    crate::flight_state::flight_begins(crate::flight_state::NewFlight::Foreign);
    let atc = crate::mode::atc_enabled();
    let t0 = std::time::Instant::now();
    let fix = if atc { settled_position(xp, SettleTiming::LIVE).await } else { None };
    let station = match fix {
        Some(fix) => new_flight_station(fix, STATION_UP_S).await,
        None => NewFlightStation::default(),
    };
    crate::blackbox::record(
        "setup",
        json!({ "event": "foreign_new_flight", "seq": seq, "field": station.field,
                "tower": station.tower, "station_up": station.up,
                "was": station.was,
                "lat": fix.map(|f| f.lat), "lon": fix.map(|f| f.lon),
                "on_ground": fix.map(|f| f.on_ground),
                "settled": fix.is_some(), "ms": t0.elapsed().as_millis() as u64 }),
    );
}

// ---- The new flight's position, and its station ----------------------------
//
// A new flight is at a new field, and the station must be that field's. The
// sim's own report of where the new flight is lies for a while: through the
// load it gives the first runway's midpoint (EGKK 2026-09-28: 51.14830 N
// 0.18915 W, "on the ground" at 157 ft AGL, 16 s, before stand 71 at
// 51.15540 N 0.16537 W and 0 ft; LEMG #134: runway 12's midpoint at -84 ft
// AGL). The station is chosen only from a position that has settled.

/// One read of where the aircraft is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosSample {
    /// Seconds, any epoch.
    pub t: f64,
    pub lat: f64,
    pub lon: f64,
    pub agl_ft: f64,
    pub on_ground: bool,
    /// Metres per second.
    pub gs_mps: f64,
}

/// A position the station can be chosen from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fix {
    pub lat: f64,
    pub lon: f64,
    pub on_ground: bool,
}

/// On the ground the wheels are within this of the terrain.
const SETTLE_GROUND_AGL_FT: f64 = 20.0;
/// Airborne, the aircraft is at least this far above it.
const SETTLE_AIR_AGL_FT: f64 = 10.0;
/// Slack on the dead-reckoned move between two reads, metres.
const SETTLE_SLACK_M: f64 = 15.0;
/// Plausible reads, each consistent with the last, for this long.
const SETTLE_SPAN_S: f64 = 2.0;
const SETTLE_MIN_SAMPLES: usize = 3;

/// Has the new flight's position settled? Each read must be plausible (on
/// the ground at the terrain, airborne above it) and follow from the last
/// by its groundspeed; a run of them over [`SETTLE_SPAN_S`] is the fix. A
/// read that fails either starts the run again. Pure.
#[derive(Debug, Default)]
pub struct SettleJudge {
    run: Vec<PosSample>,
}

impl SettleJudge {
    pub fn feed(&mut self, s: PosSample) -> Option<Fix> {
        let plausible = if s.on_ground {
            s.agl_ft.abs() <= SETTLE_GROUND_AGL_FT
        } else {
            s.agl_ft >= SETTLE_AIR_AGL_FT
        };
        let follows = self.run.last().is_none_or(|p| {
            let moved = crate::nav::haversine_nm(p.lat, p.lon, s.lat, s.lon) * 1852.0;
            let dt = (s.t - p.t).max(0.0);
            p.on_ground == s.on_ground && moved <= s.gs_mps.abs().max(p.gs_mps.abs()) * dt + SETTLE_SLACK_M
        });
        if !plausible {
            self.run.clear();
            return None;
        }
        if !follows {
            self.run.clear();
        }
        self.run.push(s);
        let first = self.run.first()?;
        (self.run.len() >= SETTLE_MIN_SAMPLES && s.t - first.t >= SETTLE_SPAN_S).then_some(Fix {
            lat: s.lat,
            lon: s.lon,
            on_ground: s.on_ground,
        })
    }
}

/// How the settle wait keeps time.
#[derive(Debug, Clone, Copy)]
pub struct SettleTiming {
    pub poll: std::time::Duration,
    pub deadline: std::time::Duration,
}

impl SettleTiming {
    pub const LIVE: SettleTiming = SettleTiming {
        poll: std::time::Duration::from_millis(500),
        deadline: std::time::Duration::from_secs(60),
    };
}

/// Read the position until it settles ([`SettleJudge`]); `None` by the
/// deadline. A failed read is no sample.
pub async fn settled_position(xp: &crate::xplane::XPlaneClient, t: SettleTiming) -> Option<Fix> {
    let t0 = std::time::Instant::now();
    let mut judge = SettleJudge::default();
    while t0.elapsed() < t.deadline {
        let (lat, lon, agl, g, gs) = tokio::join!(
            xp.get_float(crate::datarefs::LATITUDE),
            xp.get_float(crate::datarefs::LONGITUDE),
            xp.get_float(crate::datarefs::AGL_METERS),
            xp.get_float(crate::datarefs::ON_GROUND_ANY),
            xp.get_float(crate::datarefs::GROUNDSPEED_MS),
        );
        if let (Ok(lat), Ok(lon), Ok(agl), Ok(g), Ok(gs)) = (lat, lon, agl, g, gs) {
            let s = PosSample {
                t: t0.elapsed().as_secs_f64(),
                lat,
                lon,
                agl_ft: agl * 3.28084,
                on_ground: g > 0.5,
                gs_mps: gs,
            };
            if let Some(fix) = judge.feed(s) {
                return Some(fix);
            }
        }
        tokio::time::sleep(t.poll).await;
    }
    None
}

/// How long a new flight waits for its station to be installed.
pub(crate) const STATION_UP_S: u64 = 15;

/// What a new flight's station came to.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NewFlightStation {
    /// The towered field under the new position (`None`: none, nothing armed).
    pub field: Option<String>,
    pub tower: Option<i64>,
    /// The main loop installed it within the wait.
    pub up: bool,
    /// The station that was up before.
    pub was: Option<String>,
}

/// The station for the field under the new flight's settled position: armed
/// ([`crate::station::arm_new_flight_at`]) and waited on until the main loop
/// has installed it, at most `wait_s`.
pub async fn new_flight_station(fix: Fix, wait_s: u64) -> NewFlightStation {
    let was = crate::station::active_icao().map(|(icao, _, _)| icao);
    let Some((field, khz)) =
        crate::station::arm_new_flight_at(fix.lat, fix.lon, fix.on_ground, std::time::Instant::now())
    else {
        return NewFlightStation { was, ..NewFlightStation::default() };
    };
    let up = await_station_up(khz, wait_s).await;
    NewFlightStation { field: Some(field), tower: Some(khz), up, was }
}

/// Wait for the armed swap to be installed with this tower, at most `wait_s`.
pub(crate) async fn await_station_up(khz: i64, wait_s: u64) -> bool {
    let t0 = std::time::Instant::now();
    let up = || {
        !crate::party_line::station_swap_pending()
            && crate::party_line::home_tower_khz() == Some(khz)
    };
    while t0.elapsed().as_secs() < wait_s && !up() {
        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
    }
    up()
}

/// The ICAO type an `.acf` declares (`P acf/_ICAO B738`). Pure.
pub fn acf_icao(acf_text: &str) -> Option<String> {
    acf_text.lines().find_map(|l| {
        let rest = l.strip_prefix("P acf/_ICAO ")?;
        let t = rest.trim();
        (!t.is_empty()).then(|| t.to_ascii_uppercase())
    })
}

/// Every installed aircraft's `.acf`, relative to the X-Plane root, with the
/// ICAO type it declares. Read once per process: the files are megabytes and
/// the type sits tens of thousands of lines in.
pub fn aircraft_library(root: &str) -> &'static [(String, String)] {
    static LIB: std::sync::OnceLock<Vec<(String, String)>> = std::sync::OnceLock::new();
    LIB.get_or_init(|| {
        let base = std::path::Path::new(root);
        let mut out = Vec::new();
        let mut stack = vec![base.join("Aircraft")];
        while let Some(dir) = stack.pop() {
            let Ok(rd) = std::fs::read_dir(&dir) else { continue };
            for e in rd.flatten() {
                let p = e.path();
                if p.is_dir() {
                    stack.push(p);
                } else if p.extension().is_some_and(|x| x.eq_ignore_ascii_case("acf")) {
                    let Ok(text) = std::fs::read_to_string(&p) else { continue };
                    if let (Some(icao), Ok(rel)) = (acf_icao(&text), p.strip_prefix(base)) {
                        out.push((icao, rel.to_string_lossy().to_string()));
                    }
                }
            }
        }
        out.sort();
        out
    })
}

/// The installed aircraft for an ICAO type: an exact match, Laminar's own
/// first (the profiles are written against them). Pure over the library.
pub fn find_acf<'a>(lib: &'a [(String, String)], icao: &str) -> Option<&'a str> {
    let icao = icao.trim().to_ascii_uppercase();
    let mut hits: Vec<&(String, String)> = lib.iter().filter(|(t, _)| *t == icao).collect();
    hits.sort_by_key(|(_, p)| (!p.contains("Laminar Research"), p.len()));
    hits.first().map(|(_, p)| p.as_str())
}

/// What a stand must accept for this aircraft (apt.dat 1300 categories).
pub fn stand_class(jet: bool, heli: bool) -> &'static [&'static str] {
    match (jet, heli) {
        (_, true) => &["helos"],
        (true, _) => &["jets", "heavy"],
        _ => &["props", "turboprops"],
    }
}

/// A stand for the aircraft: a gate (a jet) or a tie-down (a prop) that
/// accepts its class, never a hangar; the first by name, so the pick is
/// repeatable. Pure.
pub fn pick_stand<'a>(stands: &'a [crate::nav::Stand], class: &[&str], jet: bool) -> Option<&'a crate::nav::Stand> {
    let fits = |s: &crate::nav::Stand| {
        s.is_suitable()
            && (class.is_empty() || s.accepts.is_empty() || s.accepts.iter().any(|a| class.contains(&a.as_str())))
    };
    let preferred = if jet { "gate" } else { "tie_down" };
    let mut pool: Vec<&crate::nav::Stand> = stands
        .iter()
        .filter(|s| fits(s) && s.kind.eq_ignore_ascii_case(preferred))
        .collect();
    if pool.is_empty() {
        pool = stands.iter().filter(|s| fits(s)).collect();
    }
    pool.sort_by(|a, b| a.name.cmp(&b.name));
    pool.first().copied()
}

/// The flight-initialisation JSON: the aircraft at the ramp, cold and dark.
pub fn init_json(acf_rel: &str, airport: &str, ramp: &str) -> String {
    json!({
        "aircraft": { "path": acf_rel },
        "ramp_start": { "airport_id": airport, "ramp": ramp },
        "engine_status": { "all_engines": { "running": false } },
    })
    .to_string()
}

pub struct SetUpFlight;

#[async_trait]
impl Tool for SetUpFlight {
    fn name(&self) -> &str {
        "set_up_flight"
    }
    fn description(&self) -> &str {
        "Set up the flight from the flight plan: load the plan's aircraft at a stand at \
         its origin, cold and dark, then load the GPS and the fuel and payload. For \
         'set up the flight', 'set up my flight', 'prepare the flight'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Starting a flight ENDS this one: only parked, and nothing flying it.
        // The load sheet's "stopped" (it refused at 1 kt while this ended
        // the flight at 2 kt; refactor audit 2026-10-04, duplicates #9).
        if !sim::ground::read_stopped_on_ground(ctx.xplane).await {
            return Ok("Only when we're stopped on the ground: setting up a flight ends this one.".into());
        }
        if crate::full_auto::ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
            return Ok("I'm flying. Say my controls first.".into());
        }
        let fp = match super::loading::plan_for_loading(&ctx.config.simbrief_id).await {
            Ok(fp) => fp,
            Err(line) => return Ok(line),
        };
        // The first scan reads every installed .acf: off the command loop.
        let root = crate::config::xplane_root();
        let lib = tokio::task::spawn_blocking(move || aircraft_library(&root)).await?;
        let Some(acf) = find_acf(lib, &fp.aircraft) else {
            return Ok(format!(
                "I can't find an installed aircraft of type {}, the plan's. Load it yourself, or change the plan.",
                fp.aircraft
            ));
        };
        let Ok(Some(net)) = crate::nav::find_taxi_net(&ctx.config.apt_data_path, &fp.origin) else {
            return Ok(format!("{} has no parking positions I can read.", fp.origin));
        };
        // Jet or not, from a profile of the plan's TYPE (a plan names no
        // model, and any model of the type answers this); an unknown type
        // takes any suitable stand.
        let known = crate::aircraft::profile_for_type(&fp.aircraft);
        let jet = known.as_ref().is_some_and(|p| {
            p.reference
                .as_ref()
                .and_then(|r| r.kind.as_deref())
                .is_some_and(|k| k.to_ascii_lowercase().contains("jet"))
        });
        let heli = known.as_ref().is_some_and(|p| p.rotorcraft);
        let class: &[&str] = if known.is_some() { stand_class(jet, heli) } else { &[] };
        let Some(stand) = pick_stand(&net.stands, class, jet) else {
            return Ok(format!("No stand at {} takes this aircraft.", fp.origin));
        };
        let body = init_json(acf, &fp.origin, &stand.name);
        let (base, voice) = (ctx.config.xplane_base_url.clone(), ctx.voice.cloned());
        let (origin, name, stand_name) = (fp.origin.clone(), fp.aircraft.clone(), stand.name.clone());
        crate::blackbox::record(
            "setup",
            json!({ "event": "init", "acf": acf, "airport": origin, "ramp": stand_name, "json": body }),
        );
        let loading = Loading::begin();
        spawn_off_loop(async move {
            let _loading = loading;
            follow_up(base, voice, body, origin, stand_name).await;
        });
        Ok(format!(
            "Setting up: the {name} at {}, {}, cold and dark. Then the plan and the load sheet.",
            fp.origin, stand.name
        ))
    }
}

/// After the write: wait for the plugin to run it, then for the sim to have
/// reloaded, then ask for the GPS load and the load sheet the way a typed
/// command would (the aircraft has changed, so the registry has too).
async fn follow_up(base: String, voice: Option<Voice>, body: String, origin: String, stand: String) {
    let xp = crate::xplane::XPlaneClient::new(base);
    let say = |line: &str| {
        if let Some(v) = &voice {
            v.alert(Priority::Advisory, line.to_string());
        }
    };
    // The plugin runs it on its next flight loop; the reload itself can take
    // a minute with scenery to load.
    let t0 = std::time::Instant::now();
    let run = run_init(&xp, body.as_bytes(), InitTiming::RAMP, &|w| {
        crate::blackbox::record("setup", json!({ "event": "init_wait", "seq": w.seq, "result": w.result,
            "seq0": w.seq0, "read_errors": w.read_errors, "polls": w.polls }));
    })
    .await;
    let result = match run.outcome {
        InitOutcome::Done(code) => Some(code),
        InitOutcome::Unreachable | InitOutcome::NoBaseline => {
            crate::blackbox::record(
                "setup",
                json!({ "event": "result", "outcome": run.outcome.name(),
                        "read_errors": run.read_errors }),
            );
            say("I couldn't reach the plugin to set up the flight. Is it up to date?");
            return;
        }
        InitOutcome::TimedOut => None,
    };
    crate::blackbox::record("setup", json!({ "event": "result", "result": result }));
    match result {
        // The flight that ended ends here: its loops, its facts, its
        // checklist and its clearances (the door; the stand-down is new to
        // the ramp, which only ever refused a mission in flight).
        Some(0) => {
            crate::flight_state::flight_begins(crate::flight_state::NewFlight::Ramp);
        }
        Some(code) => {
            let why = match code {
                1 => "the request was invalid",
                2 => "the aircraft would not load",
                4 => "X-Plane doesn't know the airport",
                5 => "that stand isn't in the loaded scenery",
                _ => "X-Plane refused it",
            };
            say(&format!("The setup didn't take: {why}."));
            return;
        }
        None => {
            say("The setup didn't answer. Check X-Plane.");
            return;
        }
    }
    // Settled: on the ground AT the terrain and still for five seconds (the
    // load's transient sits "on the ground" 157 ft up a runway's midpoint,
    // EGKK 2026-09-28), then the profile detection has its moment before the
    // plan and the load.
    let mut still = 0;
    while t0.elapsed().as_secs() < 240 && still < 10 {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        let (g, gs, agl) = tokio::join!(
            xp.get_float(crate::datarefs::ON_GROUND_ANY),
            xp.get_float(crate::datarefs::GROUNDSPEED_MS),
            xp.get_float(crate::datarefs::AGL_METERS),
        );
        still = if sim::ground::stopped_on_ground(g.ok(), gs.ok())
            && agl.is_ok_and(|m| (m * 3.28084).abs() <= SETTLE_GROUND_AGL_FT)
        {
            still + 1
        } else {
            0
        };
    }
    // The station is the field we are now at, up before the flag drops and
    // anything arms (#151).
    if crate::mode::atc_enabled() {
        let fix = settled_position(&xp, SettleTiming::LIVE).await;
        let station = match fix {
            Some(fix) => new_flight_station(fix, STATION_UP_S).await,
            None => NewFlightStation::default(),
        };
        crate::blackbox::record(
            "setup",
            json!({ "event": "ramp_station", "airport": origin, "field": station.field,
                    "tower": station.tower, "station_up": station.up, "was": station.was,
                    "settled": fix.is_some() }),
        );
    }
    tokio::time::sleep(std::time::Duration::from_secs(8)).await;
    say(&format!("We're at {origin}, {stand}. Loading the plan and the load sheet."));
    let _ = xp.set_data(TYPED, b"load the flight plan").await;
    tokio::time::sleep(std::time::Duration::from_secs(8)).await;
    let _ = xp.set_data(TYPED, b"load the aircraft").await;
    crate::blackbox::record("setup", json!({ "event": "done", "airport": origin, "ramp": stand }));
}

// ---- A new flight on a final --------------------------------------------------
//
// "Put us on a ten mile final to LEMG one three": the CURRENT aircraft on a
// runway final, engines running, through the same mailbox. X-Plane places the
// aircraft with NOBODY flying it (autopilot off, 180 to 210 kt, sinking 1,000
// to 3,500 fpm on LEMG's 10 nm final over 2,300 ft hills, 2026-09-26: forty
// seconds of "Airspeed!" and "Sink rate!" and no hands until a typed
// "recover the aircraft"). So the copilot takes the aircraft the moment the
// flight exists: autoland FORCED onto that runway, armed on the result, or,
// when the pilot said "my controls", monitoring only. The tower is told it is
// a new flight: its station is rebuilt and the final report goes out.

/// The final a pilot means by "put us on final": ten miles.
pub const DEFAULT_FINAL_NM: i64 = 10;
/// The longest final the setup accepts, miles.
pub const MAX_FINAL_NM: i64 = 30;

/// The flight-initialisation JSON for a runway final: the aircraft that is
/// loaded, the field and runway, the distance out, engines running (the SDK's
/// `runway_start` with `final_distance_in_nautical_miles`; without the distance
/// it is a start on the numbers, which this never writes). Pure.
pub fn approach_init_json(acf_rel: &str, airport: &str, runway: &str, final_nm: i64) -> String {
    json!({
        "aircraft": { "path": acf_rel },
        "runway_start": {
            "airport_id": airport,
            "runway": runway,
            "final_distance_in_nautical_miles": final_nm,
        },
        "engine_status": { "all_engines": { "running": true } },
    })
    .to_string()
}

/// Why X-Plane refused an initialisation, in words (`XPLMInitResult`,
/// XPLMPlanes.h). Pure.
pub fn init_refusal(code: i64, icao: &str, runway: &str) -> String {
    let why = match code {
        1 => "X-Plane called the request invalid".to_string(),
        2 => "X-Plane couldn't load this aircraft".to_string(),
        3 => "X-Plane couldn't load the livery".to_string(),
        4 => format!("X-Plane doesn't know the airport {}", crate::flightplan::spell_code(icao)),
        5 => "X-Plane couldn't find that ramp".to_string(),
        6 => format!(
            "X-Plane has no runway {} at {}",
            crate::crew_radio::spell_runway(runway),
            crate::flightplan::spell_code(icao)
        ),
        _ => format!("X-Plane refused it, code {code}"),
    };
    format!("The approach setup didn't take: {why}.")
}

/// What follows the plugin's answer. `Arm` is the copilot's hands on the new
/// flight at once, autoland forced onto the runway the flight was started
/// on; `Monitor` leaves the pilot flying.
#[derive(Debug, Clone, PartialEq)]
pub enum AfterInit {
    Arm {
        runway: crate::runway::PlannedRunway,
        line: String,
    },
    Monitor {
        line: String,
    },
    Failed {
        line: String,
    },
}

/// The decision on the plugin's result (`None`: no answer in time). Pure:
/// the follow-up applies it with nothing between the result and the arm.
pub fn after_init(
    result: Option<i64>,
    icao: &str,
    runway: &str,
    final_nm: i64,
    hand_fly: bool,
    can_land: bool,
) -> AfterInit {
    let spoken = crate::crew_radio::spell_runway(runway);
    match result {
        Some(0) if hand_fly => AfterInit::Monitor {
            line: format!("On a {final_nm} mile final for {spoken}. Your aircraft, I'm monitoring."),
        },
        Some(0) if !can_land => AfterInit::Monitor {
            line: format!(
                "On a {final_nm} mile final for {spoken}. I have no autopilot to fly it with. Your aircraft."
            ),
        },
        Some(0) => AfterInit::Arm {
            runway: crate::runway::PlannedRunway::at(icao, runway),
            line: format!("On a {final_nm} mile final for {spoken}, I have the aircraft."),
        },
        Some(code) => AfterInit::Failed { line: init_refusal(code, icao, runway) },
        None => AfterInit::Failed {
            line: "The approach setup didn't answer. Check X-Plane.".to_string(),
        },
    }
}

/// "13" and "013" and "13" are one end; "13" is not "13L". Pure.
fn same_end(a: &str, b: &str) -> bool {
    let split = |s: &str| {
        let s = s.trim().to_ascii_uppercase();
        let digits: String = s.chars().take_while(|c| c.is_ascii_digit()).collect();
        let n = digits.parse::<u32>().ok();
        (n, s[digits.len()..].to_string())
    };
    let (na, sa) = split(a);
    na.is_some() && (na, sa) == split(b)
}

/// Every runway end the field has, as apt.dat spells them.
pub fn runway_ends(runways: &[crate::nav::Runway]) -> Vec<String> {
    runways
        .iter()
        .flat_map(|r| [r.ident_a.clone(), r.ident_b.clone()])
        .collect()
}

/// The apt.dat spelling of the end the words name, if the field has it.
pub fn match_runway_end(runways: &[crate::nav::Runway], asked: &str) -> Option<String> {
    runway_ends(runways).into_iter().find(|e| same_end(e, asked))
}

/// The runway for the new final, and where it came from: the pilot's words,
/// else the tower's clearance at this field, else the plan's arrival runway,
/// else the end facing the field's METAR wind, else the longest runway. A
/// runway the pilot NAMED that the field lacks is refused with the ends it
/// has, never swapped for another. Pure.
pub fn pick_runway(
    icao: &str,
    runways: &[crate::nav::Runway],
    asked: Option<&str>,
    tower: Option<&str>,
    plan: Option<&str>,
    wind: Option<(f64, f64)>,
) -> Result<(String, &'static str), String> {
    if let Some(a) = asked {
        return match_runway_end(runways, a)
            .map(|e| (e, "asked"))
            .ok_or_else(|| {
                format!(
                    "{} has no runway {}. It has {}.",
                    crate::flightplan::spell_code(icao),
                    crate::crew_radio::spell_runway(a),
                    runway_ends(runways)
                        .iter()
                        .map(|e| crate::crew_radio::spell_runway(e))
                        .collect::<Vec<_>>()
                        .join(", ")
                )
            });
    }
    if let Some(e) = tower.and_then(|t| match_runway_end(runways, t)) {
        return Ok((e, "tower"));
    }
    if let Some(e) = plan.and_then(|p| match_runway_end(runways, p)) {
        return Ok((e, "plan"));
    }
    if let Some((from, kts)) = wind {
        if let Some((e, _, _)) = crate::nav::best_runway_for_wind(runways, from, kts) {
            return Ok((e, "wind"));
        }
    }
    crate::nav::assignable_runways(runways)
        .max_by(|a, b| a.length_m().total_cmp(&b.length_m()))
        .map(|r| (r.ident_a.clone(), "longest"))
        .ok_or_else(|| format!("{} has no runway I can read.", crate::flightplan::spell_code(icao)))
}

/// The automation that holds the aircraft now, by name, if any: a new flight
/// ends it, so the pilot takes it back first (a misheard ask must not pull
/// the aircraft out from under a landing).
///
/// [`crate::flight_state::crew_flying`], innermost: the helicopter circuit
/// and a recovery count too (they were missing, so a setup could pull the
/// aircraft out from under either).
pub(crate) fn flying_now() -> Option<&'static str> {
    crate::flight_state::crew_flying().map(crate::flight_state::Owner::doing)
}

#[derive(serde::Deserialize, Default, Clone)]
struct ApproachArgs {
    #[serde(default)]
    final_nm: Option<i64>,
    #[serde(default)]
    runway: Option<String>,
    #[serde(default)]
    to: Option<String>,
    #[serde(default)]
    to_codes: Vec<String>,
    #[serde(default)]
    hand_fly: bool,
}

/// The field and its runways, resolved off the command loop: the named
/// place, else the plan's destination, else the nearest field.
fn resolve_field(
    apt: &str,
    a: &ApproachArgs,
    here: (f64, f64),
) -> Result<(String, crate::nav::AirportInfo), String> {
    let icao = if a.to.is_some() || !a.to_codes.is_empty() {
        let dest = crate::flightplan::Destination {
            codes: a.to_codes.clone(),
            name: a.to.clone().unwrap_or_default(),
        };
        let spoken = a.to.clone().unwrap_or_else(|| dest.spoken());
        match crate::flightplan::resolve(apt, &dest, here) {
            Ok(r) => r
                .dest
                .map(|d| d.icao)
                .ok_or_else(|| format!("I don't know an airport called {spoken}."))?,
            Err(_) => return Err("I can't read the airport data.".to_string()),
        }
    } else if let Some(d) =
        crate::simbrief::loaded_plan().map(|p| p.destination).filter(|d| !d.trim().is_empty())
    {
        d
    } else {
        let none = crate::flightplan::Destination { codes: Vec::new(), name: String::new() };
        match crate::flightplan::resolve(apt, &none, here) {
            Ok(r) => r.origin.map(|o| o.icao).ok_or("There's no field near us I can find.")?,
            Err(_) => return Err("I can't read the airport data.".to_string()),
        }
    };
    match crate::nav::find_airport(apt, &icao) {
        Ok(Some(info)) if !info.runways.is_empty() => Ok((icao, info)),
        _ => Err(format!("{} has no runways I can read.", crate::flightplan::spell_code(&icao))),
    }
}

pub struct SetUpApproach {
    /// The autopilot and approach speed autoland flies with, from the
    /// registry's own autoland; `None` on an airframe without an autopilot,
    /// where the new flight is the pilot's.
    pub land: Option<(crate::tools::autopilot::ApConfig, f64)>,
}

#[async_trait]
impl Tool for SetUpApproach {
    fn name(&self) -> &str {
        "setup_approach"
    }
    fn description(&self) -> &str {
        "Start a new flight on a runway final with this aircraft, engines running, and \
         take the aircraft (autoland onto that runway) unless the pilot will fly it. For \
         'put us on a 10 mile final to LEMG 13', 'set us up on final for runway one three', \
         'put us on approach, my controls'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "final_nm": {"type": "integer"},
                "runway": {"type": "string"},
                "to": {"type": "string"},
                "to_codes": {"type": "array", "items": {"type": "string"}},
                "hand_fly": {"type": "boolean"}
            },
            "additionalProperties": false
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: ApproachArgs = serde_json::from_value(args.clone()).unwrap_or_default();
        let final_nm = a.final_nm.unwrap_or(DEFAULT_FINAL_NM);
        if !(1..=MAX_FINAL_NM).contains(&final_nm) {
            return Ok(format!("A final between one and {MAX_FINAL_NM} miles, please."));
        }
        if ctx
            .xplane
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5)
        {
            return Ok("We're in replay. Go back to live and ask again.".into());
        }
        if let Some(who) = flying_now() {
            return Ok(format!("{who}. Say my controls first, then ask again."));
        }
        // The string read is the concrete client's (the tool trait reads numbers).
        let strings = crate::xplane::XPlaneClient::new(ctx.config.xplane_base_url.clone());
        let (ground, lat, lon, acf) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY),
            ctx.xplane.get_float(crate::datarefs::LATITUDE),
            ctx.xplane.get_float(crate::datarefs::LONGITUDE),
            strings.get_string(crate::datarefs::ACF_RELATIVE_PATH),
        );
        // In the air a bare "put us on final" could be words about the
        // circuit being flown; a new flight there needs a distance, a runway
        // or a field said with it.
        let explicit = a.final_nm.is_some() || a.runway.is_some() || a.to.is_some();
        if !ground.is_ok_and(|g| g > 0.5) && !explicit {
            return Ok(
                "That starts a new flight on a final. Say the distance or the runway if that's what you want."
                    .into(),
            );
        }
        let (Ok(lat), Ok(lon)) = (lat, lon) else {
            return Ok("I can't read where we are.".into());
        };
        let Some(acf) = acf.ok().filter(|p| !p.trim().is_empty()) else {
            return Ok("I can't read which aircraft is loaded.".into());
        };
        let apt = ctx.config.apt_data_path.clone();
        let resolved = {
            let (apt, a) = (apt.clone(), a.clone());
            tokio::task::spawn_blocking(move || resolve_field(&apt, &a, (lat, lon))).await?
        };
        let (icao, info) = match resolved {
            Ok(x) => x,
            Err(line) => return Ok(line),
        };
        // The tower's runway only when its station is this field.
        let tower = crate::station::active_icao()
            .filter(|(i, _, _)| i.eq_ignore_ascii_case(&icao))
            .and_then(|_| crate::party_line::clearance_runway());
        let plan = crate::simbrief::loaded_plan()
            .filter(|p| p.destination.eq_ignore_ascii_case(&icao))
            .and_then(|p| p.procedures.dest_rwy);
        let wind = crate::weather::surface_wind(&crate::config::xplane_root(), &icao);
        let (rwy, source) = match pick_runway(
            &icao,
            &info.runways,
            a.runway.as_deref(),
            tower.as_deref(),
            plan.as_deref(),
            wind,
        ) {
            Ok(x) => x,
            Err(line) => return Ok(line),
        };
        let body = approach_init_json(&acf, &icao, &rwy, final_nm);
        crate::blackbox::record(
            "setup",
            json!({ "event": "approach_init", "acf": acf, "airport": icao, "runway": rwy,
                    "runway_from": source, "final_nm": final_nm, "hand_fly": a.hand_fly,
                    "json": body }),
        );
        // The new flight ends this one: nothing may keep flying it.
        crate::flight_state::stand_down();
        let job = ApproachJob {
            base: ctx.config.xplane_base_url.clone(),
            apt,
            voice: ctx.voice.cloned(),
            body,
            icao: icao.clone(),
            runway: rwy.clone(),
            final_nm,
            hand_fly: a.hand_fly,
            land: self.land.clone(),
            field: info.position(),
        };
        // Its own thread and runtime, like every loop that must keep time:
        // the command loop's runtime is current-thread and blocks on the
        // microphone between passes, so a task spawned on it runs only in the
        // slivers where the loop awaits. At LEMG (2026-09-26) the first setup
        // never wrote the mailbox, and the second wrote it (the sim
        // repositioned 6 s later) but never read the result: no arm, no line,
        // not even the timeout, and the 737 dived with nobody flying.
        let loading = Loading::begin();
        spawn_off_loop(approach_follow_up(job, loading));
        Ok(format!(
            "Setting up a {final_nm} mile final for runway {} at {}.",
            crate::crew_radio::spell_runway(&rwy),
            crate::flightplan::spell_code(&icao)
        ))
    }
}

struct ApproachJob {
    base: String,
    apt: String,
    voice: Option<Voice>,
    body: String,
    icao: String,
    runway: String,
    final_nm: i64,
    hand_fly: bool,
    land: Option<(crate::tools::autopilot::ApConfig, f64)>,
    field: Option<(f64, f64)>,
}

/// Run a future to completion on its own thread and runtime, the way
/// `run_autoland` and every loop run. The command loop's runtime is
/// current-thread and blocks synchronously on the microphone between passes,
/// so a `tokio::spawn` from a tool starves (`speedhold`'s governor sat
/// unpolled 23 s; the approach setup never read its result). Pinned by
/// `no_tool_leaves_work_on_the_command_loops_runtime`.
pub fn spawn_off_loop<F>(fut: F)
where
    F: std::future::Future<Output = ()> + Send + 'static,
{
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            return;
        };
        rt.block_on(fut);
    });
}

/// How long a sortie waits for its loop to start, and how often it looks.
#[derive(Debug, Clone, Copy)]
pub struct SortieHold {
    pub start_within: std::time::Duration,
    pub poll: std::time::Duration,
}

impl SortieHold {
    pub const LIVE: SortieHold = SortieHold {
        start_within: std::time::Duration::from_secs(10),
        poll: std::time::Duration::from_millis(250),
    };
}

/// Keep a sortie open while `flag` (a loop's ACTIVE) is up: it ends when
/// the loop lets go, or when the loop never started. The flag, not the
/// thread, because a re-armed autoland hands ACTIVE to its successor and
/// the landing it flies is still this sortie's.
pub fn hold_sortie_while(
    flag: &'static std::sync::atomic::AtomicBool,
    sortie: crate::full_auto::SortieGuard,
    t: SortieHold,
) -> std::thread::JoinHandle<()> {
    use std::sync::atomic::Ordering::SeqCst;
    std::thread::spawn(move || {
        let t0 = std::time::Instant::now();
        while !flag.load(SeqCst) && t0.elapsed() < t.start_within {
            std::thread::sleep(t.poll);
        }
        while flag.load(SeqCst) {
            std::thread::sleep(t.poll);
        }
        drop(sortie);
    })
}

/// The flight-initialisation mailbox as the follow-up sees it: numbers read,
/// one JSON posted. The live one is the Web API; the tests fake a reload.
#[async_trait]
pub trait InitMailbox: Send + Sync {
    async fn read(&self, name: &str) -> Result<f64>;
    async fn post(&self, body: &[u8]) -> Result<()>;
}

#[async_trait]
impl InitMailbox for crate::xplane::XPlaneClient {
    async fn read(&self, name: &str) -> Result<f64> {
        self.get_float(name).await
    }
    async fn post(&self, body: &[u8]) -> Result<()> {
        self.set_data(INIT_JSON, body).await
    }
}

/// How the mailbox wait keeps time.
#[derive(Debug, Clone, Copy)]
pub struct InitTiming {
    /// Between polls: the most the arm can trail the flight's existence.
    pub poll: std::time::Duration,
    /// One read's cap. The client's own is 8 s, and a read that hangs through
    /// the sim's reload must not hold the arm back: it is abandoned and the
    /// next poll asks again.
    pub read_timeout: std::time::Duration,
    /// No answer by then is a timeout, said.
    pub deadline: std::time::Duration,
    /// How often the wait reports what it sees (blackbox `approach_wait`).
    pub report_every: std::time::Duration,
}

impl InitTiming {
    // The mailbox reads ride through the load quiet window (the only reads
    // that do), so they ask no faster than this: two reads a poll were 13 a
    // second into a loading sim (2026-09-28).
    pub const LIVE: InitTiming = InitTiming {
        poll: std::time::Duration::from_millis(350),
        read_timeout: std::time::Duration::from_millis(600),
        deadline: std::time::Duration::from_secs(120),
        report_every: std::time::Duration::from_secs(2),
    };
    pub const RAMP: InitTiming = InitTiming {
        poll: std::time::Duration::from_millis(500),
        read_timeout: std::time::Duration::from_secs(2),
        deadline: std::time::Duration::from_secs(120),
        report_every: std::time::Duration::from_secs(2),
    };
}

/// How a mailbox run ended.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InitOutcome {
    /// The plugin ran it: `XPLMInitResult` (0 = the flight exists).
    Done(i64),
    /// The write did not reach the plugin.
    Unreachable,
    /// The completion counter could not be read before the write, so no
    /// answer could be told from the last flight's: nothing was written.
    NoBaseline,
    /// Written, and no answer by the deadline.
    TimedOut,
}

impl InitOutcome {
    pub fn name(&self) -> &'static str {
        match self {
            InitOutcome::Done(0) => "ok",
            InitOutcome::Done(_) => "refused",
            InitOutcome::Unreachable => "unreachable",
            InitOutcome::NoBaseline => "no_baseline",
            InitOutcome::TimedOut => "timeout",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitRun {
    pub outcome: InitOutcome,
    /// Reads that failed or were abandoned (the sim mid-reload).
    pub read_errors: u32,
    pub polls: u32,
}

/// What the wait sees, reported every `report_every`: the last good reads
/// (`None` = that read failed) against the baseline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitWait {
    pub seq0: f64,
    pub seq: Option<f64>,
    pub result: Option<f64>,
    pub read_errors: u32,
    pub polls: u32,
}

/// Post the JSON and wait for the plugin's answer. A failed or hung read is
/// no answer and never the end of the wait: the sim reloading the aircraft
/// answers the Web API with errors for a while, and the result is the next
/// good read. The baseline counter is read FIRST (retried): without it the
/// last flight's pair (seq 0, result 0) reads as success before this one ran.
pub async fn run_init(
    mb: &dyn InitMailbox,
    body: &[u8],
    t: InitTiming,
    report: &(dyn Fn(InitWait) + Sync),
) -> InitRun {
    let read = |name: &'static str| async move {
        match tokio::time::timeout(t.read_timeout, mb.read(name)).await {
            Ok(Ok(v)) => Some(v),
            _ => None,
        }
    };
    let mut run = InitRun { outcome: InitOutcome::TimedOut, read_errors: 0, polls: 0 };
    let mut seq0 = None;
    for _ in 0..10 {
        match read(INIT_SEQ).await {
            Some(v) => {
                seq0 = Some(v);
                break;
            }
            None => {
                run.read_errors += 1;
                tokio::time::sleep(t.poll).await;
            }
        }
    }
    let Some(seq0) = seq0 else {
        run.outcome = InitOutcome::NoBaseline;
        return run;
    };
    // Our own run: the new-flight watch stands aside while it is out, and
    // its number is ours once answered (unanswered, the next one is).
    use std::sync::atomic::Ordering::SeqCst;
    OWN_INITS.fetch_add(1, SeqCst);
    let (run, seq) = run_posted(mb, body, t, report, run, seq0).await;
    KNOWN_INIT_SEQ.fetch_max(seq.unwrap_or(seq0 as i64 + 1), SeqCst);
    OWN_INITS.fetch_sub(1, SeqCst);
    run
}

/// [`run_init`] from the post on, with the counter it saw answered.
async fn run_posted(
    mb: &dyn InitMailbox,
    body: &[u8],
    t: InitTiming,
    report: &(dyn Fn(InitWait) + Sync),
    mut run: InitRun,
    seq0: f64,
) -> (InitRun, Option<i64>) {
    let read = |name: &'static str| async move {
        match tokio::time::timeout(t.read_timeout, mb.read(name)).await {
            Ok(Ok(v)) => Some(v),
            _ => None,
        }
    };
    let posted = tokio::time::timeout(std::time::Duration::from_secs(5), mb.post(body)).await;
    if !matches!(posted, Ok(Ok(()))) {
        run.outcome = InitOutcome::Unreachable;
        return (run, Some(seq0 as i64));
    }
    let t0 = tokio::time::Instant::now();
    let mut reported = t0;
    while t0.elapsed() < t.deadline {
        tokio::time::sleep(t.poll).await;
        run.polls += 1;
        let (seq, res) = tokio::join!(read(INIT_SEQ), read(INIT_RESULT));
        if let (Some(seq), Some(res)) = (seq, res) {
            if seq > seq0 && res >= 0.0 {
                run.outcome = InitOutcome::Done(res as i64);
                return (run, Some(seq as i64));
            }
        }
        run.read_errors += u32::from(seq.is_none()) + u32::from(res.is_none());
        if reported.elapsed() >= t.report_every {
            reported = tokio::time::Instant::now();
            report(InitWait { seq0, seq, result: res, read_errors: run.read_errors, polls: run.polls });
        }
    }
    (run, None)
}

/// Did this init start a new flight? Answered, or written and never
/// answered (the sim may well have moved us): yes. Refused, or never written
/// (no baseline, plugin unreachable): the old flight goes on. Pure.
pub fn new_flight_began(outcome: &InitOutcome) -> bool {
    matches!(outcome, InitOutcome::Done(0) | InitOutcome::TimedOut)
}

async fn approach_follow_up(job: ApproachJob, loading: Loading) {
    let xp = crate::xplane::XPlaneClient::new(job.base.clone());
    let say = |line: &str| {
        if let Some(v) = &job.voice {
            v.alert(Priority::Caution, line.to_string());
        }
    };
    let t0 = std::time::Instant::now();
    let run = run_init(&xp, job.body.as_bytes(), InitTiming::LIVE, &|w| {
        crate::blackbox::record(
            "setup",
            json!({ "event": "approach_wait", "seq": w.seq, "result": w.result, "seq0": w.seq0,
                    "read_errors": w.read_errors, "polls": w.polls,
                    "ms": t0.elapsed().as_millis() as u64 }),
        );
    })
    .await;
    let seen = std::time::Instant::now();
    let result = match run.outcome {
        InitOutcome::Done(code) => Some(code),
        _ => None,
    };
    let next = match run.outcome {
        InitOutcome::Unreachable | InitOutcome::NoBaseline => AfterInit::Failed {
            line: "I couldn't reach the plugin to set up the approach. Is it up to date?".into(),
        },
        _ => after_init(
            result,
            &job.icao,
            &job.runway,
            job.final_nm,
            job.hand_fly,
            job.land.is_some(),
        ),
    };
    // The flight that ended takes its clearances with it, BEFORE anything
    // arms: autoland reads the party line on its first pass, and at LEMG
    // (sortie 1790584266-approach, 2026-09-28) it read the last case's
    // "cleared to land 12" 1.3 s into a new 10 nm final for 13.
    // The station is rebuilt for the field the flight was put at, armed here
    // with the retirement so the main loop installs it while autoland arms.
    let mut tower = None;
    if new_flight_began(&run.outcome) {
        // The door: loops, facts, the old checklist and (ATC on) the party
        // line; with ATC off the station is uninstalled, its party line
        // already retired.
        crate::flight_state::flight_begins(crate::flight_state::NewFlight::Approach);
        if !matches!(next, AfterInit::Failed { .. }) && crate::mode::atc_enabled() {
            tower = crate::station::arm_new_flight(&job.icao, std::time::Instant::now());
        }
    }
    // The arm first, the record and the words after: the aircraft is
    // already flying.
    if let AfterInit::Arm { runway, .. } = &next {
        if let Some((cfg, ias)) = job.land.clone() {
            // The landing is a sortie: one file and one verdict for the
            // sortie runner, begun before autoland's first tick.
            let sortie = crate::full_auto::SortieGuard::begin(
                "approach",
                json!({ "airport": job.icao, "runway": job.runway, "final_nm": job.final_nm }),
            );
            hold_sortie_while(&crate::autoland::ACTIVE, sortie, SortieHold::LIVE);
            // The new flight's landing, launched by the setup alone: no
            // parent (a stand-down takes it through autoland's own run).
            crate::autoland::run_autoland_scoped(
                job.base.clone(),
                job.apt.clone(),
                cfg,
                ias,
                job.voice.clone(),
                None,
                Some(runway.clone()),
                Vec::new(),
                crate::autoland::HandDown::default(),
            );
            crate::flight::note_crew_takeover("setup_approach");
        }
    }
    // Armed (or not): the new flight exists and is somebody's.
    drop(loading);
    // Always on file and always said: a setup that ends in silence is the
    // one the pilot cannot act on. `arm_ms` is result-seen to armed.
    crate::blackbox::record(
        "setup",
        json!({ "event": "approach_result", "outcome": run.outcome.name(), "result": result,
                "armed": matches!(next, AfterInit::Arm { .. }),
                "arm_ms": seen.elapsed().as_millis() as u64,
                "read_errors": run.read_errors, "polls": run.polls,
                "ms": t0.elapsed().as_millis() as u64 }),
    );
    match &next {
        AfterInit::Arm { line, .. } | AfterInit::Monitor { line } | AfterInit::Failed { line } => {
            say(line)
        }
    }
    if matches!(next, AfterInit::Failed { .. }) || !crate::mode::atc_enabled() {
        return;
    }
    new_flight_radio(&xp, &job, tower).await;
}

/// The tower hears a NEW arrival: its station for this field is rebuilt from
/// nothing, COM1 goes to it, the field's own wind is published, and once the
/// new station is up the final report goes out (the landing is cleared; at
/// LEMG it earned "you landed without clearance").
async fn new_flight_radio(xp: &crate::xplane::XPlaneClient, job: &ApproachJob, tower: Option<i64>) {
    let Some(khz) = tower else {
        crate::blackbox::record(
            "setup",
            json!({ "event": "approach_radio", "icao": job.icao, "tower": null }),
        );
        return;
    };
    let _ = xp.set_float(crate::datarefs::COM1_FREQ_KHZ, khz as f64).await;
    if let Some((lat, lon)) = job.field {
        let wind = crate::weather::surface_wind(&crate::config::xplane_root(), &job.icao);
        crate::station::set_surface_wind(lat, lon, wind);
    }
    // The call belongs to the NEW station: routed a pass early it reaches
    // the old one and its worksheet.
    let up = await_station_up(khz, STATION_UP_S).await;
    let callsign = crate::crew_radio::crew_callsign(xp).await;
    let words = format!(
        "{} mile final runway {}",
        job.final_nm,
        crate::crew_radio::spell_runway(&job.runway)
    );
    if up {
        crate::crew_radio::transmit(crate::crew_radio::radio_call(&callsign, &words));
    }
    crate::blackbox::record(
        "setup",
        json!({ "event": "approach_radio", "icao": job.icao, "tower": khz,
                "station_up": up, "call": if up { Some(words) } else { None } }),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_approach_sortie_lasts_as_long_as_the_landing() {
        use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
        static FLYING: AtomicBool = AtomicBool::new(false);
        let t = SortieHold {
            start_within: std::time::Duration::from_millis(500),
            poll: std::time::Duration::from_millis(5),
        };
        let guard = crate::full_auto::SortieGuard::begin("approach_test", json!({}));
        let id = crate::blackbox::sortie().expect("begun");
        let h = hold_sortie_while(&FLYING, guard, t);
        // The loop comes up late (its own thread), then flies a while.
        std::thread::sleep(std::time::Duration::from_millis(50));
        FLYING.store(true, SeqCst);
        std::thread::sleep(std::time::Duration::from_millis(600));
        assert_eq!(crate::blackbox::sortie(), Some(id), "open while the loop flies");
        FLYING.store(false, SeqCst);
        h.join().unwrap();
        assert!(crate::blackbox::sortie().is_none(), "closed when it lets go");
        // A loop that never started does not hold a sortie open.
        let guard = crate::full_auto::SortieGuard::begin("approach_test", json!({}));
        hold_sortie_while(&FLYING, guard, t).join().unwrap();
        assert!(crate::blackbox::sortie().is_none());
    }

    #[test]
    fn an_acf_declares_its_type() {
        let text = "I\n1200 Version\nP acf/_name Boeing 737-800\nP acf/_ICAO b738\n";
        assert_eq!(acf_icao(text), Some("B738".into()));
        assert_eq!(acf_icao("P acf/_name x\n"), None);
    }

    #[test]
    fn the_plans_type_finds_the_laminar_aircraft_first() {
        let lib = vec![
            ("B738".to_string(), "Aircraft/Zibo/B737-800X/b738_4k.acf".to_string()),
            ("B738".to_string(), "Aircraft/Laminar Research/Boeing 737-800/b738.acf".to_string()),
            ("C172".to_string(), "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf".to_string()),
        ];
        assert_eq!(find_acf(&lib, "b738"), Some("Aircraft/Laminar Research/Boeing 737-800/b738.acf"));
        assert_eq!(find_acf(&lib, "A320"), None);
    }

    #[test]
    fn a_jet_gets_a_gate_that_takes_jets_and_a_prop_a_tie_down() {
        let st = |name: &str, kind: &str, accepts: &[&str]| crate::nav::Stand {
            lat: 0.0,
            lon: 0.0,
            heading: 0.0,
            name: name.into(),
            kind: kind.into(),
            accepts: accepts.iter().map(|s| s.to_string()).collect(),
        };
        let stands = vec![
            st("H1", "hangar", &["jets"]),
            st("GA 2", "tie_down", &["props"]),
            st("B4", "gate", &["jets", "heavy"]),
            st("A1", "gate", &["props"]),
        ];
        assert_eq!(pick_stand(&stands, stand_class(true, false), true).unwrap().name, "B4");
        assert_eq!(pick_stand(&stands, stand_class(false, false), false).unwrap().name, "GA 2");
    }

    fn rwy(a: &str, b: &str, from: (f64, f64), to: (f64, f64), surface: u32) -> crate::nav::Runway {
        crate::nav::Runway {
            ident_a: a.into(),
            ident_b: b.into(),
            lat_a: from.0,
            lon_a: from.1,
            lat_b: to.0,
            lon_b: to.1,
            width_m: 45.0,
            surface,
        }
    }

    /// Malaga's two strips, as apt.dat spells them (headings about 130/310
    /// and 120/300).
    fn lemg() -> Vec<crate::nav::Runway> {
        vec![
            rwy("13", "31", (36.700, -4.510), (36.680, -4.480), 1),
            rwy("12", "30", (36.690, -4.525), (36.672, -4.490), 1),
        ]
    }

    /// A new flight retires the old one's clearances before the arm (LEMG,
    /// sortie 1790584266-approach): answered, or written and never
    /// answered, the sim may have moved us; refused or never written, the
    /// old flight and its tower go on.
    #[test]
    fn a_new_flight_is_one_the_sim_may_have_started() {
        assert!(new_flight_began(&InitOutcome::Done(0)));
        assert!(new_flight_began(&InitOutcome::TimedOut));
        assert!(!new_flight_began(&InitOutcome::Done(3)));
        assert!(!new_flight_began(&InitOutcome::Unreachable));
        assert!(!new_flight_began(&InitOutcome::NoBaseline));
    }

    #[test]
    fn the_approach_init_is_this_aircraft_on_a_final_with_engines_running() {
        // The JSON driven by hand at LEMG on 2026-09-26 (seq 1, result 0).
        let body = approach_init_json(
            "Aircraft/Laminar Research/Boeing 737-800/b738.acf",
            "LEMG",
            "13",
            10,
        );
        let v: Value = serde_json::from_str(&body).unwrap();
        assert_eq!(v["aircraft"]["path"], "Aircraft/Laminar Research/Boeing 737-800/b738.acf");
        assert_eq!(v["runway_start"]["airport_id"], "LEMG");
        assert_eq!(v["runway_start"]["runway"], "13");
        assert_eq!(v["runway_start"]["final_distance_in_nautical_miles"], 10);
        assert_eq!(v["engine_status"]["all_engines"]["running"], true);
        // Never a ramp start, never cold and dark.
        assert!(v.get("ramp_start").is_none());
        assert!(body.contains("\"final_distance_in_nautical_miles\":10"), "{body}");
    }

    #[test]
    fn a_refused_setup_names_the_reason() {
        assert_eq!(
            init_refusal(4, "LEMG", "13"),
            "The approach setup didn't take: X-Plane doesn't know the airport L E M G."
        );
        assert_eq!(
            init_refusal(6, "LEMG", "13"),
            "The approach setup didn't take: X-Plane has no runway one three at L E M G."
        );
        assert_eq!(
            init_refusal(2, "LEMG", "13"),
            "The approach setup didn't take: X-Plane couldn't load this aircraft."
        );
        assert!(init_refusal(9, "LEMG", "13").contains("code 9"));
    }

    #[test]
    fn the_copilot_takes_the_new_flight_on_the_runway_it_was_started_on() {
        // LEMG 2026-09-26: forty seconds with nobody flying. On result 0 the
        // arm is autoland FORCED onto the field and runway of the new final
        // (the arrival's `PlannedRunway`, never a proximity self-pick).
        assert_eq!(
            after_init(Some(0), "LEMG", "13", 10, false, true),
            AfterInit::Arm {
                runway: crate::runway::PlannedRunway::at("LEMG", "13"),
                line: "On a 10 mile final for one three, I have the aircraft.".into(),
            }
        );
        // "...my controls": monitoring only.
        assert_eq!(
            after_init(Some(0), "LEMG", "13", 5, true, true),
            AfterInit::Monitor {
                line: "On a 5 mile final for one three. Your aircraft, I'm monitoring.".into()
            }
        );
        // No autopilot to land with: said, never pretended.
        assert!(matches!(
            after_init(Some(0), "LEMG", "13", 10, false, false),
            AfterInit::Monitor { .. }
        ));
        // A refusal arms nothing.
        assert_eq!(
            after_init(Some(6), "LEMG", "13", 10, false, true),
            AfterInit::Failed { line: init_refusal(6, "LEMG", "13") }
        );
        assert!(matches!(
            after_init(None, "LEMG", "13", 10, false, true),
            AfterInit::Failed { .. }
        ));
    }

    #[test]
    fn the_runway_is_the_pilots_then_the_towers_then_the_plans_then_the_winds() {
        let r = lemg();
        assert_eq!(
            pick_runway("LEMG", &r, Some("13"), Some("12"), Some("30"), None).unwrap(),
            ("13".into(), "asked")
        );
        assert_eq!(
            pick_runway("LEMG", &r, None, Some("12"), Some("30"), Some((310.0, 15.0))).unwrap(),
            ("12".into(), "tower")
        );
        assert_eq!(
            pick_runway("LEMG", &r, None, None, Some("30"), Some((130.0, 15.0))).unwrap(),
            ("30".into(), "plan")
        );
        let (end, from) = pick_runway("LEMG", &r, None, None, None, Some((310.0, 15.0))).unwrap();
        assert!(end == "31" || end == "30", "{end}");
        assert_eq!(from, "wind");
        // A tower or plan runway the field lacks falls through, never refuses.
        assert_eq!(
            pick_runway("LEMG", &r, None, Some("27"), None, None).unwrap().1,
            "longest"
        );
        // "3" is "03", "13" is not "13L".
        assert!(same_end("03", "3"));
        assert!(!same_end("13", "13L"));
    }

    #[test]
    fn a_runway_the_field_lacks_is_refused_with_the_ones_it_has() {
        let err = pick_runway("LEMG", &lemg(), Some("14"), None, None, None).unwrap_err();
        assert_eq!(
            err,
            "L E M G has no runway one four. It has one three, three one, one two, three zero."
        );
    }

    /// The mailbox through a reload: reads fail (or hang) for a while after
    /// the post, then the plugin's answer is there.
    struct Reloading {
        failing: std::sync::atomic::AtomicU32,
        hanging: std::sync::atomic::AtomicU32,
        /// Result reads after the post that still show "pending".
        pending: std::sync::atomic::AtomicU32,
        posted: std::sync::atomic::AtomicBool,
        answer: i64,
        post_fails: bool,
    }

    impl Reloading {
        fn new(failing: u32, hanging: u32, pending: u32, answer: i64) -> Self {
            Reloading {
                failing: failing.into(),
                hanging: hanging.into(),
                pending: pending.into(),
                posted: false.into(),
                answer,
                post_fails: false,
            }
        }
    }

    #[async_trait]
    impl InitMailbox for Reloading {
        async fn read(&self, name: &str) -> Result<f64> {
            use std::sync::atomic::Ordering::SeqCst;
            let posted = self.posted.load(SeqCst);
            if posted && self.hanging.load(SeqCst) > 0 {
                self.hanging.fetch_sub(1, SeqCst);
                tokio::time::sleep(std::time::Duration::from_secs(30)).await;
            }
            if posted && self.failing.load(SeqCst) > 0 {
                self.failing.fetch_sub(1, SeqCst);
                anyhow::bail!("503 mid-reload");
            }
            let done = posted && self.pending.load(SeqCst) == 0;
            if posted && !done && name == INIT_RESULT {
                self.pending.fetch_sub(1, SeqCst);
            }
            Ok(match (name, posted, done) {
                (INIT_SEQ, _, true) => 1.0,
                (INIT_SEQ, _, false) => 0.0,
                (_, _, true) => self.answer as f64,
                (_, true, false) => -1.0,
                // The last flight's answer: success, seq 0.
                _ => 0.0,
            })
        }
        async fn post(&self, _body: &[u8]) -> Result<()> {
            if self.post_fails {
                anyhow::bail!("no plugin");
            }
            self.posted.store(true, std::sync::atomic::Ordering::SeqCst);
            Ok(())
        }
    }

    const FAST: InitTiming = InitTiming {
        poll: std::time::Duration::from_millis(5),
        read_timeout: std::time::Duration::from_millis(40),
        deadline: std::time::Duration::from_secs(2),
        report_every: std::time::Duration::from_millis(20),
    };

    fn quiet(_: InitWait) {}

    #[tokio::test]
    async fn the_result_is_read_through_a_reload_that_answers_errors() {
        // A failed read is one more poll, never the end of the wait.
        let mb = Reloading::new(12, 0, 3, 0);
        let run = run_init(&mb, b"{}", FAST, &quiet).await;
        assert_eq!(run.outcome, InitOutcome::Done(0));
        assert!(run.read_errors >= 12, "{run:?}");
    }

    #[tokio::test]
    async fn a_read_that_hangs_through_the_reload_does_not_hold_the_arm_back() {
        let mb = Reloading::new(0, 4, 1, 0);
        let t0 = std::time::Instant::now();
        let run = run_init(&mb, b"{}", FAST, &quiet).await;
        assert_eq!(run.outcome, InitOutcome::Done(0));
        assert!(t0.elapsed() < std::time::Duration::from_secs(1), "{:?}", t0.elapsed());
        // Live: a hung read plus a poll is under a second.
        assert!(
            (InitTiming::LIVE.read_timeout + InitTiming::LIVE.poll)
                < std::time::Duration::from_secs(1)
        );
    }

    #[tokio::test]
    async fn the_last_flights_answer_is_never_this_ones_and_a_stuck_wait_reports() {
        // Before the post the mailbox reads seq 0, result 0: success from
        // the flight before. Never taken; the wait says what it sees.
        let mb = Reloading::new(0, 0, u32::MAX, 0);
        let seen = std::sync::Mutex::new(Vec::new());
        let t = InitTiming { deadline: std::time::Duration::from_millis(150), ..FAST };
        let run = run_init(&mb, b"{}", t, &|w| seen.lock().unwrap().push(w)).await;
        assert_eq!(run.outcome, InitOutcome::TimedOut);
        let seen = seen.into_inner().unwrap();
        assert!(seen.len() >= 3, "{}", seen.len());
        assert_eq!(seen[0].seq0, 0.0);
        assert_eq!(seen[0].seq, Some(0.0));
        assert_eq!(seen[0].result, Some(-1.0), "pending while the plugin has it");
    }

    #[tokio::test]
    async fn a_refusal_and_an_unreachable_plugin_are_outcomes() {
        let mb = Reloading::new(0, 0, 0, 6);
        assert_eq!(run_init(&mb, b"{}", FAST, &quiet).await.outcome, InitOutcome::Done(6));
        let mut dead = Reloading::new(0, 0, 0, 0);
        dead.post_fails = true;
        let run = run_init(&dead, b"{}", FAST, &quiet).await;
        assert_eq!(run.outcome.name(), "unreachable");
        assert_eq!(InitOutcome::TimedOut.name(), "timeout");
        assert_eq!(InitOutcome::Done(0).name(), "ok");
        assert_eq!(InitOutcome::Done(6).name(), "refused");
    }

    /// #134: the watch acts only on a run nobody here posted, once it has an
    /// answer. The first read is the baseline (a flight loaded before boot
    /// is no new flight); a run still pending (-1) waits; a refusal (6) made
    /// no flight.
    #[test]
    fn the_watch_acts_on_a_new_flight_someone_else_started() {
        let (seen, known) = judge_init_seq(i64::MIN, 8, 0);
        assert_eq!((seen, known), (InitSeen::Nothing, 8));
        assert_eq!(judge_init_seq(8, 8, 0), (InitSeen::Nothing, 8));
        assert_eq!(judge_init_seq(8, 9, -1), (InitSeen::Nothing, 8), "pending");
        assert_eq!(
            judge_init_seq(8, 9, 0),
            (InitSeen::Foreign { new_flight: true }, 9)
        );
        assert_eq!(
            judge_init_seq(8, 9, 6),
            (InitSeen::Foreign { new_flight: false }, 9)
        );
        // An older number (a plugin reload restarted the counter) is no run.
        assert_eq!(judge_init_seq(9, 0, 0), (InitSeen::Nothing, 9));
    }

    /// The app's own run accounts for its own number: the watch never takes
    /// it for someone else's (it would stand down the autoland the approach
    /// setup just armed).
    #[tokio::test]
    async fn the_apps_own_run_is_never_foreign() {
        use std::sync::atomic::Ordering::SeqCst;
        let mb = Reloading::new(0, 0, 1, 0);
        let run = run_init(&mb, b"{}", FAST, &quiet).await;
        assert_eq!(run.outcome, InitOutcome::Done(0));
        let (seen, _) = judge_init_seq(KNOWN_INIT_SEQ.load(SeqCst), 1, 0);
        assert_eq!(seen, InitSeen::Nothing);
        // Unanswered, the next number is presumed ours too.
        let mb = Reloading::new(0, 0, u32::MAX, 0);
        let t = InitTiming { deadline: std::time::Duration::from_millis(60), ..FAST };
        assert_eq!(run_init(&mb, b"{}", t, &quiet).await.outcome, InitOutcome::TimedOut);
        let (seen, _) = judge_init_seq(KNOWN_INIT_SEQ.load(SeqCst), 1, 0);
        assert_eq!(seen, InitSeen::Nothing);
    }

    /// While the app's own new flight loads, a mission's arm is refused in
    /// words; the flag drops with the follow-up. And the full-auto tool asks
    /// before anything else it does (LEMG sortie 1790584476-full_auto: "fly a
    /// circuit" armed 0.6 s after "put us on a 5 mile final").
    #[test]
    fn nothing_arms_while_a_new_flight_loads() {
        {
            let _loading = Loading::begin();
            assert!(new_flight_loading());
            let line = new_flight_loading_refusal().expect("refused while loading");
            assert!(line.contains("still loading"), "{line}");
        }
        let src = crate::test_support::module_source("tools/full_auto");
        let refusal = src
            .find("new_flight_loading_refusal()")
            .expect("the full-auto tool asks");
        let running = src
            .find("crate::full_auto::running()")
            .expect("the running check");
        assert!(refusal < running, "asked before anything arms");
        // The approach follow-up holds the flag until it has armed.
        let this = crate::test_support::module_source("tools/setup");
        let arm = this.find("crate::autoland::run_autoland_scoped(").unwrap();
        let drop_at = this.find("drop(loading);").unwrap();
        assert!(arm < drop_at, "the flag drops after the arm");
    }

    fn at(t: f64, lat: f64, lon: f64, agl_ft: f64, on_ground: bool, gs_mps: f64) -> PosSample {
        PosSample { t, lat, lon, agl_ft, on_ground, gs_mps }
    }

    /// EGKK, 2026-09-28 (#151): the recorded load. The sim said "on the
    /// ground" at runway 08R/26L's midpoint 157 ft up for 16 s, then stand 71
    /// at the terrain. The station is chosen only from stand 71.
    #[test]
    fn a_new_flight_is_placed_only_once_its_position_settles() {
        let mut j = SettleJudge::default();
        // The load's transient: plausible nowhere, however still.
        for k in 0..40 {
            let t = 1509.3 + 0.4 * f64::from(k);
            assert_eq!(j.feed(at(t, 51.14829935, -0.1891522, 157.1, true, 0.0)), None, "t {t}");
        }
        // Stand 71: settles once 2 s of it agree.
        let stand = [
            (1531.6, 51.15539550997997, -0.16536942320749573, -0.0, 0.0),
            (1531.9, 51.15539550997997, -0.16536942320749573, -0.0, 0.0),
            (1532.3, 51.15539550997997, -0.16536942320749573, -0.0, 0.0),
            (1532.7, 51.15539550997997, -0.16536942320749573, -0.0, 0.0),
            (1533.0, 51.15539550997997, -0.16536942320749573, -0.0, 0.0),
        ];
        for (t, lat, lon, agl, gs) in stand {
            assert_eq!(j.feed(at(t, lat, lon, agl, true, gs)), None, "t {t}");
        }
        let fix = j
            .feed(at(1534.4, 51.155395652708314, -0.16536949061987397, 0.0, true, 0.1))
            .expect("settled on the stand");
        assert!((fix.lat - 51.1554).abs() < 1e-3 && (fix.lon + 0.1654).abs() < 1e-3, "{fix:?}");
        assert!(fix.on_ground);

        // LEMG #134: a final's transient at -84 ft AGL is no position; the
        // final itself, flying 70 m/s, settles.
        let mut j = SettleJudge::default();
        for k in 0..5 {
            let t = f64::from(k) * 0.5;
            assert_eq!(j.feed(at(t, 36.68532005, -4.49416975, -84.0, false, 0.0)), None);
        }
        let mut fix = None;
        for k in 0..6 {
            let t = 10.0 + f64::from(k) * 0.5;
            // 35 m a read along the final, about 0.000315 degrees of latitude.
            let lat = 36.60 + 0.000315 * f64::from(k);
            fix = fix.or(j.feed(at(t, lat, -4.55, 2_900.0, false, 70.0)));
        }
        assert!(fix.is_some_and(|f| !f.on_ground));

        // A jump is a new place, never a movement: the run starts again.
        let mut j = SettleJudge::default();
        assert_eq!(j.feed(at(0.0, 36.6716, -4.4890, 0.1, true, 0.0)), None);
        assert_eq!(j.feed(at(1.0, 36.6716, -4.4890, 0.1, true, 0.0)), None);
        assert_eq!(j.feed(at(1.5, 51.1554, -0.1654, 0.0, true, 0.0)), None);
        assert_eq!(j.feed(at(2.5, 51.1554, -0.1654, 0.0, true, 0.0)), None, "1 s of the new place");
        assert!(j.feed(at(3.5, 51.1554, -0.1654, 0.0, true, 0.0)).is_some());
    }

    /// Every new flight resolves its station from where it now is: the watch
    /// never re-arms the field the station happened to be working, and the
    /// ramp setup rebuilds it before its flag drops (#151).
    #[test]
    fn a_new_flight_takes_its_station_from_its_new_position() {
        let src = crate::test_support::module_source("tools/setup");
        let body = |name: &str| {
            let a = src.find(name).unwrap();
            let b = a + src[a..].find("\n}\n").unwrap();
            &src[a..b]
        };
        let foreign = body("async fn foreign_new_flight(");
        assert!(!foreign.contains(&["active", "_icao()"].concat()), "never the active field");
        assert!(foreign.contains("settled_position(") && foreign.contains("new_flight_station("));
        assert!(foreign.contains("Loading::begin()"), "the arm waits while it resolves");
        let ramp = body("async fn follow_up(");
        let station = ramp.find("new_flight_station(").expect("the ramp setup resolves it");
        let typed = ramp.find("load the flight plan").unwrap();
        assert!(station < typed, "up before anything is asked of the new flight");
        let resolve = body("pub async fn new_flight_station(");
        assert!(resolve.contains("arm_new_flight_at(fix.lat, fix.lon"));
    }

    #[test]
    fn a_follow_up_runs_while_the_command_loop_blocks() {
        // The live failure reproduced: on a current-thread runtime that
        // blocks (the command loop on the mic), a `tokio::spawn`ed task does
        // not run; one on its own thread does.
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        let starved = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let ran = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        rt.block_on(async {
            let s = starved.clone();
            tokio::spawn(async move { s.store(true, std::sync::atomic::Ordering::SeqCst) });
            let r = ran.clone();
            spawn_off_loop(async move { r.store(true, std::sync::atomic::Ordering::SeqCst) });
            std::thread::sleep(std::time::Duration::from_millis(300));
            assert!(!starved.load(std::sync::atomic::Ordering::SeqCst));
            assert!(ran.load(std::sync::atomic::Ordering::SeqCst), "the follow-up starved");
        });
    }

    #[test]
    fn no_tool_leaves_work_on_the_command_loops_runtime() {
        // Tools run on the command loop's current-thread runtime, which
        // blocks on the microphone: work that must outlive the call goes on
        // `spawn_off_loop` (or its own thread), never `tokio::spawn`. Test
        // modules may spawn (a test runtime is theirs); comments may name it.
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/tools");
        let mut offenders = Vec::new();
        for p in crate::test_support::rust_files(&dir) {
            if crate::test_support::is_test_file(&p) {
                continue;
            }
            let text = std::fs::read_to_string(&p).unwrap();
            let live = text.split("#[cfg(test)]").next().unwrap_or("");
            for (i, line) in live.lines().enumerate() {
                let code = line.split("//").next().unwrap_or("");
                if code.contains("tokio::spawn(") {
                    offenders.push(format!("{}:{}", p.display(), i + 1));
                }
            }
        }
        assert!(offenders.is_empty(), "tokio::spawn from a tool starves: {offenders:?}");
    }

    #[test]
    fn the_init_is_the_aircraft_at_the_ramp_cold_and_dark() {
        let v: Value = serde_json::from_str(&init_json("Aircraft/x.acf", "LIBP", "Stand 1")).unwrap();
        assert_eq!(v["aircraft"]["path"], "Aircraft/x.acf");
        assert_eq!(v["ramp_start"]["airport_id"], "LIBP");
        assert_eq!(v["ramp_start"]["ramp"], "Stand 1");
        assert_eq!(v["engine_status"]["all_engines"]["running"], false);
    }
}
