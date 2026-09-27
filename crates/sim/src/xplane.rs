//! Minimal client for the X-Plane 12 Web API.
//!
//! The Web API addresses datarefs by a numeric id, not by name, so we resolve
//! `name -> id` once via the index endpoint and cache it. Values are read and
//! written through `/api/v1/datarefs/{id}/value`, whose body is `{"data": <v>}`.
//!
//! This is the "REST first" path from the project brief — a separate process
//! talking to the same API Horizon polls. A native plugin is the latency endgame.

use std::collections::HashMap;

use anyhow::{anyhow, Context, Result};
use async_trait::async_trait;
use base64::Engine as _;
use serde::Deserialize;
use serde_json::json;
use tokio::sync::Mutex;

/// Web API requests can transiently fail on a busy sim frame (a timeout, a dropped
/// connection, a 5xx). Every dataref op here is idempotent — reads, and writes
/// that set an absolute value — so retrying a couple of times is safe and stops a
/// single hiccup from failing a trust-critical action (the parking-brake case: one
/// PATCH failed, the identical retry worked).
const HTTP_ATTEMPTS: usize = 3;
const HTTP_RETRY_DELAY: std::time::Duration = std::time::Duration::from_millis(60);

/// How often the request-rate accounting reports, seconds.
const LOAD_REPORT_SECS: f64 = 30.0;

/// How long a dataref VALUE may be reused before it is fetched again.
///
/// The Web API is served from inside X-Plane's flight loop, so every request is
/// frame time the sim does not get. Measured on 2026-07-28 we were sustaining
/// 70-80 requests a second, and the per-window slowest response climbed 50 ms →
/// 100 → 125 in the minutes before the sim wedged — the shape of load, not of a
/// single bad event. Most of that volume is DUPLICATION: the monitor's eighteen
/// checks run in one burst and several want the same airspeed and altitude, and
/// the control loops re-read the position datarefs the monitor just read.
///
/// A window this short is invisible to the laws — every control loop ticks at
/// 150 ms or slower — but it collapses a burst of duplicate reads into one
/// request. Writes invalidate their own entry (see `set_value`), so the
/// readback-after-write that every tool's trust rests on still sees the real
/// post-write value. `XCOPILOT_DREF_CACHE_MS=0` disables it.
fn value_cache_ttl() -> std::time::Duration {
    static MS: std::sync::OnceLock<u64> = std::sync::OnceLock::new();
    let ms = *MS.get_or_init(|| {
        std::env::var("XCOPILOT_DREF_CACHE_MS")
            .ok()
            .and_then(|v| v.trim().parse().ok())
            .unwrap_or(60)
    });
    std::time::Duration::from_millis(ms)
}

/// Datarefs that change almost never, and the longer window they get.
///
/// The 60 ms cache was sized for DUPLICATION — two loops wanting the same
/// airspeed in one burst — and measured, it recovers about 2.5%, because the
/// loops mostly read DIFFERENT datarefs. Volume, not duplication, is the
/// problem: a whole EGHI sortie logged `cached=0` in most windows while
/// sustaining 70-80 req/s, and the per-dataref histogram named the reason.
/// After latitude and longitude came `is_in_replay` (3,803) and `paused`
/// (3,791) — the freeze check, about four reads a second between them, for two
/// flags that change a handful of times in a whole flight.
///
/// Nothing needs those within 60 ms. A pause or a replay entry noticed half a
/// second late costs one control tick, which every loop already tolerates
/// (`sim_frozen` fails OPEN, and the laws it gates re-check every tick anyway).
/// Measured at 20:42 on 2026-08-01 the sim dropped to 7 fps against a 60 fps
/// median in the same 30 s window that logged 76 req/s and a 1,040 ms slowest
/// response — the API is served from inside X-Plane's flight loop, so this is
/// frame time the sim does not get.
const SLOW_DATAREFS: &[&str] = &[
    "sim/time/is_in_replay",
    "sim/time/paused",
    "sim/aircraft/view/acf_ICAO",
    "sim/aircraft/view/acf_tailnum",
    // The tug on the bar: a fact that changes over minutes, read by the
    // taxi tick as evidence (a re-hooked tug owns the nosewheel).
    "sim/aircraft/overflow/pushback_attached",
    // The sun moves over minutes; the light stages ask it once per decision.
    "sim/graphics/scenery/sun_pitch_degrees",
    // The local QNH and the altimeter knob: the monitor's cross-check reads
    // both every tick, and neither moves faster than a weather edition.
    "sim/weather/aircraft/qnh_pas",
    "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot",
];
/// How long a [`SLOW_DATAREFS`] value may be reused. Well under the time any
/// of them plausibly changes twice, and a whole order of magnitude off the
/// request rate they were driving.
const SLOW_DATAREF_TTL: std::time::Duration = std::time::Duration::from_millis(500);

/// The cache window for one dataref: the long one for the flags that never
/// move, the configured one for everything else. Disabling the cache
/// (`XCOPILOT_DREF_CACHE_MS=0`) disables both, so the escape hatch still works.
fn ttl_for(name: &str) -> std::time::Duration {
    let base = value_cache_ttl();
    if base.is_zero() {
        return base;
    }
    if SLOW_DATAREFS.contains(&name) {
        SLOW_DATAREF_TTL.max(base)
    } else {
        base
    }
}

/// Web API load over the current reporting window. The Web API is served from
/// inside X-Plane's own frame loop, so what we ask of it is a load the SIM
/// carries — and when the sim freezes, the only thing our logs record is the
/// failures that follow. Twice on 2026-07-28 the question "were we hammering
/// it?" could not be answered from the logs at all, because nothing ever
/// counted the requests that SUCCEEDED. Now the rate is on the record, in the
/// log and in the blackbox, whether or not anything goes wrong.
struct LoadWindow {
    start: std::time::Instant,
    requests: u64,
    failures: u64,
    slowest_ms: u64,
    /// Reads served from the value cache — requests the sim never saw.
    cache_hits: u64,
    /// Requests per dataref name, so a load report names the hogs. Cutting a
    /// rate needs to know what it is MADE of: the first cache attempt assumed
    /// duplication and recovered 2.5%, because the loops each read different
    /// datarefs rather than the same ones twice (2026-07-28).
    by_name: std::collections::HashMap<String, u64>,
}

static LOAD: std::sync::Mutex<Option<LoadWindow>> = std::sync::Mutex::new(None);

/// A read-only look at the load window in progress, for the desktop health
/// screen. The 30 s report tells the story AFTER the fact, in a log line and a
/// blackbox event; a freeze is watched in real time, so the same counters have
/// to be readable mid-window. Reading never resets the window — the report
/// still owns that.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LoadSnapshot {
    /// Seconds elapsed in the window so far (resets every `LOAD_REPORT_SECS`).
    pub window_s: f64,
    pub requests: u64,
    pub cache_hits: u64,
    pub failures: u64,
    pub slowest_ms: u64,
    /// The busiest datarefs so far this window, descending.
    pub top: Vec<(String, u64)>,
}

impl LoadSnapshot {
    /// Requests per second so far this window. Meaningless in the first
    /// fraction of a second, hence the floor.
    pub fn rate_per_s(&self) -> f64 {
        self.requests as f64 / self.window_s.max(0.25)
    }
}

/// Snapshot the current load window, or `None` before the first request.
pub fn load_snapshot() -> Option<LoadSnapshot> {
    let guard = LOAD.lock().unwrap_or_else(|e| e.into_inner());
    let w = guard.as_ref()?;
    let mut top: Vec<(String, u64)> = w.by_name.iter().map(|(n, c)| (n.clone(), *c)).collect();
    top.sort_by_key(|item| std::cmp::Reverse(item.1));
    top.truncate(6);
    Some(LoadSnapshot {
        window_s: w.start.elapsed().as_secs_f64(),
        requests: w.requests,
        cache_hits: w.cache_hits,
        failures: w.failures,
        slowest_ms: w.slowest_ms,
        top,
    })
}

/// A read served from cache: counted so the load report shows what the
/// window would have cost without it.
fn note_cache_hit() {
    let mut guard = LOAD.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(w) = guard.as_mut() {
        w.cache_hits += 1;
    }
}

/// Account for one attempt against the API — one actual trip to the socket,
/// which is what the sim pays for, not one logical operation.
/// Attribute one request to the dataref it was for.
fn note_named(name: &str) {
    let mut guard = LOAD.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(w) = guard.as_mut() {
        *w.by_name.entry(name.to_string()).or_insert(0) += 1;
    }
}

fn note_request(elapsed: std::time::Duration, ok: bool) {
    let mut guard = LOAD.lock().unwrap_or_else(|e| e.into_inner());
    let w = guard.get_or_insert_with(|| LoadWindow {
        start: std::time::Instant::now(),
        requests: 0,
        failures: 0,
        slowest_ms: 0,
        cache_hits: 0,
        by_name: std::collections::HashMap::new(),
    });
    w.requests += 1;
    w.failures += u64::from(!ok);
    w.slowest_ms = w.slowest_ms.max(elapsed.as_millis() as u64);
    let secs = w.start.elapsed().as_secs_f64();
    if secs < LOAD_REPORT_SECS {
        return;
    }
    let rate = w.requests as f64 / secs;
    tracing::info!(
        target: "sim::load",
        rate = format!("{rate:.0}/s"),
        requests = w.requests,
        cached = w.cache_hits,
        failures = w.failures,
        slowest_ms = w.slowest_ms,
        "web api load"
    );
    let mut top: Vec<(String, u64)> = w.by_name.drain().collect();
    top.sort_by_key(|item| std::cmp::Reverse(item.1));
    top.truncate(8);
    crate::blackbox::record(
        "api_load",
        serde_json::json!({
            "top": top.iter().map(|(n, c)| format!("{n} x{c}")).collect::<Vec<_>>(),
            "rate_per_s": (rate * 10.0).round() / 10.0,
            "requests": w.requests,
            "cache_hits": w.cache_hits,
            "failures": w.failures,
            "slowest_ms": w.slowest_ms,
            "window_s": (secs * 10.0).round() / 10.0,
        }),
    );
    *w = LoadWindow {
        start: std::time::Instant::now(),
        requests: 0,
        failures: 0,
        slowest_ms: 0,
        cache_hits: 0,
        by_name: std::collections::HashMap::new(),
    };
}

/// Run an idempotent Web API request, retrying a few times on failure. A 404 is
/// *not* retried: it means the dataref id is stale (X-Plane reassigns ids across
/// restarts and aircraft reloads), so re-issuing the same id just spams the log —
/// the caller drops the cached id and re-resolves instead (see [`is_not_found`]).
async fn with_retry<T, F, Fut>(op: F) -> Result<T>
where
    F: Fn() -> Fut,
    Fut: std::future::Future<Output = Result<T>>,
{
    let mut attempt = 0;
    loop {
        attempt += 1;
        let started = std::time::Instant::now();
        let result = op().await;
        note_request(started.elapsed(), result.is_ok());
        match result {
            Ok(v) => return Ok(v),
            Err(e) if attempt < HTTP_ATTEMPTS && !is_not_found(&e) => {
                tracing::warn!(attempt, "X-Plane request failed, retrying: {e:#}");
                tokio::time::sleep(HTTP_RETRY_DELAY).await;
            }
            Err(e) => return Err(e),
        }
    }
}

/// True when an error is a Web API 404 (Not Found). The API 404s a dataref id
/// that's gone stale — it reassigns numeric ids across X-Plane restarts and
/// aircraft reloads — so a cached handle stops resolving until we drop it.
fn is_not_found(err: &anyhow::Error) -> bool {
    err.chain().any(|e| {
        e.downcast_ref::<reqwest::Error>()
            .and_then(reqwest::Error::status)
            == Some(reqwest::StatusCode::NOT_FOUND)
    })
}

/// The dataref/command surface a tool uses to talk to the simulator. Abstracting
/// it behind a trait lets the trust-critical tool logic (autopilot capture, safety
/// gates) be unit-tested against a fake, with no running X-Plane. The live impl is
/// [`XPlaneClient`]; tests use a recording fake. (Index/string helpers like
/// `aircraft_icao` stay inherent to the client — only the loop, not the tools,
/// needs them.)
#[async_trait]
pub trait XPlane: Send + Sync {
    fn cancellation(&self) -> Vec<std::sync::Arc<std::sync::atomic::AtomicBool>> {
        Vec::new()
    }
    async fn get_float(&self, name: &str) -> Result<f64>;
    async fn set_float(&self, name: &str, value: f64) -> Result<()>;
    async fn set_int(&self, name: &str, value: i64) -> Result<()>;
    async fn run_command(&self, name: &str) -> Result<()>;
    async fn run_command_for(&self, name: &str, duration: f64) -> Result<()>;
    async fn get_float_array(&self, name: &str) -> Result<Vec<f64>>;
    async fn set_int_array_elem(&self, name: &str, index: usize, value: i64) -> Result<()>;
    async fn set_float_array_elem(&self, name: &str, index: usize, value: f64) -> Result<()>;
}

/// True when what the sim shows is not the aircraft flying NOW: paused, or
/// playing a replay. Control loops freeze on it — no writes, no integration
/// (a P+D throttle law ticking against a frozen airspeed walks the lever to
/// its clamp and surges on unpause), no blackbox — and the monitor skips its
/// reactive tick. Fail open: a failed read means live.
pub async fn sim_frozen(xp: &dyn XPlane) -> bool {
    for dref in [crate::datarefs::IS_IN_REPLAY, crate::datarefs::PAUSED] {
        if xp.get_float(dref).await.is_ok_and(|v| v > 0.5) {
            return true;
        }
    }
    false
}

/// HTTP client for one running X-Plane instance.
pub struct XPlaneClient {
    http: reqwest::Client,
    base: String,
    cancel: Vec<std::sync::Arc<std::sync::atomic::AtomicBool>>,
}

/// The caches are PROCESS-WIDE, not per-client, keyed by base URL then name —
/// two clients on the same sim share every resolution and value, while tests
/// against distinct mock servers stay isolated.
///
/// They were per-instance until 2026-08-02, which quietly turned every TTL
/// into a per-LOOP cap: each control loop builds its own client on its own
/// thread, and the flight mission runs five at once (procedure follower,
/// cruise governor, monitor, session, region). `is_in_replay` + `paused` —
/// capped at 2/s each by the 500 ms TTL — still hit the sim at ~11/s
/// combined, inside a sortie sustaining 96 req/s while the sim dipped to
/// 9.5 fps on the climb-out and the pilot watched the rudder judder and the
/// pitch bob (LXGB, 2026-08-02). The API is served from X-Plane's flight
/// loop; every deduplicated read is frame time handed back. Sharing the id
/// caches also kills the resolution burst each newly spawned loop paid — the
/// arrival used to re-resolve ~30 names right as autoland armed.
///
/// Ids are stable within a sim session but X-Plane reassigns them across
/// restarts and aircraft reloads, so a value op that 404s evicts the entry
/// and re-resolves by name (see `get_value` / `set_value`) — self-healing,
/// now for every client at once. Values: `name -> (when, value)`, windows per
/// [`value_cache_ttl`] / [`ttl_for`]; a write drops its own entry so the
/// readback every tool's trust rests on still sees the real post-write value.
type PerBase<V> = HashMap<String, HashMap<String, V>>;
static ID_CACHE: std::sync::LazyLock<Mutex<PerBase<i64>>> =
    std::sync::LazyLock::new(Default::default);
static COMMAND_CACHE: std::sync::LazyLock<Mutex<PerBase<i64>>> =
    std::sync::LazyLock::new(Default::default);
static VALUE_CACHE: std::sync::LazyLock<Mutex<PerBase<(std::time::Instant, serde_json::Value)>>> =
    std::sync::LazyLock::new(Default::default);

#[derive(Deserialize)]
struct CommandIndex {
    data: Vec<CommandMeta>,
}

#[derive(Deserialize)]
struct CommandMeta {
    id: i64,
    name: String,
}

#[derive(Deserialize)]
struct DatarefIndex {
    data: Vec<DatarefMeta>,
}

#[derive(Deserialize)]
struct DatarefMeta {
    id: i64,
    name: String,
    #[serde(default)]
    is_writable: Option<bool>,
}

#[derive(Deserialize)]
struct ValueResponse {
    data: serde_json::Value,
}

impl XPlaneClient {
    pub fn new(base: String) -> Self {
        // A per-request timeout so a stalled X-Plane call can never hang the
        // voice loop forever — it errors, and the loop recovers to listening.
        let http = reqwest::Client::builder()
            // Always localhost: a system proxy lookup is never wanted, and reqwest's
            // default one can stall the first build for seconds (2026-09-27).
            .no_proxy()
            .timeout(std::time::Duration::from_secs(8))
            .build()
            .unwrap_or_else(|_| reqwest::Client::new());
        Self {
            http,
            base: base.trim_end_matches('/').to_string(),
            cancel: Vec::new(),
        }
    }

    /// Scope every mutation to the worker that owns this client.
    pub fn with_cancel(mut self, cancel: std::sync::Arc<std::sync::atomic::AtomicBool>) -> Self {
        self.cancel.push(cancel);
        self
    }

    pub fn with_parents(
        mut self,
        parents: Vec<std::sync::Arc<std::sync::atomic::AtomicBool>>,
    ) -> Self {
        self.cancel.extend(parents);
        self
    }
    /// The same connection with NO cancellation scope: for the moves that
    /// make the aircraft safe as an automation ends (the hand-back's parking
    /// brake, a reject's stop). Scoped, they were refused as "automation
    /// cancelled" by the very give-up they belong to, and a 172 was held on
    /// its wheel brakes for fifteen minutes instead (LIPK 2026-09-26).
    pub fn unscoped(&self) -> Self {
        Self {
            http: self.http.clone(),
            base: self.base.clone(),
            cancel: Vec::new(),
        }
    }

    pub fn cancellation(&self) -> Vec<std::sync::Arc<std::sync::atomic::AtomicBool>> {
        self.cancel.clone()
    }

    pub fn cancelled(&self) -> bool {
        self.cancel
            .iter()
            .any(|c| c.load(std::sync::atomic::Ordering::SeqCst))
    }

    fn allow_write(&self) -> Result<()> {
        anyhow::ensure!(!self.cancelled(), "automation cancelled");
        Ok(())
    }

    /// Resolve a dataref's numeric id, caching the result.
    async fn dataref_id(&self, name: &str) -> Result<i64> {
        if let Some(id) = ID_CACHE
            .lock()
            .await
            .get(&self.base)
            .and_then(|m| m.get(name))
            .copied()
        {
            return Ok(id);
        }

        let url = format!("{}/api/v1/datarefs", self.base);
        let index: DatarefIndex = with_retry(|| async {
            self.http
                .get(&url)
                .query(&[("filter[name]", name)])
                .send()
                .await
                .with_context(|| {
                    format!("GET {url} (is X-Plane running with the Web API enabled?)")
                })?
                .error_for_status()?
                .json()
                .await
                .context("decoding dataref index")
        })
        .await?;

        let id = index
            .data
            .into_iter()
            .find(|d| d.name == name)
            .map(|d| d.id)
            .ok_or_else(|| anyhow!("dataref not found: {name}"))?;

        ID_CACHE
            .lock()
            .await
            .entry(self.base.clone())
            .or_default()
            .insert(name.to_string(), id);
        Ok(id)
    }

    /// GET a dataref's `value`, re-resolving the id once if the cached one 404s.
    /// X-Plane reassigns numeric ids across restarts and aircraft reloads, so a
    /// cached handle eventually points at nothing; rather than spam 404s forever
    /// (the poller hits this every frame), drop the stale id and resolve by name
    /// again. `decoding` labels the JSON-decode step for error context.
    async fn get_value(&self, name: &str, decoding: &'static str) -> Result<serde_json::Value> {
        note_named(name);
        let ttl = ttl_for(name);
        if !ttl.is_zero() {
            if let Some((at, v)) = VALUE_CACHE
                .lock()
                .await
                .get(&self.base)
                .and_then(|m| m.get(name))
            {
                if at.elapsed() < ttl {
                    note_cache_hit();
                    return Ok(v.clone());
                }
            }
        }
        let mut refreshed = false;
        loop {
            let id = self.dataref_id(name).await?;
            let url = format!("{}/api/v1/datarefs/{}/value", self.base, id);
            let result: Result<ValueResponse> = with_retry(|| async {
                self.http
                    .get(&url)
                    .send()
                    .await
                    .with_context(|| format!("GET {url}"))?
                    .error_for_status()?
                    .json()
                    .await
                    .context(decoding)
            })
            .await;
            match result {
                Ok(resp) => {
                    if !ttl.is_zero() {
                        VALUE_CACHE
                            .lock()
                            .await
                            .entry(self.base.clone())
                            .or_default()
                            .insert(
                                name.to_string(),
                                (std::time::Instant::now(), resp.data.clone()),
                            );
                    }
                    return Ok(resp.data);
                }
                Err(e) if !refreshed && is_not_found(&e) => {
                    if let Some(m) = ID_CACHE.lock().await.get_mut(&self.base) {
                        m.remove(name); // stale handle — re-resolve
                    }
                    refreshed = true;
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Read a scalar dataref as `f64`.
    pub async fn get_float(&self, name: &str) -> Result<f64> {
        let data = self.get_value(name, "decoding dataref value").await?;
        data.as_f64()
            .ok_or_else(|| anyhow!("dataref {name} is not a scalar number: {data}"))
    }

    /// List every dataref whose name starts with `prefix`, as `(name, id)` pairs.
    /// The Web API's `filter[name]` is exact-match only, so this fetches the full
    /// index once and filters locally — used to enumerate the *complete* failure
    /// set (`sim/operation/failures/rel_*`, ~800 entries) without hardcoding names,
    /// so the copilot can never be blind to a failure whose dataref it forgot.
    pub async fn list_datarefs_by_prefix(&self, prefix: &str) -> Result<Vec<(String, i64)>> {
        let url = format!("{}/api/v1/datarefs", self.base);
        let index: DatarefIndex = with_retry(|| async {
            self.http
                .get(&url)
                .send()
                .await
                .with_context(|| format!("GET {url} (Web API enabled?)"))?
                .error_for_status()?
                .json()
                .await
                .context("decoding dataref index")
        })
        .await?;
        Ok(index
            .data
            .into_iter()
            .filter(|d| d.name.starts_with(prefix))
            .map(|d| (d.name, d.id))
            .collect())
    }

    /// Read a dataref's `value` directly by numeric id, skipping name→id
    /// resolution — for bulk reads of an already-enumerated set (the failure scan
    /// reads hundreds at once). Returns the raw JSON so the caller decodes the
    /// scalar-or-array shape.
    pub async fn get_value_by_id(&self, id: i64) -> Result<serde_json::Value> {
        let url = format!("{}/api/v1/datarefs/{}/value", self.base, id);
        let resp: ValueResponse = with_retry(|| async {
            self.http
                .get(&url)
                .send()
                .await
                .with_context(|| format!("GET {url}"))?
                .error_for_status()?
                .json()
                .await
                .context("decoding dataref value")
        })
        .await?;
        Ok(resp.data)
    }

    /// Write a scalar float dataref.
    pub async fn set_float(&self, name: &str, value: f64) -> Result<()> {
        self.set_value(name, json!(value)).await
    }

    /// Write an integer dataref (e.g. a 0/1 switch). Sent as a JSON integer so
    /// X-Plane doesn't reject a `1.0` for an int-typed dataref.
    pub async fn set_int(&self, name: &str, value: i64) -> Result<()> {
        self.set_value(name, json!(value)).await
    }

    /// Write a `data` (byte-array) dataref, base64-encoded as the Web API expects.
    /// Used to hand the plugin a payload (e.g. the FMS route).
    pub async fn set_data(&self, name: &str, bytes: &[u8]) -> Result<()> {
        let b64 = base64::engine::general_purpose::STANDARD.encode(bytes);
        self.set_value(name, json!(b64)).await
    }

    /// Write a raw JSON value to a dataref's `value` endpoint.
    async fn set_value(&self, name: &str, value: serde_json::Value) -> Result<()> {
        self.allow_write()?;
        // A write makes any cached value a lie, and the readback that follows
        // it is the whole trust model — drop the entry so that read is real.
        if let Some(m) = VALUE_CACHE.lock().await.get_mut(&self.base) {
            m.remove(name);
        }
        let body = json!({ "data": value });
        let mut refreshed = false;
        loop {
            let id = self.dataref_id(name).await?;
            let url = format!("{}/api/v1/datarefs/{}/value", self.base, id);
            let result = with_retry(|| async {
                self.allow_write()?;
                self.http
                    .patch(&url)
                    .json(&body)
                    .send()
                    .await
                    .with_context(|| format!("PATCH {url}"))?
                    .error_for_status()?;
                Ok::<(), anyhow::Error>(())
            })
            .await;
            match result {
                Ok(()) => return Ok(()),
                Err(e) if !refreshed && is_not_found(&e) => {
                    if let Some(m) = ID_CACHE.lock().await.get_mut(&self.base) {
                        m.remove(name); // stale handle — re-resolve
                    }
                    refreshed = true;
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Write ONE element of an array dataref (`?index=`), leaving the others
    /// untouched. `set_float_array_elem` reads the whole array and writes it
    /// back, which rewrites every other element with a value read a round trip
    /// ago: fine for a switch, a fight for a control loop ticking at 10 Hz on
    /// an array whose other elements the sim moves itself (the S-76's
    /// `prop_ratio[1]` follows the collective, measured 2026-09-22). Not
    /// retried and not re-resolved: the next tick is the retry.
    pub async fn set_float_index(&self, name: &str, index: usize, value: f64) -> Result<()> {
        self.allow_write()?;
        if let Some(m) = VALUE_CACHE.lock().await.get_mut(&self.base) {
            m.remove(name);
        }
        let id = self.dataref_id(name).await?;
        let url = format!("{}/api/v1/datarefs/{}/value?index={index}", self.base, id);
        self.http
            .patch(&url)
            .json(&json!({ "data": value }))
            .send()
            .await
            .with_context(|| format!("PATCH {url}"))?
            .error_for_status()?;
        Ok(())
    }

    /// Resolve a command's numeric id (commands are on the v2 API). The v2
    /// `filter[name]` query doesn't work for commands, so we fetch the full
    /// index once and cache it.
    async fn command_id(&self, name: &str) -> Result<i64> {
        if let Some(id) = COMMAND_CACHE
            .lock()
            .await
            .get(&self.base)
            .and_then(|m| m.get(name))
            .copied()
        {
            return Ok(id);
        }
        let url = format!("{}/api/v2/commands", self.base);
        let index: CommandIndex = with_retry(|| async {
            self.http
                .get(&url)
                .send()
                .await
                .with_context(|| format!("GET {url}"))?
                .error_for_status()?
                .json()
                .await
                .context("decoding command index")
        })
        .await?;

        let id = index
            .data
            .into_iter()
            .find(|c| c.name == name)
            .map(|c| c.id)
            .ok_or_else(|| anyhow!("command not found: {name}"))?;

        COMMAND_CACHE
            .lock()
            .await
            .entry(self.base.clone())
            .or_default()
            .insert(name.to_string(), id);
        Ok(id)
    }

    /// Read a byte-array dataref RAW (e.g. the TCAS flight_id table: 8 bytes
    /// per target, NUL-padded) — interior NULs and slot alignment preserved,
    /// unlike [`XPlaneClient::get_string`] which is for single values.
    pub async fn get_bytes(&self, name: &str) -> Result<Vec<u8>> {
        let data = self.get_value(name, "decoding byte-array dataref").await?;
        let b64 = data
            .as_str()
            .ok_or_else(|| anyhow!("dataref {name} is not a data value: {data}"))?;
        base64::engine::general_purpose::STANDARD
            .decode(b64)
            .context("base64-decoding dataref")
    }

    /// Read a `data`-typed dataref (returned base64-encoded) as a string,
    /// trimming the null padding X-Plane uses for fixed-width fields.
    pub async fn get_string(&self, name: &str) -> Result<String> {
        let data = self.get_value(name, "decoding string dataref").await?;
        let b64 = data
            .as_str()
            .ok_or_else(|| anyhow!("dataref {name} is not a string value: {data}"))?;
        let bytes = base64::engine::general_purpose::STANDARD
            .decode(b64)
            .context("base64-decoding dataref")?;
        Ok(String::from_utf8_lossy(&bytes)
            .trim_matches('\0')
            .trim()
            .to_string())
    }

    /// Every dataref the running sim has, as `(name, writable)`: ONE request,
    /// for the profile probe that asks about dozens of names at once (a
    /// profile line is a hypothesis until asked of the sim; the RV-10's xlua
    /// fuel selector and its autopilot went unverified for a whole flight,
    /// 2026-09-11). Seeds the id cache on the way past.
    pub async fn dataref_index(&self) -> Result<Vec<(String, bool)>> {
        let url = format!("{}/api/v1/datarefs", self.base);
        let index: DatarefIndex = with_retry(|| async {
            self.http
                .get(&url)
                .send()
                .await
                .with_context(|| format!("GET {url} (Web API enabled?)"))?
                .error_for_status()?
                .json()
                .await
                .context("decoding dataref index")
        })
        .await?;
        let mut cache = ID_CACHE.lock().await;
        let entry = cache.entry(self.base.clone()).or_default();
        Ok(index
            .data
            .into_iter()
            .map(|d| {
                entry.insert(d.name.clone(), d.id);
                (d.name, d.is_writable.unwrap_or(false))
            })
            .collect())
    }

    /// Every command the running sim has: one request, for the probe.
    pub async fn command_index(&self) -> Result<Vec<String>> {
        let url = format!("{}/api/v2/commands", self.base);
        let index: CommandIndex = with_retry(|| async {
            self.http
                .get(&url)
                .send()
                .await
                .with_context(|| format!("GET {url}"))?
                .error_for_status()?
                .json()
                .await
                .context("decoding command index")
        })
        .await?;
        Ok(index.data.into_iter().map(|c| c.name).collect())
    }

    /// The loaded aircraft's ICAO type code (e.g. `C172`), for auto-detection.
    pub async fn aircraft_icao(&self) -> Result<String> {
        self.get_string("sim/aircraft/view/acf_ICAO").await
    }

    /// Activate (momentarily press) a command by name.
    pub async fn run_command(&self, name: &str) -> Result<()> {
        self.run_command_for(name, 0.0).await
    }

    /// Activate a command, holding it for `duration` seconds (e.g. cranking a
    /// starter). `0.0` is a momentary press.
    pub async fn run_command_for(&self, name: &str, duration: f64) -> Result<()> {
        self.allow_write()?;
        let id = self.command_id(name).await?;
        self.allow_write()?;
        let url = format!("{}/api/v2/command/{}/activate", self.base, id);
        // NOT retried: a command can be a toggle (chocks, GPU), so a retry after a
        // lost response could activate it twice. Absolute-value writes (set_value)
        // are idempotent and do retry; command activation isn't.
        self.http
            .post(&url)
            .json(&json!({ "duration": duration }))
            .send()
            .await
            .with_context(|| format!("POST {url}"))?
            .error_for_status()?;
        // A command can move ANY dataref, so every cached value is suspect:
        // the read-before-fire / read-after tools (toggles, stepped knobs)
        // rest on the read after it being real. The altimeter knob sits in
        // the 500 ms slow cache, and three `barometer_up` presses read back
        // the value from BEFORE them: "altimeter stepped ... now=30.35" with
        // the dial at 30.38 (2026-09-21). Commands are rare; the cache is for
        // the per-tick reads between them.
        if let Some(m) = VALUE_CACHE.lock().await.get_mut(&self.base) {
            m.clear();
        }
        Ok(())
    }

    /// Read an array-typed dataref (e.g. per-engine values) as `f64`s.
    pub async fn get_float_array(&self, name: &str) -> Result<Vec<f64>> {
        let data = self.get_value(name, "decoding array dataref").await?;
        let arr = data
            .as_array()
            .ok_or_else(|| anyhow!("dataref {name} is not an array: {data}"))?;
        Ok(arr.iter().filter_map(|v| v.as_f64()).collect())
    }

    /// Set one element of an integer array dataref (read-modify-write the array).
    pub async fn set_int_array_elem(&self, name: &str, index: usize, value: i64) -> Result<()> {
        let mut arr = self.get_float_array(name).await?;
        if index >= arr.len() {
            return Err(anyhow!("array {name} has no index {index}"));
        }
        arr[index] = value as f64;
        let data: Vec<i64> = arr.iter().map(|v| v.round() as i64).collect();
        self.set_value(name, json!(data)).await
    }

    /// Set one element of a float array dataref (read-modify-write), preserving
    /// the other elements as floats — for things like a per-channel brightness.
    pub async fn set_float_array_elem(&self, name: &str, index: usize, value: f64) -> Result<()> {
        let mut arr = self.get_float_array(name).await?;
        if index >= arr.len() {
            return Err(anyhow!("array {name} has no index {index}"));
        }
        arr[index] = value;
        self.set_value(name, json!(arr)).await
    }

    /// The world side's handle on the simulator: reads only. ATC observes and
    /// returns text — this view exposes no `set_*`/`run_command`, so the
    /// compiler enforces the "the world never actuates" trust boundary.
    pub fn view(&self) -> SimView<'_> {
        SimView(self)
    }
}

/// Read-only view over [`XPlaneClient`] — see [`XPlaneClient::view`]. Exposes
/// exactly the reads the ATC world needs; add methods only for reads, never
/// writes or commands.
#[derive(Clone, Copy)]
pub struct SimView<'a>(&'a XPlaneClient);

impl SimView<'_> {
    pub async fn get_float(&self, name: &str) -> Result<f64> {
        self.0.get_float(name).await
    }
    pub async fn get_string(&self, name: &str) -> Result<String> {
        self.0.get_string(name).await
    }
    pub async fn get_bytes(&self, name: &str) -> Result<Vec<u8>> {
        self.0.get_bytes(name).await
    }
    pub async fn get_float_array(&self, name: &str) -> Result<Vec<f64>> {
        self.0.get_float_array(name).await
    }
}

// The trait just forwards to the inherent methods above (which stay the canonical
// implementation, and which the loop calls directly for the index/string helpers).
#[async_trait]
impl XPlane for XPlaneClient {
    fn cancellation(&self) -> Vec<std::sync::Arc<std::sync::atomic::AtomicBool>> {
        self.cancellation()
    }
    async fn get_float(&self, name: &str) -> Result<f64> {
        XPlaneClient::get_float(self, name).await
    }
    async fn set_float(&self, name: &str, value: f64) -> Result<()> {
        XPlaneClient::set_float(self, name, value).await
    }
    async fn set_int(&self, name: &str, value: i64) -> Result<()> {
        XPlaneClient::set_int(self, name, value).await
    }
    async fn run_command(&self, name: &str) -> Result<()> {
        XPlaneClient::run_command(self, name).await
    }
    async fn run_command_for(&self, name: &str, duration: f64) -> Result<()> {
        XPlaneClient::run_command_for(self, name, duration).await
    }
    async fn get_float_array(&self, name: &str) -> Result<Vec<f64>> {
        XPlaneClient::get_float_array(self, name).await
    }
    async fn set_int_array_elem(&self, name: &str, index: usize, value: i64) -> Result<()> {
        XPlaneClient::set_int_array_elem(self, name, index, value).await
    }
    async fn set_float_array_elem(&self, name: &str, index: usize, value: f64) -> Result<()> {
        XPlaneClient::set_float_array_elem(self, name, index, value).await
    }
}

/// Recording fake for tests: reads come from a preset table, writes and commands
/// are recorded for assertions. A read with no preset value errors (so a test
/// fails loudly if the code under test reads a dataref it didn't set up).
/// Compiled unconditionally so downstream crates' test suites can use it.
pub struct FakeXPlane {
    reads: std::sync::Mutex<HashMap<String, f64>>,
    /// Array-valued reads, for the per-engine and per-bus datarefs. Without
    /// these every array read came back EMPTY, which reads downstream as zero
    /// volts / no engine running — so any code path behind an array gate was
    /// untestable, and the paths behind them are exactly the ones that fail
    /// silently in the cockpit (a start that cranks a dry engine).
    array_reads: std::sync::Mutex<HashMap<String, Vec<f64>>>,
    /// Every set_float/set_int, in order, as (dataref, value).
    writes: std::sync::Mutex<Vec<(String, f64)>>,
    /// Every command run, in order.
    commands: std::sync::Mutex<Vec<String>>,
    /// A command that moves a read: (dataref, delta per press). A stepped
    /// knob whose dial write does not stand (the 737's altimeter) is this.
    command_steps: std::sync::Mutex<HashMap<String, (String, f64)>>,
    /// Presses (by ordinal, 1-based, over every command run) whose step lands
    /// one read LATE: a command the sim executes in a flight loop after the
    /// read that followed it. `presses` counts them; `held` are the late
    /// steps, with whether a read has already gone by since the press.
    late_presses: std::sync::Mutex<std::collections::HashSet<usize>>,
    presses: std::sync::Mutex<usize>,
    held: std::sync::Mutex<Vec<(String, f64, bool)>>,
    /// Every array element write, in order, as (dataref, index, value). The
    /// flat `writes` log drops the index, which is the whole question for a
    /// switch that is one element of a per-tank array.
    index_writes: std::sync::Mutex<Vec<(String, usize, f64)>>,
    /// Arrays whose element writes LAND in the preset read (a switch the sim
    /// takes), so a readback after the write sees it. Opt-in: by default a
    /// preset array stays as set, which is a switch that did not take.
    live_arrays: std::sync::Mutex<std::collections::HashSet<String>>,
}

impl FakeXPlane {
    pub fn new() -> Self {
        Self {
            reads: std::sync::Mutex::new(HashMap::new()),
            array_reads: std::sync::Mutex::new(HashMap::new()),
            writes: std::sync::Mutex::new(Vec::new()),
            commands: std::sync::Mutex::new(Vec::new()),
            command_steps: std::sync::Mutex::new(HashMap::new()),
            late_presses: std::sync::Mutex::new(std::collections::HashSet::new()),
            presses: std::sync::Mutex::new(0),
            held: std::sync::Mutex::new(Vec::new()),
            index_writes: std::sync::Mutex::new(Vec::new()),
            live_arrays: std::sync::Mutex::new(std::collections::HashSet::new()),
        }
    }

    /// Make element writes to `name` land in its preset array read.
    pub fn apply_array_writes(&self, name: &str) {
        self.live_arrays.lock().unwrap().insert(name.to_string());
    }

    /// Every element written to `name`, in order, as (index, value).
    pub fn index_writes(&self, name: &str) -> Vec<(usize, f64)> {
        self.index_writes
            .lock()
            .unwrap()
            .iter()
            .filter(|(n, _, _)| n == name)
            .map(|(_, i, v)| (*i, *v))
            .collect()
    }

    fn note_index_write(&self, name: &str, index: usize, value: f64) {
        self.index_writes
            .lock()
            .unwrap()
            .push((name.to_string(), index, value));
        if self.live_arrays.lock().unwrap().contains(name) {
            if let Some(slot) = self
                .array_reads
                .lock()
                .unwrap()
                .get_mut(name)
                .and_then(|a| a.get_mut(index))
            {
                *slot = value;
            }
        }
    }

    /// The `nth` command run (1-based, counted over every command) moves its
    /// stepped read one read late: the read straight after it still shows the
    /// dial without it, the read after that has it. This is the sim as it
    /// was at LIBP (2026-09-26): eighteen altimeter presses, a read 120 ms
    /// later showing seventeen, the eighteenth landing before the next read.
    pub fn land_press_late(&self, nth: usize) {
        self.late_presses.lock().unwrap().insert(nth);
    }

    /// Make `command` move the read of `dataref` by `delta` per press.
    pub fn step_on_command(&self, command: &str, dataref: &str, delta: f64) {
        self.command_steps
            .lock()
            .unwrap()
            .insert(command.to_string(), (dataref.to_string(), delta));
    }

    /// Preset the values reads will return.
    pub fn with_reads(pairs: &[(&str, f64)]) -> Self {
        let me = Self::new();
        for (k, v) in pairs {
            me.set_read(k, *v);
        }
        me
    }

    /// Update (or insert) the value a read will return. This is the scripted-
    /// timeline primitive: a test that advances a simulated world calls this
    /// each tick to move the aircraft, so the code under test reads a changing
    /// dataref exactly as it would against a live sim. `with_reads` sets a
    /// static table; `set_read` makes it a timeline.
    pub fn set_read(&self, name: &str, value: f64) {
        self.reads.lock().unwrap().insert(name.to_string(), value);
    }

    /// The most recent value written to `name`, if any.
    pub fn last_write(&self, name: &str) -> Option<f64> {
        self.writes
            .lock()
            .unwrap()
            .iter()
            .rev()
            .find(|(n, _)| n == name)
            .map(|(_, v)| *v)
    }

    /// Preset an array-valued read (bus volts, per-engine indicators).
    pub fn set_read_array(&self, name: &str, values: &[f64]) {
        self.array_reads
            .lock()
            .unwrap()
            .insert(name.to_string(), values.to_vec());
    }

    /// Whether `name` was ever written.
    pub fn wrote(&self, name: &str) -> bool {
        self.writes.lock().unwrap().iter().any(|(n, _)| n == name)
    }

    /// Whether `command` was ever run.
    pub fn ran(&self, command: &str) -> bool {
        self.commands.lock().unwrap().iter().any(|c| c == command)
    }

    /// How many times a command was fired (a stepped knob is a count).
    pub fn ran_count(&self, command: &str) -> usize {
        self.commands
            .lock()
            .unwrap()
            .iter()
            .filter(|c| *c == command)
            .count()
    }

    /// Index of the first time `command` ran, for ordering assertions.
    pub fn command_index(&self, command: &str) -> Option<usize> {
        self.commands
            .lock()
            .unwrap()
            .iter()
            .position(|c| c == command)
    }
}

impl Default for FakeXPlane {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl XPlane for FakeXPlane {
    async fn get_float(&self, name: &str) -> Result<f64> {
        // A late press lands on the SECOND read after it: the first read
        // marks it seen, the next applies it.
        {
            let mut held = self.held.lock().unwrap();
            let mut reads = self.reads.lock().unwrap();
            held.retain(|(dref, delta, seen)| {
                if *seen {
                    *reads.entry(dref.clone()).or_insert(0.0) += delta;
                }
                !*seen
            });
            for h in held.iter_mut() {
                h.2 = true;
            }
        }
        self.reads
            .lock()
            .unwrap()
            .get(name)
            .copied()
            .ok_or_else(|| anyhow!("FakeXPlane: no read preset for {name}"))
    }
    async fn set_float(&self, name: &str, value: f64) -> Result<()> {
        self.writes.lock().unwrap().push((name.to_string(), value));
        Ok(())
    }
    async fn set_int(&self, name: &str, value: i64) -> Result<()> {
        self.writes
            .lock()
            .unwrap()
            .push((name.to_string(), value as f64));
        Ok(())
    }
    async fn run_command(&self, name: &str) -> Result<()> {
        self.commands.lock().unwrap().push(name.to_string());
        let nth = {
            let mut n = self.presses.lock().unwrap();
            *n += 1;
            *n
        };
        if let Some((dref, delta)) = self.command_steps.lock().unwrap().get(name) {
            if self.late_presses.lock().unwrap().contains(&nth) {
                self.held
                    .lock()
                    .unwrap()
                    .push((dref.clone(), *delta, false));
            } else {
                *self
                    .reads
                    .lock()
                    .unwrap()
                    .entry(dref.clone())
                    .or_insert(0.0) += delta;
            }
        }
        Ok(())
    }
    async fn run_command_for(&self, name: &str, _duration: f64) -> Result<()> {
        self.commands.lock().unwrap().push(name.to_string());
        Ok(())
    }
    async fn get_float_array(&self, name: &str) -> Result<Vec<f64>> {
        // A preset array wins; then a scalar preset, read as a one-element
        // array (which is what a single-engine dataref is); else empty, the
        // long-standing default.
        if let Some(v) = self.array_reads.lock().unwrap().get(name) {
            return Ok(v.clone());
        }
        Ok(self
            .reads
            .lock()
            .unwrap()
            .get(name)
            .map(|v| vec![*v])
            .unwrap_or_default())
    }
    async fn set_int_array_elem(&self, name: &str, index: usize, value: i64) -> Result<()> {
        self.writes
            .lock()
            .unwrap()
            .push((name.to_string(), value as f64));
        self.note_index_write(name, index, value as f64);
        Ok(())
    }
    async fn set_float_array_elem(&self, name: &str, index: usize, value: f64) -> Result<()> {
        self.writes.lock().unwrap().push((name.to_string(), value));
        self.note_index_write(name, index, value);
        Ok(())
    }
}

#[cfg(test)]
mod load_tests {
    /// The cache window has to be short enough that no control law can tell.
    /// Every loop that flies the aircraft ticks at 150 ms or slower, so a value
    /// may never be more than a fraction of a tick old.
    #[test]
    fn the_value_cache_window_is_shorter_than_a_control_tick() {
        let ttl = super::value_cache_ttl();
        assert!(
            ttl <= std::time::Duration::from_millis(75),
            "a stale read must not reach a control law: {ttl:?}"
        );
    }

    /// The freeze flags were ~4 reads a second of a value that changes a
    /// handful of times in a whole flight, and they sat second only to
    /// position in the per-dataref histogram. The API is served from inside
    /// X-Plane's flight loop, so that is frame time the sim does not get: at
    /// 20:42 on 2026-08-01 it dropped to 7 fps against a 60 fps median in the
    /// same window that logged 76 req/s and a 1,040 ms slowest response.
    #[test]
    fn the_flags_that_never_move_get_a_longer_window() {
        let base = super::value_cache_ttl();
        for slow in super::SLOW_DATAREFS {
            assert!(
                super::ttl_for(slow) >= super::SLOW_DATAREF_TTL,
                "{slow} should get the long window"
            );
            assert!(
                super::ttl_for(slow) > base,
                "...and a longer one than the default"
            );
        }
        // Everything a control law reads keeps the short window.
        for live in [
            "sim/flightmodel/position/latitude",
            "sim/flightmodel/position/indicated_airspeed",
            "sim/cockpit2/controls/parking_brake_ratio",
        ] {
            assert_eq!(super::ttl_for(live), base, "{live} must stay live");
        }
        // Half a second is still far short of anything that would let a
        // stale freeze flag matter: the loops re-check every tick and
        // `sim_frozen` fails open.
        assert!(super::SLOW_DATAREF_TTL < std::time::Duration::from_secs(1));
    }
}

#[cfg(test)]
mod failure_sequences {
    use super::*;
    use std::{
        io::{Read, Write},
        net::TcpListener,
        sync::{
            atomic::{AtomicBool, Ordering},
            Arc,
        },
    };

    struct Reply {
        request: &'static str,
        status: u16,
        body: &'static str,
        cancel: Option<Arc<AtomicBool>>,
    }
    fn server(replies: Vec<Reply>) -> (String, std::thread::JoinHandle<()>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let base = format!("http://{}", listener.local_addr().unwrap());
        listener.set_nonblocking(true).unwrap();
        let thread = std::thread::spawn(move || {
            for reply in replies {
                let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
                let mut socket = loop {
                    if let Ok((s, _)) = listener.accept() {
                        break s;
                    }
                    assert!(
                        std::time::Instant::now() < deadline,
                        "missing {}",
                        reply.request
                    );
                    std::thread::sleep(std::time::Duration::from_millis(5));
                };
                // BSD hands the accepted socket the listener's O_NONBLOCK; Linux
                // does not. Left set, every read here is a race with the bytes
                // arriving and `read_exact` returns WouldBlock (macos CI, 2026-09-19).
                socket.set_nonblocking(false).unwrap();
                socket
                    .set_read_timeout(Some(std::time::Duration::from_secs(3)))
                    .unwrap();
                let mut request = Vec::new();
                let header_end = loop {
                    let mut byte = [0];
                    socket.read_exact(&mut byte).unwrap();
                    request.push(byte[0]);
                    if request.ends_with(b"\r\n\r\n") {
                        break request.len();
                    }
                };
                let text = String::from_utf8_lossy(&request);
                assert!(text.starts_with(reply.request), "{text}");
                let length: usize = text
                    .lines()
                    .find_map(|l| {
                        l.to_ascii_lowercase()
                            .strip_prefix("content-length:")
                            .map(|v| v.trim().parse().unwrap())
                    })
                    .unwrap_or(0);
                request.resize(header_end + length, 0);
                socket.read_exact(&mut request[header_end..]).unwrap();
                if let Some(cancel) = reply.cancel {
                    cancel.store(true, Ordering::SeqCst);
                }
                write!(socket, "HTTP/1.1 {} Test\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}", reply.status, reply.body.len(), reply.body).unwrap();
            }
        });
        (base, thread)
    }
    fn reply(request: &'static str, status: u16, body: &'static str) -> Reply {
        Reply {
            request,
            status,
            body,
            cancel: None,
        }
    }

    #[tokio::test]
    async fn plugin_reload_re_resolves_a_stale_write_and_reads_back_the_new_id() {
        let (base, server) = server(vec![
            reply(
                "GET /api/v1/datarefs?",
                200,
                r#"{"data":[{"id":1,"name":"test/dial"}]}"#,
            ),
            reply("GET /api/v1/datarefs/1/value", 200, r#"{"data":7}"#),
            reply("PATCH /api/v1/datarefs/1/value", 404, "{}"),
            reply(
                "GET /api/v1/datarefs?",
                200,
                r#"{"data":[{"id":2,"name":"test/dial"}]}"#,
            ),
            reply("PATCH /api/v1/datarefs/2/value", 200, "{}"),
            reply("GET /api/v1/datarefs/2/value", 200, r#"{"data":9}"#),
        ]);
        let xp = XPlaneClient::new(base);
        assert_eq!(xp.get_float("test/dial").await.unwrap(), 7.0);
        xp.set_float("test/dial", 9.0).await.unwrap();
        assert_eq!(xp.get_float("test/dial").await.unwrap(), 9.0);
        server.join().unwrap();
    }

    #[tokio::test]
    async fn cancellation_during_resolution_prevents_the_later_patch() {
        let cancel = Arc::new(AtomicBool::new(false));
        let (base, server) = server(vec![Reply {
            request: "GET /api/v1/datarefs?",
            status: 200,
            body: r#"{"data":[{"id":1,"name":"test/dial"}]}"#,
            cancel: Some(cancel.clone()),
        }]);
        let xp = XPlaneClient::new(base).with_cancel(cancel);
        assert!(xp
            .set_float("test/dial", 9.0)
            .await
            .unwrap_err()
            .to_string()
            .contains("cancelled"));
        assert!(xp
            .run_command("test/toggle")
            .await
            .unwrap_err()
            .to_string()
            .contains("cancelled"));
        server.join().unwrap();
    }

    #[tokio::test]
    async fn a_cancelled_parent_blocks_a_child_created_after_cancellation() {
        let parent = Arc::new(AtomicBool::new(true));
        let child = Arc::new(AtomicBool::new(false));
        let xp = XPlaneClient::new("http://127.0.0.1:1".into())
            .with_cancel(child.clone())
            .with_parents(vec![parent]);
        assert!(xp.cancelled());
        assert!(!child.load(Ordering::SeqCst));
        assert!(xp
            .set_float("test/dial", 1.0)
            .await
            .unwrap_err()
            .to_string()
            .contains("cancelled"));
        assert!(xp
            .run_command("test/toggle")
            .await
            .unwrap_err()
            .to_string()
            .contains("cancelled"));
    }

    #[tokio::test]
    async fn pause_defers_handoff_until_live_measurements_resume() {
        let (base, server) = server(vec![
            reply(
                "GET /api/v1/datarefs?",
                200,
                r#"{"data":[{"id":1,"name":"sim/time/is_in_replay"}]}"#,
            ),
            reply("GET /api/v1/datarefs/1/value", 200, r#"{"data":0}"#),
            reply(
                "GET /api/v1/datarefs?",
                200,
                r#"{"data":[{"id":2,"name":"sim/time/paused"}]}"#,
            ),
            reply("GET /api/v1/datarefs/2/value", 200, r#"{"data":1}"#),
            reply("GET /api/v1/datarefs/1/value", 200, r#"{"data":0}"#),
            reply("GET /api/v1/datarefs/2/value", 200, r#"{"data":0}"#),
        ]);
        let xp = XPlaneClient::new(base.clone());
        assert!(sim_frozen(&xp).await);
        VALUE_CACHE.lock().await.remove(&base); // next sample after cache expiry
        assert!(!sim_frozen(&xp).await);
        server.join().unwrap();
    }
}
