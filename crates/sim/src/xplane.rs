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
        match op().await {
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
        e.downcast_ref::<reqwest::Error>().and_then(reqwest::Error::status)
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
    /// Resolved `dataref name -> numeric id`, to save a round trip per read/write.
    /// Ids are stable within a session but X-Plane reassigns them across restarts
    /// and aircraft reloads, so a value op that 404s evicts the entry here and
    /// re-resolves by name (see `get_value` / `set_value`) — the cache self-heals.
    id_cache: Mutex<HashMap<String, i64>>,
    /// Resolved `command name -> numeric id` (commands live on the v2 API).
    command_cache: Mutex<HashMap<String, i64>>,
}

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
            .timeout(std::time::Duration::from_secs(8))
            .build()
            .unwrap_or_else(|_| reqwest::Client::new());
        Self {
            http,
            base: base.trim_end_matches('/').to_string(),
            id_cache: Mutex::new(HashMap::new()),
            command_cache: Mutex::new(HashMap::new()),
        }
    }

    /// Resolve a dataref's numeric id, caching the result.
    async fn dataref_id(&self, name: &str) -> Result<i64> {
        if let Some(id) = self.id_cache.lock().await.get(name).copied() {
            return Ok(id);
        }

        let url = format!("{}/api/v1/datarefs", self.base);
        let index: DatarefIndex = with_retry(|| async {
            self.http
                .get(&url)
                .query(&[("filter[name]", name)])
                .send()
                .await
                .with_context(|| format!("GET {url} (is X-Plane running with the Web API enabled?)"))?
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

        self.id_cache.lock().await.insert(name.to_string(), id);
        Ok(id)
    }

    /// GET a dataref's `value`, re-resolving the id once if the cached one 404s.
    /// X-Plane reassigns numeric ids across restarts and aircraft reloads, so a
    /// cached handle eventually points at nothing; rather than spam 404s forever
    /// (the poller hits this every frame), drop the stale id and resolve by name
    /// again. `decoding` labels the JSON-decode step for error context.
    async fn get_value(&self, name: &str, decoding: &'static str) -> Result<serde_json::Value> {
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
                Ok(resp) => return Ok(resp.data),
                Err(e) if !refreshed && is_not_found(&e) => {
                    self.id_cache.lock().await.remove(name); // stale handle — re-resolve
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
        let body = json!({ "data": value });
        let mut refreshed = false;
        loop {
            let id = self.dataref_id(name).await?;
            let url = format!("{}/api/v1/datarefs/{}/value", self.base, id);
            let result = with_retry(|| async {
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
                    self.id_cache.lock().await.remove(name); // stale handle — re-resolve
                    refreshed = true;
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Resolve a command's numeric id (commands are on the v2 API). The v2
    /// `filter[name]` query doesn't work for commands, so we fetch the full
    /// index once and cache it.
    async fn command_id(&self, name: &str) -> Result<i64> {
        if let Some(id) = self.command_cache.lock().await.get(name).copied() {
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

        self.command_cache.lock().await.insert(name.to_string(), id);
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
        let id = self.command_id(name).await?;
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
    /// Every set_float/set_int, in order, as (dataref, value).
    writes: std::sync::Mutex<Vec<(String, f64)>>,
    /// Every command run, in order.
    commands: std::sync::Mutex<Vec<String>>,
}

impl FakeXPlane {
    pub fn new() -> Self {
        Self {
            reads: std::sync::Mutex::new(HashMap::new()),
            writes: std::sync::Mutex::new(Vec::new()),
            commands: std::sync::Mutex::new(Vec::new()),
        }
    }

    /// Preset the values reads will return.
    pub fn with_reads(pairs: &[(&str, f64)]) -> Self {
        let me = Self::new();
        let mut r = me.reads.lock().unwrap();
        for (k, v) in pairs {
            r.insert((*k).to_string(), *v);
        }
        drop(r);
        me
    }

    /// The most recent value written to `name`, if any.
    pub fn last_write(&self, name: &str) -> Option<f64> {
        self.writes.lock().unwrap().iter().rev().find(|(n, _)| n == name).map(|(_, v)| *v)
    }

    /// Whether `name` was ever written.
    pub fn wrote(&self, name: &str) -> bool {
        self.writes.lock().unwrap().iter().any(|(n, _)| n == name)
    }

    /// Whether `command` was ever run.
    pub fn ran(&self, command: &str) -> bool {
        self.commands.lock().unwrap().iter().any(|c| c == command)
    }

    /// Index of the first time `command` ran, for ordering assertions.
    pub fn command_index(&self, command: &str) -> Option<usize> {
        self.commands.lock().unwrap().iter().position(|c| c == command)
    }
}

#[async_trait]
impl XPlane for FakeXPlane {
    async fn get_float(&self, name: &str) -> Result<f64> {
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
        self.writes.lock().unwrap().push((name.to_string(), value as f64));
        Ok(())
    }
    async fn run_command(&self, name: &str) -> Result<()> {
        self.commands.lock().unwrap().push(name.to_string());
        Ok(())
    }
    async fn run_command_for(&self, name: &str, _duration: f64) -> Result<()> {
        self.commands.lock().unwrap().push(name.to_string());
        Ok(())
    }
    async fn get_float_array(&self, _name: &str) -> Result<Vec<f64>> {
        Ok(Vec::new())
    }
    async fn set_int_array_elem(&self, name: &str, _index: usize, value: i64) -> Result<()> {
        self.writes.lock().unwrap().push((name.to_string(), value as f64));
        Ok(())
    }
    async fn set_float_array_elem(&self, name: &str, _index: usize, value: f64) -> Result<()> {
        self.writes.lock().unwrap().push((name.to_string(), value));
        Ok(())
    }
}
