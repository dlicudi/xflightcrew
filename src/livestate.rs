//! Live aircraft-state cache over the X-Plane WebSocket API.
//!
//! The REST API reads one dataref per request, so the continuously-read state —
//! the command-loop snapshot, the dashboard, the monitor — pays a burst of
//! round-trips every time. The WebSocket API instead lets us **subscribe** to
//! many datarefs once and have the sim *push* changes, so a read becomes an
//! instant local cache lookup with no round-trip.
//!
//! Ported from the proven client in the sibling `rustdecks` project
//! (`rustdecks/src/sim.rs`): one thread owns the socket and REST resolution,
//! callers just read the cache. Sync (`tungstenite` + `reqwest::blocking`) on its
//! own thread, matching XVoiceLayer's other background pollers — no async runtime.
//!
//! Best-effort: if X-Plane is unreachable or the socket drops, the cache simply
//! stops updating and callers fall back to a REST read (see `state_snapshot`), so
//! this is never worse than polling.

use std::collections::HashMap;
use std::io::ErrorKind;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde_json::{json, Value};
use tungstenite::stream::MaybeTlsStream;
use tungstenite::Message;

/// Read timeout on the socket so the loop wakes regularly to handle pings/close
/// even when no values are changing. Kept short because PTT press/release is read
/// from this cache: a long timeout would delay the mic stopping on release and
/// the barge-in firing on press. 20ms (~50Hz) is well under one X-Plane frame.
const WS_READ_TIMEOUT: Duration = Duration::from_millis(20);

/// How long to wait before re-connecting after the socket drops (X-Plane
/// restart, network blip). Short enough to recover quickly, long enough not to
/// hammer a down sim.
const RECONNECT_DELAY: Duration = Duration::from_secs(2);

/// A cache of the latest pushed value per subscribed dataref name. Cloneable
/// handle; the owning thread updates it in the background.
#[derive(Clone)]
pub struct LiveState {
    cache: Arc<Mutex<HashMap<String, f64>>>,
}

impl LiveState {
    /// Subscribe to `names` and start streaming their values into the cache on a
    /// background thread. Returns immediately; the cache fills as updates arrive.
    /// `base_url` is the X-Plane Web API root, e.g. `http://127.0.0.1:8086`.
    pub fn start(base_url: &str, names: &[&'static str]) -> LiveState {
        let cache: Arc<Mutex<HashMap<String, f64>>> = Arc::new(Mutex::new(HashMap::new()));
        let base = base_url.trim_end_matches('/').to_string();
        let names: Vec<String> = names.iter().map(|s| s.to_string()).collect();
        let cache_bg = Arc::clone(&cache);
        std::thread::spawn(move || run(base, names, cache_bg));
        LiveState { cache }
    }

    /// The latest value pushed for `name`, or `None` if nothing has arrived yet
    /// (caller should fall back to a REST read).
    pub fn get(&self, name: &str) -> Option<f64> {
        self.cache.lock().ok()?.get(name).copied()
    }

    /// A pre-filled state with no socket — for tests that exercise consumers
    /// (PTT source routing, governors) against fixed dataref values.
    #[cfg(test)]
    pub fn from_pairs(pairs: &[(&str, f64)]) -> LiveState {
        let cache = pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect();
        LiveState {
            cache: Arc::new(Mutex::new(cache)),
        }
    }
}

/// The socket thread: (re)connect for the whole session. Each `connect_and_pump`
/// runs until the socket drops; on drop we **clear the cache** — so a stale value
/// can't linger (critical for PTT: a frozen "held" would keep the mic open with
/// the button released; cleared, `get` returns `None` and the reader fails safe)
/// — then retry. X-Plane reassigns dataref ids across restarts, so each attempt
/// re-resolves them.
fn run(base: String, names: Vec<String>, cache: Arc<Mutex<HashMap<String, f64>>>) {
    let Ok(http) = reqwest::blocking::Client::builder()
        .timeout(Duration::from_secs(5))
        .build()
    else {
        return;
    };
    loop {
        connect_and_pump(&http, &base, &names, &cache);
        // Socket dropped or never connected — drop stale values so readers fall
        // back (REST snapshot) or fail safe (PTT reads released), then retry.
        if let Ok(mut c) = cache.lock() {
            c.clear();
        }
        std::thread::sleep(RECONNECT_DELAY);
    }
}

/// One connection lifecycle: resolve ids, subscribe, pump updates into the cache
/// until the socket drops (then returns so the caller can reconnect).
fn connect_and_pump(
    http: &reqwest::blocking::Client,
    base: &str,
    names: &[String],
    cache: &Arc<Mutex<HashMap<String, f64>>>,
) {
    let version = detect_version(http, base);
    let rest = format!("{base}/api/{version}");

    // Resolve each name to its numeric id, keeping the reverse map to label
    // incoming updates (which are keyed by id).
    let mut id_to_name: HashMap<i64, String> = HashMap::new();
    for name in names {
        if let Some(id) = resolve_id(http, &rest, name) {
            id_to_name.insert(id, name.clone());
        } else {
            tracing::debug!(dataref = %name, "livestate: could not resolve id (will REST-fallback)");
        }
    }
    if id_to_name.is_empty() {
        return;
    }
    let ids: Vec<i64> = id_to_name.keys().copied().collect();

    // http -> ws, https -> wss.
    let ws_url = format!("{}/api/{version}", base.replacen("http", "ws", 1));
    let Ok((mut socket, _)) = tungstenite::connect(&ws_url) else {
        tracing::warn!(url = %ws_url, "livestate: WebSocket connect failed; will retry");
        return;
    };
    if let MaybeTlsStream::Plain(s) = socket.get_mut() {
        let _ = s.set_read_timeout(Some(WS_READ_TIMEOUT));
    }
    if socket.send(Message::Text(subscribe_msg(&ids, 1))).is_err() {
        return;
    }
    tracing::info!(
        datarefs = id_to_name.len(),
        "livestate: subscribed over WebSocket"
    );

    loop {
        match socket.read() {
            Ok(Message::Text(text)) => {
                for (id, value) in parse_updates(&text) {
                    if let Some(name) = id_to_name.get(&id) {
                        if let Ok(mut c) = cache.lock() {
                            c.insert(name.clone(), value);
                        }
                    }
                }
            }
            Ok(Message::Ping(p)) => {
                let _ = socket.send(Message::Pong(p));
            }
            Ok(Message::Close(_)) => return,
            Ok(_) => {}
            // Read timeout — normal; loop so we keep handling pings/close.
            Err(tungstenite::Error::Io(e))
                if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
            Err(_) => return, // socket dropped — return so run() reconnects
        }
    }
}

/// The newest API version the server advertises (`/api/capabilities`), falling
/// back to `v2` (the first with the WebSocket value API).
fn detect_version(http: &reqwest::blocking::Client, base: &str) -> String {
    let fallback = "v2".to_string();
    let Ok(resp) = http.get(format!("{base}/api/capabilities")).send() else {
        return fallback;
    };
    let Ok(body) = resp.json::<Value>() else {
        return fallback;
    };
    body.get("api")
        .and_then(|a| a.get("versions"))
        .and_then(Value::as_array)
        .and_then(|v| v.last())
        .and_then(Value::as_str)
        .map(String::from)
        .unwrap_or(fallback)
}

/// Resolve a dataref name to its numeric id via REST.
fn resolve_id(http: &reqwest::blocking::Client, rest: &str, name: &str) -> Option<i64> {
    let url = format!("{rest}/datarefs");
    let body: Value = http
        .get(&url)
        .query(&[("filter[name]", name)])
        .send()
        .ok()?
        .json()
        .ok()?;
    body.get("data")?.as_array()?.first()?.get("id")?.as_i64()
}

/// Build the `dataref_subscribe_values` message for a set of ids.
fn subscribe_msg(ids: &[i64], req_id: u64) -> String {
    let datarefs: Vec<Value> = ids.iter().map(|id| json!({ "id": id })).collect();
    json!({
        "type": "dataref_subscribe_values",
        "req_id": req_id,
        "params": { "datarefs": datarefs },
    })
    .to_string()
}

/// Parse a `dataref_update_values` message into `(id, scalar value)` pairs. Other
/// message types (results, command updates) yield nothing. Booleans map to 0/1
/// and arrays take their first element (the snapshot only reads scalars).
fn parse_updates(text: &str) -> Vec<(i64, f64)> {
    let Ok(msg) = serde_json::from_str::<Value>(text) else {
        return Vec::new();
    };
    if msg.get("type").and_then(Value::as_str) != Some("dataref_update_values") {
        return Vec::new();
    }
    let Some(data) = msg.get("data").and_then(Value::as_object) else {
        return Vec::new();
    };
    data.iter()
        .filter_map(|(k, v)| Some((k.parse::<i64>().ok()?, json_scalar(v)?)))
        .collect()
}

/// A JSON value as a scalar f64: number as-is, bool as 0/1, array as its first
/// numeric element. Strings/objects yield `None`.
fn json_scalar(v: &Value) -> Option<f64> {
    if let Some(n) = v.as_f64() {
        return Some(n);
    }
    if let Some(b) = v.as_bool() {
        return Some(if b { 1.0 } else { 0.0 });
    }
    if let Some(arr) = v.as_array() {
        return arr.first().and_then(Value::as_f64);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_update_values_into_scalars() {
        let text = r#"{"type":"dataref_update_values","data":{"42":3.5,"7":[1.0,0.0],"9":true}}"#;
        let mut ups = parse_updates(text);
        ups.sort_by_key(|(id, _)| *id);
        assert_eq!(ups, vec![(7, 1.0), (9, 1.0), (42, 3.5)]); // array→first, bool→1
    }

    #[test]
    fn ignores_non_update_messages() {
        assert!(parse_updates(r#"{"type":"result","req_id":1,"success":true}"#).is_empty());
        assert!(parse_updates("not json").is_empty());
    }

    #[test]
    fn subscribe_message_has_the_ids() {
        let msg: Value = serde_json::from_str(&subscribe_msg(&[11, 22], 1)).unwrap();
        assert_eq!(msg["type"], "dataref_subscribe_values");
        let refs = msg["params"]["datarefs"].as_array().unwrap();
        assert_eq!(refs.len(), 2);
        assert_eq!(refs[0]["id"], 11);
        assert_eq!(refs[1]["id"], 22);
    }
}
