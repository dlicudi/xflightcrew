//! Push copilot status into the X-Plane plugin's display datarefs.
//!
//! The bundled plugin ([plugin/](../plugin)) registers writable datarefs and
//! draws them in a compact in-sim panel. This is the *write* side: a background
//! thread that resolves those datarefs' ids once and PATCHes status into them
//! over the same Web API the rest of the app uses — so the loop never blocks on
//! HTTP just to update a HUD.
//!
//! Best-effort: if the plugin isn't installed (ids don't resolve) or X-Plane is
//! unreachable, updates are dropped silently — the copilot runs fine without the
//! display.

use std::sync::mpsc::{Receiver, Sender};
use std::time::{Duration, Instant};

use base64::engine::general_purpose::STANDARD;
use base64::Engine;
use serde_json::json;

/// Plugin datarefs the panel reads (registered writable by the plugin).
const DR_PHASE: &str = "xcopilot/status/phase";
const DR_HEARD: &str = "xcopilot/status/heard";
const DR_ACTION: &str = "xcopilot/status/action";
const DR_ALERT: &str = "xcopilot/status/alert";
const DR_AIRCRAFT: &str = "xcopilot/status/aircraft";
const DR_STRIP: &str = "xcopilot/status/strip";

/// Keep text within the plugin's fixed display buffer (512 bytes, NUL-terminated)
/// — long enough for a full engine/weather summary readback.
const TEXT_MAX: usize = 500;

/// One status change to mirror into the sim.
enum Update {
    /// Activity code: 0 idle, 1 listening, 2 thinking, 3 speaking.
    Phase(u8),
    /// Last transcript heard.
    Heard(String),
    /// Last action read back.
    Action(String),
    /// A monitor callout or command-loop warning, with its severity.
    Alert(AlertLevel, String),
    /// The active aircraft's ICAO code, shown beside the connection chip.
    Aircraft(String),
    /// The live flight strip — one compact status row (emergency phase/runway,
    /// cabin altitude, endurance), refreshed continuously. Empty hides it.
    Strip(String),
}

/// Severity of a panel alert — drives the chip colour/label in the plugin.
#[derive(Clone, Copy)]
pub enum AlertLevel {
    Advisory,
    Caution,
    Warning,
}

impl AlertLevel {
    /// Single-char marker prefixed to the alert payload so the plugin can colour
    /// it without a second (race-prone) dataref.
    fn code(self) -> char {
        match self {
            AlertLevel::Advisory => '1',
            AlertLevel::Caution => '2',
            AlertLevel::Warning => '3',
        }
    }
}

/// Cloneable handle the [`crate::ui::Reporter`] uses to mirror status. Sends are
/// non-blocking; a dropped/failed writer just means no HUD.
#[derive(Clone)]
pub struct StatusSink(Sender<Update>);

impl StatusSink {
    /// Start the background writer for the plugin at `base_url`. Returns a sink
    /// even if X-Plane is down — it simply no-ops until (or unless) the datarefs
    /// resolve.
    pub fn start(base_url: &str) -> StatusSink {
        let (tx, rx) = std::sync::mpsc::channel::<Update>();
        let base = base_url.trim_end_matches('/').to_string();
        std::thread::spawn(move || run(base, rx));
        StatusSink(tx)
    }

    /// Activity code: 0 idle, 1 listening, 2 thinking, 3 speaking.
    pub fn phase(&self, code: u8) {
        let _ = self.0.send(Update::Phase(code));
    }
    pub fn heard(&self, text: &str) {
        let _ = self.0.send(Update::Heard(clip(text)));
    }
    pub fn action(&self, text: &str) {
        let _ = self.0.send(Update::Action(clip(text)));
    }
    pub fn alert(&self, level: AlertLevel, text: &str) {
        let _ = self.0.send(Update::Alert(level, clip(text)));
    }
    /// The active aircraft's ICAO code (e.g. "MD82"), uppercased for the panel chip.
    pub fn aircraft(&self, code: &str) {
        let _ = self.0.send(Update::Aircraft(code.trim().to_uppercase()));
    }
    /// Refresh the live flight strip. Empty text hides the row.
    pub fn strip(&self, text: &str) {
        let _ = self.0.send(Update::Strip(clip(text)));
    }
}

/// Scrub em/en dashes (the panel font can't render them) and trim to the plugin
/// buffer, on a char boundary so base64 of the bytes stays valid UTF-8.
fn clip(s: &str) -> String {
    let s = crate::phrase::scrub_dashes(s);
    if s.len() <= TEXT_MAX {
        return s;
    }
    let mut end = TEXT_MAX;
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    s[..end].to_string()
}

/// Which status dataref an update targets — so the id can be re-looked-up after
/// a re-resolve (X-Plane reassigns ids on restart).
#[derive(Clone, Copy)]
enum Slot {
    Phase,
    Heard,
    Action,
    Alert,
    Aircraft,
    Strip,
}

/// The resolved numeric ids of the plugin's status datarefs. All `None` until a
/// successful resolve (plugin not loaded yet, or X-Plane down).
#[derive(Default)]
struct Ids {
    phase: Option<i64>,
    heard: Option<i64>,
    action: Option<i64>,
    alert: Option<i64>,
    aircraft: Option<i64>,
    strip: Option<i64>,
}

impl Ids {
    fn get(&self, slot: Slot) -> Option<i64> {
        match slot {
            Slot::Phase => self.phase,
            Slot::Heard => self.heard,
            Slot::Action => self.action,
            Slot::Alert => self.alert,
            Slot::Aircraft => self.aircraft,
            Slot::Strip => self.strip,
        }
    }
    /// True once at least one dataref resolved (i.e. the plugin is present).
    fn any(&self) -> bool {
        self.phase.is_some() || self.heard.is_some() || self.action.is_some()
    }
}

/// Resolve all status dataref ids by name. Cheap (5 small GETs) and only done at
/// startup or after the link drops.
fn resolve_all(http: &reqwest::blocking::Client, rest: &str) -> Ids {
    Ids {
        phase: resolve_id(http, rest, DR_PHASE),
        heard: resolve_id(http, rest, DR_HEARD),
        action: resolve_id(http, rest, DR_ACTION),
        alert: resolve_id(http, rest, DR_ALERT),
        aircraft: resolve_id(http, rest, DR_AIRCRAFT),
        strip: resolve_id(http, rest, DR_STRIP),
    }
}

/// The writer thread: resolve ids, then PATCH each update as it arrives —
/// re-resolving whenever the link drops so the panel recovers on its own after
/// an X-Plane restart (which reassigns dataref ids) without restarting the app.
fn run(base: String, rx: Receiver<Update>) {
    let Ok(http) = reqwest::blocking::Client::builder()
        .timeout(Duration::from_secs(2))
        .build()
    else {
        return;
    };
    let rest = format!("{base}/api/v1");

    let mut ids = resolve_all(&http, &rest);
    if !ids.any() {
        tracing::debug!("plugin_status: status datarefs not found (plugin not loaded, or X-Plane down)");
    }
    // `healthy` tracks whether our cached ids are landing. A failed PATCH (or
    // ids that won't resolve) flips it false; while unhealthy we re-resolve on a
    // throttled retry, so a returning X-Plane reconnects within a few seconds.
    let mut healthy = ids.any();
    let mut last_resolve = Instant::now();
    const RESOLVE_RETRY: Duration = Duration::from_secs(3);

    // Re-send the phase on idle as a heartbeat, so the plugin can tell "app
    // connected but idle" from "app not running" (and show NOT CONNECTED for the
    // latter). When the app exits, the channel drops and the heartbeat stops.
    use std::sync::mpsc::RecvTimeoutError;
    const HEARTBEAT: Duration = Duration::from_secs(1);
    let mut last_phase = 0u8;
    loop {
        let (slot, value) = match rx.recv_timeout(HEARTBEAT) {
            Ok(Update::Phase(code)) => {
                last_phase = code;
                (Slot::Phase, json!(code))
            }
            Ok(Update::Heard(text)) => (Slot::Heard, json!(STANDARD.encode(text.as_bytes()))),
            Ok(Update::Action(text)) => (Slot::Action, json!(STANDARD.encode(text.as_bytes()))),
            Ok(Update::Alert(level, text)) => {
                // Prefix a severity marker so the plugin can colour the chip.
                let payload = format!("{}{}", level.code(), text);
                (Slot::Alert, json!(STANDARD.encode(payload.as_bytes())))
            }
            Ok(Update::Aircraft(code)) => (Slot::Aircraft, json!(STANDARD.encode(code.as_bytes()))),
            Ok(Update::Strip(text)) => (Slot::Strip, json!(STANDARD.encode(text.as_bytes()))),
            Err(RecvTimeoutError::Timeout) => (Slot::Phase, json!(last_phase)), // heartbeat
            Err(RecvTimeoutError::Disconnected) => break,
        };
        // Link down (or never up)? Re-resolve ids on a throttled retry — X-Plane
        // hands out new ids across restarts, so the stale ones must be dropped.
        if !healthy && last_resolve.elapsed() >= RESOLVE_RETRY {
            ids = resolve_all(&http, &rest);
            last_resolve = Instant::now();
        }
        let Some(id) = ids.get(slot) else {
            healthy = false;
            continue;
        };
        let url = format!("{rest}/datarefs/{id}/value");
        // A successful PATCH means our ids are still valid; any failure (X-Plane
        // gone, stale id → 404) marks us unhealthy so the next heartbeat retries.
        healthy = http
            .patch(&url)
            .json(&json!({ "data": value }))
            .send()
            .map(|r| r.status().is_success())
            .unwrap_or(false);
    }
}

/// Resolve a dataref name to its numeric id via REST, or `None`. Shared with the
/// panel-request poller in main.rs.
pub(crate) fn resolve_id(http: &reqwest::blocking::Client, rest: &str, name: &str) -> Option<i64> {
    let body: serde_json::Value = http
        .get(format!("{rest}/datarefs"))
        .query(&[("filter[name]", name)])
        .send()
        .ok()?
        .json()
        .ok()?;
    body.get("data")?.as_array()?.first()?.get("id")?.as_i64()
}
