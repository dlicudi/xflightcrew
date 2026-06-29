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
use std::time::Duration;

use base64::engine::general_purpose::STANDARD;
use base64::Engine;
use serde_json::json;

/// Plugin datarefs the panel reads (registered writable by the plugin).
const DR_PHASE: &str = "xcopilot/status/phase";
const DR_HEARD: &str = "xcopilot/status/heard";
const DR_ACTION: &str = "xcopilot/status/action";
const DR_ALERT: &str = "xcopilot/status/alert";
const DR_AIRCRAFT: &str = "xcopilot/status/aircraft";

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

/// The writer thread: resolve ids once, then PATCH each update as it arrives.
fn run(base: String, rx: Receiver<Update>) {
    let Ok(http) = reqwest::blocking::Client::builder()
        .timeout(Duration::from_secs(2))
        .build()
    else {
        return;
    };
    let rest = format!("{base}/api/v1");

    // Resolve once. If the plugin isn't loaded these stay None and we drop
    // updates — but keep draining the channel so senders never block.
    let phase_id = resolve_id(&http, &rest, DR_PHASE);
    let heard_id = resolve_id(&http, &rest, DR_HEARD);
    let action_id = resolve_id(&http, &rest, DR_ACTION);
    let alert_id = resolve_id(&http, &rest, DR_ALERT);
    let aircraft_id = resolve_id(&http, &rest, DR_AIRCRAFT);
    if phase_id.is_none() && heard_id.is_none() && action_id.is_none() {
        tracing::debug!("plugin_status: status datarefs not found (plugin not loaded?)");
    }

    // Re-send the phase on idle as a heartbeat, so the plugin can tell "app
    // connected but idle" from "app not running" (and show NOT CONNECTED for the
    // latter). When the app exits, the channel drops and the heartbeat stops.
    use std::sync::mpsc::RecvTimeoutError;
    const HEARTBEAT: Duration = Duration::from_secs(1);
    let mut last_phase = 0u8;
    loop {
        let (id, value) = match rx.recv_timeout(HEARTBEAT) {
            Ok(Update::Phase(code)) => {
                last_phase = code;
                (phase_id, json!(code))
            }
            Ok(Update::Heard(text)) => (heard_id, json!(STANDARD.encode(text.as_bytes()))),
            Ok(Update::Action(text)) => (action_id, json!(STANDARD.encode(text.as_bytes()))),
            Ok(Update::Alert(level, text)) => {
                // Prefix a severity marker so the plugin can colour the chip.
                let payload = format!("{}{}", level.code(), text);
                (alert_id, json!(STANDARD.encode(payload.as_bytes())))
            }
            Ok(Update::Aircraft(code)) => (aircraft_id, json!(STANDARD.encode(code.as_bytes()))),
            Err(RecvTimeoutError::Timeout) => (phase_id, json!(last_phase)), // heartbeat
            Err(RecvTimeoutError::Disconnected) => break,
        };
        let Some(id) = id else { continue };
        let url = format!("{rest}/datarefs/{id}/value");
        let _ = http.patch(&url).json(&json!({ "data": value })).send();
    }
}

/// Resolve a dataref name to its numeric id via REST, or `None`.
fn resolve_id(http: &reqwest::blocking::Client, rest: &str, name: &str) -> Option<i64> {
    let body: serde_json::Value = http
        .get(format!("{rest}/datarefs"))
        .query(&[("filter[name]", name)])
        .send()
        .ok()?
        .json()
        .ok()?;
    body.get("data")?.as_array()?.first()?.get("id")?.as_i64()
}
