//! The background pollers of the in-sim panel: button presses and typed lines
//! (`xcopilot/request`, `xcopilot/typed`), the voice picker and the settings
//! switches. Each runs on its own thread with a blocking client and hands
//! its result to the command loop. Split out of main.rs on 2026-09-16.

use super::*;

/// Poll the plugin panel's request dataref (`xcopilot/request`) and turn button
/// presses into commands on the typed-command channel — the phrases live in
/// [`auto_status::REQUEST_PHRASES`], the same English a pilot would speak, so
/// a button can never do anything the voice can't. CANCEL (code 11) has no
/// fixed phrase: the cancel is per-automation in the grammar, so the phrase is
/// picked from whatever is actually running. The value is consumed (written
/// back to 0) once injected, so a press fires exactly once. Best-effort like the
/// status sink: no plugin / no X-Plane just means the buttons do nothing.
///
/// The same thread polls the panel's typed line (`xcopilot/typed`, a text
/// dataref the Transcript tab writes on Enter): the words go on the same
/// queue verbatim, consumed by writing a single NUL back (the plugin reads
/// that as empty). A plugin without the dataref (older build) simply never
/// resolves it, and the buttons keep working.
pub(crate) fn spawn_panel_request_poller(base_url: String, tx: std::sync::mpsc::Sender<String>) {
    std::thread::spawn(move || {
        let Ok(http) = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(2))
            .build()
        else {
            return;
        };
        let rest = format!("{}/api/v1", base_url.trim_end_matches('/'));
        let mut id: Option<i64> = None;
        let mut typed_id: Option<i64> = None;
        loop {
            std::thread::sleep(std::time::Duration::from_millis(300));
            if id.is_none() {
                id = crate::plugin_status::resolve_id(&http, &rest, "xcopilot/request");
                if id.is_none() {
                    // Plugin not loaded / sim down — retry lazily.
                    std::thread::sleep(std::time::Duration::from_secs(3));
                    continue;
                }
                typed_id = crate::plugin_status::resolve_id(&http, &rest, "xcopilot/typed");
            }
            if let Some(tid) = typed_id {
                if let Some(text) = take_typed(&http, &rest, tid) {
                    tracing::info!(typed = %text, "panel typed command");
                    if tx.send(text).is_err() {
                        return; // session gone
                    }
                }
            }
            let Some(dref) = id else { continue };
            let url = format!("{rest}/datarefs/{dref}/value");
            let value = http
                .get(&url)
                .send()
                .ok()
                .filter(|r| r.status().is_success())
                .and_then(|r| r.json::<serde_json::Value>().ok())
                .and_then(|b| b.get("data").and_then(|d| d.as_i64()));
            let Some(v) = value else {
                id = None; // stale id (sim restarted) — re-resolve
                continue;
            };
            let cmd = if v == auto_status::REQUEST_CANCEL {
                auto_status::cancel_phrase()
            } else {
                auto_status::REQUEST_PHRASES
                    .iter()
                    .find(|(code, _)| *code == v)
                    .map(|(_, phrase)| *phrase)
            };
            if v != 0 {
                tracing::info!(request = v, cmd, "panel button request");
                // Consume first so a slow channel can't double-fire the
                // request — and even a code with nothing to say (CANCEL with
                // nothing running, an unknown code from a newer plugin) is
                // consumed, or it would re-log every 300 ms forever.
                let _ = http
                    .patch(&url)
                    .json(&serde_json::json!({ "data": 0 }))
                    .send();
            }
            let Some(cmd) = cmd else { continue };
            if tx.send(cmd.to_string()).is_err() {
                return; // session gone
            }
        }
    });
}

/// One poll of the panel's typed line: the text if there is one (consumed),
/// else `None`. Consume FIRST, like the request code, so a slow channel can't
/// inject the same line twice.
pub(crate) fn take_typed(http: &reqwest::blocking::Client, rest: &str, id: i64) -> Option<String> {
    use base64::Engine;
    let url = format!("{rest}/datarefs/{id}/value");
    let raw = http
        .get(&url)
        .send()
        .ok()
        .filter(|r| r.status().is_success())
        .and_then(|r| r.json::<serde_json::Value>().ok())
        .and_then(|b| b.get("data").and_then(|d| d.as_str().map(str::to_owned)))?;
    let bytes = base64::engine::general_purpose::STANDARD.decode(raw).ok()?;
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    let text = String::from_utf8_lossy(&bytes[..end]).trim().to_string();
    if text.is_empty() {
        return None;
    }
    let nul = base64::engine::general_purpose::STANDARD.encode([0u8]);
    let _ = http
        .patch(&url)
        .json(&serde_json::json!({ "data": nul }))
        .send();
    Some(text)
}

/// The panel's copilot-voice picker (see plugin/src/xcopilot.cpp): the app
/// publishes the roster + current index, the Settings combo writes a pick back.
pub(crate) const DR_OPT_VOICE: &str = "xcopilot/opt/voice";

pub(crate) const DR_OPT_VOICE_LIST: &str = "xcopilot/opt/voice_list";

/// The cabin crew's pick from the same roster. `-1` = unpublished.
pub(crate) const DR_OPT_CABIN_VOICE: &str = "xcopilot/opt/cabin_voice";

/// What a cabin-voice pick earns: the two intercom personas must never be
/// one voice (the chime is not enough to tell the back from the right seat).
#[derive(Debug, PartialEq)]
pub(crate) enum CabinPick {
    /// Already applied, or an index the roster lacks.
    Ignore,
    /// The copilot's own voice: refused, the old index republished.
    Refuse,
    Apply(String),
}

pub(crate) fn cabin_pick(v: i64, applied: i64, choices: &[String], pilot_spec: &str) -> CabinPick {
    if v == applied {
        return CabinPick::Ignore;
    }
    match usize::try_from(v).ok().and_then(|i| choices.get(i)) {
        None => CabinPick::Ignore,
        Some(spec) if spec == pilot_spec => CabinPick::Refuse,
        Some(spec) => CabinPick::Apply(spec.clone()),
    }
}

/// The panel's app-mode radio buttons (Both / Copilot only / ATC only) —
/// values are `mode::Mode::panel_index`, `-1` = plugin default (unpublished).
pub(crate) const DR_OPT_MODE: &str = "xcopilot/opt/mode";

/// The panel's cabin-crew radio buttons (Auto / Aboard / Off) — values are
/// `cabin::Staffing::panel_index`, `-1` = plugin default (unpublished).
pub(crate) const DR_OPT_CABIN: &str = "xcopilot/opt/cabin_crew";

/// The panel's UI-scale radio buttons (100% .. 200% in quarter steps, index 0..4) —
/// purely a panel concern; the app remembers the pick and republishes it
/// across plugin reloads and boots. `-1` = plugin default (unpublished).
pub(crate) const DR_OPT_UI_SCALE: &str = "xcopilot/opt/ui_scale";

/// The UI scale the app currently holds for the publish/pick protocol.
pub(crate) static UI_SCALE: std::sync::atomic::AtomicI64 = std::sync::atomic::AtomicI64::new(0);

/// Publish the pilot-voice roster (every piper model on disk) to the plugin's
/// Settings picker, then poll for picks. A pick swaps the copilot's voice live
/// ([`Voice::set_pilot_voice`]), confirms with a short line spoken in the NEW
/// voice, and persists to `prefs.json` so the next session boots with it (env
/// `PIPER_VOICE` still beats the saved pick — see [`config`]). Best-effort like
/// the status sink: no plugin, no X-Plane, or no models just means the picker
/// stays empty. The roster re-publishes whenever the index reads -1 with a pick
/// on record — that's the plugin's freshly-registered default, i.e. a plugin
/// reload or sim restart wiped the panel's state.
///
/// The cabin crew's picker rides the same thread and roster with its own
/// index ([`DR_OPT_CABIN_VOICE`], resolved separately so a plugin built
/// before it existed costs only the cabin's combo).
pub(crate) fn spawn_voice_picker(
    base_url: String,
    voice: Voice,
    current_spec: String,
    current_cabin_spec: String,
) {
    std::thread::spawn(move || {
        use base64::Engine;
        let choices = voice::pilot_voice_choices();
        if choices.is_empty() {
            return;
        }
        // Display names are the model file stems ("en_US-amy-medium") —
        // unambiguous, and quality variants (ryan-high vs ryan-medium) stay
        // distinct. Newline-separated for the plugin's combo.
        let roster: String = choices
            .iter()
            .map(|p| {
                std::path::Path::new(p)
                    .file_stem()
                    .map(|s| s.to_string_lossy().into_owned())
                    .unwrap_or_else(|| p.clone())
            })
            .collect::<Vec<_>>()
            .join("\n");
        let roster_b64 = base64::engine::general_purpose::STANDARD.encode(roster.as_bytes());
        let Ok(http) = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(2))
            .build()
        else {
            return;
        };
        let rest = format!("{}/api/v1", base_url.trim_end_matches('/'));
        // The index the panel should show right now: the active voice's spot in
        // the roster, or -1 when it isn't a roster entry (custom env path).
        let mut applied: i64 = choices
            .iter()
            .position(|c| *c == current_spec)
            .map_or(-1, |i| i as i64);
        let mut ids: Option<(i64, i64)> = None; // (list, index)
        let mut pilot_spec = current_spec.clone();
        let mut cabin_applied: i64 = choices
            .iter()
            .position(|c| *c == current_cabin_spec)
            .map_or(-1, |i| i as i64);
        let mut cabin_id: Option<i64> = None;
        let mut pass: u64 = 0;

        let patch = |id: i64, value: serde_json::Value| -> bool {
            http.patch(format!("{rest}/datarefs/{id}/value"))
                .json(&serde_json::json!({ "data": value }))
                .send()
                .map(|r| r.status().is_success())
                .unwrap_or(false)
        };
        loop {
            std::thread::sleep(std::time::Duration::from_millis(500));
            if ids.is_none() {
                ids = crate::plugin_status::resolve_id(&http, &rest, DR_OPT_VOICE_LIST)
                    .zip(crate::plugin_status::resolve_id(&http, &rest, DR_OPT_VOICE));
                let Some((list_id, idx_id)) = ids else {
                    // Plugin not loaded / sim down — retry lazily.
                    std::thread::sleep(std::time::Duration::from_secs(3));
                    continue;
                };
                if !patch(list_id, serde_json::json!(roster_b64))
                    || !patch(idx_id, serde_json::json!(applied))
                {
                    ids = None;
                    continue;
                }
            }
            let Some((list_id, idx_id)) = ids else {
                continue;
            };
            let read = |id: i64| -> Option<i64> {
                http.get(format!("{rest}/datarefs/{id}/value"))
                    .send()
                    .ok()
                    .filter(|r| r.status().is_success())
                    .and_then(|r| r.json::<serde_json::Value>().ok())
                    .and_then(|b| b.get("data").and_then(|d| d.as_i64()))
            };
            // The cabin's index first: the pilot's handling below leaves the
            // pass early.
            pass += 1;
            if cabin_id.is_none() && pass % 10 == 1 {
                cabin_id = crate::plugin_status::resolve_id(&http, &rest, DR_OPT_CABIN_VOICE)
                    .filter(|id| patch(*id, serde_json::json!(cabin_applied)));
            }
            if let Some(cid) = cabin_id {
                match read(cid) {
                    None => cabin_id = None,
                    // The plugin reloaded: republish.
                    Some(v) if v < 0 => {
                        if cabin_applied >= 0 && !patch(cid, serde_json::json!(cabin_applied)) {
                            cabin_id = None;
                        }
                    }
                    Some(v) => match cabin_pick(v, cabin_applied, &choices, &pilot_spec) {
                        CabinPick::Ignore => {}
                        CabinPick::Refuse => {
                            patch(cid, serde_json::json!(cabin_applied));
                            voice.say(
                                voice::Priority::Advisory,
                                "That's my voice. Pick another for the cabin.",
                            );
                        }
                        CabinPick::Apply(spec) => {
                            cabin_applied = v;
                            tracing::info!(model = %spec, "panel cabin voice pick");
                            voice.set_cabin_voice(spec.clone());
                            prefs::save_cabin_voice(&spec);
                            voice.say_cabin(
                                voice::Priority::Advisory,
                                "Flight deck, cabin crew. New voice, checked in.",
                            );
                        }
                    },
                }
            }
            let picked = read(idx_id);
            let Some(v) = picked else {
                ids = None; // stale ids (sim restarted) — re-resolve
                continue;
            };
            if v < 0 {
                // The plugin's registered default: its state was wiped (plugin
                // reload) while our ids stayed valid — repopulate the picker.
                if applied >= 0
                    && (!patch(list_id, serde_json::json!(roster_b64))
                        || !patch(idx_id, serde_json::json!(applied)))
                {
                    ids = None;
                }
                continue;
            }
            if v == applied || v as usize >= choices.len() {
                continue;
            }
            applied = v;
            let spec = choices[v as usize].clone();
            tracing::info!(model = %spec, "panel voice pick");
            voice.set_pilot_voice(spec.clone());
            prefs::save_pilot_voice(&spec);
            pilot_spec = spec.clone();
            // Confirmed in the new voice itself: the swap applies before the
            // next intercom utterance, which is exactly this line.
            voice.say(voice::Priority::Advisory, "New voice, checked in.");
        }
    });
}

/// Sync the app's Settings-tab toggles with the plugin: the app mode
/// (`xcopilot/opt/mode`) and cabin-crew staffing (`xcopilot/opt/cabin_crew`).
/// Publish the current values so the radio buttons show the truth, and apply
/// a panel pick the moment it lands (mode: the main loop's reconciliation
/// block notices within ~2 s; staffing: read live at the routing sites).
/// Picks persist to prefs.json as the next boot's default. Same lifecycle
/// discipline as the voice picker: `-1` means the plugin reloaded and lost
/// its state, so the truth is re-published rather than read back as a pick.
pub(crate) fn spawn_settings_poller(base_url: String) {
    /// One panel toggle: its dataref, what the app currently holds, and how
    /// to apply a pick (an unknown index is ignored).
    struct PanelSetting {
        dref: &'static str,
        id: Option<i64>,
        current: fn() -> i64,
        apply: fn(i64),
    }
    let mut settings = [
        PanelSetting {
            dref: DR_OPT_MODE,
            id: None,
            current: || mode::get().panel_index(),
            apply: |v| {
                if let Some(m) = mode::Mode::from_panel_index(v) {
                    tracing::info!(mode = m.as_str(), "panel mode pick");
                    mode::set(m);
                    prefs::save_mode(m.as_str());
                }
            },
        },
        PanelSetting {
            dref: DR_OPT_CABIN,
            id: None,
            current: || cabin::staffing().panel_index(),
            apply: |v| {
                if let Some(s) = cabin::Staffing::from_panel_index(v) {
                    tracing::info!(cabin_crew = s.as_str(), "panel cabin staffing pick");
                    cabin::set_staffing(s);
                    prefs::save_cabin_crew(s.as_str());
                }
            },
        },
        PanelSetting {
            dref: DR_OPT_AUDIO_BUS,
            id: None,
            current: || tts::AUDIO_BUS.load(std::sync::atomic::Ordering::Relaxed) as i64,
            apply: |v| {
                // Pinned on by XCOPILOT_AUDIO_BUS: the pick is refused and
                // the poller republishes the truth, so the box snaps back.
                if !tts::set_audio_bus(v > 0) {
                    tracing::info!("audio bus pinned on by XCOPILOT_AUDIO_BUS; panel pick ignored");
                } else {
                    tracing::info!(audio_bus = v > 0, "panel audio bus pick");
                }
            },
        },
        PanelSetting {
            dref: DR_OPT_UI_SCALE,
            id: None,
            current: || UI_SCALE.load(std::sync::atomic::Ordering::Relaxed),
            apply: |v| {
                if (0..=4).contains(&v) {
                    tracing::info!(ui_scale = v, "panel UI scale pick");
                    UI_SCALE.store(v, std::sync::atomic::Ordering::Relaxed);
                    prefs::save_ui_scale(v);
                }
            },
        },
    ];
    // The scale is pure remembered state — seed it from prefs so the first
    // publish shows the saved pick, not a hardcoded default.
    UI_SCALE.store(
        prefs::ui_scale().unwrap_or(0),
        std::sync::atomic::Ordering::Relaxed,
    );
    std::thread::spawn(move || {
        let Ok(http) = reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_secs(2))
            .build()
        else {
            return;
        };
        let rest = format!("{}/api/v1", base_url.trim_end_matches('/'));
        let patch = |id: i64, value: i64| -> bool {
            http.patch(format!("{rest}/datarefs/{id}/value"))
                .json(&serde_json::json!({ "data": value }))
                .send()
                .map(|r| r.status().is_success())
                .unwrap_or(false)
        };
        loop {
            std::thread::sleep(std::time::Duration::from_millis(500));
            for s in settings.iter_mut() {
                let applied = (s.current)();
                if s.id.is_none() {
                    s.id = crate::plugin_status::resolve_id(&http, &rest, s.dref);
                    let Some(id) = s.id else {
                        // Plugin not loaded / sim down — retry lazily.
                        std::thread::sleep(std::time::Duration::from_secs(3));
                        continue;
                    };
                    if !patch(id, applied) {
                        s.id = None;
                        continue;
                    }
                }
                let Some(id) = s.id else { continue };
                let read = http
                    .get(format!("{rest}/datarefs/{id}/value"))
                    .send()
                    .ok()
                    .filter(|r| r.status().is_success())
                    .and_then(|r| r.json::<serde_json::Value>().ok())
                    .and_then(|b| b.get("data").and_then(|d| d.as_i64()));
                let Some(v) = read else {
                    s.id = None; // stale id (sim restarted) — re-resolve
                    continue;
                };
                if v < 0 {
                    // Plugin reloaded and lost its state — re-publish the truth.
                    if !patch(id, applied) {
                        s.id = None;
                    }
                    continue;
                }
                if v != applied {
                    (s.apply)(v);
                }
            }
        }
    });
}

/// Panel settings-popup switches, polled the same way. Missing (no plugin) =
/// the feature's default (both on).
pub(crate) const DR_OPT_SNAP: &str = "xcopilot/opt/camera_snap";

pub(crate) const DR_OPT_TONE: &str = "xcopilot/opt/negative_tone";

pub(crate) const DR_OPT_AUDIO_BUS: &str = "xcopilot/opt/audio_bus";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cabin_pick_is_never_the_copilots_voice() {
        let choices: Vec<String> = ["models/amy.onnx", "models/cori.onnx", "models/hfc.onnx"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        let pilot = "models/amy.onnx";
        assert_eq!(cabin_pick(2, 2, &choices, pilot), CabinPick::Ignore);
        assert_eq!(cabin_pick(7, 2, &choices, pilot), CabinPick::Ignore);
        assert_eq!(cabin_pick(0, 2, &choices, pilot), CabinPick::Refuse);
        assert_eq!(
            cabin_pick(1, 2, &choices, pilot),
            CabinPick::Apply("models/cori.onnx".to_string())
        );
        // A cabin voice from outside the roster (custom env path) shows -1.
        assert_eq!(
            cabin_pick(2, -1, &choices, pilot),
            CabinPick::Apply("models/hfc.onnx".to_string())
        );
    }
}
