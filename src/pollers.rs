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
            // Always localhost: a system proxy lookup is never wanted, and reqwest's
            // default one can stall the first build for seconds (2026-09-27).
            .no_proxy()
            .timeout(std::time::Duration::from_secs(2))
            .build()
        else {
            return;
        };
        let rest = format!("{}/api/v1", base_url.trim_end_matches('/'));
        let mut id: Option<i64> = None;
        let mut typed_id: Option<i64> = None;
        let mut typed_age_id: Option<i64> = None;
        loop {
            std::thread::sleep(std::time::Duration::from_millis(300));
            // A flight is loading: the panel waits (xplane::sim_loading).
            if crate::xplane::sim_loading() {
                continue;
            }
            if id.is_none() {
                id = crate::plugin_status::resolve_id(&http, &rest, "xcopilot/request");
                if id.is_none() {
                    // Plugin not loaded / sim down — retry lazily.
                    std::thread::sleep(std::time::Duration::from_secs(3));
                    continue;
                }
                typed_id = crate::plugin_status::resolve_id(&http, &rest, "xcopilot/typed");
                // An older plugin has no age: the line is still taken.
                typed_age_id = crate::plugin_status::resolve_id(&http, &rest, "xcopilot/typed_age");
            }
            if let Some(tid) = typed_id {
                if let Some((text, age_s)) = take_typed_aged(&http, &rest, tid, typed_age_id) {
                    tracing::info!(typed = %text, age_s, "panel typed command");
                    crate::blackbox::record(
                        "typed",
                        serde_json::json!({ "event": "taken", "text": text,
                                            "age_s": age_s.map(|a| (a * 100.0).round() / 100.0) }),
                    );
                    if !queue_command(&tx, text) {
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
            if !queue_command(&tx, cmd.to_string()) {
                return; // session gone
            }
        }
    });
}

/// When each line on the command channel was queued, oldest first (#90). The
/// command loop takes one line per pass, and a pass that SAYS something used
/// to block until the line was spoken, however long the intercom queue made
/// that; a typed line or a panel button waited behind it. `say` in main.rs
/// stops waiting (the line still plays) while [`command_waiting`], and the
/// loop records how long each line sat ([`took_command`], blackbox `typed`).
static QUEUED: std::sync::Mutex<std::collections::VecDeque<std::time::Instant>> =
    std::sync::Mutex::new(std::collections::VecDeque::new());

fn queued() -> std::sync::MutexGuard<'static, std::collections::VecDeque<std::time::Instant>> {
    QUEUED.lock().unwrap_or_else(|e| e.into_inner())
}

/// Put a pilot's line (typed, a panel button, the terminal) on the command
/// channel, stamped. `false` when the loop is gone.
pub(crate) fn queue_command(tx: &std::sync::mpsc::Sender<String>, text: String) -> bool {
    queued().push_back(std::time::Instant::now());
    if tx.send(text).is_err() {
        queued().pop_back();
        return false;
    }
    true
}

/// Held by a test that stamps lines on the channel ([`QUEUED`] is
/// process-wide): the panel's and the EFB's tests share it.
#[cfg(test)]
pub(crate) static QUEUE_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Is a pilot's line waiting for the command loop?
pub(crate) fn command_waiting() -> bool {
    !queued().is_empty()
}

/// The loop took the oldest line: how long it sat on the channel.
pub(crate) fn took_command() -> Option<std::time::Duration> {
    queued().pop_front().map(|at| at.elapsed())
}

/// [`take_typed`] for the pilot's typed line, with the plugin's own age of it
/// (`xcopilot/typed_age`, seconds since the panel set it), read only when a
/// line is there and before it is consumed. `None` from an older plugin or a
/// failed read: no measurement.
pub(crate) fn take_typed_aged(
    http: &reqwest::blocking::Client,
    rest: &str,
    id: i64,
    age_id: Option<i64>,
) -> Option<(String, Option<f64>)> {
    let text = read_typed(http, rest, id)?;
    let age_s = age_id.and_then(|a| {
        http.get(format!("{rest}/datarefs/{a}/value"))
            .send()
            .ok()
            .filter(|r| r.status().is_success())
            .and_then(|r| r.json::<serde_json::Value>().ok())
            .and_then(|b| b.get("data").and_then(|d| d.as_f64()))
            .filter(|v| v.is_finite() && *v >= 0.0)
    });
    consume_typed(http, rest, id);
    Some((text, age_s))
}

/// One poll of the panel's typed line: the text if there is one (consumed),
/// else `None`. Consume FIRST, like the request code, so a slow channel can't
/// inject the same line twice.
pub(crate) fn take_typed(http: &reqwest::blocking::Client, rest: &str, id: i64) -> Option<String> {
    let text = read_typed(http, rest, id)?;
    consume_typed(http, rest, id);
    Some(text)
}

/// The typed line's text, `None` when empty or unreadable.
fn read_typed(http: &reqwest::blocking::Client, rest: &str, id: i64) -> Option<String> {
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
    Some(text)
}

/// Consume the typed line: a single NUL, which the plugin reads as empty.
fn consume_typed(http: &reqwest::blocking::Client, rest: &str, id: i64) {
    use base64::Engine;
    let url = format!("{rest}/datarefs/{id}/value");
    let nul = base64::engine::general_purpose::STANDARD.encode([0u8]);
    let _ = http
        .patch(&url)
        .json(&serde_json::json!({ "data": nul }))
        .send();
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
            // Always localhost: a system proxy lookup is never wanted, and reqwest's
            // default one can stall the first build for seconds (2026-09-27).
            .no_proxy()
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
            // A flight is loading: the panel waits (xplane::sim_loading).
            if crate::xplane::sim_loading() {
                continue;
            }
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
        /// The value we last wrote to the panel (`None` until the first).
        published: Option<i64>,
        current: fn() -> i64,
        apply: fn(i64),
    }
    let mut settings = [
        PanelSetting {
            dref: DR_OPT_MODE,
            id: None,
            published: None,
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
            published: None,
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
            published: None,
            current: || tts::AUDIO_BUS.load(std::sync::atomic::Ordering::Relaxed) as i64,
            apply: |v| {
                // Through the registry, so the pick is saved (#228: it was
                // applied and never persisted). Pinned by XCOPILOT_AUDIO_BUS:
                // refused, and the poller republishes the truth, so the box
                // snaps back.
                match crate::settings::set("audio_bus", &crate::settings::Value::Bool(v > 0)) {
                    Ok(()) => tracing::info!(audio_bus = v > 0, "panel audio bus pick"),
                    Err(e) => tracing::info!("audio bus panel pick refused: {e}"),
                }
            },
        },
        PanelSetting {
            dref: DR_OPT_UI_SCALE,
            id: None,
            published: None,
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
            // Always localhost: a system proxy lookup is never wanted, and reqwest's
            // default one can stall the first build for seconds (2026-09-27).
            .no_proxy()
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
            // A flight is loading: the panel waits (xplane::sim_loading).
            if crate::xplane::sim_loading() {
                continue;
            }
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
                    s.published = Some(applied);
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
                match panel_sync(v, s.published, applied) {
                    // Agreeing is as good as our own write: the next
                    // difference is judged against this value.
                    PanelSync::Quiet => s.published = Some(v),
                    PanelSync::Pick => {
                        (s.apply)(v);
                        s.published = Some(v);
                    }
                    PanelSync::Publish => {
                        if patch(id, applied) {
                            s.published = Some(applied);
                        } else {
                            s.id = None;
                        }
                    }
                }
            }
        }
    });
}

/// What one pass of the settings poller does with a panel toggle.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum PanelSync {
    /// The panel shows what the app holds.
    Quiet,
    /// The pilot clicked the panel since our last write: apply it.
    Pick,
    /// The panel shows our last write but the app changed since (the
    /// terminal, the EFB) or the plugin reloaded (`-1`): publish the truth.
    Publish,
}

/// Decide a pass from the panel's value `v`, the value we last `published`
/// and the app's `applied` one. A difference alone is not a pick: the
/// terminal's or the EFB's change through the registry would read as one and
/// be reverted by the panel's stale value within half a second. Pure.
pub(crate) fn panel_sync(v: i64, published: Option<i64>, applied: i64) -> PanelSync {
    if v < 0 {
        return PanelSync::Publish;
    }
    if v == applied {
        return PanelSync::Quiet;
    }
    if published == Some(v) {
        PanelSync::Publish
    } else {
        PanelSync::Pick
    }
}

/// The panel's text settings, app to plugin: one row per registry knob of
/// [`settings::Kind::Text`] and [`PANEL_CHOICES`],
/// `key|label|pinned_env|value|notice|help|options` (options comma-separated,
/// empty for a text knob), rows
/// '\n'-separated. `pinned_env` names the variable that pins the knob (the
/// field is then read-only), empty when the panel may edit it; `notice` is
/// the refusal of the last edit, empty after one that took.
pub(crate) const DR_OPT_TEXT: &str = "xcopilot/opt/text";

/// The panel's text edits, plugin to app: `key=value` lines, written on Enter
/// or focus loss and consumed by writing a NUL back (the typed line's
/// protocol). A value may hold '=' ("WAT=Walker"): the key ends at the first.
pub(crate) const DR_OPT_TEXT_EDIT: &str = "xcopilot/opt/text_edit";

/// The `key=value` lines of one consumed edit, in order; a line without a
/// key is ignored.
pub(crate) fn parse_text_edits(raw: &str) -> Vec<(String, String)> {
    raw.lines()
        .filter_map(|l| {
            let (k, v) = l.split_once('=')?;
            let k = k.trim();
            (!k.is_empty()).then(|| (k.to_string(), v.trim().to_string()))
        })
        .collect()
}

/// Apply one panel edit through the registry (validate, apply live, persist;
/// an env-pinned knob is refused naming its variable). `Err` is said on the
/// panel under the field.
pub(crate) fn apply_text_edit(key: &str, raw: &str) -> Result<(), String> {
    let s = settings::ALL
        .iter()
        .find(|s| s.key == key && on_panel_rows(s))
        .ok_or_else(|| format!("no text setting {key}"))?;
    let v = s.kind.parse(raw).ok_or_else(|| match s.kind {
        settings::Kind::Choice(options) => format!("{} is one of {}", s.label, options.join(", ")),
        _ => format!("{} is too long or not one line", s.label),
    })?;
    settings::set(key, &v)
}

/// The choice knobs the panel draws as a dropdown on the text rows, a
/// seventh field naming the options: our own traffic and how busy it is
/// (2026-10-09: the pilot looked for the traffic setting on the panel and
/// it was only in the terminal and the EFB).
pub(crate) const PANEL_CHOICES: &[&str] = &["traffic", "traffic_density"];

/// A registry knob the panel's text rows carry: every text knob, and the
/// [`PANEL_CHOICES`].
fn on_panel_rows(s: &settings::Setting) -> bool {
    match s.kind {
        settings::Kind::Text { .. } => true,
        settings::Kind::Choice(_) => PANEL_CHOICES.contains(&s.key),
        _ => false,
    }
}

/// The [`DR_OPT_TEXT`] payload: every text knob with its live value.
pub(crate) fn text_settings_payload(notices: &std::collections::HashMap<String, String>) -> String {
    let clean = |t: &str| t.replace(['|', '\n'], " ");
    settings::ALL
        .iter()
        .filter(|s| on_panel_rows(s))
        .map(|s| {
            let value = match s.value() {
                settings::Value::Choice(c) => c,
                v => v.as_text().map(str::to_string).unwrap_or_default(),
            };
            let options = match s.kind {
                settings::Kind::Choice(o) => o.join(","),
                _ => String::new(),
            };
            format!(
                "{}|{}|{}|{}|{}|{}|{}",
                s.key,
                clean(s.label),
                if s.env_pinned() { s.env } else { "" },
                clean(&value),
                clean(notices.get(s.key).map(String::as_str).unwrap_or("")),
                clean(s.help),
                options,
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Sync the panel's text fields with the registry: publish every text knob
/// ([`DR_OPT_TEXT`], on change and every 5 s, since a plugin reload zeroes
/// the buffer) and apply the edits the panel sends ([`DR_OPT_TEXT_EDIT`],
/// polled each second: a setting, not a control, so no faster). A plugin
/// built before the text fields resolves neither and costs one lookup per
/// retry.
pub(crate) fn spawn_text_settings_poller(base_url: String) {
    std::thread::spawn(move || {
        use base64::Engine;
        let Ok(http) = reqwest::blocking::Client::builder()
            // Always localhost: a system proxy lookup is never wanted, and reqwest's
            // default one can stall the first build for seconds (2026-09-27).
            .no_proxy()
            .timeout(std::time::Duration::from_secs(2))
            .build()
        else {
            return;
        };
        let rest = format!("{}/api/v1", base_url.trim_end_matches('/'));
        let mut ids: Option<(i64, i64)> = None; // (text, edit)
        let mut notices: std::collections::HashMap<String, String> = Default::default();
        let mut last_sent = String::new();
        let mut sent_at = std::time::Instant::now();
        loop {
            std::thread::sleep(std::time::Duration::from_secs(1));
            // A flight is loading: the panel waits (xplane::sim_loading).
            if crate::xplane::sim_loading() {
                continue;
            }
            if ids.is_none() {
                ids = crate::plugin_status::resolve_id(&http, &rest, DR_OPT_TEXT).zip(
                    crate::plugin_status::resolve_id(&http, &rest, DR_OPT_TEXT_EDIT),
                );
                if ids.is_none() {
                    // Plugin not loaded, sim down, or an older build: retry lazily.
                    std::thread::sleep(std::time::Duration::from_secs(5));
                    continue;
                }
                last_sent.clear();
            }
            let Some((text_id, edit_id)) = ids else {
                continue;
            };
            if let Some(raw) = take_typed(&http, &rest, edit_id) {
                for (key, value) in parse_text_edits(&raw) {
                    match apply_text_edit(&key, &value) {
                        Ok(()) => {
                            notices.remove(&key);
                        }
                        Err(e) => {
                            tracing::info!(key = %key, error = %e, "panel text setting refused");
                            notices.insert(key, e);
                        }
                    }
                }
            }
            let payload = text_settings_payload(&notices);
            if payload != last_sent || sent_at.elapsed() >= std::time::Duration::from_secs(5) {
                let b64 = base64::engine::general_purpose::STANDARD.encode(payload.as_bytes());
                let ok = http
                    .patch(format!("{rest}/datarefs/{text_id}/value"))
                    .json(&serde_json::json!({ "data": b64 }))
                    .send()
                    .map(|r| r.status().is_success())
                    .unwrap_or(false);
                if !ok {
                    ids = None; // stale ids (sim restarted): re-resolve
                    continue;
                }
                last_sent = payload;
                sent_at = std::time::Instant::now();
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

    /// Issue #90: a typed line waited behind the command loop's blocking
    /// readback. Every line on the channel is stamped, so the loop's `say`
    /// can see one waiting and stop waiting for its own line, and each line
    /// is taken with how long it sat, oldest first. The readback in main.rs
    /// waits through this, not through the unconditional `Voice::say`.
    #[test]
    fn a_line_on_the_channel_is_seen_waiting_and_timed() {
        let _q = QUEUE_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tx, rx) = std::sync::mpsc::channel::<String>();
        assert!(!command_waiting());
        assert!(queue_command(&tx, "recover the aircraft".into()));
        assert!(queue_command(&tx, "land the aircraft".into()));
        assert!(command_waiting());
        assert_eq!(rx.try_recv().as_deref(), Ok("recover the aircraft"));
        assert!(took_command().is_some_and(|d| d < std::time::Duration::from_secs(1)));
        assert!(command_waiting(), "the second is still there");
        assert_eq!(rx.try_recv().as_deref(), Ok("land the aircraft"));
        assert!(took_command().is_some());
        assert!(!command_waiting());
        assert_eq!(took_command(), None);
        // A loop that is gone leaves nothing stamped behind.
        drop(rx);
        assert!(!queue_command(&tx, "stop".into()));
        assert!(!command_waiting());

        let main = crate::test_support::module_source("main");
        // #167: and to an armed station swap (`loop_owed`), which only the
        // loop installs.
        assert!(main.contains("say_or(voice, reporter, kind, line, || loop_owed(false))"));
        assert!(main.contains("pilot_line_pending(in_hand), party_line::station_swap_pending()"));
        assert!(main.contains("voice.say_until(Priority::Readback, line, stop)"));
        assert!(!main.contains("voice.say(Priority::Readback, line)"));
        // #135: the crew-radio drain gives way to a line already in hand too,
        // the tower's reply included (it waited out a whole taxi clearance
        // with "my controls" taken and unrouted, LEMG 2026-09-28).
        assert!(main.contains("let pilot_waiting = || loop_owed(input.is_some());"));
        assert!(main.contains("voice.say_station_until("));
    }

    /// A panel edit is `key=value` lines; the value keeps its own '='.
    /// A change made elsewhere (the terminal, the EFB) is published to the
    /// panel, never reverted by the panel's stale value; a click on the
    /// panel is still a pick; a reloaded plugin gets the truth.
    #[test]
    fn a_registry_change_is_published_and_a_panel_click_is_a_pick() {
        assert_eq!(panel_sync(0, Some(0), 0), PanelSync::Quiet);
        assert_eq!(
            panel_sync(0, Some(0), 3),
            PanelSync::Publish,
            "the EFB set instructor: the panel's 0 is ours, not a pick"
        );
        assert_eq!(panel_sync(2, Some(0), 0), PanelSync::Pick);
        assert_eq!(panel_sync(2, Some(0), 3), PanelSync::Pick);
        assert_eq!(panel_sync(-1, Some(0), 0), PanelSync::Publish);
        // A refused pick (the audio bus pinned on): the panel snaps back.
        assert_eq!(panel_sync(0, Some(0), 1), PanelSync::Publish);
    }

    #[test]
    fn text_edits_split_at_the_first_equals() {
        assert_eq!(
            parse_text_edits(
                "airline_telephony=WAT=Walker; XYZ=Zulu\ncallsign= WAT2516 \n=x\njunk"
            ),
            vec![
                (
                    "airline_telephony".to_string(),
                    "WAT=Walker; XYZ=Zulu".to_string()
                ),
                ("callsign".to_string(), "WAT2516".to_string()),
            ]
        );
        assert_eq!(
            parse_text_edits("callsign="),
            vec![("callsign".to_string(), String::new())]
        );
    }

    /// The panel's rows name every text knob, and a pinned one carries its
    /// variable so the field is read-only; an edit to it is refused naming
    /// the variable, and nothing is written.
    #[test]
    fn a_pinned_text_setting_is_published_read_only_and_refused() {
        let _guard = settings::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        // SAFETY: the lock above serialises every env-mutating test.
        unsafe { std::env::set_var("XCOPILOT_AIRLINE_TELEPHONY", "WAT=Walker") };
        settings::update(|c| c.airline_telephony = "WAT=Walker".into());
        let mut notices = std::collections::HashMap::new();
        let err = apply_text_edit("airline_telephony", "XYZ=Zulu").expect_err("pinned");
        assert!(err.contains("XCOPILOT_AIRLINE_TELEPHONY"), "{err}");
        notices.insert("airline_telephony".to_string(), err);
        let payload = text_settings_payload(&notices);
        let row = payload
            .lines()
            .find(|l| l.starts_with("airline_telephony|"))
            .expect("published");
        let f: Vec<&str> = row.split('|').collect();
        assert_eq!(f.len(), 7, "{row}");
        assert_eq!(f[6], "", "a text knob names no options");
        assert_eq!(f[2], "XCOPILOT_AIRLINE_TELEPHONY");
        assert_eq!(f[3], "WAT=Walker");
        assert!(
            f[4].contains("pinned by XCOPILOT_AIRLINE_TELEPHONY"),
            "{row}"
        );
        assert!(
            payload.lines().any(|l| l.starts_with("callsign|")),
            "{payload}"
        );
        assert_eq!(settings::live().airline_telephony, "WAT=Walker");
        unsafe { std::env::remove_var("XCOPILOT_AIRLINE_TELEPHONY") };
        settings::update(|c| c.airline_telephony = String::new());
        assert!(apply_text_edit("mode", "atc").is_err(), "not a text knob");
    }

    /// The traffic knobs ride the panel's rows as dropdowns (2026-10-09:
    /// the pilot looked for them on the panel): their options in the
    /// seventh field, and a pick outside them refused by the registry,
    /// naming the options, with nothing written.
    #[test]
    fn the_traffic_choices_are_published_as_dropdowns() {
        let _guard = settings::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let payload = text_settings_payload(&std::collections::HashMap::new());
        let row = |k: &str| {
            payload
                .lines()
                .find(|l| l.starts_with(&format!("{k}|")))
                .map(|l| l.split('|').map(str::to_string).collect::<Vec<_>>())
        };
        let t = row("traffic").expect("traffic published");
        assert_eq!(t.len(), 7);
        assert_eq!(t[6], "off,real,ours");
        assert!(["off", "real", "ours"].contains(&t[3].as_str()), "{t:?}");
        assert_eq!(row("traffic_density").expect("density published")[6], "off,light,normal");
        let before = settings::live().traffic.clone();
        let err = apply_text_edit("traffic", "livetraffic").expect_err("not an option");
        assert!(err.contains("off, real, ours"), "{err}");
        assert_eq!(settings::live().traffic, before);
        assert!(apply_text_edit("mode", "atc").is_err(), "not on the panel's rows");
    }

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
