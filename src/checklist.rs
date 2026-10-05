//! XChecklist interpreter.
//!
//! Checklists are **not** baked into Rust — they're read from an XChecklist file
//! (the de-facto X-Plane community format, used by the XChecklist plug-in) named
//! by the aircraft profile (`checklist:` in the YAML). This makes the copilot
//! compatible with the whole ecosystem: drop in any aircraft's `.txt` and it
//! runs. We never edit the file; we interpret it as-is.
//!
//! ## What the format gives us
//!
//! Each actionable line is `sw_item:CHALLENGE|RESPONSE[:dataref[:condition]]`.
//! The condition is a *pass test* (what turns the checkbox green), not an
//! action — e.g. `parkbrake:1`, `flap_ratio:<0.02`, `throttle_ratio[0]:0.15|0.20`.
//! So per item the file tells us **what to say**, **how to verify it** (read the
//! dataref, test the condition) and — for the systems we own — **enough to derive
//! a target to act on**.
//!
//! ## Hybrid execution (faithful to the baked-in behaviour it replaces)
//!
//! - Systems we own → [`action_for`] maps the file's dataref to one of our
//!   curated tools, which actuates *and reads back* (gates, command-vs-raw-write
//!   knowledge live in the tool, not here — the file's dataref is only a key).
//! - A dataref we don't actuate → **verify**: read it, confirm or flag.
//! - No dataref (`FUEL QUANTITY|SUFFICIENT`) → **call-out**: the pilot's item.
//! - `sw_remark` / reference lines → narration.

use anyhow::{bail, Context, Result};
use serde_json::{json, Value};

use crate::aircraft::{
    AircraftProfile, ChecklistAction, ChecklistMapEntry, ElectricalSystem, FlapProfile,
    FuelPosition, FuelSelectorProfile, LightProfile,
};
use crate::tools::autopilot::AutopilotOff;
use crate::tools::controls::{SetParkingBrake, SetPitchTrim};
use crate::tools::electrical::SetElectrical;
use crate::tools::engine::{
    SetCarbHeat, SetFuelPump, SetFuelSelector, SetMagnetos, SetMixture, SetThrottle, StartEngine,
    StopEngine,
};
use crate::tools::flaps::SetFlaps;
use crate::tools::instruments::SetPitotHeat;
use crate::tools::lights::SetLights;
use crate::tools::radio::SetTransponderMode;
use crate::tools::{Tool, ToolCtx};
use crate::xplane::XPlaneClient;

/// A pass condition parsed from the file's `dataref:condition` field.
#[derive(Debug, Clone, PartialEq)]
pub enum Cond {
    /// `:N` — equals (enums/booleans; matched with a 0.5 tolerance).
    Eq(f64),
    /// `:>N`
    Gt(f64),
    /// `:<N`
    Lt(f64),
    /// `:!N` — not equal.
    Ne(f64),
    /// `:min|max` — within an inclusive range.
    Range(f64, f64),
}

impl Cond {
    fn parse(s: &str) -> Option<Cond> {
        let s = s.trim();
        if s.is_empty() {
            return None;
        }
        if let Some((a, b)) = s.split_once('|') {
            return Some(Cond::Range(a.trim().parse().ok()?, b.trim().parse().ok()?));
        }
        let (ctor, rest): (fn(f64) -> Cond, &str) = match s.as_bytes()[0] {
            b'>' => (Cond::Gt, &s[1..]),
            b'<' => (Cond::Lt, &s[1..]),
            b'!' => (Cond::Ne, &s[1..]),
            _ => (Cond::Eq, s),
        };
        Some(ctor(rest.trim().parse().ok()?))
    }

    /// Is the condition satisfied by a live dataref value?
    pub fn eval(&self, v: f64) -> bool {
        match *self {
            Cond::Eq(n) => (v - n).abs() < 0.5,
            Cond::Gt(n) => v > n,
            Cond::Lt(n) => v < n,
            Cond::Ne(n) => (v - n).abs() >= 0.5,
            Cond::Range(a, b) => v >= a && v <= b,
        }
    }

    /// A representative value that satisfies the condition — used to derive a
    /// target when acting (e.g. a throttle range → its midpoint).
    pub(crate) fn target(&self) -> f64 {
        match *self {
            Cond::Eq(n) | Cond::Ne(n) => n,
            Cond::Range(a, b) => (a + b) / 2.0,
            // "more than": full if near the top, else just over the line.
            Cond::Gt(n) => {
                if n >= 0.85 {
                    1.0
                } else {
                    (n + 0.05).min(1.0)
                }
            }
            // "less than": zero if near the bottom, else just under the line.
            Cond::Lt(n) => {
                if n <= 0.05 {
                    0.0
                } else {
                    (n - 0.05).max(0.0)
                }
            }
        }
    }

    /// Boolean interpretation for on/off switches (ON when the satisfying value
    /// is non-zero).
    fn is_on(&self) -> bool {
        match *self {
            Cond::Eq(n) => n > 0.5,
            Cond::Ne(n) => n < 0.5, // "not 0" → on
            Cond::Gt(_) => true,
            Cond::Lt(_) => false,
            Cond::Range(a, b) => (a + b) / 2.0 > 0.5,
        }
    }
}

/// One interpreted line of a checklist.
pub enum Item {
    /// Spoken context only — `sw_remark` and reference (`sw_itemvoid`) lines.
    Narrate(String),
    /// The pilot's item — spoken so they know it's theirs, not performed.
    CallOut(String),
    /// A system we own: perform it via a curated tool (which reads back).
    /// `confirm` is `Some(hazard)` for destructive steps (engine start/stop, the
    /// run-up) — the runner asks "Confirm, {hazard}?" and skips on a no.
    Act {
        say: String,
        tool: &'static str,
        args: Value,
        confirm: Option<&'static str>,
        /// The file's own pass test, kept alongside the action so a runner
        /// can read the state FIRST (an item already in state is confirmed,
        /// not re-written under the pilot's hand) and an instructor can watch
        /// for the pilot to satisfy it without ever writing.
        dataref: String,
        index: Option<usize>,
        cond: Cond,
    },
    /// A dataref outside our curated tools: read it and check the state. When
    /// `settable` it's an actionable switch we may write to the condition's target
    /// if it's not already there (then confirm). When `!settable` it's an
    /// observe-only item — a CHECK/test step or a read-only indicator/annunciator
    /// (or a custom `laminar/*` dataref we can't write from outside): read and
    /// confirm if satisfied, otherwise just call it out for the pilot. Never write.
    Verify {
        say: String,
        dataref: String,
        index: Option<usize>,
        cond: Cond,
        settable: bool,
    },
}

/// One interpreted checklist. Carries the profile bits its actions need (lights,
/// flaps), so the runner can dispatch without re-plumbing the profile.
pub struct Checklist {
    pub id: String,
    pub title: String,
    pub items: Vec<Item>,
    pub lights: Vec<LightProfile>,
    pub flaps: Option<FlapProfile>,
    pub electrical: Vec<ElectricalSystem>,
    pub fuel_selector: Option<FuelSelectorProfile>,
    pub engines: usize,
    pub magnetos_per_engine: bool,
    /// Per-airframe park-brake valve sense for `set_parking_brake` (see
    /// [`crate::aircraft::AircraftProfile::park_brake_valve_set`]).
    pub park_brake_valve_set: Option<f64>,
    /// Per-airframe pitot-heat switch override (see
    /// [`crate::aircraft::AircraftProfile::pitot_heat_dataref`]).
    pub pitot_heat_dataref: Option<String>,
}

/// Read and interpret the airframe's XChecklist file into runnable checklists.
/// `Ok(vec![])` if the profile names no checklist file.
pub fn load(profile: &AircraftProfile) -> Result<Vec<Checklist>> {
    let Some(path) = &profile.checklist else {
        return Ok(Vec::new());
    };
    let text =
        std::fs::read_to_string(path).with_context(|| format!("reading checklist file {path}"))?;
    Ok(parse(&text, profile))
}

/// Does the utterance ask for a checklist at all? One place for the whisper
/// wart where the compound splits ("check list"), so the caller's decline
/// logic and [`match_request`] can never disagree on what counts.
pub fn mentions_checklist(transcript: &str) -> bool {
    let t = transcript.to_lowercase();
    t.contains("checklist") || t.contains("check list")
}

/// Deterministic voice match: resolve "run the before takeoff checklist" (in
/// any polite wrapping, whisper warts included) to one of this airframe's
/// checklists. The word "checklist" is REQUIRED, so a bare "climb" or
/// "descent" can never trigger a whole phase by accident. Title words match
/// with prefix tolerance ("start" ~ "starting"), candidates are scored by how
/// much of the title went unmatched, and a tie prefers the "before" section
/// (a bare "takeoff checklist" means BEFORE takeoff, not the roll callouts;
/// same for "landing"). Ambiguity that survives all of that returns `None`,
/// never a guess - the caller declines and names the options.
pub fn match_request<'a>(transcript: &str, checklists: &'a [Checklist]) -> Option<&'a Checklist> {
    let query = request_words(transcript);
    if query.is_empty() {
        return None;
    }
    // (unmatched title words, checklist) for every checklist whose title
    // accounts for EVERY query word - a stray word excludes the candidate.
    let mut scored: Vec<(usize, &Checklist)> = checklists
        .iter()
        .filter_map(|c| {
            let title: Vec<&str> = c.id.split('_').collect();
            let matched = |q: &String| title.iter().any(|t| word_match(q, t));
            query.iter().all(matched).then(|| {
                let hit = title
                    .iter()
                    .filter(|t| query.iter().any(|q| word_match(q, t)))
                    .count();
                (title.len() - hit, c)
            })
        })
        .collect();
    scored.sort_by_key(|(s, _)| *s);
    match scored.as_slice() {
        [] => None,
        [(_, one)] => Some(one),
        [(a, best), (b, _), ..] if a < b => Some(best),
        tied => {
            // Joint best score: take the "before" section if exactly one of the
            // tied leaders has one, otherwise refuse to guess.
            let top = tied.iter().take_while(|(s, _)| *s == tied[0].0);
            let mut before = top.filter(|(_, c)| c.id.starts_with("before_"));
            match (before.next(), before.next()) {
                (Some((_, one)), None) => Some(one),
                _ => None,
            }
        }
    }
}

/// The utterance reduced to the words that could name a checklist: lowercase,
/// whisper-split compounds rejoined ("check list", "take off", "pre flight",
/// "shut down"), and the filler of a spoken request stripped. Empty when the
/// utterance never says "checklist" - the required trigger word.
fn request_words(transcript: &str) -> Vec<String> {
    let text = transcript.to_lowercase();
    // Single letters are apostrophe shrapnel ("let's" splits to "let", "s"),
    // never part of a section name - drop them with the empties.
    let mut words: Vec<String> = text
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| w.len() > 1)
        .map(str::to_string)
        .collect();
    let mut i = 0;
    while i + 1 < words.len() {
        let joined = match (words[i].as_str(), words[i + 1].as_str()) {
            ("check", "list" | "lists") => Some("checklist"),
            ("take", "off") => Some("takeoff"),
            ("pre", "flight") => Some("preflight"),
            ("shut", "down") => Some("shutdown"),
            _ => None,
        };
        if let Some(j) = joined {
            words[i] = j.to_string();
            words.remove(i + 1);
        }
        i += 1;
    }
    if !words.iter().any(|w| w == "checklist" || w == "checklists") {
        return Vec::new();
    }
    const FILLER: &[&str] = &[
        "checklist",
        "checklists",
        "run",
        "running",
        "do",
        "read",
        "give",
        "perform",
        "complete",
        "execute",
        "begin",
        "go",
        "through",
        "over",
        "the",
        "a",
        "an",
        "my",
        "me",
        "us",
        "please",
        "let",
        "lets",
        "now",
        "for",
        "to",
        "and",
        "then",
        "okay",
        "ok",
        "roger",
        "it",
        "its",
        "is",
        "time",
        "ready",
        "with",
        "you",
        "your",
        "we",
        "can",
        "could",
    ];
    words.retain(|w| !FILLER.contains(&w.as_str()));
    words
}

/// A query word matches a title word exactly, or by prefix when the shorter of
/// the two carries at least four letters - "start" reaches "starting",
/// "engine" reaches "engines", but "off" can't reach "takeoff".
fn word_match(q: &str, t: &str) -> bool {
    q == t || (q.len() >= 4 && t.starts_with(q)) || (t.len() >= 4 && q.starts_with(t))
}

/// Parse XChecklist text against an airframe profile (its `checklist_map` drives
/// which datarefs become actions). Tolerant: unknown directives and malformed
/// lines are skipped rather than failing the whole file.
pub fn parse(text: &str, profile: &AircraftProfile) -> Vec<Checklist> {
    let lights = profile.lights.clone();
    let flaps = profile.flaps.clone();
    let electrical = &profile.electrical;
    let fuel_selector = &profile.fuel_selector;
    // The selector's positions (with their dataref values) let `action_for`
    // decode a file condition like `fuel_tank_selector:1` into the right position
    // for *this* airframe (172: 4=both; SR22: 0=off/1=left/2=right).
    let fuel_positions: &[FuelPosition] = fuel_selector
        .as_ref()
        .map_or(&[], |f| f.positions.as_slice());
    let map = &profile.checklist_map;
    let mut out: Vec<Checklist> = Vec::new();
    let mut cur: Option<Checklist> = None;
    // The "MAGNETO TEST" remark arms a one-time confirm on the next actioned
    // step (the throttle-up that starts the run-up).
    let mut arm_runup = false;

    let finish = |out: &mut Vec<Checklist>, cur: &mut Option<Checklist>| {
        if let Some(c) = cur.take() {
            if !c.items.is_empty() {
                out.push(c);
            }
        }
    };

    for raw in text.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let Some((dir, rest)) = line.split_once(':') else {
            continue;
        };
        // Coloured variants (`sw_item_c`, `sw_itemvoid_c`, `sw_remark_c`) behave
        // exactly like the plain directive — the colour is display-only and gets
        // stripped from the text. `sw_define_colour`/`sw_continue` keep their
        // names (no `_c` suffix) and fall through to "ignored".
        let dir = dir.strip_suffix("_c").unwrap_or(dir);
        match dir {
            "sw_checklist" => {
                finish(&mut out, &mut cur);
                arm_runup = false; // don't let an arm leak across sections
                                   // `MENU:TITLE` — the title is what we name and speak.
                let title = rest.split_once(':').map(|(_, t)| t).unwrap_or(rest).trim();
                cur = Some(Checklist {
                    id: slug(title),
                    title: humanize(title),
                    items: Vec::new(),
                    lights: lights.clone(),
                    flaps: flaps.clone(),
                    electrical: electrical.clone(),
                    fuel_selector: fuel_selector.clone(),
                    engines: profile.engines,
                    magnetos_per_engine: profile.magnetos_per_engine,
                    park_brake_valve_set: profile.park_brake_valve_set,
                    pitot_heat_dataref: profile.pitot_heat_dataref.clone(),
                });
            }
            // Both narration directives: arm the run-up confirm when the file
            // announces the magneto test (some files use a remark, some an
            // itemvoid header), then speak any substantive content.
            "sw_remark" | "sw_itemvoid" => {
                if let Some(c) = cur.as_mut() {
                    if rest.to_lowercase().contains("magneto") {
                        arm_runup = true;
                    }
                    if let Some(text) = clean_narration(rest) {
                        c.items.push(Item::Narrate(text));
                    }
                }
            }
            "sw_item" => {
                if let Some(c) = cur.as_mut() {
                    let mut item = parse_item(rest, map, fuel_positions);
                    // Arm the run-up confirm on the first actioned step after the
                    // magneto-test remark (the throttle-up).
                    if arm_runup {
                        if let Item::Act { confirm, .. } = &mut item {
                            *confirm = Some("run the mag check and advance the throttle");
                            arm_runup = false;
                        }
                    }
                    c.items.push(item);
                }
            }
            // Display-only directives (column width, etc.) — ignored.
            _ => {}
        }
    }
    finish(&mut out, &mut cur);
    out
}

/// Parse one item body: `CHALLENGE|RESPONSE[:condition]`. The text carries no
/// colon, so the first colon (if any) splits the spoken text from the condition
/// spec. Colour codes (`\green\…`) are stripped by [`humanize`].
fn parse_item(rest: &str, map: &[ChecklistMapEntry], fuel: &[FuelPosition]) -> Item {
    let (text, condspec) = match rest.split_once(':') {
        Some((t, c)) => (t, Some(c.trim())),
        None => (rest, None),
    };

    let (challenge, response) = text.split_once('|').unwrap_or((text, ""));
    let say = speak_item(challenge, response);

    let Some(condspec) = condspec.filter(|c| !c.is_empty()) else {
        // No condition → the pilot's call-out.
        return Item::CallOut(say);
    };
    // Compound (`&&`/`||`/parenthesised) and function (`closer_than(...)`)
    // conditions are beyond our single-dataref evaluator — we can't act on or
    // verify them honestly, so they become call-outs (the pilot's item).
    if is_compound_condition(condspec) {
        return Item::CallOut(say);
    }
    // A simple `dataref:cond` (dataref has no colon; cond uses `>`/`<`/`|`/etc.).
    let Some((dataref, cond_str)) = condspec.split_once(':') else {
        return Item::CallOut(say);
    };
    let Some(cond) = Cond::parse(cond_str) else {
        // An operator we don't model (`><`, `+>`, `-<`, …) — don't fake it.
        return Item::CallOut(say);
    };
    let (base, index) = split_index(dataref.trim());

    // The map sees the dataref WITH its index: a per-element entry
    // ("landing_lights_switch[1]", the A330's taxi light on a three-position
    // nose-light switch, 2026-09-20) can only match against the full name,
    // and a base-only entry matches the full name just the same.
    match action_for(map, dataref.trim(), &cond, fuel) {
        Some((tool, args)) => {
            let confirm = confirm_for(tool, &args);
            Item::Act {
                say,
                tool,
                args,
                confirm,
                dataref: base,
                index,
                cond,
            }
        }
        None => {
            // Only auto-set items that are both an action (not a CHECK/observe
            // response) and a plausibly-writable dataref. CHECK/test steps and
            // read-only indicators (annunciators, indicators, warnings, custom
            // `laminar/*` datarefs) are observe-only — reading them is honest,
            // writing them fails and floods the readback with "couldn't set it".
            let settable = !is_observe_response(response) && !is_readonly_dataref(&base);
            Item::Verify {
                say,
                dataref: base,
                index,
                cond,
                settable,
            }
        }
    }
}

/// A response verb that means "look at it / confirm it", not "set it" — so the
/// runner reads and confirms but never writes. Covers the XChecklist CHECK/test
/// vocabulary and the open-ended "as required" responses.
fn is_observe_response(resp: &str) -> bool {
    let r = resp.trim().to_ascii_uppercase();
    const OBSERVE: &[&str] = &[
        "CHECK", "VERIFY", "CONFIRM", "MONITOR", "NOTE", "OBSERVE", "TEST", "CHECKED", "ENSURE",
    ];
    OBSERVE.iter().any(|w| r.starts_with(w))
        || r.contains("AS REQ")
        || r.contains("AS DESIRED")
        || r.contains("AS NEEDED")
        || r.contains("IF NEED")    // "ON IF NEEDED" — pilot's discretion, don't force it
        || r.contains("IF REQ")
        || r.contains("COMPLETE")
}

/// A dataref that's a status/indicator readout (or a custom `laminar/*` dataref),
/// which is read-only from the Web API — writing it can't take, so the runner
/// must only read it, never try to set it.
fn is_readonly_dataref(d: &str) -> bool {
    d.starts_with("laminar/")
        || d.contains("/annunciators/")
        || d.contains("/indicators/")
        || d.contains("/warnings/")
        || d.contains("/failures/")
        || d.contains("/gauges/")
}

/// A condition we can't evaluate as a single `dataref:cond` — boolean compounds
/// (`(a:1)&&(b:1)`, `||`), function predicates (`closer_than(...)`), or anything
/// with braces/parens. Such items degrade to call-outs.
fn is_compound_condition(c: &str) -> bool {
    c.starts_with('(') || c.contains("&&") || c.contains("||") || c.contains('(') || c.contains('{')
}

/// Hazard text for pulling the mixture to idle cutoff. Only worth confirming
/// when the engine is actually turning — pre-start (e.g. the SR22's "mixture
/// lean" before-engine-start item) it's the normal lean, so the runner suppresses
/// this prompt when the engine isn't running (see [`engine_running`]).
pub const CONFIRM_MIXTURE_CUTOFF: &str = "cut the mixture and stop the engine";

/// Steps the runner should confirm before performing — the engine-state changes.
/// Cranking (`start_engine`), an explicit stop, and mixture-to-cutoff (which kills
/// a running engine) are the destructive ones. The run-up is armed separately
/// (it's a throttle-up, not a distinct tool).
fn confirm_for(tool: &str, args: &Value) -> Option<&'static str> {
    match tool {
        "start_engine" => Some("start the engine"),
        "stop_engine" => Some("shut the engine down"),
        "set_mixture" if args.get("percent").and_then(Value::as_i64) == Some(0) => {
            Some(CONFIRM_MIXTURE_CUTOFF)
        }
        _ => None,
    }
}

/// Is engine 1 running? Used to suppress the mixture-cutoff confirm pre-start.
/// On a read error, assume running (fail safe — keep the confirm).
pub async fn engine_running(xp: &XPlaneClient) -> bool {
    xp.get_float_array(crate::datarefs::ENGINE_RUNNING)
        .await
        .ok()
        .and_then(|v| v.first().copied())
        .is_none_or(|r| r > 0.5)
}

/// Map a file dataref + its pass condition to one of our curated tools, using
/// the airframe's [`checklist_map`](crate::aircraft::AircraftProfile). `None`
/// means we don't actuate it (→ verify or call-out): no map entry matched, or
/// the matched action can't honour this condition. *Which* dataref maps to
/// *which* system is config (per-airframe); *how* the condition becomes the
/// argument is the shared logic here.
fn action_for(
    map: &[ChecklistMapEntry],
    dataref: &str,
    cond: &Cond,
    fuel: &[FuelPosition],
) -> Option<(&'static str, Value)> {
    let entry = map.iter().find(|e| dataref.contains(&e.match_))?;
    let on = cond.is_on();
    match &entry.action {
        ChecklistAction::ParkingBrake => Some(("set_parking_brake", json!({ "on": on }))),
        ChecklistAction::PitotHeat => Some(("set_pitot_heat", json!({ "on": on }))),
        ChecklistAction::FuelPump => Some(("set_fuel_pump", json!({ "on": on }))),
        ChecklistAction::AutopilotOff => Some(("autopilot_off", json!({}))),
        ChecklistAction::PitchTrim => Some(("set_pitch_trim", json!({ "action": "takeoff" }))),
        ChecklistAction::Electrical(system) => {
            Some(("set_electrical", json!({ "system": system, "on": on })))
        }
        ChecklistAction::Light(key) => Some(("set_lights", json!({ "light": key, "on": on }))),
        ChecklistAction::Transponder => match cond {
            Cond::Eq(n) => {
                let mode = if *n < 0.5 {
                    "off"
                } else if *n < 2.5 {
                    "standby"
                } else {
                    "altitude"
                };
                Some(("set_transponder_mode", json!({ "mode": mode })))
            }
            _ => None,
        },
        ChecklistAction::FuelSelector => {
            // Decode the file's selector value against this airframe's positions,
            // so the same logic serves the 172 (4=both) and the SR22
            // (0=off/1=left/2=right). `:N` picks the position whose dataref value
            // is N; `:!N` ("not both" on the 172) picks the first position that
            // isn't N. Inequalities (`:>0` shutoff-pushed-in) stay verify-only.
            let key = match cond {
                Cond::Eq(n) => fuel
                    .iter()
                    .find(|p| p.value.is_some_and(|v| (v - n).abs() < 0.5)),
                Cond::Ne(n) => fuel
                    .iter()
                    .find(|p| p.value.is_some_and(|v| (v - n).abs() >= 0.5)),
                _ => None,
            }?;
            Some(("set_fuel_selector", json!({ "position": key.key })))
        }
        ChecklistAction::Flaps => match cond {
            // Files express "up"/"full" as either a bound (172: `<0.02`/`>0.90`)
            // or an exact value (SR22: `0.0`/`1.0`).
            Cond::Lt(n) | Cond::Eq(n) if *n <= 0.1 => Some(("set_flaps", json!({ "notch": "up" }))),
            Cond::Gt(n) | Cond::Eq(n) if *n >= 0.85 => {
                Some(("set_flaps", json!({ "notch": "full" })))
            }
            _ => None, // "as required" / intermediate (e.g. 50%) → the pilot's call
        },
        ChecklistAction::Mixture => {
            let t = cond.target();
            let pct = if t >= 0.9 {
                100
            } else if t <= 0.1 {
                0
            } else {
                65
            };
            Some(("set_mixture", json!({ "percent": pct })))
        }
        ChecklistAction::Throttle => {
            let pct = (cond.target() * 100.0).round() as i64;
            Some(("set_throttle", json!({ "percent": pct })))
        }
        ChecklistAction::Ignition => match cond {
            // 0 off · 1 right · 2 left · 3 both · 4 start.
            Cond::Eq(n) => {
                let v = *n;
                if v >= 3.5 {
                    Some(("start_engine", json!({})))
                } else {
                    let pos = if v < 0.5 {
                        "off"
                    } else if v < 1.5 {
                        "right"
                    } else if v < 2.5 {
                        "left"
                    } else {
                        "both"
                    };
                    Some(("set_magnetos", json!({ "position": pos })))
                }
            }
            _ => None,
        },
    }
}

/// Read the value behind a (possibly indexed) dataref, for the verify path.
pub async fn read_dataref(xp: &XPlaneClient, dataref: &str, index: Option<usize>) -> Option<f64> {
    match index {
        Some(i) => xp.get_float_array(dataref).await.ok()?.get(i).copied(),
        None => xp.get_float(dataref).await.ok(),
    }
}

/// Perform one checklist action by dispatching to the matching single-action
/// tool (which reads the dataref back, so the action is verified). Lights/flaps
/// tools are built from the profile.
#[allow(clippy::too_many_arguments)]
pub async fn run_step(
    tool: &str,
    args: &Value,
    lights: &[LightProfile],
    flaps: &Option<FlapProfile>,
    electrical: &[ElectricalSystem],
    fuel_selector: &Option<FuelSelectorProfile>,
    engines: usize,
    magnetos_per_engine: bool,
    park_brake_valve_set: Option<f64>,
    pitot_heat_dataref: Option<String>,
    ctx: &ToolCtx<'_>,
) -> Result<String> {
    match tool {
        "set_parking_brake" => {
            SetParkingBrake {
                valve_set: park_brake_valve_set,
            }
            .execute(args, ctx)
            .await
        }
        "set_pitch_trim" => SetPitchTrim.execute(args, ctx).await,
        "set_electrical" => {
            SetElectrical::new(electrical.to_vec())
                .execute(args, ctx)
                .await
        }
        "set_mixture" => SetMixture.execute(args, ctx).await,
        "set_throttle" => SetThrottle.execute(args, ctx).await,
        "set_fuel_selector" => match fuel_selector {
            Some(f) => SetFuelSelector::new(f.clone()).execute(args, ctx).await,
            None => bail!("no fuel selector on this airframe"),
        },
        "set_magnetos" => {
            SetMagnetos {
                engines,
                per_engine: magnetos_per_engine,
            }
            .execute(args, ctx)
            .await
        }
        "set_pitot_heat" => {
            SetPitotHeat {
                override_dataref: pitot_heat_dataref,
            }
            .execute(args, ctx)
            .await
        }
        "set_carb_heat" => SetCarbHeat { engines }.execute(args, ctx).await,
        // Universal-array pumps only: checklist-mapped airframes (172/SR22)
        // don't use the command override.
        "set_fuel_pump" => {
            SetFuelPump {
                engines,
                commands: None,
            }
            .execute(args, ctx)
            .await
        }
        "set_transponder_mode" => SetTransponderMode.execute(args, ctx).await,
        "autopilot_off" => AutopilotOff.execute(args, ctx).await,
        "start_engine" => {
            StartEngine {
                engines,
                cycle_selectors: Vec::new(),
                fuel_selector: fuel_selector.clone(),
                alternator_on_cmds: crate::tools::alternator_on_cmds(electrical),
                bus_power_on_cmds: crate::tools::bus_power_on_cmds(electrical),
            }
            .execute(args, ctx)
            .await
        }
        "stop_engine" => {
            StopEngine {
                engines,
                park_brake_valve_set,
            }
            .execute(args, ctx)
            .await
        }
        "set_lights" => SetLights::new(lights.to_vec()).execute(args, ctx).await,
        "set_flaps" => match flaps {
            Some(f) => SetFlaps::new(f.clone()).execute(args, ctx).await,
            None => bail!("no flaps on this airframe"),
        },
        other => bail!("checklist step references unknown tool {other}"),
    }
}

// ---- text helpers -------------------------------------------------------

/// Slug for the tool name: lowercase, runs of non-alphanumerics → single `_`.
fn slug(title: &str) -> String {
    let mut s = String::new();
    let mut last_us = false;
    for ch in title.chars() {
        if ch.is_ascii_alphanumeric() {
            s.push(ch.to_ascii_lowercase());
            last_us = false;
        } else if !last_us {
            s.push('_');
            last_us = true;
        }
    }
    s.trim_matches('_').to_string()
}

/// Turn a SHOUTED challenge+response into a speakable line: "PARKING BRAKE"|"ON"
/// → "Parking brake, on." Brackets (non-simulated markers) are stripped.
fn speak_item(challenge: &str, response: &str) -> String {
    let c = humanize(challenge);
    let r = humanize(response);
    match (c.is_empty(), r.is_empty()) {
        (false, false) => format!("{c}, {}.", lower_first(&r)),
        (false, true) => format!("{c}."),
        (true, false) => format!("{}.", r),
        (true, true) => String::new(),
    }
}

/// Decoration characters used for separators, rules, and pointers in the files.
/// (No letters — trimming those would eat real words like "go"/"oxygen".)
const DECORATION: &[char] = &['=', '-', '_', '*', '>', '<', '~', '.', ' '];

/// Narration line, or `None` for separators/blanks/pointers we shouldn't speak.
fn clean_narration(s: &str) -> Option<String> {
    let t = strip_colors(s);
    let t = t.trim();
    // Lines led with `>>>` are XChecklist pointer/sub-note decoration — "next
    // section" hints ("\\cyan\\ >>> NEXT: BEFORE TAXI") and conditional advisory
    // headers (">>>If ICING Conditions", ">>> Temp below 10 C / Fog / Rain"). They
    // frame the items that follow rather than being steps themselves, so reading
    // them aloud is noise — the actionable items below carry the real responses.
    if t.starts_with('>') {
        return None;
    }
    // Pure decoration ("=====", "-----", "*** ", "-o-") → nothing to say. The
    // ≤1 alphanumeric guard also catches lone leftovers like the "o" in "-o-".
    let core = t.trim_matches(DECORATION);
    if core.chars().filter(|c| c.is_alphanumeric()).count() <= 1 {
        return None;
    }
    // Drop section header/footer boilerplate ("CRUISE CHECKLIST", "... COMPLETED")
    // — the runner announces start/end itself. Substantive remarks ("MAGNETO
    // TEST", speeds) don't mention "checklist", so only duplicates are shed.
    if core.to_lowercase().contains("checklist") {
        return None;
    }
    let line = humanize(core).replace('|', ", ");
    let line = line.trim().trim_end_matches('.');
    if line.is_empty() {
        None
    } else {
        Some(format!("{line}."))
    }
}

/// Lowercase, strip colour codes and the `[...]` non-simulated brackets, and
/// capitalize the first letter — readable for TTS without SHOUTING acronyms.
fn humanize(s: &str) -> String {
    let stripped: String = strip_colors(s)
        .chars()
        .filter(|&c| c != '[' && c != ']')
        .collect();
    let lowered = stripped.trim().to_lowercase();
    cap_first(&lowered)
}

/// Remove XChecklist colour codes — `\green\`, `\white\`, … — which can appear
/// anywhere in an item's text (`\green\Door(s)\yellow\, open`). A colour code is
/// a backslash, an identifier, and a closing backslash.
fn strip_colors(s: &str) -> String {
    let mut out = String::new();
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c == '\\' {
            // Consume up to and including the closing backslash.
            for n in chars.by_ref() {
                if n == '\\' {
                    break;
                }
            }
        } else {
            out.push(c);
        }
    }
    out
}

fn cap_first(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

fn lower_first(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_lowercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// Split `dataref[0]` → (`dataref`, `Some(0)`); a plain name → (`dataref`, None).
fn split_index(dr: &str) -> (String, Option<usize>) {
    if let Some(open) = dr.find('[') {
        if let Some(close) = dr[open..].find(']') {
            let idx = dr[open + 1..open + close].parse().ok();
            return (dr[..open].to_string(), idx);
        }
    }
    (dr.to_string(), None)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 172 profile (carries the real checklist_map the tests exercise).
    fn p172() -> AircraftProfile {
        crate::aircraft::builtin("cessna_172sp").unwrap()
    }

    #[test]
    fn coloured_format_and_compound_conditions() {
        let m = p172().checklist_map;
        let parse_item = |r: &str| super::parse_item(r, &m, &[]);
        // sw_item_c with colour codes mid-text — colours stripped, item mapped.
        match parse_item(
            r"\green\Parking brakes\yellow\, engaged|\white\CHECK:sim/flightmodel/controls/parkbrake:1",
        ) {
            Item::Act { say, tool, .. } => {
                assert_eq!(tool, "set_parking_brake");
                assert_eq!(say, "Parking brakes, engaged, check.");
            }
            other => panic!("expected Act, got {}", variant(&other)),
        }
        // Compound (&&) condition → call-out (can't evaluate honestly).
        assert!(matches!(
            parse_item(
                r"\green\Batteries\yellow\, both on|\white\CHECK:(sim/.../battery_on[0]:1)&&(sim/.../battery_on[1]:1)"
            ),
            Item::CallOut(_)
        ));
        // Function predicate → call-out.
        assert!(matches!(
            parse_item(
                r"\green\Altimeter\yellow\, set QNH|\white\CHECK:1:closer_than({a},{b},0.02)"
            ),
            Item::CallOut(_)
        ));
        // An operator we don't model (`><`) → call-out, never a fake verify.
        assert!(matches!(
            parse_item(
                r"\green\HDG Bug\yellow\, set|\white\CHECK:sim/cockpit/autopilot/heading:><0.01"
            ),
            Item::CallOut(_)
        ));
    }

    #[test]
    fn strips_colours_and_skips_decoration() {
        assert_eq!(
            strip_colors(r"\green\Door(s)\yellow\, open"),
            "Door(s), open"
        );
        // Nav pointers and pure rules are silent…
        assert_eq!(clean_narration(r"\cyan\ >>> NEXT: BEFORE TAXI"), None);
        assert_eq!(clean_narration(r"\cyan\ ======================"), None);
        assert_eq!(clean_narration(r"\cyan\ -o-"), None);
        // …but a real heading survives, colour-stripped.
        assert_eq!(
            clean_narration(r"\white\ *** MAGNETOS TEST").as_deref(),
            Some("Magnetos test.")
        );
    }

    // Small helper so the panics above name the variant.
    fn variant(i: &Item) -> &'static str {
        match i {
            Item::Narrate(_) => "Narrate",
            Item::CallOut(_) => "CallOut",
            Item::Act { .. } => "Act",
            Item::Verify { .. } => "Verify",
        }
    }

    #[test]
    fn condition_grammar() {
        assert_eq!(Cond::parse("1"), Some(Cond::Eq(1.0)));
        assert_eq!(Cond::parse(">0.9"), Some(Cond::Gt(0.9)));
        assert_eq!(Cond::parse("<0.02"), Some(Cond::Lt(0.02)));
        assert_eq!(Cond::parse("!4"), Some(Cond::Ne(4.0)));
        assert_eq!(Cond::parse("0.15|0.20"), Some(Cond::Range(0.15, 0.20)));

        assert!(Cond::Eq(1.0).eval(1.0) && !Cond::Eq(1.0).eval(0.0));
        assert!(Cond::Gt(0.9).eval(0.95) && !Cond::Gt(0.9).eval(0.9));
        assert!(Cond::Lt(0.02).eval(0.0) && !Cond::Lt(0.02).eval(0.5));
        assert!(Cond::Ne(4.0).eval(2.0) && !Cond::Ne(4.0).eval(4.0));
        assert!(Cond::Range(0.15, 0.20).eval(0.17) && !Cond::Range(0.15, 0.20).eval(0.3));
    }

    #[test]
    fn slug_and_index() {
        assert_eq!(slug("BEFORE TAKEOFF"), "before_takeoff");
        assert_eq!(slug("STARTING ENGINE"), "starting_engine");
        assert_eq!(slug("*** CESSNA 172SP ***"), "cessna_172sp");
        assert_eq!(
            split_index("sim/.../mixture_ratio[0]"),
            ("sim/.../mixture_ratio".into(), Some(0))
        );
        assert_eq!(
            split_index("sim/.../parkbrake"),
            ("sim/.../parkbrake".into(), None)
        );
    }

    #[test]
    fn voice_request_resolves_by_title() {
        let cls = load(&p172()).expect("172 checklist loads");
        let hit = |t: &str| match_request(t, &cls).map(|c| c.id.as_str());

        // The live miss (2026-07-14, G1000 172): whisper's exact output.
        assert_eq!(hit("before takeoff checklist"), Some("before_takeoff"));
        // Polite wrappings, split compounds, and whisper's trailing period/"?"
        // arrive as separator noise - all resolve to the same section.
        assert_eq!(
            hit("run the before take off check list"),
            Some("before_takeoff")
        );
        assert_eq!(
            hit("let's do the pre-flight checklist, please"),
            Some("preflight")
        );
        assert_eq!(hit("checklist before taxi"), Some("before_taxi"));
        assert_eq!(hit("shut down checklist"), Some("engine_shutdown"));
        assert_eq!(hit("cruise checklist"), Some("cruise"));
        assert_eq!(hit("after landing checklist"), Some("after_landing"));
        // A bare phase prefers the "before" section - that's what pilots mean.
        assert_eq!(hit("takeoff checklist"), Some("before_takeoff"));
        assert_eq!(hit("landing checklist"), Some("before_landing"));
        // Prefix tolerance: "start" reaches "starting", and the exact-title
        // candidate beats the longer "before starting engine".
        assert_eq!(hit("engine start checklist"), Some("starting_engine"));
        assert_eq!(
            hit("before start checklist"),
            Some("before_starting_engine")
        );

        // Never a guess: ambiguous or unknown asks return None (the caller
        // declines and names the options), and without the trigger word
        // nothing matches at all - "climb" alone must stay a climb command.
        assert_eq!(hit("engine checklist"), None);
        assert_eq!(hit("run the checklist"), None);
        assert_eq!(hit("securing aircraft checklist"), None);
        assert_eq!(hit("climb"), None);
        assert_eq!(hit("before takeoff"), None);
        assert_eq!(hit("flaps up"), None);

        assert!(mentions_checklist("run the check list"));
        assert!(mentions_checklist("before takeoff checklist"));
        assert!(!mentions_checklist("check the flaps"));
    }

    #[test]
    fn sr22_checklist_acts_on_wired_tools_only() {
        let p = crate::aircraft::builtin("cirrus_sr22").unwrap();
        let cls = load(&p).expect("SR22 checklist loads");
        assert!(cls.len() > 15, "expected the full SR22 section set");

        // Everything it acts on is a tool the SR22 profile actually wires — the
        // universal-dataref ones plus the rustdecks-verified boost pump, key-start
        // ignition (set_magnetos / start_engine) and pitch trim. Never an unbuilt
        // system (e.g. the off/left/right fuel selector isn't checklist-mapped).
        let wired = [
            "set_parking_brake",
            "set_transponder_mode",
            "set_pitot_heat",
            "set_lights",
            "set_throttle",
            "set_mixture",
            "set_flaps",
            "set_electrical",
            "set_fuel_pump",
            "set_magnetos",
            "start_engine",
            "set_pitch_trim",
            "set_fuel_selector",
        ];
        let mut acts = 0;
        let mut seen = std::collections::BTreeSet::new();
        for c in &cls {
            for it in &c.items {
                if let Item::Act { tool, .. } = it {
                    assert!(
                        wired.contains(tool),
                        "SR22 acted on an unwired tool: {tool}"
                    );
                    seen.insert(*tool);
                    acts += 1;
                }
            }
        }
        assert!(acts > 0, "expected the wired acts");
        // The newly-wired systems actually show up in the interpreted file.
        for t in [
            "set_fuel_pump",
            "set_magnetos",
            "set_pitch_trim",
            "set_fuel_selector",
        ] {
            assert!(seen.contains(t), "expected the SR22 file to drive {t}");
        }
    }

    #[test]
    fn empty_map_never_acts() {
        // With no checklist_map, an item we'd normally act on becomes a verify
        // (we can read it) — never an Act. This is the SR22's state until its
        // map is filled in: read-along + call-out, no actuation.
        let it = super::parse_item(
            "PARKING BRAKE|ON:sim/flightmodel/controls/parkbrake:1",
            &[],
            &[],
        );
        assert!(matches!(it, Item::Verify { .. }));
    }

    #[test]
    fn maps_actions_and_speech() {
        let m = p172().checklist_map;
        let parse_item = |r: &str| super::parse_item(r, &m, &[]);
        // Parking brake ON → set_parking_brake(on).
        let it = parse_item("PARKING BRAKE|ON:sim/flightmodel/controls/parkbrake:1");
        match it {
            Item::Act {
                say,
                tool,
                args,
                confirm,
                ..
            } => {
                assert_eq!(tool, "set_parking_brake");
                assert_eq!(args, json!({ "on": true }));
                assert_eq!(say, "Parking brake, on.");
                assert!(confirm.is_none()); // not a destructive step
            }
            _ => panic!("expected Act"),
        }
        // Mixture FULL RICH (>0.95) → set_mixture(100).
        match parse_item("MIXTURE|FULL RICH:sim/cockpit2/engine/actuators/mixture_ratio[0]:>0.95") {
            Item::Act { tool, args, .. } => {
                assert_eq!(tool, "set_mixture");
                assert_eq!(args, json!({ "percent": 100 }));
            }
            _ => panic!("expected Act"),
        }
        // Ignition START (==4) → start_engine.
        match parse_item("IGNITION SWITCH|START:sim/cockpit2/engine/actuators/ignition_key[0]:4") {
            Item::Act { tool, .. } => assert_eq!(tool, "start_engine"),
            _ => panic!("expected Act"),
        }
        // Throttle range → midpoint percent.
        match parse_item(
            "THROTTLE|OPEN 1/4 INCH:sim/cockpit2/engine/actuators/throttle_ratio[0]:0.15|0.20",
        ) {
            Item::Act { tool, args, .. } => {
                assert_eq!(tool, "set_throttle");
                assert_eq!(args, json!({ "percent": 18 }));
            }
            _ => panic!("expected Act"),
        }
        // No dataref → call-out.
        assert!(matches!(
            parse_item("FUEL QUANTITY|SUFFICIENT"),
            Item::CallOut(_)
        ));
        // A dataref we don't own → verify, and a writable switch is settable.
        match parse_item(
            "STATIC PRESSURE ALTERNATE|OFF:sim/cockpit2/switches/alternate_static_air_ratio:<0.1",
        ) {
            Item::Verify { cond, settable, .. } => {
                assert_eq!(cond, Cond::Lt(0.1));
                assert!(
                    settable,
                    "a writable switch with an action response is settable"
                );
            }
            _ => panic!("expected Verify"),
        }
    }

    #[test]
    fn observe_and_readonly_items_are_not_settable() {
        let parse_item = |r: &str| super::parse_item(r, &[], &[]);
        // A CHECK response → observe-only, never written.
        match parse_item(
            "CLEAR MASTER CAUTION|CHECK:sim/cockpit/warnings/annunciators/master_caution:0",
        ) {
            Item::Verify { settable, .. } => assert!(!settable, "CHECK is observe-only"),
            other => panic!("expected Verify, got {}", variant(&other)),
        }
        // A read-only indicator dataref → observe-only even with an action response.
        match parse_item("APU POWER|ON:sim/cockpit2/bleedair/indicators/bleed_available_center:1") {
            Item::Verify { settable, .. } => {
                assert!(!settable, "an indicator dataref isn't writable")
            }
            other => panic!("expected Verify, got {}", variant(&other)),
        }
        // "ON IF NEEDED" is the pilot's discretion → observe-only.
        match parse_item(
            "ENGINE ANTI ICE|ON IF NEEDED:sim/cockpit/switches/anti_ice_inlet_heat_per_engine[0]:1",
        ) {
            Item::Verify { settable, .. } => {
                assert!(!settable, "conditional 'if needed' isn't forced")
            }
            other => panic!("expected Verify, got {}", variant(&other)),
        }
    }

    #[test]
    fn pointer_remarks_are_dropped() {
        // `>>>` advisory headers ("If ICING Conditions", weather descriptors) are
        // sub-note decoration, not spoken steps.
        assert_eq!(super::clean_narration(">>>If ICING Conditions"), None);
        assert_eq!(
            super::clean_narration(">>> Temp below 10 C / Fog / Rain / Snow"),
            None
        );
        // A substantive remark still survives.
        assert!(super::clean_narration("MAGNETO TEST").is_some());
    }

    #[test]
    fn fuel_selector_decodes_per_airframe() {
        use crate::aircraft::FuelPosition;
        let pos = |key: &str, value: f64| FuelPosition {
            key: key.into(),
            spoken: key.into(),
            command: String::new(),
            value: Some(value),
        };
        let map = [ChecklistMapEntry {
            match_: "fuel_tank_selector".into(),
            action: ChecklistAction::FuelSelector,
        }];
        let act = |r: &str, fuel: &[FuelPosition]| super::parse_item(r, &map, fuel);

        // 172 positions: left=1, both=4, right=3.
        let c172 = [pos("left", 1.0), pos("both", 4.0), pos("right", 3.0)];
        // `:4` → both; `:!4` ("left or right") → the first non-4 position (left).
        match act("FUEL|BOTH:sim/cockpit2/fuel/fuel_tank_selector:4", &c172) {
            Item::Act { args, .. } => assert_eq!(args, json!({ "position": "both" })),
            other => panic!("expected Act both, got {}", variant(&other)),
        }
        match act("FUEL|L or R:sim/cockpit2/fuel/fuel_tank_selector:!4", &c172) {
            Item::Act { args, .. } => assert_eq!(args, json!({ "position": "left" })),
            other => panic!("expected Act left, got {}", variant(&other)),
        }

        // SR22 positions: off=0, left=1, right=2 — the case that used to fall
        // through to a read-only verify.
        let sr22 = [pos("off", 0.0), pos("left", 1.0), pos("right", 2.0)];
        for (val, want) in [("0", "off"), ("1", "left"), ("2", "right")] {
            match act(
                &format!("FUEL|X:sim/cockpit2/fuel/fuel_tank_selector:{val}"),
                &sr22,
            ) {
                Item::Act { tool, args, .. } => {
                    assert_eq!(tool, "set_fuel_selector");
                    assert_eq!(args, json!({ "position": want }));
                }
                other => panic!("expected Act {want}, got {}", variant(&other)),
            }
        }
        // "switch to fullest" (`:>0.1`) names no single position, so it stays a
        // read-only verify rather than guessing a tank.
        assert!(matches!(
            act(
                "FUEL|FULLEST:sim/cockpit2/fuel/fuel_tank_selector:>0.1",
                &sr22
            ),
            Item::Verify { .. }
        ));
    }

    #[test]
    fn parses_the_172_file() {
        let text = std::fs::read_to_string("checklists/cessna_172sp.txt")
            .or_else(|_| std::fs::read_to_string("profiles/Cessna_172SP_clist.txt"))
            .expect("172 checklist file present");
        let cls = parse(&text, &p172());
        // All the named sections come through with distinct slugs.
        let ids: Vec<&str> = cls.iter().map(|c| c.id.as_str()).collect();
        assert!(ids.contains(&"before_takeoff"));
        assert!(ids.contains(&"starting_engine"));
        assert!(ids.contains(&"before_starting_engine"));
        assert!(ids.contains(&"engine_shutdown"));
        // The before-takeoff list actuates the parking brake (first real item).
        let bt = cls.iter().find(|c| c.id == "before_takeoff").unwrap();
        assert!(bt
            .items
            .iter()
            .any(|i| matches!(i, Item::Act { tool, .. } if *tool == "set_parking_brake")));

        // The file's "X CHECKLIST" / "...COMPLETED" boilerplate is dropped — the
        // runner announces start/end itself, so none should survive as narration.
        for c in &cls {
            for item in &c.items {
                if let Item::Narrate(t) = item {
                    assert!(
                        !t.to_lowercase().contains("checklist"),
                        "leaked boilerplate: {t:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn destructive_steps_are_gated() {
        let text = std::fs::read_to_string("checklists/cessna_172sp.txt").unwrap();
        let cls = parse(&text, &p172());

        // Engine start carries a confirm.
        let start = cls.iter().find(|c| c.id == "starting_engine").unwrap();
        assert!(start.items.iter().any(|i| matches!(i, Item::Act { tool, confirm, .. } if *tool == "start_engine" && confirm.is_some())));

        // Engine shutdown kills the engine via mixture cutoff — that step is gated.
        let shut = cls.iter().find(|c| c.id == "engine_shutdown").unwrap();
        assert!(shut
            .items
            .iter()
            .any(|i| matches!(i, Item::Act { tool, args, confirm, .. }
            if *tool == "set_mixture" && args["percent"] == 0 && confirm.is_some())));

        // The run-up is armed by the MAGNETO TEST remark: exactly one step in
        // before-takeoff confirms (the throttle-up), and it's a throttle step.
        let bt = cls.iter().find(|c| c.id == "before_takeoff").unwrap();
        let gated: Vec<&str> = bt
            .items
            .iter()
            .filter_map(|i| match i {
                Item::Act {
                    tool,
                    confirm: Some(_),
                    ..
                } => Some(*tool),
                _ => None,
            })
            .collect();
        assert_eq!(gated, vec!["set_throttle"]);
    }
}
