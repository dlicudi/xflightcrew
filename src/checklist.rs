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
    AircraftProfile, ChecklistAction, ChecklistMapEntry, ElectricalSystem, FlapProfile, LightProfile,
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
use crate::tools::{Tool, ToolCtx, ToolSpec};
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
    fn target(&self) -> f64 {
        match *self {
            Cond::Eq(n) | Cond::Ne(n) => n,
            Cond::Range(a, b) => (a + b) / 2.0,
            // "more than": full if near the top, else just over the line.
            Cond::Gt(n) => if n >= 0.85 { 1.0 } else { (n + 0.05).min(1.0) },
            // "less than": zero if near the bottom, else just under the line.
            Cond::Lt(n) => if n <= 0.05 { 0.0 } else { (n - 0.05).max(0.0) },
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
    Act { say: String, tool: &'static str, args: Value, confirm: Option<&'static str> },
    /// A dataref we don't actuate but can read: confirm or flag it.
    Verify { say: String, dataref: String, index: Option<usize>, cond: Cond },
}

/// One interpreted checklist. Carries the profile bits its actions need (lights,
/// flaps), so the runner can dispatch without re-plumbing the profile.
pub struct Checklist {
    pub id: String,
    pub title: String,
    pub description: String,
    pub items: Vec<Item>,
    pub lights: Vec<LightProfile>,
    pub flaps: Option<FlapProfile>,
    pub electrical: Vec<ElectricalSystem>,
}

impl Checklist {
    /// The no-arg tool spec the LLM routes against (the name *is* the pick).
    pub fn spec(&self) -> ToolSpec {
        ToolSpec {
            name: self.id.clone(),
            description: self.description.clone(),
            parameters: json!({ "type": "object", "properties": {}, "additionalProperties": false }),
        }
    }
}

/// Read and interpret the airframe's XChecklist file into runnable checklists.
/// `Ok(vec![])` if the profile names no checklist file.
pub fn load(profile: &AircraftProfile) -> Result<Vec<Checklist>> {
    let Some(path) = &profile.checklist else { return Ok(Vec::new()) };
    let text = std::fs::read_to_string(path).with_context(|| format!("reading checklist file {path}"))?;
    Ok(parse(&text, profile))
}

/// Parse XChecklist text against an airframe profile (its `checklist_map` drives
/// which datarefs become actions). Tolerant: unknown directives and malformed
/// lines are skipped rather than failing the whole file.
pub fn parse(text: &str, profile: &AircraftProfile) -> Vec<Checklist> {
    let lights = profile.lights.clone();
    let flaps = profile.flaps.clone();
    let electrical = &profile.electrical;
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
        let Some((dir, rest)) = line.split_once(':') else { continue };
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
                    description: describe(title),
                    items: Vec::new(),
                    lights: lights.clone(),
                    flaps: flaps.clone(),
                    electrical: electrical.clone(),
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
                    let mut item = parse_item(rest, map);
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
fn parse_item(rest: &str, map: &[ChecklistMapEntry]) -> Item {
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

    match action_for(map, &base, &cond) {
        Some((tool, args)) => {
            let confirm = confirm_for(tool, &args);
            Item::Act { say, tool, args, confirm }
        }
        None => Item::Verify { say, dataref: base, index, cond },
    }
}

/// A condition we can't evaluate as a single `dataref:cond` — boolean compounds
/// (`(a:1)&&(b:1)`, `||`), function predicates (`closer_than(...)`), or anything
/// with braces/parens. Such items degrade to call-outs.
fn is_compound_condition(c: &str) -> bool {
    c.starts_with('(')
        || c.contains("&&")
        || c.contains("||")
        || c.contains('(')
        || c.contains('{')
}

/// Steps the runner should confirm before performing — the engine-state changes.
/// Cranking (`start_engine`), an explicit stop, and mixture-to-cutoff (which kills
/// a running engine) are the destructive ones. The run-up is armed separately
/// (it's a throttle-up, not a distinct tool).
fn confirm_for(tool: &str, args: &Value) -> Option<&'static str> {
    match tool {
        "start_engine" => Some("start the engine"),
        "stop_engine" => Some("shut the engine down"),
        "set_mixture" if args.get("percent").and_then(Value::as_i64) == Some(0) => {
            Some("cut the mixture and stop the engine")
        }
        _ => None,
    }
}

/// Map a file dataref + its pass condition to one of our curated tools, using
/// the airframe's [`checklist_map`](crate::aircraft::AircraftProfile). `None`
/// means we don't actuate it (→ verify or call-out): no map entry matched, or
/// the matched action can't honour this condition. *Which* dataref maps to
/// *which* system is config (per-airframe); *how* the condition becomes the
/// argument is the shared logic here.
fn action_for(map: &[ChecklistMapEntry], dataref: &str, cond: &Cond) -> Option<(&'static str, Value)> {
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
                let mode = if *n < 0.5 { "off" } else if *n < 2.5 { "standby" } else { "altitude" };
                Some(("set_transponder_mode", json!({ "mode": mode })))
            }
            _ => None,
        },
        ChecklistAction::FuelSelector => match cond {
            Cond::Eq(n) if (*n - 4.0).abs() < 0.5 => Some(("set_fuel_selector", json!({ "position": "both" }))),
            Cond::Ne(n) if (*n - 4.0).abs() < 0.5 => Some(("set_fuel_selector", json!({ "position": "left" }))),
            _ => None, // ">0" (shutoff pushed in) etc. → verify only
        },
        ChecklistAction::Flaps => match cond {
            // Files express "up"/"full" as either a bound (172: `<0.02`/`>0.90`)
            // or an exact value (SR22: `0.0`/`1.0`).
            Cond::Lt(n) | Cond::Eq(n) if *n <= 0.1 => Some(("set_flaps", json!({ "notch": "up" }))),
            Cond::Gt(n) | Cond::Eq(n) if *n >= 0.85 => Some(("set_flaps", json!({ "notch": "full" }))),
            _ => None, // "as required" / intermediate (e.g. 50%) → the pilot's call
        },
        ChecklistAction::Mixture => {
            let t = cond.target();
            let pct = if t >= 0.9 { 100 } else if t <= 0.1 { 0 } else { 65 };
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
                    let pos = if v < 0.5 { "off" } else if v < 1.5 { "right" } else if v < 2.5 { "left" } else { "both" };
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
pub async fn run_step(
    tool: &str,
    args: &Value,
    lights: &[LightProfile],
    flaps: &Option<FlapProfile>,
    electrical: &[ElectricalSystem],
    ctx: &ToolCtx<'_>,
) -> Result<String> {
    match tool {
        "set_parking_brake" => SetParkingBrake.execute(args, ctx).await,
        "set_pitch_trim" => SetPitchTrim.execute(args, ctx).await,
        "set_electrical" => SetElectrical::new(electrical.to_vec()).execute(args, ctx).await,
        "set_mixture" => SetMixture.execute(args, ctx).await,
        "set_throttle" => SetThrottle.execute(args, ctx).await,
        "set_fuel_selector" => SetFuelSelector.execute(args, ctx).await,
        "set_magnetos" => SetMagnetos.execute(args, ctx).await,
        "set_pitot_heat" => SetPitotHeat.execute(args, ctx).await,
        "set_carb_heat" => SetCarbHeat.execute(args, ctx).await,
        "set_fuel_pump" => SetFuelPump.execute(args, ctx).await,
        "set_transponder_mode" => SetTransponderMode.execute(args, ctx).await,
        "autopilot_off" => AutopilotOff.execute(args, ctx).await,
        "start_engine" => StartEngine.execute(args, ctx).await,
        "stop_engine" => StopEngine.execute(args, ctx).await,
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

/// The LLM-facing description: trigger phrasing plus the anti-greedy guard so a
/// checklist doesn't steal a single-system command.
fn describe(title: &str) -> String {
    let t = humanize(title).to_lowercase();
    format!(
        "Run the {t} checklist — the copilot reads each item aloud, performs the \
         ones it can, verifies what it can read, and calls out the pilot's items. \
         ONLY for a single command asking for the whole '{t}' checklist (e.g. \
         '{t} checklist', '{t}', 'run the {t} checklist'). Do NOT use when one \
         system is named ('mixture rich', 'flaps up') — those have their own tools."
    )
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
    // Navigation pointers ("\\cyan\\ >>> NEXT: BEFORE TAXI") are XChecklist's own
    // "next section" hints — the runner advances on command, so they're noise.
    if t.trim_start_matches(DECORATION).starts_with("NEXT")
        || t.trim_start_matches(DECORATION).to_lowercase().starts_with("next:")
    {
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
    let stripped: String = strip_colors(s).chars().filter(|&c| c != '[' && c != ']').collect();
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
        let parse_item = |r: &str| super::parse_item(r, &m);
        // sw_item_c with colour codes mid-text — colours stripped, item mapped.
        match parse_item(r"\green\Parking brakes\yellow\, engaged|\white\CHECK:sim/flightmodel/controls/parkbrake:1") {
            Item::Act { say, tool, .. } => {
                assert_eq!(tool, "set_parking_brake");
                assert_eq!(say, "Parking brakes, engaged, check.");
            }
            other => panic!("expected Act, got {}", variant(&other)),
        }
        // Compound (&&) condition → call-out (can't evaluate honestly).
        assert!(matches!(
            parse_item(r"\green\Batteries\yellow\, both on|\white\CHECK:(sim/.../battery_on[0]:1)&&(sim/.../battery_on[1]:1)"),
            Item::CallOut(_)
        ));
        // Function predicate → call-out.
        assert!(matches!(
            parse_item(r"\green\Altimeter\yellow\, set QNH|\white\CHECK:1:closer_than({a},{b},0.02)"),
            Item::CallOut(_)
        ));
        // An operator we don't model (`><`) → call-out, never a fake verify.
        assert!(matches!(
            parse_item(r"\green\HDG Bug\yellow\, set|\white\CHECK:sim/cockpit/autopilot/heading:><0.01"),
            Item::CallOut(_)
        ));
    }

    #[test]
    fn strips_colours_and_skips_decoration() {
        assert_eq!(strip_colors(r"\green\Door(s)\yellow\, open"), "Door(s), open");
        // Nav pointers and pure rules are silent…
        assert_eq!(clean_narration(r"\cyan\ >>> NEXT: BEFORE TAXI"), None);
        assert_eq!(clean_narration(r"\cyan\ ======================"), None);
        assert_eq!(clean_narration(r"\cyan\ -o-"), None);
        // …but a real heading survives, colour-stripped.
        assert_eq!(clean_narration(r"\white\ *** MAGNETOS TEST").as_deref(), Some("Magnetos test."));
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
        assert_eq!(split_index("sim/.../mixture_ratio[0]"), ("sim/.../mixture_ratio".into(), Some(0)));
        assert_eq!(split_index("sim/.../parkbrake"), ("sim/.../parkbrake".into(), None));
    }

    #[test]
    fn sr22_checklist_runs_conservatively() {
        let p = crate::aircraft::builtin("cirrus_sr22").unwrap();
        let cls = load(&p).expect("SR22 checklist loads");
        assert!(cls.len() > 15, "expected the full SR22 section set");

        // Everything it acts on is a universal-dataref tool — never an SR22
        // system we haven't built (electrical, boost pump, fuel selector).
        let safe = [
            "set_parking_brake", "set_transponder_mode", "set_pitot_heat", "set_lights",
            "set_throttle", "set_mixture", "set_flaps", "set_electrical",
        ];
        let mut acts = 0;
        for c in &cls {
            for it in &c.items {
                if let Item::Act { tool, .. } = it {
                    assert!(safe.contains(tool), "SR22 acted on a non-conservative tool: {tool}");
                    acts += 1;
                }
            }
        }
        assert!(acts > 0, "expected at least the universal-safe acts");
    }

    #[test]
    fn empty_map_never_acts() {
        // With no checklist_map, an item we'd normally act on becomes a verify
        // (we can read it) — never an Act. This is the SR22's state until its
        // map is filled in: read-along + call-out, no actuation.
        let it = super::parse_item("PARKING BRAKE|ON:sim/flightmodel/controls/parkbrake:1", &[]);
        assert!(matches!(it, Item::Verify { .. }));
    }

    #[test]
    fn maps_actions_and_speech() {
        let m = p172().checklist_map;
        let parse_item = |r: &str| super::parse_item(r, &m);
        // Parking brake ON → set_parking_brake(on).
        let it = parse_item("PARKING BRAKE|ON:sim/flightmodel/controls/parkbrake:1");
        match it {
            Item::Act { say, tool, args, confirm } => {
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
        match parse_item("THROTTLE|OPEN 1/4 INCH:sim/cockpit2/engine/actuators/throttle_ratio[0]:0.15|0.20") {
            Item::Act { tool, args, .. } => {
                assert_eq!(tool, "set_throttle");
                assert_eq!(args, json!({ "percent": 18 }));
            }
            _ => panic!("expected Act"),
        }
        // No dataref → call-out.
        assert!(matches!(parse_item("FUEL QUANTITY|SUFFICIENT"), Item::CallOut(_)));
        // A dataref we don't own → verify.
        match parse_item("STATIC PRESSURE ALTERNATE|OFF:sim/cockpit2/switches/alternate_static_air_ratio:<0.1") {
            Item::Verify { cond, .. } => assert_eq!(cond, Cond::Lt(0.1)),
            _ => panic!("expected Verify"),
        }
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
        assert!(bt.items.iter().any(|i| matches!(i, Item::Act { tool, .. } if *tool == "set_parking_brake")));

        // The file's "X CHECKLIST" / "...COMPLETED" boilerplate is dropped — the
        // runner announces start/end itself, so none should survive as narration.
        for c in &cls {
            for item in &c.items {
                if let Item::Narrate(t) = item {
                    assert!(!t.to_lowercase().contains("checklist"), "leaked boilerplate: {t:?}");
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
        assert!(shut.items.iter().any(|i| matches!(i, Item::Act { tool, args, confirm, .. }
            if *tool == "set_mixture" && args["percent"] == 0 && confirm.is_some())));

        // The run-up is armed by the MAGNETO TEST remark: exactly one step in
        // before-takeoff confirms (the throttle-up), and it's a throttle step.
        let bt = cls.iter().find(|c| c.id == "before_takeoff").unwrap();
        let gated: Vec<&str> = bt.items.iter().filter_map(|i| match i {
            Item::Act { tool, confirm: Some(_), .. } => Some(*tool),
            _ => None,
        }).collect();
        assert_eq!(gated, vec!["set_throttle"]);
    }
}
