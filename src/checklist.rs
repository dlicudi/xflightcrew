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

use crate::aircraft::{FlapProfile, LightProfile};
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
    Act { say: String, tool: &'static str, args: Value },
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

/// Read and interpret an XChecklist file into runnable checklists.
pub fn load(path: &str, lights: Vec<LightProfile>, flaps: Option<FlapProfile>) -> Result<Vec<Checklist>> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading checklist file {path}"))?;
    Ok(parse(&text, lights, flaps))
}

/// Parse XChecklist text. Tolerant: unknown directives and malformed lines are
/// skipped rather than failing the whole file.
pub fn parse(text: &str, lights: Vec<LightProfile>, flaps: Option<FlapProfile>) -> Vec<Checklist> {
    let mut out: Vec<Checklist> = Vec::new();
    let mut cur: Option<Checklist> = None;

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
        match dir {
            "sw_checklist" => {
                finish(&mut out, &mut cur);
                // `MENU:TITLE` — the title is what we name and speak.
                let title = rest.split_once(':').map(|(_, t)| t).unwrap_or(rest).trim();
                cur = Some(Checklist {
                    id: slug(title),
                    title: humanize(title),
                    description: describe(title),
                    items: Vec::new(),
                    lights: lights.clone(),
                    flaps: flaps.clone(),
                });
            }
            "sw_remark" => {
                if let Some(c) = cur.as_mut() {
                    if let Some(text) = clean_narration(rest) {
                        c.items.push(Item::Narrate(text));
                    }
                }
            }
            "sw_itemvoid" => {
                if let Some(c) = cur.as_mut() {
                    // Reference/header lines. Speak the ones that carry content,
                    // skip separators and blanks.
                    if let Some(text) = clean_narration(rest) {
                        c.items.push(Item::Narrate(text));
                    }
                }
            }
            "sw_item" => {
                if let Some(c) = cur.as_mut() {
                    c.items.push(parse_item(rest));
                }
            }
            // Display-only directives (column width, etc.) — ignored.
            _ => {}
        }
    }
    finish(&mut out, &mut cur);
    out
}

/// Parse one `sw_item` body: `CHALLENGE|RESPONSE[:dataref[:cond]]`.
fn parse_item(rest: &str) -> Item {
    // Datarefs use '/', conditions use '|' — neither uses ':' — so colon splits
    // the three fields cleanly.
    let mut segs = rest.splitn(3, ':');
    let text = segs.next().unwrap_or("");
    let dataref = segs.next();
    let cond = segs.next();

    let (challenge, response) = text.split_once('|').unwrap_or((text, ""));
    let say = speak_item(challenge, response);

    let Some(dataref) = dataref.map(str::trim).filter(|d| !d.is_empty()) else {
        // No dataref → the pilot's call-out.
        return Item::CallOut(say);
    };
    let Some(cond) = cond.and_then(Cond::parse) else {
        return Item::CallOut(say);
    };
    let (base, index) = split_index(dataref);

    match action_for(&base, &cond) {
        Some((tool, args)) => Item::Act { say, tool, args },
        None => Item::Verify { say, dataref: base, index, cond },
    }
}

/// Map a file dataref + its pass condition to one of our curated tools. `None`
/// means we don't actuate it (→ verify or call-out). Light datarefs are matched
/// by name so both the file's namespaces (`cockpit/electrical/*_lights_on` and
/// `cockpit2/switches/*`) resolve to the same tool.
fn action_for(dataref: &str, cond: &Cond) -> Option<(&'static str, Value)> {
    let on = cond.is_on();
    let light = |key: &str| Some(("set_lights", json!({ "light": key, "on": on })));

    match dataref {
        "sim/flightmodel/controls/parkbrake" => Some(("set_parking_brake", json!({ "on": on }))),
        "sim/cockpit2/autopilot/autopilot_on" => Some(("autopilot_off", json!({}))),
        "sim/cockpit/electrical/battery_on" => Some(("set_electrical", json!({ "system": "battery", "on": on }))),
        "sim/cockpit/electrical/avionics_on" => Some(("set_electrical", json!({ "system": "avionics", "on": on }))),
        "sim/cockpit/electrical/generator_on" => Some(("set_electrical", json!({ "system": "alternator", "on": on }))),
        "sim/cockpit/switches/pitot_heat_on" => Some(("set_pitot_heat", json!({ "on": on }))),
        "sim/cockpit/engine/fuel_pump_on" => Some(("set_fuel_pump", json!({ "on": on }))),
        "sim/cockpit/radios/transponder_mode" => match cond {
            Cond::Eq(n) => {
                let mode = if *n < 0.5 { "off" } else if *n < 2.5 { "standby" } else { "altitude" };
                Some(("set_transponder_mode", json!({ "mode": mode })))
            }
            _ => None,
        },
        d if d.ends_with("fuel_tank_selector") => match cond {
            Cond::Eq(n) if (*n - 4.0).abs() < 0.5 => Some(("set_fuel_selector", json!({ "position": "both" }))),
            Cond::Ne(n) if (*n - 4.0).abs() < 0.5 => Some(("set_fuel_selector", json!({ "position": "left" }))),
            _ => None, // ">0" (shutoff pushed in) etc. → verify only
        },
        d if d.ends_with("flap_ratio") => match cond {
            Cond::Lt(n) if *n <= 0.1 => Some(("set_flaps", json!({ "notch": "up" }))),
            Cond::Gt(n) if *n >= 0.85 => Some(("set_flaps", json!({ "notch": "full" }))),
            _ => None, // "as required" → the pilot's call
        },
        d if d.ends_with("mixture_ratio") => {
            let t = cond.target();
            let pct = if t >= 0.9 { 100 } else if t <= 0.1 { 0 } else { 65 };
            Some(("set_mixture", json!({ "percent": pct })))
        }
        d if d.ends_with("throttle_ratio") => {
            let pct = (cond.target() * 100.0).round() as i64;
            Some(("set_throttle", json!({ "percent": pct })))
        }
        d if d.ends_with("ignition_key") => match cond {
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
        d if d.ends_with("elevator_trim") => Some(("set_pitch_trim", json!({ "action": "takeoff" }))),
        d if d.contains("taxi_light") => light("taxi"),
        d if d.contains("landing_lights") => light("landing"),
        d if d.contains("strobe_lights") => light("strobe"),
        d if d.contains("beacon_lights") => light("beacon"),
        d if d.contains("navigation_lights") || d.contains("nav_lights") => light("nav"),
        _ => None,
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
    ctx: &ToolCtx<'_>,
) -> Result<String> {
    match tool {
        "set_parking_brake" => SetParkingBrake.execute(args, ctx).await,
        "set_pitch_trim" => SetPitchTrim.execute(args, ctx).await,
        "set_electrical" => SetElectrical.execute(args, ctx).await,
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

/// Narration line, or `None` for separators/blanks we shouldn't speak.
fn clean_narration(s: &str) -> Option<String> {
    let t = s.trim().trim_matches('_').trim();
    if t.is_empty() {
        return None;
    }
    let line = humanize(t);
    // Replace the challenge|response bar with a comma for natural reading.
    let line = line.replace('|', ", ");
    if line.trim().is_empty() {
        None
    } else {
        Some(format!("{}.", line.trim_end_matches('.')))
    }
}

/// Lowercase, strip the `[...]` non-simulated brackets, and capitalize the first
/// letter — readable for TTS without SHOUTING acronyms.
fn humanize(s: &str) -> String {
    let stripped: String = s.chars().filter(|&c| c != '[' && c != ']').collect();
    let lowered = stripped.trim().to_lowercase();
    cap_first(&lowered)
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
    fn maps_actions_and_speech() {
        // Parking brake ON → set_parking_brake(on).
        let it = parse_item("PARKING BRAKE|ON:sim/flightmodel/controls/parkbrake:1");
        match it {
            Item::Act { say, tool, args } => {
                assert_eq!(tool, "set_parking_brake");
                assert_eq!(args, json!({ "on": true }));
                assert_eq!(say, "Parking brake, on.");
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
        let cls = parse(&text, vec![], None);
        // All the named sections come through with distinct slugs.
        let ids: Vec<&str> = cls.iter().map(|c| c.id.as_str()).collect();
        assert!(ids.contains(&"before_takeoff"));
        assert!(ids.contains(&"starting_engine"));
        assert!(ids.contains(&"before_starting_engine"));
        assert!(ids.contains(&"engine_shutdown"));
        // The before-takeoff list actuates the parking brake (first real item).
        let bt = cls.iter().find(|c| c.id == "before_takeoff").unwrap();
        assert!(bt.items.iter().any(|i| matches!(i, Item::Act { tool, .. } if *tool == "set_parking_brake")));
    }
}
