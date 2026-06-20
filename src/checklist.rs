//! Checklists, parsed from the [XChecklist] `.txt` format.
//!
//! XChecklist is a widely-used X-Plane plugin, so a large library of community
//! checklists already exists in this format. We parse it directly (the pilot
//! supplies a file; see [`crate::config`]) rather than inventing our own, and
//! ship a curated 172 file cross-checked against the aircraft profile.
//!
//! Only the lines we can act on are kept as structured [`Item`]s. Each actionable
//! line carries a **challenge** ("FUEL SELECTOR VALVE"), a **response**
//! ("BOTH"), and — when the format provides one — a verifiable [`Check`]: a
//! dataref plus a [`Pred`]icate. Lines without a dataref become spoken-only
//! confirms; `sw_itemvoid`/`sw_remark` lines become non-verified [`Item::info`]
//! context.
//!
//! Format reference (the subset that appears in real files):
//! ```text
//! sw_checklist:<menu name>:<title>
//! sw_item:CHALLENGE|RESPONSE                       # spoken confirm, no verify
//! sw_item:CHALLENGE|RESPONSE:dataref:COND          # verifiable
//! sw_itemvoid:free text                            # info / non-simulated
//! sw_remark:section note
//! ```
//! `dataref` may carry an array index (`...ignition_key[0]`). `COND` is one of
//! `N` (equals), `>N`, `<N`, `!N` (not-equals), or `LO|HI` (inclusive range).

use crate::tools::ToolSpec;
use serde_json::json;

/// Load and parse a checklist file, returning the checklists (empty on any
/// error — checklists are optional, a missing/broken file just disables the
/// feature rather than stopping the copilot).
pub fn load(path: &str) -> Vec<Checklist> {
    match std::fs::read_to_string(path) {
        Ok(text) => parse(&text),
        Err(e) => {
            tracing::info!(path, error = %e, "no checklist file; run_checklist disabled");
            Vec::new()
        }
    }
}

/// Find a checklist by the id the LLM selected, falling back to a loose
/// title/slug match for robustness.
pub fn find<'a>(lists: &'a [Checklist], want: &str) -> Option<&'a Checklist> {
    let want = slug(want);
    lists
        .iter()
        .find(|c| c.id == want)
        .or_else(|| lists.iter().find(|c| c.id.contains(&want) || want.contains(&c.id)))
}

/// The `run_checklist` tool spec, with the loaded checklist ids as the `name`
/// enum so the model can only pick one we actually have. Built dynamically
/// because the checklists come from a user-supplied file at runtime.
pub fn run_checklist_spec(lists: &[Checklist]) -> ToolSpec {
    let ids: Vec<&str> = lists.iter().map(|c| c.id.as_str()).collect();
    let names = lists
        .iter()
        .map(|c| c.title.to_lowercase())
        .collect::<Vec<_>>()
        .join(", ");
    ToolSpec {
        name: "run_checklist".to_string(),
        description: format!(
            "Read an aircraft checklist aloud, verifying each item against the \
             aircraft and offering to fix anything not set. Use for 'run the \
             before takeoff checklist', 'before takeoff checklist', 'do the \
             shutdown checklist', 'run the cruise checklist'. Available: {names}."
        ),
        parameters: json!({
            "type": "object",
            "properties": {
                "checklist": {
                    "type": "string",
                    "enum": ids,
                    "description": "Which checklist to run."
                }
            },
            "required": ["checklist"]
        }),
    }
}

/// One parsed checklist: a title and its ordered items.
#[derive(Debug, Clone, PartialEq)]
pub struct Checklist {
    /// Slug derived from the title, e.g. `before_takeoff` — what a tool selects by.
    pub id: String,
    /// Spoken title, e.g. "BEFORE TAKEOFF".
    pub title: String,
    pub items: Vec<Item>,
}

/// One checklist line.
#[derive(Debug, Clone, PartialEq)]
pub struct Item {
    /// The item name, e.g. "FUEL SELECTOR VALVE".
    pub challenge: String,
    /// The target state, e.g. "BOTH".
    pub response: String,
    /// How to verify it, if the format gave a dataref + condition.
    pub check: Option<Check>,
    /// Non-actionable context (`sw_itemvoid`/`sw_remark`, or a non-simulated
    /// `[bracketed]` line): spoken but never verified or fixed.
    pub info: bool,
}

impl Item {
    /// The spoken command that would set this item to its expected state, for
    /// routing through the normal tool pipeline when a verify fails (the hybrid
    /// "offer to fix" path). Only verifiable items yield one — a confirm-only
    /// item ("FUEL QUANTITY — CHECKED") has no settable state. Phrased as the
    /// pilot would say it ("fuel selector both", "flaps up"); if the resulting
    /// pick is wrong, the runner's re-verify catches it.
    pub fn fix_command(&self) -> Option<String> {
        self.check.as_ref()?;
        Some(format!("{} {}", self.challenge, self.response).to_lowercase())
    }
}

/// A verifiable condition: read `dataref` (optionally one array element) and test
/// it against `pred`.
#[derive(Debug, Clone, PartialEq)]
pub struct Check {
    pub dataref: String,
    pub index: Option<usize>,
    pub pred: Pred,
}

/// A pass condition on a dataref's numeric value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Pred {
    Eq(f64),
    Ne(f64),
    Gt(f64),
    Lt(f64),
    /// Inclusive range `[lo, hi]`.
    Range(f64, f64),
}

impl Pred {
    /// Does `v` satisfy the predicate? Exact comparisons use a small epsilon so
    /// e.g. a selector dataref reading 4.0 matches `Eq(4)` despite float noise.
    pub fn eval(&self, v: f64) -> bool {
        const EPS: f64 = 1e-6;
        match *self {
            Pred::Eq(t) => (v - t).abs() <= EPS,
            Pred::Ne(t) => (v - t).abs() > EPS,
            Pred::Gt(t) => v > t,
            Pred::Lt(t) => v < t,
            Pred::Range(lo, hi) => v >= lo - EPS && v <= hi + EPS,
        }
    }

    fn parse(cond: &str) -> Option<Pred> {
        let cond = cond.trim();
        if let Some(rest) = cond.strip_prefix('>') {
            return rest.trim().parse().ok().map(Pred::Gt);
        }
        if let Some(rest) = cond.strip_prefix('<') {
            return rest.trim().parse().ok().map(Pred::Lt);
        }
        if let Some(rest) = cond.strip_prefix('!') {
            return rest.trim().parse().ok().map(Pred::Ne);
        }
        if let Some((lo, hi)) = cond.split_once('|') {
            return Some(Pred::Range(lo.trim().parse().ok()?, hi.trim().parse().ok()?));
        }
        cond.parse().ok().map(Pred::Eq)
    }
}

/// Parse a whole XChecklist file into its checklists. Unknown directives and
/// comments are ignored; malformed conditions degrade to a non-verified item
/// rather than failing the parse.
pub fn parse(text: &str) -> Vec<Checklist> {
    let mut out: Vec<Checklist> = Vec::new();
    for raw in text.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let Some((directive, rest)) = line.split_once(':') else { continue };
        match directive {
            "sw_checklist" => {
                // `<menu>:<title>` — the title is the spoken name.
                let title = rest.rsplit(':').next().unwrap_or(rest).trim().to_string();
                out.push(Checklist { id: slug(&title), title, items: Vec::new() });
            }
            "sw_item" => {
                if let Some(item) = parse_item(rest, false) {
                    if let Some(cl) = out.last_mut() {
                        cl.items.push(item);
                    }
                }
            }
            "sw_itemvoid" | "sw_remark" => {
                if let Some(item) = parse_item(rest, true) {
                    if let Some(cl) = out.last_mut() {
                        cl.items.push(item);
                    }
                }
            }
            _ => {} // sw_rcolsize and friends: display-only, ignored.
        }
    }
    out
}

/// Parse the payload after `sw_item:` / `sw_itemvoid:` / `sw_remark:`.
/// `info` marks non-actionable lines. Returns `None` for pure separators.
fn parse_item(rest: &str, info: bool) -> Option<Item> {
    // The dataref + condition (if any) follow the response, each colon-delimited.
    // Datarefs use `/`, never `:`, so colons only ever separate the three fields.
    let mut fields = rest.split(':');
    let head = fields.next()?; // "CHALLENGE|RESPONSE"
    let dataref = fields.next();
    let cond = fields.next();

    let (challenge, response) = match head.split_once('|') {
        Some((c, r)) => (c.trim().to_string(), r.trim().to_string()),
        None => (head.trim().to_string(), String::new()),
    };
    // Drop visual separators (rows of underscores, blank info lines).
    if challenge.is_empty() || challenge.chars().all(|c| c == '_' || c.is_whitespace()) {
        return None;
    }
    // A `[bracketed]` challenge marks a non-simulated step (e.g. external
    // preflight): keep it as spoken context, never try to verify it.
    let bracketed = challenge.starts_with('[');

    let check = match (info || bracketed, dataref, cond.and_then(Pred::parse)) {
        (false, Some(dr), Some(pred)) => {
            let (dataref, index) = split_index(dr.trim());
            Some(Check { dataref, index, pred })
        }
        _ => None,
    };

    Some(Item { challenge, response, check, info: info || bracketed })
}

/// Split a trailing `[n]` array index off a dataref token.
fn split_index(dr: &str) -> (String, Option<usize>) {
    if let Some(open) = dr.find('[') {
        if let Some(close) = dr[open..].find(']') {
            if let Ok(i) = dr[open + 1..open + close].parse::<usize>() {
                return (dr[..open].to_string(), Some(i));
            }
        }
    }
    (dr.to_string(), None)
}

/// Lowercase the title into a stable id: non-alphanumerics collapse to single
/// underscores. "BEFORE TAKEOFF" → "before_takeoff".
fn slug(title: &str) -> String {
    let mut s = String::with_capacity(title.len());
    let mut prev_us = false;
    for c in title.chars() {
        if c.is_ascii_alphanumeric() {
            s.push(c.to_ascii_lowercase());
            prev_us = false;
        } else if !prev_us {
            s.push('_');
            prev_us = true;
        }
    }
    s.trim_matches('_').to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn predicate_forms_parse_and_eval() {
        assert_eq!(Pred::parse("4"), Some(Pred::Eq(4.0)));
        assert_eq!(Pred::parse(">0.9"), Some(Pred::Gt(0.9)));
        assert_eq!(Pred::parse("<0.02"), Some(Pred::Lt(0.02)));
        assert_eq!(Pred::parse("!4"), Some(Pred::Ne(4.0)));
        assert_eq!(Pred::parse("0.15|0.20"), Some(Pred::Range(0.15, 0.20)));
        assert_eq!(Pred::parse("-0.03|0.03"), Some(Pred::Range(-0.03, 0.03)));

        assert!(Pred::Eq(4.0).eval(4.0));
        assert!(!Pred::Eq(4.0).eval(3.0));
        assert!(Pred::Ne(4.0).eval(1.0));
        assert!(!Pred::Ne(4.0).eval(4.0));
        assert!(Pred::Gt(0.9).eval(0.95));
        assert!(Pred::Lt(0.02).eval(0.0));
        assert!(Pred::Range(-0.03, 0.03).eval(0.0));
        assert!(!Pred::Range(-0.03, 0.03).eval(0.5));
    }

    #[test]
    fn parses_a_verifiable_item_with_array_index() {
        let cl = &parse(
            "sw_checklist:STARTING ENGINE:STARTING ENGINE\n\
             sw_item:IGNITION SWITCH|START:sim/cockpit2/engine/actuators/ignition_key[0]:4",
        )[0];
        assert_eq!(cl.id, "starting_engine");
        let item = &cl.items[0];
        assert_eq!(item.challenge, "IGNITION SWITCH");
        assert_eq!(item.response, "START");
        let check = item.check.as_ref().unwrap();
        assert_eq!(check.dataref, "sim/cockpit2/engine/actuators/ignition_key");
        assert_eq!(check.index, Some(0));
        assert_eq!(check.pred, Pred::Eq(4.0));
    }

    #[test]
    fn item_without_dataref_is_spoken_confirm_only() {
        let cl = &parse("sw_checklist:BEFORE TAXI:BEFORE TAXI\nsw_item:FUEL QUANTITY|CHECKED")[0];
        let item = &cl.items[0];
        assert_eq!(item.challenge, "FUEL QUANTITY");
        assert_eq!(item.response, "CHECKED");
        assert!(item.check.is_none());
        assert!(!item.info);
    }

    #[test]
    fn range_and_negation_and_bracketed_items() {
        let cl = &parse(
            "sw_checklist:X:X\n\
             sw_item:ELEVATOR TRIM|SET FOR TAKEOFF:sim/cockpit2/controls/elevator_trim:-0.03|0.03\n\
             sw_item:FUEL SELECTOR VALVE|LEFT or RIGHT:sim/cockpit2/fuel/fuel_tank_selector:!4\n\
             sw_itemvoid:[EXTERNAL PREFLIGHT]|[COMPLETED]\n\
             sw_remark:MAGNETO TEST",
        )[0];
        assert_eq!(cl.items[0].check.as_ref().unwrap().pred, Pred::Range(-0.03, 0.03));
        assert_eq!(cl.items[1].check.as_ref().unwrap().pred, Pred::Ne(4.0));
        // Bracketed itemvoid: context only, never verified.
        assert!(cl.items[2].info && cl.items[2].check.is_none());
        // Remark becomes spoken context.
        assert!(cl.items[3].info);
        assert_eq!(cl.items[3].challenge, "MAGNETO TEST");
    }

    #[test]
    fn shipped_172_file_parses_cleanly() {
        let cls = parse(include_str!("../checklists/cessna_172sp.txt"));
        // The 12 operational checklists we ship.
        assert_eq!(cls.len(), 12, "expected 12 checklists, got {}", cls.len());
        let bt = cls.iter().find(|c| c.id == "before_takeoff").expect("before_takeoff");
        // A representative verifiable item resolved correctly.
        let trim = bt.items.iter().find(|i| i.challenge == "ELEVATOR TRIM").unwrap();
        assert_eq!(
            trim.check.as_ref().unwrap().dataref,
            "sim/cockpit2/controls/elevator_trim"
        );
        assert_eq!(trim.check.as_ref().unwrap().pred, Pred::Range(-0.03, 0.03));
        // Every non-info item with a dataref parsed into a real predicate (no
        // half-parsed verify silently dropped to a confirm).
        let verifiable = cls.iter().flat_map(|c| &c.items).filter(|i| i.check.is_some()).count();
        assert!(verifiable > 30, "only {verifiable} verifiable items parsed");
    }

    #[test]
    fn separators_are_dropped() {
        let cls = parse(
            "sw_checklist:X:X\n\
             sw_itemvoid:________________________________________________\n\
             sw_itemvoid:",
        );
        assert!(cls[0].items.is_empty());
    }
}
