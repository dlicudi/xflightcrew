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
//!
//! ## Compound conditions
//!
//! A pass test may join `dataref:cond` terms with `&&` and `||` under
//! parentheses (`(generator_on[0]:1)&&(rel_genera0:0)&&...`), `&&` binding
//! tighter than `||` as in C (the files read that way: "the GPU powered AND
//! not failed, OR the cart deployed"). Such an item is a VERIFY: every term
//! is read and the expression judged, and a term that does not read is no
//! measurement for the whole item ([`Expr::judge`]). It is an ACT only when
//! exactly one of its terms is something we could set (a mapped tool or a
//! writable switch), that term maps to a tool, and every other term is
//! read-only or a `sim/operation/failures/*` state. A term whose operator we
//! cannot judge from one read leaves the item a call-out: `><` (the value
//! CHANGED since the list began), `a:b` (it moved FROM a TO b), `+>` / `-<`
//! (rising / falling) and `{dataref}` arithmetic all need history or another
//! read, and an `||` with an unjudgeable branch could pass on the branch we
//! cannot see.

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
    /// `:>=N`
    Ge(f64),
    /// `:<=N`
    Le(f64),
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
        if let Some(n) = s.strip_prefix(">=") {
            return Some(Cond::Ge(n.trim().parse().ok()?));
        }
        if let Some(n) = s.strip_prefix("<=") {
            return Some(Cond::Le(n.trim().parse().ok()?));
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
            Cond::Ge(n) => v >= n,
            Cond::Le(n) => v <= n,
            Cond::Range(a, b) => v >= a && v <= b,
        }
    }

    /// A representative value that satisfies the condition — used to derive a
    /// target when acting (e.g. a throttle range → its midpoint).
    pub(crate) fn target(&self) -> f64 {
        match *self {
            Cond::Eq(n) | Cond::Ne(n) | Cond::Ge(n) | Cond::Le(n) => n,
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
            Cond::Gt(_) | Cond::Ge(_) => true,
            Cond::Lt(_) | Cond::Le(_) => false,
            Cond::Range(a, b) => (a + b) / 2.0 > 0.5,
        }
    }
}

/// One `dataref:cond` term of a pass test.
#[derive(Debug, Clone, PartialEq)]
pub struct Atom {
    /// The dataref without its `[index]`.
    pub dataref: String,
    pub index: Option<usize>,
    pub cond: Cond,
}

impl Atom {
    /// `dataref:cond`; `None` for an operator we cannot judge from one read.
    fn parse(s: &str) -> Option<Atom> {
        let (dataref, cond) = s.trim().split_once(':')?;
        let cond = Cond::parse(cond)?;
        let (dataref, index) = split_index(dataref.trim());
        Some(Atom {
            dataref,
            index,
            cond,
        })
    }

    /// The dataref as the file names it, `[index]` included (what the
    /// action map matches: a per-element entry only matches the full name).
    pub fn name(&self) -> String {
        match self.index {
            Some(i) => format!("{}[{i}]", self.dataref),
            None => self.dataref.clone(),
        }
    }
}

/// A compound pass test: terms joined by `&&` (`All`) and `||` (`Any`).
#[derive(Debug, Clone, PartialEq)]
pub enum Expr {
    Atom(Atom),
    All(Vec<Expr>),
    Any(Vec<Expr>),
}

/// A token of a condition spec.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Tok<'a> {
    Open,
    Close,
    And,
    Or,
    Term(&'a str),
}

fn tokens<'a>(s: &'a str) -> Vec<Tok<'a>> {
    let mut out = Vec::new();
    let b = s.as_bytes();
    let (mut i, mut start) = (0, 0);
    let flush = |out: &mut Vec<Tok<'a>>, from: usize, to: usize| {
        let t = s[from..to].trim();
        if !t.is_empty() {
            out.push(Tok::Term(t));
        }
    };
    while i < b.len() {
        let tok = match b[i] {
            b'(' => Some((Tok::Open, 1)),
            b')' => Some((Tok::Close, 1)),
            b'&' if b.get(i + 1) == Some(&b'&') => Some((Tok::And, 2)),
            b'|' if b.get(i + 1) == Some(&b'|') => Some((Tok::Or, 2)),
            _ => None,
        };
        match tok {
            Some((t, len)) => {
                flush(&mut out, start, i);
                out.push(t);
                i += len;
                start = i;
            }
            None => i += 1,
        }
    }
    flush(&mut out, start, b.len());
    out
}

impl Expr {
    /// Parse a whole condition spec. `None` when any part of it is beyond
    /// us: an unbalanced parenthesis, a function call, an operator a single
    /// read cannot judge. Never a partial expression.
    pub fn parse(spec: &str) -> Option<Expr> {
        let toks = tokens(spec);
        let mut at = 0;
        let e = Self::or(&toks, &mut at)?;
        (at == toks.len()).then_some(e)
    }

    fn or(t: &[Tok<'_>], at: &mut usize) -> Option<Expr> {
        let mut terms = vec![Self::and(t, at)?];
        while t.get(*at) == Some(&Tok::Or) {
            *at += 1;
            terms.push(Self::and(t, at)?);
        }
        Some(if terms.len() == 1 {
            terms.remove(0)
        } else {
            Expr::Any(terms)
        })
    }

    fn and(t: &[Tok<'_>], at: &mut usize) -> Option<Expr> {
        let mut terms = vec![Self::primary(t, at)?];
        while t.get(*at) == Some(&Tok::And) {
            *at += 1;
            terms.push(Self::primary(t, at)?);
        }
        Some(if terms.len() == 1 {
            terms.remove(0)
        } else {
            Expr::All(terms)
        })
    }

    fn primary(t: &[Tok<'_>], at: &mut usize) -> Option<Expr> {
        match *t.get(*at)? {
            Tok::Open => {
                *at += 1;
                let e = Self::or(t, at)?;
                if t.get(*at) != Some(&Tok::Close) {
                    return None;
                }
                *at += 1;
                Some(e)
            }
            Tok::Term(s) => {
                *at += 1;
                Atom::parse(s).map(Expr::Atom)
            }
            _ => None,
        }
    }

    /// Every term, in the file's order.
    pub fn atoms(&self) -> Vec<&Atom> {
        let mut out = Vec::new();
        self.collect(&mut out);
        out
    }

    fn collect<'a>(&'a self, out: &mut Vec<&'a Atom>) {
        match self {
            Expr::Atom(a) => out.push(a),
            Expr::All(es) | Expr::Any(es) => es.iter().for_each(|e| e.collect(out)),
        }
    }

    /// Judge the expression from each term's value. ANY term that did not
    /// read is no measurement: `None`, never a pass or a fail (an `||` whose
    /// other branch passes included: the list must not confirm what it did
    /// not read). Pure.
    pub fn judge(&self, value_of: impl Fn(&Atom) -> Option<f64>) -> Option<bool> {
        let values: Vec<f64> = self
            .atoms()
            .into_iter()
            .map(&value_of)
            .collect::<Option<_>>()?;
        let mut at = 0;
        Some(self.eval_in_order(&values, &mut at))
    }

    /// Every term is visited (no short circuit) so `at` walks the values in
    /// [`Expr::atoms`] order.
    fn eval_in_order(&self, values: &[f64], at: &mut usize) -> bool {
        match self {
            Expr::Atom(a) => {
                let v = values[*at];
                *at += 1;
                a.cond.eval(v)
            }
            Expr::All(es) => es
                .iter()
                .map(|e| e.eval_in_order(values, at))
                .fold(true, |acc, b| acc && b),
            Expr::Any(es) => es
                .iter()
                .map(|e| e.eval_in_order(values, at))
                .fold(false, |acc, b| acc || b),
        }
    }
}

/// A `sim/operation/failures/*` state: judged with the item, never acted on.
fn is_failure_dataref(d: &str) -> bool {
    d.contains("/operation/failures/")
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
        /// The whole pass test when the file gave a compound one; the
        /// `dataref`/`index`/`cond` above are then the term the tool acts
        /// on. `None`: that term IS the test.
        expr: Option<Expr>,
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
        /// The whole pass test when the file gave a compound one; the
        /// `dataref`/`index`/`cond` above are then its first term that is
        /// not a failure state (what a loop's ownership is judged by), and
        /// the item is never `settable`. `None`: that term IS the test.
        expr: Option<Expr>,
    },
}

impl Item {
    /// The pass test's datarefs (base names), the acted-on term first.
    /// Empty for narration and call-outs.
    pub fn datarefs(&self) -> Vec<&str> {
        match self {
            Item::Act { dataref, expr, .. } | Item::Verify { dataref, expr, .. } => {
                let mut out = vec![dataref.as_str()];
                for a in expr.iter().flat_map(Expr::atoms) {
                    if !out.contains(&a.dataref.as_str()) {
                        out.push(a.dataref.as_str());
                    }
                }
                out
            }
            Item::Narrate(_) | Item::CallOut(_) => Vec::new(),
        }
    }

    /// Does the item carry a pass test (a dataref to read)?
    pub fn has_test(&self) -> bool {
        matches!(self, Item::Act { .. } | Item::Verify { .. })
    }

    /// Every term the item's pass test reads, in the file's order.
    pub fn atoms(&self) -> Vec<Atom> {
        match self {
            Item::Act {
                dataref,
                index,
                cond,
                expr,
                ..
            }
            | Item::Verify {
                dataref,
                index,
                cond,
                expr,
                ..
            } => match expr {
                Some(e) => e.atoms().into_iter().cloned().collect(),
                None => vec![Atom {
                    dataref: dataref.clone(),
                    index: *index,
                    cond: cond.clone(),
                }],
            },
            Item::Narrate(_) | Item::CallOut(_) => Vec::new(),
        }
    }

    /// Judge the item from values already read: the compound expression,
    /// else the one term. `None` for no test, or a term that did not read.
    /// Pure.
    pub fn judge_with(&self, value_of: impl Fn(&Atom) -> Option<f64>) -> Option<bool> {
        match self {
            Item::Act { expr: Some(e), .. } | Item::Verify { expr: Some(e), .. } => {
                e.judge(value_of)
            }
            Item::Act { .. } | Item::Verify { .. } => {
                let a = self.atoms().remove(0);
                value_of(&a).map(|v| a.cond.eval(v))
            }
            Item::Narrate(_) | Item::CallOut(_) => None,
        }
    }
}

/// What the runner read of one item: its verdict (`None` = no test, or a
/// term that did not read) and every term's value, in the file's order.
#[derive(Debug, Clone, PartialEq)]
pub struct Reading {
    pub satisfied: Option<bool>,
    pub values: Vec<Option<f64>>,
}

/// Read every term of an item's pass test, in one join, and judge it.
pub async fn read_item(xp: &XPlaneClient, item: &Item) -> Reading {
    let atoms = item.atoms();
    let values: Vec<Option<f64>> =
        futures::future::join_all(atoms.iter().map(|a| read_dataref(xp, &a.dataref, a.index)))
            .await;
    Reading {
        satisfied: judge_read(item, &values),
        values,
    }
}

/// The item's verdict from its terms' values, read in [`Item::atoms`] order.
/// Pure.
pub fn judge_read(item: &Item, values: &[Option<f64>]) -> Option<bool> {
    // `judge_with` asks for each term once, in `atoms` order.
    let next = std::cell::Cell::new(0usize);
    item.judge_with(|_| {
        let i = next.get();
        next.set(i + 1);
        values.get(i).copied().flatten()
    })
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
    /// The `checklist_flow:` key this list was JOINED for (its parts read in
    /// order as one list); `None` for a section as the file has it.
    pub section: Option<String>,
}

/// Read and interpret the airframe's XChecklist file into runnable checklists.
/// `Ok(vec![])` if the profile names no checklist file. A relative path
/// (`checklists/<file>`) is a shipped file (`sim::paths::resource`).
pub fn load(profile: &AircraftProfile) -> Result<Vec<Checklist>> {
    let Some(path) = &profile.checklist else {
        return Ok(Vec::new());
    };
    if let Some(file) = path.strip_prefix(ACF_PREFIX) {
        bail!(
            "the checklist is {file} in the aircraft's own folder, and no loaded aircraft named that folder"
        );
    }
    let text = std::fs::read_to_string(sim::paths::resource(path))
        .with_context(|| format!("reading checklist file {path}"))?;
    Ok(parse(&text, profile))
}

/// A profile's `checklist:` naming a file in the aircraft's own folder:
/// `acf:clist.txt`. The file ships with the aircraft and is not ours to copy.
pub const ACF_PREFIX: &str = "acf:";

/// Resolve an `acf:<file>` checklist against the loaded aircraft: the
/// `.acf`'s directory (`acf_relative_path`, relative to the install `root`).
/// Any other value is returned as it is; `None` when the `.acf` path names
/// no folder. Pure.
pub fn resolve_acf_path(checklist: &str, acf_relative_path: &str, root: &str) -> Option<String> {
    let Some(file) = checklist.strip_prefix(ACF_PREFIX) else {
        return Some(checklist.to_string());
    };
    let rel = acf_relative_path.trim().replace('\\', "/");
    let dir = std::path::Path::new(&rel).parent()?.to_str()?;
    if dir.is_empty() {
        return None;
    }
    Some(format!("{root}/{dir}/{}", file.trim()))
}

/// The line said once when the profile names a checklist the copilot
/// cannot read (an `acf:` file missing from the aircraft's folder, a moved
/// file): no flows run for this airframe, and the pilot should know why.
/// `None` when there is no checklist or it reads.
pub fn unavailable_line(profile: &AircraftProfile) -> Option<String> {
    let path = profile.checklist.as_deref()?;
    let file = path.strip_prefix(ACF_PREFIX).unwrap_or(path);
    let name = std::path::Path::new(file)
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or(file);
    let readable =
        !path.starts_with(ACF_PREFIX) && std::fs::metadata(sim::paths::resource(path)).is_ok();
    (!readable).then(|| {
        format!(
            "I can't find this aircraft's checklist file, {name}, so I won't run its checklists."
        )
    })
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
    // never part of a section name - drop them with the empties. A single
    // DIGIT is a part's number ("preliminary preflight procedure 1" and its
    // "2", the MD-82's), kept so the parts are told apart.
    let mut words: Vec<String> = text
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| w.len() > 1 || w.bytes().any(|c| c.is_ascii_digit()))
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
                    section: None,
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
                    // Only the map acts on this airframe: the verify path
                    // reads, never writes.
                    if profile.checklist_map_only {
                        if let Item::Verify { settable, .. } = &mut item {
                            *settable = false;
                        }
                    }
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
    join_flows(&mut out, profile);
    out
}

/// Join the sections a `checklist_flow:` entry lists (more than one title)
/// into ONE list, read in the order given, appended after the file's own
/// sections, which stay as they are (a spoken "climb procedure one of two"
/// still finds its part). The joined list is titled by the parts' shared
/// name ("CLIMB PROCEDURE (1/2)" -> "Climb procedure"). A title the file
/// lacks is skipped and logged; none found, no list.
fn join_flows(out: &mut Vec<Checklist>, profile: &AircraftProfile) {
    for (key, titles) in &profile.checklist_flow {
        let titles = titles.titles();
        if titles.len() < 2 {
            continue;
        }
        let parts: Vec<usize> = titles
            .iter()
            .filter_map(|t| {
                let i = out
                    .iter()
                    .position(|c| c.section.is_none() && c.title.eq_ignore_ascii_case(t.trim()));
                if i.is_none() {
                    tracing::warn!(key = %key, title = %t, "checklist_flow names a section the file lacks");
                }
                i
            })
            .collect();
        let Some(&first) = parts.first() else {
            continue;
        };
        let base = joined_title(titles[0]);
        let title = if out.iter().any(|c| c.title.eq_ignore_ascii_case(&base)) {
            format!("{base} (all parts)")
        } else {
            base
        };
        let items = parts
            .iter()
            .flat_map(|&i| out[i].items.iter().map(clone_item))
            .collect();
        let f = &out[first];
        let joined = Checklist {
            id: slug(&title),
            title,
            items,
            lights: f.lights.clone(),
            flaps: f.flaps.clone(),
            electrical: f.electrical.clone(),
            fuel_selector: f.fuel_selector.clone(),
            engines: f.engines,
            magnetos_per_engine: f.magnetos_per_engine,
            park_brake_valve_set: f.park_brake_valve_set,
            pitot_heat_dataref: f.pitot_heat_dataref.clone(),
            section: Some(section_key(key)),
        };
        out.push(joined);
    }
}

/// "before_landing" / "Before Landing" -> "before landing": a
/// `checklist_flow:` key as the flow module compares it.
pub fn section_key(k: &str) -> String {
    k.to_ascii_lowercase().replace('_', " ").trim().to_string()
}

/// "CLIMB PROCEDURE (1/2)" -> "Climb procedure": the title without its
/// trailing parenthesised part.
fn joined_title(t: &str) -> String {
    let t = t.trim();
    let base = match t.rfind(" (") {
        Some(i) if t.ends_with(')') => &t[..i],
        _ => t,
    };
    humanize(base)
}

fn clone_item(i: &Item) -> Item {
    match i {
        Item::Narrate(t) => Item::Narrate(t.clone()),
        Item::CallOut(t) => Item::CallOut(t.clone()),
        Item::Act {
            say,
            tool,
            args,
            confirm,
            dataref,
            index,
            cond,
            expr,
        } => Item::Act {
            say: say.clone(),
            tool,
            args: args.clone(),
            confirm: *confirm,
            dataref: dataref.clone(),
            index: *index,
            cond: cond.clone(),
            expr: expr.clone(),
        },
        Item::Verify {
            say,
            dataref,
            index,
            cond,
            settable,
            expr,
        } => Item::Verify {
            say: say.clone(),
            dataref: dataref.clone(),
            index: *index,
            cond: cond.clone(),
            settable: *settable,
            expr: expr.clone(),
        },
    }
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
    // A test we can't judge from one read of each term (a function such as
    // `closer_than(...)`, `{dataref}` arithmetic, `><`, `a:b`, an unbalanced
    // parenthesis) - we can't act on or verify it honestly, so it is a
    // call-out (the pilot's item).
    let Some(expr) = Expr::parse(condspec) else {
        return Item::CallOut(say);
    };
    let atom = match expr {
        Expr::Atom(a) => a,
        compound => return compound_item(say, compound, map, fuel),
    };
    let name = atom.name();
    let Atom {
        dataref: base,
        index,
        cond,
    } = atom;

    // The map sees the dataref WITH its index: a per-element entry
    // ("landing_lights_switch[1]", the A330's taxi light on a three-position
    // nose-light switch, 2026-09-20) can only match against the full name,
    // and a base-only entry matches the full name just the same.
    match action_for(map, &name, &cond, fuel) {
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
                expr: None,
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
                expr: None,
            }
        }
    }
}

/// A compound pass test. A VERIFY, never written raw; an ACT only when
/// exactly ONE term is something we could set (a mapped tool, or a writable
/// switch) and that term maps: two settable terms ("generators 1 AND 2",
/// "nav AND NOT strobe", "knob at 1 OR at 2") name no single action. Failure
/// states (`sim/operation/failures/*`) are judged with the item but never
/// counted or acted on.
fn compound_item(
    say: String,
    expr: Expr,
    map: &[ChecklistMapEntry],
    fuel: &[FuelPosition],
) -> Item {
    let atoms = expr.atoms();
    let live: Vec<&Atom> = atoms
        .iter()
        .copied()
        .filter(|a| !is_failure_dataref(&a.dataref))
        .collect();
    let settable: Vec<(&Atom, Option<(&'static str, Value)>)> = live
        .iter()
        .map(|a| (*a, action_for(map, &a.name(), &a.cond, fuel)))
        .filter(|(a, act)| act.is_some() || !is_readonly_dataref(&a.dataref))
        .collect();
    if let [(a, Some((tool, args)))] = settable.as_slice() {
        let (tool, args, a) = (*tool, args.clone(), (*a).clone());
        let confirm = confirm_for(tool, &args);
        return Item::Act {
            say,
            tool,
            args,
            confirm,
            dataref: a.dataref,
            index: a.index,
            cond: a.cond,
            expr: Some(expr),
        };
    }
    let primary = (*live
        .first()
        .or(atoms.first())
        .expect("an expression has a term"))
    .clone();
    Item::Verify {
        say,
        dataref: primary.dataref,
        index: primary.index,
        cond: primary.cond,
        settable: false,
        expr: Some(expr),
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
        // Compound (&&) condition → a verify of both terms (two batteries
        // name no single action), never written raw.
        match parse_item(
            r"\green\Batteries\yellow\, both on|\white\CHECK:(sim/.../battery_on[0]:1)&&(sim/.../battery_on[1]:1)",
        ) {
            Item::Verify { settable, expr, .. } => {
                assert!(!settable);
                assert_eq!(expr.expect("compound").atoms().len(), 2);
            }
            other => panic!("expected Verify, got {}", variant(&other)),
        }
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

    // ---- compound conditions -------------------------------------------

    fn atom(d: &str, cond: Cond) -> Expr {
        Expr::Atom(Atom {
            dataref: d.into(),
            index: None,
            cond,
        })
    }

    #[test]
    fn compound_grammar_binds_and_over_or() {
        // `a && b || c` is `(a && b) || c`, as in C.
        assert_eq!(
            Expr::parse("(a:1)&&(b:>0)||(c:1)"),
            Some(Expr::Any(vec![
                Expr::All(vec![atom("a", Cond::Eq(1.0)), atom("b", Cond::Gt(0.0))]),
                atom("c", Cond::Eq(1.0)),
            ]))
        );
        assert_eq!(
            Expr::parse("(c:1)||(a:1)&&(b:0)"),
            Some(Expr::Any(vec![
                atom("c", Cond::Eq(1.0)),
                Expr::All(vec![atom("a", Cond::Eq(1.0)), atom("b", Cond::Eq(0.0))]),
            ]))
        );
        // Parentheses group; doubled ones are harmless; an index survives.
        assert_eq!(
            Expr::parse("((x[2]:1))&&((y:0)||(z:0))"),
            Some(Expr::All(vec![
                Expr::Atom(Atom {
                    dataref: "x".into(),
                    index: Some(2),
                    cond: Cond::Eq(1.0),
                }),
                Expr::Any(vec![atom("y", Cond::Eq(0.0)), atom("z", Cond::Eq(0.0))]),
            ]))
        );
        // A range's single `|` is the range, not an `||`.
        assert_eq!(
            Expr::parse("(r:-0.005|0.005)&&(s:!2000)"),
            Some(Expr::All(vec![
                atom("r", Cond::Range(-0.005, 0.005)),
                atom("s", Cond::Ne(2000.0)),
            ]))
        );
        // `>=` / `<=` are the inclusive bounds.
        assert_eq!(Cond::parse(">=0.125"), Some(Cond::Ge(0.125)));
        assert_eq!(Cond::parse("<=3"), Some(Cond::Le(3.0)));
        assert!(Cond::Ge(0.125).eval(0.125) && !Cond::Ge(0.125).eval(0.124));
        assert!(Cond::Le(3.0).eval(3.0) && !Cond::Le(3.0).eval(3.1));
        // Never a partial expression: any term or shape we cannot judge from
        // one read is no expression at all.
        for spec in [
            "(a:1)&&(b:><0)",              // changed since the start
            "(a:0:1)||(b:1)",              // moved from 0 to 1
            "(a:+>0.9)",                   // rising
            "a:{other/dref}",              // another dataref's value
            "a:round({x})-1|round({x})+1", // arithmetic
            "1:closer_than({a},{b},0.02)", // a function
            "(a:1)&&b:1)",                 // unbalanced (the MD-82's)
            "(a:1)&&",                     // dangling operator
            "(a:1)(b:1)",                  // no operator
            "",
        ] {
            assert_eq!(Expr::parse(spec), None, "{spec}");
        }
    }

    #[test]
    fn a_term_that_did_not_read_is_no_measurement() {
        let e = Expr::parse("(gen0:1)&&(rel_gen0:0)||(gpu:1)").unwrap();
        let read = |vals: [Option<f64>; 3]| {
            let names = ["gen0", "rel_gen0", "gpu"];
            e.judge(|a| vals[names.iter().position(|n| *n == a.dataref).unwrap()])
        };
        assert_eq!(read([Some(1.0), Some(0.0), Some(0.0)]), Some(true));
        assert_eq!(read([Some(1.0), Some(6.0), Some(0.0)]), Some(false));
        assert_eq!(read([Some(0.0), Some(0.0), Some(1.0)]), Some(true));
        assert_eq!(read([Some(0.0), Some(0.0), Some(0.0)]), Some(false));
        // One failed read: no verdict either way, even where the other
        // branch alone would pass.
        assert_eq!(read([None, Some(0.0), Some(0.0)]), None);
        assert_eq!(read([Some(0.0), Some(0.0), None]), None);
        assert_eq!(read([None, Some(0.0), Some(1.0)]), None);

        // The item judges the same way, through `judge_read`'s term order.
        let it = super::parse_item(
            "GENERATORS|ON:(sim/a/generator_on[0]:1)&&(sim/operation/failures/rel_genera0:0)",
            &[],
            &[],
        );
        assert_eq!(judge_read(&it, &[Some(1.0), Some(0.0)]), Some(true));
        assert_eq!(judge_read(&it, &[Some(1.0), Some(6.0)]), Some(false));
        assert_eq!(judge_read(&it, &[Some(1.0), None]), None);
        // A simple item: its one term.
        let simple = super::parse_item(
            "PARKING BRAKE|ON:sim/flightmodel/controls/parkbrake:1",
            &[],
            &[],
        );
        assert_eq!(judge_read(&simple, &[Some(1.0)]), Some(true));
        assert_eq!(judge_read(&simple, &[None]), None);
    }

    #[test]
    fn a_compound_item_acts_only_through_one_mapped_term() {
        let m = p172().checklist_map;
        let parse_item = |r: &str| super::parse_item(r, &m, &[]);
        // One switch plus its failure state: an act on that switch, the
        // whole test kept for the read.
        match parse_item(
            "BEACON|ON:(sim/cockpit/electrical/beacon_lights_on:1)&&(sim/operation/failures/rel_lites_beac:0)",
        ) {
            Item::Act {
                tool,
                args,
                dataref,
                expr,
                ..
            } => {
                assert_eq!(tool, "set_lights");
                assert_eq!(args, json!({ "light": "beacon", "on": true }));
                assert_eq!(dataref, "sim/cockpit/electrical/beacon_lights_on");
                assert_eq!(expr.unwrap().atoms().len(), 2);
            }
            other => panic!("expected Act, got {}", variant(&other)),
        }
        // A mapped switch and a read-only guard: still one thing to set.
        assert!(matches!(
            parse_item("BEACON|ON:(sim/cockpit/electrical/beacon_lights_on:1)&&(laminar/x/cover_position[2]:0)"),
            Item::Act { .. }
        ));
        // Two settable terms name no single action: nav AND NOT strobe.
        assert!(matches!(
            parse_item(
                "POSITION|STEADY:(sim/cockpit2/switches/navigation_lights_on:1)&&(sim/cockpit2/switches/strobe_lights_on:0)"
            ),
            Item::Verify { settable: false, .. }
        ));
        // A mapped switch and an unmapped writable one: a read, never a
        // half-done act.
        assert!(matches!(
            parse_item("BEACON|ON:(sim/cockpit/electrical/beacon_lights_on:1)&&(sim/cockpit2/switches/other:1)"),
            Item::Verify { settable: false, .. }
        ));
        // The same mapped term twice ("knob at 4 OR knob at 4"): two terms.
        assert!(matches!(
            parse_item("BEACON|ON:(sim/cockpit/electrical/beacon_lights_on:1)||(sim/cockpit/electrical/beacon_lights_on:1)"),
            Item::Verify { .. }
        ));
        // Nothing mapped: a read whose primary is the first live term.
        match parse_item("FIRE|TEST:(sim/operation/failures/rel_x:0)&&(laminar/b/fire_test:>0)") {
            Item::Verify {
                dataref, settable, ..
            } => {
                assert_eq!(dataref, "laminar/b/fire_test");
                assert!(!settable);
            }
            other => panic!("expected Verify, got {}", variant(&other)),
        }
        // A lone parenthesised term is a simple item.
        assert!(matches!(
            parse_item("BEACON|OFF:(sim/cockpit/electrical/beacon_lights_on:0)"),
            Item::Act { expr: None, .. }
        ));
    }

    #[test]
    fn an_acf_checklist_resolves_in_the_aircraft_folder() {
        let root = "/X-Plane 12";
        assert_eq!(
            resolve_acf_path("acf:clist.txt", "Aircraft/B737-800X/b738_4k.acf", root).as_deref(),
            Some("/X-Plane 12/Aircraft/B737-800X/clist.txt")
        );
        assert_eq!(
            resolve_acf_path("acf:clist.txt", r"Aircraft\B737-800X\b738_4k.acf", root).as_deref(),
            Some("/X-Plane 12/Aircraft/B737-800X/clist.txt")
        );
        // Not an `acf:` path: as it is.
        assert_eq!(
            resolve_acf_path("checklists/md82.txt", "Aircraft/x/y.acf", root).as_deref(),
            Some("checklists/md82.txt")
        );
        // No folder named: nothing to resolve against.
        assert_eq!(resolve_acf_path("acf:clist.txt", "", root), None);
        assert_eq!(resolve_acf_path("acf:clist.txt", "b738.acf", root), None);

        // Unresolved (a profile loaded by id, no aircraft): refused, said,
        // never a crash.
        let mut p = p172();
        p.checklist = Some("acf:clist.txt".into());
        assert!(load(&p).is_err());
        assert!(unavailable_line(&p).unwrap().contains("clist.txt"));
        p.checklist = Some("/nonexistent/folder/clist.txt".into());
        assert!(load(&p).is_err());
        assert!(unavailable_line(&p).is_some());
        assert_eq!(unavailable_line(&p172()), None);
    }

    /// The Zibo's own XChecklist, where it is installed (CI has no X-Plane:
    /// skipped there). The counts pin what the interpreter makes of the
    /// file; the mapped flows must all resolve, joined parts in order.
    #[test]
    fn the_zibo_file_reads_and_its_flows_resolve() {
        let root = crate::config::xplane_root();
        let Some(path) = resolve_acf_path("acf:clist.txt", "Aircraft/B737-800X/b738_4k.acf", &root)
        else {
            return;
        };
        let Ok(text) = std::fs::read_to_string(&path) else {
            eprintln!("no Zibo checklist at {path}: skipped");
            return;
        };
        let mut p = crate::aircraft::load("zibo_738").expect("zibo profile");
        p.checklist = Some(path);
        assert!(p.checklist_map_only);
        let lists = parse(&text, &p);
        let file_lists: Vec<&Checklist> = lists.iter().filter(|c| c.section.is_none()).collect();
        let (mut acts, mut verifies, mut callouts, mut compound) = (0, 0, 0, 0);
        let mut tools = std::collections::BTreeSet::new();
        for it in file_lists.iter().flat_map(|c| &c.items) {
            match it {
                Item::Act { tool, expr, .. } => {
                    acts += 1;
                    tools.insert(*tool);
                    compound += expr.is_some() as usize;
                }
                Item::Verify { settable, expr, .. } => {
                    verifies += 1;
                    assert!(!settable, "map only: nothing is written raw");
                    compound += expr.is_some() as usize;
                }
                Item::CallOut(_) => callouts += 1,
                Item::Narrate(_) => {}
            }
        }
        eprintln!("zibo: {acts} act, {verifies} verify ({compound} compound), {callouts} call-out; tools {tools:?}");
        assert_eq!((acts, verifies, callouts), ZIBO_COUNTS, "{tools:?}");
        // Only the tools the profile says work here.
        for t in &tools {
            assert!(
                [
                    "set_parking_brake",
                    "set_transponder_mode",
                    "set_lights",
                    "set_electrical"
                ]
                .contains(t),
                "acts through {t}"
            );
        }
        // Every phase and ramp key resolves; a joined phase is its parts in
        // order, under its shared name.
        let overrides = crate::flow::overrides_for(&p, &lists);
        for key in [
            "climb",
            "cruise",
            "descent",
            "before landing",
            "after landing",
            "preflight",
            "engine start",
            "before taxi",
            "before takeoff",
            "shutdown",
        ] {
            let i = crate::flow::find_section_in(&lists, key, &overrides)
                .unwrap_or_else(|| panic!("no list for {key}"));
            assert!(!lists[i].items.is_empty(), "{key}");
        }
        let climb = &lists[crate::flow::find_section_in(&lists, "climb", &overrides).unwrap()];
        assert_eq!(climb.title, "Climb procedure");
        let part = |t: &str| {
            lists
                .iter()
                .find(|c| c.title.eq_ignore_ascii_case(t))
                .unwrap()
                .items
                .len()
        };
        assert_eq!(
            climb.items.len(),
            part("CLIMB PROCEDURE (1/2)") + part("CLIMB PROCEDURE (2/2)")
        );
        // Spoken titles: "procedure" is "checklist", the parts' suffixes no
        // obstacle, the joined list wins its own name.
        for (asked, want) in [
            ("before taxi checklist", "Before taxi procedure"),
            ("the before takeoff checklist", "Before takeoff procedure"),
            ("climb", "Climb procedure"),
            ("descent checklist", "Descent procedure"),
            ("after landing", "After landing procedure"),
        ] {
            let i = crate::flow::find_titled(&lists, asked)
                .unwrap_or_else(|e| panic!("{asked}: {e:?}"));
            assert_eq!(lists[i].title, want, "{asked}");
        }
        assert_eq!(
            match_request("run the before taxi checklist", &lists).map(|c| c.title.as_str()),
            Some("Before taxi procedure")
        );
        assert_eq!(
            match_request("climb checklist", &lists).map(|c| c.title.as_str()),
            Some("Climb procedure")
        );
    }

    /// The Zibo's ground lists judged against the LIVE sim, read-only (GETs
    /// only, nothing written, no command fired): every tested item's verdict
    /// and term values, for a parked-state survey. Needs X-Plane with the
    /// Zibo loaded: `cargo test -- --ignored live_zibo_ground_lists
    /// --nocapture`.
    #[tokio::test]
    #[ignore]
    async fn live_zibo_ground_lists() {
        let root = crate::config::xplane_root();
        let path =
            resolve_acf_path("acf:clist.txt", "Aircraft/B737-800X/b738_4k.acf", &root).unwrap();
        let text = std::fs::read_to_string(&path).expect("the Zibo's clist.txt");
        let mut p = crate::aircraft::load("zibo_738").expect("zibo profile");
        p.checklist = Some(path);
        let lists = parse(&text, &p);
        let xp = XPlaneClient::new("http://localhost:8086".into());
        for title in ["Before taxi procedure", "Before takeoff procedure"] {
            let cl = lists.iter().find(|c| c.title == title).expect(title);
            let (mut pass, mut fail, mut none) = (0, 0, 0);
            for it in &cl.items {
                let kind = match it {
                    Item::Act { .. } => "act",
                    Item::Verify { .. } => "verify",
                    _ => continue,
                };
                let r = read_item(&xp, it).await;
                let verdict = match r.satisfied {
                    Some(true) => {
                        pass += 1;
                        "PASS"
                    }
                    Some(false) => {
                        fail += 1;
                        "FAIL"
                    }
                    None => {
                        none += 1;
                        "NO MEASUREMENT"
                    }
                };
                let terms: Vec<String> = it
                    .atoms()
                    .iter()
                    .zip(&r.values)
                    .map(|(a, v)| format!("{}={v:?}", a.name()))
                    .collect();
                eprintln!("{title} [{kind}] {verdict}: {} {terms:?}", item_line_for_test(it));
            }
            eprintln!("{title}: {pass} pass, {fail} fail, {none} no measurement");
        }
    }

    fn item_line_for_test(i: &Item) -> &str {
        match i {
            Item::Narrate(t) | Item::CallOut(t) => t,
            Item::Act { say, .. } | Item::Verify { say, .. } => say,
        }
    }

    /// (act, verify, call-out) over the Zibo file's own sections, v9.0.
    const ZIBO_COUNTS: (usize, usize, usize) = (14, 428, 218);
}
