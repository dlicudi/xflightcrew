//! `xfc debrief --sortie <id|path|last>` and `xfc debrief --airframe
//! <profile> [--since YYYY-MM-DD]`: why a sortie ended, read from its own
//! file (`logs/sorties/<id>.jsonl`).
//!
//! Finding why an A330 sortie gave up took an hour of grep (2026-10-14): the
//! `ap_pitch_authority` verdict `no_authority` (vs -1681) came half a second
//! after the copilot reset the altimeter 30.32 -> 29.92, while the 1 Hz
//! `state` records a second earlier said +2136. This page says that at once:
//! the header and stage verdicts, the moment the mission ended, the ~30 s of
//! DECISION records before it, and [`RULES`]-driven contradiction flags.
//!
//! Pure over the lines like the rest of the debrief; only [`main_cmd`]
//! touches the disk. A sortie file does not carry the airframe, so the
//! profile is the latest `profile_probe` in the blackbox at or before the
//! sortie began (streamed, only lines naming `profile_probe` parsed).
//! Compressed (`.gz`) sortie files and rotated blackboxes are skipped.

use serde_json::Value;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// Seconds of decision records shown before the moment a sortie ended.
const TIMELINE_BEFORE_S: f64 = 30.0;
/// Seconds after that moment still shown (the cascade it set off).
const TIMELINE_AFTER_S: f64 = 5.0;
/// How far back from the sortie's end a `stand_down` or recovery take-over
/// is still the moment it ended (the cascade into `give_up`).
const CASCADE_S: f64 = 60.0;
/// A verdict's measured vs against the nearest `state` vs within this.
const STATE_NEAR_S: f64 = 3.0;
/// Disagreement worth a flag, fpm.
const VS_DISAGREE_FPM: f64 = 1000.0;
/// An altimeter change this long before an adverse verdict is flagged...
const ALTIMETER_BEFORE_S: f64 = 5.0;
/// ...or this long after it: the change's own record lands after its
/// callout, in the same tick as the verdict (+0.2 s on the A330 above).
const ALTIMETER_AFTER_S: f64 = 1.0;

/// A verdict that carries a vs it measured: where to find the verdict and
/// the number. Add a row to check another.
struct MeasuredVs {
    kind: &'static str,
    event: &'static str,
    vs: &'static [&'static str],
}

const RULES: &[MeasuredVs] = &[
    MeasuredVs {
        kind: "ap_pitch_authority",
        event: "verdict",
        vs: &["numbers", "vs"],
    },
    MeasuredVs {
        kind: "procedure",
        event: "climb_watch",
        vs: &["vs"],
    },
];

/// Verdict strings that are not a failure: no altimeter flag for them.
const BENIGN_VERDICTS: &[&str] = &["confirmed", "inconclusive"];

/// Record kinds that are decisions, whatever their event.
const DECISION_KINDS: &[&str] = &[
    "stand_down",
    "recover",
    "ap_pitch_authority",
    "climb_watch",
    "autoflight",
    "altimeter",
    "lnav",
    "full_auto",
    "crash",
];
/// Events that are decisions inside any kind (`procedure` `stand_down`).
const DECISION_EVENTS: &[&str] = &["stand_down", "climb_watch", "altimeter_set", "take_over"];

fn s<'a>(v: &'a Value, k: &str) -> &'a str {
    v.get(k).and_then(Value::as_str).unwrap_or("")
}
fn t_of(v: &Value) -> f64 {
    v.get("t").and_then(Value::as_f64).unwrap_or(0.0)
}
fn at<'a>(v: &'a Value, path: &[&str]) -> Option<&'a Value> {
    path.iter().try_fold(v, |v, k| v.get(*k))
}

/// The lines as records, in time order (file order kept within a tick).
pub fn parse<'a>(lines: impl Iterator<Item = &'a str>) -> Vec<Value> {
    let mut v: Vec<Value> = lines
        .filter_map(|l| serde_json::from_str::<Value>(l).ok())
        .filter(|v| v.get("t").and_then(Value::as_f64).is_some())
        .collect();
    v.sort_by(|a, b| t_of(a).total_cmp(&t_of(b)));
    v
}

/// One sortie, summarised.
#[derive(Debug, Clone, Default)]
pub struct Summary {
    pub id: String,
    pub t0: f64,
    pub origin: String,
    pub dest: String,
    /// `sortie_result.outcome`, else the `sortie end` outcome.
    pub outcome: Option<String>,
    pub pass: Option<bool>,
    /// The `give_up` reason, else the `sortie end` reason.
    pub reason: Option<String>,
    pub end_line: Option<String>,
    /// Every `stage_result`: stage, pass, why.
    pub stages: Vec<(String, bool, Vec<String>)>,
    /// A profile the file itself names, if any.
    pub profile: Option<String>,
}

impl Summary {
    pub fn failed(&self) -> impl Iterator<Item = &(String, bool, Vec<String>)> {
        self.stages.iter().filter(|st| !st.1)
    }
}

pub fn summarise(events: &[Value]) -> Summary {
    let mut out = Summary::default();
    for e in events {
        if out.id.is_empty() && !s(e, "sortie").is_empty() {
            out.id = s(e, "sortie").to_string();
        }
        match (s(e, "kind"), s(e, "event")) {
            ("sortie", "begin") => {
                out.t0 = t_of(e);
                out.origin = s(e, "origin").to_string();
                out.dest = s(e, "dest").to_string();
            }
            ("sortie", "end") => {
                if out.outcome.is_none() {
                    out.outcome = Some(s(e, "outcome").to_string()).filter(|o| !o.is_empty());
                }
                if out.reason.is_none() {
                    out.reason = Some(s(e, "reason").to_string()).filter(|o| !o.is_empty());
                }
                out.end_line = Some(s(e, "line").to_string()).filter(|o| !o.is_empty());
            }
            ("sortie_result", _) => {
                out.outcome = Some(s(e, "outcome").to_string()).filter(|o| !o.is_empty());
                out.pass = e.get("pass").and_then(Value::as_bool);
            }
            ("stage_result", _) => {
                let why = e
                    .get("why")
                    .and_then(Value::as_array)
                    .map(|a| {
                        a.iter()
                            .filter_map(|w| w.as_str().map(String::from))
                            .collect()
                    })
                    .unwrap_or_default();
                out.stages.push((
                    s(e, "stage").to_string(),
                    e.get("pass").and_then(Value::as_bool).unwrap_or(false),
                    why,
                ));
            }
            ("full_auto", _) if s(e, "stage") == "give_up" => {
                if let Some(r) = at(e, &["detail", "reason"]).and_then(Value::as_str) {
                    out.reason = Some(r.to_string());
                }
            }
            ("profile_probe", _) | ("flight_state", _) => {
                if let Some(p) = e.get("profile").and_then(Value::as_str) {
                    out.profile = Some(p.to_string());
                }
            }
            ("flight_meta", _) => {
                if let Some(p) = e.get("aircraft").and_then(Value::as_str) {
                    out.profile = Some(p.to_string());
                }
            }
            _ => {}
        }
    }
    if out.t0 == 0.0 {
        out.t0 = events.first().map(t_of).unwrap_or(0.0);
    }
    out
}

/// The record a non-pass sortie ended on: the sortie's end anchor
/// (`give_up` or `sortie end`, the first), walked back to the earliest
/// `stand_down` or recovery `take_over` within [`CASCADE_S`] before it.
pub fn end_moment(events: &[Value]) -> Option<&Value> {
    let anchor = events.iter().find(|e| {
        (s(e, "kind") == "full_auto" && s(e, "stage") == "give_up")
            || (s(e, "kind") == "sortie" && s(e, "event") == "end")
    })?;
    let ta = t_of(anchor);
    events
        .iter()
        .filter(|e| {
            let t = t_of(e);
            t <= ta && ta - t <= CASCADE_S
        })
        .find(|e| {
            s(e, "kind") == "stand_down"
                || s(e, "event") == "stand_down"
                || (s(e, "kind") == "recover" && s(e, "event") == "take_over")
        })
        .or(Some(anchor))
}

fn is_decision(e: &Value) -> bool {
    let kind = s(e, "kind");
    if kind == "said" {
        return matches!(s(e, "prio"), "Caution" | "Warning");
    }
    if kind == "callout" {
        return s(e, "id").starts_with("altimeter") || matches!(s(e, "sev"), "Caution" | "Warning");
    }
    DECISION_KINDS.contains(&kind) || DECISION_EVENTS.contains(&s(e, "event"))
}

fn label(e: &Value) -> String {
    let kind = s(e, "kind");
    let sub = [s(e, "event"), s(e, "stage"), s(e, "id")]
        .into_iter()
        .find(|x| !x.is_empty());
    match sub {
        Some(sub) => format!("{kind} {sub}"),
        None => kind.to_string(),
    }
}

fn scalar(v: &Value) -> Option<String> {
    match v {
        Value::Null => None,
        Value::Bool(b) => Some(b.to_string()),
        Value::Number(n) => n.as_f64().map(|f| {
            if f.fract() == 0.0 && f.abs() < 1e9 {
                format!("{f:.0}")
            } else if f.abs() >= 100.0 {
                format!("{f:.0}")
            } else {
                format!("{f:.2}")
            }
        }),
        Value::String(x) => {
            let x: String = x.chars().take(60).collect();
            Some(if x.contains(' ') {
                format!("\"{x}\"")
            } else {
                x
            })
        }
        Value::Array(a) if a.len() <= 4 => {
            let parts: Vec<String> = a.iter().filter_map(scalar).collect();
            Some(format!("[{}]", parts.join(",")))
        }
        _ => None,
    }
}

/// The record's numbers, compactly: scalars, one level of nesting
/// flattened, the bookkeeping fields dropped.
fn numbers(e: &Value) -> String {
    const SKIP: &[&str] = &["t", "sortie", "kind", "event", "stage", "id", "overheard"];
    const FIRST: &[&str] = &[
        "verdict", "why", "reason", "text", "vs", "dial", "was", "now", "want",
    ];
    let mut parts = Vec::new();
    if let Some(m) = e.as_object() {
        for (k, v) in m {
            if SKIP.contains(&k.as_str()) {
                continue;
            }
            match v {
                Value::Object(inner) => {
                    for (k2, v2) in inner {
                        if let Some(x) = scalar(v2) {
                            parts.push(format!("{k}.{k2}={x}"));
                        }
                    }
                }
                _ => {
                    if let Some(x) = scalar(v) {
                        parts.push(format!("{k}={x}"));
                    }
                }
            }
        }
    }
    // The verdict and the measured numbers first, so a long record's
    // truncation never hides them.
    parts.sort_by_key(|p| {
        let key = p.split('=').next().unwrap_or("");
        let leaf = key.rsplit('.').next().unwrap_or(key);
        FIRST.iter().position(|f| *f == leaf).unwrap_or(FIRST.len())
    });
    let mut line = parts.join(" ");
    if line.chars().count() > 200 {
        line = line.chars().take(197).collect::<String>() + "...";
    }
    line
}

/// One flag: when, and what disagrees.
#[derive(Debug, Clone, PartialEq)]
pub struct Flag {
    pub t: f64,
    pub text: String,
}

/// The altimeter change a record reports, as text, if it is one.
fn altimeter_change(e: &Value) -> Option<String> {
    let is_alt = s(e, "kind") == "altimeter"
        || s(e, "event") == "altimeter_set"
        || (s(e, "kind") == "callout" && s(e, "id").starts_with("altimeter"));
    if !is_alt {
        return None;
    }
    let was = e.get("was").and_then(Value::as_f64);
    let now = e.get("now").and_then(Value::as_f64);
    match (was, now) {
        (Some(w), Some(n)) if (w - n).abs() > 0.005 => Some(format!("{w:.2} -> {n:.2} inHg")),
        (Some(_), Some(_)) => None,
        _ => {
            let msg = s(e, "msg");
            (!msg.is_empty()).then(|| format!("\"{msg}\""))
        }
    }
}

/// Every [`RULES`] verdict checked against the `state` stream and the
/// altimeter: pure over the sortie's records.
pub fn contradictions(events: &[Value]) -> Vec<Flag> {
    let states: Vec<(f64, f64)> = events
        .iter()
        .filter(|e| s(e, "kind") == "state")
        .filter_map(|e| e.get("vs").and_then(Value::as_f64).map(|vs| (t_of(e), vs)))
        .collect();
    let mut flags = Vec::new();
    for e in events {
        let Some(rule) = RULES
            .iter()
            .find(|r| s(e, "kind") == r.kind && s(e, "event") == r.event)
        else {
            continue;
        };
        let Some(vs) = at(e, rule.vs).and_then(Value::as_f64) else {
            continue;
        };
        let t = t_of(e);
        let verdict = s(e, "verdict");
        let what = format!("{} {}", label(e), verdict).trim().to_string();
        if let Some(&(ts, svs)) = states
            .iter()
            .filter(|(ts, _)| (ts - t).abs() <= STATE_NEAR_S)
            .min_by(|a, b| (a.0 - t).abs().total_cmp(&(b.0 - t).abs()))
        {
            if (svs - vs).abs() > VS_DISAGREE_FPM {
                flags.push(Flag {
                    t,
                    text: format!(
                        "{what} measured vs {vs:+.0} fpm, but state at {:+.1} s says {svs:+.0} fpm",
                        ts - t
                    ),
                });
            }
        }
        if BENIGN_VERDICTS.contains(&verdict) {
            continue;
        }
        for a in events.iter().filter(|a| {
            let d = t_of(a) - t;
            (-ALTIMETER_BEFORE_S..=ALTIMETER_AFTER_S).contains(&d)
        }) {
            if let Some(change) = altimeter_change(a) {
                flags.push(Flag {
                    t,
                    text: format!(
                        "{what}: altimeter {} at {:+.1} s ({change})",
                        label(a),
                        t_of(a) - t
                    ),
                });
            }
        }
    }
    flags
}

/// The full page for one sortie. `profile` is the caller's lookup when the
/// file names none.
pub fn report(events: &[Value], profile: Option<&str>) -> String {
    let sum = summarise(events);
    let mut out = String::new();
    let profile = sum.profile.as_deref().or(profile).unwrap_or("unknown");
    out.push_str(&format!(
        "Sortie {} ({}Z), {} -> {}, airframe {profile}.\n",
        if sum.id.is_empty() { "?" } else { &sum.id },
        date_hm(sum.t0),
        or_q(&sum.origin),
        or_q(&sum.dest),
    ));
    out.push_str(&format!(
        "Outcome {}, {}{}.\n",
        sum.outcome.as_deref().unwrap_or("no end record"),
        match sum.pass {
            Some(true) => "PASS",
            Some(false) => "FAIL",
            None => "no verdict",
        },
        sum.reason
            .as_deref()
            .map(|r| format!(", reason {r}"))
            .unwrap_or_default(),
    ));
    if let Some(line) = &sum.end_line {
        out.push_str(&format!("  said: \"{line}\"\n"));
    }
    for (stage, _, why) in sum.failed() {
        out.push_str(&format!("  failed {stage}: {}\n", why.join("; ")));
    }

    if sum.pass != Some(true) {
        if let Some(end) = end_moment(events) {
            let te = t_of(end);
            out.push_str(&format!(
                "\nEnded at {}Z on {} {}\n",
                hms(te),
                label(end),
                numbers(end)
            ));
            out.push_str(&format!(
                "Decisions from {:.0} s before:\n",
                TIMELINE_BEFORE_S
            ));
            for e in events.iter().filter(|e| {
                let d = t_of(e) - te;
                (-TIMELINE_BEFORE_S..=TIMELINE_AFTER_S).contains(&d) && is_decision(e)
            }) {
                out.push_str(&format!(
                    "  {:+6.1} s  {:<32} {}\n",
                    t_of(e) - te,
                    label(e),
                    numbers(e)
                ));
            }
        }
    }

    let flags = contradictions(events);
    if !flags.is_empty() {
        out.push_str("\nContradictions:\n");
        for f in &flags {
            out.push_str(&format!("  {}Z  {}\n", hms(f.t), f.text));
        }
    }

    if !sum.stages.is_empty() {
        out.push_str("\nStages:\n");
        for (stage, pass, why) in &sum.stages {
            out.push_str(&format!(
                "  {:<14} {}{}\n",
                stage,
                if *pass { "pass" } else { "FAIL" },
                if why.is_empty() {
                    String::new()
                } else {
                    format!("  {}", why.join("; "))
                }
            ));
        }
    }
    out
}

/// One line per sortie, then the failures grouped.
pub fn airframe_report(profile: &str, sorties: &[Summary]) -> String {
    let mut out = format!("{} sortie(s) flown on {profile}.\n", sorties.len());
    let mut reasons: BTreeMap<String, usize> = BTreeMap::new();
    let mut stages: BTreeMap<String, (usize, BTreeMap<String, usize>)> = BTreeMap::new();
    for sum in sorties {
        let failed: Vec<&str> = sum.failed().map(|st| st.0.as_str()).collect();
        out.push_str(&format!(
            "  {:<24} {}Z  {}-{}  {:<10} {}{}\n",
            sum.id,
            date_hm(sum.t0),
            or_q(&sum.origin),
            or_q(&sum.dest),
            sum.outcome.as_deref().unwrap_or("no_end"),
            if failed.is_empty() {
                "no stage failed".to_string()
            } else {
                format!("failed {}", failed.join(","))
            },
            sum.reason
                .as_deref()
                .map(|r| format!("  reason {r}"))
                .unwrap_or_default(),
        ));
        if sum.pass != Some(true) {
            *reasons
                .entry(sum.reason.clone().unwrap_or_else(|| {
                    sum.outcome
                        .clone()
                        .unwrap_or_else(|| "no end record".into())
                }))
                .or_default() += 1;
        }
        for (stage, _, why) in sum.failed() {
            let entry = stages.entry(stage.clone()).or_default();
            entry.0 += 1;
            for w in why {
                *entry.1.entry(w.clone()).or_default() += 1;
            }
        }
    }
    if !reasons.is_empty() {
        out.push_str("Not passed, by reason:\n");
        for (r, n) in &reasons {
            out.push_str(&format!("  {n:>3}  {r}\n"));
        }
    }
    if !stages.is_empty() {
        out.push_str("Failed stages:\n");
        for (st, (n, whys)) in &stages {
            out.push_str(&format!("  {n:>3}  {st}\n"));
            for (w, k) in whys {
                out.push_str(&format!("         {k:>3}x {w}\n"));
            }
        }
    }
    out
}

/// The `profile_probe` records among blackbox lines, as (t, profile). Only
/// lines naming the kind are parsed: the blackbox runs to hundreds of MB.
pub fn probes<'a>(lines: impl Iterator<Item = &'a str>) -> Vec<(f64, String)> {
    let mut v: Vec<(f64, String)> = lines
        .filter(|l| l.contains("\"profile_probe\""))
        .filter_map(|l| serde_json::from_str::<Value>(l).ok())
        .filter(|v| s(v, "kind") == "profile_probe")
        .filter_map(|v| Some((t_of(&v), v.get("profile")?.as_str()?.to_string())))
        .collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

/// The profile loaded when a sortie began: the latest probe at or before it.
pub fn profile_at(probes: &[(f64, String)], t: f64) -> Option<&str> {
    probes
        .iter()
        .rev()
        .find(|(pt, _)| *pt <= t)
        .map(|(_, p)| p.as_str())
}

/// `YYYY-MM-DD` as Unix seconds at 00:00 UTC.
pub fn parse_date(d: &str) -> Option<f64> {
    let mut it = d.split('-').map(|x| x.parse::<i64>().ok());
    let (y, m, day) = (it.next()??, it.next()??, it.next()??);
    if !(1..=12).contains(&m) || !(1..=31).contains(&day) {
        return None;
    }
    // Days from civil (Howard Hinnant).
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + day - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    Some(((era * 146_097 + doe - 719_468) * 86_400) as f64)
}

fn date_hm(t: f64) -> String {
    let days = (t.max(0.0) as i64).div_euclid(86_400);
    // Civil from days (Howard Hinnant).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    let hm = hms(t);
    format!("{y:04}-{m:02}-{d:02} {}", &hm[..5])
}

fn hms(t: f64) -> String {
    let secs = t.max(0.0) as u64 % 86_400;
    format!(
        "{:02}:{:02}:{:02}",
        secs / 3600,
        (secs / 60) % 60,
        secs % 60
    )
}

fn or_q(x: &str) -> &str {
    if x.is_empty() {
        "?"
    } else {
        x
    }
}

// --- The disk: the only part that is not pure.

/// The sortie files on disk (`.jsonl`, gz skipped), with the Unix second
/// their name starts with.
fn sortie_files(dir: &Path) -> Vec<(u64, PathBuf)> {
    let mut v: Vec<(u64, PathBuf)> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "jsonl"))
        .filter_map(|p| {
            let stem = p.file_stem()?.to_str()?.to_string();
            let t = stem.split('-').next()?.parse::<u64>().ok()?;
            Some((t, p))
        })
        .collect();
    v.sort();
    v
}

fn resolve_sortie(arg: &str, dir: &Path) -> anyhow::Result<PathBuf> {
    let p = PathBuf::from(arg);
    if p.is_file() {
        return Ok(p);
    }
    let files = sortie_files(dir);
    if arg == "last" {
        return files
            .last()
            .map(|(_, p)| p.clone())
            .ok_or_else(|| anyhow::anyhow!("no sortie files in {}", dir.display()));
    }
    files
        .iter()
        .map(|(_, p)| p)
        .find(|p| {
            p.file_stem()
                .and_then(|s| s.to_str())
                .is_some_and(|s| s == arg || s.starts_with(&format!("{arg}-")))
        })
        .cloned()
        .ok_or_else(|| anyhow::anyhow!("no sortie {arg} in {}", dir.display()))
}

/// The blackbox's probes, streamed line by line.
fn blackbox_probes() -> Vec<(f64, String)> {
    use std::io::BufRead;
    let Ok(f) = std::fs::File::open(sim::paths::blackbox_file()) else {
        return Vec::new();
    };
    let mut lines = Vec::new();
    for l in std::io::BufReader::new(f)
        .split(b'\n')
        .map_while(Result::ok)
    {
        if l.windows(15).any(|w| w == b"\"profile_probe\"") {
            lines.push(String::from_utf8_lossy(&l).into_owned());
        }
    }
    probes(lines.iter().map(String::as_str))
}

/// `xfc debrief --sortie ...` / `--airframe ...`: the page, or None when the
/// arguments are neither (the plain debrief).
pub fn main_cmd(args: &[String]) -> anyhow::Result<Option<String>> {
    let flag = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1).cloned())
    };
    let dir = sim::paths::sorties_dir();
    if let Some(arg) = flag("--sortie") {
        let path = resolve_sortie(&arg, &dir)?;
        let text = std::fs::read_to_string(&path)
            .map_err(|e| anyhow::anyhow!("reading {}: {e}", path.display()))?;
        let events = parse(text.lines());
        let mut profile = None;
        if summarise(&events).profile.is_none() {
            let t0 = summarise(&events).t0;
            profile = profile_at(&blackbox_probes(), t0).map(String::from);
        }
        return Ok(Some(report(&events, profile.as_deref())));
    }
    if let Some(want) = flag("--airframe") {
        let since = match flag("--since") {
            Some(d) => Some(
                parse_date(&d).ok_or_else(|| anyhow::anyhow!("--since {d}: want YYYY-MM-DD"))?,
            ),
            None => None,
        };
        let probes = blackbox_probes();
        let mut kept = Vec::new();
        for (t, path) in sortie_files(&dir) {
            if since.is_some_and(|s| (t as f64) < s) {
                continue;
            }
            let Ok(text) = std::fs::read_to_string(&path) else {
                continue;
            };
            let events = parse(text.lines());
            let sum = summarise(&events);
            let profile = sum
                .profile
                .clone()
                .or_else(|| profile_at(&probes, sum.t0).map(String::from));
            if profile.as_deref() == Some(want.as_str()) {
                kept.push(sum);
            }
        }
        return Ok(Some(airframe_report(&want, &kept)));
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;

    // The A330 that gave up out of Gatwick (2026-10-14), in miniature: the
    // climb the state stream saw, the altimeter reset, the verdict that
    // read the reset as a dive, and the cascade into give_up.
    const A330: &[&str] = &[
        r#"{"kind":"sortie","event":"begin","origin":"EGKK","dest":"LEMG","sortie":"s1","t":1000.0}"#,
        r#"{"kind":"stage_result","stage":"line_up","pass":false,"why":["3.8 m off the line"],"sortie":"s1","t":1100.0}"#,
        r#"{"kind":"stage_result","stage":"takeoff","pass":true,"why":[],"sortie":"s1","t":1150.0}"#,
        r#"{"kind":"autoflight","event":"thrust_taken","cmd":"sim/autopilot/autothrottle_off","try":1,"sortie":"s1","t":1160.0}"#,
        r#"{"kind":"said","prio":"Advisory","text":"Traffic, 11 o'clock.","sortie":"s1","t":1180.0}"#,
        r#"{"kind":"state","vs":2136.4,"alt":6120.0,"sortie":"s1","t":1199.4}"#,
        r#"{"kind":"callout","id":"altimeter_transition","msg":"Altimeter reset, two niner niner two.","sortie":"s1","t":1199.5}"#,
        r#"{"kind":"ap_pitch_authority","event":"verdict","verdict":"no_authority","why":"vs_away_from_dial","numbers":{"vs":-1681.0,"dial":2000.0},"sortie":"s1","t":1200.0}"#,
        r#"{"kind":"procedure","event":"stand_down","sortie":"s1","t":1200.0}"#,
        r#"{"kind":"recover","event":"take_over","why":"procedure_no_pitch_authority","sortie":"s1","t":1200.0}"#,
        r#"{"kind":"altimeter","event":"copilot","was":30.319614,"now":29.92,"took":true,"sortie":"s1","t":1200.2}"#,
        r#"{"kind":"said","prio":"Warning","text":"The autopilot has no pitch control.","sortie":"s1","t":1200.2}"#,
        r#"{"kind":"full_auto","stage":"give_up","detail":{"reason":"enroute_stalled"},"sortie":"s1","t":1201.0}"#,
        r#"{"kind":"sortie","event":"end","outcome":"gave_up","reason":"enroute_stalled","line":"The enroute leg never completed. Your aircraft.","sortie":"s1","t":1201.0}"#,
        r#"{"kind":"sortie_result","outcome":"gave_up","pass":false,"failed":["line_up"],"sortie":"s1","t":1201.0}"#,
    ];

    #[test]
    fn a_give_up_is_explained_with_its_contradictions() {
        let ev = parse(A330.iter().copied());
        let page = report(&ev, Some("airbus_a330_300"));
        assert!(
            page.contains("EGKK -> LEMG, airframe airbus_a330_300"),
            "{page}"
        );
        assert!(
            page.contains("Outcome gave_up, FAIL, reason enroute_stalled"),
            "{page}"
        );
        assert!(
            page.contains("failed line_up: 3.8 m off the line"),
            "{page}"
        );
        // The moment is the stand-down, not the give_up a second later.
        assert!(
            page.contains("Ended at") && page.contains("on procedure stand_down"),
            "{page}"
        );
        assert!(
            page.contains("-0.5 s  callout altimeter_transition"),
            "{page}"
        );
        assert!(page.contains("ap_pitch_authority verdict"), "{page}");
        assert!(page.contains("+0.2 s  altimeter copilot"), "{page}");
        // Advisory chatter is not a decision; a warning is.
        assert!(!page.contains("Traffic, 11"), "{page}");
        assert!(page.contains("no pitch control"), "{page}");
        // A thrust press 40 s before is outside the window.
        assert!(!page.contains("thrust_taken"), "{page}");
        assert!(
            page.contains("measured vs -1681 fpm, but state at -0.6 s says +2136 fpm"),
            "{page}"
        );
        assert!(
            page.contains("altimeter altimeter copilot at +0.2 s (30.32 -> 29.92 inHg)"),
            "{page}"
        );
        assert!(
            page.contains("altimeter callout altimeter_transition at -0.5 s"),
            "{page}"
        );
        assert!(
            page.contains("line_up        FAIL  3.8 m off the line"),
            "{page}"
        );
        assert!(page.contains("takeoff        pass"), "{page}");
    }

    #[test]
    fn a_confirmed_verdict_that_agrees_is_not_flagged() {
        let ev = parse(
            [
                r#"{"kind":"state","vs":2000.0,"t":10.0}"#,
                r#"{"kind":"altimeter","was":30.32,"now":29.92,"t":10.5}"#,
                r#"{"kind":"ap_pitch_authority","event":"verdict","verdict":"confirmed","numbers":{"vs":2151.0},"t":11.0}"#,
                r#"{"kind":"procedure","event":"climb_watch","verdict":"TakeOver(NotFollowing)","vs":7383.0,"t":30.0}"#,
                r#"{"kind":"state","vs":7300.0,"t":31.0}"#,
            ]
            .into_iter(),
        );
        assert!(contradictions(&ev).is_empty(), "{:?}", contradictions(&ev));
        // A climb_watch that disagrees with the state is flagged.
        let ev = parse(
            [
                r#"{"kind":"procedure","event":"climb_watch","verdict":"TakeOver(NotFollowing)","vs":7383.0,"t":30.0}"#,
                r#"{"kind":"state","vs":-900.0,"t":32.0}"#,
                r#"{"kind":"state","vs":5000.0,"t":40.0}"#,
            ]
            .into_iter(),
        );
        let f = contradictions(&ev);
        assert_eq!(f.len(), 1, "{f:?}");
        assert!(
            f[0].text.contains("+7383") && f[0].text.contains("-900"),
            "{f:?}"
        );
    }

    #[test]
    fn a_pass_has_no_give_up_section() {
        let ev = parse(
            [
                r#"{"kind":"sortie","event":"begin","origin":"LEMG","dest":"LEMG","sortie":"p","t":0.0}"#,
                r#"{"kind":"sortie","event":"end","outcome":"landed","sortie":"p","t":900.0}"#,
                r#"{"kind":"sortie_result","outcome":"landed","pass":true,"sortie":"p","t":900.0}"#,
            ]
            .into_iter(),
        );
        let page = report(&ev, None);
        assert!(page.contains("airframe unknown"), "{page}");
        assert!(page.contains("Outcome landed, PASS"), "{page}");
        assert!(!page.contains("Ended at"), "{page}");
    }

    #[test]
    fn the_profile_is_the_probe_before_the_sortie() {
        let lines = [
            r#"{"kind":"profile_probe","profile":"boeing_737_800","t":100.0}"#,
            r#"{"kind":"state","note":"profile_probe in text only","t":150.0}"#,
            r#"{"kind":"profile_probe","profile":"airbus_a330_300","t":200.0}"#,
        ];
        let p = probes(lines.into_iter());
        assert_eq!(p.len(), 2);
        assert_eq!(profile_at(&p, 150.0), Some("boeing_737_800"));
        assert_eq!(profile_at(&p, 250.0), Some("airbus_a330_300"));
        assert_eq!(profile_at(&p, 50.0), None);
    }

    #[test]
    fn dates_are_utc_days() {
        assert_eq!(parse_date("2026-10-14"), Some(1_791_936_000.0));
        assert_eq!(parse_date("1970-01-01"), Some(0.0));
        assert_eq!(parse_date("2026-13-01"), None);
        assert_eq!(date_hm(1_791_990_505.7), "2026-10-14 15:08");
    }

    #[test]
    fn failures_group_by_reason_and_stage() {
        let a = summarise(&parse(A330.iter().copied()));
        let mut b = a.clone();
        b.id = "s2".into();
        b.stages = vec![("line_up".into(), false, vec!["5 degrees off".into()])];
        let page = airframe_report("airbus_a330_300", &[a, b]);
        assert!(
            page.starts_with("2 sortie(s) flown on airbus_a330_300."),
            "{page}"
        );
        assert!(page.contains("    2  enroute_stalled"), "{page}");
        assert!(page.contains("    2  line_up"), "{page}");
        assert!(page.contains("1x 3.8 m off the line"), "{page}");
        assert!(page.contains("1x 5 degrees off"), "{page}");
    }
}
