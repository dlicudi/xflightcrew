//! The flight-test card: every change that shipped without a sortie to prove
//! it, one row each, with the blackbox evidence that retires it.
//!
//! More than twenty fixes went out between 2026-09-21 and 2026-09-26 on
//! offline tests alone, recorded as "unflown" in CLAUDE.md and the session
//! notes, each naming its own events. Nobody holds twenty checklists in their
//! head after a sortie, so none were ticked off. This is the one card: the
//! items live HERE, as a static table (name, what to fly, the evidence, the
//! pass criterion and, where the criterion is a number, the predicate), and
//! `docs/flight-test-card.md` is checked against it by a test so the two
//! cannot drift. `xvl debrief` prints the verdicts over the whole file it is
//! given.
//!
//! Every event and field named here was read off the source that records it
//! (`blackbox::record("<kind>", ...)`), not off a note. An item only counts
//! evidence recorded at or after the commit that shipped it (`since`), and an
//! item for one airframe only counts that airframe's records (the latest
//! `flight_meta` before each record). Pure over the parsed lines.

use serde_json::Value;

/// Which kind of sortie retires the item: the card is grouped by what one
/// flight can cover, so a pilot picks the sortie and reads the rows.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sortie {
    /// A 737 (or A330) full-auto IFR flight: SID, STAR, ILS, plan audit,
    /// load sheet, takeoff config, the arrival and the landing.
    Airliner,
    /// A 172 (or SF50) circuit or short full-auto hop.
    Circuit,
    /// An S-76 route.
    Helicopter,
    /// On the ramp only: starts, pushback, cabin, hand-over words.
    Ramp,
    /// The pilot flying, the crew talking: rapport, tempo, say again.
    HandFlown,
}

impl Sortie {
    pub const ALL: [Sortie; 5] = [
        Sortie::Airliner,
        Sortie::Circuit,
        Sortie::Helicopter,
        Sortie::Ramp,
        Sortie::HandFlown,
    ];

    pub fn title(self) -> &'static str {
        match self {
            Sortie::Airliner => "Airliner full-auto IFR flight (737, A330)",
            Sortie::Circuit => "Light aircraft circuit or hop (172, SF50, Evolution, SR22)",
            Sortie::Helicopter => "Helicopter route (S-76)",
            Sortie::Ramp => "Ramp session (start, pushback, cabin, hand-over words)",
            Sortie::HandFlown => "Hand-flown session (rapport, tempo, say again)",
        }
    }
}

/// What the log says about one item.
#[derive(Clone, Debug, PartialEq)]
pub enum Verdict {
    Verified(String),
    Failed(String),
    NotExercised,
}

/// One row of the card. The text columns are the doc's, read by the test
/// that holds `docs/flight-test-card.md` to this table.
#[cfg_attr(not(test), allow(dead_code))]
pub struct Item {
    pub id: &'static str,
    pub sortie: Sortie,
    /// The change, in a line.
    pub what: &'static str,
    /// Date and commit.
    pub shipped: &'static str,
    /// Unix time of that commit: evidence before it is a flight of the old code.
    pub since: f64,
    /// Only this airframe's records (a profile id prefix), or any.
    pub airframe: Option<&'static str>,
    /// Airframe, field, action.
    pub fly: &'static str,
    /// Blackbox kind, event, fields.
    pub evidence: &'static str,
    /// The pass criterion, in words.
    pub pass: &'static str,
    /// The predicate, where the criterion is a number in the log. `None` is a
    /// row the pilot ticks by hand.
    pub check: Option<fn(&Log) -> Verdict>,
}

/// One blackbox record, with its name and the airframe it was flown on.
#[derive(Clone, Copy)]
pub struct Rec<'a> {
    pub v: &'a Value,
    pub t: f64,
    pub kind: &'a str,
    /// The record's event name: `event`, full auto's `stage`, the station's
    /// `outcome`, the flight model's `ev`, a callout's `id`.
    pub name: &'a str,
    pub ac: Option<&'a str>,
}

pub struct Log<'a> {
    pub recs: Vec<Rec<'a>>,
}

impl<'a> Log<'a> {
    pub fn new(events: &[&'a Value]) -> Self {
        let mut ac: Option<&'a str> = None;
        let mut recs = Vec::with_capacity(events.len());
        for v in events {
            let kind = v.get("kind").and_then(Value::as_str).unwrap_or("");
            if kind == "flight_meta" {
                ac = v.get("aircraft").and_then(Value::as_str);
            }
            let name = ["event", "stage", "outcome", "ev", "id"]
                .iter()
                .find_map(|k| v.get(*k).and_then(Value::as_str))
                .unwrap_or("");
            recs.push(Rec {
                v,
                t: v.get("t").and_then(Value::as_f64).unwrap_or(0.0),
                kind,
                name,
                ac,
            });
        }
        Log { recs }
    }

    fn window(&self, since: f64, airframe: Option<&str>) -> Log<'a> {
        Log {
            recs: self
                .recs
                .iter()
                .filter(|r| r.t >= since)
                .filter(|r| airframe.is_none_or(|a| r.ac.is_some_and(|x| x.starts_with(a))))
                .copied()
                .collect(),
        }
    }

    fn of<'s>(&'s self, kind: &'s str, name: &'s str) -> impl Iterator<Item = &'s Rec<'a>> + 's {
        self.recs
            .iter()
            .filter(move |r| r.kind == kind && r.name == name)
    }

    /// The first record of `kind`/`name` after `t`, within `secs`.
    fn next(&self, t: f64, kind: &str, name: &str, secs: f64) -> Option<&Rec<'a>> {
        self.recs
            .iter()
            .find(|r| r.t > t && r.t <= t + secs && r.kind == kind && r.name == name)
    }
}

// --- field readers: top level first, then full auto's `detail`.

fn field<'v>(v: &'v Value, k: &str) -> Option<&'v Value> {
    v.get(k).filter(|x| !x.is_null()).or_else(|| {
        v.get("detail")
            .and_then(|d| d.get(k))
            .filter(|x| !x.is_null())
    })
}
fn num(v: &Value, k: &str) -> Option<f64> {
    field(v, k).and_then(Value::as_f64)
}
fn text<'v>(v: &'v Value, k: &str) -> Option<&'v str> {
    field(v, k).and_then(Value::as_str)
}
fn flag(v: &Value, k: &str) -> Option<bool> {
    field(v, k).and_then(Value::as_bool)
}

/// Unix seconds to HH:MM (UTC), the way the pilot reads the debrief.
fn hm(t: f64) -> String {
    let s = t.max(0.0) as u64 % 86_400;
    format!("{:02}:{:02}", s / 3600, (s / 60) % 60)
}

fn wrap180(d: f64) -> f64 {
    (d + 540.0).rem_euclid(360.0) - 180.0
}

/// Consecutive ticks of one stream, split on a gap.
fn runs<'s, 'a>(log: &'s Log<'a>, kind: &'s str, name: &'s str, gap: f64) -> Vec<Vec<&'s Rec<'a>>> {
    let mut out: Vec<Vec<&Rec>> = Vec::new();
    for r in log.of(kind, name) {
        let fresh = out
            .last()
            .is_none_or(|run| run.last().is_none_or(|x| r.t - x.t > gap));
        if fresh {
            out.push(vec![r]);
        } else if let Some(run) = out.last_mut() {
            run.push(r);
        }
    }
    out
}

fn verdict(fails: Vec<String>, oks: Vec<String>) -> Verdict {
    if !fails.is_empty() {
        Verdict::Failed(fails.join("; "))
    } else if oks.is_empty() {
        Verdict::NotExercised
    } else {
        Verdict::Verified(oks.join("; "))
    }
}

/// The scorecard's own stage verdicts, re-derived from the ticks.
fn stages(log: &Log, stage: &str) -> Vec<Value> {
    crate::scorecard::score_all(log.recs.iter().map(|r| r.v))
        .into_iter()
        .filter(|v| v.get("kind").and_then(Value::as_str) == Some("stage_result"))
        .filter(|v| v.get("stage").and_then(Value::as_str) == Some(stage))
        .collect()
}

// ------------------------------------------------------------ the checks

fn c_b738_speedbrake_down(log: &Log) -> Verdict {
    let sets: Vec<&Rec> = log
        .of("autotakeoff", "takeoff_config")
        .filter(|r| r.v["set"].as_array().is_some_and(|a| !a.is_empty()))
        .collect();
    let armed = sets
        .iter()
        .filter(|r| r.v["set"].to_string().contains("spoilers armed"))
        .count();
    if sets.is_empty() {
        Verdict::NotExercised
    } else if armed > 0 {
        Verdict::Failed(format!(
            "spoilers armed for the roll in {armed} of {} configs",
            sets.len()
        ))
    } else {
        Verdict::Verified(format!(
            "{} takeoff configs, none armed the spoilers ({})",
            sets.len(),
            sets[0].v["set"]
        ))
    }
}

fn c_jet_climb_out(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    let mut open: Option<(f64, f64, Vec<f64>, bool)> = None; // t, accel, steps, handoff flaps
    fn close(
        o: Option<(f64, f64, Vec<f64>, bool)>,
        fails: &mut Vec<String>,
        oks: &mut Vec<String>,
    ) {
        if let Some((t, accel, steps, handoff)) = o {
            if steps.is_empty() && !handoff {
                return;
            }
            let low: Vec<String> = steps
                .iter()
                .filter(|a| **a < accel - 25.0)
                .map(|a| format!("{a:.0}"))
                .collect();
            let list: Vec<String> = steps.iter().map(|a| format!("{a:.0}")).collect();
            if !low.is_empty() || handoff {
                fails.push(format!(
                    "{}: flap steps at {} ft against {accel:.0}{}",
                    hm(t),
                    list.join(", "),
                    if handoff {
                        ", flaps_handoff one-shot"
                    } else {
                        ""
                    }
                ));
            } else {
                oks.push(format!(
                    "{}: flap steps at {} ft against {accel:.0}",
                    hm(t),
                    list.join(", ")
                ));
            }
        }
    }
    for r in log.recs.iter().filter(|r| r.kind == "autotakeoff") {
        match r.name {
            "climb_out" => {
                close(open.take(), &mut fails, &mut oks);
                open = Some((
                    r.t,
                    num(r.v, "accel_agl_ft").unwrap_or(1000.0),
                    Vec::new(),
                    false,
                ));
            }
            "flap_step" => {
                if let (Some(o), Some(a)) = (open.as_mut(), num(r.v, "agl_ft")) {
                    o.2.push(a);
                }
            }
            "flaps_handoff" => {
                if let Some(o) = open.as_mut() {
                    o.3 = true;
                }
            }
            "handoff" | "reject" => close(open.take(), &mut fails, &mut oks),
            _ => {}
        }
    }
    close(open.take(), &mut fails, &mut oks);
    verdict(fails, oks)
}

fn c_reject_to_a_stop(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for rej in log.of("autotakeoff", "reject") {
        let ticks: Vec<&Rec> = log
            .of("autotakeoff", "reject_rollout")
            .filter(|r| r.t >= rej.t && r.t <= rej.t + 120.0)
            .collect();
        let Some(last) = ticks.last() else {
            fails.push(format!(
                "{}: a reject with no reject_rollout record",
                hm(rej.t)
            ));
            continue;
        };
        let max_x = ticks
            .iter()
            .filter_map(|r| num(r.v, "xtk_m"))
            .fold(0.0f64, |m, x| m.max(x.abs()));
        let gs = num(last.v, "gs").unwrap_or(f64::NAN);
        let line = format!(
            "{}: last tick {gs:.0} kt, {max_x:.1} m off at worst",
            hm(rej.t)
        );
        if gs <= 25.0 && max_x <= 15.0 {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_roll_centreline(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for run in runs(log, "autotakeoff", "roll", 20.0) {
        let max_x = run
            .iter()
            .filter_map(|r| num(r.v, "xtk_m"))
            .fold(0.0f64, |m, x| m.max(x.abs()));
        let trim = run
            .iter()
            .filter(|r| num(r.v, "gs").is_some_and(|g| g > 90.0))
            .filter_map(|r| num(r.v, "trim"))
            .fold(0.0f64, |m, x| m.max(x.abs()));
        let line = format!(
            "{}: {max_x:.1} m off at worst, trim {trim:.2} above 90 kt",
            hm(run[0].t)
        );
        if max_x <= crate::scorecard::ROLL_MAX_XTK_M {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

/// Centreline crossings above 40 kt (half a metre of dead band).
fn crossings(run: &[&Rec]) -> usize {
    let mut prev = None;
    let mut n = 0;
    for x in run
        .iter()
        .filter(|r| num(r.v, "gs").is_some_and(|g| g > 40.0))
        .filter_map(|r| num(r.v, "xtk_m"))
        .filter(|x| x.abs() >= 0.5)
    {
        let side = x > 0.0;
        if prev.is_some_and(|p| p != side) {
            n += 1;
        }
        prev = Some(side);
    }
    n
}

fn c_rollout_law(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for run in runs(log, "autoland", "rollout", 20.0) {
        let first = num(run[0].v, "xtk_m").unwrap_or(0.0).abs();
        let max_x = run
            .iter()
            .filter_map(|r| num(r.v, "xtk_m"))
            .fold(0.0f64, |m, x| m.max(x.abs()));
        let n = crossings(&run);
        let line = format!(
            "{}: {first:.1} m off at the first tick, grew {:.1} m, {n} crossing(s)",
            hm(run[0].t),
            max_x - first
        );
        if max_x - first <= 2.0 && n <= 1 {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_touchdown_centreline(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for run in runs(log, "autoland", "rollout", 20.0) {
        let x = num(run[0].v, "xtk_m").unwrap_or(0.0);
        let line = format!(
            "{}: {:.1} m {} at the wheels",
            hm(run[0].t),
            x.abs(),
            if x < 0.0 { "left" } else { "right" }
        );
        if x.abs() <= crate::scorecard::ROLLOUT_MAX_XTK_M {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

/// The servos at the wheels. `state` `in.ap` is NOT evidence: it reads
/// `autopilot_on`, which stays 1 with the servos out (every landing on
/// 2026-09-26 read true through the arrest and the rollout), so the servos
/// are judged from the loop's own records: the arrest took them at the
/// roundout gate and did not hand them back, or `ap_disconnected` at the
/// rollout latch.
fn c_ap_off_at_wheels(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for run in runs(log, "autoland", "rollout", 20.0) {
        let t0 = run[0].t;
        let disc = log.next(t0 - 5.0, "autoland", "ap_disconnected", 30.0);
        // The last arrest record inside the final minute before the wheels.
        let arrest = log
            .of("autoland", "arrest_hands")
            .filter(|r| r.t <= t0 && r.t >= t0 - 60.0)
            .last();
        let taken = arrest
            .is_some_and(|a| flag(a.v, "taken") == Some(true) && flag(a.v, "ok") == Some(true));
        match disc {
            Some(d) if flag(d.v, "ok") == Some(false) => {
                fails.push(format!("{}: ap_disconnected ok false", hm(t0)))
            }
            Some(_) => oks.push(format!("{}: disconnected at the rollout", hm(t0))),
            None if taken => oks.push(format!(
                "{}: servos out since the arrest at {:.0} ft",
                hm(t0),
                arrest.and_then(|a| num(a.v, "radio")).unwrap_or(0.0)
            )),
            None => fails.push(format!("{}: no record of the servos coming off", hm(t0))),
        }
    }
    verdict(fails, oks)
}

fn c_touchdown_sink(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for s in stages(log, "landing") {
        let sink = num(&s, "sink_fpm").unwrap_or(f64::NAN);
        let line = format!("{}: {sink:.0} fpm", hm(num(&s, "t").unwrap_or(0.0)));
        if sink <= crate::scorecard::TOUCHDOWN_MAX_SINK_FPM {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_altimeter_took(log: &Log) -> Verdict {
    let sets: Vec<&Rec> = log
        .recs
        .iter()
        .filter(|r| {
            matches!(
                (r.kind, r.name),
                ("full_auto", "altimeter_set")
                    | ("procedure", "altimeter_set")
                    | ("callout", "altimeter_reset")
            )
        })
        .collect();
    let bad: Vec<String> = sets
        .iter()
        .filter(|r| flag(r.v, "took") == Some(false))
        .map(|r| {
            format!(
                "{}: wanted {:.2}, have {:.2}",
                hm(r.t),
                num(r.v, "want").unwrap_or(0.0),
                num(r.v, "now").or(num(r.v, "inhg")).unwrap_or(0.0)
            )
        })
        .collect();
    if sets.is_empty() {
        Verdict::NotExercised
    } else if !bad.is_empty() {
        Verdict::Failed(format!(
            "{} of {} did not take: {}",
            bad.len(),
            sets.len(),
            bad.join("; ")
        ))
    } else {
        Verdict::Verified(format!("{} settings, every one took", sets.len()))
    }
}

/// What ended a pattern or a reversal: done, or taken away from it.
fn pattern_outcome<'s, 'a>(log: &'s Log<'a>, t: f64) -> Option<&'s Rec<'a>> {
    log.recs.iter().find(|r| {
        r.t > t
            && r.t <= t + 900.0
            && matches!(
                (r.kind, r.name),
                (
                    "autoland",
                    "reversal_done"
                        | "reversal"
                        | "going_around"
                        | "give_up"
                        | "late_go_around"
                        | "self_pick"
                ) | ("flight", "pilot_took_over" | "touchdown")
            )
            || (r.t > t && r.kind == "flight" && r.name == "mission" && r.v["from"] == "autoland")
    })
}

fn c_reversal(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    let mut interrupted = 0;
    for rev in log.of("autoland", "reversal") {
        match pattern_outcome(log, rev.t) {
            Some(d) if d.name == "reversal_done" => {
                let x = num(d.v, "xtk_m").unwrap_or(f64::NAN);
                let line = format!(
                    "{}: done {:.0} m off at {:.1} nm",
                    hm(rev.t),
                    x.abs(),
                    num(d.v, "dist").unwrap_or(0.0)
                );
                if x.abs() <= 100.0 {
                    oks.push(line);
                } else {
                    fails.push(line);
                }
            }
            Some(d) if matches!(d.name, "going_around" | "give_up" | "touchdown") => fails.push(
                format!("{}: ended in {} before the final", hm(rev.t), d.name),
            ),
            _ => interrupted += 1,
        }
    }
    if interrupted > 0 && (!oks.is_empty() || !fails.is_empty()) {
        oks.push(format!(
            "{interrupted} more interrupted by the pilot or a re-arm"
        ));
    }
    verdict(fails, oks)
}

fn c_pattern_height(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for ext in log.of("autoland", "downwind_extended") {
        let above = num(ext.v, "above_ft").unwrap_or(0.0);
        match pattern_outcome(log, ext.t) {
            Some(d) if d.name == "reversal_done" => oks.push(format!(
                "{}: {above:.0} ft high, pattern completed",
                hm(ext.t)
            )),
            Some(d) if matches!(d.name, "going_around" | "give_up") => fails.push(format!(
                "{}: {above:.0} ft high, then {}",
                hm(ext.t),
                d.name
            )),
            _ => {}
        }
    }
    verdict(fails, oks)
}

fn c_descent_drag(log: &Log) -> Verdict {
    let evs: Vec<&Rec> = log.of("procedure", "speedbrake").collect();
    let outs = evs
        .iter()
        .filter(|r| flag(r.v, "out") == Some(true))
        .count();
    if outs == 0 {
        return Verdict::NotExercised;
    }
    // Never out with the final's gear or landing flap (EGKK 2026-09-27): a
    // MAP final configured while the brake was out. The stow lands in the
    // same tick as the configuring write, so it is allowed two seconds.
    let mut out = false;
    let mut marks: Vec<&Rec> = evs.clone();
    marks.extend(log.of("procedure", "map_config"));
    marks.sort_by(|a, b| a.t.total_cmp(&b.t));
    for r in marks {
        if r.name == "speedbrake" {
            out = flag(r.v, "out") == Some(true);
        } else if out
            && (flag(r.v, "gear") == Some(true) || num(r.v, "flap").unwrap_or(0.0) > 0.13)
            && !evs
                .iter()
                .any(|s| flag(s.v, "out") == Some(false) && s.t >= r.t && s.t <= r.t + 2.0)
        {
            return Verdict::Failed(format!(
                "speedbrake out at {} while configuring (flap {:.2})",
                hm(r.t),
                num(r.v, "flap").unwrap_or(0.0)
            ));
        }
    }
    let last = evs.last().unwrap();
    if flag(last.v, "out") == Some(true) {
        return Verdict::Failed(format!(
            "{outs} extensions, the last at {} never stowed",
            hm(last.t)
        ));
    }
    Verdict::Verified(format!(
        "{outs} extensions, all stowed; last at {} at {:.0} kt against {:.0}",
        hm(last.t),
        num(last.v, "ias").unwrap_or(0.0),
        num(last.v, "target").unwrap_or(0.0)
    ))
}

fn c_descent_path(log: &Log) -> Verdict {
    // A follower descent is a leg tick whose wanted altitude is well below it.
    let descents = log
        .of("procedure", "leg")
        .filter(|r| matches!((num(r.v, "want_alt"), num(r.v, "alt_ft")), (Some(w), Some(a)) if w < a - 3000.0))
        .count();
    if descents == 0 {
        return Verdict::NotExercised;
    }
    let outs = log
        .of("procedure", "speedbrake")
        .filter(|r| flag(r.v, "out") == Some(true))
        .count();
    let line = format!("{outs} speedbrake extension(s) over the descent");
    if outs <= 2 {
        Verdict::Verified(line)
    } else {
        Verdict::Failed(line)
    }
}

fn c_inbound_retune(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("full_auto", "inbound_retune") {
        let khz = num(r.v, "khz");
        match log.next(r.t - 1.0, "full_auto", "inbound_call", 120.0) {
            Some(c) if num(c.v, "khz") == khz => {
                oks.push(format!("{}: called on {:.0}", hm(r.t), khz.unwrap_or(0.0)))
            }
            Some(c) => fails.push(format!(
                "{}: tuned {:.0}, called on {:.0}",
                hm(r.t),
                khz.unwrap_or(0.0),
                num(c.v, "khz").unwrap_or(0.0)
            )),
            None => fails.push(format!(
                "{}: retuned, no inbound call within 2 min",
                hm(r.t)
            )),
        }
    }
    verdict(fails, oks)
}

/// The STAR's windows the audit applied, by fix (the latest arm wins).
fn star_windows(log: &Log) -> Vec<(String, Option<f64>, Option<f64>, Option<f64>)> {
    let mut out: Vec<(String, Option<f64>, Option<f64>, Option<f64>)> = Vec::new();
    for r in log.of("plan_audit", "star_constraints") {
        if num(r.v, "changed_legs").unwrap_or(0.0) <= 0.0 {
            continue;
        }
        out.clear();
        for c in r.v["constraints"].as_array().into_iter().flatten() {
            if let Some(fix) = c["fix"].as_str() {
                out.push((
                    fix.to_string(),
                    c["min_ft"].as_f64(),
                    c["max_ft"].as_f64(),
                    c["max_kts"].as_f64(),
                ));
            }
        }
    }
    out
}

/// The follower's last tick toward a fix, inside 3 nm of it.
fn crossing<'s, 'a>(log: &'s Log<'a>, fix: &str) -> Option<&'s Rec<'a>> {
    log.of("procedure", "leg")
        .filter(|r| text(r.v, "fix") == Some(fix) && num(r.v, "dist_nm").is_some_and(|d| d < 3.0))
        .last()
}

fn c_star_windows(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for (fix, min, max, _) in star_windows(log) {
        if min.is_none() && max.is_none() {
            continue;
        }
        let Some(x) = crossing(log, &fix) else {
            continue;
        };
        let alt = num(x.v, "alt_ft").unwrap_or(f64::NAN);
        let line = format!(
            "{fix} at {alt:.0} ({}..{})",
            min.map_or("".into(), |m| format!("{m:.0}")),
            max.map_or("".into(), |m| format!("{m:.0}"))
        );
        if min.is_none_or(|m| alt >= m - 300.0) && max.is_none_or(|m| alt <= m + 300.0) {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_star_speeds(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for (fix, _, _, kts) in star_windows(log) {
        let Some(kts) = kts else { continue };
        let Some(x) = crossing(log, &fix) else {
            continue;
        };
        let ias = log
            .recs
            .iter()
            .filter(|r| r.kind == "state" && (r.t - x.t).abs() <= 3.0)
            .find_map(|r| num(r.v, "ias"));
        let Some(ias) = ias else { continue };
        let line = format!("{fix} at {ias:.0} kt (limit {kts:.0})");
        if ias <= kts + 10.0 {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

/// The runway the aircraft actually landed on or turned off: the vacate's.
fn landed_runway_after(log: &Log, t: f64) -> Option<String> {
    log.recs
        .iter()
        .find(|r| r.t > t && r.kind == "taxi_in" && r.name == "vacating")
        .and_then(|r| text(r.v, "rwy").map(str::to_string))
}

fn c_own_star(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    let chosen: Vec<&Rec> = log.of("plan_audit", "star_chosen").collect();
    // One verdict per arrival: the last choice before each landing.
    let mut judged_until = 0.0;
    for (i, c) in chosen.iter().enumerate() {
        if c.t < judged_until {
            continue;
        }
        let Some(landed) = landed_runway_after(log, c.t) else {
            continue;
        };
        let vac_t = log
            .recs
            .iter()
            .find(|r| r.t > c.t && r.kind == "taxi_in" && r.name == "vacating")
            .map(|r| r.t)
            .unwrap_or(f64::MAX);
        if chosen.get(i + 1).is_some_and(|n| n.t < vac_t) {
            continue;
        }
        judged_until = vac_t;
        let rwy = text(c.v, "runway").unwrap_or("");
        let line = format!(
            "{}: chose {} for {rwy} (planned {} for {}), landed {landed}",
            hm(c.t),
            text(c.v, "star").unwrap_or("?"),
            text(c.v, "planned_star").unwrap_or("?"),
            text(c.v, "planned_runway").unwrap_or("?")
        );
        if landed == rwy {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_own_sid(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("full_auto", "departure_rechosen") {
        let line = text(r.v, "line").unwrap_or("").to_string();
        if log.next(r.t, "autotakeoff", "liftoff", 1800.0).is_some() {
            oks.push(format!("{}: {line}", hm(r.t)));
        } else {
            fails.push(format!("{}: {line}, no liftoff followed", hm(r.t)));
        }
    }
    verdict(fails, oks)
}

fn c_plan_audit(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for b in log
        .of("sortie", "begin")
        .filter(|r| text(r.v, "dest").is_some())
    {
        let audit = log
            .recs
            .iter()
            .filter(|r| r.kind == "plan_audit" && r.v.get("findings").is_some())
            .filter(|r| r.t <= b.t + 5.0 && r.t >= b.t - 300.0)
            .last();
        let route = format!(
            "{}-{}",
            text(b.v, "origin").unwrap_or("?"),
            text(b.v, "dest").unwrap_or("?")
        );
        match audit {
            Some(a) => {
                let f = a.v["findings"].as_array().map(|x| x.len()).unwrap_or(0);
                let refused = a.v["findings"].to_string().contains("\"Refuse");
                let line = format!("{} {route}: audited, {f} finding(s)", hm(b.t));
                if refused {
                    fails.push(format!("{line}, a refusal and the mission still armed"));
                } else {
                    oks.push(line);
                }
            }
            None if flag(b.v, "resume") != Some(true) => fails.push(format!(
                "{} {route}: armed with no plan_audit record",
                hm(b.t)
            )),
            None => {}
        }
    }
    verdict(fails, oks)
}

fn c_load_sheet(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("loading", "load_aircraft") {
        let close = |want: Option<f64>, got: Option<f64>| match (want, got) {
            (Some(w), Some(g)) => (w - g).abs() <= (0.02 * w).max(5.0),
            _ => true,
        };
        let fuel_ok = close(num(r.v, "fuel_want_kg"), num(r.v, "fuel_after_kg"));
        let pay_ok = close(num(r.v, "payload_want_kg"), num(r.v, "payload_after_kg"));
        let line = format!(
            "{}: fuel {:.0}/{:.0} kg, payload {:.0}/{:.0} kg",
            hm(r.t),
            num(r.v, "fuel_after_kg").unwrap_or(0.0),
            num(r.v, "fuel_want_kg").unwrap_or(0.0),
            num(r.v, "payload_after_kg").unwrap_or(0.0),
            num(r.v, "payload_want_kg").unwrap_or(0.0)
        );
        if flag(r.v, "ok") == Some(true) && fuel_ok && pay_ok {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    // One line per distinct result is enough.
    oks.dedup_by(|a, b| a[6..] == b[6..]);
    verdict(fails, oks)
}

fn c_ils_tuned(log: &Log) -> Verdict {
    let tuned: Vec<&Rec> = log.of("ils", "tuned").collect();
    let bad = tuned
        .iter()
        .filter(|r| flag(r.v, "ok") != Some(true))
        .count();
    let xchecks = log.of("ils", "xcheck").count();
    if tuned.is_empty() {
        Verdict::NotExercised
    } else if bad > 0 {
        Verdict::Failed(format!("{bad} of {} tunes did not read back", tuned.len()))
    } else {
        Verdict::Verified(format!(
            "{} tunes read back, {xchecks} needle cross-check call(s)",
            tuned.len()
        ))
    }
}

fn c_ils_coupled(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("ils", "coupling") {
        match text(r.v, "to") {
            Some("Coupled") => oks.push(format!(
                "{}: coupled at {:.1} nm",
                hm(r.t),
                num(r.v, "dist").unwrap_or(0.0)
            )),
            Some("Abandoned") => fails.push(format!(
                "{}: abandoned from {} at {:.1} nm",
                hm(r.t),
                text(r.v, "from").unwrap_or("?"),
                num(r.v, "dist").unwrap_or(0.0)
            )),
            _ => {}
        }
    }
    verdict(fails, oks)
}

fn c_ils_setup(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("ils", "setup") {
        let followed = log.recs.iter().any(|x| {
            x.t > r.t
                && x.t <= r.t + 600.0
                && x.kind == "ils"
                && matches!(x.name, "watch" | "appr_pressed" | "missed_alt_set")
        });
        let line = format!(
            "{}: {} {}, verdict {}",
            hm(r.t),
            text(r.v, "icao").unwrap_or("?"),
            text(r.v, "rwy").unwrap_or("?"),
            text(r.v, "verdict").unwrap_or("?")
        );
        if followed {
            oks.push(line);
        } else {
            fails.push(format!("{line}, no callout or APPR in 10 min"));
        }
    }
    verdict(fails, oks)
}

fn c_systems(log: &Log) -> Verdict {
    let mut seen: Vec<String> = Vec::new();
    let mut bad = Vec::new();
    for r in log.recs.iter().filter(|r| r.kind == "systems") {
        let ap = text(r.v, "ap_type").unwrap_or("none");
        let line = format!("{} {ap}", text(r.v, "profile").unwrap_or("?"));
        if ap == "none" || ap == "Unknown" {
            bad.push(line.clone());
        }
        if !seen.contains(&line) {
            seen.push(line);
        }
    }
    if seen.is_empty() {
        Verdict::NotExercised
    } else if !bad.is_empty() {
        Verdict::Failed(format!("no autopilot type read: {}", bad.join(", ")))
    } else {
        Verdict::Verified(seen.join(", "))
    }
}

fn c_seat_belts(log: &Log) -> Verdict {
    let evs: Vec<&Rec> = log.of("cabin", "seat_belt_signs").collect();
    if evs.is_empty() {
        return Verdict::NotExercised;
    }
    let bad = evs.iter().filter(|r| flag(r.v, "ok") != Some(true)).count();
    let seq: String = evs
        .iter()
        .map(|r| {
            if flag(r.v, "on") == Some(true) {
                '1'
            } else {
                '0'
            }
        })
        .collect();
    let off_then_on = seq.find('0').is_some_and(|i| seq[i..].contains('1'));
    let line = format!("{} switchings, sequence {seq} (1 on, 0 off)", evs.len());
    if bad > 0 {
        Verdict::Failed(format!("{bad} did not take; {line}"))
    } else if seq.contains('0') && !off_then_on {
        Verdict::Failed(format!("off and never on again; {line}"))
    } else {
        Verdict::Verified(line)
    }
}

fn c_cabin_after_landing(log: &Log) -> Verdict {
    let mut landed: Option<f64> = None;
    let mut landings = 0;
    let mut fails = Vec::new();
    for r in &log.recs {
        match (r.kind, r.name) {
            ("flight", "touchdown") => {
                landed = Some(r.t);
                landings += 1;
            }
            ("flight", "flight_complete" | "takeoff") | ("sortie", "begin") => landed = None,
            ("cabin", "securing_for_departure") => {
                if let Some(t) = landed {
                    fails.push(format!(
                        "{}: secure for departure {:.0} s after the landing",
                        hm(r.t),
                        r.t - t
                    ));
                }
            }
            _ => {}
        }
    }
    if landings == 0 {
        return Verdict::NotExercised;
    }
    verdict(
        fails,
        vec![format!("{landings} landings, no departure call after any")],
    )
}

fn c_cabin_air(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("full_auto", "cabin_air") {
        let (bleed, ox) = (flag(r.v, "bleed"), flag(r.v, "oxygen"));
        if bleed.is_none() && ox.is_none() {
            continue;
        }
        let line = format!("{}: bleed {bleed:?}, oxygen {ox:?}", hm(r.t));
        if bleed != Some(false) && ox != Some(false) {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_takeoff_config_taken(log: &Log) -> Verdict {
    let cfgs: Vec<&Rec> = log.of("autotakeoff", "takeoff_config").collect();
    let bad: Vec<String> = cfgs
        .iter()
        .filter(|r| r.v["not_taken"].as_array().is_some_and(|a| !a.is_empty()))
        .map(|r| format!("{}: {}", hm(r.t), r.v["not_taken"]))
        .collect();
    if cfgs.is_empty() {
        Verdict::NotExercised
    } else if !bad.is_empty() {
        Verdict::Failed(bad.join("; "))
    } else {
        Verdict::Verified(format!("{} takeoff configs, nothing not taken", cfgs.len()))
    }
}

fn c_transponder_alt(log: &Log) -> Verdict {
    let evs: Vec<&Rec> = log.of("autotakeoff", "transponder_alt").collect();
    let bad = evs.iter().filter(|r| flag(r.v, "ok") != Some(true)).count();
    if evs.is_empty() {
        Verdict::NotExercised
    } else if bad > 0 {
        Verdict::Failed(format!("{bad} of {} did not confirm ALT", evs.len()))
    } else {
        Verdict::Verified(format!("{} of {} confirmed ALT", evs.len(), evs.len()))
    }
}

fn c_a330_spoilers_disarmed(log: &Log) -> Verdict {
    let evs: Vec<&Rec> = log.of("autotakeoff", "spoilers_disarmed").collect();
    let gear = log.of("autotakeoff", "gear_up").count();
    let bad = evs.iter().filter(|r| flag(r.v, "ok") != Some(true)).count();
    if gear == 0 {
        Verdict::NotExercised
    } else if evs.len() < gear || bad > 0 {
        Verdict::Failed(format!(
            "{} disarm(s) for {gear} gear retraction(s), {bad} not taken",
            evs.len()
        ))
    } else {
        Verdict::Verified(format!("disarmed after all {gear} gear retraction(s)"))
    }
}

fn c_hand_back_brakes(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("full_auto", "hand_back_brakes") {
        let line = format!(
            "{}: held {:?}, tries {:?}, err {:?}",
            hm(r.t),
            flag(r.v, "held"),
            num(r.v, "tries"),
            text(r.v, "err")
        );
        if flag(r.v, "held") == Some(true) {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_tug_unhook(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("pushback", "brakes_set") {
        match flag(r.v, "tug_clear") {
            Some(true) => oks.push(format!("{}: tug clear", hm(r.t))),
            Some(false) => fails.push(format!("{}: tug never showed clear", hm(r.t))),
            None => {}
        }
    }
    verdict(fails, oks)
}

fn c_push_lane(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for lane in log.of("full_auto", "push_lane") {
        let want = num(lane.v, "back_m").unwrap_or(f64::NAN);
        let Some(stop) = log.next(lane.t, "pushback", "stopped", 600.0) else {
            continue;
        };
        let got = num(stop.v, "back_m").unwrap_or(f64::NAN);
        let line = format!("{}: planned {want:.0} m, pushed {got:.0} m", hm(lane.t));
        if (got - want).abs() <= 10.0 {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_leg_stall(log: &Log) -> Verdict {
    let legs = log.of("procedure", "leg").count();
    if legs == 0 {
        return Verdict::NotExercised;
    }
    let stalls: Vec<f64> = log
        .of("procedure", "leg_stalled")
        .filter_map(|r| num(r.v, "stalls"))
        .collect();
    let stood: Vec<String> = log
        .of("procedure", "stand_down")
        .filter(|r| text(r.v, "reason") == Some("leg_stalled"))
        .map(|r| hm(r.t))
        .collect();
    let line = format!(
        "{legs} follower ticks, {} leg_stalled (most {:.0} on one leg)",
        stalls.len(),
        stalls.iter().fold(0.0f64, |m, x| m.max(*x))
    );
    if stood.is_empty() {
        Verdict::Verified(line)
    } else {
        Verdict::Failed(format!(
            "stood down on stalls at {}; {line}",
            stood.join(", ")
        ))
    }
}

fn c_heading_once(log: &Log) -> Verdict {
    let sets: Vec<&Rec> = log.of("procedure", "heading_set").collect();
    if sets.is_empty() {
        return Verdict::NotExercised;
    }
    let mut flips = Vec::new();
    for w in sets.windows(3) {
        let same_leg = num(w[0].v, "idx") == num(w[2].v, "idx");
        let quick = w[2].t - w[0].t <= 30.0;
        let d1 = wrap180(num(w[1].v, "hdg").unwrap_or(0.0) - num(w[0].v, "hdg").unwrap_or(0.0));
        let d2 = wrap180(num(w[2].v, "hdg").unwrap_or(0.0) - num(w[1].v, "hdg").unwrap_or(0.0));
        if same_leg && quick && d1 * d2 < 0.0 && d1.abs() >= 2.0 && d2.abs() >= 2.0 {
            flips.push(hm(w[1].t));
        }
    }
    flips.dedup();
    if flips.is_empty() {
        Verdict::Verified(format!("{} heading sets, no flip-flop", sets.len()))
    } else {
        Verdict::Failed(format!(
            "{} heading sets, back and forth within 30 s at {}",
            sets.len(),
            flips.join(", ")
        ))
    }
}

fn c_self_pick_destination(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("autoland", "self_pick") {
        let Some(dest) = text(r.v, "destination") else {
            continue;
        };
        let rwy = text(r.v, "rwy").unwrap_or("none");
        let line = format!("{}: {rwy} for {dest}", hm(r.t));
        if rwy.starts_with(dest) {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    oks.dedup_by(|a, b| a[6..] == b[6..]);
    verdict(fails, oks)
}

fn c_late_go_around(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("autoland", "late_go_around") {
        let line = format!(
            "{}: {:.0} ft above the path at {:.1} nm",
            hm(r.t),
            num(r.v, "above_path_ft").unwrap_or(0.0),
            num(r.v, "dist").unwrap_or(0.0)
        );
        let crashed = log
            .recs
            .iter()
            .any(|x| x.t > r.t && x.t <= r.t + 300.0 && x.kind == "crash");
        if crashed {
            fails.push(format!("{line}, then a crash"));
        } else {
            oks.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_map_flaps_on_speed(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    let firsts: Vec<&Rec> = runs(log, "procedure", "map_config", 600.0)
        .into_iter()
        .map(|r| r[0])
        .collect();
    for f in firsts {
        let slow = log
            .recs
            .iter()
            .filter(|r| r.t >= f.t && r.t <= f.t + 300.0 && r.kind == "callout")
            .filter(|r| matches!(r.name, "airspeed_caution" | "airspeed_warning"))
            .count();
        let line = format!("{}: {slow} low-speed callout(s) on the final", hm(f.t));
        if slow == 0 {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_tod_once(log: &Log) -> Verdict {
    let mut per_flight: Vec<(f64, usize)> = Vec::new();
    for r in &log.recs {
        if r.kind == "flight" && r.name == "takeoff" {
            per_flight.push((r.t, 0));
        }
        if r.kind == "callout" && matches!(r.name, "tod_ahead" | "tod_passed") {
            if per_flight.is_empty() {
                per_flight.push((r.t, 0));
            }
            per_flight.last_mut().unwrap().1 += 1;
        }
    }
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for (t, n) in per_flight.into_iter().filter(|(_, n)| *n > 0) {
        let line = format!("flight from {}: {n} descent call(s)", hm(t));
        if n <= 2 {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_circuit_short_way(log: &Log) -> Verdict {
    let mut worst: f64 = 0.0;
    let mut long = Vec::new();
    let mut legs = 0;
    let mut cur: Option<(&str, f64, f64, f64)> = None; // leg, t0, last psi, net turn
    for r in log.of("circuit", "leg") {
        let (Some(leg), Some(psi)) = (text(r.v, "leg"), num(r.v, "psi")) else {
            continue;
        };
        let same = cur.is_some_and(|c| c.0 == leg && r.t - c.1 < 900.0);
        if same {
            if let Some(c) = cur.as_mut() {
                c.3 += wrap180(psi - c.2);
                c.2 = psi;
            }
        } else {
            if let Some(c) = cur.take() {
                worst = worst.max(c.3.abs());
                if c.3.abs() > 200.0 {
                    long.push(format!("{} {} {:.0} deg", hm(c.1), c.0, c.3.abs()));
                }
            }
            legs += 1;
            cur = Some((leg, r.t, psi, 0.0));
        }
    }
    if let Some(c) = cur {
        worst = worst.max(c.3.abs());
        if c.3.abs() > 200.0 {
            long.push(format!("{} {} {:.0} deg", hm(c.1), c.0, c.3.abs()));
        }
    }
    if legs == 0 {
        Verdict::NotExercised
    } else if !long.is_empty() {
        Verdict::Failed(format!("turned the long way: {}", long.join(", ")))
    } else {
        Verdict::Verified(format!(
            "{legs} legs, the most any leg turned was {worst:.0} deg"
        ))
    }
}

fn c_circuit_alt_capture(log: &Log) -> Verdict {
    let legs: Vec<&Rec> = log.of("circuit", "leg").collect();
    if legs.is_empty() {
        return Verdict::NotExercised;
    }
    let held = legs
        .iter()
        .filter(|r| flag(r.v, "ap_level") == Some(true))
        .count();
    let refused = log.of("circuit", "vs_refused").count();
    let line = format!(
        "{held} of {} leg ticks on the autopilot's capture, {refused} vs_refused",
        legs.len()
    );
    if refused > 0 || held == 0 {
        Verdict::Failed(line)
    } else {
        Verdict::Verified(line)
    }
}

fn c_sf50_gear(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("autotakeoff", "gear_up") {
        let (ias, agl) = (
            num(r.v, "ias").unwrap_or(0.0),
            num(r.v, "agl").unwrap_or(0.0),
        );
        let line = format!("{}: gear up at {ias:.0} kt, {agl:.0} ft", hm(r.t));
        // The SF50's .acf horn: below 125 kt with flap past 0.5 (ETND 26).
        if ias >= 128.0 || agl >= 990.0 || flag(r.v, "at_handoff") == Some(true) {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_committed_flare(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("autoland", "committed_flare") {
        let crash = log
            .recs
            .iter()
            .any(|x| x.t > r.t && x.t <= r.t + 60.0 && x.kind == "crash");
        let down = log.next(r.t, "flight", "touchdown", 60.0);
        let line = format!(
            "{}: committed at {:.0} ft with {:.0} m left",
            hm(r.t),
            num(r.v, "radio").unwrap_or(0.0),
            num(r.v, "remaining_m").unwrap_or(0.0)
        );
        match (crash, down) {
            (false, Some(d)) => oks.push(format!(
                "{line}, down at {:.0} fpm",
                num(d.v, "sink").unwrap_or(0.0)
            )),
            _ => fails.push(format!(
                "{line}, {}",
                if crash {
                    "crashed"
                } else {
                    "no touchdown in 60 s"
                }
            )),
        }
    }
    verdict(fails, oks)
}

fn c_stand_shutdown(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("taxi_in", "shutdown") {
        let stopped = flag(r.v, "stopped") == Some(true);
        let elec = log.next(r.t - 1.0, "taxi_in", "electrics_off", 30.0);
        let eline = elec
            .and_then(|e| text(e.v, "line"))
            .unwrap_or("no electrics_off record");
        let clean = elec.is_some() && !eline.contains("except") && !eline.contains("didn't");
        let line = format!(
            "{} {}: engines {}, \"{eline}\"",
            hm(r.t),
            r.ac.unwrap_or("?"),
            if stopped { "stopped" } else { "NOT stopped" }
        );
        if stopped && clean {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_vacate_clean(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for v in log.of("taxi_in", "vacating") {
        let done = log.next(v.t, "taxi_in", "vacated", 600.0);
        let end = log
            .next(v.t, "taxi_in", "secure", 1800.0)
            .map(|r| r.t)
            .unwrap_or(v.t + 1800.0);
        let off = log
            .recs
            .iter()
            .filter(|r| r.t > v.t && r.t <= end && r.kind == "autotaxi" && r.name == "off_route")
            .count();
        let line = format!(
            "{}: runway {} via {}",
            hm(v.t),
            text(v.v, "rwy").unwrap_or("?"),
            text(v.v, "exit").unwrap_or("?")
        );
        match done {
            Some(d) if flag(d.v, "arrived") == Some(true) && off == 0 => oks.push(line),
            Some(_) => fails.push(format!("{line}: {off} off_route stop(s) on the taxi in")),
            None => fails.push(format!("{line}: never vacated")),
        }
    }
    verdict(fails, oks)
}

fn c_untowered(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log
        .of("full_auto", "departure_field")
        .filter(|r| flag(r.v, "towered") == Some(false))
    {
        if log.next(r.t, "full_auto", "lined_up", 1800.0).is_some() {
            oks.push(format!("{}: untowered departure lined up", hm(r.t)));
        } else {
            fails.push(format!("{}: untowered departure never lined up", hm(r.t)));
        }
    }
    verdict(fails, oks)
}

fn c_waiver(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("full_auto", "waiver") {
        let gate = text(r.v, "gate").unwrap_or("?");
        if log.next(r.t, "autotakeoff", "liftoff", 1800.0).is_some() {
            oks.push(format!("{}: {gate} waived, took off", hm(r.t)));
        } else {
            fails.push(format!("{}: {gate} waived, no takeoff in 30 min", hm(r.t)));
        }
    }
    verdict(fails, oks)
}

fn c_touchdown_idle(log: &Log) -> Verdict {
    verdict(
        vec![],
        log.of("speedhold", "touchdown_idle")
            .map(|r| {
                format!(
                    "{}: idled at {:.0} kt (target {:.0})",
                    hm(r.t),
                    num(r.v, "ias").unwrap_or(0.0),
                    num(r.v, "target").unwrap_or(0.0)
                )
            })
            .collect(),
    )
}

fn c_spoken_refused(log: &Log) -> Verdict {
    verdict(
        vec![],
        log.of("speedhold", "spoken_target_refused")
            .map(|r| {
                format!(
                    "{}: refused {:.0} kt",
                    hm(r.t),
                    num(r.v, "asked").unwrap_or(0.0)
                )
            })
            .collect(),
    )
}

fn c_taxi_stage(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for s in stages(log, "taxi") {
        let line = format!(
            "{}: taxi {}",
            hm(num(&s, "t").unwrap_or(0.0)),
            crate::scorecard::line(&s)
        );
        if flag(&s, "pass") == Some(true) {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_heli_approach(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for h in log.of("heli_route", "handover_hover") {
        let ias = num(h.v, "ias").unwrap_or(f64::NAN);
        let arrived = log.next(h.t, "heli_route", "arrived", 900.0);
        let crash = log
            .recs
            .iter()
            .any(|x| x.t > h.t && x.t <= h.t + 900.0 && x.kind == "crash");
        let line = format!(
            "{}: hover loop took over at {ias:.0} kt, {:.0} m from the pad",
            hm(h.t),
            num(h.v, "to_pad_m").unwrap_or(0.0)
        );
        if !crash && ias <= 45.0 && arrived.is_some_and(|a| flag(a.v, "on_ground") == Some(true)) {
            oks.push(line);
        } else {
            fails.push(format!(
                "{line}, {}",
                if crash {
                    "crashed"
                } else {
                    "not down on the pad"
                }
            ));
        }
    }
    verdict(fails, oks)
}

fn c_heli_inbound(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("heli_route", "inbound_call") {
        let line = format!(
            "{}: inbound on {:.0}",
            hm(r.t),
            num(r.v, "khz").unwrap_or(0.0)
        );
        if flag(r.v, "station_swapped") == Some(true) {
            oks.push(line);
        } else {
            fails.push(format!("{line} before the station swapped"));
        }
    }
    verdict(fails, oks)
}

fn c_heli_route_whole(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for s in log.of("heli_route", "start") {
        let crash = log
            .recs
            .iter()
            .any(|x| x.t > s.t && x.t <= s.t + 7200.0 && x.kind == "crash");
        let arrived = log.next(s.t, "heli_route", "arrived", 7200.0);
        let line = format!(
            "{}: cruise {:.0} ft over {:.0} ft of ground",
            hm(s.t),
            num(s.v, "cruise_alt_ft").unwrap_or(0.0),
            num(s.v, "terrain_ft").unwrap_or(0.0)
        );
        match (crash, arrived) {
            (false, Some(a)) if flag(a.v, "on_ground") == Some(true) => {
                oks.push(format!("{line}, landed"))
            }
            (true, _) => fails.push(format!("{line}, CRASHED")),
            _ => {}
        }
    }
    verdict(fails, oks)
}

fn c_heli_pad_shutdown(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for p in log.of("heli_route", "pad_shutdown") {
        match log.next(p.t, "taxi_in", "shutdown", 120.0) {
            Some(s) if flag(s.v, "stopped") == Some(true) => {
                oks.push(format!("{}: shut down on the pad", hm(p.t)))
            }
            _ => fails.push(format!(
                "{}: pad shutdown did not stop the engines",
                hm(p.t)
            )),
        }
    }
    verdict(fails, oks)
}

fn c_heli_radio(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for d in log.of("heli_route", "departure_call") {
        let cleared = log.recs.iter().any(|r| {
            r.t <= d.t && r.t >= d.t - 600.0 && r.kind == "heli_route" && r.name == "zone_clearance"
        });
        if cleared {
            oks.push(format!(
                "{}: zone clearance before the departure call",
                hm(d.t)
            ));
        } else {
            fails.push(format!(
                "{}: departure call with no zone clearance",
                hm(d.t)
            ));
        }
    }
    verdict(fails, oks)
}

fn c_hover_hand_back(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("hover", "hand_back") {
        let line = format!("{}: {}", hm(r.t), text(r.v, "why").unwrap_or("?"));
        if flag(r.v, "cruise_modes") == Some(true) || flag(r.v, "hover_mode") == Some(true) {
            oks.push(line);
        } else {
            fails.push(format!(
                "{line}: handed back with neither cruise nor hover mode"
            ));
        }
    }
    verdict(fails, oks)
}

fn c_cold_start(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("turbine_start", "crank") {
        let v = text(r.v, "verdict").unwrap_or("?");
        let line = format!(
            "{}: engine {:.0} {v}, peak {:.1} %",
            hm(r.t),
            num(r.v, "engine").unwrap_or(0.0),
            num(r.v, "peak_spool").unwrap_or(0.0)
        );
        if v == "Lit" {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_cabin_open_line(log: &Log) -> Verdict {
    let opened: Vec<String> = log
        .recs
        .iter()
        .filter(|r| r.kind == "cabin" && r.v["route"] == "call")
        .filter(|r| text(r.v, "reply").is_some_and(|x| x.starts_with("Go ahead")))
        .map(|r| format!("{}: \"Go ahead, flight deck.\"", hm(r.t)))
        .collect();
    verdict(vec![], opened)
}

fn c_handover(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("handover", "decide") {
        let upset = flag(r.v, "upset") == Some(true);
        let recovered = log
            .recs
            .iter()
            .any(|x| x.t > r.t && x.t <= r.t + 10.0 && x.kind == "recover");
        let line = format!(
            "{}: {} in {} ({})",
            hm(r.t),
            if flag(r.v, "took") == Some(true) {
                "took"
            } else {
                "declined"
            },
            text(r.v, "phase").unwrap_or("?"),
            text(r.v, "why").unwrap_or("?")
        );
        if recovered && !upset {
            fails.push(format!("{line}, and the upset recovery ran"));
        } else {
            oks.push(line);
        }
    }
    verdict(fails, oks)
}

const PRAISE: [&str; 6] = [
    "Nice",
    "Lovely",
    "Beautiful",
    "Greaser",
    "Good landing",
    "Smooth",
];

fn c_rapport(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for td in log.of("rapport", "touchdown") {
        let sink = num(td.v, "sink_fpm").unwrap_or(0.0);
        let auto = flag(td.v, "by_automation") == Some(true);
        let said: Vec<&str> = log
            .of("rapport", "said")
            .filter(|r| r.t >= td.t && r.t <= td.t + 150.0)
            .filter_map(|r| text(r.v, "line"))
            .collect();
        let praised = said.iter().any(|l| PRAISE.iter().any(|p| l.contains(p)));
        let hard_said = said.iter().any(|l| l.contains("hard"));
        let line = format!(
            "{}: {sink:.0} fpm by {}, said {:?}",
            hm(td.t),
            if auto { "automation" } else { "pilot" },
            said.first().unwrap_or(&"nothing")
        );
        if (auto && praised) || (sink >= 600.0 && !hard_said) {
            fails.push(line);
        } else {
            oks.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_say_again(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("atc", "say_again_repeat") {
        let age = num(r.v, "age_s").unwrap_or(f64::NAN);
        let line = format!("{}: repeated a {age:.0} s old line", hm(r.t));
        if age <= 60.0 {
            oks.push(line);
        } else {
            fails.push(line);
        }
    }
    verdict(fails, oks)
}

fn c_duty(log: &Log) -> Verdict {
    let all: Vec<&Rec> = log.recs.iter().filter(|r| r.kind == "duty").collect();
    if all.is_empty() {
        return Verdict::NotExercised;
    }
    let mut dis: Vec<String> = all
        .iter()
        .filter(|r| flag(r.v, "agrees") == Some(false))
        .map(|r| text(r.v, "duty").unwrap_or("?").to_string())
        .collect();
    let n = dis.len();
    dis.sort();
    dis.dedup();
    if n == 0 {
        Verdict::Verified(format!(
            "{} duty changes, all agreed with the automation",
            all.len()
        ))
    } else {
        Verdict::Failed(format!(
            "{n} of {} duty changes disagreed ({})",
            all.len(),
            dis.join(", ")
        ))
    }
}

fn c_flight_model_touchdowns(log: &Log) -> Verdict {
    let (mut fails, mut oks) = (Vec::new(), Vec::new());
    for r in log.of("rapport", "touchdown") {
        let seen = log
            .recs
            .iter()
            .any(|x| x.kind == "flight" && x.name == "touchdown" && (x.t - r.t).abs() <= 5.0);
        if seen {
            oks.push(hm(r.t));
        } else {
            fails.push(format!("{}: a touchdown the flight model missed", hm(r.t)));
        }
    }
    match verdict(fails, oks) {
        Verdict::Verified(l) => {
            Verdict::Verified(format!("every touchdown seen by the model ({l})"))
        }
        v => v,
    }
}

// ------------------------------------------------------------ the card

const S0921: f64 = 1_790_024_255.0; // 3950a94
const S0922: f64 = 1_790_098_337.0; // 125e2b3
const S0923H: f64 = 1_790_173_389.0; // b3091d8 / 058c6ff
const S0923: f64 = 1_790_195_386.0; // f1d6791
const S0924: f64 = 1_790_254_732.0; // 8dddc5b
const S0924L: f64 = 1_790_261_916.0; // 858a0d4
const S0924S: f64 = 1_790_265_571.0; // 5f8a38c
const S0925A: f64 = 1_790_288_529.0; // b30865d
const S0925B: f64 = 1_790_340_379.0; // 389728a
const S0925C: f64 = 1_790_344_645.0; // cadf9f8
const S0925D: f64 = 1_790_357_663.0; // f30f1f0
const S0925E: f64 = 1_790_366_250.0; // 5d5bf46
const S0926A: f64 = 1_790_374_079.0; // ee493cc
const S0926B: f64 = 1_790_416_510.0; // 327d121
const S0926C: f64 = 1_790_428_542.0; // 3fd8e6b
const S0926D: f64 = 1_790_429_327.0; // a8730cb
const S0926E: f64 = 1_790_431_247.0; // 3226e77
const S0926F: f64 = 1_790_439_487.0; // b417a7f
const S0926G: f64 = 1_790_439_787.0; // e4a196f
const S0926H: f64 = 1_790_440_148.0; // dd501bc
const S0926I: f64 = 1_790_440_265.0; // 3b29f25
const S0926J: f64 = 1_790_442_119.0; // 3a64155
const S0926K: f64 = 1_790_443_683.0; // f107203
const S0926L: f64 = 1_790_444_833.0; // 18627ec

const B738: Option<&str> = Some("boeing_737");
const A330: Option<&str> = Some("airbus_a330");
const SF50: Option<&str> = Some("cirrus_vision_sf50");
const S76: Option<&str> = Some("sikorsky_s76");

pub static ITEMS: &[Item] = &[
    // --- Airliner full-auto IFR.
    Item {
        id: "b738_speedbrake_down",
        sortie: Sortie::Airliner,
        what: "A Boeing takeoff puts the speedbrake lever down, never armed for the roll",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: B738,
        fly: "737, any full-auto departure",
        evidence: "autotakeoff takeoff_config: set, not_taken",
        pass: "no \"spoilers armed\" in any 737 set",
        check: Some(c_b738_speedbrake_down),
    },
    Item {
        id: "jet_climb_out",
        sortie: Sortie::Airliner,
        what: "Jet climb-out: VS holds V2+20 below the acceleration height, flaps wait for it",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: None,
        fly: "737 or A330, full-auto departure to the handoff",
        evidence: "autotakeoff climb_out: accel_agl_ft; flap_step: agl_ft, ias; flaps_handoff",
        pass: "every flap_step at or above accel_agl_ft (25 ft slack), no flaps_handoff after a climb_out",
        check: Some(c_jet_climb_out),
    },
    Item {
        id: "roll_trim_scaled",
        sortie: Sortie::Airliner,
        what: "Takeoff roll rudder trim shrinks with the square of the speed (LIBP 04 crash)",
        shipped: "2026-09-26 3fd8e6b",
        since: S0926C,
        airframe: None,
        fly: "737, full-auto takeoff",
        evidence: "autotakeoff roll: xtk_m, trim, gs",
        pass: "most off the centreline on the roll within the scorecard's 5 m",
        check: Some(c_roll_centreline),
    },
    Item {
        id: "reject_to_a_stop",
        sortie: Sortie::Airliner,
        what: "A rolling reject is flown to a stop on the centreline under max braking",
        shipped: "2026-09-26 3fd8e6b",
        since: S0926C,
        airframe: None,
        fly: "any, a takeoff that rejects (excursion or stagnation)",
        evidence: "autotakeoff reject; reject_rollout: gs, xtk_m",
        pass: "last reject_rollout tick under 25 kt, never more than 15 m off",
        check: Some(c_reject_to_a_stop),
    },
    Item {
        id: "rollout_law",
        sortie: Sortie::Airliner,
        what: "Rollout steering leads with a smoothed cross-track rate",
        shipped: "2026-09-26 3a64155",
        since: S0926J,
        airframe: None,
        fly: "any autoland",
        evidence: "autoland rollout: xtk_m, xtk_rate, gs",
        pass: "the offset grows no more than 2 m after the first tick, at most one crossing above 40 kt",
        check: Some(c_rollout_law),
    },
    Item {
        id: "touchdown_centreline",
        sortie: Sortie::Airliner,
        what: "Over the runway with the servos out the wings follow the centreline law plus the crab",
        shipped: "2026-09-26 f107203",
        since: S0926K,
        airframe: None,
        fly: "737 autoland",
        evidence: "autoland rollout: xtk_m at the first tick",
        pass: "within the scorecard's 5 m of the centreline at the wheels",
        check: Some(c_touchdown_centreline),
    },
    Item {
        id: "ap_off_at_wheels",
        sortie: Sortie::Airliner,
        what: "Autoland takes the autopilot out at the wheels",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: None,
        fly: "any autoland to a stop",
        evidence: "autoland ap_disconnected: ok; state in.ap",
        pass: "AP off within 10 s of the first rollout tick (already off counts)",
        check: Some(c_ap_off_at_wheels),
    },
    Item {
        id: "b738_flare",
        sortie: Sortie::Airliner,
        what: "737 flare: retard by height, exponential target, pull-only arrest from the roundout's pull",
        shipped: "2026-09-26 630506f..f107203",
        since: S0926K,
        airframe: B738,
        fly: "737 autoland",
        evidence: "scorecard landing stage (sink_fpm) from state and autoland ticks; autoland flare_elev, cmd_vs, vvi",
        pass: "touchdown at or under the scorecard's 400 fpm",
        check: Some(c_touchdown_sink),
    },
    Item {
        id: "altimeter_rounds",
        sortie: Sortie::Airliner,
        what: "The altimeter is stepped in rounds earned by progress and says when it did not take (EGSS)",
        shipped: "2026-09-22 125e2b3",
        since: S0922,
        airframe: None,
        fly: "737 loaded cold (baro memory 0.0), full auto",
        evidence: "full_auto altimeter_set: took, want, inhg; procedure altimeter_set: took; callout altimeter_reset: took",
        pass: "every setting took",
        check: Some(c_altimeter_took),
    },
    Item {
        id: "reversal_pattern",
        sortie: Sortie::Airliner,
        what: "A reposition is a pattern, never a point (EGTE)",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: None,
        fly: "any, autoland armed facing away from the runway",
        evidence: "autoland reversal: side, platform_ft; reversal_done: xtk_m, dist",
        pass: "a pattern that runs to its end is reversal_done within 100 m of the centreline",
        check: Some(c_reversal),
    },
    Item {
        id: "pattern_height",
        sortie: Sortie::Airliner,
        what: "The pattern extends its downwind while it is too high to turn (EGSS)",
        shipped: "2026-09-22 125e2b3",
        since: S0922,
        airframe: None,
        fly: "737, autoland armed high above the field",
        evidence: "autoland downwind_extended: above_ft, along_nm",
        pass: "the extended pattern ends in reversal_done, not a go-around or give-up",
        check: Some(c_pattern_height),
    },
    Item {
        id: "descent_drag",
        sortie: Sortie::Airliner,
        what: "A descent that outruns the governor gets the speedbrake, stowed on every exit",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: None,
        fly: "737 full-auto arrival",
        evidence: "procedure speedbrake: out, ias, target",
        pass: "every extension stowed by the end",
        check: Some(c_descent_drag),
    },
    Item {
        id: "descent_path",
        sortie: Sortie::Airliner,
        what: "A descent is flown down a path at 75 % of the rate from an earlier top of descent",
        shipped: "2026-09-26 dd501bc",
        since: S0926H,
        airframe: None,
        fly: "737 full-auto arrival from cruise",
        evidence: "procedure leg: want_alt, alt_ft; procedure speedbrake: out",
        pass: "at most 2 speedbrake extensions over the descent (13 on LIBP-LEMG before)",
        check: Some(c_descent_path),
    },
    Item {
        id: "tod_once",
        sortie: Sortie::Airliner,
        what: "The top of descent is called once, from the cruise",
        shipped: "2026-09-26 b417a7f",
        since: S0926F,
        airframe: None,
        fly: "any full-auto arrival with stepped descents",
        evidence: "callout tod_ahead, tod_passed",
        pass: "at most one ahead and one passed call per flight",
        check: Some(c_tod_once),
    },
    Item {
        id: "inbound_retune",
        sortie: Sortie::Airliner,
        what: "The inbound call is keyed on the tower's frequency",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: None,
        fly: "any full-auto arrival into a towered field",
        evidence: "full_auto inbound_retune: khz, was; inbound_call: khz",
        pass: "every retune is followed by the inbound call on that frequency",
        check: Some(c_inbound_retune),
    },
    Item {
        id: "star_windows",
        sortie: Sortie::Airliner,
        what: "STAR altitude windows clamped into the plan and flown",
        shipped: "2026-09-24 b30865d",
        since: S0925A,
        airframe: None,
        fly: "737 full auto with a SimBrief STAR",
        evidence: "plan_audit star_constraints: changed_legs, constraints; procedure star_ceiling; procedure leg: fix, alt_ft, dist_nm",
        pass: "every constrained fix crossed inside its window (300 ft slack)",
        check: Some(c_star_windows),
    },
    Item {
        id: "star_speeds",
        sortie: Sortie::Airliner,
        what: "STAR speed limits slowed for in time and held",
        shipped: "2026-09-24 b30865d",
        since: S0925A,
        airframe: None,
        fly: "737 full auto with a STAR speed limit",
        evidence: "procedure star_speed: fix, kts, ias; state ias at the fix",
        pass: "IAS at each limited fix within 10 kt of the limit",
        check: Some(c_star_speeds),
    },
    Item {
        id: "own_star",
        sortie: Sortie::Airliner,
        what: "The crew picks its STAR for the METAR-favoured runway at the arm",
        shipped: "2026-09-25 f30f1f0",
        since: S0925D,
        airframe: None,
        fly: "737 full auto into a field whose wind favours another end than the plan's",
        evidence: "plan_audit star_chosen: star, runway, planned_star, planned_runway, wind; taxi_in vacating: rwy",
        pass: "the runway landed on is the runway chosen",
        check: Some(c_own_star),
    },
    Item {
        id: "own_sid",
        sortie: Sortie::Airliner,
        what: "The crew picks the SID from the cleared runway",
        shipped: "2026-09-25 f30f1f0",
        since: S0925D,
        airframe: None,
        fly: "full auto where the tower clears a runway the plan's SID does not serve (LIME 10)",
        evidence: "full_auto departure_rechosen: line",
        pass: "a liftoff follows the re-chosen departure",
        check: Some(c_own_sid),
    },
    Item {
        id: "plan_audit",
        sortie: Sortie::Airliner,
        what: "The whole plan is dry-run before it is flown",
        shipped: "2026-09-24 b30865d",
        since: S0925A,
        airframe: None,
        fly: "any full auto with a plan",
        evidence: "plan_audit (no event): findings, profile, probed_nm; sortie begin: origin, dest",
        pass: "every planned mission audited at the arm, none armed over a refusal",
        check: Some(c_plan_audit),
    },
    Item {
        id: "load_sheet",
        sortie: Sortie::Airliner,
        what: "\"Load the aircraft\" sets block fuel (wings first) and payload from the plan",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: None,
        fly: "737 or A330 on the ramp: \"load the aircraft\"",
        evidence: "loading load_aircraft: ok, fuel_want_kg, fuel_after_kg, payload_want_kg, payload_after_kg, per_tank_kg",
        pass: "ok, fuel and payload within 2 % (or 5 kg) of the plan",
        check: Some(c_load_sheet),
    },
    Item {
        id: "takeoff_config",
        sortie: Sortie::Airliner,
        what: "A crew configures the aircraft before it moves",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: None,
        fly: "any full-auto departure",
        evidence: "autotakeoff takeoff_config: set, not_taken, waited_flaps",
        pass: "nothing in not_taken",
        check: Some(c_takeoff_config_taken),
    },
    Item {
        id: "seat_belt_signs",
        sortie: Sortie::Airliner,
        what: "Seat belt signs on before anything turns, off through 10,000, on again for the arrival",
        shipped: "2026-09-24 8dddc5b",
        since: S0924,
        airframe: None,
        fly: "737 or A330 full auto through 10,000 ft both ways",
        evidence: "cabin seat_belt_signs: on, ok",
        pass: "every switching took, and an off is followed by an on",
        check: Some(c_seat_belts),
    },
    Item {
        id: "cabin_air",
        sortie: Sortie::Airliner,
        what: "Bleed and crew oxygen on for every full-auto mission (EDLP-LOWI hypoxia)",
        shipped: "2026-09-24 8dddc5b",
        since: S0924,
        airframe: None,
        fly: "SF50 (the only profile with cabin_air) full auto above FL100",
        evidence: "full_auto cabin_air: bleed, oxygen, changed; cabin_air valve_sync",
        pass: "bleed and oxygen both read on",
        check: Some(c_cabin_air),
    },
    Item {
        id: "ils_tuned",
        sortie: Sortie::Airliner,
        what: "NAV1 tuned to the ILS and its needles cross-checked on every landing",
        shipped: "2026-09-25 f30f1f0",
        since: S0925D,
        airframe: None,
        fly: "any full-auto arrival to an ILS runway",
        evidence: "ils tuned: ok, freq_mhz, course; ils xcheck: hdef, vdef",
        pass: "every tune read back",
        check: Some(c_ils_tuned),
    },
    Item {
        id: "ils_coupled_737",
        sortie: Sortie::Airliner,
        what: "The 737 flies the ILS coupled through its own APP button (app_press, app_status)",
        shipped: "2026-09-26 18627ec",
        since: S0926L,
        airframe: B738,
        fly: "737 autoland on an ILS runway",
        evidence: "ils coupling: from, to, appr_reads, dist",
        pass: "a coupling reaches Coupled, none Abandoned",
        check: Some(c_ils_coupled),
    },
    Item {
        id: "ils_setup",
        sortie: Sortie::HandFlown,
        what: "\"Set up the ILS\" as a pilot monitoring: tuned, APPR from off, needles called, missed altitude",
        shipped: "2026-09-25 5d5bf46",
        since: S0925E,
        airframe: None,
        fly: "737 or 172 hand-flown arrival: \"set up the ILS\", then \"couple the approach\"",
        evidence: "ils setup: rwy, freq_mhz, dh_set, arm, verdict; ils appr_pressed; ils watch: said; ils missed_alt_set: alt",
        pass: "a callout, APPR press or missed altitude follows every setup within 10 min",
        check: Some(c_ils_setup),
    },
    Item {
        id: "late_go_around",
        sortie: Sortie::Airliner,
        what: "A late high approach inside the FAP goes around straight ahead before it repositions",
        shipped: "2026-09-26 3b29f25",
        since: S0926I,
        airframe: None,
        fly: "737, autoland armed high inside the FAP (LEMG 13 at 2.6 nm)",
        evidence: "autoland late_go_around: dist, radio, above_path_ft",
        pass: "the go-around is flown with no crash after it",
        check: Some(c_late_go_around),
    },
    Item {
        id: "map_flaps_on_speed",
        sortie: Sortie::Airliner,
        what: "The final to a runway MAP puts the flaps out on speed as well as by distance",
        shipped: "2026-09-26 e4a196f",
        since: S0926G,
        airframe: None,
        fly: "737 full auto onto an RNAV approach whose MAP is the runway",
        evidence: "procedure map_config: flap, ias; callout airspeed_caution, airspeed_warning",
        pass: "no low-speed callout in the 5 min after the first map_config",
        check: Some(c_map_flaps_on_speed),
    },
    Item {
        id: "a330_transponder_knob",
        sortie: Sortie::Airliner,
        what: "The A330's transponder knobs are stepped by command and confirmed from the output",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: A330,
        fly: "A330 full-auto departure (EGPF circuit is the reference)",
        evidence: "autotakeoff transponder_alt: ok",
        pass: "every ALT set confirmed",
        check: Some(c_transponder_alt),
    },
    Item {
        id: "a330_spoilers_disarmed",
        sortie: Sortie::Airliner,
        what: "Ground spoilers armed for the roll are disarmed after the gear",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: A330,
        fly: "A330 full-auto departure",
        evidence: "autotakeoff spoilers_disarmed: ok; gear_up",
        pass: "one disarm per gear retraction, every one taken",
        check: Some(c_a330_spoilers_disarmed),
    },
    Item {
        id: "a330_runway_start",
        sortie: Sortie::Airliner,
        what: "A runway start still calls ready for departure and waits for the clearance",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: A330,
        fly: "A330 spawned on the numbers, full auto",
        evidence: "full_auto request_departure, readback_takeoff; atc party_line",
        pass: "the call and the clearance precede the roll (read the tower states Parked, LinedUp, TakeoffCleared)",
        check: None,
    },
    Item {
        id: "a330_alt_capture",
        sortie: Sortie::Airliner,
        what: "The autopilot's own capture holds the circuit's level legs",
        shipped: "2026-09-21 3950a94",
        since: S0921,
        airframe: A330,
        fly: "A330 circuit",
        evidence: "circuit leg: ap_level, vs_on; circuit vs_refused",
        pass: "ap_level true on the level legs, no vs_refused",
        check: Some(c_circuit_alt_capture),
    },
    Item {
        id: "systems_survey",
        sortie: Sortie::Airliner,
        what: "The systems survey at detection knows each autopilot",
        shipped: "2026-09-25 5d5bf46",
        since: S0925E,
        airframe: None,
        fly: "any airframe swap (172, 737, A330)",
        evidence: "systems: profile, ap_type, glass_hsi, tcas, auto_pressurization",
        pass: "every survey names an autopilot type",
        check: Some(c_systems),
    },
    Item {
        id: "self_pick_destination",
        sortie: Sortie::Airliner,
        what: "\"Land the aircraft\" prefers the plan's destination",
        shipped: "2026-09-25 ee493cc",
        since: S0926A,
        airframe: None,
        fly: "any, autoland asked near the destination with a plan loaded",
        evidence: "autoland self_pick: destination, rwy, min_length_m",
        pass: "the picked runway is at the destination",
        check: Some(c_self_pick_destination),
    },
    // --- Light aircraft circuit or hop.
    Item {
        id: "xpdr_alt_at_roll",
        sortie: Sortie::Circuit,
        what: "The transponder goes to ALT as the roll starts",
        shipped: "2026-09-24 8dddc5b",
        since: S0924,
        airframe: None,
        fly: "any full-auto or spoken takeoff",
        evidence: "autotakeoff transponder_alt: ok",
        pass: "every roll confirmed ALT",
        check: Some(c_transponder_alt),
    },
    Item {
        id: "leg_stall_per_leg",
        sortie: Sortie::Circuit,
        what: "The follower counts stalls per leg with a turn grace (LIPK approach stand-down)",
        shipped: "2026-09-25 ee493cc",
        since: S0926A,
        airframe: None,
        fly: "172 full auto with an approach that turns more than 90 degrees",
        evidence: "procedure leg_stalled: idx, stalls; procedure stand_down: reason",
        pass: "no stand_down for leg_stalled",
        check: Some(c_leg_stall),
    },
    Item {
        id: "heading_set_once",
        sortie: Sortie::Circuit,
        what: "The follower sets the heading once per leg like a pilot, no flip-flop",
        shipped: "2026-09-26 327d121",
        since: S0926B,
        airframe: None,
        fly: "any full-auto enroute leg",
        evidence: "procedure heading_set: idx, hdg",
        pass: "no three sets on one leg inside 30 s that go back and forth",
        check: Some(c_heading_once),
    },
    Item {
        id: "circuit_short_way",
        sortie: Sortie::Circuit,
        what: "Circuit turns go the short way (bug entry lead 25 deg)",
        shipped: "2026-09-23 f1d6791",
        since: S0923,
        airframe: None,
        fly: "172 or SF50 rectangle circuit",
        evidence: "circuit leg: leg, psi, bug, tgt",
        pass: "no leg turns more than 200 degrees net",
        check: Some(c_circuit_short_way),
    },
    Item {
        id: "sf50_gear_warning",
        sortie: Sortie::Circuit,
        what: "The SF50's gear comes up outside its warning envelope",
        shipped: "2026-09-24 8dddc5b",
        since: S0924,
        airframe: SF50,
        fly: "SF50 full-auto takeoff",
        evidence: "autotakeoff gear_up: agl, ias, flap, at_handoff",
        pass: "gear up at 128 kt or more, or 1,000 ft, or at the handoff",
        check: Some(c_sf50_gear),
    },
    Item {
        id: "sf50_flare",
        sortie: Sortie::Circuit,
        what: "The SF50's flare gains (read off the EDLP 24 trace)",
        shipped: "2026-09-24 8dddc5b",
        since: S0924,
        airframe: SF50,
        fly: "SF50 autoland",
        evidence: "scorecard landing stage (sink_fpm); autoland flare_elev, vvi",
        pass: "touchdown at or under the scorecard's 400 fpm",
        check: Some(c_touchdown_sink),
    },
    Item {
        id: "committed_flare",
        sortie: Sortie::Circuit,
        what: "A low flare or balloon with the airframe's landing distance ahead is committed",
        shipped: "2026-09-24 8dddc5b",
        since: S0924,
        airframe: None,
        fly: "SF50 autoland (a float past the allowance)",
        evidence: "autoland committed_flare: radio, remaining_m; flight touchdown: sink",
        pass: "a touchdown within 60 s and no crash",
        check: Some(c_committed_flare),
    },
    Item {
        id: "stand_shutdown",
        sortie: Sortie::Circuit,
        what: "A mission ends shut down at the stand, engines and electrics off",
        shipped: "2026-09-24 8dddc5b",
        since: S0924,
        airframe: None,
        fly: "any full auto to the stand",
        evidence: "taxi_in shutdown: tool, stopped; taxi_in electrics_off: line",
        pass: "engines stopped and the electrics line names no exception",
        check: Some(c_stand_shutdown),
    },
    Item {
        id: "vacate_stand_side",
        sortie: Sortie::Circuit,
        what: "The vacate turns toward the parking and never onto another runway (EBLG)",
        shipped: "2026-09-22 125e2b3",
        since: S0922,
        airframe: None,
        fly: "737 into a field with parallels (EBLG 04R)",
        evidence: "taxi_in vacating: rwy, exit; taxi_in vacated: arrived; autotaxi off_route",
        pass: "vacated and arrived with no off_route stop on the taxi in (the side is judged by eye)",
        check: Some(c_vacate_clean),
    },
    Item {
        id: "untowered_departure",
        sortie: Sortie::Circuit,
        what: "An out-of-reach station leaves full auto to plan its own taxi (LILB)",
        shipped: "2026-09-25 389728a",
        since: S0925B,
        airframe: None,
        fly: "any, full auto from an untowered field",
        evidence: "full_auto departure_field: towered; full_auto lined_up",
        pass: "an untowered departure reaches the line-up",
        check: Some(c_untowered),
    },
    Item {
        id: "margin_waiver",
        sortie: Sortie::Circuit,
        what: "\"Fly it anyway\" waives a margin gate and re-arms",
        shipped: "2026-09-25 389728a",
        since: S0925B,
        airframe: None,
        fly: "any, a plan refused on the runway factor or the plan weight",
        evidence: "full_auto waiver: gate",
        pass: "a takeoff follows the waiver",
        check: Some(c_waiver),
    },
    Item {
        id: "touchdown_idle",
        sortie: Sortie::Circuit,
        what: "A flight speed hold on the runway closes the throttle and lets go",
        shipped: "2026-09-25 f30f1f0",
        since: S0925D,
        airframe: None,
        fly: "any, a speed hold still armed at the touchdown",
        evidence: "speedhold touchdown_idle: ias, target",
        pass: "the hold idles on the runway",
        check: Some(c_touchdown_idle),
    },
    Item {
        id: "evolution_taxi",
        sortie: Sortie::Circuit,
        what: "The Evolution taxis on a capped bend lookahead and smoothed brakes",
        shipped: "2026-09-25 f30f1f0",
        since: S0925D,
        airframe: Some("lancair_evolution"),
        fly: "Lancair Evolution full-auto taxi",
        evidence: "scorecard taxi stage from autotaxi drive ticks",
        pass: "every taxi stage passes the scorecard",
        check: Some(c_taxi_stage),
    },
    Item {
        id: "sr22_speed_refused",
        sortie: Sortie::Circuit,
        what: "A misheard spoken speed past the ceiling or under the floor is refused",
        shipped: "2026-09-24 5f8a38c",
        since: S0924S,
        airframe: None,
        fly: "SR22, ask for an impossible speed",
        evidence: "speedhold spoken_target_refused: asked",
        pass: "the refusal fires",
        check: Some(c_spoken_refused),
    },
    Item {
        id: "sr22_lineup_pivot",
        sortie: Sortie::Circuit,
        what: "The line-up pivots on the brake for a big heading error under 7 kt",
        shipped: "2026-09-24 5f8a38c",
        since: S0924S,
        airframe: Some("cirrus_sr22"),
        fly: "SR22 full-auto line-up at a turn of more than 60 degrees",
        evidence: "autotaxi drive: bs, bl, br in the slow turn",
        pass: "the inside brake leads and the line-up ends aligned (read the ticks)",
        check: None,
    },
    Item {
        id: "lfpb_strip",
        sortie: Sortie::Circuit,
        what: "The taxi route ends on the cleared strip where two thresholds share an end",
        shipped: "2026-09-22 125e2b3",
        since: S0922,
        airframe: None,
        fly: "737, full-auto departure from LFPB 07 (or any of the 1,815 such fields)",
        evidence: "full_auto line_up_no_fit: why; full_auto lined_up",
        pass: "lined up with no line_up_no_fit at a field whose ends converge",
        check: None,
    },
    // --- Helicopter route.
    Item {
        id: "heli_approach",
        sortie: Sortie::Helicopter,
        what: "The S-76 comes down at speed to a gate, then the hover loop's final",
        shipped: "2026-09-23 f1d6791",
        since: S0923,
        airframe: S76,
        fly: "S-76 route into a lowland pad",
        evidence: "heli_route approach, descent_vs, slow (approach true), handover_hover: ias, to_pad_m; arrived: on_ground",
        pass: "the hover loop takes over at 45 kt or less and it lands on the pad, no crash",
        check: Some(c_heli_approach),
    },
    Item {
        id: "heli_inbound",
        sortie: Sortie::Helicopter,
        what: "The helicopter's inbound call waits for the destination's station",
        shipped: "2026-09-23 f1d6791",
        since: S0923,
        airframe: S76,
        fly: "S-76 route into a towered field",
        evidence: "heli_route inbound_call: khz, station_swapped",
        pass: "station_swapped true",
        check: Some(c_heli_inbound),
    },
    Item {
        id: "heli_radio",
        sortie: Sortie::Helicopter,
        what: "The helicopter works the radio with the aeroplanes' code (zone clearance, handoffs)",
        shipped: "2026-09-23 f1d6791",
        since: S0923,
        airframe: S76,
        fly: "S-76 route out of a towered field",
        evidence: "heli_route zone_clearance, departure_call, departure_handoff, region_handoff",
        pass: "a zone clearance before every departure call",
        check: Some(c_heli_radio),
    },
    Item {
        id: "heli_route_terrain",
        sortie: Sortie::Helicopter,
        what: "Climb gate, corridor floors, fast climb and a collective floor on the approach (LILO-LILB crashes)",
        shipped: "2026-09-24 b30865d",
        since: S0925A,
        airframe: S76,
        fly: "S-76 route through a valley, lowland arrival first",
        evidence: "heli_route start: cruise_alt_ft, terrain_ft; gate_lowered; arrived: on_ground; crash",
        pass: "every started route lands, none crashes",
        check: Some(c_heli_route_whole),
    },
    Item {
        id: "heli_pad_shutdown",
        sortie: Sortie::Helicopter,
        what: "A helipad landing ends there, shut down",
        shipped: "2026-09-24 858a0d4",
        since: S0924L,
        airframe: S76,
        fly: "S-76 route to a helipad",
        evidence: "heli_route pad_shutdown; taxi_in shutdown: stopped",
        pass: "the engines stop on the pad",
        check: Some(c_heli_pad_shutdown),
    },
    Item {
        id: "hover_hand_back",
        sortie: Sortie::Helicopter,
        what: "In the air the autopilot is never simply turned off",
        shipped: "2026-09-23 058c6ff",
        since: S0923H,
        airframe: S76,
        fly: "S-76, any hover-loop hand-back",
        evidence: "hover hand_back: why, cruise_modes, hover_mode",
        pass: "every hand-back leaves cruise modes or the hover mode on",
        check: Some(c_hover_hand_back),
    },
    // --- Ramp.
    Item {
        id: "tug_unhook",
        sortie: Sortie::Ramp,
        what: "The toe brakes hold until the tug is off the bar, then the parking brake",
        shipped: "2026-09-26 327d121",
        since: S0926B,
        airframe: None,
        fly: "737 pushback from a stand",
        evidence: "pushback brakes_set: tug_clear; pushback tug_not_clear",
        pass: "the tug shows clear on every push",
        check: Some(c_tug_unhook),
    },
    Item {
        id: "push_lane",
        sortie: Sortie::Ramp,
        what: "The push follows the stand's painted lead-in and ends at the planned distance",
        shipped: "2026-09-26 3226e77",
        since: S0926E,
        airframe: None,
        fly: "737 pushback from a stand with a drawn lead-in",
        evidence: "full_auto push_lane: back_m, target_hdg; pushback stopped: back_m",
        pass: "pushed within 10 m of the planned distance",
        check: Some(c_push_lane),
    },
    Item {
        id: "hand_back_brakes",
        sortie: Sortie::Ramp,
        what: "The hand-back's parking brake is set on an unscoped client (\"automation cancelled\")",
        shipped: "2026-09-26 a8730cb",
        since: S0926D,
        airframe: None,
        fly: "any full auto that gives up on the ground",
        evidence: "full_auto hand_back_brakes: held, tries, err",
        pass: "held true",
        check: Some(c_hand_back_brakes),
    },
    Item {
        id: "citation_cold_start",
        sortie: Sortie::Ramp,
        what: "The Citation X starts from a cold APU (loaded at 99, bleed read back)",
        shipped: "2026-09-23 b3091d8",
        since: S0923H,
        airframe: Some("citation_x"),
        fly: "Citation X cold and dark: \"engines on\"",
        evidence: "turbine_start apu, apu_loaded: air, volts; turbine_start crank: engine, verdict, peak_spool",
        pass: "every engine's crank verdict is Lit",
        check: Some(c_cold_start),
    },
    Item {
        id: "cabin_open_line",
        sortie: Sortie::Ramp,
        what: "A bare \"cabin crew?\" answers go ahead and opens the line",
        shipped: "2026-09-22 125e2b3",
        since: S0922,
        airframe: None,
        fly: "737 or A330 on the ramp: \"cabin crew?\", then a cabin call",
        evidence: "cabin (route call): call, reply",
        pass: "the reply is \"Go ahead, flight deck.\"",
        check: Some(c_cabin_open_line),
    },
    Item {
        id: "cabin_voice_picker",
        sortie: Sortie::Ramp,
        what: "The panel's cabin voice picker",
        shipped: "2026-09-22 125e2b3",
        since: S0922,
        airframe: None,
        fly: "reload the plugin with status/auto empty, restart xvl, pick a cabin voice",
        evidence: "Health has no missing xcopilot/opt/cabin_voice; the new voice checks in",
        pass: "\"Flight deck, cabin crew. New voice, checked in.\" in the picked voice",
        check: None,
    },
    Item {
        id: "cabin_watch",
        sortie: Sortie::Ramp,
        what: "The cabin works the flight unasked and never calls a departure after a landing (EBLG)",
        shipped: "2026-09-22 125e2b3",
        since: S0922,
        airframe: None,
        fly: "any airliner leg to the stand",
        evidence: "cabin (route watch) securing_for_departure, secure_at_liftoff, securing_for_landing, deplaning; flight touchdown, flight_complete",
        pass: "no securing_for_departure between a touchdown and the flight's end",
        check: Some(c_cabin_after_landing),
    },
    Item {
        id: "voice_handover",
        sortie: Sortie::Ramp,
        what: "\"You have control\" is a hand-over from the flight model, the recovery only for an upset",
        shipped: "2026-09-25 ee493cc",
        since: S0926A,
        airframe: None,
        fly: "any phase: \"you have control\", then \"I'll take it\"",
        evidence: "handover decide: took, why, phase, upset; recover",
        pass: "no upset recovery runs after a hand-over without an upset",
        check: Some(c_handover),
    },
    // --- Hand-flown.
    Item {
        id: "rapport_landing_word",
        sortie: Sortie::HandFlown,
        what: "The landing word: praise for the pilot only, a hard landing always said",
        shipped: "2026-09-23 f1d6791",
        since: S0923,
        airframe: None,
        fly: "any landing, pilot and automation",
        evidence: "rapport touchdown: sink_fpm, by_automation; rapport said: line",
        pass: "no praise after an automation's landing, \"hard\" said at 600 fpm or more",
        check: Some(c_rapport),
    },
    Item {
        id: "say_again_repeat",
        sortie: Sortie::HandFlown,
        what: "The frequency's \"say again\" repeats the controller's last line",
        shipped: "2026-09-23 f1d6791",
        since: S0923,
        airframe: None,
        fly: "any towered field: key over a clearance, then \"say again\" on the radio",
        evidence: "atc (outcome say_again_repeat): reply, age_s, service",
        pass: "the repeat is of a line under 60 s old",
        check: Some(c_say_again),
    },
    Item {
        id: "recall_cut_lines",
        sortie: Sortie::HandFlown,
        what: "\"What were you saying?\" says the lines a press cut",
        shipped: "2026-09-23 f1d6791",
        since: S0923,
        airframe: None,
        fly: "any: key the intercom over a copilot line, then ask",
        evidence: "said records around the press; the xvl log's recall line",
        pass: "the cut line is said again",
        check: None,
    },
    Item {
        id: "tempo_persona",
        sortie: Sortie::HandFlown,
        what: "Reaction pause, \"stand by\", line pace and small talk",
        shipped: "2026-09-23 f1d6791",
        since: S0923,
        airframe: None,
        fly: "any hand-flown session with some chat",
        evidence: "said: text, prio (no tempo event is recorded)",
        pass: "the pilot says it sounds like a person",
        check: None,
    },
    Item {
        id: "crew_speech_trims",
        sortie: Sortie::HandFlown,
        what: "Fewer pre-announcements, narration perishes at 6 s, one take-back hint a session",
        shipped: "2026-09-26 327d121",
        since: S0926B,
        airframe: None,
        fly: "any full auto from cold",
        evidence: "said lines against the xvl log's \"dropped stale\" count",
        pass: "no backlog of stale lines (about 20 dropped before)",
        check: None,
    },
    Item {
        id: "duty_engine",
        sortie: Sortie::HandFlown,
        what: "The duty engine (shadow) agrees with the flights flown",
        shipped: "2026-09-25 cadf9f8",
        since: S0925C,
        airframe: None,
        fly: "any flight",
        evidence: "duty: duty, phase, why, agrees",
        pass: "no disagreement with the running automation",
        check: Some(c_duty),
    },
    Item {
        id: "flight_model_touchdowns",
        sortie: Sortie::HandFlown,
        what: "The flight model sees every landing",
        shipped: "2026-09-25 389728a",
        since: S0925B,
        airframe: None,
        fly: "any landing",
        evidence: "flight touchdown: sink, gs; rapport touchdown",
        pass: "a flight touchdown within 5 s of every rapport touchdown",
        check: Some(c_flight_model_touchdowns),
    },
    Item {
        id: "mountain_map_handover",
        sortie: Sortie::HandFlown,
        what: "A mountain field's MAP approach is flown to the MAP and handed over with the talk-down",
        shipped: "2026-09-24 858a0d4",
        since: S0924L,
        airframe: None,
        fly: "LSZS RNAV 03 in the SR22 or SF50, full auto",
        evidence: "procedure map_config, visual_segment; autoland glidepath",
        pass: "handed over at the MAP on a flyable visual segment, the talk-down heard",
        check: None,
    },
];

/// The verdict for one item over a log.
pub fn judge(item: &Item, log: &Log) -> Option<Verdict> {
    let check = item.check?;
    Some(check(&log.window(item.since, item.airframe)))
}

/// The card, judged over every record in the file: failures first, then the
/// verified, then what the log has not exercised, then the rows for a person.
pub fn report(events: &[&Value]) -> String {
    let log = Log::new(events);
    let mut failed = Vec::new();
    let mut verified = Vec::new();
    let mut idle: Vec<&Item> = Vec::new();
    let mut by_hand = Vec::new();
    for item in ITEMS {
        match judge(item, &log) {
            Some(Verdict::Failed(d)) => failed.push(format!("  FAILED    {}: {d}", item.id)),
            Some(Verdict::Verified(d)) => verified.push(format!("  verified  {}: {d}", item.id)),
            Some(Verdict::NotExercised) => idle.push(item),
            None => by_hand.push(item.id),
        }
    }
    let mut out = format!(
        "Flight-test card ({} items over the whole file): {} verified, {} failed, {} not exercised, {} by hand.\n",
        ITEMS.len(),
        verified.len(),
        failed.len(),
        idle.len(),
        by_hand.len()
    );
    for l in failed.iter().chain(verified.iter()) {
        out.push_str(l);
        out.push('\n');
    }
    // What is left, by the sortie that would exercise it.
    for s in Sortie::ALL {
        let ids: Vec<&str> = idle
            .iter()
            .filter(|i| i.sortie == s)
            .map(|i| i.id)
            .collect();
        if !ids.is_empty() {
            out.push_str(&format!(
                "  not exercised, {}: {}\n",
                s.title(),
                ids.join(", ")
            ));
        }
    }
    if !by_hand.is_empty() {
        out.push_str(&format!(
            "  by hand (docs/flight-test-card.md): {}\n",
            by_hand.join(", ")
        ));
    }
    out
}

/// The card's fixed columns for one item, as `docs/flight-test-card.md` has
/// them: item, shipped, what to fly, evidence, pass criterion. The status
/// column is the pilot's (and the debrief's), so it is not generated.
#[cfg(test)]
pub fn row_cells(item: &Item) -> [String; 5] {
    [
        format!("`{}` {}", item.id, item.what),
        item.shipped.to_string(),
        item.fly.to_string(),
        item.evidence.to_string(),
        format!(
            "{}{}",
            item.pass,
            if item.check.is_none() {
                " (by hand)"
            } else {
                ""
            }
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn log_of(v: &[Value]) -> Vec<&Value> {
        v.iter().collect()
    }

    fn judged(id: &str, recs: &[Value]) -> Verdict {
        let refs = log_of(recs);
        let log = Log::new(&refs);
        let item = ITEMS.iter().find(|i| i.id == id).unwrap();
        judge(item, &log).unwrap()
    }

    const T: f64 = 1_790_430_000.0; // 2026-09-26 13:40Z, after every S0926 up to E

    /// The LIBP-LEMG day in miniature: the numbers the card reported on the
    /// real log, so a change to a predicate that changes them fails here.
    #[test]
    fn the_card_reads_the_numbers_off_a_sortie() {
        let recs = vec![
            json!({"kind":"flight_meta","aircraft":"boeing_737_800","t":T}),
            json!({"kind":"autotakeoff","event":"takeoff_config","set":["flaps","trim","autobrake max"],"not_taken":[],"waited_flaps":false,"t":T+1.0}),
            json!({"kind":"autotakeoff","event":"climb_out","accel_agl_ft":1000.0,"max_fpm":3000.0,"hold_kts":170.0,"t":T+2.0}),
            json!({"kind":"autotakeoff","event":"roll","gs":60.0,"xtk_m":1.0,"trim":0.01,"t":T+3.0}),
            json!({"kind":"autotakeoff","event":"roll","gs":120.0,"xtk_m":9.9,"trim":0.124,"t":T+4.0}),
            json!({"kind":"autotakeoff","event":"flap_step","agl_ft":1001.9,"ias":190.9,"to":0.25,"t":T+60.0}),
            json!({"kind":"autotakeoff","event":"flap_step","agl_ft":1374.2,"ias":210.2,"to":0.0,"t":T+70.0}),
            json!({"kind":"autotakeoff","event":"handoff","agl":1374.0,"t":T+70.0}),
            json!({"kind":"pushback","event":"brakes_set","tug_clear":false,"t":T+80.0}),
            json!({"kind":"taxi_in","event":"shutdown","tool":"stop_engines","stopped":true,"t":T+100.0}),
            json!({"kind":"taxi_in","event":"electrics_off","line":"All electrical systems off, except: The battery didn't move.","t":T+100.0}),
            json!({"kind":"full_auto","stage":"altimeter_set","detail":{"inhg":30.18,"took":true,"want":30.18,"was":30.21},"t":T+101.0}),
            json!({"kind":"duty","duty":"taxi_out","agrees":false,"t":T+102.0}),
            json!({"kind":"duty","duty":"hold","agrees":true,"t":T+103.0}),
        ];
        assert!(matches!(
            judged("b738_speedbrake_down", &recs),
            Verdict::Verified(_)
        ));
        match judged("jet_climb_out", &recs) {
            Verdict::Verified(d) => assert!(d.contains("1002, 1374 ft against 1000"), "{d}"),
            v => panic!("{v:?}"),
        }
        match judged("roll_trim_scaled", &recs) {
            Verdict::Failed(d) => assert!(d.contains("9.9 m off at worst, trim 0.12"), "{d}"),
            v => panic!("{v:?}"),
        }
        assert!(matches!(judged("tug_unhook", &recs), Verdict::Failed(_)));
        match judged("stand_shutdown", &recs) {
            Verdict::Failed(d) => assert!(d.contains("battery didn't move"), "{d}"),
            v => panic!("{v:?}"),
        }
        assert!(matches!(
            judged("altimeter_rounds", &recs),
            Verdict::Verified(_)
        ));
        match judged("duty_engine", &recs) {
            Verdict::Failed(d) => assert!(d.starts_with("1 of 2"), "{d}"),
            v => panic!("{v:?}"),
        }
        // Nothing of the kind flown: not exercised, never a pass.
        assert_eq!(judged("heli_approach", &recs), Verdict::NotExercised);
        assert_eq!(judged("own_sid", &recs), Verdict::NotExercised);
    }

    /// Evidence from before the fix shipped is a flight of the old code, and
    /// another airframe's records are not this airframe's.
    #[test]
    fn old_code_and_other_airframes_are_not_evidence() {
        let recs = vec![
            json!({"kind":"flight_meta","aircraft":"cessna_172sp","t":T}),
            json!({"kind":"autotakeoff","event":"takeoff_config","set":["spoilers armed"],"not_taken":[],"t":T+1.0}),
            json!({"kind":"full_auto","stage":"hand_back_brakes","detail":{"held":false,"err":"automation cancelled","tries":3},"t":S0926D - 60.0}),
        ];
        assert_eq!(judged("b738_speedbrake_down", &recs), Verdict::NotExercised);
        assert_eq!(judged("hand_back_brakes", &recs), Verdict::NotExercised);
    }

    #[test]
    fn a_reversal_is_judged_by_how_it_ended() {
        let recs = vec![
            json!({"kind":"autoland","event":"reversal","side":1.0,"t":T}),
            json!({"kind":"autoland","event":"reversal_done","xtk_m":57.9,"dist":5.5,"t":T+480.0}),
            json!({"kind":"autoland","event":"reversal","side":-1.0,"t":T+1000.0}),
            json!({"kind":"flight","ev":"pilot_took_over","t":T+1014.0}),
        ];
        match judged("reversal_pattern", &recs) {
            Verdict::Verified(d) => {
                assert!(d.contains("done 58 m off at 5.5 nm"), "{d}");
                assert!(d.contains("1 more interrupted"), "{d}");
            }
            v => panic!("{v:?}"),
        }
    }

    /// LEMG 13, 2026-09-26: `in.ap` read true through the whole rollout
    /// while the arrest had the servos out since 79 ft. The loop's own
    /// record is the evidence; a rollout with neither record fails.
    #[test]
    fn the_servos_are_judged_from_the_loop_not_autopilot_on() {
        let tick =
            |t: f64| json!({"kind":"autoland","event":"rollout","gs":120.0,"xtk_m":-3.0,"t":t});
        let state = |t: f64| json!({"kind":"state","in":{"ap":true},"t":t});
        let recs = vec![
            json!({"kind":"autoland","event":"arrest_hands","taken":true,"ok":true,"radio":79.2,"t":T}),
            tick(T + 12.0),
            state(T + 13.0),
            tick(T + 13.0),
            // A second landing, no arrest and no disconnect on file.
            tick(T + 600.0),
            state(T + 601.0),
        ];
        match judged("ap_off_at_wheels", &recs) {
            Verdict::Failed(d) => assert_eq!(d, "13:50: no record of the servos coming off"),
            v => panic!("{v:?}"),
        }
        match judged("ap_off_at_wheels", &recs[..4]) {
            Verdict::Verified(d) => assert_eq!(d, "13:40: servos out since the arrest at 79 ft"),
            v => panic!("{v:?}"),
        }
    }

    #[test]
    fn a_long_way_circuit_turn_fails() {
        let mut recs = vec![];
        let mut t = T;
        for psi in [0.0, 90.0, 180.0, 270.0, 0.0] {
            recs.push(json!({"kind":"circuit","event":"leg","leg":"Crosswind","psi":psi,"t":t}));
            t += 5.0;
        }
        match judged("circuit_short_way", &recs) {
            Verdict::Failed(d) => assert!(d.contains("Crosswind 360 deg"), "{d}"),
            v => panic!("{v:?}"),
        }
    }

    #[test]
    fn every_item_is_well_formed() {
        let mut ids: Vec<&str> = ITEMS.iter().map(|i| i.id).collect();
        ids.sort();
        let n = ids.len();
        ids.dedup();
        assert_eq!(ids.len(), n, "duplicate ids");
        for i in ITEMS {
            for cell in row_cells(i) {
                assert!(!cell.contains('|'), "a pipe breaks the table: {cell}");
                assert!(
                    !cell.contains('\u{2014}') && !cell.contains('\u{2013}'),
                    "no dashes the TTS and panel choke on: {cell}"
                );
            }
        }
    }

    /// `docs/flight-test-card.md` carries exactly these rows, each cell as
    /// the table has it; only the status column is free.
    #[test]
    fn the_doc_and_the_table_agree() {
        let doc = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/docs/flight-test-card.md"
        ))
        .expect("docs/flight-test-card.md");
        let mut seen = Vec::new();
        for line in doc.lines().filter(|l| l.starts_with("| `")) {
            let cells: Vec<String> = line
                .trim()
                .trim_matches('|')
                .split('|')
                .map(|c| c.trim().to_string())
                .collect();
            assert_eq!(cells.len(), 6, "six columns: {line}");
            let id = cells[0]
                .trim_start_matches('`')
                .split('`')
                .next()
                .unwrap()
                .to_string();
            let item = ITEMS
                .iter()
                .find(|i| i.id == id)
                .unwrap_or_else(|| panic!("row {id} is not in test_card::ITEMS"));
            let want = row_cells(item);
            for (k, w) in want.iter().enumerate() {
                assert_eq!(&cells[k], w, "row {id}, column {}", k + 1);
            }
            seen.push(id);
        }
        for i in ITEMS {
            assert!(
                seen.iter().any(|s| s == i.id),
                "{} has no row in the doc",
                i.id
            );
        }
        assert_eq!(seen.len(), ITEMS.len(), "a row appears twice");
    }

    /// Prints the rows for the doc (status left for the pilot):
    /// `cargo test test_card::tests::print_rows -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn print_rows() {
        for s in Sortie::ALL {
            println!("\n## {}\n", s.title());
            println!(
                "| item | shipped | what to fly | blackbox evidence | pass criterion | status |"
            );
            println!("|---|---|---|---|---|---|");
            for i in ITEMS.iter().filter(|i| i.sortie == s) {
                let c = row_cells(i);
                println!("| {} | {} | {} | {} | {} | |", c[0], c[1], c[2], c[3], c[4]);
            }
        }
    }
}
