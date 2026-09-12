//! `xvl debrief`: the flight, read back from the blackbox.
//!
//! Every automation and the monitor already write what they saw to
//! `logs/blackbox.jsonl`; the rule is "mine it after every flight", and until
//! now that meant a person with grep. This is the read a pilot gets after the
//! sortie and the one the fly-fix-fly loop starts from: what the aircraft was,
//! how the flight went by the numbers, and every event that would make an
//! instructor reach for a pen — crashes, balked landings, stalls, calls into
//! the void, targets the airframe floor had to catch, taxis that stopped.
//!
//! Pure over the lines, so it tests on a handful of synthetic events and runs
//! on the flights already on disk. Flights are split on a gap of
//! [`FLIGHT_GAP_SECS`] between events; the report is the LAST flight.

use serde_json::Value;
use std::collections::BTreeMap;

/// Silence this long between events is a new flight.
const FLIGHT_GAP_SECS: f64 = 1800.0;
/// Below this AGL a state sample is "on or near the ground" for the airborne
/// minimum-speed figure — the takeoff roll and the flare are not slow flight.
const AIRBORNE_AGL_FT: f64 = 100.0;
/// Most events listed in the narrative, so a long flight stays readable.
const MAX_NARRATIVE: usize = 60;

pub fn report<'a>(lines: impl Iterator<Item = &'a str>) -> String {
    let events: Vec<Value> = lines
        .filter_map(|l| serde_json::from_str::<Value>(l).ok())
        .filter(|v| v.get("t").and_then(Value::as_f64).is_some())
        .collect();
    if events.is_empty() {
        return "No blackbox events.\n".to_string();
    }
    // Split into flights on silence.
    let mut flights: Vec<Vec<&Value>> = vec![Vec::new()];
    let mut last_t = None::<f64>;
    for e in &events {
        let t = e["t"].as_f64().unwrap_or(0.0);
        if last_t.is_some_and(|lt| t - lt > FLIGHT_GAP_SECS) {
            flights.push(Vec::new());
        }
        flights.last_mut().unwrap().push(e);
        last_t = Some(t);
    }
    let n = flights.len();
    let flight = flights.last().unwrap();
    let mut out = String::new();
    out.push_str(&format!(
        "Flight {n} of {n} in the blackbox ({} events, {} to {}Z).\n",
        flight.len(),
        hms(flight[0]["t"].as_f64().unwrap_or(0.0)),
        hms(flight[flight.len() - 1]["t"].as_f64().unwrap_or(0.0)),
    ));
    out.push_str(&summarise(flight));
    out
}

fn summarise(flight: &[&Value]) -> String {
    let mut out = String::new();
    let s = |v: &Value, k: &str| v.get(k).and_then(Value::as_str).unwrap_or("").to_string();
    let f = |v: &Value, k: &str| v.get(k).and_then(Value::as_f64);
    let b = |v: &Value, k: &str| v.get(k).and_then(Value::as_bool);

    // --- Who and what.
    let aircraft: Vec<String> = flight
        .iter()
        .filter(|e| s(e, "kind") == "flight_meta")
        .map(|e| s(e, "aircraft"))
        .fold(Vec::new(), |mut acc, a| {
            if acc.last() != Some(&a) {
                acc.push(a);
            }
            acc
        });
    if !aircraft.is_empty() {
        out.push_str(&format!("Aircraft: {}\n", aircraft.join(" -> ")));
        if aircraft.iter().any(|a| a.starts_with("generic_")) {
            out.push_str("  NO PROFILE: flown on placards read from the aircraft file.\n");
        }
    }

    // --- The numbers, from the 1 Hz state.
    let states: Vec<&&Value> = flight.iter().filter(|e| s(e, "kind") == "state").collect();
    if !states.is_empty() {
        let max_alt = states.iter().filter_map(|e| f(e, "alt")).fold(f64::MIN, f64::max);
        let max_ias = states.iter().filter_map(|e| f(e, "ias")).fold(f64::MIN, f64::max);
        let airborne = states
            .iter()
            .filter(|e| b(e, "ground") == Some(false) && f(e, "agl").is_some_and(|a| a > AIRBORNE_AGL_FT));
        let min_ias = airborne.clone().filter_map(|e| f(e, "ias")).fold(f64::MAX, f64::min);
        let min_ias_at = airborne
            .clone()
            .filter(|e| f(e, "ias") == Some(min_ias))
            .map(|e| format!(" at {}Z, {:.0} ft AGL", hms(f(e, "t").unwrap_or(0.0)), f(e, "agl").unwrap_or(0.0)))
            .next()
            .unwrap_or_default();
        let flew = airborne.count() > 0;
        out.push_str(&format!(
            "Max altitude {:.0} ft, max IAS {:.0} kt{}.\n",
            max_alt,
            max_ias,
            if flew { format!(", min IAS airborne {min_ias:.0} kt{min_ias_at}") } else { String::new() }
        ));
        // Touchdowns: airborne -> ground transitions, with the sink rate.
        let mut prev_ground = None::<bool>;
        for e in &states {
            let g = b(e, "ground").unwrap_or(true);
            if prev_ground == Some(false) && g {
                out.push_str(&format!(
                    "Touchdown {}Z at {:.0} fpm, {:.0} kt.\n",
                    hms(f(e, "t").unwrap_or(0.0)),
                    f(e, "vsi").unwrap_or(0.0),
                    f(e, "ias").unwrap_or(0.0)
                ));
            }
            prev_ground = Some(g);
        }
        // Stops during an auto taxi: rolling to a halt while the drive was on.
        let mut driving = false;
        let mut rolling = false;
        let mut stops = 0u32;
        for e in flight {
            match (s(e, "kind").as_str(), s(e, "event").as_str()) {
                ("autotaxi", "engaged") => driving = true,
                ("autotaxi", "holding_point" | "off_route") => driving = false,
                ("state", _) if driving => {
                    let gs = f(e, "gs").unwrap_or(0.0);
                    if gs > 2.0 {
                        rolling = true;
                    } else if gs < 0.5 && rolling {
                        rolling = false;
                        stops += 1;
                    }
                }
                _ => {}
            }
        }
        if stops > 0 {
            out.push_str(&format!("Auto taxi rolled to a stop {stops} time(s) before its holding point.\n"));
        }
    }

    // --- Callouts, by id.
    let mut callouts: BTreeMap<String, u32> = BTreeMap::new();
    for e in flight.iter().filter(|e| s(e, "kind") == "callout") {
        *callouts.entry(s(e, "id")).or_default() += 1;
    }
    if !callouts.is_empty() {
        let list: Vec<String> = callouts.iter().map(|(k, v)| format!("{k} x{v}")).collect();
        out.push_str(&format!("Callouts: {}\n", list.join(", ")));
    }
    let mishears = flight
        .iter()
        .filter(|e| s(e, "kind") == "heard" && f(e, "confidence").is_some_and(|c| c < 0.7))
        .count();
    if mishears > 0 {
        out.push_str(&format!("Utterances heard under 0.7 confidence: {mishears}\n"));
    }

    // --- The narrative: everything an instructor would write down.
    out.push_str("Events:\n");
    let mut lines = Vec::new();
    // Autoland's guidance ticks are per-tick records; the ARMING is the
    // event, read off the first tick of each run (a new runway, or a gap).
    let mut last_autoland: Option<(String, f64)> = None;
    for e in flight {
        let kind = s(e, "kind");
        let ev = s(e, "event");
        let stage = s(e, "stage");
        let t = hms(f(e, "t").unwrap_or(0.0));
        // An autoland TICK: the runway and no event name. Labelled autoland
        // events carry a runway too (the final report), and rendered as an
        // arming they read "13 from 5.0 nm, 0 ft radio, 0 kt, phase " (LEMG,
        // 2026-09-12).
        if kind == "autoland" && e.get("rwy").is_some() && e.get("event").is_none() {
            let rwy = s(e, "rwy");
            let now = f(e, "t").unwrap_or(0.0);
            let fresh = last_autoland
                .as_ref()
                .is_none_or(|(r, lt)| *r != rwy || now - lt > 60.0);
            if fresh {
                lines.push(format!(
                    "  {t}Z autoland armed: {rwy} from {:.1} nm, {} ft radio, {:.0} kt, phase {}",
                    f(e, "dist").unwrap_or(0.0),
                    f(e, "radio").unwrap_or(0.0).round(),
                    f(e, "ias").unwrap_or(0.0),
                    s(e, "phase")
                ));
            }
            last_autoland = Some((rwy, now));
            continue;
        }
        let line = match (kind.as_str(), ev.as_str()) {
            ("crash", _) => Some(format!("CRASH at {:.0} ft", f(e, "alt").unwrap_or(0.0))),
            ("full_auto", _) => Some(format!("full auto: {stage}{}", detail(e))),
            ("autoland", "going_around") => {
                let mut l = format!("GO-AROUND at {:.0} ft radio", f(e, "radio").unwrap_or(0.0));
                if let Some(why) = e.get("why").and_then(|v| v.as_str()) {
                    l.push_str(&format!(": {}", why.replace('_', " ")));
                }
                if let Some(x) = f(e, "xtk_m") {
                    l.push_str(&format!(", {:.0} m {} of the centreline", x.abs(), if x < 0.0 { "left" } else { "right" }));
                }
                Some(l)
            }
            ("autoland", "final_report") => Some(format!(
                "final report: runway {}, {:.1} nm",
                s(e, "rwy"),
                f(e, "dist").unwrap_or(0.0)
            )),
            ("autoland", "ordered_go_around") => Some("go-around ORDERED by the tower".into()),
            ("autoland", "give_up") => Some(format!("autoland gave up: {}", s(e, "reason"))),
            ("autoland", "approach_ias_floored") => Some(format!(
                "autoland approach speed {:.0} raised to the airframe floor {:.0}",
                f(e, "asked").unwrap_or(0.0),
                f(e, "floor").unwrap_or(0.0)
            )),
            ("autoland", "rolling_handoff") => Some("landed, rolling handoff to the taxi-in".into()),
            ("autoland", "forced_runway_missing") => Some(format!(
                "autoland could not find the planned runway {}, self-picked by track",
                s(e, "id")
            )),
            ("autoland", "runway_from_clearance") => Some(format!(
                "autoland took the CLEARED runway {} over the planned {}",
                s(e, "cleared"),
                s(e, "planned")
            )),
            ("speedhold", "target_floored") => Some(format!(
                "speed target {:.0} raised to the airframe floor {:.0}",
                f(e, "asked").unwrap_or(0.0),
                f(e, "floor").unwrap_or(0.0)
            )),
            ("procedure", "armed") => Some(format!("procedure: {}", s(e, "name"))),
            ("procedure", "handoff_autoland" | "complete_enroute" | "complete_no_handoff" | "stand_down") => {
                Some(format!("procedure: {ev}{}", if ev == "stand_down" { format!(" ({})", s(e, "reason")) } else { String::new() }))
            }
            ("autotaxi", "engaged" | "off_route" | "holding_point" | "join") => Some(format!("auto taxi: {ev}{}", detail(e))),
            ("pushback", "engaged" | "no_tug" | "hooked") => Some(format!("pushback: {ev}")),
            ("taxi_in", _) => Some(format!("taxi in: {ev}")),
            ("atc", _) if s(e, "outcome") == "void" => Some(format!("radio call into the VOID: \"{}\"", s(e, "transcript"))),
            ("atc", _) if s(e, "route") == "compliance" => Some(format!("ATC compliance: {}", s(e, "reply"))),
            ("atc", _) if s(e, "route") == "hold_release" => Some(format!("ATC: {}", s(e, "reply"))),
            ("atc_handoff", _) => Some(format!("ATC handoff: {}", s(e, "msg"))),
            ("atc_region", _) => Some(format!("ATC region: {}", s(e, "msg"))),
            ("recover", _) => None, // per tick; counted below
            _ => None,
        };
        if let Some(l) = line {
            lines.push(format!("  {t}Z {l}"));
        }
    }
    let recovers = flight.iter().filter(|e| s(e, "kind") == "recover").count();
    if recovers > 0 {
        lines.push(format!("  (recover tool ran for {recovers} ticks)"));
    }
    if lines.is_empty() {
        out.push_str("  (none of note)\n");
    } else {
        let extra = lines.len().saturating_sub(MAX_NARRATIVE);
        for l in lines.iter().take(MAX_NARRATIVE) {
            out.push_str(l);
            out.push('\n');
        }
        if extra > 0 {
            out.push_str(&format!("  ... and {extra} more\n"));
        }
    }
    out
}

/// A full-auto stage's or a taxi event's detail object, flattened.
fn detail(e: &Value) -> String {
    let d = match e.get("detail") {
        Some(Value::Object(m)) if !m.is_empty() => m,
        _ => return String::new(),
    };
    let parts: Vec<String> = d
        .iter()
        .map(|(k, v)| match v {
            Value::Number(n) => format!("{k}={}", n.as_f64().map(|x| format!("{x:.1}")).unwrap_or_default()),
            Value::String(s) => format!("{k}={s}"),
            other => format!("{k}={other}"),
        })
        .collect();
    format!(" ({})", parts.join(", "))
}

/// Unix seconds → HH:MM:SS (UTC).
fn hms(t: f64) -> String {
    let secs = t.max(0.0) as u64 % 86_400;
    format!("{:02}:{:02}:{:02}", secs / 3600, (secs / 60) % 60, secs % 60)
}

#[cfg(test)]
mod tests {
    use super::report;

    // The Gibraltar and RV-10 sorties, in miniature: the report has to say
    // the things a person spent two hours finding by hand (2026-09-11).
    #[test]
    fn the_debrief_names_what_went_wrong() {
        let lines = [
            r#"{"kind":"flight_meta","aircraft":"generic_rv10","t":1000.0}"#,
            r#"{"kind":"state","t":1001.0,"alt":30.0,"agl":0.0,"ias":0.0,"gs":0.0,"ground":true,"vsi":0.0}"#,
            r#"{"kind":"autotaxi","event":"engaged","t":1002.0}"#,
            r#"{"kind":"state","t":1003.0,"alt":30.0,"agl":0.0,"ias":5.0,"gs":7.0,"ground":true,"vsi":0.0}"#,
            r#"{"kind":"state","t":1004.0,"alt":30.0,"agl":0.0,"ias":0.0,"gs":0.1,"ground":true,"vsi":0.0}"#,
            r#"{"kind":"state","t":1005.0,"alt":30.0,"agl":0.0,"ias":5.0,"gs":7.0,"ground":true,"vsi":0.0}"#,
            r#"{"kind":"autotaxi","event":"holding_point","t":1006.0}"#,
            r#"{"kind":"full_auto","stage":"airborne","detail":{"agl_ft":1019.6},"t":1100.0}"#,
            r#"{"kind":"state","t":1200.0,"alt":3900.0,"agl":3800.0,"ias":180.0,"gs":200.0,"ground":false,"vsi":0.0}"#,
            r#"{"kind":"atc","route":"keyed","outcome":"void","transcript":"november four five x-ray sierra, inbound for landing","t":1300.0}"#,
            r#"{"kind":"speedhold","event":"target_floored","asked":65.0,"floor":78.0,"t":1350.0}"#,
            r#"{"kind":"state","t":1400.0,"alt":600.0,"agl":500.0,"ias":63.0,"gs":60.0,"ground":false,"vsi":-300.0}"#,
            r#"{"kind":"autoland","rwy":"LXGB 09","dist":1.47,"radio":660.0,"ias":128.0,"phase":"Final","t":1360.0}"#,
            r#"{"kind":"autoland","rwy":"LXGB 09","dist":1.45,"radio":655.0,"ias":127.0,"phase":"Final","t":1361.0}"#,
            r#"{"kind":"callout","id":"airspeed_warning","msg":"Airspeed.","sev":"Warning","t":1401.0}"#,
            r#"{"kind":"autoland","event":"going_around","radio":95.0,"why":"off_centreline_low","xtk_m":-37.7,"t":1500.0}"#,
            r#"{"kind":"autoland","event":"final_report","rwy":"09","dist":4.99,"t":1501.0}"#,
            r#"{"kind":"state","t":1503.0,"alt":114.0,"agl":77.0,"ias":101.0,"gs":96.0,"ground":false,"vsi":-259.0}"#,
            r#"{"kind":"crash","lat":36.148,"lon":-5.358,"alt":113.6,"t":1504.0}"#,
            r#"{"kind":"state","t":1505.0,"alt":110.0,"agl":0.0,"ias":0.0,"gs":0.0,"ground":true,"vsi":-900.0}"#,
        ];
        let r = report(lines.iter().copied());
        assert!(r.contains("Flight 1 of 1"), "{r}");
        assert!(r.contains("NO PROFILE"), "{r}");
        assert!(r.contains("min IAS airborne 63 kt"), "{r}");
        assert!(r.contains("Touchdown 00:25:05Z at -900 fpm"), "{r}");
        assert!(r.contains("rolled to a stop 1 time(s)"), "{r}");
        assert!(r.contains("airspeed_warning x1"), "{r}");
        assert!(r.contains("into the VOID"), "{r}");
        assert!(r.contains("raised to the airframe floor 78"), "{r}");
        assert!(r.contains("GO-AROUND at 95 ft radio: off centreline low, 38 m left of the centreline"), "{r}");
        assert!(r.contains("final report: runway 09, 5.0 nm"), "{r}");
        assert_eq!(r.matches("autoland armed").count(), 1, "one arming, not one per tick: {r}");
        assert!(r.contains("autoland armed: LXGB 09 from 1.5 nm, 660 ft radio, 128 kt, phase Final"), "{r}");
        assert!(r.contains("CRASH at 114 ft"), "{r}");
        assert!(r.contains("full auto: airborne (agl_ft=1019.6)"), "{r}");
    }

    #[test]
    fn flights_split_on_silence_and_the_last_is_reported() {
        let lines = [
            r#"{"kind":"flight_meta","aircraft":"cessna_172sp","t":1000.0}"#,
            r#"{"kind":"crash","alt":10.0,"t":1001.0}"#,
            r#"{"kind":"flight_meta","aircraft":"beechcraft_baron_58","t":9000.0}"#,
            r#"{"kind":"full_auto","stage":"start","detail":{},"t":9001.0}"#,
        ];
        let r = report(lines.iter().copied());
        assert!(r.contains("Flight 2 of 2"), "{r}");
        assert!(r.contains("beechcraft_baron_58") && !r.contains("CRASH"), "{r}");
        assert_eq!(report(std::iter::empty()), "No blackbox events.\n");
    }
}
