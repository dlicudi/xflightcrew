//! The radios: frequencies and the transponder.

use super::*;

/// Radio tuning by number: COM1, COM2 and NAV1.
pub(super) fn tune(line: &Line) -> Option<ToolCall> {
    let t = line.t;

    // Radio tuning by number: "tune radio to 122.8", "com one 118.5", "set com
    // two to 121.9", "nav one 109.5". Was LLM-only, so with the LLM off every
    // phrasing missed (six in a row on the 2026-07-08 night flight, stranding
    // the pilot off-frequency). Requires a radio word AND a decimal in the
    // right band, so headings ("heading 122") and altimeter settings never
    // match. Whisper glues the radio index into the number ("com 1 122.8" →
    // "com 1122.8"), so 1xxx.x/2xxx.x recover com1/com2 + the frequency.
    // A change/switch verb with a decimal is a tuning request even without a
    // radio word, "change to 125.655" missed live (2026-07-20) while
    // "change to frequency 125.655" matched. The band check below is the
    // real guard; the subject exclusions keep fractional non-radio values
    // ("change heading 122.5") out of the block entirely.
    let change_with_decimal = (has(t, "change") || has(t, "switch"))
        && t.contains('.')
        && !(has(t, "heading")
            || has(t, "course")
            || has(t, "altitude")
            || has(t, "qnh")
            || has(t, "altimeter")
            || has(t, "barometer")
            || has(t, "baro"));
    if has(t, "com")
        || has(t, "radio")
        || has(t, "frequency")
        || has(t, "freq")
        || has(t, "nav")
        || has(t, "tune")
        || change_with_decimal
    {
        let com_band = 118.0..=136.975;
        let nav_band = 108.0..=117.95;
        // Spoken decimals survive as words ("one two two decimal eight");
        // rejoin them so the band check sees one number.
        let spelled = t.replace(" decimal ", ".").replace(" point ", ".");
        let com2_named = spelled.contains("com two") || spelled.contains("com 2") || has(t, "com2");
        let mut found: Option<(&str, f64)> = None;
        for w in spelled.split(|c: char| !(c.is_ascii_digit() || c == '.')) {
            let Ok(v) = w.parse::<f64>() else { continue };
            if !w.contains('.') && v < 1000.0 {
                continue; // integers below the glue range: headings, squawks
            }
            found = if nav_band.contains(&v) && has(t, "nav") {
                Some(("set_nav1", v))
            } else if com_band.contains(&v) && !has(t, "nav") {
                Some((if com2_named { "set_com2" } else { "set_com1" }, v))
            } else if com_band.contains(&(v - 1000.0)) {
                // "com 1122.8" glue; re-round to kHz (float subtraction dust).
                Some(("set_com1", ((v - 1000.0) * 1000.0).round() / 1000.0))
            } else if com_band.contains(&(v - 2000.0)) {
                Some(("set_com2", ((v - 2000.0) * 1000.0).round() / 1000.0))
            } else {
                None
            };
            if found.is_some() {
                break;
            }
        }
        // No decimal token: whisper also writes frequencies as spoken parts,
        // "130 four hundred", "1 thirty four hundred", "131, 200" (all shapes
        // from the 2026-07-09 morning log, every one a miss). Fold the number
        // atoms into integer MHz + kHz: a leading 1 glues onto a tens word
        // ("1 thirty one" = 131), "four hundred" multiplies onto the 4, and a
        // 2-3 digit trailer is the kHz part (a single digit is tenths).
        if found.is_none() && !has(t, "nav") {
            let mut atoms: Vec<i64> = Vec::new();
            for tok in spelled.split(|c: char| !c.is_ascii_alphanumeric()) {
                if tok.is_empty() {
                    continue;
                }
                if let Ok(d) = tok.parse::<i64>() {
                    atoms.push(d);
                } else if let Some((v, mult)) = word_value(tok) {
                    if mult && v == 100 {
                        if let Some(last) = atoms.last_mut() {
                            *last *= 100;
                        }
                    } else if !mult {
                        atoms.push(v);
                    }
                }
            }
            let mut i = 0;
            while i < atoms.len() && found.is_none() {
                let (int_mhz, j) = if atoms[i] == 1
                    && atoms
                        .get(i + 1)
                        .is_some_and(|v| (20..=90).contains(v) && v % 10 == 0)
                {
                    let mut v = 100 + atoms[i + 1];
                    let mut j = i + 2;
                    if atoms.get(j).is_some_and(|u| (1..=9).contains(u)) {
                        v += atoms[j];
                        j += 1;
                    }
                    (v, j)
                } else {
                    (atoms[i], i + 1)
                };
                if com_band.contains(&(int_mhz as f64)) {
                    let frac = match atoms.get(j) {
                        Some(&k) if (10..=999).contains(&k) => k as f64 / 1000.0,
                        Some(&d) if (1..=9).contains(&d) => d as f64 / 10.0,
                        _ => 0.0,
                    };
                    let tool = if com2_named { "set_com2" } else { "set_com1" };
                    found = Some((tool, int_mhz as f64 + frac));
                }
                i += 1;
            }
        }
        if let Some((tool, mhz)) = found {
            return Some(call(tool, json!({ "mhz": mhz })));
        }
    }

    None
}

/// The transponder: modes and codes.
pub(super) fn squawk(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // transponder / squawk
    if has(t, "squawk") || has(t, "transponder") {
        if has(t, "standby") {
            return Some(call("set_transponder_mode", json!({ "mode": "standby" })));
        }
        if has(t, "ident") {
            return Some(call("set_transponder_mode", json!({ "mode": "ident" })));
        }
        // Mode words before on/off: "mode C on" and "squawk altitude" are the
        // altitude mode, not a bare "on". The 2026-07-10 pilot said
        // "transponder on" ten ways and every one missed to the LLM, only
        // standby/ident/vfr/codes were covered.
        if has(t, "altitude")
            || has(t, "alt")
            || has(t, "charlie")
            || (has(t, "mode") && has(t, "c"))
        {
            return Some(call("set_transponder_mode", json!({ "mode": "altitude" })));
        }
        if has(t, "off") {
            return Some(call("set_transponder_mode", json!({ "mode": "off" })));
        }
        if has(t, "on") || has(t, "activate") || has(t, "enable") {
            return Some(call("set_transponder_mode", json!({ "mode": "on" })));
        }
        if has(t, "vfr") {
            return Some(call("set_transponder", json!({ "code": 1200 })));
        }
        if let Some(n) = num {
            return Some(call("set_transponder", json!({ "code": n })));
        }
        // "Set transponder." (SR22 climbing out of LSZS, 2026-09-24: a dead
        // end, and the box stayed in standby): no code and no mode is the
        // box brought to work, which is ALT, as the takeoff sets it. "Set
        // transponder code" with no code is left to ask again.
        if has(t, "set") && !has(t, "code") {
            return Some(call("set_transponder_mode", json!({ "mode": "altitude" })));
        }
    }

    None
}
