//! Airspace from X-Plane's own OpenAir file — the polygons the G1000 draws.
//!
//! The sim ships `Custom Data/airspaces/airspace.txt` (Resources fallback):
//! ~26,000 volumes in OpenAir format — `AC` class, `AN` name, `AL`/`AH`
//! floor/ceiling, `DP` polygon points (D:M:S), `V X=`+`DC` circles. This
//! module reads the NO-GO classes — P (prohibited), R (restricted), Q
//! (danger) — near a point, so the circuit's pattern picker can treat a
//! restricted volume exactly like terrain: one generic rule, no per-airport
//! exceptions (R-164 ALGECIRAS sits beside LXGB's pattern, 2026-07-24).
//!
//! Deliberately coarse where the format is baroque: `DB` arcs contribute
//! their endpoints only, and AGL altitudes are read as MSL numbers. Both are
//! documented approximations for a horizontal-avoidance check at pattern
//! heights, not a legal navigation database.

use std::io::{BufRead, BufReader};

/// One no-go volume: class P/R/Q, its altitude band (ft), and the polygon.
#[derive(Debug, Clone)]
pub struct Airspace {
    /// 'P' prohibited, 'R' restricted, 'Q' danger.
    pub class: char,
    /// Raw AN line, e.g. `R-164        (ALGECIRAS (CADIZ))`.
    pub name: String,
    pub floor_ft: f64,
    pub ceiling_ft: f64,
    /// (lat, lon) vertices.
    pub poly: Vec<(f64, f64)>,
}

impl Airspace {
    /// Even-odd ray cast; planar treatment is fine at pattern scales.
    pub fn contains(&self, lat: f64, lon: f64) -> bool {
        let n = self.poly.len();
        if n < 3 {
            return false;
        }
        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let (yi, xi) = self.poly[i];
            let (yj, xj) = self.poly[j];
            if ((yi > lat) != (yj > lat))
                && lon < (xj - xi) * (lat - yi) / (yj - yi + f64::EPSILON) + xi
            {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    /// Is this volume a conflict at `alt_ft` over `(lat, lon)`?
    pub fn blocks(&self, lat: f64, lon: f64, alt_ft: f64) -> bool {
        alt_ft >= self.floor_ft - 100.0
            && alt_ft <= self.ceiling_ft + 100.0
            && self.contains(lat, lon)
    }

    /// How the copilot says it: "restricted area R 164". Dashes swapped for
    /// spaces (TTS).
    ///
    /// The OpenAir `AN` line is `<ident> (<description>)`, and WHICH half is
    /// speakable depends on the ident. A real designator is what a pilot says
    /// — "restricted area R 521" — so `R-521 (LUCAS HEIGHTS ANSTO RESEARCH R)`
    /// keeps the ident and drops the rest. But plenty of idents are squashed
    /// words, and reading those aloud is gibberish: `W-METRSEARCH (MET
    /// RESEARCH FLTS AREA ALPHA)` came out as "danger area W METRSEARCH", the
    /// hyphen having become a space on the way (seen in the transcript,
    /// 2026-07-30). There the parenthetical IS the name.
    pub fn spoken_name(&self) -> String {
        let kind = match self.class {
            'P' => "prohibited area",
            'Q' => "danger area",
            _ => "restricted area",
        };
        let (ident_raw, desc) = match self.name.split_once('(') {
            Some((i, rest)) => (
                i.trim(),
                rest.split_once(')').map_or(rest, |(d, _)| d).trim(),
            ),
            None => (self.name.trim(), ""),
        };
        // A DESIGNATOR is an identifier: keep its case, just unpick the
        // dashes for TTS. A DESCRIPTION is prose, so it gets the full
        // treatment — shorthand expanded, the repeated class word dropped,
        // and the shouting lowercased.
        let ident = ident_raw
            .replace('-', " ")
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ");
        let described = speakable(&desc.replace('-', " "), kind);
        match () {
            _ if is_designator(ident_raw) && !ident.is_empty() => format!("{kind} {ident}"),
            _ if !described.is_empty() => format!("{kind} {described}"),
            _ if !ident.is_empty() => format!("{kind} {ident}"),
            _ => kind.to_string(),
        }
    }
}

/// Aviation shorthand that appears in OpenAir names and is not a word.
///
/// Counted across the shipping file's 15,700 no-go names: TEMP 1359, PJE 343,
/// MOA 162, UAS 141, UAV 124, FLTS 4. Read aloud these come out as noise —
/// "MET RESEARCH FLTS AREA ALPHA" said "flits" (heard 2026-07-30).
///
/// Deliberately short: only expansions that are unambiguous. FBZ (145) and UAF
/// (91) are frequent but I could not pin their meaning, and a confident wrong
/// expansion is worse than an abbreviation the pilot can still recognise.
const SPOKEN_ABBREVIATIONS: &[(&str, &str)] = &[
    ("FLTS", "flights"),
    ("FLT", "flight"),
    ("TEMP", "temporary"),
    ("PJE", "parachute jumping"),
    ("MOA", "military operations"),
    ("UAS", "drone"),
    ("UAV", "drone"),
    ("ACFT", "aircraft"),
    ("TRNG", "training"),
    ("MIL", "military"),
];

/// Make an OpenAir name speakable: expand the shorthand, drop a repeat of the
/// class word, and lowercase the shouting.
///
/// The repeat matters — 19% of no-go names already contain "AREA", so pairing
/// them with a "danger area" prefix says it twice ("danger area MET RESEARCH
/// FLTS AREA ALPHA").
fn speakable(name: &str, kind: &str) -> String {
    name.split_whitespace()
        .filter(|w| {
            // Drop a standalone "AREA"/"ZONE" the class word already supplies.
            let bare = w.trim_matches(|c: char| !c.is_ascii_alphanumeric());
            !kind
                .split_whitespace()
                .any(|k| k.eq_ignore_ascii_case(bare))
        })
        .map(|w| {
            SPOKEN_ABBREVIATIONS
                .iter()
                .find(|(a, _)| w.eq_ignore_ascii_case(a))
                .map_or_else(|| w.to_ascii_lowercase(), |(_, full)| (*full).to_string())
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// Does this OpenAir ident read as a real airspace DESIGNATOR — the thing a
/// pilot actually says, like `R-521`, `D-138`, `EG D513`?
///
/// The test is simply whether every letter run in it is short. Designators are
/// a letter or two beside a number; the idents that must NOT be spoken are
/// squashed words, and `METRSEARCH` fails on its ten-letter run where `R` and
/// `EG` pass. A digit is required too, so a bare word can never qualify.
fn is_designator(ident: &str) -> bool {
    let ident = ident.trim();
    if ident.is_empty() || !ident.chars().any(|c| c.is_ascii_digit()) {
        return false;
    }
    let mut run = 0usize;
    for c in ident.chars() {
        if c.is_ascii_alphabetic() {
            run += 1;
            if run > 3 {
                return false;
            }
        } else {
            run = 0;
        }
    }
    true
}

/// "36:09:13 N" / "005:28:58 W" → signed degrees. Also accepts D:M forms.
fn dms(tok: &str, hemi: char) -> Option<f64> {
    let mut parts = tok.split(':');
    let d: f64 = parts.next()?.trim().parse().ok()?;
    let m: f64 = parts
        .next()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(0.0);
    let s: f64 = parts
        .next()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(0.0);
    let v = d + m / 60.0 + s / 3600.0;
    Some(if hemi == 'S' || hemi == 'W' { -v } else { v })
}

/// One "lat-hemi lon-hemi" coordinate anywhere in `s`.
fn coord(s: &str) -> Option<(f64, f64)> {
    let mut lat: Option<f64> = None;
    let mut pending: Option<&str> = None;
    let mut lon: Option<f64> = None;
    for tok in s.split_whitespace() {
        match tok {
            "N" | "S" | "E" | "W" => {
                let h = tok.chars().next().unwrap();
                if let Some(num) = pending.take() {
                    let v = dms(num, h)?;
                    if h == 'N' || h == 'S' {
                        lat = Some(v);
                    } else {
                        lon = Some(v);
                    }
                }
            }
            t if t.contains(':') => pending = Some(t),
            // Compact "36:09:13N" form.
            t if t.len() > 1 && t.ends_with(['N', 'S', 'E', 'W']) && t.contains(':') => {
                pending = Some(&t[..t.len() - 1]);
            }
            _ => {}
        }
        if let (Some(a), Some(b)) = (lat, lon) {
            return Some((a, b));
        }
    }
    lat.zip(lon)
}

/// OpenAir altitude → feet. GND/SFC = 0, UNL = unbounded, FLxxx = xxx·100,
/// otherwise the first number (AGL read as MSL — documented approximation).
fn alt_ft(s: &str) -> f64 {
    let u = s.trim().to_ascii_uppercase();
    if u.starts_with("GND") || u.starts_with("SFC") {
        return 0.0;
    }
    if u.starts_with("UNL") {
        return 400_000.0;
    }
    if let Some(fl) = u.strip_prefix("FL") {
        if let Ok(v) = fl.trim().parse::<f64>() {
            return v * 100.0;
        }
    }
    u.split_whitespace()
        .find_map(|t| t.trim_end_matches("FT").parse::<f64>().ok())
        .unwrap_or(0.0)
}

/// Parse the NO-GO volumes (P/R/Q) out of OpenAir text.
pub fn parse_no_go(text: impl Iterator<Item = String>) -> Vec<Airspace> {
    let mut out: Vec<Airspace> = Vec::new();
    let mut cur: Option<Airspace> = None;
    let mut center: Option<(f64, f64)> = None;
    let mut keep = false;
    for line in text {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("AC ") {
            if keep {
                if let Some(a) = cur.take() {
                    if a.poly.len() >= 3 {
                        out.push(a);
                    }
                }
            }
            let class = rest.trim();
            keep = matches!(class, "P" | "R" | "Q");
            center = None;
            cur = keep.then(|| Airspace {
                class: class.chars().next().unwrap_or('R'),
                name: String::new(),
                floor_ft: 0.0,
                ceiling_ft: 400_000.0,
                poly: Vec::new(),
            });
            continue;
        }
        let Some(a) = cur.as_mut() else { continue };
        if let Some(rest) = line.strip_prefix("AN ") {
            a.name = rest.trim().to_string();
        } else if let Some(rest) = line.strip_prefix("AL ") {
            a.floor_ft = alt_ft(rest);
        } else if let Some(rest) = line.strip_prefix("AH ") {
            a.ceiling_ft = alt_ft(rest);
        } else if let Some(rest) = line.strip_prefix("DP ") {
            if let Some(p) = coord(rest) {
                a.poly.push(p);
            }
        } else if let Some(rest) = line.strip_prefix("V ") {
            if let Some(c) = rest.trim().strip_prefix("X=").and_then(coord) {
                center = Some(c);
            }
        } else if let Some(rest) = line.strip_prefix("DC ") {
            // A circle of radius nm around the V X= center → a 24-gon.
            if let (Some((clat, clon)), Ok(r_nm)) = (center, rest.trim().parse::<f64>()) {
                let r_lat = r_nm / 60.0;
                let r_lon = r_lat / clat.to_radians().cos().max(1e-6);
                for k in 0..24 {
                    let th = (k as f64) * std::f64::consts::TAU / 24.0;
                    a.poly
                        .push((clat + r_lat * th.cos(), clon + r_lon * th.sin()));
                }
            }
        } else if let Some(rest) = line.strip_prefix("DB ") {
            // Arc: coarse — keep the two endpoints (horizontal avoidance only).
            for part in rest.split(',') {
                if let Some(p) = coord(part) {
                    a.poly.push(p);
                }
            }
        }
    }
    if keep {
        if let Some(a) = cur.take() {
            if a.poly.len() >= 3 {
                out.push(a);
            }
        }
    }
    out
}

/// The no-go volumes near `(lat, lon)` (any vertex within `half_deg`, or the
/// polygon's bounding box straddling the point). One streaming pass; call it
/// off the reply path — the file is ~30 MB.
pub fn load_near(path: &str, lat: f64, lon: f64, half_deg: f64) -> Vec<Airspace> {
    let Ok(file) = std::fs::File::open(path) else {
        return Vec::new();
    };
    let all = parse_no_go(BufReader::new(file).lines().map_while(Result::ok));
    all.into_iter()
        .filter(|a| {
            let near = a
                .poly
                .iter()
                .any(|(la, lo)| (la - lat).abs() <= half_deg && (lo - lon).abs() <= half_deg);
            near || {
                let (mut lo_la, mut hi_la, mut lo_lo, mut hi_lo) =
                    (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
                for (la, lo) in &a.poly {
                    lo_la = lo_la.min(*la);
                    hi_la = hi_la.max(*la);
                    lo_lo = lo_lo.min(*lo);
                    hi_lo = hi_lo.max(*lo);
                }
                (lo_la..=hi_la).contains(&lat) && (lo_lo..=hi_lo).contains(&lon)
            }
        })
        .collect()
}

/// The first volume any of `pts` enters at `alt_ft`; `None` = the path is
/// clear of every no-go volume in `spaces`.
pub fn first_conflict<'a>(
    spaces: &'a [Airspace],
    pts: &[(f64, f64)],
    alt_ft: f64,
) -> Option<&'a Airspace> {
    spaces
        .iter()
        .find(|a| pts.iter().any(|(la, lo)| a.blocks(*la, *lo, alt_ft)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(text: &str) -> Vec<Airspace> {
        parse_no_go(text.lines().map(|s| s.to_string()))
    }

    #[test]
    fn parses_a_restricted_square_and_checks_containment() {
        let spaces = parse(
            "AC R\n\
             AN R-999 (TESTZONE)\n\
             AL GND\n\
             AH 3000 MSL\n\
             DP 36:00:00 N 005:30:00 W\n\
             DP 36:00:00 N 005:00:00 W\n\
             DP 36:30:00 N 005:00:00 W\n\
             DP 36:30:00 N 005:30:00 W\n\
             \n\
             AC C\n\
             AN IGNORED TMA\n\
             AL 1000 MSL\n\
             AH 10000 MSL\n\
             DP 36:00:00 N 005:30:00 W\n\
             DP 36:00:00 N 005:00:00 W\n\
             DP 36:30:00 N 005:15:00 W\n",
        );
        // Class C is not a no-go; only the R square survives.
        assert_eq!(spaces.len(), 1);
        let r = &spaces[0];
        assert_eq!(r.class, 'R');
        assert!((r.floor_ft - 0.0).abs() < 1.0 && (r.ceiling_ft - 3000.0).abs() < 1.0);
        // Inside at pattern height: blocked. Above the ceiling: clear.
        assert!(r.blocks(36.25, -5.25, 1000.0));
        assert!(!r.blocks(36.25, -5.25, 5000.0));
        // Outside laterally: clear.
        assert!(!r.blocks(35.9, -5.25, 1000.0));
        assert_eq!(r.spoken_name(), "restricted area R 999");
    }

    #[test]
    fn parses_a_danger_circle() {
        let spaces = parse(
            "AC Q\n\
             AN D-1\n\
             AL GND\n\
             AH 2000 MSL\n\
             V X=36:10:00 N 005:20:00 W\n\
             DC 2\n",
        );
        assert_eq!(spaces.len(), 1);
        let q = &spaces[0];
        assert_eq!(q.poly.len(), 24);
        // Center is inside; 5 nm east is not.
        assert!(q.blocks(36.1667, -5.3333, 1000.0));
        assert!(!q.blocks(36.1667, -5.2, 1000.0));
    }

    #[test]
    fn altitude_forms_parse() {
        assert_eq!(alt_ft("GND"), 0.0);
        assert_eq!(alt_ft("FL100"), 10000.0);
        assert_eq!(alt_ft("2500 MSL"), 2500.0);
        assert_eq!(alt_ft("400 AGL"), 400.0);
        assert!(alt_ft("UNL") > 100_000.0);
    }

    #[test]
    fn first_conflict_finds_the_entered_volume() {
        let spaces = parse(
            "AC R\nAN R-1\nAL GND\nAH 3000 MSL\n\
             DP 36:00:00 N 005:30:00 W\nDP 36:00:00 N 005:00:00 W\n\
             DP 36:30:00 N 005:00:00 W\nDP 36:30:00 N 005:30:00 W\n",
        );
        let inside = [(36.25, -5.25)];
        let outside = [(35.5, -5.25)];
        assert!(first_conflict(&spaces, &inside, 1000.0).is_some());
        assert!(first_conflict(&spaces, &outside, 1000.0).is_none());
        assert!(
            first_conflict(&spaces, &inside, 9000.0).is_none(),
            "above the band"
        );
    }

    /// The OpenAir `AN` line is `<ident> (<description>)` and only ONE of the
    /// two is speakable, depending on the ident.
    ///
    /// `W-METRSEARCH (MET RESEARCH FLTS AREA ALPHA)` was announced as "danger
    /// area W METRSEARCH" — the hyphen became a space and the readable half
    /// was thrown away (transcript, 2026-07-30). A real designator like R-521
    /// must still win, because that IS what a pilot says.
    #[test]
    fn the_speakable_half_of_an_airspace_name_wins() {
        let space = |class: char, name: &str| Airspace {
            class,
            name: name.to_string(),
            floor_ft: 0.0,
            ceiling_ft: 5000.0,
            poly: vec![],
        };

        // A designator: keep it, dashes to spaces.
        assert_eq!(
            space('R', "R-521        (LUCAS HEIGHTS ANSTO RESEARCH R)").spoken_name(),
            "restricted area R 521"
        );
        // A squashed word: the parenthetical is the name.
        assert_eq!(
            space('Q', "W-METRSEARCH (MET RESEARCH FLTS AREA ALPHA)").spoken_name(),
            // FLTS expanded, the repeated "AREA" dropped (the class word
            // already said it), and the shouting lowered.
            "danger area met research flights alpha"
        );
        // Never emits the gibberish that started this.
        assert!(!space('Q', "W-METRSEARCH (MET RESEARCH FLTS AREA ALPHA)")
            .spoken_name()
            .contains("METRSEARCH"));

        // No parenthetical at all: whatever we have is better than nothing.
        assert_eq!(space('P', "P-56").spoken_name(), "prohibited area P 56");
        // A bare name with no parenthetical takes the IDENT path, which
        // preserves case — it may be a designator and those are identifiers.
        assert_eq!(
            space('Q', "DANGER ZONE").spoken_name(),
            "danger area DANGER ZONE"
        );
        // Nothing usable: the class alone, never a dangling "area ".
        assert_eq!(space('Q', "   ").spoken_name(), "danger area");

        // The designator test itself.
        for d in ["R-521", "P-56", "EG D513", "D-138"] {
            assert!(is_designator(d), "{d} is a designator");
        }
        for w in ["METRSEARCH", "W-METRSEARCH", "DANGER ZONE", ""] {
            assert!(!is_designator(w), "{w} is not a designator");
        }
    }
}
