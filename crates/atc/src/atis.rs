//! ATIS report synthesis: pure text from a live-weather snapshot.
//!
//! The `atc` binary (`src/bin/atc.rs`) reads the datarefs and speaks the
//! result; everything here is deterministic string-building, so it unit-tests
//! without a simulator. Spoken-aviation conventions apply: digits read
//! individually ("one three one decimal two"), "niner" for 9, QNH in whole
//! hectopascals, wind direction rounded to ten degrees.

/// Everything one ATIS edition is built from. The `atc` binary fills this
/// from datarefs each broadcast cycle.
#[derive(Debug, Clone)]
pub struct Snapshot {
    /// Spoken station name ("Gibraltar"), from the apt.dat header.
    pub airport: String,
    /// Sim UTC, seconds since midnight (`sim/time/zulu_time_sec`).
    pub zulu_sec: f64,
    /// Runway in use ("27"); empty skips the sentence.
    pub runway: String,
    /// Wind direction FROM, degrees true (the XP12 aircraft-weather dataref).
    pub wind_from_deg: f64,
    pub wind_kts: f64,
    pub visibility_sm: f64,
    pub qnh_hpa: f64,
    pub oat_c: f64,
}

/// Below this, the wind direction is meaningless for edition purposes (a
/// METAR would call it variable) — a light breeze wanders tens of degrees.
const VARIABLE_WIND_KTS: f64 = 6.0;

/// Does `cur` differ enough from the snapshot that cut the current edition to
/// deserve a new information letter? Thresholds against the *edition's*
/// values, not fixed rounding buckets: buckets flap when the value sits on an
/// edge (seen live at LXGB 2026-07-07, a 3 kt wind wobbling 92-107 degrees
/// burned a letter per broadcast). With hysteresis the reference only moves
/// when an edition is actually cut, so wobble around any point is absorbed.
pub fn new_edition(edition: &Snapshot, cur: &Snapshot) -> bool {
    if cur.runway != edition.runway {
        return true;
    }
    if (cur.qnh_hpa.round() - edition.qnh_hpa.round()).abs() >= 1.0 {
        return true;
    }
    if (cur.oat_c - edition.oat_c).abs() >= 2.0 {
        return true;
    }
    if (cur.wind_kts - edition.wind_kts).abs() >= 5.0 {
        return true;
    }
    // Direction only matters when there's enough wind for it to mean anything.
    if cur.wind_kts.max(edition.wind_kts) >= VARIABLE_WIND_KTS {
        let d = (cur.wind_from_deg - edition.wind_from_deg + 540.0).rem_euclid(360.0) - 180.0;
        if d.abs() >= 30.0 {
            return true;
        }
    }
    false
}

/// The next information letter, wrapping Z back to A.
pub fn next_letter(c: char) -> char {
    match c {
        'Z' => 'A',
        c => (c as u8 + 1) as char,
    }
}

/// ICAO spelling alphabet, for the information letter.
#[rustfmt::skip]
pub fn nato(c: char) -> &'static str {
    match c {
        'A' => "Alpha", 'B' => "Bravo", 'C' => "Charlie", 'D' => "Delta",
        'E' => "Echo", 'F' => "Foxtrot", 'G' => "Golf", 'H' => "Hotel",
        'I' => "India", 'J' => "Juliett", 'K' => "Kilo", 'L' => "Lima",
        'M' => "Mike", 'N' => "November", 'O' => "Oscar", 'P' => "Papa",
        'Q' => "Quebec", 'R' => "Romeo", 'S' => "Sierra", 'T' => "Tango",
        'U' => "Uniform", 'V' => "Victor", 'W' => "Whiskey", 'X' => "X-ray",
        'Y' => "Yankee", 'Z' => "Zulu",
        _ => "Alpha",
    }
}

/// Digits (and a decimal point) read individually, ICAO style: 9 is "niner".
pub fn spoken_digits(s: &str) -> String {
    s.chars()
        .filter_map(|c| match c {
            '0' => Some("zero"),
            '1' => Some("one"),
            '2' => Some("two"),
            '3' => Some("three"),
            '4' => Some("four"),
            '5' => Some("five"),
            '6' => Some("six"),
            '7' => Some("seven"),
            '8' => Some("eight"),
            '9' => Some("niner"),
            '.' => Some("decimal"),
            _ => None,
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// A radio frequency in kHz spoken as MHz: 131200 -> "one three one decimal two".
/// An altitude in controller words, to the nearest hundred: "two thousand
/// five hundred", "eight hundred". Shared by the tower's instructions and the
/// crew's check-ins so both sides say altitudes the same way.
pub fn spoken_alt(ft: f64) -> String {
    let hundreds = ((ft / 100.0).round() as i64).max(1);
    let (th, hu) = (hundreds / 10, hundreds % 10);
    let word = |n: i64| spoken_digits(&n.to_string());
    match (th, hu) {
        (0, h) => format!("{} hundred", word(h)),
        (t, 0) => format!("{} thousand", word(t)),
        (t, h) => format!("{} thousand {} hundred", word(t), word(h)),
    }
}

pub fn spoken_mhz(khz: i64) -> String {
    let mhz = khz / 1000;
    let frac = (khz % 1000).abs();
    let frac = if frac % 100 == 0 {
        format!("{}", frac / 100)
    } else if frac % 10 == 0 {
        format!("{:02}", frac / 10)
    } else {
        format!("{frac:03}")
    };
    spoken_digits(&format!("{mhz}.{frac}"))
}

/// A runway designator spoken out: "27" -> "two seven", "16L" -> "one six left".
pub fn spoken_runway(id: &str) -> String {
    let mut words: Vec<&str> = Vec::new();
    let digits: String = id.chars().take_while(|c| c.is_ascii_digit()).collect();
    let spoken = spoken_digits(&digits);
    if !spoken.is_empty() {
        words.push(&spoken);
    }
    match id.chars().last() {
        Some('L') => words.push("left"),
        Some('R') => words.push("right"),
        Some('C') => words.push("center"),
        _ => {}
    }
    words.join(" ")
}

/// Build one full broadcast for this edition.
pub fn report(s: &Snapshot, letter: char) -> String {
    let info = nato(letter.to_ascii_uppercase());

    let hh = ((s.zulu_sec / 3600.0) as i64).rem_euclid(24);
    let mm = ((s.zulu_sec / 60.0) as i64).rem_euclid(60);
    let time = spoken_digits(&format!("{hh:02}{mm:02}"));

    let wind = if s.wind_kts < 3.0 {
        "Wind calm".to_string()
    } else {
        // Direction to the nearest ten degrees; a north wind is 360, not 000.
        let dir = ((s.wind_from_deg / 10.0).round() as i64 * 10).rem_euclid(360);
        let dir = if dir == 0 { 360 } else { dir };
        format!(
            "Wind {} degrees, {} knots",
            spoken_digits(&format!("{dir:03}")),
            spoken_digits(&format!("{:.0}", s.wind_kts))
        )
    };

    let km = s.visibility_sm * 1.609344;
    let vis = if km >= 9.9 {
        "Visibility one zero kilometers or more".to_string()
    } else {
        format!(
            "Visibility {} kilometers",
            spoken_digits(&format!("{:.0}", km.max(1.0)))
        )
    };

    let t = s.oat_c.round() as i64;
    let temp = if t < 0 {
        format!("Temperature minus {}", spoken_digits(&format!("{}", -t)))
    } else {
        format!("Temperature {}", spoken_digits(&format!("{t}")))
    };

    let qnh = spoken_digits(&format!("{:.0}", s.qnh_hpa));

    let mut out = format!("{} information {}. Time {} zulu.", s.airport, info, time);
    if !s.runway.is_empty() {
        out.push_str(&format!(" Runway in use {}.", spoken_runway(&s.runway)));
    }
    out.push_str(&format!(
        " {}. {}. {}. QNH {}. Advise on initial contact, you have information {}.",
        wind, vis, temp, qnh, info
    ));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snap() -> Snapshot {
        Snapshot {
            airport: "Gibraltar".into(),
            zulu_sec: 12.0 * 3600.0 + 50.0 * 60.0, // 1250Z
            runway: "27".into(),
            wind_from_deg: 251.0,
            wind_kts: 12.0,
            visibility_sm: 10.0,
            qnh_hpa: 1018.4,
            oat_c: 24.3,
        }
    }

    #[test]
    fn frequencies_speak_as_mhz() {
        // LXGB's real apt.dat rows: tower, approach, departure.
        assert_eq!(spoken_mhz(131200), "one three one decimal two");
        assert_eq!(spoken_mhz(122800), "one two two decimal eight");
        assert_eq!(spoken_mhz(130400), "one three zero decimal four");
        // An 8.33 channel keeps all three decimals; guard keeps its zero.
        assert_eq!(spoken_mhz(118325), "one one eight decimal three two five");
        assert_eq!(spoken_mhz(121500), "one two one decimal five");
        assert_eq!(spoken_mhz(118050), "one one eight decimal zero five");
    }

    #[test]
    fn report_speaks_the_edition() {
        let r = report(&snap(), 'A');
        assert!(r.starts_with("Gibraltar information Alpha."), "{r}");
        assert!(r.contains("Time one two five zero zulu"), "{r}");
        assert!(r.contains("Runway in use two seven"), "{r}");
        assert!(
            r.contains("Wind two five zero degrees, one two knots"),
            "{r}"
        );
        assert!(r.contains("Visibility one zero kilometers or more"), "{r}");
        assert!(r.contains("Temperature two four"), "{r}");
        assert!(r.contains("QNH one zero one eight"), "{r}");
        assert!(r.ends_with("you have information Alpha."), "{r}");
    }

    #[test]
    fn niner_calm_wind_and_negative_temps() {
        let mut s = snap();
        s.qnh_hpa = 999.0;
        s.wind_kts = 1.0;
        s.oat_c = -2.0;
        let r = report(&s, 'B');
        assert!(r.contains("QNH niner niner niner"), "{r}");
        assert!(r.contains("Wind calm"), "{r}");
        assert!(r.contains("Temperature minus two"), "{r}");
        assert!(r.contains("information Bravo"), "{r}");
    }

    #[test]
    fn runway_designators_speak_the_side() {
        assert_eq!(spoken_runway("27"), "two seven");
        assert_eq!(spoken_runway("09"), "zero niner");
        assert_eq!(spoken_runway("16L"), "one six left");
        assert_eq!(spoken_runway("34C"), "three four center");
    }

    #[test]
    fn light_wind_wobble_is_not_a_new_edition() {
        // The live failure (LXGB 2026-07-07): a 3 kt wind wandering 92-107
        // degrees advanced the letter every broadcast. Direction is ignored
        // below the variable-wind threshold.
        let mut edition = snap();
        edition.wind_from_deg = 92.0;
        edition.wind_kts = 3.2;
        for dir in [107.0, 100.0, 75.0, 92.0] {
            let mut cur = edition.clone();
            cur.wind_from_deg = dir;
            cur.zulu_sec += 600.0; // time never makes a new edition
            assert!(
                !new_edition(&edition, &cur),
                "3 kt wobble to {dir} cut an edition"
            );
        }
    }

    #[test]
    fn real_changes_cut_a_new_edition() {
        let edition = snap(); // wind 251 at 12
        let mut wind_swing = snap();
        wind_swing.wind_from_deg = 285.0;
        assert!(
            new_edition(&edition, &wind_swing),
            "30+ degree swing at 12 kt"
        );
        let mut wind_north = snap();
        wind_north.wind_from_deg = 5.0; // circular diff vs 251 is ~114, not ~246
        assert!(new_edition(&edition, &wind_north));
        let mut qnh = snap();
        qnh.qnh_hpa = 1016.0;
        assert!(new_edition(&edition, &qnh), "pressure change");
        let mut rwy = snap();
        rwy.runway = "09".into();
        assert!(new_edition(&edition, &rwy), "runway change");
        let mut speed = snap();
        speed.wind_kts = 18.0;
        assert!(new_edition(&edition, &speed), "5+ kt speed change");
        // Hysteresis: wobble around the edition's own values stays put.
        let mut wobble = snap();
        wobble.wind_from_deg = 262.0;
        wobble.wind_kts = 14.0;
        wobble.oat_c = 25.0;
        wobble.qnh_hpa = 1018.2;
        assert!(!new_edition(&edition, &wobble), "within-threshold drift");
    }

    #[test]
    fn letters_advance_and_wrap() {
        assert_eq!(next_letter('A'), 'B');
        assert_eq!(next_letter('Z'), 'A');
        assert_eq!(nato('X'), "X-ray");
    }
}
