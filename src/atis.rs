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

/// The fields that make an ATIS *edition*, rounded the way the broadcast
/// rounds them. When this changes between cycles, the information letter
/// advances; comparing rounded values keeps sensor jitter from burning
/// through the alphabet.
pub fn fingerprint(s: &Snapshot) -> (String, i64, i64, i64, i64) {
    (
        s.runway.clone(),
        s.qnh_hpa.round() as i64,
        (s.wind_from_deg / 20.0).round() as i64,
        (s.wind_kts / 5.0).round() as i64,
        (s.oat_c / 2.0).round() as i64,
    )
}

/// The next information letter, wrapping Z back to A.
pub fn next_letter(c: char) -> char {
    match c {
        'Z' => 'A',
        c => (c as u8 + 1) as char,
    }
}

/// ICAO spelling alphabet, for the information letter.
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
        format!("Visibility {} kilometers", spoken_digits(&format!("{:.0}", km.max(1.0))))
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
    fn report_speaks_the_edition() {
        let r = report(&snap(), 'A');
        assert!(r.starts_with("Gibraltar information Alpha."), "{r}");
        assert!(r.contains("Time one two five zero zulu"), "{r}");
        assert!(r.contains("Runway in use two seven"), "{r}");
        assert!(r.contains("Wind two five zero degrees, one two knots"), "{r}");
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
    fn fingerprint_ignores_jitter_but_not_changes() {
        let a = snap();
        let mut b = snap();
        b.wind_from_deg = 254.0; // sensor wobble within the rounding buckets:
        b.wind_kts = 11.0; // same edition (a wobble ACROSS a bucket edge is
                           // allowed to advance the letter — harmless)
        b.zulu_sec += 600.0; // time never makes a new edition
        assert_eq!(fingerprint(&a), fingerprint(&b));
        let mut c = snap();
        c.qnh_hpa = 1016.0;
        assert_ne!(fingerprint(&a), fingerprint(&c), "pressure change is a new edition");
        let mut d = snap();
        d.runway = "09".into();
        assert_ne!(fingerprint(&a), fingerprint(&d), "runway change is a new edition");
    }

    #[test]
    fn letters_advance_and_wrap() {
        assert_eq!(next_letter('A'), 'B');
        assert_eq!(next_letter('Z'), 'A');
        assert_eq!(nato('X'), "X-ray");
    }
}
