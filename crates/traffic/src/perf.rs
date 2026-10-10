//! Performance tables and wake categories: the numbers a traffic aircraft
//! flies by. One table per class, never per type; the type picks the class
//! ([`class_for_type`]). The piston single is the 172's book.

/// What one class of aircraft flies at. Speeds are knots (indicated and
/// ground alike in v1: no wind triangle), rates feet per minute, distances
/// nautical miles unless the name says otherwise.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Perf {
    /// The class name, for logs and tests.
    pub name: &'static str,
    /// Straight-line taxi pace.
    pub taxi_kts: f64,
    /// The most the nose turns on the ground, degrees a second.
    pub turn_taxi_dps: f64,
    /// Rotation speed.
    pub vr_kts: f64,
    /// The initial climb speed (V2, or Vx-ish for a light aircraft).
    pub v2_kts: f64,
    /// Climb rate.
    pub climb_fpm: f64,
    /// Climb speed (Vy for a light aircraft).
    pub climb_kts: f64,
    pub cruise_kts: f64,
    pub descent_fpm: f64,
    /// Final approach speed.
    pub app_kts: f64,
    /// Over the threshold.
    pub vref_kts: f64,
    /// Rollout deceleration, knots a second.
    pub decel_kts_s: f64,
    /// Circuit height above the field.
    pub pattern_agl_ft: f64,
    /// The downwind's distance off the runway centreline.
    pub downwind_offset_nm: f64,
    /// How far past the threshold the downwind runs before the base turn.
    pub base_ext_nm: f64,
    /// Takeoff-roll acceleration, knots a second.
    pub roll_accel_kts_s: f64,
    /// The most the aircraft turns in the air, degrees a second (a light
    /// aircraft banks about 20 degrees round a circuit: 5 at 85 kt).
    pub turn_air_dps: f64,
    /// Speed on the circuit's downwind.
    pub pattern_kts: f64,
    /// The gear comes up after takeoff (a 172 has none to raise).
    pub retractable_gear: bool,
    /// Wingspan, metres: the corridor a taxi keeps clear
    /// ([`crate::spacing`]).
    pub span_m: f64,
    /// Overall length, metres.
    pub length_m: f64,
    /// The gap kept behind the aircraft ahead on a taxiway, tail to nose,
    /// metres (a jet's is its blast).
    pub taxi_gap_m: f64,
    /// The speed a rapid (acute) exit is taken at, knots: a square one is
    /// taken at taxi pace. A jet leaves a rapid exit at 40 to 60 kt, a
    /// turboprop at 30 to 40, a light aircraft turns off at taxi pace on
    /// whatever exit it reaches.
    pub exit_kts: f64,
    /// The braking a rollout plans its exit on, metres a second squared
    /// (an autobrake's medium setting for a jet): the first exit past the
    /// point it brings the aircraft down to the exit's speed is the one
    /// taken. Under the class's [`Perf::decel_kts_s`], which is the most it
    /// brakes.
    pub exit_decel_ms2: f64,
    /// Nose gear to main gear, metres, for a type not in
    /// [`type_wheelbase_m`]'s table ([`crate::kin::Gear`]).
    pub wheelbase_m: f64,
    /// The main gear behind the published point (mid-length, where the
    /// model is drawn and the spacing measures from), metres; negative
    /// ahead.
    pub mains_aft_m: f64,
}

/// A wake turbulence category (ICAO: L, M, H, J).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Wake {
    Light,
    Medium,
    Heavy,
    Super,
}

impl Wake {
    /// The ICAO letter.
    pub fn letter(self) -> char {
        match self {
            Wake::Light => 'L',
            Wake::Medium => 'M',
            Wake::Heavy => 'H',
            Wake::Super => 'J',
        }
    }
}

/// The Cessna 172S: taxi 10 kt, rotate 55, Vy 74 at 700 fpm, approach 65,
/// 61 over the fence, a 1,000 ft circuit (POH figures, rounded).
pub const PISTON_SINGLE: Perf = Perf {
    name: "piston single",
    taxi_kts: 10.0,
    turn_taxi_dps: 20.0,
    vr_kts: 55.0,
    v2_kts: 62.0,
    climb_fpm: 700.0,
    climb_kts: 74.0,
    cruise_kts: 110.0,
    descent_fpm: 500.0,
    app_kts: 65.0,
    vref_kts: 61.0,
    decel_kts_s: 3.0,
    pattern_agl_ft: 1000.0,
    downwind_offset_nm: 0.8,
    base_ext_nm: 1.0,
    roll_accel_kts_s: 3.5,
    turn_air_dps: 5.0,
    pattern_kts: 85.0,
    retractable_gear: false,
    span_m: 11.0,
    length_m: 8.3,
    taxi_gap_m: 30.0,
    exit_kts: 12.0,
    exit_decel_ms2: 1.5,
    // The 172: 1.65 m (65 in) from the nosewheel to the mains, which stand
    // about 2.8 m behind the spinner, 1.3 m ahead of mid-length.
    wheelbase_m: 1.65,
    mains_aft_m: -1.3,
};

/// A King Air or ATR class turboprop.
pub const TURBOPROP: Perf = Perf {
    name: "turboprop",
    taxi_kts: 14.0,
    turn_taxi_dps: 15.0,
    vr_kts: 105.0,
    v2_kts: 115.0,
    climb_fpm: 1500.0,
    climb_kts: 160.0,
    cruise_kts: 260.0,
    descent_fpm: 1500.0,
    app_kts: 120.0,
    vref_kts: 110.0,
    decel_kts_s: 4.0,
    pattern_agl_ft: 1500.0,
    downwind_offset_nm: 1.5,
    base_ext_nm: 1.5,
    roll_accel_kts_s: 4.0,
    turn_air_dps: 4.0,
    pattern_kts: 160.0,
    retractable_gear: true,
    span_m: 27.0,
    length_m: 27.0,
    taxi_gap_m: 35.0,
    exit_kts: 35.0,
    exit_decel_ms2: 1.5,
    // The ATR 72: 10.8 m, its mains under the wing's trailing edge at
    // about mid-length (27.2 m).
    wheelbase_m: 10.8,
    mains_aft_m: 0.0,
};

/// An A320 or 737 class jet.
pub const JET_MED: Perf = Perf {
    name: "medium jet",
    taxi_kts: 18.0,
    turn_taxi_dps: 10.0,
    vr_kts: 145.0,
    v2_kts: 155.0,
    climb_fpm: 2500.0,
    climb_kts: 250.0,
    cruise_kts: 450.0,
    descent_fpm: 2000.0,
    app_kts: 145.0,
    vref_kts: 137.0,
    decel_kts_s: 5.0,
    pattern_agl_ft: 1500.0,
    downwind_offset_nm: 2.5,
    base_ext_nm: 3.0,
    roll_accel_kts_s: 4.0,
    turn_air_dps: 3.0,
    pattern_kts: 210.0,
    retractable_gear: true,
    span_m: 36.0,
    length_m: 38.0,
    taxi_gap_m: 40.0,
    exit_kts: 50.0,
    exit_decel_ms2: 1.75,
    // Between the A320's 12.6 m and the 737-800's 15.6 m; the mains are
    // within a metre of mid-length on both (A320 17.7 m behind the nose
    // of 37.6, 737-800 about 20 of 39.5).
    wheelbase_m: 14.0,
    mains_aft_m: 0.0,
};

/// A 777 or A330 class jet.
pub const JET_HEAVY: Perf = Perf {
    name: "heavy jet",
    taxi_kts: 16.0,
    turn_taxi_dps: 8.0,
    vr_kts: 160.0,
    v2_kts: 170.0,
    climb_fpm: 2000.0,
    climb_kts: 250.0,
    cruise_kts: 480.0,
    descent_fpm: 2000.0,
    app_kts: 155.0,
    vref_kts: 145.0,
    decel_kts_s: 4.5,
    pattern_agl_ft: 1500.0,
    downwind_offset_nm: 3.0,
    base_ext_nm: 3.5,
    roll_accel_kts_s: 3.0,
    turn_air_dps: 3.0,
    pattern_kts: 220.0,
    retractable_gear: true,
    span_m: 65.0,
    length_m: 70.0,
    taxi_gap_m: 50.0,
    exit_kts: 45.0,
    exit_decel_ms2: 1.6,
    // The A330-300's 25.4 m, the 777-200's 25.9, the 747-400's 25.6; the
    // mains near mid-length again (the 777-300ER's 31.2 is in the table).
    wheelbase_m: 26.0,
    mains_aft_m: 0.0,
};

/// The class table and wake category for an ICAO type designator. An
/// unknown type is a piston single, light: the commonest thing in a
/// circuit, and the slowest, so a mistake errs on the cautious side.
pub fn class_for_type(icao_type: &str) -> (&'static Perf, Wake) {
    let t = icao_type.trim().to_ascii_uppercase();
    const HEAVY: &[&str] = &[
        "B744", "B748", "B763", "B764", "B772", "B77W", "B77L", "B788", "B789", "B78X", "A332",
        "A333", "A339", "A343", "A346", "A359", "A35K", "MD11",
    ];
    const SUPER: &[&str] = &["A388"];
    const MEDIUM_JET: &[&str] = &[
        "A318", "A319", "A320", "A321", "A19N", "A20N", "A21N", "B736", "B737", "B738", "B739",
        "B38M", "B39M", "B752", "B753", "E170", "E175", "E190", "E195", "CRJ7", "CRJ9", "MD82",
        "MD83", "MD88", "BCS1", "BCS3",
    ];
    const TURBOPROPS: &[&str] = &[
        "AT43", "AT45", "AT72", "AT76", "DH8A", "DH8B", "DH8C", "DH8D", "B350", "BE20", "BE9L",
        "C208", "PC12", "SF34", "D328", "JS41",
    ];
    if SUPER.contains(&t.as_str()) {
        (&JET_HEAVY, Wake::Super)
    } else if HEAVY.contains(&t.as_str()) {
        (&JET_HEAVY, Wake::Heavy)
    } else if MEDIUM_JET.contains(&t.as_str()) {
        (&JET_MED, Wake::Medium)
    } else if TURBOPROPS.contains(&t.as_str()) {
        // A King Air or a Caravan is light; an ATR or a Dash 8 is medium.
        let light = matches!(t.as_str(), "B350" | "BE20" | "BE9L" | "C208" | "PC12");
        (&TURBOPROP, if light { Wake::Light } else { Wake::Medium })
    } else {
        (&PISTON_SINGLE, Wake::Light)
    }
}

/// A type's wingspan, metres, from its manufacturer's figures (rounded).
/// `None` for a type not in the table.
fn type_span_m(icao_type: &str) -> Option<f64> {
    let span = match icao_type {
        "C152" => 10.1,
        "C172" => 11.0,
        "PA28" => 10.7,
        "SR22" => 11.7,
        "B350" => 17.7,
        "BE20" => 16.6,
        "BE9L" => 15.3,
        "C208" => 15.9,
        "PC12" => 16.3,
        "SF34" => 21.4,
        "D328" => 21.0,
        "JS41" => 18.4,
        "AT43" | "AT45" => 24.6,
        "AT72" | "AT76" => 27.1,
        "DH8A" | "DH8B" => 25.9,
        "DH8C" => 27.4,
        "DH8D" => 28.4,
        "CRJ7" => 23.2,
        "CRJ9" => 24.9,
        "E170" | "E175" => 26.0,
        "E190" | "E195" => 28.7,
        "BCS1" | "BCS3" => 35.1,
        "A318" | "A319" | "A320" | "A321" => 34.1,
        "A19N" | "A20N" | "A21N" => 35.8,
        "B736" | "B737" | "B738" | "B739" => 34.3,
        "B38M" | "B39M" => 35.9,
        "MD82" | "MD83" | "MD88" => 32.9,
        "B752" | "B753" => 38.1,
        "B763" | "B764" => 47.6,
        "MD11" => 51.7,
        "A332" | "A333" | "A343" | "A346" => 60.3,
        "A339" => 64.0,
        "A359" | "A35K" => 64.8,
        "B772" | "B77L" => 60.9,
        "B77W" => 64.8,
        "B788" | "B789" | "B78X" => 60.1,
        "B744" => 64.4,
        "B748" => 68.4,
        "A388" => 79.8,
        _ => return None,
    };
    Some(span)
}

/// The shortest runway `icao_type` is scheduled on, metres: its takeoff
/// or landing field length at the weights a short sector flies (the
/// manufacturers' airport planning charts, rounded down to the strips the
/// types use every day). One floor for every jet was 1,800 m and barred
/// the A320s easyJet and BA fly into Gibraltar's 1,680 m (2026-10-09); an
/// A320 or a 737-800 needs about 1,600 m. A type not in the table takes
/// its class's (a widebody 2,500 m, a medium jet 1,800, a turboprop 1,100,
/// a light aircraft 600). The schedule also holds the type's good landing
/// ([`crate::aircraft::landing_distance_m`]) to it, whichever is longer.
pub fn runway_min_m(icao_type: &str) -> f64 {
    let t = icao_type.trim().to_ascii_uppercase();
    match t.as_str() {
        "SF34" | "D328" | "JS41" => 1000.0,
        "AT43" | "AT45" | "AT72" | "AT76" => 1100.0,
        "DH8A" | "DH8B" | "DH8C" | "DH8D" => 1200.0,
        "E170" | "E175" | "E190" | "E195" | "CRJ7" | "CRJ9" => 1500.0,
        "A318" | "A319" | "BCS1" | "BCS3" | "B736" | "B737" => 1500.0,
        "A320" | "A20N" | "B738" | "B38M" => 1600.0,
        "A321" | "A21N" | "B739" | "B39M" | "B752" | "B753" => 1800.0,
        "B763" | "B764" | "A332" | "A333" | "A339" => 2400.0,
        "B788" | "B789" | "B78X" | "A359" | "A35K" | "B772" => 2500.0,
        "B77W" | "B77L" | "A343" | "A346" | "B744" | "B748" | "MD11" => 2800.0,
        "A388" => 3000.0,
        _ => {
            let p = class_for_type(&t).0;
            if p.name == JET_HEAVY.name {
                2500.0
            } else if p.name == JET_MED.name {
                1800.0
            } else if p.name == TURBOPROP.name {
                1100.0
            } else {
                600.0
            }
        }
    }
}

/// A type's wingspan, metres: its own ([`type_span_m`]), else its class's.
pub fn span_of(icao_type: &str) -> f64 {
    let t = icao_type.trim().to_ascii_uppercase();
    type_span_m(&t).unwrap_or_else(|| class_for_type(&t).0.span_m)
}

/// The ICAO aerodrome reference code letter for a wingspan: 'A' under
/// 15 m, 'B' under 24, 'C' under 36, 'D' under 52, 'E' under 65, else 'F'.
pub fn span_code(span_m: f64) -> char {
    match span_m {
        s if s < 15.0 => 'A',
        s if s < 24.0 => 'B',
        s if s < 36.0 => 'C',
        s if s < 52.0 => 'D',
        s if s < 65.0 => 'E',
        _ => 'F',
    }
}

/// The ICAO wingspan code of `icao_type`: the stand size it needs (apt.dat's
/// 1301 row sizes a stand the same way). A type not in the table is judged
/// by its class's span ([`class_for_type`]).
pub fn wingspan_code(icao_type: &str) -> char {
    let t = icao_type.trim().to_ascii_uppercase();
    span_code(type_span_m(&t).unwrap_or_else(|| class_for_type(&t).0.span_m))
}

/// Nose gear to main gear, metres, from the manufacturers' airport
/// planning figures (rounded; a bogie's centre). `None` for a type not in
/// the table: its class's figure stands. Matched without allocating, as
/// it is asked every taxi substep.
pub fn type_wheelbase_m(icao_type: &str) -> Option<f64> {
    const TABLE: &[(&str, f64)] = &[
        ("C172", 1.65),
        ("AT43", 8.8),
        ("AT45", 8.8),
        ("AT72", 10.8),
        ("AT76", 10.8),
        ("A319", 11.0),
        ("A19N", 11.0),
        ("A320", 12.6),
        ("A20N", 12.6),
        ("A321", 16.9),
        ("A21N", 16.9),
        ("B737", 12.6),
        ("B738", 15.6),
        ("B38M", 15.6),
        ("B739", 17.2),
        ("B752", 18.3),
        ("B763", 22.8),
        ("A332", 22.2),
        ("A333", 25.4),
        ("B772", 25.9),
        ("B77W", 31.2),
        ("B744", 25.6),
    ];
    let t = icao_type.trim();
    TABLE
        .iter()
        .find(|(k, _)| k.eq_ignore_ascii_case(t))
        .map(|&(_, w)| w)
}

/// The taxi geometry of `icao_type` flying `perf`: its own wheelbase where
/// the table has it, the class's mains and pace.
pub fn gear_for(icao_type: &str, perf: &Perf) -> crate::kin::Gear {
    crate::kin::Gear {
        wheelbase_m: type_wheelbase_m(icao_type).unwrap_or(perf.wheelbase_m),
        mains_aft_m: perf.mains_aft_m,
        pace_kts: perf.taxi_kts,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A stand is sized by the wingspan code, and the schedule's types each
    /// have theirs: the light aircraft A, the regional turboprop and the
    /// narrowbodies C, a widebody E, the A380 F; an unknown type by its class.
    #[test]
    fn every_type_has_the_wingspan_code_of_its_span() {
        for t in ["C172", "C152", "PA28", "SR22"] {
            assert_eq!(wingspan_code(t), 'A', "{t}");
        }
        for t in ["AT76", "A320", "A319", "B738", "E190"] {
            assert_eq!(wingspan_code(t), 'C', "{t}");
        }
        assert_eq!(wingspan_code("b738"), 'C', "any case");
        assert_eq!(wingspan_code("C208"), 'B');
        assert_eq!(wingspan_code("B763"), 'D');
        assert_eq!(wingspan_code("B77W"), 'E');
        assert_eq!(wingspan_code("A388"), 'F');
        assert_eq!(wingspan_code("ZZZZ"), span_code(PISTON_SINGLE.span_m));
        assert_eq!(span_code(14.9), 'A');
        assert_eq!(span_code(15.0), 'B');
        assert_eq!(span_code(36.0), 'D');
    }

    #[test]
    fn the_172_flies_its_book() {
        let p = PISTON_SINGLE;
        assert_eq!(p.taxi_kts, 10.0);
        assert_eq!(p.vr_kts, 55.0);
        assert_eq!(p.climb_kts, 74.0);
        assert_eq!(p.climb_fpm, 700.0);
        assert_eq!(p.app_kts, 65.0);
        assert_eq!(p.vref_kts, 61.0);
        assert_eq!(p.pattern_agl_ft, 1000.0);
    }

    #[test]
    fn every_table_is_ordered_like_an_aircraft() {
        for p in [PISTON_SINGLE, TURBOPROP, JET_MED, JET_HEAVY] {
            assert!(p.vref_kts < p.app_kts, "{}", p.name);
            assert!(p.vr_kts < p.v2_kts && p.v2_kts <= p.climb_kts, "{}", p.name);
            assert!(p.climb_kts < p.cruise_kts, "{}", p.name);
            assert!(
                p.base_ext_nm > 0.0 && p.downwind_offset_nm > 0.0,
                "{}",
                p.name
            );
        }
    }

    #[test]
    fn a_type_picks_its_class_and_wake() {
        assert_eq!(class_for_type("C172").0.name, "piston single");
        assert_eq!(class_for_type("C172").1, Wake::Light);
        assert_eq!(class_for_type("b738").0.name, "medium jet");
        assert_eq!(class_for_type("B77W").1, Wake::Heavy);
        assert_eq!(class_for_type("A388").1, Wake::Super);
        assert_eq!(class_for_type("AT76"), (&TURBOPROP, Wake::Medium));
        assert_eq!(class_for_type("ZZZZ").0.name, "piston single");
    }
}
