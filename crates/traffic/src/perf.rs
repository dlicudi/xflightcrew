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

#[cfg(test)]
mod tests {
    use super::*;

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
