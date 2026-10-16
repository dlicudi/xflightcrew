//! ICAO fuel declarations (Doc 4444 15.5.4, Annex 6, CAP 413 words): the
//! crew tells ATC when the fuel at landing nears or breaks the final reserve.
//!
//! - MINIMUM FUEL: committed to the destination, and any change to the
//!   clearance may land us below final reserve. No priority.
//! - MAYDAY FUEL: the fuel at landing is below final reserve. Distress.
//!
//! The prediction is the fuel callouts' own (`monitor::endurance_minutes`,
//! at the profile's cruise burn) less the time to the destination at this
//! groundspeed, as "How are we for fuel?" projects it. The final reserve is
//! the profile's `reserve_warning_min`. EGKK-EGPH (sortie
//! 1792158539-full_auto, 2026-10-16): two go-arounds, about 800 kg left,
//! and the crew never told ATC.

/// Minimum fuel is declared this many minutes above the final reserve at
/// landing: the margin a change to the clearance would eat.
pub const MIN_FUEL_MARGIN_MIN: f64 = 10.0;

/// A declaration is made once its condition has held this long, seconds:
/// a groundspeed blip is not a fuel state.
pub const SUSTAIN_S: f64 = 20.0;

/// Below this groundspeed, knots, the time to the field is no prediction.
const MIN_GS_KTS: f64 = 40.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Declaration {
    MinimumFuel,
    Mayday,
}

/// Minutes of fuel at landing: the endurance now less the time to fly
/// `dist_nm` at `gs_kts`. `None` without a groundspeed to time it by. Pure.
pub fn minutes_at_landing(endurance_min: f64, dist_nm: f64, gs_kts: f64) -> Option<f64> {
    (gs_kts > MIN_GS_KTS).then(|| endurance_min - dist_nm / gs_kts * 60.0)
}

/// The fuel at landing at the destination and the final reserve, minutes,
/// from the monitor's snapshot: airborne with an engine running, a
/// destination and a groundspeed, else `None`. Pure.
pub fn predict(
    s: &crate::monitor::Snapshot,
    fuel: &crate::aircraft::FuelProfile,
) -> Option<(f64, f64)> {
    if s.on_ground || !s.engine_running {
        return None;
    }
    let cruise = Some(fuel.cruise_burn_kg_hr).filter(|b| *b > 0.0);
    let endurance = crate::monitor::endurance_minutes(s.fuel_kg, s.fuel_flow_kg_sec, cruise)?;
    let dist = s.dest_route_nm.or(s.dest_dist_nm)?;
    let at_landing = minutes_at_landing(endurance, dist, s.groundspeed_kts)?;
    Some((at_landing, fuel.reserve_warning_min))
}

/// Which declaration the fuel at landing calls for. Pure.
pub fn due(at_landing_min: f64, final_reserve_min: f64) -> Option<Declaration> {
    if at_landing_min < final_reserve_min {
        Some(Declaration::Mayday)
    } else if at_landing_min < final_reserve_min + MIN_FUEL_MARGIN_MIN {
        Some(Declaration::MinimumFuel)
    } else {
        None
    }
}

/// What this flight has declared, and since when the condition has held.
/// Each declaration once per flight (the caller resets it on the ground); a
/// mayday makes a later minimum fuel call moot.
#[derive(Debug, Default)]
pub struct Declarations {
    minimum: bool,
    mayday: bool,
    since_s: Option<f64>,
}

impl Declarations {
    /// The declaration to make now, `t_s` a monotonic clock in seconds.
    /// Pure.
    pub fn tick(&mut self, due: Option<Declaration>, t_s: f64) -> Option<Declaration> {
        let Some(d) = due else {
            self.since_s = None;
            return None;
        };
        let since = *self.since_s.get_or_insert(t_s);
        if t_s - since < SUSTAIN_S {
            return None;
        }
        match d {
            Declaration::Mayday if !self.mayday => {
                self.mayday = true;
                self.minimum = true;
                Some(d)
            }
            Declaration::MinimumFuel if !self.minimum => {
                self.minimum = true;
                Some(d)
            }
            _ => None,
        }
    }
}

/// The copilot's line on the intercom: the crew declaring, or advising the
/// pilot who works the radio to declare.
pub fn intercom_line(d: Declaration, crew_declares: bool) -> &'static str {
    match (d, crew_declares) {
        (Declaration::MinimumFuel, true) => {
            "Fuel's close to final reserve at landing, declaring minimum fuel."
        }
        (Declaration::Mayday, true) => {
            "Fuel's below final reserve at landing, declaring mayday fuel."
        }
        (Declaration::MinimumFuel, false) => {
            "Fuel's close to final reserve at landing. Declare minimum fuel."
        }
        (Declaration::Mayday, false) => {
            "Fuel's below final reserve at landing. Declare a mayday, fuel."
        }
    }
}

/// The transmission, CAP 413: "MAYDAY MAYDAY MAYDAY FUEL" ahead of the
/// callsign; "<callsign>, minimum fuel".
pub fn radio_words(d: Declaration, callsign: &Option<String>) -> String {
    match d {
        Declaration::MinimumFuel => crate::crew_radio::radio_call(callsign, "minimum fuel"),
        Declaration::Mayday => match callsign {
            Some(cs) => format!("mayday, mayday, mayday, fuel, {cs}"),
            None => "mayday, mayday, mayday, fuel".to_string(),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_fuel_at_landing_is_the_endurance_less_the_time_to_the_field() {
        // 60 minutes aboard, 50 nm at 300 kt: 10 minutes to fly.
        assert_eq!(minutes_at_landing(60.0, 50.0, 300.0), Some(50.0));
        assert_eq!(minutes_at_landing(60.0, 50.0, 20.0), None);
    }

    #[test]
    fn minimum_fuel_near_the_reserve_mayday_below_it() {
        assert_eq!(due(45.0, 30.0), None);
        assert_eq!(due(40.0, 30.0), None);
        assert_eq!(due(39.0, 30.0), Some(Declaration::MinimumFuel));
        assert_eq!(due(30.0, 30.0), Some(Declaration::MinimumFuel));
        assert_eq!(due(29.0, 30.0), Some(Declaration::Mayday));
        // EGPH 2026-10-16: about 800 kg at a 2,500 kg/hr cruise burn is
        // 19 minutes, below the 737's 30-minute final reserve.
        assert_eq!(due(800.0 / 2500.0 * 60.0, 30.0), Some(Declaration::Mayday));
    }

    /// Each declaration once a flight, only once its condition has held,
    /// and never a minimum fuel after the mayday.
    #[test]
    fn each_declaration_is_made_once_after_it_has_held() {
        let mut d = Declarations::default();
        let min = Some(Declaration::MinimumFuel);
        let may = Some(Declaration::Mayday);
        assert_eq!(d.tick(min, 0.0), None);
        // A blip back above the margin restarts the clock.
        assert_eq!(d.tick(None, 10.0), None);
        assert_eq!(d.tick(min, 11.0), None);
        assert_eq!(d.tick(min, 11.0 + SUSTAIN_S), min);
        assert_eq!(d.tick(min, 60.0), None, "once");
        // Worse: the mayday, once, after the minimum fuel.
        assert_eq!(d.tick(may, 61.0), may);
        assert_eq!(d.tick(may, 200.0), None, "once");
        assert_eq!(d.tick(min, 300.0), None);
        // Straight to a mayday: no minimum fuel afterwards.
        let mut d = Declarations::default();
        assert_eq!(d.tick(may, 0.0), None);
        assert_eq!(d.tick(may, SUSTAIN_S), may);
        assert_eq!(d.tick(min, 100.0), None);
        assert_eq!(d.tick(min, 100.0 + SUSTAIN_S), None);
    }

    #[test]
    fn the_words_are_cap_413s() {
        let cs = Some("Speedbird one two three".to_string());
        assert_eq!(
            radio_words(Declaration::MinimumFuel, &cs),
            "Speedbird one two three, minimum fuel"
        );
        assert_eq!(
            radio_words(Declaration::Mayday, &cs),
            "mayday, mayday, mayday, fuel, Speedbird one two three"
        );
        // The tower hears what the crew says.
        assert_eq!(
            crate::tower::parse_intent(&radio_words(Declaration::MinimumFuel, &cs)),
            crate::tower::Intent::MinimumFuel
        );
        assert_eq!(
            crate::tower::parse_intent(&radio_words(Declaration::Mayday, &cs)),
            crate::tower::Intent::Mayday
        );
    }
}
