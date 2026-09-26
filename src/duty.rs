//! What the flight owes next: the copilot's own sequencing, from the one
//! state (`flight::Status`), as a pure function.
//!
//! Full auto is a hand-written script of this sequence, and every surprise
//! ends it in "your aircraft". A copilot that flies the flight on its own
//! instead asks, every tick, "given everything that is true now, what is the
//! next thing a crew would do?", and a failed step is simply a state the
//! next answer starts from.
//!
//! SHADOW ONLY (2026-09-25): the monitor asks it every tick, blackboxes the
//! answer when it changes (`duty`) beside whether the automation actually
//! running agrees, and `xvl debrief` lists the trail. It acts on nothing. A
//! duty earns the right to act once the shadow logs agree with the flights
//! the pilot and full auto actually flew.

use crate::flight::{Abnormal, Control, Phase, Status};

/// The next thing the flight owes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Duty {
    /// Nothing is ours to do: the pilot's own flight, or no plan.
    Monitor,
    /// The pilot took the aircraft back: watch, never act.
    PilotHasIt,
    StartEngines,
    RequestClearance,
    Taxi,
    LineUp,
    RequestTakeoff,
    TakeOff,
    Climb,
    /// Airborne with a plan and nothing steering it at a fix.
    Rejoin,
    FlyThePlan,
    Land,
    GoAround,
    Vacate,
    TaxiToStand,
    /// A helicopter slow in the air: to the pad, or down where it is.
    LandAtPad,
    /// The flight is over.
    Done,
    /// Wrecked or declared: no sequencing at all.
    Emergency,
}

impl Duty {
    pub fn name(self) -> &'static str {
        match self {
            Duty::Monitor => "monitor",
            Duty::PilotHasIt => "pilot_has_it",
            Duty::StartEngines => "start_engines",
            Duty::RequestClearance => "request_clearance",
            Duty::Taxi => "taxi",
            Duty::LineUp => "line_up",
            Duty::RequestTakeoff => "request_takeoff",
            Duty::TakeOff => "take_off",
            Duty::Climb => "climb",
            Duty::Rejoin => "rejoin",
            Duty::FlyThePlan => "fly_the_plan",
            Duty::Land => "land",
            Duty::GoAround => "go_around",
            Duty::Vacate => "vacate",
            Duty::TaxiToStand => "taxi_to_stand",
            Duty::LandAtPad => "land_at_pad",
            Duty::Done => "done",
            Duty::Emergency => "emergency",
        }
    }
}

/// A takeover this recent still stands: the pilot is flying.
pub const TAKEOVER_STANDS_S: f64 = 120.0;

/// The next duty and the fact that decided it. Pure: `now_s` is the
/// status's own clock (the flight model's sample time). `landed` is the
/// model's memory of a real touchdown this flight.
pub fn next(s: &Status, landed: bool, now_s: f64) -> (Duty, &'static str) {
    use Phase::*;
    match &s.abnormal {
        Abnormal::Crashed => return (Duty::Emergency, "the aircraft is wrecked"),
        Abnormal::Emergency => return (Duty::Emergency, "an emergency is declared"),
        _ => {}
    }
    if let Some((t, _)) = s.situation.pilot_takeover {
        if now_s - t < TAKEOVER_STANDS_S && matches!(s.control, Control::Pilot | Control::Autopilot) {
            return (Duty::PilotHasIt, "the pilot took the aircraft back");
        }
    }
    let plan = s.situation.plan.is_some();
    let mission = s.situation.mission.is_some();
    if !plan && !mission {
        return (Duty::Monitor, "no plan and no mission: the pilot's own flight");
    }
    let clearance = s.situation.clearance.as_ref().map(|c| c.state.as_str());
    let towered = s.situation.towered.unwrap_or(false);
    match s.phase {
        Parked if landed => (Duty::Done, "parked after the landing"),
        Parked => (Duty::StartEngines, "parked with a plan"),
        EngineStart => (Duty::StartEngines, "the start is under way"),
        Ready | Pushback | TaxiOut if landed => (Duty::TaxiToStand, "down, heading in"),
        Ready | Pushback if towered && clearance.is_none() => {
            (Duty::RequestClearance, "engines running, a tower, no clearance")
        }
        Ready | Pushback | TaxiOut => (Duty::Taxi, "engines running, to the runway"),
        // Stopped on the way out but not AT the hold (the taxi stood down
        // short of it at LIME and this read "at the hold", 2026-09-25).
        Holding if s.situation.at_hold == Some(false) => {
            (Duty::Taxi, "stopped short of the holding point")
        }
        Holding if towered && !matches!(clearance, Some("LinedUp" | "TakeoffCleared")) => {
            (Duty::RequestTakeoff, "at the hold, the tower's word needed")
        }
        Holding => (Duty::LineUp, "at the hold, cleared or no tower"),
        LinedUp if towered && clearance != Some("TakeoffCleared") => {
            (Duty::RequestTakeoff, "lined up, no takeoff clearance")
        }
        LinedUp => (Duty::TakeOff, "lined up and free to go"),
        TakeoffRoll => (Duty::TakeOff, "on the roll"),
        RejectedTakeoff => (Duty::Taxi, "rejected: clear the runway, then decide"),
        InitialClimb | Climb => (Duty::Climb, "climbing out"),
        Cruise | Descent if s.situation.active_fix.is_some() => {
            (Duty::FlyThePlan, "a fix is being flown")
        }
        Cruise | Descent if plan => (Duty::Rejoin, "airborne with a plan, no fix being flown"),
        Cruise | Descent => (Duty::Monitor, "airborne, no plan"),
        Approach => (Duty::Land, "on the approach"),
        Landing => (Duty::Land, "in the flare"),
        GoAround => (Duty::GoAround, "going around"),
        Hover => (Duty::LandAtPad, "slow in the air"),
        Rollout => (Duty::Vacate, "on the landing roll"),
        TaxiIn => (Duty::TaxiToStand, "down, heading in"),
    }
}

/// Whether the automation running now is doing the duty (the shadow's
/// agreement). `None` when nothing is running or the duty is not one an
/// automation performs: no verdict.
pub fn agrees(d: Duty, control: Control) -> Option<bool> {
    let Control::Automation(running) = control else {
        return None;
    };
    let ok: &[&str] = match d {
        Duty::StartEngines | Duty::RequestClearance | Duty::RequestTakeoff => &["full auto"],
        Duty::Taxi | Duty::TaxiToStand | Duty::Vacate => &["taxi", "pushback", "full auto"],
        Duty::LineUp => &["taxi", "full auto"],
        Duty::TakeOff | Duty::Climb => &["takeoff", "circuit", "full auto", "follower"],
        Duty::FlyThePlan | Duty::Rejoin => &["follower", "full auto", "helicopter route"],
        Duty::Land | Duty::GoAround => {
            &["autoland", "circuit", "follower", "full auto", "helicopter route", "hover"]
        }
        Duty::LandAtPad => &["hover", "helicopter route", "helicopter circuit"],
        _ => return None,
    };
    Some(ok.contains(&running))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight::{ClearanceFact, PlanFact, Situation};

    fn status(phase: Phase, towered: bool, clearance: Option<&str>) -> Status {
        Status {
            phase,
            control: Control::Automation("full auto"),
            abnormal: Abnormal::Normal,
            situation: Situation {
                mission: Some("full auto"),
                plan: Some(PlanFact {
                    source: "simbrief",
                    origin: "LILB".into(),
                    destination: "LIPK".into(),
                    cruise_alt_ft: 25000.0,
                }),
                clearance: clearance.map(|c| ClearanceFact {
                    state: c.into(),
                    runway: None,
                    squawk: None,
                    readback_owed: false,
                }),
                towered: Some(towered),
                ..Situation::default()
            },
            alt_ft: 0.0,
            ias_kts: 0.0,
            gs_kts: 0.0,
            peak_alt_ft: 0.0,
        }
    }

    #[test]
    fn a_whole_departure_in_order_towered_and_not() {
        let d = |p, tw, c| next(&status(p, tw, c), false, 0.0).0;
        assert_eq!(d(Phase::Parked, true, None), Duty::StartEngines);
        assert_eq!(d(Phase::Ready, true, None), Duty::RequestClearance);
        assert_eq!(d(Phase::Ready, true, Some("TaxiCleared")), Duty::Taxi);
        assert_eq!(d(Phase::Holding, true, Some("TaxiCleared")), Duty::RequestTakeoff);
        assert_eq!(d(Phase::Holding, true, Some("TakeoffCleared")), Duty::LineUp);
        assert_eq!(d(Phase::LinedUp, true, Some("LinedUp")), Duty::RequestTakeoff);
        assert_eq!(d(Phase::LinedUp, true, Some("TakeoffCleared")), Duty::TakeOff);
        // LILB: no tower in reach, nobody to ask.
        assert_eq!(d(Phase::Ready, false, None), Duty::Taxi);
        assert_eq!(d(Phase::Holding, false, None), Duty::LineUp);
        assert_eq!(d(Phase::LinedUp, false, None), Duty::TakeOff);
    }

    #[test]
    fn a_stop_short_of_the_hold_is_still_the_taxi() {
        let mut s = status(Phase::Holding, true, Some("TaxiCleared"));
        s.situation.at_hold = Some(false);
        assert_eq!(next(&s, false, 0.0).0, Duty::Taxi);
        s.situation.at_hold = Some(true);
        assert_eq!(next(&s, false, 0.0).0, Duty::RequestTakeoff);
    }

    #[test]
    fn the_air_and_the_arrival() {
        let d = |p| next(&status(p, true, None), false, 0.0).0;
        assert_eq!(d(Phase::Climb), Duty::Climb);
        assert_eq!(d(Phase::Cruise), Duty::Rejoin, "a plan and no fix flown");
        let mut s = status(Phase::Cruise, true, None);
        s.situation.active_fix = Some("GIPUL".into());
        assert_eq!(next(&s, false, 0.0).0, Duty::FlyThePlan);
        assert_eq!(d(Phase::Approach), Duty::Land);
        assert_eq!(d(Phase::GoAround), Duty::GoAround);
        assert_eq!(d(Phase::Rollout), Duty::Vacate);
        assert_eq!(d(Phase::TaxiIn), Duty::TaxiToStand);
        assert_eq!(next(&status(Phase::Parked, true, None), true, 0.0).0, Duty::Done);
        // The S-76 hovering 1.3 km short of LIME's pad (2026-09-25).
        assert_eq!(d(Phase::Hover), Duty::LandAtPad);
    }

    #[test]
    fn the_pilot_and_the_wreck_outrank_everything() {
        let mut s = status(Phase::Cruise, true, None);
        s.control = Control::Pilot;
        s.situation.pilot_takeover = Some((100.0, "my controls"));
        assert_eq!(next(&s, false, 150.0).0, Duty::PilotHasIt);
        assert_ne!(next(&s, false, 100.0 + TAKEOVER_STANDS_S + 1.0).0, Duty::PilotHasIt);
        s.abnormal = Abnormal::Crashed;
        assert_eq!(next(&s, false, 150.0).0, Duty::Emergency);
        let mut none = status(Phase::Cruise, true, None);
        none.situation.plan = None;
        none.situation.mission = None;
        assert_eq!(next(&none, false, 0.0).0, Duty::Monitor);
    }

    #[test]
    fn agreement_is_judged_only_against_a_running_automation() {
        assert_eq!(agrees(Duty::Taxi, Control::Automation("taxi")), Some(true));
        assert_eq!(agrees(Duty::TakeOff, Control::Automation("taxi")), Some(false));
        assert_eq!(agrees(Duty::Taxi, Control::Pilot), None);
        assert_eq!(agrees(Duty::Monitor, Control::Automation("taxi")), None);
    }
}
