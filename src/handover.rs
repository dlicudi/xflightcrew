//! "You have control": the copilot takes the aircraft and does what the
//! flight needs NOW, read from the one state (`flight::Status`). Pure.
//!
//! Before (2026-09-25) every whole-aircraft hand-over ("take the controls",
//! "you have control", "take over") armed the UPSET recovery: joystick
//! overrides and an unstall sequence, on a calm cruise or on the taxiway
//! alike. A crew hand-over is not an emergency. The recovery stays for an
//! actual upset and for the distress words ("recover", "save it", "help").
//!
//! The decision names tool calls the grammar already makes, so every gate
//! and readback of those tools still applies. The other direction ("my
//! controls", "I'll taxi", "let me fly") is the release guard in the main
//! loop; both directions land in the flight model's situation
//! (`flight::note_pilot_takeover` / `flight::note_crew_takeover`).

use crate::flight::{Abnormal, Control, Phase, Status};
use serde_json::{json, Value};

/// What the hand-over does.
#[derive(Debug, Clone, PartialEq)]
pub enum Handover {
    /// Run these calls in order. `lead` is said first when the calls' own
    /// readbacks would not say who has the aircraft.
    Take {
        calls: Vec<(&'static str, Value)>,
        lead: Option<&'static str>,
        why: &'static str,
    },
    /// Nothing to take (or not now): say why, the pilot keeps it.
    Decline { line: String, why: &'static str },
}

/// Live facts the flight model does not carry.
#[derive(Debug, Clone, Copy, Default)]
pub struct Live {
    /// Bank or pitch past what a crew would hand over in (see [`upset`]).
    pub upset: bool,
    /// One of our loops is flying (`monitor::flying_loop`), by name.
    pub running: Option<&'static str>,
    pub rotorcraft: bool,
}

/// An attitude no crew hands over in: the recovery's business.
pub fn upset(bank_deg: f64, pitch_deg: f64) -> bool {
    bank_deg.abs() > 45.0 || !(-20.0..=25.0).contains(&pitch_deg)
}

/// The `if_idle` argument: run this call only if nothing of ours took the
/// aircraft with the calls before it (a refused resume falls back to it).
pub const IF_IDLE: &str = "if_idle";

fn take(calls: Vec<(&'static str, Value)>, why: &'static str) -> Handover {
    Handover::Take { calls, lead: None, why }
}

fn decline(line: &str, why: &'static str) -> Handover {
    Handover::Decline { line: line.to_string(), why }
}

/// Hold the level we are at: wings level, altitude hold. Said with who has it.
fn hold_level(why: &'static str) -> Handover {
    Handover::Take {
        calls: vec![("level_off", json!({}))],
        lead: Some("I have control."),
        why,
    }
}

pub fn decide(s: Option<&Status>, live: &Live) -> Handover {
    let Some(s) = s else {
        return decline(
            "Give me a moment, I'm still reading the aircraft. You have control.",
            "no flight state yet",
        );
    };
    if s.abnormal == Abnormal::Crashed {
        return decline("The aircraft is wrecked. There's nothing to fly.", "crashed");
    }
    if let Some(name) = live.running.or(match s.control {
        Control::Automation(n) => Some(n),
        _ => None,
    }) {
        return Handover::Decline {
            line: format!("I already have it, the {name} is flying. Say my controls to take it back."),
            why: "an automation is already flying",
        };
    }
    let airborne = !matches!(
        s.phase,
        Phase::Parked
            | Phase::EngineStart
            | Phase::Ready
            | Phase::Pushback
            | Phase::TaxiOut
            | Phase::Holding
            | Phase::LinedUp
            | Phase::TakeoffRoll
            | Phase::RejectedTakeoff
            | Phase::Rollout
            | Phase::TaxiIn
    );
    if airborne && live.upset && !live.rotorcraft {
        return take(vec![("recover", json!({}))], "an upset");
    }
    let plan = s.situation.plan.is_some();
    // Resume the plan; a refused resume (off the route, below the sector
    // altitude) still leaves the aircraft held, never nobody's.
    let resume_or_hold = |why| Handover::Take {
        calls: vec![
            ("full_auto", json!({ "resume": true })),
            ("level_off", json!({ IF_IDLE: true })),
        ],
        lead: None,
        why,
    };
    if airborne && s.abnormal == Abnormal::Emergency && !live.rotorcraft {
        return take(vec![("autoland", json!({}))], "an emergency: land");
    }
    match s.phase {
        Phase::Parked => decline(
            "The engines are off. Say start the engines, or fly the flight plan and I'll do the lot.",
            "parked",
        ),
        Phase::EngineStart => decline("The start is still running. Ask me again once it's done.", "engine start"),
        Phase::Pushback => decline(
            "The tug has us. Ask me again once the bar is off.",
            "pushback",
        ),
        Phase::TakeoffRoll => decline(
            "Not on the roll. It's yours; ask me again once we're climbing.",
            "takeoff roll",
        ),
        Phase::Rollout => decline(
            "Keep it on the rollout. Ask me again once we're slow and I'll take us in.",
            "landing roll",
        ),
        Phase::Landing => decline("Too low to change hands. Your landing.", "in the flare"),
        _ if live.rotorcraft => match s.phase {
            Phase::Hover => take(vec![("hover", json!({ "action": "hold" }))], "hovering"),
            Phase::Climb | Phase::Cruise | Phase::Descent | Phase::Approach => {
                hold_level("a helicopter in the air")
            }
            _ => decline(
                "On the ground. Say take off, or taxi to parking.",
                "a helicopter on the ground",
            ),
        },
        // A stop at the holding point or on the runway is held, never rolled
        // from: the takeoff is its own ask.
        Phase::LinedUp => Handover::Take {
            calls: vec![("hold_speed", json!({ "knots": 0 }))],
            lead: Some("I have control. Holding on the runway; say take off for me when you're ready."),
            why: "lined up",
        },
        Phase::Holding if s.situation.at_hold == Some(true) => Handover::Take {
            calls: vec![("hold_speed", json!({ "knots": 0 }))],
            lead: Some("I have control. Holding short; say take off for me when you're ready."),
            why: "at the hold",
        },
        Phase::RejectedTakeoff => Handover::Take {
            calls: vec![("hold_speed", json!({ "knots": 0 }))],
            lead: Some("I have control. Stopping."),
            why: "a rejected takeoff",
        },
        Phase::Ready | Phase::TaxiOut | Phase::Holding => {
            take(vec![("auto_taxi", json!({}))], "on the ground, outbound")
        }
        Phase::TaxiIn => take(vec![("taxi_to_parking", json!({}))], "on the ground, inbound"),
        Phase::Approach => take(vec![("autoland", json!({}))], "on the approach"),
        Phase::InitialClimb | Phase::GoAround if plan => Handover::Take {
            calls: vec![("full_auto", json!({ "resume": true }))],
            lead: None,
            why: "climbing out with a plan",
        },
        // Levelling off under a thousand feet is not a take-over the crew
        // would make without being asked for it.
        Phase::InitialClimb | Phase::GoAround => decline(
            "Keep climbing it. Ask me again above a thousand feet, or say level off.",
            "climbing out low, no plan",
        ),
        Phase::Climb | Phase::Cruise | Phase::Descent if plan => resume_or_hold("in the air with a plan"),
        Phase::Climb | Phase::Cruise | Phase::Descent | Phase::Hover => hold_level("in the air, no plan"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight::{PlanFact, Situation};

    fn status(phase: Phase) -> Status {
        Status {
            phase,
            control: Control::Pilot,
            abnormal: Abnormal::Normal,
            situation: Situation::default(),
            alt_ft: 3000.0,
            ias_kts: 95.0,
            gs_kts: 95.0,
        }
    }

    fn with_plan(mut s: Status) -> Status {
        s.situation.plan = Some(PlanFact {
            source: "simbrief",
            origin: "LIPE".into(),
            destination: "LIPK".into(),
            cruise_alt_ft: 5000.0,
        });
        s
    }

    fn names(h: &Handover) -> Vec<&'static str> {
        match h {
            Handover::Take { calls, .. } => calls.iter().map(|(n, _)| *n).collect(),
            Handover::Decline { .. } => vec![],
        }
    }

    const CALM: Live = Live { upset: false, running: None, rotorcraft: false };

    /// The whole flight, phase by phase: a crew hand-over is never the
    /// upset recovery.
    #[test]
    fn a_hand_over_does_what_the_phase_needs() {
        let d = |p| names(&decide(Some(&status(p)), &CALM));
        assert_eq!(d(Phase::Parked), Vec::<&str>::new());
        assert_eq!(d(Phase::Ready), ["auto_taxi"]);
        assert_eq!(d(Phase::TaxiOut), ["auto_taxi"]);
        assert_eq!(d(Phase::Holding), ["auto_taxi"], "a stop short of the hold");
        assert_eq!(d(Phase::LinedUp), ["hold_speed"]);
        assert_eq!(d(Phase::TakeoffRoll), Vec::<&str>::new());
        assert_eq!(d(Phase::InitialClimb), Vec::<&str>::new());
        assert_eq!(d(Phase::Cruise), ["level_off"]);
        assert_eq!(d(Phase::Approach), ["autoland"]);
        assert_eq!(d(Phase::Landing), Vec::<&str>::new());
        assert_eq!(d(Phase::Rollout), Vec::<&str>::new());
        assert_eq!(d(Phase::TaxiIn), ["taxi_to_parking"]);
        let mut hold = status(Phase::Holding);
        hold.situation.at_hold = Some(true);
        assert_eq!(names(&decide(Some(&hold), &CALM)), ["hold_speed"]);
    }

    /// With a plan the crew rejoins it, and a refused resume still leaves
    /// the level held (LIPK 2026-09-25: resume refused below the MSA, and
    /// nobody was flying).
    #[test]
    fn with_a_plan_it_resumes_and_falls_back_to_holding_the_level() {
        let h = decide(Some(&with_plan(status(Phase::Cruise))), &CALM);
        assert_eq!(names(&h), ["full_auto", "level_off"]);
        let Handover::Take { calls, .. } = &h else { unreachable!() };
        assert_eq!(calls[0].1, json!({ "resume": true }));
        assert_eq!(calls[1].1, json!({ IF_IDLE: true }));
        assert_eq!(
            names(&decide(Some(&with_plan(status(Phase::InitialClimb))), &CALM)),
            ["full_auto"]
        );
    }

    #[test]
    fn the_recovery_is_for_an_upset_only() {
        let upset = Live { upset: true, ..CALM };
        assert_eq!(names(&decide(Some(&status(Phase::Cruise)), &upset)), ["recover"]);
        // On the ground the attitude means nothing.
        assert_eq!(names(&decide(Some(&status(Phase::TaxiOut)), &upset)), ["auto_taxi"]);
        assert!(upset_is(60.0, 0.0) && upset_is(0.0, 30.0) && upset_is(0.0, -25.0));
        assert!(!upset_is(25.0, 5.0));
    }

    fn upset_is(b: f64, p: f64) -> bool {
        super::upset(b, p)
    }

    #[test]
    fn a_running_loop_a_wreck_and_no_state_are_declined() {
        let running = Live { running: Some("taxi"), ..CALM };
        let h = decide(Some(&status(Phase::TaxiOut)), &running);
        assert!(matches!(&h, Handover::Decline { line, .. } if line.contains("the taxi is flying")));
        let mut wreck = status(Phase::Cruise);
        wreck.abnormal = Abnormal::Crashed;
        assert!(matches!(decide(Some(&wreck), &CALM), Handover::Decline { .. }));
        assert!(matches!(decide(None, &CALM), Handover::Decline { .. }));
    }

    #[test]
    fn a_helicopter_holds_its_hover_and_never_recovers() {
        let heli = Live { rotorcraft: true, upset: true, ..CALM };
        assert_eq!(names(&decide(Some(&status(Phase::Hover)), &heli)), ["hover"]);
        assert_eq!(names(&decide(Some(&status(Phase::Cruise)), &heli)), ["level_off"]);
        assert_eq!(names(&decide(Some(&status(Phase::Ready)), &heli)), Vec::<&str>::new());
    }

    #[test]
    fn a_declared_emergency_in_the_air_lands() {
        let mut s = with_plan(status(Phase::Cruise));
        s.abnormal = Abnormal::Emergency;
        assert_eq!(names(&decide(Some(&s), &CALM)), ["autoland"]);
    }
}
