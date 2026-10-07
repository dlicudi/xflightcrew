//! The process-wide facts that belong to ONE airframe, and the one place they
//! are set and cleared.
//!
//! The recurring bug class in this tree is a static that outlives its airframe
//! or its process: a radio flag that stopped being published and latched
//! across a swap, a transponder knob selector that survived into a type
//! without one, a "we're at the hold" flag from the morning's taxi read as
//! true at 1,475 ft (EGHI, 2026-08-02). The cure the party line already uses
//! for the world's side is one page with the writes on it. This is the crew's.
//!
//! The doors:
//! - [`publish_airframe`]: every selector the tools read per airframe, from
//!   the profile, including to `None`. Called at boot and on every swap.
//! - [`stand_down`]: every loop that can hold the aircraft, stopped. Called
//!   on a swap (a loop cannot fly a different aircraft), on entering
//!   instructor mode (hands off), and when the sim connection is lost.
//!   It cancels every run in [`LOOPS`], and `stand_down_cancels_every_run`
//!   keeps that registry equal to every `sim::run::Slot` in the tree (and
//!   names the door of any loop flag not yet on a run), so a new loop
//!   cannot be forgotten here.
//! - [`flight_begins`]: THE new-flight door (ramp, approach and enroute
//!   setups, a foreign start, the model's reposition): stand-down, the
//!   crew's flight facts forgotten (`forget_flight`), the old checklist
//!   dropped, the party line retired; blackbox `flight_state`
//!   `flight_begins` with `why`.
//! - [`airframe_begins`]: boot and every swap: stand-down, then
//!   `publish_airframe`, then the flight forgotten (`airframe_begins`).
//!
//! Inventory of what is NOT yet cleared here and lives in its own module,
//! per FLIGHT rather than per airframe (each reset on its own path today):
//! `autotaxi::LAST_ARRIVED`, `autoland::VACATE_FOLLOWS`,
//! `full_auto::INBOUND_MADE` / `STAGE` / `MISSION`, `procedure::TOD` /
//! `TERRAIN_FLOOR` / `PILOT_ALT` / `LEG`, `taxi_in::ARREST_HOLD`,
//! `instructor::LESSON`, `flow::*`. `pushback::LEARNED_SENSE` is per SESSION
//! by design (the sim build's tug sense, learnt once), never per flight.

use crate::aircraft::AircraftProfile;

/// Publish the per-airframe selectors the tools read. Every one is set from
/// the profile, including to `None`: a process-wide fact must never outlive
/// its airframe, and a selector that is only set when present is the bug.
/// The facts are one snapshot ([`airframe_facts`]) installed at once; what
/// stays beside it are the claims and survey this airframe's flying earns
/// (systems, masks, the altimeter truth, anti-ice claims) and the mission.
pub(crate) fn publish_airframe(p: &AircraftProfile) {
    crate::aircraft::install_airframe_facts(airframe_facts(p));
    // The weight the stalls ride was the last airframe's: the declared
    // stalls fly until the monitor's next tick weighs this one.
    crate::aircraft::publish_weight_now(None);
    // The systems survey is the detector's, from the sim; a new airframe
    // starts knowing nothing about its avionics.
    crate::systems::publish(None);
    crate::full_auto::forget_mission();
    // A claim on the masks never outlives its airframe.
    crate::tools::cabin_air::forget_masks();
    // A scripted start's verdict (an abort latched the last load) is the
    // last airframe's.
    crate::tools::scripted_start::forget();
    // A plan's transition belongs to the flight it was armed for.
    crate::tools::instruments::publish_flight_transition(None);
    // The altimeter cross-check was about the last airframe's instruments.
    crate::tools::instruments::publish_altimeter_truth(None);
    // What the crew switched on was the last airframe's anti-ice.
    crate::tools::anti_ice::clear_claims();
}

/// A second process's view of the live airframe (`xvl conform`): the facts
/// the seams read, installed from the profile, and nothing else. It begins
/// no flight, stands nothing down and records nothing: the running xvl owns
/// all three.
pub(crate) fn install_for_check(p: &AircraftProfile) {
    crate::aircraft::install_airframe_facts(airframe_facts(p));
}

/// Every per-airframe fact the tools read, from the profile, as one
/// snapshot (`aircraft::AirframeFacts`). A literal naming every field, so a
/// fact added to the struct and forgotten here does not compile: the
/// "selector only set when present" bug cannot be written. The `.acf`
/// facts start unread (the detector updates them once it has the file).
pub(crate) fn airframe_facts(p: &AircraftProfile) -> crate::aircraft::AirframeFacts {
    use crate::aircraft as a;
    let vs0 = p.reference.as_ref().and_then(|r| r.vs0_kts);
    a::AirframeFacts {
        profile: Some(std::sync::Arc::new(p.clone())),
        fuel_unit: a::fuel_unit_for(p.fuel.as_ref()),
        throttle_dref: p.throttle_dataref.clone(),
        // The .acf's castoring nose is read at detection; a new airframe
        // starts from its profile alone.
        nose_castors: false,
        nose_free: false,
        gear_limit_kts: p.vlo_kts,
        flap_limits: a::flap_limits_for(p.flaps.as_ref()),
        bus_volts_nominal: p.bus_volts_nominal,
        speed_floor_kts: a::speed_floor_for(p.vs_clean_kts),
        speed_limits: a::SpeedLimits::new(p.vne_kts, p.vmo_high),
        approach_floor_kts: a::approach_floor_for(vs0, p.vs_clean_kts),
        stall_curve: a::stall_curve_for(p.vs_clean_kts, vs0, p.flaps.as_ref()),
        approach_thrust_min: a::approach_thrust_min_for(p.approach_thrust_min),
        arrest_lead: p
            .arrest_lead
            .and_then(a::ArrestLeadProfile::valid)
            .unwrap_or_default(),
        flare_elevator: p.flare_elevator.and_then(a::FlareElevatorProfile::valid),
        roll_rudder: p.roll_rudder.and_then(a::RollRudderProfile::valid),
        landing_distance_m: a::landing_distance_m_for(
            p.reference.as_ref().and_then(|r| r.landing_distance_ft),
        ),
        vs_limits: Some(crate::procedure::vs_limits_for(p.reference.as_ref())),
        // The profile's envelope; the detector reads the .acf's over a
        // silent profile.
        gear_warning: p.gear_warning,
        climb_out: a::ClimbOut::from_profile(p),
        takeoff_power: p.takeoff_power.clone(),
        takeoff_config: p.takeoff_config.clone(),
        ap_altitude_capture: p.ap_altitude_capture,
        // The rotor is the helicopter's: an aeroplane after the S-76 must
        // not hover taxi in, be "on the ground" to the tower aloft, or skip
        // a vacate.
        rotor: crate::hover::rotor_facts(p),
        signs: a::SignsSwitch::for_profile(p.seat_belt_signs.clone(), p.cabin_crew),
        cabin_air: p.cabin_air.clone(),
        mode_knob: p.transponder_mode_knob.clone(),
        exterior: a::Exterior::from_lights(&p.lights),
        altimeter: p.altimeter.clone(),
    }
}

/// What began a new flight: the cause [`flight_begins`] records and the
/// checklist's stand-down names.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum NewFlight {
    /// "Set up the flight": the plan's origin, cold and dark.
    Ramp,
    /// "Put us on a ten mile final".
    Approach,
    /// The sortie runner's airborne start on the plan.
    Enroute,
    /// A mailbox run the app did not post (`setup::foreign_new_flight`).
    Foreign,
    /// The flight model saw the aircraft jump (`flight::Event::Reposition`)
    /// with no setup of ours or anyone's accounting for it.
    Reposition,
}

impl NewFlight {
    /// Its name in the blackbox (`flight_state` `flight_begins` `why`).
    pub(crate) fn name(self) -> &'static str {
        match self {
            NewFlight::Ramp => "ramp",
            NewFlight::Approach => "approach",
            NewFlight::Enroute => "enroute",
            NewFlight::Foreign => "foreign",
            NewFlight::Reposition => "reposition",
        }
    }

    /// The cause the checklist's `stood_down` record names (the strings it
    /// wrote before the door: the setups said "setup", the model's jump
    /// "reposition").
    fn list_cause(self) -> &'static str {
        match self {
            NewFlight::Ramp | NewFlight::Approach | NewFlight::Enroute => "setup",
            NewFlight::Foreign => "foreign",
            NewFlight::Reposition => "reposition",
        }
    }
}

/// Bumped once per flight begun (and per airframe, which is a new flight
/// too): the generation a flight-scoped fact can be stamped with.
static FLIGHTS_BEGUN: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// The current flight's generation: what a `scoped::Scoped::flight()` cell
/// is stamped with. A value stamped before the last door reads as gone.
pub(crate) fn flight_gen() -> u64 {
    FLIGHTS_BEGUN.load(std::sync::atomic::Ordering::SeqCst)
}
/// When the last full door ran ([`flight_begins`] other than a reposition,
/// or [`airframe_begins`]): the flight model's jump that follows our own
/// setup is the same new flight, not another.
static LAST_BEGUN: std::sync::Mutex<Option<std::time::Instant>> = std::sync::Mutex::new(None);
/// A reposition seen this soon after a door is that door's flight: the
/// setups run theirs as the init returns, and the model sees the jump on its
/// first live tick after the load gate opens (a few seconds).
const REPOSITION_SAME_FLIGHT_S: u64 = 60;

/// What a reposition's door does: `None` runs it, else the reason it is the
/// flight a door already began. Pure, so the idempotency is a plain test.
fn reposition_skip(
    loading: bool,
    since_begun: Option<std::time::Duration>,
) -> Option<&'static str> {
    if loading {
        // A setup of ours (or the runner's start) is loading: its own door
        // runs when the init returns.
        return Some("loading");
    }
    if since_begun.is_some_and(|d| d.as_secs() < REPOSITION_SAME_FLIGHT_S) {
        return Some("begun");
    }
    None
}

/// THE NEW-FLIGHT DOOR. Every path that begins a new flight comes through
/// here, so the steps cannot drift apart again (the ramp setup forgot the
/// stand-down, the foreign start the checklist, the approach retired the
/// party line on its own; refactor audit 2026-10-04, phase 3).
///
/// A setup's flight (ramp, approach, enroute) and a foreign start: every
/// loop stood down (nothing keeps flying the flight that ended), the crew's
/// flight facts forgotten, the checklist of the old flight dropped, and with
/// ATC on the party line retired (its clearances were the old flight's).
/// Called once the new flight EXISTS (the init returned), before anything
/// arms on it. The approach and enroute setups also stand down before their
/// init, so nothing flies through the jump.
///
/// A reposition the model saw is deliberately lighter: only the checklist
/// (its behaviour before the door). The model's jump is also a replay's
/// rewind and its return, where standing the loops down or retiring a
/// landing clearance mid-final would be wrong; and a jump this app or the
/// runner made already ran its own door ([`reposition_skip`]).
pub(crate) fn flight_begins(why: NewFlight) {
    use std::sync::atomic::Ordering::SeqCst;
    if why == NewFlight::Reposition {
        let since = LAST_BEGUN
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .map(|t| t.elapsed());
        if let Some(skip) = reposition_skip(crate::tools::setup::new_flight_loading(), since) {
            crate::blackbox::record(
                "flight_state",
                serde_json::json!({ "event": "flight_begins", "why": why.name(), "skipped": skip }),
            );
            return;
        }
        let list = crate::flow::stand_down(crate::flow::StandDown::NewFlight(why.list_cause()));
        crate::blackbox::record(
            "flight_state",
            serde_json::json!({ "event": "flight_begins", "why": why.name(),
                                "list_stood_down": list }),
        );
        return;
    }
    stand_down();
    forget_flight();
    let list = crate::flow::stand_down(crate::flow::StandDown::NewFlight(why.list_cause()));
    let atc = crate::mode::atc_enabled();
    if atc {
        crate::party_line::retire_for_new_flight();
    }
    let flight = FLIGHTS_BEGUN.fetch_add(1, SeqCst) + 1;
    *LAST_BEGUN.lock().unwrap_or_else(|e| e.into_inner()) = Some(std::time::Instant::now());
    crate::blackbox::record(
        "flight_state",
        serde_json::json!({ "event": "flight_begins", "why": why.name(), "flight": flight,
                            "list_stood_down": list, "party_line_retired": atc }),
    );
}

/// THE NEW-AIRFRAME DOOR: at boot and on every swap. In order: every loop
/// stood down first (a loop armed on the old airframe must not fly the new
/// one with its speeds and limits), then every selector republished from the
/// new profile, then the old flight's facts forgotten (a swap reloads the
/// aircraft: the last landing's runway and the plan's destination were the
/// old airframe's flight).
pub(crate) fn airframe_begins(p: &AircraftProfile, why: &'static str) {
    use std::sync::atomic::Ordering::SeqCst;
    stand_down();
    publish_airframe(p);
    forget_flight();
    let flight = FLIGHTS_BEGUN.fetch_add(1, SeqCst) + 1;
    *LAST_BEGUN.lock().unwrap_or_else(|e| e.into_inner()) = Some(std::time::Instant::now());
    crate::blackbox::record(
        "flight_state",
        serde_json::json!({ "event": "airframe_begins", "why": why, "profile": p.id,
                            "flight": flight }),
    );
}

/// The crew's facts about the flight that ended, forgotten when a new one
/// begins. Only the doors call it ([`flight_begins`], [`airframe_begins`]);
/// the world's side is `party_line::retire_for_new_flight`, beside it there.
fn forget_flight() {
    // The plan's transition was the old flight's: the monitor's altimeter
    // check WRITES the setting with an automation flying.
    crate::tools::instruments::publish_flight_transition(None);
    // The runway the last autoland rolled out on and its tracking.
    crate::autoland::forget_flight();
    // A runway waiver or departure minimums were the old mission's.
    crate::full_auto::forget_mission();
    // The plan's destination is published again by the next plan read.
    crate::monitor::forget_destination();
    // "The pilot has it" was about the flight that ended.
    crate::flight::forget_pilot_takeover();
    // Calls and readbacks queued for the old flight go to nobody.
    crate::crew_radio::forget_flight_calls();
    // "Say again" never repeats the last flight's readback.
    crate::forget_last_readback();
    // A cart summoned on the old ramp, a light the pilot moved, an offer
    // the copilot made: the last flight's.
    crate::tools::ground::forget_cart_request();
    crate::cockpit_auto::forget_pilot_moved_lights();
    crate::rapport::clear_offer();
}

/// Every loop run in the tree (`sim::run::Slot`), by its path: THE registry
/// [`stand_down`] cancels. `stand_down_cancels_every_run` keeps it equal to
/// every `Slot` declared under `src`, so a new loop cannot outlive a swap by
/// being forgotten here (the old check matched names, and `BRAKES_HELD`, the
/// capture watcher and the start sequences slipped through it).
pub(crate) static LOOPS: [(&str, &sim::run::Slot); 22] = [
    ("full_auto::RUN", &crate::full_auto::RUN),
    ("procedure::RUN", &crate::procedure::RUN),
    ("autotakeoff::RUN", &crate::autotakeoff::RUN),
    ("autoland::RUN", &crate::autoland::RUN),
    ("circuit::RUN", &crate::circuit::RUN),
    ("tools::recover::RUN", &crate::tools::recover::RUN),
    ("taxi_in::RUN", &crate::taxi_in::RUN),
    ("autotaxi::RUN", &crate::autotaxi::RUN),
    ("pushback::PUSH", &crate::pushback::PUSH),
    ("hover::RUN", &crate::hover::RUN),
    ("heli_circuit::RUN", &crate::heli_circuit::RUN),
    ("heli_route::RUN", &crate::heli_route::RUN),
    ("tools::speedhold::GOVERNOR", &crate::tools::speedhold::GOVERNOR),
    ("tools::reverse::WATCH", &crate::tools::reverse::WATCH),
    ("tools::controls::BRAKE_HOLD", &crate::tools::controls::BRAKE_HOLD),
    ("tools::autopilot::CAPTURE", &crate::tools::autopilot::CAPTURE),
    ("tools::power::POWER_UP", &crate::tools::power::POWER_UP),
    ("tools::jet_start::JET_START", &crate::tools::jet_start::JET_START),
    ("tools::engine::MIXTURE_MANAGER", &crate::tools::engine::MIXTURE_MANAGER),
    ("tools::engine::LEAN_SEEK", &crate::tools::engine::LEAN_SEEK),
    ("ils::SETUP", &crate::ils::SETUP),
    ("flow::RUN", &crate::flow::RUN),
];

/// Every loop that can hold the aircraft, stopped. What a stand-down leaves
/// in the air is the safe hold's (`safe_hold`, never stood down): nobody
/// flying is held clear of the ground, never left on a loop's last command.
/// First the doors that do
/// more than cancel (end the mission and its children, free the brakes,
/// hand back a flyable aircraft, clear a backtrack or a hand-down), then
/// every run in [`LOOPS`] cancelled, then the loops not yet on a run.
pub(crate) fn stand_down() {
    crate::full_auto::abort();
    crate::circuit::abort();
    crate::procedure::abort();
    crate::autoland::abort();
    // The pitch-authority check was about the flight that ended (#181).
    crate::climb_watch::disarm_authority();
    crate::autotakeoff::cancel();
    crate::autotaxi::halt();
    crate::pushback::clear();
    // A new flight's setup owns the aircraft: the taxi-in only lets go.
    crate::taxi_in::abort(false);
    // Every run, whoever holds it: the recovery, the helicopter loops, the
    // governor, reverse, the brake hold (its held command expires), the
    // capture watcher (it writes the dial, ALT hold and throttle), a
    // power-up or jet start (stopped at its next step), the mixture manager
    // and a lean seek (their writes refused from now), the ILS setup watch
    // and a checklist (stopped where it is, its summary says so).
    for (_, run) in &LOOPS {
        run.cancel();
    }
    // Loops not on a run yet: each its own door.
    crate::approach_guide::cancel();
    // The arrival's hold on the station is a fact about a flight, not about
    // the world: nothing is flying to that destination any more, so the
    // region's ordinary nearest-field handoffs resume.
    crate::full_auto::release_arrival_station();
}

/// A loop of ours that can hold the aircraft. The ONE list of them: every
/// "is an automation flying?" question in the tree is a filter over it
/// ([`crew_has_aircraft`], [`crew_flying`], [`crew_mission`] and the named
/// narrower sets below). Nine inline copies with different members used to
/// answer it: the recovery was in none of the four "automation flying" ones,
/// and with the helicopter route or a stand-alone takeoff flying the monitor
/// treated a pilot as flying, advising where it should have reset the
/// altimeter (refactor audit 2026-10-04, duplicates #1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Owner {
    FullAuto,
    HeliRoute,
    Circuit,
    HeliCircuit,
    Procedure,
    Autoland,
    Autotakeoff,
    Autotaxi,
    Pushback,
    Hover,
    Recover,
}

impl Owner {
    /// Innermost first: the loop with its hands on the controls now (full
    /// auto flies through autoland, the takeoff and the taxi; a recovery
    /// takes the aircraft from whatever was flying it).
    pub(crate) const INNERMOST_FIRST: [Owner; 11] = [
        Owner::Recover,
        Owner::Autoland,
        Owner::Autotakeoff,
        Owner::Pushback,
        Owner::Autotaxi,
        Owner::Hover,
        Owner::Procedure,
        Owner::Circuit,
        Owner::HeliCircuit,
        Owner::HeliRoute,
        Owner::FullAuto,
    ];

    /// Outermost first: what the flight is FOR (the missions own every loop
    /// under them).
    pub(crate) const OUTERMOST_FIRST: [Owner; 11] = [
        Owner::FullAuto,
        Owner::HeliRoute,
        Owner::Circuit,
        Owner::HeliCircuit,
        Owner::Procedure,
        Owner::Autoland,
        Owner::Autotakeoff,
        Owner::Autotaxi,
        Owner::Pushback,
        Owner::Hover,
        Owner::Recover,
    ];

    /// Its loop flag reads set.
    pub(crate) fn active(self) -> bool {
        use std::sync::atomic::Ordering::Relaxed;
        match self {
            Owner::FullAuto => crate::full_auto::ACTIVE.load(Relaxed),
            Owner::HeliRoute => crate::heli_route::ACTIVE.load(Relaxed),
            Owner::Circuit => crate::circuit::ACTIVE.load(Relaxed),
            Owner::HeliCircuit => crate::heli_circuit::ACTIVE.load(Relaxed),
            Owner::Procedure => crate::procedure::ACTIVE.load(Relaxed),
            Owner::Autoland => crate::autoland::ACTIVE.load(Relaxed),
            Owner::Autotakeoff => crate::autotakeoff::ACTIVE.load(Relaxed),
            Owner::Autotaxi => crate::autotaxi::ACTIVE.load(Relaxed),
            Owner::Pushback => crate::pushback::ACTIVE.load(Relaxed),
            Owner::Hover => crate::hover::ACTIVE.load(Relaxed),
            Owner::Recover => crate::tools::recover::ACTIVE.load(Relaxed),
        }
    }

    /// Its loop is live: its flag reads set, or, for the two loops that keep
    /// a flag beside an unflagged run (full auto and the takeoff), its run
    /// still runs. Every other owner is live exactly when [`Owner::active`].
    #[cfg_attr(test, allow(dead_code))]
    pub(crate) fn live(self) -> bool {
        self.active()
            || match self {
                Owner::FullAuto => crate::full_auto::running(),
                Owner::Autotakeoff => crate::autotakeoff::running(),
                _ => false,
            }
    }

    /// Stand this one loop down through its own door, the one
    /// [`stand_down`] calls for it: a takeoff hands over with the power set,
    /// a taxi stops its drive, a circuit, a landing or a mission gives back a
    /// flyable aircraft. What the door says it stopped is the caller's to
    /// read off [`Owner::active`] first; nothing else is let go here.
    pub(crate) fn stop(self) {
        match self {
            Owner::FullAuto => {
                crate::full_auto::abort();
            }
            Owner::HeliRoute => crate::heli_route::abort(),
            Owner::Circuit => {
                crate::circuit::abort();
            }
            Owner::HeliCircuit => crate::heli_circuit::abort(),
            Owner::Procedure => {
                crate::procedure::abort();
            }
            Owner::Autoland => {
                crate::autoland::abort();
            }
            Owner::Autotakeoff => crate::autotakeoff::cancel(),
            Owner::Autotaxi => {
                crate::autotaxi::halt();
            }
            Owner::Pushback => {
                crate::pushback::clear();
            }
            Owner::Hover => crate::hover::abort(),
            Owner::Recover => {
                crate::tools::recover::abort();
            }
        }
    }

    /// Its name in the blackbox, the flight model's `Control::Automation`
    /// and the duty engine (`monitor::flying_loop`'s names, kept).
    pub(crate) fn name(self) -> &'static str {
        match self {
            Owner::FullAuto => "full auto",
            Owner::HeliRoute => "helicopter route",
            Owner::Circuit => "circuit",
            Owner::HeliCircuit => "helicopter circuit",
            Owner::Procedure => "follower",
            Owner::Autoland => "autoland",
            Owner::Autotakeoff => "takeoff",
            Owner::Autotaxi => "taxi",
            Owner::Pushback => "pushback",
            Owner::Hover => "hover",
            Owner::Recover => "recovery",
        }
    }

    /// Its name in a spoken sentence ("held by the circuit").
    pub(crate) fn spoken(self) -> &'static str {
        match self {
            Owner::FullAuto => "full auto",
            Owner::HeliRoute => "the route",
            Owner::Circuit | Owner::HeliCircuit => "the circuit",
            Owner::Procedure => "the follower",
            Owner::Autoland => "autoland",
            Owner::Autotakeoff => "the takeoff",
            Owner::Autotaxi => "the taxi",
            Owner::Pushback => "the pushback",
            Owner::Hover => "the hover",
            Owner::Recover => "the recovery",
        }
    }

    /// What the crew says it is doing, refusing a request that would take
    /// the aircraft from it ("I'm landing").
    pub(crate) fn doing(self) -> &'static str {
        match self {
            Owner::Autoland => "I'm landing",
            Owner::Circuit | Owner::HeliCircuit => "I'm flying the circuit",
            Owner::Procedure => "I'm flying the route",
            Owner::Autotakeoff => "I'm taking off",
            Owner::HeliRoute => "I'm flying the helicopter route",
            Owner::Hover => "I'm holding the hover",
            Owner::Recover => "I'm recovering the aircraft",
            Owner::Autotaxi => "I'm taxiing",
            Owner::Pushback => "We're pushing back",
            Owner::FullAuto => "I'm flying",
        }
    }

    /// It FLIES the aircraft, as opposed to driving it on the ground (the
    /// taxi and the tug share the cockpit with a pilot's after-landing
    /// clean-up and leave the monitor's advice to the pilot).
    pub(crate) fn flies(self) -> bool {
        !matches!(self, Owner::Autotaxi | Owner::Pushback)
    }

    /// The critical phases a loop flies: the takeoff, the landing, an upset
    /// recovery. The copilot's workload (`tempo`) is high under them.
    pub(crate) fn critical(self) -> bool {
        matches!(self, Owner::Recover | Owner::Autoland | Owner::Autotakeoff)
    }

    /// It takes every lever: a landing or an upset recovery
    /// ([`crew_landing_or_recovering`]).
    pub(crate) fn takes_every_lever(self) -> bool {
        matches!(self, Owner::Recover | Owner::Autoland)
    }
}

/// The innermost loop of ours holding the aircraft, in the air or on the
/// ground. `None` = the pilot's own aircraft.
pub(crate) fn crew_has_aircraft() -> Option<Owner> {
    innermost(Owner::active, |_| true)
}

/// The innermost loop of ours FLYING the aircraft ([`Owner::flies`]): the
/// copilot is the pilot flying, so a checklist is its own cross-check, the
/// monitor's checks fix rather than advise, praise goes to nobody, and an
/// ILS setup or a coupling is the loop's.
pub(crate) fn crew_flying() -> Option<Owner> {
    innermost(Owner::active, Owner::flies)
}

/// The outermost loop of ours armed: what the flight is FOR.
pub(crate) fn crew_mission() -> Option<Owner> {
    Owner::OUTERMOST_FIRST.into_iter().find(|o| o.active())
}

/// A loop in a critical phase ([`Owner::critical`]) is flying, at any depth.
pub(crate) fn crew_in_critical_phase() -> bool {
    innermost(Owner::active, Owner::critical).is_some()
}

/// A landing or an upset recovery has the whole aircraft: the loops that
/// take every lever (the mixture manager, the speed hold and the lean seek
/// yield to them), and the moments the monitor's pilot-directed callouts
/// stay quiet ("lower the nose, add power" is backwards when the copilot is
/// the one doing it). Narrower than [`crew_flying`] on purpose: under a
/// circuit or the follower the monitor's callouts still speak, and its fixes
/// (the altimeter reset) ride the spoken edge. Four inline copies of this
/// pair, now one.
pub(crate) fn crew_landing_or_recovering() -> bool {
    crew_taking_every_lever().is_some()
}

/// WHICH loop takes every lever now, innermost first: a recovery or a
/// landing (`None` when neither). The monitor tells them apart: a landing
/// still owes the altimeter its QNH, a recovery owes nothing but itself.
pub(crate) fn crew_taking_every_lever() -> Option<Owner> {
    innermost(Owner::active, Owner::takes_every_lever)
}

/// The speed governor (`tools::speedhold`) holds the throttle, in flight or
/// taxiing. Not an [`Owner`]: it holds a lever, never the aircraft.
pub(crate) fn speed_held() -> bool {
    crate::tools::speedhold::ACTIVE.load(std::sync::atomic::Ordering::Relaxed)
}

/// The speed governor is TAXIING (a ground hold, not a flight hold).
pub(crate) fn taxi_speed_held() -> bool {
    crate::tools::speedhold::taxi_hold_active()
}

/// The speed governor lets go of the throttle where it is; whether it was
/// holding it.
pub(crate) fn release_speed_hold() -> bool {
    crate::tools::speedhold::release()
}

/// "My controls" from the autopilot tool: a recovery and a landing both let
/// go (both doors called, never short-circuited); whether either was flying.
pub(crate) fn release_recovery_and_landing() -> bool {
    crate::tools::recover::abort() | crate::autoland::abort()
}

/// What the crew holds when the pilot asks for the controls back: one read
/// of every loop and lever a release can hand over, taken BEFORE anything is
/// let go, so the branches of [`Held::release_to_pilot`] judge the same
/// moment. The command loop's release guard is its one reader.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Held {
    pub(crate) speed: bool,
    /// The continuous mixture manager OR a lean seek still hunting the peak.
    pub(crate) mixture: bool,
    pub(crate) reverse: bool,
    pub(crate) taxi: bool,
    pub(crate) takeoff: bool,
    pub(crate) circuit: bool,
    pub(crate) procedure: bool,
    pub(crate) full_auto: bool,
    /// A taxi-in between its drives (the request's wait) holds the aircraft
    /// with no auto taxi running.
    pub(crate) taxi_in: bool,
    pub(crate) hover: bool,
    /// A helicopter's route or circuit: in cruise the hover loop is not
    /// running, and the route's speed law owns the collective until it is
    /// told to let go.
    pub(crate) heli: bool,
}

impl Held {
    /// Every loop and lever read now.
    pub(crate) fn now() -> Held {
        use std::sync::atomic::Ordering::Relaxed;
        Held {
            speed: speed_held(),
            mixture: crate::tools::engine::MIX_MANAGED.load(Relaxed)
                || crate::tools::engine::LEAN_SEEKING.load(Relaxed),
            reverse: crate::tools::reverse::ACTIVE.load(Relaxed),
            taxi: Owner::Autotaxi.active(),
            takeoff: Owner::Autotakeoff.active(),
            circuit: Owner::Circuit.active(),
            procedure: Owner::Procedure.active(),
            full_auto: Owner::FullAuto.active(),
            taxi_in: crate::taxi_in::running(),
            hover: Owner::Hover.active(),
            heli: Owner::HeliRoute.active() || Owner::HeliCircuit.active(),
        }
    }

    /// The crew holds something a release can hand back.
    pub(crate) fn any(&self) -> bool {
        self.hover
            || self.heli
            || self.speed
            || self.mixture
            || self.reverse
            || self.taxi
            || self.taxi_in
            || self.takeoff
            || self.circuit
            || self.procedure
            || self.full_auto
    }

    /// Hand back what `tl` (the pilot's words, lowercased) asks for: naming
    /// a lever releases just that one; a generic "my controls" / "let go"
    /// releases everything held. Each loop is released by its own door, in
    /// this order, and the spoken parts of the readback are returned (empty:
    /// nothing named was held).
    pub(crate) fn release_to_pilot(&self, tl: &str) -> Vec<&'static str> {
        let names_mix = tl.contains("mixture") || tl.contains("mix");
        let names_speed = tl.contains("throttle")
            || tl.contains("speed")
            || tl.contains("power")
            || tl.contains("taxi")
            || tl.contains("thr");
        // The steering half of an auto taxi, releasable on its
        // own: "my steering" keeps the speed hold, the
        // pre-existing "you steer, I have the throttle" split.
        let names_steer = tl.contains("steer")
            || tl.contains("pedal")
            || tl.contains("rudder")
            || tl.contains("nosewheel");
        let names_reverse = tl.contains("reverse");
        // "Release parking brake" names a lever NOBODY here holds:
        // it's for the brake tool, so it must not count as a
        // generic release: that dropped the speed hold instead
        // and left the throttle up with the brake still set
        // (SR22, 2026-07-11).
        let generic =
            !names_mix && !names_speed && !names_reverse && !names_steer && !tl.contains("brake");
        let mut released: Vec<&'static str> = Vec::new();
        // A helicopter's hover: the collective is ours and the
        // autopilot holds the position. "My controls" or naming
        // the collective hands both back where they are.
        if self.heli && (generic || tl.contains("collective")) {
            // The route lets go of the collective where it is and
            // leaves the autopilot in the mode it is flying.
            Owner::HeliRoute.stop();
            Owner::HeliCircuit.stop();
            released.push("route off");
        }
        if self.hover && (generic || tl.contains("collective")) {
            Owner::Hover.stop();
            released.push("hover off, your controls");
        }
        // Full auto owns the entire chain (engine, taxi, line-up,
        // circuit); cancelling it stands them all down and wins the
        // release, so the per-loop branches below are skipped.
        if self.full_auto && (names_speed || names_steer || generic) {
            Owner::FullAuto.stop();
            released.push("full auto off, your aircraft");
        }
        // A circuit in progress owns whichever sub-loop is flying
        // (takeoff or autoland); aborting it stands them ALL down,
        // so it wins the generic release and the per-loop branches
        // below are skipped (they'd double up on its sub-loops).
        if !self.full_auto && self.circuit && (names_speed || names_steer || generic) {
            Owner::Circuit.stop();
            released.push("circuit off, your aircraft");
        }
        // A published-procedure follower is airborne on the AP (it
        // aborts its autoland handoff too); a lever-shaped or generic
        // release hands it back.
        if !self.full_auto && !self.circuit && self.procedure && (names_speed || generic) {
            Owner::Procedure.stop();
            released.push("procedure off, your aircraft");
        }
        // A takeoff in progress: "my controls" is a HANDOVER, not
        // a reject: the loop stands down silently, steering
        // expires, and the power stays set for the pilot now
        // flying it ("abort takeoff" is the arresting path).
        if !self.circuit
            && !self.full_auto
            && self.takeoff
            && (names_speed || names_steer || generic)
        {
            Owner::Autotakeoff.stop();
            released.push("your takeoff, power is set");
        }
        // The taxi-in ends where it is, braked to a stop and
        // parked: a stopped drive alone was driven again to the
        // stand 7 s later (LEMG, 2026-09-28).
        let taxi_in_ended = self.taxi_in && (names_speed || generic);
        if taxi_in_ended {
            crate::taxi_in::abort(true);
            released.push("taxi in off, your aircraft");
        }
        if !self.circuit
            && !self.full_auto
            && !taxi_in_ended
            && self.taxi
            && (names_speed || names_steer || generic)
        {
            Owner::Autotaxi.stop();
            if names_steer && !names_speed && !generic {
                released.push("steering's yours, I still have the speed");
            } else {
                release_speed_hold();
                released.push("auto taxi off, your aircraft");
            }
        } else if self.speed && !taxi_in_ended && (names_speed || generic) {
            release_speed_hold();
            released.push("speed hold off, your throttle");
        }
        if self.mixture && (names_mix || generic) {
            crate::tools::engine::release_mixture();
            released.push("mixture's yours");
        }
        // Reverse rides the same levers as the speed hold: any
        // lever-shaped release stows it (the watcher glides the
        // levers back to idle on its way out).
        if self.reverse && (names_reverse || names_speed || generic) {
            crate::tools::reverse::stow();
            released.push("stowing reverse");
        }
        released
    }
}

/// The innermost owner `active` says is set among those `keep` admits. Pure
/// over `active`, so the orders are a plain test.
fn innermost(active: impl Fn(Owner) -> bool, keep: impl Fn(Owner) -> bool) -> Option<Owner> {
    Owner::INNERMOST_FIRST
        .into_iter()
        .find(|&o| keep(o) && active(o))
}

#[cfg(test)]
mod tests {
    /// The copilot never reaches into a flying loop's own flags or doors:
    /// "is it flying?" is [`super::Owner`]'s ([`super::crew_has_aircraft`]
    /// and the rest), "let it go" is [`super::Owner::stop`],
    /// [`super::Held::release_to_pilot`] or a named door on this page
    /// ([`super::release_speed_hold`]). Read from the source: no COPILOT
    /// module (the pillars audit's classification, 2026-10-07) names an
    /// AUTONOMY module's `ACTIVE`, `ABORT`, `abort`, `release`, `cancel` or
    /// `halt`. The knots still tied are listed by file and reference, and
    /// each must still be there: the list only shrinks.
    #[test]
    fn copilot_reaches_the_loops_only_through_flight_state() {
        const COPILOT: &[&str] = &[
            "answers",
            "audio",
            "cabin",
            "checklist",
            "flow",
            "conversation",
            "dialogue",
            "grammar",
            "router",
            "mic",
            "stt",
            "tts",
            "tts_onnx",
            "vad",
            "voice",
            "phrase",
            "persona",
            "tempo",
            "rapport",
            "readback",
            "instructor",
            "monitor",
            "cockpit_auto",
            "undo",
            "shortcuts",
            "interaction",
            "handover",
            "go_around",
            "radiofx",
            "crew_radio",
            "plan",
            "failures",
            "approach_guide",
            "taxi_guide",
            "takeoff_config",
            "crew_pace",
            "tools/anti_ice",
            "tools/approach",
            "tools/autoflight",
            "tools/autopilot",
            "tools/cabin_air",
            "tools/electrical",
            "tools/engine",
            "tools/flaps",
            "tools/fuel_pumps",
            "tools/gear",
            "tools/ground",
            "tools/instruments",
            "tools/lever",
            "tools/lights",
            "tools/loading",
            "tools/power",
            "tools/prop",
            "tools/radio",
            "tools/reminder",
            "tools/signs",
            "tools/cycle",
            "tools/jet_start",
            "tools/scripted_start",
            "tools/controls",
            "tools/reverse",
            "tools/go_around",
        ];
        const AUTONOMY: &[&str] = &[
            "ap_engage",
            "ap_lands",
            "approach_energy",
            "autoland",
            "autotakeoff",
            "autotaxi",
            "circuit",
            "climb_watch",
            "course_leg",
            "full_auto",
            "ground_steer",
            "heading_bug",
            "heli_circuit",
            "heli_route",
            "hover",
            "ils",
            "lnav",
            "procedure",
            "pushback",
            "recovery_plan",
            "reversal",
            "roll_steer",
            "safe_hold",
            "stable_approach",
            "taxi_in",
            "takeoff_thrust",
            "takeoff_trim",
            "tiller",
            "yoke",
            "preflight",
            "plan_audit",
            "control",
            "tools/autoland",
            "tools/circuit",
            "tools/full_auto",
            "tools/heli",
            "tools/procedure",
            "tools/recover",
            "tools/speedhold",
            "tools/takeoff",
            "tools/taxi",
            "tools/setup",
            "tools/setup_enroute",
        ];
        const DOORS: &[&str] = &["ACTIVE", "ABORT", "abort", "release", "cancel", "halt"];
        // The four copilot to autonomy knots (pillars audit), by file and the
        // reference each still makes.
        const KNOTS: &[(&str, &str)] = &[
            // monitor's runway-fit judgement for full auto (knot 3): untied
            // in stage A item 2.
            ("monitor.rs", "full_auto::ACTIVE"),
        ];
        let src = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut names: Vec<&str> = AUTONOMY
            .iter()
            .map(|m| m.rsplit('/').next().unwrap())
            .collect();
        names.sort_unstable();
        names.dedup();
        let is_ident = |c: char| c.is_ascii_alphanumeric() || c == '_';
        let mut found: Vec<(String, String, usize)> = Vec::new();
        let mut scanned = Vec::new();
        for module in COPILOT {
            let mut files = Vec::new();
            let file = src.join(format!("{module}.rs"));
            if file.is_file() {
                files.push(file);
            }
            files.extend(crate::test_support::rust_files(&src.join(module)));
            for f in files {
                if crate::test_support::is_test_file(&f) {
                    continue;
                }
                let rel = f.strip_prefix(&src).unwrap().to_string_lossy().to_string();
                let text = std::fs::read_to_string(&f).unwrap();
                let production = text.split("#[cfg(test)]\nmod ").next().unwrap();
                scanned.push(rel.clone());
                for (i, l) in production.lines().enumerate() {
                    let code = l.split("//").next().unwrap_or("");
                    for name in &names {
                        let pat = format!("{name}::");
                        let mut from = 0;
                        while let Some(at) = code[from..].find(&pat) {
                            let start = from + at;
                            from = start + pat.len();
                            if code[..start].chars().next_back().is_some_and(is_ident) {
                                continue;
                            }
                            let rest = &code[from..];
                            let ident: String = rest.chars().take_while(|c| is_ident(*c)).collect();
                            if DOORS.contains(&ident.as_str()) {
                                found.push((rel.clone(), format!("{name}::{ident}"), i + 1));
                            }
                        }
                    }
                }
            }
        }
        assert!(
            scanned.iter().any(|f| f == "monitor.rs") && scanned.len() > 50,
            "the scan reads the copilot: {scanned:?}"
        );
        let stray: Vec<String> = found
            .iter()
            .filter(|(f, r, _)| !KNOTS.contains(&(f.as_str(), r.as_str())))
            .map(|(f, r, l)| format!("{f}:{l}: {r}"))
            .collect();
        assert!(
            stray.is_empty(),
            "copilot names a loop's flag or door; go through flight_state: {stray:#?}"
        );
        for (f, r) in KNOTS {
            assert!(
                found.iter().any(|(ff, rr, _)| ff == f && rr == r),
                "{f} no longer names {r}: drop it from the knots"
            );
        }
    }

    /// Every loop run in the tree is in the registry [`super::stand_down`]
    /// cancels, read from the source like the plugin contract's: every
    /// `sim::run::Slot` declared under `src` is in [`super::LOOPS`] by its
    /// path, once, and the stand-down sweeps the list. A loop flag NOT on a
    /// run (an `ABORT`, a `GEN`, a `_HELD`) must be named below with the door
    /// stand_down calls for it.
    #[test]
    fn stand_down_cancels_every_run() {
        // Loop flags not (yet) on a run, each with the door that stops it,
        // and the generation counters that are no loop at all.
        const NOT_ON_A_RUN: &[(&str, &str, &str)] = &[
            // The mission's and the follower's own stop latches, read by
            // their loops beside their runs.
            ("full_auto", "ABORT", "crate::full_auto::abort()"),
            ("procedure", "ABORT", "crate::procedure::abort()"),
            // Flags beside an unflagged run (one run at a time, `begin`),
            // cleared by their doors and the run's cleanup.
            ("full_auto", "ACTIVE", "crate::full_auto::abort()"),
            ("procedure", "ACTIVE", "crate::procedure::abort()"),
            ("autotakeoff", "ACTIVE", "crate::autotakeoff::cancel()"),
            // The pilot's stop word for the steered push (a mailbox).
            ("pushback", "ABORT", "crate::pushback::clear()"),
            // The checklist runner's flag beside its unflagged run: its
            // thread clears it on the way out once the sweep cancels the run.
            ("flow", "ACTIVE", ""),
            // The copilot's speech turn and the barge-in cut per channel.
            ("tempo", "TURN_GEN", ""),
            ("tts", "CUT_GEN", ""),
        ];
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let this = crate::test_support::module_source("flight_state").to_string();
        let body = this.split("pub(crate) fn stand_down()").nth(1).unwrap();
        let body = body.split("\n}\n").next().unwrap();
        assert!(
            body.contains("for (_, run) in &LOOPS {\n        run.cancel();"),
            "stand_down sweeps the registry"
        );
        let names: Vec<&str> = super::LOOPS.iter().map(|(n, _)| *n).collect();
        let mut sorted = names.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), names.len(), "each run once: {names:?}");
        let mut slots = Vec::new();
        let mut stray = Vec::new();
        for f in crate::test_support::rust_files(&root) {
            if crate::test_support::is_test_file(&f) || f.ends_with("flight_state.rs") {
                continue;
            }
            let text = std::fs::read_to_string(&f).unwrap();
            let rel = f.strip_prefix(&root).unwrap().with_extension("");
            let module = rel
                .components()
                .map(|c| c.as_os_str().to_string_lossy().to_string())
                .filter(|c| c != "mod")
                .collect::<Vec<_>>()
                .join("::");
            let production = text.split("#[cfg(test)]\nmod ").next().unwrap();
            let flags: Vec<&str> = production
                .lines()
                .filter_map(|l| {
                    let at = l.find("sim::run::Slot::flagged(&")?;
                    let rest = &l[at + "sim::run::Slot::flagged(&".len()..];
                    Some(&rest[..rest.find(')')?])
                })
                .collect();
            for line in production.lines() {
                let Some((name, ty)) = static_decl(line) else {
                    continue;
                };
                if ty.starts_with("sim::run::Slot") {
                    slots.push(format!("{module}::{name}"));
                } else if is_loop_flag_name(name) && !flags.contains(&name) {
                    match NOT_ON_A_RUN.iter().find(|(m, n, _)| *m == module && *n == name) {
                        Some((_, _, door)) => assert!(
                            door.is_empty() || body.contains(door),
                            "stand_down must call {door} for {module}::{name}"
                        ),
                        None => stray.push(format!("{module}::{name}")),
                    }
                }
            }
        }
        slots.sort();
        assert_eq!(slots, sorted, "every run in the registry, and only runs");
        assert!(
            stray.is_empty(),
            "loop flags on no run and not named with their door: {stray:?}"
        );
    }

    /// A process-wide `static` declaration: its name and type, at any
    /// visibility.
    fn static_decl(line: &str) -> Option<(&str, &str)> {
        let l = line.trim_start();
        let l = l
            .strip_prefix("pub(crate) ")
            .or_else(|| l.strip_prefix("pub "))
            .unwrap_or(l);
        let decl = l.strip_prefix("static ")?;
        let (name, ty) = decl.split_once(':')?;
        let name = name.trim();
        let upper = !name.is_empty()
            && name
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_');
        upper.then_some((name, ty.trim_start()))
    }

    /// The names a loop flag goes by when it is not on a run.
    fn is_loop_flag_name(name: &str) -> bool {
        matches!(name, "ACTIVE" | "ABORT" | "RUN" | "GENERATION")
            || name.ends_with("_HELD")
            || name.ends_with("GEN")
    }

    #[test]
    fn loop_flags_beyond_active_are_caught() {
        let decl = |l| static_decl(l).map(|(n, _)| n);
        assert_eq!(
            decl("pub static ACTIVE: AtomicBool = AtomicBool::new(false);"),
            Some("ACTIVE")
        );
        assert!(is_loop_flag_name("BRAKES_HELD"));
        assert!(is_loop_flag_name("CLIMB_WATCHER_GEN"));
        assert_eq!(
            static_decl("    pub(crate) static RUN: sim::run::Slot = sim::run::Slot::new();"),
            Some(("RUN", "sim::run::Slot = sim::run::Slot::new();"))
        );
        assert!(!is_loop_flag_name("LEGEND"));
        assert_eq!(decl("let ACTIVE: bool = true;"), None);
    }

    /// The rotor facts follow the profile: the S-76 has them, an aeroplane
    /// clears them, so a swap away from the helicopter no longer hover taxis
    /// in, reads "on the ground" to the tower aloft, or skips the vacate.
    #[test]
    fn rotor_facts_are_republished_from_every_profile() {
        let heli = crate::aircraft::load("sikorsky_s76c").expect("S-76C profile loads");
        assert!(crate::hover::rotor_facts(&heli).is_some());
        let plane = crate::aircraft::load("cessna_172sp").expect("172 profile loads");
        assert!(crate::hover::rotor_facts(&plane).is_none());
        // The snapshot publish_airframe installs carries them, and none off
        // a helicopter.
        assert!(super::airframe_facts(&heli).rotor.is_some());
        assert!(
            super::airframe_facts(&plane).rotor.is_none(),
            "publish_airframe must republish the rotor, None off a helicopter"
        );
        let this = crate::test_support::module_source("flight_state");
        let body = this
            .split("pub(crate) fn publish_airframe(")
            .nth(1)
            .unwrap();
        let body = body.split("\n}\n").next().unwrap();
        assert!(
            body.contains("crate::aircraft::install_airframe_facts(airframe_facts(p));"),
            "publish_airframe installs the whole snapshot"
        );
    }

    /// The text of `name`'s body in `src`, up to the next top-level item.
    fn body_of<'a>(src: &'a str, name: &str) -> &'a str {
        let at = src.find(name).unwrap_or_else(|| panic!("{name} not found"));
        let rest = &src[at..];
        let end = rest[1..].find("\nfn ").map_or(rest.len(), |e| e + 1);
        let end = end.min(rest[1..].find("\npub ").map_or(rest.len(), |e| e + 1));
        let end = end.min(
            rest[1..]
                .find("\npub(crate) ")
                .map_or(rest.len(), |e| e + 1),
        );
        &rest[..end]
    }

    /// A plan's transition, the last landing's runway and tracking, and the
    /// plan's destination outlived the flight: a "put us on a ten mile final"
    /// after LIEA-LEBL was judged against TA 6,000, and the altimeter check
    /// WRITES the setting with an automation flying. The forgetting is the
    /// doors' and the transition dies with the mission too.
    /// The crew's per-flight facts die at the door: a pilot takeover, a
    /// queued call and its readback record, the copilot's "say again", a
    /// summoned cart, the pilot's light flag and an open offer were each
    /// cleared only on their own path, if at all. The masks and the anti-ice
    /// claims are the switches' truth, not the flight's: never cleared here.
    #[test]
    fn forget_flight_clears_the_crews_flight_facts() {
        let _q = crate::crew_radio::QUEUE_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let _g = crate::tools::ground::GPU_TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        crate::flight::note_pilot_takeover("my controls");
        crate::crew_radio::transmit_readback("Cleared to land runway one three".into(), false);
        *crate::LAST_READBACK.lock().unwrap() =
            Some(("Gear down.".into(), std::time::Instant::now()));
        crate::tools::ground::note_cart_requested_for_test();
        crate::cockpit_auto::note_pilot_moved_lights();
        crate::rapport::set_offer_for_test();
        assert!(crate::flight::pilot_takeover().is_some());
        assert!(crate::crew_radio::crew_radio_pending());

        super::forget_flight();

        assert_eq!(crate::flight::pilot_takeover(), None);
        assert!(!crate::crew_radio::crew_radio_pending());
        assert_eq!(crate::crew_radio::last_readback(), None);
        assert!(crate::LAST_READBACK.lock().unwrap().is_none());
        assert!(!crate::tools::ground::cart_pending_for_test());
        assert!(!crate::cockpit_auto::take_pilot_moved_lights());
        assert_eq!(crate::rapport::offer(), None);
        let door = body_of(
            crate::test_support::module_source("flight_state"),
            "fn forget_flight()",
        );
        assert!(!door.contains("cabin_air"), "the masks are not the flight's");
        assert!(!door.contains("anti_ice"), "the anti-ice claims are not the flight's");
    }

    #[test]
    fn every_new_flight_and_mission_end_forgets_the_flight() {
        let door = body_of(
            crate::test_support::module_source("flight_state"),
            "fn forget_flight()",
        );
        for clear in [
            "publish_flight_transition(None)",
            "crate::autoland::forget_flight()",
            "crate::monitor::forget_destination()",
        ] {
            assert!(door.contains(clear), "forget_flight must call {clear}");
        }
        let mission_end = body_of(
            crate::test_support::module_source("full_auto"),
            "fn stop_active()",
        );
        assert!(mission_end.contains("publish_flight_transition(None)"));
    }

    /// The doors do every step, in order: the loops stop before the
    /// selectors change under them, and the flight's facts go after.
    #[test]
    fn the_doors_do_every_step_in_order() {
        let src = crate::test_support::module_source("flight_state");
        let flight = body_of(src, "pub(crate) fn flight_begins(");
        let full = flight
            .split("    stand_down();\n")
            .nth(1)
            .expect("a full door stands down");
        for step in [
            "forget_flight();",
            "crate::flow::stand_down(crate::flow::StandDown::NewFlight(",
            "crate::party_line::retire_for_new_flight();",
            "\"flight_begins\"",
        ] {
            assert!(full.contains(step), "flight_begins must do {step}");
        }
        let air = body_of(src, "pub(crate) fn airframe_begins(");
        let at = |s: &str| {
            air.find(s)
                .unwrap_or_else(|| panic!("airframe_begins: {s}"))
        };
        assert!(at("stand_down();") < at("publish_airframe(p);"));
        assert!(at("publish_airframe(p);") < at("forget_flight();"));
        assert!(air.contains("\"airframe_begins\""));
    }

    /// Every path that begins a flight goes through a door, and the steps a
    /// door does are called nowhere else: the five new-flight sites had
    /// drifted (the ramp never stood down, the foreign start never dropped
    /// the checklist).
    #[test]
    fn new_flight_steps_live_only_in_the_doors() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut stray = Vec::new();
        for f in crate::test_support::rust_files(&root) {
            if crate::test_support::is_test_file(&f) || f.ends_with("flight_state.rs") {
                continue;
            }
            let rel = f.strip_prefix(&root).unwrap().display().to_string();
            let text = std::fs::read_to_string(&f).unwrap();
            for (n, line) in text.lines().enumerate() {
                let code = line.trim_start();
                if code.starts_with("//") {
                    continue;
                }
                let mut steps = vec![
                    "retire_for_new_flight(",
                    "flight_state::forget_flight(",
                    "flight_state::publish_airframe(",
                ];
                // The list's own watch on the mailbox is the list's.
                if rel != "flow.rs" {
                    steps.push("StandDown::NewFlight(");
                }
                for step in steps {
                    if code.contains(step) {
                        stray.push(format!("{rel}:{} {step}", n + 1));
                    }
                }
            }
        }
        assert!(
            stray.is_empty(),
            "new-flight steps outside the doors: {stray:?}"
        );
        for (module, door) in [
            (
                "tools/setup",
                "flight_begins(crate::flight_state::NewFlight::Foreign)",
            ),
            (
                "tools/setup",
                "flight_begins(crate::flight_state::NewFlight::Ramp)",
            ),
            (
                "tools/setup",
                "flight_begins(crate::flight_state::NewFlight::Approach)",
            ),
            (
                "tools/setup_enroute",
                "flight_begins(crate::flight_state::NewFlight::Enroute)",
            ),
            (
                "monitor",
                "flight_begins(crate::flight_state::NewFlight::Reposition)",
            ),
            ("detect", "airframe_begins(&p, \"boot\")"),
            ("detect", "airframe_begins(&p, \"swap\")"),
        ] {
            assert!(
                crate::test_support::module_source(module).contains(door),
                "{module} must call {door}"
            );
        }
    }

    /// A jump our own setup (or the runner's start) made is that setup's
    /// flight: its door runs as the init returns. Only an unaccounted jump
    /// runs the reposition's door.
    #[test]
    fn a_reposition_after_our_own_door_is_the_same_flight() {
        use std::time::Duration;
        assert_eq!(super::reposition_skip(true, None), Some("loading"));
        assert_eq!(
            super::reposition_skip(false, Some(Duration::from_secs(5))),
            Some("begun")
        );
        assert_eq!(
            super::reposition_skip(false, Some(Duration::from_secs(600))),
            None
        );
        assert_eq!(super::reposition_skip(false, None), None);
        assert_eq!(super::NewFlight::Reposition.list_cause(), "reposition");
        assert_eq!(super::NewFlight::Approach.list_cause(), "setup");
    }

    /// A loop's flying flag has one writer: its run (`sim::run::Slot`,
    /// flagged). Ten modules stored `speedhold::ACTIVE` and seven
    /// `autotaxi::ACTIVE` (bypassing `halt`), and a late store from a
    /// superseded run flipped the new run's (refactor audit 2026-10-04,
    /// phase 3). So: no module writes a flag its slot owns, and no module
    /// writes another module's `ACTIVE` at all (the owner's door does).
    #[test]
    fn a_loop_flag_has_one_writer() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        // The production text of a file: up to its first inline test module.
        fn production(text: &str) -> &str {
            text.split("#[cfg(test)]\nmod ").next().unwrap()
        }
        let mut flagged = Vec::new();
        let mut files = Vec::new();
        for f in crate::test_support::rust_files(&root) {
            if crate::test_support::is_test_file(&f) {
                continue;
            }
            let text = std::fs::read_to_string(&f).unwrap();
            let rel = f.strip_prefix(&root).unwrap().display().to_string();
            for line in production(&text).lines() {
                if let Some(at) = line.find("sim::run::Slot::flagged(&") {
                    let rest = &line[at + "sim::run::Slot::flagged(&".len()..];
                    flagged.push((rel.clone(), rest[..rest.find(')').unwrap()].to_string()));
                }
            }
            files.push((rel, text));
        }
        assert!(flagged.len() >= 12, "the flagged runs: {flagged:?}");
        let mut writes = Vec::new();
        for (rel, text) in &files {
            for (n, line) in production(text).lines().enumerate() {
                let code = line.trim_start();
                if code.starts_with("//") {
                    continue;
                }
                for op in [".store(", ".swap(", ".compare_exchange(", ".fetch_"] {
                    // Another module's flag, by path.
                    if code.contains(&format!("::ACTIVE{op}")) {
                        writes.push(format!("{rel}:{} {code}", n + 1));
                    }
                    // A flag this module's own slot owns.
                    for (owner, flag) in &flagged {
                        let bare = format!("{flag}{op}");
                        if owner == rel
                            && code.contains(&bare)
                            && !code.contains(&format!("_{bare}"))
                        {
                            writes.push(format!("{rel}:{} {code}", n + 1));
                        }
                    }
                }
            }
        }
        assert!(writes.is_empty(), "loop flags written outside their run: {writes:#?}");
    }

    /// The circuit awaited the published-arrival lookup, then cleared
    /// autoland's ABORT and launched it: a stand-down landing inside that
    /// await armed an autoland no parent could cancel. Its own cancel is
    /// read again between the await and the launch, the launch takes the
    /// circuit's token as its parent, and nobody clears another loop's
    /// latch: autoland has none (its run is cancelled, never a latch the
    /// next launch inherits), and every launch names its parents.
    #[test]
    fn the_circuit_rechecks_its_abort_before_launching_autoland() {
        let src = crate::test_support::module_source("circuit");
        let at = src.find("if try_published_arrival(").unwrap();
        let rest = &src[at..];
        let launch = rest.find("crate::autoland::run_autoland_scoped(").unwrap();
        let recheck = rest.find("if xp.cancelled()").unwrap();
        assert!(recheck < launch, "the circuit's cancel re-read before the launch");
        let call = &rest[launch..launch + rest[launch..].find(");").unwrap()];
        assert!(call.contains("xp.cancellation()"), "scoped to the circuit");
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        for f in crate::test_support::rust_files(&root) {
            if f.ends_with("flight_state.rs") {
                continue;
            }
            let text = std::fs::read_to_string(&f).unwrap();
            for banned in ["autoland::ABORT", "autoland::run_autoland("] {
                assert!(!text.contains(banned), "{}: {banned}", f.display());
            }
        }
        let autoland = crate::test_support::module_source("autoland");
        assert!(!autoland.contains("static ABORT"), "autoland has no latch");
        assert!(!autoland.contains("pub fn run_autoland("), "no unscoped launch");
    }

    /// A power-up or jet start on its own thread kept firing switches through
    /// a swap or instructor entry: it had no cancel at all. Each now owns a
    /// run taken on the caller's thread, and stand_down cancels it.
    #[test]
    fn a_stand_down_stops_a_running_start_sequence() {
        let src = crate::test_support::module_source("tools/jet_start");
        let steps = body_of(src, "async fn run_sequence(");
        assert!(
            steps.matches("if stood_down(xp)").count() >= 5,
            "checked between steps"
        );
        let power = crate::test_support::module_source("tools/power");
        let spawn = body_of(power, "fn spawn(");
        assert!(
            spawn.find("POWER_UP.take_over()").unwrap() < spawn.find("std::thread::spawn").unwrap(),
            "the run is taken on the caller's thread"
        );
        let names: Vec<&str> = super::LOOPS.iter().map(|(n, _)| *n).collect();
        assert!(names.contains(&"tools::power::POWER_UP"));
        assert!(names.contains(&"tools::jet_start::JET_START"));
        let prep = body_of(
            crate::test_support::module_source("tools/power"),
            "pub(crate) async fn run_prep(",
        );
        assert!(
            prep.matches("if stood_down()").count() >= 4,
            "checked between steps"
        );
    }

    /// A procedure's stashed missed approach is its own landing's: a new
    /// procedure and an abort drop it.
    #[test]
    fn a_missed_approach_stash_dies_with_its_procedure() {
        let src = crate::test_support::module_source("procedure");
        assert!(body_of(src, "pub fn abort() -> bool").contains("take_missed_legs();"));
        assert!(body_of(src, "pub fn start_scoped(").contains("take_missed_legs();"));
    }

    use super::{innermost, Owner};

    fn set(on: &'static [Owner]) -> impl Fn(Owner) -> bool {
        move |o| on.contains(&o)
    }

    /// Both orders hold every owner exactly once.
    #[test]
    fn the_owner_orders_name_every_loop_once() {
        for order in [Owner::INNERMOST_FIRST, Owner::OUTERMOST_FIRST] {
            for o in Owner::INNERMOST_FIRST {
                assert_eq!(order.iter().filter(|x| **x == o).count(), 1, "{o:?}");
            }
        }
    }

    /// The innermost loop answers "who has it": autoland inside full auto,
    /// a recovery over the circuit it took from, the taxi inside full auto.
    #[test]
    fn the_innermost_owner_has_the_aircraft() {
        let any = |_| true;
        assert_eq!(
            innermost(set(&[Owner::FullAuto, Owner::Autoland]), any),
            Some(Owner::Autoland)
        );
        assert_eq!(
            innermost(set(&[Owner::Circuit, Owner::Recover]), any),
            Some(Owner::Recover)
        );
        assert_eq!(
            innermost(set(&[Owner::FullAuto, Owner::Autotaxi]), any),
            Some(Owner::Autotaxi)
        );
        assert_eq!(innermost(set(&[]), any), None);
    }

    /// Flying leaves the ground drives out: full auto taxiing is full auto
    /// flying, a stand-alone taxi is nobody flying.
    #[test]
    fn the_ground_drives_do_not_fly() {
        assert_eq!(
            innermost(set(&[Owner::FullAuto, Owner::Autotaxi]), Owner::flies),
            Some(Owner::FullAuto)
        );
        assert_eq!(innermost(set(&[Owner::Pushback]), Owner::flies), None);
        // The loops the four old "automation flying" copies forgot.
        for o in [
            Owner::Autotakeoff,
            Owner::HeliRoute,
            Owner::HeliCircuit,
            Owner::Hover,
            Owner::Recover,
        ] {
            assert_eq!(
                innermost(move |x| x == o, Owner::flies),
                Some(o),
                "{o:?} flies"
            );
        }
    }

    /// The narrower sets: critical is the takeoff, the landing and a
    /// recovery; every lever is the landing and a recovery only.
    #[test]
    fn the_narrow_sets_are_named() {
        let critical: Vec<Owner> = Owner::INNERMOST_FIRST
            .into_iter()
            .filter(|o| o.critical())
            .collect();
        assert_eq!(
            critical,
            [Owner::Recover, Owner::Autoland, Owner::Autotakeoff]
        );
        let levers: Vec<Owner> = Owner::INNERMOST_FIRST
            .into_iter()
            .filter(|o| o.takes_every_lever())
            .collect();
        assert_eq!(levers, [Owner::Recover, Owner::Autoland]);
        assert_eq!(
            innermost(
                set(&[Owner::FullAuto, Owner::Autotakeoff]),
                Owner::takes_every_lever
            ),
            None
        );
    }

    /// The names the blackbox, the flight model and the duty engine read
    /// are `monitor::flying_loop`'s old ones, plus the recovery.
    #[test]
    fn owner_names_are_the_recorded_ones() {
        let names: Vec<&str> = Owner::INNERMOST_FIRST
            .into_iter()
            .map(Owner::name)
            .collect();
        assert_eq!(
            names,
            [
                "recovery",
                "autoland",
                "takeoff",
                "pushback",
                "taxi",
                "hover",
                "follower",
                "circuit",
                "helicopter circuit",
                "helicopter route",
                "full auto"
            ]
        );
    }

    /// The on-ground dataref is named once, `datarefs::ON_GROUND_ANY`: it
    /// was redeclared in fifteen files (refactor audit 2026-10-04, #9).
    #[test]
    fn the_on_ground_dataref_is_named_once() {
        let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let mut files = Vec::new();
        fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for e in std::fs::read_dir(dir).unwrap().flatten() {
                let p = e.path();
                if p.is_dir() {
                    walk(&p, out);
                } else if p.extension().is_some_and(|x| x == "rs") {
                    out.push(p);
                }
            }
        }
        for dir in ["src", "crates/atc/src", "crates/sim/src"] {
            walk(&manifest.join(dir), &mut files);
        }
        let literal = concat!("\"sim/flightmodel/failures/", "onground_any\"");
        let named: Vec<_> = files
            .iter()
            .filter(|f| std::fs::read_to_string(f).unwrap().contains(literal))
            .map(|f| f.strip_prefix(manifest).unwrap().display().to_string())
            .collect();
        assert_eq!(named, ["crates/sim/src/datarefs.rs"]);
    }

    /// "Is an automation flying?" has one home: no file outside this one
    /// lists the loops' flags inline again. Three or more different owners'
    /// `ACTIVE.load` within a few lines is the pattern that grew nine copies
    /// with different members (refactor audit 2026-10-04, duplicates #1).
    /// A site that really needs each loop by name (the release gate lets go
    /// of each lever separately) says so with `owners-by-name:` in the window.
    #[test]
    fn no_inline_automation_flying_lists() {
        const MODULES: &[&str] = &[
            "full_auto::",
            "heli_route::",
            "circuit::",
            "heli_circuit::",
            "procedure::",
            "autoland::",
            "autotakeoff::",
            "autotaxi::",
            "pushback::",
            "hover::",
            "recover::",
        ];
        const WINDOW: usize = 8;
        const MARKER_REACH: usize = 40;
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut files = Vec::new();
        fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for e in std::fs::read_dir(dir).unwrap().flatten() {
                let p = e.path();
                if p.is_dir() {
                    walk(&p, out);
                } else if p.extension().is_some_and(|x| x == "rs") {
                    out.push(p);
                }
            }
        }
        walk(&root, &mut files);
        // Which owner, if any, a line loads the flag of. A module name
        // inside another (`circuit::` in `heli_circuit::`) matches the
        // longer name.
        fn owner_of(l: &str) -> Option<&'static str> {
            let at = l.find("::ACTIVE.load")?;
            let head = &l[..at + 2];
            MODULES
                .iter()
                .filter(|m| head.ends_with(*m))
                .max_by_key(|m| m.len())
                .copied()
        }
        let mut found = Vec::new();
        for f in files {
            if f.ends_with("flight_state.rs") {
                continue;
            }
            let text = std::fs::read_to_string(&f).unwrap();
            // A marker excuses the block under it.
            let mut excused = 0usize;
            let owners_of: Vec<Option<&str>> = text
                .lines()
                .map(|l| {
                    if l.contains("owners-by-name:") {
                        excused = MARKER_REACH;
                    }
                    excused = excused.saturating_sub(1);
                    if excused > 0 {
                        None
                    } else {
                        owner_of(l)
                    }
                })
                .collect();
            for start in 0..owners_of.len() {
                let window = &owners_of[start..(start + WINDOW).min(owners_of.len())];
                let mut owners: Vec<&str> = window.iter().flatten().copied().collect();
                owners.sort_unstable();
                owners.dedup();
                if owners.len() >= 3 {
                    found.push(format!(
                        "{}:{} {owners:?}",
                        f.strip_prefix(&root).unwrap().display(),
                        start + 1
                    ));
                    break;
                }
            }
        }
        assert!(
            found.is_empty(),
            "inline automation-flying lists, use flight_state: {found:?}"
        );
    }

    /// Every airframe's facts come whole from its own profile: what one
    /// airframe declares or the detector read from its `.acf` is never in
    /// the next one's snapshot (the S-76's rotor on a 172 was the bug,
    /// refactor audit 2026-10-04). Each field against the profile it was
    /// built from, over every profile shipped.
    #[test]
    fn airframe_facts_come_whole_from_each_profile() {
        let mut checked = 0;
        for entry in std::fs::read_dir("profiles").unwrap() {
            let path = entry.unwrap().path();
            if path.extension().and_then(|e| e.to_str()) != Some("yaml") {
                continue;
            }
            let id = path.file_stem().unwrap().to_str().unwrap().to_string();
            let Some(p) = crate::aircraft::load(&id) else { continue };
            let f = super::airframe_facts(&p);
            checked += 1;
            assert_eq!(f.profile.as_ref().map(|q| q.id.as_str()), Some(p.id.as_str()));
            assert!(!f.nose_castors && !f.nose_free, "{id}: the .acf is unread");
            assert_eq!(f.rotor.is_some(), p.heli_start.is_some(), "{id}: rotor");
            assert_eq!(f.mode_knob.is_some(), p.transponder_mode_knob.is_some(), "{id}");
            assert_eq!(f.takeoff_config.is_some(), p.takeoff_config.is_some(), "{id}");
            assert_eq!(f.cabin_air.is_some(), p.cabin_air.is_some(), "{id}");
            assert_eq!(f.altimeter.is_some(), p.altimeter.is_some(), "{id}");
            assert_eq!(f.gear_warning, p.gear_warning, "{id}");
            assert_eq!(f.throttle_dref, p.throttle_dataref, "{id}");
            assert_eq!(f.gear_limit_kts, p.vlo_kts, "{id}");
            assert_eq!(f.ap_altitude_capture, p.ap_altitude_capture, "{id}");
            assert_eq!(
                f.signs.is_some(),
                p.seat_belt_signs.is_some() || p.cabin_crew,
                "{id}: signs"
            );
            assert_eq!(
                f.speed_floor_kts,
                crate::aircraft::speed_floor_for(p.vs_clean_kts),
                "{id}"
            );
            assert_eq!(
                f.vs_limits,
                Some(crate::procedure::vs_limits_for(p.reference.as_ref())),
                "{id}"
            );
            assert_eq!(
                f.exterior.landing.is_some(),
                p.lights.iter().any(|l| l.key.eq_ignore_ascii_case("landing")),
                "{id}"
            );
        }
        assert!(checked >= 10, "only {checked} profiles");
    }

    /// The live snapshot has ONE whole writer, the door's
    /// `install_airframe_facts`, and the few copy-on-write updates are the
    /// detector's `.acf` facts; anything else is a test's setter. A new
    /// production writer of a per-airframe fact is a fact that can outlive
    /// its airframe again, so it fails here.
    #[test]
    fn the_airframe_snapshot_has_one_door() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        fn production(text: &str) -> &str {
            text.split("#[cfg(test)]\nmod ").next().unwrap()
        }
        // The .acf's facts, read by the detector once it has the file.
        const ACF_WRITERS: [&str; 3] = [
            "pub fn publish_nose_castors(",
            "pub fn publish_nose_free(",
            "pub fn publish_gear_warning(",
        ];
        let mut found = Vec::new();
        for f in crate::test_support::rust_files(&root) {
            if crate::test_support::is_test_file(&f) {
                continue;
            }
            let text = std::fs::read_to_string(&f).unwrap();
            let rel = f.strip_prefix(&root).unwrap().display().to_string();
            let lines: Vec<&str> = production(&text).lines().collect();
            for (n, line) in lines.iter().enumerate() {
                let code = line.trim_start();
                if code.starts_with("//") {
                    continue;
                }
                let install = code.contains("install_airframe_facts(")
                    && !code.starts_with("pub fn install_airframe_facts(");
                let update = code.contains("update_airframe_facts(")
                    && !code.starts_with("pub fn update_airframe_facts(");
                if install && !(rel == "flight_state.rs" && code.contains("airframe_facts(p)")) {
                    found.push(format!("{rel}:{} {code}", n + 1));
                }
                if update {
                    // The enclosing fn and whether it is test-only.
                    let at = (0..n).rev().find(|i| {
                        let t = lines[*i].trim_start();
                        t.starts_with("pub fn ") || t.starts_with("fn ")
                            || t.starts_with("pub(crate) fn ")
                    });
                    let Some(at) = at else {
                        found.push(format!("{rel}:{} {code}", n + 1));
                        continue;
                    };
                    let header = lines[at].trim_start();
                    let test_only = at > 0 && lines[at - 1].trim() == "#[cfg(test)]";
                    if !test_only && !ACF_WRITERS.iter().any(|w| header.starts_with(w)) {
                        found.push(format!("{rel}:{} in `{header}`", n + 1));
                    }
                }
                // The .acf writers are the detector's alone.
                for w in ACF_WRITERS {
                    let name = &w["pub fn ".len()..];
                    if code.contains(name) && !code.starts_with(w) && rel != "detect.rs" {
                        found.push(format!("{rel}:{} {code}", n + 1));
                    }
                }
            }
        }
        assert!(found.is_empty(), "airframe facts written outside the door: {found:#?}");
    }
}
