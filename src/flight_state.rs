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
//! Three doors:
//! - [`publish_airframe`]: every selector the tools read per airframe, from
//!   the profile, including to `None`. Called at boot and on every swap.
//! - [`stand_down`]: every loop that can hold the aircraft, stopped. Called
//!   on a swap (a loop cannot fly a different aircraft), on entering
//!   instructor mode (hands off), and when the sim connection is lost.
//!   `stand_down_pins_every_active_loop` keeps this list equal to every
//!   loop flag in the tree (`ACTIVE`, `ABORT`, `RUN`, a `_HELD` hold, a
//!   watcher's `GEN`), so a new loop cannot be forgotten here.
//! - [`forget_flight`]: the crew's facts about the flight that ended, at the
//!   setups' new flight and a new flight someone else started.
//!
//! Inventory of what is NOT yet cleared here and lives in its own module,
//! per FLIGHT rather than per airframe (each reset on its own path today):
//! `autotaxi::LAST_ARRIVED`, `autoland::VACATE_FOLLOWS`,
//! `full_auto::INBOUND_MADE` / `STAGE` / `MISSION`, `procedure::TOD` /
//! `TERRAIN_FLOOR` / `PILOT_ALT` / `LEG`, `tools::ground::CART_REQUESTED`,
//! `taxi_in::ARREST_HOLD`, `cockpit_auto::PILOT_MOVED_LIGHTS`,
//! `instructor::LESSON`, `flow::*`. `pushback::LEARNED_SENSE` is per SESSION
//! by design (the sim build's tug sense, learnt once), never per flight.

use crate::aircraft::AircraftProfile;

/// Publish the per-airframe selectors the tools read. Every one is set from
/// the profile, including to `None`: a process-wide fact must never outlive
/// its airframe, and a selector that is only set when present is the bug.
pub(crate) fn publish_airframe(p: &AircraftProfile) {
    crate::aircraft::publish_live_profile(p);
    // The systems survey is the detector's, from the sim; a new airframe
    // starts knowing nothing about its avionics.
    crate::systems::publish(None);
    // The .acf's castoring nose is read at detection; a new airframe starts
    // from its profile alone.
    crate::aircraft::publish_nose_castors(false);
    crate::aircraft::publish_nose_free(false);
    // The rotor is the helicopter's: an aeroplane after the S-76 must not
    // hover taxi in, be "on the ground" to the tower aloft, or skip a vacate.
    crate::hover::publish_rotor(crate::hover::rotor_facts(p));
    crate::autotakeoff::DEPARTURE_MINIMUMS.store(false, std::sync::atomic::Ordering::SeqCst);
    crate::tools::cabin_air::publish(p.cabin_air.clone());
    crate::tools::radio::publish_mode_knob(p.transponder_mode_knob.clone());
    crate::takeoff_config::publish(p.takeoff_config.clone());
    crate::autotakeoff::publish_climb_out(crate::autotakeoff::ClimbOut::from_profile(p));
    crate::autotakeoff::publish_takeoff_power(p.takeoff_power.clone());
    crate::autotakeoff::publish_gear_warning(p.gear_warning);
    crate::tools::signs::publish(p.seat_belt_signs.clone(), p.cabin_crew);
    crate::circuit::publish_ap_altitude_capture(p.ap_altitude_capture);
    crate::tools::lights::publish_exterior(&p.lights);
    crate::aircraft::publish_throttle_dataref(p.throttle_dataref.clone());
    crate::aircraft::publish_config_limits(p.vlo_kts, p.flaps.as_ref());
    crate::aircraft::publish_bus_volts(p.bus_volts_nominal);
    crate::aircraft::publish_speed_floor(p.vs_clean_kts);
    crate::aircraft::publish_speed_ceiling(p.vne_kts);
    crate::aircraft::publish_approach_floor(
        p.reference.as_ref().and_then(|r| r.vs0_kts),
        p.vs_clean_kts,
    );
    crate::aircraft::publish_stall_curve(
        p.vs_clean_kts,
        p.reference.as_ref().and_then(|r| r.vs0_kts),
        p.flaps.as_ref(),
    );
    crate::aircraft::publish_approach_thrust_min(p.approach_thrust_min);
    crate::aircraft::publish_arrest_lead(p.arrest_lead);
    crate::aircraft::publish_flare_elevator(p.flare_elevator);
    crate::aircraft::publish_roll_rudder(p.roll_rudder);
    crate::aircraft::publish_landing_distance(
        p.reference.as_ref().and_then(|r| r.landing_distance_ft),
    );
    crate::procedure::publish_vs_limits(p.reference.as_ref());
    crate::aircraft::publish_fuel_unit(p.fuel.as_ref());
    // A plan's transition belongs to the flight it was armed for.
    crate::tools::instruments::publish_flight_transition(None);
    crate::tools::instruments::publish_altimeter(p.altimeter.clone());
    // The altimeter cross-check was about the last airframe's instruments.
    crate::tools::instruments::publish_altimeter_truth(None);
    // What the crew switched on was the last airframe's anti-ice.
    crate::tools::anti_ice::clear_claims();
}

/// The crew's facts about the flight that ended, forgotten when a new one
/// begins: the ramp and approach setups and a new flight someone else
/// started (`setup::foreign_new_flight`). The world's side is
/// `party_line::retire_for_new_flight`, called beside it.
pub(crate) fn forget_flight() {
    // The plan's transition was the old flight's: the monitor's altimeter
    // check WRITES the setting with an automation flying.
    crate::tools::instruments::publish_flight_transition(None);
    // The runway the last autoland rolled out on, its tracking, and any gate
    // or path handed down to an arm that never flew.
    crate::autoland::forget_flight();
    // The plan's destination is published again by the next plan read.
    crate::monitor::forget_destination();
}

/// Every loop that can hold the aircraft, stopped. Aborts go through each
/// loop's own door where it has one (the door frees the brakes, ends the
/// mission, hands back a flyable aircraft); the plain flags are cleared here.
pub(crate) fn stand_down() {
    use std::sync::atomic::Ordering::SeqCst;

    crate::full_auto::abort();
    crate::circuit::abort();
    crate::procedure::abort();
    crate::autoland::abort();
    crate::tools::recover::abort();
    // The pitch-authority check was about the flight that ended (#181).
    crate::climb_watch::disarm_authority();
    crate::approach_guide::cancel();
    crate::ils::stop_setup_watch();
    crate::autotakeoff::cancel();
    crate::flow::abort();
    crate::heli_circuit::abort();
    crate::heli_route::abort();
    crate::hover::abort();
    crate::autotaxi::ACTIVE.store(false, SeqCst);
    crate::autotaxi::BACKTRACKING.store(false, SeqCst);
    crate::tools::speedhold::ACTIVE.store(false, SeqCst);
    crate::tools::reverse::ACTIVE.store(false, SeqCst);
    crate::tools::engine::MIX_MANAGED.store(false, SeqCst);
    crate::tools::engine::LEAN_SEEKING.store(false, SeqCst);
    crate::pushback::ACTIVE.store(false, SeqCst);
    // The crew's brake hold renews the held command; dropped, it expires.
    crate::tools::controls::release_held_brakes();
    // The altitude capture watcher writes the dial, ALT hold and throttle.
    crate::tools::autopilot::stop_capture_watcher();
    // A power-up or jet start on its own thread stops at its next step.
    crate::tools::power::stop_sequences();
    // A new flight's setup owns the aircraft: the taxi-in only lets go.
    crate::taxi_in::abort(false);
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
    innermost(Owner::active, Owner::takes_every_lever).is_some()
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
    /// Every module in the tree that declares a loop flag is named in
    /// [`super::stand_down`], so a new loop cannot outlive a swap by being
    /// forgotten here. The list is read from the source, like the plugin
    /// contract's. A loop flag is any static named `ACTIVE`, `ABORT`, `RUN`,
    /// `GENERATION`, or ending `_HELD` / `GEN` (the brake hold, the capture
    /// watcher, the start sequences), or typed `sim::run::Slot`, at any
    /// visibility: matching only `static ACTIVE:` let `BRAKES_HELD`, the
    /// capture watcher and the jet start through.
    #[test]
    fn stand_down_pins_every_active_loop() {
        // Generation counters that are not loops holding the aircraft: the
        // copilot's speech turn, the barge-in cut per audio channel.
        const NOT_LOOPS: &[&str] = &["tempo", "tts"];
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let this = std::fs::read_to_string(root.join("flight_state.rs")).unwrap();
        let body = this.split("pub(crate) fn stand_down()").nth(1).unwrap();
        // The function alone: the owner list below it names every loop too.
        let body = body.split("\n}\n").next().unwrap();
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
        let mut missing = Vec::new();
        let mut checked = 0;
        for f in files {
            let text = std::fs::read_to_string(&f).unwrap();
            let declares = text.lines().any(is_loop_flag);
            if !declares {
                continue;
            }
            let rel = f.strip_prefix(&root).unwrap().with_extension("");
            let module = rel
                .components()
                .map(|c| c.as_os_str().to_string_lossy().to_string())
                .collect::<Vec<_>>()
                .join("::");
            if NOT_LOOPS.contains(&module.as_str()) {
                continue;
            }
            checked += 1;
            if !body.contains(&format!("crate::{module}::")) {
                missing.push(module);
            }
        }
        assert!(checked >= 10, "expected the loop modules, found {checked}");
        assert!(
            missing.is_empty(),
            "loops with a loop flag not in stand_down: {missing:?}"
        );
    }

    /// A process-wide `static` declaring a loop flag, at any visibility.
    fn is_loop_flag(line: &str) -> bool {
        let l = line.trim_start();
        let l = l
            .strip_prefix("pub(crate) ")
            .or_else(|| l.strip_prefix("pub "))
            .unwrap_or(l);
        let Some(decl) = l.strip_prefix("static ") else {
            return false;
        };
        let Some((name, ty)) = decl.split_once(':') else {
            return false;
        };
        let name = name.trim();
        if name.is_empty()
            || !name
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
        {
            return false;
        }
        matches!(name, "ACTIVE" | "ABORT" | "RUN" | "GENERATION")
            || name.ends_with("_HELD")
            || name.ends_with("GEN")
            || ty.trim_start().starts_with("sim::run::Slot")
    }

    #[test]
    fn loop_flags_beyond_active_are_caught() {
        assert!(is_loop_flag(
            "pub static ACTIVE: AtomicBool = AtomicBool::new(false);"
        ));
        assert!(is_loop_flag(
            "static BRAKES_HELD: AtomicBool = AtomicBool::new(false);"
        ));
        assert!(is_loop_flag(
            "static CLIMB_WATCHER_GEN: AtomicU64 = AtomicU64::new(0);"
        ));
        assert!(is_loop_flag(
            "    pub(crate) static RUN: sim::run::Slot = sim::run::Slot::new();"
        ));
        assert!(is_loop_flag(
            "static OWNER: sim::run::Slot = sim::run::Slot::new();"
        ));
        assert!(!is_loop_flag(
            "static LEGEND: AtomicBool = AtomicBool::new(false);"
        ));
        assert!(!is_loop_flag("let ACTIVE: bool = true;"));
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
        let this = include_str!("flight_state.rs");
        let body = this
            .split("pub(crate) fn publish_airframe(")
            .nth(1)
            .unwrap();
        let body = body.split("pub(crate) fn stand_down()").next().unwrap();
        assert!(
            body.contains("crate::hover::publish_rotor(crate::hover::rotor_facts(p))"),
            "publish_airframe must republish the rotor, None off a helicopter"
        );
    }

    /// The text of `name`'s body in `src`, up to the next top-level item.
    fn body_of<'a>(src: &'a str, name: &str) -> &'a str {
        let at = src.find(name).unwrap_or_else(|| panic!("{name} not found"));
        let rest = &src[at..];
        let end = rest[1..].find("\nfn ").map_or(rest.len(), |e| e + 1);
        let end = end.min(rest[1..].find("\npub ").map_or(rest.len(), |e| e + 1));
        &rest[..end]
    }

    /// A plan's transition, the last landing's runway and tracking, and the
    /// plan's destination outlived the flight: a "put us on a ten mile final"
    /// after LIEA-LEBL was judged against TA 6,000, and the altimeter check
    /// WRITES the setting with an automation flying. Every crew-side new
    /// flight forgets them, and the transition dies with the mission too.
    #[test]
    fn every_new_flight_and_mission_end_forgets_the_flight() {
        let door = body_of(
            include_str!("flight_state.rs"),
            "pub(crate) fn forget_flight()",
        );
        for clear in [
            "publish_flight_transition(None)",
            "crate::autoland::forget_flight()",
            "crate::monitor::forget_destination()",
        ] {
            assert!(door.contains(clear), "forget_flight must call {clear}");
        }
        let setup = include_str!("tools/setup.rs");
        let foreign = body_of(setup, "async fn foreign_new_flight(");
        assert!(foreign.contains("crate::flight_state::forget_flight();"));
        // The ramp and approach setups: once per new flight begun.
        let calls = setup
            .matches("crate::flight_state::forget_flight();")
            .count();
        assert!(
            calls >= 3,
            "ramp, approach and foreign new flights: {calls}"
        );
        let mission_end = body_of(include_str!("full_auto.rs"), "fn stop_active()");
        assert!(mission_end.contains("publish_flight_transition(None)"));
    }

    /// The circuit awaited the published-arrival lookup, then cleared
    /// autoland's ABORT and launched it: a stand-down landing inside that
    /// await armed an autoland no parent could cancel. Its own ABORT is
    /// read again between the await and the launch.
    #[test]
    fn the_circuit_rechecks_its_abort_before_launching_autoland() {
        let src = include_str!("circuit.rs");
        let at = src.find("if try_published_arrival(").unwrap();
        let rest = &src[at..];
        let launch = rest.find("crate::autoland::ABORT.store(false").unwrap();
        let recheck = rest.find("if ABORT.load(Ordering::SeqCst)").unwrap();
        assert!(
            recheck < launch,
            "ABORT re-read before autoland's latch is cleared"
        );
    }

    /// A power-up or jet start on its own thread kept firing switches through
    /// a swap or instructor entry: it had no cancel at all. The epoch it
    /// began in is stale once a stand-down has landed.
    #[test]
    fn a_stand_down_stops_a_running_start_sequence() {
        let epoch = crate::tools::power::sequence_epoch();
        crate::tools::power::stop_sequences();
        assert!(crate::tools::power::sequence_stood_down(epoch));
        let src = include_str!("tools/jet_start.rs");
        let steps = body_of(src, "async fn run_sequence(");
        assert!(
            steps.matches("if stood_down(epoch)").count() >= 5,
            "checked between steps"
        );
        let prep = body_of(
            include_str!("tools/power.rs"),
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
        let src = include_str!("procedure.rs");
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
}
