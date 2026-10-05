//! The copilot's own hands on the cockpit: the things a crew member sets
//! without being asked, when the aircraft wakes up or the sun goes down.
//!
//! Two duties so far, both per-airframe and both driven from the monitor tick:
//! the interior lights a crew member reaches up and switches on once it is dark
//! ([`crate::aircraft::LightProfile::auto_at_night`]), and the GNS430's
//! welcome page, which sits there until somebody presses ENT twice
//! ([`crate::aircraft::AircraftProfile::gps_ack_command`]).
//!
//! Everything here is a pure state machine over (powered, sun angle) so the
//! rules battle-test as plain `cargo test`; the monitor owns the reads, the
//! writes and the voice. Events blackbox as `"kind":"cockpit_auto"`.
//!
//! The one rule that shapes the rest: **the copilot acts, then stops arguing.**
//! Every action is one-shot per spell, and "off" is only ever commanded for a
//! light the copilot itself switched on. A pilot who turns the roof floods back
//! off has made a decision, and a watch that re-asserts every tick is not an
//! automation, it is a fight over a switch. Same reason the gyro auto-sync
//! carries a cooldown rather than a per-tick correction.

/// Sun elevation at or below which the copilot calls it night, degrees.
/// Zero is the sun on the horizon: cockpit lighting is a sunset job, not a
/// civil-twilight one, and a crew reaching for the floods as the sun goes down
/// is early by nobody's reckoning.
const NIGHT_SUN_PITCH_DEG: f64 = 0.0;
/// ...and back to day only once the sun is properly up. The gap between the two
/// is hysteresis, not indecision: the sun crosses the horizon slowly and a
/// single threshold would flip the lights back and forth through every ripple
/// in the read (and on the ground at dawn, through every attitude change, since
/// this is the SUN's pitch in the world, not ours).
const DAY_SUN_PITCH_DEG: f64 = 5.0;

/// Dusk or dark for the exterior-light stages (taxi on the landing lamp,
/// vacate): the sun below the day threshold, so twilight counts as dark.
pub fn dusk_or_dark(sun_pitch_deg: f64) -> bool {
    sun_pitch_deg < DAY_SUN_PITCH_DEG
}
/// Main-bus volts above which the aircraft counts as powered — battery,
/// alternator or a ground cart, whichever is feeding it. Well under a healthy
/// 28 V bus and well over the noise on a dead one: full auto has read a
/// genuinely flat battery at 21 V, so this is not a health test, only a
/// "something is on" test.
const POWERED_VOLTS: f64 = 15.0;

/// What the copilot should do to the cockpit this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Switch on every light the profile flags `auto_at_night`.
    LightsOn,
    /// Switch those same lights back off — only ever after a [`Action::LightsOn`].
    LightsOff,
    /// Press the GNS430's ENT twice to clear its welcome page.
    AckGps,
}

/// Tracks the cockpit's day/night and powered/dead spells, and fires each duty
/// once per spell.
#[derive(Debug, Default)]
pub struct CockpitAuto {
    /// Latched night state, so the hysteresis band holds rather than flips.
    night: bool,
    powered: bool,
    /// The copilot switched the night lights on and has not switched them off.
    /// Gates the "off" action: a light the pilot turned on is the pilot's.
    lights_ours: bool,
    /// Already switched on during THIS powered-night spell. Separate from
    /// `lights_ours` because the pilot may switch them off afterwards, and that
    /// must not re-arm us to switch them straight back on.
    lights_done: bool,
    /// The GNS430 welcome page has been acknowledged for this power-up.
    gps_done: bool,
}

impl CockpitAuto {
    /// Advance the state machine. `sun_pitch_deg` is the sun's elevation
    /// (`sim/graphics/scenery/sun_pitch_degrees`) and `bus_volts` the main bus.
    ///
    /// Returns the actions owed, in the order they should be performed.
    pub fn tick(&mut self, sun_pitch_deg: f64, bus_volts: f64) -> Vec<Action> {
        let powered = bus_volts > POWERED_VOLTS;
        // Hysteresis: only the outer thresholds move the latch, so the band
        // between them holds whatever we last decided.
        if sun_pitch_deg <= NIGHT_SUN_PITCH_DEG {
            self.night = true;
        } else if sun_pitch_deg >= DAY_SUN_PITCH_DEG {
            self.night = false;
        }

        let mut actions = Vec::new();

        // Power lost: forget everything owed to this spell. A cold aircraft has
        // dark lights and a dark GPS whatever we did before, so the next
        // power-up is a fresh cockpit and earns the full sequence again.
        if !powered {
            self.powered = false;
            self.lights_ours = false;
            self.lights_done = false;
            self.gps_done = false;
            return actions;
        }
        self.powered = true;

        // The welcome page appears when the unit powers up, so this is a
        // power-up duty and not a night one.
        if !self.gps_done {
            self.gps_done = true;
            actions.push(Action::AckGps);
        }

        // Lights on covers all three triggers the same way, because they are
        // one condition and not three: the sun going down on a powered
        // aircraft, the battery coming on at night, and the engines starting at
        // night are all "it is dark and the aircraft is alive" becoming true.
        if self.night {
            if !self.lights_done {
                self.lights_done = true;
                self.lights_ours = true;
                actions.push(Action::LightsOn);
            }
        } else {
            // Daybreak. Put back only what we put on.
            if self.lights_ours {
                self.lights_ours = false;
                actions.push(Action::LightsOff);
            }
            self.lights_done = false;
        }

        actions
    }

    /// The pilot moved these lights themselves: drop our claim on them, so
    /// daybreak leaves them exactly where the pilot left them.
    pub fn pilot_took_the_lights(&mut self) {
        self.lights_ours = false;
    }
}

/// Set when the `set_lights` tool switches a light this module also drives —
/// i.e. the PILOT (by voice or panel button) reached for a switch the copilot
/// had claimed. Consumed by the monitor's watch on its next pass.
///
/// A process-wide flag, so the rule CLAUDE.md keeps re-learning applies: it
/// must not outlive its airframe. `swap(false)` on read makes it a one-shot,
/// and the monitor rebuilds the whole state machine on a profile change, so a
/// flag set by the last aircraft cannot decide anything about the next one.
static PILOT_MOVED_LIGHTS: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Called by the light tool when a pilot-driven switch touches a light flagged
/// [`crate::aircraft::LightProfile::auto_at_night`]. The copilot's own writes
/// go through [`crate::tools::lights::apply`] directly and never come here,
/// which is the point: this means the PILOT moved it.
pub fn note_pilot_moved_lights() {
    PILOT_MOVED_LIGHTS.store(true, std::sync::atomic::Ordering::Relaxed);
}

/// The ended flight's flag, forgotten at the new-flight door
/// (`flight_state::forget_flight`).
pub fn forget_pilot_moved_lights() {
    PILOT_MOVED_LIGHTS.store(false, std::sync::atomic::Ordering::Relaxed);
}

/// Take the flag, clearing it.
pub fn take_pilot_moved_lights() -> bool {
    PILOT_MOVED_LIGHTS.swap(false, std::sync::atomic::Ordering::Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Day into night on a powered aircraft: the lights come on once, and once
    /// only. The "once only" half is the whole design — a watch that re-fires
    /// every tick is a fight with the pilot over a switch.
    #[test]
    fn night_falls_on_a_powered_aircraft_and_the_lights_come_on_once() {
        let mut a = CockpitAuto::default();
        // Powered in daylight: the GPS gets its ENT, the lights stay off.
        assert_eq!(a.tick(30.0, 28.0), vec![Action::AckGps]);
        assert_eq!(a.tick(30.0, 28.0), vec![]);
        // Dusk, still above the night threshold: nothing yet.
        assert_eq!(a.tick(2.0, 28.0), vec![]);
        // Sun on the horizon.
        assert_eq!(a.tick(0.0, 28.0), vec![Action::LightsOn]);
        // Deeper into the night, every tick: silence.
        for p in [-1.0, -6.0, -20.0, -30.0] {
            assert_eq!(a.tick(p, 28.0), vec![], "re-asserted at {p}");
        }
    }

    /// The pilot switches them back off. That is a decision, not a fault, and
    /// the copilot must not switch them on again.
    #[test]
    fn the_pilot_switching_them_off_is_respected() {
        let mut a = CockpitAuto::default();
        a.tick(30.0, 28.0);
        assert_eq!(a.tick(-10.0, 28.0), vec![Action::LightsOn]);
        a.pilot_took_the_lights();
        for _ in 0..50 {
            assert_eq!(a.tick(-10.0, 28.0), vec![]);
        }
        // And daybreak does not command "off" for lights that are no longer
        // ours — the pilot may have switched them on again for their own
        // reasons, and reaching over to undo that is exactly the argument this
        // whole module refuses to have.
        assert_eq!(a.tick(30.0, 28.0), vec![]);
    }

    /// Powering up at night is the same condition arriving from the other side,
    /// which is what "when we switch electrics on, or switch engines on" means:
    /// the bus coming live after dark.
    #[test]
    fn coming_alive_at_night_lights_the_cockpit() {
        let mut a = CockpitAuto::default();
        // Cold and dark at night: nothing, and nothing to acknowledge either —
        // a GPS with no power has no welcome page to clear.
        assert_eq!(a.tick(-20.0, 0.0), vec![]);
        assert_eq!(a.tick(-20.0, 0.4), vec![]);
        // Battery on. Both duties fall due at once, GPS first.
        assert_eq!(
            a.tick(-20.0, 24.0),
            vec![Action::AckGps, Action::LightsOn],
            "the bus coming live after dark owes both"
        );
        assert_eq!(a.tick(-20.0, 28.0), vec![]);
    }

    /// A battery master cycled off and on again is a fresh cockpit: the GNS430
    /// shows its welcome page again, so the ENT is owed again.
    #[test]
    fn a_power_cycle_re_arms_both_duties() {
        let mut a = CockpitAuto::default();
        assert_eq!(a.tick(-20.0, 28.0), vec![Action::AckGps, Action::LightsOn]);
        assert_eq!(a.tick(-20.0, 0.0), vec![], "power off does nothing itself");
        assert_eq!(a.tick(-20.0, 28.0), vec![Action::AckGps, Action::LightsOn]);
    }

    /// Daybreak puts back what the copilot put on, once.
    #[test]
    fn daybreak_switches_off_what_the_copilot_switched_on() {
        let mut a = CockpitAuto::default();
        a.tick(-20.0, 28.0);
        // Dawn, still inside the hysteresis band: the latch holds night.
        assert_eq!(a.tick(1.0, 28.0), vec![], "inside the band, hold");
        assert_eq!(a.tick(4.9, 28.0), vec![], "still inside the band");
        assert_eq!(a.tick(5.0, 28.0), vec![Action::LightsOff]);
        assert_eq!(a.tick(30.0, 28.0), vec![]);
        // And the next night earns them again.
        assert_eq!(a.tick(-1.0, 28.0), vec![Action::LightsOn]);
    }

    /// The hysteresis band exists so a sun sitting near the horizon cannot
    /// chatter the switches. Sweep across it repeatedly and count the actions.
    #[test]
    fn a_sun_on_the_horizon_cannot_chatter_the_lights() {
        let mut a = CockpitAuto::default();
        a.tick(30.0, 28.0);
        let mut switches = 0;
        // Sunset wobbling either side of zero, as a real read does.
        for step in 0..40 {
            let jitter = if step % 2 == 0 { 0.4 } else { -0.4 };
            let actions = a.tick(0.2 + jitter, 28.0);
            switches += actions
                .iter()
                .filter(|x| matches!(x, Action::LightsOn | Action::LightsOff))
                .count();
        }
        assert_eq!(switches, 1, "the sun crossing the horizon is one action");
    }

    /// A dead bus is not a night duty: nothing is switched on an aircraft with
    /// no power, or the copilot narrates lights the pilot cannot see.
    #[test]
    fn a_dead_bus_earns_nothing() {
        let mut a = CockpitAuto::default();
        for v in [0.0, 1.0, 12.0, 14.9] {
            assert_eq!(a.tick(-20.0, v), vec![], "acted on a dead bus at {v} V");
        }
        // A flat-but-live battery still counts: full auto has met 21 V, and a
        // pilot in that cockpit wants the lights as much as anyone.
        assert_eq!(a.tick(-20.0, 21.0), vec![Action::AckGps, Action::LightsOn]);
    }
}
