//! Undo: the snapshot stack of datarefs a tool changed, and the "undo that"
//! grammar. Split out of main.rs on 2026-09-16.

/// How to recover from one utterance's worth of actions. The command loop keeps a
/// stack of these so "belay that" can walk back command by command, each with its
/// own recovery (the whole point: remember what was done and how to undo it).
pub(crate) struct UndoSnapshot {
    /// What the pilot said, for the readback ("belay the throttle").
    pub(crate) label: String,
    /// (dataref, spoken label, previous value, is-integer) to write back.
    pub(crate) reverts: Vec<(String, String, f64, bool)>,
    /// The action switched the autopilot on (it was off before), so undo should
    /// switch it back off, not just reset the dial it was flying to.
    pub(crate) disengage_ap: bool,
    /// Tools to run to recover an action that isn't a simple value write — e.g.
    /// `start_engine` to undo an engine shutdown.
    pub(crate) inverse: Vec<(String, serde_json::Value)>,
}

/// Keep recovery for the last dozen utterances — enough to walk back a bad stretch,
/// bounded so it can't grow without limit over a long flight.
pub(crate) const UNDO_STACK_MAX: usize = 12;

/// Tools that can engage the autopilot servos. If one of these turns the AP on,
/// "belay that" disengages it again.
pub(crate) fn engages_autopilot(tool: &str) -> bool {
    matches!(
        tool,
        "set_heading" | "set_altitude" | "set_vertical_speed" | "set_climb" | "navigate"
    )
}

/// One reversible dataref a tool writes: (dataref, spoken label, is it an integer
/// dataref). The int flag matters on restore — transponder/radio datarefs are
/// integers and must be written with `set_int`, not `set_float`.
pub(crate) type Undoable = (&'static str, &'static str, bool);

pub(crate) const F: bool = false; // float dataref

pub(crate) const I: bool = true; // integer dataref

/// The reversible dataref(s) a tool writes, so the command loop can snapshot them
/// before the write and restore on "belay that". Tools that fire commands rather
/// than set values (engine start, pushback, gyro align) have no entry; engine
/// shutdown is undone by an inverse command instead (see the capture loop). Kept
/// here, next to the undo logic, rather than in each tool: undo is a loop-level
/// concern and this keeps it in one place.
pub(crate) fn undoable_drefs(tool: &str) -> &'static [Undoable] {
    match tool {
        "set_flaps" => &[("sim/cockpit2/controls/flap_ratio", "flaps", F)],
        "gear" => &[("sim/cockpit2/controls/gear_handle_down", "the gear", F)],
        "speedbrake" => &[(
            "sim/cockpit2/controls/speedbrake_ratio",
            "the speedbrakes",
            F,
        )],
        "set_throttle" => &[(
            "sim/cockpit2/engine/actuators/throttle_ratio_all",
            "throttle",
            F,
        )],
        "set_mixture" | "lean_mixture" => &[(
            "sim/cockpit2/engine/actuators/mixture_ratio_all",
            "mixture",
            F,
        )],
        "set_pitch_trim" => &[("sim/cockpit2/controls/elevator_trim", "pitch trim", F)],
        "set_barometer" | "set_altimeter_current" => &[(
            "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot",
            "the altimeter",
            F,
        )],
        "set_parking_brake" => &[(
            "sim/cockpit2/controls/parking_brake_ratio",
            "the parking brake",
            F,
        )],
        "set_heading" => &[(
            "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot",
            "the heading bug",
            F,
        )],
        "set_altitude" => &[(
            "sim/cockpit2/autopilot/altitude_dial_ft",
            "the altitude select",
            F,
        )],
        "set_vertical_speed" | "set_climb" => &[(
            "sim/cockpit/autopilot/vertical_velocity",
            "vertical speed",
            F,
        )],
        "set_transponder" => &[(
            "sim/cockpit2/radios/actuators/transponder_code",
            "the squawk",
            I,
        )],
        "set_transponder_mode" => &[(
            "sim/cockpit2/radios/actuators/transponder_mode",
            "the transponder mode",
            I,
        )],
        "set_com1" => &[(
            "sim/cockpit2/radios/actuators/com1_frequency_hz_833",
            "com 1",
            I,
        )],
        "set_com2" => &[(
            "sim/cockpit2/radios/actuators/com2_frequency_hz_833",
            "com 2",
            I,
        )],
        "set_nav1" => &[(
            "sim/cockpit2/radios/actuators/nav1_frequency_hz",
            "nav 1",
            I,
        )],
        "set_obs" => &[(
            "sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot",
            "the OBS",
            F,
        )],
        _ => &[],
    }
}

/// Spoken label for an inverse recovery tool run during undo.
pub(crate) fn recovery_label(tool: &str) -> String {
    match tool {
        "start_engine" => "engine restarted".to_string(),
        other => format!("{} run", other.replace('_', " ")),
    }
}

/// An emergency abort/undo of the last action. Deterministic and matched before any
/// gate or model, so it always works. Phrased a few natural ways a pilot would say
/// it under stress.
pub(crate) fn is_undo(text: &str) -> bool {
    let t = text.to_ascii_lowercase();
    t.contains("belay")
        || t.contains("undo")
        || t.contains("scratch that")
        || t.contains("as you were")
        || t.contains("disregard")
        || t.contains("revert")
        || t.contains("reverse that")
        || t.contains("take it back")
        || t.contains("cancel that")
        || t.contains("abort that")
        || t.contains("abort the last")
        || t.contains("abort")
        // "abort" is reliably misheard as "a boat" / "a board" — both meaningless in
        // a cockpit, and too important a word to lose to a transcription slip.
        || t.contains("a boat")
        || t.contains("a board")
}

/// Join words as "a", "a and b", or "a, b and c" for a natural spoken list.
pub(crate) fn join_and(items: &[String]) -> String {
    match items {
        [] => String::new(),
        [a] => a.clone(),
        [a, b] => format!("{a} and {b}"),
        [rest @ .., last] => format!("{}, and {last}", rest.join(", ")),
    }
}
