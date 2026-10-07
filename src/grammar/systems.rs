//! Switchable systems: lights, electrical, pumps, anti-ice, heat.

use super::*;

/// Switchable systems: floods, the whole aircraft off or up, centre pumps, anti-ice, every on/off switch.
pub(super) fn systems(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // flood lights (on/off or a percent). Substring match so "floodlights" glued
    // into one word by STT also resolves.
    if t.contains("flood") {
        if let Some(n) = num {
            return Some(call("set_flood_lights", json!({ "percent": n })));
        }
        if let Some(on) = onoff(t) {
            return Some(call("set_flood_lights", json!({ "on": on })));
        }
        return line.stop();
    }

    // The whole aircraft off, the pilot's last words at the stand ("switch
    // off the aircraft", LIPK 2026-09-25, a miss; "electrics off" a second
    // later worked). Before the power-up shapes: "power the aircraft down"
    // contains "power the aircraft". Exact shapes, so "switch off the
    // aircraft lights" stays the lights'.
    if [
        "switch off the aircraft",
        "turn off the aircraft",
        "switch the aircraft off",
        "turn the aircraft off",
        "power down the aircraft",
        "power the aircraft down",
        "power down",
    ]
    .contains(&t.trim_end_matches(['.', '!']))
    {
        return Some(call(
            "set_electrical",
            json!({ "system": "all", "on": false }),
        ));
    }

    // "power up" / "powering up" / "bring the electrical up" → bring everything up.
    // No on/off token, so it'd otherwise miss the block below and fall to the LLM.
    // The main loop remaps set_electrical:all to the step-by-step power_up tool on
    // airframes that have it (auto_prep); on a GA single it just switches all on.
    if t.contains("power up")
        || t.contains("powering up")
        || t.contains("power the aircraft")
        || (has(t, "bring") && (has(t, "electrical") || has(t, "electrics")))
    {
        return Some(call(
            "set_electrical",
            json!({ "system": "all", "on": true }),
        ));
    }

    // Bare electrical shapes with NO state word: whisper hears the trailing
    // "on" as "are" ("Electrical systems are", live 2026-07-22). The ask is
    // obviously ON, cold-dark that's the only thing it can mean, and if
    // they're already on the tool's readback says so, which beats a silent
    // miss. Exact shapes only, so longer sentences (and question forms,
    // which the question gate catches first anyway) never land here.
    if t == "electrical systems are"
        || t == "electrical systems"
        || t == "electrical system"
        || t == "electrics"
    {
        return Some(call(
            "set_electrical",
            json!({ "system": "all", "on": true }),
        ));
    }

    // The centre tank pumps ("centre pumps off", "turn off the center
    // pumps", "ctr pumps on"): before the fuel pump rule, which would take
    // "centre fuel pumps" to the engine boost pumps.
    if (has(t, "centre") || has(t, "center") || has(t, "ctr"))
        && (has(t, "pump") || has(t, "pumps"))
    {
        if let Some(on) = onoff(t) {
            return Some(call("set_centre_pumps", json!({ "on": on })));
        }
    }

    // Anti-ice and de-ice (#100): "anti-ice on", "engine anti-ice on", "wing
    // anti-ice off", "window heat on", "prop heat on", "boots on", "ice
    // protection on", "de-ice off". Pitot heat and carb heat keep their own
    // tools below.
    if let Some(c) = anti_ice_call(t) {
        return Some(c);
    }

    // <system> on/off
    if let Some(on) = onoff(t) {
        for (kw, key) in [
            ("landing", "landing"),
            ("taxi", "taxi"),
            ("beacon", "beacon"),
            ("strobe", "strobe"),
            ("strobes", "strobe"),
        ] {
            if has(t, kw) {
                return Some(call("set_lights", json!({ "light": key, "on": on })));
            }
        }
        // nav LIGHT (require the word light/lights so "nav one 109" stays radio)
        if has(t, "nav") && (has(t, "light") || has(t, "lights")) {
            return Some(call("set_lights", json!({ "light": "nav", "on": on })));
        }
        // Roof floods (the Baron's cockpit pods). The copilot sets these itself
        // after dark, but the pilot must still be able to say so, and "roof"
        // needs no light/lights qualifier, since nothing else on the aircraft
        // is called one.
        if has(t, "roof") {
            return Some(call("set_lights", json!({ "light": "roof", "on": on })));
        }
        for (kw, key) in [
            ("avionics", "avionics"),
            ("battery", "battery"),
            ("alternator", "alternator"),
            ("generator", "generator"),
            ("master", "master"),
            // The inverter powers the C90's AC bus (avionics/EFIS/autopilot); on
            // airframes without one the tool just lists what it has.
            ("inverter", "inverter"),
            ("inverters", "inverter"),
            // EFIS power switch (separate from avionics master), lights the EADI/EHSI.
            ("efis", "efis"),
            // Bleed air valves (Twin Otter pneumatics: heat, anti-ice, and the
            // AP run off them), a profile key like any other switchable system.
            ("bleed", "bleed"),
        ] {
            if has(t, kw) {
                // Split systems: "alternator two", "battery 1" name one of a
                // numbered pair (SR22 BAT 1/2, ALT 1/2). The tool resolves a
                // bare noun to the pair, and strips the digit back off on
                // airframes with a single unnumbered system.
                let key = if matches!(key, "battery" | "alternator" | "generator") {
                    if has(t, "two") || has(t, "2") {
                        format!("{key}2")
                    } else if has(t, "one") || has(t, "1") {
                        format!("{key}1")
                    } else {
                        key.to_string()
                    }
                } else {
                    key.to_string()
                };
                return Some(call("set_electrical", json!({ "system": key, "on": on })));
            }
        }
        if has(t, "electrical") || has(t, "electrics") || has(t, "electric") {
            return Some(call("set_electrical", json!({ "system": "all", "on": on })));
        }
        // "fuel pump(s)" or "boost pump(s)", the King Air's standby boost pumps
        // are what clears an L/R FUEL PRESS annunciator, and pilots call them
        // either way. "boost" alone counts (whisper often drops "pump").
        if t.contains("fuel pump") || t.contains("boost pump") || has(t, "boost") {
            return Some(call("set_fuel_pump", json!({ "on": on })));
        }
        if has(t, "pitot") {
            return Some(call("set_pitot_heat", json!({ "on": on })));
        }
        if has(t, "carb") || has(t, "carburetor") {
            return Some(call("set_carb_heat", json!({ "on": on })));
        }
    }

    None
}
