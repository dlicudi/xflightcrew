//! Failure awareness — read the *complete* X-Plane failure set, not a subset.
//!
//! X-Plane exposes ~800 failure datarefs under `sim/operation/failures/rel_*`,
//! each 0 = working, 6 = failed/inoperative (1..5 are arm-to-fail-later modes).
//! A copilot that keyword-filters these WILL misdiagnose: a seized engine lives
//! at `rel_seize_0` — no "engine" in the name — so a name-filtered sweep skips
//! the one dataref that matters and reports "nothing failed" over a dead engine.
//! (That happened. This module exists so it can't happen again.)
//!
//! So we enumerate the whole set from the live index and read every value, then
//! map the cryptic names to spoken labels. Unknown names still surface (raw,
//! prettified) rather than being dropped — being *aware of everything* is the
//! entire point.

use anyhow::Result;
use futures::stream::{self, StreamExt};

use crate::xplane::XPlaneClient;

/// The failure dataref namespace. Everything here is `<prefix>rel_*`.
const FAILURE_PREFIX: &str = "sim/operation/failures/rel_";
/// A failure dataref at this value is failed/inoperative now (X-Plane's "fail
/// now" state). Lower values are working or armed-to-fail-later, not yet failed.
const FAILED: f64 = 6.0;
/// Concurrency for the bulk value read — enough to scan ~800 refs in well under a
/// second without flooding the Web API.
const READ_CONCURRENCY: usize = 24;

/// A system bucket, so a multi-failure readout can lead with what matters
/// (engine before a nav radio). Ordered by in-flight priority.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum System {
    Engine,
    Fuel,
    Electrical,
    FlightControls,
    Gear,
    Instruments,
    Other,
}

/// One currently-failed system: the raw dataref (so nothing is ever hidden) plus
/// a spoken label and its bucket.
#[derive(Debug, Clone, PartialEq)]
pub struct ActiveFailure {
    pub dataref: String,
    pub label: String,
    pub system: System,
}

/// Read every failure dataref and return the ones that are failed. Enumerates the
/// full set from the live index (so it adapts per-aircraft and can't miss a ref),
/// then bulk-reads values concurrently. A read that errors is skipped, not
/// treated as failed — a transient miss must never invent a failure.
pub async fn scan(xp: &XPlaneClient) -> Result<Vec<ActiveFailure>> {
    let refs = xp.list_datarefs_by_prefix(FAILURE_PREFIX).await?;
    let mut failed: Vec<ActiveFailure> = stream::iter(refs)
        .map(|(name, id)| async move {
            let v = xp.get_value_by_id(id).await.ok()?;
            let scalar = match &v {
                serde_json::Value::Array(a) => a.first().and_then(serde_json::Value::as_f64),
                other => other.as_f64(),
            }?;
            (scalar >= FAILED).then(|| {
                let (system, label) = classify(&name);
                ActiveFailure { dataref: name, label, system }
            })
        })
        .buffer_unordered(READ_CONCURRENCY)
        .filter_map(|x| async move { x })
        .collect()
        .await;
    // Highest-priority system first, then stable by dataref for determinism.
    failed.sort_by(|a, b| a.system.cmp(&b.system).then_with(|| a.dataref.cmp(&b.dataref)));
    Ok(failed)
}

/// One spoken line summarizing active failures, or `None` when all clear. Leads
/// with the count and the highest-priority items so the pilot hears the engine
/// before the transponder.
pub fn summary(failures: &[ActiveFailure]) -> Option<String> {
    if failures.is_empty() {
        return None;
    }
    let labels: Vec<&str> = failures.iter().map(|f| f.label.as_str()).collect();
    let list = join_spoken(&labels);
    let lead = if failures.len() == 1 {
        "One active failure: "
    } else {
        "Active failures: "
    };
    Some(format!("{lead}{list}."))
}

/// Map a failure dataref name to its system bucket and spoken label. The trailing
/// digit (present on per-engine refs) is an engine index; strip it for matching
/// and add one for the spoken "engine 1". Pure and total: an unrecognized name
/// still yields a prettified label so nothing is ever silently dropped.
fn classify(dataref: &str) -> (System, String) {
    let stem = dataref.strip_prefix(FAILURE_PREFIX).unwrap_or(dataref);
    // Split a trailing engine index ("seize_0" -> ("seize_", 1), "engfai2" -> ("engfai", 3)).
    let (core, eng) = split_engine_index(stem);
    let eng_suffix = eng.map(|n| format!(" {n}")).unwrap_or_default();

    // Substring rules over the core, most-specific first. Each yields (system, base label).
    const RULES: &[(&str, System, &str)] = &[
        ("seize", System::Engine, "engine{E} seized"),
        ("engfir", System::Engine, "engine{E} fire"),
        ("engfla", System::Engine, "engine{E} flameout"),
        ("engsep", System::Engine, "engine{E} separation"),
        ("engfai", System::Engine, "engine{E} failure"),
        ("conrod", System::Engine, "engine{E} thrown connecting rod"),
        ("engcon", System::Engine, "engine{E} won't run"),
        ("engbat", System::Engine, "engine{E} bearing"),
        ("oilpmp", System::Engine, "engine{E} oil pump"),
        ("oilt", System::Engine, "engine{E} oil temperature"),
        ("oilp", System::Engine, "engine{E} oil pressure"),
        ("mag", System::Engine, "engine{E} magneto"),
        ("carb", System::Engine, "engine{E} carburettor ice"),
        ("prop", System::Engine, "engine{E} propeller"),
        ("fuelfl", System::Fuel, "engine{E} fuel flow"),
        ("fuel_block", System::Fuel, "fuel line blocked"),
        ("fuel_leak", System::Fuel, "fuel leak"),
        ("fuel_water", System::Fuel, "water in the fuel"),
        ("fuelcap", System::Fuel, "fuel cap"),
        ("fuelpmp", System::Fuel, "fuel pump"),
        ("fuel", System::Fuel, "fuel system"),
        ("batter", System::Electrical, "battery{E} failure"),
        ("genera", System::Electrical, "generator{E} failure"),
        ("invert", System::Electrical, "inverter{E} failure"),
        ("bus", System::Electrical, "electrical bus"),
        ("brown_out", System::Electrical, "electrical brownout"),
        ("gen", System::Electrical, "generator{E} failure"),
        ("bat", System::Electrical, "battery{E} failure"),
        ("vacuum", System::Instruments, "vacuum pump"),
        ("pitot", System::Instruments, "pitot heat"),
        ("static", System::Instruments, "static port"),
        ("g_asi", System::Instruments, "airspeed indicator"),
        ("g_alt", System::Instruments, "altimeter"),
        ("g_vvi", System::Instruments, "vertical speed indicator"),
        ("g_arthorz", System::Instruments, "attitude indicator"),
        ("g_pfd", System::Instruments, "primary flight display"),
        ("g_mfd", System::Instruments, "multifunction display"),
        ("g430", System::Instruments, "GPS navigator"),
        ("gps", System::Instruments, "GPS"),
        ("gear_ind", System::Gear, "gear indicator"),
        ("gear_act", System::Gear, "landing gear actuator"),
        ("gear", System::Gear, "landing gear"),
        ("fcon", System::FlightControls, "flight control"),
        ("ailn", System::FlightControls, "aileron"),
        ("elev", System::FlightControls, "elevator"),
        ("rudd", System::FlightControls, "rudder"),
        ("flap", System::FlightControls, "flaps"),
        ("sbrk", System::FlightControls, "speedbrake"),
    ];
    for (needle, system, base) in RULES {
        if core.contains(needle) {
            return (*system, base.replace("{E}", &eng_suffix));
        }
    }
    // Unknown — surface it anyway, prettified, so awareness stays complete.
    (System::Other, prettify(core))
}

/// Split a trailing engine index off a core name, returning the 1-based engine
/// number. Handles both `foo_0` and `foo0` shapes. `None` when there's no index.
fn split_engine_index(core: &str) -> (&str, Option<u32>) {
    let trimmed = core.trim_end_matches('_');
    let digits: String = trimmed.chars().rev().take_while(|c| c.is_ascii_digit()).collect();
    if digits.is_empty() {
        return (core, None);
    }
    let n: u32 = digits.chars().rev().collect::<String>().parse().unwrap_or(0);
    let base = &trimmed[..trimmed.len() - digits.len()];
    (base.trim_end_matches('_'), Some(n + 1))
}

/// Fallback label for an unrecognized failure: drop separators, keep it readable.
fn prettify(core: &str) -> String {
    let words: Vec<&str> = core.split(['_', '-']).filter(|w| !w.is_empty()).collect();
    if words.is_empty() {
        core.to_string()
    } else {
        format!("{} fault", words.join(" "))
    }
}

/// Join labels for speech: "a", "a and b", "a, b, and c".
fn join_spoken(items: &[&str]) -> String {
    match items {
        [] => String::new(),
        [a] => a.to_string(),
        [a, b] => format!("{a} and {b}"),
        [rest @ .., last] => format!("{}, and {last}", rest.join(", ")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_seize_that_fooled_us_is_now_classified() {
        // rel_seize_0 has no "eng" in the name — the exact ref a keyword filter
        // skipped. It must classify as an engine seizure, engine 1.
        let (sys, label) = classify("sim/operation/failures/rel_seize_0");
        assert_eq!(sys, System::Engine);
        assert_eq!(label, "engine 1 seized");
    }

    #[test]
    fn engine_index_is_one_based_and_both_name_shapes_work() {
        assert_eq!(classify("sim/operation/failures/rel_seize_1").1, "engine 2 seized");
        assert_eq!(classify("sim/operation/failures/rel_engfai0").1, "engine 1 failure");
        assert_eq!(classify("sim/operation/failures/rel_engfir2").1, "engine 3 fire");
    }

    #[test]
    fn systems_are_bucketed_for_priority() {
        assert_eq!(classify("sim/operation/failures/rel_fuel_leak").0, System::Fuel);
        assert_eq!(classify("sim/operation/failures/rel_batter0").0, System::Electrical);
        assert_eq!(classify("sim/operation/failures/rel_genera1").0, System::Electrical);
        assert_eq!(classify("sim/operation/failures/rel_vacuum").0, System::Instruments);
        assert_eq!(classify("sim/operation/failures/rel_gear_ind").0, System::Gear);
    }

    #[test]
    fn unknown_failures_still_surface_prettified() {
        // Nothing is ever dropped — an unmapped ref becomes a readable "… fault".
        let (sys, label) = classify("sim/operation/failures/rel_whatsit_thing");
        assert_eq!(sys, System::Other);
        assert_eq!(label, "whatsit thing fault");
    }

    #[test]
    fn summary_leads_with_count_and_joins_naturally() {
        let f = |label: &str, system| ActiveFailure {
            dataref: "x".into(),
            label: label.into(),
            system,
        };
        assert_eq!(summary(&[]), None);
        assert_eq!(
            summary(&[f("engine 1 seized", System::Engine)]).unwrap(),
            "One active failure: engine 1 seized."
        );
        assert_eq!(
            summary(&[
                f("engine 1 seized", System::Engine),
                f("fuel leak", System::Fuel),
                f("vacuum pump", System::Instruments),
            ])
            .unwrap(),
            "Active failures: engine 1 seized, fuel leak, and vacuum pump."
        );
    }
}
