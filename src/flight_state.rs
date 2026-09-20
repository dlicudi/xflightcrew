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
//! Two doors:
//! - [`publish_airframe`]: every selector the tools read per airframe, from
//!   the profile, including to `None`. Called at boot and on every swap.
//! - [`stand_down`]: every loop that can hold the aircraft, stopped. Called
//!   on a swap (a loop cannot fly a different aircraft), on entering
//!   instructor mode (hands off), and when the sim connection is lost.
//!   `stand_down_pins_every_active_loop` keeps this list equal to every
//!   `static ACTIVE` in the tree, so a new loop cannot be forgotten here.
//!
//! Inventory of what is NOT yet cleared here and lives in its own module,
//! per FLIGHT rather than per airframe (each reset on its own path today;
//! the next step is a `flight_begins` door for them): `autotaxi::LAST_ARRIVED`,
//! `autoland::LANDED_RUNWAY` / `VACATE_FOLLOWS` / `TRACKING_ERROR`,
//! `full_auto::INBOUND_MADE` / `STAGE` / `MISSION`, `procedure::TOD` /
//! `TERRAIN_FLOOR` / `PILOT_ALT` / `LEG`, `pushback::LEARNED_SENSE`,
//! `tools::ground::CART_REQUESTED`, `taxi_in::ARREST_HOLD`,
//! `cockpit_auto::PILOT_MOVED_LIGHTS`, `instructor::LESSON`, `flow::*`.

use crate::aircraft::AircraftProfile;

/// Publish the per-airframe selectors the tools read. Every one is set from
/// the profile, including to `None`: a process-wide fact must never outlive
/// its airframe, and a selector that is only set when present is the bug.
pub(crate) fn publish_airframe(p: &AircraftProfile) {
    crate::tools::radio::publish_mode_knob(p.transponder_mode_knob.clone());
    crate::tools::lights::publish_exterior(&p.lights);
    crate::aircraft::publish_throttle_dataref(p.throttle_dataref.clone());
    crate::aircraft::publish_config_limits(p.vlo_kts, p.flaps.as_ref());
    crate::aircraft::publish_bus_volts(p.bus_volts_nominal);
    crate::aircraft::publish_speed_floor(p.vs_clean_kts);
    crate::aircraft::publish_approach_floor(
        p.reference.as_ref().and_then(|r| r.vs0_kts),
        p.vs_clean_kts,
    );
    crate::aircraft::publish_approach_thrust_min(p.approach_thrust_min);
    crate::aircraft::publish_arrest_lead(p.arrest_lead);
    crate::aircraft::publish_landing_distance(
        p.reference.as_ref().and_then(|r| r.landing_distance_ft),
    );
    crate::procedure::publish_vs_limits(p.reference.as_ref());
    crate::aircraft::publish_fuel_unit(p.fuel.as_ref());
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
    crate::approach_guide::cancel();
    crate::autotakeoff::cancel();
    crate::flow::abort();
    crate::autotaxi::ACTIVE.store(false, SeqCst);
    crate::autotaxi::BACKTRACKING.store(false, SeqCst);
    crate::tools::speedhold::ACTIVE.store(false, SeqCst);
    crate::tools::reverse::ACTIVE.store(false, SeqCst);
    crate::tools::engine::MIX_MANAGED.store(false, SeqCst);
    crate::tools::engine::LEAN_SEEKING.store(false, SeqCst);
    crate::pushback::ACTIVE.store(false, SeqCst);
    // The arrival's hold on the station is a fact about a flight, not about
    // the world: nothing is flying to that destination any more, so the
    // region's ordinary nearest-field handoffs resume.
    crate::full_auto::release_arrival_station();
}

#[cfg(test)]
mod tests {
    /// Every module in the tree that declares a `static ACTIVE` loop flag is
    /// named in [`super::stand_down`], so a new loop cannot outlive a swap by
    /// being forgotten here. The list is read from the source, like the
    /// plugin contract's.
    #[test]
    fn stand_down_pins_every_active_loop() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let this = std::fs::read_to_string(root.join("flight_state.rs")).unwrap();
        let body = this.split("pub(crate) fn stand_down()").nth(1).unwrap();
        let body = body.split("#[cfg(test)]").next().unwrap();
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
            let declares = text.lines().any(|l| {
                let l = l.trim_start();
                l.strip_prefix("pub(crate) ")
                    .or_else(|| l.strip_prefix("pub "))
                    .unwrap_or(l)
                    .starts_with("static ACTIVE:")
            });
            if !declares {
                continue;
            }
            checked += 1;
            let rel = f.strip_prefix(&root).unwrap().with_extension("");
            let module = rel
                .components()
                .map(|c| c.as_os_str().to_string_lossy().to_string())
                .collect::<Vec<_>>()
                .join("::");
            if !body.contains(&format!("crate::{module}::")) {
                missing.push(module);
            }
        }
        assert!(checked >= 10, "expected the loop modules, found {checked}");
        assert!(missing.is_empty(), "loops with a static ACTIVE not in stand_down: {missing:?}");
    }
}
