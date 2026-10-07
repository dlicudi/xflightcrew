//! The yoke route: what the plugin's pitch and roll channels (`xcopilot/pitch`,
//! `xcopilot/roll`) drive on this airframe (`yoke_input:`,
//! [`crate::aircraft::YokeInput`]).
//!
//! On the stock path the plugin writes the yoke ratios under the stock
//! joystick overrides. The Zibo 737 holds those overrides itself and rewrites
//! the ratios every frame: the LXGB takeoff of 2026-10-07 held full nose-down
//! on the channel for 3 s while the nose rose from 10 to 22 degrees, and the
//! autopilot took a 6,600 fpm climb. Its own input
//! (`laminar/B738/flt_ctrls/joy_override` and `joy_pitch` / `joy_roll`)
//! moved the elevator and ailerons with the stock sense (probed parked).
//!
//! Every loop that writes the channels arms the route first, from the LIVE
//! airframe, including the stock 0: the plugin's method must never be the
//! last airframe's.

use crate::xplane::XPlane;

/// What the plugin's yoke channels drive (`YokeInput::code`).
pub const YOKE_METHOD: &str = "xcopilot/yoke_method";

/// Tell the plugin the live airframe's route. A plugin without the dataref
/// (stale) is said in the log and blackboxed; the stock path is then all it
/// can do.
pub async fn arm(xp: &(dyn XPlane + '_), site: &'static str) {
    let input = crate::aircraft::yoke_input();
    let ok = xp
        .set_int(
            YOKE_METHOD,
            i64::from(crate::aircraft::YokeInput::code(input)),
        )
        .await
        .is_ok();
    if input.is_some() || !ok {
        crate::blackbox::record(
            "yoke",
            serde_json::json!({"event": "armed", "input": input, "ok": ok, "site": site}),
        );
    }
    if !ok && input.is_some() {
        tracing::warn!(
            "yoke: the plugin has no yoke route; the pitch channel cannot reach this airframe"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft::YokeInput;

    #[test]
    fn the_codes_are_the_plugins() {
        let plugin =
            std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/plugin/src/drefs.cpp"))
                .unwrap();
        assert!(plugin.contains(&format!("\"{YOKE_METHOD}\"")));
        assert!(crate::health::PLUGIN_DATAREFS.contains(&YOKE_METHOD));
        assert_eq!(YokeInput::code(None), 0);
        assert!(plugin.contains("kYokeStock = 0"));
        assert_eq!(YokeInput::code(Some(YokeInput::ZiboJoy)), 1);
        assert!(plugin.contains("kYokeZiboJoy = 1"));
        for name in YokeInput::ZiboJoy.references() {
            assert!(plugin.contains(&format!("\"{name}\"")), "{name}");
        }
    }
}
