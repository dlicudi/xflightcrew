//! The seat belt signs, worked the way a crew works them: on before the
//! engines start, off through ten thousand in the climb, on again through ten
//! thousand on the way down.
//!
//! Until 2026-09-21 the only hand on the switch was the A330's advisor row,
//! which noticed the signs were off four seconds after LIFTOFF and pressed
//! the switch once (OFF to AUTO), then again seventy seconds later when AUTO
//! read dark with the gear and flaps up. A sign that goes on at rotation is
//! not a sign.
//!
//! An airframe whose switch is its own (the A330 rewrites the stock dataref
//! from `laminar/A333/switches/fasten_seatbelts` every frame) declares it in
//! the profile's `seat_belt_signs:` and is stepped by command, read before
//! every step. Any other airframe with a cabin gets the stock switch written.
//! An airframe with neither has no signs and nothing is touched.

use crate::aircraft::SignsSwitch as Signs;
use crate::xplane::XPlane;

/// 0 off, 1 auto, 2 on.
pub(crate) const STOCK_SWITCH: &str = "sim/cockpit2/switches/fasten_seat_belts";

/// The signs change at this altimeter reading, with a band so a level-off at
/// ten thousand does not ring the cabin chime every minute.
pub const SIGNS_ALT_FT: f64 = 10_000.0;
const SIGNS_BAND_FT: f64 = 300.0;

/// Publish the signs alone (the snapshot's `signs`, republished on every
/// swap, including to nothing, by [`crate::flight_state::publish_airframe`]).
#[cfg(test)]
pub fn publish(own: Option<crate::aircraft::SeatBeltSigns>, has_cabin: bool) {
    let s = Signs::for_profile(own, has_cabin);
    crate::aircraft::update_airframe_facts(|a| a.signs = s);
}

/// The live airframe's seat belt switch, `None` with no signs.
fn signs() -> Option<Signs> {
    crate::aircraft::airframe_facts().signs.clone()
}

/// Switch the signs. `None` on an airframe with none; otherwise whether the
/// switch READ BACK where it was asked.
pub async fn set(xp: &dyn XPlane, on: bool) -> Option<bool> {
    let signs = signs()?;
    let ok = match signs {
        Signs::Stock => {
            let want = if on { 2 } else { 0 };
            xp.set_int(STOCK_SWITCH, want).await.is_ok()
                && xp
                    .get_float(STOCK_SWITCH)
                    .await
                    .is_ok_and(|v| v.round() as i64 == want)
        }
        Signs::Own(s) => {
            let want = if on { s.on } else { s.off };
            super::radio::step_knob(xp, &s.position, want, &s.up_command, &s.down_command).await
        }
    };
    crate::blackbox::record(
        "cabin",
        serde_json::json!({"event": "seat_belt_signs", "on": on, "ok": ok}),
    );
    Some(ok)
}

/// Which way the signs go as the flight passes ten thousand. Fed the
/// altimeter every pass; answers once per crossing.
#[derive(Default)]
pub struct Watch {
    above: bool,
}

impl Watch {
    /// `Some(false)` climbing through, `Some(true)` descending through.
    pub fn tick(&mut self, alt_ft: f64) -> Option<bool> {
        if !alt_ft.is_finite() {
            return None;
        }
        if !self.above && alt_ft > SIGNS_ALT_FT + SIGNS_BAND_FT {
            self.above = true;
            return Some(false);
        }
        if self.above && alt_ft < SIGNS_ALT_FT - SIGNS_BAND_FT {
            self.above = false;
            return Some(true);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::await_holding_lock)]
    use super::*;
    use crate::xplane::FakeXPlane;

    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn the_signs_change_once_per_crossing() {
        let mut w = Watch::default();
        assert_eq!(w.tick(3_000.0), None);
        assert_eq!(w.tick(10_100.0), None, "inside the band");
        assert_eq!(w.tick(10_400.0), Some(false));
        assert_eq!(
            w.tick(10_050.0),
            None,
            "a level-off at ten is not a descent"
        );
        assert_eq!(w.tick(35_000.0), None);
        assert_eq!(w.tick(f64::NAN), None, "no measurement");
        assert_eq!(w.tick(9_600.0), Some(true));
        assert_eq!(w.tick(4_000.0), None);
    }

    /// The A330's switch is its own and read-only: stepped by command from
    /// OFF through AUTO to ON, never written.
    #[tokio::test(start_paused = true)]
    async fn the_a330s_switch_is_stepped_not_written() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let p = crate::aircraft::load("airbus_a330_300").expect("A330 profile loads");
        let s = p.seat_belt_signs.clone().expect("declared");
        assert_eq!(
            (s.off, s.on),
            (1, 2),
            "AUTO in cruise, ON for the ground and below ten"
        );
        publish(Some(s.clone()), p.cabin_crew);

        let on = FakeXPlane::with_reads(&[(s.position.as_str(), 2.0)]);
        assert_eq!(set(&on, true).await, Some(true));
        assert!(!on.ran(&s.up_command), "already on: nothing pressed");

        let off = FakeXPlane::with_reads(&[(s.position.as_str(), 0.0)]);
        assert_eq!(
            set(&off, true).await,
            Some(false),
            "a switch that never moves is said"
        );
        assert!(off.ran(&s.up_command) && !off.wrote(&s.position));

        publish(None, false);
        assert_eq!(set(&on, true).await, None, "no cabin, no signs");
    }

    #[tokio::test]
    async fn a_stock_cabin_gets_the_stock_switch() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish(None, true);
        let xp = FakeXPlane::with_reads(&[(STOCK_SWITCH, 0.0)]);
        set(&xp, true).await;
        assert_eq!(xp.last_write(STOCK_SWITCH), Some(2.0));
        publish(None, false);
    }
}
