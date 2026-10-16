//! The nosewheel tiller: the plugin's `xcopilot/tiller` channel, written beside
//! the pedal channel (`xcopilot/taxi_steer`) where the airframe declares one
//! (`taxi: tiller:`, [`crate::aircraft::TillerProfile`]).
//!
//! The pedals of an airliner steer the nose a few degrees: auto taxi held full
//! pedal on the Zibo 737 at 8 kt with 152 degrees to turn and rolled straight
//! until it was 90 m off EGHL's route (2026-10-07). The tiller turns it 78.
//! The steering laws are unchanged: their command already means "this share of
//! the nose's low-speed authority" on a stock airframe, where X-Plane gives the
//! pedals the full low-speed angle, so the same command goes to the tiller
//! (the 737 sums tiller and pedals) and the airframe that needs it answers
//! like the stock one does. The tiller's share fades from whole below
//! `full_below_kts` to nothing at `hand_off_kts` ([`share`]): a crew steers
//! the takeoff roll and the fast part of the rollout on the pedals. The
//! takeoff roll never writes it.
//!
//! An airframe that declares nothing writes nothing here: [`Tiller::none`] is
//! inert, so stock airframes are byte-identical.

use crate::aircraft::TillerProfile;
use crate::xplane::XPlane;

/// The plugin's tiller channel (-1..1, right positive), a 1 s heartbeat.
pub const TILLER: &str = "xcopilot/tiller";
/// What the channel drives (`TillerMethod::code`), set once per arm.
pub const TILLER_METHOD: &str = "xcopilot/tiller_method";
/// Full deflection in nosewheel degrees, for the stock method.
pub const TILLER_MAX_DEG: &str = "xcopilot/tiller_max_deg";

/// The tiller is let go at and above this groundspeed when the profile is
/// silent, knots.
pub const HAND_OFF_KTS: f64 = 30.0;
/// And takes the whole command below this one.
pub const FULL_BELOW_KTS: f64 = 20.0;

/// The share of the steer command the tiller takes at this groundspeed: 1
/// below `full_below_kts`, 0 at and above `hand_off_kts`, linear between. No
/// groundspeed (not finite) is no tiller. Pure.
pub fn share(t: &TillerProfile, gs_kts: f64) -> f64 {
    let off = t.hand_off_kts.unwrap_or(HAND_OFF_KTS);
    let full = t.full_below_kts.unwrap_or(FULL_BELOW_KTS).min(off);
    let gs = gs_kts.abs();
    if !gs.is_finite() || gs >= off {
        0.0
    } else if gs <= full {
        1.0
    } else {
        (off - gs) / (off - full)
    }
}

/// The tiller command for a steer command at this groundspeed. Pure.
pub fn command(t: &TillerProfile, steer: f64, gs_kts: f64) -> f64 {
    if !steer.is_finite() {
        return 0.0;
    }
    (steer * share(t, gs_kts)).clamp(-1.0, 1.0)
}

/// One loop's tiller: the airframe's declaration, and whether the plugin has
/// been told the method. A failed arm (a plugin without the channel) leaves it
/// inert for the loop: the pedals steer as before, said once in the log.
#[derive(Debug, Clone, Default)]
pub struct Tiller {
    profile: Option<TillerProfile>,
}

impl Tiller {
    /// Nothing declared: every call is a no-op.
    #[cfg(test)]
    pub fn none() -> Self {
        Self { profile: None }
    }

    /// The live airframe's.
    pub fn for_airframe() -> Self {
        Self {
            profile: crate::aircraft::tiller(),
        }
    }

    #[cfg(test)]
    pub fn with(profile: TillerProfile) -> Self {
        Self {
            profile: Some(profile),
        }
    }

    pub fn declared(&self) -> bool {
        self.profile.is_some()
    }

    /// Tell the plugin what the channel drives. Call before the first
    /// [`Tiller::command`] is written; a failure makes this loop's tiller
    /// inert.
    pub async fn arm(&mut self, xp: &(dyn XPlane + '_)) {
        let Some(t) = self.profile.clone() else {
            return;
        };
        let method = xp.set_int(TILLER_METHOD, i64::from(t.method.code())).await;
        let max = match t.max_deg {
            Some(d) => xp.set_float(TILLER_MAX_DEG, d).await,
            None => Ok(()),
        };
        let ok = method.is_ok() && max.is_ok();
        crate::blackbox::record(
            "autotaxi",
            serde_json::json!({"event": "tiller_armed", "method": t.method, "ok": ok}),
        );
        if !ok {
            tracing::warn!("tiller: the plugin has no tiller channel; steering on the pedals");
            self.profile = None;
        }
    }

    /// The tiller command for this tick: `None` when nothing is declared, or
    /// at and above the hand-off where the tiller has no share. Not written
    /// there, the plugin lets it go within the second and the pedals steer:
    /// written as 0 it stayed applied, and on the Zibo's YAW option the
    /// plugin put the tiller's 0 on the yaw axis in place of the pedal
    /// command. The LEMG 30 rollout asked 0.65 right for 40 s from 135 kt,
    /// turned 29 degrees left and ran 94 m off the centreline (2026-10-07).
    pub fn command(&self, steer: f64, gs_kts: f64) -> Option<f64> {
        let t = self.profile.as_ref()?;
        if share(t, gs_kts) <= 0.0 {
            return None;
        }
        Some(command(t, steer, gs_kts))
    }

    /// Write this tick's command (a no-op when nothing is declared).
    pub async fn write(&self, xp: &(dyn XPlane + '_), steer: f64, gs_kts: f64) {
        if let Some(c) = self.command(steer, gs_kts) {
            let _ = xp.set_float(TILLER, c).await;
        }
    }

    /// Let go now (expiry would within the second).
    pub async fn release(&self, xp: &(dyn XPlane + '_)) {
        if self.profile.is_some() {
            let _ = xp.set_float(TILLER, 0.0).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft::TillerMethod;

    fn zibo() -> TillerProfile {
        TillerProfile {
            method: TillerMethod::ZiboVr,
            max_deg: Some(78.0),
            hand_off_kts: None,
            full_below_kts: None,
        }
    }

    /// The taxi gets the whole command; the takeoff roll's speeds none; the
    /// band between fades, so the hand-off is never a step.
    #[test]
    fn the_tiller_is_whole_at_taxi_speed_and_gone_by_the_hand_off() {
        let t = zibo();
        assert_eq!(share(&t, 0.0), 1.0);
        assert_eq!(share(&t, 8.0), 1.0);
        assert_eq!(share(&t, 20.0), 1.0);
        assert!((share(&t, 25.0) - 0.5).abs() < 1e-9);
        assert_eq!(share(&t, 30.0), 0.0);
        assert_eq!(share(&t, 80.0), 0.0);
        assert_eq!(share(&t, f64::NAN), 0.0);
        let mut prev = 1.0;
        for i in 0..=60 {
            let s = share(&t, f64::from(i) * 0.5);
            assert!(s <= prev + 1e-12, "monotone");
            assert!(prev - s < 0.06, "no step at {}", f64::from(i) * 0.5);
            prev = s;
        }
    }

    /// The EGHL case: full left steer at 8 kt is full left tiller.
    #[test]
    fn full_steer_at_taxi_speed_is_full_tiller() {
        let t = zibo();
        assert_eq!(command(&t, -1.0, 8.0), -1.0);
        assert_eq!(command(&t, 0.4, 8.0), 0.4);
        assert_eq!(command(&t, 3.0, 8.0), 1.0);
        assert_eq!(command(&t, f64::NAN, 8.0), 0.0);
        assert_eq!(command(&t, -1.0, 45.0), 0.0);
    }

    #[test]
    fn a_profile_moves_the_hand_off() {
        let t = TillerProfile {
            hand_off_kts: Some(15.0),
            full_below_kts: Some(10.0),
            ..zibo()
        };
        assert_eq!(share(&t, 10.0), 1.0);
        assert!((share(&t, 12.5) - 0.5).abs() < 1e-9);
        assert_eq!(share(&t, 15.0), 0.0);
        // A full-below past the hand-off is clamped to it: no division by 0.
        let odd = TillerProfile {
            hand_off_kts: Some(10.0),
            full_below_kts: Some(20.0),
            ..zibo()
        };
        assert_eq!(share(&odd, 9.0), 1.0);
        assert_eq!(share(&odd, 10.0), 0.0);
    }

    /// Nothing declared writes nothing: stock airframes byte-identical.
    #[tokio::test]
    async fn an_undeclared_tiller_writes_nothing() {
        let xp = crate::xplane::FakeXPlane::new();
        let mut t = Tiller::none();
        t.arm(&xp).await;
        t.write(&xp, -1.0, 8.0).await;
        t.release(&xp).await;
        assert!(xp.writes_to(TILLER).is_empty());
        assert!(xp.writes_to(TILLER_METHOD).is_empty());
    }

    #[tokio::test]
    async fn a_declared_tiller_arms_then_writes_the_faded_command() {
        let xp = crate::xplane::FakeXPlane::new();
        let mut t = Tiller::with(zibo());
        t.arm(&xp).await;
        assert_eq!(xp.writes_to(TILLER_METHOD), vec![1.0]);
        assert_eq!(xp.writes_to(TILLER_MAX_DEG), vec![78.0]);
        t.write(&xp, -1.0, 8.0).await;
        t.write(&xp, -1.0, 25.0).await;
        // At and above the hand-off: nothing written, the pedals steer.
        t.write(&xp, -1.0, 30.0).await;
        t.write(&xp, -1.0, 120.0).await;
        t.release(&xp).await;
        assert_eq!(xp.writes_to(TILLER), vec![-1.0, -0.5, 0.0]);
        assert_eq!(t.command(0.65, 100.0), None);
    }

    /// The channel's names are the plugin's, listed for the stale-plugin
    /// check, and the method codes are the plugin's enum.
    #[test]
    fn the_tiller_datarefs_are_listed_and_registered() {
        let plugin = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/plugin/src/tiller.cpp"
        ))
        .unwrap();
        for name in [TILLER, TILLER_METHOD, TILLER_MAX_DEG] {
            assert!(crate::health::PLUGIN_DATAREFS.contains(&name), "{name}");
            assert!(
                plugin.contains(&format!("\"{name}\"")),
                "{name} not registered"
            );
        }
        assert!(plugin.contains(
            "enum Method { kNone = 0, kZiboVr = 1, kZiboYaw = 2, kZiboOverride = 3, kStock = 4 };"
        ));
        assert_eq!(TillerMethod::ZiboVr.code(), 1);
        assert_eq!(TillerMethod::ZiboYaw.code(), 2);
        assert_eq!(TillerMethod::ZiboOverride.code(), 3);
        assert_eq!(TillerMethod::Stock.code(), 4);
    }

    /// The Zibo's profile declares it, and the probe names what the plugin
    /// will write.
    #[test]
    fn the_zibo_declares_its_tiller_and_the_probe_lists_its_names() {
        let p = crate::aircraft::load("zibo_738").expect("zibo profile");
        let t = p
            .taxi
            .as_ref()
            .and_then(|t| t.tiller.clone())
            .expect("tiller");
        assert_eq!(t.method, TillerMethod::ZiboYaw);
        let refs = crate::aircraft::profile_references(&p);
        for name in t.method.references() {
            assert!(refs.iter().any(|r| r == name), "{name}");
        }
        // And no stock profile declares one.
        for id in ["boeing_737_800", "cessna_172sp", "md82", "airbus_a330_300"] {
            let p = crate::aircraft::load(id).expect(id);
            assert!(
                p.taxi.as_ref().and_then(|t| t.tiller.as_ref()).is_none(),
                "{id}"
            );
        }
    }
}
