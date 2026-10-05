//! The approach's energy, planned BACKWARD from the stabilised gate: a final
//! asks for descent OR deceleration, never both.
//!
//! # Why (LEMG 13, 737, 2026-10-05 13:13 to 13:16)
//!
//! The distance schedule (`autoland::scheduled_ias`: clean speed beyond
//! 10 nm, the approach speed at 5 nm, a straight line between) is a wish,
//! not a plan: it ignores what the airframe can do. The pattern's inbound
//! leg at 14.5 nm read "clean, 210 kt" off it and the governor took the 737
//! from 177 to 222 kt level (13:13:07 to 13:13:29); the path was joined at
//! 208 kt with flaps 5 at 10.7 nm, where a 737 at idle cannot slow at all
//! (on a 3 degree path gravity gives back 1.0 kt/s and flaps 5 make 0.8),
//! and the ramp then asked for 63 kt in 5 nm while the VS law asked for the
//! path: 198 to 177 kt at idle, 570 ft high at 3.9 nm, a go-around.
//!
//! # The model
//!
//! - **What the airframe can do**: each flap detent's level-flight idle
//!   deceleration (`FlapDetent::idle_decel_kts_s`, fitted from its own idle
//!   ticks; the gear as flown, down from half flap). On a path of angle
//!   `g`, idle slows it by that less `G sin g` (1.0 kt/s at 3 degrees).
//!   The configuration a speed allows is the deepest detent whose placard
//!   clears it by the extension margin (maximum drag: an approach that must
//!   slow wants all of it).
//! - **Backward from the gate** ([`on_path_max_kts`]): at the gate the
//!   landing speed, reached [`GATE_MARGIN_S`] early (time for the last flap
//!   to run and settle, the transient the 737's full flap pitched it up
//!   with). Going back up the path, the fastest speed from which idle and
//!   the configuration that speed allows still make it. Where a
//!   configuration cannot slow on the path the curve stops rising: no
//!   faster from further out.
//! - **Descent or deceleration** ([`ask`]): before the path (a pattern's
//!   level legs, a straight-in below the glidepath) the aircraft slows to
//!   the speed the path can be joined at ([`Ask::join_kts`]) and configures
//!   for it; on the path it is never asked for more than the path's own
//!   curve. The vertical law keeps the path either way: descent first.
//! - **Subsumes** 89fe7a6's `landing_flap_lead_nm`: the landing flap goes
//!   where the plan's speed lets it, which is at least [`GATE_MARGIN_S`]
//!   plus its own slowing before the gate, a time from the airframe's
//!   numbers instead of a hand-set distance.
//!
//! An airframe that declares no deceleration keeps the distance schedule
//! ([`EnergyModel::from_profile`] is `None`): a plan is only as good as its
//! fit.

use crate::aircraft::{AircraftProfile, FlapDetent};

/// One g as knots per second.
pub const G_KTS_PER_S: f64 = 9.806_65 * 3600.0 / 1852.0;
/// The landing speed is planned this early before the gate.
pub const GATE_MARGIN_S: f64 = 20.0;
/// A detent counts as available this far under its placard (autoland's own
/// extension margin).
pub const EXTEND_MARGIN_KTS: f64 = 5.0;
/// The backward integration's step, nm.
const STEP_NM: f64 = 0.02;
/// Below this a configuration does not slow on the path.
const NO_DECEL_KTS_S: f64 = 1e-3;

/// One detent as the plan sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detent {
    pub ratio: f64,
    /// Level-flight idle deceleration, kt/s.
    pub decel_kts_s: f64,
    pub vfe_kts: Option<f64>,
}

/// What the airframe can do with its energy.
#[derive(Debug, Clone, PartialEq)]
pub struct EnergyModel {
    /// Retracted first.
    pub detents: Vec<Detent>,
    /// The path angle, degrees.
    pub gp_deg: f64,
}

impl EnergyModel {
    /// The profile's declared deceleration, every detent of it: `None`
    /// when any detent is silent (half a table is not a plan).
    pub fn from_profile(p: &AircraftProfile, gp_deg: f64) -> Option<Self> {
        Self::from_detents(&p.flaps.as_ref()?.detents, gp_deg)
    }

    pub fn from_detents(detents: &[FlapDetent], gp_deg: f64) -> Option<Self> {
        let mut out: Vec<Detent> = detents
            .iter()
            .map(|d| {
                d.idle_decel_kts_s
                    .filter(|a| a.is_finite() && *a > 0.0)
                    .map(|a| Detent {
                        ratio: d.ratio,
                        decel_kts_s: a,
                        vfe_kts: d.vfe_kts,
                    })
            })
            .collect::<Option<Vec<_>>>()?;
        if out.is_empty() || !(gp_deg.is_finite() && gp_deg > 0.0) {
            return None;
        }
        out.sort_by(|a, b| a.ratio.total_cmp(&b.ratio));
        Some(Self {
            detents: out,
            gp_deg,
        })
    }

    /// The deepest detent at or under `flap` (the one out).
    fn detent_at(&self, flap: f64) -> &Detent {
        self.detents
            .iter()
            .rev()
            .find(|d| d.ratio <= flap + 0.02)
            .unwrap_or(&self.detents[0])
    }

    /// Level-flight idle deceleration with `flap` out, kt/s.
    pub fn level_decel(&self, flap: f64) -> f64 {
        self.detent_at(flap).decel_kts_s
    }

    /// Idle deceleration on the path with `flap` out, kt/s (negative: it
    /// gains speed).
    pub fn path_decel(&self, flap: f64) -> f64 {
        self.level_decel(flap) - G_KTS_PER_S * self.gp_deg.to_radians().sin()
    }

    /// The deepest flap available at `kts`: its placard clears the speed by
    /// [`EXTEND_MARGIN_KTS`].
    pub fn flap_for(&self, kts: f64) -> f64 {
        self.detents
            .iter()
            .filter(|d| d.vfe_kts.is_none_or(|v| kts <= v - EXTEND_MARGIN_KTS))
            .map(|d| d.ratio)
            .fold(0.0, f64::max)
    }

    /// The landing flap: the deepest detent.
    pub fn landing_flap(&self) -> f64 {
        self.detents.last().map_or(1.0, |d| d.ratio)
    }
}

/// The fastest speed `s_nm` before the gate on the path from which idle and
/// the configuration each speed allows still reach `vapp_kts` at the gate
/// [`GATE_MARGIN_S`] early. Never under `vapp_kts`. Pure.
pub fn on_path_max_kts(m: &EnergyModel, vapp_kts: f64, s_nm: f64) -> f64 {
    let margin_nm = vapp_kts * GATE_MARGIN_S / 3600.0;
    let mut v = vapp_kts;
    let mut s = margin_nm;
    while s < s_nm {
        let a = m.path_decel(m.flap_for(v));
        if a <= NO_DECEL_KTS_S {
            break;
        }
        let ds = STEP_NM.min(s_nm - s);
        // dV/ds = a / V: ds nm at V kt is ds/V hours.
        v += a * 3600.0 * ds / v;
        s += ds;
    }
    v
}

/// The configuration for the join goes out this long before it (a crew
/// configures before the path, never on it).
pub const CONFIGURE_AHEAD_S: f64 = 30.0;

/// What the plan asks of one tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ask {
    /// The speed to fly now, kt (the governor's target, never faster).
    pub kts: f64,
    /// The configuration for it: the deepest flap that speed allows, owed
    /// on the path and from [`CONFIGURE_AHEAD_S`] before it (`None` before
    /// that: the leg's own schedule).
    pub flap: Option<f64>,
    /// The speed the path is joined at, kt.
    pub join_kts: f64,
    /// Descending the path now (else level before it).
    pub on_path: bool,
}

/// The plan's ask at `s_nm` along the route to the gate, of which the last
/// `path_nm` is the path (the rest is level and flown first): level, slow to
/// the join speed and configure for it; on the path, no faster than the
/// path's own curve here. Pure.
pub fn ask(m: &EnergyModel, vapp_kts: f64, s_nm: f64, path_nm: f64, gs_kts: f64) -> Ask {
    let s = s_nm.max(0.0);
    let path = path_nm.clamp(0.0, s);
    let join_kts = on_path_max_kts(m, vapp_kts, path);
    let on_path = s <= path + 1e-6;
    let kts = if on_path {
        on_path_max_kts(m, vapp_kts, s)
    } else {
        join_kts
    };
    let ahead_nm = gs_kts.max(60.0) * CONFIGURE_AHEAD_S / 3600.0;
    Ask {
        kts,
        flap: (on_path || s - path <= ahead_nm).then(|| m.flap_for(kts)),
        join_kts,
        on_path,
    }
}

/// The speed the plan predicts at the gate from `ias_kts` now, at idle and
/// the configuration each speed allows: level for the part of `s_nm` before
/// the path, then the path. Floored at `vapp_kts` (below it the governor
/// adds power). The tape's `e_gate`: over the gate's limit it says, before
/// the gate, that this final will not be stable. Pure.
pub fn predicted_gate_kts(
    m: &EnergyModel,
    vapp_kts: f64,
    ias_kts: f64,
    s_nm: f64,
    path_nm: f64,
) -> f64 {
    let path = path_nm.clamp(0.0, s_nm.max(0.0));
    let mut s = s_nm.max(0.0);
    let mut v = ias_kts.max(vapp_kts);
    while s > 0.0 && v > vapp_kts {
        let flap = m.flap_for(v);
        let a = if s > path {
            m.level_decel(flap)
        } else {
            m.path_decel(flap)
        };
        let ds = STEP_NM.min(s);
        v = (v - a * 3600.0 * ds / v).max(vapp_kts);
        s -= ds;
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    fn b738() -> EnergyModel {
        let p = crate::aircraft::load("boeing_737_800").expect("737 profile");
        EnergyModel::from_profile(&p, 3.0).expect("the 737 declares its deceleration")
    }

    /// 3 degrees of path is 1.0 kt/s of gravity: the 737 cannot slow on the
    /// path with flaps 5 or cleaner, and slows 0.2 to 0.4 kt/s with 10 to 30
    /// and 1.2 with 40.
    #[test]
    fn the_737_slows_on_the_path_only_with_the_flap_out() {
        let m = b738();
        assert!((G_KTS_PER_S * 3f64.to_radians().sin() - 0.997).abs() < 0.01);
        assert!(m.path_decel(0.0) < 0.0 && m.path_decel(0.375) < 0.0);
        assert!((0.15..0.45).contains(&m.path_decel(0.5)));
        assert!((1.1..1.3).contains(&m.path_decel(1.0)));
        // The configuration a speed allows: flaps 15 at 195, 40 at 157.
        assert_eq!(m.flap_for(195.0), 0.625);
        assert_eq!(m.flap_for(157.0), 1.0);
        assert_eq!(m.flap_for(206.0), 0.375);
        assert_eq!(m.landing_flap(), 1.0);
    }

    /// From the gate back: 145 kt at the gate, the landing flap out from
    /// 157 kt, and no faster than flaps 10's placard less the margin
    /// anywhere on the path (flaps 5 cannot slow on it).
    #[test]
    fn the_path_curve_rises_from_the_gate_and_stops_where_the_flap_cannot_slow() {
        let m = b738();
        let v0 = on_path_max_kts(&m, 145.0, 0.0);
        assert_eq!(v0, 145.0);
        let v1 = on_path_max_kts(&m, 145.0, 1.0);
        let v3 = on_path_max_kts(&m, 145.0, 3.0);
        let v8 = on_path_max_kts(&m, 145.0, 8.0);
        let v20 = on_path_max_kts(&m, 145.0, 20.0);
        assert!(v0 < v1 && v1 < v3 && v3 < v8, "{v1} {v3} {v8}");
        assert!(v20 <= 205.5, "{v20}");
        assert!((185.0..=205.0).contains(&v8), "{v8}");
    }

    /// Level before the path: slow to the join speed and configure for it;
    /// on the path: its own curve. A plan never asks for more than the curve
    /// and the prediction from the curve makes the gate.
    #[test]
    fn level_legs_slow_to_the_join_speed_and_the_path_keeps_its_curve() {
        let m = b738();
        // The LEMG 13 inbound: 11.5 nm to the gate, the path the last 7.65.
        let far = ask(&m, 145.0, 11.5, 7.65, 180.0);
        assert!(!far.on_path);
        assert!((185.0..=200.0).contains(&far.join_kts), "{far:?}");
        assert_eq!(far.kts, far.join_kts);
        assert_eq!(
            far.flap, None,
            "3.85 nm before the join: the leg's own flap"
        );
        let a = ask(&m, 145.0, 8.9, 7.65, 180.0);
        assert!(
            a.flap.is_some_and(|f| f >= 0.625),
            "configured for the join: {a:?}"
        );
        let b = ask(&m, 145.0, 4.0, 7.65, 160.0);
        assert!(b.on_path && b.kts < a.kts && b.flap.is_some());
        // From the plan's own speed the prediction makes the gate.
        let g = predicted_gate_kts(&m, 145.0, a.kts, 11.5, 7.65);
        assert!(g <= 145.0 + 1.0, "{g}");
        // From the recorded 208 kt at the join with flaps 5 it cannot.
        let g = predicted_gate_kts(&m, 145.0, 208.0, 7.65, 7.65);
        assert!(g > 145.0 + 15.0, "{g}");
    }

    /// A silent detent is no plan: the 172 and a half table keep the
    /// distance schedule.
    #[test]
    fn a_half_table_is_not_a_plan() {
        let c172 = crate::aircraft::load("cessna_172sp").expect("172 profile");
        assert!(EnergyModel::from_profile(&c172, 3.0).is_none());
        let p = crate::aircraft::load("boeing_737_800").expect("737");
        let mut flaps = p.flaps.clone().unwrap();
        flaps.detents[3].idle_decel_kts_s = None;
        assert!(EnergyModel::from_detents(&flaps.detents, 3.0).is_none());
    }
}
