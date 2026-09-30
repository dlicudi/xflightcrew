//! The stabilised approach: ONE rule for every final autoland flies, the
//! circuit's handoff included (#199).
//!
//! The airline rule (Flight Safety Foundation ALAR 7.1, and every SOP built on
//! it): by a GATE the aircraft is in the landing configuration, on speed, on
//! the path and not sinking hard, and an approach that is not, or that stops
//! being so below the gate, is a go-around THEN, with height in hand. Never a
//! continuation to see whether the flare can save it.
//!
//! - The gate: 1,000 ft above the runway on an instrument or straight-in final
//!   ([`GateKind::Instrument`]), 500 ft on a visual or circuit final
//!   ([`GateKind::Visual`]).
//! - Configuration from the gate: the gear down and the landing flap SELECTED
//!   (what the approach has commanded: the handle, as a crew calls it).
//! - Speed: never more than [`SLOW_KTS`] under the landing target (the
//!   margin speed protection itself acts at) and never more than
//!   [`FAST_KTS`] over it from [`STABLE_HEIGHT_AAL_FT`] down (Vref to
//!   Vref + 20 on a Vref + 5 target). Between an instrument gate and 500 ft a
//!   speed still bleeding off in the landing configuration may be up to
//!   [`FAST_ABOVE_STABLE_KTS`] over (configured by 1,000, stable by 500).
//! - Sink: no more than 1,000 fpm, or the path's own sink at the TARGET speed
//!   over the ground plus the path law's catch-up ([`SINK_MARGIN_FPM`]) where
//!   the path itself demands more (a steep visual segment, a tailwind). The
//!   speed the approach should be flying, never the speed it is flying: 255 kt
//!   "demands" 1,350 fpm.
//! - Path: within [`PATH_TOL_DEG`] of the path angle (two dots of glideslope).
//! - Lateral: the gate's own time model (`reversal::stabilisable`,
//!   `autoland::guidance`'s `unstabilised_at_gate`), unchanged.
//!
//! A breach counts only once it has held [`PERSIST_S`] ([`StableWatch`]): a
//! gust through the band is not an unstable approach, a final that stays out
//! of it is. Judged from the gate down to [`JUDGE_FLOOR_AAL_FT`]; below that
//! the roundout, the flare and their own gates own the landing.
//!
//! EGNX 09, iMac 2026-09-30 (sortie 1790761044-full_auto, the stock 737 on a
//! 40 nm local plan from EGBB): the final was flown at 254 to 255 KIAS with
//! the flaps up (their placards held them) and 1,362 to 1,419 fpm down from
//! 1,000 ft to 121 ft radio, the approach speed 145. The only gate that
//! existed was lateral, and the 737 was on the centreline. It ballooned out
//! of the arrest at 150 ft. Under this rule: `fast` from the gate, a
//! go-around at about 920 ft above the runway.

/// Which gate an approach is judged at.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GateKind {
    /// An instrument or straight-in final: stable by 1,000 ft.
    #[default]
    Instrument,
    /// A visual or circuit final: stable by 500 ft.
    Visual,
}

impl GateKind {
    /// The gate, feet above the runway.
    pub fn gate_ft(self) -> f64 {
        match self {
            GateKind::Instrument => 1000.0,
            GateKind::Visual => STABLE_HEIGHT_AAL_FT,
        }
    }

    /// The blackbox's word.
    pub fn word(self) -> &'static str {
        match self {
            GateKind::Instrument => "instrument",
            GateKind::Visual => "visual",
        }
    }
}

/// Fully stable from here down, feet above the runway: the visual gate, and
/// where an instrument final's speed allowance closes to the band.
pub const STABLE_HEIGHT_AAL_FT: f64 = 500.0;
/// Below this the roundout and the flare own the landing.
pub const JUDGE_FLOOR_AAL_FT: f64 = 200.0;
/// Slower than the landing target by more than this is unstable: the margin
/// autoland's speed protection lowers the nose at (`SPEED_FLOOR_MARGIN_KTS`).
/// The SR22's circuit finals of 2026-09-30 sat at 70 to 72 against 75.
pub const SLOW_KTS: f64 = 7.0;
/// Faster than the target by more than this is unstable from
/// [`STABLE_HEIGHT_AAL_FT`] down.
pub const FAST_KTS: f64 = 15.0;
/// ...and between an instrument gate and 500 ft, with the landing flap
/// selected. The 737's EGNX 09 second approach (2026-09-30) and its EGBB 33
/// final (sortie 1790759458, 169 kt at 1,008 ft, 151 by 500) are inside it.
pub const FAST_ABOVE_STABLE_KTS: f64 = 25.0;
/// The sink never needs a special brief under this, fpm.
pub const SINK_FLOOR_FPM: f64 = 1000.0;
/// Over the path's own sink at the target speed: the path law's 300 fpm
/// catch-up and 100 of air.
pub const SINK_MARGIN_FPM: f64 = 400.0;
/// Off the path angle by more than this, degrees: two dots of glideslope.
pub const PATH_TOL_DEG: f64 = 0.7;
/// ...never tighter than this, feet (the last half mile).
pub const PATH_TOL_MIN_FT: f64 = 50.0;
/// A breach counts once it has held this long, seconds.
pub const PERSIST_S: f64 = 3.0;

/// One criterion an approach fails.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Breach {
    Gear,
    Flap,
    Fast,
    Slow,
    Sink,
    High,
    Low,
}

impl Breach {
    /// Every breach, in the order a crew calls them.
    pub const ALL: [Breach; 7] = [
        Breach::Gear,
        Breach::Flap,
        Breach::Fast,
        Breach::Slow,
        Breach::Sink,
        Breach::High,
        Breach::Low,
    ];

    /// The balk word (`going_around` `why`).
    pub fn word(self) -> &'static str {
        match self {
            Breach::Gear => "unstabilised_gear",
            Breach::Flap => "unstabilised_flap",
            Breach::Fast => "unstabilised_fast",
            Breach::Slow => "unstabilised_slow",
            Breach::Sink => "unstabilised_sink",
            Breach::High => "unstabilised_high",
            Breach::Low => "unstabilised_low",
        }
    }

    /// Said on the intercom.
    pub fn spoken(self) -> &'static str {
        match self {
            Breach::Gear => "the gear is not down",
            Breach::Flap => "no landing flap",
            Breach::Fast => "too fast",
            Breach::Slow => "too slow",
            Breach::Sink => "sinking too fast",
            Breach::High => "high on the path",
            Breach::Low => "low on the path",
        }
    }

    fn index(self) -> usize {
        Breach::ALL.iter().position(|b| *b == self).unwrap_or(0)
    }
}

/// What the rule reads, one tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Feet above the runway (the threshold's elevation).
    pub height_aal_ft: f64,
    /// Miles to the touchdown aim along the final.
    pub dist_to_aim_nm: f64,
    pub ias_kts: f64,
    pub gs_kts: f64,
    /// The landing configuration's speed target.
    pub target_kts: f64,
    /// Vertical speed, fpm (negative down); `None` unread: not judged.
    pub vvi_fpm: Option<f64>,
    /// Feet above the path (negative below).
    pub above_path_ft: f64,
    /// The path's gradient, feet per mile (318 at 3 degrees).
    pub path_ft_per_nm: f64,
    /// The gear handle is down (a fixed gear is always down).
    pub gear_down: bool,
    /// The landing flap is selected.
    pub landing_flap: bool,
}

/// Is this height judged by the rule behind `gate`?
pub fn judged(gate: GateKind, height_aal_ft: f64) -> bool {
    (JUDGE_FLOOR_AAL_FT..=gate.gate_ft()).contains(&height_aal_ft)
}

/// The most a stabilised final may sink, fpm (positive): 1,000, or the
/// path's own sink at the TARGET speed over the ground plus the catch-up.
/// `gs_kts / ias_kts` carries the wind and the air's density over. Pure.
pub fn sink_limit_fpm(target_kts: f64, ias_kts: f64, gs_kts: f64, path_ft_per_nm: f64) -> f64 {
    let ratio = if ias_kts > 30.0 && gs_kts > 0.0 {
        (gs_kts / ias_kts).clamp(0.5, 2.0)
    } else {
        1.0
    };
    let path_fpm = target_kts * ratio / 60.0 * path_ft_per_nm;
    SINK_FLOOR_FPM.max(path_fpm + SINK_MARGIN_FPM)
}

/// How far off the path is still on it, feet, at `dist_to_aim_nm`. Pure.
pub fn path_tolerance_ft(dist_to_aim_nm: f64) -> f64 {
    (dist_to_aim_nm.max(0.0) * 6076.1 * PATH_TOL_DEG.to_radians().tan()).max(PATH_TOL_MIN_FT)
}

/// Every criterion the sample fails behind `gate`, in the order a crew calls
/// them; empty is stable. Not judged outside [`judged`]. Pure.
pub fn breaches(gate: GateKind, s: &Sample) -> Vec<Breach> {
    let mut out = Vec::new();
    if !judged(gate, s.height_aal_ft) {
        return out;
    }
    if !s.gear_down {
        out.push(Breach::Gear);
    }
    if !s.landing_flap {
        out.push(Breach::Flap);
    }
    let fast_by = if s.height_aal_ft > STABLE_HEIGHT_AAL_FT {
        FAST_ABOVE_STABLE_KTS
    } else {
        FAST_KTS
    };
    if s.ias_kts > s.target_kts + fast_by {
        out.push(Breach::Fast);
    }
    if s.ias_kts < s.target_kts - SLOW_KTS {
        out.push(Breach::Slow);
    }
    if let Some(vvi) = s.vvi_fpm {
        if -vvi > sink_limit_fpm(s.target_kts, s.ias_kts, s.gs_kts, s.path_ft_per_nm) {
            out.push(Breach::Sink);
        }
    }
    let tol = path_tolerance_ft(s.dist_to_aim_nm);
    if s.above_path_ft > tol {
        out.push(Breach::High);
    } else if s.above_path_ft < -tol {
        out.push(Breach::Low);
    }
    out
}

/// The rule over time: a breach is the approach's once it has held
/// [`PERSIST_S`]. `t_s` is seconds on any monotonic clock.
#[derive(Debug, Default, Clone)]
pub struct StableWatch {
    since: [Option<f64>; 7],
}

impl StableWatch {
    /// Note this tick's breaches (empty when stable or not judged); the first
    /// one to have held [`PERSIST_S`], in the crew's order.
    pub fn step(&mut self, t_s: f64, now: &[Breach]) -> Option<Breach> {
        for b in Breach::ALL {
            let i = b.index();
            if now.contains(&b) {
                self.since[i].get_or_insert(t_s);
            } else {
                self.since[i] = None;
            }
        }
        Breach::ALL
            .into_iter()
            .find(|b| self.since[b.index()].is_some_and(|t0| t_s - t0 >= PERSIST_S))
    }

    /// Forget everything (a go-around, a new approach).
    pub fn reset(&mut self) {
        self.since = [None; 7];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GP: f64 = 318.0;

    fn sample(height: f64, ias: f64, target: f64, vvi: f64, above: f64) -> Sample {
        Sample {
            height_aal_ft: height,
            dist_to_aim_nm: height / GP,
            ias_kts: ias,
            gs_kts: ias,
            target_kts: target,
            vvi_fpm: Some(vvi),
            above_path_ft: above,
            path_ft_per_nm: GP,
            gear_down: true,
            landing_flap: true,
        }
    }

    /// Run a recorded final through the watch: `(t, height, ias, vvi, above,
    /// flap)` rows; the first breach to hold.
    fn fly(gate: GateKind, target: f64, rows: &[(f64, f64, f64, f64, f64, bool)]) -> Option<(Breach, f64)> {
        let mut w = StableWatch::default();
        for &(t, h, ias, vvi, above, flap) in rows {
            let mut s = sample(h, ias, target, vvi, above);
            s.landing_flap = flap;
            if let Some(b) = w.step(t, &breaches(gate, &s)) {
                return Some((b, h));
            }
        }
        None
    }

    #[test]
    fn the_egnx_final_at_255_kt_clean_goes_around_at_the_gate() {
        // Sortie 1790761044-full_auto, the first EGNX 09 approach (iMac
        // 2026-09-30): height above the runway, IAS, VVI and height above
        // the path from the autoland ticks, the flaps held up by their
        // placards the whole way (so never selected: flap 0 at 250 kt).
        let rows = [
            (0.0, 1041.0, 254.5, -1390.0, 5.0, false),
            (2.7, 995.0, 254.5, -1419.0, 12.0, false),
            (5.4, 895.0, 254.1, -1408.0, 9.0, false),
            (8.1, 857.0, 254.2, -1404.0, 8.0, false),
            (10.8, 819.0, 254.3, -1400.0, 7.0, false),
            (13.5, 782.0, 254.3, -1397.0, 6.0, false),
        ];
        let (b, h) = fly(GateKind::Instrument, 145.0, &rows).expect("unstable");
        assert_eq!(b, Breach::Flap, "no landing flap is called first");
        assert!(h > 850.0, "with height in hand: {h}");
        // Every failure it had, and the landing flap selected would not
        // have made it stable: fast and sinking on their own.
        let s = sample(895.0, 254.1, 145.0, -1408.0, 9.0);
        assert_eq!(breaches(GateKind::Instrument, &s), vec![Breach::Fast, Breach::Sink]);
    }

    #[test]
    fn the_good_finals_of_2026_09_30_stay_stable() {
        // 737, EGNX 09 second approach (the same sortie, landed at 230 fpm),
        // its autoland ticks from 1,012 ft. The flap was commanded full at
        // 3.0 nm (990 ft) and read 1.0 by 937; the schedule now selects it
        // a mile before the gate (`autoland::flap_schedule`), at 150 kt
        // well inside its 162 kt placard, so it is selected throughout.
        let second = [
            (0.0, 1012.0, 149.7, -818.0, -11.0, true),
            (2.7, 967.0, 150.4, -764.0, -9.0, true),
            (5.4, 923.0, 154.6, -518.0, 9.0, true),
            (8.1, 879.0, 156.1, -592.0, 34.0, true),
            (10.8, 842.0, 153.3, -692.0, 42.0, true),
            (13.5, 798.0, 152.1, -456.0, 69.0, true),
            (16.2, 765.0, 150.3, -708.0, 78.0, true),
            (18.9, 717.0, 150.2, -986.0, 71.0, true),
            (21.6, 686.0, 150.4, -1046.0, 64.0, true),
            (24.3, 656.0, 150.7, -1046.0, 57.0, true),
            (27.0, 601.0, 150.9, -982.0, 47.0, true),
            (29.7, 552.0, 150.9, -939.0, 40.0, true),
            (32.4, 502.0, 150.8, -916.0, 34.0, true),
            (35.1, 479.0, 150.7, -909.0, 31.0, true),
            (37.8, 374.0, 150.4, -884.0, 21.0, true),
            (40.5, 305.0, 150.1, -863.0, 15.0, true),
            (43.2, 236.0, 149.8, -850.0, 11.0, true),
            (45.9, 189.0, 149.6, -838.0, 8.0, true),
        ];
        assert_eq!(fly(GateKind::Instrument, 145.0, &second), None);
        // 737 EGBB 33 (sortie 1790759458-full_auto): 169 at 1,008 ft with
        // the speed still coming off, 151 by 500.
        let egbb = [
            (0.0, 1008.0, 168.9, -956.0, -14.0, true),
            (2.0, 954.0, 167.1, -919.0, -15.0, true),
            (4.0, 908.0, 165.0, -776.0, -11.0, true),
            (6.0, 865.0, 162.4, -725.0, -2.0, true),
            (8.0, 822.0, 159.5, -699.0, 7.0, true),
            (10.0, 770.0, 154.7, -633.0, 23.0, true),
            (12.0, 724.0, 150.8, -593.0, 41.0, true),
            (14.0, 657.0, 150.2, -894.0, 41.0, true),
            (16.0, 573.0, 150.6, -932.0, 29.0, true),
            (18.0, 496.0, 150.6, -870.0, 22.0, true),
            (20.0, 395.0, 150.4, -859.0, 15.0, true),
        ];
        assert_eq!(fly(GateKind::Instrument, 145.0, &egbb), None);
        // SR22 circuit final (sortie 1790732424-full_auto): handed over at
        // 354 ft 75 ft under the path, 69.6 to 79.5 kt against 75.
        let sr22 = [
            (0.0, 354.0, 71.1, -461.0, -75.0, true),
            (1.0, 345.0, 70.5, -372.0, -75.0, true),
            (2.0, 339.0, 69.9, -234.0, -71.0, true),
            (3.0, 337.0, 69.6, -114.0, -64.0, true),
            (4.0, 337.0, 69.9, -3.0, -52.0, true),
            (5.0, 340.0, 70.7, 83.0, -38.0, true),
            (6.0, 341.0, 73.4, -29.0, -14.0, true),
            (8.0, 319.0, 79.2, -263.0, 12.0, true),
            (10.0, 285.0, 78.6, -464.0, 15.0, true),
            (12.0, 240.0, 76.3, -545.0, 5.0, true),
            (14.0, 196.0, 74.4, -514.0, -5.0, true),
        ];
        assert_eq!(fly(GateKind::Visual, 75.0, &sr22), None);
    }

    #[test]
    fn a_gust_through_the_band_is_not_a_go_around() {
        let rows = [
            (0.0, 700.0, 150.0, -800.0, 0.0, true),
            (1.0, 680.0, 172.0, -800.0, 0.0, true),
            (2.0, 660.0, 150.0, -800.0, 0.0, true),
            (3.0, 640.0, 172.0, -800.0, 0.0, true),
            (4.0, 620.0, 151.0, -800.0, 0.0, true),
        ];
        assert_eq!(fly(GateKind::Instrument, 145.0, &rows), None);
    }

    #[test]
    fn the_speed_allowance_closes_at_five_hundred_feet() {
        let over = |h| breaches(GateKind::Instrument, &sample(h, 168.0, 145.0, -800.0, 0.0));
        assert!(over(700.0).is_empty(), "configured and bleeding off above 500");
        assert_eq!(over(480.0), vec![Breach::Fast]);
        // A visual gate has no allowance: judged from 500 only.
        assert!(breaches(GateKind::Visual, &sample(700.0, 200.0, 145.0, -800.0, 0.0)).is_empty());
        assert_eq!(
            breaches(GateKind::Visual, &sample(480.0, 168.0, 145.0, -800.0, 0.0)),
            vec![Breach::Fast]
        );
    }

    #[test]
    fn the_sink_limit_is_the_paths_at_the_target_speed() {
        assert_eq!(sink_limit_fpm(75.0, 75.0, 75.0, GP), 1000.0);
        // The 737 on speed: 145 kt over the ground makes 769 on the path.
        assert!((sink_limit_fpm(145.0, 150.0, 150.0, GP) - 1169.0).abs() < 1.0);
        // Fast is not a licence: 255 kt still gets the 145 kt path's limit.
        assert!((sink_limit_fpm(145.0, 255.0, 255.0, GP) - 1169.0).abs() < 1.0);
        // A tailwind or a steep visual segment does demand more.
        assert!(sink_limit_fpm(145.0, 150.0, 175.0, GP) > 1250.0);
        assert!(sink_limit_fpm(87.0, 87.0, 87.0, 4.5_f64.to_radians().tan() * 6076.1) >= 1000.0);
    }

    #[test]
    fn the_path_tolerance_is_two_dots() {
        assert!((path_tolerance_ft(3.0) - 223.0).abs() < 2.0);
        assert_eq!(path_tolerance_ft(0.3), PATH_TOL_MIN_FT);
        // A dive 330 ft over the path at 1,000 ft (the high setups of
        // 2026-09-29, 2,200 to 2,900 fpm down) is not stable.
        let s = sample(990.0, 98.0, 75.0, -2205.0, 330.0);
        assert_eq!(breaches(GateKind::Instrument, &s), vec![Breach::Sink, Breach::High]);
    }

    #[test]
    fn gear_and_flap_are_the_configuration() {
        let mut s = sample(900.0, 150.0, 145.0, -800.0, 0.0);
        s.gear_down = false;
        s.landing_flap = false;
        assert_eq!(breaches(GateKind::Instrument, &s), vec![Breach::Gear, Breach::Flap]);
        assert!(breaches(GateKind::Instrument, &sample(1200.0, 250.0, 145.0, -2000.0, 900.0)).is_empty(), "above the gate");
        assert!(breaches(GateKind::Instrument, &sample(150.0, 250.0, 145.0, -2000.0, 900.0)).is_empty(), "the flare's");
    }
}
