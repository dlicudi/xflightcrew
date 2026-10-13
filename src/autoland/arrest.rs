//! The arrest's lateral and the landing limits: hands, bank targets, flap
//! schedules and the established final path.

use super::*;

/// A Cat I ILS's decision height, ft: a coupled final is the autopilot's
/// down to it unless the profile declares its own (`ils_decouple_agl_ft`).
pub const DECISION_HEIGHT_FT: f64 = 200.0;
/// A final on our own heading and path (visual, non-precision), ft: an
/// LNAV MDA is 300 to 500 ft over the field, and from 300 ft a 140 kt jet
/// still has 24 s to the wheels, three times what the bank needs to close
/// the 18 m a whole-degree heading leaves (`final_heading_tests`).
pub const VISUAL_MINIMUMS_FT: f64 = 300.0;

/// Minimums for this final: from them it is hand flown to the wheels.
/// Pure.
pub fn minimums_ft(coupled: bool, decouple_agl_ft: Option<f64>) -> f64 {
    if coupled {
        decouple_agl_ft.filter(|h| h.is_finite() && *h > 0.0).unwrap_or(DECISION_HEIGHT_FT)
    } else {
        VISUAL_MINIMUMS_FT
    }
}

/// Who has the servos this tick. The hand-flown segment flies the channels
/// with the autopilot's servos OFF (the pitch servo writes the yoke back over
/// the channel otherwise; LFPG 09R, 2026-09-22), takes them at minimums
/// (2026-10-12; it was the roundout gate) and hands them back only the tick
/// the segment ends in a go-around
/// (a balk, the missed approach, a pattern): a balloon back over the band
/// is still the arrest ([`hold_roundout`]; LEMG 12, 2026-09-28, handed back
/// at 50.5 ft and dived to -814 fpm). The rollout latch never reaches this decision:
/// it disconnects on its own and finds the servos already off.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArrestHands {
    /// The autopilot flies; nothing to do.
    Off,
    /// This tick: servos off, the channels take over.
    Take,
    /// The arrest has the servos and keeps flying the channels.
    Keep,
    /// This tick: servos on, modes re-engaged, the channels expire.
    Return,
}

pub fn arrest_hands_step(taken: bool, arresting: bool) -> ArrestHands {
    match (taken, arresting) {
        (false, false) => ArrestHands::Off,
        (false, true) => ArrestHands::Take,
        (true, true) => ArrestHands::Keep,
        (true, false) => ArrestHands::Return,
    }
}
/// May the roundout own the descent from HERE? Only where a
/// [`ROUNDOUT_VS_FPM`] descent still REACHES the pavement. At or past the
/// threshold, always — there the unsigned glidepath distance grows again and
/// alt_err stops meaning anything, which is the arithmetic the roundout
/// exists to pre-empt. Short of it, the sink from this height must cover the
/// metres to the threshold: 40 ft at 0.2 nm closes (a normal crossing), and
/// 30 ft at 0.3 nm does not — that aircraft is dragged low, and a commanded
/// descent grounds it ~190 m into the approach lights where the level-hold
/// would have let the path come down to it (audit, 2026-08-02). A failed
/// (zero) groundspeed read denies the roundout, which falls back to the
/// level-hold: short of the runway, level is always the safe reading.
pub(super) fn roundout_reaches_pavement(along_nm: f64, radio_alt_ft: f64, gs_kts: f64) -> bool {
    if along_nm <= 0.0 {
        return true;
    }
    let sink_s = radio_alt_ft / (ROUNDOUT_VS_FPM.abs() / 60.0);
    let reach_m = gs_kts * 0.514_444 * sink_s;
    along_nm * M_PER_NM <= reach_m
}
/// ...and only within this much cross-track of the centreline (~37 m — about a
/// runway half-width plus margin). Beyond it the aircraft is over grass, and
/// touching down there is how the LXGB 09 rollout ended in wreckage
/// (2026-07-22). A balked landing straight ahead is always the better trade.
pub(super) const FLARE_MAX_XTK_NM: f64 = 0.02;

/// How far inside the pavement's edge a landing's centreline offset must stay,
/// metres: a jet's main gear (the 737's is 5.7 m across, 2.9 m either side)
/// plus a margin for the arrest's last closure.
pub(super) const LANDING_EDGE_MARGIN_M: f64 = 7.5;
/// ...and how close to the edge the flare may still be at the flare gate: the
/// gear on the pavement with a metre to spare.
pub(super) const FLARE_EDGE_MARGIN_M: f64 = 4.0;

/// The most centreline offset a landing may carry to the wheels on a runway
/// `width_m` wide, metres: the stabilised gates below the terrain floor and
/// the low gate judge against it. `FLARE_MAX_XTK_NM`'s 37 m is a 60 m strip's
/// half-width plus margin, and on LEMG 13's 45 m it is 14.5 m past the edge:
/// the 737 flew the last 500 ft 30 to 50 m off, the gates never fired, and it
/// touched down 37 m right of the centreline, in the grass (sortie
/// 1790591256-full_auto, 2026-09-28). Unknown width keeps the old 37 m; a
/// narrow strip keeps at least a quarter of its width.
pub fn landing_xtk_limit_m(width_m: Option<f64>) -> f64 {
    let legacy = FLARE_MAX_XTK_NM * M_PER_NM;
    match width_m.filter(|w| w.is_finite() && *w > 1.0) {
        Some(w) => ((w / 2.0 - LANDING_EDGE_MARGIN_M).max(w / 4.0)).min(legacy),
        None => legacy,
    }
}

/// The offset the FLARE gate balks beyond, metres: the gear still on the
/// pavement ([`FLARE_EDGE_MARGIN_M`] inside the edge), never tighter than the
/// landing limit, never looser than the old 37 m.
pub fn flare_xtk_limit_m(width_m: Option<f64>) -> f64 {
    let legacy = FLARE_MAX_XTK_NM * M_PER_NM;
    match width_m.filter(|w| w.is_finite() && *w > 1.0) {
        Some(w) => (w / 2.0 - FLARE_EDGE_MARGIN_M)
            .max(landing_xtk_limit_m(Some(w)))
            .min(legacy),
        None => legacy,
    }
}

/// Inside the FAP and within this of the runway's course, an approach that
/// needs repositioning goes around first instead of turning at once.
pub(super) const LATE_GO_AROUND_TRACK_DEG: f64 = 45.0;

/// Is a reposition this late a go-around? `along_nm` is the distance still
/// to the threshold along the approach (positive short of it). Pure.
pub fn late_go_around(along_nm: f64, track_off_deg: f64) -> bool {
    along_nm > 0.0 && along_nm < FAP_NM && track_off_deg.abs() < LATE_GO_AROUND_TRACK_DEG
}

/// Flap setting for the approach: half inside 6 nm, full a mile before the
/// stabilised gate ([`landing_flap_nm`], about 4 nm on a 3 degree path), but
/// full only once established ON the glidepath. While holding level BELOW the
/// path waiting to capture (`alt_err` well negative), full flaps are an energy
/// trap: the LERL 172 approach dragged along at 50 to 57 kt against a 65-kt
/// target with the throttle governor already at its ceiling, and arrived at
/// the flare with nothing left. Half flaps keep the drag survivable until the
/// path is captured.
///
/// Full used to wait for 3 nm, which on a 3 degree path IS the 1,000 ft gate:
/// every final on file reached the gate with half flap out and the speed
/// still on the half flap's floor (the 737 at 169 kt on EGBB 33, sortie
/// 1790759458-full_auto, 2026-09-30), so no final could be configured by the
/// gate (`stable_approach`, #199).
pub fn flap_schedule(dist_nm: f64, alt_err_ft: f64) -> f64 {
    if dist_nm < landing_flap_nm() && alt_err_ft > -150.0 {
        1.0
    } else if dist_nm < 6.0 {
        0.5
    } else {
        0.0
    }
}

/// Flap schedule for the leg being flown. A normal downwind inside the
/// configuration area uses half flap, but a distant outbound reversal is a
/// reposition rather than a traffic-pattern leg and remains clean (#211).
pub fn approach_leg_flap_schedule(dist_nm: f64, alt_err_ft: f64, outbound: bool) -> f64 {
    if outbound {
        if dist_nm < 6.0 {
            0.5
        } else {
            0.0
        }
    } else {
        flap_schedule(dist_nm, alt_err_ft)
    }
}

/// The landing flap is selected this far ahead of the stabilised gate, nm:
/// time for it to run and for the speed to come back to its target before
/// the gate judges both.
pub(super) const LANDING_FLAP_LEAD_NM: f64 = 1.0;

/// Where the approach selects its landing flap, nm from the threshold: a
/// mile before the instrument gate on this approach's path (3.98 nm at 3
/// degrees). Never beyond the half-flap point.
pub fn landing_flap_nm() -> f64 {
    let gate_nm = crate::stable_approach::GateKind::Instrument.gate_ft() / gp_ft_per_nm()
        - TOUCHDOWN_AIM_NM;
    (gate_nm + LANDING_FLAP_LEAD_NM).clamp(3.0, 5.5)
}

/// Inside this of the centreline, nm...
pub const ESTABLISHED_PATH_XTK_NM: f64 = 0.25;
/// ...closing or opening it no faster than this, nm/min (10 degrees at
/// 200 kt): established on the final path.
pub const ESTABLISHED_PATH_XTK_RATE_NM_MIN: f64 = 0.6;

/// Established on the final's path beyond the FAP (see `guidance_on`):
/// Intercept by phase only because the lateral still steers for the FAP,
/// on the approach side, on the centreline and tracking it, on or above the
/// path. Pure.
pub fn established_on_final_path(
    phase: ApproachPhase,
    balk: Option<&'static str>,
    along_nm: f64,
    xtk_nm: f64,
    xtk_rate_nm_min: f64,
    alt_err_ft: f64,
) -> bool {
    phase == ApproachPhase::Intercept
        && balk.is_none()
        && along_nm > FAP_NM
        && xtk_nm.abs() <= ESTABLISHED_PATH_XTK_NM
        && xtk_rate_nm_min.abs() <= ESTABLISHED_PATH_XTK_RATE_NM_MIN
        && alt_err_ft > FLOOR_BELOW_PATH_FT
}

/// The energy plan's geometry for one tick: the route to the gate, nm, and
/// the path's share of it. A pattern (`pattern`: its track to go and the
/// distance from the threshold it ends at) flies level to its end and the
/// path from there; a straight-in on or above the path is on it; one below
/// it flies level until the path comes down to it. Pure.
pub fn energy_geometry(
    pattern: Option<(f64, f64)>,
    dist_nm: f64,
    above_path_ft: f64,
    path_ft_per_nm: f64,
    gate_nm: f64,
) -> (f64, f64) {
    match pattern {
        Some((to_go_nm, end_nm)) => {
            let path = (end_nm - gate_nm).max(0.0);
            (to_go_nm.max(0.0) + path, path)
        }
        None => {
            let s = (dist_nm - gate_nm).max(0.0);
            if above_path_ft < -50.0 {
                // The path reaches our height (-above) / slope further in.
                let meet_nm = dist_nm - (-above_path_ft) / path_ft_per_nm.max(1.0);
                (s, (meet_nm - gate_nm).clamp(0.0, s))
            } else {
                (s, s)
            }
        }
    }
}

/// The flap the plan's configuration asks for: its own, except the landing
/// flap below the path (full flap held level under the path is the LERL
/// energy trap: [`flap_schedule`]). Pure.
pub fn plan_flap_wanted(plan_flap: f64, alt_err_ft: f64) -> f64 {
    if plan_flap >= LANDING_FLAP_RATIO - 1e-6 && alt_err_ft <= -150.0 {
        0.5f64.min(plan_flap)
    } else {
        plan_flap
    }
}
