//! The charts flown: the missed approach point, the dial target, departure
//! minimums, the VS action, STAR windows, ceilings and speeds, and the leg
//! capture geometry.

use super::*;

/// Autoland flies the visual segment from a MAP. Off until it is proven: at
/// LSZS (2026-09-24) the steep path in 18 kt of mountain wind porpoised
/// +/-1,500 fpm, levelled at 238 ft for no reason yet found, and climbed away
/// over the runway. The MAP is where a crew goes visual; the pilot takes it
/// there, talked down (`map_handover_path`).
pub const MAP_AUTOLAND: bool = false;

/// The talk-down's path for a hand-over at a MAP: from where the aircraft is
/// to 50 ft over the threshold, so the first call is "on glidepath" and every
/// later one measures the segment actually being flown. Slope clamped to
/// 3..6 degrees. Pure.
pub fn map_handover_path(
    rwy: &crate::runway::Runway,
    lat: f64,
    lon: f64,
    alt_ft: f64,
) -> crate::approach_guide::ApproachPath {
    let dist_nm = crate::nav::haversine_nm(lat, lon, rwy.thr_lat, rwy.thr_lon).max(0.2);
    let height = alt_ft - (rwy.elev_ft + crate::approach_guide::THRESHOLD_CROSSING_FT);
    let (lo, hi) = (6076.12 * 3f64.to_radians().tan(), 6076.12 * 6f64.to_radians().tan());
    crate::approach_guide::ApproachPath {
        icao: rwy.icao.clone(),
        runway_id: rwy.id.clone(),
        thr_lat: rwy.thr_lat,
        thr_lon: rwy.thr_lon,
        course_deg: rwy.course_deg,
        elev_ft: rwy.elev_ft,
        ft_per_nm: (height / dist_nm).clamp(lo, hi),
    }
}

/// A missed approach point SHORT of the runway: a visual segment follows
/// (LSZS RNAV 03). A MAP coded on the runway fix itself is the landing, and
/// autoland's: LIPE's ILS 12 codes its MAP on RW12, the aircraft (high) never
/// captured the FAF, the follower reached RW12 0.8 nm out and handed the
/// pilot a "visual segment" onto an ILS runway (2026-09-25).
pub fn map_short_of_runway(leg: &ProcLeg) -> bool {
    leg.is_map && !leg.fix_ident.to_ascii_uppercase().starts_with("RW")
}

/// On the leg to a missed approach point, the published path: the altitude it
/// wants at `dist_nm` from the MAP and the descent rate that stays on it at
/// this groundspeed (fpm, negative). A P law cannot track a ramp; the rate is
/// fed forward. `None` off such a leg. Pure.
pub fn map_path(leg: &ProcLeg, dist_nm: Option<f64>, gs_kts: f64) -> Option<(f64, f64)> {
    if !leg.is_map {
        return None;
    }
    let t = leg.vpa_deg?.to_radians().tan();
    let map_ft = leg.alt1_ft?;
    let d = dist_nm?;
    Some((map_ft + d * 6076.12 * t, -gs_kts.max(0.0) * 6076.12 / 60.0 * t))
}

/// What goes on the ALTITUDE DIAL: the level this climb or descent ends at,
/// set once, the way a pilot sets the preselect and flies the rate. The
/// follower used to write its paced target (`want_alt`) every tick: on a
/// continuous descent that is a dial walking just under the aircraft, the
/// autopilot captured it, levelled, the dial stepped 50 ft lower, and the
/// descent went down a staircase 60 to 120 ft above its own path (LIPR 30,
/// 2026-09-26, handed to autoland 120 ft high at 0.8 nm). Descending, the
/// dial is the leg's constraint (`carry`), never under a terrain floor or a
/// pilot's level; climbing, the higher of the target and the constraint;
/// level, the level held; on a published path, the bottom of it. Whole
/// hundreds, rounded UP so a floor or a minimum is never under-set. Pure.
///
/// `path_fpm` is the target's own rate (`Vertical::target_fpm`, zero when
/// the target is a level or not the plan's): while the target is WALKING
/// DOWN its path the descent is not over, whatever the band says. An
/// aircraft that has caught its paced target sits inside the 100 ft band
/// for most of a good descent, and the band used to put the dial at the
/// target there, a hundred feet under the aircraft: the 737 captured it,
/// held level 10 to 15 s, the dial went back to the constraint, VS was
/// pressed again, five times from FL190 to 12,700 at about 1,450 fpm for
/// a planned 1,800 (LEGR-LXGB, 2026-10-06, sortie 1791275171-full_auto,
/// `vs_mode` `leave` with the dial at 18,200, 16,600, 14,800, 13,200 and
/// 12,900, each the paced target rounded).
pub fn dial_target_ft(
    want_ft: f64,
    alt_ft: f64,
    carry_ft: f64,
    held_ft: Option<f64>,
    path_bottom_ft: Option<f64>,
    path_fpm: f64,
) -> f64 {
    const BAND_FT: f64 = 100.0;
    let up = |v: f64| (v / 100.0).ceil() * 100.0;
    if let Some(b) = path_bottom_ft {
        return up(b.max(held_ft.unwrap_or(f64::MIN)));
    }
    let walking_down = path_fpm < 0.0 && carry_ft < want_ft - BAND_FT;
    if want_ft < alt_ft - BAND_FT || walking_down {
        up(carry_ft.max(held_ft.unwrap_or(f64::MIN)))
    } else if want_ft > alt_ft + BAND_FT {
        up(want_ft.max(carry_ft))
    } else {
        (want_ft / 100.0).round() * 100.0
    }
}

/// Split a departure's minimums into the ones this plan can fly and the ones
/// above its cruise. A minimum over the planned cruise is a level the
/// follower was never going to climb to (the plan owns the target), and
/// judging the aircraft against it handed a 172 back to its pilot at 5,000
/// ft, three miles from D121Z, for not climbing to ANC8A's 9,500
/// (LIPR-LIBP 2026-09-26). Pure.
pub fn minimums_within_cruise(
    mins: Vec<crate::nav::ClimbMinimum>,
    cruise_ft: f64,
) -> (Vec<crate::nav::ClimbMinimum>, Vec<crate::nav::ClimbMinimum>) {
    mins.into_iter().partition(|m| m.min_ft <= cruise_ft + MINIMUM_MET_TOL_FT)
}

/// Within this of the clean floor, the approach's flaps come out on speed.
pub(super) const FLAPS_ON_SPEED_MARGIN_KTS: f64 = 15.0;

/// A commanded rate as the VS wheel sets it: the nearest 100 fpm. Pure.
pub fn vs_dial_fpm(vs: f64) -> f64 {
    (vs / 100.0).round() * 100.0
}

/// A departure minimum is met this close under it, feet.
pub const MINIMUM_MET_TOL_FT: f64 = 100.0;

/// The first departure minimum still ahead of the aircraft and not yet met:
/// (leg index of its fix, minimum ft). Legs before `idx` are behind us.
/// Stateless, so it flips on the tolerance's last foot: the loop uses
/// [`DepartureMinimums`], which remembers; this is the old rule the #153
/// replay is judged against.
#[cfg(test)]
pub fn unmet_minimum(
    minimums: &[crate::nav::ClimbMinimum],
    legs: &[ProcLeg],
    idx: usize,
    alt_ft: f64,
) -> Option<(usize, f64)> {
    legs.iter().enumerate().skip(idx).find_map(|(i, leg)| {
        minimums
            .iter()
            .find(|m| m.fix.eq_ignore_ascii_case(&leg.fix_ident))
            // Met within the autopilot's own hold: levelled AT the minimum
            // is meeting it. Compared exactly, a 5,000 ft minimum at a 5,000
            // ft cruise flipped met/unmet on the altimeter's last foot and
            // the target with it, 74 to 108 knots and back (LIPE-LIPK,
            // 2026-09-25).
            .filter(|m| alt_ft < m.min_ft - MINIMUM_MET_TOL_FT)
            .map(|m| (i, m.min_ft))
    })
}

/// The departure minimums with MEMORY: a minimum once reached stays met, and
/// every minimum at a fix still ahead is a FLOOR on the target, met or not,
/// until its fix is behind.
///
/// EGKK LAM5P (2026-09-28, sortie 1790601822-full_auto, #153): D156J's
/// "at or above 6,000" was a level to climb TO only while unmet, and met
/// within [`MINIMUM_MET_TOL_FT`], so at 5,900 ft the target fell back to the
/// plan's navlog 5,000 (ACORN, D156O) and the speed from Vy's 210 to the
/// cruise's 250; the autopilot dived for the 5,000 dial, the minimum came
/// unmet under 5,900 and the climb and the 210 came back. Nineteen flips in
/// 207 s between 5,742 and 5,959 ft, at -1,822 to +1,145 fpm,
/// the yoke visibly jerking. A floor does not flip: met or not, the level
/// is held until the fix is passed, and "met" (Vy released) happens once.
#[derive(Debug, Default, Clone)]
pub struct DepartureMinimums {
    pub(super) met: std::collections::HashSet<String>,
}

impl DepartureMinimums {
    /// The first minimum ahead not yet met, after marking met every one
    /// ahead whose level has been reached (`alt_ft` within the tolerance).
    /// Keyed by fix, so a splice of the arrival never forgets one.
    pub fn unmet(
        &mut self,
        minimums: &[crate::nav::ClimbMinimum],
        legs: &[ProcLeg],
        idx: usize,
        alt_ft: f64,
    ) -> Option<(usize, f64)> {
        for leg in legs.iter().skip(idx) {
            if let Some(m) = minimums.iter().find(|m| m.fix.eq_ignore_ascii_case(&leg.fix_ident)) {
                if alt_ft >= m.min_ft - MINIMUM_MET_TOL_FT {
                    self.met.insert(m.fix.to_ascii_uppercase());
                }
            }
        }
        legs.iter().enumerate().skip(idx).find_map(|(i, leg)| {
            minimums
                .iter()
                .find(|m| m.fix.eq_ignore_ascii_case(&leg.fix_ident))
                .filter(|m| !self.met.contains(&m.fix.to_ascii_uppercase()))
                .map(|m| (i, m.min_ft))
        })
    }

    /// The highest minimum at a fix still ahead (leg index, feet), met or
    /// not: the level the target may not go under until that fix is flown.
    pub fn floor(
        minimums: &[crate::nav::ClimbMinimum],
        legs: &[ProcLeg],
        idx: usize,
    ) -> Option<(usize, f64)> {
        legs.iter()
            .enumerate()
            .skip(idx)
            .filter_map(|(i, leg)| {
                minimums
                    .iter()
                    .find(|m| m.fix.eq_ignore_ascii_case(&leg.fix_ident))
                    .map(|m| (i, m.min_ft))
            })
            .max_by(|a, b| a.1.total_cmp(&b.1))
    }
}

/// The follower's altitude once its departure owners have spoken, in their
/// order: the plan's (`plan_ft`, the vertical law with the pilot and the
/// ground already applied), raised to a departure minimum still ahead
/// (`minimum_ft`, [`DepartureMinimums::floor`]; never on a published path),
/// then capped at an IFR clearance's altitude until departure clears higher
/// (`cap_at_clearance`: a pilot's own altitude and the terrain floor still
/// outrank the cap). A minimum above the clearance is climbed to only once
/// the check-in has released the cap: the dial and the target are one number
/// and a controller's altitude is never climbed through on a chart's
/// account. Pure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepartureTarget {
    /// The altitude to command.
    pub want_ft: f64,
    /// A departure minimum raised the plan's target.
    pub raised_by_minimum: bool,
    /// The clearance holds the target under a minimum still ahead.
    pub minimum_capped: bool,
    /// The clearance lowered the target (plan or minimum).
    pub cap_limited: bool,
}

pub fn departure_target_ft(
    plan_ft: f64,
    minimum_ft: Option<f64>,
    ceiling_ft: Option<f64>,
    pilot_held: bool,
    terrain_floor_ft: Option<f64>,
) -> DepartureTarget {
    let raised = minimum_ft.filter(|m| *m > plan_ft);
    let uncapped = raised.unwrap_or(plan_ft);
    let want_ft = cap_at_clearance(uncapped, ceiling_ft, pilot_held, terrain_floor_ft);
    DepartureTarget {
        want_ft,
        raised_by_minimum: raised.is_some(),
        minimum_capped: raised.is_some() && want_ft < uncapped - 0.5,
        cap_limited: want_ft < uncapped - 0.5,
    }
}

/// What the follower does with the VS wheel this tick, read from the modes
/// first: [`VsAction::Write`] only while VS mode reads ON. A VS dial written
/// under ALT HOLD reads back 0 (`write_unconfirmed vertical_velocity want
/// 400 actual 0`, EGKK 2026-09-28 with `ap_state` 16386), and the failed
/// write threw the rest of the tick away.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VsAction {
    /// VS mode is on: set the wheel.
    Write,
    /// The autopilot's own capture or hold has the level we want: leave it.
    Leave,
    /// A movement the holding mode will not make: engage VS, then (mode
    /// read again) set the wheel.
    Engage,
}

/// Feet from the dial inside which an ALT HOLD (or capture) on OUR dial is
/// left to finish the level-off: the 737 began capturing 6,000 at 5,817 ft
/// and 962 fpm, 183 ft out (EGKK 2026-09-28).
pub const ALT_HOLD_LEAVE_FT: f64 = 300.0;

/// Pure: `vs_on` the VS mode's status, `alt_hold_on` the altitude hold
/// ENGAGED (`altitude_hold_status` 2, the capture included), `rate_fpm` the
/// rate being flown.
pub fn vs_action(
    vs_on: bool,
    alt_hold_on: bool,
    alt_ft: f64,
    want_ft: f64,
    dial_ft: f64,
    vs_cmd_fpm: f64,
    rate_fpm: f64,
) -> VsAction {
    if vs_on {
        return VsAction::Write;
    }
    let lead = if rate_fpm.abs() > vs_cmd_fpm.abs() { rate_fpm } else { vs_cmd_fpm };
    if vs_mode_capturing(alt_ft, want_ft, dial_ft, lead) {
        return VsAction::Leave;
    }
    // A hold on OUR dial, at it or still closing on it: the capture's own.
    // A hold parked short of the dial (not closing) is not a level-off.
    let off = dial_ft - alt_ft;
    let closing = off * rate_fpm > 0.0 && off.abs() <= ALT_HOLD_LEAVE_FT;
    if alt_hold_on && (dial_ft - want_ft).abs() <= 100.0 && (off.abs() <= 100.0 || closing) {
        return VsAction::Leave;
    }
    VsAction::Engage
}

/// Along-route distance, nm, from the aircraft to leg `target`'s fix: to the
/// current leg's fix, then fix to fix.
pub fn distance_to_leg_nm(lat: f64, lon: f64, legs: &[ProcLeg], idx: usize, target: usize) -> f64 {
    let mut d = 0.0;
    let mut from = (lat, lon);
    for leg in legs.iter().take(target + 1).skip(idx) {
        if let Some(f) = &leg.fix {
            d += crate::nav::haversine_nm(from.0, from.1, f.lat, f.lon);
            from = (f.lat, f.lon);
        }
    }
    d
}

/// What a departure minimum's watch says this tick.
#[derive(Debug, Clone, PartialEq)]
pub enum MinimumWatch {
    OnTrack,
    /// Predicted to cross short: said once, the climb is already at its best.
    Short { predicted_ft: f64 },
    /// Will not be made: the aircraft goes back to the pilot, with the numbers.
    Unable { predicted_ft: f64 },
}

/// Altitude samples (about half a second apart) before the climb rate is a
/// measurement the minimum watch may judge on.
pub(super) const MIN_RATE_SAMPLES: u32 = 10;
/// Below this the aircraft is not climbing, and a minimum it will miss goes
/// back to the pilot.
pub(super) const NOT_CLIMBING_FPM: f64 = 200.0;

/// Feet short of a minimum that is still only a warning...
pub(super) const MIN_WARN_FT: f64 = 100.0;
/// ...and short by this, within [`MIN_UNABLE_NM`] of the fix, is unable.
pub(super) const MIN_UNABLE_FT: f64 = 300.0;
pub(super) const MIN_UNABLE_NM: f64 = 3.0;

/// Judge one minimum from the present climb: altitude now, rate (fpm),
/// groundspeed and the distance to the fix. Pure.
pub fn watch_minimum(alt_ft: f64, vs_fpm: f64, gs_kts: f64, dist_nm: f64, min_ft: f64) -> MinimumWatch {
    let predicted_ft = alt_ft + vs_fpm.max(0.0) * dist_nm / gs_kts.max(40.0) * 60.0;
    let short = min_ft - predicted_ft;
    if short >= MIN_UNABLE_FT && dist_nm <= MIN_UNABLE_NM {
        MinimumWatch::Unable { predicted_ft }
    } else if short >= MIN_WARN_FT {
        MinimumWatch::Short { predicted_ft }
    } else {
        MinimumWatch::OnTrack
    }
}

/// The arrival restriction published for this leg's fix, if any.
pub(super) fn restriction_at<'a>(
    cons: &'a [crate::nav::StarConstraint],
    leg: &ProcLeg,
) -> Option<&'a crate::nav::StarConstraint> {
    cons.iter().find(|c| c.fix.eq_ignore_ascii_case(&leg.fix_ident))
}

/// Clamp a plan's altitudes into its STAR's restrictions, at arm time. Each
/// restricted fix gets its window ([`crate::nav::StarConstraint::clamp_ft`]);
/// past the first restricted fix the profile only goes DOWN unless a fix's
/// own minimum says otherwise, so SimBrief's 8,000 after a 7,000 "at or
/// below" is never a climb back. Returns how many legs it changed.
///
/// A window above the plan's cruise (the highest altitude planned before
/// the first restricted fix) is flown at the cruise, never climbed to
/// ([`windows_above_cruise`] says which): EGKK-EGBB (2026-09-28, sortie
/// 1790601822-full_auto) clamped HEME1B's "HEMEL at FL220" into an FL180
/// plan, the follower climbed for 22,000 and crossed HEMEL at 14,638, the
/// window unmade and unsaid (#156, #32).
pub fn apply_arrival_constraints(legs: &mut [ProcLeg], cons: &[crate::nav::StarConstraint]) -> usize {
    let cruise = plan_cruise_before_star(legs, cons);
    let mut changed = 0;
    let mut prev: Option<f64> = None;
    for leg in legs.iter_mut() {
        let Some(alt) = leg.alt1_ft else { continue };
        let own = restriction_at(cons, leg);
        let mut want = own.map_or(alt, |c| c.clamp_ft(alt));
        if let (Some(c), Some(_)) = (cruise, own) {
            want = want.min(c.max(alt));
        }
        if let Some(p) = prev {
            want = want.min(p);
            if let Some(min) = own.and_then(|c| c.min_ft) {
                want = want.max(min);
            }
        }
        if own.is_some() || prev.is_some() {
            prev = Some(want);
        }
        if (want - alt).abs() > 0.5 {
            leg.alt1_ft = Some(want);
            changed += 1;
        }
    }
    changed
}

/// The plan's cruise as the STAR meets it: the highest altitude planned on
/// the legs before the first restricted fix. `None` when the plan starts on
/// the STAR (nothing before it to call a cruise).
pub(super) fn plan_cruise_before_star(legs: &[ProcLeg], cons: &[crate::nav::StarConstraint]) -> Option<f64> {
    let first = legs.iter().position(|l| restriction_at(cons, l).is_some())?;
    legs[..first].iter().filter_map(|l| l.alt1_ft).reduce(f64::max)
}

/// A STAR window this far above the cruise is one the plan does not make.
pub(super) const ABOVE_CRUISE_SLACK_FT: f64 = 100.0;

/// The STAR's "at or above" windows over the plan's cruise
/// ([`plan_cruise_before_star`]): `(fix, window's minimum, cruise)`. The
/// follower flies them at the cruise ([`apply_arrival_constraints`]) and
/// says so once, as a crew briefing the arrival would. Pure.
pub fn windows_above_cruise(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    cruise_ft: f64,
) -> Vec<(String, f64, f64)> {
    let mut out = Vec::new();
    for leg in legs {
        if let Some(min) = restriction_at(cons, leg).and_then(|c| c.min_ft) {
            if min > cruise_ft + ABOVE_CRUISE_SLACK_FT && !out.iter().any(|(f, _, _): &(String, f64, f64)| *f == leg.fix_ident) {
                out.push((leg.fix_ident.clone(), min, cruise_ft));
            }
        }
    }
    out
}

/// A window missed by more than this is said (the card's slack, #32).
pub(super) const STAR_MISS_SLACK_FT: f64 = 300.0;

/// A STAR altitude window the airframe cannot make from here, at its best.
#[derive(Debug, Clone, PartialEq)]
pub struct WindowMiss {
    pub fix: String,
    /// Under an "at or above" (true) or over an "at or below".
    pub low: bool,
    /// The window's edge judged (a minimum over the cruise is the cruise).
    pub limit_ft: f64,
    pub predicted_ft: f64,
    pub by_ft: f64,
    pub dist_nm: f64,
}

impl WindowMiss {
    /// What a crew says: once, with the number.
    pub fn line(&self) -> String {
        format!(
            "We'll cross {} about {} feet {} its {}.",
            self.fix,
            crate::auto_status::commas(((self.by_ft / 50.0).round() * 50.0) as i64),
            if self.low { "under" } else { "above" },
            crate::flightplan::spoken_feet(self.limit_ft),
        )
    }
}

/// The next STAR altitude window ahead (leg `idx` on), judged from here:
/// climbing at `climb_fpm`, the climb being flown ([`window_climb_fpm`]),
/// to an "at or above" (never
/// above `cruise_ft`: a window over the cruise is flown at the cruise,
/// [`windows_above_cruise`]), descending at `descent_fpm` (the speedbrake's
/// rate) to an "at or below". `Some` when even that misses it by more than
/// [`STAR_MISS_SLACK_FT`]. #32's owed rule: a restriction that cannot be
/// made is said, not silently missed (EGKK-EGBB 2026-09-28: HEMEL crossed
/// at 14,638 against FL220, unsaid). Pure.
#[allow(clippy::too_many_arguments)]
pub fn star_window_miss(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    idx: usize,
    lat: f64,
    lon: f64,
    alt_ft: f64,
    gs_kts: f64,
    climb_fpm: f64,
    descent_fpm: f64,
    cruise_ft: Option<f64>,
) -> Option<WindowMiss> {
    let (k, c) = legs.iter().enumerate().skip(idx).find_map(|(k, l)| {
        restriction_at(cons, l)
            .filter(|c| c.min_ft.is_some() || c.max_ft.is_some())
            .map(|c| (k, c))
    })?;
    let dist_nm = distance_to_leg_nm(lat, lon, legs, idx, k);
    let minutes = dist_nm / gs_kts.max(60.0) * 60.0;
    let fix = legs[k].fix_ident.clone();
    if let Some(min) = c.min_ft.map(|m| cruise_ft.map_or(m, |cr| m.min(cr))) {
        if alt_ft < min {
            let predicted_ft = alt_ft + climb_fpm.max(0.0) * minutes;
            let by_ft = min - predicted_ft;
            if by_ft > STAR_MISS_SLACK_FT {
                return Some(WindowMiss { fix, low: true, limit_ft: min, predicted_ft, by_ft, dist_nm });
            }
        }
    }
    if let Some(max) = c.max_ft {
        if alt_ft > max {
            let predicted_ft = alt_ft - descent_fpm.max(0.0) * minutes;
            let by_ft = predicted_ft - max;
            if by_ft > STAR_MISS_SLACK_FT {
                return Some(WindowMiss { fix, low: false, limit_ft: max, predicted_ft, by_ft, dist_nm });
            }
        }
    }
    None
}

/// The climb a STAR "at or above" is judged on: the rate the follower is
/// commanding this tick (`vs_cmd`, after its speed protection), never the
/// airframe's book rate; `None` while an IFR clearance holds the target
/// down (`cap_limited`: the controller's level, judged once released).
/// EGKK-EGBB (sortie 1790624015-full_auto, 2026-09-28, #173): held level
/// at 6,000, the book's 2,000 fpm said "HEMEL about 350 feet under its
/// 18,000"; it was crossed at 14,587. Pure.
pub fn window_climb_fpm(vs_cmd: f64, cap_limited: bool) -> Option<f64> {
    (!cap_limited).then_some(vs_cmd.max(0.0))
}

/// What a crew says about a window over its cruise, once.
pub fn above_cruise_line(fix: &str, min_ft: f64, cruise_ft: f64) -> String {
    format!(
        "{fix}'s {} is above our cruise. We'll cross it at {}.",
        crate::flightplan::spoken_feet(min_ft),
        crate::flightplan::spoken_feet(cruise_ft),
    )
}

/// The highest altitude leg `idx`'s fix may be crossed at so that every
/// arrival "at or below" further on is still reachable at `ft_per_nm`
/// (the descent law's rate over the groundspeed), never under that fix's
/// own minimum. `None` with no ceiling ahead. Pacing each leg only to its
/// OWN fix left a restriction two fixes on to the last leg: FL070 at TARTN
/// with 12 nm from HAVEN is not a descent that starts at HAVEN. Geometric,
/// so a descent already made is never climbed back out of.
pub fn arrival_ceiling_ft(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    idx: usize,
    ft_per_nm: f64,
) -> Option<f64> {
    let from = legs.get(idx)?.fix.as_ref()?;
    let mut ceiling: Option<f64> = None;
    for (k, leg) in legs.iter().enumerate().skip(idx) {
        let Some(max) = restriction_at(cons, leg).and_then(|c| c.max_ft) else {
            continue;
        };
        let d = distance_to_leg_nm(from.lat, from.lon, legs, idx, k);
        let c = max + path_nm(d) * ft_per_nm;
        ceiling = Some(ceiling.map_or(c, |x: f64| x.min(c)));
    }
    let floor = restriction_at(cons, &legs[idx]).and_then(|c| c.min_ft);
    ceiling.map(|c| floor.map_or(c, |f| c.max(f)))
}

/// The plan's OWN altitudes ahead as ceilings, the same way: this fix's
/// target is lowered to what still makes every later fix's planned altitude
/// at `ft_per_nm`. Only fixes after `idx`, and only a plain or "at or below"
/// altitude (an approach's "at or above" is a floor, never a ceiling).
/// LIME-LIPE (2026-09-25): each leg was paced only to its own fix, so the
/// descent to PE725's 8,000 ft could not start until PE730 was behind, with
/// four miles to lose six thousand feet: 14,200 ft at 11 nm from the field
/// and the pilot stepping the dial down by hand.
pub fn profile_ceiling_ft(legs: &[ProcLeg], idx: usize, ft_per_nm: f64) -> Option<f64> {
    let from = legs.get(idx)?.fix.as_ref()?;
    let mut ceiling: Option<f64> = None;
    for (k, leg) in legs.iter().enumerate().skip(idx + 1) {
        if !matches!(leg.alt_desc, ' ' | '@' | '-') {
            continue;
        }
        let Some(alt) = leg.alt1_ft else { continue };
        let d = distance_to_leg_nm(from.lat, from.lon, legs, idx, k);
        let c = alt + path_nm(d) * ft_per_nm;
        ceiling = Some(ceiling.map_or(c, |x: f64| x.min(c)));
    }
    ceiling
}

/// The miles of a later fix's distance a ceiling may climb on: all of it
/// but the descent's lead ([`TOD_LEAD_NM`]), which the path to that fix
/// spends level at its altitude. The ceilings and the path are then ONE
/// line: crossing a fix at its ceiling puts the aircraft exactly on the next
/// leg's path. With the whole distance counted, every leg of the LEMG UNTO2A
/// began a lead's worth of gradient above its own path (2026-09-26).
pub(super) fn path_nm(d: f64) -> f64 {
    if d <= 0.0 {
        0.0
    } else {
        (d - TOD_LEAD_NM).max(0.0)
    }
}

/// Feet per nm the arrival ceiling is planned on at this groundspeed: the
/// SAME planning rate the descent path is flown at
/// ([`DESCENT_PLAN_FRACTION`]). It was 85 % against the path's 75 %, so a
/// fix crossed at its ceiling was above the next leg's path and each leg
/// started by catching up at the full rate, the margin a speed limit needs
/// already spent (FOFUS 386 ft high, LEMG 2026-09-26).
pub fn arrival_ft_per_nm(descent_fpm: f64, gs_kts: f64) -> f64 {
    DESCENT_PLAN_FRACTION * descent_fpm / (gs_kts.max(60.0) / 60.0)
}

/// The altitude leg `idx` is flown to once the ceilings ahead
/// ([`arrival_ceiling_ft`], [`profile_ceiling_ft`]) are applied: `Some((alt,
/// ceiling))` when a ceiling lowered the plan's altitude, `None` otherwise.
/// `latch` holds the lowest ceiling seen on this leg: the ceiling is a
/// gradient over the groundspeed, so it RISES as the aircraft slows, and an
/// unlatched one lifted MG539's target from 14,900 to 15,500 ft in the last
/// two miles and the 737 climbed at +260 fpm into its 230 kt slow-down
/// (LEMG 2026-09-26). A descent's target never climbs back.
pub fn ceilinged_leg_alt(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    idx: usize,
    gs_kts: f64,
    descent_fpm: f64,
    latch: &mut Option<(usize, f64)>,
) -> Option<(f64, f64)> {
    let grad = arrival_ft_per_nm(descent_fpm, gs_kts);
    let ceiling = match (
        arrival_ceiling_ft(legs, cons, idx, grad),
        profile_ceiling_ft(legs, idx, grad),
    ) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (a, b) => a.or(b),
    };
    let ceiling = match (ceiling, *latch) {
        (Some(c), Some((i, l))) if i == idx => Some(c.min(l)),
        (c, _) => c,
    };
    if let Some(c) = ceiling {
        *latch = Some((idx, c));
    }
    match (legs.get(idx)?.alt1_ft, ceiling) {
        (Some(a), Some(c)) if c < a - 50.0 => Some(((c / 100.0).floor() * 100.0, c)),
        _ => None,
    }
}

/// Knots a second the governor is planned to lose speed at in a descent.
pub(super) const ARRIVAL_DECEL_KTS_S: f64 = 0.5;
/// Slowed this far before the fix, nm, on top of the deceleration itself.
pub(super) const ARRIVAL_SPEED_LEAD_NM: f64 = 2.0;
/// Knots under a STAR's speed LIMIT the follower flies: a maximum is not a
/// target, and the speed held down a descent at idle wanders a few knots
/// either side of what is asked (OSKOT and GROVE's 210 crossed at 216 and
/// 218, EGKK-EGBB 2026-09-28, #156).
pub const STAR_SPEED_MARGIN_KTS: f64 = 3.0;

/// The arrival's speed limit now: the lowest of every limit already passed
/// (a limit holds from its fix on) and every one ahead whose slow-down has
/// begun, at [`ARRIVAL_DECEL_KTS_S`] from `ias_kts` plus the lead. `(knots,
/// fix)`; `None` when nothing applies yet.
///
/// A limit whose slow-down has BEGUN holds (`begun`, the legs' indices,
/// carried tick to tick): the start is judged from the speed still to lose,
/// so an aircraft slowing at the planned rate rides the boundary and the
/// limit came and went with it. EGKK-EGBB (2026-09-27, #126): OSKOT's 210
/// against 250, 8.9 nm out at 257 kt, flipped the governor's target five
/// times in 27 s, and each flip moved the speedbrake, said aloud.
pub fn arrival_speed_cap(
    legs: &[ProcLeg],
    cons: &[crate::nav::StarConstraint],
    idx: usize,
    lat: f64,
    lon: f64,
    ias_kts: f64,
    gs_kts: f64,
    begun: &mut std::collections::HashSet<usize>,
) -> Option<(f64, String)> {
    let mut cap: Option<(f64, String)> = None;
    for (k, leg) in legs.iter().enumerate() {
        let Some(kts) = restriction_at(cons, leg).and_then(|c| c.max_kts) else {
            continue;
        };
        let applies = k < idx || begun.contains(&k) || {
            let d = distance_to_leg_nm(lat, lon, legs, idx, k);
            let slow_nm = (ias_kts - kts).max(0.0) / ARRIVAL_DECEL_KTS_S * gs_kts.max(60.0) / 3600.0;
            d <= slow_nm + ARRIVAL_SPEED_LEAD_NM
        };
        if applies {
            begun.insert(k);
        }
        if applies && cap.as_ref().is_none_or(|(c, _)| kts < *c) {
            cap = Some((kts, leg.fix_ident.clone()));
        }
    }
    cap
}

/// A STAR speed limit judged where it is FLOWN: the airspeed as the leg to
/// its fix sequences, against the limit and the target under it
/// ([`STAR_SPEED_MARGIN_KTS`]). Blackboxed `star_speed_crossed`.
#[derive(Debug, Clone, PartialEq)]
pub struct StarSpeedCrossing {
    pub fix: String,
    pub kts: f64,
    pub target_kts: f64,
    pub ias: f64,
    /// At or under the limit.
    pub met: bool,
    /// Knots over the target (negative: under it).
    pub over_target_kts: f64,
}

/// The crossing of `leg`'s speed limit at `ias_kts`, or `None` when the fix
/// has none.
///
/// `star_speed` is logged when the slow-down BEGINS, with the airspeed of
/// that moment, and was read as the crossing: iMac 2026-09-30 (sortie
/// 1790759458-full_auto, #156) "OSKOT 248 against 210" was the slow-down
/// starting 7.86 nm out at 248 kt; OSKOT sequenced 2.1 nm out at 208.4, the
/// fix itself inside the limit, as were PUFAX (246.7 against 250) and GROVE
/// (206.1 against 210). The crossing now has its own record.
pub fn star_speed_crossing(
    cons: &[crate::nav::StarConstraint],
    leg: &ProcLeg,
    ias_kts: f64,
) -> Option<StarSpeedCrossing> {
    let kts = restriction_at(cons, leg)?.max_kts?;
    let target_kts = kts - STAR_SPEED_MARGIN_KTS;
    Some(StarSpeedCrossing {
        fix: leg.fix_ident.clone(),
        kts,
        target_kts,
        ias: ias_kts,
        met: ias_kts <= kts,
        over_target_kts: ias_kts - target_kts,
    })
}

/// The arrival's hand-over speed a plan carries: an enroute plan's only,
/// and only a real speed.
pub(super) fn arrival_handoff_for(kts: Option<f64>, end: &EndAction) -> Option<f64> {
    kts.filter(|k| k.is_finite() && *k > 0.0)
        .filter(|_| matches!(end, EndAction::Enroute))
}

/// The arrival's hand-over speed as a limit at the END of the route, slowed
/// for in time the way a STAR limit is ([`arrival_speed_cap`]: at
/// [`ARRIVAL_DECEL_KTS_S`] from `ias_kts` plus [`ARRIVAL_SPEED_LEAD_NM`]),
/// and held once begun (`begun`). `to_end_nm` is the route still to fly.
/// `None` while the slow-down is still ahead. Pure.
///
/// EGBB-EGNX (2026-09-30, sortie 1790761044-full_auto, #199): a 40 nm local
/// plan cruised at 3,000 ft and 250 kt to its last fix, and autoland was
/// handed a clean 737 at 252 kt on the path 8.4 nm out. It flew the final at
/// 255 with the flaps held up by their placards. Autoland expects its clean
/// approach speed (`autoland::clean_approach_ias`, 210 for the 737) there.
pub fn arrival_handoff_cap(
    to_end_nm: f64,
    ias_kts: f64,
    gs_kts: f64,
    handoff_kts: f64,
    begun: &mut bool,
) -> Option<f64> {
    if !*begun {
        let slow_nm =
            (ias_kts - handoff_kts).max(0.0) / ARRIVAL_DECEL_KTS_S * gs_kts.max(60.0) / 3600.0;
        *begun = to_end_nm <= slow_nm + ARRIVAL_SPEED_LEAD_NM;
    }
    begun.then_some(handoff_kts)
}

/// The true great-circle bearing to a leg's fix, or `None` for a to-altitude
/// leg (no fix to steer at). Pure — the lateral-steering decision, testable
/// without a sim.
pub fn bearing_to_fix(lat: f64, lon: f64, leg: &ProcLeg) -> Option<f64> {
    leg.fix
        .as_ref()
        .map(|f| crate::nav::bearing_deg(lat, lon, f.lat, f.lon))
}

/// The turn radius an enroute turn is actually flown at, nautical miles:
/// v²/(g·tan φ) at [`ENROUTE_BANK_DEG`].
///
/// NOT [`crate::circuit::turn_radius_at`], whose 250..2500 m clamp is a
/// PATTERN bound — 2500 m is 1.35 nm, which is already less than this
/// airframe's real radius at climb speed and a ninth of it at cruise. A clamp
/// that exists to keep a light aircraft's circuit geometry sane must not be
/// the thing that decides whether an airliner can reach its next fix.
pub fn enroute_turn_radius_nm(gs_kts: f64) -> f64 {
    let v = gs_kts.max(40.0) * 0.514_444;
    v * v / (9.81 * ENROUTE_BANK_DEG.to_radians().tan()) / 1852.0
}

/// Turn anticipation: how far before the fix the turn onto the next leg has to
/// begin, which is therefore how far out the leg counts as flown.
///
/// `R·tan(θ/2)` — the standard fly-by geometry, the distance at which a circle
/// of radius `R` is tangent to both legs — floored at [`CAPTURE_NM`] and
/// capped at `R` so a near-reversal can't ask for an unbounded lead.
///
/// The shape is what makes it right on both airframes: a 172 at 100 knots has
/// a 0.4 nm radius, so even a 90° turn asks for less than the floor and its
/// behaviour is unchanged; a 737 at cruise has an 8 nm radius but a 10° track
/// change, which also lands under the floor. It only bites where it must —
/// a big radius AND a real turn, which is exactly the jet departure that
/// orbited.
pub fn capture_radius_nm(gs_kts: f64, turn_deg: f64) -> f64 {
    let r = enroute_turn_radius_nm(gs_kts);
    let half = (turn_deg.abs().min(175.0) / 2.0).to_radians();
    (r * half.tan()).clamp(CAPTURE_NM, r.max(CAPTURE_NM))
}

/// The track change at this leg's fix: the bearing we are flying INTO it
/// against the bearing from it on to the next leg's fix, signed right-positive.
/// `None` when either end has no fix to measure between — a to-altitude leg,
/// an unresolved fix, or the last leg of the route, none of which are turns
/// this anticipates.
/// Seconds a leg's first turn is given before the closure watchdog may judge
/// it: the turn at 2 degrees a second, slower than standard rate because the
/// bug leads the nose by only 25 degrees until the bank is established.
pub fn leg_turn_grace_s(turn_deg: f64) -> f64 {
    turn_deg.abs().min(180.0) / 2.0
}

pub fn leg_turn_deg(lat: f64, lon: f64, leg: &ProcLeg, next: Option<&ProcLeg>) -> Option<f64> {
    let here = leg.fix.as_ref()?;
    let there = next?.fix.as_ref()?;
    let inbound = crate::nav::bearing_deg(lat, lon, here.lat, here.lon);
    let outbound = crate::nav::bearing_deg(here.lat, here.lon, there.lat, there.lon);
    Some(crate::angles::signed_diff_deg(outbound, inbound))
}

/// Has the current leg been flown? A fix leg completes inside `capture_nm`
/// (see [`capture_radius_nm`] — the turn onto the next leg has to START
/// there, so that IS the fix being reached); a to-altitude leg completes when
/// the altitude constraint is satisfied. Pure and position-based, so
/// sequencing is immune to how slow the AP's turns are (the same lesson that
/// drove the circuit off timers).
/// A DME-terminated leg (CD, VD, FD: "the 291 course to 6 DME from RIM") is
/// done when the aircraft is that far from its navaid. `None` for any other
/// leg. Its altitude is a CONSTRAINT, not the terminator: judged by altitude
/// alone, ANC8A's "to 6 DME, at or above 1,500" would have turned back at
/// 1,500 feet, two miles out. Pure.
pub fn dme_leg_done(leg: &ProcLeg, lat: f64, lon: f64) -> Option<bool> {
    if !matches!(leg.leg_type.as_str(), "CD" | "VD" | "FD") {
        return None;
    }
    let (n, d) = (leg.navaid.as_ref()?, leg.dist_nm.filter(|d| *d > 0.0)?);
    Some(crate::nav::haversine_nm(lat, lon, n.lat, n.lon) >= d - 0.1)
}

/// [`leg_captured`], DME-terminated legs first.
pub fn leg_done(leg: &ProcLeg, dist_to_fix_nm: Option<f64>, alt_ft: f64, capture_nm: f64, lat: f64, lon: f64) -> bool {
    dme_leg_done(leg, lat, lon).unwrap_or_else(|| leg_captured(leg, dist_to_fix_nm, alt_ft, capture_nm))
}

pub fn leg_captured(
    leg: &ProcLeg,
    dist_to_fix_nm: Option<f64>,
    alt_ft: f64,
    capture_nm: f64,
) -> bool {
    match dist_to_fix_nm {
        Some(d) => d <= capture_nm.max(CAPTURE_NM),
        // No fix: a to-altitude leg. '+' climb to at-or-above, '-' descend to
        // at-or-below, otherwise cross at the altitude.
        None => match leg.alt1_ft {
            Some(target) => match leg.alt_desc {
                '+' => alt_ft >= target - ALT_MARGIN_FT,
                '-' => alt_ft <= target + ALT_MARGIN_FT,
                _ => (alt_ft - target).abs() <= ALT_MARGIN_FT,
            },
            // A heading leg with no altitude (rare, FM/VM manual) — nothing to
            // wait for; treat as instantly done so we don't stall the sequence.
            None => true,
        },
    }
}

/// The altitude-dial target for a leg, feet MSL: its own constraint when it has
/// one, else `carry` (hold the last commanded altitude across an unconstrained
/// fix). The AP's ALT hold flies toward it — climb or descent falls out of
/// whether the target is above or below.
pub fn commanded_alt_ft(leg: &ProcLeg, carry: f64) -> f64 {
    leg.alt1_ft.unwrap_or(carry)
}
