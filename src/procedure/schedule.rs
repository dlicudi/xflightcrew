//! The plan's shape: the approach plan, the cruise speed and steps, the
//! enroute legs and the arrival gateway.

use super::*;

/// A human word for the approach type from its ident's leading letter.
pub(crate) fn approach_kind(ident: &str) -> &'static str {
    match ident.chars().next() {
        Some('I') => "ILS",
        Some('X') | Some('R') | Some('Q') => "RNAV",
        Some('L') => "localizer",
        Some('D') | Some('S') | Some('V') => "VOR",
        _ => "instrument",
    }
}

/// Build an APPROACH plan for `rwy_id` at `icao`: pick the best published
/// approach, load its final segment, resolve every fix against the nav data.
/// `Err` carries an honest refusal (no published approach / unreadable legs) for
/// the caller to speak. Shared by the `fly_procedure` tool and the circuit's
/// arrival handoff so both build the arrival the same way.
#[allow(clippy::too_many_arguments)]
pub fn approach_plan_for(
    cifp: &str,
    fix_path: &str,
    nav_path: &str,
    airport: &crate::nav::AirportInfo,
    icao: &str,
    rwy_id: &str,
    lat: f64,
    lon: f64,
    end: EndAction,
) -> Result<Plan, String> {
    let spoken = crate::crew_radio::spell_runway(rwy_id);
    // Prefer a full CIFP approach; fall back to a straight-in reconstructed from
    // the runway's published DME fixes (the way a pilot flies the DME approach
    // when there's no coded procedure — Gibraltar's whole story).
    let mut visual: Option<crate::nav::VisualSegment> = None;
    let mut missed_legs: Vec<ProcLeg> = Vec::new();
    let mut found: Option<(Vec<ProcLeg>, String)> = None;
    for appch in crate::nav::approach_candidates(cifp, icao, rwy_id) {
        let raw = crate::nav::load_proc_legs(cifp, icao, crate::nav::ProcKind::Appch, &appch, None);
        // As far as the threshold: the missed approach is never the approach
        // (`straight_in_legs`).
        if let Some(mut legs) = crate::nav::straight_in_legs(raw.clone(), rwy_id) {
            crate::nav::resolve_proc_fixes(fix_path, nav_path, airport, icao, &mut legs, lat, lon);
            found = Some((legs, format!("the {} approach runway {spoken}", approach_kind(&appch))));
            break;
        }
        // An approach to a missed approach point short of the runway (LSZS):
        // flown from its TRANSITION, the published way in over the terrain,
        // to the MAP on its own path, and landed from there on a visual
        // segment the landing can fly. Judged once the fixes are resolved.
        let Some((legs, missed)) =
            map_approach_legs(cifp, fix_path, nav_path, airport, icao, &appch, lat, lon)
        else {
            continue;
        };
        let Some(seg) = runway_end(airport, rwy_id).and_then(|(thr, course)| {
            crate::nav::visual_segment(&legs, thr, course, airport.elev_ft)
        }) else {
            continue;
        };
        crate::blackbox::record(
            "procedure",
            serde_json::json!({"event": "visual_segment", "approach": appch, "runway": rwy_id,
                "dist_nm": seg.dist_nm, "lateral_nm": seg.lateral_nm,
                "course_diff": seg.course_diff_deg, "angle": seg.angle_deg, "flyable": seg.flyable()}),
        );
        if seg.flyable() {
            visual = Some(seg);
            missed_legs = missed;
            found = Some((
                legs,
                format!("the {} approach runway {spoken}", approach_kind(&appch)),
            ));
            break;
        }
    }
    let (legs, name) = found.unwrap_or_default();
    // The angle handed to the landing is measured to ITS aim point (the
    // touchdown zone past the threshold), so the MAP sits on its path rather
    // than under it: LSZS 03, 4.74 to the threshold, 4.4 to the aim point.
    let visual_deg = visual.map(|v| {
        let height_ft = v.dist_nm * 6076.12 * v.angle_deg.to_radians().tan();
        (height_ft / ((v.dist_nm + crate::autoland::TOUCHDOWN_AIM_NM) * 6076.12))
            .atan()
            .to_degrees()
    });
    let (legs, name) = if legs.is_empty() {
        let recon = crate::nav::reconstruct_approach(fix_path, airport, icao, rwy_id);
        if recon.is_empty() {
            return Err(format!(
                "{icao} has no published approach to runway {spoken}. No coded procedure and no \
                 DME approach fixes to reconstruct one from."
            ));
        }
        (recon, format!("a straight-in approach to runway {spoken}"))
    } else {
        (legs, name)
    };
    // The FAF is where autoland takes the aircraft, so it has to be somewhere
    // autoland can land FROM. A published approach's FAF is on the course by
    // definition; a reconstructed one is only as good as its fixes. A visual
    // segment is handed over at the MAP instead, judged above.
    if visual.is_none() {
        if let Some(why) = faf_off_course(&legs, airport, rwy_id) {
            return Err(why);
        }
    }
    // Approaches descend; the climb protection reference stays off unless the
    // caller sets one (e.g. the enroute follower).
    Ok(Plan {
        name,
        legs,
        end,
        vy_kts: 0.0,
        cruise_ias_kts: None, // the follower holds the clean approach speed
        transition_altitude_ft: None,
        climb_minimums: Vec::new(),
        arrival_constraints: Vec::new(),
        visual_deg,
        missed_legs,
        cruise_schedule: Default::default(),
        arrival_handoff_kts: None,
    })
}

/// A MAP approach's legs from its best TRANSITION: the one whose first fix is
/// nearest the aircraft, straight legs only (a hold or a procedure turn is not
/// flown), joined to the final and cut at the MAP. Resolved. `None` when the
/// approach does not end at a MAP or no transition can be flown (the final's
/// IF alone could lie across the ridges the transition goes round: LSZS's
/// ZS701 is 39 nm from RONAG, over the Bernina).
#[allow(clippy::too_many_arguments)]
pub(super) fn map_approach_legs(
    cifp: &str,
    fix_path: &str,
    nav_path: &str,
    airport: &crate::nav::AirportInfo,
    icao: &str,
    appch: &str,
    lat: f64,
    lon: f64,
) -> Option<(Vec<ProcLeg>, Vec<ProcLeg>)> {
    let flyable = |l: &ProcLeg| matches!(l.leg_type.as_str(), "IF" | "TF" | "CF" | "DF");
    let mut best: Option<(f64, Vec<ProcLeg>)> = None;
    for t in crate::nav::approach_transitions(cifp, icao, appch) {
        let raw = crate::nav::load_proc_legs(cifp, icao, crate::nav::ProcKind::Appch, appch, Some(&t));
        let Some(mut legs) = crate::nav::map_final_legs(raw) else { continue };
        if !legs.iter().all(flyable) {
            continue;
        }
        // The transition's last fix is the final's IF: flown once.
        legs.dedup_by(|b, a| !a.fix_ident.is_empty() && a.fix_ident == b.fix_ident);
        crate::nav::resolve_proc_fixes(fix_path, nav_path, airport, icao, &mut legs, lat, lon);
        let Some(first) = legs.iter().find_map(|l| l.fix.as_ref()) else { continue };
        let d = crate::nav::haversine_nm(lat, lon, first.lat, first.lon);
        if best.as_ref().is_none_or(|(bd, _)| d < *bd) {
            best = Some((d, legs));
        }
    }
    let (_, legs) = best?;
    // The published missed approach: the final's legs after the MAP, straight
    // legs only (the hold at its end is not flown; the follower levels at the
    // last constraint and hands over there).
    let all = crate::nav::load_proc_legs(cifp, icao, crate::nav::ProcKind::Appch, appch, None);
    let mut missed: Vec<ProcLeg> = all
        .iter()
        .position(|l| l.is_map)
        .map(|at| all[at + 1..].iter().filter(|l| flyable(l) && !l.fix_ident.is_empty()).cloned().collect())
        .unwrap_or_default();
    missed.dedup_by(|b, a| a.fix_ident == b.fix_ident);
    crate::nav::resolve_proc_fixes(fix_path, nav_path, airport, icao, &mut missed, lat, lon);
    Some((legs, missed))
}

/// The threshold of runway end `rwy_id` and its course, degrees true.
pub(super) fn runway_end(airport: &crate::nav::AirportInfo, rwy_id: &str) -> Option<((f64, f64), f64)> {
    airport.runways.iter().find_map(|r| {
        if r.ident_a.eq_ignore_ascii_case(rwy_id) {
            Some(((r.lat_a, r.lon_a), r.heading_a_deg()))
        } else if r.ident_b.eq_ignore_ascii_case(rwy_id) {
            Some(((r.lat_b, r.lon_b), r.heading_b_deg()))
        } else {
            None
        }
    })
}

/// The refusal when the handoff fix is not somewhere autoland can intercept the
/// runway from: the FAF's cross-track and along-track distances in the runway
/// frame have to pass autoland's own `interceptable` test, the one the circuit
/// applies before it hands over. Gibraltar's 09D05 sits 3.6 km abeam the 09
/// centreline with 5.5 km to run (a 50 degree offset); handing off there flew a
/// saturated 35 degree cut that was still 300 m off at 95 ft, went around, and
/// crashed (2026-09-11). `None` = flyable, or nothing to judge (no FAF, no
/// runway geometry) — the follower's own checks take over from there.
pub(super) fn faf_off_course(
    legs: &[ProcLeg],
    airport: &crate::nav::AirportInfo,
    rwy_id: &str,
) -> Option<String> {
    let hi = handoff_index(legs)?;
    let faf = legs[hi].fix.as_ref()?;
    let (thr_lat, thr_lon) = crate::nav::runway_threshold(airport, rwy_id)?;
    let course = crate::nav::runway_course_deg(airport, rwy_id)?;
    let (xtk_m, along_m) = crate::nav::runway_frame_m(thr_lat, thr_lon, course, faf.lat, faf.lon);
    if crate::circuit::interceptable(xtk_m, along_m) {
        return None;
    }
    Some(format!(
        "The {} fix is {:.1} miles off the runway {} centreline with {:.1} to run. \
         Autoland can't line up from there, so that approach is out.",
        faf.ident,
        xtk_m.abs() / 1852.0,
        crate::crew_radio::spell_runway(rwy_id),
        along_m.max(0.0) / 1852.0
    ))
}

/// The indicated speed the throttle governor holds while the follower flies
/// this plan, if any. Enroute: the profile's cruise speed. An APPROACH: the
/// clean approach speed autoland itself flies outside the deceleration ramp,
/// so the handoff at the FAF is at a speed it expects. Before this, an approach
/// plan left the throttle wherever the enroute leg had it: cruise power all the
/// way down the straight-in, 195 kt at the FAF against a 100 kt approach speed,
/// and the gear and flaps went out 40 kt over their limits (2026-09-11).
pub fn power_hold_target(end: &EndAction, cruise_ias_kts: Option<f64>) -> Option<f64> {
    match end {
        EndAction::Enroute => cruise_ias_kts,
        EndAction::Approach { approach_ias, .. } => {
            Some(crate::autoland::clean_approach_ias(*approach_ias))
        }
        EndAction::Level => None,
    }
}

/// A leg's planned cruise speed from the OFP: the airspeed and Mach the cost
/// index gave its fix, and what the governor should hold at `alt_ft`.
#[derive(Debug, Clone, PartialEq)]
pub struct ScheduledSpeed {
    pub fix: String,
    pub plan_ias_kts: Option<f64>,
    pub mach: Option<f64>,
    /// The lower of the planned airspeed and the planned Mach at the level
    /// flown: an airline's IAS/Mach schedule, so a level other than the
    /// plan's (a pilot's, a controller's) still stays on the Mach.
    pub target_kts: f64,
}

/// The fix a schedule entry must be within to be this leg's, nm: an ident is
/// not unique across the world, a position is.
pub(super) const SCHEDULE_MATCH_NM: f64 = 2.0;

/// The OFP cruise speed for the leg flown, if the navlog planned it as a
/// CRUISE leg (its fix's `stage` is CRZ). Climb and descent legs are `None`:
/// the airframe's speed, the 250 below ten thousand and the STAR limits
/// keep them as before. Pure.
pub fn scheduled_cruise_kts(
    schedule: &crate::simbrief::CruiseSchedule,
    leg: &ProcLeg,
    alt_ft: f64,
) -> Option<ScheduledSpeed> {
    let e = schedule.fixes.iter().find(|e| {
        e.stage == crate::simbrief::Stage::Cruise
            && e.ident.eq_ignore_ascii_case(&leg.fix_ident)
            && leg
                .fix
                .as_ref()
                .is_some_and(|f| crate::nav::haversine_nm(f.lat, f.lon, e.lat, e.lon) <= SCHEDULE_MATCH_NM)
    })?;
    let by_mach = e
        .mach
        .filter(|_| alt_ft.is_finite())
        .map(|m| cas_for_mach(m, alt_ft));
    let target = match (e.ias_kts, by_mach) {
        (Some(a), Some(b)) => a.min(b),
        (a, b) => a.or(b)?,
    };
    Some(ScheduledSpeed {
        fix: e.ident.clone(),
        plan_ias_kts: e.ias_kts,
        mach: e.mach,
        target_kts: target.round(),
    })
}

/// Which cruise speed the governor holds on this leg, and why.
#[derive(Debug, Clone, PartialEq)]
pub struct CruisePick {
    pub kts: f64,
    /// "ofp" or "profile".
    pub source: &'static str,
    /// Why the OFP's speed was not taken: "below_floor", "above_ceiling".
    pub rejected: Option<&'static str>,
    /// The OFP's speed when it was taken down to [`cruise_cap_kts`].
    pub capped_from: Option<f64>,
}

/// The highest cruise the follower schedules: the overspeed caution's
/// floor less its margin, the governor's own ceiling
/// ([`crate::aircraft::ceiling_from_vne`]). `vne_kts` is the airframe's;
/// `None` without one.
///
/// EGKK-EGBB, 2026-09-28 (sortie 1790601822-full_auto, #156): the OFP's
/// 299 and 300 KIAS at FL180 against the 737's 340 kt Vne (caution from
/// 306): the governor held 294.8 to 302.7, and the nose-down at the top of
/// descent (+2115.3, 305 kt with the trend) said "Airspeed, you're getting
/// fast." A crew flies the plan's speed, never into the caution. Pure.
pub fn cruise_cap_kts(vne_kts: Option<f64>) -> Option<f64> {
    vne_kts
        .filter(|v| *v > 0.0)
        .map(crate::aircraft::ceiling_from_vne)
}

/// The OFP's speed where the leg has one and it is inside the airframe's
/// floor and ceiling; the profile's `cruise_ias_kts` otherwise. A plan made
/// for another type, or a number outside what this airframe may fly, is
/// never flown: the airframe's own figure is (a wrong number handed down
/// from a plan is still wrong). A target over the ceiling would also be
/// clamped by the governor every tick and re-armed every tick. Pure.
pub fn cruise_pick(
    profile_kts: f64,
    scheduled: Option<&ScheduledSpeed>,
    floor_kts: Option<f64>,
    ceiling_kts: Option<f64>,
    cap_kts: Option<f64>,
) -> CruisePick {
    let profile = |rejected| CruisePick {
        kts: profile_kts,
        source: "profile",
        rejected,
        capped_from: None,
    };
    let Some(s) = scheduled else {
        return profile(None);
    };
    if floor_kts.is_some_and(|f| s.target_kts < f) {
        return profile(Some("below_floor"));
    }
    if ceiling_kts.is_some_and(|c| s.target_kts > c) {
        return profile(Some("above_ceiling"));
    }
    // Under the overspeed caution ([`cruise_cap_kts`]): the plan's speed,
    // as near it as the caution allows (never under the floor).
    match cap_kts.filter(|c| s.target_kts > *c && floor_kts.is_none_or(|f| *c >= f)) {
        Some(c) => CruisePick {
            kts: c,
            source: "ofp",
            rejected: None,
            capped_from: Some(s.target_kts),
        },
        None => CruisePick {
            kts: s.target_kts,
            source: "ofp",
            rejected: None,
            capped_from: None,
        },
    }
}

/// A planned step placed on the legs: from the leg after `idx` (the leg
/// flown FROM the step fix) the plan's level is `alt_ft`.
#[derive(Debug, Clone, PartialEq)]
pub struct PlacedStep {
    pub fix: String,
    pub idx: usize,
    pub alt_ft: f64,
}

/// The plan's step climbs, flown where the OFP planned them. `enroute_legs`
/// commands the plan's HIGHEST level from the first leg (one continuous climb
/// to cruise), which for a stepped plan is the LAST step: FL340 then FL360
/// at ERNAN was flown as FL360 from the runway. With the OFP's
/// `stepclimb_string` read, every climb and cruise leg (before the first
/// DESCENT-stage fix) commands the level in force: the initial level up to
/// and including the first step fix, the step's level after it, and so on.
/// The descent keeps the navlog's own numbers. Returns the steps placed. A
/// plan with one level, no stages, an implausible initial level or a step
/// fix not on the route (before the descent) is left exactly as built. Pure.
pub fn apply_step_climbs(
    legs: &mut [ProcLeg],
    schedule: &crate::simbrief::CruiseSchedule,
) -> Vec<PlacedStep> {
    let Some((first, rest)) = schedule.steps.split_first() else {
        return Vec::new();
    };
    if rest.is_empty() || !(1_000.0..=60_000.0).contains(&first.alt_ft) {
        return Vec::new();
    }
    let stage_of = |leg: &ProcLeg| {
        schedule
            .fixes
            .iter()
            .find(|e| {
                e.ident.eq_ignore_ascii_case(&leg.fix_ident)
                    && leg.fix.as_ref().is_some_and(|f| {
                        crate::nav::haversine_nm(f.lat, f.lon, e.lat, e.lon) <= SCHEDULE_MATCH_NM
                    })
            })
            .map(|e| e.stage)
    };
    let Some(top) = legs
        .iter()
        .position(|l| stage_of(l) == Some(crate::simbrief::Stage::Descent))
        .or_else(|| legs.iter().any(|l| stage_of(l).is_some()).then_some(legs.len()))
    else {
        return Vec::new();
    };
    let mut placed = Vec::new();
    let mut from = 0usize;
    for s in rest {
        let Some(j) = legs[..top]
            .iter()
            .enumerate()
            .skip(from)
            .find(|(_, l)| l.fix_ident.eq_ignore_ascii_case(&s.fix))
            .map(|(j, _)| j)
        else {
            // A step the route does not carry: the plan cannot be flown as
            // stepped, so none of it is (the navlog's levels stand).
            return Vec::new();
        };
        placed.push(PlacedStep { fix: s.fix.clone(), idx: j, alt_ft: s.alt_ft });
        from = j + 1;
    }
    for (i, leg) in legs[..top].iter_mut().enumerate() {
        let level = placed
            .iter()
            .rev()
            .find(|p| p.idx < i)
            .map_or(first.alt_ft, |p| p.alt_ft);
        leg.alt1_ft = Some(level);
    }
    placed
}

/// The step climb that begins on leg `idx`: the leg before it ends at a
/// planned step fix and this leg's level is a step above that leg's. Pure.
pub fn step_climb_at(
    legs: &[ProcLeg],
    idx: usize,
    schedule: &crate::simbrief::CruiseSchedule,
) -> Option<(String, f64, f64)> {
    let prev = legs.get(idx.checked_sub(1)?)?;
    let (from, to) = (prev.alt1_ft?, legs.get(idx)?.alt1_ft?);
    (to > from + STEP_FT
        && schedule
            .steps
            .iter()
            .skip(1)
            .any(|s| s.fix.eq_ignore_ascii_case(&prev.fix_ident)))
    .then(|| (prev.fix_ident.clone(), from, to))
}

/// Build the ENROUTE legs of a SimBrief plan: direct-to legs over the named
/// fixes, with the plan's own crossing altitudes as constraints. Pure, so the
/// altitude logic battle-tests without a fetch.
///
/// - The origin airport (prepended by the fetch) and the destination airport
///   (the arrival phase owns that) are dropped — this is the middle of the
///   flight only.
/// - SimBrief's per-fix altitudes carry the whole profile: rising through the
///   climb, cruise in the middle, falling after TOD. Flying the CLIMB fixes at
///   their charted crossing altitudes would stair-step the departure (level at
///   3,000 over a fix mid-climb, then climb again), so every fix before the
///   first at-cruise fix commands cruise directly — one continuous climb — and
///   from there the plan's own numbers stand, which is what flies the descent.
/// - A fix with no plausible altitude (0 in the navlog) commands cruise.
pub fn enroute_legs(
    waypoints: &[crate::simbrief::Waypoint],
    origin: &str,
    destination: &str,
    cruise_alt_ft: f64,
) -> Vec<ProcLeg> {
    let mid: Vec<&crate::simbrief::Waypoint> = waypoints
        .iter()
        .enumerate()
        .filter(|(i, w)| {
            (*i != 0 || !w.ident.eq_ignore_ascii_case(origin))
                && (*i != waypoints.len() - 1 || !w.ident.eq_ignore_ascii_case(destination))
        })
        .map(|(_, w)| w)
        .collect();
    let plausible = |alt: f64| (500.0..=60_000.0).contains(&alt);
    let raw: Vec<f64> = mid
        .iter()
        .map(|w| {
            if plausible(w.alt) {
                w.alt
            } else {
                cruise_alt_ft
            }
        })
        .collect();
    let peak = raw.iter().copied().fold(cruise_alt_ft, f64::max);
    let first_at_peak = raw.iter().position(|a| *a >= peak - 100.0).unwrap_or(0);
    // A fix charted within a few hundred feet of cruise IS cruise: SimBrief
    // put MG834 at 3,900 between two 4,000 ft fixes and the dial dipped and
    // came back for nothing (2026-09-11). Real descents are thousands.
    const CRUISE_DIP_FT: f64 = 300.0;
    mid.iter()
        .zip(raw.iter())
        .enumerate()
        .map(|(i, (w, alt))| ProcLeg {
            seq: i as u32,
            transition: String::new(),
            leg_type: "TF".to_string(),
            fix_ident: w.ident.clone(),
            fix_region: String::new(),
            fix: Some(crate::nav::Fix {
                ident: w.ident.clone(),
                lat: w.lat,
                lon: w.lon,
            }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: Some(
                if i < first_at_peak || (peak - *alt).abs() <= CRUISE_DIP_FT {
                    peak
                } else {
                    *alt
                },
            ),
            alt2_ft: None,
            is_faf: false,
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
        })
        .collect()
}

/// Extend a filed route whose last real fix is remote from its destination to
/// an arrival handoff point. The destination airport itself stays out of the
/// enroute legs: flying over it before arming an approach would be too late.
///
/// SimBrief EGLF-LFOS ended at WAFFU, 46 nm from LFOS. The follower treated
/// WAFFU as the end of the flight, slowed for the approach there, and handed
/// autoland an airport outside its 40 nm runway search (#214). A STAR or local
/// route that already reaches the arrival area is left byte-for-byte alone.
pub const ARRIVAL_GATEWAY_NM: f64 = 12.0;

/// Whether [`extend_to_arrival_gateway`] would add a gateway: the route's
/// last fix lies beyond the arrival area. A caller that must read the
/// field's elevation for the gateway asks this first (the apt.dat scan
/// takes a moment). Pure.
pub fn needs_arrival_gateway(legs: &[ProcLeg], destination: &crate::simbrief::Waypoint) -> bool {
    let Some(last) = legs.last().and_then(|l| l.fix.as_ref()) else {
        return false;
    };
    let dist_nm = crate::nav::haversine_nm(last.lat, last.lon, destination.lat, destination.lon);
    dist_nm.is_finite() && dist_nm > ARRIVAL_GATEWAY_NM + 3.0
}

/// The altitude the arrival gateway is crossed at, ft: the route's last
/// altitude, never above the 3 degree path from the field at the gateway's
/// distance. A route that ends remote from its field has no STAR reaching
/// the arrival area, so autoland takes it from the gateway on a straight-in
/// or a reposition pattern, and both want the aircraft near the field at
/// about the pattern's height: on the path is the most a straight-in can
/// use, and a pattern's downwind loses the rest at its own descent.
/// LEGR-LXGB, 2026-10-06 (sortie 1791275171-full_auto, 737): the gateway
/// copied the plan's 12,700 ft, autoland took the arrival 12 nm east of
/// the field at 13,300 ft, and the reposition began 10,100 ft above its
/// platform and extended its downwind over Spain to lose it. `None` for the
/// field's elevation keeps the route's altitude. Pure.
pub fn arrival_gateway_alt_ft(last_alt_ft: Option<f64>, field_elev_ft: Option<f64>) -> Option<f64> {
    let on_path = field_elev_ft
        .filter(|e| e.is_finite())
        .map(|e| e + ARRIVAL_GATEWAY_NM * crate::autoland::GLIDEPATH_FT_PER_NM);
    match (last_alt_ft, on_path) {
        (Some(a), Some(p)) => Some(a.min((p / 100.0).round() * 100.0)),
        (a, None) => a,
        (None, p) => p.map(|p| (p / 100.0).round() * 100.0),
    }
}

/// The gateway leg's ident: synthetic, never a fix an FMS holds.
pub fn arrival_gateway_ident(destination: &str) -> String {
    format!("{destination} arrival")
}

pub fn extend_to_arrival_gateway(
    legs: &mut Vec<ProcLeg>,
    destination: &crate::simbrief::Waypoint,
    field_elev_ft: Option<f64>,
) -> bool {
    if !needs_arrival_gateway(legs, destination) {
        return false;
    }
    let Some(last) = legs.last().and_then(|l| l.fix.as_ref()) else {
        return false;
    };
    let bearing = crate::nav::bearing_deg(destination.lat, destination.lon, last.lat, last.lon);
    let (lat, lon) = crate::nav::project_latlon(
        destination.lat,
        destination.lon,
        bearing,
        ARRIVAL_GATEWAY_NM,
    );
    let alt = arrival_gateway_alt_ft(legs.last().and_then(|l| l.alt1_ft), field_elev_ft);
    legs.push(ProcLeg {
        seq: legs.len() as u32,
        transition: String::new(),
        leg_type: "TF".to_string(),
        fix_ident: arrival_gateway_ident(&destination.ident),
        fix_region: String::new(),
        fix: Some(crate::nav::Fix {
            ident: destination.ident.clone(),
            lat,
            lon,
        }),
        course_mag: None,
        dist_nm: None,
        turn: None,
        alt_desc: ' ',
        alt1_ft: alt,
        alt2_ft: None,
        is_faf: false,
        is_map: false,
        vpa_deg: None,
        navaid_ident: String::new(),
        navaid_region: String::new(),
        navaid: None,
        path: Default::default(),
    });
    true
}

/// Whether the aircraft is positioned to actually FLY this approach — outside
/// the final approach fix and at/above its crossing altitude. A tight circuit at
/// pattern height INSIDE the FAF is not (arming there would command a climb back
/// to platform); those stay with autoland. An enroute arrival at altitude is.
/// The seam that lets the circuit's arrival prefer the real approach only when
/// it makes sense.
pub fn approach_flyable_from(plan: &Plan, lat: f64, lon: f64, alt_ft: f64) -> bool {
    let Some(hi) = handoff_index(&plan.legs) else {
        return false;
    };
    let leg = &plan.legs[hi];
    let Some(fix) = &leg.fix else {
        return false;
    };
    let dist = crate::nav::haversine_nm(lat, lon, fix.lat, fix.lon);
    let faf_alt = leg.alt1_ft.unwrap_or(0.0);
    dist >= 2.0 && alt_ft >= faf_alt - 500.0
}
