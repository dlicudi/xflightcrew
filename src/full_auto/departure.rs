//! The departure: the SID chosen, spliced, requested and checked, the calls,
//! the backtrack, the takeoff clearance and its readback.

use super::*;

/// The crew's own departure from `cleared`, spliced into the route: the
/// SID from that runway that joins the route furthest along and whose
/// minimums this airframe can climb to (`nav::sids_for_route`,
/// `departure_gradients`), else direct to the planned SID's exit, where
/// the follower's ground-ahead floor guards the climb. `Ok` is the line to
/// say; `Err` why neither works.
pub(super) async fn choose_departure(
    config: &Config,
    route: &mut Route,
    cleared: &str,
    vy_kts: f64,
    cleared_sid: Option<String>,
    only_cleared: bool,
) -> Result<Rechosen, String> {
    use crate::nav::ProcKind;
    let wanted = cleared_sid.clone();
    let idents: Vec<String> = route.legs.iter().map(|l| l.fix_ident.clone()).collect();
    let (cifp, apt_path, fix_path, nav_path) = (
        config.cifp_dir.clone(),
        config.apt_data_path.clone(),
        config.fix_data_path.clone(),
        config.nav_data_path.clone(),
    );
    let origin = route.origin_icao.clone();
    let planned = route.departure_rwy.clone();
    let cl = cleared.to_string();
    let best_climb = crate::procedure::best_climb_fpm();
    let found = tokio::task::spawn_blocking(move || {
        let apt = crate::nav::find_airport(&apt_path, &origin).ok().flatten()?;
        let (alat, alon) = apt.position()?;
        let der = apt.runways.iter().find_map(|r| {
            if r.ident_a.eq_ignore_ascii_case(&cl) {
                Some((r.lat_b, r.lon_b))
            } else if r.ident_b.eq_ignore_ascii_case(&cl) {
                Some((r.lat_a, r.lon_a))
            } else {
                None
            }
        })?;
        let choices = prefer_cleared_sid(
            crate::nav::sids_for_route(&cifp, &origin, &cl, &idents),
            wanted.as_deref(),
            only_cleared,
        );
        for choice in choices {
            let mut legs =
                crate::nav::load_proc_legs(&cifp, &origin, ProcKind::Sid, &choice.ident, Some(&choice.transition));
            crate::nav::resolve_proc_fixes(&fix_path, &nav_path, &apt, &origin, &mut legs, alat, alon);
            // The whole chart for the gradient, fixless legs included; the
            // flown copy below keeps only what the follower can steer at.
            let chart = legs.clone();
            let Some(sid) = flown_sid_legs(legs, &choice.join) else {
                continue;
            };
            let mins = crate::nav::sid_minimums(&cifp, &origin, &choice.ident, &cl);
            let checks = departure_gradients(der, apt.elev_ft, &chart, &mins, vy_kts, best_climb);
            match gradient_verdict(&choice.ident, &checks) {
                Ok(note) => return Some(Ok((choice, sid, mins, note))),
                Err(_) => continue,
            }
        }
        // The tower's SID was all we would fly and it does not work from
        // here: say so, the plan stands.
        if only_cleared {
            return Some(Err(None));
        }
        // No SID from this runway to the route: direct to the planned SID's
        // exit. The planned SID is the one the route STARTS on (its first
        // fix is one of the SID's); the furthest-joining SID from that
        // runway is a different departure (LIME 28: SRN1Z, not LEGL1P).
        let exit = planned.as_deref().and_then(|p| {
            let all = crate::nav::sids_for_route(&cifp, &origin, p, &idents);
            let first = idents.first()?;
            all.iter()
                .find(|c| c.fixes.iter().any(|f| f.eq_ignore_ascii_case(first)))
                .or_else(|| all.iter().min_by_key(|c| c.join_index))
                .map(|c| c.join.clone())
        });
        Some(Err(exit))
    })
    .await
    .ok()
    .flatten()
    .ok_or_else(|| "I can't read the departure data for this field.".to_string())?;
    let spoken = crate::crew_radio::spell_runway(cleared);
    match found {
        Ok((choice, sid, mins, note)) => {
            let idx = route
                .legs
                .iter()
                .position(|l| l.fix_ident.eq_ignore_ascii_case(&choice.join))
                .ok_or_else(|| "The new departure doesn't meet the route.".to_string())?;
            let mut legs = sid;
            // The chart's altitudes are constraints, the plan's the levels
            // (#173): re-derived while the plan's own legs are still here.
            rederive_sid_altitudes(&mut legs, &route.legs, &choice.join);
            legs.extend(route.legs.drain(idx + 1..));
            route.legs = legs;
            route.fms_payload = payload_with_legs(&route.fms_payload, &route.legs);
            route.climb_minimums = mins;
            route.departure_rwy = Some(cleared.to_string());
            route.origin_rwy = Some(cleared.to_string());
            route.sid = Some(choice.ident.clone());
            let source = departure_source(Some(&choice.ident), cleared_sid.as_deref());
            let note = note.map(|n| format!(" {n}")).unwrap_or_default();
            let line = if source == "cleared" {
                format!(
                    "Cleared via the {} departure, runway {spoken}, to {}.{note}",
                    crate::tower::spoken_sid(&choice.ident),
                    choice.join,
                )
            } else {
                format!(
                    "Runway {spoken} in use, so the {} departure to {}.{note}",
                    choice.ident, choice.join,
                )
            };
            Ok(Rechosen { line, source, sid: Some(choice.ident.clone()) })
        }
        Err(Some(exit)) => {
            let idx = route
                .legs
                .iter()
                .position(|l| l.fix_ident.eq_ignore_ascii_case(&exit))
                .ok_or_else(|| "The planned departure doesn't meet the route.".to_string())?;
            route.legs.drain(..idx);
            route.fms_payload = payload_with_legs(&route.fms_payload, &route.legs);
            route.climb_minimums = Vec::new();
            route.departure_rwy = Some(cleared.to_string());
            route.origin_rwy = Some(cleared.to_string());
            route.sid = None;
            Ok(Rechosen {
                line: format!(
                    "Runway {spoken} in use and no departure from it joins our route, so direct to {exit}, watching the terrain."
                ),
                source: "chosen",
                sid: None,
            })
        }
        Err(None) if only_cleared => Err("The cleared departure doesn't join our route.".to_string()),
        Err(None) => Err("No departure from that runway joins our route.".to_string()),
    }
}

/// The SID legs the follower flies, up to `join`: its fix legs, the runway
/// and the fixless legs left out. `None` when a fix is unplaced or the
/// chart never reaches `join`. Pure.
pub fn flown_sid_legs(
    legs: Vec<crate::nav::ProcLeg>,
    join: &str,
) -> Option<Vec<crate::nav::ProcLeg>> {
    let mut sid: Vec<crate::nav::ProcLeg> = Vec::new();
    for l in legs {
        if l.fix_ident.is_empty() || l.fix_ident.starts_with("RW") {
            continue;
        }
        l.fix.as_ref()?;
        let last = l.fix_ident.eq_ignore_ascii_case(join);
        sid.push(l);
        if last {
            return Some(sid);
        }
    }
    None
}

/// A chart leg's own ceiling: an "at or below" (`-`), or a `B` window's
/// upper edge (ARINC 424 codes it first). "At" and "at or above" bound
/// nothing from above once departure clears higher: they are minimums
/// (`nav::sid_minimums`), held as floors by `DepartureMinimums`.
pub(super) fn sid_leg_ceiling_ft(l: &crate::nav::ProcLeg) -> Option<f64> {
    match l.alt_desc {
        '-' | 'B' => l.alt1_ft,
        _ => None,
    }
}

/// Each leg's altitude lowered to the tightest ceiling at it or further on
/// (`own[i]`, [`sid_leg_ceiling_ft`] of the chart's leg `i`): a fix to be
/// crossed at or below a level is never approached above it. Pure.
pub(super) fn apply_sid_ceilings(legs: &mut [crate::nav::ProcLeg], own: &[Option<f64>]) {
    let mut ceiling: Option<f64> = None;
    for (leg, c) in legs.iter_mut().zip(own).rev() {
        if let Some(c) = c {
            ceiling = Some(ceiling.map_or(*c, |x: f64| x.min(*c)));
        }
        if let (Some(c), Some(a)) = (ceiling, leg.alt1_ft) {
            if a > c {
                leg.alt1_ft = Some(c);
                leg.alt_desc = '-';
            }
        }
    }
}

/// A SID the crew spliced in (`choose_departure`) flown at the PLAN's
/// levels, the chart's altitudes kept only as constraints (#173): a fix the
/// plan also has keeps the plan's altitude, any other the plan's altitude at
/// `join`, and only the chart's "at or below"s lower them
/// ([`apply_sid_ceilings`]); "at" and "at or above" are the departure's
/// minimums (`climb_minimums`). EGKK 08R (sortie 1790624015-full_auto,
/// 2026-09-28): the cleared LAM5P's coded "at 5,000" at ACORN and D156O
/// became the plan's level, the 6,000 minimum at LAM floored it, and the
/// follower held 6,000 to LAM after departure cleared FL180 (the filed
/// plan: 16,000 at ACORN, 18,000 by D156O). Unchanged without `join` in the
/// plan. Pure.
pub fn rederive_sid_altitudes(
    sid: &mut [crate::nav::ProcLeg],
    plan: &[crate::nav::ProcLeg],
    join: &str,
) {
    let Some(join_plan) = plan.iter().find(|l| l.fix_ident.eq_ignore_ascii_case(join)) else {
        return;
    };
    let own: Vec<Option<f64>> = sid.iter().map(sid_leg_ceiling_ft).collect();
    for leg in sid.iter_mut() {
        match plan.iter().find(|p| p.fix_ident.eq_ignore_ascii_case(&leg.fix_ident)) {
            Some(p) => {
                leg.alt1_ft = p.alt1_ft;
                leg.alt_desc = p.alt_desc;
            }
            None => {
                leg.alt1_ft = join_plan.alt1_ft;
                leg.alt_desc = ' ';
            }
        }
        leg.alt2_ft = None;
    }
    apply_sid_ceilings(sid, &own);
}

/// A departure the crew spliced into the route: the line to say and whose
/// pick it was ("cleared": the tower's SID; "chosen": the crew's own).
#[derive(Debug, Clone, PartialEq)]
pub struct Rechosen {
    pub line: String,
    pub source: &'static str,
    /// The SID flown; `None` direct to the planned SID's exit.
    pub sid: Option<String>,
}

/// What the crew does about the tower's SID, once its own departure is
/// chosen (issue #152: at EGKK 08R the tower cleared KENE3P and the crew
/// flew LAM5P without a word on the radio). Pure.
#[derive(Debug, Clone, PartialEq)]
pub enum ClearedSidPlan {
    /// Fly what was chosen: it is the tower's SID, or there is no IFR SID.
    Fly,
    /// Ask the tower for this SID before flying it.
    Request(String),
    /// The tower's SID cannot be flown and there is no SID to ask for.
    Cannot,
}

/// [`ClearedSidPlan`] for a flown SID (`None`: direct to the exit, or the
/// plan's own departure) against the IFR clearance's SID. Pure.
pub fn cleared_sid_plan(flown: Option<&str>, cleared: Option<&str>) -> ClearedSidPlan {
    match (cleared, flown) {
        (None, _) => ClearedSidPlan::Fly,
        (Some(c), Some(f)) if f.eq_ignore_ascii_case(c) => ClearedSidPlan::Fly,
        (Some(_), Some(f)) => ClearedSidPlan::Request(f.to_string()),
        (Some(_), None) => ClearedSidPlan::Cannot,
    }
}

/// The crew's call asking for another SID: "..., unable Kene three Papa,
/// request the Lam five Papa departure" (`tower::requested_departure` reads
/// it back). Pure.
pub fn sid_request_call(callsign: &Option<String>, cleared: Option<&str>, want: &str) -> String {
    let unable = cleared
        .map(|c| format!("unable {}, ", crate::tower::spoken_sid(c)))
        .unwrap_or_default();
    crate::crew_radio::radio_call(
        callsign,
        &format!("{unable}request the {} departure", crate::tower::spoken_sid(want)),
    )
}

/// Ask the tower for `want` in place of the cleared SID, and wait for the
/// held clearance to name it (`party_line::assigned_clearance`). True when
/// it does; blackbox `full_auto` `sid_requested` either way.
pub(super) async fn request_sid(callsign: &Option<String>, cleared: Option<&str>, want: &str) -> bool {
    radio_out(sid_request_call(callsign, cleared, want));
    waiting("the tower to approve the departure");
    let named = |c: Option<crate::party_line::IfrClearance>| {
        c.and_then(|c| c.sid).is_some_and(|s| s.eq_ignore_ascii_case(want))
    };
    let approved = wait_until(60.0, 500, || async {
        named(crate::party_line::assigned_clearance())
    })
    .await;
    event(
        "sid_requested",
        json!({ "cleared": cleared, "asked": want, "approved": approved }),
    );
    approved
}

/// The SIDs to try, the tower's first: the clearance's SID moved to the
/// front when it is among them (it serves the runway and joins the route),
/// the crew's own order after it. `only` keeps the tower's alone (the
/// planned runway stands and only the SID changed: a crew never swaps a
/// plan's SID for one of its own). No clearance: the order unchanged. Pure.
pub fn prefer_cleared_sid(
    mut choices: Vec<crate::nav::SidChoice>,
    cleared: Option<&str>,
    only: bool,
) -> Vec<crate::nav::SidChoice> {
    let Some(c) = cleared else {
        return choices;
    };
    match choices.iter().position(|x| x.ident.eq_ignore_ascii_case(c)) {
        Some(i) => {
            let x = choices.remove(i);
            if only {
                return vec![x];
            }
            choices.insert(0, x);
            choices
        }
        None if only => Vec::new(),
        None => choices,
    }
}

/// Whose departure is flown: "cleared" when it is the tower's SID, else
/// "chosen". Pure.
pub fn departure_source(flown: Option<&str>, cleared: Option<&str>) -> &'static str {
    match (flown, cleared) {
        (Some(f), Some(c)) if f.eq_ignore_ascii_case(c) => "cleared",
        _ => "chosen",
    }
}

/// The SID the IFR clearance hands the crew for the runway it was cleared
/// on: the clearance's SID when its runway is the cleared one (with no
/// runway clearance yet, its own runway). `None` with no clearance, no SID
/// in it (runway heading), or a runway since changed. Pure.
pub fn cleared_sid_for(
    clearance: Option<&crate::party_line::IfrClearance>,
    cleared_runway: Option<&str>,
) -> Option<String> {
    let c = clearance?;
    let sid = c.sid.clone()?;
    match cleared_runway {
        Some(r) if !r.eq_ignore_ascii_case(&c.runway) => None,
        _ => Some(sid),
    }
}

/// The GPS payload for `legs`, keeping the old payload's first and last
/// entries (the origin and destination fields that fill the CDU's
/// ORIGIN/DEST). The old payload unchanged when it has no ends to keep.
pub fn payload_with_legs(old: &str, legs: &[crate::nav::ProcLeg]) -> String {
    let point = |e: &str| {
        let f: Vec<&str> = e.split(',').collect();
        Some(crate::simbrief::Waypoint {
            ident: f.first()?.to_string(),
            lat: f.get(1)?.parse().ok()?,
            lon: f.get(2)?.parse().ok()?,
            alt: f.get(3).and_then(|a| a.parse().ok()).unwrap_or(0.0),
        })
    };
    let entries: Vec<&str> = old.split('|').collect();
    let (Some(first), Some(last)) = (
        entries.first().and_then(|e| point(e)),
        entries.last().and_then(|e| point(e)),
    ) else {
        return old.to_string();
    };
    if entries.len() < 2 {
        return old.to_string();
    }
    let mut ws = vec![first];
    ws.extend(legs.iter().filter_map(|l| {
        l.fix.as_ref().map(|f| crate::simbrief::Waypoint {
            ident: l.fix_ident.clone(),
            lat: f.lat,
            lon: f.lon,
            alt: l.alt1_ft.unwrap_or(0.0),
        })
    }));
    ws.push(last);
    crate::simbrief::fms_route_payload(&ws)
}

/// One departure minimum judged against the climb: the gradient it needs from
/// the departure end of the runway, and what the airframe makes at Vy.
#[derive(Debug, Clone, PartialEq)]
pub struct GradientCheck {
    pub fix: String,
    pub min_ft: f64,
    pub dist_nm: f64,
    pub need_ft_per_nm: f64,
    pub have_ft_per_nm: f64,
}

/// Refused above this share of the airframe's climb gradient...
pub(super) const GRADIENT_REFUSE_FRAC: f64 = 0.90;
/// ...said as a caution above this.
pub(super) const GRADIENT_CAUTION_FRAC: f64 = 0.75;

/// Every departure minimum on the route, from the runway's departure end
/// `der` at `field_ft`, against `best_climb_fpm` at `vy_kts` (true airspeed
/// rising 2 % a thousand feet, no wind credit). Pure.
/// The departure as the CHART flies it, up to `join` (a fix both the chart
/// and `plan` have), ready for the follower: fix legs, course or heading to
/// an altitude (CA, VA) and course to a DME distance (CD, VD). The plan
/// lists only a SID's named fixes, so ANC8A's climb out on 304 and the 6-mile
/// out-and-back to RIM were flown as "direct RIM", back over the field
/// (LIPR 2026-09-26). `None` when the chart has a leg the follower cannot
/// fly (arcs, holds, vectors) or a fix or navaid it cannot place: then the
/// plan's own fixes are flown, as before.
///
/// A chart altitude is a MINIMUM, not a level: the minimums are guarded by
/// the departure's `climb_minimums`. So a fix the plan also has keeps the
/// plan's altitude; a climb-to-altitude leg keeps its own (it is where the
/// leg ends); everything else takes the plan's altitude at the join, for one
/// continuous climb. Pure.
pub fn charted_departure_legs(
    chart: &[crate::nav::ProcLeg],
    plan: &[crate::nav::ProcLeg],
    join: &str,
) -> Option<Vec<crate::nav::ProcLeg>> {
    let join_plan = plan.iter().find(|l| l.fix_ident.eq_ignore_ascii_case(join))?;
    let mut out = Vec::new();
    let mut own = Vec::new();
    for l in chart {
        let mut leg = l.clone();
        let ident = l.fix_ident.to_ascii_uppercase();
        if ident.starts_with("RW") {
            continue; // the runway threshold the roll starts from
        }
        match l.leg_type.as_str() {
            "IF" | "TF" | "CF" | "DF" => {
                l.fix.as_ref()?;
                match plan.iter().find(|p| p.fix_ident.eq_ignore_ascii_case(&l.fix_ident)) {
                    Some(p) => {
                        leg.alt1_ft = p.alt1_ft;
                        leg.alt_desc = p.alt_desc;
                    }
                    None => {
                        leg.alt1_ft = join_plan.alt1_ft;
                        leg.alt_desc = ' ';
                    }
                }
            }
            "CA" | "VA" => {
                l.course_mag?;
                l.alt1_ft?;
                leg.fix = None;
                leg.fix_ident.clear();
            }
            "CD" | "VD" => {
                l.course_mag?;
                l.navaid.as_ref()?;
                l.dist_nm.filter(|d| *d > 0.0)?;
                leg.fix = None;
                leg.fix_ident.clear();
                leg.alt1_ft = join_plan.alt1_ft;
                leg.alt_desc = ' ';
            }
            _ => return None,
        }
        let last = l.fix_ident.eq_ignore_ascii_case(join);
        out.push(leg);
        own.push(sid_leg_ceiling_ft(l));
        if last {
            // ...and the chart's "at or below"s still bound it (#173).
            apply_sid_ceilings(&mut out, &own);
            return Some(out);
        }
    }
    None
}

/// SID `sid` from `rwy` as CHARTED: its runway transition and common legs,
/// the climb-to-altitude and course legs included, fixes resolved. What the
/// gradient check must walk: the flight plan lists only the SID's named
/// fixes, and at LIPR the one fix with a minimum is a VOR on the field that
/// the procedure reaches after a 12-mile out-and-back. Empty when the chart
/// or the airport is missing (the caller falls back to the plan's legs).
pub fn sid_chart_legs(
    cifp_dir: &str,
    fix_path: &str,
    nav_path: &str,
    apt: &crate::nav::AirportInfo,
    origin: &str,
    sid: &str,
    rwy: &str,
) -> Vec<crate::nav::ProcLeg> {
    let Some((alat, alon)) = apt.position() else {
        return Vec::new();
    };
    let tr = format!("RW{rwy}");
    let mut legs = crate::nav::load_proc_legs(cifp_dir, origin, crate::nav::ProcKind::Sid, sid, Some(&tr));
    crate::nav::resolve_proc_fixes(fix_path, nav_path, apt, origin, &mut legs, alat, alon);
    legs
}

pub fn departure_gradients(
    der: (f64, f64),
    field_ft: f64,
    legs: &[crate::nav::ProcLeg],
    minimums: &[crate::nav::ClimbMinimum],
    vy_kts: f64,
    best_climb_fpm: f64,
) -> Vec<GradientCheck> {
    let mut out = Vec::new();
    let mut dist = 0.0;
    let mut from = der;
    // Distance is measured ALONG the procedure. Only the legs with a fix used
    // to count, straight from one fix to the next, so a departure that flies
    // out and turns back to a VOR on the field measured as the mile between
    // them: LIPR's ANC8A (climb to 450 ft on 304, the 291 course to 6 DME,
    // back 6 nm to RIM at 4,000 or above) read as 3,388 feet a mile for a
    // 172 that climbs 569, and the flight was refused (2026-09-26). A leg's
    // own published distance wins; a climb-to-altitude leg is as long as
    // this airframe takes to reach the altitude; a fix-to-fix leg with no
    // distance is the straight line.
    let low_tas = vy_kts * (1.0 + 0.02 * field_ft / 1000.0);
    let low_have = best_climb_fpm / (low_tas / 60.0);
    let mut alt_now = field_ft;
    for leg in legs {
        let leg_nm = match (leg.dist_nm.filter(|d| *d > 0.0), &leg.fix) {
            (Some(d), fix) => {
                if let Some(f) = fix {
                    from = (f.lat, f.lon);
                }
                d
            }
            (None, Some(f)) => {
                let d = crate::nav::haversine_nm(from.0, from.1, f.lat, f.lon);
                from = (f.lat, f.lon);
                d
            }
            (None, None) => match leg.alt1_ft {
                Some(a) if matches!(leg.leg_type.as_str(), "CA" | "VA" | "FA") => {
                    (a - alt_now).max(0.0) / low_have.max(1.0)
                }
                _ => 0.0,
            },
        };
        dist += leg_nm;
        alt_now = alt_now.max(leg.alt1_ft.unwrap_or(alt_now).min(alt_now + leg_nm * low_have));
        if leg.fix.is_none() {
            continue;
        }
        if let Some(m) = minimums.iter().find(|m| m.fix.eq_ignore_ascii_case(&leg.fix_ident)) {
            let mid_kft = (field_ft + m.min_ft) / 2.0 / 1000.0;
            let tas = vy_kts * (1.0 + 0.02 * mid_kft);
            out.push(GradientCheck {
                fix: m.fix.clone(),
                min_ft: m.min_ft,
                dist_nm: dist,
                need_ft_per_nm: (m.min_ft - field_ft).max(0.0) / dist.max(0.1),
                have_ft_per_nm: best_climb_fpm / (tas / 60.0),
            });
        }
    }
    out
}

/// The departure's verdict: `Err` a refusal naming the binding minimum,
/// `Ok(Some(line))` a caution for the brief, `Ok(None)` comfortable.
pub fn gradient_verdict(sid: &str, checks: &[GradientCheck]) -> Result<Option<String>, String> {
    let Some(worst) = checks
        .iter()
        .max_by(|a, b| (a.need_ft_per_nm / a.have_ft_per_nm).total_cmp(&(b.need_ft_per_nm / b.have_ft_per_nm)))
    else {
        return Ok(None);
    };
    let share = worst.need_ft_per_nm / worst.have_ft_per_nm.max(1.0);
    let line = format!(
        "{sid} needs {} at {}, {:.0} feet a mile; we climb about {:.0} at best rate.",
        crate::flightplan::spoken_feet(worst.min_ft),
        worst.fix,
        worst.need_ft_per_nm,
        worst.have_ft_per_nm
    );
    if share > GRADIENT_REFUSE_FRAC {
        Err(format!("Negative. {line} Not flying that departure."))
    } else if share > GRADIENT_CAUTION_FRAC {
        Ok(Some(format!("{line} Tight, I'll climb at Vy until it's made.")))
    } else {
        Ok(Some(format!("{line} I'll climb at Vy until it's made.")))
    }
}

/// The most tailwind an arrival accepts to fly a published approach rather
/// than a visual landing.
pub(super) const APPROACH_TAILWIND_MAX_KTS: f64 = 10.0;

/// The other hard runway ends, most headwind first, whose tailwind is within
/// [`APPROACH_TAILWIND_MAX_KTS`]. With no wind every end qualifies.
pub(super) fn approach_fallback_ends(
    airport: &crate::nav::AirportInfo,
    tried: &str,
    wind: Option<(f64, f64)>,
) -> Vec<String> {
    let mut ends: Vec<(String, f64)> = crate::nav::assignable_runways(&airport.runways)
        .flat_map(|r| [r.ident_a.clone(), r.ident_b.clone()])
        .filter(|id| id != tried)
        .map(|id| {
            let hw = wind
                .and_then(|(d, k)| crate::nav::headwind_for(&airport.runways, &id, d, k))
                .unwrap_or(0.0);
            (id, hw)
        })
        .filter(|(_, hw)| *hw >= -APPROACH_TAILWIND_MAX_KTS)
        .collect();
    ends.sort_by(|a, b| b.1.total_cmp(&a.1));
    ends.into_iter().map(|(id, _)| id).collect()
}

/// A circuit never obeys a departure handoff: the pattern comes back to the
/// tower's runway. One that reached the party line anyway (a station that
/// did not hear "for circuits") is drained and blackboxed, never left for
/// the next mission's climb to retune on.
pub(super) fn drop_departure_handoff(at: &str) {
    if let Some((to, khz)) = crate::party_line::take_departure_handoff() {
        event(
            "departure_handoff_ignored",
            json!({ "at": at, "to": to, "khz": khz }),
        );
    }
}

/// Is this mission a circuit? The tower keeps pattern traffic, so the crew
/// says so on every departure request (`tower::wants_circuits`).
pub(super) fn flying_circuits() -> bool {
    MISSION.lock().map(|m| *m == "circuit").unwrap_or(false)
}

/// The departure-readiness call: "for circuits" on a circuit, so the tower
/// keeps the flight instead of handing it to Departure on the crosswind leg
/// (LEMG 31, sortie 1790584715, 2026-09-28).
pub(super) fn ready_call(circuits: bool) -> &'static str {
    if circuits {
        "ready for departure, for circuits"
    } else {
        "ready for departure"
    }
}

/// The taxi request: a planned departure asks for its runway (a SID exists
/// for one runway, and in calm air the pick could have been the other); a
/// circuit says it is one, since the tower keeps pattern traffic. A runway
/// the IFR clearance already named (`cleared`) is the one asked for: EGKK
/// (sortie 1790601822-full_auto, 2026-09-28, #155) was cleared "via the Kene
/// three Papa departure, runway zero eight right" and then asked "request
/// taxi, runway two six left", the plan's runway.
pub(super) fn taxi_call(mission: &Mission, cleared: Option<&str>) -> String {
    match mission {
        Mission::Flight(r) => cleared
            .filter(|c| !c.is_empty())
            .or(r.departure_rwy.as_deref())
            .map(|rwy| {
                format!(
                    "request taxi, runway {}",
                    crate::crew_radio::spell_runway(rwy)
                )
            })
            .unwrap_or_else(|| "request taxi".to_string()),
        Mission::Circuit { .. } => "request taxi for circuits".to_string(),
        Mission::Resume(_) => "request taxi".to_string(),
    }
}

/// Call the tower for departure and wait for it to move us to a departure
/// clearance (line-up-and-wait or cleared for takeoff). The call itself is
/// voiced + transcribed by the crew-radio drain, along with the tower's reply.
/// A runway start (`on_runway`) stops waiting as soon as the answer is for
/// another runway ([`cleared_elsewhere`]): a taxi is not coming as a line-up.
pub(super) async fn request_departure(callsign: &Option<String>, on_runway: Option<&str>) {
    use crate::tower::State::{LinedUp, TakeoffCleared};
    // The call itself is heard; a line announcing it first is a crew narrating
    // its own hands (LIPK 2026-09-26: said, queued behind the tower, and
    // dropped stale, or heard after the call it announced).
    radio_out(crate::crew_radio::radio_call(
        callsign,
        ready_call(flying_circuits()),
    ));
    event("request_departure", json!({}));
    waiting("departure clearance");
    wait_until(20.0, 1000, || async {
        matches!(
            crate::party_line::clearance_state(),
            Some(LinedUp) | Some(TakeoffCleared)
        ) || cleared_elsewhere(
            on_runway,
            crate::party_line::clearance_state(),
            crate::party_line::clearance_runway().as_deref(),
        )
        .is_some()
    })
    .await;
}

/// A RUNWAY START IS NEVER CLEARED FROM A RUNWAY IT IS NOT ON. Standing on
/// `on`, the tower's taxi, line-up or takeoff for another runway (the
/// reciprocal of our own strip included) is that runway, returned: the crew
/// taxis to it and never reads back a line-up or a takeoff it is not in
/// position for. LEMG (sortie 1790631369-full_auto, 2026-09-28, #178): lined
/// up on 30, the ready call was answered "taxi to holding point Hotel
/// November one runway one three first", the crew sat, read back "lining up
/// runway one three" and "cleared for takeoff runway one three" from 30, and
/// said "Lined up and ready" until autotakeoff's wrong_runway stopped it. A
/// zone clearance alone (Parked) is judged at the arm (`under_clearance`).
/// Pure.
pub(crate) fn cleared_elsewhere(
    on: Option<&str>,
    state: Option<crate::tower::State>,
    cleared: Option<&str>,
) -> Option<String> {
    use crate::tower::State::{LinedUp, TakeoffCleared, TaxiCleared};
    let on = on.filter(|r| !r.is_empty())?;
    let cleared = cleared.filter(|r| !r.is_empty())?;
    (matches!(state, Some(TaxiCleared | LinedUp | TakeoffCleared))
        && !crate::autotakeoff::runway_matches(on, cleared))
    .then(|| cleared.to_string())
}

/// What the crew says when a runway start is sent to another runway: where
/// it is, where it goes, and that it taxis (never "lined up"). Pure.
pub(crate) fn other_runway_line(on: &str, cleared: &str) -> String {
    format!(
        "The tower wants runway {}, and we're on {}. Taxiing there.",
        crate::atis::spoken_runway(cleared),
        crate::atis::spoken_runway(on)
    )
}

/// Whether `cleared` is the other end of the strip `on` ("13" and "31",
/// "08R" and "26L"): the runway start's backtrack, not a taxi. Pure.
pub(crate) fn is_reciprocal(on: &str, cleared: &str) -> bool {
    crate::party_line::reciprocal_end(on.trim())
        .is_some_and(|r| crate::autotakeoff::runway_matches(&r, cleared.trim()))
}

/// What the crew says when a runway start is cleared from the far end of its
/// own strip: it backtracks down it. Pure.
pub(crate) fn backtrack_line(on: &str, cleared: &str) -> String {
    format!(
        "The tower wants runway {}, and we're on {}, the same strip. Asking to backtrack.",
        crate::atis::spoken_runway(cleared),
        crate::atis::spoken_runway(on)
    )
}

/// How a wait for the takeoff clearance ended ([`ensure_takeoff_cleared`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum ClearanceWait {
    /// Cleared (or, held short, lined up) and read back.
    Cleared,
    /// No clearance in the wait, or the pilot's cancel.
    NotCleared,
    /// A runway start answered for another runway ([`cleared_elsewhere`]):
    /// nothing was read back, the caller taxis there.
    OtherRunway(String),
}

/// How long one backtrack request waits for the tower's line-up before it
/// is asked again (a hold for traffic on final can run minutes).
pub(super) const BACKTRACK_WAIT_SECS: f64 = 60.0;
/// Asks before full auto gives the aircraft back on the runway.
pub(super) const BACKTRACK_ASKS: usize = 5;

/// A runway start with too little runway ahead (issue #128): ask the tower
/// to backtrack, read its line-up back, and drive the line-up tool's own
/// backtrack to the threshold and round onto the centreline. No tower (no
/// simulated ATC, none in reach) leaves it to the crew's judgment, as the
/// line-up tool does. `Err((line, why))` is the give-up.
pub(super) async fn backtrack_from_runway_start(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    tools: &SubTools,
    callsign: &Option<String>,
    runway: &str,
    towered: bool,
) -> Result<(), (String, &'static str)> {
    use crate::tower::State::{LinedUp, TakeoffCleared};
    if towered {
        if let Some(khz) = crate::party_line::home_tower_khz() {
            let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
        }
        let call = format!(
            "request backtrack runway {}",
            crate::crew_radio::spell_runway(runway)
        );
        let mut approved = false;
        for attempt in 0..BACKTRACK_ASKS {
            radio_out(crate::crew_radio::radio_call(callsign, &call));
            event(
                "request_backtrack",
                json!({ "runway": runway, "attempt": attempt }),
            );
            waiting("the backtrack approval");
            approved = wait_until(BACKTRACK_WAIT_SECS, 1000, || async {
                matches!(
                    crate::party_line::clearance_state(),
                    Some(LinedUp) | Some(TakeoffCleared)
                )
            })
            .await;
            if approved || ABORT.load(SeqCst) {
                break;
            }
            // Held for traffic: "hold position" is read back like any
            // instruction, then asked again once it has had its minute.
            if let Some(rb) = crate::party_line::readback_owed() {
                radio_readback(rb.trim_end_matches('.').to_string(), true);
            }
        }
        if ABORT.load(SeqCst) {
            return Ok(());
        }
        if !approved {
            return Err((
                "The tower never let us backtrack, and there isn't the runway ahead to go from here. \
                 Your aircraft."
                    .to_string(),
                "no_backtrack_approval",
            ));
        }
        if let Some(rb) = crate::party_line::readback_owed() {
            radio_readback(rb.trim_end_matches('.').to_string(), true);
            event("backtrack_readback", json!({}));
        }
    }
    if let Ok(reply) = tools
        .line_up
        .execute(&json!({ "backtrack": true }), ctx)
        .await
    {
        announce(voice, Priority::Advisory, &reply);
    }
    waiting("the backtrack");
    if crate::autotaxi::driving()
        && !wait_for_drive().await
    {
        return Err((
            "The backtrack didn't finish. Your aircraft.".to_string(),
            "backtrack_stalled",
        ));
    }
    if ABORT.load(SeqCst) {
        return Ok(());
    }
    match runway_check(xp, tools).await {
        Ok(()) => {
            event("backtracked", json!({ "runway": runway }));
            Ok(())
        }
        Err(why) => {
            event("backtrack_no_fit", json!({ "why": format!("{why:?}") }));
            Err((
                match &why {
                    Some(w) => format!(
                        "{} Your aircraft.",
                        crate::autotakeoff::no_fit_line(w, false).0
                    ),
                    None => "The backtrack didn't put us on the runway. Your aircraft.".to_string(),
                },
                "backtrack_no_fit",
            ))
        }
    }
}

/// When the crew reads back an instruction the tower is waiting on, judged
/// against the crew radio's own record of its last readback
/// (`crew_radio::last_readback`), whoever queued it: never while the same words
/// are queued and not yet out, and again only if the tower still waits
/// [`OwedReadback::AGAIN`] after they WENT OUT (a readback dropped stale on
/// the radio). LEMG 13 (2026-09-28, sortie 1790627884-full_auto): the
/// clearance's readback queued at +13.7 was still queued at +34.0, this
/// queued it again, and the copy went out in the takeoff roll. Pure.
pub(super) struct OwedReadback;

impl OwedReadback {
    pub(super) const AGAIN: Duration = Duration::from_secs(30);

    /// The readback to transmit now, if any.
    pub(super) fn due(
        owed: Option<&str>,
        last: Option<&crate::crew_radio::ReadbackRec>,
        now: Instant,
    ) -> Option<String> {
        let owed = owed?;
        let words = crate::crew_radio::readback_words(owed);
        let again = match last.filter(|r| r.words == words) {
            None => true,
            Some(r) => r
                .out_at
                .is_some_and(|out| now.saturating_duration_since(out) >= Self::AGAIN),
        };
        again.then(|| owed.to_string())
    }
}

/// Wait until the tower actually clears the takeoff, through a traffic hold if
/// that's what it takes (a final can be minutes out). Re-report "ready for
/// departure" periodically while merely waiting, but stay SILENT while
/// explicitly held for traffic: the tower said hold, and nagging a held
/// clearance is wrong. Reads the clearance back ONCE across calls
/// (`read_back`); the tower enforces the runway-critical readback. Returns
/// whether we ended up cleared; abortable throughout.
///
/// `accept_line_up` is the difference between the two calls this makes. Held
/// SHORT of the runway, a line-up-and-wait is the answer we're waiting for —
/// the tower has sequenced us and wants us on the strip, so we take it, read
/// it back, and drive on (the caller asks again from the runway). Already ON
/// the runway, only the takeoff clearance will do. Without the distinction a
/// sequenced departure waited out its whole budget behind the hold line while
/// the tower believed it was lined up (Heathrow 27L, 2026-07-27).
///
/// `on_runway` is a runway start's own runway: an answer for another one
/// ends the wait unread ([`ClearanceWait::OtherRunway`]).
pub(super) async fn ensure_takeoff_cleared(
    voice: &Option<Voice>,
    callsign: &Option<String>,
    read_back: &mut bool,
    accept_line_up: bool,
    on_runway: Option<&str>,
) -> ClearanceWait {
    use crate::tower::State::{LinedUp, TakeoffCleared};
    /// Wait this long overall for the clearance — long enough to sit out a
    /// traffic hold, bounded so a dead frequency eventually hands back.
    const OVERALL_SECS: f64 = 480.0;
    /// Re-report "ready" this often while lined up and NOT held for traffic.
    const NUDGE_EVERY: Duration = Duration::from_secs(40);
    /// Held for traffic, ask again after this long without a word: a real
    /// hold lasts a landing or two, and the tower keeps its own promise
    /// well inside it (the world's `takeoff_hold_tick`).
    const HELD_REASK: Duration = Duration::from_secs(150);

    let deadline = Instant::now() + Duration::from_secs_f64(OVERALL_SECS);
    let mut last_call = Instant::now(); // request_departure just called
    let mut announced_hold = false;
    loop {
        if ABORT.load(SeqCst) {
            return ClearanceWait::NotCleared;
        }
        let state = crate::party_line::clearance_state();
        // Before anything is read back or called: an answer for another
        // runway is not ours to read back from this one (#178).
        if let Some(other) = cleared_elsewhere(
            on_runway,
            state,
            crate::party_line::clearance_runway().as_deref(),
        ) {
            return ClearanceWait::OtherRunway(other);
        }
        if matches!(state, Some(TakeoffCleared))
            || (accept_line_up && matches!(state, Some(LinedUp)))
        {
            break;
        }
        if Instant::now() >= deadline {
            return ClearanceWait::NotCleared;
        }
        // An instruction the tower wants read back ("hold position") is
        // read back before anything else is said: the ready call repeated
        // over it earned "read back" three times (LEMG 13, #172).
        let owed = crate::party_line::readback_owed();
        let last = crate::crew_radio::last_readback();
        if let Some(rb) = OwedReadback::due(owed.as_deref(), last.as_ref(), Instant::now()) {
            radio_readback(rb.trim_end_matches('.').to_string(), true);
            event("readback_hold", json!({ "call": rb }));
            last_call = Instant::now();
        } else if owed.is_some() {
            // Owed and just read back: wait for the tower to hear it.
        } else if crate::party_line::takeoff_hold_armed() {
            // Held, and a promise is only as good as its keeping: past
            // HELD_REASK the crew asks again, so a hold that outlived its
            // traffic is never waited out in silence (LEMG 13, sortie
            // 1790623420-full_auto: "expect clearance after the landing
            // traffic", then 5 min of nothing and no_takeoff_clearance).
            if last_call.elapsed() >= HELD_REASK {
                event(
                    "reask_held",
                    json!({ "why": "held", "since_call_s": last_call.elapsed().as_secs() }),
                );
                radio_out(crate::crew_radio::radio_call(
                    callsign,
                    ready_call(flying_circuits()),
                ));
                last_call = Instant::now();
            }
            // Held for traffic: wait it out, silently. Say so once.
            if !announced_hold {
                announce(
                    voice,
                    Priority::Advisory,
                    crate::readback::one_of(&[
                        "Holding, the tower has traffic. I'll wait.",
                        "Tower's got traffic. Holding here.",
                        "We've got traffic ahead. Holding for now.",
                    ]),
                );
                announced_hold = true;
            }
        } else if last_call.elapsed() >= NUDGE_EVERY {
            radio_out(crate::crew_radio::radio_call(
                callsign,
                ready_call(flying_circuits()),
            ));
            last_call = Instant::now();
        }
        tokio::time::sleep(Duration::from_millis(1000)).await;
    }
    // Read back what we were actually given. A line-up-and-wait is its own
    // runway-critical item — echoing "cleared for takeoff" at it would earn a
    // "negative" from the readback enforcement and, worse, is not what we
    // were told to do. The takeoff readback stays one-shot across the whole
    // departure sequence; the line-up's is owed at the moment it's issued.
    let lined_up_only = matches!(crate::party_line::clearance_state(), Some(LinedUp));
    if let Some(rwy) = crate::party_line::clearance_runway() {
        let spelled = crate::crew_radio::spell_runway(&rwy);
        let sent = if lined_up_only {
            let call =
                crate::crew_radio::radio_call(callsign, &format!("lining up runway {spelled}"));
            radio_readback(call.clone(), false);
            event("readback_line_up", json!({ "runway": rwy }));
            Some(call)
        } else if !*read_back {
            *read_back = true;
            let call = crate::crew_radio::radio_call(
                callsign,
                &format!("cleared for takeoff runway {spelled}"),
            );
            radio_readback(call.clone(), false);
            event("readback_takeoff", json!({ "runway": rwy }));
            Some(call)
        } else {
            None
        };
        // Heard, read back, THEN moved: the roll began at +52.8 on the
        // tower's state, before its "cleared for takeoff" was spoken at
        // +53.5, and the readback went out at +71.2 in the roll (LEMG 13,
        // 2026-09-28, sortie 1790627884-full_auto). The readback rides the
        // crew radio behind the clearance it answers, so once it is out the
        // clearance has been heard.
        if let Some(call) = sent {
            let t0 = Instant::now();
            let out = readback_before_moving(
                READBACK_OUT_S,
                READBACK_POLL_MS,
                || crate::crew_radio::readback_out(&call),
                || voice.as_ref().is_some_and(|v| v.radio_on_air()),
            )
            .await;
            event(
                "readback_out",
                json!({ "out": out, "waited_ms": t0.elapsed().as_millis() as u64 }),
            );
        }
    }
    ClearanceWait::Cleared
}

/// The longest the crew waits for its takeoff or line-up readback to go out
/// before it moves anyway: a busy frequency never strands it on the runway.
pub(super) const READBACK_OUT_S: f64 = 45.0;
/// ...and then for its words to finish on the air.
pub(super) const READBACK_ON_AIR_S: f64 = 10.0;
pub(super) const READBACK_POLL_MS: u64 = 250;

/// Wait, bounded, until the readback has gone OUT (`out`) and the frequency
/// is quiet again (`on_air` false). True when it went out in time. The
/// pilot's cancel ends the wait.
pub(super) async fn readback_before_moving(
    secs: f64,
    poll_ms: u64,
    out: impl Fn() -> bool,
    on_air: impl Fn() -> bool,
) -> bool {
    let poll = Duration::from_millis(poll_ms);
    let t0 = tokio::time::Instant::now();
    while !out() {
        if ABORT.load(SeqCst) || t0.elapsed().as_secs_f64() >= secs {
            return false;
        }
        tokio::time::sleep(poll).await;
    }
    let t1 = tokio::time::Instant::now();
    while on_air() && t1.elapsed().as_secs_f64() < READBACK_ON_AIR_S && !ABORT.load(SeqCst) {
        tokio::time::sleep(poll).await;
    }
    true
}
