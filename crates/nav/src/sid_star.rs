//! STAR constraints and the crew's SID / STAR choice for a route.

use super::*;

/// One published crossing restriction of an arrival (STAR) fix: altitude
/// bounds in feet (either or both; an "at" sets both) and a speed limit.
#[derive(Debug, Clone, PartialEq)]
pub struct StarConstraint {
    pub fix: String,
    /// At or above (a `+`, an "at", a `B` window's lower bound).
    pub min_ft: Option<f64>,
    /// At or below (a `-`, an "at", a `B` window's upper bound).
    pub max_ft: Option<f64>,
    /// At or below this indicated speed crossing the fix, and from there on.
    pub max_kts: Option<f64>,
}

impl StarConstraint {
    /// The altitude a plan's `planned_ft` becomes at this fix: clamped into
    /// the published window. An "at" is exactly its number.
    pub fn clamp_ft(&self, planned_ft: f64) -> f64 {
        let mut a = planned_ft;
        if let Some(max) = self.max_ft {
            a = a.min(max);
        }
        if let Some(min) = self.min_ft {
            a = a.max(min);
        }
        a
    }
}

/// The crossing restrictions of STAR `star` at `icao`, for the fixes on the
/// route: its common part, the enroute transition the route enters by (a
/// transition named for a fix in `route_fixes`), and the runway transition
/// for `runway` ("ALL" and a parallel's "RW10B" included). Speed cells are
/// field 26 (descriptor) and 27: a blank or `-` descriptor is a limit, a `+`
/// is a minimum the governor has no use for. EGPH's STARs all end "FL070,
/// 230 knots" at TARTN, and the follower flew SimBrief's own guess at a
/// descent there, 278 kt at the last fix (EGTE 2026-09-21 had the same shape).
pub fn star_constraints(
    cifp_dir: &str,
    icao: &str,
    star: &str,
    runway: Option<&str>,
    route_fixes: &[&str],
) -> Vec<StarConstraint> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    star_constraints_in(text.lines(), star, runway, route_fixes)
}

/// [`star_constraints`] over the file's lines (pure, for tests). One entry
/// per fix, in procedure order; a fix published twice keeps the tighter
/// bounds of both.
pub fn star_constraints_in<'a>(
    lines: impl Iterator<Item = &'a str>,
    star: &str,
    runway: Option<&str>,
    route_fixes: &[&str],
) -> Vec<StarConstraint> {
    let on_route = |f: &str| route_fixes.iter().any(|r| r.eq_ignore_ascii_case(f));
    let mut out: Vec<StarConstraint> = Vec::new();
    for line in lines {
        let Some(body) = line.strip_prefix("STAR:") else {
            continue;
        };
        let f: Vec<&str> = body.split(',').collect();
        if !cifp_field(&f, 2).eq_ignore_ascii_case(star) {
            continue;
        }
        let tr = cifp_field(&f, 3);
        let ours = if tr.is_empty() || tr.eq_ignore_ascii_case("ALL") {
            true
        } else if let Some(r) = tr.strip_prefix("RW") {
            match runway {
                Some(want) if r.ends_with('B') => runway_number(r) == runway_number(want),
                Some(want) => runway_matches(r, want) && r.len() == want.len(),
                // No planned runway: a runway transition's numbers are a
                // guess, and a wrong guess is a wrong number.
                None => false,
            }
        } else {
            on_route(tr)
        };
        let fix = cifp_field(&f, 4);
        if !ours || fix.is_empty() || !on_route(fix) {
            continue;
        }
        let (a1, a2) = (arinc_alt_ft(cifp_field(&f, 23)), arinc_alt_ft(cifp_field(&f, 24)));
        let (min_ft, max_ft) = match cifp_field(&f, 22).chars().next().unwrap_or(' ') {
            ' ' | '@' => (a1, a1),
            '+' => (a1, None),
            '-' => (None, a1),
            'B' => (a2, a1),
            _ => (None, None),
        };
        let max_kts = match cifp_field(&f, 26).chars().next().unwrap_or(' ') {
            ' ' | '-' | '@' => cifp_field(&f, 27).parse::<f64>().ok().filter(|k| *k > 0.0),
            _ => None,
        };
        if min_ft.is_none() && max_ft.is_none() && max_kts.is_none() {
            continue;
        }
        let tighter = |a: Option<f64>, b: Option<f64>, lo: bool| match (a, b) {
            (Some(x), Some(y)) => Some(if lo { x.max(y) } else { x.min(y) }),
            (x, y) => x.or(y),
        };
        match out.iter_mut().find(|c| c.fix.eq_ignore_ascii_case(fix)) {
            Some(c) => {
                c.min_ft = tighter(c.min_ft, min_ft, true);
                c.max_ft = tighter(c.max_ft, max_ft, false);
                c.max_kts = tighter(c.max_kts, max_kts, false);
            }
            None => out.push(StarConstraint {
                fix: fix.to_string(),
                min_ft,
                max_ft,
                max_kts,
            }),
        }
    }
    out
}

/// A SID that departs `runway` at `icao` — the first ident with a leg whose
/// transition is `RW<runway>`. `None` if the field publishes no SID for it.
/// A departure procedure the crew can fly from the runway it was given, and
/// where it joins the planned route.
#[derive(Debug, Clone, PartialEq)]
pub struct SidChoice {
    pub ident: String,
    /// The runway transition ("RW10", "RW26B").
    pub transition: String,
    /// The last SID fix that is also on the route: from here the route goes on.
    pub join: String,
    /// Its index in the route idents given.
    pub join_index: usize,
    /// The SID's fixes up to and including the join.
    pub fixes: Vec<String>,
}

/// The SIDs that serve `runway` and join `route` (its fix idents in order),
/// best first: the one that joins furthest along the route, then the one
/// with the fewest fixes. The crew chooses its own departure from the
/// runway actually in use: at LIME (2026-09-25) the tower gave runway 10 and
/// the plan's LEGL1P departs 28; LEGL1R and LEGL1W depart 10 to the same
/// LEGLO. Pure over the CIFP file's lines.
pub fn sids_for_route_in<'a>(
    lines: impl Iterator<Item = &'a str> + Clone,
    runway: &str,
    route: &[String],
) -> Vec<SidChoice> {
    let idents = sid_transitions_in(lines.clone(), runway);
    let mut out: Vec<SidChoice> = Vec::new();
    for (id, tr) in idents {
        let fixes: Vec<String> = parse_proc_legs(lines.clone(), ProcKind::Sid, &id, Some(&tr))
            .into_iter()
            .map(|l| l.fix_ident)
            .filter(|f| !f.is_empty() && !f.starts_with("RW"))
            .collect();
        let join = fixes.iter().enumerate().filter_map(|(k, f)| {
            route
                .iter()
                .position(|r| r.eq_ignore_ascii_case(f))
                .map(|ri| (k, ri))
        });
        let Some((k, ri)) = join.max_by_key(|(_, ri)| *ri) else { continue };
        out.push(SidChoice {
            ident: id,
            transition: tr,
            join: fixes[k].clone(),
            join_index: ri,
            fixes: fixes[..=k].to_vec(),
        });
    }
    out.sort_by(|a, b| b.join_index.cmp(&a.join_index).then(a.fixes.len().cmp(&b.fixes.len())));
    out
}

/// [`sids_for_route_in`] from the airport's CIFP file.
pub fn sids_for_route(cifp_dir: &str, icao: &str, runway: &str, route: &[String]) -> Vec<SidChoice> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    sids_for_route_in(text.lines(), runway, route)
}

/// Every SID that serves `runway`, with its runway transition ("RW08R",
/// "RW26B"), in file order. Pure over the CIFP file's lines.
pub fn sid_transitions_in<'a>(lines: impl Iterator<Item = &'a str>, runway: &str) -> Vec<(String, String)> {
    let mut idents: Vec<(String, String)> = Vec::new();
    for line in lines {
        let Some(body) = line.strip_prefix("SID:") else { continue };
        let f: Vec<&str> = body.split(',').collect();
        let (id, tr) = (cifp_field(&f, 2), cifp_field(&f, 3));
        let serves = tr
            .strip_prefix("RW")
            .is_some_and(|r| runway_matches(r.trim_end_matches('B'), runway) || r.eq_ignore_ascii_case(runway));
        if serves && !id.is_empty() && !idents.iter().any(|(i, t)| i == id && t == tr) {
            idents.push((id.to_string(), tr.to_string()));
        }
    }
    idents
}

/// [`sid_transitions_in`] from the airport's CIFP file. Empty with no file.
pub fn sid_transitions(cifp_dir: &str, icao: &str, runway: &str) -> Vec<(String, String)> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    sid_transitions_in(text.lines(), runway)
}

/// The departure for a FILED route: the filed SID itself when it serves the
/// runway and joins the route, else the best of [`sids_for_route`] (it
/// serves the runway and joins the route furthest along, the fewest fixes
/// on a tie). ONE rule for both sides of the frequency: the tower clears it
/// (the station's IFR clearance) and the crew flies it
/// (`full_auto::choose_departure` walks the same order), so a clearance is
/// never a SID the crew cannot fly to its route. EGKK 08R (sortie
/// 1790601822, 2026-09-28): the tower cleared KENE3P, whose exit KENET lay
/// nearest the bearing to Birmingham, for a route filed LAM L610 HEMEL; the
/// crew flew LAM5P. `None` when no SID off the runway meets the route.
/// Pure over the CIFP file's lines.
pub fn sid_for_filed_route_in<'a>(
    lines: impl Iterator<Item = &'a str> + Clone,
    runway: &str,
    route: &[String],
    filed_sid: Option<&str>,
) -> Option<SidChoice> {
    let mut all = sids_for_route_in(lines, runway, route);
    let filed = filed_sid.and_then(|f| all.iter().position(|c| c.ident.eq_ignore_ascii_case(f)));
    match filed {
        Some(i) => Some(all.swap_remove(i)),
        None => all.into_iter().next(),
    }
}

/// [`sid_for_filed_route_in`] from the airport's CIFP file.
pub fn sid_for_filed_route(
    cifp_dir: &str,
    icao: &str,
    runway: &str,
    route: &[String],
    filed_sid: Option<&str>,
) -> Option<SidChoice> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let text = std::fs::read_to_string(path).ok()?;
    sid_for_filed_route_in(text.lines(), runway, route, filed_sid)
}

/// An arrival procedure the crew can fly to the runway it expects, and where
/// it leaves the planned route.
#[derive(Debug, Clone)]
pub struct StarChoice {
    pub ident: String,
    /// The first STAR fix on the route: the route is flown to here, the
    /// STAR from here.
    pub entry: String,
    pub entry_index: usize,
    /// The STAR's legs from the entry on (fixes only, unresolved): its
    /// enroute transition, common part and runway transition, in order.
    pub legs: Vec<ProcLeg>,
    /// Legs the follower cannot fly as published from the entry on: arcs
    /// (AF/RF) flown as chords, vector legs dropped.
    pub bent: usize,
}

/// Every STAR at the field that serves `runway` and meets `route`, the one
/// entering earliest first (the most of the route flown as published), then
/// the one with fewest legs that fix-to-fix flying bends (BAME6N's DME arc
/// against BAME5M's straight RNAV legs, LIPE 12), then the shortest. A STAR's rows are three parts by route type (field 1):
/// 1/4/7 an enroute transition named for its first fix, 2/5/8 the common
/// part (named for its runway, "ALL" or blank), 3/6/9 a runway transition.
/// Vector legs (no fix) are dropped: the follower flies fix to fix, and the
/// approach picks up where the fixes end.
pub fn stars_for_route_in<'a>(
    lines: impl Iterator<Item = &'a str> + Clone,
    runway: &str,
    route: &[String],
) -> Vec<StarChoice> {
    let serves = |tr: &str| {
        tr.strip_prefix("RW").is_some_and(|r| {
            if r.ends_with('B') {
                runway_number(r) == runway_number(runway)
            } else {
                runway_matches(r, runway) && r.len() == runway.len()
            }
        })
    };
    let group = |t: &str| match t {
        "1" | "4" | "7" => 0,
        "2" | "5" | "8" => 1,
        "3" | "6" | "9" => 2,
        _ => 3,
    };
    // ident -> (group, transition) in file order.
    let mut parts: Vec<(String, Vec<(u8, String)>)> = Vec::new();
    for line in lines.clone() {
        let Some(body) = line.strip_prefix("STAR:") else { continue };
        let f: Vec<&str> = body.split(',').collect();
        let (id, tr, g) = (cifp_field(&f, 2), cifp_field(&f, 3), group(cifp_field(&f, 1)));
        if id.is_empty() || g == 3 {
            continue;
        }
        let entry = match parts.iter_mut().find(|(i, _)| i == id) {
            Some(e) => e,
            None => {
                parts.push((id.to_string(), Vec::new()));
                parts.last_mut().expect("just pushed")
            }
        };
        if !entry.1.iter().any(|(gg, t)| *gg == g && t == tr) {
            entry.1.push((g, tr.to_string()));
        }
    }
    let on_route = |f: &str| route.iter().position(|r| r.eq_ignore_ascii_case(f));
    let legs_of = |id: &str, tr: &str| -> Vec<ProcLeg> {
        parse_proc_legs(lines.clone(), ProcKind::Star, id, Some(tr))
            .into_iter()
            .filter(|l| l.transition == tr)
            .collect()
    };
    let mut out = Vec::new();
    for (id, trs) in parts {
        let common: Vec<&String> = trs
            .iter()
            .filter(|(g, t)| *g == 1 && (t.is_empty() || t.eq_ignore_ascii_case("ALL") || serves(t)))
            .map(|(_, t)| t)
            .collect();
        let rw_tr = trs.iter().find(|(g, t)| *g == 2 && serves(t)).map(|(_, t)| t);
        let has_rw_trs = trs.iter().any(|(g, _)| *g == 2);
        let common_for_rw = common.iter().any(|t| serves(t));
        if !(common_for_rw || rw_tr.is_some() || (!common.is_empty() && !has_rw_trs)) {
            continue;
        }
        // The enroute transition the route enters by: the one with a fix
        // earliest on the route.
        let enroute = trs
            .iter()
            .filter(|(g, _)| *g == 0)
            .filter_map(|(_, t)| {
                let legs = legs_of(&id, t);
                let first = legs.iter().filter_map(|l| on_route(&l.fix_ident)).min()?;
                Some((first, legs))
            })
            .min_by_key(|(first, _)| *first)
            .map(|(_, legs)| legs)
            .unwrap_or_default();
        let mut path: Vec<ProcLeg> = enroute;
        for t in &common {
            path.extend(legs_of(&id, t));
        }
        if let Some(t) = rw_tr {
            path.extend(legs_of(&id, t));
        }
        path.retain(|l| !l.fix_ident.starts_with("RW"));
        path.dedup_by(|b, a| a.fix_ident.eq_ignore_ascii_case(&b.fix_ident));
        let Some((p, ri)) = path
            .iter()
            .enumerate()
            .find_map(|(p, l)| on_route(&l.fix_ident).map(|ri| (p, ri)))
        else {
            continue;
        };
        let mut legs = path.split_off(p);
        let bent = legs
            .iter()
            .filter(|l| l.fix_ident.is_empty() || matches!(l.leg_type.as_str(), "AF" | "RF"))
            .count();
        legs.retain(|l| !l.fix_ident.is_empty());
        out.push(StarChoice {
            ident: id,
            entry: legs[0].fix_ident.clone(),
            entry_index: ri,
            legs,
            bent,
        });
    }
    out.sort_by(|a, b| {
        a.entry_index
            .cmp(&b.entry_index)
            .then(a.bent.cmp(&b.bent))
            .then(a.legs.len().cmp(&b.legs.len()))
    });
    out
}

/// [`stars_for_route_in`] from the airport's CIFP file.
pub fn stars_for_route(cifp_dir: &str, icao: &str, runway: &str, route: &[String]) -> Vec<StarChoice> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    stars_for_route_in(text.lines(), runway, route)
}

/// Where one SID off one runway leaves the terminal area: the last fix of
/// its runway transition plus common route, positioned.
#[derive(Debug, Clone, PartialEq)]
pub struct SidExit {
    pub ident: String,
    /// The runway transition ("RW24", "RW26B").
    pub transition: String,
    pub fix_ident: String,
    pub lat: f64,
    pub lon: f64,
}

/// Every SID's exit off every runway at a field, from the CIFP lines, the
/// fixes positioned through [`resolve_proc_fixes`] (ONE scan of each nav
/// file for the whole field: seconds on the real files, so the owner runs
/// it off the reply path and caches it). A SID whose legs resolve no fix
/// has no exit and is left out.
pub fn sid_exits_in<'a>(
    lines: impl Iterator<Item = &'a str> + Clone,
    fix_path: &str,
    nav_path: &str,
    airport: &AirportInfo,
    icao: &str,
) -> Vec<SidExit> {
    let mut idents: Vec<(String, String)> = Vec::new();
    for line in lines.clone() {
        let Some(body) = line.strip_prefix("SID:") else { continue };
        let f: Vec<&str> = body.split(',').collect();
        let (id, tr) = (cifp_field(&f, 2), cifp_field(&f, 3));
        if !id.is_empty() && tr.starts_with("RW") && !idents.iter().any(|(i, t)| i == id && t == tr) {
            idents.push((id.to_string(), tr.to_string()));
        }
    }
    // All the legs in one vector, so the nav files are scanned once.
    let mut legs: Vec<ProcLeg> = Vec::new();
    let mut spans: Vec<(usize, usize)> = Vec::new();
    for (id, tr) in &idents {
        let start = legs.len();
        legs.extend(parse_proc_legs(lines.clone(), ProcKind::Sid, id, Some(tr)));
        spans.push((start, legs.len()));
    }
    let (alat, alon) = airport.position().unwrap_or((0.0, 0.0));
    resolve_proc_fixes(fix_path, nav_path, airport, icao, &mut legs, alat, alon);
    idents
        .into_iter()
        .zip(spans)
        .filter_map(|((ident, transition), (a, b))| {
            let last = legs[a..b]
                .iter()
                .rev()
                .find(|l| l.fix.is_some() && !l.fix_ident.starts_with("RW"))?;
            let fix = last.fix.as_ref()?;
            Some(SidExit {
                ident,
                transition,
                fix_ident: last.fix_ident.clone(),
                lat: fix.lat,
                lon: fix.lon,
            })
        })
        .collect()
}

/// [`sid_exits_in`] from the field's CIFP file. Empty with no file.
pub fn sid_exits(
    cifp_dir: &str,
    fix_path: &str,
    nav_path: &str,
    airport: &AirportInfo,
    icao: &str,
) -> Vec<SidExit> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    sid_exits_in(text.lines(), fix_path, nav_path, airport, icao)
}

/// The SID off `runway` that leaves toward the destination: the one whose
/// exit fix lies closest in bearing (from the field) to the great-circle
/// bearing from the field to the destination. `(ident, exit bearing,
/// destination bearing)`, degrees true; `None` when no exit serves the
/// runway. Pure.
pub fn sid_toward(
    exits: &[SidExit],
    runway: &str,
    field: (f64, f64),
    dest: (f64, f64),
) -> Option<(String, f64, f64)> {
    let want = bearing_deg(field.0, field.1, dest.0, dest.1);
    let off = |b: f64| {
        let d = (b - want).rem_euclid(360.0);
        d.min(360.0 - d)
    };
    exits
        .iter()
        .filter(|e| {
            e.transition.strip_prefix("RW").is_some_and(|r| {
                r.eq_ignore_ascii_case(runway) || runway_matches(r.trim_end_matches('B'), runway)
            })
        })
        .map(|e| (e, bearing_deg(field.0, field.1, e.lat, e.lon)))
        .min_by(|a, b| off(a.1).total_cmp(&off(b.1)))
        .map(|(e, b)| (e.ident.clone(), b, want))
}

pub fn sid_for_runway(cifp_dir: &str, icao: &str, runway: &str) -> Option<String> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let text = std::fs::read_to_string(path).ok()?;
    let want = format!("RW{runway}");
    for line in text.lines() {
        if !line.starts_with("SID:") {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        let tr = cifp_field(&f, 3);
        if tr == want
            || (runway_matches(tr.strip_prefix("RW").unwrap_or(tr), runway) && tr.starts_with("RW"))
        {
            let id = cifp_field(&f, 2);
            if !id.is_empty() {
                return Some(id.to_string());
            }
        }
    }
    None
}
