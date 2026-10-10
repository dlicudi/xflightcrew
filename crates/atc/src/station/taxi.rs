//! Taxi routes, ILS facts and taxi phraseology.

use super::*;

// --- Taxi routes -------------------------------------------------------------

/// The active field's taxi network, loaded in the background (a 400 MB
/// apt.dat scan must never stall the reply path). `None` while loading or
/// when the field has none — taxi clearances then phrase route-less,
/// exactly as before.
pub(super) static TAXI_NET: Mutex<Option<std::sync::Arc<nav::TaxiNet>>> = Mutex::new(None);

/// The loaded taxi network, for the party line's stand accessor. `None`
/// until the background load lands or when the field has none.
pub(crate) fn taxi_net() -> Option<std::sync::Arc<nav::TaxiNet>> {
    TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// The taxi route the pilot is CLEARED on — published by the reply path
/// when a taxi clearance is issued, consumed by the compliance watch (ATC's
/// off-route challenge) and the copilot's progressive taxi guide (the crew
/// side: "next left onto Whiskey").
#[derive(Clone)]
pub struct TaxiWatch {
    /// Node polyline of the cleared route.
    pub path: Vec<(f64, f64)>,
    /// The spoken via line, for the challenge ("taxi via Alpha, Bravo").
    pub via: String,
    /// Per-segment RAW (name, is_runway) aligned with `path` segments
    /// (`seg[i]` = the leg `path[i] -> path[i+1]`); phrase with
    /// [`spoken_taxiway`] / [`atis::spoken_runway`] at speak time.
    pub seg: Vec<(String, bool)>,
    /// RAW holding-point taxiway name ("A13"), the route's destination.
    pub holding_point: String,
    /// The authored lead-on the hold-short trim removed: the painted line from
    /// the holding point onto the runway centreline. The crew's line-up drives
    /// it instead of a synthetic diagonal. Empty when the route ends at a
    /// genuine off-runway node.
    pub lead_on: Vec<(f64, f64)>,
}

/// The active field's ILS facilities (earth_nav.dat) and per-runway platform
/// altitudes (CIFP approach legs), background-loaded per station — both are
/// full-file scans that must never stall the reply path. `None` while loading
/// or when the field has no ILS; vectors then fly the visual profile.
pub struct IlsFacts {
    pub ils: Vec<nav::AirportIls>,
    /// Glideslope-intercept ("descend to X until established") altitude per
    /// runway ident, ft MSL, where the CIFP has the approach.
    pub platform_ft: std::collections::HashMap<String, f64>,
}

pub(super) static ILS_FACTS: Mutex<Option<std::sync::Arc<IlsFacts>>> = Mutex::new(None);

/// Kick off the background ILS + procedures load for the active airport.
/// Called by the station's owner at install time, like the taxi network.
pub fn spawn_ils_load(nav_path: String, cifp_dir: String, icao: String) {
    *ILS_FACTS.lock().unwrap_or_else(|e| e.into_inner()) = None;
    std::thread::spawn(move || match nav::airport_ils(&nav_path, &icao) {
        Ok(list) if !list.is_empty() => {
            let platform_ft: std::collections::HashMap<String, f64> = list
                .iter()
                .filter_map(|i| {
                    nav::ils_platform_alt_ft(&cifp_dir, &icao, &i.runway)
                        .map(|a| (i.runway.clone(), a))
                })
                .collect();
            tracing::info!(
                %icao,
                ils = list.len(),
                platforms = platform_ft.len(),
                "ILS data loaded: vectors can offer the ILS approach"
            );
            *ILS_FACTS.lock().unwrap_or_else(|e| e.into_inner()) =
                Some(std::sync::Arc::new(IlsFacts {
                    ils: list,
                    platform_ft,
                }));
        }
        Ok(_) => tracing::info!(%icao, "no ILS at this field; visual vectors only"),
        Err(e) => tracing::warn!(%icao, "ILS scan failed: {e:#}"),
    });
}

/// Kick off the background taxi-network load for the active airport.
/// Called by the station's owner at install time.
pub fn spawn_taxi_net_load(apt_path: String, icao: String) {
    *TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()) = None;
    std::thread::spawn(move || match nav::find_taxi_net(&apt_path, &icao) {
        Ok(Some(net)) => {
            tracing::info!(
                %icao,
                nodes = net.nodes.len(),
                edges = net.edges.len(),
                "taxi network loaded: clearances will carry named routes"
            );
            *TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()) = Some(std::sync::Arc::new(net));
        }
        Ok(None) => tracing::info!(%icao, "no taxi network in apt.dat; route-less taxi clearances"),
        Err(e) => tracing::warn!(%icao, "taxi network scan failed: {e:#}"),
    });
}

/// Plan the taxi route from the aircraft to the holding point of `ident`'s
/// runway end. `None` when there's no network (yet), no snap, or no path —
/// the clearance then falls back to the route-less phrasing.
/// Route an ARRIVAL from where it is to the NEAREST suitable parking stand —
/// the mirror of [`plan_taxi_route_to`] for taxi-in. Same generic Dijkstra; the
/// destination is a stand, so there's no runway hold-short trim (a stand is a
/// genuine off-runway end node), and the plan's `holding_point` carries the raw
/// stand name so the reply and the crew call it by name. Runway crossings on
/// the way in are still named. `None` when the field has no stands or neither
/// end snaps to the network.
/// `occupied`: where aircraft stand on the ground (a traffic desk's
/// picture); a stand one of them is on is passed over for the next nearest.
/// Empty for ours: our route is planned as it always was.
pub(super) fn plan_taxi_route_to_stand(
    runways: &[nav::Runway],
    lat: f64,
    lon: f64,
    psi_true: Option<f64>,
    occupied: &[(f64, f64)],
) -> Option<nav::TaxiRoutePlan> {
    let net = TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
    stand_route_avoiding(&net, runways, lat, lon, psi_true, occupied)
}

/// A stand is taken when an aircraft stands this close to it, metres
/// (the traffic crate's own `STAND_TAKEN_M`).
const STAND_TAKEN_M: f64 = 20.0;

/// True heading, for which way is forward off a runway.
pub(super) const TRUE_PSI: &str = sim::datarefs::TRUE_HEADING_DEG;

/// [`plan_taxi_route_to_stand`] on a given network, so the real apt.dat can
/// test it. ON a runway with a heading, the route starts with the drive off
/// it, FORWARD ([`nav::TaxiNet::plan_vacate`]), and the stand is the one
/// nearest where that drive ends: planned from the strip itself it snapped
/// to the nearest node whatever the heading and backtracked (EDLP 24,
/// 2026-09-24, see [`nav::after_vacate`]). Off a runway, no heading, or no
/// way off ahead: the plain route from here, as before.
#[cfg(test)]
pub(super) fn stand_route_on(
    net: &nav::TaxiNet,
    runways: &[nav::Runway],
    lat: f64,
    lon: f64,
    psi_true: Option<f64>,
) -> Option<nav::TaxiRoutePlan> {
    stand_route_avoiding(net, runways, lat, lon, psi_true, &[])
}

/// [`stand_route_on`] to the nearest stand nobody in `occupied` stands on
/// (within [`STAND_TAKEN_M`]). A traffic arrival is not sent onto a stand
/// a parked aircraft stands on (stage D's parked aircraft, 2026-10-08).
pub(super) fn stand_route_avoiding(
    net: &nav::TaxiNet,
    runways: &[nav::Runway],
    lat: f64,
    lon: f64,
    psi_true: Option<f64>,
    occupied: &[(f64, f64)],
) -> Option<nav::TaxiRoutePlan> {
    let on = runways.iter().find(|r| on_runway(r, lat, lon).is_some());
    let vacate = match (on, psi_true.filter(|p| p.is_finite())) {
        (Some(r), Some(psi)) => {
            let zone = vec![r.ident_a.clone(), r.ident_b.clone()];
            net.plan_vacate(lat, lon, psi, &zone, nav::VACATE_CLEAR_M)
                .filter(|v| v.path.len() >= 2)
                .map(|v| (v, format!("{}/{}", r.ident_a, r.ident_b)))
        }
        _ => None,
    };
    let (slat, slon) = vacate
        .as_ref()
        .and_then(|(v, _)| v.path.last().copied())
        .unwrap_or((lat, lon));
    let (stand, mut plan) = if occupied.is_empty() {
        let stand = net.nearest_stand(slat, slon)?.clone();
        // To the STAND, not to the taxiway node nearest it — see `plan_to_stand`.
        let mut plan = net.plan_to_stand(slat, slon, &stand)?;
        mark_runway_crossings(&mut plan, runways);
        (stand, plan)
    } else {
        let taken = |s: &nav::Stand| {
            occupied
                .iter()
                .any(|(a, b)| nav::haversine_nm(s.lat, s.lon, *a, *b) * 1852.0 < STAND_TAKEN_M)
        };
        let d = |s: &nav::Stand| nav::haversine_nm(slat, slon, s.lat, s.lon);
        let mut free: Vec<&nav::Stand> = net
            .stands
            .iter()
            .filter(|s| s.is_suitable() && !taken(s))
            .collect();
        free.sort_by(|a, b| d(a).total_cmp(&d(b)));
        let planned_from = |from: (f64, f64), s: &nav::Stand| {
            let mut plan = net.plan_to_stand(from.0, from.1, s)?;
            mark_runway_crossings(&mut plan, runways);
            Some(((*s).clone(), plan))
        };
        let first = free.first().copied()?;
        let clear_of =
            |from: (f64, f64)| planned_from(from, first).filter(|(_, p)| p.crossings.is_empty());
        clear_of((slat, slon))
            .or_else(|| {
                // Off the strip but nearest a network node on it (the route
                // went back up the exit through the runway's strip: an
                // arrival 66 m off LFPG's 09R, south of its exit Y3, held
                // short of the runway it had just left): from the nearest
                // node clear of every strip instead, the drive there first.
                let off = runways.iter().all(|r| on_runway(r, slat, slon).is_none());
                let node = off
                    .then(|| node_clear_of_strips(net, runways, slat, slon))
                    .flatten()?;
                let (stand, mut plan) = clear_of(node)?;
                plan.path.insert(0, (slat, slon));
                plan.seg.insert(0, (String::new(), false));
                Some((stand, plan))
            })
            .or_else(|| planned_from((slat, slon), first))?
    };
    plan.holding_point = stand.name.clone();
    if let Some((v, strip)) = vacate {
        sim::blackbox::record(
            "atc",
            serde_json::json!({
                "event": "stand_route_from_runway", "runway": strip,
                "exit": v.exit_name, "vacate_pts": v.path.len(),
                "stand": stand.name,
            }),
        );
        plan = nav::after_vacate(v, plan);
    }
    Some(plan)
}

/// The network node nearest (`lat`, `lon`) that touches a taxiway edge and
/// lies on no runway strip, within [`CLEAR_NODE_MAX_M`].
fn node_clear_of_strips(
    net: &nav::TaxiNet,
    runways: &[nav::Runway],
    lat: f64,
    lon: f64,
) -> Option<(f64, f64)> {
    let taxiway: std::collections::HashSet<i64> = net
        .edges
        .iter()
        .filter(|e| !e.runway)
        .flat_map(|e| [e.a, e.b])
        .collect();
    let d = |p: (f64, f64)| nav::haversine_nm(lat, lon, p.0, p.1) * 1852.0;
    taxiway
        .iter()
        .filter_map(|id| net.nodes.get(id).copied())
        .filter(|p| runways.iter().all(|r| on_runway(r, p.0, p.1).is_none()))
        .filter(|p| d(*p) < CLEAR_NODE_MAX_M)
        .min_by(|a, b| d(*a).total_cmp(&d(*b)))
}

/// The farthest a stand route's start is moved to a node clear of the
/// runway strips, metres ([`node_clear_of_strips`]).
const CLEAR_NODE_MAX_M: f64 = 150.0;

/// Plan the drive off `runway` from where the rollout stopped. Reached only
/// through [`crate::party_line::plan_vacate`] — `TAXI_NET` stays station-private.
///
/// The active zone is asked for by BOTH ends of the strip, not just the one we
/// landed on: apt.dat 1204 rows are per runway END (an arrival zone for 09 and
/// another for 27 covering much the same pavement), and an edge marked only
/// for the opposite direction is still the pavement we are trying to get off.
pub(crate) fn plan_vacate_from(
    lat: f64,
    lon: f64,
    psi_true: f64,
    runway: &str,
) -> Option<nav::VacatePlan> {
    let net = TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
    let mut zone = vec![runway.to_string()];
    if let Some(r) = reciprocal_end(runway) {
        zone.push(r);
    }
    net.plan_vacate(lat, lon, psi_true, &zone, nav::VACATE_CLEAR_M)
}

/// The other end of the same strip: "09L" -> "27R", "36" -> "18". `None` when
/// the ident isn't a runway designator we can turn around.
pub fn reciprocal_end(id: &str) -> Option<String> {
    let digits: String = id.chars().take_while(|c| c.is_ascii_digit()).collect();
    let side: String = id[digits.len()..].to_string();
    let n: u32 = digits.parse().ok()?;
    if n == 0 || n > 36 {
        return None;
    }
    let opp = if n > 18 { n - 18 } else { n + 18 };
    let side = match side.as_str() {
        "L" => "R",
        "R" => "L",
        s => s,
    };
    Some(format!("{opp:02}{side}"))
}

/// Speak a parking-stand name: spell digit runs ("Stand 12" -> "stand one
/// two"), lowercase the rest, and prefix "stand" for a bare designator (LXGB
/// names its gates "1".."5", so this reads "stand five", not just "five") —
/// unless the name already says what kind of parking it is. `pub` so the crew's
/// auto-taxi calls the destination by the same name.
pub fn spoken_stand(name: &str) -> String {
    let spoken = name
        .split_whitespace()
        .map(|w| {
            if !w.is_empty() && w.chars().all(|c| c.is_ascii_digit()) {
                atis::spoken_digits(w)
            } else {
                w.to_ascii_lowercase()
            }
        })
        .collect::<Vec<_>>()
        .join(" ");
    if spoken.is_empty() {
        return spoken;
    }
    let names_the_kind = name.split_whitespace().any(|w| {
        matches!(
            w.to_ascii_lowercase().as_str(),
            "stand" | "gate" | "ramp" | "apron" | "bay" | "parking" | "tie-down" | "tiedown"
        )
    });
    if names_the_kind {
        spoken
    } else {
        format!("stand {spoken}")
    }
}

/// [`spoken_stand`] in the field's dialect: ICAO and Europe park on a
/// "stand", the FAA at a "gate" (7110.65 says gate, ramp or parking; a
/// stand is not an American word). A name that says ramp, apron or
/// tie-down keeps it; "stand" itself becomes "gate" at an FAA field.
pub fn spoken_stand_in(name: &str, dialect: Dialect) -> String {
    let spoken = spoken_stand(name);
    match dialect {
        Dialect::Icao => spoken,
        Dialect::Faa => spoken
            .split(' ')
            .map(|w| if w == "stand" { "gate" } else { w })
            .collect::<Vec<_>>()
            .join(" "),
    }
}

pub(super) fn plan_taxi_route_to(
    runways: &[nav::Runway],
    ident: &str,
    lat: f64,
    lon: f64,
) -> Option<nav::TaxiRoutePlan> {
    let net = TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
    let r = strip(runways, ident)?;
    let (tlat, tlon) = if r.ident_a == ident {
        (r.lat_a, r.lon_a)
    } else {
        (r.lat_b, r.lon_b)
    };
    // The hold-short standoff scales with the runway's class: instrument
    // runways paint their holding positions much further out than the GA
    // default (Heathrow's CAT holds sit ~100 m from the centreline — the
    // 40 m stop parked a taxi on the 09L entry stub past the line,
    // visually ON the runway, 2026-07-20). ILS on either end of the strip
    // = instrument holds; otherwise width-scaled, floored at the default.
    // ILS facts still loading falls back to the GA figure — early over
    // absent.
    let instrument = ILS_FACTS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .is_some_and(|f| {
            f.ils
                .iter()
                .any(|i| i.runway == r.ident_a || i.runway == r.ident_b)
        });
    let standoff = if instrument {
        100.0
    } else {
        (r.width_m / 2.0 + 30.0).max(nav::HOLD_SHORT_STANDOFF_M)
    };
    let zone = [r.ident_a.clone(), r.ident_b.clone()];
    let mut plan = net.plan_to_runway(lat, lon, tlat, tlon, standoff, &zone, Some(r))?;
    mark_runway_crossings(&mut plan, runways);
    Some(plan)
}

/// Name the crossings the planner couldn't — geometry is the authority,
/// never the network flags, because apt.dat authors get crossings wrong two
/// ways: runway edges left UNNAMED (the planner's crossings list is named
/// edges only — the KPDK clearance never said "cross runway ...",
/// 2026-07-14), and crossing links drawn as plain TAXIWAY edges straight
/// across the strip (EGLL's N7: the route crossed an active 09R with no
/// crossing clause, no guide call, and no hold, 2026-07-20). Any leg whose
/// sampled points sit on a physical strip is renamed to the strip's
/// designator pair and runway-flagged — which feeds the clearance's "cross
/// runway ..." clause, the guide's "check it's clear, then cross" cue, AND
/// the cleared-crossing exemption in the incursion watch, all keyed on that
/// flag. The route's TAIL is exempt when taxiway-flagged: post-trim it ends
/// at the hold-short point NEAR the destination strip, which must not read as
/// crossing it.
///
/// How much tail that is. One network edge used to be the unit of a leg, so
/// exempting the last leg exempted the whole approach to the holding point;
/// legs are metres long now that routes are resampled onto the paint. This is
/// the standoff the trim itself works in — anything nearer the end than this
/// is the same final approach to the same hold.
pub(super) const TAIL_EXEMPT_M: f64 = nav::HOLD_SHORT_STANDOFF_M;

pub(super) fn mark_runway_crossings(plan: &mut nav::TaxiRoutePlan, runways: &[nav::Runway]) {
    let last = plan.seg.len().saturating_sub(1);
    // Distance from each leg's start to the end of the route, so the "we stop
    // short of the destination runway, we don't cross it" exemption covers
    // the whole approach to the holding point. It used to be `i == last`,
    // which was the same thing while a leg meant a network edge — since the
    // route is resampled onto the paint a leg is a few metres, and the last
    // one alone is a sliver that exempts nothing.
    let mut to_end: Vec<f64> = vec![0.0; plan.path.len()];
    for i in (0..plan.path.len().saturating_sub(1)).rev() {
        let (a, b) = (plan.path[i], plan.path[i + 1]);
        to_end[i] = to_end[i + 1] + crate::nav::haversine_nm(a.0, a.1, b.0, b.1) * 1852.0;
    }
    for i in 0..plan.seg.len() {
        let (name, is_runway) = plan.seg[i].clone();
        if is_runway && !name.is_empty() {
            continue; // the planner already counted this one
        }
        if !is_runway && (i == last || to_end.get(i).is_some_and(|d| *d < TAIL_EXEMPT_M)) {
            continue; // ends short of the destination runway, not across it
        }
        // Sample the leg every ~20 m (its midpoint alone can miss a strip
        // the leg only clips): first sampled point on a runway names it.
        let (a, b) = (plan.path[i], plan.path[i + 1]);
        let len_m = crate::nav::haversine_nm(a.0, a.1, b.0, b.1) * 1852.0;
        let steps = ((len_m / 20.0).ceil() as usize).max(1);
        let hit = (0..=steps).find_map(|k| {
            let t = k as f64 / steps as f64;
            let (plat, plon) = (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
            runways.iter().find(|r| on_runway(r, plat, plon).is_some())
        });
        if let Some(r) = hit {
            let full = format!("{}/{}", r.ident_a, r.ident_b);
            plan.seg[i] = (full.clone(), true);
            if !plan.crossings.contains(&full) {
                plan.crossings.push(full);
            }
        }
    }
}

/// A taxiway name spoken like a controller says it: designator letters go
/// NATO, digits are read out, real words stay words — "B2" → "Bravo two",
/// "N4W" → "November four Whiskey", "LINK27" → "Link two seven",
/// "Inner" stays "Inner". Per token: letter runs of 1-2 are designators
/// (NATO), 3+ are words; digit runs are always spoken.
pub fn spoken_taxiway(name: &str) -> String {
    let mut out: Vec<String> = Vec::new();
    for tok in name.split_whitespace() {
        let mut run = String::new();
        let flush = |run: &mut String, out: &mut Vec<String>| {
            if run.is_empty() {
                return;
            }
            if run.chars().all(|c| c.is_ascii_digit()) {
                out.push(atis::spoken_digits(run));
            } else if run.chars().count() <= 2 {
                out.push(tower::spoken_callsign(run));
            } else {
                // A real word: keep it, title-cased so ALL-CAPS apt.dat
                // names ("LINK") don't get spelled out by the TTS.
                let mut w = run.to_lowercase();
                if let Some(c) = w.get_mut(0..1) {
                    c.make_ascii_uppercase();
                }
                out.push(w);
            }
            run.clear();
        };
        let mut prev_digit: Option<bool> = None;
        for c in tok.chars() {
            let digit = c.is_ascii_digit();
            if prev_digit.is_some_and(|p| p != digit) {
                flush(&mut run, &mut out);
            }
            run.push(c);
            prev_digit = Some(digit);
        }
        flush(&mut run, &mut out);
    }
    out.join(" ")
}

/// The designator-and-number split of a taxiway name ("M3" → ("M", 3)) —
/// `None` when the name doesn't end in digits or doesn't start with them.
pub(super) fn stem_num(name: &str) -> Option<(&str, i64)> {
    let idx = name.find(|c: char| c.is_ascii_digit())?;
    let (stem, num) = name.split_at(idx);
    if stem.is_empty() {
        return None;
    }
    Some((stem, num.parse().ok()?))
}

/// A via list spoken like a controller reads it: a run of three or more
/// same-designator, consecutively numbered segments collapses to its ends —
/// "C1, M1, M2, M3, M4, M5" becomes "Charlie one, Mike one to Mike five".
/// The apt.dat network names every stretch of pavement; a clearance reads
/// like a controller, not like the database (Madrid, 2026-07-13).
pub fn spoken_via(names: &[String]) -> Vec<String> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < names.len() {
        let mut j = i + 1;
        if let Some((stem, first)) = stem_num(&names[i]) {
            let mut prev = first;
            let mut step = 0i64;
            while j < names.len() {
                let Some((s, n)) = stem_num(&names[j]) else {
                    break;
                };
                let d = n - prev;
                if s != stem || d.abs() != 1 || (step != 0 && d != step) {
                    break;
                }
                step = d;
                prev = n;
                j += 1;
            }
            if j - i >= 3 {
                out.push(format!(
                    "{} to {}",
                    spoken_taxiway(&names[i]),
                    spoken_taxiway(&names[j - 1])
                ));
                i = j;
                continue;
            }
        }
        out.push(spoken_taxiway(&names[i]));
        i += 1;
    }
    out
}

/// The first designator of a runway-edge name ("09L/27R" → "09L") — the
/// end a crossing instruction names. `pub` for the copilot's taxi guide,
/// which phrases the same runway-edge names.
pub fn first_designator(name: &str) -> &str {
    name.split('/').next().unwrap_or(name).trim()
}

/// Distance from a point to a polyline, meters — the off-route check.
/// Whether the pilot stands where the cleared taxi route itself runs over a
/// runway — a crossing or backtrack the tower put in the clearance ("cross
/// runway two seven"). Geometry, not the apt.dat runway flag: authors leave
/// crossing edges unnamed and sometimes unflagged, and the flag adds nothing
/// the pavement test doesn't. The pilot must be within the off-route corridor
/// of the route AND the route's nearest point must itself be ON a runway —
/// so nosing past a hold-short point onto a runway the route only ends
/// NEAR still reads as an incursion (the route's nearest point there is the
/// holding point, off the pavement).
#[cfg_attr(not(test), allow(dead_code))] // the callers hold a strip (`_for`)
pub(super) fn on_cleared_runway_crossing(lat: f64, lon: f64) -> bool {
    on_cleared_runway_crossing_for(&OWN_STRIP, lat, lon)
}

/// [`on_cleared_runway_crossing`] on one flight's strip (its cleared route).
pub(super) fn on_cleared_runway_crossing_for(strip: &FlightStrip, lat: f64, lon: f64) -> bool {
    const CORRIDOR_M: f64 = 60.0;
    let nearest = {
        let route = strip.taxi_route.lock().unwrap_or_else(|e| e.into_inner());
        let Some(r) = route.as_ref() else {
            return false;
        };
        nearest_on_polyline(&r.path, lat, lon)
    };
    // Route lock dropped before taking the runway lock — no nesting.
    nearest.is_some_and(|(p, d)| d <= CORRIDOR_M && on_active_runway(p.0, p.1).is_some())
}

/// The closest point of a polyline to (lat, lon), with its distance in
/// meters. Flat-earth at taxi scales, like [`dist_to_polyline_m`].
pub(super) fn nearest_on_polyline(path: &[(f64, f64)], lat: f64, lon: f64) -> Option<((f64, f64), f64)> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    path.windows(2)
        .map(|w| {
            let (ax, ay) = (
                (w[0].1 - lon) * m_per_deg_lon,
                (w[0].0 - lat) * m_per_deg_lat,
            );
            let (bx, by) = (
                (w[1].1 - lon) * m_per_deg_lon,
                (w[1].0 - lat) * m_per_deg_lat,
            );
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            let t = if len2 < 1.0 {
                0.0
            } else {
                (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0)
            };
            let (cx, cy) = (ax + t * dx, ay + t * dy);
            let plat = w[0].0 + t * (w[1].0 - w[0].0);
            let plon = w[0].1 + t * (w[1].1 - w[0].1);
            ((plat, plon), (cx * cx + cy * cy).sqrt())
        })
        .min_by(|x, y| x.1.total_cmp(&y.1))
}

pub(super) fn dist_to_polyline_m(path: &[(f64, f64)], lat: f64, lon: f64) -> Option<f64> {
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
    path.windows(2)
        .map(|w| {
            let (ax, ay) = (
                (w[0].1 - lon) * m_per_deg_lon,
                (w[0].0 - lat) * m_per_deg_lat,
            );
            let (bx, by) = (
                (w[1].1 - lon) * m_per_deg_lon,
                (w[1].0 - lat) * m_per_deg_lat,
            );
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            let t = if len2 < 1.0 {
                0.0
            } else {
                (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0)
            };
            let (cx, cy) = (ax + t * dx, ay + t * dy);
            (cx * cx + cy * cy).sqrt()
        })
        .min_by(|x, y| x.total_cmp(y))
}
