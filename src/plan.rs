//! The flight plan on the shelf and the questions asked of it: the SimBrief
//! fetch and its cache policy, the local planner entry, the FMS load, the brief,
//! top of descent, ETA and named-fix queries. Split out of main.rs on 2026-09-16.

use super::*;

/// A question about a NAMED fix on the plan.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct FixQuery {
    /// Index into the plan's waypoints.
    pub(crate) idx: usize,
    pub(crate) ident: String,
    /// The fix after it, when the plan has one.
    pub(crate) next: Option<String>,
    /// Coordinates, for the distance.
    pub(crate) lat: f64,
    pub(crate) lon: f64,
    /// "what's after X" (true) vs "how far to X" (false).
    pub(crate) after: bool,
    /// Where the fix sits in the plan, for "fix 3 of 15".
    pub(crate) of: usize,
}

/// Edit distance between two short idents, after collapsing doubled letters
/// (whisper writes "pimmons" for PIMOS; the doubled m is its habit, not the
/// pilot's).
pub(crate) fn ident_distance(a: &str, b: &str) -> usize {
    fn collapse(s: &str) -> Vec<u8> {
        let mut out: Vec<u8> = Vec::new();
        for c in s.bytes() {
            if out.last() != Some(&c) {
                out.push(c);
            }
        }
        out
    }
    let (a, b) = (collapse(a), collapse(b));
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    for (i, ca) in a.iter().enumerate() {
        let mut cur = vec![i + 1];
        for (j, cb) in b.iter().enumerate() {
            let cost = usize::from(ca != cb);
            cur.push((prev[j] + cost).min(prev[j + 1] + 1).min(cur[j] + 1));
        }
        prev = cur;
    }
    prev[b.len()]
}

/// The plan waypoint a spoken token most plausibly names, if any. First
/// letter must agree (whisper gets the onset right), and the rest may drift
/// by up to two edits on a fix of four letters or more. Airports at either
/// end are excluded: "how far to LEMG" is the destination ETA's job.
pub(crate) fn match_ident<'a>(
    token: &str,
    waypoints: &'a [simbrief::Waypoint],
) -> Option<(usize, &'a simbrief::Waypoint)> {
    let tok = token.to_ascii_lowercase();
    if tok.len() < 3 {
        return None;
    }
    let last = waypoints.len().checked_sub(1)?;
    let mut best: Option<(usize, usize, &simbrief::Waypoint)> = None;
    for (i, wp) in waypoints.iter().enumerate() {
        if i == 0 || i == last {
            continue;
        }
        let id = wp.ident.to_ascii_lowercase();
        if id.len() < 3 || id.as_bytes()[0] != tok.as_bytes()[0] {
            continue;
        }
        let allowed = if id.len() >= 5 {
            2
        } else if id.len() == 4 {
            1
        } else {
            0
        };
        let d = ident_distance(&tok, &id);
        if d <= allowed && best.is_none_or(|(bd, _, _)| d < bd) {
            best = Some((d, i, wp));
        }
    }
    best.map(|(_, i, wp)| (i, wp))
}

/// "How much left for PIMOS", "what's after PIMOS", "when do we reach
/// PIMOS": a fix named after a preposition, matched against the plan.
/// Without a preposition there is no candidate, so ordinary chatter with
/// a fix-like word in it never answers.
pub(crate) fn fix_query(t: &str, waypoints: &[simbrief::Waypoint]) -> Option<FixQuery> {
    const PREPS: &[&str] = &[
        "to",
        "for",
        "at",
        "after",
        "past",
        "beyond",
        "following",
        "reach",
        "till",
        "until",
        "before",
    ];
    // The question has to be ABOUT a fix: a range cue, an "after" word, or
    // the word waypoint/fix. Without one, "set heading to 270" and friends
    // never get their object matched against the plan.
    let range_cue = [
        "how far", "how long", "how much", "when", "eta", "time", "distance", "miles", "minutes",
        "till", "until", "reach",
    ]
    .iter()
    .any(|c| t.contains(c));
    let after_word = ["after", "past", "beyond", "following"]
        .iter()
        .any(|w| t.contains(w));
    if !(range_cue || after_word || t.contains("waypoint") || t.contains("fix")) {
        return None;
    }
    let words: Vec<String> = t
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .map(str::to_ascii_lowercase)
        .collect();
    let mut found: Option<(usize, &simbrief::Waypoint, bool)> = None;
    for (i, w) in words.iter().enumerate() {
        if !PREPS.contains(&w.as_str()) {
            continue;
        }
        // "to the next waypoint" is the plain progress question, not a fix.
        let cand = words[i + 1..]
            .iter()
            .find(|c| !matches!(c.as_str(), "the" | "a" | "our" | "that" | "this"))?;
        if let Some((idx, wp)) = match_ident(cand, waypoints) {
            let after = matches!(w.as_str(), "after" | "past" | "beyond" | "following");
            found = Some((idx, wp, after));
            break;
        }
    }
    let (idx, wp, mut after) = found?;
    // "what's the waypoint ... PIMOS" with no distance cue: they want the plan
    // around the fix (what comes next), not a range to it.
    if !range_cue && (t.contains("waypoint") || t.contains("fix") || t.contains("next")) {
        after = true;
    }
    Some(FixQuery {
        idx,
        ident: wp.ident.clone(),
        next: waypoints.get(idx + 1).map(|n| n.ident.clone()),
        lat: wp.lat,
        lon: wp.lon,
        after,
        of: waypoints.len(),
    })
}

/// Distance and time to a named fix from where we are, and what follows it.
pub(crate) async fn fix_answer(q: &FixQuery, xp: &XPlaneClient) -> String {
    let Some((lat, lon, _)) = position(xp).await else {
        return "I can't read our position right now.".to_string();
    };
    let dist = nav::haversine_nm(lat, lon, q.lat, q.lon);
    let gs_kts = xp
        .get_float("sim/flightmodel/position/groundspeed")
        .await
        .unwrap_or(0.0)
        * MS_TO_KTS;
    let active = xp
        .get_string("sim/cockpit2/radios/indicators/gps_nav_id")
        .await
        .unwrap_or_default()
        .trim()
        .to_string();
    let time = if gs_kts >= 30.0 {
        let mins = (dist / gs_kts * 60.0).round() as i64;
        if mins >= 60 {
            format!(", {} hours {} minutes", mins / 60, mins % 60)
        } else {
            format!(", {mins} minutes")
        }
    } else {
        String::new()
    };
    let position_word = if active.eq_ignore_ascii_case(&q.ident) {
        "the active waypoint".to_string()
    } else {
        format!("fix {} of {}", q.idx + 1, q.of)
    };
    if q.after {
        return match &q.next {
            Some(n) => format!(
                "After {} comes {n}. {} is {position_word}, {dist:.0} miles{time}.",
                q.ident, q.ident
            ),
            None => format!(
                "{} is the last fix before the destination, {dist:.0} miles{time}.",
                q.ident
            ),
        };
    }
    format!(
        "{}, {dist:.0} miles{time}. That's {position_word}.",
        q.ident
    )
}

/// Time and distance to the top of descent ("how long before TOD"), with
/// fallbacks so it works whether or not the box has VNAV:
///   0. the procedure follower's OWN point while one flies (the number its
///      hands use; the ring below said "start down" three minutes early on
///      the LEJR-LXGB straight-in, 2026-09-14);
///   1. the FMS's own computed TOD distance (when the avionics publish one);
///   2. SimBrief's planned TOD ring (`tod_to_dest_nm`) measured back from the
///      destination, which most GA boxes never compute themselves;
///   3. the altitude rule of thumb, cruise_alt/1000 * 3.5 nm, as a last resort.
pub(crate) async fn time_to_tod(
    config: &Config,
    xp: &XPlaneClient,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
) -> String {
    const TOD_DIST: &str = "sim/cockpit2/radios/indicators/fms_distance_to_tod_pilot";
    // 0. The follower's own point, to its own altitude.
    if let Some(p) = procedure::planned_tod() {
        let alt = auto_status::commas((p.target_ft / 100.0).round() as i64 * 100);
        if p.started {
            return format!("We're already descending, down to {alt} feet at {}.", p.fix);
        }
        if p.to_tod_nm <= 0.0 {
            return format!("We're at top of descent, starting down to {alt} feet.");
        }
        return format!(
            "{} Then down to {alt} feet.",
            tod_phrase(p.to_tod_nm, xp).await
        );
    }
    // 1. The avionics' own VNAV TOD, if it computes one.
    let fms = xp.get_float(TOD_DIST).await.unwrap_or(0.0);
    if fms > 0.0 {
        return tod_phrase(fms, xp).await;
    }
    // 2/3. Derive it from the flight plan: the SimBrief TOD ring, else the formula.
    if let Err(msg) = ensure_plan(config, cache, PlanFetch::Cached).await {
        return msg;
    }
    let Some((_, fp)) = cache.as_ref() else {
        return "Couldn't fetch your SimBrief plan.".to_string();
    };
    let Some(dest) = fp.waypoints.last().cloned() else {
        return "No destination in the plan.".to_string();
    };
    let ring_nm = if fp.tod_to_dest_nm > 0.0 {
        fp.tod_to_dest_nm
    } else {
        fp.cruise_alt_ft / 1000.0 * TOD_NM_PER_1000FT
    };
    let (lat, lon) = match (
        xp.get_float("sim/flightmodel/position/latitude").await,
        xp.get_float("sim/flightmodel/position/longitude").await,
    ) {
        (Ok(a), Ok(b)) => (a, b),
        _ => return "I can't read our position right now.".to_string(),
    };
    let to_tod = nav::haversine_nm(lat, lon, dest.lat, dest.lon) - ring_nm;
    tod_phrase(to_tod, xp).await
}

/// Format a distance-to-TOD as a spoken line, paced by groundspeed.
pub(crate) async fn tod_phrase(dist_nm: f64, xp: &XPlaneClient) -> String {
    if dist_nm <= 0.0 {
        return "We're at top of descent, time to start down.".to_string();
    }
    let gs_kts = xp
        .get_float("sim/flightmodel/position/groundspeed")
        .await
        .unwrap_or(0.0)
        * MS_TO_KTS;
    if gs_kts < 30.0 {
        return format!("Top of descent about {dist_nm:.0} miles ahead.");
    }
    let mins = (dist_nm / gs_kts * 60.0).round() as i64;
    let time = if mins >= 60 {
        format!("{} hours {} minutes", mins / 60, mins % 60)
    } else {
        format!("{mins} minutes")
    };
    format!("About {time} to top of descent, {dist_nm:.0} miles ahead.")
}

/// Estimated time to the destination ("time before we land", "when do we get in"):
/// great-circle distance from our position to the destination airport over current
/// groundspeed. Needs the SimBrief plan (for the destination coordinates) and us
/// actually moving — a straight-line estimate, not the routed distance.
pub(crate) async fn eta_to_destination(
    config: &Config,
    xp: &XPlaneClient,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
) -> String {
    if let Err(msg) = ensure_plan(config, cache, PlanFetch::Cached).await {
        return msg;
    }
    let Some((_, fp)) = cache.as_ref() else {
        return "Couldn't fetch your SimBrief plan.".to_string();
    };
    let Some(dest) = fp.waypoints.last().cloned() else {
        return "No destination in the plan.".to_string();
    };
    let dest_name = if fp.destination.is_empty() {
        dest.ident.clone()
    } else {
        fp.destination.clone()
    };
    let (lat, lon, gs_ms) = match (
        xp.get_float("sim/flightmodel/position/latitude").await,
        xp.get_float("sim/flightmodel/position/longitude").await,
        xp.get_float("sim/flightmodel/position/groundspeed").await,
    ) {
        (Ok(a), Ok(b), Ok(c)) => (a, b, c),
        _ => return "I can't read our position right now.".to_string(),
    };
    let dist = nav::haversine_nm(lat, lon, dest.lat, dest.lon);
    let gs_kts = gs_ms * MS_TO_KTS;
    if gs_kts < 30.0 {
        return format!(
            "{dest_name} is {dist:.0} miles out. I'll have a time once we're up and moving."
        );
    }
    let mins = (dist / gs_kts * 60.0).round() as i64;
    let (h, m) = (mins / 60, mins % 60);
    let eta = if h > 0 {
        format!("{h} hours {m} minutes")
    } else {
        format!("{m} minutes")
    };
    format!("About {eta} to {dest_name}, {dist:.0} miles out at {gs_kts:.0} knots.")
}

/// How long a fetched SimBrief plan is reused by the IMPLICIT consumers (the
/// fuel projection, TOD, ETA, the destination weather) before re-fetching.
/// A spoken ask for the plan never waits on it (`PlanFetch::Asked`).
pub(crate) const FLIGHT_PLAN_TTL: Duration = Duration::from_secs(600);

/// Who is asking for the plan decides whether SimBrief is consulted.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum PlanFetch {
    /// A side consumer (fuel, TOD, ETA): the shelf plan within its TTL.
    Cached,
    /// The pilot asked for the plan by name ("load / what's the flight
    /// plan"): a SimBrief plan is re-fetched EVERY time, because the pilot
    /// regenerates it on the website and the words are the same ("load the
    /// flight plan" x4 in two minutes, LEMG-LEJR loaded four times from the
    /// shelf after "that's the wrong plan", 2026-09-14). A local plan stays:
    /// it is the pilot's own decision and "load it" must not swap it out.
    Asked,
    /// "refresh / reload / update": re-fetch even over a local plan.
    Forced,
}

/// Make sure `cache` holds a usable plan per `fetch`, keeping the cached one
/// when a re-fetch fails. `Ok(true)` = a fresh fetch replaced the shelf;
/// `Err` carries a spoken message.
pub(crate) async fn ensure_plan(
    config: &Config,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
    fetch: PlanFetch,
) -> Result<bool, String> {
    // Full auto may have expanded the local direct plan into a checked route.
    // Keep questions, fuel advice and later FMS loads on that same shelf.
    if let (Some((_, current)), Some(published)) = (cache.as_mut(), simbrief::local_plan()) {
        if current.source == simbrief::PlanSource::Local
            && current.origin == published.origin
            && current.destination == published.destination
        {
            *current = published;
        }
    }
    // A local plan (the direct-to planner) is the pilot's own decision: it
    // never expires into a SimBrief refetch, and needs no SimBrief ID. Only
    // an explicit refresh may swap it for SimBrief when that fetch succeeds.
    let local = cache
        .as_ref()
        .is_some_and(|(_, fp)| fp.source == simbrief::PlanSource::Local);
    let refresh = match fetch {
        PlanFetch::Cached => false,
        PlanFetch::Asked => !local,
        PlanFetch::Forced => true,
    };
    if config.simbrief_id.trim().is_empty() && !local {
        return Err(
            "No SimBrief ID set. Add XCOPILOT_SIMBRIEF_ID and I can pull your flight plan, \
             or say plan a flight to, and a destination."
                .to_string(),
        );
    }
    let fresh = cache
        .as_ref()
        .is_some_and(|(t, _)| t.elapsed() < FLIGHT_PLAN_TTL)
        || (local && !refresh);
    let mut fetched = false;
    if (refresh || !fresh) && !config.simbrief_id.trim().is_empty() {
        match simbrief::fetch(&config.simbrief_id).await {
            Ok(fp) => {
                *cache = Some((Instant::now(), fp));
                fetched = true;
            }
            Err(e) => {
                tracing::warn!("simbrief fetch failed: {e:#}");
                if cache.is_none() {
                    return Err(
                        "Couldn't fetch your SimBrief plan. Generate one first, or check the ID."
                            .to_string(),
                    );
                }
            }
        }
    }
    // Every path that yields a plan also hands the monitor's descent advisor
    // its destination — the copilot volunteers TOD instead of waiting to be
    // asked ("when is TOD" still works via time_to_tod).
    if let Some((_, fp)) = cache.as_ref() {
        simbrief::publish_plan(fp);
        if let Some(w) = fp.waypoints.last() {
            let ring = if fp.tod_to_dest_nm > 0.0 {
                fp.tod_to_dest_nm
            } else {
                fp.cruise_alt_ft / 1000.0 * TOD_NM_PER_1000FT
            };
            // Field elevation comes from apt.dat, the authoritative source — the
            // plan's destination-waypoint altitude is a *crossing* altitude
            // (spoken live as "4600 feet" for a 2,084 ft field, and it skews the
            // descent advisor's profile math too). Cached per ICAO: the scan of
            // the 400 MB file runs once per destination, not per TTL refresh.
            static ELEV_CACHE: std::sync::Mutex<Option<(String, f64)>> =
                std::sync::Mutex::new(None);
            let cached = ELEV_CACHE
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
                .filter(|(icao, _)| *icao == fp.destination);
            let elev_ft = match cached {
                Some((_, e)) => e,
                None => {
                    let (path, icao) = (config.apt_data_path.clone(), fp.destination.clone());
                    let scanned =
                        tokio::task::spawn_blocking(move || nav::find_airport(&path, &icao))
                            .await
                            .ok()
                            .and_then(|r| r.ok())
                            .flatten()
                            .map(|a| a.elev_ft);
                    // Fallback: the waypoint altitude when plausible, else sea
                    // level (which only makes TOD earlier — the safe direction).
                    let e = scanned.unwrap_or(if (0.0..12_000.0).contains(&w.alt) {
                        w.alt
                    } else {
                        0.0
                    });
                    *ELEV_CACHE.lock().unwrap_or_else(|e| e.into_inner()) =
                        Some((fp.destination.clone(), e));
                    e
                }
            };
            *monitor::DESTINATION
                .lock()
                .unwrap_or_else(|e| e.into_inner()) = Some(monitor::Destination {
                lat: w.lat,
                lon: w.lon,
                elev_ft,
                tod_ring_nm: ring,
            });
        }
    }
    Ok(fetched)
}

/// How long a bare "divert" / "plan a flight" waits for its "to where?" answer.
pub(crate) const PLAN_ASK_WINDOW: Duration = Duration::from_secs(30);

/// The local planner: a direct flight to a spoken destination, no SimBrief.
/// Resolves the field from apt.dat (and the field we are at or nearest to as
/// the origin), refuses a runway the profile cannot land on, picks the cruise
/// altitude by rule over the sim's own terrain, works the fuel from the
/// profile's cruise burn, and shelves the result where every other plan
/// consumer looks (GPS load, TOD, ETA, the monitor's destination). `load`
/// also sends it to the GPS in the same breath.
pub(crate) async fn plan_flight(
    config: &Config,
    xp: &XPlaneClient,
    profile: &aircraft::AircraftProfile,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
    req: &flightplan::PlanRequest,
) -> String {
    let Some(dest) = req.dest.as_ref() else {
        return "To where?".to_string();
    };
    let (lat, lon, fuel_kg) = tokio::join!(
        xp.get_float(datarefs::LATITUDE),
        xp.get_float(datarefs::LONGITUDE),
        xp.get_float(datarefs::FUEL_TOTAL_KG),
    );
    let (Ok(lat), Ok(lon)) = (lat, lon) else {
        return "I can't read our position, so I can't plan from here.".to_string();
    };
    // apt.dat is a ~400 MB streaming scan: off the async thread.
    let (path, d) = (config.apt_data_path.clone(), dest.clone());
    let resolved =
        match tokio::task::spawn_blocking(move || flightplan::resolve(&path, &d, (lat, lon))).await
        {
            Ok(Ok(r)) => r,
            Ok(Err(e)) => {
                tracing::warn!("flight plan: apt.dat scan failed: {e:#}");
                return "I can't read the airport database right now.".to_string();
            }
            Err(_) => return "The airport lookup died on me. Say it again.".to_string(),
        };
    let Some(to) = resolved.dest else {
        return format!(
            "I don't know an airport called {}. Try the ICAO code, letter by letter.",
            dest.spoken()
        );
    };
    let Some(from) = resolved.origin else {
        return "I can't find an airport near us to plan from.".to_string();
    };
    if from.icao == to.icao {
        return format!(
            "We're already at {}, {}.",
            to.name,
            flightplan::spell_code(&to.icao)
        );
    }
    let r = profile.reference.as_ref();
    let needed = flightplan::runway_needed_ft(
        r.and_then(|r| r.landing_distance_ft),
        r.and_then(|r| r.takeoff_distance_ft),
    );
    if let Some(line) = flightplan::runway_refusal(&to, needed) {
        blackbox::record(
            "flightplan",
            serde_json::json!({ "event": "refused", "dest": to.icao, "runway_ft": to.longest_runway_ft, "needed_ft": needed }),
        );
        return line;
    }
    let dist_nm = nav::haversine_nm(from.lat, from.lon, to.lat, to.lon);
    let bearing = nav::bearing_deg(from.lat, from.lon, to.lat, to.lon);
    let samples = flightplan::route_samples((from.lat, from.lon), (to.lat, to.lon));
    let max_terrain_ft = terrain::max_elevation_ft(xp, &samples).await;
    let kind = r
        .and_then(|r| r.kind.as_deref())
        .unwrap_or("")
        .to_ascii_lowercase();
    let pressurised = kind.contains("jet") || kind.contains("turboprop");
    let cruise_alt_ft = flightplan::cruise_altitude_ft(&flightplan::AltInputs {
        bearing_deg: bearing,
        dist_nm,
        origin_elev_ft: from.elev_ft,
        dest_elev_ft: to.elev_ft,
        max_terrain_ft,
        service_ceiling_ft: r.and_then(|r| r.service_ceiling_ft),
        pressurised,
        // The winds the level will actually be flown in. Best effort: a plan
        // without them is the plan we always made.
        winds: crate::weather::winds_aloft(xp).await,
    });
    let cruise_kts = r.and_then(|r| r.cruise_kts);
    let burn = profile
        .fuel
        .as_ref()
        .map(|f| f.cruise_burn_kg_hr)
        .unwrap_or(0.0);
    let planned = flightplan::Planned {
        dest_matches: resolved.dest_matches,
        origin: from,
        dest: to,
        dist_nm,
        bearing_deg: bearing,
        cruise_alt_ft,
        max_terrain_ft,
        fuel: flightplan::fuel_plan(dist_nm, cruise_kts, burn),
        cruise_kts,
        aboard_kg: fuel_kg.ok(),
        aircraft: profile.display_name.clone(),
    };
    blackbox::record(
        "flightplan",
        serde_json::json!({
            "event": "planned",
            "origin": planned.origin.icao,
            "dest": planned.dest.icao,
            "dist_nm": (dist_nm * 10.0).round() / 10.0,
            "bearing": bearing.round(),
            "cruise_alt_ft": cruise_alt_ft,
            "max_terrain_ft": max_terrain_ft,
            "block_kg": planned.fuel.as_ref().map(|f| f.block_kg),
            "aboard_kg": planned.aboard_kg,
            "load": req.load,
        }),
    );
    let fp = planned.to_flight_plan();
    simbrief::publish_plan(&fp);
    *cache = Some((Instant::now(), fp));
    // The same seam SimBrief uses to hand the monitor its destination (TOD).
    let _ = ensure_plan(config, cache, PlanFetch::Cached).await;
    let mut line = planned.brief(req.load);
    if req.load {
        line.push(' ');
        line.push_str(&load_route_to_fms(config, xp, cache, PlanFetch::Cached).await);
    }
    line
}

/// Load the SimBrief route into the GPS/FMS: hand the waypoints to the plugin
/// (only it can touch the FMS, via XPLMNavigation) and trigger the load.
pub(crate) async fn load_route_to_fms(
    config: &Config,
    xp: &XPlaneClient,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
    fetch: PlanFetch,
) -> String {
    let fetched = match ensure_plan(config, cache, fetch).await {
        Ok(f) => f,
        Err(msg) => return msg,
    };
    let Some((_, fp)) = cache.as_ref() else {
        return "Couldn't fetch your SimBrief plan.".to_string();
    };
    // A load that wanted SimBrief and got the shelf instead says so: the
    // pilot may have just regenerated the plan the site failed to serve.
    let stale =
        if !fetched && fetch != PlanFetch::Cached && fp.source == simbrief::PlanSource::SimBrief {
            "Couldn't reach SimBrief, using the plan I already had. "
        } else {
            ""
        };
    if fp.waypoints.is_empty() {
        return "No waypoints in the plan to load.".to_string();
    }
    // "IDENT,lat,lon,alt|..." — the plugin parses this and builds the FMS route,
    // thinned in the middle if the route is longer than its mailbox.
    let payload = simbrief::fms_route_payload(&fp.waypoints);
    let (origin, dest, n) = (
        fp.origin.clone(),
        fp.destination.clone(),
        fp.waypoints.len(),
    );
    if xp
        .set_data("xcopilot/fms/route", payload.as_bytes())
        .await
        .is_err()
        || xp.set_int("xcopilot/fms/load", 1).await.is_err()
    {
        return "Couldn't reach the plugin to load the route. Is the XVoiceLayer plugin running?"
            .to_string();
    }
    format!(
        "{stale}Loading {n} waypoints into the GPS, {origin} to {dest}. Check the flight plan page."
    )
}

/// Fetch (or reuse) the SimBrief flight plan and brief it. Deterministic (no LLM);
/// the one network call beyond X-Plane. `want_route` reads back the airway route;
/// otherwise it's a full brief: route summary, climb-to-cruise, top of descent,
/// and a fuel check against what's actually on board.
pub(crate) async fn flight_plan_report(
    config: &Config,
    xp: &XPlaneClient,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
    fetch: PlanFetch,
    want_route: bool,
) -> String {
    if let Err(msg) = ensure_plan(config, cache, fetch).await {
        return msg;
    }
    let Some((_, fp)) = cache.as_ref() else {
        return "Couldn't fetch your SimBrief plan.".to_string();
    };

    if want_route {
        return if fp.route.is_empty() {
            "No route in the plan.".to_string()
        } else {
            format!("Route, {} {} {}.", fp.origin, fp.route, fp.destination)
        };
    }

    let mut line = fp.summary();
    // Where we are vs planned cruise.
    // On the ground the whole climb is ahead and the summary already said
    // the cruise: "You're at 49, 4951 to climb to cruise" is a number
    // nobody asked for (LXGB, 2026-09-15).
    let on_ground = xp
        .get_float(datarefs::ON_GROUND_ANY)
        .await
        .is_ok_and(|v| v > 0.5);
    if on_ground {
    } else if let Ok(alt) = xp.get_float(datarefs::ALTITUDE_FT).await {
        let diff = fp.cruise_alt_ft - alt;
        if diff > 300.0 {
            line.push_str(&format!(
                " You're at {alt:.0} feet, {diff:.0} to climb to cruise."
            ));
        } else if diff < -300.0 {
            line.push_str(&format!(" You're at {alt:.0} feet, above planned cruise."));
        } else {
            line.push_str(" At planned cruise.");
        }
    }
    // Top of descent.
    if fp.tod_to_dest_nm > 0.5 {
        line.push_str(&format!(
            " Top of descent about {:.0} miles before {}.",
            fp.tod_to_dest_nm, fp.destination
        ));
    }
    // Fuel check: planned block against what's in the tanks. BOTH SIDES IN
    // KILOGRAMS — the sim's own unit and the plan's — converted to the
    // airframe's spoken unit only at the last moment, and ROUNDED THERE so
    // the three numbers the pilot hears actually subtract.
    if let Ok(kg) = xp.get_float(datarefs::FUEL_TOTAL_KG).await {
        let aboard = crate::aircraft::fuel_in_unit_kg(kg).round();
        let need = crate::aircraft::fuel_in_unit_kg(fp.block_fuel_kg).round();
        if aboard >= need {
            line.push_str(&format!(
                " You have {} aboard, enough for the plan.",
                crate::aircraft::say_fuel_figure(aboard)
            ));
        } else {
            line.push_str(&format!(
                " You have {} aboard, but the plan needs {}; short about {}.",
                crate::aircraft::say_fuel_figure(aboard),
                crate::aircraft::fuel_figure_kg(fp.block_fuel_kg),
                crate::aircraft::say_fuel_figure(need - aboard),
            ));
            // The crew can do the load sheet, on the ground only.
            if on_ground {
                line.push_str(" Say load the aircraft and I'll set it from the plan.");
            }
        }
    }
    line
}

/// "Check the flight plan": the arm's whole-flight audit, spoken in full,
/// without arming anything (`plan_audit`). The departure is probed only on
/// the ground at the origin, where that scenery is loaded.
pub(crate) async fn plan_check_report(
    config: &Config,
    xp: &XPlaneClient,
    cache: &mut Option<(Instant, simbrief::FlightPlan)>,
) -> String {
    if let Err(msg) = ensure_plan(config, cache, PlanFetch::Asked).await {
        return msg;
    }
    let Some((_, fp)) = cache.as_ref() else {
        return "Couldn't fetch your SimBrief plan.".to_string();
    };
    let fp = fp.clone();
    let mut legs =
        crate::procedure::enroute_legs(&fp.waypoints, &fp.origin, &fp.destination, fp.cruise_alt_ft);
    let (stars, _) = crate::plan_audit::arrival_restrictions(&config.cifp_dir, &fp, &mut legs);
    let (path, o, d) = (config.apt_data_path.clone(), fp.origin.clone(), fp.destination.clone());
    let (origin, dest) = tokio::task::spawn_blocking(move || {
        (
            crate::nav::find_airport(&path, &o).ok().flatten(),
            crate::nav::find_airport(&path, &d).ok().flatten(),
        )
    })
    .await
    .unwrap_or((None, None));
    let (ground, lat, lon) = tokio::join!(
        xp.get_float(datarefs::ON_GROUND_ANY),
        xp.get_float("sim/flightmodel/position/latitude"),
        xp.get_float("sim/flightmodel/position/longitude"),
    );
    let at_origin = ground.is_ok_and(|g| g > 0.5)
        && match (lat, lon, fp.waypoints.first()) {
            (Ok(la), Ok(lo), Some(w)) => crate::nav::haversine_nm(la, lo, w.lat, w.lon) < 5.0,
            _ => false,
        };
    let suit = crate::plan_audit::gather_suitability(xp, &fp, origin.as_ref(), dest.as_ref()).await;
    let audit = crate::plan_audit::check(
        xp,
        &config.fix_data_path,
        &fp,
        &legs,
        &stars,
        origin.as_ref().map_or(0.0, |a| a.elev_ft),
        crate::plan_audit::live_airframe(None),
        Some(&suit),
        at_origin,
    )
    .await;
    format!("{} to {}. {}", fp.origin, fp.destination, crate::plan_audit::spoken(&audit, true))
}

/// The fetch policy a spoken plan ask carries: "refresh / reload / update"
/// forces SimBrief even over a local plan; any other ask re-fetches a
/// SimBrief plan and keeps a local one.
pub(crate) fn plan_fetch_for(tl: &str) -> PlanFetch {
    if tl.contains("refresh") || tl.contains("reload") || tl.contains("update") {
        PlanFetch::Forced
    } else {
        PlanFetch::Asked
    }
}

/// "That's the wrong flight plan" / "not that plan": the pilot disowns the
/// plan on the shelf. Closed shape: a wrongness word AND a plan word
/// (whisper hears "flag plan", "flight plane", "flight lamp").
pub(crate) fn wrong_plan_complaint(tl: &str) -> bool {
    let wrong = tl.contains("wrong") || tl.contains("not that") || tl.contains("not the right");
    let plan = tl.contains("plan")
        || tl.contains("flag")
        || tl.contains("flight")
        || tl.contains("route")
        || tl.contains("root");
    wrong && plan
}

/// What a forced re-fetch after "wrong plan" found. Compares route identity
/// (origin, destination, airway string), never the timestamp: a SimBrief
/// plan re-served unchanged is still the same plan.
pub(crate) fn plan_change_line(
    before: Option<&simbrief::FlightPlan>,
    after: Option<&simbrief::FlightPlan>,
    fetched: bool,
) -> String {
    let Some(after) = after else {
        return "No plan on the shelf. Generate one on SimBrief, or say plan a flight to, and a \
                destination."
            .to_string();
    };
    if !fetched {
        return format!(
            "Couldn't reach SimBrief. I still have {} to {}.",
            after.origin, after.destination
        );
    }
    let same = before.is_some_and(|b| {
        b.origin == after.origin && b.destination == after.destination && b.route == after.route
    });
    if same {
        format!(
            "SimBrief still has {} to {}. Generate the new plan there, then say load the flight \
             plan.",
            after.origin, after.destination
        )
    } else {
        format!(
            "Got the new plan, {} to {}, {} waypoints. Say load it to send it to the GPS.",
            after.origin,
            after.destination,
            after.waypoints.len()
        )
    }
}
