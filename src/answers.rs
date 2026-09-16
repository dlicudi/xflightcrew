//! The state answerers: configuration, fuel, systems, traffic, weather, speed
//! verdicts and the reference card, all read from the same datarefs the tools
//! write. Split out of main.rs on 2026-09-16.

use super::*;

/// Read-only adapter: fuel has its own quantity/endurance reporter, distinct
/// from the instrument reader. Every other question stays on the state path.
pub(crate) async fn conversation_read(
    query: &str,
    xp: &XPlaneClient,
    profile: &aircraft::AircraftProfile,
) -> Option<String> {
    if is_fuel_query(query) {
        Some(fuel_report(xp, profile, None, is_endurance_query(query)).await)
    } else {
        answer_state_query(query, xp, None, profile).await
    }
}

/// The datarefs the live-state cache subscribes to over the WebSocket — the
/// continuously-read snapshot + autopilot-status set. (Arrays like engine-running,
/// and the tool path's read-after-write confirms, stay on REST.)
pub(crate) const SNAPSHOT_DATAREFS: &[&str] = &[
    datarefs::ALTITUDE_FT,
    datarefs::AIRSPEED_KTS,
    datarefs::VVI_FPM,
    datarefs::COMPASS_HEADING_DEG,
    datarefs::FLAP_RATIO,
    datarefs::GEAR_HANDLE_DOWN,
    datarefs::ON_GROUND_ANY,
    datarefs::FUEL_TOTAL_KG,
    datarefs::WIND_DIR_DEG_TRUE,
    datarefs::WIND_SPEED_MSC,
    datarefs::OAT_DEG_C,
    datarefs::VISIBILITY_SM,
    datarefs::QNH_PASCALS,
    datarefs::SERVOS_ON,
    datarefs::APPROACH_STATUS,
    datarefs::NAV_STATUS,
    datarefs::HEADING_STATUS,
    datarefs::COM1_FREQ_KHZ,
    DR_OPT_SNAP,
    DR_OPT_TONE,
];

/// The 3D-cockpit camera position (pilot's-head datarefs) — read to remember the
/// current view before a snap-to-panel, and written to restore it after. Order is
/// x, y, z (metres from CG), then psi, the, phi (view angles, degrees).
pub(crate) const HEAD_DREFS: [&str; 6] = [
    "sim/graphics/view/pilots_head_x",
    "sim/graphics/view/pilots_head_y",
    "sim/graphics/view/pilots_head_z",
    "sim/graphics/view/pilots_head_psi",
    "sim/graphics/view/pilots_head_the",
    "sim/graphics/view/pilots_head_phi",
];

/// Deterministic "where is X" answers from the profile's `locations:` map — the
/// spatial knowledge PDF manuals can't carry. Self-gated: only a where/location
/// question, and only for an item the profile actually knows (longest key match
/// wins, so "fuel selector" beats "fuel"). `None` falls through to normal handling.
pub(crate) fn location_answer(command: &str, profile: &aircraft::AircraftProfile) -> Option<String> {
    if profile.locations.is_empty() {
        return None;
    }
    let t = grammar::normalize(command);
    if !(t.contains("where") || t.contains("located") || t.contains("location")) {
        return None;
    }
    let mut best: Option<(&String, &String)> = None;
    for (key, desc) in &profile.locations {
        if t.contains(key.as_str()) && best.is_none_or(|(bk, _)| key.len() > bk.len()) {
            best = Some((key, desc));
        }
    }
    best.map(|(key, desc)| format!("The {key} is {desc}."))
}

/// Interpolate between two headings/angles along the shortest arc, degrees.
pub(crate) fn lerp_angle_deg(a: f64, b: f64, t: f64) -> f64 {
    let d = ((b - a) % 360.0 + 540.0) % 360.0 - 180.0;
    (a + d * t).rem_euclid(360.0)
}

/// Signed shortest difference b − a in degrees (−180..180).
pub(crate) fn angle_delta_deg(a: f64, b: f64) -> f64 {
    ((b - a) % 360.0 + 540.0) % 360.0 - 180.0
}

/// Read a dataref preferring the live-state cache (instant, no round-trip), with a
/// REST read as the fallback on a cache miss — so it's never worse than polling
/// and near-free once the WebSocket subscription is warm.
pub(crate) async fn read1(live: Option<&LiveState>, xp: &XPlaneClient, name: &str) -> Result<f64> {
    if let Some(v) = live.and_then(|l| l.get(name)) {
        return Ok(v);
    }
    xp.get_float(name).await
}

/// 100LL avgas ≈ 2.72 kg/gal (~6 lb/gal) — to report fuel in gallons.
pub(crate) const AVGAS_KG_PER_GAL: f64 = 2.72;

/// Pounds per kilogram — jet fuel flow/quantity is read in lb (airliner convention).
pub(crate) const LB_PER_KG: f64 = 2.20462;

/// Meters per second → knots.
pub(crate) const MS_TO_KTS: f64 = 1.943_844;

/// The active GPS waypoint with distance and time/ETA to it — answers "ETA",
/// "distance/time to next waypoint", "how far". Often phrased as a command, not a
/// question, so it's dispatched unconditionally rather than via answer_state_query.
/// "Are we good for landing?" — the landing-configuration check, concluded:
/// gear (where it retracts), flap state, speed against the approach reference,
/// and the landing-phase SOP rows (landing light). Went to "No reading for
/// that" live; this is the one question a copilot must answer well.
pub(crate) async fn landing_readiness(xp: &XPlaneClient, profile: &aircraft::AircraftProfile) -> String {
    let airborne = xp
        .get_float("sim/flightmodel/failures/onground_any")
        .await
        .map(|v| v < 0.5)
        .unwrap_or(true);
    if !airborne {
        return "We're already on the ground.".to_string();
    }
    let mut issues: Vec<String> = Vec::new();
    let mut goods: Vec<String> = Vec::new();
    if profile.retractable_gear {
        let down = xp
            .get_float("sim/cockpit/switches/gear_handle_status")
            .await
            .map(|v| v > 0.5)
            .unwrap_or(false);
        if down {
            goods.push("gear's down".to_string());
        } else {
            issues.push("gear's still up".to_string());
        }
    }
    if let Some(flaps) = &profile.flaps {
        let ratio = xp
            .get_float("sim/cockpit2/controls/flap_handle_deploy_ratio")
            .await
            .unwrap_or(0.0);
        let nearest = flaps
            .detents
            .iter()
            .min_by(|a, b| (a.ratio - ratio).abs().total_cmp(&(b.ratio - ratio).abs()));
        if let Some(d) = nearest {
            if d.ratio < 0.01 {
                issues.push("no flap yet".to_string());
            } else {
                goods.push(format!("flaps {}", d.spoken));
            }
        }
    }
    let ias = xp
        .get_float("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")
        .await
        .unwrap_or(0.0);
    if let Some(app) = profile.reference.as_ref().and_then(|r| r.approach_kts) {
        if ias > app + 15.0 {
            issues.push(format!(
                "{ias:.0} knots is fast against the {app:.0} reference"
            ));
        } else {
            goods.push(format!("{ias:.0} knots against a {app:.0} reference"));
        }
    } else {
        goods.push(format!("{ias:.0} knots",));
    }
    // Landing-phase SOP rows (landing light and friends).
    for item in &profile.advisor {
        if !item.phases.iter().any(|p| p == "landing") {
            continue;
        }
        let Ok(v) = xp.get_float(&item.dataref).await else {
            continue;
        };
        if (v > 0.5) != item.expect_on {
            issues.push(item.say.trim_end_matches('.').to_lowercase());
        }
    }
    if issues.is_empty() {
        format!("You're set: {}. Fly it in.", goods.join(", "))
    } else {
        let good_tail = if goods.is_empty() {
            String::new()
        } else {
            format!(" Rest is good: {}.", goods.join(", "))
        };
        format!("Not yet — {}.{good_tail}", issues.join("; "))
    }
}

/// "Do we need any lights on?" / "are we configured right?" — the profile's SOP
/// advisor table crossed with the live switches, spoken as a conclusion. Went to
/// "No reading for that" live: the table knew the answer, nothing asked it.
pub(crate) async fn config_advice(xp: &XPlaneClient, profile: &aircraft::AircraftProfile) -> String {
    if profile.advisor.is_empty() {
        return "No configuration table for this airframe yet.".to_string();
    }
    let airborne = xp
        .get_float("sim/flightmodel/failures/onground_any")
        .await
        .map(|v| v < 0.5)
        .unwrap_or(false);
    let phase = if airborne { "cruise" } else { "ground" };
    let mut wrong: Vec<String> = Vec::new();
    let mut arrival_items = false;
    for item in &profile.advisor {
        let Ok(v) = xp.get_float(&item.dataref).await else {
            continue;
        };
        let satisfied = (v > 0.5) == item.expect_on;
        if item.phases.iter().any(|p| p == phase) {
            if !satisfied {
                wrong.push(item.say.trim_end_matches('.').to_string());
            }
        } else if airborne
            && !satisfied
            && item
                .phases
                .iter()
                .any(|p| p == "approach" || p == "landing")
        {
            arrival_items = true;
        }
    }
    if wrong.is_empty() {
        let tail = if arrival_items {
            " One or two come back on for the arrival — I'll speak up."
        } else {
            ""
        };
        format!("You're set for {phase}; nothing needed right now.{tail}")
    } else {
        format!("Couple of things: {}.", wrong.join("; "))
    }
}

/// "Any traffic around?" — the nearest TCAS targets as clock-position callouts
/// ("11 o'clock, 3 miles, 500 feet above"), from the sim's TCAS arrays. Went
/// unanswered live ("No reading for that"); a copilot's traffic scan is exactly
/// the kind of question that must never be guessed.
pub(crate) async fn traffic_report(xp: &XPlaneClient) -> String {
    let (bear, dist, alt) = (
        xp.get_float_array("sim/cockpit2/tcas/indicators/relative_bearing_degs")
            .await,
        xp.get_float_array("sim/cockpit2/tcas/indicators/relative_distance_mtrs")
            .await,
        xp.get_float_array("sim/cockpit2/tcas/indicators/relative_altitude_mtrs")
            .await,
    );
    let (Ok(bear), Ok(dist), Ok(alt)) = (bear, dist, alt) else {
        return "No traffic picture available.".to_string();
    };
    // Slot 0 is own ship; live targets have a real distance.
    let all: Vec<(f64, f64, f64)> = dist
        .iter()
        .zip(&bear)
        .zip(&alt)
        .skip(1)
        .filter(|((d, _), _)| **d > 1.0)
        .map(|((d, b), a)| (*d, *b, *a))
        .collect();
    if all.is_empty() {
        return "No traffic on the scope.".to_string();
    }
    // A copilot's scan reports *factors* — inside ~15 nm and ±3,000 ft — not the
    // whole online-traffic feed (an airliner 24,000 ft overhead is scenery, and
    // the first live scan recited three of them plus "60 more further out").
    let mut targets: Vec<(f64, f64, f64)> = all
        .iter()
        .copied()
        .filter(|(d, _, a)| *d < 15.0 * 1852.0 && a.abs() < 3000.0 / 3.28084)
        .collect();
    if targets.is_empty() {
        let nearest = all
            .iter()
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .expect("non-empty");
        return format!(
            "No factor traffic. Nearest is {:.0} miles out, {:.0} feet {}.",
            nearest.0 / 1852.0,
            ((nearest.2 * 3.28084).abs() / 100.0).round() * 100.0,
            if nearest.2 > 0.0 { "above" } else { "below" }
        );
    }
    targets.sort_by(|a, b| a.0.total_cmp(&b.0));
    let calls: Vec<String> = targets
        .iter()
        .take(3)
        .map(|(d, b, a)| {
            let nm = d / 1852.0;
            let range = if nm < 1.0 {
                "under a mile".to_string()
            } else {
                format!("{nm:.0} miles")
            };
            let clock = match (((b + 15.0).rem_euclid(360.0)) / 30.0) as i32 {
                0 => 12,
                c => c,
            };
            let ft = ((a * 3.28084) / 100.0).round() * 100.0;
            let vert = if ft.abs() < 300.0 {
                "same altitude".to_string()
            } else if ft > 0.0 {
                format!("{ft:.0} feet above")
            } else {
                format!("{:.0} feet below", ft.abs())
            };
            format!("{clock} oclock, {range}, {vert}")
        })
        .collect();
    let more = targets.len().saturating_sub(3);
    let tail = if more > 0 {
        format!(", and {more} more further out")
    } else {
        String::new()
    };
    let noun = if targets.len() == 1 {
        "One target"
    } else {
        "Traffic"
    };
    format!("{noun}: {}{tail}.", calls.join("; "))
}

/// "Are we on course?" — answered from the HSI course deviation plus the active
/// waypoint, never the LLM (which once replied with the bare word "heading").
/// Positive hdef = the course is to our right, i.e. we're left of it.
pub(crate) async fn on_course_report(xp: &XPlaneClient, live: Option<&LiveState>) -> String {
    // Parked, the question has no meaning — and the naive answer was nonsense
    // ("On course. 0 miles, 0 minutes." spoken live on the ramp).
    let on_ground = xp
        .get_float("sim/flightmodel/failures/onground_any")
        .await
        .unwrap_or(0.0)
        > 0.5;
    let gs = xp
        .get_float("sim/flightmodel/position/groundspeed")
        .await
        .unwrap_or(0.0);
    if on_ground && gs < 15.0 {
        return "We're on the ground — course comes alive once we're flying the route.".to_string();
    }
    let progress = nav_progress(xp, live).await;
    if progress.starts_with("No active GPS waypoint") {
        return progress;
    }
    // The follower steers direct to its fix; the HSI needle still shows the
    // GPS's parked first leg, which is nobody's course.
    if procedure::active_leg().is_some() {
        return format!("On course, the autopilot is steering the plan. {progress}");
    }
    let dots = read1(
        live,
        xp,
        "sim/cockpit2/radios/indicators/hsi_hdef_dots_pilot",
    )
    .await
    .ok();
    let verdict = match dots {
        Some(d) if d.abs() <= 0.5 => "On course.".to_string(),
        Some(d) => format!(
            "{:.1} dots {} of course, correct {}.",
            d.abs(),
            if d > 0.0 { "left" } else { "right" },
            if d > 0.0 { "right" } else { "left" },
        ),
        None => String::new(),
    };
    format!("{verdict} {progress}").trim().to_string()
}

pub(crate) async fn nav_progress(xp: &XPlaneClient, live: Option<&LiveState>) -> String {
    // A procedure in flight steers its own legs; the sim's GPS is parked on
    // the plan's first entry and would name a fix passed half an hour ago.
    if let Some(leg) = procedure::active_leg() {
        let (lat, lon, gs) = tokio::join!(
            read1(live, xp, "sim/flightmodel/position/latitude"),
            read1(live, xp, "sim/flightmodel/position/longitude"),
            read1(live, xp, "sim/flightmodel/position/groundspeed"),
        );
        if let (Ok(lat), Ok(lon)) = (lat, lon) {
            let gs_kts = gs.unwrap_or(0.0) * MS_TO_KTS;
            let (dist, mins) = procedure::leg_progress(&leg, lat, lon, gs_kts);
            return progress_line(&leg.fix, Some(dist), mins);
        }
    }
    let wp = xp
        .get_string("sim/cockpit2/radios/indicators/gps_nav_id")
        .await
        .unwrap_or_default();
    let dist = read1(
        live,
        xp,
        "sim/cockpit2/radios/indicators/gps_dme_distance_nm",
    )
    .await
    .ok();
    let tmin = read1(live, xp, "sim/cockpit2/radios/indicators/gps_dme_time_min")
        .await
        .ok();
    if wp.trim().is_empty() && dist.is_none() {
        return "No active GPS waypoint. Load and navigate a route first.".to_string();
    }
    progress_line(wp.trim(), dist, tmin)
}

/// "Next waypoint X, N miles, M minutes." from whichever source knows the fix.
pub(crate) fn progress_line(wp: &str, dist: Option<f64>, tmin: Option<f64>) -> String {
    let mut s = String::new();
    if !wp.is_empty() {
        s.push_str(&format!("Next waypoint {wp}, "));
    }
    if let Some(d) = dist {
        s.push_str(&format!("{d:.0} miles",));
    }
    if let Some(tm) = tmin {
        let (h, mn) = ((tm / 60.0) as i32, (tm % 60.0).round() as i32);
        let time = if h > 0 {
            format!("{h} hours {mn} minutes")
        } else {
            format!("{mn} minutes")
        };
        s.push_str(&format!(", {time}"));
    }
    format!("{}.", s.trim_end_matches([',', ' ']))
}

/// Does the utterance ask about weather at all (the lot, or one element)?
pub(crate) fn weather_words(t: &str) -> bool {
    [
        "weather",
        "metar",
        "conditions",
        "wind",
        "visibility",
        "cloud",
        "ceiling",
        "qnh",
        "altimeter",
    ]
    .iter()
    .any(|w| t.contains(w))
}

/// Where a weather question points when it is not "here".
#[derive(Debug, PartialEq)]
pub(crate) enum RemoteWeather {
    /// The question named the destination but no plan is loaded.
    NoPlan,
    Station {
        icao: String,
        label: String,
    },
}

/// Resolve a weather question to a station OTHER than the one we are at:
/// the plan's destination ("on arrival", "at the destination", "when we
/// land"), its alternate, its origin ("back at departure"), or an ICAO
/// spoken after "at/for/in". `None` means the sim's own weather answers.
pub(crate) fn remote_weather_target(t: &str, plan: Option<&simbrief::FlightPlan>) -> Option<RemoteWeather> {
    let words: Vec<&str> = t.split_whitespace().collect();
    let has = |w: &str| words.contains(&w);
    let dest_cue = t.contains("arrival")
        || has("arrive")
        || t.contains("destination")
        || t.contains("when we land")
        || t.contains("at landing")
        || t.contains("on landing")
        || t.contains("for landing")
        || t.contains("where we're going")
        || t.contains("where were going")
        || t.contains("at the other end")
        || (has("expect") && !has("here"));
    let alt_cue = t.contains("alternate") || t.contains("diversion");
    let origin_cue = t.contains("departure") || t.contains("origin") || t.contains("back home");
    // An explicit ICAO after a preposition wins over the cues: "weather at LXGB".
    for (i, w) in words.iter().enumerate() {
        if matches!(*w, "at" | "for" | "in" | "into" | "over") {
            if let Some(n) = words.get(i + 1) {
                let n = n.trim_matches(|c: char| !c.is_ascii_alphabetic());
                if n.len() == 4 && n.chars().all(|c| c.is_ascii_alphabetic()) {
                    let icao = n.to_ascii_uppercase();
                    let known = plan.is_some_and(|fp| {
                        [&fp.origin, &fp.destination, &fp.alternate]
                            .iter()
                            .any(|s| s.eq_ignore_ascii_case(&icao))
                    });
                    // Not on the plan: only if the METAR file actually lists it,
                    // so "in this" or "at that" never becomes an airport.
                    let on_file = || {
                        weather::latest_file(&tools::autoland::xplane_root())
                            .and_then(|p| std::fs::read_to_string(p).ok())
                            .is_some_and(|text| weather::has_station(&text, &icao))
                    };
                    if known
                        || (![
                            "this", "that", "here", "what", "when", "them", "your", "some", "same",
                            "next", "last", "high", "much", "long", "wind", "hold", "will", "also",
                            "just", "them",
                        ]
                        .contains(&n)
                            && on_file())
                    {
                        return Some(RemoteWeather::Station {
                            label: icao.clone(),
                            icao,
                        });
                    }
                }
            }
        }
    }
    let pick = |icao: &str| {
        let icao = icao.trim();
        (!icao.is_empty()).then(|| RemoteWeather::Station {
            icao: icao.to_string(),
            label: icao.to_string(),
        })
    };
    if alt_cue {
        return match plan {
            None => Some(RemoteWeather::NoPlan),
            Some(fp) => pick(&fp.alternate).or(Some(RemoteWeather::NoPlan)),
        };
    }
    if dest_cue {
        return match plan {
            None => Some(RemoteWeather::NoPlan),
            Some(fp) => pick(&fp.destination).or(Some(RemoteWeather::NoPlan)),
        };
    }
    if origin_cue {
        return plan.and_then(|fp| pick(&fp.origin));
    }
    None
}

/// Which runway to use: the field's *actual* runways (apt.dat) crossed with the
/// live wind — deterministic, so it can only name a runway that exists. On the
/// ground it's the departure field, airborne it's the destination (using local
/// wind as the approximation). Replaces the LLM answering this from wind alone,
/// which once produced "runway 18" at a field that only has 10 and 28.
pub(crate) async fn runway_advice(
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
    let on_ground = xp
        .get_float("sim/flightmodel/failures/onground_any")
        .await
        .unwrap_or(1.0)
        > 0.5;
    let icao = if on_ground {
        fp.origin.clone()
    } else {
        fp.destination.clone()
    };
    // apt.dat is a ~400 MB streaming scan — off the async thread it goes.
    let (path, icao2) = (config.apt_data_path.clone(), icao.clone());
    let runways = tokio::task::spawn_blocking(move || nav::find_runways(&path, &icao2))
        .await
        .ok()
        .and_then(|r| r.ok())
        .unwrap_or_default();
    if runways.is_empty() {
        return format!("I don't have runway data for {icao}.");
    }
    let all: Vec<&str> = runways
        .iter()
        .flat_map(|r| [r.ident_a.as_str(), r.ident_b.as_str()])
        .collect();
    let wind_dir = xp
        .get_float(datarefs::WIND_DIR_DEG_TRUE)
        .await
        .unwrap_or(0.0);
    let wind_kts = xp.get_float(datarefs::WIND_SPEED_MSC).await.unwrap_or(0.0) * MS_TO_KTS;
    if wind_kts < 3.0 {
        return format!(
            "Wind's calm. Any runway works; {icao} has {}.",
            all.join(" and ")
        );
    }
    let Some((ident, _, headwind)) = nav::best_runway_for_wind(&runways, wind_dir, wind_kts) else {
        return format!("I don't have runway data for {icao}.");
    };
    let component = if headwind >= 0.0 {
        format!("{headwind:.0} knots of headwind")
    } else {
        format!("mind the {:.0} knots of tailwind", headwind.abs())
    };
    format!(
        "Wind's {wind_dir:.0} at {wind_kts:.0}. Runway {ident}, {component}. {icao} has {}.",
        all.join(" and ")
    )
}

/// First element of a per-engine array dataref (engine 1), or `None` on a failed
/// read. Engine indicators are arrays, so they bypass the scalar `read1`/cache.
pub(crate) async fn read_engine0(xp: &XPlaneClient, name: &str) -> Option<f64> {
    xp.get_float_array(name)
        .await
        .ok()
        .and_then(|v| v.first().copied())
}

/// Live "are all systems within safe parameters?" check — reads the engine,
/// vacuum, and electrical instruments and reports status against the same limits
/// the monitor uses, instead of reciting static POH numbers. (oil temp / CHT
/// datarefs read °F despite their names.) The deterministic answer to "check all
/// systems".
pub(crate) async fn systems_check(xp: &XPlaneClient, profile: &aircraft::AircraftProfile) -> String {
    let running = read_engine0(xp, "sim/flightmodel/engine/ENGN_running")
        .await
        .is_some_and(|v| v > 0.5);
    if !running {
        return "Engine's not running, nothing to check.".to_string();
    }
    let mut ok: Vec<String> = Vec::new();
    let mut flags: Vec<String> = Vec::new();
    if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_pressure_psi").await {
        // Judge against the sim's own gauge red-lines, not a fixed piston band:
        // the modeled PT6 sits ~127 PSI in normal cruise, which the old 20..=105
        // flagged all flight (seen live on the Evolution). Fall back to the
        // piston band when the .acf carries no sensible arc.
        let (lo, hi) = match (
            xp.get_float("sim/aircraft/limits/red_lo_oilP").await,
            xp.get_float("sim/aircraft/limits/red_hi_oilP").await,
        ) {
            (Ok(lo), Ok(hi)) if hi > lo && hi > 0.0 => (lo, hi),
            _ => (20.0, 105.0),
        };
        if (lo..=hi).contains(&v) {
            ok.push(format!("oil pressure {v:.0}"));
        } else {
            flags.push(format!("oil pressure {v:.0} pounds"));
        }
    }
    if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_temperature_deg_C").await
    {
        if v < 240.0 {
            ok.push(format!("oil temp {v:.0}"));
        } else {
            flags.push(format!("oil temp {v:.0} degrees"));
        }
    }
    // CHT is a piston reading; a turbine profile (no piston `engine` group) has
    // no cylinders to report.
    if profile.engine {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/CHT_deg_C").await {
            if v < 450.0 {
                ok.push(format!("C H T {v:.0}"));
            } else {
                flags.push(format!("C H T {v:.0} degrees"));
            }
        }
    }
    // Suction only matters where a vacuum gyro exists — a glass/turbine panel
    // (Evolution, SR22) has none, and the stale reading spoke "low vacuum 3.8"
    // on every check (seen live).
    if profile.directional_gyro {
        if let Ok(v) = xp.get_float("sim/cockpit/misc/vacuum").await {
            if v >= 4.0 {
                ok.push(format!("vacuum {v:.1}"));
            } else {
                flags.push(format!("low vacuum {v:.1}"));
            }
        }
    }
    if let Some(v) = read_engine0(xp, "sim/cockpit2/electrical/battery_amps").await {
        if v >= -5.0 {
            ok.push("charging".to_string());
        } else {
            flags.push(format!("battery discharging {:.0} amps", v.abs()));
        }
    }
    if flags.is_empty() {
        // Verdict first, gauge recital never: a crew member concludes ("all in
        // the green") and keeps the numbers in reserve — each one is a "say the
        // oil pressure" away. Rotating openers so the answer doesn't sound like
        // the same recording every time.
        const VERDICTS: &[&str] = &[
            "Checked it over: everything's in the green.",
            "All in the green, engine's happy.",
            "Looks good up here, nothing out of place.",
        ];
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let i = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) % VERDICTS.len();
        let charging = ok.iter().any(|s| s == "charging");
        format!(
            "{}{}",
            VERDICTS[i],
            if charging {
                " Battery's charging fine."
            } else {
                ""
            }
        )
    } else {
        // Something's off: the flags get the airtime, the rest gets one word.
        format!(
            "Mostly green, but watch this: {}. Everything else is normal.",
            flags.join(", ")
        )
    }
}

/// True for "why have we lost power?" / "the engine quit" phrasings — a
/// diagnosis request, not a throttle command. Adjacent "lost power" wording (not
/// bare "power" + "lost") so "reduce power, we lost too much altitude" still
/// commands; engine fail-words are past-tense so "stop the engine" still
/// commands too.
pub(crate) fn is_power_loss_query(t: &str) -> bool {
    t.contains("lost power")
        || t.contains("losing power")
        || t.contains("lose power")
        || t.contains("power loss")
        || t.contains("loss of power")
        || (t.contains("engine")
            && (t.contains("quit")
                || t.contains("stopped")
                || t.contains("died")
                || t.contains("dead")
                || t.contains("failed")
                || t.contains("failure")
                || t.contains("cut out")
                || t.contains("cutting out")
                || (t.contains("why") && (t.contains("lost") || t.contains("lose")))))
}

/// Deterministic power-loss diagnosis ("why have we lost power?") — walk the
/// engine's fuel and ignition chain and name the likely culprit, most common
/// first, with the remedy in the same breath (the pilot asking this is about to
/// be busy). Same trust model as `systems_check`: read the datarefs, never the
/// LLM.
pub(crate) async fn power_loss_diagnosis(xp: &XPlaneClient, profile: &aircraft::AircraftProfile) -> String {
    let running = read_engine0(xp, "sim/flightmodel/engine/ENGN_running")
        .await
        .is_some_and(|v| v > 0.5);
    let piston = profile.turboprop_start.is_none() && !profile.turbine_start;
    let mut causes: Vec<String> = Vec::new();

    // Fuel chain, most likely first. Selector OFF starves the engine within a
    // minute (and an xlua plugin reload can snap it there behind your back).
    let selector = xp
        .get_float("sim/cockpit2/fuel/fuel_tank_selector")
        .await
        .ok();
    if profile.fuel_selector.is_some() && selector.is_some_and(|s| s < 0.5) {
        causes.push("the fuel selector is off, select a tank now".to_string());
    }
    let tanks = xp
        .get_float_array(datarefs::FUEL_PER_TANK_KG)
        .await
        .unwrap_or_default();
    let total_kg: f64 = tanks.iter().sum();
    if !tanks.is_empty() && total_kg < 2.0 {
        causes.push("the tanks are dry, we're out of fuel".to_string());
    } else if tanks.len() == 2 {
        // Feeding from a dry side while the other wing still has fuel
        // (sim selector positions: 1 = left, 3 = right).
        match selector.map(|s| s.round() as i64) {
            Some(1) if tanks[0] < 1.0 && tanks[1] > 2.0 => {
                causes.push("the left tank is dry, switch to the right".to_string());
            }
            Some(3) if tanks[1] < 1.0 && tanks[0] > 2.0 => {
                causes.push("the right tank is dry, switch to the left".to_string());
            }
            _ => {}
        }
    }
    if piston {
        if read_engine0(xp, "sim/cockpit2/engine/actuators/mixture_ratio")
            .await
            .is_some_and(|m| m < 0.15)
        {
            causes.push("the mixture is at cutoff, push it rich".to_string());
        }
        if read_engine0(xp, "sim/cockpit2/engine/actuators/ignition_key")
            .await
            .is_some_and(|k| k < 0.5)
        {
            causes.push("the ignition is off, mags to both".to_string());
        }
    }
    // Fuel not reaching the engine with everything above in order.
    let flow_kg_s = xp
        .get_float_array(datarefs::FUEL_FLOW_KG_SEC)
        .await
        .map(|f| f.iter().sum::<f64>())
        .unwrap_or(0.0);
    if causes.is_empty() && !running && flow_kg_s < 1e-5 && total_kg > 2.0 {
        causes.push(
            "no fuel flow with fuel on board, check the boost pump and fuel pressure".to_string(),
        );
    }
    let throttle = read_engine0(xp, "sim/cockpit2/engine/actuators/throttle_ratio")
        .await
        .unwrap_or(0.0);
    if running && causes.is_empty() && throttle < 0.05 {
        causes.push("the throttle is closed".to_string());
    }

    if !causes.is_empty() {
        let state = if running {
            "Engine's still turning"
        } else {
            "Engine's stopped"
        };
        return format!("{state}. Likely cause: {}.", causes.join("; also "));
    }
    // Nothing conclusive: give the raw picture so the pilot can work the problem.
    let rpm = read_engine0(xp, "sim/cockpit2/engine/indicators/prop_speed_rpm")
        .await
        .unwrap_or(0.0);
    let gph = flow_kg_s * 3600.0 / AVGAS_KG_PER_GAL;
    let mut line = if running {
        format!(
            "Nothing obviously wrong. Engine's running at {rpm:.0} RPM, fuel flow {gph:.0} gallons an hour, throttle {:.0} percent",
            throttle * 100.0
        )
    } else {
        format!(
            "Engine's stopped and the fuel and ignition look set. Fuel flow {gph:.1}, throttle {:.0} percent",
            throttle * 100.0
        )
    };
    if profile.carb_heat {
        line.push_str(". Could be carburettor ice, try carb heat");
    }
    format!("{line}.")
}

/// A brief spoken answer to "what are we flying / tell me about this aircraft",
/// from the profile + the 172 reference figures, instead of the LLM declining for
/// lack of data. `None` if the utterance isn't asking about the aircraft.
pub(crate) fn aircraft_info(profile: &aircraft::AircraftProfile, t: &str) -> Option<String> {
    let asks = t.contains("what are we flying")
        || t.contains("what aircraft")
        || t.contains("what plane")
        || t.contains("what are we in")
        || ((t.contains("tell me about") || t.contains("describe") || t.contains("about the"))
            && (t.contains("aircraft")
                || t.contains("plane")
                || t.contains("cessna")
                || t.contains("172")
                || t.contains("skyhawk")));
    if !asks {
        return None;
    }
    let r = profile.reference.as_ref();
    // The type descriptor comes from the profile (e.g. "twin-engine jet"); without
    // one, just name the aircraft rather than assert an engine count.
    let mut line = match r.and_then(|r| r.kind.as_deref()) {
        Some(kind) => format!("We're in the {}, a {kind}.", profile.display_name),
        None => format!("We're in the {}.", profile.display_name),
    };
    if let Some(v) = r.and_then(|r| r.cruise_kts) {
        line.push_str(&format!(" Cruise about {v:.0} knots."));
    }
    if let Some(v) = profile.vne_kts {
        // Vne for piston singles, Vmo for jets — "max speed" reads right for both.
        line.push_str(&format!(" Max speed {v:.0}."));
    }
    if let Some(v) = r.and_then(|r| r.service_ceiling_ft) {
        line.push_str(&format!(" Service ceiling {} feet.", with_commas(v)));
    }
    if let (Some(g), Some(h)) = (
        r.and_then(|r| r.fuel_useable_gal),
        r.and_then(|r| r.endurance_hours),
    ) {
        line.push_str(&format!(
            " {g:.0} gallons useable for roughly {h:.0} hours.",
        ));
    }
    Some(line)
}

/// Static reference figures (V-speeds, weights, limits) answered as a spoken query
/// — the old "performance limits" checklist, now a question. Every figure comes
/// from the airframe profile ([`aircraft::ReferenceProfile`]), so it's per-aircraft
/// and not hardcoded. `None` if the profile has no figures or the utterance asks
/// about none of them.
pub(crate) fn reference_limits(profile: &aircraft::AircraftProfile, t: &str) -> Option<String> {
    let r = profile.reference.as_ref();
    let mut parts: Vec<String> = Vec::new();
    let kt = |v: f64| format!("{v:.0} knots");

    if t.contains("vne") || t.contains("never exceed") {
        if let Some(v) = profile.vne_kts {
            parts.push(format!("Vne {}", kt(v)));
        }
    }
    if t.contains("vno") || t.contains("max cruise") || t.contains("structural cruise") {
        if let Some(v) = r.and_then(|r| r.vno_kts) {
            parts.push(format!("Vno {}", kt(v)));
        }
    }
    if t.contains("maneuver") {
        if let Some(v) = r.and_then(|r| r.va_kts) {
            parts.push(format!("Va, maneuvering speed, {}", kt(v)));
        }
    }
    if t.contains("vfe")
        || t.contains("flap speed")
        || t.contains("flaps speed")
        || t.contains("flap limit")
    {
        if let Some(p) = vfe_phrase(profile) {
            parts.push(p);
        }
    }
    if t.contains("stall") {
        match (profile.vs_clean_kts, r.and_then(|r| r.vs0_kts)) {
            (Some(c), Some(f)) => {
                parts.push(format!("stall {c:.0} knots clean, {f:.0} with full flaps"))
            }
            (Some(c), None) => parts.push(format!("stall {c:.0} knots clean")),
            (None, Some(f)) => parts.push(format!("stall {f:.0} knots with full flaps")),
            (None, None) => {}
        }
    }
    if t.contains("vx") || t.contains("best angle") {
        if let Some(v) = r.and_then(|r| r.vx_kts) {
            parts.push(format!("Vx, best angle of climb, {}", kt(v)));
        }
    }
    if t.contains("vy") || t.contains("best rate") || t.contains("best climb") {
        if let Some(v) = r.and_then(|r| r.vy_kts) {
            parts.push(format!("Vy, best rate of climb, {}", kt(v)));
        }
    }
    // The twin pair. Asked for by name far more often than by definition ("what's
    // blue line?"), and absent on a single, where the profile leaves them unset.
    if t.contains("vyse") || t.contains("blue line") || t.contains("single engine climb") {
        if let Some(v) = r.and_then(|r| r.vyse_kts) {
            parts.push(format!(
                "blue line, Vyse, {} — single-engine best rate",
                kt(v)
            ));
        }
    }
    if t.contains("vmc") || t.contains("red line") || t.contains("minimum control") {
        if let Some(v) = r.and_then(|r| r.vmca_kts) {
            parts.push(format!("Vmca, single-engine minimum control, {}", kt(v)));
        }
    }
    if t.contains("rotat")
        || t.contains("liftoff")
        || t.contains("lift off")
        || t.contains("takeoff speed")
    {
        if let Some(v) = r.and_then(|r| r.vr_kts) {
            parts.push(format!("rotate at {}", kt(v)));
        }
    }
    if t.contains("glide") {
        if let Some(v) = r.and_then(|r| r.best_glide_kts) {
            parts.push(format!("best glide {}", kt(v)));
        }
    }
    // "approach" alone is a COMMAND word as often as a question word: "fly
    // the approach" / "shoot the approach" arm the procedure follower, and a
    // bare contains() swallowed the command here and answered "approach
    // around 100 knots" to a pilot trying to start the Malaga arrival
    // (2026-08-02). A speed question about the approach always carries a
    // speed word; require one.
    if t.contains("landing speed")
        || t.contains("vref")
        || (t.contains("approach")
            && (t.contains("speed") || t.contains("knots") || t.contains("how fast")))
    {
        if let Some(v) = r.and_then(|r| r.approach_kts) {
            parts.push(format!("approach around {}", kt(v)));
        }
    }
    if t.contains("crosswind") {
        if let Some(v) = r.and_then(|r| r.max_crosswind_kts) {
            parts.push(format!("max demonstrated crosswind {}", kt(v)));
        }
    }
    if t.contains("rate of climb") || t.contains("climb rate") {
        if let Some(v) = r.and_then(|r| r.best_climb_fpm) {
            parts.push(format!("best rate of climb about {v:.0} feet per minute"));
        }
    }
    if t.contains("gross weight") || t.contains("max weight") || t.contains("max gross") {
        if let Some(v) = r.and_then(|r| r.max_gross_lb) {
            parts.push(format!("max gross weight {} pounds", with_commas(v)));
        }
    }
    if t.contains("useful load") {
        if let Some(v) = r.and_then(|r| r.useful_load_lb) {
            parts.push(format!("useful load {} pounds", with_commas(v)));
        }
    }
    if t.contains("empty weight") {
        if let Some(v) = r.and_then(|r| r.empty_weight_lb) {
            parts.push(format!("empty weight {} pounds", with_commas(v)));
        }
    }
    if t.contains("service ceiling") || t.contains("max altitude") {
        if let Some(v) = r.and_then(|r| r.service_ceiling_ft) {
            parts.push(format!("service ceiling {} feet", with_commas(v)));
        }
    }
    if t.contains("fuel capacity") || t.contains("tank capacity") {
        match (
            r.and_then(|r| r.fuel_capacity_gal),
            r.and_then(|r| r.fuel_useable_gal),
        ) {
            (Some(c), Some(u)) => {
                parts.push(format!("fuel capacity {c:.0} gallons, {u:.0} useable"))
            }
            (Some(c), None) => parts.push(format!("fuel capacity {c:.0} gallons")),
            _ => {}
        }
    }
    if t.contains("takeoff distance") || t.contains("takeoff roll") {
        if let Some(v) = r.and_then(|r| r.takeoff_distance_ft) {
            parts.push(format!("takeoff distance about {} feet", with_commas(v)));
        }
    }
    if t.contains("landing distance") || t.contains("landing roll") {
        if let Some(v) = r.and_then(|r| r.landing_distance_ft) {
            parts.push(format!("landing distance about {} feet", with_commas(v)));
        }
    }
    if t.contains("oil") && (t.contains("limit") || t.contains("range") || t.contains("normal")) {
        if let (Some(tmin), Some(tmax), Some(pmin), Some(pmax)) = (
            r.and_then(|r| r.oil_temp_min_f),
            r.and_then(|r| r.oil_temp_max_f),
            r.and_then(|r| r.oil_press_min_psi),
            r.and_then(|r| r.oil_press_max_psi),
        ) {
            parts.push(format!(
                "oil temperature normal {tmin:.0} to {tmax:.0} Fahrenheit, oil pressure {pmin:.0} to {pmax:.0} PSI"
            ));
        }
    }
    if t.contains("redline")
        || t.contains("max rpm")
        || t.contains("maximum rpm")
        || t.contains("red line")
    {
        if let Some(v) = r.and_then(|r| r.max_rpm) {
            parts.push(format!("maximum RPM {}", with_commas(v)));
        }
    }
    // A general "performance limits" / "V-speeds" ask gets the key summary.
    if parts.is_empty()
        && (t.contains("performance limit")
            || t.contains("v speed")
            || t.contains("v-speed")
            || (t.contains("limits") && t.contains("performance")))
    {
        let mut s: Vec<String> = Vec::new();
        if let Some(v) = profile.vne_kts {
            s.push(format!("Vne {v:.0}"));
        }
        if let Some(v) = r.and_then(|r| r.vno_kts) {
            s.push(format!("Vno {v:.0}"));
        }
        if let Some(v) = r.and_then(|r| r.va_kts) {
            s.push(format!("Va {v:.0}"));
        }
        if let Some(v) = r.and_then(|r| r.vy_kts) {
            s.push(format!("Vy {v:.0}"));
        }
        if let Some(v) = r.and_then(|r| r.best_glide_kts) {
            s.push(format!("best glide {v:.0}"));
        }
        if let Some(v) = profile.vs_clean_kts {
            s.push(format!("stall {v:.0} clean"));
        }
        if let Some(v) = r.and_then(|r| r.max_gross_lb) {
            s.push(format!("max gross {} pounds", with_commas(v)));
        }
        if let Some(v) = r.and_then(|r| r.service_ceiling_ft) {
            s.push(format!("service ceiling {} feet", with_commas(v)));
        }
        if !s.is_empty() {
            parts.push(s.join(", "));
        }
    }
    (!parts.is_empty()).then(|| format!("{}.", parts.join(", ")))
}

/// Vfe phrase built from the flap detents' per-notch limits, grouped by speed:
/// "Vfe 110 at ten degrees, 85 at twenty degrees and full". `None` if no detent
/// carries a Vfe.
pub(crate) fn vfe_phrase(profile: &aircraft::AircraftProfile) -> Option<String> {
    let detents = &profile.flaps.as_ref()?.detents;
    let mut groups: Vec<(i64, Vec<String>)> = Vec::new();
    for d in detents {
        let Some(vfe) = d.vfe_kts else { continue };
        let v = vfe.round() as i64;
        match groups.last_mut() {
            Some(g) if g.0 == v => g.1.push(d.spoken.clone()),
            _ => groups.push((v, vec![d.spoken.clone()])),
        }
    }
    if groups.is_empty() {
        return None;
    }
    let phrase = groups
        .iter()
        .map(|(v, notches)| format!("{v} at {}", join_and(notches)))
        .collect::<Vec<_>>()
        .join(", ");
    Some(format!("Vfe {phrase}"))
}

/// Format an integer-valued number with thousands separators ("2558" -> "2,558").
pub(crate) fn with_commas(n: f64) -> String {
    let n = n.round() as i64;
    let digits = n.abs().to_string();
    let bytes = digits.as_bytes();
    let mut out = String::new();
    for (i, c) in bytes.iter().enumerate() {
        if i > 0 && (bytes.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(*c as char);
    }
    if n < 0 {
        format!("-{out}")
    } else {
        out
    }
}

/// True if the utterance is asking about fuel/endurance (a question we answer
/// deterministically), not commanding the fuel *system* (pump/selector).
pub(crate) fn is_fuel_query(text: &str) -> bool {
    let t = text.to_ascii_lowercase();
    let asks = t.contains("fuel") || t.contains("gas") || t.contains("endurance");
    // Exclude fuel *commands* — system switches (pump/selector) and the ground
    // refuel request ("refuel"/"top off"/"fill") — those are actions, not queries.
    let command = t.contains("pump")
        || t.contains("selector")
        || t.contains("boost")
        || t.contains("primer")
        || t.contains("refuel")
        || t.contains("top off")
        || t.contains("fill")
        // Tank-SELECTOR commands ("switch tanks", "switch fuel to right", "set
        // fuel to left") — these hijacked the quantity report (seen live, with and
        // without "tank" said). A swap verb or an explicit "to <side>" is a
        // command; "how much fuel in the left tank" has neither and stays a query.
        // "select"/"set"/"feed" alone stay tank-gated so phrasings like "what's
        // the fuel setting" keep reading as questions.
        || t.contains("switch") || t.contains("swap") || t.contains("alternate")
        || t.contains("rotate")
        || t.contains("to left") || t.contains("to right") || t.contains("to off")
        || (t.contains("tank")
            && (t.contains("change") || t.contains("select") || t.contains("set")
                || t.contains("feed")));
    // "fuel flow" is an engine-instrument read, answered in answer_state_query.
    asks && !command && !t.contains("flow")
}

/// True if the utterance is asking for taxi directions — the on-demand side
/// of the taxi guide ("which way?", "say the taxi route", "where's the
/// holding point"). Deliberately question-shaped: a taxi READBACK ("taxi to
/// holding point Alpha via Whiskey...") contains the same nouns but none of
/// the question markers, and must fall through to the radio path untouched.
pub(crate) fn is_taxi_query(text: &str) -> bool {
    let t = text.to_ascii_lowercase();
    let way = t.contains("which way")
        || t.contains("where do i go")
        || t.contains("where do we go")
        || t.contains("where are we going")
        || t.contains("where to now")
        || t.contains("which taxiway");
    let hp = t.contains("holding point")
        && (t.contains("where") || t.contains("which") || t.contains("how far"));
    let route = (t.contains("taxi") || t.contains(" route"))
        && (t.contains("what")
            || t.contains("which")
            || t.contains("where")
            || t.contains("say")
            || t.contains("repeat")
            || t.contains("remind"));
    way || hp || route
}

/// True when the fuel question is about *time* remaining ("how long will the fuel
/// last", "what's our endurance", "how much longer can we fly") rather than the
/// quantity on board. Drives `fuel_report` to lead with the duration.
pub(crate) fn is_endurance_query(text: &str) -> bool {
    let t = text.to_ascii_lowercase();
    t.contains("endurance")
        || t.contains("how long")
        || t.contains("how much longer")
        || (t.contains("last") && (t.contains("fuel") || t.contains("gas")))
}

/// Deterministic spoken fuel report — total in gallons, endurance from the
/// profile's cruise burn, and the left/right balance. Read straight from the
/// datarefs (cache when warm, REST otherwise); never goes near the LLM.
pub(crate) async fn fuel_report(
    xp: &XPlaneClient,
    profile: &aircraft::AircraftProfile,
    live: Option<&LiveState>,
    endurance_first: bool,
) -> String {
    let Ok(total_kg) = read1(live, xp, datarefs::FUEL_TOTAL_KG).await else {
        return "I can't read the fuel right now — check the connection to X-Plane.".to_string();
    };
    let gal = total_kg / AVGAS_KG_PER_GAL;

    // Endurance: prefer the live burn (sum of per-engine fuel flow) when the
    // engines are running, since it reflects the actual power setting; fall back
    // to the profile's cruise figure on the ground or with engines shut down.
    let live_burn_kg_hr = xp
        .get_float_array(datarefs::FUEL_FLOW_KG_SEC)
        .await
        .map(|f| f.iter().sum::<f64>() * 3600.0)
        .unwrap_or(0.0);
    let burn_kg_hr = if live_burn_kg_hr > 1.0 {
        live_burn_kg_hr
    } else {
        profile
            .fuel
            .as_ref()
            .map(|f| f.cruise_burn_kg_hr)
            .unwrap_or(0.0)
    };
    let basis = if live_burn_kg_hr > 1.0 {
        "at this burn"
    } else {
        "at cruise"
    };

    let dur = if burn_kg_hr > 0.0 {
        let hours = total_kg / burn_kg_hr;
        let (h, m) = (hours.trunc() as i32, (hours.fract() * 60.0).round() as i32);
        let mut d = String::new();
        if h > 0 {
            d.push_str(&format!("{h} hour{}", if h == 1 { "" } else { "s" }));
        }
        if m > 0 {
            if !d.is_empty() {
                d.push(' ');
            }
            d.push_str(&format!("{m} minute{}", if m == 1 { "" } else { "s" }));
        }
        d
    } else {
        String::new()
    };

    // An endurance question wants the *time* first; a plain fuel question leads
    // with the quantity and tacks the duration on.
    let mut line = if endurance_first && !dur.is_empty() {
        format!("About {dur} of fuel {basis}, {gal:.0} gallons on board")
    } else {
        let mut l = format!("{gal:.0} gallons on board");
        if !dur.is_empty() {
            l.push_str(&format!(", about {dur} {basis}"));
        }
        l
    };
    // Per-tank balance, when the airframe has two tanks.
    if let Ok(tanks) = xp.get_float_array(datarefs::FUEL_PER_TANK_KG).await {
        if tanks.len() == 2 {
            let (l, r) = (tanks[0] / AVGAS_KG_PER_GAL, tanks[1] / AVGAS_KG_PER_GAL);
            line.push_str(&format!(". Left {l:.0}, right {r:.0}"));
            if (l - r).abs() >= 5.0 {
                line.push_str(if l > r {
                    " — favor the right"
                } else {
                    " — favor the left"
                });
            }
        }
    }
    // The answer a copilot actually gives is not "what's in the tanks" but
    // "what's left when we get there": project to the planned destination at the
    // live burn and judge it against the airframe's reserve. Airborne with a
    // known destination and real groundspeed only.
    let dest = monitor::DESTINATION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone();
    if let Some(d) = dest {
        if live_burn_kg_hr > 1.0 {
            let (lat, lon, gs_ms) = (
                xp.get_float("sim/flightmodel/position/latitude")
                    .await
                    .unwrap_or(0.0),
                xp.get_float("sim/flightmodel/position/longitude")
                    .await
                    .unwrap_or(0.0),
                xp.get_float("sim/flightmodel/position/groundspeed")
                    .await
                    .unwrap_or(0.0),
            );
            let gs = gs_ms * MS_TO_KTS;
            if gs > 40.0 {
                let dist = nav::haversine_nm(lat, lon, d.lat, d.lon);
                let at_dest_kg = total_kg - live_burn_kg_hr * (dist / gs);
                let mins_left = (at_dest_kg.max(0.0) / live_burn_kg_hr) * 60.0;
                let reserve = profile
                    .fuel
                    .as_ref()
                    .map(|f| f.reserve_caution_min)
                    .unwrap_or(45.0);
                let at_dest_gal = at_dest_kg / AVGAS_KG_PER_GAL;
                let verdict = if at_dest_kg <= 0.0 {
                    "that doesn't reach the destination — plan a fuel stop".to_string()
                } else if mins_left >= reserve + 15.0 {
                    format!("comfortably above your {reserve:.0}-minute reserve")
                } else if mins_left >= reserve {
                    format!("just above your {reserve:.0}-minute reserve, keep it leaned")
                } else {
                    format!("below your {reserve:.0}-minute reserve — think about a stop short")
                };
                if at_dest_kg > 0.0 {
                    line.push_str(&format!(
                        ". Should land with about {at_dest_gal:.0} gallons, {verdict}"
                    ));
                } else {
                    line.push_str(&format!(". At this burn, {verdict}"));
                }
            }
        }
    }
    // A question — just answer it; "Roger" is for acknowledging a command.
    format!("{line}.")
}

/// Answer the common "what's our X" state questions deterministically, straight
/// from the datarefs — no LLM (it proved unreliable even with the data in hand).
/// Multiple fields can be asked at once ("altitude and heading"). Returns `None`
/// for anything not a recognized state read, to fall through to the LLM for the
/// open-ended questions ("do we have enough fuel to reach Bristol?"). Only call
/// this for utterances that are questions, so "set heading 270" still commands.
/// Is the battery voltage low (depleted state of charge)? System-class-aware, so
/// it doesn't flag a healthy 12V-system battery as low the way a fixed 24V
/// threshold would: ~24V-class systems sit near 24-28V (low below ~23), ~12V-class
/// near 12-14V (low below ~11.5). The magnitude selects the class.
pub(crate) fn battery_low(volts: f64) -> bool {
    if volts >= 18.0 {
        volts < 23.0 // 24V-class system
    } else {
        volts < 11.5 // 12V-class system
    }
}

/// Does a speed question ask for a judgement, not just the number?
pub(crate) fn speed_assessment_words(t: &str) -> bool {
    let words: Vec<&str> = t
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    let has = |w: &str| words.contains(&w);
    let about_speed = has("speed")
        || has("airspeed")
        || has("slow")
        || has("fast")
        || has("slowly")
        || has("quick");
    let judged = has("okay")
        || has("ok")
        || has("alright")
        || has("fine")
        || has("good")
        || has("right")
        || has("normal")
        || has("safe")
        || has("enough")
        || has("too")
        || has("bit")
        || has("slow")
        || has("fast")
        || has("slowly")
        || has("comfortable")
        || has("happy");
    about_speed && judged && !t.contains("how fast")
}

/// What the aircraft is doing, for judging its airspeed.
pub(crate) struct SpeedContext {
    pub(crate) on_ground: bool,
    pub(crate) vvi_fpm: f64,
    pub(crate) agl_ft: f64,
    pub(crate) flap_ratio: f64,
    /// The governor's flight target, when it is holding one.
    pub(crate) hold_kts: Option<f64>,
    /// The airframe's floor (1.2 x clean Vs), republished per swap.
    pub(crate) floor_kts: Option<f64>,
}

/// The verdict on an indicated airspeed: the governor's target first, then
/// the floor, then the reference speed for the phase (Vy climbing, the
/// approach reference low and descending or with flap out, cruise IAS
/// otherwise). `None` when nothing in the profile can judge it, so the
/// copilot says the number and no more rather than inventing a standard.
pub(crate) fn speed_verdict(
    ias: f64,
    c: &SpeedContext,
    r: Option<&aircraft::ReferenceProfile>,
) -> Option<String> {
    if c.on_ground {
        return None;
    }
    if let Some(floor) = c.floor_kts {
        if ias < floor {
            return Some(format!(
                "that's below the {floor:.0} knot floor, one point two times stall. Nose down or power up."
            ));
        }
    }
    if let Some(h) = c.hold_kts {
        let d = ias - h;
        return Some(if d.abs() <= 3.0 {
            format!("I'm holding {h:.0}, on target.")
        } else if d < 0.0 {
            format!("I'm holding {h:.0}, {:.0} slow and correcting.", -d)
        } else {
            format!("I'm holding {h:.0}, {d:.0} fast and correcting.")
        });
    }
    let r = r?;
    let climbing = c.vvi_fpm > 300.0;
    let approaching = (c.agl_ft < 3000.0 && c.vvi_fpm < -100.0) || c.flap_ratio > 0.3;
    if climbing {
        let vy = r.vy_kts?;
        let d = ias - vy;
        return Some(if d.abs() <= 5.0 {
            format!("we're climbing at Vy, {vy:.0}, that's right.")
        } else if d < 0.0 {
            format!("we're climbing, Vy is {vy:.0}, so a touch slow. Lower the nose a little.")
        } else {
            format!("we're climbing, Vy is {vy:.0}, so a bit fast, fine for cooling.")
        });
    }
    if approaching {
        let app = r.approach_kts?;
        let d = ias - app;
        return Some(if d.abs() <= 5.0 {
            format!("on the {app:.0} knot approach reference.")
        } else if d < 0.0 {
            format!("below the {app:.0} knot approach reference. Add power.")
        } else {
            format!("above the {app:.0} knot approach reference, {d:.0} fast.")
        });
    }
    let cruise = r.cruise_ias_kts.or(r.cruise_kts)?;
    let d = ias - cruise;
    Some(if d.abs() <= cruise * 0.1 {
        format!("normal cruise is {cruise:.0}, so about right.")
    } else if d < 0.0 {
        format!("normal cruise is {cruise:.0}, so we're slow. Check the power.")
    } else {
        format!("above the normal {cruise:.0} cruise.")
    })
}

pub(crate) async fn answer_state_query(
    cmd: &str,
    xp: &XPlaneClient,
    live: Option<&LiveState>,
    profile: &aircraft::AircraftProfile,
) -> Option<String> {
    let t = cmd.to_ascii_lowercase();
    // A turbine (jet) reports N1/N2/ITT, not the piston RPM/CHT metrics.
    let turbine = !profile.engine && profile.throttle;
    let mut parts: Vec<String> = Vec::new();

    // Configuration state — "is the gear down?", "where are the flaps?",
    // "is the parking brake set?". Answered from the SAME datarefs the tools
    // write (the pitot switch honours the profile override, so a study-level
    // airframe's relocated switch never makes this a lie). These only run on
    // question-shaped input, so "gear down" stays a command.
    if t.contains("flap") {
        if let Ok(v) = read1(live, xp, "sim/cockpit2/controls/flap_ratio").await {
            parts.push(if v <= 0.01 {
                "flaps up".to_string()
            } else if v >= 0.99 {
                "flaps full".to_string()
            } else {
                format!("flaps {:.0} percent", v * 100.0)
            });
        }
    }
    if t.contains("gear") {
        if let Ok(v) = read1(live, xp, "sim/cockpit2/controls/gear_handle_down").await {
            parts.push(if v > 0.5 {
                "gear down".to_string()
            } else {
                "gear up".to_string()
            });
        }
    }
    if t.contains("parking brake") || t.contains("park brake") || t.contains("handbrake") {
        if let Ok(v) = read1(live, xp, "sim/cockpit2/controls/parking_brake_ratio").await {
            parts.push(if v > 0.5 {
                "parking brake set".to_string()
            } else {
                "parking brake released".to_string()
            });
        }
    }
    if t.contains("pitot") {
        let dref = profile
            .pitot_heat_dataref
            .as_deref()
            .unwrap_or("sim/cockpit2/ice/ice_pitot_heat_on_pilot");
        if let Ok(v) = read1(live, xp, dref).await {
            parts.push(format!("pitot heat {}", if v > 0.5 { "on" } else { "off" }));
        }
    }
    // "What's the squawk code?" went to the LLM and came back as the words
    // "Squawk code." (LEMG, 2026-09-12). The code and the mode are two
    // datarefs the transponder tools already write.
    if t.contains("squawk") || t.contains("transponder") {
        if let Ok(c) = read1(live, xp, datarefs::TRANSPONDER_CODE).await {
            let code = atis::spoken_digits(&format!("{:04}", c.round() as i64));
            let mode = match read1(live, xp, "sim/cockpit2/radios/actuators/transponder_mode")
                .await
                .map(|m| m.round() as i64)
            {
                Ok(0) => ", transponder off",
                Ok(1) => ", standby",
                Ok(2) => ", on",
                Ok(3) => ", altitude",
                Ok(4) => ", test",
                Ok(5) => ", ground",
                _ => "",
            };
            parts.push(format!("squawk {code}{mode}"));
        }
    }
    // "Where are we?" (not "where are we going", the plan's question): the
    // position report a pilot gives, from the instruments, the leg being
    // flown and the plan's destination. Went to the LLM three times and came
    // back as an echo of the question (LEMG, 2026-09-12).
    let where_now = (t.contains("where are we")
        || t.contains("where am i")
        || t.contains("our position")
        || t.contains("position report")
        || t.contains("where exactly are we"))
        && !t.contains("going")
        && !t.contains("headed")
        && !t.contains("heading");
    if where_now {
        let (alt, hdg, gs, on_ground, lat, lon) = tokio::join!(
            read1(live, xp, datarefs::ALTITUDE_FT),
            read1(live, xp, datarefs::COMPASS_HEADING_DEG),
            read1(live, xp, "sim/flightmodel/position/groundspeed"),
            read1(live, xp, datarefs::ON_GROUND_ANY),
            read1(live, xp, datarefs::LATITUDE),
            read1(live, xp, datarefs::LONGITUDE),
        );
        if on_ground.is_ok_and(|g| g > 0.5) {
            parts.push("on the ground".to_string());
        } else if let Ok(a) = alt {
            parts.push(format!(
                "{} feet",
                auto_status::commas((a / 10.0).round() as i64 * 10)
            ));
        }
        if let Ok(h) = hdg {
            parts.push(format!("heading {:03}", (h.round() as i64).rem_euclid(360)));
        }
        if let Ok(g) = gs {
            let kts = g * 1.943_84;
            if kts >= 5.0 {
                parts.push(format!("{kts:.0} knots over the ground"));
            }
        }
        if let (Ok(lat), Ok(lon)) = (lat, lon) {
            if let Some(leg) = procedure::active_leg() {
                let d = nav::haversine_nm(lat, lon, leg.lat, leg.lon);
                parts.push(format!("{d:.0} miles to {}", leg.fix));
            }
            let dest = monitor::DESTINATION
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .as_ref()
                .map(|d| (d.lat, d.lon));
            if let Some((dlat, dlon)) = dest {
                let d = nav::haversine_nm(lat, lon, dlat, dlon);
                let b = nav::bearing_deg(lat, lon, dlat, dlon);
                parts.push(format!(
                    "{d:.0} miles to the destination, bearing {:03}",
                    (b.round() as i64).rem_euclid(360)
                ));
            }
        }
    }

    // System-wide failure check — reads the COMPLETE X-Plane failure set (see
    // failures.rs), never a keyword subset, so a failure with an off-pattern name
    // (a seized engine lives at rel_seize_0, no "engine" in it) can't hide. It's
    // an early return because the scan is ~800 concurrent reads — a beat slower
    // than a single value, so it only runs when the pilot actually asks.
    let asks_failures = t.contains("what's wrong")
        || t.contains("whats wrong")
        || t.contains("anything wrong")
        || t.contains("any failure")
        || t.contains("failures")
        || t.contains("anything fail")
        || t.contains("anything broken")
        || t.contains("malfunction")
        || t.contains("systems check")
        || t.contains("system status")
        || t.contains("systems status")
        || t.contains("status check")
        || t.contains("all systems")
        || t.contains("everything ok");
    if asks_failures {
        return Some(match failures::scan(xp).await {
            Ok(f) => failures::summary(&f)
                .unwrap_or_else(|| "All systems normal, no active failures.".to_string()),
            Err(_) => "I couldn't read the aircraft failure system.".to_string(),
        });
    }

    // Current control settings (e.g. "what's the mixture/throttle set to").
    if t.contains("mixture") {
        if let Ok(v) = read1(live, xp, "sim/cockpit2/engine/actuators/mixture_ratio_all").await {
            parts.push(format!("mixture {:.0} percent", v * 100.0));
        }
    }
    // "autothrottle" / "auto throttle" is the autoflight state, not the lever
    // position — answer the engage state and don't let the bare-throttle branch
    // below also fire ("autothrottle" contains "throttle").
    let autothrottle_q =
        t.contains("autothrottle") || t.contains("auto throttle") || t.contains("auto-throttle");
    if autothrottle_q {
        if let Ok(v) = read1(live, xp, "sim/cockpit2/autopilot/autothrottle_enabled").await {
            parts.push(match v {
                v if v >= 1.5 => "autothrottle engaged".to_string(),
                v if v >= 0.5 => "autothrottle armed".to_string(),
                _ => "autothrottle off".to_string(),
            });
        }
    }
    if t.contains("throttle") && !autothrottle_q {
        if let Ok(v) = read1(live, xp, "sim/cockpit2/engine/actuators/throttle_ratio_all").await {
            parts.push(format!("throttle {:.0} percent", v * 100.0));
        }
    }

    if t.contains("altitude") || t.contains("how high") {
        if let Ok(v) = read1(live, xp, datarefs::ALTITUDE_FT).await {
            parts.push(format!("{v:.0} feet"));
        }
    }
    if t.contains("heading") {
        if let Ok(v) = read1(live, xp, datarefs::COMPASS_HEADING_DEG).await {
            parts.push(format!("heading {v:03.0}"));
        }
    }
    let vs_q = t.contains("vertical speed") || t.contains("climb rate") || t.contains("rate of");
    if vs_q {
        if let Ok(v) = read1(live, xp, datarefs::VVI_FPM).await {
            parts.push(format!("{v:+.0} feet per minute"));
        }
    }
    let groundspeed_q = t.contains("groundspeed") || t.contains("ground speed");
    if groundspeed_q {
        if let Ok(v) = xp.get_float("sim/flightmodel/position/groundspeed").await {
            parts.push(format!("groundspeed {:.0} knots", v * MS_TO_KTS));
        }
    }
    // "speed" alone → airspeed, unless a different speed was explicitly named.
    // "Is our speed okay?" / "aren't we going a bit slow?" want a VERDICT
    // with the number: against what the governor holds, else the airframe's
    // reference for the phase we are in. A bare "72 knots" answered neither
    // (live, 2026-09-12).
    let speed_judged = speed_assessment_words(&t);
    if t.contains("airspeed")
        || t.contains("air speed")
        || (!groundspeed_q && (t.contains("how fast") || (t.contains("speed") && !vs_q)))
        || speed_judged
    {
        if let Ok(v) = read1(live, xp, datarefs::AIRSPEED_KTS).await {
            let mut line = format!("{v:.0} knots");
            if speed_judged {
                let (on_ground, vvi, agl_m, flap) = tokio::join!(
                    read1(live, xp, datarefs::ON_GROUND_ANY),
                    read1(live, xp, datarefs::VVI_FPM),
                    read1(live, xp, datarefs::AGL_METERS),
                    read1(live, xp, datarefs::FLAP_RATIO),
                );
                let ctx = SpeedContext {
                    on_ground: on_ground.map(|g| g > 0.5).unwrap_or(false),
                    vvi_fpm: vvi.unwrap_or(0.0),
                    agl_ft: agl_m.map(|m| m * 3.28084).unwrap_or(f64::MAX),
                    flap_ratio: flap.unwrap_or(0.0),
                    hold_kts: tools::speedhold::flight_target(),
                    floor_kts: aircraft::speed_floor_kts(),
                };
                if let Some(verdict) = speed_verdict(v, &ctx, profile.reference.as_ref()) {
                    line.push_str(", ");
                    line.push_str(&verdict);
                }
            }
            parts.push(line);
        }
    }
    // Weather — answered deterministically, not via the LLM (same reliability fix
    // as fuel). "weather" gives the lot; specific words give just that field.
    let wx = t.contains("weather") || t.contains("metar") || t.contains("conditions");
    if wx || t.contains("wind") {
        if let (Ok(dir), Ok(spd_ms)) = (
            read1(live, xp, datarefs::WIND_DIR_DEG_TRUE).await,
            read1(live, xp, datarefs::WIND_SPEED_MSC).await,
        ) {
            let kts = spd_ms * MS_TO_KTS;
            parts.push(if kts < 1.0 {
                "wind calm".to_string()
            } else {
                format!("wind {dir:03.0} at {kts:.0} knots")
            });
        }
    }
    if wx || t.contains("visibility") {
        if let Ok(v) = read1(live, xp, datarefs::VISIBILITY_SM).await {
            parts.push(format!("visibility {v:.0} statute miles"));
        }
    }
    // "pressure" alone could mean the altimeter or the oil — exclude oil pressure
    // so "how's the oil pressure?" doesn't also blurt the altimeter setting.
    if wx
        || t.contains("altimeter")
        || t.contains("qnh")
        || t.contains("barometric")
        || (t.contains("pressure") && !t.contains("oil"))
    {
        if let Ok(v) = read1(live, xp, datarefs::QNH_PASCALS).await {
            // qnh in Pascals → altimeter setting in inHg (1 inHg = 3386.389 Pa).
            parts.push(format!("altimeter {:.2}", v / 3386.389));
        }
    }
    if wx
        || (t.contains("temperature") && !t.contains("oil"))
        || t.contains("how cold")
        || t.contains("how hot")
        || t.contains(" oat")
    {
        if let Ok(v) = read1(live, xp, datarefs::OAT_DEG_C).await {
            parts.push(format!("outside air temperature {v:.0} degrees celsius"));
        }
    }

    // General "how's the engine?" → a one-line health summary. Only when no
    // specific instrument is named (those are handled below, no double-up).
    if t.contains("engine")
        && ![
            "oil",
            "egt",
            "exhaust",
            "itt",
            "cht",
            "cylinder",
            "rpm",
            "n1",
            "n2",
            "fuel flow",
        ]
        .iter()
        .any(|k| t.contains(k))
    {
        let running = read_engine0(xp, "sim/flightmodel/engine/ENGN_running")
            .await
            .is_some_and(|v| v > 0.5);
        if !running {
            parts.push(if turbine { "engines off" } else { "engine off" }.to_string());
        } else if turbine {
            // Turbine summary: spool speeds (N1/N2), turbine temp (ITT), oil, burn.
            let mut bits = vec!["engines running".to_string()];
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N1_percent").await {
                bits.push(format!("N 1 {v:.0} percent"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N2_percent").await {
                bits.push(format!("N 2 {v:.0} percent"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/ITT_deg_C").await {
                bits.push(format!("I T T {v:.0} degrees"));
            }
            if let Some(v) =
                read_engine0(xp, "sim/cockpit2/engine/indicators/oil_pressure_psi").await
            {
                bits.push(format!("oil pressure {v:.0} pounds"));
            }
            if let Some(v) =
                read_engine0(xp, "sim/cockpit2/engine/indicators/fuel_flow_kg_sec").await
            {
                bits.push(format!(
                    "fuel flow {:.0} pounds an hour",
                    v * 3600.0 * LB_PER_KG
                ));
            }
            parts.push(bits.join(", "));
        } else {
            let mut bits = vec!["engine running".to_string()];
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/prop_speed_rpm").await
            {
                bits.push(format!("{v:.0} R P M"));
            }
            if let Some(v) =
                read_engine0(xp, "sim/cockpit2/engine/indicators/oil_pressure_psi").await
            {
                bits.push(format!("oil pressure {v:.0} pounds"));
            }
            // oil_temp / CHT datarefs read °F despite their names.
            if let Some(v) =
                read_engine0(xp, "sim/cockpit2/engine/indicators/oil_temperature_deg_C").await
            {
                bits.push(format!("oil temp {v:.0} degrees"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/CHT_deg_C").await {
                bits.push(format!("C H T {v:.0}"));
            }
            if let Some(v) =
                read_engine0(xp, "sim/cockpit2/engine/indicators/fuel_flow_kg_sec").await
            {
                bits.push(format!(
                    "fuel flow {:.1} gallons an hour",
                    v * 3600.0 / AVGAS_KG_PER_GAL
                ));
            }
            parts.push(bits.join(", "));
        }
    }

    // Turbine spool speeds, asked directly ("what's the N1?").
    if turbine && (t.contains("n1") || t.contains("n 1")) {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N1_percent").await {
            parts.push(format!("N 1 {v:.0} percent"));
        }
    }
    if turbine && (t.contains("n2") || t.contains("n 2")) {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N2_percent").await {
            parts.push(format!("N 2 {v:.0} percent"));
        }
    }
    if turbine && (t.contains("itt") || t.contains("turbine temp")) {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/ITT_deg_C").await {
            parts.push(format!("I T T {v:.0} degrees"));
        }
    }

    // Engine instruments (per-engine arrays → element 0). Temperatures spoken in
    // °F to match the 172's gauges and the POH. Names verified against the stock
    // 172's cockpitdecks mapping.
    let c_to_f = |c: f64| c * 9.0 / 5.0 + 32.0;
    if t.contains("oil pressure") {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/oil_pressure_psi").await {
            parts.push(format!("oil pressure {v:.0} pounds"));
        }
    }
    if t.contains("oil temp") || (t.contains("oil") && t.contains("temperature")) {
        // oil_temperature_deg_C is misnamed — reads °F already.
        if let Some(v) =
            read_engine0(xp, "sim/cockpit2/engine/indicators/oil_temperature_deg_C").await
        {
            parts.push(format!("oil temperature {v:.0} degrees"));
        }
    }
    if !turbine && (t.contains("egt") || t.contains("exhaust")) {
        // EGT_deg_cel is genuinely Celsius → convert to °F for the readout. (On a
        // turbine the equivalent is ITT, handled by the n1/n2/itt block above; route
        // "exhaust temp" there too so a jet doesn't fall through to nothing.)
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/EGT_deg_cel").await {
            parts.push(format!("exhaust gas temperature {:.0} degrees", c_to_f(v)));
        }
    }
    if turbine && (t.contains("egt") || t.contains("exhaust")) {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/ITT_deg_C").await {
            parts.push(format!("I T T {v:.0} degrees"));
        }
    }
    if t.contains("cht") || t.contains("cylinder") {
        if turbine {
            parts.push("no cylinder head temp on a jet — that's a piston gauge".to_string());
        } else if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/CHT_deg_C").await {
            // CHT_deg_C is misnamed — reads °F already.
            parts.push(format!("cylinder head temperature {v:.0} degrees"));
        }
    }
    if t.contains("rpm") || t.contains("r p m") || t.contains("engine speed") {
        if turbine {
            // A jet's "engine speed" is the spool speeds, not prop RPM.
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N1_percent").await {
                parts.push(format!("N 1 {v:.0} percent"));
            }
            if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/N2_percent").await {
                parts.push(format!("N 2 {v:.0} percent"));
            }
        } else if let Some(v) =
            read_engine0(xp, "sim/cockpit2/engine/indicators/prop_speed_rpm").await
        {
            parts.push(format!("{v:.0} R P M"));
        }
    }
    if t.contains("fuel flow") || t.contains("flow") || t.contains("burning") {
        if let Some(v) = read_engine0(xp, "sim/cockpit2/engine/indicators/fuel_flow_kg_sec").await {
            if turbine {
                parts.push(format!(
                    "fuel flow {:.0} pounds an hour",
                    v * 3600.0 * LB_PER_KG
                ));
            } else {
                parts.push(format!(
                    "fuel flow {:.1} gallons an hour",
                    v * 3600.0 / AVGAS_KG_PER_GAL
                ));
            }
        }
    }
    if t.contains("ammeter")
        || t.contains("amps")
        || t.contains("charging")
        || t.contains("battery")
        || t.contains("voltage")
        || t.contains("volts")
        || t.contains("state of charge")
    {
        // Report VOLTAGE and amps together. Amps alone is misleading: a dead-flat
        // battery reads "+15 amps charging" while sitting at 20 volts, going
        // nowhere (seen live). Voltage is the state-of-charge/health number; amps
        // is only the direction of flow. The copilot used to read only amps.
        if let Some(v) =
            read_engine0(xp, "sim/cockpit2/electrical/battery_voltage_actual_volts").await
        {
            let health = if battery_low(v) { " (low)" } else { "" };
            parts.push(format!("battery {v:.1} volts{health}"));
        }
        if let Some(v) = read_engine0(xp, "sim/cockpit2/electrical/bus_volts").await {
            parts.push(format!("bus {v:.1} volts"));
        }
        if let Some(v) = read_engine0(xp, "sim/cockpit2/electrical/battery_amps").await {
            let state = if v > 1.0 {
                "charging"
            } else if v < -1.0 {
                "discharging"
            } else {
                "steady"
            };
            parts.push(format!("{v:+.0} amps, {state}"));
        }
    }
    if t.contains("vacuum") || t.contains("suction") {
        if let Ok(v) = read1(live, xp, "sim/cockpit/misc/vacuum").await {
            parts.push(format!("suction {v:.1} inches"));
        }
    }
    // A question — answer plainly; "Roger" is a command acknowledgment.
    (!parts.is_empty()).then(|| format!("{}.", parts.join(", ")))
}
