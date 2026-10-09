//! Traffic advisories and near-miss calls.

use super::*;

/// The nearest airborne factor from the TCAS picture: range, clock position,
/// and relative altitude (positive = traffic above us), rounded to the
/// hundred feet a pilot scans in; with what Doc 4444 12.3.1 asks a
/// controller to add where known: the direction of flight and the type.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TrafficFactor {
    pub nm: f64,
    pub clock: i32,
    pub ft: f64,
    /// The target's direction of flight, spoken ("westbound"), when its
    /// track is known.
    pub bound: Option<&'static str>,
    /// The target's type, spoken ("A320"), when the picture names it.
    pub type_spoken: Option<String>,
    /// The installed field's dialect: the safety alert's words are the
    /// field's (FAA 7110.65 2-1-6 "traffic alert ... advise you ...
    /// immediately"; ICAO Doc 4444 "turn left immediately ... to avoid
    /// traffic"; CAP 413 "avoiding action").
    pub dialect: Dialect,
    /// The installed field speaks CAP 413 ([`speaks_cap413`]).
    pub uk: bool,
}

impl TrafficFactor {
    /// A factor from its three measurements, nothing else known.
    pub fn new(nm: f64, clock: i32, ft: f64) -> TrafficFactor {
        TrafficFactor {
            nm,
            clock,
            ft,
            ..TrafficFactor::default()
        }
    }
}

/// The traffic's aircraft types, by RAW callsign: what a controller names
/// traffic by (a conditional line-up, a traffic information). Fed by the
/// sim's TCAS table ([`read_tcas_targets`]'s `icao_type`) and by our own
/// fleet (`world::note_traffic_type`). The field's: reset by `install`
/// and `uninstall` ([`StationField`]).
pub(crate) static TRAFFIC_TYPES: Mutex<Option<std::collections::HashMap<String, String>>> =
    Mutex::new(None);

/// Note a traffic aircraft's ICAO type designator ("A320", "B738") by its
/// raw callsign, as the TCAS picture names it. Our own fleet's producer
/// calls it (the TCAS target carries no type).
pub fn note_traffic_type(callsign: &str, icao_type: &str) {
    let (cs, ty) = (callsign.trim(), icao_type.trim());
    if cs.is_empty() || ty.is_empty() {
        return;
    }
    TRAFFIC_TYPES
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .get_or_insert_with(Default::default)
        .insert(cs.to_string(), ty.to_ascii_uppercase());
}

/// A traffic aircraft's type designator, when known.
pub(crate) fn traffic_type(callsign: &str) -> Option<String> {
    TRAFFIC_TYPES
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .and_then(|m| m.get(callsign.trim()).cloned())
}

/// An ICAO type designator as a controller says it: "A320" stays "A320"
/// (TTS reads it "A three twenty"), "B738" is "Boeing 737", "AT76" "ATR",
/// "C172" "Cessna". `None` for a designator not in the table: an unknown
/// type is "traffic", never a guess.
pub fn spoken_type(icao_type: &str) -> Option<String> {
    let t = icao_type.trim().to_ascii_uppercase();
    let name = match t.as_str() {
        "A318" | "A319" | "A320" | "A321" | "A332" | "A333" | "A339" | "A343" | "A346" | "A359"
        | "A35K" | "A388" => {
            // "A332" is an "A330", "A359" an "A350": the family as said.
            match &t[..3] {
                "A33" => "A330",
                "A34" => "A340",
                "A35" => "A350",
                "A38" => "A380",
                _ => return Some(t),
            }
        }
        "A20N" => "A320",
        "A21N" => "A321",
        "A19N" => "A319",
        "B737" | "B738" | "B739" | "B38M" | "B39M" | "B736" => "Boeing 737",
        "B744" | "B748" => "Boeing 747",
        "B752" | "B753" => "Boeing 757",
        "B763" | "B764" => "Boeing 767",
        "B772" | "B773" | "B77W" | "B77L" => "Boeing 777",
        "B788" | "B789" | "B78X" => "Boeing 787",
        "E190" | "E195" | "E290" | "E295" | "E170" | "E175" => "Embraer",
        "CRJ9" | "CRJ7" | "CRJ2" | "CRJX" => "CRJ",
        "AT72" | "AT75" | "AT76" | "AT45" | "AT43" => "ATR",
        "DH8D" | "DH8C" | "DH8A" | "DH8B" => "Dash 8",
        "C172" | "C152" | "C182" | "C150" | "C206" | "C208" => "Cessna",
        "PA28" | "P28A" | "PA32" | "PA34" | "PA44" => "Piper",
        "SR20" | "SR22" | "S22T" => "Cirrus",
        "DA40" | "DA42" | "DA62" => "Diamond",
        "BE58" | "BE36" | "BE9L" | "BE20" | "B350" => "Beechcraft",
        "C25A" | "C25B" | "C525" | "C510" | "C56X" | "C68A" => "Citation",
        _ => return None,
    };
    Some(name.to_string())
}

/// A direction of flight as a controller says it, from a true track:
/// eight points, "northbound" ... "north westbound".
pub(super) fn bound_word(track_deg: f64) -> &'static str {
    const WORDS: [&str; 8] = [
        "northbound",
        "north eastbound",
        "eastbound",
        "south eastbound",
        "southbound",
        "south westbound",
        "westbound",
        "north westbound",
    ];
    WORDS[((track_deg.rem_euclid(360.0) + 22.5) / 45.0) as usize % 8]
}

/// Is the target BEHIND us and moving AWAY: in our rear half (four to
/// eight o'clock) and its track pointing more than 90 degrees from the
/// line back to us. We fly forward, away from it; it flies away from us:
/// it can never become a factor, and a controller does not call it.
/// `rel_bearing` is the target's bearing off our nose, `rel_track` its
/// track off our heading. Unknown track: not judged (called as before).
pub(super) fn behind_and_diverging(rel_bearing_deg: f64, rel_track_deg: Option<f64>) -> bool {
    let Some(track) = rel_track_deg else {
        return false;
    };
    let b = rel_bearing_deg.rem_euclid(360.0);
    if !(105.0..=255.0).contains(&b) {
        return false;
    }
    let to_us = (b + 180.0).rem_euclid(360.0);
    let off = (track - to_us).rem_euclid(360.0);
    let off = off.min(360.0 - off);
    off > 90.0
}

/// Where each picture target was last seen, by raw callsign, for its
/// track over the ground: our own fleet's TCAS targets carry no heading.
/// Self-expiring: a sample older than [`TRACK_MEMORY_S`] is never used.
pub(super) static TRACK_MEMORY: Mutex<
    Option<std::collections::HashMap<String, (f64, f64, Instant)>>,
> = Mutex::new(None);

/// A track sample this old says nothing about where the target is going.
const TRACK_MEMORY_S: f64 = 30.0;

/// The track over the ground of a picture target since it was last seen
/// (at least 2 s and 30 m ago), true degrees; the sample is updated.
fn track_from_memory(t: &TcasTarget, now: Instant) -> Option<f64> {
    let mut guard = TRACK_MEMORY.lock().unwrap_or_else(|e| e.into_inner());
    let mem = guard.get_or_insert_with(Default::default);
    mem.retain(|_, (_, _, at)| now.saturating_duration_since(*at).as_secs_f64() < TRACK_MEMORY_S);
    let track = mem.get(&t.callsign).and_then(|(lat, lon, at)| {
        let moved = metres_between(*lat, *lon, t.lat, t.lon);
        (now.saturating_duration_since(*at).as_secs_f64() >= 2.0 && moved >= 30.0)
            .then(|| crate::nav::bearing_deg(*lat, *lon, t.lat, t.lon))
    });
    let keep = match (track, mem.get(&t.callsign)) {
        (None, Some((_, _, at))) => now.saturating_duration_since(*at).as_secs_f64() < 2.0,
        _ => false,
    };
    if !keep {
        mem.insert(t.callsign.clone(), (t.lat, t.lon, now));
    }
    track
}

/// The nearest TCAS target inside 6 nm and ±1,500 ft; `None` = no factor.
/// Reads the sim's TCAS arrays, which any traffic source (LiveTraffic,
/// default AI aircraft) populates — ATC narrates traffic, never controls it.
///
/// While our own fleet flies ([`traffic_picture`]) the factor is ITS
/// nearest aircraft, from our position and heading (the reads the monitor's
/// snapshot already made this tick): the sim's TCAS holds the same aircraft
/// once the plugin draws them, and every traffic read is the one picture.
pub async fn nearest_traffic_factor(xp: SimView<'_>) -> Option<TrafficFactor> {
    if let Some(picture) = traffic_picture() {
        let (Ok(lat), Ok(lon), Ok(ele_m), Ok(hdg)) = (
            xp.get_float(datarefs::LATITUDE).await,
            xp.get_float(datarefs::LONGITUDE).await,
            xp.get_float(datarefs::ELEVATION_M).await,
            xp.get_float(datarefs::TRUE_HEADING_DEG).await,
        ) else {
            return None;
        };
        return factor_from_picture(lat, lon, ele_m * 3.28084, hdg, &picture);
    }
    let bear = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_bearing_degs")
        .await
        .ok()?;
    let dist = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_distance_mtrs")
        .await
        .ok()?;
    let alt = xp
        .get_float_array("sim/cockpit2/tcas/indicators/relative_altitude_mtrs")
        .await
        .ok()?;
    // Weight-on-wheels, same slot indexing as the indicator arrays: used to drop
    // ground traffic. Optional — an empty read fails open (announce), since the
    // ±1,500 ft gate already excludes ground targets when we're up at altitude.
    let wow = xp
        .get_float_array(datarefs::TCAS_WOW)
        .await
        .unwrap_or_default();
    // The direction of flight and the type, same slot indexing: optional,
    // a failed read only leaves them out of the call (and a target with no
    // track is never judged diverging).
    let psi = xp
        .get_float_array("sim/cockpit2/tcas/targets/position/psi")
        .await
        .unwrap_or_default();
    let hdg = xp.get_float(datarefs::TRUE_HEADING_DEG).await.ok();
    let types: Vec<Option<String>> = xp
        .get_bytes("sim/cockpit2/tcas/targets/icao_type")
        .await
        .map(|b| {
            b.chunks(8)
                .map(|c| {
                    let t = String::from_utf8_lossy(c)
                        .trim_matches('\0')
                        .trim()
                        .to_string();
                    (!t.is_empty()).then_some(t)
                })
                .collect()
        })
        .unwrap_or_default();
    // A slot with no aircraft reads psi 0: only slots with a range have one.
    let tracks: Vec<Option<f64>> = psi
        .iter()
        .enumerate()
        .map(|(i, p)| (hdg.is_some() && dist.get(i).is_some_and(|d| *d > 1.0)).then_some(*p))
        .collect();
    select_traffic_factor_with(&dist, &bear, &alt, &wow, &tracks, hdg, &types)
        .map(with_field_phraseology)
}

/// The installed field's dialect on a factor, for its safety alert.
fn with_field_phraseology(f: TrafficFactor) -> TrafficFactor {
    let (dialect, uk) = installed_phraseology();
    TrafficFactor { dialect, uk, ..f }
}

/// [`nearest_traffic_factor`] over a picture of targets, from our position
/// (feet MSL) and true heading: the same selection as the sim's indicator
/// arrays ([`select_traffic_factor`]), slot 0 standing for us. Each
/// target's track is its movement since it was last seen (our fleet's
/// targets carry no heading), its type the type table's.
pub(super) fn factor_from_picture(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    hdg_deg: f64,
    picture: &[TcasTarget],
) -> Option<TrafficFactor> {
    factor_from_picture_at(lat, lon, alt_ft, hdg_deg, picture, Instant::now())
        .map(with_field_phraseology)
}

/// [`factor_from_picture`] on a given clock, for tests.
pub(super) fn factor_from_picture_at(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    hdg_deg: f64,
    picture: &[TcasTarget],
    now: Instant,
) -> Option<TrafficFactor> {
    let (mut dist, mut bear, mut alt, mut wow) = (vec![0.0], vec![0.0], vec![0.0], vec![0.0]);
    let (mut tracks, mut types) = (vec![None], vec![None]);
    for t in picture {
        dist.push(crate::nav::haversine_nm(lat, lon, t.lat, t.lon) * 1852.0);
        bear.push((crate::nav::bearing_deg(lat, lon, t.lat, t.lon) - hdg_deg).rem_euclid(360.0));
        alt.push((t.ele_ft - alt_ft) / 3.28084);
        wow.push(if t.on_ground { 1.0 } else { 0.0 });
        tracks.push(track_from_memory(t, now));
        types.push(traffic_type(&t.callsign));
    }
    select_traffic_factor_with(&dist, &bear, &alt, &wow, &tracks, Some(hdg_deg), &types)
}

/// Pick the nearest airborne factor from the parallel TCAS indicator arrays.
/// Slot 0 is own ship. A target with weight on wheels is SKIPPED — a parked or
/// taxiing aircraft at the field sits within 6 nm and (from pattern height)
/// ±1,500 ft, and was being called "traffic, three o'clock, same altitude" on
/// the ramp (2026-07-24). Pure, so the ground filter is unit-tested.
#[cfg(test)]
pub(super) fn select_traffic_factor(
    dist: &[f64],
    bear: &[f64],
    alt: &[f64],
    wow: &[f64],
) -> Option<TrafficFactor> {
    select_traffic_factor_with(dist, bear, alt, wow, &[], None, &[])
}

/// [`select_traffic_factor`] with each slot's true track and type where
/// known, and our true heading: a target behind us and moving away is
/// never a factor ([`behind_and_diverging`]; the next nearest is called),
/// and the call names the direction of flight and the type. Pure.
pub(super) fn select_traffic_factor_with(
    dist: &[f64],
    bear: &[f64],
    alt: &[f64],
    wow: &[f64],
    track_true: &[Option<f64>],
    own_hdg: Option<f64>,
    types: &[Option<String>],
) -> Option<TrafficFactor> {
    let track = |i: usize| track_true.get(i).copied().flatten();
    let (i, d, b, a) = (1..dist.len())
        .filter(|&i| dist[i] > 1.0)
        // On the ground = not an airborne factor. Fail open when unfed.
        .filter(|&i| wow.get(i).is_none_or(|w| *w < 0.5))
        .map(|i| {
            (
                i,
                dist[i],
                *bear.get(i).unwrap_or(&0.0),
                *alt.get(i).unwrap_or(&0.0),
            )
        })
        .filter(|(_, d, _, a)| *d < 6.0 * 1852.0 && a.abs() < 1500.0 / 3.28084)
        .filter(|(i, _, b, _)| {
            let rel = own_hdg.zip(track(*i)).map(|(h, t)| t - h);
            !behind_and_diverging(*b, rel)
        })
        .min_by(|x, y| x.1.total_cmp(&y.1))?;
    let clock = match (((b + 15.0).rem_euclid(360.0)) / 30.0) as i32 {
        0 => 12,
        c => c,
    };
    Some(TrafficFactor {
        nm: d / 1852.0,
        clock,
        ft: ((a * 3.28084) / 100.0).round() * 100.0,
        bound: track(i).map(bound_word),
        type_spoken: types
            .get(i)
            .cloned()
            .flatten()
            .and_then(|t| spoken_type(&t)),
        ..TrafficFactor::default()
    })
}

/// The relative-altitude clause both traffic calls share.
pub(super) fn vert_phrase(ft: f64) -> String {
    if ft.abs() < 300.0 {
        "same altitude".to_string()
    } else if ft > 0.0 {
        format!("{ft:.0} feet above")
    } else {
        format!("{:.0} feet below", ft.abs())
    }
}

/// Factor traffic near the vectored arrival, spoken as a controller advisory
/// ("traffic, two o'clock, three miles, five hundred feet above").
/// `None` = no factor, frequency stays quiet.
#[allow(dead_code)] // no caller: world_tick builds its advisory from the parts
pub async fn traffic_advisory(xp: SimView<'_>) -> Option<String> {
    let f = nearest_traffic_factor(xp).await?;
    Some(advisory_line(&f))
}

/// Is `now` worth calling again after `called`? A controller calls a
/// target once and again only when it has become MORE of a factor: two
/// miles closer, or 500 ft less between us. At the old 45 s cadence one
/// diverging target was called ten times across a cruise ("3 o'clock, 6
/// miles, 900 feet below", then 700, 1,000, 1,400 below; LIPK-LIPR
/// 2026-09-26). Pure.
pub fn traffic_worth_recalling(called: &TrafficFactor, now: &TrafficFactor) -> bool {
    now.nm <= called.nm - 2.0 || called.ft.abs() - now.ft.abs() >= 500.0
}

/// The advisory's words for a factor.
pub fn advisory_line(f: &TrafficFactor) -> String {
    // A mile is "a mile": 1.0 to 1.5 nm rounds to one, and "1 miles" was
    // said twice on one approach (EGBB 15, sortie 1790601822-full_auto,
    // +3194.7 and +3240.1, 2026-09-28, #155).
    let range = match f.nm {
        nm if nm < 1.0 => "under a mile".to_string(),
        nm if nm < 1.5 => "a mile".to_string(),
        nm => format!("{nm:.0} miles"),
    };
    // Doc 4444 12.3.1's order: clock, distance, direction of flight, then
    // the type and the level, each where known ("traffic, two o'clock,
    // four miles, westbound, A320, 1000 feet above").
    let bound = f.bound.map(|b| format!("{b}, ")).unwrap_or_default();
    let kind = f
        .type_spoken
        .as_ref()
        .map(|t| format!("{t}, "))
        .unwrap_or_default();
    format!(
        "traffic, {} o'clock, {range}, {bound}{kind}{}.",
        f.clock,
        vert_phrase(f.ft)
    )
}

/// Is this factor a NEAR MISS — unsafe proximity, not just a factor to scan
/// for? Half a mile and 500 ft is the NMAC line; the advisory tier has been
/// narrating the target since 6 nm, so by here the calm call has already
/// failed to separate us.
pub fn is_near_miss(f: &TrafficFactor) -> bool {
    f.nm < 0.5 && f.ft.abs() < 500.0
}

/// Which way OUT of a near miss: vertical when the intruder is clearly above
/// or below (go the other way), lateral when co-altitude — away from the
/// traffic's side, right for head-on (the right-of-way rule every pilot
/// already knows), and vertical again for one directly behind, where a turn
/// keeps it in the blind spot.
pub(super) fn avoidance(clock: i32, ft: f64) -> &'static str {
    if ft >= 100.0 {
        "descend"
    } else if ft <= -100.0 {
        "climb"
    } else {
        match clock {
            12 => "turn right",
            1..=5 => "turn left",
            7..=11 => "turn right",
            _ => "climb",
        }
    }
}

/// The near-miss call: the controller's safety alert in the installed
/// field's words when a frequency is tuned, the crew's shout when nobody
/// is. FAA (7110.65 2-1-6): "traffic alert, ... advise you climb
/// immediately". ICAO (Doc 4444 15.7.3, avoiding action): "climb
/// immediately to avoid traffic, ...". CAP 413: "avoiding action, climb
/// immediately, traffic ...". Europe heard the FAA's words until
/// 2026-10-09. Advisory words only, all ways: the copilot never
/// manoeuvres uncommanded.
pub fn near_miss_line(f: &TrafficFactor, crew: bool) -> String {
    let act = avoidance(f.clock, f.ft);
    let vert = vert_phrase(f.ft);
    if crew {
        return format!(
            "Traffic! {} o'clock, half a mile, {vert}. {act}, now.",
            f.clock
        );
    }
    match (f.dialect, f.uk) {
        (Dialect::Faa, _) => format!(
            "traffic alert, {} o'clock, less than a mile, {vert}, advise you {act} immediately.",
            f.clock
        ),
        (Dialect::Icao, true) => format!(
            "avoiding action, {act} immediately, traffic {} o'clock, less than a mile, {vert}.",
            f.clock
        ),
        (Dialect::Icao, false) => format!(
            "{act} immediately to avoid traffic, {} o'clock, less than a mile, {vert}.",
            f.clock
        ),
    }
}

/// An altitude spoken controller-style: 2000 → "two thousand", 1500 → "one
/// thousand five hundred".
pub(super) fn spoken_alt(ft: f64) -> String {
    atis::spoken_alt(ft)
}
