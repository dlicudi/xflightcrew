//! Traffic advisories and near-miss calls.

use super::*;

/// The nearest airborne factor from the TCAS picture: range, clock position,
/// and relative altitude (positive = traffic above us), rounded to the
/// hundred feet a pilot scans in.
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficFactor {
    pub nm: f64,
    pub clock: i32,
    pub ft: f64,
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
    select_traffic_factor(&dist, &bear, &alt, &wow)
}

/// [`nearest_traffic_factor`] over a picture of targets, from our position
/// (feet MSL) and true heading: the same selection as the sim's indicator
/// arrays ([`select_traffic_factor`]), slot 0 standing for us.
pub(super) fn factor_from_picture(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    hdg_deg: f64,
    picture: &[TcasTarget],
) -> Option<TrafficFactor> {
    let (mut dist, mut bear, mut alt, mut wow) = (vec![0.0], vec![0.0], vec![0.0], vec![0.0]);
    for t in picture {
        dist.push(crate::nav::haversine_nm(lat, lon, t.lat, t.lon) * 1852.0);
        bear.push((crate::nav::bearing_deg(lat, lon, t.lat, t.lon) - hdg_deg).rem_euclid(360.0));
        alt.push((t.ele_ft - alt_ft) / 3.28084);
        wow.push(if t.on_ground { 1.0 } else { 0.0 });
    }
    select_traffic_factor(&dist, &bear, &alt, &wow)
}

/// Pick the nearest airborne factor from the parallel TCAS indicator arrays.
/// Slot 0 is own ship. A target with weight on wheels is SKIPPED — a parked or
/// taxiing aircraft at the field sits within 6 nm and (from pattern height)
/// ±1,500 ft, and was being called "traffic, three o'clock, same altitude" on
/// the ramp (2026-07-24). Pure, so the ground filter is unit-tested.
pub(super) fn select_traffic_factor(
    dist: &[f64],
    bear: &[f64],
    alt: &[f64],
    wow: &[f64],
) -> Option<TrafficFactor> {
    let (d, b, a) = (1..dist.len())
        .filter(|&i| dist[i] > 1.0)
        // On the ground = not an airborne factor. Fail open when unfed.
        .filter(|&i| wow.get(i).is_none_or(|w| *w < 0.5))
        .map(|i| {
            (
                dist[i],
                *bear.get(i).unwrap_or(&0.0),
                *alt.get(i).unwrap_or(&0.0),
            )
        })
        .filter(|(d, _, a)| *d < 6.0 * 1852.0 && a.abs() < 1500.0 / 3.28084)
        .min_by(|x, y| x.0.total_cmp(&y.0))?;
    let clock = match (((b + 15.0).rem_euclid(360.0)) / 30.0) as i32 {
        0 => 12,
        c => c,
    };
    Some(TrafficFactor {
        nm: d / 1852.0,
        clock,
        ft: ((a * 3.28084) / 100.0).round() * 100.0,
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
    format!(
        "traffic, {} o'clock, {range}, {}.",
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

/// The near-miss call: the controller's safety alert (FAA 7110.65 shape,
/// "traffic alert ... advise you ... immediately") when a frequency is tuned,
/// the crew's shout when nobody is. Advisory words only, both ways — the
/// copilot never maneuvers uncommanded.
pub fn near_miss_line(f: &TrafficFactor, crew: bool) -> String {
    let act = avoidance(f.clock, f.ft);
    if crew {
        format!(
            "Traffic! {} o'clock, half a mile, {}. {act}, now.",
            f.clock,
            vert_phrase(f.ft)
        )
    } else {
        format!(
            "traffic alert, {} o'clock, less than a mile, {}, advise you {act} immediately.",
            f.clock,
            vert_phrase(f.ft)
        )
    }
}

/// An altitude spoken controller-style: 2000 → "two thousand", 1500 → "one
/// thousand five hundred".
pub(super) fn spoken_alt(ft: f64) -> String {
    atis::spoken_alt(ft)
}
