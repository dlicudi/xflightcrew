//! Live traffic from the TCAS picture.

use super::*;

// --- Live-traffic watches (party line, go-around, held clearance) -----------
//
// LiveTraffic aircraft are replays of real flights — nothing can command
// them. The design stance (2026-07-10): the tower controls the ONE aircraft
// with a pilot in it and tells the truth about everything else. The party
// line NARRATES the clearance a ghost is about to comply with; the go-around
// order protects the player when a ghost takes the runway; the held takeoff
// clearance releases itself when the final actually clears.

/// One live TCAS target with an identity and an absolute position.
#[derive(Clone)]
pub struct TcasTarget {
    pub callsign: String,
    pub lat: f64,
    pub lon: f64,
    pub ele_ft: f64,
    pub on_ground: bool,
    /// Vertical speed, ft/min; 0.0 when the feed doesn't populate the array
    /// (reads as level, which every watch treats conservatively).
    pub vs_fpm: f64,
}

/// Read the TCAS target table (slot 0 = own ship, skipped). Targets without
/// a flight id are dropped — a tower can't address an aircraft it can't
/// name. Any missing array (older sim, no traffic feed) = empty.
pub async fn read_tcas_targets(xp: SimView<'_>) -> Vec<TcasTarget> {
    let (Ok(lat), Ok(lon), Ok(ele), Ok(wow), Ok(ids)) = (
        xp.get_float_array(datarefs::TCAS_LAT).await,
        xp.get_float_array(datarefs::TCAS_LON).await,
        xp.get_float_array(datarefs::TCAS_ELE_M).await,
        xp.get_float_array(datarefs::TCAS_WOW).await,
        xp.get_bytes(datarefs::TCAS_FLIGHT_ID).await,
    ) else {
        return Vec::new();
    };
    let ids: Vec<String> = ids
        .chunks(8)
        .map(|c| {
            String::from_utf8_lossy(c)
                .trim_matches('\0')
                .trim()
                .to_string()
        })
        .collect();
    // Vertical speed is OPTIONAL — an older sim or a feed that skips the
    // array must not blank the whole traffic picture, so a failed read pads
    // with level flight rather than joining the all-or-nothing gate above.
    let vs = xp
        .get_float_array(datarefs::TCAS_VS_FPM)
        .await
        .unwrap_or_default();
    lat.iter()
        .zip(&lon)
        .zip(&ele)
        .zip(&wow)
        .zip(&ids)
        .zip(vs.into_iter().chain(std::iter::repeat(0.0)))
        .skip(1) // slot 0 is own ship
        .filter(|(((((la, lo), _), _), id), _)| !id.is_empty() && (**la != 0.0 || **lo != 0.0))
        .map(|(((((la, lo), el), ww), id), vs)| TcasTarget {
            callsign: id.clone(),
            lat: *la,
            lon: *lo,
            ele_ft: *el * 3.28084,
            on_ground: *ww > 0.5,
            vs_fpm: vs,
        })
        .collect()
}
