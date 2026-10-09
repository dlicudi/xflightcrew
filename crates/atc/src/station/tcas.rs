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

/// Our own AI traffic as the TCAS picture, while the fleet flies
/// (`XCOPILOT_TRAFFIC=ours`): published by the crew's traffic link every
/// monitor tick, `None` otherwise. The field's: `install` and `uninstall`
/// clear it ([`TrafficStrips`]), and the link publishes it again.
pub(crate) static TRAFFIC_PICTURE: Mutex<Option<Vec<TcasTarget>>> = Mutex::new(None);

/// The crew's traffic link publishes its fleet as the TCAS picture
/// (`None`: the fleet does not fly, the sim's TCAS is the picture).
pub fn publish_traffic_picture(picture: Option<Vec<TcasTarget>>) {
    *TRAFFIC_PICTURE.lock().unwrap_or_else(|e| e.into_inner()) = picture;
}

/// Our own traffic, as the crew may see it: every aircraft of the fleet as
/// a TCAS target (ours not in it); `None` while the fleet does not fly.
pub fn traffic_picture() -> Option<Vec<TcasTarget>> {
    TRAFFIC_PICTURE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
}

/// THE traffic picture, for every reader in the crew and the station: our
/// own fleet while it flies ([`traffic_picture`]: the plugin puts the same
/// aircraft in the sim's TCAS slots, so reading both would count each
/// twice), else the sim's TCAS table. The only caller of the sim read.
pub async fn targets(xp: SimView<'_>) -> Vec<TcasTarget> {
    match traffic_picture() {
        Some(picture) => picture,
        None => read_tcas_targets(xp).await,
    }
}

/// Read the TCAS target table (slot 0 = own ship, skipped). Targets without
/// a flight id are dropped — a tower can't address an aircraft it can't
/// name. Any missing array (older sim, no traffic feed) = empty. Read only
/// through [`targets`].
async fn read_tcas_targets(xp: SimView<'_>) -> Vec<TcasTarget> {
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
    // The type of each slot, for a controller who names traffic by it (a
    // conditional line-up, a traffic information): optional like the
    // vertical speed, noted in the field's type table by flight id.
    if let Ok(types) = xp.get_bytes("sim/cockpit2/tcas/targets/icao_type").await {
        for (id, ty) in ids.iter().zip(types.chunks(8)).skip(1) {
            let ty = String::from_utf8_lossy(ty);
            note_traffic_type(id, ty.trim_matches(char::from(0)).trim());
        }
    }
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
