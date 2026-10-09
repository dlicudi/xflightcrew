//! The in-process ATIS broadcast state.

use super::*;

/// The in-process ATIS: one broadcast state per station, driven by the
/// monitor's tick while COM1 sits on the ATIS frequency. Editions cut and the
/// letter advances exactly as the standalone binary did (`atis::new_edition`
/// hysteresis included); the report always reads the FROZEN edition, so the
/// numbers can't drift mid-cycle.
pub struct AtisState {
    pub khz: i64,
    pub(super) airport: String,
    pub(super) runways: Vec<nav::Runway>,
    pub(super) letter: char,
    pub(super) edition: Option<atis::Snapshot>,
    pub(super) last_spoken: Option<Instant>,
}

pub(super) static ATIS: Mutex<Option<AtisState>> = Mutex::new(None);

/// Re-broadcast cadence while the pilot stays tuned — a real ATIS loops.
pub(super) const ATIS_REPEAT_SECS: f64 = 45.0;

/// The live weather one ATIS cycle is cut from (read by the monitor).
#[derive(Debug, Clone, Copy)]
pub struct AtisWx {
    pub zulu_sec: f64,
    pub wind_from_deg: f64,
    pub wind_kts: f64,
    pub visibility_sm: f64,
    pub qnh_hpa: f64,
    pub oat_c: f64,
}

/// The monitor's cheap gate: true when an ATIS exists, COM1 is on it, AND a
/// broadcast is due — only then is the full weather set worth reading (six
/// REST GETs; reading them every tick while parked on the frequency was
/// measurable main-thread load on the sim). Off frequency it also rewinds the
/// loop so a retune starts the broadcast from the top.
pub fn atis_due(com1_khz: i64, now: Instant) -> bool {
    let mut guard = ATIS.lock().unwrap_or_else(|e| e.into_inner());
    let Some(st) = guard.as_mut() else {
        return false;
    };
    if (com1_khz - st.khz).abs() > 5 {
        st.last_spoken = None;
        return false;
    }
    st.last_spoken
        .is_none_or(|t| now.duration_since(t).as_secs_f64() >= ATIS_REPEAT_SECS)
}

/// Follow COM1 to any in-range field's ATIS. The ATC tab lets the pilot tune
/// every listed frequency, but the broadcast only existed for the ACTIVE
/// station — tuning another field's ATIS was dead air (2026-07-10). Swapping
/// fields restarts that field's loop at information Alpha.
pub fn atis_retune(com1_khz: i64, lat: f64, lon: f64, alt_msl_ft: f64) {
    {
        let cur = ATIS.lock().unwrap_or_else(|e| e.into_inner());
        if cur.as_ref().is_some_and(|a| (a.khz - com1_khz).abs() <= 5) {
            return; // already broadcasting on this frequency
        }
    }
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    let hit = region.iter().find(|a| {
        a.apt
            .freq(nav::FREQ_ATIS)
            .is_some_and(|f| (f.khz - com1_khz).abs() <= 5)
            && crate::nav::haversine_nm(lat, lon, a.lat, a.lon)
                <= vhf_range_nm(alt_msl_ft - a.apt.elev_ft)
    });
    if let Some(a) = hit {
        let khz = a
            .apt
            .freq(nav::FREQ_ATIS)
            .map(|f| f.khz)
            .unwrap_or(com1_khz);
        tracing::info!(
            airport = %a.apt.name,
            mhz = khz as f64 / 1000.0,
            "ATIS following COM1 to another field"
        );
        *ATIS.lock().unwrap_or_else(|e| e.into_inner()) = Some(AtisState {
            khz,
            airport: a.apt.radio_name(),
            runways: a.apt.runways.clone(),
            letter: 'A',
            edition: None,
            last_spoken: None,
        });
    }
}

/// One tick of the ATIS broadcast: the full report when tuned and due (first
/// tune-in speaks immediately, then the loop repeats), silence otherwise.
/// Cuts a new edition — advancing the letter — when the weather has genuinely
/// moved (see `atis::new_edition`).
pub fn atis_tick(com1_khz: i64, wx: &AtisWx, now: Instant) -> Option<String> {
    if station_unavailable() {
        return None;
    }
    let mut guard = ATIS.lock().unwrap_or_else(|e| e.into_inner());
    let st = guard.as_mut()?;
    if (com1_khz - st.khz).abs() > 5 {
        st.last_spoken = None; // retuning restarts the loop from the top
        return None;
    }
    if st
        .last_spoken
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < ATIS_REPEAT_SECS)
    {
        return None;
    }
    // Runway from the wind — but in calm keep the edition's pick, or the
    // coin-toss flip would cut a fresh edition per broadcast.
    let runway = if wx.wind_kts < CALM_WIND_KTS {
        st.edition.as_ref().map(|e| e.runway.clone())
    } else {
        None
    }
    .or_else(|| {
        nav::best_runway_for_wind(&st.runways, wx.wind_from_deg, wx.wind_kts).map(|(id, _, _)| id)
    })
    .unwrap_or_default();
    let snap = atis::Snapshot {
        airport: st.airport.clone(),
        zulu_sec: wx.zulu_sec,
        runway,
        wind_from_deg: wx.wind_from_deg,
        wind_kts: wx.wind_kts,
        visibility_sm: wx.visibility_sm,
        qnh_hpa: wx.qnh_hpa,
        oat_c: wx.oat_c,
    };
    match &st.edition {
        None => st.edition = Some(snap),
        Some(e) if atis::new_edition(e, &snap) => {
            st.letter = atis::next_letter(st.letter);
            st.edition = Some(snap);
        }
        // Same edition: time still moves in the report (a frozen clock reads
        // as a dead station), everything else stays the edition's numbers.
        Some(_) => {
            if let Some(e) = st.edition.as_mut() {
                e.zulu_sec = wx.zulu_sec;
            }
        }
    }
    st.last_spoken = Some(now);
    Some(atis::report(st.edition.as_ref()?, st.letter))
}

/// The information letter currently on the broadcast — `None` until the first
/// edition is cut (a pilot can't "have information" that never aired).
pub fn atis_current_letter() -> Option<char> {
    ATIS.lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|a| a.edition.is_some())
        .map(|a| a.letter)
}
