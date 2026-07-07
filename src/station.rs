//! The ATC station living inside the copilot process — the merge of the
//! standalone `atc` binary's tower service into the one binary the pilot
//! actually flies with. One mic, one PTT, one speech arbiter; the tuned COM1
//! frequency plus the shape of the words decide who answers.
//!
//! Trust boundary: this module only ever *reads* the simulator (frequency,
//! weather, tail number) and returns text. It has no access to the tool
//! registry and can never actuate anything — ATC is the world, not the crew.

use crate::xplane::XPlaneClient;
use crate::{atis, datarefs, nav, tower};

pub struct Station {
    apt: nav::AirportInfo,
    tower_khz: i64,
    /// The station's `say` voice and rate — distinct from the copilot's.
    pub say_voice: String,
    pub say_rate: String,
    twr: tower::Tower,
}

impl Station {
    /// Load the station for `icao` from apt.dat. `None` (with a log line) when
    /// the field is missing or has no tower frequency — the copilot then runs
    /// exactly as before the merge.
    pub fn load(apt_path: &str, icao: &str, say_voice: String, say_rate: String) -> Option<Station> {
        let apt = match nav::find_airport(apt_path, icao) {
            Ok(Some(a)) => a,
            Ok(None) => {
                tracing::warn!(%icao, "ATC station: airport not in apt.dat; ATC off");
                return None;
            }
            Err(e) => {
                tracing::warn!(%icao, "ATC station: apt.dat scan failed ({e:#}); ATC off");
                return None;
            }
        };
        let Some(twr_row) = apt.freq(nav::FREQ_TOWER) else {
            tracing::warn!(%icao, "ATC station: no tower frequency in apt.dat; ATC off");
            return None;
        };
        tracing::info!(
            airport = %apt.name,
            mhz = twr_row.khz as f64 / 1000.0,
            "ATC station up: tower answers radio calls on this frequency"
        );
        Some(Station { tower_khz: twr_row.khz, apt, say_voice, say_rate, twr: tower::Tower::new() })
    }

    /// If the utterance is a radio call AND COM1 is on this station's tower
    /// frequency, answer it; `None` falls through to the copilot untouched.
    /// Intent is checked before any dataref read, so the common case (a
    /// cockpit command) costs nothing.
    pub async fn try_handle(&mut self, transcript: &str, xp: &XPlaneClient) -> Option<String> {
        let intent = tower::parse_intent(transcript);
        if intent == tower::Intent::Unknown {
            return None;
        }
        let com1 = xp.get_float(datarefs::COM1_FREQ_KHZ).await.ok()?.round() as i64;
        // 8.33 channel names can sit 5 kHz off the carrier; adjacent channels
        // are 8 kHz apart, so 5 can't cross into one.
        if (com1 - self.tower_khz).abs() > 5 {
            return None;
        }

        // Live numbers for the clearance. Best-effort: a failed read falls
        // back to silence on that item rather than a wrong number.
        let wind_from = xp.get_float(datarefs::WIND_DIR_DEG_TRUE).await.unwrap_or(0.0);
        let wind_kts =
            xp.get_float(datarefs::WIND_SPEED_MSC).await.unwrap_or(0.0) * 1.943844;
        let qnh = match xp.get_float(datarefs::QNH_PASCALS).await {
            Ok(pa) => atis::spoken_digits(&format!("{:.0}", pa / 100.0)),
            Err(_) => String::new(),
        };
        let runway = nav::best_runway_for_wind(&self.apt.runways, wind_from, wind_kts)
            .map(|(id, _, _)| id)
            .unwrap_or_default();
        let callsign = xp
            .get_string(datarefs::TAILNUM)
            .await
            .ok()
            .map(|t| tower::spoken_callsign(&t))
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| "aircraft calling".to_string());
        let ctx = tower::Ctx {
            callsign,
            airport: self.apt.name.clone(),
            runway: atis::spoken_runway(&runway),
            qnh,
            wind: spoken_wind(wind_from, wind_kts),
            holding_point: "Alpha".to_string(),
        };
        Some(self.twr.handle(intent, &ctx))
    }
}

/// Wind spoken tower-style ("zero niner zero degrees, three knots" / "calm").
fn spoken_wind(from_deg: f64, kts: f64) -> String {
    if kts < 3.0 {
        return "calm".to_string();
    }
    let dir = ((from_deg / 10.0).round() as i64 * 10).rem_euclid(360);
    let dir = if dir == 0 { 360 } else { dir };
    format!(
        "{} degrees, {} knots",
        atis::spoken_digits(&format!("{dir:03}")),
        atis::spoken_digits(&format!("{kts:.0}"))
    )
}
