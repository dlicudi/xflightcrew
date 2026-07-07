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

    /// Content routing (shared PTT): if the utterance is a radio call AND COM1
    /// is on this station's tower frequency, answer it; `None` falls through
    /// to the copilot untouched. Intent is checked before any dataref read,
    /// so the common case (a cockpit command) costs nothing.
    pub async fn try_handle(&mut self, transcript: &str, xp: &XPlaneClient) -> Option<String> {
        let intent = tower::parse_intent(transcript);
        if intent == tower::Intent::Unknown {
            return None;
        }
        if !self.on_frequency(xp).await {
            return None;
        }
        Some(self.respond(intent, xp).await)
    }

    /// Physical routing (the radio PTT was keyed): the utterance was
    /// TRANSMITTED, so it goes to the tuned station or nowhere — never to the
    /// copilot. On frequency, even an unparsed call gets the tower's "say
    /// again" (a real controller answers garble; your crew doesn't). `None`
    /// means nobody was listening on that frequency.
    pub async fn handle_transmission(&mut self, transcript: &str, xp: &XPlaneClient) -> Option<String> {
        if !self.on_frequency(xp).await {
            return None;
        }
        Some(self.respond(tower::parse_intent(transcript), xp).await)
    }

    /// Is COM1 tuned to this station's tower? 8.33 channel names can sit 5 kHz
    /// off the carrier; adjacent channels are 8 kHz apart, so 5 can't cross.
    pub async fn on_frequency(&self, xp: &XPlaneClient) -> bool {
        match xp.get_float(datarefs::COM1_FREQ_KHZ).await {
            Ok(v) => (v.round() as i64 - self.tower_khz).abs() <= 5,
            Err(_) => false,
        }
    }

    /// Build the live context and let the tower answer.
    async fn respond(&mut self, intent: tower::Intent, xp: &XPlaneClient) -> String {
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
        self.twr.handle(intent, &ctx)
    }
}

/// Outcome of a named-station tuning request ("tune the tower").
pub enum Tune {
    /// Set COM1 to this kHz; `label` is spoken in the readback.
    Found { label: String, khz: i64 },
    /// The airport has no such frequency — decline by name, don't guess.
    Missing(String),
}

impl Station {
    /// Deterministic named-station tuning: "tune/select/switch to/contact the
    /// tower|atis|ground|approach|departure" (or generic "atc"/"station" =
    /// tower) resolved against this airport's apt.dat frequencies. Requires a
    /// tuning verb, so radio chatter that merely mentions a station never
    /// re-tunes the radio. `None` = not a tuning request at all.
    pub fn tune_request(&self, t: &str) -> Option<Tune> {
        let verb = t.contains("tune") || t.contains("switch to") || t.contains("select")
            || t.contains("contact") || (t.contains("set") && t.contains("com"));
        if !verb {
            return None;
        }
        let kinds: &[(&str, u32)] = &[
            ("tower", nav::FREQ_TOWER),
            ("atis", nav::FREQ_ATIS),
            ("ground", nav::FREQ_GROUND),
            ("approach", nav::FREQ_APPROACH),
            ("departure", nav::FREQ_DEPARTURE),
            ("unicom", nav::FREQ_UNICOM),
            ("clearance", nav::FREQ_CLEARANCE),
        ];
        let named = kinds.iter().find(|(w, _)| t.contains(w));
        // Generic "tune atc" / "tune the station" means the tower.
        let (word, kind) = match named {
            Some((w, k)) => (*w, *k),
            None if t.contains("atc") || t.contains("station") => ("tower", nav::FREQ_TOWER),
            None => return None,
        };
        Some(match self.apt.freq(kind) {
            Some(f) => Tune::Found {
                label: format!("{} {}", self.apt.name, word),
                khz: f.khz,
            },
            None => Tune::Missing(format!("{} has no {} frequency.", self.apt.name, word)),
        })
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

#[cfg(test)]
mod tests {
    use super::*;

    fn lxgb() -> Station {
        Station {
            apt: nav::AirportInfo {
                name: "Gibraltar".into(),
                elev_ft: 11.0,
                runways: Vec::new(),
                freqs: vec![
                    nav::AtcFreq { kind: nav::FREQ_TOWER, khz: 131200, name: "Tower Freq".into() },
                    nav::AtcFreq { kind: nav::FREQ_APPROACH, khz: 122800, name: "Approach".into() },
                ],
            },
            tower_khz: 131200,
            say_voice: String::new(),
            say_rate: String::new(),
            twr: tower::Tower::new(),
        }
    }

    #[test]
    fn named_station_tuning_resolves_from_apt_dat() {
        let st = lxgb();
        match st.tune_request("tune the tower") {
            Some(Tune::Found { label, khz }) => {
                assert_eq!(khz, 131200);
                assert_eq!(label, "Gibraltar tower");
            }
            other => panic!("expected tower: {:?}", other.is_some()),
        }
        // Generic "atc"/"station" means the tower.
        assert!(matches!(st.tune_request("tune to atc"), Some(Tune::Found { khz: 131200, .. })));
        assert!(matches!(st.tune_request("contact approach"), Some(Tune::Found { khz: 122800, .. })));
        // A named station the field doesn't have declines by name, no guessing.
        match st.tune_request("tune the atis") {
            Some(Tune::Missing(msg)) => assert!(msg.contains("no atis"), "{msg}"),
            other => panic!("expected Missing: {:?}", other.is_some()),
        }
        // No tuning verb → not a tuning request, even if a station is named
        // ("gibraltar tower request taxi" must reach the tower, not the tuner).
        assert!(st.tune_request("gibraltar tower request taxi").is_none());
        // A verb without any station name is someone else's command.
        assert!(st.tune_request("tune com one to one one eight decimal five").is_none());
    }
}
