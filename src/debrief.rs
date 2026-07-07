//! Post-flight distiller and carry-forward memory — the copilot's long-term
//! memory, fully deterministic (a stats pass over the black box, no LLM).
//!
//! After a flight ends (airborne → parked), [`distill`] reduces the black-box
//! JSONL to a [`FlightSummary`]: phase timings, extremes, and every monitor
//! callout that fired, with counts. The summary is written to
//! `memory/flights/<stamp>-<aircraft>.yaml`, and the callout counts are folded
//! into `memory/carry_forward.yaml`, keyed by airframe. On the next startup the
//! briefing reads the carry-forward and tells the pilot what to watch — the
//! copilot that flew with you last time, not a fresh install.

use std::collections::BTreeMap;
use std::io::BufRead;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

/// Where memory lives, relative to the working directory (like `logs/`).
pub const MEMORY_DIR: &str = "memory";

/// One flight, distilled.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlightSummary {
    pub aircraft: String,
    /// Unix seconds of the first and last airborne records.
    pub takeoff_t: f64,
    pub landing_t: f64,
    pub airborne_min: f64,
    pub max_alt_ft: f64,
    pub max_ias_kts: f64,
    /// Sink rate at the final touchdown, fpm (from the last airborne tick) —
    /// the "how hard did we arrive" number.
    pub touchdown_fpm: f64,
    pub fuel_burned_kg: f64,
    /// Monitor callouts that fired while airborne, id → count.
    pub callouts: BTreeMap<String, u32>,
}

impl FlightSummary {
    /// One spoken sentence for the shutdown debrief.
    pub fn spoken(&self) -> String {
        let mut s = format!(
            "Debrief: {:.0} minutes airborne, {:.0} kilos burned, touchdown {:.0} feet a minute.",
            self.airborne_min,
            self.fuel_burned_kg,
            self.touchdown_fpm.abs()
        );
        if self.callouts.is_empty() {
            s.push_str(" Clean flight, no callouts.");
        } else {
            let total: u32 = self.callouts.values().sum();
            let mut ids: Vec<_> = self.callouts.iter().collect();
            ids.sort_by(|a, b| b.1.cmp(a.1));
            let top: Vec<String> = ids.iter().take(2).map(|(id, _)| spoken_name(id)).collect();
            s.push_str(&format!(" {total} callouts, mostly {}.", top.join(" and ")));
        }
        s
    }
}

/// Carry-forward memory: per airframe, the accumulated callout tallies and the
/// last flight's one-liner. Small on purpose — it's a briefing, not a log.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CarryForward {
    /// Airframe id → accumulated callout counts across flights.
    #[serde(default)]
    pub tallies: BTreeMap<String, BTreeMap<String, u32>>,
    /// Airframe id → number of flights distilled.
    #[serde(default)]
    pub flights: BTreeMap<String, u32>,
    /// Airframe id → the last flight's spoken summary.
    #[serde(default)]
    pub last_flight: BTreeMap<String, String>,
    /// Airframe id → epoch DAY (secs/86400) the briefing last played. A
    /// session that restarts five times must not deliver the same heads-up
    /// five times ("annoying", live 2026-07-07) — once a day is a briefing,
    /// more is a nag.
    #[serde(default)]
    pub briefed_day: BTreeMap<String, u64>,
}

impl CarryForward {
    pub fn load(dir: &Path) -> Self {
        std::fs::read_to_string(dir.join("carry_forward.yaml"))
            .ok()
            .and_then(|s| serde_yaml::from_str(&s).ok())
            .unwrap_or_default()
    }

    fn save(&self, dir: &Path) -> Result<()> {
        std::fs::create_dir_all(dir)?;
        std::fs::write(dir.join("carry_forward.yaml"), serde_yaml::to_string(self)?)?;
        Ok(())
    }

    /// The startup briefing for this airframe: only the *recurring* things worth
    /// watching today, or `None` when there's nothing notable. Deliberately silent
    /// on a normal startup — it used to open every flight by reciting the last
    /// flight's stats ("45 minutes airborne, 90 kilos burned…"), backward-looking
    /// trivia that got old fast. Now it speaks only when a pattern has actually
    /// recurred (a callout seen 3+ times), which is the one forward-looking thing
    /// a pre-flight heads-up should carry.
    pub fn briefing(&self, aircraft: &str) -> Option<String> {
        let watch: Vec<String> = self
            .tallies
            .get(aircraft)
            .map(|t| {
                let mut items: Vec<_> = t
                    .iter()
                    // Advisor items are profile-internal SOP nudges; their ids
                    // are meaningless spoken ("advisor 0 cessna 172sp").
                    .filter(|(id, &n)| n >= 3 && !id.starts_with("advisor_"))
                    .collect();
                items.sort_by(|a, b| b.1.cmp(a.1));
                items.into_iter().take(2).map(|(id, _)| spoken_name(id)).collect()
            })
            .unwrap_or_default();
        (!watch.is_empty())
            .then(|| format!("Heads up, this aircraft has a history of {}. I'll watch for it today.", watch.join(" and ")))
    }

    /// [`briefing`](Self::briefing), at most once per day per airframe: checks
    /// and records `briefed_day`, persisting on change. Restarting the copilot
    /// mid-session stays silent.
    pub fn briefing_once_today(dir: &Path, aircraft: &str) -> Option<String> {
        let today = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs() / 86_400)
            .unwrap_or(0);
        let mut cf = Self::load(dir);
        if cf.briefed_day.get(aircraft) == Some(&today) {
            return None;
        }
        let brief = cf.briefing(aircraft)?;
        cf.briefed_day.insert(aircraft.to_string(), today);
        if let Err(e) = cf.save(dir) {
            tracing::warn!("carry-forward save failed: {e:#}");
        }
        Some(brief)
    }
}

/// Check ids are code identifiers; speak them like a person. The fallback
/// de-underscores, so an unmapped id degrades to words, never to "cht_warn".
fn spoken_name(id: &str) -> String {
    match id {
        "sink_rate" => "sink rate",
        "altitude_bust" => "altitude capture",
        "alt_offline" => "the alternator",
        "alt_not_charging" => "the charging system",
        "cabin_alt_caution" | "cabin_alt_warning" => "cabin altitude",
        "park_brake_rolling" => "the parking brake",
        "cht_caution" | "cht_warn" => "high cylinder head temps",
        "oil_temp_caution" | "oil_temp_warn" => "high oil temperature",
        "oil_press_low" => "low oil pressure",
        "airspeed_caution" | "airspeed_warning" => "airspeed excursions",
        "flap_overspeed" => "flap overspeeds",
        "gyro_drift" => "gyro drift",
        "carb_ice" => "carb ice",
        other => return other.replace('_', " "),
    }
    .to_string()
}

/// Minimal view of one black-box line — unknown kinds/fields ignored.
#[derive(Deserialize)]
struct Row {
    t: f64,
    kind: String,
    #[serde(default)]
    aircraft: Option<String>,
    #[serde(default)]
    ground: Option<bool>,
    #[serde(default)]
    alt: Option<f64>,
    #[serde(default)]
    ias: Option<f64>,
    #[serde(default)]
    vsi: Option<f64>,
    #[serde(default)]
    fuel_kg: Option<f64>,
    #[serde(default)]
    id: Option<String>,
}

/// Distill the *last completed flight* (the final airborne stretch) out of the
/// black box. `None` if the log holds no airborne time — nothing to remember.
pub fn distill(blackbox: &Path) -> Result<Option<FlightSummary>> {
    let file = std::fs::File::open(blackbox).context("open black box")?;
    let mut aircraft = "unknown".to_string();
    // The last airborne stretch, accumulated in one pass.
    let mut summary: Option<FlightSummary> = None;
    let mut airborne = false;
    let mut fuel_at_takeoff = 0.0_f64;
    let mut last_vsi = 0.0_f64;
    for line in std::io::BufReader::new(file).lines() {
        let Ok(line) = line else { break };
        let Ok(row) = serde_json::from_str::<Row>(&line) else { continue };
        match row.kind.as_str() {
            "flight_meta" => {
                if let Some(a) = row.aircraft {
                    aircraft = a;
                }
            }
            "state" => {
                let (Some(ground), Some(alt), Some(ias), Some(vsi), Some(fuel)) =
                    (row.ground, row.alt, row.ias, row.vsi, row.fuel_kg)
                else {
                    continue;
                };
                if !ground && !airborne {
                    // Liftoff: a new (possibly the new last) flight begins.
                    airborne = true;
                    fuel_at_takeoff = fuel;
                    summary = Some(FlightSummary {
                        aircraft: aircraft.clone(),
                        takeoff_t: row.t,
                        landing_t: row.t,
                        airborne_min: 0.0,
                        max_alt_ft: alt,
                        max_ias_kts: ias,
                        touchdown_fpm: 0.0,
                        fuel_burned_kg: 0.0,
                        callouts: BTreeMap::new(),
                    });
                }
                if let Some(s) = summary.as_mut() {
                    if airborne {
                        s.landing_t = row.t;
                        s.max_alt_ft = s.max_alt_ft.max(alt);
                        s.max_ias_kts = s.max_ias_kts.max(ias);
                        s.fuel_burned_kg = (fuel_at_takeoff - fuel).max(0.0);
                        if ground {
                            // Touchdown: the sink rate is the last airborne tick's.
                            airborne = false;
                            s.touchdown_fpm = last_vsi;
                            s.airborne_min = (s.landing_t - s.takeoff_t) / 60.0;
                        } else {
                            last_vsi = vsi;
                        }
                    }
                }
            }
            "callout" => {
                if airborne {
                    if let (Some(s), Some(id)) = (summary.as_mut(), row.id) {
                        *s.callouts.entry(id).or_insert(0) += 1;
                    }
                }
            }
            _ => {}
        }
    }
    // Still airborne at end-of-log = the flight didn't finish; don't remember it.
    Ok(summary.filter(|s| !airborne && s.airborne_min >= 1.0))
}

/// Distill, persist the flight file, and fold the result into carry-forward.
/// Returns the summary for the spoken debrief. Best-effort by design: memory
/// must never take the copilot down with it.
pub fn remember(blackbox: &Path, memory_dir: &Path) -> Option<FlightSummary> {
    let summary = distill(blackbox).ok().flatten()?;
    let flights_dir = memory_dir.join("flights");
    let _ = std::fs::create_dir_all(&flights_dir);
    let stamp = stamp_utc(summary.takeoff_t);
    if let Ok(yaml) = serde_yaml::to_string(&summary) {
        let _ = std::fs::write(flights_dir.join(format!("{stamp}-{}.yaml", summary.aircraft)), yaml);
    }
    let mut cf = CarryForward::load(memory_dir);
    let tally = cf.tallies.entry(summary.aircraft.clone()).or_default();
    for (id, n) in &summary.callouts {
        *tally.entry(id.clone()).or_insert(0) += n;
    }
    *cf.flights.entry(summary.aircraft.clone()).or_insert(0) += 1;
    cf.last_flight.insert(summary.aircraft.clone(), summary.spoken());
    let _ = cf.save(memory_dir);
    Some(summary)
}

/// The default memory directory, alongside `logs/`.
pub fn memory_dir() -> PathBuf {
    PathBuf::from(MEMORY_DIR)
}

/// `YYYYMMDD-HHMM` in UTC from unix seconds, without pulling in chrono —
/// civil-from-days per Howard Hinnant's algorithm.
fn stamp_utc(unix: f64) -> String {
    let secs = unix as i64;
    let days = secs.div_euclid(86_400);
    let sod = secs.rem_euclid(86_400);
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}{m:02}{d:02}-{:02}{:02}", sod / 3600, (sod % 3600) / 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_box(dir: &Path, lines: &[&str]) -> PathBuf {
        let p = dir.join("blackbox.jsonl");
        std::fs::write(&p, lines.join("\n")).unwrap();
        p
    }

    fn state(t: f64, ground: bool, alt: f64, ias: f64, vsi: f64, fuel: f64) -> String {
        format!(
            r#"{{"t":{t},"kind":"state","ground":{ground},"alt":{alt},"ias":{ias},"vsi":{vsi},"fuel_kg":{fuel},"agl":0,"gs":0,"hdg":0,"lat":0,"lon":0,"flap":0,"park":0,"cabin":0,"eng":true,"phase":"Cruise"}}"#
        )
    }

    #[test]
    fn distills_the_last_flight() {
        let dir = tempfile::tempdir().unwrap();
        let meta = r#"{"t":0,"kind":"flight_meta","aircraft":"md82"}"#.to_string();
        let callout = r#"{"t":400,"kind":"callout","id":"sink_rate","sev":"Warning","msg":"Sink rate!"}"#.to_string();
        let lines: Vec<String> = vec![
            meta,
            state(100.0, true, 100.0, 0.0, 0.0, 5000.0),
            state(200.0, false, 2000.0, 150.0, 1000.0, 4900.0), // liftoff
            state(400.0, false, 14000.0, 250.0, 0.0, 4000.0),
            callout,
            state(500.0, false, 3000.0, 210.0, -3400.0, 3800.0),
            state(560.0, true, 1900.0, 200.0, -50.0, 3750.0), // touchdown
            state(600.0, true, 1900.0, 0.0, 0.0, 3740.0),
        ];
        let refs: Vec<&str> = lines.iter().map(|s| s.as_str()).collect();
        let p = write_box(dir.path(), &refs);
        let s = distill(&p).unwrap().expect("a flight");
        assert_eq!(s.aircraft, "md82");
        assert_eq!(s.max_alt_ft, 14000.0);
        assert_eq!(s.max_ias_kts, 250.0);
        assert_eq!(s.touchdown_fpm, -3400.0); // the last airborne tick's sink
        assert!((s.fuel_burned_kg - 1150.0).abs() < 1.0); // 4900 → 3750 at touchdown
        assert_eq!(s.callouts.get("sink_rate"), Some(&1));
        assert_eq!(s.airborne_min, 6.0);
        // The debrief speaks the ugly number instead of burying it.
        assert!(s.spoken().contains("3400 feet a minute"));
    }

    #[test]
    fn carry_forward_accumulates_and_briefs() {
        let dir = tempfile::tempdir().unwrap();
        let mem = dir.path().join("memory");
        let mut cf = CarryForward::default();
        // Three sink-rate callouts across flights → named in the briefing.
        for _ in 0..3 {
            *cf.tallies.entry("md82".into()).or_default().entry("sink_rate".into()).or_insert(0) += 1;
        }
        cf.last_flight.insert("md82".into(), "Debrief: rough one.".into());
        cf.save(&mem).unwrap();
        let loaded = CarryForward::load(&mem);
        let brief = loaded.briefing("md82").expect("has a briefing");
        // Only the forward-looking recurring watch — no last-flight recitation.
        assert!(brief.contains("sink rate"), "{brief}");
        assert!(!brief.contains("rough one"), "no backward-looking trivia: {brief}");
        // Silent when there's no recurring pattern (and for an unflown airframe).
        assert!(loaded.briefing("dc3").is_none(), "no recurring pattern → silent");
    }

    #[test]
    fn startup_is_silent_without_a_recurring_pattern() {
        // A one-off callout (only twice) must not trigger a startup briefing —
        // startup stays quiet unless a pattern has actually recurred (3+).
        let mut cf = CarryForward::default();
        cf.last_flight.insert("c172".into(), "Debrief: uneventful.".into());
        *cf.tallies.entry("c172".into()).or_default().entry("sink_rate".into()).or_insert(0) += 2;
        assert!(cf.briefing("c172").is_none(), "2 occurrences is not a pattern");
    }

    #[test]
    fn unfinished_flight_is_not_remembered() {
        let dir = tempfile::tempdir().unwrap();
        let lines: Vec<String> = vec![
            state(100.0, true, 100.0, 0.0, 0.0, 5000.0),
            state(200.0, false, 2000.0, 150.0, 1000.0, 4900.0),
            state(400.0, false, 14000.0, 250.0, 0.0, 4000.0), // log ends airborne
        ];
        let refs: Vec<&str> = lines.iter().map(|s| s.as_str()).collect();
        let p = write_box(dir.path(), &refs);
        assert!(distill(&p).unwrap().is_none());
    }

    #[test]
    fn stamp_is_civil_utc() {
        // 2026-07-04 ~12:00 UTC (today's flight).
        assert_eq!(stamp_utc(1_783_166_400.0), "20260704-1200");
        assert_eq!(stamp_utc(0.0), "19700101-0000");
    }
}
