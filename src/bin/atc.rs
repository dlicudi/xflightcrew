//! `atc`: the air traffic side of the radio, a separate process from the
//! copilot on purpose. The two share code (via the `x_copilot` lib target),
//! not state: both watch the same simulator through the Web API, and the
//! tuned COM1 frequency decides who answers. This process is structurally
//! READ-ONLY toward the simulator: every dataref access goes through
//! [`SimView`], which exposes no writes. The copilot is crew and acts on the
//! aircraft; ATC is the world and never touches it.
//!
//! Milestone zero is ATIS only: tune COM1 to the ATIS frequency and a looping
//! broadcast built from live sim weather plays in its own voice. No STT, no
//! dialogue yet. Frequencies come from the airport's apt.dat comm rows, not
//! hardcoding; the tower frequency stands in when the data has no ATIS row
//! (true at LXGB), until the tower service exists.
//!
//! Environment:
//!   XCOPILOT_ATC_AIRPORT  ICAO of the field to serve (default LXGB)
//!   XCOPILOT_ATIS_FREQ    override frequency, MHz ("131.2") or kHz ("131200")
//!   XCOPILOT_ATC_VOICE    macOS `say` voice for the broadcast (default Daniel)
//!   XCOPILOT_ATC_RATE     `say` words-per-minute (default 165, a read pace)

use anyhow::{anyhow, Context, Result};
use x_copilot::{atis, config::Config, datarefs, nav, phrase, tts, xplane::XPlaneClient};

/// Read-only view of the simulator. ATC observes and speaks; it must never
/// actuate anything, so only reads are reachable through this type.
struct SimView(XPlaneClient);

impl SimView {
    async fn get_float(&self, name: &str) -> Result<f64> {
        self.0.get_float(name).await
    }
}

/// "131.2" / "131.200" (MHz) or "131200" (kHz) to kHz.
fn parse_freq_khz(s: &str) -> Result<i64> {
    let s = s.trim();
    if s.contains('.') {
        let mhz: f64 = s.parse().with_context(|| format!("bad frequency: {s}"))?;
        Ok((mhz * 1000.0).round() as i64)
    } else {
        s.parse().with_context(|| format!("bad frequency: {s}"))
    }
}

fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).ok().filter(|v| !v.is_empty()).unwrap_or_else(|| default.to_string())
}

/// One weather snapshot off the sim, plus the wind-preferred runway.
async fn read_snapshot(sim: &SimView, apt: &nav::AirportInfo) -> Result<atis::Snapshot> {
    let wind_from_deg = sim.get_float(datarefs::WIND_DIR_DEG_TRUE).await?;
    let wind_kts = sim.get_float(datarefs::WIND_SPEED_MSC).await? * 1.943844;
    // Wind and runway headings are both degrees true, so the pick is consistent
    // even though the spoken direction skips the (tiny, ~1 degree at LXGB)
    // magnetic conversion for now.
    let runway = nav::best_runway_for_wind(&apt.runways, wind_from_deg, wind_kts)
        .map(|(id, _, _)| id)
        .unwrap_or_default();
    Ok(atis::Snapshot {
        airport: apt.name.clone(),
        zulu_sec: sim.get_float(datarefs::ZULU_TIME_SEC).await?,
        runway,
        wind_from_deg,
        wind_kts,
        visibility_sm: sim.get_float(datarefs::VISIBILITY_SM).await?,
        qnh_hpa: sim.get_float(datarefs::QNH_PASCALS).await? / 100.0,
        oat_c: sim.get_float(datarefs::OAT_DEG_C).await?,
    })
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let config = Config::from_env()?;
    let icao = env_or("XCOPILOT_ATC_AIRPORT", "LXGB").to_uppercase();

    tracing::info!(%icao, "scanning apt.dat for the airport (once, at startup)");
    let apt = nav::find_airport(&config.apt_data_path, &icao)?
        .ok_or_else(|| anyhow!("{icao} not found in {}", config.apt_data_path))?;
    for f in &apt.freqs {
        tracing::info!(kind = f.kind, mhz = f.khz as f64 / 1000.0, name = %f.name, "comm row");
    }

    let atis_khz = match std::env::var("XCOPILOT_ATIS_FREQ").ok().filter(|v| !v.is_empty()) {
        Some(v) => parse_freq_khz(&v)?,
        None => match apt.freq(nav::FREQ_ATIS) {
            Some(f) => f.khz,
            None => {
                let twr = apt.freq(nav::FREQ_TOWER).ok_or_else(|| {
                    anyhow!(
                        "{icao} has no ATIS or tower row in apt.dat; set XCOPILOT_ATIS_FREQ"
                    )
                })?;
                tracing::info!(
                    mhz = twr.khz as f64 / 1000.0,
                    "no ATIS row for {icao}; serving ATIS on the tower frequency until \
                     the tower service exists (XCOPILOT_ATIS_FREQ overrides)"
                );
                twr.khz
            }
        },
    };
    tracing::info!(
        airport = %apt.name,
        mhz = atis_khz as f64 / 1000.0,
        "ATIS ready; tune COM1 to hear it"
    );

    // The broadcast voice: deliberately not the copilot's. The `say` backend
    // needs nothing installed; a cloned radio voice can slot in later.
    let voice = Config {
        tts_backend: "say".into(),
        say_voice: env_or("XCOPILOT_ATC_VOICE", "Daniel"),
        say_rate: env_or("XCOPILOT_ATC_RATE", "165"),
        ..Config::default()
    };

    let sim = SimView(XPlaneClient::new(config.xplane_base_url.clone()));
    let mut letter = 'A';
    let mut last_fp: Option<(String, i64, i64, i64, i64)> = None;
    let mut sim_down = false;

    loop {
        // Who is tuned in? 8.33 channel names can sit 5 kHz off the actual
        // carrier (118.330 names 118.325), hence the small tolerance; adjacent
        // 8.33 channels are 8 kHz apart so 5 cannot cross into one.
        let com1 = match sim.get_float(datarefs::COM1_FREQ_KHZ).await {
            Ok(v) => {
                if sim_down {
                    tracing::info!("simulator is back");
                }
                sim_down = false;
                v.round() as i64
            }
            Err(e) => {
                if !sim_down {
                    tracing::warn!("simulator unreachable, waiting: {e:#}");
                }
                sim_down = true;
                tokio::time::sleep(std::time::Duration::from_secs(5)).await;
                continue;
            }
        };
        if (com1 - atis_khz).abs() > 5 {
            tokio::time::sleep(std::time::Duration::from_secs(1)).await;
            continue;
        }

        let snap = match read_snapshot(&sim, &apt).await {
            Ok(s) => s,
            Err(e) => {
                tracing::warn!("weather read failed, retrying: {e:#}");
                tokio::time::sleep(std::time::Duration::from_secs(2)).await;
                continue;
            }
        };
        let fp = atis::fingerprint(&snap);
        if last_fp.as_ref() != Some(&fp) {
            if last_fp.is_some() {
                letter = atis::next_letter(letter);
            }
            last_fp = Some(fp);
        }

        let text = phrase::scrub_dashes(&atis::report(&snap, letter));
        tracing::info!(%text, "broadcasting");
        // Blocking speech is fine here: broadcasting is this process's only
        // job, and the frequency is re-checked between repeats. Off the async
        // worker so tokio's timers stay live while `say` runs.
        let v = voice.clone();
        if let Err(e) = tokio::task::spawn_blocking(move || tts::speak(&v, &text)).await? {
            tracing::warn!("tts failed: {e:#}");
            tokio::time::sleep(std::time::Duration::from_secs(5)).await;
        }
        // The gap between loops of a real ATIS tape.
        tokio::time::sleep(std::time::Duration::from_millis(1500)).await;
    }
}
