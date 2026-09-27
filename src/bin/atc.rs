//! `atc`: the air traffic side of the radio, a separate process from the
//! copilot on purpose. The two share code (via the `xvoicelayer` lib target),
//! not state: both watch the same simulator through the Web API, and the
//! tuned COM1 frequency decides who answers. This process is structurally
//! READ-ONLY toward the simulator: every dataref access goes through
//! [`SimView`], which exposes no writes. The copilot is crew and acts on the
//! aircraft; ATC is the world and never touches it.
//!
//! Services, each on its own apt.dat frequency:
//!   - Tower (row 1054): two-way. Key the mic (Enter in this terminal to key,
//!     Enter again to transmit), speak a request, get a clearance back from
//!     the deterministic state machine in [`xvoicelayer::tower`].
//!   - ATIS (row 1050, or XCOPILOT_ATIS_FREQ): a looping broadcast built from
//!     live sim weather. Only exists where the data (or the env) says so —
//!     it never squats on the tower frequency, and it respects the audio
//!     panel's COM1 monitor switch so it can be muted like a real radio.
//!
//! Environment:
//!   XCOPILOT_ATC_AIRPORT  ICAO of the field to serve (default LXGB)
//!   XCOPILOT_ATIS_FREQ    ATIS frequency, MHz ("131.2") or kHz ("131200")
//!   XCOPILOT_ATC_VOICE    macOS `say` voice for the station (default Daniel)
//!   XCOPILOT_ATC_RATE     `say` words-per-minute (default 165, a read pace)

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use xvoicelayer::{
    atis, config::Config, datarefs, mic, nav, phrase, stt, tower, tts, xplane::XPlaneClient,
};

/// Read-only view of the simulator. ATC observes and speaks; it must never
/// actuate anything, so only reads are reachable through this type. Sync
/// wrappers (this binary has no other async work worth a runtime's ceremony).
struct SimView {
    client: XPlaneClient,
    rt: tokio::runtime::Runtime,
}

impl SimView {
    fn new(base: String) -> Result<Self> {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .context("building tokio runtime")?;
        Ok(SimView {
            client: XPlaneClient::new(base),
            rt,
        })
    }
    fn get_float(&self, name: &str) -> Result<f64> {
        self.rt.block_on(self.client.get_float(name))
    }
    fn get_string(&self, name: &str) -> Result<String> {
        self.rt.block_on(self.client.get_string(name))
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
    std::env::var(key)
        .ok()
        .filter(|v| !v.is_empty())
        .unwrap_or_else(|| default.to_string())
}

/// 8.33 channel names can sit 5 kHz off the actual carrier (118.330 names
/// 118.325); adjacent 8.33 channels are 8 kHz apart so 5 cannot cross over.
fn tuned(com1_khz: i64, station_khz: i64) -> bool {
    (com1_khz - station_khz).abs() <= 5
}

/// One weather snapshot off the sim, plus the wind-preferred runway.
fn read_snapshot(sim: &SimView, apt: &nav::AirportInfo) -> Result<atis::Snapshot> {
    let wind_from_deg = sim.get_float(datarefs::WIND_DIR_DEG_TRUE)?;
    let wind_kts = sim.get_float(datarefs::WIND_SPEED_MSC)? * 1.943844;
    // Wind and runway headings are both degrees true, so the pick is consistent
    // even though the spoken direction skips the (tiny, ~1 degree at LXGB)
    // magnetic conversion for now.
    let runway = nav::best_runway_for_wind(&apt.runways, wind_from_deg, wind_kts)
        .map(|(id, _, _)| id)
        .unwrap_or_default();
    Ok(atis::Snapshot {
        airport: apt.name.clone(),
        zulu_sec: sim.get_float(datarefs::ZULU_TIME_SEC)?,
        runway,
        wind_from_deg,
        wind_kts,
        visibility_sm: sim.get_float(datarefs::VISIBILITY_SM)?,
        qnh_hpa: sim.get_float(datarefs::QNH_PASCALS)? / 100.0,
        oat_c: sim.get_float(datarefs::OAT_DEG_C)?,
    })
}

/// Wind spoken tower-style off a snapshot ("zero niner zero degrees, three
/// knots", or "calm").
fn spoken_wind(s: &atis::Snapshot) -> String {
    if s.wind_kts < 3.0 {
        return "calm".to_string();
    }
    let dir = ((s.wind_from_deg / 10.0).round() as i64 * 10).rem_euclid(360);
    let dir = if dir == 0 { 360 } else { dir };
    format!(
        "{} degrees, {} knots",
        atis::spoken_digits(&format!("{dir:03}")),
        atis::spoken_digits(&format!("{:.0}", s.wind_kts))
    )
}

/// Speak as the station, raising the mic gate so the PTT capture can't hear
/// our own voice over the speakers.
fn transmit(voice: &Config, gate: &mic::MicGate, text: &str) {
    let text = phrase::scrub_dashes(text);
    tracing::info!(%text, "transmitting");
    gate.set_speaking(true);
    if let Err(e) = tts::speak(voice, &text) {
        tracing::warn!("tts failed: {e:#}");
    }
    gate.set_speaking(false);
}

/// Is COM1 audible on the audio panel? Unknown (dataref missing on some
/// airframes) counts as audible — better to speak than to go silently deaf.
fn com1_audible(sim: &SimView) -> bool {
    sim.get_float(datarefs::AUDIO_SELECTION_COM1)
        .map(|v| v > 0.5)
        .unwrap_or(true)
}

fn main() -> Result<()> {
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

    let tower_khz = apt.freq(nav::FREQ_TOWER).map(|f| f.khz);
    // ATIS only where the data or the env says so — never on the tower.
    let mut atis_khz = match std::env::var("XCOPILOT_ATIS_FREQ")
        .ok()
        .filter(|v| !v.is_empty())
    {
        Some(v) => Some(parse_freq_khz(&v)?),
        None => apt.freq(nav::FREQ_ATIS).map(|f| f.khz),
    };
    if atis_khz.is_some() && atis_khz == tower_khz {
        tracing::warn!("ATIS frequency equals the tower frequency; tower wins, ATIS disabled");
        atis_khz = None;
    }
    if tower_khz.is_none() && atis_khz.is_none() {
        return Err(anyhow!(
            "{icao} has no tower or ATIS frequency; set XCOPILOT_ATIS_FREQ or pick another field"
        ));
    }
    match tower_khz {
        Some(k) => tracing::info!(mhz = k as f64 / 1000.0, "{} Tower on frequency", apt.name),
        None => tracing::info!("no tower row for {icao}; tower service off"),
    }
    match atis_khz {
        Some(k) => tracing::info!(mhz = k as f64 / 1000.0, "ATIS on frequency"),
        None => tracing::info!("no ATIS row for {icao}; ATIS off (XCOPILOT_ATIS_FREQ enables it)"),
    }

    // The station voice: deliberately not the copilot's.
    let voice = Config {
        tts_backend: "say".into(),
        say_voice: env_or("XCOPILOT_ATC_VOICE", "Daniel"),
        say_rate: env_or("XCOPILOT_ATC_RATE", "165"),
        ..Config::default()
    };

    let sim = SimView::new(config.xplane_base_url.clone())?;

    // Radio PTT: Enter in this terminal keys the mic, Enter again transmits.
    // (A held key or a plugin dataref can replace this later; the capture only
    // sees a closure.) The stdin thread just toggles the flag per line.
    let ptt = Arc::new(AtomicBool::new(false));
    {
        let ptt = ptt.clone();
        std::thread::spawn(move || {
            let mut line = String::new();
            loop {
                line.clear();
                if std::io::stdin().read_line(&mut line).is_err() {
                    return;
                }
                let keyed = !ptt.load(Ordering::Relaxed);
                ptt.store(keyed, Ordering::Relaxed);
                eprintln!(
                    "{}",
                    if keyed {
                        "[mic keyed - speak, then press Enter]"
                    } else {
                        "[transmitting]"
                    }
                );
            }
        });
    }

    let gate = mic::MicGate::new();
    let (ptt_mic, whisper) = if tower_khz.is_some() {
        tracing::info!(model = %config.whisper_model_path, "loading whisper for the tower");
        let mut w = stt::Stt::load(&config.whisper_model_path, config.whisper_gpu)?;
        w.set_prompt(tower::STT_PROMPT.to_string());
        let m = mic::PttMic::start(gate.clone(), Duration::from_millis(700))
            .context("opening the mic (tower needs one; is a mic connected?)")?;
        tracing::info!(
            "tower ready — tune COM1, press Enter to key the mic, Enter again to transmit"
        );
        (Some(m), Some(w))
    } else {
        (None, None)
    };

    let mut twr = tower::Tower::new();
    let mut letter = 'A';
    let mut edition: Option<atis::Snapshot> = None;
    let mut sim_down = false;

    loop {
        let com1 = match sim.get_float(datarefs::COM1_FREQ_KHZ) {
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
                std::thread::sleep(Duration::from_secs(5));
                continue;
            }
        };

        // --- Tower: wait (briefly) for a transmission, answer it ---
        if let (Some(twr_khz), Some(ptt_mic), Some(whisper)) = (tower_khz, &ptt_mic, &whisper) {
            if tuned(com1, twr_khz) {
                // Blocks up to 1 s waiting for a key-down, then until key-up —
                // the 1 s timeout doubles as the COM1 re-poll cadence.
                let audio = ptt_mic
                    .capture(|| ptt.load(Ordering::Relaxed), Some(Duration::from_secs(1)))?;
                if audio.len() < mic::WHISPER_SAMPLE_RATE as usize / 4 {
                    continue; // nothing (or a sub-quarter-second blip)
                }
                let heard = whisper.transcribe(&audio)?;
                if heard.text.is_empty() {
                    continue;
                }
                tracing::info!(text = %heard.text, confidence = heard.confidence, "received");

                let snap = match read_snapshot(&sim, &apt) {
                    Ok(s) => s,
                    Err(e) => {
                        tracing::warn!("weather read failed: {e:#}");
                        continue;
                    }
                };
                let mtow_kg = sim.get_float(datarefs::ACF_M_MAX_KG).ok();
                let callsign = sim
                    .get_string(datarefs::TAILNUM)
                    .ok()
                    .map(|t| tower::spoken_callsign_wake(&t, mtow_kg))
                    .filter(|s| !s.is_empty())
                    .unwrap_or_else(|| "aircraft calling".to_string());
                let ctx = tower::Ctx {
                    callsign,
                    airport: apt.name.clone(),
                    runway: atis::spoken_runway(&snap.runway),
                    qnh: atis::spoken_digits(&format!("{:.0}", snap.qnh_hpa)),
                    wind: spoken_wind(&snap),
                    holding_point: "Alpha".to_string(),
                    stand: String::new(),
                    approach: apt
                        .freq(nav::FREQ_APPROACH)
                        .map(|f| atis::spoken_mhz(f.khz)),
                    // The standalone binary predates the ground-aware mayday
                    // and the assigned-squawk state: conspicuity only. Same
                    // for the traffic/wake/taxi-route context — no TCAS or
                    // taxi network here.
                    squawk: "seven zero zero zero".to_string(),
                    final_traffic_nm: None,
                    final_traffic_callsign: None,
                    final_traffic_count: 0,
                    runway_occupied: false,
                    wake_secs_left: None,
                    taxi_via: Vec::new(),
                    taxi_crossings: Vec::new(),
                    on_ground: false,
                    own_on_runway: false,
                    helicopter: false,
                    // No airport index here: an IFR request is asked to say
                    // its destination again, never cleared.
                    ifr: None,
                    runway_refusal: None,
                };
                let intent = if heard.confidence < config.stt_min_confidence {
                    tower::Intent::Unknown // garbled call → "say again", never a guessed clearance
                } else {
                    tower::parse_intent(&heard.text)
                };
                let reply = twr.handle(intent, &ctx);
                transmit(&voice, &gate, &reply);
                continue;
            }
        }

        // --- ATIS: loop the broadcast while tuned and audible ---
        if let Some(atis_khz) = atis_khz {
            if tuned(com1, atis_khz) {
                if !com1_audible(&sim) {
                    std::thread::sleep(Duration::from_secs(1));
                    continue; // COM1 deselected on the audio panel — muted
                }
                let snap = match read_snapshot(&sim, &apt) {
                    Ok(s) => s,
                    Err(e) => {
                        tracing::warn!("weather read failed, retrying: {e:#}");
                        std::thread::sleep(Duration::from_secs(2));
                        continue;
                    }
                };
                match &edition {
                    Some(e) if atis::new_edition(e, &snap) => {
                        letter = atis::next_letter(letter);
                        edition = Some(snap.clone());
                    }
                    None => edition = Some(snap.clone()),
                    Some(_) => {} // same edition: speak the letter it was cut as
                }
                transmit(&voice, &gate, &atis::report(&snap, letter));
                std::thread::sleep(Duration::from_millis(1500));
                continue;
            }
        }

        std::thread::sleep(Duration::from_secs(1));
    }
}
