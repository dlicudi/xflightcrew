//! Sidecar TTS backend (`tts_backend = "sidecar"`, the default).
//!
//! Spark-TTS (the voice-clone model) is MLX/Python with no Rust runtime, so the
//! cloned "Roger" voice is served by a small warm Python process
//! ([`voice/spark_sidecar.py`]) that keeps the model loaded. This backend POSTs
//! the readback text to it and plays the returned WAV — fully local, just a
//! separate process. The model stays warm across calls (the whole reason for a
//! server rather than a per-utterance subprocess).
//!
//! By default the app **owns that process**: [`SidecarTts::spawn`] finds an
//! mlx-audio Python, launches the sidecar, waits for it to report `ready.`, and
//! supervises it for the rest of the run — killing it on shutdown (and, via an
//! stdin watchdog, even on a hard Ctrl-C). If no suitable Python is found or the
//! sidecar never comes up, the caller falls back to plain `say`, so a fresh
//! checkout is never left mute. Set `XCOPILOT_TTS_SIDECAR_AUTOSPAWN=false` to
//! talk to a sidecar you started yourself ([`SidecarTts::connect`]).

use std::io::{BufRead, BufReader};
use std::net::TcpStream;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};

/// The sidecar script, relative to the working directory (the repo root).
const SCRIPT: &str = "voice/spark_sidecar.py";

/// Python interpreters to try, in order, when none is configured. The first that
/// exists wins; it still has to actually have `mlx_audio` (we find that out at
/// launch and fall back if import fails).
const PYTHON_CANDIDATES: &[&str] = &[
    "/tmp/mlxtts/bin/python", // the documented mlx-audio venv
    ".venv/bin/python",
    "voice/.venv/bin/python",
];

/// Talks to the warm Spark sidecar over HTTP, optionally owning its process.
pub struct SidecarTts {
    url: String,
    client: reqwest::blocking::Client,
    /// Present when we spawned the sidecar (so [`Drop`] can tear it down). `None`
    /// when talking to an externally-started one.
    child: Option<Child>,
    /// Kept open for a spawned child: the sidecar's stdin watchdog exits the
    /// process when this pipe closes, so it dies with us even on a hard Ctrl-C
    /// that skips `Drop`. Never written to — its lifetime is the signal.
    _stdin: Option<ChildStdin>,
}

impl SidecarTts {
    /// Connect to an already-running sidecar (manual launch). Never fails at
    /// build; a down server surfaces per-utterance in [`speak`](Self::speak).
    pub fn connect(url: String) -> Self {
        Self { url, client: build_client(), child: None, _stdin: None }
    }

    /// Launch the sidecar, wait until it reports ready, and supervise it.
    ///
    /// `python_override` (from `XCOPILOT_TTS_SIDECAR_PYTHON`) wins if set;
    /// otherwise an mlx-audio interpreter is discovered from [`PYTHON_CANDIDATES`].
    /// `temp` is the Spark sampling temperature and `voice` the default voice
    /// name, both forwarded to the sidecar. `ready_timeout` bounds the wait for
    /// the model to load — generous, since a first-ever run downloads the model.
    /// Errors (no Python, missing script, crash on load, timeout) bubble up so
    /// the caller can fall back to `say`.
    pub fn spawn(
        url: &str,
        python_override: &str,
        temp: &str,
        voice: &str,
        ready_timeout: Duration,
    ) -> Result<Self> {
        let python = discover_python(python_override)
            .context("no mlx-audio Python found (set XCOPILOT_TTS_SIDECAR_PYTHON)")?;
        if !Path::new(SCRIPT).exists() {
            bail!("sidecar script {SCRIPT} not found (run from the repo root)");
        }
        let (host, port) = parse_host_port(url)?;
        tracing::info!(python = %python.display(), %host, port, temp, "launching Spark sidecar");

        let mut child = Command::new(&python)
            .arg(SCRIPT)
            .env("ROGER_SIDECAR_HOST", &host)
            .env("ROGER_SIDECAR_PORT", port.to_string())
            .env("ROGER_SIDECAR_TEMP", temp)
            .env("ROGER_SIDECAR_VOICE", voice)
            .env("ROGER_SIDECAR_PARENT_WATCH", "1")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .with_context(|| format!("spawning sidecar ({})", python.display()))?;

        let stdin = child.stdin.take();
        let stdout = child.stdout.take().context("sidecar stdout unavailable")?;
        let stderr = child.stderr.take();

        // Build the struct up front so any early return drops (and kills) the
        // child instead of leaking it.
        let me = Self { url: url.to_string(), client: build_client(), child: Some(child), _stdin: stdin };

        // Forward the sidecar's stderr (import errors, tracebacks) to our logs.
        if let Some(stderr) = stderr {
            std::thread::Builder::new()
                .name("sidecar-stderr".into())
                .spawn(move || {
                    for line in BufReader::new(stderr).lines().map_while(Result::ok) {
                        tracing::warn!(target: "spark_sidecar", "{}", line.trim_end());
                    }
                })
                .ok();
        }

        // Read stdout: forward every line to logs, and signal readiness when the
        // model finishes loading. If stdout hits EOF before that, the process
        // died — the channel disconnects and the wait below reports it.
        let (ready_tx, ready_rx) = mpsc::channel::<()>();
        std::thread::Builder::new()
            .name("sidecar-stdout".into())
            .spawn(move || {
                for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                    let line = line.trim_end();
                    tracing::info!(target: "spark_sidecar", "{line}");
                    if line.trim() == "ready." {
                        let _ = ready_tx.send(());
                    }
                }
            })
            .ok();

        match ready_rx.recv_timeout(ready_timeout) {
            Ok(()) => {}
            Err(RecvTimeoutError::Timeout) => {
                bail!("sidecar didn't report ready within {ready_timeout:?}")
            }
            Err(RecvTimeoutError::Disconnected) => {
                bail!("sidecar exited before becoming ready (see warnings above)")
            }
        }

        // The model is loaded; the HTTP listener binds a hair later. Poll the
        // port briefly so the first readback can't race the bind.
        wait_for_port(&host, port, Duration::from_secs(5))
            .context("sidecar reported ready but its port never opened")?;

        tracing::info!(url = %me.url, "Spark sidecar ready");
        Ok(me)
    }

    /// Synthesize `text` in the cloned voice and play it. The sidecar renders
    /// sentence-by-sentence (Spark drifts on long single generations) and returns
    /// one stitched WAV.
    pub fn speak(&self, text: &str) -> Result<()> {
        let resp = self
            .client
            .post(&self.url)
            .body(text.to_string())
            .send()
            .with_context(|| format!("POST {} (is the Spark sidecar running?)", self.url))?;
        if !resp.status().is_success() {
            bail!("sidecar returned {}", resp.status());
        }
        let bytes = resp.bytes().context("reading sidecar WAV")?;
        if bytes.is_empty() {
            bail!("sidecar returned empty audio");
        }
        let wav = std::env::temp_dir().join("x-copilot-sidecar.wav");
        std::fs::write(&wav, &bytes).with_context(|| format!("writing {}", wav.display()))?;
        crate::tts::play(&wav)
    }
}

impl Drop for SidecarTts {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            tracing::info!("stopping Spark sidecar");
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// Blocking HTTP client. Generous timeout: a cold first request waits on model
/// load; warm ones are quick. The speaker thread is synchronous.
fn build_client() -> reqwest::blocking::Client {
    reqwest::blocking::Client::builder()
        .timeout(Duration::from_secs(60))
        .build()
        .unwrap_or_else(|_| reqwest::blocking::Client::new())
}

/// Pick the Python interpreter: an explicit override, else the first existing
/// candidate venv. Returns `None` if nothing plausible is on disk.
fn discover_python(override_path: &str) -> Option<PathBuf> {
    if !override_path.is_empty() {
        return Some(PathBuf::from(override_path));
    }
    PYTHON_CANDIDATES
        .iter()
        .map(PathBuf::from)
        .find(|p| p.exists())
}

/// Split the `/synth` URL into the host and port the Python sidecar binds to.
fn parse_host_port(url: &str) -> Result<(String, u16)> {
    let parsed = reqwest::Url::parse(url).with_context(|| format!("parsing sidecar URL {url}"))?;
    let host = parsed.host_str().unwrap_or("127.0.0.1").to_string();
    let port = parsed.port_or_known_default().unwrap_or(8765);
    Ok((host, port))
}

/// Poll a TCP connect until it succeeds or the deadline passes.
fn wait_for_port(host: &str, port: u16, timeout: Duration) -> Result<()> {
    let deadline = Instant::now() + timeout;
    let addr = format!("{host}:{port}");
    loop {
        match addr.parse() {
            Ok(sa) if TcpStream::connect_timeout(&sa, Duration::from_millis(200)).is_ok() => {
                return Ok(())
            }
            _ => {}
        }
        if Instant::now() >= deadline {
            bail!("no listener on {addr} after {timeout:?}");
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_port_parsed_from_synth_url() {
        assert_eq!(
            parse_host_port("http://127.0.0.1:8765/synth").unwrap(),
            ("127.0.0.1".to_string(), 8765)
        );
        // Default port for the scheme when none is given.
        assert_eq!(parse_host_port("http://localhost/synth").unwrap().1, 80);
        assert!(parse_host_port("not a url").is_err());
    }

    #[test]
    fn python_override_wins_and_discovery_is_optional() {
        // An explicit override is returned verbatim, even if it doesn't exist
        // (launch surfaces the failure, where we can fall back).
        assert_eq!(
            discover_python("/some/python").unwrap(),
            PathBuf::from("/some/python")
        );
        // Discovery never panics; it returns None when no candidate exists.
        let _ = discover_python("");
    }

    /// End-to-end against the real mlx-audio venv: spawn, confirm it loads and
    /// becomes reachable, synthesize one line, then drop and confirm the child is
    /// gone (supervision). Ignored — needs the venv + cached model and plays audio.
    /// Run: `cargo test sidecar_spawns_and_is_supervised -- --ignored --nocapture`
    #[test]
    #[ignore = "requires the mlx-audio venv and cached Spark model"]
    fn sidecar_spawns_and_is_supervised() {
        let url = "http://127.0.0.1:8765/synth";
        let tts = SidecarTts::spawn(url, "", "0.8", "", Duration::from_secs(120))
            .expect("sidecar should spawn and report ready");
        let pid = tts.child.as_ref().expect("spawned child").id();
        tts.speak("Roger, flaps up.").expect("synth + playback");
        drop(tts);
        // After Drop, the process must be reaped — kill -0 should fail.
        let alive = Command::new("kill")
            .args(["-0", &pid.to_string()])
            .status()
            .map(|s| s.success())
            .unwrap_or(false);
        assert!(!alive, "sidecar pid {pid} should be gone after drop");
    }
}
