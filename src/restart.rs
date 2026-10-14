//! "Restart the crew" from the one app (`src/app.rs`): ON THE GROUND ONLY.
//!
//! An in-process re-boot is not sound: the boot spawns a dozen threads
//! with no stop (the monitor, the safe hold, the aircraft detector, the
//! pollers, the radio panel, the health poll, the EFB server holding its
//! port, the command loop), registers process-wide facts once
//! (`voice::RUNNING`, `LISTEN_OFF`, the callsign providers), and the
//! safe hold is never stood down by design. Running a second boot beside
//! them would make two writers of every dataref they own. So the restart
//! is the process starting again in place: the same binary, the same
//! arguments, the same terminal, the environment AS THE SHELL GAVE IT
//! ([`capture_original_env`], before any `.env` was loaded), so a pin moved
//! out of `.env` (`env_import`) stops pinning and every boot knob is read
//! afresh. It is what "quit and start xfc again" does, done for the pilot.
//!
//! The gate ([`verdict`], pure): nothing of ours holds the aircraft
//! (`flight_state`), the sim reads on the ground and stopped
//! (`sim::ground`), or X-Plane is not running at all (the Web API refuses
//! the connection: no flight to leave). A failed read is no measurement:
//! refused, said.

use std::ffi::OsString;
use std::sync::OnceLock;

/// The environment as the shell gave it, before `.env` was loaded. Process
/// lifetime; set once, first thing in `main`.
static ORIGINAL_ENV: OnceLock<Vec<(OsString, OsString)>> = OnceLock::new();

/// Remember the shell's environment. First thing in `main`, before
/// `Config::from_env` loads any `.env` into the process.
pub fn capture_original_env() {
    let _ = ORIGINAL_ENV.set(std::env::vars_os().collect());
}

/// The variable was in the shell's own environment.
fn in_original_env(name: &str) -> bool {
    ORIGINAL_ENV
        .get()
        .is_some_and(|env| env.iter().any(|(k, _)| k == name))
}

/// What the sim said about where we are.
#[derive(Clone, Debug, PartialEq)]
pub enum SimAnswer {
    /// The Web API refused the connection: X-Plane is not running.
    NotRunning,
    /// It answered (or timed out) but a read failed: no measurement.
    Unread(String),
    Read {
        on_ground: bool,
        gs_kts: f64,
    },
}

/// Whether the crew may restart now (pure). `held` names a loop of ours
/// holding the aircraft or a lever.
pub fn verdict(held: Option<&str>, sim: &SimAnswer) -> Result<(), String> {
    if let Some(what) = held {
        return Err(format!(
            "not restarting: the crew holds {what}. Say \"my controls\" first, then restart on the ground"
        ));
    }
    match sim {
        SimAnswer::NotRunning => Ok(()),
        SimAnswer::Unread(why) => Err(format!(
            "not restarting: the sim does not say we are on the ground ({why}). Quit (q) and start xfc again once you are sure"
        )),
        SimAnswer::Read {
            on_ground: false, ..
        } => Err(
            "not restarting while airborne: the crew restarts on the ground only. The changed settings wait for the next start"
                .into(),
        ),
        SimAnswer::Read { gs_kts, .. } if !sim::ground::stopped_kts(*gs_kts) => Err(format!(
            "not restarting while rolling ({gs_kts:.0} kt): stop first"
        )),
        SimAnswer::Read { .. } => Ok(()),
    }
}

/// What of ours holds the aircraft now, by name, or `None`.
pub fn held_by_crew() -> Option<String> {
    if let Some(owner) = crate::flight_state::crew_has_aircraft() {
        return Some(owner.name().to_string());
    }
    crate::flight_state::Held::now()
        .any()
        .then(|| "a lever (the speed, mixture, reverse or a loop)".to_string())
}

/// Ask the sim where we are: ONE GET to see it is there, then the ground
/// and the groundspeed in one round. Blocking: run it on a worker.
pub fn read_sim(base_url: &str) -> SimAnswer {
    let url = format!("{}/api/capabilities", base_url.trim_end_matches('/'));
    let client = match reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_millis(1500))
        .build()
    {
        Ok(c) => c,
        Err(e) => return SimAnswer::Unread(e.to_string()),
    };
    if let Err(e) = client.get(&url).send() {
        return if e.is_connect() {
            SimAnswer::NotRunning
        } else {
            SimAnswer::Unread(e.to_string())
        };
    }
    let rt = match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => return SimAnswer::Unread(e.to_string()),
    };
    let xp = crate::xplane::XPlaneClient::new(base_url.to_string());
    let (g, gs) = rt.block_on(async {
        tokio::join!(
            xp.get_float(crate::datarefs::ON_GROUND_ANY),
            xp.get_float(crate::datarefs::GROUNDSPEED_MS)
        )
    });
    match (g, gs) {
        (Ok(g), Ok(gs)) if g.is_finite() && gs.is_finite() => SimAnswer::Read {
            on_ground: sim::ground::on_ground(g),
            gs_kts: gs * sim::ground::MS_TO_KTS,
        },
        (Err(e), _) | (_, Err(e)) => SimAnswer::Unread(format!("{e:#}")),
        _ => SimAnswer::Unread("a read was not a number".into()),
    }
}

/// The registry knobs pinned in THIS process by a `.env` line that has
/// since been moved into xfc settings (`env_import`): set now, not in the
/// shell's own environment, no longer in any `.env`. A restart clears them.
pub fn stale_pins() -> Vec<&'static str> {
    if ORIGINAL_ENV.get().is_none() {
        return Vec::new();
    }
    let still = crate::env_import::pending();
    crate::settings::ALL
        .iter()
        .filter(|s| s.env_pinned() && !in_original_env(s.env))
        .filter(|s| !still.iter().any(|m| m.env == s.env))
        .map(|s| s.label)
        .collect()
}

/// What waits for a restart, by label: the boot knobs changed since this
/// start, then the pins moved out of `.env`.
pub fn waiting() -> Vec<String> {
    let mut out: Vec<String> = crate::config::restart_pending()
        .into_iter()
        .map(|env| {
            crate::config::BOOT_KNOBS
                .iter()
                .find(|(e, _)| *e == env)
                .and_then(|(_, key)| crate::settings::ALL.iter().find(|s| s.key == *key))
                .map_or(env.to_string(), |s| s.label.to_string())
        })
        .collect();
    for label in stale_pins() {
        let line = format!("{label} (moved from .env)");
        if !out.contains(&line) {
            out.push(line);
        }
    }
    out
}

/// Start this process again in place: the same binary and arguments, the
/// shell's environment. Returns only on failure, with the error to say.
pub fn exec_again() -> std::io::Error {
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt;
        let exe = match std::env::current_exe() {
            Ok(p) => p,
            Err(e) => return e,
        };
        let mut cmd = std::process::Command::new(exe);
        cmd.args(std::env::args_os().skip(1));
        if let Some(env) = ORIGINAL_ENV.get() {
            cmd.env_clear();
            cmd.envs(env.iter().cloned());
        }
        cmd.exec()
    }
    #[cfg(not(unix))]
    {
        std::io::Error::other("restarting in place needs a Unix system: quit and start xfc again")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_crew_restarts_on_the_ground_and_stopped_only() {
        let ground = SimAnswer::Read {
            on_ground: true,
            gs_kts: 0.2,
        };
        assert_eq!(verdict(None, &ground), Ok(()));
        let air = SimAnswer::Read {
            on_ground: false,
            gs_kts: 0.0,
        };
        let e = verdict(None, &air).unwrap_err();
        assert!(e.contains("airborne"), "{e}");
        let rolling = SimAnswer::Read {
            on_ground: true,
            gs_kts: 12.0,
        };
        let e = verdict(None, &rolling).unwrap_err();
        assert!(e.contains("rolling (12 kt)"), "{e}");
    }

    #[test]
    fn a_loop_holding_the_aircraft_refuses_even_on_the_ground() {
        let ground = SimAnswer::Read {
            on_ground: true,
            gs_kts: 0.0,
        };
        let e = verdict(Some("auto taxi"), &ground).unwrap_err();
        assert!(e.contains("auto taxi") && e.contains("my controls"), "{e}");
    }

    #[test]
    fn a_failed_read_is_no_measurement_but_no_sim_is_no_flight() {
        let e = verdict(None, &SimAnswer::Unread("timed out".into())).unwrap_err();
        assert!(e.contains("timed out") && e.contains("Quit (q)"), "{e}");
        assert_eq!(verdict(None, &SimAnswer::NotRunning), Ok(()));
        // Not running still refuses a loop of ours (it would be lost).
        assert!(verdict(Some("full auto"), &SimAnswer::NotRunning).is_err());
    }

    #[test]
    fn no_line_says_a_dash() {
        for line in [
            verdict(Some("x"), &SimAnswer::NotRunning).unwrap_err(),
            verdict(None, &SimAnswer::Unread("y".into())).unwrap_err(),
        ] {
            assert!(!line.contains('\u{2014}') && !line.contains('\u{2013}'));
        }
    }
}
