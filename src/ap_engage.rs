//! Autopilot engage: the one home (refactor audit 2026-10-04,
//! `docs/tasks/refactor-audit/audit-duplicates.md` #5).
//!
//! The rule: "The autopilot is engaged and CONFIRMED, never assumed": the
//! flight director commands first (`autopilot_prep.flight_director`: the
//! C90's `fd_monitor` drops the servos unless its FD switch is up), then
//! `servos_on`, pressed until the servos READ on with the sim live, each try
//! blackboxed `ap_engage`, the confirmation `ap_engaged`.
//!
//! [`ApEngage`] was autoland's (its arm and `setup_enroute` used it); every
//! other loop pressed `servos_on` blind, without the FD commands, so on an
//! FD airframe its "re-engage" guard pressed a button that could not hold.
//! Each site now goes through one of three doors, chosen by its shape:
//!
//! - a tick loop that keeps the autopilot in ([`ServoWatch`]): a fresh
//!   [`ApEngage`] each time the servos read OFF, stepped by the loop's own
//!   tick, never a blocking wait;
//! - a one-shot setup followed by its own watch ([`press_logged`]): FD then
//!   servos, on the tape, the confirmation left to the site's watch;
//! - a one-shot with nothing after it to confirm ([`engage_confirmed`]):
//!   the [`ApEngage`] loop, bounded.
//!
//! What each site does with a give-up stays the site's: autoland's arm hands
//! the approach over, the circuit's `PatternGuard` judges the servos out.

use crate::xplane::XPlane;

/// The servo master. Not a toggle: pressing it with the servos in is
/// harmless.
pub const CMD_SERVOS_ON: &str = "sim/autopilot/servos_on";

/// THE AUTOPILOT IS ENGAGED AND CONFIRMED, never assumed. The arm used to
/// fire `servos_on` once and never read it back. After `setup_approach` the
/// sim is still starting the new flight when autoland arms (IAS reads 0 to
/// 12 kt for the first second), and that one press was lost: LEMG 13
/// (sortie 1790587073-approach, 2026-09-28) flew with `ap_state` 132 and the
/// servos off from the arm until the VS watchdog's re-press at +9.5 s, the
/// aircraft flown by nobody while autoland commanded a climb: -900 fpm at
/// the start growing to -3,450, 2,318 to 1,697 ft MSL, 405 ft AGL over the
/// rising ground. Two of six runs lost the press the same way (1790583434,
/// 1790583663 reached 200 and 145 ft AGL); the re-arms that engaged did so
/// on the re-arm's own press.
///
/// So the press is re-fired every [`ENGAGE_RETRY_S`] until the servos READ
/// on for [`ENGAGE_CONFIRM_READS`] ticks running with the sim live (an
/// airspeed that is a flight's, [`ENGAGE_LIVE_IAS_KTS`]: the flight start
/// can drop an engage made before it), each try blackboxed `ap_engage`. A
/// failed read is no measurement: it neither confirms nor breaks the run.
/// Until confirmed, with the servos read OFF, the loop holds the aircraft
/// level itself on the elevator channel ([`crate::autoland::engage_hold_gains`]). No
/// confirmation within [`ENGAGE_GIVE_UP_S`] of live flight is said and the
/// approach handed over.
#[derive(Debug, Clone, Default)]
pub struct ApEngage {
    tries: u32,
    last_fire_s: Option<f64>,
    live_since_s: Option<f64>,
    on_reads: u32,
    /// The last GOOD servo read (true = on); `None` = never read.
    last_read: Option<bool>,
    confirmed: bool,
}

/// Seconds between presses while the servos do not read on: every tick at
/// the loop's 0.6 s (`servos_on` is not a toggle).
pub const ENGAGE_RETRY_S: f64 = 0.5;
/// Consecutive live ticks the servos must read on to be confirmed.
pub const ENGAGE_CONFIRM_READS: u32 = 2;
/// An airspeed that is a flight's, not a flight still being started.
pub const ENGAGE_LIVE_IAS_KTS: f64 = 30.0;
/// Seconds of live flight without a confirmed engage before the approach is
/// handed over.
pub const ENGAGE_GIVE_UP_S: f64 = 10.0;

/// What the engage watch wants this tick.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EngageStep {
    /// Press the flight director and `servos_on` now (try number `try_n`).
    pub fire: bool,
    pub try_n: u32,
    /// Hold the aircraft on the elevator channel this tick: not confirmed,
    /// and the servos last READ off.
    pub hold: bool,
    /// Confirmed on this tick (the record is written once).
    pub confirmed_now: bool,
    /// No engage within the bound: say it and hand over.
    pub give_up: bool,
}

impl ApEngage {
    /// The press at the arm (try 1): the arm fires before the first tick.
    pub fn arm(&mut self, now_s: f64) -> EngageStep {
        self.tries = 1;
        self.last_fire_s = Some(now_s);
        EngageStep {
            fire: true,
            try_n: 1,
            hold: false,
            ..Default::default()
        }
    }

    pub fn confirmed(&self) -> bool {
        self.confirmed
    }

    pub fn tries(&self) -> u32 {
        self.tries
    }

    /// One tick. `servos` is the `servos_on` read (`None` = failed read),
    /// `ias_kts` the airspeed read this tick.
    pub fn step(&mut self, now_s: f64, servos: Option<f64>, ias_kts: f64) -> EngageStep {
        if self.confirmed {
            return EngageStep::default();
        }
        let live = ias_kts.is_finite() && ias_kts >= ENGAGE_LIVE_IAS_KTS;
        if live && self.live_since_s.is_none() {
            self.live_since_s = Some(now_s);
        }
        match servos {
            Some(v) if v > 0.5 => {
                self.last_read = Some(true);
                if live {
                    self.on_reads += 1;
                } else {
                    self.on_reads = 0;
                }
            }
            Some(_) => {
                self.last_read = Some(false);
                self.on_reads = 0;
            }
            None => {}
        }
        if self.on_reads >= ENGAGE_CONFIRM_READS {
            self.confirmed = true;
            return EngageStep {
                confirmed_now: true,
                try_n: self.tries,
                ..Default::default()
            };
        }
        if self
            .live_since_s
            .is_some_and(|t0| now_s - t0 >= ENGAGE_GIVE_UP_S)
        {
            return EngageStep {
                give_up: true,
                try_n: self.tries,
                ..Default::default()
            };
        }
        let due = self
            .last_fire_s
            .is_none_or(|t| now_s - t >= ENGAGE_RETRY_S - 1e-9);
        // Pressed while the servos last READ off, or were never read; a
        // failed read after an "on" waits for the next measurement.
        let fire = due && self.last_read != Some(true);
        if fire {
            self.tries += 1;
            self.last_fire_s = Some(now_s);
        }
        EngageStep {
            fire,
            try_n: self.tries,
            hold: self.last_read == Some(false),
            ..Default::default()
        }
    }
}

/// The live airframe's flight director commands (`autopilot_prep`), for the
/// sites that carry no `ApConfig`. Empty = plain servos-on.
pub fn fd_commands() -> Vec<String> {
    crate::aircraft::live_profile()
        .and_then(|p| p.autopilot_prep.as_ref().map(|a| a.flight_director.clone()))
        .unwrap_or_default()
}

/// One press: the flight director commands first (each an "up" or "on"
/// command, never a toggle), then the servos. True when the servo command
/// was sent, which is not the servos being in.
pub async fn press(xp: &dyn XPlane, fd: &[String]) -> bool {
    for c in fd {
        let _ = xp.run_command(c).await;
    }
    xp.run_command(CMD_SERVOS_ON).await.is_ok()
}

/// [`press`], on the tape: blackbox `kind` `ap_engage` with the `site` and
/// the try number.
pub async fn press_logged(xp: &dyn XPlane, fd: &[String], kind: &str, site: &str, try_n: u32) -> bool {
    let sent = press(xp, fd).await;
    crate::blackbox::record(
        kind,
        serde_json::json!({"event": "ap_engage", "site": site, "try": try_n, "sent": sent,
                           "fd": !fd.is_empty()}),
    );
    sent
}

/// The confirmation, on the tape: blackbox `kind` `ap_engaged`.
pub fn record_engaged(kind: &str, site: &str, tries: u32) {
    crate::blackbox::record(
        kind,
        serde_json::json!({"event": "ap_engaged", "site": site, "tries": tries}),
    );
}

/// A tick loop's re-engage, as [`ApEngage`] engages autoland: a fresh engage
/// each time the site WANTS the autopilot back (its own trigger, e.g. the
/// heading mode read off) and the servos read OFF; pressed every
/// [`ENGAGE_RETRY_S`] until they read on for [`ENGAGE_CONFIRM_READS`] live
/// ticks. A failed servo read is no measurement: it never starts an engage.
/// Once the site stops wanting it (the mode read back on), no further press
/// goes out, but the engage in progress is still stepped to its
/// confirmation, so the tape says whether the servos came in. A give-up
/// ends the engage and the next wanted tick starts another: what to do with
/// an autopilot that will not come in stays the site's guard (the circuit's
/// `PatternGuard`, the follower's watchdogs). Never the level hold: these
/// loops fly their own vertical. Pure.
#[derive(Debug, Clone, Default)]
pub struct ServoWatch {
    engage: Option<ApEngage>,
}

impl ServoWatch {
    /// One tick: `wanted` is the site's trigger, `servos` the `servos_on`
    /// read (`None` = failed read), `ias_kts` the airspeed read.
    pub fn step(&mut self, now_s: f64, wanted: bool, servos: Option<f64>, ias_kts: f64) -> EngageStep {
        let servos = servos.filter(|v| v.is_finite());
        match self.engage.as_mut() {
            None => {
                if wanted && servos.is_some_and(|v| v <= 0.5) {
                    let mut e = ApEngage::default();
                    let st = e.arm(now_s);
                    self.engage = Some(e);
                    st
                } else {
                    EngageStep::default()
                }
            }
            Some(e) => {
                let mut st = e.step(now_s, servos, ias_kts);
                if !wanted {
                    st.fire = false;
                }
                if st.confirmed_now || st.give_up {
                    self.engage = None;
                }
                st
            }
        }
    }

    /// An engage in progress.
    #[cfg(test)]
    pub fn engaging(&self) -> bool {
        self.engage.is_some()
    }
}

/// Seconds between reads in [`engage_confirmed`]: the engage's own retry.
const CONFIRM_POLL_S: f64 = ENGAGE_RETRY_S;

/// A one-shot engage with nothing after it to confirm it (a tool's reply):
/// [`ApEngage`] driven here, pressed (FD first) and read every
/// [`CONFIRM_POLL_S`] until confirmed, at most `max_s`. Each press is
/// blackboxed `ap_engage`, the confirmation `ap_engaged`. True = the servos
/// READ in for [`ENGAGE_CONFIRM_READS`] live reads.
pub async fn engage_confirmed(xp: &dyn XPlane, fd: &[String], kind: &str, site: &str, max_s: f64) -> bool {
    let mut e = ApEngage::default();
    let t0 = std::time::Instant::now();
    let mut step = e.arm(0.0);
    loop {
        if step.fire {
            press_logged(xp, fd, kind, site, step.try_n).await;
        }
        if step.confirmed_now {
            record_engaged(kind, site, step.try_n);
            return true;
        }
        if step.give_up || t0.elapsed().as_secs_f64() >= max_s {
            return false;
        }
        tokio::time::sleep(std::time::Duration::from_secs_f64(CONFIRM_POLL_S)).await;
        let (servos, ias) = tokio::join!(
            xp.get_float(crate::datarefs::SERVOS_ON),
            xp.get_float(crate::datarefs::AIRSPEED_KTS),
        );
        let ias = ias.ok().filter(|v| v.is_finite()).unwrap_or(0.0);
        step = e.step(t0.elapsed().as_secs_f64(), servos.ok(), ias);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_watch_starts_only_when_wanted_and_read_off() {
        let mut w = ServoWatch::default();
        assert!(!w.step(0.0, false, Some(0.0), 120.0).fire, "not wanted");
        assert!(!w.step(0.5, true, Some(1.0), 120.0).fire, "servos in");
        assert!(!w.step(1.0, true, None, 120.0).fire, "a failed read is no measurement");
        assert!(!w.engaging());
        let s = w.step(1.5, true, Some(0.0), 120.0);
        assert!(s.fire && s.try_n == 1, "{s:?}");
        assert!(w.engaging());
    }

    #[test]
    fn the_watch_presses_until_confirmed_then_rests() {
        let mut w = ServoWatch::default();
        assert!(w.step(0.0, true, Some(0.0), 120.0).fire);
        let s = w.step(0.5, true, Some(0.0), 120.0);
        assert!(s.fire && s.try_n == 2, "re-pressed at the retry: {s:?}");
        assert!(!w.step(1.0, true, Some(1.0), 120.0).confirmed_now, "one read is not two");
        let s = w.step(1.5, true, Some(1.0), 120.0);
        assert!(s.confirmed_now && s.try_n == 2, "{s:?}");
        assert!(!w.engaging());
        // In: nothing more until the servos read off again.
        assert!(!w.step(2.0, true, Some(1.0), 120.0).fire);
        assert!(w.step(2.5, true, Some(0.0), 120.0).fire, "a fresh engage");
    }

    #[test]
    fn the_watch_never_presses_once_the_site_stops_wanting_it() {
        let mut w = ServoWatch::default();
        assert!(w.step(0.0, true, Some(0.0), 120.0).fire);
        // The mode read back on: no press, the engage still confirms.
        assert!(!w.step(0.5, false, Some(0.0), 120.0).fire);
        assert!(!w.step(1.0, false, Some(1.0), 120.0).confirmed_now);
        assert!(w.step(1.5, false, Some(1.0), 120.0).confirmed_now);
    }

    #[test]
    fn a_give_up_ends_the_engage_and_the_next_wanted_tick_starts_another() {
        let mut w = ServoWatch::default();
        let mut t = 0.0;
        assert!(w.step(t, true, Some(0.0), 120.0).fire);
        let mut gave = false;
        while t < ENGAGE_GIVE_UP_S + 2.0 {
            t += 0.5;
            if w.step(t, true, Some(0.0), 120.0).give_up {
                gave = true;
                break;
            }
        }
        assert!(gave, "gave up by {t}");
        assert!(!w.engaging());
        let s = w.step(t + 0.5, true, Some(0.0), 120.0);
        assert!(s.fire && s.try_n == 1, "{s:?}");
    }

    #[tokio::test]
    async fn a_press_raises_the_flight_director_first() {
        let xp = crate::xplane::FakeXPlane::with_reads(&[]);
        let fd = vec!["laminar/c90/avionics/switch/fd_up".to_string()];
        assert!(press(&xp, &fd).await);
        let fd_at = xp.command_index(&fd[0]).expect("FD pressed");
        let servos_at = xp.command_index(CMD_SERVOS_ON).expect("servos pressed");
        assert!(fd_at < servos_at, "FD before the servos");
    }

    #[tokio::test]
    async fn a_one_shot_engage_is_confirmed_from_the_reads() {
        let xp = crate::xplane::FakeXPlane::with_reads(&[
            (crate::datarefs::SERVOS_ON, 1.0),
            (crate::datarefs::AIRSPEED_KTS, 120.0),
        ]);
        assert!(engage_confirmed(&xp, &[], "test", "unit", 3.0).await);
        assert!(xp.ran(CMD_SERVOS_ON));
        let dead = crate::xplane::FakeXPlane::with_reads(&[
            (crate::datarefs::SERVOS_ON, 0.0),
            (crate::datarefs::AIRSPEED_KTS, 120.0),
        ]);
        assert!(!engage_confirmed(&dead, &[], "test", "unit", 1.2).await);
    }
}
