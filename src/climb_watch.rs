//! The autopilot's vertical answer, WATCHED (#181). A mode that reads lit
//! and a dial that reads back are a request, not a climb being flown.
//!
//! LEMG 13 (737, sortie 1790649043, 2026-09-29): VS mode lit, the servos in,
//! the dial read back at 3,934 and walked to -500 by the circuit; the aircraft
//! climbed at 4,000 then 9,600 fpm, 166 kt to 73, and the only guard (servos
//! out, or 1,500 ft over the pattern) fired at 73 kt. The same airframe had
//! zoomed from 18,000 to 26,000 ft under ALT HOLD in the case before. Nothing
//! watched what the autopilot DID with the number.
//!
//! Two ways a climb goes wrong, both judged on what the aircraft does, fed
//! every tick, pure:
//! - NOT FOLLOWING: the climb far above the dial (`DIVERGE_FPM`) for
//!   `DIVERGE_S`, not coming down (`RESPONSE_DROP_FPM`), with the speed
//!   decaying (`DECAY_KTS`). With the speed holding it is only noted: an
//!   A330 took 10 s to bring 3,700 fpm to its dial at 180 kt and was flying.
//! - A SLOW CLIMB: under the floor with a climb still on. The dial is sent
//!   down (`DESCEND_FPM`) and the answer confirmed within `RESPONSE_S`; a
//!   climb that has not come down by `RESPONSE_DROP_FPM` while the speed
//!   still falls is taken over, and under `DEEP_FLOOR_RATIO` of the floor at
//!   once.
//!
//! A take-over is the recovery's (`tools::recover::take_over`): the servos
//! out, the aircraft flown level on the column, the autopilot given it back
//! only once it is seen to hold.

/// The climb this far above the dial is not the dial's.
pub const DIVERGE_FPM: f64 = 2500.0;
/// ...held this long.
pub const DIVERGE_S: f64 = 3.0;
/// A climb, for the slow watch: a level-off in progress is not one.
pub const CLIMBING_FPM: f64 = 300.0;
/// How long a descent command has to show in the climb rate.
pub const RESPONSE_S: f64 = 4.0;
/// ...and by how much: a climb this much lower since the mark is answering.
pub const RESPONSE_DROP_FPM: f64 = 1000.0;
/// The speed lost since the mark that makes an unanswered climb a threat.
pub const DECAY_KTS: f64 = 3.0;
/// Under this share of the floor with a climb on, no waiting.
pub const DEEP_FLOOR_RATIO: f64 = 0.9;
/// The dial a slow climb is sent to.
pub const DESCEND_FPM: f64 = -1000.0;
/// The climb-rate estimate's weight on the newest sample (the circuit's own
/// `vs_est`), seeded from the first, never from zero.
const VS_ALPHA: f64 = 0.4;

/// Why the watch took the aircraft.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Why {
    /// The climb far above the dial and the speed going.
    NotFollowing,
    /// Under the floor, sent down, and still climbing.
    SlowClimb,
    /// Far under the floor with a climb on.
    DeepSlow,
    /// The autopilot's pitch moves nothing ([`AuthorityCheck`]): the pitch
    /// is flown on our own column, no autopilot given it back.
    NoAuthority,
}

impl Why {
    /// The blackbox word.
    pub fn word(self) -> &'static str {
        match self {
            Why::NotFollowing => "not_following",
            Why::SlowClimb => "slow_climb",
            Why::DeepSlow => "deep_slow",
            Why::NoAuthority => "no_pitch_authority",
        }
    }

    /// The spoken half, no dashes (TTS).
    pub fn line(self) -> &'static str {
        match self {
            Why::NotFollowing => "The autopilot isn't following the climb.",
            Why::SlowClimb | Why::DeepSlow => "We're climbing into a stall.",
            Why::NoAuthority => NO_PITCH_LINE,
        }
    }

    /// The recovery flies the pitch itself and never gives it back to the
    /// autopilot.
    pub fn ap_has_no_pitch(self) -> bool {
        self == Why::NoAuthority
    }
}

/// What the loop does this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Verdict {
    Flying,
    /// The dial is not being followed but the speed holds: said to the log
    /// once per episode, nothing else.
    NotFollowed,
    /// Send the dial down to [`DESCEND_FPM`] (or lower) and watch.
    Descend,
    /// Take the aircraft.
    TakeOver(Why),
}

#[derive(Debug, Clone, Copy)]
struct Mark {
    t: f64,
    vs: f64,
    ias: f64,
}

/// Fed every tick. A failed read is no measurement: it changes no clock.
#[derive(Debug, Default, Clone)]
pub struct ClimbWatch {
    prev_alt: Option<(f64, f64)>,
    vs: Option<f64>,
    diverge: Option<Mark>,
    slow: Option<Mark>,
    noted: bool,
}

impl ClimbWatch {
    /// The climb rate the watch judges on, fpm (`None` until two samples).
    pub fn vs_fpm(&self) -> Option<f64> {
        self.vs
    }

    /// `alt_ft` the TRUE altitude now (never the altimeter, which steps
    /// when its setting changes); `floor_kts` the configuration's floor
    /// (`None` = unknown, no slow watch); `dial_fpm` the VS the loop commands
    /// through the autopilot, `None` when the dial is not what flies (the
    /// autopilot's own altitude hold owns the level).
    pub fn step(
        &mut self,
        now_s: f64,
        alt_ft: f64,
        ias_kts: f64,
        floor_kts: Option<f64>,
        dial_fpm: Option<f64>,
    ) -> Verdict {
        if !(now_s.is_finite() && alt_ft.is_finite() && ias_kts.is_finite()) {
            return Verdict::Flying;
        }
        if let Some((a0, t0)) = self.prev_alt {
            let dt = now_s - t0;
            if (0.1..=2.0).contains(&dt) {
                let inst = (alt_ft - a0) / dt * 60.0;
                self.vs = Some(match self.vs {
                    Some(v) => v + VS_ALPHA * (inst - v),
                    None => inst,
                });
            } else if dt > 2.0 {
                // A gap (a pause, a stall of the loop): start again.
                self.vs = None;
            }
        }
        self.prev_alt = Some((alt_ft, now_s));
        let Some(vs) = self.vs else {
            return Verdict::Flying;
        };
        let here = Mark {
            t: now_s,
            vs,
            ias: ias_kts,
        };

        // Not following the dial.
        let mut verdict = Verdict::Flying;
        match dial_fpm.filter(|d| d.is_finite()) {
            Some(dial) if vs - dial > DIVERGE_FPM && vs > CLIMBING_FPM => {
                match self.diverge {
                    Some(m) if vs >= m.vs - RESPONSE_DROP_FPM => {
                        if now_s - m.t >= DIVERGE_S {
                            if ias_kts <= m.ias - DECAY_KTS {
                                verdict = Verdict::TakeOver(Why::NotFollowing);
                            } else if !self.noted {
                                self.noted = true;
                                verdict = Verdict::NotFollowed;
                            }
                        }
                    }
                    // Coming down: answering, a fresh mark.
                    _ => self.diverge = Some(here),
                }
            }
            _ => {
                self.diverge = None;
                self.noted = false;
            }
        }

        // A climb under the floor.
        match floor_kts.filter(|f| f.is_finite() && *f > 0.0) {
            Some(floor) if ias_kts < floor && vs > CLIMBING_FPM => {
                let deep = ias_kts < floor * DEEP_FLOOR_RATIO;
                let slow = match self.slow {
                    Some(m) if vs >= m.vs - RESPONSE_DROP_FPM => {
                        if now_s - m.t >= RESPONSE_S && ias_kts <= m.ias - DECAY_KTS {
                            Verdict::TakeOver(Why::SlowClimb)
                        } else {
                            Verdict::Descend
                        }
                    }
                    _ => {
                        self.slow = Some(here);
                        Verdict::Descend
                    }
                };
                let slow = if deep {
                    Verdict::TakeOver(Why::DeepSlow)
                } else {
                    slow
                };
                verdict = worst(verdict, slow);
            }
            _ => self.slow = None,
        }
        verdict
    }
}

fn rank(v: Verdict) -> u8 {
    match v {
        Verdict::Flying => 0,
        Verdict::NotFollowed => 1,
        Verdict::Descend => 2,
        Verdict::TakeOver(_) => 3,
    }
}

fn worst(a: Verdict, b: Verdict) -> Verdict {
    if rank(b) > rank(a) {
        b
    } else {
        a
    }
}

/// The dial for a verdict: a slow climb is sent down, else as commanded.
/// Pure.
pub fn dial_for(verdict: Verdict, vs_cmd: f64) -> f64 {
    match verdict {
        Verdict::Descend | Verdict::TakeOver(_) => vs_cmd.min(DESCEND_FPM),
        _ => vs_cmd,
    }
}

// ---- The autopilot's pitch AUTHORITY, checked at every engage (#181) -------
//
// 2026-09-29 from about t 1790648000: on the 737 and the A330, across aircraft
// reloads and app restarts, the autopilot's pitch servo moved nothing. Mode
// bits, dials and servos all read right, the roll channel worked, and our own
// `xcopilot/pitch` still moved the elevator. The 737's takeoff engage (sortie
// 1790649043) climbed AWAY from its dial (3,949 to 6,118 fpm in 4 s against
// 3,934 walking down); the A330's (1790651279) sat at 3,500 fpm for 40 s under
// a dial walked to -500; its approach (1790651449) sank a steady 640 fpm
// under a dial of 0. Nothing read the servo or the elevator.
//
// Judged from each vertical engage (the takeoff's, the circuit's and the
// follower's VS, autoland's confirmed servos, the enroute setup's), fed by
// whichever loop flies the dial, the servo and elevator from the monitor's
// reads ([`publish_telemetry`]). NO AUTHORITY, from [`AUTH_MIN_S`]:
// - AWAY: the climb still moving away from the dial ([`AWAY_RATE_FPM_S`] over
//   the last [`SLOPE_SPAN_S`]), [`AWAY_FPM`] from the engage and
//   [`AUTH_ERR_FPM`] off it (a working autopilot reverses a sink it inherited
//   inside 4 s: 1790646179).
// - LINKAGE: the servo moved [`SERVO_MOVE`] and the elevator never
//   [`ELEV_STILL_DEG`], the climb off the dial.
// At [`AUTH_MAX_S`], unless the elevator was seen following the servo:
// - STUCK: the climb less than [`STUCK_RATIO`] of the way to a dial that
//   asked [`ASK_MIN_FPM`], [`STUCK_ERR_FPM`] off it.
// - NOT HELD: a steady dial never within [`HOLD_ERR_FPM`] in the second half.
// Confirmed as soon as the climb is [`ANSWER_RATIO`] of the way to what the
// dial asked (a 737 answers a ramp by half in 12 to 14 s, an A330 a 3,700 fpm
// step by half in 10 s, 1790621184).

/// The first judgement after the engage.
pub const AUTH_MIN_S: f64 = 4.0;
/// The last: the verdict is in by now.
pub const AUTH_MAX_S: f64 = 15.0;
/// Moved this far from the engage's climb in the error's direction...
pub const AWAY_FPM: f64 = 600.0;
/// ...this far off the dial...
pub const AUTH_ERR_FPM: f64 = 1000.0;
/// ...and still moving away this fast.
pub const AWAY_RATE_FPM_S: f64 = 150.0;
/// The span the away rate is measured over.
pub const SLOPE_SPAN_S: f64 = 1.0;
/// A dial asking this much from the engage's climb is a question.
pub const ASK_MIN_FPM: f64 = 1000.0;
/// The share of the ask that confirms an answer.
pub const ANSWER_RATIO: f64 = 0.5;
/// Under this share of the ask at the end is no answer.
pub const STUCK_RATIO: f64 = 0.3;
/// ...when the climb is still this far off the dial.
pub const STUCK_ERR_FPM: f64 = 2000.0;
/// A steady dial never held within this in the second half.
pub const HOLD_ERR_FPM: f64 = 500.0;
/// A dial that moved less than this over the second half is steady.
pub const STEADY_DIAL_FPM: f64 = 200.0;
/// A second half held within this confirms a dial that asked little.
pub const HELD_FPM: f64 = 300.0;
/// The servo moved this much from the engage: it is commanding.
pub const SERVO_MOVE: f64 = 0.1;
/// The servo moved at least this much with the elevator: it is followed.
pub const SERVO_STIR: f64 = 0.05;
/// The elevator moved this much: it follows.
pub const ELEV_MOVE_DEG: f64 = 0.5;
/// The elevator never moved this much: it does not.
pub const ELEV_STILL_DEG: f64 = 0.3;
/// A gap in the feed this long ends the check without a verdict.
pub const AUTH_GAP_S: f64 = 3.0;
/// Telemetry older than this is no measurement.
pub const TELEMETRY_FRESH_S: f64 = 1.5;
const AUTH_ALPHA: f64 = 0.4;

/// What the crew says when the autopilot has no pitch. No dashes (TTS).
pub const NO_PITCH_LINE: &str = "The autopilot has no pitch control, I'm flying the pitch.";

/// The pitch channel as the monitor read it: the autopilot's servo output,
/// the elevator surface (degrees, the sim's sign: trailing edge down
/// positive) and the summed column. A failed read is `None`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PitchTelemetry {
    pub servo: Option<f64>,
    pub elev_deg: Option<f64>,
    pub total: Option<f64>,
    /// Our own pitch channel holds the elevator (the plugin's override):
    /// the surface is ours, not the servo's.
    pub ours: bool,
}

/// The authority verdict.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Authority {
    Pending,
    Confirmed(&'static str),
    NoAuthority(&'static str),
    /// Nothing to judge: the dial asked little and the feed ended or gapped.
    Inconclusive(&'static str),
}

impl Authority {
    fn is_final(self) -> bool {
        !matches!(self, Authority::Pending)
    }
}

/// Fed every tick after a vertical engage. A failed read is no measurement.
#[derive(Debug, Clone)]
pub struct AuthorityCheck {
    t0: f64,
    vs0: f64,
    vs: f64,
    last_t: f64,
    hist: Vec<(f64, f64)>,
    servo0: Option<f64>,
    elev0: Option<f64>,
    servo: Option<f64>,
    elev: Option<f64>,
    servo_dev: f64,
    elev_dev: f64,
    /// Second half: |err| min and max, dial min and max.
    late: Option<(f64, f64, f64, f64)>,
    dial: Option<f64>,
    /// Every dial since the arm, min and max: a VS between two of them is
    /// lagging a dial that moved, never running away from it.
    dials: Option<(f64, f64)>,
    done: Option<Authority>,
}

impl AuthorityCheck {
    /// Armed at `t` on the climb `vs` being flown.
    pub fn new(t: f64, vs: f64, tele: Option<PitchTelemetry>) -> Self {
        let mut c = AuthorityCheck {
            t0: t,
            vs0: vs,
            vs,
            last_t: t,
            hist: vec![(t, vs)],
            servo0: None,
            elev0: None,
            servo: None,
            elev: None,
            servo_dev: 0.0,
            elev_dev: 0.0,
            late: None,
            dial: None,
            dials: None,
            done: None,
        };
        c.note_telemetry(tele);
        c
    }

    fn note_telemetry(&mut self, tele: Option<PitchTelemetry>) {
        let Some(t) = tele else { return };
        // Under our own channel the elevator is ours: no baseline, no
        // movement. The takeoff held the column to its 300 ft handoff after
        // the engage armed this check, and the servo winding meanwhile
        // against an elevator we held read as "elevator not following"
        // (LEMG 30 737 1790698748, 2026-09-29: servo0 -0.054 elev0 8.73 taken
        // under the override; released, elevator 10.12 -> 8.65 with the
        // servo -0.46 -> -0.39).
        if t.ours {
            self.servo0 = None;
            self.elev0 = None;
            self.servo_dev = 0.0;
            self.elev_dev = 0.0;
            return;
        }
        if let Some(s) = t.servo.filter(|v| v.is_finite()) {
            let s0 = *self.servo0.get_or_insert(s);
            self.servo_dev = self.servo_dev.max((s - s0).abs());
            self.servo = Some(s);
        }
        if let Some(e) = t.elev_deg.filter(|v| v.is_finite()) {
            let e0 = *self.elev0.get_or_insert(e);
            self.elev_dev = self.elev_dev.max((e - e0).abs());
            self.elev = Some(e);
        }
    }

    /// The elevator seen following the servo.
    pub fn elevator_follows(&self) -> bool {
        self.servo_dev >= SERVO_STIR && self.elev_dev >= ELEV_MOVE_DEG
    }

    /// The RAW climb's trend over the last [`SLOPE_SPAN_S`]: the smoothed
    /// one lags a second and still read "away" from autopilots that had
    /// already reversed an inherited sink (1790590150, 1790598548).
    fn slope(&self, now: f64, vs_raw: f64) -> Option<f64> {
        let (t, v) = *self.hist.iter().rev().find(|(t, _)| now - t >= SLOPE_SPAN_S)?;
        let dt = now - t;
        (dt > 0.0).then(|| (vs_raw - v) / dt)
    }

    /// One tick: `vs` the climb flown, `dial` the VS the loop commands
    /// (`None` = the dial is not what flies this tick). Once final, the
    /// verdict stands.
    pub fn step(
        &mut self,
        t: f64,
        vs: Option<f64>,
        dial: Option<f64>,
        tele: Option<PitchTelemetry>,
    ) -> Authority {
        if let Some(v) = self.done {
            return v;
        }
        let v = self.judge(t, vs, dial, tele);
        if v.is_final() {
            self.done = Some(v);
        }
        v
    }

    fn judge(
        &mut self,
        t: f64,
        vs: Option<f64>,
        dial: Option<f64>,
        tele: Option<PitchTelemetry>,
    ) -> Authority {
        let Some(vs) = vs.filter(|v| v.is_finite()) else {
            return Authority::Pending;
        };
        if !t.is_finite() || t <= self.last_t {
            return Authority::Pending;
        }
        if t - self.last_t > AUTH_GAP_S {
            return Authority::Inconclusive("gap");
        }
        self.last_t = t;
        // Under our own column the climb is ours too: the check starts again
        // from the release, on every branch. The takeoff flew the Citation's
        // climb on the column for the check's first 5.5 s and "vs away from
        // the dial" (2,789 -> 3,825 against 1,677) handed a healthy autopilot
        // to a hand-flown recovery (LEMG 1790734216, 2026-09-30).
        if tele.is_some_and(|p| p.ours) {
            self.t0 = t;
            self.vs0 = vs;
            self.vs = vs;
            self.hist.clear();
            self.hist.push((t, vs));
            self.late = None;
            self.dials = None;
            self.note_telemetry(tele);
            return Authority::Pending;
        }
        self.vs += AUTH_ALPHA * (vs - self.vs);
        self.hist.push((t, vs));
        self.hist.retain(|(ht, _)| t - ht <= SLOPE_SPAN_S + 2.0);
        self.note_telemetry(tele);
        let e = t - self.t0;
        let Some(dial) = dial.filter(|d| d.is_finite()) else {
            return if e >= AUTH_MAX_S + AUTH_GAP_S {
                Authority::Inconclusive("no_dial")
            } else {
                Authority::Pending
            };
        };
        self.dial = Some(dial);
        let d = self.dials.get_or_insert((dial, dial));
        *d = (d.0.min(dial), d.1.max(dial));
        let (dial_lo, dial_hi) = *d;
        let ask = dial - self.vs0;
        let answer = self.vs - self.vs0;
        let err = self.vs - dial;
        let ratio = if ask.abs() >= ASK_MIN_FPM { answer / ask } else { f64::NAN };
        if e >= AUTH_MAX_S / 2.0 {
            let l = self.late.get_or_insert((err.abs(), err.abs(), dial, dial));
            *l = (l.0.min(err.abs()), l.1.max(err.abs()), l.2.min(dial), l.3.max(dial));
        }
        if e < AUTH_MIN_S {
            return Authority::Pending;
        }
        let away_rate = self.slope(t, vs).map(|s| s * err.signum());
        // Inside the dials asked since the arm the VS is still answering
        // an earlier one: the follower eased -1,829 to -522 through a check
        // and the 737's -1,588 on its way round was "away" (EGKK-EGBB
        // 1790671979, the elevator following the servo; #181).
        let lagging = vs >= dial_lo - AUTH_ERR_FPM / 2.0 && vs <= dial_hi + AUTH_ERR_FPM / 2.0;
        if err.abs() >= AUTH_ERR_FPM
            && !lagging
            && answer.signum() == err.signum()
            && answer.abs() >= AWAY_FPM
            && away_rate.is_some_and(|r| r >= AWAY_RATE_FPM_S)
        {
            return Authority::NoAuthority("vs_away_from_dial");
        }
        if self.servo0.is_some()
            && self.elev0.is_some()
            && self.servo_dev >= SERVO_MOVE
            && self.elev_dev < ELEV_STILL_DEG
            && err.abs() >= HOLD_ERR_FPM
        {
            return Authority::NoAuthority("elevator_not_following_servo");
        }
        if ratio >= ANSWER_RATIO {
            return Authority::Confirmed("vs_answers_dial");
        }
        if e < AUTH_MAX_S {
            return Authority::Pending;
        }
        let follows = self.elevator_follows();
        if !follows && ratio < STUCK_RATIO && err.abs() >= STUCK_ERR_FPM {
            return Authority::NoAuthority("vs_not_answering");
        }
        if let Some((err_min, err_max, dmin, dmax)) = self.late {
            if !follows && dmax - dmin <= STEADY_DIAL_FPM && err_min >= HOLD_ERR_FPM {
                return Authority::NoAuthority("dial_not_held");
            }
            if err_max <= HELD_FPM {
                return Authority::Confirmed("holds_dial");
            }
        }
        if follows {
            Authority::Confirmed("elevator_follows_servo")
        } else if ratio.is_nan() {
            Authority::Inconclusive("nothing_asked")
        } else {
            Authority::Confirmed("slow_answer")
        }
    }

    /// The numbers behind a verdict, for the blackbox.
    pub fn json(&self) -> serde_json::Value {
        let r = |v: f64| (v * 10.0).round() / 10.0;
        let r3 = |v: Option<f64>| v.map(|x| (x * 1000.0).round() / 1000.0);
        serde_json::json!({
            "after_s": r(self.last_t - self.t0), "vs0": r(self.vs0), "vs": r(self.vs),
            "dial": self.dial.map(r), "servo0": r3(self.servo0), "servo": r3(self.servo),
            "servo_dev": r3(Some(self.servo_dev)), "elev0": r3(self.elev0),
            "elev": r3(self.elev), "elev_dev": r3(Some(self.elev_dev)),
            "late_err": self.late.map(|l| [r(l.0), r(l.1)]),
        })
    }
}

fn now_s() -> f64 {
    static EPOCH: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    EPOCH.get_or_init(std::time::Instant::now).elapsed().as_secs_f64()
}

static TELEMETRY: std::sync::Mutex<Option<(f64, PitchTelemetry)>> = std::sync::Mutex::new(None);

/// The monitor's latest read of the pitch channel.
pub fn publish_telemetry(t: PitchTelemetry) {
    *TELEMETRY.lock().unwrap_or_else(|e| e.into_inner()) = Some((now_s(), t));
}

/// The latest pitch read, if fresh.
pub fn telemetry() -> Option<PitchTelemetry> {
    let g = TELEMETRY.lock().unwrap_or_else(|e| e.into_inner());
    g.filter(|(t, _)| now_s() - t <= TELEMETRY_FRESH_S).map(|(_, p)| p)
}

static SIM_SPEED: std::sync::Mutex<Option<(f64, f64)>> = std::sync::Mutex::new(None);

/// The monitor's latest read of `sim/time/sim_speed` (`None`: unread).
pub fn publish_sim_speed(v: Option<f64>) {
    let v = v.filter(|x| x.is_finite());
    *SIM_SPEED.lock().unwrap_or_else(|e| e.into_inner()) = v.map(|x| (now_s(), x));
}

/// Fast forward: the check's clock is the wall's, so at 4x it judges four
/// times the flight in the window, and the autopilot's own lag reads as
/// running away (EGKK-EGBB 1790675237: a top of descent at 4x overshot to
/// -4,290 fpm on a -1,822 dial and was stood down as no authority, #181).
/// A fresh read over this is no measurement.
const FAST_FORWARD_OVER: f64 = 1.2;

fn fast_forward() -> bool {
    sim_speed_now().is_some_and(|v| v > FAST_FORWARD_OVER)
}

/// The sim's time acceleration as the monitor last read it, if fresh
/// (our traffic freezes above 1x: `traffic_link`).
pub fn sim_speed_now() -> Option<f64> {
    let g = SIM_SPEED.lock().unwrap_or_else(|e| e.into_inner());
    g.filter(|(t, _)| now_s() - t <= TELEMETRY_FRESH_S)
        .map(|(_, v)| v)
}

struct Armed {
    source: &'static str,
    check: AuthorityCheck,
}

/// ONE check, on purpose (refactor audit 2026-10-04 asked whether five
/// arming loops should each own one): the check is about the ONE
/// autopilot's elevator after its latest vertical engage, not about a loop.
/// The loops that arm it fly the dial one after another, never side by side
/// (the takeoff hands the climb to the circuit or the follower, either hands
/// the final to autoland, the enroute setup's settle ends before any of
/// them), and the check is HANDED DOWN with the dial: the takeoff's engage
/// is judged by whoever takes its climb (`circuit`, `procedure` feed a check
/// `autotakeoff` armed). A per-arm token would cut exactly that hand-down. A
/// newer engage replacing the older check is the right reading of the
/// autopilot (its newest mode is the one flying), and every loop that arms
/// it ends before the next one's engage, or is stood down with the check
/// (`flight_state::stand_down` disarms it). Pinned by
/// `one_authority_check_for_the_one_autopilot`.
static ARMED: std::sync::Mutex<Option<Armed>> = std::sync::Mutex::new(None);

/// A vertical mode was engaged by `source` on the climb `vs`: the check
/// starts again (blackbox `ap_pitch_authority` `armed`).
pub fn arm_authority(source: &'static str, vs: f64, dial: f64) {
    if !vs.is_finite() {
        return;
    }
    let check = AuthorityCheck::new(now_s(), vs, telemetry());
    crate::blackbox::record(
        "ap_pitch_authority",
        serde_json::json!({"event": "armed", "source": source, "vs": vs, "dial": dial,
                           "telemetry": telemetry().is_some()}),
    );
    *ARMED.lock().unwrap_or_else(|e| e.into_inner()) = Some(Armed { source, check });
}

/// Whoever flies the dial feeds the armed check. `Some` once, with the
/// final verdict (blackboxed `ap_pitch_authority` `verdict` with the
/// numbers); `None` while pending or with nothing armed.
pub fn feed_authority(vs: Option<f64>, dial: Option<f64>) -> Option<Authority> {
    let mut g = ARMED.lock().unwrap_or_else(|e| e.into_inner());
    let armed = g.as_mut()?;
    let v = if fast_forward() {
        Authority::Inconclusive("fast_forward")
    } else {
        armed.check.step(now_s(), vs, dial, telemetry())
    };
    if !v.is_final() {
        return None;
    }
    let (word, why) = match v {
        Authority::Confirmed(w) => ("confirmed", w),
        Authority::NoAuthority(w) => ("no_authority", w),
        Authority::Inconclusive(w) => ("inconclusive", w),
        Authority::Pending => ("pending", ""),
    };
    crate::blackbox::record(
        "ap_pitch_authority",
        serde_json::json!({"event": "verdict", "source": armed.source, "verdict": word,
                           "why": why, "numbers": armed.check.json()}),
    );
    *g = None;
    Some(v)
}

/// Nothing is flying the dial any more (a stand-down, the arrest).
pub fn disarm_authority() {
    *ARMED.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// The height a hand-flown take-over holds after a takeoff engage: the
/// selected altitude when it is a climb still ahead within reason, else a
/// thousand feet over where we are. Pure.
pub fn takeover_hold_ft(dial_ft: Option<f64>, alt_ft: Option<f64>) -> Option<f64> {
    let alt = alt_ft.filter(|a| a.is_finite())?;
    let above = dial_ft
        .filter(|d| d.is_finite())
        .filter(|d| *d >= alt + 500.0 && *d <= alt + 6_000.0);
    Some(above.unwrap_or(((alt + 1_000.0) / 100.0).round() * 100.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    /// The circuit's leg ticks of a trimmed sortie: (t, alt_msl, ias,
    /// climb_floor, cmd_vs, ap_level).
    fn legs(src: &str) -> Vec<(f64, f64, f64, f64, f64, bool)> {
        src.lines()
            .filter_map(|l| serde_json::from_str::<Value>(l).ok())
            .filter(|r| r["kind"] == "circuit" && r["event"] == "leg")
            .filter_map(|r| {
                Some((
                    r["t"].as_f64()?,
                    r["alt_msl"].as_f64()?,
                    r["ias"].as_f64()?,
                    r["climb_floor"].as_f64()?,
                    r["cmd_vs"].as_f64()?,
                    r["ap_level"].as_bool().unwrap_or(false),
                ))
            })
            .collect()
    }

    /// Replays a sortie's circuit ticks through the watch: the first
    /// take-over (seconds into the circuit, ias, agl-free alt) if any, and
    /// every verdict seen.
    fn replay(src: &str) -> (Option<(f64, Why, f64, f64)>, Vec<Verdict>) {
        let ticks = legs(src);
        let t0 = ticks.first().map_or(0.0, |t| t.0);
        let mut w = ClimbWatch::default();
        let mut first = None;
        let mut seen = vec![];
        for (t, alt, ias, floor, cmd, level) in ticks {
            let v = w.step(t, alt, ias, Some(floor), (!level).then_some(cmd));
            if let (Verdict::TakeOver(why), None) = (v, first) {
                first = Some((t - t0, why, ias, alt));
            }
            seen.push(v);
        }
        (first, seen)
    }

    const STALL: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/sorties/1790649043-full_auto.trimmed.jsonl"));
    const FLOWN: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/sorties/1790631752-full_auto.trimmed.jsonl"));

    /// #181: the 737 whose autopilot did not follow the dial. As flown the
    /// circuit rode it from 166 kt to 73 and 2,643 ft AGL; the watch takes it
    /// 3.5 s into the pattern at 156 kt and 1,050 ft MSL, before the climb
    /// floor (149.5) is reached, on the climb 5,000 fpm over its dial.
    #[test]
    fn the_lemg_stall_is_taken_before_the_floor() {
        let (first, _) = replay(STALL);
        let (at_s, why, ias, alt) = first.expect("the watch must take the aircraft");
        assert_eq!(why, Why::NotFollowing, "{first:?}");
        assert!(at_s < 6.0, "taken {at_s:.1} s into the circuit");
        assert!(ias > 149.5, "taken at {ias:.1} kt, the floor is 149.5");
        assert!(alt < 1_400.0, "taken at {alt:.0} ft MSL");
    }

    /// The same case flown cleanly on 602513e (sortie 1790631752, PASS):
    /// the whole circuit, climb, level-off, downwind, base and final, never
    /// trips the watch and never sends a dial down.
    #[test]
    fn a_circuit_the_autopilot_flew_is_never_taken() {
        let (first, seen) = replay(FLOWN);
        assert!(first.is_none(), "{first:?}");
        assert!(
            !seen.iter().any(|v| matches!(v, Verdict::Descend)),
            "a descent was forced on a flying autopilot"
        );
        assert!(
            seen.len() > 400,
            "the whole circuit is replayed: {}",
            seen.len()
        );
    }

    /// A slow autopilot at a healthy speed is noted, never taken (an A330
    /// took 10 s to bring 3,700 fpm to a 0 dial at 180 kt and rising,
    /// sortie 1790621184).
    #[test]
    fn a_slow_autopilot_at_a_good_speed_is_only_noted() {
        let mut w = ClimbWatch::default();
        let mut alt = 600.0;
        let mut out = vec![];
        for i in 0..20 {
            let t = i as f64 * 0.6;
            alt += 3_500.0 / 60.0 * 0.6;
            out.push(w.step(t, alt, 180.0 + i as f64 * 0.2, Some(150.0), Some(0.0)));
        }
        assert!(out.contains(&Verdict::NotFollowed));
        assert!(!out
            .iter()
            .any(|v| matches!(v, Verdict::TakeOver(_) | Verdict::Descend)));
    }

    /// Under the floor with a climb on: sent down at once, taken when the
    /// climb does not answer in RESPONSE_S while the speed still falls,
    /// released when it does answer.
    #[test]
    fn a_slow_climb_is_sent_down_then_taken_or_released() {
        // Unanswered: 2,000 fpm, losing a knot a second, from 148 under 150.
        let mut w = ClimbWatch::default();
        let mut alt = 1_000.0;
        let mut first_take = None;
        for i in 0..20 {
            let t = i as f64 * 0.5;
            alt += 2_000.0 / 60.0 * 0.5;
            let v = w.step(t, alt, 148.0 - t, Some(150.0), Some(0.0));
            if i >= 1 {
                assert!(
                    matches!(v, Verdict::Descend | Verdict::TakeOver(_)),
                    "{i}: {v:?}"
                );
            }
            if matches!(v, Verdict::TakeOver(Why::SlowClimb)) && first_take.is_none() {
                first_take = Some(t);
            }
        }
        let at = first_take.expect("taken");
        assert!(
            (RESPONSE_S..RESPONSE_S + 1.1).contains(&at),
            "taken at {at}"
        );

        // Answered: the climb comes down 600 fpm a second.
        let mut w = ClimbWatch::default();
        let mut alt = 1_000.0;
        let mut vs = 2_000.0;
        for i in 0..20 {
            let t = i as f64 * 0.5;
            alt += vs / 60.0 * 0.5;
            vs -= 300.0;
            let v = w.step(t, alt, 148.0 - 0.3 * t, Some(150.0), Some(-500.0));
            assert!(!matches!(v, Verdict::TakeOver(_)), "{i}: {v:?}");
        }
    }

    /// Far under the floor with a climb on is taken at once, and a descent
    /// or level flight under the floor (an approach) is left alone.
    #[test]
    fn deep_under_the_floor_climbing_is_taken_at_once() {
        let mut w = ClimbWatch::default();
        assert_eq!(
            w.step(0.0, 1_000.0, 130.0, Some(150.0), Some(-500.0)),
            Verdict::Flying
        );
        assert_eq!(
            w.step(0.5, 1_010.0, 130.0, Some(150.0), Some(-500.0)),
            Verdict::TakeOver(Why::DeepSlow)
        );
        let mut w = ClimbWatch::default();
        for i in 0..20 {
            let t = i as f64 * 0.5;
            let v = w.step(
                t,
                1_500.0 - 6.0 * i as f64,
                120.0,
                Some(150.0),
                Some(-700.0),
            );
            assert_eq!(
                v,
                Verdict::Flying,
                "a descent under the floor is an approach"
            );
        }
    }

    /// A failed read or a gap is no measurement: nothing is judged across it.
    #[test]
    fn a_gap_is_no_measurement() {
        let mut w = ClimbWatch::default();
        assert_eq!(
            w.step(0.0, 1_000.0, 130.0, Some(150.0), None),
            Verdict::Flying
        );
        assert_eq!(
            w.step(0.5, f64::NAN, 130.0, Some(150.0), None),
            Verdict::Flying
        );
        // 10 s later, 3,000 ft higher: not a 18,000 fpm climb.
        assert_eq!(
            w.step(10.0, 4_000.0, 130.0, Some(150.0), None),
            Verdict::Flying
        );
        assert_eq!(w.vs_fpm(), None);
    }

    #[test]
    fn the_dial_is_only_ever_sent_down() {
        assert_eq!(dial_for(Verdict::Descend, 2_000.0), DESCEND_FPM);
        assert_eq!(dial_for(Verdict::Descend, -1_500.0), -1_500.0);
        assert_eq!(dial_for(Verdict::NotFollowed, 2_000.0), 2_000.0);
        assert_eq!(dial_for(Verdict::Flying, 800.0), 800.0);
    }

    // ---- the authority check, replayed ------------------------------------

    const A330_CLIMB: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/sorties/1790651279-full_auto.trimmed.jsonl"));
    const A330_FLOWN: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/sorties/1790621184-full_auto.trimmed.jsonl"));
    const A330_FINAL: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/sorties/1790651449-approach.trimmed.jsonl"));
    const FINAL_FLOWN: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/sorties/1790646179-approach.trimmed.jsonl"));

    /// Replays a sortie's engages the way the loops feed the check: armed at
    /// the takeoff's `ap_engaged` on the climb flown, fed its `climb` ticks
    /// (`vs`, `cmd_vs`), then the circuit's legs (the climb from `alt_msl` as
    /// the circuit's own watch estimates it, the dial `cmd_vs`, 0 while the
    /// autopilot's capture holds the level); armed at autoland's
    /// `ap_engaged` on the next tick's `vvi`, fed `vvi` and `vs_dial` until
    /// the arrest. Every final verdict, with its seconds after the engage.
    fn authority_replay(src: &str) -> Vec<(Authority, f64)> {
        let mut out = vec![];
        let mut check: Option<AuthorityCheck> = None;
        let mut t0 = 0.0;
        let mut prev_alt: Option<(f64, f64)> = None;
        let mut vs_est: Option<f64> = None;
        let mut autoland_owed = false;
        let mut finish = |c: &mut Option<AuthorityCheck>, v: Authority, t: f64, t0: f64| {
            if v != Authority::Pending {
                out.push((v, t - t0));
                *c = None;
            }
        };
        for r in src.lines().filter_map(|l| serde_json::from_str::<Value>(l).ok()) {
            let (kind, event, t) = (r["kind"].as_str(), r["event"].as_str(), r["t"].as_f64());
            let Some(t) = t else { continue };
            match (kind, event) {
                (Some("autotakeoff"), Some("ap_engaged")) => {
                    check = r["vs"].as_f64().map(|v| AuthorityCheck::new(t, v, None));
                    t0 = t;
                }
                (Some("autotakeoff"), Some("climb")) => {
                    if let Some(c) = check.as_mut() {
                        let v = c.step(t, r["vs"].as_f64(), r["cmd_vs"].as_f64(), None);
                        finish(&mut check, v, t, t0);
                    }
                }
                (Some("circuit"), Some("leg")) => {
                    if let Some(a) = r["alt_msl"].as_f64() {
                        if let Some((a0, ta)) = prev_alt {
                            let dt = t - ta;
                            if (0.1..=2.0).contains(&dt) {
                                let inst = (a - a0) / dt * 60.0;
                                vs_est = Some(vs_est.map_or(inst, |v| v + 0.4 * (inst - v)));
                            } else if dt > 2.0 {
                                vs_est = None;
                            }
                        }
                        prev_alt = Some((a, t));
                    }
                    if let Some(c) = check.as_mut() {
                        let level = r["ap_level"].as_bool().unwrap_or(false);
                        let dial = if level { Some(0.0) } else { r["cmd_vs"].as_f64() };
                        let v = c.step(t, vs_est, dial, None);
                        finish(&mut check, v, t, t0);
                    }
                }
                (Some("autoland"), Some("ap_engaged")) => {
                    check = None;
                    autoland_owed = true;
                }
                (Some("autoland"), None) => {
                    let vvi = r["vvi"].as_f64();
                    if r["arresting"].as_bool() == Some(true) {
                        check = None;
                        continue;
                    }
                    if autoland_owed {
                        if let Some(v) = vvi {
                            check = Some(AuthorityCheck::new(t, v, None));
                            t0 = t;
                            autoland_owed = false;
                        }
                        continue;
                    }
                    if let Some(c) = check.as_mut() {
                        let v = c.step(t, vvi, r["vs_dial"].as_f64(), None);
                        finish(&mut check, v, t, t0);
                    }
                }
                _ => {}
            }
        }
        out
    }

    fn no_authority(v: &[(Authority, f64)]) -> Option<(&'static str, f64)> {
        v.iter().find_map(|(a, e)| match a {
            Authority::NoAuthority(w) => Some((*w, *e)),
            _ => None,
        })
    }

    /// EGKK-EGBB 737 (1790671979, t 1790673833.4): the follower's descent
    /// dial -1,829 eased to -522 inside the check while the 737 was still
    /// steepening toward the first; the elevator followed the servo. That
    /// is a lag, never "away" (it stood the flight down as no authority).
    #[test]
    fn a_vs_lagging_an_eased_dial_is_not_no_authority() {
        let tele = |servo: f64, elev: f64| {
            Some(PitchTelemetry { servo: Some(servo), elev_deg: Some(elev), ..Default::default() })
        };
        let mut c = AuthorityCheck::new(0.0, -511.0, tele(0.039, 0.69));
        let mut last = Authority::Pending;
        for i in 1..=52 {
            let t = i as f64 * 0.2;
            let vs = (-511.0 - 150.0 * t).max(-1_600.0);
            let dial = if t < 4.0 { -1_829.0 } else { (-1_829.0 + 330.0 * (t - 4.0)).min(-522.0) };
            last = c.step(t, Some(vs), Some(dial), tele(0.039 - 0.017 * t.min(10.0), 0.69 - 0.16 * t.min(10.0)));
            assert!(!matches!(last, Authority::NoAuthority(_)), "t {t}: {last:?}");
        }
        assert!(matches!(last, Authority::Confirmed(_) | Authority::Pending), "{last:?}");
    }

    /// #181, the 737 (1790649043): the climb went AWAY from its dial after
    /// the takeoff's engage. Found 4 to 6 s after it, before the climb watch
    /// took it (5.4 s after the engage) and 17 s before the speed reached
    /// the 73 kt it was handed back at.
    #[test]
    fn the_737_whose_servo_moved_nothing_is_found_at_the_engage() {
        let v = authority_replay(STALL);
        let (why, after) = no_authority(&v).expect("no authority");
        assert_eq!(why, "vs_away_from_dial");
        assert!((AUTH_MIN_S..5.4).contains(&after), "found {after:.1} s after the engage");
    }

    /// #181, the A330's takeoff (1790651279): 3,500 fpm for 40 s under a
    /// dial walked to -500, the speed rising, so the climb watch only took it
    /// at 42.9 s. The check has it at the end of its window.
    #[test]
    fn the_a330_climb_that_never_answered_is_found_at_fifteen_seconds() {
        let v = authority_replay(A330_CLIMB);
        let (why, after) = no_authority(&v).expect("no authority");
        assert_eq!(why, "vs_not_answering");
        assert!(after < AUTH_MAX_S + 1.0, "found {after:.1} s after the engage");
    }

    /// #181, the A330's approach after it (1790651449): a steady 640 fpm
    /// sink under a dial of 0 for 100 s.
    #[test]
    fn the_a330_final_that_never_held_its_dial_is_found() {
        let v = authority_replay(A330_FINAL);
        let (why, after) = no_authority(&v).expect("no authority");
        assert_eq!(why, "dial_not_held");
        assert!(after < AUTH_MAX_S + 1.0, "found {after:.1} s after the engage");
    }

    /// The same sorties flown by autopilots that had the elevator: the
    /// 737's PASS circuit (1790631752), the A330's circuit before the fault
    /// (1790621184, a 3,700 fpm step answered in 10 s) and an approach whose
    /// autopilot inherited a sink and reversed it (1790646179). Never
    /// taken, each confirmed.
    #[test]
    fn autopilots_that_had_the_elevator_are_never_taken() {
        for (name, src) in [
            ("737 circuit", FLOWN),
            ("A330 circuit", A330_FLOWN),
            ("approach", FINAL_FLOWN),
        ] {
            let v = authority_replay(src);
            assert!(no_authority(&v).is_none(), "{name}: {v:?}");
            assert!(
                v.iter().any(|(a, _)| matches!(a, Authority::Confirmed(_))),
                "{name}: {v:?}"
            );
        }
    }

    fn tele(servo: f64, elev: f64) -> Option<PitchTelemetry> {
        Some(PitchTelemetry {
            servo: Some(servo),
            elev_deg: Some(elev),
            total: None,
            ours: false,
        })
    }

    /// The servo commanding and the elevator not moving is the fault itself:
    /// taken at the first judgement.
    #[test]
    fn a_servo_the_elevator_does_not_follow_is_no_authority() {
        let mut c = AuthorityCheck::new(0.0, 0.0, tele(0.0, 0.0));
        let mut out = Authority::Pending;
        for i in 1..=12 {
            let t = i as f64 * 0.5;
            out = c.step(t, Some(-700.0), Some(0.0), tele(0.02 * i as f64, 0.05));
            if out != Authority::Pending {
                assert!(t >= AUTH_MIN_S, "{t}");
                break;
            }
        }
        assert_eq!(out, Authority::NoAuthority("elevator_not_following_servo"));
        // Followed by the elevator, a slow answer is the autopilot's.
        let mut c = AuthorityCheck::new(0.0, 3_000.0, tele(0.0, 0.0));
        let mut out = Authority::Pending;
        for i in 1..=40 {
            let t = i as f64 * 0.5;
            let s = (0.02 * i as f64).min(0.3);
            out = c.step(t, Some(3_000.0), Some(0.0), tele(s, -8.0 * s));
            if out != Authority::Pending {
                break;
            }
        }
        assert_eq!(out, Authority::Confirmed("elevator_follows_servo"));
    }

    /// The column held by our own channel is no evidence (LEMG 30 737
    /// 1790698748): a servo winding against an elevator the takeoff still
    /// held until the handoff read as no authority. Baselined after the
    /// release, the elevator follows.
    #[test]
    fn our_own_column_is_no_evidence_against_the_servo() {
        let ours = |servo: f64, elev: f64| {
            Some(PitchTelemetry { servo: Some(servo), elev_deg: Some(elev), total: None, ours: true })
        };
        let mut c = AuthorityCheck::new(0.0, 3_900.0, ours(-0.054, 8.73));
        let mut out = Authority::Pending;
        for i in 1..=5 {
            let t = i as f64 * 0.5;
            out = c.step(t, Some(3_900.0), Some(3_850.0), ours(-0.054 - 0.08 * i as f64, 8.73));
        }
        assert_eq!(out, Authority::Pending, "nothing judged under our column");
        for i in 6..=40 {
            let t = i as f64 * 0.5;
            let s = -0.46 + 0.01 * (i - 5) as f64;
            let e = 10.12 - 4.0 * (s + 0.46) / 0.07;
            out = c.step(t, Some(3_900.0), Some(3_400.0), tele(s.min(-0.2), e.max(6.0)));
            if out != Authority::Pending {
                break;
            }
        }
        assert_ne!(out, Authority::NoAuthority("elevator_not_following_servo"));
    }

    /// The climb flown under our own column is not the autopilot's: a VS
    /// running away from the dial while the takeoff holds the column is no
    /// verdict, and the check starts again at the release (LEMG Citation
    /// 1790734216: 2,789 -> 3,825 fpm against a 1,677 dial in 5.5 s).
    #[test]
    fn a_climb_under_our_own_column_is_no_verdict() {
        let ours = Some(PitchTelemetry { servo: Some(-0.041), elev_deg: Some(-0.97), total: None, ours: true });
        let mut c = AuthorityCheck::new(0.0, 2_789.0, ours);
        for i in 1..=24 {
            let t = i as f64 * 0.5;
            let v = c.step(t, Some(2_789.0 + 90.0 * i as f64), Some(1_677.0), ours);
            assert_eq!(v, Authority::Pending, "t {t}");
        }
    }

    /// A gap in the feed (a pause, a loop that stopped) is no verdict; a
    /// failed read changes nothing; a final verdict stands.
    #[test]
    fn a_gap_ends_the_check_without_a_verdict() {
        let mut c = AuthorityCheck::new(0.0, 0.0, None);
        assert_eq!(c.step(1.0, None, Some(0.0), None), Authority::Pending);
        assert_eq!(c.step(0.5, Some(0.0), Some(0.0), None), Authority::Pending);
        assert_eq!(c.step(1.0, Some(0.0), Some(0.0), None), Authority::Pending);
        assert_eq!(c.step(9.0, Some(0.0), Some(0.0), None), Authority::Inconclusive("gap"));
        assert_eq!(c.step(9.5, Some(5_000.0), Some(0.0), None), Authority::Inconclusive("gap"));
    }

    /// Nothing armed, nothing to feed; a disarm ends it.
    #[test]
    fn the_armed_check_is_fed_only_while_armed() {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        disarm_authority();
        assert_eq!(feed_authority(Some(1_000.0), Some(0.0)), None);
        arm_authority("test", 1_000.0, 0.0);
        assert_eq!(feed_authority(Some(1_000.0), Some(0.0)), None);
        disarm_authority();
        assert_eq!(feed_authority(Some(9_000.0), Some(0.0)), None);
    }

    /// The take-over's height after a takeoff engage: the selected level
    /// when it is a climb ahead, else a thousand feet up.
    #[test]
    fn a_takeover_after_the_engage_climbs_to_the_selected_level() {
        assert_eq!(takeover_hold_ft(Some(3_000.0), Some(700.0)), Some(3_000.0));
        assert_eq!(takeover_hold_ft(Some(100.0), Some(700.0)), Some(1_700.0));
        assert_eq!(takeover_hold_ft(Some(37_000.0), Some(700.0)), Some(1_700.0));
        assert_eq!(takeover_hold_ft(None, Some(740.0)), Some(1_700.0));
        assert_eq!(takeover_hold_ft(Some(3_000.0), None), None);
    }
}

#[cfg(test)]
mod one_check_tests {
    /// The authority check is one slot for the one autopilot: the loops that
    /// arm it are the known five, flying the dial in turn, and only a
    /// stand-down and autoland's arrest disarm it. A new arming loop must be
    /// argued for here (see `ARMED`).
    #[test]
    fn one_authority_check_for_the_one_autopilot() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut sources = Vec::new();
        let mut disarms = Vec::new();
        let arm = "climb_watch::arm_authority(\"";
        for f in crate::test_support::rust_files(&root) {
            if crate::test_support::is_test_file(&f) || f.ends_with("climb_watch.rs") {
                continue;
            }
            let text = std::fs::read_to_string(&f).unwrap();
            let rel = f.strip_prefix(&root).unwrap().display().to_string();
            for (i, _) in text.match_indices(arm) {
                let rest = &text[i + arm.len()..];
                sources.push(rest[..rest.find('"').unwrap()].to_string());
            }
            if text.contains("climb_watch::disarm_authority()") {
                disarms.push(format!("{}.rs", crate::test_support::namespace_of(std::path::Path::new(&rel))));
            }
        }
        sources.sort();
        sources.dedup();
        assert_eq!(
            sources,
            ["autoland", "autotakeoff", "circuit", "procedure", "setup_enroute"]
        );
        disarms.sort();
        assert_eq!(disarms, ["autoland.rs", "flight_state.rs"]);
    }
}
