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
}

impl Why {
    /// The blackbox word.
    pub fn word(self) -> &'static str {
        match self {
            Why::NotFollowing => "not_following",
            Why::SlowClimb => "slow_climb",
            Why::DeepSlow => "deep_slow",
        }
    }

    /// The spoken half, no dashes (TTS).
    pub fn line(self) -> &'static str {
        match self {
            Why::NotFollowing => "The autopilot isn't following the climb.",
            Why::SlowClimb | Why::DeepSlow => "We're climbing into a stall.",
        }
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

    /// `alt_ft` the altitude now; `floor_kts` the configuration's floor
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

    const STALL: &str = include_str!("../bench/sorties/1790649043-full_auto.trimmed.jsonl");
    const FLOWN: &str = include_str!("../bench/sorties/1790631752-full_auto.trimmed.jsonl");

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
}
