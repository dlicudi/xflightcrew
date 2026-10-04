//! The heading bug: the one turn-safe steer and the one way to SET it
//! (refactor audit 2026-10-04, `docs/tasks/refactor-audit/audit-duplicates.md`
//! #6). Two rules, both from flights:
//!
//! - **The sim's heading mode never reverses a turn in progress**: a bug set
//!   on the far side of a bank goes the long way round. [`bug_toward`] (the
//!   circuit's per-tick steer) and [`HeadingHold`] (the follower's set-once
//!   steer) share one entry lead ([`TURN_ENTRY_LEAD_DEG`], 25 degrees, flown
//!   on the circuit at ETND 2026-09-23 and adopted by the follower for #70,
//!   EGBB-EGTF 2026-10-01), one in-turn bank ([`TURN_IN_PROGRESS_BANK_DEG`]),
//!   one smallest reversal ([`REVERSAL_MIN_DEG`]: the follower stages every
//!   turn past it, which with the 25 degree cap is the same bug for turns
//!   under 25) and one wings-level hysteresis ([`LEVEL_BANK_DEG`]), through
//!   one predicate ([`banked_against`]).
//! - **The knob is set, not rewritten** ([`BugHold`], issue #93): once per
//!   leg or phase, steering of [`BUG_STEER_DEG`] or more at once, corrections
//!   past a deadband. The follower's [`HeadingHold`] is the same idea with its
//!   own wings-level, 30 s correction.
//!
//! Before this module the circuit's door, the follower's copy and the
//! circuit's knob lived in two files, and autoland's pattern, reversal and
//! join legs and the helicopter route set the bug with no reversal guard.
//! Pure.

/// Move the heading bug toward `target` by at most `max_step` degrees, the
/// SHORT way around, returning the new dial value. A single 90-degree write
/// across the 0/360 seam (271 -> 2 for the right pattern's crosswind) reads
/// to the sim's AP as a 270-degree counterclockwise dial spin, and it turned
/// the LONG way: the 1.8 km loop south over the bay with an AGL dip to
/// 126 ft (LXGB, 2026-07-24). Stepping the dial keeps every write's turn
/// direction unambiguous; at 40 degrees per 500 ms tick a 90-degree leg
/// change completes in ~1.5 s, faster than the AP banks anyway.
pub fn slew_bug(current: f64, target: f64, max_step: f64) -> f64 {
    let d = crate::angles::signed_diff_deg(target, current);
    (current + d.clamp(-max_step, max_step)).rem_euclid(360.0)
}

/// Bank beyond which the AP is judged to be IN a turn. X-Plane's heading
/// mode sits at 11-13 degrees for a standard-rate turn; the orbit at LXGB
/// started from 4.3 degrees, so the threshold sits under that, above the
/// one or two degrees of wings-level jitter.
pub const TURN_IN_PROGRESS_BANK_DEG: f64 = 3.0;
/// A wanted turn smaller than this against the bank in progress is left to
/// the AP: it is finishing a capture, not going the long way round.
pub const REVERSAL_MIN_DEG: f64 = 10.0;
/// Until the autopilot is banked INTO the wanted turn, the bug leads the
/// nose by no more than this: a small bug on the right side starts the turn
/// the right way, and once the turn is in progress the sim's refusal to
/// reverse it works for us. The rectangle circuit slewed the bug 90 degrees
/// off the nose from wings level, and the SF50 went round the long way on 13
/// of its 30 circuit turns (the A330 on 2 of 4), each a 270 right for a 90
/// left (ETND, 2026-09-23).
pub const TURN_ENTRY_LEAD_DEG: f64 = 25.0;
/// Wings level, for [`bug_toward`]'s roll-out hysteresis.
pub const LEVEL_BANK_DEG: f64 = 1.0;
/// A bug this close to the heading is "on the nose".
pub const ON_NOSE_DEG: f64 = 5.0;

/// Banked INTO a turn the other way from `turn` (degrees, positive right):
/// past `bank_threshold` of bank, a wanted turn past [`REVERSAL_MIN_DEG`],
/// and the bank's sign against it. The one reversal test both steers use.
pub fn banked_against(turn: f64, bank_deg: f64, bank_threshold: f64) -> bool {
    bank_deg.abs() > bank_threshold && turn.abs() > REVERSAL_MIN_DEG && (turn > 0.0) != (bank_deg > 0.0)
}

/// Step the bug toward `want_mag` (as [`slew_bug`]) unless the aircraft is
/// banked INTO a turn the other way, in which case hold the bug ON THE NOSE
/// so the AP rolls out first.
///
/// The sim's heading mode does not reverse a turn in progress. At LXGB
/// (2026-09-14) the authored follower captured waypoint 2 while banked four
/// degrees right for it and stepped the bug 25 degrees LEFT for waypoint 3;
/// the AP rolled on to 12 degrees right and went round the long way, 340
/// degrees, and because pure pursuit keeps the bug moving in the direction
/// of the turn it never caught up: five minutes of standard-rate orbits
/// drifting downwind across the bay, waypoints caught only when a circle
/// happened to pass within the capture radius. A bug on the nose is a
/// capture, and a capture is the one thing that rolls that AP level; from
/// level the next slew starts the turn the right way.
pub fn bug_toward(
    current_bug: f64,
    want_mag: f64,
    heading_mag: f64,
    phi_deg: f64,
    max_step: f64,
) -> f64 {
    let turn = crate::angles::signed_diff_deg(want_mag, heading_mag);
    // Hysteresis for a big turn: once the bug is on the nose to roll the
    // wrong-way bank out, it stays there until the wings are LEVEL, not merely
    // under the in-turn threshold. LIPK (2026-09-25): at 2 to 3 degrees of
    // left bank the 25 degree right lead went back on, the autopilot kept
    // turning left, the bank passed 3, the bug went back to the nose, and the
    // two alternated tick by tick for 50 s while the 172 turned 35 degrees
    // the wrong way.
    let on_nose =
        crate::angles::abs_diff_deg(current_bug, heading_mag) < ON_NOSE_DEG;
    let reversal_bank = if on_nose && turn.abs() > TURN_ENTRY_LEAD_DEG {
        LEVEL_BANK_DEG
    } else {
        TURN_IN_PROGRESS_BANK_DEG
    };
    if banked_against(turn, phi_deg, reversal_bank) {
        return slew_bug(current_bug, heading_mag, max_step);
    }
    let committed =
        phi_deg.abs() > TURN_IN_PROGRESS_BANK_DEG && (turn > 0.0) == (phi_deg > 0.0);
    if !committed && turn.abs() > TURN_ENTRY_LEAD_DEG {
        let entry = (heading_mag + turn.signum() * TURN_ENTRY_LEAD_DEG).rem_euclid(360.0);
        return slew_bug(current_bug, entry, max_step);
    }
    slew_bug(current_bug, want_mag, max_step)
}

/// How a loop's heading bug is SET, the way a pilot sets it (issue #93):
/// once per leg or phase (`key`), a turn led through at once (a move of
/// [`BUG_STEER_DEG`] or more is steering: the lead through a turn, a new
/// intercept), and otherwise a correction only when it has grown past the
/// loop's `deadband_deg` and not more often than every `every_s`. The laws
/// keep computing their fine bug every tick; the KNOB moves only when this
/// says so. [`HeadingHold`] is the follower's version (a whole
/// degree per leg, 5 degrees, 30 s); these loops fly patterns and finals, so
/// each passes its own deadband. LIPR 2026-09-26: "a human pilot would never
/// use the heading bug like that"; LEMG 2026-09-28: the circuit's downwind
/// knob stepped 10 to 24 times in two minutes on a one-degree crab, and
/// autoland's final 40 to 80 times in 100 s in tenths. Pure.
#[derive(Debug, Default, Clone, Copy)]
pub struct BugHold {
    key: Option<u32>,
    set: Option<f64>,
    last_write_s: f64,
}

/// A bug move this big is steering, not a correction: written at once.
pub const BUG_STEER_DEG: f64 = 5.0;

/// A loop's correction deadband: the smallest correction worth a hand on
/// the knob, and how often.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BugDeadband {
    pub deg: f64,
    pub every_s: f64,
}

impl BugHold {
    /// The bug to set now (magnetic), or `None` to leave the knob alone.
    pub fn decide(&mut self, key: u32, want_mag: f64, now_s: f64, db: BugDeadband) -> Option<f64> {
        if !want_mag.is_finite() {
            return None;
        }
        let want = want_mag.rem_euclid(360.0);
        let write = match (self.key == Some(key), self.set) {
            (true, Some(set)) => {
                let off = crate::angles::abs_diff_deg(want, set);
                off >= BUG_STEER_DEG
                    || (off >= db.deg && now_s - self.last_write_s >= db.every_s)
            }
            _ => true,
        };
        if !write {
            return None;
        }
        self.key = Some(key);
        self.set = Some(want);
        self.last_write_s = now_s;
        Some(want)
    }

    /// What the knob was last set to (the laws' `current_bug`), if anything.
    pub fn set(&self) -> Option<f64> {
        self.set
    }

    /// The knob read back after a write (a knob that only stepped part of
    /// the way, or a failed write: `None` forgets it so the next tick sets
    /// it afresh).
    pub fn took(&mut self, read_mag: Option<f64>) {
        match read_mag.filter(|v| v.is_finite()) {
            Some(v) => self.set = Some(v.rem_euclid(360.0)),
            None => {
                self.key = None;
                self.set = None;
            }
        }
    }

    /// Forget everything: the next decision writes.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// How the follower handles the heading bug: the way a pilot does. It was
/// rewritten every tick (bearing to the fix plus a live crab, stepped 25
/// degrees ahead of the nose through every turn), and the pilot watched it
/// twitch for the whole flight (LIPR 2026-09-26): "if we want to go a certain
/// heading, just set the heading". So: a leg's heading is SET once, a whole
/// degree, wind correction included; it is touched again only when the
/// track has drifted enough to need it, wings level, not more than twice a
/// minute; and a new heading on the far side of a turn still in progress
/// waits for the wings to come level (the autopilot turns the short way
/// from where the nose is, and a bug set against the bank went round the
/// long way, LXGB 2026-09-14).
#[derive(Debug, Default, Clone, Copy, PartialEq)]
enum HeadingTurn {
    #[default]
    Direct,
    Entry { sign: f64 },
    Rollout { sign: f64 },
}

#[derive(Debug, Default)]
pub struct HeadingHold {
    leg: Option<usize>,
    set: Option<f64>,
    last_write_s: f64,
    turn: HeadingTurn,
}

/// A correction smaller than this is not worth reaching for the knob.
const BUG_CORRECT_DEG: f64 = 5.0;
/// And not more often than this.
const BUG_CORRECT_EVERY_S: f64 = 30.0;
/// Wings this level count as level for a correction.
const BUG_LEVEL_BANK_DEG: f64 = 5.0;

// The follower's turn staging shares the circuit's numbers (it had its own
// copies of the same values): a large turn starts [`TURN_ENTRY_LEAD_DEG`]
// off the nose, since X-Plane's HDG mode can choose the long way when the
// final heading is selected in one jump, even from nearly level flight (C172
// EGBB-EGTF, 2026-10-01, #70); a bank past [`TURN_IN_PROGRESS_BANK_DEG`] has
// committed to a direction (or exposed a wrong-way entry), deliberately
// below the heading mode's 11-17 degree turn; every turn past
// [`REVERSAL_MIN_DEG`] is staged; once neutralised, the bug stays on the
// captured heading until the wings are within [`LEVEL_BANK_DEG`].

/// The staged entry bug: at most [`TURN_ENTRY_LEAD_DEG`] off the nose toward
/// the turn, a whole degree.
fn entry_bug(heading_mag: f64, sign: f64, turn_abs: f64) -> f64 {
    (heading_mag + sign * turn_abs.min(TURN_ENTRY_LEAD_DEG))
        .round()
        .rem_euclid(360.0)
}

impl HeadingHold {
    /// The heading to set now, or `None` to leave the knob alone. Pure.
    pub fn decide(&mut self, leg: usize, want_mag: f64, heading_mag: f64, bank_deg: f64, now_s: f64) -> Option<f64> {
        let want = want_mag.round().rem_euclid(360.0);
        let turn = crate::angles::signed_diff_deg(want, heading_mag);
        let new_leg = self.leg != Some(leg) || self.set.is_none();
        let command = if new_leg {
            let sign = turn.signum();
            if banked_against(turn, bank_deg, TURN_IN_PROGRESS_BANK_DEG) {
                self.turn = HeadingTurn::Rollout { sign };
                Some(heading_mag.round().rem_euclid(360.0))
            } else if turn.abs() > REVERSAL_MIN_DEG {
                self.turn = HeadingTurn::Entry { sign };
                Some(entry_bug(heading_mag, sign, turn.abs()))
            } else {
                self.turn = HeadingTurn::Direct;
                Some(want)
            }
        } else {
            match self.turn {
                HeadingTurn::Rollout { .. } => {
                    if bank_deg.abs() <= LEVEL_BANK_DEG {
                        if turn.abs() <= REVERSAL_MIN_DEG {
                            self.turn = HeadingTurn::Direct;
                            Some(want)
                        } else {
                            let sign = turn.signum();
                            self.turn = HeadingTurn::Entry { sign };
                            Some(entry_bug(heading_mag, sign, turn.abs()))
                        }
                    } else {
                        None
                    }
                }
                HeadingTurn::Entry { sign } => {
                    if bank_deg.abs() > TURN_IN_PROGRESS_BANK_DEG && bank_deg.signum() != sign {
                        self.turn = HeadingTurn::Rollout { sign };
                        Some(heading_mag.round().rem_euclid(360.0))
                    } else if bank_deg.abs() > TURN_IN_PROGRESS_BANK_DEG && bank_deg.signum() == sign {
                        self.turn = HeadingTurn::Direct;
                        Some(want)
                    } else {
                        None
                    }
                }
                HeadingTurn::Direct => {
                    let off = crate::angles::abs_diff_deg(want, self.set.unwrap_or(want));
                    (off >= BUG_CORRECT_DEG
                        && bank_deg.abs() < BUG_LEVEL_BANK_DEG
                        && now_s - self.last_write_s >= BUG_CORRECT_EVERY_S)
                        .then_some(want)
                }
            }
        };
        let command = command?;
        self.leg = Some(leg);
        self.set = Some(command);
        self.last_write_s = now_s;
        Some(command)
    }

    /// Forget the leg, so the next tick sets its heading afresh (a rejoin).
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn banked_against_is_a_bank_the_other_way_past_the_smallest_reversal() {
        assert!(banked_against(-90.0, 12.0, TURN_IN_PROGRESS_BANK_DEG), "right bank, left turn");
        assert!(banked_against(90.0, -12.0, TURN_IN_PROGRESS_BANK_DEG), "left bank, right turn");
        assert!(!banked_against(90.0, 12.0, TURN_IN_PROGRESS_BANK_DEG), "into the turn");
        assert!(!banked_against(-8.0, 12.0, TURN_IN_PROGRESS_BANK_DEG), "a capture finishing");
        assert!(!banked_against(-90.0, 2.0, TURN_IN_PROGRESS_BANK_DEG), "wings near level");
        assert!(banked_against(-90.0, 2.0, LEVEL_BANK_DEG), "the roll-out hysteresis");
    }

    /// The two steers are one rule: from level, a 90 degree turn either way
    /// starts on the same entry bug, and banked the other way both put the
    /// bug on the nose.
    #[test]
    fn the_circuit_steer_and_the_follower_steer_agree() {
        for (hdg, want) in [(190.0, 280.0), (190.0, 100.0), (350.0, 80.0), (10.0, 280.0)] {
            let circuit = bug_toward(hdg, want, hdg, 0.0, 360.0);
            let mut hold = HeadingHold::default();
            let follower = hold.decide(0, want, hdg, 0.0, 0.0).expect("a new leg is set");
            assert!(
                crate::angles::abs_diff_deg(circuit, follower) < 0.51,
                "{hdg} to {want}: circuit {circuit} follower {follower}"
            );
            let turn = crate::angles::turn_deg(hdg, want);
            assert!(
                (crate::angles::turn_deg(hdg, circuit) - turn.signum() * TURN_ENTRY_LEAD_DEG).abs()
                    < 0.51,
                "{hdg} to {want}: {circuit}"
            );
            let against = -turn.signum() * 12.0;
            let circuit = bug_toward(hdg, want, hdg, against, 360.0);
            let mut hold = HeadingHold::default();
            let follower = hold.decide(0, want, hdg, against, 0.0).expect("set");
            assert!(crate::angles::abs_diff_deg(circuit, hdg) < 0.51, "{circuit}");
            assert!(crate::angles::abs_diff_deg(follower, hdg) < 0.51, "{follower}");
        }
    }

    #[test]
    fn a_small_turn_is_set_directly_by_both() {
        let mut hold = HeadingHold::default();
        assert_eq!(hold.decide(0, 200.0, 190.0, 0.0, 0.0), Some(200.0));
        assert_eq!(bug_toward(190.0, 200.0, 190.0, 0.0, 360.0), 200.0);
    }

    #[test]
    fn the_knob_is_set_once_per_key_and_corrected_past_the_deadband() {
        let db = BugDeadband { deg: 2.0, every_s: 10.0 };
        let mut h = BugHold::default();
        assert_eq!(h.decide(1, 90.0, 0.0, db), Some(90.0), "a new key writes");
        assert_eq!(h.decide(1, 91.0, 1.0, db), None, "inside the deadband");
        assert_eq!(h.decide(1, 93.0, 5.0, db), None, "past it, too soon");
        assert_eq!(h.decide(1, 93.0, 10.0, db), Some(93.0), "past it, in time");
        assert_eq!(h.decide(1, 93.0 + BUG_STEER_DEG, 10.5, db), Some(98.0), "steering at once");
        assert_eq!(h.decide(2, 98.0, 11.0, db), Some(98.0), "a new key writes");
    }
}
