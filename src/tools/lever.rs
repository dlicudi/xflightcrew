//! A lever that answers only to its step commands: one notch per press, read
//! back from the lever itself after EVERY press (`aircraft::LeverCommands`).
//!
//! The Zibo 737 is the case (probed parked, 2026-10-06): a write to the stock
//! `flap_ratio` is put straight back, a write to the stock `speedbrake_ratio`
//! sticks in the stock dataref while its own lever never moves, and the stock
//! `flaps_down` / `flaps_up`, `speed_brakes_down_one` / `_up_one` step its
//! levers one detent each. So the lever is pressed toward the target a notch
//! at a time, each press waited on until the lever MOVES, and a press that
//! moves nothing is named, never answered "Roger" (the readback rule).
//!
//! One seam per lever on top of this: `tools::flaps::set_handle`,
//! `tools::controls::speedbrake_set`.

use crate::aircraft::LeverCommands;
use crate::xplane::XPlane;

/// The most presses one request may take: a 737's flap lever has nine
/// detents, so eight steps end to end, and a lever that ignores its commands
/// must not be hammered.
const MAX_PRESSES: usize = 12;
/// How often a press is read back while it is waited on.
const POLL_MS: u64 = 100;
/// A reading this far from the last is the lever having MOVED.
const MOVED: f64 = 0.005;

/// What a stepped lever did. The position is always the lever's READING.
#[derive(Debug, Clone, PartialEq)]
pub enum Stepped {
    /// At the target (within the tolerance), after `presses` presses.
    At { at: f64, presses: usize },
    /// The next notch would cross the target: the lever stopped at the
    /// notch nearest it on the side it was going (a target between two
    /// detents).
    Nearest { at: f64, presses: usize },
    /// A press that moved nothing within the timeout: the add-on did not
    /// take it. `at` is where the lever stayed.
    DidNotTake { at: f64, presses: usize },
    /// A failed read: no measurement, nothing pressed after it.
    Unread { presses: usize },
}

impl Stepped {
    /// Where the lever reads now, when it was read.
    pub fn at(&self) -> Option<f64> {
        match self {
            Stepped::At { at, .. }
            | Stepped::Nearest { at, .. }
            | Stepped::DidNotTake { at, .. } => Some(*at),
            Stepped::Unread { .. } => None,
        }
    }

    /// Whether the lever went where it was sent (or the nearest notch to it).
    pub fn took(&self) -> bool {
        matches!(self, Stepped::At { .. } | Stepped::Nearest { .. })
    }
}

/// Which way the next press goes: `Some(true)` toward MORE, `Some(false)`
/// toward LESS, `None` when the lever already reads the target. Pure.
pub fn next_press(at: f64, want: f64, tol: f64) -> Option<bool> {
    if (at - want).abs() <= tol {
        None
    } else {
        Some(at < want)
    }
}

/// Step the lever to `want`, one press at a time, each read back until it
/// moves (or `step_timeout_s` says it did not take). Blackboxed `lever`.
pub async fn step_lever(xp: &dyn XPlane, c: &LeverCommands, want: f64, tol: f64) -> Stepped {
    let out = step(xp, c, want, tol).await;
    crate::blackbox::record(
        "lever",
        serde_json::json!({
            "readback": c.readback, "want": want, "at": out.at(),
            "outcome": format!("{out:?}"),
        }),
    );
    out
}

async fn read(xp: &dyn XPlane, name: &str) -> Option<f64> {
    xp.get_float(name).await.ok().filter(|v| v.is_finite())
}

async fn step(xp: &dyn XPlane, c: &LeverCommands, want: f64, tol: f64) -> Stepped {
    let mut presses = 0usize;
    let Some(mut at) = read(xp, &c.readback).await else {
        return Stepped::Unread { presses };
    };
    loop {
        let Some(more) = next_press(at, want, tol) else {
            return Stepped::At { at, presses };
        };
        if presses >= MAX_PRESSES {
            return Stepped::DidNotTake { at, presses };
        }
        let cmd = if more { &c.more } else { &c.less };
        if xp.run_command(cmd).await.is_err() {
            return Stepped::DidNotTake { at, presses };
        }
        presses += 1;
        // Wait for the lever to MOVE: the add-on runs the command in its
        // own flight loop, a frame or two after the request.
        let deadline = tokio::time::Instant::now()
            + std::time::Duration::from_secs_f64(c.step_timeout_s.max(0.1));
        let now = loop {
            tokio::time::sleep(std::time::Duration::from_millis(POLL_MS)).await;
            let Some(v) = read(xp, &c.readback).await else {
                return Stepped::Unread { presses };
            };
            if (v - at).abs() > MOVED {
                break v;
            }
            if tokio::time::Instant::now() >= deadline {
                return Stepped::DidNotTake { at, presses };
            }
        };
        // Moved the wrong way, or across the target: a notch between two
        // detents. Stop here rather than hunt back and forth.
        let crossed = if more {
            now > want + tol
        } else {
            now < want - tol
        };
        let wrong_way = if more { now < at } else { now > at };
        at = now;
        if wrong_way {
            return Stepped::DidNotTake { at, presses };
        }
        if crossed {
            return Stepped::Nearest { at, presses };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xplane::FakeXPlane;

    fn cmds() -> LeverCommands {
        LeverCommands {
            more: "more".into(),
            less: "less".into(),
            readback: "lever".into(),
            step_timeout_s: 1.5,
        }
    }

    #[test]
    fn the_next_press_goes_toward_the_target() {
        assert_eq!(next_press(0.5, 0.625, 0.01), Some(true));
        assert_eq!(next_press(0.5, 0.0, 0.01), Some(false));
        assert_eq!(next_press(0.505, 0.5, 0.01), None);
    }

    /// The Zibo's flap lever, as probed: one eighth per press.
    #[tokio::test(start_paused = true)]
    async fn a_lever_is_stepped_a_notch_at_a_time_and_read_back() {
        let xp = FakeXPlane::with_reads(&[("lever", 0.0)]);
        xp.step_on_command("more", "lever", 0.125);
        xp.step_on_command("less", "lever", -0.125);
        let out = step_lever(&xp, &cmds(), 0.375, 0.01).await;
        assert_eq!(
            out,
            Stepped::At {
                at: 0.375,
                presses: 3
            }
        );
        assert_eq!(xp.ran_count("more"), 3);
        assert!(!xp.wrote("lever"), "a stepped lever is never written");
        let back = step_lever(&xp, &cmds(), 0.0, 0.01).await;
        assert_eq!(
            back,
            Stepped::At {
                at: 0.0,
                presses: 3
            }
        );
        assert_eq!(xp.ran_count("less"), 3);
    }

    #[tokio::test(start_paused = true)]
    async fn a_press_that_moves_nothing_is_named_and_not_repeated() {
        let xp = FakeXPlane::with_reads(&[("lever", 0.25)]);
        let out = step_lever(&xp, &cmds(), 0.5, 0.01).await;
        assert_eq!(
            out,
            Stepped::DidNotTake {
                at: 0.25,
                presses: 1
            }
        );
        assert!(!out.took());
        assert_eq!(xp.ran_count("more"), 1, "one press, then it is said");
    }

    #[tokio::test(start_paused = true)]
    async fn a_target_between_notches_stops_at_the_nearest_never_hunts() {
        let xp = FakeXPlane::with_reads(&[("lever", 0.0)]);
        xp.step_on_command("more", "lever", 0.4);
        xp.step_on_command("less", "lever", -0.4);
        let out = step_lever(&xp, &cmds(), 0.5, 0.01).await;
        assert_eq!(
            out,
            Stepped::Nearest {
                at: 0.8,
                presses: 2
            }
        );
        assert_eq!(xp.ran_count("less"), 0);
    }

    #[tokio::test(start_paused = true)]
    async fn a_lever_already_there_is_not_pressed_and_a_failed_read_presses_nothing() {
        let xp = FakeXPlane::with_reads(&[("lever", 0.5)]);
        assert_eq!(
            step_lever(&xp, &cmds(), 0.5, 0.01).await,
            Stepped::At {
                at: 0.5,
                presses: 0
            }
        );
        let unread = FakeXPlane::new();
        assert_eq!(
            step_lever(&unread, &cmds(), 0.5, 0.01).await,
            Stepped::Unread { presses: 0 }
        );
        assert!(!unread.ran("more"));
    }
}
