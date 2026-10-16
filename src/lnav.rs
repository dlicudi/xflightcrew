//! The enroute follower's lateral on the aircraft's own LNAV (#227).
//!
//! The pilot, 2026-10-06: "for navigation we should rely on FMS, for lateral
//! navigation". On an airframe that declares its LNAV (`autoflight:`
//! `lnav_command` and `lnav_status`, the 737 only for now) the full-auto
//! follower (`procedure`, `EndAction::Enroute`) hands the lateral of each
//! plain leg to a fix (TF, DF, IF) to the FMS, keeps sequencing it with
//! `xcopilot/fms/goto` from its OWN active leg, and WATCHES the answer; its
//! heading bug is the fallback, taken loudly and retried only on the next
//! leg. The vertical stays the follower's.
//!
//! # The probe (737 at FL360 over Spain, 2026-10-06)
//!
//! - `lnav_press` engages it: the lamp 1, `autopilot_state` 540672 (LNAV
//!   524288 + ALT HOLD; HDG SEL replaced). It turns at about 25 degrees of
//!   bank, intercepts at about 40 and captures `gps_xtk` to 0.0.
//! - It flies the FMS's ACTIVE leg, and a load makes the FIRST leg active:
//!   it turned back for a fix 645 nm behind. So it is engaged only once the
//!   FMS's active ident IS the follower's leg (`goto` re-targets it live),
//!   and a disagreement once agreed is answered with a goto, then a drop.
//! - An empty FMS (`gps_nav_id` "") never agrees, so LNAV is never engaged
//!   on one; a leg whose fix the payload did not carry never agrees either,
//!   and stays on the bug.
//!
//! Pure: the follower feeds a [`Tick`] and acts on the [`Decision`]; every
//! press and the hand-back go through the panel's owner (`autoflight`).

use crate::nav::ProcLeg;

use crate::autoflight::{LnavEngage, LnavStep};

/// Said once when LNAV takes the route (and again only after a fallback).
pub const ENGAGE_LINE: &str = "LNAV engaged, following the route.";
/// Said on every fallback to the heading bug.
pub const FALLBACK_LINE: &str = "LNAV dropped, I'm on heading.";

/// Captured: on the leg within this, nm (the probe captured to 0.0).
pub const CAPTURED_XTK_NM: f64 = 0.3;
/// Once captured, off the leg by more than this is a fallback, nm.
pub const CAPTURED_LIMIT_NM: f64 = 2.0;
/// Capturing: no verdict on divergence before this, s. A flyby onto a new
/// leg starts MILES off it (a 90 degree turn at FL360 leads by about 6 nm,
/// the cross-track to the new leg at the sequence), and the turn toward an
/// intercept can open it before it closes.
pub const CAPTURE_GRACE_S: f64 = 120.0;
/// Capturing past the grace: off by this much more than the best seen is
/// diverging, nm.
pub const DIVERGE_NM: f64 = 2.0;
/// The capture's floor allowance, s ([`capture_allowance_s`]).
pub const CAPTURE_MIN_S: f64 = 180.0;
/// The intercept angle the probe flew, degrees.
pub const INTERCEPT_DEG: f64 = 40.0;
/// LNAV reading out (past a press's settle) this long is lost, s.
pub const LOST_S: f64 = 2.0;
/// The FMS's active ident off the follower's leg this long: a goto, then
/// (on a leg it agreed on) a drop after as long again; on a leg it never
/// agreed on, the leg is not in the FMS and goes back to the bug, s.
pub const IDENT_MISMATCH_S: f64 = 5.0;
/// Fallbacks in one run before LNAV is retired for it.
pub const MAX_FALLBACKS: u32 = 3;
/// The FMS on the leg but the aircraft off its line by more than this, nm:
/// the follower flies the fix direct from here (a resume's join, about
/// 13 nm off RIBEL-INPIP), where LNAV would only arm (737 at FL380, "LNAV
/// NOT ON INTERCEPT HEADING", 2026-10-16), so the FMS is given the same leg
/// by its own DIR TO first. Wide of the few miles an intercept engages from.
pub const DIRECT_TO_XTK_NM: f64 = 5.0;

/// A leg LNAV can fly: a plain leg to a fix (TF, DF, IF) with a resolved
/// fix and an ident, never a hold, an RF arc, a course leg or a MAP. Pure.
pub fn leg_flyable(leg: &ProcLeg) -> bool {
    matches!(leg.leg_type.as_str(), "TF" | "DF" | "IF")
        && leg.fix.is_some()
        && !leg.fix_ident.trim().is_empty()
        && !leg.is_map
        && !leg.is_hold()
        && leg.arc().is_none()
}

/// Why the heading bug, not LNAV, flies this leg: the follower's ONE
/// answer to "is it using the GPS?", blackboxed `lnav` `on_bug` on each
/// change (an EGKK departure flew the bug with the FMS loaded and nothing
/// said why, 2026-10-14). `None` while LNAV owns the lateral. Pure.
#[derive(Clone, Copy)]
pub struct BugWhy<'a> {
    pub owned: bool,
    pub declared: bool,
    pub enroute: bool,
    pub retired: bool,
    /// The follower's own steer is not a plain direct (a course, an arc,
    /// a hold, a pattern): said as its word.
    pub steering: Option<&'static str>,
    pub leg: &'a ProcLeg,
    pub fms_ident: Option<&'a str>,
}

pub fn bug_reason(w: &BugWhy) -> Option<&'static str> {
    if w.owned {
        return None;
    }
    Some(if !w.declared {
        "lnav_not_declared"
    } else if !w.enroute {
        "not_enroute"
    } else if w.retired {
        "retired"
    } else if let Some(s) = w.steering {
        s
    } else if !leg_flyable(w.leg) {
        "leg_not_flyable"
    } else {
        match w.fms_ident {
            None => "fms_unread",
            Some(f) if f.trim().is_empty() => "fms_empty",
            Some(f) if !same_ident(f, &w.leg.fix_ident) => "fms_not_on_leg",
            Some(_) => "engaging",
        }
    })
}

/// The FMS's active ident is this leg's fix, as the route payload spelt it
/// (`simbrief::fms_ident`). An empty FMS or an empty ident never is. Pure.
pub fn same_ident(fms: &str, leg_ident: &str) -> bool {
    let f = fms.trim().to_ascii_uppercase();
    !f.is_empty() && !leg_ident.trim().is_empty() && f == crate::simbrief::fms_ident(leg_ident)
}

/// How long a capture from `xtk0_nm` off at `gs_kts` is given before it is
/// a fallback: the floor plus half again the time to close at the probe's
/// intercept angle. Pure.
pub fn capture_allowance_s(xtk0_nm: f64, gs_kts: f64) -> f64 {
    let closing_kts = gs_kts.max(100.0) * INTERCEPT_DEG.to_radians().sin();
    CAPTURE_MIN_S + 1.5 * 3600.0 * xtk0_nm.abs() / closing_kts
}

/// Why LNAV was given up for the bug.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Why {
    /// It read out past a press's settle for [`LOST_S`].
    StatusLost,
    /// Off the leg by more than [`CAPTURED_LIMIT_NM`] once captured.
    CrossTrack,
    /// Opening from the best seen by [`DIVERGE_NM`] past the grace.
    Diverging,
    /// Not captured inside [`capture_allowance_s`].
    NoCapture,
    /// The FMS's active leg off the follower's after a goto.
    FmsDisagrees,
    /// Pressed [`crate::autoflight::LNAV_PRESSES`] times, never in.
    NoEngage,
    /// The follower's closure watchdog found the fix not closing.
    LegStalled,
}

impl Why {
    pub fn word(self) -> &'static str {
        match self {
            Why::StatusLost => "status_lost",
            Why::CrossTrack => "cross_track",
            Why::Diverging => "diverging",
            Why::NoCapture => "no_capture",
            Why::FmsDisagrees => "fms_disagrees",
            Why::NoEngage => "no_engage",
            Why::LegStalled => "leg_stalled",
        }
    }
}

/// What the follower does this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Act {
    Nothing,
    /// Press the LNAV button (`autoflight::press_lnav`).
    Press,
    /// Sequence the FMS to the follower's leg again (`xcopilot/fms/goto`).
    ReGoto,
    /// The FMS's own DIR TO the leg's fix (`tools::direct_to::execute`),
    /// once per leg; LNAV is then pressed as ever.
    DirectTo,
    /// A fallback: LNAV out through the owner, the bug flies, said.
    Drop(Why),
    /// A planned hand-back to the bug (a leg LNAV does not fly): LNAV out
    /// through the owner, nothing said.
    HandBack(&'static str),
}

/// What happened this tick, for the tape and the voice.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Note {
    /// LNAV reads in. `say`: the engage line is owed (the first, or the
    /// first after a fallback). `reengage`: after a fallback.
    Engaged {
        say: bool,
        reengage: bool,
        presses: u32,
    },
    /// On the leg within [`CAPTURED_XTK_NM`], `secs` after it began.
    Captured { secs: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Decision {
    pub act: Act,
    pub note: Option<Note>,
}

impl Decision {
    fn act(act: Act) -> Self {
        Self { act, note: None }
    }
}

/// One tick, as the follower read it.
#[derive(Debug, Clone, Copy)]
pub struct Tick<'a> {
    pub now_s: f64,
    /// The follower's leg.
    pub idx: usize,
    /// This tick the leg is LNAV's to fly ([`leg_flyable`], flown direct:
    /// no hold, arc, course, pattern or join steering it).
    pub eligible: bool,
    pub leg_ident: &'a str,
    /// The leg after it: the FMS may sequence there itself first.
    pub next_ident: Option<&'a str>,
    /// `gps_nav_id`; `None` unread.
    pub fms_ident: Option<&'a str>,
    /// `autoflight::read_lnav`; `None` unread.
    pub engaged: Option<bool>,
    /// `gps_xtk`, nm signed; `None` unread.
    pub xtk_nm: Option<f64>,
    pub gs_kts: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    /// The bug flies.
    Off,
    /// The bug still flies while the button is pressed and read.
    Engaging,
    /// LNAV flies, closing on its leg.
    Capturing {
        since_s: f64,
        xtk0: Option<f64>,
        best: Option<f64>,
    },
    /// LNAV flies, on its leg.
    Captured,
}

/// The follower's lateral owner: the bug, or the aircraft's LNAV. Pure.
#[derive(Debug, Clone)]
pub struct LnavLateral {
    phase: Phase,
    engage: LnavEngage,
    /// The leg a fallback happened on: LNAV is retried on the next.
    dropped_leg: Option<usize>,
    fallbacks: u32,
    retired: bool,
    /// The engage line has been said since the last fallback.
    said_engaged: bool,
    ever_engaged: bool,
    /// The leg the FMS agreed with the follower on.
    agreed_leg: Option<usize>,
    /// The FMS off the follower's leg since.
    mismatch_since: Option<f64>,
    /// The leg the one re-goto was spent on.
    regoto_leg: Option<usize>,
    /// The leg the one DIR TO was spent on.
    direct_leg: Option<usize>,
    lost_since: Option<f64>,
    /// When LNAV was last taken out (HDG SEL settling).
    out_at: Option<f64>,
    /// The (leg, FMS ident) a capture is judged from: either changing is a
    /// new leg for LNAV.
    basis: Option<(usize, String)>,
    /// The follower's leg last tick: a new one starts its own FMS clock.
    leg: Option<usize>,
}

impl Default for LnavLateral {
    fn default() -> Self {
        Self {
            phase: Phase::Off,
            engage: LnavEngage::default(),
            dropped_leg: None,
            fallbacks: 0,
            retired: false,
            said_engaged: false,
            ever_engaged: false,
            agreed_leg: None,
            mismatch_since: None,
            regoto_leg: None,
            direct_leg: None,
            lost_since: None,
            out_at: None,
            basis: None,
            leg: None,
        }
    }
}

impl LnavLateral {
    /// LNAV flies the lateral: the follower writes no heading bug and
    /// presses no HDG SEL.
    pub fn owns_lateral(&self) -> bool {
        matches!(self.phase, Phase::Capturing { .. } | Phase::Captured)
    }

    /// The follower may write its heading bug (one writer: never while LNAV
    /// flies).
    pub fn bug_write_allowed(&self) -> bool {
        !self.owns_lateral()
    }

    /// The follower keeps HDG SEL in: only with the bug its owner, not while
    /// LNAV is being pressed in, and not while a hand-back's HDG SEL press
    /// settles (a second press of a toggle undoes the first).
    pub fn heading_mode_wanted(&self, now_s: f64) -> bool {
        self.phase == Phase::Off
            && self
                .out_at
                .is_none_or(|t| now_s - t >= crate::autoflight::LNAV_SETTLE_S)
    }

    /// The button is being pressed in, or LNAV flies: the reads are owed
    /// even on a leg it cannot fly (to hand it back).
    pub fn in_play(&self) -> bool {
        self.phase != Phase::Off
    }

    pub fn retired(&self) -> bool {
        self.retired
    }

    pub fn fallbacks(&self) -> u32 {
        self.fallbacks
    }

    /// The best cross-track seen on the capture in progress.
    pub fn best_xtk(&self) -> Option<f64> {
        match self.phase {
            Phase::Capturing { best, .. } => best,
            _ => None,
        }
    }

    pub fn presses(&self) -> u32 {
        self.engage.presses()
    }

    /// A fallback the follower found itself (its closure watchdog): `true`
    /// when LNAV had the lateral and is now dropped.
    pub fn drop_now(&mut self, idx: usize, now_s: f64) -> bool {
        if !self.in_play() {
            return false;
        }
        self.fall_back(idx, now_s);
        true
    }

    fn fall_back(&mut self, idx: usize, now_s: f64) {
        self.phase = Phase::Off;
        self.dropped_leg = Some(idx);
        self.fallbacks += 1;
        self.said_engaged = false;
        self.out_at = Some(now_s);
        self.lost_since = None;
        self.basis = None;
        if self.fallbacks >= MAX_FALLBACKS {
            self.retired = true;
        }
    }

    fn hand_back(&mut self, now_s: f64) {
        self.phase = Phase::Off;
        self.out_at = Some(now_s);
        self.lost_since = None;
        self.basis = None;
    }

    fn drop(&mut self, why: Why, idx: usize, now_s: f64) -> Decision {
        self.fall_back(idx, now_s);
        Decision::act(Act::Drop(why))
    }

    /// One tick. Pure.
    pub fn decide(&mut self, t: Tick) -> Decision {
        let nothing = Decision::act(Act::Nothing);
        if self.retired {
            return nothing;
        }
        if self.leg != Some(t.idx) {
            // The follower's goto for this leg is in flight: the old leg's
            // disagreement is not this one's.
            self.leg = Some(t.idx);
            self.mismatch_since = None;
        }
        let exact = t.fms_ident.map(|f| same_ident(f, t.leg_ident));
        // Owned, the FMS sequencing one leg ahead of us by itself is LNAV's
        // own turn anticipation, not a disagreement.
        let agrees = t
            .fms_ident
            .map(|f| same_ident(f, t.leg_ident) || t.next_ident.is_some_and(|n| same_ident(f, n)));
        if (agrees == Some(true) && self.owns_lateral()) || exact == Some(true) {
            self.agreed_leg = Some(t.idx);
        }
        let ok = if self.owns_lateral() { agrees } else { exact };
        match ok {
            Some(true) => self.mismatch_since = None,
            Some(false) => {
                self.mismatch_since.get_or_insert(t.now_s);
            }
            // Unread: no measurement, the clock neither starts nor stops.
            None => {}
        }
        let mismatch_s = self.mismatch_since.map_or(0.0, |s| t.now_s - s);

        match self.phase {
            Phase::Off => {
                if !t.eligible || self.dropped_leg == Some(t.idx) {
                    return nothing;
                }
                if exact != Some(true) {
                    // The goto lost to a load in flight, or the FMS moved:
                    // one more goto per leg, then the bug keeps it.
                    if mismatch_s >= IDENT_MISMATCH_S && self.regoto_leg != Some(t.idx) {
                        self.regoto_leg = Some(t.idx);
                        self.mismatch_since = Some(t.now_s);
                        return Decision::act(Act::ReGoto);
                    }
                    return nothing;
                }
                // On the leg's fix but far off its line: DIR TO once, then
                // engage whatever it left (a failed one is today's path).
                if t.xtk_nm.is_some_and(|x| x.abs() > DIRECT_TO_XTK_NM)
                    && self.direct_leg != Some(t.idx)
                {
                    self.direct_leg = Some(t.idx);
                    return Decision::act(Act::DirectTo);
                }
                self.phase = Phase::Engaging;
                self.engage = LnavEngage::default();
                self.engaging(t)
            }
            Phase::Engaging => {
                if !t.eligible || exact == Some(false) {
                    self.hand_back(t.now_s);
                    return Decision::act(Act::HandBack("engage_abandoned"));
                }
                self.engaging(t)
            }
            Phase::Capturing { .. } | Phase::Captured => self.owned(t, mismatch_s),
        }
    }

    fn engaging(&mut self, t: Tick) -> Decision {
        match self.engage.step(t.now_s, t.engaged) {
            LnavStep::Engaged => {
                let say = !self.said_engaged;
                let reengage = self.ever_engaged;
                self.said_engaged = true;
                self.ever_engaged = true;
                self.lost_since = None;
                self.begin_capture(t);
                Decision {
                    act: Act::Nothing,
                    note: Some(Note::Engaged {
                        say,
                        reengage,
                        presses: self.engage.presses(),
                    }),
                }
            }
            LnavStep::Press => Decision::act(Act::Press),
            LnavStep::Wait | LnavStep::Unread => Decision::act(Act::Nothing),
            LnavStep::Failed => self.drop(Why::NoEngage, t.idx, t.now_s),
        }
    }

    fn begin_capture(&mut self, t: Tick) {
        let x = t.xtk_nm.map(f64::abs);
        self.phase = Phase::Capturing {
            since_s: t.now_s,
            xtk0: x,
            best: x,
        };
        self.basis = Some((t.idx, t.fms_ident.unwrap_or("").to_string()));
    }

    fn owned(&mut self, t: Tick, mismatch_s: f64) -> Decision {
        if !t.eligible {
            self.hand_back(t.now_s);
            return Decision::act(Act::HandBack("leg_not_lnav"));
        }
        // A new leg for LNAV (ours sequenced, or the FMS did): its capture is
        // judged afresh, from where the turn onto it starts.
        if let Some(f) = t.fms_ident {
            let changed = self
                .basis
                .as_ref()
                .is_none_or(|(i, id)| *i != t.idx || id != f);
            if changed {
                self.begin_capture(t);
            }
        }
        // The mode itself.
        match t.engaged {
            Some(false) if !self.engage.settling(t.now_s) => {
                let since = *self.lost_since.get_or_insert(t.now_s);
                if t.now_s - since >= LOST_S {
                    return self.drop(Why::StatusLost, t.idx, t.now_s);
                }
            }
            Some(true) => self.lost_since = None,
            _ => {}
        }
        // The FMS pointed where the follower is.
        if mismatch_s >= IDENT_MISMATCH_S {
            if self.agreed_leg != Some(t.idx) {
                // Never agreed on this leg: its fix is not in the FMS.
                self.hand_back(t.now_s);
                return Decision::act(Act::HandBack("leg_not_in_fms"));
            }
            if self.regoto_leg != Some(t.idx) {
                self.regoto_leg = Some(t.idx);
                self.mismatch_since = Some(t.now_s);
                return Decision::act(Act::ReGoto);
            }
            return self.drop(Why::FmsDisagrees, t.idx, t.now_s);
        }
        // The answer: the cross-track.
        let Some(x) = t.xtk_nm.filter(|v| v.is_finite()).map(f64::abs) else {
            return Decision::act(Act::Nothing);
        };
        match self.phase {
            Phase::Capturing {
                since_s,
                xtk0,
                best,
            } => {
                let xtk0 = xtk0.unwrap_or(x);
                let best = best.map_or(x, |b| b.min(x));
                let secs = t.now_s - since_s;
                if x <= CAPTURED_XTK_NM {
                    self.phase = Phase::Captured;
                    return Decision {
                        act: Act::Nothing,
                        note: Some(Note::Captured { secs }),
                    };
                }
                self.phase = Phase::Capturing {
                    since_s,
                    xtk0: Some(xtk0),
                    best: Some(best),
                };
                if secs >= CAPTURE_GRACE_S && x > best + DIVERGE_NM {
                    return self.drop(Why::Diverging, t.idx, t.now_s);
                }
                if secs > capture_allowance_s(xtk0, t.gs_kts) {
                    return self.drop(Why::NoCapture, t.idx, t.now_s);
                }
                Decision::act(Act::Nothing)
            }
            Phase::Captured if x > CAPTURED_LIMIT_NM => self.drop(Why::CrossTrack, t.idx, t.now_s),
            _ => Decision::act(Act::Nothing),
        }
    }
}

#[cfg(test)]
mod tests;
