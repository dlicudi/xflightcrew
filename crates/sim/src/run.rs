//! Worker ownership survives cancellation until the worker and its I/O exit.
//!
//! A [`Slot`] is one loop's run: who owns it now (a [`Lease`] with an id and
//! a cancel token), how the last run ended, and optionally the loop's
//! process-wide "flying" flag ([`Slot::flagged`]). The flag has ONE writer,
//! the slot: set when a run begins, cleared at a cancel, when the run lowers
//! it ([`Lease::lower_flag`]) and when the run's lease drops. A run that was
//! superseded ([`Slot::take_over`], [`Slot::claim`]) no longer owns the slot,
//! so nothing it does on the way out (dropping its lease included) can flip
//! the flag, the outcome or the cleanup of the run that replaced it.
use std::sync::{
    atomic::{AtomicBool, AtomicU64, Ordering},
    Arc, Mutex,
};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    Running,
    Completed,
    HandedOff,
    Cancelled,
    PilotOverride,
    Failed(String),
}

struct State {
    id: u64,
    cancel: Arc<AtomicBool>,
    outcome: Outcome,
    /// The run lowered its flag and is only cleaning up: a new claim may
    /// supersede it.
    ending: bool,
}

pub struct Slot {
    state: Mutex<Option<State>>,
    next: AtomicU64,
    last: Mutex<Option<(u64, Outcome)>>,
    flag: Option<&'static AtomicBool>,
}
impl Slot {
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(None),
            next: AtomicU64::new(1),
            last: Mutex::new(None),
            flag: None,
        }
    }
    /// A slot that owns the loop's flag: the flag reads set exactly while a
    /// run that has not been cancelled, superseded or lowered holds it.
    pub const fn flagged(flag: &'static AtomicBool) -> Self {
        Self {
            state: Mutex::new(None),
            next: AtomicU64::new(1),
            last: Mutex::new(None),
            flag: Some(flag),
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Option<State>> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    fn set_flag(&self, on: bool) {
        if let Some(f) = self.flag {
            f.store(on, Ordering::SeqCst);
        }
    }

    fn install(&self, state: &mut Option<State>) -> Lease<'_> {
        let id = self.next.fetch_add(1, Ordering::SeqCst);
        let cancel = Arc::new(AtomicBool::new(false));
        *state = Some(State {
            id,
            cancel: cancel.clone(),
            outcome: Outcome::Running,
            ending: false,
        });
        self.set_flag(true);
        Lease {
            slot: self,
            id,
            cancel,
            cleanup: None,
        }
    }

    /// Retire the holder in place: its token cancelled, its outcome (a
    /// cancel, unless it had already recorded one) written as the last.
    fn supersede(&self, state: &mut Option<State>) {
        if let Some(s) = state.take() {
            s.cancel.store(true, Ordering::SeqCst);
            let outcome = match s.outcome {
                Outcome::Running => Outcome::Cancelled,
                o => o,
            };
            *self.last.lock().unwrap_or_else(|e| e.into_inner()) = Some((s.id, outcome));
        }
    }

    /// A cancelled worker still owns its slot until it has returned.
    pub fn begin(&self) -> Option<Lease<'_>> {
        let mut state = self.lock();
        if state.is_some() {
            return None;
        }
        Some(self.install(&mut state))
    }

    /// A new run unless a live one holds the slot (the old `ACTIVE.swap(true)`
    /// refusal). A cancelled or ending run still winding down is superseded:
    /// it keeps running to its exit but owns nothing any more.
    pub fn claim(&self) -> Option<Lease<'_>> {
        let mut state = self.lock();
        if state
            .as_ref()
            .is_some_and(|s| !s.ending && !s.cancel.load(Ordering::SeqCst))
        {
            return None;
        }
        self.supersede(&mut state);
        Some(self.install(&mut state))
    }

    /// A new run that replaces whatever holds the slot (a re-arm: the new
    /// run owns the aircraft from its first tick, the old one is cancelled
    /// and exits on its own).
    pub fn take_over(&self) -> Lease<'_> {
        let mut state = self.lock();
        self.supersede(&mut state);
        self.install(&mut state)
    }

    /// Cancel the run holding the slot; the flag drops at once. Returns
    /// whether a run was live (flagged or not, not already cancelled).
    pub fn cancel(&self) -> bool {
        let mut state = self.lock();
        let mut was = false;
        if let Some(s) = state.as_mut() {
            was = !s.ending && !s.cancel.load(Ordering::SeqCst);
            s.cancel.store(true, Ordering::SeqCst);
            s.outcome = Outcome::Cancelled;
        }
        self.set_flag(false);
        was
    }
    pub fn fail(&self, reason: &str) {
        if let Some(s) = self.lock().as_mut() {
            if !s.cancel.load(Ordering::SeqCst) {
                s.outcome = Outcome::Failed(reason.into());
                s.cancel.store(true, Ordering::SeqCst);
            }
        }
        self.set_flag(false);
    }
    pub fn running(&self) -> bool {
        self.lock().is_some()
    }
    /// A run holds the slot and has been neither cancelled nor lowered.
    pub fn live(&self) -> bool {
        self.lock()
            .as_ref()
            .is_some_and(|s| !s.ending && !s.cancel.load(Ordering::SeqCst))
    }
    pub fn record(&self, outcome: Outcome) {
        if let Some(s) = self.lock().as_mut() {
            if !s.cancel.load(Ordering::SeqCst) {
                s.outcome = outcome;
            }
        }
    }
    pub fn outcome(&self) -> Option<Outcome> {
        let state = self.lock();
        if state.is_some() {
            return Some(Outcome::Running);
        }
        self.last
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map(|(_, o)| o.clone())
    }
}

impl Default for Slot {
    fn default() -> Self {
        Self::new()
    }
}

pub struct Lease<'a> {
    slot: &'a Slot,
    pub id: u64,
    pub cancel: Arc<AtomicBool>,
    cleanup: Option<(&'static AtomicBool, fn())>,
}
impl Lease<'_> {
    pub fn with_cleanup(mut self, active: &'static AtomicBool, cleanup: fn()) -> Self {
        self.cleanup = Some((active, cleanup));
        self
    }
    /// This run still owns its slot (nobody took it over).
    pub fn current(&self) -> bool {
        self.slot.lock().as_ref().is_some_and(|s| s.id == self.id)
    }
    /// This run owns its slot and has not been cancelled: the loop's own
    /// "keep going" test.
    pub fn live(&self) -> bool {
        !self.cancel.load(Ordering::SeqCst) && self.current()
    }
    /// The run is over but still cleaning up: the flag drops now (readers
    /// see it stopped) and a new claim may supersede it. A no-op for a run
    /// already superseded, whose flag is the new run's.
    pub fn lower_flag(&self) {
        let mut state = self.slot.lock();
        if let Some(s) = state.as_mut().filter(|s| s.id == self.id) {
            s.ending = true;
            self.slot.set_flag(false);
        }
    }
    /// Record this run's outcome, if it still owns the slot.
    pub fn record(&self, outcome: Outcome) {
        if let Some(s) = self.slot.lock().as_mut().filter(|s| s.id == self.id) {
            if !s.cancel.load(Ordering::SeqCst) {
                s.outcome = outcome;
            }
        }
    }
}
impl Drop for Lease<'_> {
    fn drop(&mut self) {
        let mut state = self.slot.lock();
        if !state.as_ref().is_some_and(|s| s.id == self.id) {
            // Superseded: the slot, its flag and its cleanup are the new
            // run's. Only our own token is ours to cancel.
            self.cancel.store(true, Ordering::SeqCst);
            return;
        }
        if let Some((active, cleanup)) = self.cleanup {
            active.store(false, Ordering::SeqCst);
            cleanup();
        }
        self.slot.set_flag(false);
        let s = state.take().unwrap();
        let outcome = match s.outcome {
            Outcome::Running => Outcome::Failed("worker exited without completion".into()),
            o => o,
        };
        if !matches!(outcome, Outcome::Completed | Outcome::HandedOff) {
            s.cancel.store(true, Ordering::SeqCst);
        }
        *self.slot.last.lock().unwrap_or_else(|e| e.into_inner()) = Some((s.id, outcome));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cancellation_blocks_restart_until_old_worker_and_cleanup_finish() {
        let slot = Slot::new();
        let old = slot.begin().unwrap();
        slot.cancel();
        assert!(old.cancel.load(Ordering::SeqCst));
        assert!(slot.begin().is_none());
        slot.record(Outcome::Completed); // late success cannot erase cancellation
        drop(old);
        assert_eq!(slot.outcome(), Some(Outcome::Cancelled));
        let new = slot.begin().unwrap();
        assert!(!new.cancel.load(Ordering::SeqCst));
        assert_eq!(slot.outcome(), Some(Outcome::Running));
        slot.record(Outcome::Completed);
        drop(new);
        assert_eq!(slot.outcome(), Some(Outcome::Completed));
    }
    #[test]
    fn panic_releases_ownership_after_status_cleanup() {
        static ACTIVE: AtomicBool = AtomicBool::new(false);
        let slot = Slot::new();
        let _ = std::panic::catch_unwind(|| {
            let _run = slot.begin().unwrap().with_cleanup(&ACTIVE, || {});
            ACTIVE.store(true, Ordering::SeqCst);
            panic!("simulated worker failure");
        });
        assert!(!ACTIVE.load(Ordering::SeqCst));
        assert!(!slot.running());
        assert!(matches!(slot.outcome(), Some(Outcome::Failed(_))));
        assert!(slot.begin().is_some());
    }

    #[test]
    fn worker_exit_without_success_is_failure() {
        let slot = Slot::new();
        let run = slot.begin().unwrap();
        let token = run.cancel.clone();
        drop(run);
        assert!(token.load(Ordering::SeqCst));
        assert!(matches!(slot.outcome(), Some(Outcome::Failed(_))));
    }

    /// The point of the lease: a superseded run's exit cannot touch the run
    /// that replaced it (its flag, its cleanup, its outcome).
    #[test]
    fn a_superseded_run_cannot_flip_its_successor() {
        static FLAG: AtomicBool = AtomicBool::new(false);
        static CLEANED: AtomicU64 = AtomicU64::new(0);
        static SLOT: Slot = Slot::flagged(&FLAG);
        let old = SLOT.take_over().with_cleanup(&FLAG, || {
            CLEANED.fetch_add(1, Ordering::SeqCst);
        });
        assert!(FLAG.load(Ordering::SeqCst));
        let new = SLOT.take_over();
        assert!(old.cancel.load(Ordering::SeqCst), "superseded = cancelled");
        assert!(!old.live() && !old.current());
        assert!(new.live());
        old.lower_flag();
        old.record(Outcome::Completed);
        drop(old);
        assert!(FLAG.load(Ordering::SeqCst), "the new run still flies");
        assert_eq!(CLEANED.load(Ordering::SeqCst), 0);
        assert_eq!(SLOT.outcome(), Some(Outcome::Running));
        drop(new);
        assert!(!FLAG.load(Ordering::SeqCst));
    }

    /// A claim refuses a live run and supersedes one winding down.
    #[test]
    fn a_claim_refuses_a_live_run_and_replaces_a_dying_one() {
        static FLAG: AtomicBool = AtomicBool::new(false);
        static SLOT: Slot = Slot::flagged(&FLAG);
        let first = SLOT.claim().unwrap();
        assert!(SLOT.claim().is_none());
        assert!(SLOT.cancel(), "a live run was cancelled");
        assert!(!FLAG.load(Ordering::SeqCst), "the flag drops at the cancel");
        assert!(!SLOT.cancel(), "nothing live any more");
        let second = SLOT.claim().expect("a cancelled run is superseded");
        assert!(FLAG.load(Ordering::SeqCst));
        drop(first);
        assert!(FLAG.load(Ordering::SeqCst));
        second.lower_flag();
        assert!(!FLAG.load(Ordering::SeqCst));
        assert!(SLOT.running(), "still cleaning up");
        let third = SLOT.claim().expect("an ending run is superseded");
        drop(second);
        assert!(FLAG.load(Ordering::SeqCst));
        drop(third);
        assert!(!FLAG.load(Ordering::SeqCst));
    }
}
