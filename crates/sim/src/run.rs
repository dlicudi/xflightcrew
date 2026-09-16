//! Worker ownership survives cancellation until the worker and its I/O exit.
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
}

pub struct Slot {
    state: Mutex<Option<State>>,
    next: AtomicU64,
    last: Mutex<Option<(u64, Outcome)>>,
}
impl Slot {
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(None),
            next: AtomicU64::new(1),
            last: Mutex::new(None),
        }
    }
    /// A cancelled worker still owns its slot until it has returned.
    pub fn begin(&self) -> Option<Lease<'_>> {
        let mut state = self.state.lock().unwrap();
        if state.is_some() {
            return None;
        }
        let id = self.next.fetch_add(1, Ordering::SeqCst);
        let cancel = Arc::new(AtomicBool::new(false));
        *state = Some(State {
            id,
            cancel: cancel.clone(),
            outcome: Outcome::Running,
        });
        Some(Lease {
            slot: self,
            id,
            cancel,
            cleanup: None,
        })
    }
    pub fn cancel(&self) {
        if let Some(s) = self.state.lock().unwrap().as_mut() {
            s.cancel.store(true, Ordering::SeqCst);
            s.outcome = Outcome::Cancelled;
        }
    }
    pub fn fail(&self, reason: &str) {
        if let Some(s) = self.state.lock().unwrap().as_mut() {
            if !s.cancel.load(Ordering::SeqCst) {
                s.outcome = Outcome::Failed(reason.into());
                s.cancel.store(true, Ordering::SeqCst);
            }
        }
    }
    pub fn running(&self) -> bool {
        self.state.lock().unwrap().is_some()
    }
    pub fn record(&self, outcome: Outcome) {
        if let Some(s) = self.state.lock().unwrap().as_mut() {
            if !s.cancel.load(Ordering::SeqCst) {
                s.outcome = outcome;
            }
        }
    }
    pub fn outcome(&self) -> Option<Outcome> {
        let state = self.state.lock().unwrap();
        if state.is_some() {
            return Some(Outcome::Running);
        }
        self.last.lock().unwrap().as_ref().map(|(_, o)| o.clone())
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
}
impl Drop for Lease<'_> {
    fn drop(&mut self) {
        if let Some((active, cleanup)) = self.cleanup {
            active.store(false, Ordering::SeqCst);
            cleanup();
        }
        let mut state = self.slot.state.lock().unwrap();
        if state.as_ref().is_some_and(|s| s.id == self.id) {
            let s = state.take().unwrap();
            let outcome = match s.outcome {
                Outcome::Running => Outcome::Failed("worker exited without completion".into()),
                o => o,
            };
            if !matches!(outcome, Outcome::Completed | Outcome::HandedOff) {
                s.cancel.store(true, Ordering::SeqCst);
            }
            *self.slot.last.lock().unwrap() = Some((s.id, outcome));
        }
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
}
