//! `Scoped<T>`: a process-wide cell stamped with the generation of the
//! lifetime its value belongs to (refactor audit 2026-10-04, phase 3 item 2).
//!
//! The doors in `flight_state` forget a flight's facts when a new one begins,
//! but a door cannot stop a write that lands AFTER it: a stood-down autoland
//! is cancelled at its next tick, not at the door, and a tick already past
//! its check writes the old flight's landing runway into the new flight. A
//! scoped cell remembers which flight wrote it, reads as `None` once that
//! flight has ended, and refuses a write stamped with a flight that is over
//! ([`Scoped::set_in`], the generation a loop was launched in), so a late
//! write is born dead and never clobbers the new flight's value.
//!
//! For a FEW statics the audit named as leaks, not a mass conversion: each
//! still has its door; this is the guard behind it.

/// A value that lives as long as one generation of its lifetime.
pub(crate) struct Scoped<T> {
    /// The lifetime's current generation.
    epoch: fn() -> u64,
    cell: std::sync::Mutex<Option<(u64, T)>>,
}

impl<T> Scoped<T> {
    /// A cell living one generation of `epoch`.
    pub(crate) const fn new(epoch: fn() -> u64) -> Self {
        Scoped {
            epoch,
            cell: std::sync::Mutex::new(None),
        }
    }

    /// A cell living one FLIGHT (`flight_state::flight_gen`: every
    /// `flight_begins` other than a reposition, and every `airframe_begins`).
    pub(crate) const fn flight() -> Self {
        Self::new(crate::flight_state::flight_gen)
    }

    /// The generation now: what a loop takes at its launch for [`Self::set_in`].
    pub(crate) fn now(&self) -> u64 {
        (self.epoch)()
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Option<(u64, T)>> {
        self.cell.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Set it for the generation now (tests; a loop writes with [`Self::set_in`],
    /// stamped with the generation it was launched in).
    #[cfg(test)]
    pub(crate) fn set(&self, v: T) {
        let mut g = self.lock();
        *g = Some((self.now(), v));
    }

    /// Set it for `generation`: dropped, leaving the cell alone, when that
    /// generation is over (a late write from a loop of the last flight).
    /// Whether it was kept.
    pub(crate) fn set_in(&self, generation: u64, v: T) -> bool {
        let mut g = self.lock();
        if generation != self.now() {
            return false;
        }
        *g = Some((generation, v));
        true
    }

    /// Forget it.
    pub(crate) fn clear(&self) {
        *self.lock() = None;
    }

    /// Take it, if it is this generation's.
    pub(crate) fn take(&self) -> Option<T> {
        let now = self.now();
        self.lock().take().filter(|(g, _)| *g == now).map(|(_, v)| v)
    }
}

impl<T: Clone> Scoped<T> {
    /// The value, if it is this generation's.
    pub(crate) fn get(&self) -> Option<T> {
        let now = self.now();
        match self.lock().as_ref() {
            Some((g, v)) if *g == now => Some(v.clone()),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Scoped;
    use std::sync::atomic::{AtomicU64, Ordering};

    // A private epoch per test, so no test moves the app's flight.
    static EPOCH_A: AtomicU64 = AtomicU64::new(0);
    fn epoch_a() -> u64 {
        EPOCH_A.load(Ordering::SeqCst)
    }
    static EPOCH_B: AtomicU64 = AtomicU64::new(0);
    fn epoch_b() -> u64 {
        EPOCH_B.load(Ordering::SeqCst)
    }

    #[test]
    fn a_value_dies_with_its_generation() {
        static CELL: Scoped<String> = Scoped::new(epoch_a);
        CELL.set("13".into());
        assert_eq!(CELL.get().as_deref(), Some("13"));
        // A new flight begins, and no door cleared the cell.
        EPOCH_A.fetch_add(1, Ordering::SeqCst);
        assert_eq!(CELL.get(), None);
        assert_eq!(CELL.take(), None);
        CELL.set("31".into());
        assert_eq!(CELL.take().as_deref(), Some("31"));
        assert_eq!(CELL.get(), None, "taken");
    }

    /// The leak the doors cannot close: a stood-down autoland is cancelled
    /// at its next tick, so a tick already past its check writes the old
    /// flight's landed runway AFTER `forget_flight` ran, and the new flight's
    /// vacate read it (refactor audit 2026-10-04, medium leaks). Stamped
    /// with the launch's generation, the write is born dead and the new
    /// flight's own value stands.
    #[test]
    fn a_late_write_from_the_last_flight_is_born_dead() {
        static CELL: Scoped<String> = Scoped::new(epoch_b);
        let launched = CELL.now();
        assert!(CELL.set_in(launched, "13".into()));
        // The door: a new flight begins and forgets.
        EPOCH_B.fetch_add(1, Ordering::SeqCst);
        CELL.clear();
        CELL.set("31".into());
        // The old loop's tick lands now.
        assert!(!CELL.set_in(launched, "13".into()));
        assert_eq!(CELL.get().as_deref(), Some("31"));
        // And with nothing of the new flight's, it still reads nothing.
        CELL.clear();
        assert!(!CELL.set_in(launched, "13".into()));
        assert_eq!(CELL.get(), None);
    }
}
