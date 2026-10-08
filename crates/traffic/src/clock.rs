//! Sim time for the fleet: advances only while the sim runs, in fixed
//! substeps, so the same inputs give the same flight whatever the tick rate.

/// One substep of the fleet, seconds. Every law integrates at this step and
/// only this step: a recorded flight replays bit for bit.
pub const SUBSTEP_S: f64 = 0.1;

/// The most one tick may advance the clock. A long gap (a load, a debugger,
/// a stalled monitor) is time the fleet did not see; it must not arrive as
/// one giant step that teleports every aircraft.
pub const MAX_TICK_S: f64 = 0.5;

/// The fleet's clock. Fed either a delta ([`SimClock::advance`]) or the
/// sim's own running time ([`SimClock::observe`]), and answers how many
/// fixed substeps to run.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SimClock {
    /// Frozen (replay, pause, time acceleration above 1x): nothing advances
    /// and nothing accrues.
    pub frozen: bool,
    /// Seconds not yet spent on a whole substep.
    pending_s: f64,
    /// Substeps run since the clock began.
    steps: u64,
    /// The last sim running time observed, for [`SimClock::observe`].
    last_sim_s: Option<f64>,
}

impl SimClock {
    pub fn new() -> Self {
        Self::default()
    }

    /// Advance by `dt` seconds, clamped to 0..[`MAX_TICK_S`]; returns the
    /// number of whole [`SUBSTEP_S`] substeps now owed. A frozen clock
    /// returns 0 and banks nothing.
    pub fn advance(&mut self, dt: f64) -> u32 {
        if self.frozen || !dt.is_finite() {
            return 0;
        }
        self.pending_s += dt.clamp(0.0, MAX_TICK_S);
        // A hair of tolerance so 0.1 + 0.1 + 0.1 is three steps, not two.
        let n = ((self.pending_s + 1e-9) / SUBSTEP_S).floor();
        self.pending_s = (self.pending_s - n * SUBSTEP_S).max(0.0);
        self.steps += n as u64;
        n as u32
    }

    /// Advance from the sim's running time (`sim/time/total_running_time_sec`):
    /// the delta since the last observation, through [`SimClock::advance`].
    /// The first observation only sets the origin; a time that goes
    /// backwards (a new flight) re-seats it and advances nothing.
    pub fn observe(&mut self, sim_s: f64) -> u32 {
        let last = self.last_sim_s.replace(sim_s);
        match last {
            Some(prev) if sim_s >= prev => self.advance(sim_s - prev),
            _ => 0,
        }
    }

    /// Seconds of fleet time run so far (whole substeps).
    pub fn now_s(&self) -> f64 {
        self.steps as f64 * SUBSTEP_S
    }

    /// Substeps run so far.
    pub fn steps(&self) -> u64 {
        self.steps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tick_is_spent_in_whole_substeps() {
        let mut c = SimClock::new();
        assert_eq!(c.advance(0.25), 2);
        assert_eq!(c.advance(0.05), 1, "the banked 0.05 makes a third");
        assert_eq!(c.advance(0.1), 1);
        assert!((c.now_s() - 0.4).abs() < 1e-12);
    }

    #[test]
    fn a_long_gap_is_clamped_and_a_negative_one_is_nothing() {
        let mut c = SimClock::new();
        assert_eq!(c.advance(30.0), 5, "half a second at most");
        assert_eq!(c.advance(-3.0), 0);
        assert_eq!(c.advance(f64::NAN), 0);
        assert_eq!(c.steps(), 5);
    }

    #[test]
    fn a_frozen_clock_banks_nothing() {
        let mut c = SimClock::new();
        c.frozen = true;
        assert_eq!(c.advance(0.4), 0);
        c.frozen = false;
        assert_eq!(c.advance(0.05), 0, "the frozen 0.4 was never banked");
        assert_eq!(c.advance(0.05), 1);
    }

    #[test]
    fn the_sim_running_time_drives_it() {
        let mut c = SimClock::new();
        assert_eq!(c.observe(100.0), 0, "the first sight is the origin");
        assert_eq!(c.observe(100.3), 3);
        assert_eq!(c.observe(5.0), 0, "a new flight re-seats the origin");
        assert_eq!(c.observe(5.2), 2);
    }
}
