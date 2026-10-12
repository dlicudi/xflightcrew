//! A visual final flown as a pilot flies it on the autopilot: the vertical
//! speed SET to rejoin the path and re-set only when that answer changes, no
//! oftener than every [`NUDGE_EVERY_S`] (docs/tasks/autonomy-layers). The dial
//! was rewritten by the path law every tick: 56 times in four minutes on
//! LEMG 12's visual final, 270 times in eight on the old ILS approach
//! (2026-10-11/12).
//!
//! The rate is the path's own plus the height off it closed in [`REJOIN_S`]:
//! a pilot 300 ft low levels off and lets the path come down to him. Nudged
//! 50 fpm every 10 s instead, and never shallower than the path less 300, the
//! SR22 and the RV-10 stayed 150 to 300 ft low from an air start's dive, fast,
//! no landing flap, and went round (LEMG 13, 1791810002, 1791804448).

/// The height off the path is closed in this long, seconds.
pub const REJOIN_S: f64 = 30.0;
/// No oftener than this, seconds.
pub const NUDGE_EVERY_S: f64 = 10.0;
/// The most the dial goes steeper than the path's rate, fpm (high: never a
/// dive to rejoin). Low, it may go as far as level, never a climb.
const MAX_OFF_PATH_FPM: f64 = 300.0;

/// The dial as set, and when it last moved.
#[derive(Debug, Clone, Copy, Default)]
pub struct PathNudge {
    dial: Option<f64>,
    last_s: f64,
}

fn round50(fpm: f64) -> f64 {
    (fpm / 50.0).round() * 50.0
}

impl PathNudge {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// The VS dial for this tick, fpm: `path_fpm` the path's own rate at the
    /// groundspeed (negative), `above_ft` the height over the path, `t_s` a
    /// clock in seconds.
    pub fn step(&mut self, path_fpm: f64, above_ft: f64, t_s: f64) -> f64 {
        let want = round50(path_fpm - above_ft / REJOIN_S * 60.0)
            .clamp(round50(path_fpm - MAX_OFF_PATH_FPM), 0.0);
        match self.dial {
            Some(dial) if want == dial || t_s - self.last_s < NUDGE_EVERY_S => dial,
            _ => {
                self.dial = Some(want);
                self.last_s = t_s;
                want
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_once_on_the_path() {
        let mut n = PathNudge::default();
        assert_eq!(n.step(-338.0, 0.0, 0.0), -350.0);
        for k in 1..60 {
            assert_eq!(n.step(-340.0 + (k % 3) as f64, 5.0, k as f64), -350.0, "no change on the path");
        }
    }

    /// The SR22 from its air start: 270 ft low on a -450 path. Level until
    /// the path comes down, re-set no oftener than every ten seconds, and
    /// back on the path's own rate as it rejoins.
    #[test]
    fn low_it_levels_and_rejoins() {
        let mut n = PathNudge::default();
        assert_eq!(n.step(-450.0, -270.0, 0.0), 0.0, "300 ft low: level");
        assert_eq!(n.step(-450.0, -120.0, 5.0), 0.0, "too soon to re-set");
        assert_eq!(n.step(-450.0, -120.0, 10.0), -200.0);
        assert_eq!(n.step(-450.0, -20.0, 20.0), -400.0);
        assert_eq!(n.step(-450.0, 0.0, 30.0), -450.0, "on the path: its own rate");
    }

    #[test]
    fn high_it_steepens_but_never_dives_nor_climbs() {
        let mut n = PathNudge::default();
        assert_eq!(n.step(-350.0, 300.0, 0.0), -650.0, "at most 300 under the path's rate");
        let mut m = PathNudge::default();
        assert_eq!(m.step(-200.0, -500.0, 0.0), 0.0, "never a climb");
    }
}
