//! A visual final flown as a pilot flies it on the autopilot: the vertical
//! speed SET ONCE to the path's rate for the groundspeed, and NUDGED by 100
//! fpm when the aircraft is clearly off the path, no oftener than every
//! [`NUDGE_EVERY_S`] (docs/tasks/autonomy-layers). The dial was rewritten by
//! the path law every tick: 56 times in four minutes on LEMG 12's visual
//! final, 270 times in eight on the old ILS approach (2026-10-11/12).

/// Off the path by more than this, ft, is a nudge.
pub const NUDGE_BAND_FT: f64 = 40.0;
/// ...and back within this, ft, the dial returns toward the path's rate.
const BACK_ON_FT: f64 = 15.0;
/// One nudge, fpm.
pub const NUDGE_FPM: f64 = 100.0;
/// No oftener than this, seconds.
pub const NUDGE_EVERY_S: f64 = 10.0;
/// The most the dial strays from the path's rate, fpm.
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
        let set = round50(path_fpm);
        let Some(dial) = self.dial else {
            self.dial = Some(set);
            self.last_s = t_s;
            return set;
        };
        if t_s - self.last_s < NUDGE_EVERY_S {
            return dial;
        }
        let want = if above_ft > NUDGE_BAND_FT {
            dial - NUDGE_FPM
        } else if above_ft < -NUDGE_BAND_FT {
            dial + NUDGE_FPM
        } else if above_ft.abs() < BACK_ON_FT && (dial - set).abs() >= NUDGE_FPM / 2.0 {
            dial + NUDGE_FPM * (set - dial).signum()
        } else {
            dial
        };
        let want = want.clamp(set - MAX_OFF_PATH_FPM, (set + MAX_OFF_PATH_FPM).min(0.0));
        if (want - dial).abs() > 1.0 {
            self.dial = Some(want);
            self.last_s = t_s;
        }
        self.dial.unwrap_or(set)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_once_on_the_path() {
        let mut n = PathNudge::default();
        let d0 = n.step(-338.0, 0.0, 0.0);
        assert_eq!(d0, -350.0);
        for k in 1..60 {
            assert_eq!(n.step(-340.0 + (k % 3) as f64, 10.0, k as f64), -350.0, "no change on the path");
        }
    }

    #[test]
    fn nudged_when_off_no_oftener_than_every_ten_seconds_and_back() {
        let mut n = PathNudge::default();
        n.step(-350.0, 0.0, 0.0);
        assert_eq!(n.step(-350.0, 60.0, 5.0), -350.0, "too soon");
        assert_eq!(n.step(-350.0, 60.0, 10.0), -450.0, "high: steeper");
        assert_eq!(n.step(-350.0, 50.0, 15.0), -450.0, "too soon again");
        assert_eq!(n.step(-350.0, 45.0, 20.0), -550.0);
        assert_eq!(n.step(-350.0, 10.0, 30.0), -450.0, "back on: toward the path's rate");
        assert_eq!(n.step(-350.0, 5.0, 40.0), -350.0);
        assert_eq!(n.step(-350.0, -60.0, 50.0), -250.0, "low: shallower");
    }

    #[test]
    fn never_strays_far_nor_climbs() {
        let mut n = PathNudge::default();
        n.step(-200.0, 0.0, 0.0);
        let mut d = 0.0;
        for k in 1..20 {
            d = n.step(-200.0, -200.0, k as f64 * 10.0);
        }
        assert_eq!(d, -0.0_f64.min(-200.0 + 300.0).min(0.0), "capped at level, never a climb");
        let mut m = PathNudge::default();
        m.step(-350.0, 0.0, 0.0);
        for k in 1..20 {
            d = m.step(-350.0, 200.0, k as f64 * 10.0);
        }
        assert_eq!(d, -650.0, "at most 300 under the path's rate");
    }
}
