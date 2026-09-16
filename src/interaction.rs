//! Deterministic crew interaction: explicit ownership, alert recovery and a
//! sampled landing summary. No writes to the simulator.
use std::time::{Duration, Instant};

pub fn route_continuation(flying: bool) -> &'static str {
    if flying {
        " Continuing the route."
    } else {
        ""
    }
}

/// A recovery is said only after an alert actually spoke and the whole family
/// has remained clear for five seconds (warning -> caution is NOT recovery).
#[derive(Default)]
pub struct Recovery {
    announced: bool,
    clear_since: Option<Instant>,
}
impl Recovery {
    pub fn tick(&mut self, bad: bool, spoke: bool, now: Instant) -> bool {
        self.announced |= spoke;
        if bad {
            self.clear_since = None;
            return false;
        }
        if !self.announced {
            return false;
        }
        let since = *self.clear_since.get_or_insert(now);
        if now.duration_since(since) < Duration::from_secs(5) {
            return false;
        }
        *self = Self::default();
        true
    }
}

/// Don't report an impact measurement from the filtered VSI. Keep the last
/// airborne sample, collect bounces, and wait for a slow, settled rollout.
#[derive(Default)]
pub struct Landing {
    airborne: bool,
    last_air: Option<(Instant, f64, f64)>, // time, AGL, VSI
    contact: Option<Instant>,
    sink: Option<f64>,
    bounces: u32,
    off_ground: bool,
    tracking: Option<f64>,
    automated: bool,
    interrupted: bool,
}
impl Landing {
    pub fn note_interruption(&mut self) {
        self.interrupted = true;
    }
    #[allow(clippy::too_many_arguments)]
    pub fn tick(
        &mut self,
        now: Instant,
        ground: bool,
        agl: f64,
        vsi: f64,
        speed: f64,
        automated: bool,
        xtk: Option<f64>,
    ) -> Option<String> {
        if !agl.is_finite() || !vsi.is_finite() || !speed.is_finite() {
            return None;
        }
        if !ground {
            if self.contact.is_some() {
                if agl > 50.0 {
                    *self = Self::default();
                }
                // go-around, not a bounce
                else if agl > 2.0 {
                    self.off_ground = true;
                }
            }
            if agl > 50.0 {
                self.airborne = true;
            }
            self.last_air = Some((now, agl, vsi));
            if agl < 100.0 {
                self.automated |= automated;
                if let Some(x) = xtk.filter(|x| x.is_finite()) {
                    self.tracking = Some(x.abs());
                }
            }
            return None;
        }
        if !self.airborne {
            return None;
        }
        if self.contact.is_none() {
            self.contact = Some(now);
            self.sink = self
                .last_air
                .filter(|(t, h, _)| now.duration_since(*t) <= Duration::from_secs(3) && *h < 30.0)
                .map(|(_, _, v)| (-v).max(0.0));
        } else if self.off_ground {
            self.bounces += 1;
            self.off_ground = false;
        }
        if speed > 20.0 || now.duration_since(self.contact.unwrap()) < Duration::from_secs(8) {
            return None;
        }
        let mut line = match self.sink {
            Some(s) => format!("Landing summary: last airborne sample about {:.0} feet per minute down; exact touchdown rate unavailable.", (s / 50.0).round() * 50.0),
            None => "Landing summary: touchdown rate unavailable.".into(),
        };
        if let Some(x) = self.tracking {
            line.push_str(&format!(
                " Last approach tracking {:.0} metres from centreline.",
                x.round()
            ));
        }
        if self.bounces > 0 {
            line.push_str(&format!(
                " {} possible bounce{} detected.",
                self.bounces,
                if self.bounces == 1 { "" } else { "s" }
            ));
        }
        if self.automated {
            line.push_str(" Autoland flew the final.");
        }
        if self.interrupted {
            line.push_str(" Flight automation was interrupted earlier.");
        }
        *self = Self::default();
        Some(line)
    }
}

/// Join only an existing route segment, toward its forward endpoint. No
/// nearest-fix jump across a loop or invented direct-to from far off route.
fn valid_position(lat: f64, lon: f64) -> bool {
    lat.is_finite() && lon.is_finite() && lat.abs() <= 90.0 && lon.abs() <= 180.0
}

/// Half-width of the rejoin corridor, nm: abeam a leg and inside this, the
/// follower joins it; outside, the refusal says which leg, which side and
/// the heading for its fix, so the pilot can close it and ask again (an
/// orbit over the Gulf of Lion left the Baron 5 nm right of DIBER-SOSUR and
/// the old refusal named no leg, 2026-09-16).
pub const REJOIN_CORRIDOR_NM: f64 = 3.0;

/// The abeam leg that fits the aircraft, as the index of its END fix.
/// `mag_var` (X-Plane's sign: magnetic = true - var) makes the refusal's
/// heading magnetic; `None` says it in true.
pub fn rejoin_index(
    legs: &[crate::nav::ProcLeg],
    lat: f64,
    lon: f64,
    mag_var: Option<f64>,
) -> Result<usize, String> {
    if !valid_position(lat, lon) {
        return Err("Position unavailable.".into());
    }
    // Every leg the aircraft is abeam of: (cross-track nm, end index,
    // right of course, true bearing to the end fix, names).
    let mut abeam: Vec<(f64, usize, bool, f64, String, String)> = Vec::new();
    for (i, pair) in legs.windows(2).enumerate() {
        let (Some(a), Some(b)) = (&pair[0].fix, &pair[1].fix) else {
            continue;
        };
        if !valid_position(a.lat, a.lon) || !valid_position(b.lat, b.lon) {
            return Err("The route contains an invalid position. I haven't taken control.".into());
        }
        let scale = lat.to_radians().cos();
        let (ax, ay) = ((a.lon - lon) * 60.0 * scale, (a.lat - lat) * 60.0);
        let (bx, by) = ((b.lon - lon) * 60.0 * scale, (b.lat - lat) * 60.0);
        let (dx, dy) = (bx - ax, by - ay);
        let length2 = dx * dx + dy * dy;
        if length2 < 0.0001 {
            continue;
        }
        let u = -(ax * dx + ay * dy) / length2;
        if !(0.0..1.0).contains(&u) {
            continue;
        }
        let distance = (ax + u * dx).hypot(ay + u * dy);
        // cross(leg direction, aircraft - A) < 0: the aircraft is right of course.
        let right = dx * (-ay) - dy * (-ax) < 0.0;
        let bearing = (bx.atan2(by).to_degrees() + 360.0) % 360.0;
        abeam.push((distance, i + 1, right, bearing, a.ident.clone(), b.ident.clone()));
    }
    abeam.sort_by(|a, b| a.0.total_cmp(&b.0));
    let Some((distance, index, right, bearing, from, to)) = abeam.first().cloned() else {
        return Err("We're not abeam any leg of the route. I haven't taken control. Request an approach or land for me.".into());
    };
    if distance > REJOIN_CORRIDOR_NM {
        let miles = distance.round().max(1.0);
        let heading = match mag_var {
            Some(v) => format!("{:03}", (((bearing - v) % 360.0 + 360.0) % 360.0).round() as i64 % 360),
            None => format!("{:03} true", bearing.round() as i64 % 360),
        };
        return Err(format!(
            "We're {miles:.0} mile{} {} of course on the {from} to {to} leg, outside the {REJOIN_CORRIDOR_NM:.0} mile rejoin corridor. I haven't taken control. Steer {heading} for {to} and ask again inside it.",
            if miles > 1.0 { "s" } else { "" },
            if right { "right" } else { "left" },
        ));
    }
    let candidates: Vec<(f64, usize)> = abeam
        .iter()
        .filter(|c| c.0 <= REJOIN_CORRIDOR_NM)
        .map(|c| (c.0, c.1))
        .collect();
    if candidates
        .iter()
        .skip(1)
        .any(|(d, i)| i.abs_diff(index) > 1 && *d - distance < 1.0)
    {
        return Err("More than one route leg fits our position. I haven't taken control; select the onward leg in the GPS and say follow the flight plan.".into());
    }
    Ok(index)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn legs(points: &[(f64, f64)]) -> Vec<crate::nav::ProcLeg> {
        let points: Vec<_> = points
            .iter()
            .enumerate()
            .map(|(i, (lat, lon))| crate::simbrief::Waypoint {
                ident: format!("FIX{i}"),
                lat: *lat,
                lon: *lon,
                alt: 6000.0,
            })
            .collect();
        crate::procedure::enroute_legs(&points, "ORIGIN", "DEST", 6000.0)
    }
    #[test]
    fn rejoin_chooses_forward_leg_and_refuses_off_route_or_ambiguous_crossing() {
        let straight = legs(&[(0.0, 0.0), (0.0, 1.0), (0.0, 2.0)]);
        assert_eq!(rejoin_index(&straight, 0.01, 1.4, None), Ok(2));
        assert!(rejoin_index(&straight, 1.0, 1.4, None).is_err());
        assert!(rejoin_index(&straight, 0.0, 2.1, None).is_err());
        assert!(rejoin_index(&straight, f64::NAN, 1.4, None).is_err());
        let invalid = legs(&[(0.0, 0.0), (91.0, 1.0)]);
        assert!(rejoin_index(&invalid, 0.0, 0.0, None).is_err());
        let crossing = legs(&[(-1.0, -1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, 1.0)]);
        assert!(rejoin_index(&crossing, 0.0, 0.0, None).is_err());
    }

    /// Outside the corridor the refusal names the leg, the side, the miles
    /// and the heading for the leg's end fix, so the pilot can close it.
    #[test]
    fn off_corridor_refusal_says_which_leg_which_side_and_the_heading() {
        let straight = legs(&[(0.0, 0.0), (0.0, 1.0), (0.0, 2.0)]);
        // 6 nm north of an eastbound leg is LEFT of course.
        let why = rejoin_index(&straight, 0.1, 1.4, Some(0.0)).unwrap_err();
        assert!(why.contains("6 miles left of course on the FIX1 to FIX2 leg"), "{why}");
        assert!(why.contains("Steer 099 for FIX2"), "{why}");
        // South is right of course; a 3 degree east variation moves the heading west.
        let why = rejoin_index(&straight, -0.1, 1.4, Some(3.0)).unwrap_err();
        assert!(why.contains("6 miles right of course"), "{why}");
        assert!(why.contains("Steer 078 for FIX2"), "{why}");
        // Without variation the heading is said as true.
        let why = rejoin_index(&straight, 0.1, 1.4, None).unwrap_err();
        assert!(why.contains("Steer 099 true"), "{why}");
        // Past the last fix there is no leg to name.
        let why = rejoin_index(&straight, 0.0, 2.1, None).unwrap_err();
        assert!(why.contains("not abeam any leg"), "{why}");
    }

    #[test]
    fn recovery_needs_a_spoken_alert_and_stable_clearance() {
        let t = Instant::now();
        let mut r = Recovery::default();
        assert!(!r.tick(false, false, t));
        assert!(!r.tick(true, true, t));
        assert!(!r.tick(false, false, t + Duration::from_secs(1)));
        assert!(!r.tick(true, false, t + Duration::from_secs(4)));
        assert!(!r.tick(false, false, t + Duration::from_secs(5)));
        assert!(r.tick(false, false, t + Duration::from_secs(10)));
        assert!(!r.tick(false, false, t + Duration::from_secs(20)));
    }
    #[test]
    fn landing_summary_is_sampled_delayed_and_once() {
        let t = Instant::now();
        let mut l = Landing::default();
        assert!(l.tick(t, true, 0.0, 0.0, 0.0, false, None).is_none());
        l.tick(t, false, 100.0, -500.0, 100.0, true, None);
        l.tick(
            t + Duration::from_secs(1),
            false,
            5.0,
            -446.0,
            87.0,
            true,
            Some(0.4),
        );
        assert!(l
            .tick(
                t + Duration::from_secs(2),
                true,
                0.0,
                -300.0,
                80.0,
                true,
                None
            )
            .is_none());
        l.tick(
            t + Duration::from_secs(3),
            false,
            3.0,
            100.0,
            70.0,
            true,
            None,
        );
        l.tick(
            t + Duration::from_secs(4),
            true,
            0.0,
            -100.0,
            60.0,
            true,
            None,
        );
        let line = l
            .tick(
                t + Duration::from_secs(11),
                true,
                0.0,
                0.0,
                10.0,
                false,
                None,
            )
            .unwrap();
        assert!(line.contains("450 feet"));
        assert!(line.contains("exact touchdown rate unavailable"));
        assert!(line.contains("1 possible bounce"));
        assert!(l
            .tick(
                t + Duration::from_secs(12),
                true,
                0.0,
                0.0,
                0.0,
                false,
                None
            )
            .is_none());
    }
    #[test]
    fn altitude_readback_only_promises_route_when_active() {
        assert_eq!(route_continuation(false), "");
        assert_eq!(route_continuation(true), " Continuing the route.");
    }
}
