//! The drive's traffic guard: stop behind an aircraft on the route ahead,
//! never drive into it.
//!
//! The pilot's screenshot of 2026-10-10: our own auto taxi drove straight
//! into an AI airliner standing at a runway holding point, the two drawn
//! inside each other. The drive never read the traffic picture; our own
//! traffic gives way to us (it treats our aircraft as OWN), but a STANDING
//! aircraft cannot. So every tick the route ahead is searched for an
//! on-ground target inside a corridor about it, and the nearest one along
//! the route caps the speed so we stop a taxi gap behind it, on the route,
//! never steering round it. A target about to cross the route ahead holds
//! us the same way until it is through. No picture (traffic off, a failed
//! read) is no guard: exactly the drive as it was.
//!
//! Pure; the loop feeds it the picture ([`crate::world::targets`], the ONE
//! traffic picture: our own fleet while it flies, else the sim's TCAS) and
//! writes the cap through the governor like every other schedule cap.

use super::*;

/// A target's wingspan when we cannot know it, m: the TCAS picture carries
/// no type, and an A320 or a 737 (34 to 36 m) is the aircraft a holding
/// point most often has on it. Conservative for anything smaller.
pub const TRAFFIC_SPAN_DEFAULT_M: f64 = 36.0;
/// ...and its half length, centre to tail, m (an A320 is 37.6 m long, a
/// 737-800 39.5 m).
pub const TRAFFIC_HALF_LEN_DEFAULT_M: f64 = 20.0;
/// Wingtip margin on top of both half spans, m. Small on purpose: code C
/// parallel taxiways are 44 m apart and two 36 m spans plus this still
/// clears them, so a pass on the next taxiway is no hold.
pub const CORRIDOR_MARGIN_M: f64 = 3.0;
/// The shortest look ahead along the route, m.
pub const LOOK_MIN_M: f64 = 150.0;
/// A moving target is projected this far ahead in time to see it cross.
pub const CROSS_PREDICT_S: f64 = 8.0;
/// Slower than this, m/s, a target is standing (its own jitter is no
/// crossing).
pub const CROSS_MIN_MPS: f64 = 1.5;
/// Behind the aircraft the route is searched this far, m, only so a target
/// beside or behind our centre projects there and is dropped as behind.
const BEHIND_M: f64 = 60.0;
/// A hold lets go once the stop point is this far ahead again, m: a leader
/// edging forward a metre must not start us creeping and stopping.
pub const HOLD_RELEASE_M: f64 = 10.0;
/// The governor's target while held for traffic: under 1 kt (idle and
/// brakes, `speedhold::taxi_control`) but never 0.0, which is what a spoken
/// "stop" writes. Held on exactly 0.0, a "stop" said during the hold would
/// read as our own schedule, and the drive would roll off when the traffic
/// cleared against the pilot's word.
pub const TRAFFIC_HOLD_KTS: f64 = 0.05;
/// A hold this long is said again (and blackboxed), and still held.
pub const TRAFFIC_LONG_S: f64 = 600.0;
/// "Traffic clear." only after a hold this long; a short one resumes in
/// silence.
pub const CLEAR_SAID_AFTER_S: f64 = 20.0;
/// How often the loop reads the picture, s: the sim's TCAS table is five
/// Web API reads (frame time), our own fleet's picture is published once a
/// monitor tick anyway.
pub const TRAFFIC_READ_S: f64 = 1.0;

/// Below this cap, kt, the guard is what keeps us slow: no stall, no lost
/// progress.
pub const TRAFFIC_EXCUSES_BELOW_KTS: f64 = 2.0;

/// Our own size for the guard, from the maximum takeoff mass (the drive
/// reads it already for its join turns).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OwnSize {
    pub half_span_m: f64,
    pub half_len_m: f64,
    /// Nose to the leader's tail when stopped, m: a real taxi gap.
    pub gap_m: f64,
}

impl OwnSize {
    /// Light (12 m span), regional (25 m), narrowbody (36 m), heavy (65 m);
    /// unread is the narrowbody. Airliners stop 18 m behind, light aircraft
    /// 8 m.
    pub fn from_mtow(mtow_kg: Option<f64>) -> OwnSize {
        let span = match mtow_kg {
            Some(m) if m <= 5_700.0 => 12.0,
            Some(m) if m <= 25_000.0 => 25.0,
            Some(m) if m > 100_000.0 => 65.0,
            _ => 36.0,
        };
        OwnSize {
            half_span_m: span / 2.0,
            half_len_m: span / 2.0,
            gap_m: if span >= 24.0 { 18.0 } else { 8.0 },
        }
    }

    /// Half the corridor about the route a target's centre must be outside.
    pub fn corridor_half_m(&self) -> f64 {
        self.half_span_m + TRAFFIC_SPAN_DEFAULT_M / 2.0 + CORRIDOR_MARGIN_M
    }
}

/// One on-ground target as the guard sees it: where, and how it moves
/// (m/s east and north; zero until it has been seen twice).
#[derive(Debug, Clone, PartialEq)]
pub struct Seen {
    pub callsign: String,
    pub lat: f64,
    pub lon: f64,
    pub v_east: f64,
    pub v_north: f64,
}

/// The targets between reads, for their velocities.
#[derive(Debug, Default)]
pub struct TrafficTrack {
    last: std::collections::HashMap<String, (f64, f64, f64)>,
}

impl TrafficTrack {
    /// The on-ground targets of a fresh picture read at `t_s`, with each
    /// one's velocity from its last reading. Airborne targets are dropped:
    /// a taxi stops for nothing in the air.
    pub fn update(&mut self, picture: &[crate::world::TcasTarget], t_s: f64) -> Vec<Seen> {
        let mut next = std::collections::HashMap::new();
        let seen = picture
            .iter()
            .filter(|t| t.on_ground && !t.callsign.is_empty())
            .map(|t| {
                let (mut ve, mut vn) = (0.0, 0.0);
                if let Some(&(la, lo, t0)) = self.last.get(&t.callsign) {
                    let dt = t_s - t0;
                    if (0.3..=10.0).contains(&dt) {
                        let (e, n) = taxi_guide::Geo::new(la).to_m((la, lo), (t.lat, t.lon));
                        (ve, vn) = (e / dt, n / dt);
                    }
                }
                next.insert(t.callsign.clone(), (t.lat, t.lon, t_s));
                Seen {
                    callsign: t.callsign.clone(),
                    lat: t.lat,
                    lon: t.lon,
                    v_east: ve,
                    v_north: vn,
                }
            })
            .collect();
        self.last = next;
        seen
    }
}

/// The nearest target on the route ahead.
#[derive(Debug, Clone, PartialEq)]
pub struct Ahead {
    pub callsign: String,
    /// Along the route, our centre to theirs, m.
    pub along_m: f64,
    /// Our nose to their tail, m.
    pub gap_m: f64,
    /// How far we may still roll before the taxi gap is reached, m.
    pub stop_in_m: f64,
    /// Not on the route yet, crossing onto it.
    pub crossing: bool,
}

/// How far ahead the route is searched: twice the stop at the drive's
/// own stop deceleration, never under [`LOOK_MIN_M`].
pub fn look_ahead_m(gs_ms: f64) -> f64 {
    let stop = gs_ms * gs_ms / (2.0 * STOP_DECEL_MS2);
    (2.0 * stop).max(LOOK_MIN_M)
}

/// A point (route frame, m) projected onto the route between `from` and
/// `to` along it: (along m, distance off m). Nearest wins; past the
/// route's end the distance is to the end itself.
fn project_window(
    pts: &[(f64, f64)],
    cum: &[f64],
    from: f64,
    to: f64,
    p: (f64, f64),
) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    for i in 0..pts.len().saturating_sub(1) {
        let (c0, c1) = (cum[i], cum[i + 1]);
        let (a, b) = (from.max(c0), to.min(c1));
        let len = c1 - c0;
        if a > b || len <= 1e-6 {
            continue;
        }
        let (d_x, d_y) = (
            (pts[i + 1].0 - pts[i].0) / len,
            (pts[i + 1].1 - pts[i].1) / len,
        );
        let s = ((p.0 - pts[i].0) * d_x + (p.1 - pts[i].1) * d_y + c0).clamp(a, b);
        let q = (pts[i].0 + d_x * (s - c0), pts[i].1 + d_y * (s - c0));
        let d = taxi_guide::dist(p, q);
        if best.is_none_or(|(_, bd)| d < bd) {
            best = Some((s, d));
        }
    }
    best
}

impl Route {
    /// The nearest on-ground target inside the corridor on the route ahead
    /// of `progress`, or crossing into it within [`CROSS_PREDICT_S`]. Pure.
    pub fn traffic_ahead(
        &self,
        progress: f64,
        gs_ms: f64,
        own: &OwnSize,
        seen: &[Seen],
    ) -> Option<Ahead> {
        let look = look_ahead_m(gs_ms);
        let half = own.corridor_half_m();
        let on_path = |p: (f64, f64)| {
            project_window(
                &self.pts,
                &self.cum,
                progress - BEHIND_M,
                progress + look,
                p,
            )
            .filter(|&(s, d)| d <= half && s > progress)
            .map(|(s, _)| s - progress)
        };
        seen.iter()
            .filter_map(|t| {
                let xy = self.local_m(t.lat, t.lon);
                if let Some(along) = on_path(xy) {
                    return Some((t, along, false));
                }
                if t.v_east.hypot(t.v_north) < CROSS_MIN_MPS {
                    return None;
                }
                let ahead = (
                    xy.0 + t.v_east * CROSS_PREDICT_S,
                    xy.1 + t.v_north * CROSS_PREDICT_S,
                );
                on_path(ahead).map(|along| (t, along, true))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(t, along, crossing)| {
                let gap = along - own.half_len_m - TRAFFIC_HALF_LEN_DEFAULT_M;
                Ahead {
                    callsign: t.callsign.clone(),
                    along_m: along,
                    gap_m: gap,
                    stop_in_m: gap - own.gap_m,
                    crossing,
                }
            })
    }
}

/// The speed that stops us at the taxi gap: the drive's own stop ramp
/// ([`stop_ramp_kts`]) to the stop point, zero inside it.
pub fn traffic_cap_kts(ahead: &Ahead) -> f64 {
    stop_ramp_kts(ahead.stop_in_m)
}

/// What a hold step reports for the voice and the blackbox.
#[derive(Debug, Clone, PartialEq)]
pub enum HoldEvent {
    Began {
        callsign: String,
        gap_m: f64,
        along_m: f64,
        crossing: bool,
    },
    Long {
        callsign: String,
        held_s: f64,
    },
    Cleared {
        callsign: String,
        held_s: f64,
    },
}

/// The hold, pure: when it begins, how long it has lasted, when it lets go.
#[derive(Debug, Default)]
pub struct TrafficHold {
    held: Option<(String, f64, bool)>,
}

impl TrafficHold {
    pub fn holding(&self) -> bool {
        self.held.is_some()
    }

    /// The tick's cap (`None`: no traffic in the way) and what to say. A
    /// hold begins when the stop point is reached and lasts until it is
    /// [`HOLD_RELEASE_M`] ahead again or the route is clear, however long
    /// that takes: it is never given up into the traffic.
    pub fn step(&mut self, ahead: Option<&Ahead>, t_s: f64) -> (Option<f64>, Option<HoldEvent>) {
        if let Some((callsign, since, long_said)) = self.held.as_mut() {
            if let Some(a) = ahead.filter(|a| a.stop_in_m < HOLD_RELEASE_M) {
                let held_s = t_s - *since;
                if !*long_said && held_s >= TRAFFIC_LONG_S {
                    *long_said = true;
                    return (
                        Some(TRAFFIC_HOLD_KTS),
                        Some(HoldEvent::Long {
                            callsign: a.callsign.clone(),
                            held_s,
                        }),
                    );
                }
                return (Some(TRAFFIC_HOLD_KTS), None);
            }
            let ev = HoldEvent::Cleared {
                callsign: callsign.clone(),
                held_s: t_s - *since,
            };
            self.held = None;
            return (ahead.map(traffic_cap_kts), Some(ev));
        }
        let Some(a) = ahead else {
            return (None, None);
        };
        let cap = traffic_cap_kts(a);
        if cap > 0.0 {
            return (Some(cap), None);
        }
        self.held = Some((a.callsign.clone(), t_s, false));
        (
            Some(TRAFFIC_HOLD_KTS),
            Some(HoldEvent::Began {
                callsign: a.callsign.clone(),
                gap_m: a.gap_m,
                along_m: a.along_m,
                crossing: a.crossing,
            }),
        )
    }
}

/// What the crew says for a hold event, if anything.
pub fn hold_line(ev: &HoldEvent) -> Option<&'static str> {
    match ev {
        HoldEvent::Began { .. } => Some("Holding for traffic ahead."),
        HoldEvent::Long { .. } => {
            Some("Still holding for the traffic ahead, ten minutes now. I'll stay stopped until it's clear.")
        }
        HoldEvent::Cleared { held_s, .. } if *held_s > CLEAR_SAID_AFTER_S => Some("Traffic clear."),
        HoldEvent::Cleared { .. } => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LAT0: f64 = 36.0;
    const LON0: f64 = -5.0;
    fn ll(x_m: f64, y_m: f64) -> (f64, f64) {
        (
            LAT0 + y_m / 111_320.0,
            LON0 + x_m / (111_320.0 * LAT0.to_radians().cos()),
        )
    }
    /// North 400 m to a holding point.
    fn route() -> Route {
        Route::build(&TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(0.0, 400.0)],
            via: String::new(),
            seg: vec![(String::new(), false)],
            holding_point: "A1".into(),
            lead_on: Vec::new(),
        })
        .expect("a route")
    }
    fn at(cs: &str, x: f64, y: f64) -> Seen {
        let (lat, lon) = ll(x, y);
        Seen {
            callsign: cs.into(),
            lat,
            lon,
            v_east: 0.0,
            v_north: 0.0,
        }
    }
    fn jet() -> OwnSize {
        OwnSize::from_mtow(Some(79_000.0))
    }

    #[test]
    fn a_target_on_the_path_ahead_is_picked_with_its_gap() {
        let a = route()
            .traffic_ahead(0.0, 4.0, &jet(), &[at("EZY12", 2.0, 120.0)])
            .expect("ahead");
        assert!((a.along_m - 120.0).abs() < 1.0, "{a:?}");
        // 120 m centre to centre less 18 m of us and 20 m of them.
        assert!((a.gap_m - 82.0).abs() < 1.0, "{a:?}");
        assert!((a.stop_in_m - 64.0).abs() < 1.0, "{a:?}");
        assert!(!a.crossing);
    }

    #[test]
    fn beside_the_path_behind_us_or_beyond_the_look_is_no_traffic() {
        let r = route();
        let own = jet();
        // A parallel taxiway 44 m off (code C): clear of 18 + 18 + 3.
        assert!(r
            .traffic_ahead(0.0, 4.0, &own, &[at("A", 44.0, 100.0)])
            .is_none());
        // Behind us on the route.
        assert!(r
            .traffic_ahead(100.0, 4.0, &own, &[at("B", 0.0, 60.0)])
            .is_none());
        // Ahead but past the look (150 m at a walking pace).
        assert!(r
            .traffic_ahead(0.0, 2.0, &own, &[at("C", 0.0, 300.0)])
            .is_none());
        // ...which grows with speed: 15 kt stops in 119 m, looked at twice.
        assert!(r
            .traffic_ahead(0.0, 7.7, &own, &[at("C", 0.0, 230.0)])
            .is_some());
        // Airborne traffic never reaches the guard.
        let mut track = TrafficTrack::default();
        let (lat, lon) = ll(0.0, 100.0);
        let air = crate::world::TcasTarget {
            callsign: "D".into(),
            lat,
            lon,
            ele_ft: 500.0,
            on_ground: false,
            vs_fpm: 0.0,
        };
        assert!(track.update(&[air], 0.0).is_empty());
    }

    #[test]
    fn the_screenshot_an_airliner_already_at_the_hold_is_stopped_behind() {
        // Our route ends at the holding point (400 m); the AI airliner is
        // standing ON it. We stop the taxi gap behind its tail, short of
        // the hold, never on it.
        let r = route();
        let own = jet();
        let a = r
            .traffic_ahead(250.0, 4.0, &own, &[at("RYR4K", 0.0, 400.0)])
            .expect("at the hold");
        let stop_at = 250.0 + a.stop_in_m;
        assert!(
            (stop_at - (400.0 - 20.0 - 18.0 - 18.0)).abs() < 1.0,
            "stops at {stop_at:.0}"
        );
        // A little past the hold point (its nose on the line) still counts.
        assert!(r
            .traffic_ahead(250.0, 4.0, &own, &[at("RYR4K", 0.0, 415.0)])
            .is_some());
    }

    #[test]
    fn the_nearest_of_several_sets_the_cap() {
        let r = route();
        let seen = [
            at("FAR", 0.0, 140.0),
            at("NEAR", -3.0, 90.0),
            at("OFF", 60.0, 50.0),
        ];
        assert_eq!(
            r.traffic_ahead(0.0, 4.0, &jet(), &seen)
                .expect("ahead")
                .callsign,
            "NEAR"
        );
    }

    #[test]
    fn a_target_crossing_ahead_holds_us_and_one_moving_away_does_not() {
        let r = route();
        let own = jet();
        // 60 m left of the route at 100 m, crossing right at 6 m/s: inside
        // the corridor within 8 s.
        let mut crossing = at("X", -60.0, 100.0);
        crossing.v_east = 6.0;
        let a = r
            .traffic_ahead(0.0, 4.0, &own, &[crossing.clone()])
            .expect("crossing");
        assert!(a.crossing);
        // Moving away from the route: nothing.
        crossing.v_east = -6.0;
        assert!(r.traffic_ahead(0.0, 4.0, &own, &[crossing]).is_none());
        // Velocity comes from two readings.
        let mut track = TrafficTrack::default();
        let tgt = |x: f64| {
            let (lat, lon) = ll(x, 100.0);
            crate::world::TcasTarget {
                callsign: "X".into(),
                lat,
                lon,
                ele_ft: 0.0,
                on_ground: true,
                vs_fpm: 0.0,
            }
        };
        track.update(&[tgt(-60.0)], 10.0);
        let s = track.update(&[tgt(-54.0)], 11.0);
        assert!(
            (s[0].v_east - 6.0).abs() < 0.1 && s[0].v_north.abs() < 0.1,
            "{s:?}"
        );
    }

    #[test]
    fn the_cap_ramps_to_zero_at_the_gap_and_the_hold_resumes_when_clear() {
        let r = route();
        let own = jet();
        let leader = |y: f64| [at("EZY12", 0.0, y)];
        // Far: the drive's own stop ramp, falling as we close.
        let far = r.traffic_ahead(0.0, 4.0, &own, &leader(140.0)).unwrap();
        let near = r.traffic_ahead(70.0, 4.0, &own, &leader(140.0)).unwrap();
        assert!(traffic_cap_kts(&far) > traffic_cap_kts(&near) && traffic_cap_kts(&near) > 0.0);
        // At the gap: zero, and the hold begins, said once.
        let mut hold = TrafficHold::default();
        let (cap, ev) = hold.step(Some(&near), 0.0);
        assert!(cap.is_some_and(|c| c > 0.0) && ev.is_none() && !hold.holding());
        let there = r.traffic_ahead(84.0, 0.0, &own, &leader(140.0)).unwrap();
        assert!(there.stop_in_m <= STOP_ZERO_M, "{there:?}");
        let (cap, ev) = hold.step(Some(&there), 1.0);
        assert_eq!(cap, Some(TRAFFIC_HOLD_KTS));
        assert_eq!(
            hold_line(ev.as_ref().unwrap()),
            Some("Holding for traffic ahead.")
        );
        assert!(
            TRAFFIC_HOLD_KTS < 1.0 && TRAFFIC_HOLD_KTS > 0.0,
            "brakes, yet no spoken stop"
        );
        // Held, silently, as the leader edges a few metres.
        let edged = r.traffic_ahead(84.0, 0.0, &own, &leader(145.0)).unwrap();
        assert_eq!(hold.step(Some(&edged), 2.0), (Some(TRAFFIC_HOLD_KTS), None));
        // Ten minutes: said again, still held, never given up.
        let (cap, ev) = hold.step(Some(&edged), 1.0 + TRAFFIC_LONG_S);
        assert_eq!(cap, Some(TRAFFIC_HOLD_KTS));
        assert!(matches!(ev, Some(HoldEvent::Long { .. })));
        assert_eq!(hold.step(Some(&edged), 2.0 * TRAFFIC_LONG_S).1, None);
        // The leader taxis on: the hold lets go and says so after a long wait.
        let (cap, ev) = hold.step(None, 2.0 * TRAFFIC_LONG_S + 1.0);
        assert_eq!(cap, None);
        let ev = ev.expect("cleared");
        assert!(matches!(ev, HoldEvent::Cleared { .. }));
        assert_eq!(hold_line(&ev), Some("Traffic clear."));
        assert!(!hold.holding());
        // A short hold resumes in silence.
        let mut short = TrafficHold::default();
        short.step(Some(&there), 0.0);
        assert_eq!(hold_line(&short.step(None, 5.0).1.unwrap()), None);
    }

    #[test]
    fn our_size_sets_the_corridor_and_the_gap() {
        let light = OwnSize::from_mtow(Some(1_100.0));
        assert_eq!(light.gap_m, 8.0);
        assert!((light.corridor_half_m() - 27.0).abs() < 0.01);
        let unread = OwnSize::from_mtow(None);
        assert_eq!(unread, jet());
        assert_eq!(unread.gap_m, 18.0);
    }
}
