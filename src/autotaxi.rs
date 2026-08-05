//! Auto-taxi — the copilot steers the cleared taxi route to the holding point.
//!
//! Composes the two halves that already exist: the taxi-speed governor
//! ([`crate::tools::speedhold`], throttle + brakes) and the tower's published
//! route ([`crate::party_line::with_taxi_route`], the same polyline the
//! progressive taxi guide narrates). This module adds the missing piece — a
//! ground steering law — and a speed schedule that slows for bends and ramps
//! to a stop short of the holding point. The guide keeps talking over it, so
//! the pilot hears the same "next left onto Whiskey" narration whether they
//! or the copilot are steering.
//!
//! Steering goes through the plugin's `xcopilot/taxi_steer` channel (the
//! proportional-brake pattern: per-frame writes under
//! `override_joystick_heading`, expiring within a second of writes stopping),
//! because REST-rate writes to `yoke_heading_ratio` lose the re-drive fight
//! with the pilot's hardware exactly like the brake ratios did.
//!
//! Handback, every path instant:
//! - The pilot's FEET win in-plugin: real yaw-axis deflection latches
//!   `xcopilot/steer_yield` and drops the override that frame — this loop
//!   polls the latch and stands down, but the pedals were live already.
//! - "My controls" (voice or panel button) releases steering and throttle
//!   together via the main loop's release path; "my steering" releases just
//!   the steering, leaving the speed hold — the pre-existing "you steer, I
//!   have the throttle" division.
//! - The governor standing down for ANY reason (park handle, throttle push,
//!   airborne, release words) takes the steering with it: speed authority
//!   gone means nobody is stopping the aircraft, so steering alone must not
//!   keep driving it.
//! - An engage that starts off the route or facing against it is a JOIN, not
//!   a fault: the law turns toward the route at bend speed and the off-route
//!   guard is widened to the engage distance plus [`JOIN_GROW_M`] until the
//!   aircraft is captured (close and aligned) — then the tight guard resumes.
//!   Once ESTABLISHED, drifting past [`OFF_ROUTE_STOP_M`] still STOPS the
//!   aircraft and releases steering: a law that held meters for a while and
//!   then walked off means something is broken, and a driven "recovery" on
//!   pavement it was never near is nobody's clearance.
//!
//! Everything here is pure geometry over the published polyline; the laws
//! battle-test as plain `cargo test`, no sim.

use std::sync::atomic::{AtomicBool, Ordering};

use crate::station::TaxiWatch;
use crate::taxi_guide;
use crate::tools::speedhold;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

pub(crate) const LAT: &str = "sim/flightmodel/position/latitude";
pub(crate) const LON: &str = "sim/flightmodel/position/longitude";
/// TRUE heading, matching the route bearings computed in the local
/// east/north frame — never the magnetic compass.
pub(crate) const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";
/// The plugin's proportional steering channel (-1..1, right positive).
const TAXI_STEER: &str = "xcopilot/taxi_steer";
/// The sim's own toe-brake inputs, for the differential-brake steering assist
/// ([`brake_steer_split`]). Written one side at a time and zeroed on every
/// stand-down; pilot toe input on the SAME datarefs wins by the pedal/steer
/// yield machinery, not by arbitration here.
/// The differential-brake steering channels. NOT the stock ratios: XP12
/// re-drives `cockpit2/controls/*_brake_ratio` from the pilot's assigned
/// toe-brake axes every frame, so a REST write is gone before the next read
/// (probed live 2026-07-30 — a PATCH of 0.25 read back 0.0000 within 400 ms,
/// and held 0.2500 under `override_toe_brakes`). Only the plugin can hold that
/// override at frame rate, and it arbitrates these against the governor's
/// stopping brake per wheel. The channels expire a second after writes stop,
/// so a dead app frees the pedals by silence.
const LEFT_BRAKE: &str = "xcopilot/taxi_brake_l";
const RIGHT_BRAKE: &str = "xcopilot/taxi_brake_r";
/// The plugin's pilot-feet latch: 1 = the yaw axis moved, override dropped.
const STEER_YIELD: &str = "xcopilot/steer_yield";
const MS_TO_KTS: f64 = 1.943_844;
const M_PER_NM: f64 = 1852.0;

/// Stop this many meters short of the route's end. The planner already ends
/// the route at the hold-short standoff (40 m back from a runway-junction
/// tail — nav.rs `plan`), so this is only the nose-ahead-of-CG margin, not
/// the hold line's position. It was 12 m back when the route ended ON the
/// runway centreline, and the aircraft still stopped past the hold line
/// (2026-07-20): the standoff belongs to the planner, not this margin.
const STOP_SHORT_M: f64 = 5.0;
/// Inside this many meters of the stop, steer to the route's FINAL BEARING
/// rather than to a pursuit aim point. Chosen from the live trace: the
/// pursuit error was still honest at 8 m to run (-8°) and had begun walking
/// by 4 m (+1°), so the swap happens before it degrades rather than after.
const STOP_HEADING_HOLD_M: f64 = 8.0;
/// Lateral drift at which the loop stops the aircraft and gives steering
/// back, once ESTABLISHED on the route. Deliberately much tighter than the
/// guide/compliance 60 m: the law holds a couple of meters in normal work,
/// so 25 m off means something is wrong (bad fix, blocked nosewheel, a
/// route the geometry can't follow).
pub(crate) const OFF_ROUTE_STOP_M: f64 = 25.0;
/// Farther off than this at engage is a refusal: the projection onto the
/// route stops meaning anything, and a blind cross-country drive to it
/// isn't a join, it's a new route the tower never gave us.
pub(crate) const MAX_JOIN_M: f64 = 150.0;
/// Extra lateral allowance granted while joining, on top of the engage
/// distance: room for the turning circle of a turn-around without ever
/// letting a broken law wander further from the route than it started.
const JOIN_GROW_M: f64 = 40.0;
/// Joined when inside this lateral AND aligned within [`JOIN_DONE_ERR_DEG`]:
/// the tight established guard takes over from there.
const JOIN_DONE_LAT_M: f64 = 15.0;
const JOIN_DONE_ERR_DEG: f64 = 45.0;
/// While joining with the nose still this far off the aim, hold the
/// sharp-turn pace; better aligned joins ride at ordinary bend speed.
const JOIN_ERR_SLOW_DEG: f64 = 60.0;
/// Cross-track error that has a wheel off the centreline but is still well
/// inside the paint (a code-C taxiway is ~23 m wide): back off to the bend
/// pace. Past [`TRACK_SHARP_LAT_M`] the aircraft is at or beyond the edge and
/// only the sharp-corner pace is honest. See [`Pace::track_cap_kts`].
const TRACK_TURN_LAT_M: f64 = 6.0;
const TRACK_SHARP_LAT_M: f64 = 12.0;
/// The same two bands in heading. Deliberately looser than the lateral pair: a
/// healthy pure-pursuit intercept carries a real crossing angle by design, so
/// only an angle no intercept would ask for should slow the schedule.
const TRACK_TURN_ERR_DEG: f64 = 20.0;
const TRACK_SHARP_ERR_DEG: f64 = 35.0;
/// Full steering deflection at this much heading error.
const STEER_FULL_DEG: f64 = 30.0;
/// Route bends at/above this (smoothed like the guide's cues) slow the
/// schedule to the pace's turn cap; at/above `SHARP_DEG`, to its sharp cap.
const TURN_DEG: f64 = 30.0;
const SHARP_DEG: f64 = 100.0;
/// The GA pace, tuned on the 172 and the default for any airframe whose
/// profile declares nothing. See [`Pace`].
const STRAIGHT_KTS: f64 = 10.0;
const TURN_KTS: f64 = 6.0;
const SHARP_KTS: f64 = 4.0;

/// How briskly this airframe taxis: the straight-leg pace and the two bend
/// caps. Carried BY THE ROUTE, not a process-wide static, so a profile swap
/// can't leave the last aircraft's pace pointed at the next one — the same
/// trap the radio-power latch and the transponder knob selector fell into.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pace {
    pub straight: f64,
    pub turn: f64,
    pub sharp: f64,
    /// Steering feel, per airframe — see [`crate::aircraft::TaxiProfile`].
    pub steer_full_deg: f64,
    pub steer_lead_s: f64,
    pub steer_slew_per_s: Option<f64>,
    /// Differential-brake steering assist (castoring nosewheel + no prop
    /// wash) — see [`brake_steer_split`].
    pub brake_steer: bool,
    /// Pure-pursuit aim time, seconds — see [`lookahead_m_with`].
    pub lookahead_s: f64,
}

impl Default for Pace {
    fn default() -> Self {
        Pace {
            straight: STRAIGHT_KTS,
            turn: TURN_KTS,
            sharp: SHARP_KTS,
            steer_full_deg: STEER_FULL_DEG,
            steer_lead_s: STEER_RATE_LEAD_S,
            steer_slew_per_s: None,
            brake_steer: false,
            lookahead_s: 5.0,
        }
    }
}

impl Pace {
    /// Resolve an airframe's declared pacing, field by field — anything the
    /// profile leaves out keeps the GA figure.
    pub fn from_profile(t: Option<&crate::aircraft::TaxiProfile>) -> Pace {
        let d = Pace::default();
        let Some(t) = t else { return d };
        // Clamped to the governor's own taxi ceiling: a profile typo must not
        // become a takeoff roll with the tower's route as its plan.
        Pace {
            straight: t
                .kts
                .map_or(d.straight, crate::tools::speedhold::clamp_taxi_target),
            turn: t.turn_kts.map_or(d.turn, |k| k.clamp(2.0, 25.0)),
            sharp: t.sharp_kts.map_or(d.sharp, |k| k.clamp(2.0, 25.0)),
            // Clamped the same way the speeds are: a profile typo must not be
            // able to make the steering either inert or a bang-bang switch.
            steer_full_deg: t
                .steer_full_deg
                .map_or(d.steer_full_deg, |v| v.clamp(10.0, 90.0)),
            steer_lead_s: t.steer_lead_s.map_or(d.steer_lead_s, |v| v.clamp(0.0, 3.0)),
            // A slew below 0.5/s is the auto-takeoff limit-cycle trap; refuse to
            // configure one that tight rather than ship a new oscillation.
            steer_slew_per_s: t.steer_slew_per_s.map(|v| v.clamp(0.5, 10.0)),
            brake_steer: t.brake_steer,
            lookahead_s: t.lookahead_s.map_or(5.0, |v| v.clamp(3.0, 12.0)),
        }
    }

    /// The speed cap while joining: a turn-around happens at the sharp-corner
    /// pace, an aligned rejoin at ordinary bend speed. Never the open-taxiway
    /// pace — the aircraft is off the pavement it was cleared on.
    pub fn join_cap_kts(&self, err_deg: f64) -> f64 {
        if err_deg.abs() > JOIN_ERR_SLOW_DEG {
            self.sharp
        } else {
            self.turn
        }
    }

    /// The speed cap for how the aircraft is actually TRACKING, as opposed to
    /// how the route ahead bends.
    ///
    /// The bend caps read the paint and nothing else: they answer "how sharp is
    /// what's coming", never "are we on it". Between them and `join_cap_kts`
    /// there was a hole — the join cap stops at the `join_done` latch (15 m,
    /// 45 deg), which is loose enough to fire in the middle of a capture that
    /// has not happened, and past it the schedule went straight back to the
    /// open-taxiway pace. Southampton, 2026-07-30: the SF50 latched joined at
    /// 14.8 m and -28 deg, the last leg was straight so the cap returned to
    /// 12 kt, and it accelerated 6.5 -> 12.7 kt while still 13 m off with 27
    /// deg of crossing angle and 33 m to run. On a castoring-wheel jet whose
    /// entire steering authority is a quarter of one-sided brake (~3.5 deg/s
    /// flat out) that is unrecoverable: it simply floats wide.
    ///
    /// So speed backs off for the error itself, and the cap outlives the join
    /// latch. Slowing down is what a pilot does when the centreline is drifting
    /// away from under the nose; it buys the steering more degrees per metre,
    /// which is the only currency a weak plant has.
    pub fn track_cap_kts(&self, lateral_m: f64, err_deg: f64) -> f64 {
        if lateral_m > TRACK_SHARP_LAT_M || err_deg.abs() > TRACK_SHARP_ERR_DEG {
            self.sharp
        } else if lateral_m > TRACK_TURN_LAT_M || err_deg.abs() > TRACK_TURN_ERR_DEG {
            self.turn
        } else {
            self.straight
        }
    }
}
/// Bearing smoothing window for bend detection, meters (the guide's value:
/// densely-noded fillet arcs must not read as hairpins).
const WINDOW_M: f64 = 15.0;

/// True while the steering loop runs, mirroring `speedhold::ACTIVE` /
/// `autoland::ACTIVE`. Cleared by the loop itself, the release path in
/// main, or the `auto_taxi` tool's off switch.
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// True while a BACKTRACK is in progress, including the gap between its two
/// legs. `ACTIVE` alone drops false in that gap — leg one ends, the supervisor
/// builds leg two, and a caller polling `ACTIVE` sees "the drive is finished"
/// for a moment that isn't. Full auto did exactly that and called a running
/// line-up a failure one second after leg two engaged (Luton, 2026-07-28).
pub static BACKTRACKING: AtomicBool = AtomicBool::new(false);

/// True while ANY auto drive owns the aircraft — the drive itself, or a
/// backtrack between its legs. What a caller waiting for "are we there yet"
/// should poll.
pub fn driving() -> bool {
    ACTIVE.load(Ordering::Relaxed) || BACKTRACKING.load(Ordering::Relaxed)
}

/// Whether the LAST drive ended by ARRIVING at its stop (vs a handback,
/// arrest, or off-route stand-down). The backtrack supervisor reads this to
/// decide whether leg two may begin — any other ending already spoke and
/// must not be driven over.
pub static LAST_ARRIVED: AtomicBool = AtomicBool::new(false);

/// One-line status for the dashboard's "managing" panel.
pub fn status() -> Option<String> {
    ACTIVE
        .load(Ordering::Relaxed)
        .then(|| "auto taxi".to_string())
}

/// The cleared route, pre-digested into the local flat-earth frame the
/// steering law works in.
pub struct Route {
    /// Lat/lon polyline, for projecting fixes (same code path as the guide).
    path: Vec<(f64, f64)>,
    /// The polyline in meters (x east, y north) from `path[0]`.
    pts: Vec<(f64, f64)>,
    /// Cumulative along-route distance per vertex.
    cum: Vec<f64>,
    total: f64,
    /// (along-route m, |bend| degrees) per real corner, for the slow-down.
    bends: Vec<(f64, f64)>,
    /// Spoken holding point ("November five Whiskey"), may be empty.
    pub holding_point: String,
    /// Overrides the arrival line when set — a LINE-UP drive ends with
    /// "Lined up runway two seven", not "Holding point".
    pub done_line: Option<String>,
    /// This airframe's pacing. `build` defaults it to the GA figures; the
    /// arming tool sets the flying aircraft's own (see [`Pace`]).
    pub pace: Pace,
}

impl Route {
    /// Digest a published route. `None` when it's too short to drive.
    pub fn build(watch: &TaxiWatch) -> Option<Route> {
        if watch.path.len() < 2 {
            return None;
        }
        let geo = taxi_guide::Geo::new(watch.path[0].0);
        let pts: Vec<(f64, f64)> = watch
            .path
            .iter()
            .map(|&p| geo.to_m(watch.path[0], p))
            .collect();
        let mut cum = vec![0.0_f64];
        for w in pts.windows(2) {
            cum.push(cum.last().unwrap() + taxi_guide::dist(w[0], w[1]));
        }
        let total = *cum.last().unwrap();
        // Shorter than the stop-short margin plus a rolling start there is
        // nothing to drive — the tool answers "we're already there".
        if total < STOP_SHORT_M + 15.0 {
            return None;
        }
        let mut bends: Vec<(f64, f64)> = Vec::new();
        for i in 1..pts.len() - 1 {
            let b_in =
                taxi_guide::bearing(taxi_guide::point_at(&pts, &cum, cum[i] - WINDOW_M), pts[i]);
            let b_out =
                taxi_guide::bearing(pts[i], taxi_guide::point_at(&pts, &cum, cum[i] + WINDOW_M));
            let delta = taxi_guide::angle_diff(b_out, b_in).abs();
            if delta >= TURN_DEG {
                bends.push((cum[i], delta));
            }
        }
        Some(Route {
            path: watch.path.clone(),
            pts,
            cum,
            total,
            bends,
            holding_point: crate::station::spoken_taxiway(&watch.holding_point),
            done_line: None,
            pace: Pace::default(),
        })
    }

    /// Project a lat/lon fix: (along-route progress m, DISTANCE off the route
    /// in m). Never negative — every gate here asks "how far off", which is
    /// the same question on both sides of the line.
    pub fn fix(&self, lat: f64, lon: f64) -> Option<(f64, f64)> {
        taxi_guide::project(&self.path, lat, lon)
    }

    /// [`fix`](Self::fix) keeping the SIDE: positive is right of the route.
    /// What the drive records, so a trail can be read against the paint
    /// afterwards instead of only saying how far off it was.
    pub fn fix_signed(&self, lat: f64, lon: f64) -> Option<(f64, f64)> {
        taxi_guide::project_signed(&self.path, lat, lon)
    }

    /// Heading error (signed degrees, right positive) from the aircraft's
    /// true heading to the pure-pursuit aim point `lookahead` meters further
    /// along the route. Aiming at a point ON the route (not the tangent) is
    /// what folds the cross-track error back to the centreline.
    pub fn heading_err_deg(
        &self,
        lat: f64,
        lon: f64,
        heading_true: f64,
        progress: f64,
        lookahead: f64,
    ) -> f64 {
        let geo = taxi_guide::Geo::new(self.path[0].0);
        let here = geo.to_m(self.path[0], (lat, lon));
        let aim = taxi_guide::point_at(&self.pts, &self.cum, progress + lookahead);
        taxi_guide::angle_diff(taxi_guide::bearing(here, aim), heading_true)
    }

    /// The direction the route's LAST segment runs, degrees true.
    ///
    /// Near the stop this replaces the pure-pursuit aim, because the aim point
    /// saturates: `point_at` clamps at the final point, so once the aircraft
    /// rolls past it the bearing to it swings round BEHIND and the law
    /// commands a hard turn toward a waypoint already passed. Measured at
    /// EGHI (2026-07-30), the error ran away with the aircraft dead on the
    /// centreline:
    ///
    ///   rem  0.04 m -> err   9.5°     rem -3.6 m -> err  38.6°
    ///   rem -4.4  m -> err  70.2°     rem -5.0 m -> err 162.6°
    ///
    /// Steering pinned at its limit and the nose came round: 0.07 m off the
    /// line to 3.00 m in four seconds. Every stop did it — hold, line-up,
    /// backtrack, taxi-in — because they all drive this one loop.
    pub fn final_bearing_deg(&self) -> Option<f64> {
        let n = self.pts.len();
        (n >= 2).then(|| taxi_guide::bearing(self.pts[n - 2], self.pts[n - 1]))
    }

    /// Heading error against the route's final bearing — what to fly once the
    /// pursuit aim can no longer be trusted. Falls back to the pursuit error
    /// when the route is too short to have a final segment.
    pub fn stop_heading_err_deg(
        &self,
        lat: f64,
        lon: f64,
        heading_true: f64,
        progress: f64,
        lookahead: f64,
    ) -> f64 {
        match self.final_bearing_deg() {
            Some(b) => taxi_guide::angle_diff(b, heading_true),
            None => self.heading_err_deg(lat, lon, heading_true, progress, lookahead),
        }
    }

    /// Meters left before the stop point short of the holding point.
    pub fn remaining(&self, progress: f64) -> f64 {
        self.total - STOP_SHORT_M - progress
    }

    /// The bend-imposed speed cap near `progress`, if a corner is close
    /// enough to matter: inside a speed-scaled lead distance before the
    /// corner, and until 15 m past it (mid-turn is no place to accelerate).
    pub fn bend_cap_kts(&self, progress: f64, gs_kts: f64) -> Option<f64> {
        let lead = (gs_kts * 0.514 * 6.0).clamp(25.0, 90.0);
        self.bends
            .iter()
            .filter(|(at, _)| progress < at + 15.0 && at - progress < lead)
            .map(|&(_, deg)| {
                if deg >= SHARP_DEG {
                    self.pace.sharp
                } else {
                    self.pace.turn
                }
            })
            .fold(None, |acc: Option<f64>, cap| {
                Some(acc.map_or(cap, |a| a.min(cap)))
            })
    }
}

/// Steering command for a heading error: proportional, full deflection at
/// [`STEER_FULL_DEG`]. The plant decides whether raw P is enough: a linked
/// nosewheel (172) is self-damped, a castoring one is not — feed this
/// [`damped_err_deg`], never the raw error.
pub fn steer_cmd_with(err_deg: f64, full_deg: f64) -> f64 {
    (err_deg / full_deg).clamp(-1.0, 1.0)
}

/// Rate-limit the steering command toward `want`. A taxiing aircraft is a
/// vehicle with people in it: the lever slamming half its travel in under a
/// second is felt in the cabin as a jolt, whatever it does for tracking.
///
/// Per SECOND and dt-scaled, never per tick — the auto-takeoff roll learned
/// that the hard way (a 0.25-per-tick limiter swept only 0.5/s and the rudder
/// chased the error a second behind in exact steps, a textbook slew-induced
/// limit cycle, 2026-07-21). `None` is no limit, which is every airframe that
/// has never needed one.
pub fn slew_steer(prev: f64, want: f64, per_s: Option<f64>, dt: f64) -> f64 {
    match per_s {
        Some(rate) if dt > 0.0 => {
            let step = rate * dt;
            prev + (want - prev).clamp(-step, step)
        }
        _ => want,
    }
}

/// Lead the heading error by the yaw rate. On a castoring nosewheel (SR22)
/// the steer channel is effectively RUDDER-only — a lagged, integrating
/// plant that pure P sent into a ±40° snake at three knots (heading swung
/// 137°→213°→138° on successive state ticks, 2026-07-22). Leading by where
/// the nose will be in [`STEER_RATE_LEAD_S`] checks the swing before the
/// crest. On a linked nosewheel taxi yaw rates are small and the term is
/// nearly inert, so the 172 keeps its proven feel.
pub const STEER_RATE_LEAD_S: f64 = 0.6;
pub fn damped_err_deg(err_deg: f64, yaw_rate_dps: f64, lead_s: f64) -> f64 {
    err_deg - yaw_rate_dps * lead_s
}

/// Steering authority vs groundspeed: a fully deflected nosewheel at a
/// standstill is a chock, not a turn — the tire scrubs sideways and the
/// taxi governor's throttle cap (well below takeoff power on purpose)
/// can't break the aircraft away against it. A 69°-off join at Gatwick
/// sat static at full lock exactly this way (2026-07-20). Partial lock
/// parked lets the roll start; full authority arrives by 3 knots, when
/// the wheel is actually steering.
pub fn steer_authority(gs_kts: f64) -> f64 {
    0.35 + 0.65 * (gs_kts / 3.0).clamp(0.0, 1.0)
}

/// The steer command below which the brakes stay out of it. Small on
/// purpose: "small corrections are the yoke channel's" was the original
/// rationale, and on the one class of airframe that carries the assist the
/// yoke channel is DEAD — below the deadband nothing steers at all. At 0.2
/// the SF50 could not make small sustained corrections, so it drifted until
/// the error bought a command past 0.5, then slow-corrected, 13 m out and
/// back for whole legs (2026-07-29, the fifth taxi). A castoring-wheel jet
/// rides a breath of inside brake down a not-quite-straight taxiway because
/// that is the only steering it has — the real aircraft does the same.
const BRAKE_STEER_DEADBAND: f64 = 0.05;
/// The most inside-brake the assist may apply. Well under the governor's own
/// stop braking: this is a nudge, not a pivot. History: 0.25 pulled 14 m of
/// offset back in ten seconds on the assist's first sortie but read as
/// "violent, oversteering" from the cabin (2026-07-28) and was cut to 0.16 —
/// but the violence was the OLD unsaturated mapping slamming the full cap on
/// at the deadband edge, a shape that no longer exists, and the retreated cap
/// then could not fly a BEND: fifth taxi, mid-bend at 10-13 kt, steer at
/// 0.52-0.60 — effectively the cap — returned 0-0.5 deg/s of yaw against the
/// 3-4 the curve demanded, and the aircraft ran 24 m off the outside
/// (2026-07-29). Back to the value that demonstrably turned the aircraft,
/// applied smoothly by the saturated mapping and the slew.
const BRAKE_STEER_MAX: f64 = 0.25;
/// The command at which the assist reaches that max. NOT 1.0: tracking can
/// never command full deflection — the heading error would have to exceed
/// the whole `steer_full_deg` — so a max reached only at 1.0 is a max the
/// loop can never draw on. The fourth SF50 taxi proved it (2026-07-29):
/// steer sat above 0.5 for two thirds of an 11-minute taxi, which mapped to
/// ~0.07 of brake and 0.69 deg/s of yaw against the ~3.5 the assist gives
/// flat out, and the aircraft held a one-sided 9 m offset (15 m max, the
/// error changing sign four times in 11 minutes) — starvation, not a weave.
/// Saturating at 0.6 hands the tracking band the full authority the cap was
/// sized for.
const BRAKE_STEER_SAT: f64 = 0.6;
/// The assist's own slew, per second — pedals squeeze, they don't stab,
/// exactly the taxi governor's brake discipline.
const BRAKE_STEER_SLEW_PER_S: f64 = 0.4;
/// Speed band the assist fades in across. Below it the nosewheel's own
/// low-speed steering (and the stop machinery) owns the aircraft — braking a
/// near-stopped aircraft asymmetrically is a lurch.
const BRAKE_STEER_MIN_KTS: f64 = 4.0;
const BRAKE_STEER_FULL_KTS: f64 = 7.0;

/// Differential-brake steering: the (left, right) brake ratios for a steer
/// command, for airframes flagged [`Pace::brake_steer`].
///
/// The yoke-heading channel is nosewheel + rudder, and on a castoring-wheel
/// JET both halves are dead at taxi speed — the wheel takes no input and a
/// turbofan puts no wash over the rudder. The SF50 proved it: steer −0.6 held
/// for thirty seconds while the cross-track error grew to 13.8 m and the
/// aircraft tracked the grass parallel to the taxiway (Luton, 2026-07-28,
/// the blackbox trail). A touch of inside brake is how the real aircraft is
/// steered.
///
/// One side only, ever — the governor's pilot-brake detector reads
/// `min(left, right)`, deliberately, so a single-side assist never reads as
/// the pilot slowing the aircraft. The governor's speed loop simply carries
/// the extra drag.
pub fn brake_steer_split(steer: f64, gs_kts: f64) -> (f64, f64) {
    let authority = ((gs_kts - BRAKE_STEER_MIN_KTS) / (BRAKE_STEER_FULL_KTS - BRAKE_STEER_MIN_KTS))
        .clamp(0.0, 1.0);
    let excess = ((steer.abs() - BRAKE_STEER_DEADBAND) / (BRAKE_STEER_SAT - BRAKE_STEER_DEADBAND))
        .clamp(0.0, 1.0);
    let b = excess * BRAKE_STEER_MAX * authority;
    if steer > 0.0 {
        (0.0, b) // nose right: right brake
    } else {
        (b, 0.0)
    }
}

/// Steering held while still rolling, even after the distance fade has run
/// out. Enough for the yaw damper to arrest a rotation, far short of the
/// full deflection that carves.
const STRAIGHTEN_FLOOR: f64 = 0.35;
/// Below this groundspeed the aircraft has genuinely stopped and the fade may
/// go to zero. Matches the arrival test's own 1.5 kt so the two agree.
const STRAIGHTEN_HOLD_KTS: f64 = 1.5;

/// Straighten the nosewheel as a stop closes: braking shifts weight onto
/// the nosewheel, and a deflected one carves under it — every stop at a
/// bendy route end turned into a swerve-and-lurch ("when you brake, the
/// aircraft steers", 2026-07-20). Full steering beyond 8 m to go, faded
/// across the last meters.
///
/// The fade may NOT reach zero while the aircraft is still moving.
/// `remaining_m` SATURATES at `-STOP_SHORT_M` the instant the projection
/// reaches the end of the polyline (`remaining = total - STOP_SHORT_M -
/// progress`, and progress cannot exceed total), so the distance term pins at
/// -5 and the fade hits zero with 2-3 kt still on the clock. Zeroing the
/// command also kills the yaw-rate damping *inside* it, so a residual rotation
/// goes unarrested through the last metres.
///
/// EGHI 2026-07-30, at the HOLDING POINT and repeatably: steer pinned at 0.00,
/// yaw −2 deg/s unarrested, cross-track growing 2.45 → 5.08 m while coasting
/// 2.7 → 1.6 kt. The aircraft stopped several metres off the paint and visibly
/// crooked. The same signature is in all three stops of that sortie.
///
/// The carve this exists to prevent needs SPEED; by the arrival threshold
/// there is none left to carve with, so a floor below it is free.
pub fn stop_straighten(steer: f64, remaining_m: f64, gs_kts: f64) -> f64 {
    let fade = ((remaining_m - 1.0) / 7.0).clamp(0.0, 1.0);
    let floor = if gs_kts > STRAIGHTEN_HOLD_KTS {
        STRAIGHTEN_FLOOR
    } else {
        0.0
    };
    steer * fade.max(floor)
}

/// The lateral allowance for an engage at `lateral` meters off the route
/// with `err_deg` of heading error: zero for an on-route, along-route start
/// (the tight established guard applies from the first tick), otherwise the
/// engage distance plus turning-circle room. The guard while joining is
/// [`OFF_ROUTE_STOP_M`] plus this.
pub fn join_allowance_m(lateral: f64, err_deg: f64) -> f64 {
    if lateral > OFF_ROUTE_STOP_M || err_deg.abs() > JOIN_DONE_ERR_DEG {
        lateral.max(OFF_ROUTE_STOP_M) + JOIN_GROW_M
    } else {
        0.0
    }
}

/// True when a join has captured the route: close in and pointed along it.
pub fn join_done(lateral: f64, err_deg: f64) -> bool {
    lateral < JOIN_DONE_LAT_M && err_deg.abs() < JOIN_DONE_ERR_DEG
}

// ---- Line-up: from the hold-short position onto the runway centreline ----

/// Farther than this from the centreline is not a hold-short position —
/// the same reasoning as [`MAX_JOIN_M`]: the geometry stops meaning
/// "line up" and starts meaning "drive somewhere".
pub(crate) const LINE_UP_MAX_LATERAL_M: f64 = 150.0;
/// Runway that must remain beyond the alignment stop — lined up with less
/// than this ahead is a line-up onto the wrong end.
const LINE_UP_MIN_REMAINING_M: f64 = 400.0;
/// The drive continues this far down the centreline past the entry point,
/// so the nose is pointing down the runway when it stops, not across it.
/// The arrival check is distance-based, so this leg is what buys the
/// alignment — 60 m left the nose 11 degrees off at the stop (2026-07-20).
const LINE_UP_ALIGN_M: f64 = 80.0;

/// The synthetic route of a line-up: from where the aircraft sits, onto the
/// centreline ahead, and down it far enough to finish aligned. `None` when
/// the geometry refuses (too far off, or not enough runway past the stop).
pub fn line_up_path(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    heading_deg: Option<f64>,
) -> Option<Vec<(f64, f64)>> {
    use crate::autoland::project_latlon;
    let (entry_m, stop_m) = line_up_stops_m(rwy, lat, lon, heading_deg)?;
    if rwy.length_m - stop_m < LINE_UP_MIN_REMAINING_M {
        return None;
    }
    let e = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, entry_m / M_PER_NM);
    let b = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, stop_m / M_PER_NM);
    Some(vec![(lat, lon), e, b])
}

/// Where a plain line-up from this position would enter and stop (meters
/// past the threshold), or `None` when the position isn't a hold-short
/// geometry at all. Shared by the path builder and the full-length check.
/// `heading_deg` (the aircraft's current heading) makes leg one roll STRAIGHT
/// FORWARD along the taxiway lead-on onto the centreline instead of cutting a
/// diagonal across to it (the "beeline" the pilot saw, 2026-07-23); `None`
/// keeps the old diagonal (backtrack leg two, tests).
pub fn line_up_stops_m(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    heading_deg: Option<f64>,
) -> Option<(f64, f64)> {
    use crate::autoland::{along_track_nm, cross_track_nm};
    let x0 = cross_track_nm(lat, lon, rwy) * M_PER_NM; // signed, right of course +
    let lateral = x0.abs();
    if lateral > LINE_UP_MAX_LATERAL_M {
        return None;
    }
    // Signed distance past the threshold (positive = down the runway).
    let a0 = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    // Roll straight forward along the current heading onto the centreline —
    // the taxiway lead-on line — when the heading gives a clean crossing.
    // Otherwise the old fallback: enter ahead of abeam by ~the lateral offset
    // (a ~45 degree swing), floored at one turn's worth and capped so a far-set
    // hold (LXGB's Echo ~90 m off the strip) gets a sharper swing, not a 90 m lead.
    let entry_m = roll_forward_entry_m(rwy.course_deg, a0, x0, heading_deg)
        .unwrap_or_else(|| a0.max(0.0) + lateral.clamp(30.0, 60.0));
    Some((entry_m, entry_m + LINE_UP_ALIGN_M))
}

/// Distance past the threshold where the aircraft's heading, rolled straight
/// forward, crosses the centreline — following the taxiway lead-on onto the
/// runway. `None` (use the diagonal fallback) when the heading is within ~15
/// degrees of the runway (no clean crossing), points away from the centreline,
/// crosses behind the threshold, or would run implausibly far.
fn roll_forward_entry_m(
    course_deg: f64,
    a0: f64,
    x0: f64,
    heading_deg: Option<f64>,
) -> Option<f64> {
    let psi = heading_deg?;
    let rel = ((psi - course_deg + 540.0).rem_euclid(360.0) - 180.0).to_radians();
    let s = rel.sin();
    if s.abs() < 0.26 {
        return None; // within ~15 degrees of the runway: no clean forward crossing
    }
    let fwd = -x0 / s; // meters forward to reach cross-track zero
    if fwd <= 0.0 || fwd > 150.0 {
        return None; // heading away from the centreline, or too shallow / far
    }
    let entry = a0 + fwd * rel.cos();
    (entry >= 0.0).then_some(entry)
}

/// A line-up riding the AUTHORED lead-on: the trimmed tail of the cleared taxi
/// route — the painted yellow line from the holding point onto the runway
/// centreline — instead of a synthetic diagonal (the aircraft cut the Echo
/// stub's curve and stopped crooked, LXGB 2026-07-24). Returns the path and
/// its centreline stop (m past the threshold). `None` when the lead-on isn't
/// THIS hold or THIS runway: it must start near the aircraft and end on the
/// centreline with runway enough beyond the stop.
pub fn line_up_path_from_lead_on(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    lead_on: &[(f64, f64)],
) -> Option<(Vec<(f64, f64)>, f64)> {
    use crate::autoland::{along_track_nm, cross_track_nm, haversine_nm, project_latlon};
    let dist_m = |a: (f64, f64), b: (f64, f64)| haversine_nm(a.0, a.1, b.0, b.1) * M_PER_NM;
    let (&first, &last) = (lead_on.first()?, lead_on.last()?);
    // OURS, not another stub's: compare ALONG-TRACK positions, not straight-
    // line distance — the stub runs perpendicular to the runway, so a legit
    // hold sits ~90 m from its own junction (LXGB Echo: 92 m; the old 80 m
    // straight-line guard silently rejected it and the diagonal came back,
    // 2026-07-24, probed). A foreign stub is displaced ALONG the runway.
    let a_air = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    let a_first = -along_track_nm(first.0, first.1, rwy) * M_PER_NM;
    if (a_first - a_air).abs() > 80.0 {
        return None;
    }
    // ...and ends ON this runway's centreline (the junction node).
    if (cross_track_nm(last.0, last.1, rwy) * M_PER_NM).abs() > 20.0 {
        return None;
    }
    let junction_a = -along_track_nm(last.0, last.1, rwy) * M_PER_NM;
    if junction_a < 0.0 {
        return None;
    }
    let stop_m = junction_a + LINE_UP_ALIGN_M;
    if rwy.length_m - stop_m < LINE_UP_MIN_REMAINING_M {
        return None;
    }
    // Aircraft, then the authored points (skipping any we're already on top
    // of), then a FILLET ARC around the junction corner onto the centreline,
    // then the alignment stop. The network models the entry as a right angle;
    // the paint curves — a sampled arc where the sharp corner was is what the
    // pursuit can actually follow (it chorded the raw corner, probed
    // 2026-07-24).
    let mut path = vec![(lat, lon)];
    path.extend(
        lead_on
            .iter()
            .copied()
            .filter(|p| dist_m((lat, lon), *p) > 5.0),
    );
    let stop = project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, stop_m / M_PER_NM);
    let corner = path.pop()?; // the junction (or the aircraft itself — refused below)
    if path.is_empty() {
        return None;
    }
    path.extend(fillet_corner(*path.last()?, corner, stop, 18.0));
    path.push(stop);
    (path.len() >= 3).then_some((path, stop_m))
}

/// A sampled quadratic-Bezier fillet replacing the sharp vertex at `corner`:
/// starts `r_m` back along the inbound leg, ends `r_m` down the outbound leg,
/// bulging toward the corner — the painted lead-on curve the taxi networks
/// don't model. Radii clamp to half of each leg so short legs stay sane.
fn fillet_corner(
    prev: (f64, f64),
    corner: (f64, f64),
    next: (f64, f64),
    r_m: f64,
) -> Vec<(f64, f64)> {
    let seg = |a: (f64, f64), b: (f64, f64), t: f64| (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
    let len =
        |a: (f64, f64), b: (f64, f64)| crate::autoland::haversine_nm(a.0, a.1, b.0, b.1) * M_PER_NM;
    let l_in = len(prev, corner);
    let l_out = len(corner, next);
    if l_in < 1.0 || l_out < 1.0 {
        return vec![corner];
    }
    let a = seg(corner, prev, (r_m.min(l_in * 0.5)) / l_in); // arc entry, on the inbound leg
    let b = seg(corner, next, (r_m.min(l_out * 0.5)) / l_out); // arc exit, on the outbound leg
                                                               // Quadratic Bezier a -> corner(control) -> b, sampled.
    let bez = |t: f64| {
        let u = 1.0 - t;
        (
            u * u * a.0 + 2.0 * u * t * corner.0 + t * t * b.0,
            u * u * a.1 + 2.0 * u * t * corner.1 + t * t * b.1,
        )
    };
    vec![a, bez(0.25), bez(0.5), bez(0.75), b]
}

/// A backtrack pocket sits this far past the threshold along the runway —
/// room for the turnaround to finish at the numbers.
const BACKTRACK_TURN_START_M: f64 = 60.0;
/// ...and this far off the centreline (right of the backtrack direction),
/// so the 180 is a teardrop with turning room, never a pivot on the spot.
const BACKTRACK_POCKET_OFFSET_M: f64 = 12.0;
/// Closer to the threshold than this, a backtrack is pointless — the plain
/// line-up already stops within a couple of plane-lengths of the numbers.
const BACKTRACK_MIN_M: f64 = 150.0;

/// Leg one of a backtrack: from the hold, onto the centreline heading TOWARD
/// the departure threshold, down the strip, ending in a turn pocket beside
/// the numbers. Leg two is a plain [`line_up_path`] from the pocket — the
/// join machinery already handles "facing the wrong way", and splitting the
/// maneuver keeps each route free of overlapping legs (a single out-and-back
/// polyline is ambiguous to project onto). Real Gibraltar: the taxiways only
/// reach the east half, so a full-length 09 departure backtracks ~1.2 km
/// (2026-07-22, held at Charlie with 580 m ahead).
pub fn backtrack_pocket_path(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
) -> Option<Vec<(f64, f64)>> {
    use crate::autoland::{along_track_nm, cross_track_nm, project_latlon};
    let lateral = (cross_track_nm(lat, lon, rwy) * M_PER_NM).abs();
    if lateral > LINE_UP_MAX_LATERAL_M {
        return None;
    }
    let past_m = (-along_track_nm(lat, lon, rwy) * M_PER_NM).max(0.0);
    if past_m < BACKTRACK_MIN_M {
        return None;
    }
    let c = |d_m: f64| project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, d_m / M_PER_NM);
    // Enter the runway swinging toward the threshold (the mirror of the
    // line-up's entry lead), never inside the turn pocket itself.
    let entry_m = (past_m - lateral.clamp(30.0, 60.0)).max(BACKTRACK_TURN_START_M + 30.0);
    let entry = c(entry_m);
    let turn_start = c(BACKTRACK_TURN_START_M);
    // Swing the teardrop toward the side the aircraft CAME IN on — the taxiway/
    // apron side, which is the way the painted turn-pad leads. A fixed hand
    // turned the wrong way against the yellow line at LXGB 09 (2026-07-26).
    // `cross_track` is signed (+ = right of the runway course); default to the
    // right of travel when we're dead on the centreline.
    let entry_ct_m = cross_track_nm(lat, lon, rwy) * M_PER_NM;
    let pocket_bearing = if entry_ct_m >= 0.0 {
        rwy.course_deg + 90.0
    } else {
        rwy.course_deg - 90.0
    };
    let mid = c(BACKTRACK_TURN_START_M / 2.0);
    let pocket = project_latlon(
        mid.0,
        mid.1,
        pocket_bearing,
        BACKTRACK_POCKET_OFFSET_M / M_PER_NM,
    );
    Some(vec![(lat, lon), entry, turn_start, pocket])
}

/// The same backtrack, entering the runway along the AUTHORED lead-on — the
/// painted line from the holding point onto the centreline — instead of a
/// diagonal aimed at a point down the strip.
///
/// The plain line-up learned this in 2026-07-24 ([`line_up_path_from_lead_on`]);
/// the backtrack kept its diagonal and it put an MD-82 in the grass at Luton
/// (2026-07-27). Its leg one ran from the holding point to a centreline point
/// 30 to 60 m nearer the threshold — 23 degrees off the stub the aircraft was
/// standing on — so the drive left the pavement within fifty metres, bogged,
/// and the governor called it stagnant with the aircraft off the taxiway. The
/// entry is not ours to invent: the stub is drawn, and the only question is
/// which way we turn at the top of it.
pub fn backtrack_pocket_path_from_lead_on(
    rwy: &crate::autoland::Runway,
    lat: f64,
    lon: f64,
    lead_on: &[(f64, f64)],
) -> Option<Vec<(f64, f64)>> {
    use crate::autoland::{along_track_nm, cross_track_nm, haversine_nm, project_latlon};
    let dist_m = |a: (f64, f64), b: (f64, f64)| haversine_nm(a.0, a.1, b.0, b.1) * M_PER_NM;
    let (&first, &last) = (lead_on.first()?, lead_on.last()?);
    // OURS, and ending on THIS centreline — the same two tests the plain
    // lead-on line-up applies, for the same reason: a foreign stub is
    // displaced along the runway, not merely far away.
    let a_air = -along_track_nm(lat, lon, rwy) * M_PER_NM;
    if (-along_track_nm(first.0, first.1, rwy) * M_PER_NM - a_air).abs() > 80.0 {
        return None;
    }
    if (cross_track_nm(last.0, last.1, rwy) * M_PER_NM).abs() > 20.0 {
        return None;
    }
    // Same "is a backtrack worth it" test as the synthetic version — measured
    // from the AIRCRAFT, not the junction: near the numbers the plain line-up
    // already stops within a plane-length or two of them. And the junction has
    // to sit beyond the turn pocket, or there is nothing to drive back along.
    if a_air < BACKTRACK_MIN_M {
        return None;
    }
    let junction_a = -along_track_nm(last.0, last.1, rwy) * M_PER_NM;
    if junction_a < BACKTRACK_TURN_START_M + 30.0 {
        return None;
    }
    let c = |d_m: f64| project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, d_m / M_PER_NM);
    let turn_start = c(BACKTRACK_TURN_START_M);
    let entry_ct_m = cross_track_nm(lat, lon, rwy) * M_PER_NM;
    let pocket_bearing = if entry_ct_m >= 0.0 {
        rwy.course_deg + 90.0
    } else {
        rwy.course_deg - 90.0
    };
    let mid = c(BACKTRACK_TURN_START_M / 2.0);
    let pocket = project_latlon(
        mid.0,
        mid.1,
        pocket_bearing,
        BACKTRACK_POCKET_OFFSET_M / M_PER_NM,
    );
    // Aircraft, up the drawn stub, a fillet around the junction (the network
    // draws a right angle, the paint curves), then down the strip to the
    // pocket.
    let mut path = vec![(lat, lon)];
    path.extend(
        lead_on
            .iter()
            .copied()
            .filter(|p| dist_m((lat, lon), *p) > 5.0),
    );
    let corner = path.pop()?;
    if path.is_empty() {
        return None;
    }
    path.extend(fillet_corner(*path.last()?, corner, turn_start, 18.0));
    path.push(turn_start);
    path.push(pocket);
    (path.len() >= 4).then_some(path)
}

/// How a line-up will be driven: straight on, or the full-length backtrack.
pub enum LineUpPlan {
    Plain(Vec<(f64, f64)>),
    Backtrack(Vec<(f64, f64)>),
}

/// Pick the runway AND the maneuver: a plain line-up when the runway ahead
/// of its stop covers `needed_m`, otherwise a backtrack to the threshold
/// when the geometry offers one (`force_backtrack` skips straight to it —
/// the pilot asked for the full length in so many words).
#[allow(clippy::too_many_arguments)]
pub fn pick_line_up_plan(
    cands: &[crate::autoland::Runway],
    lat: f64,
    lon: f64,
    cleared: Option<&str>,
    needed_m: f64,
    force_backtrack: bool,
    heading_deg: Option<f64>,
    lead_on: Option<&[(f64, f64)]>,
) -> Option<(crate::autoland::Runway, LineUpPlan)> {
    let mut best: Option<(crate::autoland::Runway, LineUpPlan, f64)> = None;
    for r in cands {
        if let Some(c) = cleared {
            if !crate::autotakeoff::runway_matches(&r.id, c) {
                continue;
            }
        }
        // The authored lead-on wins when it belongs to this runway and this
        // hold — the painted line IS the maneuver. Synthetic geometry is the
        // fallback (no published route, wrong stub, backtracks).
        let via_lead = if force_backtrack {
            None
        } else {
            lead_on.and_then(|lo| line_up_path_from_lead_on(r, lat, lon, lo))
        };
        let (plain, stop_m) = match via_lead {
            Some((p, s)) => (Some(p), Some(s)),
            None => (
                line_up_path(r, lat, lon, heading_deg),
                line_up_stops_m(r, lat, lon, heading_deg).map(|(_, stop)| stop),
            ),
        };
        let ahead = stop_m.map(|s| r.length_m - s);
        // Use the FULL length for a departure: if the line-up entry sits well
        // past the numbers (a deep intersection like LXGB's Charlie, which
        // enters the middle of 09) on a short/medium runway, backtrack to the
        // threshold rather than accept the reduced length — even when the roll
        // would fit. A departure lines up at the START of the runway; the pilot
        // shouldn't have to ask (2026-07-26). Long runways keep intersection
        // departures (backtracking 3 km of active runway is not the move).
        const DEEP_INTERSECTION_M: f64 = 200.0;
        const FULL_LENGTH_MATTERS_M: f64 = 2500.0;
        let deep_intersection =
            stop_m.is_some_and(|s| s > DEEP_INTERSECTION_M) && r.length_m <= FULL_LENGTH_MATTERS_M;
        let wants_backtrack = force_backtrack
            || ahead.is_none_or(|a| a < needed_m)
            || plain.is_none()
            || deep_intersection;
        let plan = if wants_backtrack {
            // The drawn stub first, here too — a backtrack still has to get
            // ONTO the runway, and inventing that entry is what put an
            // aircraft in the grass (see `backtrack_pocket_path_from_lead_on`).
            let via_lead_bt = lead_on
                .and_then(|lo| backtrack_pocket_path_from_lead_on(r, lat, lon, lo))
                .or_else(|| backtrack_pocket_path(r, lat, lon));
            match via_lead_bt {
                Some(p) => Some(LineUpPlan::Backtrack(p)),
                None => plain.map(LineUpPlan::Plain),
            }
        } else {
            plain.map(LineUpPlan::Plain)
        };
        let Some(plan) = plan else { continue };
        if best.as_ref().is_none_or(|(_, _, d)| r.dist_nm < *d) {
            best = Some((r.clone(), plan, r.dist_nm));
        }
    }
    best.map(|(r, p, _)| (r, p))
}

/// Pick the runway end to line up on: feasible geometry, matching the
/// cleared runway when the tower named one (lining up on 09 with a 27
/// clearance is the backwards trap, refused upstream by returning the
/// match only). Nearest threshold among the feasible wins.
pub fn pick_line_up(
    cands: &[crate::autoland::Runway],
    lat: f64,
    lon: f64,
    cleared: Option<&str>,
    heading_deg: Option<f64>,
) -> Option<(crate::autoland::Runway, Vec<(f64, f64)>)> {
    type LineUpCandidate = (crate::autoland::Runway, Vec<(f64, f64)>, f64);
    let mut best: Option<LineUpCandidate> = None;
    for r in cands {
        if let Some(c) = cleared {
            if !crate::autotakeoff::runway_matches(&r.id, c) {
                continue;
            }
        }
        let Some(path) = line_up_path(r, lat, lon, heading_deg) else {
            continue;
        };
        if best.as_ref().is_none_or(|(_, _, d)| r.dist_nm < *d) {
            best = Some((r.clone(), path, r.dist_nm));
        }
    }
    best.map(|(r, p, _)| (r, p))
}

/// Pure-pursuit lookahead: seconds of taxi ahead, floored so creep-speed
/// taxiing still aims somewhere and capped so the aim point can't cut a
/// corner two junctions away. 5 s, not 3.5: at 14 m of lookahead a 3 m
/// centreline offset read as 12° of error and half-lock steer — the SR22
/// S-weaved ±4 m for whole legs even with the rate lead (2026-07-22).
/// Longer aim turns the same offset into ~8° and a quarter-lock nudge.
/// Test-only since the live loop took the aim from the profile
/// ([`Pace::lookahead_s`]); the tests pin this proven 5 s baseline.
#[cfg(test)]
pub fn lookahead_m(gs_kts: f64) -> f64 {
    lookahead_m_with(gs_kts, 5.0)
}

/// [`lookahead_m`] with the airframe's own aim time. A slow plant needs a
/// LONG aim: the SF50 (castoring wheel, brake-assisted, ~3.5 deg/s of yaw at
/// full command) crossed the centreline 24 degrees ANGLED at 12 kt with the
/// 5 s aim — it cannot turn fast enough to roll out in 31 m, so every
/// crossing overshot ~10 m the other way, a slow drunk S down the taxiway
/// (2026-07-29). Aim further out and the same offset asks for a shallower
/// crossing the plant can actually fly. The cap scales with the aim so a
/// long-aim airframe keeps it at speed.
pub fn lookahead_m_with(gs_kts: f64, aim_s: f64) -> f64 {
    (gs_kts * 0.514 * aim_s).clamp(16.0, 10.0 * aim_s)
}

/// Deceleration the stop ramp plans for, m/s².
///
/// NOT the deceleration the brakes can achieve — that is about 0.7 m/s² at the
/// gentle end (measured: 0.14-0.18 brake gave a -1.36 kt/s trend at EGHI). It
/// is deliberately far below it, because the ramp has to command a speed the
/// governor will actually ACT on, and `taxi_control` only brakes once the
/// aircraft is 3 kt above target. A √ curve is flat far out and steep near
/// zero, so at 0.5 that gap did not open until ~7 m to go:
///
///   rem 22.9 m -> target 9.3 kt vs 9.54 actual   no brake
///   rem 11.2 m -> target 6.5 kt vs 9.24 actual   no brake
///   rem  6.8 m -> target 5.1 kt vs 9.14 actual   first brake, far too late
///
/// The aircraft then crossed the hold line at 6.7 kt and coasted five metres
/// past it, nose over the yellow — a runway incursion, and with the steering
/// already released it stopped skewed as well (EGHI, photographed 2026-07-30).
///
/// Worse, `taxi_control` cannot tell a planned STOP from a bend trim: both are
/// just a target above 1 kt, so both get the 0.22 comfort cap that three
/// airframes' jerkiness complaints put there. Full 0.6 authority arrives only
/// inside the last metre. Rather than re-open that tuning, this starts the
/// deceleration early enough that comfort braking is enough: at 0.25 the same
/// 22.9 m commands 6.6 kt, the 3 kt gap opens around 15 m out, and there is
/// then twice the distance to stop in with ~2.8x the needed authority.
const STOP_DECEL_MS2: f64 = 0.25;

/// The braking-distance speed cap approaching the stop point: v = √(2·a·d) at
/// [`STOP_DECEL_MS2`], floored at a creep so the last meters actually finish.
/// At/inside [`STOP_ZERO_M`] it's a stop.
pub fn stop_ramp_kts(remaining_m: f64) -> f64 {
    if remaining_m <= STOP_ZERO_M {
        return 0.0;
    }
    (MS_TO_KTS * (2.0 * STOP_DECEL_MS2 * remaining_m).sqrt()).max(CREEP_KTS)
}

/// Where the schedule commands a full stop, and the creep it will not go below
/// on the way in. These two are a PAIR, and getting them out of step is what
/// made the SF50 unable to line up at Luton (2026-07-28).
///
/// The old figures were a 2 kt creep held until half a metre to go, against an
/// arrival test of `remaining <= 1 m && gs < 1.5 kt`. So the schedule was still
/// DRIVING at 2 kt through the window the arrival test needed it under 1.5, and
/// 2 kt needs `v^2/2a` = 1.06 m to stop at the ramp's own 0.5 m/s^2 — more than
/// the distance left — before any braking lag. Every leg therefore sailed past
/// its stop point: the blackbox shows `remaining_m` pinned at -5 with the
/// aircraft still rolling, and because `stop_straighten` has faded the steering
/// out by then, it drifted while nobody was holding the nose. Three line-up
/// legs in a row finished 2.5 to 8.6 m off with the aim point behind them, and
/// full auto's `lined_up` check — correctly — called it a failure.
///
/// 1.2 kt needs 0.38 m, comfortably inside the 1 m zero. The creep still moves
/// (the reason it exists at all: too low and the last metres never finish), and
/// the zero now matches the arrival test's own 1 m so the two agree about where
/// the route ends.
const CREEP_KTS: f64 = 1.2;
const STOP_ZERO_M: f64 = 1.0;

/// The scheduled taxi target: the pilot's pace, capped by any nearby bend
/// and by the stop ramp.
pub fn schedule_kts(base: f64, route: &Route, progress: f64, gs_kts: f64) -> f64 {
    let mut v = base.min(stop_ramp_kts(route.remaining(progress)));
    if let Some(cap) = route.bend_cap_kts(progress, gs_kts) {
        v = v.min(cap);
    }
    v
}

/// True when the plugin's steering channel is live — a zero write is a
/// no-op steer, so probing costs nothing.
pub async fn steer_channel_ok(xp: &dyn crate::xplane::XPlane) -> bool {
    xp.set_float(TAXI_STEER, 0.0).await.is_ok()
}

/// Spawn the steering loop. The caller (the `auto_taxi` tool) has already
/// validated the route, armed the taxi governor, and proved the plugin's
/// steer channel writable.
pub fn start(base_url: String, voice: Option<Voice>, route: Route, base_kts: f64) {
    if ACTIVE.swap(true, Ordering::Relaxed) {
        return; // already driving — the tool retargets instead
    }
    LAST_ARRIVED.store(false, Ordering::Relaxed);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            ACTIVE.store(false, Ordering::Relaxed);
            return;
        };
        rt.block_on(drive(XPlaneClient::new(base_url), voice, route, base_kts));
    });
}

/// Drive a backtrack: leg one (down the strip to the turn pocket) now, and
/// when it ARRIVES, leg two — a plain line-up from wherever the aircraft
/// actually stopped, which the join machinery turns into the 180. Any other
/// ending of leg one (pedals, "my controls", park handle, off-route) stands
/// the whole maneuver down: those paths already spoke, and the pilot who
/// took over mid-backtrack does not want a second drive starting itself.
pub fn start_backtrack(
    base_url: String,
    voice: Option<Voice>,
    route: Route,
    rwy: crate::autoland::Runway,
    base_kts: f64,
) {
    // Leg two is built here, far from the tool that knows the airframe — carry
    // the pace across rather than letting the fresh route fall back to GA.
    let pace = route.pace;
    BACKTRACKING.store(true, Ordering::Relaxed);
    start(base_url.clone(), voice.clone(), route, base_kts);
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async move {
            // Wait out leg one — bounded, so a stuck drive can't leak this
            // thread forever.
            for _ in 0..2400 {
                if !ACTIVE.load(Ordering::Relaxed) {
                    break;
                }
                tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            }
            if ACTIVE.load(Ordering::Relaxed) || !LAST_ARRIVED.load(Ordering::Relaxed) {
                BACKTRACKING.store(false, Ordering::Relaxed);
                return; // taken over, arrested, or still (impossibly) driving
            }
            if !crate::tools::speedhold::taxi_hold_active() {
                BACKTRACKING.store(false, Ordering::Relaxed);
                return; // speed authority gone with leg one — nobody drives
            }
            let xp = XPlaneClient::new(base_url.clone());
            let (Ok(lat), Ok(lon)) = (xp.get_float(LAT).await, xp.get_float(LON).await) else {
                BACKTRACKING.store(false, Ordering::Relaxed);
                return;
            };
            // Leg two of a backtrack: the aircraft has turned around and is
            // already aligned down the strip, so the diagonal fallback (None)
            // is correct — there is no lead-on to follow here.
            let Some(path) = line_up_path(&rwy, lat, lon, None) else {
                BACKTRACKING.store(false, Ordering::Relaxed);
                alert(
                    &voice,
                    Priority::Caution,
                    "I couldn't set up the turn. The line-up is yours.",
                );
                return;
            };
            let watch = crate::station::TaxiWatch {
                path,
                via: String::new(),
                seg: Vec::new(),
                holding_point: String::new(),
                lead_on: Vec::new(),
            };
            let Some(mut route2) = Route::build(&watch) else {
                BACKTRACKING.store(false, Ordering::Relaxed);
                alert(
                    &voice,
                    Priority::Caution,
                    "I couldn't set up the turn. The line-up is yours.",
                );
                return;
            };
            let spoken = crate::atis::spoken_runway(&rwy.id);
            route2.pace = pace;
            route2.done_line = Some(format!("Lined up runway {spoken}. Holding position."));
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "backtrack_turn", "rwy": rwy.id}),
            );
            crate::tools::speedhold::set_taxi_target(base_kts);
            start(base_url, voice, route2, base_kts);
            // Leg two owns ACTIVE from here.
            BACKTRACKING.store(false, Ordering::Relaxed);
        });
    });
}

/// Say a line without blocking the loop (queued; gated off in ATC-only mode
/// like every crew alert).
fn alert(voice: &Option<Voice>, priority: Priority, line: &str) {
    if let Some(v) = voice {
        v.alert(priority, line);
    }
}

/// The steering loop. Steering writes ride the plugin's expiring channel, so
/// every exit path — including a panic or kill -9 — releases the override
/// within a second by silence alone; the explicit zero write on the way out
/// is just politeness.
async fn drive(xp: XPlaneClient, voice: Option<Voice>, route: Route, base_kts: f64) {
    tracing::info!("auto taxi started");
    crate::blackbox::record(
        "autotaxi",
        serde_json::json!({"event": "engaged", "total_m": route.total, "base_kts": base_kts}),
    );
    // The pilot's pace: adopted from a spoken "taxi at N" mid-drive.
    let mut base = base_kts;
    // What the schedule last wrote into the governor — any OTHER value read
    // back is an external retarget (spoken stop/go/speed) to fold in.
    let mut last_sched = base_kts;
    // External stop ("stop" / both pedals park the target at 0): the
    // schedule stands aside until a "go" restores a rolling target.
    let mut ext_stopped = false;
    // Seeded at the first fix: >0 means we engaged off the route or facing
    // against it and are JOINING — widened guard, bend-speed cap — until
    // captured, when it drops to 0 and the established guard takes over.
    let mut join_allow: Option<f64> = None;
    // Yaw rate for the steering lead, EMA-smoothed: raw tick-to-tick heading
    // differences of an HTTP read are noise the lead would amplify.
    let mut prev_psi: Option<(f64, std::time::Instant)> = None;
    let mut yaw_rate = 0.0f64;
    // Last steering command, so the slew limiter has something to move FROM.
    let mut prev_steer = 0.0f64;
    // Whether the brake-steer assist currently has a wheel braked, so every
    // exit path can release it exactly once — and the last commanded pair,
    // so the assist's own slew has something to move from.
    let mut bs_on = false;
    let mut bs_prev = (0.0f64, 0.0f64);
    // Stagnation watch: commanding motion but not moving (see below).
    let mut stalled_since: Option<std::time::Instant> = None;
    loop {
        if !ACTIVE.load(Ordering::Relaxed) {
            break;
        }
        // Speed authority gone — steering alone must not keep driving. This
        // is NOT only the deliberate paths (park handle, spoken release):
        // the governor's throttle-override check can't tell a pilot's hand
        // from a hardware-axis bump the sim re-drives, and mid-drive that
        // left an aircraft rolling at taxi power with NOBODY on the
        // throttle — the 2026-07-20 Gatwick runaway ran 10 to 50 knots
        // through the grass in silence. On the ground the safe reading is
        // stopping: throttle to idle plus ONE expiring brake pulse (the
        // pilot's own brake key, frees itself within a second, can't
        // stick), and say it out loud — a pilot who truly wants the power
        // pushes again with everything already released. Airborne (a
        // runaway that got that far), touch nothing.
        if !speedhold::taxi_hold_active() {
            let on_ground = xp.get_float(ON_GROUND).await.unwrap_or(0.0) > 0.5;
            tracing::info!(
                on_ground,
                "auto taxi: taxi hold gone, arresting and releasing"
            );
            if on_ground {
                let _ = xp.set_float(&throttle_dref(), 0.0).await;
                if xp
                    .run_command_for(crate::tools::BRAKES_REGULAR, crate::tools::BRAKE_HOLD_SECS)
                    .await
                    .is_ok()
                {
                    crate::tools::note_brake_cmd();
                }
                alert(
                    &voice,
                    Priority::Caution,
                    "I've lost the speed hold. Throttle's idle. Your aircraft.",
                );
            } else {
                alert(
                    &voice,
                    Priority::Caution,
                    "I've lost the speed hold. Your aircraft.",
                );
            }
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "hold_gone", "arrested": on_ground}),
            );
            break;
        }
        // Paused or replay: stop writing — the steer channel expires and the
        // pedals are the pilot's until the sim is live again.
        if crate::xplane::sim_frozen(&xp).await {
            stalled_since = None; // a pause is not a stall
            tokio::time::sleep(std::time::Duration::from_millis(500)).await;
            continue;
        }
        // One read burst, not a chain: sequential HTTP awaits stretched the
        // takeoff tick to half a second and bought a slew-limit weave
        // (sortie four) — the same medicine here keeps the steer tick honest.
        let (yield_r, lat_r, lon_r, psi_r, gs_r) = tokio::join!(
            xp.get_float(STEER_YIELD),
            xp.get_float(LAT),
            xp.get_float(LON),
            xp.get_float(PSI_TRUE),
            xp.get_float(GS_MS),
        );
        // The pilot's feet moved the yaw axis: the plugin already dropped
        // the override — stand the loop down and keep the speed hold, which
        // is the pre-existing "you steer, I have the throttle" division.
        if yield_r.is_ok_and(|v| v > 0.5) {
            tracing::info!("auto taxi: pilot on the pedals, steering released");
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "pedal_yield"}));
            alert(
                &voice,
                Priority::Advisory,
                "Your steering. I still have the throttle and brakes.",
            );
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        let (Ok(lat), Ok(lon), Ok(psi), Ok(gs_ms)) = (lat_r, lon_r, psi_r, gs_r) else {
            // Sim hiccup: no writes, the channel expires, retry.
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
            continue;
        };
        let gs_kts = gs_ms * MS_TO_KTS;
        // Signed here, magnitude everywhere below: the gates ask how far off
        // the route we are, and the record wants to know which side.
        let Some((progress, offset)) = route.fix_signed(lat, lon) else {
            tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
            continue;
        };
        let lateral = offset.abs();
        let remaining = route.remaining(progress);
        // Steering error first: the join state and the guard both need it.
        // The aim is capped by what's LEFT of the route: near the stop the
        // pursuit point must walk the path through the final corner, not
        // chord across it — the 16 m minimum aim against a corner ~20 m
        // before the hold parked the aircraft off the paint, and
        // stop_straighten then froze the leftover angle (LXGB Echo,
        // 2026-07-24, twice). Only bites close-in, where the stop ramp has
        // already slowed us below weave speeds.
        let look = lookahead_m_with(gs_kts, route.pace.lookahead_s).min((remaining * 0.6).max(6.0));
        // Inside the stop zone, fly the route's FINAL BEARING instead of
        // pursuing an aim point that has stopped moving. See
        // `final_bearing_deg`: past the end the pursuit error runs to 160°+
        // and steers the aircraft into a turn toward a waypoint behind it.
        // Holding the last segment's direction is what a pilot does — nose
        // straight down the paint, roll to a halt.
        let err = if remaining <= STOP_HEADING_HOLD_M {
            route.stop_heading_err_deg(lat, lon, psi, progress, look)
        } else {
            route.heading_err_deg(lat, lon, psi, progress, look)
        };
        let allow = *join_allow.get_or_insert_with(|| {
            let a = join_allowance_m(lateral, err);
            if a > 0.0 {
                tracing::info!(lateral, err, "auto taxi: joining the route");
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "join", "lateral_m": lateral, "err_deg": err}),
                );
            }
            a
        });
        if allow > 0.0 && join_done(lateral, err) {
            tracing::info!(lateral, "auto taxi: joined the route");
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "joined"}));
            join_allow = Some(0.0);
        }
        // Too far off the pavement we were cleared on: stop, hand steering
        // back, and say so. Established that's the tight guard; a join gets
        // its engage distance plus turning room, never more — the one drive
        // this loop makes is TOWARD the route.
        if lateral > OFF_ROUTE_STOP_M + join_allow.unwrap_or(0.0) {
            tracing::warn!(lateral, "auto taxi: off route, stopping");
            crate::blackbox::record(
                "autotaxi",
                serde_json::json!({"event": "off_route", "lateral_m": lateral}),
            );
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            alert(
                &voice,
                Priority::Caution,
                "We're off the cleared route. Stopping here. Your steering.",
            );
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        // Arrived: stopped at the hold-short line. The governor stays,
        // holding position on the brakes, exactly as a spoken "stop" —
        // "go" past this point is the pilot's call (a crossing clearance).
        if remaining <= 1.0 && gs_kts < 1.5 {
            tracing::info!("auto taxi: holding point, done");
            LAST_ARRIVED.store(true, Ordering::Relaxed);
            crate::blackbox::record("autotaxi", serde_json::json!({"event": "holding_point"}));
            speedhold::set_taxi_target(0.0);
            let _ = xp.set_float(TAXI_STEER, 0.0).await;
            let line = if let Some(done) = &route.done_line {
                done.clone()
            } else if route.holding_point.is_empty() {
                "Holding point. Holding position, brakes set.".to_string()
            } else {
                format!(
                    "Holding point {}. Holding position, brakes set.",
                    route.holding_point
                )
            };
            alert(&voice, Priority::Advisory, &line);
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        // A drive that commands motion but never MOVES is stuck. A backwards
        // join's near-full nosewheel lock is a chock the governor's gentle
        // taxi throttle can't break away against: a 169°-off engage at a
        // Jerez stand crept at millimetres per second for half a minute, no
        // voice line, until the pilot's pedals rescued it (16:48Z,
        // 2026-07-24). Twenty seconds of commanded-but-stationary says so out
        // loud and hands back, instead of pirouetting in place.
        if !ext_stopped && gs_kts < 0.7 {
            let t0 = *stalled_since.get_or_insert_with(std::time::Instant::now);
            if t0.elapsed().as_secs_f64() > 20.0 {
                tracing::warn!(err, "auto taxi: stagnant, standing down");
                crate::blackbox::record(
                    "autotaxi",
                    serde_json::json!({"event": "stagnant", "err_deg": err, "remaining_m": remaining}),
                );
                speedhold::set_taxi_target(0.0);
                let _ = xp.set_float(TAXI_STEER, 0.0).await;
                let line = if err.abs() > JOIN_ERR_SLOW_DEG {
                    "I can't get us moving, the nose is pointing away from the taxiway. \
                     Turn us toward it, then ask me to taxi again."
                } else {
                    "I can't get us moving. Standing down. Your aircraft."
                };
                alert(&voice, Priority::Caution, line);
                ACTIVE.store(false, Ordering::Relaxed);
                break;
            }
        } else {
            stalled_since = None;
        }
        // Steering: aim at the route ahead, command the error down — led by
        // the yaw rate so a laggy plant (castoring nosewheel steering by
        // rudder) can't snake — with the lock tapered by groundspeed so a
        // standstill can break away. From a backwards or off-route start
        // this is the whole join — the deflection tightens as the roll
        // builds. Approaching the stop (route end, or a spoken "stop"
        // bleeding speed off) the wheel straightens so the brakes never
        // bite onto a deflected nosewheel.
        let now = std::time::Instant::now();
        let dt = prev_psi.map_or(0.2, |(_, t0)| {
            now.duration_since(t0).as_secs_f64().clamp(0.05, 1.0)
        });
        let raw_rate = prev_psi.map_or(0.0, |(p, t0)| {
            let dt = now.duration_since(t0).as_secs_f64();
            if (0.05..=1.0).contains(&dt) {
                (((psi - p + 540.0).rem_euclid(360.0)) - 180.0) / dt
            } else {
                0.0
            }
        });
        prev_psi = Some((psi, now));
        yaw_rate = yaw_rate * 0.5 + raw_rate.clamp(-90.0, 90.0) * 0.5;
        let want = stop_straighten(
            steer_cmd_with(
                damped_err_deg(err, yaw_rate, route.pace.steer_lead_s),
                route.pace.steer_full_deg,
            ) * steer_authority(gs_kts),
            remaining,
            gs_kts,
        );
        let mut steer = slew_steer(prev_steer, want, route.pace.steer_slew_per_s, dt);
        prev_steer = steer;
        if ext_stopped {
            steer *= (gs_kts / 3.0).clamp(0.0, 1.0);
        }
        // The brake half of the steering, where the airframe needs it. Only
        // ever one side, slewed on like a foot (BRAKE_STEER_SLEW_PER_S),
        // released the moment the command shrinks back into the deadband.
        //
        // Written EVERY tick, never edge-gated on change. The plugin channel
        // is a heartbeat that expires a second after writes stop, so skipping
        // a steady command doesn't leave the last one standing, it drops the
        // brake mid-turn — and a turn is exactly where the command sits still
        // for seconds. (The old gate existed to keep HTTP off the tick, which
        // the join! below already buys: all three control writes go out
        // together, so the whole steering output costs ONE round trip, not
        // three. The 2026-07-28 lesson was about sequential awaits dragging
        // the loop to a second, not about the number of writes.)
        let (bl, br) = if route.pace.brake_steer {
            let (wl, wr) = brake_steer_split(steer, gs_kts);
            let bl = slew_steer(bs_prev.0, wl, Some(BRAKE_STEER_SLEW_PER_S), dt);
            let br = slew_steer(bs_prev.1, wr, Some(BRAKE_STEER_SLEW_PER_S), dt);
            bs_prev = (bl, br);
            bs_on = bl > 0.0 || br > 0.0;
            (bl, br)
        } else {
            (0.0, 0.0)
        };
        crate::blackbox::record(
            "autotaxi",
            serde_json::json!({
                // `lat_m` is SIGNED — right of the route is positive. It was
                // a distance until 2026-08-05, which made every taxi trail in
                // the log unreadable against the paint: "two metres off" says
                // nothing about which side, and the side is the whole
                // question when the pilot can see the line.
                "event": "drive", "gs": gs_kts, "err": err, "yaw_dps": yaw_rate,
                "steer": steer, "lat_m": offset, "remaining_m": remaining,
                "bs": bs_on, "bl": bl, "br": br,
            }),
        );
        let steer_write = if route.pace.brake_steer {
            let (s, a, b) = tokio::join!(
                xp.set_float(TAXI_STEER, steer),
                xp.set_float(LEFT_BRAKE, bl),
                xp.set_float(RIGHT_BRAKE, br),
            );
            let _ = (a, b);
            s
        } else {
            xp.set_float(TAXI_STEER, steer).await
        };
        if steer_write.is_err() {
            // The channel vanished (plugin reload): no steering, no drive.
            tracing::warn!("auto taxi: steer channel lost, stopping");
            speedhold::set_taxi_target(0.0);
            alert(
                &voice,
                Priority::Caution,
                "I've lost the steering. Stopping. Your aircraft.",
            );
            ACTIVE.store(false, Ordering::Relaxed);
            break;
        }
        // Speed: fold in any external retarget, then write the schedule.
        match speedhold::taxi_target() {
            Some(t) if (t - last_sched).abs() > 0.01 => {
                // Someone else moved the target: a spoken "stop" parks the
                // schedule; a spoken pace ("go", "taxi at 12") becomes the
                // new base.
                if t < 1.0 {
                    ext_stopped = true;
                } else {
                    base = t;
                    ext_stopped = false;
                }
                last_sched = t;
            }
            _ => {}
        }
        if !ext_stopped {
            let mut sched = schedule_kts(base, &route, progress, gs_kts);
            if join_allow.unwrap_or(0.0) > 0.0 {
                sched = sched.min(route.pace.join_cap_kts(err));
            }
            // Applied every tick, joined or not: the route's bends know nothing
            // about where the aircraft actually is, and the join cap lets go
            // too early to cover it.
            sched = sched.min(route.pace.track_cap_kts(lateral, err));
            if (sched - last_sched).abs() > 0.01 {
                speedhold::set_taxi_target(sched);
                last_sched = sched;
            }
        }
        tokio::time::sleep(std::time::Duration::from_millis(150)).await;
    }
    // Politeness on the way out — expiry would do it within the second.
    let _ = xp.set_float(TAXI_STEER, 0.0).await;
    // The brake-steer datarefs do NOT expire like the plugin channel does, so
    // a braked wheel left behind would drag forever. Every exit is a `break`
    // to here (checked — no returns inside the loop), so one release covers
    // them all.
    if bs_on {
        let _ = xp.set_float(LEFT_BRAKE, 0.0).await;
        let _ = xp.set_float(RIGHT_BRAKE, 0.0).await;
    }
    ACTIVE.store(false, Ordering::Relaxed);
    tracing::info!("auto taxi stopped");
}

#[cfg(test)]
mod tests {
    use super::*;

    const LAT0: f64 = 36.0;
    const LON0: f64 = -5.0;
    /// Lat/lon from meters east/north of the origin — the inverse of
    /// [`Geo::to_m`], so routes read like a taxi diagram.
    fn ll(x_m: f64, y_m: f64) -> (f64, f64) {
        (
            LAT0 + y_m / 111_320.0,
            LON0 + x_m / (111_320.0 * LAT0.to_radians().cos()),
        )
    }

    /// East 200 m, left turn north, 300 m to the holding point — the guide's
    /// test diagram, minus the runway crossing.
    fn watch() -> TaxiWatch {
        TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(200.0, 0.0), ll(200.0, 300.0)],
            via: "Whiskey, November".into(),
            seg: vec![("W".into(), false), ("N".into(), false)],
            holding_point: "N1".into(),
            lead_on: Vec::new(),
        }
    }

    #[test]
    fn capped_aim_drives_the_final_corner_instead_of_chording_it() {
        // A short L: 60 m east, then a left corner, 18 m north to the stop —
        // the hold-short-after-a-corner shape (LXGB Echo). Aircraft 5 m
        // before the corner, heading east along the paint.
        let watch = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(60.0, 0.0), ll(60.0, 18.0)],
            via: String::new(),
            seg: vec![(String::new(), false), (String::new(), false)],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let r = Route::build(&watch).expect("route");
        let (lat, lon) = ll(55.0, 0.0);
        let (progress, lateral) = r.fix(lat, lon).expect("on the route");
        assert!(lateral < 1.0);
        // The old 16 m minimum aim reaches PAST the corner: the aim point is
        // on the northbound leg, so the error commands an early cut inside
        // the paint. The capped aim (remaining*0.6 min 6) stays on the
        // eastbound leg: drive straight to the corner first.
        let err_chord = r.heading_err_deg(lat, lon, 90.0, progress, 16.0);
        let remaining = r.remaining(progress);
        let capped = lookahead_m(3.0).min((remaining * 0.6).max(6.0));
        assert!(capped < 16.0, "the cap engages near the stop: {capped}");
        let err_capped = r.heading_err_deg(lat, lon, 90.0, progress, capped);
        assert!(
            err_capped.abs() < err_chord.abs() - 5.0,
            "capped aim hugs the paint: capped {err_capped:.1} vs chord {err_chord:.1}"
        );
    }

    #[test]
    fn steering_is_proportional_and_bounded() {
        assert_eq!(steer_cmd_with(0.0, STEER_FULL_DEG), 0.0);
        assert!(
            steer_cmd_with(10.0, STEER_FULL_DEG) > 0.0
                && steer_cmd_with(10.0, STEER_FULL_DEG) < 1.0,
            "right of course, steer right"
        );
        assert!(steer_cmd_with(-10.0, STEER_FULL_DEG) < 0.0);
        assert_eq!(
            steer_cmd_with(90.0, STEER_FULL_DEG),
            1.0,
            "full deflection is the cap"
        );
        assert_eq!(steer_cmd_with(-170.0, STEER_FULL_DEG), -1.0);
    }

    #[test]
    fn pure_pursuit_steers_back_to_the_centreline() {
        let r = Route::build(&watch()).expect("route");
        // On the first leg (eastbound, true bearing 090), 8 m LEFT of the
        // centreline, heading exactly east: the aim point is on the route
        // ahead, so the error must steer RIGHT (positive), back toward it.
        let (lat, lon) = ll(50.0, 8.0);
        let (progress, lateral) = r.fix(lat, lon).expect("fix");
        assert!((lateral - 8.0).abs() < 1.0, "{lateral}");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err > 2.0, "must steer back right, got {err}");
        // Mirror: 8 m right of the line steers left.
        let (lat, lon) = ll(50.0, -8.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err < -2.0, "must steer back left, got {err}");
        // On the line, on heading: hands still.
        let (lat, lon) = ll(50.0, 0.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(8.0));
        assert!(err.abs() < 1.0, "{err}");
    }

    #[test]
    fn the_turn_is_anticipated_through_the_aim_point() {
        let r = Route::build(&watch()).expect("route");
        // 10 m short of the left turn at 10 knots: the aim point is already
        // around the corner, so the command leads the turn.
        let (lat, lon) = ll(190.0, 0.0);
        let (progress, _) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(10.0));
        assert!(
            err < -10.0,
            "aim point should pull left into the turn, got {err}"
        );
    }

    #[test]
    fn schedule_slows_for_bends_and_ramps_to_the_stop() {
        let r = Route::build(&watch()).expect("route");
        // Mid first leg: cruise pace.
        assert_eq!(schedule_kts(10.0, &r, 50.0, 10.0), 10.0);
        // Approaching the 90° corner at 200 m: capped to the turn speed.
        let near_turn = schedule_kts(10.0, &r, 170.0, 10.0);
        assert_eq!(near_turn, TURN_KTS, "{near_turn}");
        // Mid-turn (just past the vertex): still capped.
        assert_eq!(schedule_kts(10.0, &r, 210.0, 6.0), TURN_KTS);
        // Well past it on the straight: back to pace.
        assert_eq!(schedule_kts(10.0, &r, 300.0, 10.0), 10.0);
        // 20 m from the stop point: ramping down, still moving.
        let ramp = schedule_kts(10.0, &r, r.total - STOP_SHORT_M - 20.0, 8.0);
        assert!(ramp > 2.0 && ramp < 10.0, "{ramp}");
        // At the stop point: zero.
        assert_eq!(schedule_kts(10.0, &r, r.total - STOP_SHORT_M, 3.0), 0.0);
    }

    #[test]
    fn an_airframes_own_pace_drives_the_bend_caps() {
        // The 172 default is unchanged by the profile hook existing at all.
        let ga = Route::build(&watch()).expect("route");
        assert_eq!(ga.pace, Pace::default());
        assert_eq!(schedule_kts(10.0, &ga, 170.0, 10.0), TURN_KTS);

        // A jet declares its own: 18 down the straight, 10 through the bends.
        let mut jet = Route::build(&watch()).expect("route");
        jet.pace = Pace::from_profile(Some(&crate::aircraft::TaxiProfile {
            kts: Some(18.0),
            turn_kts: Some(10.0),
            sharp_kts: Some(6.0),
            ..Default::default()
        }));
        assert_eq!(jet.pace.straight, 18.0);
        assert_eq!(schedule_kts(18.0, &jet, 50.0, 18.0), 18.0, "open taxiway");
        assert_eq!(schedule_kts(18.0, &jet, 170.0, 18.0), 10.0, "into the bend");
        // The stop is still a stop, at any pace.
        assert_eq!(schedule_kts(18.0, &jet, jet.total - STOP_SHORT_M, 3.0), 0.0);
        // Turn-arounds stay at the sharp crawl on both airframes.
        assert_eq!(jet.pace.join_cap_kts(-170.0), 6.0);
    }

    #[test]
    fn a_partial_or_absurd_taxi_profile_keeps_safe_figures() {
        // Omitted fields keep the GA defaults, field by field.
        let partial = Pace::from_profile(Some(&crate::aircraft::TaxiProfile {
            kts: Some(15.0),
            turn_kts: None,
            sharp_kts: None,
            ..Default::default()
        }));
        assert_eq!(partial.straight, 15.0);
        assert_eq!(partial.turn, TURN_KTS);
        assert_eq!(partial.sharp, SHARP_KTS);
        // A typo can't become a takeoff roll with the tower's route as a plan.
        let absurd = Pace::from_profile(Some(&crate::aircraft::TaxiProfile {
            kts: Some(180.0),
            turn_kts: Some(90.0),
            sharp_kts: Some(0.0),
            ..Default::default()
        }));
        assert_eq!(absurd.straight, 25.0);
        assert_eq!(absurd.turn, 25.0);
        assert_eq!(absurd.sharp, 2.0);
        // No profile at all is the GA pace.
        assert_eq!(Pace::from_profile(None), Pace::default());
    }

    #[test]
    fn stop_ramp_is_gentle_and_finishes() {
        assert_eq!(stop_ramp_kts(0.0), 0.0);
        assert_eq!(stop_ramp_kts(0.4), 0.0, "inside the stop zone is stopped");
        assert!(
            stop_ramp_kts(2.0) >= CREEP_KTS,
            "creep floor carries the last meters"
        );
        let (near, far) = (stop_ramp_kts(10.0), stop_ramp_kts(40.0));
        assert!(near < far, "closer is slower: {near} vs {far}");
        assert!(far < 13.0, "40 m out needs no takeoff-roll pace: {far}");
    }

    /// The schedule must be able to STOP the aircraft inside the window the
    /// arrival test is watching, or every leg overshoots its own end.
    ///
    /// Luton 2026-07-28: a 2 kt creep held to 0.5 m to go, against arrival at
    /// `remaining <= 1 m && gs < 1.5 kt`. The schedule was still commanding
    /// 2 kt through the window that wanted under 1.5, and 2 kt cannot stop in
    /// half a metre at the ramp's own 0.5 m/s^2. Three SF50 line-up legs ran
    /// past their stop points — `remaining_m` pinned at -5, still rolling,
    /// steering already faded out by `stop_straighten` so the nose drifted —
    /// and full auto rightly called the line-up a failure.
    #[test]
    fn the_stop_ramp_can_actually_stop_before_the_arrival_window() {
        // Commanded stop no later than the arrival test's own 1 m.
        const ARRIVAL_M: f64 = 1.0;
        const ARRIVAL_KTS: f64 = 1.5;
        assert_eq!(stop_ramp_kts(ARRIVAL_M), 0.0, "zero by the arrival window");
        // The creep must be slow enough to stop within the zero distance at
        // the ramp's own deceleration, before any braking lag.
        let v = CREEP_KTS / MS_TO_KTS;
        let stopping_m = v * v / (2.0 * 0.5);
        assert!(
            stopping_m < STOP_ZERO_M,
            "creep of {CREEP_KTS} kt needs {stopping_m:.2} m but only gets {STOP_ZERO_M}"
        );
        // ...and the creep must still be under the arrival test's speed, so
        // reaching the point is enough to be judged arrived.
        const {
            assert!(CREEP_KTS < ARRIVAL_KTS, "the creep must satisfy arrival");
        }
    }

    /// The yoke-heading channel is dead on a castoring-wheel jet at taxi
    /// speed (steer −0.6 for 30 s while the offset grew to 13.8 m — Luton,
    /// 2026-07-28), so the assist brakes the inside wheel. One side only,
    /// ever: the governor's pilot-brake detector reads `min(left, right)`.
    #[test]
    fn brake_steer_brakes_the_inside_wheel_only() {
        // A breath of command is a breath of brake — on a castoring wheel
        // the brake IS the steering, so only jitter stays off it entirely.
        assert_eq!(brake_steer_split(0.04, 10.0), (0.0, 0.0));
        let (_, small) = brake_steer_split(0.15, 10.0);
        assert!(small > 0.0 && small < 0.06, "gentle, not zero: {small}");
        // Near-stopped: no asymmetric lurch, whatever the command.
        assert_eq!(brake_steer_split(0.9, 2.0), (0.0, 0.0));
        // A real correction at taxi speed: inside wheel, bounded, one side.
        let (l, r) = brake_steer_split(0.6, 10.0);
        assert_eq!(l, 0.0, "nose right brakes the RIGHT wheel only");
        assert!(r > 0.05 && r <= BRAKE_STEER_MAX, "{r}");
        let (l2, r2) = brake_steer_split(-0.6, 10.0);
        assert_eq!(r2, 0.0);
        assert!((l2 - r).abs() < 1e-12, "symmetric: {l2} vs {r}");
        // Full authority arrives INSIDE the tracking band, not at a command
        // the loop can never reach: steer sat at ~0.55 for two thirds of the
        // fourth SF50 taxi drawing only 0.07 of brake, and the aircraft held
        // a one-sided 9 m offset the whole way (2026-07-29).
        assert_eq!(r, BRAKE_STEER_MAX, "saturated by {BRAKE_STEER_SAT}: {r}");
        let (_, mid) = brake_steer_split(0.55, 10.0);
        assert!(
            mid > 0.8 * BRAKE_STEER_MAX,
            "tracking-band command near max: {mid}"
        );
        assert!(brake_steer_split(1.0, 60.0).1 <= BRAKE_STEER_MAX);
        // min(left, right) is zero in every case above — the governor can
        // never read the assist as the pilot slowing the aircraft.
        for s in [-1.0, -0.5, 0.5, 1.0] {
            let (a, b) = brake_steer_split(s, 12.0);
            assert_eq!(a.min(b), 0.0);
        }
        // The SF50 declares it; the 172 (linked nosewheel) must not.
        let sf = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        assert!(sf.brake_steer);
        let c172 = Pace::from_profile(
            crate::aircraft::load("cessna_172sp")
                .expect("172")
                .taxi
                .as_ref(),
        );
        assert!(!c172.brake_steer);
    }

    /// The schedule has to know where the aircraft IS, not just how the paint
    /// bends ahead of it — the Southampton SF50 taxi (2026-07-30).
    #[test]
    fn a_mistracking_aircraft_is_slowed_whatever_the_route_ahead_does() {
        let sf = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        // On the centreline and pointed along it: the open-taxiway pace,
        // untouched. The cap must never be the thing that slows a good taxi.
        assert_eq!(sf.track_cap_kts(0.5, 2.0), sf.straight);
        assert_eq!(sf.track_cap_kts(TRACK_TURN_LAT_M, 0.0), sf.straight);
        // Drifting off the line, or crossing it at an angle no intercept would
        // ask for: bend pace.
        assert_eq!(sf.track_cap_kts(8.0, 5.0), sf.turn);
        assert_eq!(sf.track_cap_kts(1.0, 25.0), sf.turn);
        // The numbers off the blackbox: 13 m off with 27 deg of crossing angle,
        // where it was doing 12.7 kt on a straight last leg. Sharp pace.
        assert_eq!(sf.track_cap_kts(13.0, 27.0), sf.sharp);
        // Which side is irrelevant.
        assert_eq!(sf.track_cap_kts(1.0, -40.0), sf.sharp);
        // It can only ever slow the schedule down.
        for lat in [0.0, 5.0, 7.0, 13.0, 40.0] {
            for err in [0.0, 10.0, 30.0, 90.0] {
                assert!(sf.track_cap_kts(lat, err) <= sf.straight);
                assert!(sf.track_cap_kts(lat, err) >= sf.sharp);
            }
        }
        // A trainer holding the centreline within a wheel's width is untouched
        // too — this is not a tax on airframes that track well.
        let c172 = Pace::from_profile(
            crate::aircraft::load("cessna_172sp")
                .expect("172")
                .taxi
                .as_ref(),
        );
        assert_eq!(c172.track_cap_kts(2.0, 8.0), c172.straight);
    }

    /// A taxiing aircraft carries people. The lever may not slam.
    #[test]
    fn the_steering_lever_is_rate_limited_when_an_airframe_asks() {
        // No limit declared: unchanged, whatever the demand.
        assert_eq!(slew_steer(0.0, 1.0, None, 0.2), 1.0);
        // Limited: 0.8/s over a 0.2 s tick is 0.16 of travel, not the lot.
        let s = slew_steer(0.0, 1.0, Some(0.8), 0.2);
        assert!((s - 0.16).abs() < 1e-9, "{s}");
        // Symmetric, and it converges rather than ringing past the target.
        assert!((slew_steer(0.0, -1.0, Some(0.8), 0.2) + 0.16).abs() < 1e-9);
        let near = slew_steer(0.5, 0.52, Some(0.8), 0.2);
        assert!(
            (near - 0.52).abs() < 1e-9,
            "small moves pass through: {near}"
        );
    }

    /// The steering law is per-airframe because the PLANT is. Measured over the
    /// blackbox at 4-12 kt, yaw per unit steer is 12.8 deg/s on the 172 and
    /// 6.9 on the SF50 — the castoring nosewheel is the laggier plant, and lag
    /// wants phase lead, not gain.
    #[test]
    fn the_sf50_taxis_more_gently_than_the_airframe_the_law_was_tuned_on() {
        let sf = Pace::from_profile(
            crate::aircraft::load("cirrus_vision_sf50")
                .expect("sf50")
                .taxi
                .as_ref(),
        );
        let c172 = Pace::from_profile(
            crate::aircraft::load("cessna_172sp")
                .expect("172")
                .taxi
                .as_ref(),
        );
        // The 172 keeps exactly the feel it was tuned with.
        assert_eq!(c172.steer_full_deg, STEER_FULL_DEG);
        assert_eq!(c172.steer_lead_s, STEER_RATE_LEAD_S);
        assert_eq!(c172.steer_slew_per_s, None);
        // The SF50 moves the lever less, and leads harder.
        assert!(sf.steer_full_deg > c172.steer_full_deg, "gentler gain");
        assert!(sf.steer_lead_s > c172.steer_lead_s, "more damping");
        let slew = sf.steer_slew_per_s.expect("a rate limit");
        // Generous on purpose: a tight slew is its own limit cycle, and the
        // aircraft's own p90 lever rate over the whole taxi was 0.18/s.
        assert!(
            slew >= 0.5,
            "a tight slew re-creates the oscillation: {slew}"
        );
        // Same error, less lever — that is the thing the cabin feels.
        let err = 18.0;
        assert!(steer_cmd_with(err, sf.steer_full_deg) < steer_cmd_with(err, c172.steer_full_deg));
        // And a longer aim: ~3.5 deg/s of yaw cannot fly the 5 s aim's
        // crossings — it met the centreline 24 degrees angled and S-weaved
        // ±10 m (2026-07-29). Same offset, shallower ask.
        assert!(sf.lookahead_s > c172.lookahead_s);
        assert_eq!(c172.lookahead_s, 5.0, "the 172 keeps its proven aim");
        assert!(
            lookahead_m_with(12.0, sf.lookahead_s) > 1.5 * lookahead_m_with(12.0, 5.0),
            "the SF50 aims well past the shared default at taxi pace"
        );
    }

    #[test]
    fn rate_lead_checks_the_swing() {
        // Nose swinging hard through a small error: the lead commands
        // countersteer before the crest — this is the SR22 snake killer.
        assert!(damped_err_deg(5.0, 40.0, STEER_RATE_LEAD_S) < 0.0);
        // Steady nose: pure P unchanged, the 172 keeps its proven feel.
        assert_eq!(damped_err_deg(10.0, 0.0, STEER_RATE_LEAD_S), 10.0);
        // A gentle commanded bend (~8 deg/s) keeps most of its steer.
        assert!(damped_err_deg(15.0, 8.0, STEER_RATE_LEAD_S) > 9.0);
    }

    #[test]
    fn lookahead_scales_with_speed_within_bounds() {
        assert_eq!(lookahead_m(0.0), 16.0);
        assert!(lookahead_m(10.0) > 16.0 && lookahead_m(10.0) < 50.0);
        assert_eq!(lookahead_m(60.0), 50.0);
    }

    #[test]
    fn steering_authority_tapers_with_speed_so_a_standstill_breaks_away() {
        assert!(steer_authority(0.0) < 0.4, "parked lock must stay partial");
        assert!(steer_authority(0.0) > 0.2, "but enough to actually turn");
        assert!(
            steer_authority(1.5) > steer_authority(0.5),
            "authority builds with the roll"
        );
        assert_eq!(steer_authority(3.0), 1.0, "full lock once rolling");
        assert_eq!(steer_authority(20.0), 1.0);
    }

    #[test]
    fn a_backwards_start_is_a_join_and_steers_hard_around() {
        let r = Route::build(&watch()).expect("route");
        // On the first leg, ON the centreline, but facing WEST against an
        // eastbound route: near-180 error, full deflection, join granted.
        let (lat, lon) = ll(50.0, 0.0);
        let (progress, lateral) = r.fix(lat, lon).expect("fix");
        let err = r.heading_err_deg(lat, lon, 270.0, progress, lookahead_m(3.0));
        assert!(err.abs() > 150.0, "backwards means a big error, got {err}");
        assert_eq!(
            steer_cmd_with(err, STEER_FULL_DEG).abs(),
            1.0,
            "turn-around wants full deflection"
        );
        assert!(
            join_allowance_m(lateral, err) > 0.0,
            "wrong-facing start must join"
        );
        assert_eq!(
            Pace::default().join_cap_kts(err),
            SHARP_KTS,
            "turn-arounds happen at the sharp pace"
        );
    }

    #[test]
    fn an_off_route_start_is_a_join_and_aims_back_at_the_route() {
        let r = Route::build(&watch()).expect("route");
        // 60 m LEFT of the first leg, heading east (parallel to it): the aim
        // point on the route pulls strongly RIGHT, back toward the pavement.
        let (lat, lon) = ll(50.0, 60.0);
        let (progress, lateral) = r.fix(lat, lon).expect("fix");
        assert!((lateral - 60.0).abs() < 2.0, "{lateral}");
        let err = r.heading_err_deg(lat, lon, 90.0, progress, lookahead_m(5.0));
        assert!(err > 20.0, "must aim back at the route, got {err}");
        let allow = join_allowance_m(lateral, err);
        assert!(
            allow >= lateral + JOIN_GROW_M - 1.0,
            "guard covers the engage distance: {allow}"
        );
    }

    #[test]
    fn join_state_machine_grants_and_captures() {
        // On route, aligned: no join, the tight guard from tick one.
        assert_eq!(join_allowance_m(2.0, 5.0), 0.0);
        // Off route or backwards: joined with turning room.
        assert!(join_allowance_m(40.0, 0.0) > 40.0);
        assert!(join_allowance_m(2.0, 170.0) > OFF_ROUTE_STOP_M);
        // Captured only when close AND aligned.
        assert!(join_done(5.0, 10.0));
        assert!(
            !join_done(5.0, 90.0),
            "close but still turning is not joined"
        );
        assert!(!join_done(20.0, 10.0), "aligned but wide is not joined");
        // Aligned rejoins ride at bend speed, not the sharp crawl.
        assert_eq!(Pace::default().join_cap_kts(20.0), TURN_KTS);
        assert_eq!(Pace::default().join_cap_kts(-170.0), SHARP_KTS);
    }

    #[test]
    fn the_wheel_straightens_into_the_stop() {
        // Stopped (or as good as): the fade owns it, so the brakes never bite
        // a deflected wheel. Well out: steering untouched. Closing: faded.
        const STOPPED: f64 = 0.0;
        assert_eq!(stop_straighten(0.8, 50.0, STOPPED), 0.8);
        let close = stop_straighten(0.8, 4.0, STOPPED);
        assert!(close > 0.0 && close < 0.5, "{close}");
        assert_eq!(stop_straighten(0.8, 1.0, STOPPED), 0.0);
        assert_eq!(stop_straighten(-0.8, 0.5, STOPPED), 0.0, "both directions");
    }

    /// The ramp must open the governor's braking gap while there is still room
    /// to stop, not a couple of metres out.
    ///
    /// `taxi_control` brakes when `(target - gs) - trend*2.5 < -3.0`, i.e. only
    /// once the aircraft is ~3 kt above target, and until the target drops
    /// below 1 kt it is capped at the 0.22 comfort effort. At EGHI, taxiing at
    /// 12 kt base, that gap did not open until 6.8 m to go: the aircraft
    /// crossed the hold line at 6.7 kt and finished five metres past it with
    /// its nose over the yellow.
    #[test]
    fn braking_starts_with_room_to_stop_at_the_hold() {
        const BASE: f64 = 12.0; // the EGHI taxi's own base speed
        const BRAKE_GAP_KTS: f64 = 3.0; // taxi_control's threshold
                                        // Measured achievable deceleration at the gentle comfort effort.
        const ACHIEVABLE_MS2: f64 = 0.6;

        // Where does the aircraft, still doing its base speed, first find
        // itself 3 kt above the commanded target?
        let mut brake_at = 0.0;
        let mut d = 0.0;
        while d < 80.0 {
            if BASE - stop_ramp_kts(d) >= BRAKE_GAP_KTS {
                brake_at = d;
            }
            d += 0.1;
        }
        // It has to be far enough out to actually stop from base speed.
        let need_m = (BASE / MS_TO_KTS).powi(2) / (2.0 * ACHIEVABLE_MS2);
        assert!(
            brake_at > need_m,
            "braking must begin with room: opens at {brake_at:.1} m but \
             stopping from {BASE} kt needs {need_m:.1} m"
        );
        // And with margin, not just barely — the governor has its own loop
        // lag on top of the physics.
        //
        // The margin is only ~1.35x, and that is the honest ceiling of this
        // approach rather than a number to tune toward. Stopping from 12 kt
        // takes ~32 m ONLY because `taxi_control` caps a planned stop at the
        // 0.22 comfort effort until the last metre, and a real aircraft stops
        // from 12 kt in a fraction of that. The deeper fix is to tell
        // `taxi_control` that a stop is planned so it can use its 0.6
        // authority earlier; that re-opens tuning three airframes have already
        // complained about, so it is deliberately not bundled with this.
        assert!(
            brake_at > need_m * 1.25,
            "want margin: {brake_at:.1} m vs {need_m:.1} m needed"
        );

        // The ramp is still a ramp: monotonic, and it still reaches a stop.
        let mut prev = 0.0;
        let mut d = 0.0;
        while d <= 60.0 {
            let v = stop_ramp_kts(d);
            assert!(v >= prev - 1e-9, "monotonic at {d} m");
            prev = v;
            d += 0.5;
        }
        assert_eq!(stop_ramp_kts(STOP_ZERO_M), 0.0, "still stops");
        assert!(stop_ramp_kts(2.0) >= CREEP_KTS, "creep survives");
    }

    /// The fade must not disarm the nose while the aircraft is still rolling.
    ///
    /// `remaining_m` saturates at `-STOP_SHORT_M` as soon as the projection
    /// reaches the end of the polyline, so the distance term pins at -5 with
    /// 2-3 kt still on the clock — and zeroing the command there also kills
    /// the yaw damping inside it. EGHI 2026-07-30: steer 0.00, yaw -2 deg/s
    /// unarrested, cross-track growing 2.45 -> 5.08 m while coasting to a
    /// stop, at the HOLDING POINT and in all three stops of that sortie.
    #[test]
    fn the_nose_is_still_held_while_the_aircraft_is_still_moving() {
        // The exact saturated value the blackbox pins at, at the speed it was
        // actually doing when the steering went to zero.
        let rolling = stop_straighten(0.8, -STOP_SHORT_M, 2.7);
        assert!(
            rolling.abs() > 0.0,
            "steering must survive past the end of the polyline while rolling"
        );
        assert!(
            rolling.abs() >= 0.8 * STRAIGHTEN_FLOOR - 1e-9,
            "and keep enough authority to arrest yaw: {rolling}"
        );
        // Sign is preserved — this only ever scales the command.
        assert!(stop_straighten(-0.8, -STOP_SHORT_M, 2.7) < 0.0);

        // ...but once genuinely stopped it goes to zero, so the original
        // carve-under-braking fix still holds.
        assert_eq!(stop_straighten(0.8, -STOP_SHORT_M, 0.4), 0.0);
        assert_eq!(
            stop_straighten(0.8, -STOP_SHORT_M, STRAIGHTEN_HOLD_KTS),
            0.0
        );

        // It is a FLOOR, never a boost: far out, full steering is unchanged
        // whether moving or not.
        assert_eq!(stop_straighten(0.8, 50.0, 12.0), 0.8);
        // And it never exceeds the command it was given.
        for gs in [0.0, 1.4, 1.6, 3.0, 10.0] {
            for rem in [-5.0, 0.0, 2.0, 8.0, 40.0] {
                assert!(
                    stop_straighten(0.8, rem, gs) <= 0.8 + 1e-9,
                    "never amplifies (rem {rem}, gs {gs})"
                );
            }
        }
    }

    #[test]
    fn line_up_enters_ahead_and_finishes_down_the_centreline() {
        use crate::autoland::{along_track_nm, cross_track_nm, Runway};
        // A 2,000 m runway taking off due north from (0,0); hold-short spot
        // 40 m EAST of the centreline, abeam the numbers.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let (hlat, hlon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 40.0 / 1852.0);
        let path = line_up_path(&rwy, hlat, hlon, None).expect("feasible");
        assert_eq!(path.len(), 3);
        // The entry point is ON the centreline, ahead of abeam.
        let (elat, elon) = path[1];
        assert!(
            (cross_track_nm(elat, elon, &rwy) * 1852.0).abs() < 2.0,
            "entry on centreline"
        );
        let entry_past = -along_track_nm(elat, elon, &rwy) * 1852.0;
        assert!(
            entry_past > 20.0 && entry_past < 120.0,
            "ahead of abeam: {entry_past}"
        );
        // The stop point is further down, still leaving plenty of runway.
        let (blat, blon) = path[2];
        let stop_past = -along_track_nm(blat, blon, &rwy) * 1852.0;
        assert!(
            stop_past > entry_past + 40.0,
            "finishes aligned: {stop_past}"
        );
        // Too far off the strip is not a line-up.
        let (flat, flon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 300.0 / 1852.0);
        assert!(line_up_path(&rwy, flat, flon, None).is_none());
        // Near the far end there is no runway left to line up onto.
        let (nlat, nlon) = crate::autoland::project_latlon(0.0, 0.0, 0.0, 1800.0 / 1852.0);
        let (nlat, nlon) = crate::autoland::project_latlon(nlat, nlon, 90.0, 40.0 / 1852.0);
        assert!(line_up_path(&rwy, nlat, nlon, None).is_none());
    }

    #[test]
    fn line_up_rolls_forward_along_the_heading_not_a_diagonal() {
        use crate::autoland::{bearing_deg, cross_track_nm, Runway};
        // Runway 36 (course 0, due north) from (0,0). Hold-short 40 m EAST of
        // the centreline, abeam the numbers.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let (hlat, hlon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 40.0 / 1852.0);
        // Nose pointed due WEST — straight at the runway. Leg one should follow
        // that heading onto the centreline (the taxiway lead-on), NOT cut a
        // ~45 degree diagonal ahead. Bearing of leg one ≈ 270.
        let path = line_up_path(&rwy, hlat, hlon, Some(270.0)).expect("feasible");
        let brg = bearing_deg(path[0].0, path[0].1, path[1].0, path[1].1);
        let off = ((brg - 270.0 + 540.0).rem_euclid(360.0) - 180.0).abs();
        assert!(off < 12.0, "leg one follows the nose (270), got {brg}");
        assert!(
            (cross_track_nm(path[1].0, path[1].1, &rwy) * 1852.0).abs() < 3.0,
            "entry on centreline"
        );

        // Nose nearly aligned with the runway (5 degrees): no clean forward
        // crossing, so it falls back to the diagonal swing — clearly off the
        // near-aligned heading.
        let path2 = line_up_path(&rwy, hlat, hlon, Some(5.0)).expect("feasible");
        let brg2 = bearing_deg(path2[0].0, path2[0].1, path2[1].0, path2[1].1);
        let off2 = ((brg2 - 5.0 + 540.0).rem_euclid(360.0) - 180.0).abs();
        assert!(
            off2 > 20.0,
            "near-aligned nose falls back to the diagonal, got {brg2}"
        );
    }

    /// Real-install probe: what does the LXGB planner actually hand the
    /// line-up at the runway 27 hold? Prints the route tail, the lead-on,
    /// and the drive path the picker would build — evidence, not theory.
    /// `cargo test -- --ignored live_lxgb_line_up --nocapture`
    #[test]
    #[ignore]
    fn live_lxgb_line_up_lead_on() {
        let apt =
            "/Users/duanelicudi/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat";
        let net = crate::nav::find_taxi_net(apt, "LXGB")
            .unwrap()
            .expect("LXGB network");
        let zones: Vec<String> = vec!["27".into(), "09".into()];
        // Apron, mid-field north side -> the 27 end.
        let plan = net
            .plan(
                36.1535,
                -5.3495,
                36.1515,
                -5.3412,
                crate::nav::HOLD_SHORT_STANDOFF_M,
                &zones,
            )
            .expect("a route");
        println!("holding_point: {} via {:?}", plan.holding_point, plan.via);
        let m = |p: (f64, f64)| {
            let dx = (p.1 + 5.3412) * 111_320.0 * 36.15_f64.to_radians().cos();
            let dy = (p.0 - 36.1515) * 111_320.0;
            (dx.round(), dy.round())
        };
        println!("route tail (m from 27 thr, x=E y=N):");
        for p in plan.path.iter().rev().take(4).rev() {
            println!("  {:?}", m(*p));
        }
        println!("lead_on ({} pts):", plan.lead_on.len());
        for p in &plan.lead_on {
            println!("  {:?}", m(*p));
        }
        assert!(
            !plan.lead_on.is_empty(),
            "zone-trimmed route must keep its lead-on"
        );
        // Now the picker, from the route's actual end (where the taxi stops).
        let &(hlat, hlon) = plan.path.last().unwrap();
        let cands = crate::autoland::nearby_runways(apt, hlat, hlon, 1.0).unwrap();
        let picked = pick_line_up_plan(
            &cands,
            hlat,
            hlon,
            Some("27"),
            400.0,
            false,
            None,
            Some(&plan.lead_on),
        );
        let (rwy, lp) = picked.expect("a line-up plan");
        println!("picked runway {}", rwy.id);
        match lp {
            LineUpPlan::Plain(p) => {
                println!("drive path ({} pts):", p.len());
                for q in &p {
                    println!("  {:?}", m(*q));
                }
                assert!(
                    p.len() > 3,
                    "authored points present, not a 3-point diagonal: {}",
                    p.len()
                );
            }
            LineUpPlan::Backtrack(_) => println!("backtrack picked"),
        }
    }

    #[test]
    fn line_up_drives_the_authored_lead_on_when_it_is_ours() {
        use crate::autoland::{project_latlon, Runway};
        // Runway 36 from (0,0); hold 40 m east of the centreline, abeam 100 m.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2000.0,
            surface: 1,
            dist_nm: 0.02,
            elev_ft: 0.0,
        };
        let at = |a_m: f64, x_m: f64| {
            let (la, lo) = project_latlon(0.0, 0.0, 0.0, a_m / 1852.0);
            project_latlon(la, lo, 90.0, x_m / 1852.0)
        };
        let (hlat, hlon) = at(100.0, 40.0);
        // The authored yellow line: from the hold, curving onto the centreline
        // at 160 m past the threshold (two intermediate points shape the curve).
        let lead_on = vec![
            at(105.0, 38.0),
            at(125.0, 25.0),
            at(150.0, 8.0),
            at(160.0, 0.0),
        ];
        let (path, stop) =
            line_up_path_from_lead_on(&rwy, hlat, hlon, &lead_on).expect("drives the lead-on");
        // The authored points lead the drive path in order; the sharp junction
        // corner is replaced by a sampled fillet arc, then the stop continues
        // down the centreline.
        assert!(
            path.len() >= 8,
            "aircraft + authored + arc + stop: {}",
            path.len()
        );
        assert_eq!(path[1], lead_on[0]);
        assert_eq!(path[2], lead_on[1]);
        assert_eq!(path[3], lead_on[2]);
        assert!(
            (stop - 240.0).abs() < 1.0,
            "junction 160 + align 80: {stop}"
        );
        // The arc stays between the two legs: every sampled point is on the
        // hold side of the centreline and short of the stop.
        for p in &path[4..path.len() - 1] {
            let x = crate::autoland::cross_track_nm(p.0, p.1, &rwy) * 1852.0;
            assert!(
                (-1.0..=40.0).contains(&x),
                "arc point off the corner: x={x}"
            );
        }
        // A lead-on from ANOTHER stub (300 m away) is refused — never drive
        // someone else's painted line.
        let foreign = vec![at(400.0, 38.0), at(460.0, 0.0)];
        assert!(line_up_path_from_lead_on(&rwy, hlat, hlon, &foreign).is_none());
        // A lead-on ending far OFF the centreline is not a runway entry.
        let sideways = vec![at(105.0, 38.0), at(160.0, 60.0)];
        assert!(line_up_path_from_lead_on(&rwy, hlat, hlon, &sideways).is_none());
        // And the picker prefers it over the synthetic diagonal for the same
        // runway.
        let cands = vec![rwy.clone()];
        let (r, plan) = pick_line_up_plan(
            &cands,
            hlat,
            hlon,
            Some("36"),
            400.0,
            false,
            Some(270.0),
            Some(&lead_on),
        )
        .expect("picked");
        assert_eq!(r.id, "36");
        match plan {
            LineUpPlan::Plain(p) => assert_eq!(p[1], lead_on[0], "authored path won"),
            LineUpPlan::Backtrack(_) => panic!("no backtrack expected"),
        }
    }

    /// Luton, 2026-07-27: a full-length departure off a mid-runway holding
    /// point. The backtrack's leg one aimed at a centreline point 30-60 m
    /// nearer the threshold — 23 degrees off the stub the aircraft was
    /// standing on — and drove an MD-82 off the pavement inside fifty metres.
    /// A backtrack has to reach the runway along the DRAWN stub too; only the
    /// direction it turns at the top is ours to choose.
    #[test]
    fn a_backtrack_enters_along_the_drawn_stub_not_across_the_grass() {
        use crate::autoland::{cross_track_nm, project_latlon, Runway};
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 2156.0,
            surface: 1,
            dist_nm: 0.05,
            elev_ft: 0.0,
        };
        let at = |a_m: f64, x_m: f64| {
            let (la, lo) = project_latlon(0.0, 0.0, 0.0, a_m / 1852.0);
            project_latlon(la, lo, 90.0, x_m / 1852.0)
        };
        // Hold 430 m down the strip, 45 m off to the side; the drawn stub runs
        // straight in, perpendicular, to the junction.
        let (hlat, hlon) = at(430.0, 45.0);
        let lead_on = vec![at(430.0, 0.0)];
        let path = backtrack_pocket_path_from_lead_on(&rwy, hlat, hlon, &lead_on)
            .expect("backtrack via the stub");
        // Every point of the entry, up to the junction, stays on the stub's
        // own along-track station — it never drifts down the runway before
        // reaching the centreline, which is exactly what crossed the grass.
        let a_of = |p: (f64, f64)| -crate::autoland::along_track_nm(p.0, p.1, &rwy) * 1852.0;
        let mut reached = false;
        for p in &path {
            let x = cross_track_nm(p.0, p.1, &rwy) * 1852.0;
            if x.abs() <= 1.0 {
                reached = true;
            }
            if !reached {
                assert!(
                    (a_of(*p) - 430.0).abs() < 25.0,
                    "entry wandered down the runway before reaching it: a={}",
                    a_of(*p)
                );
            }
        }
        assert!(reached, "the entry never reaches the centreline");
        // ...and it still ends in the turn pocket by the numbers.
        let last = *path.last().unwrap();
        assert!(
            a_of(last) < BACKTRACK_TURN_START_M + 5.0,
            "ends at the numbers"
        );
        // The synthetic fallback is what it replaces: that one leaves the stub
        // immediately, which is the bug.
        let synth = backtrack_pocket_path(&rwy, hlat, hlon).expect("synthetic exists");
        assert!(
            (a_of(synth[1]) - 430.0).abs() > 25.0,
            "the old diagonal aimed down the runway from the start"
        );
    }

    #[test]
    fn line_up_pick_honors_the_cleared_runway() {
        use crate::autoland::Runway;
        let mk = |id: &str, course: f64, thr: (f64, f64), dist: f64| Runway {
            icao: "TEST".into(),
            id: id.into(),
            thr_lat: thr.0,
            thr_lon: thr.1,
            course_deg: course,
            length_m: 2000.0,
            surface: 1,
            dist_nm: dist,
            elev_ft: 0.0,
        };
        // Both ends of one strip: 36 at the near threshold, 18 at the far.
        let far = crate::autoland::project_latlon(0.0, 0.0, 0.0, 2000.0 / 1852.0);
        let cands = vec![mk("36", 0.0, (0.0, 0.0), 0.02), mk("18", 180.0, far, 1.05)];
        let (hlat, hlon) = crate::autoland::project_latlon(0.0, 0.0, 90.0, 40.0 / 1852.0);
        // No clearance on record: geometry picks the near end.
        let (r, _) = pick_line_up(&cands, hlat, hlon, None, None).expect("pick");
        assert_eq!(r.id, "36");
        // The tower cleared 36: same answer, crosschecked.
        assert_eq!(
            pick_line_up(&cands, hlat, hlon, Some("36"), None)
                .unwrap()
                .0
                .id,
            "36"
        );
        // The tower cleared 18 and we're at the 36 end: nothing safe to
        // drive onto — the far end is a taxi clearance, not a line-up.
        assert!(pick_line_up(&cands, hlat, hlon, Some("18"), None).is_none());
    }

    #[test]
    fn backtrack_drives_to_a_pocket_beside_the_numbers() {
        use crate::autoland::{along_track_nm, cross_track_nm, Runway};
        // The LXGB shape: a 1,712 m runway, hold-short stub 1,170 m down it
        // (holding point Charlie left 580 m ahead for a 09 departure,
        // 2026-07-22). Runway 36 from (0,0) for test geometry.
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 1712.0,
            surface: 1,
            dist_nm: 0.6,
            elev_ft: 0.0,
        };
        let mid = crate::autoland::project_latlon(0.0, 0.0, 0.0, 1170.0 / 1852.0);
        let (hlat, hlon) = crate::autoland::project_latlon(mid.0, mid.1, 90.0, 40.0 / 1852.0);
        let path = backtrack_pocket_path(&rwy, hlat, hlon).expect("feasible");
        // The route heads TOWARD the threshold: each point's along-track
        // shrinks, and the pocket ends up beside the numbers, off the
        // centreline (turning room, never a pivot).
        let along: Vec<f64> = path
            .iter()
            .map(|&(a, b)| -along_track_nm(a, b, &rwy) * 1852.0)
            .collect();
        assert!(
            along.windows(2).all(|w| w[1] < w[0] + 1.0),
            "monotonic toward the numbers: {along:?}"
        );
        let (plat, plon) = *path.last().unwrap();
        let pocket_along = -along_track_nm(plat, plon, &rwy) * 1852.0;
        let pocket_off = (cross_track_nm(plat, plon, &rwy) * 1852.0).abs();
        assert!(
            pocket_along < 60.0,
            "pocket beside the numbers: {pocket_along}"
        );
        assert!(
            pocket_off > 8.0 && pocket_off < 20.0,
            "turning room: {pocket_off}"
        );
        // A line-up FROM the pocket is feasible — that's leg two, the 180.
        assert!(
            line_up_path(&rwy, plat, plon, None).is_some(),
            "leg two must exist"
        );
        // Near the numbers already: no backtrack, the plain line-up owns it.
        let near = crate::autoland::project_latlon(0.0, 0.0, 0.0, 60.0 / 1852.0);
        let (nlat, nlon) = crate::autoland::project_latlon(near.0, near.1, 90.0, 40.0 / 1852.0);
        assert!(backtrack_pocket_path(&rwy, nlat, nlon).is_none());
    }

    #[test]
    fn line_up_plan_backtracks_to_the_numbers_from_a_deep_intersection() {
        use crate::autoland::Runway;
        let rwy = Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 1712.0,
            surface: 1,
            dist_nm: 0.6,
            elev_ft: 0.0,
        };
        let cands = vec![rwy];
        // 1,170 m down the strip: a deep intersection on a short field. The
        // plain line-up would leave ~460 m — enough for a 172's roll — but a
        // departure uses the FULL length, so we backtrack to the numbers even
        // when it fits (the LXGB Charlie case the pilot kept flagging).
        let mid = crate::autoland::project_latlon(0.0, 0.0, 0.0, 1170.0 / 1852.0);
        let (hlat, hlon) = crate::autoland::project_latlon(mid.0, mid.1, 90.0, 40.0 / 1852.0);
        assert!(matches!(
            pick_line_up_plan(&cands, hlat, hlon, Some("36"), 400.0, false, None, None),
            Some((_, LineUpPlan::Backtrack(_)))
        ));
        assert!(matches!(
            pick_line_up_plan(&cands, hlat, hlon, Some("36"), 700.0, false, None, None),
            Some((_, LineUpPlan::Backtrack(_)))
        ));
        // The pilot asked for the full length in so many words: backtrack
        // even though the plain remaining would do.
        assert!(matches!(
            pick_line_up_plan(&cands, hlat, hlon, Some("36"), 400.0, true, None, None),
            Some((_, LineUpPlan::Backtrack(_)))
        ));
        // Abeam the numbers a backtrack is impossible — the ask still gets
        // the plain line-up, never a refusal.
        let near = crate::autoland::project_latlon(0.0, 0.0, 0.0, 60.0 / 1852.0);
        let (nlat, nlon) = crate::autoland::project_latlon(near.0, near.1, 90.0, 40.0 / 1852.0);
        assert!(matches!(
            pick_line_up_plan(&cands, nlat, nlon, Some("36"), 700.0, true, None, None),
            Some((_, LineUpPlan::Plain(_)))
        ));
    }

    #[test]
    fn short_routes_refuse_to_build() {
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(15.0, 0.0)],
            via: "A".into(),
            seg: vec![("A".into(), false)],
            holding_point: "A1".into(),
            lead_on: Vec::new(),
        };
        assert!(Route::build(&w).is_none(), "nothing to drive on 15 m");
    }

    /// Every stop turned the aircraft, and the guidance was doing it.
    ///
    /// The pursuit aim point clamps at the route's final point, so once the
    /// aircraft rolls past it the bearing to that point swings BEHIND and the
    /// law commands a hard turn toward a waypoint already passed. EGHI
    /// 2026-07-30, dead on the centreline at the stop point and then:
    ///
    ///   rem  0.04 -> err   9.5     rem -3.6 -> err  38.6
    ///   rem -4.4  -> err  70.2     rem -5.0 -> err 162.6
    ///
    /// 0.07 m off the line to 3.00 m in four seconds, steering pinned. It hit
    /// the hold, the line-up, the backtrack and the taxi-in alike, because all
    /// four drive this one loop.
    #[test]
    fn the_stop_holds_the_routes_heading_instead_of_chasing_a_passed_point() {
        // A straight route heading due east, 120 m long.
        let w = TaxiWatch {
            path: vec![ll(0.0, 0.0), ll(120.0, 0.0)],
            via: String::new(),
            seg: vec![(String::new(), false)],
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let r = Route::build(&w).expect("a straight two-point route drives");
        let bearing = r.final_bearing_deg().expect("a two-point route has one");
        assert!(
            (bearing - 90.0).abs() < 2.0,
            "due east route, got {bearing:.1}"
        );

        // Sitting PAST the end, pointed correctly down the route. The pursuit
        // error is nonsense here; the stop error must be ~0.
        let past = ll(125.0, 0.0);
        let (past_lat, past_lon) = (past.0, past.1);
        let progress = r.total;
        let pursuit = r.heading_err_deg(past_lat, past_lon, 90.0, progress, 6.0);
        let held = r.stop_heading_err_deg(past_lat, past_lon, 90.0, progress, 6.0);
        assert!(
            pursuit.abs() > 90.0,
            "the pursuit error should be wild past the end: {pursuit:.1}"
        );
        assert!(
            held.abs() < 2.0,
            "holding the route heading must read straight: {held:.1}"
        );

        // And it still CORRECTS a genuinely crooked nose rather than going
        // blind — 20° off reads as 20° off, with the sign that unwinds it.
        let crooked = r.stop_heading_err_deg(past_lat, past_lon, 110.0, progress, 6.0);
        assert!(
            (crooked + 20.0).abs() < 2.0,
            "20 deg right of course should command left: {crooked:.1}"
        );

        // The swap only happens near the stop; further out the pursuit still
        // owns the steering, which is what folds cross-track back in.
        assert!(
            STOP_HEADING_HOLD_M > 0.0 && STOP_HEADING_HOLD_M < 20.0,
            "the hold zone is the last few metres, not the whole leg"
        );
    }
}
