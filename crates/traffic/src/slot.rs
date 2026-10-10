//! Runway slots: who reaches the threshold first, and when each is off the
//! runway again. ONE timed rule fits every arrival round every other:
//! number two behind a leader the tower named ([`crate::aircraft`]'s
//! `leader_ready`), a circuit aircraft at the end of its downwind round a
//! straight-in, and a straight-in round whatever is already committed to
//! the runway ahead of it.
//!
//! Each aircraft publishes its own [`RunwaySlot`] in the picture: its time
//! to the threshold along its own route, at its own legs' speeds; its time
//! until it reports off the runway again (the flare, the rollout and its
//! own way off DRIVEN as it will drive it, a backtrack included, or a touch
//! and go's roll); and how long before its threshold it needs the runway
//! clear (its landing clearance, due by [`NO_CLEARANCE_AGL_FT`]). A
//! follower fits behind a slot when that slot is off [`FOLLOW_GAP_S`]
//! before the follower needs the runway, and seen leaving by the tower
//! before the follower is inside the tower's go-around window
//! ([`off_before`]).
//!
//! BIEG 03 (the hour sweep, 2026-10-09): a fixed 45 s on the runway was
//! two minutes short of an arrival there (no parallel taxiway, every
//! landing backtracks to the only exit), circuit traffic turned base in
//! front of straight-ins on a long final it never sequenced against, and
//! one of each pair went around, eleven times in the hour. Now a circuit
//! aircraft extends its downwind behind a straight-in that will be there
//! first (it can wait; the straight-in cannot), and a straight-in that
//! cannot fit behind traffic already committed ahead of it is given its
//! delay early, an orbit well out on the approach, never a go-around on
//! short final.

use crate::aircraft::{
    time_to_exit_s, Seen, TrafficAircraft, FLARE_AGL_FT, FLARE_SINK_FPM, FT_PER_M, GLIDE_DEG,
    LINE_UP_KTS, NO_CLEARANCE_AGL_FT, TCH_FT,
};
use crate::field::{FieldModel, RunwayEnd};
use crate::kin::{bearing, dist_m, follow, project_m, Follow, FollowSpec, Kin, LatLon, KT_TO_MS};
use crate::perf::Wake;
use crate::phase::{Leg, Phase, Plan};

/// The gap between the one ahead reporting off the runway and the follower
/// needing it, seconds: the tower's next tick, its release margin.
pub const FOLLOW_GAP_S: f64 = 20.0;

/// The tower decides a go-around this long before the threshold, seconds
/// (atc's `GO_AROUND_DECIDE_S`, 60), plus its margin and the time its
/// track takes to see a turn-off: an occupant it cannot see leaving
/// (backtracking toward us, turning onto the centreline to line up) is
/// never judged clear, so the follower must still be further out than this
/// when the tower can see it go. At 70 a Cessna backtracking off BIEG 03
/// was off the strip one second after the tower sent the next one round.
pub const TOWER_DECIDE_S: f64 = 80.0;

/// A circuit aircraft turns base in FRONT of a straight-in only with this
/// much more in hand than the gap: the straight-in cannot extend, so the
/// estimate's drift is taken out of the circuit's side.
pub const AHEAD_MARGIN_S: f64 = 20.0;

/// Two slots are on one runway when a threshold of one is this near a
/// threshold or far end of the other, metres.
const SAME_STRIP_M: f64 = 150.0;

/// A straight-in is given its delay only this far out along its route,
/// metres (4 nm): an orbit there is flown well before the final.
pub const ORBIT_MIN_M: f64 = 4.0 * 1852.0;

/// The most orbits one approach is given.
pub const MAX_ORBITS: u32 = 3;

/// An orbit's radius over the turn the type flies at its speed.
const ORBIT_RADIUS_FACTOR: f64 = 1.25;

/// Corners an orbit is drawn with.
const ORBIT_POINTS: usize = 12;

/// A drive back along the runway toward the threshold this long is a
/// backtrack, metres.
const BACKTRACK_M: f64 = 50.0;

/// A drive on the ground is estimated by driving it, in the fleet's own
/// substeps, never longer than [`DRIVE_MAX_S`].
const DRIVE_STEP_S: f64 = crate::clock::SUBSTEP_S;
const DRIVE_MAX_S: f64 = 900.0;

/// A drive's estimate is driven again this often while it is driven,
/// seconds.
const DRIVE_REFRESH_S: f64 = 5.0;

/// A flight's place at its runway, as the others see it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunwaySlot {
    /// The threshold it lands over (or rolls from), and the far end.
    pub thr: LatLon,
    pub far: LatLon,
    /// Seconds until it crosses the threshold (0 once over it).
    pub to_thr_s: f64,
    /// Seconds until it reports off the runway again ("runway vacated",
    /// which is when the tower clears the next), or is airborne off it.
    pub clear_s: f64,
    /// Seconds until the tower can see it leaving: through a backtrack
    /// toward the threshold, or a line-up still turning onto the
    /// centreline, the tower judges nobody behind it clear (0 for a
    /// rollout to a turn-off ahead, a roll, a touch and go).
    pub unjudged_s: f64,
    /// How long before its threshold it needs the runway clear, seconds:
    /// a lander's clearance is due by [`NO_CLEARANCE_AGL_FT`] (0 for one
    /// already on the runway).
    pub needs_clear_s: f64,
    /// It will not wait for anybody: on base or final, a straight-in,
    /// on the runway.
    pub committed: bool,
    /// A departure at the hold, waiting for the tower to release it: the
    /// circuit makes room for it (it extends its downwind), a straight-in
    /// does not (the tower threads it in ahead).
    pub waiting: bool,
    /// Its wake turbulence category: the spacing an instrument arrival
    /// behind it keeps on final ([`crate::arrival::wake_gap_nm`]).
    pub wake: Wake,
    /// Coming back round off a missed approach, not yet on the final: it
    /// takes its place at the far end of the sequence
    /// ([`crate::arrival`]'s spacing), and nobody slows for it until then.
    pub rejoining: bool,
}

impl RunwaySlot {
    /// The same slot with its threshold `to_thr_s` from now.
    pub fn arriving_in(&self, to_thr_s: f64) -> RunwaySlot {
        let shift = to_thr_s - self.to_thr_s;
        RunwaySlot {
            to_thr_s,
            clear_s: self.clear_s + shift,
            unjudged_s: if self.unjudged_s > 0.0 {
                self.unjudged_s + shift
            } else {
                0.0
            },
            ..*self
        }
    }

    /// Both slots are on one strip, either direction.
    pub fn same_strip(&self, other: &RunwaySlot) -> bool {
        [other.thr, other.far]
            .iter()
            .any(|p| dist_m(self.thr, *p) < SAME_STRIP_M)
    }

    /// Seconds from now until it needs the runway clear.
    pub fn needs_runway_in_s(&self) -> f64 {
        self.to_thr_s - self.needs_clear_s
    }
}

/// `lead` reports off the runway [`FOLLOW_GAP_S`] before `follower` needs
/// it, and is seen leaving before the follower is inside the tower's
/// go-around window.
pub fn off_before(lead: &RunwaySlot, follower: &RunwaySlot) -> bool {
    lead.clear_s + FOLLOW_GAP_S <= follower.needs_runway_in_s()
        && (lead.unjudged_s <= 0.0 || lead.unjudged_s + TOWER_DECIDE_S <= follower.to_thr_s)
}

/// Our slot (`own`, as we would publish it) fits round every COMMITTED
/// slot of the others on our runway: each one ahead of us (it reaches the
/// threshold first) is off before we need the runway, and, when we
/// `yield` (we can still wait: a downwind), we are off with a margin
/// before each one behind us needs it. Uncommitted traffic is sequenced
/// by the tower and by its own downwind, never here.
pub fn slot_free(me: u32, own: &RunwaySlot, others: &[Seen], yields: bool) -> bool {
    others
        .iter()
        .filter(|s| s.id != me)
        .filter_map(|s| s.slot.map(|x| (s.id, x)))
        .filter(|(_, x)| (x.committed || (yields && x.waiting)) && own.same_strip(x))
        .all(|(id, x)| {
            let ahead = x.to_thr_s < own.to_thr_s || (x.to_thr_s == own.to_thr_s && id < me);
            if ahead {
                off_before(&x, own)
            } else {
                !yields || off_before(own, &x.arriving_in(x.to_thr_s - AHEAD_MARGIN_S))
            }
        })
}

/// What a landing on one runway end costs this aircraft, worked out once
/// per end and lap (the way off is a plan on the network, driven).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Occupancy {
    pub thr: LatLon,
    /// Worked out for this many laps flown (the last lap stops).
    pub laps_done: u32,
    /// The flare, seconds.
    pub flare_s: f64,
    /// The rollout to the exit pace, seconds.
    pub roll_s: f64,
    /// Then off the strip, and the report: its way off, or a touch and
    /// go's roll (both the same).
    pub off_s: f64,
    pub done_s: f64,
    pub backtrack: bool,
    /// The rollout is to a junction planned at speed
    /// ([`TrafficAircraft::plan_exit`]): the tower reads it as rolling on
    /// down the strip until it turns off there.
    pub turns_off: bool,
}

/// A ground drive's estimate while it is driven: the phase it was made
/// in, when (the aircraft's age), when it is off the strip and when its
/// drive ends, and whether it backtracks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DriveEstimate {
    pub phase: Phase,
    pub made_at_s: f64,
    pub off_at_s: f64,
    pub done_at_s: f64,
    pub backtrack: bool,
}

/// An orbit being flown: the route's vertex it rejoins at and the height
/// it holds round it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Orbit {
    pub until: usize,
    pub alt_ft: f64,
}

/// A drive DRIVEN: seconds until last on the strip, until the drive ends,
/// until "runway vacated" is said (clear of the pavement, tail and all,
/// `TrafficAircraft::clear_of_runway`, else the drive's end), and whether
/// it ended on the strip back toward the threshold.
struct Drive {
    off_s: f64,
    done_s: f64,
    report_s: f64,
    backtrack: bool,
}

impl TrafficAircraft {
    /// The speed a leg is flown at, knots (the phase machine's own).
    fn leg_kts(&self, leg: Leg) -> f64 {
        match leg {
            Leg::Base | Leg::Final | Leg::Runway => self.perf.app_kts,
            _ => self.perf.pattern_kts,
        }
    }

    /// Seconds to the landing threshold along the route, each leg at its
    /// own speed (the one flown now between the speed held and the leg's).
    /// `None` for a route that does not land.
    pub(crate) fn time_to_thr_s(&self) -> Option<f64> {
        let i = self.route.thr_idx?;
        let seg = self.route.follow.seg;
        if seg >= i || i >= self.route.path.len() {
            return Some(0.0);
        }
        let p = &self.route.path;
        let ms = |kts: f64| kts.max(40.0) * KT_TO_MS;
        // An instrument arrival flies the radar unit's schedule by its
        // distance to go; the next mile at the speed it holds now.
        if self.instrument() {
            let d = self.route.dist_to(self.kin.pos(), i);
            let first = d.min(1852.0);
            let sched = crate::arrival::scheduled_kts(self.perf, d);
            let rest = crate::arrival::scheduled_time_s(self.perf, d - first);
            return Some(first / ms(0.5 * (self.kin.gs_kts + sched)) + rest);
        }
        let now = 0.5 * (self.kin.gs_kts + self.leg_kts(self.route.leg()));
        let mut t = dist_m(self.kin.pos(), p[seg + 1]) / ms(now);
        for k in seg + 1..i {
            let leg = self.route.legs.get(k).copied().unwrap_or(Leg::Final);
            t += dist_m(p[k], p[k + 1]) / ms(self.leg_kts(leg));
        }
        Some(t)
    }

    /// Seconds before the threshold its landing clearance is due: the
    /// path from [`NO_CLEARANCE_AGL_FT`] down, at the approach speed.
    fn clearance_lead_s(&self) -> f64 {
        let m = (NO_CLEARANCE_AGL_FT - TCH_FT) / (GLIDE_DEG.to_radians().tan() * FT_PER_M);
        m / (self.perf.app_kts.max(40.0) * KT_TO_MS)
    }

    /// Exit pace off the rollout, knots (the landing's own).
    fn exit_kts(&self) -> f64 {
        self.perf.taxi_kts + 2.0
    }

    /// A touch and go follows this landing.
    pub(crate) fn touch_and_go(&self) -> bool {
        matches!(self.plan, Plan::Circuit { laps, .. } if self.laps_done + 1 < laps)
    }

    /// The flare's sink, feet a minute (the landing's own).
    fn flare_sink_fpm(&self) -> f64 {
        let approach = self.perf.vref_kts * KT_TO_MS * 196.85 * GLIDE_DEG.to_radians().tan();
        FLARE_SINK_FPM.max(0.5 * approach)
    }

    /// The runway time a landing on `end` costs, cached per end and lap.
    fn occupancy(&mut self, field: &FieldModel, end: &RunwayEnd) -> Occupancy {
        if let Some(o) = self
            .occupancy
            .filter(|o| dist_m(o.thr, end.thr) < 1.0 && o.laps_done == self.laps_done)
        {
            return o;
        }
        let p = self.perf;
        let flare_s = FLARE_AGL_FT / self.flare_sink_fpm() * 60.0;
        let touch_kts = p.vref_kts - 5.0;
        let mut roll_s = ((touch_kts - self.exit_kts()) / p.decel_kts_s).max(0.0);
        let flare_at = (TCH_FT - FLARE_AGL_FT) / (GLIDE_DEG.to_radians().tan() * FT_PER_M);
        // The way off planned at speed from where the flare puts the
        // wheels down, as the landing plans it at touchdown
        // ([`TrafficAircraft::plan_exit`]): the rollout to its junction,
        // then its drive.
        let touch_m = flare_at + touch_kts * KT_TO_MS * flare_s;
        let touch = end.at(touch_m.clamp(0.0, (end.length_m - 50.0).max(0.0)), 0.0);
        let fast = if self.touch_and_go() {
            None
        } else {
            self.plan_exit(field, end, touch, end.course_deg, touch_kts, &[])
        };
        let drive = if let Some((path, run)) = &fast {
            let d: f64 = path[..=run.junction]
                .windows(2)
                .map(|w| dist_m(w[0], w[1]))
                .sum();
            roll_s = time_to_exit_s(p, touch_kts, run.kts, d);
            let run = *run;
            Drive {
                off_s: run.off_s,
                done_s: run.done_s,
                report_s: run.done_s,
                backtrack: false,
            }
        } else if self.touch_and_go() {
            let s = ((p.vr_kts - self.exit_kts()) / p.roll_accel_kts_s).max(0.0);
            Drive {
                off_s: s,
                done_s: s,
                report_s: s,
                backtrack: false,
            }
        } else {
            // Where the flare starts on the path, then the flare and the
            // rollout: where the way off begins.
            let stop = flare_at
                + touch_kts * KT_TO_MS * flare_s
                + 0.5 * (touch_kts + self.exit_kts()) * KT_TO_MS * roll_s;
            let at = end.at(stop.clamp(0.0, (end.length_m - 50.0).max(0.0)), 0.0);
            match field.vacate(at, end.course_deg, end, &self.icao_type, &[]) {
                Some(v) => {
                    let mut kin = Kin::parked(at, end.elev_ft, end.course_deg);
                    kin.gs_kts = self.exit_kts();
                    let mut path = vec![at];
                    path.extend(v.path.into_iter().skip_while(|q| dist_m(*q, at) < 1.0));
                    self.runway_drive(field, end, kin, &path, p.taxi_kts, false)
                }
                None => Drive {
                    off_s: 0.0,
                    done_s: 0.0,
                    report_s: 0.0,
                    backtrack: false,
                },
            }
        };
        let o = Occupancy {
            thr: end.thr,
            laps_done: self.laps_done,
            flare_s,
            roll_s,
            off_s: drive.off_s,
            done_s: drive.report_s,
            backtrack: drive.backtrack,
            turns_off: fast.is_some(),
        };
        self.occupancy = Some(o);
        o
    }

    /// The drive along `path` from `kin`, DRIVEN as the phase machine
    /// drives it (the runway's pace on the runway, `off_kts` off it, the
    /// corners slowed for). A sum of lengths over a pace was half of
    /// BIEG's backtrack (the turns, the speed up).
    fn runway_drive(
        &self,
        field: &FieldModel,
        end: &RunwayEnd,
        mut kin: Kin,
        path: &[LatLon],
        off_kts: f64,
        exit_brake: bool,
    ) -> Drive {
        let mut at = Follow::default();
        let (mut t, mut off_s) = (0.0, 0.0);
        let (mut first, mut last) = (None, None);
        let mut report = None;
        // A way off is over, for the slot, once it is said: a line-up is
        // driven to its end.
        let to_report = !matches!(self.phase, Phase::LineUp | Phase::HoldShort);
        while t < DRIVE_MAX_S {
            let on = field.on_runway(kin.pos());
            if report.is_none() && !on && self.clear_of_runway_at(field, &kin) {
                report = Some(t);
                if to_report {
                    break;
                }
            }
            if on {
                off_s = t + DRIVE_STEP_S;
                first.get_or_insert(kin.pos());
                last = Some(kin.pos());
            }
            let kts = if on { self.backtrack_kts() } else { off_kts };
            let mut spec = FollowSpec::taxi(kts, self.perf.turn_taxi_dps);
            if exit_brake {
                // Braked off the fast exit as the vacate brakes
                // (`TrafficAircraft::ground_spec`).
                spec.decel_kts_s = self.exit_spec_decel(kin.gs_kts, kts, spec.decel_kts_s, false);
            }
            t += DRIVE_STEP_S;
            if follow(path, &mut at, &mut kin, &spec, DRIVE_STEP_S) {
                break;
            }
        }
        let backtrack = match (first, last) {
            (Some(a), Some(b)) => end.frame(b).0 < end.frame(a).0 - BACKTRACK_M,
            _ => false,
        };
        Drive {
            off_s,
            done_s: t,
            report_s: report.unwrap_or(t),
            backtrack,
        }
    }

    /// The way off from its junction (`path`, entered at `kts`), DRIVEN as
    /// the vacate drives it: seconds until off the strip, and until it says
    /// "runway vacated".
    pub(crate) fn exit_drive_s(
        &self,
        field: &FieldModel,
        end: &RunwayEnd,
        path: &[LatLon],
        kts: f64,
    ) -> (f64, f64) {
        let Some(&j) = path.first() else {
            return (0.0, 0.0);
        };
        let hdg = path
            .get(1)
            .map(|q| bearing(j, *q))
            .unwrap_or(end.course_deg);
        let mut kin = Kin::parked(j, end.elev_ft, hdg);
        kin.gs_kts = kts;
        let d = self.runway_drive(field, end, kin, path, self.perf.taxi_kts, true);
        (d.off_s, d.report_s)
    }

    /// The ground drive now driven (a vacate, a line-up), driven again
    /// every [`DRIVE_REFRESH_S`]: seconds until off the strip, until the
    /// drive ends, and whether it backtracks.
    fn drive_now(&mut self, field: &FieldModel, end: &RunwayEnd, off_kts: f64) -> (f64, f64, bool) {
        let fresh = self
            .drive_est
            .filter(|d| d.phase == self.phase && self.age_s - d.made_at_s < DRIVE_REFRESH_S);
        let d = match fresh {
            Some(d) => d,
            None => {
                let seg = self
                    .route
                    .follow
                    .seg
                    .min(self.route.path.len().saturating_sub(1));
                let mut rest = vec![self.kin.pos()];
                rest.extend(self.route.path.iter().skip(seg + 1).copied());
                let drive = self.runway_drive(field, end, self.kin.clone(), &rest, off_kts, true);
                let d = DriveEstimate {
                    phase: self.phase,
                    made_at_s: self.age_s,
                    off_at_s: self.age_s + drive.off_s,
                    // A vacate's end is its report: said as it clears the
                    // pavement.
                    done_at_s: self.age_s
                        + if self.phase == Phase::Vacate {
                            drive.report_s
                        } else {
                            drive.done_s
                        },
                    backtrack: drive.backtrack,
                };
                self.drive_est = Some(d);
                d
            }
        };
        let left = |at: f64| (at - self.age_s).max(0.0);
        (left(d.off_at_s), left(d.done_at_s), d.backtrack)
    }

    /// The slot this aircraft publishes now, refreshed once a substep:
    /// `None` with no landing ahead and nothing on the runway.
    pub(crate) fn refresh_slot(&mut self, field: &FieldModel, end: Option<&RunwayEnd>) {
        self.slot = end.and_then(|end| self.slot_now(field, end));
    }

    fn slot_now(&mut self, field: &FieldModel, end: &RunwayEnd) -> Option<RunwaySlot> {
        let straight_in = matches!(
            self.plan,
            Plan::Arrival {
                straight_in: true,
                ..
            }
        );
        let lead_s = self.clearance_lead_s();
        let wake = self.wake;
        let rejoining = self.missed && self.route.leg() == Leg::Inbound;
        // On the runway: seconds to off the strip and to the report; a
        // backtrack is seen leaving only once it is off the strip.
        let on_runway = |off_s: f64, done_s: f64, backtrack: bool| RunwaySlot {
            thr: end.thr,
            far: end.far,
            to_thr_s: 0.0,
            clear_s: done_s,
            unjudged_s: if backtrack { off_s } else { 0.0 },
            needs_clear_s: 0.0,
            committed: true,
            waiting: false,
            wake,
            rejoining,
        };
        match self.phase {
            Phase::Arrive | Phase::Pattern { .. } | Phase::Final => {
                if matches!(
                    self.phase,
                    Phase::Pattern {
                        leg: Leg::Upwind | Leg::Crosswind
                    }
                ) {
                    return None;
                }
                let to = self.time_to_thr_s()?;
                let o = self.occupancy(field, end);
                let committed = match self.phase {
                    Phase::Arrive => straight_in || self.instrument(),
                    Phase::Pattern { leg } => leg == Leg::Base,
                    _ => true,
                };
                let down = to + o.flare_s + o.roll_s;
                // A rollout at speed to its turn-off is, to the tower's
                // track, a lander still rolling down the strip until it
                // turns: seen leaving only there. EHAM 09 (the peak hour,
                // 2026-10-10): a jet rolling at its exit speed to a turn-off
                // 1.9 km in was off 8 s after the tower sent the next one
                // round from 4 km out.
                let unjudged = if o.backtrack {
                    down + o.off_s
                } else if o.turns_off {
                    down
                } else {
                    0.0
                };
                Some(RunwaySlot {
                    to_thr_s: to,
                    clear_s: down + o.done_s,
                    unjudged_s: unjudged,
                    needs_clear_s: lead_s.min(to),
                    committed,
                    ..on_runway(0.0, 0.0, false)
                })
            }
            Phase::Landing => {
                let o = self.occupancy(field, end);
                // Its way off planned at touchdown: to the junction, then
                // the drive from there.
                if let Some(run) = self.exit.filter(|_| self.kin.on_ground) {
                    let d = self.route.dist_to(self.kin.pos(), run.junction);
                    let rest = time_to_exit_s(self.perf, self.kin.gs_kts, run.kts, d);
                    return Some(RunwaySlot {
                        unjudged_s: rest,
                        ..on_runway(rest + run.off_s, rest + run.done_s, false)
                    });
                }
                let rest = if self.kin.on_ground {
                    ((self.kin.gs_kts - self.exit_kts()) / self.perf.decel_kts_s).max(0.0)
                } else {
                    let agl = (self.kin.alt_ft - end.elev_ft).max(0.0);
                    agl / self.flare_sink_fpm() * 60.0 + o.roll_s
                };
                let slot = on_runway(rest + o.off_s, rest + o.done_s, o.backtrack);
                // In the flare, a rollout to a turn-off at speed ahead.
                Some(if o.turns_off && !self.kin.on_ground {
                    RunwaySlot {
                        unjudged_s: slot.unjudged_s.max(rest),
                        ..slot
                    }
                } else {
                    slot
                })
            }
            // Off the runway once it has said so.
            Phase::Vacate if self.vacated_at_s.is_none() => {
                let (off, done, backtrack) = self.drive_now(field, end, self.perf.taxi_kts);
                Some(on_runway(off, done, backtrack))
            }
            // Cleared onto the runway: the line-up, then the roll. The
            // tower sees it leaving only once it rolls.
            Phase::LineUp => {
                let off = LINE_UP_KTS.min(self.perf.taxi_kts);
                let (_, done, _) = self.drive_now(field, end, off);
                let roll = self.perf.vr_kts / self.perf.roll_accel_kts_s;
                Some(RunwaySlot {
                    unjudged_s: done,
                    ..on_runway(0.0, done + roll, false)
                })
            }
            // At the hold: the tower has yet to release it, so its line-up
            // and roll stand in front of the circuit, never counting down.
            // USHQ 16 (the hour sweep, 2026-10-09): with three circuits
            // spaced on their finals and none extending for it, a departure
            // waited at the hold the whole hour and five queued behind it.
            Phase::HoldShort => {
                let path = self.line_up_path(end);
                let mut kin = self.kin.clone();
                kin.gs_kts = 0.0;
                let off = LINE_UP_KTS.min(self.perf.taxi_kts);
                let line_up = match self.drive_est.filter(|d| d.phase == Phase::HoldShort) {
                    Some(d) => d.done_at_s - d.made_at_s,
                    None => {
                        let done = self.runway_drive(field, end, kin, &path, off, false).done_s;
                        self.drive_est = Some(DriveEstimate {
                            phase: Phase::HoldShort,
                            made_at_s: self.age_s,
                            off_at_s: self.age_s + done,
                            done_at_s: self.age_s + done,
                            backtrack: false,
                        });
                        done
                    }
                };
                let roll = self.perf.vr_kts / self.perf.roll_accel_kts_s;
                Some(RunwaySlot {
                    unjudged_s: line_up,
                    committed: false,
                    waiting: true,
                    ..on_runway(0.0, line_up + roll, false)
                })
            }
            Phase::Roll => {
                let p = self.perf;
                let s = ((p.vr_kts - self.kin.gs_kts) / p.roll_accel_kts_s).max(0.0);
                Some(on_runway(s, s, false))
            }
            _ => None,
        }
    }

    /// A straight-in that will not fit behind the traffic committed ahead
    /// of it is given its delay EARLY: one orbit where it is, on the side
    /// away from the circuit, while it is still [`ORBIT_MIN_M`] out. The
    /// orbit is flown level and the approach goes on from where it began.
    pub(crate) fn delay_if_no_slot(&mut self, end: &RunwayEnd, others: &[Seen]) {
        let straight_in = matches!(
            self.plan,
            Plan::Arrival {
                straight_in: true,
                ..
            }
        );
        if !straight_in
            || !matches!(self.phase, Phase::Arrive | Phase::Final)
            || !matches!(self.route.leg(), Leg::Inbound | Leg::Final)
            || self.orbit.is_some()
            || self.orbits >= MAX_ORBITS
        {
            return;
        }
        let Some(thr_idx) = self.route.thr_idx else {
            return;
        };
        if self.route.dist_to(self.kin.pos(), thr_idx) < ORBIT_MIN_M {
            return;
        }
        let Some(own) = self.slot else { return };
        if slot_free(self.id, &own, others, false) {
            return;
        }
        self.start_orbit(end);
    }

    /// One orbit inserted into the route from here, turning away from the
    /// runway's circuit side (a straight-in away from the circuit, a
    /// downwind away from the runway).
    pub(crate) fn start_orbit(&mut self, end: &RunwayEnd) {
        let here = self.kin.pos();
        let v = self.kin.gs_kts.max(40.0) * KT_TO_MS;
        let r = ORBIT_RADIUS_FACTOR * v / self.perf.turn_air_dps.to_radians();
        // A left-hand circuit orbits right: off the final, off the runway.
        let s = -end.side();
        let h = self.kin.hdg_deg;
        let centre = project_m(here, h + 90.0 * s, r);
        let start = bearing(centre, here);
        let pts: Vec<LatLon> = (1..=ORBIT_POINTS)
            .map(|k| {
                if k == ORBIT_POINTS {
                    here
                } else {
                    project_m(
                        centre,
                        start + s * 360.0 * k as f64 / ORBIT_POINTS as f64,
                        r,
                    )
                }
            })
            .collect();
        let seg = self.route.follow.seg;
        let leg = self.route.leg();
        let at = seg + 1;
        let n = pts.len();
        self.route.path.splice(at..at, pts);
        let legs_at = at.min(self.route.legs.len());
        self.route.legs.splice(legs_at..legs_at, vec![leg; n]);
        if let Some(i) = self.route.thr_idx.as_mut() {
            *i += n;
        }
        self.orbit = Some(Orbit {
            until: seg + n,
            alt_ft: self.kin.alt_ft,
        });
        self.orbits += 1;
    }

    /// The height an orbit holds, while one is flown; it ends at its
    /// rejoining vertex.
    pub(crate) fn orbit_alt(&mut self) -> Option<f64> {
        let o = self.orbit?;
        if self.route.follow.seg >= o.until {
            self.orbit = None;
            return None;
        }
        Some(o.alt_ft)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const THR: LatLon = (60.0, 10.0);
    const FAR: LatLon = (60.02, 10.0);

    /// A slot `to_thr_s` out, reporting off `clear_s` from now.
    fn slot(to_thr_s: f64, clear_s: f64) -> RunwaySlot {
        RunwaySlot {
            thr: THR,
            far: FAR,
            to_thr_s,
            clear_s,
            unjudged_s: 0.0,
            needs_clear_s: 0.0,
            committed: true,
            waiting: false,
            wake: Wake::Medium,
            rejoining: false,
        }
    }

    fn seen(id: u32, slot: RunwaySlot) -> Seen {
        Seen {
            id,
            phase: Phase::Final,
            pos: THR,
            gs_kts: 140.0,
            slot: Some(slot),
        }
    }

    /// A circuit 172 at the end of its downwind, 120 s from the threshold
    /// and off the runway 40 s after: a straight-in on a long final that
    /// gets there first and off again inside the gap holds its base (BIEG
    /// 03, 2026-10-09: the base turned in front of it and one of the two
    /// went around); once the straight-in will be off in time, or is far
    /// enough out for the 172 to land, roll and leave first, it turns.
    #[test]
    fn a_circuit_holds_its_base_for_a_straight_in_that_reaches_the_runway_first() {
        let own = RunwaySlot {
            committed: false,
            ..slot(120.0, 160.0)
        };
        let straight_in = |to: f64, clear: f64| [seen(2, slot(to, clear))];
        assert!(
            !slot_free(1, &own, &straight_in(90.0, 110.0), true),
            "off 10 s before us"
        );
        assert!(
            slot_free(1, &own, &straight_in(60.0, 100.0), true),
            "off 20 s before us"
        );
        // Behind us: we are off at 160, with the gap and the margin.
        assert!(
            !slot_free(1, &own, &straight_in(180.0, 220.0), true),
            "it would catch us"
        );
        assert!(
            slot_free(1, &own, &straight_in(200.0, 240.0), true),
            "room ahead of it"
        );
        // Another circuit on its downwind is the tower's to sequence.
        let downwind = [seen(
            2,
            RunwaySlot {
                committed: false,
                ..slot(100.0, 140.0)
            },
        )];
        assert!(slot_free(1, &own, &downwind, true));
        // Another runway is no conflict.
        let elsewhere = RunwaySlot {
            thr: (61.0, 11.0),
            far: (61.02, 11.0),
            ..slot(90.0, 110.0)
        };
        assert!(slot_free(1, &own, &[seen(2, elsewhere)], true));
    }

    /// A straight-in cannot wait for traffic behind it: only what reaches
    /// the threshold first counts for it, and the circuit makes the room.
    #[test]
    fn a_straight_in_never_waits_for_traffic_behind_it() {
        let own = slot(120.0, 160.0);
        let behind = [seen(2, slot(150.0, 190.0))];
        assert!(slot_free(1, &own, &behind, false));
        assert!(!slot_free(1, &own, &behind, true), "a yielder would");
        // Ahead and still on the runway when we need it: no slot.
        assert!(!slot_free(1, &own, &[seen(2, slot(60.0, 110.0))], false));
    }

    /// The runway is needed by the landing clearance, not the threshold:
    /// a 172's is due 44 s out (300 ft on the path), so the one ahead must
    /// have reported off with the gap before that.
    #[test]
    fn the_runway_is_needed_by_the_landing_clearance_not_the_threshold() {
        let lead = slot(30.0, 90.0);
        let follower = |needs: f64| RunwaySlot {
            needs_clear_s: needs,
            ..slot(140.0, 180.0)
        };
        assert!(off_before(&lead, &follower(20.0)));
        assert!(!off_before(&lead, &follower(44.0)));
    }

    /// An occupant backtracking toward the threshold is never judged clear
    /// by the tower until it is off: the follower must still be outside
    /// the tower's go-around window then, or the tower sends it round
    /// (BIEG 03: a Cessna off the strip one second after the order).
    #[test]
    fn a_backtrack_holds_the_follower_out_of_the_towers_go_around_window() {
        let lead = RunwaySlot {
            unjudged_s: 100.0,
            ..slot(0.0, 110.0)
        };
        assert!(off_before(&slot(0.0, 0.0), &slot(25.0, 60.0)));
        assert!(!off_before(&lead, &slot(170.0, 210.0)), "inside the window");
        assert!(off_before(&lead, &slot(180.0, 220.0)));
        // A rollout to a turn-off ahead is seen leaving at once.
        assert!(off_before(&slot(0.0, 110.0), &slot(130.0, 170.0)));
    }

    /// A departure at the hold waiting for its release holds the circuit
    /// back (it extends, the tower finds the gap) but never a straight-in
    /// (the tower threads the departure in ahead of it).
    #[test]
    fn a_departure_at_the_hold_holds_the_circuit_but_never_the_straight_in() {
        let waiting = [seen(
            2,
            RunwaySlot {
                unjudged_s: 50.0,
                committed: false,
                waiting: true,
                ..slot(0.0, 90.0)
            },
        )];
        let circuit = RunwaySlot {
            committed: false,
            ..slot(120.0, 160.0)
        };
        assert!(!slot_free(1, &circuit, &waiting, true));
        assert!(slot_free(1, &slot(120.0, 160.0), &waiting, false));
        // Far enough out, the circuit fits behind its line-up and roll.
        assert!(slot_free(1, &slot(150.0, 190.0), &waiting, true));
    }

    /// The same slot moved along in time keeps its runway time and its
    /// unseen part.
    #[test]
    fn a_slot_moved_along_keeps_its_runway_time() {
        let s = RunwaySlot {
            unjudged_s: 130.0,
            ..slot(100.0, 150.0)
        };
        let later = s.arriving_in(160.0);
        assert_eq!((later.clear_s, later.unjudged_s), (210.0, 190.0));
        assert_eq!(slot(100.0, 150.0).arriving_in(40.0).unjudged_s, 0.0);
    }
}
