//! An airliner's arrival, flown and controlled as an instrument one (Doc
//! 4444 ch. 8 and 12, CAP 413 ch. 6): it appears out of sight on the far
//! side of the approach, descends on a 3 degree profile with the speeds a
//! radar unit hands out (250 kt below FL100, then 210, 180 and 160 to
//! 4 nm), joins a long final 11 nm out, works the approach unit and then
//! the tower, and keeps its wake spacing on final by speed. A go-around
//! flies the missed approach and is vectored round for another approach.
//!
//! The realism review (2026-10-10): every airliner spawned 10 nm out,
//! joined a 6 nm final at 210 kt, called the TOWER "ten miles west,
//! inbound for landing" and was told "join final"; off a go-around it was
//! sent round a VFR downwind. Light aircraft joining VFR keep all that.

use crate::aircraft::{IfrRadio, Seen, TrafficAircraft};
use crate::field::{FieldModel, RunwayEnd};
use crate::kin::{bearing, dist_m, project_m, LatLon, KT_TO_MS};
use crate::perf::{Perf, Wake, PISTON_SINGLE};
use crate::phase::{Call, Event, Leg, Phase, Plan};
use crate::slot::{FOLLOW_GAP_S, MAX_ORBITS, ORBIT_MIN_M, TOWER_DECIDE_S};

const NM: f64 = 1852.0;

/// An instrument arrival appears this far out along its entry bearing,
/// nm, a draw between the two: beyond the 20 nm the tower's eye and our
/// own traffic display see, as a real one comes out of the en-route
/// sector.
pub const ENTRY_NM: (f64, f64) = (25.0, 40.0);

/// An entry nearer than this is pushed out to [`ENTRY_NM`] (the schedule's
/// VFR-sized 10 nm entries).
pub const ENTRY_MIN_NM: f64 = 20.0;

/// The final is joined this far out, nm: the ILS intercepted at about
/// 3,000 ft on the 3 degree path, 10 to 12 nm out at a busy field.
pub const JOIN_NM: f64 = 11.0;

/// The intercept leg meets the final at this angle, degrees (a vector to
/// the localiser is 30 degrees or less, Doc 4444 8.9.4.1).
pub const INTERCEPT_DEG: f64 = 30.0;

/// The intercept begins this far before the join along the final, nm.
const INTERCEPT_RUN_NM: f64 = 3.0;

/// The highest an instrument arrival appears, feet above the field.
pub const ENTRY_MAX_AGL_FT: f64 = 11_000.0;

/// With an approach unit, the flight is handed to the tower and calls it
/// established this far out, nm; with none it calls the tower at
/// [`TOWER_CALL_NM`] (CAP 413 6.4: "established ILS 26L" or "8 miles").
pub const TOWER_CALL_APPROACH_NM: f64 = 9.0;
pub const TOWER_CALL_NM: f64 = 8.0;

/// The speed schedule's bands, nm from the threshold and knots: 250 kt
/// below FL100 out to 20 nm, 210 to 15, 180 to the join, 160 to 4 nm
/// (the UK's "160 knots to 4 DME"), then the approach speed.
pub const SPEED_BANDS: [(f64, f64); 4] =
    [(20.0, 250.0), (15.0, 210.0), (JOIN_NM, 180.0), (4.0, 160.0)];

/// The least a follower is slowed to for its spacing beyond 4 nm, over
/// its approach speed, knots: a radar unit's "reduce to minimum approach
/// speed" is about this.
pub const MIN_SPACING_OVER_APP_KTS: f64 = 10.0;

/// Kept on top of the wake spacing, nm: a radar controller's buffer, so
/// the minimum is never infringed as the leader slows to its threshold
/// speed (a follower spaced to exactly 3 nm was 2.95 nm out as the leader
/// flared, the test below).
pub const WAKE_BUFFER_NM: f64 = 0.3;

/// How hard a follower slows for its spacing: knots per second of gap
/// short.
/// At 1 a follower 36 s short flew 214 kt behind a leader at 210 and
/// was still 10 s short at the threshold (the spacing test below).
const SPACING_GAIN_KTS_PER_S: f64 = 3.0;

/// The follower aims this many seconds behind its place: its leader's
/// own slowing and the runway's time move the place back as they come.
const SPACING_MARGIN_S: f64 = 10.0;

/// An airliner arriving: any arrival but a light piston single, which
/// joins VFR as before.
pub fn is_instrument(perf: &Perf) -> bool {
    perf.name != PISTON_SINGLE.name
}

/// Where an instrument arrival appears: `entry` itself when it is already
/// beyond [`ENTRY_MIN_NM`], else on the same bearing from the threshold,
/// `draw` (0 to 1) of the way through [`ENTRY_NM`].
pub fn entry_point(end: &RunwayEnd, entry: LatLon, draw: f64) -> LatLon {
    if dist_m(end.thr, entry) >= ENTRY_MIN_NM * NM {
        return entry;
    }
    let nm = ENTRY_NM.0 + draw.clamp(0.0, 1.0) * (ENTRY_NM.1 - ENTRY_NM.0);
    project_m(end.thr, bearing(end.thr, entry), nm * NM)
}

/// The route from `entry` to the threshold: an intercept leg onto the
/// final at no more than [`INTERCEPT_DEG`] when the entry is off the
/// centreline, then the final from [`JOIN_NM`]. Points only; the last is
/// the join fix (the threshold and far end are the caller's).
pub fn approach_points(end: &RunwayEnd, entry: LatLon) -> Vec<LatLon> {
    let join = end.at(-JOIN_NM * NM, 0.0);
    let (along, right) = end.frame(entry);
    let run = INTERCEPT_RUN_NM * NM;
    let lateral = run * INTERCEPT_DEG.to_radians().tan();
    // Already behind the join and near enough the line: straight to it.
    if along <= -(JOIN_NM * NM + run) && right.abs() <= lateral {
        return vec![entry, join];
    }
    let side = if right >= 0.0 { 1.0 } else { -1.0 };
    let intercept = end.at(-(JOIN_NM * NM + run), side * lateral);
    vec![entry, intercept, join]
}

/// The scheduled speed `d_m` metres from the threshold along the route,
/// knots, for `p` ([`SPEED_BANDS`], never above the type's own cruise
/// nor below its approach speed; a turboprop flies its 250 band at its
/// pattern speed plus 40).
pub fn scheduled_kts(p: &Perf, d_m: f64) -> f64 {
    let d = d_m / NM;
    let top = 250.0_f64.min(p.cruise_kts).min(p.pattern_kts + 40.0);
    let kts = SPEED_BANDS
        .iter()
        .find(|(from, _)| d > *from)
        .map_or(p.app_kts, |(_, k)| *k);
    kts.min(top).max(p.app_kts)
}

/// Seconds to fly the last `d_m` metres to the threshold on the schedule.
pub fn scheduled_time_s(p: &Perf, d_m: f64) -> f64 {
    let mut edges: Vec<f64> = SPEED_BANDS.iter().map(|(nm, _)| nm * NM).collect();
    edges.push(0.0);
    let mut t = 0.0;
    let mut hi = d_m.max(0.0);
    for lo in edges {
        if hi > lo {
            let mid = 0.5 * (hi + lo);
            t += (hi - lo) / (scheduled_kts(p, mid) * KT_TO_MS);
            hi = lo;
        }
    }
    t
}

/// The wake turbulence spacing behind `leader` for `follower` on final,
/// nm (Doc 4444 8.7.3.4, distance-based: 3 nm minimum radar separation,
/// 4 behind a heavy for a heavy, 5 for a medium, 6 for a light; behind
/// a super 6, 7 and 8; 5 for a light behind a medium).
pub fn wake_gap_nm(leader: Wake, follower: Wake) -> f64 {
    use Wake::*;
    match (leader, follower) {
        (Super, Super) => 3.0,
        (Super, Heavy) => 6.0,
        (Super, Medium) => 7.0,
        (Super, Light) => 8.0,
        (Heavy, Heavy) => 4.0,
        (Heavy, Medium) => 5.0,
        (Heavy, Light) => 6.0,
        (Medium, Light) => 5.0,
        _ => 3.0,
    }
}

/// The speed a follower flies for its spacing: the schedule's, slowed by
/// [`SPACING_GAIN_KTS_PER_S`] for each second it is short of its place
/// (`short_s`), never below its approach speed plus
/// [`MIN_SPACING_OVER_APP_KTS`] (inside 4 nm the schedule is already the
/// approach speed and stays).
pub fn spacing_kts(p: &Perf, scheduled: f64, short_s: f64) -> f64 {
    if short_s <= 0.0 || scheduled <= p.app_kts {
        return scheduled;
    }
    let floor = (p.app_kts + MIN_SPACING_OVER_APP_KTS).min(scheduled);
    (scheduled - SPACING_GAIN_KTS_PER_S * short_s).max(floor)
}

/// The no-procedure missed approach altitude, feet on the QNH: 2,000 ft
/// above the field rounded UP to the thousand, never below 3,000 (the
/// tower's own `missed_approach_alt_ft`, so the flight climbs to what it
/// was told).
pub fn missed_approach_alt_ft(field_elev_ft: f64) -> f64 {
    (((field_elev_ft.max(0.0) + 2000.0) / 1000.0).ceil() * 1000.0).max(3000.0)
}

/// The missed approach and the vectors back round: on runway heading to
/// 3 nm past the far end, a turn to a downwind 5 nm off on the runway's
/// pattern side, back to 15 nm out on the approach side, then the same
/// intercept onto the final as an arrival ([`approach_points`]). About
/// 45 nm, 12 to 15 minutes: the "about 10 to 15 minutes later" of a real
/// re-sequenced go-around. Points from the upwind end, the last the join
/// fix; how many of them are the upwind is 1.
pub fn missed_points(end: &RunwayEnd) -> Vec<LatLon> {
    let side = end.side();
    let out = end.length_m + 3.0 * NM;
    let wide = side * 5.0 * NM;
    let downwind_end = end.at(-15.0 * NM, wide);
    let mut pts = vec![end.at(out, 0.0), end.at(out, wide)];
    pts.extend(approach_points(end, downwind_end));
    pts
}

/// An altitude as a pilot says it on a check-in: "seven thousand", "one
/// one thousand" (the thousands, rounded down; ICAO digits past ten).
pub fn alt_words(ft: f64) -> String {
    let k = (ft.max(0.0) / 1000.0).floor() as u32;
    let lead = if k >= 10 {
        k.to_string()
            .chars()
            .filter_map(|c| c.to_digit(10))
            .map(|d| crate::phase::digit_word(d))
            .collect::<Vec<_>>()
            .join(" ")
    } else {
        crate::phase::number_words(k)
    };
    format!("{lead} thousand")
}

/// A flight coming back round off a missed approach lets everyone who
/// would reach the threshold up to this long after it go first, seconds:
/// LFPG (the hour sweep, 2026-10-10), one turned in off its downwind 1.3 nm
/// ahead of an arrival already established and sent it round.
pub const REJOIN_YIELD_S: f64 = 180.0;

/// An orbit is flown only when the follower is short of its place by
/// this much more than slowing to its floor can still make up, seconds.
const ORBIT_MARGIN_S: f64 = 15.0;

impl TrafficAircraft {
    /// An instrument arrival: an airliner arriving ([`is_instrument`]).
    pub fn instrument(&self) -> bool {
        matches!(self.plan, Plan::Arrival { .. }) && is_instrument(self.perf)
    }

    /// The speed flown `d_m` along the route from the threshold: the
    /// schedule's, slowed for the spacing behind the one ahead.
    pub(crate) fn instrument_kts(&self, d_m: f64, others: &[Seen]) -> f64 {
        spacing_kts(
            self.perf,
            scheduled_kts(self.perf, d_m),
            self.spacing_short_s(others) + SPACING_MARGIN_S,
        )
    }

    /// Seconds this arrival is short of its place behind everyone committed
    /// ahead of it on its runway (negative: room to spare). Its place is the
    /// latest of: the runway's (the one ahead off it [`FOLLOW_GAP_S`] before
    /// our landing clearance is due, and seen leaving outside the tower's
    /// go-around window, [`crate::slot::off_before`]), and, behind an
    /// arrival still on its approach, the wake spacing at the threshold
    /// ([`wake_gap_nm`]) at our own final speeds.
    pub(crate) fn spacing_short_s(&self, others: &[Seen]) -> f64 {
        let Some(own) = self.slot else {
            return 0.0;
        };
        let mut need = f64::NEG_INFINITY;
        for s in others.iter().filter(|s| s.id != self.id) {
            let Some(x) = s.slot else { continue };
            if !x.committed || !own.same_strip(&x) {
                continue;
            }
            // Off a missed approach it goes to the far end of the sequence:
            // whoever reaches the threshold up to [`REJOIN_YIELD_S`] after
            // it goes first, and slows for it no more than it would for a
            // flight not yet in the sequence.
            let ahead = match (own.rejoining, x.rejoining) {
                (true, false) => x.to_thr_s < own.to_thr_s + REJOIN_YIELD_S,
                (false, true) => x.to_thr_s + REJOIN_YIELD_S < own.to_thr_s,
                _ => x.to_thr_s < own.to_thr_s || (x.to_thr_s == own.to_thr_s && s.id < self.id),
            };
            if !ahead {
                continue;
            }
            let mut req = x.clear_s + FOLLOW_GAP_S + own.needs_clear_s;
            if x.unjudged_s > 0.0 {
                req = req.max(x.unjudged_s + TOWER_DECIDE_S);
            }
            let approaching = x.to_thr_s > 0.0
                && matches!(
                    s.phase,
                    Phase::Arrive | Phase::Final | Phase::Pattern { .. }
                );
            if approaching {
                let gap_m = (wake_gap_nm(x.wake, self.wake) + WAKE_BUFFER_NM) * NM;
                req = req.max(x.to_thr_s + scheduled_time_s(self.perf, gap_m));
            }
            need = need.max(req);
        }
        if need.is_finite() {
            need - own.to_thr_s
        } else {
            f64::NEG_INFINITY
        }
    }

    /// Seconds slowing to the floor ([`spacing_kts`]) can still add over the
    /// rest of the approach, `d_m` from the threshold.
    fn slowing_can_add_s(&self, d_m: f64) -> f64 {
        let p = self.perf;
        let inner = d_m.min(4.0 * NM);
        let outer = (d_m - inner).max(0.0);
        let slow = outer / ((p.app_kts + MIN_SPACING_OVER_APP_KTS) * KT_TO_MS)
            + inner / (p.app_kts * KT_TO_MS);
        (slow - scheduled_time_s(p, d_m)).max(0.0)
    }

    /// The delay a radar unit gives when speed cannot make the gap: an
    /// orbit, only while still [`ORBIT_MIN_M`] out, and only once slowing
    /// to the floor could not make up what it is short.
    pub(crate) fn instrument_delay(&mut self, end: &RunwayEnd, others: &[Seen]) {
        if !matches!(self.phase, Phase::Arrive | Phase::Final)
            || !matches!(self.route.leg(), Leg::Inbound | Leg::Final)
            || self.orbit.is_some()
            || self.orbits >= MAX_ORBITS
        {
            return;
        }
        let Some(thr_idx) = self.route.thr_idx else {
            return;
        };
        let d = self.route.dist_to(self.kin.pos(), thr_idx);
        if d < ORBIT_MIN_M {
            return;
        }
        let short = self.spacing_short_s(others);
        if short > self.slowing_can_add_s(d) + ORBIT_MARGIN_S {
            self.start_orbit(end);
        }
    }

    /// The calls of an instrument approach, made as the flight gets to
    /// them: with an approach unit, its check-in (or the missed approach's),
    /// then "established" to it, which hands the flight to the tower; then
    /// to the tower, established, [`TOWER_CALL_APPROACH_NM`] out. With
    /// none, the tower is called established at [`TOWER_CALL_NM`].
    pub(crate) fn instrument_calls(
        &mut self,
        field: &FieldModel,
        path_to_thr: Option<f64>,
        events: &mut Vec<Event>,
    ) {
        let unit = field.airport.freq(nav::FREQ_APPROACH).is_some();
        let joined = matches!(self.route.leg(), Leg::Final | Leg::Runway)
            && matches!(self.phase, Phase::Final | Phase::Arrive);
        let nm = path_to_thr.unwrap_or(0.0) / NM;
        let mut say = |me: &mut Self, call: Call| events.push(Event::Call { id: me.id, call });
        match self.ifr_radio {
            IfrRadio::Quiet if !unit => self.ifr_radio = IfrRadio::WithTower,
            IfrRadio::Quiet => {
                let call = match self.phase {
                    Phase::Arrive if !self.missed => Some(Call::ArrivalCheckIn {
                        alt_ft: self.kin.alt_ft.max(0.0) as u32,
                    }),
                    // Off the missed approach, once the tower has had the
                    // "going around" and its answer.
                    Phase::GoAround | Phase::Arrive if self.missed && self.phase_s > 10.0 => {
                        Some(Call::MissedCheckIn {
                            alt_ft: self.entry_alt_ft as u32,
                        })
                    }
                    _ => None,
                };
                if let Some(call) = call {
                    say(self, call);
                    self.ifr_radio = IfrRadio::CheckedIn;
                }
            }
            IfrRadio::CheckedIn if joined => {
                let how = self.approach_kind(field);
                say(
                    self,
                    Call::Established {
                        approach: true,
                        how,
                        miles: 0,
                    },
                );
                self.ifr_radio = IfrRadio::WithTower;
            }
            IfrRadio::WithTower => {
                let at = if unit {
                    TOWER_CALL_APPROACH_NM
                } else {
                    TOWER_CALL_NM
                };
                if joined && nm <= at {
                    let how = self.approach_kind(field);
                    say(
                        self,
                        Call::Established {
                            approach: false,
                            how,
                            miles: nm.round().max(1.0) as u32,
                        },
                    );
                    self.ifr_radio = IfrRadio::TowerCalled;
                }
            }
            _ => {}
        }
    }

    /// The approach this flight flies to its runway, as said: "ILS" where
    /// the field publishes one (an `I` procedure in its CIFP), "RNP
    /// approach" for another published one, empty for none.
    fn approach_kind(&self, field: &FieldModel) -> String {
        let Some(rwy) = self.plan.runway() else {
            return String::new();
        };
        let ils = field.procs.approaches.iter().any(|a| {
            a.starts_with('I')
                && nav::approach_runway(a).is_some_and(|r| nav::same_runway_end(&r, rwy))
        });
        if ils {
            "ILS".into()
        } else if field.procs.approach_for(rwy).is_some() {
            "RNP approach".into()
        } else {
            String::new()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field::fixtures;
    use crate::perf::{JET_HEAVY, JET_MED, TURBOPROP};

    #[test]
    fn an_entry_ten_miles_out_is_pushed_out_on_its_bearing() {
        let end = fixtures::xtst().end("27").unwrap();
        let near = project_m(end.thr, 120.0, 10.0 * NM);
        for draw in [0.0, 0.5, 1.0] {
            let e = entry_point(&end, near, draw);
            let nm = dist_m(end.thr, e) / NM;
            assert!(
                (ENTRY_NM.0 - 0.01..=ENTRY_NM.1 + 0.01).contains(&nm),
                "{nm}"
            );
            assert!((bearing(end.thr, e) - 120.0).abs() < 0.5);
        }
        let far = project_m(end.thr, 100.0, 30.0 * NM);
        assert_eq!(entry_point(&end, far, 0.3), far, "a far entry stands");
    }

    #[test]
    fn the_intercept_meets_the_final_at_thirty_degrees_eleven_miles_out() {
        let end = fixtures::xtst().end("27").unwrap();
        let entry = project_m(end.thr, end.course_deg + 180.0 + 45.0, 30.0 * NM);
        let pts = approach_points(&end, entry);
        assert_eq!(pts.len(), 3);
        let join = pts[2];
        assert!((dist_m(join, end.thr) / NM - JOIN_NM).abs() < 0.01);
        let cut = (bearing(pts[1], join) - end.course_deg + 540.0).rem_euclid(360.0) - 180.0;
        assert!((cut.abs() - INTERCEPT_DEG).abs() < 0.5, "{cut}");
        // On the centreline already: no dogleg.
        let straight = end.at(-30.0 * NM, 0.0);
        assert_eq!(approach_points(&end, straight).len(), 2);
    }

    #[test]
    fn the_speed_schedule_steps_down_to_the_approach_speed() {
        let p = &JET_MED;
        assert_eq!(scheduled_kts(p, 30.0 * NM), 250.0);
        assert_eq!(scheduled_kts(p, 18.0 * NM), 210.0);
        assert_eq!(scheduled_kts(p, 12.0 * NM), 180.0);
        assert_eq!(scheduled_kts(p, 8.0 * NM), 160.0);
        assert_eq!(scheduled_kts(p, 2.0 * NM), p.app_kts);
        // A turboprop never 250: its pattern speed and a margin.
        assert!(scheduled_kts(&TURBOPROP, 30.0 * NM) <= TURBOPROP.pattern_kts + 40.0);
        // The last 4 nm at the approach speed.
        let t = scheduled_time_s(p, 4.0 * NM);
        assert!((t - 4.0 * NM / (p.app_kts * KT_TO_MS)).abs() < 0.5);
    }

    #[test]
    fn wake_spacing_is_wider_behind_a_heavier_leader() {
        assert_eq!(wake_gap_nm(Wake::Medium, Wake::Medium), 3.0);
        assert_eq!(wake_gap_nm(Wake::Heavy, Wake::Medium), 5.0);
        assert_eq!(wake_gap_nm(Wake::Heavy, Wake::Heavy), 4.0);
        assert_eq!(wake_gap_nm(Wake::Super, Wake::Medium), 7.0);
        assert_eq!(wake_gap_nm(Wake::Medium, Wake::Heavy), 3.0);
        let _ = JET_HEAVY;
    }

    #[test]
    fn a_follower_short_of_its_place_slows_but_not_below_its_floor() {
        let p = &JET_MED;
        assert_eq!(spacing_kts(p, 180.0, 0.0), 180.0);
        assert_eq!(spacing_kts(p, 180.0, 5.0), 165.0);
        assert_eq!(
            spacing_kts(p, 180.0, 300.0),
            p.app_kts + MIN_SPACING_OVER_APP_KTS
        );
        assert_eq!(
            spacing_kts(p, p.app_kts, 300.0),
            p.app_kts,
            "inside 4 nm it stays"
        );
    }

    #[test]
    fn the_missed_approach_goes_round_for_about_forty_five_miles() {
        let end = fixtures::xtst().end("27").unwrap();
        let mut pts = vec![end.thr];
        pts.extend(missed_points(&end));
        pts.push(end.thr);
        let nm: f64 = pts.windows(2).map(|w| dist_m(w[0], w[1])).sum::<f64>() / NM;
        assert!((35.0..60.0).contains(&nm), "{nm}");
        assert_eq!(missed_approach_alt_ft(52.0), 3000.0);
        assert_eq!(missed_approach_alt_ft(5600.0), 8000.0);
        assert_eq!(alt_words(9600.0), "nine thousand");
        assert_eq!(alt_words(11_200.0), "one one thousand");
    }
}
