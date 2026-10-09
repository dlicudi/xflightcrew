//! One traffic aircraft and its phase machine. Pure: each substep reads
//! the tower's [`Order`] for this aircraft and the picture of the others,
//! moves the [`Kin`] and reports [`Event`]s. It never waits on speech and
//! never asks anybody anything but its order.

use crate::field::{FieldModel, RunwayEnd};
use crate::kin::{bearing, dist_m, follow, project_m, Follow, FollowSpec, Kin, LatLon, KT_TO_MS};
use crate::perf::{class_for_type, Perf, Wake};
use crate::phase::{Call, Despawn, Event, Leg, Order, Phase, Plan, RadioState, TowerState};
use crate::rng::XorShift64;
use crate::spacing::{GroundSpacing, SPACING_DECEL_MS2};

/// A departure is gone this far from the field.
pub const DEPART_GONE_NM: f64 = 12.0;

/// An aircraft that finished its flight on a stand is gone after this.
pub const PARKED_LINGER_S: f64 = 120.0;

/// No landing clearance by this height on final: go around.
pub const NO_CLEARANCE_AGL_FT: f64 = 300.0;

/// The flare begins below this height.
pub const FLARE_AGL_FT: f64 = 30.0;

/// The least sink held through the flare, feet a minute (half the
/// approach's sink for anything faster than a light aircraft).
pub const FLARE_SINK_FPM: f64 = 200.0;

/// The final approach path, degrees.
pub const GLIDE_DEG: f64 = 3.0;

/// The path crosses the threshold this high.
pub const TCH_FT: f64 = 50.0;

/// How long a leader holds the runway once it reaches the threshold,
/// seconds: a touch and go's roll, or a full stop's rollout to an exit.
const LEADER_RUNWAY_S: f64 = 45.0;

/// The gap number two keeps behind that, seconds.
const FOLLOW_GAP_S: f64 = 20.0;

/// The longest a downwind is extended for a leader.
pub const MAX_DOWNWIND_EXT_NM: f64 = 4.0;

/// Line-up pace, knots.
const LINE_UP_KTS: f64 = 6.0;

/// The line-up ends this far along the centreline from where the lead-on
/// meets it, so the aircraft stops ALIGNED.
const LINE_UP_ALIGN_M: f64 = 60.0;

/// How long a failed taxi plan waits before trying again.
const REPLAN_S: f64 = 10.0;

/// Held short of a crossing, the request is made again this often.
pub const CROSS_ASK_RETRY_S: f64 = 30.0;

/// An owed readback still owed this long after it went out was not taken:
/// it is given again (the link may hold a call a few seconds before it
/// keys, `CALL_GAP_S` in the bin, so this is several of those).
pub const READBACK_RETRY_S: f64 = 12.0;

/// A readback is given this many times before the flight asks "say
/// again", once, and then leaves the frequency (the spec's readback cap).
pub const READBACK_TRIES: u32 = 2;

/// A departure climbs to this above the field before it is gone.
const DEPART_CLIMB_AGL_FT: f64 = 6000.0;

const FT_PER_M: f64 = 3.28084;
const NM: f64 = 1852.0;

/// The path an aircraft is on, tagged leg by leg.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Route {
    pub path: Vec<LatLon>,
    /// `legs[i]` is what `path[i] -> path[i + 1]` is for.
    pub legs: Vec<Leg>,
    pub follow: Follow,
    /// The index in `path` of the landing threshold, if the route lands.
    pub thr_idx: Option<usize>,
}

impl Route {
    pub fn new(path: Vec<LatLon>, legs: Vec<Leg>, thr_idx: Option<usize>) -> Route {
        debug_assert_eq!(legs.len() + 1, path.len().max(1));
        Route {
            path,
            legs,
            follow: Follow::default(),
            thr_idx,
        }
    }

    /// A ground route: every leg taxi.
    pub fn taxi(path: Vec<LatLon>) -> Route {
        let n = path.len().saturating_sub(1);
        Route::new(path, vec![Leg::Taxi; n], None)
    }

    /// The leg being flown.
    pub fn leg(&self) -> Leg {
        self.legs
            .get(self.follow.seg.min(self.legs.len().saturating_sub(1)))
            .copied()
            .unwrap_or(Leg::Taxi)
    }

    /// Metres along the path from `pos` to vertex `idx` (0 once past it).
    pub fn dist_to(&self, pos: LatLon, idx: usize) -> f64 {
        let seg = self.follow.seg;
        if seg >= idx || idx >= self.path.len() {
            return 0.0;
        }
        dist_m(pos, self.path[seg + 1])
            + self.path[seg + 1..=idx]
                .windows(2)
                .map(|w| dist_m(w[0], w[1]))
                .sum::<f64>()
    }
}

/// What one aircraft sees of another.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seen {
    pub id: u32,
    pub phase: Phase,
    pub pos: LatLon,
    /// Groundspeed, knots: number two times its base against it.
    pub gs_kts: f64,
}

/// One traffic aircraft.
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficAircraft {
    /// The strip's `FlightId::Traffic(id)`.
    pub id: u32,
    pub callsign: String,
    /// The callsign as spoken ("Cessna three four five").
    pub spoken: String,
    pub icao_type: String,
    pub wake: Wake,
    pub perf: &'static Perf,
    pub plan: Plan,
    pub kin: Kin,
    pub phase: Phase,
    pub radio: RadioState,
    pub seed: u64,
    pub route: Route,
    /// The stand it started on, or is going to.
    pub stand: Option<String>,
    /// Touch and goes flown so far.
    pub laps_done: u32,
    /// Where it last touched down.
    pub touchdown: Option<LatLon>,
    /// The drawn lead-on from the hold onto the strip.
    lead_on: Vec<LatLon>,
    /// The taxi route past each runway it crosses: the runway the stop at
    /// the end of the piece now driven is short of, and the piece beyond
    /// it, in order ([`FieldModel::cut_at_crossings`]).
    taxi_rest: Vec<(String, Vec<LatLon>)>,
    /// When the crossing held short of was last asked for (phase seconds).
    cross_asked_at: Option<f64>,
    /// Seconds since it spawned (the readback's clock: phases reset theirs).
    age_s: f64,
    /// Seconds in the current phase.
    phase_s: f64,
    /// Metres the downwind has been extended.
    ext_m: f64,
    /// The flight is flown: parked again, lingering.
    flown: bool,
    /// Phase seconds at which the vacate drive ended clear of the runway
    /// and "runway vacated" was called: the wait for a stand runs from
    /// here, never from the end of the rollout (#242).
    vacated_at_s: Option<f64>,
    /// The taxi route being driven is the tower's (its cleared route): the
    /// spacing's guard never plans another, the tower watches it.
    tower_route: bool,
    /// The altitude an arrival came in at.
    entry_alt_ft: f64,
    /// Its distance from the ground traffic ahead, set by the fleet each
    /// substep ([`crate::spacing::apply`]); the drive stops short of it.
    pub spacing: GroundSpacing,
    rng: XorShift64,
}

impl TrafficAircraft {
    /// An aircraft at `kin` in `phase`. Performance and wake from the type.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: u32,
        callsign: &str,
        spoken: &str,
        icao_type: &str,
        plan: Plan,
        kin: Kin,
        phase: Phase,
        seed: u64,
    ) -> TrafficAircraft {
        let (perf, wake) = class_for_type(icao_type);
        TrafficAircraft {
            id,
            callsign: callsign.to_string(),
            spoken: spoken.to_string(),
            icao_type: icao_type.to_string(),
            wake,
            perf,
            plan,
            entry_alt_ft: kin.alt_ft,
            kin,
            phase,
            radio: RadioState::default(),
            seed,
            route: Route::default(),
            stand: None,
            laps_done: 0,
            touchdown: None,
            lead_on: Vec::new(),
            taxi_rest: Vec::new(),
            cross_asked_at: None,
            age_s: 0.0,
            phase_s: 0.0,
            ext_m: 0.0,
            flown: false,
            vacated_at_s: None,
            tower_route: false,
            spacing: GroundSpacing::default(),
            rng: XorShift64::new(seed),
        }
    }

    /// Parked on stand `stand` at `field`. `None` when the field has no
    /// such stand.
    #[allow(clippy::too_many_arguments)]
    pub fn at_stand(
        id: u32,
        callsign: &str,
        spoken: &str,
        icao_type: &str,
        plan: Plan,
        field: &FieldModel,
        stand: &str,
        seed: u64,
    ) -> Option<TrafficAircraft> {
        let s = field.stand(stand)?;
        let kin = Kin::parked((s.lat, s.lon), field.elev_ft(), s.heading);
        let mut ac = TrafficAircraft::new(
            id,
            callsign,
            spoken,
            icao_type,
            plan,
            kin,
            Phase::Parked,
            seed,
        );
        ac.stand = Some(s.name.clone());
        Some(ac)
    }

    /// An arrival at its entry point, flying. Straight in, it is on the
    /// 3 degree path (no higher than 3,000 ft above the field); joining, at
    /// circuit height. `None` for a plan that is not an arrival or a runway
    /// the field does not have.
    pub fn at_entry(
        id: u32,
        callsign: &str,
        spoken: &str,
        icao_type: &str,
        plan: Plan,
        field: &FieldModel,
        seed: u64,
    ) -> Option<TrafficAircraft> {
        let Plan::Arrival {
            entry,
            runway,
            straight_in,
            ..
        } = &plan
        else {
            return None;
        };
        let end = field.end(runway)?;
        let (perf, _) = class_for_type(icao_type);
        let alt = if *straight_in {
            let d = dist_m(*entry, end.thr);
            (end.elev_ft + TCH_FT + glide_ft(d)).min(end.elev_ft + 3000.0)
        } else {
            end.elev_ft + perf.pattern_agl_ft
        };
        let mut kin = Kin::parked(*entry, alt, bearing(*entry, end.thr));
        kin.on_ground = false;
        kin.gear = if perf.retractable_gear { 0.0 } else { 1.0 };
        kin.gs_kts = perf.pattern_kts;
        kin.lights.nav = true;
        kin.lights.beacon = true;
        kin.lights.strobe = true;
        let entry = *entry;
        let mut ac = TrafficAircraft::new(
            id,
            callsign,
            spoken,
            icao_type,
            plan,
            kin,
            Phase::Arrive,
            seed,
        );
        ac.route = if matches!(
            &ac.plan,
            Plan::Arrival {
                straight_in: true,
                ..
            }
        ) {
            let fix = end.at(-straight_in_fix_m(perf), 0.0);
            Route::new(
                vec![entry, fix, end.thr, end.far],
                vec![Leg::Inbound, Leg::Final, Leg::Runway],
                Some(2),
            )
        } else {
            let p = pattern_points(&end, perf);
            Route::new(
                vec![
                    entry,
                    end.at(end.length_m / 2.0, p.offset),
                    p.downwind_end,
                    p.base_end,
                    end.thr,
                    end.far,
                ],
                vec![
                    Leg::Inbound,
                    Leg::Downwind,
                    Leg::Base,
                    Leg::Final,
                    Leg::Runway,
                ],
                Some(4),
            )
        };
        Some(ac)
    }

    /// Parked and going nowhere yet ([`Plan::Parked`]): a parked aircraft,
    /// or a departure waiting on its stand for its time. It has said
    /// nothing, so the tower holds no strip for it.
    pub fn dormant(&self) -> bool {
        matches!(self.plan, Plan::Parked)
    }

    /// A [`dormant`](Self::dormant) aircraft given its flight: it starts up
    /// on its stand and calls for taxi from there. False (nothing changes)
    /// for one not dormant, or a `plan` that is itself [`Plan::Parked`].
    pub fn release(&mut self, plan: Plan) -> bool {
        if !self.dormant() || matches!(plan, Plan::Parked) || self.phase != Phase::Parked {
            return false;
        }
        self.plan = plan;
        self.phase_s = 0.0;
        self.radio.called = false;
        true
    }

    /// Height above the field, feet.
    pub fn agl_ft(&self, field: &FieldModel) -> f64 {
        self.kin.alt_ft - field.elev_ft()
    }

    /// The runway this aircraft uses: the tower's, else the plan's.
    pub fn runway_end(&self, field: &FieldModel, order: &Order) -> Option<RunwayEnd> {
        let ident = order.runway.as_deref().or(self.plan.runway())?;
        let mut end = field.end(ident)?;
        if let Plan::Circuit { left, .. } = self.plan {
            end.left = left;
        }
        Some(end)
    }

    fn set_phase(&mut self, to: Phase, events: &mut Vec<Event>) {
        if to != self.phase {
            events.push(Event::Phase {
                id: self.id,
                from: self.phase,
                to,
            });
            self.phase = to;
            self.radio.called = false;
            self.phase_s = 0.0;
            self.spacing.new_phase();
        }
    }

    fn call_once(&mut self, call: Call, events: &mut Vec<Event>) {
        if !self.radio.called {
            self.radio.called = true;
            events.push(Event::Call { id: self.id, call });
        }
    }

    /// Coast to a stop where it is.
    fn hold_still(&mut self, dt: f64) {
        self.kin.speed_toward(0.0, 1.0, 3.0, dt);
        self.kin.integrate(0.0, dt);
    }

    /// Taxi along the route at `kts`; true once stopped at its end. A
    /// "hold position" stops it where it is. Traffic ahead on the route
    /// slows it to a pace it can stop from short of the spacing
    /// ([`GroundSpacing::free_m`]), and holds it there until the way clears.
    fn drive(&mut self, kts: f64, order: &Order, dt: f64) -> bool {
        if order.hold {
            self.hold_still(dt);
            return false;
        }
        let kts = match self.spacing.free_m {
            Some(free) => kts.min((2.0 * SPACING_DECEL_MS2 * free.max(0.0)).sqrt() / KT_TO_MS),
            None => kts,
        };
        let spec = FollowSpec::taxi(kts, self.perf.turn_taxi_dps);
        follow(
            &self.route.path,
            &mut self.route.follow,
            &mut self.kin,
            &spec,
            dt,
        )
    }

    /// One fixed substep of the phase machine.
    pub fn substep(
        &mut self,
        dt: f64,
        order: Option<&Order>,
        field: &FieldModel,
        others: &[Seen],
        events: &mut Vec<Event>,
    ) {
        let none = Order::default();
        let o = order.unwrap_or(&none);
        self.age_s += dt;
        if self.answer_readback(o, events) {
            return;
        }
        let end = self.runway_end(field, o);
        match self.phase {
            Phase::Parked => self.parked(dt, o, field, end.as_ref(), events),
            Phase::TaxiOut => {
                if self.drive(self.perf.taxi_kts, o, dt) && self.past_crossings(o, field, events) {
                    self.set_phase(Phase::HoldShort, events);
                }
            }
            Phase::HoldShort => {
                self.hold_still(dt);
                if !o.hold {
                    self.call_once(Call::Ready, events);
                }
                if matches!(o.state, TowerState::LinedUp | TowerState::TakeoffCleared) {
                    if let Some(end) = &end {
                        self.route = Route::taxi(self.line_up_path(end));
                        self.set_phase(Phase::LineUp, events);
                    }
                }
            }
            Phase::LineUp => {
                let done = self.drive(LINE_UP_KTS.min(self.perf.taxi_kts), o, dt);
                if done && o.state == TowerState::TakeoffCleared {
                    if let Some(end) = &end {
                        self.route = self.takeoff_route(end, field);
                        self.set_phase(Phase::Roll, events);
                    }
                }
            }
            Phase::Roll => self.roll(dt, events),
            Phase::Climb
            | Phase::Leave
            | Phase::Arrive
            | Phase::Pattern { .. }
            | Phase::Final
            | Phase::GoAround => {
                if let Some(end) = &end {
                    self.fly(dt, o, field, end, others, events);
                }
            }
            Phase::Landing => {
                if let Some(end) = &end {
                    self.landing(dt, o, field, end, events);
                }
            }
            Phase::Vacate => {
                let driven = self.drive(self.perf.taxi_kts, o, dt);
                // Stopped clear of the strip behind traffic in the exit, it
                // HAS vacated: said and on to the stand from here. A vacate
                // waiting to reach its drawn end never reported off, its
                // landing held the runway whole, and the queue ahead (for a
                // crossing of that same runway) was never cleared across:
                // BIEG 03, 2026-10-08, the field gridlocked for two hours.
                let queued = self.spacing.free_m.is_some_and(|f| f < 1.0)
                    && self.kin.gs_kts < 0.5
                    && !field.on_runway(self.kin.pos());
                if driven || queued {
                    self.call_once(Call::Vacated, events);
                    let vacated_at = *self.vacated_at_s.get_or_insert(self.phase_s);
                    if o.state == TowerState::TaxiCleared {
                        self.start_taxi_in(o, field, others, events);
                    }
                    // Nowhere to park (no stands, all taken, no route):
                    // clear of the runway and called vacated, it waits,
                    // then leaves. Timed from the vacate's END: a long
                    // rollout to a far exit (BIEG 03, 1.2 km at taxi pace)
                    // spent the whole wait on the strip and left unheard,
                    // the strip held behind it (#242).
                    if self.phase == Phase::Vacate
                        && self.phase_s - vacated_at >= 2.0 * PARKED_LINGER_S
                    {
                        self.set_phase(Phase::Gone, events);
                        events.push(Event::Despawn {
                            id: self.id,
                            reason: Despawn::NoStand,
                        });
                    }
                }
            }
            Phase::TaxiIn => {
                if self.drive(self.perf.taxi_kts, o, dt) && self.past_crossings(o, field, events) {
                    self.kin.lights = Default::default();
                    self.flown = true;
                    self.set_phase(Phase::Parked, events);
                }
            }
            Phase::Gone => {}
        }
        self.gear_for_phase(field);
        self.phase_s += dt;
    }

    /// The gear, whatever flew this substep: down on the ground and on any
    /// final, landing or go-around below 1,500 ft. The circuit lowered it on
    /// its own legs only, so a straight-in arrival landed, rolled out and
    /// taxied in with it up (LEMG, 2026-10-08, the pilot watching).
    fn gear_for_phase(&mut self, field: &FieldModel) {
        if !self.perf.retractable_gear {
            self.kin.gear = 1.0;
            return;
        }
        let low_approach = matches!(self.phase, Phase::Final | Phase::Landing)
            || (self.phase == Phase::GoAround && self.agl_ft(field) < 1500.0);
        if self.kin.on_ground || low_approach {
            self.kin.gear = 1.0;
        }
    }

    fn parked(
        &mut self,
        dt: f64,
        o: &Order,
        field: &FieldModel,
        end: Option<&RunwayEnd>,
        events: &mut Vec<Event>,
    ) {
        self.hold_still(dt);
        if matches!(self.plan, Plan::Parked) {
            return;
        }
        if self.flown {
            if self.phase_s >= PARKED_LINGER_S {
                self.set_phase(Phase::Gone, events);
                events.push(Event::Despawn {
                    id: self.id,
                    reason: Despawn::Parked,
                });
            }
            return;
        }
        if matches!(self.plan, Plan::Arrival { .. }) {
            return;
        }
        self.kin.lights.nav = true;
        self.kin.lights.beacon = true;
        self.call_once(Call::Taxi, events);
        let cleared = matches!(
            o.state,
            TowerState::TaxiCleared | TowerState::LinedUp | TowerState::TakeoffCleared
        );
        if !cleared || o.hold {
            return;
        }
        // A failed plan is tried again a little later, never every substep.
        if self.phase_s % REPLAN_S > dt * 0.5 && self.phase_s > dt {
            return;
        }
        let Some(end) = end else { return };
        let here = self.kin.pos();
        let (path, lead_on) = match &o.taxi_path {
            Some(p) => {
                self.tower_route = true;
                (p.clone(), Vec::new())
            }
            None => match field.taxi_to_runway(here, end) {
                Some(plan) => (plan.path, plan.lead_on),
                None => return,
            },
        };
        let mut full = vec![here];
        full.extend(path.into_iter().skip_while(|p| dist_m(*p, here) < 1.0));
        self.lead_on = lead_on;
        self.set_taxi_route(&full, field);
        self.kin.lights.taxi = true;
        self.set_phase(Phase::TaxiOut, events);
    }

    /// The owed readback ([`Order::readback_owed`]), once per owing: given
    /// when it comes owed, given again if it is still owed
    /// [`READBACK_RETRY_S`] later (not taken), [`READBACK_TRIES`] times in
    /// all, then "say again" once, then the flight leaves the frequency
    /// ([`Despawn::ReadbackFailed`], true: the substep stops). Nothing owed
    /// forgets it, so the next identical clearance is read back afresh.
    fn answer_readback(&mut self, o: &Order, events: &mut Vec<Event>) -> bool {
        let r = &mut self.radio;
        let Some(rb) = &o.readback_owed else {
            r.last_readback = None;
            r.readback_tries = 0;
            r.said_again = false;
            return false;
        };
        let call = if r.last_readback.as_ref() != Some(rb) {
            r.last_readback = Some(rb.clone());
            r.readback_tries = 1;
            r.said_again = false;
            Some(Call::Readback(rb.clone()))
        } else if self.age_s - r.readback_at_s < READBACK_RETRY_S {
            None
        } else if r.readback_tries < READBACK_TRIES {
            r.readback_tries += 1;
            Some(Call::Readback(rb.clone()))
        } else if !r.said_again {
            r.said_again = true;
            Some(Call::SayAgain)
        } else {
            self.set_phase(Phase::Gone, events);
            events.push(Event::Despawn {
                id: self.id,
                reason: Despawn::ReadbackFailed,
            });
            return true;
        };
        if let Some(call) = call {
            self.radio.readback_at_s = self.age_s;
            events.push(Event::Call { id: self.id, call });
        }
        false
    }

    /// Taxi `full`, stopping short of every runway it crosses (by the
    /// strips' geometry) until the tower clears the crossing.
    fn set_taxi_route(&mut self, full: &[LatLon], field: &FieldModel) {
        let cut = field.cut_at_crossings(full);
        let mut pieces = cut.pieces.into_iter();
        self.route = Route::taxi(pieces.next().unwrap_or_default());
        self.taxi_rest = cut.crossings.into_iter().zip(pieces).collect();
        self.cross_asked_at = None;
    }

    /// Stopped at the end of the piece of taxi route being driven: true
    /// when it is the route's end. Short of a crossing it asks to cross
    /// (again every [`CROSS_ASK_RETRY_S`]) and holds until the tower has
    /// cleared it across THAT strip and heard the readback; then it drives
    /// the next piece.
    fn past_crossings(&mut self, o: &Order, field: &FieldModel, events: &mut Vec<Event>) -> bool {
        let Some((runway, _)) = self.taxi_rest.first() else {
            return true;
        };
        let same_strip = |a: &str, b: &str| match (field.strip(a), field.strip(b)) {
            (Some(x), Some(y)) => std::ptr::eq(x, y),
            _ => false,
        };
        let cleared = o
            .cross_cleared
            .as_deref()
            .is_some_and(|c| same_strip(c, runway))
            && o.readback_owed.is_none();
        if cleared {
            let (_, piece) = self.taxi_rest.remove(0);
            let here = self.kin.pos();
            let mut path = vec![here];
            path.extend(piece.into_iter().skip_while(|p| dist_m(*p, here) < 1.0));
            self.route = Route::taxi(path);
            self.cross_asked_at = None;
            return false;
        }
        let due = self
            .cross_asked_at
            .is_none_or(|t| self.phase_s - t >= CROSS_ASK_RETRY_S);
        if !o.hold && due {
            self.cross_asked_at = Some(self.phase_s);
            events.push(Event::Call {
                id: self.id,
                call: Call::RequestCrossing {
                    runway: runway.clone(),
                },
            });
        }
        false
    }

    /// From the hold onto the centreline: the drawn lead-on, then along
    /// the centreline far enough to stop aligned.
    fn line_up_path(&self, end: &RunwayEnd) -> Vec<LatLon> {
        let here = self.kin.pos();
        let mut path = vec![here];
        path.extend(
            self.lead_on
                .iter()
                .copied()
                .skip_while(|p| dist_m(*p, here) < 1.0),
        );
        let last = *path.last().unwrap_or(&here);
        let along = end.frame(last).0.max(0.0);
        let on = end.at(along, 0.0);
        if dist_m(on, last) > 1.0 {
            path.push(on);
        }
        path.push(end.at(along + LINE_UP_ALIGN_M, 0.0));
        path
    }

    /// The route from the takeoff run onward: the circuit, or the way out.
    fn takeoff_route(&self, end: &RunwayEnd, _field: &FieldModel) -> Route {
        let here = self.kin.pos();
        let along = end.frame(here).0.clamp(0.0, end.length_m - 1.0);
        let start = end.at(along, 0.0);
        match &self.plan {
            Plan::Departure { exit_bearing, .. } => {
                let upwind_end = end.at(end.length_m + 1.5 * NM, 0.0);
                Route::new(
                    vec![
                        start,
                        end.far,
                        upwind_end,
                        project_m(upwind_end, *exit_bearing, 30.0 * NM),
                    ],
                    vec![Leg::Runway, Leg::Upwind, Leg::Outbound],
                    None,
                )
            }
            _ => {
                let p = pattern_points(end, self.perf);
                Route::new(
                    vec![
                        start,
                        end.far,
                        p.upwind_end,
                        p.crosswind_end,
                        p.downwind_end,
                        p.base_end,
                        end.thr,
                        end.far,
                    ],
                    vec![
                        Leg::Runway,
                        Leg::Upwind,
                        Leg::Crosswind,
                        Leg::Downwind,
                        Leg::Base,
                        Leg::Final,
                        Leg::Runway,
                    ],
                    Some(6),
                )
            }
        }
    }

    fn roll(&mut self, dt: f64, events: &mut Vec<Event>) {
        self.kin.lights.landing = true;
        self.kin.lights.strobe = true;
        self.kin.lights.taxi = false;
        let spec = FollowSpec {
            kts: self.perf.vr_kts + 10.0,
            max_turn_dps: self.perf.turn_taxi_dps,
            accel_kts_s: self.perf.roll_accel_kts_s,
            decel_kts_s: self.perf.decel_kts_s,
            stop_at_end: false,
            slow_for_corners: false,
            min_look_m: 30.0,
        };
        follow(
            &self.route.path,
            &mut self.route.follow,
            &mut self.kin,
            &spec,
            dt,
        );
        if self.kin.gs_kts >= self.perf.vr_kts {
            self.kin.on_ground = false;
            self.kin.vs_fpm = 0.0;
            self.ext_m = 0.0;
            self.set_phase(Phase::Climb, events);
        }
    }

    /// Every airborne phase: the leg's height and speed, the path, and
    /// the phase that follows from the leg reached.
    fn fly(
        &mut self,
        dt: f64,
        o: &Order,
        field: &FieldModel,
        end: &RunwayEnd,
        others: &[Seen],
        events: &mut Vec<Event>,
    ) {
        let p = self.perf;
        let elev = end.elev_ft;
        let agl = self.kin.alt_ft - elev;
        let pattern_alt = elev + p.pattern_agl_ft;
        let here = self.kin.pos();
        let leg = self.route.leg();
        let path_to_thr = self.route.thr_idx.map(|i| self.route.dist_to(here, i));
        let glide = path_to_thr.map(|d| elev + TCH_FT + glide_ft(d));
        let past_abeam = end.frame(here).0 < 0.0;

        if self.phase == Phase::Arrive {
            let (miles, from) = inbound_position(field, here);
            self.call_once(Call::Inbound { miles, from }, events);
        }

        // Height and speed for the leg.
        // The final's own path, on the runway frame, past the threshold
        // too (it meets the ground at the aim point).
        let final_glide =
            elev + TCH_FT - end.frame(here).0 * GLIDE_DEG.to_radians().tan() * FT_PER_M;
        let landing = self.phase == Phase::Final;
        let (alt_target, kts) = match leg {
            Leg::Final | Leg::Runway if landing => (
                final_glide,
                if agl < 300.0 { p.vref_kts } else { p.app_kts },
            ),
            Leg::Upwind | Leg::Runway => {
                let target = match self.plan {
                    Plan::Departure { .. } => elev + DEPART_CLIMB_AGL_FT,
                    _ => pattern_alt,
                };
                (target, if agl < 400.0 { p.v2_kts } else { p.climb_kts })
            }
            Leg::Crosswind => (pattern_alt, p.climb_kts),
            // The 3 degrees along the rest of the circuit: the descent
            // begins on the downwind wherever that path is below the
            // circuit, so the final is joined ON it.
            Leg::Downwind => (glide.unwrap_or(pattern_alt).min(pattern_alt), p.pattern_kts),
            Leg::Base => (glide.unwrap_or(pattern_alt).min(pattern_alt), p.app_kts),
            Leg::Final => (
                glide.unwrap_or(elev),
                if agl < 300.0 { p.vref_kts } else { p.app_kts },
            ),
            Leg::Inbound => {
                let a = match (&self.plan, glide) {
                    (
                        Plan::Arrival {
                            straight_in: true, ..
                        },
                        Some(g),
                    ) => g.min(self.entry_alt_ft),
                    _ => pattern_alt,
                };
                (a, p.pattern_kts)
            }
            Leg::Outbound | Leg::Taxi => (elev + DEPART_CLIMB_AGL_FT, p.climb_kts.min(250.0)),
        };
        // On the path (the final, or a descending leg whose target IS the
        // path) the descent is a ramp: fed forward on the groundspeed,
        // trimmed on the error. A P law alone lags a ramp and arrives high.
        let on_path = matches!(leg, Leg::Downwind | Leg::Base | Leg::Inbound)
            && glide.is_some_and(|g| (g - alt_target).abs() < 1.0);
        if landing || leg == Leg::Final || on_path {
            let ff = -self.kin.gs_kts * KT_TO_MS * 196.85 * GLIDE_DEG.to_radians().tan();
            let fix = (alt_target - self.kin.alt_ft) * 60.0 / crate::kin::VS_TAU_S;
            self.kin
                .vs_toward((ff + fix).clamp(-p.descent_fpm * 1.5, p.climb_fpm), dt);
        } else {
            self.kin
                .vertical_toward(alt_target, p.climb_fpm, p.descent_fpm, dt);
        }

        // The look of it.
        let gear_down = matches!(leg, Leg::Base | Leg::Final)
            || (leg == Leg::Downwind && past_abeam)
            || (leg == Leg::Upwind && agl < 300.0);
        self.kin.gear = if gear_down || !p.retractable_gear {
            1.0
        } else {
            0.0
        };
        self.kin.flaps = match leg {
            Leg::Final => 1.0,
            Leg::Base => 0.5,
            Leg::Downwind if past_abeam => 0.25,
            Leg::Upwind if agl < 400.0 && p.retractable_gear => 0.25,
            _ => 0.0,
        };
        self.kin.lights.landing = agl < 1000.0;
        self.kin.lights.nav = true;
        self.kin.lights.strobe = true;

        // Number two: the downwind runs on until the leader is on short
        // final.
        if leg == Leg::Downwind {
            if let Some(lid) = o.follow {
                let seg = self.route.follow.seg;
                let left = dist_m(here, self.route.path[seg + 1]);
                let radius = self.kin.gs_kts * KT_TO_MS / p.turn_air_dps.to_radians();
                // Our own time to the threshold from here: the rest of the
                // downwind, the base, the final, at the pattern's pace.
                let own_s = (left
                    + self
                        .route
                        .path
                        .get(seg + 2)
                        .map_or(0.0, |b| dist_m(self.route.path[seg + 1], *b) + dist_m(*b, end.thr)))
                    / (self.kin.gs_kts.max(40.0) * KT_TO_MS);
                if left < radius + 200.0
                    && self.ext_m < MAX_DOWNWIND_EXT_NM * NM
                    && !leader_ready(lid, others, end, own_s)
                {
                    let step = self.kin.gs_kts * KT_TO_MS * dt;
                    for k in [seg + 1, seg + 2] {
                        if let Some(pt) = self.route.path.get_mut(k) {
                            *pt = project_m(*pt, end.course_deg + 180.0, step);
                        }
                    }
                    self.ext_m += step;
                }
            }
        }

        let spec = FollowSpec {
            accel_kts_s: 2.0,
            decel_kts_s: 1.5,
            ..FollowSpec::air(kts, p.turn_air_dps)
        };
        follow(
            &self.route.path,
            &mut self.route.follow,
            &mut self.kin,
            &spec,
            dt,
        );

        let agl = self.kin.alt_ft - elev;
        let leg = self.route.leg();
        match self.phase {
            Phase::Climb if leg == Leg::Crosswind => self.set_phase(
                Phase::Pattern {
                    leg: Leg::Crosswind,
                },
                events,
            ),
            Phase::Climb if leg == Leg::Outbound => self.set_phase(Phase::Leave, events),
            Phase::GoAround if leg == Leg::Crosswind => self.set_phase(
                Phase::Pattern {
                    leg: Leg::Crosswind,
                },
                events,
            ),
            Phase::Leave => {
                let gone = field
                    .reference()
                    .is_none_or(|r| dist_m(r, self.kin.pos()) > DEPART_GONE_NM * NM);
                if gone {
                    self.set_phase(Phase::Gone, events);
                    events.push(Event::Despawn {
                        id: self.id,
                        reason: Despawn::LeftTheArea,
                    });
                }
            }
            Phase::Pattern { .. } | Phase::Arrive => {
                let next = match leg {
                    Leg::Downwind => Some((Phase::Pattern { leg }, Call::Downwind)),
                    Leg::Base => Some((Phase::Pattern { leg }, Call::Base)),
                    Leg::Final => Some((Phase::Final, Call::Final)),
                    _ => None,
                };
                if let Some((to, call)) = next {
                    if to != self.phase {
                        self.set_phase(to, events);
                        self.call_once(call, events);
                    }
                }
                if o.go_around && matches!(self.phase, Phase::Pattern { leg: Leg::Base }) {
                    self.go_around(end, events);
                }
            }
            Phase::Final => {
                let uncleared = o.state != TowerState::ClearedToLand && agl < NO_CLEARANCE_AGL_FT;
                if o.go_around || uncleared {
                    self.go_around(end, events);
                } else if agl < FLARE_AGL_FT {
                    self.set_phase(Phase::Landing, events);
                }
            }
            _ => {}
        }
    }

    /// Climb away on the runway heading and fly the circuit again.
    fn go_around(&mut self, end: &RunwayEnd, events: &mut Vec<Event>) {
        let here = self.kin.pos();
        let p = pattern_points(end, self.perf);
        let ahead = end.frame(here).0.max(end.length_m) + 300.0;
        let upwind_end = if end.frame(p.upwind_end).0 > ahead {
            p.upwind_end
        } else {
            end.at(ahead, 0.0)
        };
        self.route = Route::new(
            vec![
                here,
                upwind_end,
                p.crosswind_end,
                p.downwind_end,
                p.base_end,
                end.thr,
                end.far,
            ],
            vec![
                Leg::Upwind,
                Leg::Crosswind,
                Leg::Downwind,
                Leg::Base,
                Leg::Final,
                Leg::Runway,
            ],
            Some(5),
        );
        self.ext_m = 0.0;
        self.kin.on_ground = false;
        self.set_phase(Phase::GoAround, events);
        self.call_once(Call::GoingAround, events);
    }

    fn landing(
        &mut self,
        dt: f64,
        o: &Order,
        field: &FieldModel,
        end: &RunwayEnd,
        events: &mut Vec<Event>,
    ) {
        let p = self.perf;
        if !self.kin.on_ground {
            if o.go_around {
                self.go_around(end, events);
                return;
            }
            // Half the approach's sink, never less than the light
            // aircraft's: a fast aircraft floating at 200 fpm lands long.
            let approach_sink = p.vref_kts * KT_TO_MS * 196.85 * GLIDE_DEG.to_radians().tan();
            self.kin
                .vs_toward(-FLARE_SINK_FPM.max(0.5 * approach_sink), dt);
            let spec = FollowSpec {
                accel_kts_s: 0.0,
                decel_kts_s: 1.0,
                ..FollowSpec::air(p.vref_kts - 5.0, p.turn_air_dps)
            };
            follow(
                &self.route.path,
                &mut self.route.follow,
                &mut self.kin,
                &spec,
                dt,
            );
            if self.kin.alt_ft <= end.elev_ft {
                self.kin.alt_ft = end.elev_ft;
                self.kin.on_ground = true;
                self.kin.vs_fpm = 0.0;
                self.kin.pitch = 0.0;
                let at = self.kin.pos();
                self.touchdown = Some(at);
                events.push(Event::Touchdown {
                    id: self.id,
                    at,
                    along_m: end.frame(at).0,
                });
            }
            return;
        }
        let exit_kts = p.taxi_kts + 2.0;
        let spec = FollowSpec {
            kts: exit_kts,
            max_turn_dps: p.turn_taxi_dps,
            accel_kts_s: 0.0,
            decel_kts_s: p.decel_kts_s,
            stop_at_end: false,
            slow_for_corners: false,
            min_look_m: 30.0,
        };
        follow(
            &self.route.path,
            &mut self.route.follow,
            &mut self.kin,
            &spec,
            dt,
        );
        self.kin.flaps = 0.0;
        if self.kin.gs_kts > exit_kts + 0.05 {
            return;
        }
        if let Plan::Circuit { laps, .. } = self.plan {
            if self.laps_done + 1 < laps {
                self.laps_done += 1;
                self.route = self.takeoff_route(end, field);
                self.set_phase(Phase::Roll, events);
                return;
            }
        }
        let here = self.kin.pos();
        let path = match field.vacate(here, self.kin.hdg_deg, end) {
            Some(v) => {
                let mut full = vec![here];
                full.extend(v.path.into_iter().skip_while(|p| dist_m(*p, here) < 1.0));
                full
            }
            // No network to leave by: stop on the strip and report.
            None => vec![here],
        };
        self.kin.lights.landing = false;
        self.kin.lights.strobe = false;
        self.kin.lights.taxi = true;
        self.route = Route::taxi(path);
        self.vacated_at_s = None;
        self.set_phase(Phase::Vacate, events);
    }

    fn start_taxi_in(
        &mut self,
        o: &Order,
        field: &FieldModel,
        others: &[Seen],
        events: &mut Vec<Event>,
    ) {
        let here = self.kin.pos();
        let occupied: Vec<LatLon> = others
            .iter()
            .filter(|s| s.id != self.id && s.phase == Phase::Parked)
            .map(|s| s.pos)
            .collect();
        let own = self
            .stand
            .as_deref()
            .and_then(|n| field.stand(n))
            .filter(|s| !crate::field::stand_taken(s, &occupied));
        let Some(stand) = own.or_else(|| field.free_stand(here, &occupied)) else {
            return;
        };
        let mut name = stand.name.clone();
        // The tower's route is driven unless it ends on a stand somebody
        // stands on or is taxiing in to: the tower picks from where the
        // aircraft stand NOW, so two arrivals in a row were both sent to
        // the one free stand (the hour sweep, 2026-10-08). The stand it
        // ends on is the one taken.
        let tower = o.taxi_path.as_ref().filter(|p| {
            p.last().is_some_and(|end| {
                !occupied
                    .iter()
                    .any(|q| dist_m(*q, *end) < crate::field::STAND_TAKEN_M)
            })
        });
        self.tower_route = tower.is_some();
        let path = match tower {
            Some(p) => {
                let end = *p.last().unwrap_or(&here);
                if let Some(s) = field.net.stands.iter().find(|s| {
                    s.is_suitable() && dist_m((s.lat, s.lon), end) < crate::field::STAND_TAKEN_M
                }) {
                    name = s.name.clone();
                }
                p.clone()
            }
            None => match field.taxi_to_stand(here, stand) {
                Some(plan) => plan.path,
                None => return,
            },
        };
        let mut full = vec![here];
        full.extend(path.into_iter().skip_while(|p| dist_m(*p, here) < 1.0));
        self.stand = Some(name);
        self.set_taxi_route(&full, field);
        self.set_phase(Phase::TaxiIn, events);
    }

    /// A new way to where the taxi is going that keeps `clear_m` from
    /// `avoid` (an aircraft that will not move off the route), planned on
    /// the network without the edges that pass it: the ground spacing's
    /// guard ([`crate::spacing::apply`]), which before drove one aircraft
    /// THROUGH the other (2026-10-09). False, and nothing changes, when
    /// there is none.
    pub(crate) fn reroute_around(
        &mut self,
        field: &FieldModel,
        avoid: LatLon,
        clear_m: f64,
    ) -> bool {
        // The tower's cleared route is the tower's to change: driven off
        // it, the tower called "off the cleared taxi route" (PANC and ENBO,
        // the hour sweep, 2026-10-09).
        if self.tower_route && self.phase != Phase::Vacate {
            return false;
        }
        let here = self.kin.pos();
        let open = field.without_edges_near(avoid, clear_m);
        let end = self.plan.runway().and_then(|r| field.end(r));
        let planned = match self.phase {
            Phase::TaxiOut => end
                .as_ref()
                .and_then(|e| open.taxi_to_runway(here, e))
                .map(|p| (p.path, Some(p.lead_on))),
            Phase::TaxiIn => self
                .stand
                .as_deref()
                .and_then(|n| field.stand(n))
                .and_then(|s| open.taxi_to_stand(here, s))
                .map(|p| (p.path, None)),
            Phase::Vacate => end
                .as_ref()
                .and_then(|e| open.vacate(here, self.kin.hdg_deg, e))
                .map(|v| (v.path, None)),
            _ => None,
        };
        let Some((path, lead_on)) = planned else {
            return false;
        };
        let mut full = vec![here];
        full.extend(path.into_iter().skip_while(|p| dist_m(*p, here) < 1.0));
        // The way from where it stands to the first node is no edge: it can
        // still run through the blocker.
        if full.len() < 2 || crate::kin::cross_track_m(&full, avoid) < clear_m {
            return false;
        }
        if self.phase == Phase::Vacate {
            self.route = Route::taxi(full);
        } else {
            if let Some(l) = lead_on.filter(|l| !l.is_empty()) {
                self.lead_on = l;
            }
            self.set_taxi_route(&full, field);
        }
        true
    }

    /// A number from this aircraft's own generator (jitter for the bin's
    /// schedules; the phase machine itself draws none).
    pub fn draw(&mut self) -> f64 {
        self.rng.next_f64()
    }
}

/// Feet of height on the 3 degree path `d_m` metres out.
pub fn glide_ft(d_m: f64) -> f64 {
    d_m.max(0.0) * GLIDE_DEG.to_radians().tan() * FT_PER_M
}

/// The straight-in's final fix: far enough out to be established.
fn straight_in_fix_m(p: &Perf) -> f64 {
    (p.base_ext_nm * 2.0).max(4.0) * NM
}

/// The circuit's corners for `end`, flown by `p`.
pub struct PatternPoints {
    /// Off the centreline, signed (right positive).
    pub offset: f64,
    pub upwind_end: LatLon,
    pub crosswind_end: LatLon,
    pub downwind_end: LatLon,
    pub base_end: LatLon,
}

/// The rectangle: upwind to `base_ext` past the far end, the downwind
/// `downwind_offset` out on the pattern side, base `base_ext` before the
/// threshold.
pub fn pattern_points(end: &RunwayEnd, p: &Perf) -> PatternPoints {
    let ext = p.base_ext_nm * NM;
    let offset = end.side() * p.downwind_offset_nm * NM;
    PatternPoints {
        offset,
        upwind_end: end.at(end.length_m + ext, 0.0),
        crosswind_end: end.at(end.length_m + ext, offset),
        downwind_end: end.at(-ext, offset),
        base_end: end.at(-ext, 0.0),
    }
}

/// The leader is far enough ahead for number two to turn base: on final
/// and off the runway again before we reach it (`own_s`, our time to the
/// threshold from here), off the runway, or not in the circuit at all. A
/// leader still ON the runway (rolling out, vacating, lining up, rolling)
/// is the occupant the tower sequenced the downwind behind: the base waits
/// for it to be off (BIEG 03, 2026-10-08: a backtracking jet held the strip
/// for minutes and the base turned on time went around). Timed, never a
/// fixed distance: a 737 that turned base with a 172 at 1.5 nm on final
/// caught it up and went around behind it, five times running.
pub(crate) fn leader_ready(lid: u32, others: &[Seen], end: &RunwayEnd, own_s: f64) -> bool {
    let Some(l) = others.iter().find(|s| s.id == lid) else {
        return true;
    };
    match l.phase {
        Phase::Final => {
            let to_thr_s = dist_m(l.pos, end.thr) / (l.gs_kts.max(40.0) * KT_TO_MS);
            to_thr_s + LEADER_RUNWAY_S + FOLLOW_GAP_S <= own_s
        }
        Phase::Climb | Phase::Arrive | Phase::Pattern { .. } | Phase::GoAround => false,
        Phase::Landing | Phase::Vacate | Phase::LineUp | Phase::Roll => false,
        _ => true,
    }
}

/// Miles and the compass point from the field, for the inbound call.
fn inbound_position(field: &FieldModel, here: LatLon) -> (u32, String) {
    let Some(r) = field.reference() else {
        return (0, String::new());
    };
    let miles = (dist_m(r, here) / NM).round() as u32;
    const POINTS: [&str; 8] = [
        "north",
        "north east",
        "east",
        "south east",
        "south",
        "south west",
        "west",
        "north west",
    ];
    let i = ((bearing(r, here) + 22.5).rem_euclid(360.0) / 45.0) as usize % 8;
    (miles, POINTS[i].to_string())
}
