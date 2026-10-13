//! THE WHOLE FLIGHT, the link's half (2026-10-13; the model is
//! `traffic::track`). The pilot at LEMG saw an aircraft "pop up out of
//! nowhere" on the EFB map: an arrival spawned 25 to 40 nm out, a departure
//! vanished at 12. Every timetabled flight of a simulated field now exists
//! from the origin's blocks to the destination's:
//!
//! - An arrival still to enter is its [`Due`] movement; its model
//!   ([`Modeled`]) ends at the very aircraft its spawn will make
//!   (`Movement::entry_anchor`), at its first due time, and holds there
//!   past it until the spawn takes it, with the id the model showed it
//!   under. No second fleet: the movement IS the flight, the model only
//!   its position.
//! - A departure that leaves the area (`Fleet::take_left`) is flown on by
//!   the model from its state at that substep, the same aircraft value,
//!   id and all, to its destination ([`Link::beyond`]); one gone before
//!   the start (`traffic::Start::Away`) from its wheels up.
//!
//! The model ticks at most once a second of fleet time ([`MODEL_EVERY_S`]),
//! O(flights): a few dozen closed-form evaluations, no sim read. Its
//! aircraft are drawn by the plugin only inside the draw ring
//! (`traffic_sink::DrawCull`), never TCAS targets for the station.

use traffic::board::Dir;
use traffic::track::{gc_dist_m, End, Fix, Stage, Whole, TAXI_IN_S, TAXI_OUT_S};
use traffic::{FlightInfo, Kin, MovementKind, Phase, Plan, TrafficAircraft};

use super::{Due, Link};

/// The model ticks at most this often, fleet seconds.
pub(super) const MODEL_EVERY_S: f64 = 1.0;

/// An arrival's model is built once its entry is this near, fleet seconds:
/// the longest flight the tables name and its taxi.
const MODEL_AHEAD_S: f64 = 9.0 * 3600.0;

/// A flight the model flies: the aircraft as drawn and shown (the
/// simulation's id, callsign, type and plan; the model's position), its
/// whole flight, and its timetable's words.
#[derive(Debug, Clone)]
pub(crate) struct Modeled {
    pub(crate) ac: TrafficAircraft,
    pub(crate) whole: Whole,
    pub(crate) info: Option<FlightInfo>,
    /// What the model said at its last tick; `None`: not shown now (before
    /// its off-block at the origin).
    pub(crate) stage: Option<Stage>,
}

impl Modeled {
    /// The model at `t`: the aircraft put where it says.
    fn fly(&mut self, t: f64) {
        self.stage = self.whole.at(t).map(|f| {
            put(&mut self.ac, &f, t < self.whole.track.t_a);
            f.stage
        });
    }

    /// Shown now (between its off-block and its on-block or its handover).
    pub(crate) fn shown(&self) -> bool {
        self.stage.is_some()
    }

    /// What it is doing, in a pilot's words.
    pub(crate) fn words(&self) -> &'static str {
        match self.stage {
            Some(Stage::Taxi) if self.ac.phase == Phase::TaxiOut => "taxiing out",
            Some(Stage::Taxi) => "taxiing in",
            Some(Stage::Holding) => "holding",
            _ if self.ac.kin.vs_fpm > 300.0 => "climbing",
            _ if self.ac.kin.vs_fpm < -300.0 => "descending",
            _ => "cruising",
        }
    }
}

/// The aircraft where the model puts it: position, height, speed and
/// heading, gear and lights as a crew sets them, the phase that draws
/// its engines running.
fn put(ac: &mut TrafficAircraft, f: &Fix, leaving: bool) {
    let k = &mut ac.kin;
    k.lat = f.pos.0;
    k.lon = f.pos.1;
    k.alt_ft = f.alt_ft;
    k.gs_kts = f.gs_kts;
    k.hdg_deg = f.hdg_deg;
    k.track_deg = f.hdg_deg;
    k.vs_fpm = f.vs_fpm;
    k.on_ground = f.on_ground;
    k.roll = 0.0;
    k.flaps = 0.0;
    k.pitch = if f.on_ground {
        0.0
    } else {
        2.0 + (f.vs_fpm / (f.gs_kts.max(60.0) * 101.3))
            .atan()
            .to_degrees()
    };
    k.gear = if f.on_ground || !ac.perf.retractable_gear {
        1.0
    } else {
        0.0
    };
    k.lights.nav = true;
    k.lights.beacon = true;
    k.lights.strobe = !f.on_ground;
    k.lights.taxi = f.on_ground;
    k.lights.landing = !f.on_ground && f.alt_ft < 10_000.0;
    ac.phase = match (f.on_ground, &ac.plan) {
        (true, _) if leaving => Phase::TaxiOut,
        (true, _) => Phase::TaxiIn,
        (false, Plan::Departure { .. }) => Phase::Leave,
        (false, _) => Phase::Arrive,
    };
}

/// The handover in, as the spawn takes `d` as `ac` at fleet time `now`:
/// blackboxed with how far the model and the simulation stood apart
/// (`off_m`, `off_ft`: zero but for a runway turned since the model's last
/// tick) and how long the flight held at its entry. Its whole flight, for
/// the card.
pub(super) fn entered(d: &Due, ac: &TrafficAircraft, now: f64, field: &str) -> Option<Whole> {
    let m = d.model.as_ref()?.1.as_ref()?;
    let fix = m.whole.at(now);
    super::rec(
        field,
        serde_json::json!({
            "event": "handover", "dir": "in", "id": ac.id, "callsign": ac.callsign,
            "from": m.whole.origin,
            "off_m": fix.map(|f| (gc_dist_m(f.pos, ac.kin.pos()) * 10.0).round() / 10.0),
            "off_ft": fix.map(|f| (f.alt_ft - ac.kin.alt_ft).round()),
            "alt_ft": ac.kin.alt_ft.round(),
            "held_s": (now - d.first_s).max(0.0).round(),
            "shown": m.shown(),
        }),
    );
    Some(m.whole.clone())
}

impl Link {
    /// The model's tick: the departures that left the area this step
    /// handed to it; then, at most every [`MODEL_EVERY_S`], every flight it
    /// flies put where it is now, an arrival's model built (or rebuilt for
    /// a runway turned) once its entry is within [`MODEL_AHEAD_S`], an id
    /// given to each the first time it shows, and the flights on their
    /// blocks at the other end dropped.
    pub(super) fn fly_models(&mut self) {
        let now = self.fleet.clock.now_s();
        for (ac, at_s) in self.fleet.take_left() {
            if let Some(mut m) = self.outbound(ac, at_s) {
                m.fly(now);
                super::rec(
                    self.icao(),
                    serde_json::json!({
                        "event": "handover", "dir": "out", "id": m.ac.id,
                        "callsign": m.ac.callsign, "to": m.whole.dest,
                        "alt_ft": m.whole.track.a.alt_ft.round(),
                        "cruise_ft": m.whole.track.peak_ft.round(),
                        "eta_s": (m.whole.on_block_s - at_s).round(),
                    }),
                );
                self.beyond.push(m);
            }
        }
        if now - self.model_s < MODEL_EVERY_S - 1e-6 {
            return;
        }
        self.model_s = now;
        for m in std::mem::take(&mut self.away) {
            if let Some(x) = self.from_field(m, now) {
                self.beyond.push(x);
            }
        }
        let runway = self.runway_now();
        for i in 0..self.due.len() {
            let d = &self.due[i];
            if d.m.kind != MovementKind::Arrival
                || d.m.start.is_some()
                || d.m.flight.is_none()
                || d.first_s - now > MODEL_AHEAD_S
            {
                continue;
            }
            let rwy = runway
                .clone()
                .filter(|r| self.fleet.field.end(r).is_some())
                .unwrap_or_else(|| d.m.runway.clone());
            // Built once per runway; a flight the model cannot fly (an
            // origin the timetable does not place) is not tried again.
            if d.model.as_ref().is_none_or(|(r, _)| *r != rwy) {
                let built = self.inbound(d, &rwy);
                self.due[i].model = Some((rwy, built));
            }
            let d = &mut self.due[i];
            let Some((_, Some(m))) = d.model.as_mut() else {
                continue;
            };
            m.fly(now);
            if m.shown() && d.id.is_none() {
                d.id = Some(self.ids.peek());
                self.ids.bump();
            }
            if let Some(id) = d.id {
                m.ac.id = id;
            }
        }
        self.beyond.retain_mut(|m| {
            m.fly(now);
            !m.whole.over(now)
        });
        let fleet = &self.fleet;
        self.entered
            .retain(|id, (cs, _)| fleet.get(*id).is_some_and(|a| a.callsign == *cs));
    }

    /// An arrival's model, for `runway`: from its origin to the aircraft
    /// its spawn will make, reached at its first due time.
    fn inbound(&self, d: &Due, runway: &str) -> Option<Modeled> {
        let info = d.m.flight.clone()?;
        let mut m = d.m.clone();
        m.runway = runway.to_string();
        let ac = m.entry_anchor(d.id.unwrap_or(0), &self.fleet.field)?;
        let here = self.fleet.field.reference()?;
        let on_block = info
            .est_s
            .map_or(d.first_s + 20.0 * 60.0, |e| e - self.t0_local_s);
        let whole = Whole::inbound(
            &info.other,
            self.icao(),
            here,
            End::air(&ac.kin),
            d.first_s,
            on_block,
            ac.perf,
        )?;
        Some(Modeled {
            ac,
            whole,
            info: Some(info),
            stage: None,
        })
    }

    /// A departure that left the area at `at_s`, as it was then: flown on
    /// to its destination. `None` for a flight the board does not carry
    /// or a destination the timetable does not place (it goes, as before).
    fn outbound(&self, ac: TrafficAircraft, at_s: f64) -> Option<Modeled> {
        let b = self
            .board
            .iter()
            .find(|f| f.dir == Dir::Departure && f.callsign == ac.callsign)?;
        let here = self.fleet.field.reference()?;
        let off = b
            .info
            .est_s
            .map_or(at_s - 15.0 * 60.0, |e| e - self.t0_local_s);
        let whole = Whole::outbound(
            self.icao(),
            here,
            &b.info.other,
            End::air(&ac.kin),
            at_s,
            off,
            ac.perf,
        )?;
        Some(Modeled {
            ac,
            whole,
            info: Some(b.info.clone()),
            stage: None,
        })
    }

    /// A departure gone before the start ([`traffic::Start::Away`]): from
    /// the field at its wheels up, [`TAXI_OUT_S`] after its off-block, with
    /// an id of its own. `None` once it is on the blocks at the other end.
    fn from_field(&self, m: traffic::Movement, now: f64) -> Option<Modeled> {
        let Some(traffic::Start::Away { off_s }) = m.start else {
            return None;
        };
        let info = m.flight.clone()?;
        let here = self.fleet.field.reference()?;
        let elev = self.fleet.field.elev_ft();
        let perf = traffic::class_for_type(&m.icao_type).0;
        let whole = Whole::outbound(
            self.icao(),
            here,
            &info.other,
            End::field(here, elev, perf, true),
            off_s + TAXI_OUT_S,
            off_s,
            perf,
        )?;
        if whole.over(now) {
            return None;
        }
        let plan = m.plan(&self.fleet.field).ok()?;
        let id = self.ids.peek();
        self.ids.bump();
        let ac = TrafficAircraft::new(
            id,
            &m.callsign,
            &m.spoken,
            &m.icao_type,
            plan,
            Kin::parked(here, elev, 0.0),
            Phase::Leave,
            m.seed,
        );
        let mut x = Modeled {
            ac,
            whole,
            info: Some(info),
            stage: None,
        };
        x.fly(now);
        Some(x)
    }

    /// Every flight the model shows now: the arrivals still to enter, then
    /// the departures flown on.
    pub(crate) fn modeled(&self) -> impl Iterator<Item = &Modeled> {
        self.due
            .iter()
            .filter_map(|d| d.model.as_ref().and_then(|(_, m)| m.as_ref()))
            .chain(&self.beyond)
            .filter(|m| m.shown())
    }

    /// The model's aircraft in the air, for the plugin's draw ring (on the
    /// ground they are at the far end, out of it anyway).
    pub(crate) fn modeled_drawn(&self) -> impl Iterator<Item = &TrafficAircraft> {
        self.modeled().map(|m| &m.ac).filter(|a| !a.kin.on_ground)
    }

    /// "HH:MM" on the clock at `icao` at fleet time `t` (each country's
    /// offset, `traffic::places`).
    fn clock_at(&self, icao: &str, t: f64) -> String {
        let shift = traffic::places::utc_offset_h(icao, self.summer)
            - traffic::places::utc_offset_h(self.icao(), self.summer);
        traffic::rotation::hhmm(self.t0_local_s + t + shift * 3600.0)
    }

    /// The EFB's journey of a flight: both ends, the off-block on the
    /// origin's clock and the on-block on the destination's, the time since
    /// one and to the other, and the share of the great circle flown.
    fn journey_of(
        &self,
        (origin, o): (&str, (f64, f64)),
        (dest, d): (&str, (f64, f64)),
        off_s: f64,
        on_s: f64,
        pos: (f64, f64),
    ) -> crate::efb::feed::Journey {
        let now = self.fleet.clock.now_s();
        crate::efb::feed::Journey {
            from: origin.to_string(),
            to: dest.to_string(),
            from_pos: o,
            to_pos: d,
            off: self.clock_at(origin, off_s),
            on: self.clock_at(dest, on_s),
            since_s: now - off_s,
            to_go_s: on_s - now,
            progress: traffic::track::progress(o, d, pos),
        }
    }

    /// A modelled flight's journey.
    pub(crate) fn journey_modeled(&self, m: &Modeled) -> crate::efb::feed::Journey {
        let w = &m.whole;
        self.journey_of(
            (&w.origin, w.origin_pos),
            (&w.dest, w.dest_pos),
            w.off_block_s,
            w.on_block_s,
            m.ac.kin.pos(),
        )
    }

    /// A simulated flight's journey, where the board names its other end
    /// and the timetable places it: an arrival's off-block from its model
    /// when it had one, else a block time before its estimate; its on-block
    /// the live estimate in the air; a departure's on-block a block time
    /// after its off-block.
    pub(crate) fn journey_simulated(
        &self,
        a: &TrafficAircraft,
        f: &traffic::board::BoardFlight,
    ) -> Option<crate::efb::feed::Journey> {
        if a.dormant() {
            return None;
        }
        let here = self.fleet.field.reference()?;
        let other = traffic::places::position(&f.info.other)?;
        let now = self.fleet.clock.now_s();
        let fleet_s = |local: f64| local - self.t0_local_s;
        let block = traffic::rotation::block_s(&f.info.other, self.icao(), &a.icao_type)?;
        match f.dir {
            Dir::Arrival => {
                let est = f.info.est_s.map(fleet_s);
                let on = a
                    .slot()
                    .filter(|_| a.phase.airborne())
                    .map(|s| now + s.to_thr_s.max(0.0) + TAXI_IN_S)
                    .or(est)
                    .unwrap_or(now);
                let off = self
                    .entered
                    .get(&a.id)
                    .filter(|(cs, _)| *cs == a.callsign)
                    .map(|(_, w)| w.off_block_s)
                    .unwrap_or(est.unwrap_or(on) - block);
                Some(self.journey_of(
                    (&f.info.other, other),
                    (self.icao(), here),
                    off,
                    on,
                    a.kin.pos(),
                ))
            }
            Dir::Departure => {
                let off = f.info.est_s.map(fleet_s).unwrap_or(now);
                Some(self.journey_of(
                    (self.icao(), here),
                    (&f.info.other, other),
                    off,
                    off + block,
                    a.kin.pos(),
                ))
            }
        }
    }

    /// Set the timetable's season for the other ends' clocks.
    pub(crate) fn set_summer(&mut self, summer: bool) {
        self.summer = summer;
    }
}

#[cfg(test)]
mod tests {
    use super::super::{timetable_start, Link};
    use std::collections::HashMap;
    use std::time::Instant;
    use traffic::{Density, FieldModel};

    fn fixtures() -> String {
        format!("{}/crates/traffic/fixtures", env!("CARGO_MANIFEST_DIR"))
    }

    /// ULMM flying a timetable whose one route is to and from Heathrow
    /// (about 1,500 nm), at 10:00 local on a summer Tuesday, on its desk
    /// (no station, no party line).
    fn heathrow_link() -> Link {
        use traffic::routes::{FieldRoutes, RouteRow};
        let row = traffic::routes::field_routes("LXGB").unwrap().rows[0];
        let rows: &'static [RouteRow] = Box::leak(Box::new([RouteRow {
            summer: 42,
            winter: 42,
            windows: &[(6.0, 22.0)],
            based: false,
            types: &[("A320", 100)],
            ..row
        }]));
        let table = FieldRoutes {
            icao: "ULMM",
            utc_offset_h: 3.0,
            rows,
        };
        let apt = format!("{}/ulmm.dat", fixtures());
        let field = FieldModel::load(&apt, "ULMM")
            .unwrap()
            .unwrap()
            .with_cifp(&format!("{}/cifp", fixtures()));
        let date = traffic::rotation::Date::ymd(2026, 7, 14);
        let (schedule, board, local_s) = timetable_start(
            &field,
            &table,
            Density::Normal,
            7,
            Some(6.0 * 3600.0),
            Some(date.ordinal),
            None,
        );
        let mut link = Link::new(field, schedule, "");
        link.set_board(board, local_s);
        link.set_summer(true);
        link.make_desk();
        link
    }

    /// A WHOLE FLIGHT NEVER JUMPS (2026-10-13): every flight of the field,
    /// by its id, wherever it is (the model's or the simulation's, never
    /// both at once), moves each second no further than its speed takes it;
    /// arrivals come out of the model into the simulation and departures go
    /// the other way under the same id; the modelled flights far out are on
    /// their great circles to or from Heathrow with a journey the EFB can
    /// show.
    #[test]
    fn a_whole_flight_never_jumps_at_either_handover() {
        let mut link = heathrow_link();
        let epoch = Instant::now();
        let mut last: HashMap<u32, ((f64, f64), f64, bool)> = HashMap::new();
        let (mut ins, mut outs, mut far) = (0, 0, 0usize);
        let heathrow = traffic::places::position("EGLL").unwrap();
        for k in 1..=(4 * 3600) {
            let now = epoch + std::time::Duration::from_secs(k);
            link.tick(None, 1.0, now);
            let mut seen: HashMap<u32, ((f64, f64), f64, bool)> = HashMap::new();
            for a in link.fleet().aircraft() {
                seen.insert(a.id, (a.kin.pos(), a.kin.gs_kts, false));
            }
            for m in link.modeled() {
                let was = seen.insert(m.ac.id, (m.ac.kin.pos(), m.ac.kin.gs_kts, true));
                assert!(
                    was.is_none(),
                    "{} both simulated and modelled",
                    m.ac.callsign
                );
                let j = link.journey_modeled(m);
                assert!(j.from == "EGLL" || j.to == "EGLL", "{j:?}");
                assert!((0.0..=1.0).contains(&j.progress));
                // Within reach of ULMM an arrival leaves its great circle
                // for its entry: judge the far ones only.
                if traffic::track::gc_dist_m(m.ac.kin.pos(), heathrow) < 1_000.0 * 1852.0 {
                    far += 1;
                    let gc = traffic::track::gc_point(
                        j.from_pos,
                        j.to_pos,
                        traffic::track::progress(j.from_pos, j.to_pos, m.ac.kin.pos()),
                    );
                    let off = traffic::track::gc_dist_m(gc, m.ac.kin.pos());
                    assert!(
                        off < 30.0 * 1852.0,
                        "{} {off} m off its great circle",
                        m.ac.callsign
                    );
                }
            }
            for (id, (p, gs, modeled)) in &seen {
                if let Some((q, gs0, was)) = last.get(id) {
                    let moved = traffic::track::gc_dist_m(*p, *q);
                    let can = gs.max(*gs0) * traffic::kin::KT_TO_MS;
                    assert!(
                        moved <= 1.3 * can + 30.0,
                        "id {id} jumped {moved:.0} m in a second at {gs:.0} kt \
                         (modelled {modeled}, was {was})"
                    );
                    if *was && !*modeled {
                        ins += 1;
                    }
                    if !*was && *modeled {
                        outs += 1;
                    }
                }
            }
            last = seen;
        }
        println!("{ins} handovers in, {outs} out, {far} far-out samples");
        assert!(ins >= 1, "an arrival came out of the model");
        assert!(outs >= 1, "a departure went into it");
        assert!(far > 0, "flights far out");
    }

    /// LEMG's whole flights on the real install (Malaga's timetable at
    /// 10:00 local on a summer Tuesday, on its desk, 90 minutes): how many
    /// the model flies, what a model tick and the EFB's tap cost, and no
    /// flight ever jumps. `cargo test --release -- --ignored
    /// real_install_lemg_whole_flights --nocapture`.
    #[test]
    #[ignore = "needs X-Plane's Global Airports apt.dat; run with -- --ignored"]
    fn real_install_lemg_whole_flights() {
        let apt = std::env::var("XCOPILOT_APT_DAT").unwrap_or_else(|_| {
            crate::nav::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat")
        });
        let Ok(Some(field)) = FieldModel::load(&apt, "LEMG") else {
            println!("no install, no verdict");
            return;
        };
        let field = field.with_cifp(&crate::nav::test_xplane_path("Custom Data/CIFP"));
        let table = traffic::routes::field_routes("LEMG").unwrap();
        let date = traffic::rotation::Date::ymd(2026, 7, 14);
        let (schedule, board, local_s) = timetable_start(
            &field,
            table,
            Density::Normal,
            traffic::field_seed("LEMG", super::super::SESSION_SEED),
            Some(8.0 * 3600.0),
            Some(date.ordinal),
            None,
        );
        let away = schedule
            .iter()
            .filter(|m| matches!(m.start, Some(traffic::Start::Away { .. })))
            .count();
        let mut link = Link::new(field, schedule, "");
        link.set_board(board, local_s);
        link.set_summer(true);
        link.make_desk();
        let epoch = Instant::now();
        let mut last: HashMap<u32, ((f64, f64), f64, f64, f64)> = HashMap::new();
        let mut worst = 0.0f64;
        let mut worst_ft = 0.0f64;
        let mut wire_max = 0usize;
        let mut entered = std::collections::BTreeSet::new();
        let mut gone = std::collections::BTreeSet::new();
        let mut simulated = std::collections::BTreeSet::new();
        let (mut model_s, mut model_n, mut efb_s, mut shown_max) = (0.0, 0u32, 0.0, 0usize);
        for k in 1..=(90 * 60) {
            link.tick(None, 1.0, epoch + std::time::Duration::from_secs(k));
            // A forced model tick, timed alone.
            link.model_s = f64::NEG_INFINITY;
            let t = Instant::now();
            link.fly_models();
            model_s += t.elapsed().as_secs_f64();
            model_n += 1;
            let t = Instant::now();
            let tap = link.efb_traffic();
            efb_s += t.elapsed().as_secs_f64();
            shown_max = shown_max.max(tap.aircraft.iter().filter(|a| a.modeled).count());
            let wire = serde_json::to_string(&crate::efb::airport::traffic_out(&[tap])).unwrap();
            wire_max = wire_max.max(wire.len());
            let mut seen: HashMap<u32, ((f64, f64), f64, f64, f64)> = HashMap::new();
            for a in link.fleet().aircraft() {
                seen.insert(
                    a.id,
                    (a.kin.pos(), a.kin.gs_kts, a.kin.alt_ft, a.kin.vs_fpm),
                );
                simulated.insert(a.id);
            }
            for m in link.modeled() {
                assert!(seen
                    .insert(
                        m.ac.id,
                        (
                            m.ac.kin.pos(),
                            m.ac.kin.gs_kts,
                            m.ac.kin.alt_ft,
                            m.ac.kin.vs_fpm
                        )
                    )
                    .is_none());
            }
            for (id, (p, gs, alt, vs)) in &seen {
                if let Some((q, gs0, alt0, vs0)) = last.get(id) {
                    let climb = (vs.abs().max(vs0.abs())) / 60.0;
                    let dz = (alt - alt0).abs();
                    worst_ft = worst_ft.max(dz - climb);
                    assert!(dz <= 1.3 * climb + 60.0, "id {id} jumped {dz:.0} ft");
                    let moved = traffic::track::gc_dist_m(*p, *q);
                    let can = gs.max(*gs0) * traffic::kin::KT_TO_MS;
                    worst = worst.max(moved - can);
                    assert!(moved <= 1.3 * can + 30.0, "id {id} jumped {moved:.0} m");
                }
            }
            entered.extend(link.entered.keys().copied());
            gone.extend(
                link.beyond
                    .iter()
                    .map(|m| m.ac.id)
                    .filter(|id| simulated.contains(id)),
            );
            last = seen;
        }
        println!(
            "LEMG 10:00: {away} gone before the start, up to {shown_max} modelled at once; \
             a model tick {:.1} us, the EFB tap {:.1} us; {} arrivals out of the model, {} \
             departures into it; the map's traffic up to {wire_max} bytes; worst excess over a second's flight {worst:.1} m, {worst_ft:.0} ft",
            model_s / f64::from(model_n) * 1e6,
            efb_s / f64::from(model_n) * 1e6,
            entered.len(),
            gone.len(),
        );
        assert!(shown_max > 10, "a busy field's sky");
        assert!(!entered.is_empty(), "arrivals came out of the model");
    }
}
