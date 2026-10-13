//! The tower's runway book: who holds which runway, COMPUTED per query from
//! the strips' clearances and positions and the ghost memory. Nothing here is
//! stored: no static, no lifetime. The four questions the party line and the
//! incursion watch used to answer each from our clearance alone
//! ([`own_strip_claim`], [`player_blocked_strip`],
//! [`runway_occupancy_authorised`], a ghost's conflict with our claim) are
//! queries over one book, so a traffic strip's claim is read by the same
//! rules as ours.

use super::*;

/// Who holds a claim: a flight's strip, or a ghost the replay shows us.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Claimant {
    Strip(FlightId),
    Ghost(String),
}

/// What a claim is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ClaimKind {
    /// Cleared to line up and wait (or a cancelled takeoff): the whole strip.
    LineUp,
    /// Cleared for takeoff: the whole strip.
    Takeoff,
    /// Cleared to land and not yet reported off: the whole strip (ours far
    /// out, its direction).
    Landing,
    /// Taxied to the runway or told to join final: its DIRECTION.
    Sequenced,
    /// The tower's book holds the runway for a flight whose state claims no
    /// movement on it (a clearance's `holds_runway` outside the above).
    Held,
    /// A cleared crossing on the route, moving.
    Crossing,
    /// Cleared to cross the strip ("cross runway one three", the strip's
    /// `crossing` cell): the whole strip, from the clearance until the flight
    /// is seen off it again. Nobody lines up, departs or lands meanwhile.
    CrossingCleared,
    /// Physically on the pavement, on the ground (authorised or not).
    Occupying,
    /// A runway start: the strip the aircraft first appeared on, stopped.
    RunwayStart,
    /// Rolling off the strip it landed on (cleared or not).
    Vacating,
    /// A ghost standing on the pavement, as the replay shows it.
    Observed,
}

/// One claim on a runway.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Claim {
    pub(crate) who: Claimant,
    /// A runway END ("09R"), or a strip's `ident_a` for a claim made from a
    /// position; empty when the clearance named no runway.
    pub(crate) end: String,
    /// The whole strip, either direction; else only this end's direction.
    pub(crate) whole_strip: bool,
    pub(crate) kind: ClaimKind,
    /// The tower's book holds the strip for the claimant (the clearance's
    /// `holds_runway`); false for claims made from a position.
    pub(crate) held: bool,
    /// Since when, where the memory knows it (a ghost's standing clock).
    pub(crate) since: Option<Instant>,
}

/// One flight as the book reads it: its id, its strip and its state where
/// one is known.
pub(crate) struct BookEntry<'a> {
    pub(crate) id: FlightId,
    pub(crate) strip: &'a FlightStrip,
    pub(crate) state: Option<AircraftState>,
}

/// The runway book at one moment. Computed, never stored.
pub(crate) struct RunwayBook {
    claims: Vec<Claim>,
    /// Each flight's state as the book was computed with it.
    states: Vec<(FlightId, AircraftState)>,
    /// The strip (its `ident_a`) under each flight that stands on one.
    under: Vec<(FlightId, String)>,
    /// The field's runways as they were at the computation.
    runways: Vec<nav::Runway>,
}

impl RunwayBook {
    /// The book over our own strip (with our state where the caller holds
    /// it), every open traffic strip (by its clearance and where it was
    /// last seen, [`FlightStrip::seen`]) and the ghost memory.
    pub(crate) fn now(own: Option<AircraftState>) -> RunwayBook {
        // The map's lock is dropped here, before any strip's cell.
        let traffic = traffic_strips();
        let mut entries = vec![BookEntry {
            id: FlightId::Own,
            strip: &OWN_STRIP,
            state: own.or_else(own_seen),
        }];
        entries.extend(traffic.iter().map(|(n, s)| BookEntry {
            id: FlightId::Traffic(*n),
            strip: &**s,
            state: seen(s),
        }));
        RunwayBook::compute(&entries)
    }

    /// The book over these strips and the ghost memory, on the published
    /// field's runways. Each cell is locked alone and dropped before the
    /// next: nothing is held across another.
    pub(crate) fn compute(entries: &[BookEntry<'_>]) -> RunwayBook {
        RunwayBook::compute_on(active_runways(), entries)
    }

    /// [`RunwayBook::compute`] on these runways (a station's own field).
    pub(crate) fn compute_on(runways: Vec<nav::Runway>, entries: &[BookEntry<'_>]) -> RunwayBook {
        let mut book = RunwayBook {
            claims: Vec::new(),
            states: Vec::new(),
            under: Vec::new(),
            runways,
        };
        for e in entries {
            book.add_strip(e);
        }
        book.add_ghosts(standing_ghosts());
        book
    }

    fn add_strip(&mut self, e: &BookEntry<'_>) {
        let who = Claimant::Strip(e.id);
        let clr = e
            .strip
            .clearance
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
            .map(|c| (c.state, c.runway_ident.clone(), c.holds_runway));
        // A flight known to be flying has USED its takeoff clearance, which
        // holds the strip no more (a traffic flight keeps the clearance
        // until it leaves the area, 12 nm out: held whole all that way, it
        // closed the field behind it). Ours too, judged by the aircraft, not
        // by a next call a hand-flown departure may never make: LEMG 12
        // (2026-10-13), our 737 flown by hand after "contact Departure" kept
        // TakeoffCleared and a traffic departure sat held "traffic on the
        // runway" from 14:08Z. The next departure waits out the liftoff's
        // wake ([`wake_remaining_for`]).
        let airborne = e.state.is_some_and(|s| !s.on_ground);
        if let Some((state, ident, held)) = &clr {
            let kind = match state {
                tower::State::LinedUp => Some((ClaimKind::LineUp, true)),
                tower::State::TakeoffCleared if airborne => None,
                tower::State::TakeoffCleared => Some((ClaimKind::Takeoff, true)),
                // Cleared to land and still far out (a reposition flown
                // after an early clearance) holds only the direction: LEJR
                // 2026-10-09, our "cleared to land" from 7 nm held 02 whole
                // through an 11 minute reversal pattern, an A320 went round
                // twice and a departure sat held. Ours only: a traffic
                // flight flies the final it was cleared on, and two of them
                // cleared at once is the conflict the book exists to stop;
                // ours back inside the ring over a landing given meanwhile
                // is the go-around watch's, judged on time. Still a LANDING:
                // a departure judges it in time like any lander
                // ([`Self::departure_blocked`]).
                tower::State::ClearedToLand
                    if *held && e.strip.is_own() && self.far_out(e.state, ident) =>
                {
                    Some((ClaimKind::Landing, false))
                }
                // A landing reported vacated claims nothing.
                tower::State::ClearedToLand if *held => Some((ClaimKind::Landing, true)),
                tower::State::ClearedToLand => None,
                tower::State::TaxiCleared | tower::State::Inbound => {
                    Some((ClaimKind::Sequenced, false))
                }
                tower::State::Parked if *held => Some((ClaimKind::Held, false)),
                tower::State::Parked => None,
            };
            if let Some((kind, whole_strip)) = kind {
                self.claims.push(Claim {
                    who: who.clone(),
                    end: ident.clone(),
                    whole_strip,
                    kind,
                    held: *held,
                    since: None,
                });
            }
        }
        // A crossing the tower cleared holds the strip crossed, whole, from
        // the clearance (the flight is still 20 m short of the pavement)
        // until it is seen off it: the 2026-10-08 world sweep's ULMM, a
        // takeoff released 6 s after two crossings were cleared of 13.
        let crossing = e
            .strip
            .crossing
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
            .map(|c| c.0.clone());
        if let Some(end) = crossing {
            self.claims.push(Claim {
                who: who.clone(),
                end,
                whole_strip: true,
                kind: ClaimKind::CrossingCleared,
                held: false,
                since: None,
            });
        }
        let Some(state) = e.state else {
            return;
        };
        self.states.push((e.id, state));
        let Some((lat, lon)) = state.position() else {
            return;
        };
        // Standing on the pavement, by the nearer end's name.
        if state.on_ground {
            if let Some(end) = self.runways.iter().find_map(|r| on_runway(r, lat, lon)) {
                self.claims.push(Claim {
                    who: who.clone(),
                    end,
                    whole_strip: false,
                    kind: ClaimKind::Occupying,
                    held: false,
                    since: None,
                });
            }
        }
        // The strip under it, and what the book says of its being there.
        let Some(here) = self
            .runways
            .iter()
            .find(|r| on_runway(r, lat, lon).is_some())
            .map(|r| r.ident_a.clone())
        else {
            return;
        };
        self.under.push((e.id, here.clone()));
        let position_claim = |kind| Claim {
            who: who.clone(),
            end: here.clone(),
            whole_strip: false,
            kind,
            held: false,
            since: None,
        };
        let (started_here, landed_here) = {
            let w = e
                .strip
                .compliance_flight
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            (
                w.runway_start.as_deref() == Some(here.as_str()),
                w.landed_strip.as_deref() == Some(here.as_str())
                    && state.gs_kts >= LANDED_VACATING_KTS,
            )
        };
        if started_here {
            self.claims.push(position_claim(ClaimKind::RunwayStart));
        }
        if landed_here {
            self.claims.push(position_claim(ClaimKind::Vacating));
        }
        let arrival_taxi = *e
            .strip
            .taxi_to_stand
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let cleared_taxi = clr
            .as_ref()
            .is_some_and(|(s, _, _)| *s == tower::State::TaxiCleared)
            || arrival_taxi;
        if cleared_taxi && state.gs_kts >= 3.0 && on_cleared_runway_crossing_for(e.strip, lat, lon)
        {
            self.claims.push(position_claim(ClaimKind::Crossing));
        }
    }

    /// Airborne farther than [`LANDING_CLAIM_NM`] from both ends of the
    /// strip `end` is on. Unknown (no state, no position, no strip) is not
    /// far out: the clearance holds as it was given.
    fn far_out(&self, state: Option<AircraftState>, end: &str) -> bool {
        let Some(r) = strip(&self.runways, end) else {
            return false;
        };
        state
            .filter(|s| !s.on_ground)
            .and_then(|s| s.position())
            .is_some_and(|(lat, lon)| {
                let a = nav::haversine_nm(lat, lon, r.lat_a, r.lon_a);
                let b = nav::haversine_nm(lat, lon, r.lat_b, r.lon_b);
                a.min(b) > LANDING_CLAIM_NM
            })
    }

    fn add_ghosts(&mut self, mut ghosts: Vec<(String, f64, f64, Instant)>) {
        ghosts.sort_by(|a, b| a.0.cmp(&b.0));
        for (cs, lat, lon, since) in ghosts {
            if let Some(end) = self.runways.iter().find_map(|r| on_runway(r, lat, lon)) {
                self.claims.push(Claim {
                    who: Claimant::Ghost(cs),
                    end,
                    whole_strip: false,
                    kind: ClaimKind::Observed,
                    held: false,
                    since: Some(since),
                });
            }
        }
    }

    /// Every claim in the book.
    pub(crate) fn claims(&self) -> &[Claim] {
        &self.claims
    }

    fn claims_of(&self, id: FlightId) -> impl Iterator<Item = &Claim> {
        self.claims
            .iter()
            .filter(move |c| c.who == Claimant::Strip(id))
    }

    /// What a flight's CLEARANCE claims of a strip, for the party line, on
    /// the ground or in the air. Cleared to land and not yet reported off,
    /// lined up or cleared for takeoff, the WHOLE strip; sequenced for it in
    /// any other way (taxi, the circuit's Inbound, a go-around's), its
    /// DIRECTION. `None` without a clearance or a runway.
    pub(crate) fn strip_claim(&self, id: FlightId) -> Option<&Claim> {
        self.claims_of(id).find(|c| {
            !c.end.is_empty()
                && matches!(
                    c.kind,
                    ClaimKind::Landing
                        | ClaimKind::LineUp
                        | ClaimKind::Takeoff
                        | ClaimKind::Sequenced
                )
        })
    }

    /// The strip a flight HOLDS for the party line: the one it stands on, or,
    /// on the ground, the one the tower's book still gives it. `None`
    /// airborne or with no state.
    pub(crate) fn blocked_strip(&self, id: FlightId) -> Option<String> {
        let (_, state) = self.states.iter().find(|(f, _)| *f == id)?;
        if !state.on_ground {
            return None;
        }
        self.claims_of(id)
            .find(|c| c.kind == ClaimKind::Occupying)
            .or_else(|| self.claims_of(id).find(|c| c.held && !c.end.is_empty()))
            .map(|c| c.end.clone())
    }

    /// Does the book put this flight on the strip it stands on? Cleared onto
    /// it (any runway named), cleared to land on THIS strip (or on none named)
    /// and not reported off, a runway start, rolling off the strip it landed
    /// on, or a moving cleared crossing.
    pub(crate) fn occupancy_authorised(&self, id: FlightId) -> bool {
        let Some((_, here)) = self.under.iter().find(|(f, _)| *f == id) else {
            return false;
        };
        self.claims_of(id).any(|c| match c.kind {
            ClaimKind::LineUp | ClaimKind::Takeoff => true,
            ClaimKind::Landing => {
                c.end.is_empty()
                    || strip(&self.runways, &c.end).map(|r| r.ident_a.as_str())
                        == Some(here.as_str())
            }
            ClaimKind::RunwayStart | ClaimKind::Vacating | ClaimKind::Crossing => true,
            ClaimKind::CrossingCleared => {
                strip(&self.runways, &c.end).map(|r| r.ident_a.as_str()) == Some(here.as_str())
            }
            _ => false,
        })
    }

    /// Another flight's strip holds the strip `end` is on WHOLE: lined up,
    /// cleared for takeoff, cleared to land and not yet reported off, or
    /// cleared to cross it. Nobody else lines up, departs or lands on it.
    #[cfg_attr(not(test), allow(dead_code))] // the holds ask through strip_held_by_others
    pub(crate) fn held_by_another(&self, id: FlightId, end: &str) -> Option<&Claim> {
        self.held_whole(end, Some(id))
    }

    /// A strip (other than `except`) holds the strip `end` is on WHOLE, or
    /// stands on its pavement ([`ClaimKind::Occupying`]: on the ground, on
    /// the strip, authorised or not). Nobody lines up, departs or lands on a
    /// strip a flight stands on.
    fn blocked(&self, end: &str, except: Option<FlightId>) -> Option<&Claim> {
        self.held_whole(end, except).or_else(|| {
            let ours = strip(&self.runways, end)?;
            self.claims.iter().find(|c| {
                matches!(&c.who, Claimant::Strip(f) if Some(*f) != except)
                    && c.kind == ClaimKind::Occupying
                    && strip(&self.runways, &c.end).is_some_and(|r| std::ptr::eq(r, ours))
            })
        })
    }

    /// A strip (other than `except`) holds the strip `end` is on WHOLE:
    /// lined up, cleared for takeoff, cleared to land, or cleared to cross.
    fn held_whole(&self, end: &str, except: Option<FlightId>) -> Option<&Claim> {
        let ours = strip(&self.runways, end)?;
        self.claims.iter().find(|c| {
            matches!(&c.who, Claimant::Strip(f) if Some(*f) != except)
                && c.whole_strip
                && matches!(
                    c.kind,
                    ClaimKind::Landing
                        | ClaimKind::LineUp
                        | ClaimKind::Takeoff
                        | ClaimKind::CrossingCleared
                )
                && strip(&self.runways, &c.end).is_some_and(|r| std::ptr::eq(r, ours))
        })
    }

    /// What holds a DEPARTURE off the strip `end` is on: [`Self::blocked`],
    /// except that a flight cleared to land and still FLYING is an arrival,
    /// judged by its TIME to its threshold (the soonest one's,
    /// [`DepartureBlock::holds`]), never a blanket occupant, and at ANY
    /// distance: one farther out than [`LANDING_CLAIM_NM`] claims only its
    /// direction, yet 4.5 nm at 160 kt is 101 s, inside a line-up's time.
    /// LEMG 12 (2026-10-13): Air Nostrum cleared to land held our 737 at HW1
    /// "hold position, traffic on the runway" with nothing on it and the
    /// lander well out. An unknown position, a lander on the ground, or any
    /// other whole-strip claim is an occupant as before.
    pub(crate) fn departure_blocked(
        &self,
        end: &str,
        except: Option<FlightId>,
    ) -> Option<DepartureBlock> {
        let ours = strip(&self.runways, end)?;
        let mut lander: Option<Lander> = None;
        for c in self.claims.iter().filter(|c| {
            matches!(&c.who, Claimant::Strip(f) if Some(*f) != except)
                && strip(&self.runways, &c.end).is_some_and(|r| std::ptr::eq(r, ours))
        }) {
            let whole = c.whole_strip
                && matches!(
                    c.kind,
                    ClaimKind::Landing
                        | ClaimKind::LineUp
                        | ClaimKind::Takeoff
                        | ClaimKind::CrossingCleared
                );
            if !whole && !matches!(c.kind, ClaimKind::Occupying | ClaimKind::Landing) {
                continue;
            }
            match self.airborne_lander(c) {
                Some(l) => {
                    let sooner =
                        |a: &Lander, b: &Lander| a.secs.unwrap_or(0.0) < b.secs.unwrap_or(0.0);
                    if lander.as_ref().is_none_or(|k| sooner(&l, k)) {
                        lander = Some(l);
                    }
                }
                None => return Some(DepartureBlock::Occupied),
            }
        }
        lander.map(DepartureBlock::Lander)
    }

    /// A landing claim whose flight is airborne at a known position: how far
    /// and how long from the threshold it was cleared onto
    /// ([`lander_secs`]); its time is unknown once its state is stale.
    fn airborne_lander(&self, c: &Claim) -> Option<Lander> {
        let Claimant::Strip(id) = c.who else {
            return None;
        };
        if c.kind != ClaimKind::Landing {
            return None;
        }
        let (_, state) = self.states.iter().find(|(f, _)| *f == id)?;
        if state.on_ground {
            return None;
        }
        let (lat, lon) = state.position()?;
        let r = strip(&self.runways, &c.end)?;
        let (tlat, tlon) = if r.ident_a == c.end {
            (r.lat_a, r.lon_a)
        } else {
            (r.lat_b, r.lon_b)
        };
        let nm = nav::haversine_nm(lat, lon, tlat, tlon);
        let secs = (state.at.elapsed() < OWN_SEEN_FRESH).then(|| lander_secs(nm, state.gs_kts));
        Some(Lander { nm, secs })
    }

    /// Whether movement on `end` conflicts with this flight's strip claim:
    /// the same strip, and either the whole strip is claimed or the movement
    /// uses the other direction. One strip, one direction (LEMG 2026-09-28).
    pub(crate) fn conflicts(&self, id: FlightId, end: &str) -> bool {
        let Some(c) = self.strip_claim(id) else {
            return false;
        };
        let same = matches!(
            (strip(&self.runways, end), strip(&self.runways, &c.end)),
            (Some(x), Some(y)) if std::ptr::eq(x, y)
        );
        same && (c.whole_strip || end != c.end)
    }
}

/// Where a traffic strip's flight was last seen ([`FlightStrip::seen`]).
pub(super) fn seen(strip: &FlightStrip) -> Option<AircraftState> {
    *strip.seen.lock().unwrap_or_else(|p| p.into_inner())
}

/// How long our own position, as the traffic link noted it
/// ([`note_own_state`]), stands for us in the book.
pub(super) const OWN_SEEN_FRESH: std::time::Duration = std::time::Duration::from_secs(10);

/// A landing clearance holds the whole strip from this close in; farther
/// out, airborne, only its direction.
const LANDING_CLAIM_NM: f64 = 4.0;

/// Our own position as the traffic link last noted it, while fresh.
pub(super) fn own_seen() -> Option<AircraftState> {
    seen(&OWN_STRIP).filter(|s| s.at.elapsed() < OWN_SEEN_FRESH)
}

/// A traffic strip holds the strip `end` is on whole (cleared onto it or to
/// land on it) or stands on its pavement, on these runways: our line-up,
/// takeoff and released holds wait for it as for an occupied runway.
/// Nothing to read (and nothing locked past the map) while no traffic strip
/// is open.
#[cfg_attr(not(test), allow(dead_code))] // the reply path asks per strip
pub(super) fn strip_held_by_traffic(runways: &[nav::Runway], end: &str) -> bool {
    strip_held_by_others(runways, end, &OWN_STRIP)
}

/// [`strip_held_by_traffic`] for any flight's strip `me`: every OTHER strip
/// is a claimant (ours too, for a traffic strip, by its clearance: our
/// position is the caller's picture, never a cell), each traffic strip with
/// the state it was last seen in. For our strip, nothing is read past the
/// map while no traffic strip is open.
pub(super) fn strip_held_by_others(runways: &[nav::Runway], end: &str, me: &FlightStrip) -> bool {
    book_of_others(runways, me).is_some_and(|b| b.blocked(end, None).is_some())
}

/// What another strip's claim means to a DEPARTURE from the strip `end` is
/// on ([`RunwayBook::departure_blocked`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum DepartureBlock {
    /// On the pavement, cleared onto it, or a lander whose position is
    /// unknown or on the ground: "traffic on the runway".
    Occupied,
    /// Only flights cleared to land, still flying: the soonest one.
    Lander(Lander),
}

/// A flight cleared to land and still flying, as a departure reads it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Lander {
    /// To its threshold, nm: what the tower SAYS ("traffic on 4 mile final").
    pub(crate) nm: f64,
    /// To its threshold, seconds ([`lander_secs`]): what the release JUDGES.
    /// `None` when its position is stale.
    pub(crate) secs: Option<f64>,
}

impl DepartureBlock {
    /// Does this hold a departure released now, `lined_up` or still at the
    /// holding point? An occupant always; a lander unless the departure is
    /// off the strip before the lander's go-around watch would send it round
    /// ([`departure_clears_lander`]: [`LINED_UP_ROLL_S`] to roll lined up,
    /// else [`LINE_UP_S`]). The ONE release rule the reply and the held
    /// release share.
    pub(crate) fn holds(&self, lined_up: bool) -> bool {
        match self {
            DepartureBlock::Occupied => true,
            DepartureBlock::Lander(l) => {
                !departure_clears_lander(l.secs, if lined_up { LINED_UP_ROLL_S } else { LINE_UP_S })
            }
        }
    }
}

/// [`strip_held_by_others`] as a departure reads it.
pub(super) fn departure_held_by_others(
    runways: &[nav::Runway],
    end: &str,
    me: &FlightStrip,
) -> Option<DepartureBlock> {
    book_of_others(runways, me).and_then(|b| b.departure_blocked(end, None))
}

/// The book over every strip but `me` (see [`strip_held_by_others`]);
/// `None` for ours while no traffic strip is open.
fn book_of_others(runways: &[nav::Runway], me: &FlightStrip) -> Option<RunwayBook> {
    let traffic = traffic_strips();
    if me.is_own() && traffic.is_empty() {
        return None;
    }
    let mut entries: Vec<BookEntry<'_>> = Vec::new();
    if !me.is_own() {
        entries.push(BookEntry {
            id: FlightId::Own,
            strip: &OWN_STRIP,
            state: own_seen(),
        });
    }
    entries.extend(
        traffic
            .iter()
            .filter(|(_, s)| !std::ptr::eq(&**s, me))
            .map(|(n, s)| BookEntry {
                id: FlightId::Traffic(*n),
                strip: &**s,
                state: seen(s),
            }),
    );
    Some(RunwayBook::compute_on(runways.to_vec(), &entries))
}

/// The strip the PLAYER holds for the party line: the one it stands on, or,
/// on the ground, the one the tower's book still gives it (cleared onto it,
/// or landed and not yet reported off). A ghost landing or departing that
/// strip is not narrated a clearance over our head. The "runway vacated"
/// report frees it, as it frees a real tower's runway, before the taxi
/// request that used to be the only thing that did (Liege 2026-09-22).
pub fn player_blocked_strip(on_ground: bool, lat: f64, lon: f64) -> Option<String> {
    let own = AircraftState::at_position(lat, lon, on_ground, f64::NAN);
    RunwayBook::now(Some(own)).blocked_strip(FlightId::Own)
}

/// What OUR clearance claims of a strip, for the party line:
/// `(end, whole)` ([`RunwayBook::strip_claim`]).
#[cfg_attr(not(test), allow(dead_code))] // the party line asks the book itself
pub(super) fn own_strip_claim() -> Option<(String, bool)> {
    RunwayBook::now(None)
        .strip_claim(FlightId::Own)
        .map(|c| (c.end.clone(), c.whole_strip))
}

/// Does the tower's own book put us on the runway we stand on? The runway
/// incursion watch ("traffic on final, exit the runway") asks this before it
/// shouts: a standing takeoff clearance or line-up; a landing clearance on
/// THIS strip not yet reported vacated (the rollout and the vacate, EGKK 26L
/// 2026-09-27: "exit the runway immediately" from 20 s after touchdown to
/// the vacate, eight times); a runway start; the strip we LANDED on, cleared
/// or not, until we leave it (EGKK-EGBB 15, 2026-09-28: four "exit the
/// runway" during the vacate of an uncleared landing); a crossing on the
/// cleared route that keeps moving. Anything else on a strip with traffic on
/// final is the incursion it is.
pub(crate) fn runway_occupancy_authorised(lat: f64, lon: f64, gs_kts: f64) -> bool {
    let own = AircraftState::at_position(lat, lon, true, gs_kts);
    RunwayBook::now(Some(own)).occupancy_authorised(FlightId::Own)
}

/// The book's own audit: every pair of flights whose claims on ONE strip
/// cannot stand together now, described ("Traffic(1) Takeoff 09 / Traffic(2)
/// Occupying 09"), with our state where the caller holds it. Two flights
/// each holding the whole strip (lined up, cleared for takeoff, cleared to
/// land and not reported off, cleared to cross), or one cleared for takeoff
/// on a strip another stands on; a takeoff beside a landing whose lander
/// still flies is the release's anticipated separation, not a conflict.
/// Empty is a clean book. A read for tests and the
/// traffic harness (a field hour has zero), never a decision.
pub fn runway_conflicts(own: Option<AircraftState>) -> Vec<String> {
    let book = RunwayBook::now(own);
    let whole = |c: &Claim| {
        c.whole_strip
            && matches!(
                c.kind,
                ClaimKind::Landing
                    | ClaimKind::LineUp
                    | ClaimKind::Takeoff
                    | ClaimKind::CrossingCleared
            )
    };
    let same_strip = |a: &Claim, b: &Claim| {
        matches!(
            (strip(&book.runways, &a.end), strip(&book.runways, &b.end)),
            (Some(x), Some(y)) if std::ptr::eq(x, y)
        )
    };
    let mut out = Vec::new();
    let claims = book.claims();
    for (i, a) in claims.iter().enumerate() {
        for b in &claims[i + 1..] {
            let (Claimant::Strip(fa), Claimant::Strip(fb)) = (&a.who, &b.who) else {
                continue;
            };
            if fa == fb || !same_strip(a, b) {
                continue;
            }
            // "Behind the landing traffic, line up": a conditional line-up
            // stands beside the landing it waits for, so long as it is not
            // ON the strip while that landing is still in the air.
            let conditional =
                |x: &Claim, y: &Claim| x.kind == ClaimKind::LineUp && y.kind == ClaimKind::Landing;
            let on_strip = |f: &FlightId| {
                claims
                    .iter()
                    .any(|c| c.who == Claimant::Strip(*f) && c.kind == ClaimKind::Occupying)
            };
            let flying = |f: &FlightId| book.states.iter().any(|(id, s)| id == f && !s.on_ground);
            let lined_up_under = |x: &Claim, fx: &FlightId, y: &Claim, fy: &FlightId| {
                conditional(x, y) && on_strip(fx) && flying(fy)
            };
            // A departure released ahead of a lander still flying: the
            // tower's anticipated separation (`DepartureBlock::holds`, the
            // lander's go-around watch's own time rule). Both on the ground
            // with their claims is a conflict.
            let ahead = |x: &Claim, y: &Claim, fy: &FlightId| {
                x.kind == ClaimKind::Takeoff && y.kind == ClaimKind::Landing && flying(fy)
            };
            let both_whole = whole(a)
                && whole(b)
                && !((conditional(a, b) && !lined_up_under(a, fa, b, fb))
                    || (conditional(b, a) && !lined_up_under(b, fb, a, fa))
                    || ahead(a, b, fb)
                    || ahead(b, a, fa));
            let rolls_into = |x: &Claim, y: &Claim| {
                x.kind == ClaimKind::Takeoff && y.kind == ClaimKind::Occupying
            };
            if both_whole || rolls_into(a, b) || rolls_into(b, a) {
                out.push(format!(
                    "{fa:?} {:?} {} / {fb:?} {:?} {}",
                    a.kind, a.end, b.kind, b.end
                ));
            }
        }
    }
    out
}
