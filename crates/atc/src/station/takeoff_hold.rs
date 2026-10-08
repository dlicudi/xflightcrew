//! The held takeoff clearance and its release.

use super::*;

/// A takeoff clearance withheld for landing traffic — armed by the reply
/// path, released by the monitor once the final has stayed clear.
pub struct TakeoffHold {
    pub(super) callsign: String,
    /// Spoken, for the release line ("zero niner right").
    pub(super) runway: String,
    /// The end ident ("09R"), for the strip-restricted traffic check.
    pub(super) runway_ident: String,
    pub(super) clear_since: Option<Instant>,
    /// When the hold went on — the wake block is measured against this, so a
    /// busy field's departure stream can't extend it forever (see
    /// [`takeoff_hold_tick`]).
    pub(super) armed_at: Instant,
    /// When the "still held" diagnostic last went out.
    pub(super) logged_at: Instant,
    /// Whether the watch has already sequenced this departure onto the
    /// runway with a line-up-and-wait. One shot: the aircraft can only take
    /// the runway once.
    pub(super) lined_up: bool,
    /// The line-up was CONDITIONAL ("behind the landing X, line up behind"):
    /// the pavement is the pilot's only once X has passed. The watch
    /// publishes that condition for the crew every tick.
    pub(super) behind: bool,
    /// The traffic this hold is FOR, by raw callsign: the strip's occupants
    /// at the watch's first tick and every arrival seen on our final since.
    /// Seeded by [`takeoff_hold_tick_from`]; kept across a re-request on
    /// the same runway. `None` until seeded.
    pub(super) held_for: Option<std::collections::HashSet<String>>,
    /// Cut-ins already blackboxed (`hold_cut_in`), once per ghost.
    pub(super) cut_in_logged: std::collections::HashSet<String>,
}

/// How much better another runway's headwind must be before the tower swaps
/// the runway in use. Big enough that two parallels (identical headings,
/// headwinds equal to rounding error) can never trade places on noise.
pub(super) const RUNWAY_SWAP_MARGIN_KTS: f64 = 3.0;

/// Under this the wind does not pick the runway: the pick keeps the runway in
/// use (the into-wind choice flip-flops on noise), the ATIS keeps its
/// edition's runway, and an arrival's stated runway stands.
pub(super) const CALM_WIND_KTS: f64 = 5.0;

/// The tailwind an arrival's (or a taxi request's) stated runway may carry and
/// still stand: ICAO PANS-ATM 7.2.3 accepts a preferential runway with up to
/// 5 kt of tailwind on a dry runway.
pub(super) const STATED_RUNWAY_TAILWIND_KTS: f64 = 5.0;

/// Whether the wind argues with the runway the pilot named. `hw` is its
/// headwind (negative = tailwind), `best` the into-wind pick. It stands in
/// calm; within STATED_RUNWAY_TAILWIND_KTS of tailwind when its crosswind is
/// no worse than the best end's by more than the swap margin (the reciprocal
/// of the best end has the same crosswind; a crossing strip may not); or with
/// a headwind inside the swap margin of the best.
pub(super) fn stated_runway_stands(
    runways: &[nav::Runway],
    ident: &str,
    hw: Option<f64>,
    best: Option<(String, f64, f64)>,
    wind_from: f64,
    wind_kts: f64,
) -> bool {
    runway_stands_within(
        runways,
        ident,
        hw,
        best,
        wind_from,
        wind_kts,
        STATED_RUNWAY_TAILWIND_KTS,
    )
}

/// The tailwind a RUNWAY START's own strip may carry and still be given it:
/// a pilot lined up and asking for the runway under him takes a tailwind
/// within the common transport limit of 10 kt, as a controller with nothing
/// else on the field approves a pilot's request (#178). A stream on the
/// other end still wins.
pub(super) const ON_STRIP_TAILWIND_KTS: f64 = 10.0;

/// [`stated_runway_stands`] with its tailwind limit.
pub(super) fn runway_stands_within(
    runways: &[nav::Runway],
    ident: &str,
    hw: Option<f64>,
    best: Option<(String, f64, f64)>,
    wind_from: f64,
    wind_kts: f64,
    tailwind_kts: f64,
) -> bool {
    let (Some(hw), Some((_, best_hdg, best_hw))) = (hw, best) else {
        return false;
    };
    if wind_kts < CALM_WIND_KTS {
        return true;
    }
    if best_hw - hw <= RUNWAY_SWAP_MARGIN_KTS {
        return true;
    }
    let crosswind =
        |hdg: f64| (wind_kts * sim::angles::signed_diff_deg(wind_from, hdg).to_radians().sin()).abs();
    let stated_hdg = runways.iter().find_map(|r| {
        if r.ident_a == ident {
            Some(r.heading_a_deg())
        } else if r.ident_b == ident {
            Some(r.heading_b_deg())
        } else {
            None
        }
    });
    let Some(stated_hdg) = stated_hdg else {
        return false;
    };
    hw >= -tailwind_kts
        && crosswind(stated_hdg) <= crosswind(best_hdg) + RUNWAY_SWAP_MARGIN_KTS
}

/// The final must stay clear this long before the held clearance releases —
/// an arrival crossing the fence vanishes from the airborne band while still
/// rolling out, and "cleared for takeoff" over a lander is a contradiction.
pub(super) const HOLD_CLEAR_SECS: f64 = 15.0;

/// How far out the nearest arrival may be and still have a departure released
/// ahead of it.
///
/// A real tower THREADS departures into the arrival stream; it does not wait
/// for an empty four-mile final, because at a single-runway field working a
/// landing every couple of minutes that final is never empty. Requiring
/// [`final_traffic_for`]'s whole 4 nm corridor to be clear made the release
/// unreachable at Gatwick: held at holding point Mike for runway 26L with an
/// arrival at half a mile, then eight minutes of stream behind it, and full
/// auto handed the aircraft back (2026-07-27). Three miles is about eighty
/// seconds at approach speed — a light twin is airborne and turning out well
/// inside that, which is the gap controllers actually use. The runway itself
/// being clear is a SEPARATE and absolute condition below.
pub(super) const DEPART_GAP_NM: f64 = 3.0;

/// While a departure is held, log what is holding it at this cadence. "Held
/// for traffic" is undiagnosable after the fact otherwise — the Heathrow hold
/// had to be reconstructed by probing the live sim (2026-07-27).
pub(super) const HOLD_LOG_SECS: f64 = 15.0;

/// One pass of the held-clearance watch: with OUR runway's final clear (no
/// arrival in its corridor, no ghost on the strip — a parallel runway's
/// stream doesn't count) long enough, the tower comes back unprompted —
/// like a real one — and the FSM sync is handed to the main loop.
/// When a takeoff hold's clock started: `now` for a new hold, and the
/// ORIGINAL moment for one already standing on the same runway.
///
/// Asking again does not restart the separation you have already waited out.
/// The wake gate in [`takeoff_hold_tick`] is capped at one full interval at
/// the hold, measured from this stamp — and re-stamping it on every request
/// handed that cap straight back: at a field whose departure interval is
/// shorter than the separation itself, a pilot who asks a second time resets
/// their own clock, hears "wake turbulence, departing traffic ahead of you"
/// again, and never takes off. The clock belongs to the departure ahead of us
/// in sequence, not to our last transmission. A hold on a DIFFERENT runway is
/// a different queue and starts fresh.
pub(super) fn hold_armed_at(existing: Option<&TakeoffHold>, runway_ident: &str, now: Instant) -> Instant {
    existing
        .filter(|h| h.runway_ident == runway_ident)
        .map_or(now, |h| h.armed_at)
}

/// Groundspeed past which our aircraft on the ground is ROLLING for takeoff,
/// never taxiing (taxi paces top out near 30 kt).
pub const HOLD_ROLL_KTS: f64 = 40.0;

/// Retire a held release once our aircraft rolls on the ground past
/// [`HOLD_ROLL_KTS`]: the takeoff it was holding has happened (through the
/// cancel, or before it was held). The reject call retires it too; this is
/// the tower's own eyes for a reject it never heard. `true` when a hold was
/// voided.
pub fn void_takeoff_hold_by_roll(gs_kts: f64) -> bool {
    if gs_kts < HOLD_ROLL_KTS {
        return false;
    }
    let voided = OWN_STRIP.takeoff_hold
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .take()
        .is_some();
    if voided {
        *OWN_STRIP.line_up_condition.lock().unwrap_or_else(|e| e.into_inner()) = None;
        tracing::info!(gs_kts, "held takeoff release voided: the aircraft rolled");
    }
    voided
}

/// [`takeoff_hold_tick_from`] without our own position: every occupant of
/// the pavement holds the departure.
#[cfg_attr(not(test), allow(dead_code))] // station tests only
pub fn takeoff_hold_tick(
    targets: &[TcasTarget],
    field_elev_ft: f64,
    player_on_ground: bool,
    now: Instant,
) -> Option<String> {
    takeoff_hold_tick_from(targets, field_elev_ft, player_on_ground, None, now)
}

/// A ghost more than this behind our reference point, on the departure
/// end's heading, is behind our tail: it is not in the way of our roll.
pub(super) const OWN_TAIL_M: f64 = 40.0;

/// A ghost on our strip standing this long is standing: a lined-up or
/// parked replay, not a roll or a rollout.
pub(super) const CUT_IN_STANDING_S: f64 = 10.0;

/// What a departure sees on its own strip.
#[derive(Debug, Default, PartialEq)]
pub(super) struct StripPicture {
    /// Something on the pavement holds the departure.
    pub(super) blocked: bool,
    /// The occupants ahead of our tail, raw callsigns (the watch's seed).
    occupants: Vec<String>,
    /// Ghosts standing on the strip that the hold was not held for: a
    /// replay's departure queue lining up in front of an aircraft the tower
    /// gave the runway to. The party line withholds their clearance
    /// (`party_line_withheld`); the hold is not held for them.
    cut_in: Vec<String>,
}

/// A departure's traffic on its own strip: a ghost that OCCUPIES the
/// pavement ([`occupies`]: on it, and not stale data) and, when we stand on
/// that pavement ourselves (`own`), is not behind our tail on the departure
/// end's heading. With `held_for` (a hold issued to an aircraft ON the
/// strip), a ghost the hold was not held for holds it only while it moves
/// (a roll leaving, a rollout); standing, it is a cut-in. The same pavement
/// and stale-ghost rules as the landing judgement (#162): the incursion
/// watch's wider capture, and a replay queue of 13 departures lining up in
/// front of it, held a 737 lined up on LEMG 13 for five minutes after the
/// traffic it was promised a clearance behind had landed (sortie
/// 1790623420-full_auto, 2026-09-28, issue #172).
pub(super) fn departure_strip_picture(
    r: &nav::Runway,
    departure_end: &str,
    targets: &[TcasTarget],
    own: Option<(f64, f64)>,
    held_for: Option<&std::collections::HashSet<String>>,
    now: Instant,
) -> StripPicture {
    let frame = LandingFrame::new(r, departure_end);
    let own_along = match (&frame, own) {
        (Some(f), Some((lat, lon))) if on_runway_pavement(r, lat, lon).is_some() => {
            Some(f.place(lat, lon).0)
        }
        _ => None,
    };
    let mut out = StripPicture::default();
    for t in targets {
        if occupies(r, t, now).is_none() {
            continue;
        }
        if let (Some(f), Some(ours)) = (&frame, own_along) {
            if f.place(t.lat, t.lon).0 < ours - OWN_TAIL_M {
                continue;
            }
        }
        out.occupants.push(t.callsign.clone());
        let cut_in = held_for.is_some_and(|set| {
            !set.contains(&t.callsign) && standing_secs(&t.callsign, now) >= CUT_IN_STANDING_S
        });
        if cut_in {
            out.cut_in.push(t.callsign.clone());
        } else {
            out.blocked = true;
        }
    }
    out
}

/// [`departure_strip_picture`]'s verdict for the tower's reply: the hold
/// standing on this runway (if any) says who it was held for.
/// Asked for the flight whose strip `flight` is (ours, or a traffic flight's
/// answered on its own desk).
pub(super) fn departure_strip_occupied_for(
    flight: &FlightStrip,
    r: &nav::Runway,
    departure_end: &str,
    targets: &[TcasTarget],
    own: Option<(f64, f64)>,
    now: Instant,
) -> bool {
    let held_for = flight.takeoff_hold
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|h| h.lined_up && h.runway_ident == departure_end)
        .and_then(|h| h.held_for.clone());
    departure_strip_picture(r, departure_end, targets, own, held_for.as_ref(), now).blocked
}

/// One pass of the held-clearance watch with our own position (`own`, for
/// the occupant behind our tail; see [`departure_strip_occupied_for`]).
pub fn takeoff_hold_tick_from(
    targets: &[TcasTarget],
    field_elev_ft: f64,
    player_on_ground: bool,
    own: Option<(f64, f64)>,
    now: Instant,
) -> Option<String> {
    let (lat, lon) = own.unwrap_or((f64::NAN, f64::NAN));
    let state = AircraftState::at_position(lat, lon, player_on_ground, f64::NAN);
    takeoff_hold_tick_for(&OWN_STRIP, &state, targets, field_elev_ft, now)
}

/// [`takeoff_hold_tick_from`] for one flight's strip: its held release, its
/// position on the pavement, its clearance; every other strip a claimant
/// in the book.
pub(super) fn takeoff_hold_tick_for(
    flight: &FlightStrip,
    state: &AircraftState,
    targets: &[TcasTarget],
    field_elev_ft: f64,
    now: Instant,
) -> Option<String> {
    if station_unavailable() {
        return None;
    }
    let own = state.position();
    // The standing clocks the stale-ghost rule reads (idempotent per tick).
    note_ground_traffic(targets, now);
    // The book: another strip cleared onto the held runway (or to land on
    // it) holds it like an occupant, whatever the replay shows. Asked before
    // the hold's lock (for ours, nothing to ask with no traffic strip open).
    let other_strip_holds = (!flight.is_own() || !traffic_strips().is_empty()) && {
        let held_ident = flight
            .takeoff_hold
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map(|h| h.runway_ident.clone());
        held_ident
            .is_some_and(|ident| strip_held_by_others(&active_runways(), &ident, flight))
    };
    let player_on_ground = state.on_ground;
    let mut guard = flight.takeoff_hold.lock().unwrap_or_else(|e| e.into_inner());
    let hold = guard.as_mut()?;
    // Airborne, the hold is moot: either the pilot departed through the
    // cancel (the compliance watch owns that conversation) or the whole
    // thing is stale. "Traffic clear, cleared for takeoff" spoken to an
    // aircraft two minutes into its climb (Gatwick, 2026-07-13) helps
    // nobody — void it, silently.
    if !player_on_ground {
        *guard = None;
        *flight.line_up_condition.lock().unwrap_or_else(|e| e.into_inner()) = None;
        return None;
    }
    // Our strip, cloned out so the runway lock isn't held across the checks.
    let (rwy, runways) = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        (strip(&runways, &hold.runway_ident).cloned(), runways.clone())
    };
    // The two conditions are DIFFERENT in kind, and conflating them is what
    // made the release unreachable under injected traffic: the runway being
    // occupied is absolute, while an arrival on final is a question of GAP.
    let inbound = rwy
        .as_ref()
        .and_then(|r| final_traffic_for(targets, r, &runways, field_elev_ft));
    // The traffic this hold is for: the strip's occupants at its first
    // tick, and every arrival on our final since.
    if hold.held_for.is_none() {
        let seed = rwy.as_ref().map(|r| {
            departure_strip_picture(r, &hold.runway_ident, targets, own, None, now).occupants
        });
        hold.held_for = Some(seed.unwrap_or_default().into_iter().collect());
    }
    if let (Some(set), Some(f)) = (hold.held_for.as_mut(), inbound.as_ref()) {
        set.extend(f.ghosts.iter().cloned());
    }
    // Held ON the strip, the runway is ours: a ghost lining up in front of
    // us is the replay's queue, not the traffic we were held for.
    let held_for = if hold.lined_up {
        hold.held_for.clone()
    } else {
        None
    };
    let picture = rwy.as_ref().map(|r| {
        departure_strip_picture(r, &hold.runway_ident, targets, own, held_for.as_ref(), now)
    });
    for g in picture.iter().flat_map(|p| p.cut_in.iter()) {
        if hold.cut_in_logged.insert(g.clone()) {
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "hold_cut_in", "ghost": g,
                    "runway": hold.runway_ident}),
            );
        }
    }
    let on_strip = match &picture {
        Some(p) => p.blocked,
        None => targets
            .iter()
            .any(|t| t.on_ground && on_active_runway(t.lat, t.lon).is_some()),
    } || other_strip_holds;
    // A CONDITIONAL line-up's condition, for the crew: the named arrival has
    // passed once it is off the final (down on the strip, or gone). Published
    // every tick, cleared with the hold. The Baron at Malaga crossed the hold
    // line five seconds after "behind the landing Bee-line, line up behind"
    // with the lander at 2.4 nm, and only its 85 s crawl kept it off the
    // pavement until 15 s after the touchdown (12:21Z, 2026-09-11).
    *flight.line_up_condition.lock().unwrap_or_else(|e| e.into_inner()) =
        hold.behind.then(|| inbound.is_none());
    let traffic = match &rwy {
        Some(_) => on_strip || inbound.as_ref().is_some_and(|f| f.nm < DEPART_GAP_NM),
        // No geometry for the held runway: fall back to the blunt any-runway
        // checks rather than clearing blind.
        None => {
            on_strip
                || targets
                    .iter()
                    .any(|t| !t.on_ground && (30.0..1500.0).contains(&(t.ele_ft - field_elev_ft)))
        }
    };
    // The wake clock blocks the release exactly like traffic does: a held
    // clearance behind a departed heavy waits out the two minutes. But it is
    // measured from when the HOLD went on, not from the last liftoff —
    // separation is owed behind the departure ahead of you IN SEQUENCE, and
    // a ghost that took your slot while you sat at the taxiway restarts
    // nothing you owe. Un-capped, a field whose departure interval is
    // shorter than the separation itself (Heathrow's 27L: a liftoff about
    // every two minutes) re-armed the clock faster than it could expire, the
    // release condition became unreachable, and full auto sat out its entire
    // budget at holding point Sierra before handing the aircraft back
    // (2026-07-27). One full interval at the hold IS the separation.
    let wake = now.duration_since(hold.armed_at).as_secs() < WAKE_SECS
        && wake_remaining(&hold.runway_ident, now).is_some();
    if traffic || wake {
        hold.clear_since = None;
        // What is holding us, at a cadence — a held departure that says
        // nothing for eight minutes can only be diagnosed by probing the
        // live sim, which is how the Heathrow hold had to be found.
        // On the injected clock like the rest of the tick, so tests that
        // drive `now` see the same cadence the sim does (duration_since
        // saturates to zero if `now` sits behind the stamp).
        if now.duration_since(hold.logged_at).as_secs_f64() >= HOLD_LOG_SECS {
            hold.logged_at = now;
            tracing::info!(
                runway = %hold.runway_ident,
                final_nm = ?inbound.as_ref().map(|f| (f.nm * 10.0).round() / 10.0),
                arrivals = inbound.as_ref().map(|f| f.count).unwrap_or(0),
                on_strip,
                wake,
                lined_up = hold.lined_up,
                "departure still held"
            );
        }
        // The strip is clear and the blocker is only a CLOCK or a gap that
        // hasn't opened yet: sequence the departure ONTO the runway instead
        // of leaving it at the taxiway while the stream rolls past. Line up
        // and wait is what a real tower gives here, and it is what makes the
        // release meaningful — an aircraft still behind the hold line is not
        // next for anything. Never onto an OCCUPIED runway, which is why
        // every branch sits under `!on_strip`.
        //
        // Behind an ARRIVAL the same rule the tower's own conditional uses
        // applies: one aircraft, far enough out (`CONDITIONAL_MIN_NM`). A
        // lander at half a mile earns the hold, not the pavement.
        let staging = if !hold.lined_up && !on_strip {
            if wake {
                Some("wake turbulence, departing traffic ahead of you")
            } else if inbound
                .as_ref()
                .is_some_and(|f| f.count <= 1 && f.nm >= tower::CONDITIONAL_MIN_NM)
            {
                Some("landing traffic ahead of you")
            } else {
                None
            }
        } else {
            None
        };
        if let Some(reason) = staging {
            hold.lined_up = true;
            let line = format!(
                "{}, line up and wait runway {}, {reason}.",
                hold.callsign, hold.runway
            );
            if let Some(c) = flight.clearance.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
                c.state = tower::State::LinedUp;
            }
            *flight.pending_line_up.lock().unwrap_or_else(|e| e.into_inner()) = true;
            return Some(line);
        }
        return None;
    }
    let since = *hold.clear_since.get_or_insert(now);
    if now.duration_since(since).as_secs_f64() < HOLD_CLEAR_SECS {
        return None;
    }
    let line = format!(
        "{}, traffic clear. Runway {}, cleared for takeoff.",
        hold.callsign, hold.runway
    );
    if let Some(c) = flight.clearance.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
        c.state = tower::State::TakeoffCleared;
    }
    *flight.pending_takeoff_clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = true;
    *guard = None;
    // Released: the clearance stands on its own, no condition left to wait on.
    *flight.line_up_condition.lock().unwrap_or_else(|e| e.into_inner()) = None;
    Some(line)
}
