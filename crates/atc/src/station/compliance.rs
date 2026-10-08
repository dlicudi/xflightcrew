//! The compliance watch: clearance picture, incursions, the zone.

use super::*;

/// Two-way contact plus the clearance picture, published by the reply path
/// after every answered call, for the monitor's compliance watch. `None`
/// until the pilot's first call to this station (and again after a swap) —
/// no contact means the tower doesn't know you, and the zone watch takes
/// over instead.
pub struct ClearanceSnap {
    pub state: tower::State,
    pub callsign: String,
    /// The runway END in use when this was published (e.g. "09R") — conflict
    /// watches restrict themselves to that strip; a parallel runway's
    /// traffic is somebody else's problem. Empty when no runway is known.
    pub runway_ident: String,
    /// The exact readback the pilot owes right now, phrased — the coach's
    /// first suggestion. `None` = no readback outstanding.
    pub pending_readback: Option<String>,
    /// The tower's book says OUR aircraft holds `runway_ident`: cleared onto
    /// it, or landed on it and not yet reported off ([`tower::Tower::holds_runway`]).
    /// The "runway vacated" report clears it, before the taxi request.
    pub holds_runway: bool,
}

/// A compliance or zone call: what to say, and whether the COPILOT says it
/// (not tuned to the station — a controller can't berate a radio that isn't
/// listening; the crew flags it instead, like a real FO would).
pub struct ComplianceCall {
    pub line: String,
    pub crew: bool,
}

/// The compliance watch's memory of ONE flight between ticks: a cell of its
/// strip (`watch`, reset by `install` and `uninstall` as the field fact it
/// was split from), so each aircraft's ground and air history is its own.
#[derive(Default)]
pub(crate) struct ComplianceFlight {
    pub(super) prev_on_ground: Option<bool>,
    /// Continuously airborne since: a landing accusation needs a FLIGHT
    /// first. A one-sample unstick during the takeoff roll (0.3 ft AGL for
    /// one tick at 83 knots, Jerez 2026-07-11) read as air-then-ground and
    /// earned "you landed without clearance" mid-departure.
    pub(super) airborne_since: Option<Instant>,
    /// When the tower state was last seen at ClearedToLand: grace for the
    /// rollout and vacating after a legal landing.
    pub(super) cleared_to_land_seen: Option<Instant>,
    /// Each transition violation is called once, not every tick.
    pub(super) takeoff_berated: bool,
    pub(super) landing_berated: bool,
    /// The strip (its `ident_a`) the aircraft stood on at the last ground
    /// tick, `None` off every runway, airborne or before the first tick.
    pub(super) prev_strip: Option<String>,
    /// A RUNWAY START: the strip the aircraft first appeared on, stopped,
    /// without being seen to taxi onto it (the sim put it there). It is a
    /// lined-up aircraft awaiting its clearance, never an incursion, for as
    /// long as it stays on that strip (EGKK 26L, 2026-09-27).
    pub(super) runway_start: Option<String>,
    /// The strip (its `ident_a`) the aircraft LANDED on, cleared or not,
    /// for as long as it stays on it: the rollout and the vacate are the only
    /// way off, never an incursion (EGKK-EGBB 15, sortie
    /// 1790601822-full_auto, 2026-09-28: landed uncleared, then "exit the
    /// runway" four times and "you are on runway one five without clearance"
    /// during the vacate the tower had just ordered with "taxi to parking").
    pub(super) landed_strip: Option<String>,
    /// Our last LIFTOFF: the strip (its `ident_a`) we rolled on and when.
    /// The party line clears no departure off that strip behind us inside
    /// [`WAKE_SECS`] (EGKK 08R, 2026-09-28: "Easy eight six five three,
    /// cleared for takeoff" 1.6 s after our liftoff).
    pub(super) liftoff: Option<(String, Instant)>,
}

impl ComplianceFlight {
    pub(crate) const NEW: ComplianceFlight = ComplianceFlight {
        prev_on_ground: None,
        airborne_since: None,
        cleared_to_land_seen: None,
        takeoff_berated: false,
        landing_berated: false,
        prev_strip: None,
        runway_start: None,
        landed_strip: None,
        liftoff: None,
    };
}

/// The compliance watch's radio cooldowns, the FIELD's: one frequency, one
/// thing at a time on it, whichever aircraft the call is about.
pub(super) struct ComplianceState {
    /// Last violation call of any kind: one thing at a time on frequency.
    pub(super) last_call: Option<Instant>,
    pub(super) zone_last: Option<Instant>,
    /// Last off-route taxi challenge: its own cooldown; wandering is a
    /// nudge, not a violation.
    pub(super) off_route_last: Option<Instant>,
}

impl ComplianceState {
    pub(super) const NEW: ComplianceState = ComplianceState {
        last_call: None,
        zone_last: None,
        off_route_last: None,
    };
}

pub(super) static COMPLIANCE: Mutex<ComplianceState> = Mutex::new(ComplianceState::NEW);

/// Between repeat violation calls.
pub(super) const VIOLATION_COOLDOWN_SECS: f64 = 60.0;
/// Rollout/vacating time after a landing clearance before "on the runway
/// without clearance" can fire.
pub(super) const VACATE_GRACE_SECS: f64 = 120.0;
/// Below this a first sight of the aircraft on a strip is a standing start
/// (the sim's runway start reads 0.0 kt), never a taxi onto it.
pub(super) const RUNWAY_START_MAX_KTS: f64 = 1.0;
/// The pseudo control zone around the active towered field: apt.dat has no
/// airspace geometry, so a cylinder stands in for the CTR.
pub(super) const ZONE_RADIUS_NM: f64 = 5.0;
pub(super) const ZONE_HEIGHT_FT: f64 = 3000.0;
pub(super) const ZONE_COOLDOWN_SECS: f64 = 120.0;
/// Rolling out or vacating the strip it landed on, an aircraft at this pace or
/// more is on its way off (the recorded EGBB 15 vacate: 3.8 to 12 kt).
/// Stopped on it, it is squatting, cleared landing or not.
pub(super) const LANDED_VACATING_KTS: f64 = 3.0;
/// Continuous air time before ground contact can be called a LANDING —
/// shorter is a skip on the takeoff roll or a bounce, not an arrival.
pub(super) const MIN_FLIGHT_SECS: f64 = 30.0;

/// The flight stands on the strip its strip's `crossing` cell clears it to
/// cross ([`FlightStrip::crossing`]). The cell is locked alone and dropped
/// before the runways are read.
fn cleared_to_cross(flight: &FlightStrip, lat: f64, lon: f64) -> bool {
    let end = flight
        .crossing
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| c.0.clone());
    end.is_some_and(|end| {
        strip(&active_runways(), &end).is_some_and(|r| on_runway(r, lat, lon).is_some())
    })
}

/// One tick of the compliance watch: the tower holds the pilot to the
/// clearances it actually issued, and challenges zone traffic it has never
/// talked to. Everything is derived from the published `OWN_STRIP.clearance` state:
/// a tower that never saw you on its ground can't accuse you of an uncleared
/// takeoff.
#[cfg_attr(not(test), allow(dead_code))] // station tests only
pub fn compliance_tick(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    on_ground: bool,
    gs_kts: f64,
    tuned: bool,
    now: Instant,
) -> Option<ComplianceCall> {
    compliance_tick_facing(lat, lon, alt_ft, on_ground, gs_kts, tuned, now, None)
}

/// [`compliance_tick`] with the aircraft's true heading, so a runway it is
/// named on is the end it is pointing down ([`on_active_runway_facing`]).
#[allow(clippy::too_many_arguments)]
pub fn compliance_tick_facing(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    on_ground: bool,
    gs_kts: f64,
    tuned: bool,
    now: Instant,
    heading_true: Option<f64>,
) -> Option<ComplianceCall> {
    let state = AircraftState {
        alt_ft,
        psi_deg: heading_true,
        ..AircraftState::at_position(lat, lon, on_ground, gs_kts)
    };
    compliance_tick_for(&OWN_STRIP, &state, tuned, now)
}

/// [`compliance_tick_facing`] for one flight's strip: its clearance, its
/// route, its coach and challenge cells. A position that did not read is no
/// position, as before (no strip under it, no distance).
pub(super) fn compliance_tick_for(
    flight: &FlightStrip,
    state: &AircraftState,
    tuned: bool,
    now: Instant,
) -> Option<ComplianceCall> {
    let (lat, lon) = state.position().unwrap_or((f64::NAN, f64::NAN));
    let (alt_ft, on_ground, gs_kts, heading_true) =
        (state.alt_ft, state.on_ground, state.gs_kts, state.psi_deg);
    if station_unavailable() {
        return None;
    }
    let (field_lat, field_lon) = {
        let f = ACTIVE_FIELD.lock().unwrap_or_else(|e| e.into_inner());
        let (_, la, lo) = f.as_ref()?;
        (*la, *lo)
    };
    let dist_nm = crate::nav::haversine_nm(lat, lon, field_lat, field_lon);
    let clr: Option<(tower::State, String)> = flight.clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.callsign.clone()));

    // This flight's memory, then the field's radio cooldowns (always in this
    // order; nothing else holds both).
    let mut f = flight.compliance_flight.lock().unwrap_or_else(|e| e.into_inner());
    let mut w = COMPLIANCE.lock().unwrap_or_else(|e| e.into_inner());
    let prev_ground = f.prev_on_ground.replace(on_ground);
    // How long the flight before THIS ground contact lasted (0.0 while the
    // wheels never left) — read before the touchdown resets the clock.
    let airborne_secs = f
        .airborne_since
        .map_or(0.0, |t| now.duration_since(t).as_secs_f64());
    if on_ground {
        f.airborne_since = None;
        // Down: the landing-clearance coach is moot, and a stale handoff must
        // not nag on the next takeoff.
        *flight.landing_handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
    } else if f.airborne_since.is_none() {
        f.airborne_since = Some(now);
    }
    // A runway start: first seen on this strip, and stopped. A taxi onto a
    // runway arrives moving, a landing arrives airborne; only the sim's own
    // placement (boot, a reposition, the first tick of a fresh station)
    // shows a stopped aircraft on a strip it was not on a tick ago.
    let here = if on_ground { strip_under(lat, lon) } else { None };
    if here.is_none() || f.runway_start != here {
        f.runway_start = None;
    }
    if here.is_some() && here != f.prev_strip && gs_kts < RUNWAY_START_MAX_KTS {
        f.runway_start = here.clone();
    }
    // Our liftoff, off the strip we rolled on (see `liftoff`).
    if prev_ground == Some(true) && !on_ground {
        if let Some(strip) = f.prev_strip.clone() {
            f.liftoff = Some((strip, now));
        }
    }
    // A landing's strip is ours until we leave it (see `landed_strip`).
    if !on_ground {
        f.landed_strip = None;
    } else if prev_ground == Some(false) && airborne_secs >= MIN_FLIGHT_SECS {
        f.landed_strip = here.clone();
    } else if f.landed_strip.is_some() && here != f.landed_strip {
        f.landed_strip = None;
    }
    let landed_here =
        here.is_some() && f.landed_strip == here && gs_kts >= LANDED_VACATING_KTS;
    f.prev_strip = here;
    let runway_start = f.runway_start.is_some();
    if clr
        .as_ref()
        .is_some_and(|(s, _)| *s == tower::State::ClearedToLand)
    {
        f.cleared_to_land_seen = Some(now);
        // Cleared to land — the coach got what it was waiting for.
        *flight.landing_handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }
    let vacating = f
        .cleared_to_land_seen
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < VACATE_GRACE_SECS);
    let quiet = w
        .last_call
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < VIOLATION_COOLDOWN_SECS);

    if let Some((state, callsign)) = &clr {
        // Landing without clearance: airborne last tick, on the active runway
        // now, and the tower never said "cleared to land". The classic ends
        // with a phone number, exactly once.
        if prev_ground == Some(false)
            && on_ground
            && airborne_secs >= MIN_FLIGHT_SECS
            && !f.landing_berated
            && *state != tower::State::ClearedToLand
        {
            if let Some(rwy) = on_active_runway_facing(lat, lon, heading_true) {
                f.landing_berated = true;
                w.last_call = Some(now);
                let rwy = atis::spoken_runway(&rwy);
                return Some(if tuned {
                    ComplianceCall {
                        line: format!(
                            "{callsign}, you landed on runway {rwy} without clearance. \
                             Taxi to parking. I have a number for you to copy when ready."
                        ),
                        crew: false,
                    }
                } else {
                    ComplianceCall {
                        line: "That was a landing without clearance. Expect a phone number."
                            .to_string(),
                        crew: true,
                    }
                });
            }
        }
        // Takeoff without clearance: ground last tick, airborne now, near the
        // field, and the tower never cleared it. Once.
        if prev_ground == Some(true)
            && !on_ground
            && dist_nm < 5.0
            && !f.takeoff_berated
            && !matches!(
                state,
                tower::State::TakeoffCleared | tower::State::ClearedToLand
            )
        {
            f.takeoff_berated = true;
            w.last_call = Some(now);
            return Some(if tuned {
                ComplianceCall {
                    line: format!("{callsign}, you were not cleared for takeoff. Say intentions."),
                    crew: false,
                }
            } else {
                ComplianceCall {
                    line: "We just took off without a clearance.".to_string(),
                    crew: true,
                }
            });
        }
        // Off the cleared taxi route: taxiing (moving, on the ground, taxi
        // clearance current), a route was issued, and the aircraft has
        // wandered off its polyline. A nudge with the via line, on its own
        // cooldown — controllers redirect strays, they don't phone them in.
        if on_ground
            && (5.0..40.0).contains(&gs_kts)
            && *state == tower::State::TaxiCleared
            && !w
                .off_route_last
                .is_some_and(|t| now.duration_since(t).as_secs_f64() < 45.0)
            && on_active_runway(lat, lon).is_none()
        {
            let stray = flight.taxi_route
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .as_ref()
                .and_then(|r| {
                    let d = dist_to_polyline_m(&r.path, lat, lon)?;
                    (d > 60.0).then(|| (r.via.clone(), r.path.last().copied()))
                });
            if let Some((via, goal)) = stray {
                w.off_route_last = Some(now);
                if !tuned {
                    return Some(ComplianceCall {
                        line: "We're off the cleared taxi route.".to_string(),
                        crew: true,
                    });
                }
                // A stray gets redirected from PRESENT POSITION: replan to
                // the same holding point and re-clear, instead of repeating
                // a via line for pavement the pilot left behind (the same
                // stale via spoken seven times across one Malaga taxi while
                // the pilot was 680 m off it, 2026-07-13). The republished
                // route also re-aims the taxi guide.
                let fresh = goal.and_then(|(glat, glon)| {
                    let net = TAXI_NET.lock().unwrap_or_else(|e| e.into_inner()).clone()?;
                    // The goal is the standing route's (already-trimmed)
                    // endpoint, never a runway junction — the standoff is
                    // moot, so the default rides.
                    net.plan(lat, lon, glat, glon, nav::HOLD_SHORT_STANDOFF_M, &[])
                        .filter(|r| r.path.len() >= 2)
                });
                let line = if let Some(r) = fresh {
                    let via = spoken_via(&r.via).join(", ");
                    let hp = spoken_taxiway(&r.holding_point);
                    *flight.taxi_route.lock().unwrap_or_else(|e| e.into_inner()) = Some(TaxiWatch {
                        path: r.path.clone(),
                        via: via.clone(),
                        seg: r.seg.clone(),
                        holding_point: r.holding_point.clone(),
                        lead_on: r.lead_on.clone(),
                    });
                    format!(
                        "{callsign}, you appear to be off the cleared taxi route. \
                         Taxi to holding point {hp} via {via}."
                    )
                } else if via.is_empty() {
                    format!(
                        "{callsign}, you appear to be off the cleared taxi route. \
                         Continue with caution to the holding point."
                    )
                } else {
                    format!(
                        "{callsign}, you appear to be off the cleared taxi route. \
                         Taxi via {via}."
                    )
                };
                return Some(ComplianceCall { line, crew: false });
            }
        }
        // A cleared ARRIVAL taxi-in to a stand authorises being on the runway
        // just landed on and following the published route to parking — the
        // taxi-in sets the FSM to `Parked` (not `TaxiCleared`) and ends the
        // vacate grace early, so without this the berate fired ~2 min after
        // "taxi to stand" at a crew still clearing the landing runway on its own
        // cleared route. It's the arrival mirror of the departure `TaxiCleared`
        // exemption below; off-route deviations are still caught by the
        // route-deviation check above.
        // The flag only counts WITH the route it was published alongside,
        // matching the crew-side consumers: the exemption below silences the
        // incursion berate outright, and a bare flag that survived a swap or
        // a route-less clearance would switch that safety net off with
        // nothing to show the occupancy is part of a cleared drive.
        let arrival_taxi = *flight.taxi_to_stand.lock().unwrap_or_else(|e| e.into_inner())
            && flight.taxi_route
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .is_some();
        // On the active runway without any clearance to be there: not rolling
        // (a takeoff/landing roll is covered above), not vacating after a
        // legal landing, and the state says holding-point at best. Repeats on
        // the violation cooldown while it persists — runway occupancy is the
        // one the tower keeps shouting about.
        if on_ground
            && gs_kts < 30.0
            && !vacating
            && !quiet
            && !runway_start
            && !landed_here
            && !matches!(
                state,
                // LinedUp is the tower's OWN instruction to be on the runway.
                tower::State::LinedUp | tower::State::TakeoffCleared | tower::State::ClearedToLand
            )
        {
            if let Some(rwy) = on_active_runway_facing(lat, lon, heading_true) {
                // The tower's OWN taxi route may cross (or backtrack) a
                // runway — apt.dat networks do it all the time, and the
                // clearance speaks the crossing ("cross runway two seven").
                // On a runway-flagged leg of the cleared route the occupancy
                // is LEGAL: silence while it keeps moving, and a stopped
                // crossing earns "expedite" — never the berate (the tower
                // shouted "you are on runway two seven without clearance" at
                // a pilot following its own route, 2026-07-14). Holds for a
                // DEPARTURE (state TaxiCleared) and an ARRIVAL taxi-in alike.
                let cleared_taxi = *state == tower::State::TaxiCleared || arrival_taxi;
                // A crossing the tower cleared on request ("cross runway one
                // three", the strip's `crossing`) is the same legal
                // occupancy, wherever on the strip's pavement the flight is:
                // a corner cut at the pavement's edge left the route's
                // nearest point just off it (EFKU 15, the world sweep).
                let cleared_across = cleared_to_cross(flight, lat, lon);
                if cleared_across
                    || (cleared_taxi && on_cleared_runway_crossing_for(flight, lat, lon))
                {
                    if gs_kts < 3.0 {
                        w.last_call = Some(now);
                        // Crossing, the heading is square to the strip and
                        // names neither end: the nearer one it is.
                        let rwy = atis::spoken_runway(&on_active_runway(lat, lon).unwrap_or(rwy));
                        return Some(if tuned {
                            ComplianceCall {
                                line: format!("{callsign}, expedite crossing runway {rwy}."),
                                crew: false,
                            }
                        } else {
                            ComplianceCall {
                                line: format!(
                                    "We've stopped on runway {rwy}. Keep it moving across."
                                ),
                                crew: true,
                            }
                        });
                    }
                    return None;
                }
                // An arrival vacating the runway it just landed on is NOT on a
                // route "crossing" (the runway is the start of the taxi-in, not a
                // leg to cross), so the check above misses it — but vacating under
                // a taxi-in clearance can never be an incursion. Stay silent.
                if arrival_taxi {
                    return None;
                }
                w.last_call = Some(now);
                let rwy = atis::spoken_runway(&rwy);
                return Some(if tuned {
                    ComplianceCall {
                        line: format!(
                            "{callsign}, you are on runway {rwy} without clearance. \
                             Exit the runway immediately."
                        ),
                        crew: false,
                    }
                } else {
                    ComplianceCall {
                        line: format!("We're on runway {rwy} without clearance. We should exit."),
                        crew: true,
                    }
                });
            }
        }
        return None;
    }

    // No two-way contact at all: the zone watch. Inside the pseudo-CTR
    // (cylinder: 5 nm, surface to 3,000 ft above the field), airborne, and
    // the tower has never heard from you — a real controller challenges,
    // a real FO squirms.
    if !on_ground && dist_nm < ZONE_RADIUS_NM {
        let (zone_name, elev_ft, tower_khz) = {
            let p = ACTIVE_PANEL.lock().unwrap_or_else(|e| e.into_inner());
            let (name, elev, svcs) = p.as_ref()?;
            let khz = svcs.iter().find(|(w, _)| w == "tower").map(|(_, k)| *k);
            (spoken_airport(name).to_string(), *elev, khz)
        };
        if alt_ft < elev_ft + ZONE_HEIGHT_FT
            && !w
                .zone_last
                .is_some_and(|t| now.duration_since(t).as_secs_f64() < ZONE_COOLDOWN_SECS)
        {
            w.zone_last = Some(now);
            return Some(if tuned {
                // The crew hears the question as data too: an inbound call is
                // the answer when this is the destination's tower.
                *flight.zone_challenge.lock().unwrap_or_else(|e| e.into_inner()) = true;
                ComplianceCall {
                    line: format!(
                        "Aircraft in the {zone_name} control zone, you are inside \
                         controlled airspace without clearance. Say callsign and intentions."
                    ),
                    crew: false,
                }
            } else {
                ComplianceCall {
                    line: match tower_khz {
                        Some(k) => format!(
                            "We're inside the {zone_name} zone without clearance. \
                             Tower is on {}.",
                            atis::spoken_mhz(k)
                        ),
                        None => format!("We're inside the {zone_name} zone without clearance."),
                    },
                    crew: true,
                }
            });
        }
    }
    None
}
