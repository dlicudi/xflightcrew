//! The party line: ghosts' radio chatter and our strip claim.

use super::*;

/// What the party line remembers about a ghost between passes.
pub(super) struct GhostMemory {
    last_seen: Instant,
    prev_ele_ft: f64,
    prev_ground: bool,
    /// Last tick's position — the roll direction at rotation, so a narrated
    /// takeoff names the runway END the ghost is departing FROM, not the
    /// threshold it happens to be nearest (liftoff is past midfield: every
    /// 09R departure read "two seven left" on the 2026-07-10 tape).
    prev_lat: f64,
    prev_lon: f64,
    announced_landing: bool,
    announced_takeoff: bool,
    /// "Continue approach" spoken (its aim runway was blocked by the player).
    announced_continue: bool,
}

pub(super) static GHOSTS: Mutex<Option<std::collections::HashMap<String, GhostMemory>>> = Mutex::new(None);

/// One line of a party-line exchange: who says it, and what.
pub struct PartyLine {
    /// Spoken by the GHOST pilot (its own pooled voice); false = the tower.
    pub ghost: bool,
    pub text: String,
}

/// A narrated exchange — the ghost pilot's call, the tower's reply, the
/// readback — queued in speaking order through the one arbiter.
pub struct PartyExchange {
    pub lines: Vec<PartyLine>,
    /// Stable voice seed for this ghost (hash of its callsign): Speedbird
    /// 306 is the same person on every call.
    pub ghost_seed: usize,
}

/// The strip the PLAYER holds for the party line: the one it stands on, or,
/// on the ground, the one the tower's book still gives it (cleared onto it,
/// or landed and not yet reported off). A ghost landing or departing that
/// strip is not narrated a clearance over our head. The "runway vacated"
/// report frees it, as it frees a real tower's runway, before the taxi
/// request that used to be the only thing that did (Liege 2026-09-22).
pub fn player_blocked_strip(on_ground: bool, lat: f64, lon: f64) -> Option<String> {
    if !on_ground {
        return None;
    }
    on_active_runway(lat, lon).or_else(|| {
        OWN_STRIP.clearance
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .filter(|c| c.holds_runway && !c.runway_ident.is_empty())
            .map(|c| c.runway_ident.clone())
    })
}

/// What OUR clearance claims of a strip, for the party line, on the ground
/// or in the air: `(end, whole)`. Cleared to land and not yet reported off,
/// the WHOLE strip is ours (nobody else is cleared onto it, either end);
/// sequenced for it in any other way (taxi, line-up, takeoff, the circuit's
/// Inbound, a go-around's), its DIRECTION is (nobody is cleared off or onto
/// its reciprocal). `None` without a clearance or a runway.
pub(super) fn own_strip_claim() -> Option<(String, bool)> {
    let clr = OWN_STRIP.clearance.lock().unwrap_or_else(|e| e.into_inner());
    let c = clr.as_ref().filter(|c| !c.runway_ident.is_empty())?;
    match c.state {
        tower::State::ClearedToLand if c.holds_runway => Some((c.runway_ident.clone(), true)),
        // Lined up or cleared for takeoff, the runway is ours alone: one
        // departure on it at a time, either direction (EGKK 08R, sortie
        // 1790601822-full_auto, 2026-09-28: "Easy eight six eight zero,
        // runway zero eight right, cleared for takeoff" 1.6 s after our own
        // takeoff clearance, while we lined up).
        tower::State::LinedUp | tower::State::TakeoffCleared => {
            Some((c.runway_ident.clone(), true))
        }
        tower::State::TaxiCleared | tower::State::Inbound => {
            Some((c.runway_ident.clone(), false))
        }
        _ => None,
    }
}

/// Our own departure is still spacing the strip `ghost_end` is on: we lifted
/// off it less than [`WAKE_SECS`] ago, so the tower has cleared nobody off it
/// behind us (the wake and departure spacing a real tower gives the next
/// departure). Read from the compliance watch's memory of our liftoff.
pub(super) fn own_departure_spacing(ghost_end: &str, now: Instant) -> bool {
    let Some((ours, at)) = COMPLIANCE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .liftoff
        .clone()
    else {
        return false;
    };
    if now.saturating_duration_since(at).as_secs() >= WAKE_SECS {
        return false;
    }
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    same_strip_or_twin(&runways, ghost_end, &ours)
}

/// Two runway ends on one strip, or on a strip and its close-parallel main
/// twin (`nav::close_parallel_main`): the replay puts a real 08R departure on
/// EGKK's 08L pavement 190 m away, and a wake clock kept on "26R" never
/// matched the held 08R (sortie 1790601822-full_auto, 2026-09-28).
pub(super) fn same_strip_or_twin(runways: &[nav::Runway], a: &str, b: &str) -> bool {
    let a = nav::close_parallel_main(runways, a);
    let b = nav::close_parallel_main(runways, b);
    matches!(
        (strip(runways, &a), strip(runways, &b)),
        (Some(x), Some(y)) if std::ptr::eq(x, y)
    )
}

/// How recently the party line must have seen a ghost for a change in it to
/// be narrated as happening now. The chatter is not ticked while a release is
/// held or for 45 s after an exchange, and a liftoff remembered across that
/// gap was narrated 17 s late as a fresh clearance ("Easy eight six eight
/// zero, cleared for takeoff" at 13:40:48 for U28680 airborne at 13:40:31,
/// EGKK 2026-09-28).
pub(super) const PARTY_LINE_FRESH_S: f64 = 5.0;

/// Whether a ghost's movement on `ghost_end` conflicts with our claim: the
/// same strip, and either the whole strip is ours or the ghost uses the
/// other direction. One strip, one direction (LEMG 2026-09-28).
pub(super) fn conflicts_with_claim(ghost_end: &str, claim: &(String, bool)) -> bool {
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    let same = matches!(
        (strip(&runways, ghost_end), strip(&runways, &claim.0)),
        (Some(x), Some(y)) if std::ptr::eq(x, y)
    );
    same && (claim.1 || ghost_end != claim.0)
}

/// One pass of the party line: narrate at most ONE exchange for the ghost
/// traffic — an arrival's final call answered with the landing clearance
/// while the ghost is still descending (spoken BEFORE reality lands it), a
/// takeoff clearance and readback as one rotates. Each ghost gets each
/// exchange once. The tower is never wrong: the "instruction" is derived
/// from what the replay is about to do anyway.
pub fn chatter_tick(
    targets: &[TcasTarget],
    station_name: &str,
    field_lat: f64,
    field_lon: f64,
    field_elev_ft: f64,
    // The runway END the PLAYER currently occupies (or holds a clearance
    // for): a ghost landing that strip gets "continue approach", never a
    // narrated "cleared to land" over the player's head — the 2026-07-10
    // tape cleared a takeoff and a landing on 09L eight seconds apart.
    blocked_ident: Option<&str>,
    now: Instant,
) -> Option<PartyExchange> {
    if station_unavailable() {
        return None;
    }
    // Read before the ghost memory's lock: our own clearance's claim.
    let claim = own_strip_claim();
    let mut guard = GHOSTS.lock().unwrap_or_else(|e| e.into_inner());
    let mem = guard.get_or_insert_with(std::collections::HashMap::new);
    mem.retain(|_, m| now.duration_since(m.last_seen).as_secs() < 180);
    let mut exchange = None;
    for t in targets {
        let dist = crate::nav::haversine_nm(t.lat, t.lon, field_lat, field_lon);
        if dist > 8.0 {
            continue; // beyond this tower's world
        }
        let m = mem.entry(t.callsign.clone()).or_insert(GhostMemory {
            last_seen: now,
            prev_ele_ft: t.ele_ft,
            prev_ground: t.on_ground,
            prev_lat: t.lat,
            prev_lon: t.lon,
            announced_landing: false,
            announced_takeoff: false,
            announced_continue: false,
        });
        let descending = t.ele_ft < m.prev_ele_ft - 1.0;
        // A change seen across a gap in the watch is history, not news.
        let fresh = now.saturating_duration_since(m.last_seen).as_secs_f64() <= PARTY_LINE_FRESH_S;
        let was_ground = m.prev_ground;
        let agl = t.ele_ft - field_elev_ft;
        if exchange.is_none()
            && !t.on_ground
            && !m.announced_landing
            && dist < 5.0
            && (150.0..1200.0).contains(&agl)
            && descending
        {
            let cs = spoken_ghost(&t.callsign);
            let aim = nearest_runway_ident(t.lat, t.lon).map(|id| main_runway_of(&id));
            // The player is on (or cleared onto) this ghost's strip: no
            // narrated landing clearance over their head — "continue" is
            // what a real tower says, and it stays true when the replay
            // lands anyway. The clearance may still come once the player
            // vacates and the ghost is re-seen clear.
            let blocked = match (&aim, blocked_ident) {
                (Some(a), Some(b)) => {
                    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
                    matches!(
                        (strip(&runways, a), strip(&runways, b)),
                        (Some(x), Some(y)) if std::ptr::eq(x, y)
                    )
                }
                _ => false,
            };
            if blocked {
                if !m.announced_continue {
                    m.announced_continue = true;
                    exchange = Some(PartyExchange {
                        lines: vec![
                            PartyLine {
                                ghost: true,
                                text: format!(
                                    "{station_name} Tower, {cs}, final runway {}.",
                                    aim.as_deref().map(atis::spoken_runway).unwrap_or_default()
                                ),
                            },
                            PartyLine {
                                ghost: false,
                                text: format!(
                                    "{cs}, continue approach, traffic on the runway ahead."
                                ),
                            },
                        ],
                        ghost_seed: crate::speaker::station_voice_seed(&t.callsign),
                    });
                }
                m.last_seen = now;
                m.prev_ele_ft = t.ele_ft;
                m.prev_ground = t.on_ground;
                m.prev_lat = t.lat;
                m.prev_lon = t.lon;
                continue;
            }
            // Our clearance claims this strip (cleared to land on it, or
            // sequenced the other way): the tower narrates no clearance it
            // could not have given. Silent, not "continue": the ghost may
            // be the one flying against us, and the truth about it is the
            // go-around watch's, not flavour. The clearance may still come
            // once the claim is gone and the ghost is re-seen in the band.
            if let (Some(a), Some(c)) = (&aim, &claim) {
                if conflicts_with_claim(a, c) {
                    sim::blackbox::record(
                        "atc",
                        serde_json::json!({"event": "party_line_withheld", "ghost": t.callsign,
                            "call": "landing", "runway": a, "ours": c.0, "whole": c.1}),
                    );
                    m.last_seen = now;
                    m.prev_ele_ft = t.ele_ft;
                    m.prev_ground = t.on_ground;
                    m.prev_lat = t.lat;
                    m.prev_lon = t.lon;
                    continue;
                }
            }
            m.announced_landing = true;
            let rwy = aim.as_deref().map(atis::spoken_runway);
            let lines = match &rwy {
                Some(r) => vec![
                    PartyLine {
                        ghost: true,
                        text: format!("{station_name} Tower, {cs}, final runway {r}."),
                    },
                    PartyLine {
                        ghost: false,
                        text: format!("{cs}, runway {r}, cleared to land."),
                    },
                    PartyLine {
                        ghost: true,
                        text: format!("Cleared to land runway {r}, {cs}."),
                    },
                ],
                None => vec![
                    PartyLine {
                        ghost: true,
                        text: format!("{station_name} Tower, {cs}, final."),
                    },
                    PartyLine {
                        ghost: false,
                        text: format!("{cs}, cleared to land."),
                    },
                ],
            };
            exchange = Some(PartyExchange {
                lines,
                ghost_seed: crate::speaker::station_voice_seed(&t.callsign),
            });
        }
        if exchange.is_none() && was_ground && !t.on_ground && !m.announced_takeoff && dist < 3.0
            && !fresh
        {
            // Lifted off while nobody was watching: spent, never narrated late.
            m.announced_takeoff = true;
        }
        if exchange.is_none() && was_ground && !t.on_ground && !m.announced_takeoff && dist < 3.0 {
            m.announced_takeoff = true;
            let cs = spoken_ghost(&t.callsign);
            // Name the end the ghost DEPARTED, by its roll direction —
            // liftoff is past midfield, so the nearest threshold is the
            // reciprocal (every 09R departure read "two seven left" on the
            // 2026-07-10 tape).
            let ident = departure_runway_ident(t.lat, t.lon, m.prev_lat, m.prev_lon)
                .or_else(|| nearest_runway_ident(t.lat, t.lon))
                .map(|id| main_runway_of(&id));
            // The player holds this strip: the tower gave nobody else a
            // takeoff on it, so the replay's departure goes un-narrated.
            let held = match (&ident, blocked_ident) {
                (Some(a), Some(b)) => {
                    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
                    matches!(
                        (strip(&runways, a), strip(&runways, b)),
                        (Some(x), Some(y)) if std::ptr::eq(x, y)
                    )
                }
                _ => false,
            };
            // In the air too, and by direction: nobody is cleared off the
            // strip we are cleared to land on, nor off its reciprocal while
            // we are sequenced for it (LEMG, sortie 1790584715-full_auto:
            // "Vueling eight one seven six, runway one three, cleared for
            // takeoff" with our circuit on downwind for 31, "Easy two seven
            // one six" after our go-around from the 31 final).
            let claimed = match (&ident, &claim) {
                (Some(a), Some(c)) => conflicts_with_claim(a, c),
                _ => false,
            } || ident.as_deref().is_some_and(|a| own_departure_spacing(a, now));
            if claimed {
                sim::blackbox::record(
                    "atc",
                    serde_json::json!({"event": "party_line_withheld", "ghost": t.callsign,
                        "call": "takeoff", "runway": ident, "ours": claim.as_ref().map(|c| c.0.clone()),
                        "whole": claim.as_ref().map(|c| c.1)}),
                );
            }
            if held || claimed {
                m.last_seen = now;
                m.prev_ele_ft = t.ele_ft;
                m.prev_ground = t.on_ground;
                m.prev_lat = t.lat;
                m.prev_lon = t.lon;
                continue;
            }
            let clearance = match ident {
                Some(id) => format!(
                    "{cs}, runway {}, cleared for takeoff.",
                    atis::spoken_runway(&id)
                ),
                None => format!("{cs}, cleared for takeoff."),
            };
            exchange = Some(PartyExchange {
                lines: vec![
                    PartyLine {
                        ghost: false,
                        text: clearance,
                    },
                    PartyLine {
                        ghost: true,
                        text: format!("Cleared for takeoff, {cs}."),
                    },
                ],
                ghost_seed: crate::speaker::station_voice_seed(&t.callsign),
            });
        }
        m.last_seen = now;
        m.prev_ele_ft = t.ele_ft;
        m.prev_ground = t.on_ground;
        m.prev_lat = t.lat;
        m.prev_lon = t.lon;
    }
    exchange
}
