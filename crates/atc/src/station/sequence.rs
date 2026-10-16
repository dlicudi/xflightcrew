//! The arrival sequence (2026-10-14, `docs/tasks/2026-10-14-arrival-sequencing.md`):
//! our own arrival as the station has armed it, for the traffic link to put
//! our slot in the traffic's picture.

use super::*;

/// Our arrival as the station has it armed: the runway END we land on,
/// where the vectors (or approach's own-navigation clearance) join the
/// final, and whether the tower has cleared us to land. Read by the traffic
/// link ([`crate::party_line::own_arrival_plan`]), which works out our slot
/// at the runway from it.
#[derive(Debug, Clone, PartialEq)]
pub struct OwnArrivalPlan {
    /// The landing END ("13").
    pub runway_ident: String,
    /// Where the final is joined, nm from the threshold, while a radar
    /// controller is bringing us onto it.
    pub join_nm: Option<f64>,
    /// Cleared to land.
    pub cleared: bool,
}

/// The runway end whose threshold is at `(lat, lon)`, of the installed
/// field's runways (a vectors plan and an approach watch carry the
/// threshold, not the ident).
fn end_at(runways: &[nav::Runway], lat: f64, lon: f64) -> Option<String> {
    runways.iter().find_map(|r| {
        if nav::haversine_nm(lat, lon, r.lat_a, r.lon_a) < 0.1 {
            Some(r.ident_a.clone())
        } else if nav::haversine_nm(lat, lon, r.lat_b, r.lon_b) < 0.1 {
            Some(r.ident_b.clone())
        } else {
            None
        }
    })
}

/// Our arrival as armed on our strip: cleared to land, under vectors,
/// watched by approach on its own navigation, a landing clearance withheld,
/// or Inbound to the tower for a runway. `None` with none of them (a crew
/// on a final nobody armed is found by the link's own final wedge).
pub fn own_arrival_plan() -> Option<OwnArrivalPlan> {
    let runways = ACTIVE_RUNWAYS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone();
    let clearance = OWN_STRIP
        .clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|c| (c.state, c.runway_ident.clone()));
    if let Some((tower::State::ClearedToLand, ident)) = &clearance {
        if !ident.is_empty() {
            return Some(OwnArrivalPlan {
                runway_ident: ident.clone(),
                join_nm: None,
                cleared: true,
            });
        }
    }
    let vectored = OWN_STRIP
        .vectors
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .and_then(|p| Some((end_at(&runways, p.thr_lat, p.thr_lon)?, p.join_nm)));
    let watched = || {
        OWN_STRIP
            .approach_watch
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .and_then(|w| Some((end_at(&runways, w.thr_lat, w.thr_lon)?, w.join_nm)))
    };
    if let Some((ident, join)) = vectored.or_else(watched) {
        return Some(OwnArrivalPlan {
            runway_ident: ident,
            join_nm: Some(join),
            cleared: false,
        });
    }
    let held = OWN_STRIP
        .landing_hold
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|h| h.runway_ident.clone());
    let inbound = match clearance {
        Some((tower::State::Inbound, ident)) => Some(ident),
        _ => None,
    };
    held.or(inbound)
        .filter(|i| !i.is_empty())
        .map(|ident| OwnArrivalPlan {
            runway_ident: ident,
            join_nm: None,
            cleared: false,
        })
}

/// A producer's runway times are a measurement for this long, seconds: a
/// link that stops ticking is no place in the sequence.
pub const TIMES_FRESH_S: f64 = 5.0;

/// An order once SPOKEN to us holds unless the gap reverses by more than
/// this, seconds: a number that flips every tick is worse than none.
pub const SEQUENCE_REVERSE_S: f64 = 30.0;

/// One strip's fresh runway times, if it has any.
fn fresh_times(strip: &FlightStrip, now: Instant) -> Option<RunwayTimes> {
    strip
        .runway_times
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
        .filter(|t| now.saturating_duration_since(t.at).as_secs_f64() <= TIMES_FRESH_S)
}

/// Every flight's fresh runway times, ours and the traffic's.
fn all_times(now: Instant) -> Vec<(FlightId, RunwayTimes)> {
    fresh_times(&OWN_STRIP, now)
        .map(|t| (FlightId::Own, t))
        .into_iter()
        .chain(
            traffic_strips()
                .into_iter()
                .filter_map(|(id, s)| fresh_times(&s, now).map(|t| (FlightId::Traffic(id), t))),
        )
        .collect()
}

/// THE LANDING SEQUENCE on `runway` (a landing end), computed from the
/// strips every time it is asked, never stored, in the spirit of
/// [`RunwayBook`]: every flight whose producer says it is landing there
/// and not yet over the threshold, ordered by its time to the threshold.
pub fn landing_sequence(runway: &str, now: Instant) -> Vec<(FlightId, f64)> {
    let mut seq: Vec<(FlightId, f64)> = all_times(now)
        .into_iter()
        .filter(|(_, t)| t.runway.eq_ignore_ascii_case(runway) && t.to_thr_s > 0.0)
        .map(|(id, t)| (id, t.to_thr_s))
        .collect();
    seq.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
    seq
}

/// Whether `other`, `other_s` from its threshold, is ahead of us at
/// `own_s`, with the hysteresis of the order last spoken to us: one said
/// to be ahead stays ahead until it is [`SEQUENCE_REVERSE_S`] BEHIND us,
/// one said not to be (or not there when it was said) goes ahead only
/// once it is that much in front. With nothing spoken, the times decide.
pub(crate) fn ahead_with_hysteresis(
    spoken: Option<&SpokenOrder>,
    other: FlightId,
    other_s: f64,
    own_s: f64,
) -> bool {
    match spoken.map(|s| s.ahead.contains(&other)) {
        Some(true) => other_s <= own_s + SEQUENCE_REVERSE_S,
        Some(false) => other_s < own_s - SEQUENCE_REVERSE_S,
        None => other_s < own_s,
    }
}

/// The flights ahead of us in our runway's landing sequence, nearest the
/// threshold first, as the hysteresis of the order spoken to us has it
/// ([`ahead_with_hysteresis`]), with our runway and our own time. `None`
/// when we are not in a sequence.
pub(crate) fn own_ahead(now: Instant) -> Option<(String, f64, Vec<(FlightId, f64)>)> {
    let own = fresh_times(&OWN_STRIP, now).filter(|t| t.to_thr_s > 0.0)?;
    let spoken = OWN_STRIP
        .arrival_order
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
        .filter(|s| s.runway.eq_ignore_ascii_case(&own.runway));
    // A mayday is number one: nobody still flying a final is ahead of it.
    let distress = OWN_STRIP.distress.load(Ordering::SeqCst);
    let ahead = landing_sequence(&own.runway, now)
        .into_iter()
        .filter(|_| !distress)
        .filter(|(id, _)| *id != FlightId::Own)
        .filter(|(id, s)| ahead_with_hysteresis(spoken.as_ref(), *id, *s, own.to_thr_s))
        .collect();
    Some((own.runway, own.to_thr_s, ahead))
}

/// Our number in the landing sequence of our runway: one plus the flights
/// ahead of us. `None` when we are not in a sequence.
pub fn own_number() -> Option<u8> {
    own_number_at(Instant::now())
}

pub(crate) fn own_number_at(now: Instant) -> Option<u8> {
    let (_, _, ahead) = own_ahead(now)?;
    Some(u8::try_from(ahead.len() + 1).unwrap_or(u8::MAX))
}

/// The flight we follow: the last of those ahead of us. Its raw callsign,
/// its type when known and its distance to the threshold, nm.
pub fn own_leader() -> Option<(String, Option<String>, f64)> {
    own_leader_at(Instant::now()).map(|l| (l.callsign, l.icao_type, l.dist_nm))
}

/// The flight we follow, as the approach and the tower need it.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Leader {
    pub id: FlightId,
    pub callsign: String,
    pub icao_type: Option<String>,
    pub dist_nm: f64,
    /// Seconds from it to us at the threshold.
    pub gap_s: f64,
    /// Seconds from now until it is off the runway again.
    pub clear_s: f64,
}

pub(crate) fn own_leader_at(now: Instant) -> Option<Leader> {
    let (_, own_s, ahead) = own_ahead(now)?;
    let (id, s) = *ahead.last()?;
    let strip = match id {
        FlightId::Own => return None,
        FlightId::Traffic(n) => traffic_strip(n)?,
    };
    let t = fresh_times(&strip, now)?;
    Some(Leader {
        id,
        icao_type: t
            .icao_type
            .clone()
            .or_else(|| crate::station::traffic_type(&t.callsign)),
        callsign: t.callsign,
        dist_nm: t.dist_nm,
        gap_s: own_s - s,
        clear_s: t.clear_s,
    })
}

/// The order as it stands for us now is SPOKEN: from here it holds until
/// the gap reverses ([`SEQUENCE_REVERSE_S`]).
pub(crate) fn note_order_spoken(now: Instant) {
    let Some((runway, _, ahead)) = own_ahead(now) else {
        return;
    };
    *OWN_STRIP
        .arrival_order
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = Some(SpokenOrder {
        runway,
        ahead: ahead.into_iter().map(|(id, _)| id).collect(),
    });
}

// ---- approach: the sequence spoken, the speed controlled ----

/// The radar unit's speed schedule, nm from the threshold and knots: 250
/// below FL100 beyond 20 nm, 210 to 15, 180 to the join (11), 160 to 4 nm.
/// The SAME bands the traffic flies (`traffic::arrival::SPEED_BANDS`, the
/// crew pins the two equal: the crates never name each other).
pub const APPROACH_SPEED_BANDS: [(f64, f64); 4] =
    [(20.0, 250.0), (15.0, 210.0), (11.0, 180.0), (4.0, 160.0)];

/// The gap between the one ahead reporting off the runway and us needing
/// it, seconds: the traffic's own (`traffic::slot::FOLLOW_GAP_S`, pinned
/// equal by the crew).
pub const SEQUENCE_GAP_S: f64 = 20.0;

/// The least radar spacing behind the one ahead on final, nm: 3 nm and a
/// controller's buffer (`traffic::arrival::WAKE_BUFFER_NM`).
pub const RADAR_SPACING_NM: f64 = 3.3;

/// A spacing reduction stands until we are this far ahead of our place,
/// seconds: slowed, we make the gap back, and a speed that flips back the
/// moment the gap closes is worse than none.
const SPACING_RELEASE_S: f64 = 15.0;

/// The base is delayed only once slowing to our minimum would still leave
/// us short by this much, seconds.
const DELAY_MARGIN_S: f64 = 15.0;

/// The join is never moved further out than this, nm.
const JOIN_MAX_NM: f64 = 20.0;

/// An owed speed readback is asked for again after this, seconds.
const SPEED_READBACK_S: f64 = 20.0;

/// The speed approach assigned, and its readback.
#[derive(Debug, Clone, PartialEq)]
pub struct AssignedSpeed {
    pub kts: f64,
    /// "band" (the schedule) or "spacing" (behind the one ahead).
    pub why: &'static str,
    /// The instruction as said, without the callsign.
    pub words: String,
    pub read_back: bool,
    pub at: Instant,
    /// The readback has been asked for again.
    pub nudged: bool,
}

/// What approach last said about our sequence and speed.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ApproachSaid {
    /// Our number and the flight we follow, as last said.
    pub order: Option<(u8, FlightId)>,
    /// The speed last assigned, and why.
    pub speed: Option<(f64, &'static str)>,
    /// The join was moved out to this, nm.
    pub join_nm: Option<f64>,
}

/// The speed approach wants of us `dist_nm` from the threshold at `alt_ft`:
/// the schedule's band ([`APPROACH_SPEED_BANDS`], the 250 only below
/// FL100), only when the type can fly it (`min_kts` to `max_kts`); behind
/// a leader and short of our place by `short_s`, our minimum approach speed
/// instead (`was_spacing`: one already given stands until we are
/// [`SPACING_RELEASE_S`] ahead of our place). Inside 4 nm nothing: the 160
/// simply ends. Pure.
pub fn wanted_speed(
    dist_nm: f64,
    alt_ft: f64,
    speeds: crate::party_line::ApproachSpeeds,
    short_s: f64,
    was_spacing: bool,
) -> Option<(f64, &'static str)> {
    if !dist_nm.is_finite() || dist_nm <= 4.0 {
        return None;
    }
    let band = APPROACH_SPEED_BANDS
        .iter()
        .find(|(from, _)| dist_nm > *from)
        .map(|(_, k)| *k)?;
    let min = speeds.min_kts.round();
    let spacing = if was_spacing {
        short_s > -SPACING_RELEASE_S
    } else {
        short_s > 0.0
    };
    if spacing && min < band {
        return Some((min, "spacing"));
    }
    if band >= 250.0 && alt_ft >= 10_000.0 {
        return None;
    }
    if band < min || speeds.max_kts.is_some_and(|m| band > m) {
        return None;
    }
    Some((band, "band"))
}

/// Seconds we are short of our place behind `leader`: it off the runway
/// [`SEQUENCE_GAP_S`] before our landing clearance is due, and at least
/// [`RADAR_SPACING_NM`] behind it at our own pace. Negative: room to spare.
pub(crate) fn spacing_short_s(own: &RunwayTimes, leader: &Leader) -> f64 {
    let leader_s = own.to_thr_s - leader.gap_s;
    let kts = if own.to_thr_s > 0.0 {
        own.dist_nm / own.to_thr_s * 3600.0
    } else {
        f64::NAN
    };
    let mut need = leader.clear_s + SEQUENCE_GAP_S + own.needs_clear_s;
    if kts.is_finite() && kts > 40.0 {
        need = need.max(leader_s + RADAR_SPACING_NM / kts * 3600.0);
    }
    need - own.to_thr_s
}

/// "number two, following a Boeing 737 on 8 mile final" in the dialect
/// (CAP 413 6.5: "number two, following ..."; FAA 7110.65 5-9: "number
/// two, follow ..."). An unknown type is "traffic".
pub fn sequence_words(
    dialect: Dialect,
    number: u8,
    icao_type: Option<&str>,
    dist_nm: f64,
) -> String {
    let what = match icao_type.and_then(crate::station::spoken_type) {
        Some(t) => {
            let an = t
                .chars()
                .next()
                .is_some_and(|c| matches!(c.to_ascii_uppercase(), 'A' | 'E' | 'I' | 'O' | 'U'));
            format!("{} {t}", if an { "an" } else { "a" })
        }
        None => "traffic".to_string(),
    };
    let fin = tower::spoken_final(dist_nm);
    let n = tower::sequence_number(number);
    match dialect {
        Dialect::Icao => format!("number {n}, following {what} on {fin}"),
        Dialect::Faa => format!("number {n}, follow {what}, {fin}"),
    }
}

/// A speed instruction in the dialect: the 160 to 4 nm says where it ends
/// ("maintain one six zero knots to four DME"; FAA "until four mile
/// final"), a reduction "reduce speed to ..." (`reducing`), else
/// "maintain ... knots".
pub fn speed_words(dialect: Dialect, kts: f64, reducing: bool) -> String {
    let k = atis::spoken_digits(&format!("{:03}", kts.round() as i64));
    if (kts - 160.0).abs() < 0.5 {
        return match dialect {
            Dialect::Icao => format!("maintain {k} knots to four DME"),
            Dialect::Faa => format!("maintain {k} knots until four mile final"),
        };
    }
    if reducing {
        format!("reduce speed to {k} knots")
    } else {
        format!("maintain {k} knots")
    }
}

/// Approach's sequencing for OUR flight, one tick (`vector_tick_for`, the
/// approach watch): our number and the traffic we follow, ONCE per change;
/// a speed on the schedule's band when the band changes, or the minimum
/// approach speed behind a leader we are short of; still short beyond the
/// join with the speed spent, the join moved out (`plan`: the vectors'),
/// "extend downwind" on a downwind-shaped leg; an owed speed readback asked
/// for once. The words without the callsign, `None` with nothing to say.
/// Blackbox `atc` `sequence`, `speed_assigned`, `downwind_extended`.
pub(crate) fn approach_sequence_lines(
    flight: &FlightStrip,
    state: &AircraftState,
    track_deg: Option<f64>,
    plan: Option<&mut VectorPlan>,
    dialect: Dialect,
    now: Instant,
) -> Option<String> {
    if !flight.is_own() || state.on_ground {
        return None;
    }
    let own = fresh_times(flight, now).filter(|t| t.to_thr_s > 0.0)?;
    let number = own_number_at(now)?;
    let leader = own_leader_at(now);
    let mut said_guard = flight
        .approach_said
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let said = said_guard.get_or_insert_with(ApproachSaid::default);
    let mut parts: Vec<String> = Vec::new();
    // The number and the traffic, once per change.
    if let Some(l) = &leader {
        let key = (number, l.id);
        if said.order != Some(key) {
            said.order = Some(key);
            parts.push(sequence_words(
                dialect,
                number,
                l.icao_type.as_deref(),
                l.dist_nm,
            ));
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "sequence", "number": number,
                    "leader": l.callsign, "gap_s": l.gap_s.round(),
                    "leader_nm": (l.dist_nm * 10.0).round() / 10.0, "runway": own.runway}),
            );
        }
    }
    // The speed: behind the leader, and leaving a departure waiting at the
    // hold its gap ahead of us as the traffic does (#300).
    let short = leader
        .as_ref()
        .map_or(f64::NEG_INFINITY, |l| spacing_short_s(&own, l))
        .max(
            own.depart_gap_s
                .map_or(f64::NEG_INFINITY, |g| g - own.to_thr_s),
        );
    if let Some(speeds) = crate::party_line::approach_speeds() {
        let was_spacing = said.speed.is_some_and(|(_, w)| w == "spacing");
        let want = wanted_speed(own.dist_nm, state.alt_ft, speeds, short, was_spacing);
        match want {
            Some((kts, why)) if said.speed != Some((kts, why)) => {
                let reducing = state.gs_kts.is_finite() && state.gs_kts > kts + 10.0;
                let words = speed_words(dialect, kts, reducing);
                said.speed = Some((kts, why));
                *flight
                    .assigned_speed
                    .lock()
                    .unwrap_or_else(|e| e.into_inner()) = Some(AssignedSpeed {
                    kts,
                    why,
                    words: words.clone(),
                    read_back: false,
                    at: now,
                    nudged: false,
                });
                publish_speed_readback(flight, kts);
                sim::blackbox::record(
                    "atc",
                    serde_json::json!({"event": "speed_assigned", "kts": kts, "why": why,
                        "dist_nm": (own.dist_nm * 10.0).round() / 10.0}),
                );
                parts.push(words);
            }
            Some(_) => {}
            // Inside 4 nm the 160 simply ends: nothing said, nothing owed.
            None if own.dist_nm <= 4.0 => {
                said.speed = None;
                *flight
                    .assigned_speed
                    .lock()
                    .unwrap_or_else(|e| e.into_inner()) = None;
            }
            None => {}
        }
        // Still short beyond the join with the speed spent: the base is
        // delayed, the join moved out along the final.
        if let Some(plan) = plan {
            let min = speeds.min_kts.max(40.0);
            let can_add = (own.dist_nm / min * 3600.0 - own.to_thr_s).max(0.0);
            let beyond = own.dist_nm > plan.join_nm + 1.0;
            if beyond && short > can_add + DELAY_MARGIN_S {
                let out = ((short - can_add) * min / 3600.0).max(1.0);
                let join = (plan.join_nm + out).min(JOIN_MAX_NM);
                if join > plan.join_nm + 0.5 {
                    plan.join_nm = join;
                    said.join_nm = Some(join);
                    sim::blackbox::record(
                        "atc",
                        serde_json::json!({"event": "downwind_extended",
                            "join_nm": (join * 10.0).round() / 10.0,
                            "short_s": short.round()}),
                    );
                    let downwind = track_deg.is_some_and(|t| {
                        sim::angles::abs_diff_deg(t, plan.course_deg + 180.0) <= 45.0
                    });
                    if downwind {
                        parts.push("extend downwind".to_string());
                    }
                }
            }
        }
    }
    drop(said_guard);
    // An owed speed readback, asked for once.
    if parts.is_empty() {
        let mut g = flight
            .assigned_speed
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if let Some(a) = g.as_mut().filter(|a| !a.read_back && !a.nudged) {
            if now.saturating_duration_since(a.at).as_secs_f64() >= SPEED_READBACK_S {
                a.nudged = true;
                parts.push(format!("read back the speed, {}", a.words));
            }
        }
    }
    if parts.is_empty() {
        return None;
    }
    // Spoken: the order holds from here until the gap reverses.
    note_order_spoken(now);
    Some(parts.join(", "))
}

/// The crew's readback of an assigned speed ("one six zero knots"), with
/// approach's clearance readback when one is owed too.
fn publish_speed_readback(flight: &FlightStrip, kts: f64) {
    let words = format!(
        "{} knots",
        atis::spoken_digits(&format!("{:03}", kts.round() as i64))
    );
    let mut g = flight
        .approach_readback
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let next = match g.take() {
        Some(prev) => format!("{prev}, {words}"),
        None => words,
    };
    *g = Some(next);
}

/// The speed and what approach said of the sequence are over: the handoff
/// to the tower, the landing.
pub(crate) fn retire_approach_speed(flight: &FlightStrip) {
    *flight
        .assigned_speed
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = None;
    *flight
        .approach_said
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = None;
}

/// A go-around: the speed and the order spoken are void (the arrival is
/// sequenced afresh when it comes back round).
pub(crate) fn retire_sequence(flight: &FlightStrip) {
    retire_approach_speed(flight);
    *flight
        .arrival_order
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = None;
}

/// Our transmission to approach while a speed readback is owed: the right
/// speed is the readback (silence), a different one is corrected; no speed
/// heard is no readback (the call is answered as before). `None` with no
/// readback owed or no speed heard.
pub(crate) fn judge_speed_readback(transcript: &str, callsign: &str) -> Option<String> {
    let mut g = OWN_STRIP
        .assigned_speed
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let a = g.as_mut().filter(|a| !a.read_back)?;
    let heard = tower::heard_speed_kts(transcript)?;
    if (heard as f64 - a.kts).abs() < 0.5 {
        a.read_back = true;
        sim::blackbox::record(
            "atc",
            serde_json::json!({"event": "speed_read_back", "kts": a.kts}),
        );
        return Some(String::new());
    }
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "speed_readback_wrong", "kts": a.kts, "heard": heard}),
    );
    Some(format!("{callsign}, negative, {}.", a.words))
}

/// The sequence as it stands for us now, in the dialect, for a reply that
/// arms our approach (the vectors' opening): `None` with no leader.
pub(crate) fn sequence_clause_now(dialect: Dialect, now: Instant) -> Option<String> {
    let number = own_number_at(now)?;
    let l = own_leader_at(now)?;
    let words = sequence_words(dialect, number, l.icao_type.as_deref(), l.dist_nm);
    OWN_STRIP
        .approach_said
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .get_or_insert_with(ApproachSaid::default)
        .order = Some((number, l.id));
    note_order_spoken(now);
    Some(words)
}

/// A line of approach's: `msg` (a heading, a clearance) with the
/// sequencing's words after it, or the sequencing's words alone, to
/// `callsign`.
pub(crate) fn with_sequence(
    callsign: &str,
    msg: Option<String>,
    extra: Option<String>,
) -> Option<String> {
    match (msg, extra) {
        (Some(m), Some(e)) => Some(format!("{m} {}.", capitalise(&e))),
        (Some(m), None) => Some(m),
        (None, Some(e)) => Some(format!("{callsign}, {e}.")),
        (None, None) => None,
    }
}

fn capitalise(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

// ---- the tower: number two, never a clearance it must cancel (#237) ----

/// A lander the tower knows nothing more of (the sim's traffic, a ghost)
/// is off the runway this long after its threshold, seconds: a jet's flare
/// and rollout to a turn-off (our own 737 was off 43 s after touchdown at
/// XTST, `traffic`'s slot test).
pub const GHOST_RUNWAY_S: f64 = 50.0;

/// Our landing clearance is due this far out, nm: 300 ft on a 3 degree
/// path (the traffic's `NO_CLEARANCE_AGL_FT`).
const CLEARANCE_LEAD_NM: f64 = 0.94;

/// The flight ahead of us on our final that will NOT be off the runway
/// [`SEQUENCE_GAP_S`] before we need it (our landing clearance is due):
/// the tower says "continue approach, number two, traffic ..." and holds
/// the clearance. With our own times in the landing sequence, the
/// sequence's leader and its runway time ([`own_leader_at`]); else (the
/// sim's traffic, no producer) the nearest flight airborne on our final
/// between us and the threshold, timed at [`DEFAULT_FINAL_KTS`] with
/// [`GHOST_RUNWAY_S`] on the runway. Our strip only. `None`: nobody ahead
/// that the runway is not clear of in time. LEMG 30 (#237, 2026-10-07):
/// "cleared to land" at our 997 ft final report with live traffic landing
/// 1.4 nm ahead, then "go around, traffic on the runway" twice.
pub(crate) fn landing_leader_blocking(
    flight: &FlightStrip,
    runway_ident: &str,
    targets: &[TcasTarget],
    now: Instant,
) -> Option<tower::LandingLeader> {
    // A mayday's landing is never withheld behind traffic still flying.
    if !flight.is_own() || flight.distress.load(Ordering::SeqCst) {
        return None;
    }
    let own_times = fresh_times(flight, now)
        .filter(|t| t.runway.eq_ignore_ascii_case(runway_ident) && t.to_thr_s > 0.0);
    if let Some(own) = own_times {
        let l = own_leader_at(now)?;
        let needs_runway_in = own.to_thr_s - own.needs_clear_s;
        if l.clear_s + SEQUENCE_GAP_S <= needs_runway_in {
            return None;
        }
        return Some(tower::LandingLeader {
            number: own_number_at(now).unwrap_or(2),
            type_spoken: l.icao_type.as_deref().and_then(crate::station::spoken_type),
            dist_nm: l.dist_nm,
        });
    }
    let r = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        strip(&runways, runway_ident).cloned()
    }?;
    let (own_nm, own_s) = time_to_threshold_for(flight, &r, runway_ident, now)?;
    let f = LandingFrame::new(&r, runway_ident)?;
    let elev = ACTIVE_PANEL
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|p| p.1);
    let ahead: Vec<(f64, &TcasTarget)> = targets
        .iter()
        .filter(|t| !t.on_ground && t.vs_fpm <= CLIMBOUT_FPM)
        .filter_map(|t| {
            let (along, cross) = f.place(t.lat, t.lon);
            let out = -along;
            if out <= 0.0
                || cross.abs() > out * FINAL_CONE_DEG.to_radians().tan() + FINAL_CONE_BASE_M
            {
                return None;
            }
            let path_ft = out * 3.28084 * 3.0_f64.to_radians().tan();
            if elev.is_some_and(|e| t.ele_ft - e > 2.0 * path_ft + FINAL_HIGH_MARGIN_FT) {
                return None;
            }
            let nm = out / 1852.0;
            (nm < own_nm - 0.2).then_some((nm, t))
        })
        .collect();
    let needs_runway_in = own_s - own_s * CLEARANCE_LEAD_NM / own_nm.max(CLEARANCE_LEAD_NM);
    let (nm, t) = ahead
        .iter()
        .filter(|(nm, _)| {
            nm / DEFAULT_FINAL_KTS * 3600.0 + GHOST_RUNWAY_S + SEQUENCE_GAP_S > needs_runway_in
        })
        .max_by(|a, b| a.0.total_cmp(&b.0))?;
    Some(tower::LandingLeader {
        number: u8::try_from(ahead.len() + 1).unwrap_or(u8::MAX),
        type_spoken: crate::station::traffic_type(&t.callsign)
            .as_deref()
            .and_then(crate::station::spoken_type),
        dist_nm: *nm,
    })
}
