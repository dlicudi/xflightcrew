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
    let ahead = landing_sequence(&own.runway, now)
        .into_iter()
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
