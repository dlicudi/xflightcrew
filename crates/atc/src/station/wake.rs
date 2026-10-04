//! Wake turbulence and the live departure and arrival streams.

use super::*;

// --- Wake turbulence -------------------------------------------------------

/// The last airline-sized departure off a strip: wake separation runs from
/// its liftoff. TCAS carries no aircraft type, so "an airline callsign" is
/// the deterministic stand-in for "medium or heavier" — at the fields where
/// wake matters (Heathrow's replay stream) it's right essentially always,
/// and a GA ghost departing ahead of you correctly starts no clock.
pub struct Wake {
    pub since: Instant,
    /// The strip it departed from (end ident, e.g. "27R").
    pub runway_ident: String,
}

pub(super) static WAKE: Mutex<Option<Wake>> = Mutex::new(None);

/// ICAO's two minutes for a light departing behind a medium/heavy from the
/// same position.
pub const WAKE_SECS: u64 = 120;

/// Ground/air memory per ghost for liftoff detection, independent of the
/// party line (which is config-gated and must not be a safety dependency).
pub(super) static WAKE_PREV: Mutex<Option<std::collections::HashMap<String, WakePrev>>> =
    Mutex::new(None);

/// One ghost's last sample for the liftoff watch: on the ground, when seen,
/// and where (the roll direction names the END it departed).
#[derive(Clone, Copy)]
pub(super) struct WakePrev {
    on_ground: bool,
    seen: Instant,
    lat: f64,
    lon: f64,
}

/// The live DEPARTURE stream: the runway end the last ghost lifted off, by
/// its roll direction, and when. A departure is an event, so it is
/// remembered here, for [`DEPARTURE_STREAM_SECS`], by the same watch that
/// starts the wake clock; landings likewise ([`ARRIVAL_STREAM`]).
pub(super) static DEPARTURE_STREAM: Mutex<Option<(String, Instant)>> = Mutex::new(None);

/// How long a ghost's liftoff still says which way the field departs.
pub const DEPARTURE_STREAM_SECS: u64 = 600;

/// The end the field is departing from right now: the last ghost liftoff
/// within [`DEPARTURE_STREAM_SECS`], else `None`.
pub fn departure_end(now: Instant) -> Option<String> {
    DEPARTURE_STREAM
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(_, at)| now.saturating_duration_since(*at).as_secs() < DEPARTURE_STREAM_SECS)
        .map(|(end, _)| end.clone())
}

/// The remembered ARRIVAL stream: the runway end the last ghost touched down
/// on, by its track from the air to the pavement, and when. [`arrival_end`]
/// sees a lander only while it is on final; between landers nothing knew the
/// field was landing 12, and the SR22's stated 30 was cleared head-on to it in
/// calm (sortie 1790610218-approach, LEMG 2026-09-28: Ryanair 431 and 590
/// landed 12 at t 1790610003 and 1790610134, "cleared to land runway three
/// zero" at t 1790610223, sent round twice, issue #164). Kept, like the
/// departure stream, for [`ARRIVAL_STREAM_SECS`] by the wake watch.
pub(super) static ARRIVAL_STREAM: Mutex<Option<(String, Instant)>> = Mutex::new(None);

/// How long a ghost's touchdown still says which way the field lands.
pub const ARRIVAL_STREAM_SECS: u64 = 600;

/// The end the field has been landing on: the last ghost touchdown within
/// [`ARRIVAL_STREAM_SECS`], else `None`. The memory only: the live sighting
/// on final ([`arrival_end`]) outranks it through [`field_streams`].
pub fn arrival_stream_end(now: Instant) -> Option<String> {
    ARRIVAL_STREAM
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|(_, at)| now.saturating_duration_since(*at).as_secs() < ARRIVAL_STREAM_SECS)
        .map(|(end, _)| end.clone())
}

/// The streams the field is working, `(arrival, departure)`, as the runway
/// pick, the stated-runway refusal and the re-sequence read them. The
/// arrival is the live lander on final ([`arrival_end`], passed as `live`),
/// else the remembered touchdown ([`arrival_stream_end`]); the departure is
/// the remembered liftoff ([`departure_end`]). When the two point against
/// each other the field has turned round: the OLDER memory is dropped (a
/// live lander is now).
pub fn field_streams(
    runways: &[nav::Runway],
    live: Option<String>,
    now: Instant,
) -> (Option<String>, Option<String>) {
    let fresh = |at: &Instant| now.saturating_duration_since(*at).as_secs();
    let landed = ARRIVAL_STREAM
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
        .filter(|(_, at)| fresh(at) < ARRIVAL_STREAM_SECS);
    let departed = DEPARTURE_STREAM
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
        .filter(|(_, at)| fresh(at) < DEPARTURE_STREAM_SECS);
    let arrival = match live {
        Some(end) => Some((end, now)),
        None => landed,
    };
    match (arrival, departed) {
        (Some((a, at_a)), Some((d, at_d))) if ends_oppose(runways, &a, &d) => {
            if at_a >= at_d {
                (Some(a), None)
            } else {
                (None, Some(d))
            }
        }
        (a, d) => (a.map(|x| x.0), d.map(|x| x.0)),
    }
}

/// Does this flight id carry an airline prefix (ICAO 3-letter or IATA
/// 2-letter + digit)? The wake heuristic's "medium or heavier".
pub(super) fn is_airline(id: &str) -> bool {
    // `get`, never a byte-index slice: the id comes off raw TCAS bytes via
    // from_utf8_lossy, and slicing a replacement character mid-way panics.
    let id = id.trim();
    (id.len() > 3
        && id
            .get(..3)
            .is_some_and(|pfx| TELEPHONY.iter().any(|(p, _)| *p == pfx)))
        || (id.len() > 2
            && id.as_bytes()[2].is_ascii_digit()
            && id
                .get(..2)
                .is_some_and(|pfx| TELEPHONY_IATA.iter().any(|(p, _)| *p == pfx)))
}

/// One pass of the wake watch: a ghost transitioning ground→air near the
/// field starts the two-minute clock on its strip. Called by the monitor on
/// the same TCAS read as the other traffic watches.
pub fn wake_note_targets(targets: &[TcasTarget], now: Instant) {
    let Some((_, flat, flon, _)) = active_field() else {
        return;
    };
    // The standing clocks ride every traffic read (`GHOST_STALE_S`).
    note_ground_traffic(targets, now);
    let mut guard = WAKE_PREV.lock().unwrap_or_else(|e| e.into_inner());
    let mem = guard.get_or_insert_with(std::collections::HashMap::new);
    mem.retain(|_, p| now.duration_since(p.seen).as_secs() < 300);
    for t in targets {
        if crate::nav::haversine_nm(t.lat, t.lon, flat, flon) > 3.0 {
            continue;
        }
        let here = WakePrev {
            on_ground: t.on_ground,
            seen: now,
            lat: t.lat,
            lon: t.lon,
        };
        let prev = *mem.entry(t.callsign.clone()).or_insert(here);
        mem.insert(t.callsign.clone(), here);
        let was_ground = prev.on_ground;
        // Any liftoff says which way the field departs, airline or not: the
        // end by the roll direction, as the party line names it.
        if was_ground && !t.on_ground {
            if let Some(end) = departure_runway_ident(t.lat, t.lon, prev.lat, prev.lon) {
                let end = main_runway_of(&end);
                tracing::info!(ghost = %t.callsign, runway = %end, "departure stream");
                *DEPARTURE_STREAM.lock().unwrap_or_else(|e| e.into_inner()) = Some((end, now));
            }
        }
        // Any touchdown ON a runway says which way the field lands: the end
        // by its track from the last airborne sample (issue #164).
        if !was_ground && t.on_ground {
            let on_pavement = ACTIVE_RUNWAYS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .iter()
                .any(|r| on_runway(r, t.lat, t.lon).is_some());
            if on_pavement {
                if let Some(end) = departure_runway_ident(t.lat, t.lon, prev.lat, prev.lon) {
                    let end = main_runway_of(&end);
                    tracing::info!(ghost = %t.callsign, runway = %end, "arrival stream");
                    let changed = ARRIVAL_STREAM
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .as_ref()
                        .is_none_or(|(e, at)| {
                            *e != end
                                || now.saturating_duration_since(*at).as_secs()
                                    >= ARRIVAL_STREAM_SECS
                        });
                    if changed {
                        sim::blackbox::record(
                            "atc",
                            serde_json::json!({"event": "arrival_stream", "ghost": t.callsign,
                                "runway": end}),
                        );
                    }
                    *ARRIVAL_STREAM.lock().unwrap_or_else(|e| e.into_inner()) = Some((end, now));
                }
            }
        }
        if was_ground && !t.on_ground && is_airline(&t.callsign) {
            // The END it departed, by the roll, on the main strip of a close
            // parallel pair: the nearest threshold of the nearest runway was
            // EGKK's 26R (the 08L pavement) for an 08R departure, and the
            // held 08R release ignored the clock (2026-09-28, #155).
            if let Some(ident) = departure_runway_ident(t.lat, t.lon, prev.lat, prev.lon)
                .or_else(|| nearest_runway_ident(t.lat, t.lon))
                .map(|id| main_runway_of(&id))
            {
                tracing::info!(ghost = %t.callsign, runway = %ident, "wake clock started");
                *WAKE.lock().unwrap_or_else(|e| e.into_inner()) = Some(Wake {
                    since: now,
                    runway_ident: ident,
                });
            }
        }
    }
}

/// Seconds of wake separation still to run behind the last departure from
/// THIS strip (end ident). `None` = clock expired, other strip, or no
/// departure seen.
pub fn wake_remaining(runway_ident: &str, now: Instant) -> Option<u64> {
    let guard = WAKE.lock().unwrap_or_else(|e| e.into_inner());
    let w = guard.as_ref()?;
    let elapsed = now.duration_since(w.since).as_secs();
    if elapsed >= WAKE_SECS {
        return None;
    }
    let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
    same_strip_or_twin(&runways, &w.runway_ident, runway_ident).then_some(WAKE_SECS - elapsed)
}
