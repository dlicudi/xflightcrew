//! Strip occupancy, go-around orders, re-sequencing, the landing hold.

use super::*;

/// Recent positions of ghosts on the ground, by callsign: how a strip's
/// occupant is MOVING (rolling, vacating, standing) is what says whether it
/// will still be there when we arrive.
pub(super) type GroundTracks =
    std::collections::HashMap<String, std::collections::VecDeque<(f64, f64, Instant)>>;
pub(super) static STRIP_SEEN: Mutex<Option<GroundTracks>> = Mutex::new(None);

/// How much track history the judgements keep.
pub(super) const TRACK_KEEP_S: f64 = 8.0;
/// A velocity is measured over at least this long (a tick-to-tick
/// difference is noise).
pub(super) const TRACK_MIN_DT_S: f64 = 1.0;

/// Record our own position for the go-around judgement. Airborne only: on
/// the ground there is no threshold to reach.
pub(crate) fn note_own_fix(lat: f64, lon: f64, on_ground: bool, now: Instant) {
    let mut f = OWN_STRIP.own_fixes.lock().unwrap_or_else(|e| e.into_inner());
    if on_ground {
        f.clear();
        return;
    }
    if f.back()
        .is_some_and(|b| now.saturating_duration_since(b.2).as_secs_f64() < 0.05)
    {
        return;
    }
    f.push_back((lat, lon, now));
    while f
        .front()
        .is_some_and(|a| now.saturating_duration_since(a.2).as_secs_f64() > TRACK_KEEP_S)
    {
        f.pop_front();
    }
}

/// The tower decides a go-around only inside this time to the threshold: a
/// runway occupant further out than that is a sequencing matter ("continue
/// approach"), never a cancellation. About two miles at approach speed.
pub(super) const GO_AROUND_DECIDE_S: f64 = 60.0;
/// The runway must be clear this long before we cross the threshold.
pub(super) const RUNWAY_CLEAR_MARGIN_S: f64 = 10.0;
/// Groundspeed assumed on final when ours is not measured.
pub(super) const DEFAULT_FINAL_KTS: f64 = 140.0;
/// A departure's roll: it lifts off near this speed, accelerating at this
/// rate (a jet from brake release lifts in about 35 s).
pub(super) const LIFTOFF_KTS: f64 = 150.0;
pub(super) const ROLL_ACCEL_KTS_S: f64 = 4.0;
/// A lander ahead rolling out (fast, not accelerating) is off at a turn-off
/// within this.
pub(super) const ROLLOUT_CLEAR_S: f64 = 40.0;
/// A ghost standing still this close to the threshold we land on is a
/// departure lined up: it is off the runway a roll after it starts.
pub(super) const LINED_UP_ZONE_M: f64 = 800.0;
/// Under this a ghost is standing, not moving.
pub(super) const MOVING_KTS: f64 = 3.0;
/// An occupant is ON the runway when its reported position is within the
/// paved half-width plus this: a mainwheel on the edge line, never the
/// holding point (90 m out on a code C strip). The runway judgement asks the
/// PAVEMENT, not the incursion watch's wider capture (`on_runway`, 10 m of
/// slack and 2 % of the length past each end). Issue #162.
pub(super) const OCCUPANT_EDGE_M: f64 = 5.0;
/// Past either END of the pavement an occupant still counts this far: a
/// displaced threshold or an overrun, not a blast pad or a taxiway on the
/// extended centreline (2 % of LEMG's 3,200 m was 64 m).
pub(super) const OCCUPANT_END_M: f64 = 30.0;
/// A ghost within this of where it stopped is still standing (the feed's
/// position noise, a nudge forward in the line-up).
pub(super) const STANDING_RADIUS_M: f64 = 15.0;
/// A ghost standing on a runway longer than this is stale data, not an
/// occupant: no real aircraft holds a runway for five minutes, a frozen
/// LiveTraffic target does. Logged `ghost_stale` once per standing spell.
/// Issue #162.
pub(super) const GHOST_STALE_S: f64 = 300.0;
/// A standing clock survives this long without a sighting (the traffic read
/// is only made while a watch needs it); a ghost seen again where it stood
/// has stood all along.
pub(super) const GHOST_MEMORY_S: f64 = 900.0;
/// Another runway offered after a go-around must be at least this share of
/// the blocked strip's length: the tower knows the runway the flight chose,
/// not the aircraft's numbers, so it never offers a much shorter one.
pub(super) const RESEQUENCE_MIN_LENGTH_RATIO: f64 = 0.75;

/// How a runway occupant will leave the strip, judged from its track.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum StripExit {
    /// Moving off it: a departure rolling our way, the lander ahead rolling
    /// out, an aircraft crossing or vacating. Off in about this many seconds.
    Clearing(f64),
    /// Standing at the threshold we land on: a departure lined up, off one
    /// roll (this many seconds) after it starts.
    LinedUp(f64),
    /// Standing elsewhere, or unknown: no telling when.
    Stays,
    /// Moving TOWARD us along the strip: traffic using the other direction
    /// (a lander rolling out, a departure rolling, a backtrack). No telling
    /// when it is off, and the runway's direction is contested: the
    /// re-sequence after the go-around offers the other end
    /// ([`resequence_runway`]).
    Oncoming,
}

impl StripExit {
    fn secs(&self) -> Option<f64> {
        match self {
            StripExit::Clearing(s) | StripExit::LinedUp(s) => Some(*s),
            StripExit::Stays | StripExit::Oncoming => None,
        }
    }
}

/// Seconds for a departure rolling at `kts` to lift off.
pub(super) fn roll_to_liftoff_s(kts: f64) -> f64 {
    (LIFTOFF_KTS - kts).max(0.0) / ROLL_ACCEL_KTS_S
}

/// Half the strip's width for occupancy: the pavement plus
/// [`OCCUPANT_EDGE_M`]. A row without a width keeps the blanket capture.
pub(super) fn strip_half_width_m(r: &nav::Runway) -> f64 {
    if r.width_m > 1.0 {
        r.width_m / 2.0 + OCCUPANT_EDGE_M
    } else {
        RUNWAY_HALF_WIDTH_M
    }
}

/// Is this position ON the runway's pavement, for the landing judgement?
/// Laterally within [`strip_half_width_m`] of the centreline, lengthwise
/// between the ends with [`OCCUPANT_END_M`] past each. Returns (metres from
/// the `ident_a` end, metres off the centreline). Issue #162: "on the
/// runway" is the pavement, never the strip's area or its hold line.
pub(crate) fn on_runway_pavement(r: &nav::Runway, lat: f64, lon: f64) -> Option<(f64, f64)> {
    // Scaled at end A's latitude.
    let f = nav::strip_frame(r, lat, lon, r.lat_a)?;
    let (len, along, cross) = (f.len_m(), f.along_m(), f.cross_m());
    ((-OCCUPANT_END_M..=len + OCCUPANT_END_M).contains(&along) && cross <= strip_half_width_m(r))
        .then_some((along, cross))
}

/// Where a ghost on the ground stopped, and since when: the stale-ghost
/// clock ([`GHOST_STALE_S`]).
pub(super) struct Standing {
    lat: f64,
    lon: f64,
    since: Instant,
    seen: Instant,
    /// `ghost_stale` was recorded for this standing spell.
    stale_logged: bool,
}

pub(super) static STANDING: Mutex<Option<std::collections::HashMap<String, Standing>>> = Mutex::new(None);

/// Flat-earth metres between two nearby positions.
pub(super) fn metres_between(lat0: f64, lon0: f64, lat1: f64, lon1: f64) -> f64 {
    let m_lon = 111_320.0 * lat0.to_radians().cos();
    let (dx, dy) = ((lon1 - lon0) * m_lon, (lat1 - lat0) * 111_320.0);
    (dx * dx + dy * dy).sqrt()
}

/// Note every ghost's position: the short tracks the runway judgement reads
/// its motion from (airborne too, so a lander's first sample on the wheels
/// already has a velocity: at first sight a touchdown was "standing"), and
/// each ground ghost's standing clock. Idempotent within a tick; fed by
/// every watch that reads the traffic.
pub(super) fn note_ground_traffic(targets: &[TcasTarget], now: Instant) {
    {
        let mut guard = STRIP_SEEN.lock().unwrap_or_else(|e| e.into_inner());
        let seen = guard.get_or_insert_with(std::collections::HashMap::new);
        for t in targets {
            let tr = seen.entry(t.callsign.clone()).or_default();
            if tr
                .back()
                .is_none_or(|b| now.saturating_duration_since(b.2).as_secs_f64() >= 0.05)
            {
                tr.push_back((t.lat, t.lon, now));
            }
            while tr
                .front()
                .is_some_and(|a| now.saturating_duration_since(a.2).as_secs_f64() > TRACK_KEEP_S)
            {
                tr.pop_front();
            }
        }
        seen.retain(|_, tr| {
            tr.back()
                .is_some_and(|b| now.saturating_duration_since(b.2).as_secs_f64() <= TRACK_KEEP_S)
        });
    }
    let mut guard = STANDING.lock().unwrap_or_else(|e| e.into_inner());
    let standing = guard.get_or_insert_with(std::collections::HashMap::new);
    for t in targets {
        if !t.on_ground {
            standing.remove(&t.callsign);
            continue;
        }
        let fresh = Standing {
            lat: t.lat,
            lon: t.lon,
            since: now,
            seen: now,
            stale_logged: false,
        };
        match standing.get_mut(&t.callsign) {
            Some(s)
                if metres_between(s.lat, s.lon, t.lat, t.lon) <= STANDING_RADIUS_M
                    && now.saturating_duration_since(s.seen).as_secs_f64() <= GHOST_MEMORY_S =>
            {
                s.seen = s.seen.max(now);
            }
            _ => {
                standing.insert(t.callsign.clone(), fresh);
            }
        }
    }
    standing.retain(|_, s| now.saturating_duration_since(s.seen).as_secs_f64() <= GHOST_MEMORY_S);
}

/// How long this ghost has stood where it is, seconds (0 when moving or
/// unknown).
pub(super) fn standing_secs(callsign: &str, now: Instant) -> f64 {
    STANDING
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .and_then(|m| m.get(callsign))
        .map(|s| now.saturating_duration_since(s.since).as_secs_f64())
        .unwrap_or(0.0)
}

/// Is this ghost stale data: standing on a runway past [`GHOST_STALE_S`]?
/// The first time per standing spell it is recorded (`ghost_stale`).
pub(super) fn ghost_stale(t: &TcasTarget, runway: &str, place: (f64, f64), now: Instant) -> bool {
    let mut guard = STANDING.lock().unwrap_or_else(|e| e.into_inner());
    let Some(s) = guard.as_mut().and_then(|m| m.get_mut(&t.callsign)) else {
        return false;
    };
    let secs = now.saturating_duration_since(s.since).as_secs_f64();
    if secs < GHOST_STALE_S {
        return false;
    }
    if !s.stale_logged {
        s.stale_logged = true;
        sim::blackbox::record(
            "atc",
            serde_json::json!({"event": "ghost_stale", "ghost": t.callsign, "runway": runway,
                "along_m": place.0.round(), "cross_m": place.1.round(),
                "standing_s": secs.round(), "lat": t.lat, "lon": t.lon}),
        );
    }
    true
}

/// A ghost that OCCUPIES this runway for the landing judgement: on the
/// ground, on the pavement ([`on_runway_pavement`]), and not stale data
/// ([`ghost_stale`]). Returns its place on the pavement.
pub(super) fn occupies(r: &nav::Runway, t: &TcasTarget, now: Instant) -> Option<(f64, f64)> {
    if !t.on_ground {
        return None;
    }
    let place = on_runway_pavement(r, t.lat, t.lon)?;
    (!ghost_stale(t, &r.ident_a, place, now)).then_some(place)
}

/// A frame on the landing END: origin its threshold, `u` along the landing
/// direction; positions in metres, along and to the left.
pub(super) struct LandingFrame {
    pub(super) lat0: f64,
    pub(super) lon0: f64,
    pub(super) m_lat: f64,
    pub(super) m_lon: f64,
    pub(super) u: (f64, f64),
}

impl LandingFrame {
    pub(super) fn new(r: &nav::Runway, landing_end: &str) -> Option<Self> {
        let (lat0, lon0, lat1, lon1) = if r.ident_a == landing_end {
            (r.lat_a, r.lon_a, r.lat_b, r.lon_b)
        } else if r.ident_b == landing_end {
            (r.lat_b, r.lon_b, r.lat_a, r.lon_a)
        } else {
            return None;
        };
        let m_lat = 111_320.0;
        let m_lon = 111_320.0 * lat0.to_radians().cos();
        let (bx, by) = ((lon1 - lon0) * m_lon, (lat1 - lat0) * m_lat);
        let len = (bx * bx + by * by).sqrt();
        if len < 1.0 {
            return None;
        }
        Some(Self {
            lat0,
            lon0,
            m_lat,
            m_lon,
            u: (bx / len, by / len),
        })
    }

    /// Metres east and north of the threshold.
    fn xy(&self, lat: f64, lon: f64) -> (f64, f64) {
        (
            (lon - self.lon0) * self.m_lon,
            (lat - self.lat0) * self.m_lat,
        )
    }

    /// (along, cross) of a position, metres.
    pub(super) fn place(&self, lat: f64, lon: f64) -> (f64, f64) {
        let (x, y) = self.xy(lat, lon);
        (x * self.u.0 + y * self.u.1, -x * self.u.1 + y * self.u.0)
    }
}

/// The velocity (along, cross; m/s) between sample `newest` and the latest
/// one at least [`TRACK_MIN_DT_S`] older, with that older sample's index.
pub(super) fn track_velocity(
    f: &LandingFrame,
    track: &[(f64, f64, f64)],
    newest: usize,
) -> Option<((f64, f64), usize)> {
    let (lat, lon, t) = track[newest];
    let i = (0..newest)
        .rev()
        .find(|&i| t - track[i].2 >= TRACK_MIN_DT_S)?;
    let (a1, c1) = f.place(lat, lon);
    let (a0, c0) = f.place(track[i].0, track[i].1);
    let dt = t - track[i].2;
    Some((((a1 - a0) / dt, (c1 - c0) / dt), i))
}

/// How a ghost on our strip will leave it, from its track (lat, lon, seconds;
/// oldest first, the last is now). Pure: the go-around judgement's and the
/// landing release's one question.
pub(crate) fn strip_exit(
    r: &nav::Runway,
    landing_end: &str,
    track: &[(f64, f64, f64)],
) -> StripExit {
    let Some(f) = LandingFrame::new(r, landing_end) else {
        return StripExit::Stays;
    };
    let Some(&(lat, lon, _)) = track.last() else {
        return StripExit::Stays;
    };
    let (along, cross) = f.place(lat, lon);
    let standing = || {
        if along < LINED_UP_ZONE_M {
            StripExit::LinedUp(roll_to_liftoff_s(0.0))
        } else {
            StripExit::Stays
        }
    };
    let Some(((va, vc), older)) = track_velocity(&f, track, track.len() - 1) else {
        // First sight: standing until it is seen to move.
        return standing();
    };
    let kts = (va * va + vc * vc).sqrt() * 1.943_844;
    if kts < MOVING_KTS {
        return standing();
    }
    let angle = vc.abs().atan2(va.abs()).to_degrees();
    if angle <= 30.0 {
        if va <= 0.0 {
            // Toward us along the strip: a backtrack, or the other direction.
            return StripExit::Oncoming;
        }
        // Our way along the strip: a departure rolling, or the lander ahead
        // rolling out. Slow or accelerating is a roll; fast and slowing is a
        // rollout, off at a turn-off.
        let accel = track_velocity(&f, track, older).map(|((pa, pc), _)| {
            let prev_kts = (pa * pa + pc * pc).sqrt() * 1.943_844;
            (kts - prev_kts) / (track[track.len() - 1].2 - track[older].2)
        });
        return if kts < 40.0 || accel.is_some_and(|a| a >= 0.5) {
            StripExit::Clearing(roll_to_liftoff_s(kts))
        } else {
            StripExit::Clearing(ROLLOUT_CLEAR_S)
        };
    }
    // Across the strip: crossing or vacating, off at the edge it heads for.
    let half = strip_half_width_m(r);
    let dist = if vc.signum() == cross.signum() {
        half - cross.abs()
    } else {
        half + cross.abs()
    };
    StripExit::Clearing(dist.max(0.0) / vc.abs())
}

/// Our distance (nm) and time (s) to the landing threshold from the fixes the
/// monitor noted. `None` without a fresh fix (a test, the first tick).
pub(super) fn own_time_to_threshold(r: &nav::Runway, landing_end: &str, now: Instant) -> Option<(f64, f64)> {
    let f = LandingFrame::new(r, landing_end)?;
    let fixes = OWN_STRIP.own_fixes.lock().unwrap_or_else(|e| e.into_inner());
    let &(lat, lon, at) = fixes.back()?;
    if now.saturating_duration_since(at).as_secs_f64() > 3.0 {
        return None;
    }
    let (x, y) = f.xy(lat, lon);
    let dist_m = (x * x + y * y).sqrt();
    let gs_kts = fixes
        .iter()
        .rev()
        .find(|p| at.saturating_duration_since(p.2).as_secs_f64() >= TRACK_MIN_DT_S)
        .map(|p| {
            let dt = at.saturating_duration_since(p.2).as_secs_f64();
            crate::nav::haversine_nm(p.0, p.1, lat, lon) / dt * 3600.0
        })
        .filter(|g| *g >= 40.0)
        .unwrap_or(DEFAULT_FINAL_KTS);
    Some((dist_m / 1852.0, dist_m / (gs_kts / 1.943_844)))
}

/// The go-around is due: we are inside [`GO_AROUND_DECIDE_S`] of the
/// threshold and some occupant will not be off the runway
/// [`RUNWAY_CLEAR_MARGIN_S`] before we cross it. Further out it is only
/// sequencing: a departure lined up gets its roll, a vacating arrival its
/// turn-off. An unmeasured time decides as before: the order.
pub(crate) fn go_around_due(own_s: Option<f64>, exits: &[StripExit]) -> bool {
    if exits.is_empty() {
        return false;
    }
    let Some(t) = own_s else {
        return true;
    };
    t <= GO_AROUND_DECIDE_S
        && exits
            .iter()
            .any(|e| e.secs().is_none_or(|s| s + RUNWAY_CLEAR_MARGIN_S > t))
}

/// A withheld landing clearance may go out: the strip is clear, or every
/// occupant is MOVING off it and will be gone [`RUNWAY_CLEAR_MARGIN_S`]
/// before we arrive (a departure rolling, an arrival vacating). A lined-up
/// or standing aircraft never earns it.
pub(crate) fn landing_release_due(own_s: Option<f64>, exits: &[StripExit]) -> bool {
    exits.iter().all(|e| match (e, own_s) {
        (StripExit::Clearing(s), Some(t)) => s + RUNWAY_CLEAR_MARGIN_S <= t,
        _ => false,
    })
}

/// One runway occupant as the landing judgement saw it: who, how it will
/// leave, and WHERE it was (the blackbox's evidence: issue #162's ghosts
/// were judged with no position on file).
#[derive(Debug, Clone)]
pub(super) struct Occupant {
    callsign: String,
    pub(super) exit: StripExit,
    /// Metres past our landing threshold along the landing direction, and
    /// off the centreline (left positive). `None` without our strip.
    along_m: Option<f64>,
    cross_m: Option<f64>,
    /// How long it has stood where it is.
    standing_s: f64,
}

/// The ghosts OCCUPYING our strip (on its pavement, not stale:
/// [`occupies`]) and how each will leave it; notes every ghost's position
/// for the tracks and the standing clocks first.
pub(super) fn strip_exits(
    targets: &[TcasTarget],
    ours: Option<&nav::Runway>,
    landing_end: &str,
    now: Instant,
) -> Vec<Occupant> {
    note_ground_traffic(targets, now);
    let runways: Vec<nav::Runway> = match ours {
        Some(r) => vec![r.clone()],
        None => ACTIVE_RUNWAYS
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone(),
    };
    let frame = ours.and_then(|r| LandingFrame::new(r, landing_end));
    let guard = STRIP_SEEN.lock().unwrap_or_else(|e| e.into_inner());
    targets
        .iter()
        .filter(|t| runways.iter().any(|r| occupies(r, t, now).is_some()))
        .map(|t| {
            let exit = match (ours, guard.as_ref().and_then(|g| g.get(&t.callsign))) {
                (Some(r), Some(tr)) => {
                    let track: Vec<(f64, f64, f64)> = tr
                        .iter()
                        .map(|p| (p.0, p.1, -now.saturating_duration_since(p.2).as_secs_f64()))
                        .collect();
                    strip_exit(r, landing_end, &track)
                }
                _ => StripExit::Stays,
            };
            let place = frame.as_ref().map(|f| f.place(t.lat, t.lon));
            Occupant {
                callsign: t.callsign.clone(),
                exit,
                along_m: place.map(|p| p.0),
                cross_m: place.map(|p| p.1),
                standing_s: standing_secs(&t.callsign, now),
            }
        })
        .collect()
}

/// The last "go-around deferred" record: the judgement is logged at a
/// cadence, not every tick.
pub(super) static GO_AROUND_DEFER_LOGGED: Mutex<Option<Instant>> = Mutex::new(None);

/// The occupants a runway judgement used, for the blackbox: how each leaves
/// and where it stood.
pub(super) fn exits_json(exits: &[Occupant]) -> serde_json::Value {
    serde_json::Value::Array(
        exits
            .iter()
            .map(|o| {
                let (kind, s) = match o.exit {
                    StripExit::Clearing(s) => ("clearing", Some(s)),
                    StripExit::LinedUp(s) => ("lined_up", Some(s)),
                    StripExit::Stays => ("stays", None),
                    StripExit::Oncoming => ("oncoming", None),
                };
                serde_json::json!({"ghost": o.callsign, "exit": kind,
                    "clear_s": s.map(|s| (s * 10.0).round() / 10.0),
                    "along_m": o.along_m.map(f64::round),
                    "cross_m": o.cross_m.map(f64::round),
                    "standing_s": o.standing_s.round()})
            })
            .collect(),
    )
}

/// Our numbers for the blackbox: distance (nm) and time (s) to the threshold.
pub(super) fn own_json(own: Option<(f64, f64)>) -> (serde_json::Value, serde_json::Value) {
    (
        serde_json::json!(own.map(|o| (o.0 * 100.0).round() / 100.0)),
        serde_json::json!(own.map(|o| o.1.round())),
    )
}

/// The tower protects the one aircraft it controls: cleared to land (or held
/// for the landing clearance), low, and a ghost on the runway that will still
/// be there when we arrive: the clearance is cancelled out loud instead of
/// letting the world contradict it. Crew phrasing when the pilot isn't tuned
/// (a safety net must not depend on the radio). Judged on TIME: inside
/// [`GO_AROUND_DECIDE_S`] of the threshold, against how the occupant is
/// leaving ([`strip_exit`]). LEMG 13 (sortie 1790594471-full_auto,
/// 2026-09-28): ordered at 3.8 nm, about 80 s out, for the departure TO4611
/// that lifted off 81 s later.
pub fn go_around_tick(
    targets: &[TcasTarget],
    own_agl_ft: f64,
    on_ground: bool,
    tuned: bool,
    now: Instant,
) -> Option<ComplianceCall> {
    // The standing clocks run through the whole final, not only its last
    // 800 ft ([`GHOST_STALE_S`]).
    note_ground_traffic(targets, now);
    if on_ground || own_agl_ft > 800.0 {
        return None;
    }
    let cleared = OWN_STRIP.clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .filter(|c| c.state == tower::State::ClearedToLand)
        .map(|c| (c.callsign.clone(), c.runway_ident.clone()));
    let held = || {
        OWN_STRIP.landing_hold
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .map(|h| (h.callsign.clone(), h.runway_ident.clone()))
    };
    let (callsign, runway_ident) = cleared.or_else(held)?;
    // Only a ghost on OUR strip cancels the clearance — a parallel runway's
    // occupant is not our conflict (Heathrow, 2026-07-10).
    let ours = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        strip(&runways, &runway_ident).cloned()
    };
    let exits = strip_exits(targets, ours.as_ref(), &runway_ident, now);
    if exits.is_empty() {
        return None;
    }
    let own = ours
        .as_ref()
        .and_then(|r| own_time_to_threshold(r, &runway_ident, now));
    let plain: Vec<StripExit> = exits.iter().map(|e| e.exit).collect();
    let (own_nm, own_s) = own_json(own);
    if !go_around_due(own.map(|o| o.1), &plain) {
        let mut logged = GO_AROUND_DEFER_LOGGED
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if logged.is_none_or(|t| now.saturating_duration_since(t).as_secs_f64() >= 10.0) {
            *logged = Some(now);
            sim::blackbox::record(
                "atc",
                serde_json::json!({"event": "go_around_deferred", "runway": runway_ident,
                    "own_nm": own_nm, "own_s": own_s, "occupants": exits_json(&exits)}),
            );
        }
        return None;
    }
    let mut last = OWN_STRIP.go_around_last.lock().unwrap_or_else(|e| e.into_inner());
    if last.is_some_and(|t| now.duration_since(t).as_secs_f64() < 30.0) {
        return None;
    }
    *last = Some(now);
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "go_around_judged", "runway": runway_ident,
            "own_nm": own_nm, "own_s": own_s, "occupants": exits_json(&exits)}),
    );
    // A withheld landing is over: the order is the answer it was waiting for.
    *OWN_STRIP.landing_hold.lock().unwrap_or_else(|e| e.into_inner()) = None;
    // The order voids the clearance in the FSM too, or the coach keeps
    // suggesting "cleared to land ..." as the readback owed while the tower
    // is shouting go around (seen live, 2026-07-14). The sync is handed to
    // the main loop, which owns the `Station`, same as the hold release.
    *OWN_STRIP.pending_go_around.lock().unwrap_or_else(|e| e.into_inner()) = true;
    // An occupant that will not leave on its own clock (standing, or using
    // the other direction) would send the next approach round the same way:
    // the tower re-sequences now (issue #162). Only a flight on the
    // frequency can be told.
    let expect = if tuned {
        resequence_after_go_around(targets, &runway_ident, &exits, now)
    } else {
        None
    };
    // The runway the order names rides with it, published BEFORE its count
    // moves, so a crew watch that sees the order sees its runway (#182).
    *OWN_STRIP.resequenced_order.lock().unwrap_or_else(|e| e.into_inner()) = expect.clone().map(|to| {
        (
            GO_AROUND_ORDERS.load(std::sync::atomic::Ordering::SeqCst) + 1,
            to,
        )
    });
    // ...and the order itself, for every crew loop that flies it.
    GO_AROUND_ORDERS.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    Some(if tuned {
        let expect = expect
            .map(|to| format!("expect runway {}, ", atis::spoken_runway(&to)))
            .unwrap_or_default();
        ComplianceCall {
            line: format!(
                "{callsign}, go around, traffic on the runway. Climb to circuit \
                 altitude, {expect}report downwind."
            ),
            crew: false,
        }
    } else {
        ComplianceCall {
            line: "Traffic on the runway ahead. Go around.".to_string(),
            crew: true,
        }
    })
}

/// The wind the tower last worked with (the field's surface wind when it
/// had one), for the re-sequence the go-around watch makes between calls.
pub(super) static TOWER_WIND: Mutex<Option<(f64, f64)>> = Mutex::new(None);

/// The runway the tower offers an arrival sent round by an occupant that
/// will not leave on its own clock, or why none. Pure. `oncoming` (traffic
/// using the other direction on our strip) offers the RECIPROCAL first,
/// landing with that stream; a standing occupant blocks its whole strip, so
/// only another strip will do. Every candidate: not the blocked end, not a
/// strip with a fresh occupant (`occupied`, by `ident_a`), not against a
/// live stream (`streams`, and the oncoming traffic's own end), at least
/// [`RESEQUENCE_MIN_LENGTH_RATIO`] of the blocked strip's length, and a
/// wind that does not argue ([`stated_runway_stands`]). Ranked: a stream's
/// own end, then the smallest turn from the direction we were landing, then
/// the headwind.
pub(crate) fn resequence_runway(
    runways: &[nav::Runway],
    blocked_end: &str,
    oncoming: bool,
    occupied: &[String],
    streams: &[String],
    wind: Option<(f64, f64)>,
) -> Result<String, &'static str> {
    let Some(blocked) = strip(runways, blocked_end) else {
        return Err("unknown_runway");
    };
    let Some(landing_hdg) = end_heading(runways, blocked_end) else {
        return Err("unknown_runway");
    };
    let Some((wind_from, wind_kts)) = wind else {
        return Err("no_wind");
    };
    let len = |r: &nav::Runway| {
        let m_lon = 111_320.0 * r.lat_a.to_radians().cos();
        let (dx, dy) = ((r.lon_b - r.lon_a) * m_lon, (r.lat_b - r.lat_a) * 111_320.0);
        (dx * dx + dy * dy).sqrt()
    };
    let blocked_len = len(blocked);
    let reciprocal = if blocked.ident_a == blocked_end {
        blocked.ident_b.clone()
    } else {
        blocked.ident_a.clone()
    };
    let mut streams: Vec<String> = streams.to_vec();
    if oncoming {
        streams.push(reciprocal);
    }
    let best = nav::best_runway_for_wind(runways, wind_from, wind_kts);
    let mut picks: Vec<(u8, f64, f64, String)> = Vec::new();
    for r in runways {
        for end in [&r.ident_a, &r.ident_b] {
            if end == blocked_end {
                continue;
            }
            let same_strip = r.ident_a == blocked.ident_a;
            if same_strip && !oncoming {
                continue;
            }
            if !same_strip && occupied.contains(&r.ident_a) {
                continue;
            }
            if len(r) < RESEQUENCE_MIN_LENGTH_RATIO * blocked_len {
                continue;
            }
            if streams.iter().any(|s| ends_oppose(runways, s, end)) {
                continue;
            }
            let hw = nav::headwind_for(runways, end, wind_from, wind_kts);
            if !stated_runway_stands(runways, end, hw, best.clone(), wind_from, wind_kts) {
                continue;
            }
            let Some(hdg) = end_heading(runways, end) else {
                continue;
            };
            let stream_rank = u8::from(!streams.iter().any(|s| s == end));
            picks.push((
                stream_rank,
                sim::angles::abs_diff_deg(landing_hdg, hdg),
                -hw.unwrap_or(0.0),
                end.clone(),
            ));
        }
    }
    picks.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then(a.1.total_cmp(&b.1))
            .then(a.2.total_cmp(&b.2))
    });
    picks
        .into_iter()
        .next()
        .map(|p| p.3)
        .ok_or("no_other_runway")
}

/// The go-around watch's re-sequence: when an occupant of our strip will not
/// leave on its own clock (standing, or oncoming), pick the runway to offer
/// ([`resequence_runway`]) and hand it to the FSM sync; else the arrival is
/// held on its runway (the next final report is withheld while the strip is
/// occupied and released when it clears). Recorded `resequenced` either way.
pub(super) fn resequence_after_go_around(
    targets: &[TcasTarget],
    runway_ident: &str,
    exits: &[Occupant],
    now: Instant,
) -> Option<String> {
    let oncoming = exits.iter().any(|o| o.exit == StripExit::Oncoming);
    let standing = exits.iter().any(|o| o.exit == StripExit::Stays);
    if !oncoming && !standing {
        return None;
    }
    let runways = ACTIVE_RUNWAYS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone();
    let occupied: Vec<String> = runways
        .iter()
        .filter(|r| targets.iter().any(|t| occupies(r, t, now).is_some()))
        .map(|r| r.ident_a.clone())
        .collect();
    let field = active_field();
    let (arrival, departure) = field_streams(
        &runways,
        arrival_end(
            targets,
            &runways,
            field.as_ref().map(|f| f.3).unwrap_or(0.0),
        ),
        now,
    );
    let streams: Vec<String> = arrival.iter().chain(departure.iter()).cloned().collect();
    let wind = surface_wind_at(field.as_ref().map(|f| (f.1, f.2)))
        .or(*TOWER_WIND.lock().unwrap_or_else(|e| e.into_inner()));
    let pick = resequence_runway(&runways, runway_ident, oncoming, &occupied, &streams, wind);
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "resequenced", "from": runway_ident,
            "to": pick.as_ref().ok(), "hold": pick.is_err(),
            "why": if oncoming { "oncoming" } else { "standing" },
            "refused": pick.as_ref().err(), "occupied": occupied,
            "arrival_end": arrival, "departure_end": departure,
            "wind_from": wind.map(|w| w.0.round()),
            "wind_kts": wind.map(|w| (w.1 * 10.0).round() / 10.0),
            "occupants": exits_json(exits)}),
    );
    let to = pick.ok()?;
    *OWN_STRIP.pending_resequence.lock().unwrap_or_else(|e| e.into_inner()) =
        Some((runway_ident.to_string(), to.clone()));
    Some(to)
}

/// A landing clearance withheld for traffic on the runway: armed by the reply
/// to a final report ("continue approach, traffic on the runway"), released
/// by [`landing_hold_tick`].
pub struct LandingHold {
    pub(super) callsign: String,
    /// Spoken, for the readback ("one three").
    pub(super) runway: String,
    /// The END we land on ("13").
    pub(super) runway_ident: String,
    /// The clearance as it will be spoken (the wind of the final report).
    pub(super) line: String,
}

/// One pass of the withheld-landing watch: once our strip is clear, or its
/// occupant is moving off it in time ([`landing_release_due`]), the tower
/// comes back unprompted with the landing clearance. The published clearance
/// moves to ClearedToLand at once (the crew reads the readback owed); the
/// station's FSM syncs before it answers the next call
/// ([`Station::sync_landing_release`]). Landed, the hold is moot: dropped.
/// The go-around order, when the traffic stays, is [`go_around_tick`]'s.
pub fn landing_hold_tick(targets: &[TcasTarget], on_ground: bool, now: Instant) -> Option<String> {
    let mut guard = OWN_STRIP.landing_hold.lock().unwrap_or_else(|e| e.into_inner());
    let hold = guard.as_ref()?;
    if on_ground || station_unavailable() {
        *guard = None;
        return None;
    }
    let ours = {
        let runways = ACTIVE_RUNWAYS.lock().unwrap_or_else(|e| e.into_inner());
        strip(&runways, &hold.runway_ident).cloned()
    };
    let exits = strip_exits(targets, ours.as_ref(), &hold.runway_ident, now);
    let own = ours
        .as_ref()
        .and_then(|r| own_time_to_threshold(r, &hold.runway_ident, now));
    let plain: Vec<StripExit> = exits.iter().map(|e| e.exit).collect();
    if !landing_release_due(own.map(|o| o.1), &plain) {
        return None;
    }
    let hold = guard.take()?;
    let (own_nm, own_s) = own_json(own);
    sim::blackbox::record(
        "atc",
        serde_json::json!({"event": "landing_released", "runway": hold.runway_ident,
            "own_nm": own_nm, "own_s": own_s, "occupants": exits_json(&exits)}),
    );
    if let Some(c) = OWN_STRIP.clearance.lock().unwrap_or_else(|e| e.into_inner()).as_mut() {
        c.state = tower::State::ClearedToLand;
        c.holds_runway = true;
        c.runway_ident = hold.runway_ident.clone();
        c.pending_readback = Some(format!(
            "Cleared to land runway {}, {}.",
            hold.runway, hold.callsign
        ));
    }
    *OWN_STRIP.landing_handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.pending_landing_release
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = Some((hold.runway.clone(), hold.line.clone()));
    Some(hold.line)
}

#[cfg(test)]
mod strip_frame_tests {
    use super::*;

    /// The two strip tests before they shared `nav::strip_frame`, verbatim:
    /// the shared frame must give each the same answer to the bit.
    fn old_on_runway(r: &nav::Runway, lat: f64, lon: f64) -> Option<String> {
        let m_per_deg_lat = 111_320.0;
        let m_per_deg_lon = 111_320.0 * lat.to_radians().cos();
        let (px, py) = (
            (lon - r.lon_a) * m_per_deg_lon,
            (lat - r.lat_a) * m_per_deg_lat,
        );
        let (bx, by) = (
            (r.lon_b - r.lon_a) * m_per_deg_lon,
            (r.lat_b - r.lat_a) * m_per_deg_lat,
        );
        let len2 = bx * bx + by * by;
        if len2 < 1.0 {
            return None;
        }
        let t = (px * bx + py * by) / len2;
        let cross_m = (px * by - py * bx).abs() / len2.sqrt();
        let half_width = if r.width_m > 1.0 {
            r.width_m / 2.0 + RUNWAY_EDGE_SLACK_M
        } else {
            RUNWAY_HALF_WIDTH_M
        };
        if (-0.02..=1.02).contains(&t) && cross_m <= half_width {
            let ident = if t < 0.5 { &r.ident_a } else { &r.ident_b };
            return Some(ident.clone());
        }
        None
    }

    fn old_on_pavement(r: &nav::Runway, lat: f64, lon: f64) -> Option<(f64, f64)> {
        let m_lat = 111_320.0;
        let m_lon = 111_320.0 * r.lat_a.to_radians().cos();
        let (px, py) = ((lon - r.lon_a) * m_lon, (lat - r.lat_a) * m_lat);
        let (bx, by) = ((r.lon_b - r.lon_a) * m_lon, (r.lat_b - r.lat_a) * m_lat);
        let len = (bx * bx + by * by).sqrt();
        if len < 1.0 {
            return None;
        }
        let along = (px * bx + py * by) / len;
        let cross = (px * by - py * bx).abs() / len;
        ((-OCCUPANT_END_M..=len + OCCUPANT_END_M).contains(&along)
            && cross <= strip_half_width_m(r))
        .then_some((along, cross))
    }

    fn rwy(a: &str, b: &str, la: (f64, f64), lb: (f64, f64), width_m: f64) -> nav::Runway {
        nav::Runway {
            ident_a: a.into(),
            ident_b: b.into(),
            lat_a: la.0,
            lon_a: la.1,
            lat_b: lb.0,
            lon_b: lb.1,
            width_m,
            surface: 1,
        }
    }

    #[test]
    fn the_shared_strip_frame_answers_both_tests_to_the_bit() {
        let strips = [
            // LEMG 13/31, 12/30 (no width: the blanket capture), EGKK 08R/26L,
            // a row one end on the other (degenerate).
            rwy("13", "31", (36.6878, -4.5124), (36.6649, -4.4794), 45.0),
            rwy("12", "30", (36.6905, -4.5063), (36.6683, -4.4747), 0.0),
            rwy("08R", "26L", (51.1505, -0.2011), (51.1440, -0.1527), 45.0),
            rwy("09", "27", (60.0, 10.0), (60.0, 10.0), 30.0),
        ];
        let mut on = 0;
        for r in &strips {
            let (la, lo) = ((r.lat_a + r.lat_b) / 2.0, (r.lon_a + r.lon_b) / 2.0);
            for i in -12..=12 {
                for j in -12..=12 {
                    let lat = la + i as f64 * 0.0011;
                    let lon = lo + j as f64 * 0.0013;
                    let new = on_runway(r, lat, lon);
                    assert_eq!(new, old_on_runway(r, lat, lon), "{} {lat} {lon}", r.ident_a);
                    let p = on_runway_pavement(r, lat, lon);
                    let q = old_on_pavement(r, lat, lon);
                    assert_eq!(
                        p.map(|(a, c)| (a.to_bits(), c.to_bits())),
                        q.map(|(a, c)| (a.to_bits(), c.to_bits())),
                        "{} {lat} {lon}",
                        r.ident_a
                    );
                    on += usize::from(new.is_some()) + usize::from(p.is_some());
                }
            }
        }
        assert!(on > 20, "the grid must cross the strips ({on} hits)");
    }
}
