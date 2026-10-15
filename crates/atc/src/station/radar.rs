//! Radar services: the squawk watch, the departure handoff, vectors to final.

use super::*;

/// This flight's discrete squawk, deterministic from the callsign so tests
/// and a restarted session reproduce it. Codes live in the 45xx block with
/// octal last digits — every code valid, and the block is clear of all
/// reserved codes (0000, 1200, 2000, 7000, 7500, 7600, 7700).
pub fn discrete_squawk(callsign: &str) -> i64 {
    let seed: u32 = callsign.bytes().map(u32::from).sum();
    4500 + ((seed / 8) % 8) as i64 * 10 + (seed % 8) as i64
}

/// The radar watch on an assigned squawk: armed when a controller issues a
/// discrete code, ticked by the monitor. "Radar contact" is spoken ONCE the
/// sim actually shows the code squawking — until then it's an instruction,
/// not theatre — and while it doesn't, the controller nudges ("confirm
/// squawk ...") a couple of times, then gives up chasing like a real one.
pub struct SquawkWatch {
    pub code: i64,
    pub(super) callsign: String,
    pub(super) speaker: StationSpeaker,
    /// "Radar contact" has been spoken; the watch is satisfied.
    pub(super) observed: bool,
    /// A departure handoff is armed and the flight has not checked in: the
    /// check-in's reply says "radar contact", so the watch holds it.
    pub(super) check_in_owed: bool,
    /// First tick after assignment — starts the grace period.
    pub(super) first_seen: Option<Instant>,
    pub(super) last_nudge: Option<Instant>,
    pub(super) nudges: u32,
}

/// Knob-spinning time before the first nudge.
pub(super) const SQUAWK_GRACE_SECS: f64 = 20.0;
/// Quiet time between nudges.
pub(super) const SQUAWK_NUDGE_SECS: f64 = 45.0;
/// Then the controller stops chasing.
pub(super) const SQUAWK_MAX_NUDGES: u32 = 2;

/// Cheap gate for the monitor: only read the transponder datarefs while a
/// watch is armed.
pub fn squawk_watch_active() -> bool {
    OWN_STRIP.squawk_watch
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .is_some_and(|w| !w.observed)
}

/// Can this position say "radar contact"? Only a radar unit: a tower works
/// by eye, and never relays another unit's contact.
pub(super) fn radar_unit(s: StationSpeaker) -> bool {
    matches!(s, StationSpeaker::Approach | StationSpeaker::Departure)
}

/// The departure check-in's reply said "radar contact": the watch is
/// satisfied, and says nothing more.
pub(super) fn radar_contact_at_check_in(strip: &FlightStrip) {
    if let Some(w) = strip.squawk_watch
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_mut()
    {
        w.observed = true;
        w.check_in_owed = false;
    }
}

/// One tick of the radar watch: the actual transponder code, whether the
/// mode is replying (ON/ALT), and who COM1 is tuned to. Returns what to
/// speak and who says it.
///
/// "Radar contact" is the RADAR unit's, on its own frequency: tuned to the
/// tower the watch holds it (a tower does not relay it), and with a
/// departure handoff armed it is the check-in's reply, never the watch's.
/// LEMG 31 (sortie 1790584715, 2026-09-28): a departure heard "radar
/// contact" in the tower's voice 4.3 s after rotation (t 1790585265.4,
/// route squawk_watch), and again from Departure at the check-in. The
/// nudges ("confirm squawk") are still relayed: they are an instruction.
pub fn squawk_tick(
    actual: Option<i64>,
    mode_on: bool,
    tuned: StationSpeaker,
    now: Instant,
) -> Option<(String, StationSpeaker)> {
    squawk_tick_for(&OWN_STRIP, actual, mode_on, tuned, now)
}

/// [`squawk_tick`] for one flight's strip: its watch, its nudge. No
/// aircraft state is read (the transponder is not in it).
pub(super) fn squawk_tick_for(
    flight: &FlightStrip,
    actual: Option<i64>,
    mode_on: bool,
    tuned: StationSpeaker,
    now: Instant,
) -> Option<(String, StationSpeaker)> {
    let mut guard = flight.squawk_watch.lock().unwrap_or_else(|e| e.into_inner());
    let w = guard.as_mut()?;
    let first = *w.first_seen.get_or_insert(now);
    if actual == Some(w.code) && mode_on {
        if w.observed || !radar_unit(tuned) || w.check_in_owed {
            return None;
        }
        w.observed = true;
        return Some((format!("{}, radar contact.", w.callsign), w.speaker));
    }
    // Once contact was called, a later mode change isn't chased — the real
    // frequency stays quiet about it unless it matters (future: radar
    // contact lost on a long mismatch).
    if w.observed {
        return None;
    }
    if now.duration_since(first).as_secs_f64() < SQUAWK_GRACE_SECS
        || w.nudges >= SQUAWK_MAX_NUDGES
        || w.last_nudge
            .is_some_and(|t| now.duration_since(t).as_secs_f64() < SQUAWK_NUDGE_SECS)
    {
        return None;
    }
    w.last_nudge = Some(now);
    w.nudges += 1;
    // Two different problems, two different nudges: the code dialed right
    // with the transponder in standby got "confirm squawk 4503" (Gatwick,
    // 2026-07-13) — a nag that reads as "your digits are wrong" when the
    // digits are the one thing that's right.
    let recycle = actual == Some(w.code);
    *flight.squawk_nudge.lock().unwrap_or_else(|e| e.into_inner()) = Some(crate::party_line::SquawkNudge {
        code: w.code,
        recycle,
    });
    let line = if recycle {
        format!(
            "{}, negative radar contact, recycle transponder.",
            w.callsign
        )
    } else {
        format!(
            "{}, confirm squawk {}.",
            w.callsign,
            atis::spoken_digits(&w.code.to_string())
        )
    };
    Some((line, w.speaker))
}

/// The tower's after-takeoff handoff, armed by a takeoff clearance and spoken
/// once airborne — see [`Station::arm_departure_handoff`] and [`handoff_tick`].
pub struct Handoff {
    /// The full transmission, prebuilt ("… contact Gibraltar Departure on …").
    pub line: String,
    /// Speak once climbing through this altitude (ft MSL, field + 700).
    pub min_alt_ft: f64,
    /// The station handed to, as spoken ("Gibraltar Departure"), and its
    /// frequency — published for the crew the moment the line goes out.
    pub to: String,
    pub khz: i64,
}

/// One tick of the after-takeoff handoff, ~monitor rate: silence until the
/// flight is airborne and through the arming altitude, then the handoff
/// exactly once.
pub fn handoff_tick(alt_ft: f64, on_ground: bool) -> Option<String> {
    let state = AircraftState {
        alt_ft,
        ..AircraftState::at_position(f64::NAN, f64::NAN, on_ground, f64::NAN)
    };
    handoff_tick_for(&OWN_STRIP, &state)
}

/// [`handoff_tick`] for one flight's strip: its armed handoff, published on
/// its strip as it goes out.
pub(super) fn handoff_tick_for(flight: &FlightStrip, state: &AircraftState) -> Option<String> {
    let mut guard = flight.handoff.lock().unwrap_or_else(|e| e.into_inner());
    let plan = guard.as_ref()?;
    if state.on_ground || state.alt_ft < plan.min_alt_ft {
        return None;
    }
    let line = plan.line.clone();
    // The crew that works the radio gets the instruction as data, not just
    // as audio: a readback, a retune and a check-in are owed, and until this
    // was published nothing on the crew side could do them — the Malaga
    // departure stayed on tower for thirteen minutes (12:23Z, 2026-09-11).
    *flight.departure_handoff.lock().unwrap_or_else(|e| e.into_inner()) =
        Some((plan.to.clone(), plan.khz));
    *guard = None;
    Some(line)
}

/// An armed vectors-to-final plan — the approach controller's worksheet. Set
/// by [`Station::arm_vectors`], driven by [`vector_tick`] from the monitor's
/// poll loop (the same shape as the talk-down's `approach_guide::GUIDE`).
pub struct VectorPlan {
    pub callsign: String,
    pub thr_lat: f64,
    pub thr_lon: f64,
    /// Final approach course, degrees true.
    pub course_deg: f64,
    pub runway_spoken: String,
    /// Tower frequency, spoken — the handoff at the end of the vectors.
    pub tower_spoken: Option<String>,
    /// Intercept altitude, ft MSL.
    pub target_alt_ft: f64,
    /// ILS mode: the runway has a localizer, so the vectors end in "cleared
    /// ILS approach, report established" instead of the visual clearance.
    pub ils: bool,
    /// Where the vectors aim the final join, nm from the threshold. Visual:
    /// the stock 6. ILS: far enough out that the glideslope is still ABOVE
    /// the platform altitude there — intercepting from below is the whole
    /// discipline; a fixed 6 nm join under Heathrow's 2,500 ft platform
    /// delivered the aircraft on top of the slope.
    pub(super) join_nm: f64,
    /// The ILS clearance has been spoken (it rides the intercept turn, once).
    pub(super) cleared: bool,
    /// Phraseology dialect, carried from the station that armed the plan.
    pub(super) dialect: Dialect,
    pub(super) last_tx: Option<Instant>,
    pub(super) last_heading: Option<i64>,
}

impl VectorPlan {
    /// The approach clearance has been spoken, so the vectors are finished and
    /// the crew owes the "established" report.
    pub(crate) fn is_cleared(&self) -> bool {
        self.cleared
    }
    /// The heading last actually TRANSMITTED, not an internal target — a crew
    /// complying with vectors must fly the number it was given. `None` before
    /// the first instruction goes out.
    pub(crate) fn last_heading_deg(&self) -> Option<i64> {
        self.last_heading
    }
}

/// How far out the vectors aim you onto final (the join point) for a VISUAL
/// approach; an ILS join is computed from its platform instead.
pub(super) const JOIN_FINAL_NM: f64 = 6.0;

/// The ILS join distance: where a `gs_deg` slope reaches `platform_agl_ft`
/// above the field, plus a mile of level flight to capture from below —
/// never inside the visual join. Pure, so the geometry unit-tests.
pub(super) fn ils_join_nm(platform_agl_ft: f64, gs_deg: f64) -> f64 {
    let slope_nm = platform_agl_ft.max(0.0) / (6076.12 * gs_deg.max(1.0).to_radians().tan());
    (slope_nm + 1.0).max(JOIN_FINAL_NM)
}

/// One tick of the approach controller, ~monitor rate. Returns an instruction
/// when one is due: a fresh heading when the required course has moved, the
/// approach clearance + tower handoff once established, silence otherwise.
/// Pure narration — the pilot flies. `track_deg` is the ground track (degrees
/// true, `None` = read failed); `now` is injected so the cadence unit-tests.
pub fn vector_tick(
    lat: f64,
    lon: f64,
    alt_ft: f64,
    track_deg: Option<f64>,
    on_ground: bool,
    now: Instant,
) -> Option<String> {
    let state = AircraftState {
        alt_ft,
        ..AircraftState::at_position(lat, lon, on_ground, f64::NAN)
    };
    vector_tick_for(&OWN_STRIP, &state, track_deg, now)
        .or_else(|| approach_watch_tick_for(&OWN_STRIP, &state, track_deg))
}

/// Established on the final course, the one test the vectors' handoff and
/// the approach watch's share. Beyond a mile: inside the join (plus two),
/// within 15 degrees of the course from the threshold and flying it.
/// Inside a mile the bearing is too twitchy, so landing intent instead:
/// below the platform, pointed down the runway (see `vector_tick_for`,
/// Gibraltar 2026-07-13). Pure.
pub(super) fn established_on_course(
    dist_thr_nm: f64,
    off_course_deg: f64,
    aligned: bool,
    alt_ft: f64,
    target_alt_ft: f64,
    join_nm: f64,
) -> bool {
    if dist_thr_nm >= 1.0 {
        dist_thr_nm <= join_nm + 2.0 && off_course_deg <= 15.0 && aligned
    } else {
        alt_ft < target_alt_ft && off_course_deg <= 45.0 && aligned
    }
}

/// Approach's worksheet for an INSTRUMENT arrival it cleared on its own
/// navigation ([`Station::clear_instrument_approach`]): no headings, only
/// the watch for the moment it is established, when approach hands it to
/// the tower. Armed only for our own strip (the traffic flies its own
/// handoff).
pub struct ApproachWatch {
    pub(super) callsign: String,
    pub(super) thr_lat: f64,
    pub(super) thr_lon: f64,
    /// Final approach course, degrees true.
    pub(super) course_deg: f64,
    pub(super) target_alt_ft: f64,
    pub(super) join_nm: f64,
    pub(super) runway_spoken: String,
    pub(super) ils: bool,
    /// The tower handed to: its spoken name ("Gatwick Tower"), its
    /// frequency spoken in the dialect, and in kHz.
    pub(super) tower: (String, String, i64),
    /// The approach clearance, held while the platform cannot be reached by
    /// the join ([`platform_reachable`]): the check-in got the descent and
    /// "expect", the tick says this once the height can be lost. `None`:
    /// cleared.
    pub(super) pending: Option<ApproachWords>,
}

/// Can an arrival at `alt_ft`, `to_join_nm` from the join, be down to the
/// platform there on no more than a 3 degree path? A height that did not
/// read is no reason to hold the clearance. Pure. EGKK-EGBB (2026-10-15,
/// sortie 1792081624-full_auto): Birmingham Approach cleared the ILS 15
/// "descend 2,500" at 9,800 ft, 7.8 nm from the join.
pub(super) fn platform_reachable(alt_ft: f64, target_alt_ft: f64, to_join_nm: f64) -> bool {
    !(alt_ft - target_alt_ft > nav::GLIDE_FT_PER_NM * to_join_nm)
}

/// From (`lat`, `lon`) to the join: `join_nm` out from the threshold on the
/// extended centreline of a final flown on `course_deg`, nm.
pub(super) fn to_join_nm(
    lat: f64,
    lon: f64,
    thr: (f64, f64),
    course_deg: f64,
    join_nm: f64,
) -> f64 {
    let (jlat, jlon) = nav::project_latlon(thr.0, thr.1, (course_deg + 180.0) % 360.0, join_nm);
    nav::haversine_nm(lat, lon, jlat, jlon)
}

/// One tick of the approach watch: silence until the flight is seen
/// established on the final course (the vectors' own test,
/// [`established_on_course`]), then "contact Tower" once, published for the
/// crew ([`crate::party_line::take_arrival_handoff`]) and for the
/// landing-clearance coach. A pilot who never reports established is still
/// handed over at the join, about eight to eleven miles out on an ILS, where
/// a real director lets an established arrival go. Landed (or armed on the
/// ground) is no approach: the watch ends silent.
pub(super) fn approach_watch_tick_for(
    flight: &FlightStrip,
    state: &AircraftState,
    track_deg: Option<f64>,
) -> Option<String> {
    let mut guard = flight.approach_watch.lock().unwrap_or_else(|e| e.into_inner());
    let w = guard.as_mut()?;
    if state.on_ground {
        *guard = None;
        retire_approach_speed(flight);
        return None;
    }
    let (lat, lon) = state.position()?;
    if w.pending.is_some() {
        // Not cleared yet: the clearance once the height can be lost by the
        // join, and never a handoff before it.
        let to_join = to_join_nm(lat, lon, (w.thr_lat, w.thr_lon), w.course_deg, w.join_nm);
        if platform_reachable(state.alt_ft, w.target_alt_ft, to_join) {
            let words = w.pending.take()?;
            let mut rb = flight
                .approach_readback
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            *rb = Some(match rb.take() {
                Some(prev) => format!("{prev}, {}", words.readback),
                None => words.readback,
            });
            drop(rb);
            sim::blackbox::record(
                "atc",
                serde_json::json!({
                    "event": "approach_cleared", "runway": w.runway_spoken, "held": true,
                    "alt_ft": state.alt_ft.round(), "to_join_nm": (to_join * 10.0).round() / 10.0,
                }),
            );
            return Some(words.line);
        }
        let dialect = crate::station::active_icao().map_or(Dialect::Icao, |(icao, ..)| {
            crate::station::dialect_for(&icao)
        });
        let callsign = w.callsign.clone();
        let extra =
            approach_sequence_lines(flight, state, track_deg, None, dialect, Instant::now());
        return with_sequence(&callsign, None, extra);
    }
    let dist_thr = crate::nav::haversine_nm(lat, lon, w.thr_lat, w.thr_lon);
    let brg_thr = crate::nav::bearing_deg(lat, lon, w.thr_lat, w.thr_lon);
    let off_course = sim::angles::abs_diff_deg(brg_thr, w.course_deg);
    let aligned = track_deg.is_none_or(|t| sim::angles::abs_diff_deg(t, w.course_deg) <= 30.0);
    if !established_on_course(dist_thr, off_course, aligned, state.alt_ft, w.target_alt_ft, w.join_nm)
    {
        // Not yet established: the sequence and the speed (2026-10-14).
        let dialect = crate::station::active_icao()
            .map_or(Dialect::Icao, |(icao, ..)| crate::station::dialect_for(&icao));
        let extra =
            approach_sequence_lines(flight, state, track_deg, None, dialect, Instant::now());
        return with_sequence(&w.callsign, None, extra);
    }
    retire_approach_speed(flight);
    let (name, spoken, khz) = w.tower.clone();
    let line = format!("{}, contact {name} {spoken}.", w.callsign);
    *flight.arrival_handoff.lock().unwrap_or_else(|e| e.into_inner()) = Some(ArrivalHandoff {
        to: name,
        khz,
        to_tower: true,
        missed: false,
        runway_spoken: w.runway_spoken.clone(),
        ils: w.ils,
    });
    *flight.landing_handoff.lock().unwrap_or_else(|e| e.into_inner()) =
        Some(w.runway_spoken.clone());
    sim::blackbox::record(
        "atc",
        serde_json::json!({
            "event": "approach_handoff", "why": "seen_established", "to_khz": khz,
            "dist_nm": (dist_thr * 10.0).round() / 10.0, "runway": w.runway_spoken,
        }),
    );
    *guard = None;
    Some(line)
}

/// [`vector_tick`] for one flight's strip: its vectors plan and its
/// approach handoff. A position that did not read is no position, as before
/// (every distance and bearing from it is NaN).
pub(super) fn vector_tick_for(
    flight: &FlightStrip,
    state: &AircraftState,
    track_deg: Option<f64>,
    now: Instant,
) -> Option<String> {
    let (lat, lon) = state.position().unwrap_or((f64::NAN, f64::NAN));
    let alt_ft = state.alt_ft;
    let mut guard = flight.vectors.lock().unwrap_or_else(|e| e.into_inner());
    let plan = guard.as_mut()?;
    if state.on_ground {
        *guard = None; // landed (or armed on the ramp by mistake): vectors over
        retire_approach_speed(flight);
        return None;
    }

    let dist_thr = crate::nav::haversine_nm(lat, lon, plan.thr_lat, plan.thr_lon);
    let brg_thr = crate::nav::bearing_deg(lat, lon, plan.thr_lat, plan.thr_lon);
    // Established: on the approach side, on the extended centerline, inside the
    // join distance, AND actually flying the final course — the position wedge
    // alone would clear an aircraft crossing final at ninety degrees. A failed
    // track read doesn't block the clearance (best-effort, like every read).
    let off_course = sim::angles::abs_diff_deg(brg_thr, plan.course_deg);
    let aligned = track_deg.is_none_or(|t| sim::angles::abs_diff_deg(t, plan.course_deg) <= 30.0);
    // Established, two regimes. Beyond a mile: the position wedge — on the
    // approach side, near the extended centerline, inside the join, flying
    // the course. Inside a mile the threshold bearing is too twitchy for a
    // 15-degree wedge, and a tight base that rolls out on final at 0.9 nm
    // never satisfied the old 1.0 nm floor — the plan vectored a short-final
    // Cessna AWAY ("fly heading zero niner zero" at 272 feet, toward the
    // join point behind it) and touchdown then discarded the plan with no
    // clearance ever spoken, handing a compliant pilot the landing berate
    // (Gibraltar, 2026-07-13). Inside the mile the test is landing intent:
    // below the vector altitude, pointed down the runway, threshold still
    // broadly ahead.
    let established =
        established_on_course(dist_thr, off_course, aligned, alt_ft, plan.target_alt_ft, plan.join_nm);
    if established {
        let handoff = plan
            .tower_spoken
            .as_ref()
            .map(|f| format!(" Contact Tower {f}."))
            .unwrap_or_default();
        // A short-final clearance rounds to "1 mile", never "0 miles".
        let miles = (dist_thr.round() as i64).max(1);
        let miles = format!("{miles} mile{}", if miles == 1 { "" } else { "s" });
        let msg = if plan.ils && plan.cleared {
            // Cleared on the intercept, now seen established: hand off. (If
            // the pilot reports established first, respond_approach does
            // this and the plan is already gone.)
            format!("{}, {miles} from touchdown.{handoff}", plan.callsign)
        } else if plan.ils {
            // Straight-in that never needed an intercept turn: clearance and
            // handoff together.
            format!(
                "{}, {miles} from touchdown. Cleared ILS approach runway {}.{handoff}",
                plan.callsign, plan.runway_spoken
            )
        } else {
            format!(
                "{}, {miles} from touchdown. Cleared visual approach runway {}.{handoff}",
                plan.callsign, plan.runway_spoken
            )
        };
        // Handed to the tower with an APPROACH clearance — the pilot still owes
        // a "final" report before the tower's landing clearance. Arm the crew
        // coach so a missed report doesn't become a landing-without-clearance
        // (the compliance watch clears this on the landing clearance and on
        // touchdown). Only when there's a tower to report to.
        if plan.tower_spoken.is_some() {
            *flight.landing_handoff.lock().unwrap_or_else(|e| e.into_inner()) =
                Some(plan.runway_spoken.clone());
        }
        // Handed to the tower: approach's speed is over.
        retire_approach_speed(flight);
        *guard = None; // approach's job is done
        return Some(msg);
    }

    // The sequence and the speed (2026-10-14): said with the heading when
    // one is due, alone when it is not.
    let dialect = plan.dialect;
    let extra = approach_sequence_lines(flight, state, track_deg, Some(&mut *plan), dialect, now);

    // Inside a mile and not (yet) landing: no heading. Any vector from here
    // aims at a join point BEHIND the aircraft — a reversal at circuit
    // height. Stay quiet and let the geometry develop; if they blow through
    // final, steering resumes beyond the mile.
    if dist_thr < 1.0 {
        return with_sequence(&plan.callsign, None, extra);
    }

    // Steering, two regimes. Near the extended centerline on the approach
    // side: ONE honest intercept — a 30° cut toward the final course,
    // relaxing to the course itself as the centerline arrives. This used to
    // be a pursuit curve (correction ∝ cross-track); against a real pilot's
    // turn lag and the 25 s call cadence the gain wound up, marched the
    // headings 200→250, blew through the localizer, and then re-vectored a
    // reversal to the fallen-behind join point — an orbit, flown live at
    // Jerez 2026-07-12. Controllers give a cut, not a curve. Anywhere else:
    // direct to the join point, which is the downwind/base leg for free.
    let along_nm = dist_thr * off_course.to_radians().cos(); // + = approach side
    let xtk_nm = dist_thr * sim::angles::signed_diff_deg(brg_thr, plan.course_deg).to_radians().sin();
    let capturing = along_nm >= 1.0 && xtk_nm.abs() <= 2.5;
    // On the intercept after the ILS clearance, TRACKING the course: the
    // frequency goes quiet. Capturing the localizer is the pilot's job now;
    // a controller who keeps issuing headings against a coupling autopilot
    // fights it ("cleared ILS" followed by "fly heading 360", taped
    // 2026-07-11). But quiet needs BOTH near the centerline AND actually
    // flying its direction — a pilot sailing through the localizer on the
    // cut heading still gets the correction.
    if plan.ils && plan.cleared && capturing && xtk_nm.abs() <= 1.5 && aligned {
        return with_sequence(&plan.callsign, None, extra);
    }
    let desired = if capturing {
        if xtk_nm.abs() <= 0.4 {
            plan.course_deg
        } else {
            (plan.course_deg + 35.0_f64.copysign(xtk_nm)).rem_euclid(360.0)
        }
    } else {
        // Aim BEYOND the join, not at it: the last leg then converges onto
        // the centerline's axis with room for the 35° cut to finish. Aiming
        // AT the join arrives perpendicular with no room, drops out of the
        // capture band at the bottom, and the re-vector back to a join now
        // BEHIND the aircraft is the other half of the Jerez orbit.
        let recip = (plan.course_deg + 180.0).rem_euclid(360.0);
        let (join_lat, join_lon) =
            crate::nav::project_latlon(plan.thr_lat, plan.thr_lon, recip, plan.join_nm + 2.0);
        crate::nav::bearing_deg(lat, lon, join_lat, join_lon)
    };
    let heading10 = {
        let h = ((desired / 10.0).round() as i64 * 10).rem_euclid(360);
        if h == 0 {
            360
        } else {
            h
        }
    };

    // Cadence: speak immediately the first time, then only when the required
    // heading has genuinely moved — a real drift after 25 s, a big cut after
    // 8 s. Holding the assigned heading stays silent, like a real frequency.
    // The drift floor is 20°: headings are rounded to 10°, so a 10° "move" is
    // one quantization step — re-vectoring it every 25 s trimmed a stable
    // downwind 070→080→070 (taped 2026-07-12). A controller who keeps issuing
    // 10-degree corrections is chasing the aircraft; a real one picks a
    // heading that works and lets the geometry develop (the 35° intercept cut
    // always clears the 30° fast rule, so the turn onto final is never held).
    let elapsed = plan
        .last_tx
        .map(|t| now.duration_since(t).as_secs_f64())
        .unwrap_or(f64::MAX);
    let moved = plan
        .last_heading
        .map(|h| sim::angles::abs_diff_deg(h as f64, heading10 as f64))
        .unwrap_or(360.0);
    let due = plan.last_tx.is_none()
        || (elapsed >= 25.0 && moved >= 20.0)
        || (elapsed >= 8.0 && moved >= 30.0);
    if !due {
        return with_sequence(&plan.callsign, None, extra);
    }
    plan.last_tx = Some(now);
    plan.last_heading = Some(heading10);

    let mut msg = format!(
        "{}, fly heading {}{}",
        plan.callsign,
        atis::spoken_digits(&format!("{heading10:03}")),
        // CAP 413 headings carry the unit; FAA's don't.
        if plan.dialect == Dialect::Icao {
            " degrees"
        } else {
            ""
        }
    );
    if alt_ft > plan.target_alt_ft + 400.0 {
        msg.push_str(&match plan.dialect {
            Dialect::Icao => {
                format!(
                    ", descend to altitude {} feet",
                    spoken_alt(plan.target_alt_ft)
                )
            }
            Dialect::Faa => format!(", descend and maintain {}", spoken_alt(plan.target_alt_ft)),
        });
    }
    msg.push('.');
    // The ILS clearance rides the intercept turn, once — the heading that
    // captures the localizer is the one that comes with "cleared ILS".
    if plan.ils && !plan.cleared && capturing {
        plan.cleared = true;
        msg.push_str(&match plan.dialect {
            Dialect::Icao => format!(
                " Cleared ILS approach runway {}, report established.",
                plan.runway_spoken
            ),
            // The FAA PTAC: position was the vector, this is turn-altitude-
            // clearance in one breath.
            Dialect::Faa => format!(
                " Maintain {} until established, cleared ILS runway {} approach.",
                spoken_alt(plan.target_alt_ft),
                plan.runway_spoken
            ),
        });
    }
    with_sequence(&plan.callsign, Some(msg), extra)
}
