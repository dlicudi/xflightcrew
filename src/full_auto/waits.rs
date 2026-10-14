//! The waits that work the radio: en route, the handoffs, the check-in, the
//! takeoff, the drive, the self taxi and the departure station.

use super::*;

pub(super) async fn engine_running(xp: &XPlaneClient) -> bool {
    xp.get_float_array(ENGN_RUNNING)
        .await
        .ok()
        .and_then(|v| v.first().copied())
        .map(|r| r > 0.5)
        .unwrap_or(false)
}

/// Poll `cond` every `step`, up to `secs`, bailing on ABORT. Returns whether it
/// became true in time.
/// How long a conditional line-up waits for the named arrival to pass before
/// full auto gives the aircraft back at the hold line.
pub(super) const LINE_UP_BEHIND_SECS: f64 = 300.0;
/// Within this of the IFR clearance's altitude the climb has levelled there.
pub(super) const CLEARED_LEVEL_BAND_FT: f64 = 300.0;
/// Level at the IFR clearance's altitude this long with no departure
/// handoff (a tower-only field hands nobody over): the crew climbs as filed
/// rather than cruise the whole flight at the initial altitude.
pub(super) const CLEARED_LEVEL_RELEASE_SECS: f64 = 90.0;

/// The hold at the clearance's altitude has run its course with nobody to
/// clear us higher. Pure.
pub fn cleared_hold_expired(level_secs: f64) -> bool {
    level_secs >= CLEARED_LEVEL_RELEASE_SECS
}

/// How long the inbound call waits for the destination's station to take
/// over the frequency before going out regardless.
pub(super) const INBOUND_SWAP_WAIT_SECS: f64 = 45.0;
/// Between the handoff readback and the retune: the readback has to be off
/// the air on the frequency it was given on before the radio leaves it.
pub(super) const HANDOFF_READBACK_SECS: u64 = 4;
/// The inbound call goes out this far from the destination, once the region
/// has handed us to its own tower: twice the tower's 5 nm zone, so the crew
/// has called before the zone watch can ask who we are. Until 2026-09-15 the
/// call waited for the last enroute fix, and a plan ending at GRA (a mile
/// from Granada's runway) flew the whole zone unannounced: three "say
/// callsign and intentions" in four minutes, none answered.
pub(super) const INBOUND_CALL_NM: f64 = 10.0;
/// The inbound call has been made this mission (by the enroute leg or the
/// arrival), so no later stage makes it twice. Reset at the arm.
pub(super) static INBOUND_MADE: AtomicBool = AtomicBool::new(false);

/// Is the inbound call owed now? Only on the DESTINATION's tower frequency
/// (`home` is whoever the region last handed us to; at LINTO that was
/// Tangier, mid-strait), inside [`INBOUND_CALL_NM`], and not already made. A
/// zone challenge from that tower asks for it regardless of the distance.
pub fn inbound_due(
    dest_khz: Option<i64>,
    home_khz: Option<i64>,
    dist_nm: f64,
    challenged: bool,
    made: bool,
) -> bool {
    if made {
        return false;
    }
    match (dest_khz, home_khz) {
        (Some(d), Some(h)) if d == h => challenged || dist_nm <= INBOUND_CALL_NM,
        _ => false,
    }
}
/// Clear of the pattern: the landing AND taxi lights go off passing this
/// height (the taxi light was lit for the taxi and nothing else puts it out).
pub(super) const LANDING_LIGHTS_OFF_AGL_FT: f64 = 1000.0;

/// The enroute wait, working the radio. The tower's after-takeoff handoff
/// ("contact Malaga Departure on ...") is read back on the frequency it came
/// on, COM1 is retuned, and the departure is checked in with altitude — the
/// exchange nobody made at Malaga, where the flight stayed on tower for
/// thirteen minutes and heard the party line all the way to the region swap
/// (12:23Z, 2026-09-11). The inbound call is made HERE when the region hands
/// us to the destination's own tower and the field is inside
/// [`INBOUND_CALL_NM`], or the moment that tower challenges us (`dest`: the
/// destination's position and tower frequency; `None` for a field with no
/// tower row, where nobody is there to call). Returns true once the enroute
/// leg completes, false on abort or the four-hour ceiling.
pub(super) async fn wait_enroute_working_the_radio(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    callsign: &Option<String>,
    climb_to_ft: f64,
    dest: Option<(f64, f64, i64)>,
    arr: &mut ArrivalRepick,
) -> bool {
    let deadline = Instant::now() + Duration::from_secs_f64(4.0 * 3600.0);
    // Landing and taxi lights off once clear of the pattern: one read per
    // pass until the height is made, then never again this leg.
    let mut landing_off = false;
    // The signs follow the flight through ten thousand, both ways.
    let mut signs = crate::tools::signs::Watch::default();
    // Level at the IFR clearance's altitude since, with nobody yet to clear
    // us higher.
    let mut level_since: Option<Instant> = None;
    loop {
        if ABORT.load(SeqCst) {
            return false;
        }
        if let Ok(alt_ft) = xp.get_float(ALT_IND_FT).await {
            let ceiling = crate::procedure::cleared_ceiling();
            level_since = match ceiling {
                Some(c) if (alt_ft - c).abs() <= CLEARED_LEVEL_BAND_FT => {
                    level_since.or(Some(Instant::now()))
                }
                _ => None,
            };
            if let (Some(c), Some(t)) = (ceiling, level_since) {
                if cleared_hold_expired(t.elapsed().as_secs_f64()) {
                    crate::procedure::hold_at_cleared(None, "no_departure_handoff");
                    level_since = None;
                    event("cleared_ceiling_released", json!({ "ft": c, "why": "no_handoff" }));
                    announce(
                        voice,
                        Priority::Advisory,
                        &format!(
                            "Level at {}, nobody to ask for higher. Climbing as filed.",
                            crate::flightplan::spoken_feet(c)
                        ),
                    );
                }
            }
            if let Some(on) = signs.tick(alt_ft) {
                if crate::tools::signs::set(xp, on).await == Some(true) {
                    announce(
                        voice,
                        Priority::Advisory,
                        if on {
                            "Seat belt signs on."
                        } else {
                            "Seat belt signs off."
                        },
                    );
                }
            }
        }
        if !crate::procedure::running() {
            return crate::procedure::outcome() == Some(sim::run::Outcome::Completed);
        }
        if Instant::now() >= deadline {
            return false;
        }
        if !landing_off {
            let agl_ft = xp.get_float(AGL_M).await.unwrap_or(0.0) * M_TO_FT;
            if agl_ft >= LANDING_LIGHTS_OFF_AGL_FT {
                landing_off = true;
                let did = crate::tools::lights::after_climb(xp).await;
                event(
                    "lights_climb",
                    json!({ "off": did, "agl_ft": agl_ft.round() }),
                );
            }
        }
        if let Some((to, khz)) = crate::party_line::take_departure_handoff() {
            obey_departure_handoff(xp, callsign, to, khz, climb_to_ft).await;
        }
        if let Some(h) = crate::party_line::take_region_handoff() {
            obey_region_handoff(xp, ctx, voice, callsign, h, true).await;
        }
        // The tower's question is drained every pass whoever asked it: an
        // answer is owed the destination's tower (the inbound call, below);
        // a transit field's challenge has no answer yet and is blackboxed.
        let challenged = crate::party_line::take_zone_challenge();
        let home = crate::party_line::home_tower_khz();
        if let Some((dlat, dlon, dkhz)) = dest {
            let on_dest = home == Some(dkhz);
            if challenged && !on_dest {
                event(
                    "zone_challenge",
                    json!({ "answered": false, "home_khz": home, "dest_khz": dkhz }),
                );
            }
            if on_dest && !INBOUND_MADE.load(SeqCst) {
                let dist_nm = match tokio::join!(xp.get_float(LAT), xp.get_float(LON)) {
                    (Ok(lat), Ok(lon)) => crate::nav::haversine_nm(lat, lon, dlat, dlon),
                    _ => f64::INFINITY,
                };
                if inbound_due(Some(dkhz), home, dist_nm, challenged, false) {
                    event(
                        "inbound_enroute",
                        json!({
                            "khz": dkhz, "dist_nm": (dist_nm * 10.0).round() / 10.0,
                            "challenged": challenged,
                        }),
                    );
                    call_inbound(xp, ctx, voice, callsign, dkhz, true, arrival_runway().as_deref())
                        .await;
                }
            }
        } else if challenged {
            event(
                "zone_challenge",
                json!({ "answered": false, "home_khz": home }),
            );
        }
        // The arrival re-picked in flight (#79): the destination's tower
        // naming another end, or its METAR moving to one.
        let tower_rwy = dest
            .filter(|(_, _, dkhz)| home == Some(*dkhz))
            .filter(|_| {
                matches!(
                    crate::party_line::clearance_state(),
                    Some(crate::tower::State::Inbound | crate::tower::State::ClearedToLand)
                )
            })
            .and_then(|_| crate::party_line::clearance_runway());
        let wind_due = arr.last_check.is_none_or(|t| t.elapsed().as_secs_f64() >= REPICK_CHECK_SECS);
        if tower_rwy.is_some() || wind_due {
            let wind_rwy = if wind_due {
                arr.last_check = Some(Instant::now());
                let far = match (arr.dest_pos, tokio::join!(xp.get_float(LAT), xp.get_float(LON))) {
                    (Some((dlat, dlon)), (Ok(lat), Ok(lon))) => {
                        crate::nav::haversine_nm(lat, lon, dlat, dlon) > REPICK_MIN_DEST_NM
                    }
                    _ => false,
                };
                if far {
                    wind_runway(
                        &arr.runways,
                        crate::weather::surface_wind(&crate::config::xplane_root(), &arr.dest_icao),
                    )
                } else {
                    None
                }
            } else {
                None
            };
            let current = arrival_runway();
            if let Some((rwy, why)) = arr.watch.decide(current.as_deref(), tower_rwy.as_deref(), wind_rwy.as_deref()) {
                if let Some(line) = repick_arrival(xp, ctx.config, arr, &rwy, why).await {
                    announce(voice, Priority::Advisory, &line);
                }
            }
        }
        tokio::time::sleep(Duration::from_millis(2000)).await;
    }
}

/// Obey the tower's after-takeoff handoff: read it back on the tower's
/// frequency, retune, check in with the climb. Shared with the helicopter
/// route, which never took one (EDVW, 2026-09-23).
pub(crate) async fn obey_departure_handoff(
    xp: &XPlaneClient,
    callsign: &Option<String>,
    to: String,
    khz: i64,
    climb_to_ft: f64,
) {
    radio_out(crate::crew_radio::radio_call(
        callsign,
        &format!("{to} on {}", crate::atis::spoken_mhz(khz)),
    ));
    tokio::time::sleep(Duration::from_secs(HANDOFF_READBACK_SECS)).await;
    let _ = xp.set_float(COM1_FREQ_KHZ, khz as f64).await;
    // The altitude is read by the main loop as the call goes out, not
    // here: the queue can hold it for twenty seconds behind a busy
    // frequency, and "passing one thousand" was said at 1,207 ft.
    let alt_ft = xp.get_float(ALT_IND_FT).await.unwrap_or(0.0);
    crate::crew_radio::transmit_check_in(callsign.clone(), to.clone(), climb_to_ft);
    // The check-in announces the climb to the plan and departure clears it
    // ("radar contact, climb ..."): the IFR clearance's altitude has done its
    // job, the follower goes on to the plan's levels.
    crate::procedure::hold_at_cleared(None, "departure_check_in");
    event(
        "departure_checkin",
        json!({ "to": to, "khz": khz, "alt_ft_queued": alt_ft.round() }),
    );
}

/// The crew's readback of the enroute handoff, on the frequency it came on:
/// "squawking seven thousand, Malaga Tower on one one eight decimal one".
/// Shaped as a READBACK (the tower FSM's `is_readback` words) so the
/// controller who gave it answers with silence, never "say again".
pub fn region_readback(callsign: &Option<String>, h: &crate::party_line::RegionHandoff) -> String {
    let lead = if h.squawk_vfr {
        "squawking seven thousand"
    } else {
        "roger"
    };
    crate::crew_radio::radio_call(
        callsign,
        &format!("{lead}, {} on {}", h.to, crate::atis::spoken_mhz(h.khz)),
    )
}

/// Obey "radar services terminated, squawk seven thousand, contact X Tower on
/// Y": read it back on the old frequency (when `readback`: the arrival stage
/// has already tuned the destination, and a readback there would land on the
/// NEW tower as a garbled call), give the discrete code back, retune COM1.
/// The spoken line had no crew consumer until 2026-09-14: the flight kept the
/// departure code and the old frequency until its own arrival stage.
///
/// An IFR flight is handed on with its code (`h.ifr_code`, no squawk
/// ordered): nothing is dialled, and the event records the code it kept.
/// LIEA-LEBL, 2026-09-27: an IFR 737 on 4501 set 7000 at FL200.
pub(crate) async fn obey_region_handoff(
    xp: &dyn crate::xplane::XPlane,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    callsign: &Option<String>,
    h: crate::party_line::RegionHandoff,
    readback: bool,
) {
    if readback {
        radio_out(region_readback(callsign, &h));
        tokio::time::sleep(Duration::from_secs(HANDOFF_READBACK_SECS)).await;
    }
    let squawk_ok = if h.squawk_vfr {
        Some(
            crate::tools::radio::SetTransponder
                .execute(&json!({ "code": 7000 }), ctx)
                .await
                .is_ok(),
        )
    } else {
        None
    };
    let tuned = xp.set_float(COM1_FREQ_KHZ, h.khz as f64).await.is_ok();
    // The code the transponder shows after the handoff: 7000 for a VFR
    // service ending, the IFR code unchanged otherwise. A failed read is
    // recorded as null, never a guess.
    let code = xp
        .get_float(crate::datarefs::TRANSPONDER_CODE)
        .await
        .ok()
        .map(|v| v.round() as i64);
    event(
        "region_handoff",
        json!({
            "to": h.to, "khz": h.khz, "squawk_vfr": h.squawk_vfr,
            "squawk_set": squawk_ok, "tuned": tuned, "readback": readback,
            "ifr_code": h.ifr_code, "code": code,
        }),
    );
    if squawk_ok == Some(true) {
        announce(voice, Priority::Advisory, "Seven thousand set.");
    }
}

/// The first call to the departure controller: who we are, where we are,
/// where we're going ("Malaga Departure, November four five X-ray Sierra,
/// passing two thousand climbing four thousand"). Shaped so the station reads
/// it as a check-in (a readback with "passing"/"climbing") and answers "radar
/// contact". With no planned altitude the climb clause is left off rather
/// than invented.
#[cfg(test)]
pub fn departure_check_in(
    callsign: &Option<String>,
    to: &str,
    alt_ft: f64,
    climb_to_ft: f64,
) -> String {
    departure_check_in_above(callsign, to, alt_ft, climb_to_ft, f64::INFINITY)
}

/// [`departure_check_in`] with the flight's transition altitude: a level
/// above it (passed or climbed to) is a flight level ("passing flight level
/// seven zero climbing flight level one five zero"), one at or below it
/// feet, the way the climb is flown on the altimeter (issue #97; the
/// setting rule is `instruments::correct_setting_inhg`'s).
pub fn departure_check_in_above(
    callsign: &Option<String>,
    to: &str,
    alt_ft: f64,
    climb_to_ft: f64,
    transition_ft: f64,
) -> String {
    let who = callsign
        .as_ref()
        .map(|c| format!("{c}, "))
        .unwrap_or_default();
    let said = |ft: f64| {
        if ft > transition_ft {
            crate::atis::spoken_flight_level(ft)
        } else {
            crate::atis::spoken_alt(ft)
        }
    };
    let climb = if climb_to_ft > alt_ft + 200.0 {
        format!(" climbing {}", said(climb_to_ft))
    } else {
        String::new()
    };
    format!("{to}, {who}passing {}{climb}", said(alt_ft))
}

pub(super) async fn wait_until<F, Fut>(secs: f64, step_ms: u64, mut cond: F) -> bool
where
    F: FnMut() -> Fut,
    Fut: std::future::Future<Output = bool>,
{
    let deadline = Instant::now() + Duration::from_secs_f64(secs);
    loop {
        if ABORT.load(SeqCst) {
            return false;
        }
        if cond().await {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        tokio::time::sleep(Duration::from_millis(step_ms)).await;
    }
}

/// The takeoff wait's poll: a departure handoff is read back within this of
/// the tower's line reaching the party line.
pub(super) const TAKEOFF_POLL_MS: u64 = 500;

/// How the takeoff wait ended: `finished` false at the deadline or an abort;
/// `checked_in` when the departure handoff was obeyed inside it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct TakeoffWait {
    pub(super) finished: bool,
    pub(super) checked_in: bool,
}

/// Wait for the takeoff to end, working the radio as it climbs: `obey` takes
/// the tower's departure handoff (readback on the tower, retune, check-in) on
/// the poll it is published, true once obeyed. Until #176 only the enroute
/// wait took it, so the readback waited for the takeoff's handoff (EGKK 08R,
/// sortie 1790624015: given at +1082.9, 960 ft; read back at +1120.2, on a
/// tower that had gone quiet).
pub(super) async fn wait_takeoff_working_the_radio<R, RF, H, HF>(
    secs: f64,
    step_ms: u64,
    mut running: R,
    mut obey: H,
) -> TakeoffWait
where
    R: FnMut() -> RF,
    RF: std::future::Future<Output = bool>,
    H: FnMut() -> HF,
    HF: std::future::Future<Output = bool>,
{
    let deadline = Instant::now() + Duration::from_secs_f64(secs);
    let mut checked_in = false;
    loop {
        if ABORT.load(SeqCst) {
            return TakeoffWait { finished: false, checked_in };
        }
        if !checked_in && obey().await {
            checked_in = true;
        }
        if !running().await {
            return TakeoffWait { finished: true, checked_in };
        }
        if Instant::now() >= deadline {
            return TakeoffWait { finished: false, checked_in };
        }
        tokio::time::sleep(Duration::from_millis(step_ms)).await;
    }
}

/// Wait for the auto drive (taxi, line-up, backtrack) to end. False when
/// it gained no ground along its route for `autotaxi::DRIVE_NO_PROGRESS_S`,
/// ran past `autotaxi::DRIVE_CEILING_S`, or the mission was aborted. Never
/// a flat clock: at LEMG (2026-09-28) the SR22's 4.7 km taxis were given up
/// at 600 s still rolling at 4 to 9 kt with 2 km to go.
pub(super) async fn wait_for_drive() -> bool {
    let t0 = Instant::now();
    let ended = wait_until(crate::autotaxi::DRIVE_CEILING_S, 1000, || async {
        !crate::autotaxi::driving() || crate::autotaxi::drive_stalled()
    })
    .await;
    let still = crate::autotaxi::driving();
    if still && !ABORT.load(SeqCst) {
        event(
            "drive_no_progress",
            json!({ "waited_s": t0.elapsed().as_secs_f64(),
                    "stalled": crate::autotaxi::drive_stalled() }),
        );
    }
    ended && !still
}

/// Is the published taxi route one WE can drive from where we are standing?
///
/// A departure route (not a taxi-in to a stand) whose path passes within
/// 200 m of us. The same test gates the initial check and the wait for a new
/// clearance — they used to differ, and the weaker one accepted the stale
/// route it had just rejected (Edinburgh, 2026-09-20).
/// An uncontrolled departure: the crew plans its own taxi from the field's
/// taxi network to the plan's runway (or the wind's) and says so on the
/// intercom. The route is published like a tower's, so everything after
/// this (the push, the drive, the line-up) is the towered path unchanged.
pub(super) async fn self_taxi(
    xp: &XPlaneClient,
    config: &Config,
    voice: &Option<Voice>,
    tools: &SubTools,
) -> Result<(), String> {
    let (lat_r, lon_r, dir_r, spd_r, psi_r, mtow_r) = tokio::join!(
        xp.get_float(LAT),
        xp.get_float(LON),
        xp.get_float(crate::datarefs::WIND_DIR_DEG_TRUE),
        xp.get_float(crate::datarefs::WIND_SPEED_MSC),
        xp.get_float(crate::autotaxi::PSI_TRUE),
        xp.get_float(crate::datarefs::ACF_M_MAX_KG),
    );
    let (Ok(lat), Ok(lon)) = (lat_r, lon_r) else {
        return Err("I can't read our position to plan the taxi.".to_string());
    };
    let wind = dir_r.ok().zip(spd_r.ok().map(|ms| ms * 1.943_844));
    // The route starts ahead of the nose of an aircraft that cannot turn on
    // the spot (LEMG 2026-10-14, #288): the drive's own rule for who pivots
    // (light, by `acf_m_max`; an unread mass is a heavy one).
    let heavy = mtow_r
        .as_ref()
        .ok()
        .is_none_or(|&m| m > crate::autotaxi::JOIN_TURN_LIGHT_KG);
    let facing = psi_r.ok().filter(|_| heavy);
    let (icao, rwy) = match &tools.mission {
        Mission::Flight(r) | Mission::Resume(r) => (
            Some(r.origin_icao.clone()).filter(|i| !i.is_empty()),
            r.origin_rwy.clone().or_else(|| r.departure_rwy.clone()),
        ),
        _ => (None, None),
    };
    announce(voice, Priority::Advisory, "No tower here. Planning our own taxi.");
    waiting("the taxi plan");
    let apt_path = config.apt_data_path.clone();
    let planned = tokio::task::spawn_blocking(move || {
        crate::party_line::plan_self_taxi(
            &apt_path,
            icao.as_deref(),
            lat,
            lon,
            rwy.as_deref(),
            wind,
            facing,
        )
    })
    .await
    .map_err(|_| "The taxi plan failed.".to_string())??;
    event(
        "self_taxi",
        json!({ "field": planned.field, "runway": planned.runway, "via": planned.via }),
    );
    // The via names stay in the blackbox: an uncontrolled field's apt.dat
    // names are whatever the author typed ("Taxy", "21_hold" at LILB), and
    // no controller has turned them into words.
    let cross = if planned.crossings.is_empty() {
        String::new()
    } else {
        format!(", crossing runway {}", planned.crossings.join(" and "))
    };
    announce(
        voice,
        Priority::Advisory,
        &format!(
            "Taxiing to runway {}{cross}.",
            crate::atis::spoken_runway(&planned.runway)
        ),
    );
    Ok(())
}

/// A running turboprop's condition levers at the profile's run setting
/// (`turboprop_start.condition_run`): the Evolution's is low idle, per its
/// POH, and at high idle it taxied at 17 kt for 10 with full beta and the
/// brakes on (LIME 2026-09-25). A lever at cutoff is an engine that is off:
/// never touched here.
pub(super) async fn condition_to_run(xp: &XPlaneClient) {
    let Some(p) = crate::aircraft::live_profile() else { return };
    let Some(ts) = p.turboprop_start.as_ref() else { return };
    let Ok(levers) = xp.get_float_array(&ts.condition_lever).await else { return };
    for (i, v) in levers.iter().take(p.engines.max(1)).enumerate() {
        if *v > 0.05 && (v - ts.condition_run).abs() > 0.05 {
            let ok = xp
                .set_float_array_elem(&ts.condition_lever, i, ts.condition_run)
                .await
                .is_ok();
            event(
                "condition_run",
                json!({ "engine": i, "was": v, "set": ts.condition_run, "ok": ok }),
            );
        }
    }
}

/// How long the arm waits for the departure field's station to be installed.
pub(super) const DEPARTURE_STATION_WAIT: Duration = Duration::from_secs(20);

/// How the arm's wait for the departure station ended.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum DepartureWait {
    /// The station up is the field's (or there is no towered field here).
    Ready { armed: bool },
    /// Not by the deadline: the field wanted and the station still up.
    Refused { want: String, active: Option<String> },
}

/// Wait until the installed station is the field under the aircraft. `look`
/// judges once (`true` = it may arm the field's station, asked at most once:
/// the main loop installs it); the wait ends Ready, or Refused at `wait`.
pub(crate) async fn wait_departure_station<F>(mut look: F, wait: Duration, poll: Duration) -> DepartureWait
where
    F: FnMut(bool) -> (crate::party_line::DepartureStation, Option<String>),
{
    use crate::party_line::DepartureStation as D;
    let t0 = tokio::time::Instant::now();
    let mut armed = false;
    loop {
        let (verdict, active) = look(!armed);
        let want = match verdict {
            D::Ready => return DepartureWait::Ready { armed },
            D::Arm { want } => {
                armed = true;
                want
            }
            D::Waiting { want } => want,
        };
        if t0.elapsed() >= wait {
            return DepartureWait::Refused { want, active };
        }
        tokio::time::sleep(poll).await;
    }
}

/// The words for a departure whose field's station never came up. Pure.
pub(crate) fn departure_station_refusal(want: &str, active: Option<&str>) -> String {
    match active {
        Some(a) => format!(
            "The tower for {want} isn't up; the station is still {a}. Ask again in a moment. Your aircraft."
        ),
        None => format!("The tower for {want} isn't up. Ask again in a moment. Your aircraft."),
    }
}

/// The arm's departure-station check on the aircraft's position (blackbox
/// `full_auto` `departure_station`). An unread position is no judgement:
/// the mission goes on as before.
pub(super) async fn departure_station(xp: &XPlaneClient) -> Result<(), String> {
    let (lat, lon) = tokio::join!(xp.get_float(LAT), xp.get_float(LON));
    let (Ok(lat), Ok(lon)) = (lat, lon) else {
        return Ok(());
    };
    let under = crate::party_line::field_under(lat, lon, true).map(|(icao, _)| icao);
    let t0 = Instant::now();
    let out = wait_departure_station(
        |arm| crate::party_line::departure_station(lat, lon, arm, Instant::now()),
        DEPARTURE_STATION_WAIT,
        Duration::from_millis(250),
    )
    .await;
    let waited_ms = t0.elapsed().as_millis() as u64;
    match out {
        DepartureWait::Ready { armed } => {
            event(
                "departure_station",
                json!({ "field": under, "armed": armed, "ready": true, "waited_ms": waited_ms }),
            );
            Ok(())
        }
        DepartureWait::Refused { want, active } => {
            event(
                "departure_station",
                json!({ "field": want, "active": active, "ready": false, "waited_ms": waited_ms }),
            );
            Err(departure_station_refusal(&want, active.as_deref()))
        }
    }
}

pub(super) async fn taxi_route_is_ours(xp: &XPlaneClient) -> bool {
    if crate::party_line::taxi_to_stand() {
        return false;
    }
    let (lat_r, lon_r) = tokio::join!(xp.get_float(LAT), xp.get_float(LON));
    match (lat_r, lon_r) {
        (Ok(lat), Ok(lon)) => crate::party_line::with_taxi_route(|r| {
            r.map(|w| {
                w.path
                    .iter()
                    .any(|p| crate::nav::haversine_nm(lat, lon, p.0, p.1) * 1852.0 < 200.0)
            })
        })
        .unwrap_or(false),
        // Position unknown: trust the route rather than re-request in a loop
        // we cannot verify our way out of.
        _ => crate::party_line::with_taxi_route(|r| r.is_some()),
    }
}
