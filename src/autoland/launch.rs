//! The arm: the launch doors and the runway armed for the approach.

use super::*;

/// Cancel an active autoland; returns whether one was flying.
pub fn abort() -> bool {
    VACATE_FOLLOWS.store(false, Ordering::SeqCst);
    RUN.cancel()
}

/// Fly a full emergency autoland to the nearest suitable runway. AP-first: HDG bug
/// tracks the guidance heading, VS mode flies the glidepath, throttle holds the
/// approach speed, gear/flaps come down on final, and it eases to idle in the
/// flare. EXPERIMENTAL — untuned; test high, over an airport, with a hand ready.
/// Own thread/runtime/client (like the recover watcher).
#[allow(clippy::too_many_arguments)]
pub fn run_autoland_scoped(
    base: String,
    apt_dat: String,
    cfg: ApConfig,
    approach_ias: f64,
    voice: Option<Voice>,
    // The chat mirror now rides `voice.alert`; an explicit `s.action` here too
    // double-wrote every autoland line to the transcript (2026-07-24).
    _status: Option<crate::plugin_status::StatusSink>,
    // The arrival's already-chosen landing runway (by wind) AND its airport.
    // `Some` forces that end; `None` self-picks by track — the emergency and
    // manual path, unchanged.
    forced_rwy: Option<PlannedRunway>,
    // The launcher's cancel tokens: a cancel of the loop that handed the
    // landing down (a stand-down landing before or after the launch) is the
    // autoland's too. Empty only for the pilot's own request.
    parents: Vec<std::sync::Arc<AtomicBool>>,
    // This arm's gate and path angle (`HandDown::default()`: the
    // instrument gate on 3 degrees).
    hand_down: HandDown,
) {
    spawn_autoland(AutolandLaunch {
        base,
        apt_dat,
        cfg,
        approach_ias,
        voice,
        forced_rwy,
        parents,
        hand_down,
    });
}

/// Everything an autoland worker owns from arm until hand-back. Keeping the
/// launch facts together prevents the approach thread and its runway re-arm
/// from acquiring parallel argument lists as new per-approach state is added.
pub(super) struct AutolandLaunch {
    pub(super) base: String,
    pub(super) apt_dat: String,
    pub(super) cfg: ApConfig,
    pub(super) approach_ias: f64,
    pub(super) voice: Option<Voice>,
    pub(super) forced_rwy: Option<PlannedRunway>,
    pub(super) parents: Vec<std::sync::Arc<AtomicBool>>,
    pub(super) hand_down: HandDown,
}

pub(super) struct ArmedRunway {
    pub(super) runway: Runway,
    pub(super) ils: Option<crate::nav::Ils>,
    pub(super) coupled_ok: bool,
    /// The autopilot lands it on this ILS (`ap_lands`), where declared.
    pub(super) lands: Option<crate::aircraft::AutopilotLands>,
}

/// Mutable facts that describe this approach as an operation rather than a
/// flight law: clearances, stability judgment and go-around communication.
/// Keeping them under one owner makes reset-at-new-approach explicit and
/// prevents another free-standing latch being added to the control loop.
pub(super) struct ApproachRuntime {
    pub(super) go_around: GoAroundLatch,
    pub(super) stable_watch: crate::stable_approach::StableWatch,
    pub(super) stable_clock: std::time::Instant,
    pub(super) stable_balk: Option<&'static str>,
    pub(super) stable_said: Option<&'static str>,
    pub(super) gate_recorded: bool,
    pub(super) orders: crate::go_around::GoAroundWatch,
    pub(super) uncleared: UnclearedWatch,
    pub(super) owe_final_report: bool,
    pub(super) said_going_around: bool,
    pub(super) going_around_callsign: Option<Option<String>>,
    pub(super) balk_watch: Option<BalkWatch>,
    pub(super) spoke_going_around: bool,
}

impl ApproachRuntime {
    pub(super) fn arm() -> Self {
        Self {
            go_around: GoAroundLatch::new(),
            stable_watch: crate::stable_approach::StableWatch::default(),
            stable_clock: std::time::Instant::now(),
            stable_balk: None,
            stable_said: None,
            gate_recorded: false,
            orders: crate::go_around::GoAroundWatch::arm(),
            uncleared: UnclearedWatch::default(),
            owe_final_report: false,
            said_going_around: false,
            going_around_callsign: None,
            balk_watch: None,
            spoke_going_around: false,
        }
    }
}

pub(super) fn announce_autoland(voice: &Option<Voice>, line: String, sev: Priority) {
    tracing::info!(target: "xvoicelayer::autoland", spoke = %line, "copilot");
    if let Some(v) = voice {
        v.alert(sev, line);
    }
}

/// Resolve, announce, tune and terrain-correct the one runway this worker
/// will fly. Failure has already been spoken and logged when this returns
/// `None`; the caller only has to release the active worker.
pub(super) async fn arm_runway(
    xp: &XPlaneClient,
    apt_dat: &str,
    forced_rwy: Option<&PlannedRunway>,
    gp_deg: f64,
    voice: &Option<Voice>,
) -> Option<ArmedRunway> {
    let (Ok(lat0), Ok(lon0), Ok(_mag_var)) = (
        xp.get_float(LAT).await,
        xp.get_float(LON).await,
        xp.get_float(MAG_VAR).await,
    ) else {
        return None;
    };
    let hdg0 = xp
        .get_float("sim/flightmodel/position/psi")
        .await
        .unwrap_or(0.0);
    let cleared = crate::party_line::clearance_runway();
    let mut from_clearance = false;
    let picked = match forced_rwy {
        Some(planned_rwy) => match runway_by_id(
            apt_dat,
            lat0,
            lon0,
            &planned_rwy.id,
            40.0,
            planned_rwy.icao.as_deref(),
        ) {
            Ok(None) => {
                crate::blackbox::record(
                    "autoland",
                    serde_json::json!({"event": "forced_runway_missing",
                        "id": planned_rwy.id, "icao": planned_rwy.icao}),
                );
                // A named arrival is not an emergency "land anywhere" arm.
                // Losing its exact end must never silently change airports.
                Ok(None)
            }
            Ok(Some(planned)) => {
                let switch_to = clearance_switch(
                    &planned.id,
                    crate::party_line::clearance_state(),
                    cleared.as_deref(),
                    None,
                    false,
                    None,
                );
                match switch_to.as_deref() {
                    Some(c) => match runway_by_id(
                        apt_dat,
                        lat0,
                        lon0,
                        c,
                        40.0,
                        Some(planned.icao.as_str()),
                    ) {
                        Ok(Some(r)) if r.icao == planned.icao => {
                            crate::blackbox::record(
                                "autoland",
                                serde_json::json!({"event": "runway_from_clearance",
                                    "planned": planned.id, "cleared": r.id}),
                            );
                            announce_autoland(
                                voice,
                                format!(
                                    "Tower cleared us for runway {}, not the planned {}. Landing runway {}.",
                                    crate::crew_radio::spell_runway(&r.id),
                                    crate::crew_radio::spell_runway(&planned.id),
                                    crate::crew_radio::spell_runway(&r.id)
                                ),
                                Priority::Caution,
                            );
                            from_clearance = true;
                            Ok(Some(r))
                        }
                        _ => Ok(Some(planned)),
                    },
                    None => Ok(Some(planned)),
                }
            }
            err => err,
        },
        None => self_pick(apt_dat, lat0, lon0, hdg0),
    };
    let mut runway = match picked {
        Ok(Some(r)) => r,
        other => {
            tracing::warn!(target: "xvoicelayer::autoland", ?other, from=?(lat0, lon0), "no runway found");
            announce_autoland(
                voice,
                "No suitable runway within range. You have the aircraft.".into(),
                Priority::Warning,
            );
            return None;
        }
    };
    tracing::info!(
        target: "xvoicelayer::autoland",
        icao = %runway.icao, id = %runway.id, dist_nm = runway.dist_nm,
        len_m = runway.length_m, course = runway.course_deg, elev = runway.elev_ft,
        "runway selected"
    );
    crate::full_auto::retarget_arrival_runway(
        &runway.id,
        armed_runway_why(forced_rwy.map(|p| p.id.as_str()), &runway.id, from_clearance),
    );
    announce_autoland(
        voice,
        format!(
            "I'll land at {} runway {}, {:.0} miles. I have the approach.",
            runway.icao, runway.id, runway.dist_nm
        ),
        Priority::Warning,
    );
    let ils = crate::ils::tune(xp, &runway.icao, &runway.id, lat0, lon0).await;
    // The airframe that LANDS on its own autopilot (`ap_lands`): both
    // receivers on the ILS, and the coupling is its approach mode.
    let lands = ils.as_ref().and_then(|_| crate::ap_lands::declared());
    if let (Some(l), Some(i)) = (lands.as_ref(), ils.as_ref()) {
        let nav2 = crate::ap_lands::prepare(xp, l, i).await;
        crate::blackbox::record(
            "autoland",
            serde_json::json!({"event": "ap_lands_armed", "rwy": format!("{} {}", runway.icao, runway.id),
                "freq_mhz": i.freq_mhz(), "course": i.course_mag, "nav2_ok": nav2,
                "autothrottle": l.autothrottle}),
        );
        announce_autoland(
            voice,
            format!(
                "I.L.S. runway {} is set. The autopilot will land it, I'll monitor.",
                crate::crew_radio::spell_runway(&runway.id)
            ),
            Priority::Advisory,
        );
    }
    let coupled_ok = ils.is_some() && (crate::ils::coupled_allowed() || lands.is_some());

    let (aim_lat, aim_lon) = project_latlon(
        runway.thr_lat,
        runway.thr_lon,
        runway.course_deg,
        TOUCHDOWN_AIM_NM,
    );
    let probed = crate::terrain::sample_ft(
        xp,
        &[(runway.thr_lat, runway.thr_lon), (aim_lat, aim_lon)],
    )
    .await;
    let (pavement_aim, seen) = aim_elevation_ft(runway.elev_ft, probed.as_deref());
    let gs_aim = if ils.is_some() {
        let (nav, icao, id) = (
            crate::settings::live().nav_data_path.clone(),
            runway.icao.clone(),
            runway.id.clone(),
        );
        tokio::task::spawn_blocking(move || crate::ils::find_glideslope(&nav, &icao, &id))
            .await
            .ok()
            .flatten()
            .and_then(|gs| {
                crate::ils::glideslope_aim_ft(
                    &gs,
                    runway.thr_lat,
                    runway.thr_lon,
                    runway.course_deg,
                    TOUCHDOWN_AIM_NM,
                    gp_deg,
                )
            })
    } else {
        None
    };
    let aim = aim_with_glideslope(pavement_aim, gs_aim);
    crate::blackbox::record("autoland", serde_json::json!({
        "event": "threshold_elev", "rwy": format!("{} {}", runway.icao, runway.id),
        "header_ft": runway.elev_ft, "probed_ft": seen, "aim_ft": aim,
        "pavement_aim_ft": pavement_aim, "gs_aim_ft": gs_aim,
        "width_m": runway_width_m(&runway),
        "xtk_limit_m": landing_xtk_limit_m(runway_width_m(&runway)),
        "flare_xtk_limit_m": flare_xtk_limit_m(runway_width_m(&runway)),
    }));
    if (aim - runway.elev_ft).abs() > THRESHOLD_ELEV_TRUST_FT {
        let diff = aim - runway.elev_ft;
        let line = if gs_aim.is_some() {
            format!(
                "Runway {} on the glideslope, aiming at {:.0} feet.",
                crate::crew_radio::spell_runway(&runway.id),
                aim
            )
        } else {
            format!(
                "Runway {} threshold is {:.0} feet {} the field elevation. Aiming at {:.0} feet.",
                crate::crew_radio::spell_runway(&runway.id),
                diff.abs(),
                if diff < 0.0 { "below" } else { "above" },
                aim
            )
        };
        announce_autoland(voice, line, Priority::Advisory);
        runway.elev_ft = aim;
    } else if (aim - pavement_aim).abs() > 0.5 {
        runway.elev_ft = aim;
    }
    Some(ArmedRunway {
        runway,
        ils,
        coupled_ok,
        lands,
    })
}
