//! The arrival: its watch, the taxi in, the airborne end, the brief, the
//! inbound call, the arrival built and re-picked.

use super::*;

/// How long the arrival may stop closing on the fix the follower is steering
/// at before the mission gives up on it, seconds. Longer than the follower's
/// own per-leg stall watch (which rejoins twice before it stands itself down),
/// because this is the backstop under that, not a second opinion on it.
pub(super) const ARRIVAL_STALL_SECS: f64 = 600.0;
/// Closure that restarts that clock, miles.
pub(super) const ARRIVAL_PROGRESS_NM: f64 = 0.5;
/// Absolute ceiling on the arrival wait, seconds — three hours. Not a judgement
/// on the approach (the watch below is): a bound on a mission thread whose
/// follower has wedged ACTIVE without ticking.
pub(super) const ARRIVAL_CAP_SECS: f64 = 3.0 * 3600.0;

/// The mission's watch on an arrival it has handed to the follower: a watchdog
/// on PROGRESS, not a clock.
///
/// It was a flat 1,800 seconds from the arm, and an approach armed far enough
/// out simply takes longer than that to fly. EGLL-EGPH, 2026-09-17: the
/// SimBrief route's last fix was RIBEL, 121 nm from the ILS 24 initial fix, so
/// the arrival armed there and the follower flew the straight line at 265 kt,
/// descending on its profile the whole way. At 4.2 nm to run — two minutes
/// from the fix — the 1,800 seconds expired, the mission returned, and its
/// scope cancelled the follower under it at 4,900 ft. Nothing flew the
/// aircraft for the next eleven minutes: the speed bled from 200 to 128 kt
/// through six minutes of airspeed warnings and the 737 sank into the ground
/// near Perth. A clock cannot tell a long arrival from a stuck one; closure
/// can, and closure is what the mission actually cares about.
#[derive(Default)]
pub(crate) struct ArrivalWatch {
    pub(super) fix: Option<String>,
    pub(super) best_nm: f64,
    pub(super) best_at_s: f64,
}

impl ArrivalWatch {
    /// Feed the fix the follower is steering at and the miles to it. True once
    /// per stall: no new low for [`ARRIVAL_STALL_SECS`]. A new fix starts the
    /// judgement afresh (the distance jumps UP when a leg sequences), and a
    /// follower steering at no fix at all holds the clock rather than starting
    /// it — a to-altitude leg is the follower's own business.
    pub(crate) fn stalled(&mut self, fix: Option<&str>, dist_nm: Option<f64>, now_s: f64) -> bool {
        let (Some(fix), Some(d)) = (fix, dist_nm) else {
            self.fix = None;
            return false;
        };
        if self.fix.as_deref() != Some(fix) {
            self.fix = Some(fix.to_string());
            self.best_nm = d;
            self.best_at_s = now_s;
            return false;
        }
        if d < self.best_nm - ARRIVAL_PROGRESS_NM {
            self.best_nm = d;
            self.best_at_s = now_s;
            return false;
        }
        if now_s - self.best_at_s > ARRIVAL_STALL_SECS {
            self.best_nm = d;
            self.best_at_s = now_s;
            return true;
        }
        false
    }
}

/// The last stage of any mission that ends with a landing: once autoland has
/// rolled out and stopped, request the taxi-in and drive to the stand.
///
/// This is not tidiness, it is the runway. Autoland stops ON the strip and
/// hands back, and for a CIRCUIT nothing came after it — full auto armed the
/// circuit and exited, so the aircraft simply sat where it stopped. At
/// Southampton it held runway 02 for ninety seconds after touchdown while an
/// arrival closed from 2.6 nm to 0.5 nm: six `runway_incursion` records, and
/// the tower telling the lander "continue approach, traffic on the runway
/// ahead". It only moved because the pilot said "taxi to parking"
/// (2026-08-01). Vacating is part of the landing, so it belongs to whoever
/// owns the landing.
///
/// `wait_for_arm` is for the circuit, whose autoland is minutes away when the
/// mission is armed; the flight path calls this with autoland already running.
pub(super) async fn taxi_in_after_landing(
    xp: &XPlaneClient,
    config: &Config,
    voice: &Option<Voice>,
    tools: &SubTools,
    wait_for_arm: bool,
) {
    use std::sync::atomic::Ordering::Relaxed;
    if wait_for_arm {
        // Wait for autoland to ARM — or for the mission to DIE first. The
        // condition used to be autoland alone, and a circuit cancelled with
        // "cancel the circuit" (which touches no full-auto flag) left this
        // wait holding full auto for its whole 900 s (audit, 2026-08-02).
        // The mission is alive while any of its flyers is: the circuit, the
        // published approach it may escalate to, or autoland itself.
        waiting("the landing to arm");
        wait_until(ARM_WAIT_SECS, 1000, || async {
            crate::autoland::ACTIVE.load(Relaxed)
                || !(crate::circuit::ACTIVE.load(Relaxed) || crate::procedure::ACTIVE.load(Relaxed))
        })
        .await;
        // The handoff clears the circuit's flag AROUND arming autoland, so a
        // "nothing flying" read gets a beat of grace before it means dead.
        let armed = crate::autoland::ACTIVE.load(Relaxed)
            || wait_until(5.0, 250, || async { crate::autoland::ACTIVE.load(Relaxed) }).await;
        if !armed {
            event("no_autoland", json!({}));
            return;
        }
    }
    if ABORT.load(SeqCst) {
        return;
    }
    // The panel's automation well reads the stage string, and full auto now
    // stays ACTIVE through the mission instead of dropping out at the arm —
    // so say where we actually are rather than leaving "circuit armed" up for
    // the whole flight.
    event("on_approach", json!({}));
    // A vacate drive follows this landing, so the rollout may hand off at taxi
    // pace instead of stopping dead on the strip — landing includes getting
    // off the runway, and it is one continuous manoeuvre, not a stop and a
    // fresh start (EGHI, 2026-08-02). Consumed by the rollout; cleared below
    // if the landing never happens, so it cannot go stale.
    crate::autoland::VACATE_FOLLOWS.store(true, Relaxed);
    waiting("the landing");
    let landed = wait_until(1800.0, 1000, || async {
        !crate::autoland::ACTIVE.load(Relaxed)
    })
    .await;
    // Autoland also goes inactive climbing away from a go-around, so being on
    // the ground is the half of "landed" that matters here.
    let on_ground = xp
        .get_float(crate::datarefs::ON_GROUND_ANY)
        .await
        .unwrap_or(0.0)
        > 0.5;
    // A wreck is "on the ground" too. The Gibraltar go-around hit the North
    // Front, the sim raised has_crashed, autoland read the stop as a rollout,
    // and this said "On the ground. I'll clear the runway", called for taxi,
    // and armed auto taxi on the wreckage (12:46Z, 2026-09-11). Fail open: a
    // failed read is not a crash.
    let crashed = xp
        .get_float(crate::datarefs::HAS_CRASHED)
        .await
        .unwrap_or(0.0)
        > 0.5;
    if !landed || !on_ground || crashed || ABORT.load(SeqCst) {
        // The landing never completed here — take the rolling-handoff offer
        // back, or a later standalone landing would hand off to nobody.
        crate::autoland::VACATE_FOLLOWS.store(false, Relaxed);
        event(
            "no_taxi_in",
            // `autoland_ended`, not "landed": autoland also goes inactive on
            // a give-up, and `landed: true` was logged at 1,900 ft.
            json!({ "autoland_ended": landed, "on_ground": on_ground, "crashed": crashed }),
        );
        // Autoland let go in the AIR and nobody asked for it: the mission is
        // not over until someone is flying. LEMG 13 SR22 circuit 2026-09-29
        // (sortie 1790692781-full_auto): "Going around, climbing straight
        // ahead. Your aircraft." at 805 ft, and the mission ended with an
        // unattended runner and nobody flying.
        if landed && !on_ground && !crashed && !ABORT.load(SeqCst) {
            hold_airborne_end(xp, &config.xplane_base_url, voice).await;
        }
        return;
    }
    announce(
        voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Down. I'll get us off the runway and taxi to parking.",
            "Down. Vacating and taxiing us to parking.",
            "On the ground. I'll clear the runway and get us to the gate.",
        ]),
    );
    event("taxi_in", json!({}));
    crate::taxi_in::start(
        config.xplane_base_url.clone(),
        config.clone(),
        voice.clone(),
        tools.taxi.park_brake_valve_set,
        tools.taxi.prop.clone(),
        tools.taxi.pace,
        // We have just landed, so the first thing owed is the runway itself.
        crate::taxi_in::vacate_runway(),
        // The mission ends shut down on the stand.
        true,
    );
    RUN.record(sim::run::Outcome::HandedOff);
}

/// How long a published missed approach (`procedure`, flown from autoland's
/// go-around) is left to finish before the airborne end is judged.
pub(super) const AIRBORNE_END_PROCEDURE_WAIT_S: f64 = 900.0;
/// The autopilot is watched this long before an airborne end believes it.
pub(super) const AIRBORNE_END_VERIFY_S: f64 = crate::tools::recover::AP_VERIFY_S;
/// The first seconds of that watch are the modes engaging, not judged.
pub(super) const AIRBORNE_END_GRACE_S: f64 = 3.0;
/// ...and it ends this close to level (VS 0 commanded at the start).
pub(super) const AIRBORNE_END_LEVEL_FPM: f64 = 1_000.0;

/// One look at the autopilot after the mission's landing let go airborne.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct AirborneLook {
    pub servos: Option<f64>,
    pub hdg_status: Option<f64>,
    pub vs_status: Option<f64>,
    pub alt_hold_status: Option<f64>,
    pub vvi_fpm: Option<f64>,
    pub pitch_deg: Option<f64>,
    pub bank_deg: Option<f64>,
    pub ias_kts: Option<f64>,
}

/// Is the autopilot holding the aircraft, or why not? A failed read of the
/// servos or the modes is NOT holding (only what reads back is claimed); a
/// failed attitude read is no evidence against. `last` is the end of the
/// watch, where the climb or descent must have come to near level. Pure.
pub(crate) fn airborne_end_verdict(
    look: &AirborneLook,
    min_safe_ias: f64,
    last: bool,
) -> Result<(), &'static str> {
    let on = |v: Option<f64>| v.is_some_and(|x| x > 0.5);
    if !on(look.servos) {
        return Err("servos_off");
    }
    if !on(look.hdg_status) {
        return Err("no_heading_mode");
    }
    if !(on(look.vs_status) || on(look.alt_hold_status)) {
        return Err("no_vertical_mode");
    }
    if !crate::tools::recover::ap_holding(
        look.vvi_fpm,
        look.pitch_deg,
        look.bank_deg,
        look.ias_kts,
        min_safe_ias,
    ) {
        return Err("not_holding");
    }
    if last
        && look
            .vvi_fpm
            .filter(|v| v.is_finite())
            .is_some_and(|v| v.abs() > AIRBORNE_END_LEVEL_FPM)
    {
        return Err("not_level");
    }
    Ok(())
}

pub(super) async fn read_airborne_look(xp: &XPlaneClient) -> AirborneLook {
    let (servos, hdg, vs, alt, vvi, pitch, bank, ias) = tokio::join!(
        crate::autoflight::read_status(&*xp, crate::datarefs::SERVOS_ON),
        crate::autoflight::read_status(&*xp, crate::datarefs::HEADING_STATUS),
        crate::autoflight::read_status(&*xp, autoflight::names::VS_STATUS),
        crate::autoflight::read_status(&*xp, autoflight::names::ALT_HOLD_STATUS),
        xp.get_float("sim/flightmodel/position/vh_ind_fpm"),
        xp.get_float("sim/flightmodel/position/theta"),
        xp.get_float("sim/flightmodel/position/phi"),
        xp.get_float("sim/flightmodel/position/indicated_airspeed"),
    );
    AirborneLook {
        servos: servos.ok(),
        hdg_status: hdg.ok(),
        vs_status: vs.ok(),
        alt_hold_status: alt.ok(),
        vvi_fpm: vvi.ok(),
        pitch_deg: pitch.ok(),
        bank_deg: bank.ok(),
        ias_kts: ias.ok(),
    }
}

/// The pilot's "my controls" or a new flight loading ended this: nothing of
/// ours may touch the aircraft now.
pub(super) fn airborne_end_stood_down() -> bool {
    ABORT.load(SeqCst) || crate::xplane::sim_loading()
}

/// The mission's landing let go in the air (a give-up after its go-arounds,
/// or anything else that ended autoland airborne): never "your aircraft" to
/// nobody. A published missed approach flying is waited for; then the
/// autopilot is put on heading and VS 0 and WATCHED; only a held level is
/// handed over, said with its numbers, the governor keeping the speed.
/// Anything else (slow, upset, the servos out, a mode that will not hold)
/// is the recovery's (`recover::take_over`), which flies it level and keeps
/// it until "my controls". LEMG 13 SR22 circuit 2026-09-29: handed back at
/// 805 ft climbing at 1,000 fpm, the runner unattended.
pub(super) async fn hold_airborne_end(xp: &XPlaneClient, base_url: &str, voice: &Option<Voice>) {
    use std::sync::atomic::Ordering::Relaxed;
    waiting("the missed approach");
    wait_until(AIRBORNE_END_PROCEDURE_WAIT_S, 1000, || async {
        !crate::procedure::ACTIVE.load(Relaxed) || airborne_end_stood_down()
    })
    .await;
    if airborne_end_stood_down() {
        return;
    }
    let floor = crate::aircraft::speed_floor_kts();
    let min_safe = floor.filter(|f| f.is_finite() && *f > 0.0).unwrap_or(70.0);
    let take_over = |why: &'static str, look: AirborneLook| {
        // One writer on the lever: the governor autoland armed lets go first.
        crate::tools::speedhold::release();
        let started = crate::tools::recover::take_over(
            base_url.to_string(),
            voice.clone(),
            crate::tools::recover::TakeOver {
                why: format!("full_auto_airborne_{why}"),
                min_safe_ias: floor.map(|f| f + 5.0),
                hold_kts: None,
                ap_no_pitch: false,
                hold_ft: None,
            },
        );
        event(
            "airborne_end",
            json!({ "verdict": "take_over", "why": why, "recovery_started": started,
                    "servos": look.servos, "vvi": look.vvi_fpm, "ias": look.ias_kts,
                    "pitch": look.pitch_deg, "bank": look.bank_deg }),
        );
        announce(
            voice,
            Priority::Warning,
            "The flight is cancelled. I have the aircraft until you say my controls.",
        );
    };
    if crate::tools::recover::ACTIVE.load(SeqCst) {
        event("airborne_end", json!({ "verdict": "recovery_has_it" }));
        announce(
            voice,
            Priority::Warning,
            "The flight is cancelled. I have the aircraft until you say my controls.",
        );
        return;
    }
    if let Some(what) = crate::tools::recover::read_unflyable(xp, floor).await {
        take_over(what, read_airborne_look(xp).await);
        return;
    }
    // The autopilot on heading and VS 0: the servos in if they read out, the
    // modes pressed only when they read off (toggles), the wheel written
    // only in VS mode.
    let look = read_airborne_look(xp).await;
    if look.servos.is_some_and(|v| v < 0.5) {
        // FD first, on the tape (`ap_engage`); the watch below judges it.
        crate::ap_engage::press_logged(
            xp, &crate::ap_engage::fd_commands(), "full_auto", "airborne_end", 1,
        )
        .await;
    }
    let alt_held = look.alt_hold_status.is_some_and(|v| v > 0.5);
    let want = autoflight::Want {
        lateral: Some(autoflight::Lateral::Heading),
        vertical: (!alt_held).then_some(autoflight::Vertical::Vs),
    };
    let read = autoflight::ModeRead {
        hdg_status: look.hdg_status,
        vs_status: look.vs_status,
        modes: None,
    };
    autoflight::bring(xp, want, read, "full_auto_airborne_end").await;
    if !alt_held {
        let _ = autoflight::set_dial(xp, Dial::Vs, 0.0).await;
    }
    let t0 = Instant::now();
    let mut look = read_airborne_look(xp).await;
    loop {
        if airborne_end_stood_down() || crate::tools::recover::ACTIVE.load(SeqCst) {
            return;
        }
        let secs = t0.elapsed().as_secs_f64();
        let last = secs >= AIRBORNE_END_VERIFY_S;
        if secs >= AIRBORNE_END_GRACE_S {
            if let Err(why) = airborne_end_verdict(&look, min_safe, last) {
                take_over(why, look);
                return;
            }
        }
        if last {
            break;
        }
        tokio::time::sleep(Duration::from_millis(1000)).await;
        look = read_airborne_look(xp).await;
    }
    let (alt_r, hdg_r) = tokio::join!(
        xp.get_float(crate::datarefs::ALTITUDE_FT),
        xp.get_float(crate::datarefs::COMPASS_HEADING_DEG),
    );
    let alt = alt_r.ok().filter(|v| v.is_finite());
    let hdg = hdg_r.ok().filter(|v| v.is_finite());
    event(
        "airborne_end",
        json!({ "verdict": "ap_holding", "alt": alt, "hdg": hdg, "vvi": look.vvi_fpm,
                "ias": look.ias_kts, "alt_hold": alt_held }),
    );
    announce(voice, Priority::Warning, &airborne_end_line(hdg, alt));
}

/// The hand-over line once the autopilot is seen holding. No dashes (TTS).
/// Pure.
pub(crate) fn airborne_end_line(hdg_mag: Option<f64>, alt_ft: Option<f64>) -> String {
    let hdg = hdg_mag.map(|h| {
        let h = h.round().rem_euclid(360.0);
        format!(" heading {:03.0}", if h == 0.0 { 360.0 } else { h })
    });
    let alt = alt_ft.map(|a| format!(" at {:.0} feet", (a / 100.0).round() * 100.0));
    format!(
        "I've stopped flying. The autopilot is holding{}{}, level. Your aircraft.",
        hdg.unwrap_or_default(),
        alt.unwrap_or_default()
    )
}

/// The arrival, briefed AT THE GATE: what the destination offers and what
/// full auto will therefore do there — a published approach, a straight-in
/// reconstructed from DME fixes, or a visual landing. The Gibraltar crash was
/// discovered 25 nm out, when the arrival was first built; nothing about it
/// needed the aircraft to be airborne. `Err` is a refusal: a destination the
/// airport data doesn't have, or one with no runway, is not a destination.
pub fn brief_arrival(
    apt_dat: &str,
    cifp: &str,
    fix_path: &str,
    dest: &str,
    runway: Option<&str>,
) -> Result<String, String> {
    let airport = crate::nav::find_airport(apt_dat, dest)
        .ok()
        .flatten()
        .ok_or_else(|| {
            format!("I can't find {dest} in the airport data, so I can't plan the arrival.")
        })?;
    if airport.runways.is_empty() {
        return Err(format!(
            "{} has no runways in the airport data.",
            airport.name
        ));
    }
    // (kind, spoken runway) per end with a published approach; the spoken
    // runway per end with only a DME-fix straight-in.
    let (mut published, mut straight_in, mut visual) = (Vec::new(), Vec::new(), Vec::new());
    let nav_path = std::path::Path::new(fix_path)
        .with_file_name("earth_nav.dat")
        .to_string_lossy()
        .into_owned();
    let (alat, alon) = airport.position().unwrap_or((airport.runways[0].lat_a, airport.runways[0].lon_a));
    for r in &airport.runways {
        for id in [&r.ident_a, &r.ident_b] {
            let spoken = crate::crew_radio::spell_runway(id);
            // Built exactly as the arrival builds it: an approach the brief
            // names is one the arrival can fly (LSZS's RNAV 21 has a 5.3
            // degree visual segment, too steep, and is not named).
            match crate::nav::best_approach(cifp, dest, id).and_then(|appch| {
                crate::procedure::approach_plan_for(
                    cifp,
                    fix_path,
                    &nav_path,
                    &airport,
                    dest,
                    id,
                    alat,
                    alon,
                    crate::procedure::EndAction::Level,
                )
                .ok()
                .map(|p| (appch, p))
            }) {
                Some((appch, plan)) if plan.visual_deg.is_some() => {
                    visual.push((crate::procedure::approach_kind(&appch), spoken))
                }
                Some((appch, _)) => published.push((crate::procedure::approach_kind(&appch), spoken)),
                None if !crate::nav::reconstruct_approach(fix_path, &airport, dest, id).is_empty() => {
                    straight_in.push(spoken)
                }
                None => {}
            }
        }
    }
    if published.is_empty() && straight_in.is_empty() && !visual.is_empty() {
        let (kind, rwy) = &visual[0];
        return Ok(format!(
            "{} has the {kind} to runway {rwy}: I'll fly it to the missed approach point, hand \
             it to you there and talk you down the visual segment.",
            airport.name
        ));
    }
    if published.is_empty() && straight_in.is_empty() {
        let charted = crate::nav::list_procedures(cifp, dest, crate::nav::ProcKind::Appch);
        let msa = crate::nav::airport_msa_ft(&msa_path_for(fix_path), dest);
        if mountain_field(msa, airport.elev_ft) {
            return Ok(format!(
                "{} is a mountain field, and {}. I'll fly the route and hand over at its end, \
                 above the sector altitude of {}.",
                airport.name,
                if charted.is_empty() {
                    "it has no published approach".to_string()
                } else {
                    "its published approaches end short of the runway, which I can't fly".to_string()
                },
                crate::flightplan::spoken_feet(msa.unwrap_or(0.0))
            ));
        }
        if !charted.is_empty() {
            return Ok(format!(
                "{}'s published approaches end short of the runway, which I can't fly, so I'll land visually.",
                airport.name
            ));
        }
    }
    let line = arrival_brief_line(
        &airport.name,
        airport.runways.len() * 2,
        &published,
        &straight_in,
    );
    // The runway chosen at the arm, the one the arrival flies
    // ([`arrival_runway`]), named with the approach it has.
    Ok(match runway {
        Some(r) => format!("{line} {}", planned_runway_line(r, &published, &straight_in)),
        None => line,
    })
}

/// "Planned runway one two, the ILS." Pure.
pub(super) fn planned_runway_line(runway: &str, published: &[(&str, String)], straight_in: &[String]) -> String {
    let spoken = crate::crew_radio::spell_runway(runway);
    match published.iter().find(|(_, s)| *s == spoken) {
        Some((kind, _)) => format!("Planned runway {spoken}, the {kind}."),
        None if straight_in.contains(&spoken) => format!("Planned runway {spoken}, the straight-in."),
        None => format!("Planned runway {spoken}."),
    }
}

/// Sector altitude this far above the field makes it a mountain field: no
/// visual fallback, the arrival is an approach we can fly or a hand-over.
pub(super) const MOUNTAIN_MSA_MARGIN_FT: f64 = 3_000.0;

pub(super) fn mountain_field(msa_ft: Option<f64>, elev_ft: f64) -> bool {
    msa_ft.is_some_and(|m| m - elev_ft >= MOUNTAIN_MSA_MARGIN_FT)
}

/// `earth_msa.dat` beside the fix file (both in Custom Data, or both in the
/// default data).
pub(crate) fn msa_path_for(fix_path: &str) -> String {
    std::path::Path::new(fix_path)
        .with_file_name("earth_msa.dat")
        .to_string_lossy()
        .into_owned()
}

/// The hand-over at the end of the route into a mountain field we cannot fly
/// an approach to. Pure.
pub fn mountain_handover_line(dest: &str, msa_ft: f64, hold_ft: f64) -> String {
    format!(
        "End of the route. I can't fly an approach into {dest}. Your aircraft. The autopilot \
         is holding {}, above the sector altitude of {}.",
        crate::flightplan::spoken_feet(hold_ft),
        crate::flightplan::spoken_feet(msa_ft)
    )
}

/// One line, sized to the field: a big airport's approaches are SUMMARISED
/// ("the ILS on every runway"), a small one's are named. Eight runway ends
/// read out one by one was a paragraph before the mission was said, and the
/// wind sentence that followed said nothing the pilot did not know (EGSS-LFPG,
/// 2026-09-22). Pure, tested.
pub(super) fn arrival_brief_line(
    name: &str,
    ends: usize,
    published: &[(&str, String)],
    straight_in: &[String],
) -> String {
    let kinds: std::collections::BTreeSet<&str> = published.iter().map(|(k, _)| *k).collect();
    let what = if kinds.len() == 1 {
        format!("the {}", kinds.iter().next().unwrap())
    } else {
        "a published approach".to_string()
    };
    let n = published.len();
    if n > 0 && n == ends {
        return format!("{name} has {what} on every runway.");
    }
    if n > 2 {
        return format!("{name} has {what} on {n} of {ends} runway ends.");
    }
    let mut parts: Vec<String> = published
        .iter()
        .map(|(k, rwy)| format!("the {k} on {rwy}"))
        .collect();
    if !straight_in.is_empty() {
        parts.push(format!("a straight-in on {}", straight_in.join(" and ")));
    }
    if parts.is_empty() {
        format!("{name} has no published approach, so I'll land visually.")
    } else {
        format!("{name} has {}.", parts.join(" and "))
    }
}

/// What the blocking arrival builder found: the approach plan + runway when
/// one could be built, and the destination's own tower frequency either way.
pub(super) struct Arrival {
    /// The published/reconstructed approach for `rwy_id`, when one is flyable.
    pub(super) plan: Option<crate::procedure::Plan>,
    /// The chosen landing runway — set whenever the field was found, whether or
    /// not a plan came with it (the visual fallback and the final call need it).
    pub(super) rwy_id: Option<String>,
    pub(super) tower_khz: Option<i64>,
    /// The field's highest sector altitude when it is a MOUNTAIN field
    /// (`mountain_field`): with no approach we can fly, the arrival hands the
    /// aircraft over above it instead of landing visually.
    pub(super) mountain_msa_ft: Option<f64>,
    /// Why the runway chosen at the arm is not `rwy_id` ([`planned_end_verdict`]
    /// or "no_approach"); `None` when it stands or there was none.
    pub(super) overruled: Option<&'static str>,
}

/// The inbound call to the destination's tower, once its station is the one
/// on the frequency (`swapped`), or regardless at the last moment before the
/// final report. The swap IS the region handoff: the frequency is already
/// ours and the readback would be a garbled call to the NEW tower, but a
/// discrete code still goes back to seven thousand.
/// The active station's field wind from its METAR, published now: the
/// tower picks its runway from it, never from the air the aircraft is in.
pub(super) fn publish_field_wind() {
    if let Some((icao, lat, lon)) = crate::party_line::active_icao() {
        let wind = crate::weather::field_wind(&crate::config::xplane_root(), &icao);
        event("field_wind", json!({ "icao": icao, "wind": wind }));
        crate::party_line::set_surface_wind(lat, lon, wind);
    }
}

pub(super) async fn call_inbound(
    xp: &XPlaneClient,
    ctx: &ToolCtx<'_>,
    voice: &Option<Voice>,
    callsign: &Option<String>,
    khz: i64,
    swapped: bool,
    runway: Option<&str>,
) {
    if let Some(h) = crate::party_line::take_region_handoff() {
        obey_region_handoff(xp, ctx, voice, callsign, h, false).await;
    }
    // ON the tower's frequency before the call is keyed. The enroute leg
    // calls when the region has made the destination's tower the station,
    // but a region change near a field is a crew notice ("closest field is
    // Exeter"), not a handoff, and nothing retunes on a notice: EGPH-EGTE
    // made this call on Carlisle's frequency, where COM1 had sat since the
    // Scottish border, it went into the void marked as made, and Exeter
    // challenged the arrival as an unknown aircraft (2026-09-21).
    // An instrument arrival at a field with an approach unit checks in with
    // APPROACH, which clears the approach and hands us to the tower once
    // established (`arrival_radio`); the tower is called first only by a VFR
    // arrival or at a field with no approach row. The realism review,
    // 2026-10-10: our arrival called the tower and was told "join final".
    let approach = approach_first_khz(
        swapped,
        crate::party_line::active_approach_khz(),
        crate::party_line::own_instrument_arrival(callsign.as_deref().unwrap_or("")),
    );
    APPROACH_FIRST.store(approach.is_some(), SeqCst);
    let call_khz = approach.unwrap_or(khz);
    let tuned = xp
        .get_float(COM1_FREQ_KHZ)
        .await
        .ok()
        .map(|v| v.round() as i64);
    if !inbound_on_frequency(tuned, call_khz) {
        let _ = xp.set_float(COM1_FREQ_KHZ, call_khz as f64).await;
        event("inbound_retune", json!({ "khz": call_khz, "was": tuned }));
    }
    // The field's OWN wind before the tower hears us: the station may have
    // become this field a moment ago, and until the monitor's next feed its
    // wind is the winds aloft at the aircraft (LIPK 2026-09-25: 053 at 12 at
    // 5,000 ft against the field's 240 at 4, and "join final runway one two"
    // to a crew set up for the ILS 30).
    publish_field_wind();
    let words = match runway {
        Some(r) => format!(
            "inbound for landing, runway {}",
            crate::crew_radio::spell_runway(r)
        ),
        None => "inbound for landing".to_string(),
    };
    radio_out(crate::crew_radio::radio_call(callsign, &words));
    INBOUND_MADE.store(true, SeqCst);
    // From here the destination's tower is OURS: the region's service-weighted
    // nearest-field pick may not hand the station to a closer field while we
    // fly its zone (London City took the station fifteen miles out and both
    // the final report and the taxi-in request went into the void, EGLL
    // 2026-09-17). Released by `release_arrival_station`.
    crate::party_line::pin_arrival_station(Some(khz));
    event(
        "inbound_call",
        json!({ "khz": khz, "station_swapped": swapped, "approach_khz": approach }),
    );
}

/// Is COM1 already on the tower the inbound call is for? A failed read is
/// not "yes": the retune is one write and the void call cost a zone challenge.
pub fn inbound_on_frequency(com1_khz: Option<i64>, tower_khz: i64) -> bool {
    com1_khz == Some(tower_khz)
}

/// Release the arrival's hold on the station (see
/// [`crate::party_line::pin_arrival_station`]). Every path that ends a mission
/// calls this, so a pin can never outlive the arrival that set it: the
/// region's ordinary nearest-field handoffs resume for the next flight.
pub fn release_arrival_station() {
    crate::party_line::pin_arrival_station(None);
    crate::party_line::end_arrival();
}

/// The wind the arrival picks its runway by, and where it came from: the
/// destination's METAR when there is one, else the aircraft's own wind
/// datarefs (the winds aloft where we are, the only wind without a report).
pub fn arrival_wind(
    metar: Option<(f64, f64)>,
    aircraft: Option<(f64, f64)>,
) -> (Option<(f64, f64)>, &'static str) {
    match (metar, aircraft) {
        (Some(w), _) => (Some(w), "metar"),
        (None, Some(w)) => (Some(w), "aircraft"),
        (None, None) => (None, "none"),
    }
}

/// Blocking arrival builder: destination airport + the landing runway, then the
/// published-approach plan for it. The runway is the end that best faces INTO
/// the surface wind when the wind is decisive (`WIND_DECISIVE_KTS`); in a light
/// or variable wind it's the end whose landing direction best matches our
/// bearing to the field (no big turn onto final for a wind that doesn't matter).
/// The tower frequency comes back even when no plan does — the radio still needs
/// the right station for the visual fallback. `wind` is (from_deg true, kts).
#[allow(clippy::too_many_arguments)]
pub(super) fn build_arrival(
    apt_dat: &str,
    cifp: &str,
    fixp: &str,
    navp: &str,
    dest: &str,
    lat: f64,
    lon: f64,
    cfg: crate::tools::autopilot::ApConfig,
    approach_ias: f64,
    wind: Option<(f64, f64)>,
    forced_rwy: Option<&str>,
    planned_rwy: Option<&str>,
) -> Arrival {
    let Some(airport) = crate::nav::find_airport(apt_dat, dest).ok().flatten() else {
        return Arrival {
            plan: None,
            rwy_id: None,
            tower_khz: None,
            mountain_msa_ft: None,
            overruled: None,
        };
    };
    let tower_khz = airport.staffed_tower().map(|f| f.khz);
    let mountain_msa_ft = crate::nav::airport_msa_ft(&msa_path_for(fixp), dest)
        .filter(|m| mountain_field(Some(*m), airport.elev_ft));
    // The tower's runway when it has named one (no fallback to another end:
    // the caller keeps its own plan if this one cannot be flown)...
    if let Some(forced) = forced_rwy {
        let end = crate::procedure::EndAction::Approach {
            apt_dat: apt_dat.to_string(),
            cfg,
            approach_ias,
            rwy_id: Some(crate::runway::PlannedRunway::at(dest, forced)),
        };
        let plan = crate::procedure::approach_plan_for(
            cifp, fixp, navp, &airport, dest, forced, lat, lon, end,
        )
        .ok();
        return Arrival {
            plan,
            rwy_id: Some(forced.to_string()),
            tower_khz,
            mountain_msa_ft,
            overruled: None,
        };
    }
    // The runway chosen at the arm, which the STAR flown serves, while it is
    // one we may land on in this wind...
    let (planned, mut overruled) = match planned_rwy {
        Some(p) => match planned_end_verdict(&airport, p, wind) {
            Ok(()) => (Some(p.to_string()), None),
            Err(why) => (None, Some(why)),
        },
        None => (None, None),
    };
    // ...else into the wind when it's strong enough to matter...
    let by_wind = wind
        .filter(|(_, kts)| *kts >= WIND_DECISIVE_KTS)
        .and_then(|(dir, kts)| crate::nav::best_runway_for_wind(&airport.runways, dir, kts))
        .map(|(id, _, _)| id);
    // ...otherwise the end most aligned with our arrival bearing.
    let rwy_id = planned.or(by_wind).or_else(|| {
        let diff = |a: f64, b: f64| crate::angles::abs_diff_deg(a, b);
        let (mut best_id, mut best_score) = (None::<String>, f64::MAX);
        // Hard runways only: LOWI's alignment pick was the grass "26R".
        for r in crate::nav::assignable_runways(&airport.runways) {
            let hdg_a = crate::nav::bearing_deg(r.lat_a, r.lon_a, r.lat_b, r.lon_b);
            let hdg_b = (hdg_a + 180.0).rem_euclid(360.0);
            for (id, thr_lat, thr_lon, hdg) in [
                (&r.ident_a, r.lat_a, r.lon_a, hdg_a),
                (&r.ident_b, r.lat_b, r.lon_b, hdg_b),
            ] {
                let arrival_brg = crate::nav::bearing_deg(lat, lon, thr_lat, thr_lon);
                let score = diff(arrival_brg, hdg);
                if score < best_score {
                    best_score = score;
                    best_id = Some(id.clone());
                }
            }
        }
        best_id
    });
    let Some(rwy_id) = rwy_id else {
        return Arrival {
            plan: None,
            rwy_id: None,
            tower_khz,
            mountain_msa_ft,
            overruled,
        };
    };
    let plan_for = |id: &str| {
        let end = crate::procedure::EndAction::Approach {
            apt_dat: apt_dat.to_string(),
            cfg: cfg.clone(),
            approach_ias,
            rwy_id: Some(crate::runway::PlannedRunway::at(dest, id)),
        };
        crate::procedure::approach_plan_for(cifp, fixp, navp, &airport, dest, id, lat, lon, end)
            .ok()
    };
    if let Some(plan) = plan_for(&rwy_id) {
        return Arrival {
            plan: Some(plan),
            rwy_id: Some(rwy_id),
            tower_khz,
            mountain_msa_ft,
            overruled,
        };
    }
    // No approach we can fly to that end: another end that HAS one, if its
    // tailwind is acceptable, before a visual landing. LOWI at 080/6: 08's
    // approaches are a circling one and an RF arc, 26's RNAV Z is a
    // straight-in down the valley; the visual landing at Innsbruck is not an
    // option anyone should take (2026-09-24).
    for id in approach_fallback_ends(&airport, &rwy_id, wind) {
        if let Some(plan) = plan_for(&id) {
            if planned_rwy == Some(rwy_id.as_str()) {
                overruled = Some("no_approach");
            }
            return Arrival {
                plan: Some(plan),
                rwy_id: Some(id),
                tower_khz,
                mountain_msa_ft,
                overruled,
            };
        }
    }
    Arrival {
        plan: None,
        rwy_id: Some(rwy_id),
        tower_khz,
        mountain_msa_ft,
        overruled,
    }
}

/// May the arrival land on the runway chosen at the arm? Only an end of this
/// field's landable runways, and only with no more tailwind than an approach
/// is flown with ([`APPROACH_TAILWIND_MAX_KTS`]): the METAR at the arm is
/// 45 minutes old at the arrival, and a safety limit is not a re-pick.
/// `Err` names why ("not_at_field", "tailwind").
pub(super) fn planned_end_verdict(
    airport: &crate::nav::AirportInfo,
    planned: &str,
    wind: Option<(f64, f64)>,
) -> Result<(), &'static str> {
    if !crate::nav::assignable_runways(&airport.runways)
        .any(|r| r.ident_a.eq_ignore_ascii_case(planned) || r.ident_b.eq_ignore_ascii_case(planned))
    {
        return Err("not_at_field");
    }
    match wind.and_then(|(d, k)| crate::nav::headwind_for(&airport.runways, planned, d, k)) {
        Some(hw) if hw < -APPROACH_TAILWIND_MAX_KTS => Err("tailwind"),
        _ => Ok(()),
    }
}

/// What the crew says when the arrival cannot keep the runway chosen at the
/// arm. Pure.
pub(super) fn overruled_line(planned: &str, now: Option<&str>, why: &str) -> Option<String> {
    let now = now.filter(|n| !n.eq_ignore_ascii_case(planned))?;
    let (was, now) = (
        crate::crew_radio::spell_runway(planned),
        crate::crew_radio::spell_runway(now),
    );
    Some(match why {
        "tailwind" => format!(
            "The wind has gone round: too much tailwind for runway {was}. Setting up for runway {now}."
        ),
        "no_approach" => format!(
            "There's no approach I can fly to runway {was}. Setting up for runway {now}."
        ),
        _ => format!("Runway {was} isn't one we can land on. Setting up for runway {now}."),
    })
}

/// The runway the destination's tower has given an arrival (its `Inbound`
/// clearance), waited for up to [`TOWER_RUNWAY_WAIT_SECS`] after the inbound
/// call; `None` with no reply, another station on the frequency, or no call.
pub(super) async fn tower_arrival_runway(dest_khz: Option<i64>) -> Option<String> {
    let dest_khz = dest_khz?;
    let deadline = Instant::now() + Duration::from_secs_f64(TOWER_RUNWAY_WAIT_SECS);
    loop {
        if crate::party_line::home_tower_khz() == Some(dest_khz)
            && matches!(
                crate::party_line::clearance_state(),
                Some(crate::tower::State::Inbound | crate::tower::State::ClearedToLand)
            )
        {
            if let Some(r) = crate::party_line::clearance_runway() {
                return Some(r);
            }
        }
        if Instant::now() >= deadline || !INBOUND_MADE.load(SeqCst) {
            return None;
        }
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
}

/// How long the arrival waits for the tower's answer to the inbound call.
pub(super) const TOWER_RUNWAY_WAIT_SECS: f64 = 10.0;

/// The line said when the tower clears a runway the planned SID does not
/// depart from; `None` when they agree or either is unknown. Pure.
pub fn departure_runway_mismatch(planned: Option<&str>, cleared: Option<&str>) -> Option<String> {
    let (p, c) = (planned?, cleared?);
    (!p.eq_ignore_ascii_case(c)).then(|| {
        format!(
            "The tower has given us runway {}, and the planned departure is from runway {}. \
             I can't fly that departure from here.",
            crate::crew_radio::spell_runway(c),
            crate::crew_radio::spell_runway(p)
        )
    })
}

/// Re-picks of the arrival runway one flight allows: a wind sitting on the
/// fence would otherwise swap STARs every few minutes.
pub const MAX_ARRIVAL_REPICKS: u32 = 2;
/// A wind re-pick is judged no closer than this to the destination: inside
/// it the arrival stage's own pick (`build_arrival`) has the runway.
pub const REPICK_MIN_DEST_NM: f64 = 25.0;
/// How often the destination's METAR is read for a re-pick.
pub const REPICK_CHECK_SECS: f64 = 60.0;

/// When the arrival runway chosen at the arm is re-picked in flight (#79):
/// the METAR moves, or the destination's tower names another end. Pure, fed
/// by the enroute wait (`wait_enroute_working_the_radio`).
///
/// The tower outranks the wind and is taken at once. A wind pick must be the
/// same on two readings in a row (a METAR on the fence, a file mid-write),
/// and nothing moves a runway the mission does not hold (a circuit, a plan
/// with none: `retarget_arrival_runway`'s own rule) or more than
/// [`MAX_ARRIVAL_REPICKS`] times.
#[derive(Debug, Default)]
pub struct RepickWatch {
    pub(super) wind_seen: Option<String>,
    pub(super) repicks: u32,
}

impl RepickWatch {
    /// The runway to move to and why ("tower" | "wind"), or `None`.
    pub fn decide(
        &mut self,
        current: Option<&str>,
        tower: Option<&str>,
        wind: Option<&str>,
    ) -> Option<(String, &'static str)> {
        let current = current?;
        if self.repicks >= MAX_ARRIVAL_REPICKS {
            return None;
        }
        let differs = |r: &str| !r.trim().eq_ignore_ascii_case(current.trim());
        if let Some(t) = tower.filter(|t| !t.trim().is_empty() && differs(t)) {
            self.repicks += 1;
            self.wind_seen = None;
            return Some((t.trim().to_string(), "tower"));
        }
        // With the tower on frequency and naming our end, the wind has no say.
        if tower.is_some() {
            self.wind_seen = None;
            return None;
        }
        match wind.filter(|w| differs(w)) {
            Some(w) if self.wind_seen.as_deref().is_some_and(|s| s.eq_ignore_ascii_case(w)) => {
                self.repicks += 1;
                self.wind_seen = None;
                Some((w.to_string(), "wind"))
            }
            Some(w) => {
                self.wind_seen = Some(w.to_string());
                None
            }
            None => {
                self.wind_seen = None;
                None
            }
        }
    }
}

/// The destination runway the surface wind picks now, or `None` in a calm
/// (under [`WIND_DECISIVE_KTS`]: the plan's end stands). Pure.
pub fn wind_runway(runways: &[crate::nav::Runway], wind: Option<(f64, f64)>) -> Option<String> {
    let (dir, kts) = wind.filter(|(_, k)| *k >= WIND_DECISIVE_KTS)?;
    crate::nav::best_runway_for_wind(runways, dir, kts).map(|(r, _, _)| r)
}

/// The arrival as the enroute leg carries it: what a re-pick reads and moves.
pub struct ArrivalRepick {
    pub dest_icao: String,
    /// The destination's reference point and runways (apt.dat), read once.
    pub dest_pos: Option<(f64, f64)>,
    pub runways: Vec<crate::nav::Runway>,
    pub star: Option<String>,
    pub fms_payload: String,
    pub watch: RepickWatch,
    pub last_check: Option<Instant>,
}

impl ArrivalRepick {
    /// The route's arrival, the destination read from apt.dat (none found:
    /// no wind re-pick, the tower's still counts).
    pub fn of(route: &Route, apt_dat: &str) -> Self {
        let apt = crate::nav::find_airport(apt_dat, &route.dest_icao).ok().flatten();
        ArrivalRepick {
            dest_icao: route.dest_icao.clone(),
            dest_pos: apt.as_ref().and_then(|a| a.position()),
            runways: apt.map(|a| a.runways).unwrap_or_default(),
            star: route.arrival_star.clone(),
            fms_payload: route.fms_payload.clone(),
            watch: RepickWatch::default(),
            last_check: None,
        }
    }
}

/// A re-pick's splice for the follower flying `legs` at `idx`: the STAR
/// serving `rwy` from the legs ahead (`plan_audit::star_splice`, the arm's
/// rule), the new STAR's windows clamped into it (`nav::star_constraints`,
/// `procedure::apply_arrival_constraints`), and the whole new route for the
/// GPS. `None`: no STAR from here serves `rwy`.
#[allow(clippy::too_many_arguments)]
pub fn arrival_splice(
    cifp_dir: &str,
    fix_path: &str,
    nav_path: &str,
    apt: &crate::nav::AirportInfo,
    dest: &str,
    rwy: &str,
    star: Option<&str>,
    legs: &[crate::nav::ProcLeg],
    idx: usize,
    why: &str,
) -> Option<(crate::procedure::Splice, String, Vec<crate::nav::ProcLeg>)> {
    let s = crate::plan_audit::star_splice(cifp_dir, fix_path, nav_path, apt, dest, rwy, star, legs, idx)?;
    let mut full: Vec<crate::nav::ProcLeg> = legs[..s.entry_index].to_vec();
    full.extend(s.legs);
    let fixes: Vec<String> = full.iter().map(|l| l.fix_ident.clone()).collect();
    let fixes: Vec<&str> = fixes.iter().map(String::as_str).collect();
    let cons = crate::nav::star_constraints(cifp_dir, dest, &s.ident, Some(rwy), &fixes);
    crate::procedure::apply_arrival_constraints(&mut full, &cons);
    let splice = crate::procedure::Splice {
        from: s.entry_index,
        entry: s.entry,
        legs: full[s.entry_index..].to_vec(),
        constraints: cons,
        why: why.to_string(),
    };
    Some((splice, s.ident, full))
}

/// The line for a re-pick. Pure.
pub fn repick_line(dest: &str, rwy: &str, why: &str, star: Option<(&str, &str)>) -> String {
    let spoken = crate::crew_radio::spell_runway(rwy);
    let head = if why == "tower" {
        format!("Tower wants runway {spoken}")
    } else {
        format!("The wind at {dest} now favours runway {spoken}")
    };
    match star {
        Some((ident, entry)) => format!("{head}, so the {ident} arrival from {entry}."),
        None => format!("{head}, and no arrival from here serves it; I'll fly the route and set up from there."),
    }
}

/// Re-pick the arrival in flight (#79): the new runway through
/// [`retarget_arrival_runway`], the one write point, the STAR serving it
/// spliced into the flying follower (`procedure::offer_splice`) and the GPS
/// reloaded with the procedure flown, as the arm's pick does. Blackbox
/// `full_auto` `arrival_repick`. Returns the line to say.
pub(super) async fn repick_arrival(
    xp: &XPlaneClient,
    config: &Config,
    arr: &mut ArrivalRepick,
    rwy: &str,
    why: &'static str,
) -> Option<String> {
    let was = arrival_runway();
    if !retarget_arrival_runway(rwy, why) {
        return None;
    }
    let ahead = crate::procedure::route_ahead();
    let (cifp, fixp, navp, aptp) = (
        config.cifp_dir.clone(),
        config.fix_data_path.clone(),
        config.nav_data_path.clone(),
        config.apt_data_path.clone(),
    );
    let (dest, star, r) = (arr.dest_icao.clone(), arr.star.clone(), rwy.to_string());
    let spliced = tokio::task::spawn_blocking(move || {
        let (idx, legs) = ahead?;
        let apt = crate::nav::find_airport(&aptp, &dest).ok().flatten()?;
        arrival_splice(&cifp, &fixp, &navp, &apt, &dest, &r, star.as_deref(), &legs, idx, why)
    })
    .await
    .ok()
    .flatten();
    let line = match spliced {
        Some((splice, ident, full)) => {
            let entry = splice.entry.clone();
            event(
                "arrival_repick",
                json!({ "why": why, "was": was, "now": rwy, "star": ident, "was_star": arr.star, "entry": entry,
                    "from": splice.from, "fixes": splice.legs.iter().map(|l| l.fix_ident.clone()).collect::<Vec<_>>(),
                    "constraints": splice.constraints.len() }),
            );
            let offered = crate::procedure::offer_splice(splice);
            if offered {
                arr.star = Some(ident.clone());
                arr.fms_payload = payload_with_legs(&arr.fms_payload, &full);
                if !arr.fms_payload.is_empty() {
                    let _ = xp.set_data("xcopilot/fms/route", arr.fms_payload.as_bytes()).await;
                    let _ = xp.set_int("xcopilot/fms/load", 1).await;
                }
            }
            repick_line(&arr.dest_icao, rwy, why, Some((&ident, &entry)))
        }
        None => {
            event(
                "arrival_repick",
                json!({ "why": why, "was": was, "now": rwy, "star": null, "was_star": arr.star }),
            );
            repick_line(&arr.dest_icao, rwy, why, None)
        }
    };
    Some(line)
}
