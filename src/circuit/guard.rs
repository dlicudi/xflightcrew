//! The pattern's guard: leg closure and targets, the trips, the give-up and
//! the terrain hold.

use super::*;

/// Distance-closure watchdog for one waypoint (see [`STALL_S`]).
#[derive(Debug, Clone, Copy)]
pub struct Closure {
    pub(super) best_m: f64,
    pub(super) best_at_s: f64,
}

impl Closure {
    pub fn new(d_m: f64, now_s: f64) -> Self {
        Closure {
            best_m: d_m,
            best_at_s: now_s,
        }
    }

    /// A watchdog for a leg that BEGINS with a turn: no verdict for
    /// `grace_s`, because an aircraft turning to the fix opens the distance
    /// before it closes it. The follower's leg to FI30 at LIPK started with a
    /// 141 degree turn, was judged "not closing" 45 s in, still turning, and
    /// the whole approach stood down 5 nm from the fix (2026-09-25).
    pub fn with_grace(d_m: f64, now_s: f64, grace_s: f64) -> Self {
        Closure {
            best_m: d_m,
            best_at_s: now_s + grace_s.max(0.0),
        }
    }

    /// Feed this tick's distance. True once per stall: no new low for
    /// [`STALL_S`]. The clock restarts so the next stall is judged afresh.
    pub fn stalled(&mut self, d_m: f64, now_s: f64) -> bool {
        if d_m < self.best_m - STALL_PROGRESS_M {
            self.best_m = d_m;
            self.best_at_s = now_s;
            return false;
        }
        if now_s - self.best_at_s > STALL_S {
            self.best_m = d_m;
            self.best_at_s = now_s;
            return true;
        }
        false
    }
}

/// The leg's altitude target (MSL): the pattern altitude, the downwind's
/// descent from abeam the touchdown point, base and final's path. Pure; the
/// descent is ratcheted through `descent_target`.
#[cfg(test)]
pub fn leg_target_ft(
    leg: Leg,
    a_m: f64,
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    downwind_past_m: f64,
    descent_target: &mut Option<f64>,
) -> f64 {
    leg_target_shaped(
        leg,
        a_m,
        field_elev_ft,
        tpa_msl_ft,
        downwind_past_m,
        DescentShape::default(),
        descent_target,
    )
}

/// [`leg_target_ft`] with the descent the pattern pick settled on
/// ([`DescentShape`]: the downwind's platform and the path's slope, raised
/// only where the ground under the pattern asks for it, #182).
pub fn leg_target_shaped(
    leg: Leg,
    a_m: f64,
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    downwind_past_m: f64,
    shape: DescentShape,
    descent_target: &mut Option<f64>,
) -> f64 {
    match leg {
        // Abeam the touchdown point and later, START DOWN: the 180, where
        // a real pattern reduces power. Holding TPA to the base turn left
        // the whole descent to one short leg it could not fit (see
        // DOWNWIND_DESCENT_AGL_FT). Deliberately a FLAT target and not the
        // path: the path's distance term here is the Manhattan
        // |along| + |across|, which on an OUTBOUND downwind shrinks as the
        // aircraft flies away from the runway and would ratchet us down to
        // a few hundred feet while still abeam. Ratcheted like base, so it
        // can only ever descend.
        Leg::Downwind if a_m < 0.0 => {
            let want = (field_elev_ft + shape.platform_agl_ft).min(tpa_msl_ft);
            let t = descent_target.map_or(want, |d: f64| d.min(want));
            *descent_target = Some(t);
            t
        }
        // Base and final descend the 3 degree PATH, not to a flat height.
        Leg::Base | Leg::Final => {
            // ...and RATCHETED: an approach descends. Whatever the geometry
            // does mid-turn, the target may fall and hold, never climb back:
            // a target that rises is a level-off with the runway ahead, and
            // the aircraft chases it every time.
            let want = approach_leg_alt_on(
                field_elev_ft,
                tpa_msl_ft,
                path_dist_m(leg, a_m, downwind_past_m),
                shape.gp_ft_per_nm,
            );
            let t = descent_target.map_or(want, |d: f64| d.min(want));
            *descent_target = Some(t);
            t
        }
        _ => {
            *descent_target = None;
            tpa_msl_ft
        }
    }
}

/// Higher than this over the pattern altitude is not a pattern (#160).
/// LEMG 13 SF50 (sortie 1790607325): nothing was flying the aircraft, and
/// the circuit rode it from the 1,606 ft pattern to 21,900 ft in circles for
/// 14 minutes, the leg ticks writing VS -500 and a bug on the nose the whole
/// way. At this margin it gives up at 3,100 ft, 42 s after the takeoff.
pub const PATTERN_CEILING_MARGIN_FT: f64 = 1500.0;
/// The margin must hold this long: one frame's glitch is not a climb-away.
pub const PATTERN_GUARD_SUSTAIN_S: f64 = 3.0;
/// A leg lasting this many times its flown length at the pattern speed, plus
/// [`LEG_TIME_MARGIN_S`], is a pattern that is not being flown: that downwind
/// ran 778 s, until the sortie ended, against its 244 s (limit 669).
pub const LEG_TIME_FACTOR: f64 = 2.5;
pub const LEG_TIME_MARGIN_S: f64 = 60.0;
/// The servos out this long: someone took the autopilot out, or it dropped
/// out. The circuit's knobs mean nothing without it.
pub const SERVOS_OFF_S: f64 = 3.0;

/// The margin over the configuration's floor a recovery holds (#181).
pub const RECOVERY_MARGIN_KTS: f64 = 10.0;

/// Why the circuit gave the aircraft back (blackbox `pattern_guard`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PatternTrip {
    /// Above the pattern altitude by `above_ft` for [`PATTERN_GUARD_SUSTAIN_S`].
    AbovePattern { above_ft: f64 },
    /// On one leg `secs` against a `limit_s`.
    LegOverdue { leg: Leg, secs: f64, limit_s: f64 },
    /// The servos read out for `secs`.
    ServosOff { secs: f64 },
    /// The autopilot did not hold the climb (#181, `climb_watch`): `ias`
    /// and the climb `vs` when it was taken.
    ClimbNotHeld {
        why: crate::climb_watch::Why,
        ias: f64,
        vs: f64,
    },
}

impl PatternTrip {
    /// The blackbox's `why`.
    pub fn why(&self) -> &'static str {
        match self {
            PatternTrip::AbovePattern { .. } => "above_pattern",
            PatternTrip::LegOverdue { .. } => "leg_overdue",
            PatternTrip::ServosOff { .. } => "servos_off",
            PatternTrip::ClimbNotHeld { .. } => "climb_not_held",
        }
    }

    /// The first half of the spoken give-up. No dashes (TTS).
    pub fn words(&self) -> String {
        match self {
            PatternTrip::AbovePattern { above_ft } => format!(
                "Circuit abandoned, {:.0} feet above pattern height.",
                (above_ft / 100.0).round() * 100.0
            ),
            PatternTrip::LegOverdue { leg, secs, .. } => format!(
                "Circuit abandoned, {} for {:.0} minutes.",
                format!("{leg:?}").to_lowercase(),
                (secs / 60.0).max(1.0).round()
            ),
            PatternTrip::ServosOff { .. } => "The autopilot is off. Circuit cancelled.".into(),
            PatternTrip::ClimbNotHeld { why, .. } => format!("{} Circuit cancelled.", why.line()),
        }
    }

    /// The autopilot's pitch moves nothing (#181): hand flown, never given
    /// back to it.
    pub fn ap_has_no_pitch(&self) -> bool {
        matches!(self, PatternTrip::ClimbNotHeld { why, .. } if why.ap_has_no_pitch())
    }
}

/// How long `leg` may take before [`PatternGuard`] calls it overdue: its
/// length in the pattern's own geometry plus its turn, flown at the pattern
/// speed, times [`LEG_TIME_FACTOR`], plus [`LEG_TIME_MARGIN_S`]. Pure.
pub fn leg_limit_s(leg: Leg, g: &Geometry, runway_len_m: f64, pattern_kts: f64) -> f64 {
    let radius = g.lead_m / 0.9;
    let turn = std::f64::consts::FRAC_PI_2 * radius;
    let upwind = runway_len_m.max(0.0) + g.upwind_past_m;
    let dist = match leg {
        Leg::Upwind => upwind + turn,
        Leg::Crosswind | Leg::Base => g.width_m + turn,
        Leg::Downwind => upwind + g.downwind_past_m + 2.0 * turn,
        Leg::Final => g.downwind_past_m + turn,
    };
    let mps = pattern_kts.max(40.0) * 0.514_444;
    // An extended downwind (#188) is still the pattern, and so is its longer
    // final: the extension's own time, once (not factored, so the #160
    // downwind that never ended still trips before its sortie did).
    let extension = match leg {
        Leg::Downwind | Leg::Final => MAX_DOWNWIND_EXTEND_M / mps,
        _ => 0.0,
    };
    LEG_TIME_FACTOR * dist / mps + LEG_TIME_MARGIN_S + extension
}

/// The circuit's own give-up (#160): the aircraft far above the pattern, a
/// leg that never ends, or the servos out. Fed every tick; a failed read is
/// no measurement and changes no clock. Pure.
#[derive(Debug, Default, Clone)]
pub struct PatternGuard {
    pub(super) leg: Option<Leg>,
    pub(super) leg_t0: f64,
    pub(super) above_since: Option<f64>,
    pub(super) servos_off_since: Option<f64>,
}

impl PatternGuard {
    /// `pattern_ft` is the height the pattern is flying to (the pattern
    /// altitude, or a terrain floor over it); `servos` the servos' reading.
    pub fn step(
        &mut self,
        now_s: f64,
        leg: Leg,
        alt_msl_ft: f64,
        pattern_ft: f64,
        leg_limit_s: f64,
        servos: Option<bool>,
    ) -> Option<PatternTrip> {
        if self.leg != Some(leg) {
            self.leg = Some(leg);
            self.leg_t0 = now_s;
        }
        if alt_msl_ft.is_finite() && pattern_ft.is_finite() {
            let above_ft = alt_msl_ft - pattern_ft;
            if above_ft > PATTERN_CEILING_MARGIN_FT {
                let since = *self.above_since.get_or_insert(now_s);
                if now_s - since >= PATTERN_GUARD_SUSTAIN_S {
                    return Some(PatternTrip::AbovePattern { above_ft });
                }
            } else {
                self.above_since = None;
            }
        }
        match servos {
            Some(false) => {
                let since = *self.servos_off_since.get_or_insert(now_s);
                if now_s - since >= SERVOS_OFF_S {
                    return Some(PatternTrip::ServosOff {
                        secs: now_s - since,
                    });
                }
            }
            Some(true) => self.servos_off_since = None,
            None => {}
        }
        let secs = now_s - self.leg_t0;
        (leg_limit_s.is_finite() && secs > leg_limit_s).then_some(PatternTrip::LegOverdue {
            leg,
            secs,
            limit_s: leg_limit_s,
        })
    }
}

/// Hand the aircraft back FLYABLE and say so loudly. Wings to the heading
/// flown and VS 0 on an autopilot we own (the servos in if they read out),
/// the speed to the governor at `hold_kts`; a pilot's disconnect is left
/// alone. Only what reads back is claimed.
#[allow(clippy::too_many_arguments)]
/// The circuit's lateral re-engage, one tick: the servos pressed through
/// [`crate::ap_engage::ServoWatch`] (flight director first, re-pressed until
/// they read in, `ap_engage` / `ap_engaged` on the tape under `site`) while
/// the heading mode reads off (`wanted`). It used to press `servos_on` alone,
/// every tick, blind: on an FD airframe a press that cannot hold (refactor
/// audit 2026-10-04 #5). The servos read OUT with the mode on is the
/// `PatternGuard`'s, never re-pressed here.
#[allow(clippy::too_many_arguments)]
pub(super) async fn reengage_lateral(
    xp: &dyn crate::xplane::XPlane,
    watch: &mut crate::ap_engage::ServoWatch,
    fd: &[String],
    now_s: f64,
    wanted: bool,
    servos: Option<f64>,
    ias_kts: f64,
    site: &str,
) {
    let st = watch.step(now_s, wanted, servos, ias_kts);
    if st.fire {
        crate::ap_engage::press_logged(xp, fd, "circuit", site, st.try_n).await;
    }
    if st.confirmed_now {
        crate::ap_engage::record_engaged("circuit", site, st.try_n);
    }
}

pub(super) async fn give_up_pattern(
    xp: &XPlaneClient,
    base_url: &str,
    voice: &Option<Voice>,
    trip: PatternTrip,
    leg: Leg,
    alt_msl_ft: f64,
    pattern_ft: f64,
    mag_var: f64,
    hold_kts: f64,
    floor_kts: Option<f64>,
) {
    lower_flag(xp);
    // Never "your aircraft" to an aircraft that is slow, stalling or upset,
    // nor to one whose autopilot would not hold the climb (#181: handed
    // back at 73 kt, the 737 rolled past 160 degrees with nobody flying).
    // The recovery takes it: servos out, flown level, the autopilot given
    // it back only once it is seen to hold.
    let upset = match trip {
        PatternTrip::ClimbNotHeld { why, .. } if why.ap_has_no_pitch() => {
            Some("no_pitch_authority")
        }
        PatternTrip::ClimbNotHeld { .. } => Some("climb_not_held"),
        _ => crate::tools::recover::read_unflyable(xp, floor_kts).await,
    };
    if let Some(what) = upset {
        let no_pitch = trip.ap_has_no_pitch();
        let started = crate::tools::recover::take_over(
            base_url.to_string(),
            voice.clone(),
            crate::tools::recover::TakeOver {
                why: format!("circuit_{what}"),
                min_safe_ias: floor_kts.map(|f| f + RECOVERY_MARGIN_KTS),
                hold_kts: Some(hold_kts),
                ap_no_pitch: no_pitch,
                // The pattern's height: a climb the takeoff began is kept.
                hold_ft: no_pitch.then_some(pattern_ft).filter(|h| h.is_finite()),
            },
        );
        let (watch, ias, vs) = match trip {
            PatternTrip::ClimbNotHeld { why, ias, vs } => (Some(why.word()), Some(ias), Some(vs)),
            _ => (None, None, None),
        };
        crate::blackbox::record(
            "circuit",
            serde_json::json!({
                "event": "pattern_guard", "why": trip.why(), "leg": format!("{leg:?}"),
                "alt_msl": alt_msl_ft, "pattern_ft": pattern_ft, "hold_kts": hold_kts,
                "recovering": what, "recovery_started": started, "floor_kts": floor_kts,
                "watch": watch, "ias": ias, "vs": vs,
            }),
        );
        // The recovery says the missing pitch itself, first thing.
        let line = if no_pitch {
            "Circuit cancelled.".to_string()
        } else {
            format!("{} I have control, recovering.", trip.words())
        };
        alert(voice, Priority::Warning, &line);
        return;
    }
    let psi = xp.get_float(PSI_TRUE).await.ok().filter(|v| v.is_finite());
    let hdg_mag = psi.map(|p| true_to_mag(p, mag_var).round().rem_euclid(360.0));
    let ours = !matches!(trip, PatternTrip::ServosOff { .. });
    if ours {
        let servos = crate::autoflight::read_status(&*xp, SERVOS_ON).await.ok();
        if servos.is_some_and(|v| v < 0.5) {
            // FD first, on the tape (`ap_engage`); the read-back below
            // judges whether it is holding.
            crate::ap_engage::press_logged(
                xp, &crate::ap_engage::fd_commands(), "circuit", "pattern_guard", 1,
            )
            .await;
        }
        if let Some(h) = hdg_mag {
            write_bug(xp, h).await;
        }
        if crate::autoflight::read_status(&*xp, VS_STATUS).await.is_ok_and(|v| v <= 0.5) {
            let _ = autoflight::press(xp, Button::VerticalSpeed).await;
        }
        let _ = autoflight::set_dial(xp, Dial::Vs, 0.0).await;
    }
    let (servos_after, hdg_st, vs_st) = tokio::join!(
        crate::autoflight::read_status(&*xp, SERVOS_ON),
        crate::autoflight::read_status(&*xp, HDG_STATUS),
        crate::autoflight::read_status(&*xp, VS_STATUS)
    );
    let holding = ours
        && [&servos_after, &hdg_st, &vs_st]
            .iter()
            .all(|r| r.as_ref().is_ok_and(|v| *v > 0.5));
    let (leg_secs, limit_s, above_ft) = match trip {
        PatternTrip::LegOverdue { secs, limit_s, .. } => (Some(secs), Some(limit_s), None),
        PatternTrip::AbovePattern { above_ft } => (None, None, Some(above_ft)),
        PatternTrip::ServosOff { secs } => (Some(secs), None, None),
        PatternTrip::ClimbNotHeld { .. } => (None, None, None),
    };
    crate::blackbox::record(
        "circuit",
        serde_json::json!({
            "event": "pattern_guard", "why": trip.why(), "leg": format!("{leg:?}"),
            "alt_msl": alt_msl_ft, "pattern_ft": pattern_ft, "above_ft": above_ft,
            "secs": leg_secs, "limit_s": limit_s, "hdg_mag": hdg_mag,
            "servos_after": servos_after.ok(), "holding": holding, "hold_kts": hold_kts,
        }),
    );
    let tail = match (holding, hdg_mag) {
        (true, Some(h)) => format!("Autopilot on heading {h:03.0}, levelling off. Your aircraft."),
        (true, None) => "Autopilot levelling off. Your aircraft.".to_string(),
        (false, _) if ours => "The autopilot is not flying it. Your aircraft.".to_string(),
        (false, _) => "Your aircraft.".to_string(),
    };
    alert(
        voice,
        Priority::Warning,
        &format!("{} {tail}", trip.words()),
    );
    crate::tools::speedhold::arm_flight(base_url.to_string(), voice.clone(), hold_kts);
}

/// The climb rate a terrain hold's give-up clock credits, feet per minute:
/// a laden light single's, so the clock never runs out on a climb that is
/// going.
pub(super) const TERRAIN_HOLD_CREDIT_FPM: f64 = 300.0;

/// How long a terrain hold's climb may take before the circuit's guard
/// ([`PatternGuard`]) gives it back: the climb at
/// [`TERRAIN_HOLD_CREDIT_FPM`] plus [`LEG_TIME_MARGIN_S`]. Pure.
pub fn terrain_hold_limit_s(alt_ft: f64, hold_ft: f64) -> f64 {
    let climb = if alt_ft.is_finite() && hold_ft.is_finite() {
        (hold_ft - alt_ft).max(0.0)
    } else {
        0.0
    };
    climb / TERRAIN_HOLD_CREDIT_FPM * 60.0 + LEG_TIME_MARGIN_S
}

/// The spoken airborne refusal (#182): the reason with its numbers, then
/// what the autopilot is doing about it. No dashes (TTS). Pure.
pub fn terrain_hold_line(why: &str, hold_ft: f64) -> String {
    format!("{why} Climbing straight ahead to {hold_ft:.0} feet and holding it.")
}

/// ...and the hand-over once level there: autoland's words. Pure.
pub fn terrain_hold_level_line(hold_ft: f64) -> String {
    format!(
        "Level at {hold_ft:.0} feet, straight ahead, the autopilot holding heading and \
         altitude. Your aircraft."
    )
}

/// Airborne and no circuit clears the ground (#182; LXGB, the Rock route
/// and the rectangle both refused after the takeoff): the reversal's
/// refusal ([`crate::reversal::TerrainHold`]), never "your aircraft" low
/// over the hills. Climb straight ahead on the autopilot to 1,000 ft over
/// the highest ground under and 12 nm ahead
/// ([`crate::reversal::hold_altitude_ahead_ft`], the corridors' highest
/// standing in for an unanswered probe), the altitude dial on it and the
/// governor on the configuration's speed, said with the numbers; handed
/// over only once level there. The circuit's guard still gives it back
/// loudly (servos out, a climb that never ends), through the recovery
/// when slow or upset.
#[allow(clippy::too_many_arguments)]
pub(super) async fn hold_clear_of_terrain(
    xp: &XPlaneClient,
    base_url: &str,
    voice: &Option<Voice>,
    why: &str,
    corridor_max_ft: Option<f64>,
    default_platform_ft: f64,
    fallback_hdg_true: f64,
    mag_var: f64,
    hold_kts: f64,
    floor_kts: Option<f64>,
) {
    let (pos, hpath_r) = tokio::join!(sim::position::read(xp), xp.get_float(HPATH));
    let (lat, lon) = sim::position::lat_lon(&pos);
    let alt_ft = sim::position::field(&pos, |p| p.elev_m)
        .map(|m| m * M_TO_FT)
        .unwrap_or(f64::NAN);
    let psi = sim::position::field(&pos, |p| p.psi_true_deg)
        .ok()
        .filter(|v| v.is_finite());
    let track = hpath_r.ok().filter(|v| v.is_finite()).or(psi);
    let hdg_true = psi.unwrap_or(fallback_hdg_true);
    let (ahead_ft, hold_ft) = match (lat, lon) {
        (Ok(lat), Ok(lon)) => {
            crate::reversal::hold_altitude_ahead_ft(
                xp,
                (lat, lon, track.unwrap_or(hdg_true), alt_ft),
                corridor_max_ft,
                default_platform_ft,
            )
            .await
        }
        _ => (
            None,
            crate::reversal::hold_altitude_ft(None, corridor_max_ft, alt_ft, default_platform_ft),
        ),
    };
    let mut hold = crate::reversal::TerrainHold::refused(hdg_true, hold_ft);
    crate::blackbox::record(
        "circuit",
        serde_json::json!({
            "event": "terrain_hold", "hold_ft": hold_ft, "ahead_ft": ahead_ft,
            "corridor_max_ft": corridor_max_ft, "alt": alt_ft, "hdg_true": hdg_true,
            "hold_kts": hold_kts, "why": why,
        }),
    );
    alert(voice, Priority::Warning, &terrain_hold_line(why, hold_ft));
    // The autopilot's own capture holds the level: the dial on the hold.
    let bug_mag = true_to_mag(hdg_true, mag_var).rem_euclid(360.0);
    if crate::autoflight::read_status(&*xp, SERVOS_ON).await.is_ok_and(|v| v < 0.5) {
        // FD first, on the tape (`ap_engage`); the hold's own guard watches
        // the servos from here.
        crate::ap_engage::press_logged(
            xp, &crate::ap_engage::fd_commands(), "circuit", "terrain_hold", 1,
        )
        .await;
    }
    let _ = autoflight::set_dial(xp, Dial::Altitude, hold_ft).await;
    write_bug(xp, bug_mag).await;
    crate::tools::speedhold::arm_flight(base_url.to_string(), voice.clone(), hold_kts);
    let limit_s = terrain_hold_limit_s(alt_ft, hold_ft);
    let mut guard = PatternGuard::default();
    let t0 = std::time::Instant::now();
    let mut last_vs: Option<f64> = None;
    let mut tick: u32 = 0;
    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if xp.cancelled() {
            stand_down(&xp, voice, "Circuit cancelled. Your aircraft.", "abort_terrain_hold");
            return;
        }
        if crate::xplane::sim_frozen(xp).await {
            continue;
        }
        let (pos, ias_r, servos_r, hdg_st, vs_st) = tokio::join!(
            sim::position::read(xp),
            xp.get_float(IAS),
            crate::autoflight::read_status(&*xp, SERVOS_ON),
            crate::autoflight::read_status(&*xp, HDG_STATUS),
            crate::autoflight::read_status(&*xp, VS_STATUS),
        );
        // A failed read is no measurement: the tick is skipped.
        let Some(alt) = sim::position::field(&pos, |p| p.elev_m)
            .ok()
            .map(|m| m * M_TO_FT)
            .filter(|v| v.is_finite())
        else {
            continue;
        };
        tick += 1;
        if hold.level_now(alt) {
            let _ = autoflight::set_dial(xp, Dial::Vs, 0.0).await;
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "terrain_hold_level", "hold_ft": hold.alt_ft,
                                   "alt": alt, "secs": t0.elapsed().as_secs_f64()}),
            );
            stand_down(&xp, voice, &terrain_hold_level_line(hold.alt_ft), TERRAIN_REFUSED);
            return;
        }
        let servos = servos_r
            .as_ref()
            .ok()
            .filter(|v| v.is_finite())
            .map(|v| *v > 0.5);
        if let Some(trip) = guard.step(
            t0.elapsed().as_secs_f64(),
            Leg::Upwind,
            alt,
            hold.alt_ft,
            limit_s,
            servos,
        ) {
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "terrain_hold_given_back", "why": trip.why(),
                                   "hold_ft": hold.alt_ft, "alt": alt}),
            );
            *STOOD_DOWN.lock().unwrap_or_else(|e| e.into_inner()) = Some(TERRAIN_REFUSED);
            give_up_pattern(
                xp,
                base_url,
                voice,
                trip,
                Leg::Upwind,
                alt,
                hold.alt_ft,
                mag_var,
                hold_kts,
                floor_kts,
            )
            .await;
            return;
        }
        // Modes re-engaged only when their status reads off, the dial
        // rewritten after (an engage syncs it away). The capture at the
        // hold is ALT HOLD, not a dropped VS mode.
        let hdg = autoflight::bring_heading(xp, hdg_st.ok(), "circuit_terrain_hold").await;
        if hdg.pressed(Button::Heading) {
            write_bug(xp, bug_mag).await;
        }
        if hold.alt_ft - alt > crate::reversal::HOLD_LEVEL_FT
            && autoflight::bring_vs(xp, vs_st.ok(), "circuit_terrain_hold")
                .await
                .pressed(Button::VerticalSpeed)
        {
            last_vs = None;
        }
        let ias = ias_r.ok().filter(|v| v.is_finite());
        let slow = matches!((ias, floor_kts), (Some(i), Some(f)) if i < f);
        let vs = crate::reversal::platform_vs_fpm(
            hold.alt_ft,
            alt,
            slow,
            crate::autoland::max_climb_fpm(),
            crate::procedure::descent_fpm(),
        );
        if last_vs.is_none_or(|v| (v - vs).abs() >= 10.0) {
            let _ = autoflight::set_dial(xp, Dial::Vs, vs).await;
            last_vs = Some(vs);
        }
        if tick % 10 == 1 {
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "terrain_hold_tick", "alt": alt, "hold_ft": hold.alt_ft,
                                   "vs_cmd": vs, "ias": ias, "slow": slow, "servos": servos}),
            );
        }
    }
}

/// The next leg in sequence, or `None` past Final.
pub fn next_leg(leg: Leg) -> Option<Leg> {
    match leg {
        Leg::Upwind => Some(Leg::Crosswind),
        Leg::Crosswind => Some(Leg::Downwind),
        Leg::Downwind => Some(Leg::Base),
        Leg::Base => Some(Leg::Final),
        Leg::Final => None,
    }
}

/// The pure leg-advance decision — POSITION-based, so the sequencing is
/// testable without a sim and immune to how long the autopilot's turns take.
/// `a_m` is distance along the takeoff direction from the departure threshold
/// (negative = on the approach side, before the threshold); `x_m` is cross-
/// track, right of the takeoff course positive. `left` selects the pattern
/// side. Each leg advances when the aircraft reaches the geometric point LESS
/// the turn-anticipation lead, so the slow AP turn rolls out on the line.
pub fn pattern_advance(
    leg: Leg,
    a_m: f64,
    x_m: f64,
    g: &Geometry,
    runway_len_m: f64,
    left: bool,
) -> bool {
    // The pattern sits on the LEFT (negative cross-track) for a standard
    // circuit; `side * x_m` is then "distance out to the pattern side", always
    // positive as we fly away from the runway on crosswind.
    let side = if left { -1.0 } else { 1.0 };
    match leg {
        // Past the departure end — turn crosswind while still climbing to
        // pattern altitude (a real pattern climbs on the crosswind/downwind,
        // not four kilometres straight ahead waiting to reach TPA first).
        Leg::Upwind => a_m >= runway_len_m + g.upwind_past_m,
        // Reaching the downwind spacing — led, so the rollout lands AT it.
        Leg::Crosswind => side * x_m >= g.width_m - g.lead_m,
        // Flown back down the downwind until past the landing threshold by
        // enough to build a normal final — led for the base turn.
        Leg::Downwind => a_m <= -(g.downwind_past_m - g.lead_m),
        // Base has brought us back within one turn radius of the extended
        // centreline: the 90° turn to final rolls out on it.
        Leg::Base => side * x_m <= g.final_capture_m,
        Leg::Final => false,
    }
}
