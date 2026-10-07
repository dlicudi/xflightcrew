//! The authored circuit (LXGB's Rock route): its waypoints, vertical, join,
//! terrain probe and `fly_authored_route`, whole.

use super::*;

/// How close (metres) to an authored waypoint counts as reaching it. Generous,
/// so the slow AP turn rolls through the corner instead of circling the point.
pub(super) const AUTHORED_CAPTURE_M: f64 = 350.0;

/// How far the Rock circuit's downwind runs from the runway line, metres:
/// SOUTH of Europa Point (4,500 m from the centreline at the lighthouse),
/// so the loop goes AROUND the Rock, over the Strait. The old 1.3 x pattern
/// width put it 2,050 m out, over Middle Hill (2026-09-15, runway 09): the
/// probe read 1,098 ft under the leg, the raised pattern altitude cleared
/// that on paper, and the aircraft crossed the ridge at 1,412 ft STILL
/// CLIMBING, 454 ft over the ground, with the terrain check satisfied.
pub(super) const LXGB_ROCK_CLEAR_M: f64 = 5_400.0;

/// The most climb the authored follower commands, feet per minute (the
/// `vs_raw` clamp), and the fraction of it the pre-flight climb model
/// credits: measured 550 to 650 fpm at 73 kt on the 172 against a 700 clamp.
pub(super) const AUTHORED_CLIMB_FPM: f64 = 700.0;

/// ...or the airframe's own, when it climbs harder than a 172 does. The
/// figure above is a measurement OF a 172 ("550 to 650 fpm at 73 kt"), which
/// is exactly the kind of number that has no business capping a jet.
pub(super) fn authored_climb_fpm() -> f64 {
    crate::procedure::climb_fpm().max(AUTHORED_CLIMB_FPM)
}
pub(super) const AUTHORED_CLIMB_CREDIT: f64 = 0.75;

/// Feet gained per metre flown at the pattern speed, for the climb model.
pub(super) fn authored_climb_gradient(pattern_kts: f64) -> f64 {
    let m_per_min = pattern_kts.max(40.0) * M_PER_NM / 60.0;
    AUTHORED_CLIMB_FPM * AUTHORED_CLIMB_CREDIT / m_per_min
}

/// One authored circuit waypoint, in the departure-runway frame (see
/// `frame_to_latlon`): `a_m` along the takeoff course from the threshold, `x_m`
/// right of it, and the target altitude (MSL) to be at approaching it.
#[derive(Debug, Clone, Copy)]
pub(super) struct AuthoredWpt {
    pub(super) a_m: f64,
    pub(super) x_m: f64,
    pub(super) alt_msl_ft: f64,
}

/// A hand-authored circuit for a field where the parametric rectangle can't be
/// flown clear — Gibraltar (LXGB), where R-164 (Spanish restricted, GND-FL300)
/// blankets both sides of any standard pattern (`choose_pattern` returns
/// `airspace_unavoidable`). The airspace can't be dodged — nothing at LXGB can,
/// it's the real political geometry — but the ROCK'S TERRAIN can, and a tight,
/// sane track over the bay beats flying the rectangle through the granite. The
/// route stays on the low-terrain side `choose_pattern` picked, holds the
/// downwind WIDE to carry the ground track around the Rock's flank, and rejoins
/// a straight final where autoland takes it. Scaled to the same geometry, so
/// the pattern speed-hold keeps it this size (at climb power it balloons back
/// over the Rock). `None` at every other field. DRAFT coordinates — the caller
/// validates every waypoint against the live terrain probe before flying it.
pub(super) fn authored_circuit(
    rwy: &crate::runway::Runway,
    geom: &Geometry,
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    left: bool,
) -> Option<Vec<AuthoredWpt>> {
    if rwy.icao != "LXGB" {
        return None;
    }
    let side = if left { -1.0 } else { 1.0 };
    let handoff = field_elev_ft + HANDOFF_AGL_FT;
    let w = geom.width_m;
    let up = rwy.length_m + geom.upwind_past_m;
    let dn = geom.downwind_past_m;
    // The downwind runs south of Europa Point, whatever the airframe's
    // pattern width: the Rock is a fixed piece of granite, not a fraction
    // of the turn radius.
    let out = (w * 1.3).max(LXGB_ROCK_CLEAR_M);
    Some(vec![
        // Climb out on the runway heading.
        AuthoredWpt {
            a_m: up,
            x_m: 0.0,
            alt_msl_ft: tpa_msl_ft,
        },
        // Crosswind south over the sea off the Rock's east face, the length
        // of the Rock: the climb to pattern altitude happens here, over
        // water, before any leg turns back toward the granite.
        AuthoredWpt {
            a_m: up,
            x_m: side * out,
            alt_msl_ft: tpa_msl_ft,
        },
        AuthoredWpt {
            a_m: 0.0,
            x_m: side * out,
            alt_msl_ft: tpa_msl_ft,
        },
        AuthoredWpt {
            a_m: -dn,
            x_m: side * out,
            alt_msl_ft: tpa_msl_ft,
        },
        // Base back over the water, descending to the handoff height.
        AuthoredWpt {
            a_m: -dn * 1.2,
            x_m: side * w * 0.6,
            alt_msl_ft: handoff,
        },
        // Rejoin the centreline and hand to autoland on a NORMAL final: ~2 nm
        // out (`-dn`, not `-dn * 0.5`), so the handoff height sits on a ~3°
        // glidepath, not a 6.5° dive autoland can't capture. Handing off at 1 nm
        // / 700 ft looped the LXGB approach into a permanent go-around
        // (2026-07-26).
        AuthoredWpt {
            a_m: -dn,
            x_m: 0.0,
            alt_msl_ft: handoff,
        },
    ])
}

/// How the authored route flies this tick's waypoint altitude (#80).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AuthoredVertical {
    /// A dial write owed: the waypoint's level to the hundred.
    pub dial_ft: Option<f64>,
    /// The autopilot's altitude hold has the level: the VS law stands off.
    pub ap_level: bool,
}

/// A captured hold this far from the level is no hold of ours: the dial was
/// just moved and the autopilot is still holding the old level.
pub const AUTHORED_HOLD_BAND_FT: f64 = 200.0;

/// The authored route's vertical where the airframe declares
/// `ap_altitude_capture`: each waypoint's altitude is a level target, so it
/// goes on the altitude dial (`circuit_dial_ft` with no rate: the route has
/// no glidepath, autoland parks the dial at the handoff), and a CAPTURED hold
/// on that dial within [`AUTHORED_HOLD_BAND_FT`] owns the leg
/// (`ap_holds_level`). The LXGB route flew its levels on the VS law behind
/// whatever the dial held (dev-history 2026-09-21: "the authored LXGB route
/// has no altitude capture"). Off, or with either read failed (no
/// measurement), nothing is written and the VS law flies it as before. Pure.
pub fn authored_vertical(
    ap_capture: bool,
    wp_alt_ft: f64,
    alt_ft: f64,
    dial_ft: Option<f64>,
    alt_hold_status: Option<f64>,
) -> AuthoredVertical {
    let none = AuthoredVertical { dial_ft: None, ap_level: false };
    if !ap_capture || !alt_ft.is_finite() || !wp_alt_ft.is_finite() {
        return none;
    }
    let (Some(dial), Some(hold)) = (
        dial_ft.filter(|v| v.is_finite()),
        alt_hold_status.filter(|v| v.is_finite()),
    ) else {
        return none;
    };
    match circuit_dial_ft(0.0, wp_alt_ft, dial, alt_ft) {
        Some(want) => AuthoredVertical { dial_ft: Some(want), ap_level: false },
        None => AuthoredVertical {
            dial_ft: None,
            ap_level: ap_holds_level(0.0, wp_alt_ft, dial, hold)
                && (alt_ft - wp_alt_ft).abs() <= AUTHORED_HOLD_BAND_FT,
        },
    }
}

/// The waypoint to fly to from `(a_m, x_m)` (runway frame): the end of the
/// route leg the aircraft is nearest to, or the one after it when the
/// aircraft is already past that leg's end. Leg 0 runs from the threshold
/// along the course to waypoint 0. The follower joins there. At LXGB
/// (2026-09-14) the takeoff sank at the AP engage and handed off 1.8 km PAST
/// the climb-out point, and the follower steered back to it: a 270-degree
/// turn at 60 kt, the wrong way round the first leg. Past the last leg →
/// the last waypoint (the autoland handoff).
pub(super) fn join_waypoint(route: &[AuthoredWpt], a_m: f64, x_m: f64) -> usize {
    let mut best: Option<(f64, usize)> = None;
    for (i, wp) in route.iter().enumerate() {
        let (sa, sx) = if i == 0 {
            (0.0, 0.0)
        } else {
            (route[i - 1].a_m, route[i - 1].x_m)
        };
        let (da, dx) = (wp.a_m - sa, wp.x_m - sx);
        let len2 = da * da + dx * dx;
        if len2 < 1.0 {
            continue;
        }
        let t = (((a_m - sa) * da + (x_m - sx) * dx) / len2).clamp(0.0, 1.0);
        let (pa, px) = (sa + t * da, sx + t * dx);
        let dist = ((a_m - pa).powi(2) + (x_m - px).powi(2)).sqrt();
        let target = if t >= 0.999 { i + 1 } else { i };
        if best.is_none_or(|(d, _)| dist < d) {
            best = Some((dist, target));
        }
    }
    best.map_or(0, |(_, t)| t.min(route.len().saturating_sub(1)))
}

/// Spacing of the terrain samples along an authored leg, metres. The Rock's
/// ridge is ~300 m wide; six waypoints probed alone (2026-09-14) sat on sea
/// either side of it while the downwind between them crossed it 523 ft up.
pub(super) const LEG_SAMPLE_M: f64 = 150.0;
/// The most the authored pattern altitude may be raised over its terrain
/// before the route is refused instead.
pub(super) const MAX_TERRAIN_RAISE_FT: f64 = 1000.0;

/// Sample points along every leg of the route in the runway frame, tagged
/// with the index of the waypoint the leg runs INTO (legs 1..). Leg 0, the
/// climb-out from the runway to the first waypoint, is the takeoff's and is
/// not sampled.
pub(super) fn leg_samples(route: &[AuthoredWpt]) -> Vec<(usize, f64, f64)> {
    let mut out = Vec::new();
    for i in 1..route.len() {
        let (p, q) = (route[i - 1], route[i]);
        let len = ((q.a_m - p.a_m).powi(2) + (q.x_m - p.x_m).powi(2)).sqrt();
        let n = ((len / LEG_SAMPLE_M).ceil() as usize).max(1);
        for k in 0..=n {
            let f = k as f64 / n as f64;
            out.push((i, p.a_m + (q.a_m - p.a_m) * f, p.x_m + (q.x_m - p.x_m) * f));
        }
    }
    out
}

/// The highest ground (feet MSL) under each leg of the route, indexed like
/// the waypoints (entry 0 unused). `None` for a leg = the probe did not
/// answer (no plugin): read as clear, the old fail-open behaviour.
pub(super) async fn leg_terrain_ft(
    xp: &XPlaneClient,
    rwy: &crate::runway::Runway,
    route: &[AuthoredWpt],
) -> Vec<Option<f64>> {
    let samples = leg_samples(route);
    let pts: Vec<(f64, f64)> = samples
        .iter()
        .map(|&(_, a, x)| frame_to_latlon(rwy, a, x))
        .collect();
    let mut out = vec![None; route.len()];
    let mut at = 0;
    for chunk in pts.chunks(crate::terrain::MAX_POINTS) {
        if let Some(elev) = crate::terrain::sample_ft(xp, chunk).await {
            for (k, e) in elev.into_iter().enumerate() {
                let leg = samples[at + k].0;
                out[leg] = Some(out[leg].map_or(e, |m: f64| m.max(e)));
            }
        }
        at += chunk.len();
    }
    out
}

/// The altitude (feet MSL) the climb can have reached at each waypoint:
/// the field elevation plus `gradient_ft_per_m` over the path flown from
/// the threshold through every waypoint before it. Uncapped; the caller
/// caps it at the altitude actually asked for.
pub(super) fn climb_reach_ft(route: &[AuthoredWpt], field_elev_ft: f64, gradient_ft_per_m: f64) -> Vec<f64> {
    let mut out = Vec::with_capacity(route.len());
    let (mut pa, mut px, mut len) = (0.0_f64, 0.0_f64, 0.0_f64);
    for w in route {
        len += ((w.a_m - pa).powi(2) + (w.x_m - px).powi(2)).sqrt();
        (pa, px) = (w.a_m, w.x_m);
        out.push(field_elev_ft + gradient_ft_per_m * len);
    }
    out
}

/// Raise the route's pattern altitude until every leg flown AT pattern
/// altitude clears its terrain by [`AUTHORED_TERRAIN_CLEAR_FT`], or refuse. Legs that
/// descend to the handoff height (into a waypoint below the TPA) cannot be
/// raised and are refused when their terrain needs more than the height
/// they end at; a raise past [`MAX_TERRAIN_RAISE_FT`] is refused too.
///
/// "Flown at pattern altitude" is only true once the climb has got there:
/// the follower leaves the takeoff's handoff at ~500 ft and climbs along
/// the route, so a leg with terrain under it is also checked against the
/// altitude the climb ([`climb_reach_ft`], `gradient_ft_per_m`) can have
/// reached at the leg's START, and refused when the terrain needs more.
/// Runway 09 at LXGB (2026-09-15): a 2,108 ft pattern altitude cleared the
/// 1,098 ft under the downwind on paper, and the aircraft entered that leg
/// at 1,130 ft.
/// Returns the route to fly and the pattern altitude it flies.
pub(super) fn raise_for_terrain(
    route: &[AuthoredWpt],
    tpa_msl_ft: f64,
    leg_max_ft: &[Option<f64>],
    field_elev_ft: f64,
    gradient_ft_per_m: f64,
) -> Result<(Vec<AuthoredWpt>, f64), String> {
    let at_tpa = |w: &AuthoredWpt| w.alt_msl_ft >= tpa_msl_ft - 1.0;
    let reach = climb_reach_ft(route, field_elev_ft, gradient_ft_per_m);
    let mut tpa_needed = tpa_msl_ft;
    for i in 1..route.len() {
        let Some(Some(ground)) = leg_max_ft.get(i) else {
            continue;
        };
        let need = ground + AUTHORED_TERRAIN_CLEAR_FT;
        if at_tpa(&route[i - 1]) && at_tpa(&route[i]) {
            tpa_needed = tpa_needed.max(need);
            let entry = reach[i - 1].min(tpa_needed);
            if need > entry {
                return Err(format!(
                    "leg {i} needs {need:.0} ft over {ground:.0} ft ground, entered at {entry:.0} ft in the climb"
                ));
            }
        } else {
            let floor = route[i - 1].alt_msl_ft.min(route[i].alt_msl_ft);
            if need > floor {
                return Err(format!(
                    "leg {i} needs {need:.0} ft over {ground:.0} ft ground, flown down to {floor:.0}"
                ));
            }
        }
    }
    if tpa_needed > tpa_msl_ft + MAX_TERRAIN_RAISE_FT {
        return Err(format!(
            "pattern altitude would need {tpa_needed:.0} ft, {:.0} over the TPA",
            tpa_needed - tpa_msl_ft
        ));
    }
    let raised = route
        .iter()
        .map(|w| {
            let mut w = *w;
            if at_tpa(&w) {
                w.alt_msl_ft = tpa_needed;
            }
            w
        })
        .collect();
    Ok((raised, tpa_needed))
}

/// The authored route checked over its legs' ground ([`leg_terrain_ft`],
/// [`raise_for_terrain`]): the ground per leg and the route to fly at its
/// pattern altitude, or why not. The same answer on the runway and in the
/// air (the route is fixed to the runway, not to the aircraft).
pub(super) async fn check_authored_route(
    xp: &XPlaneClient,
    rwy: &crate::runway::Runway,
    route: &[AuthoredWpt],
    tpa_msl_ft: f64,
    field_elev_ft: f64,
    pattern_kts: f64,
) -> (Vec<Option<f64>>, Result<(Vec<AuthoredWpt>, f64), String>) {
    let leg_max = leg_terrain_ft(xp, rwy, route).await;
    let verdict = raise_for_terrain(
        route,
        tpa_msl_ft,
        &leg_max,
        field_elev_ft,
        authored_climb_gradient(pattern_kts),
    );
    (leg_max, verdict)
}

/// The Rock refused both ways (#182): the spoken line, the rectangle's
/// numbers in it. Pure.
pub fn rock_refused_line(rectangle_line: &str) -> String {
    format!("The Rock circuit isn't clear from here. {rectangle_line}")
}

/// Height above the field at which the authored circuit switches the landing
/// and taxi lights off (the rectangle and full auto's flight do it through
/// the same `lights::after_climb`).
pub(super) const LIGHTS_OFF_AGL_FT: f64 = 1000.0;

/// Fly an authored waypoint route (the LXGB Rock circuit), then hand the final
/// to autoland. Same control surface as the rectangle loop — HDG bug + VS + the
/// pattern speed-hold — but steering great-circle to each waypoint instead of
/// the leg geometry. The last waypoint sits on short final.
///
/// Joins at the nearest leg (`join_waypoint`), never
/// steps the bug against a turn in progress (`bug_toward`), climbs only as
/// fast as the airspeed pays for (`climb_limit_fpm`, the rectangle's own
/// law: this loop clamped a flat 700 fpm and flew the first turn at 60 kt),
/// and gives the aircraft back after [`MAX_STALLS`] legs it could not close.
#[allow(clippy::too_many_arguments)]
pub(super) async fn fly_authored_route(
    xp: XPlaneClient,
    base_url: String,
    voice: Option<Voice>,
    route: Vec<AuthoredWpt>,
    rwy: crate::runway::Runway,
    mag_var: f64,
    pattern_kts: f64,
    full_kts: f64,
    floor_kts: f64,
    callsign: Option<String>,
    apt_dat: String,
    cfg: crate::tools::autopilot::ApConfig,
    approach_ias: f64,
) {
    let rwy_id = rwy.id.clone();
    let mut bug_mag = autoflight::read_dial(&xp, Dial::HeadingBug).await.unwrap_or(rwy.course_deg);
    // A new flight of the laws: whatever the knob holds now is unknown.
    LAST_BUG.store(u64::MAX, std::sync::atomic::Ordering::Relaxed);
    let mut last_vs_cmd: Option<f64> = None;
    let mut prev_ias: Option<(f64, std::time::Instant)> = None;
    let mut ias_trend = 0.0_f64;
    let t_start = std::time::Instant::now();
    let mut bug_hold = BugHold::default();
    // The lateral re-engage (`ap_engage::ServoWatch`): FD first, confirmed.
    let mut servo_watch = crate::ap_engage::ServoWatch::default();
    // Picked from the first good position read, not assumed to be 0.
    let mut wpt_idx: Option<usize> = None;
    let mut closure: Option<Closure> = None;
    let mut stalls = 0u32;
    let mut lights_done = false;
    // The autopilot's own capture on the level legs, where the airframe
    // declares it (`ap_altitude_capture`), as the rectangle does (#80).
    let ap_capture = ap_altitude_capture();
    let mut vs_refused: Option<std::time::Instant> = None;
    let mut vs_refused_said = false;
    loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if xp.cancelled() {
            stand_down(
                &xp,
                &voice,
                "Circuit cancelled. Your aircraft.",
                "abort_authored",
            );
            return;
        }
        if crate::xplane::sim_frozen(&xp).await {
            continue;
        }
        let thr_dref = throttle_dataref();
        // Position, elevation and heading as ONE frame saw them (#157).
        let (pos, hp_r, phi_r, hdg_st, vs_st, ias_r, thr_r, servos_r) = tokio::join!(
            sim::position::read(&xp),
            xp.get_float(HPATH),
            xp.get_float(PHI),
            crate::autoflight::read_status(&xp, HDG_STATUS),
            crate::autoflight::read_status(&xp, VS_STATUS),
            xp.get_float(IAS),
            xp.get_float(&thr_dref),
            crate::autoflight::read_status(&xp, SERVOS_ON),
        );
        let (lat_r, lon_r) = sim::position::lat_lon(&pos);
        let elev_r = sim::position::field(&pos, |p| p.elev_m);
        let psi_r = sim::position::field(&pos, |p| p.psi_true_deg);
        // Read only where the capture is declared: nothing else uses them.
        let (dial_r, alt_hold_r) = if ap_capture {
            let (d, h) =
                tokio::join!(autoflight::read_dial(&xp, Dial::Altitude), crate::autoflight::read_status(&xp, ALT_HOLD_STATUS));
            (d.ok(), h.ok())
        } else {
            (None, None)
        };
        let (Ok(lat), Ok(lon)) = (lat_r, lon_r) else {
            continue;
        };
        let alt_msl_ft = elev_r.map(|m| m * M_TO_FT).unwrap_or(f64::NAN);
        let (hdg_read, vs_read) = (hdg_st.ok(), vs_st.ok());
        let (hdg_on, vs_on) = (hdg_read.unwrap_or(1.0) > 0.5, vs_read.unwrap_or(1.0) > 0.5);
        // Position in the departure-runway frame (along the takeoff course
        // from the threshold, right positive) — the authored waypoints' frame.
        let a_m = -along_track_nm(lat, lon, &rwy) * M_PER_NM;
        let x_m = cross_track_nm(lat, lon, &rwy) * M_PER_NM;
        let idx = match wpt_idx {
            Some(i) => i,
            None => {
                let i = join_waypoint(&route, a_m, x_m);
                crate::blackbox::record(
                    "circuit",
                    serde_json::json!({
                        "event": "authored_join", "wpt": i, "skipped": i, "a_m": a_m, "x_m": x_m,
                    }),
                );
                wpt_idx = Some(i);
                i
            }
        };
        let wp = route[idx];
        let (wlat, wlon) = frame_to_latlon(&rwy, wp.a_m, wp.x_m);
        // Crab so the AP holds the ground TRACK to the waypoint, like the legs.
        let psi_r = psi_r.ok();
        let crab = match (psi_r, hp_r) {
            (Some(psi), Ok(hp)) => {
                crate::angles::signed_diff_deg(psi, hp).clamp(-25.0, 25.0)
            }
            _ => 0.0,
        };
        let want_mag = true_to_mag(
            crate::nav::bearing_deg(lat, lon, wlat, wlon) + crab,
            mag_var,
        );
        let phi = phi_r.unwrap_or(0.0);
        bug_mag = match psi_r {
            Some(psi) => bug_toward(
                bug_mag,
                want_mag,
                true_to_mag(psi, mag_var),
                phi,
                BUG_SLEW_PER_TICK,
            ),
            None => slew_bug(bug_mag, want_mag, BUG_SLEW_PER_TICK),
        };
        // Set, not rewritten: per waypoint, a correction past the deadband
        // (`BugHold`, issue #93).
        hold_bug(
            &xp,
            &mut bug_hold,
            idx as u32,
            &mut bug_mag,
            t_start.elapsed().as_secs_f64(),
            CIRCUIT_LEG_DEADBAND,
        )
        .await;
        reengage_lateral(
            &xp,
            &mut servo_watch,
            &cfg.fd_commands,
            t_start.elapsed().as_secs_f64(),
            !hdg_on,
            servos_r.as_ref().ok().copied(),
            ias_r.as_ref().ok().copied().unwrap_or(f64::NAN),
            "authored",
        )
        .await;
        autoflight::bring_heading(&xp, hdg_read, "circuit_authored").await;

        // Airspeed trend first: both the climb limiter and the throttle law
        // judge the speed they are heading for.
        let ias = ias_r.as_ref().ok().copied().unwrap_or(f64::NAN);
        if ias.is_finite() {
            let t_now = std::time::Instant::now();
            if let Some((prev, t_prev)) = prev_ias {
                let dt = t_now.duration_since(t_prev).as_secs_f64();
                if (0.1..=2.0).contains(&dt) {
                    ias_trend = 0.6 * ias_trend + 0.4 * (ias - prev) / dt;
                }
            }
            prev_ias = Some((ias, t_now));
        }

        // Vertical: VS toward the waypoint altitude (the takeoff's proven
        // recipe), and only as much climb as the airspeed will pay for.
        let vs_raw = if alt_msl_ft.is_finite() {
            ((wp.alt_msl_ft - alt_msl_ft) * 3.0).clamp(-500.0, authored_climb_fpm())
        } else {
            0.0
        };
        let vs_cmd = climb_limit_fpm(vs_raw, ias, ias_trend, full_kts, floor_kts);
        // Each waypoint's altitude is a LEVEL target: with the capture
        // declared it goes on the dial, and a captured hold on it owns the
        // leg while the VS law stands off (`authored_vertical`).
        let vert = authored_vertical(ap_capture, wp.alt_msl_ft, alt_msl_ft, dial_r, alt_hold_r);
        if let Some(want) = vert.dial_ft {
            let ok = autoflight::set_dial(&xp, Dial::Altitude, want).await.is_ok();
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "alt_dial", "was": dial_r, "now": want,
                                   "wpt": idx, "ok": ok, "authored": true}),
            );
        }
        if vert.ap_level {
            // Whatever the VS dial says is stale the moment VS re-engages.
            last_vs_cmd = None;
            vs_refused = None;
        } else {
            if autoflight::bring_vs(&xp, vs_read, "circuit_authored")
                .await
                .pressed(Button::VerticalSpeed)
            {
                last_vs_cmd = None;
            }
            if last_vs_cmd.is_none_or(|v| (v - vs_cmd).abs() > 40.0) {
                let _ = autoflight::set_dial(&xp, Dial::Vs, vs_cmd).await;
                last_vs_cmd = Some(vs_cmd);
            }
            // The capture rests on VS taking the aircraft back OUT of the
            // hold for the next level: an autopilot that will not is said
            // once, with a base leg's height still to act on.
            if ap_capture && !vs_on {
                let since = *vs_refused.get_or_insert_with(std::time::Instant::now);
                if vs_stuck(since.elapsed().as_secs_f64()) && !vs_refused_said {
                    vs_refused_said = true;
                    crate::blackbox::record(
                        "circuit",
                        serde_json::json!({"event": "vs_refused", "wpt": idx, "cmd_vs": vs_cmd, "authored": true}),
                    );
                    alert(
                        &voice,
                        Priority::Warning,
                        "The autopilot won't take vertical speed. Check the vertical mode.",
                    );
                }
            } else {
                vs_refused = None;
            }
        }

        // Power holds the pattern speed — the same law the rectangle loop uses.
        if let (true, Ok(thr)) = (ias.is_finite(), thr_r) {
            let delta = crate::tools::speedhold::throttle_adjust_live(pattern_kts - ias, ias_trend);
            if delta != 0.0 {
                let _ = xp
                    .set_float(&throttle_dataref(), (thr + delta).clamp(0.0, 1.0))
                    .await;
            }
        }

        // Landing and taxi lights off through 1,000 ft, as the rectangle does.
        if !lights_done && alt_msl_ft.is_finite() && alt_msl_ft - rwy.elev_ft > LIGHTS_OFF_AGL_FT {
            lights_done = true;
            let did = crate::tools::lights::after_climb(&xp).await;
            crate::blackbox::record(
                "circuit",
                serde_json::json!({"event": "lights_climb", "off": did}),
            );
        }

        let d_m = crate::nav::haversine_nm(lat, lon, wlat, wlon) * M_PER_NM;
        crate::blackbox::record(
            "circuit",
            serde_json::json!({
                "event": "authored_leg", "wpt": idx, "d_m": d_m, "bug": bug_mag,
                "alt_msl": alt_msl_ft, "tgt_alt": wp.alt_msl_ft, "cmd_vs": vs_cmd,
                "vs_raw": vs_raw, "ias": ias, "phi": phi, "ap_level": vert.ap_level,
            }),
        );

        if d_m < AUTHORED_CAPTURE_M {
            wpt_idx = Some(idx + 1);
            closure = None;
            if idx + 1 >= route.len() {
                break;
            }
            continue;
        }
        // Not closing on the waypoint for a whole [`STALL_S`]: orbiting, or
        // blown off the leg. Level off (bug on the nose) and let the next
        // ticks re-steer; three of those and the aircraft goes back.
        let now_s = t_start.elapsed().as_secs_f64();
        let stalled = closure
            .get_or_insert_with(|| Closure::new(d_m, now_s))
            .stalled(d_m, now_s);
        if stalled {
            stalls += 1;
            crate::blackbox::record(
                "circuit",
                serde_json::json!({
                    "event": "authored_stalled", "wpt": idx, "d_m": d_m, "stalls": stalls,
                }),
            );
            if stalls >= MAX_STALLS {
                let _ = autoflight::set_dial(&xp, Dial::Vs, 0.0).await;
                stand_down(
                    &xp,
                    &voice,
                    "I can't make the circuit's next point. Holding heading and altitude. Your aircraft.",
                    "authored_stalled_out",
                );
                return;
            }
            alert(
                &voice,
                Priority::Caution,
                "Not closing on the next point. Levelling off and re-joining.",
            );
            if let Some(psi) = psi_r {
                bug_mag = true_to_mag(psi, mag_var);
                bug_hold.reset();
                write_bug(&xp, bug_mag).await;
                bug_hold.decide(idx as u32, bug_mag, t_start.elapsed().as_secs_f64(), CIRCUIT_LEG_DEADBAND);
            }
        }
    }
    // The last waypoint sits on short final — hand the landing to autoland.
    alert(
        &voice,
        Priority::Advisory,
        crate::readback::one_of(&[
            "Turning final. Landing us.",
            "On final now. I'll put her down.",
            "Turning final. Bringing us in.",
        ]),
    );
    transmit(radio_call(
        &callsign,
        &format!("final runway {}", spell_runway(&rwy_id)),
    ));
    crate::blackbox::record(
        "circuit",
        serde_json::json!({"event": "handoff_autoland_authored", "rwy": rwy_id}),
    );
    // FORCE the pattern's own runway. Bare `None` sends autoland to its
    // EMERGENCY behaviour — search the whole apt.dat for the best end by track
    // — which is right for an engine failure and catastrophically wrong here:
    // we are on short final to a runway we just took off from and flew a
    // pattern around, and the search does not know that.
    // The circuit's own token is the autoland's parent: a stand-down landing
    // around the launch cancels both (`handed_off` keeps the circuit's
    // normal exit from doing it).
    crate::autoland::run_autoland_scoped(
        base_url,
        apt_dat,
        cfg,
        approach_ias,
        voice,
        None,
        // No ICAO needed: this is the runway we just took off from and have
        // been circling for minutes, a mile away. Proximity IS the right
        // answer here — unlike an arrival, which knows its destination and
        // must say so (see `PlannedRunway`).
        Some(crate::runway::PlannedRunway::anywhere(&rwy_id)),
        xp.cancellation(),
        // A circuit final is judged at the visual gate, 500 ft (#199).
        crate::autoland::HandDown::circuit(None),
    );
    handed_off(&xp);
}
