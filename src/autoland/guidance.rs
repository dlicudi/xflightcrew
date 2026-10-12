//! The guidance core: the knob, phases, tick, pursuit, the flare and roundout
//! holds, the pavement need, the landing call and the commit.

use super::*;

/// Autoland's intercepts, patterns and missed approach: a two-degree
/// correction at most every five seconds.
pub const AUTOLAND_INTERCEPT_DEADBAND: crate::heading_bug::BugDeadband =
    crate::heading_bug::BugDeadband { deg: 2.0, every_s: 5.0 };
/// The final outside [`SHORT_FINAL_NM`]: half a degree at most every two
/// seconds (a degree every three left the fitted 737 9.8 m off at the
/// roundout gate from a 30 m LEMG 13 handover, against 3.9 every tick).
pub const AUTOLAND_FINAL_DEADBAND: crate::heading_bug::BugDeadband =
    crate::heading_bug::BugDeadband { deg: 0.5, every_s: 2.0 };

/// How autoland's heading knob is set this tick (issue #93): the key and
/// deadband for [`crate::heading_bug::BugHold`], or `None` to steer it every
/// tick as before (the short final and the flare, where the last 0.3 nm
/// closes the centreline in seconds). A pattern's legs and each missed
/// approach stage are their own keys, so a new leg is set at once. Pure.
pub fn bug_knob(
    phase: ApproachPhase,
    short_final: bool,
    pattern_leg: Option<crate::reversal::Leg>,
    missed: Option<MissedStage>,
) -> Option<(u32, crate::heading_bug::BugDeadband)> {
    match (phase, short_final, pattern_leg, missed) {
        (ApproachPhase::Flare, ..) | (_, true, ..) => None,
        (_, _, Some(leg), _) => Some((10 + leg as u32, AUTOLAND_INTERCEPT_DEADBAND)),
        (_, _, None, Some(stage)) => Some((20 + stage as u32, AUTOLAND_INTERCEPT_DEADBAND)),
        (ApproachPhase::Final, ..) => Some((2, AUTOLAND_FINAL_DEADBAND)),
        (ApproachPhase::Intercept, ..) => Some((1, AUTOLAND_INTERCEPT_DEADBAND)),
    }
}

/// Where the autoland is in the approach.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApproachPhase {
    /// Flying to intercept the final approach course.
    Intercept,
    /// Established on final, tracking the centreline and glidepath.
    Final,
    /// Very short — over the threshold, flaring.
    Flare,
}

/// How long to sleep between ticks, milliseconds — by what the loop is
/// actually flying.
///
/// 400 ms everywhere made the tick 0.6 s end to end (measured on the EGPH
/// landing, 2026-09-20: `dt` median 0.59 s, so 1.7 Hz), and a flare is over
/// in three seconds. The 737's own `ArrestLeadProfile` declares 1.5 s of
/// dead time, so an arrest commanded six times a flare gets about TWO
/// commands the aircraft can still act on — it touched down at 977 fpm
/// against a 175 fpm target, and the yoke visibly stepped between updates.
/// No gain fixes that; there is not enough bandwidth to fly a flare.
///
/// The Web API is served from X-Plane's flight loop, so this is sim frame
/// time and it is spent where it buys something: fast where feet matter,
/// unchanged out on the intercept where nothing moves in half a second.
///
/// Since 2026-09-30 this is the tick's PERIOD, paced from its start
/// (`sim::state::Pacer`), not a sleep after its work: the old sleeps (100,
/// 200, 400) plus the requests' rounds made the MacBook's 0.20 s Flare and
/// 0.33 s Final (bench landings, `power` dt) and the iMac's 0.39 and 0.56.
/// These are the MacBook's, the ticks the laws were tuned on, now on any
/// machine; the Intercept's 540 is its old 400 sleep plus seven 20 ms rounds
/// (no MacBook record).
pub(super) fn tick_ms(phase: Option<ApproachPhase>, arresting: bool) -> u64 {
    // The roundout is hand-flown on the channels too (it is where the
    // servos come off), and it needs the flare's rate, not Final's.
    if arresting {
        return 200;
    }
    match phase {
        // Over the threshold: the flare and the arrest.
        Some(ApproachPhase::Flare) => 200,
        // Established, configuring, closing the last miles.
        Some(ApproachPhase::Final) => 330,
        _ => 540,
    }
}

/// The rollout's tick period, ms: its old 300 ms sleep plus the MacBook's
/// rounds (bench rollouts before 2026-09-29, `dt` median 0.378 s).
pub(super) const ROLLOUT_TICK_MS: u64 = 380;
/// The approach's cross-track rate filter, seconds: half the raw rate a
/// tick at the MacBook's 0.33 s Final tick is a time constant of
/// 0.33 / ln 2.
pub(super) const XTK_RATE_TAU_S: f64 = 0.476;

/// The command the control loop should fly this tick.
#[derive(Debug, Clone, Copy)]
pub struct Guidance {
    pub target_heading_deg: f64,
    pub target_vs_fpm: f64,
    pub target_ias_kts: f64,
    pub dist_nm: f64,
    /// Where the aircraft *should* be for the glidepath, ft MSL.
    pub glidepath_alt_ft: f64,
    pub phase: ApproachPhase,
    /// Why this tick balked the landing, when it did: the gate that flipped
    /// the phase to `Intercept` low enough for the latch to fly it as a missed
    /// approach. `None` on every ordinary tick. Blackboxed with the
    /// `going_around` event, because "climbing straight ahead" with no reason
    /// attached cost a person an hour of reconstruction from the command
    /// column (LEMG 13, 2026-09-12).
    pub balk: Option<&'static str>,
    /// The vertical target is a landing ARREST (roundout or flare): the VS
    /// dial must lead it through the autopilot's lag ([`vs_dial_fpm`]).
    /// False everywhere else, where the dial is the target.
    pub arresting: bool,
}

/// Where the final's pure pursuit aims, in nm from the threshold measured back
/// up the approach. Negative means a point PAST the threshold, on the runway.
///
/// The lead is fixed — about thirty seconds of travel — because that is what
/// makes the convergence exponential in TIME rather than linear in distance:
/// hard correction early, established miles out, then a tracked wings-level
/// final, which is what "stabilised approach" means. An aim at half the
/// remaining distance decays the cut only linearly, and the aircraft rides the
/// whole final at an angle (EGSS 22, 2026-07-28: 28 degrees at 5.4 nm, still
/// 20 at 3, aligned only at 0.13 nm, i.e. over the numbers).
///
/// The trap this closes is that the fix for that was itself undone on short
/// final. Flooring the aim AT the threshold meant that inside one lead
/// distance the aim stopped advancing and the EFFECTIVE lead shrank — 0.875 nm
/// at 1.2 nm out, 0.55 at 0.7, 0.20 at 0.35 — which is the d/2 law again,
/// reintroduced exactly where being lined up matters most. At Southampton the
/// cut sat at 16 degrees from 0.87 nm all the way in to 0.32, then snapped to
/// the runway course over the numbers with the aircraft still 6 degrees off
/// and still turning (2026-08-01).
///
/// So the aim keeps its full lead all the way in and simply walks past the
/// threshold. That is not a trick: the extended centreline and the runway
/// centreline are one line. Bounded by the pavement so the aim is always a
/// point on the runway.
pub fn pursuit_aim_nm(along_nm: f64, gs_kts: f64, rwy_len_m: f64) -> f64 {
    let lead_nm = (gs_kts / 120.0).clamp(0.6, 1.5);
    let past_nm = (rwy_len_m / M_PER_NM * 0.8).clamp(0.2, 1.5);
    (along_nm - lead_nm).max(-past_nm)
}

/// ONCE FLARING, STAY FLARING. `guidance` decides the phase from this tick's
/// radio height alone, so a flare that ballooned back above its gate became
/// "final" again, and final's law put the nose down: forward elevator and a
/// -1,200 fpm dial from 58 ft, then the flare again at 24 ft with the column
/// already at its stop, and the wheels arrived at -1,031 fpm (the Citation,
/// EDDV 27L, 2026-09-22). A pilot holds a balloon and lets it settle, or
/// goes around. So: a tick that was flaring and now reads Final stays in the
/// flare up to the top of the roundout band; above the band the landing is
/// balked (`balloon`) and the go-around latch flies it. Anything else
/// guidance decided (a balk, an overflight) stands.
pub fn hold_flare(
    g: Guidance,
    was_flaring: bool,
    radio_ft: f64,
    gs_kts: f64,
    flare_radio_ft: f64,
) -> Guidance {
    if !was_flaring || g.phase != ApproachPhase::Final {
        return g;
    }
    let (_, band_ft) = arrest_gates_ft(flare_radio_ft, gs_kts);
    if radio_ft > band_ft {
        return Guidance {
            phase: ApproachPhase::Intercept,
            balk: Some("balloon"),
            arresting: false,
            ..g
        };
    }
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (_, flare_vs) = arrest_targets_fpm(nominal_vs);
    Guidance {
        phase: ApproachPhase::Flare,
        target_vs_fpm: flare_vs,
        arresting: true,
        ..g
    }
}

/// A balloon out of the flare is landed, not gone around from, below this
/// height (with room to stop). EDLP 24 (2026-09-24): the SF50's flare held
/// 0.3 of elevator until it was already climbing, ballooned to 52 ft at +850
/// fpm, balked `balloon` above the 50 ft band, and the go-around from idle at
/// 81 kt and slowing was given 0 fpm and stalled onto the runway at -1,568
/// fpm. A pilot holds a balloon, adds a little power and lets it settle.
pub const BALLOON_COMMIT_FT: f64 = 100.0;

/// The height a recited landing distance is measured from: the book figure
/// (POM, AFM) runs from 50 ft over the threshold, on a stabilised 3 degree
/// approach, to a stop.
pub(super) const LANDING_SCREEN_FT: f64 = 50.0;
/// The part of that figure a 3 degree path covers from the screen height to
/// the ground: 50 ft x 0.3048 / tan 3 degrees, about 291 m. The book's
/// definition, so the 3 degree constant, not this approach's path.
pub(super) const SCREEN_AIR_M: f64 = LANDING_SCREEN_FT * M_PER_FT / GLIDEPATH_TAN;

/// The pavement a flare at `radio_ft` still needs to land and stop, metres.
///
/// A recited landing distance starts at 50 ft over the threshold; an
/// aircraft flaring at 8 ft has already flown the part of it between the
/// two, and asking the pavement for the whole figure counts that part twice.
/// LFBD 11, 2026-09-27 (the stock 737, POM 5,360 ft = 1,634 m, 2,418 m of
/// runway): the flare floated to 7.9 ft radio at 145 kt over the ground,
/// 796 m past the threshold with 1,622 m ahead and sinking 184 fpm, and the
/// overflight balk stood because 1,622 < 1,634, by 12 m. "Going around",
/// full power for a second, the wheels touched 1.3 s later and it stopped
/// with about 700 m to spare. The air flown is taken on the book's 3 degree
/// path from 50 ft down to this height, in proportion (a flare at 7.9 ft is
/// credited 245 m of the 291, a balloon at or above 50 ft nothing): the
/// book's own air segment is longer than that geometry, since it includes
/// the flare's float, so what is left asked for is the ground roll plus a
/// float, never less.
///
/// Floored at half the declared figure: a light single's ground roll is a
/// small part of its over-50-ft figure (the 172's book: 1,335 ft total,
/// 575 ft of roll, 43%), and the geometric 291 m would leave it asking for
/// 116 m to stop in 175. Half covers it; a jet's need never reaches the
/// floor (the 737's 1,634 less 291 is 1,343, against 817).
///
/// With NO declared figure the need stays [`FLOAT_MIN_REMAINING_M`] whole:
/// that 900 m is the Baron's measured ~550 m ROLLOUT plus a margin, a
/// ground figure already, with no air segment in it to credit. Pure.
pub fn flare_pavement_need_m(radio_ft: f64, landing_m: Option<f64>) -> f64 {
    match landing_m.filter(|v| v.is_finite() && *v > 0.0) {
        Some(book_m) => {
            let flown_ft = (LANDING_SCREEN_FT - radio_ft.max(0.0)).clamp(0.0, LANDING_SCREEN_FT);
            let air_flown_m = SCREEN_AIR_M * flown_ft / LANDING_SCREEN_FT;
            (book_m - air_flown_m).max(0.5 * book_m)
        }
        None => FLOAT_MIN_REMAINING_M,
    }
}

/// The pavement a landing ON THE WHEELS needs, metres: the airframe's
/// declared landing ROLL (`landing_roll_ft`) when it has one, else the
/// committed flare's book need at the wheels ([`flare_pavement_need_m`] at
/// 0 ft). The book figure is a factored field length from 50 ft on the
/// jets: the 737's 1,634 m less the 291 m of air is 1,343 m to stop from
/// the wheels, and LXGB 09 (2026-10-06) went around at 4.5 ft with 1,109 m
/// ahead, touched 1.6 s later on the spool-up and stopped in about 540.
/// Pure.
pub fn wheels_need_m(landing_m: Option<f64>, roll_m: Option<f64>) -> f64 {
    match roll_m.filter(|r| r.is_finite() && *r > 0.0) {
        Some(roll) => roll,
        None => flare_pavement_need_m(0.0, landing_m),
    }
}

/// A flare bleeds this much airspeed a second at idle, kt/s, for the float
/// an approach's excess speed adds to the prediction: each knot over the
/// approach speed is half a second more in the air (a jet's rule of thumb,
/// a thousand feet of float for ten knots at 140).
pub(super) const FLARE_BLEED_KTS_S: f64 = 2.0;

/// Seconds from `radio_ft` to the wheels on the flare the law FLIES
/// ([`super::attitude_flare`]): the path's sink (at this groundspeed) down
/// to its flare height, then its round-out and hold-off
/// (`attitude_flare::vs_target_fpm`), summed over the height. `None` for a
/// measurement that is not one. Pure.
pub fn flare_time_s(radio_ft: f64, gs_kts: f64) -> Option<f64> {
    use super::attitude_flare as af;
    if !radio_ft.is_finite() || !gs_kts.is_finite() || gs_kts <= 0.0 {
        return None;
    }
    let h = radio_ft.max(0.0);
    let path_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let gate = af::gate_ft(path_vs, af::RESPONSE_S);
    let above = (h - gate).max(0.0) / -path_vs * 60.0;
    // The round-out, in tenth-of-a-foot steps at each step's middle.
    const STEP_FT: f64 = 0.1;
    let below = h.min(gate);
    let n = (below / STEP_FT).ceil() as usize;
    let step = if n > 0 { below / n as f64 } else { 0.0 };
    let flare: f64 = (0..n)
        .map(|i| step / -af::vs_target_fpm((i as f64 + 0.5) * step, path_vs, gate) * 60.0)
        .sum();
    Some(above + flare)
}

/// The go/no-go at the roundout gate (see [`landing_call`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingCall {
    /// Land: the touchdown the energy predicts leaves the room to stop.
    pub land: bool,
    /// The predicted touchdown, metres past the runway's threshold (its
    /// apt.dat end; negative short of it).
    pub touchdown_m: f64,
    /// The pavement predicted ahead of that touchdown, and what stopping
    /// needs ([`wheels_need_m`]), metres.
    pub remaining_m: f64,
    pub need_m: f64,
    /// Seconds of flare and float the prediction flew.
    pub air_s: f64,
}

/// LAND IN THE ROOM THE ENERGY LEAVES, OR GO AROUND WITH HEIGHT IN HAND.
/// Decided ONCE, at the roundout gate (the arrest takes the servos), from
/// where the touchdown will be: the flare the law flies from here
/// ([`flare_time_s`]) plus the float the speed over the approach speed adds
/// ([`FLARE_BLEED_KTS_S`]), at this groundspeed, from this position. The
/// touchdown must leave the room to stop ([`wheels_need_m`]); else the
/// go-around is flown NOW, on the approach's thrust, 80 ft up, never at
/// 4 ft from an idle flare. LXGB 09 (737, wet, 2026-10-06, sortie
/// 1791275171): no decision before the flare; the float allowance's 0.35
/// nm ran out at 4.5 ft radio and the overflight balk stood (1,109 m ahead
/// against the book's 1,369), "Going around. Full power, climbing straight
/// ahead." toward the Rock, the wheels touched 1.6 s later on the spool-up
/// and it stopped. From that gate (79.8 ft, 137 m short of the strip at
/// 140 kt over the ground and 146 indicated) the call is LAND with about
/// 1,085 m predicted ahead against 792. `along_m` is the signed along-track
/// to the threshold (positive on the approach side), `length_m` the strip.
/// `None` when a measurement is missing (no guessed decision). Pure.
pub fn landing_call(
    along_m: f64,
    length_m: f64,
    radio_ft: f64,
    gs_kts: f64,
    ias_kts: Option<f64>,
    approach_ias: f64,
    need_m: f64,
) -> Option<LandingCall> {
    if !along_m.is_finite() || !length_m.is_finite() || !need_m.is_finite() {
        return None;
    }
    let flare_s = flare_time_s(radio_ft, gs_kts)?;
    let excess_kts = match ias_kts {
        Some(v) if v.is_finite() && approach_ias.is_finite() => (v - approach_ias).max(0.0),
        _ => 0.0,
    };
    let air_s = flare_s + excess_kts / FLARE_BLEED_KTS_S;
    let touchdown_m = gs_kts * KTS_TO_MPS * air_s - along_m;
    let remaining_m = length_m - touchdown_m.max(0.0);
    Some(LandingCall {
        land: remaining_m >= need_m,
        touchdown_m,
        remaining_m,
        need_m,
        air_s,
    })
}

/// Knots to metres per second.
pub(super) const KTS_TO_MPS: f64 = 0.514_444;

/// COMMITTED, ONCE FLARING LOW. A flare below the roundout band is at idle,
/// slow and nose up, and the go-around the overflight balk hands it cannot
/// climb: the vertical law climbs only with speed in hand, a flare has none,
/// and a jet's engines are a few seconds from thrust. ETND 26 (2026-09-23):
/// the SF50 ballooned to 32 ft, the float allowance (1,244 m of runway less the
/// Baron's 900 m floor) ran out at 344 m, it went around with 900 m of runway
/// ahead, the law gave it 0 fpm at 80 kt, and it met the runway at about -740
/// fpm. So below the band, an `overflew_the_runway` verdict does not stand
/// while the pavement left still covers the airframe's OWN landing distance
/// (the 900 m floor is for the float's judgement above, not for a landing
/// already in its last seconds). A BALLOON is committed the same way up to
/// [`BALLOON_COMMIT_FT`] (EDLP 24, the crash). No room to stop is still a
/// go-around; every other balk stands. The room is
/// [`flare_pavement_need_m`]: the recited distance less the air segment
/// already flown (LFBD 11). Pure.
#[allow(clippy::too_many_arguments)]
#[cfg(test)]
pub fn commit_flare(
    g: Guidance,
    was_flaring: bool,
    radio_ft: f64,
    gs_kts: f64,
    flare_radio_ft: f64,
    remaining_m: f64,
    landing_m: Option<f64>,
    course_deg: f64,
) -> Guidance {
    commit_flare_latched(
        g,
        was_flaring,
        None,
        radio_ft,
        gs_kts,
        flare_radio_ft,
        remaining_m,
        landing_m,
        course_deg,
    )
}

/// ONCE COMMITTED, THE FLARE STAYS COMMITTED. [`commit_flare`] judged the
/// room afresh every tick against [`flare_pavement_need_m`] at THIS height,
/// which high in the band is the overflight allowance itself less the few
/// metres of air credited from 50 ft: the A330 at LEMG 30 (sortie
/// 1790634412-approach, 2026-09-28) was committed at 46.4 ft with 1,432 m
/// ahead against a need of 1,427, floating level, and 0.3 s and 23 m later
/// the same test failed, the arrest handed the servos back and it went
/// around `overflew_the_runway` with 1,409 m of runway left. A commitment
/// that lasts one tick is no commitment. So once committed
/// (`committed_need_m`, the loop's latch for this flare or the roundout
/// gate's [`landing_call`]), the landing continues while the pavement ahead
/// still covers the room to stop from the wheels ([`wheels_need_m`]: the
/// declared landing roll, else the book's ground roll with the whole air
/// segment credited); past that it is still a go-around. The ceiling
/// heights stand. Pure.
#[allow(clippy::too_many_arguments)]
pub fn commit_flare_latched(
    g: Guidance,
    was_flaring: bool,
    committed_need_m: Option<f64>,
    radio_ft: f64,
    gs_kts: f64,
    flare_radio_ft: f64,
    remaining_m: f64,
    landing_m: Option<f64>,
    course_deg: f64,
) -> Guidance {
    let (_, band_ft) = arrest_gates_ft(flare_radio_ft, gs_kts);
    let ceiling_ft = match g.balk {
        Some("overflew_the_runway") if was_flaring => band_ft,
        Some("balloon") if was_flaring => BALLOON_COMMIT_FT,
        _ => return g,
    };
    let need_m = committed_need_m
        .filter(|n| n.is_finite())
        .unwrap_or_else(|| flare_pavement_need_m(radio_ft, landing_m));
    if radio_ft > ceiling_ft || remaining_m < need_m {
        return g;
    }
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (roundout_vs, flare_vs) = arrest_targets_fpm(nominal_vs);
    Guidance {
        // The overflight turned the heading back to the FAP; a landing
        // keeps the runway's.
        target_heading_deg: course_deg,
        phase: ApproachPhase::Flare,
        // Above the band (a balloon's top) come back down at the roundout's
        // sink: the flare's -100 from 60 ft is half a minute of runway.
        target_vs_fpm: if radio_ft > band_ft {
            roundout_vs
        } else {
            flare_vs
        },
        balk: None,
        arresting: true,
        ..g
    }
}

/// ONCE THE ARREST HAS THE SERVOS, IT KEEPS THEM. `guidance` calls the
/// roundout from this tick's radio height alone (`radio_alt_ft <= band_ft`),
/// so a balloon in the roundout that rose back over the band top turned the
/// arrest off, and [`arrest_hands_step`] gave the servos back to an
/// autopilot whose VS dial was still the approach's descent: LEMG 12, the
/// SF50 (sortie 1790611629, 2026-09-28), taken at 51 ft, ballooned to +263
/// fpm, handed back at 50.5 ft against a 50 ft band; the autopilot flew its
/// -445 fpm dial and the governor, no longer held to "only back", walked the
/// lever 0.16 to 0.27 (on a tail-mounted engine, nose down) for 1.2 s; taken
/// again at 48.4 ft with the sink already rebuilding, 45 to 3 ft in 1.5 s,
/// -814 fpm at the wheels. So a tick called Final and not arresting, with no
/// balk, after the arrest has taken the servos is still the arrest: flown
/// down at the roundout's sink up to [`BALLOON_COMMIT_FT`] while the pavement
/// ahead still covers the landing ([`flare_pavement_need_m`], as
/// [`commit_flare`]); higher, or with no room left, it is a balked landing
/// (`balloon`) and the go-around latch flies it. A real balk (off the
/// centreline, overflown, ordered, a pattern) is never overruled: only a
/// go-around brings the autopilot back. The flare's own balloon is
/// [`hold_flare`]'s. Pure.
pub fn hold_roundout(
    g: Guidance,
    arrest_taken: bool,
    radio_ft: f64,
    gs_kts: f64,
    remaining_m: f64,
    landing_m: Option<f64>,
) -> Guidance {
    if !arrest_taken || g.arresting || g.phase != ApproachPhase::Final || g.balk.is_some() {
        return g;
    }
    if radio_ft > BALLOON_COMMIT_FT || remaining_m < flare_pavement_need_m(radio_ft, landing_m) {
        return Guidance {
            phase: ApproachPhase::Intercept,
            balk: Some("balloon"),
            arresting: false,
            ..g
        };
    }
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (roundout_vs, _) = arrest_targets_fpm(nominal_vs);
    Guidance {
        target_vs_fpm: roundout_vs,
        arresting: true,
        ..g
    }
}

/// Compute the approach guidance: steer to the final approach point then track the
/// centreline, and descend the 3° glidepath. Pure — the control loop applies it via
/// the autopilot. `flare_radio_ft` is the height above the runway to start the flare.
/// `xtk_rate_nm_min` is the cross-track closure rate (positive = drifting right),
/// 0.0 when unknown — the damping term on the final's centreline correction.
#[cfg(test)]
#[allow(clippy::too_many_arguments)]
pub fn guidance(
    cur_lat: f64,
    cur_lon: f64,
    cur_alt_ft: f64,
    radio_alt_ft: f64,
    gs_kts: f64,
    ias_kts: f64,
    rwy: &Runway,
    approach_ias: f64,
    flare_radio_ft: f64,
    xtk_rate_nm_min: f64,
) -> Guidance {
    guidance_on(
        cur_lat,
        cur_lon,
        cur_alt_ft,
        radio_alt_ft,
        gs_kts,
        ias_kts,
        rwy,
        approach_ias,
        flare_radio_ft,
        xtk_rate_nm_min,
        None,
    )
}

/// [`guidance`] on a runway of known pavement width (`width_m`, apt.dat's):
/// the lateral gates below the terrain floor judge the final against the
/// pavement ([`landing_xtk_limit_m`], [`flare_xtk_limit_m`]). `None` is the
/// old 37 m everywhere.
#[allow(clippy::too_many_arguments)]
pub fn guidance_on(
    cur_lat: f64,
    cur_lon: f64,
    cur_alt_ft: f64,
    radio_alt_ft: f64,
    gs_kts: f64,
    ias_kts: f64,
    rwy: &Runway,
    approach_ias: f64,
    flare_radio_ft: f64,
    xtk_rate_nm_min: f64,
    width_m: Option<f64>,
) -> Guidance {
    let landing_xtk_nm = landing_xtk_limit_m(width_m) / M_PER_NM;
    let flare_xtk_nm = flare_xtk_limit_m(width_m) / M_PER_NM;
    let dist_nm = haversine_nm(cur_lat, cur_lon, rwy.thr_lat, rwy.thr_lon);
    let recip = (rwy.course_deg + 180.0) % 360.0;

    // 3° glidepath altitude at this distance (+ = we're above it). Aimed at the
    // touchdown zone, so it crosses the threshold at ~50 ft, not zero.
    let glidepath_alt_ft = rwy.elev_ft + (dist_nm + TOUCHDOWN_AIM_NM) * gp_ft_per_nm();
    let alt_err = cur_alt_ft - glidepath_alt_ft;

    // Above what the way in can lose, we can't just dive onto the path: fly to
    // the JOIN point instead: where a 3° descent (with 15% margin) meets our
    // CURRENT altitude, out along the extended centreline. When we're inside it
    // (armed high overhead, the failure seen live: 7,000 ft at 6 nm, VS pegged,
    // never capturing) that means flying OUTBOUND first; the point walks back
    // toward the FAP as altitude burns off (a teardrop join). See
    // [`aims_at_join_point`] for the height a straight-in keeps.
    let join_nm = join_point_nm(cur_alt_ft, rwy.elev_ft);

    // Are we past the threshold, on the departure side, still airborne — by
    // more than the landing float is allowed? (The flare branch below, checked
    // first, owns a normal float/rollout across the numbers.)
    let along_nm = along_track_nm(cur_lat, cur_lon, rwy);
    // The arrest's gates and targets for THIS groundspeed (seconds of sink,
    // never bare feet: see `arrest_gates_ft`). `flare_radio_ft` is the floor.
    let nominal_vs = -(gs_kts / 60.0) * gp_ft_per_nm();
    let (flare_ft, band_ft) = arrest_gates_ft(flare_radio_ft, gs_kts);
    let (roundout_vs, flare_vs) = arrest_targets_fpm(nominal_vs);
    let float_nm = overflight_allowance_nm(
        rwy.length_m,
        radio_alt_ft,
        band_ft,
        crate::aircraft::landing_distance_m(),
    );
    let past_threshold = along_nm < -float_nm;
    // The flare/roundout environment: short final by plain distance, or
    // anywhere over the pavement the float allowance still covers — `dist_nm`
    // is unsigned, so past the threshold it alone would end the flare zone at
    // 0.35 nm while the float is still legitimately landing.
    let in_flare_zone = dist_nm <= FLARE_MAX_DIST_NM || (along_nm < 0.0 && !past_threshold);

    let xtk = cross_track_nm(cur_lat, cur_lon, rwy);
    let mut balk: Option<&'static str> = None;
    let (phase, target_heading) = if radio_alt_ft <= flare_ft && in_flare_zone {
        // Low radio altitude alone is NOT the flare cue — terrain rises to meet
        // the approach path miles from the field (seen live: "flared" at 7 ft AGL
        // 4.1 nm out at LEMI, power cut over open ground; and again into a dip
        // 0.4 nm short at LERL). Flare only over the runway environment.
        if xtk.abs() > flare_xtk_nm {
            // ...and only over the PAVEMENT. The final's P-only tracking left a
            // standing 0.10 nm cross-track under the LXGB crosswind and the
            // flare froze it: touchdown 185 m right of the centreline, in the
            // grass, wrecked on the rollout (2026-07-22). Off-centreline at
            // flare height is a balked landing — the phase flip drops the
            // GoAroundLatch, which climbs straight ahead.
            balk = Some("off_centreline_at_flare");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else {
            // Keep steering through the flare — locking the bare runway course
            // froze whatever offset the final still carried. Gentler clamp:
            // no big heading swings a few feet off the ground. Rate-damped
            // like the final, so a closing drift isn't chased into an S.
            let corr = ((xtk + 0.2 * xtk_rate_nm_min) * 80.0).clamp(-8.0, 8.0);
            (
                ApproachPhase::Flare,
                (rwy.course_deg - corr + 360.0) % 360.0,
            )
        }
    } else if past_threshold {
        // Overflew the runway without landing (too high/fast at the numbers). Go
        // around: steer BACK to the Final Approach Point, never track the runway
        // course outbound — that departs the field forever (seen live 2026-07-06:
        // over the numbers at 167 ft, then straight out to 5.4 nm at 60 kt, never
        // turning back). The vertical law climbs to a safe altitude on the way, if
        // energy allows. Takes priority over the join/FAP/final branches so a
        // go-around always turns toward the field regardless of altitude.
        let (fap_lat, fap_lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, FAP_NM);
        balk = Some("overflew_the_runway");
        (
            ApproachPhase::Intercept,
            bearing_deg(cur_lat, cur_lon, fap_lat, fap_lon),
        )
    } else if aims_at_join_point(alt_err, along_nm, xtk, gs_kts) {
        // Too high for this distance — descend toward the join point.
        let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
        (
            ApproachPhase::Intercept,
            bearing_deg(cur_lat, cur_lon, jlat, jlon),
        )
    } else if dist_nm > FAP_NM + 0.5 {
        // Outside the FAP: steer TO the Final Approach Point — a point FAP_NM out
        // on the approach side of the threshold along the extended centreline —
        // not the bare runway course. Tracking the raw course assumes we're
        // already on the approach side flying inbound; when we're off to the side
        // or PAST the threshold it flies us away from the field, the distance
        // growing every tick (seen live: XLE000N, 12.6 -> 13.8 nm on a fixed
        // heading, never intercepting). Aiming at the FAP always closes on the
        // runway from the correct side, then the final branch tracks the centreline.
        let (fap_lat, fap_lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, FAP_NM);
        (
            ApproachPhase::Intercept,
            bearing_deg(cur_lat, cur_lon, fap_lat, fap_lon),
        )
    } else {
        // On final: PURSUE a point ON the centreline halfway to the threshold
        // instead of P-correcting the offset. Offset-proportional gains kept
        // failing by geometry: 80°/nm commanded all of 7° of cut at 0.09 nm
        // off — mathematically unable to centre in the distance remaining,
        // and the aircraft crossed 150 ft still 145 m left (2026-07-24; same
        // family as the 120 m/2026-07-14 and 55 m/2026-07-22 misses at lower
        // gains). Aiming at the centreline point makes the cut DISTANCE-
        // aware: atan(offset / distance-to-aim) — gentle far out, up to a
        // 35° intercept when close and offset, rolling out on the line as
        // the pursuit point recedes.
        // Inside ~550 m the halfway aim point would fall BEHIND the aircraft
        // (the 0.25 nm floor exceeded the distance) and the bearing flipped
        // the cut ±35° at 100 ft over the numbers (2026-07-24). Short final
        // uses small course-referenced corrections only — offsets there are
        // either small or the balk gate's problem.
        // FIXED-lead pursuit, not half-the-remaining-distance. With the aim at
        // d/2 the cut angle decays only LINEARLY with distance (offset falls
        // as d², cut ~ offset/d ~ d), so the aircraft rode the WHOLE final at
        // an angle and only rolled out at the flare — measured at EGSS 22:
        // final began 5.4 nm out with a 28° cut, still 20° at 3 nm, 9° at
        // 1.9, 5° inside a mile, aligned at 0.13 nm, i.e. over the numbers
        // ("never in line with the runway, always at an angle", 2026-07-28).
        // A fixed lead makes the convergence exponential in TIME instead:
        // hard correction early, established miles out, then a tracked,
        // wings-level final — which is what "stabilised approach" means.
        // ~30 s of travel, floored so a slow single still aims somewhere and
        // capped so the far-out cut stays flyable.
        let cut = if dist_nm > 0.30 {
            let along = along_track_nm(cur_lat, cur_lon, rwy);
            let aim_nm = pursuit_aim_nm(along, gs_kts, rwy.length_m);
            let (alat, alon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, aim_nm);
            let brg = bearing_deg(cur_lat, cur_lon, alat, alon);
            crate::angles::signed_diff_deg(brg, rwy.course_deg).clamp(-35.0, 35.0)
        } else {
            (-xtk * 80.0).clamp(-8.0, 8.0)
        };
        if radio_alt_ft < 100.0 && dist_nm < 0.6 && xtk.abs() > landing_xtk_nm {
            // Not going to make the pavement: balk NOW, with height to work
            // with. The 22 ft flare-gate balk arrested nothing — the aircraft
            // still skimmed the ground at 91 kt before climbing (2026-07-22).
            balk = Some("off_centreline_low");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else if along_nm > STABILISED_MIN_ALONG_NM
            && radio_alt_ft >= TERRAIN_FLOOR_AGL_FT
            && !stabilised_xtk(xtk, along_nm)
        {
            // Inside the FAP but not somewhere a final can be flown from:
            // reposition to the join point out along the centreline, the
            // way the alt_err branch above does. "Land the aircraft" from
            // 1.5 nm abeam Gibraltar's 09 used to be called FINAL here — a
            // saturated 35 degree cut, the speed schedule pulling the RV-10
            // to its 65 kt "approach speed", and a stall (2026-09-11).
            let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
            (
                ApproachPhase::Intercept,
                bearing_deg(cur_lat, cur_lon, jlat, jlon),
            )
        } else if (TERRAIN_FLOOR_AGL_FT..STABILISED_GATE_AGL_FT).contains(&radio_alt_ft)
            && (STABILISED_MIN_ALONG_NM..FAP_NM).contains(&along_nm)
            && !crate::reversal::stabilisable(xtk, along_nm, gs_kts)
        {
            // THE STABILISED GATE, where a crew has one: a thousand feet.
            // Position alone called 1,880 m off at 3.3 nm "interceptable"
            // (a 35 degree cut does reach it), the 737 rode the glidepath
            // down beside the runway and the first lateral balk it met was
            // the flare gate's, at 94 ft and 207 m off; the second approach
            // got to 143 ft (EGTE, 2026-09-21). The pursuit converges in
            // TIME, so the question is the seconds of final left
            // (`reversal::predicted_residual_m`). The loop flies this one as
            // a go-around whatever the height.
            balk = Some("unstabilised_at_gate");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else if radio_alt_ft < TERRAIN_FLOOR_AGL_FT
            && (STABILISED_MIN_ALONG_NM..FAP_NM).contains(&along_nm)
            && !(stabilised_xtk(xtk, along_nm)
                && (width_m.is_none()
                    || lands_on_the_pavement(xtk, along_nm, gs_kts, landing_xtk_nm * M_PER_NM)))
        {
            // ...and "interceptable" is not stabilised either: a 35 degree
            // cut reaches 50 m from anywhere, and the 737 rode 30 to 50 m
            // beside a 45 m strip from 500 ft to the wheels (LEMG 13,
            // 2026-09-28). Below the floor the pursuit's own TIME must bring
            // the offset onto the pavement by the threshold.
            // Below the terrain floor and not laterally stabilised: a balk
            // with height in hand. That flare gate above was the ONLY lateral
            // balk, so a final that never converged (handed off 50 degrees
            // off at Gibraltar) flew to 95 ft still 306 m off the centreline,
            // went around with a 470 fpm sink and full flap, and hit the
            // North Front four seconds later (2026-09-11). Inside the FAP
            // only: further out the intercept is still legitimately wide,
            // and radio height alone means nothing over rising ground.
            balk = Some("unstabilised_below_floor");
            (ApproachPhase::Intercept, rwy.course_deg)
        } else {
            (
                ApproachPhase::Final,
                (rwy.course_deg + cut).rem_euclid(360.0),
            )
        }
    };

    // Track the path with the nominal descent for the current groundspeed
    // (`nominal_vs`, above) plus an error correction; bounded so it never dives.
    // SPEED PROTECTION overrides everything except the flare. Holding level below
    // the glidepath to "wait for the path" bleeds airspeed just as surely as
    // climbing does when the energy is marginal — that's the failure seen live:
    // stuck below the path 13 nm out, IAS oscillating down to 53 kt in a level
    // hold until the AP stalled off. A stall is fatal; being low is not. So when
    // airspeed decays below the approach speed, lower the nose and trade a little
    // altitude back for speed, proportional to how slow it is.
    // SAFE-ALTITUDE CLIMB: below the terrain floor and outside the flare zone, the
    // low radio altitude is rising terrain under the flight path — climb away from
    // it. But ONLY with surplus airspeed: climbing slow bleeds energy to a stall
    // (why this loop was descent-only). Low + slow falls through to speed
    // protection / level hold, and the raised throttle target below rebuilds the
    // surplus so the climb can engage on a later tick.
    // ESTABLISHED on the extended centreline beyond the FAP, inbound and on
    // or above the path: the descent is the final's, under the final's
    // floors, not the reposition's soft floor at circuit height. LEMG 13
    // (2026-10-05, 13:14:20 to 13:15:21): the pattern joined the path at
    // 10.7 nm, phase still Intercept (steering for the FAP), and the fade
    // keyed on radio height over the hills under the final cut the path's
    // -1,100 fpm to a dial jumping -210 to -1,400 that no VS mode follows:
    // on the path to 511 ft high in 60 s. The path itself was the pattern's
    // to clear (`reversal::plan`, 500 ft over its corridor).
    let established_path = established_on_final_path(phase, balk, along_nm, xtk, xtk_rate_nm_min, alt_err);
    let below_terrain_floor = match phase {
        ApproachPhase::Flare => false,
        _ if established_path => {
            radio_alt_ft < TERRAIN_FLOOR_AGL_FT && alt_err < FLOOR_BELOW_PATH_FT
        }
        // Final owns the descent to the runway — the floor stands down inside
        // short-final range where low radio altitude IS the approach, and it
        // never captures an aircraft ON the path (FLOOR_BELOW_PATH_FT: the
        // LXGB 09 overflight, 2026-07-22).
        ApproachPhase::Final => {
            radio_alt_ft < TERRAIN_FLOOR_AGL_FT
                && dist_nm > FLOOR_ENFORCE_MIN_DIST_NM
                && alt_err < FLOOR_BELOW_PATH_FT
        }
        // Intercept is repositioning, not landing: the floor holds at ANY
        // distance (see FLOOR_ENFORCE_MIN_DIST_NM for the KPDK 16 accident),
        // and at CIRCUIT height, not the bare terrain floor — a go-around
        // was vectoring back at 500-600 ft AGL ("is that a bit too low?",
        // 2026-07-22; it is).
        ApproachPhase::Intercept => radio_alt_ft < REPOSITION_MIN_AGL_FT,
    };
    let has_climb_energy = ias_kts > approach_ias + CLIMB_SPEED_MARGIN_KTS;

    // A SOFT floor under the descent, not the hard edge the climb gate is.
    // Repositioning wants REPOSITION_MIN_AGL_FT (~1,590 ft AGL), the 3° path
    // inside 5 nm sits below it, and the two laws met at a step: dive at 1,200
    // toward the path, cross the floor, climb at 800 back through it, repeat.
    // The Luton go-around (2026-07-28) rode that limit cycle for three and a
    // half minutes, porpoising 1,859 to 2,050 ft while wandering between 1.1
    // and 4.1 nm from the field. Fading the permitted descent to zero AT the
    // floor lands the aircraft on it instead of through it.
    let descent_cap_fpm = path_descent_cap_fpm(nominal_vs);
    let descent_limit_fpm = if phase == ApproachPhase::Intercept && !established_path {
        -((radio_alt_ft - REPOSITION_MIN_AGL_FT) * 2.0).clamp(0.0, -descent_cap_fpm)
    } else {
        descent_cap_fpm
    };

    // The roundout band's own gates, named once: the vertical law below owns
    // the descent from here, and the dial leads it (`arresting`).
    let roundout = phase == ApproachPhase::Final
        && !below_terrain_floor
        && in_flare_zone
        && xtk.abs() <= FLARE_MAX_XTK_NM
        && radio_alt_ft <= band_ft
        && roundout_reaches_pavement(along_nm, radio_alt_ft, gs_kts);

    // What the vertical path asks for, computed first so the speed-protection
    // branch below can be measured against it.
    let path_vs = match phase {
        // Arrest the descent near the ground.
        ApproachPhase::Flare => flare_vs,
        // Below the floor without the surplus to climb: hold level while the
        // throttle bonus rebuilds it — NEVER track the glidepath down. The
        // glidepath altitude is distance-symmetric, so a rejoin on the wrong
        // side of the threshold reads "on path" and descends (KPDK 16).
        _ if below_terrain_floor => 0.0,
        // ROUNDOUT: on final over the pavement, just above the flare gate,
        // keep coming down gently — never chase the path and never hold
        // level. Both wrong answers happened from the same arithmetic: past
        // the threshold the (unsigned) glidepath distance grows again, so
        // alt_err goes negative and the level-hold branch below fired at
        // 27 ft over the runway. The aircraft floated level, bled the very
        // airspeed the flare needed, and dropped on from the mush at
        // -380 fpm one knot above the clean stall (EGHI, 2026-08-02). A
        // staged descent — this, then the flare's -100 — arrives at the
        // flare with flying speed, which is what a soft touchdown is made
        // of.
        // ...but only where its own descent still REACHES the pavement.
        // `in_flare_zone`'s first term is an unsigned distance, so this
        // branch also fires SHORT of the threshold — right for a normal
        // landing (40 ft crosses the numbers from ~0.1 nm out), and a
        // commanded descent into the approach lights for an aircraft dragged
        // low: at 30 ft radio 0.3 nm short, -250 fpm touches down ~190 m
        // BEFORE the runway, where the old level-hold below let the path
        // come down to it (audit, 2026-08-02). The test is the physical one
        // — see `roundout_reaches_pavement`.
        ApproachPhase::Final if roundout => roundout_vs,
        // OVER THE PAVEMENT above the band: keep the path's own sink. The
        // glidepath altitude is distance-symmetric, so past the threshold
        // alt_err turns negative and the level-hold below fired at 60 ft
        // over EGCC 23L with a 737 sinking 730 fpm; the dial read zero, the
        // aircraft levelled at 30 ft and floated 500 m into the balloon
        // (2026-09-19). A jet that crosses the numbers high has runway to
        // spend and no height to hold; the float rule above still bounds
        // how far. Never steeper either: the aim point 300 m in is not
        // worth a dive from 90 ft.
        ApproachPhase::Final if along_nm < 0.0 && xtk.abs() <= FLARE_MAX_XTK_NM => nominal_vs,
        // On/above speed but below the glidepath: hold level and let the path
        // descend to us — don't climb to intercept. Speed protection above has
        // already handled the case where holding level would stall us.
        _ if alt_err < -50.0 => 0.0,
        // 1.2 fpm per ft of path error: the old 0.4 recovered 77 fpm
        // against the 190 ft the LXGB 09 final carried — it arrived at
        // the numbers still 190 ft high (2026-07-22). 1.2 converges a
        // 200 ft error inside the last mile; the clamp still forbids a
        // dive, and it is never shallower than the path itself plus a
        // catch-up (`path_descent_cap_fpm`).
        _ => (nominal_vs - alt_err * glidepath_gain_fpm_per_ft(gs_kts))
            .clamp(descent_cap_fpm, 0.0)
            .max(descent_limit_fpm),
    };

    let speed_deficit = approach_ias - ias_kts;
    let target_vs = if phase != ApproachPhase::Flare && speed_deficit > SPEED_FLOOR_MARGIN_KTS {
        // Nose down to regain speed — but NEVER shallower than the path was
        // already asking for. The protection rate is a fixed 40 fpm per knot
        // capped at 700, which on a slow airframe is steeper than the path and
        // on a fast one is not: a jet on a 3° path at 140 kt needs ~740 fpm, so
        // 9 knots slow produced a 360 fpm "protection" that HALVED the descent
        // and made the aircraft slower still, the exact opposite of the intent.
        // The MD-82 drifted above the path through the last mile that way and
        // crossed the threshold 51 ft high at 152 kt, which is the overflight
        // that forced the Luton go-around (2026-07-28).
        path_vs.min(-(speed_deficit * 40.0).clamp(200.0, 700.0))
    } else if below_terrain_floor && has_climb_energy {
        // Bounded climb toward THIS PHASE'S floor, steeper the deeper below
        // it. The magnitude used to reference the 500 ft terrain floor even
        // when the repositioning gate had armed the climb — above 500 AGL it
        // clamped to a 100 fpm dribble, and the LXGB go-around crept along
        // at ~650 ft into the Rock (2026-07-22).
        let floor_ft = match phase {
            ApproachPhase::Intercept => REPOSITION_MIN_AGL_FT,
            _ => TERRAIN_FLOOR_AGL_FT,
        };
        ((floor_ft - radio_alt_ft) * 2.0).clamp(100.0, max_climb_fpm())
    } else {
        path_vs
    };

    // Below the terrain floor, ask the throttle governor for extra speed so it
    // builds the surplus the climb gate needs. Otherwise fly the DECELERATION
    // SCHEDULE: clean out at range, approach speed by the FAP. The floor case
    // still wins, and the schedule can only ever ask for MORE speed than the
    // bare approach figure, so nothing downstream sees a slower target than it
    // used to.
    let target_ias = if below_terrain_floor {
        approach_ias + CLIMB_ENERGY_BONUS_KTS
    } else {
        scheduled_ias(dist_nm, approach_ias)
    };

    Guidance {
        target_heading_deg: target_heading,
        target_vs_fpm: target_vs,
        target_ias_kts: target_ias,
        dist_nm,
        glidepath_alt_ft,
        phase,
        balk,
        arresting: phase == ApproachPhase::Flare || roundout,
    }
}
