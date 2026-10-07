//! The go-around and missed approach: stages, latch, guidance, speed, thrust,
//! climb and flap.

use super::*;

/// Latched low go-around: a missed approach called BELOW the terrain floor is
/// flown wings-level STRAIGHT AHEAD at full power until clear of the ground —
/// never the immediate turn-back the intercept guidance commands. Seen live
/// 2026-07-14 (LXGB 09): overflew the numbers at ~57 ft, the guidance flipped
/// the heading command 178° toward the FAP, the flare branch re-fired 0.3 nm
/// PAST the threshold (`dist_nm` is unsigned, so the flare gate re-arms on the
/// departure side) and pulled the power, then speed protection commanded
/// -590 fpm at ground level — impact 200 m south of the strip. The latch owns
/// the whole low go-around: it arms when Final/Flare degrades to Intercept
/// below [`TERRAIN_FLOOR_AGL_FT`], holds through every phase flicker the raw
/// guidance produces down there, and releases at the floor, where the normal
/// intercept guidance takes over the turn back to the FAP.
/// Where a balked landing is in the missed approach. Two stages, because a
/// missed approach is a PROCEDURE, not a moment: releasing the aircraft back to
/// the ordinary intercept guidance the instant it was clear of the ground left
/// it 800 ft over the far end with the field behind it and no plan, and the
/// stateless guidance then argued with itself — see `descent_limit_fpm` for the
/// vertical half of the Luton orbit (2026-07-28) and `Reposition` for the
/// lateral half.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissedStage {
    /// Straight ahead on runway heading, climbing away from the ground. The one
    /// direction a runway promises is flyable.
    Climb,
    /// Repositioning: outbound to the join point at platform height, held there
    /// until established OUTSIDE the FAP on the approach side — which is simply
    /// where a normal approach begins.
    ///
    /// This stage is what makes the decision STICKY, and stickiness is the
    /// whole point. The ordinary guidance picks its target from the CURRENT
    /// altitude error, so at reposition height the choice flips every time the
    /// aircraft crosses the distance where the path is 500 ft below it: turn
    /// out, fly out, the error shrinks, turn back in, the error grows, turn out
    /// again. At Luton that oscillation ran between 1.1 and 4.1 nm for three
    /// and a half minutes with a landing clearance live and the tower working
    /// arrivals underneath. An outbound leg has to run to completion.
    Reposition,
}

#[derive(Debug, Default)]
pub struct GoAroundLatch {
    pub(super) stage: Option<MissedStage>,
    /// How many go-arounds this approach has flown — a runaway backstop. An
    /// approach that keeps missing (arrives too fast/high to capture, as the
    /// LXGB Twin Otter did off the tight circuit) would otherwise firewall a
    /// climb, release at 800 ft, re-intercept, miss again — forever, accelerating
    /// toward Vne. After [`MAX_GO_AROUNDS`] the loop gives the aircraft back.
    pub(super) attempts: u32,
    /// The reposition is being flown as a pattern by the loop
    /// ([`crate::reversal`]) and ends when the pattern does
    /// ([`GoAroundLatch::finish_reposition`]), not at the join point: the
    /// pattern's downwind passes abeam it, outbound, inside the old release
    /// test's half mile on a light single.
    pub(super) hold: bool,
    /// An unstabilised go-around climbs over the gate it failed before the
    /// approach may be judged again: feet above the runway, and the last
    /// height read (`note_height_aal`).
    pub(super) climb_floor_aal_ft: Option<f64>,
    pub(super) height_aal_ft: Option<f64>,
}

/// Over the gate a go-around failed, feet: the next approach crosses the
/// gate descending, never from below it.
pub const GATE_CLIMB_MARGIN_FT: f64 = 200.0;

/// Give up (level off, hand to the pilot) after this many go-arounds on one
/// approach rather than loop a missed approach until the fuel runs out.
pub const MAX_GO_AROUNDS: u32 = 2;
// The counter above cannot catch a lost reposition, because a second
// go-around needs another approach to fail first. A single missed approach
// is given up when it stops making progress (`reversal::MissedWatch`,
// 2026-09-30), never on a flat clock: 300 s expired with EGHI's Baron
// compliant mid-leg (2026-08-02), and the 600 s that replaced it gave up an
// SR22 on LEMG 13's terrain-raised pattern with the path a few seconds ahead
// (iMac 2026-09-30).

impl GoAroundLatch {
    pub fn new() -> Self {
        Self::default()
    }

    /// Go-arounds flown so far this approach.
    pub fn attempts(&self) -> u32 {
        self.attempts
    }

    /// Is a missed approach being flown right now?
    pub fn active(&self) -> bool {
        self.stage.is_some()
    }

    /// ...and still in its straight-ahead climb away from the ground?
    pub fn climbing(&self) -> bool {
        self.stage == Some(MissedStage::Climb)
    }

    /// The loop flies the reposition as a pattern: hold the stage until told.
    pub fn hold_reposition(&mut self) {
        self.hold = true;
    }

    /// The pattern is complete and the aircraft is inbound on the final: the
    /// missed approach is over and the ordinary approach has it.
    pub fn finish_reposition(&mut self) {
        self.hold = false;
        if self.stage == Some(MissedStage::Reposition) {
            self.stage = None;
        }
    }

    /// A controller ORDERED a go-around: fly it, now, from wherever we are.
    ///
    /// Every other entry to a missed approach is something the aircraft
    /// noticed about itself (off the centreline at flare height, past the
    /// numbers still airborne). This one comes from outside and outranks all
    /// of it — the tower can see the runway and we cannot. Returns whether
    /// this call started one, so the caller records it once.
    ///
    /// Malaga, 2026-08-05: "cleared to land" at 13:41:06 and "GO AROUND,
    /// traffic on the runway" at 13:41:07. The order was published on the
    /// party line and SPOKEN, and nothing consumed it — the approach flew on
    /// for another seventy seconds, past a 0.5 nm same-altitude conflict, and
    /// only balked because it happened to arrive high. The clearance the
    /// order voided was never re-earned, so the aircraft then landed without
    /// one, could not vacate for want of it, and sat on the runway with an
    /// arrival inside two miles. One unread flag cost all of that.
    ///
    /// Ignored once on the ground: a landing that has already happened is not
    /// a landing to abandon, and the rollout is the safer place to be.
    pub fn order(&mut self, on_ground: bool) -> bool {
        if on_ground || self.stage.is_some() {
            return false;
        }
        self.stage = Some(MissedStage::Climb);
        self.attempts += 1;
        true
    }

    /// An UNSTABILISED approach's go-around: [`Self::order`], and the climb
    /// holds until the aircraft is [`GATE_CLIMB_MARGIN_FT`] over `gate_ft`
    /// above the runway as well as clear of the ground. SR22 LEMG 13 (iMac
    /// 2026-09-30, 1790792031): placed diving, it failed the 1,000 ft gate
    /// at 4 nm below the path; over the hills the radio already read 800 ft,
    /// so the climb ended the next tick, the reposition released at once (the
    /// path's join point ahead of an aircraft under it), the gate judged the
    /// same final again, and three go-arounds in 8 s gave it up airborne.
    pub fn order_over_gate(&mut self, on_ground: bool, gate_ft: f64) -> bool {
        let started = self.order(on_ground);
        if started {
            self.climb_floor_aal_ft = Some(gate_ft + GATE_CLIMB_MARGIN_FT);
        }
        started
    }

    /// The height above the runway this tick, for the gate's climb floor.
    pub fn note_height_aal(&mut self, aal_ft: f64) {
        self.height_aal_ft = aal_ft.is_finite().then_some(aal_ft);
    }

    /// Advance the latch one tick: `prev`/`now` are the raw guidance phases,
    /// `radio_ft` the radio altitude, `dist_nm` the distance to the threshold,
    /// `along_nm` the signed along-track (positive = approach side),
    /// `join_nm` the [`join_point_nm`] the reposition leg is flying out to
    /// and `xtk_nm` the cross-track from the extended centreline.
    /// Returns the stage being flown, `None` when the approach is the pilot's
    /// normal one again.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        prev: Option<ApproachPhase>,
        now: ApproachPhase,
        radio_ft: f64,
        dist_nm: f64,
        along_nm: f64,
        join_nm: f64,
        xtk_nm: f64,
    ) -> Option<MissedStage> {
        match self.stage {
            // Clear of the ground — stop climbing straight ahead and start
            // repositioning. 800 ft, not the 500 ft terrain floor: the turn
            // back is the leg that crosses whatever the runway points away
            // from, and releasing at 500 put the LXGB go-around eastbound at
            // the Rock with no height in hand (2026-07-22).
            Some(MissedStage::Climb)
                if radio_ft >= GO_AROUND_TURN_AGL_FT
                    && self
                        .climb_floor_aal_ft
                        .is_none_or(|f| self.height_aal_ft.is_some_and(|h| h >= f)) =>
            {
                self.stage = Some(MissedStage::Reposition);
                self.climb_floor_aal_ft = None;
            }
            // Out at the join point on the approach side: that is where an
            // approach starts, so hand back. Releasing merely OUTSIDE the FAP
            // would hand back mid-outbound-leg, pointing away from the field,
            // and leave the ordinary guidance to fly a 180 on final.
            // AT the join point, not merely abeam it: the leg homes on the
            // point, and a fast aircraft's turn-back leaves it miles beside
            // the centreline when the distance test first passes. The 737
            // was released 2.7 nm abeam at 6.7 nm, called "final" 1.9 nm off
            // (a 19 degree cut is interceptable on paper), and the pursuit's
            // 30 s time constant had 110 s of final to close 3.5 km: it
            // balked at 95 ft still 84 m off (LFLL 35L, 2026-09-17).
            Some(MissedStage::Reposition)
                if !self.hold
                    && along_nm > 0.0
                    && dist_nm >= join_nm - 0.2
                    && xtk_nm.abs() <= REPOSITION_RELEASE_XTK_NM =>
            {
                self.stage = None;
            }
            None if now == ApproachPhase::Intercept
                && matches!(prev, Some(ApproachPhase::Final | ApproachPhase::Flare))
                && radio_ft < TERRAIN_FLOOR_AGL_FT =>
            {
                self.stage = Some(MissedStage::Climb);
                self.attempts += 1;
            }
            _ => {}
        }
        self.stage
    }
}

/// Where a descent from `cur_alt_ft` meets the extended centreline: the point
/// to be established at before flying the path down, never closer than the FAP.
/// The 1.15 is margin — arriving exactly on a 3° path with no slack leaves
/// nothing for the capture.
pub fn join_point_nm(cur_alt_ft: f64, rwy_elev_ft: f64) -> f64 {
    ((cur_alt_ft - rwy_elev_ft) / gp_ft_per_nm() * 1.15).clamp(FAP_NM, 40.0)
}

/// The command a latched go-around flies.
///
/// [`MissedStage::Climb`] is runway heading (straight ahead, no turn near the
/// ground) and a bounded climb once the speed is there — level acceleration
/// first, because climbing slow bleeds energy to a stall.
///
/// [`MissedStage::Reposition`] steers OUTBOUND to the join point down the
/// extended centreline and holds platform height there. It does not chase the
/// glidepath: the path inside the join point is the one this leg is flying away
/// from, and descending toward it is what turned the Luton missed approach into
/// an orbit. Levelling at platform height means arriving at the FAP already on
/// the path, which is the whole reason the reposition altitude is defined as
/// the path height at the FAP.
///
/// Both stages ask the throttle governor for a margin over the approach speed
/// (the loop firewalls it anyway while latched; the target keeps the governor
/// sane on the release tick).
#[allow(clippy::too_many_arguments)]
pub fn missed_approach_guidance(
    rwy: &Runway,
    stage: MissedStage,
    cur_lat: f64,
    cur_lon: f64,
    cur_alt_ft: f64,
    radio_alt_ft: f64,
    dist_nm: f64,
    glidepath_alt_ft: f64,
    ias_kts: f64,
    approach_ias: f64,
    ias_trend_kts_s: f64,
    floor_kts: Option<f64>,
) -> Guidance {
    let slow = approach_ias - ias_kts > SPEED_FLOOR_MARGIN_KTS;
    let (target_heading_deg, target_vs_fpm) = match stage {
        MissedStage::Climb => (
            rwy.course_deg,
            go_around_vs_fpm(
                ias_kts,
                ias_trend_kts_s,
                approach_ias,
                floor_kts,
                radio_alt_ft,
                max_climb_fpm(),
            ),
        ),
        MissedStage::Reposition => {
            // Same join point the ordinary guidance uses, so the handback is
            // continuous.
            let join_nm = join_point_nm(cur_alt_ft, rwy.elev_ft);
            let recip = (rwy.course_deg + 180.0) % 360.0;
            let (jlat, jlon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, join_nm);
            let vs = if radio_alt_ft < REPOSITION_MIN_AGL_FT - 50.0 && !slow {
                ((REPOSITION_MIN_AGL_FT - radio_alt_ft) * 2.0).clamp(100.0, max_climb_fpm())
            } else {
                0.0
            };
            (bearing_deg(cur_lat, cur_lon, jlat, jlon), vs)
        }
    };
    Guidance {
        target_heading_deg,
        target_vs_fpm,
        target_ias_kts: approach_ias + CLIMB_ENERGY_BONUS_KTS,
        dist_nm,
        glidepath_alt_ft,
        phase: ApproachPhase::Intercept,
        balk: None,
        arresting: false,
    }
}

/// The flap a go-around cleans up to (the loop's go-around flap).
pub(super) const GO_AROUND_FLAP: f64 = 0.5;
/// The missed approach holds go-around thrust until this high above the
/// ground ...
pub(super) const GA_THRUST_HOLD_AGL_FT: f64 = 400.0;
/// ... and climbing at least this, fpm: then it is established.
pub(super) const GA_CLIMB_ESTABLISHED_FPM: f64 = 500.0;
/// Established and this far over the go-around speed, the thrust comes back
/// at once to the climb floor, not a governor step a tick: at 0.04 a tick the
/// lever takes twenty seconds from 1.0, and the clean 737 gains 3 kt/s.
pub(super) const GA_FAST_KTS: f64 = 10.0;
/// ...and this far under it, go-around thrust again.
pub(super) const GA_SLOW_KTS: f64 = 10.0;

/// The speed a missed approach flies, kt: the give-up's pattern speed for the
/// flap out (`hand_back_speed_kts`, the clean approach speed with the flaps
/// up), at least the approach speed plus the climb margin and fast enough for
/// the go-around flap to come up (`go_around_floor_kts`, the floor of
/// [`GO_AROUND_FLAP`], plus 5), never within 10 kt of the placard of the flap
/// out, never over 250 below 10,000 ft (`alt_ft`), never under the floor of
/// the configuration flown (`config_floor_kts`, which wins). EGNX 09
/// (2026-09-30, #199): the clean 737 flew its missed approach at full power
/// from 250 to 355 kt below 1,700 ft; this is 210. Pure.
pub fn missed_approach_speed_kts(
    approach_ias: f64,
    flap_ratio: f64,
    flap_limits: &[(f64, Option<f64>)],
    go_around_floor_kts: Option<f64>,
    config_floor_kts: Option<f64>,
    alt_ft: f64,
) -> f64 {
    let placard = flap_placard_kts(flap_ratio, flap_limits).unwrap_or(f64::INFINITY);
    let mut t = hand_back_speed_kts(approach_ias, flap_ratio, flap_limits)
        .max(approach_ias + CLIMB_ENERGY_BONUS_KTS)
        .max(go_around_floor_kts.map_or(0.0, |f| f + 5.0))
        .min(placard - PLACARD_TARGET_MARGIN_KTS);
    if alt_ft < crate::tools::speedhold::SPEED_LIMIT_ALT_FT {
        t = t.min(crate::tools::speedhold::BELOW_10K_LIMIT_KTS);
    }
    crate::aircraft::floored_target(t, config_floor_kts)
}

/// The lever a missed approach's climb flies. Go-around thrust until the
/// climb is ESTABLISHED ([`GA_THRUST_HOLD_AGL_FT`] up and climbing at
/// [`GA_CLIMB_ESTABLISHED_FPM`]; an unread climb is not established), and
/// whenever [`GA_SLOW_KTS`] slow; then the speed: more than [`GA_FAST_KTS`]
/// over `target_kts`, the airframe's climb floor at once (`climb_floor`, its
/// approach thrust, else idle: the VS mode holds the climb and the excess
/// becomes height), else the speed governor's step, inside the floor and
/// full. A crew's go-around: TOGA, positive climb, gear and flap, then the
/// thrust set for the go-around speed. Pure.
pub fn missed_approach_throttle(
    throttle_now: f64,
    ias_kts: f64,
    trend_kts_s: f64,
    target_kts: f64,
    radio_ft: f64,
    vvi_fpm: Option<f64>,
    climb_floor: Option<f64>,
) -> f64 {
    let established = radio_ft >= GA_THRUST_HOLD_AGL_FT
        && vvi_fpm.is_some_and(|v| v >= GA_CLIMB_ESTABLISHED_FPM);
    if !established || ias_kts < target_kts - GA_SLOW_KTS {
        return 1.0;
    }
    let floor = climb_floor.unwrap_or(IDLE_THROTTLE).max(IDLE_THROTTLE);
    if ias_kts > target_kts + GA_FAST_KTS {
        return floor;
    }
    let trend = if trend_kts_s.is_finite() { trend_kts_s } else { 0.0 };
    (throttle_now + crate::tools::speedhold::throttle_adjust_live(target_kts - ias_kts, trend))
        .clamp(floor, 1.0)
}

/// A go-around judges the speed it is HEADING for: the airspeed this many
/// seconds ahead on its trend.
pub(super) const GA_SPEED_LEAD_S: f64 = 2.0;
/// Below the floor, the sink asked per knot short of it (fpm per kt).
pub(super) const GA_FPM_PER_KT: f64 = 100.0;
/// ...and never more sink than reaches the ground in this long: height is
/// traded for speed only where there is height to trade.
pub(super) const GA_GROUND_S: f64 = 10.0;

/// The vertical speed a slow go-around flies: PITCH FOR SPEED FIRST (issue
/// #77). Not slow (the projected airspeed within [`SPEED_FLOOR_MARGIN_KTS`]
/// of the approach speed): `climb_fpm`. Slow but at or over the floor (1.2
/// Vs in the configuration flown, [`crate::aircraft::config_floor_kts`]):
/// level, the thrust builds the speed. Under the floor: a descent of
/// [`GA_FPM_PER_KT`] per knot short, never steeper than the ground ten
/// seconds away ([`GA_GROUND_S`]), so near the runway it is almost level and
/// on it level. The old law asked 0 fpm whatever the speed, and a VS mode
/// holds 0 fpm by pitching up until the wing stalls: EDLP 24 (2026-09-24)
/// the SF50 balked `balloon` at 52 ft climbing 860 fpm at 81 kt on idle, was
/// given +1,500 then 0, bled to 74 kt with the go-around's half flap, and
/// stalled onto the runway at -1,568 fpm. `floor_kts` unknown: the approach
/// speed's own 1.2/1.3. Pure.
pub fn go_around_vs_fpm(
    ias_kts: f64,
    trend_kts_s: f64,
    approach_ias: f64,
    floor_kts: Option<f64>,
    radio_ft: f64,
    climb_fpm: f64,
) -> f64 {
    let trend = if trend_kts_s.is_finite() {
        trend_kts_s.clamp(-10.0, 10.0)
    } else {
        0.0
    };
    let projected = ias_kts + trend * GA_SPEED_LEAD_S;
    if !(approach_ias - projected > SPEED_FLOOR_MARGIN_KTS) {
        return climb_fpm;
    }
    let floor = floor_kts
        .filter(|f| f.is_finite() && *f > 0.0)
        .unwrap_or(approach_ias * crate::aircraft::SPEED_FLOOR_RATIO / 1.3);
    let give = -(radio_ft.max(0.0) / GA_GROUND_S * 60.0);
    (GA_FPM_PER_KT * (projected - floor)).clamp(give.min(0.0), 0.0)
}

/// The go-around flap comes up only at the speed that flies it: its floor
/// (1.2 Vs in that configuration). EDLP 24 (2026-09-24): half flap at 81 kt
/// put the SF50's stall at 74, where the go-around met it. Pure.
pub fn go_around_flap_due(ias_kts: f64, floor_kts: Option<f64>) -> bool {
    match floor_kts {
        Some(f) if f.is_finite() => ias_kts >= f,
        _ => true,
    }
}
