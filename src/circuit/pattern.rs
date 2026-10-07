//! The pattern chosen: the corridors, the descent shape and check, the side
//! and the downwind picked over the ground.

use super::*;

/// How far outside (and inside) the drawn rectangle the pattern is probed, in
/// turn radii: the autopilot does not fly the line. Measured from every
/// jet circuit logged (`circuit` `leg` records against their `armed`
/// geometry, 17 circuits of the 737 and A330 at LEMG and LIEA, r 2,500 m,
/// 2026-09-26 to 09-29): the downwind was flown 930 to 2,200 m OUTSIDE its
/// line (median 1,330, 0.53 r), once 3,170 (LEMG 30, sortie 1790633832,
/// 1.27 r), and never inside it; the base ran 320 to 840 m past
/// `downwind_past_m` (1,525 on that 30). The SF50 (r 1,885) flew 250 m
/// INSIDE (0.13 r). So copies at 0.5 and 1.0 radii out cover the median and
/// all but the worst, and 0.25 in covers the tight airframe. The LEMG 13
/// base turn that went 305 ft over the ground (1790599750) was 1,200 m
/// outside the corner, between the two outer copies.
pub const CORRIDOR_OUT_RADII: [f64; 2] = [0.5, 1.0];
pub const CORRIDOR_IN_RADII: f64 = 0.25;
/// Sample spacing on the corridor's copies, metres: twice the line's
/// [`LEG_SAMPLE_M`], so the probe stays a few seconds on the runway. The
/// line itself keeps its 150 m (the Rock's 300 m ridge).
pub(super) const CORRIDOR_SAMPLE_M: f64 = 300.0;

/// The least height over the ground a DESCENDING leg (the downwind from abeam
/// the threshold, base, final) may be commanded to, from
/// [`DESCENT_FLOOR_FULL_NM`] out, tapering in a straight line to nothing at
/// the threshold (#182). 500 ft is SERA.5005(f)(2)'s minimum height away
/// from congested areas; the taper is because the 3 degree path itself is
/// only 636 ft over a level field at 2 nm (250 ft a mile against the path's
/// 318, so a level field always passes and a hill on final does not). The
/// old law descended to 800 ft over the FIELD from abeam the threshold and
/// never looked at the ground: LEMG 13's base went 305 ft over a hill 900 ft
/// high (sortie 1790599750), the LEMG 30 downwind 475 (1790633832).
pub(super) const DESCENT_FLOOR_FT: f64 = 500.0;
pub(super) const DESCENT_FLOOR_FULL_NM: f64 = 2.0;
/// The steepest path the circuit will fly to clear the ground on its base and
/// final: PANS-OPS's upper limit for a procedure that is not a steep approach
/// (anything past it needs an approval no profile declares). Autoland is
/// handed the same angle (`autoland::HandDown::circuit`).
pub const STEEP_PATH_MAX_DEG: f64 = 3.5;
/// A raise smaller than this is carried out but not said: with 1,000 ft of
/// clearance a 1,000 ft pattern lifts over any ground above the field, and
/// "flying it at 1,030 feet" is noise.
pub(super) const RAISE_SAID_FT: f64 = 100.0;
/// With both sides needing a raise, the asked-for side is kept unless the
/// other one is lower by more than this.
pub(super) const RAISE_SIDE_MARGIN_FT: f64 = 100.0;

/// Feet of path per nautical mile at `deg`.
pub(super) fn gp_ft_per_nm_at(deg: f64) -> f64 {
    6076.12 * deg.to_radians().tan()
}

/// The descent the pattern flies from abeam the threshold: the downwind's
/// platform (feet over the field) and the path base and final follow.
/// The default is the law as it always flew (800 ft, 3 degrees); the pick
/// raises either only where the ground under the pattern asks for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DescentShape {
    pub platform_agl_ft: f64,
    pub gp_ft_per_nm: f64,
}

impl Default for DescentShape {
    fn default() -> Self {
        DescentShape {
            platform_agl_ft: DOWNWIND_DESCENT_AGL_FT,
            gp_ft_per_nm: crate::autoland::GLIDEPATH_FT_PER_NM,
        }
    }
}

impl DescentShape {
    /// The path's angle, degrees.
    pub fn glidepath_deg(&self) -> f64 {
        (self.gp_ft_per_nm / 6076.12).atan().to_degrees()
    }

    /// Steeper than the standard 3 degrees.
    pub fn steep(&self) -> bool {
        self.gp_ft_per_nm > crate::autoland::GLIDEPATH_FT_PER_NM + 0.5
    }
}

/// Which part of the descent a ground sample sits under.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DescentLeg {
    /// The downwind from abeam the threshold: the flat platform.
    Platform,
    /// Base, the turn onto it included (the leg starts at the downwind's
    /// led turn), and final: the lower of the platform and the path at
    /// `path_m`.
    Base,
    Final,
}

impl DescentLeg {
    pub(super) fn on_path(self) -> bool {
        !matches!(self, DescentLeg::Platform)
    }

    pub(super) fn words(self) -> &'static str {
        match self {
            DescentLeg::Platform => "the downwind",
            DescentLeg::Base => "the base",
            DescentLeg::Final => "the final",
        }
    }
}

/// The ground under one point of a descending leg, with the distances its
/// target and its floor are judged on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundSample {
    pub leg: DescentLeg,
    /// The distance the leg's path target is computed on ([`path_dist_m`]).
    pub path_m: f64,
    /// Straight-line distance to the threshold, for the floor's taper.
    pub thr_m: f64,
    pub ground_ft: f64,
}

/// The floor a descending leg keeps over the ground `thr_m` from the
/// threshold ([`DESCENT_FLOOR_FT`]). Pure.
pub fn descent_floor_ft(thr_m: f64) -> f64 {
    DESCENT_FLOOR_FT * (thr_m.abs() / M_PER_NM / DESCENT_FLOOR_FULL_NM).clamp(0.0, 1.0)
}

/// The altitude the pattern commands over a descent sample, feet MSL: what
/// [`leg_target_shaped`] ratchets to there (the platform, then the lower of
/// it and the path). Pure.
pub fn descent_target_ft(
    s: &GroundSample,
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    shape: DescentShape,
) -> f64 {
    let platform = (field_elev_ft + shape.platform_agl_ft).min(tpa_msl_ft);
    match s.leg {
        DescentLeg::Platform => platform,
        DescentLeg::Base | DescentLeg::Final => platform.min(approach_leg_alt_on(
            field_elev_ft,
            tpa_msl_ft,
            s.path_m,
            shape.gp_ft_per_nm,
        )),
    }
}

/// The lowest point of a descent flown with `shape`: (clearance over the
/// ground, feet, less its floor; the sample). `None` without samples. Pure.
pub(super) fn worst_descent_sample(
    samples: &[GroundSample],
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    shape: DescentShape,
) -> Option<(f64, GroundSample)> {
    samples
        .iter()
        .map(|s| {
            let over = descent_target_ft(s, field_elev_ft, tpa_msl_ft, shape) - s.ground_ft;
            (over - descent_floor_ft(s.thr_m), *s)
        })
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))
}

/// How a side's descent from abeam the threshold clears its ground.
#[derive(Debug, Clone, PartialEq)]
pub struct DescentCheck {
    /// The shape to fly: the default, or raised (platform, then a steeper
    /// path up to [`STEEP_PATH_MAX_DEG`]) to keep every sample over its
    /// floor. `None`: nothing legal clears it.
    pub shape: Option<DescentShape>,
    /// The least height over the ground the DEFAULT descent would have
    /// flown, feet (the number the old law never looked at), and where.
    pub default_min_ft: Option<f64>,
    /// ...and with the shape flown.
    pub flown_min_ft: Option<f64>,
    /// Spoken when refused: the worst point, no dashes.
    pub problem: Option<String>,
}

/// Check a side's descending legs at pattern altitude `tpa_msl_ft` and find
/// the least change that clears them: the downwind platform raised to the
/// highest sample's need (never over the pattern altitude), the path steepened
/// to the need of every base and final sample (never past
/// [`STEEP_PATH_MAX_DEG`]). No samples: the default, the old behaviour (no
/// probe, no verdict). Pure.
pub fn check_descent(
    samples: &[GroundSample],
    field_elev_ft: f64,
    tpa_msl_ft: f64,
) -> DescentCheck {
    check_descent_from(samples, field_elev_ft, tpa_msl_ft, DescentShape::default())
}

/// [`check_descent`] from a `default` descent other than the circuit's (its
/// 800 ft downwind platform): autoland's reversal holds its pattern altitude
/// until the path meets it (`reversal::judge`, #182). Pure.
pub fn check_descent_from(
    samples: &[GroundSample],
    field_elev_ft: f64,
    tpa_msl_ft: f64,
    default: DescentShape,
) -> DescentCheck {
    let over_ground = |shape: DescentShape| {
        samples
            .iter()
            .map(|s| descent_target_ft(s, field_elev_ft, tpa_msl_ft, shape) - s.ground_ft)
            .fold(None, |m: Option<f64>, v| Some(m.map_or(v, |m| m.min(v))))
    };
    let default_min_ft = over_ground(default);
    let Some((worst, at)) = worst_descent_sample(samples, field_elev_ft, tpa_msl_ft, default)
    else {
        return DescentCheck {
            shape: Some(default),
            default_min_ft: None,
            flown_min_ft: None,
            problem: None,
        };
    };
    if worst >= 0.0 {
        return DescentCheck {
            shape: Some(default),
            default_min_ft,
            flown_min_ft: default_min_ft,
            problem: None,
        };
    }
    let need = |s: &GroundSample| s.ground_ft + descent_floor_ft(s.thr_m);
    let platform_need = samples
        .iter()
        .map(need)
        .fold(field_elev_ft + default.platform_agl_ft, f64::max);
    let gp_need = samples
        .iter()
        .filter(|s| s.leg.on_path() && s.path_m > 1.0)
        .map(|s| (need(s) - field_elev_ft) / (s.path_m / M_PER_NM))
        .fold(default.gp_ft_per_nm, f64::max);
    let gp_max = gp_ft_per_nm_at(STEEP_PATH_MAX_DEG);
    if platform_need > tpa_msl_ft + 1.0 || gp_need > gp_max + 1e-6 {
        let over = descent_target_ft(&at, field_elev_ft, tpa_msl_ft, default) - at.ground_ft;
        let tail = if platform_need > tpa_msl_ft + 1.0 {
            "clearing it needs more than the pattern altitude".to_string()
        } else {
            format!(
                "clearing it needs a {:.1} degree path",
                (gp_need / 6076.12).atan().to_degrees()
            )
        };
        let problem = format!(
            "{} would pass {:.0} feet over {:.0} feet of ground {:.1} miles out, {tail}",
            at.leg.words(),
            (over.max(0.0) / 10.0).round() * 10.0,
            (at.ground_ft / 10.0).round() * 10.0,
            at.thr_m / M_PER_NM,
        );
        return DescentCheck {
            shape: None,
            default_min_ft,
            flown_min_ft: None,
            problem: Some(problem),
        };
    }
    let shape = DescentShape {
        platform_agl_ft: (platform_need - field_elev_ft).max(default.platform_agl_ft),
        gp_ft_per_nm: gp_need,
    };
    DescentCheck {
        shape: Some(shape),
        default_min_ft,
        flown_min_ft: over_ground(shape),
        problem: None,
    }
}

/// What a candidate pattern side knows about the world under and around it:
/// the highest sampled terrain under the legs flown AT pattern altitude,
/// corridor included (`None` = no probe data), the first no-go airspace
/// (prohibited/restricted/danger) its polyline enters at pattern height
/// (`None` = clear or no data), and the ground under its descending legs
/// (empty = no probe data, the old behaviour). `line_max_ft` is the drawn
/// rectangle alone, the number the old check used, for the record.
#[derive(Debug, Clone, Default)]
pub struct SideFacts {
    pub terrain_max_ft: Option<f64>,
    pub airspace: Option<String>,
    pub line_max_ft: Option<f64>,
    pub descent: Vec<GroundSample>,
}

/// The pattern the pick settled on.
#[derive(Debug, Clone, PartialEq)]
pub struct PatternPick {
    pub left: bool,
    pub tpa_msl_ft: f64,
    pub why: &'static str,
    /// The descent flown from abeam the threshold.
    pub shape: DescentShape,
    /// Each side's descent check at the altitude it was judged at (left,
    /// right), for the record.
    pub descent_left: DescentCheck,
    pub descent_right: DescentCheck,
    /// `Some`: no side can be flown clear of the ground. The spoken reason,
    /// with the numbers; the circuit is never flown.
    pub refused: Option<String>,
}

/// Decide the pattern side, altitude and descent — the generic rule, no
/// per-airport exceptions: fly the side clear of terrain (level legs by
/// [`PATTERN_TERRAIN_CLEAR_FT`], the descent by [`check_descent`]) and of
/// no-go airspace; switch sides when only the other one is; when neither is
/// clean, prefer the side without an airspace block, raised over its terrain;
/// when airspace is unavoidable, fall back to terrain-safety alone (terrain
/// kills, paper doesn't) and say so; and when no side's descent can be flown
/// clear, REFUSE with the numbers (#182). Missing data degrades gracefully:
/// `None` terrain = assume clear (the old behavior), `None` airspace = clear,
/// no descent samples = the default descent. Pure.
pub fn choose_pattern(
    default_left: bool,
    l: &SideFacts,
    r: &SideFacts,
    field_elev_ft: f64,
    tpa_agl_ft: f64,
) -> PatternPick {
    choose_pattern_with(
        default_left,
        l,
        r,
        field_elev_ft,
        tpa_agl_ft,
        PickRules::circuit(tpa_agl_ft),
    )
}

/// How [`choose_pattern_with`] raises a side and where its descent starts:
/// the circuit's rectangle, or autoland's reversal (`reversal::judge`, #182),
/// one rule behind both.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PickRules {
    /// A raised side flies this far over the highest ground under its level
    /// legs, at least [`PATTERN_TERRAIN_CLEAR_FT`].
    pub raise_over_ft: f64,
    /// The descent starts from the pattern altitude itself (the reversal's
    /// inbound holds its platform until the 3 degree path comes down to it),
    /// not from the circuit's 800 ft downwind platform.
    pub descend_from_pattern: bool,
}

impl PickRules {
    /// The circuit: a raised pattern keeps a pattern height over the ground.
    pub fn circuit(tpa_agl_ft: f64) -> Self {
        PickRules {
            raise_over_ft: tpa_agl_ft.max(PATTERN_TERRAIN_CLEAR_FT),
            descend_from_pattern: false,
        }
    }

    /// Autoland's reversal: the lowest legal platform (its join point moves
    /// out with every foot of it), descending on the path from it.
    pub fn reversal() -> Self {
        PickRules {
            raise_over_ft: PATTERN_TERRAIN_CLEAR_FT,
            descend_from_pattern: true,
        }
    }
}

/// [`choose_pattern`] under `rules`. Pure.
pub fn choose_pattern_with(
    default_left: bool,
    l: &SideFacts,
    r: &SideFacts,
    field_elev_ft: f64,
    tpa_agl_ft: f64,
    rules: PickRules,
) -> PatternPick {
    let tpa = field_elev_ft + tpa_agl_ft;
    let level_ok = |f: &SideFacts, at: f64| {
        f.terrain_max_ft
            .is_none_or(|m| at - m >= PATTERN_TERRAIN_CLEAR_FT - 0.5)
    };
    // Only as high as the level legs need, and at least a pattern height
    // over the ground: a 1,000 ft clearance under an 800 ft pattern would
    // otherwise leave it lower than the rule.
    let raise = |f: &SideFacts| match f.terrain_max_ft {
        Some(m) => (m + rules.raise_over_ft.max(PATTERN_TERRAIN_CLEAR_FT)).max(tpa),
        None => tpa,
    };
    let side_facts = |left: bool| if left { l } else { r };
    let from = |at: f64| {
        if rules.descend_from_pattern {
            DescentShape {
                platform_agl_ft: at - field_elev_ft,
                ..DescentShape::default()
            }
        } else {
            DescentShape::default()
        }
    };
    let descent = |left: bool, at: f64| {
        check_descent_from(&side_facts(left).descent, field_elev_ft, at, from(at))
    };
    let pick = |left: bool, at: f64, why: &'static str, shape: DescentShape| PatternPick {
        left,
        tpa_msl_ft: at,
        why,
        shape,
        descent_left: descent(true, if left { at } else { raise(l) }),
        descent_right: descent(false, if left { raise(r) } else { at }),
        refused: None,
    };
    let clean = |left: bool| {
        let f = side_facts(left);
        level_ok(f, tpa) && f.airspace.is_none()
    };
    let df = side_facts(default_left);
    let of = side_facts(!default_left);
    if clean(default_left) {
        if let Some(shape) = descent(default_left, tpa).shape {
            return pick(default_left, tpa, "default_side_clear", shape);
        }
    }
    if clean(!default_left) {
        if let Some(shape) = descent(!default_left, tpa).shape {
            let why = if df.airspace.is_some() {
                "switched_side_for_airspace"
            } else {
                "switched_side_for_terrain"
            };
            return pick(!default_left, tpa, why, shape);
        }
    }
    // Neither side is clean. Airspace can't be out-climbed at pattern heights
    // (R-164 runs to 30,000), so prefer the side WITHOUT a block, raised over
    // its terrain when needed; the other side, raised, is the second choice.
    let lower_left = {
        let (dr, or) = (raise(df), raise(of));
        if dr <= or + RAISE_SIDE_MARGIN_FT {
            default_left
        } else {
            !default_left
        }
    };
    let first = match (df.airspace.is_some(), of.airspace.is_some()) {
        (true, false) => !default_left,
        (false, true) => default_left,
        _ => lower_left,
    };
    let why_for = |left: bool| {
        let f = side_facts(left);
        if f.airspace.is_some() {
            "airspace_unavoidable"
        } else if raise(f) > tpa + 0.5 {
            "raised_for_terrain"
        } else if left != default_left {
            "switched_side_for_terrain"
        } else {
            "default_side_clear"
        }
    };
    for left in [first, !first] {
        let at = raise(side_facts(left));
        if let Some(shape) = descent(left, at).shape {
            return pick(left, at, why_for(left), shape);
        }
    }
    // Nothing legal clears the ground: never fly it.
    let side_word = |left: bool| if left { "left" } else { "right" };
    let problem = |left: bool| {
        descent(left, raise(side_facts(left)))
            .problem
            .unwrap_or_else(|| "its descent does not clear the ground".into())
    };
    let words = format!(
        "No circuit here clears the ground. On the {} side {}; on the {} side {}.",
        side_word(first),
        problem(first),
        side_word(!first),
        problem(!first)
    );
    PatternPick {
        refused: Some(words),
        ..pick(
            first,
            raise(side_facts(first)),
            "refused_for_terrain",
            DescentShape::default(),
        )
    }
}

/// Which pattern side keeps the downwind SOUTH (over the sea, around the Rock)
/// for a runway of the given true course — returns `left` (true = left-hand).
/// At Gibraltar south is the only flyable side (north is Spain); the runway
/// sets the turn: departing 09 (east) the sea is to the RIGHT → right-hand
/// (clockwise); departing 27 (west) it's to the LEFT → left-hand.
pub fn sea_side_is_left(course_deg: f64) -> bool {
    let right_normal = (course_deg + 90.0).rem_euclid(360.0);
    // Right of course points south (bearing 90..270) → fly right-hand (not left).
    !(90.0..270.0).contains(&right_normal)
}

/// A point in the runway frame (`a_m` along the takeoff course from the
/// departure threshold, `x_m` right of it) → lat/lon.
pub(super) fn frame_to_latlon(rwy: &crate::runway::Runway, a_m: f64, x_m: f64) -> (f64, f64) {
    let (lat, lon) =
        crate::nav::project_latlon(rwy.thr_lat, rwy.thr_lon, rwy.course_deg, a_m / M_PER_NM);
    crate::nav::project_latlon(
        lat,
        lon,
        (rwy.course_deg + 90.0).rem_euclid(360.0),
        x_m / M_PER_NM,
    )
}

/// The sampling polyline for one pattern side: points along upwind-extension,
/// crosswind, downwind, base, and final, in the runway frame. ~20 points —
/// inside the plugin probe channel's per-query cap.
pub(super) fn pattern_polyline(
    rwy: &crate::runway::Runway,
    g: &Geometry,
    runway_len_m: f64,
    left: bool,
) -> Vec<(f64, f64)> {
    let side = if left { -1.0 } else { 1.0 };
    let x_out = side * g.width_m;
    let a_up = runway_len_m + g.upwind_past_m;
    let a_dn = -g.downwind_past_m;
    let mut pts = Vec::with_capacity(20);
    // Crosswind line (includes the upwind end at x=0).
    for i in 0..4 {
        pts.push((a_up, x_out * i as f64 / 3.0));
    }
    // Downwind line.
    for i in 1..=8 {
        pts.push((a_up + (a_dn - a_up) * i as f64 / 8.0, x_out));
    }
    // Base line back to the centreline.
    for i in 1..=3 {
        pts.push((a_dn, x_out * (3 - i) as f64 / 3.0));
    }
    // Final, back toward the threshold.
    for i in 1..=4 {
        pts.push((a_dn + (0.0 - a_dn) * i as f64 / 4.0, 0.0));
    }
    densify(&pts, LEG_SAMPLE_M)
        .into_iter()
        .map(|(a, x)| frame_to_latlon(rwy, a, x))
        .collect()
}

/// One probe point of a pattern side, in the runway frame: the leg the
/// circuit flies there and whether it is on the drawn line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct FramePt {
    pub(super) a_m: f64,
    pub(super) x_m: f64,
    pub(super) leg: Leg,
    pub(super) line: bool,
}

/// The rectangle moved `d_m` outward (negative: inward) from its drawn line on
/// every side, sampled every `step_m` and tagged by leg: the crosswind
/// further along, the downwind further out, the base further back, and the
/// final longer.
pub(super) fn rectangle_offset(
    g: &Geometry,
    runway_len_m: f64,
    left: bool,
    d_m: f64,
    step_m: f64,
) -> Vec<FramePt> {
    let side = if left { -1.0 } else { 1.0 };
    let a_up = runway_len_m + g.upwind_past_m + d_m;
    let a_dn = -(g.downwind_past_m + d_m);
    let x_out = side * (g.width_m + d_m);
    let legs = [
        (Leg::Crosswind, (a_up, 0.0), (a_up, x_out)),
        (Leg::Downwind, (a_up, x_out), (a_dn, x_out)),
        (Leg::Base, (a_dn, x_out), (a_dn, 0.0)),
        (Leg::Final, (a_dn, 0.0), (0.0, 0.0)),
    ];
    let mut out = Vec::new();
    for (leg, from, to) in legs {
        for (a_m, x_m) in densify(&[from, to], step_m) {
            out.push(FramePt {
                a_m,
                x_m,
                leg,
                line: d_m == 0.0,
            });
        }
    }
    out
}

/// Every point probed for one pattern side: the drawn line every
/// [`LEG_SAMPLE_M`], and the corridor the autopilot actually flies
/// ([`CORRIDOR_OUT_RADII`], [`CORRIDOR_IN_RADII`]) every
/// [`CORRIDOR_SAMPLE_M`]. Pure.
pub(super) fn corridor_frame(g: &Geometry, runway_len_m: f64, left: bool) -> Vec<FramePt> {
    let r = g.lead_m / 0.9;
    let mut out = rectangle_offset(g, runway_len_m, left, 0.0, LEG_SAMPLE_M);
    for k in CORRIDOR_OUT_RADII {
        out.extend(rectangle_offset(
            g,
            runway_len_m,
            left,
            k * r,
            CORRIDOR_SAMPLE_M,
        ));
    }
    out.extend(rectangle_offset(
        g,
        runway_len_m,
        left,
        -CORRIDOR_IN_RADII * r,
        CORRIDOR_SAMPLE_M,
    ));
    out
}

/// Inside this of the threshold the descent is autoland's (the circuit hands
/// over further out, and the last mile's ground is the runway's own slope,
/// which the path's floor would read as a hill).
pub const DESCENT_CHECK_FROM_M: f64 = M_PER_NM;

/// The part of the descent a probe point is flown on, as [`leg_target_ft`]
/// flies it: `None` for the legs flown level (the crosswind, the downwind
/// until abeam the threshold); the downwind after it is the platform until
/// its led turn, where the BASE leg begins; the base's path is judged at
/// the roll-out distance or further ([`path_dist_m`]). Pure.
pub(super) fn descent_leg_at(p: &FramePt, g: &Geometry) -> Option<(DescentLeg, f64)> {
    match p.leg {
        Leg::Upwind | Leg::Crosswind => None,
        Leg::Downwind if p.a_m >= 0.0 => None,
        Leg::Downwind if p.a_m > -(g.downwind_past_m - g.lead_m) => {
            Some((DescentLeg::Platform, p.a_m.abs()))
        }
        Leg::Downwind | Leg::Base => Some((
            DescentLeg::Base,
            path_dist_m(Leg::Base, p.a_m, g.downwind_past_m),
        )),
        Leg::Final => Some((DescentLeg::Final, p.a_m.abs())),
    }
}

/// A side's terrain facts from its probe points and the ground under each
/// (feet MSL, same order): the drawn line's highest (the old number), the
/// highest under the legs flown level, corridor included, and the descent's
/// samples. Pure.
pub(super) fn corridor_facts(pts: &[FramePt], ground_ft: &[f64], g: &Geometry) -> SideFacts {
    let mut f = SideFacts::default();
    let max = |m: Option<f64>, v: f64| Some(m.map_or(v, |m: f64| m.max(v)));
    for (p, &e) in pts.iter().zip(ground_ft) {
        if p.line {
            f.line_max_ft = max(f.line_max_ft, e);
        }
        match descent_leg_at(p, g) {
            None => f.terrain_max_ft = max(f.terrain_max_ft, e),
            Some((leg, path_m)) => {
                let thr_m = p.a_m.hypot(p.x_m);
                if thr_m >= DESCENT_CHECK_FROM_M {
                    f.descent.push(GroundSample {
                        leg,
                        path_m,
                        thr_m,
                        ground_ft: e,
                    });
                }
            }
        }
    }
    f
}

/// Probe one pattern side's corridor ([`corridor_frame`]) through the plugin
/// and read its facts ([`corridor_facts`]); airspace is the caller's. Any
/// unanswered chunk is no data at all (`SideFacts::default()`), never a
/// partial picture read as clear ground.
pub(super) async fn probe_side(
    xp: &XPlaneClient,
    rwy: &crate::runway::Runway,
    g: &Geometry,
    left: bool,
) -> SideFacts {
    let pts = corridor_frame(g, rwy.length_m, left);
    let lls: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| frame_to_latlon(rwy, p.a_m, p.x_m))
        .collect();
    let mut ground = Vec::with_capacity(lls.len());
    for chunk in lls.chunks(crate::terrain::MAX_POINTS) {
        match crate::terrain::sample_ft(xp, chunk).await {
            Some(v) => ground.extend(v),
            None => return SideFacts::default(),
        }
    }
    corridor_facts(&pts, &ground, g)
}

/// The shortest `downwind_past_m` that still leaves autoland a final: the
/// base-to-final turn (`final_r_m`) plus [`FINAL_STRAIGHT_MIN_S`] of straight
/// final at the approach speed. Pure.
pub(super) fn shortest_downwind_past_m(final_r_m: f64, approach_kts: f64) -> f64 {
    (final_r_m + FINAL_STRAIGHT_MIN_S * approach_kts.max(40.0) * 0.514_444).max(2000.0)
}

/// The least straight final a shortened pattern may leave, seconds: the 737
/// handed over at LEMG 13 with 46 s of final flew it 30 to 50 m off and
/// landed off the pavement (sortie 1790591256-full_auto, 2026-09-28).
pub(super) const FINAL_STRAIGHT_MIN_S: f64 = 60.0;
/// Each shortening of the downwind tried when no side's descent clears.
pub(super) const DOWNWIND_SHORTEN_STEP_M: f64 = 500.0;

/// The shorter downwinds to try, longest first, never under `min_m`. For a
/// jet the drawn `2 r + 1,200 m` is already under it and there are none. Pure.
pub(super) fn shorter_downwinds(dn_m: f64, min_m: f64) -> Vec<f64> {
    let mut out = Vec::new();
    let mut d = dn_m - DOWNWIND_SHORTEN_STEP_M;
    while d >= min_m && out.len() < 4 {
        out.push(d);
        d -= DOWNWIND_SHORTEN_STEP_M;
    }
    out
}

/// Fill a frame polyline so no two consecutive samples are further apart
/// than `step_m`. The Rock's ridge is ~300 m wide and the rectangle's eight
/// downwind samples were 580 m apart: the probe reported 650 ft of terrain
/// under a side whose ridge stands at ~1,150 (2026-09-14).
pub fn densify(pts: &[(f64, f64)], step_m: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(pts.len() * 4);
    for (i, &(a, x)) in pts.iter().enumerate() {
        if i > 0 {
            let (pa, px) = pts[i - 1];
            let len = ((a - pa).powi(2) + (x - px).powi(2)).sqrt();
            let n = (len / step_m).ceil() as usize;
            for k in 1..n {
                let f = k as f64 / n as f64;
                out.push((pa + (a - pa) * f, px + (x - px) * f));
            }
        }
        out.push((a, x));
    }
    out
}
