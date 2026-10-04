//! "Put us enroute 30 miles before top of descent": a NEW FLIGHT started in
//! the air ON THE LOADED PLAN at the point under test, so a sortie that tests
//! the arrival does not fly the whole departure and cruise first (the
//! pilot's test wall time). The sim's own flight initialisation places the
//! aircraft (`lle_air_start`: position, true heading, altitude and speed,
//! engines running) through the same plugin mailbox and the same new-flight
//! path as the approach setup (`setup::run_init`, `retire_for_new_flight`);
//! the follow-up engages the autopilot on the leg's heading and the plan's
//! altitude (confirmed, `autoland::ApEngage`), arms the speed governor at
//! the plan's speed there, and hands the world an IFR flight mid-route: its
//! discrete code kept unit to unit, the nearest radar unit's station, COM1
//! on its approach frequency (`station::arm_new_flight_enroute`). Nothing
//! more: "fly the flight plan" then takes it through full auto's airborne
//! RESUME, as a pilot handing over mid-route would.
//!
//! The placement is pure ([`place`]): tested on the pilot's real OFP.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::setup::{InitOutcome, InitTiming};
use super::{Tool, ToolCtx};
use crate::voice::{Priority, Voice};

/// The point on the plan the new flight starts at.
#[derive(Debug, Clone, PartialEq)]
pub enum EnroutePoint {
    /// Over this fix of the plan, on the leg that leaves it.
    Fix(String),
    /// This many miles along the route before the plan's top of descent.
    BeforeTod(f64),
    /// This many miles along the route before the destination.
    BeforeDestination(f64),
}

impl EnroutePoint {
    /// The spoken point, for the lines and the runner's rows.
    pub fn words(&self, dest: &str) -> String {
        match self {
            EnroutePoint::Fix(f) => format!("over {f}"),
            EnroutePoint::BeforeTod(n) => format!("{n:.0} miles before the top of descent"),
            EnroutePoint::BeforeDestination(n) => format!("{n:.0} miles before {dest}"),
        }
    }
}

/// The furthest a point may be before the top of descent or the
/// destination, miles along the route.
pub const MAX_POINT_NM: f64 = 1_000.0;
/// The plan's altitude at the point must be at least this: the resume needs
/// a thousand feet above the ground, and the plan knows nothing of it.
pub const MIN_START_FT: f64 = 3_000.0;
/// Below this the 250 kt airspace limit is the most a start is given.
const BELOW_10K_FT: f64 = 10_000.0;
const BELOW_10K_KTS: f64 = 250.0;
/// A schedule entry is the fix's when within this, nm (an ident is not
/// unique across the world, a position is).
const SCHEDULE_MATCH_NM: f64 = 2.0;
const KTS_TO_MPS: f64 = 0.514_444;
const FT_TO_M: f64 = 0.3048;

/// The airframe's speeds the placement may use.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Speeds {
    /// The profile's cruise (`reference.cruise_ias_kts`): the speed where the
    /// OFP planned none.
    pub profile_kts: Option<f64>,
    /// The airframe's floor (1.2 Vs clean) and ceiling (the lower of Vne
    /// less 15 and the cruise cap under the overspeed caution).
    pub floor_kts: Option<f64>,
    pub ceiling_kts: Option<f64>,
    /// The higher Vmo above an altitude (`aircraft::VmoHigh`): placed at or
    /// over it, the ceiling is that limit's, not `ceiling_kts` (the low one).
    pub vmo_high: Option<crate::aircraft::VmoHigh>,
}

/// Where and how the new flight starts.
#[derive(Debug, Clone, PartialEq)]
pub struct Placement {
    pub lat: f64,
    pub lon: f64,
    /// The leg's track from the point to the next fix, degrees true: the
    /// heading the sim starts on and the autopilot holds.
    pub track_true: f64,
    /// The plan's altitude there, feet (the navlog's profile, the descent
    /// from the top of descent included).
    pub alt_ft: f64,
    /// The indicated speed held from the start, and the true speed the sim
    /// is started at (ISA).
    pub ias_kts: f64,
    pub tas_kts: f64,
    /// "ofp" (the navlog's IAS/Mach at the next fix), "profile".
    pub speed_from: &'static str,
    /// The leg: the fix behind and the fix ahead.
    pub leg_from: String,
    pub next_fix: String,
    /// Miles along the route to the destination, and to the top of descent
    /// (negative past it; `None`: the plan has none).
    pub to_dest_nm: f64,
    pub to_tod_nm: Option<f64>,
}

/// The plan's route as flown: each waypoint with its miles from the origin.
fn cumulative(plan: &crate::simbrief::FlightPlan) -> Vec<f64> {
    let w = &plan.waypoints;
    let mut cum = Vec::with_capacity(w.len());
    let mut d = 0.0;
    for (i, p) in w.iter().enumerate() {
        if i > 0 {
            d += crate::nav::haversine_nm(w[i - 1].lat, w[i - 1].lon, p.lat, p.lon);
        }
        cum.push(d);
    }
    cum
}

/// The navlog's top of descent as miles along the route: its position
/// projected onto the nearest leg. `None` without one on the navlog.
fn tod_along(plan: &crate::simbrief::FlightPlan, cum: &[f64]) -> Option<f64> {
    let (lat, lon) = plan.schedule.tod?;
    let w = &plan.waypoints;
    let scale = lat.to_radians().cos();
    (1..w.len())
        .filter_map(|i| {
            let (a, b) = (&w[i - 1], &w[i]);
            let (ax, ay) = ((a.lon - lon) * 60.0 * scale, (a.lat - lat) * 60.0);
            let (bx, by) = ((b.lon - lon) * 60.0 * scale, (b.lat - lat) * 60.0);
            let (dx, dy) = (bx - ax, by - ay);
            let len2 = dx * dx + dy * dy;
            if len2 < 1e-6 {
                return None;
            }
            let u = (-(ax * dx + ay * dy) / len2).clamp(0.0, 1.0);
            let off = (ax + u * dx).hypot(ay + u * dy);
            Some((off, cum[i - 1] + u * (cum[i] - cum[i - 1])))
        })
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, along)| along)
}

/// The plan's altitude `along` miles from the origin: its waypoints'
/// planned altitudes and the top of descent at cruise, straight lines
/// between. Pure.
fn profile_alt(
    plan: &crate::simbrief::FlightPlan,
    cum: &[f64],
    tod: Option<f64>,
    along: f64,
) -> f64 {
    let mut nodes: Vec<(f64, f64)> =
        cum.iter().zip(&plan.waypoints).map(|(c, w)| (*c, w.alt)).collect();
    if let Some(t) = tod {
        nodes.push((t, plan.cruise_alt_ft));
    }
    nodes.sort_by(|a, b| a.0.total_cmp(&b.0));
    for pair in nodes.windows(2) {
        let ((c0, a0), (c1, a1)) = (pair[0], pair[1]);
        if along >= c0 && along <= c1 {
            let f = if c1 > c0 { (along - c0) / (c1 - c0) } else { 0.0 };
            return a0 + f * (a1 - a0);
        }
    }
    nodes.last().map_or(plan.cruise_alt_ft, |n| n.1)
}

/// True airspeed (ISA) of an indicated airspeed at a pressure altitude, kt:
/// the Mach it is there, times the speed of sound there. Pure.
pub fn tas_kts(ias_kts: f64, alt_ft: f64) -> f64 {
    let (mut lo, mut hi) = (0.0_f64, 1.5_f64);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if crate::procedure::cas_for_mach(mid, alt_ft) < ias_kts {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let h = alt_ft.clamp(-2_000.0, 65_000.0);
    let t = if h <= 36_089.0 { 288.15 - 0.001_981_2 * h } else { 216.65 };
    0.5 * (lo + hi) * 661.4786 * (t / 288.15).sqrt()
}

/// The speed to start and hold at: the OFP's airspeed and Mach planned at
/// the fix ahead (the lower, at the level), else the profile's cruise; at
/// most 250 below ten thousand; inside the airframe's floor and ceiling.
fn start_speed(
    plan: &crate::simbrief::FlightPlan,
    next: &crate::simbrief::Waypoint,
    alt_ft: f64,
    speeds: &Speeds,
) -> Option<(f64, &'static str)> {
    let planned = plan.schedule.fixes.iter().find(|e| {
        e.ident.eq_ignore_ascii_case(&next.ident)
            && crate::nav::haversine_nm(e.lat, e.lon, next.lat, next.lon) <= SCHEDULE_MATCH_NM
    });
    // The Mach limits only on a CRUISE fix (the schedule's own rule,
    // `procedure::scheduled_cruise_kts`): a climb or descent fix's Mach is
    // its planned airspeed at its own level (HON at 11,900 heading for
    // OSKOT's M0.41 at 5,700 would be 218, not the planned 250).
    let ofp = planned.and_then(|e| {
        let by_mach = e
            .mach
            .filter(|_| e.stage == crate::simbrief::Stage::Cruise)
            .map(|m| crate::procedure::cas_for_mach(m, alt_ft));
        match (e.ias_kts, by_mach) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    });
    let (mut kts, from) = match (ofp, speeds.profile_kts) {
        (Some(k), _) => (k, "ofp"),
        (None, Some(k)) => (k, "profile"),
        (None, None) => return None,
    };
    if alt_ft < BELOW_10K_FT {
        kts = kts.min(BELOW_10K_KTS);
    }
    let ceiling = crate::aircraft::vne_at(None, speeds.vmo_high, alt_ft)
        .map(crate::aircraft::ceiling_from_vne)
        .or(speeds.ceiling_kts);
    if let Some(c) = ceiling {
        kts = kts.min(c);
    }
    if let Some(f) = speeds.floor_kts {
        kts = kts.max(f);
    }
    Some((kts.round(), from))
}

/// Place the new flight at `point` on the plan. Refused, with the reason in
/// words: a fix the plan does not hold (or its ends), a plan with no top of
/// descent for a point before it, a point before the plan's first fix or at
/// or past its destination, a plan too low there, a point with no speed.
/// Pure.
pub fn place(
    plan: &crate::simbrief::FlightPlan,
    point: &EnroutePoint,
    speeds: &Speeds,
) -> std::result::Result<Placement, String> {
    let w = &plan.waypoints;
    if w.len() < 3 {
        return Err("The plan has no route between its fields.".into());
    }
    let cum = cumulative(plan);
    let total = *cum.last().unwrap_or(&0.0);
    let tod = tod_along(plan, &cum);
    let dest = plan.destination.clone();
    // The miles along the route, and the leg (index of the fix ahead).
    let (along, ahead) = match point {
        EnroutePoint::Fix(f) => {
            let i = w
                .iter()
                .position(|p| p.ident.eq_ignore_ascii_case(f.trim()))
                .ok_or_else(|| format!("{} is not on the plan.", f.trim().to_ascii_uppercase()))?;
            if i == 0 || i + 1 >= w.len() {
                return Err(format!(
                    "{} is an end of the plan, not a point on its route.",
                    w[i].ident
                ));
            }
            (cum[i], i + 1)
        }
        EnroutePoint::BeforeTod(n) | EnroutePoint::BeforeDestination(n) => {
            if !n.is_finite() || *n < 0.0 || *n > MAX_POINT_NM {
                return Err(format!("A point 0 to {MAX_POINT_NM:.0} miles before it, please."));
            }
            let along = match point {
                EnroutePoint::BeforeTod(_) => {
                    tod.ok_or("The plan has no top of descent on its navlog.")? - n
                }
                _ => total - n,
            };
            if along <= cum[1] {
                return Err(format!(
                    "That is before the plan's first fix, {}: {:.0} miles of route before it.",
                    w[1].ident,
                    cum[1] - along
                ));
            }
            if along >= total {
                return Err(format!("That is at or past {dest}."));
            }
            let ahead = (1..w.len()).find(|i| cum[*i] > along).unwrap_or(w.len() - 1);
            (along, ahead)
        }
    };
    let (a, b) = (&w[ahead - 1], &w[ahead]);
    let seg = cum[ahead] - cum[ahead - 1];
    let into = (along - cum[ahead - 1]).max(0.0);
    let (lat, lon) = if into <= 1e-9 || seg <= 1e-9 {
        (a.lat, a.lon)
    } else {
        let brg = crate::nav::bearing_deg(a.lat, a.lon, b.lat, b.lon);
        crate::nav::project_latlon(a.lat, a.lon, brg, into)
    };
    let track_true = crate::nav::bearing_deg(lat, lon, b.lat, b.lon);
    let alt_ft = profile_alt(plan, &cum, tod, along).round();
    if !(alt_ft >= MIN_START_FT) {
        return Err(format!(
            "The plan is at {alt_ft:.0} feet there, under the {MIN_START_FT:.0} a start in \
             the air needs."
        ));
    }
    let (ias_kts, speed_from) = start_speed(plan, b, alt_ft, speeds)
        .ok_or("Neither the plan nor this aircraft's profile has a speed for that point.")?;
    Ok(Placement {
        lat,
        lon,
        track_true,
        alt_ft,
        ias_kts,
        tas_kts: tas_kts(ias_kts, alt_ft),
        speed_from,
        leg_from: a.ident.clone(),
        next_fix: b.ident.clone(),
        to_dest_nm: total - along,
        to_tod_nm: tod.map(|t| t - along),
    })
}

/// The flight-initialisation JSON for a start in the air: the loaded
/// aircraft at the placement (the SDK's `lle_air_start`: MSL metres, true
/// heading, metres per second), engines running. Pure.
pub fn enroute_init_json(acf_rel: &str, p: &Placement) -> String {
    json!({
        "aircraft": { "path": acf_rel },
        "lle_air_start": {
            "latitude": p.lat,
            "longitude": p.lon,
            "elevation_in_meters": (p.alt_ft * FT_TO_M * 10.0).round() / 10.0,
            "heading_true": (p.track_true * 10.0).round() / 10.0,
            "speed_in_meters_per_second": (p.tas_kts * KTS_TO_MPS * 10.0).round() / 10.0,
        },
        "engine_status": { "all_engines": { "running": true } },
    })
    .to_string()
}

/// The typed words for the setup (the grammar's `setup_enroute`, pinned by
/// a test against the real parser).
pub fn enroute_line(point: &EnroutePoint) -> String {
    match point {
        EnroutePoint::Fix(f) => format!("put us enroute at {}", f.to_ascii_uppercase()),
        EnroutePoint::BeforeTod(n) => format!("put us enroute {n:.0} miles before top of descent"),
        EnroutePoint::BeforeDestination(n) => {
            format!("put us enroute {n:.0} miles before the destination")
        }
    }
}

/// The tool's arguments as the grammar writes them.
pub fn point_from_args(args: &Value) -> std::result::Result<EnroutePoint, String> {
    let nm = || args.get("nm").and_then(Value::as_f64).ok_or("How many miles?".to_string());
    match args.get("at").and_then(Value::as_str) {
        Some("fix") => args
            .get("fix")
            .and_then(Value::as_str)
            .filter(|f| !f.trim().is_empty())
            .map(|f| EnroutePoint::Fix(f.trim().to_ascii_uppercase()))
            .ok_or_else(|| "Which fix?".to_string()),
        Some("before_tod") => Ok(EnroutePoint::BeforeTod(nm()?)),
        Some("before_destination") => Ok(EnroutePoint::BeforeDestination(nm()?)),
        _ => Err("Where on the plan? A fix, miles before top of descent, or miles before \
                  the destination."
            .into()),
    }
}

pub struct SetUpEnroute {
    /// The autopilot the new flight is held on; `None` without one (refused:
    /// nobody could hold it until the resume).
    pub ap: Option<crate::tools::autopilot::ApConfig>,
    pub cruise_ias_kts: Option<f64>,
    pub vne_kts: Option<f64>,
    /// The airframe's transition altitude, where the plan gives none.
    pub transition_altitude_ft: f64,
}

/// The new flight is READY only once settled (#177): within this of the
/// dial, feet...
pub const SETTLE_ALT_FT: f64 = 150.0;
/// ...for this long, seconds...
pub const SETTLE_LEVEL_S: f64 = 10.0;
/// ...with the governor holding the speed within this, knots, and the
/// altimeter on the setting owed there...
pub const SETTLE_IAS_KTS: f64 = 10.0;
/// ...or, after this long, what did not settle is said and put on file.
/// The placement zooms a few hundred feet and the 737's VS mode answers the
/// wheel in about 20 s: 60 s left EGKK-EGBB 149 ft off the dial, healthy and
/// closing, called unsettled (2026-09-29, #177).
pub const SETTLE_MAX_S: f64 = 120.0;
/// The VS wheel toward the dial: fpm per foot off, and its bounds.
const SETTLE_VS_PER_FT: f64 = 2.0;
const SETTLE_VS_MIN_FPM: f64 = 300.0;
const SETTLE_VS_MAX_FPM: f64 = 1_500.0;
/// Inside this of the dial the wheel is zero, feet.
const SETTLE_VS_ZERO_FT: f64 = 50.0;

/// One look at the new flight while it settles.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SettleSample {
    /// Seconds since the settle began.
    pub t_s: f64,
    pub alt_ft: f64,
    pub ias_kts: f64,
    /// The governor's target; `None`: no governor holds the speed.
    pub governed_kts: Option<f64>,
    /// The altimeter reads the setting owed; `None`: unread.
    pub altimeter_ok: Option<bool>,
}

/// What the settle came to.
#[derive(Debug, Clone, PartialEq)]
pub enum Settle {
    Settling,
    Settled,
    /// Out of time: what did not settle, each with its number.
    Unsettled(Vec<String>),
}

/// Judges [`SettleSample`]s against the dial: level within
/// [`SETTLE_ALT_FT`] for [`SETTLE_LEVEL_S`], speed governed within
/// [`SETTLE_IAS_KTS`], altimeter set; bounded by [`SETTLE_MAX_S`].
#[derive(Debug, Default)]
pub struct SettleJudge {
    level_since: Option<f64>,
}

impl SettleJudge {
    pub fn step(&mut self, dial_ft: f64, s: &SettleSample) -> Settle {
        let off = s.alt_ft - dial_ft;
        self.level_since = match off.abs() <= SETTLE_ALT_FT {
            true => self.level_since.or(Some(s.t_s)),
            false => None,
        };
        let level_s = self.level_since.map_or(0.0, |t| s.t_s - t);
        let speed_off = s.governed_kts.map(|g| s.ias_kts - g);
        let speed = speed_off.is_some_and(|o| o.abs() <= SETTLE_IAS_KTS);
        if level_s >= SETTLE_LEVEL_S && speed && s.altimeter_ok == Some(true) {
            return Settle::Settled;
        }
        if s.t_s < SETTLE_MAX_S {
            return Settle::Settling;
        }
        let mut why = Vec::new();
        if off.abs() > SETTLE_ALT_FT {
            why.push(format!("{:.0} feet off the {:.0} dial", off.abs(), dial_ft));
        } else if level_s < SETTLE_LEVEL_S {
            why.push(format!("level only {level_s:.0} seconds"));
        }
        match (s.governed_kts, speed) {
            (None, _) => why.push("no speed hold".to_string()),
            (Some(g), false) => why.push(format!("{:.0} knots against {g:.0}", s.ias_kts)),
            _ => {}
        }
        match s.altimeter_ok {
            Some(true) => {}
            Some(false) => why.push("the altimeter is not set".to_string()),
            None => why.push("the altimeter is unread".to_string()),
        }
        Settle::Unsettled(why)
    }
}

/// The line (and the result's `why`) for a flight that did not settle.
pub fn unsettled_line(why: &[String]) -> String {
    if why.iter().any(|w| w == NO_PITCH_WHY) {
        return format!("{NO_PITCH_WHY}. I'm flying the pitch until you say my controls.");
    }
    format!(
        "it has not settled after {SETTLE_MAX_S:.0} seconds: {}. The autopilot has it; I \
         haven't called it ready.",
        why.join(", ")
    )
}

/// The settle's reason when the autopilot's pitch moves nothing (#181).
pub const NO_PITCH_WHY: &str = "the autopilot has no pitch control";

/// The vertical mode while the new flight settles.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Vertical {
    /// The autopilot's altitude hold has the dial: leave it.
    Leave,
    /// VS mode at this rate (fpm, the wheel's 100 fpm clicks) toward the
    /// dial, the dial armed for the capture.
    Vs(f64),
}

/// Toward the dial by VS, the follower's way, never by an ALT HOLD engaged
/// off it: after the air start the 737's hold, engaged in the placement's
/// zoom at 4,450 fpm, flew a phugoid from 18,000 to 26,200 ft for three
/// minutes (#177, 2026-09-29); the same zoom in the first air start stopped
/// within 5 s of the follower's VS (1790635335). A hold at the dial, or
/// closing on it inside [`crate::procedure::ALT_HOLD_LEAVE_FT`], is the
/// capture's own and is left. Pure.
pub fn vertical_toward_dial(
    alt_ft: f64,
    dial_ft: f64,
    rate_fpm: f64,
    alt_hold_on: bool,
) -> Vertical {
    let off = dial_ft - alt_ft;
    let closing = off * rate_fpm > 0.0 && off.abs() <= crate::procedure::ALT_HOLD_LEAVE_FT;
    if alt_hold_on && (off.abs() <= SETTLE_ALT_FT || closing) {
        return Vertical::Leave;
    }
    if off.abs() <= SETTLE_VS_ZERO_FT {
        return Vertical::Vs(0.0);
    }
    let rate = (off.abs() * SETTLE_VS_PER_FT).clamp(SETTLE_VS_MIN_FPM, SETTLE_VS_MAX_FPM);
    Vertical::Vs(crate::procedure::vs_dial_fpm(rate.copysign(off)))
}

/// Say a refusal and put it on file, so the sortie runner can name it.
fn refused(line: String) -> Result<String> {
    crate::blackbox::record("setup", json!({ "event": "enroute_refused", "why": line }));
    Ok(line)
}

#[async_trait]
impl Tool for SetUpEnroute {
    fn name(&self) -> &str {
        "setup_enroute"
    }
    fn description(&self) -> &str {
        "Start a new flight in the air on the loaded flight plan, at a fix or a number of \
         miles before the top of descent or the destination, engines running, the autopilot \
         holding the leg and the plan's altitude; then 'fly the flight plan' resumes it. For \
         'put us enroute 30 miles before top of descent', 'put us enroute at WELIN'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "at": {"type": "string", "enum": ["fix", "before_tod", "before_destination"]},
                "fix": {"type": "string"},
                "nm": {"type": "number"}
            },
            "required": ["at"],
            "additionalProperties": false
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let point = match point_from_args(args) {
            Ok(p) => p,
            Err(line) => return refused(line),
        };
        if ctx.xplane.get_float(crate::datarefs::IS_IN_REPLAY).await.is_ok_and(|v| v > 0.5) {
            return refused("We're in replay. Go back to live and ask again.".into());
        }
        if let Some(who) = super::setup::flying_now() {
            return refused(format!("{who}. Say my controls first, then ask again."));
        }
        let Some(ap) = self.ap.clone() else {
            return refused("I have no autopilot to hold a start in the air with.".into());
        };
        // The plan the resume will fly: the shelf, fetched fresh when there
        // is a SimBrief id (the pilot regenerates and says the same words).
        if crate::simbrief::local_plan().is_some() {
            return refused(
                "A local plan is re-planned after an interruption, so it can't be joined in the \
                 air. Load a SimBrief plan."
                    .into(),
            );
        }
        let id = ctx.config.simbrief_id.trim().to_string();
        let plan = match id.is_empty() {
            true => crate::simbrief::loaded_plan(),
            false => match crate::simbrief::fetch(&id).await {
                Ok(p) => {
                    crate::simbrief::publish_plan(&p);
                    Some(p)
                }
                Err(e) => {
                    tracing::warn!("simbrief fetch failed: {e:#}");
                    crate::simbrief::loaded_plan()
                }
            },
        };
        let Some(plan) = plan else {
            return refused("No flight plan to put us on. Generate one first.".into());
        };
        let strings = crate::xplane::XPlaneClient::new(ctx.config.xplane_base_url.clone());
        let (acf, icao) = tokio::join!(
            strings.get_string("sim/aircraft/view/acf_relative_path"),
            strings.get_string("sim/aircraft/view/acf_ICAO"),
        );
        let Some(acf) = acf.ok().filter(|p| !p.trim().is_empty()) else {
            return refused("I can't read which aircraft is loaded.".into());
        };
        let icao = icao.unwrap_or_default();
        if !icao.trim().eq_ignore_ascii_case(plan.aircraft.trim()) {
            return refused(format!(
                "The plan is for a {}, and this is a {}. Load its aircraft first.",
                plan.aircraft,
                icao.trim()
            ));
        }
        let ceiling = [
            crate::aircraft::speed_ceiling_kts(),
            crate::procedure::cruise_cap_kts(self.vne_kts),
        ]
        .into_iter()
        .flatten()
        .reduce(f64::min);
        let speeds = Speeds {
            profile_kts: self.cruise_ias_kts,
            floor_kts: crate::aircraft::speed_floor_kts(),
            ceiling_kts: ceiling,
            vmo_high: crate::aircraft::published_vmo_high(),
        };
        let p = match place(&plan, &point, &speeds) {
            Ok(p) => p,
            Err(line) => return refused(line),
        };
        let body = enroute_init_json(&acf, &p);
        let words = point.words(&plan.destination);
        crate::blackbox::record(
            "setup",
            json!({ "event": "enroute_init", "acf": acf, "point": words,
                    "origin": plan.origin, "dest": plan.destination,
                    "lat": p.lat, "lon": p.lon, "track_true": p.track_true, "alt_ft": p.alt_ft,
                    "ias_kts": p.ias_kts, "tas_kts": p.tas_kts.round(),
                    "speed_from": p.speed_from, "leg_from": p.leg_from, "next_fix": p.next_fix,
                    "to_dest_nm": p.to_dest_nm, "to_tod_nm": p.to_tod_nm, "json": body }),
        );
        // The new flight ends this one: nothing may keep flying it.
        crate::flight_state::stand_down();
        // And the crew forgets it (last landing, tracking, destination)
        // before the new flight's transition is published below.
        crate::flight_state::forget_flight();
        // The flight's transition, as the resume publishes it: the altimeter
        // is set to it at the settle and the monitor checks against it.
        crate::tools::instruments::publish_flight_transition(Some(
            crate::tools::instruments::FlightTransition {
                altitude_ft: plan.transition_altitude_ft,
                level_ft: plan.transition_level_ft,
                descending: false,
            },
        ));
        let job = EnrouteJob {
            base: ctx.config.xplane_base_url.clone(),
            voice: ctx.voice.cloned(),
            body,
            placement: p.clone(),
            words: words.clone(),
            ap,
            transition_ft: self.transition_altitude_ft,
        };
        let loading = super::setup::Loading::begin();
        super::setup::spawn_off_loop(enroute_follow_up(job, loading));
        Ok(format!(
            "Setting up {words}, at {} and {:.0} knots.",
            crate::flightplan::spoken_feet(p.alt_ft),
            p.ias_kts
        ))
    }
}

struct EnrouteJob {
    base: String,
    voice: Option<Voice>,
    body: String,
    placement: Placement,
    words: String,
    ap: crate::tools::autopilot::ApConfig,
    /// The airframe's transition altitude, where the plan gives none.
    transition_ft: f64,
}

const CMD_HDG_MODE: &str = "sim/autopilot/heading";
const CMD_VS_MODE: &str = "sim/autopilot/vertical_speed";
const VS_STATUS: &str = "sim/cockpit2/autopilot/vvi_status";
const VS_DIAL: &str = "sim/cockpit/autopilot/vertical_velocity";
const HDG_BUG: &str = "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot";
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
const ALT_HOLD_STATUS: &str = "sim/cockpit2/autopilot/altitude_hold_status";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";

/// How the autopilot came up on the new flight.
#[derive(Debug, Clone, Copy, Default)]
struct Engaged {
    servos: bool,
    tries: u32,
    heading: bool,
    /// VS mode, or the capture's altitude hold, reads engaged.
    vertical: bool,
}

/// The heading bug and the altitude dial to the placement, then the servos
/// pressed until they READ on with the sim live (`ApEngage`, as autoland
/// engages after its setup), then heading and VS toward the dial, each read
/// back. At most `ENGAGE_GIVE_UP_S` of live flight.
async fn engage(xp: &crate::xplane::XPlaneClient, job: &EnrouteJob) -> Engaged {
    use crate::ap_engage::ApEngage;
    let p = &job.placement;
    let var = xp.get_float(MAG_VAR).await.ok().filter(|v| v.is_finite()).unwrap_or(0.0);
    // X-Plane's variation is positive-west: magnetic = true + variation.
    let bug = crate::autoland::true_to_mag(p.track_true, var);
    let mut out = Engaged::default();
    let mut e = ApEngage::default();
    let t0 = std::time::Instant::now();
    let mut step = e.arm(0.0);
    // The wait before live flight is bounded too: a sim that never flies.
    while t0.elapsed().as_secs() < 60 {
        if step.fire {
            let _ = xp.set_float(HDG_BUG, bug).await;
            let _ = xp.set_float(ALT_DIAL, p.alt_ft).await;
            let sent = crate::ap_engage::press(xp, &job.ap.fd_commands).await;
            crate::blackbox::record(
                "setup",
                json!({ "event": "enroute_ap_engage", "try": step.try_n, "sent": sent }),
            );
        }
        if step.confirmed_now {
            out.servos = true;
            break;
        }
        if step.give_up {
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        let (servos, ias) = tokio::join!(
            xp.get_float(crate::datarefs::SERVOS_ON),
            xp.get_float(crate::datarefs::AIRSPEED_KTS),
        );
        let ias = ias.ok().filter(|v| v.is_finite()).unwrap_or(0.0);
        step = e.step(t0.elapsed().as_secs_f64(), servos.ok().filter(|v| v.is_finite()), ias);
    }
    out.tries = e.tries();
    if !out.servos {
        return out;
    }
    let _ = xp.set_float(HDG_BUG, bug).await;
    if xp.get_float(crate::datarefs::HEADING_STATUS).await.is_ok_and(|v| v <= 0.5) {
        let _ = xp.run_command(CMD_HDG_MODE).await;
    }
    let _ = xp.set_float(ALT_DIAL, p.alt_ft).await;
    // The vertical by VS toward the dial, never ALT HOLD off the zoom
    // (`vertical_toward_dial`); the settle keeps the wheel on it.
    let alt = xp.get_float(crate::datarefs::ALTITUDE_FT).await.unwrap_or(p.alt_ft);
    if let Vertical::Vs(rate) = vertical_toward_dial(alt, p.alt_ft, 0.0, false) {
        if xp.get_float(VS_STATUS).await.is_ok_and(|v| v <= 0.5) {
            let _ = xp.run_command(CMD_VS_MODE).await;
        }
        let _ = xp.set_float(VS_DIAL, rate).await;
    }
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    let (hdg, vs, alt) = tokio::join!(
        xp.get_float(crate::datarefs::HEADING_STATUS),
        xp.get_float(VS_STATUS),
        xp.get_float(ALT_HOLD_STATUS),
    );
    out.heading = hdg.is_ok_and(|v| v > 0.5);
    out.vertical = vs.is_ok_and(|v| v > 0.5) || alt.is_ok_and(|v| v > 1.5);
    out
}

/// The dial the new flight settles on, the governor's speed and the
/// altimeter: the vertical kept toward the dial by VS until the capture has
/// it ([`vertical_toward_dial`]), judged by [`SettleJudge`] every half
/// second, at most [`SETTLE_MAX_S`]. Each change of the vertical is
/// blackboxed (`enroute_vertical`), the end too (`enroute_settle`).
async fn settle(
    xp: &crate::xplane::XPlaneClient,
    base: &str,
    dial_ft: f64,
    altimeter_ok: Option<bool>,
) -> Settle {
    let t0 = std::time::Instant::now();
    let mut judge = SettleJudge::default();
    let mut last: Option<Vertical> = None;
    let mut last_sample = None;
    // The VS the settle flies is an engage like any other: its pitch
    // authority is checked from the first tick that flies it (#181).
    let mut authority_armed = false;
    loop {
        let (alt, ias, vvi, vs_st, hold_st) = tokio::join!(
            xp.get_float(crate::datarefs::ALTITUDE_FT),
            xp.get_float(crate::datarefs::AIRSPEED_KTS),
            xp.get_float(crate::datarefs::VVI_FPM),
            xp.get_float(VS_STATUS),
            xp.get_float(ALT_HOLD_STATUS),
        );
        let t_s = t0.elapsed().as_secs_f64();
        // A failed read is no measurement: the tick is skipped, the clock
        // still runs.
        if let Some([alt, ias, vvi]) = crate::control::measurements("setup", [alt, ias, vvi]) {
            let hold_on = hold_st.is_ok_and(|v| v > 1.5);
            let want = vertical_toward_dial(alt, dial_ft, vvi, hold_on);
            if let Vertical::Vs(rate) = want {
                let vs_off = vs_st.is_ok_and(|v| v <= 0.5);
                if vs_off {
                    let _ = xp.run_command(CMD_VS_MODE).await;
                }
                if last != Some(want) || vs_off {
                    let _ = xp.set_float(VS_DIAL, rate).await;
                }
            }
            let dial_fpm = match want {
                Vertical::Vs(rate) => Some(rate),
                _ => None,
            };
            if !authority_armed {
                if let Some(rate) = dial_fpm {
                    crate::climb_watch::arm_authority("setup_enroute", vvi, rate);
                    authority_armed = true;
                }
            } else if let Some(crate::climb_watch::Authority::NoAuthority(why)) =
                crate::climb_watch::feed_authority(Some(vvi), dial_fpm)
            {
                // Hand flown to the placement's level, never given back to
                // an autopilot with no pitch.
                crate::tools::speedhold::ACTIVE.store(false, std::sync::atomic::Ordering::SeqCst);
                let started = crate::tools::recover::take_over(
                    base.to_string(),
                    None,
                    crate::tools::recover::TakeOver {
                        why: format!("setup_enroute_{why}"),
                        min_safe_ias: None,
                        hold_kts: None,
                        ap_no_pitch: true,
                        hold_ft: Some(dial_ft),
                    },
                );
                crate::blackbox::record(
                    "setup",
                    json!({ "event": "enroute_ap_no_pitch", "why": why, "alt_ft": alt,
                            "dial": dial_ft, "recovery_started": started, "t_s": t_s }),
                );
                let verdict = Settle::Unsettled(vec![NO_PITCH_WHY.to_string()]);
                record_settle(&verdict, dial_ft, last_sample.as_ref());
                return verdict;
            }
            if last != Some(want) {
                crate::blackbox::record(
                    "setup",
                    json!({ "event": "enroute_vertical", "action": format!("{want:?}"),
                            "alt_ft": alt, "dial": dial_ft, "vvi": vvi, "hold": hold_on,
                            "t_s": t_s }),
                );
                last = Some(want);
            }
            let sample = SettleSample {
                t_s,
                alt_ft: alt,
                ias_kts: ias,
                governed_kts: crate::tools::speedhold::flight_target(),
                altimeter_ok,
            };
            last_sample = Some(sample);
            let verdict = judge.step(dial_ft, &sample);
            if verdict != Settle::Settling {
                record_settle(&verdict, dial_ft, Some(&sample));
                return verdict;
            }
        } else if t_s >= SETTLE_MAX_S {
            let verdict = Settle::Unsettled(vec!["the sim did not answer".to_string()]);
            record_settle(&verdict, dial_ft, last_sample.as_ref());
            return verdict;
        }
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    }
}

fn record_settle(verdict: &Settle, dial_ft: f64, s: Option<&SettleSample>) {
    let unsettled = match verdict {
        Settle::Unsettled(why) => why.clone(),
        _ => Vec::new(),
    };
    crate::blackbox::record(
        "setup",
        json!({ "event": "enroute_settle", "settled": *verdict == Settle::Settled,
                "unsettled": unsettled, "dial": dial_ft, "t_s": s.map(|s| s.t_s),
                "alt_ft": s.map(|s| s.alt_ft), "ias_kts": s.map(|s| s.ias_kts),
                "governed_kts": s.and_then(|s| s.governed_kts),
                "altimeter_ok": s.and_then(|s| s.altimeter_ok) }),
    );
}

/// Gear up and flaps in, each only when it reads otherwise, read back.
async fn clean_up(xp: &crate::xplane::XPlaneClient) -> (Option<bool>, Option<bool>) {
    let read = |name: &'static str| async move { xp.get_float(name).await.ok() };
    if read(crate::datarefs::GEAR_HANDLE_DOWN).await.is_some_and(|g| g > 0.5) {
        let _ = xp.set_float(crate::datarefs::GEAR_HANDLE_DOWN, 0.0).await;
    }
    if read(crate::datarefs::FLAP_RATIO).await.is_some_and(|f| f > 0.01) {
        let _ = xp.set_float(crate::datarefs::FLAP_RATIO, 0.0).await;
    }
    tokio::time::sleep(std::time::Duration::from_millis(300)).await;
    (
        read(crate::datarefs::GEAR_HANDLE_DOWN).await.map(|g| g < 0.5),
        read(crate::datarefs::FLAP_RATIO).await.map(|f| f <= 0.01),
    )
}

async fn enroute_follow_up(job: EnrouteJob, loading: super::setup::Loading) {
    let xp = crate::xplane::XPlaneClient::new(job.base.clone());
    let say = |line: &str| {
        if let Some(v) = &job.voice {
            v.alert(Priority::Caution, line.to_string());
        }
    };
    let t0 = std::time::Instant::now();
    let run = super::setup::run_init(&xp, job.body.as_bytes(), InitTiming::LIVE, &|w| {
        crate::blackbox::record(
            "setup",
            json!({ "event": "enroute_wait", "seq": w.seq, "result": w.result, "seq0": w.seq0,
                    "read_errors": w.read_errors, "polls": w.polls,
                    "ms": t0.elapsed().as_millis() as u64 }),
        );
    })
    .await;
    // The flight that ended takes its clearances with it before anything
    // arms (the approach setup's order).
    if super::setup::new_flight_began(&run.outcome) {
        crate::party_line::retire_for_new_flight();
        crate::flow::stand_down(crate::flow::StandDown::NewFlight("setup"));
    }
    let p = &job.placement;
    let result = match run.outcome {
        InitOutcome::Done(code) => Some(code),
        _ => None,
    };
    let failed = match run.outcome {
        InitOutcome::Done(0) => None,
        InitOutcome::Done(code) => Some(
            super::setup::init_refusal(code, "", "")
                .replace("The approach setup", "The enroute setup"),
        ),
        InitOutcome::Unreachable | InitOutcome::NoBaseline => {
            Some("I couldn't reach the plugin to set up the flight. Is it up to date?".into())
        }
        InitOutcome::TimedOut => Some("The enroute setup didn't answer. Check X-Plane.".into()),
    };
    if let Some(line) = failed {
        drop(loading);
        crate::blackbox::record(
            "setup",
            json!({ "event": "enroute_result", "outcome": run.outcome.name(), "result": result,
                    "engaged": false, "why": line, "ms": t0.elapsed().as_millis() as u64 }),
        );
        say(&line);
        return;
    }
    // Hands on first: the aircraft is flying.
    let engaged = engage(&xp, &job).await;
    if engaged.servos {
        crate::tools::speedhold::arm_flight(job.base.clone(), job.voice.clone(), p.ias_kts);
    }
    let (gear_up, flaps_up) = clean_up(&xp).await;
    // The world: an IFR flight mid-route, under the nearest radar unit.
    let mut station = None;
    let mut station_up = false;
    let mut xpdr = (false, false);
    if engaged.servos && crate::mode::atc_enabled() {
        if let Some(fix) =
            super::setup::settled_position(&xp, super::setup::SettleTiming::LIVE).await
        {
            let callsign = crate::circuit::crew_callsign(&xp).await.unwrap_or_default();
            station = crate::station::arm_new_flight_enroute(
                fix.lat,
                fix.lon,
                &callsign,
                std::time::Instant::now(),
            );
        }
        if let Some(code) = crate::party_line::ifr_flight_code() {
            xpdr = (
                crate::tools::radio::set_code(&xp, code).await,
                crate::tools::radio::set_mode(&xp, crate::tools::radio::XpdrMode::Altitude).await,
            );
        }
        if let Some(st) = &station {
            let _ = xp.set_float(crate::datarefs::COM1_FREQ_KHZ, st.khz as f64).await;
            station_up = super::setup::await_station_up(st.tower_khz, super::setup::STATION_UP_S)
                .await;
        }
    }
    // Ready only once settled (#177): the altimeter on the setting owed at
    // the level (standard above the transition), then the level and the
    // governed speed held.
    let settled = match engaged.servos {
        true => {
            let sync = crate::tools::instruments::sync_altimeter(&xp, job.transition_ft).await;
            if let Some(s) = &sync {
                crate::blackbox::record(
                    "setup",
                    json!({ "event": "enroute_altimeter", "was": s.was, "now": s.now,
                            "want": s.want, "took": s.took() }),
                );
            }
            settle(&xp, &job.base, p.alt_ft, sync.map(|s| s.took())).await
        }
        false => Settle::Settling,
    };
    let unsettled = match &settled {
        Settle::Unsettled(why) => Some(unsettled_line(why)),
        _ => None,
    };
    let outcome = match unsettled {
        Some(_) => "unsettled",
        None => run.outcome.name(),
    };
    drop(loading);
    crate::blackbox::record(
        "setup",
        json!({ "event": "enroute_result", "outcome": outcome, "result": result,
                "engaged": engaged.servos, "tries": engaged.tries, "why": unsettled,
                "settled": settled == Settle::Settled,
                "hdg_mode": engaged.heading, "vertical": engaged.vertical,
                "gear_up": gear_up, "flaps_up": flaps_up,
                "ias_kts": p.ias_kts, "alt_ft": p.alt_ft, "next_fix": p.next_fix,
                "station": station.as_ref().map(|s| &s.icao),
                "position": station.as_ref().map(|s| s.position),
                "khz": station.as_ref().map(|s| s.khz),
                "code": station.as_ref().map(|s| s.code),
                "squawk_set": xpdr.0, "alt_mode": xpdr.1, "station_up": station_up,
                "ms": t0.elapsed().as_millis() as u64 }),
    );
    let line = if let Some(why) = unsettled {
        format!("Enroute {}, but {why}", job.words)
    } else if engaged.servos {
        format!(
            "Enroute {}, {} and {:.0} knots, heading for {}. Autopilot holding. Say fly the \
             flight plan and I'll take it.",
            job.words,
            crate::flightplan::spoken_feet(p.alt_ft),
            p.ias_kts,
            p.next_fix
        )
    } else {
        "The autopilot didn't engage on the new flight. Your aircraft.".to_string()
    };
    say(&line);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The pilot's real OFP (EGKK-EGBB, B738, CI 77, FL180), trimmed.
    pub(crate) fn egkk_egbb() -> crate::simbrief::FlightPlan {
        let ofp = include_str!("../../bench/ofp/egkk-egbb-738.trimmed.json");
        let v: Value = serde_json::from_str(ofp).unwrap();
        crate::simbrief::plan_from_ofp(&v).unwrap()
    }

    /// The 737's numbers: 255 cruise, floor about 167, ceiling the cruise
    /// cap under the overspeed caution (Vne 340: 296).
    fn b738() -> Speeds {
        Speeds {
            profile_kts: Some(255.0),
            floor_kts: Some(167.0),
            ceiling_kts: Some(296.0),
            vmo_high: None,
        }
    }

    /// A start over a `vmo_high:` altitude takes the HIGH ceiling: with the
    /// low one alone (the Citation X's 233) the OFP's 299 at FL180 would
    /// have been cut to it.
    #[test]
    fn a_start_up_high_takes_the_high_vmo_ceiling() {
        let plan = egkk_egbb();
        let low_only = Speeds { ceiling_kts: Some(233.0), ..b738() };
        let p = place(&plan, &EnroutePoint::BeforeTod(30.0), &low_only).unwrap();
        assert_eq!(p.ias_kts, 233.0);
        let high = Speeds {
            vmo_high: Some(crate::aircraft::VmoHigh {
                above_ft: 8_000.0,
                kts: 350.0,
                mmo: Some(0.935),
            }),
            ..low_only
        };
        let q = place(&plan, &EnroutePoint::BeforeTod(30.0), &high).unwrap();
        assert_eq!(q.alt_ft, 18_000.0);
        assert_eq!(q.ias_kts, 299.0, "under the high ceiling, 305");
    }

    /// The navlog's top of descent (52.3054 N 1.2774 W) sits 16 nm past
    /// WELIN on the leg to PUFAX. Its `tod_to_dest_nm` (61) is the ARRIVAL
    /// as flown and would put it before WELIN: the point is measured from
    /// the TOD's own position.
    #[test]
    fn thirty_miles_before_the_top_of_descent_is_on_the_buzad_welin_leg_at_cruise() {
        let plan = egkk_egbb();
        assert!(plan.schedule.tod.is_some());
        let p = place(&plan, &EnroutePoint::BeforeTod(30.0), &b738()).unwrap();
        assert_eq!((p.leg_from.as_str(), p.next_fix.as_str()), ("BUZAD", "WELIN"));
        assert!((p.to_tod_nm.unwrap() - 30.0).abs() < 1e-6);
        // The TOD itself: on the WELIN-PUFAX leg, near the navlog's point.
        let at_tod = place(&plan, &EnroutePoint::BeforeTod(0.0), &b738()).unwrap();
        assert_eq!(at_tod.next_fix, "PUFAX");
        let off = crate::nav::haversine_nm(at_tod.lat, at_tod.lon, 52.305361, -1.277410);
        assert!(off < 1.0, "{off}");
        // FL180, the OFP's 299 at WELIN (M0.626 at FL180 is 299 too), and
        // the true speed the sim starts at is the plan's cruise TAS (394).
        assert_eq!(p.alt_ft, 18_000.0);
        assert_eq!((p.ias_kts, p.speed_from), (296.0, "ofp"), "capped under the caution");
        let uncapped = Speeds { ceiling_kts: None, ..b738() };
        let q = place(&plan, &EnroutePoint::BeforeTod(30.0), &uncapped).unwrap();
        assert_eq!(q.ias_kts, 299.0);
        assert!((q.tas_kts - 394.0).abs() < 8.0, "{}", q.tas_kts);
        // On the leg: its track toward WELIN, and on the line from BUZAD.
        let buzad = plan.waypoints.iter().find(|w| w.ident == "BUZAD").unwrap();
        let welin = plan.waypoints.iter().find(|w| w.ident == "WELIN").unwrap();
        let leg = crate::nav::bearing_deg(buzad.lat, buzad.lon, welin.lat, welin.lon);
        assert!((p.track_true - leg).abs() < 1.0, "{} vs {leg}", p.track_true);
        let d_buzad = crate::nav::haversine_nm(buzad.lat, buzad.lon, p.lat, p.lon);
        let d_welin = crate::nav::haversine_nm(p.lat, p.lon, welin.lat, welin.lon);
        let leg_nm = crate::nav::haversine_nm(buzad.lat, buzad.lon, welin.lat, welin.lon);
        assert!((d_buzad + d_welin - leg_nm).abs() < 0.05);
        assert!((d_welin - 13.9).abs() < 1.0, "{d_welin}");
        // The sim's JSON: the SDK's air start, engines running.
        let j: Value = serde_json::from_str(&enroute_init_json("Aircraft/b738.acf", &p)).unwrap();
        let s = &j["lle_air_start"];
        assert_eq!(j["aircraft"]["path"], "Aircraft/b738.acf");
        assert_eq!(s["elevation_in_meters"], 5486.4);
        let mps = s["speed_in_meters_per_second"].as_f64().unwrap();
        assert!((mps - p.tas_kts * KTS_TO_MPS).abs() < 0.1);
        assert!(s["heading_true"].as_f64().unwrap() > 300.0);
        assert_eq!(j["engine_status"]["all_engines"]["running"], true);
    }

    /// A point on the descent flies the plan's descent there; a fix is the
    /// fix, on the leg that leaves it; below ten thousand never over 250.
    #[test]
    fn a_point_on_the_descent_or_over_a_fix_is_the_plans_profile_there() {
        let plan = egkk_egbb();
        // 40 miles of route before EGBB: past the TOD, short of PUFAX.
        let d = place(&plan, &EnroutePoint::BeforeDestination(40.0), &b738()).unwrap();
        assert_eq!(d.next_fix, "PUFAX");
        assert!((16_800.0..18_000.0).contains(&d.alt_ft), "{}", d.alt_ft);
        assert!(d.to_tod_nm.unwrap() < 0.0);
        assert_eq!(d.ias_kts, 280.0);
        // Over HEMEL, heading for BUZAD at the cruise.
        let h = place(&plan, &EnroutePoint::Fix("hemel".into()), &b738()).unwrap();
        assert_eq!((h.leg_from.as_str(), h.next_fix.as_str()), ("HEMEL", "BUZAD"));
        let hemel = plan.waypoints.iter().find(|w| w.ident == "HEMEL").unwrap();
        assert_eq!((h.lat, h.lon, h.alt_ft), (hemel.lat, hemel.lon, 18_000.0));
        // Over HON (11,900), heading for OSKOT at 250 planned.
        let hon = place(&plan, &EnroutePoint::Fix("HON".into()), &b738()).unwrap();
        assert_eq!((hon.alt_ft, hon.ias_kts), (11_900.0, 250.0));
        // No schedule: the profile's cruise, capped at 250 under ten thousand.
        let mut bare = plan.clone();
        bare.schedule.fixes.clear();
        let g = place(&bare, &EnroutePoint::Fix("GROVE".into()), &b738()).unwrap();
        assert_eq!((g.alt_ft, g.ias_kts, g.speed_from), (3_300.0, 250.0, "profile"));
        assert_eq!(g.next_fix, "EGBB");
        let f = place(&bare, &EnroutePoint::Fix("HEMEL".into()), &b738()).unwrap();
        assert_eq!((f.ias_kts, f.speed_from), (255.0, "profile"));
    }

    /// Points the plan does not have are refused, naming why.
    #[test]
    fn a_point_off_the_plan_is_refused() {
        let plan = egkk_egbb();
        let e = |p: EnroutePoint| place(&plan, &p, &b738()).unwrap_err();
        assert_eq!(e(EnroutePoint::Fix("NOSUCH".into())), "NOSUCH is not on the plan.");
        assert!(e(EnroutePoint::Fix("EGKK".into())).contains("an end of the plan"));
        assert!(e(EnroutePoint::Fix("EGBB".into())).contains("an end of the plan"));
        assert!(e(EnroutePoint::BeforeTod(200.0)).contains("before the plan's first fix, D257B"));
        assert!(e(EnroutePoint::BeforeDestination(0.0)).contains("at or past EGBB"));
        assert!(e(EnroutePoint::BeforeTod(-1.0)).contains("0 to 1000"));
        // Too low to start in the air: 2 miles before EGBB, on the plan's
        // GROVE (3,300) to field line, is under 3,000.
        let low = e(EnroutePoint::BeforeDestination(2.0));
        assert!(low.contains("under the 3000"), "{low}");
        // A plan with no top of descent on its navlog.
        let mut no_tod = plan.clone();
        no_tod.schedule.tod = None;
        assert_eq!(
            place(&no_tod, &EnroutePoint::BeforeTod(30.0), &b738()).unwrap_err(),
            "The plan has no top of descent on its navlog."
        );
    }

    /// The airborne resume takes every placement: its state gates pass at
    /// the placement's height and speed, and it joins the leg the aircraft
    /// was put on, heading for the placement's next fix, within the 30 nm
    /// the rejoin checks the terrain over, at the plan's level there.
    #[test]
    fn the_resume_joins_the_leg_the_placement_is_on() {
        use crate::tools::full_auto::{resume_gate, resume_join, ResumeJoin};
        let plan = egkk_egbb();
        let legs = crate::procedure::enroute_legs(
            &plan.waypoints,
            &plan.origin,
            &plan.destination,
            plan.cruise_alt_ft,
        );
        for point in [
            EnroutePoint::BeforeTod(30.0),
            EnroutePoint::BeforeTod(0.0),
            EnroutePoint::Fix("HEMEL".into()),
            EnroutePoint::Fix("LAM".into()),
            EnroutePoint::BeforeDestination(40.0),
            EnroutePoint::BeforeDestination(20.0),
        ] {
            let p = place(&plan, &point, &b738()).unwrap();
            // Over flat England: the plan's altitude less a field's height.
            assert_eq!(resume_gate(false, false, p.alt_ft - 500.0, 0.0, p.ias_kts, 167.0), None);
            let join = resume_join(
                &legs,
                &plan.waypoints,
                &plan.destination,
                p.lat,
                p.lon,
                Some(-1.0),
                None,
                None,
            );
            let i = match join {
                ResumeJoin::Leg(i) => i,
                other => panic!("{point:?}: {other:?}"),
            };
            assert_eq!(legs[i].fix_ident, p.next_fix, "{point:?}");
            let fix = legs[i].fix.as_ref().unwrap();
            assert!(crate::nav::haversine_nm(p.lat, p.lon, fix.lat, fix.lon) <= 30.0, "{point:?}");
            assert!(legs[i].alt1_ft.unwrap() <= p.alt_ft + 1.0, "{point:?}: never a climb back");
        }
        // The gates still refuse what they refused: slow, banked, low.
        assert!(resume_gate(false, false, 18_000.0, 0.0, 150.0, 167.0).unwrap().contains("slow"));
        assert!(resume_gate(false, false, 800.0, 0.0, 250.0, 167.0).unwrap().contains("thousand"));
    }

    #[test]
    fn the_tas_of_an_airspeed_is_its_mach_at_the_level() {
        assert!((tas_kts(250.0, 0.0) - 250.0).abs() < 0.5);
        let fl180 = tas_kts(300.0, 18_000.0);
        assert!((380.0..=395.0).contains(&fl180), "{fl180}");
        assert!(tas_kts(250.0, 36_000.0) > tas_kts(250.0, 30_000.0));
    }

    #[test]
    fn the_typed_line_is_the_tools_arguments_back() {
        for p in [
            EnroutePoint::BeforeTod(30.0),
            EnroutePoint::BeforeDestination(40.0),
            EnroutePoint::Fix("WELIN".into()),
        ] {
            let t = crate::grammar::normalize(&enroute_line(&p));
            let c = crate::grammar::enroute_setup_call(&t).unwrap_or_else(|| panic!("{t}"));
            assert_eq!(c.name, "setup_enroute");
            assert_eq!(point_from_args(&c.arguments).unwrap(), p, "{t}");
        }
        let at_tod = crate::grammar::enroute_setup_call("put us en route at the top of descent");
        assert_eq!(point_from_args(&at_tod.unwrap().arguments), Ok(EnroutePoint::BeforeTod(0.0)));
        // CLOSED: nothing else is a new flight in the air.
        for t in [
            "put us enroute",
            "put us enroute 30 miles",
            "put us enroute thirty miles after top of descent",
            "put us on a 10 mile final",
            "put us enroute at the gate please now",
            "we are enroute at welin",
        ] {
            assert!(crate::grammar::enroute_setup_call(t).is_none(), "{t}");
        }
    }

    /// #177, the second flown air start (2026-09-29, state records from
    /// the enroute_result at t 1790648838.5, every five seconds): seconds,
    /// feet, knots, fpm. ALT HOLD engaged in the zoom, the governor at 296
    /// and full power, the altimeter at 30.03 at FL180.
    const PHUGOID: [(f64, f64, f64, f64); 14] = [
        (0.0, 18155.0, 268.0, 9310.0),
        (4.8, 19184.0, 231.0, 15897.0),
        (9.4, 20497.0, 192.0, 18086.0),
        (14.1, 21881.0, 150.0, 16934.0),
        (18.9, 23110.0, 104.0, 12920.0),
        (23.6, 23918.0, 61.0, 7433.0),
        (28.5, 24243.0, 20.0, 481.0),
        (33.3, 23997.0, 39.0, -6490.0),
        (38.0, 23166.0, 119.0, -14965.0),
        (42.7, 21785.0, 178.0, -19209.0),
        (47.5, 20334.0, 228.0, -15935.0),
        (52.1, 19516.0, 255.0, -4309.0),
        (56.7, 19730.0, 253.0, 9883.0),
        (61.5, 20985.0, 223.0, 20282.0),
    ];

    /// That flight was called ready at +14 s. Judged by the settle it is
    /// never settled, and at a minute it names all three things.
    #[test]
    fn the_phugoid_air_start_is_never_called_ready() {
        let mut j = SettleJudge::default();
        let mut end = Settle::Settling;
        for (t_s, alt_ft, ias_kts, _) in PHUGOID {
            let s = SettleSample {
                t_s,
                alt_ft,
                ias_kts,
                governed_kts: Some(296.0),
                altimeter_ok: Some(false),
            };
            end = j.step(18_000.0, &s);
            if t_s < SETTLE_MAX_S {
                assert_eq!(end, Settle::Settling, "{t_s}");
            }
        }
        // The same last look, held until the budget runs out.
        let (_, alt_ft, ias_kts, _) = PHUGOID[PHUGOID.len() - 1];
        let last = SettleSample {
            t_s: SETTLE_MAX_S,
            alt_ft,
            ias_kts,
            governed_kts: Some(296.0),
            altimeter_ok: Some(false),
        };
        end = j.step(18_000.0, &last);
        let Settle::Unsettled(why) = end else { panic!("{end:?}") };
        assert_eq!(
            why,
            vec![
                "2985 feet off the 18000 dial".to_string(),
                "223 knots against 296".to_string(),
                "the altimeter is not set".to_string(),
            ]
        );
        let line = unsettled_line(&why);
        let head = format!("it has not settled after {SETTLE_MAX_S:.0} seconds: 2985 feet");
        assert!(line.starts_with(&head), "{line}");
        assert!(!line.contains('\u{2014}') && !line.contains('\u{2013}'));
    }

    /// #181: an autopilot with no pitch is never "the autopilot has it".
    #[test]
    fn a_setup_with_no_autopilot_pitch_says_who_flies_it() {
        let line = unsettled_line(&[NO_PITCH_WHY.to_string()]);
        assert!(line.starts_with("the autopilot has no pitch control."), "{line}");
        assert!(line.contains("say my controls") && !line.contains("The autopilot has it"));
        assert!(!line.contains('\u{2014}') && !line.contains('\u{2013}'));
    }

    /// A flight at the dial, the speed governed and standard set, is ready
    /// after ten seconds level and not before; leaving the band restarts it.
    #[test]
    fn a_level_governed_flight_is_ready_after_ten_seconds() {
        let mut j = SettleJudge::default();
        let s = |t_s: f64, alt_ft: f64, ias_kts: f64| SettleSample {
            t_s,
            alt_ft,
            ias_kts,
            governed_kts: Some(296.0),
            altimeter_ok: Some(true),
        };
        assert_eq!(j.step(18_000.0, &s(0.0, 18_120.0, 290.0)), Settle::Settling);
        assert_eq!(j.step(18_000.0, &s(6.0, 18_200.0, 290.0)), Settle::Settling);
        assert_eq!(j.step(18_000.0, &s(7.0, 18_060.0, 280.0)), Settle::Settling);
        assert_eq!(j.step(18_000.0, &s(16.5, 17_950.0, 285.0)), Settle::Settling);
        assert_eq!(j.step(18_000.0, &s(17.0, 17_990.0, 287.0)), Settle::Settled);
        // Level but with nobody on the speed, or the altimeter unread: no.
        let mut k = SettleJudge::default();
        let bare = |t_s| SettleSample { governed_kts: None, ..s(t_s, 18_000.0, 296.0) };
        assert_eq!(k.step(18_000.0, &bare(0.0)), Settle::Settling);
        assert_eq!(k.step(18_000.0, &bare(30.0)), Settle::Settling);
        let Settle::Unsettled(why) = k.step(18_000.0, &bare(SETTLE_MAX_S)) else { panic!() };
        assert_eq!(why, vec!["no speed hold".to_string()]);
    }

    /// The vertical from the same records: toward the dial by VS at the
    /// engage (17,813 ft climbing), back down by VS once the hold ran
    /// through the dial, and a hold AT the dial or closing on it left alone.
    #[test]
    fn the_vertical_goes_to_the_dial_by_vs_never_an_alt_hold_off_it() {
        assert_eq!(vertical_toward_dial(17_813.0, 18_000.0, 4_450.0, false), Vertical::Vs(400.0));
        // PHUGOID +0: 155 ft through the dial at 9,310 fpm under ALT HOLD.
        let (_, alt, _, vvi) = PHUGOID[0];
        assert_eq!(vertical_toward_dial(alt, 18_000.0, vvi, true), Vertical::Vs(-300.0));
        // +9.4: 2,497 ft over, the wheel at its bound.
        let (_, alt, _, vvi) = PHUGOID[2];
        assert_eq!(vertical_toward_dial(alt, 18_000.0, vvi, true), Vertical::Vs(-1_500.0));
        assert_eq!(vertical_toward_dial(18_090.0, 18_000.0, -200.0, true), Vertical::Leave);
        assert_eq!(vertical_toward_dial(18_250.0, 18_000.0, -900.0, true), Vertical::Leave);
        assert_eq!(vertical_toward_dial(18_250.0, 18_000.0, 900.0, true), Vertical::Vs(-500.0));
        assert_eq!(vertical_toward_dial(18_030.0, 18_000.0, 0.0, false), Vertical::Vs(0.0));
        assert_eq!(vertical_toward_dial(17_900.0, 18_000.0, 0.0, false), Vertical::Vs(300.0));
    }
}
