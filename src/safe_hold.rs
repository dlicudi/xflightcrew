//! The safe hold: an airborne aircraft that NOBODY flies is never left in a
//! state that flies into the ground.
//!
//! LEMG 737, 2026-10-05 (sortie 1791187965-full_auto): xvl was restarted at
//! 12:42:27 with the autopilot still flying autoland's last command, VS -330
//! fpm with the altitude window at 0 ft, heading 306 toward the hills west of
//! Malaga at 2,900 ft. Nothing of ours owned the aircraft after the restart,
//! three "fly the flight plan" were refused ("I haven't taken control"), and
//! the aircraft kept descending until the pilot pulled up by hand at about
//! 270 ft above the ground. A refusal, a cancel, a give-up, a stand-down and
//! a fresh process start all left whatever the autopilot was last told.
//!
//! # Design
//!
//! **The invariant.** Airborne, no crew loop owns the aircraft
//! (`flight_state::crew_has_aircraft()` is `None`), and no pilot is flying it:
//! then the autopilot's trajectory must stay at or above the SAFE FLOOR, the
//! highest ground ahead along the track ([`LOOK_NM`]) plus
//! [`crate::procedure::TERRAIN_CLEARANCE_FT`], from the sim's own terrain
//! probe (grid MORA, then the ground under us, when the probe is silent).
//!
//! **Who flies** ([`Who`]), judged every second, never assumed:
//! - a crew loop: nothing to do here, the loop is the one writer;
//! - HANDS: the yoke deflected by someone other than our own channel writes,
//!   or a plugin yield latch: the pilot is flying, we only warn;
//! - AP OFF: the servos are out, so a human holds the controls: warn only;
//! - MCP: the pilot moved the autopilot (a dial, a mode, the servos) since
//!   the aircraft became nobody's, or since our own hold was set: the pilot
//!   flies through the autopilot, we warn once and act only to escape;
//! - UNATTENDED: none of the above. The autopilot is flying what it was last
//!   told, by a loop that has gone, a boot, or a release.
//!
//! **What is unsafe** ([`vertical`], [`unsafe_why`]): where the autopilot's
//! vertical mode will take the aircraft. A hold below the floor, a capture
//! (the window) below it, a descent with no window to stop it, or a climb too
//! shallow for the ground ahead. A coupled glideslope and any flight
//! aligned with a nearby runway on a descent path ([`on_approach`]) is an
//! approach, never "into the ground".
//!
//! **What is done** ([`SafeHold::step`], a state machine over one EPISODE,
//! from the moment the aircraft is nobody's until a crew loop takes it, it
//! lands, or a new flight begins):
//! - UNATTENDED and unsafe for [`CONFIRM_S`]: the HOLD. A climb to the floor
//!   (on a heading whose ground the climb clears, [`clear_heading`], else the
//!   current one), or a descent bounded at the floor. Set through the
//!   pilot's own tools (`set_heading`, `set_climb`: their gates, the capture
//!   watcher, the climb's speed protection), the speed held by the governor
//!   at the configuration's hand-back speed when no autothrottle holds it,
//!   read back, said once, blackboxed `safe_hold` `hold`. Never re-asserted
//!   against the pilot: any later move of a dial is the pilot's.
//! - The autopilot's ANSWER is watched ([`VERIFY_S`]): a hold that does not
//!   climb, or a bound that the descent flies through, is taken by the
//!   recovery (`recover::take_over`, hand flown to the floor until "my
//!   controls").
//! - UNATTENDED and unflyable (stall, slow, upset): the recovery at once.
//! - ESCAPE ([`escape_due`]): the projected path meets the ground (plus
//!   [`ESCAPE_CLEAR_FT`]) inside [`ESCAPE_LOOK_S`]. The one case that acts at
//!   once, and the only one that acts over a pilot flying the autopilot (no
//!   hands on the yoke). A pilot hand flying only hears "Terrain, pull up".
//! - Not ours to actuate (instructor or ATC-only mode, no autopilot): the
//!   warning only. A rotorcraft is the hover loop's (`hover::hand_back`),
//!   never judged here: it flies low on purpose.
//!
//! The decisions are pure ([`SafeHold::step`] over a [`Tick`]), so the real
//! 2026-10-05 records replay through them (`bench/replay/`). The live driver
//! ([`spawn`]) is its own thread at 1 Hz, reads nothing while a loop owns the
//! aircraft, and probes the terrain only while the aircraft is nobody's.

use serde_json::{json, Value};

// ---------------------------------------------------------------- constants

/// How far ahead along the track the floor looks, nm (the follower's look).
pub const LOOK_NM: f64 = 15.0;
/// Points along it (one a nautical mile).
pub const LOOK_POINTS: usize = 15;
/// The fan's rays, degrees either side of the track, for a heading whose
/// ground the climb clears.
pub const FAN_OFFSETS_DEG: [f64; 11] = [
    0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0, 150.0, 180.0,
];
/// The fan's reach and points per ray.
pub const FAN_LOOK_NM: f64 = 12.0;
pub const FAN_POINTS: usize = 6;
/// Clearance over the ground every hold keeps, feet.
pub const CLEARANCE_FT: f64 = crate::procedure::TERRAIN_CLEARANCE_FT;
/// While climbing, the margin over each point of ground on the way, feet.
pub const CLIMB_MARGIN_FT: f64 = 500.0;
/// Room for the turn before the climb is credited, nm.
pub const TURN_ROOM_NM: f64 = 2.0;
/// The escape looks this far ahead along the projected path, seconds...
pub const ESCAPE_LOOK_S: f64 = 60.0;
/// ...for a projected clearance under this, feet.
pub const ESCAPE_CLEAR_FT: f64 = 500.0;
/// Unsafe this long before the hold is set, seconds: one blip of a mode
/// change mid-capture is not a trajectory.
pub const CONFIRM_S: f64 = 3.0;
/// The autopilot's answer to a hold is judged after this, seconds.
pub const VERIFY_S: f64 = 25.0;
/// A hold is raised for higher ground ahead at most this often, seconds.
pub const REPLAN_S: f64 = 30.0;
/// A stop this far below the floor is below it, feet (the window's detent).
pub const FLOOR_TOL_FT: f64 = 100.0;
/// Past this, a yoke under the autopilot is a hand (the monitor's figure).
pub const YOKE_HANDS: f64 = 0.15;
/// A pilot's "my controls" this recent names the episode's cause.
pub const RELEASE_WORDS_S: f64 = 10.0;
/// "Terrain! Pull up." repeats this often while the ground is still
/// coming, seconds (a GPWS repeats it continuously; this is a voice).
pub const WARN_REPEAT_S: f64 = 10.0;
/// Unflyable this long, unattended, before the recovery takes it, seconds.
pub const UNFLYABLE_S: f64 = 2.0;

// The autopilot's mode bits are the panel owner's (`autoflight::Modes`, the
// `autopilot_state` bitfield; 4 wing leveler, 512 HNAV and 8192 FMS engaged
// are the autopilot's own, never read here).
use crate::autoflight::{Modes, OurChange, Ours};

// ------------------------------------------------------------- the readings

/// What the autopilot is doing, as read. A failed read is `None`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ApRead {
    pub servos: Option<bool>,
    /// `sim/cockpit/autopilot/autopilot_state`, decoded by the panel owner
    /// ([`Modes::from_read`]: a negative or broken read is no state).
    pub state: Option<Modes>,
    pub alt_dial_ft: Option<f64>,
    pub vs_dial_fpm: Option<f64>,
    pub hdg_bug_mag: Option<f64>,
}

impl ApRead {
    fn has(&self, bit: u32) -> bool {
        self.state.is_some_and(|s| s.0 & bit != 0)
    }
    /// The autothrottle holds the speed (the pilot's or the aircraft's):
    /// our governor would fight it.
    pub fn autothrottle(&self) -> bool {
        self.has(Modes::AUTOTHROTTLE)
    }
    pub fn json(&self) -> Value {
        json!({ "servos": self.servos, "state": self.state.map(|m| m.0),
                "alt_dial": self.alt_dial_ft.map(r0),
                "vs_dial": self.vs_dial_fpm.map(r0), "hdg_bug": self.hdg_bug_mag.map(r0) })
    }
}

/// The aircraft, as read. Altitudes in feet: `alt_ft` is the altimeter (what
/// the autopilot flies), `alt_true_ft` the height above the sea (what the
/// ground is compared with).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Air {
    pub lat: f64,
    pub lon: f64,
    pub alt_ft: f64,
    pub alt_true_ft: f64,
    pub agl_ft: f64,
    pub vs_fpm: f64,
    pub ias_kts: f64,
    pub gs_kts: f64,
    pub track_true: f64,
    /// X-Plane's `magnetic_variation` (positive WEST: mag = true + var).
    pub mag_var: f64,
    pub bank_deg: f64,
    pub pitch_deg: f64,
    pub flap: f64,
}

impl Air {
    /// The ground under the aircraft, feet MSL.
    pub fn ground_under_ft(&self) -> f64 {
        self.alt_true_ft - self.agl_ft
    }
    /// A true altitude as the altimeter reads it.
    pub fn indicated(&self, true_ft: f64) -> f64 {
        true_ft + (self.alt_ft - self.alt_true_ft)
    }
    pub fn json(&self) -> Value {
        json!({ "lat": r4(self.lat), "lon": r4(self.lon), "alt": r0(self.alt_ft),
                "alt_true": r0(self.alt_true_ft), "agl": r0(self.agl_ft), "vs": r0(self.vs_fpm),
                "ias": r0(self.ias_kts), "gs": r0(self.gs_kts), "track": r0(self.track_true),
                "bank": r0(self.bank_deg), "flap": (self.flap * 100.0).round() / 100.0 })
    }
}

/// The ground along one bearing: `(nm out, feet MSL)`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Ray {
    pub bearing_true: f64,
    pub ground_ft: Vec<(f64, f64)>,
}

impl Ray {
    pub fn max_ft(&self) -> Option<f64> {
        self.ground_ft.iter().map(|(_, g)| *g).reduce(f64::max)
    }
}

/// The terrain picture the floor is judged on. `None` / empty = not probed.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Terrain {
    /// Along the track.
    pub ahead: Option<Ray>,
    /// Round the compass, for a clear heading (probed when a climb needs it).
    pub fan: Vec<Ray>,
    /// The grid MORA here and ahead (already the ground plus its margin).
    pub mora_ft: Option<f64>,
}

/// The safe floor, true feet, and where it came from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Floor {
    pub true_ft: f64,
    pub source: &'static str,
}

/// The floor: the highest ground ahead (and under) plus [`CLEARANCE_FT`]; the
/// grid MORA without a probe; the ground under us plus the clearance with
/// neither (the sim always knows the height above the ground). Pure.
pub fn floor(air: &Air, t: &Terrain) -> Floor {
    let under = air.ground_under_ft();
    if let Some(m) = t.ahead.as_ref().and_then(Ray::max_ft) {
        return Floor {
            true_ft: m.max(under) + CLEARANCE_FT,
            source: "terrain",
        };
    }
    if let Some(m) = t.mora_ft.filter(|m| m.is_finite()) {
        return Floor {
            true_ft: m.max(under + CLEARANCE_FT),
            source: "mora",
        };
    }
    Floor {
        true_ft: under + CLEARANCE_FT,
        source: "under",
    }
}

/// Where the autopilot's vertical mode takes the aircraft.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Vertical {
    /// Level there (indicated feet).
    Holds(f64),
    /// Descending to a window it will capture (indicated feet).
    Captures(f64),
    /// Climbing.
    Climbs,
    /// Descending with nothing to stop it.
    Unbounded,
    /// Coupled to a glideslope: an approach.
    Glideslope,
}

/// The vertical path the autopilot is on. A failed dial read is no window.
/// Pure.
pub fn vertical(ap: &ApRead, air: &Air) -> Vertical {
    if ap.has(Modes::GS) {
        return Vertical::Glideslope;
    }
    let alt = air.alt_ft;
    if ap.has(Modes::ALT_HOLD) {
        return Vertical::Holds(alt);
    }
    let window = ap.alt_dial_ft.filter(|w| w.is_finite());
    let mode_descends = if ap.has(Modes::VS) {
        ap.vs_dial_fpm.map(|v| v < -50.0)
    } else if ap.has(Modes::LEVEL_CHANGE) || ap.has(Modes::VNAV) {
        window.map(|w| w < alt - FLOOR_TOL_FT)
    } else {
        None
    };
    let descending = mode_descends.unwrap_or(air.vs_fpm < -300.0);
    let climbing = if ap.has(Modes::VS) {
        ap.vs_dial_fpm.is_some_and(|v| v > 50.0)
    } else if ap.has(Modes::LEVEL_CHANGE) || ap.has(Modes::VNAV) {
        window.is_some_and(|w| w > alt + FLOOR_TOL_FT)
    } else {
        air.vs_fpm > 300.0
    };
    if descending {
        // A window below in the direction of travel is captured where the
        // mode arms it: FLCH and VNAV always, VS with the altitude armed.
        let armed = ap.has(Modes::ALT_ARMED) || ap.has(Modes::LEVEL_CHANGE) || ap.has(Modes::VNAV);
        return match window {
            Some(w) if w < alt - FLOOR_TOL_FT && armed => Vertical::Captures(w),
            _ => Vertical::Unbounded,
        };
    }
    if climbing {
        return Vertical::Climbs;
    }
    Vertical::Holds(alt)
}

/// Why the trajectory is unsafe.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Why {
    /// Level (or about to be) below the floor.
    HoldsBelow,
    /// Descending to a window below the floor.
    CapturesBelow,
    /// Descending with no window.
    NoFloor,
    /// Climbing too shallow for the ground ahead.
    ClimbShort,
}

impl Why {
    pub fn word(self) -> &'static str {
        match self {
            Why::HoldsBelow => "holds_below",
            Why::CapturesBelow => "captures_below",
            Why::NoFloor => "no_floor",
            Why::ClimbShort => "climb_short",
        }
    }
}

/// The climb clears every point of the ray: the margin over each, at the
/// gradient (feet per nm), with [`TURN_ROOM_NM`] flown before it is
/// credited when `turning`. Pure.
pub fn climb_clears(ray: &Ray, alt_true_ft: f64, gradient_ft_nm: f64, turning: bool) -> bool {
    let room = if turning { TURN_ROOM_NM } else { 0.0 };
    ray.ground_ft.iter().all(|&(d, g)| {
        g + CLIMB_MARGIN_FT <= alt_true_ft + gradient_ft_nm.max(0.0) * (d - room).max(0.0)
    })
}

/// The climb gradient a vertical speed flies at a groundspeed, feet per nm.
pub fn gradient_ft_nm(vs_fpm: f64, gs_kts: f64) -> f64 {
    vs_fpm * 60.0 / gs_kts.max(60.0)
}

/// Is the trajectory unsafe against the floor (indicated feet)? Pure.
pub fn unsafe_why(v: Vertical, air: &Air, floor_ind_ft: f64, ahead: Option<&Ray>) -> Option<Why> {
    match v {
        Vertical::Glideslope => None,
        Vertical::Holds(h) => (h < floor_ind_ft - FLOOR_TOL_FT).then_some(Why::HoldsBelow),
        Vertical::Captures(w) => (w < floor_ind_ft - FLOOR_TOL_FT).then_some(Why::CapturesBelow),
        Vertical::Unbounded => Some(Why::NoFloor),
        Vertical::Climbs => {
            if air.alt_ft >= floor_ind_ft - FLOOR_TOL_FT {
                return None;
            }
            let grad = gradient_ft_nm(air.vs_fpm, air.gs_kts);
            let clears = ahead.is_none_or(|r| climb_clears(r, air.alt_true_ft, grad, false));
            (!clears).then_some(Why::ClimbShort)
        }
    }
}

/// A landing runway end, for the approach test.
#[derive(Debug, Clone, PartialEq)]
pub struct End {
    pub ident: String,
    pub thr_lat: f64,
    pub thr_lon: f64,
    pub course_true: f64,
    pub elev_ft: f64,
    pub length_m: f64,
}

/// Inside this far from the threshold an aligned descent is an approach, nm.
pub const APPROACH_NM: f64 = 12.0;
/// Track within this of the landing course, degrees.
pub const APPROACH_ALIGN_DEG: f64 = 30.0;
/// The highest path an approach is flown on, degrees, and the margin.
pub const APPROACH_MAX_PATH_DEG: f64 = 4.5;
pub const APPROACH_HIGH_FT: f64 = 600.0;

/// The runway end the aircraft is flying an approach to, if any: in front of
/// its threshold inside [`APPROACH_NM`] (or over the pavement), inside a
/// splay of the centreline, tracking the landing course, not above a steep
/// path and not climbing. Pure.
pub fn on_approach(ends: &[End], air: &Air) -> Option<String> {
    if air.vs_fpm > 500.0 {
        return None;
    }
    ends.iter().find_map(|e| {
        let (x_m, along_m) =
            crate::nav::runway_frame_m(e.thr_lat, e.thr_lon, e.course_true, air.lat, air.lon);
        let (along_nm, xtk_nm) = (along_m / 1852.0, x_m / 1852.0);
        let over_pavement = along_m <= 0.0 && -along_m <= e.length_m;
        let in_front = along_nm > 0.0 && along_nm <= APPROACH_NM;
        let splay = (along_nm.max(0.0) * 0.25).max(0.6);
        let aligned =
            crate::angles::abs_diff_deg(air.track_true, e.course_true) <= APPROACH_ALIGN_DEG;
        let high = air.alt_true_ft - e.elev_ft;
        let max_h = along_m.max(0.0) * 3.280_84 * APPROACH_MAX_PATH_DEG.to_radians().tan()
            + APPROACH_HIGH_FT;
        ((in_front || over_pavement) && xtk_nm.abs() <= splay && aligned && high <= max_h)
            .then(|| e.ident.clone())
    })
}

/// The projected path meets the ground: along the track, at the vertical
/// speed, inside [`ESCAPE_LOOK_S`], closer than its clearance to the ground at
/// that point ([`ESCAPE_CLEAR_FT`]); away from a field, also under it and
/// sinking now. Near a field ([`in_terminal`]) only a collision course counts
/// ([`TERMINAL_CLEAR_FT`] inside [`TERMINAL_LOOK_S`]): a circuit and a visual
/// approach descend low near the ground on purpose. Pure.
pub fn escape_due(air: &Air, ahead: Option<&Ray>, terminal: bool) -> bool {
    let gs = air.gs_kts.max(60.0);
    let clear = if terminal {
        TERMINAL_CLEAR_FT
    } else {
        ESCAPE_CLEAR_FT
    };
    let look = if terminal {
        TERMINAL_LOOK_S
    } else {
        ESCAPE_LOOK_S
    };
    if !terminal && air.agl_ft < ESCAPE_CLEAR_FT && air.vs_fpm < -300.0 {
        return true;
    }
    ahead.is_some_and(|r| {
        r.ground_ft.iter().any(|&(d, g)| {
            let t_s = d / gs * 3600.0;
            t_s <= look && air.alt_true_ft + air.vs_fpm * t_s / 60.0 < g + clear
        })
    })
}

/// Inside this far from a runway end, nm...
pub const TERMINAL_NM: f64 = 5.0;
/// ...and this high above it, feet, the aircraft is flying the field's
/// circuit or its visual approach.
pub const TERMINAL_HEIGHT_FT: f64 = 2_000.0;
/// Near a field the escape asks only for a collision course: this
/// clearance, this soon (a base leg at 600 ft sinking 700 meets the ground in
/// 51 s on paper and turns final in 20).
pub const TERMINAL_CLEAR_FT: f64 = 100.0;
pub const TERMINAL_LOOK_S: f64 = 20.0;

/// The field whose circuit the aircraft is flying in, if any: a runway end
/// inside [`TERMINAL_NM`] and the aircraft under [`TERMINAL_HEIGHT_FT`] above
/// it. Pure.
pub fn in_terminal(ends: &[End], air: &Air) -> Option<String> {
    ends.iter()
        .find(|e| {
            crate::nav::haversine_nm(e.thr_lat, e.thr_lon, air.lat, air.lon) <= TERMINAL_NM
                && air.alt_true_ft - e.elev_ft <= TERMINAL_HEIGHT_FT
        })
        .map(|e| e.ident.clone())
}

/// The heading whose ground the climb clears, nearest the track: a ray the
/// climb clears and whose highest ground the hold altitude clears first,
/// then the lowest ground, then the smallest turn. Returns the bearing
/// (true) and the altitude it needs (true feet). `None` with no fan. Pure.
pub fn clear_heading(
    fan: &[Ray],
    air: &Air,
    target_true_ft: f64,
    gradient_ft_nm: f64,
) -> Option<(f64, f64)> {
    fan.iter()
        .filter_map(|r| {
            let need = r.max_ft()? + CLEARANCE_FT;
            let turn = crate::angles::abs_diff_deg(r.bearing_true, air.track_true);
            let clears = climb_clears(r, air.alt_true_ft, gradient_ft_nm, turn > 15.0);
            let excess = (need - target_true_ft).max(0.0);
            Some(((!clears) as u8, excess, turn, r.bearing_true, need))
        })
        .min_by(|a, b| {
            a.0.cmp(&b.0)
                .then(a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
                .then(a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
        })
        .map(|(_, _, _, b, need)| (b, need))
}

/// What the hold sets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoldCmd {
    /// The altitude, indicated feet, a round hundred.
    pub alt_ft: f64,
    /// The rate to it, fpm (magnitude).
    pub fpm: f64,
    /// A heading to turn to, magnetic; `None` keeps the lateral mode.
    pub heading_mag: Option<f64>,
    pub climb: bool,
    pub why: Why,
    pub floor: Floor,
}

impl HoldCmd {
    pub fn json(&self) -> Value {
        json!({ "alt_ft": self.alt_ft, "fpm": r0(self.fpm), "heading": self.heading_mag.map(r0),
                "climb": self.climb, "why": self.why.word(), "floor_ft": r0(self.floor.true_ft),
                "floor_source": self.floor.source })
    }
}

/// Round up to the next hundred feet.
pub fn up_100(ft: f64) -> f64 {
    (ft / 100.0).ceil() * 100.0
}

/// The hold for an unsafe trajectory: the floor as a round hundred, a climb
/// at the airframe's rate when below it, the descent bounded there when above
/// it; a heading off the track only when the climb does not clear the ground
/// ahead and the fan finds one that does. `capability_fpm` is the climb the
/// airframe can make. Pure.
pub fn hold_cmd(
    why: Why,
    air: &Air,
    ap: &ApRead,
    fl: Floor,
    t: &Terrain,
    capability_fpm: f64,
) -> HoldCmd {
    let mut target_true = fl.true_ft;
    let climb = air.alt_ft < air.indicated(target_true) - FLOOR_TOL_FT;
    let mut heading_mag = None;
    let grad = gradient_ft_nm(capability_fpm, air.gs_kts);
    if climb {
        let ahead_clears = t
            .ahead
            .as_ref()
            .is_none_or(|r| climb_clears(r, air.alt_true_ft, grad, false));
        if !ahead_clears {
            if let Some((brg, need)) = clear_heading(&t.fan, air, target_true, grad) {
                if crate::angles::abs_diff_deg(brg, air.track_true) > 5.0 {
                    heading_mag = Some(crate::runway::true_to_mag(brg, air.mag_var).round());
                }
                target_true = target_true.max(need);
            }
        }
    }
    let alt_ft = up_100(air.indicated(target_true));
    let fpm = if climb {
        capability_fpm.clamp(500.0, 2_000.0)
    } else {
        ap.vs_dial_fpm
            .map(f64::abs)
            .filter(|v| v.is_finite())
            .unwrap_or(1_000.0)
            .clamp(300.0, 1_500.0)
    };
    HoldCmd {
        alt_ft,
        fpm,
        heading_mag,
        climb,
        why,
        floor: fl,
    }
}

/// The escape's command: never a descent. Above the floor the plain hold
/// bounds a descent AT the floor (window at the floor, the descent rate
/// kept), which on a path that meets the ground is still a descent: LEMD
/// 2026-10-05 17:07, diving at 4,000 fpm through 4,015 ft, the escape set
/// 3,100 at -1,000 fpm. An escape levels at once, at the floor or where the
/// aircraft is, whichever is higher, climbing to it. Pure.
pub fn escape_cmd(mut cmd: HoldCmd, alt_ft: f64) -> HoldCmd {
    if !cmd.climb {
        cmd.alt_ft = up_100(cmd.alt_ft.max(alt_ft));
        cmd.climb = true;
        cmd.fpm = ESCAPE_LEVEL_FPM;
    }
    cmd
}

/// The rate an escape that is already above its floor levels off at, fpm.
pub const ESCAPE_LEVEL_FPM: f64 = 500.0;

/// Who flies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Who {
    Hands,
    ApOff,
    Mcp,
    Unattended,
}

impl Who {
    pub fn word(self) -> &'static str {
        match self {
            Who::Hands => "hands",
            Who::ApOff => "ap_off",
            Who::Mcp => "mcp",
            Who::Unattended => "unattended",
        }
    }
}

/// Our own changes from this long before the reference count (the hold's
/// writes precede the `written` that stamps them).
pub const OURS_SLACK_S: f64 = 10.0;

/// The bits a pilot presses and the autopilot never sets by itself.
/// (An armed mode going engaged, a capture, the wing leveler the servos
/// default to: the autopilot's own, never counted.)
const PILOT_MODE_BITS: u32 = Modes::AUTOTHROTTLE
    | Modes::SPEED_ON_PITCH
    | Modes::LEVEL_CHANGE
    | Modes::LOC_ARMED
    | Modes::FMS_ARMED
    | Modes::GS_ARMED
    | Modes::TOGA_LATERAL
    | Modes::TOGA_VERTICAL
    | Modes::VNAV_ARMED;

/// Did someone move the autopilot since `reference` (the episode's first
/// read, or our own hold read back)? The servos coming in or out, the window
/// moved, the heading bug moved in HDG mode, a mode the autopilot never
/// engages by itself, and outside our hold the VS wheel or a new VS/HDG/ALT
/// HOLD that is no capture. `held`: our hold is set (its climb moves the VS
/// wheel and the capture brings ALT HOLD). `ours`: the panel owner's ledger
/// of OUR presses and dial writes since the reference
/// (`autoflight::ours_since`): every press and write in the tree goes through
/// it, so a change it explains is ours, never the pilot's. Pure.
pub fn mcp_moved(
    reference: &ApRead,
    now: &ApRead,
    held: bool,
    alt_ft: f64,
    ours: &Ours,
) -> Option<&'static str> {
    use crate::autoflight::Dial;
    if let (Some(a), Some(b)) = (reference.servos, now.servos) {
        if a != b && !ours.servos() {
            return Some("servos");
        }
    }
    if let (Some(a), Some(b)) = (reference.alt_dial_ft, now.alt_dial_ft) {
        if (a - b).abs() > FLOOR_TOL_FT && !ours.wrote(Dial::Altitude, b, 50.0) {
            return Some("window");
        }
    }
    if let (Some(a), Some(b), true) = (
        reference.hdg_bug_mag,
        now.hdg_bug_mag,
        reference.has(Modes::HEADING) && now.has(Modes::HEADING),
    ) {
        if crate::angles::abs_diff_deg(a, b) > 3.0 && !ours.wrote(Dial::HeadingBug, b, 3.0) {
            return Some("heading");
        }
    }
    let (Some(Modes(s0)), Some(Modes(s1))) = (reference.state, now.state) else {
        return None;
    };
    // A mode our own press brought in is ours.
    let new = s1 & !s0 & !ours.mode_bits();
    if new & PILOT_MODE_BITS != 0 {
        return Some("mode");
    }
    if held {
        return None;
    }
    if let (Some(a), Some(b), true) = (
        reference.vs_dial_fpm,
        now.vs_dial_fpm,
        s0 & Modes::VS != 0 && s1 & Modes::VS != 0,
    ) {
        if (a - b).abs() > FLOOR_TOL_FT && !ours.wrote(Dial::Vs, b, FLOOR_TOL_FT) {
            return Some("vs_wheel");
        }
    }
    if new & (Modes::VS | Modes::HEADING) != 0 {
        return Some("mode");
    }
    // ALT HOLD appearing away from the window is a press, not a capture.
    if new & Modes::ALT_HOLD != 0 && now.alt_dial_ft.is_some_and(|w| (w - alt_ft).abs() > 300.0) {
        return Some("alt_hold");
    }
    None
}

// ------------------------------------------------------------ the episode

/// One tick's facts, everything the decision needs.
#[derive(Debug, Clone)]
pub struct Tick {
    pub now_s: f64,
    pub airborne: bool,
    /// Paused, in replay, loading or wrecked: no judgement this tick.
    pub frozen: bool,
    /// A crew loop holds the aircraft, by name.
    pub owner: Option<&'static str>,
    /// The pilot said "my controls" (or a loop's yield saw a hand) recently.
    pub pilot_words: bool,
    /// Hands on the yoke or a yield latch, now.
    pub hands: bool,
    pub ap: ApRead,
    pub air: Air,
    pub terrain: Terrain,
    /// The runway end flown to, when on an approach.
    pub approach: Option<String>,
    /// The field whose circuit area we are low in ([`in_terminal`]).
    pub terminal: Option<String>,
    /// What the aircraft is doing no one may be handed (`recover::unflyable`).
    pub unflyable: Option<&'static str>,
    /// The airframe's climb, fpm.
    pub capability_fpm: f64,
    /// We may move the autopilot (crew mode, an autopilot, not a rotorcraft).
    pub actuate: bool,
    /// Our own panel changes (`autoflight::ours_since`), on this clock:
    /// `(now_s of the change, the change)`.
    pub ours: Vec<(f64, OurChange)>,
}

/// What the driver does this tick.
#[derive(Debug, Clone, PartialEq)]
pub enum Act {
    Nothing,
    /// Set the hold (said at Caution).
    Hold(HoldCmd),
    /// Set the hold now, the ground is near (said at Warning).
    Escape(HoldCmd),
    /// Say this; nothing moves.
    Warn(String),
    /// The recovery takes the aircraft, hand flown to this altitude.
    TakeOver {
        alt_ft: Option<f64>,
        why: &'static str,
    },
    /// The episode ended (a crew loop took it, the ground, a new flight):
    /// the driver lets go of anything of ours still running.
    End(&'static str),
}

#[derive(Debug, Clone)]
struct Held {
    at_s: f64,
    cmd: HoldCmd,
    alt_at_ft: f64,
    /// The autopilot read back after our write: the reference for the pilot.
    expect: Option<ApRead>,
}

#[derive(Debug, Clone)]
struct Episode {
    start_s: f64,
    cause: &'static str,
    baseline: ApRead,
    pilot: Option<&'static str>,
    unsafe_since: Option<f64>,
    held: Option<Held>,
    warned_at: Option<f64>,
    took_over: bool,
    last_verdict: &'static str,
    unflyable_since: Option<f64>,
}

/// The safe hold's state over one episode. Pure: [`SafeHold::step`] decides,
/// the driver acts and reports back ([`SafeHold::written`]).
#[derive(Debug, Clone, Default)]
pub struct SafeHold {
    ep: Option<Episode>,
    /// Episodes begun, for the first one's cause ("boot").
    episodes: u32,
}

impl SafeHold {
    /// The hold this episode set, if any.
    pub fn held(&self) -> Option<HoldCmd> {
        self.ep
            .as_ref()
            .and_then(|e| e.held.as_ref().map(|h| h.cmd))
    }

    /// The pilot evidence this episode latched, if any.
    #[cfg(test)]
    pub fn pilot(&self) -> Option<&'static str> {
        self.ep.as_ref().and_then(|e| e.pilot)
    }

    /// An episode is open (the aircraft is nobody's).
    pub fn open(&self) -> bool {
        self.ep.is_some()
    }

    /// The driver set the hold; `ap` is the autopilot read back after it.
    pub fn written(&mut self, cmd: HoldCmd, now_s: f64, alt_ft: f64, ap: Option<ApRead>) {
        if let Some(ep) = self.ep.as_mut() {
            ep.held = Some(Held {
                at_s: now_s,
                cmd,
                alt_at_ft: alt_ft,
                expect: ap,
            });
            ep.unsafe_since = None;
        }
    }

    /// The recovery has it.
    pub fn took_over(&mut self) {
        if let Some(ep) = self.ep.as_mut() {
            ep.took_over = true;
        }
    }

    /// One tick: what to do, and the records to blackbox (`safe_hold`).
    pub fn step(&mut self, t: &Tick) -> (Act, Vec<Value>) {
        let mut recs = Vec::new();
        // Paused, loading, a wreck, or a reading no flight has (a load
        // reports the ground 87 ft over the aircraft): no judgement.
        if t.frozen || (t.airborne && t.air.agl_ft < -50.0) {
            return (Act::Nothing, recs);
        }
        if !t.airborne || t.owner.is_some() {
            let why = if t.airborne { "crew" } else { "ground" };
            if let Some(ep) = self.ep.take() {
                recs.push(json!({ "event": "end", "why": why, "owner": t.owner,
                                  "held": ep.held.as_ref().map(|h| h.cmd.json()),
                                  "pilot": ep.pilot, "secs": r0(t.now_s - ep.start_s) }));
                return (Act::End(why), recs);
            }
            return (Act::Nothing, recs);
        }
        if self.ep.is_none() {
            let cause = if t.pilot_words {
                "pilot_release"
            } else if self.episodes == 0 {
                "boot"
            } else {
                "release"
            };
            self.episodes += 1;
            recs.push(json!({ "event": "episode", "cause": cause, "ap": t.ap.json(), "air": t.air.json() }));
            self.ep = Some(Episode {
                start_s: t.now_s,
                cause,
                baseline: t.ap,
                pilot: None,
                unsafe_since: None,
                held: None,
                warned_at: None,
                took_over: false,
                last_verdict: "",
                unflyable_since: None,
            });
        }
        let ep = self.ep.as_mut().expect("opened above");
        if ep.took_over {
            return (Act::Nothing, recs);
        }
        // The pilot's evidence, latched for the episode.
        if ep.pilot.is_none() {
            let reference = ep
                .held
                .as_ref()
                .and_then(|h| h.expect)
                .unwrap_or(ep.baseline);
            // Our changes since the reference (the write a beat before
            // `written` stamps it, hence the slack).
            let ref_s = ep.held.as_ref().map_or(ep.start_s, |h| h.at_s) - OURS_SLACK_S;
            let ours = Ours(t.ours.iter().filter(|(s, _)| *s >= ref_s).map(|(_, c)| *c).collect());
            let ev = if t.hands {
                Some("hands")
            } else {
                mcp_moved(&reference, &t.ap, ep.held.is_some(), t.air.alt_ft, &ours)
            };
            if let Some(e) = ev {
                ep.pilot = Some(e);
                recs.push(json!({ "event": "pilot", "evidence": e, "ap": t.ap.json(),
                                  "reference": reference.json(), "held": ep.held.is_some() }));
            }
        }
        let who = if t.hands {
            Who::Hands
        } else if t.ap.servos == Some(false) {
            Who::ApOff
        } else if ep.pilot.is_some() {
            Who::Mcp
        } else {
            Who::Unattended
        };
        let fl = floor(&t.air, &t.terrain);
        let floor_ind = t.air.indicated(fl.true_ft);
        let v = vertical(&t.ap, &t.air);
        // Coupled to a glideslope is an approach, whatever the runway data.
        let approach = t
            .approach
            .clone()
            .or_else(|| (v == Vertical::Glideslope).then(|| "glideslope".to_string()));
        let unsafe_now = if approach.is_some() {
            None
        } else {
            unsafe_why(v, &t.air, floor_ind, t.terrain.ahead.as_ref())
        };
        let terminal = t.terminal.is_some();
        let escape = approach.is_none() && escape_due(&t.air, t.terrain.ahead.as_ref(), terminal);
        let verdict = match (escape, unsafe_now, approach.is_some()) {
            (_, _, true) => "approach",
            (true, _, _) => "escape",
            (false, Some(_), _) => "unsafe",
            _ => "safe",
        };
        if verdict != ep.last_verdict {
            ep.last_verdict = verdict;
            recs.push(
                json!({ "event": "verdict", "verdict": verdict, "who": who.word(),
                              "why": unsafe_now.map(Why::word), "vertical": format!("{v:?}"),
                              "floor_ft": r0(fl.true_ft), "floor_ind_ft": r0(floor_ind),
                              "floor_source": fl.source, "approach": approach,
                              "terminal": t.terminal,
                              "air": t.air.json(), "ap": t.ap.json() }),
            );
        }
        let warn_due = |ep: &Episode| ep.warned_at.is_none_or(|w| t.now_s - w >= WARN_REPEAT_S);
        // Hands or the servos out: a human holds the controls. Words only.
        if matches!(who, Who::Hands | Who::ApOff) {
            if escape && warn_due(ep) {
                ep.warned_at = Some(t.now_s);
                recs.push(json!({ "event": "warn", "who": who.word(), "air": t.air.json() }));
                return (Act::Warn(TERRAIN_PULL_UP.to_string()), recs);
            }
            return (Act::Nothing, recs);
        }
        // Unattended and unflyable for [`UNFLYABLE_S`]: the recovery, never
        // nobody (a tick of stall warning in a gust is not an upset).
        match t.unflyable.filter(|_| who == Who::Unattended) {
            None => ep.unflyable_since = None,
            Some(_) => {
                ep.unflyable_since.get_or_insert(t.now_s);
            }
        }
        if t.actuate
            && ep
                .unflyable_since
                .is_some_and(|s| t.now_s - s >= UNFLYABLE_S)
        {
            if let Some(u) = t.unflyable {
                ep.took_over = true;
                recs.push(json!({ "event": "take_over", "why": u, "air": t.air.json() }));
                return (
                    Act::TakeOver {
                        alt_ft: Some(up_100(floor_ind.max(t.air.alt_ft))),
                        why: u,
                    },
                    recs,
                );
            }
        }
        let cmd_for = |why: Why| hold_cmd(why, &t.air, &t.ap, fl, &t.terrain, t.capability_fpm);
        if escape {
            let recent_climb = ep
                .held
                .as_ref()
                .is_some_and(|h| h.cmd.climb && t.now_s - h.at_s < VERIFY_S);
            if t.actuate && !recent_climb {
                let cmd = escape_cmd(cmd_for(unsafe_now.unwrap_or(Why::HoldsBelow)), t.air.alt_ft);
                recs.push(
                    json!({ "event": "escape", "who": who.word(), "cmd": cmd.json(),
                                  "air": t.air.json(), "ap": t.ap.json() }),
                );
                return (Act::Escape(cmd), recs);
            }
            if !t.actuate && warn_due(ep) {
                ep.warned_at = Some(t.now_s);
                recs.push(json!({ "event": "warn", "who": who.word(), "air": t.air.json() }));
                return (Act::Warn(TERRAIN_PULL_UP.to_string()), recs);
            }
        }
        if who == Who::Mcp {
            // The pilot flies the autopilot: told once (never in a field's
            // circuit, where low is the point), never overruled short of the
            // escape.
            if let Some(why) = unsafe_now.filter(|_| !terminal) {
                if ep.warned_at.is_none() {
                    ep.warned_at = Some(t.now_s);
                    recs.push(json!({ "event": "warn", "who": "mcp", "why": why.word(),
                                      "floor_ind_ft": r0(floor_ind) }));
                    return (Act::Warn(below_floor_line(up_100(floor_ind))), recs);
                }
            }
            return (Act::Nothing, recs);
        }
        // Unattended.
        if let Some(h) = ep.held.clone() {
            let since = t.now_s - h.at_s;
            // The autopilot's answer: a climb that does not climb, a bound
            // the descent flies through.
            let not_climbing = h.cmd.climb
                && since >= VERIFY_S
                && t.air.vs_fpm < 0.0
                && t.air.alt_ft <= h.alt_at_ft + 100.0;
            let through_bound =
                !h.cmd.climb && t.air.alt_ft < h.cmd.alt_ft - 300.0 && t.air.vs_fpm < -500.0;
            if t.actuate && (not_climbing || through_bound) {
                ep.took_over = true;
                let why = if not_climbing {
                    "hold_not_climbing"
                } else {
                    "hold_flown_through"
                };
                recs.push(
                    json!({ "event": "take_over", "why": why, "cmd": h.cmd.json(),
                                  "air": t.air.json(), "ap": t.ap.json() }),
                );
                return (
                    Act::TakeOver {
                        alt_ft: Some(h.cmd.alt_ft),
                        why,
                    },
                    recs,
                );
            }
            // Higher ground found ahead since: the hold is raised.
            if t.actuate && since >= REPLAN_S && up_100(floor_ind) > h.cmd.alt_ft + 200.0 {
                let cmd = cmd_for(Why::HoldsBelow);
                recs.push(json!({ "event": "raise", "was": h.cmd.json(), "cmd": cmd.json() }));
                return (Act::Hold(cmd), recs);
            }
            return (Act::Nothing, recs);
        }
        let Some(why) = unsafe_now else {
            ep.unsafe_since = None;
            return (Act::Nothing, recs);
        };
        let since = *ep.unsafe_since.get_or_insert(t.now_s);
        if t.now_s - since < CONFIRM_S {
            return (Act::Nothing, recs);
        }
        if !t.actuate {
            if ep.warned_at.is_none() {
                ep.warned_at = Some(t.now_s);
                recs.push(
                    json!({ "event": "warn", "who": "unattended", "why": why.word(),
                                  "floor_ind_ft": r0(floor_ind), "actuate": false }),
                );
                return (Act::Warn(below_floor_line(up_100(floor_ind))), recs);
            }
            return (Act::Nothing, recs);
        }
        let cmd = cmd_for(why);
        recs.push(
            json!({ "event": "hold", "cause": ep.cause, "cmd": cmd.json(),
                          "vertical": format!("{v:?}"), "air": t.air.json(), "ap": t.ap.json() }),
        );
        (Act::Hold(cmd), recs)
    }
}

/// The warning to a pilot hand flying toward the ground.
pub const TERRAIN_PULL_UP: &str = "Terrain! Pull up.";

/// The advisory to a pilot flying the autopilot below the floor.
pub fn below_floor_line(floor_ft: f64) -> String {
    format!(
        "Terrain ahead. The autopilot is taking us below {}, the safe altitude here.",
        crate::flightplan::spoken_feet(floor_ft)
    )
}

/// The line the hold says, with what it set and how to hand on.
pub fn hold_line(cmd: &HoldCmd, escape: bool, cause: &str) -> String {
    let feet = crate::flightplan::spoken_feet(cmd.alt_ft);
    let heading = cmd
        .heading_mag
        .map(|h| format!(", heading {}", spoken_heading(h)))
        .unwrap_or_default();
    let lead = match (escape, cmd.climb, cause) {
        (true, _, _) => "Terrain!".to_string(),
        (false, _, "boot") => "Nobody was flying the autopilot's last command.".to_string(),
        (false, true, _) => "We're below the safe altitude for the ground ahead.".to_string(),
        (false, false, _) => "That descent had nothing to stop it.".to_string(),
    };
    let what = if cmd.climb {
        format!("Climbing to {feet}{heading}")
    } else {
        format!("Levelling at {feet}, the safe altitude here{heading}")
    };
    format!("{lead} {what}. Say fly the flight plan, or my controls.")
}

fn spoken_heading(deg: f64) -> String {
    let d = (deg.round() as i64).rem_euclid(360);
    format!("{:03}", if d == 0 { 360 } else { d })
}

fn r0(v: f64) -> f64 {
    v.round()
}
fn r4(v: f64) -> f64 {
    (v * 10_000.0).round() / 10_000.0
}

// ------------------------------------------------------------ the driver

/// The points along a bearing, `n` evenly out to `look_nm`.
pub fn ray_points(
    lat: f64,
    lon: f64,
    brg_true: f64,
    look_nm: f64,
    n: usize,
) -> Vec<(f64, f64, f64)> {
    let n = n.max(1);
    (1..=n)
        .map(|i| {
            let d = look_nm * i as f64 / n as f64;
            let (la, lo) = crate::nav::project_latlon(lat, lon, brg_true, d);
            (d, la, lo)
        })
        .collect()
}

/// Probe one ray. `None` = the probe did not answer.
pub async fn probe_ray(
    xp: &crate::xplane::XPlaneClient,
    lat: f64,
    lon: f64,
    brg_true: f64,
    look_nm: f64,
    n: usize,
) -> Option<Ray> {
    let pts = ray_points(lat, lon, brg_true, look_nm, n);
    let ll: Vec<(f64, f64)> = pts.iter().map(|&(_, a, b)| (a, b)).collect();
    let mut ground = Vec::with_capacity(ll.len());
    for chunk in ll.chunks(crate::terrain::MAX_POINTS) {
        ground.extend(crate::terrain::sample_ft(xp, chunk).await?);
    }
    Some(Ray {
        bearing_true: brg_true.rem_euclid(360.0),
        ground_ft: pts.iter().map(|p| p.0).zip(ground).collect(),
    })
}

/// Probe the fan round the track ([`FAN_OFFSETS_DEG`]) in as few mailbox
/// transactions as the points allow. Empty when the probe does not answer.
pub async fn probe_fan(xp: &crate::xplane::XPlaneClient, air: &Air) -> Vec<Ray> {
    let rays: Vec<(f64, Vec<(f64, f64, f64)>)> = FAN_OFFSETS_DEG
        .iter()
        .map(|off| {
            let brg = (air.track_true + off).rem_euclid(360.0);
            (
                brg,
                ray_points(air.lat, air.lon, brg, FAN_LOOK_NM, FAN_POINTS),
            )
        })
        .collect();
    let ll: Vec<(f64, f64)> = rays
        .iter()
        .flat_map(|(_, p)| p.iter().map(|&(_, a, b)| (a, b)))
        .collect();
    let mut ground = Vec::with_capacity(ll.len());
    for chunk in ll.chunks(crate::terrain::MAX_POINTS) {
        match crate::terrain::sample_ft(xp, chunk).await {
            Some(g) => ground.extend(g),
            None => return Vec::new(),
        }
    }
    split_fan(&rays, &ground)
}

/// The fan's flat answer back into its rays. Pure.
pub fn split_fan(rays: &[(f64, Vec<(f64, f64, f64)>)], ground: &[f64]) -> Vec<Ray> {
    let mut at = 0;
    let mut out = Vec::new();
    for (brg, pts) in rays {
        let Some(g) = ground.get(at..at + pts.len()) else {
            break;
        };
        out.push(Ray {
            bearing_true: *brg,
            ground_ft: pts.iter().map(|p| p.0).zip(g.iter().copied()).collect(),
        });
        at += pts.len();
    }
    out
}

/// The grid MORA, loaded once per process.
pub fn mora_grid() -> Option<&'static crate::nav::MoraGrid> {
    static GRID: std::sync::OnceLock<Option<crate::nav::MoraGrid>> = std::sync::OnceLock::new();
    GRID.get_or_init(|| {
        crate::nav::MoraGrid::load(&format!(
            "{}/Resources/default data/earth_mora.dat",
            crate::tools::autoland::xplane_root()
        ))
    })
    .as_ref()
}

/// The MORA here and [`LOOK_NM`] ahead, the higher.
pub fn mora_ahead(air: &Air) -> Option<f64> {
    let g = mora_grid()?;
    let (la, lo) = crate::nav::project_latlon(air.lat, air.lon, air.track_true, LOOK_NM);
    [g.at(air.lat, air.lon), g.at(la, lo)]
        .into_iter()
        .flatten()
        .reduce(f64::max)
}

const HPATH: &str = "sim/flightmodel/position/hpath";
const MAG_VAR: &str = "sim/flightmodel/position/magnetic_variation";
const YOKE_PITCH: &str = "sim/joystick/yoke_pitch_ratio";
const YOKE_ROLL: &str = "sim/joystick/yoke_roll_ratio";
const PITCH_YIELD: &str = "xcopilot/pitch_yield";
const STEER_YIELD: &str = "xcopilot/steer_yield";
const STALL_RATIO: &str = "sim/cockpit2/annunciators/stall_warning_ratio";

async fn read_ap(xp: &crate::xplane::XPlaneClient) -> ApRead {
    use crate::datarefs as d;
    let ok = |r: anyhow::Result<f64>| r.ok().filter(|v| v.is_finite());
    let (servos, state, alt, vs, hdg) = tokio::join!(
        xp.get_float(d::SERVOS_ON),
        xp.get_float(crate::autoflight::names::AP_STATE),
        crate::autoflight::read_dial(xp, crate::autoflight::Dial::Altitude),
        crate::autoflight::read_dial(xp, crate::autoflight::Dial::Vs),
        crate::autoflight::read_dial(xp, crate::autoflight::Dial::HeadingBug),
    );
    ApRead {
        servos: ok(servos).map(|v| v > 0.5),
        state: ok(state).and_then(Modes::from_read),
        alt_dial_ft: ok(alt),
        vs_dial_fpm: ok(vs),
        hdg_bug_mag: ok(hdg),
    }
}

/// The aircraft, the hands and the stall warning; `None` on a failed read of
/// anything the decision rests on (no measurement, no verdict).
async fn read_air(xp: &dyn crate::xplane::XPlane) -> Option<(Air, bool, Option<f64>)> {
    use crate::datarefs as d;
    // Position, height, groundspeed and vertical speed are ONE frame
    // (`sim::position`, #157); the track and the rest ride beside it.
    let (pos, alt, ias, hpath, mag_var) = tokio::join!(
        sim::position::read(xp),
        xp.get_float(d::ALTITUDE_FT),
        xp.get_float(d::AIRSPEED_KTS),
        xp.get_float(HPATH),
        xp.get_float(MAG_VAR),
    );
    let (lat, lon) = sim::position::lat_lon(&pos);
    let elev = sim::position::field(&pos, |p| p.elev_m);
    let agl = sim::position::field(&pos, |p| p.agl_m);
    let vs = sim::position::field(&pos, |p| p.vs_fpm);
    let gs = sim::position::field(&pos, |p| p.gs_ms);
    let (bank, pitch, flap, yp, yr, pyield, syield, stall) = tokio::join!(
        xp.get_float(d::BANK_DEG),
        xp.get_float(d::PITCH_DEG),
        xp.get_float(d::FLAP_RATIO),
        xp.get_float(YOKE_PITCH),
        xp.get_float(YOKE_ROLL),
        xp.get_float(PITCH_YIELD),
        xp.get_float(STEER_YIELD),
        xp.get_float(STALL_RATIO),
    );
    let [lat, lon, alt, elev, agl, vs, ias, gs, hpath] =
        crate::control::measurements("safe_hold", [lat, lon, alt, elev, agl, vs, ias, gs, hpath])?;
    let ok = |r: anyhow::Result<f64>| r.ok().filter(|v| v.is_finite());
    let yoke = [ok(yp), ok(yr)]
        .into_iter()
        .flatten()
        .any(|v| v.abs() > YOKE_HANDS)
        && !crate::monitor::crew_yoke_fresh();
    let latch = [ok(pyield), ok(syield)]
        .into_iter()
        .flatten()
        .any(|v| v > 0.5);
    let air = Air {
        lat,
        lon,
        alt_ft: alt,
        alt_true_ft: elev * 3.280_84,
        agl_ft: agl * 3.280_84,
        vs_fpm: vs,
        ias_kts: ias,
        gs_kts: gs * 1.943_844,
        track_true: hpath.rem_euclid(360.0),
        mag_var: ok(mag_var).unwrap_or(0.0),
        bank_deg: ok(bank).unwrap_or(0.0),
        pitch_deg: ok(pitch).unwrap_or(0.0),
        flap: ok(flap).unwrap_or(0.0),
    };
    Some((air, yoke || latch, ok(stall)))
}

/// The runway ends near a point, from apt.dat, for the approach test.
fn ends_near(lat: f64, lon: f64) -> Vec<End> {
    crate::runway::nearby_runways(
        &crate::tools::autoland::apt_dat_path(),
        lat,
        lon,
        APPROACH_NM + 3.0,
    )
    .unwrap_or_default()
    .into_iter()
    .map(|r| End {
        ident: format!("{} {}", r.icao.trim(), r.id),
        thr_lat: r.thr_lat,
        thr_lon: r.thr_lon,
        course_true: r.course_deg,
        elev_ft: r.elev_ft,
        length_m: r.length_m,
    })
    .collect()
}

/// Seconds between terrain probes along the track, and the track change and
/// distance that ask for one sooner.
const PROBE_S: f64 = 10.0;
const PROBE_TURN_DEG: f64 = 15.0;
const PROBE_MOVED_NM: f64 = 3.0;
/// The runway ends are read again this far from where they were read, nm.
const ENDS_MOVED_NM: f64 = 6.0;

/// Start the safe hold's watch: its own thread, runtime and client, for the
/// life of the process (a stand-down is exactly when it is needed, so no run
/// stands it down). Reads nothing but the owner flag while a loop flies.
pub fn spawn(
    config: crate::config::Config,
    live: std::sync::Arc<crate::detect::LiveProfile>,
    voice: crate::voice::Voice,
    status: Option<crate::plugin_status::StatusSink>,
) {
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(watch(config, live, voice, status));
    });
}

async fn watch(
    config: crate::config::Config,
    live: std::sync::Arc<crate::detect::LiveProfile>,
    voice: crate::voice::Voice,
    status: Option<crate::plugin_status::StatusSink>,
) {
    use std::time::{Duration, Instant};
    let xp = crate::xplane::XPlaneClient::new(config.xplane_base_url.clone());
    let clock = Instant::now();
    let mut hold = SafeHold::default();
    let mut gen = live.generation();
    // The last probe along the track: when, from where, on what track, and
    // its answer (`None` = the probe did not answer: MORA stands in).
    let mut ahead: Option<(f64, f64, f64, f64, Option<Ray>)> = None;
    let mut ends: Option<(f64, f64, Vec<End>)> = None;
    let mut fan: Vec<Ray> = Vec::new();
    loop {
        tokio::time::sleep(Duration::from_millis(1_000)).await;
        let now_s = clock.elapsed().as_secs_f64();
        if live.generation() != gen {
            // A new airframe is a new flight: nothing carries over.
            gen = live.generation();
            hold = SafeHold::default();
            hold.episodes = 1;
            ahead = None;
            ends = None;
        }
        // A rotorcraft flies its own hover loop and hands back on its own
        // terms (`hover::hand_back`); its low flight is no terrain conflict.
        let rotorcraft = live.get().rotorcraft;
        let owner = crate::flight_state::crew_has_aircraft()
            .map(|o| o.name())
            .or(rotorcraft.then_some("rotorcraft"));
        let loading = crate::tools::setup::new_flight_loading() || crate::xplane::sim_loading();
        // A loop owns it, and the hold has nothing open: read nothing.
        if owner.is_some() && !hold.open() {
            continue;
        }
        let ground = xp.get_float(crate::datarefs::ON_GROUND_ANY).await.ok();
        let airborne = ground.is_some_and(|g| g < 0.5);
        let frozen = airborne
            && (loading
                || xp
                    .get_float(crate::datarefs::HAS_CRASHED)
                    .await
                    .is_ok_and(|v| v > 0.5)
                || crate::xplane::sim_frozen(&xp).await);
        let mut tick = Tick {
            now_s,
            airborne,
            frozen,
            owner,
            // "My controls" through the release guard or the autopilot-off
            // tool (the cancel of an autoland or a recovery).
            pilot_words: crate::monitor::pilot_released_recently()
                || crate::flight::pilot_takeover().is_some_and(|(t, _)| {
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .map(|n| n.as_secs_f64() - t <= RELEASE_WORDS_S)
                        .unwrap_or(false)
                }),
            hands: false,
            ap: ApRead::default(),
            air: Air::default(),
            terrain: Terrain::default(),
            approach: None,
            terminal: None,
            unflyable: None,
            capability_fpm: 0.0,
            actuate: false,
            // Our own panel changes, for telling the pilot's from ours.
            ours: crate::autoflight::ours_since(
                Instant::now()
                    .checked_sub(Duration::from_secs_f64(crate::autoflight::LEDGER_S))
                    .unwrap_or(clock),
            )
            .into_iter()
            .map(|(t, c)| (t.saturating_duration_since(clock).as_secs_f64(), c))
            .collect(),
        };
        if airborne && owner.is_none() && !frozen {
            let Some((air, hands, stall)) = read_air(&xp).await else {
                continue;
            };
            let ap = read_ap(&xp).await;
            // The ground ahead: on a clock, a turn, or a move.
            let stale = ahead.as_ref().is_none_or(|(at, la, lo, trk, _)| {
                now_s - at >= PROBE_S
                    || crate::angles::abs_diff_deg(*trk, air.track_true) > PROBE_TURN_DEG
                    || crate::nav::haversine_nm(*la, *lo, air.lat, air.lon) > PROBE_MOVED_NM
            });
            if stale {
                let ray =
                    probe_ray(&xp, air.lat, air.lon, air.track_true, LOOK_NM, LOOK_POINTS).await;
                ahead = Some((now_s, air.lat, air.lon, air.track_true, ray));
                fan.clear();
            }
            if ends.as_ref().is_none_or(|(la, lo, _)| {
                crate::nav::haversine_nm(*la, *lo, air.lat, air.lon) > ENDS_MOVED_NM
            }) {
                let (la, lo) = (air.lat, air.lon);
                let found = tokio::task::spawn_blocking(move || ends_near(la, lo))
                    .await
                    .unwrap_or_default();
                ends = Some((la, lo, found));
            }
            let profile = live.get();
            let floor_kts = crate::aircraft::config_floor_kts(air.flap)
                .or_else(crate::aircraft::speed_floor_kts)
                .map(|f| f * 0.95);
            tick.unflyable = crate::tools::recover::unflyable(
                Some(air.ias_kts),
                floor_kts,
                Some(air.pitch_deg),
                Some(air.bank_deg),
                stall,
            );
            tick.capability_fpm =
                crate::tools::autopilot::ClimbPerf::from_profile(&profile).max_climb_fpm;
            tick.actuate = crate::mode::crew_actuates() && profile.autopilot;
            tick.approach = ends.as_ref().and_then(|(_, _, e)| on_approach(e, &air));
            tick.terminal = ends.as_ref().and_then(|(_, _, e)| in_terminal(e, &air));
            tick.hands = hands;
            tick.ap = ap;
            tick.air = air;
            tick.terrain = Terrain {
                ahead: ahead.as_ref().and_then(|a| a.4.clone()),
                fan: fan.clone(),
                mora_ft: mora_ahead(&air),
            };
            // A climb that the ground ahead may outrun asks the fan first.
            if tick.actuate && fan.is_empty() && hold.held().is_none() {
                let fl = floor(&air, &tick.terrain);
                let below = air.alt_ft < air.indicated(fl.true_ft) - FLOOR_TOL_FT;
                let grad = gradient_ft_nm(tick.capability_fpm, air.gs_kts);
                let short = tick
                    .terrain
                    .ahead
                    .as_ref()
                    .is_some_and(|r| !climb_clears(r, air.alt_true_ft, grad, false));
                if below && short {
                    fan = probe_fan(&xp, &air).await;
                    tick.terrain.fan = fan.clone();
                }
            }
        }
        let (act, recs) = hold.step(&tick);
        for r in recs {
            crate::blackbox::record("safe_hold", r);
        }
        match act {
            Act::Nothing => {}
            Act::End(_) => {
                // Our capture watcher must not write under the loop that
                // took the aircraft (one writer per dataref).
                if owner.is_some() {
                    crate::tools::autopilot::CAPTURE.cancel();
                }
                fan.clear();
            }
            Act::Warn(line) => {
                say(&voice, &status, crate::voice::Priority::Warning, &line);
            }
            Act::TakeOver { alt_ft, why } => {
                hold.took_over();
                let taken = crate::tools::recover::take_over(
                    config.xplane_base_url.clone(),
                    Some(voice.clone()),
                    crate::tools::recover::TakeOver {
                        why: format!("safe_hold_{why}"),
                        min_safe_ias: None,
                        hold_kts: None,
                        ap_no_pitch: why.starts_with("hold_"),
                        hold_ft: alt_ft,
                    },
                );
                if taken {
                    say(
                        &voice,
                        &status,
                        crate::voice::Priority::Warning,
                        &take_over_line(alt_ft),
                    );
                }
            }
            Act::Hold(cmd) | Act::Escape(cmd) => {
                let escape = matches!(act, Act::Escape(_));
                let cause = hold.ep.as_ref().map(|e| e.cause).unwrap_or("release");
                let set = set_hold(&xp, &config, &live, &voice, &status, &tick, &cmd).await;
                let ap_after = read_ap(&xp).await;
                let took = ap_after.servos == Some(true)
                    && ap_after
                        .alt_dial_ft
                        .is_some_and(|d| (d - cmd.alt_ft).abs() <= 50.0);
                crate::blackbox::record(
                    "safe_hold",
                    json!({ "event": "set", "escape": escape, "cmd": cmd.json(), "took": took,
                            "replies": set, "ap_after": ap_after.json() }),
                );
                hold.written(cmd, now_s, tick.air.alt_ft, Some(ap_after));
                if took {
                    let prio = if escape {
                        crate::voice::Priority::Warning
                    } else {
                        crate::voice::Priority::Caution
                    };
                    say(&voice, &status, prio, &hold_line(&cmd, escape, cause));
                } else {
                    // The autopilot did not take it: nobody may be left
                    // flying a descent. The recovery flies it to the floor.
                    hold.took_over();
                    let taken = crate::tools::recover::take_over(
                        config.xplane_base_url.clone(),
                        Some(voice.clone()),
                        crate::tools::recover::TakeOver {
                            why: "safe_hold_not_set".into(),
                            min_safe_ias: None,
                            hold_kts: None,
                            ap_no_pitch: true,
                            hold_ft: Some(cmd.alt_ft),
                        },
                    );
                    crate::blackbox::record(
                        "safe_hold",
                        json!({ "event": "take_over", "why": "not_set", "taken": taken }),
                    );
                    if taken {
                        say(
                            &voice,
                            &status,
                            crate::voice::Priority::Warning,
                            &take_over_line(Some(cmd.alt_ft)),
                        );
                    }
                }
            }
        }
    }
}

/// The line when the recovery takes the aircraft for the hold.
pub fn take_over_line(alt_ft: Option<f64>) -> String {
    match alt_ft {
        Some(a) => format!(
            "The autopilot isn't holding it. I have the aircraft, flying it to {}. Say my controls to take it back.",
            crate::flightplan::spoken_feet(a)
        ),
        None => "I have the aircraft. Say my controls to take it back.".to_string(),
    }
}

fn say(
    voice: &crate::voice::Voice,
    status: &Option<crate::plugin_status::StatusSink>,
    prio: crate::voice::Priority,
    line: &str,
) {
    // `Voice::alert` mirrors to the panel transcript itself; the action line
    // is the panel's status strip.
    if let Some(s) = status {
        s.action(line);
    }
    voice.alert(prio, line.to_string());
}

/// Set the hold through the panel's owner (`autoflight`, the safe hold its
/// writer only while no loop has the aircraft): the heading first (so the
/// climb is flown on it), the speed held by the governor at the
/// configuration's hand-back speed when no autothrottle holds it and the
/// governor is idle, then the climb or the bounded descent to the level,
/// captured on the window. Returns what each step read back, for the record.
async fn set_hold(
    xp: &crate::xplane::XPlaneClient,
    config: &crate::config::Config,
    live: &crate::detect::LiveProfile,
    _voice: &crate::voice::Voice,
    _status: &Option<crate::plugin_status::StatusSink>,
    tick: &Tick,
    cmd: &HoldCmd,
) -> Vec<String> {
    use crate::autoflight::{climb_to, hold_heading, SAFE_HOLD};
    let profile = live.get();
    if crate::tools::ap_config(&profile).is_none() {
        return vec!["no autopilot".into()];
    }
    let mut replies = Vec::new();
    if let Some(h) = cmd.heading_mag {
        let ok = hold_heading(xp, h.round(), SAFE_HOLD).await;
        replies.push(format!("heading {:.0} {}", h.round(), if ok { "held" } else { "not held" }));
    }
    // The speed: a hold levels off, and a level-off at the descent's idle
    // decays toward the stall. Held at the configuration's hand-back speed
    // unless the autothrottle (the pilot's or the aircraft's) holds it.
    if !tick.ap.autothrottle() && crate::tools::speedhold::flight_target().is_none() {
        let kts = crate::autoland::hand_back_speed_kts(
            crate::tools::approach_ias(&profile),
            tick.air.flap,
            &crate::aircraft::flap_limits(),
        );
        crate::tools::speedhold::arm_flight(config.xplane_base_url.clone(), None, kts);
        replies.push(format!("governor {kts:.0}"));
    }
    // Toward the level: a climb's rate up, a bounded descent's down.
    let fpm = if cmd.climb { cmd.fpm.abs() } else { -cmd.fpm.abs() };
    let ok = climb_to(xp, cmd.alt_ft, fpm.round(), SAFE_HOLD).await;
    replies.push(format!("level {:.0} at {:.0} {}", cmd.alt_ft, fpm, if ok { "set" } else { "not set" }));
    replies
}

#[cfg(test)]
mod tests;

/// The real 2026-10-05 LEMG records, replayed through the decision.
#[cfg(test)]
mod lemg_replay_tests;
