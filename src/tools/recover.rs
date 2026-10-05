//! `recover` — take the controls and fly the aircraft out of an upset into a
//! stable, wings-level state, then hand it to the autopilot.
//!
//! The autopilot can't do this: in a stall it holds the nose UP (deepening the
//! stall), and it won't roll out of a steep bank. So this takes direct control of
//! pitch and roll (`override_joystick_*` + `yoke_*_ratio`), runs a conservative
//! **unstall → roll upright → pitch to level** sequence in a background loop, and
//! once stable engages ALT hold + wing-level and releases the controls.
//!
//! SAFETY: the control law is deliberately gentle (low gains, rate-damped, hard
//! clamped) — it needs in-sim tuning and should be tested with altitude to spare.
//! The pilot takes it back instantly with "my controls" (cancels its run, [`RUN`]).

use crate::autoflight::{self, Button, Dial};
use std::sync::atomic::{AtomicBool, Ordering};

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::tools::autopilot::ApConfig;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

// --- controls (written under override) ---
const OVR_PITCH: &str = "sim/operation/override/override_joystick_pitch";
const OVR_ROLL: &str = "sim/operation/override/override_joystick_roll";
const YOKE_PITCH: &str = "sim/joystick/yoke_pitch_ratio"; // +1 nose up, -1 nose down
const YOKE_ROLL: &str = "sim/joystick/yoke_roll_ratio"; // +1 right, -1 left
/// The thrust lever THIS airframe obeys — not always the stock one (the
/// Citation X tracks `throttle_jet_rev_ratio`, so every write to
/// `throttle_ratio_all` went nowhere and it flew a whole pattern at takeoff
/// thrust). Resolved per call, so a profile swap takes effect immediately.
fn throttle_dref() -> String {
    crate::aircraft::throttle_dataref()
}

// --- state reads ---
const BANK: &str = "sim/flightmodel/position/phi"; // roll, deg (+ = right wing down)
const PITCH: &str = "sim/flightmodel/position/theta"; // pitch, deg (+ = nose up)
const ROLL_RATE: &str = "sim/flightmodel/position/P"; // deg/s
const PITCH_RATE: &str = "sim/flightmodel/position/Q"; // deg/s
const IAS: &str = "sim/flightmodel/position/indicated_airspeed"; // kt
const VS_FPM: &str = "sim/flightmodel/position/vh_ind_fpm";
const STALL_RATIO: &str = "sim/cockpit2/annunciators/stall_warning_ratio"; // 0..1
const ELEV_TRIM: &str = "sim/cockpit2/controls/elevator_trim"; // -1 nose down .. +1 nose up

// --- AP handoff (mirror of autopilot.rs constants) ---
const CURRENT_ALT: &str = "sim/cockpit2/gauges/indicators/altitude_ft_pilot";
/// The shared gauge name: the local spelling `compass_heading_deg_magnetic`
/// does not exist in X-Plane 12, so every read failed and the bug after a
/// recovery went to ZERO (the `unwrap_or`), turning the aircraft north.
const CURRENT_HDG: &str = crate::datarefs::COMPASS_HEADING_DEG;
const HEADING_STATUS: &str = autoflight::names::HDG_STATUS;
const ALT_HOLD_STATUS: &str = autoflight::names::ALT_HOLD_STATUS;

/// Cruise power to settle at after recovery; firewall power while unstalling.
const CRUISE_THROTTLE: f64 = 0.6;
const RECOVERY_THROTTLE: f64 = 0.95;
/// Above this stall-warning ratio, treat it as stalling and unstall first.
const STALL_TRIP: f64 = 0.15;

/// True while a recovery is actively flying — so "my controls" can acknowledge it
/// ("you have the aircraft") rather than fall through to the AP-off readback.
/// Written only by the run ([`RUN`]).
pub static ACTIVE: AtomicBool = AtomicBool::new(false);
/// The recovery's run: claimed by the tool and by a crew loop's take-over
/// (refused while one flies), cancelled by "my controls" and a stand-down.
/// A recovery started while a cancelled one still winds down supersedes it,
/// and the old one can no longer write or clear the new one's flag.
pub(crate) static RUN: sim::run::Slot = sim::run::Slot::flagged(&ACTIVE);

/// The live state the control law reacts to.
#[derive(Clone, Copy, Debug)]
pub struct RecoveryState {
    pub bank_deg: f64,
    pub pitch_deg: f64,
    pub roll_rate_dps: f64,
    pub pitch_rate_dps: f64,
    pub ias_kts: f64,
    /// Stall-warning ratio, 0 (clean) .. 1 (full stall).
    pub stall_ratio: f64,
    /// The slowest airspeed to hold during recovery (≈1.3×Vs) — below it the law
    /// commands full power; well above it the power comes back to cruise.
    pub min_safe_ias: f64,
    /// The attitude the pitch-to-level phase aims at, degrees (level, or a
    /// hand-flown hold's correction toward its height).
    pub target_pitch_deg: f64,
    /// Height above the ground, feet; `None` unread (no ground rule).
    pub agl_ft: Option<f64>,
    /// Vertical speed, feet per minute; `None` unread.
    pub vs_fpm: Option<f64>,
}

/// Low and going down: the recovery climbs away from the ground, not just
/// to level (a 737 unpaused at 200 ft, 7 degrees nose down, was eased up at
/// 0.27 of column and struck the ground, LEMG 13 2026-09-29; #181).
pub const GROUND_ESCAPE_AGL_FT: f64 = 1_000.0;
const GROUND_ESCAPE_PITCH_DEG: f64 = 10.0;
const GROUND_ESCAPE_PULL: f64 = 0.6;

pub fn ground_escape(s: &RecoveryState) -> bool {
    match (s.agl_ft, s.vs_fpm) {
        (Some(agl), Some(vs)) => agl < GROUND_ESCAPE_AGL_FT && (vs < -200.0 || s.pitch_deg < -2.0),
        _ => false,
    }
}

/// What to command this tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecoveryCmd {
    pub pitch: f64,
    pub roll: f64,
    /// Set the throttle this tick, if the phase calls for it.
    pub throttle: Option<f64>,
    /// Recovery complete — hand off to the autopilot.
    pub stable: bool,
}

/// The recovery control law. Pure, so it's testable without a sim. Priorities:
/// unstall first (push to reduce AoA, gentle wings, power up), then roll upright
/// and pitch to level — all rate-damped (PD) and hard-clamped so it can't snatch.
pub fn recovery_inputs(s: &RecoveryState) -> RecoveryCmd {
    // Gains — conservative; tune live against the "recover" log line.
    const KP_ROLL: f64 = 0.014; // per deg bank
    const KD_ROLL: f64 = 0.010; // per deg/s roll rate (damping)
    const KP_PITCH: f64 = 0.030; // per deg pitch error
    const KD_PITCH: f64 = 0.015; // per deg/s pitch rate (damping)
    const STALL_PUSH: f64 = 0.55; // nose-down authority to break a stall (decisive)

    // Roll toward wings level: a right bank (phi > 0) needs left yoke (negative).
    // Damp with roll rate so it doesn't overshoot through level.
    let roll = -(KP_ROLL * s.bank_deg + KD_ROLL * s.roll_rate_dps);

    if s.stall_ratio > STALL_TRIP {
        // Unstall: push the nose down (reduce AoA), keep wings only gently level
        // (aggressive aileron near the stall risks a wing drop / spin), full power.
        // The push scales with how deep the stall is.
        // Steeply banked, the push has unloaded the wing, so the roll gets full
        // authority (upset recovery: push, ROLL, power): a 737 at 103 degrees
        // held to 0.25 rolled on through inverted, 700 ft lost (LEMG 13
        // 2026-09-29, a mouse yoke's full back and left; #181).
        let push = -STALL_PUSH * (s.stall_ratio.clamp(0.0, 1.0)).max(0.4);
        let cap = stall_roll_cap(s.bank_deg);
        return RecoveryCmd {
            pitch: push.clamp(-STALL_PUSH, 0.0),
            roll: roll.clamp(-cap, cap),
            throttle: Some(RECOVERY_THROTTLE),
            stable: false,
        };
    }

    // Flying: roll upright and pitch to level, and keep the energy up — full power
    // while below the safe speed, otherwise leave the throttle where the handoff
    // will set cruise. (Props are driven to full forward once at recovery start.)
    let escape = ground_escape(s);
    let target = if escape {
        s.target_pitch_deg.max(GROUND_ESCAPE_PITCH_DEG)
    } else {
        s.target_pitch_deg
    };
    let pitch = KP_PITCH * (target - s.pitch_deg) - KD_PITCH * s.pitch_rate_dps;
    let pull_max = if escape { GROUND_ESCAPE_PULL } else { 0.4 };
    let throttle = (escape || s.ias_kts < s.min_safe_ias).then_some(RECOVERY_THROTTLE);
    let stable = !escape
        && s.bank_deg.abs() < 5.0
        && s.pitch_deg.abs() < 8.0
        && s.roll_rate_dps.abs() < 5.0
        && s.stall_ratio <= STALL_TRIP
        && s.ias_kts >= s.min_safe_ias;

    RecoveryCmd {
        // More nose-DOWN authority than up — getting the nose down (out of a stall /
        // nose-high) is the hard, safety-critical direction.
        pitch: pitch.clamp(-0.6, pull_max),
        roll: roll.clamp(-0.6, 0.6),
        throttle,
        stable,
    }
}

/// The stall branch's aileron limit: gentle near wings level (a wing drop
/// risk), the flying branch's full 0.6 from 90 degrees of bank.
pub fn stall_roll_cap(bank_deg: f64) -> f64 {
    const GENTLE: f64 = 0.25;
    const FULL: f64 = 0.6;
    let f = ((bank_deg.abs() - 45.0) / 45.0).clamp(0.0, 1.0);
    GENTLE + (FULL - GENTLE) * f
}

pub struct Recover {
    pub cfg: ApConfig,
    /// Constant-speed prop, if any — driven full forward for max power at the start
    /// of a recovery (a turboprop go-around action). `None` = fixed-pitch.
    pub prop: Option<crate::aircraft::PropProfile>,
    /// Clean stall speed (KIAS). The recovery holds ≈1.3×Vs; defaults if unknown.
    pub stall_speed_kts: f64,
}

/// rad/sec → RPM (shared with the prop tool's convention).
const RAD_PER_SEC_TO_RPM: f64 = 9.549_296_585_513_72;

#[async_trait]
impl Tool for Recover {
    fn name(&self) -> &str {
        "recover"
    }
    fn description(&self) -> &str {
        "EMERGENCY: take control and fly the aircraft back to stable, wings-level \
         flight, recovering from a bank, dive, or stall, then hold altitude. Use for \
         'take control', 'you have control', 'recover', 'stabilize', 'level the \
         aircraft', 'save it', 'help'. The pilot takes back control with 'my controls'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Already flying one: a repeated "recover" (the one phrase a panicked
        // pilot repeats) must not arm a second copy. The superseded first run
        // used to zero the joystick overrides the new one had just taken,
        // ~100 ms in, leaving the live recovery writing yoke ratios the sim
        // re-drove from the pilot's hardware every frame: an inert recovery
        // in an actual upset, reported as flown.
        let Some(lease) = RUN.claim() else {
            return Ok(
                "I'm already recovering — I have the aircraft. Say my controls to take it back."
                    .to_string(),
            );
        };
        // Hold ~1.3 x Vs during the recovery; a sane floor if Vs is unknown.
        run_recovery(
            lease,
            ctx.config.xplane_base_url.clone(),
            self.cfg.fd_commands.clone(),
            self.prop.clone(),
            crate::aircraft::recovery_hold_kts(self.stall_speed_kts),
            ctx.voice.cloned(),
            ctx.status.cloned(),
            Handback {
                why: "recover_tool".into(),
                hold_kts: None,
                ap_no_pitch: false,
                hold_ft: None,
            },
        );
        Ok("I have the aircraft — recovering. Say 'my controls' to take it back.".to_string())
    }
}

/// Read the current recovery state; `None` if a read fails (skip the tick).
/// `min_safe_ias` is the target speed floor, constant for the recovery.
async fn read_state(
    xp: &XPlaneClient,
    min_safe_ias: f64,
    target_pitch_deg: f64,
) -> Option<RecoveryState> {
    Some(RecoveryState {
        target_pitch_deg,
        bank_deg: xp.get_float(BANK).await.ok()?,
        pitch_deg: xp.get_float(PITCH).await.ok()?,
        roll_rate_dps: xp.get_float(ROLL_RATE).await.unwrap_or(0.0),
        pitch_rate_dps: xp.get_float(PITCH_RATE).await.unwrap_or(0.0),
        ias_kts: xp.get_float(IAS).await.ok()?,
        stall_ratio: xp.get_float(STALL_RATIO).await.unwrap_or(0.0),
        min_safe_ias,
        agl_ft: xp
            .get_float(crate::datarefs::AGL_METERS)
            .await
            .ok()
            .map(|m| m * 3.28084),
        vs_fpm: xp.get_float(VS_FPM).await.ok(),
    })
}

/// How a crew loop hands the aircraft to the recovery (#181): a climb the
/// autopilot does not answer (`climb_watch`), or a give-up that finds the
/// aircraft slow or upset. Never "your aircraft" to nobody.
#[derive(Debug, Clone)]
pub struct TakeOver {
    /// The blackbox word: whose loop and why.
    pub why: String,
    /// The slowest speed the recovery holds (the configuration's floor and a
    /// margin); `None` = the airframe's clean floor.
    pub min_safe_ias: Option<f64>,
    /// The governor's speed once the autopilot is seen to hold; `None` =
    /// cruise throttle, as the recover tool does.
    pub hold_kts: Option<f64>,
    /// The autopilot's pitch moves nothing (`climb_watch::AuthorityCheck`):
    /// the pitch is hand flown from the start and never given back to it.
    pub ap_no_pitch: bool,
    /// The height the hand-flown hold flies to; `None` = where we are.
    pub hold_ft: Option<f64>,
}

/// Take the aircraft from a crew loop: the servos out, flown level on the
/// column, the autopilot given it back only once it is seen to hold, else
/// hand flown level until "my controls". `false` when a recovery is already
/// flying (it keeps the aircraft).
pub fn take_over(base: String, voice: Option<Voice>, t: TakeOver) -> bool {
    let Some(lease) = RUN.claim() else {
        return false;
    };
    let min_safe_ias = t
        .min_safe_ias
        .or_else(crate::aircraft::speed_floor_kts)
        .filter(|v| v.is_finite() && *v > 0.0)
        .unwrap_or(70.0);
    crate::blackbox::record(
        "recover",
        json!({"event": "take_over", "why": t.why, "min_safe_ias": min_safe_ias,
               "hold_kts": t.hold_kts, "ap_no_pitch": t.ap_no_pitch, "hold_ft": t.hold_ft}),
    );
    run_recovery(
        lease,
        base,
        Vec::new(),
        None,
        min_safe_ias,
        voice,
        None,
        Handback {
            why: t.why,
            hold_kts: t.hold_kts,
            ap_no_pitch: t.ap_no_pitch,
            hold_ft: t.hold_ft.filter(|h| h.is_finite()),
        },
    );
    true
}

/// What the aircraft is doing that no one may be handed: a stall warning, a
/// speed under the floor, a bank past 45 degrees, the nose far up or down.
/// `None` = flyable. A missing reading is no evidence either way. Pure.
pub fn unflyable(
    ias_kts: Option<f64>,
    floor_kts: Option<f64>,
    pitch_deg: Option<f64>,
    bank_deg: Option<f64>,
    stall_ratio: Option<f64>,
) -> Option<&'static str> {
    let ok = |v: Option<f64>| v.filter(|x| x.is_finite());
    if ok(stall_ratio).is_some_and(|s| s > STALL_TRIP) {
        return Some("stall");
    }
    if let (Some(ias), Some(floor)) = (ok(ias_kts), ok(floor_kts)) {
        if ias < floor {
            return Some("slow");
        }
    }
    if ok(bank_deg).is_some_and(|b| b.abs() > UPSET_BANK_DEG) {
        return Some("bank");
    }
    match ok(pitch_deg) {
        Some(p) if p > UPSET_NOSE_UP_DEG => Some("nose_high"),
        Some(p) if p < UPSET_NOSE_DOWN_DEG => Some("nose_low"),
        _ => None,
    }
}

/// Read the attitude and speed a give-up judges with ([`unflyable`]).
pub async fn read_unflyable(
    xp: &dyn crate::xplane::XPlane,
    floor_kts: Option<f64>,
) -> Option<&'static str> {
    let (ias, pitch, bank, stall) = tokio::join!(
        xp.get_float(IAS),
        xp.get_float(PITCH),
        xp.get_float(BANK),
        xp.get_float(STALL_RATIO)
    );
    unflyable(ias.ok(), floor_kts, pitch.ok(), bank.ok(), stall.ok())
}

/// The failures that take the autopilot's elevator away: failed is 6.
const PITCH_FAILURES: [&str; 5] = [
    "sim/operation/failures/rel_otto",
    "sim/operation/failures/rel_auto_servos",
    "sim/operation/failures/rel_servo_elev",
    "sim/operation/failures/rel_elv_trim_run",
    "sim/operation/failures/rel_trim_elv",
];
const OVR_AUTOPILOT: &str = "sim/operation/override/override_autopilot";
/// The autopilot's vertical mode as X-Plane flies it (4 = vertical speed).
const ALT_MODE: &str = autoflight::names::ALT_MODE;
const CG_MAC: &str = "sim/flightmodel2/misc/cg_offset_z_mac";
const WEIGHT: &str = "sim/flightmodel/weight/m_total";
/// A failure dataref's "failed now".
const FAILED: f64 = 6.0;

/// Who holds the elevator, and the airframe's balance, as read (#181: the
/// 737 that would not follow its dial had none of this logged, so its cause
/// is unproven). A failed read is `None`, never a value.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PitchAuthority {
    /// The pitch failures reading failed, by dataref.
    pub failed: Vec<&'static str>,
    pub override_autopilot: Option<bool>,
    pub override_pitch: Option<bool>,
    pub alt_mode: Option<f64>,
    pub cg_mac_pct: Option<f64>,
    pub weight_kg: Option<f64>,
    pub trim: Option<f64>,
    /// The autopilot's servo output, the elevator surface (degrees, trailing
    /// edge down positive) and the summed column (#181: the servo moved
    /// nothing, and nothing logged either).
    pub servo: Option<f64>,
    pub elev_deg: Option<f64>,
    pub total: Option<f64>,
    pub vvi_dial: Option<f64>,
    pub alt_hold_ft: Option<f64>,
    pub override_fd_pitch: Option<bool>,
}

impl PitchAuthority {
    /// Why no automation may take off on this airframe: a failed autopilot
    /// or elevator trim, or another plugin owning the autopilot's brains.
    /// Pure.
    pub fn refusal(&self) -> Option<String> {
        if !self.failed.is_empty() {
            let names: Vec<&str> = self
                .failed
                .iter()
                .map(|d| d.trim_start_matches("sim/operation/failures/"))
                .collect();
            return Some(format!(
                "The autopilot's pitch control is failed ({}).",
                names.join(", ").replace('_', " ")
            ));
        }
        (self.override_autopilot == Some(true))
            .then(|| "Another plugin is overriding the autopilot.".to_string())
    }

    pub fn json(&self) -> Value {
        json!({
            "failed": self.failed, "override_autopilot": self.override_autopilot,
            "override_pitch": self.override_pitch, "alt_mode": self.alt_mode,
            "cg_mac_pct": self.cg_mac_pct, "weight_kg": self.weight_kg, "trim": self.trim,
            "servo": self.servo, "elev": self.elev_deg, "total": self.total,
            "vvi_dial": self.vvi_dial, "alt_hold_ft": self.alt_hold_ft,
            "override_fd_pitch": self.override_fd_pitch,
        })
    }
}

/// Read [`PitchAuthority`].
pub async fn read_pitch_authority(xp: &dyn crate::xplane::XPlane) -> PitchAuthority {
    let fails = futures::future::join_all(PITCH_FAILURES.iter().map(|d| xp.get_float(d))).await;
    let failed = PITCH_FAILURES
        .iter()
        .zip(fails)
        .filter(|(_, v)| v.as_ref().is_ok_and(|v| (*v - FAILED).abs() < 0.5))
        .map(|(d, _)| *d)
        .collect();
    let (ovr_ap, ovr_pitch, mode, cg, weight, trim) = tokio::join!(
        xp.get_float(OVR_AUTOPILOT),
        xp.get_float(OVR_PITCH),
        xp.get_float(ALT_MODE),
        xp.get_float(CG_MAC),
        xp.get_float(WEIGHT),
        xp.get_float(ELEV_TRIM)
    );
    let (servo, elev, total, vvi_dial, alt_hold, ovr_fd) = tokio::join!(
        xp.get_float(crate::datarefs::SERVO_PITCH_RATIO),
        xp.get_float_array(crate::datarefs::ELEVATOR1_DEG),
        xp.get_float(crate::datarefs::TOTAL_PITCH_RATIO),
        xp.get_float(crate::datarefs::VVI_DIAL_FPM),
        xp.get_float(crate::datarefs::ALTITUDE_HOLD_FT),
        xp.get_float(crate::datarefs::OVERRIDE_FD_PITCH)
    );
    let ok = |r: anyhow::Result<f64>| r.ok().filter(|v| v.is_finite());
    PitchAuthority {
        failed,
        override_autopilot: ok(ovr_ap).map(|v| v > 0.5),
        override_pitch: ok(ovr_pitch).map(|v| v > 0.5),
        alt_mode: ok(mode),
        cg_mac_pct: ok(cg),
        weight_kg: ok(weight),
        trim: ok(trim),
        servo: ok(servo),
        elev_deg: elev.ok().and_then(|v| crate::datarefs::elevator_surface_deg(&v)),
        total: ok(total),
        vvi_dial: ok(vvi_dial),
        alt_hold_ft: ok(alt_hold),
        override_fd_pitch: ok(ovr_fd).map(|v| v > 0.5),
    }
}

/// A bank past this is an upset, not a turn.
pub const UPSET_BANK_DEG: f64 = 45.0;
/// Nose attitudes past these are upsets.
pub const UPSET_NOSE_UP_DEG: f64 = 25.0;
pub const UPSET_NOSE_DOWN_DEG: f64 = -15.0;

/// After a recovery the autopilot is watched this long before it is
/// believed (#181: an autopilot that did not hold the climb will not hold
/// the level either).
pub const AP_VERIFY_S: f64 = 12.0;

/// The autopilot is holding what it was given: a level, wings nearly level,
/// the nose in a flying band, the speed above most of the floor. A missing
/// reading is not a failure (it is no measurement). Pure.
pub fn ap_holding(
    vvi_fpm: Option<f64>,
    pitch_deg: Option<f64>,
    bank_deg: Option<f64>,
    ias_kts: Option<f64>,
    min_safe_ias: f64,
) -> bool {
    let ok = |v: Option<f64>| v.filter(|x| x.is_finite());
    !(ok(vvi_fpm).is_some_and(|v| v.abs() > 2_500.0)
        || ok(pitch_deg).is_some_and(|p| !(-12.0..=15.0).contains(&p))
        || ok(bank_deg).is_some_and(|b| b.abs() > 40.0)
        || ok(ias_kts).is_some_and(|v| v < 0.9 * min_safe_ias))
}

/// The attitude a hand-flown hold asks for: level plus the altitude to
/// regain or lose, bounded. Pure.
pub fn hold_pitch_deg(alt_err_ft: f64) -> f64 {
    if !alt_err_ft.is_finite() {
        return LEVEL_PITCH_DEG;
    }
    (LEVEL_PITCH_DEG + alt_err_ft / 100.0).clamp(-3.0, 6.0)
}

/// The thrust a hand-flown hold sets: full under the safe speed, cruise
/// well over it, else left alone (`None`). Pure.
pub fn hold_throttle(ias_kts: f64, min_safe_ias: f64) -> Option<f64> {
    if !ias_kts.is_finite() {
        return None;
    }
    if ias_kts < min_safe_ias {
        Some(RECOVERY_THROTTLE)
    } else if ias_kts > min_safe_ias + 40.0 {
        Some(CRUISE_THROTTLE)
    } else {
        None
    }
}

/// The attitude the recovery's pitch-to-level phase aims at.
const LEVEL_PITCH_DEG: f64 = 2.0;
/// The plugin's column and wheel channels (1 s heartbeats, written every
/// tick; the pilot's deflection yields them, `xcopilot/pitch_yield`).
const CH_PITCH: &str = "xcopilot/pitch";
const CH_ROLL: &str = "xcopilot/roll";
const SERVOS_ON: &str = autoflight::names::SERVOS_ON;
const VVI: &str = "sim/cockpit2/gauges/indicators/vvi_fpm_pilot";

/// What the recovery does once stable.
struct Handback {
    why: String,
    hold_kts: Option<f64>,
    /// No autopilot pitch: hand flown from the start, never handed back.
    ap_no_pitch: bool,
    /// The hand-flown hold's height; `None` = where we are.
    hold_ft: Option<f64>,
}

/// The servos out, read back: a servo left in writes the column back over
/// ours. `servos_off_any` first, the FD-off disconnect if it did not take.
async fn servos_out(xp: &XPlaneClient) -> Option<f64> {
    if xp.get_float(SERVOS_ON).await.is_ok_and(|v| v > 0.5) {
        let _ = autoflight::press(xp, Button::Disconnect).await;
        if xp.get_float(SERVOS_ON).await.is_ok_and(|v| v > 0.5) {
            let _ = autoflight::press(xp, Button::DisconnectFd).await;
        }
    }
    xp.get_float(SERVOS_ON).await.ok()
}

/// One tick of column and wheel, through the plugin's channels, or the raw
/// joystick override where the plugin has none.
async fn write_controls(xp: &XPlaneClient, plugin: bool, pitch: f64, roll: f64) {
    // Our own column, not a pilot's hand: the scorer counted the recovery's
    // writes as yoke_under_ap interventions (A330 LEMG, 2026-09-29).
    crate::monitor::note_crew_yoke();
    if plugin {
        let _ = tokio::join!(xp.set_float(CH_PITCH, pitch), xp.set_float(CH_ROLL, roll));
    } else {
        let _ = xp.set_float(YOKE_PITCH, pitch).await;
        let _ = xp.set_float(YOKE_ROLL, roll).await;
    }
}

/// Let go of the controls: the channels expire by silence after a centred
/// write; the raw override is dropped.
async fn release_controls(xp: &XPlaneClient, plugin: bool) {
    if plugin {
        // No centring write: it would hold the override on, column neutral,
        // for another second under the servo about to take the elevator.
        // Silence expires the channel (the plugin centres and drops it).
        tokio::time::sleep(CHANNEL_EXPIRY).await;
    } else {
        write_controls(xp, plugin, 0.0, 0.0).await;
        let _ = xp.set_float(OVR_PITCH, 0.0).await;
        let _ = xp.set_float(OVR_ROLL, 0.0).await;
    }
}

/// The plugin channels' freshness (`kTaxiBrakeFreshSecs`, 1 s) and a margin.
const CHANNEL_EXPIRY: std::time::Duration = std::time::Duration::from_millis(1_200);

fn say(voice: &Option<Voice>, status: &Option<crate::plugin_status::StatusSink>, line: &str) {
    if let Some(s) = status {
        s.action(line);
    }
    if let Some(v) = voice {
        v.alert(Priority::Warning, line.to_string());
    }
}

/// Own thread + runtime + client (like the AP capture watcher): drive the controls
/// in a tight loop until stable, then hand to the autopilot and WATCH that it
/// holds; one that does not is taken back and the aircraft hand flown level
/// until "my controls". Releases immediately when its run is cancelled ("my
/// controls", a stand-down) or superseded.
/// No deadline lets go of an unrecovered aircraft (#181).
fn run_recovery(
    lease: sim::run::Lease<'static>,
    base: String,
    fd_commands: Vec<String>,
    prop: Option<crate::aircraft::PropProfile>,
    min_safe_ias: f64,
    voice: Option<Voice>,
    status: Option<crate::plugin_status::StatusSink>,
    handback: Handback,
) {
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        let lease = &lease;
        rt.block_on(async move {
            let base_url = base.clone();
            let xp = XPlaneClient::new(base);
            // Asked before EVERY write, not once a tick: a request held by
            // the sim's load gate returns into the next flight (#181: a
            // tick in flight when the setup began wrote the column 6 s into
            // the new approach, t 1790651450).
            let live = || may_write(lease);
            // The servos out FIRST: an engaged servo writes the column back
            // over ours, and the autopilot holds the nose up into a stall.
            let servos_after = servos_out(&xp).await;
            // The plugin's channels where it has them (the pilot's hands
            // yield them), else the raw joystick override.
            let plugin = xp.set_float(CH_PITCH, 0.0).await.is_ok()
                && xp.set_float(CH_ROLL, 0.0).await.is_ok();
            if !plugin {
                let _ = xp.set_float(OVR_PITCH, 1.0).await;
                let _ = xp.set_float(OVR_ROLL, 1.0).await;
            }
            let authority = read_pitch_authority(&xp).await;
            crate::blackbox::record(
                "recover",
                json!({"event": "start", "why": handback.why, "servos_after": servos_after,
                       "authority": authority.json(),
                       "channels": if plugin { "plugin" } else { "override" },
                       "min_safe_ias": min_safe_ias, "ap_no_pitch": handback.ap_no_pitch}),
            );
            // Props full forward: the whole power band for the unstall.
            if let Some(p) = &prop {
                let _ = xp.set_float(&p.dataref, p.max_rpm / RAD_PER_SEC_TO_RPM).await;
            }

            // An autopilot with no pitch is never given the aircraft back:
            // its hold law flies it from the first tick (upsets included).
            let hold = if handback.ap_no_pitch {
                let alt = xp.get_float(CURRENT_ALT).await.ok().filter(|a| a.is_finite());
                let hold = handback
                    .hold_ft
                    .or(alt.map(|a| (a / 100.0).round() * 100.0))
                    .unwrap_or(f64::NAN);
                say(&voice, &status, crate::climb_watch::NO_PITCH_LINE);
                hold
            } else {
                let t0 = std::time::Instant::now();
                let mut still_said = false;
                let mut stable_ticks = 0;
                let mut last_phase = ""; // announce ANNOUNCE-mode actions on phase change
                while live() {
                    // Frozen sim (paused or replay): nothing to read, nothing
                    // to write, and a mid-recovery pause hands back nothing.
                    if crate::xplane::sim_frozen(&xp).await {
                        tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                        continue;
                    }
                    let Some(s) = read_state(&xp, min_safe_ias, LEVEL_PITCH_DEG).await else {
                        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                        continue;
                    };
                    let cmd = recovery_inputs(&s);
                    if !live() {
                        break;
                    }
                    write_controls(&xp, plugin, cmd.pitch, cmd.roll).await;
                    if let Some(t) = cmd.throttle.filter(|_| live()) {
                        let _ = xp.set_float(&throttle_dref(), t).await;
                    }
                    // Re-trim to RELIEVE the yoke: a nose-up trim (from slowing
                    // toward a stall) fights a nose-down push, so when we're
                    // holding a big pitch input, walk the trim that way. This is
                    // what makes "nose down" work when the aircraft is trimmed
                    // against us. Rate-limited and bounded.
                    if cmd.pitch.abs() > 0.15 {
                        let trim = xp.get_float(ELEV_TRIM).await.unwrap_or(0.0);
                        let step = 0.012 * cmd.pitch.signum();
                        if live() {
                            let _ = xp.set_float(ELEV_TRIM, (trim + step).clamp(-0.6, 0.4)).await;
                        }
                    }
                    // ANNOUNCE mode: say what we're DOING as the phase changes
                    // (not every tick), rather than the monitor's
                    // pilot-directed advice (which is muted while we fly).
                    let phase = if s.stall_ratio > STALL_TRIP {
                        "unstall"
                    } else if s.bank_deg.abs() > 25.0 {
                        "roll"
                    } else {
                        "pitch"
                    };
                    if phase != last_phase {
                        last_phase = phase;
                        crate::autoland::set_panel(&format!(
                            "RECOVERING: {}",
                            phase.to_uppercase()
                        ));
                        let line = match phase {
                            "unstall" => "Breaking the stall, nose down, full power.",
                            "roll" => "Rolling wings level.",
                            _ => "Easing the nose to level.",
                        };
                        say(&voice, &status, line);
                    }
                    if !still_said && t0.elapsed().as_secs() >= 60 {
                        still_said = true;
                        say(&voice, &status, "Still recovering. Say my controls to take it.");
                    }
                    tracing::info!(
                        target: "xvoicelayer::recover",
                        bank = s.bank_deg, pitch = s.pitch_deg, stall = s.stall_ratio,
                        ias = s.ias_kts, yoke_pitch = cmd.pitch, yoke_roll = cmd.roll, "recover"
                    );
                    crate::blackbox::record("recover", serde_json::json!({
                        "bank": s.bank_deg, "pitch": s.pitch_deg, "stall": s.stall_ratio,
                        "ias": s.ias_kts, "yoke_pitch": cmd.pitch, "yoke_roll": cmd.roll,
                    }));
                    // Require a few consecutive stable ticks so a momentary
                    // pass through level doesn't end the recovery prematurely.
                    if cmd.stable {
                        stable_ticks += 1;
                        if stable_ticks >= 3 {
                            break;
                        }
                    } else {
                        stable_ticks = 0;
                    }
                    tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                }

                // If the pilot took it back, let go and don't grab it again
                // via the AP.
                if !live() {
                    stood_down(lease);
                    release_controls(&xp, plugin).await;
                    crate::autoland::set_panel("");
                    finish(lease);
                    return;
                }

                // Hand to the autopilot: FD up, servos on, then HOLD the
                // current heading AND altitude (heading hold, not just
                // wings-level, so it doesn't drift). The mode commands are
                // toggles: only fire when the mode is off, and write the
                // heading bug LAST (engaging snaps the dial). The column is
                // let go first: the servo takes the elevator.
                release_controls(&xp, plugin).await;
                if !live() {
                    stood_down(lease);
                    crate::autoland::set_panel("");
                    finish(lease);
                    return;
                }
                for c in &fd_commands {
                    let _ = xp.run_command(c).await;
                }
                let _ = autoflight::press(&xp, Button::Engage).await;
                if xp.get_float(HEADING_STATUS).await.unwrap_or(0.0) <= 0.5 {
                    let _ = autoflight::press(&xp, Button::Heading).await;
                }
                // No compass read, no bug write: a heading of "zero" is a turn
                // north, not a hold, and heading mode keeps whatever is dialled.
                let hdg = xp.get_float(CURRENT_HDG).await.ok();
                if let Some(h) = hdg {
                    let _ = crate::tools::autopilot::drive_heading_bug(&xp, h).await;
                }
                let alt = xp.get_float(CURRENT_ALT).await.unwrap_or(0.0);
                let hold = (alt / 100.0).round() * 100.0;
                let _ = autoflight::set_dial(&xp, Dial::Altitude, hold).await;
                if xp.get_float(ALT_HOLD_STATUS).await.unwrap_or(0.0) <= 0.5 {
                    let _ = autoflight::press(&xp, Button::AltitudeHold).await;
                }
                if handback.hold_kts.is_none() {
                    let _ = xp.set_float(&throttle_dref(), CRUISE_THROTTLE).await;
                }

                // The autopilot is WATCHED before it is believed (#181): the
                // one that let the climb run away would let the level go too.
                let verify_t0 = std::time::Instant::now();
                let mut holding = true;
                while live() && verify_t0.elapsed().as_secs_f64() < AP_VERIFY_S {
                    tokio::time::sleep(std::time::Duration::from_millis(250)).await;
                    if crate::xplane::sim_frozen(&xp).await {
                        continue;
                    }
                    let (vvi, pitch, bank, ias, servos) = tokio::join!(
                        xp.get_float(VVI),
                        xp.get_float(PITCH),
                        xp.get_float(BANK),
                        xp.get_float(IAS),
                        xp.get_float(SERVOS_ON)
                    );
                    let (vvi, pitch, bank, ias, servos) =
                        (vvi.ok(), pitch.ok(), bank.ok(), ias.ok(), servos.ok());
                    // A failed servo read is no measurement, not a disconnect.
                    let servos_in = servos.is_none_or(|v| v > 0.5);
                    if !servos_in || !ap_holding(vvi, pitch, bank, ias, min_safe_ias) {
                        holding = false;
                        crate::blackbox::record(
                            "recover",
                            json!({"event": "ap_not_holding", "vvi": vvi, "pitch": pitch,
                                   "bank": bank, "ias": ias, "servos": servos,
                                   "after_s": verify_t0.elapsed().as_secs_f64()}),
                        );
                        break;
                    }
                }
                if !live() {
                    stood_down(lease);
                    crate::autoland::set_panel("");
                    finish(lease);
                    return;
                }

                if holding {
                    crate::autoland::set_panel("");
                    finish(lease);
                    if let Some(kts) = handback.hold_kts {
                        crate::tools::speedhold::arm_flight(base_url.clone(), voice.clone(), kts);
                    }
                    crate::blackbox::record(
                        "recover",
                        json!({"event": "handed_to_autopilot", "hold_ft": hold, "hdg": hdg,
                               "hold_kts": handback.hold_kts}),
                    );
                    // The line says what is actually held: no compass read,
                    // no heading claimed.
                    let line = match hdg {
                        Some(h) => format!(
                            "Recovered. Autopilot holding heading {h:.0} and {hold:.0} feet. You have control when ready."
                        ),
                        None => format!(
                            "Recovered. Autopilot holding the dialled heading and {hold:.0} feet. You have control when ready."
                        ),
                    };
                    if let Some(s) = &status {
                        s.action(&line);
                    }
                    if let Some(v) = &voice {
                        v.say(Priority::Warning, line);
                    }
                    return;
                }
                hold
            };

            // Hand flown to the hold, until the pilot says so: the autopilot
            // would not hold it, or has no pitch at all.
            let servos_after = servos_out(&xp).await;
            crate::blackbox::record(
                "recover",
                json!({"event": "hand_flying", "hold_ft": hold, "servos_after": servos_after,
                       "ap_no_pitch": handback.ap_no_pitch}),
            );
            crate::autoland::set_panel("HAND FLYING: SAY MY CONTROLS");
            let line = if !hold.is_finite() {
                "Wings level. Say my controls.".to_string()
            } else if handback.ap_no_pitch {
                format!("Holding {hold:.0} feet, wings level. Say my controls.")
            } else {
                format!(
                    "The autopilot won't hold it. I'm hand flying, wings level at {hold:.0} \
                     feet. Say my controls."
                )
            };
            say(&voice, &status, &line);
            while live() {
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                if crate::xplane::sim_frozen(&xp).await {
                    continue;
                }
                let alt_now = xp.get_float(CURRENT_ALT).await.ok();
                let target = hold_pitch_deg(alt_now.map_or(f64::NAN, |a| hold - a));
                let Some(s) = read_state(&xp, min_safe_ias, target).await else {
                    continue;
                };
                let cmd = recovery_inputs(&s);
                if !live() {
                    break;
                }
                write_controls(&xp, plugin, cmd.pitch, cmd.roll).await;
                let thr = if s.stall_ratio > STALL_TRIP {
                    cmd.throttle
                } else {
                    hold_throttle(s.ias_kts, min_safe_ias)
                };
                if let Some(t) = thr.filter(|_| live()) {
                    let _ = xp.set_float(&throttle_dref(), t).await;
                }
            }
            stood_down(lease);
            release_controls(&xp, plugin).await;
            crate::autoland::set_panel("");
            finish(lease);
        });
    });
}

/// May this recovery still write? Not after "my controls" or a stand-down
/// (its run cancelled), once a newer recovery took the run, or while the sim
/// loads a new flight (`xplane::sim_loading`): a new flight is never this
/// recovery's.
fn may_write(lease: &sim::run::Lease<'_>) -> bool {
    recovery_live(
        lease.current(),
        lease.cancel.load(Ordering::SeqCst),
        crate::xplane::sim_loading(),
    )
}

/// [`may_write`]'s rule. Pure.
pub fn recovery_live(current: bool, aborted: bool, loading: bool) -> bool {
    current && !aborted && !loading
}

/// Why a recovery let go, on the tape (blackbox `recover` `stood_down`).
fn stood_down(lease: &sim::run::Lease<'_>) {
    let why = if !lease.current() {
        "superseded"
    } else if lease.cancel.load(Ordering::SeqCst) {
        "abort"
    } else if crate::xplane::sim_loading() {
        "new_flight"
    } else {
        "ended"
    };
    crate::blackbox::record("recover", json!({"event": "stood_down", "why": why}));
}

/// The recovery is over (its lease drops the rest with the thread): the
/// flag goes now, and only while this is the live run (the slot's rule).
fn finish(lease: &sim::run::Lease<'_>) {
    lease.lower_flag();
}

/// Signal an active recovery to release the controls immediately ("my controls").
/// Returns whether one was actually running, so the caller can acknowledge it
/// ("you have the aircraft") instead of the plain AP-off readback. Safe to call
/// when none is running.
pub fn abort() -> bool {
    RUN.cancel()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(bank: f64, pitch: f64, stall: f64, ias: f64) -> RecoveryState {
        RecoveryState {
            bank_deg: bank,
            pitch_deg: pitch,
            roll_rate_dps: 0.0,
            pitch_rate_dps: 0.0,
            ias_kts: ias,
            stall_ratio: stall,
            min_safe_ias: 90.0,
            target_pitch_deg: LEVEL_PITCH_DEG,
            agl_ft: None,
            vs_fpm: None,
        }
    }

    #[test]
    fn low_and_descending_climbs_away_from_the_ground() {
        // LEMG 13 2026-09-29: unpaused at 200 ft, 7 degrees nose down, 218 kt.
        let s = RecoveryState {
            agl_ft: Some(204.0),
            vs_fpm: Some(-2_500.0),
            ..state(-23.8, -7.0, 0.0, 218.0)
        };
        let c = recovery_inputs(&s);
        assert!(c.pitch >= 0.5, "a firm pull, got {}", c.pitch);
        assert_eq!(c.throttle, Some(RECOVERY_THROTTLE));
        assert!(!c.stable);
        // High, the same attitude is only levelled, gently.
        let high = RecoveryState { agl_ft: Some(5_000.0), ..s };
        assert!(recovery_inputs(&high).pitch <= 0.4);
        // Low but climbing and level is no escape: stable as before.
        let fine = RecoveryState {
            agl_ft: Some(600.0),
            vs_fpm: Some(300.0),
            ..state(1.0, 2.0, 0.0, 200.0)
        };
        assert!(recovery_inputs(&fine).stable);
    }

    #[test]
    fn stall_pushes_nose_down_and_powers_up_before_rolling() {
        // Deep in a stall: nose-down push, gentle roll, full power, not stable.
        let c = recovery_inputs(&state(30.0, 15.0, 0.8, 75.0));
        assert!(
            c.pitch < 0.0,
            "must push nose down to unstall, got {}",
            c.pitch
        );
        assert_eq!(c.throttle, Some(RECOVERY_THROTTLE));
        assert!(c.roll.abs() <= 0.25, "roll stays gentle near the stall");
        assert!(!c.stable);
    }

    #[test]
    fn a_steep_stalled_bank_rolls_with_full_authority() {
        // LEMG 13 2026-09-29: 103 degrees left, stall warning, rolling on left.
        let mut s = state(-103.0, -0.2, 1.0, 178.0);
        s.roll_rate_dps = -55.0;
        let c = recovery_inputs(&s);
        assert!(c.pitch < 0.0, "still unloads");
        assert!((c.roll - 0.6).abs() < 1e-9, "full right aileron, got {}", c.roll);
        assert_eq!(stall_roll_cap(30.0), 0.25);
        assert!((stall_roll_cap(-67.5) - 0.425).abs() < 1e-9);
    }

    #[test]
    fn right_bank_commands_left_roll() {
        // A right bank (phi > 0), flying: command left roll (negative), no power change.
        let c = recovery_inputs(&state(45.0, 0.0, 0.0, 150.0));
        assert!(c.roll < 0.0, "right bank needs left roll, got {}", c.roll);
        assert_eq!(c.throttle, None);
        assert!(!c.stable, "45 deg of bank is not stable");
    }

    #[test]
    fn nose_high_commands_nose_down() {
        // Nose way up, flying: command nose down (negative pitch).
        let c = recovery_inputs(&state(0.0, 25.0, 0.0, 140.0));
        assert!(c.pitch < 0.0, "nose-high needs nose-down, got {}", c.pitch);
    }

    #[test]
    fn wings_level_and_flying_is_stable() {
        let c = recovery_inputs(&state(2.0, 1.5, 0.0, 150.0));
        assert!(c.stable, "wings level, on speed, not stalling → stable");
    }

    #[test]
    fn slow_but_not_stalling_still_commands_full_power() {
        // Below the safe speed (min_safe_ias 90) even without a stall warning:
        // add power and don't call it stable.
        let c = recovery_inputs(&state(0.0, 2.0, 0.0, 80.0));
        assert_eq!(c.throttle, Some(RECOVERY_THROTTLE));
        assert!(!c.stable, "below safe speed is not stable");
    }

    #[test]
    fn a_stall_is_never_reported_stable_even_wings_level() {
        // Wings level but stalling → NOT stable (must unstall first).
        let c = recovery_inputs(&state(0.0, 12.0, 0.6, 80.0));
        assert!(!c.stable);
        assert!(c.pitch < 0.0);
    }

    #[test]
    fn commands_are_always_clamped() {
        // An extreme upset can't produce out-of-range control inputs.
        let c = recovery_inputs(&state(179.0, 89.0, 0.0, 200.0));
        assert!((-1.0..=1.0).contains(&c.pitch) && (-1.0..=1.0).contains(&c.roll));
        // Asymmetric pitch clamp: up to 0.6 nose-down, 0.4 nose-up; roll ±0.6.
        assert!((-0.6..=0.4).contains(&c.pitch) && c.roll.abs() <= 0.6);
    }

    /// #181: the circuit's give-up found the 737 at 73 kt (floor 149.5)
    /// and said "Your aircraft." Slow, stalling, banked or nose far up is
    /// never handed to nobody; a missing reading is no evidence.
    #[test]
    fn a_stalling_or_upset_aircraft_is_unflyable() {
        let f = Some(149.5);
        assert_eq!(unflyable(Some(73.8), f, Some(20.0), Some(2.0), Some(0.0)), Some("slow"));
        assert_eq!(unflyable(Some(160.0), f, Some(5.0), Some(0.0), Some(0.6)), Some("stall"));
        assert_eq!(unflyable(Some(160.0), f, Some(5.0), Some(160.0), None), Some("bank"));
        assert_eq!(unflyable(Some(160.0), f, Some(30.0), Some(0.0), None), Some("nose_high"));
        assert_eq!(unflyable(Some(160.0), f, Some(-20.0), Some(0.0), None), Some("nose_low"));
        assert_eq!(unflyable(Some(160.0), f, Some(5.0), Some(20.0), Some(0.0)), None);
        assert_eq!(unflyable(None, f, None, None, None), None);
        assert_eq!(unflyable(Some(f64::NAN), f, Some(f64::NAN), None, None), None);
    }

    /// The autopilot given the aircraft back is believed only while it holds
    /// a level; the #181 airframe's 9,000 fpm climb under a lit mode is not.
    #[test]
    fn an_autopilot_is_believed_only_while_it_holds() {
        assert!(ap_holding(Some(300.0), Some(3.0), Some(2.0), Some(190.0), 180.0));
        assert!(!ap_holding(Some(9_000.0), Some(3.0), Some(2.0), Some(190.0), 180.0));
        assert!(!ap_holding(Some(0.0), Some(22.0), Some(2.0), Some(190.0), 180.0));
        assert!(!ap_holding(Some(0.0), Some(3.0), Some(60.0), Some(190.0), 180.0));
        assert!(!ap_holding(Some(0.0), Some(3.0), Some(0.0), Some(150.0), 180.0));
        assert!(ap_holding(None, None, None, None, 180.0), "no reading, no verdict");
    }

    /// No automation takes off with the autopilot's elevator failed or
    /// another plugin on its brains; the balance alone refuses nothing.
    #[test]
    fn a_takeoff_is_refused_without_the_autopilots_pitch() {
        let ok = PitchAuthority {
            cg_mac_pct: Some(31.0),
            weight_kg: Some(62_000.0),
            ..PitchAuthority::default()
        };
        assert_eq!(ok.refusal(), None);
        let servo = PitchAuthority {
            failed: vec!["sim/operation/failures/rel_servo_elev"],
            ..ok.clone()
        };
        let why = servo.refusal().expect("refused");
        assert!(why.contains("rel servo elev"), "{why}");
        assert!(!why.contains('\u{2014}') && !why.contains('\u{2013}'));
        let other = PitchAuthority {
            override_autopilot: Some(true),
            ..ok
        };
        assert!(other.refusal().is_some());
    }

    /// #181 (t 1790651450): a recovery tick held by the sim's load gate
    /// wrote the column 6 s into the next setup's approach. A recovery
    /// writes only while it is the live one: not after "my controls" or a
    /// stand-down, not once superseded, never while a new flight loads.
    #[test]
    fn a_recovery_never_writes_into_the_next_flight() {
        assert!(recovery_live(true, false, false));
        assert!(!recovery_live(true, true, false), "aborted");
        assert!(!recovery_live(false, false, false), "superseded");
        assert!(!recovery_live(true, false, true), "a new flight loading");
    }

    /// Every streaming write in the recovery's loops asks first, on the line
    /// before it: a tick's reads can return into the next flight.
    #[test]
    fn every_streaming_write_asks_if_it_is_still_live() {
        let src = crate::test_support::module_source("tools/recover");
        let body = src.split("fn run_recovery(").nth(1).unwrap();
        let body = body.split("fn may_write(").next().unwrap();
        let lines: Vec<&str> = body.lines().collect();
        let mut seen = 0;
        for (i, l) in lines.iter().enumerate() {
            let streaming = l.contains("write_controls(&xp, plugin, cmd.")
                || l.contains("set_float(&throttle_dref(), t)")
                || l.contains("set_float(ELEV_TRIM, (trim");
            if !streaming {
                continue;
            }
            seen += 1;
            let before = &lines[i.saturating_sub(3)..i];
            assert!(
                before.iter().any(|b| b.contains("live()")),
                "line {i} writes without asking: {l}"
            );
        }
        assert!(seen >= 5, "the loops' writes were found: {seen}");
    }

    /// A hand-flown hold: level plus the height to make up, bounded; full
    /// thrust under the safe speed, cruise well over it.
    #[test]
    fn a_hand_flown_hold_is_level_and_bounded() {
        assert_eq!(hold_pitch_deg(0.0), LEVEL_PITCH_DEG);
        assert_eq!(hold_pitch_deg(10_000.0), 6.0);
        assert_eq!(hold_pitch_deg(-10_000.0), -3.0);
        assert_eq!(hold_pitch_deg(f64::NAN), LEVEL_PITCH_DEG);
        assert_eq!(hold_throttle(150.0, 180.0), Some(RECOVERY_THROTTLE));
        assert_eq!(hold_throttle(200.0, 180.0), None);
        assert_eq!(hold_throttle(230.0, 180.0), Some(CRUISE_THROTTLE));
        // The stall law still pushes with the height target nose up.
        let s = RecoveryState {
            target_pitch_deg: 6.0,
            ..state(0.0, 12.0, 0.6, 80.0)
        };
        assert!(recovery_inputs(&s).pitch < 0.0);
    }
}
