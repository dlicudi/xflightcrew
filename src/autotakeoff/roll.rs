//! The roll: the squawk, the brake splits, the alignment pivot and the
//! rotation's gate.

use super::*;

/// The transponder to ALT (mode C) through the shared writer, so an add-on's
/// own box (the Twin Otter's KT73) has its KNOB turned; on its own task, since
/// a stepped knob takes a few hundred milliseconds. The result is kept and a
/// failure said: the A330 flew out in standby to two "recycle transponder"
/// calls with nothing said on the flight deck (EGPH, 2026-09-20). It is the
/// ONE departure check (`radio::departure_check`): the tower's assigned code
/// is dialled and read back too, so the spoken takeoff and the circuit leave
/// with the squawk full auto would have set.
pub(super) fn squawk_altitude(base_url: String, v: Option<Voice>, audit: bool) {
    tokio::spawn(async move {
        let xp = XPlaneClient::new(base_url);
        let check =
            crate::tools::radio::departure_check(&xp, crate::party_line::assigned_squawk()).await;
        let mut rec = check.json();
        rec["event"] = serde_json::json!("transponder_alt");
        rec["ok"] = serde_json::json!(check.ok());
        crate::blackbox::record("autotakeoff", rec);
        if let Some(line) = check.failure_line() {
            alert(&v, Priority::Caution, &line);
        }
        // The roll's first call: brakes off and the transponder done, the
        // list audits the rest (`takeoff_config::before_takeoff_audit`).
        if audit {
            crate::takeoff_config::before_takeoff_audit(v);
        }
    });
}

/// Differential brake on the takeoff roll, for an airframe that steers on its
/// brakes ([`crate::aircraft::TaxiProfile::brake_steer`]): the taxi's own
/// split of the steer command, faded out between [`ROLL_BRAKE_FULL_KTS`] and
/// [`ROLL_BRAKE_OFF_KTS`] as the rudder comes alive. EDLP 24 (2026-09-24): the
/// SF50's castoring nosewheel and a rudder with no airflow gave the roll no
/// steering at all; the heading error grew from 8 to 27 degrees with the
/// channel at full deflection until the pilot took the pedals at 21 kt.
pub fn roll_brake_split(steer: f64, gs_kts: f64) -> (f64, f64) {
    let fade = roll_brake_fade(gs_kts);
    let (l, r) = crate::autotaxi::brake_steer_split(steer, gs_kts);
    (l * fade, r * fade)
}

/// The share of the roll's brake steering left at `gs_kts`: whole below
/// [`ROLL_BRAKE_FULL_KTS`], none above [`ROLL_BRAKE_OFF_KTS`] (a brake on
/// the roll costs the acceleration; the rudder steers there). Pure.
pub fn roll_brake_fade(gs_kts: f64) -> f64 {
    ((ROLL_BRAKE_OFF_KTS - gs_kts) / (ROLL_BRAKE_OFF_KTS - ROLL_BRAKE_FULL_KTS)).clamp(0.0, 1.0)
}

/// The roll's brakes on a FREE castoring nose (`aircraft::nose_free`): the
/// one ground law (`ground_steer`) on the roll's heading target, the roll's
/// rudder a known input, its wheel difference faded out with the speed
/// ([`roll_brake_fade`]) and no faster a turn than the aligning pivot's
/// while the power waits ([`RollAlign`]). It replaces the aligning PD and
/// its trim ([`roll_brake_split_aligning`], [`RollAlign::trimmed`]) and the
/// taxi split below 40 kt on these noses: the SR22 at LEMG 12 and the
/// Evolution at LILB 03 (2026-09-29/30) pirouetted +/-12 degrees and held
/// 4 to 5 degrees off on them. Pure.
pub fn free_roll_brakes(
    law: &mut crate::ground_steer::GroundSteer,
    err_deg: f64,
    yaw: crate::ground_steer::Yaw,
    rudder: f64,
    gs_kts: f64,
    dt: f64,
    holding: bool,
) -> (f64, f64) {
    let r_max = if holding {
        crate::autotaxi::ALIGN_RATE_MAX_DPS
    } else {
        crate::ground_steer::DEFAULT_MAX_YAW_DPS
    };
    law.step(crate::ground_steer::heading_demand(
        dt,
        gs_kts / MS_TO_KTS,
        yaw,
        err_deg,
        crate::ground_steer::Rudder::Given(rudder),
        FREE_ROLL_DIFF_MAX * roll_brake_fade(gs_kts),
        r_max,
    ))
    .wheels()
}
/// The most wheel difference the roll's brakes use on a free nose (the
/// pivot's own).
pub(super) const FREE_ROLL_DIFF_MAX: f64 = 0.45;

/// [`roll_brake_split`], plus the start of a roll that begins crooked: below
/// [`ROLL_ALIGN_KTS`] a heading error past [`ROLL_ALIGN_ERR_DEG`], or a swing
/// past [`crate::autotaxi::ALIGN_RATE_MAX_DPS`], is the aligning pivot's
/// ([`crate::autotaxi::align_pivot_brakes`]): the roll law's steer is capped
/// tiny at walking pace (prop wash) and its brake split has no authority
/// under 2 kt. LIME 28 (2026-09-25): the Evolution started its roll 21
/// degrees off the runway, the steer sat at 0.17 with no brake at all, the
/// error grew to 30 at 6 kt and the pilot's pedals ended it.
///
/// The pivot was full brake from 15 degrees on the error alone, capped only
/// past 15 deg/s, and at the iMac's 0.3 to 0.45 s tick it pirouetted: the
/// SR22 at LEMG 12 (1790690995, 2026-09-29) from 10.8 off to -11.6 at 17.5
/// deg/s, then +/-12 degrees from 4 to 18 kt; the Evolution at LILB 03
/// (1790712974) from 24.6 off through -4.4 at 21.6 deg/s. Now the PD led on
/// `yaw_dps` (the sim's `R` where read), which catches the swing with the
/// opposite brake before it overshoots.
pub fn roll_brake_split_aligning(steer: f64, gs_kts: f64, err_deg: f64, yaw_dps: f64) -> (f64, f64) {
    let (l, r) = roll_brake_split(steer, gs_kts);
    if gs_kts >= ROLL_ALIGN_KTS
        || (err_deg.abs() <= ROLL_ALIGN_ERR_DEG
            && yaw_dps.abs() <= crate::autotaxi::ALIGN_RATE_MAX_DPS)
    {
        return (l, r);
    }
    crate::autotaxi::align_pivot_brakes(err_deg, yaw_dps)
}
/// The aligning pivot works below this groundspeed...
pub(super) const ROLL_ALIGN_KTS: f64 = 15.0;
/// ...on a heading error past this: the aligned band itself, since the
/// power waits until the nose is inside it ([`RollAlign`]); at 3 a nose
/// held 2 to 3 off had no brake and no power to come round.
pub(super) const ROLL_ALIGN_ERR_DEG: f64 = crate::autotaxi::ALIGNED_DEG;

/// The roll's power waits for the nose on a brake-steered airframe
/// ([`RollAlign`]): a roll that STARTS at this groundspeed or more is the
/// roll law's from its first tick...
pub(super) const ROLL_ALIGN_HOLD_KTS: f64 = 6.0;
/// ...a held one creeps at this, the line-up's own creep...
pub(super) const ROLL_ALIGN_CREEP_KTS: f64 = crate::autotaxi::ALIGN_CREEP_KTS;
/// ...stopped on both brakes when this far over it, until back under it...
pub(super) const ROLL_ALIGN_OVER_KTS: f64 = crate::autotaxi::ALIGN_CREEP_OVER_KTS;
/// ...at this much brake past the creep, and this more per knot over it...
pub(super) const ROLL_ALIGN_BRAKE_MIN: f64 = 0.15;
pub(super) const ROLL_ALIGN_BRAKE_PER_KT: f64 = 0.1;
/// ...to at most this...
pub(super) const ROLL_ALIGN_BRAKE_MAX: f64 = 0.45;
/// ...and held past this groundspeed all the same, it is a reject on the
/// ground, never a roll.
pub(super) const ROLL_ALIGN_RUNAWAY_KTS: f64 = 10.0;
/// ...while the nose is more than [`crate::autotaxi::ALIGNED_DEG`] off, or
/// swinging faster than this, the lever stays at [`ROLL_PIVOT_THR`] and the
/// ramp and the stagnation clock wait...
pub(super) const ROLL_ALIGNED_DPS: f64 = 3.0;
/// ...the pivot's power (the ramp's own first step)...
pub(super) const ROLL_PIVOT_THR: f64 = 0.3;
/// ...and for no longer than this: a nose that will not come round is a
/// reject at walking pace, said.
pub const ROLL_ALIGN_MAX_S: f64 = 20.0;

/// Whether the roll's power may come up: a castoring nose that starts its
/// roll crooked is pivoted straight at the pivot's power FIRST, then rolled.
/// The time ramp ran the lever from 0.3 to 0.5 while the nose was still
/// coming round, and the power drove the pirouette (LEMG 12 SR22 and LILB 03
/// Evolution, 2026-09-29: 10.8 and 24.6 degrees off at the first tick,
/// swinging +/-12 and through 29 degrees at 0.41 to 0.59 of lever). Once
/// straight (or rolling past [`ROLL_ALIGN_HOLD_KTS`] at its first tick) it
/// never holds again: a gust-yaw at speed is the roll law's.
///
/// A hold HOLDS the aircraft ([`RollAlign::creep`]): a creep over its speed
/// is stopped on both brakes with the lever at idle, and speed never lets
/// the power up. LILB 03 Evolution (1790732094, 2026-09-30): held at 0.3 of
/// lever, the PT6 at 300 to 330 ft-lb ran the aircraft from 0.2 to 6.6 kt
/// in 5 s while the pivot took the nose from 16.6 to 4.4 degrees off its
/// target; the 6 kt gate then let the power up (`roll_aligned` held_s
/// 5.06, err 4.4), the roll diverged to 7.7 degrees and rejected at 68 kt,
/// 9.4 m off. Pure.
#[derive(Debug, Default, Clone)]
pub struct RollAlign {
    pub(super) held_s: f64,
    pub(super) done: bool,
    /// Over the creep: stopped until back under it.
    pub(super) over: bool,
    /// The standing-yaw trim, signed brake (+ right), the line-up's own
    /// ([`crate::autotaxi::LineUpAlign`]).
    pub(super) trim: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RollAlignState {
    /// Power at the pivot's, the ramp waiting.
    Hold,
    /// Straight (or rolling): the roll's own power.
    Go,
    /// [`ROLL_ALIGN_MAX_S`] held and still crooked.
    Failed,
}

impl RollAlign {
    pub fn step(&mut self, err_deg: f64, yaw_dps: f64, gs_kts: f64, dt: f64) -> RollAlignState {
        if self.done {
            return RollAlignState::Go;
        }
        let rolling_start = self.held_s == 0.0 && gs_kts >= ROLL_ALIGN_HOLD_KTS;
        if rolling_start
            || (err_deg.abs() <= crate::autotaxi::ALIGNED_DEG && yaw_dps.abs() <= ROLL_ALIGNED_DPS)
        {
            self.done = true;
            return RollAlignState::Go;
        }
        self.held_s += dt.max(0.0);
        if self.held_s > ROLL_ALIGN_MAX_S || gs_kts > ROLL_ALIGN_RUNAWAY_KTS {
            return RollAlignState::Failed;
        }
        RollAlignState::Hold
    }

    /// While it holds: the most lever the creep allows and the brake on
    /// BOTH wheels (added to the pivot's split). Under the creep the pivot's
    /// power and no brake; [`ROLL_ALIGN_OVER_KTS`] over it the lever at idle
    /// and the brakes on in proportion, until back under the creep. Pure.
    pub fn creep(&mut self, gs_kts: f64) -> (f64, f64) {
        if gs_kts > ROLL_ALIGN_CREEP_KTS + ROLL_ALIGN_OVER_KTS {
            self.over = true;
        } else if gs_kts <= ROLL_ALIGN_CREEP_KTS {
            self.over = false;
        }
        if !self.over {
            return (ROLL_PIVOT_THR, 0.0);
        }
        let brake = (ROLL_ALIGN_BRAKE_MIN
            + (gs_kts - ROLL_ALIGN_CREEP_KTS).max(0.0) * ROLL_ALIGN_BRAKE_PER_KT)
            .min(ROLL_ALIGN_BRAKE_MAX);
        (0.0, brake)
    }

    /// The pivot's split while it holds, with the standing yaw trimmed out
    /// as the line-up's alignment does: a creep held 4 to 5 degrees off by
    /// the propeller's pull against the pivot's 0.03 of brake (1790732094,
    /// 2026-09-30, the err 6.1 to 4.4 with the yaw dying 1.5 to 0.4 deg/s)
    /// is otherwise a creep that never straightens. Pure.
    pub fn trimmed(
        &mut self,
        (l, r): (f64, f64),
        err_deg: f64,
        yaw_dps: f64,
        gs_kts: f64,
        dt: f64,
    ) -> (f64, f64) {
        self.trim = crate::autotaxi::align_trim_step(self.trim, err_deg, yaw_dps, gs_kts, dt);
        crate::autotaxi::with_align_trim((l, r), self.trim, yaw_dps)
    }

    /// Seconds the power has waited for the nose.
    pub fn held_s(&self) -> f64 {
        self.held_s
    }

    /// Whether it has let the power come up.
    #[cfg(test)]
    pub fn done(&self) -> bool {
        self.done
    }
}

/// The lever while [`RollAlign`] holds: never over the pivot's.
#[cfg(test)]
pub fn aligning_throttle(want: f64, holding: bool) -> f64 {
    aligning_throttle_capped(want, holding, ROLL_PIVOT_THR)
}

/// [`aligning_throttle`] under the creep's own cap ([`RollAlign::creep`]).
pub fn aligning_throttle_capped(want: f64, holding: bool, cap: f64) -> f64 {
    if holding {
        want.min(ROLL_PIVOT_THR).min(cap)
    } else {
        want
    }
}

/// The pivot's split with the creep's brake on both wheels. Pure.
pub fn with_hold_brake((l, r): (f64, f64), both: f64) -> (f64, f64) {
    ((l + both).min(1.0), (r + both).min(1.0))
}

/// Full brake-steering authority on the roll below this groundspeed...
pub(super) const ROLL_BRAKE_FULL_KTS: f64 = 20.0;
/// ...and none above this, where the rudder steers.
pub(super) const ROLL_BRAKE_OFF_KTS: f64 = 40.0;

/// Whether the live airframe steers on its brakes.
pub(super) fn brake_steers() -> bool {
    crate::aircraft::nose_castors()
        || crate::aircraft::live_profile()
            .and_then(|p| p.taxi.as_ref().map(|t| t.brake_steer))
            .unwrap_or(false)
}

/// Rotation waits for a roughly straight nose: pulling the aircraft off
/// yawed launches it sideways off the strip (the second sortie rotated 39
/// degrees off heading, from the grass). Unless the runway is running out,
/// where flying beats the overrun.
pub fn rotate_ok(err_deg: f64, remaining_m: f64) -> bool {
    err_deg.abs() < 15.0 || remaining_m < 400.0
}
