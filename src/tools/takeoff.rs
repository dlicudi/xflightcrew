//! `auto_takeoff` — the copilot flies the takeoff roll and initial climb.
//!
//! The arming end of [`crate::autotakeoff`]: quick gates here (replay, on
//! the ground, not already rolling, the ATC clearance when a tower is
//! running, the steer channel), then the background loop re-validates the
//! runway itself — the apt.dat scan is seconds, too slow for this reply —
//! and speaks its own "Rolling" or refusal. Distinct from `autoland` on
//! purpose: that one is an emergency; this is a normal crew duty, so it
//! REFUSES without a takeoff clearance instead of assuming one.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::PropProfile;
use crate::autotakeoff;
use crate::party_line;
use crate::tools::autopilot::ApConfig;
use crate::tower;

pub struct AutoTakeoff {
    pub cfg: ApConfig,
    pub vr_kts: f64,
    pub vy_kts: f64,
    /// Initial climb speed (see `autotakeoff::Params::initial_climb_kts`).
    pub initial_climb_kts: f64,
    pub climb_fpm: f64,
    pub retract_gear: bool,
    /// Flap-handle ratio set for takeoff and retracted climbing out (from the
    /// profile's `flaps.takeoff_ratio`); `None` leaves the flaps alone.
    pub takeoff_flap_ratio: Option<f64>,
    pub flap_schedule: Vec<(f64, f64)>,
    /// POH takeoff ground roll, meters, when the profile knows it.
    pub takeoff_roll_m: Option<f64>,
    /// Max takeoff weight, kg — the weight `takeoff_roll_m` is quoted AT, so
    /// the roll can be scaled to what the aircraft actually weighs today.
    pub mtow_kg: Option<f64>,
    /// Constant-speed prop levers, advanced to full-forward before the roll.
    /// A feathered turboprop makes no thrust at any power (`None` on pistons).
    pub prop: Option<PropProfile>,
    /// This airframe's tail-strike attitude, degrees (profile).
    pub tail_strike_deg: Option<f64>,
    pub pitch_authority_down: Option<f64>,
    /// Vne/Vmo for the climb's overspeed protection.
    pub vne_kts: Option<f64>,
    /// Who gets the aircraft at the handoff. False for the registry's copy —
    /// a takeoff the PILOT asked for hands to the pilot, which is the whole
    /// point of asking. True for full auto's copy: its Flight mission arms
    /// the takeoff through THIS tool, and "your aircraft" at 1,000 ft with
    /// `fly_route` about to arm the enroute legs is the handoff-truth bug in
    /// its last remaining home (audit, 2026-08-02).
    pub handoff_to_caller: bool,
    /// The airframe's park-brake valve SET sense (profile), for the brakes-off
    /// at brake release.
    pub park_brake_valve_set: Option<f64>,
}

#[derive(Deserialize, Default)]
struct Args {
    /// true = abort/reject the takeoff.
    #[serde(default)]
    off: bool,
}

#[async_trait]
impl Tool for AutoTakeoff {
    fn name(&self) -> &str {
        "auto_takeoff"
    }

    fn description(&self) -> &str {
        "The copilot flies the takeoff: full power, steers the centreline, \
         rotates at Vr, climbs out on the autopilot and hands back at a \
         thousand feet. Use ONLY when the pilot asks the copilot to fly the \
         takeoff itself: 'take off for me', 'you have the takeoff', 'your \
         takeoff', 'auto takeoff'. off=true for 'abort takeoff' / 'reject \
         takeoff' / 'cancel the takeoff'. (A readback like 'cleared for \
         takeoff runway 27' is the pilot talking to ATC, never this.)"
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "off": {"type": "boolean", "description": "true = abort/reject the takeoff"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        use std::sync::atomic::Ordering::Relaxed;
        let a: Args = serde_json::from_value(args.clone()).unwrap_or_default();
        if a.off {
            return Ok(if autotakeoff::ACTIVE.load(Relaxed) {
                // The loop does the arresting (throttle, brakes) and speaks
                // the state; this reply is just the acknowledgment.
                autotakeoff::REJECT.store(true, Relaxed);
                "Aborting.".to_string()
            } else {
                "I wasn't flying the takeoff.".to_string()
            });
        }
        if autotakeoff::ACTIVE.load(Relaxed) {
            return Ok("I already have the takeoff.".to_string());
        }
        if ctx
            .xplane
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5)
        {
            return Ok("We're in replay. Go back to live and ask again.".to_string());
        }
        if ctx
            .xplane
            .get_float(crate::datarefs::ON_GROUND_ANY)
            .await
            .unwrap_or(1.0)
            < 0.5
        {
            return Ok("We're already flying.".to_string());
        }
        // A drive in progress (line-up, auto taxi) is NOT killed by asking
        // for the takeoff — the second sortie's ask mid-line-up stood the
        // governor down, the takeoff then refused, and the aircraft coasted
        // uncontrolled for half a minute (LXGB, 2026-07-20). Let the drive
        // finish; it announces when it's done.
        if crate::autotaxi::ACTIVE.load(Relaxed) {
            return Ok(
                "I'm still driving us. When I call us lined up, ask for the takeoff again."
                    .to_string(),
            );
        }
        // A takeoff starts from a standstill: the roll law owns the whole
        // acceleration, and engaging on a moving, swinging aircraft is how
        // the second sortie left the pavement.
        let gs_kts = ctx
            .xplane
            .get_float("sim/flightmodel/position/groundspeed")
            .await
            .unwrap_or(0.0)
            * 1.943_844;
        if gs_kts > 5.0 {
            return Ok("We're still moving. Let us come to a stop, then ask again.".to_string());
        }
        // The clearance gate: when a tower is running, a takeoff needs its
        // clearance — this is a normal duty, not an emergency (autoland
        // deliberately skips this). No station (copilot-only mode, real
        // ATC on the radio) means the pilot owns the clearance question.
        match party_line::clearance_state() {
            None | Some(tower::State::TakeoffCleared) => {}
            Some(tower::State::LinedUp) => {
                return Ok(if party_line::takeoff_hold_armed() {
                    "The tower is holding us for traffic. When the clearance comes through, ask me again."
                        .to_string()
                } else {
                    "We're only cleared to line up and wait. No takeoff clearance yet.".to_string()
                });
            }
            Some(_) => {
                return Ok(
                    "We don't have a takeoff clearance. Call the tower, then I'll fly it."
                        .to_string(),
                );
            }
        }
        // What the aircraft weighs NOW: the roll figure in the profile is
        // quoted at max takeoff weight. A failed read falls back to that
        // figure, which is the conservative end.
        let weight_now = ctx
            .xplane
            .get_float(sim::datarefs::TOTAL_WEIGHT_KG)
            .await
            .ok()
            .filter(|w| w.is_finite() && *w > 0.0);
        if !crate::autotaxi::steer_channel_ok(ctx.xplane).await {
            return Ok(
                "I can't steer. The XVoiceLayer plugin needs updating, so this takeoff is yours."
                    .to_string(),
            );
        }
        // The rotation flies the ELEVATOR and AILERONS through the plugin's
        // pitch/roll channels; a plugin that predates either would roll fine
        // and then reject (or sail off the far end) at rotation. Probe both
        // now (neutral writes that expire in a second, parked).
        let (pp, pr) = tokio::join!(
            ctx.xplane.set_float("xcopilot/pitch", 0.0),
            ctx.xplane.set_float("xcopilot/roll", 0.0)
        );
        if pp.is_err() || pr.is_err() {
            return Ok(
                "I can't reach the yoke. The XVoiceLayer plugin needs updating, so this takeoff is yours."
                    .to_string(),
            );
        }
        // Props full forward before the throttle ramps — feathered blades make
        // no thrust and the roll would stall against the stagnation reject.
        super::prop::advance_for_ground_roll(ctx.xplane, &self.prop).await;
        if !autotakeoff::start_scoped(
            ctx.config.xplane_base_url.clone(),
            ctx.voice.cloned(),
            autotakeoff::Params {
                // Registry copy: the pilot asked, the pilot gets the
                // aircraft. Full auto's copy sets true — see the field doc.
                handoff_to_caller: self.handoff_to_caller,
                park_brake_valve_set: self.park_brake_valve_set,
                tail_strike_deg: self.tail_strike_deg,
                pitch_authority_down: self.pitch_authority_down,
                vne_kts: self.vne_kts,
                apt_dat: super::autoland::apt_dat_path(),
                cfg: self.cfg.clone(),
                vr_kts: self.vr_kts,
                vy_kts: self.vy_kts,
                initial_climb_kts: self.initial_climb_kts,
                climb_fpm: self.climb_fpm,
                retract_gear: self.retract_gear,
                takeoff_flap_ratio: self.takeoff_flap_ratio,
                flap_schedule: self.flap_schedule.clone(),
                cleared_runway: party_line::clearance_runway(),
                // The roll is a MAX-WEIGHT figure; scale it to today's. An
                // A330 on a light sector was refused a 2,549 m runway by
                // eleven metres, from every position on it (2026-09-20).
                needed_m: self.takeoff_roll_m.map_or(500.0, |m| {
                    crate::aircraft::takeoff_roll_m(m, weight_now, self.mtow_kg) * 1.4
                }),
                handoff_agl_ft: autotakeoff::DEFAULT_HANDOFF_AGL_FT,
            },
            ctx.xplane.cancellation(),
        ) {
            return Ok(
                "The previous automation is still stopping. Ask again once it has stopped."
                    .to_string(),
            );
        }
        Ok(
            "My takeoff. Checking the runway. Say abort takeoff or my controls any time."
                .to_string(),
        )
    }
}
