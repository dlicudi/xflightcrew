//! `auto_taxi` — the copilot taxis the aircraft to the holding point.
//!
//! The arming end of [`crate::autotaxi`]: validates that there is a cleared
//! route to drive and that we're actually on it, arms the taxi-speed
//! governor through the same `hold_speed` path a spoken taxi uses (so every
//! parking-brake and brake-toggle gate applies unchanged), then hands the
//! steering to the background loop. Distinct from `hold_speed`'s taxi mode
//! on purpose: that one is "I have the throttle, you steer" — this one is
//! the copilot steering too, and it only exists once ATC has published a
//! route to steer along.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{speedhold, Tool, ToolCtx};
use crate::aircraft::PropProfile;
use crate::autotaxi;
use crate::party_line;
use crate::tower;

pub struct AutoTaxi {
    /// The airframe's park-brake valve SET sense, passed through to the
    /// inner `hold_speed` arm so its release gates work unchanged.
    pub park_brake_valve_set: Option<f64>,
    /// How briskly this airframe taxis — the pace when the pilot names none,
    /// and the bend caps the route's schedule falls to. Built from the profile
    /// (so a swap rebuilds it), never read from a static.
    pub pace: autotaxi::Pace,
    /// Constant-speed prop levers, when the airframe has them. Advanced out of
    /// feather at arm time — a turboprop makes no taxi thrust with feathered
    /// blades however far the power lever comes up. `None` on the pistons.
    pub prop: Option<PropProfile>,
    /// This arm is a CALLER's, not a pilot's: it asked for one specific route
    /// to be driven, so "a drive is already running" is a failure, not a
    /// reassurance.
    ///
    /// The spoken path wants the opposite — a pilot who says "taxi" while
    /// already taxiing should be told so, kindly. But taxi-in used the same
    /// door, and when its vacate over-ran, the reassuring `Ok` read to the
    /// caller as "my stand drive started": it then watched the OLD drive,
    /// took its arrival for its own, and secured the aircraft on a taxiway
    /// (BIKF, 2026-09-20). A caller that names a route gets an error.
    pub must_start: bool,
}

#[derive(Deserialize, Default)]
struct Args {
    /// Taxi pace, knots. Absent = the airframe's own pace.
    #[serde(default)]
    knots: Option<f64>,
    /// true = stop auto-taxiing and hand everything back.
    #[serde(default)]
    off: bool,
}

#[async_trait]
impl Tool for AutoTaxi {
    fn name(&self) -> &str {
        "auto_taxi"
    }

    fn description(&self) -> &str {
        "The copilot taxis the aircraft along the CLEARED taxi route to the \
         holding point: steering, throttle and brakes, stopping short of the \
         hold line. Use ONLY when the pilot asks the copilot to do the taxiing \
         or steering itself: 'taxi for me', 'you taxi', 'you have the taxi', \
         'take us to the holding point', 'auto taxi'. Optional knots sets the \
         pace. off=true for 'stop auto taxi' / 'cancel auto taxi'. (A plain \
         'taxi at 10 knots' where the pilot steers is hold_speed, not this.)"
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "knots": {"type": "number", "description": "taxi pace, knots; absent = this airframe's normal taxi pace"},
                "off": {"type": "boolean", "description": "true = stop auto-taxiing, hand everything back"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        use std::sync::atomic::Ordering::Relaxed;
        let a: Args = serde_json::from_value(args.clone()).unwrap_or_default();
        if a.off {
            let was_driving = autotaxi::ACTIVE.swap(false, Relaxed);
            let had_speed = speedhold::ACTIVE.swap(false, Relaxed);
            return Ok(if was_driving || had_speed {
                "Auto taxi off. You have the aircraft.".to_string()
            } else {
                "I wasn't taxiing us.".to_string()
            });
        }
        // Already driving: a pace request retargets, anything else reassures
        // — unless the caller named a route, for whom this is a failure.
        if autotaxi::ACTIVE.load(Relaxed) {
            if self.must_start && a.knots.is_none() {
                return Err(anyhow::anyhow!(
                    "A taxi drive is already running; I can't start another."
                ));
            }
            return Ok(if let Some(k) = a.knots {
                let k = speedhold::clamp_taxi_target(k);
                speedhold::set_taxi_target(k);
                format!("Making it {k:.0} knots.")
            } else {
                "I already have the taxi. We're on our way to the holding point.".to_string()
            });
        }
        // A replay can't be taxied: the position is a recording, and the
        // governor + steering would spring on the pilot at live.
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
            .unwrap_or(0.0)
            < 0.5
        {
            return Ok("We're airborne. No taxiing up here.".to_string());
        }
        // The route is ATC's: no clearance, no guidance, no driving.
        let Some(watch) = party_line::with_taxi_route(|r| r.cloned()) else {
            return Ok(
                "We don't have a taxi clearance yet. Call the tower and request taxi, then I'll take it."
                    .to_string(),
            );
        };
        let Some(mut route) = autotaxi::Route::build(&watch) else {
            return Ok(
                "The cleared route's too short to drive. We're basically there.".to_string(),
            );
        };
        route.pace = self.pace;
        let (Ok(lat), Ok(lon), Ok(psi)) = (
            ctx.xplane.get_float(autotaxi::LAT).await,
            ctx.xplane.get_float(autotaxi::LON).await,
            ctx.xplane.get_float(autotaxi::PSI_TRUE).await,
        ) else {
            return Ok("I can't read our position right now.".to_string());
        };
        let Some((progress, lateral)) = route.fix(lat, lon) else {
            return Ok("I can't make out the taxi route from here.".to_string());
        };
        // Way off the route the projection stops meaning anything — that's a
        // refusal. Merely off it, or facing the wrong way, is a JOIN: the
        // loop turns toward the cleared pavement at bend speed and picks the
        // route up where the projection says.
        if lateral > autotaxi::MAX_JOIN_M {
            return Ok(format!(
                "We're about {} meters off the cleared route. Get us near it and ask again.",
                (lateral / 10.0).round() as i64 * 10
            ));
        }
        if route.remaining(progress) <= 1.0 {
            return Ok("We're already at the holding point.".to_string());
        }
        let err = route.heading_err_deg(lat, lon, psi, progress, 20.0);
        let joining = autotaxi::join_allowance_m(lateral, err) > 0.0;
        // No steering channel (plugin missing or old) — offer what still works.
        if !autotaxi::steer_channel_ok(ctx.xplane).await {
            return Ok(
                "I can't steer. The XVoiceLayer plugin needs updating. I can hold the taxi speed while you steer."
                    .to_string(),
            );
        }
        let base = speedhold::clamp_taxi_target(a.knots.unwrap_or(self.pace.straight)).max(3.0);
        // Arm the governor through the front door: every parking-brake and
        // held-brake gate in hold_speed applies to an auto taxi unchanged.
        let arm_line = speedhold::HoldSpeed {
            approach_kts: None,
            park_brake_valve_set: self.park_brake_valve_set,
        }
        .execute(&json!({ "knots": base }), ctx)
        .await?;
        if !speedhold::taxi_hold_active() {
            // The governor refused (parking brake stuck, etc.) — its answer
            // says why, and it's ours too.
            return Ok(arm_line);
        }
        // Props out of feather before the governor opens the throttle — a
        // turboprop with feathered blades makes no thrust and never rolls.
        super::prop::advance_for_ground_roll(ctx.xplane, &self.prop).await;
        autotaxi::start(
            ctx.config.xplane_base_url.clone(),
            ctx.voice.cloned(),
            route,
            base,
        );
        // An arrival route ends at a parking stand, not a holding point — call
        // it by the right name (the tower cleared us "to stand ...").
        let dest = if party_line::taxi_to_stand() {
            let s = crate::station::spoken_stand(&watch.holding_point);
            if s.is_empty() {
                "parking".to_string()
            } else {
                s
            }
        } else {
            let hp = crate::station::spoken_taxiway(&watch.holding_point);
            if hp.is_empty() {
                "the holding point".to_string()
            } else {
                format!("holding point {hp}")
            }
        };
        // A route with no named legs (a sparse network, or a stand a few
        // metres off the taxiway) is "then the holding point", not "via ."
        // (heard at Lake Hood, 2026-09-10).
        let via = if watch.via.trim().is_empty() {
            String::new()
        } else {
            format!(" via {}", watch.via)
        };
        Ok(if joining {
            format!(
                "My taxi. I'll get us onto the route first, then {dest}{via}. Your pedals or my controls take it back any time.",
            )
        } else {
            format!(
                "My taxi. {dest}{via}, {base:.0} knots. Your pedals or my controls take it back any time.",
            )
        })
    }
}

/// `line_up` — the copilot drives from the hold-short position onto the
/// runway and stops aligned on the centreline: the missing middle between
/// auto taxi (ends at the hold line) and auto takeoff (needs the aircraft
/// already pointing down the strip). Same governor front door, same
/// steering loop — the route is synthetic ([`autotaxi::line_up_path`]),
/// picked against the CLEARED runway so a "line up and wait runway two
/// seven" can never be driven onto the 09 end.
pub struct LineUp {
    pub park_brake_valve_set: Option<f64>,
    /// POH ground roll, meters — the full-length check: a line-up that
    /// leaves less than 1.4× this ahead backtracks to the threshold
    /// instead (LXGB's taxiways only reach the east half; holding point
    /// Charlie left an SR22 580 m for a 09 departure, 2026-07-22).
    pub takeoff_roll_m: Option<f64>,
    /// Constant-speed prop levers, advanced out of feather before the drive
    /// (a line-up is an auto taxi — same feathered-blade wall). `None` on
    /// the pistons.
    pub prop: Option<PropProfile>,
    /// This airframe's pacing — a line-up rides the TURN cap (a runway entry
    /// is a bend end to end), a backtrack the straight-leg pace.
    pub pace: autotaxi::Pace,
}

/// A full-length backtrack is a long straight run down the strip — taxi it at a
/// brisk pace, not the tight line-up crawl (LXGB's ~1 km 09 backtrack took 6
/// minutes at 6 kt and full auto timed out). The governor still auto-caps
/// through the turn-around and ramps to the stop at the numbers. Floored here
/// so a GA airframe keeps the 14 kt that fixed that timeout, while a jet
/// backtracks at its own (higher) straight-leg pace.
const BACKTRACK_FLOOR_KTS: f64 = 14.0;

#[async_trait]
impl Tool for LineUp {
    fn name(&self) -> &str {
        "line_up"
    }

    fn description(&self) -> &str {
        "The copilot drives the aircraft from the hold-short position onto \
         the runway and stops lined up on the centreline — backtracking to \
         the threshold first when the entry leaves too little runway (or \
         when asked: 'backtrack runway 09'). Use ONLY when the pilot asks \
         the copilot to do it: 'line us up', 'get us lined up', 'take us \
         onto the runway', 'backtrack'. (The pilot READING BACK 'line up \
         and wait runway 27' is talking to ATC, never this. Stopping it is \
         'my controls' / 'stop'.)"
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {
            "backtrack": {"type": "boolean", "description": "true = the pilot explicitly asked to backtrack for the full length"}
        } })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        use std::sync::atomic::Ordering::Relaxed;
        if autotaxi::ACTIVE.load(Relaxed) {
            return Ok("I'm already driving. Let me finish, then ask again.".to_string());
        }
        if crate::autotakeoff::ACTIVE.load(Relaxed) {
            return Ok("I'm flying the takeoff already.".to_string());
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
            .unwrap_or(0.0)
            < 0.5
        {
            return Ok("We're airborne.".to_string());
        }
        // Driving onto a runway needs the tower's word for it when a tower
        // is running: "line up and wait" or the takeoff clearance itself.
        // No station (copilot-only mode, real ATC) leaves it to the pilot.
        match party_line::clearance_state() {
            None | Some(tower::State::LinedUp) | Some(tower::State::TakeoffCleared) => {}
            Some(tower::State::TaxiCleared) => {
                return Ok(
                    "We're only cleared to taxi. Get the line-up or takeoff clearance first, then I'll put us on."
                        .to_string(),
                );
            }
            Some(_) => {
                return Ok("We're not cleared onto the runway.".to_string());
            }
        }
        let (Ok(lat), Ok(lon)) = (
            ctx.xplane.get_float(autotaxi::LAT).await,
            ctx.xplane.get_float(autotaxi::LON).await,
        ) else {
            return Ok("I can't read our position right now.".to_string());
        };
        // The current heading makes leg one roll forward along the taxiway
        // lead-on onto the centreline instead of cutting a diagonal to it.
        let heading = ctx.xplane.get_float(autotaxi::PSI_TRUE).await.ok();
        if !autotaxi::steer_channel_ok(ctx.xplane).await {
            return Ok(
                "I can't steer. The XVoiceLayer plugin needs updating, so the line-up is yours."
                    .to_string(),
            );
        }
        // The apt.dat scan is seconds of file streaming — off the executor.
        let apt = super::autoland::apt_dat_path();
        let cands = tokio::task::spawn_blocking(move || {
            crate::autoland::nearby_runways(&apt, lat, lon, 1.0)
        })
        .await
        .ok()
        .and_then(|r| r.ok())
        .unwrap_or_default();
        let cleared = party_line::clearance_runway();
        let force_backtrack = args
            .get("backtrack")
            .and_then(Value::as_bool)
            .unwrap_or(false);
        // Full-length check: the POH roll with the takeoff tool's 1.4
        // margin, floored at the plain line-up's own minimum.
        let needed_m = self.takeoff_roll_m.map_or(400.0, |m| (m * 1.4).max(400.0));
        // The cleared route's trimmed tail — the authored yellow lead-on from
        // the hold onto the centreline. The line-up drives it when it's ours.
        let lead_on =
            party_line::with_taxi_route(|r| r.map(|w| w.lead_on.clone()).unwrap_or_default());
        let Some((rwy, plan)) = autotaxi::pick_line_up_plan(
            &cands,
            lat,
            lon,
            cleared.as_deref(),
            needed_m,
            force_backtrack,
            heading,
            (!lead_on.is_empty()).then_some(&lead_on[..]),
        ) else {
            return Ok(match cleared {
                Some(c) => format!(
                    "I can't find runway {} next to us. Taxi us closer and ask again.",
                    crate::atis::spoken_runway(&c)
                ),
                None => "We're not next to a runway I can line up on.".to_string(),
            });
        };
        let spoken = crate::atis::spoken_runway(&rwy.id);
        let (path, backtracking) = match plan {
            autotaxi::LineUpPlan::Plain(p) => (p, false),
            autotaxi::LineUpPlan::Backtrack(p) => (p, true),
        };
        // Which maneuver got built, in the blackbox — a synthetic diagonal is
        // 3 points, a lead-on ride with its fillet arc is 7+; the "not
        // following the yellow line" reports were undiagnosable without this
        // (2026-07-24).
        crate::blackbox::record(
            "autotaxi",
            serde_json::json!({
                "event": "line_up_path", "pts": path.len(),
                "lead_on_pts": lead_on.len(), "backtracking": backtracking,
                "runway": rwy.id,
            }),
        );
        let watch = crate::station::TaxiWatch {
            path,
            via: String::new(),
            seg: Vec::new(),
            holding_point: String::new(),
            lead_on: Vec::new(),
        };
        let Some(mut route) = autotaxi::Route::build(&watch) else {
            return Ok(
                "We're basically on the runway already. Roll forward a touch and we're lined up."
                    .to_string(),
            );
        };
        route.pace = self.pace;
        route.done_line = Some(if backtracking {
            "At the numbers. Coming around.".to_string()
        } else {
            format!("Lined up runway {spoken}. Holding position.")
        });
        // A backtrack taxis briskly down the strip; a plain line-up creeps.
        let drive_kts = if backtracking {
            self.pace.straight.max(BACKTRACK_FLOOR_KTS)
        } else {
            self.pace.turn
        };
        // Governor through the front door: parking-brake and held-brake
        // gates apply to a line-up unchanged.
        let arm_line = speedhold::HoldSpeed {
            approach_kts: None,
            park_brake_valve_set: self.park_brake_valve_set,
        }
        .execute(&json!({ "knots": drive_kts }), ctx)
        .await?;
        if !speedhold::taxi_hold_active() {
            return Ok(arm_line);
        }
        // Props out of feather before the drive rolls (same wall as auto taxi).
        super::prop::advance_for_ground_roll(ctx.xplane, &self.prop).await;
        if backtracking {
            autotaxi::start_backtrack(
                ctx.config.xplane_base_url.clone(),
                ctx.voice.cloned(),
                route,
                rwy.clone(),
                drive_kts,
            );
            return Ok(format!(
                "Backtracking runway {spoken} for the full length. Your pedals or my controls take it back any time."
            ));
        }
        autotaxi::start(
            ctx.config.xplane_base_url.clone(),
            ctx.voice.cloned(),
            route,
            drive_kts,
        );
        Ok(format!(
            "Lining us up, runway {spoken}. Your pedals or my controls take it back any time."
        ))
    }
}

/// `taxi_to_parking` — the copilot requests the taxi-in from ATC and drives the
/// aircraft to the nearest stand after landing. The arrival mirror of the
/// departure taxi: [`AutoTaxi`] drives an already-cleared route, this ASKS for
/// it first, then drives it (see [`crate::taxi_in`]).
pub struct TaxiToParking {
    pub park_brake_valve_set: Option<f64>,
    /// Constant-speed prop levers, so a turboprop unfeathers before the taxi.
    pub prop: Option<PropProfile>,
    /// This airframe's pacing, handed to the arrival drive.
    pub pace: autotaxi::Pace,
}

#[async_trait]
impl Tool for TaxiToParking {
    fn name(&self) -> &str {
        "taxi_to_parking"
    }

    fn description(&self) -> &str {
        "The copilot taxis the aircraft to PARKING after landing: it requests \
         the taxi-in clearance from the tower, then drives the cleared route to \
         the nearest stand. Use when the pilot asks the copilot to taxi in or \
         park: 'taxi to parking', 'take us to the stand', 'park us', 'park the \
         aircraft', 'taxi to the gate', 'taxi to the ramp'. (A DEPARTURE 'taxi \
         for me' to the holding point is auto_taxi, not this.)"
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {} })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        use std::sync::atomic::Ordering::Relaxed;
        if autotaxi::ACTIVE.load(Relaxed) {
            return Ok("I'm already taxiing us.".to_string());
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
            .unwrap_or(0.0)
            < 0.5
        {
            return Ok("We're airborne. Land first, then I'll taxi us in.".to_string());
        }
        // Vacate first only if we are still ON the runway — see
        // `taxi_in::vacate_runway` for why a standing landing clearance is
        // the signal and a bare `clearance_runway` is not.
        let vacate_from = crate::taxi_in::vacate_runway();
        crate::taxi_in::start(
            ctx.config.xplane_base_url.clone(),
            ctx.config.clone(),
            ctx.voice.cloned(),
            self.park_brake_valve_set,
            self.prop.clone(),
            self.pace,
            vacate_from,
        );
        Ok("Taxiing to parking — getting the clearance.".to_string())
    }
}

#[cfg(test)]
mod tests {
    /// Taxi-in names its route, so its arm must FAIL when a drive is already
    /// running rather than be reassured by one. The spoken path keeps the
    /// kind answer: a pilot saying "taxi" while taxiing should be told so.
    ///
    /// Read from the source, because the bug was a construction site and not
    /// a branch: taxi-in built the same tool the pilot's words build, got
    /// back "I already have the taxi", read that `Ok` as its own stand drive
    /// starting, then watched the OLD drive and secured on its arrival 140 m
    /// from the nearest stand (BIKF, 2026-09-20).
    #[test]
    fn taxi_in_is_the_caller_that_must_start_its_own_drive() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let sites = |text: &str| -> Vec<bool> {
            text.match_indices("AutoTaxi {")
                .map(|(i, _)| {
                    let tail = &text[i..];
                    let end = tail.find('}').unwrap_or(tail.len());
                    tail[..end].contains("must_start: true")
                })
                .collect()
        };
        let taxi_in = std::fs::read_to_string(root.join("taxi_in.rs")).unwrap();
        let built = sites(&taxi_in);
        assert!(!built.is_empty(), "taxi_in builds the tool");
        assert!(
            built.iter().all(|&b| b),
            "every taxi-in arm names its route: {built:?}"
        );
        // ...and the pilot's own door keeps the reassurance.
        let registry = std::fs::read_to_string(root.join("tools").join("mod.rs")).unwrap();
        assert!(
            sites(&registry).iter().all(|&b| !b),
            "the spoken tool stays forgiving"
        );
    }
}
