//! A helicopter's engine start and shutdown, from the profile's `heli_start`.
//!
//! The procedure was measured by hand on the S-76C (2026-09-22) before a line
//! of this was written: batteries on, rotor brake off, collective down; per
//! engine the throttle to idle (throttle 0 is OFF, anything above it lets fuel
//! in, the mixture follows by itself) and the starter HELD until the engine
//! settles at about 62 % N1; then both throttles to FLY, the generators on,
//! and the rotor read back at its governed 313 rpm. Nothing here writes the
//! collective: a lever the pilot's hand may be on is refused, not moved.

use crate::datarefs::ON_GROUND_ANY;
use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};
use std::time::Duration;

use super::{Tool, ToolCtx};
use crate::aircraft::HeliStartProfile;

const ENGN_RUNNING: &str = "sim/flightmodel/engine/ENGN_running";
const N1_PERCENT: &str = "sim/cockpit2/engine/indicators/N1_percent";
const BUS_VOLTS: &str = "sim/cockpit2/electrical/bus_volts";
const Y_AGL: &str = "sim/flightmodel/position/y_agl";
const GROUNDSPEED: &str = "sim/flightmodel/position/groundspeed";
const PSI: &str = "sim/flightmodel/position/psi";
/// Collective above this is "up": a start with it there is refused.
const COLLECTIVE_DOWN_MAX: f64 = 0.05;
/// The bus a starter needs (the S-76 read 24 V on batteries alone).
const START_VOLTS: f64 = 20.0;
/// The rotor is "at speed" within this fraction of its governed figure.
const ROTOR_TOLERANCE: f64 = 0.03;
/// How long the rotor may take to reach its governed speed at FLY (it took
/// under ten seconds live).
const ROTOR_WAIT_SECS: u32 = 30;
/// Re-arm cadence for a held starter: shorter than each 2 s hold, so the
/// switch never springs back mid-start.
const STARTER_HOLD_S: f64 = 2.0;
const STARTER_REARM_MS: u64 = 1500;

async fn elem(ctx: &ToolCtx<'_>, dref: &str, i: usize) -> Option<f64> {
    ctx.xplane
        .get_float_array(dref)
        .await
        .ok()
        .and_then(|v| v.get(i).copied())
}

async fn running(ctx: &ToolCtx<'_>, i: usize) -> bool {
    elem(ctx, ENGN_RUNNING, i).await.is_some_and(|r| r > 0.5)
}

/// What one engine's start came to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EngineStart {
    AlreadyRunning,
    Lit,
    /// Cranked with fuel in and never reached idle.
    NoLight,
}

/// The spoken line for the whole start, from each engine's outcome and the
/// rotor's. Engines are "number one" and "number two", as a helicopter crew
/// says them.
pub fn start_line(engines: &[EngineStart], rotor_at_speed: Option<bool>) -> String {
    let failed: Vec<usize> = engines
        .iter()
        .enumerate()
        .filter(|(_, e)| **e == EngineStart::NoLight)
        .map(|(i, _)| i + 1)
        .collect();
    if !failed.is_empty() {
        let which = failed
            .iter()
            .map(|n| format!("number {n}"))
            .collect::<Vec<_>>()
            .join(" and ");
        return format!(
            "Engine {which} didn't light, so its throttle is back to off. \
             Check the fuel levers and try again."
        );
    }
    let n = engines.len();
    let engines_said = if n == 2 {
        "Both engines".to_string()
    } else {
        format!("All {n} engines")
    };
    match rotor_at_speed {
        Some(true) => format!("{engines_said} running, throttles to fly, rotor at 100 percent."),
        Some(false) => format!(
            "{engines_said} running and at fly, but the rotor hasn't reached its governed \
             speed. Check the rotor RPM."
        ),
        None => format!("{engines_said} running, throttles to fly."),
    }
}

#[derive(Clone)]
pub struct StartHelicopter {
    p: HeliStartProfile,
    engines: usize,
    battery_on: Vec<String>,
}

impl StartHelicopter {
    pub fn new(p: HeliStartProfile, engines: usize, battery_on: Vec<String>) -> Self {
        Self {
            engines: engines.max(1).min(p.starters.len().max(1)),
            p,
            battery_on,
        }
    }
}

#[async_trait]
impl Tool for StartHelicopter {
    fn name(&self) -> &str {
        "start_engines"
    }
    fn description(&self) -> &str {
        "Start the helicopter's engines: batteries on, rotor brake off, each engine \
         started on the idle throttle with its starter held, then both throttles to \
         fly and the rotor brought to its governed speed. Use for 'start the \
         engines', 'engines on', 'start her up', 'spool them up'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let p = &self.p;
        if ctx.xplane.get_float(ON_GROUND_ANY).await? < 0.5 {
            return Ok("Negative, we're airborne. I won't run a start in flight.".to_string());
        }
        // The collective is the pilot's lever, and a start with it up is a
        // lift-off at the first rotor speed that can make one.
        match elem(ctx, &p.collective, 0).await {
            Some(c) if c <= COLLECTIVE_DOWN_MAX => {}
            Some(c) => {
                return Ok(format!(
                    "The collective is up, at {:.0} percent. Lower it fully, then ask again.",
                    c * 100.0
                ))
            }
            None => return Ok("I can't read the collective, so I won't start.".to_string()),
        }

        // Power for the starters.
        let volts = |v: Option<Vec<f64>>| v.map_or(0.0, |v| v.into_iter().fold(0.0, f64::max));
        if volts(ctx.xplane.get_float_array(BUS_VOLTS).await.ok()) < START_VOLTS {
            for cmd in &self.battery_on {
                let _ = ctx.xplane.run_command(cmd).await;
            }
            tokio::time::sleep(Duration::from_millis(800)).await;
        }
        let v = volts(ctx.xplane.get_float_array(BUS_VOLTS).await.ok());
        if v < START_VOLTS {
            return Ok(format!(
                "No power for the starters, only {v:.0} volts. Batteries on, then ask again."
            ));
        }

        if !lock_tail_wheel(ctx, p).await {
            return Ok("The tail wheel won't lock, so I haven't started.".to_string());
        }

        if let Some(brake) = &p.rotor_brake {
            let _ = ctx.xplane.set_float(brake, 0.0).await;
            if ctx.xplane.get_float(brake).await.is_ok_and(|b| b > 0.05) {
                return Ok("The rotor brake won't come off, so I haven't started.".to_string());
            }
        }

        let mut outcomes = Vec::with_capacity(self.engines);
        for i in 0..self.engines {
            if running(ctx, i).await {
                outcomes.push(EngineStart::AlreadyRunning);
                continue;
            }
            ctx.xplane
                .set_float_array_elem(&p.throttle, i, p.idle)
                .await?;
            let started = std::time::Instant::now();
            let mut lit = false;
            let mut peak_n1 = 0.0_f64;
            while started.elapsed().as_secs_f64() < p.crank_secs {
                let _ = ctx
                    .xplane
                    .run_command_for(&p.starters[i], STARTER_HOLD_S)
                    .await;
                tokio::time::sleep(Duration::from_millis(STARTER_REARM_MS)).await;
                let n1 = elem(ctx, N1_PERCENT, i).await.unwrap_or(0.0);
                peak_n1 = peak_n1.max(n1);
                if n1 >= p.idle_n1 && running(ctx, i).await {
                    lit = true;
                    break;
                }
            }
            crate::blackbox::record(
                "heli_start",
                json!({
                    "event": "engine", "engine": i, "lit": lit, "peak_n1": peak_n1,
                    "secs": started.elapsed().as_secs_f64(),
                }),
            );
            if !lit {
                // No fuel left flowing into an engine that did not catch.
                let _ = ctx.xplane.set_float_array_elem(&p.throttle, i, 0.0).await;
                outcomes.push(EngineStart::NoLight);
                return Ok(start_line(&outcomes, None));
            }
            outcomes.push(EngineStart::Lit);
        }

        // Both at FLY, generators on, the rotor read back at its governed speed.
        for i in 0..self.engines {
            ctx.xplane
                .set_float_array_elem(&p.throttle, i, p.fly)
                .await?;
        }
        for cmd in p.generators.iter().chain(&p.after_start) {
            let _ = ctx.xplane.run_command(cmd).await;
        }
        let mut rpm = None;
        let mut at_speed = false;
        for _ in 0..ROTOR_WAIT_SECS {
            tokio::time::sleep(Duration::from_secs(1)).await;
            rpm = elem(ctx, &p.rotor_rpm, 0).await;
            if rpm.is_some_and(|r| (r / p.rotor_rpm_governed - 1.0).abs() <= ROTOR_TOLERANCE) {
                at_speed = true;
                break;
            }
        }
        crate::blackbox::record(
            "heli_start",
            json!({ "event": "rotor", "rpm": rpm, "governed": p.rotor_rpm_governed, "at_speed": at_speed }),
        );
        // An unreadable rotor is no verdict either way.
        let verdict = rpm.map(|_| at_speed);
        // Every throttle READ BACK at FLY, one more write for a lever that is
        // not: engine 2's sat at 0.61 after "throttles to fly", its torque
        // under half the other's, and a heavy lift drooped the rotor to 83 %
        // at full collective (EDDV, 2026-09-23).
        let mut short = Vec::new();
        for i in 0..self.engines {
            let at = |v: Option<f64>| v.is_some_and(|t| (t - p.fly).abs() <= 0.02);
            if !at(elem(ctx, &p.throttle, i).await) {
                let _ = ctx.xplane.set_float_array_elem(&p.throttle, i, p.fly).await;
                tokio::time::sleep(Duration::from_millis(500)).await;
                let now = elem(ctx, &p.throttle, i).await;
                crate::blackbox::record(
                    "heli_start",
                    json!({ "event": "throttle_rewritten", "engine": i, "now": now }),
                );
                if !at(now) {
                    short.push((i + 1, now));
                }
            }
        }
        let mut line = start_line(&outcomes, verdict);
        for (n, now) in short {
            line.push_str(&format!(
                " Engine {n}'s throttle reads {}, not fly: check the lever.",
                now.map_or("nothing".to_string(), |v| format!("{v:.2}"))
            ));
        }
        Ok(line)
    }
}

pub struct StopHelicopter {
    p: HeliStartProfile,
    engines: usize,
}

impl StopHelicopter {
    pub fn new(p: HeliStartProfile, engines: usize) -> Self {
        Self {
            engines: engines.max(1).min(p.starters.len().max(1)),
            p,
        }
    }
}

#[async_trait]
impl Tool for StopHelicopter {
    fn name(&self) -> &str {
        "stop_engines"
    }
    fn description(&self) -> &str {
        "Shut the helicopter's engines down on the ground: throttles to off. \
         Use for 'shut down', 'engines off', 'stop the engines'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        if ctx.xplane.get_float(ON_GROUND_ANY).await? < 0.5 {
            return Ok(
                "Negative, we're airborne. I won't shut an engine down in flight.".to_string(),
            );
        }
        match elem(ctx, &self.p.collective, 0).await {
            Some(c) if c <= COLLECTIVE_DOWN_MAX => {}
            _ => {
                return Ok(
                    "The collective isn't fully down. Lower it before we shut down.".to_string(),
                )
            }
        }
        for i in 0..self.engines {
            ctx.xplane
                .set_float_array_elem(&self.p.throttle, i, 0.0)
                .await?;
        }
        // READBACK: the engines are off when they stop running, not when the
        // write returned.
        for _ in 0..10 {
            tokio::time::sleep(Duration::from_secs(1)).await;
            let mut any = false;
            for i in 0..self.engines {
                any |= running(ctx, i).await;
            }
            if !any {
                return Ok("Throttles off, both engines shut down.".to_string());
            }
        }
        Ok(
            "I've pulled the throttles to off but an engine is still running. \
            It's yours on the throttles."
                .to_string(),
        )
    }
}

/// Lift to a hover, hold one, or set down: the autopilot's hover mode holds
/// the position and [`crate::hover`] holds the height on the collective.
pub struct Hover {
    collective: String,
    /// The whole flight's other ends: the engine start it runs from cold,
    /// and the taxi-in's settings for the parking at the destination.
    start: StartHelicopter,
    taxi: crate::heli_route::TaxiIn,
}

impl Hover {
    pub fn new(
        p: &HeliStartProfile,
        start: StartHelicopter,
        taxi: crate::heli_route::TaxiIn,
    ) -> Self {
        crate::hover::publish_rotor(Some((
            p.rotor_rpm.clone(),
            p.rotor_rpm_governed,
            p.collective.clone(),
        )));
        Self {
            collective: p.collective.clone(),
            start,
            taxi,
        }
    }
}

#[async_trait]
impl Tool for Hover {
    fn name(&self) -> &str {
        "hover"
    }
    fn description(&self) -> &str {
        "Helicopter hover: 'up' lifts to a hover (default ten feet) or changes the \
         hover height; 'down' sets the helicopter down where it is. The autopilot \
         holds the position, the copilot works the collective. Use for 'lift to a \
         hover', 'hover at fifteen feet', 'set it down'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "action": { "type": "string", "enum": ["up", "down", "land", "pad", "move", "hold", "depart", "circuit", "speed", "route"] },
                "right": { "type": "boolean" },
                "resume": { "type": "boolean" },
                "height_ft": { "type": "number" },
                "direction": { "type": "string", "enum": ["forward", "back", "left", "right"] },
                "speed_kts": { "type": "number" }
            },
            "required": ["action"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let action = args.get("action").and_then(Value::as_str);
        if matches!(action, Some("move") | Some("hold")) {
            return air_taxi(args, action == Some("hold"), &self.collective, ctx).await;
        }
        if action == Some("depart") {
            return depart(args, ctx).await;
        }
        if action == Some("speed") {
            let kts = args.get("speed_kts").and_then(Value::as_f64).unwrap_or(0.0);
            return Ok(crate::hover::depart_speed(kts).unwrap_or_else(|| {
                "I change the speed only in a departure I'm flying.".to_string()
            }));
        }
        if action == Some("route") {
            return route(self, args, ctx).await;
        }
        if action == Some("circuit") {
            return circuit(args, ctx).await;
        }
        // "Land at the airport": the destination's pad. "Land the aircraft":
        // the pad when one is in reach, else here. Never cancels a route
        // that is already taking us to the pad (LIME 2026-09-25).
        if matches!(action, Some("pad") | Some("land")) {
            if crate::heli_route::ACTIVE.load(std::sync::atomic::Ordering::SeqCst) {
                return Ok(
                    "We're on the way to the pad. Say land here to set it down now.".to_string(),
                );
            }
            match arrival(self, ctx).await? {
                Ok(line) => return Ok(line),
                Err(why) if action == Some("pad") => return Ok(why),
                Err(_) => {}
            }
        }
        let down = matches!(action, Some("down") | Some("land"));
        if down {
            if !crate::hover::ACTIVE.load(std::sync::atomic::Ordering::SeqCst)
                && ctx.xplane.get_float(ON_GROUND_ANY).await? > 0.5
            {
                return Ok("We're already on the ground.".to_string());
            }
            if !crate::hover::ACTIVE.load(std::sync::atomic::Ordering::SeqCst) {
                if let Some(no) = takeover_refusal(ctx).await {
                    return Ok(no);
                }
            }
            crate::hover::request(
                ctx.config.xplane_base_url.clone(),
                ctx.voice.cloned(),
                self.collective.clone(),
                crate::hover::Goal::Land,
            );
            return Ok("Setting it down.".to_string());
        }
        let h = args
            .get("height_ft")
            .and_then(Value::as_f64)
            .unwrap_or(10.0);
        if !(3.0..=crate::hover::MAX_HOVER_FT).contains(&h) {
            return Ok(format!(
                "I'll hover between 3 and {:.0} feet. Say a height in that range.",
                crate::hover::MAX_HOVER_FT
            ));
        }
        let running = crate::hover::ACTIVE.load(std::sync::atomic::Ordering::SeqCst);
        let airborne = !running && ctx.xplane.get_float(ON_GROUND_ANY).await? < 0.5;
        if !running {
            if airborne {
                if let Some(no) = takeover_refusal(ctx).await {
                    return Ok(no);
                }
            } else if !running_all(ctx).await {
                return Ok("Both engines need to be running at fly first.".to_string());
            }
        }
        crate::hover::request(
            ctx.config.xplane_base_url.clone(),
            ctx.voice.cloned(),
            self.collective.clone(),
            crate::hover::Goal::Hover { height_ft: h },
        );
        Ok(if running {
            format!("Hover at {h:.0} feet.")
        } else if airborne {
            format!("I have it. Hover at {h:.0} feet.")
        } else {
            format!("Lifting to a {h:.0} foot hover.")
        })
    }
}

/// The destination's pad, from the air near it: the route's final only
/// ([`crate::heli_route::Route::arrive_only`]). `Err` says why not, for the
/// pilot who asked for the pad by name.
async fn arrival(tool: &Hover, ctx: &ToolCtx<'_>) -> Result<std::result::Result<String, String>> {
    if ctx.xplane.get_float(ON_GROUND_ANY).await? > 0.5 {
        return Ok(Err("We're on the ground. Say fly the flight plan for the whole flight.".into()));
    }
    if !crate::hover::ACTIVE.load(std::sync::atomic::Ordering::SeqCst) {
        if let Some(no) = takeover_refusal(ctx).await {
            return Ok(Err(no));
        }
    }
    let Some(plan) = crate::simbrief::loaded_plan() else {
        return Ok(Err("No flight plan loaded, so I don't know which airport. Load the flight plan first.".into()));
    };
    let (lat, lon) = tokio::join!(
        ctx.xplane.get_float(crate::datarefs::LATITUDE),
        ctx.xplane.get_float(crate::datarefs::LONGITUDE)
    );
    let (Ok(lat), Ok(lon)) = (lat, lon) else {
        return Ok(Err("I can't read our position.".into()));
    };
    let dest = plan.destination.clone();
    let pads = crate::heli_route::helipads(&ctx.config.apt_data_path, &dest);
    let apt = crate::nav::find_airport(&ctx.config.apt_data_path, &dest).ok().flatten();
    let on_pad = !pads.is_empty();
    let spot = crate::heli_route::pick_pad(&pads, (lat, lon))
        .or_else(|| apt.as_ref().and_then(|a| crate::heli_route::runway_spot(a, (lat, lon))));
    let Some((plat, plon, name)) = spot else {
        return Ok(Err(format!("I can't find a place to land at {dest}.")));
    };
    let geo = crate::taxi_guide::Geo::new(lat);
    let d = crate::taxi_guide::dist((0.0, 0.0), geo.to_m((lat, lon), (plat, plon)));
    if d > crate::heli_route::ARRIVE_ONLY_MAX_M {
        return Ok(Err(format!(
            "The pad at {dest} is {:.0} miles away, too far to join the final from here.",
            d / 1852.0
        )));
    }
    crate::hover::set_motion(crate::hover::Motion::default());
    crate::heli_route::start(
        ctx.voice.cloned(),
        ctx.config.clone(),
        tool.collective.clone(),
        crate::heli_route::Route {
            points: vec![(format!("{dest} {name}"), plat, plon)],
            on_pad,
            fast_climb: false,
            destination: dest.clone(),
            dest_tower_khz: apt.as_ref().and_then(|a| a.staffed_tower().map(|f| f.khz)),
            arrive_only: true,
            resume: false,
            runway_spots: match apt.as_ref() {
                Some(a) if !on_pad => crate::heli_route::runway_spots(a),
                _ => Vec::new(),
            },
        },
        crate::heli_route::Ends {
            start: None,
            lift: false,
            taxi: tool.taxi.clone(),
        },
    );
    Ok(Ok(format!(
        "To the {} at {dest}, {name}, {:.0} metres.",
        if on_pad { "pad" } else { "runway" },
        d
    )))
}

/// Move the hover along the ground ("move forward at five knots") or stop it
/// over the spot ("hold position"). Only a hover the loop is holding, off the
/// ground; the direction is relative to the nose, turned into a TRUE track.
async fn air_taxi(args: &Value, hold: bool, collective: &str, ctx: &ToolCtx<'_>) -> Result<String> {
    use crate::hover::{self, Motion};
    let departing = hover::goal().is_some_and(|g| g.vector().is_some());
    let flying = hover::ACTIVE.load(std::sync::atomic::Ordering::SeqCst)
        && (departing || matches!(hover::goal(), Some(hover::Goal::Hover { .. })));
    let on_ground = ctx.xplane.get_float(ON_GROUND_ANY).await? > 0.5;
    if hold && !flying && !on_ground {
        // Flown by the pilot (or the autopilot's cruise modes): take it at
        // the velocity it has and stop into a hover at this height.
        if let Some(no) = takeover_refusal(ctx).await {
            return Ok(no);
        }
        let h = (ctx.xplane.get_float(Y_AGL).await? * 3.28084)
            .round()
            .max(10.0);
        hover::request(
            ctx.config.xplane_base_url.clone(),
            ctx.voice.cloned(),
            collective.to_string(),
            hover::Goal::Hover { height_ft: h },
        );
        return Ok(format!("I have it. Stopping to a hover at {h:.0} feet."));
    }
    if !flying || on_ground {
        return Ok("I move us only from a hover I'm holding. Ask for a hover first.".to_string());
    }
    if hold {
        hover::set_motion(Motion::default());
        if departing {
            // Out of a departure: slow to a hover at the height we are at.
            let h = (ctx.xplane.get_float(Y_AGL).await? * 3.28084)
                .round()
                .max(10.0);
            hover::retarget(hover::Goal::Hover { height_ft: h });
            return Ok(format!("Slowing to a hover at {h:.0} feet."));
        }
        return Ok("Holding position.".to_string());
    }
    if departing {
        return Ok("We're departing. Say hold position to stop first.".to_string());
    }
    let kts = args.get("speed_kts").and_then(Value::as_f64).unwrap_or(5.0);
    let mps = kts / 1.94384;
    if !(0.5..=hover::MAX_AIR_TAXI_MPS + 1e-6).contains(&mps) {
        return Ok(format!(
            "I'll air taxi up to {:.0} knots.",
            hover::MAX_AIR_TAXI_MPS * 1.94384
        ));
    }
    let (dir, off) = match args.get("direction").and_then(Value::as_str) {
        Some("back") => ("back", 180.0),
        Some("left") => ("left", 270.0),
        Some("right") => ("right", 90.0),
        _ => ("forward", 0.0),
    };
    let heading = ctx.xplane.get_float(PSI).await?;
    hover::set_motion(Motion {
        speed_mps: mps,
        track_deg: (heading + off).rem_euclid(360.0),
    });
    Ok(format!("Moving {dir} at {kts:.0} knots."))
}

/// A plan whose first point is further than this from the aircraft is
/// another field's.
const PLAN_ORIGIN_MAX_M: f64 = 5_000.0;

/// The SimBrief plan from the hover the loop is holding, to a helipad at the
/// destination ([`crate::heli_route`]).
async fn route(tool: &Hover, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
    use crate::hover;
    let collective = tool.collective.as_str();
    let resume = args.get("resume").and_then(Value::as_bool) == Some(true);
    // The whole flight, like full auto for the aeroplanes: from the ground
    // (engines started first when they are not running) or from a hover the
    // loop holds; never from a flight someone else is flying.
    let hovering = hover::ACTIVE.load(std::sync::atomic::Ordering::SeqCst)
        && matches!(hover::goal(), Some(hover::Goal::Hover { .. }));
    let on_ground = ctx.xplane.get_float(ON_GROUND_ANY).await? > 0.5;
    if !resume && !hovering && !on_ground {
        return Ok(
            "You're flying it. I fly the plan from the ground or a hover I'm holding.".to_string(),
        );
    }
    if hover::ACTIVE.load(std::sync::atomic::Ordering::SeqCst) && !hovering {
        return Ok("I'm already flying. Say hold position first.".to_string());
    }
    let from_cold = on_ground && !running_all(ctx).await;
    // owners-by-name: a hover it holds is the route's to take, the others refuse.
    if crate::heli_route::ACTIVE.load(std::sync::atomic::Ordering::SeqCst)
        || crate::heli_circuit::ACTIVE.load(std::sync::atomic::Ordering::SeqCst)
    {
        return Ok("We're already flying a route.".to_string());
    }
    // A local plan ("plan a flight to X") first, as full auto takes it; else
    // the pilot's latest SimBrief plan.
    let plan = match crate::simbrief::local_plan() {
        Some(p) => p,
        None if resume => match crate::simbrief::loaded_plan() {
            Some(p) => p,
            None => return Ok("No flight plan is loaded. Load it first, then say resume.".into()),
        },
        None => match crate::simbrief::fetch(&ctx.config.simbrief_id).await {
            Ok(p) => p,
            Err(e) => return Ok(format!("I couldn't get the flight plan: {e}.")),
        },
    };
    // The plan must start where we are: flying a plan out of another field
    // threads the wrong corridor entirely (full auto's rule).
    let (lat, lon) = tokio::join!(
        ctx.xplane.get_float(crate::datarefs::LATITUDE),
        ctx.xplane.get_float(crate::datarefs::LONGITUDE)
    );
    let position = lat.as_ref().ok().copied().zip(lon.as_ref().ok().copied());
    if let (Some((lat, lon)), Some(first)) = (position, plan.waypoints.first()) {
        let geo = crate::taxi_guide::Geo::new(lat);
        let d = crate::taxi_guide::dist((0.0, 0.0), geo.to_m((lat, lon), (first.lat, first.lon)));
        if !resume && d > PLAN_ORIGIN_MAX_M && plan.origin != plan.destination {
            return Ok(format!(
                "That plan departs from {}, {:.0} miles from here. Plan a flight from here first.",
                plan.origin,
                d / 1852.0
            ));
        }
    }
    let mut points: Vec<(String, f64, f64)> = plan
        .waypoints
        .iter()
        .map(|w| (w.ident.clone(), w.lat, w.lon))
        .collect();
    if points.is_empty() {
        return Ok("The flight plan has no waypoints.".to_string());
    }
    // The destination point becomes the helipad nearest the inbound fix;
    // a field without pads keeps its reference point.
    let dest = plan.destination.clone();
    let inbound = if points.len() >= 2 {
        let p = &points[points.len() - 2];
        (p.1, p.2)
    } else {
        let p = &points[0];
        (p.1, p.2)
    };
    let pads = crate::heli_route::helipads(&ctx.config.apt_data_path, &dest);
    let dest_apt = crate::nav::find_airport(&ctx.config.apt_data_path, &dest)
        .ok()
        .flatten();
    let dest_tower_khz = dest_apt
        .as_ref()
        .and_then(|a| a.staffed_tower().map(|f| f.khz));
    // No helipad: the runway end nearest the arrival, never the reference
    // point (LILB's is 120 m off its strip); a tower's runway moves it to
    // that end in flight (`runway_spots`, #192).
    let on_pad = !pads.is_empty();
    let spot = crate::heli_route::pick_pad(&pads, inbound)
        .or_else(|| dest_apt.as_ref().and_then(|a| crate::heli_route::runway_spot(a, inbound)));
    let runway_spots = match dest_apt.as_ref() {
        Some(a) if !on_pad => crate::heli_route::runway_spots(a),
        _ => Vec::new(),
    };
    let pad_name = match spot {
        Some((lat, lon, name)) => {
            if let Some(last) = points.last_mut() {
                *last = (format!("{dest} {name}"), lat, lon);
            }
            Some(name)
        }
        None => None,
    };
    let mut points = crate::heli_route::drop_terminal_fixes(points);
    if resume && !on_ground {
        let Some((lat, lon)) = position else {
            return Ok("I can't read our position. I haven't taken control.".into());
        };
        match crate::heli_route::resume_index(&points, lat, lon) {
            Ok(join) => { points.drain(..join); }
            Err(why) => return Ok(why),
        }
    }
    let names: Vec<String> = points
        .iter()
        .take(points.len() - 1)
        .map(|p| p.0.clone())
        .collect();
    // The climb out, against the ground it will meet: from the ground, the
    // hover loop climbs at 500 fpm along the track, and nothing in the air
    // looks ahead of it (LILO-LILB, 2026-09-24: into the ridge 4.3 nm out).
    let mut climb_note = "";
    // Without a probed climb-out, the slow departure: the steeper one.
    let mut fast_climb = false;
    if on_ground {
        let (here_lat, here_lon, elev) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::LATITUDE),
            ctx.xplane.get_float(crate::datarefs::LONGITUDE),
            ctx.xplane.get_float("sim/flightmodel/position/elevation"),
        );
        if let (Ok(la), Ok(lo), Ok(el)) = (here_lat, here_lon, elev) {
            let mut line = vec![(la, lo)];
            line.extend(points.iter().skip(1).map(|p| (p.1, p.2)));
            let stations = crate::heli_route::climb_stations(
                &line,
                crate::heli_route::CLIMB_CHECK_NM,
                400.0,
                500.0,
            );
            let xp = crate::xplane::XPlaneClient::new(ctx.config.xplane_base_url.clone());
            let mut ground: Vec<(f64, f64)> = Vec::new();
            let mut probed = true;
            for chunk in stations.chunks(crate::terrain::MAX_POINTS) {
                let pts: Vec<(f64, f64)> = chunk.iter().map(|s| (s.1, s.2)).collect();
                match crate::terrain::sample_ft(&xp, &pts).await {
                    Some(h) => ground.extend(chunk.iter().map(|s| s.0).zip(h)),
                    None => {
                        probed = false;
                        break;
                    }
                }
            }
            // The fast climb where the ground allows it, the slow valley
            // climb where only that clears, else no departure at all.
            let field = el * 3.280_84;
            let fast_conflict = crate::heli_route::climb_conflict(field, &ground, true);
            let conflict = crate::heli_route::climb_conflict(field, &ground, false);
            fast_climb = probed && fast_conflict.is_none();
            crate::blackbox::record(
                "heli_route",
                serde_json::json!({ "event": "climb_check", "probed": probed, "points": ground.len(), "fast": fast_climb,
                    "conflict": conflict.map(|c| serde_json::json!({"nm": c.0, "ground_ft": c.1, "climb_ft": c.2})),
                    "fast_conflict": fast_conflict.map(|c| serde_json::json!({"nm": c.0, "ground_ft": c.1, "climb_ft": c.2})) }),
            );
            if let Some((d, g, a)) = conflict {
                return Ok(format!(
                    "Negative. Climbing out at 500 feet a minute we'd be at about {} {:.0} miles out, where the ground rises to {}. Plan a route that climbs up a valley.",
                    crate::flightplan::spoken_feet((a / 100.0).round() * 100.0),
                    d,
                    crate::flightplan::spoken_feet(g)
                ));
            }
            if probed && !fast_climb {
                climb_note = " Slow climb out for the terrain.";
            }
            if !probed {
                climb_note = " I couldn't check all the ground on the climb out.";
            }
        }
    }
    hover::set_motion(hover::Motion::default());
    crate::heli_route::start(
        ctx.voice.cloned(),
        ctx.config.clone(),
        collective.to_string(),
        crate::heli_route::Route {
            points,
            on_pad,
            fast_climb,
            destination: dest.clone(),
            dest_tower_khz,
            arrive_only: false,
            resume: resume && !on_ground,
            runway_spots,
        },
        crate::heli_route::Ends {
            start: (on_ground && from_cold).then(|| tool.start.clone()),
            lift: on_ground,
            taxi: tool.taxi.clone(),
        },
    );
    // The numbers, never the fix list: twelve lat/lon idents read aloud
    // were a paragraph (LSZS-LILO, 2026-09-24). The cruise level is said by
    // the route once the ground has been probed, since only then is it known.
    Ok(format!(
        "{} the plan to {dest}{}, {} {}, {:.0} knots.{climb_note}",
        if resume && !on_ground { "Resuming" } else { "Flying" },
        pad_name
            .map(|n| if on_pad { format!(", pad {n}") } else { format!(", {n}") })
            .unwrap_or_default(),
        names.len(),
        if names.len() == 1 { "fix" } else { "fixes" },
        crate::heli_route::cruise_kts(),
    ))
}

/// A circuit from the hover the loop is holding: out along the nose, round,
/// and back to a hover over this spot, then down ([`crate::heli_circuit`]).
async fn circuit(args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
    use crate::hover;
    let hovering = hover::ACTIVE.load(std::sync::atomic::Ordering::SeqCst)
        && matches!(hover::goal(), Some(hover::Goal::Hover { .. }));
    if !hovering || ctx.xplane.get_float(ON_GROUND_ANY).await? > 0.5 {
        return Ok(
            "I fly the circuit from a hover I'm holding. Ask for a hover first.".to_string(),
        );
    }
    if crate::heli_circuit::ACTIVE.load(std::sync::atomic::Ordering::SeqCst) {
        return Ok("We're already in the circuit.".to_string());
    }
    let right = args.get("right").and_then(Value::as_bool).unwrap_or(false);
    hover::set_motion(hover::Motion::default());
    crate::heli_circuit::start(
        ctx.voice.cloned(),
        ctx.config.xplane_base_url.clone(),
        right,
    );
    Ok(format!(
        "{} circuit, {:.0} feet, {:.0} knots, back to this spot.",
        if right { "Right-hand" } else { "Left-hand" },
        crate::heli_circuit::PATTERN_FT,
        crate::heli_circuit::PATTERN_MPS * 1.94384
    ))
}

/// Out of a hover into forward flight along the nose: level acceleration,
/// then a climb to the asked height once past translational lift.
async fn depart(args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
    use crate::hover;
    let hovering = hover::ACTIVE.load(std::sync::atomic::Ordering::SeqCst)
        && matches!(hover::goal(), Some(hover::Goal::Hover { .. }));
    if !hovering || ctx.xplane.get_float(ON_GROUND_ANY).await? > 0.5 {
        return Ok("I depart from a hover I'm holding. Ask for a hover first.".to_string());
    }
    let h = args
        .get("height_ft")
        .and_then(Value::as_f64)
        .unwrap_or(300.0);
    if !(100.0..=hover::DEPART_MAX_FT).contains(&h) {
        return Ok(format!(
            "I'll depart to between 100 and {:.0} feet.",
            hover::DEPART_MAX_FT
        ));
    }
    let kts = args
        .get("speed_kts")
        .and_then(Value::as_f64)
        .unwrap_or(30.0);
    let mps = kts / 1.94384;
    if !(hover::DEPART_CLIMB_MPS..=hover::DEPART_MAX_MPS + 1e-6).contains(&mps) {
        return Ok(format!(
            "I'll depart at {:.0} to {:.0} knots.",
            hover::DEPART_CLIMB_MPS * 1.94384,
            hover::DEPART_MAX_MPS * 1.94384
        ));
    }
    let heading = ctx.xplane.get_float(PSI).await?;
    hover::set_motion(hover::Motion::default());
    hover::retarget(hover::Goal::Depart {
        height_ft: h,
        speed_mps: mps,
        track_deg: heading,
    });
    Ok(format!(
        "Departing straight ahead, {kts:.0} knots, climbing to {h:.0} feet."
    ))
}

/// Airborne with nothing of ours flying it, the hover loop takes over only
/// low and slow, with the engines at fly: the pilot handed a 15 ft hover
/// back asked "land the aircraft" and "hover at 3 feet", was told "you're
/// flying it" twice, and rolled it over (EDDV, 2026-09-23). `None` = take it.
async fn takeover_refusal(ctx: &ToolCtx<'_>) -> Option<String> {
    let (agl, gs) = tokio::join!(
        ctx.xplane.get_float(Y_AGL),
        ctx.xplane.get_float(GROUNDSPEED)
    );
    let (Ok(agl), Ok(gs)) = (agl, gs) else {
        return Some("I can't read our height. You have it.".to_string());
    };
    let h_ft = agl * 3.28084;
    if h_ft > crate::hover::DEPART_MAX_FT {
        return Some(format!(
            "We're {h_ft:.0} feet up. I take it only below {:.0} feet.",
            crate::hover::DEPART_MAX_FT
        ));
    }
    if gs > crate::hover::TAKEOVER_MAX_MPS {
        return Some(format!(
            "We're moving at {:.0} knots. Slow to a hover and I'll take it.",
            gs * 1.94384
        ));
    }
    if !running_all(ctx).await {
        return Some("Both engines need to be running at fly first.".to_string());
    }
    None
}

/// The stock tail wheel lock, read then toggled when off (the command is a
/// toggle), read back. True when locked or when the airframe declares none.
pub async fn lock_tail_wheel(ctx: &ToolCtx<'_>, p: &HeliStartProfile) -> bool {
    let Some(toggle) = &p.tail_wheel_lock else {
        return true;
    };
    let locked = |v: Result<f64>| v.is_ok_and(|r| r > 0.5);
    if locked(ctx.xplane.get_float(TAIL_WHEEL_LOCK).await) {
        return true;
    }
    let _ = ctx.xplane.run_command(toggle).await;
    tokio::time::sleep(Duration::from_millis(500)).await;
    let ok = locked(ctx.xplane.get_float(TAIL_WHEEL_LOCK).await);
    crate::blackbox::record(
        "heli_start",
        json!({ "event": "tail_wheel_lock", "locked": ok }),
    );
    ok
}
const TAIL_WHEEL_LOCK: &str = "sim/cockpit2/controls/tailwheel_lock_ratio";

/// Every declared engine running.
async fn running_all(ctx: &ToolCtx<'_>) -> bool {
    ctx.xplane
        .get_float_array(ENGN_RUNNING)
        .await
        .map(|v| v.iter().take(2).all(|&r| r > 0.5))
        .unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use super::{start_line, EngineStart as E};

    #[test]
    fn the_start_says_what_came_up() {
        assert_eq!(
            start_line(&[E::Lit, E::Lit], Some(true)),
            "Both engines running, throttles to fly, rotor at 100 percent."
        );
        assert!(start_line(&[E::AlreadyRunning, E::Lit], Some(false)).contains("governed"));
        assert!(start_line(&[E::Lit, E::NoLight], None).starts_with("Engine number 2 didn't light"));
        for line in [
            start_line(&[E::Lit, E::Lit], Some(true)),
            start_line(&[E::NoLight], None),
            start_line(&[E::Lit, E::Lit], None),
        ] {
            assert!(
                !line.contains('\u{2014}') && !line.contains('\u{2013}'),
                "{line}"
            );
        }
    }

    /// The S-76C declares the start measured on it, and the registry builds
    /// the tools from it; the fixed-wing laws stay withheld.
    #[test]
    fn the_s76_starts_from_its_profile() {
        let prof = crate::aircraft::load("sikorsky_s76c").expect("s76 profile");
        let h = prof.heli_start.clone().expect("heli_start");
        assert_eq!(h.starters.len(), 2);
        assert!(
            h.idle > 0.0 && h.idle < h.fly,
            "0 is OFF: {} {}",
            h.idle,
            h.fly
        );
        assert!(
            h.idle_n1 < 62.0,
            "the idle measured live was 62 %: {}",
            h.idle_n1
        );
        let reg = crate::tools::Registry::for_aircraft(&prof);
        assert!(reg.get("start_engines").is_some() && reg.get("stop_engines").is_some());
        assert!(reg.get("hover").is_some());
        assert!(
            reg.get("circuit").is_none(),
            "still withheld until a helicopter law flies it"
        );
    }
}
