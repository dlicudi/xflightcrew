//! Jet engine start — `start_engines`. Runs the MD-82 start the way the crew (and
//! the aircraft's own autostart) does it: crank each engine on bleed air, watch
//! N2, introduce fuel at 20%, wait for idle. Steps and controls come from the
//! aircraft's xlua autostart script and were verified live: the starter is a HELD
//! command, the fuel handle is `mixture_ratio[idx]` (not ENGN_mixt), and the
//! engine bleed cross-feeds route start air. If the APU/electrical isn't up yet it
//! runs the power-up first (as the autostart does).

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::plugin_status::StatusSink;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const APU_N1: &str = "sim/cockpit2/electrical/APU_N1_percent";
const N2: &str = "sim/cockpit2/engine/indicators/N2_percent";
const MIXTURE: &str = "sim/cockpit2/engine/actuators/mixture_ratio"; // the fuel handle
const GEN_ON: &str = "sim/cockpit2/electrical/generator_on";
const ENGN_RUNNING: &str = "sim/flightmodel/engine/ENGN_running";
const XFEED_L: &str = "laminar/md82/bleedair/engineL_xfeed_lever";
const XFEED_R: &str = "laminar/md82/bleedair/engineR_xfeed_lever";
// Post-start: transfer to engine generators and shut the APU down (start-only).
const APU_ON: &str = "laminar/md82/bleedair/APU_on";
const CMD_APU_BLEED_OFF: &str = "laminar/md82cmd/bleedair/APU_up"; // "up" decrements 1 -> 0
const CMD_APU_BLEED_ON: &str = "laminar/md82cmd/bleedair/APU_dwn"; // "dwn" increments 0 -> 1
                                                                   // Bleed-air manifold pressure needle — the pneumatic starters need this up to crank.
const BLEED_NEEDLE: &str = "laminar/md82/bleedair/bleedair_needle";
const XTIE_L: &str = "laminar/md82/electrical/cross_tie_APU_L";
const XTIE_R: &str = "laminar/md82/electrical/cross_tie_APU_R";
const CMD_XTIE_L: &str = "laminar/md82cmd/electrical/cross_tie_APU_L";
const CMD_XTIE_R: &str = "laminar/md82cmd/electrical/cross_tie_APU_R";
const APU_STARTER: &str = "sim/cockpit2/electrical/APU_starter_switch";

const N2_LIGHT: f64 = 20.0; // introduce fuel above this
const N2_IDLE: f64 = 50.0; // self-sustaining
const POLL_SECS: u64 = 2;
/// Hold the starter this long (X-Plane keeps it ~6s more), enough to reach 20% N2.
const STARTER_HOLD_SECS: f64 = 10.0;

pub struct StartEngines;

#[async_trait]
impl Tool for StartEngines {
    fn name(&self) -> &str {
        "start_engines"
    }
    fn description(&self) -> &str {
        "Start the jet engines: crank each on bleed air, introduce fuel, bring to \
         idle, calling out each. Use for 'start engines', 'start the engines', \
         'engines on', 'start up the engines', 'crank the engines', 'fire up the \
         engines'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {} })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        if let Ok(r) = ctx.xplane.get_float_array(ENGN_RUNNING).await {
            if r.len() >= 2 && r.iter().take(2).all(|&v| v > 0.5) {
                return Ok("Engines are already running.".to_string());
            }
        }
        spawn(
            ctx.config.xplane_base_url.clone(),
            ctx.voice.cloned(),
            ctx.status.cloned(),
        );
        Ok("Starting engines, stand by.".to_string())
    }
}

fn say(voice: &Option<Voice>, status: &Option<StatusSink>, line: &str) {
    // Mirror OR action, never both: v.alert already posts to the panel.
    let _ = status;
    if let Some(v) = voice {
        v.alert(Priority::Advisory, line.to_string());
    }
}

fn spawn(base: String, voice: Option<Voice>, status: Option<StatusSink>) {
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async move {
            let xp = XPlaneClient::new(base);
            run_sequence(&xp, &voice, &status).await;
        });
    });
}

async fn n2(xp: &XPlaneClient, idx: usize) -> f64 {
    xp.get_float_array(N2)
        .await
        .ok()
        .and_then(|v| v.get(idx).copied())
        .unwrap_or(0.0)
}

/// Poll engine `idx`'s N2 until it passes `pct`, or give up after `secs`.
async fn wait_n2(xp: &XPlaneClient, idx: usize, pct: f64, secs: u64) -> bool {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs);
    while std::time::Instant::now() < deadline {
        if n2(xp, idx).await > pct {
            return true;
        }
        tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
    }
    false
}

async fn run_sequence(xp: &XPlaneClient, voice: &Option<Voice>, status: &Option<StatusSink>) {
    // Electrical/APU/bleed must be up first (the autostart runs the boarding prep
    // if it isn't). Reuse the same procedural power-up.
    if xp.get_float(APU_N1).await.unwrap_or(0.0) < 95.0 {
        say(voice, status, "Electrical isn't up. Powering up first.");
        crate::tools::power::run_prep(xp, voice, status).await;
        if xp.get_float(APU_N1).await.unwrap_or(0.0) < 95.0 {
            say(voice, status, "No A P U bleed, can't start the engines.");
            return;
        }
    }

    // The pneumatic starters need APU bleed air on the manifold. The APU can be
    // spinning (N1 > 95) with its bleed valve CLOSED — e.g. a prior start's
    // post-shutdown closes it — so an N1 check alone isn't enough; cranking then
    // gets no air and the engine "won't spool". Make sure the bleed valve is open
    // and pressure is up before we crank.
    if xp.get_float(APU_ON).await.unwrap_or(0.0) < 0.5 {
        say(voice, status, "Opening A P U bleed.");
        xp.run_command(CMD_APU_BLEED_ON).await.ok();
    }
    let mut needle = 0.0;
    for _ in 0..10 {
        needle = xp.get_float(BLEED_NEEDLE).await.unwrap_or(0.0);
        if needle > 2.0 {
            break;
        }
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
    tracing::info!(
        apu_n1 = xp.get_float(APU_N1).await.unwrap_or(0.0),
        apu_bleed = xp.get_float(APU_ON).await.unwrap_or(0.0),
        bleed_needle = needle,
        "jet start: pre-crank bleed state"
    );
    if needle < 2.0 {
        say(
            voice,
            status,
            "No bleed pressure on the manifold, can't crank. Check the A P U bleed.",
        );
        return;
    }

    // Crank each engine in turn: right (idx 1, starter 2), then left (idx 0, starter 1).
    for (idx, starter, lever_open, lever_close, side) in [
        (
            1usize,
            "sim/starters/engage_starter_2",
            XFEED_R,
            XFEED_L,
            "right",
        ),
        (
            0usize,
            "sim/starters/engage_starter_1",
            XFEED_L,
            XFEED_R,
            "left",
        ),
    ] {
        // Generator on for this engine; route start air to it (open its cross-feed,
        // close the other).
        xp.set_int_array_elem(GEN_ON, idx, 1).await.ok();
        xp.set_float(lever_open, 1.0).await.ok();
        xp.set_float(lever_close, 0.0).await.ok();

        say(voice, status, &format!("Cranking the {side} engine."));
        xp.run_command_for(starter, STARTER_HOLD_SECS).await.ok();
        if !wait_n2(xp, idx, N2_LIGHT, 30).await {
            tracing::warn!(
                side,
                n2 = n2(xp, idx).await,
                bleed_needle = xp.get_float(BLEED_NEEDLE).await.unwrap_or(0.0),
                "jet start: engine failed to spool"
            );
            say(
                voice,
                status,
                &format!("The {side} engine won't spool, check bleed air. Start aborted."),
            );
            return;
        }
        // 20% N2 — introduce fuel (the fuel handle), then wait for idle.
        say(voice, status, &format!("{} fuel on.", cap(side)));
        xp.set_float_array_elem(MIXTURE, idx, 1.0).await.ok();
        if !wait_n2(xp, idx, N2_IDLE, 60).await {
            say(
                voice,
                status,
                &format!("The {side} engine hung below idle, watch it."),
            );
            return;
        }
        say(voice, status, &format!("The {side} engine is running."));
    }

    // Both cross-feeds open for normal ops.
    xp.set_float(XFEED_L, 1.0).await.ok();
    xp.set_float(XFEED_R, 1.0).await.ok();

    // Transfer to the engine generators (on from the crank) and shut the APU down
    // -- it's only needed to start. Bleed off, APU buses off, APU master off.
    say(
        voice,
        status,
        "On engine generators, shutting the A P U down.",
    );
    if xp.get_float(APU_ON).await.unwrap_or(0.0) > 0.5 {
        xp.run_command(CMD_APU_BLEED_OFF).await.ok();
    }
    if xp.get_float(XTIE_L).await.unwrap_or(0.0) > 0.5 {
        xp.run_command(CMD_XTIE_L).await.ok();
    }
    if xp.get_float(XTIE_R).await.unwrap_or(0.0) > 0.5 {
        xp.run_command(CMD_XTIE_R).await.ok();
    }
    xp.set_int(APU_STARTER, 0).await.ok();

    say(voice, status, "Both engines running, start complete.");
}

fn cap(s: &str) -> String {
    let mut c = s.chars();
    c.next()
        .map(|f| f.to_uppercase().collect::<String>() + c.as_str())
        .unwrap_or_default()
}

const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";

/// Shut the jet engines down — cut both fuel levers. Gated hard against doing it
/// in flight: an airborne "engines off" is a refusal, not an action. Without this
/// tool the LLM has nothing to map "engines off" to on a jet and improvises a
/// nonsensical multi-tool cascade, so the tool exists mainly to absorb the intent
/// and answer it safely.
pub struct StopEngines;

#[async_trait]
impl Tool for StopEngines {
    fn name(&self) -> &str {
        "stop_engines"
    }
    fn description(&self) -> &str {
        "Shut the jet engines down (cut the fuel levers). Use for 'shut down the \
         engines', 'engines off', 'shut the engines down', 'cut the engines', 'kill \
         the engines', 'secure the engines'. Refused in flight."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {} })
    }
    fn confirm_prompt(&self, _args: &Value) -> Option<String> {
        Some("shut both engines down".to_string())
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Safety gate: never shut the engines down airborne. Default to airborne if
        // the read fails, so an unknown state errs safe.
        if ctx.xplane.get_float(ON_GROUND).await.unwrap_or(0.0) <= 0.5 {
            return Ok(
                "Negative, we're airborne. I will not shut the engines down in flight.".to_string(),
            );
        }
        if let Ok(r) = ctx.xplane.get_float_array(ENGN_RUNNING).await {
            if r.len() >= 2 && r.iter().take(2).all(|&v| v < 0.5) {
                return Ok("Engines are already shut down.".to_string());
            }
        }
        // Cut both fuel levers (the start sequence raises these to 1 to run).
        ctx.xplane.set_float_array_elem(MIXTURE, 0, 0.0).await.ok();
        ctx.xplane.set_float_array_elem(MIXTURE, 1, 0.0).await.ok();
        Ok("Fuel levers cut, both engines shutting down.".to_string())
    }
}
