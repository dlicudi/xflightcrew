//! Jet engine start — `start_engines`. A turbine start is a timed sequence with
//! feedback (crank, watch N2, introduce fuel, wait for idle), not a single switch
//! like the piston `start_engine`. So this kicks off a background sequence and
//! returns immediately, announcing progress as each engine comes up.
//!
//! The control set is the X-Plane MD-82's (air-driven starter fed by APU/engine
//! bleed; ignition selector is a laminar/md82 dataref). Gated by the profile's
//! `turbine_start` flag; tuned for the MD-82 until a second jet needs it generalized.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::plugin_status::{AlertLevel, StatusSink};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const FUEL_PUMP: &str = "sim/cockpit2/fuel/fuel_tank_pump_on"; // int array, tanks 0..2
const IGNITION_SYS: &str = "laminar/md82/ignition_sys"; // 0 off, 1 A, 2 B
const IGN_UP: &str = "laminar/md82cmd/ignition_sys_up";
const IGN_DWN: &str = "laminar/md82cmd/ignition_sys_dwn";
const N2: &str = "sim/cockpit2/engine/indicators/N2_percent"; // float array, %
const ENGN_MIXT: &str = "sim/flightmodel/engine/ENGN_mixt"; // float array, 1 = fuel/run
const BLEED: &str = "sim/cockpit2/bleedair/indicators/bleed_available_center";
const APU_START: &str = "sim/electrical/APU_start";

/// N2 at which the starter has the engine turning fast enough to introduce fuel.
const N2_LIGHT_PCT: f64 = 20.0;
/// N2 at which the engine is self-sustaining at idle.
const N2_IDLE_PCT: f64 = 50.0;
const POLL_SECS: u64 = 1;

pub struct StartEngines;

#[async_trait]
impl Tool for StartEngines {
    fn name(&self) -> &str {
        "start_engines"
    }
    fn description(&self) -> &str {
        "Start the jet engines: run the turbine start sequence (bleed air, fuel \
         pumps, ignition, crank each engine to idle). Use for 'start engines', \
         'start the engines', 'engines on', 'start up the engines', 'spool up the \
         engines', 'crank the engines', 'fire up the engines'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {} })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let base = ctx.config.xplane_base_url.clone();
        // Already turning and burning?
        if let Ok(n2) = XPlaneClient::new(base.clone()).get_float_array(N2).await {
            if !n2.is_empty() && n2.iter().all(|&v| v > N2_IDLE_PCT) {
                return Ok("Engines are already running.".to_string());
            }
        }
        spawn(base, ctx.voice.cloned(), ctx.status.cloned());
        Ok("Starting engines, stand by.".to_string())
    }
}

fn announce(voice: &Option<Voice>, status: &Option<StatusSink>, line: &str) {
    if let Some(s) = status {
        s.alert(AlertLevel::Advisory, line);
    }
    if let Some(v) = voice {
        v.alert(Priority::Advisory, line.to_string());
    }
}

fn spawn(base: String, voice: Option<Voice>, status: Option<StatusSink>) {
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            return;
        };
        rt.block_on(async move {
            let xp = XPlaneClient::new(base);
            run_sequence(&xp, &voice, &status).await;
        });
    });
}

/// Wait until engine `idx` reaches `pct` N2, or give up after `secs`.
async fn wait_n2(xp: &XPlaneClient, idx: usize, pct: f64, secs: u64) -> bool {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs);
    while std::time::Instant::now() < deadline {
        if let Ok(v) = xp.get_float_array(N2).await {
            if v.get(idx).copied().unwrap_or(0.0) > pct {
                return true;
            }
        }
        tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
    }
    false
}

async fn run_sequence(xp: &XPlaneClient, voice: &Option<Voice>, status: &Option<StatusSink>) {
    // 1. Bleed air for the air-driven starter — start the APU if it isn't available.
    if xp.get_float(BLEED).await.unwrap_or(0.0) < 1.0 {
        announce(voice, status, "Starting the A P U for bleed air.");
        xp.run_command(APU_START).await.ok();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
        let mut ready = false;
        while std::time::Instant::now() < deadline {
            if xp.get_float(BLEED).await.unwrap_or(0.0) >= 1.0 {
                ready = true;
                break;
            }
            tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
        }
        if !ready {
            announce(voice, status, "No bleed air, the A P U didn't come online. Holding the start.");
            return;
        }
    }

    // 2. Fuel boost pumps on (center, left, right tanks).
    for i in 0..3 {
        xp.set_int_array_elem(FUEL_PUMP, i, 1).await.ok();
    }
    // 3. Ignition to system A.
    if xp.get_float(IGNITION_SYS).await.unwrap_or(0.0) < 0.5 {
        xp.run_command(IGN_UP).await.ok();
    }

    // 4. Crank each engine in turn (right, then left — one starter on the bleed at
    //    a time): spin up, introduce fuel at 20% N2, wait for idle.
    for (starter_n, idx, side) in [(2usize, 1usize, "right"), (1, 0, "left")] {
        announce(voice, status, &format!("Cranking the {side} engine."));
        xp.run_command(&format!("sim/starters/engage_starter_{starter_n}")).await.ok();
        if !wait_n2(xp, idx, N2_LIGHT_PCT, 45).await {
            announce(voice, status, &format!("The {side} engine isn't spooling up, check bleed air. Start aborted."));
            return;
        }
        // Fuel handle to run — light off.
        xp.set_float_array_elem(ENGN_MIXT, idx, 1.0).await.ok();
        if !wait_n2(xp, idx, N2_IDLE_PCT, 60).await {
            announce(voice, status, &format!("The {side} engine start is hung below idle, watch it."));
            return;
        }
        announce(voice, status, &format!("The {side} engine is running."));
    }

    // 5. Ignition back off.
    for _ in 0..4 {
        if xp.get_float(IGNITION_SYS).await.unwrap_or(0.0) > 0.5 {
            xp.run_command(IGN_DWN).await.ok();
            tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        }
    }
    announce(voice, status, "Both engines running, start complete.");
}
