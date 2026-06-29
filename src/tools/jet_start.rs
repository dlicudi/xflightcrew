//! Jet engine start — `start_engines`. A study-level aircraft like the MD-82 hides
//! its start logic in xlua scripts whose custom datarefs/commands don't respond to
//! external activation. But X-Plane's built-in operations DO work over the Web API
//! and drive that logic internally: `sim/operation/auto_board` brings up the APU
//! and electrical, and `sim/operation/auto_start` cranks the engines. So the start
//! is: ensure the APU is up (auto_board), then auto_start, watching N2/running.
//! Verified live on the MD-82. Generic — no airframe-specific datarefs.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::plugin_status::{AlertLevel, StatusSink};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

const AUTO_BOARD: &str = "sim/operation/auto_board"; // APU + electrical bring-up
const AUTO_START: &str = "sim/operation/auto_start"; // engine start
const APU_N1: &str = "sim/cockpit/engine/APU_N1";
const N2: &str = "sim/cockpit2/engine/indicators/N2_percent";
const ENGN_RUNNING: &str = "sim/flightmodel/engine/ENGN_running";

const POLL_SECS: u64 = 2;

pub struct StartEngines;

#[async_trait]
impl Tool for StartEngines {
    fn name(&self) -> &str {
        "start_engines"
    }
    fn description(&self) -> &str {
        "Start the jet engines: bring up the APU/electrical if needed, then run the \
         engine start. Use for 'start engines', 'start the engines', 'engines on', \
         'start up the engines', 'spool up the engines', 'crank the engines', 'fire \
         up the engines'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {} })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        if let Ok(r) = ctx.xplane.get_float_array(ENGN_RUNNING).await {
            if r.iter().take(2).all(|&v| v > 0.5) && r.len() >= 2 {
                return Ok("Engines are already running.".to_string());
            }
        }
        spawn(ctx.config.xplane_base_url.clone(), ctx.voice.cloned(), ctx.status.cloned());
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

/// Count of the first two engines that are running.
async fn running_count(xp: &XPlaneClient) -> usize {
    xp.get_float_array(ENGN_RUNNING)
        .await
        .map(|r| r.iter().take(2).filter(|&&v| v > 0.5).count())
        .unwrap_or(0)
}

async fn run_sequence(xp: &XPlaneClient, voice: &Option<Voice>, status: &Option<StatusSink>) {
    // 1. APU up for bleed/power? If not, prep the aircraft (starts the APU) and wait
    //    for it to spool, since auto_start needs bleed air.
    if xp.get_float(APU_N1).await.unwrap_or(0.0) < 90.0 {
        announce(voice, status, "Bringing up the A P U and electrical.");
        xp.run_command(AUTO_BOARD).await.ok();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(75);
        let mut ready = false;
        while std::time::Instant::now() < deadline {
            if xp.get_float(APU_N1).await.unwrap_or(0.0) >= 90.0 {
                ready = true;
                break;
            }
            tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
        }
        if !ready {
            announce(voice, status, "The A P U didn't come up. Holding the start.");
            return;
        }
    }

    // 2. Crank the engines. auto_start can take a moment per engine; re-issue once
    //    if the second hasn't caught after a while.
    announce(voice, status, "Cranking the engines.");
    xp.run_command(AUTO_START).await.ok();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(90);
    let mut renudged = false;
    loop {
        let running = running_count(xp).await;
        if running >= 2 {
            announce(voice, status, "Both engines running, start complete.");
            return;
        }
        if std::time::Instant::now() >= deadline {
            let line = match running {
                1 => "One engine running, the other didn't catch. Check it.",
                _ => "The engines didn't start. Check bleed air and fuel.",
            };
            announce(voice, status, line);
            return;
        }
        // Halfway through, give auto_start another go for the lagging engine.
        if !renudged && std::time::Instant::now() > deadline - std::time::Duration::from_secs(45) {
            xp.run_command(AUTO_START).await.ok();
            renudged = true;
        }
        tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
    }
}
