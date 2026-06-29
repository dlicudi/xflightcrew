//! Procedural power-up — `power_up` ("all electrical systems on"). Runs the
//! aircraft's electrical bring-up the way a crew does it: one step at a time, with
//! a callout for each, NOT a single shortcut command. The control semantics come
//! from the MD-82's own xlua scripts (e.g. APU bleed is the `APU_dwn` command from
//! off; bus cross-ties are toggles), so each step drives the real switch and the
//! readback confirms it moved.
//!
//! MD-82-tuned (laminar/md82 controls); gated by the profile's `auto_prep` flag.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::plugin_status::{AlertLevel, StatusSink};
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

// Readback datarefs (state) ...
const BATTERY_ON: &str = "sim/cockpit/electrical/battery_on";
const APU_N1: &str = "sim/cockpit2/electrical/APU_N1_percent";
const APU_BLEED: &str = "laminar/md82/bleedair/APU_on"; // 0 off, 1 on, 2 cooler
const XTIE_L: &str = "laminar/md82/electrical/cross_tie_APU_L";
const XTIE_R: &str = "laminar/md82/electrical/cross_tie_APU_R";
const GEN_ON: &str = "sim/cockpit/electrical/generator_on"; // array

// ... and the commands that drive them (from the xlua scripts).
const CMD_BATTERY: &str = "sim/electrical/battery_1_on";
const CMD_APU_START: &str = "sim/electrical/APU_start";
const CMD_APU_BLEED_INC: &str = "laminar/md82cmd/bleedair/APU_dwn"; // "dwn" INCREMENTS 0->1
const CMD_XTIE_L: &str = "laminar/md82cmd/electrical/cross_tie_APU_L"; // toggle
const CMD_XTIE_R: &str = "laminar/md82cmd/electrical/cross_tie_APU_R"; // toggle
const CMD_GEN1: &str = "sim/electrical/generator_1_on";
const CMD_GEN2: &str = "sim/electrical/generator_2_on";

const POLL_SECS: u64 = 2;
const STEP_PAUSE_MS: u64 = 400;

pub struct PowerUp;

#[async_trait]
impl Tool for PowerUp {
    fn name(&self) -> &str {
        "power_up"
    }
    fn description(&self) -> &str {
        "Bring the electrical system up step by step (battery, APU, bleed, buses, \
         generators), calling out each. Use for 'all electrical systems on', 'power \
         up', 'power up the aircraft', 'bring up the electrical', 'electrics on'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {} })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        spawn(ctx.config.xplane_base_url.clone(), ctx.voice.cloned(), ctx.status.cloned());
        Ok("Powering up. Running the electrical.".to_string())
    }
}

fn say(voice: &Option<Voice>, status: &Option<StatusSink>, line: &str) {
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
            run_prep(&xp, &voice, &status).await;
        });
    });
}

async fn pause() {
    tokio::time::sleep(std::time::Duration::from_millis(STEP_PAUSE_MS)).await;
}

async fn run_prep(xp: &XPlaneClient, voice: &Option<Voice>, status: &Option<StatusSink>) {
    // 1. Battery.
    if xp.get_float(BATTERY_ON).await.unwrap_or(0.0) < 0.5 {
        xp.run_command(CMD_BATTERY).await.ok();
    }
    say(voice, status, "Battery on.");
    pause().await;

    // 2. APU — start it and wait for it to spool to power.
    if xp.get_float(APU_N1).await.unwrap_or(0.0) < 90.0 {
        say(voice, status, "Starting the A P U.");
        xp.run_command(CMD_APU_START).await.ok();
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
            say(voice, status, "The A P U didn't come up. Holding the power-up.");
            return;
        }
    }
    say(voice, status, "A P U running, power available.");
    pause().await;

    // 3. APU bleed air on (APU_dwn increments the switch 0 -> 1; don't overshoot to 2).
    if xp.get_float(APU_BLEED).await.unwrap_or(0.0) < 0.5 {
        xp.run_command(CMD_APU_BLEED_INC).await.ok();
    }
    say(voice, status, "A P U bleed air on.");
    pause().await;

    // 4. APU bus cross-ties (toggle commands — only fire if not already tied).
    if xp.get_float(XTIE_L).await.unwrap_or(0.0) < 0.5 {
        xp.run_command(CMD_XTIE_L).await.ok();
    }
    if xp.get_float(XTIE_R).await.unwrap_or(0.0) < 0.5 {
        xp.run_command(CMD_XTIE_R).await.ok();
    }
    say(voice, status, "A P U buses tied.");
    pause().await;

    // 5. Generators on the bus.
    xp.run_command(CMD_GEN1).await.ok();
    xp.run_command(CMD_GEN2).await.ok();
    let gens = xp
        .get_float_array(GEN_ON)
        .await
        .map(|g| g.iter().take(2).filter(|&&v| v > 0.5).count())
        .unwrap_or(0);
    say(
        voice,
        status,
        if gens >= 2 { "Generators on the bus. Electrical complete." } else { "Generators selected. Electrical complete." },
    );
}
