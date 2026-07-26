//! Procedural power-up — `power_up` ("all electrical systems on"). Runs the MD-82
//! electrical bring-up the way the crew (and the aircraft's own boarding sequence)
//! does it: one step at a time, each driving the real control, with a callout.
//!
//! The steps and controls come straight from the aircraft's xlua boarding script
//! (md80_auto_board_start.lua) and were verified live: the APU starts by writing
//! `APU_starter_switch = 2` directly (the `APU_start` command is gated on a fuel
//! pump the API can't spin); the bleed valve is the `APU_dwn` command; bus
//! cross-ties are toggle commands; array datarefs are set whole-array.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::plugin_status::StatusSink;
use crate::voice::{Priority, Voice};
use crate::xplane::XPlaneClient;

// State (read) ...
const BATTERY_ON: &str = "sim/cockpit/electrical/battery_on";
const BATTERY_LOCK: &str = "laminar/md82/safeguard[3]";
const APU_STARTER: &str = "sim/cockpit2/electrical/APU_starter_switch"; // 0 off, 1 on, 2 start
const APU_N1: &str = "sim/cockpit2/electrical/APU_N1_percent";
const APU_BLEED: &str = "laminar/md82/bleedair/APU_on"; // 0 off, 1 on, 2 cooler
const BLEED_NEEDLE: &str = "laminar/md82/bleedair/bleedair_needle";
const XTIE_L: &str = "laminar/md82/electrical/cross_tie_APU_L";
const XTIE_R: &str = "laminar/md82/electrical/cross_tie_APU_R";
const START_PUMP: &str = "sim/cockpit2/engine/actuators/fuel_pump_on"; // [0] = aux start pump
const TANK_PUMP: &str = "sim/cockpit2/fuel/fuel_tank_pump_on"; // [0]=ctr [1]=L [2]=R

// ... and the controls that drive them.
const CMD_BATTERY: &str = "sim/electrical/battery_1_on";
const CMD_BATTERY_LOCK: &str = "laminar/md82cmd/safeguard03";
const CMD_APU_BLEED_ON: &str = "laminar/md82cmd/bleedair/APU_dwn"; // "dwn" increments 0->1
const CMD_XTIE_L: &str = "laminar/md82cmd/electrical/cross_tie_APU_L"; // toggle
const CMD_XTIE_R: &str = "laminar/md82cmd/electrical/cross_tie_APU_R"; // toggle

const POLL_SECS: u64 = 2;
const STEP_PAUSE_MS: u64 = 400;

pub struct PowerUp;

#[async_trait]
impl Tool for PowerUp {
    fn name(&self) -> &str {
        "power_up"
    }
    fn description(&self) -> &str {
        "Bring the electrical system up step by step (battery, APU, buses, bleed, \
         fuel pumps), calling out each. Use for 'all electrical systems on', 'power \
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
    // Mirror OR action, never both: v.alert already posts the line to the panel
    // transcript, so a separate s.alert here double-wrote it.
    let _ = status;
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

/// Poll `dr` until `ok(value)` is true, or give up after `secs`. Returns whether it met it.
async fn wait_until(xp: &XPlaneClient, dr: &str, secs: u64, ok: impl Fn(f64) -> bool) -> bool {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs);
    while std::time::Instant::now() < deadline {
        if ok(xp.get_float(dr).await.unwrap_or(0.0)) {
            return true;
        }
        tokio::time::sleep(std::time::Duration::from_secs(POLL_SECS)).await;
    }
    false
}

/// The electrical bring-up, step by step. `pub(crate)` so the engine start can run
/// it first when the APU/bleed isn't up yet (mirrors the aircraft's autostart,
/// which runs autoboard first).
pub(crate) async fn run_prep(xp: &XPlaneClient, voice: &Option<Voice>, status: &Option<StatusSink>) {
    // 1. Battery on (and locked).
    if xp.get_float(BATTERY_ON).await.unwrap_or(0.0) < 0.5 {
        xp.run_command(CMD_BATTERY).await.ok();
    }
    if xp.get_float(BATTERY_LOCK).await.unwrap_or(0.0) < 0.5 {
        xp.run_command(CMD_BATTERY_LOCK).await.ok();
    }
    say(voice, status, "Battery on and locked.");
    pause().await;

    // 2. Start the APU: aux fuel pump on, then engage the starter (set the switch
    //    to start directly — the APU_start command is gated on a spinning pump).
    if xp.get_float(APU_N1).await.unwrap_or(0.0) < 95.0 {
        say(voice, status, "Starting the A P U.");
        xp.set_int_array_elem(START_PUMP, 0, 1).await.ok(); // aux electric fuel pump
        xp.set_int(APU_STARTER, 2).await.ok(); // engage starter
        if !wait_until(xp, APU_N1, 75, |v| v > 95.0).await {
            say(voice, status, "The A P U didn't come up. Holding the power-up.");
            return;
        }
    }
    xp.set_int(APU_STARTER, 1).await.ok(); // starter to run
    say(voice, status, "A P U running, power available.");
    pause().await;

    // 3. APU buses (toggle cross-ties only if not already tied), aux pump off.
    if xp.get_float(XTIE_L).await.unwrap_or(0.0) < 0.5 {
        xp.run_command(CMD_XTIE_L).await.ok();
    }
    if xp.get_float(XTIE_R).await.unwrap_or(0.0) < 0.5 {
        xp.run_command(CMD_XTIE_R).await.ok();
    }
    xp.set_int_array_elem(START_PUMP, 0, 0).await.ok(); // aux pump no longer needed
    say(voice, status, "A P U on the buses.");
    pause().await;

    // 4. APU bleed air on, then wait for pressure to build.
    if xp.get_float(APU_BLEED).await.unwrap_or(0.0) < 0.5 {
        xp.run_command(CMD_APU_BLEED_ON).await.ok();
    }
    say(voice, status, "Bleed air on.");
    wait_until(xp, BLEED_NEEDLE, 30, |v| v > 2.0).await;
    pause().await;

    // 5. Boost pumps on (centre, left, right tanks).
    for i in 0..3 {
        xp.set_int_array_elem(TANK_PUMP, i, 1).await.ok();
    }
    say(voice, status, "Fuel pumps on. Electrical systems up.");
}
