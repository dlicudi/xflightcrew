//! Jet engine start — `start_engines`. Runs the MD-82 start the way the crew (and
//! the aircraft's own autostart) does it: crank each engine on bleed air, watch
//! N2, introduce fuel at 20%, wait for idle. Steps and controls come from the
//! aircraft's xlua autostart script and were verified live: the starter is a HELD
//! command, the fuel handle is `mixture_ratio[idx]` (not ENGN_mixt), and the
//! engine bleed cross-feeds route start air. If the APU/electrical isn't up yet it
//! runs the power-up first (as the autostart does).

use std::time::Duration;

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

/// The shutdown readback's watch: a big fan takes several seconds to wind down
/// past "running", so the window has to outlast the spool, not the write.
const SHUTDOWN_POLLS: usize = 12;
const SHUTDOWN_POLL_MS: u64 = 700;

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

/// Shut the jet engines down — cut the fuel levers. Gated hard against doing it
/// in flight: an airborne "engines off" is a refusal, not an action.
///
/// The levers are the AIRFRAME's ([`crate::aircraft::AircraftProfile::fuel_cutoff_levers`]),
/// not the stock `mixture_ratio`, and the answer is a READBACK: the write is
/// made, then every engine is watched down before "engines shut down" is
/// spoken. It used to write the stock mixture and answer "Fuel levers cut" on
/// the strength of having sent the request, both writes `.ok()`ed. On the stock
/// 737 that write is erased by the aircraft's own lua on the next frame, so the
/// copilot said the levers were cut at a Dublin stand while both engines ran
/// on, and the pilot cut them by hand (2026-09-20). A silently failed write
/// must not answer "Roger".
pub struct StopEngines {
    /// Per-engine cutoff lever; empty = the stock mixture handle.
    levers: Vec<String>,
    /// What those levers read at cutoff.
    cutoff: f64,
    /// How many engines must be watched down.
    engines: usize,
}

impl StopEngines {
    pub fn new(profile: &crate::aircraft::AircraftProfile) -> Self {
        Self {
            levers: profile.fuel_cutoff_levers.clone(),
            cutoff: profile.fuel_cutoff_value,
            engines: profile.engines.max(1),
        }
    }

    /// Put every lever at cutoff. `Err` carries the line to speak: a write the
    /// API refused is not a cut lever.
    async fn cut(&self, ctx: &ToolCtx<'_>) -> Result<(), String> {
        let fail = |e| format!("The fuel lever write failed: {e}. The levers are yours.");
        if self.levers.is_empty() {
            for i in 0..self.engines {
                ctx.xplane
                    .set_float_array_elem(MIXTURE, i, self.cutoff)
                    .await
                    .map_err(fail)?;
            }
            return Ok(());
        }
        for lever in &self.levers {
            ctx.xplane
                .set_float(lever, self.cutoff)
                .await
                .map_err(fail)?;
        }
        Ok(())
    }
}

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
            if r.len() >= self.engines && r.iter().take(self.engines).all(|&v| v < 0.5) {
                return Ok("Engines are already shut down.".to_string());
            }
        }
        if let Err(line) = self.cut(ctx).await {
            return Ok(line);
        }
        // READBACK: the levers are cut when the engines are DOWN, not when the
        // write returned 200. Watch every one of them — element 0 can read
        // stopped with the other still turning.
        for _ in 0..SHUTDOWN_POLLS {
            tokio::time::sleep(Duration::from_millis(SHUTDOWN_POLL_MS)).await;
            let Ok(r) = ctx.xplane.get_float_array(ENGN_RUNNING).await else {
                continue;
            };
            // A SHORT array is no verdict on the engines it does not carry.
            if r.len() >= self.engines && r.iter().take(self.engines).all(|&v| v < 0.5) {
                return Ok("Fuel levers cut, engines shut down.".to_string());
            }
        }
        // The levers were written and the engines are still turning: on this
        // airframe that write is not the control. Say so and hand them back,
        // rather than claim a shutdown that did not happen.
        Ok("I cut the fuel levers but the engines are still running. \
            They're yours, cut them on the pedestal."
            .to_string())
    }
}

/// The stock `auto_start` command on an aircraft whose xlua REPLACES it with
/// its own full sequence (`AircraftProfile::scripted_auto_start`). The 737-800's
/// runs cold and dark to both engines at idle in about two minutes: battery,
/// standby power, fire tests, APU and its generator, packs, starter 2 then 1 on
/// N2, generators, probe heat, engine bleeds, APU off, continuous ignition
/// (probed on the parked aircraft, 2026-09-17). The crew fires it once and
/// watches the engines; the aircraft's own script owns every switch until
/// then, so nothing here writes one.
pub struct ScriptedAutoStart;

const CMD_AUTO_START: &str = "sim/operation/auto_start";
const AUTO_START_IN_PROGRESS: &str = "sim/flightmodel2/misc/auto_start_in_progress";

#[async_trait]
impl Tool for ScriptedAutoStart {
    fn name(&self) -> &str {
        "start_engines"
    }
    fn description(&self) -> &str {
        "Start the engines using this aircraft's own scripted start sequence. Use \
         for 'start engines', 'start the engines', 'engines on', 'start up the \
         engines', 'fire up the engines'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {} })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let running = ctx
            .xplane
            .get_float_array(ENGN_RUNNING)
            .await
            .map(|v| v.iter().take(2).all(|r| *r > 0.5))
            .unwrap_or(false);
        if running {
            return Ok("Engines are already running.".to_string());
        }
        if ctx
            .xplane
            .get_float(AUTO_START_IN_PROGRESS)
            .await
            .unwrap_or(0.0)
            > 0.5
        {
            return Ok("The start sequence is already running.".to_string());
        }
        ctx.xplane.run_command(CMD_AUTO_START).await?;
        Ok(
            "Running the aircraft's own start sequence. About two minutes to both engines."
                .to_string(),
        )
    }
}

// ---- The airliner start: ENG MODE + guarded master switches -----------------

use crate::aircraft::AirlinerStartProfile;

/// `start_engines` for an airframe that starts the way an airliner does.
///
/// The MD-82 sequence above is that aircraft's, and `turboprop_start` is
/// condition levers. An Airbus or a Boeing has an ENG MODE knob and two
/// guarded master switches and fits neither, so every airliner here but the
/// MD-82 simply could not start its own engines (found in the A330 on the
/// ramp, 2026-09-20). The steps and every value come from the aircraft's OWN
/// autostart script, which is the only honest source for a study-level model.
pub struct AirlinerStart {
    p: AirlinerStartProfile,
    engines: usize,
}

impl AirlinerStart {
    pub fn new(p: AirlinerStartProfile, engines: usize) -> Self {
        AirlinerStart {
            p,
            engines: engines.clamp(1, 4),
        }
    }

    /// Lift the guard, set the switch, drop the guard. A bare write with the
    /// guard down is the silent no-op a study-level add-on is full of.
    async fn set_master(&self, xp: &dyn crate::xplane::XPlane, idx: usize, on: bool) {
        let guard = self.p.master_guards.get(idx);
        if let Some(g) = guard {
            let _ = xp.set_float(g, self.p.guard_lift).await;
        }
        let want = if on {
            self.p.master_on
        } else {
            self.p.master_off
        };
        // `set_float_array_elem`, NOT a scalar write to "name[0]": that string
        // is not a dataref the API knows, so the write went nowhere and
        // returned nothing to say so. The APU came up, the bleed came on, all
        // eight pumps came on, and the masters sat at zero while the engine
        // was given two minutes to reach an idle it was never going to reach
        // (2026-09-20). A failed write is now recorded, because a silent one
        // is what cost the sortie.
        if let Err(e) = xp.set_float_array_elem(&self.p.masters, idx, want).await {
            crate::blackbox::record(
                "airliner_start",
                serde_json::json!({
                    "event": "master_write_failed", "idx": idx, "err": e.to_string(),
                }),
            );
        }
        if let Some(g) = guard {
            tokio::time::sleep(Duration::from_secs_f64(self.p.guard_secs)).await;
            let _ = xp.set_float(g, 0.0).await;
        }
    }

    /// Fire an ordered list of switch presses, reading each one's position
    /// first where it has one.
    async fn run_steps(
        &self,
        xp: &dyn crate::xplane::XPlane,
        steps: &[crate::aircraft::StartStep],
    ) {
        for step in steps {
            // Read before fire. These are toggles, and a toggle fired at a
            // switch already on turns it OFF — which on the APU battery is a
            // start that never begins, with no error anywhere.
            // Which way to turn, for a knob that has a way to turn.
            let mut cmd = step.command.as_str();
            if let Some(pos) = &step.position {
                let at = xp.get_float(pos).await.ok().filter(|v| v.is_finite());
                if at.is_some_and(|v| (v - step.want).abs() < 0.5) {
                    continue;
                }
                // Past the wanted position: step back down instead. Without
                // this an ADIRS knob left in ATT is driven further from NAV,
                // not toward it.
                if let (Some(v), Some(down)) = (at, step.down_command.as_deref()) {
                    if v > step.want + 0.5 {
                        cmd = down;
                    }
                }
            }
            match step.held_secs {
                Some(secs) => {
                    let _ = xp.run_command_for(cmd, secs).await;
                }
                None => {
                    let _ = xp.run_command(cmd).await;
                }
            }
            // The aircraft's own script spaces these half a second apart; a
            // contactor needs a moment to make before the next press.
            tokio::time::sleep(Duration::from_millis(600)).await;
        }
    }

    /// This engine's spool gauge, or `None` when the read fails — which is no
    /// measurement, never a zero.
    async fn spool(&self, xp: &dyn crate::xplane::XPlane, idx: usize) -> Option<f64> {
        xp.get_float_array(&self.p.spool)
            .await
            .ok()
            .and_then(|v| v.get(idx).copied())
            .filter(|v| v.is_finite())
    }
}

#[async_trait]
impl Tool for AirlinerStart {
    fn name(&self) -> &str {
        "start_engines"
    }
    fn description(&self) -> &str {
        "Start the engines: APU and bleed air up, engine mode selector to \
         ignition/start, each master switch on in turn, calling out each as it \
         reaches idle, then the selector back to normal. Use for 'start \
         engines', 'start the engines', 'engines on', 'start up the engines', \
         'fire up the engines'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {} })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let xp = ctx.xplane;
        if let Ok(r) = xp.get_float_array(ENGN_RUNNING).await {
            if r.len() >= self.engines && r.iter().take(self.engines).all(|&v| v > 0.5) {
                return Ok("Engines are already running.".to_string());
            }
        }
        let say = |v: Option<&Voice>, s: &str| {
            if let Some(v) = v {
                v.alert(Priority::Advisory, s.to_string());
            }
        };

        // --- Bleed air. Without it the starters have nothing to turn. ---
        let apu_n1 = xp
            .get_float(&self.p.apu_n1)
            .await
            .ok()
            .filter(|v| v.is_finite())
            .unwrap_or(0.0);
        if apu_n1 < self.p.apu_ready_n1 {
            say(ctx.voice, "Batteries on, starting the A.P.U.");
            self.run_steps(xp, &self.p.apu_steps).await;
            // It takes its own time, and a start attempted without it just
            // hangs the engine at a few percent with nothing to say why.
            let deadline = std::time::Instant::now() + Duration::from_secs(90);
            loop {
                tokio::time::sleep(Duration::from_secs(2)).await;
                let n1 = xp.get_float(&self.p.apu_n1).await.unwrap_or(0.0);
                if n1 >= self.p.apu_ready_n1 {
                    break;
                }
                if std::time::Instant::now() >= deadline {
                    return Ok(
                        "The A.P.U. hasn't come up, so there's no start air. Your aircraft."
                            .to_string(),
                    );
                }
            }
            say(ctx.voice, "A.P.U. running.");
        }
        // Bleed air, beacon, and the FUEL PUMPS — the engine needs something
        // to burn before a master switch means anything. The first version of
        // this went straight from the APU to the masters (2026-09-20).
        say(ctx.voice, "Bleed air and fuel pumps.");
        self.run_steps(xp, &self.p.pre_start_steps).await;

        // --- ENG MODE to IGN/START, and leave it there for both engines. ---
        let _ = xp.set_float(&self.p.mode_selector, self.p.mode_start).await;
        say(ctx.voice, "Ignition start. Starting number one.");

        let mut started = 0usize;
        for idx in 0..self.engines {
            self.set_master(xp, idx, true).await;
            let deadline = std::time::Instant::now() + Duration::from_secs(120);
            loop {
                tokio::time::sleep(Duration::from_secs(2)).await;
                match self.spool(xp, idx).await {
                    Some(n) if n >= self.p.spool_idle => break,
                    _ if std::time::Instant::now() >= deadline => {
                        // Knob back to normal before handing back: a selector
                        // left in IGN/START is a continuous-ignition item the
                        // pilot has to find for themselves.
                        let _ = xp.set_float(&self.p.mode_selector, self.p.mode_norm).await;
                        return Ok(format!(
                            "Number {} hasn't come up to idle. Selector back to normal. Your aircraft.",
                            idx + 1
                        ));
                    }
                    _ => {}
                }
            }
            started += 1;
            if idx + 1 < self.engines {
                say(
                    ctx.voice,
                    &format!("Number {} at idle. Starting number {}.", idx + 1, idx + 2),
                );
            }
        }
        let _ = xp.set_float(&self.p.mode_selector, self.p.mode_norm).await;
        // Packs on, APU bleed off, APU off — the aircraft's own sequence
        // finishes the job rather than leaving the APU carrying the bleed.
        self.run_steps(xp, &self.p.post_start_steps).await;
        Ok(format!("{started} engines running, packs on, A.P.U. off."))
    }
}
