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
            // A lever may be one element of an array (the 747's four fuel
            // control switches are `toggle_sw_pos[0..3]`): an element is
            // written by index, never by a subscripted name.
            let (base, idx) = super::lights::split_index(lever);
            match idx {
                Some(i) => ctx.xplane.set_float_array_elem(base, i, self.cutoff).await,
                None => ctx.xplane.set_float(base, self.cutoff).await,
            }
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

/// A crew's flow pace: one switch every second and a half. A pilot monitoring
/// works the overhead at about this rate, and the aircraft's own script
/// spaces its steps so a contactor can make before the next press. Twenty
/// presses in two seconds reads as a macro; the pace is what makes the flow
/// followable from the other seat.
const FLOW_STEP_MS: u64 = 1500;
/// How long a switch is given to reach the position it was sent to before
/// the step is recorded as NOT taken. Pushbuttons animate over a few hundred
/// milliseconds; a knob stepped twice takes two presses' worth.
const CONFIRM_SECS: f64 = 3.0;
const CONFIRM_POLL_MS: u64 = 250;

/// What one step of the flow did, read back from the switch it moved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepOutcome {
    /// Already in the wanted position: nothing fired, no pause.
    AlreadySet,
    /// Fired, and the position read back as wanted.
    Confirmed,
    /// Fired, and the position never reached the wanted value. Named in the
    /// closing line: a silent no-op is what cost the first A330 start.
    NotTaken,
    /// Fired with no position to read (a momentary button).
    Fired,
}

/// A step's switch position, which may be one element of an array
/// (`battery_on[0]`). Read as a scalar, an indexed name is a 400 from the Web
/// API: both A330 battery steps came back `before: null, after: null` and
/// were reported as not taken on a ramp where they had (EGPF, 2026-09-21).
/// `None` is no measurement.
async fn read_position(xp: &dyn crate::xplane::XPlane, pos: &str) -> Option<f64> {
    let v = match crate::tools::lights::split_index(pos) {
        (base, Some(i)) => xp.get_float_array(base).await.ok()?.get(i).copied()?,
        (name, None) => xp.get_float(name).await.ok()?,
    };
    v.is_finite().then_some(v)
}

/// A step's short name for the panel and the closing line: the last path
/// segment of its position (else its command), underscores as spaces, the
/// `_pos` / `_toggle` suffixes dropped. "left1_pump_toggle" -> "left1 pump".
pub fn step_short_name(step: &crate::aircraft::StartStep) -> String {
    let src = step.position.as_deref().unwrap_or(&step.command);
    let tail = src.rsplit('/').next().unwrap_or(src);
    let mut t = tail.to_string();
    for suffix in ["_pos", "_toggle", "_on_off", "_knob"] {
        if let Some(stripped) = t.strip_suffix(suffix) {
            t = stripped.to_string();
        }
    }
    t.replace('_', " ")
}

impl AirlinerStart {
    pub fn new(p: AirlinerStartProfile, engines: usize) -> Self {
        AirlinerStart {
            p,
            engines: engines.clamp(1, 4),
        }
    }

    /// The engines in the order the profile starts them (the A330's POM says
    /// number two first); `0, 1, ...` when the profile is silent or names an
    /// engine it does not have.
    pub fn start_order(&self) -> Vec<usize> {
        let declared = &self.p.start_order;
        if !declared.is_empty()
            && declared.iter().all(|&i| i < self.engines)
            && declared.len() == self.engines
        {
            return declared.clone();
        }
        (0..self.engines).collect()
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

    /// Fire an ordered list of switch presses at a crew's pace, reading each
    /// one's position first where it has one and AFTER, until it agrees.
    ///
    /// The pace is gated on the aircraft, not the clock: a step waits for its
    /// switch to read back, then a human beat before the next. A switch
    /// already set is passed over without a pause (eyes move faster than
    /// hands). Every step is blackboxed and shown on the panel transcript;
    /// only a step's `say` milestone is spoken, and only once it confirmed.
    /// Returns the steps that did NOT take, by short name, for the closing
    /// line.
    async fn run_steps(
        &self,
        xp: &dyn crate::xplane::XPlane,
        steps: &[crate::aircraft::StartStep],
        say: &mut (dyn FnMut(&str) + Send),
        panel: &mut (dyn FnMut(&str) + Send),
    ) -> Vec<String> {
        let mut not_taken = Vec::new();
        for step in steps {
            let short = step_short_name(step);
            // Read before fire. These are toggles, and a toggle fired at a
            // switch already on turns it OFF — which on the APU battery is a
            // start that never begins, with no error anywhere.
            let mut cmd = step.command.as_str();
            let before = match &step.position {
                Some(pos) => read_position(xp, pos).await,
                None => None,
            };
            if step.position.is_some() && before.is_some_and(|v| (v - step.want).abs() < 0.5) {
                crate::blackbox::record(
                    "airliner_start",
                    serde_json::json!({"event": "step", "command": step.command, "position": step.position,
                                       "before": before, "after": before,
                                       "outcome": format!("{:?}", StepOutcome::AlreadySet)}),
                );
                panel(&format!("{short}: already set"));
                if let Some(line) = &step.say {
                    say(line);
                }
                continue;
            }
            // Past the wanted position: step back down instead. Without
            // this an ADIRS knob left in ATT is driven further from NAV,
            // not toward it.
            if let (Some(v), Some(down)) = (before, step.down_command.as_deref()) {
                if v > step.want + 0.5 {
                    cmd = down;
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
            // Wait for the switch to agree — the aircraft's pace, not ours.
            let (outcome, after) = match &step.position {
                None => (StepOutcome::Fired, None),
                Some(pos) => {
                    let deadline =
                        tokio::time::Instant::now() + Duration::from_secs_f64(CONFIRM_SECS);
                    loop {
                        tokio::time::sleep(Duration::from_millis(CONFIRM_POLL_MS)).await;
                        let now = read_position(xp, pos).await;
                        if now.is_some_and(|v| (v - step.want).abs() < 0.5) {
                            break (StepOutcome::Confirmed, now);
                        }
                        if tokio::time::Instant::now() >= deadline {
                            break (StepOutcome::NotTaken, now);
                        }
                    }
                }
            };
            crate::blackbox::record(
                "airliner_start",
                serde_json::json!({"event": "step", "command": cmd, "position": step.position,
                                   "before": before, "after": after, "outcome": format!("{outcome:?}")}),
            );
            match outcome {
                StepOutcome::NotTaken => {
                    panel(&format!("{short}: did not take"));
                    not_taken.push(short);
                }
                _ => {
                    panel(&short);
                    if let Some(line) = &step.say {
                        say(line);
                    }
                }
            }
            tokio::time::sleep(Duration::from_millis(FLOW_STEP_MS)).await;
        }
        not_taken
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

    /// The whole start, from whatever state the aircraft is in to both
    /// engines at idle and the after-start flow done. `say` is the voice
    /// (milestones only), `panel` the transcript (every switch). Returns the
    /// closing line. Pure over the sim handle so a test can fly it against a
    /// fake with the clock paused.
    pub async fn run(
        &self,
        xp: &dyn crate::xplane::XPlane,
        say: &mut (dyn FnMut(&str) + Send),
        panel: &mut (dyn FnMut(&str) + Send),
    ) -> String {
        let mut not_taken: Vec<String> = Vec::new();

        // Seat belt signs before anything turns, whoever asked for the start.
        if crate::tools::signs::set(xp, true).await == Some(false) {
            not_taken.push("seat belt signs".into());
        }

        // --- Bleed air. Without it the starters have nothing to turn. ---
        let apu_n1 = xp
            .get_float(&self.p.apu_n1)
            .await
            .ok()
            .filter(|v| v.is_finite())
            .unwrap_or(0.0);
        if apu_n1 < self.p.apu_ready_n1 {
            say("Starting the A.P.U.");
            not_taken.extend(self.run_steps(xp, &self.p.apu_steps, say, panel).await);
            // It takes its own time, and a start attempted without it just
            // hangs the engine at a few percent with nothing to say why.
            let deadline = tokio::time::Instant::now() + Duration::from_secs(90);
            loop {
                tokio::time::sleep(Duration::from_secs(2)).await;
                let n1 = xp.get_float(&self.p.apu_n1).await.unwrap_or(0.0);
                if n1 >= self.p.apu_ready_n1 {
                    break;
                }
                if tokio::time::Instant::now() >= deadline {
                    return not_taken_suffix(
                        "The A.P.U. hasn't come up, so there's no start air. Your aircraft.",
                        &not_taken,
                    );
                }
            }
            say("A.P.U. running.");
        }
        // The pre-start flow: ADIRS, bleed, beacon, the FUEL PUMPS — the
        // engine needs something to burn before a master switch means
        // anything (the first version went straight from the APU to the
        // masters, 2026-09-20). Silent but for the profile's milestones.
        not_taken.extend(
            self.run_steps(xp, &self.p.pre_start_steps, say, panel)
                .await,
        );

        // --- ENG MODE to IGN/START, and leave it there for every engine. ---
        let _ = xp.set_float(&self.p.mode_selector, self.p.mode_start).await;
        let order = self.start_order();
        say(&format!(
            "Ignition start. Starting number {}.",
            spoken_engine(order[0])
        ));

        let mut started = 0usize;
        for (n, &idx) in order.iter().enumerate() {
            self.set_master(xp, idx, true).await;
            let deadline = tokio::time::Instant::now() + Duration::from_secs(120);
            loop {
                tokio::time::sleep(Duration::from_secs(2)).await;
                match self.spool(xp, idx).await {
                    Some(v) if v >= self.p.spool_idle => break,
                    _ if tokio::time::Instant::now() >= deadline => {
                        // Knob back to normal before handing back: a selector
                        // left in IGN/START is a continuous-ignition item the
                        // pilot has to find for themselves.
                        let _ = xp.set_float(&self.p.mode_selector, self.p.mode_norm).await;
                        return not_taken_suffix(
                            &format!(
                                "Number {} hasn't come up to idle. Selector back to normal. Your aircraft.",
                                spoken_engine(idx)
                            ),
                            &not_taken,
                        );
                    }
                    _ => {}
                }
            }
            started += 1;
            if let Some(&next) = order.get(n + 1) {
                say(&format!(
                    "Number {} at idle. Starting number {}.",
                    spoken_engine(idx),
                    spoken_engine(next)
                ));
            }
        }
        let _ = xp.set_float(&self.p.mode_selector, self.p.mode_norm).await;
        // The after-start flow — generators, bleeds, packs, APU off — the
        // aircraft's own sequence finishes the job rather than leaving the APU
        // carrying the bleed.
        not_taken.extend(
            self.run_steps(xp, &self.p.post_start_steps, say, panel)
                .await,
        );
        let engines = if started == 2 {
            "Both engines".to_string()
        } else {
            format!("{started} engines")
        };
        not_taken_suffix(
            &format!("{engines} running. After-start flow complete."),
            &not_taken,
        )
    }
}

fn spoken_engine(idx: usize) -> &'static str {
    ["one", "two", "three", "four"]
        .get(idx)
        .copied()
        .unwrap_or("one")
}

/// The closing line, with every switch that did not take named after it —
/// the readback compares against the INTENT, and a flow that says "complete"
/// over a pump that never came on is the lie this whole module exists to
/// stop telling.
fn not_taken_suffix(line: &str, not_taken: &[String]) -> String {
    if not_taken.is_empty() {
        return line.to_string();
    }
    let n = not_taken.len();
    format!(
        "{line} {} didn't take: {}.",
        if n == 1 {
            "One switch".to_string()
        } else {
            format!("{n} switches")
        },
        not_taken.join(", ")
    )
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
        let voice = ctx.voice;
        let status = ctx.status;
        // Mirror OR action, never both: `alert` already posts the spoken
        // line to the panel; the silent switches go to the panel only.
        let mut say = |s: &str| {
            if let Some(v) = voice {
                v.alert(Priority::Advisory, s.to_string());
            }
        };
        let mut panel = |s: &str| {
            if let Some(st) = status {
                st.action(s);
            }
        };
        Ok(self.run(xp, &mut say, &mut panel).await)
    }
}

#[cfg(test)]
mod airliner_start_tests {
    use super::*;
    use crate::aircraft::{AirlinerStartProfile, StartStep};
    use crate::xplane::XPlane;
    use std::collections::HashMap;
    use std::sync::Mutex;

    /// A sim whose switches MOVE when their commands fire — the fake the
    /// flow needs, because the flow's whole point is reading them back.
    struct Cockpit {
        reads: Mutex<HashMap<String, f64>>,
        /// command -> (position dataref, value it produces). A toggle flips.
        wiring: HashMap<String, (String, Option<f64>)>,
        commands: Mutex<Vec<String>>,
        writes: Mutex<Vec<(String, usize, f64)>>,
        /// Commands that do NOTHING to their switch: the silent no-op.
        dead: Vec<String>,
    }

    impl Cockpit {
        fn new() -> Self {
            Cockpit {
                reads: Mutex::new(HashMap::new()),
                wiring: HashMap::new(),
                commands: Mutex::new(Vec::new()),
                writes: Mutex::new(Vec::new()),
                dead: Vec::new(),
            }
        }
        fn set(&self, name: &str, v: f64) {
            self.reads.lock().unwrap().insert(name.to_string(), v);
        }
        fn toggle(&mut self, cmd: &str, pos: &str) {
            self.wiring.insert(cmd.to_string(), (pos.to_string(), None));
        }
        fn sets(&mut self, cmd: &str, pos: &str, v: f64) {
            self.wiring
                .insert(cmd.to_string(), (pos.to_string(), Some(v)));
        }
        fn ran(&self) -> Vec<String> {
            self.commands.lock().unwrap().clone()
        }
    }

    #[async_trait]
    impl XPlane for Cockpit {
        async fn get_float(&self, name: &str) -> Result<f64> {
            self.reads
                .lock()
                .unwrap()
                .get(name)
                .copied()
                .ok_or_else(|| anyhow::anyhow!("no read for {name}"))
        }
        async fn set_float(&self, name: &str, value: f64) -> Result<()> {
            self.set(name, value);
            Ok(())
        }
        async fn set_int(&self, name: &str, value: i64) -> Result<()> {
            self.set(name, value as f64);
            Ok(())
        }
        async fn run_command(&self, name: &str) -> Result<()> {
            self.commands.lock().unwrap().push(name.to_string());
            if self.dead.iter().any(|d| d == name) {
                return Ok(());
            }
            if let Some((pos, v)) = self.wiring.get(name) {
                let cur = self.reads.lock().unwrap().get(pos).copied().unwrap_or(0.0);
                let next = v.unwrap_or(if cur > 0.5 { 0.0 } else { 1.0 });
                self.set(pos, next);
            }
            Ok(())
        }
        async fn run_command_for(&self, name: &str, _d: f64) -> Result<()> {
            self.run_command(name).await
        }
        async fn get_float_array(&self, name: &str) -> Result<Vec<f64>> {
            let r = self.reads.lock().unwrap();
            Ok((0..4)
                .map(|i| r.get(&format!("{name}[{i}]")).copied().unwrap_or(0.0))
                .collect())
        }
        async fn set_int_array_elem(&self, name: &str, index: usize, value: i64) -> Result<()> {
            self.set_float_array_elem(name, index, value as f64).await
        }
        async fn set_float_array_elem(&self, name: &str, index: usize, value: f64) -> Result<()> {
            self.writes
                .lock()
                .unwrap()
                .push((name.to_string(), index, value));
            self.set(&format!("{name}[{index}]"), value);
            // The engine lights off once its master is on: the spool gauge
            // reads idle from the next poll.
            if name == "masters" && value > 0.5 {
                self.set(&format!("n2[{index}]"), 60.0);
            }
            Ok(())
        }
    }

    fn step(cmd: &str, pos: &str, want: f64, say: Option<&str>) -> StartStep {
        StartStep {
            command: cmd.into(),
            position: Some(pos.into()),
            want,
            held_secs: None,
            down_command: None,
            say: say.map(str::to_string),
        }
    }

    fn profile() -> AirlinerStartProfile {
        AirlinerStartProfile {
            mode_selector: "mode".into(),
            mode_start: 1.0,
            mode_norm: 0.0,
            masters: "masters".into(),
            master_on: 1.0,
            master_off: 0.0,
            master_guards: vec![],
            guard_lift: 0.002,
            guard_secs: 0.0,
            start_order: vec![1, 0],
            apu_steps: vec![
                step("bat1_tog", "bat1", 1.0, None),
                step("apu_master_tog", "apu_master", 1.0, None),
            ],
            apu_n1: "apu_n1".into(),
            pre_start_steps: vec![
                step("ir1_tog", "ir1", 1.0, None),
                step("ir2_tog", "ir2", 1.0, Some("A.D.I.R.S. on, aligning.")),
                step("pumpL_tog", "pumpL", 1.0, None),
                step("pumpR_tog", "pumpR", 1.0, Some("Fuel pumps on.")),
            ],
            post_start_steps: vec![step("pack1_tog", "pack1", 1.0, Some("Packs on."))],
            apu_ready_n1: 95.0,
            spool: "n2".into(),
            spool_idle: 50.0,
        }
    }

    fn cold_cockpit() -> Cockpit {
        let mut c = Cockpit::new();
        for (cmd, pos) in [
            ("bat1_tog", "bat1"),
            ("apu_master_tog", "apu_master"),
            ("ir1_tog", "ir1"),
            ("ir2_tog", "ir2"),
            ("pumpL_tog", "pumpL"),
            ("pumpR_tog", "pumpR"),
            ("pack1_tog", "pack1"),
        ] {
            c.toggle(cmd, pos);
            c.set(pos, 0.0);
        }
        // The APU master coming on is what brings the APU up.
        c.sets("apu_master_tog", "apu_master", 1.0);
        c.set("apu_n1", 0.0);
        c.set("mode", 0.0);
        c
    }

    /// Drive the flow with the clock paused: every sleep advances instantly,
    /// so a thirty-second flow runs in milliseconds and the order and the
    /// readbacks are what is tested, not the wall clock.
    async fn fly(c: &Cockpit, p: AirlinerStartProfile) -> (String, Vec<String>, Vec<String>) {
        let tool = AirlinerStart::new(p, 2);
        let mut said = Vec::new();
        let mut panel = Vec::new();
        // The APU spools once its master is on (the master step sets it).
        let apu = async {
            loop {
                tokio::time::sleep(Duration::from_millis(500)).await;
                if c.get_float("apu_master").await.unwrap_or(0.0) > 0.5 {
                    c.set("apu_n1", 100.0);
                }
            }
        };
        let mut say = |s: &str| said.push(s.to_string());
        let mut show = |s: &str| panel.push(s.to_string());
        let line = tokio::select! {
            l = tool.run(c, &mut say, &mut show) => l,
            _ = apu => unreachable!(),
        };
        (line, said, panel)
    }

    #[tokio::test(start_paused = true)]
    async fn the_flow_speaks_milestones_and_logs_every_switch() {
        let c = cold_cockpit();
        let (line, said, panel) = fly(&c, profile()).await;
        assert_eq!(line, "Both engines running. After-start flow complete.");
        assert_eq!(
            said,
            vec![
                "Starting the A.P.U.",
                "A.P.U. running.",
                "A.D.I.R.S. on, aligning.",
                "Fuel pumps on.",
                "Ignition start. Starting number two.",
                "Number two at idle. Starting number one.",
                "Packs on.",
            ],
            "the voice says the state, not the switches"
        );
        // Every switch reached the panel transcript, by short name.
        for name in [
            "bat1",
            "apu master",
            "ir1",
            "ir2",
            "pumpL",
            "pumpR",
            "pack1",
        ] {
            assert!(
                panel.iter().any(|l| l == name),
                "panel lacks {name}: {panel:?}"
            );
        }
    }

    #[tokio::test(start_paused = true)]
    async fn engines_start_in_the_profiles_order() {
        let c = cold_cockpit();
        let _ = fly(&c, profile()).await;
        let masters: Vec<usize> = c
            .writes
            .lock()
            .unwrap()
            .iter()
            .filter(|(n, _, v)| n == "masters" && *v > 0.5)
            .map(|(_, i, _)| *i)
            .collect();
        assert_eq!(
            masters,
            vec![1, 0],
            "number two first, as the A330's POM says"
        );
    }

    /// EGPF, 2026-09-21: `battery_on[0]` read as a scalar is a 400, so both
    /// A330 battery steps logged `before: null, after: null` and "did not
    /// take" on a ramp where they had. An indexed position is one element.
    #[tokio::test]
    async fn an_indexed_position_reads_its_element() {
        let xp = crate::xplane::FakeXPlane::new();
        xp.set_read_array("sim/cockpit2/electrical/battery_on", &[1.0, 0.0]);
        let base = "sim/cockpit2/electrical/battery_on";
        assert_eq!(read_position(&xp, &format!("{base}[0]")).await, Some(1.0));
        assert_eq!(read_position(&xp, &format!("{base}[1]")).await, Some(0.0));
        assert_eq!(read_position(&xp, &format!("{base}[7]")).await, None);
        xp.set_read("laminar/A333/buttons/APU_batt_pos", 1.0);
        assert_eq!(read_position(&xp, "laminar/A333/buttons/APU_batt_pos").await, Some(1.0));
        assert_eq!(read_position(&xp, "not/there").await, None, "no measurement");
    }

    #[tokio::test(start_paused = true)]
    async fn a_switch_already_set_is_never_toggled_off() {
        let c = cold_cockpit();
        c.set("ir1", 1.0);
        c.set("pumpL", 1.0);
        let _ = fly(&c, profile()).await;
        let ran = c.ran();
        assert!(
            !ran.iter().any(|r| r == "ir1_tog"),
            "ir1 was already on: {ran:?}"
        );
        assert!(!ran.iter().any(|r| r == "pumpL_tog"));
        assert_eq!(c.get_float("ir1").await.unwrap(), 1.0);
        assert_eq!(c.get_float("pumpL").await.unwrap(), 1.0);
    }

    #[tokio::test(start_paused = true)]
    async fn a_switch_that_does_not_take_is_named_and_its_milestone_withheld() {
        let mut c = cold_cockpit();
        // The right pump's command reaches nothing — the study-level no-op.
        c.dead.push("pumpR_tog".into());
        let (line, said, panel) = fly(&c, profile()).await;
        assert_eq!(
            line,
            "Both engines running. After-start flow complete. One switch didn't take: pumpR."
        );
        assert!(
            !said.iter().any(|s| s == "Fuel pumps on."),
            "no milestone over a dead switch: {said:?}"
        );
        assert!(panel.iter().any(|l| l == "pumpR: did not take"));
    }

    #[tokio::test(start_paused = true)]
    async fn the_pace_is_a_crews_not_a_macros() {
        let c = cold_cockpit();
        let t0 = tokio::time::Instant::now();
        let _ = fly(&c, profile()).await;
        let secs = t0.elapsed().as_secs_f64();
        // Seven fired switches at a beat and a half each, plus confirm polls
        // and the engine spools' first 2 s polls: tens of seconds, not two.
        assert!(
            secs > 12.0,
            "flow took {secs:.1} s of sim time: that is a macro"
        );
        assert!(
            secs < 90.0,
            "flow took {secs:.1} s: something waited on a timeout"
        );
    }

    #[test]
    fn short_names_read_like_a_switch_label() {
        let s = step(
            "laminar/A333/switches/left1_pump_toggle",
            "laminar/A333/fuel/buttons/left1_pump_pos",
            1.0,
            None,
        );
        assert_eq!(step_short_name(&s), "left1 pump");
        let k = StartStep {
            command: "laminar/A333/buttons/APU_start".into(),
            position: None,
            want: 1.0,
            held_secs: Some(1.0),
            down_command: None,
            say: None,
        };
        assert_eq!(step_short_name(&k), "APU start");
        assert_eq!(
            not_taken_suffix("Done.", &["left1 pump".into(), "apu bleed".into()]),
            "Done. 2 switches didn't take: left1 pump, apu bleed."
        );
    }
}
