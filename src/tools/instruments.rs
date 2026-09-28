//! Instrument settings: `set_barometer` (altimeter setting).

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

const BARO: &str = "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot";
/// The stock altimeter-knob commands, one hundredth of an inch per press. A
/// study-level xlua that owns the knob replaces THESE (the 737's handlers move
/// its own memory dataref), which is what makes them the path that works
/// when the dial write does not.
const CMD_BARO_UP: &str = "sim/instruments/barometer_up";
const CMD_BARO_DOWN: &str = "sim/instruments/barometer_down";
const BARO_STEP_INHG: f64 = 0.01;
/// One ROUND of presses: the whole 28.0..31.5 range. The dial is read again
/// after every round, and only a dial that is closing on the ask earns
/// another ([`keep_stepping`]).
const BARO_MAX_STEPS: usize = 350;
/// 0.0 to 31.5 is nine rounds. The 737 loaded with its baro memory at ZERO
/// (EGSS, 2026-09-22): one capped round per ask moved it 3.5 inHg at a time,
/// five asks left it at 17.5, the altimeter read 12,500 ft low, and the
/// follower "levelled at 4,600 ft" at 17,000.
const BARO_MAX_ROUNDS: usize = 10;
/// How long a frame takes to overwrite a write that will not stand, and how
/// long a round of presses takes to all land: the API runs a command in a
/// flight loop AFTER acknowledging it, and the 737's xlua copies its memory
/// into the dial a frame later again. At LIBP (2026-09-26) eighteen presses
/// were read 120 ms later as seventeen (30.06 to 30.23 for 30.24); the
/// eighteenth landed with the "correcting" press and the dial stopped at
/// 30.25, judged as not taken. A set happens a few times a flight: the wait
/// is cheap, the wrong verdict is a Warning and a wrong altimeter.
const BARO_SETTLE_MS: u64 = 300;
/// Pascals per inch of mercury (1013.25 hPa ≈ 29.92 inHg).
pub const PA_PER_INHG: f64 = 3386.389;
const DG_SYNC: &str = "sim/instruments/DG_sync_mag";
const DG_DRIFT: &str = "sim/cockpit/gyros/dg_drift_vac_deg";
// Low-vacuum annunciator: the DG is vacuum-driven, so a sync won't hold if the
// vacuum's failed — worth saying so rather than confirming a sync that'll drift.
const LOW_VACUUM: &str = "sim/cockpit/warnings/annunciators/low_vacuum";
const PITOT_HEAT: &str = "sim/cockpit2/ice/ice_pitot_heat_on_pilot";
// Command-driven so the panel switch animates (a raw write may not); PITOT_HEAT
// is read back to confirm.
const CMD_PITOT_HEAT_ON: &str = "sim/ice/pitot_heat0_on";
const CMD_PITOT_HEAT_OFF: &str = "sim/ice/pitot_heat0_off";

/// Which knob command, and how many presses, take the dial from `actual` to
/// `wanted`. Pure.
pub fn baro_steps(actual: f64, wanted: f64) -> (&'static str, usize) {
    let n = ((wanted - actual).abs() / BARO_STEP_INHG).round() as usize;
    let cmd = if wanted >= actual {
        CMD_BARO_UP
    } else {
        CMD_BARO_DOWN
    };
    (cmd, n.min(BARO_MAX_STEPS))
}

/// The dial reads what was asked, within a hundredth.
pub fn altimeter_took(actual: f64, wanted: f64) -> bool {
    (actual - wanted).abs() < BARO_STEP_INHG / 2.0
}

/// The first officer's altimeter and its knob (#94: the 737's copilot side
/// was still 0.0 after the captain's was walked to 30.42 at EGSS).
const BARO_COPILOT: &str = "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_copilot";
const CMD_BARO_COPILOT_UP: &str = "sim/instruments/barometer_copilot_up";
const CMD_BARO_COPILOT_DOWN: &str = "sim/instruments/barometer_copilot_down";

/// One side's altimeter: its dial, its knob and, where the airframe declares
/// one, a STD button that puts it at 29.92 from anywhere.
struct Side<'a> {
    name: &'static str,
    dial: &'static str,
    up: &'static str,
    down: &'static str,
    std: Option<(&'a str, usize)>,
}

impl<'a> Side<'a> {
    fn pilot(cfg: &'a crate::aircraft::AltimeterProfile) -> Self {
        Self {
            name: "pilot",
            dial: BARO,
            up: CMD_BARO_UP,
            down: CMD_BARO_DOWN,
            std: cfg.std_command.as_deref().map(|c| (c, cfg.std_presses.max(1))),
        }
    }
    fn copilot(cfg: &'a crate::aircraft::AltimeterProfile) -> Self {
        Self {
            name: "copilot",
            dial: BARO_COPILOT,
            up: CMD_BARO_COPILOT_UP,
            down: CMD_BARO_COPILOT_DOWN,
            std: cfg
                .copilot_std_command
                .as_deref()
                .map(|c| (c, cfg.std_presses.max(1))),
        }
    }
}

/// The airframe's altimeter fast paths, published per swap
/// (`flight_state::publish_airframe`), `None` the stock knob alone.
static ALTIMETER: std::sync::Mutex<Option<crate::aircraft::AltimeterProfile>> =
    std::sync::Mutex::new(None);

pub fn publish_altimeter(p: Option<crate::aircraft::AltimeterProfile>) {
    *ALTIMETER.lock().unwrap_or_else(|e| e.into_inner()) = p;
}

fn altimeter_profile() -> crate::aircraft::AltimeterProfile {
    ALTIMETER
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
        .unwrap_or_default()
}

/// The STD button earns its presses when going to 29.92 first leaves at
/// least ten knob presses fewer than stepping from where the dial is. Pure.
pub fn std_fast_path_pays(now: f64, wanted: f64) -> bool {
    (now - wanted).abs() - (STANDARD_INHG - wanted).abs() >= 10.0 * BARO_STEP_INHG
}

/// Set the altimeter and PROVE it. The stock dial is written and re-read a
/// frame later; when the write did not stand, the knob is stepped there with
/// the barometer commands and read again. Returns what the dial reads
/// afterwards, which the callers compare against what they asked.
///
/// The 737-800's xlua copies its own `laminar/B738/barometer/capt_setting`
/// into the stock dial every frame (`B738_baro_memory`, before_physics) and
/// that dataref is not writable over the API: a dial write was a silent
/// no-op and the immediate readback said "Roger, altimeter 30.11" to an ask
/// of 30.00 (probed on the parked aircraft at LFLL, 2026-09-17). Its
/// replaced `barometer_up/down` handlers move the memory a hundredth per
/// press, so the steps are the one path in. On a stock airframe the first
/// write stands and no command is sent.
///
/// A crew sets BOTH altimeters: the first officer's follows the captain's
/// through the same proving setter (best effort, blackboxed `altimeter`
/// `copilot`; a failed read there is no measurement and nothing is pressed).
pub async fn set_altimeter(xp: &dyn crate::xplane::XPlane, inhg: f64) -> Result<f64> {
    set_altimeter_with(xp, inhg, &altimeter_profile()).await
}

/// [`set_altimeter`] with the airframe's fast paths given, not published.
pub async fn set_altimeter_with(
    xp: &dyn crate::xplane::XPlane,
    inhg: f64,
    cfg: &crate::aircraft::AltimeterProfile,
) -> Result<f64> {
    let inhg = inhg.clamp(28.0, 31.5);
    let now = set_side(xp, &Side::pilot(cfg), inhg).await?;
    copilot_follow(xp, inhg, cfg).await;
    Ok(now)
}

/// The first officer's side to `inhg`, when it reads something else.
async fn copilot_follow(
    xp: &dyn crate::xplane::XPlane,
    inhg: f64,
    cfg: &crate::aircraft::AltimeterProfile,
) {
    let Ok(was) = xp.get_float(BARO_COPILOT).await else {
        return;
    };
    if altimeter_took(was, inhg) {
        return;
    }
    let now = set_side(xp, &Side::copilot(cfg), inhg).await.ok();
    crate::blackbox::record(
        "altimeter",
        serde_json::json!({
            "event": "copilot", "was": was, "now": now, "want": inhg,
            "took": now.is_some_and(|n| altimeter_took(n, inhg)),
        }),
    );
}

async fn set_side(xp: &dyn crate::xplane::XPlane, side: &Side<'_>, inhg: f64) -> Result<f64> {
    let settle = || tokio::time::sleep(std::time::Duration::from_millis(BARO_SETTLE_MS));
    xp.set_float(side.dial, inhg).await?;
    settle().await;
    let actual = xp.get_float(side.dial).await?;
    if altimeter_took(actual, inhg) {
        return Ok(actual);
    }
    let mut now = actual;
    // The STD button: 29.92 from anywhere in a press or two, then the knob
    // from there. Each press is its own frame (the 737 resets its memory in
    // before_physics only while STD SHOWS), and the dial says whether it
    // worked; a button that did not take leaves the knob to step from where
    // it is.
    if let Some((cmd, presses)) = side.std {
        if std_fast_path_pays(now, inhg) {
            for _ in 0..presses {
                xp.run_command(cmd).await?;
                settle().await;
            }
            let after = xp.get_float(side.dial).await?;
            crate::blackbox::record(
                "altimeter",
                serde_json::json!({
                    "event": "std_fast_path", "side": side.name, "cmd": cmd,
                    "presses": presses, "before": now, "after": after,
                    "took": altimeter_took(after, STANDARD_INHG),
                }),
            );
            now = after;
            if altimeter_took(now, inhg) {
                return Ok(now);
            }
        }
    }
    let mut presses = 0;
    for round in 1..=BARO_MAX_ROUNDS {
        let (up_down, n) = baro_steps(now, inhg);
        let cmd = if up_down == CMD_BARO_UP {
            side.up
        } else {
            side.down
        };
        for _ in 0..n {
            xp.run_command(cmd).await?;
        }
        presses += n;
        settle().await;
        let before = now;
        now = xp.get_float(side.dial).await?;
        tracing::info!(target: "xvoicelayer::instruments", side = side.name, asked = inhg, dial_write_read = actual, cmd, round, presses, now, "altimeter stepped");
        if !keep_stepping(before, now, inhg, n) {
            break;
        }
    }
    Ok(now)
}

/// Feet of indicated altitude per inch of mercury on the knob: the
/// altimeter's datum shifts about 27 to 30 ft per hectopascal, and an inch
/// is 33.86 of them.
pub const FT_PER_INHG: f64 = 960.0;

/// What the altimeter says against where the aircraft IS, and how much of
/// the difference the setting and the air's temperature explain. EGSS
/// (2026-09-22): 4,675 ft indicated at 16,884 true, a knob at 17.50 against a
/// QNH of 30.40, and the follower "levelled at 4,600 ft". The follower flies
/// the altimeter (LINTO 2026-09-14), so the cross-check is the one thing that
/// sees the altimeter itself is wrong. All in feet; positive = the aircraft
/// is HIGHER than the altimeter says.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltimeterTruth {
    pub indicated_ft: f64,
    pub true_ft: f64,
    /// `true_ft - indicated_ft`.
    pub error_ft: f64,
    /// The part the knob explains against the air's actual QNH: an
    /// altimeter reads true only on QNH (and in ISA air). On 29.92 in the
    /// flight levels this is the ordinary pressure-altitude difference.
    pub pressure_ft: f64,
    /// How far the knob is off the setting it SHOULD carry here (QNH below
    /// the transition, 29.92 above; `correct_setting_inhg`), in feet.
    pub misset_ft: f64,
    /// The part the air's departure from ISA explains: about 4 ft per 1,000
    /// per degree, so 10 % at cruise on a hot or cold day.
    pub temperature_ft: f64,
    /// What neither explains: a static system, a frozen port, a lying dial.
    pub residual_ft: f64,
    /// How large the residual may be before it is said.
    pub gate_ft: f64,
}

impl AltimeterTruth {
    /// The altimeter itself disagrees with the aircraft's height.
    pub fn instrument_suspect(&self) -> bool {
        self.residual_ft.abs() > self.gate_ft
    }
    /// A mis-set knob is costing more than [`ALTIMETER_GROSS_FT`].
    pub fn setting_gross(&self) -> bool {
        self.misset_ft.abs() > ALTIMETER_GROSS_FT.max(self.gate_ft)
    }
}

/// A setting error this large is no longer "altimeter check": it is a
/// different altitude (EGSS was 12,000 ft).
pub const ALTIMETER_GROSS_FT: f64 = 1_000.0;

/// ISA temperature at a pressure altitude, degrees C (tropopause floor).
fn isa_c(pressure_alt_ft: f64) -> f64 {
    (15.0 - 1.98 * pressure_alt_ft / 1000.0).max(-56.5)
}

/// The cross-check. `setting_inhg` is the knob, `qnh_inhg` the air's
/// sea-level pressure, `reference_inhg` the setting that SHOULD be on the
/// knob here (`correct_setting_inhg`). Pure.
pub fn altimeter_truth(
    indicated_ft: f64,
    true_ft: f64,
    setting_inhg: f64,
    qnh_inhg: f64,
    reference_inhg: f64,
    oat_c: f64,
) -> AltimeterTruth {
    let error_ft = true_ft - indicated_ft;
    // A knob BELOW the air's QNH reads low: the aircraft is higher.
    let pressure_ft = (qnh_inhg - setting_inhg) * FT_PER_INHG;
    let misset_ft = (reference_inhg - setting_inhg) * FT_PER_INHG;
    // Height above the sea-level pressure datum, where the column's
    // temperature acts.
    let above_datum = (indicated_ft + pressure_ft).max(0.0);
    let isa = isa_c(above_datum);
    let temperature_ft = (oat_c - isa) / (273.15 + isa) * above_datum;
    let residual_ft = error_ft - pressure_ft - temperature_ft;
    let gate_ft = 300.0 + 0.05 * above_datum + 0.05 * pressure_ft.abs();
    AltimeterTruth {
        indicated_ft,
        true_ft,
        error_ft,
        pressure_ft,
        misset_ft,
        temperature_ft,
        residual_ft,
        gate_ft,
    }
}

/// The monitor's last cross-check, for a follower that wants to know whether
/// the altimeter it flies can be believed: the seam issue #94 owes the
/// follower (it reads [`altimeter_truth_now`], never a copy of its own).
static ALTIMETER_TRUTH: std::sync::Mutex<Option<(std::time::Instant, AltimeterTruth)>> =
    std::sync::Mutex::new(None);

pub fn publish_altimeter_truth(t: Option<AltimeterTruth>) {
    *ALTIMETER_TRUTH.lock().unwrap_or_else(|e| e.into_inner()) =
        t.map(|t| (std::time::Instant::now(), t));
}

/// The cross-check if it is fresh (under five seconds old), else `None`: an
/// old verdict is no measurement. No consumer yet, on purpose: the follower
/// (another change's files) adopts it.
#[allow(dead_code)]
pub fn altimeter_truth_now() -> Option<AltimeterTruth> {
    ALTIMETER_TRUTH
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .filter(|(at, _)| at.elapsed() < std::time::Duration::from_secs(5))
        .map(|(_, t)| t)
}

/// Another round of presses is earned only by a dial that is not there yet
/// AND answered the last round: it closed on the ask by at least half of what
/// was pressed. A bad read, a dead knob or a box that owns its own number
/// shows no progress and gets no second round.
///
/// A dial ONE step past the ask that moved this round earns the one press
/// back: that is a press read a frame early, not a knob that missed (LIBP,
/// 2026-09-26: 30.23 read after eighteen presses, 30.25 after the nineteenth,
/// and the old rule saw an overshoot as no progress and left it there). A
/// dead knob does not move and still gets no second round; the rounds are
/// bounded either way. Pure.
pub fn keep_stepping(before: f64, now: f64, wanted: f64, pressed: usize) -> bool {
    if altimeter_took(now, wanted) || pressed == 0 {
        return false;
    }
    let closed = (before - wanted).abs() - (now - wanted).abs();
    if closed >= pressed as f64 * BARO_STEP_INHG / 2.0 {
        return true;
    }
    let one_step_past = (now - wanted).abs() < BARO_STEP_INHG * 1.5;
    let moved = (now - before).abs() >= BARO_STEP_INHG / 2.0;
    one_step_past && moved
}

pub struct SetBarometer;

#[derive(Deserialize)]
struct Args {
    /// Altimeter setting in inches of mercury (e.g. 29.92).
    inhg: f64,
}

#[async_trait]
impl Tool for SetBarometer {
    fn name(&self) -> &str {
        "set_barometer"
    }

    fn description(&self) -> &str {
        "Set the altimeter (barometric pressure) in inches of mercury. Use for \
         'set altimeter two niner niner two' (29.92), 'altimeter 30.15', \
         'set baro 29.80', 'set the Kollsman to 29.92', 'altimeter setting \
         30.02', 'set QNH three zero one zero'. (inHg only — convert a hPa/QNH \
         value before calling.)"
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "inhg": {
                    "type": "number",
                    "minimum": 28.0,
                    "maximum": 31.5,
                    "description": "Altimeter setting in inches of mercury, e.g. 29.92."
                }
            },
            "required": ["inhg"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        let inhg = args.inhg.clamp(28.0, 31.5);
        let actual = set_altimeter(ctx.xplane, inhg).await?;
        if altimeter_took(actual, inhg) {
            Ok(format!("Roger, altimeter {actual:.2}."))
        } else {
            Ok(format!(
                "Altimeter didn't take. It reads {actual:.2}, not {inhg:.2}."
            ))
        }
    }
}

/// Standard pressure (inHg) — what the altimeter is set to above the transition
/// altitude (1013.25 hPa).
pub const STANDARD_INHG: f64 = 29.92;

/// An altimeter error worth a word or a hand: 0.03 inHg is about 30 ft. The
/// sim's QNH drifts a hundredth at a time under real weather, so anything
/// tighter would have the copilot re-touching the knob all flight.
pub const ALTIMETER_TOLERANCE_INHG: f64 = 0.03;

/// The setting the altimeter SHOULD carry: local QNH below the transition
/// altitude, standard at or above it. Pure; the one rule behind the spoken
/// tool, full auto's departure set, the follower's arm and the monitor's
/// cross-check, so they can never disagree about what "correct" is.
pub fn correct_setting_inhg(
    indicated_alt_ft: f64,
    transition_altitude_ft: f64,
    qnh_pa: f64,
) -> f64 {
    if indicated_alt_ft >= transition_altitude_ft {
        STANDARD_INHG
    } else {
        (qnh_pa / PA_PER_INHG).clamp(28.0, 31.5)
    }
}

/// THIS FLIGHT's transition: the origin's altitude on the way up, the
/// destination's level on the way down, from the plan. `None` fields fall
/// back to the airframe's default. Process-wide because the consumers are
/// (the monitor's check, the spoken tool, full auto, the follower); set at
/// the arm, cleared when the mission ends or the airframe changes.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FlightTransition {
    pub altitude_ft: Option<f64>,
    pub level_ft: Option<f64>,
    /// The descent has begun: the destination's level is the one that counts.
    pub descending: bool,
}

static FLIGHT_TRANSITION: std::sync::Mutex<Option<FlightTransition>> = std::sync::Mutex::new(None);

pub fn publish_flight_transition(t: Option<FlightTransition>) {
    *FLIGHT_TRANSITION.lock().unwrap_or_else(|e| e.into_inner()) = t;
}

/// The follower has started down: from here the destination's level rules.
pub fn note_descending() {
    if let Some(t) = FLIGHT_TRANSITION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_mut()
    {
        t.descending = true;
    }
}

/// The transition altitude in force: the flight's when a plan gave one, else
/// `airframe_default_ft`. Every consumer of [`correct_setting_inhg`] resolves
/// its number here, so they still cannot disagree.
pub fn transition_in_force_ft(airframe_default_ft: f64) -> f64 {
    resolve_transition(
        *FLIGHT_TRANSITION.lock().unwrap_or_else(|e| e.into_inner()),
        airframe_default_ft,
    )
}

pub fn resolve_transition(flight: Option<FlightTransition>, airframe_default_ft: f64) -> f64 {
    let Some(t) = flight else {
        return airframe_default_ft;
    };
    let (first, second) = if t.descending {
        (t.level_ft, t.altitude_ft)
    } else {
        (t.altitude_ft, t.level_ft)
    };
    first.or(second).unwrap_or(airframe_default_ft)
}

/// "three zero one zero" for 30.10: the way a setting is read on the air.
pub fn spoken_setting(inhg: f64) -> String {
    crate::atis::spoken_digits(&format!("{inhg:.2}").replace('.', ""))
}

/// What a [`sync_altimeter`] did: the knob before and after.
#[derive(Debug, Clone, Copy)]
pub struct AltimeterSync {
    pub was: f64,
    pub now: f64,
    /// What was asked: `now` is the knob, this is the intent.
    pub want: f64,
}

impl AltimeterSync {
    /// The knob reads what was asked.
    pub fn took(&self) -> bool {
        altimeter_took(self.now, self.want)
    }

    /// What a crew says after the set: the setting when it took, and both
    /// numbers when it did not (everything flown off the altimeter is wrong
    /// by the difference, so it is never a quiet line).
    pub fn line(&self) -> String {
        if self.took() {
            format!("Altimeter set, {}.", spoken_setting(self.now))
        } else {
            format!(
                "The altimeter did not take. It reads {}, the setting is {}. Set it by hand.",
                spoken_setting(self.now),
                spoken_setting(self.want)
            )
        }
    }

    /// The knob actually moved (by more than the tolerance).
    pub fn changed(&self) -> bool {
        (self.now - self.was).abs() >= ALTIMETER_TOLERANCE_INHG
    }
}

/// Set the altimeter to the correct setting for where the aircraft is now.
/// `None` when the sim gave no pressure (nothing written). Read back from the
/// knob, so `now` is what the pilot sees, not what was asked.
pub async fn sync_altimeter(
    xp: &crate::xplane::XPlaneClient,
    transition_altitude_ft: f64,
) -> Option<AltimeterSync> {
    let pa = xp.get_float(crate::datarefs::QNH_PASCALS).await.ok()?;
    if pa <= 0.0 {
        return None;
    }
    let alt = xp
        .get_float(crate::datarefs::ALTITUDE_FT)
        .await
        .unwrap_or(0.0);
    let was = xp.get_float(BARO).await.unwrap_or(0.0);
    let want = correct_setting_inhg(alt, transition_in_force_ft(transition_altitude_ft), pa);
    let now = set_altimeter(xp, want).await.ok()?;
    Some(AltimeterSync { was, now, want })
}

pub struct SetAltimeterCurrent {
    /// Transition altitude (ft MSL): at or above it the "correct" setting is
    /// standard pressure, not local QNH.
    pub transition_altitude_ft: f64,
}

#[async_trait]
impl Tool for SetAltimeterCurrent {
    fn name(&self) -> &str {
        "set_altimeter_current"
    }

    fn description(&self) -> &str {
        "Set the altimeter to the CORRECT setting automatically — no number needed. \
         Below the transition altitude that's the local pressure (QNH), read from \
         the environment; at or above it that's standard pressure (29.92). Use when \
         the pilot wants it right without stating a value: 'set the altimeter', 'set \
         altimeter to current', 'altimeter to field', 'set the local QNH', 'set \
         standard', 'altimeter standard', 'update the altimeter', 'correct the \
         altimeter'. For an explicit value ('altimeter two niner niner two') use \
         set_barometer instead."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Above the transition altitude everyone flies on standard (29.92), so the
        // "correct" setting there is standard, not QNH — dialling QNH in the flight
        // levels would be wrong. Below it, the sim's ambient sea-level pressure
        // (QNH) is correct: with it set the altimeter reads true MSL / field
        // elevation. Use indicated altitude to decide (it's what the pilot sees).
        let alt = ctx
            .xplane
            .get_float(crate::datarefs::ALTITUDE_FT)
            .await
            .unwrap_or(0.0);
        let pa = ctx.xplane.get_float(crate::datarefs::QNH_PASCALS).await?;
        let transition_ft = transition_in_force_ft(self.transition_altitude_ft);
        let inhg = correct_setting_inhg(alt, transition_ft, pa);
        // Through the proving setter: a raw write here was a silent no-op on
        // the 737, and the readback then "Rogered" whatever the dial held.
        let actual = set_altimeter(ctx.xplane, inhg).await?;
        if !altimeter_took(actual, inhg) {
            return Ok(AltimeterSync {
                was: actual,
                now: actual,
                want: inhg,
            }
            .line());
        }
        if alt >= transition_ft {
            let fl = (transition_ft / 100.0).round() as i64;
            return Ok(format!(
                "Roger, above the transition altitude — altimeter standard, {actual:.2}. \
                 (Local QNH below flight level {fl}.)"
            ));
        }
        Ok(format!("Roger, altimeter set to the local {actual:.2}."))
    }
}

pub struct SetPitotHeat {
    /// Airframe-specific switch when the stock pitot-heat command is a no-op
    /// (the profile's `pitot_heat_dataref`). The Laminar MD-82's ice script
    /// drives all pitot/stall heat from its overhead heat-selector knob
    /// (`laminar/md82/ice/heatknob`, 0 = off, 1–9 = on) and overwrites the
    /// stock datarefs every frame, so only the knob works. The stock mirror
    /// dataref is still used for the readback — the script keeps it honest.
    pub override_dataref: Option<String>,
}

#[derive(Deserialize)]
struct PitotArgs {
    on: bool,
}

#[async_trait]
impl Tool for SetPitotHeat {
    fn name(&self) -> &str {
        "set_pitot_heat"
    }

    fn description(&self) -> &str {
        "Turn pitot heat on or off (anti-ice for the airspeed system). Use for \
         'pitot heat on', 'pitot heat off', 'turn on pitot heat', 'pitot anti-ice \
         on', 'pitot heat', 'anti-ice on'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": { "on": {"type": "boolean", "description": "true = on, false = off"} },
            "required": ["on"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: PitotArgs = serde_json::from_value(args.clone())?;
        match &self.override_dataref {
            Some(dref) => {
                ctx.xplane
                    .set_float(dref, if a.on { 1.0 } else { 0.0 })
                    .await?;
            }
            None => {
                ctx.xplane
                    .run_command(if a.on {
                        CMD_PITOT_HEAT_ON
                    } else {
                        CMD_PITOT_HEAT_OFF
                    })
                    .await?;
            }
        }
        // The mirror dataref updates a frame behind an overridden switch (the
        // airframe's script copies switch → dataref in after_physics).
        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        let on = ctx.xplane.get_float(PITOT_HEAT).await? > 0.5;
        if on != a.on {
            return Ok(format!(
                "Tried, but pitot heat reads {}. Check the switch.",
                if on { "on" } else { "off" }
            ));
        }
        Ok(format!(
            "Roger, pitot heat {}.",
            if on { "on" } else { "off" }
        ))
    }
}

pub struct AlignGyro;

#[async_trait]
impl Tool for AlignGyro {
    fn name(&self) -> &str {
        "align_gyro"
    }

    fn description(&self) -> &str {
        "Align the directional gyro (heading indicator) to the magnetic compass. \
         This is the fix when the gyro/heading indicator has drifted or is wrong. \
         Use for 'align the gyro', 'sync the heading indicator', 'set the DG', \
         'sync the directional gyro', 'fix the gyro drift', 'the gyro is drifting', \
         'correct the heading indicator', 'the DG is off'."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        ctx.xplane.run_command(DG_SYNC).await?;
        // Confirm the drift is gone (best-effort). dg_drift_vac_deg reads near
        // 360 for a small *negative* drift, so fold it into -180..180 first —
        // otherwise a -4° drift would read back as "356 degrees".
        let raw = ctx.xplane.get_float(DG_DRIFT).await.unwrap_or(0.0);
        let drift = (raw + 180.0).rem_euclid(360.0) - 180.0;
        // If the vacuum's low, the gyro itself is failing — a sync won't hold, so
        // flag that instead of confirming a sync that'll just drift back off.
        if ctx.xplane.get_float(LOW_VACUUM).await.unwrap_or(0.0) > 0.5 {
            return Ok("Synced the gyro, but vacuum's low — it won't hold. \
                       Use the compass and cross-check the attitude indicator."
                .to_string());
        }
        // A couple of degrees is a good sync (the compass itself isn't perfect).
        if drift.abs() < 2.0 {
            Ok("Roger, directional gyro synced to the compass.".to_string())
        } else {
            Ok(format!(
                "Gyro synced; still {:.0} degrees of drift.",
                drift.abs()
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    /// EGPH-EGTE, 2026-09-21: the plan said 6,000; the 737's profile said
    /// 18,000, and the cruise at "one seven thousand" was flown on QNH.
    #[test]
    fn the_plans_transition_outranks_the_airframes() {
        let none = None;
        assert_eq!(resolve_transition(none, 18_000.0), 18_000.0);
        let uk = FlightTransition {
            altitude_ft: Some(6_000.0),
            level_ft: Some(7_000.0),
            descending: false,
        };
        assert_eq!(resolve_transition(Some(uk), 18_000.0), 6_000.0);
        assert_eq!(
            correct_setting_inhg(17_000.0, resolve_transition(Some(uk), 18_000.0), 103_000.0),
            STANDARD_INHG
        );
        // Down from cruise the destination's level is the one that counts...
        let down = FlightTransition {
            descending: true,
            ..uk
        };
        assert_eq!(resolve_transition(Some(down), 18_000.0), 7_000.0);
        // ...and a plan with only one of the two uses it both ways.
        let half = FlightTransition {
            altitude_ft: Some(5_000.0),
            level_ft: None,
            descending: true,
        };
        assert_eq!(resolve_transition(Some(half), 18_000.0), 5_000.0);
        // A local plan carries neither: the airframe's default stands.
        assert_eq!(
            resolve_transition(Some(FlightTransition::default()), 18_000.0),
            18_000.0
        );
    }

    use super::*;

    /// The 737's dial write never stands (its xlua copies its own memory over
    /// it every frame): the setter must notice and step the knob there with
    /// the commands the aircraft handles, and the tool must not say "Roger"
    /// to a dial that still reads the old number.
    #[tokio::test]
    async fn a_dial_write_that_does_not_stand_is_stepped_and_never_rogered() {
        use crate::xplane::FakeXPlane;
        // The fake never reflects a write into its reads: the 737 exactly.
        let fake = FakeXPlane::with_reads(&[(BARO, 30.11)]);
        let now = set_altimeter_with(&fake, 30.00, &Default::default()).await.unwrap();
        assert!(fake.wrote(BARO), "the dial is tried first");
        assert_eq!(
            fake.ran_count(CMD_BARO_DOWN),
            11,
            "30.11 to 30.00 is eleven presses down"
        );
        assert!(!fake.ran(CMD_BARO_UP));
        // The knob still reads the old number (this fake cannot move it), so
        // the readback says so instead of Roger.
        assert!(!altimeter_took(now, 30.00));
        let cfg = crate::config::Config::default();
        let ctx = ToolCtx {
            xplane: &fake,
            config: &cfg,
            voice: None,
            status: None,
            reminders: None,
        };
        let out = SetBarometer
            .execute(&json!({"inhg": 30.00}), &ctx)
            .await
            .unwrap();
        assert!(out.starts_with("Altimeter didn't take"), "{out}");
    }

    /// EGSS, 2026-09-22: the 737 loaded with its baro memory at 0.0 and one
    /// capped round per ask left it at 17.5 after five asks. One ask gets
    /// there, a round at a time, because every round is answered.
    #[tokio::test]
    async fn a_knob_at_zero_is_walked_all_the_way_in_one_ask() {
        use crate::xplane::FakeXPlane;
        let fake = FakeXPlane::with_reads(&[(BARO, 0.0)]);
        fake.step_on_command(CMD_BARO_UP, BARO, BARO_STEP_INHG);
        let now = set_altimeter_with(&fake, 30.40, &Default::default()).await.unwrap();
        assert!(altimeter_took(now, 30.40), "{now}");
        assert_eq!(fake.ran_count(CMD_BARO_UP), 3040);
    }

    /// A knob that does not answer is pressed for ONE round, never ten.
    #[tokio::test]
    async fn a_dead_knob_gets_one_round() {
        use crate::xplane::FakeXPlane;
        let fake = FakeXPlane::with_reads(&[(BARO, 0.0)]);
        let now = set_altimeter_with(&fake, 30.40, &Default::default()).await.unwrap();
        assert_eq!(fake.ran_count(CMD_BARO_UP), BARO_MAX_STEPS);
        let sync = AltimeterSync {
            was: 0.0,
            now,
            want: 30.40,
        };
        assert!(!sync.took());
        assert!(
            sync.line().starts_with("The altimeter did not take"),
            "{}",
            sync.line()
        );
        assert!(keep_stepping(0.0, 3.5, 30.4, 350));
        assert!(!keep_stepping(0.0, 0.0, 30.4, 350));
        assert!(!keep_stepping(30.39, 30.40, 30.4, 1));
    }

    /// LIBP, 2026-09-26, QNH 1024 (30.238): the 737's dial read 30.06 after
    /// the write, 30.23 after eighteen presses (one still in the sim's
    /// queue), 30.25 after the nineteenth. The round after presses ONE back
    /// and the dial takes; a dead knob one step off still gets nothing.
    #[test]
    fn one_step_past_the_ask_is_pressed_back_when_the_dial_moved() {
        assert!(keep_stepping(30.06, 30.23, 30.238, 18), "closing: earned");
        assert!(
            keep_stepping(30.23, 30.25, 30.238, 1),
            "a step past after a press that moved it: one back"
        );
        assert!(
            !keep_stepping(30.23, 30.23, 30.238, 1),
            "one step short and not moving: a dead knob"
        );
        assert!(
            !keep_stepping(30.21, 30.27, 30.238, 6),
            "three past is a knob that runs, not a frame's lag"
        );
        assert_eq!(baro_steps(30.25, 30.238), (CMD_BARO_DOWN, 1));
    }

    #[tokio::test]
    async fn a_press_the_sim_lands_late_is_corrected_not_reported() {
        use crate::xplane::FakeXPlane;
        let fake = FakeXPlane::with_reads(&[(BARO, 30.06)]);
        fake.step_on_command(CMD_BARO_UP, BARO, BARO_STEP_INHG);
        fake.step_on_command(CMD_BARO_DOWN, BARO, -BARO_STEP_INHG);
        fake.land_press_late(18);
        let want = 1024.0 * 100.0 / PA_PER_INHG;
        let now = set_altimeter_with(&fake, want, &Default::default()).await.unwrap();
        assert!(altimeter_took(now, want), "{now}");
        assert_eq!(fake.ran_count(CMD_BARO_UP), 19);
        assert_eq!(fake.ran_count(CMD_BARO_DOWN), 1);
        let sync = AltimeterSync {
            was: 30.06,
            now,
            want,
        };
        assert!(sync.took());
        assert_eq!(sync.line(), "Altimeter set, three zero two four.");
    }

    #[test]
    fn baro_steps_count_hundredths_and_pick_the_direction() {
        assert_eq!(baro_steps(30.11, 30.00), (CMD_BARO_DOWN, 11));
        assert_eq!(baro_steps(29.92, 30.10), (CMD_BARO_UP, 18));
        assert_eq!(baro_steps(30.10, 30.104).1, 0);
        assert_eq!(
            baro_steps(0.0, 31.5).1,
            BARO_MAX_STEPS,
            "a round is bounded; the next one is earned (keep_stepping)"
        );
        assert!(altimeter_took(30.104, 30.10) && !altimeter_took(30.11, 30.10));
    }

    #[test]
    fn correct_setting_is_qnh_below_and_standard_above_transition() {
        // 101,946 Pa is the LINTO QNH of 2026-09-14: 30.10, not the 30.15 flown.
        let q = correct_setting_inhg(7_000.0, 18_000.0, 101_946.0);
        assert!((q - 30.10).abs() < 0.005, "{q}");
        assert_eq!(correct_setting_inhg(18_000.0, 18_000.0, 101_946.0), 29.92);
        assert_eq!(correct_setting_inhg(25_000.0, 6_000.0, 98_000.0), 29.92);
        // A silly pressure is clamped, never written raw.
        assert_eq!(correct_setting_inhg(0.0, 18_000.0, 0.0), 28.0);
    }

    /// EGSS, 2026-09-22: the 737's memory loaded at 0.0. With its STD button
    /// declared, one ask is two STD presses (29.92 from anywhere) and 48
    /// knob presses to 30.40, where the knob alone was 3,040.
    #[tokio::test]
    async fn the_737_goes_to_standard_first_and_steps_from_there() {
        use crate::xplane::FakeXPlane;
        let cfg = crate::aircraft::load("boeing_737_800")
            .unwrap()
            .altimeter
            .expect("the 737 declares its STD buttons");
        let std = cfg.std_command.clone().unwrap();
        assert_eq!(cfg.std_presses, 2);
        let fake = FakeXPlane::with_reads(&[(BARO, 0.0)]);
        fake.step_on_command(CMD_BARO_UP, BARO, BARO_STEP_INHG);
        fake.set_on_command(&std, BARO, STANDARD_INHG);
        let now = set_altimeter_with(&fake, 30.40, &cfg).await.unwrap();
        assert!(altimeter_took(now, 30.40), "{now}");
        assert_eq!(fake.ran_count(&std), 2);
        assert_eq!(fake.ran_count(CMD_BARO_UP), 48);
        // Standard itself is the button and nothing else.
        let fake = FakeXPlane::with_reads(&[(BARO, 17.50)]);
        fake.set_on_command(&std, BARO, STANDARD_INHG);
        let now = set_altimeter_with(&fake, STANDARD_INHG, &cfg).await.unwrap();
        assert!(altimeter_took(now, STANDARD_INHG));
        assert!(!fake.ran(CMD_BARO_UP) && !fake.ran(CMD_BARO_DOWN));
        // Close to the ask the knob is quicker and the button is left alone.
        assert!(!std_fast_path_pays(30.11, 30.00));
        assert!(std_fast_path_pays(0.0, 30.40));
        assert!(std_fast_path_pays(17.50, STANDARD_INHG));
    }

    /// A STD button that does nothing costs its presses and no more: the
    /// knob steps from where the dial really is.
    #[tokio::test]
    async fn a_std_button_that_does_not_take_leaves_the_knob_to_it() {
        use crate::xplane::FakeXPlane;
        let cfg = crate::aircraft::AltimeterProfile {
            std_command: Some("laminar/fake/std".into()),
            std_presses: 2,
            copilot_std_command: None,
        };
        let fake = FakeXPlane::with_reads(&[(BARO, 29.00)]);
        fake.step_on_command(CMD_BARO_UP, BARO, BARO_STEP_INHG);
        let now = set_altimeter_with(&fake, 30.40, &cfg).await.unwrap();
        assert!(altimeter_took(now, 30.40), "{now}");
        assert_eq!(fake.ran_count("laminar/fake/std"), 2);
        assert_eq!(fake.ran_count(CMD_BARO_UP), 140);
    }

    /// The copilot's side follows the captain's, through its own knob and
    /// STD button; a side that already agrees is not touched.
    #[tokio::test]
    async fn the_first_officers_altimeter_is_set_too() {
        use crate::xplane::FakeXPlane;
        let cfg = crate::aircraft::load("boeing_737_800")
            .unwrap()
            .altimeter
            .unwrap();
        let fo_std = cfg.copilot_std_command.clone().unwrap();
        let fake = FakeXPlane::with_reads(&[(BARO, 30.40), (BARO_COPILOT, 0.0)]);
        fake.step_on_command(CMD_BARO_COPILOT_UP, BARO_COPILOT, BARO_STEP_INHG);
        fake.set_on_command(&fo_std, BARO_COPILOT, STANDARD_INHG);
        let now = set_altimeter_with(&fake, 30.40, &cfg).await.unwrap();
        assert!(altimeter_took(now, 30.40));
        assert_eq!(fake.ran_count(&fo_std), 2);
        assert_eq!(fake.ran_count(CMD_BARO_COPILOT_UP), 48);
        assert!(!fake.ran(CMD_BARO_UP), "the captain's side already read right");
        // Agreeing: nothing pressed on either side.
        let fake = FakeXPlane::with_reads(&[(BARO, 30.40), (BARO_COPILOT, 30.40)]);
        set_altimeter_with(&fake, 30.40, &cfg).await.unwrap();
        assert!(!fake.ran(&fo_std) && !fake.ran(CMD_BARO_COPILOT_UP));
    }

    /// "Set the altimeter" on a knob that will not move is said, never
    /// "Roger" with the number the dial still holds.
    #[tokio::test]
    async fn set_the_altimeter_says_when_it_did_not_take() {
        use crate::xplane::FakeXPlane;
        let fake = FakeXPlane::with_reads(&[
            (crate::datarefs::ALTITUDE_FT, 2_000.0),
            (crate::datarefs::QNH_PASCALS, 102_946.0),
            (BARO, 30.11),
        ]);
        let cfg = crate::config::Config::default();
        let ctx = ToolCtx {
            xplane: &fake,
            config: &cfg,
            voice: None,
            status: None,
            reminders: None,
        };
        let out = SetAltimeterCurrent {
            transition_altitude_ft: 18_000.0,
        }
        .execute(&json!({}), &ctx)
        .await
        .unwrap();
        assert!(out.starts_with("The altimeter did not take"), "{out}");
    }

    /// The cross-check against the aircraft's true height, on the numbers
    /// of the three flights that shaped it.
    #[test]
    fn the_altimeter_is_cross_checked_against_true_altitude() {
        // EGSS 2026-09-22: 4,675 indicated at 16,884 true, knob 17.50, QNH
        // 30.40 (below the transition, so QNH is the reference too).
        let egss = altimeter_truth(4_675.0, 16_884.0, 17.50, 30.40, 30.40, -18.0);
        assert!(egss.setting_gross(), "{egss:?}");
        assert!(!egss.instrument_suspect(), "the knob explains it: {egss:?}");
        assert!(egss.error_ft > 12_000.0);
        // LINTO 2026-09-14: 6,838 indicated, 7,012 true, knob 30.15 against
        // 30.10, OAT 16 C (ISA+14): nothing wrong worth a word.
        let linto = altimeter_truth(6_838.0, 7_012.0, 30.15, 30.10, 30.10, 16.1);
        assert!(!linto.setting_gross() && !linto.instrument_suspect(), "{linto:?}");
        // A cold day at FL350 on standard over a low QNH: 3,100 ft of true
        // against pressure altitude and none of it a fault.
        let cold = altimeter_truth(35_000.0, 31_900.0, 29.92, 29.40, 29.92, -70.0);
        assert!(!cold.setting_gross() && !cold.instrument_suspect(), "{cold:?}");
        // A frozen static port: the knob is right and the altimeter reads
        // 5,000 at 9,000 true.
        let frozen = altimeter_truth(5_000.0, 9_000.0, 30.10, 30.10, 30.10, -5.0);
        assert!(frozen.instrument_suspect(), "{frozen:?}");
        assert!(!frozen.setting_gross());
    }

    #[test]
    fn a_setting_is_spoken_digit_by_digit() {
        assert_eq!(spoken_setting(30.10), "three zero one zero");
        assert_eq!(spoken_setting(29.92), "two niner niner two");
    }

    #[test]
    fn a_sync_within_tolerance_did_not_change_anything() {
        assert!(!AltimeterSync {
            was: 30.15,
            now: 30.16,
            want: 30.16
        }
        .changed());
        assert!(AltimeterSync {
            was: 30.15,
            now: 30.10,
            want: 30.10
        }
        .changed());
    }
}
