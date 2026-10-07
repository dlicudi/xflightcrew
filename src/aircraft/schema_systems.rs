//! Profile schema, the systems: fuel and its units, switches and selectors,
//! the APU and the engine starts, flaps, levers, gear and the Boeing start.

use super::*;

/// Fuel-endurance alert thresholds (minutes of fuel remaining at the current
/// burn). The monitor warns as the figure drops through these — caution first,
/// then warning. Defaults are the usual VFR reserves (30 min day / 45 min night
/// → caution at 45, warning at 30), per-airframe so a thirstier type can raise
/// them.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelProfile {
    /// Endurance (minutes) at or below which to caution.
    pub reserve_caution_min: f64,
    /// Endurance (minutes) at or below which to warn (more urgent).
    pub reserve_warning_min: f64,
    /// Nominal cruise fuel burn (kg/hr). Used by the *ground* check to estimate
    /// endurance before takeoff (current idle burn would read as near-infinite),
    /// so "you're low for a flight" can fire while parked. The 172 burns ~24 kg/hr.
    pub cruise_burn_kg_hr: f64,
    /// Density of what this airframe actually burns, kg per US gallon.
    /// `None` = 100LL (2.72), which is every piston here.
    ///
    /// Jet A-1 is 3.04. Briefing a 737's kerosene at avgas density inflated
    /// every fuel figure it spoke by 11.8% (BIKF-EGPH, 2026-09-20: 3,126 kg
    /// in the tanks read back as "1,149 gallons" — 1,028 is the truth).
    #[serde(default)]
    pub kg_per_gal: Option<f64>,
    /// The unit the CREW of this airframe works in: "gallons", "kg" or "lb".
    /// `None` = gallons.
    ///
    /// An airliner is never fuelled in gallons, and the app briefed one in
    /// them because gallons were the GA unit it was built around — while full
    /// auto's own preflight quoted the SAME fuel in pounds. Two parts of one
    /// cockpit, two units, one of them not the airframe's.
    #[serde(default)]
    pub spoken_unit: Option<String>,
}

/// How a given airframe's fuel is spoken.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FuelSpoken {
    Gallons,
    Kilos,
    Pounds,
}

/// 100LL, the default for anything that declares nothing.
pub const AVGAS_KG_PER_GAL: f64 = 2.72;
pub(super) const LB_PER_KG: f64 = 2.204_62;

/// An airframe's fuel density and spoken unit, from its profile. Pure.
pub fn fuel_unit_for(f: Option<&FuelProfile>) -> (f64, FuelSpoken) {
    let density = f
        .and_then(|f| f.kg_per_gal)
        .filter(|d| d.is_finite() && *d > 0.0)
        .unwrap_or(AVGAS_KG_PER_GAL);
    let unit = match f
        .and_then(|f| f.spoken_unit.as_deref())
        .unwrap_or("gallons")
        .trim()
        .to_ascii_lowercase()
        .as_str()
    {
        "kg" | "kgs" | "kilos" | "kilograms" => FuelSpoken::Kilos,
        "lb" | "lbs" | "pounds" => FuelSpoken::Pounds,
        _ => FuelSpoken::Gallons,
    };
    (density, unit)
}

/// Publish the airframe's fuel density and spoken unit alone.
#[cfg(test)]
pub fn publish_fuel_unit(f: Option<&FuelProfile>) {
    let u = fuel_unit_for(f);
    update_airframe_facts(|a| a.fuel_unit = u);
}

/// The live airframe's (kg per gallon, spoken unit), republished on every
/// profile swap: a process-wide fact must never outlive its airframe.
pub fn fuel_unit() -> (f64, FuelSpoken) {
    airframe_facts().fuel_unit
}

/// Thousands separators. A local copy on purpose: this module compiles into
/// the LIB as well as the bin, and `auto_status` is the bin's.
pub(super) fn commas(n: i64) -> String {
    let (sign, mut d) = (if n < 0 { "-" } else { "" }, n.abs().to_string());
    let mut out = String::new();
    while d.len() > 3 {
        out.insert_str(0, &format!(",{}", &d[d.len() - 3..]));
        d.truncate(d.len() - 3);
    }
    format!("{sign}{d}{out}")
}

/// `kg` of fuel as this airframe's crew would say it, with its unit word.
pub fn say_fuel_kg(kg: f64) -> String {
    say_fuel_figure(fuel_in_unit_kg(kg))
}

/// `kg` as a NUMBER in the airframe's unit — for a caller that has to round
/// before it subtracts. Three independently rounded figures do not add up:
/// "You have 1149 aboard, but the plan needs 3002; short about 1852" was off
/// by one because each was rounded from its own float (2026-09-20).
pub fn fuel_in_unit_kg(kg: f64) -> f64 {
    let (density, unit) = fuel_unit();
    match unit {
        FuelSpoken::Gallons => kg / density,
        FuelSpoken::Kilos => kg,
        FuelSpoken::Pounds => kg * LB_PER_KG,
    }
}

/// The same figure with no unit word — for the second and third numbers in a
/// list that has already named the unit once.
pub fn fuel_figure_kg(kg: f64) -> String {
    commas(fuel_in_unit_kg(kg).round() as i64)
}

/// A fuel FLOW in kg/hr, as this airframe's crew would say it. Same rule as
/// the quantity: an airliner's burn is tonnes an hour, not gallons an hour at
/// the density of petrol.
pub fn say_fuel_flow_kg_hr(kg_hr: f64) -> String {
    let (density, unit) = fuel_unit();
    match unit {
        FuelSpoken::Gallons => format!("{:.1} gallons an hour", kg_hr / density),
        FuelSpoken::Kilos => format!("{} kilos an hour", commas(kg_hr.round() as i64)),
        FuelSpoken::Pounds => format!(
            "{} pounds an hour",
            commas((kg_hr * LB_PER_KG).round() as i64)
        ),
    }
}

/// A number already in the airframe's unit, said with its unit word.
pub fn say_fuel_figure(n: f64) -> String {
    let word = match fuel_unit().1 {
        FuelSpoken::Gallons => "gallons",
        FuelSpoken::Kilos => "kilos",
        FuelSpoken::Pounds => "pounds",
    };
    format!("{} {word}", commas(n.round() as i64))
}

/// Fuel tank selector positions, per airframe — the tool's whole surface is
/// config, because airframes differ: the 172 selector is LEFT/BOTH/RIGHT (OFF is
/// a separate shutoff valve, not a selector position); the SR22 is OFF/LEFT/RIGHT
/// with no "both". Each position fires a `command` (commands animate the 3D knob;
/// a raw dataref write may not), then — when `readback` is set and the position
/// carries a known `value` — the dataref is read to confirm; otherwise the
/// command is trusted.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelSelectorProfile {
    /// Dataref read after switching to confirm the position (an enum int).
    /// `None` → trust the command (no confirmation read).
    #[serde(default)]
    pub readback: Option<String>,
    /// Selectable positions, in the order offered to the LLM.
    pub positions: Vec<FuelPosition>,
}

/// One fuel-selector position.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelPosition {
    /// Token the LLM selects by and returns (e.g. `"left"`, `"both"`, `"off"`).
    pub key: String,
    /// Spoken form for the readback (e.g. `"left"`, `"both"`, `"off"`).
    pub spoken: String,
    /// Command that selects this position (preferred — animates the knob).
    pub command: String,
    /// Expected `readback` value for this position. `None` → can't confirm by
    /// value, so the command is trusted.
    #[serde(default)]
    pub value: Option<f64>,
}

/// A multi-position switch driven by up/down CYCLE commands rather than a
/// direct "set to N" — the AWX DC-3 pattern (power switch, fuel selectors). To
/// reach a target the tool reads `position`, then fires `up`/`down` the right
/// number of times. (Writing the position dataref doesn't work on the DC-3: the
/// add-on's dataref handlers are no-ops, so only the commands move the system.)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CycleSwitch {
    /// Integer-position dataref read to find the current detent.
    pub position: String,
    /// Command that increments the position by one.
    pub up: String,
    /// Command that decrements the position by one.
    pub down: String,
    /// Lowest detent value.
    pub min: i64,
    /// Highest detent value.
    pub max: i64,
    /// Whether the switch wraps `max`→`min` (the fuel selectors do; the power
    /// switch is clamped at the ends).
    #[serde(default)]
    pub wrap: bool,
}

/// One named target position of a [`CycleSwitch`] (the token the LLM selects by,
/// its spoken form, and the detent value to cycle to).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchState {
    pub key: String,
    pub spoken: String,
    pub value: i64,
}

/// The DC power switch (battery / off / ground power) on a cycle switch. Gates
/// the `set_power` tool. `None` = no power tool.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerSwitchProfile {
    pub switch: CycleSwitch,
    /// Selectable named positions, in the order offered to the LLM.
    pub states: Vec<SwitchState>,
    /// Dataref read to confirm the battery is live (any element > 0). `None` =
    /// trust the switch position.
    #[serde(default)]
    pub battery_readback: Option<String>,
}

/// One fuel selector on a cycle switch (the DC-3 has one per side). Gates the
/// `set_fuel_selector` tool when any are present; also drives the start
/// sequence's "fuel to a main tank" step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CycleSelectorProfile {
    /// Token the LLM selects this selector by (e.g. `left`, `right`).
    pub key: String,
    /// Spoken form (e.g. `left fuel selector`).
    pub spoken: String,
    pub switch: CycleSwitch,
    /// Engine index (0-based) this selector feeds — for the start sequence.
    #[serde(default)]
    pub engine: usize,
    /// Position value to select for engine start (a main tank). `None` = leave
    /// the selector where it is during start.
    #[serde(default)]
    pub start_value: Option<i64>,
    /// Selectable positions, in the order offered to the LLM.
    pub positions: Vec<SwitchState>,
}

/// Twin (or more) turboprop start, switch/condition-lever driven — the Laminar
/// King Air pattern. Gates `start_engines` / `stop_engines` for turbines. The
/// sequence per the verified C90B start: condition levers to low idle, then for
/// each engine fire its start switch (crank), wait for N1 to reach idle, release
/// the starter, bring that engine's generator online (so the bus can power the
/// next start), and finally condition levers to the run position.
/// The APU bring-up a jet's engine start needs first.
///
/// A turbofan is motored by BLEED AIR, and on the Citation X the APU also
/// powers a bus the engine's own start gate reads. Verified live 2026-07-28:
/// with batteries on but the APU off, the starter command is accepted, the
/// annunciator lights and `ignition_key` goes to 4 — and the engine does not
/// turn at all. With the APU generator and bleed on, the same command spooled
/// N2 from 0 to 30% in fifteen seconds. A turboprop needs none of this, so the
/// whole block is optional.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApuStartProfile {
    /// Toggle for the APU master, fired only if `master_readback` says it's off.
    pub master_toggle: String,
    pub master_readback: String,
    /// The starter command, HELD until the APU lights (see `light_n1`).
    ///
    /// It is a spring-loaded momentary on the Citation X, and the script only
    /// engages the starter after the command has been held longer than a
    /// second: `phase == 1 and duration > 1.0`, springing back to ON the
    /// instant it is released. Firing it twice for a second and a half each —
    /// which is what "press it, then press it again" looks like in code — gives
    /// the APU about half a second of starter and it never lights.
    pub starter_commands: Vec<String>,
    /// APU speed at which it is lit and self-sustaining, so the starter can be
    /// let go and it spools to `ready_n1` on its own. The aircraft's own
    /// checklist says "HOLD TILL START" against N1 > 10.
    #[serde(default = "default_apu_light_n1")]
    pub light_n1: f64,
    /// APU speed dataref and the value that counts as running.
    pub n1_dataref: String,
    #[serde(default = "default_apu_ready_n1")]
    pub ready_n1: f64,
    /// How long to wait for it to come up (it took ~25 s live).
    #[serde(default = "default_apu_ready_secs")]
    pub ready_secs: f64,
    /// Fired once it's running — generator, then bleed air.
    #[serde(default)]
    pub after_ready_commands: Vec<String>,
}

pub(super) fn default_apu_ready_n1() -> f64 {
    95.0
}
pub(super) fn default_apu_light_n1() -> f64 {
    15.0
}
pub(super) fn default_apu_ready_secs() -> f64 {
    90.0
}

/// One switch on the way to a running APU: fire `command`, but only when the
/// switch is not already where we want it.
///
/// Read-before-fire, because these are TOGGLES: firing one blind turns a
/// switch that was already on back off. The same rule the transponder knob
/// and the light pairs learned, and the reason `position` is a separate name
/// from `command` — on this aircraft they are frequently the same string
/// meaning two different things.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StartStep {
    pub command: String,
    /// The switch POSITION to read first. `None` fires unconditionally, which
    /// is right for a momentary button and wrong for anything that latches.
    #[serde(default)]
    pub position: Option<String>,
    #[serde(default = "one")]
    pub want: f64,
    /// A momentary press-and-hold (the APU START button is one), seconds.
    /// `None` is a single press.
    #[serde(default)]
    pub held_secs: Option<f64>,
    /// For a KNOB rather than a toggle: the command that steps the other way,
    /// used when the position sits ABOVE `want`.
    ///
    /// The ADIRS selectors are three-position (OFF / NAV / ATT) and the
    /// aircraft's own script picks a direction from where the knob is. A
    /// toggle model cannot say that: told only "press right until it reads
    /// NAV", a knob left in ATT walks the wrong way.
    #[serde(default)]
    pub down_command: Option<String>,
    /// A milestone SPOKEN once this step has confirmed ("Fuel pumps on, eight
    /// of eight."). Most steps carry none: a crew's flow is silent, and the
    /// voice says the state of the aircraft, not the name of every switch.
    /// Every step is still blackboxed and shown on the panel transcript.
    #[serde(default)]
    pub say: Option<String>,
    /// A KNOB more than one detent from `want` (the 737's IRS selectors,
    /// OFF / ALIGN / NAV / ATT; its AC meter and autobrake knobs): stepped
    /// one press at a time, each read back, up to this many presses. `None`
    /// is the single press every step had before 2026-10-07.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_presses: Option<u32>,
    /// Run the step only while a reading is above a value (the 737's centre
    /// tank pumps, on only with more than 453 kg in the tank). A failed read
    /// skips the step: no measurement is no reason to switch.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub when_above: Option<StepCondition>,
}

/// A reading a [`StartStep`] waits on: `dataref` (which may be `name[i]`)
/// above `value`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StepCondition {
    pub dataref: String,
    pub value: f64,
}

/// The crew's preflight, from the battery to ready for the start, on the
/// airframe's own switches (the Zibo's: its `clist.txt` PRELIMINARY
/// FLIGHTDECK and PREFLIGHT PREPARATION sections). Run by full auto once the
/// bus is live and before the ramp items, so the lights it reads back are
/// powered (the Zibo's lamps are AC: on the battery alone every exterior
/// lamp reads 0 whatever its switch says, probed 2026-10-07).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PreflightProfile {
    /// Bring the APU and its generators and bleed up FIRST, through the
    /// start's own APU steps (`boeing_start` `apu_steps` and
    /// `apu_power_steps`): the clist's "APU STARTED" and "APU GENERATORS
    /// ON" open the preliminary flight deck procedure.
    #[serde(default)]
    pub apu_first: bool,
    /// The switches, in the checklist's order: read before fire, read back,
    /// at a crew's pace (`jet_start::run_steps`).
    #[serde(default)]
    pub steps: Vec<StartStep>,
    /// The inertial reference alignment the steps begin ([`AlignWait`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub align: Option<AlignWait>,
    /// Switches that hold only on an ALIGNED platform, run once the wait has
    /// it: the Zibo's yaw damper dropped every preflight while the IRS was
    /// aligning, and one press held at once with it in NAV (LEMG,
    /// 2026-10-07).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub after_aligned: Vec<StartStep>,
}

/// An alignment begun by the preflight and waited on before the taxi: the
/// IRS gives the attitude the autopilot flies on, so no mission rolls on an
/// unaligned platform. Every name is the airframe's.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlignWait {
    /// Each system's mode; aligned when every one reads `want`.
    pub modes: Vec<String>,
    #[serde(default = "two")]
    pub want: f64,
    /// Seconds of alignment left, for the spoken wait; optional.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub remaining: Option<String>,
    /// How long the taxi waits on it before handing back.
    #[serde(default = "align_timeout_default")]
    pub timeout_secs: f64,
    /// The present position entered for the alignment, as a crew types it
    /// into POS INIT ([`PositionEntry`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position_entry: Option<PositionEntry>,
    /// The mode knobs, one per entry of `modes` in its order, each cycled
    /// OFF and back when its system reads off after the preflight's
    /// switches or at the taxi's wait ([`KnobCycle`]): a knob already at NAV
    /// begins nothing.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub restart: Vec<KnobCycle>,
}

/// One knob turned down to its stop and back: `presses` of `off`, then as
/// many of `on`. The Zibo's IRS begins aligning only on the knob's MOVE with
/// power: a runway start left both at NAV with the IRS off (mode 0), the
/// preflight read them "already set", and full auto gave up at the taxi
/// (LEMG, 2026-10-07); OFF and back to NAV read mode 1, 384 s left.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KnobCycle {
    pub off: String,
    pub on: String,
    pub presses: u32,
}

/// Where the alignment takes its present position: a `data` dataref the
/// position is written to as `N5111.1W00102.5`, then a command that hands it
/// to the IRS (the Zibo's `FMS/irs_pos_fmc_set` and `fmc_to_irs_pos`, read
/// from B738.LATLON.lua; without it a first alignment ends flashing ALIGN,
/// waiting for the entry, and never reaches NAV).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PositionEntry {
    pub dataref: String,
    pub command: String,
}

pub(super) fn two() -> f64 {
    2.0
}
pub(super) fn align_timeout_default() -> f64 {
    900.0
}
pub(super) fn yes() -> bool {
    true
}
pub(super) fn is_true(v: &bool) -> bool {
    *v
}

/// The airliner start: bleed air up, ENG MODE to IGN/START, each master
/// switch lifted past its guard and set, then the knob back to NORM.
///
/// Every field is a name the aircraft answers to, so every one is a
/// hypothesis until the swap-time probe confirms it — `profile_references`
/// walks this like the rest of the profile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AirlinerStartProfile {
    /// ENG MODE selector, and the values for IGN/START and NORM.
    pub mode_selector: String,
    pub mode_start: f64,
    pub mode_norm: f64,
    /// Engine master switch ARRAY — one element per engine.
    pub masters: String,
    #[serde(default = "one")]
    pub master_on: f64,
    #[serde(default)]
    pub master_off: f64,
    /// Per-engine guard the master sits under, lifted to `guard_lift` while
    /// the switch is set and dropped `guard_secs` later. Empty when the
    /// switch has no guard. The A330's masters do: its own autostart lifts,
    /// sets, and drops after a quarter second — a bare switch write with the
    /// guard down is the kind of silent no-op a study-level add-on is full of.
    #[serde(default)]
    pub master_guards: Vec<String>,
    #[serde(default = "guard_lift_default")]
    pub guard_lift: f64,
    #[serde(default = "guard_secs_default")]
    pub guard_secs: f64,
    /// The order the engines are started in, as ENGINE INDICES (0 = number
    /// one). Empty = 0, 1, 2... The A330's POM starts number two first, the
    /// way Airbus crews do; the MD-82's and the 737's start number one.
    #[serde(default)]
    pub start_order: Vec<usize>,
    /// Bringing the APU up: an ORDERED list of switch presses, because that
    /// is what the aircraft has. Batteries, APU battery, APU master, APU
    /// START, bleed.
    ///
    /// Not a list of datarefs. `laminar/A333/buttons/APU_start` exists as
    /// BOTH a dataref and a command — the dataref is the button's position,
    /// which the aircraft rewrites every frame, so setting it is the silent
    /// no-op a study-level add-on is full of. It was declared as a dataref
    /// here first and the APU never moved (2026-09-20).
    #[serde(default)]
    pub apu_steps: Vec<StartStep>,
    /// The N1 gauge that says the bleed source is actually there.
    pub apu_n1: String,
    /// Everything between a running APU and the first master switch: bleed
    /// air, the beacon, and the FUEL PUMPS. Same read-before-fire rule.
    ///
    /// The pumps are the reason this list exists. The first version of this
    /// block went straight from the APU to the master switches, which is a
    /// start with no fuel at the engine — and the aircraft's own sequence
    /// spends eight of its twenty-four steps on them (2026-09-20).
    #[serde(default)]
    pub pre_start_steps: Vec<StartStep>,
    /// ...and everything after both engines are running: packs, APU bleed
    /// off, APU off. Leaving the APU running on its bleed is not a failed
    /// start, but it is not a finished one either.
    #[serde(default)]
    pub post_start_steps: Vec<StartStep>,
    /// The APU N1 at which it can supply start air.
    #[serde(default = "apu_ready_default")]
    pub apu_ready_n1: f64,
    /// Spool gauge and the value that means this engine is away. N2 on most,
    /// N3 on a three-spool Trent — the profile names which dataref.
    pub spool: String,
    #[serde(default = "spool_idle_default")]
    pub spool_idle: f64,
}

pub(super) fn one() -> f64 {
    1.0
}
pub(super) fn guard_lift_default() -> f64 {
    0.002
}
pub(super) fn guard_secs_default() -> f64 {
    0.25
}
pub(super) fn apu_ready_default() -> f64 {
    95.0
}
pub(super) fn spool_idle_default() -> f64 {
    50.0
}

/// A helicopter's engine start, measured live on the S-76C (2026-09-22):
/// throttle 0 is OFF and anything above it lets fuel in (the mixture follows
/// the throttle by itself), the starter is held, each engine settles at about
/// 62 % N1 on the idle throttle, and both throttles to FLY bring the rotor to
/// its governed speed.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HeliStartProfile {
    /// Per-engine starter command, HELD while the engine cranks.
    pub starters: Vec<String>,
    /// Per-engine throttle array: 0 off, `idle` to start, `fly` to run.
    pub throttle: String,
    pub idle: f64,
    pub fly: f64,
    /// N1 at which an engine is lit and self-sustaining on the idle throttle.
    pub idle_n1: f64,
    /// How long one engine may crank before it is given up on.
    #[serde(default = "heli_crank_secs")]
    pub crank_secs: f64,
    /// Collective array (element 0 is the lever). A start with it up is refused.
    pub collective: String,
    /// Rotor brake ratio, released before the start.
    #[serde(default)]
    pub rotor_brake: Option<String>,
    /// Rotor speed (element 0) and the governed figure it should reach.
    pub rotor_rpm: String,
    pub rotor_rpm_governed: f64,
    /// Fired once both engines are at FLY.
    #[serde(default)]
    pub generators: Vec<String>,
    /// Fired after the generators: what the rest of the start leaves on so
    /// the autopilot can engage (the S-76's autopilot refused every mode
    /// until the stick trim / yaw damper and the inverter were on).
    #[serde(default)]
    pub after_start: Vec<String>,
    /// The tail wheel lock's TOGGLE command (the engage command did nothing on
    /// the S-76), fired only when the lock reads off. Unlocked, rotor torque
    /// pivoted the S-76 on its main wheels at up to 20 deg/s with the brake
    /// set and the pedals neutral, until the engines were cut (ETNW,
    /// 2026-09-23).
    #[serde(default)]
    pub tail_wheel_lock: Option<String>,
}

pub(super) fn heli_crank_secs() -> f64 {
    40.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TurbineStartProfile {
    /// Condition-lever (fuel) array dataref. Each engine's element is written.
    pub condition_lever: String,
    /// Condition value for starting (low idle) and for normal run (high idle).
    pub condition_start: f64,
    pub condition_run: f64,
    /// Per-engine command that engages the starter (the ign/start switch UP).
    pub start_up: Vec<String>,
    /// Per-engine command that releases the starter (the switch back to centre).
    /// May be empty when `starter_held` is set (nothing to release).
    #[serde(default)]
    pub start_dn: Vec<String>,
    /// The starter is a momentary press-and-hold control, not a latching switch:
    /// `start_up` is *held* for up to `crank_secs` (the Web API's command
    /// duration) instead of fired once, and `start_dn` is skipped. The Lancair
    /// Evolution's ENGINE START button (`sim/starters/engage_starter_1`) works
    /// this way; the C90's ign/start switch latches and stays `false`.
    #[serde(default)]
    pub starter_held: bool,
    /// Per-engine generator-ON command, fired once that engine reaches idle.
    #[serde(default)]
    pub generators: Vec<String>,
    /// The airframe's OWN securing control, fired by `stop_engines` after the
    /// condition levers go to cutoff. Empty = the levers are the whole story,
    /// which is the case on every free turbine here.
    ///
    /// Same shape as [`AircraftProfile::fuel_pump_commands`] and
    /// [`TransponderModeKnob`]: the universal write is not wrong, it is simply
    /// not the control the aircraft is secured with. The SF50 is the case that
    /// found this — its POM's Parking checklist says "ENGINE START / STOP -
    /// STOP", which is the ignition knob to OFF, and sf50_ignition.lua's
    /// handler is what actually fires `sim/starters/shut_down_1` *and* drops
    /// the boost pump and igniter with it. Cutting the mixture alone winds the
    /// engine down but leaves the knob at RUN with the pump and igniter live,
    /// which is not a secured aircraft.
    ///
    /// Fired AFTER the cutoff, never instead of it: the lever write is the
    /// backstop if the command is a no-op, and every airframe that declares
    /// nothing here behaves exactly as it did before this existed.
    #[serde(default)]
    pub shutdown_commands: Vec<String>,
    /// N1 (%) at which the engine is lit and stable enough to cut the starter.
    #[serde(default = "default_n1_idle")]
    pub n1_idle: f64,
    /// Introduce fuel at this N2 (%) instead of the default N1 threshold.
    ///
    /// A free turbine and a turbofan spool differently, and the sequence was
    /// written against the first: fuel goes in at 12% N1, which on a PT6 is a
    /// well-lit starter motor. On the Citation X's AE3007 the aircraft's own
    /// procedure introduces fuel at 10% N2, where N1 is barely off the stop —
    /// wait for 12% N1 there and the starter has already auto-cut (its script
    /// stops it at 40% N2) with no fuel ever introduced. `None` keeps the N1
    /// behaviour every turboprop profile was tuned on.
    #[serde(default)]
    pub fuel_intro_n2: Option<f64>,
    /// Max seconds to crank one engine before giving up.
    #[serde(default = "default_turbine_crank_secs")]
    pub crank_secs: f64,
    /// Turn each engine's boost pump on before cranking it (the C90 POM's fuel
    /// panel: the boost pump feeds the engine "before it starts" — the
    /// engine-driven pump takes over once running). Written to the universal
    /// `fuel_pump_on[i]`.
    #[serde(default)]
    pub boost_pumps: bool,
    /// Per-engine auto-ignition ARM command, fired before cranking (POM: "Engine
    /// Auto Ignition Arm – ON"). Empty = airframe has none.
    #[serde(default)]
    pub ignition_arm: Vec<String>,
    /// The APU bring-up this airframe needs before it can crank at all.
    /// `None` on every free turbine (see [`ApuStartProfile`]).
    #[serde(default)]
    pub apu: Option<ApuStartProfile>,
}

pub(super) fn default_n1_idle() -> f64 {
    50.0
}
pub(super) fn default_turbine_crank_secs() -> f64 {
    30.0
}

/// A notched mixture lever (the AWX DC-3's xlua-overridden control: idle
/// cutoff / auto-lean / auto-reach / max, written as 0..3). Gates `set_mixture`.
/// Distinct from the GA singles' continuous 0–100% mixture (the `engine` group).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MixtureNotchProfile {
    /// Lever dataref written to select a notch (drives `ENGN_mixt` via override).
    pub lever: String,
    /// Readback dataref to confirm the notch settled. `None` = trust the write.
    #[serde(default)]
    pub readback: Option<String>,
    /// Selectable notches, lean → rich, in the order offered to the LLM.
    pub notches: Vec<SwitchState>,
}

/// Twin piston start/shutdown for cycle-switch airframes whose mixture and fuel
/// are xlua-overridden (so the generic piston `engine` group can't drive them).
/// Gates `start_engines` / `stop_engines`. Universal datarefs (throttle, engine
/// running, bus volts) stay in the tool; the per-airframe commands live here.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PistonStartProfile {
    /// Per-engine starter commands, held to crank.
    pub starters: Vec<String>,
    /// Command setting all magnetos to BOTH (start) and OFF (shutdown).
    pub mags_both: String,
    pub mags_off: String,
    /// Notched mixture lever dataref (xlua-overridden); the rich notch used for
    /// start and the cutoff notch used for shutdown.
    pub mixture_lever: String,
    pub mixture_start_notch: f64,
    pub mixture_cutoff_notch: f64,
    /// Seconds to hold each starter.
    #[serde(default = "default_crank_secs")]
    pub crank_secs: f64,
    /// Throttle crack fraction (0..1) for the start.
    #[serde(default = "default_throttle_crack")]
    pub throttle_crack: f64,
    /// Per-engine generator/alternator ON commands, fired once running.
    #[serde(default)]
    pub generators: Vec<String>,
}

pub(super) fn default_crank_secs() -> f64 {
    6.0
}
pub(super) fn default_throttle_crack() -> f64 {
    0.12
}

/// One phase-expectation rule: in the named phases, the dataref should read on
/// (or off); otherwise the copilot speaks `say` once, as an Advisory.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdvisorItem {
    /// Switch/annunciator to read (scalar float; > 0.5 = on).
    pub dataref: String,
    /// Expected state in the listed phases (default: on).
    #[serde(default = "advisor_expect_on_default")]
    pub expect_on: bool,
    /// Phases where the expectation applies, lowercase (`ground`, `takeoff`,
    /// `climb`, `cruise`, `approach`, `landing`).
    pub phases: Vec<String>,
    /// What the copilot says when the state doesn't match.
    pub say: String,
    /// A command the copilot FIRES to fix the state itself, rather than only
    /// advising — the strobe/landing-light SOP rows the pilot wants actioned,
    /// not asked about. `None` = advisory only (the pilot's switch to flip).
    #[serde(default)]
    pub command: Option<String>,
}

pub(super) fn advisor_expect_on_default() -> bool {
    true
}

/// A flap system as an ordered list of detents, retracted → fully extended.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlapProfile {
    pub detents: Vec<FlapDetent>,
    /// The flap-handle ratio set for takeoff (0.0 = clean). `None` means the
    /// auto-takeoff leaves the flaps where they are. 0.0 on a clean-takeoff type
    /// (the 172) still matters — it ensures the config before the roll.
    #[serde(default)]
    pub takeoff_ratio: Option<f64>,
    /// The lever is stepped by commands, one detent per press, never
    /// written (`tools::flaps::set_handle`). The Zibo 737: a write to the
    /// stock `flap_ratio` is put straight back, while the stock
    /// `flaps_down` / `flaps_up` move its own lever one detent each
    /// (probed parked, 2026-10-06). `None` writes the stock handle.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub by_command: Option<LeverCommands>,
}

/// A lever that answers only to its step commands: one notch per press, read
/// back from the lever itself after every press
/// ([`crate::tools::lever::step_lever`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LeverCommands {
    /// One notch toward MORE (flaps down, speedbrake out).
    pub more: String,
    /// One notch toward LESS (flaps up, speedbrake in).
    pub less: String,
    /// The lever's own position, read before every press and after it.
    pub readback: String,
    /// How long a press is given to MOVE the lever before it is named as
    /// not taken.
    #[serde(default = "lever_step_timeout_default")]
    pub step_timeout_s: f64,
}

pub(super) fn lever_step_timeout_default() -> f64 {
    1.5
}

/// A speedbrake lever stepped by commands (`tools::controls::speedbrake_set`),
/// for an airframe whose own lever ignores the stock `speedbrake_ratio` (the
/// Zibo: the stock write sticks in the stock dataref while the lever never
/// moves, 2026-10-06). The positions are the LEVER's; the callers keep the
/// stock sense (-0.5 armed, 0 down, 0.5 the flight detent, 1 full), mapped
/// here both ways.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpeedbrakeLever {
    pub more: String,
    pub less: String,
    pub readback: String,
    #[serde(default = "lever_step_timeout_default")]
    pub step_timeout_s: f64,
    #[serde(default)]
    pub down: f64,
    pub armed: f64,
    /// The flight detent: what the stock sense calls 0.5.
    pub flight: f64,
    #[serde(default = "one")]
    pub full: f64,
}

impl SpeedbrakeLever {
    /// How close the lever must read to a position to be AT it.
    pub const TOLERANCE: f64 = 0.02;

    pub fn commands(&self) -> LeverCommands {
        LeverCommands {
            more: self.more.clone(),
            less: self.less.clone(),
            readback: self.readback.clone(),
            step_timeout_s: self.step_timeout_s,
        }
    }

    /// The lever position for a stock-sense value: armed below -0.25, down
    /// to 0.05, the flight detent short of full, else full. Pure.
    pub fn lever_for(&self, stock: f64) -> f64 {
        if stock < -0.25 {
            self.armed
        } else if stock <= 0.05 {
            self.down
        } else if stock < 0.99 {
            self.flight
        } else {
            self.full
        }
    }

    /// The stock sense of a lever reading: -0.5 armed, 0 down, the flight
    /// detent 0.5, full 1, between them in proportion. Pure.
    pub fn stock_for(&self, lever: f64) -> f64 {
        let tol = Self::TOLERANCE;
        if (lever - self.armed).abs() <= tol {
            -0.5
        } else if lever <= self.down + tol {
            0.0
        } else if lever >= self.full - tol {
            1.0
        } else if lever <= self.flight {
            let span = (self.flight - self.armed).max(1e-6);
            (0.5 * (lever - self.armed) / span).clamp(0.0, 0.5)
        } else {
            let span = (self.full - self.flight).max(1e-6);
            (0.5 + 0.5 * (lever - self.flight) / span).clamp(0.5, 1.0)
        }
    }
}

/// The parking brake's toggle and the add-on's own position
/// ([`AircraftProfile::park_brake_toggle`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParkBrakeToggle {
    pub command: String,
    /// The add-on's own brake position, `>= 0.5` set.
    pub position: String,
}

/// The gear lever's OWN commands and handle, for an airframe whose script
/// rewrites the stock gear commands (the Zibo: stock `landing_gear_down` is
/// "off, down", two presses from UP). Declared, both the `gear` tool and
/// every loop that raises or lowers the gear go through
/// [`crate::tools::gear::set_handle`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GearCommands {
    pub up: String,
    pub down: String,
    /// The handle's own position, and its values UP and DOWN.
    pub handle: String,
    #[serde(default)]
    pub handle_up: f64,
    #[serde(default = "one")]
    pub handle_down: f64,
}

/// A Boeing start: a starter knob and a start lever PER ENGINE (the 737:
/// starter to GRD, the lever to IDLE at 25 % N2, the knob springs back to
/// AUTO at about 56 %). Flown by `tools::jet_start::BoeingStart`; the steps
/// and limits are the aircraft's own checklist's (the Zibo's `clist.txt`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoeingStartProfile {
    /// The engines, IN START ORDER (a 737 starts number two first).
    pub engines: Vec<BoeingEngineStart>,
    /// Start air: the duct pressure gauges; the highest must read
    /// `min_duct_psi` before a starter is turned.
    pub duct_pressure: Vec<String>,
    #[serde(default = "min_duct_psi_default")]
    pub min_duct_psi: f64,
    /// The starter knob's GRD value and the value it reads once released
    /// (AUTO / OFF).
    #[serde(default)]
    pub starter_grd: f64,
    #[serde(default = "one")]
    pub starter_released: f64,
    /// The start lever's IDLE and CUTOFF values.
    #[serde(default = "one")]
    pub lever_idle: f64,
    #[serde(default)]
    pub lever_cutoff: f64,
    /// N2 at which fuel goes in.
    #[serde(default = "fuel_on_n2_default")]
    pub fuel_on_n2: f64,
    /// N2 at which the engine is stable at idle (with the starter released).
    #[serde(default = "stable_n2_default")]
    pub stable_n2: f64,
    /// The EGT start limit, deg C: past it the lever goes to CUTOFF.
    #[serde(default = "egt_start_limit_default")]
    pub egt_start_limit_c: f64,
    /// How long the starter may crank to `fuel_on_n2`, and how long from
    /// fuel to stable, seconds.
    #[serde(default = "crank_secs_default")]
    pub crank_secs: f64,
    #[serde(default = "light_secs_default")]
    pub light_secs: f64,
    /// Before the first starter (packs off, beacon) and after the last
    /// engine is stable: read before fire, read back, at a crew's pace.
    #[serde(default)]
    pub pre_start_steps: Vec<StartStep>,
    #[serde(default)]
    pub post_start_steps: Vec<StartStep>,
    /// The APU from cold, run only when `apu_n1` reads under `apu_ready_n1`
    /// (battery, fuel pump, the switch to ON, START held: the Zibo's START
    /// is spring-loaded back to ON, so its step carries no position).
    /// Empty: the APU is the pilot's, as before 2026-10-06.
    #[serde(default)]
    pub apu_steps: Vec<StartStep>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub apu_n1: Option<String>,
    #[serde(default = "apu_ready_n1_default")]
    pub apu_ready_n1: f64,
    /// The lamp that says the APU will take the bus (the 737's blue APU GEN
    /// OFF BUS), waited on, lit, before the generators are pressed. `None` =
    /// a fixed pause after `apu_ready_n1`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub apu_gen_ready: Option<String>,
    /// Once the APU runs, every start: its generators and its bleed, read
    /// before fire (the Zibo's generator switches need a HELD press).
    #[serde(default)]
    pub apu_power_steps: Vec<StartStep>,
}

pub(super) fn apu_ready_n1_default() -> f64 {
    95.0
}

/// One engine's switches and gauges in a [`BoeingStartProfile`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoeingEngineStart {
    /// The engine index (0 = number one), for the spoken name.
    pub engine: usize,
    pub starter_grd_command: String,
    pub starter_off_command: String,
    pub starter_position: String,
    pub lever_idle_command: String,
    pub lever_cutoff_command: String,
    pub lever_position: String,
    pub n2: String,
    pub egt: String,
}

pub(super) fn min_duct_psi_default() -> f64 {
    25.0
}
pub(super) fn fuel_on_n2_default() -> f64 {
    25.0
}
pub(super) fn stable_n2_default() -> f64 {
    55.0
}
pub(super) fn egt_start_limit_default() -> f64 {
    725.0
}
pub(super) fn crank_secs_default() -> f64 {
    60.0
}
pub(super) fn light_secs_default() -> f64 {
    90.0
}
