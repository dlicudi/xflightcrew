//! Anti-ice and de-ice (issue #100): `set_anti_ice`, and the crew's
//! icing-conditions practice the monitor runs.
//!
//! Which systems an airframe has is declared in its profile (`anti_ice:`),
//! else read from the `.acf`'s electrical loads (`systems::IceKit`): pitot,
//! window, engine inlet and prop heat by their stock switches. A wing is four
//! different machines (boots, bleed, electric heat, TKS fluid) and the `.acf`
//! only says THAT it is protected, so an undeclared wing is left alone and
//! said. The C90B and the DC-3 had no de-ice tool at all (2026-07-01).
//!
//! Every switch is read FIRST (a system in state is confirmed, never
//! re-written), switched by its `on` command (not a toggle), read back
//! against the intent, and a switch that did not take is named. A failed
//! read is no measurement.
//!
//! The practice, a crew's (FCOM wording, every type the same shape): engine
//! anti-ice, probes and windows ON in icing conditions (visible moisture at
//! +10 C or colder), wing anti-ice and boots once ice is ACCRETING. With an
//! automation flying the crew switches them (`monitor` `icing`); with the
//! pilot flying it advises once. Clear of icing for five minutes, the crew
//! switches off only what it switched on itself (the cockpit_auto rule).

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::{AntiIceProfile, IceKind};

/// The monitor's fix sentinel: its `fix_write` names this and the loop runs
/// [`monitor_fix`] (a set of switches, not one dataref).
pub const MONITOR_FIX: &str = "anti_ice";

/// The stock `sim/ice/*` switch for a kind: on commands, off commands, the
/// dataref that reads it back, and whether that dataref is per engine.
pub fn stock(
    kind: IceKind,
) -> (
    &'static [&'static str],
    &'static [&'static str],
    &'static str,
    bool,
) {
    match kind {
        IceKind::Pitot => (
            &["sim/ice/pitot_heat0_on", "sim/ice/pitot_heat1_on"],
            &["sim/ice/pitot_heat0_off", "sim/ice/pitot_heat1_off"],
            "sim/cockpit2/ice/ice_pitot_heat_on_pilot",
            false,
        ),
        IceKind::Window => (
            &["sim/ice/window_heat_on"],
            &["sim/ice/window_heat_off"],
            "sim/cockpit2/ice/ice_window_heat_on",
            false,
        ),
        IceKind::Engine => (
            &["sim/ice/inlet_heat_on"],
            &["sim/ice/inlet_heat_off"],
            "sim/cockpit2/ice/ice_inlet_heat_on_per_engine",
            true,
        ),
        IceKind::Prop => (
            &["sim/ice/prop_heat_on"],
            &["sim/ice/prop_heat_off"],
            "sim/cockpit2/ice/ice_prop_heat_on_per_engine",
            true,
        ),
        IceKind::WingBoots => (
            &["sim/ice/wing_boot_on"],
            &["sim/ice/wing_boot_off"],
            "sim/cockpit2/ice/ice_surface_boot_on",
            false,
        ),
        IceKind::WingBleed => (
            &["sim/ice/wing_tai_on"],
            &["sim/ice/wing_tai_off"],
            "sim/cockpit2/ice/ice_surface_hot_bleed_air_on",
            false,
        ),
        IceKind::WingHeat => (
            &["sim/ice/wing_heat_on"],
            &["sim/ice/wing_heat_off"],
            "sim/cockpit2/ice/ice_surfce_heat_on",
            false,
        ),
        IceKind::WingTks => (
            &["sim/ice/tks_norm"],
            &["sim/ice/tks_off"],
            "sim/cockpit2/ice/ice_surface_tks_on",
            false,
        ),
    }
}

/// What a crew calls it.
pub fn spoken(kind: IceKind) -> &'static str {
    match kind {
        IceKind::Pitot => "pitot heat",
        IceKind::Window => "window heat",
        IceKind::Engine => "engine anti-ice",
        IceKind::Prop => "prop heat",
        IceKind::WingBoots => "the boots",
        IceKind::WingBleed | IceKind::WingHeat => "wing anti-ice",
        IceKind::WingTks => "the TKS",
    }
}

fn is_wing(kind: IceKind) -> bool {
    matches!(
        kind,
        IceKind::WingBoots | IceKind::WingBleed | IceKind::WingHeat | IceKind::WingTks
    )
}

/// One system, its switch resolved.
#[derive(Debug, Clone, PartialEq)]
pub struct Resolved {
    pub kind: IceKind,
    pub on: Vec<String>,
    pub off: Vec<String>,
    /// A dataref written 1/0 in place of the commands (the MD-82's heat
    /// knob, the Citation's pitot switch: `pitot_heat_dataref`).
    pub write: Option<String>,
    pub readback: String,
    /// Elements of a per-engine readback to check (0 = a scalar or an
    /// indexed `name[i]`).
    pub engines: usize,
}

/// The airframe's systems: the profile's declaration when it has one, else
/// what the `.acf` says is fitted, by the stock switches. `pitot_heat` and
/// `pitot_override` are the profile's existing pitot fields. Pure.
pub fn resolve(
    declared: Option<&AntiIceProfile>,
    kit: &crate::systems::IceKit,
    engines: usize,
    pitot_heat: bool,
    pitot_override: Option<&str>,
) -> Vec<Resolved> {
    let engines = engines.max(1);
    let build = |kind: IceKind, on: &[String], off: &[String], readback: Option<&str>| {
        let (s_on, s_off, s_read, per_engine) = stock(kind);
        let pick = |given: &[String], fallback: &[&str]| -> Vec<String> {
            if given.is_empty() {
                fallback.iter().map(|s| s.to_string()).collect()
            } else {
                given.to_vec()
            }
        };
        let readback = readback.unwrap_or(s_read).to_string();
        let indexed = readback.contains('[');
        Resolved {
            kind,
            on: pick(on, s_on),
            off: pick(off, s_off),
            write: (kind == IceKind::Pitot && on.is_empty())
                .then(|| pitot_override.map(str::to_string))
                .flatten(),
            readback,
            engines: if per_engine && !indexed { engines } else { 0 },
        }
    };
    if let Some(d) = declared {
        return d
            .systems
            .iter()
            .map(|s| {
                build(
                    s.kind,
                    &s.on_commands,
                    &s.off_commands,
                    s.readback.as_deref(),
                )
            })
            .collect();
    }
    let mut out = Vec::new();
    if pitot_heat {
        out.push(build(IceKind::Pitot, &[], &[], None));
    }
    if kit.window {
        out.push(build(IceKind::Window, &[], &[], None));
    }
    if kit.engine_inlet {
        out.push(build(IceKind::Engine, &[], &[], None));
    }
    if kit.prop {
        out.push(build(IceKind::Prop, &[], &[], None));
    }
    out
}

/// The systems of the airframe loaded (live profile and systems survey).
pub fn resolved_live() -> Vec<Resolved> {
    let Some(p) = crate::aircraft::live_profile() else {
        return Vec::new();
    };
    resolve(
        p.anti_ice.as_ref(),
        &crate::systems::current().ice,
        p.engines,
        p.pitot_heat,
        p.pitot_heat_dataref.as_deref(),
    )
}

/// Read one system's switch: on when every element read is at or above 0.5.
/// `None` = the read failed (no measurement).
pub async fn read_state(xp: &dyn crate::xplane::XPlane, r: &Resolved) -> Option<bool> {
    if let Some((name, idx)) = r.readback.split_once('[') {
        let i: usize = idx.trim_end_matches(']').parse().ok()?;
        let v = xp.get_float_array(name).await.ok()?;
        return v.get(i).map(|x| *x >= 0.5);
    }
    if r.engines > 0 {
        let v = xp.get_float_array(&r.readback).await.ok()?;
        if v.len() < r.engines {
            return None;
        }
        return Some(v[..r.engines].iter().all(|x| *x >= 0.5));
    }
    xp.get_float(&r.readback).await.ok().map(|x| x >= 0.5)
}

/// What one switch did.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Outcome {
    pub kind: IceKind,
    pub want: bool,
    pub before: Option<bool>,
    pub after: Option<bool>,
}

impl Outcome {
    pub fn took(&self) -> bool {
        self.after == Some(self.want)
    }
    /// It was already there and nothing was touched.
    pub fn was_set(&self) -> bool {
        self.before == Some(self.want)
    }
}

/// How long a switch's own script takes to show it (the stock ice datarefs
/// update a frame behind a command; an add-on's mirror a frame behind that).
const SETTLE_MS: u64 = 300;

/// Switch `systems` to `on`, read first and back after. Blackboxed
/// `anti_ice` `switched` with `source`.
pub async fn apply(
    xp: &dyn crate::xplane::XPlane,
    systems: &[Resolved],
    on: bool,
    source: &str,
) -> Vec<Outcome> {
    let mut out = Vec::new();
    for r in systems {
        let before = read_state(xp, r).await;
        if before == Some(on) {
            out.push(Outcome {
                kind: r.kind,
                want: on,
                before,
                after: before,
            });
            continue;
        }
        match &r.write {
            Some(dref) => {
                let _ = xp.set_float(dref, if on { 1.0 } else { 0.0 }).await;
            }
            None => {
                for c in if on { &r.on } else { &r.off } {
                    let _ = xp.run_command(c).await;
                }
            }
        }
        tokio::time::sleep(std::time::Duration::from_millis(SETTLE_MS)).await;
        let after = read_state(xp, r).await;
        let o = Outcome {
            kind: r.kind,
            want: on,
            before,
            after,
        };
        crate::blackbox::record(
            "anti_ice",
            json!({
                "event": "switched", "source": source, "kind": r.kind,
                "want": on, "before": before, "after": after, "took": o.took(),
            }),
        );
        out.push(o);
    }
    out
}

fn join_and(items: &[&str]) -> String {
    match items {
        [] => String::new(),
        [a] => a.to_string(),
        [rest @ .., last] => format!("{} and {last}", rest.join(", ")),
    }
}

fn dedup<'a>(mut v: Vec<&'a str>) -> Vec<&'a str> {
    let mut seen = Vec::new();
    v.retain(|s| {
        if seen.contains(s) {
            false
        } else {
            seen.push(*s);
            true
        }
    });
    v
}

/// The readback line: what is now on (or off), and every switch that did
/// not take by name. Pure.
pub fn outcome_line(outcomes: &[Outcome], on: bool) -> String {
    let state = if on { "on" } else { "off" };
    let took: Vec<&str> = dedup(
        outcomes
            .iter()
            .filter(|o| o.took())
            .map(|o| spoken(o.kind))
            .collect(),
    );
    let failed: Vec<&str> = dedup(
        outcomes
            .iter()
            .filter(|o| !o.took() && o.after.is_some())
            .map(|o| spoken(o.kind))
            .collect(),
    );
    let unread: Vec<&str> = dedup(
        outcomes
            .iter()
            .filter(|o| o.after.is_none())
            .map(|o| spoken(o.kind))
            .collect(),
    );
    let mut parts = Vec::new();
    if !took.is_empty() {
        let mut s = join_and(&took);
        if let Some(c) = s.get(..1) {
            s = format!("{}{}", c.to_uppercase(), &s[1..]);
        }
        parts.push(format!("{s} {state}."));
    }
    if !failed.is_empty() {
        parts.push(format!(
            "{} didn't take, {} still {}.",
            capitalise(&join_and(&failed)),
            if failed.len() == 1 { "it's" } else { "they're" },
            if on { "off" } else { "on" }
        ));
    }
    if !unread.is_empty() {
        parts.push(format!("I can't read {}.", join_and(&unread)));
    }
    parts.join(" ")
}

fn capitalise(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// Which of `systems` a spoken group names.
pub fn select<'a>(systems: &'a [Resolved], group: &str) -> Vec<&'a Resolved> {
    systems
        .iter()
        .filter(|r| match group {
            "all" => true,
            "pitot" => r.kind == IceKind::Pitot,
            "window" => r.kind == IceKind::Window,
            "engine" => r.kind == IceKind::Engine,
            "prop" => r.kind == IceKind::Prop,
            "wing" => is_wing(r.kind),
            _ => false,
        })
        .collect()
}

pub struct SetAntiIce;

#[derive(Deserialize)]
struct Args {
    /// all | pitot | window | engine | prop | wing
    #[serde(default = "all")]
    system: String,
    on: bool,
}

fn all() -> String {
    "all".to_string()
}

#[async_trait]
impl Tool for SetAntiIce {
    fn name(&self) -> &str {
        "set_anti_ice"
    }

    fn description(&self) -> &str {
        "Anti-ice and de-ice systems on or off: 'anti-ice on', 'engine anti-ice on', \
         'wing anti-ice off', 'window heat on', 'prop heat on', 'boots on', 'ice \
         protection on', 'de-ice off'. The airframe's own systems, declared or read \
         from the .acf; each is read back."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "system": {
                    "type": "string",
                    "enum": ["all", "pitot", "window", "engine", "prop", "wing"],
                    "description": "which system; all = every one fitted"
                },
                "on": {"type": "boolean"}
            },
            "required": ["on"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: Args = serde_json::from_value(args.clone())?;
        let systems = resolved_live();
        Ok(run(ctx.xplane, &systems, &a.system, a.on, wing_undeclared()).await)
    }
}

/// The `.acf` loads a wing system the profile does not say how to switch.
fn wing_undeclared() -> bool {
    let declared = crate::aircraft::live_profile().is_some_and(|p| p.anti_ice.is_some());
    !declared && crate::systems::current().ice.wing
}

/// The tool's body over resolved systems. Pure over the sim.
pub async fn run(
    xp: &dyn crate::xplane::XPlane,
    systems: &[Resolved],
    group: &str,
    on: bool,
    wing_undeclared: bool,
) -> String {
    let chosen: Vec<Resolved> = select(systems, group).into_iter().cloned().collect();
    if chosen.is_empty() {
        if group == "wing" && wing_undeclared {
            return "The wing's protection isn't declared for this airframe, so I've left it \
                    alone. It's on your overhead."
                .to_string();
        }
        return match group {
            "all" => "No anti-ice I can switch on this airframe.".to_string(),
            g => format!("There's no {} on this airframe.", spoken_group(g)),
        };
    }
    let outcomes = apply(xp, &chosen, on, "tool").await;
    if outcomes.iter().all(|o| o.was_set()) {
        return format!(
            "{} already {}.",
            capitalise(&join_and(&dedup(
                outcomes.iter().map(|o| spoken(o.kind)).collect()
            ))),
            if on { "on" } else { "off" }
        );
    }
    let mut line = outcome_line(&outcomes, on);
    if group == "all" && on && wing_undeclared {
        line.push_str(" The wing's yours, it isn't declared for me.");
    }
    line
}

fn spoken_group(g: &str) -> &'static str {
    match g {
        "pitot" => "pitot heat",
        "window" => "window heat",
        "engine" => "engine anti-ice",
        "prop" => "prop heat",
        "wing" => "wing ice protection",
        _ => "anti-ice",
    }
}

/// The ice picture, read on the monitor's slow clock in cold air.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct IcingRead {
    pub oat_c: f64,
    pub dewpoint_c: f64,
    /// Ratios of ice (0..1): airframe (worse wing), pitot, engine inlets
    /// (worst), props (worst), windshield, static port.
    pub airframe: f64,
    pub pitot: f64,
    pub inlet: f64,
    pub prop: f64,
    pub window: f64,
    pub static_port: f64,
    /// Each resolved system and its switch (`None` = unread).
    pub switches: Vec<(IceKind, Option<bool>)>,
}

/// Visible moisture at +10 C or colder: the book's icing conditions. The
/// sim gives no cloud flag at the aircraft, so moisture is a spread of 3 C
/// or less between the air and its dewpoint. Pure.
pub fn icing_conditions(r: &IcingRead) -> bool {
    (-40.0..=10.0).contains(&r.oat_c) && (r.oat_c - r.dewpoint_c) <= 3.0
}

/// Ice is FORMING somewhere the crew can see or the sim measures.
pub const ACCRETING: f64 = 0.02;

pub fn accreting(r: &IcingRead) -> bool {
    [r.airframe, r.inlet, r.prop, r.window, r.pitot]
        .iter()
        .any(|x| *x > ACCRETING)
}

/// The systems the practice wants ON now that read off. Probes, windows,
/// engines and props in icing conditions; wing systems once ice accretes.
/// Pure.
pub fn practice_wants(r: &IcingRead) -> Vec<IceKind> {
    let cond = icing_conditions(r) || accreting(r);
    if !cond {
        return Vec::new();
    }
    let ice = accreting(r);
    r.switches
        .iter()
        .filter(|(k, state)| {
            *state == Some(false)
                && match k {
                    IceKind::Pitot | IceKind::Window | IceKind::Engine | IceKind::Prop => true,
                    _ => ice,
                }
        })
        .map(|(k, _)| *k)
        .collect()
}

/// The practice's line: what to switch (or what was switched), the
/// accreting case a notch more urgent. Pure.
pub fn practice_line(r: &IcingRead, kinds: &[IceKind], acting: bool) -> String {
    let names = join_and(&dedup(kinds.iter().map(|k| spoken(*k)).collect()));
    let lead = if accreting(r) {
        "We're picking up ice"
    } else {
        "Icing conditions"
    };
    if acting {
        format!("{lead}, {names} on.")
    } else {
        format!("{lead}, recommend {names} on.")
    }
}

/// What the crew switched on by itself, so it switches off only that.
static CLAIMED: std::sync::Mutex<Vec<IceKind>> = std::sync::Mutex::new(Vec::new());

pub fn claimed() -> Vec<IceKind> {
    CLAIMED.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// A new airframe: nothing on it was switched by us.
pub fn clear_claims() {
    CLAIMED.lock().unwrap_or_else(|e| e.into_inner()).clear();
}

/// The monitor's hands: switch `kinds` of the live airframe to `on`, claim
/// what the crew turned on, release what it turned off. Returns a line for
/// a switch that did not take.
pub async fn monitor_fix(
    xp: &dyn crate::xplane::XPlane,
    kinds: &[IceKind],
    on: bool,
) -> Option<String> {
    let systems: Vec<Resolved> = resolved_live()
        .into_iter()
        .filter(|r| kinds.contains(&r.kind))
        .collect();
    let outcomes = apply(xp, &systems, on, "monitor").await;
    {
        let mut c = CLAIMED.lock().unwrap_or_else(|e| e.into_inner());
        for o in outcomes.iter().filter(|o| o.took() && !o.was_set()) {
            if on {
                if !c.contains(&o.kind) {
                    c.push(o.kind);
                }
            } else {
                c.retain(|k| *k != o.kind);
            }
        }
    }
    let failed: Vec<&str> = outcomes
        .iter()
        .filter(|o| !o.took())
        .map(|o| spoken(o.kind))
        .collect();
    (!failed.is_empty()).then(|| {
        format!(
            "{} didn't take. Check it on the overhead.",
            capitalise(&join_and(&dedup(failed)))
        )
    })
}

/// Read the ice picture for the live airframe's systems.
pub async fn read_icing(
    xp: &dyn crate::xplane::XPlane,
    oat_c: f64,
    dewpoint_c: f64,
    systems: &[Resolved],
) -> IcingRead {
    let one = |v: Result<f64>| v.unwrap_or(0.0);
    let worst = |v: Result<Vec<f64>>| v.ok().map_or(0.0, |a| a.into_iter().fold(0.0, f64::max));
    let (l, r, pitot, inlet, prop, window, stat) = tokio::join!(
        xp.get_float("sim/flightmodel/failures/frm_ice"),
        xp.get_float("sim/flightmodel/failures/frm_ice2"),
        xp.get_float("sim/flightmodel/failures/pitot_ice"),
        xp.get_float_array("sim/flightmodel/failures/inlet_ice_per_engine"),
        xp.get_float_array("sim/flightmodel/failures/prop_ice_per_engine"),
        xp.get_float("sim/flightmodel/failures/window_ice"),
        xp.get_float("sim/flightmodel/failures/stat_ice"),
    );
    let mut switches = Vec::new();
    for s in systems {
        switches.push((s.kind, read_state(xp, s).await));
    }
    IcingRead {
        oat_c,
        dewpoint_c,
        airframe: one(l).max(one(r)),
        pitot: one(pitot),
        inlet: worst(inlet),
        prop: worst(prop),
        window: one(window),
        static_port: one(stat),
        switches,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::systems::IceKit;
    use crate::xplane::FakeXPlane;

    /// The C90B and the DC-3 had no de-ice tool (2026-07-01). A declared
    /// profile is used as declared; an undeclared airframe gets what its
    /// `.acf` loads, by the stock switches, and never a guessed wing.
    #[test]
    fn systems_come_from_the_profile_else_the_acf() {
        let c90 = crate::aircraft::load("king_air_c90b").unwrap();
        let r = resolve(c90.anti_ice.as_ref(), &IceKit::default(), 2, true, None);
        let kinds: Vec<IceKind> = r.iter().map(|s| s.kind).collect();
        assert!(kinds.contains(&IceKind::WingBoots), "{kinds:?}");
        assert!(kinds.contains(&IceKind::Engine), "{kinds:?}");
        let kit = IceKit {
            pitot: true,
            window: true,
            engine_inlet: true,
            prop: false,
            wing: true,
        };
        let generic = resolve(None, &kit, 2, true, None);
        let kinds: Vec<IceKind> = generic.iter().map(|s| s.kind).collect();
        assert_eq!(
            kinds,
            vec![IceKind::Pitot, IceKind::Window, IceKind::Engine]
        );
        let eng = &generic[2];
        assert_eq!(eng.on, vec!["sim/ice/inlet_heat_on".to_string()]);
        assert_eq!(eng.engines, 2, "every engine read back");
        // The MD-82's pitot knob is a dataref write.
        let md = resolve(
            None,
            &IceKit::default(),
            2,
            true,
            Some("laminar/md82/ice/heatknob"),
        );
        assert_eq!(md[0].write.as_deref(), Some("laminar/md82/ice/heatknob"));
    }

    fn engines_on(fake: &FakeXPlane) {
        fake.set_read_array("sim/cockpit2/ice/ice_inlet_heat_on_per_engine", &[0.0, 0.0]);
        fake.apply_array_writes("sim/cockpit2/ice/ice_inlet_heat_on_per_engine");
    }

    /// A switch in state is confirmed, never re-written; one that did not
    /// take is named with what it reads.
    #[tokio::test]
    async fn the_tool_reads_first_and_names_a_switch_that_did_not_take() {
        let kit = IceKit {
            pitot: true,
            window: true,
            engine_inlet: true,
            ..IceKit::default()
        };
        let systems = resolve(None, &kit, 2, true, None);
        let fake = FakeXPlane::with_reads(&[
            ("sim/cockpit2/ice/ice_pitot_heat_on_pilot", 1.0),
            ("sim/cockpit2/ice/ice_window_heat_on", 0.0),
        ]);
        engines_on(&fake);
        fake.set_on_command(
            "sim/ice/window_heat_on",
            "sim/cockpit2/ice/ice_window_heat_on",
            1.0,
        );
        let line = run(&fake, &systems, "all", true, false).await;
        assert!(!fake.ran("sim/ice/pitot_heat0_on"), "pitot already on");
        assert!(fake.ran("sim/ice/window_heat_on"));
        assert!(fake.ran("sim/ice/inlet_heat_on"));
        assert!(
            line.contains("Window heat on") || line.contains("window heat"),
            "{line}"
        );
        assert!(
            line.contains("Engine anti-ice didn't take, it's still off"),
            "{line}"
        );
        // Everything already on: said, nothing pressed.
        let fake = FakeXPlane::with_reads(&[("sim/cockpit2/ice/ice_window_heat_on", 1.0)]);
        let line = run(&fake, &systems, "window", true, false).await;
        assert_eq!(line, "Window heat already on.");
        assert!(!fake.ran("sim/ice/window_heat_on"));
    }

    #[tokio::test]
    async fn an_undeclared_wing_is_left_alone_and_said() {
        let kit = IceKit {
            pitot: true,
            wing: true,
            ..IceKit::default()
        };
        let systems = resolve(None, &kit, 1, true, None);
        let fake = FakeXPlane::new();
        let line = run(&fake, &systems, "wing", true, true).await;
        assert!(line.contains("isn't declared"), "{line}");
        assert!(!fake.ran("sim/ice/wing_heat_on") && !fake.ran("sim/ice/wing_boot_on"));
        let line = run(&fake, &systems, "prop", true, false).await;
        assert_eq!(line, "There's no prop heat on this airframe.");
    }

    /// The practice: engines and probes in icing conditions, the wing once
    /// ice forms; dry or warm air wants nothing.
    #[test]
    fn the_practice_is_engines_in_conditions_and_the_wing_on_ice() {
        let base = IcingRead {
            oat_c: -5.0,
            dewpoint_c: -6.0,
            switches: vec![
                (IceKind::Pitot, Some(true)),
                (IceKind::Engine, Some(false)),
                (IceKind::WingBoots, Some(false)),
                (IceKind::Window, None),
            ],
            ..IcingRead::default()
        };
        assert_eq!(practice_wants(&base), vec![IceKind::Engine]);
        assert_eq!(
            practice_line(&base, &[IceKind::Engine], true),
            "Icing conditions, engine anti-ice on."
        );
        let ice = IcingRead {
            airframe: 0.08,
            ..base.clone()
        };
        assert_eq!(
            practice_wants(&ice),
            vec![IceKind::Engine, IceKind::WingBoots]
        );
        assert!(practice_line(&ice, &[IceKind::WingBoots], false)
            .starts_with("We're picking up ice, recommend the boots on"));
        let dry = IcingRead {
            dewpoint_c: -20.0,
            ..base.clone()
        };
        assert!(practice_wants(&dry).is_empty());
        let warm = IcingRead {
            oat_c: 15.0,
            dewpoint_c: 14.0,
            ..base
        };
        assert!(practice_wants(&warm).is_empty());
    }

    #[test]
    fn every_stock_switch_is_a_sim_name() {
        // The install check scans literals; this pins that each kind has a
        // readback in the ice namespace and matching on/off commands.
        for k in [
            IceKind::Pitot,
            IceKind::Window,
            IceKind::Engine,
            IceKind::Prop,
            IceKind::WingBoots,
            IceKind::WingBleed,
            IceKind::WingHeat,
            IceKind::WingTks,
        ] {
            let (on, off, read, _) = stock(k);
            assert_eq!(on.len(), off.len(), "{k:?}");
            assert!(read.starts_with("sim/cockpit2/ice/"), "{k:?}");
            assert!(on.iter().all(|c| c.starts_with("sim/ice/")));
        }
    }
}
