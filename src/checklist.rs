//! The airframe's checklists, native: `checklists/<profile id>.yaml`.
//!
//! One file per airframe, found by the profile's id. Sections are keyed by
//! name (`before_takeoff`: the phase keys the tracker fires are fixed names)
//! and titled by the key's words ("Before takeoff"); items run in file order:
//!
//! ```yaml
//! before_takeoff:
//!   - { say: "Parking brake, set.", check: park_brake, is: on }
//!   - { say: "Speed brake, down.", check: speedbrake, is: down }
//!   - { say: "Fuel quantity, sufficient." }
//!   - { note: "Run-up." }
//! ```
//!
//! An item with `check` names a STATE from a closed vocabulary ([`STATES`]),
//! never a dataref; without one it is a call-out, the pilot's. `note` is
//! narration. An ACTABLE state is a curated tool's: the crew reads it first,
//! acts through the tool (its gates and readback included), and `act: false`
//! keeps the item a confirm on an airframe where only the read is honest. A
//! READ-ONLY state is only ever read. Nothing writes a dataref raw.
//!
//! Each state's read has ONE home: the owning tool's own readback (a light's
//! switch, an electrical system's readback, the flap lever, the transponder
//! knob, the seat belt switch), a stock dataref in `datarefs::*`, or the
//! airframe's `checklist_states:` for an add-on's own switch. An unknown state
//! or value fails the LOAD, file and item named, never in flight.

use std::sync::Arc;

use anyhow::{bail, Context, Result};
use serde::Deserialize;
use serde_json::{json, Value};

use crate::aircraft::{AircraftProfile, StateValue};
use crate::tools::autopilot::AutopilotOff;
use crate::tools::controls::{SetParkingBrake, SetPitchTrim};
use crate::tools::electrical::SetElectrical;
use crate::tools::engine::{
    SetFuelPump, SetFuelSelector, SetMagnetos, SetMixture, SetThrottle, StartEngine, StopEngine,
};
use crate::tools::flaps::SetFlaps;
use crate::tools::instruments::SetPitotHeat;
use crate::tools::lights::SetLights;
use crate::tools::radio::SetTransponderMode;
use crate::tools::{Tool, ToolCtx};
use crate::xplane::XPlaneClient;

/// Every state and its values. `lights.<key>` and `electrical.<key>` name
/// the profile's own light or system; `flaps` also takes a detent's label,
/// `fuel_selector` a position's key.
pub const STATES: &[(&str, &[&str])] = &[
    // Actable: a curated tool sets them.
    ("park_brake", &["on", "off"]),
    ("flaps", &["up", "takeoff", "landing", "full"]),
    ("lights", &["on", "off"]),
    ("electrical", &["on", "off"]),
    ("transponder", &["off", "standby", "altitude"]),
    ("mixture", &["rich", "lean", "cutoff"]),
    ("throttle", &["idle", "start", "runup", "cruise", "full"]),
    ("magnetos", &["off", "right", "left", "both"]),
    ("engine", &["running", "stopped"]),
    ("fuel_pump", &["on", "off"]),
    ("fuel_selector", &[]),
    ("pitot_heat", &["on", "off"]),
    ("autopilot", &["off", "on"]),
    ("pitch_trim", &["takeoff"]),
    // Read-only: confirmed, never written.
    ("apu", &["on", "off"]),
    ("apu_bleed", &["on", "off"]),
    ("apu_generator", &["on", "off"]),
    ("packs", &["on", "off"]),
    ("engine_bleeds", &["on", "off"]),
    ("fuel_pumps", &["on", "off"]),
    ("gear", &["up", "down"]),
    ("speedbrake", &["down", "armed"]),
    ("seat_belts", &["on", "off"]),
    ("flight_director", &["on", "off"]),
    ("autothrottle", &["armed", "off", "on"]),
    ("yaw_damper", &["on", "off"]),
    ("ground_power", &["on", "off"]),
    ("doors", &["open", "closed"]),
    ("hydraulic_pumps", &["on", "off"]),
    ("start_levers", &["idle", "cutoff"]),
    ("altimeter", &["standard"]),
    ("autobrake", &["rto", "off", "on"]),
    ("bus_tie", &["on", "off"]),
    ("window_heat", &["on", "off"]),
    ("oxygen", &["on", "off"]),
    ("master_caution", &["clear"]),
    ("master_warning", &["clear"]),
    ("weather_radar", &["on", "off"]),
    ("speed_bug", &["v2"]),
    ("n2", &["turning", "stable", "stopped"]),
    ("start_switch", &["ground", "normal", "continuous"]),
    (
        "controls",
        &[
            "roll_left",
            "roll_right",
            "pitch_forward",
            "pitch_aft",
            "yaw_left",
            "yaw_right",
        ],
    ),
    ("lamp_test", &["pressed"]),
    ("cargo_doors", &["open", "closed"]),
];

/// A flap handle at or past this is a landing flap: the fleet's shallowest
/// (the MD-82's 28, the A330's 3).
const LANDING_FLAP_MIN: f64 = 0.7;
/// A flap handle within this of a detent is at it.
const FLAP_TOL: f64 = 0.05;

/// One read a state is judged by: the dataref (`[index]` split off) within
/// `lo..=hi`.
#[derive(Debug, Clone, PartialEq)]
pub struct Term {
    pub dataref: String,
    pub index: Option<usize>,
    pub lo: f64,
    pub hi: f64,
}

impl Term {
    fn new(dataref: &str, lo: f64, hi: f64) -> Term {
        let (dataref, index) = split_index(dataref);
        Term {
            dataref,
            index,
            lo,
            hi,
        }
    }

    /// `name` or `name[i]` at a switch position (the XChecklist tolerance).
    fn at(dataref: &str, n: f64) -> Term {
        Term::new(dataref, n - 0.49, n + 0.49)
    }

    fn above(dataref: &str, lo: f64) -> Term {
        Term::new(dataref, lo, f64::INFINITY)
    }

    fn below(dataref: &str, hi: f64) -> Term {
        Term::new(dataref, f64::NEG_INFINITY, hi)
    }

    pub fn holds(&self, v: f64) -> bool {
        self.lo <= v && v <= self.hi
    }
}

/// A switch on (0.5 and up) or off (below it), by the value's sense.
fn switch(dataref: &str, value: &str) -> Term {
    match value {
        "on" | "open" | "down" | "running" => Term::above(dataref, 0.5),
        _ => Term::below(dataref, 0.49),
    }
}

/// The tool call that sets an actable state, and the hazard asked first.
#[derive(Debug, Clone, PartialEq)]
pub struct Action {
    pub tool: &'static str,
    pub args: Value,
    pub confirm: Option<&'static str>,
}

/// A checked item's state, its value, how it reads and (actable, and not
/// `act: false`) how it is set.
#[derive(Debug, Clone, PartialEq)]
pub struct Check {
    pub state: String,
    pub value: String,
    pub terms: Vec<Term>,
    pub action: Option<Action>,
}

/// One item of a list.
#[derive(Debug, Clone, PartialEq)]
pub struct Item {
    /// The spoken line.
    pub say: String,
    /// Narration only (`note:`).
    pub note: bool,
    pub check: Option<Check>,
}

impl Item {
    /// Does the item carry a read (a state)?
    pub fn has_test(&self) -> bool {
        self.check.is_some()
    }

    /// The state's reads, in order; empty for a call-out or a note.
    pub fn terms(&self) -> &[Term] {
        self.check.as_ref().map_or(&[], |c| c.terms.as_slice())
    }

    /// The tool call, for an item the crew may act on.
    pub fn action(&self) -> Option<&Action> {
        self.check.as_ref()?.action.as_ref()
    }

    /// The state checked ("flaps"), if any.
    pub fn state(&self) -> Option<&str> {
        self.check.as_ref().map(|c| c.state.as_str())
    }
}

/// What the runner read of one item: its verdict (`None` = no test, or a
/// term that did not read) and every term's value, in order.
#[derive(Debug, Clone, PartialEq)]
pub struct Reading {
    pub satisfied: Option<bool>,
    pub values: Vec<Option<f64>>,
}

/// Read every term of an item, in one join, and judge it.
pub async fn read_item(xp: &XPlaneClient, item: &Item) -> Reading {
    let values: Vec<Option<f64>> = futures::future::join_all(
        item.terms()
            .iter()
            .map(|t| read_dataref(xp, &t.dataref, t.index)),
    )
    .await;
    Reading {
        satisfied: judge_read(item, &values),
        values,
    }
}

/// The item's verdict from its terms' values, in [`Item::terms`] order:
/// every term holds. A term that did not read, or a state with nothing to
/// read, is no verdict. Pure.
pub fn judge_read(item: &Item, values: &[Option<f64>]) -> Option<bool> {
    let terms = item.terms();
    if terms.is_empty() || values.len() < terms.len() {
        return None;
    }
    let values: Vec<f64> = values[..terms.len()]
        .iter()
        .copied()
        .collect::<Option<_>>()?;
    Some(terms.iter().zip(values).all(|(t, v)| t.holds(v)))
}

/// One airframe list. Carries the profile its actions are built from.
pub struct Checklist {
    /// The section key ("before_takeoff").
    pub id: String,
    pub title: String,
    pub items: Vec<Item>,
    pub profile: Arc<AircraftProfile>,
}

/// The airframe's lists: `checklists/<id>.yaml`, a shipped file
/// (`sim::paths::checklist_file`). `Ok(vec![])` when the airframe has none.
pub fn load(profile: &AircraftProfile) -> Result<Vec<Checklist>> {
    let path = sim::paths::checklist_file(&profile.id);
    if !path.exists() {
        return Ok(Vec::new());
    }
    let file = path.display().to_string();
    let text = std::fs::read_to_string(&path).with_context(|| format!("reading {file}"))?;
    parse(&text, profile).with_context(|| file)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawItem {
    say: Option<String>,
    note: Option<String>,
    check: Option<String>,
    is: Option<serde_yaml::Value>,
    act: Option<bool>,
}

/// Parse a native file against its airframe. Strict: an unknown state or
/// value, or a malformed item, is an error naming the section and item.
pub fn parse(text: &str, profile: &AircraftProfile) -> Result<Vec<Checklist>> {
    let doc: serde_yaml::Mapping = serde_yaml::from_str(text)?;
    let shared = Arc::new(profile.clone());
    let mut out = Vec::new();
    for (key, section) in doc {
        let Some(id) = key.as_str().map(str::to_string) else {
            bail!("a section key is not a name: {key:?}");
        };
        // A list of items, or `{ title, items }` where the airframe's own
        // name for the section differs from its key.
        let (title, items) = match section {
            serde_yaml::Value::Mapping(mut m) => {
                let title = m
                    .remove("title")
                    .and_then(|t| t.as_str().map(str::to_string));
                let items = m.remove("items").unwrap_or_default();
                if title.is_none() || !m.is_empty() {
                    bail!("section {id}: a titled section is `{{ title, items }}`");
                }
                (title, items)
            }
            items => (None, items),
        };
        let raw: Vec<RawItem> =
            serde_yaml::from_value(items).with_context(|| format!("section {id}"))?;
        let items = raw
            .into_iter()
            .enumerate()
            .map(|(i, r)| item(r, profile).with_context(|| format!("{id} item {}", i + 1)))
            .collect::<Result<Vec<_>>>()?;
        out.push(Checklist {
            title: title.unwrap_or_else(|| title_of(&id)),
            id,
            items,
            profile: shared.clone(),
        });
    }
    Ok(out)
}

fn item(r: RawItem, p: &AircraftProfile) -> Result<Item> {
    if let Some(note) = r.note {
        if r.say.is_some() || r.check.is_some() {
            bail!("a note is narration only");
        }
        return Ok(Item {
            say: note,
            note: true,
            check: None,
        });
    }
    let Some(say) = r.say else {
        bail!("an item needs `say` or `note`");
    };
    let check = match (r.check, r.is) {
        (None, None) => None,
        (Some(state), Some(is)) => {
            let value = match is {
                serde_yaml::Value::String(s) => s,
                serde_yaml::Value::Number(n) => n.to_string(),
                other => bail!("`is` must be a word: {other:?}"),
            };
            Some(resolve(&state, &value, r.act.unwrap_or(true), p)?)
        }
        _ => bail!("`check` and `is` go together"),
    };
    Ok(Item {
        say,
        note: false,
        check,
    })
}

/// "before_takeoff" -> "Before takeoff".
fn title_of(key: &str) -> String {
    cap_first(&key.replace('_', " "))
}

/// A state and value, resolved against the airframe: validated, its read,
/// and its action (`act` and actable). A read-only state may name one
/// engine, tank or door of its read (`n2.2`): without one it reads them all.
fn resolve(state: &str, value: &str, act: bool, p: &AircraftProfile) -> Result<Check> {
    let (base, sub) = match state.split_once('.') {
        Some((b, s)) => (b, Some(s)),
        None => (state, None),
    };
    let Some((_, values)) = STATES.iter().find(|(s, _)| *s == base) else {
        bail!("unknown state `{state}`");
    };
    let one: Option<usize> = sub.and_then(|s| s.parse().ok());
    let keyed = matches!(base, "lights" | "electrical");
    if keyed != (sub.is_some() && one.is_none())
        || one.is_some_and(|n| n == 0 || action(base, None, value, p).is_some())
    {
        bail!("unknown state `{state}`");
    }
    let known = values.contains(&value)
        || match base {
            "flaps" => p
                .flaps
                .as_ref()
                .is_some_and(|f| f.detents.iter().any(|d| d.label == value)),
            "fuel_selector" => p
                .fuel_selector
                .as_ref()
                .is_some_and(|f| f.positions.iter().any(|x| x.key == value)),
            _ => false,
        };
    if !known {
        bail!("`{state}` has no value `{value}`");
    }
    let read = if one.is_some() { base } else { state };
    let terms = match p.checklist_states.get(read) {
        Some(read) => {
            let Some(v) = read.values.get(value) else {
                bail!("the airframe's `{state}` read has no value `{value}`");
            };
            read.datarefs
                .iter()
                .map(|d| match *v {
                    StateValue::At(n) => Term::at(d, n),
                    StateValue::Range([lo, hi]) => Term::new(d, lo, hi),
                })
                .collect()
        }
        None => stock_read(base, sub, value, p)?,
    };
    let terms = match one {
        Some(n) => vec![nth(terms, n).with_context(|| format!("`{state}`: no {n} to read"))?],
        None => terms,
    };
    let action = if act {
        action(base, sub, value, p)
    } else {
        None
    };
    Ok(Check {
        state: state.to_string(),
        value: value.to_string(),
        terms,
        action,
    })
}

/// The `n`th (1-based) of a state's reads, or the `n`th element of its one
/// array read.
fn nth(mut terms: Vec<Term>, n: usize) -> Option<Term> {
    if terms.len() >= n {
        return Some(terms.swap_remove(n - 1));
    }
    match terms.as_mut_slice() {
        [t] if t.index.is_some() => {
            t.index = Some(n - 1);
            terms.pop()
        }
        _ => None,
    }
}

/// The read a state has without the airframe's own: its tool's readback, or
/// a stock dataref.
fn stock_read(
    base: &str,
    sub: Option<&str>,
    value: &str,
    p: &AircraftProfile,
) -> Result<Vec<Term>> {
    use crate::aircraft::TakeoffConfigProfile as Tc;
    use crate::autoflight::names::{AP_ON, AT_MODE, FD_MODE};
    use crate::datarefs as d;
    let one = |t: Term| Ok(vec![t]);
    // One read per engine of a per-engine array.
    let each = |name: &str, t: &dyn Fn(&str) -> Term| -> Vec<Term> {
        (0..p.engines.max(1))
            .map(|i| t(&format!("{name}[{i}]")))
            .collect()
    };
    match base {
        "park_brake" => one(match value {
            "on" => Term::above(crate::tools::PARK_BRAKE, 0.5),
            _ => Term::below(crate::tools::PARK_BRAKE, 0.1),
        }),
        "flaps" => {
            let Some(f) = &p.flaps else {
                bail!("no flaps on this airframe");
            };
            let lever = f
                .by_command
                .as_ref()
                .map_or(d::FLAP_RATIO, |c| c.readback.as_str());
            let near = |r: f64| Term::new(lever, r - FLAP_TOL, r + FLAP_TOL);
            let max = f.detents.iter().map(|d| d.ratio).fold(0.0, f64::max);
            one(match value {
                "up" => Term::below(lever, FLAP_TOL),
                "landing" => Term::above(lever, LANDING_FLAP_MIN),
                "full" => Term::above(lever, max - FLAP_TOL),
                "takeoff" => match f.takeoff_ratio {
                    Some(r) => near(r),
                    None => bail!("no takeoff flap on this airframe"),
                },
                label => near(
                    f.detents
                        .iter()
                        .find(|d| d.label == label)
                        .map_or(0.0, |d| d.ratio),
                ),
            })
        }
        "lights" => {
            let key = sub.unwrap_or_default();
            let Some(l) = p.lights.iter().find(|l| l.key == key) else {
                bail!("no light `{key}` on this airframe");
            };
            one(switch(&l.dataref, value))
        }
        "electrical" => {
            let key = sub.unwrap_or_default();
            let Some(e) = p.electrical.iter().find(|e| e.key == key) else {
                bail!("no electrical system `{key}` on this airframe");
            };
            Ok(e.readback.iter().map(|r| switch(r, value)).collect())
        }
        "transponder" => one(match &p.transponder_mode_knob {
            Some(k) => Term::at(
                &k.dataref,
                match value {
                    "off" => k.off,
                    "standby" => k.standby,
                    _ => k.altitude,
                } as f64,
            ),
            None => match value {
                "off" => Term::below(d::TRANSPONDER_MODE, 0.49),
                "standby" => Term::new(d::TRANSPONDER_MODE, 0.5, 2.49),
                _ => Term::above(d::TRANSPONDER_MODE, 2.5),
            },
        }),
        "mixture" => {
            let m = crate::tools::engine::MIXTURE;
            one(match value {
                "rich" => Term::above(m, 0.9),
                "cutoff" => Term::below(m, 0.05),
                _ => Term::new(m, 0.06, 0.89),
            })
        }
        "throttle" => {
            let t = p
                .throttle_dataref
                .as_deref()
                .unwrap_or(crate::aircraft::STOCK_THROTTLE);
            one(match value {
                "idle" => Term::below(t, 0.05),
                "start" => Term::new(t, 0.08, 0.2),
                "cruise" => Term::new(t, 0.75, 0.99),
                "runup" => Term::new(t, 0.5, 0.9),
                _ => Term::above(t, 0.9),
            })
        }
        "magnetos" => {
            use crate::tools::engine::{IGNITION_KEY, IGNITION_ON};
            let key = if p.magnetos_per_engine {
                IGNITION_ON
            } else {
                IGNITION_KEY
            };
            let n = ["off", "right", "left", "both"]
                .iter()
                .position(|v| *v == value)
                .unwrap_or(0);
            one(Term::at(&format!("{key}[0]"), n as f64))
        }
        "engine" => one(switch(&format!("{}[0]", d::ENGINE_RUNNING), value)),
        "fuel_pump" => one(switch(
            &format!("{}[0]", crate::tools::engine::FUEL_PUMP),
            value,
        )),
        "fuel_selector" => {
            let f = p.fuel_selector.as_ref();
            let read = f.and_then(|f| f.readback.as_deref());
            let at = f
                .and_then(|f| f.positions.iter().find(|x| x.key == value))
                .and_then(|x| x.value);
            Ok(read
                .zip(at)
                .map(|(r, n)| Term::at(r, n))
                .into_iter()
                .collect())
        }
        "pitot_heat" => one(switch(
            p.pitot_heat_dataref
                .as_deref()
                .unwrap_or(crate::tools::instruments::PITOT_HEAT),
            value,
        )),
        "autopilot" => one(switch(AP_ON, value)),
        "pitch_trim" => {
            use crate::tools::controls::{TRIM_TAKEOFF, TRIM_TAKEOFF_TOL};
            one(Term::new(
                d::ELEVATOR_TRIM,
                TRIM_TAKEOFF - TRIM_TAKEOFF_TOL,
                TRIM_TAKEOFF + TRIM_TAKEOFF_TOL,
            ))
        }
        "apu" => one(switch(d::APU_RUNNING, value)),
        "apu_bleed" => one(switch(d::APU_BLEED, value)),
        "apu_generator" => one(switch(d::APU_GENERATOR_ON, value)),
        "packs" => one(switch(d::PACK_LEFT, value)),
        "engine_bleeds" => Ok(each(d::ENGINE_BLEED_SOV, &|r| switch(r, value))),
        "fuel_pumps" => one(switch(&format!("{}[0]", d::FUEL_TANK_PUMP_ON), value)),
        "gear" => one(switch(d::GEAR_HANDLE_DOWN, value)),
        "speedbrake" => one(match value {
            "armed" => Term::below(Tc::SPEEDBRAKE, -0.25),
            _ => Term::new(Tc::SPEEDBRAKE, -0.24, 0.05),
        }),
        "seat_belts" => one(match &p.seat_belt_signs {
            Some(s) => Term::at(&s.position, if value == "on" { s.on } else { s.off } as f64),
            None => switch(crate::tools::signs::STOCK_SWITCH, value),
        }),
        "flight_director" => one(switch(FD_MODE, value)),
        "autothrottle" => one(match value {
            "off" => Term::below(AT_MODE, -0.5),
            "armed" => Term::new(AT_MODE, -0.49, 0.49),
            _ => Term::above(AT_MODE, 0.5),
        }),
        "yaw_damper" => one(switch(crate::autoland::YAW_DAMPER_ON, value)),
        "ground_power" => one(switch(crate::tools::ground::GPU_ON, value)),
        "doors" => one(switch(&format!("{}[0]", d::DOOR_OPEN), value)),
        "hydraulic_pumps" => one(switch(d::ELECTRIC_HYDRAULIC_PUMP_ON, value)),
        "start_levers" => Ok(each(crate::tools::jet_start::MIXTURE, &|m| match value {
            "idle" => Term::above(m, 0.3),
            _ => Term::below(m, 0.1),
        })),
        "n2" => Ok(each(d::N2_PERCENT, &|n| match value {
            "turning" => Term::above(n, 20.0),
            "stable" => Term::above(n, 50.0),
            _ => Term::below(n, 10.0),
        })),
        "start_switch" => one(match value {
            "ground" => Term::above(d::ENG_MODE_SELECTOR, 0.5),
            "normal" => Term::new(d::ENG_MODE_SELECTOR, -0.49, 0.49),
            _ => bail!("no stock `start_switch` position `{value}`"),
        }),
        "controls" => {
            let (axis, sign) = match value {
                "roll_left" => (d::CONTROL_ROLL, -1.0),
                "roll_right" => (d::CONTROL_ROLL, 1.0),
                "pitch_forward" => (d::CONTROL_PITCH, -1.0),
                "pitch_aft" => (d::CONTROL_PITCH, 1.0),
                "yaw_left" => (d::CONTROL_YAW, -1.0),
                _ => (d::CONTROL_YAW, 1.0),
            };
            one(if sign < 0.0 {
                Term::below(axis, -0.5)
            } else {
                Term::above(axis, 0.5)
            })
        }
        "lamp_test" => one(Term::above(d::LAMP_TEST, 0.5)),
        "altimeter" => one(Term::new(crate::tools::instruments::BARO, 29.91, 29.93)),
        "autobrake" => one(match value {
            "rto" => Term::below(Tc::AUTOBRAKE, 0.49),
            "off" => Term::at(Tc::AUTOBRAKE, 1.0),
            _ => Term::above(Tc::AUTOBRAKE, 1.5),
        }),
        "bus_tie" => one(switch(d::CROSS_TIE, value)),
        "window_heat" => one(switch(d::WINDOW_HEAT, value)),
        "oxygen" => one(switch(d::O2_VALVE_ON, value)),
        "master_caution" => one(Term::below(d::MASTER_CAUTION, 0.49)),
        "master_warning" => one(Term::below(d::MASTER_WARNING, 0.49)),
        "weather_radar" => one(switch(d::WEATHER_RADAR, value)),
        "speed_bug" => match p.takeoff_config.as_ref().and_then(|t| t.v2_kts) {
            Some(v2) => one(Term::new(
                crate::autoflight::Dial::Airspeed.read_name(),
                v2 - 5.0,
                v2 + 10.0,
            )),
            None => bail!("no V2 on this airframe (`takeoff_config.v2_kts`)"),
        },
        _ => bail!("`{base}` has no stock read; declare it in `checklist_states`"),
    }
}

/// Hazard text for pulling the mixture to idle cutoff. Only worth confirming
/// when the engine is actually turning: pre-start it is the normal lean, so
/// the runner suppresses this prompt when the engine isn't running (see
/// [`engine_running`]).
pub const CONFIRM_MIXTURE_CUTOFF: &str = "cut the mixture and stop the engine";
/// The run-up: power up against the brakes.
const CONFIRM_RUNUP: &str = "run the mag check and advance the throttle";

/// The tool call that sets an actable state to `value`; `None` for a
/// read-only state (or the autopilot's "on", which no tool engages here).
fn action(base: &str, sub: Option<&str>, value: &str, p: &AircraftProfile) -> Option<Action> {
    let on = value == "on";
    let (tool, args, confirm) = match base {
        "park_brake" => ("set_parking_brake", json!({ "on": on }), None),
        "flaps" => {
            let f = p.flaps.as_ref()?;
            let nearest = |r: f64| {
                f.detents
                    .iter()
                    .min_by(|a, b| (a.ratio - r).abs().total_cmp(&(b.ratio - r).abs()))
                    .map(|d| d.label.clone())
            };
            let notch = match value {
                "up" => nearest(0.0)?,
                "takeoff" => nearest(f.takeoff_ratio?)?,
                other => other.to_string(),
            };
            ("set_flaps", json!({ "notch": notch }), None)
        }
        "lights" => ("set_lights", json!({ "light": sub?, "on": on }), None),
        "electrical" => ("set_electrical", json!({ "system": sub?, "on": on }), None),
        "transponder" => ("set_transponder_mode", json!({ "mode": value }), None),
        "mixture" => {
            let pct = match value {
                "rich" => 100,
                "cutoff" => 0,
                _ => 65,
            };
            let confirm = (pct == 0).then_some(CONFIRM_MIXTURE_CUTOFF);
            ("set_mixture", json!({ "percent": pct }), confirm)
        }
        "throttle" => {
            let pct = match value {
                "idle" => 0,
                "start" => 15,
                "runup" => 60,
                "cruise" => 94,
                _ => 100,
            };
            let confirm = (value == "runup").then_some(CONFIRM_RUNUP);
            ("set_throttle", json!({ "percent": pct }), confirm)
        }
        "magnetos" => ("set_magnetos", json!({ "position": value }), None),
        "engine" if value == "running" => ("start_engine", json!({}), Some("start the engine")),
        "engine" => ("stop_engine", json!({}), Some("shut the engine down")),
        "fuel_pump" => ("set_fuel_pump", json!({ "on": on }), None),
        "fuel_selector" => ("set_fuel_selector", json!({ "position": value }), None),
        "pitot_heat" => ("set_pitot_heat", json!({ "on": on }), None),
        "autopilot" if value == "off" => ("autopilot_off", json!({}), None),
        "pitch_trim" => ("set_pitch_trim", json!({ "action": "takeoff" }), None),
        _ => return None,
    };
    Some(Action {
        tool,
        args,
        confirm,
    })
}

/// Is engine 1 running? Used to suppress the mixture-cutoff confirm pre-start.
/// On a read error, assume running (fail safe: keep the confirm).
pub async fn engine_running(xp: &XPlaneClient) -> bool {
    xp.get_float_array(crate::datarefs::ENGINE_RUNNING)
        .await
        .ok()
        .and_then(|v| v.first().copied())
        .is_none_or(|r| r > 0.5)
}

/// Read the value behind a (possibly indexed) dataref.
pub async fn read_dataref(xp: &XPlaneClient, dataref: &str, index: Option<usize>) -> Option<f64> {
    match index {
        Some(i) => xp.get_float_array(dataref).await.ok()?.get(i).copied(),
        None => xp.get_float(dataref).await.ok(),
    }
}

/// Perform one checklist action through its curated tool (which reads back,
/// so the action is verified), built from the airframe's profile.
pub async fn run_step(a: &Action, p: &AircraftProfile, ctx: &ToolCtx<'_>) -> Result<String> {
    let args = &a.args;
    match a.tool {
        "set_parking_brake" => {
            SetParkingBrake {
                valve_set: p.park_brake_valve_set,
            }
            .execute(args, ctx)
            .await
        }
        "set_pitch_trim" => SetPitchTrim.execute(args, ctx).await,
        "set_electrical" => {
            SetElectrical::new(p.electrical.clone())
                .execute(args, ctx)
                .await
        }
        "set_mixture" => SetMixture.execute(args, ctx).await,
        "set_throttle" => SetThrottle.execute(args, ctx).await,
        "set_fuel_selector" => match &p.fuel_selector {
            Some(f) => SetFuelSelector::new(f.clone()).execute(args, ctx).await,
            None => bail!("no fuel selector on this airframe"),
        },
        "set_magnetos" => {
            SetMagnetos {
                engines: p.engines,
                per_engine: p.magnetos_per_engine,
            }
            .execute(args, ctx)
            .await
        }
        "set_pitot_heat" => {
            SetPitotHeat {
                override_dataref: p.pitot_heat_dataref.clone(),
            }
            .execute(args, ctx)
            .await
        }
        // Universal-array pumps only, as the lists always drove them.
        "set_fuel_pump" => {
            SetFuelPump {
                engines: p.engines,
                commands: None,
            }
            .execute(args, ctx)
            .await
        }
        "set_transponder_mode" => SetTransponderMode.execute(args, ctx).await,
        "autopilot_off" => AutopilotOff.execute(args, ctx).await,
        "start_engine" => {
            StartEngine {
                engines: p.engines,
                cycle_selectors: Vec::new(),
                fuel_selector: p.fuel_selector.clone(),
                alternator_on_cmds: crate::tools::alternator_on_cmds(&p.electrical),
                bus_power_on_cmds: crate::tools::bus_power_on_cmds(&p.electrical),
            }
            .execute(args, ctx)
            .await
        }
        "stop_engine" => {
            StopEngine {
                engines: p.engines,
                park_brake_valve_set: p.park_brake_valve_set,
            }
            .execute(args, ctx)
            .await
        }
        "set_lights" => SetLights::new(p.lights.clone()).execute(args, ctx).await,
        "set_flaps" => match &p.flaps {
            Some(f) => SetFlaps::new(f.clone()).execute(args, ctx).await,
            None => bail!("no flaps on this airframe"),
        },
        other => bail!("checklist step references unknown tool {other}"),
    }
}

/// Does the utterance ask for a checklist at all? One place for the whisper
/// wart where the compound splits ("check list"), so the caller's decline
/// logic and [`match_request`] can never disagree on what counts.
pub fn mentions_checklist(transcript: &str) -> bool {
    let t = transcript.to_lowercase();
    t.contains("checklist") || t.contains("check list")
}

/// Deterministic voice match: resolve "run the before takeoff checklist" (in
/// any polite wrapping, whisper warts included) to one of this airframe's
/// checklists. The word "checklist" is REQUIRED, so a bare "climb" or
/// "descent" can never trigger a whole phase by accident. Title words match
/// with prefix tolerance ("start" ~ "starting"), candidates are scored by how
/// much of the title went unmatched, and a tie prefers the "before" section
/// (a bare "takeoff checklist" means BEFORE takeoff, not the roll callouts;
/// same for "landing"). Ambiguity that survives all of that returns `None`,
/// never a guess - the caller declines and names the options.
pub fn match_request<'a>(transcript: &str, checklists: &'a [Checklist]) -> Option<&'a Checklist> {
    let query = request_words(transcript);
    if query.is_empty() {
        return None;
    }
    // (unmatched title words, checklist) for every checklist whose title
    // accounts for EVERY query word - a stray word excludes the candidate.
    let mut scored: Vec<(usize, &Checklist)> = checklists
        .iter()
        .filter_map(|c| {
            // The key's words, or the section's own title's.
            let title = c.title.to_lowercase();
            [c.id.split('_').collect::<Vec<_>>(), words(&title)]
                .into_iter()
                .filter_map(|title| {
                    let matched = |q: &String| title.iter().any(|t| word_match(q, t));
                    query.iter().all(matched).then(|| {
                        let hit = title
                            .iter()
                            .filter(|t| query.iter().any(|q| word_match(q, t)))
                            .count();
                        title.len() - hit
                    })
                })
                .min()
                .map(|s| (s, c))
        })
        .collect();
    scored.sort_by_key(|(s, _)| *s);
    match scored.as_slice() {
        [] => None,
        [(_, one)] => Some(one),
        [(a, best), (b, _), ..] if a < b => Some(best),
        tied => {
            // Joint best score: take the "before" section if exactly one of the
            // tied leaders has one, otherwise refuse to guess.
            let top = tied.iter().take_while(|(s, _)| *s == tied[0].0);
            let mut before = top.filter(|(_, c)| c.id.starts_with("before_"));
            match (before.next(), before.next()) {
                (Some((_, one)), None) => Some(one),
                _ => None,
            }
        }
    }
}

/// The utterance reduced to the words that could name a checklist: lowercase,
/// whisper-split compounds rejoined ("check list", "take off", "pre flight",
/// "shut down"), and the filler of a spoken request stripped. Empty when the
/// utterance never says "checklist" - the required trigger word.
fn request_words(transcript: &str) -> Vec<String> {
    let text = transcript.to_lowercase();
    // Single letters are apostrophe shrapnel ("let's" splits to "let", "s"),
    // never part of a section name - drop them with the empties. A single
    // DIGIT is a part's number (the MD-82's "shut down 1" and its "2"), kept
    // so the parts are told apart.
    let mut words: Vec<String> = text
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| w.len() > 1 || w.bytes().any(|c| c.is_ascii_digit()))
        .map(str::to_string)
        .collect();
    let mut i = 0;
    while i + 1 < words.len() {
        let joined = match (words[i].as_str(), words[i + 1].as_str()) {
            ("check", "list" | "lists") => Some("checklist"),
            ("take", "off") => Some("takeoff"),
            ("pre", "flight") => Some("preflight"),
            ("shut", "down") => Some("shutdown"),
            _ => None,
        };
        if let Some(j) = joined {
            words[i] = j.to_string();
            words.remove(i + 1);
        }
        i += 1;
    }
    if !words.iter().any(|w| w == "checklist" || w == "checklists") {
        return Vec::new();
    }
    const FILLER: &[&str] = &[
        "checklist",
        "checklists",
        "run",
        "running",
        "do",
        "read",
        "give",
        "perform",
        "complete",
        "execute",
        "begin",
        "go",
        "through",
        "over",
        "the",
        "a",
        "an",
        "my",
        "me",
        "us",
        "please",
        "let",
        "lets",
        "now",
        "for",
        "to",
        "and",
        "then",
        "okay",
        "ok",
        "roger",
        "it",
        "its",
        "is",
        "time",
        "ready",
        "with",
        "you",
        "your",
        "we",
        "can",
        "could",
    ];
    words.retain(|w| !FILLER.contains(&w.as_str()));
    words
}

/// A title's words, lowercase, without the words that only say it is a list
/// ("procedure", "checklist").
fn words(title: &str) -> Vec<&str> {
    title
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty() && !matches!(*w, "procedure" | "procedures" | "checklist"))
        .collect()
}

/// A query word matches a title word exactly, or by prefix when the shorter of
/// the two carries at least four letters - "start" reaches "starting",
/// "engine" reaches "engines", but "off" can't reach "takeoff".
fn word_match(q: &str, t: &str) -> bool {
    q == t || (q.len() >= 4 && t.starts_with(q)) || (t.len() >= 4 && q.starts_with(t))
}

pub(crate) fn cap_first(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// Split `dataref[0]` → (`dataref`, `Some(0)`); a plain name → (`dataref`, None).
fn split_index(dr: &str) -> (String, Option<usize>) {
    if let Some(open) = dr.find('[') {
        if let Some(close) = dr[open..].find(']') {
            let idx = dr[open + 1..open + close].parse().ok();
            return (dr[..open].to_string(), idx);
        }
    }
    (dr.to_string(), None)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p172() -> AircraftProfile {
        crate::aircraft::load("cessna_172sp").unwrap()
    }

    /// Every shipped list loads against its airframe: no unknown state, no
    /// value the airframe cannot read. A file without a profile is an error.
    #[test]
    fn every_shipped_checklist_loads_against_its_profile() {
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/checklists");
        let mut n = 0;
        for e in std::fs::read_dir(dir).unwrap() {
            let path = e.unwrap().path();
            let id = path.file_stem().unwrap().to_str().unwrap().to_string();
            assert_eq!(
                path.extension().and_then(|x| x.to_str()),
                Some("yaml"),
                "{id}"
            );
            let p = crate::aircraft::load(&id).unwrap_or_else(|| panic!("no profile {id}"));
            assert_eq!(p.id, id, "a profile of its own");
            let lists = load(&p).unwrap_or_else(|e| panic!("{e:#}"));
            assert!(!lists.is_empty(), "{id}");
            n += 1;
        }
        assert_eq!(n, 9);
    }

    #[test]
    fn an_unknown_state_or_value_fails_the_load_naming_the_item() {
        let err = |text: &str| format!("{:#}", parse(text, &p172()).err().expect(text));
        let e = err("climb:\n  - { say: x }\n  - { say: y, check: flapz, is: up }\n");
        assert!(e.contains("climb item 2") && e.contains("flapz"), "{e}");
        let e = err("climb:\n  - { say: y, check: flaps, is: sideways }\n");
        assert!(e.contains("sideways"), "{e}");
        // The 172 has no `wing` light, and no V2 for a speed bug.
        assert!(err("a:\n  - { say: y, check: lights.wing, is: on }\n").contains("wing"));
        assert!(err("a:\n  - { say: y, check: speed_bug, is: v2 }\n").contains("V2"));
        assert!(err("a:\n  - { say: y, check: lights, is: on }\n").contains("lights"));
        // A misspelt key is no silent call-out.
        assert!(err("a:\n  - { say: y, chek: gear, is: up }\n").contains("chek"));
        assert!(err("a:\n  - { say: y, check: gear }\n").contains("together"));
    }

    #[test]
    fn a_state_reads_its_tools_home_and_acts_through_the_tool() {
        let l = parse(
            r#"a:
  - { say: "Flaps, ten.", check: flaps, is: "10" }
  - { say: "Taxi light, on.", check: lights.taxi, is: on }
  - { say: "Gear, down.", check: gear, is: down }
  - { say: "Brake, on.", check: park_brake, is: on, act: false }
  - { note: "Run-up." }
"#,
            &p172(),
        )
        .unwrap();
        let it = &l[0].items;
        assert_eq!(l[0].title, "A");
        assert_eq!(it[0].action().unwrap().args, json!({ "notch": "10" }));
        assert_eq!(it[0].terms()[0].dataref, crate::datarefs::FLAP_RATIO);
        assert!(it[0].terms()[0].holds(0.3333) && !it[0].terms()[0].holds(0.0));
        assert_eq!(
            it[1].terms()[0].dataref,
            "sim/cockpit2/switches/taxi_light_on"
        );
        // Read-only, and `act: false`: confirmed, never written.
        assert!(it[2].has_test() && it[2].action().is_none());
        assert!(it[3].has_test() && it[3].action().is_none());
        assert!(it[4].note && !it[4].has_test());
        // Unread is no verdict; every term must hold.
        assert_eq!(judge_read(&it[1], &[Some(1.0)]), Some(true));
        assert_eq!(judge_read(&it[1], &[Some(0.0)]), Some(false));
        assert_eq!(judge_read(&it[1], &[None]), None);
        assert_eq!(judge_read(&it[4], &[]), None);
    }

    /// A read-only state reads every engine, or the one it names; an
    /// actable state names none.
    #[test]
    fn a_state_reads_every_engine_or_the_one_it_names() {
        let p = crate::aircraft::load("boeing_737_800").unwrap();
        let l = parse(
            r#"a:
  - { say: x, check: start_levers, is: cutoff }
  - { say: x, check: n2.2, is: stable }
  - { say: x, check: fuel_pumps.3, is: on }
  - { say: x, check: start_switch.2, is: ground }
"#,
            &p,
        )
        .unwrap();
        let it = &l[0].items;
        let idx =
            |i: usize| -> Vec<Option<usize>> { it[i].terms().iter().map(|t| t.index).collect() };
        assert_eq!(idx(0), vec![Some(0), Some(1)]);
        assert_eq!(idx(1), vec![Some(1)]);
        assert_eq!(idx(2), vec![Some(2)]);
        assert_eq!(
            it[3].terms()[0].dataref,
            "laminar/B738/spring_knob/starter_2"
        );
        for bad in ["flaps.2", "n2.0", "n2.x"] {
            let text = format!("a:\n  - {{ say: y, check: {bad}, is: up }}\n");
            assert!(parse(&text, &p).is_err(), "{bad}");
        }
        // The 172's power: cruise and the start, read and set.
        let l = parse(
            "a:\n  - { say: y, check: throttle, is: cruise }\n  - { say: y, check: throttle, is: start }\n",
            &p172(),
        )
        .unwrap();
        assert_eq!(
            l[0].items[0].action().unwrap().args,
            json!({ "percent": 94 })
        );
        assert_eq!(
            l[0].items[1].action().unwrap().args,
            json!({ "percent": 15 })
        );
        assert!(l[0].items[1].terms()[0].holds(0.18));
    }

    #[test]
    fn a_section_may_carry_its_own_title() {
        let l = parse(
            "shutdown:\n  title: Engine shutdown\n  items:\n    - { say: y }\n",
            &p172(),
        )
        .unwrap();
        assert_eq!(
            (l[0].id.as_str(), l[0].title.as_str()),
            ("shutdown", "Engine shutdown")
        );
        assert!(parse("a:\n  title: T\n  items: []\n  other: 1\n", &p172()).is_err());
    }

    #[test]
    fn hazard_steps_are_confirmed() {
        let lists = load(&p172()).unwrap();
        let section = |k: &str| &lists.iter().find(|c| c.id == k).unwrap().items;
        let confirms = |k: &str| -> Vec<(&str, &str)> {
            section(k)
                .iter()
                .filter_map(|i| Some((i.action()?.tool, i.action()?.confirm?)))
                .collect()
        };
        assert_eq!(
            confirms("engine_start"),
            vec![("start_engine", "start the engine")]
        );
        assert_eq!(
            confirms("shutdown"),
            vec![("set_mixture", CONFIRM_MIXTURE_CUTOFF)]
        );
        // The run-up is the one confirm before takeoff: the throttle-up.
        assert_eq!(
            confirms("before_takeoff"),
            vec![("set_throttle", CONFIRM_RUNUP)]
        );
    }

    #[test]
    fn sr22_checklist_acts_on_wired_tools_only() {
        let p = crate::aircraft::load("cirrus_sr22").unwrap();
        let cls = load(&p).expect("SR22 checklist loads");
        assert!(cls.len() > 15, "expected the full SR22 section set");
        let tools: std::collections::BTreeSet<&str> = cls
            .iter()
            .flat_map(|c| &c.items)
            .filter_map(|i| Some(i.action()?.tool))
            .collect();
        let wired = [
            "set_parking_brake",
            "set_transponder_mode",
            "set_pitot_heat",
            "set_lights",
            "set_throttle",
            "set_mixture",
            "set_flaps",
            "set_electrical",
            "set_fuel_pump",
            "set_magnetos",
            "start_engine",
            "set_pitch_trim",
            "set_fuel_selector",
        ];
        assert!(tools.iter().all(|t| wired.contains(t)), "{tools:?}");
        for t in [
            "set_fuel_pump",
            "set_magnetos",
            "set_pitch_trim",
            "set_fuel_selector",
        ] {
            assert!(tools.contains(t), "expected the SR22 list to drive {t}");
        }
    }

    #[test]
    fn voice_request_resolves_by_title() {
        let cls = load(&p172()).expect("172 checklist loads");
        let hit = |t: &str| match_request(t, &cls).map(|c| c.id.as_str());

        // The live miss (2026-07-14, G1000 172): whisper's exact output.
        assert_eq!(hit("before takeoff checklist"), Some("before_takeoff"));
        // Polite wrappings, split compounds, and whisper's trailing period/"?"
        // arrive as separator noise - all resolve to the same section.
        assert_eq!(
            hit("run the before take off check list"),
            Some("before_takeoff")
        );
        assert_eq!(
            hit("let's do the pre-flight checklist, please"),
            Some("preflight")
        );
        assert_eq!(hit("checklist before taxi"), Some("before_taxi"));
        assert_eq!(hit("shut down checklist"), Some("shutdown"));
        assert_eq!(hit("cruise checklist"), Some("cruise"));
        assert_eq!(hit("after landing checklist"), Some("after_landing"));
        // A bare phase prefers the "before" section - that's what pilots mean.
        assert_eq!(hit("takeoff checklist"), Some("before_takeoff"));
        assert_eq!(hit("landing checklist"), Some("before_landing"));
        assert_eq!(hit("engine start checklist"), Some("engine_start"));
        assert_eq!(hit("before start checklist"), Some("before_start"));
        // The airframe's own section names still reach them.
        assert_eq!(hit("engine shutdown checklist"), Some("shutdown"));
        assert_eq!(
            hit("before starting engine checklist"),
            Some("before_start")
        );
        assert_eq!(hit("starting engine checklist"), Some("engine_start"));
        let shut = cls.iter().find(|c| c.id == "shutdown").unwrap();
        assert_eq!(shut.title, "Engine shutdown");

        // Never a guess: ambiguous or unknown asks return None (the caller
        // declines and names the options), and without the trigger word
        // nothing matches at all - "climb" alone must stay a climb command.
        assert_eq!(hit("run the checklist"), None);
        assert_eq!(hit("securing aircraft checklist"), None);
        assert_eq!(hit("climb"), None);
        assert_eq!(hit("before takeoff"), None);
        assert_eq!(hit("flaps up"), None);

        assert!(mentions_checklist("run the check list"));
        assert!(mentions_checklist("before takeoff checklist"));
        assert!(!mentions_checklist("check the flaps"));
    }

    #[test]
    fn titles_and_indexes() {
        assert_eq!(title_of("before_takeoff"), "Before takeoff");
        assert_eq!(
            split_index("sim/.../mixture_ratio[0]"),
            ("sim/.../mixture_ratio".into(), Some(0))
        );
        assert_eq!(
            split_index("sim/.../parkbrake"),
            ("sim/.../parkbrake".into(), None)
        );
    }
}
