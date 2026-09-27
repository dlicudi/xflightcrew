//! "Set up the flight": the flight plan's aircraft, at a stand that suits it at
//! the plan's origin, cold and dark, then the GPS and the load sheet from the
//! plan. One ask instead of X-Plane's flight configuration screen, the plan
//! page and the load sheet, and the same path the unattended sortie runner
//! takes to load an aircraft at a field.
//!
//! Only the plugin can start a flight (`XPLMInitFlight`, on the main thread):
//! the app writes the flight-initialisation JSON to `xcopilot/sim/init_flight`
//! and waits on `xcopilot/sim/init_seq`/`init_result`.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::voice::{Priority, Voice};

const INIT_JSON: &str = "xcopilot/sim/init_flight";
const INIT_RESULT: &str = "xcopilot/sim/init_result";
const INIT_SEQ: &str = "xcopilot/sim/init_seq";
const TYPED: &str = "xcopilot/typed";

/// The ICAO type an `.acf` declares (`P acf/_ICAO B738`). Pure.
pub fn acf_icao(acf_text: &str) -> Option<String> {
    acf_text.lines().find_map(|l| {
        let rest = l.strip_prefix("P acf/_ICAO ")?;
        let t = rest.trim();
        (!t.is_empty()).then(|| t.to_ascii_uppercase())
    })
}

/// Every installed aircraft's `.acf`, relative to the X-Plane root, with the
/// ICAO type it declares. Read once per process: the files are megabytes and
/// the type sits tens of thousands of lines in.
pub fn aircraft_library(root: &str) -> &'static [(String, String)] {
    static LIB: std::sync::OnceLock<Vec<(String, String)>> = std::sync::OnceLock::new();
    LIB.get_or_init(|| {
        let base = std::path::Path::new(root);
        let mut out = Vec::new();
        let mut stack = vec![base.join("Aircraft")];
        while let Some(dir) = stack.pop() {
            let Ok(rd) = std::fs::read_dir(&dir) else { continue };
            for e in rd.flatten() {
                let p = e.path();
                if p.is_dir() {
                    stack.push(p);
                } else if p.extension().is_some_and(|x| x.eq_ignore_ascii_case("acf")) {
                    let Ok(text) = std::fs::read_to_string(&p) else { continue };
                    if let (Some(icao), Ok(rel)) = (acf_icao(&text), p.strip_prefix(base)) {
                        out.push((icao, rel.to_string_lossy().to_string()));
                    }
                }
            }
        }
        out.sort();
        out
    })
}

/// The installed aircraft for an ICAO type: an exact match, Laminar's own
/// first (the profiles are written against them). Pure over the library.
pub fn find_acf<'a>(lib: &'a [(String, String)], icao: &str) -> Option<&'a str> {
    let icao = icao.trim().to_ascii_uppercase();
    let mut hits: Vec<&(String, String)> = lib.iter().filter(|(t, _)| *t == icao).collect();
    hits.sort_by_key(|(_, p)| (!p.contains("Laminar Research"), p.len()));
    hits.first().map(|(_, p)| p.as_str())
}

/// What a stand must accept for this aircraft (apt.dat 1300 categories).
pub fn stand_class(jet: bool, heli: bool) -> &'static [&'static str] {
    match (jet, heli) {
        (_, true) => &["helos"],
        (true, _) => &["jets", "heavy"],
        _ => &["props", "turboprops"],
    }
}

/// A stand for the aircraft: a gate (a jet) or a tie-down (a prop) that
/// accepts its class, never a hangar; the first by name, so the pick is
/// repeatable. Pure.
pub fn pick_stand<'a>(stands: &'a [crate::nav::Stand], class: &[&str], jet: bool) -> Option<&'a crate::nav::Stand> {
    let fits = |s: &crate::nav::Stand| {
        s.is_suitable()
            && (class.is_empty() || s.accepts.is_empty() || s.accepts.iter().any(|a| class.contains(&a.as_str())))
    };
    let preferred = if jet { "gate" } else { "tie_down" };
    let mut pool: Vec<&crate::nav::Stand> = stands
        .iter()
        .filter(|s| fits(s) && s.kind.eq_ignore_ascii_case(preferred))
        .collect();
    if pool.is_empty() {
        pool = stands.iter().filter(|s| fits(s)).collect();
    }
    pool.sort_by(|a, b| a.name.cmp(&b.name));
    pool.first().copied()
}

/// The flight-initialisation JSON: the aircraft at the ramp, cold and dark.
pub fn init_json(acf_rel: &str, airport: &str, ramp: &str) -> String {
    json!({
        "aircraft": { "path": acf_rel },
        "ramp_start": { "airport_id": airport, "ramp": ramp },
        "engine_status": { "all_engines": { "running": false } },
    })
    .to_string()
}

pub struct SetUpFlight;

#[async_trait]
impl Tool for SetUpFlight {
    fn name(&self) -> &str {
        "set_up_flight"
    }
    fn description(&self) -> &str {
        "Set up the flight from the flight plan: load the plan's aircraft at a stand at \
         its origin, cold and dark, then load the GPS and the fuel and payload. For \
         'set up the flight', 'set up my flight', 'prepare the flight'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Starting a flight ENDS this one: only parked, and nothing flying it.
        let (ground, gs) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY),
            ctx.xplane.get_float("sim/flightmodel/position/groundspeed"),
        );
        if !ground.is_ok_and(|g| g > 0.5) || !gs.is_ok_and(|v| v.abs() < 1.0) {
            return Ok("Only when we're stopped on the ground: setting up a flight ends this one.".into());
        }
        if crate::full_auto::ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
            return Ok("Full auto is flying. Say my controls first.".into());
        }
        let fp = match super::loading::plan_for_loading(&ctx.config.simbrief_id).await {
            Ok(fp) => fp,
            Err(line) => return Ok(line),
        };
        // The first scan reads every installed .acf: off the command loop.
        let root = super::autoland::xplane_root();
        let lib = tokio::task::spawn_blocking(move || aircraft_library(&root)).await?;
        let Some(acf) = find_acf(lib, &fp.aircraft) else {
            return Ok(format!(
                "I can't find an installed aircraft of type {}, the plan's. Load it yourself, or change the plan.",
                fp.aircraft
            ));
        };
        let Ok(Some(net)) = crate::nav::find_taxi_net(&ctx.config.apt_data_path, &fp.origin) else {
            return Ok(format!("{} has no parking positions I can read.", fp.origin));
        };
        // Jet or not, from a profile of the plan's TYPE (a plan names no
        // model, and any model of the type answers this); an unknown type
        // takes any suitable stand.
        let known = crate::aircraft::profile_for_type(&fp.aircraft);
        let jet = known.as_ref().is_some_and(|p| {
            p.reference
                .as_ref()
                .and_then(|r| r.kind.as_deref())
                .is_some_and(|k| k.to_ascii_lowercase().contains("jet"))
        });
        let heli = known.as_ref().is_some_and(|p| p.rotorcraft);
        let class: &[&str] = if known.is_some() { stand_class(jet, heli) } else { &[] };
        let Some(stand) = pick_stand(&net.stands, class, jet) else {
            return Ok(format!("No stand at {} takes this aircraft.", fp.origin));
        };
        let body = init_json(acf, &fp.origin, &stand.name);
        let (base, voice) = (ctx.config.xplane_base_url.clone(), ctx.voice.cloned());
        let (origin, name, stand_name) = (fp.origin.clone(), fp.aircraft.clone(), stand.name.clone());
        crate::blackbox::record(
            "setup",
            json!({ "event": "init", "acf": acf, "airport": origin, "ramp": stand_name, "json": body }),
        );
        spawn_off_loop(async move {
            follow_up(base, voice, body, origin, stand_name).await;
        });
        Ok(format!(
            "Setting up: the {name} at {}, {}, cold and dark. Then the plan and the load sheet.",
            fp.origin, stand.name
        ))
    }
}

/// After the write: wait for the plugin to run it, then for the sim to have
/// reloaded, then ask for the GPS load and the load sheet the way a typed
/// command would (the aircraft has changed, so the registry has too).
async fn follow_up(base: String, voice: Option<Voice>, body: String, origin: String, stand: String) {
    let xp = crate::xplane::XPlaneClient::new(base);
    let say = |line: &str| {
        if let Some(v) = &voice {
            v.alert(Priority::Advisory, line.to_string());
        }
    };
    // The plugin runs it on its next flight loop; the reload itself can take
    // a minute with scenery to load.
    let t0 = std::time::Instant::now();
    let run = run_init(&xp, body.as_bytes(), InitTiming::RAMP, &|w| {
        crate::blackbox::record("setup", json!({ "event": "init_wait", "seq": w.seq, "result": w.result,
            "seq0": w.seq0, "read_errors": w.read_errors, "polls": w.polls }));
    })
    .await;
    let result = match run.outcome {
        InitOutcome::Done(code) => Some(code),
        InitOutcome::Unreachable | InitOutcome::NoBaseline => {
            crate::blackbox::record(
                "setup",
                json!({ "event": "result", "outcome": run.outcome.name(),
                        "read_errors": run.read_errors }),
            );
            say("I couldn't reach the plugin to set up the flight. Is it up to date?");
            return;
        }
        InitOutcome::TimedOut => None,
    };
    crate::blackbox::record("setup", json!({ "event": "result", "result": result }));
    match result {
        Some(0) => {}
        Some(code) => {
            let why = match code {
                1 => "the request was invalid",
                2 => "the aircraft would not load",
                4 => "X-Plane doesn't know the airport",
                5 => "that stand isn't in the loaded scenery",
                _ => "X-Plane refused it",
            };
            say(&format!("The setup didn't take: {why}."));
            return;
        }
        None => {
            say("The setup didn't answer. Check X-Plane.");
            return;
        }
    }
    // Settled: on the ground and still for five seconds, then the profile
    // detection has its moment before the plan and the load.
    let mut still = 0;
    while t0.elapsed().as_secs() < 240 && still < 10 {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        let (g, gs) = tokio::join!(
            xp.get_float(crate::datarefs::ON_GROUND_ANY),
            xp.get_float("sim/flightmodel/position/groundspeed"),
        );
        still = if g.is_ok_and(|g| g > 0.5) && gs.is_ok_and(|v| v.abs() < 0.5) { still + 1 } else { 0 };
    }
    tokio::time::sleep(std::time::Duration::from_secs(8)).await;
    say(&format!("We're at {origin}, {stand}. Loading the plan and the load sheet."));
    let _ = xp.set_data(TYPED, b"load the flight plan").await;
    tokio::time::sleep(std::time::Duration::from_secs(8)).await;
    let _ = xp.set_data(TYPED, b"load the aircraft").await;
    crate::blackbox::record("setup", json!({ "event": "done", "airport": origin, "ramp": stand }));
}

// ---- A new flight on a final --------------------------------------------------
//
// "Put us on a ten mile final to LEMG one three": the CURRENT aircraft on a
// runway final, engines running, through the same mailbox. X-Plane places the
// aircraft with NOBODY flying it (autopilot off, 180 to 210 kt, sinking 1,000
// to 3,500 fpm on LEMG's 10 nm final over 2,300 ft hills, 2026-09-26: forty
// seconds of "Airspeed!" and "Sink rate!" and no hands until a typed
// "recover the aircraft"). So the copilot takes the aircraft the moment the
// flight exists: autoland FORCED onto that runway, armed on the result, or,
// when the pilot said "my controls", monitoring only. The tower is told it is
// a new flight: its station is rebuilt and the final report goes out.

/// The final a pilot means by "put us on final": ten miles.
pub const DEFAULT_FINAL_NM: i64 = 10;
/// The longest final the setup accepts, miles.
pub const MAX_FINAL_NM: i64 = 30;

/// The flight-initialisation JSON for a runway final: the aircraft that is
/// loaded, the field and runway, the distance out, engines running (the SDK's
/// `runway_start` with `final_distance_in_nautical_miles`; without the distance
/// it is a start on the numbers, which this never writes). Pure.
pub fn approach_init_json(acf_rel: &str, airport: &str, runway: &str, final_nm: i64) -> String {
    json!({
        "aircraft": { "path": acf_rel },
        "runway_start": {
            "airport_id": airport,
            "runway": runway,
            "final_distance_in_nautical_miles": final_nm,
        },
        "engine_status": { "all_engines": { "running": true } },
    })
    .to_string()
}

/// Why X-Plane refused an initialisation, in words (`XPLMInitResult`,
/// XPLMPlanes.h). Pure.
pub fn init_refusal(code: i64, icao: &str, runway: &str) -> String {
    let why = match code {
        1 => "X-Plane called the request invalid".to_string(),
        2 => "X-Plane couldn't load this aircraft".to_string(),
        3 => "X-Plane couldn't load the livery".to_string(),
        4 => format!("X-Plane doesn't know the airport {}", crate::flightplan::spell_code(icao)),
        5 => "X-Plane couldn't find that ramp".to_string(),
        6 => format!(
            "X-Plane has no runway {} at {}",
            crate::circuit::spell_runway(runway),
            crate::flightplan::spell_code(icao)
        ),
        _ => format!("X-Plane refused it, code {code}"),
    };
    format!("The approach setup didn't take: {why}.")
}

/// What follows the plugin's answer. `Arm` is the copilot's hands on the new
/// flight at once, autoland forced onto the runway the flight was started
/// on; `Monitor` leaves the pilot flying.
#[derive(Debug, Clone, PartialEq)]
pub enum AfterInit {
    Arm { runway: crate::autoland::PlannedRunway, line: String },
    Monitor { line: String },
    Failed { line: String },
}

/// The decision on the plugin's result (`None`: no answer in time). Pure:
/// the follow-up applies it with nothing between the result and the arm.
pub fn after_init(
    result: Option<i64>,
    icao: &str,
    runway: &str,
    final_nm: i64,
    hand_fly: bool,
    can_land: bool,
) -> AfterInit {
    let spoken = crate::circuit::spell_runway(runway);
    match result {
        Some(0) if hand_fly => AfterInit::Monitor {
            line: format!("On a {final_nm} mile final for {spoken}. Your aircraft, I'm monitoring."),
        },
        Some(0) if !can_land => AfterInit::Monitor {
            line: format!(
                "On a {final_nm} mile final for {spoken}. I have no autopilot to fly it with. Your aircraft."
            ),
        },
        Some(0) => AfterInit::Arm {
            runway: crate::autoland::PlannedRunway::at(icao, runway),
            line: format!("On a {final_nm} mile final for {spoken}, I have the aircraft."),
        },
        Some(code) => AfterInit::Failed { line: init_refusal(code, icao, runway) },
        None => AfterInit::Failed {
            line: "The approach setup didn't answer. Check X-Plane.".to_string(),
        },
    }
}

/// "13" and "013" and "13" are one end; "13" is not "13L". Pure.
fn same_end(a: &str, b: &str) -> bool {
    let split = |s: &str| {
        let s = s.trim().to_ascii_uppercase();
        let digits: String = s.chars().take_while(|c| c.is_ascii_digit()).collect();
        let n = digits.parse::<u32>().ok();
        (n, s[digits.len()..].to_string())
    };
    let (na, sa) = split(a);
    na.is_some() && (na, sa) == split(b)
}

/// Every runway end the field has, as apt.dat spells them.
pub fn runway_ends(runways: &[crate::nav::Runway]) -> Vec<String> {
    runways
        .iter()
        .flat_map(|r| [r.ident_a.clone(), r.ident_b.clone()])
        .collect()
}

/// The apt.dat spelling of the end the words name, if the field has it.
pub fn match_runway_end(runways: &[crate::nav::Runway], asked: &str) -> Option<String> {
    runway_ends(runways).into_iter().find(|e| same_end(e, asked))
}

/// The runway for the new final, and where it came from: the pilot's words,
/// else the tower's clearance at this field, else the plan's arrival runway,
/// else the end facing the field's METAR wind, else the longest runway. A
/// runway the pilot NAMED that the field lacks is refused with the ends it
/// has, never swapped for another. Pure.
pub fn pick_runway(
    icao: &str,
    runways: &[crate::nav::Runway],
    asked: Option<&str>,
    tower: Option<&str>,
    plan: Option<&str>,
    wind: Option<(f64, f64)>,
) -> Result<(String, &'static str), String> {
    if let Some(a) = asked {
        return match_runway_end(runways, a).map(|e| (e, "asked")).ok_or_else(|| {
            format!(
                "{} has no runway {}. It has {}.",
                crate::flightplan::spell_code(icao),
                crate::circuit::spell_runway(a),
                runway_ends(runways)
                    .iter()
                    .map(|e| crate::circuit::spell_runway(e))
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        });
    }
    if let Some(e) = tower.and_then(|t| match_runway_end(runways, t)) {
        return Ok((e, "tower"));
    }
    if let Some(e) = plan.and_then(|p| match_runway_end(runways, p)) {
        return Ok((e, "plan"));
    }
    if let Some((from, kts)) = wind {
        if let Some((e, _, _)) = crate::nav::best_runway_for_wind(runways, from, kts) {
            return Ok((e, "wind"));
        }
    }
    crate::nav::assignable_runways(runways)
        .max_by(|a, b| a.length_m().total_cmp(&b.length_m()))
        .map(|r| (r.ident_a.clone(), "longest"))
        .ok_or_else(|| format!("{} has no runway I can read.", crate::flightplan::spell_code(icao)))
}

/// The automation that holds the aircraft now, by name, if any: a new flight
/// ends it, so the pilot takes it back first (a misheard ask must not pull
/// the aircraft out from under a landing).
fn flying_now() -> Option<&'static str> {
    use std::sync::atomic::Ordering::Relaxed;
    [
        (crate::full_auto::ACTIVE.load(Relaxed), "Full auto"),
        (crate::autoland::ACTIVE.load(Relaxed), "Autoland"),
        (crate::circuit::ACTIVE.load(Relaxed), "The circuit"),
        (crate::procedure::ACTIVE.load(Relaxed), "The route"),
        (crate::autotakeoff::ACTIVE.load(Relaxed), "The takeoff"),
        (crate::heli_route::ACTIVE.load(Relaxed), "The helicopter route"),
        (crate::hover::ACTIVE.load(Relaxed), "The hover"),
    ]
    .into_iter()
    .find_map(|(on, name)| on.then_some(name))
}

#[derive(serde::Deserialize, Default, Clone)]
struct ApproachArgs {
    #[serde(default)]
    final_nm: Option<i64>,
    #[serde(default)]
    runway: Option<String>,
    #[serde(default)]
    to: Option<String>,
    #[serde(default)]
    to_codes: Vec<String>,
    #[serde(default)]
    hand_fly: bool,
}

/// The field and its runways, resolved off the command loop: the named
/// place, else the plan's destination, else the nearest field.
fn resolve_field(
    apt: &str,
    a: &ApproachArgs,
    here: (f64, f64),
) -> Result<(String, crate::nav::AirportInfo), String> {
    let icao = if a.to.is_some() || !a.to_codes.is_empty() {
        let dest = crate::flightplan::Destination {
            codes: a.to_codes.clone(),
            name: a.to.clone().unwrap_or_default(),
        };
        let spoken = a.to.clone().unwrap_or_else(|| dest.spoken());
        match crate::flightplan::resolve(apt, &dest, here) {
            Ok(r) => r
                .dest
                .map(|d| d.icao)
                .ok_or_else(|| format!("I don't know an airport called {spoken}."))?,
            Err(_) => return Err("I can't read the airport data.".to_string()),
        }
    } else if let Some(d) =
        crate::simbrief::loaded_plan().map(|p| p.destination).filter(|d| !d.trim().is_empty())
    {
        d
    } else {
        let none = crate::flightplan::Destination { codes: Vec::new(), name: String::new() };
        match crate::flightplan::resolve(apt, &none, here) {
            Ok(r) => r.origin.map(|o| o.icao).ok_or("There's no field near us I can find.")?,
            Err(_) => return Err("I can't read the airport data.".to_string()),
        }
    };
    match crate::nav::find_airport(apt, &icao) {
        Ok(Some(info)) if !info.runways.is_empty() => Ok((icao, info)),
        _ => Err(format!("{} has no runways I can read.", crate::flightplan::spell_code(&icao))),
    }
}

pub struct SetUpApproach {
    /// The autopilot and approach speed autoland flies with, from the
    /// registry's own autoland; `None` on an airframe without an autopilot,
    /// where the new flight is the pilot's.
    pub land: Option<(crate::tools::autopilot::ApConfig, f64)>,
}

#[async_trait]
impl Tool for SetUpApproach {
    fn name(&self) -> &str {
        "setup_approach"
    }
    fn description(&self) -> &str {
        "Start a new flight on a runway final with this aircraft, engines running, and \
         take the aircraft (autoland onto that runway) unless the pilot will fly it. For \
         'put us on a 10 mile final to LEMG 13', 'set us up on final for runway one three', \
         'put us on approach, my controls'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "final_nm": {"type": "integer"},
                "runway": {"type": "string"},
                "to": {"type": "string"},
                "to_codes": {"type": "array", "items": {"type": "string"}},
                "hand_fly": {"type": "boolean"}
            },
            "additionalProperties": false
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: ApproachArgs = serde_json::from_value(args.clone()).unwrap_or_default();
        let final_nm = a.final_nm.unwrap_or(DEFAULT_FINAL_NM);
        if !(1..=MAX_FINAL_NM).contains(&final_nm) {
            return Ok(format!("A final between one and {MAX_FINAL_NM} miles, please."));
        }
        if ctx
            .xplane
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5)
        {
            return Ok("We're in replay. Go back to live and ask again.".into());
        }
        if let Some(who) = flying_now() {
            return Ok(format!("{who} is flying. Say my controls first, then ask again."));
        }
        // The string read is the concrete client's (the tool trait reads numbers).
        let strings = crate::xplane::XPlaneClient::new(ctx.config.xplane_base_url.clone());
        let (ground, lat, lon, acf) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY),
            ctx.xplane.get_float("sim/flightmodel/position/latitude"),
            ctx.xplane.get_float("sim/flightmodel/position/longitude"),
            strings.get_string("sim/aircraft/view/acf_relative_path"),
        );
        // In the air a bare "put us on final" could be words about the
        // circuit being flown; a new flight there needs a distance, a runway
        // or a field said with it.
        let explicit = a.final_nm.is_some() || a.runway.is_some() || a.to.is_some();
        if !ground.is_ok_and(|g| g > 0.5) && !explicit {
            return Ok(
                "That starts a new flight on a final. Say the distance or the runway if that's what you want."
                    .into(),
            );
        }
        let (Ok(lat), Ok(lon)) = (lat, lon) else {
            return Ok("I can't read where we are.".into());
        };
        let Some(acf) = acf.ok().filter(|p| !p.trim().is_empty()) else {
            return Ok("I can't read which aircraft is loaded.".into());
        };
        let apt = ctx.config.apt_data_path.clone();
        let resolved = {
            let (apt, a) = (apt.clone(), a.clone());
            tokio::task::spawn_blocking(move || resolve_field(&apt, &a, (lat, lon))).await?
        };
        let (icao, info) = match resolved {
            Ok(x) => x,
            Err(line) => return Ok(line),
        };
        // The tower's runway only when its station is this field.
        let tower = crate::station::active_icao()
            .filter(|(i, _, _)| i.eq_ignore_ascii_case(&icao))
            .and_then(|_| crate::party_line::clearance_runway());
        let plan = crate::simbrief::loaded_plan()
            .filter(|p| p.destination.eq_ignore_ascii_case(&icao))
            .and_then(|p| p.procedures.dest_rwy);
        let wind = crate::weather::surface_wind(&super::autoland::xplane_root(), &icao);
        let (rwy, source) = match pick_runway(
            &icao,
            &info.runways,
            a.runway.as_deref(),
            tower.as_deref(),
            plan.as_deref(),
            wind,
        ) {
            Ok(x) => x,
            Err(line) => return Ok(line),
        };
        let body = approach_init_json(&acf, &icao, &rwy, final_nm);
        crate::blackbox::record(
            "setup",
            json!({ "event": "approach_init", "acf": acf, "airport": icao, "runway": rwy,
                    "runway_from": source, "final_nm": final_nm, "hand_fly": a.hand_fly,
                    "json": body }),
        );
        // The new flight ends this one: nothing may keep flying it.
        crate::flight_state::stand_down();
        let job = ApproachJob {
            base: ctx.config.xplane_base_url.clone(),
            apt,
            voice: ctx.voice.cloned(),
            body,
            icao: icao.clone(),
            runway: rwy.clone(),
            final_nm,
            hand_fly: a.hand_fly,
            land: self.land.clone(),
            field: info.position(),
        };
        // Its own thread and runtime, like every loop that must keep time:
        // the command loop's runtime is current-thread and blocks on the
        // microphone between passes, so a task spawned on it runs only in the
        // slivers where the loop awaits. At LEMG (2026-09-26) the first setup
        // never wrote the mailbox, and the second wrote it (the sim
        // repositioned 6 s later) but never read the result: no arm, no line,
        // not even the timeout, and the 737 dived with nobody flying.
        spawn_off_loop(approach_follow_up(job));
        Ok(format!(
            "Setting up a {final_nm} mile final for runway {} at {}.",
            crate::circuit::spell_runway(&rwy),
            crate::flightplan::spell_code(&icao)
        ))
    }
}

struct ApproachJob {
    base: String,
    apt: String,
    voice: Option<Voice>,
    body: String,
    icao: String,
    runway: String,
    final_nm: i64,
    hand_fly: bool,
    land: Option<(crate::tools::autopilot::ApConfig, f64)>,
    field: Option<(f64, f64)>,
}

/// Run a future to completion on its own thread and runtime, the way
/// `run_autoland` and every loop run. The command loop's runtime is
/// current-thread and blocks synchronously on the microphone between passes,
/// so a `tokio::spawn` from a tool starves (`speedhold`'s governor sat
/// unpolled 23 s; the approach setup never read its result). Pinned by
/// `no_tool_leaves_work_on_the_command_loops_runtime`.
pub fn spawn_off_loop<F>(fut: F)
where
    F: std::future::Future<Output = ()> + Send + 'static,
{
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
            return;
        };
        rt.block_on(fut);
    });
}

/// How long a sortie waits for its loop to start, and how often it looks.
#[derive(Debug, Clone, Copy)]
pub struct SortieHold {
    pub start_within: std::time::Duration,
    pub poll: std::time::Duration,
}

impl SortieHold {
    pub const LIVE: SortieHold = SortieHold {
        start_within: std::time::Duration::from_secs(10),
        poll: std::time::Duration::from_millis(250),
    };
}

/// Keep a sortie open while `flag` (a loop's ACTIVE) is up: it ends when
/// the loop lets go, or when the loop never started. The flag, not the
/// thread, because a re-armed autoland hands ACTIVE to its successor and
/// the landing it flies is still this sortie's.
pub fn hold_sortie_while(
    flag: &'static std::sync::atomic::AtomicBool,
    sortie: crate::full_auto::SortieGuard,
    t: SortieHold,
) -> std::thread::JoinHandle<()> {
    use std::sync::atomic::Ordering::SeqCst;
    std::thread::spawn(move || {
        let t0 = std::time::Instant::now();
        while !flag.load(SeqCst) && t0.elapsed() < t.start_within {
            std::thread::sleep(t.poll);
        }
        while flag.load(SeqCst) {
            std::thread::sleep(t.poll);
        }
        drop(sortie);
    })
}

/// The flight-initialisation mailbox as the follow-up sees it: numbers read,
/// one JSON posted. The live one is the Web API; the tests fake a reload.
#[async_trait]
pub trait InitMailbox: Send + Sync {
    async fn read(&self, name: &str) -> Result<f64>;
    async fn post(&self, body: &[u8]) -> Result<()>;
}

#[async_trait]
impl InitMailbox for crate::xplane::XPlaneClient {
    async fn read(&self, name: &str) -> Result<f64> {
        self.get_float(name).await
    }
    async fn post(&self, body: &[u8]) -> Result<()> {
        self.set_data(INIT_JSON, body).await
    }
}

/// How the mailbox wait keeps time.
#[derive(Debug, Clone, Copy)]
pub struct InitTiming {
    /// Between polls: the most the arm can trail the flight's existence.
    pub poll: std::time::Duration,
    /// One read's cap. The client's own is 8 s, and a read that hangs through
    /// the sim's reload must not hold the arm back: it is abandoned and the
    /// next poll asks again.
    pub read_timeout: std::time::Duration,
    /// No answer by then is a timeout, said.
    pub deadline: std::time::Duration,
    /// How often the wait reports what it sees (blackbox `approach_wait`).
    pub report_every: std::time::Duration,
}

impl InitTiming {
    pub const LIVE: InitTiming = InitTiming {
        poll: std::time::Duration::from_millis(150),
        read_timeout: std::time::Duration::from_millis(600),
        deadline: std::time::Duration::from_secs(120),
        report_every: std::time::Duration::from_secs(2),
    };
    pub const RAMP: InitTiming = InitTiming {
        poll: std::time::Duration::from_millis(500),
        read_timeout: std::time::Duration::from_secs(2),
        deadline: std::time::Duration::from_secs(120),
        report_every: std::time::Duration::from_secs(2),
    };
}

/// How a mailbox run ended.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InitOutcome {
    /// The plugin ran it: `XPLMInitResult` (0 = the flight exists).
    Done(i64),
    /// The write did not reach the plugin.
    Unreachable,
    /// The completion counter could not be read before the write, so no
    /// answer could be told from the last flight's: nothing was written.
    NoBaseline,
    /// Written, and no answer by the deadline.
    TimedOut,
}

impl InitOutcome {
    pub fn name(&self) -> &'static str {
        match self {
            InitOutcome::Done(0) => "ok",
            InitOutcome::Done(_) => "refused",
            InitOutcome::Unreachable => "unreachable",
            InitOutcome::NoBaseline => "no_baseline",
            InitOutcome::TimedOut => "timeout",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitRun {
    pub outcome: InitOutcome,
    /// Reads that failed or were abandoned (the sim mid-reload).
    pub read_errors: u32,
    pub polls: u32,
}

/// What the wait sees, reported every `report_every`: the last good reads
/// (`None` = that read failed) against the baseline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitWait {
    pub seq0: f64,
    pub seq: Option<f64>,
    pub result: Option<f64>,
    pub read_errors: u32,
    pub polls: u32,
}

/// Post the JSON and wait for the plugin's answer. A failed or hung read is
/// no answer and never the end of the wait: the sim reloading the aircraft
/// answers the Web API with errors for a while, and the result is the next
/// good read. The baseline counter is read FIRST (retried): without it the
/// last flight's pair (seq 0, result 0) reads as success before this one ran.
pub async fn run_init(
    mb: &dyn InitMailbox,
    body: &[u8],
    t: InitTiming,
    report: &(dyn Fn(InitWait) + Sync),
) -> InitRun {
    let read = |name: &'static str| async move {
        match tokio::time::timeout(t.read_timeout, mb.read(name)).await {
            Ok(Ok(v)) => Some(v),
            _ => None,
        }
    };
    let mut run = InitRun { outcome: InitOutcome::TimedOut, read_errors: 0, polls: 0 };
    let mut seq0 = None;
    for _ in 0..10 {
        match read(INIT_SEQ).await {
            Some(v) => {
                seq0 = Some(v);
                break;
            }
            None => {
                run.read_errors += 1;
                tokio::time::sleep(t.poll).await;
            }
        }
    }
    let Some(seq0) = seq0 else {
        run.outcome = InitOutcome::NoBaseline;
        return run;
    };
    let posted = tokio::time::timeout(std::time::Duration::from_secs(5), mb.post(body)).await;
    if !matches!(posted, Ok(Ok(()))) {
        run.outcome = InitOutcome::Unreachable;
        return run;
    }
    let t0 = tokio::time::Instant::now();
    let mut reported = t0;
    while t0.elapsed() < t.deadline {
        tokio::time::sleep(t.poll).await;
        run.polls += 1;
        let (seq, res) = tokio::join!(read(INIT_SEQ), read(INIT_RESULT));
        if let (Some(seq), Some(res)) = (seq, res) {
            if seq > seq0 && res >= 0.0 {
                run.outcome = InitOutcome::Done(res as i64);
                return run;
            }
        }
        run.read_errors += u32::from(seq.is_none()) + u32::from(res.is_none());
        if reported.elapsed() >= t.report_every {
            reported = tokio::time::Instant::now();
            report(InitWait { seq0, seq, result: res, read_errors: run.read_errors, polls: run.polls });
        }
    }
    run
}

async fn approach_follow_up(job: ApproachJob) {
    let xp = crate::xplane::XPlaneClient::new(job.base.clone());
    let say = |line: &str| {
        if let Some(v) = &job.voice {
            v.alert(Priority::Caution, line.to_string());
        }
    };
    let t0 = std::time::Instant::now();
    let run = run_init(&xp, job.body.as_bytes(), InitTiming::LIVE, &|w| {
        crate::blackbox::record(
            "setup",
            json!({ "event": "approach_wait", "seq": w.seq, "result": w.result, "seq0": w.seq0,
                    "read_errors": w.read_errors, "polls": w.polls,
                    "ms": t0.elapsed().as_millis() as u64 }),
        );
    })
    .await;
    let seen = std::time::Instant::now();
    let result = match run.outcome {
        InitOutcome::Done(code) => Some(code),
        _ => None,
    };
    let next = match run.outcome {
        InitOutcome::Unreachable | InitOutcome::NoBaseline => AfterInit::Failed {
            line: "I couldn't reach the plugin to set up the approach. Is it up to date?".into(),
        },
        _ => after_init(
            result,
            &job.icao,
            &job.runway,
            job.final_nm,
            job.hand_fly,
            job.land.is_some(),
        ),
    };
    // The arm first, the record and the words after: the aircraft is
    // already flying.
    if let AfterInit::Arm { runway, .. } = &next {
        if let Some((cfg, ias)) = job.land.clone() {
            // The landing is a sortie: one file and one verdict for the
            // sortie runner, begun before autoland's first tick.
            let sortie = crate::full_auto::SortieGuard::begin(
                "approach",
                json!({ "airport": job.icao, "runway": job.runway, "final_nm": job.final_nm }),
            );
            hold_sortie_while(&crate::autoland::ACTIVE, sortie, SortieHold::LIVE);
            crate::autoland::ABORT.store(false, std::sync::atomic::Ordering::SeqCst);
            crate::autoland::run_autoland(
                job.base.clone(),
                job.apt.clone(),
                cfg,
                ias,
                job.voice.clone(),
                None,
                Some(runway.clone()),
            );
            crate::flight::note_crew_takeover("setup_approach");
        }
    }
    // Always on file and always said: a setup that ends in silence is the
    // one the pilot cannot act on. `arm_ms` is result-seen to armed.
    crate::blackbox::record(
        "setup",
        json!({ "event": "approach_result", "outcome": run.outcome.name(), "result": result,
                "armed": matches!(next, AfterInit::Arm { .. }),
                "arm_ms": seen.elapsed().as_millis() as u64,
                "read_errors": run.read_errors, "polls": run.polls,
                "ms": t0.elapsed().as_millis() as u64 }),
    );
    match &next {
        AfterInit::Arm { line, .. } | AfterInit::Monitor { line } | AfterInit::Failed { line } => {
            say(line)
        }
    }
    if matches!(next, AfterInit::Failed { .. }) || !crate::mode::atc_enabled() {
        return;
    }
    new_flight_radio(&xp, &job).await;
}

/// The tower hears a NEW arrival: its station for this field is rebuilt from
/// nothing, COM1 goes to it, the field's own wind is published, and once the
/// new station is up the final report goes out (the landing is cleared; at
/// LEMG it earned "you landed without clearance").
async fn new_flight_radio(xp: &crate::xplane::XPlaneClient, job: &ApproachJob) {
    let Some(khz) = crate::station::arm_new_flight(&job.icao, std::time::Instant::now()) else {
        crate::blackbox::record(
            "setup",
            json!({ "event": "approach_radio", "icao": job.icao, "tower": null }),
        );
        return;
    };
    let _ = xp.set_float(crate::datarefs::COM1_FREQ_KHZ, khz as f64).await;
    if let Some((lat, lon)) = job.field {
        let wind = crate::weather::surface_wind(&super::autoland::xplane_root(), &job.icao);
        crate::station::set_surface_wind(lat, lon, wind);
    }
    // The call belongs to the NEW station: routed a pass early it reaches
    // the old one and its worksheet.
    let t0 = std::time::Instant::now();
    while t0.elapsed().as_secs() < 15
        && (crate::party_line::station_swap_pending()
            || crate::party_line::home_tower_khz() != Some(khz))
    {
        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
    }
    let up = crate::party_line::home_tower_khz() == Some(khz);
    let callsign = crate::circuit::crew_callsign(xp).await;
    let words = format!(
        "{} mile final runway {}",
        job.final_nm,
        crate::circuit::spell_runway(&job.runway)
    );
    if up {
        crate::circuit::transmit(crate::circuit::radio_call(&callsign, &words));
    }
    crate::blackbox::record(
        "setup",
        json!({ "event": "approach_radio", "icao": job.icao, "tower": khz,
                "station_up": up, "call": if up { Some(words) } else { None } }),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_approach_sortie_lasts_as_long_as_the_landing() {
        use std::sync::atomic::{AtomicBool, Ordering::SeqCst};
        static FLYING: AtomicBool = AtomicBool::new(false);
        let t = SortieHold {
            start_within: std::time::Duration::from_millis(500),
            poll: std::time::Duration::from_millis(5),
        };
        let guard = crate::full_auto::SortieGuard::begin("approach_test", json!({}));
        let id = crate::blackbox::sortie().expect("begun");
        let h = hold_sortie_while(&FLYING, guard, t);
        // The loop comes up late (its own thread), then flies a while.
        std::thread::sleep(std::time::Duration::from_millis(50));
        FLYING.store(true, SeqCst);
        std::thread::sleep(std::time::Duration::from_millis(600));
        assert_eq!(crate::blackbox::sortie(), Some(id), "open while the loop flies");
        FLYING.store(false, SeqCst);
        h.join().unwrap();
        assert!(crate::blackbox::sortie().is_none(), "closed when it lets go");
        // A loop that never started does not hold a sortie open.
        let guard = crate::full_auto::SortieGuard::begin("approach_test", json!({}));
        hold_sortie_while(&FLYING, guard, t).join().unwrap();
        assert!(crate::blackbox::sortie().is_none());
    }

    #[test]
    fn an_acf_declares_its_type() {
        let text = "I\n1200 Version\nP acf/_name Boeing 737-800\nP acf/_ICAO b738\n";
        assert_eq!(acf_icao(text), Some("B738".into()));
        assert_eq!(acf_icao("P acf/_name x\n"), None);
    }

    #[test]
    fn the_plans_type_finds_the_laminar_aircraft_first() {
        let lib = vec![
            ("B738".to_string(), "Aircraft/Zibo/B737-800X/b738_4k.acf".to_string()),
            ("B738".to_string(), "Aircraft/Laminar Research/Boeing 737-800/b738.acf".to_string()),
            ("C172".to_string(), "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf".to_string()),
        ];
        assert_eq!(find_acf(&lib, "b738"), Some("Aircraft/Laminar Research/Boeing 737-800/b738.acf"));
        assert_eq!(find_acf(&lib, "A320"), None);
    }

    #[test]
    fn a_jet_gets_a_gate_that_takes_jets_and_a_prop_a_tie_down() {
        let st = |name: &str, kind: &str, accepts: &[&str]| crate::nav::Stand {
            lat: 0.0,
            lon: 0.0,
            heading: 0.0,
            name: name.into(),
            kind: kind.into(),
            accepts: accepts.iter().map(|s| s.to_string()).collect(),
        };
        let stands = vec![
            st("H1", "hangar", &["jets"]),
            st("GA 2", "tie_down", &["props"]),
            st("B4", "gate", &["jets", "heavy"]),
            st("A1", "gate", &["props"]),
        ];
        assert_eq!(pick_stand(&stands, stand_class(true, false), true).unwrap().name, "B4");
        assert_eq!(pick_stand(&stands, stand_class(false, false), false).unwrap().name, "GA 2");
    }

    fn rwy(a: &str, b: &str, from: (f64, f64), to: (f64, f64), surface: u32) -> crate::nav::Runway {
        crate::nav::Runway {
            ident_a: a.into(),
            ident_b: b.into(),
            lat_a: from.0,
            lon_a: from.1,
            lat_b: to.0,
            lon_b: to.1,
            width_m: 45.0,
            surface,
        }
    }

    /// Malaga's two strips, as apt.dat spells them (headings about 130/310
    /// and 120/300).
    fn lemg() -> Vec<crate::nav::Runway> {
        vec![
            rwy("13", "31", (36.700, -4.510), (36.680, -4.480), 1),
            rwy("12", "30", (36.690, -4.525), (36.672, -4.490), 1),
        ]
    }

    #[test]
    fn the_approach_init_is_this_aircraft_on_a_final_with_engines_running() {
        // The JSON driven by hand at LEMG on 2026-09-26 (seq 1, result 0).
        let body = approach_init_json(
            "Aircraft/Laminar Research/Boeing 737-800/b738.acf",
            "LEMG",
            "13",
            10,
        );
        let v: Value = serde_json::from_str(&body).unwrap();
        assert_eq!(v["aircraft"]["path"], "Aircraft/Laminar Research/Boeing 737-800/b738.acf");
        assert_eq!(v["runway_start"]["airport_id"], "LEMG");
        assert_eq!(v["runway_start"]["runway"], "13");
        assert_eq!(v["runway_start"]["final_distance_in_nautical_miles"], 10);
        assert_eq!(v["engine_status"]["all_engines"]["running"], true);
        // Never a ramp start, never cold and dark.
        assert!(v.get("ramp_start").is_none());
        assert!(body.contains("\"final_distance_in_nautical_miles\":10"), "{body}");
    }

    #[test]
    fn a_refused_setup_names_the_reason() {
        assert_eq!(
            init_refusal(4, "LEMG", "13"),
            "The approach setup didn't take: X-Plane doesn't know the airport L E M G."
        );
        assert_eq!(
            init_refusal(6, "LEMG", "13"),
            "The approach setup didn't take: X-Plane has no runway one three at L E M G."
        );
        assert_eq!(
            init_refusal(2, "LEMG", "13"),
            "The approach setup didn't take: X-Plane couldn't load this aircraft."
        );
        assert!(init_refusal(9, "LEMG", "13").contains("code 9"));
    }

    #[test]
    fn the_copilot_takes_the_new_flight_on_the_runway_it_was_started_on() {
        // LEMG 2026-09-26: forty seconds with nobody flying. On result 0 the
        // arm is autoland FORCED onto the field and runway of the new final
        // (the arrival's `PlannedRunway`, never a proximity self-pick).
        assert_eq!(
            after_init(Some(0), "LEMG", "13", 10, false, true),
            AfterInit::Arm {
                runway: crate::autoland::PlannedRunway::at("LEMG", "13"),
                line: "On a 10 mile final for one three, I have the aircraft.".into(),
            }
        );
        // "...my controls": monitoring only.
        assert_eq!(
            after_init(Some(0), "LEMG", "13", 5, true, true),
            AfterInit::Monitor {
                line: "On a 5 mile final for one three. Your aircraft, I'm monitoring.".into()
            }
        );
        // No autopilot to land with: said, never pretended.
        assert!(matches!(
            after_init(Some(0), "LEMG", "13", 10, false, false),
            AfterInit::Monitor { .. }
        ));
        // A refusal arms nothing.
        assert_eq!(
            after_init(Some(6), "LEMG", "13", 10, false, true),
            AfterInit::Failed { line: init_refusal(6, "LEMG", "13") }
        );
        assert!(matches!(
            after_init(None, "LEMG", "13", 10, false, true),
            AfterInit::Failed { .. }
        ));
    }

    #[test]
    fn the_runway_is_the_pilots_then_the_towers_then_the_plans_then_the_winds() {
        let r = lemg();
        assert_eq!(
            pick_runway("LEMG", &r, Some("13"), Some("12"), Some("30"), None).unwrap(),
            ("13".into(), "asked")
        );
        assert_eq!(
            pick_runway("LEMG", &r, None, Some("12"), Some("30"), Some((310.0, 15.0))).unwrap(),
            ("12".into(), "tower")
        );
        assert_eq!(
            pick_runway("LEMG", &r, None, None, Some("30"), Some((130.0, 15.0))).unwrap(),
            ("30".into(), "plan")
        );
        let (end, from) = pick_runway("LEMG", &r, None, None, None, Some((310.0, 15.0))).unwrap();
        assert!(end == "31" || end == "30", "{end}");
        assert_eq!(from, "wind");
        // A tower or plan runway the field lacks falls through, never refuses.
        assert_eq!(
            pick_runway("LEMG", &r, None, Some("27"), None, None).unwrap().1,
            "longest"
        );
        // "3" is "03", "13" is not "13L".
        assert!(same_end("03", "3"));
        assert!(!same_end("13", "13L"));
    }

    #[test]
    fn a_runway_the_field_lacks_is_refused_with_the_ones_it_has() {
        let err = pick_runway("LEMG", &lemg(), Some("14"), None, None, None).unwrap_err();
        assert_eq!(
            err,
            "L E M G has no runway one four. It has one three, three one, one two, three zero."
        );
    }

    /// The mailbox through a reload: reads fail (or hang) for a while after
    /// the post, then the plugin's answer is there.
    struct Reloading {
        failing: std::sync::atomic::AtomicU32,
        hanging: std::sync::atomic::AtomicU32,
        /// Result reads after the post that still show "pending".
        pending: std::sync::atomic::AtomicU32,
        posted: std::sync::atomic::AtomicBool,
        answer: i64,
        post_fails: bool,
    }

    impl Reloading {
        fn new(failing: u32, hanging: u32, pending: u32, answer: i64) -> Self {
            Reloading {
                failing: failing.into(),
                hanging: hanging.into(),
                pending: pending.into(),
                posted: false.into(),
                answer,
                post_fails: false,
            }
        }
    }

    #[async_trait]
    impl InitMailbox for Reloading {
        async fn read(&self, name: &str) -> Result<f64> {
            use std::sync::atomic::Ordering::SeqCst;
            let posted = self.posted.load(SeqCst);
            if posted && self.hanging.load(SeqCst) > 0 {
                self.hanging.fetch_sub(1, SeqCst);
                tokio::time::sleep(std::time::Duration::from_secs(30)).await;
            }
            if posted && self.failing.load(SeqCst) > 0 {
                self.failing.fetch_sub(1, SeqCst);
                anyhow::bail!("503 mid-reload");
            }
            let done = posted && self.pending.load(SeqCst) == 0;
            if posted && !done && name == INIT_RESULT {
                self.pending.fetch_sub(1, SeqCst);
            }
            Ok(match (name, posted, done) {
                (INIT_SEQ, _, true) => 1.0,
                (INIT_SEQ, _, false) => 0.0,
                (_, _, true) => self.answer as f64,
                (_, true, false) => -1.0,
                // The last flight's answer: success, seq 0.
                _ => 0.0,
            })
        }
        async fn post(&self, _body: &[u8]) -> Result<()> {
            if self.post_fails {
                anyhow::bail!("no plugin");
            }
            self.posted.store(true, std::sync::atomic::Ordering::SeqCst);
            Ok(())
        }
    }

    const FAST: InitTiming = InitTiming {
        poll: std::time::Duration::from_millis(5),
        read_timeout: std::time::Duration::from_millis(40),
        deadline: std::time::Duration::from_secs(2),
        report_every: std::time::Duration::from_millis(20),
    };

    fn quiet(_: InitWait) {}

    #[tokio::test]
    async fn the_result_is_read_through_a_reload_that_answers_errors() {
        // A failed read is one more poll, never the end of the wait.
        let mb = Reloading::new(12, 0, 3, 0);
        let run = run_init(&mb, b"{}", FAST, &quiet).await;
        assert_eq!(run.outcome, InitOutcome::Done(0));
        assert!(run.read_errors >= 12, "{run:?}");
    }

    #[tokio::test]
    async fn a_read_that_hangs_through_the_reload_does_not_hold_the_arm_back() {
        let mb = Reloading::new(0, 4, 1, 0);
        let t0 = std::time::Instant::now();
        let run = run_init(&mb, b"{}", FAST, &quiet).await;
        assert_eq!(run.outcome, InitOutcome::Done(0));
        assert!(t0.elapsed() < std::time::Duration::from_secs(1), "{:?}", t0.elapsed());
        // Live: a hung read plus a poll is under a second.
        assert!(
            (InitTiming::LIVE.read_timeout + InitTiming::LIVE.poll)
                < std::time::Duration::from_secs(1)
        );
    }

    #[tokio::test]
    async fn the_last_flights_answer_is_never_this_ones_and_a_stuck_wait_reports() {
        // Before the post the mailbox reads seq 0, result 0: success from
        // the flight before. Never taken; the wait says what it sees.
        let mb = Reloading::new(0, 0, u32::MAX, 0);
        let seen = std::sync::Mutex::new(Vec::new());
        let t = InitTiming { deadline: std::time::Duration::from_millis(150), ..FAST };
        let run = run_init(&mb, b"{}", t, &|w| seen.lock().unwrap().push(w)).await;
        assert_eq!(run.outcome, InitOutcome::TimedOut);
        let seen = seen.into_inner().unwrap();
        assert!(seen.len() >= 3, "{}", seen.len());
        assert_eq!(seen[0].seq0, 0.0);
        assert_eq!(seen[0].seq, Some(0.0));
        assert_eq!(seen[0].result, Some(-1.0), "pending while the plugin has it");
    }

    #[tokio::test]
    async fn a_refusal_and_an_unreachable_plugin_are_outcomes() {
        let mb = Reloading::new(0, 0, 0, 6);
        assert_eq!(run_init(&mb, b"{}", FAST, &quiet).await.outcome, InitOutcome::Done(6));
        let mut dead = Reloading::new(0, 0, 0, 0);
        dead.post_fails = true;
        let run = run_init(&dead, b"{}", FAST, &quiet).await;
        assert_eq!(run.outcome.name(), "unreachable");
        assert_eq!(InitOutcome::TimedOut.name(), "timeout");
        assert_eq!(InitOutcome::Done(0).name(), "ok");
        assert_eq!(InitOutcome::Done(6).name(), "refused");
    }

    #[test]
    fn a_follow_up_runs_while_the_command_loop_blocks() {
        // The live failure reproduced: on a current-thread runtime that
        // blocks (the command loop on the mic), a `tokio::spawn`ed task does
        // not run; one on its own thread does.
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        let starved = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let ran = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        rt.block_on(async {
            let s = starved.clone();
            tokio::spawn(async move { s.store(true, std::sync::atomic::Ordering::SeqCst) });
            let r = ran.clone();
            spawn_off_loop(async move { r.store(true, std::sync::atomic::Ordering::SeqCst) });
            std::thread::sleep(std::time::Duration::from_millis(300));
            assert!(!starved.load(std::sync::atomic::Ordering::SeqCst));
            assert!(ran.load(std::sync::atomic::Ordering::SeqCst), "the follow-up starved");
        });
    }

    #[test]
    fn no_tool_leaves_work_on_the_command_loops_runtime() {
        // Tools run on the command loop's current-thread runtime, which
        // blocks on the microphone: work that must outlive the call goes on
        // `spawn_off_loop` (or its own thread), never `tokio::spawn`. Test
        // modules may spawn (a test runtime is theirs); comments may name it.
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/tools");
        let mut offenders = Vec::new();
        for e in std::fs::read_dir(&dir).unwrap().flatten() {
            let p = e.path();
            if p.extension().is_none_or(|x| x != "rs") {
                continue;
            }
            let text = std::fs::read_to_string(&p).unwrap();
            let live = text.split("#[cfg(test)]").next().unwrap_or("");
            for (i, line) in live.lines().enumerate() {
                let code = line.split("//").next().unwrap_or("");
                if code.contains("tokio::spawn(") {
                    offenders.push(format!("{}:{}", p.display(), i + 1));
                }
            }
        }
        assert!(offenders.is_empty(), "tokio::spawn from a tool starves: {offenders:?}");
    }

    #[test]
    fn the_init_is_the_aircraft_at_the_ramp_cold_and_dark() {
        let v: Value = serde_json::from_str(&init_json("Aircraft/x.acf", "LIBP", "Stand 1")).unwrap();
        assert_eq!(v["aircraft"]["path"], "Aircraft/x.acf");
        assert_eq!(v["ramp_start"]["airport_id"], "LIBP");
        assert_eq!(v["ramp_start"]["ramp"], "Stand 1");
        assert_eq!(v["engine_status"]["all_engines"]["running"], false);
    }
}
