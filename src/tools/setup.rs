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
        // Jet or not, from the profile the type maps to; an unknown type
        // takes any suitable stand.
        let known = crate::detect::known_profile(&fp.aircraft).and_then(|id| crate::aircraft::load(&id));
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
        tokio::spawn(async move {
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
    let seq0 = xp.get_float(INIT_SEQ).await.unwrap_or(-1.0);
    if xp.set_data(INIT_JSON, body.as_bytes()).await.is_err() {
        say("I couldn't reach the plugin to set up the flight. Is it up to date?");
        return;
    }
    // The plugin runs it on its next flight loop; the reload itself can take
    // a minute with scenery to load.
    let t0 = std::time::Instant::now();
    let result = loop {
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        if t0.elapsed().as_secs() > 120 {
            break None;
        }
        let (seq, res) = tokio::join!(xp.get_float(INIT_SEQ), xp.get_float(INIT_RESULT));
        if let (Ok(seq), Ok(res)) = (seq, res) {
            if seq > seq0 && res >= 0.0 {
                break Some(res as i64);
            }
        }
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

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn the_init_is_the_aircraft_at_the_ramp_cold_and_dark() {
        let v: Value = serde_json::from_str(&init_json("Aircraft/x.acf", "LIBP", "Stand 1")).unwrap();
        assert_eq!(v["aircraft"]["path"], "Aircraft/x.acf");
        assert_eq!(v["ramp_start"]["airport_id"], "LIBP");
        assert_eq!(v["ramp_start"]["ramp"], "Stand 1");
        assert_eq!(v["engine_status"]["all_engines"]["running"], false);
    }
}
