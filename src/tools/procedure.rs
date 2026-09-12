//! `fly_procedure` — fly a published SID or instrument approach from the sim's
//! CIFP, the terrain-safe alternative to the invented circuit pattern.
//!
//! The arming end of [`crate::procedure`]. It resolves the real procedure for
//! the runway in use — reading X-Plane's `CIFP/<ICAO>.dat`, `earth_fix.dat` and
//! `earth_nav.dat` — and hands the follower a leg list with every fix placed.
//! The whole point of pulling from real data: at a field with NO published
//! procedure (Gibraltar) this REFUSES with the honest reason, instead of the
//! circuit guessing a path into the Rock or the water.
//!
//! - "approach" (default): the best instrument approach to the landing runway,
//!   handed to `autoland` at the final approach fix.
//! - "departure": the SID off the runway we're lined up on / heading down,
//!   flown to its last fix (airborne — take off first).

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::nav::{self, ProcKind};
use crate::procedure::{self, EndAction, Plan};
use crate::tools::autopilot::ApConfig;

pub struct FlyProcedure {
    pub cfg: ApConfig,
    pub approach_ias: f64,
    /// Best-climb reference for the follower's speed-protected climb law.
    pub vy_kts: f64,
}

#[derive(Deserialize, Default)]
struct Args {
    #[serde(default)]
    off: bool,
    /// "approach" (default) or "departure".
    #[serde(default)]
    kind: String,
    /// Force a specific runway ("27R"); otherwise the runway is chosen from
    /// position and heading.
    #[serde(default)]
    runway: Option<String>,
}

#[async_trait]
impl Tool for FlyProcedure {
    fn name(&self) -> &str {
        "fly_procedure"
    }

    fn description(&self) -> &str {
        "Fly a REAL published instrument procedure from the chart data — an ILS/RNAV \
         approach or a SID — instead of a guessed pattern. Use for 'fly the approach', \
         'fly the ILS', 'fly me in', 'fly the arrival', 'shoot the approach', 'fly \
         the departure', 'fly the SID'. This threads the airport's own terrain-safe \
         corridor, so prefer it over 'autoland' near terrain. kind='departure' for a \
         SID, otherwise the approach. off=true to cancel. Refuses at fields with no \
         published procedure."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "off": {"type": "boolean", "description": "true = cancel the procedure"},
                "kind": {"type": "string", "enum": ["approach", "departure"],
                         "description": "approach (default) or departure/SID"},
                "runway": {"type": "string", "description": "force a runway, e.g. '27R'"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        use std::sync::atomic::Ordering::SeqCst;
        let a: Args = serde_json::from_value(args.clone()).unwrap_or_default();
        if a.off {
            return Ok(if procedure::abort() {
                "Procedure cancelled. You have the aircraft.".to_string()
            } else {
                "No procedure running.".to_string()
            });
        }
        if procedure::ACTIVE.load(SeqCst) {
            return Ok(
                "I'm already flying a procedure. Say cancel first to start over.".to_string(),
            );
        }
        if ctx
            .xplane
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5)
        {
            return Ok(
                "We're in replay, nothing to fly. Go back to live and ask again.".to_string(),
            );
        }

        let (Ok(lat), Ok(lon)) = (
            ctx.xplane
                .get_float("sim/flightmodel/position/latitude")
                .await,
            ctx.xplane
                .get_float("sim/flightmodel/position/longitude")
                .await,
        ) else {
            return Ok("I can't read our position, so I can't set up a procedure.".to_string());
        };
        let track = ctx
            .xplane
            .get_float("sim/flightmodel/position/psi")
            .await
            .unwrap_or(0.0);

        let departure =
            a.kind.eq_ignore_ascii_case("departure") || a.kind.eq_ignore_ascii_case("sid");
        let apt_dat = ctx.config.apt_data_path.clone();
        let cifp = ctx.config.cifp_dir.clone();
        let fix_path = ctx.config.fix_data_path.clone();
        let nav_path = ctx.config.nav_data_path.clone();
        let forced_rwy = a.runway.clone();
        let cfg = self.cfg.clone();
        let approach_ias = self.approach_ias;
        let vy_kts = self.vy_kts;

        // Everything below reads big data files — off the async loop.
        let built = tokio::task::spawn_blocking(move || {
            build_plan(
                &apt_dat,
                &cifp,
                &fix_path,
                &nav_path,
                lat,
                lon,
                track,
                departure,
                forced_rwy,
                cfg,
                approach_ias,
                vy_kts,
            )
        })
        .await
        .unwrap_or(Err(
            "Something went wrong reading the chart data.".to_string()
        ));

        let (plan, ack) = match built {
            Ok(v) => v,
            Err(why) => return Ok(why),
        };
        procedure::start(ctx.config.xplane_base_url.clone(), ctx.voice.cloned(), plan);
        Ok(ack)
    }
}

/// Pick the runway, load its procedure, resolve every fix, and build the ready-
/// to-fly [`Plan`] plus the spoken acknowledgement — or an honest refusal string.
#[allow(clippy::too_many_arguments)]
fn build_plan(
    apt_dat: &str,
    cifp: &str,
    fix_path: &str,
    nav_path: &str,
    lat: f64,
    lon: f64,
    track: f64,
    departure: bool,
    forced_rwy: Option<String>,
    cfg: ApConfig,
    approach_ias: f64,
    vy_kts: f64,
) -> Result<(Plan, String), String> {
    // The runway in use: forced, aligned (departure), or best-for-landing.
    let rwy = if departure {
        let cands = crate::autoland::nearby_runways(apt_dat, lat, lon, 3.0).unwrap_or_default();
        crate::autotakeoff::runway_fit(&cands, lat, lon, track, 0.0)
            .map(|(r, _)| r)
            .or_else(|| {
                crate::autoland::best_landing_runway(apt_dat, lat, lon, track, 5.0, 0.0)
                    .ok()
                    .flatten()
            })
    } else {
        crate::autoland::best_landing_runway(apt_dat, lat, lon, track, 40.0, 0.0)
            .ok()
            .flatten()
    };
    let Some(mut rwy) = rwy else {
        return Err("I can't find a runway near us to build a procedure for.".to_string());
    };
    if let Some(force) = &forced_rwy {
        // Honour an explicit runway if it exists at this field.
        let cands = crate::autoland::nearby_runways(apt_dat, lat, lon, 40.0).unwrap_or_default();
        if let Some(r) = cands.into_iter().find(|r| r.id.eq_ignore_ascii_case(force)) {
            rwy = r;
        }
    }
    let icao = rwy.icao.clone();
    let rwy_id = rwy.id.clone();

    let airport = nav::find_airport(apt_dat, &icao)
        .ok()
        .flatten()
        .ok_or_else(|| format!("I couldn't read {icao} from the airport data."))?;

    let spoken_rwy = crate::circuit::spell_runway(&rwy_id);

    if departure {
        let Some(sid) = nav::sid_for_runway(cifp, &icao, &rwy_id) else {
            return Err(format!(
                "{icao} has no published departure for runway {spoken_rwy}, so there's no \
                 procedure for me to fly. You'll have to hand-fly the climb-out."
            ));
        };
        let mut legs = nav::load_proc_legs(
            cifp,
            &icao,
            ProcKind::Sid,
            &sid,
            Some(&format!("RW{rwy_id}")),
        );
        if legs.is_empty() {
            legs = nav::load_proc_legs(cifp, &icao, ProcKind::Sid, &sid, None);
        }
        if legs.is_empty() {
            return Err(format!(
                "I found the {sid} departure but couldn't read its legs."
            ));
        }
        nav::resolve_proc_fixes(fix_path, nav_path, &airport, &icao, &mut legs, lat, lon);
        let name = format!("the {} departure runway {spoken_rwy}", spell_ident(&sid));
        let ack = format!(
            "Flying {name}. I'll take the autopilot through the departure and level at the top. \
             Say cancel or my controls any time."
        );
        return Ok((
            Plan {
                name,
                legs,
                end: EndAction::Level,
                vy_kts,
                cruise_ias_kts: None, // a SID/approach manages its own speed
            },
            ack,
        ));
    }

    // Approach — the shared builder the circuit's arrival uses too. On refusal,
    // offer the visual autoland.
    let end = EndAction::Approach {
        apt_dat: apt_dat.to_string(),
        cfg,
        approach_ias,
        rwy_id: Some(rwy_id.clone()),
    };
    let plan = procedure::approach_plan_for(
        cifp, fix_path, nav_path, &airport, &icao, &rwy_id, lat, lon, end,
    )
    .map(|mut p| {
        // A climb leg inside an approach transition gets the protection too.
        p.vy_kts = vy_kts;
        p
    })
    .map_err(|why| {
        format!(
            "{why} I can't fly a procedure here. If it's clear I can try a visual autoland \
                 instead. Say 'land us'."
        )
    })?;
    let ack = format!(
        "Flying {}. I'll take the autopilot down the procedure and hand the final to \
         autoland at the fix. Say cancel or my controls any time.",
        plan.name
    );
    Ok((plan, ack))
}

/// Spell a procedure ident so TTS reads it as characters, not a word ("BLN4L"
/// → "B L N four L"). Keeps digits spoken and letters spelled.
fn spell_ident(ident: &str) -> String {
    ident
        .chars()
        .map(|c| c.to_string())
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_the_approach_type_from_the_ident() {
        assert_eq!(procedure::approach_kind("I27R"), "ILS");
        assert_eq!(procedure::approach_kind("R30"), "RNAV");
        assert_eq!(procedure::approach_kind("D12"), "VOR");
    }

    #[test]
    fn spells_idents_for_readback() {
        assert_eq!(spell_ident("BLN4L"), "B L N 4 L");
    }
}
