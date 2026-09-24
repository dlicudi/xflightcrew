//! `prepare_ils` — set up an ILS approach for a runway.
//!
//! Resolves "runway 28" to a frequency + course by looking up the nearest
//! matching ILS in the nav database (see [`crate::nav`]), then tunes NAV1, sets
//! the course, and arms the autopilot approach mode (localizer + glideslope
//! capture). The pilot just names the runway.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx, CMD_SERVOS_ON};
use crate::nav;

const NAV1_FREQ: &str = "sim/cockpit2/radios/actuators/nav1_frequency_hz";
const NAV1_OBS: &str = "sim/cockpit2/radios/actuators/nav1_obs_deg_mag_pilot";
const CMD_APPROACH: &str = "sim/autopilot/approach";
const CMD_ALT_HOLD: &str = "sim/autopilot/altitude_hold";
const ALT_DIAL: &str = "sim/cockpit2/autopilot/altitude_dial_ft";
const CUR_ALT: &str = "sim/cockpit2/gauges/indicators/altitude_ft_pilot";
const LAT: &str = "sim/flightmodel/position/latitude";
const LON: &str = "sim/flightmodel/position/longitude";
/// Only consider ILS records within this distance. An ILS approach is briefed
/// from at most ~30nm out, so a farther "match" is really a different airport —
/// e.g. parked at Gibraltar (no runway 13), "runway 13" should NOT reach 50nm to
/// Málaga. 30nm covers real approach setups and rejects cross-country mismatches.
const MAX_NM: f64 = 30.0;
/// If the nearest ILS-equipped field is within this distance, treat it as "the
/// field you're at": a matching runway at a *different*, farther field is then a
/// mistake (e.g. parked at LXGB, "runway 13" must not tune Málaga 60nm away).
const AT_FIELD_NM: f64 = 10.0;

pub struct PrepareIls;

#[derive(Deserialize)]
struct Args {
    /// Runway designator, e.g. "28" or "16L".
    runway: String,
}

#[async_trait]
impl Tool for PrepareIls {
    fn name(&self) -> &str {
        "prepare_ils"
    }

    fn description(&self) -> &str {
        "Prepare an ILS approach for a runway: look up the ILS frequency and \
         course for the nearest matching runway, tune NAV1, set the course, arm \
         the autopilot approach mode, and hold the current altitude until the \
         glideslope captures. Use for 'prepare the ILS for runway 28', 'set up \
         the ILS runway one six left'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "runway": {"type": "string", "description": "Runway designator, e.g. '28' or '16L'"}
            },
            "required": ["runway"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        // A bare "set up the ILS" names no runway — ask, don't guess one.
        if args.runway.trim().is_empty() {
            return Ok("Which runway? Say, set up the ILS for runway 28.".to_string());
        }
        let lat = ctx.xplane.get_float(LAT).await?;
        let lon = ctx.xplane.get_float(LON).await?;

        let ils = match nav::find_ils(&ctx.config.nav_data_path, &args.runway, lat, lon, MAX_NM) {
            Ok(Some(i)) => i,
            Ok(None) => {
                return Ok(format!(
                    "I can't find an ILS for runway {} nearby.",
                    args.runway
                ))
            }
            Err(e) => {
                tracing::error!("nav lookup failed: {e:#}");
                return Ok("Navigation data is unavailable.".to_string());
            }
        };

        // Airport sanity check: if we're sitting at an ILS-equipped field but the
        // matched runway belongs to a *different*, farther field, that's almost
        // certainly not what the pilot meant — don't silently tune it.
        if let Ok(Some((near_icao, near_nm))) =
            nav::nearest_ils_airport(&ctx.config.nav_data_path, lat, lon)
        {
            if near_nm <= AT_FIELD_NM && ils.airport != near_icao {
                return Ok(format!(
                    "{} has no ILS for runway {}. The nearest one is {} runway {}, {:.0} miles away — say again if that's the one you want.",
                    near_icao, args.runway, ils.airport, ils.runway, ils.distance_nm
                ));
            }
        }

        ctx.xplane.set_int(NAV1_FREQ, ils.freq_10khz).await?;
        ctx.xplane.set_float(NAV1_OBS, ils.course_mag).await?;

        // Engage the autopilot and arm the approach (localizer + glideslope).
        ctx.xplane.run_command(CMD_SERVOS_ON).await?;
        ctx.xplane.run_command(CMD_APPROACH).await?;

        // Hold the current altitude level until the glideslope captures — without
        // a pitch mode the aircraft would climb/descend through the glidepath.
        // When the glideslope comes alive, it supersedes altitude hold.
        let alt = ctx.xplane.get_float(CUR_ALT).await?;
        ctx.xplane.set_float(ALT_DIAL, alt).await?;
        ctx.xplane.run_command(CMD_ALT_HOLD).await?;

        Ok(format!(
            "Roger, {} runway {}, {:.0} miles, ILS {:.2}, course {:03.0}, approach armed, holding {:.0} feet.",
            ils.airport,
            ils.runway,
            ils.distance_nm,
            ils.freq_mhz(),
            ils.course_mag,
            alt
        ))
    }
}

/// `visual_approach` — arm (or cancel) the talk-down: PAR-style spoken guidance
/// down a synthesized 3° path to any runway, procedures or not. Advisory only —
/// the pilot flies, the monitor's tick narrates (see [`crate::approach_guide`]).
/// Born at LERL, whose nav data contains no approaches of any kind.
pub struct VisualApproach;

#[derive(serde::Deserialize)]
struct VisArgs {
    /// Runway designator ("28", "16L"); empty = best fit at the nearest field.
    #[serde(default)]
    runway: String,
    /// true = stop the guidance.
    #[serde(default)]
    cancel: bool,
}

/// How far out a talk-down target may be: the destination you're arriving at,
/// not a field a province away.
const GUIDE_MAX_NM: f64 = 30.0;

#[async_trait]
impl Tool for VisualApproach {
    fn name(&self) -> &str {
        "visual_approach"
    }

    fn description(&self) -> &str {
        "Arm spoken visual-approach guidance (a talk-down): deviation callouts \
         down a 3-degree path to a runway, for fields with no published approach. \
         Use for 'talk me down runway 28', 'guide me in', 'visual approach runway \
         10', 'call the approach'. Cancel with 'stop the guidance'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "runway": {"type": "string", "description": "Runway designator, e.g. '28'. Empty = nearest field, best-aligned runway."},
                "cancel": {"type": "boolean", "description": "true = stop the guidance"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: VisArgs = serde_json::from_value(args.clone())?;
        if args.cancel {
            return Ok(if crate::approach_guide::cancel() {
                "Guidance stopped.".to_string()
            } else {
                "No guidance running.".to_string()
            });
        }
        let lat = ctx.xplane.get_float(LAT).await?;
        let lon = ctx.xplane.get_float(LON).await?;
        // Candidate runway ends near us, nearest first (the autoland Stage-1
        // finder — its first live consumer).
        let apt = ctx.config.apt_data_path.clone();
        let cands = tokio::task::spawn_blocking(move || {
            crate::autoland::nearby_runways(&apt, lat, lon, GUIDE_MAX_NM)
        })
        .await
        .map_err(|e| anyhow::anyhow!("runway scan: {e}"))?
        .unwrap_or_default();
        if cands.is_empty() {
            return Ok("No runway within thirty miles.".to_string());
        }
        // Named runway: nearest end whose designator number matches. Unnamed:
        // the end at the nearest field whose course best matches our track
        // (you're usually already pointed at the runway you mean).
        let want = nav::runway_number(&args.runway);
        let pick = match want {
            Some(n) => cands.iter().find(|r| nav::runway_number(&r.id) == Some(n)),
            None => {
                let track = ctx
                    .xplane
                    .get_float("sim/flightmodel/position/hpath")
                    .await
                    .unwrap_or(0.0);
                let field = cands.first().map(|r| r.icao.clone()).unwrap_or_default();
                cands.iter().filter(|r| r.icao == field).min_by(|a, b| {
                    let da = ((a.course_deg - track + 540.0).rem_euclid(360.0) - 180.0).abs();
                    let db = ((b.course_deg - track + 540.0).rem_euclid(360.0) - 180.0).abs();
                    da.total_cmp(&db)
                })
            }
        };
        let Some(rw) = pick else {
            return Ok(format!("I can't find runway {} nearby.", args.runway));
        };
        crate::approach_guide::arm(crate::approach_guide::ApproachPath {
            icao: rw.icao.clone(),
            runway_id: rw.id.clone(),
            thr_lat: rw.thr_lat,
            thr_lon: rw.thr_lon,
            course_deg: rw.course_deg,
            elev_ft: rw.elev_ft,
            ft_per_nm: crate::approach_guide::FT_PER_NM,
        });
        Ok(format!(
            "Talk-down armed: runway {} at {}, field elevation {:.0}. Fly the needles in your ears — I'll call course and glidepath.",
            rw.id, rw.icao, (rw.elev_ft / 50.0).round() * 50.0
        ))
    }
}
