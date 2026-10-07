//! `prepare_ils`: set up an ILS approach the way a pilot monitoring does.
//!
//! Rebuilt 2026-09-25 on the systems survey ([`crate::systems`]): the old
//! tool tuned NAV1, then fired servos-on, APPR and ALT hold blind on every
//! airframe. APPR is a toggle (a second "set up the ILS" disarmed it), it was
//! never read back, the course source was never checked, a pilot hand flying
//! had the autopilot engaged under them, and a bare "set up the ILS" asked
//! "which runway?" with a landing clearance on the frequency.
//!
//! Now: the runway is the pilot's, else the tower's arrival clearance, else
//! the plan's; NAV1 and the course are tuned and read back
//! ([`crate::ils::tune`]); the course source goes to NAV1; the chart's
//! decision altitude is said (and the radio altimeter bug set on transport
//! avionics); the missed approach altitude is set at glideslope capture; and
//! APPR is armed from OFF, with the servos already in, inside the
//! localiser's coverage on an intercept heading, and read back. A pilot
//! hand flying keeps the aircraft: "couple the approach" is the explicit ask
//! for the autopilot. The watch ([`crate::ils::SetupWatch`]) calls the
//! needles alive and the modes captured.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

/// The tool's wait for the servos to read in after its press.
const APPROACH_ENGAGE_MAX_S: f64 = 3.0;
use crate::ils::{self, ArmVerdict};
use crate::nav;

const LAT: &str = crate::datarefs::LATITUDE;
const LON: &str = crate::datarefs::LONGITUDE;
/// Only consider ILS records within this distance. An ILS approach is briefed
/// from at most ~30nm out, so a farther "match" is really a different airport —
/// e.g. parked at Gibraltar (no runway 13), "runway 13" should NOT reach 50nm to
/// Málaga. 30nm covers real approach setups and rejects cross-country mismatches.
const MAX_NM: f64 = 30.0;
/// If the nearest ILS-equipped field is within this distance, treat it as "the
/// field you're at": a matching runway at a *different*, farther field is then a
/// mistake (e.g. parked at LXGB, "runway 13" must not tune Málaga 60nm away).
const AT_FIELD_NM: f64 = 10.0;
/// The CAT I decision height on the radio altimeter.
const CAT1_DH_FT: f64 = 200.0;

pub struct PrepareIls;

#[derive(Deserialize)]
struct Args {
    /// Runway designator, e.g. "28" or "16L"; empty = the flight's runway.
    #[serde(default)]
    runway: String,
    /// "Couple the approach": the pilot asks for the autopilot to fly it,
    /// so the servos go in if they are out.
    #[serde(default)]
    couple: bool,
}

/// Where the runway came from, for the line.
#[derive(Debug, Clone, Copy, PartialEq)]
enum RunwaySource {
    Spoken,
    Clearance,
    Plan,
}

/// The runway to set up for, when none was spoken: the tower's arrival
/// clearance first (it is what we will land on), then the plan's. Pure.
fn flight_runway(
    clearance: Option<String>,
    arrival_clearance: bool,
    plan_rwy: Option<String>,
) -> Option<(String, RunwaySource)> {
    clearance
        .filter(|_| arrival_clearance)
        .map(|r| (r, RunwaySource::Clearance))
        .or_else(|| plan_rwy.map(|r| (r, RunwaySource::Plan)))
}

/// What the arming part of the line says, from the state read. Pure.
#[derive(Debug, Clone, PartialEq)]
enum ArmPlan {
    /// Already armed or captured: left alone (APPR is a toggle).
    AlreadyArmed,
    /// The watcher arms it (now or on the intercept).
    Arm(ArmVerdict),
    /// Not armed, and why (said).
    Not(String),
}

#[allow(clippy::too_many_arguments)]
fn arm_plan(
    on_ground: bool,
    automation: Option<&'static str>,
    ap_power: bool,
    servos: bool,
    appr_status: Option<f64>,
    tracks_loc: Option<bool>,
    verdict: ArmVerdict,
) -> ArmPlan {
    if let Some(who) = automation {
        return ArmPlan::Not(format!("{who}. The ILS is tuned for the cross-check."));
    }
    if on_ground {
        return ArmPlan::Not("Approach mode stays off on the ground.".to_string());
    }
    if appr_status.is_some_and(|a| a >= 0.5) {
        return ArmPlan::AlreadyArmed;
    }
    if tracks_loc == Some(false) {
        return ArmPlan::Not(
            "This autopilot has no approach mode, the needles are yours.".to_string(),
        );
    }
    if !ap_power {
        return ArmPlan::Not("The autopilot has no power, fly the needles.".to_string());
    }
    if !servos {
        return ArmPlan::Not("Autopilot's off, so it's yours to fly. I'll call the needles. Say couple the approach if you want it on.".to_string());
    }
    match verdict {
        ArmVerdict::Never(why) => ArmPlan::Not(format!("I won't arm the approach, {why}.")),
        v => ArmPlan::Arm(v),
    }
}

/// What the loop flying the aircraft is doing, if one is
/// ([`crate::flight_state::crew_flying`]; the follower says "I'm flying the
/// approach" here, where the route it flies ends).
fn automation_flying() -> Option<&'static str> {
    crate::flight_state::crew_flying().map(|o| match o {
        crate::flight_state::Owner::Procedure => "I'm flying the approach",
        o => o.doing(),
    })
}

#[async_trait]
impl Tool for PrepareIls {
    fn name(&self) -> &str {
        "prepare_ils"
    }

    fn description(&self) -> &str {
        "Set up an ILS approach: the runway spoken, else the tower's arrival \
         clearance, else the plan's; NAV1 and the course tuned and read back, \
         the course source on NAV1, minimums, the approach mode armed when the \
         autopilot is flying and we're on the intercept, the needles called. \
         'Couple the approach' also engages the autopilot."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "runway": {"type": "string", "description": "Runway designator, e.g. '28' or '16L'; empty for the flight's runway"},
                "couple": {"type": "boolean", "description": "Engage the autopilot to fly it"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        let lat = ctx.xplane.get_float(LAT).await?;
        let lon = ctx.xplane.get_float(LON).await?;
        let plan = crate::simbrief::loaded_plan();

        // The runway: spoken, else the flight's own.
        let (runway, source) = if !args.runway.trim().is_empty() {
            (args.runway.trim().to_uppercase(), RunwaySource::Spoken)
        } else {
            let arrival = matches!(
                crate::party_line::clearance_state(),
                Some(crate::tower::State::Inbound | crate::tower::State::ClearedToLand)
            );
            match flight_runway(
                crate::party_line::clearance_runway(),
                arrival,
                plan.as_ref().and_then(|p| p.procedures.dest_rwy.clone()),
            ) {
                Some(r) => r,
                None => return Ok("Which runway? Say, set up the ILS for runway 28.".to_string()),
            }
        };

        // Where to look: around us, or around the plan's destination when
        // the runway is the plan's (set up in the cruise, 100 miles out).
        let dest_pos = plan.as_ref().and_then(|p| {
            p.waypoints
                .iter()
                .rev()
                .find(|w| w.ident.eq_ignore_ascii_case(&p.destination))
                .map(|w| (p.destination.clone(), w.lat, w.lon))
        });
        let nav_path = ctx.config.nav_data_path.clone();
        let near = nav::find_ils(&nav_path, &runway, lat, lon, MAX_NM)
            .ok()
            .flatten();
        let at_dest = dest_pos.as_ref().and_then(|(icao, dlat, dlon)| {
            nav::find_ils(&nav_path, &runway, *dlat, *dlon, 10.0)
                .ok()
                .flatten()
                .filter(|i| i.airport.eq_ignore_ascii_case(icao))
                .map(|i| (i, *dlat, *dlon))
        });
        let found = match source {
            RunwaySource::Plan => at_dest.or(near.map(|i| (i, lat, lon))),
            _ => near.map(|i| (i, lat, lon)).or(at_dest),
        };
        let Some((ils, slat, slon)) = found else {
            let whose = match source {
                RunwaySource::Clearance => " the tower gave us",
                RunwaySource::Plan => " in the plan",
                RunwaySource::Spoken => "",
            };
            return Ok(format!("I can't find an ILS for runway {runway}{whose}."));
        };

        // Airport sanity check: if we're sitting at an ILS-equipped field but the
        // matched runway belongs to a *different*, farther field, that's almost
        // certainly not what the pilot meant — don't silently tune it.
        if source == RunwaySource::Spoken {
            if let Ok(Some((near_icao, near_nm))) = nav::nearest_ils_airport(&nav_path, lat, lon) {
                if near_nm <= AT_FIELD_NM && ils.airport != near_icao {
                    return Ok(format!(
                        "{} has no ILS for runway {}. The nearest one is {} runway {}, {:.0} miles away. Say again if that's the one you want.",
                        near_icao, runway, ils.airport, ils.runway, ils.distance_nm
                    ));
                }
            }
        }

        // Tune and set the course, read back.
        if ils::tune(ctx.xplane, &ils.airport, &ils.runway, slat, slon)
            .await
            .is_none()
        {
            return Ok(format!(
                "NAV1 didn't take the ILS for runway {} at {}, {:.2}. Tune it by hand.",
                ils.runway,
                ils.airport,
                ils.freq_mhz()
            ));
        }
        let mut line = format!(
            "ILS runway {} at {}, {:.2}, course {:03.0}, set.",
            ils.runway,
            ils.airport,
            ils.freq_mhz(),
            ils.course_mag
        );
        if source == RunwaySource::Clearance {
            line = format!("The runway we're cleared for. {line}");
        }

        // The course source: the approach mode follows the HSI, and a panel
        // left on GPS tracks the GPS whatever NAV1 says.
        let mut source_fixed = None;
        if let Ok(src) = ctx.xplane.get_float(ils::HSI_SOURCE).await {
            if src.round() as i64 != 0 {
                let _ = crate::autoflight::press(ctx.xplane, crate::autoflight::Button::HsiNav1).await;
                let now = ctx.xplane.get_float(ils::HSI_SOURCE).await;
                let ok = now.is_ok_and(|v| v.round() as i64 == 0);
                source_fixed = Some(ok);
                line.push_str(if ok {
                    " Course source to NAV1."
                } else {
                    " The course source won't go to NAV1, check it."
                });
            }
        }

        // Minimums from the chart, the missed approach from its legs.
        let systems = crate::systems::current();
        let chart = nav::ils_chart(&ctx.config.cifp_dir, &ils.airport, &ils.runway);
        let mut dh_set = None;
        if systems.transport_avionics() {
            let _ = ctx.xplane.set_float(ils::RA_DH, CAT1_DH_FT).await;
            let ok = ctx
                .xplane
                .get_float(ils::RA_DH)
                .await
                .is_ok_and(|v| (v - CAT1_DH_FT).abs() < 5.0);
            dh_set = Some(ok);
        }
        match (chart.and_then(|c| c.decision_alt_ft()), dh_set) {
            (Some(da), Some(true)) => line.push_str(&format!(
                " Minimums {}, radio altimeter {CAT1_DH_FT:.0}.",
                crate::answers::with_commas(da)
            )),
            (Some(da), _) => {
                line.push_str(&format!(" Minimums {}.", crate::answers::with_commas(da)))
            }
            (None, Some(true)) => line.push_str(&format!(" Radio altimeter {CAT1_DH_FT:.0}.")),
            _ => {}
        }
        let missed = chart.and_then(|c| c.missed_alt_ft);

        // The autopilot: read, never assumed.
        let (on_ground, ap_power, mut servos, appr, vdef, gs_valid, hdg) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY),
            ctx.xplane.get_float(ils::AP_POWER),
            ctx.xplane.get_float(ils::SERVOS),
            crate::autoflight::read_ap_state(&*ctx.xplane),
            ctx.xplane.get_float(ils::VDEF),
            ctx.xplane.get_float(ils::GS_VALID),
            ctx.xplane.get_float(crate::datarefs::COMPASS_HEADING_DEG),
        );
        let on_ground = on_ground.is_ok_and(|v| v > 0.5);
        // Fail open on power: a sim without the dataref still has an autopilot.
        let ap_power = ap_power.map_or(true, |v| v > 0.5);
        let automation = automation_flying();
        let tracks_loc = systems.ap_type.and_then(|a| a.tracks_localizer());
        let flies_gs = systems.ap_type.and_then(|a| a.flies_glideslope());
        let mut engaged = false;
        if args.couple
            && automation.is_none()
            && !on_ground
            && ap_power
            && tracks_loc != Some(false)
            && !servos.as_ref().is_ok_and(|v| *v > 0.5)
        {
            // Pressed and CONFIRMED (`ap_engage::engage_confirmed`: FD
            // first, read until in, at most 3 s): one read straight after
            // the press could not yet see the servos come in.
            engaged = crate::ap_engage::engage_confirmed(
                ctx.xplane,
                &crate::ap_engage::fd_commands(),
                "ils",
                "setup_couple",
                APPROACH_ENGAGE_MAX_S,
            )
            .await;
            servos = ctx.xplane.get_float(ils::SERVOS).await;
            line.push_str(if engaged {
                " Autopilot on."
            } else {
                " The autopilot didn't engage."
            });
        }
        let servos = servos.is_ok_and(|v| v > 0.5);
        let verdict = match hdg {
            Ok(h) => ils::arm_verdict(lat, lon, h, &ils),
            Err(_) => ArmVerdict::Wait("I can't read our heading"),
        };
        let plan_arm = arm_plan(
            on_ground,
            automation,
            ap_power,
            servos,
            // Armed or captured, either half, on the bitfield (the 737's
            // approach_status read 0 through captures, #81).
            appr.as_ref()
                .ok()
                .and_then(|v| crate::autoflight::Modes::from_read(*v))
                .map(|m| if m.approach_any() { 1.0 } else { 0.0 }),
            tracks_loc,
            verdict,
        );
        let arm_wanted = matches!(plan_arm, ArmPlan::Arm(_));
        match &plan_arm {
            ArmPlan::AlreadyArmed => line.push_str(" Approach already armed."),
            ArmPlan::Arm(ArmVerdict::Now) => line.push_str(" Arming the approach."),
            ArmPlan::Arm(ArmVerdict::Wait(why)) => line.push_str(&format!(
                " Approach not armed yet, {why}. I'll arm it on the intercept."
            )),
            ArmPlan::Arm(ArmVerdict::Never(_)) => {}
            ArmPlan::Not(why) => {
                line.push(' ');
                line.push_str(why);
            }
        }
        if (arm_wanted || matches!(plan_arm, ArmPlan::AlreadyArmed)) && flies_gs == Some(false) {
            line.push_str(" This autopilot flies the localiser only, the glideslope is yours.");
        }
        // #175: a positive needle is above the beam (this said so of a negative one).
        let high = vdef.as_ref().is_ok_and(|v| ils::above_glideslope(*v) && v.abs() > 1.0);
        if gs_valid.is_ok_and(|v| v > 0.5) && high && !on_ground {
            line.push_str(" We're above the glideslope, it won't capture from above.");
        }
        if let (Some(m), true) = (
            missed,
            arm_wanted || matches!(plan_arm, ArmPlan::AlreadyArmed),
        ) {
            if flies_gs != Some(false) {
                line.push_str(&format!(
                    " Missed approach {}, I'll set it at glideslope capture.",
                    crate::answers::with_commas(m)
                ));
            }
        }

        crate::blackbox::record(
            "ils",
            json!({
                "event": "setup", "icao": ils.airport, "rwy": ils.runway, "source": format!("{source:?}"),
                "freq_mhz": ils.freq_mhz(), "course": ils.course_mag, "source_fixed": source_fixed,
                "dh_set": dh_set, "da": chart.and_then(|c| c.decision_alt_ft()), "missed": missed,
                "ap_type": systems.ap_type.map(|a| format!("{a:?}")), "servos": servos, "engaged": engaged,
                "arm": format!("{plan_arm:?}"), "verdict": format!("{verdict:?}"),
            }),
        );

        // The watch, unless an automation owns the approach or we are on
        // the ground.
        if automation.is_none() && !on_ground {
            let armed_now = matches!(plan_arm, ArmPlan::AlreadyArmed);
            let watch =
                ils::SetupWatch::new(arm_wanted, armed_now, missed.filter(|_| servos), flies_gs);
            ils::spawn_setup_watch(
                ctx.config.xplane_base_url.clone(),
                ils.clone(),
                watch,
                ctx.voice.cloned(),
            );
        }
        Ok(line)
    }
}

#[cfg(test)]
mod prepare_ils_tests {
    use super::*;

    #[test]
    fn the_flight_names_the_runway_when_the_pilot_does_not() {
        assert_eq!(
            flight_runway(Some("09".into()), true, Some("27".into())),
            Some(("09".into(), RunwaySource::Clearance)),
            "the landing clearance outranks the plan"
        );
        assert_eq!(
            flight_runway(Some("27".into()), false, Some("24".into())),
            Some(("24".into(), RunwaySource::Plan)),
            "a DEPARTURE clearance is not the runway we land on"
        );
        assert_eq!(flight_runway(None, false, None), None);
    }

    #[test]
    fn approach_mode_is_armed_only_from_off_with_the_autopilot_flying() {
        let now = ArmVerdict::Now;
        assert_eq!(
            arm_plan(false, None, true, true, Some(0.0), Some(true), now),
            ArmPlan::Arm(now)
        );
        assert_eq!(
            arm_plan(false, None, true, true, Some(1.0), Some(true), now),
            ArmPlan::AlreadyArmed,
            "APPR is a toggle: armed is left alone"
        );
        assert!(
            matches!(arm_plan(false, None, true, false, Some(0.0), Some(true), now), ArmPlan::Not(w) if w.contains("yours")),
            "a pilot hand flying keeps the aircraft"
        );
        assert!(matches!(
            arm_plan(true, None, true, true, Some(0.0), Some(true), now),
            ArmPlan::Not(_)
        ));
        assert!(
            matches!(arm_plan(false, Some("Autoland"), true, true, Some(0.0), Some(true), now), ArmPlan::Not(w) if w.contains("cross-check"))
        );
        assert!(
            matches!(arm_plan(false, None, true, true, Some(0.0), Some(false), now), ArmPlan::Not(w) if w.contains("no approach mode"))
        );
        assert!(
            matches!(arm_plan(false, None, false, true, Some(0.0), Some(true), now), ArmPlan::Not(w) if w.contains("no power"))
        );
        assert!(matches!(
            arm_plan(
                false,
                None,
                true,
                true,
                Some(0.0),
                None,
                ArmVerdict::Never("past it")
            ),
            ArmPlan::Not(_)
        ));
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
            crate::runway::nearby_runways(&apt, lat, lon, GUIDE_MAX_NM)
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
                    .get_float(crate::datarefs::TRACK_DEG_TRUE)
                    .await
                    .unwrap_or(0.0);
                let field = cands.first().map(|r| r.icao.clone()).unwrap_or_default();
                cands.iter().filter(|r| r.icao == field).min_by(|a, b| {
                    let da = crate::angles::abs_diff_deg(a.course_deg, track);
                    let db = crate::angles::abs_diff_deg(b.course_deg, track);
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
