//! `full_auto` — the copilot flies the WHOLE thing from cold: power up, start
//! the engine, request taxi, drive to the runway, line up, and fly the mission
//! — a circuit, or (flight=true) the pilot's actual SimBrief flight plan to
//! its destination.
//!
//! The arming end of [`crate::full_auto`]. It holds the profile params the
//! sub-tools need and hands a freshly-built set to the orchestrator thread.
//! The SimBrief mission fetches the OFP here (on demand, never in the realtime
//! loop), refuses honestly when there's no plan or we're not at its origin,
//! and best-effort loads the route into the GPS so the panel shows where we're
//! going. EXPERIMENTAL — nine subsystems in a chain; VMC, hand ready.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::FuelSelectorProfile;
use crate::full_auto::{Mission, Route};
use crate::recovery_plan::Purpose;
use crate::tools::autopilot::ApConfig;

/// Which engine start a full-auto sequence runs — one per airframe CLASS, and
/// they are not interchangeable: the piston start cranks a starter that isn't
/// there on a turbine and then reports "check fuel and mixture" about levers
/// the aircraft hasn't got.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartKind {
    Piston,
    Turboprop,
    Jet,
    /// The aircraft's own scripted start behind the stock `auto_start`
    /// command (`AircraftProfile::scripted_auto_start`): one press, the
    /// aircraft's real switches, its own timing. The stock 737-800.
    Scripted,
    /// ENG MODE knob and guarded master switches — the airliner pattern
    /// (`AircraftProfile::airliner_start`). Added 2026-09-20: the standalone
    /// `start_engines` tool was registered for the A330 and full auto still
    /// said "I don't have an engine start for this aircraft", because the
    /// class it dispatches on had never heard of it. A tool the registry
    /// builds and the mission cannot reach is not a capability.
    Airliner,
    /// A turbine airframe that declares NO start sequence — the Citation X,
    /// whose stock systems the MD-82's APU/bleed/cross-tie sequence does not
    /// fit. There is nothing to crank, so full auto says so instead of
    /// falling through to the piston start and cranking a starter that isn't
    /// there (which is exactly what it did to the MD-82 until 2026-07-27).
    None,
}

/// The class from the profile's start declarations. The PT6 sequence wins where
/// an airframe declares both, because a turboprop's start is the specific one;
/// `turbine_start` alone means a jet with an APU and bleed-air starters. The
/// absence of both is a piston — but only if the airframe HAS pistons.
///
/// The jet arm was MISSING until 2026-07-27: `turbine_start` airframes fell
/// through to the piston start, so full auto on an MD-82 announced "spinning
/// up the engine" and then "cranking, the engine hasn't caught" at a cold jet
/// that was never going to catch. `piston` closes the other half of that hole:
/// a turbine with no sequence declared is not a piston, it is an airframe we
/// cannot start.
pub fn start_kind(
    turboprop: bool,
    jet: bool,
    piston: bool,
    scripted: bool,
    airliner: bool,
) -> StartKind {
    // The airliner block wins over everything: it is the most specific, and an
    // airframe that declares it has switches the others' sequences would be
    // cranking past. Scripted comes next for the same reason it always did —
    // the aircraft's own start beats ours.
    match (airliner, scripted, turboprop, jet, piston) {
        (true, ..) => StartKind::Airliner,
        (false, true, ..) => StartKind::Scripted,
        (false, false, true, _, _) => StartKind::Turboprop,
        (false, false, false, true, _) => StartKind::Jet,
        (false, false, false, false, true) => StartKind::Piston,
        (false, false, false, false, false) => StartKind::None,
    }
}

impl StartKind {
    /// How long full auto waits for the engine to be RUNNING after it fires the
    /// start — a per-class figure, because the three starts are not the same
    /// order of duration. A piston catches in seconds. The jet sequence is a
    /// procedure: bleed pressure, then crank/fuel/idle on the right engine, then
    /// the same again on the left (which is the one full auto watches), and its
    /// own internal waits alone bound it near three minutes. The single
    /// piston-shaped 40 s deadline gave up on an MD-82 forty seconds in, while
    /// the start sequence was still waiting on bleed pressure and went on to
    /// crank both engines for a full-auto run that had already called "your
    /// aircraft" (Luton, 2026-07-27).
    pub fn engine_start_secs(self) -> f64 {
        match self {
            // The 737's script took 124 s from cold to both running; its
            // fire tests and APU spin-up are inside that.
            StartKind::Jet | StartKind::Scripted | StartKind::Airliner => 300.0,
            StartKind::Turboprop | StartKind::Piston | StartKind::None => 40.0,
        }
    }
}

/// A resume this near the plan's destination is its ARRIVAL: the approach
/// to the plan's runway (`recovery_plan`, the planner's arrival radius for a
/// resume). LIPE-LIPK (2026-09-25): 6.6 nm from Forli at 2,800 ft over the
/// coastal plain, the refusal said "climb to 7,500 feet first, the sector
/// altitude" and the pilot's "land the aircraft" then self-picked Cervia.
pub(crate) const ARRIVAL_REJOIN_NM: f64 = crate::recovery_plan::RESUME_ARRIVAL_NM;

/// The plan's landing runway as the resume judges a final against it: the
/// threshold and the landing course, degrees true.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FinalCourse {
    pub thr_lat: f64,
    pub thr_lon: f64,
    pub course_true: f64,
}

/// Runway `rwy`'s end among `runways` as a [`FinalCourse`]. Pure.
pub(crate) fn final_course(runways: &[crate::nav::Runway], rwy: &str) -> Option<FinalCourse> {
    runways.iter().find_map(|r| {
        if crate::nav::same_runway_end(&r.ident_a, rwy) {
            Some(FinalCourse { thr_lat: r.lat_a, thr_lon: r.lon_a, course_true: r.heading_a_deg() })
        } else if crate::nav::same_runway_end(&r.ident_b, rwy) {
            Some(FinalCourse { thr_lat: r.lat_b, thr_lon: r.lon_b, course_true: r.heading_b_deg() })
        } else {
            None
        }
    })
}

/// Where the aircraft sits on the plan's final: miles to fly to the
/// threshold along the course, miles off its extended centreline (right
/// positive) and degrees between the track and the landing course.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct OnFinal {
    pub along_nm: f64,
    pub xtk_nm: f64,
    pub track_off_deg: f64,
}

/// A track this near the landing course is flying the final, degrees: an
/// intercept autoland turns onto, never a leg across it.
pub(crate) const FINAL_ALIGN_DEG: f64 = 45.0;

/// The aircraft on the plan's final: in front of the threshold inside
/// [`ARRIVAL_REJOIN_NM`], within the rejoin corridor of the extended
/// centreline, tracking within [`FINAL_ALIGN_DEG`] of the landing course.
/// EGKK-EGBB (sortie 1790760672-approach, iMac 2026-09-30, t 1790760727.7):
/// the 737 on a 12 nm final to 33 after "my controls" was refused as "4
/// miles left of course on the PUFAX to HON leg", because the navlog's last
/// fix GROVE is nearer EGBB than the final was, so the arrival rule never
/// saw it past the route. Pure.
pub(crate) fn on_final(fc: &FinalCourse, lat: f64, lon: f64, track_true: f64) -> Option<OnFinal> {
    if ![lat, lon, track_true, fc.thr_lat, fc.thr_lon, fc.course_true]
        .iter()
        .all(|v| v.is_finite())
    {
        return None;
    }
    let (x_m, along_m) =
        crate::nav::runway_frame_m(fc.thr_lat, fc.thr_lon, fc.course_true, lat, lon);
    let (along_nm, xtk_nm) = (along_m / 1852.0, x_m / 1852.0);
    let track_off_deg = crate::angles::abs_diff_deg(track_true, fc.course_true);
    (along_nm > 0.0
        && along_nm <= ARRIVAL_REJOIN_NM
        && xtk_nm.abs() <= crate::interaction::REJOIN_CORRIDOR_NM
        && track_off_deg <= FINAL_ALIGN_DEG)
        .then_some(OnFinal { along_nm, xtk_nm, track_off_deg })
}

/// The airborne recovery's state gates, each naming itself: live, in the
/// air, and flyable (`recover::unflyable`: a stall, slow under the
/// configuration's floor, an upset). Everything else (low, off the route,
/// behind it, a bank) is the planner's to fly from, never a refusal: "Resume
/// needs a thousand feet" and "wings level first" left a descending 737 to
/// nobody at LEMG (2026-10-05). `None`: the aircraft may be taken. Pure.
pub(crate) fn resume_gate(frozen: bool, on_ground: bool, unflyable: Option<&str>) -> Option<String> {
    if frozen {
        Some("We're paused or in replay. Go back to live and ask again.".to_string())
    } else if on_ground {
        Some(
            "We're on the ground. Resume is for the air; say fly the flight plan for the \
             whole mission."
                .to_string(),
        )
    } else {
        unflyable.map(|u| {
            let what = match u {
                "stall" => "we're stalling",
                "slow" => "we're slow for this configuration",
                "bank" => "we're steeply banked",
                "nose_high" => "the nose is high",
                _ => "the nose is low",
            };
            format!("Not yet: {what}. Recover first, or say recover, then ask again.")
        })
    }
}

/// One read of where the aircraft is and how it moves, for the resume's
/// recheck ([`rejoin_unsteady`]): seconds, position, indicated feet, VSI
/// (fpm), groundspeed (kt) and track (degrees true).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Steady {
    pub t_s: f64,
    pub lat: f64,
    pub lon: f64,
    pub alt_ft: f64,
    pub vs_fpm: f64,
    pub gs_kts: f64,
    pub track_true: f64,
}

/// Off the first read's own track and level at the recheck, nm and feet...
pub(crate) const REJOIN_OFF_TRACK_NM: f64 = 0.5;
pub(crate) const REJOIN_OFF_LEVEL_FT: f64 = 200.0;
/// ...a vertical speed that changed this much between the reads, fpm...
pub(crate) const REJOIN_VS_CHANGE_FPM: f64 = 1_000.0;
/// ...or one no steady flight has, fpm: the aircraft is not steady.
pub(crate) const REJOIN_MAX_VS_FPM: f64 = 6_000.0;

/// Whether the aircraft stayed on the flight the rejoin was checked
/// against: the recheck is judged against the FIRST read projected along
/// its own track, groundspeed and vertical speed, never against raw
/// movement (0.1 nm is 1.2 s at 296 kt). `None`: steady. EGKK-EGBB enroute
/// setup (#177, 2026-09-29): 0.1 nm and 202 ft in 1.1 s was a zoom at
/// 10,600 to 11,900 fpm, which this still refuses. Pure.
pub(crate) fn rejoin_unsteady(a: &Steady, b: &Steady) -> Option<String> {
    const NOT_TAKEN: &str = "I haven't taken control; stabilise and try again.";
    let dt = (b.t_s - a.t_s).max(0.0);
    let (plat, plon) =
        crate::nav::project_latlon(a.lat, a.lon, a.track_true, a.gs_kts.max(0.0) * dt / 3600.0);
    let off_nm = crate::nav::haversine_nm(plat, plon, b.lat, b.lon);
    let off_ft = b.alt_ft - (a.alt_ft + a.vs_fpm * dt / 60.0);
    let vs = if a.vs_fpm.abs() >= b.vs_fpm.abs() { a.vs_fpm } else { b.vs_fpm };
    if vs.abs() > REJOIN_MAX_VS_FPM || (b.vs_fpm - a.vs_fpm).abs() > REJOIN_VS_CHANGE_FPM {
        let dir = if vs > 0.0 { "climbing" } else { "descending" };
        return Some(format!(
            "We're {dir} at {:.0} feet a minute, {:.0} to {:.0} while I checked the rejoin. \
             {NOT_TAKEN}",
            vs.abs(),
            a.vs_fpm,
            b.vs_fpm
        ));
    }
    if off_nm > REJOIN_OFF_TRACK_NM || off_ft.abs() > REJOIN_OFF_LEVEL_FT {
        return Some(format!(
            "We left the track by {off_nm:.1} miles and the level by {:.0} feet while I \
             checked the rejoin. {NOT_TAKEN}",
            off_ft.abs()
        ));
    }
    None
}

/// The resume's refusal when a climb first needs a heading and the magnetic
/// variation did not read.
pub const NO_VARIATION: &str = "I can't read the magnetic variation, so I can't set a heading for \
     the climb to the safe altitude. I haven't taken control.";

/// A [`Steady`] read now, seconds since `clock`; `None` for a failed read.
async fn steady_read(
    xp: &crate::xplane::XPlaneClient,
    clock: std::time::Instant,
) -> Option<Steady> {
    // Position and groundspeed are one frame (`sim::position`, #157).
    let (pos, alt, vs, track) = tokio::join!(
        sim::position::read(xp),
        xp.get_float(crate::datarefs::ALTITUDE_FT),
        xp.get_float(crate::datarefs::VVI_FPM),
        xp.get_float("sim/flightmodel/position/hpath"),
    );
    let (lat, lon) = sim::position::lat_lon(&pos);
    let gs = sim::position::field(&pos, |p| p.gs_ms);
    let t_s = clock.elapsed().as_secs_f64();
    let [lat, lon, alt_ft, vs_fpm, gs_ms, track_true] =
        crate::control::measurements("resume_recheck", [lat, lon, alt, vs, gs, track])?;
    let gs_kts = gs_ms * sim::ground::MS_TO_KTS;
    Some(Steady { t_s, lat, lon, alt_ft, vs_fpm, gs_kts, track_true })
}

/// What the recovery planner arms (`recovery_plan`): the follower over the
/// route (a climb first when below the way's floor) and then the arrival,
/// autoland on the destination's approach now, or, for a landing request
/// only, the nearest suitable runway and why not the destination.
pub(crate) enum ResumeArm {
    Route(Mission, String),
    Arrival {
        dest: String,
        /// The plan's runway; `None` lets autoland pick the destination's
        /// best end (the self-pick keeps to the destination).
        rwy: Option<String>,
        ack: String,
    },
    Nearest(String),
}

/// What a landing request armed through the planner.
pub(crate) enum LandArm {
    /// Flying to the destination: the line.
    Armed(String),
    /// The nearest suitable runway instead, and why.
    Nearest(String),
}

pub struct FullAuto {
    // engine start
    /// Engine count, so a piston TWIN gets both starters cranked. The piston
    /// start is sequential and per-engine; a chain that cranked only number one
    /// would taxi out on one engine.
    pub engines: usize,
    /// Cycle (up/down) fuel selectors, so the piston start opens the valves
    /// before cranking — see `StartEngine::cycle_selectors`.
    pub cycle_selectors: Vec<crate::aircraft::CycleSelectorProfile>,
    pub fuel_selector: Option<FuelSelectorProfile>,
    pub alternator_on_cmds: Vec<String>,
    /// DC-bus power-on commands (battery + master) for airframes whose stock
    /// battery dataref is overwritten by a custom switch sync (the Twin Otter).
    pub bus_power_on_cmds: Vec<String>,
    /// The turboprop start sequence, when the airframe is a turboprop — full
    /// auto cranks a PT6 with this instead of the piston start (which never
    /// catches on a turbine).
    pub turboprop_start: Option<crate::aircraft::TurbineStartProfile>,
    pub airliner_start: Option<crate::aircraft::AirlinerStartProfile>,
    /// Jet with an APU (profile `turbine_start`): full auto cranks it with the
    /// `start_engines` sequence — bleed air, N2, fuel at 20% — not the piston
    /// start. THREE airframe classes, three starts; the missing third had full
    /// auto piston-cranking an MD-82 ("cranking, the engine hasn't caught,
    /// check fuel and mixture") on an aircraft with neither, 2026-07-27.
    pub jet_start: bool,
    /// The aircraft scripts its own start behind `sim/operation/auto_start`.
    pub scripted_auto_start: bool,
    /// EVERY electrical system's ON command — the full bring-up run after the
    /// engine catches (generators AND bleed air, which feed the Twin Otter's
    /// autopilot). Empty on airframes with no electrical profile.
    pub all_electrical_on_cmds: Vec<String>,
    /// Toggle-only electrical switches (see `tools::all_electrical_toggles`).
    pub all_electrical_toggles: Vec<crate::aircraft::ToggleSwitch>,
    // taxi / line-up
    pub park_brake_valve_set: Option<f64>,
    pub takeoff_roll_m: Option<f64>,
    pub mtow_kg: Option<f64>,
    /// Book figures for the arm-time preflight of a Flight mission
    /// (`full_auto::preflight`): the runway at each end and the all-up weight
    /// are checked at the gate, not found at the hold or 25 nm out.
    pub landing_distance_ft: Option<f64>,
    pub takeoff_distance_ft: Option<f64>,
    pub max_gross_lb: Option<f64>,
    /// Whether the arm may load this airframe from the plan (#95), and the
    /// airframe's name for the line that says it will not.
    pub load_sheet: crate::aircraft::LoadSheet,
    pub airframe: String,
    /// Flap-handle ratio set for takeoff and retracted climbing out (profile's
    /// `flaps.takeoff_ratio`); `None` leaves the flaps alone.
    pub takeoff_flap_ratio: Option<f64>,
    pub flap_schedule: Vec<(f64, f64)>,
    /// Indicated cruise speed for the enroute cruise-power hold (profile's
    /// `reference.cruise_ias_kts`); `None` leaves cruise at climb power.
    pub cruise_ias_kts: Option<f64>,
    /// The airframe's transition altitude (QNH below, standard above).
    pub transition_altitude_ft: f64,
    /// Constant-speed prop levers, threaded into every ground-roll sub-tool so
    /// they unfeather before taxi/line-up/takeoff. `None` on the pistons.
    pub prop: Option<crate::aircraft::PropProfile>,
    /// This airframe's taxi pacing, threaded into the ground-roll sub-tools it
    /// builds — the chain must taxi a jet like a jet, same as a spoken ask.
    pub pace: crate::autotaxi::Pace,
    /// Its tail-strike attitude, threaded the same way.
    pub tail_strike_deg: Option<f64>,
    pub pitch_authority_down: Option<f64>,
    /// Vne/Vmo for the climb's overspeed protection.
    pub vne_kts: Option<f64>,
    /// Slowest the pattern may climb toward — see `circuit::climb_limit_fpm`.
    pub climb_floor_kts: f64,
    /// Clean stall speed, the floor under the pattern speed (flown clean).
    pub vs_clean_kts: Option<f64>,
    /// The airframe's exterior lights, for the departure light schedule.
    pub lights: Vec<crate::aircraft::LightProfile>,
    /// Panel flood-lighting dataref override, when the airframe has its own.
    pub flood_dataref: Option<String>,
    /// ...or a toggle command, when the switch is one (the Baron 58). Carried
    /// alongside the dataref because full auto builds its OWN copy of the
    /// flood tool: an override wired here and not there is a tool that behaves
    /// one way for the pilot and another for the automation.
    pub flood_toggle: Option<crate::aircraft::ToggleSwitch>,
    // circuit / climb-out
    pub cfg: ApConfig,
    pub vr_kts: f64,
    pub vy_kts: f64,
    /// Initial climb speed (see `autotakeoff::Params::initial_climb_kts`).
    pub initial_climb_kts: f64,
    pub climb_fpm: f64,
    pub retract_gear: bool,
    pub approach_ias: f64,
    /// Turbine airframe → run the APU/electrical power-up before engine start.
    pub power_up_first: bool,
    /// The airframe has SOME piston start we can fly — the generic `engine`
    /// group, or a custom cycle-switch sequence (`piston_start`, the DC-3).
    /// Without it and without a turbine sequence there is no start at all —
    /// see [`StartKind::None`].
    pub piston_engine: bool,
    /// The generic continuous mixture lever (`engine` group) specifically,
    /// not any piston: the DC-3 is a piston with a notched xlua lever the
    /// manager must not creep. Arms the enroute mixture manager and the
    /// arrival's full rich.
    pub piston_mixture: bool,
}

#[derive(Deserialize, Default)]
struct Args {
    #[serde(default)]
    off: bool,
    #[serde(default)]
    right: bool,
    /// true = fly the pilot's SimBrief flight plan instead of a circuit.
    #[serde(default)]
    flight: bool,
    #[serde(default)]
    resume: bool,
    /// The place the pilot named ("take us to Bologna"), checked against the
    /// plan's destination: its name, and ICAO candidates when spelled.
    #[serde(default)]
    to: Option<String>,
    #[serde(default)]
    to_codes: Vec<String>,
}

/// Whether the place the pilot named is the plan's destination: an ICAO
/// candidate that matches, or every word of the name found in the airport's
/// name ("bologna" in "Bologna Borgo Panigale"). Pure.
pub fn names_destination(to: &str, to_codes: &[String], icao: &str, name: &str) -> bool {
    if to_codes.iter().any(|c| c.eq_ignore_ascii_case(icao)) || to.eq_ignore_ascii_case(icao) {
        return true;
    }
    let name = name.to_ascii_lowercase();
    let words: Vec<String> = to.to_ascii_lowercase().split_whitespace().map(str::to_string).collect();
    !words.is_empty() && words.iter().all(|w| name.split(|c: char| !c.is_alphanumeric()).any(|n| n == w))
}

#[async_trait]
impl Tool for FullAuto {
    fn name(&self) -> &str {
        "full_auto"
    }

    fn description(&self) -> &str {
        "Fly the ENTIRE flight autonomously from cold: power up, start the \
         engine, request taxi from the tower, taxi to the runway, line up, take \
         off, and fly the mission. Use for 'full auto', 'do everything', \
         'start up and fly a circuit', 'take us up from cold'. flight=true \
         flies the pilot's SimBrief FLIGHT PLAN to its destination instead of \
         a circuit — 'fly my flight plan', 'fly the simbrief flight', 'fly us \
         to the destination'. off=true to cancel. (For just the airborne \
         circuit from the runway, use circuit.)"
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "off": {"type": "boolean", "description": "true = cancel full auto"},
                "right": {"type": "boolean", "description": "true = right-hand pattern (circuit only)"},
                "flight": {"type": "boolean", "description": "true = fly the SimBrief flight plan to its destination"},
                "resume": {"type": "boolean", "description": "resume the loaded flight plan from a checked airborne rejoin"}
            }
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: Args = serde_json::from_value(args.clone()).unwrap_or_default();
        // A waiver is only ever this arm's preflight's to hand on.
        crate::full_auto::RUNWAY_WAIVER_PENDING.store(false, std::sync::atomic::Ordering::SeqCst);
        if a.off {
            return Ok(if crate::full_auto::abort() {
                "I've stopped flying. You have the aircraft.".to_string()
            } else {
                "I'm not flying the aircraft.".to_string()
            });
        }
        // A new flight still loading is no place to start one: every read
        // is the sim mid-reload. LEMG (#134, sortie 1790584476-full_auto):
        // "fly a circuit" armed 0.6 s after "put us on a 5 mile final",
        // judged the load's transient position (runway 12's midpoint, AGL
        // -84 ft) as "on 12, 1,376 m left", and flew on as a zombie under
        // the approach that followed.
        if let Some(line) = crate::tools::setup::new_flight_loading_refusal() {
            return Ok(line);
        }
        if crate::full_auto::running() {
            return Ok("I'm already running the full sequence. Say cancel to stop.".to_string());
        }
        if ctx
            .xplane
            .get_float(crate::datarefs::IS_IN_REPLAY)
            .await
            .is_ok_and(|v| v > 0.5)
        {
            return Ok("We're in replay. Go back to live and ask again.".to_string());
        }

        // "Resume" on the ground is the mission from where we are: the same
        // arm as "fly the flight plan", which picks up at whatever stage the
        // aircraft is in. Refused as "resume is for the air" three times at
        // LIME after a taxi stand-down (2026-09-25).
        let on_ground = ctx
            .xplane
            .get_float(crate::datarefs::ON_GROUND_ANY)
            .await
            .is_ok_and(sim::ground::on_ground);
        // "Fly the flight plan" in the air is the checked RESUME of the plan
        // on the shelf: it used to be refused ("we're already flying, say
        // navigate"), so a pilot handing over mid-route, and the sortie
        // runner's enroute setup, had to know the other word (#67). An
        // unread wheels flag falls to the resume too, whose own gates read
        // the aircraft again and refuse with the number.
        let resume = a.resume || (a.flight && !on_ground);
        let (mission, ack) = if resume && on_ground {
            match self.build_flight_mission(ctx).await {
                Ok(v) => v,
                Err(why) => return Ok(why),
            }
        } else if resume {
            match self.build_recovery(ctx, Purpose::Resume).await {
                Ok(ResumeArm::Route(m, ack)) => (m, ack),
                Ok(ResumeArm::Arrival { dest, rwy, ack }) => {
                    return Ok(self.arm_arrival(ctx, &dest, rwy.as_deref(), ack, "resume_arrival"));
                }
                Ok(ResumeArm::Nearest(why)) | Err(why) => return Ok(why),
            }
        } else if a.flight {
            match self.build_flight_mission(ctx).await {
                Ok(v) => v,
                Err(why) => return Ok(why),
            }
        } else {
            (
                Mission::Circuit { right: a.right },
                format!(
                    "Full auto. I'll power up, start the {}, get our taxi, line us up, \
                     and fly the circuit. Say cancel or my controls any time.",
                    if self.engines > 1 {
                        "engines"
                    } else {
                        "engine"
                    }
                ),
            )
        };

        // "Take us to X": X must be where the plan goes, or nothing arms.
        if let (Some(to), Mission::Flight(r) | Mission::Resume(r)) = (&a.to, &mission) {
            if !names_destination(to, &a.to_codes, &r.dest_icao, &r.dest_name) {
                return Ok(format!(
                    "The plan goes to {} ({}), not {to}. Load a plan for {to} first.",
                    r.dest_name, r.dest_icao
                ));
            }
        }
        let sub = self.sub_tools(mission);
        if !crate::full_auto::start(
            ctx.config.xplane_base_url.clone(),
            ctx.config.clone(),
            ctx.voice.cloned(),
            sub,
        ) {
            return Ok(
                "The previous automation is still stopping. Ask again once it has stopped."
                    .to_string(),
            );
        }
        Ok(ack)
    }
}

impl FullAuto {

    /// Every sub-tool a mission flies with, built from this airframe's
    /// numbers: the one builder for the mission armed by voice and the
    /// landing a recovery arms (`arm_landing`).
    fn sub_tools(&self, mission: Mission) -> crate::full_auto::SubTools {
        let kind = start_kind(
            self.turboprop_start.is_some(),
            self.jet_start,
            self.piston_engine,
            self.scripted_auto_start,
            self.airliner_start.is_some(),
        );
        crate::full_auto::SubTools {
            lights: crate::tools::lights::SetLights::new(self.lights.clone()),
            flood: crate::tools::lights::SetFloodLights::new(
                self.flood_dataref.clone(),
                self.flood_toggle.clone(),
            ),
            no_start_sequence: kind == StartKind::None,
            // One start per airframe CLASS: a turboprop cranks with the PT6
            // sequence, a jet with the APU/bleed-air sequence, a piston with
            // the one-shot start. All run behind full auto's own bus-power
            // step. Getting this wrong is silent and total — the piston start
            // on a turbine cranks something that isn't there and reports
            // "check fuel and mixture" about levers the aircraft hasn't got.
            start_engine: match (kind, &self.turboprop_start) {
                (StartKind::Airliner, _) if self.airliner_start.is_some() => {
                    Box::new(crate::tools::jet_start::AirlinerStart::new(
                        self.airliner_start.clone().expect("checked"),
                        self.engines,
                    ))
                }
                (StartKind::Scripted, _) => Box::new(crate::tools::jet_start::ScriptedAutoStart),
                (StartKind::Turboprop, Some(ts)) => {
                    Box::new(crate::tools::cycle::StartTurbines::new(ts.clone()))
                }
                (StartKind::Jet, _) => Box::new(crate::tools::jet_start::StartEngines),
                _ => Box::new(crate::tools::engine::StartEngine {
                    engines: self.engines,
                    cycle_selectors: self.cycle_selectors.clone(),
                    fuel_selector: self.fuel_selector.clone(),
                    alternator_on_cmds: self.alternator_on_cmds.clone(),
                    bus_power_on_cmds: self.bus_power_on_cmds.clone(),
                }),
            },
            bus_power_on_cmds: self.bus_power_on_cmds.clone(),
            all_electrical_on_cmds: self.all_electrical_on_cmds.clone(),
            all_electrical_toggles: self.all_electrical_toggles.clone(),
            taxi: crate::tools::taxi::AutoTaxi {
                park_brake_valve_set: self.park_brake_valve_set,
                prop: self.prop.clone(),
                pace: self.pace,
                must_start: false,
            },
            line_up: crate::tools::taxi::LineUp {
                park_brake_valve_set: self.park_brake_valve_set,
                takeoff_roll_m: self.takeoff_roll_m,
                mtow_kg: self.mtow_kg,
                prop: self.prop.clone(),
                pace: self.pace,
            },
            circuit: crate::tools::circuit::Circuit {
                park_brake_valve_set: self.park_brake_valve_set,
                tail_strike_deg: self.tail_strike_deg,
                pitch_authority_down: self.pitch_authority_down,
                vne_kts: self.vne_kts,
                cfg: self.cfg.clone(),
                vr_kts: self.vr_kts,
                vy_kts: self.vy_kts,
                initial_climb_kts: self.initial_climb_kts,
                climb_fpm: self.climb_fpm,
                retract_gear: self.retract_gear,
                takeoff_flap_ratio: self.takeoff_flap_ratio,
                flap_schedule: self.flap_schedule.clone(),
                takeoff_roll_m: self.takeoff_roll_m,
                mtow_kg: self.mtow_kg,
                approach_ias: self.approach_ias,
                prop: self.prop.clone(),
                climb_floor_kts: self.climb_floor_kts,
                vs_clean_kts: self.vs_clean_kts,
            },
            takeoff: crate::tools::takeoff::AutoTakeoff {
                mtow_kg: self.mtow_kg,
                // An orchestrator takes this handoff; the field is unused.
                cruise_ias_kts: None,
                // The Flight mission arms the takeoff through this copy, and
                // the orchestrator takes the climb from the handoff — the
                // pilot must not be told "your aircraft" there.
                handoff_to_caller: true,
                park_brake_valve_set: self.park_brake_valve_set,
                tail_strike_deg: self.tail_strike_deg,
                pitch_authority_down: self.pitch_authority_down,
                vne_kts: self.vne_kts,
                cfg: self.cfg.clone(),
                vr_kts: self.vr_kts,
                vy_kts: self.vy_kts,
                initial_climb_kts: self.initial_climb_kts,
                climb_fpm: self.climb_fpm,
                retract_gear: self.retract_gear,
                takeoff_flap_ratio: self.takeoff_flap_ratio,
                flap_schedule: self.flap_schedule.clone(),
                takeoff_roll_m: self.takeoff_roll_m,
                prop: self.prop.clone(),
            },
            cruise_ias_kts: self.cruise_ias_kts,
            transition_altitude_ft: self.transition_altitude_ft,
            piston_mixture: self.piston_mixture,
            mission,
            power_up_first: self.power_up_first,
            engine_start_secs: kind.engine_start_secs(),
        }
    }

    /// Arm autoland on the destination's approach, as a planned arrival
    /// hands it down: the plan's runway when it names one, else the
    /// destination's best end (the self-pick keeps to the destination).
    fn arm_arrival(
        &self,
        ctx: &ToolCtx<'_>,
        dest: &str,
        rwy: Option<&str>,
        ack: String,
        why: &'static str,
    ) -> String {
        if crate::autoland::ACTIVE.load(std::sync::atomic::Ordering::SeqCst) {
            return "I'm already landing.".to_string();
        }
        crate::autoland::run_autoland_scoped(
            ctx.config.xplane_base_url.clone(),
            crate::tools::autoland::apt_dat_path(),
            self.cfg.clone(),
            self.approach_ias,
            ctx.voice.cloned(),
            ctx.status.cloned(),
            rwy.map(|r| crate::autoland::PlannedRunway::at(dest, r)),
            ctx.xplane.cancellation(),
        );
        crate::flight::note_crew_takeover(why);
        ack
    }

    /// "Land the aircraft" through the planner: the plan's destination
    /// from wherever we are (the climb, the join, its approach), or the
    /// nearest suitable runway when the pilot asked for it or the
    /// destination is impossible (`recovery_plan::landing_choice`).
    pub(crate) async fn arm_landing(
        &self,
        ctx: &ToolCtx<'_>,
    ) -> std::result::Result<LandArm, String> {
        if crate::full_auto::running() {
            return Err("I'm flying the plan already. Say my controls first to start over.".into());
        }
        match self.build_recovery(ctx, Purpose::Land).await? {
            ResumeArm::Nearest(why) => Ok(LandArm::Nearest(why)),
            ResumeArm::Arrival { dest, rwy, ack } => {
                Ok(LandArm::Armed(self.arm_arrival(ctx, &dest, rwy.as_deref(), ack, "land_destination")))
            }
            ResumeArm::Route(mission, ack) => {
                let sub = self.sub_tools(mission);
                if !crate::full_auto::start(
                    ctx.config.xplane_base_url.clone(),
                    ctx.config.clone(),
                    ctx.voice.cloned(),
                    sub,
                ) {
                    return Err("The previous automation is still stopping. Ask again once it has stopped.".into());
                }
                crate::flight::note_crew_takeover("land_destination");
                Ok(LandArm::Armed(ack))
            }
        }
    }

    /// The recovery planner's arm, every outcome blackboxed with its reason
    /// (five refusals in a row over the Gulf of Lion, 2026-09-16, and the
    /// debrief could not say why).
    async fn build_recovery(
        &self,
        ctx: &ToolCtx<'_>,
        purpose: Purpose,
    ) -> std::result::Result<ResumeArm, String> {
        let built = self.recovery(ctx, purpose).await;
        let land = purpose == Purpose::Land;
        match &built {
            Ok(ResumeArm::Route(_, ack)) => crate::full_auto::event(
                if land { "land_route" } else { "resume" },
                json!({ "ack": ack }),
            ),
            Ok(ResumeArm::Arrival { dest, rwy, ack }) => crate::full_auto::event(
                if land { "land_arrival" } else { "resume_arrival" },
                json!({ "dest": dest, "rwy": rwy, "ack": ack }),
            ),
            Ok(ResumeArm::Nearest(why)) => crate::full_auto::event("land_nearest", json!({ "why": why })),
            Err(why) => crate::full_auto::event(
                if land { "land_refused" } else { "resume_refused" },
                json!({ "why": why }),
            ),
        }
        built
    }

    /// The plan when none is on the shelf: fetched now (the SimBrief plan
    /// IS the destination), published to the shelf, blackboxed `plan_fetch`
    /// `fetched_airborne`. The line says why there is none.
    async fn fetch_airborne(
        &self,
        ctx: &ToolCtx<'_>,
    ) -> std::result::Result<crate::simbrief::FlightPlan, String> {
        if ctx.config.simbrief_id.trim().is_empty() {
            return Err("No flight plan is loaded and no SimBrief ID is set.".into());
        }
        match crate::simbrief::fetch(&ctx.config.simbrief_id).await {
            Ok(fp) => {
                crate::simbrief::publish_plan(&fp);
                crate::full_auto::event(
                    "plan_fetch",
                    json!({ "outcome": "fetched_airborne", "origin": fp.origin,
                            "destination": fp.destination, "route": fp.route }),
                );
                Ok(fp)
            }
            Err(e) => {
                tracing::warn!("simbrief fetch failed: {e:#}");
                crate::full_auto::event(
                    "plan_fetch",
                    json!({ "outcome": "failed_airborne", "error": format!("{e:#}") }),
                );
                Err("No flight plan is loaded, and SimBrief did not answer.".into())
            }
        }
    }

    /// FROM ANY AIRBORNE STATE, A SAFE PATH TO THE DESTINATION
    /// (`recovery_plan`). The plan on the shelf, else fetched; the join
    /// (`choose_join`: on the final, abeam a leg, the departure, the
    /// arrival, else direct to the fix with the shortest way in); the way's
    /// floor (the probe along it, else the grid MORA, else the fields'
    /// MSA) and a climb first when below it, on a heading the climb clears;
    /// refused only for the physically impossible, with the number (a
    /// landing request then goes to the nearest runway). LEMG 2026-10-05:
    /// three refusals in two minutes while the aircraft descended toward
    /// the hills.
    async fn recovery(
        &self,
        ctx: &ToolCtx<'_>,
        purpose: Purpose,
    ) -> std::result::Result<ResumeArm, String> {
        use crate::recovery_plan as rp;
        let land = purpose == Purpose::Land;
        let (fp, fetched) = match crate::simbrief::loaded_plan() {
            Some(p) => (p, false),
            None => match self.fetch_airborne(ctx).await {
                Ok(p) => (p, true),
                Err(why) if land => return Ok(ResumeArm::Nearest(why)),
                Err(why) => {
                    return Err(format!(
                        "{why} Load the flight plan, or say land for me for the nearest runway."
                    ))
                }
            },
        };
        let xp = crate::xplane::XPlaneClient::new(ctx.config.xplane_base_url.clone());
        // Position and heights are one frame (`sim::position`, #157).
        let (ground, pos, altitude, bank, ias, frozen, engines, aircraft, mag_var) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY),
            sim::position::read(&xp),
            ctx.xplane.get_float(crate::datarefs::ALTITUDE_FT),
            ctx.xplane.get_float(crate::datarefs::BANK_DEG),
            ctx.xplane.get_float(crate::datarefs::AIRSPEED_KTS),
            async {
                let (pause, replay) = tokio::join!(
                    ctx.xplane.get_float(crate::datarefs::PAUSED),
                    ctx.xplane.get_float(crate::datarefs::IS_IN_REPLAY)
                );
                !matches!((pause,replay), (Ok(p),Ok(r)) if p.is_finite() && r.is_finite() && p < 0.5 && r < 0.5)
            },
            ctx.xplane
                .get_float_array("sim/flightmodel/engine/ENGN_running"),
            xp.get_string("sim/aircraft/view/acf_ICAO"),
            ctx.xplane
                .get_float("sim/flightmodel/position/magnetic_variation"),
        );
        let (lat, lon) = sim::position::lat_lon(&pos);
        let agl = sim::position::field(&pos, |p| p.agl_m);
        let elevation = sim::position::field(&pos, |p| p.elev_m);
        let (pitch, flap, stall, fuel_kg, flow) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::PITCH_DEG),
            ctx.xplane.get_float(crate::datarefs::FLAP_RATIO),
            ctx.xplane.get_float("sim/cockpit2/annunciators/stall_warning_ratio"),
            ctx.xplane.get_float(crate::datarefs::FUEL_TOTAL_KG),
            ctx.xplane.get_float_array(crate::datarefs::FUEL_FLOW_KG_SEC),
        );
        let Some([ground, agl, lat, lon, altitude, bank, ias, elevation]) = crate::control::measurements(
            "resume",
            [ground, agl, lat, lon, altitude, bank, ias, elevation],
        ) else {
            return Err("I can't verify the aircraft state. I haven't taken control; try again once the sim is connected.".into());
        };
        // The flight the rejoin is checked against, for the recheck below.
        let clock = std::time::Instant::now();
        let first_read = steady_read(&xp, clock).await;
        let ok = |r: &anyhow::Result<f64>| r.as_ref().ok().copied().filter(|v| v.is_finite());
        let flap = ok(&flap).unwrap_or(0.0);
        let floor_kts = crate::aircraft::config_floor_kts(flap)
            .or_else(crate::aircraft::speed_floor_kts)
            .map(|f| f * 0.95);
        let unflyable =
            crate::tools::recover::unflyable(Some(ias), floor_kts, ok(&pitch), Some(bank), ok(&stall));
        if let Some(why) = resume_gate(frozen, sim::ground::on_ground(ground), unflyable) {
            return Err(why);
        }
        let running = engines.as_ref().ok().map(|v| {
            v.iter()
                .take(self.engines)
                .filter(|v| v.is_finite() && **v > 0.5)
                .count()
        });
        match running {
            Some(n) if n == self.engines => {}
            Some(n) if land => {
                return Ok(ResumeArm::Nearest(format!(
                    "Only {n} of {} engines are running.",
                    self.engines
                )))
            }
            Some(n) => {
                return Err(format!(
                    "I can confirm {n} of {} engine{} running. Say land at the nearest for a diversion.",
                    self.engines,
                    if self.engines == 1 { "" } else { "s" }
                ))
            }
            None => {
                return Err("I can't read the engines. I haven't taken control; try again once the sim answers.".into())
            }
        }
        // The plan's numbers are its type's; a landing goes to the place.
        if !land {
            match aircraft.as_ref().map(|a| a.trim().to_string()) {
                Ok(a) if a.eq_ignore_ascii_case(&fp.aircraft) => {}
                Ok(a) => {
                    return Err(format!(
                        "The loaded plan is for a {}, and this is a {a}. Review a matching plan first; you have control.",
                        fp.aircraft,
                    ))
                }
                Err(_) => {
                    return Err("I can't read this aircraft's type to check it against the plan. I haven't taken control.".into())
                }
            }
        }
        let mut legs = crate::procedure::enroute_legs(
            &fp.waypoints,
            &fp.origin,
            &fp.destination,
            fp.cruise_alt_ft,
        );
        let dest_wp = rp::destination_of(&fp.waypoints, &fp.destination).cloned();
        if let Some(dest) = dest_wp.as_ref() {
            crate::procedure::extend_to_arrival_gateway(&mut legs, dest);
        }
        // The OFP's planned steps, flown where it planned them (#101).
        record_step_climbs(&mut legs, &fp.schedule);
        // The plan runway's final, read only near the destination: the
        // apt.dat scan takes a moment.
        let near_dest = dest_wp.as_ref().is_some_and(|d| {
            crate::nav::haversine_nm(lat, lon, d.lat, d.lon) <= rp::arrival_radius_nm(purpose)
        });
        let dest_runways = if near_dest || fp.procedures.dest_rwy.is_some() {
            let apt = ctx.config.apt_data_path.clone();
            let dest = fp.destination.clone();
            tokio::task::spawn_blocking(move || crate::nav::find_runways(&apt, &dest).ok())
                .await
                .ok()
                .flatten()
                .unwrap_or_default()
        } else {
            Vec::new()
        };
        let final_fc = match (near_dest, fp.procedures.dest_rwy.as_deref()) {
            (true, Some(rwy)) => final_course(&dest_runways, rwy),
            _ => None,
        };
        let track_true = first_read.map(|r| r.track_true);
        let mag_var = mag_var.ok().filter(|v| v.is_finite());
        let Some(join) = rp::choose_join(
            &legs,
            &fp.waypoints,
            &fp.destination,
            lat,
            lon,
            mag_var,
            final_fc.as_ref(),
            track_true,
            purpose,
        ) else {
            return Err("The plan has no route and no destination I can fly to. Review the plan; you have control.".into());
        };
        // The way's floor: the ground along the path to the join, probed out
        // to 40 nm (the follower's own floor watches the rest in flight),
        // else the grid MORA along it, else the fields' sector altitude.
        let true_ft = elevation * 3.280_84;
        let agl_ft = agl * 3.280_84;
        let (to_lat, to_lon) = rp::join_point(&join, &legs, dest_wp.as_ref()).unwrap_or((lat, lon));
        let pts = rp::path_points(lat, lon, to_lat, to_lon);
        let mut heights: Option<Vec<f64>> = Some(Vec::with_capacity(pts.len()));
        for chunk in pts.chunks(crate::terrain::MAX_POINTS) {
            match (crate::terrain::sample_ft(&xp, chunk).await, heights.as_mut()) {
                (Some(h), Some(all)) => all.extend(h),
                _ => heights = None,
            }
        }
        let under = true_ft - agl_ft;
        let (floor_true, source) = match heights.as_ref().and_then(|h| h.iter().copied().reduce(f64::max)) {
            Some(high) => (high.max(under) + crate::procedure::TERRAIN_CLEARANCE_FT, "terrain"),
            None => {
                let mora = crate::plan_audit::mora_grid(&ctx.config.fix_data_path).and_then(|g| {
                    std::iter::once((lat, lon))
                        .chain(pts.iter().copied())
                        .filter_map(|(a, b)| g.at(a, b))
                        .reduce(f64::max)
                });
                let msa_path = std::path::Path::new(&ctx.config.fix_data_path)
                    .with_file_name("earth_msa.dat")
                    .to_string_lossy()
                    .into_owned();
                let msa = [&fp.origin, &fp.destination]
                    .into_iter()
                    .filter_map(|icao| crate::nav::airport_msa_ft(&msa_path, icao))
                    .reduce(f64::max);
                match (mora, msa) {
                    (Some(m), _) => (m.max(under + crate::procedure::TERRAIN_CLEARANCE_FT), "mora"),
                    (None, Some(m)) => (m, "msa"),
                    (None, None) => return Err(
                        "The terrain probe isn't answering and I have no grid MORA, so I can't check the way. Check the plugin build on the Health screen.".into(),
                    ),
                }
            }
        };
        let floor_ind = floor_true + (altitude - true_ft);
        let climb = rp::climb_first(altitude, floor_ind);
        // Only the physically impossible is refused.
        let flow_kg_s: Option<f64> = flow
            .ok()
            .map(|v| v.iter().filter(|x| x.is_finite() && **x > 0.0).sum::<f64>())
            .filter(|f| *f > 0.0);
        let endurance_min = match (ok(&fuel_kg), flow_kg_s) {
            (Some(kg), Some(f)) => Some(kg / f / 60.0),
            _ => None,
        };
        let gs_kts = first_read.map(|r| r.gs_kts).unwrap_or(ias);
        let to_go = rp::to_go_nm(&join, &legs, dest_wp.as_ref(), lat, lon);
        let runway = fp.procedures.dest_rwy.as_deref().and_then(|rwy| {
            let r = dest_runways.iter().find(|r| {
                crate::nav::same_runway_end(&r.ident_a, rwy) || crate::nav::same_runway_end(&r.ident_b, rwy)
            })?;
            let have = crate::nav::haversine_nm(r.lat_a, r.lon_a, r.lat_b, r.lon_b) * 1852.0;
            let need = crate::aircraft::landing_needed_m(crate::aircraft::landing_distance_m(), None, 0.0);
            (need > 0.0).then_some((rwy, have, need))
        });
        crate::full_auto::event(
            "recovery_plan",
            json!({
                "purpose": if land { "land" } else { "resume" }, "join": join.word(),
                "join_fix": join.leg_index().and_then(|i| legs.get(i)).map(|l| l.fix_ident.clone()),
                "floor_ft": floor_true.round(), "floor_source": source, "climb_ft": climb,
                "alt_ft": altitude.round(), "to_go_nm": to_go.round(), "endurance_min": endurance_min.map(f64::round),
                "fetched": fetched, "destination": fp.destination,
            }),
        );
        if let Some(why) = rp::impossible(
            &fp.destination,
            climb.unwrap_or(0.0),
            self.cfg.service_ceiling_ft,
            endurance_min,
            to_go,
            gs_kts,
            runway,
        ) {
            return if land {
                Ok(ResumeArm::Nearest(why))
            } else {
                Err(format!("{why} I'm not flying there; say land at the nearest for a diversion."))
            };
        }
        // The climb first, on the way's own bearing when the climb clears
        // its ground, else on the clear heading nearest it.
        let brg = crate::nav::bearing_deg(lat, lon, to_lat, to_lon);
        let mut climb_heading_mag: Option<f64> = None;
        let climb_leg = match climb {
            None => None,
            Some(alt) => {
                // The climb leg is a magnetic course: no variation read, no
                // course (a "0" flew the bug off by the whole variation).
                let Some(var) = mag_var else {
                    return Err(NO_VARIATION.into());
                };
                let grad = crate::safe_hold::gradient_ft_nm(self.cfg.climb.max_climb_fpm, gs_kts);
                let path_ray = heights.as_ref().map(|h| crate::safe_hold::Ray {
                    bearing_true: brg,
                    ground_ft: pts
                        .iter()
                        .map(|&(a, b)| crate::nav::haversine_nm(lat, lon, a, b))
                        .zip(h.iter().copied())
                        .collect(),
                });
                let clears = path_ray
                    .as_ref()
                    .is_none_or(|r| crate::safe_hold::climb_clears(r, true_ft, grad, true));
                let mut course_true = brg;
                let mut alt = alt;
                if !clears {
                    let air = crate::safe_hold::Air {
                        lat,
                        lon,
                        alt_ft: altitude,
                        alt_true_ft: true_ft,
                        agl_ft,
                        gs_kts,
                        track_true: brg,
                        mag_var: Some(var),
                        ..Default::default()
                    };
                    let fan = crate::safe_hold::probe_fan(&xp, &air).await;
                    if let Some((b, need)) = crate::safe_hold::clear_heading(&fan, &air, floor_true, grad) {
                        course_true = b;
                        alt = alt.max(crate::safe_hold::up_100(need + (altitude - true_ft)));
                    }
                }
                let course_mag = crate::runway::true_to_mag(course_true, var).round();
                climb_heading_mag = Some(course_mag);
                Some(rp::climb_leg(course_mag, alt))
            }
        };
        let climb_words = match (&climb_leg, climb_heading_mag) {
            (Some(l), Some(h)) => format!(
                "Climbing to {} first, heading {:03}, the safe altitude on the way. ",
                crate::flightplan::spoken_feet(l.alt1_ft.unwrap_or(0.0)),
                (h.round() as i64).rem_euclid(360)
            ),
            _ => String::new(),
        };
        let lead = if fetched {
            format!("Fetched your plan, {} to {}. ", fp.origin, fp.destination)
        } else {
            String::new()
        };
        // The arrival now, with nothing to climb first: autoland's.
        let arrival_join = matches!(join, rp::Join::OnFinal { .. } | rp::Join::Arrival { .. });
        if arrival_join && climb_leg.is_none() {
            let rwy = fp.procedures.dest_rwy.clone();
            let ack = match (&join, rwy.as_deref()) {
                (rp::Join::OnFinal { along_nm, .. }, Some(r)) => format!(
                    "{lead}On the final for runway {}, {along_nm:.0} miles out. Flying the approach to {}. Say my controls to take over.",
                    atc::atis::spoken_runway(r),
                    fp.destination,
                ),
                (rp::Join::Arrival { dist_nm }, Some(r)) => format!(
                    "{lead}{dist_nm:.0} miles from {}. Flying the approach to runway {}. Say my controls to take over.",
                    fp.destination,
                    atc::atis::spoken_runway(r),
                ),
                (_, None) => format!(
                    "{lead}{to_go:.0} miles from {}. Flying the approach to its best runway for the wind. Say my controls to take over.",
                    fp.destination,
                ),
                _ => format!("{lead}Flying the approach to {}. Say my controls to take over.", fp.destination),
            };
            return Ok(ResumeArm::Arrival { ack, dest: fp.destination.clone(), rwy });
        }
        // Clamped before the drain: a resume inside the arrival still owes
        // the restrictions ahead, and the profile rule runs from the STAR's
        // first fix.
        let (arrival_constraints, _) =
            crate::plan_audit::arrival_restrictions(&ctx.config.cifp_dir, &fp, &mut legs);
        let fix_name = |i: usize| legs.get(i).map(|l| l.fix_ident.clone()).unwrap_or_default();
        let join_words = match &join {
            rp::Join::Abeam(i) => format!("Rejoining the route toward {}", fix_name(*i)),
            rp::Join::Departure => format!("Joining the route at {}", fix_name(0)),
            rp::Join::Direct { index, dist_nm, .. } => {
                format!("Direct {}, {dist_nm:.0} miles", fix_name(*index))
            }
            rp::Join::OnFinal { .. } | rp::Join::Arrival { .. } => {
                format!("Then the arrival into {}, {to_go:.0} miles", fp.destination)
            }
        };
        let mut legs = match join.leg_index() {
            Some(i) => {
                legs.drain(..i.min(legs.len()));
                legs
            }
            None => Vec::new(),
        };
        if let Some(first) = legs.first() {
            let target = first.alt1_ft.unwrap_or(fp.cruise_alt_ft);
            if !target.is_finite() || target <= 0.0 || target > self.cfg.service_ceiling_ft {
                return Err(format!(
                    "The plan's {target:.0} feet at {} is outside this aircraft's {:.0} foot ceiling. Review the plan; you have control.",
                    first.fix_ident, self.cfg.service_ceiling_ft
                ));
            }
        }
        if let Some(c) = climb_leg {
            legs.insert(0, c);
        }
        let apt = ctx.config.apt_data_path.clone();
        let cifp = ctx.config.cifp_dir.clone();
        let fixes = ctx.config.fix_data_path.clone();
        let dest = fp.destination.clone();
        let rwy = fp.procedures.dest_rwy.clone();
        let arrival = tokio::task::spawn_blocking(move || {
            crate::full_auto::brief_arrival(&apt, &cifp, &fixes, &dest, rwy.as_deref())
        })
        .await
        .map_err(|_| "Arrival verification failed; you have control.".to_string())??;
        // Probes can take time: never arm from a position that has moved away
        // from the verified join while the pilot was waiting.
        // Judged against the first read's own track and level, never raw
        // movement: steady cruise moves (#177).
        let (Some(was), Some(now)) = (first_read, steady_read(&xp, clock).await) else {
            return Err("Connection changed during verification. I haven't taken control.".into());
        };
        if let Some(why) = rejoin_unsteady(&was, &now) {
            crate::full_auto::event(
                "rejoin_unsteady",
                json!({ "dt_s": now.t_s - was.t_s, "was": format!("{was:?}"),
                        "now": format!("{now:?}") }),
            );
            return Err(why);
        }
        let ack = format!(
            "{lead}{climb_words}{join_words}, continuing to {} and the arrival. {arrival} Say my controls to take over.",
            fp.destination
        );
        // THE one payload builder, never a hand-rolled join. This site had a
        // byte-identical copy of its entry format without the middle-thinning
        // or the 511-byte cap, so a resume overwrote full auto's fitted
        // payload with the raw navlog: EIDW-BIKF, 2026-09-20, 528 bytes into
        // the 512-byte mailbox, cut to "BIKF,6". The plugin dropped the
        // malformed tail, ELVUM became the last entry, and X-Plane refused
        // the plan it composed — "invalid destination airport: ELVUM". The
        // arm got this right at 07:30 and the resume undid it at 07:49.
        let payload = crate::simbrief::fms_route_payload(&fp.waypoints);
        // The flight's transition, as the arm publishes it: a resume after a
        // restart ran on the profile's US 18,000 ft and reset the knob to
        // QNH at FL180 over Germany, whose transition is 5,000 (EDLP-LOWI,
        // 2026-09-24). Already descending past the top of descent is the
        // follower's to note.
        crate::tools::instruments::publish_flight_transition(Some(
            crate::tools::instruments::FlightTransition {
                altitude_ft: fp.transition_altitude_ft,
                level_ft: fp.transition_level_ft,
                descending: false,
            },
        ));
        Ok(ResumeArm::Route(
            Mission::Resume(Route {
                arrival_rwy: fp.procedures.dest_rwy.clone(),
                arrival_star: fp.procedures.star.clone(),
                dest_icao: fp.destination.clone(),
                dest_name: fp.destination,
                climb_minimums: Vec::new(),
                arrival_constraints,
                departure_rwy: None,
                origin_icao: fp.origin.clone(),
                origin_rwy: None,
                legs,
                fms_payload: payload,
                // A joined route is checked here, on the way flown.
                local_runway: None,
                sid: None,
                // Airborne: no clearance to ask for.
                ifr: false,
                cruise_schedule: fp.schedule.clone(),
                cruise_ft: Some(fp.cruise_alt_ft).filter(|c| *c > 0.0),
            }),
            ack,
        ))
    }

    /// The Flight mission's arm, with what it did on the way said first: a
    /// changed or stale plan and the load (#95) ride ahead of the ack, and
    /// ahead of a refusal, which then names the number the load left.
    async fn build_flight_mission(
        &self,
        ctx: &ToolCtx<'_>,
    ) -> std::result::Result<(Mission, String), String> {
        let mut notes: Vec<String> = Vec::new();
        let built = self.flight_mission(ctx, &mut notes).await;
        let said = |line: String| {
            notes
                .iter()
                .cloned()
                .chain(std::iter::once(line))
                .filter(|l| !l.is_empty())
                .collect::<Vec<_>>()
                .join(" ")
        };
        match built {
            Ok((mission, ack)) => Ok((mission, said(ack))),
            Err(why) => Err(said(why)),
        }
    }

    /// Fetch the SimBrief OFP and distil it to a flyable [`Mission::Flight`] —
    /// or an honest spoken refusal (no plan, no waypoints, or a plan that
    /// departs somewhere we aren't). On the ground and stopped, the plan is
    /// fetched fresh, the aircraft loaded from it, THEN preflighted: the plan
    /// the FMS gets is the plan loaded and the plan preflighted (#95).
    async fn flight_mission(
        &self,
        ctx: &ToolCtx<'_>,
        notes: &mut Vec<String>,
    ) -> std::result::Result<(Mission, String), String> {
        // Airborne, the mission cannot arm — and the refusal must say what
        // DOES fly up here. The origin check below used to answer first,
        // telling a pilot enroute at 8,000 ft "we're not at LXGB... say full
        // auto for a circuit here" (2026-08-02). A failed read falls
        // through: the sequence re-checks on its own thread.
        if ctx
            .xplane
            .get_float(crate::datarefs::ON_GROUND_ANY)
            .await
            .is_ok_and(|og| og < 0.5)
        {
            return Err(
                "We're already airborne, so I can't start the flight from cold. Say navigate \
                 to follow the plan, fly the ILS for the arrival, or land for me."
                    .to_string(),
            );
        }
        // The plan, fetched fresh (`PlanFetch::Asked`'s rule): a local plan is
        // the pilot's own and stays; a SimBrief plan is fetched every arm,
        // because the pilot regenerates it on the website and says the same
        // words; the shelf flies when the site does not answer, said so.
        let before = crate::simbrief::loaded_plan();
        let mut fetch_error: Option<String> = None;
        let (mut fp, fetch) = if let Some(local) = crate::simbrief::local_plan() {
            (local, crate::full_auto::ArmFetch::Local)
        } else {
            if ctx.config.simbrief_id.trim().is_empty() {
                return Err("No flight plan. Say plan a flight to, and a destination, or configure SimBrief.".into());
            }
            match crate::simbrief::fetch(&ctx.config.simbrief_id).await {
                Ok(fp) => {
                    let judged =
                        crate::full_auto::ArmFetch::judge(false, true, before.as_ref(), &fp);
                    // The Route tab's load sheet shows the plan being loaded.
                    crate::simbrief::publish_plan(&fp);
                    (fp, judged)
                }
                Err(e) => {
                    tracing::warn!("simbrief fetch failed: {e:#}");
                    fetch_error = Some(format!("{e:#}"));
                    match before.clone() {
                        Some(shelf) => (shelf, crate::full_auto::ArmFetch::Fallback),
                        None => {
                            return Err("Couldn't fetch your SimBrief plan. Generate one first, \
                                        or check the ID."
                                .to_string())
                        }
                    }
                }
            }
        };
        crate::full_auto::event(
            "plan_fetch",
            json!({
                "outcome": format!("{fetch:?}").to_lowercase(),
                "origin": fp.origin, "destination": fp.destination, "route": fp.route,
                "before": before.as_ref().map(|b| format!("{} {} {}", b.origin, b.route, b.destination)),
                "error": fetch_error,
            }),
        );
        notes.extend(fetch.line(&fp));

        // The plan must depart from where we're parked — flying LEJR's route
        // out of some other field would thread the wrong corridor entirely.
        let (Ok(lat), Ok(lon)) = (
            ctx.xplane
                .get_float("sim/flightmodel/position/latitude")
                .await,
            ctx.xplane
                .get_float("sim/flightmodel/position/longitude")
                .await,
        ) else {
            return Err(
                "I can't read our position, so I can't line the plan up with where we are."
                    .to_string(),
            );
        };
        let apt_dat = super::autoland::apt_dat_path();
        let here = crate::autoland::nearby_runways(&apt_dat, lat, lon, 4.0)
            .unwrap_or_default()
            .into_iter()
            .any(|r| r.icao.eq_ignore_ascii_case(&fp.origin));
        if !here {
            return Err(format!(
                "Your plan departs {} but we're not there. Generate a plan from this field, \
                 or say full auto for a circuit here.",
                fp.origin
            ));
        }

        // Both fields from one blocking pass: the destination's name for the
        // ack, and each end's longest runway for the preflight.
        let (origin_apt, dest_apt) = {
            let (path, o, d) = (
                ctx.config.apt_data_path.clone(),
                fp.origin.clone(),
                fp.destination.clone(),
            );
            tokio::task::spawn_blocking(move || {
                (
                    crate::nav::find_airport(&path, &o).ok().flatten(),
                    crate::nav::find_airport(&path, &d).ok().flatten(),
                )
            })
            .await
            .unwrap_or((None, None))
        };
        let local_runway = if fp.source == crate::simbrief::PlanSource::Local {
            let origin = origin_apt
                .as_ref()
                .ok_or("The origin airport is missing from the data.")?;
            let dest = dest_apt
                .as_ref()
                .ok_or("The destination airport is missing from the data.")?;
            let speed = self
                .cruise_ias_kts
                .ok_or("The airframe needs a cruise speed to plan this local flight.")?;
            let route = crate::flightplan::local_route(
                &fp,
                origin,
                dest,
                speed,
                self.climb_fpm,
                crate::aircraft::planned_landing_needed_ft(
                    self.landing_distance_ft,
                    self.takeoff_distance_ft,
                )
                .unwrap_or(0.0),
                crate::weather::surface_wind(&super::autoland::xplane_root(), &fp.destination),
            )?;
            let xp = crate::xplane::XPlaneClient::new(ctx.config.xplane_base_url.clone());
            // The probe answers only inside the loaded scenery (about 100 nm
            // round the aircraft): a chunk beyond it is judged on the grid
            // MORA's highest ground, which errs high. LEMG-LEBL, 413 nm, was
            // refused outright on 2026-09-28.
            let fix = ctx.config.fix_data_path.clone();
            let mora = tokio::task::spawn_blocking(move || crate::plan_audit::mora_grid(&fix))
                .await
                .ok()
                .flatten();
            let mut heights = Vec::new();
            let mut from_grid = 0usize;
            let mut grid_cautions = 0usize;
            for (ci, chunk) in route.terrain.chunks(crate::terrain::MAX_POINTS).enumerate() {
                let pts: Vec<_> = chunk.iter().map(|(p, _)| *p).collect();
                match crate::terrain::sample_ft(&xp, &pts).await {
                    Some(h) => heights.extend(h),
                    None => {
                        let base = ci * crate::terrain::MAX_POINTS;
                        let grid = mora.as_ref().ok_or(
                            "I can't verify terrain along the local route: no probe beyond \
                             the loaded scenery and no MORA grid.",
                        )?;
                        // The grid is a caution, never a refusal (the plan
                        // audit's rule): a one-degree cell's highest peak
                        // stands over the whole cell, so it is logged and the
                        // follower's own terrain floor watches it in flight.
                        for (k, &(lat, lon)) in pts.iter().enumerate() {
                            let m = grid.at(lat, lon).ok_or(
                                "I can't verify terrain along the local route: a leg is \
                                 outside the MORA grid.",
                            )?;
                            let floor = route.terrain[base + k].1;
                            let ground = crate::nav::mora_ground_ft(m);
                            if ground > floor {
                                grid_cautions += 1;
                            }
                            heights.push(ground.min(floor));
                        }
                        from_grid += pts.len();
                    }
                }
            }
            crate::blackbox::record(
                "full_auto",
                json!({"event": "local_terrain", "points": heights.len(), "from_grid": from_grid,
                       "grid_cautions": grid_cautions}),
            );
            if !crate::flightplan::local_terrain_clear(&route, &heights) {
                // Name the first point that failed, so a refusal can be
                // judged (LEMG-LEBL was refused with no point, 2026-09-28).
                let bad = heights
                    .iter()
                    .zip(&route.terrain)
                    .enumerate()
                    .find(|(_, (h, (_, floor)))| !h.is_finite() || **h > *floor);
                let n = route.terrain.len();
                if let Some((i, (h, ((lat, lon), floor)))) = bad {
                    crate::blackbox::record(
                        "full_auto",
                        json!({"event": "local_terrain_blocked", "index": i, "of": n,
                               "lat": lat, "lon": lon, "ground_ft": h, "floor_ft": floor}),
                    );
                }
                return Err("Terrain blocks the local climb or arrival corridor. Choose another destination or route.".into());
            }
            // Price the authored track, including the final alignment, into
            // the same preflight fuel check. Reserve stays untouched.
            if fp.distance_nm > 0.0 {
                // All kilos now, so the scaling needs no density at all — the
                // `* 2.72` here was the avgas constant a third time.
                let extra = fp.trip_fuel_kg * (route.distance_nm / fp.distance_nm - 1.0).max(0.0);
                fp.trip_fuel_kg += extra;
                fp.block_fuel_kg += extra;
                fp.min_takeoff_fuel_kg += extra;
                fp.ete_min *= route.distance_nm / fp.distance_nm;
            }
            fp.distance_nm = route.distance_nm;
            fp.waypoints = route.waypoints;
            Some(route.runway)
        } else {
            None
        };

        let mut legs = crate::procedure::enroute_legs(
            &fp.waypoints,
            &fp.origin,
            &fp.destination,
            fp.cruise_alt_ft,
        );
        // The OFP's planned steps, flown where it planned them (#101).
        record_step_climbs(&mut legs, &fp.schedule);
        // The arrival is the crew's: the STAR to the runway the wind will
        // give us, when SimBrief planned the other end.
        let star_line = crate::plan_audit::choose_arrival(
            &ctx.config.cifp_dir,
            &ctx.config.fix_data_path,
            &ctx.config.nav_data_path,
            &ctx.config.apt_data_path,
            crate::weather::surface_wind(&super::autoland::xplane_root(), &fp.destination),
            &mut fp,
            &mut legs,
        );
        let star_rechosen = star_line.is_some();
        let (arrival_constraints, arrival_line) =
            crate::plan_audit::arrival_restrictions(&ctx.config.cifp_dir, &fp, &mut legs);
        if matches!(fp.source, crate::simbrief::PlanSource::SimBrief) {
            if let Some(dest) = fp
                .waypoints
                .last()
                .filter(|w| w.ident.eq_ignore_ascii_case(&fp.destination))
            {
                crate::procedure::extend_to_arrival_gateway(&mut legs, dest);
            }
        }
        let arrival_line = match (star_line, arrival_line) {
            (Some(a), Some(b)) => Some(format!("{a} {b}")),
            (a, b) => a.or(b),
        };
        if legs.is_empty() {
            return Err(format!(
                "Your plan to {} has no enroute waypoints I can fly. Generate it with a route.",
                fp.destination
            ));
        }

        // The planned SID's minimum altitudes, where its fixes are on this
        // route (SimBrief flies the SID as plain waypoints with its own
        // predicted altitudes; the chart's minimums are what keep a climb out
        // of the rocks).
        let climb_minimums: Vec<crate::nav::ClimbMinimum> =
            match (&fp.procedures.sid, &fp.procedures.origin_rwy) {
                (Some(sid), Some(rwy)) => {
                    crate::nav::sid_minimums(&ctx.config.cifp_dir, &fp.origin, sid, rwy)
                        .into_iter()
                        .filter(|m| legs.iter().any(|l| l.fix_ident.eq_ignore_ascii_case(&m.fix)))
                        .collect()
                }
                _ => Vec::new(),
            };
        // A minimum above the plan's cruise is not flown (the plan owns the
        // level); said here, so it is no surprise three miles from the fix.
        let (climb_minimums, above_cruise) =
            crate::procedure::minimums_within_cruise(climb_minimums, fp.cruise_alt_ft);
        let above_line = above_cruise.iter().max_by(|a, b| a.min_ft.total_cmp(&b.min_ft)).map(|m| {
            format!(
                "{} wants {} at {}, above our cruise. I'll hold the plan's {} there.",
                fp.procedures.sid.as_deref().unwrap_or("The departure"),
                crate::flightplan::spoken_feet(m.min_ft),
                m.fix,
                crate::flightplan::spoken_feet(fp.cruise_alt_ft),
            )
        });
        // The gradient each minimum needs from the runway's departure end,
        // against the climb at Vy: refused when it cannot be made, said when
        // it is tight, briefed either way.
        let departure_line = match (&fp.procedures.sid, &fp.procedures.origin_rwy, &origin_apt) {
            (Some(sid), Some(rwy), Some(apt)) if !climb_minimums.is_empty() => {
                let der = apt.runways.iter().find_map(|r| {
                    if r.ident_a.eq_ignore_ascii_case(rwy) {
                        Some((r.lat_b, r.lon_b))
                    } else if r.ident_b.eq_ignore_ascii_case(rwy) {
                        Some((r.lat_a, r.lon_a))
                    } else {
                        None
                    }
                });
                match der {
                    Some(der) => {
                        let chart = crate::full_auto::sid_chart_legs(
                            &ctx.config.cifp_dir,
                            &ctx.config.fix_data_path,
                            &ctx.config.nav_data_path,
                            apt,
                            &fp.origin,
                            sid,
                            rwy,
                        );
                        let checks = crate::full_auto::departure_gradients(
                            der,
                            apt.elev_ft,
                            if chart.is_empty() { &legs } else { &chart },
                            &climb_minimums,
                            self.vy_kts,
                            crate::procedure::best_climb_fpm(),
                        );
                        crate::full_auto::event(
                            "departure_gradient",
                            json!({ "checks": checks.iter().map(|c| json!({"fix": c.fix, "min_ft": c.min_ft,
                                "dist_nm": c.dist_nm, "need": c.need_ft_per_nm, "have": c.have_ft_per_nm})).collect::<Vec<_>>() }),
                        );
                        crate::full_auto::gradient_verdict(sid, &checks)?
                    }
                    None => None,
                }
            }
            _ => None,
        };
        crate::full_auto::event(
            "sid_minimums",
            json!({ "sid": fp.procedures.sid, "runway": fp.procedures.origin_rwy,
                    "minimums": climb_minimums.iter().map(|m| (m.fix.clone(), m.min_ft)).collect::<Vec<_>>() }),
        );
        let dest_name = dest_apt
            .as_ref()
            .map(|a| a.name.trim().to_string())
            .filter(|n| !n.is_empty())
            .unwrap_or_else(|| fp.destination.clone());

        // The load sheet, before the preflight judges it (#95): on the
        // ground and stopped, the halves the aircraft does not already carry
        // are loaded from THIS plan through the load tool's own path (wings
        // first, payload into m_fixed, read back). Its refusals are said and
        // the preflight below then refuses with its number, as ever.
        let (og, gs, fuel0, payload0) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::ON_GROUND_ANY),
            ctx.xplane.get_float(super::loading::GROUNDSPEED_MS),
            ctx.xplane.get_float(crate::datarefs::FUEL_TOTAL_KG),
            ctx.xplane.get_float(crate::datarefs::PAYLOAD_KG),
        );
        let stopped = sim::ground::stopped_on_ground(og.ok(), gs.ok());
        let decision = crate::full_auto::arm_load(
            &fp,
            stopped,
            fuel0.as_ref().ok().copied(),
            payload0.as_ref().ok().copied(),
            self.load_sheet,
            &self.airframe,
        );
        let (outcome, load_line) = match &decision {
            crate::full_auto::ArmLoad::Load(what) => {
                match super::loading::load_from_plan(ctx.xplane, &fp, *what).await {
                    Ok(line) => (crate::full_auto::arm_load_outcome(&line), Some(line)),
                    Err(e) => {
                        tracing::warn!("arm load failed: {e:#}");
                        ("failed", Some("The load didn't go through.".to_string()))
                    }
                }
            }
            crate::full_auto::ArmLoad::Declined(line) => (decision.outcome(), Some(line.clone())),
            _ => (decision.outcome(), None),
        };
        crate::full_auto::event(
            "arm_load",
            json!({
                "outcome": outcome,
                "what": match &decision {
                    crate::full_auto::ArmLoad::Load(w) => Some(format!("{w:?}")),
                    _ => None,
                },
                "fuel_before_kg": fuel0.ok(), "block_kg": fp.block_fuel_kg,
                "min_takeoff_kg": fp.min_takeoff_fuel_kg,
                "payload_before_kg": payload0.ok(),
                "plan_payload_kg": fp.weights.as_ref().map(|w| w.payload_kg),
                "stopped": stopped,
                "load_sheet": format!("{:?}", self.load_sheet),
                "line": load_line,
            }),
        );
        notes.extend(load_line);

        // The preflight: tanks, load and all-up weight as the sim has them
        // now, against the plan and the book. A refusal here costs nothing;
        // the same shortfall found airborne is a diversion.
        let (fuel, payload, total) = tokio::join!(
            ctx.xplane.get_float(crate::datarefs::FUEL_TOTAL_KG),
            ctx.xplane.get_float(crate::datarefs::PAYLOAD_KG),
            ctx.xplane.get_float(crate::datarefs::TOTAL_WEIGHT_KG),
        );
        let facts = crate::full_auto::PreflightFacts {
            fuel_kg: fuel.ok(),
            payload_kg: payload.ok(),
            total_kg: total.ok(),
            max_gross_lb: self.max_gross_lb,
            takeoff_roll_m: self.takeoff_roll_m,
            landing_distance_ft: self.landing_distance_ft,
            takeoff_distance_ft: self.takeoff_distance_ft,
            origin_longest_ft: origin_apt
                .as_ref()
                .map_or(0.0, crate::flightplan::longest_runway_ft),
            dest_longest_ft: dest_apt
                .as_ref()
                .map_or(0.0, crate::flightplan::longest_runway_ft),
            service_ceiling_ft: Some(self.cfg.service_ceiling_ft),
        };
        let waived = crate::full_auto::waived_for(&fp);
        let preflight = crate::full_auto::preflight_gated(&fp, &facts, &waived);
        crate::full_auto::event(
            "preflight",
            json!({
                "origin": fp.origin, "destination": fp.destination,
                "fuel_kg": facts.fuel_kg, "block_kg": fp.block_fuel_kg,
                "min_takeoff_kg": fp.min_takeoff_fuel_kg,
                "payload_kg": facts.payload_kg,
                "plan_payload_kg": fp.weights.as_ref().map(|w| w.payload_kg),
                "pax": fp.weights.as_ref().map(|w| w.pax),
                "cargo_kg": fp.weights.as_ref().map(|w| w.cargo_kg),
                "total_kg": facts.total_kg,
                "max_tow_kg": fp.weights.as_ref().map(|w| w.max_tow_kg),
                "origin_longest_ft": facts.origin_longest_ft,
                "dest_longest_ft": facts.dest_longest_ft,
                "refused": preflight.as_ref().err().map(|(_, why)| why),
                "gate": preflight.as_ref().err().map(|(g, _)| format!("{g:?}")),
                "waived": waived.iter().map(|g| format!("{g:?}")).collect::<Vec<_>>(),
            }),
        );
        // A refusal that is only a MARGIN is the pilot's to waive: offered,
        // never taken ("fly it anyway" inside the window re-arms).
        let preflight = preflight.map_err(|(gate, why)| {
            if gate.waivable() {
                crate::full_auto::note_refusal(gate, &fp);
                format!("{why} If you've checked it yourself, say fly it anyway.")
            } else {
                why
            }
        })?;
        crate::full_auto::RUNWAY_WAIVER_PENDING.store(
            waived.contains(&crate::full_auto::Gate::OriginRunway),
            std::sync::atomic::Ordering::SeqCst,
        );

        // The whole flight, dry-run before anything moves: is this aircraft
        // fit for this plan, does the climb clear the ground here, does the
        // route sit under the grid's safe altitudes, can the STAR's
        // restrictions be made (`plan_audit`). A refusal names the numbers;
        // cautions ride the ack.
        let audit_xp = crate::xplane::XPlaneClient::new(ctx.config.xplane_base_url.clone());
        let suit = crate::plan_audit::gather_suitability(
            &audit_xp,
            &fp,
            origin_apt.as_ref(),
            dest_apt.as_ref(),
        )
        .await;
        let audit = crate::plan_audit::check(
            &audit_xp,
            &ctx.config.fix_data_path,
            &fp,
            &legs,
            &arrival_constraints,
            origin_apt.as_ref().map_or(0.0, |a| a.elev_ft),
            crate::plan_audit::live_airframe(Some(self.cfg.service_ceiling_ft)),
            Some(&suit),
            true,
        )
        .await;
        if let Some(stop) = audit.stop() {
            return Err(stop.line.clone());
        }
        let audit_line = audit
            .cautions()
            .next()
            .is_some()
            .then(|| crate::plan_audit::spoken(&audit, false));

        // The flight's own transition altitude and level, ahead of the
        // airframe's default, for every altimeter rule from here on (the
        // departure set, the follower, the monitor's check). Cleared with
        // the mission (`full_auto::abort` / stand-down).
        crate::tools::instruments::publish_flight_transition(Some(
            crate::tools::instruments::FlightTransition {
                altitude_ft: fp.transition_altitude_ft,
                level_ft: fp.transition_level_ft,
                descending: false,
            },
        ));

        // The GPS route payload — written by the orchestrator (only the
        // concrete client can set a byte-array dataref).
        let mut fms_payload = crate::simbrief::fms_route_payload(&fp.waypoints);
        // The crew's own STAR goes into the GPS too, not SimBrief's.
        if star_rechosen {
            fms_payload = crate::full_auto::payload_with_legs(&fms_payload, &legs);
        }

        // The arrival, briefed NOW: what the destination offers and what
        // full auto will do there. Found at the gate, it is a briefing item;
        // found 25 nm out, it was a crash (Gibraltar, 2026-09-11). A
        // destination the data can't land at is refused here, not there.
        let arrival = if let Some(runway) = &local_runway {
            format!(
                "I'll fly the checked visual final on runway {}.",
                crate::circuit::spell_runway(runway)
            )
        } else {
            // Briefed on the runway the arm chose, the one the arrival flies.
            let (apt, cifp, fixp, dest, rwy) = (
                ctx.config.apt_data_path.clone(),
                ctx.config.cifp_dir.clone(),
                ctx.config.fix_data_path.clone(),
                fp.destination.clone(),
                fp.procedures.dest_rwy.clone(),
            );
            tokio::task::spawn_blocking(move || {
                crate::full_auto::brief_arrival(&apt, &cifp, &fixp, &dest, rwy.as_deref())
            })
            .await
            .map_err(|e| format!("The arrival brief failed: {e}"))??
        };

        crate::simbrief::publish_plan(&fp);

        let n = legs.len();
        let preflight = if preflight.is_empty() {
            String::new()
        } else {
            format!("{preflight} ")
        };
        // The ack is the plan's NUMBERS, not a description of full auto: the
        // pilot who says "fly the flight plan" knows what happens next, and
        // the arrival line already names the destination (EGSS-LFPG,
        // 2026-09-22, a paragraph the panel ran out of room for).
        let preflight = match &departure_line {
            Some(l) => format!("{preflight}{l} "),
            None => preflight,
        };
        let preflight = match &above_line {
            Some(l) => format!("{preflight}{l} "),
            None => preflight,
        };
        let preflight = match &arrival_line {
            Some(l) => format!("{preflight}{l} "),
            None => preflight,
        };
        let preflight = match &audit_line {
            Some(l) => format!("{preflight}{l} "),
            None => preflight,
        };
        // The whole brief is READ, on the Route tab; what is SAID is the
        // flight, anything the pilot waived and the plan's cautions. The
        // recited brief was a paragraph in the pilot's ear every arm (LIME
        // 2026-09-25: "this would be better in a tab").
        let brief = format!(
            "{} to {}. {}Cruise {}, {} enroute {}. {}",
            fp.origin,
            fp.destination,
            preflight,
            crate::flightplan::spoken_feet(fp.cruise_alt_ft),
            n,
            if n == 1 { "fix" } else { "fixes" },
            arrival,
        );
        if let Some(st) = ctx.status {
            st.brief(&format!("Full auto brief\n{brief}"));
        }
        let waived: String = preflight
            .split_inclusive(". ")
            .filter(|s| s.contains("your call"))
            .collect();
        // The departure flown as charted, where the follower can fly every
        // leg of it (`charted_departure_legs`): spliced in after the audit,
        // which judges the plan's own fixes.
        if let (Some(sid), Some(rwy), Some(apt)) =
            (&fp.procedures.sid, &fp.procedures.origin_rwy, &origin_apt)
        {
            let chart = crate::full_auto::sid_chart_legs(
                &ctx.config.cifp_dir,
                &ctx.config.fix_data_path,
                &ctx.config.nav_data_path,
                apt,
                &fp.origin,
                sid,
                rwy,
            );
            // The join: the last charted fix the plan also flies.
            let join = chart.iter().rev().find_map(|c| {
                legs.iter()
                    .position(|l| !c.fix_ident.is_empty() && l.fix_ident.eq_ignore_ascii_case(&c.fix_ident))
                    .map(|i| (i, c.fix_ident.clone()))
            });
            if let Some((i, join)) = join {
                if let Some(dep) = crate::full_auto::charted_departure_legs(&chart, &legs, &join) {
                    if dep.iter().any(|l| l.fix.is_none()) {
                        crate::full_auto::event(
                            "departure_charted",
                            json!({ "sid": sid, "join": join,
                                    "legs": dep.iter().map(|l| format!("{} {}", l.leg_type, l.fix_ident)).collect::<Vec<_>>() }),
                        );
                        let rest: Vec<crate::nav::ProcLeg> = legs.drain(i + 1..).collect();
                        legs = dep;
                        legs.extend(rest);
                    }
                }
            }
        }
        let ack = spoken_arm_line(
            &fp.origin,
            &fp.destination,
            fp.cruise_alt_ft,
            &waived,
            audit_line.as_deref(),
            ctx.status.is_some(),
        );
        Ok((
            Mission::Flight(Route {
                // The arm's runway, carried to the approach: the local
                // route's checked final, else the one the STAR flown serves.
                arrival_rwy: local_runway.clone().or_else(|| fp.procedures.dest_rwy.clone()),
                arrival_star: fp.procedures.star.clone(),
                dest_icao: fp.destination.clone(),
                dest_name,
                departure_rwy: fp.procedures.sid.as_ref().and(fp.procedures.origin_rwy.clone()),
                origin_icao: fp.origin.clone(),
                origin_rwy: fp.procedures.origin_rwy.clone(),
                climb_minimums,
                arrival_constraints,
                legs,
                fms_payload,
                local_runway,
                sid: fp.procedures.sid.clone(),
                ifr: crate::full_auto::plan_is_ifr(
                    fp.source,
                    fp.cruise_alt_ft,
                    fp.transition_altitude_ft.unwrap_or(self.transition_altitude_ft),
                ),
                cruise_schedule: fp.schedule.clone(),
                cruise_ft: Some(fp.cruise_alt_ft).filter(|c| *c > 0.0),
            }),
            ack,
        ))
    }
}

/// The OFP's planned step climbs put on the legs
/// (`procedure::apply_step_climbs`) and blackboxed with the schedule the
/// follower will fly (`full_auto` `cruise_schedule`): cost index, cruise
/// profile, cruise-stage fixes, the steps placed and any not on the route.
fn record_step_climbs(legs: &mut [crate::nav::ProcLeg], schedule: &crate::simbrief::CruiseSchedule) {
    let placed = crate::procedure::apply_step_climbs(legs, schedule);
    if schedule.fixes.is_empty() && schedule.steps.is_empty() {
        return;
    }
    let unplaced: Vec<&str> = schedule
        .steps
        .iter()
        .skip(1)
        .filter(|s| !placed.iter().any(|p| p.fix == s.fix))
        .map(|s| s.fix.as_str())
        .collect();
    crate::blackbox::record(
        "full_auto",
        serde_json::json!({
            "event": "cruise_schedule",
            "cost_index": schedule.cost_index,
            "profile": schedule.profile,
            "cruise_fixes": schedule.fixes.iter().filter(|f| f.stage == crate::simbrief::Stage::Cruise).count(),
            "initial_ft": schedule.steps.first().map(|s| s.alt_ft),
            "steps": placed.iter().map(|p| serde_json::json!({"fix": p.fix, "idx": p.idx, "alt_ft": p.alt_ft})).collect::<Vec<_>>(),
            "unplaced": unplaced,
        }),
    );
}

/// The arming ack as SPOKEN: the flight, the pilot's own waivers, the
/// plan's cautions, and where the rest is. Without a panel the brief has
/// nowhere to be read, so it says only the essentials. Pure.
pub fn spoken_arm_line(
    origin: &str,
    dest: &str,
    cruise_ft: f64,
    waived: &str,
    cautions: Option<&str>,
    panel: bool,
) -> String {
    let mut line = format!(
        "I'll fly from {origin} to {dest}, cruise {}. ",
        crate::flightplan::spoken_feet(cruise_ft)
    );
    line.push_str(waived.trim());
    if !waived.trim().is_empty() {
        line.push(' ');
    }
    if let Some(c) = cautions {
        line.push_str(c.trim());
        line.push(' ');
    }
    if panel && cautions.is_some() {
        line.push_str("Details on the brief. ");
    }
    // How to take it back, the first arm of a session only (the taxi's
    // `take_back_hint` rule: a person says it once).
    static SAID: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
    if !SAID.swap(true, std::sync::atomic::Ordering::Relaxed) {
        line.push_str("Cancel or my controls any time.");
    }
    line.trim_end().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recovery_plan::Join;

    #[test]
    fn take_us_to_is_checked_against_the_plan() {
        assert!(names_destination("bologna", &[], "LIPE", "Bologna Borgo Panigale"));
        assert!(names_destination("LIPE", &["LIPE".into()], "LIPE", "Bologna Borgo Panigale"));
        assert!(!names_destination("milan", &[], "LIPE", "Bologna Borgo Panigale"));
        assert!(!names_destination("lipk", &["LIPK".into()], "LIPE", "Bologna Borgo Panigale"));
    }

    #[test]
    fn the_arm_says_the_flight_and_the_route_tab_holds_the_brief() {
        // The first arm of a session says how to take it back; the next
        // does not. No caution, no pointer to a brief with nothing in it.
        let l = spoken_arm_line("LIME", "LIPE", 19000.0, "", None, true);
        assert_eq!(l, "I'll fly from LIME to LIPE, cruise 19,000 feet. Cancel or my controls any time.");
        let again = spoken_arm_line("LIME", "LIPE", 19000.0, "", None, true);
        assert_eq!(again, "I'll fly from LIME to LIPE, cruise 19,000 feet.");
        let w = spoken_arm_line(
            "LILB",
            "LIPK",
            25000.0,
            "No runway margin, your call: 1,970 feet of runway, 1,640 feet needed without it. ",
            Some("One thing on the plan."),
            true,
        );
        assert!(w.contains("your call") && w.contains("One thing on the plan"), "{w}");
        assert!(w.contains("Details on the brief"), "{w}");
    }

    /// Every shipped profile is exactly ONE airframe class, and full auto
    /// picks that class's start. The MD-82 is the case that found the gap:
    /// `turbine_start` with no `turboprop_start` fell through to the piston
    /// start, and full auto sat at a cold jet cranking a starter it doesn't
    /// have, telling the pilot to check a mixture lever it doesn't have
    /// either (Gatwick, 2026-07-27).
    #[test]
    fn full_auto_starts_each_airframe_the_way_its_engines_start() {
        for (id, want) in [
            ("cessna_172sp", StartKind::Piston),
            ("cirrus_sr22", StartKind::Piston),
            ("dc3", StartKind::Piston),
            ("lancair_evolution", StartKind::Turboprop),
            ("dhc6_twin_otter", StartKind::Turboprop),
            ("king_air_c90b", StartKind::Turboprop),
            ("md82", StartKind::Jet),
            // The Citation X is a turbofan, but the generic crank/fuel/idle
            // sequence fits it once the fuel goes in on N2 — so it starts
            // through the same path a PT6 does, not the MD-82's APU/bleed one.
            ("citation_x", StartKind::Turboprop),
            // The stock 737 scripts its own start behind `auto_start`.
            ("boeing_737_800", StartKind::Scripted),
            // ENG MODE knob and guarded masters. The standalone tool was
            // registered for this aircraft and full auto still said "I don't
            // have an engine start for this aircraft" — the class it
            // dispatches on had never heard of the block (2026-09-20).
            ("airbus_a330_300", StartKind::Airliner),
        ] {
            let p = crate::aircraft::load(id).unwrap_or_else(|| panic!("{id} profile loads"));
            let got = start_kind(
                p.turboprop_start.is_some(),
                p.turbine_start,
                p.engine || p.piston_start.is_some(),
                p.scripted_auto_start,
                p.airliner_start.is_some(),
            );
            // Whatever class it lands in, the mission must be able to BUILD
            // a start for it: a class with no tool behind it is the hole
            // this test exists to close.
            assert!(
                got != StartKind::None || p.airliner_start.is_none(),
                "{id}: declares an airliner start but classes as None"
            );
            assert_eq!(got, want, "{id}");
            // A jet has no piston engine group to fall back on, which is
            // exactly why the wrong start is silent and total.
            if got == StartKind::Jet {
                assert!(!p.engine, "{id}: a jet declares no piston engine group");
            }
        }
    }

    /// The wait for the engine has to cover the start it just fired. The jet
    /// sequence's own internal waits bound it near three minutes (bleed
    /// pressure, then crank/fuel/idle twice), so a deadline shorter than that
    /// gives up on a start that is still running normally — which is what a
    /// single 40 s figure did at Luton on 2026-07-27.
    #[test]
    fn the_engine_wait_outlasts_the_start_it_fired() {
        let jet = StartKind::Jet.engine_start_secs();
        assert!(jet >= 200.0, "jet start deadline too short: {jet}");
        assert!(jet > StartKind::Piston.engine_start_secs());
        assert!(StartKind::Piston.engine_start_secs() >= 30.0);
        assert!(StartKind::Turboprop.engine_start_secs() >= 30.0);
    }

    fn plan(points: &[(&str, f64, f64)]) -> Vec<crate::simbrief::Waypoint> {
        points
            .iter()
            .map(|(id, lat, lon)| crate::simbrief::Waypoint {
                ident: id.to_string(),
                lat: *lat,
                lon: *lon,
                alt: 5000.0,
            })
            .collect()
    }

    /// The recovery planner's join for a resume (`recovery_plan::choose_join`).
    fn resume_join(
        legs: &[crate::nav::ProcLeg],
        waypoints: &[crate::simbrief::Waypoint],
        destination: &str,
        lat: f64,
        lon: f64,
        mag_var: Option<f64>,
        fc: Option<&FinalCourse>,
        trk: Option<f64>,
    ) -> Join {
        crate::recovery_plan::choose_join(
            legs, waypoints, destination, lat, lon, mag_var, fc, trk, Purpose::Resume,
        )
        .expect("a join")
    }

    /// The two recorded resume refusals of #67, joined from their positions.
    /// LSZS-LILO (SR22, 2026-09-24, t 1790265324.8): 3.3 nm out of Samedan at
    /// 7,412 ft, "8 miles right of course on the 4634N01010E to 4626N01003E
    /// leg" behind "climb to 15,500 feet first"; it is a departure joining
    /// its first fix. LIPE-LIPK (172, 2026-09-25, t 1790370381.1): 6.5 nm
    /// from Forli at 2,800 ft, abeam no leg with LUMAV 41 nm behind, "climb
    /// to 7,500 feet first"; it is the arrival.
    #[test]
    fn a_resume_near_a_field_joins_the_departure_or_flies_the_arrival() {
        let lszs = plan(&[
            ("LSZS", 46.5341, 9.8841),
            ("4636N00959E", 46.6, 9.98333),
            ("4642N01006E", 46.7, 10.1),
            ("4639N01011E", 46.65, 10.18333),
            ("4634N01010E", 46.56667, 10.16667),
            ("4626N01003E", 46.43333, 10.05),
            ("4610N00957E", 46.16667, 9.95),
            ("LILO", 46.1581, 9.8869),
        ]);
        let legs = crate::procedure::enroute_legs(&lszs, "LSZS", "LILO", 10400.0);
        let (lat, lon) = (46.57445, 9.93876);
        assert!(crate::interaction::rejoin_index(&legs, lat, lon, None)
            .is_err_and(|w| w.contains("4634N01010E to 4626N01003E")));
        assert_eq!(resume_join(&legs, &lszs, "LILO", lat, lon, None, None, None), Join::Departure);

        let lipk = plan(&[
            ("LIPE", 44.5309, 11.2922),
            ("D037N", 44.715, 11.50),
            ("LUMAV", 44.81, 11.61),
            ("LIPK", 44.1948, 12.0701),
        ]);
        let legs = crate::procedure::enroute_legs(&lipk, "LIPE", "LIPK", 5000.0);
        let (lat, lon) = (44.13632, 12.20075);
        match resume_join(&legs, &lipk, "LIPK", lat, lon, None, None, None) {
            Join::Arrival { dist_nm } => assert!((6.0..7.0).contains(&dist_nm), "{dist_nm}"),
            other => panic!("the arrival, not {other:?}"),
        }
        // Past the route and 29 nm out, refused as "LIPK is 29 miles away"
        // until 2026-10-05: its fixes are 40 nm back, so the arrival direct.
        match resume_join(&legs, &lipk, "LIPK", 43.75, 12.35, None, None, None) {
            Join::Arrival { dist_nm } => assert!((28.0..31.0).contains(&dist_nm), "{dist_nm}"),
            other => panic!("the arrival direct, not {other:?}"),
        }
        // Back up the route, beside it outside the corridor: direct too,
        // never a refusal.
        assert!(matches!(
            resume_join(&legs, &lipk, "LIPK", 44.95, 11.2, None, None, None),
            Join::Direct { .. }
        ));
    }

    /// EGKK-EGBB, the pilot's real OFP (STAR HEME1B to 33: PUFAX, HON,
    /// OSKOT, GROVE). Sortie 1790760672-approach (iMac 2026-09-30): the 737
    /// on the 33 final, "my controls", then "resume flight plan" at t
    /// 1790760727.7 from 52.2743 N 1.5492 W tracking 326 true, 13 nm from
    /// EGBB, was refused "4 miles left of course on the PUFAX to HON leg":
    /// GROVE, the navlog's last fix, is nearer EGBB than the final, so the
    /// past-the-route arrival never applied. On the plan's final it is the
    /// arrival.
    #[test]
    fn a_resume_on_the_plans_final_flies_the_arrival() {
        let v: serde_json::Value =
            serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/ofp/egkk-egbb-738.trimmed.json")))
                .unwrap();
        let fp = crate::simbrief::plan_from_ofp(&v).unwrap();
        assert_eq!(fp.procedures.dest_rwy.as_deref(), Some("33"));
        let legs =
            crate::procedure::enroute_legs(&fp.waypoints, &fp.origin, &fp.destination, 18_000.0);
        // EGBB's apt.dat 100 row: 15 at the north-west end, 33 the south-east.
        let egbb = [crate::nav::Runway {
            ident_a: "15".into(),
            ident_b: "33".into(),
            lat_a: 52.4656497,
            lon_a: -1.7611282,
            lat_b: 52.4428974,
            lon_b: -1.7358498,
            width_m: 46.02,
            surface: 1,
        }];
        let fc = final_course(&egbb, "33").unwrap();
        assert!((fc.course_true - 326.0).abs() < 2.0, "{fc:?}");
        let (lat, lon, trk) = (52.274337015882715, -1.5492269078597638, 326.0);
        let join = |lat, lon, fc: Option<&FinalCourse>, trk| {
            resume_join(&legs, &fp.waypoints, &fp.destination, lat, lon, Some(-1.0), fc, trk)
        };
        // As flown, no final to judge against: the recorded refusal ("4
        // miles left of course on the PUFAX to HON leg") is now the arrival,
        // 13 nm from EGBB.
        assert!(matches!(join(lat, lon, None, Some(trk)), Join::Arrival { .. }));
        // With the plan runway's final: on it, 12 miles out on the line.
        match join(lat, lon, Some(&fc), Some(trk)) {
            Join::OnFinal { dist_nm, along_nm } => {
                assert!((12.0..14.0).contains(&dist_nm), "{dist_nm}");
                assert!((11.0..13.0).contains(&along_nm), "{along_nm}");
                let f = on_final(&fc, lat, lon, trk).unwrap();
                assert!(f.xtk_nm.abs() < 0.5, "{f:?}");
                assert!(f.track_off_deg < 2.0, "{f:?}");
            }
            other => panic!("the final, not {other:?}"),
        }
        // Crossing the final, not flying it: the arrival, never the final.
        assert!(matches!(join(lat, lon, Some(&fc), Some(240.0)), Join::Arrival { .. }));
        // No track read: never judged a final.
        assert!(matches!(join(lat, lon, Some(&fc), None), Join::Arrival { .. }));
        // Aligned but 5 nm beside the centreline, outside the corridor.
        let (olat, olon) = crate::nav::project_latlon(lat, lon, fc.course_true + 90.0, 5.0);
        assert!(!matches!(join(olat, olon, Some(&fc), Some(trk)), Join::OnFinal { .. }));
        // Past the threshold on the runway's heading: behind the final.
        let (plat, plon) = crate::nav::project_latlon(fc.thr_lat, fc.thr_lon, fc.course_true, 6.0);
        assert!(!matches!(join(plat, plon, Some(&fc), Some(trk)), Join::OnFinal { .. }));
        // Too far out: 30 nm on the extended centreline is not the arrival.
        let (flat, flon) =
            crate::nav::project_latlon(fc.thr_lat, fc.thr_lon, fc.course_true + 180.0, 30.0);
        assert!(!matches!(
            join(flat, flon, Some(&fc), Some(trk)),
            Join::Arrival { .. } | Join::OnFinal { .. }
        ));
    }

    fn steady(t_s: f64, lat: f64, lon: f64, alt_ft: f64, vs: f64, gs: f64, trk: f64) -> Steady {
        Steady { t_s, lat, lon, alt_ft, vs_fpm: vs, gs_kts: gs, track_true: trk }
    }

    /// #177 from the blackbox. Sortie 1790635335's cruise after the resume
    /// (state records +31.5 and +37.1 s): 0.6 nm on in 5.6 s, still on its
    /// own track and level. The same flight twelve seconds on, 1.3 nm (the
    /// old raw-movement rule refused past 1 nm), still steady.
    #[test]
    fn steady_cruise_is_not_movement() {
        let a = steady(0.0, 52.102_544_76, -0.708_129_74, 18_629.0, -743.4, 395.8, 329.0);
        let b = steady(5.6, 52.111_251_57, -0.716_650_71, 18_576.1, -689.5, 390.8, 328.8);
        assert_eq!(rejoin_unsteady(&a, &b), None);
        let run = 395.8 * 12.0 / 3600.0;
        let (lat, lon) = crate::nav::project_latlon(a.lat, a.lon, a.track_true, run);
        assert!(crate::nav::haversine_nm(a.lat, a.lon, lat, lon) > 1.0);
        let c = steady(12.0, lat, lon, 18_629.0 - 743.4 * 0.2, -743.4, 395.8, 329.0);
        assert_eq!(rejoin_unsteady(&a, &c), None);
    }

    /// The refused resume itself (2026-09-29, t 1790648838.7 and 840.2):
    /// the air start's zoom through 18,155 and 18,436 ft at 9,300 to
    /// 12,100 fpm. Still refused, and the line says why.
    #[test]
    fn the_air_starts_zoom_is_still_refused() {
        let a = steady(0.0, 52.055_444_29, -0.663_412_19, 18_155.1, 9_309.5, 352.3, 330.3);
        let b = steady(1.5, 52.057_500_98, -0.665_280_77, 18_436.4, 12_057.1, 337.6, 331.1);
        let why = rejoin_unsteady(&a, &b).expect("a zoom is not steady");
        assert!(why.contains("climbing at 12057 feet a minute"), "{why}");
        // A jump off the track (a relocation) is refused on the track.
        let (lat, lon) = crate::nav::project_latlon(a.lat, a.lon, 60.0, 2.0);
        let c = steady(1.5, lat, lon, 18_155.1, 0.0, 352.3, 330.3);
        let a0 = Steady { vs_fpm: 0.0, ..a };
        let why = rejoin_unsteady(&a0, &c).expect("off the track");
        assert!(why.contains("left the track"), "{why}");
        // And a level that leaves the projection by 250 ft.
        let d = steady(1.5, a.lat, a.lon, 18_405.0, 0.0, 0.0, 330.3);
        let a1 = Steady { gs_kts: 0.0, ..a0 };
        assert!(rejoin_unsteady(&a1, &d).is_some());
    }
}
