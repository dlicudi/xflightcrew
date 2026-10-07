//! The arm's gates: the fetch and load, the preflight, the margins and their
//! waiver, the runway start and the lined-up check.

use super::*;

/// How the arm came by its plan (#95): what is said and blackboxed
/// (`full_auto` `plan_fetch`) before the load and the preflight.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmFetch {
    /// A local plan: the pilot's own, never swapped for SimBrief.
    Local,
    /// SimBrief answered with the plan already on the shelf (or the first).
    Fetched,
    /// SimBrief answered with a different plan than the shelf's.
    Changed,
    /// SimBrief did not answer: the shelf's plan is flown.
    Fallback,
}

impl ArmFetch {
    /// What `before` (the shelf) and `after` (what the arm flies) say about
    /// the fetch. Route identity is `plan::same_route`'s, as the "wrong
    /// plan" check's. Pure.
    pub fn judge(
        local: bool,
        fetched: bool,
        before: Option<&crate::simbrief::FlightPlan>,
        after: &crate::simbrief::FlightPlan,
    ) -> Self {
        if local {
            ArmFetch::Local
        } else if !fetched {
            ArmFetch::Fallback
        } else if before.is_some_and(|b| !crate::plan::same_route(b, after)) {
            ArmFetch::Changed
        } else {
            ArmFetch::Fetched
        }
    }

    /// The one short line the arm says for it, if any. Pure.
    pub fn line(self, fp: &crate::simbrief::FlightPlan) -> Option<String> {
        match self {
            ArmFetch::Local | ArmFetch::Fetched => None,
            ArmFetch::Changed => Some(format!(
                "New plan from SimBrief, {} to {}.",
                fp.origin, fp.destination
            )),
            ArmFetch::Fallback => Some(format!(
                "Couldn't reach SimBrief, flying the plan I already had, {} to {}.",
                fp.origin, fp.destination
            )),
        }
    }
}

/// Fuel within this of the plan's block is aboard already: two percent,
/// never tighter than a few kilos. An absolute 100 kg would swallow a 172's
/// whole 109 kg block.
pub(super) const ARM_FUEL_FRAC: f64 = 0.02;
pub(super) const ARM_FUEL_KG: f64 = 5.0;
/// Payload within this of the load sheet is aboard already: two percent,
/// never tighter than one bag.
pub(super) const ARM_PAYLOAD_FRAC: f64 = 0.02;
pub(super) const ARM_PAYLOAD_KG: f64 = 20.0;

/// What the arm does with the plan's load sheet before the preflight.
#[derive(Debug, Clone, PartialEq)]
pub enum ArmLoad {
    /// Load these halves with `tools::loading::load_from_plan`.
    Load(crate::tools::loading::What),
    /// Everything the plan carries is aboard already: nothing is written.
    Matches,
    /// Not on the ground and stopped: the arm never loads a rolling aircraft.
    NotStopped,
    /// The airframe's load is unprobed or its own: the line names it and
    /// the load is the pilot's.
    Declined(String),
}

impl ArmLoad {
    /// The blackbox outcome before the loader has spoken.
    pub fn outcome(&self) -> &'static str {
        match self {
            ArmLoad::Load(_) => "load",
            ArmLoad::Matches => "skipped_matches",
            ArmLoad::NotStopped => "skipped_not_stopped",
            ArmLoad::Declined(_) => "refused_airframe",
        }
    }
}

/// The arm's load decision (#95), from the tanks and payload as read NOW.
/// A half the plan does not carry (no block, no load sheet) is never asked
/// for; a half whose read failed is asked for, so the loader reads again
/// and refuses on a failed read itself. Pure.
pub fn arm_load(
    fp: &crate::simbrief::FlightPlan,
    stopped_on_ground: bool,
    fuel_kg: Option<f64>,
    payload_kg: Option<f64>,
    sheet: crate::aircraft::LoadSheet,
    airframe: &str,
) -> ArmLoad {
    use crate::tools::loading::What;
    if !stopped_on_ground {
        return ArmLoad::NotStopped;
    }
    let within = |have: Option<f64>, want: f64, frac: f64, floor: f64| {
        have.is_some_and(|h| (h - want).abs() <= floor.max(want * frac))
    };
    let fuel = fp.block_fuel_kg > 0.0
        && !within(fuel_kg, fp.block_fuel_kg, ARM_FUEL_FRAC, ARM_FUEL_KG);
    let payload = fp.weights.as_ref().is_some_and(|w| {
        w.payload_kg > 0.0 && !within(payload_kg, w.payload_kg, ARM_PAYLOAD_FRAC, ARM_PAYLOAD_KG)
    });
    if !fuel && !payload {
        return ArmLoad::Matches;
    }
    match sheet {
        crate::aircraft::LoadSheet::Stock => ArmLoad::Load(match (fuel, payload) {
            (true, true) => What::Both,
            (true, false) => What::Fuel,
            _ => What::Payload,
        }),
        crate::aircraft::LoadSheet::Unprobed => ArmLoad::Declined(format!(
            "Loading the {airframe} from the plan is unprobed, so the fuel and payload are yours to set."
        )),
        crate::aircraft::LoadSheet::OwnFuelSystem => ArmLoad::Declined(format!(
            "The {airframe} manages its own fuel, so the fuel and payload are yours to set."
        )),
    }
}

/// What the load tool's line says it did, for the blackbox: every line
/// the loader returns starts with its verdict. Pure.
pub fn arm_load_outcome(line: &str) -> &'static str {
    if line.starts_with("Loaded for") {
        "loaded"
    } else if line.starts_with("Loading for") {
        "not_taken"
    } else {
        "refused"
    }
}

/// What the arm-time preflight compares the plan against: the tanks and the
/// load as the sim has them NOW, the airframe's book figures, and the longest
/// runway at each end. `None`/0 = unknown, and an unknown is never a refusal.
#[derive(Debug, Clone, Default)]
pub struct PreflightFacts {
    pub fuel_kg: Option<f64>,
    pub payload_kg: Option<f64>,
    pub total_kg: Option<f64>,
    pub max_gross_lb: Option<f64>,
    pub takeoff_roll_m: Option<f64>,
    pub landing_distance_ft: Option<f64>,
    pub takeoff_distance_ft: Option<f64>,
    pub origin_longest_ft: f64,
    pub dest_longest_ft: f64,
    /// The airframe's ceiling; `None` skips the cruise check.
    pub service_ceiling_ft: Option<f64>,
}

pub(super) const LB_PER_KG: f64 = 2.204_62;
/// A tank within this of the block is "as planned"; SimBrief rounds too.
pub(super) const FUEL_MATCH_FRAC: f64 = 0.03;
/// Payload within this of the plan (and 20 kg) is "as planned".
pub(super) const PAYLOAD_MATCH_FRAC: f64 = 0.10;
pub(super) const PAYLOAD_MATCH_KG: f64 = 20.0;

/// The plan against the aircraft, BEFORE full auto starts an engine: the
/// destination's longest runway against the landing distance, this field's
/// against the takeoff roll, the all-up weight against the max takeoff weight,
/// the tanks against the plan's block and minimum takeoff fuel, and the
/// payload against the load sheet. `Err` is a refusal that names the number;
/// `Ok` is the brief (fuel and load, each said with its figures) for the
/// arming ack. Every figure the check lacks is skipped, never guessed: a plan
/// with no load sheet is briefed on fuel alone. The pilot asked "does the
/// copilot check the fuel and the runway" (2026-09-15); it did neither at the
/// arm, and found the fuel at the enroute caution and the runway at the hold.
#[cfg(test)]
pub fn preflight(fp: &crate::simbrief::FlightPlan, f: &PreflightFacts) -> Result<String, String> {
    preflight_gated(fp, f, &[]).map_err(|(_, why)| why)
}

/// The preflight gate that refused, so the pilot can waive the ones that
/// are a MARGIN and never the ones that are the airframe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Gate {
    Ceiling,
    /// The destination's longest runway under the book landing distance:
    /// unfactored already, so there is no margin to give up.
    DestRunway,
    /// This field's runway under the FACTORED roll. Waived, the unfactored
    /// book roll must still fit.
    OriginRunway,
    /// This field's runway under the book roll itself: no margin left to
    /// give up.
    BookRoll,
    /// Over the plan's operational takeoff limit. Waived, the airframe's
    /// own max gross still holds.
    PlanWeight,
    MaxWeight,
    Fuel,
}

impl Gate {
    pub fn waivable(self) -> bool {
        matches!(self, Gate::OriginRunway | Gate::PlanWeight)
    }
    pub(super) fn spoken(self) -> &'static str {
        match self {
            Gate::OriginRunway => "the runway margin",
            Gate::PlanWeight => "the plan's weight limit",
            _ => "that",
        }
    }
}

/// A pilot's waiver: named gates, for ONE plan (origin and destination).
#[derive(Debug, Clone, PartialEq)]
pub struct Waiver {
    pub origin: String,
    pub destination: String,
    pub gates: Vec<Gate>,
}

/// The waivers the pilot has given, for the plan they were given for.
pub(super) static WAIVER: std::sync::Mutex<Option<Waiver>> = std::sync::Mutex::new(None);
/// The last waivable refusal: what "fly it anyway" answers, for
/// [`WAIVE_WINDOW`] after it was said.
pub(super) static REFUSED: std::sync::Mutex<Option<(Gate, String, String, Instant)>> =
    std::sync::Mutex::new(None);
/// The arm's preflight passed on a waived runway margin; `start` hands it
/// to the mission (`forget_mission`) or it dies here.
pub static RUNWAY_WAIVER_PENDING: AtomicBool = AtomicBool::new(false);
/// How long after a refusal "fly it anyway" still means that refusal.
pub const WAIVE_WINDOW: Duration = Duration::from_secs(120);

pub(super) fn lock<T>(m: &std::sync::Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// The gates the pilot waived for this plan (another plan's waiver is
/// dropped here: it never carries to a different flight).
pub fn waived_for(fp: &crate::simbrief::FlightPlan) -> Vec<Gate> {
    let mut w = lock(&WAIVER);
    match w.as_ref() {
        Some(v) if v.origin == fp.origin && v.destination == fp.destination => v.gates.clone(),
        Some(_) => {
            *w = None;
            Vec::new()
        }
        None => Vec::new(),
    }
}

/// A waivable refusal was just said.
pub fn note_refusal(gate: Gate, fp: &crate::simbrief::FlightPlan) {
    note_refusal_for(gate, &fp.origin, &fp.destination);
}

/// [`note_refusal`] from a mission, which carries the plan's two ends.
pub fn note_refusal_for(gate: Gate, origin: &str, destination: &str) {
    *lock(&REFUSED) = Some((gate, origin.to_string(), destination.to_string(), Instant::now()));
}

/// Lined up with too little runway ahead: the unfactored need, metres, when
/// the runway ahead covers it and only the margin is short (and the margin
/// is not already waived). `None` = short without the margin too. Pure.
pub fn margin_only_short(remaining_m: f64, needed_m: f64, waived: bool) -> Option<f64> {
    if waived {
        return None;
    }
    let bare = (needed_m / crate::aircraft::TAKEOFF_FACTOR).max(LINE_UP_FLOOR_M);
    (bare < needed_m && remaining_m >= bare).then_some(bare)
}
/// The takeoff fit's own floor (`runway_check`'s `takeoff_needed_now_m`).
pub(super) const LINE_UP_FLOOR_M: f64 = 500.0;

/// The pilot said "fly it anyway": waive the refusal just said, if one is
/// inside the window, and return the words that re-arm, with the gate.
/// `None` = nothing to waive (the words are then nobody's).
pub fn take_waiver() -> Option<Gate> {
    let (gate, origin, destination, at) = lock(&REFUSED).take()?;
    if at.elapsed() > WAIVE_WINDOW {
        return None;
    }
    let mut w = lock(&WAIVER);
    match w.as_mut() {
        Some(v) if v.origin == origin && v.destination == destination => {
            if !v.gates.contains(&gate) {
                v.gates.push(gate);
            }
        }
        _ => {
            *w = Some(Waiver {
                origin,
                destination,
                gates: vec![gate],
            })
        }
    }
    event("waiver", json!({ "gate": format!("{gate:?}") }));
    Some(gate)
}

/// The acknowledgement of a waiver: what was given up, and that it is the
/// pilot's call.
pub fn waiver_ack(gate: Gate) -> String {
    format!("Your call, {} waived for this plan. Arming.", gate.spoken())
}

/// The arm's preflight with the pilot's waivers applied; a refusal names
/// its gate.
pub fn preflight_gated(
    fp: &crate::simbrief::FlightPlan,
    f: &PreflightFacts,
    waived: &[Gate],
) -> Result<String, (Gate, String)> {
    let mut waived_lines: Vec<String> = Vec::new();
    use crate::flightplan::spoken_feet;
    // Fuel in the plan's own unit when it carries a load sheet, otherwise the
    // AIRFRAME's — never the app's avgas gallons, which is what a local plan
    // used to fall back to whatever it was flying; a weight in the plan's
    // unit, or pounds (the profile's `max_gross_lb`) when there is no sheet.
    let spoken_fuel = |kg: f64| match &fp.weights {
        Some(w) => w.spoken(kg),
        None => crate::aircraft::say_fuel_kg(kg),
    };
    let spoken_weight = |kg: f64| match &fp.weights {
        Some(w) => w.spoken(kg),
        None => crate::simbrief::spoken_weight(kg * LB_PER_KG, "pounds"),
    };

    // A cruise the aircraft cannot reach is not a plan: EDLP-LOWI's FL290
    // was armed on an airframe declared to 28,000 ft, and the refusal came
    // in the air, at the resume after a pilot takeover (2026-09-24).
    if let Some(ceiling) = f.service_ceiling_ft.filter(|c| *c > 0.0) {
        if fp.cruise_alt_ft > ceiling {
            return Err((
                Gate::Ceiling,
                format!(
                    "Negative. The plan cruises at {}, above this aircraft's ceiling of {}. \
                     Replan lower.",
                    spoken_feet(fp.cruise_alt_ft),
                    spoken_feet(ceiling)
                ),
            ));
        }
    }
    // The far end first: a field we cannot land at is not a destination.
    if let Some(need) =
        crate::aircraft::planned_landing_needed_ft(f.landing_distance_ft, f.takeoff_distance_ft)
    {
        if f.dest_longest_ft > 0.0 && f.dest_longest_ft < need {
            return Err((
                Gate::DestRunway,
                format!(
                    "Negative. {}'s longest runway is {}, and we need {} to land. Not flying that.",
                    fp.destination,
                    spoken_feet(f.dest_longest_ft),
                    spoken_feet(need)
                ),
            ));
        }
    }
    // This end: the pavement the roll needs, factored, or the line-up would
    // find out at the hold after the start and the taxi.
    // The roll is quoted at the PROFILE's max gross, whatever the plan's
    // operational limit is; scaled to the weight on the wheels now.
    let need_at = |factor: f64| {
        crate::aircraft::takeoff_needed_factored_m(
            f.takeoff_roll_m,
            f.total_kg,
            f.max_gross_lb.map(|lb| lb / LB_PER_KG),
            500.0,
            factor,
        ) * M_TO_FT
    };
    let need_ft = need_at(crate::aircraft::TAKEOFF_FACTOR);
    if f.takeoff_roll_m.is_some() && f.origin_longest_ft > 0.0 && f.origin_longest_ft < need_ft {
        // Waived, the margin goes and the book roll itself must still fit.
        let bare_ft = need_at(1.0);
        if !waived.contains(&Gate::OriginRunway) {
            return Err((
                Gate::OriginRunway,
                format!(
                    "Negative. {}'s longest runway is {}, and the takeoff needs {}. Not flying that.",
                    fp.origin,
                    spoken_feet(f.origin_longest_ft),
                    spoken_feet(need_ft)
                ),
            ));
        }
        if f.origin_longest_ft < bare_ft {
            return Err((
                Gate::BookRoll,
                format!(
                    "Negative. Even with no margin we need {}, and {}'s longest runway is {}. Not flying that.",
                    spoken_feet(bare_ft),
                    fp.origin,
                    spoken_feet(f.origin_longest_ft)
                ),
            ));
        }
        waived_lines.push(format!(
            "No runway margin, your call: {} of runway, {} needed without it.",
            spoken_feet(f.origin_longest_ft),
            spoken_feet(bare_ft)
        ));
    }
    // Weight: the plan's max takeoff weight, else the profile's max gross.
    // A waived plan limit falls back to the profile's, which always holds.
    let plan_tow_kg = fp.weights.as_ref().map(|w| w.max_tow_kg).filter(|m| *m > 0.0);
    let gross_kg = f.max_gross_lb.map(|lb| lb / LB_PER_KG);
    let plan_waived = waived.contains(&Gate::PlanWeight);
    let max_tow_kg = if plan_waived {
        gross_kg
    } else {
        plan_tow_kg.or(gross_kg)
    };
    if let (true, Some(total), Some(plan)) = (plan_waived, f.total_kg, plan_tow_kg) {
        if total > plan * 1.005 {
            waived_lines.push(format!(
                "Over the plan's weight limit, your call: {} against {}.",
                spoken_weight(total),
                spoken_weight(plan)
            ));
        }
    }
    if let (Some(total), Some(max)) = (f.total_kg, max_tow_kg) {
        if total > max * 1.005 {
            // SimBrief's max_tow is the OPERATIONAL limit, the least of the
            // structural MTOW, the landing weight plus the trip's burn and the
            // zero-fuel weight plus fuel: on a short leg it sits far under the
            // placard (LFLL-EGLL, 2026-09-17: 153,280 lb against a 174,200 lb
            // airframe, and the pilot asked whether that was right). Name it
            // as the plan's, and when the load sheet says why, say that too:
            // the excess there was 9.4 t of payload over the OFP's, not fuel.
            let from_plan = !plan_waived && plan_tow_kg.is_some();
            let mut why = format!(
                "Negative. We're at {}, over the {} of {}.",
                spoken_weight(total),
                if from_plan {
                    "plan's takeoff limit"
                } else {
                    "max takeoff weight"
                },
                spoken_weight(max)
            );
            if let (Some(w), Some(p)) = (fp.weights.as_ref(), f.payload_kg) {
                if w.payload_kg > 0.0
                    && p - w.payload_kg > PAYLOAD_MATCH_KG.max(w.payload_kg * PAYLOAD_MATCH_FRAC)
                {
                    why.push_str(&format!(
                        " The aircraft is loaded with {} of payload against the plan's {}.",
                        w.spoken(p),
                        w.spoken(w.payload_kg)
                    ));
                }
            }
            why.push_str(" Not flying that.");
            let gate = if from_plan && gross_kg.is_none_or(|g| total <= g * 1.005) {
                Gate::PlanWeight
            } else {
                Gate::MaxWeight
            };
            return Err((gate, why));
        }
    }
    let mut lines = Vec::new();
    // Fuel: below the minimum takeoff fuel is a refusal; under the block but
    // above the minimum is said with both numbers; at the block is a check.
    // The unit is said ONCE: "12,880 pounds for the plan's 12,990" (the ack
    // was a paragraph, EGSS-LFPG 2026-09-22).
    if let Some(fuel) = f.fuel_kg.filter(|_| fp.block_fuel_kg > 0.0) {
        let min = fp.min_takeoff_fuel_kg;
        if min > 0.0 && fuel < min * (1.0 - FUEL_MATCH_FRAC) {
            return Err((
                Gate::Fuel,
                format!(
                    "Negative. {} of fuel aboard, and the plan needs at least {} to go. Say load the aircraft and I'll fuel it from the plan, then say it again.",
                    spoken_fuel(fuel),
                    spoken_fuel(min)
                ),
            ));
        }
        let bare = |s: String| {
            s.split_once(' ')
                .map_or_else(|| s.clone(), |(n, _)| n.to_string())
        };
        if fuel < fp.block_fuel_kg * (1.0 - FUEL_MATCH_FRAC) {
            lines.push(format!(
                "Fuel {} against a {} block, above the minimum.",
                spoken_fuel(fuel),
                bare(spoken_fuel(fp.block_fuel_kg))
            ));
        } else {
            lines.push(format!(
                "Fuel {} for the plan's {}.",
                spoken_fuel(fuel),
                bare(spoken_fuel(fp.block_fuel_kg))
            ));
        }
    }
    // The load sheet: who and what is aboard, and whether the sim agrees.
    if let Some(w) = &fp.weights {
        let mut load = match (w.pax, w.cargo_kg > 0.0) {
            (0, false) => "No passengers or cargo".to_string(),
            (0, true) => format!("{} of cargo", w.spoken(w.cargo_kg)),
            (n, false) => format!("{n} passenger{}", if n == 1 { "" } else { "s" }),
            (n, true) => format!(
                "{n} passenger{} and {} of cargo",
                if n == 1 { "" } else { "s" },
                w.spoken(w.cargo_kg)
            ),
        };
        match f.payload_kg {
            Some(p) if w.payload_kg > 0.0 => {
                let off = (p - w.payload_kg).abs();
                if off > PAYLOAD_MATCH_KG && off > w.payload_kg * PAYLOAD_MATCH_FRAC {
                    load.push_str(&format!(
                        " on the plan, but the aircraft is loaded with {} against the plan's {}",
                        w.spoken(p),
                        w.spoken(w.payload_kg)
                    ));
                } else {
                    load.push_str(" as planned");
                }
            }
            _ => {}
        }
        load.push('.');
        lines.push(load);
    }
    waived_lines.extend(lines);
    Ok(waived_lines.join(" "))
}

/// Where the arm finds the aircraft before any taxi (issue #128).
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum RunwayStart {
    /// Not on a runway it can depart from: the taxied departure.
    None,
    /// Lined up on `runway` with the roll ahead of it: call ready.
    Lined { runway: String },
    /// Stopped on `runway`, aligned, with too little of it ahead: backtrack
    /// to the threshold. A taxi request from here was answered with a route
    /// OFF the runway to its far holding point (EGKK 26L, sortie
    /// 1790527340: 1,340 m ahead, 1,897 needed, "taxi to holding point
    /// Mike").
    Short { runway: String },
}

/// Stopped means stopped: a runway start is where the sim put the aircraft,
/// never a roll in progress (the tower's own latch uses 1 kt,
/// `RUNWAY_START_MAX_KTS`; the crew allows a creep).
pub(super) const RUNWAY_START_BACKTRACK_MAX_KTS: f64 = 3.0;

/// Classify the arm's position from the takeoff fit. Pure.
pub(crate) fn classify_runway_start(
    fit: Result<String, Option<crate::runway::NoFit>>,
    gs_kts: f64,
) -> RunwayStart {
    match fit {
        Ok(runway) => RunwayStart::Lined { runway },
        Err(Some(crate::runway::NoFit::TooLittleLeft { id, .. }))
            if gs_kts < RUNWAY_START_BACKTRACK_MAX_KTS =>
        {
            RunwayStart::Short { runway: id }
        }
        _ => RunwayStart::None,
    }
}

impl RunwayStart {
    /// The strip we stand on, when we stand on one.
    pub(crate) fn runway(&self) -> Option<&str> {
        match self {
            RunwayStart::None => None,
            RunwayStart::Lined { runway } | RunwayStart::Short { runway } => Some(runway),
        }
    }

    /// A clearance for ANOTHER runway (the IFR clearance's SID was chosen
    /// with the tower's runway in use, which the station freezes for it) is
    /// flown from that runway: the taxied departure goes there. Asking for
    /// the strip we sit on would unpick a clearance already read back, and
    /// the tower treats only the strip in use as a lined-up aircraft
    /// (`Ctx::own_on_runway`).
    ///
    /// ...but the RECIPROCAL of our own strip is not another runway: it is
    /// this pavement the other way, reached by a backtrack down it (turn at
    /// its far end, line up), never a taxi off it and back (#178). Pure.
    pub(crate) fn under_clearance(self, cleared: Option<&str>) -> RunwayStart {
        match (self.runway(), cleared) {
            (Some(on), Some(c)) if !crate::autotakeoff::runway_matches(on, c) => {
                if is_reciprocal(on, c) {
                    RunwayStart::Short {
                        runway: c.to_string(),
                    }
                } else {
                    RunwayStart::None
                }
            }
            _ => self,
        }
    }
}

/// The takeoff fit at the arm, as a [`RunwayStart`].
pub(super) async fn runway_start(xp: &XPlaneClient, tools: &SubTools) -> RunwayStart {
    let fit = runway_check_id(xp, tools).await;
    let gs_kts = xp
        .get_float(crate::autotaxi::GS_MS)
        .await
        .map(|v| v * MS_TO_KTS)
        .unwrap_or(f64::INFINITY);
    let start = classify_runway_start(fit.clone(), gs_kts);
    if start != RunwayStart::None {
        event(
            "runway_start",
            json!({ "start": format!("{start:?}"), "gs_kts": gs_kts,
                    "fit": format!("{fit:?}") }),
        );
    }
    start
}

/// The takeoff loop's own fit, asked with the takeoff's own pavement rule
/// (`aircraft::takeoff_needed_now_m`), and on a miss the gate that failed.
/// `Err(None)` = airborne, or no position to fit.
pub(super) async fn runway_check(
    xp: &XPlaneClient,
    tools: &SubTools,
) -> Result<(), Option<crate::runway::NoFit>> {
    runway_check_id(xp, tools).await.map(|_| ())
}

/// [`runway_check`] naming the runway end that fits.
pub(super) async fn runway_check_id(
    xp: &XPlaneClient,
    tools: &SubTools,
) -> Result<String, Option<crate::runway::NoFit>> {
    // Airborne is not "lined up"; and a fit needs a valid position.
    if xp.get_float(crate::datarefs::ON_GROUND_ANY).await.unwrap_or(1.0) < 0.5 {
        return Err(None);
    }
    let (Ok(lat), Ok(lon), Ok(psi)) = (
        xp.get_float("sim/flightmodel/position/latitude").await,
        xp.get_float("sim/flightmodel/position/longitude").await,
        xp.get_float("sim/flightmodel/position/psi").await,
    ) else {
        return Err(None);
    };
    let needed_m = crate::aircraft::takeoff_needed_now_m(
        xp,
        tools.line_up.takeoff_roll_m,
        tools.line_up.mtow_kg,
        500.0,
    )
    .await;
    let apt = crate::config::apt_dat_path();
    let cands = crate::runway::nearby_runways(&apt, lat, lon, 3.0).unwrap_or_default();
    match crate::runway::runway_fit(&cands, lat, lon, psi, needed_m) {
        Some((r, _)) => Ok(r.id),
        None => Err(Some(crate::runway::why_no_fit_cleared(
            &cands,
            lat,
            lon,
            psi,
            needed_m,
            crate::party_line::clearance_runway().as_deref(),
        ))),
    }
}

/// Why a line-up that the fit accepts is still not ready to roll
/// ([`judge_lined_up`]).
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum LineUpMiss {
    /// The drive we asked for did not arrive (`stopped_off_line`, off the
    /// route, stagnant, a stand-down): it already said so and let go.
    NotArrived { lateral_m: f64, err_deg: f64 },
    /// Arrived, but too far off the centreline to roll from.
    OffCentreline { lateral_m: f64, max_m: f64 },
    /// Arrived, but pointing too far off the runway.
    Crooked { err_deg: f64 },
}

/// A line-up is ready to roll within this of the centreline: the aircraft's
/// centre this far inside the pavement's edge (the outer wheel half the
/// track out)...
pub(super) const LINED_UP_EDGE_M: f64 = 4.0;
/// ...never asked tighter than this, nor looser than the drive's own
/// arrival ([`crate::autotaxi::ARRIVED_ON_LINE_M`]).
pub(super) const LINED_UP_MIN_M: f64 = 2.0;
/// ...and pointing within this of the runway: what the roll's own hold
/// straightens ([`crate::autotakeoff::RollAlign`]) on the brakes.
pub(super) const LINED_UP_MAX_ERR_DEG: f64 = 10.0;

/// Whether a line-up the runway fit accepts may go on to the takeoff. The
/// fit takes 40 m off the centreline and 20 degrees, and a drive's failure
/// was never asked: LILB 03 Evolution (1790732094, 2026-09-30) the
/// backtrack's turn-round ended `stopped_off_line` 5.6 m off and 12.4
/// degrees crooked on a 15 m strip, said "Holding here. Your steering.",
/// and 2 s later full auto logged `lined_up`, said "Lined up. Flying the
/// circuit." and rolled; it left the centreline and rejected at 68 kt,
/// 9.4 m off. `drove`: a drive was started by the line-up we asked for, so
/// `arrived` is about it. Pure.
pub(crate) fn judge_lined_up(
    drove: bool,
    arrived: bool,
    lateral_m: f64,
    err_deg: f64,
    width_m: Option<f64>,
) -> Result<(), LineUpMiss> {
    let lateral_m = lateral_m.abs();
    let err_deg = err_deg.abs();
    if drove && !arrived {
        return Err(LineUpMiss::NotArrived { lateral_m, err_deg });
    }
    let max_m = width_m.map_or(crate::autotaxi::ARRIVED_ON_LINE_M, |w| {
        (w / 2.0 - LINED_UP_EDGE_M).clamp(LINED_UP_MIN_M, crate::autotaxi::ARRIVED_ON_LINE_M)
    });
    if lateral_m > max_m {
        return Err(LineUpMiss::OffCentreline { lateral_m, max_m });
    }
    if err_deg > LINED_UP_MAX_ERR_DEG {
        return Err(LineUpMiss::Crooked { err_deg });
    }
    Ok(())
}

/// What is said of a [`LineUpMiss`], before the retry or the give-up.
pub(crate) fn line_up_miss_words(miss: &LineUpMiss) -> String {
    match miss {
        LineUpMiss::NotArrived { lateral_m, err_deg } => format!(
            "Not lined up: {:.0} metres off the centreline and {:.0} degrees off the runway.",
            lateral_m, err_deg
        ),
        LineUpMiss::OffCentreline { lateral_m, .. } => {
            format!("Not lined up: {:.0} metres off the centreline.", lateral_m)
        }
        LineUpMiss::Crooked { err_deg } => {
            format!("Not lined up: {:.0} degrees off the runway.", err_deg)
        }
    }
}

/// [`judge_lined_up`] where the aircraft stands, on the runway the fit
/// found: `Ok(Ok(()))` ready, `Ok(Err(miss))` on the strip but not ready,
/// `Err(why)` the fit's own refusal ([`runway_check`]).
pub(super) async fn lined_up_check(
    xp: &XPlaneClient,
    tools: &SubTools,
    drove: bool,
) -> Result<Result<(), LineUpMiss>, Option<crate::runway::NoFit>> {
    let id = runway_check_id(xp, tools).await?;
    let (Ok(lat), Ok(lon), Ok(psi)) = (
        xp.get_float("sim/flightmodel/position/latitude").await,
        xp.get_float("sim/flightmodel/position/longitude").await,
        xp.get_float("sim/flightmodel/position/psi").await,
    ) else {
        return Err(None);
    };
    let apt = crate::config::apt_dat_path();
    let cands = crate::runway::nearby_runways(&apt, lat, lon, 3.0).unwrap_or_default();
    let Some(rwy) = cands.iter().find(|r| r.id == id) else {
        return Ok(Ok(()));
    };
    let lateral_m = crate::runway::cross_track_nm(lat, lon, rwy) * 1852.0;
    let err_deg = crate::angles::abs_diff_deg(psi, rwy.course_deg);
    let arrived = crate::autotaxi::LAST_ARRIVED.load(SeqCst);
    let verdict = judge_lined_up(
        drove,
        arrived,
        lateral_m,
        err_deg,
        crate::runway::runway_width_m(rwy),
    );
    event(
        "line_up_check",
        json!({ "runway": id, "drove": drove, "arrived": arrived, "lateral_m": lateral_m,
                "err_deg": err_deg, "width_m": crate::runway::runway_width_m(rwy),
                "verdict": format!("{verdict:?}") }),
    );
    Ok(verdict)
}
