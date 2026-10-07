//! The whole flight, checked before it is flown.
//!
//! Full auto used to find a plan's problems where they were: the SID's
//! minimum at the hold (LOWI), the ground under a SimBrief altitude at 220 ft
//! AGL (the Sierra de Grazalema), a STAR's FL070 at 278 kt over the fix. This
//! is a DRY RUN of the follower over the plan, with the follower's own rules
//! (the plan's altitudes clamped into the STAR, the arrival ceiling, the
//! airframe's climb and descent rates), judged against:
//!
//! - the ground near the departure, PROBED from the sim's own scenery (the
//!   one place the scenery is loaded before takeoff), at the best climb the
//!   airframe has: a climb that cannot clear it is a refusal;
//! - the grid MORA (`earth_mora.dat`) everywhere else: one-degree cells, so a
//!   leg under its cell's minimum is a caution naming the ground it may meet,
//!   never a refusal (valleys are legal, and the follower's terrain floor
//!   still watches 15 nm ahead in flight);
//! - every STAR restriction: a crossing predicted high or low, a speed limit
//!   under the airframe's clean floor, a minimum above the ceiling;
//! - the cruise: never reached on a short leg, or unpressurised above 12,500.
//!
//! The fuel, weight and runway checks stay in `full_auto::preflight` and the
//! SID gradients in `full_auto::departure_gradients`; the arm runs all three.
//! Pure below [`check`], so every rule is a plain `cargo test`.

use crate::nav::{MoraGrid, ProcLeg, StarConstraint};

/// How bad a finding is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    /// The plan is not flown.
    Stop,
    /// Said at the arm, flown anyway.
    Caution,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Finding {
    pub severity: Severity,
    pub kind: &'static str,
    pub line: String,
}

/// The airframe as the dry run flies it.
#[derive(Debug, Clone)]
pub struct Airframe {
    pub ceiling_ft: Option<f64>,
    pub pressurised: Option<bool>,
    pub climb_fpm: f64,
    pub best_climb_fpm: f64,
    pub descent_fpm: f64,
    /// The climb's indicated speed (Vy).
    pub climb_ias_kts: f64,
    pub cruise_ias_kts: f64,
    /// The clean floor (`aircraft::speed_floor_kts`).
    pub speed_floor_kts: Option<f64>,
}

pub struct Inputs<'a> {
    pub origin: (f64, f64),
    pub origin_elev_ft: f64,
    pub dest: (f64, f64),
    /// The enroute legs as the follower will fly them (STAR already clamped).
    pub legs: &'a [ProcLeg],
    pub cruise_alt_ft: f64,
    pub stars: &'a [StarConstraint],
    pub mora: Option<&'a MoraGrid>,
    /// Ground probed along the departure: (nm along the route, feet MSL).
    pub departure_ground: Option<&'a [(f64, f64)]>,
    pub airframe: Airframe,
    /// Whether THIS airframe is fit for THIS plan (`None`: not judged).
    pub suitability: Option<&'a Suitability>,
}

/// A predicted fix crossing.
#[derive(Debug, Clone, PartialEq)]
pub struct Crossing {
    pub fix: String,
    pub along_nm: f64,
    pub alt_ft: f64,
}

#[derive(Debug, Clone, Default)]
pub struct Audit {
    pub findings: Vec<Finding>,
    pub profile: Vec<Crossing>,
    /// What was actually checked, for the all-clear line.
    pub checked: Vec<&'static str>,
    /// What could not be, said so the all-clear is never bigger than it is.
    pub unchecked: Vec<&'static str>,
}

impl Audit {
    pub fn stop(&self) -> Option<&Finding> {
        self.findings.iter().find(|f| f.severity == Severity::Stop)
    }
    pub fn cautions(&self) -> impl Iterator<Item = &Finding> {
        self.findings.iter().filter(|f| f.severity == Severity::Caution)
    }
}

/// Within this of either field the published procedures and the sector
/// altitude own the terrain, not a one-degree grid.
const TERMINAL_NM: f64 = 25.0;
/// Grid sampling step along a leg, nm.
const GRID_STEP_NM: f64 = 2.0;
/// Clearance the best-rate climb must keep over PROBED ground, ft...
const CLIMB_STOP_FT: f64 = 300.0;
/// ...and the normal climb, below which it is said.
const CLIMB_TIGHT_FT: f64 = 1_000.0;
/// A STAR crossing this far off its restriction is said.
const STAR_SLACK_FT: f64 = 250.0;
/// Oxygen territory for an unpressurised cabin.
const UNPRESSURISED_MAX_FT: f64 = 12_500.0;

/// Indicated to true airspeed, the 2 % a thousand feet rule.
pub fn tas_kts(ias_kts: f64, alt_ft: f64) -> f64 {
    ias_kts * (1.0 + 0.02 * alt_ft.max(0.0) / 1_000.0)
}

/// Feet a nm at `fpm` and this true airspeed (no wind: SimBrief's winds are
/// averages, and a headwind only steepens a climb).
fn ft_per_nm(fpm: f64, tas: f64) -> f64 {
    fpm / (tas.max(60.0) / 60.0)
}

/// The route as points: origin, each fix, destination; (ident, lat, lon).
fn points(i: &Inputs) -> Vec<(String, f64, f64)> {
    let mut p = vec![("origin".to_string(), i.origin.0, i.origin.1)];
    for l in i.legs {
        if let Some(f) = &l.fix {
            p.push((l.fix_ident.clone(), f.lat, f.lon));
        }
    }
    p.push(("destination".to_string(), i.dest.0, i.dest.1));
    p
}

/// The altitude the follower will be at `frac` of the way along a leg from
/// `a` to target `t` over `seg_nm`: a climb goes at once at `climb` ft/nm;
/// a descent waits for its top of descent and arrives at `t` over the fix,
/// or, with more to lose than the leg allows, starts at once and arrives
/// high.
fn along_leg(a: f64, t: f64, seg_nm: f64, frac: f64, climb: f64, descent: f64) -> f64 {
    if t >= a {
        (a + climb * frac * seg_nm).min(t)
    } else {
        let late = (t + descent * (1.0 - frac) * seg_nm).min(a);
        let now = a - descent * frac * seg_nm;
        late.max(now).clamp(t, a)
    }
}

/// One pass of the follower's vertical over the route with the given climb
/// rate and speed: for each leg, (start altitude, target, leg length, climb
/// ft/nm, descent ft/nm, along at start). The target is the leg's own
/// constraint under the arrival ceiling, as the loop computes it.
fn fly_profile(i: &Inputs, climb_fpm: f64, climb_ias: f64) -> Vec<(f64, f64, f64, f64, f64, f64)> {
    let af = &i.airframe;
    let mut out = Vec::new();
    let mut a = i.origin_elev_ft;
    let mut along = 0.0;
    let mut from = i.origin;
    for (idx, leg) in i.legs.iter().enumerate() {
        let Some(f) = &leg.fix else { continue };
        let seg = crate::nav::haversine_nm(from.0, from.1, f.lat, f.lon);
        let desc = ft_per_nm(af.descent_fpm, tas_kts(af.cruise_ias_kts, a));
        let mut t = leg.alt1_ft.unwrap_or(i.cruise_alt_ft);
        if let Some(c) = crate::procedure::arrival_ceiling_ft(
            i.legs,
            i.stars,
            idx,
            crate::procedure::arrival_ft_per_nm(af.descent_fpm, tas_kts(af.cruise_ias_kts, a)),
        ) {
            if c < t - 50.0 {
                t = (c / 100.0).floor() * 100.0;
            }
        }
        let climb = ft_per_nm(climb_fpm, tas_kts(climb_ias, a));
        out.push((a, t, seg, climb, desc, along));
        a = along_leg(a, t, seg, 1.0, climb, desc);
        along += seg;
        from = (f.lat, f.lon);
    }
    out
}

/// The predicted altitude `d` nm along the route, from a flown profile.
fn alt_at(profile: &[(f64, f64, f64, f64, f64, f64)], d: f64) -> Option<f64> {
    profile.iter().find_map(|&(a, t, seg, climb, desc, along)| {
        (d >= along && d <= along + seg).then(|| {
            along_leg(a, t, seg, (d - along) / seg.max(1e-6), climb, desc)
        })
    })
}

fn feet(ft: f64) -> String {
    crate::flightplan::spoken_feet(ft)
}

/// Round a shortfall for speech, to 50 ft.
fn by(ft: f64) -> String {
    crate::auto_status::commas(((ft / 50.0).round() * 50.0) as i64)
}

/// Run every check. Pure.
pub fn audit(i: &Inputs) -> Audit {
    let mut out = Audit::default();
    let af = &i.airframe;
    let pts = points(i);

    // --- Every fix must be a place.
    if let Some(l) = i
        .legs
        .iter()
        .find(|l| l.fix.as_ref().is_none_or(|f| f.lat == 0.0 && f.lon == 0.0))
    {
        out.findings.push(Finding {
            severity: Severity::Stop,
            kind: "fix_unresolved",
            line: format!("The plan's fix {} has no position. Regenerate the plan.", l.fix_ident),
        });
        return out;
    }

    if let Some(su) = i.suitability {
        out.checked.push("that this aircraft suits the plan");
        out.findings.extend(suitability(su, i.cruise_alt_ft, af));
        if out.stop().is_some() {
            return out;
        }
    }

    let normal = fly_profile(i, af.climb_fpm, af.climb_ias_kts);
    let best = fly_profile(i, af.best_climb_fpm.max(af.climb_fpm), af.climb_ias_kts);
    let total_nm = normal.last().map_or(0.0, |p| p.5 + p.2);
    // The crossings, for the STAR checks and the report.
    let mut along = 0.0;
    for (p, leg) in normal.iter().zip(i.legs.iter().filter(|l| l.fix.is_some())) {
        along = p.5 + p.2;
        out.profile.push(Crossing {
            fix: leg.fix_ident.clone(),
            along_nm: along,
            alt_ft: along_leg(p.0, p.1, p.2, 1.0, p.3, p.4),
        });
    }
    let _ = along;

    // --- The cruise.
    let peak = out.profile.iter().map(|c| c.alt_ft).fold(i.origin_elev_ft, f64::max);
    out.checked.push("the climb to cruise");
    if peak < i.cruise_alt_ft - 500.0 {
        out.findings.push(Finding {
            severity: Severity::Caution,
            kind: "cruise_not_reached",
            line: format!(
                "We won't reach the planned {} on this route; we top out around {}.",
                feet(i.cruise_alt_ft),
                feet((peak / 100.0).floor() * 100.0)
            ),
        });
    }
    if af.pressurised == Some(false) && i.cruise_alt_ft > UNPRESSURISED_MAX_FT {
        out.findings.push(Finding {
            severity: Severity::Caution,
            kind: "unpressurised_cruise",
            line: format!(
                "The plan cruises at {} in an unpressurised cabin. Oxygen for the whole cruise.",
                feet(i.cruise_alt_ft)
            ),
        });
    }

    // --- The ground near the departure, probed: the only terrain we can
    // measure before takeoff, and the one a climb can fail against.
    match i.departure_ground {
        Some(ground) if !ground.is_empty() => {
            out.checked.push("the ground along the climb out");
            let mut worst_best: Option<(f64, f64, f64)> = None; // (margin, d, ground)
            let mut worst_norm: Option<(f64, f64, f64)> = None;
            for &(d, g) in ground.iter().filter(|(d, _)| *d > 1.0) {
                if let Some(a) = alt_at(&best, d) {
                    let m = a - g;
                    if worst_best.is_none_or(|w| m < w.0) {
                        worst_best = Some((m, d, g));
                    }
                }
                if let Some(a) = alt_at(&normal, d) {
                    let m = a - g;
                    if worst_norm.is_none_or(|w| m < w.0) {
                        worst_norm = Some((m, d, g));
                    }
                }
            }
            let near = |d: f64| nearest_fix(&out.profile, d);
            if let Some((m, d, g)) = worst_best.filter(|w| w.0 < CLIMB_STOP_FT) {
                out.findings.push(Finding {
                    severity: Severity::Stop,
                    kind: "climb_terrain",
                    line: format!(
                        "Negative. Even at our best climb we'd be {} feet {} the ground {:.0} miles out, near {}, where it rises to {}. Not flying that route.",
                        by(m.abs()),
                        if m < 0.0 { "into" } else { "over" },
                        d,
                        near(d),
                        feet(g)
                    ),
                });
            } else if let Some((m, d, g)) = worst_norm.filter(|w| w.0 < CLIMB_TIGHT_FT) {
                out.findings.push(Finding {
                    severity: Severity::Caution,
                    kind: "climb_terrain_tight",
                    line: format!(
                        "The climb out is tight: {} feet over the ground {:.0} miles out, near {}, where it rises to {}.",
                        by(m.max(0.0)),
                        d,
                        near(d),
                        feet(g)
                    ),
                });
            }
        }
        _ => out.unchecked.push("the ground near the departure"),
    }

    // --- The grid, enroute: outside both terminal areas.
    if let Some(grid) = i.mora {
        out.checked.push("the grid safe altitudes enroute");
        let dest_along = total_nm;
        // The worst stretch under its cell's minimum: (short of MORA, d, mora).
        let mut worst: Option<(f64, f64, f64, f64)> = None;
        let mut d = 0.0;
        while d <= total_nm {
            let terminal = d < TERMINAL_NM || dest_along - d < TERMINAL_NM;
            if !terminal {
                if let (Some(a), Some((lat, lon))) = (alt_at(&normal, d), position_at(&pts, d)) {
                    if let Some(m) = grid.at(lat, lon) {
                        let short = m - a;
                        if short > 0.0 && worst.is_none_or(|w| short > w.0) {
                            worst = Some((short, d, m, a));
                        }
                    }
                }
            }
            d += GRID_STEP_NM;
        }
        if let Some((_, d, m, a)) = worst {
            let ground = crate::nav::mora_ground_ft(m);
            let beyond = af.ceiling_ft.is_some_and(|c| ground + 1_000.0 > c);
            out.findings.push(Finding {
                severity: Severity::Caution,
                kind: "under_grid_mora",
                line: format!(
                    "Near {} the plan is at {}, under the grid safe altitude of {}; the ground in that area reaches about {}.{}",
                    nearest_fix(&out.profile, d),
                    feet((a / 100.0).round() * 100.0),
                    feet(m),
                    feet(ground),
                    if beyond {
                        " That's more than we can climb above. I'll hold what clearance I can see ahead."
                    } else {
                        " I'll climb for it if the ground ahead needs it."
                    }
                ),
            });
        }
    } else {
        out.unchecked.push("the grid safe altitudes");
    }

    // --- The arrival's restrictions.
    if !i.stars.is_empty() {
        out.checked.push("the arrival's restrictions");
    }
    for c in i.stars {
        let Some(x) = out.profile.iter().find(|x| x.fix.eq_ignore_ascii_case(&c.fix)) else {
            continue;
        };
        if let (Some(min), Some(ceiling)) = (c.min_ft, af.ceiling_ft) {
            if min > ceiling {
                out.findings.push(Finding {
                    severity: Severity::Stop,
                    kind: "star_above_ceiling",
                    line: format!(
                        "Negative. {} must be crossed at {} or above, over our ceiling of {}.",
                        c.fix,
                        feet(min),
                        feet(ceiling)
                    ),
                });
                continue;
            }
        }
        // A window over the plan's cruise is flown at the cruise
        // (`procedure::apply_arrival_constraints`), and said as that, not as
        // a low crossing (HEMEL's FL220 in an FL180 plan, EGKK-EGBB
        // 2026-09-28, #156).
        if let Some(min) = c.min_ft.filter(|m| i.cruise_alt_ft > 0.0 && *m > i.cruise_alt_ft + STAR_SLACK_FT) {
            out.findings.push(Finding {
                severity: Severity::Caution,
                kind: "star_above_cruise",
                line: crate::procedure::above_cruise_line(&c.fix, min, i.cruise_alt_ft),
            });
            continue;
        }
        if let Some(max) = c.max_ft.filter(|m| x.alt_ft > m + STAR_SLACK_FT) {
            out.findings.push(Finding {
                severity: Severity::Caution,
                kind: "star_high",
                line: format!(
                    "We'll be about {} feet high at {} for its {}; the descent can't get there sooner.",
                    by(x.alt_ft - max),
                    c.fix,
                    feet(max)
                ),
            });
        }
        if let Some(min) = c.min_ft.filter(|m| x.alt_ft < m - STAR_SLACK_FT) {
            out.findings.push(Finding {
                severity: Severity::Caution,
                kind: "star_low",
                line: format!(
                    "We'll be about {} feet under {}'s {} minimum.",
                    by(min - x.alt_ft),
                    c.fix,
                    feet(min)
                ),
            });
        }
        if let (Some(kts), Some(floor)) = (c.max_kts, af.speed_floor_kts) {
            if kts < floor {
                out.findings.push(Finding {
                    severity: Severity::Caution,
                    kind: "star_speed_below_floor",
                    line: format!(
                        "{}'s {kts:.0} knot limit is under our clean minimum; I'll fly {floor:.0}.",
                        c.fix
                    ),
                });
            }
        }
    }
    out
}


/// What says whether THIS airframe can fly THIS plan. `None` / empty is
/// unknown, and an unknown is never a finding.
#[derive(Debug, Clone, Default)]
pub struct Suitability {
    /// The plan's aircraft type (SimBrief `aircraft/icaocode`).
    pub plan_type: String,
    /// The loaded aircraft's `acf_ICAO`.
    pub acf_type: Option<String>,
    /// Other types a plan for this airframe may be filed as (the profile's
    /// `simbrief_types`).
    pub plan_types: Vec<String>,
    pub distance_nm: f64,
    pub block_fuel_kg: f64,
    pub trip_fuel_kg: f64,
    /// `acf_m_fuel_tot`.
    pub tank_capacity_kg: Option<f64>,
    /// The profile's cruise burn and true cruise speed.
    pub burn_kg_hr: Option<f64>,
    pub cruise_tas_kts: Option<f64>,
    pub est_landing_kg: Option<f64>,
    pub max_landing_kg: Option<f64>,
    /// Pavement: each end's longest runway and what this airframe needs.
    pub origin_longest_ft: Option<f64>,
    pub dest_longest_ft: Option<f64>,
    pub takeoff_needed_ft: Option<f64>,
    pub landing_needed_ft: Option<f64>,
    /// The destination's surface wind NOW (true degrees, knots) and its
    /// runways' true headings, against the demonstrated crosswind.
    pub dest_wind: Option<(f64, f64)>,
    pub dest_runway_hdgs: Vec<(String, f64)>,
    pub max_crosswind_kts: Option<f64>,
    pub dest_icao: String,
}

/// A plan's fuel for the trip under this share of the airframe's own
/// estimate is a plan made for something else.
const TRIP_FUEL_LOW_SHARE: f64 = 0.7;
/// The reserve the range check adds to the airframe's trip: 45 minutes.
const RANGE_RESERVE_HR: f64 = 0.75;

/// The crosswind on the best of these runways: (knots, runway).
pub fn best_crosswind(wind: (f64, f64), runways: &[(String, f64)]) -> Option<(f64, String)> {
    runways
        .iter()
        .map(|(id, hdg)| ((wind.0 - hdg).to_radians().sin().abs() * wind.1, id.clone()))
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))
}

/// The aircraft against the plan. Pure.
pub fn suitability(s: &Suitability, cruise_alt_ft: f64, af: &Airframe) -> Vec<Finding> {
    let fuel = crate::aircraft::say_fuel_kg;
    let stop = |kind, line: String| Finding { severity: Severity::Stop, kind, line };
    let caution = |kind, line: String| Finding { severity: Severity::Caution, kind, line };
    let mut out = Vec::new();
    // The type: every number in a plan for another aircraft is its numbers.
    // A plan names a TYPE, never a model, so this stays on the ICAO: a Zibo
    // flying a plan filed as B738 is the plan's type (detect::resolve is the
    // model question, and it is not asked here).
    if let Some(acf) = s.acf_type.as_deref().map(str::trim).filter(|a| !a.is_empty()) {
        let plan = s.plan_type.trim();
        let ours = plan.eq_ignore_ascii_case(acf)
            || s.plan_types.iter().any(|t| t.trim().eq_ignore_ascii_case(plan));
        if !plan.is_empty() && !ours {
            out.push(stop(
                "type_mismatch",
                format!(
                    "Negative. The plan is for a {}, and this is a {acf}. Its speeds, fuel and weights aren't ours. Regenerate it for this aircraft.",
                    s.plan_type.trim()
                ),
            ));
            return out;
        }
    }
    if let Some(c) = af.ceiling_ft.filter(|c| cruise_alt_ft > *c) {
        out.push(stop(
            "above_ceiling",
            format!("Negative. The plan cruises at {}, above our ceiling of {}.", feet(cruise_alt_ft), feet(c)),
        ));
    }
    // Range: the tanks against the plan, and against this airframe's own
    // burn over the distance (a SimBrief profile for another variant plans
    // the fuel of another variant).
    if let Some(cap) = s.tank_capacity_kg.filter(|c| *c > 0.0) {
        if s.block_fuel_kg > cap * 1.02 {
            out.push(stop(
                "block_over_capacity",
                format!("Negative. The plan needs {} of fuel and our tanks hold {}.", fuel(s.block_fuel_kg), fuel(cap)),
            ));
        }
        if let (Some(burn), Some(tas)) = (s.burn_kg_hr, s.cruise_tas_kts.filter(|t| *t > 0.0)) {
            let own_trip = s.distance_nm / tas * burn;
            let need = own_trip + RANGE_RESERVE_HR * burn;
            if s.distance_nm > 0.0 && need > cap * 1.02 {
                out.push(stop(
                    "out_of_range",
                    format!(
                        "Negative. {:.0} miles is beyond us: about {} for the trip and reserve at our burn, and the tanks hold {}.",
                        s.distance_nm,
                        fuel(need),
                        fuel(cap)
                    ),
                ));
            } else if s.trip_fuel_kg > 0.0 && s.trip_fuel_kg < own_trip * TRIP_FUEL_LOW_SHARE {
                out.push(caution(
                    "trip_fuel_low",
                    format!(
                        "The plan burns {} for the trip; at our {} an hour I make it nearer {}. Check the SimBrief aircraft.",
                        fuel(s.trip_fuel_kg),
                        fuel(burn),
                        fuel(own_trip)
                    ),
                ));
            }
        }
    }
    // Pavement at both ends.
    if let (Some(have), Some(need)) = (s.dest_longest_ft, s.landing_needed_ft) {
        if have > 0.0 && have < need {
            out.push(stop(
                "dest_runway_short",
                format!("Negative. {}'s longest runway is {}, and we need {} to land.", s.dest_icao, feet(have), feet(need)),
            ));
        }
    }
    if let (Some(have), Some(need)) = (s.origin_longest_ft, s.takeoff_needed_ft) {
        if have > 0.0 && have < need {
            out.push(stop(
                "origin_runway_short",
                format!("Negative. The longest runway here is {}, and the takeoff needs {}.", feet(have), feet(need)),
            ));
        }
    }
    if let (Some(est), Some(max)) = (s.est_landing_kg, s.max_landing_kg) {
        if max > 0.0 && est > max * 1.005 {
            out.push(caution(
                "overweight_landing",
                format!("We'd land at {}, over the {} landing limit.", fuel(est), fuel(max)),
            ));
        }
    }
    // The destination's wind now against what this airframe has shown.
    if let (Some(w), Some(max)) = (s.dest_wind, s.max_crosswind_kts) {
        if let Some((x, rwy)) = best_crosswind(w, &s.dest_runway_hdgs).filter(|(x, _)| *x > max) {
            out.push(caution(
                "crosswind",
                format!(
                    "{}'s wind now is {:03.0} at {:.0}: {x:.0} knots across runway {rwy}, the best of them, over our demonstrated {max:.0}.",
                    s.dest_icao, w.0, w.1
                ),
            ));
        }
    }
    out
}

/// Everything [`Suitability`] needs, from the sim, the live profile, the
/// plan and the two airports (either may be unknown).
pub async fn gather_suitability(
    xp: &crate::xplane::XPlaneClient,
    fp: &crate::simbrief::FlightPlan,
    origin: Option<&crate::nav::AirportInfo>,
    dest: Option<&crate::nav::AirportInfo>,
) -> Suitability {
    let (acf, cap, weight) = tokio::join!(
        xp.aircraft_icao(),
        xp.get_float("sim/aircraft/weight/acf_m_fuel_tot"),
        xp.get_float(crate::datarefs::TOTAL_WEIGHT_KG),
    );
    let p = crate::aircraft::live_profile();
    let r = p.as_ref().and_then(|p| p.reference.as_ref());
    let takeoff_ft = r.and_then(|r| r.takeoff_distance_ft);
    // The departure end is the ONE runway rule (`aircraft::takeoff_needed_m`:
    // the roll at this weight, the margin, the floor) and the pilot's waiver
    // of the margin for this plan. The raw recited distance here refused the
    // Evolution at LILB right after the preflight had taken the waiver
    // (2026-09-25): two rules, two answers.
    let (roll_m, mtow_kg) = p
        .as_ref()
        .map(|p| crate::aircraft::profile_takeoff_roll(p))
        .unwrap_or((None, None));
    let waived = crate::full_auto::waived_for(fp).contains(&crate::full_auto::Gate::OriginRunway);
    let factor = if waived { 1.0 } else { crate::aircraft::TAKEOFF_FACTOR };
    let takeoff_needed_ft = roll_m.map(|_| {
        crate::aircraft::takeoff_needed_factored_m(
            roll_m,
            weight.ok().filter(|w| w.is_finite() && *w > 0.0),
            mtow_kg,
            500.0,
            factor,
        ) * 3.280_84
    });
    let landing_ft = crate::aircraft::landing_distance_m().map(|m| m * 3.280_84);
    let hdgs = dest.map_or_else(Vec::new, |a| {
        a.runways
            .iter()
            .flat_map(|r| {
                [
                    (r.ident_a.clone(), crate::nav::bearing_deg(r.lat_a, r.lon_a, r.lat_b, r.lon_b)),
                    (r.ident_b.clone(), crate::nav::bearing_deg(r.lat_b, r.lon_b, r.lat_a, r.lon_a)),
                ]
            })
            .collect()
    });
    let dest_icao = fp.destination.clone();
    let wind = tokio::task::spawn_blocking(move || {
        crate::weather::surface_wind(&crate::config::xplane_root(), &dest_icao)
    })
    .await
    .ok()
    .flatten();
    Suitability {
        // A local plan is built from the live airframe and carries its
        // DISPLAY name ("Cessna 172SP"), never an ICAO code to compare.
        plan_type: if fp.source == crate::simbrief::PlanSource::Local {
            String::new()
        } else {
            fp.aircraft.clone()
        },
        acf_type: acf.ok(),
        plan_types: p.as_ref().map(|p| p.simbrief_types.clone()).unwrap_or_default(),
        distance_nm: fp.distance_nm,
        block_fuel_kg: fp.block_fuel_kg,
        trip_fuel_kg: fp.trip_fuel_kg,
        tank_capacity_kg: cap.ok().filter(|c| c.is_finite() && *c > 0.0),
        burn_kg_hr: p.as_ref().and_then(|p| p.fuel.as_ref()).map(|f| f.cruise_burn_kg_hr).filter(|b| *b > 0.0),
        cruise_tas_kts: r.and_then(|r| r.cruise_kts),
        est_landing_kg: fp.weights.as_ref().map(|w| w.est_ldw_kg).filter(|w| *w > 0.0),
        max_landing_kg: fp.weights.as_ref().map(|w| w.max_ldw_kg).filter(|w| *w > 0.0),
        origin_longest_ft: origin.map(crate::flightplan::longest_runway_ft),
        dest_longest_ft: dest.map(crate::flightplan::longest_runway_ft),
        takeoff_needed_ft,
        landing_needed_ft: crate::aircraft::planned_landing_needed_ft(landing_ft, takeoff_ft),
        dest_wind: wind,
        dest_runway_hdgs: hdgs,
        max_crosswind_kts: r.and_then(|r| r.max_crosswind_kts),
        dest_icao: fp.destination.clone(),
    }
}

/// The planned STAR's crossing restrictions on this route, clamped into
/// `legs`, and the short line the arming ack carries (`None` with nothing to
/// say). A plan names its STAR but flies it as plain waypoints with
/// SimBrief's predicted altitudes; the chart's numbers are the ones owed.
pub fn arrival_restrictions(
    cifp_dir: &str,
    fp: &crate::simbrief::FlightPlan,
    legs: &mut [crate::nav::ProcLeg],
) -> (Vec<crate::nav::StarConstraint>, Option<String>) {
    let Some(star) = &fp.procedures.star else {
        return (Vec::new(), None);
    };
    let fixes: Vec<String> = legs.iter().map(|l| l.fix_ident.clone()).collect();
    let fixes: Vec<&str> = fixes.iter().map(String::as_str).collect();
    let cons = crate::nav::star_constraints(
        cifp_dir,
        &fp.destination,
        star,
        fp.procedures.dest_rwy.as_deref(),
        &fixes,
    );
    let changed = crate::procedure::apply_arrival_constraints(legs, &cons);
    crate::blackbox::record(
        "plan_audit",
        serde_json::json!({ "event": "star_constraints", "star": star, "runway": fp.procedures.dest_rwy, "changed_legs": changed,
                "constraints": cons.iter().map(|c| serde_json::json!({"fix": c.fix, "min_ft": c.min_ft,
                    "max_ft": c.max_ft, "max_kts": c.max_kts})).collect::<Vec<_>>() }),
    );
    let line = (!cons.is_empty()).then(|| {
        format!(
            "{star} restrictions at {} {}.",
            cons.len(),
            if cons.len() == 1 { "fix" } else { "fixes" }
        )
    });
    (cons, line)
}

/// The crew's own arrival: the STAR to the runway the destination's wind
/// will give us, spliced in where it meets the route, when SimBrief planned
/// another end. The arrival stage picks its runway by the same rule (the
/// destination's METAR, [`crate::full_auto::WIND_DECISIVE_KTS`]), so the STAR
/// flown ends where the approach begins; LIME's tower gave 10 to a plan for
/// 28 and full auto gave up (2026-09-25). The planned STAR is kept when it
/// serves the runway too (only its runway part changes); otherwise the STAR
/// entering the route earliest. The new fixes descend in a straight line
/// from the entry's altitude to the old route's last, and the STAR's own
/// windows then clamp them. `None` when nothing changed: calm or no METAR,
/// SimBrief's runway already, no STAR data, a local plan.
pub fn choose_arrival(
    cifp_dir: &str,
    fix_path: &str,
    nav_path: &str,
    apt_dat: &str,
    wind: Option<(f64, f64)>,
    fp: &mut crate::simbrief::FlightPlan,
    legs: &mut Vec<crate::nav::ProcLeg>,
) -> Option<String> {
    if fp.source == crate::simbrief::PlanSource::Local || legs.is_empty() {
        return None;
    }
    let apt = crate::nav::find_airport(apt_dat, &fp.destination).ok().flatten()?;
    let (dir, kts) = wind.filter(|(_, k)| *k >= crate::full_auto::WIND_DECISIVE_KTS)?;
    let (runway, _, _) = crate::nav::best_runway_for_wind(&apt.runways, dir, kts)?;
    let planned = fp.procedures.dest_rwy.clone();
    if planned.as_deref().and_then(crate::nav::runway_number) == crate::nav::runway_number(&runway) {
        return None;
    }
    let chosen = star_splice(
        cifp_dir,
        fix_path,
        nav_path,
        &apt,
        &fp.destination,
        &runway,
        fp.procedures.star.as_deref(),
        legs,
        0,
    );
    let spoken = crate::crew_radio::spell_runway(&runway);
    let Some(c) = chosen else {
        crate::blackbox::record(
            "plan_audit",
            serde_json::json!({ "event": "star_unchosen", "runway": runway, "planned": planned,
                "star": fp.procedures.star, "wind": [dir.round(), kts.round()] }),
        );
        fp.procedures.dest_rwy = Some(runway);
        return Some(format!(
            "The wind at {} favours runway {spoken}, and no arrival from our route serves it; I'll fly the route and set up from there.",
            fp.destination
        ));
    };
    legs.truncate(c.entry_index);
    legs.extend(c.legs);
    crate::blackbox::record(
        "plan_audit",
        serde_json::json!({ "event": "star_chosen", "star": c.ident, "entry": c.entry, "runway": runway,
            "planned_star": fp.procedures.star, "planned_runway": planned, "wind": [dir.round(), kts.round()],
            "fixes": legs[c.entry_index..].iter().map(|l| l.fix_ident.clone()).collect::<Vec<_>>() }),
    );
    fp.procedures.star = Some(c.ident.clone());
    fp.procedures.dest_rwy = Some(runway);
    Some(format!(
        "The wind at {} favours runway {spoken}, so the {} arrival from {}.",
        fp.destination, c.ident, c.entry
    ))
}

/// A STAR to one runway, spliced into the legs the crew flies: `legs[..entry_index]`
/// stay, `legs` here replace the rest.
#[derive(Debug, Clone)]
pub struct StarSplice {
    pub ident: String,
    /// The STAR's first fix on the route, and its index in the legs given.
    pub entry: String,
    pub entry_index: usize,
    /// The STAR from the entry on, positioned, as plain TF legs whose
    /// altitudes descend in a straight line (by distance) from the entry's
    /// altitude to the old route's last; the STAR's own windows clamp them
    /// after (`arrival_restrictions`, `nav::star_constraints`).
    pub legs: Vec<crate::nav::ProcLeg>,
}

/// The STAR serving `runway` that meets `legs` at or after `from` (the leg
/// flying, in flight; 0 at the arm), `prefer` first when it serves the
/// runway too (the STAR already flown only changes its runway part), else
/// the one entering earliest (`nav::stars_for_route`), its fixes all
/// positioned. `None`: no STAR from this route serves the runway. The arm's
/// pick (`choose_arrival`) and the in-flight re-pick (#79,
/// `full_auto::repick_arrival`) are this one rule.
#[allow(clippy::too_many_arguments)]
pub fn star_splice(
    cifp_dir: &str,
    fix_path: &str,
    nav_path: &str,
    apt: &crate::nav::AirportInfo,
    dest: &str,
    runway: &str,
    prefer: Option<&str>,
    legs: &[crate::nav::ProcLeg],
    from: usize,
) -> Option<StarSplice> {
    if from >= legs.len() {
        return None;
    }
    let idents: Vec<String> = legs[from..].iter().map(|l| l.fix_ident.clone()).collect();
    let mut choices = crate::nav::stars_for_route(cifp_dir, dest, runway, &idents);
    if let Some(star) = prefer {
        if let Some(k) = choices.iter().position(|c| c.ident.eq_ignore_ascii_case(star)) {
            let same = choices.remove(k);
            choices.insert(0, same);
        }
    }
    let (alat, alon) = apt.position()?;
    let c = choices.into_iter().find_map(|mut c| {
        crate::nav::resolve_proc_fixes(fix_path, nav_path, apt, dest, &mut c.legs, alat, alon);
        c.legs.iter().all(|l| l.fix.is_some()).then_some(c)
    })?;
    let entry_index = from + c.entry_index;
    let star_legs = realtitude_star(c.legs, legs[entry_index].alt1_ft, legs.last().and_then(|l| l.alt1_ft), entry_index);
    Some(StarSplice {
        ident: c.ident,
        entry: c.entry,
        entry_index,
        legs: star_legs,
    })
}

/// A STAR's legs as the follower flies them: TF legs from the entry, the
/// altitude a straight line by distance from `entry_alt` to `last_alt`, the
/// chart's own windows left to `apply_arrival_constraints`. Pure.
pub fn realtitude_star(
    mut star_legs: Vec<crate::nav::ProcLeg>,
    entry_alt: Option<f64>,
    last_alt: Option<f64>,
    seq_from: usize,
) -> Vec<crate::nav::ProcLeg> {
    let pos: Vec<Option<(f64, f64)>> = star_legs.iter().map(|l| l.fix.as_ref().map(|f| (f.lat, f.lon))).collect();
    let mut along = Vec::with_capacity(pos.len());
    let mut d = 0.0;
    let mut prev: Option<(f64, f64)> = None;
    for p in &pos {
        if let (Some(q), Some(p)) = (prev, p) {
            d += crate::nav::haversine_nm(q.0, q.1, p.0, p.1);
        }
        if p.is_some() {
            prev = *p;
        }
        along.push(d);
    }
    let total = along.last().copied().unwrap_or(0.0).max(0.1);
    for (i, l) in star_legs.iter_mut().enumerate() {
        l.alt1_ft = match (entry_alt, last_alt) {
            (Some(a), Some(b)) => Some(a + (b - a) * along[i] / total),
            (a, b) => a.or(b),
        };
        l.alt_desc = ' ';
        l.alt2_ft = None;
        l.transition = String::new();
        l.leg_type = "TF".to_string();
        l.seq = (seq_from + i) as u32;
    }
    star_legs
}

/// The fix nearest `d` nm along, for naming a place.
fn nearest_fix(profile: &[Crossing], d: f64) -> String {
    profile
        .iter()
        .min_by(|a, b| {
            (a.along_nm - d)
                .abs()
                .partial_cmp(&(b.along_nm - d).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map_or_else(|| "the departure".to_string(), |c| c.fix.clone())
}

/// The point `d` nm along the route's polyline.
fn position_at(pts: &[(String, f64, f64)], d: f64) -> Option<(f64, f64)> {
    let mut along = 0.0;
    for w in pts.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        let seg = crate::nav::haversine_nm(a.1, a.2, b.1, b.2);
        if d <= along + seg {
            let brg = crate::nav::bearing_deg(a.1, a.2, b.1, b.2);
            return Some(crate::nav::project_latlon(a.1, a.2, brg, (d - along).max(0.0)));
        }
        along += seg;
    }
    None
}

/// Probe stations along the first `nm` of the route, every `step` nm, on the
/// track and `offset` nm either side of it: (nm along, lat, lon).
pub fn departure_stations(
    origin: (f64, f64),
    legs: &[ProcLeg],
    nm: f64,
    step: f64,
    offset: f64,
) -> Vec<(f64, f64, f64)> {
    let mut pts = vec![("origin".to_string(), origin.0, origin.1)];
    pts.extend(
        legs.iter()
            .filter_map(|l| l.fix.as_ref().map(|f| (l.fix_ident.clone(), f.lat, f.lon))),
    );
    let mut out = Vec::new();
    let mut d = step;
    while d <= nm {
        let Some((lat, lon)) = position_at(&pts, d) else { break };
        let Some((la2, lo2)) = position_at(&pts, d + 0.1) else { break };
        let brg = crate::nav::bearing_deg(lat, lon, la2, lo2);
        out.push((d, lat, lon));
        if offset > 0.0 {
            for side in [-90.0, 90.0] {
                let (a, b) = crate::nav::project_latlon(lat, lon, brg + side, offset);
                out.push((d, a, b));
            }
        }
        d += step;
    }
    out
}

/// The whole report, spoken. `full` names every finding; the arm's version
/// names the refusal or at most two cautions.
pub fn spoken(a: &Audit, full: bool) -> String {
    if let Some(s) = a.stop() {
        return s.line.clone();
    }
    let cautions: Vec<&Finding> = a.cautions().collect();
    let unchecked = if a.unchecked.is_empty() {
        String::new()
    } else {
        format!(" I couldn't check {}.", join(&a.unchecked))
    };
    if cautions.is_empty() {
        return format!("The plan checks out: {}.{unchecked}", join(&a.checked));
    }
    if !full {
        return arm_heads_up(&cautions, &unchecked);
    }
    let shown = cautions.len();
    let mut line = match cautions.len() {
        1 => "One thing on the plan. ".to_string(),
        n => format!("{n} things on the plan. "),
    };
    for c in &cautions[..shown] {
        line.push_str(&c.line);
        line.push(' ');
    }
    if shown < cautions.len() {
        line.push_str("Ask me to check the plan for the rest. ");
    }
    format!("{}{unchecked}", line.trim_end())
}

/// A caution as the ARM says it: what, not the survey. The whole finding
/// ("The climb out is tight: 850 feet over the ground 2 miles out, near
/// D300R, where it rises to 100 feet") read into a 231-character arming
/// reply held every line behind it for fifteen seconds, and three of them
/// were dropped stale (LIPK 2026-09-26). The numbers are on the Brief tab
/// and in "check the plan"; a kind with no short form keeps its line.
fn brief_caution(f: &Finding) -> String {
    match f.kind {
        "climb_terrain_tight" => "the climb out is tight over the ground".into(),
        "under_grid_mora" => "a leg runs under the grid minimum".into(),
        "star_high" => "we'll be high on the arrival".into(),
        "star_low" => "we'll be low on the arrival".into(),
        "star_speed_below_floor" => "an arrival speed is under our clean minimum".into(),
        "cruise_not_reached" => "we won't make the cruise level".into(),
        "unpressurised_cruise" => "the cruise is high without pressurisation".into(),
        "fix_unresolved" => "there's a fix I can't find".into(),
        _ => f.line.trim_end_matches('.').to_string(),
    }
}

fn arm_heads_up(cautions: &[&Finding], unchecked: &str) -> String {
    let items: Vec<String> = cautions.iter().take(2).map(|c| brief_caution(c)).collect();
    let refs: Vec<&str> = items.iter().map(String::as_str).collect();
    let more = if cautions.len() > 2 { ", and more on the brief" } else { "" };
    let first = refs.join(" and ");
    format!("Heads up: {first}{more}.{unchecked}")
}

fn join(items: &[&str]) -> String {
    match items {
        [] => String::new(),
        [a] => a.to_string(),
        [rest @ .., last] => format!("{}, and {last}", rest.join(", ")),
    }
}

/// Distance probed along the departure, nm, and the station spacing.
const DEPARTURE_PROBE_NM: f64 = 30.0;
const DEPARTURE_STEP_NM: f64 = 0.5;
/// Either side of the track: the follower's turns cut corners.
const DEPARTURE_OFFSET_NM: f64 = 1.0;

/// The airframe as published for the live profile and the follower's rates.
pub fn live_airframe(ceiling_ft: Option<f64>) -> Airframe {
    let p = crate::aircraft::live_profile();
    let r = p.as_ref().and_then(|p| p.reference.as_ref());
    Airframe {
        ceiling_ft: ceiling_ft.or(r.and_then(|r| r.service_ceiling_ft)),
        pressurised: p.as_ref().and_then(|p| p.pressurised),
        climb_fpm: crate::procedure::climb_fpm(),
        best_climb_fpm: crate::procedure::best_climb_fpm(),
        descent_fpm: crate::procedure::descent_fpm(),
        climb_ias_kts: r.and_then(|r| r.vy_kts).unwrap_or(80.0),
        cruise_ias_kts: r.and_then(|r| r.cruise_ias_kts).unwrap_or(120.0),
        speed_floor_kts: crate::aircraft::speed_floor_kts(),
    }
}

/// Gather what the sim and the data files know and run the audit: the grid
/// from [`crate::terrain::mora_grid`], the departure probed only when
/// `probe` (on the ground at the origin: the scenery loaded is the scenery
/// around the aircraft).
pub async fn check(
    xp: &crate::xplane::XPlaneClient,
    fix_data_path: &str,
    fp: &crate::simbrief::FlightPlan,
    legs: &[ProcLeg],
    stars: &[StarConstraint],
    origin_elev_ft: f64,
    airframe: Airframe,
    suitability: Option<&Suitability>,
    probe: bool,
) -> Audit {
    let (Some(o), Some(d)) = (fp.waypoints.first(), fp.waypoints.last()) else {
        return Audit::default();
    };
    // The flight this check belongs to: a verdict that lands after a new
    // flight began is that old flight's, and the EFB never shows it.
    let generation = crate::efb::feed::AUDIT.now();
    let fix = fix_data_path.to_string();
    let mora = tokio::task::spawn_blocking(move || crate::terrain::mora_grid(&fix))
        .await
        .ok()
        .flatten();
    let mut ground: Vec<(f64, f64)> = Vec::new();
    if probe {
        let stations = departure_stations(
            (o.lat, o.lon),
            legs,
            DEPARTURE_PROBE_NM,
            DEPARTURE_STEP_NM,
            DEPARTURE_OFFSET_NM,
        );
        // A chunk the scenery cannot answer ends the probe: what came back
        // is still measured ground, and the rest is said as unchecked.
        for chunk in stations.chunks(crate::terrain::MAX_POINTS) {
            let pts: Vec<(f64, f64)> = chunk.iter().map(|s| (s.1, s.2)).collect();
            let Some(h) = crate::terrain::sample_ft(xp, &pts).await else {
                break;
            };
            for (s, g) in chunk.iter().zip(h) {
                match ground.last_mut() {
                    Some(last) if (last.0 - s.0).abs() < 1e-9 => last.1 = last.1.max(g),
                    _ => ground.push((s.0, g)),
                }
            }
        }
    }
    let inputs = Inputs {
        origin: (o.lat, o.lon),
        origin_elev_ft,
        dest: (d.lat, d.lon),
        legs,
        cruise_alt_ft: fp.cruise_alt_ft,
        stars,
        mora,
        departure_ground: (!ground.is_empty()).then_some(ground.as_slice()),
        airframe,
        suitability,
    };
    let audit = audit(&inputs);
    crate::blackbox::record(
        "plan_audit",
        serde_json::json!({
            "origin": fp.origin, "destination": fp.destination,
            "findings": audit.findings.iter().map(|f| serde_json::json!({
                "severity": format!("{:?}", f.severity), "kind": f.kind, "line": f.line})).collect::<Vec<_>>(),
            "profile": audit.profile.iter().map(|c| (c.fix.clone(), c.along_nm.round(), c.alt_ft.round())).collect::<Vec<_>>(),
            "probed_nm": ground.last().map(|g| g.0),
            "mora": mora.is_some(),
        }),
    );
    crate::efb::feed::note_audit(generation, &audit);
    audit
}

#[cfg(test)]
mod tests {

    /// LIPE, SimBrief's BAME2S for 30 with the wind straight down 12: the
    /// crew flies BAME5M from BAMEK. Needs the X-Plane install.
    /// `cargo test -- --ignored real_install_lipe_star_rechosen --nocapture`
    #[test]
    #[ignore]
    fn real_install_lipe_star_rechosen() {
        use crate::nav::ProcKind;
        let config = crate::config::Config::from_env().expect("config");
        let apt = crate::nav::find_airport(&config.apt_data_path, "LIPE").unwrap().unwrap();
        let (alat, alon) = apt.position().unwrap();
        let mut star =
            crate::nav::load_proc_legs(&config.cifp_dir, "LIPE", ProcKind::Star, "BAME2S", Some("RW30"));
        crate::nav::resolve_proc_fixes(
            &config.fix_data_path, &config.nav_data_path, &apt, "LIPE", &mut star, alat, alon,
        );
        let waypoints: Vec<crate::simbrief::Waypoint> = star
            .iter()
            .filter_map(|l| l.fix.as_ref().map(|f| (l.fix_ident.clone(), f.lat, f.lon)))
            .enumerate()
            .map(|(i, (ident, lat, lon))| crate::simbrief::Waypoint { ident, lat, lon, alt: 12000.0 - 1500.0 * i as f64 })
            .collect();
        let mut fp = crate::simbrief::FlightPlan {
            transition_altitude_ft: None,
            transition_level_ft: None,
            procedures: Default::default(),
            callsign: None,
            schedule: Default::default(),
            source: crate::simbrief::PlanSource::SimBrief,
            origin: "LIME".into(),
            destination: "LIPE".into(),
            alternate: String::new(),
            aircraft: String::new(),
            cruise_alt_ft: 12000.0,
            distance_nm: 100.0,
            ete_min: 0.0,
            trip_fuel_kg: 0.0,
            reserve_kg: 0.0,
            block_fuel_kg: 0.0,
            min_takeoff_fuel_kg: 0.0,
            weights: None,
            route: String::new(),
            wind_comp_kts: 0.0,
            wind_dir: 0.0,
            wind_spd_kts: 0.0,
            tod_to_dest_nm: 0.0,
            waypoints: Vec::new(),
        };
        fp.procedures.star = Some("BAME2S".into());
        fp.procedures.dest_rwy = Some("30".into());
        let mut legs = crate::procedure::enroute_legs(&waypoints, "LIME", "LIPE", 12000.0);
        println!("before {:?}", legs.iter().map(|l| (l.fix_ident.clone(), l.alt1_ft)).collect::<Vec<_>>());
        let line = super::choose_arrival(
            &config.cifp_dir, &config.fix_data_path, &config.nav_data_path, &config.apt_data_path,
            Some((120.0, 15.0)), &mut fp, &mut legs,
        );
        let (cons, _) = super::arrival_restrictions(&config.cifp_dir, &fp, &mut legs);
        println!("{line:?}");
        println!("after {:?}", legs.iter().map(|l| (l.fix_ident.clone(), l.alt1_ft)).collect::<Vec<_>>());
        assert!(line.unwrap().contains("BAME5M"));
        assert_eq!(fp.procedures.dest_rwy.as_deref(), Some("12"));
        assert_eq!(legs.last().map(|l| l.fix_ident.as_str()), Some("EMDUD"));
        assert!(legs.iter().all(|l| l.fix.is_some() && l.alt1_ft.is_some()));
        assert!(!cons.is_empty(), "BAME5M's windows for 12 apply");
        let emdud = legs.last().unwrap().alt1_ft.unwrap();
        assert!(emdud >= 2500.0, "{emdud}");
        // Calm: SimBrief's arrival stands.
        let mut fp2 = fp.clone();
        fp2.procedures.dest_rwy = Some("30".into());
        let mut legs2 = crate::procedure::enroute_legs(&waypoints, "LIME", "LIPE", 12000.0);
        assert!(super::choose_arrival(
            &config.cifp_dir, &config.fix_data_path, &config.nav_data_path, &config.apt_data_path,
            Some((120.0, 2.0)), &mut fp2, &mut legs2,
        ).is_none());
    }
    use super::*;
    use crate::nav::Fix;

    fn leg(id: &str, lat: f64, lon: f64, alt: f64) -> ProcLeg {
        ProcLeg {
            seq: 0,
            transition: String::new(),
            leg_type: "TF".into(),
            fix_ident: id.into(),
            fix_region: String::new(),
            fix: Some(Fix { ident: id.into(), lat, lon }),
            course_mag: None,
            dist_nm: None,
            turn: None,
            alt_desc: ' ',
            alt1_ft: Some(alt),
            alt2_ft: None,
            is_faf: false,
            is_map: false,
            vpa_deg: None,
            navaid_ident: String::new(),
            navaid_region: String::new(),
            navaid: None,
            path: Default::default(),
        }
    }

    fn jet() -> Airframe {
        Airframe {
            ceiling_ft: Some(41_000.0),
            pressurised: Some(true),
            climb_fpm: 2_000.0,
            best_climb_fpm: 3_000.0,
            descent_fpm: 2_000.0,
            climb_ias_kts: 250.0,
            cruise_ias_kts: 280.0,
            speed_floor_kts: Some(180.0),
        }
    }

    /// Due north from (50, 0), one fix every `nm`.
    fn route(alts: &[f64], nm: f64) -> Vec<ProcLeg> {
        alts.iter()
            .enumerate()
            .map(|(k, a)| leg(&format!("FIX{k}"), 50.0 + (k as f64 + 1.0) * nm / 60.0, 0.0, *a))
            .collect()
    }

    fn inputs<'a>(legs: &'a [ProcLeg], stars: &'a [StarConstraint]) -> Inputs<'a> {
        let n = legs.len() as f64 + 1.0;
        Inputs {
            origin: (50.0, 0.0),
            origin_elev_ft: 0.0,
            dest: (50.0 + n * 30.0 / 60.0, 0.0),
            legs,
            cruise_alt_ft: 30_000.0,
            stars,
            mora: None,
            departure_ground: None,
            airframe: jet(),
            suitability: None,
        }
    }

    #[test]
    fn a_clean_plan_checks_out_and_says_what_it_could_not_check() {
        let legs = route(&[30_000.0, 30_000.0, 30_000.0, 30_000.0, 10_000.0], 30.0);
        let a = audit(&inputs(&legs, &[]));
        assert!(a.findings.is_empty(), "{:?}", a.findings);
        let line = spoken(&a, true);
        assert!(line.starts_with("The plan checks out"), "{line}");
        assert!(line.contains("couldn't check the ground near the departure"), "{line}");
    }

    #[test]
    fn a_climb_into_probed_ground_is_refused_and_a_tight_one_said() {
        let legs = route(&[30_000.0, 30_000.0, 30_000.0], 30.0);
        // At about 255 kt true the best 3,000 fpm is some 700 ft a mile, the
        // normal 2,000 some 470: 3,500 and 2,350 five miles out.
        let ridge = [(5.0, 4_000.0)];
        let mut i = inputs(&legs, &[]);
        i.departure_ground = Some(&ridge);
        let a = audit(&i);
        let s = a.stop().expect("refused");
        assert_eq!(s.kind, "climb_terrain");
        assert!(s.line.contains("FIX0"), "{}", s.line);
        // Lower: clears at best rate, tight at the normal climb.
        let hill = [(5.0, 2_000.0)];
        i.departure_ground = Some(&hill);
        let a = audit(&i);
        assert!(a.stop().is_none());
        assert!(a.findings.iter().any(|f| f.kind == "climb_terrain_tight"), "{:?}", a.findings);
    }

    #[test]
    fn an_enroute_leg_under_the_grid_is_a_caution_naming_the_ground() {
        let legs = route(&[5_000.0, 5_000.0, 5_000.0, 5_000.0], 30.0);
        let grid = MoraGrid::parse("+51 -001 010 080\n+50 -001 010 010".lines());
        let mut i = inputs(&legs, &[]);
        i.cruise_alt_ft = 5_000.0;
        i.mora = Some(&grid);
        let a = audit(&i);
        let f = a.findings.iter().find(|f| f.kind == "under_grid_mora").expect("said");
        assert_eq!(f.severity, Severity::Caution);
        assert!(f.line.contains("8,000") && f.line.contains("6,000"), "{}", f.line);
    }

    #[test]
    fn a_star_restriction_the_descent_cannot_make_is_said_with_the_numbers() {
        // AGPE1E's shape: "at FL260" then FL070 40 nm on, which 2,000 fpm
        // cannot join. (Without the "at", the ceiling would bring the level-off
        // down instead, and there is nothing to be high for.)
        let legs = route(&[30_000.0, 26_000.0, 7_000.0], 40.0);
        let c = |fix: &str, min, max, kts| StarConstraint { fix: fix.into(), min_ft: min, max_ft: max, max_kts: kts };
        let stars = [c("FIX1", Some(26_000.0), Some(26_000.0), None), c("FIX2", None, Some(7_000.0), Some(160.0))];
        let a = audit(&inputs(&legs, &stars));
        let high = a.findings.iter().find(|f| f.kind == "star_high").unwrap_or_else(|| panic!("{:?} {:?}", a.findings, a.profile));
        assert!(high.line.contains("FIX2") && high.line.contains("7,000"), "{}", high.line);
        assert!(a.findings.iter().any(|f| f.kind == "star_speed_below_floor"));
        assert!(a.stop().is_none(), "a caution, not a refusal");
        // Given room (90 nm) the ceiling brings the descent forward in time.
        let legs = route(&[30_000.0, 30_000.0, 30_000.0, 30_000.0, 30_000.0, 7_000.0], 30.0);
        let stars = [StarConstraint { fix: "FIX5".into(), min_ft: None, max_ft: Some(7_000.0), max_kts: None }];
        let a = audit(&inputs(&legs, &stars));
        assert!(a.findings.iter().all(|f| f.kind != "star_high"), "{:?}", a.findings);
    }

    /// #156: HEME1B's "HEMEL at FL220" in EGKK-EGBB's FL180 plan: the plan
    /// flies it at the cruise, and the audit says it as that, once, not as
    /// a crossing 4,000 feet low.
    #[test]
    fn a_star_window_over_the_cruise_is_said_as_flown_at_the_cruise() {
        let legs = route(&[18_000.0, 18_000.0, 12_000.0, 9_000.0], 25.0);
        let c = |fix: &str, min, max| StarConstraint { fix: fix.into(), min_ft: min, max_ft: max, max_kts: None };
        let stars = [c("FIX1", Some(22_000.0), Some(22_000.0)), c("FIX3", Some(9_000.0), Some(9_000.0))];
        let mut i = inputs(&legs, &stars);
        i.cruise_alt_ft = 18_000.0;
        let a = audit(&i);
        let f = a.findings.iter().find(|f| f.kind == "star_above_cruise").unwrap_or_else(|| panic!("{:?}", a.findings));
        assert_eq!(f.line, "FIX1's 22,000 feet is above our cruise. We'll cross it at 18,000 feet.");
        assert!(a.findings.iter().all(|f| f.kind != "star_low"), "{:?}", a.findings);
        assert!(a.stop().is_none());
    }

    #[test]
    fn a_short_plan_that_never_reaches_cruise_is_said() {
        let legs = route(&[30_000.0, 5_000.0], 10.0);
        let a = audit(&inputs(&legs, &[]));
        assert!(a.findings.iter().any(|f| f.kind == "cruise_not_reached"), "{:?}", a.findings);
    }

    #[test]
    fn the_arm_says_two_cautions_short_and_points_at_the_rest() {
        let f = |k| Finding { severity: Severity::Caution, kind: k, line: format!("{k}.") };
        let a = Audit { findings: vec![f("a"), f("b"), f("c")], ..Default::default() };
        let arm = spoken(&a, false);
        assert_eq!(arm, "Heads up: a and b, and more on the brief.");
        assert!(spoken(&a, true).contains("c."));
        // A known kind is said short at the arm: what, never the survey.
        let tight = Finding {
            severity: Severity::Caution,
            kind: "climb_terrain_tight",
            line: "The climb out is tight: 850 feet over the ground 2 miles out, near D300R, where it rises to 100 feet.".into(),
        };
        let a = Audit { findings: vec![tight], ..Default::default() };
        assert_eq!(spoken(&a, false), "Heads up: the climb out is tight over the ground.");
        assert!(spoken(&a, true).contains("850 feet"), "the full report keeps the numbers");
    }

    fn suits() -> Suitability {
        Suitability {
            plan_type: "B738".into(),
            acf_type: Some("B738".into()),
            distance_nm: 500.0,
            block_fuel_kg: 8_000.0,
            trip_fuel_kg: 5_000.0,
            tank_capacity_kg: Some(20_800.0),
            burn_kg_hr: Some(2_500.0),
            cruise_tas_kts: Some(450.0),
            dest_icao: "EGPH".into(),
            ..Default::default()
        }
    }

    #[test]
    fn a_plan_for_another_type_is_refused_first() {
        let mut s = suits();
        s.acf_type = Some("C172".into());
        // A custom SimBrief airframe filed under another type is ours when
        // the profile says so (the Evolution on the Piper M500).
        let mut evo = s.clone();
        evo.acf_type = Some("EVOT".into());
        evo.plan_type = "P46T".into();
        assert!(suitability(&evo, 3000.0, &jet()).iter().any(|f| f.kind == "type_mismatch"));
        evo.plan_types = vec!["P46T".into()];
        assert!(!suitability(&evo, 3000.0, &jet()).iter().any(|f| f.kind == "type_mismatch"));
        let f = suitability(&s, 35_000.0, &jet());
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].kind, "type_mismatch");
        assert!(f[0].line.contains("B738") && f[0].line.contains("C172"));
        assert!(suitability(&suits(), 35_000.0, &jet()).is_empty(), "a matching plan is fine");
    }

    #[test]
    fn range_is_judged_on_the_tanks_and_on_our_own_burn() {
        let mut s = suits();
        s.block_fuel_kg = 22_000.0;
        assert!(suitability(&s, 35_000.0, &jet()).iter().any(|f| f.kind == "block_over_capacity"));
        // 3,500 nm at 450 kt and 2,500 kg/h is 19,400 kg plus 1,875 reserve.
        let mut s = suits();
        s.distance_nm = 3_500.0;
        s.trip_fuel_kg = 19_000.0;
        let f = suitability(&s, 35_000.0, &jet());
        assert!(f.iter().any(|f| f.kind == "out_of_range" && f.severity == Severity::Stop), "{f:?}");
        // A plan whose trip burn is a third of ours was planned for something else.
        let mut s = suits();
        s.trip_fuel_kg = 1_000.0;
        assert!(suitability(&s, 35_000.0, &jet()).iter().any(|f| f.kind == "trip_fuel_low"));
    }

    #[test]
    fn runways_ceiling_and_crosswind_against_the_airframe() {
        let mut s = suits();
        s.dest_longest_ft = Some(4_000.0);
        s.landing_needed_ft = Some(5_000.0);
        assert!(suitability(&s, 35_000.0, &jet()).iter().any(|f| f.kind == "dest_runway_short"));
        assert!(suitability(&suits(), 45_000.0, &jet()).iter().any(|f| f.kind == "above_ceiling"));
        let mut s = suits();
        s.dest_wind = Some((330.0, 30.0));
        s.dest_runway_hdgs = vec![("24".into(), 240.0), ("06".into(), 60.0)];
        s.max_crosswind_kts = Some(25.0);
        let f = suitability(&s, 35_000.0, &jet());
        let x = f.iter().find(|f| f.kind == "crosswind").expect("30 kt straight across");
        assert_eq!(x.severity, Severity::Caution);
        s.dest_runway_hdgs.push(("30".into(), 300.0));
        assert!(suitability(&s, 35_000.0, &jet()).iter().all(|f| f.kind != "crosswind"), "15 kt on 30");
    }

    #[test]
    fn departure_stations_walk_the_track_with_both_sides() {
        let legs = route(&[30_000.0], 10.0);
        let s = departure_stations((50.0, 0.0), &legs, 3.0, 1.0, 1.0);
        assert_eq!(s.len(), 9);
        assert!((s[0].1 - (50.0 + 1.0 / 60.0)).abs() < 1e-3);
        assert!(s[1].2 < 0.0 && s[2].2 > 0.0, "west and east of a northbound track");
    }
}
