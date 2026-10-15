//! The monitor's checks: stall and overspeed, the engine, carb ice, vacuum and
//! alternator, top of descent, the profile, sink and busts, cabin and park
//! brake, fuel, the altimeter, the centre tank, icing, the step climb, the
//! gyro sync, flap overspeed, the stab trim and endurance.

use super::*;

/// Build the checks enabled for this airframe.
/// Below this AGL the aircraft is in the landing flare, where slow-and-low is
/// *intended* — so the airspeed callout is suppressed there to avoid nagging on
/// every normal touchdown.
pub(super) const FLARE_AGL_FT: f64 = 50.0;

/// Approach-to-stall airspeed callout: the copilot's "Airspeed!" — it warns and
/// prompts the recovery, it never touches the controls (intent is the pilot's;
/// auto-recovery would fight intentional slow flight). Airborne and above the
/// flare only, sustained so a momentary dip in a turn doesn't trip it. Banded
/// like the fuel check: a caution as you approach the stall, an urgent prompt
/// below it. The threshold scales with flaps (`aircraft::stall_speed_in_config`)
/// so a normal slow-but-safe flaps-down approach doesn't trip it.

/// How far ahead (seconds) to project a decaying airspeed. At a 3 kt/s washout
/// this fires the caution ~12 kt early — enough lead to call it well before a
/// mushing autopilot reaches its envelope limit and disconnects. Long enough to
/// get ahead of the decay, short enough that a brief, intentional power
/// reduction settles before it nags.
pub(super) const STALL_LOOKAHEAD_S: f64 = 4.0;

/// How far ahead (seconds) to project a *building* airspeed for the overspeed
/// callout — so a descent accelerating toward redline is called with lead time,
/// not at Vne. Only a rising trend counts (see `Overspeed::eval`).
pub(super) const OVERSPEED_LOOKAHEAD_S: f64 = 4.0;
/// The overspeed caution's floor, a fraction of Vne; the follower schedules
/// its cruise under it (`procedure::cruise_cap_kts`).
pub(crate) const OVERSPEED_CAUTION_FACTOR: f64 = crate::aircraft::OVERSPEED_CAUTION_FACTOR;

pub(super) struct StallSpeed {
    pub(super) id: &'static str,
    pub(super) severity: Severity,
    /// Clean stall speed (kts) the bands scale off.
    pub(super) vs_clean_kts: f64,
    /// Landing-configuration stall speed, when the profile declares one. The
    /// full-flap stall is then THIS number and the flap-adjusted speed runs
    /// straight between the two; without it the 17% trainer reduction
    /// applies, which on the A330 (Vs 175, Vso 125) put a stabilised 150 kt
    /// final inside the WARNING band the whole way down (2026-09-20).
    pub(super) vs0_kts: Option<f64>,
    /// The detents that declare their own stall speed, `(ratio, kts)`
    /// (`aircraft::detent_stall_points`): a slatted wing is not a straight
    /// line (the A330's CONF 2 is 139, not 150; EGPF 2026-09-21).
    pub(super) detent_stall: Vec<(f64, f64)>,
    /// The weight the stalls above are quoted at
    /// (`reference.stall_ref_weight_kg`): the band then rides the stall at
    /// the snapshot's weight ([`crate::aircraft::weight_stall_scale`]).
    /// `None`, or no weight read, keeps the declared stalls.
    pub(super) stall_ref_weight_kg: Option<f64>,
    /// Fires when IAS ≤ this × the flap-adjusted stall speed …
    pub(super) at_or_below_factor: f64,
    /// … and (if set) still above this × it — the more urgent band owns below.
    pub(super) above_factor: Option<f64>,
}

/// Whether a route automation owns the vertical profile, so the ring's TOD
/// and profile advisories (and the step-climb advice) keep silent: the
/// procedure follower, or the helicopter route, which flies its own approach
/// down to its gate. LEMG-LXGB (S-76, iMac 2026-09-29, #191) heard "High on
/// the profile" and "past the descent point, about 1600 feet a minute" while
/// the route was flying its descent. Pure.
pub fn follower_flying(procedure: bool, heli_route: bool) -> bool {
    procedure || heli_route
}

/// With an automation flying, the caution band's top: the margin under its
/// own target, not the hand-flown 1.2 Vs.
pub(super) const AUTO_CAUTION_FACTOR: f64 = crate::aircraft::CAUTION_AUTO_RATIO;

/// The stall speed at this flap setting: a straight line from the clean
/// stall to the landing-configuration stall when the profile declares one,
/// else the trainer's fixed reduction. Never above clean, never below Vso.
#[cfg(test)]
pub fn stall_speed_at_flap(vs_clean_kts: f64, vs0_kts: Option<f64>, flap_ratio: f64) -> f64 {
    crate::aircraft::stall_speed_in_config(vs_clean_kts, vs0_kts, &[], flap_ratio)
}

impl Check for StallSpeed {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        self.severity
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        // Airborne and above the flare. Engine state is irrelevant — a glide can
        // stall too.
        if s.on_ground || s.agl_ft < FLARE_AGL_FT {
            return None;
        }
        // Stall speed for the current flap setting.
        let vs = crate::aircraft::stall_speed_in_config(
            self.vs_clean_kts,
            self.vs0_kts,
            &self.detent_stall,
            s.flap_ratio,
        ) * crate::aircraft::weight_stall_scale(self.stall_ref_weight_kg, s.all_up_kg)
            .unwrap_or(1.0);
        // Anticipate: when the speed is washing off, project it forward so the
        // band trips while there's still margin to recover. Only a *decaying*
        // trend counts (`.min(0.0)`), so this can lower the effective speed but
        // never raises it — it never masks a genuine low reading.
        //
        // With an AUTOMATION flying, the deceleration is its own, toward a
        // target it chose: the trend is not a warning and the caution band
        // is the margin under that target, not the 1.2 Vs a hand-flown
        // approach wants. A 737's Vref is 1.23 Vso, so the hand band called
        // "you're getting slow" through every normal slowdown to it, to a
        // pilot who was not flying (LEMG circuit, 2026-09-26).
        let auto = s.automation_flying;
        let caution = self.severity == Severity::Caution;
        let projected = if auto && caution {
            s.ias_kts
        } else {
            s.ias_kts + s.ias_trend_kts_per_s.min(0.0) * STALL_LOOKAHEAD_S
        };
        let at_or_below = if auto && caution { AUTO_CAUTION_FACTOR } else { self.at_or_below_factor };
        let in_band = projected <= at_or_below * vs
            && self.above_factor.is_none_or(|a| projected > a * vs);
        if !in_band {
            return None;
        }
        // Who is flying decides the words: never "you" to a pilot who is
        // not, never an order to the pilot's hands.
        Some(match (self.severity, auto) {
            (Severity::Warning, true) => "Airspeed low!".to_string(),
            (Severity::Warning, false) => "Airspeed! Lower the nose, add power.".to_string(),
            (_, true) => "Speed's getting low.".to_string(),
            (_, false) => "Airspeed, you're getting slow.".to_string(),
        })
    }
    fn min_duration(&self) -> Duration {
        // Speak quickly — a decaying airspeed needs lead time, not a long
        // confirm. Just enough to ignore a single-frame glitch (~1–2 samples at
        // the monitor's poll rate); the urgent band reacts fastest.
        match self.severity {
            Severity::Warning => Duration::from_millis(300),
            _ => Duration::from_millis(700),
        }
    }
}

/// Overspeed callout: the copilot's "Airspeed!" on the *high* side — it warns as
/// the speed builds toward Vne (e.g. a descent left at cruise power) and prompts
/// the recovery; like the stall check it never touches the controls. Airborne
/// only; banded — a caution approaching redline, an urgent prompt at or above it.
pub(super) struct Overspeed {
    pub(super) id: &'static str,
    pub(super) severity: Severity,
    /// Never-exceed speed (kts) the bands scale off: the LOW one on an
    /// airframe with a `vmo_high:`.
    pub(super) vne_kts: f64,
    /// The higher Vmo above an altitude (the Citation X's 350 over 8,000
    /// ft): the bands scale off the limit at the indicated altitude
    /// (`aircraft::vne_at`). `None` = `vne_kts` everywhere.
    pub(super) vmo_high: Option<crate::aircraft::VmoHigh>,
    /// Fires when IAS ≥ this × Vne …
    pub(super) at_or_above_factor: f64,
    /// … and (if set) still below this × Vne — the more urgent band owns above.
    pub(super) below_factor: Option<f64>,
}

/// The overspeed warning's words, by what the aircraft is doing. "Ease off
/// the descent" was said five times to an A330 climbing at 2,600 fpm (EGPH,
/// 2026-09-20): the pilot can see the nose is up, and an instruction that is
/// visibly wrong takes the whole sentence with it.
pub fn overspeed_line(vsi_fpm: f64) -> &'static str {
    if vsi_fpm < -300.0 {
        "Overspeed! Reduce power, ease off the descent."
    } else {
        "Overspeed! Reduce power."
    }
}

impl Check for Overspeed {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        self.severity
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground {
            return None; // can't meaningfully overspeed on the ground
        }
        // Anticipate: project only a *rising* trend forward (`.max(0.0)`), so the
        // band trips with margin before redline. A decaying trend never raises the
        // projected speed, so it can't mask a falling-back-to-safe reading.
        let projected = s.ias_kts + s.ias_trend_kts_per_s.max(0.0) * OVERSPEED_LOOKAHEAD_S;
        let vne = crate::aircraft::vne_at(Some(self.vne_kts), self.vmo_high, s.alt_ft)
            .unwrap_or(self.vne_kts);
        let in_band = projected >= self.at_or_above_factor * vne
            && self.below_factor.is_none_or(|b| projected < b * vne);
        if !in_band {
            return None;
        }
        Some(match self.severity {
            Severity::Warning => overspeed_line(s.vsi_fpm).to_string(),
            _ => "Airspeed — you're getting fast.".to_string(),
        })
    }
    fn min_duration(&self) -> Duration {
        match self.severity {
            Severity::Warning => Duration::from_millis(300),
            _ => Duration::from_millis(700),
        }
    }
}

/// Which engine instrument an [`EngineHealth`] check watches.
#[derive(Clone, Copy)]
pub(super) enum EngineParam {
    OilPressure,
    OilTemp,
    Cht,
}

impl EngineParam {
    pub(super) fn value(self, s: &Snapshot) -> Option<f64> {
        match self {
            EngineParam::OilPressure => s.oil_pressure_psi,
            EngineParam::OilTemp => s.oil_temp_f,
            EngineParam::Cht => s.cht_f,
        }
    }
}

/// An engine instrument out of its normal range. One struct, instantiated per
/// parameter and band, mirroring the stall/overspeed pattern. `trip` is the
/// threshold to cross; `clear` (toward the more-severe side) keeps bands disjoint
/// so only one severity speaks. Thresholds are in the dataref's native unit (PSI,
/// or °C for temps); temperatures are spoken in °F to match the gauge and the POH.
pub(super) struct EngineHealth {
    pub(super) id: &'static str,
    pub(super) severity: Severity,
    pub(super) param: EngineParam,
    /// True for a low-side alarm (value below `trip`); false for high-side.
    pub(super) low_side: bool,
    pub(super) trip: f64,
    /// Boundary toward the next (more severe) band, so bands don't overlap.
    pub(super) clear: Option<f64>,
    pub(super) min_dur: Duration,
}

impl Check for EngineHealth {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        self.severity
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        // Only meaningful with the engine running and the instrument reading.
        if !s.engine_running {
            return None;
        }
        let v = self.param.value(s)?;
        let in_band = if self.low_side {
            v < self.trip && self.clear.is_none_or(|c| v >= c)
        } else {
            v > self.trip && self.clear.is_none_or(|c| v <= c)
        };
        if !in_band {
            return None;
        }
        // oil_temp_f / cht_f already read in °F; no conversion.
        let (name, unit, shown) = match self.param {
            EngineParam::OilPressure => ("Oil pressure", "pounds", v),
            EngineParam::OilTemp => ("Oil temperature", "degrees", v),
            EngineParam::Cht => ("Cylinder head temperature", "degrees", v),
        };
        let dir = if self.low_side { "low" } else { "high" };
        Some(match self.severity {
            Severity::Warning => format!("{name} {dir} — {shown:.0} {unit}. Check it now."),
            _ => format!("{name} {dir}, {shown:.0} {unit}."),
        })
    }
    fn min_duration(&self) -> Duration {
        self.min_dur
    }
}

/// Carburettor-ice advisory (predictive). The stock 172 is carburetted, and the
/// classic trap is ice forming at low power in a descent — throttle closed, moist
/// air — quietly starving the engine. Fires when conditions are ripe and carb heat
/// is off, so the copilot prompts before it bites. Advisory, not alarm.
pub(super) struct CarbIce;

impl Check for CarbIce {
    fn id(&self) -> &'static str {
        "carb_ice"
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if !s.engine_running || s.carb_heat_on || s.on_ground {
            return None;
        }
        // Ice-prone: moist air (small temp–dewpoint spread) in the −10..+27 °C band,
        // at low power (throttled back for descent/idle), where icing is worst.
        let prone_temp = (-10.0..=27.0).contains(&s.oat_c);
        let humid = (s.oat_c - s.dewpoint_c) < 13.0;
        let low_power = s.rpm > 100.0 && s.rpm < 2000.0;
        (prone_temp && humid && low_power)
            .then(|| "Conditions are right for carburettor ice — recommend carb heat.".to_string())
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(10)
    }
}

/// Low vacuum: the engine-driven pump spins the attitude and heading gyros; if
/// suction drops those instruments lie. Uses the sim's low-vacuum annunciator.
/// AIRBORNE ONLY: at ground idle the pump legitimately makes too little
/// suction and the light is normal (the POH checks vacuum at run-up RPM) —
/// cautioning about it while taxiing is noise ("pointless", the pilot said,
/// 2026-07-09). In flight the RPM is up, so a lit annunciator is a real
/// failure — critical in IMC, a cross-check reminder in VMC.
pub(super) struct LowVacuum;

impl Check for LowVacuum {
    fn id(&self) -> &'static str {
        "low_vacuum"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        (s.engine_running && s.low_vacuum && !s.on_ground).then(|| {
            "Low vacuum — the attitude and heading indicators may be unreliable. \
             Cross-check the compass and turn coordinator."
                .to_string()
        })
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(5)
    }
}

/// Alternator not charging: the switch is on but the ammeter shows the battery
/// discharging — a failed or declutched charging source the on/off check can't
/// see. Pairs with [`AlternatorOffline`] (switch off). Caution: shed load before
/// the battery dies. `noun` is the airframe's charging source ("alternator" on a
/// piston, "generator" on a turbine — a King Air has no alternator).
pub(super) struct AlternatorNotCharging {
    pub(super) noun: &'static str,
}

impl Check for AlternatorNotCharging {
    fn id(&self) -> &'static str {
        "alt_not_charging"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if !s.engine_running || !s.alternator_on {
            return None;
        }
        // A visibly producing generator overrules the battery ammeter: on
        // multi-battery airframes (MD-82) the battery bus idles at a small
        // standing discharge (~-7 A, charger cycling) with both generators
        // online, which the threshold alone reads as a failure. Verified live
        // on the Laminar MD-82: generator_amps 39/16, battery_amps -7.
        if s.generator_amps_max > 1.0 {
            return None;
        }
        (s.battery_amps < -5.0).then(|| {
            format!(
                "{noun}'s not charging — battery's draining at {amps:.0} amps. \
                 Check the {lower} and shed electrical load.",
                noun = self.noun,
                lower = self.noun.to_lowercase(),
                amps = s.battery_amps.abs()
            )
        })
    }
    /// Same start-transient sustain as [`AlternatorOffline`]: a turbine start pulls
    /// 200+ amps from the battery through the starter-generator (in *starter*
    /// mode), which the 5-second window read as a failed alternator. Only warn once
    /// the discharge persists past a normal start-to-generator-online window.
    fn min_duration(&self) -> Duration {
        ALTERNATOR_SUSTAIN
    }
}

/// Phase-expectation advisor: one profile-declared switch that should be on (or
/// off) in the listed phases — the declarative SOP table, one check per row.
/// Advisory severity: it's the copilot saying "aren't you forgetting something",
/// not an alarm. The value is read into `Snapshot::advisor_values` by index.
pub(super) struct PhaseExpectation {
    /// Leaked once per profile build — check ids must be `&'static str`.
    pub(super) id: &'static str,
    pub(super) index: usize,
    pub(super) expect_on: bool,
    pub(super) phases: Vec<Phase>,
    pub(super) say: String,
    /// When set, the copilot FIRES this command to fix the state itself rather
    /// than advising — the strobe/landing-light SOP rows the pilot wants done,
    /// not asked about.
    pub(super) command: Option<String>,
}

impl Check for PhaseExpectation {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if !self.phases.contains(&s.phase) {
            return None;
        }
        // On the ground a switch is only owed ON with an engine running: the
        // 172's "taxi light on" row lit the lamp before the start and again
        // six seconds after the shutdown at the LIPK stand (2026-09-25).
        if self.expect_on && s.phase == Phase::Ground && !s.engine_running {
            return None;
        }
        let on = (*s.advisor_values.get(self.index)?)? > 0.5;
        (on != self.expect_on).then(|| self.say.clone())
    }
    fn fix_action(&self) -> Option<&str> {
        self.command.as_deref()
    }
    /// Generous sustain: SOP items get configured *around* phase changes, so
    /// give the pilot half a minute before speaking up.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(30)
    }
}

/// Parse a profile's lowercase phase name. Unknown names are a profile bug —
/// dropped with a log line rather than a panic.
pub(super) fn parse_phase(name: &str) -> Option<Phase> {
    Some(match name {
        "ground" => Phase::Ground,
        "takeoff" => Phase::Takeoff,
        "climb" => Phase::Climb,
        "cruise" => Phase::Cruise,
        "approach" => Phase::Approach,
        "landing" => Phase::Landing,
        _ => return None,
    })
}

/// Above this the aircraft is climbing, not a level cruise with a descent
/// ahead or behind it.
pub(super) const CLIMBING_FPM: f64 = 300.0;

/// Below this height a top of descent is never called: the initial climb's
/// own height (`flight::INITIAL_CLIMB_AGL_FT`). A level-off low after a
/// departure is a pattern or a terrain level, not a cruise to leave.
pub(super) const TOD_MIN_AGL_FT: f64 = crate::flight::INITIAL_CLIMB_AGL_FT;

/// The top-of-descent family (`tod_ahead`, `tod_passed`) speaks only in the
/// CRUISE, by the flight model (`coarse_phase`: a level cruise, or a descent
/// still above 3,000 ft AGL) and clear of the ground. LSZS-LILO (SR22,
/// 2026-09-24): "we're past the descent point" at 51 ft AGL, five seconds
/// after liftoff. The plan's ring (SimBrief's along-route TOD, a winding
/// valley route) was longer than the 23.6 nm straight line from Samedan to
/// Sondrio, so the ring was already behind the aircraft on the runway; the
/// vertical-speed gate alone let any level moment in the climb-out speak.
pub(super) fn tod_in_cruise(s: &Snapshot) -> bool {
    s.phase == Phase::Cruise && s.agl_ft >= TOD_MIN_AGL_FT
}

/// The distance the plan's ring is compared with: miles still to fly along
/// the plan's legs, since the ring is along-route (#109); the straight line
/// only when no route is known. Everything measured against `tod_ring_nm`
/// reads this, never `dest_dist_nm` directly.
pub(super) fn ring_dist_nm(s: &Snapshot) -> Option<f64> {
    s.dest_route_nm.or(s.dest_dist_nm)
}

/// Descent advisor, part 1: top of descent coming up. The anticipatory version
/// of the "when is TOD" question — the copilot volunteers it while there's
/// still time to act, instead of answering after being asked. Fires once as
/// the aircraft enters the 12-nm run-up to the planned TOD, level at altitude.
/// Within this of the flight's peak the aircraft is still at its cruise.
pub(super) const CRUISE_BAND_FT: f64 = 1000.0;

/// How long the run-up's episode outlives a tick that drops it. The flight
/// model's phase flickered Cruise, Descent, Cruise at FL190 and each return
/// was a new episode: "Descent coming up" three times in 115 s (LEGR-LXGB,
/// 2026-10-06, sortie 1791275171-full_auto).
pub(super) const TOD_AHEAD_HOLD: Duration = Duration::from_secs(120);

#[derive(Default)]
pub(super) struct TodAhead {
    /// The line last true and when: the episode held through a flicker.
    pub(super) held: std::sync::Mutex<Option<(String, Instant)>>,
}

impl TodAhead {
    /// The point is behind or the descent has begun, or there is nothing
    /// left to judge: the episode really ends. Pure.
    pub(super) fn ended(s: &Snapshot) -> bool {
        if s.on_ground {
            return true;
        }
        if let Some(p) = &s.follower_tod {
            return p.started || p.to_tod_nm < 0.0;
        }
        match (ring_dist_nm(s), s.tod_ring_nm) {
            (Some(d), Some(r)) => d - r < 0.0,
            _ => true,
        }
    }

    /// [`Check::eval`] at a given instant.
    pub(super) fn eval_at(&self, s: &Snapshot, now: Instant) -> Option<String> {
        let mut held = self.held.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(line) = self.line(s) {
            *held = Some((line.clone(), now));
            return Some(line);
        }
        if let Some((line, at)) = held.as_ref() {
            if now.duration_since(*at) < TOD_AHEAD_HOLD && !Self::ended(s) {
                return Some(line.clone());
            }
        }
        *held = None;
        None
    }

    pub(super) fn line(&self, s: &Snapshot) -> Option<String> {
        // Only when level: a climb or descent has its own plan. A CLIMB is
        // never announced a top of descent (09-24: "top of descent 11 miles
        // ahead" three times at 1,000 ft climbing out of a short hop).
        if s.on_ground || s.vsi_fpm.abs() > 500.0 || s.vsi_fpm > CLIMBING_FPM {
            return None;
        }
        // A cruise, by the flight model, and clear of the ground.
        if !tod_in_cruise(s) {
            return None;
        }
        // A follower flying: ITS point, to ITS altitude (LEJR-LXGB,
        // 2026-09-14: the ring said "past", the hands waited 3 more minutes).
        if let Some(p) = &s.follower_tod {
            if p.started || !(0.0..=12.0).contains(&p.to_tod_nm) {
                return None;
            }
            // THE top of descent is the one that leaves the cruise. The
            // follower publishes a point for every stepped leg, and each was
            // called: seven "Descent coming up" down one arrival, from
            // 34,400 to 8,800 feet (LIBP-LEMG, 2026-09-26). Below the
            // cruise the steps are flown, not announced.
            if s.flight_peak_ft.is_some_and(|peak| s.alt_ft < peak - CRUISE_BAND_FT) {
                return None;
            }
            let to_tod = p.to_tod_nm;
            let alt = crate::auto_status::commas((p.target_ft / 100.0).round() as i64 * 100);
            // The pilot's altitude holds the follower: the plan's descent,
            // said once as the plan's, with who holds the level.
            if let Some(h) = p.held_ft {
                let held = crate::auto_status::commas((h / 100.0).round() as i64 * 100);
                return Some(format!(
                    "The plan starts down to {alt} feet for {}, {} ahead. You've got {held} held, give me an altitude when you want to go down.",
                    p.fix,
                    spoken_miles(to_tod)
                ));
            }
            return Some(format!(
                "Descent coming up. Top of descent {}, down to {alt} feet.",
                tod_ahead_words(to_tod, s.groundspeed_kts)
            ));
        }
        if s.follower_flying {
            return None; // the follower's point or nothing — see `follower_flying`
        }
        let (dist, elev, ring) = (ring_dist_nm(s)?, s.dest_elev_ft?, s.tod_ring_nm?);
        if s.alt_ft < elev + 6000.0 {
            return None; // already low; TOD math is meaningless
        }
        let to_tod = dist - ring;
        if !(0.0..=12.0).contains(&to_tod) {
            return None;
        }
        Some(format!(
            "Descent coming up. Top of descent {}.",
            tod_ahead_words(to_tod, s.groundspeed_kts)
        ))
    }
}

impl Check for TodAhead {
    fn id(&self) -> &'static str {
        "tod_ahead"
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        self.eval_at(s, Instant::now())
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(3)
    }
}

/// Descent advisor, part 2: past the planned TOD and still level — the exact
/// failure of the LEAL–LEMD flight (TOD slipped past unannounced; the descent
/// started late, steep, and fast). Says what rate recovers the profile, so the
/// fix is in the same breath as the problem.
/// Inside this distance to the field the profile is no rate to fly.
const STRAIGHT_IN_MIN_NM: f64 = 7.0;
/// The steepest descent worth naming as the way back onto the profile: a
/// brisk descent for anything we fly, short of an emergency one.
const BELIEVABLE_DESCENT_FPM: f64 = 3_000.0;

pub(super) struct TodPassed;

impl Check for TodPassed {
    fn id(&self) -> &'static str {
        "tod_passed"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        // Descending: moot. Climbing: "still level" is false, and on a short
        // hop the ring can be behind before the climb ends (09-24: "past the
        // descent point" at 51 ft AGL, climbing out).
        if s.on_ground || s.vsi_fpm < -400.0 || s.vsi_fpm > CLIMBING_FPM {
            return None;
        }
        // And only in the cruise, clear of the ground (`tod_in_cruise`): the
        // same flight's ring was behind it from the runway.
        if !tod_in_cruise(s) {
            return None;
        }
        // A follower flying is judged against ITS OWN point: a miss is the
        // follower a mile past it with the descent still uncommanded, never
        // the plan's ring passing while the hands are correctly waiting.
        // Nor a level the PILOT is holding it at (`held_ft`): that is a
        // choice, reminded once by `TodAhead`, not a missed descent.
        if let Some(p) = &s.follower_tod {
            if p.started || p.held_ft.is_some() || p.to_tod_nm > -1.0 || s.groundspeed_kts < 60.0 {
                return None;
            }
            let height = s.alt_ft - p.target_ft;
            let mins = (p.dist_nm - 1.0).max(2.0) / s.groundspeed_kts * 60.0;
            let fpm = ((height / mins) / 100.0).round() * 100.0;
            let alt = crate::auto_status::commas((p.target_ft / 100.0).round() as i64 * 100);
            return Some(format!(
                "We're past the descent point and still level. About {fpm:.0} feet a minute makes {alt} feet at the fix."
            ));
        }
        if s.follower_flying {
            return None; // the follower's point or nothing — see `follower_flying`
        }
        let (dist, elev, ring) = (ring_dist_nm(s)?, s.dest_elev_ft?, s.tod_ring_nm?);
        if s.alt_ft < elev + 4000.0 || dist >= ring - 1.0 || s.groundspeed_kts < 60.0 {
            return None;
        }
        // Rate that reaches ~2,000 ft over the field with 5 nm to spare.
        let height = s.alt_ft - (elev + 2000.0);
        let mins = (dist - 5.0).max(2.0) / s.groundspeed_kts * 60.0;
        let fpm = ((height / mins) / 100.0).round() * 100.0;
        // Close in the rate is nonsense ("about 14,300 feet a minute",
        // 2026-10-15): too close or too steep is no profile to rejoin.
        if dist < STRAIGHT_IN_MIN_NM || fpm > BELIEVABLE_DESCENT_FPM {
            return Some("We're past the descent point. We're too high for a straight-in.".into());
        }
        Some(format!(
            "We're past the descent point. About {fpm:.0} feet a minute makes the profile."
        ))
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(5)
    }
}

/// Descent advisor, part 3: descending but above the slope. Uses a 300 ft/nm
/// profile to a point 8 nm out (deceleration allowance) — deliberately a touch
/// above the TOD ring's slope so a normal just-started descent never trips it,
/// plus a 1,500 ft margin. High *and* fast is when the boards call gets added.
pub(super) struct HighOnProfile;

impl Check for HighOnProfile {
    fn id(&self) -> &'static str {
        "high_on_profile"
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || s.vsi_fpm > -400.0 {
            return None; // only mid-descent
        }
        // A descent from the cruise or on the arrival, clear of the ground:
        // never the takeoff, the climb or the flare (`coarse_phase`).
        let descending_phase = matches!(s.phase, Phase::Cruise | Phase::Approach);
        if !descending_phase || s.agl_ft < TOD_MIN_AGL_FT {
            return None;
        }
        // The follower's profile is its own (700 fpm to the next fix, not
        // 300 ft/nm to the field): judging it against the ring's slope would
        // call a correct descent high.
        if s.follower_tod.is_some() || s.follower_flying {
            return None;
        }
        let (dist, elev, ring) = (ring_dist_nm(s)?, s.dest_elev_ft?, s.tod_ring_nm?);
        if dist > ring + 5.0 || dist < 4.0 {
            return None; // not on the arrival yet / inside the final where GPWS owns it
        }
        let profile_alt = elev + 2000.0 + (dist - 8.0).max(0.0) * 300.0;
        let excess = s.alt_ft - profile_alt;
        if excess < 1500.0 {
            return None;
        }
        let rounded = (excess / 100.0).round() * 100.0;
        Some(if s.ias_kts > 240.0 {
            format!(
                "High on the profile, about {rounded:.0} feet above the slope, and fast. Boards would help."
            )
        } else {
            format!("High on the profile, about {rounded:.0} feet above the slope.")
        })
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(8)
    }
}

/// GPWS-style sink-rate guard (mode 1, simplified): excessive descent rate close
/// to the ground. The allowance scales with height, so a normal 700-fpm final
/// never trips it but a 2,000-fpm descent through 500 AGL does. Born of an
/// LEAL–LEMD arrival that flew a 3,400-fpm dive into terrain with no callout:
/// the threshold at 300 AGL is 1,800 fpm, which would have fired ~20 seconds out.
pub(super) struct SinkRate;

impl Check for SinkRate {
    fn id(&self) -> &'static str {
        "sink_rate"
    }
    fn severity(&self) -> Severity {
        Severity::Warning
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || s.agl_ft > 2500.0 {
            return None;
        }
        let limit = 1500.0 + s.agl_ft; // fpm of sink allowed at this height
        (s.vsi_fpm < -limit).then(|| {
            format!(
                "Sink rate! {vsi:.0} feet a minute at {agl:.0} feet. Pull up.",
                vsi = s.vsi_fpm.abs(),
                agl = s.agl_ft
            )
        })
    }
    /// React fast — two ticks at 3 Hz, enough to skip a single glitched read.
    fn min_duration(&self) -> Duration {
        Duration::from_millis(600)
    }
}

/// Altitude bust: the autopilot is flying *through* the dialed altitude instead
/// of capturing it. On airframes whose AP never arms the preselector (the
/// Laminar MD-82's dial is display-only to the sim AP), a dialed-in 5,000 simply
/// doesn't level the descent — this is the check that turns that silent
/// non-capture into a callout. Gated off in landing configuration (gear down or
/// big flap): on final the dial legitimately holds the missed-approach altitude
/// while the aircraft descends below it.
pub(super) struct AltitudeBust;

impl Check for AltitudeBust {
    fn id(&self) -> &'static str {
        "altitude_bust"
    }
    fn severity(&self) -> Severity {
        Severity::Warning
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || !s.ap_servos_on || s.gear_down || s.flap_ratio > 0.6 {
            return None;
        }
        let dial = s.ap_alt_dial_ft?;
        let diff = s.alt_ft - dial;
        let busted = (s.vsi_fpm < -400.0 && diff < -200.0) // descending through it
            || (s.vsi_fpm > 400.0 && diff > 200.0); // climbing through it
        busted.then(|| {
            format!(
                "Through the selected altitude, {dial:.0} set, passing {alt:.0}. Check altitude capture.",
                alt = s.alt_ft
            )
        })
    }
    /// A couple of seconds: enough to ride out the normal capture overshoot,
    /// short enough to matter with terrain coming up.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(3)
    }
}

/// Hypoxia guard: cabin altitude climbing toward the level where the pilot greys
/// out (the blackout you get unpressurized/without oxygen up high). Banded so the
/// caution and warning don't both fire — a caution as it passes ~10,000 ft, then a
/// warning past 12,500 (the regulatory oxygen line, and X-Plane's own annunciator
/// threshold). Airborne only; on an unpressurized airframe the cabin altitude just
/// tracks the real altitude, so this covers the GA singles too.
pub(super) fn cabin_message(pressurised: Option<bool>, warning: bool) -> &'static str {
    match (pressurised, warning) {
        (Some(false), false) => "We're unpressurised and above ten thousand feet. Check oxygen or descend when terrain permits.",
        (Some(false), true) => "Unpressurised cabin above twelve thousand five hundred. Oxygen now, or descend when terrain permits.",
        (Some(true), false) => "Cabin altitude above ten thousand. Check pressurisation and oxygen.",
        (_, true) => "Cabin altitude above twelve thousand five hundred. Oxygen now, or descend when terrain permits.",
        (None, false) => "Cabin altitude above ten thousand. Check oxygen or descend when terrain permits.",
    }
}

pub(super) struct CabinAltitude {
    pub(super) id: &'static str,
    pub(super) severity: Severity,
    /// [low, high) band, feet. `high = None` means "and above".
    pub(super) band_ft: (f64, Option<f64>),
    pub(super) message: &'static str,
    pub(super) min_dur: Duration,
    /// Above `low` as the band's hysteresis last judged it.
    pub(super) above_low: std::sync::atomic::AtomicBool,
}

/// The cabin bands' hysteresis, feet: a band is entered this far past its
/// low edge and left this far below it, and it ends this far short of its
/// high edge, so the bands stay disjoint either way. LXGB-LEJR 2026-10-09:
/// a 172 cruising AT 10,000 ft crossed the caution's edge by a few feet
/// and said "above ten thousand" and "back below" fourteen times in 16
/// minutes.
pub(super) const CABIN_BAND_HYSTERESIS_FT: f64 = 250.0;

impl Check for CabinAltitude {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        self.severity
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        use std::sync::atomic::Ordering;
        let (low, high) = self.band_ft;
        let cabin = s.cabin_altitude_ft;
        let was = self.above_low.load(Ordering::Relaxed);
        let edge = if was {
            low - CABIN_BAND_HYSTERESIS_FT
        } else {
            low + CABIN_BAND_HYSTERESIS_FT
        };
        let above = cabin >= edge;
        self.above_low.store(above, Ordering::Relaxed);
        let in_band = above && high.is_none_or(|h| cabin < h - CABIN_BAND_HYSTERESIS_FT);
        (!s.on_ground && in_band).then(|| self.message.to_string())
    }
    fn repeat_level(&self, s: &Snapshot) -> Option<i64> {
        s.cabin_altitude_ft
            .is_finite()
            .then(|| (s.cabin_altitude_ft / 1000.0).floor() as i64)
    }
    fn min_duration(&self) -> Duration {
        self.min_dur
    }
}

/// Rolling with the parking brake set — dragging the brakes on a taxi/takeoff
/// roll. Caught twice live on the C90B, where the 3D handle doesn't reflect the
/// sim-level brake (a stray 'b' key sets it invisibly). Groundspeed floor keeps
/// it quiet at a standstill and during brake-on run-ups.
pub(super) struct RollingOnParkBrake;

impl Check for RollingOnParkBrake {
    fn id(&self) -> &'static str {
        "rolling_park_brake"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        // Not when the copilot set it: the taxi-in arrests a rolling handoff
        // on the parking brake and says so itself (see `taxi_in::arrest_holding`).
        // Nor while a crew brake command is in flight: a rejected takeoff's
        // `brakes_max` reads 0.7 to 1.0 on this ratio at 60 to 90 kt and
        // decays to zero once it expires (1790524628: 1.0 at the stop, 0.4
        // two seconds later, 0.0 by four); the caution said "release it"
        // mid-reject at EGKK three times (2026-09-27). This check is for the
        // brake a PILOT forgot, which reads set with our brakes quiet.
        (s.on_ground
            && s.groundspeed_kts > 5.0
            && s.park_brake > 0.5
            && !s.crew_brake
            && !crate::taxi_in::arrest_holding())
        .then(|| "Parking brake is set and we're rolling — release it.".to_string())
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(2)
    }
}

/// Fuel selector OFF with the engine running — it's burning what's left in the
/// lines. Seen live on the SR22 after a plugin reload (the xlua re-init snaps the
/// selector to OFF mid-flight), and it's the forgotten-tank failure mode generally
/// on selector-only airframes (the SR22 has no BOTH). Warning: minutes from a
/// quiet engine.
pub(super) struct FuelSelectorOff;

impl Check for FuelSelectorOff {
    fn id(&self) -> &'static str {
        "fuel_selector_off"
    }
    fn severity(&self) -> Severity {
        Severity::Warning
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        let sel = s.fuel_selector?;
        (s.engine_running && sel < 0.5).then(|| {
            "Fuel selector is off with the engine running — select a tank now.".to_string()
        })
    }
    /// Short sustain so a selector transiting through OFF between tanks doesn't
    /// trip it, but the callout still lands well before the lines run dry.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(3)
    }
}

pub(super) const BARO_KNOB: &str = "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot";

/// The altimeter against what it should read (local QNH below the transition
/// altitude, standard above; `instruments::correct_setting_inhg`). Every
/// automation flies the INDICATED altitude (LINTO, 2026-09-14), so a wrong
/// setting is a wrong altitude: with an automation flying the copilot RESETS
/// the knob and says so; with the pilot flying it is the pilot's knob, said
/// once per episode and never re-asserted (the cockpit_auto rule). Real
/// weather walks the QNH a hundredth at a time; the tolerance and the sustain
/// keep that from becoming chatter.
///
/// Two instances: the drift check (`crossing: false`, 20 s) and the
/// transition itself (`crossing: true`, [`ALTIMETER_CROSSING_SUSTAIN_MS`]):
/// crossed in the direction flown, the setting is owed AT the transition
/// (`instruments::transition_crossed`, #156), and the drift check leaves
/// that case to it so one knob is never said twice.
pub(super) struct AltimeterSetting {
    pub(super) transition_altitude_ft: f64,
    pub(super) crossing: bool,
}

/// Under this height a landing's mute holds the altimeter checks too
/// ([`Check::acts_while_crew_lands`]): short final is no place for the knob.
pub(super) const ALTIMETER_LANDING_FLOOR_AGL_FT: f64 = 1_000.0;

/// The transition crossing's sustain: a single-sample altimeter glitch is
/// not a crossing; two seconds at 2,000 fpm is under 70 ft past it.
pub(super) const ALTIMETER_CROSSING_SUSTAIN_MS: u64 = 2_000;

impl AltimeterSetting {
    pub(super) fn wanted(&self, s: &Snapshot) -> Option<(f64, f64)> {
        let ta = crate::tools::instruments::transition_in_force_ft(self.transition_altitude_ft);
        altimeter_wanted(s, ta, self.crossing)
    }
}

/// (knob, wanted) when the altimeter should be reset against transition
/// `ta` (the one in force), for the drift check or the crossing one. Pure.
pub(super) fn altimeter_wanted(s: &Snapshot, ta: f64, crossing: bool) -> Option<(f64, f64)> {
    use crate::tools::instruments as ins;
    if !s.engine_running {
        return None;
    }
    let (baro, pa) = (s.baro_inhg?, s.qnh_pa?);
    if pa <= 0.0 {
        return None;
    }
    let crossed = ins::transition_crossed(s.alt_ft, s.vsi_fpm, ta, baro, pa);
    if crossing {
        return crossed.map(|want| (baro, want));
    }
    if crossed.is_some() {
        return None;
    }
    let want = ins::correct_setting_inhg(s.alt_ft, ta, pa);
    let tol = ins::ALTIMETER_TOLERANCE_INHG;
    let qnh = (pa / ins::PA_PER_INHG).clamp(28.0, 31.5);
    let std = ins::STANDARD_INHG;
    // Near the transition either setting is right: hovering at FL180 on a
    // FL180 transition reset the knob QNH, standard, QNH inside two
    // minutes (EDLP-LOWI, 2026-09-24). Inside the band, a knob on either
    // one is left alone; the change is made clear of it.
    if in_transition_band(s.alt_ft, ta)
        && ((baro - qnh).abs() < tol || (baro - std).abs() < tol)
    {
        return None;
    }
    // Just through it the right way, the knob's own jump can put the
    // altimeter back across (standard set at 6,000 on a 30.50 QNH reads
    // 5,440): the setting the direction flown owes is left alone within
    // the knob's jump of the transition.
    let jump_ft = (qnh - std).abs() * ins::FT_PER_INHG + 100.0;
    let near = (s.alt_ft - ta).abs() < jump_ft.max(TRANSITION_BAND_FT);
    let climbing = s.vsi_fpm >= ins::TRANSITION_CROSSING_FPM;
    let descending = s.vsi_fpm <= -ins::TRANSITION_CROSSING_FPM;
    if near
        && ((climbing && (baro - std).abs() < tol) || (descending && (baro - qnh).abs() < tol))
    {
        return None;
    }
    ((baro - want).abs() >= tol).then_some((baro, want))
}

/// Feet either side of the transition where either setting stands: more than
/// the ~260 ft the knob change itself moves the indicated altitude, which is
/// what made it flip.
pub(super) const TRANSITION_BAND_FT: f64 = 500.0;

pub(super) fn in_transition_band(alt_ft: f64, transition_ft: f64) -> bool {
    (alt_ft - transition_ft).abs() < TRANSITION_BAND_FT
}

impl Check for AltimeterSetting {
    fn id(&self) -> &'static str {
        if self.crossing {
            "altimeter_transition"
        } else {
            "altimeter_setting"
        }
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        let (baro, want) = self.wanted(s)?;
        let spoken = crate::tools::instruments::spoken_setting;
        Some(if s.automation_flying {
            format!("Altimeter reset, {}.", spoken(want))
        } else {
            let what = if (want - crate::tools::instruments::STANDARD_INHG).abs() < 0.005 {
                "standard"
            } else {
                "QNH"
            };
            format!(
                "Altimeter check, {what} is {}, you have {}.",
                spoken(want),
                spoken(baro)
            )
        })
    }
    /// The pilot turning the knob passes through wrong values on the way to
    /// the right one; a weather edition moves the QNH under a set knob. The
    /// transition itself is owed at once.
    fn min_duration(&self) -> Duration {
        if self.crossing {
            Duration::from_millis(ALTIMETER_CROSSING_SUSTAIN_MS)
        } else {
            Duration::from_secs(20)
        }
    }
    fn fix_write(&self, s: &Snapshot) -> Option<(&'static str, f64)> {
        if !s.automation_flying {
            return None;
        }
        self.wanted(s).map(|(_, want)| (BARO_KNOB, want))
    }
    /// An autoland is handed the altimeter the crew sets: the QNH through
    /// the transition level on the way down is the crew's, landing or not.
    /// Not on short final: nobody turns the knob there, and the indicated
    /// altitude a reset steps is no longer anyone's to fly.
    fn acts_while_crew_lands(&self, s: &Snapshot) -> bool {
        s.agl_ft > ALTIMETER_LANDING_FLOOR_AGL_FT
    }
}

/// A centre tank empty with its pumps still on and an engine running
/// (`tools::fuel_pumps::pumps_to_switch_off`, the one decision). The stock
/// 737's scripted start switches the CTR pumps on over an empty centre, and
/// the load sheet fills the wings first, so a short sector flew with both
/// CTR LOW PRESSURE lights lit for the whole flight (2026-09-27). The
/// altimeter's shape: with an automation flying the crew switches them off
/// (`fix_write`, through `fuel_pumps::switch_pumps`, read back); with the
/// pilot flying it is the pilot's fuel panel, said once per episode.
pub(crate) const CENTRE_TANK_DRY: &str = "centre_tank_dry";

pub(super) struct CentreTankDry {
    pub(super) declared: Vec<usize>,
}

impl CentreTankDry {
    pub(super) fn tanks(&self, s: &Snapshot) -> Vec<usize> {
        crate::tools::fuel_pumps::pumps_to_switch_off(
            &self.declared,
            s.fuel_tanks.as_ref(),
            s.engine_running,
        )
    }
}

impl Check for CentreTankDry {
    fn id(&self) -> &'static str {
        CENTRE_TANK_DRY
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if self.tanks(s).is_empty() {
            return None;
        }
        Some(if s.automation_flying {
            "Centre tank's empty, centre pumps off.".to_string()
        } else {
            "Centre tank's empty, centre pumps are still on.".to_string()
        })
    }
    /// A few seconds, so a pump the pilot is reaching for, or an engine just
    /// lit, is not a callout.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(CENTRE_DRY_SUSTAIN_S)
    }
    fn fix_write(&self, s: &Snapshot) -> Option<(&'static str, f64)> {
        if !s.automation_flying || self.tanks(s).is_empty() {
            return None;
        }
        Some((crate::tools::fuel_pumps::TANK_PUMP_ON, 0.0))
    }
}

/// Seconds an empty centre with its pumps on must hold before it is said.
pub(super) const CENTRE_DRY_SUSTAIN_S: u64 = 5;

/// The altimeter against where the aircraft IS (#94). EGSS (2026-09-22) flew
/// 4,675 ft indicated at 16,884 true; every automation flies the altimeter,
/// so only a cross-check against the true height sees the altimeter itself
/// is wrong. The knob's part is `AltimeterSetting`'s; this speaks only what
/// the knob and the air's temperature do NOT explain (a static port, a
/// lying dial), airborne, sustained. Advisory: the pilot's instruments.
pub(crate) fn altimeter_truth_of(
    s: &Snapshot,
    transition_altitude_ft: f64,
) -> Option<crate::tools::instruments::AltimeterTruth> {
    use crate::tools::instruments as ins;
    let (baro, pa, truth) = (s.baro_inhg?, s.qnh_pa?, s.alt_true_ft?);
    if pa <= 0.0 || baro <= 0.0 {
        return None;
    }
    let ta = ins::transition_in_force_ft(transition_altitude_ft);
    let reference = ins::correct_setting_inhg(s.alt_ft, ta, pa);
    Some(ins::altimeter_truth(
        s.alt_ft,
        truth,
        baro,
        pa / ins::PA_PER_INHG,
        reference,
        s.oat_c,
    ))
}

pub(super) struct AltimeterTruthCheck {
    pub(super) transition_altitude_ft: f64,
}

impl Check for AltimeterTruthCheck {
    fn id(&self) -> &'static str {
        "altimeter_disagrees"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground {
            return None;
        }
        let t = altimeter_truth_of(s, self.transition_altitude_ft)?;
        // A gross setting is the knob, said and fixed by its own check.
        if t.setting_gross() || !t.instrument_suspect() {
            return None;
        }
        let off = (t.residual_ft.abs() / 100.0).round() * 100.0;
        let way = if t.residual_ft > 0.0 { "higher" } else { "lower" };
        Some(format!(
            "Altimeter disagrees with GPS altitude, we're about {off:.0} feet {way} than it says. \
             Cross-check the standby."
        ))
    }
    /// A climb or descent rate lags the static system for a few seconds.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(30)
    }
}

/// Icing conditions and the anti-ice switches (#100,
/// `tools::anti_ice::practice_wants`, the one decision): probes, windows,
/// engines and props on in visible moisture at +10 C or colder, the wing
/// once ice accretes. The altimeter's shape: with an automation flying the
/// crew switches them (`fix_write` sentinel, through `anti_ice::monitor_fix`,
/// read back); with the pilot flying it is said once per episode.
pub(crate) const ICING: &str = "icing";

pub(super) struct Icing;

impl Check for Icing {
    fn id(&self) -> &'static str {
        ICING
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || !s.engine_running {
            return None;
        }
        let r = s.icing.as_ref()?;
        let wants = crate::tools::anti_ice::practice_wants(r);
        if wants.is_empty() {
            return None;
        }
        Some(crate::tools::anti_ice::practice_line(
            r,
            &wants,
            s.automation_flying && crate::mode::crew_actuates(),
        ))
    }
    /// A cloud passed through in seconds is not icing conditions.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(20)
    }
    fn fix_write(&self, s: &Snapshot) -> Option<(&'static str, f64)> {
        if !s.automation_flying {
            return None;
        }
        let r = s.icing.as_ref()?;
        (!crate::tools::anti_ice::practice_wants(r).is_empty())
            .then_some((crate::tools::anti_ice::MONITOR_FIX, 1.0))
    }
}

/// Seconds clear of icing before the crew switches off what it switched on.
pub(super) const ICING_CLEAR_S: u64 = 300;

/// Seconds between ice reads (a slow clock: the API is frame time).
pub(super) const ICING_READ_S: u64 = 10;

/// Air this cold or colder is worth reading the ice picture in.
pub(super) const ICING_READ_OAT_C: f64 = 12.0;

/// Cruising further below the optimum altitude than a step is worth saying.
///
/// The altitude a jet flies best at rises as fuel burns off — roughly a level
/// an hour on a narrowbody — and the plan picks ONE level before departure and
/// holds it to the destination. On a long sector that is a slow, silent fuel
/// leak: EIDW-BIKF (2026-09-20) planned FL340, and by mid-Atlantic at 62.9 t
/// the optimum was 38,600 — 4,600 ft up, with the jet stream core so flat
/// across FL300-FL390 that the climb would have cost one knot of headwind.
///
/// Advisory, and advisory ONLY. A level change belongs to ATC: the copilot
/// says what it would ask for and the pilot asks. There is no `fix_write`
/// here on purpose, and it is `Advisory` not `Caution` because nothing is
/// wrong — this is money, not safety.
pub(super) struct StepClimb {
    pub(super) reference: crate::aircraft::ReferenceProfile,
}

impl StepClimb {
    /// The level to ask for, and the optimum it is chasing. `None` unless we
    /// are actually IN level cruise on an airframe that declares an optimum:
    /// a climb or descent is already going somewhere, and advising a step in
    /// the middle of one is noise.
    pub(super) fn wanted(&self, s: &Snapshot) -> Option<(f64, f64)> {
        // A LEVEL THE PLAN HANDED DOWN IS NOT A LEVEL TO ARGUE WITH. The
        // follower publishes its own top of descent for the leg it is
        // flying; while that exists the aircraft is going DOWN shortly, on
        // the plan's own constraint, and money saved by climbing first is
        // money nobody can spend. EGLF-LFPB (2026-09-22) said "sixteen
        // thousand feet below optimum, flight level two five zero would burn
        // less" - twice - at FL230, which the plan had just brought us down
        // to from FL250, with the descent advisory already given.
        //
        // And the plan's level is the plan's on EVERY leg, not only one
        // with a descent published: `follower_tod` is `None` on a leg whose
        // constraint is not below us, so EGBB-EGKK (2026-09-27) heard the
        // advice at +1405 s with the follower holding the planned 15,000 ft
        // and its descent 9 minutes off.
        if s.follower_tod.is_some() || s.follower_flying {
            return None;
        }
        if s.phase != Phase::Cruise || s.on_ground || s.vsi_fpm.abs() > CRUISE_WANDER_FPM {
            return None;
        }
        // A climb pays back only over the cruise left to fly at the new
        // level; minutes before the top of descent it is fuel spent going
        // up to come straight down.
        if step_cruise_left_min(s).is_some_and(|m| m < STEP_CLIMB_MIN_CRUISE_MIN) {
            return None;
        }
        let optimum = crate::aircraft::optimum_altitude_ft(&self.reference, s.all_up_kg?)?;
        crate::aircraft::step_climb_ft(s.alt_ft, optimum).map(|step| (step, optimum))
    }
}

/// Whether the crew's region note ("Closest field is Benson, tower is on
/// ...") is spoken. It is VOLUNTEERED: nobody asked, and the station swaps
/// behind it either way. On the ground it is the relocation note (you loaded
/// the flight at a new field) and always said. Airborne it is worth a word
/// when something is wrong, since the closest field is then the question,
/// and otherwise only to a chatty copilot (`persona` gates what is
/// volunteered, never what is owed). EGBB-EGKK (2026-09-27) heard it four
/// times in nine minutes, Benson, Heathrow, Farnborough, Gatwick, over a
/// normal descent with the follower flying. Pure.
pub(crate) fn region_note_said(
    on_ground: bool,
    abnormal: bool,
    chatter: crate::persona::Chatter,
) -> bool {
    on_ground || abnormal || chatter == crate::persona::Chatter::Chatty
}

/// Minutes of cruise left before the top of descent under which a step climb
/// is not worth asking for. A 2,000 ft step costs a couple of minutes at climb
/// power and saves a percent or two of the cruise burn after it: over half an
/// hour that is money, over the ten minutes EGBB-EGKK had left it is not.
pub(super) const STEP_CLIMB_MIN_CRUISE_MIN: f64 = 30.0;

/// Minutes to the top of descent at this groundspeed: to the plan's ring when
/// one is published, else to 3 nm per 1,000 ft above the field. `None`
/// without a destination or a groundspeed (then nothing is gated on it).
/// Pure.
pub(super) fn step_cruise_left_min(s: &Snapshot) -> Option<f64> {
    let dist = ring_dist_nm(s)?;
    if s.groundspeed_kts <= 60.0 {
        return None;
    }
    let ring = s.tod_ring_nm.unwrap_or_else(|| {
        (s.alt_ft - s.dest_elev_ft.unwrap_or(0.0)).max(0.0) / 1000.0 * 3.0
    });
    Some((dist - ring).max(0.0) / s.groundspeed_kts * 60.0)
}

/// A level cruise under an autopilot wanders, and every crossing of this gate
/// ENDS the advisory's episode and re-arms it. At 300 fpm the 737's own
/// level hold tripped it twice in six minutes and said the same thing three
/// times (EGLF-LFPB, 2026-09-22). The phase gate above already decides what
/// counts as cruise; this only has to ignore the wobble inside it.
pub(super) const CRUISE_WANDER_FPM: f64 = 600.0;

impl Check for StepClimb {
    fn id(&self) -> &'static str {
        "step_climb"
    }
    fn severity(&self) -> Severity {
        Severity::Advisory
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        let (step, optimum) = self.wanted(s)?;
        // What the wind does up there is the whole decision, so say it when we
        // know: a step into a stronger headwind can cost more than the
        // altitude saves, and a step into the SAME wind is free money. On
        // EIDW-BIKF the jet stream was flat across the band and the climb was
        // worth one knot, which nobody had because nobody looked.
        let wind = s.winds_aloft.as_ref().and_then(|w| {
            let (here, there) = (w.at_ft(s.alt_ft)?, w.at_ft(step)?);
            let change = there.1 - here.1;
            Some(if change.abs() < 5.0 {
                " The wind up there is much the same.".to_string()
            } else if change < 0.0 {
                format!(" The wind eases {:.0} knots up there.", -change)
            } else {
                format!(" It's {:.0} knots windier up there.", change)
            })
        });
        Some(format!(
            "We're {} feet below optimum at this weight. Flight level {} would burn less if \
             you can get it.{}",
            crate::auto_status::commas(((optimum - s.alt_ft) / 100.0).round() as i64 * 100),
            crate::crew_radio::spell_runway(&format!("{:03}", (step / 100.0).round() as i64)),
            wind.unwrap_or_default(),
        ))
    }
    /// Cruise altitude wanders a little under the AP and the weight falls
    /// steadily; a long sustain keeps this to one considered word rather than
    /// a nag at the threshold.
    fn min_duration(&self) -> Duration {
        Duration::from_secs(120)
    }
    /// ...and when it does speak again it must have something NEW to say.
    /// Keyed on the level it would ask for, so burning off the weight into a
    /// higher step is worth a word and holding the same cruise is not.
    fn repeat_level(&self, s: &Snapshot) -> Option<i64> {
        self.wanted(s)
            .map(|(step, _)| (step / 1000.0).round() as i64)
    }
}

/// The alternator is normally offline for a few seconds during every engine
/// start (the engine catches mid-crank, before the ALT switch goes on), so we
/// only warn once it has stayed offline this long while running — a genuine
/// "you forgot the alternator", not the start transient.
pub(super) const ALTERNATOR_SUSTAIN: Duration = Duration::from_secs(60);

/// Engine running but the alternator switch is off → the bus is being drained
/// with nothing charging it. A classic config slip a human copilot catches.
pub(super) struct AlternatorOffline {
    pub(super) noun: &'static str,
}

impl Check for AlternatorOffline {
    fn id(&self) -> &'static str {
        "alternator_offline"
    }
    fn severity(&self) -> Severity {
        Severity::Warning
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        (s.engine_running && !s.alternator_on)
            .then(|| format!("{}'s offline — we're running on the battery.", self.noun))
    }
    fn min_duration(&self) -> Duration {
        ALTERNATOR_SUSTAIN
    }
}

/// The directional gyro has precessed away from the magnetic compass. Vacuum DGs
/// drift slowly, and on a long leg it's easy to forget to realign — but a
/// right-seat pilot doesn't nag about it, they reach over and fix it ("the
/// copilot should adjust it and say so", 2026-07-09). The monitor loop performs
/// the sync itself (the same DG_sync_mag command as the align_gyro tool) and
/// announces it with this line, once the drift READ BACK shows it took
/// (`GyroSync`). Pure so it unit-tests; rounds to 5 degrees so the figure
/// sounds like an estimate, not false precision.
pub(super) fn gyro_sync_line(drift_deg: f64) -> String {
    let approx = (drift_deg.abs() / 5.0).round() * 5.0;
    format!("Heading indicator had drifted about {approx:.0} degrees, re-synced to the compass.")
}

/// Fold a DG drift into -180..180, the same as the `align_gyro` tool, so a
/// small negative drift does not read as ~360.
pub(super) fn fold_drift_deg(raw: f64) -> f64 {
    (raw + 180.0).rem_euclid(360.0) - 180.0
}

/// Time between auto-sync attempts.
pub(super) const GYRO_SYNC_COOLDOWN: Duration = Duration::from_secs(120);
/// The sync is read back this long after the command (the sim applies a
/// command on its next frame; a beat leaves room for a slow one).
pub(super) const GYRO_VERIFY_AFTER: Duration = Duration::from_secs(1);
/// Past this with no successful read the readback gives up: unmeasured, silent,
/// no latch (a failed read is no measurement either way).
pub(super) const GYRO_VERIFY_GIVE_UP: Duration = Duration::from_secs(3);

/// Did the sync take? The drift must have at least halved: a real
/// DG_sync_mag zeroes it, and an airframe whose heading is not a vacuum DG
/// (the stock 737: IRS and glass) keeps drifting as if nothing was pressed.
pub(super) fn gyro_sync_took(before_deg: f64, after_deg: f64) -> bool {
    after_deg.abs() <= before_deg.abs() / 2.0
}

/// What a readback found.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum GyroSyncOutcome {
    /// The drift fell: say so.
    Took { before: f64, after: f64 },
    /// The command was accepted and did nothing: silent, and latched off.
    DidNotTake { before: f64, after: f64 },
    /// No read landed in time: silent, not latched.
    Unmeasured { before: f64 },
}

/// The monitor's DG auto-sync, per airframe (the loop resets it on a swap):
/// the cooldown, the readback owed after a fire, and the latch that stops an
/// airframe whose sync does nothing from being "re-synced" every 120 s.
#[derive(Debug, Default)]
pub(super) struct GyroSync {
    /// When the command was last tried (the cooldown's clock).
    pub(super) last: Option<Instant>,
    /// Drift at the fire and when, while its readback is owed.
    pub(super) pending: Option<(f64, Instant)>,
    /// The sync was read back and did nothing on this airframe.
    pub(super) ineffective: bool,
}

impl GyroSync {
    /// A drift worth syncing, off cooldown, no readback owed, not latched.
    pub(super) fn due(&self, drift_deg: f64, now: Instant) -> bool {
        !self.ineffective
            && self.pending.is_none()
            && drift_deg.abs() >= GYRO_DRIFT_ADVISORY_DEG
            && self
                .last
                .is_none_or(|t| now.duration_since(t) >= GYRO_SYNC_COOLDOWN)
    }

    /// The command was accepted: a readback is owed.
    pub(super) fn fired(&mut self, before_deg: f64, now: Instant) {
        self.pending = Some((before_deg, now));
    }

    /// Time to read the drift back.
    pub(super) fn verify_due(&self, now: Instant) -> bool {
        self.pending
            .is_some_and(|(_, at)| now.duration_since(at) >= GYRO_VERIFY_AFTER)
    }

    /// Judge a readback (`None` = the read failed). A failed read inside the
    /// give-up window keeps the readback owed and says nothing yet.
    pub(super) fn settle(&mut self, after: Option<f64>, now: Instant) -> Option<GyroSyncOutcome> {
        let (before, at) = self.pending?;
        let outcome = match after {
            Some(after) if gyro_sync_took(before, after) => GyroSyncOutcome::Took { before, after },
            Some(after) => {
                self.ineffective = true;
                GyroSyncOutcome::DidNotTake { before, after }
            }
            None if now.duration_since(at) >= GYRO_VERIFY_GIVE_UP => {
                GyroSyncOutcome::Unmeasured { before }
            }
            None => return None,
        };
        self.pending = None;
        Some(outcome)
    }
}

/// Flaps extended above their speed limit (Vfe) — the per-detent limits from the
/// profile. The applicable limit is the lowest Vfe of any detent at or below the
/// current flap position (Vfe drops as the flaps extend further). Airborne only.
pub(super) struct FlapOverspeed {
    /// (detent ratio, Vfe knots) for detents that carry a limit.
    pub(super) limits: Vec<(f64, Option<f64>)>,
}

/// The flap overspeed caution speaks this far over the placard.
pub(super) const FLAP_OVERSPEED_MARGIN_KTS: f64 = 5.0;
impl Check for FlapOverspeed {
    fn id(&self) -> &'static str {
        "flap_overspeed"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || s.flap_ratio < 0.02 {
            return None;
        }
        // Lowest Vfe among detents we've reached: the one placard lookup.
        let vfe = crate::aircraft::flap_placard_kts(s.flap_ratio, &self.limits)
            .unwrap_or(f64::INFINITY);
        if vfe.is_finite() && s.ias_kts > vfe + FLAP_OVERSPEED_MARGIN_KTS {
            Some(format!("Flap overspeed, {:.0} knots over the {vfe:.0} flap limit. Slow down or bring the flaps up.", s.ias_kts))
        } else {
            None
        }
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(2)
    }
}

/// Elevator/stabilizer trim pinned at (or essentially at) its travel limit while
/// airborne — out of trim authority, the signature of a mistrim or a runaway. Set
/// high (0.98) so normal trim, including the MD-80's nose-down Mach trim at speed,
/// doesn't trip it; only a trim that's actually hit the stop does.
pub(super) const TRIM_LIMIT: f64 = 0.98;

pub(super) struct StabTrim;
impl Check for StabTrim {
    fn id(&self) -> &'static str {
        "stab_trim_limit"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || s.elevator_trim.abs() < TRIM_LIMIT {
            return None;
        }
        let dir = if s.elevator_trim < 0.0 {
            "nose down"
        } else {
            "nose up"
        };
        Some(format!(
            "Stabilizer trim's at the {dir} limit, check the trim."
        ))
    }
    fn min_duration(&self) -> Duration {
        Duration::from_secs(5)
    }
}

/// Fuel endurance (minutes remaining at the current burn) has dropped into a
/// reserve band. Tracking *time*, not gallons, is what a real copilot does — it
/// folds in the power setting. Two instances run: a Caution band between the
/// thresholds and a Warning band below the lower one; the bands are disjoint so
/// only one speaks. Airborne + engine-running only — on the ground, idle burn
/// makes "minutes remaining" misleadingly huge (a separate pre-takeoff
/// quantity check would be the way to cover that case).
pub(super) struct FuelEndurance {
    pub(super) id: &'static str,
    pub(super) severity: Severity,
    /// Fires when endurance ≤ this many minutes …
    pub(super) at_or_below_min: f64,
    /// … and (if set) still above this — the next, more urgent band owns below it.
    pub(super) above_min: Option<f64>,
    /// The profile's cruise burn, kg/hr: the most the estimate will ever
    /// assume the aircraft keeps burning (see [`endurance_minutes`]).
    pub(super) cruise_burn_kg_hr: Option<f64>,
    /// The band has been entered this flight. The estimate rides the live
    /// flow, and a governor working the lever walks it back and forth over
    /// the band's edge: every wobble was a cleared-and-recurred episode, and
    /// the 737 heard the same "about 60 minutes" every 31 s for four minutes
    /// (EGSS, 2026-09-22). Once in, the band holds until the estimate is
    /// [`FUEL_BAND_HYSTERESIS_MIN`] clear of it.
    pub(super) entered: std::sync::atomic::AtomicBool,
}

pub(super) const FUEL_BAND_HYSTERESIS_MIN: f64 = 10.0;
/// A fuel callout is said again for each of these lost, never on a clock.
pub(super) const FUEL_REPEAT_STEP_MIN: f64 = 10.0;

/// Minutes of fuel at the burn the flight will actually average: the live
/// flow, capped at the profile's cruise burn. Takeoff and climb thrust are
/// transient by definition, and dividing the tanks by them is how the 737
/// with a correct 6,672 kg block heard "fuel's getting low, about 45
/// minutes" on every departure (a 9,000 kg/hr takeoff flow against a
/// 2,500 kg/hr cruise; LFLL, 2026-09-17), then again as the estimate swung
/// through the 60-minute band in the climb. Below the cruise burn the live
/// flow stands, optimistic side and all (a profile with no burn keeps the
/// old arithmetic). `None` when nothing is burning.
pub fn endurance_minutes(
    fuel_kg: Option<f64>,
    flow_kg_sec: Option<f64>,
    cruise_burn_kg_hr: Option<f64>,
) -> Option<f64> {
    // With a declared cruise burn the estimate is at THAT burn: steady, and
    // only ever falling. On the live flow (capped at the cruise burn) a
    // circuit's idle descents lifted it out of the band and the takeoffs
    // dropped it back in, and each re-entry said "Fuel's getting low"
    // again, five times in one circuit (LEMG, 2026-10-07).
    let kg = fuel_kg?;
    let burn = match cruise_burn_kg_hr {
        Some(c) if c > 0.0 => c / 3600.0,
        _ => flow_kg_sec?,
    };
    (burn > 1e-4).then(|| kg / burn / 60.0)
}

impl Check for FuelEndurance {
    fn id(&self) -> &'static str {
        self.id
    }
    fn severity(&self) -> Severity {
        self.severity
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if s.on_ground || !s.engine_running {
            return None;
        }
        let min = endurance_minutes(s.fuel_kg, s.fuel_flow_kg_sec, self.cruise_burn_kg_hr)
            .or(s.endurance_min)?;
        use std::sync::atomic::Ordering::Relaxed;
        let above_floor = self.above_min.is_none_or(|a| min > a);
        let in_band = min <= self.at_or_below_min && above_floor;
        let held = self.entered.load(Relaxed)
            && min <= self.at_or_below_min + FUEL_BAND_HYSTERESIS_MIN
            && above_floor;
        self.entered.store(in_band || held, Relaxed);
        if !(in_band || held) {
            return None;
        }
        // Round to the nearest 5 minutes: it's an estimate, not a stopwatch.
        let approx = (min / 5.0).round() * 5.0;
        // The first line of a band introduces it; the later ones are a crew
        // keeping count, each said once as another ten minutes goes.
        let first = min > self.at_or_below_min - FUEL_REPEAT_STEP_MIN;
        Some(match (self.severity, first) {
            (Severity::Warning, true) => {
                format!("Low fuel, about {approx:.0} minutes at this burn. Plan to land.")
            }
            (Severity::Warning, false) if approx <= 10.0 => {
                format!("About {approx:.0} minutes of fuel. We land now.")
            }
            (Severity::Warning, false) => {
                format!("Fuel is down to about {approx:.0} minutes. We need to be landing.")
            }
            (_, true) => format!("Fuel's getting low, about {approx:.0} minutes remaining."),
            (_, false) => format!("Fuel check, about {approx:.0} minutes now."),
        })
    }
    /// Each ten minutes lost is a new level, and a new level is the only
    /// thing that says a fuel line twice (see `FireState::should_speak`).
    fn repeat_level(&self, s: &Snapshot) -> Option<i64> {
        let min = endurance_minutes(s.fuel_kg, s.fuel_flow_kg_sec, self.cruise_burn_kg_hr)
            .or(s.endurance_min)?;
        Some(((self.at_or_below_min - min) / FUEL_REPEAT_STEP_MIN).floor() as i64)
    }
}

/// On the ground with the engine running and not enough fuel for a sensible
/// flight. The in-flight [`FuelEndurance`] check can't help here — idle burn
/// makes its "minutes remaining" near-infinite — so this estimates endurance
/// from a nominal *cruise* burn and cautions (once) below the reserve. Catches
/// "you're about to depart low on fuel" before you're airborne. Advisory-grade
/// Caution: a one-time nudge to go refuel, not a repeating nag.
pub(super) struct FuelPreTakeoff {
    pub(super) caution_min: f64,
    pub(super) cruise_burn_kg_hr: f64,
}

impl Check for FuelPreTakeoff {
    fn id(&self) -> &'static str {
        "fuel_pretakeoff"
    }
    fn severity(&self) -> Severity {
        Severity::Caution
    }
    fn eval(&self, s: &Snapshot) -> Option<String> {
        if !s.on_ground || !s.engine_running || self.cruise_burn_kg_hr <= 0.0 {
            return None;
        }
        let fuel = s.fuel_kg?;
        let endurance_min = fuel / self.cruise_burn_kg_hr * 60.0;
        if endurance_min >= self.caution_min {
            return None;
        }
        let approx = (endurance_min / 5.0).round() * 5.0;
        Some(format!(
            "Low fuel for departure, about {approx:.0} minutes at cruise burn. Consider refueling."
        ))
    }
}
