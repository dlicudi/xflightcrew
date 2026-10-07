//! The airframe's numbers: the lever and nose, gear warnings, speed floors and
//! ceilings, the climb and pattern, the takeoff and landing distances and the
//! waiver, steps, the stall curve, weights, flaps, and the trim and thrust
//! tables.

use super::*;

// The tests' thrust lever: one per test THREAD (a test publishing the
// Citation's lever must not move another test's); the app's is the
// snapshot's `throttle_dref`.
#[cfg(test)]
thread_local! {
    pub(super) static THROTTLE_DREF_T: std::cell::RefCell<Option<String>> = const { std::cell::RefCell::new(None) };
}

/// Publish the airframe's thrust-lever dataref alone.
#[cfg(test)]
pub fn publish_throttle_dataref(dref: Option<String>) {
    THROTTLE_DREF_T.with(|t| *t.borrow_mut() = dref);
}

/// The live profile as the lever seams read it (`tools::flaps::set_handle`,
/// `tools::controls::speedbrake_set`, `tools::gear::set_handle`): the
/// stepped-by-command declarations. Under test it is per THREAD, like the
/// throttle lever above, so one test's stepped lever never reaches another
/// test's stock writes running beside it.
pub fn lever_profile() -> Option<std::sync::Arc<AircraftProfile>> {
    #[cfg(not(test))]
    {
        live_profile()
    }
    #[cfg(test)]
    {
        LEVER_PROFILE_T.with(|t| t.borrow().clone())
    }
}

#[cfg(test)]
thread_local! {
    pub(super) static LEVER_PROFILE_T: std::cell::RefCell<Option<std::sync::Arc<AircraftProfile>>> =
        const { std::cell::RefCell::new(None) };
}

/// Publish the profile the lever seams read, on this thread alone.
#[cfg(test)]
pub fn publish_lever_profile(p: Option<AircraftProfile>) {
    LEVER_PROFILE_T.with(|t| *t.borrow_mut() = p.map(std::sync::Arc::new));
}

/// X-Plane's landing-gear warning, as Plane Maker models it: with the gear
/// not down and the speed below `below_kias`, the horn sounds when the
/// throttle is under `below_throttle_pct` or the flaps are past
/// `above_flap_ratio`. The `.acf` keys are `acf/_gear_warn_below_kias`,
/// `acf/_gear_warn_below_thro_pct`, `acf/_gear_warn_above_flap_rat`.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub struct GearWarning {
    pub below_kias: f64,
    #[serde(default)]
    pub below_throttle_pct: Option<f64>,
    #[serde(default)]
    pub above_flap_ratio: Option<f64>,
}

impl GearWarning {
    /// Would the warning sound with the gear up in this state? A flap AT
    /// the threshold counts (a handle written to 0.5 reads back a whisker
    /// either side of it). Pure.
    pub fn would_sound(&self, ias_kts: f64, flap_ratio: f64, throttle_ratio: f64) -> bool {
        if ias_kts >= self.below_kias {
            return false;
        }
        let flaps = self
            .above_flap_ratio
            .is_some_and(|f| flap_ratio >= f - 0.005);
        let power = self
            .below_throttle_pct
            .is_some_and(|t| throttle_ratio * 100.0 < t);
        flaps || power
    }
}

/// The live airframe's nose gear CASTORS (its `.acf`): it turns on the
/// ground by differential brake only, whatever the profile declares. Read
/// at detection, reset per swap (`flight_state::publish_airframe`). The
/// Evolution's profile never said `brake_steer`, and full rudder at 6 kt
/// turned it 3.7 degrees a second, off LILB's route three times
/// (2026-09-25); the RV-10's never did either.
pub fn publish_nose_castors(on: bool) {
    update_airframe_facts(|a| a.nose_castors = on);
}

pub fn nose_castors() -> bool {
    airframe_facts().nose_castors
}

/// The castoring nose has NO steering of its own either (`_steerdeg_lospeed`
/// 0: the Evolution, the SR22, the RV-10; the SF50's castors with 40 degrees
/// of it): at walking pace the brakes are the only thing that turns it.
pub fn publish_nose_free(on: bool) {
    update_airframe_facts(|a| a.nose_free = on);
}

pub fn nose_free() -> bool {
    airframe_facts().nose_free
}

/// The live airframe's taxi beta depth (`taxi.beta_max`), when it taxis in
/// beta.
pub fn taxi_beta_max() -> Option<f64> {
    live_profile()
        .and_then(|p| p.taxi.as_ref().and_then(|t| t.beta_max))
        .filter(|b| *b > 0.0)
}

/// Whether an `.acf`'s nose castors with no steering angle at all. Pure.
pub fn nose_free_from_acf(acf_text: &str) -> bool {
    let steer_lo = acf_text.lines().find_map(|l| {
        let mut it = l.split_whitespace();
        (it.next() == Some("P") && it.next() == Some("_gear/0/_steerdeg_lospeed"))
            .then(|| it.next().and_then(|v| v.parse::<f64>().ok()))
            .flatten()
    });
    nose_castors_from_acf(acf_text) && steer_lo.is_none_or(|d| d.abs() < 1.0)
}

/// Whether an `.acf`'s first gear (the nose of a tricycle) castors. Pure.
pub fn nose_castors_from_acf(acf_text: &str) -> bool {
    acf_text.lines().any(|l| {
        let mut it = l.split_whitespace();
        it.next() == Some("P")
            && it.next() == Some("_gear/0/_gear_castors")
            && it.next().and_then(|v| v.parse::<f64>().ok()).is_some_and(|v| v > 0.5)
    })
}

/// The gear-warning envelope from an `.acf`'s text, or `None` when it
/// declares no warning speed. Pure.
pub fn gear_warning_from_acf(acf_text: &str) -> Option<GearWarning> {
    let mut kias = None;
    let mut thro = None;
    let mut flap = None;
    for l in acf_text.lines() {
        let mut it = l.split_whitespace();
        if it.next() != Some("P") {
            continue;
        }
        let (Some(k), Some(v)) = (it.next(), it.next()) else {
            continue;
        };
        let v = v.parse::<f64>().ok();
        match k {
            "acf/_gear_warn_below_kias" => kias = v,
            "acf/_gear_warn_below_thro_pct" => thro = v,
            "acf/_gear_warn_above_flap_rat" => flap = v,
            _ => {}
        }
    }
    let below_kias = kias.filter(|k| *k > 0.0)?;
    Some(GearWarning {
        below_kias,
        below_throttle_pct: thro.filter(|t| *t > 0.0),
        above_flap_ratio: flap.filter(|f| *f > 0.0 && *f <= 1.0),
    })
}

/// The configuration placards autoland extends the gear and flaps against:
/// VLO/VLE, and each flap detent's `(ratio, Vfe)`. Republished on every
/// profile swap like the thrust lever, so a fast handoff on the next airframe
/// is judged by ITS limits (the snapshot's `gear_limit_kts` and
/// `flap_limits`). Empty/`None` = the profile is silent, no gate.
///
/// Each flap detent's `(ratio, Vfe)` from a profile. Pure.
pub fn flap_limits_for(flaps: Option<&FlapProfile>) -> Vec<(f64, Option<f64>)> {
    flaps
        .map(|f| f.detents.iter().map(|d| (d.ratio, d.vfe_kts)).collect())
        .unwrap_or_default()
}

/// Publish the airframe's nominal bus voltage alone (the snapshot's
/// `bus_volts_nominal`, `None` the 24 volt default).
#[cfg(test)]
pub fn publish_bus_volts(nominal: Option<f64>) {
    update_airframe_facts(|a| a.bus_volts_nominal = nominal);
}

/// Below this bus voltage the battery is too flat to reliably crank: 23 on a
/// 24 volt system (a healthy pre-start bus sits ~24-25, the low 22s after
/// accessories with the engine off), and the same 4% margin under whatever
/// the live airframe's profile says its system is.
pub fn min_start_volts() -> f64 {
    let nominal = airframe_facts().bus_volts_nominal.unwrap_or(24.0);
    nominal * 23.0 / 24.0
}

/// The airspeed FLOOR under every automation's target, from the live profile's
/// clean stall speed: no governor, autoland or follower may ask for less than
/// [`SPEED_FLOOR_RATIO`] times Vs, whatever number it was handed. The RV-10
/// was flown to 63 kt by an autoland whose approach speed was a Cessna's 65,
/// its own clean stall (2026-09-11): the target was wrong and nothing below
/// it asked whether it could be. `None` = the profile declares no Vs.
/// The snapshot's `speed_floor_kts`.
/// 1.2 Vs: below the 1.3 an approach is flown at, above anything a wing can
/// hold level with a bank on. A target under this is a wrong number.
pub const SPEED_FLOOR_RATIO: f64 = 1.2;

/// The speed floor for a clean stall speed. Pure.
pub fn speed_floor_for(vs_clean_kts: Option<f64>) -> Option<f64> {
    vs_clean_kts
        .filter(|v| *v > 0.0)
        .map(|v| (v * SPEED_FLOOR_RATIO).round())
}

/// Publish the airframe's speed floor alone.
#[cfg(test)]
pub fn publish_speed_floor(vs_clean_kts: Option<f64>) {
    let f = speed_floor_for(vs_clean_kts);
    update_airframe_facts(|a| a.speed_floor_kts = f);
}

/// The live airframe's speed floor, kts, if its profile declares a clean stall.
pub fn speed_floor_kts() -> Option<f64> {
    let f = airframe_facts();
    match weighed(&f) {
        Some((s, p)) => speed_floor_for(p.vs_clean_kts.map(|v| v * s)),
        None => f.speed_floor_kts,
    }
}

// The "too slow" ratios over the clean stall, in one place so a change to
// one is made against the others (refactor audit 2026-10-04, finding #13:
// the climb limiter's floor was moved at 1.15 by the #169 fix and sat under
// the 1.2 automation floor with the 1.13 caution between them). Their order
// is pinned by `slow_ratios_are_ordered`.

/// The monitor's stall WARNING, x the stall in the configuration flown.
pub const STALL_WARNING_RATIO: f64 = 1.10;

/// The monitor's low-speed caution with a PILOT flying, x the stall in the
/// configuration flown: the band a hand-flown approach should stay over.
pub const CAUTION_HAND_RATIO: f64 = 1.20;

/// The monitor's low-speed caution with an automation flying, x the stall in
/// the configuration flown:
/// under [`SPEED_FLOOR_RATIO`], so an automation holding its floor in a
/// gust is not called slow, and well over the stall.
pub const CAUTION_AUTO_RATIO: f64 = 1.13;

/// The climb limiter's floor, x the stall in the configuration flown
/// ([`climb_floor_kts`]): the speed a climb bleeds to before the
/// limiter has taken all of it away. It IS the automation floor: it was
/// 1.15, under [`SPEED_FLOOR_RATIO`], so a climb could bleed to a speed no
/// automation may target (CLAUDE.md "No automation target below the speed
/// floor"; refactor audit 2026-10-04, finding #13).
pub const CLIMB_FLOOR_RATIO: f64 = SPEED_FLOOR_RATIO;

// --- The climb limiter, shared by every climb a crew flies: the circuit,
// the follower and the pilot's own climb through the autopilot tool
// (moved out of `circuit`, pillars audit 2026-10-07, knot 1).

/// How far ahead the climb limiter judges the airspeed, seconds. Long enough
/// to lead the airframe's own response, short enough not to invent a speed:
/// the same order as the altitude capture's lead.
pub(super) const CLIMB_SPEED_LEAD_S: f64 = 5.0;

/// The slowest the pattern will climb toward: a floor under the speed
/// protection below. Blue line where the airframe has one, else a margin over
/// the clean stall: [`CLIMB_FLOOR_RATIO`], the automation
/// floor itself. It was 1.15 Vs, under the 1.2 Vs no automation target may
/// go below, so a climb could bleed to a speed the governor would have
/// refused as a target (refactor audit 2026-10-04, finding #13; the follower
/// already faded to `speed_floor_kts`).
///
/// A twin's Vyse is the honest number here and it is not a comfort figure:
/// below blue line a light twin cannot climb on one engine at all, and the
/// margin to Vmca is what is left of the pilot's options.
pub fn climb_floor_kts(vyse_kts: Option<f64>, vs_clean_kts: Option<f64>) -> f64 {
    let from_stall = vs_clean_kts.map(|v| v * CLIMB_FLOOR_RATIO);
    match (vyse_kts, from_stall) {
        (Some(a), Some(b)) => a.max(b),
        (Some(a), None) => a,
        (None, Some(b)) => b,
        (None, None) => 0.0,
    }
}

/// This airframe's climb-speed floor, from the profile's reference card.
///
/// Blue line where the airframe publishes one (a twin below Vyse has no
/// single-engine climb at all), else a margin over the clean stall. Both are
/// optional, and a profile carrying neither gets 0, which disables the
/// protection rather than inventing a number.
pub fn climb_floor(profile: &AircraftProfile) -> f64 {
    climb_floor_kts(
        profile.reference.as_ref().and_then(|r| r.vyse_kts),
        profile.vs_clean_kts,
    )
}

/// How much of a commanded CLIMB the airspeed will actually support. Scales
/// the rate to zero as the speed falls from the pattern speed to the floor.
///
/// The autopilot's vertical mode buys rate with airspeed, and it will keep
/// buying: a Baron 58 rolled into the crosswind turn at 127 kt and came out at
/// 85 (one knot over its clean stall and one over Vmca) with the VS law
/// still asking for 880 fpm and the stabilizer trimmed to the nose-up stop.
/// Six airspeed callouts fired across one Southampton circuit and every one of
/// them was right (2026-08-01). The throttle law alone cannot fix this: it is
/// already commanding more power, and in a climbing turn on a piston twin
/// there is none left to give. Lowering the nose is the other half, and it is
/// the half a pilot would do first.
///
/// Only ever REDUCES a climb: descents (negative) pass through untouched, and
/// an aircraft at or above its full-rate speed (see [`climb_full_rate_kts`])
/// keeps the rate it asked for, so every airframe with power to spare is
/// unchanged.
pub fn climb_limit_fpm(
    vs_cmd: f64,
    ias: f64,
    ias_trend_kts_per_s: f64,
    want_kts: f64,
    floor_kts: f64,
) -> f64 {
    if vs_cmd <= 0.0 || !ias.is_finite() || floor_kts <= 0.0 {
        return vs_cmd;
    }
    // Judge the speed we are HEADING FOR, not the one on the dial. Reacting to
    // the instantaneous reading made this a bang-bang gate and it limit-cycled
    // live: the commanded rate went 700 to 0 in four seconds as the speed dipped
    // through the band, HELD there while the aircraft was already accelerating
    // back, then slammed open again, the aeroplane porpoising about thirty feet
    // either side with a period near twenty-five seconds (2026-08-01, felt in the
    // cockpit before it was seen in the log). The projection fixes both ends: it
    // starts easing the climb off before the speed is lost, so the dip is
    // shallower, and it gives the rate back as soon as the speed is genuinely
    // recovering instead of waiting for it to arrive. Same lead trick
    // `alt_capture_vs` uses on altitude and the stall callout uses on airspeed.
    let ias = ias + ias_trend_kts_per_s * CLIMB_SPEED_LEAD_S;
    // Full rate AT the pattern speed: that is the speed the airframe is meant
    // to be climbing at, and an aircraft flying it correctly must never be
    // penalised. The fade runs downward from there to the floor.
    if ias >= want_kts {
        return vs_cmd;
    }
    if ias <= floor_kts {
        return 0.0;
    }
    // A pattern speed at or below the floor leaves nothing to fade across
    // (the two checks above have already covered it).
    vs_cmd * ((ias - floor_kts) / (want_kts - floor_kts))
}

/// The speed at which [`climb_limit_fpm`] grants the FULL commanded rate: the
/// speed the climb is actually FLOWN at, never the leg target the throttle
/// holds.
///
/// Those were the same number until the clean-stall floor raised the Baron's
/// pattern speed to 109.2 kt (four knots ABOVE its Vy of 105) and the two
/// changes shipped the same evening without meeting. The throttle law holds
/// the leg speed, so wiring that same speed in as the limiter's full-rate
/// threshold parks the aircraft at the exact top of a nine-knot fade band:
/// full climb is unreachable WHILE climbing (climbing at Vy is what the
/// airframe does), and every knot of normal jitter swings the granted rate
/// across half its range. Live it porpoised exactly as the projection was
/// supposed to prevent: cmd_vs 700 to 0 on crosswind with the Baron
/// DESCENDING at 190 fpm at 670 ft AGL mid-climb-out, then the whole downwind
/// hung 100 ft below pattern altitude with the capture choked to 0-150 fpm
/// and the speed see-sawing 104 to 113 on a 25 s cycle (EGHI, 2026-08-02).
/// The unit test had pinned Vy as `want` all along; the call site passed the
/// pattern speed, and the four-knot difference was the whole failure.
///
/// Capped below the leg speed (a jet's book Vy is a cruise-climb figure far
/// above its pattern speed, and must not RAISE the threshold), and held a
/// margin above the floor so a profile whose Vy sits at or under the floor
/// still leaves a band to fade across instead of a cliff.
pub fn climb_full_rate_kts(leg_kts: f64, vy_kts: f64, floor_kts: f64) -> f64 {
    leg_kts.min(vy_kts).max(floor_kts + 2.0)
}

/// The least pattern speed, x clean Vs ([`PATTERN_MIN_STALL_MULT`]),
/// and the speed an upset recovery holds ([`recovery_hold_kts`]).
pub const PATTERN_MIN_RATIO: f64 = 1.3;

/// Ceiling on the pattern speed, as a multiple of the approach speed.
pub(super) const PATTERN_MAX_APPROACH_MULT: f64 = 1.5;

/// The speed the pattern legs are flown at: the airframe's climb speed, but
/// never more than [`PATTERN_MAX_APPROACH_MULT`] times its approach speed.
///
/// Vy is not a pattern speed on a jet. The Citation X's published best-rate
/// figure is 300 KIAS (a cruise-climb number) and flown as a circuit speed it
/// would ask for a turn radius of about ten kilometres from geometry that
/// clamps at 2.5. The MD-82's 200 kt was the same mistake in a milder form.
/// A pattern at 1.5x the approach speed is what these aircraft actually fly,
/// and for every GA profile Vy already sits below that, so nothing changes.
pub fn pattern_speed_kts(vy_kts: f64, approach_ias: f64, vs_clean_kts: Option<f64>) -> f64 {
    let base = vy_kts
        .max(60.0)
        .min((approach_ias * PATTERN_MAX_APPROACH_MULT).max(60.0));
    // ...and never near the CLEAN stall, because the pattern is flown clean.
    // The Baron's Vy is 105 against an 84 kt clean stall: 1.25 Vs, which is
    // four knots above the airspeed caution band (1.2 Vs = 100.8) and so every
    // normal variation on downwind, base and final dipped into it. Six
    // callouts in one circuit, and every one of them was correct: the aeroplane
    // really was that close to the band. A flapped approach speed is not a
    // clean pattern speed, and nothing here knew the difference.
    let floor = vs_clean_kts.map_or(0.0, |v| v * PATTERN_MIN_STALL_MULT);
    base.max(floor)
}

/// The clean pattern is flown no slower than this multiple of the CLEAN stall.
/// 1.3 Vs is the usual approach minimum, and it is applied to the clean figure
/// here because the circuit never lowers flap before the handoff. Only bites on
/// an airframe whose Vy sits close to its clean stall: a 172 (Vy 74, stall 48)
/// and an MD-82 (Vy 200, stall 100) are both untouched.
pub(super) const PATTERN_MIN_STALL_MULT: f64 = PATTERN_MIN_RATIO;

/// Default pattern altitude above the field, feet (a standard light-aircraft
/// circuit height). Not yet pilot-selectable by voice; a constant for now.
pub(super) const DEFAULT_TPA_AGL_FT: f64 = 1000.0;
/// ...and the one a jet flies, which is the real-world figure and not
/// decoration. The pattern feeds autoland, and the wider a pattern is the
/// farther out it turns final: the MD-82's base leg reaches the centreline
/// around 4 nm, where a 3° path is 1,340 ft up. Handing over from a 1,000 ft
/// circuit there means arriving low and having to make it up, which is exactly
/// what went wrong at Luton (2026-07-28).
pub(super) const JET_TPA_AGL_FT: f64 = 1500.0;
/// Approach speed at or above which the pattern is a jet's, not a trainer's.
pub(super) const JET_APPROACH_KTS: f64 = 110.0;

/// The pattern altitude this airframe should fly, from the only figure that
/// distinguishes them here: how fast it flies its approach.
pub fn pattern_alt_agl_ft(approach_ias: f64) -> f64 {
    if approach_ias >= JET_APPROACH_KTS {
        JET_TPA_AGL_FT
    } else {
        DEFAULT_TPA_AGL_FT
    }
}

/// The approach speed an airframe that declares none flies, x Vso.
pub const APPROACH_VSO_RATIO: f64 = 1.3;

/// The same, x clean Vs: the automation floor, so the fallback approach is
/// never under it.
pub const APPROACH_VS_RATIO: f64 = SPEED_FLOOR_RATIO;

/// The approach speed an airframe that declares none flies: max(1.3 Vso,
/// 1.2 Vs). ONE rule for the tool registry (autoland), `generic_from_acf`,
/// the instructor's brief and scorer, and the tests that build airframes
/// (refactor audit 2026-10-04, finding #2: the instructor still briefed 1.3
/// x the CLEAN stall, the rule the registry dropped after the SR22 floated
/// and bounced at 90 kt, LEMG 13 2026-09-29).
///
/// No flat 70 kt floor, deliberately. The registry had one and
/// `generic_from_acf` did not, so the two paths disagreed; every shipped
/// profile declares `approach_kts` and a generic airframe always carried the
/// unfloored figure, so the floor only ever reached an airframe with a
/// declared stall and no approach speed, and there it is the SR22 float by
/// another door (a 40 kt-stall wing flown at 70). The 1.2 Vs term already
/// keeps it over the automation floor. Pure.
pub fn fallback_approach_kts(vs0_kts: Option<f64>, vs_clean_kts: f64) -> f64 {
    let by_vso = vs0_kts
        .filter(|v| v.is_finite() && *v > 0.0)
        .map_or(0.0, |v| v * APPROACH_VSO_RATIO);
    by_vso.max(vs_clean_kts * APPROACH_VS_RATIO).round()
}

/// The profile's approach speed, else [`fallback_approach_kts`] from its
/// stalls; `None` when it declares neither an approach speed nor a clean
/// stall.
pub fn approach_kts(p: &AircraftProfile) -> Option<f64> {
    let r = p.reference.as_ref();
    r.and_then(|r| r.approach_kts).or_else(|| {
        p.vs_clean_kts
            .filter(|v| v.is_finite() && *v > 0.0)
            .map(|vs| fallback_approach_kts(r.and_then(|r| r.vs0_kts), vs))
    })
}

/// Rotation speed for an airframe that declares none, x clean Vs.
///
/// 1.1, not the registry's old 1.15 (min 55). The shipped profiles that
/// declare a Vr sit at 0.85 to 1.15 of their clean stall (SR22 1.04, 737
/// 1.04, SF50 1.13, 172 1.15, A330 0.85): takeoff flap lowers the stall, so
/// 1.1 x the clean figure is still over 1.2 x the takeoff stall on every one
/// of them, and 1.15 rotated late and long. A generic airframe already
/// rotated at 1.1 (`generic_from_acf`, whose figure the registry reads);
/// now an airframe with a profile and no Vr agrees with it (refactor audit
/// 2026-10-04, finding #2).
pub const VR_STALL_RATIO: f64 = 1.1;

/// [`VR_STALL_RATIO`] x the clean stall, rounded. Pure.
pub fn fallback_vr_kts(vs_clean_kts: f64) -> f64 {
    (vs_clean_kts * VR_STALL_RATIO).round()
}

/// The speed an upset recovery holds: [`PATTERN_MIN_RATIO`] x the clean
/// stall, never under 70 kt. The floor stays here though not on the
/// approach: a recovery wants margin over an unknown or optimistic stall
/// and has no runway to float down, and slow is the one way it can still
/// go wrong. Pure.
pub fn recovery_hold_kts(vs_clean_kts: f64) -> f64 {
    (vs_clean_kts * PATTERN_MIN_RATIO).max(RECOVERY_HOLD_MIN_KTS)
}

/// The least speed [`recovery_hold_kts`] asks for.
pub const RECOVERY_HOLD_MIN_KTS: f64 = 70.0;

/// The highest speed the flight governor may be asked to hold, from the
/// airframe's Vne/Vmo. `None` = the profile declares no Vne.
///
/// The governor's sanity cap was a flat 250 kt, which is a GA number: the
/// A330's cruise governor was armed at its profile's 290 and held 250 with
/// nothing said, and a spoken "hold two eighty" in any jet was quietly 250
/// (found on the profile review, 2026-09-20). The cap is now the airframe's
/// own, [`ceiling_from_vne`], republished per swap like the floor. Below 10,000 ft the 250 kt rule still applies on top of it
/// (`speedhold::BELOW_10K_LIMIT_KTS`); that is airspace, not airframe.
///
/// The live airframe's declared speed limits (the snapshot's
/// `speed_limits`), the ceilings' one source.
#[derive(Debug, Clone, Copy)]
pub struct SpeedLimits {
    pub(super) vne_kts: Option<f64>,
    pub(super) vmo_high: Option<VmoHigh>,
}

impl SpeedLimits {
    /// From a profile's Vne and high-altitude Vmo, the insane ones dropped.
    /// Pure.
    pub fn new(vne_kts: Option<f64>, vmo_high: Option<VmoHigh>) -> Self {
        SpeedLimits {
            vne_kts: vne_kts.filter(|v| *v > 0.0),
            vmo_high: vmo_high.and_then(VmoHigh::valid),
        }
    }
}
/// The monitor's overspeed caution floor, a fraction of Vne (the monitor's
/// `OVERSPEED_CAUTION_FACTOR` is this one).
pub const OVERSPEED_CAUTION_FACTOR: f64 = 0.90;

/// Knots under the monitor's overspeed caution the ceiling sits: the
/// governor's own wander (296 to 303 kt around a 299 target at FL180) and a
/// descent's first seconds must not reach the caution band.
pub const SPEED_CEILING_MARGIN_KTS: f64 = 10.0;

/// The speed ceiling for a Vne: the overspeed caution's floor
/// ([`OVERSPEED_CAUTION_FACTOR`] of Vne) less [`SPEED_CEILING_MARGIN_KTS`]. It was Vne less 15, which sits INSIDE the
/// caution band for any Vne over 150 kt: a 737 (Vne 340, caution from 306)
/// accepted "hold three twenty" and then called "Overspeed!" on it (refactor
/// audit, 2026-10-04). One rule for the governor and the follower's cruise
/// (`procedure::cruise_cap_kts`). Pure.
pub fn ceiling_from_vne(vne_kts: f64) -> f64 {
    (vne_kts * OVERSPEED_CAUTION_FACTOR - SPEED_CEILING_MARGIN_KTS).floor()
}

/// Publish the airframe's speed ceiling from its Vne and its high-altitude
/// Vmo alone.
#[cfg(test)]
pub fn publish_speed_ceiling(vne_kts: Option<f64>, vmo_high: Option<VmoHigh>) {
    let l = SpeedLimits::new(vne_kts, vmo_high);
    update_airframe_facts(|a| a.speed_limits = l);
}

/// The live airframe's speed ceiling, kts, if its profile declares a Vne:
/// the LOW one on an airframe with a `vmo_high:`, for a caller without an
/// altitude (conservative). With one in hand, [`speed_ceiling_kts_at`] or
/// [`VmoCeiling`].
pub fn speed_ceiling_kts() -> Option<f64> {
    airframe_facts().speed_limits.vne_kts.map(ceiling_from_vne)
}

/// A Vmo that applies at and above an INDICATED altitude (the profile's
/// `vmo_high:`); below it the profile's `vne_kts` is the limit.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct VmoHigh {
    /// Indicated altitude, feet, at and above which `kts` is the limit.
    pub above_ft: f64,
    /// The limit up there, KIAS.
    pub kts: f64,
    /// Mmo, when declared: up high the limit is the LOWER of `kts` and the
    /// airspeed this Mach is at the altitude ([`cas_for_mach`]).
    /// The Citation X's 350 is M0.935 by FL290 and M0.94 is 271 KIAS at its
    /// FL430 optimum: a raised Vmo without its Mmo would cruise past Mach.
    #[serde(default)]
    pub mmo: Option<f64>,
}

impl VmoHigh {
    /// Used only when the numbers are sane ones (an insane Mmo is dropped).
    pub fn valid(self) -> Option<Self> {
        (self.above_ft.is_finite() && self.above_ft > 0.0 && self.kts.is_finite() && self.kts > 0.0)
            .then_some(VmoHigh {
                mmo: self.mmo.filter(|m| m.is_finite() && (0.3..1.5).contains(m)),
                ..self
            })
    }

    /// The limit at an indicated altitude in the high band, KIAS: `kts`,
    /// or the Mmo's airspeed there when that is lower.
    pub(super) fn kts_at(self, alt_ft: f64) -> f64 {
        self.mmo
            .map_or(self.kts, |m| self.kts.min(cas_for_mach(m, alt_ft)))
    }
}

/// Calibrated airspeed, kt, of `mach` at a pressure altitude (ISA,
/// troposphere and the isothermal layer above 36,089 ft). The governor holds
/// INDICATED speed, so a planned Mach is held as the airspeed it is at the
/// level flown: M0.628 at FL180 is 300 kt, M0.78 at FL360 is 260. Pure.
pub fn cas_for_mach(mach: f64, pressure_alt_ft: f64) -> f64 {
    const A0_KTS: f64 = 661.4786;
    let h = pressure_alt_ft.clamp(-2_000.0, 65_000.0);
    let delta = if h <= 36_089.0 {
        (1.0 - 6.875_59e-6 * h).powf(5.255_88)
    } else {
        0.223_36 * (-4.806_34e-5 * (h - 36_089.0)).exp()
    };
    let qc_over_p = (1.0 + 0.2 * mach * mach).powf(3.5) - 1.0;
    let qc_over_p0 = qc_over_p * delta;
    A0_KTS * (5.0 * ((qc_over_p0 + 1.0).powf(2.0 / 7.0) - 1.0)).sqrt()
}

/// The airframe's Vne/Vmo at an INDICATED altitude: `vmo_high`'s figure at
/// and above its altitude (its Mmo's airspeed where lower), `vne_kts` below
/// it, and `vne_kts` too for an altitude not read (the low limit is the
/// conservative one). `None` = no limit declared there. Pure.
pub fn vne_at(vne_kts: Option<f64>, vmo_high: Option<VmoHigh>, alt_ft: f64) -> Option<f64> {
    match vmo_high.and_then(VmoHigh::valid) {
        Some(h) if alt_ft.is_finite() && alt_ft >= h.above_ft => Some(h.kts_at(alt_ft)),
        _ => vne_kts.filter(|v| *v > 0.0),
    }
}

impl AircraftProfile {
    /// This airframe's Vne/Vmo at an indicated altitude ([`vne_at`]).
    #[cfg(test)]
    pub fn vne_at(&self, alt_ft: f64) -> Option<f64> {
        vne_at(self.vne_kts, self.vmo_high, alt_ft)
    }
}

/// The live airframe's Vne/Vmo at an indicated altitude ([`vne_at`] over
/// the published limits).
pub fn published_vne_at(alt_ft: f64) -> Option<f64> {
    let l = airframe_facts().speed_limits;
    vne_at(l.vne_kts, l.vmo_high, alt_ft)
}

/// The live airframe's high-altitude Vmo, when its profile declares a sane one.
pub fn published_vmo_high() -> Option<VmoHigh> {
    airframe_facts().speed_limits.vmo_high.and_then(VmoHigh::valid)
}

/// The live airframe's speed ceiling at an indicated altitude: the clean
/// threshold, for a judgement made once (a spoken number). A loop that
/// holds a speed through a descent takes [`VmoCeiling`], which slows
/// BEFORE the limit drops.
pub fn speed_ceiling_kts_at(alt_ft: f64) -> Option<f64> {
    published_vne_at(alt_ft).map(ceiling_from_vne)
}

/// The highest speed ceiling the live airframe has at ANY altitude: the
/// sanity bound on a target armed without an altitude, which the governor
/// then holds under the ceiling where it flies. Equal to
/// [`speed_ceiling_kts`] without a `vmo_high:`.
pub fn speed_ceiling_kts_max() -> Option<f64> {
    let high = published_vmo_high().map(|h| ceiling_from_vne(h.kts));
    match (speed_ceiling_kts(), high) {
        (Some(a), Some(b)) => Some(a.max(b)),
        (a, b) => a.or(b),
    }
}

/// How far ahead a descent is projected, seconds, for the ceiling it will
/// meet: the low Vmo is flown BEFORE crossing its altitude, not after. A
/// descent at idle bleeds about a knot a second with the pitch shallowed to
/// slow, and 305 to 233 is 72 kt.
pub const VMO_DESCENT_LEAD_S: f64 = 90.0;

/// The climb, fpm, that lifts a held low ceiling once over the altitude.
pub const VMO_RELEASE_CLIMB_FPM: f64 = 300.0;

/// The speed ceiling with memory, for a loop that holds a speed every tick
/// on an airframe with a `vmo_high:`. Low while the descent projected
/// [`VMO_DESCENT_LEAD_S`] ahead is under the altitude; once low it HOLDS
/// (the pitch shallowing to slow would otherwise lift the projection, the
/// ceiling and the speed again, round and round) until the aircraft climbs
/// over the altitude. A level held over it after a descent stays slow,
/// which is legal. Without a `vmo_high:` it is [`speed_ceiling_kts`]
/// exactly, and with nothing measured the low ceiling stands.
#[derive(Debug, Default, Clone, Copy)]
pub struct VmoCeiling {
    /// `None` until the first finite altitude.
    pub(super) low: Option<bool>,
    /// The last finite altitude, for the Mmo's airspeed on a lost read.
    pub(super) alt_ft: Option<f64>,
}

impl VmoCeiling {
    /// Advance on the live airframe's published limits.
    pub fn ceiling(&mut self, alt_ft: f64, vs_fpm: f64) -> Option<f64> {
        let l = airframe_facts().speed_limits;
        self.ceiling_for(l.vne_kts, l.vmo_high, alt_ft, vs_fpm)
    }

    /// Advance on indicated `alt_ft` and `vs_fpm` (a vertical speed not read
    /// is level) and return the ceiling this tick. A failed altitude read
    /// keeps what was known. Pure.
    pub fn ceiling_for(
        &mut self,
        vne_kts: Option<f64>,
        vmo_high: Option<VmoHigh>,
        alt_ft: f64,
        vs_fpm: f64,
    ) -> Option<f64> {
        let low_ceiling = vne_kts.filter(|v| *v > 0.0).map(ceiling_from_vne);
        let Some(h) = vmo_high.and_then(VmoHigh::valid) else {
            return low_ceiling;
        };
        if alt_ft.is_finite() {
            let vs = if vs_fpm.is_finite() { vs_fpm } else { 0.0 };
            let ahead = alt_ft + vs.min(0.0) * VMO_DESCENT_LEAD_S / 60.0;
            let low = match self.low {
                Some(true) => !(alt_ft >= h.above_ft && vs > VMO_RELEASE_CLIMB_FPM),
                _ => ahead < h.above_ft,
            };
            self.low = Some(low);
            self.alt_ft = Some(alt_ft);
        }
        match (self.low, self.alt_ft) {
            (Some(false), Some(alt)) => Some(ceiling_from_vne(h.kts_at(alt.max(h.above_ft)))),
            _ => low_ceiling,
        }
    }

    /// Whether the low ceiling holds (`None` before any measurement).
    #[cfg(test)]
    pub fn low(&self) -> Option<bool> {
        self.low
    }
}

/// The ground roll to check a runway against, scaled by ACTUAL weight.
///
/// `roll_at_mtow_m` is the profile's figure, which is a MAX-TAKEOFF-WEIGHT
/// number; applying it at every weight is why an A330 on a light sector was
/// refused Edinburgh's 2,549 m runway by eleven metres — and refused from
/// every position on it, having been sent to line up and then to backtrack
/// (2026-09-20). Real A330s operate there.
///
/// LINEAR in weight, and that choice is the safety argument. Ground roll
/// physically scales nearer W², but measured against the published field
/// lengths (about 2,500 m at 242 t, about 1,800 m at 180 t) the real
/// exponent is about 1.1. Linear therefore asks for slightly MORE runway
/// than the aircraft needs, and the square law would ask for far less —
/// 1,416 m against a true 1,900 at that weight. When a margin has to be
/// wrong, it is wrong upward.
///
/// Clamped to half the MTOW roll at the bottom: nothing here models density
/// altitude, contamination or a rejected takeoff, and the factor the caller
/// applies is not a substitute for a performance calculation. Unknown weight
/// or unknown MTOW keeps the profile's figure exactly as before.
pub fn takeoff_roll_m(roll_at_mtow_m: f64, weight_kg: Option<f64>, mtow_kg: Option<f64>) -> f64 {
    let (Some(w), Some(mtow)) = (weight_kg, mtow_kg) else {
        return roll_at_mtow_m;
    };
    if !w.is_finite() || !mtow.is_finite() || w <= 0.0 || mtow <= 0.0 {
        return roll_at_mtow_m;
    }
    // Never LESS demanding than half, and never less demanding than the
    // profile figure would be above max weight.
    roll_at_mtow_m * (w / mtow).clamp(0.5, 1.0)
}

/// The ground roll a profile checks runways against, metres, and the weight
/// it is quoted at, kg. Prefer the declared GROUND ROLL; the recited takeoff
/// distance is the fallback the GA profiles have always used (on an airliner
/// that figure is a factored field length, see
/// `ReferenceProfile::takeoff_roll_ft`). One derivation for the tool registry
/// and the spoken "do we have enough runway".
pub fn profile_takeoff_roll(profile: &AircraftProfile) -> (Option<f64>, Option<f64>) {
    let r = profile.reference.as_ref();
    (
        r.and_then(|r| r.takeoff_roll_ft.or(r.takeoff_distance_ft))
            .map(|f| f * 0.3048),
        r.and_then(|r| r.max_gross_lb).map(|lb| lb * 0.453_592),
    )
}

/// Pavement the takeoff needs, metres: the weight-scaled ground roll with a
/// 1.4 factor (the book figure is a test pilot's on a dry day), never under
/// `floor_m`. No roll on file = the floor.
///
/// ONE rule for the line-up's full-length check, full auto's "are we lined
/// up" fit, the circuit's and the takeoff's runway fit and the arm-time field
/// check. It was five copies, and the weight scaling reached one of them: the
/// spoken takeoff accepted Edinburgh 24 while full auto, asking the same
/// question of the same aircraft on the same centreline, gave up with "I
/// couldn't get us onto the runway" (2026-09-20, the tower logging
/// `lined_up=true` throughout).
pub fn takeoff_needed_m(
    roll_at_mtow_m: Option<f64>,
    weight_kg: Option<f64>,
    mtow_kg: Option<f64>,
    floor_m: f64,
) -> f64 {
    takeoff_needed_factored_m(roll_at_mtow_m, weight_kg, mtow_kg, floor_m, takeoff_factor())
}

/// The margin on the book roll every runway gate applies.
pub const TAKEOFF_FACTOR: f64 = 1.4;

/// The pilot waived the runway margin for the full-auto mission flying the
/// plan it was waived for. The mission owns it (`full_auto::forget_mission`
/// and its arm are the only writers); it lives here only because the lib's
/// runway rule reads it.
pub(super) static TAKEOFF_MARGIN_WAIVED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Full auto's writer for its runway waiver; nobody else sets it.
pub fn set_takeoff_margin_waived(on: bool) {
    TAKEOFF_MARGIN_WAIVED.store(on, std::sync::atomic::Ordering::SeqCst);
}

/// Whether a waived full-auto mission is flying.
pub fn takeoff_margin_waived() -> bool {
    TAKEOFF_MARGIN_WAIVED.load(std::sync::atomic::Ordering::SeqCst)
}

/// [`TAKEOFF_FACTOR`], or none while a waived full-auto mission flies.
pub(super) fn takeoff_factor() -> f64 {
    if takeoff_margin_waived() {
        1.0
    } else {
        TAKEOFF_FACTOR
    }
}

/// [`takeoff_needed_m`] with the factor given (the preflight asks both
/// ways to say what a waiver gives up).
pub fn takeoff_needed_factored_m(
    roll_at_mtow_m: Option<f64>,
    weight_kg: Option<f64>,
    mtow_kg: Option<f64>,
    floor_m: f64,
    factor: f64,
) -> f64 {
    roll_at_mtow_m.map_or(floor_m, |m| {
        (takeoff_roll_m(m, weight_kg, mtow_kg) * factor).max(floor_m)
    })
}

/// [`takeoff_needed_m`] at what the aircraft weighs NOW. A failed weight read
/// keeps the max-weight figure, which is the conservative end.
pub async fn takeoff_needed_now_m(
    xp: &dyn sim::xplane::XPlane,
    roll_at_mtow_m: Option<f64>,
    mtow_kg: Option<f64>,
    floor_m: f64,
) -> f64 {
    let weight_now = xp
        .get_float(sim::datarefs::TOTAL_WEIGHT_KG)
        .await
        .ok()
        .filter(|w| w.is_finite() && *w > 0.0);
    takeoff_needed_m(roll_at_mtow_m, weight_now, mtow_kg, floor_m)
}

/// The margin on the book LANDING distance every landing-runway gate
/// applies (see [`landing_needed_m`]): none.
pub const LANDING_FACTOR: f64 = 1.0;

/// Pavement a landing needs, metres: the book landing distance, else the
/// book takeoff distance (what every GA POH gives, and the longer of the
/// two there), times [`LANDING_FACTOR`], never under `floor_m`. No figure on
/// file = the floor.
///
/// ONE rule beside [`takeoff_needed_m`], for the planner's destination
/// refusal, full auto's preflight, the plan audit and autoland's self-pick
/// (refactor audit 2026-10-04, finding #10). They disagreed: the planned
/// path took a runway equal to the bare book figure while the emergency
/// path demanded 1.3 times it, so a destination the preflight passed could
/// be one the self-pick would refuse on a go-around. The BOOK figure is the
/// one kept: the jets' POM landing distances are already factored field
/// lengths (the 737's 5,360 ft), so another 1.3 refused a 737 into LXGB's
/// 5,449 ft, which it serves daily, and an emergency pick must never ask
/// more of a runway than the plan did. The floor is the caller's: the
/// self-pick keeps its 1,200 m (it chooses among strips nobody planned), a
/// plan has none (a 172's 530 m grass field is a destination). Pure.
pub fn landing_needed_m(landing_m: Option<f64>, takeoff_m: Option<f64>, floor_m: f64) -> f64 {
    let book = landing_m
        .filter(|d| d.is_finite() && *d > 0.0)
        .or_else(|| takeoff_m.filter(|d| d.is_finite() && *d > 0.0));
    book.map_or(floor_m, |d| (d * LANDING_FACTOR).max(floor_m))
}

/// [`landing_needed_m`] for a PLAN, in feet from the profile's feet: no
/// floor, and `None` (no check) when the profile recites neither figure.
pub fn planned_landing_needed_ft(landing_ft: Option<f64>, takeoff_ft: Option<f64>) -> Option<f64> {
    let need_m = landing_needed_m(
        landing_ft.map(|f| f * 0.3048),
        takeoff_ft.map(|f| f * 0.3048),
        0.0,
    );
    (need_m > 0.0).then(|| (need_m / 0.3048).round())
}

/// The altitude this airframe cruises best at, at `weight_kg`.
///
/// A straight line through the profile's one reference point, because that is
/// what the real curve is over the weights an airliner actually cruises at:
/// the 737-800 sits near FL340 at max takeoff weight and near FL400 empty,
/// which is about 290 ft per tonne burned off. `None` unless the profile
/// declares all three numbers, so nothing invents an optimum for an airframe
/// that does not have one.
///
/// This is an ADVISORY number, never a target: a level belongs to ATC. See
/// [`step_climb_ft`].
pub fn optimum_altitude_ft(r: &ReferenceProfile, weight_kg: f64) -> Option<f64> {
    let (at, ft, slope) = (
        r.optimum_alt_at_kg?,
        r.optimum_alt_ft?,
        r.optimum_alt_ft_per_1000kg?,
    );
    (weight_kg.is_finite() && weight_kg > 0.0).then(|| ft + (at - weight_kg) / 1000.0 * slope)
}

/// How far below optimum is worth a word, and the step it would take.
///
/// The penalty for cruising low is real but gentle, so this does NOT fire for
/// being a few hundred feet off: only when a WHOLE standard step (2,000 ft,
/// the spacing of same-direction levels) sits between us and the optimum is
/// there anything a pilot could act on.
pub(super) const STEP_CLIMB_FT: f64 = 2000.0;

/// The widest gap a step climb answers: three levels. A step climb is how a
/// jet follows an optimum that drifts UP as fuel burns (EIDW-BIKF: 4,600 ft
/// by mid-Atlantic); a cruise many levels under optimum was PLANNED low, for
/// a reason the copilot cannot see (a short sector, the ceiling, the wind),
/// and one step does not address it. EGBB-EGKK (2026-09-27, 100 nm at
/// 15,000 ft) said "23,300 feet below optimum ... flight level one seven
/// zero would burn less": the gap was true (the profile's line at 64 t) and
/// the level was one step by design, and together they said nothing a pilot
/// could use.
pub(super) const STEP_CLIMB_MAX_GAP_FT: f64 = 3.0 * STEP_CLIMB_FT;

/// The next same-direction level up, if cruising `cruise_ft` is at least a
/// full step below `optimum_ft` and no more than [`STEP_CLIMB_MAX_GAP_FT`]
/// below it. `None` when we are at or near optimum, above it (a jet above
/// optimum is a different problem and not this one's to solve), or so far
/// below it that the level was planned, not drifted into. The step is
/// always exactly one level: a climb is a request to ATC, and asking for
/// two at once is how you get neither.
pub fn step_climb_ft(cruise_ft: f64, optimum_ft: f64) -> Option<f64> {
    (STEP_CLIMB_FT..=STEP_CLIMB_MAX_GAP_FT)
        .contains(&(optimum_ft - cruise_ft))
        .then_some(cruise_ft + STEP_CLIMB_FT)
}

/// The floor for the LANDING configuration: 1.2 x Vso when the profile
/// declares one, else the clean floor. The clean floor is right for a plan's
/// cruise target and the pattern, wrong for a flapped approach: the 737's
/// clean Vs is 139, its Vref with flaps 40 is 145, and flooring the
/// approach at 167 flew it to the threshold 22 kt fast, over a flap gate
/// (Vfe40 162) the schedule could never pass, and into two go-arounds
/// (LFLL 35L, 2026-09-17). On a 172 (Vso 40, Vs 48) the two floors are
/// 48 and 58 under a 65 kt approach: nothing changes. The snapshot's
/// `approach_floor_kts`. Pure.
pub fn approach_floor_for(vs0_kts: Option<f64>, vs_clean_kts: Option<f64>) -> Option<f64> {
    let by_vso = vs0_kts
        .filter(|v| *v > 0.0)
        .map(|v| (v * SPEED_FLOOR_RATIO).round());
    let clean = vs_clean_kts
        .filter(|v| *v > 0.0)
        .map(|v| (v * SPEED_FLOOR_RATIO).round());
    by_vso.or(clean)
}

/// Publish the landing-configuration floor alone.
#[cfg(test)]
pub fn publish_approach_floor(vs0_kts: Option<f64>, vs_clean_kts: Option<f64>) {
    let f = approach_floor_for(vs0_kts, vs_clean_kts);
    update_airframe_facts(|a| a.approach_floor_kts = f);
}

/// The live airframe's stall table: its clean and landing stall and the
/// detents that declare their own (`FlapDetent::vs_kts`).
#[derive(Debug, Clone, PartialEq)]
pub struct StallCurve {
    pub vs_clean_kts: f64,
    pub vs0_kts: Option<f64>,
    pub points: Vec<(f64, f64)>,
}

/// The stall table: `None` (the straight line) unless the profile has a
/// clean stall AND a detent that declares its own. Pure.
pub fn stall_curve_for(
    vs_clean_kts: Option<f64>,
    vs0_kts: Option<f64>,
    flaps: Option<&FlapProfile>,
) -> Option<StallCurve> {
    let points = detent_stall_points(flaps);
    vs_clean_kts
        .filter(|v| v.is_finite() && *v > 0.0)
        .filter(|_| !points.is_empty())
        .map(|vs| StallCurve {
            vs_clean_kts: vs,
            vs0_kts,
            points,
        })
}

/// Publish the stall table alone.
#[cfg(test)]
pub fn publish_stall_curve(
    vs_clean_kts: Option<f64>,
    vs0_kts: Option<f64>,
    flaps: Option<&FlapProfile>,
) {
    let c = stall_curve_for(vs_clean_kts, vs0_kts, flaps);
    update_airframe_facts(|a| a.stall_curve = c);
}

/// The live airframe's stall table, when its profile declares one.
pub fn stall_curve() -> Option<StallCurve> {
    let f = airframe_facts();
    match weighed(&f) {
        Some((s, p)) => {
            let vs0 = p.reference.as_ref().and_then(|r| r.vs0_kts);
            stall_curve_for(p.vs_clean_kts.map(|v| v * s), vs0.map(|v| v * s), p.flaps.as_ref())
                .map(|mut c| {
                    for k in &mut c.points {
                        k.1 *= s;
                    }
                    c
                })
        }
        None => f.stall_curve.clone(),
    }
}

/// The live airframe's landing-configuration floor, kts.
pub fn approach_floor_kts() -> Option<f64> {
    let f = airframe_facts();
    match weighed(&f) {
        Some((s, p)) => {
            let vs0 = p.reference.as_ref().and_then(|r| r.vs0_kts);
            approach_floor_for(vs0.map(|v| v * s), p.vs_clean_kts.map(|v| v * s))
        }
        None => f.approach_floor_kts,
    }
}

/// The floor for the configuration the aircraft is IN: the clean floor with
/// the flaps up, the landing floor with them fully out, interpolated by the
/// flap ratio between. A speed target is only as safe as the wing under it:
/// the 737's Vref 145 is 1.27 Vso with flaps 40 and 1.04 Vs with flaps 5,
/// and the circuit flew its final at flaps 5 toward exactly that number
/// (LFLL, 2026-09-17). Every leg that slows toward the approach speed
/// floors against this with the flaps it has actually commanded.
pub fn config_floor_kts(flap_ratio: f64) -> Option<f64> {
    let r = if flap_ratio.is_finite() {
        flap_ratio.clamp(0.0, 1.0)
    } else {
        0.0
    };
    // Declared per-detent stall speeds: the floor is 1.2 x the stall IN this
    // configuration, not a line between the clean and landing floors (the
    // A330's CONF 2 floor is 167 by its stall table, 180 by the line).
    if let Some(c) = stall_curve() {
        return Some(
            (SPEED_FLOOR_RATIO * stall_speed_in_config(c.vs_clean_kts, c.vs0_kts, &c.points, r))
                .round(),
        );
    }
    match (speed_floor_kts(), approach_floor_kts()) {
        (Some(clean), Some(landing)) => Some((clean + (landing - clean) * r).round()),
        (Some(clean), None) => Some(clean),
        (None, Some(landing)) => Some(landing),
        (None, None) => None,
    }
}

// ---- Speeds by weight ------------------------------------------------------
// A crew flies the speeds for what the aircraft weighs: the stalls, the floors
// and Vref move with it, and the takeoff speeds come off the card for the
// weight. The Zibo at 45.7 t flew its final at a fixed 145 with flaps 40,
// where the FCOM's Vref40 was 116: flat, nose-down, and on the nosewheel
// first (LEMG 30, sortie 1791371247, 2026-10-07), and it rotated at 145.

/// How long a published weight is believed, s: the monitor republishes it
/// every tick, so an older one is a monitor that stopped, not a weight.
pub const WEIGHT_FRESH_S: f64 = 30.0;

/// What the aircraft weighs, kg, and when it was read: a live reading, not
/// an airframe fact. The monitor's tick publishes it ([`publish_weight_now`]),
/// a failed read publishes none, and a new airframe starts without one
/// (`flight_state::publish_airframe`). Believed only while fresh.
pub(super) static WEIGHT_NOW: std::sync::Mutex<Option<(f64, std::time::Instant)>> =
    std::sync::Mutex::new(None);

/// Publish what the aircraft weighs now, kg (the monitor's tick). `None`
/// (a failed read) leaves no weight: the declared stalls fly.
pub fn publish_weight_now(kg: Option<f64>) {
    let w = kg
        .filter(|w| w.is_finite() && *w > 0.0)
        .map(|w| (w, std::time::Instant::now()));
    *WEIGHT_NOW.lock().unwrap_or_else(|e| e.into_inner()) = w;
}

/// The published weight, kg, while fresh ([`WEIGHT_FRESH_S`]).
pub(super) fn weight_published_kg() -> Option<f64> {
    WEIGHT_NOW
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .filter(|(_, at)| at.elapsed().as_secs_f64() <= WEIGHT_FRESH_S)
        .map(|(w, _)| w)
}

/// The stall scale at the live weight and the profile it applies to, when
/// the profile quotes its stalls at a weight and a fresh weight is known.
pub(super) fn weighed(f: &AirframeFacts) -> Option<(f64, &AircraftProfile)> {
    let p = f.profile.as_deref()?;
    Some((profile_stall_scale(p, weight_published_kg())?, p))
}

/// A profile's stall scale at `weight_kg` ([`weight_stall_scale`] on its
/// `reference.stall_ref_weight_kg`). Pure.
pub fn profile_stall_scale(p: &AircraftProfile, weight_kg: Option<f64>) -> Option<f64> {
    weight_stall_scale(p.reference.as_ref()?.stall_ref_weight_kg, weight_kg)
}

/// The factor a stall quoted at `ref_kg` takes at `weight_kg`: sqrt(W /
/// W_ref), lift going as V^2. `None` when either is missing or not a
/// weight, and for a ratio outside a quarter to four times (a read that is
/// not an aircraft's weight): the declared stalls then fly, never a guess.
/// Pure.
pub fn weight_stall_scale(ref_kg: Option<f64>, weight_kg: Option<f64>) -> Option<f64> {
    let (r, w) = (ref_kg?, weight_kg?);
    if !(r.is_finite() && w.is_finite() && r > 0.0 && w > 0.0) {
        return None;
    }
    let ratio = w / r;
    (0.25..=4.0).contains(&ratio).then(|| ratio.sqrt())
}

/// The type's speeds by weight (`weight_speeds:`), from its FCOM tables.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WeightSpeedsProfile {
    /// Vref by landing flap.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vref: Option<VrefTable>,
    /// V1, Vr and V2 for the takeoff flap.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub takeoff: Option<TakeoffSpeedTable>,
}

/// Vref, kts, by flap handle ratio (rows) and weight in tonnes (columns).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VrefTable {
    pub weights_t: Vec<f64>,
    /// Each row's flap handle ratio.
    pub flaps: Vec<f64>,
    pub table: Vec<Vec<f64>>,
    /// The approach speed's additive over Vref, kts; `None` is
    /// [`VREF_ADDITIVE_KTS`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub additive_kts: Option<f64>,
}

/// V1, Vr and V2, kts, by weight in tonnes, for ONE takeoff flap.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TakeoffSpeedTable {
    /// The takeoff flap handle ratio the table is for.
    pub flap: f64,
    pub weights_t: Vec<f64>,
    pub v1: Vec<f64>,
    pub vr: Vec<f64>,
    pub v2: Vec<f64>,
}

/// The approach speed over Vref with the autothrottle off and calm air:
/// Boeing's standard 5 kt.
pub const VREF_ADDITIVE_KTS: f64 = 5.0;

/// A speed read off a row by weight: straight between the columns, held at
/// the table's first and last column outside it (as the type's FMC holds
/// it). `None` for a malformed row (lengths differ, columns not rising, a
/// value not a speed) or a weight that is not one. Pure.
pub fn speed_by_weight(weights_t: &[f64], kts: &[f64], weight_kg: f64) -> Option<f64> {
    if weights_t.is_empty()
        || weights_t.len() != kts.len()
        || !(weight_kg.is_finite() && weight_kg > 0.0)
        || weights_t.iter().any(|w| !w.is_finite())
        || weights_t.windows(2).any(|w| w[1] <= w[0])
        || kts.iter().any(|k| !(k.is_finite() && *k > 0.0))
    {
        return None;
    }
    let t = weight_kg / 1000.0;
    let n = weights_t.len();
    if t <= weights_t[0] {
        return Some(kts[0]);
    }
    if t >= weights_t[n - 1] {
        return Some(kts[n - 1]);
    }
    let i = weights_t.iter().position(|w| *w >= t)?;
    let (w0, w1, k0, k1) = (weights_t[i - 1], weights_t[i], kts[i - 1], kts[i]);
    Some(k0 + (k1 - k0) * (t - w0) / (w1 - w0))
}

/// Vref with the landing flap (the fullest row: the flap every final here
/// lands with) at `weight_kg`, kts, unrounded. Pure.
pub fn landing_vref_kts(t: &VrefTable, weight_kg: f64) -> Option<f64> {
    if t.flaps.len() != t.table.len() {
        return None;
    }
    let row = t
        .flaps
        .iter()
        .enumerate()
        .filter(|(_, f)| f.is_finite())
        .max_by(|a, b| a.1.total_cmp(b.1))?
        .0;
    speed_by_weight(&t.weights_t, &t.table[row], weight_kg)
}

/// The approach speed at `weight_kg`: the landing Vref plus the additive,
/// whole knots. Pure.
pub fn approach_kts_at_weight(t: &VrefTable, weight_kg: f64) -> Option<f64> {
    let add = t
        .additive_kts
        .filter(|a| a.is_finite() && *a >= 0.0)
        .unwrap_or(VREF_ADDITIVE_KTS);
    landing_vref_kts(t, weight_kg).map(|v| (v + add).round())
}

/// The takeoff speeds at a weight, whole knots.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TakeoffSpeeds {
    pub v1: f64,
    pub vr: f64,
    pub v2: f64,
}

/// V1 / Vr / V2 at `weight_kg`, for `takeoff_flap` (the flap the takeoff
/// sets): `None` when the table is for another flap, so a speed is never
/// read off the wrong flap's card. Pure.
pub fn takeoff_speeds_at_weight(
    t: &TakeoffSpeedTable,
    takeoff_flap: Option<f64>,
    weight_kg: f64,
) -> Option<TakeoffSpeeds> {
    if !takeoff_flap.is_some_and(|f| (f - t.flap).abs() < 0.01) {
        return None;
    }
    let at = |k: &[f64]| speed_by_weight(&t.weights_t, k, weight_kg).map(f64::round);
    let s = TakeoffSpeeds {
        v1: at(&t.v1)?,
        vr: at(&t.vr)?,
        v2: at(&t.v2)?,
    };
    (s.v1 <= s.vr && s.vr <= s.v2).then_some(s)
}

/// What the aircraft weighs, read now, kg; `None` on a failed read.
pub async fn weight_now_kg(xp: &dyn sim::xplane::XPlane) -> Option<f64> {
    xp.get_float(sim::datarefs::TOTAL_WEIGHT_KG)
        .await
        .ok()
        .filter(|w| w.is_finite() && *w > 0.0)
}

/// The live airframe's approach speed at what it weighs now, with the weight
/// it was read at, when it declares a Vref table and the weight reads;
/// `None` keeps the caller's declared figure.
pub async fn approach_kts_now(xp: &dyn sim::xplane::XPlane) -> Option<(f64, f64)> {
    let t = live_profile()?.weight_speeds.as_ref()?.vref.clone()?;
    let w = weight_now_kg(xp).await?;
    approach_kts_at_weight(&t, w).map(|k| (k, w))
}

/// The live airframe's takeoff speeds at what it weighs now, with the
/// weight, when it declares a takeoff table for `takeoff_flap` and the
/// weight reads; `None` keeps the declared figures.
pub async fn takeoff_speeds_now(
    xp: &dyn sim::xplane::XPlane,
    takeoff_flap: Option<f64>,
) -> Option<(TakeoffSpeeds, f64)> {
    let t = live_profile()?.weight_speeds.as_ref()?.takeoff.clone()?;
    let w = weight_now_kg(xp).await?;
    takeoff_speeds_at_weight(&t, takeoff_flap, w).map(|s| (s, w))
}

/// The least thrust the autoland holds on a stabilised final, as a lever
/// ratio (`None` = idle is allowed). A jet's approach is flown ON thrust:
/// the 737 idled for the last mile to lose the circuit's excess and arrived
/// over the numbers 4 kt under Vref still spooling up (LFLL, 2026-09-17).
/// The snapshot's `approach_thrust_min`, a sane lever ratio or none. Pure.
pub fn approach_thrust_min_for(min: Option<f64>) -> Option<f64> {
    min.filter(|v| v.is_finite() && (0.0..=1.0).contains(v))
}

/// Publish the airframe's approach thrust floor alone.
#[cfg(test)]
pub fn publish_approach_thrust_min(min: Option<f64>) {
    let m = approach_thrust_min_for(min);
    update_airframe_facts(|a| a.approach_thrust_min = m);
}

/// The live airframe's approach thrust floor, if its profile declares one.
pub fn approach_thrust_min() -> Option<f64> {
    airframe_facts().approach_thrust_min
}

/// The live airframe's arrest lead (see [`ArrestLeadProfile`]): an absent
/// or malformed block is the default, never the previous airframe's.
pub fn arrest_lead() -> ArrestLeadProfile {
    airframe_facts().arrest_lead
}

/// The live airframe's whole profile, for the few consumers that need the
/// tool surface itself away from the command loop (the stand shutdown builds
/// the registry from it). Republished on EVERY swap by `publish_airframe`.
pub fn live_profile() -> Option<std::sync::Arc<AircraftProfile>> {
    airframe_facts().profile.clone()
}

/// The flare's elevator law for the live airframe (see
/// [`FlareElevatorProfile`]); `None` is the default law, never the previous
/// airframe's.
pub fn flare_elevator() -> Option<FlareElevatorProfile> {
    airframe_facts().flare_elevator
}

/// The live airframe's rudder authority on the roll (see
/// [`RollRudderProfile`]); `None` is the default roll law, never the
/// previous airframe's.
pub fn roll_rudder() -> Option<RollRudderProfile> {
    airframe_facts().roll_rudder
}

/// An airframe's landing distance in metres from the profile's feet, if it
/// recites one. Pure.
pub fn landing_distance_m_for(feet: Option<f64>) -> Option<f64> {
    feet.filter(|v| v.is_finite() && *v > 0.0)
        .map(|v| v * 0.3048)
}

/// The live airframe's landing distance, metres. The autoland's float rule
/// asks it how much pavement a landing from here still needs; `None` is the
/// old GA figure.
pub fn landing_distance_m() -> Option<f64> {
    airframe_facts().landing_distance_m
}

/// The live airframe's declared landing ROLL (`landing_roll_ft`), metres.
pub fn landing_roll_m() -> Option<f64> {
    live_profile()
        .and_then(|p| p.reference.as_ref().and_then(|r| r.landing_roll_ft))
        .filter(|v| v.is_finite() && *v > 0.0)
        .map(|v| v * 0.3048)
}

/// `asked`, raised to the floor when it is under it. Pure; the callers say so
/// aloud and record it.
pub fn floored_target(asked: f64, floor: Option<f64>) -> f64 {
    match floor {
        Some(f) if asked < f => f,
        _ => asked,
    }
}

/// The live airframe's VLO/VLE, if its profile declares one.
pub fn gear_limit_kts() -> Option<f64> {
    airframe_facts().gear_limit_kts
}

/// How far a flap reading may sit from a detent and still be AT it: the
/// flaps tool's "already there" test, under the closest detent spacing any
/// profile declares (the 737's eighths, 0.125).
pub const FLAP_DETENT_TOLERANCE: f64 = 0.02;

/// The placard of the flap that is out: the lowest Vfe of every detent at or
/// inside `flap_ratio` (within [`FLAP_DETENT_TOLERANCE`]). `None` clean, or
/// with no Vfe declared. `limits` is `(ratio, Vfe)` as [`flap_limits`] gives.
///
/// ONE lookup for the circuit's clean-up, autoland's target and missed
/// approach, the monitor's overspeed and the flaps tool (refactor audit
/// 2026-10-04, finding #12). They disagreed on two things, decided here:
/// - an entry at ratio 0 never carries a placard. Clean is limited by Vne,
///   not a Vfe, and a generic `.acf` read can put a Vfe on the retracted
///   row; the circuit counted it and capped a clean pattern with it while
///   autoland did not.
/// - the detent tolerance is 0.02. The circuit and autoland used 1e-6
///   (their ratios are commanded, exact); the monitor used 0.05 on a READ
///   position, which would take the next detent's limit on any profile with
///   detents closer than that. 0.02 counts a reading at its detent and no
///   other. Pure.
pub fn flap_placard_kts(flap_ratio: f64, limits: &[(f64, Option<f64>)]) -> Option<f64> {
    if !flap_ratio.is_finite() {
        return None;
    }
    limits
        .iter()
        .filter(|(ratio, _)| *ratio > 1e-6 && *ratio <= flap_ratio + FLAP_DETENT_TOLERANCE)
        .filter_map(|(_, vfe)| *vfe)
        .reduce(f64::min)
}

/// The live airframe's flap detents as `(ratio, Vfe)`, retracted first.
pub fn flap_limits() -> Vec<(f64, Option<f64>)> {
    airframe_facts().flap_limits.clone()
}

/// The dataref every throttle write and read should use.
///
/// The Citation X is why this exists. Its `CitX_throttle.lua` tracks
/// `throttle_jet_rev_ratio`, never `throttle_ratio_all`, and re-drives its own
/// levers from that every frame — so every reduction the speed governor
/// commanded went to a dataref the aircraft ignores. Flown live 2026-07-28 it
/// held takeoff thrust through the whole pattern: 215 knots on upwind, 309 on
/// crosswind, 373 on downwind against a 198 kt target, past both Vmo figures,
/// still climbing with a descent commanded. The copilot could not slow it down
/// and had no way to know.
/// The takeoff trim table and the switches that set it
/// (`crate::takeoff_trim`): the stabilizer read on its own gauge in units,
/// the centre of gravity in % MAC, and the type's FCOM table of units by CG
/// (rows) and weight in tonnes (columns), for the takeoff flap the profile
/// flies.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TakeoffTrimProfile {
    pub units: String,
    pub cg_mac: String,
    pub nose_up: String,
    pub nose_down: String,
    pub min_units: f64,
    pub max_units: f64,
    pub cg: Vec<f64>,
    pub weights_t: Vec<f64>,
    pub table: Vec<Vec<f64>>,
    /// The same table for a derated takeoff, by the rating's name in
    /// `takeoff_thrust:` (the 737-800's 24K and 22K tables carry more
    /// nose-up for the slower acceleration). A rating not listed, or no
    /// rating, flies the table above.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub by_rating: std::collections::BTreeMap<String, TrimTable>,
}

/// One takeoff trim table: units by CG (% MAC, rows) and weight (t, columns).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrimTable {
    pub cg: Vec<f64>,
    pub weights_t: Vec<f64>,
    pub table: Vec<Vec<f64>>,
}

impl TakeoffTrimProfile {
    /// The profile with the table for `rating` in place of the base one,
    /// where it declares one. Pure.
    pub fn for_rating(&self, rating: Option<&str>) -> TakeoffTrimProfile {
        let mut t = self.clone();
        if let Some(r) = rating.and_then(|r| self.by_rating.get(r)) {
            t.cg = r.cg.clone();
            t.weights_t = r.weights_t.clone();
            t.table = r.table.clone();
        }
        t
    }
}

/// An N1 table, percent: rows by temperature (degrees C, ascending),
/// columns by pressure altitude (ft, ascending).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct N1Table {
    pub temp_c: Vec<f64>,
    pub alt_ft: Vec<f64>,
    pub n1: Vec<Vec<f64>>,
}

/// One takeoff rating: its name as a crew says it, the weight it is chosen
/// below, and its takeoff N1 by OAT and pressure altitude.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThrustRating {
    pub name: String,
    /// Chosen when the aircraft weighs less than this, tonnes; `None` is
    /// the full rating, taken when no lighter one is.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub below_t: Option<f64>,
    /// The rating's thrust, thousands of pounds a side: the roll it needs
    /// against the full rating's is judged on it.
    pub klbf: f64,
    pub takeoff: N1Table,
    /// A further reduction by assumed temperature, where the runway has
    /// room for it. `None`: the rating's own N1.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub assumed: Option<AssumedTemp>,
}

/// A table by two axes, for the figures that are not an N1 by temperature
/// and altitude: rows and columns ascending, each use says which is which.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Grid {
    pub rows: Vec<f64>,
    pub cols: Vec<f64>,
    pub values: Vec<Vec<f64>>,
}

/// The assumed temperature method, as the type's FMC computes it: the
/// rating's N1 at the assumed temperature, less a correction for how far
/// it is above the real one.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AssumedTemp {
    /// N1 at the assumed temperature (rows) and pressure altitude.
    pub reduced: N1Table,
    /// The highest assumed temperature, C: rows OAT, columns pressure
    /// altitude (the 25 % thrust limit is in it).
    pub max_temp: Grid,
    /// The lowest, C, by pressure altitude: the rating's flat-rated one.
    pub min_temp: Grid,
    /// N1 taken off the reduced figure: rows the assumed temperature less
    /// the OAT, columns the OAT.
    pub adjust: Grid,
}

/// The takeoff and climb thrust flown to N1 (`crate::takeoff_thrust`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TakeoffThrustProfile {
    /// Lightest first: the first whose `below_t` the weight is under.
    pub ratings: Vec<ThrustRating>,
    /// Climb N1 by total air temperature and pressure altitude, flown from
    /// the thrust reduction height (`takeoff_config.thrust_reduction_agl_ft`).
    pub climb: N1Table,
    /// The lever-to-N1 line the lever is led on before the N1 answers:
    /// N1 at idle and at `lever_max`.
    pub idle_n1: f64,
    pub full_n1: f64,
    /// The highest lever the law writes. Default 1.0.
    #[serde(default = "TakeoffThrustProfile::default_lever_max")]
    pub lever_max: f64,
}

impl TakeoffThrustProfile {
    pub(super) fn default_lever_max() -> f64 {
        1.0
    }
}

/// The live airframe's reverse levers to write down to stow.
pub fn reverse_stow_levers() -> Vec<String> {
    live_profile().map(|p| p.reverse_stow_levers.clone()).unwrap_or_default()
}

/// The live airframe's reverse-hold command, when it declares one.
pub fn reverse_hold_command() -> Option<String> {
    live_profile().and_then(|p| p.reverse_hold_command.clone())
}

/// The live airframe's takeoff trim table, when it declares one.
pub fn takeoff_trim() -> Option<TakeoffTrimProfile> {
    live_profile().and_then(|p| p.takeoff_trim.clone())
}

/// The live airframe's takeoff thrust law, when it declares one.
pub fn takeoff_thrust() -> Option<TakeoffThrustProfile> {
    live_profile().and_then(|p| p.takeoff_thrust.clone())
}

/// The live airframe's hardware thrust-lever axis, when it declares one.
pub fn throttle_hardware_axis() -> Option<String> {
    live_profile().and_then(|p| p.throttle_hardware_axis.clone())
}

pub fn throttle_dataref() -> String {
    #[cfg(not(test))]
    let published = airframe_facts().throttle_dref.clone();
    #[cfg(test)]
    let published = THROTTLE_DREF_T.with(|t| t.borrow().clone());
    published.unwrap_or_else(|| STOCK_THROTTLE.to_string())
}
