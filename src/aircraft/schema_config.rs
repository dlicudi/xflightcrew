//! Profile schema, the configuration: flap detents and the stall by notch, the
//! reference speeds, taxi and tiller, the yoke, lights, electrics, the
//! transponder, the load sheet, signs, takeoff power and configuration, pumps,
//! and the airframe's own checklist state reads.

use super::*;

/// The climb-out retraction schedule from a flap profile: `(handle ratio,
/// manoeuvring speed)` for every detent that declares one, cleanest first.
/// Empty when none do, which keeps the old single-shot clean-up.
pub fn flap_schedule(f: Option<&FlapProfile>) -> Vec<(f64, f64)> {
    let mut v: Vec<(f64, f64)> = f
        .map(|f| f.detents.as_slice())
        .unwrap_or(&[])
        .iter()
        .filter_map(|d| Some((d.ratio, d.retract_above_kts?)))
        .filter(|(r, k)| r.is_finite() && k.is_finite() && *k > 0.0)
        .collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

/// The spoken name of the detent at `ratio` ("one", "ten degrees"), for the
/// call a crew makes as it selects it. `None` when no detent sits there.
pub fn flap_detent_spoken(f: Option<&FlapProfile>, ratio: f64) -> Option<String> {
    f?.detents
        .iter()
        .find(|d| (d.ratio - ratio).abs() < 1e-3)
        .map(|d| d.spoken.trim().to_string())
        .filter(|s| !s.is_empty())
}

/// One flap handle position.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlapDetent {
    /// Canonical token the LLM selects by and returns (e.g. `"10"`, `"full"`).
    pub label: String,
    /// Spoken form for the readback (e.g. `"ten degrees"`).
    pub spoken: String,
    /// Value written to the flap-handle dataref (0.0 up .. 1.0 full).
    pub ratio: f64,
    /// Max indicated airspeed (kts) at which it's safe to *extend* to this
    /// detent — the placard Vfe. `None` for retraction (always allowed).
    #[serde(default)]
    pub vfe_kts: Option<f64>,
    /// MINIMUM indicated airspeed (kts) at which this configuration is safe to
    /// fly — the detent's manoeuvring speed, and therefore the gate on
    /// RETRACTING to it. `None` leaves the detent out of the climb-out
    /// retraction schedule.
    ///
    /// Retraction had no speed gate at all: the takeoff cleaned up in ONE
    /// step, flaps to zero, on height plus `initial_climb_kts`. On a light
    /// single that is right — one flap setting, and Vy is the speed. On a
    /// 737 it took flaps 5 straight to 0 at 657 ft and 170 kt, some forty
    /// knots below flaps-up manoeuvring speed (BIKF-EGPH, 2026-09-20).
    #[serde(default)]
    pub retract_above_kts: Option<f64>,
    /// The stall speed IN this configuration, kts, at the same weight as
    /// `vs_clean_kts` and `vs0_kts`. `None` = the straight line between
    /// them. A slatted wing takes most of its reduction at the first
    /// detent: the A330's CONF 2 is 139 by its own .acf, the line said 150,
    /// and the monitor called "getting slow" at 175 kt (EGPF 2026-09-21).
    #[serde(default)]
    pub vs_kts: Option<f64>,
    /// How fast this configuration slows at idle in LEVEL flight, kt/s (the
    /// gear as the approach flies it: down from half flap), fitted from the
    /// airframe's own idle ticks (`approach_energy`, the plan backward from
    /// the stabilised gate). On a 3 degree path gravity gives back about
    /// 1.0 kt/s of it. `None` on every detent: the plan is off for the
    /// airframe and the distance schedule flies.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub idle_decel_kts_s: Option<f64>,
}

/// Flaps lower the stall speed by up to this fraction at full extension
/// when the profile declares no landing-configuration stall (172: clean Vs
/// 48, full-flap Vs0 40, about 17%).
pub const FLAP_STALL_REDUCTION: f64 = 0.17;

/// The declared per-detent stall speeds as `(handle ratio, kts)`, cleanest
/// first. Empty when no detent declares one. Pure.
pub fn detent_stall_points(f: Option<&FlapProfile>) -> Vec<(f64, f64)> {
    let mut v: Vec<(f64, f64)> = f
        .map(|f| f.detents.as_slice())
        .unwrap_or(&[])
        .iter()
        .filter_map(|d| Some((d.ratio, d.vs_kts?)))
        .filter(|(r, k)| r.is_finite() && k.is_finite() && *r > 0.0 && *r <= 1.0 && *k > 0.0)
        .collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

/// The stall speed at a flap handle ratio: through the airframe's declared
/// per-detent stall speeds when it has them (clean at 0, Vso at 1 unless a
/// detent says otherwise, straight between neighbours), else the straight
/// line from the clean stall to Vso, else the trainer's fixed reduction.
/// Never above clean. Pure.
pub fn stall_speed_in_config(
    vs_clean_kts: f64,
    vs0_kts: Option<f64>,
    points: &[(f64, f64)],
    flap_ratio: f64,
) -> f64 {
    let f = if flap_ratio.is_finite() {
        flap_ratio.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let vs0 = vs0_kts.filter(|v| *v > 0.0 && *v < vs_clean_kts);
    let mut knots: Vec<(f64, f64)> = points
        .iter()
        .copied()
        .filter(|(r, k)| r.is_finite() && k.is_finite() && *r > 0.0 && *r <= 1.0 && *k > 0.0)
        .map(|(r, k)| (r, k.min(vs_clean_kts)))
        .collect();
    if knots.is_empty() {
        return match vs0 {
            Some(vs0) => vs_clean_kts - (vs_clean_kts - vs0) * f,
            None => vs_clean_kts * (1.0 - FLAP_STALL_REDUCTION * f),
        };
    }
    knots.sort_by(|a, b| a.0.total_cmp(&b.0));
    knots.insert(0, (0.0, vs_clean_kts));
    if let (Some(v0), true) = (vs0, knots.last().is_some_and(|k| k.0 < 1.0)) {
        knots.push((1.0, v0));
    }
    for w in knots.windows(2) {
        let ((r0, k0), (r1, k1)) = (w[0], w[1]);
        if f <= r1 {
            let t = if r1 > r0 { (f - r0) / (r1 - r0) } else { 1.0 };
            return k0 + (k1 - k0) * t;
        }
    }
    knots.last().map(|k| k.1).unwrap_or(vs_clean_kts)
}

/// Lift-curve slope the .acf model assumes, per degree of angle of attack a
/// slat adds (thin-aerofoil order; the per-detent result moves under 1 kt
/// between 0.08 and 0.12 on the A330).
pub(super) const ACF_LIFT_SLOPE_PER_DEG: f64 = 0.1;

/// The per-detent stall speeds (detents 1..n, whole kts) an `.acf` implies,
/// or `None` unless it declares SLATS (`acf/_slat1_inc` over 0 with a
/// non-zero `_slat1_dn`): a flap-only wing keeps the straight line. X-Plane
/// flies the wing from these numbers: each detent adds the slat's stall-angle
/// increase (`_slat1_inc` x its `_slat1_dn/i` fraction) and the flap's lift
/// (`_flap1_cl` x `_flap1_dn/i` over the last notch); the two ends are
/// calibrated to the file's own `_Vs_kts` and `_Vso_kts` (lift goes as
/// 1/V^2). The A330: 147 / 139 / 131 for CONF 1 / 2 / 3. Pure.
pub fn detent_stall_from_acf(acf_text: &str) -> Option<Vec<f64>> {
    let mut vals: std::collections::HashMap<&str, f64> = std::collections::HashMap::new();
    for l in acf_text.lines() {
        let mut it = l.split_whitespace();
        if it.next() != Some("P") {
            continue;
        }
        if let (Some(k), Some(v)) = (it.next(), it.next().and_then(|v| v.parse::<f64>().ok())) {
            if k.starts_with("acf/") {
                vals.insert(k, v);
            }
        }
    }
    let get = |k: &str| vals.get(k).copied().filter(|v| v.is_finite());
    let n = get("acf/_flap_detents").map(|d| d.round() as usize).filter(|d| *d > 0)?;
    let vs = get("acf/_Vs_kts").filter(|v| *v > 0.0)?;
    let vso = get("acf/_Vso_kts").filter(|v| *v > 0.0 && *v < vs)?;
    let slat_inc = get("acf/_slat1_inc").unwrap_or(0.0);
    let flap: Vec<f64> = (0..=n)
        .map(|i| get(&format!("acf/_flap1_dn/{i}")).unwrap_or(0.0).max(0.0))
        .collect();
    let slat: Vec<f64> = (0..=n)
        .map(|i| get(&format!("acf/_slat1_dn/{i}")).unwrap_or(0.0).clamp(0.0, 1.0))
        .collect();
    if slat_inc <= 0.0 || slat.iter().all(|s| *s <= 0.0) {
        return None;
    }
    let flap_max = flap.iter().copied().fold(0.0, f64::max);
    let s = ACF_LIFT_SLOPE_PER_DEG * slat_inc;
    let fcl = get("acf/_flap1_cl").unwrap_or(0.0).max(0.0);
    let lift = |i: usize| {
        s * slat[i] + if flap_max > 0.0 { fcl * flap[i] / flap_max } else { 0.0 }
    };
    let full = lift(n);
    if full <= 0.0 {
        return None;
    }
    let (a, b) = (1.0 / (vs * vs), 1.0 / (vso * vso));
    Some(
        (1..=n)
            .map(|i| {
                let phi = (lift(i) / full).clamp(0.0, 1.0);
                (a + phi * (b - a)).powf(-0.5).round()
            })
            .collect(),
    )
}

/// Static reference figures recited on request (V-speeds, weights, limits), from
/// the airframe's POH/POM. Every field is optional so a profile fills in only what
/// it knows; a missing figure is simply not offered. Vne and clean-stall live at
/// the top level (the monitor uses them) and aren't duplicated here.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ReferenceProfile {
    #[serde(default)]
    pub kind: Option<String>, // e.g. "single-engine aircraft", "twin-engine jet"
    #[serde(default)]
    pub vno_kts: Option<f64>, // max structural cruise
    #[serde(default)]
    pub va_kts: Option<f64>, // maneuvering
    #[serde(default)]
    pub vx_kts: Option<f64>, // best angle of climb
    #[serde(default)]
    pub vy_kts: Option<f64>, // best rate of climb
    /// The speed the INITIAL climb is flown at, KIAS — what the auto takeoff
    /// protects between liftoff and the handoff, as opposed to the enroute
    /// climb speed `vy_kts` holds afterwards.
    ///
    /// On a trainer they are the same number and this stays `None`. On a jet
    /// they are not: the 737 lifts at V2+20 and climbs out around 170 while
    /// its `vy_kts` of 210 is the speed the pattern and the cruise climb use.
    /// Fed the enroute figure, the takeoff's climb law read 174 kt as
    /// "36 knots slow", stripped 2,150 fpm off the commanded rate and
    /// levelled a 737 at full thrust at 1,030 ft (LFLL, 2026-09-17).
    /// Defaults to `vy_kts`, so every profile that does not set it is
    /// unchanged.
    #[serde(default)]
    pub initial_climb_kts: Option<f64>,
    /// Height AGL where the climb-out stops holding `initial_climb_kts` and
    /// starts accelerating and cleaning up. A jet's declaration: below it the
    /// takeoff climbs as hard as it must (to `best_climb_fpm`) to keep the
    /// speed, and the flaps stay where the takeoff set them. `None` keeps the
    /// trainer's climb-out (see `autotakeoff::ClimbOut`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub acceleration_height_agl_ft: Option<f64>,
    #[serde(default)]
    pub vr_kts: Option<f64>, // rotate
    /// How fast the rotation raises the nose, degrees a second; `None` is
    /// the airline 2.5 (`autotakeoff::ROTATE_RATE_DEG_S`). A light single
    /// rotates faster: the SR22 at 2.5 from its nose-down roll attitude
    /// took 6.2 s from Vr 73 to lift off at 99.5 KIAS (LEMG 12 1790690995,
    /// 2026-09-29), the drift growing 5.9 m in those seconds.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rotate_rate_deg_s: Option<f64>,
    /// Blue line: single-engine best rate of climb. A TWIN's number — the one
    /// speed a multi pilot flies the whole engine-failure drill against — and
    /// meaningless on a single, so it stays `None` there.
    #[serde(default)]
    pub vyse_kts: Option<f64>,
    /// Red line: single-engine minimum control speed. The other half of the
    /// twin pair above.
    #[serde(default)]
    pub vmca_kts: Option<f64>,
    #[serde(default)]
    pub best_glide_kts: Option<f64>,
    #[serde(default)]
    pub vs0_kts: Option<f64>, // stall, full flaps
    #[serde(default)]
    pub approach_kts: Option<f64>,
    #[serde(default)]
    pub max_crosswind_kts: Option<f64>, // demonstrated
    #[serde(default)]
    pub cruise_kts: Option<f64>,
    /// Target INDICATED airspeed to hold in cruise (the autoflight cruise-power
    /// governor chases this). Distinct from `cruise_kts`, which is the spoken POH
    /// figure (usually a TAS). `None` = no automatic cruise-power reduction.
    #[serde(default)]
    pub cruise_ias_kts: Option<f64>,
    /// Optimum cruise altitude at [`Reference::optimum_alt_at_kg`], feet, and
    /// how many feet it rises per 1,000 kg burned off. Jets only: the altitude
    /// that flies best moves UP as the aircraft gets lighter, by roughly a
    /// level an hour on a narrowbody, and nothing in the tree ever looked
    /// (EIDW-BIKF, 2026-09-20: planned FL340 and held it for the whole
    /// crossing, 4,600 ft below optimum by mid-Atlantic). `None` on anything
    /// that does not step-climb, which is every piston here — the copilot then
    /// says nothing rather than inventing a number.
    #[serde(default)]
    pub optimum_alt_ft: Option<f64>,
    #[serde(default)]
    pub optimum_alt_at_kg: Option<f64>,
    #[serde(default)]
    pub optimum_alt_ft_per_1000kg: Option<f64>,
    /// The rates the ENROUTE vertical law climbs and descends at, fpm. Both
    /// default to the GA figure when a profile is silent.
    ///
    /// Distinct from `best_climb_fpm`, which is the takeoff's initial climb.
    /// These pace the follower between fixes, and the descent one also places
    /// the top of descent, so the two stay consistent by construction: the
    /// TOD is put exactly where this rate can make the constraint.
    ///
    /// 700 fpm is a 1.1 degree path at jet speeds. The EIDW-BIKF arrival
    /// (2026-09-20) needed about 2,000 fpm on every leg from BASLU inbound
    /// and the law could offer a third of it — the aircraft would have
    /// reached Keflavík some 17,000 ft high. The same constant capped the
    /// CLIMB, which is why FL340 took 37 minutes instead of 20, all of it
    /// burning low-altitude fuel.
    #[serde(default)]
    pub enroute_climb_fpm: Option<f64>,
    #[serde(default)]
    pub enroute_descent_fpm: Option<f64>,
    #[serde(default)]
    pub best_climb_fpm: Option<f64>,
    #[serde(default)]
    pub max_gross_lb: Option<f64>,
    #[serde(default)]
    pub empty_weight_lb: Option<f64>,
    #[serde(default)]
    pub useful_load_lb: Option<f64>,
    #[serde(default)]
    pub service_ceiling_ft: Option<f64>,
    #[serde(default)]
    pub fuel_capacity_gal: Option<f64>,
    #[serde(default)]
    pub fuel_useable_gal: Option<f64>,
    #[serde(default)]
    pub endurance_hours: Option<f64>,
    #[serde(default)]
    pub takeoff_distance_ft: Option<f64>,
    /// GROUND ROLL, feet — the runway the wheels actually need. The automation
    /// flies this one (the line-up's full-length check, `runway_fit`, full
    /// auto's lined-up test), all of which add their own ×1.4 margin on top.
    ///
    /// Distinct from `takeoff_distance_ft`, which is the figure the copilot
    /// RECITES: on a GA single that is the distance over a 50 ft obstacle, but
    /// on an airliner it is the factored takeoff FIELD LENGTH at max weight,
    /// which already carries its own margins. Multiplying that again demanded
    /// 3179 m for an MD-82 — more than the whole of Luton's runway — so the
    /// line-up backtracked for length it did not need and full auto then judged
    /// a perfectly good line-up a failure and quit, twice (2026-07-28).
    /// `None` falls back to `takeoff_distance_ft`, which is what every GA
    /// profile has always used.
    #[serde(default)]
    pub takeoff_roll_ft: Option<f64>,
    #[serde(default)]
    pub landing_distance_ft: Option<f64>,
    /// LANDING ROLL, feet: touchdown to a stop on a wet runway, the room a
    /// landing ALREADY ON THE WHEELS needs (`autoland::landing_call`, the
    /// committed flare). Distinct from `landing_distance_ft`, which on an
    /// airliner is the factored landing FIELD LENGTH from 50 ft (the
    /// planner's and the self-pick's figure): asked of a flare at 4 ft, that
    /// figure less the air flown sent the 737 around toward the Rock with
    /// 1,109 m of LXGB ahead (2026-10-06), and it stopped in about 540.
    /// `None` keeps the book figure's rules (no gate decision).
    #[serde(default)]
    pub landing_roll_ft: Option<f64>,
    #[serde(default)]
    pub max_rpm: Option<f64>,
    #[serde(default)]
    pub oil_temp_min_f: Option<f64>,
    #[serde(default)]
    pub oil_temp_max_f: Option<f64>,
    #[serde(default)]
    pub oil_press_min_psi: Option<f64>,
    #[serde(default)]
    pub oil_press_max_psi: Option<f64>,
    /// The all-up weight, kg, the declared stall speeds (`vs_clean_kts`,
    /// `vs0_kts`, every detent's `vs_kts`) are quoted at. Declared, the
    /// stalls and every floor built on them follow the LIVE weight as
    /// sqrt(W / this) ([`weight_stall_scale`]): lift goes as V^2, so a wing
    /// carrying less stalls slower. The Zibo at 45.7 t was floored at 137
    /// on a 64 t Vso, over the FCOM's Vref40 of 116 (LEMG 30, 2026-10-07).
    /// `None` = the declared stalls at every weight.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stall_ref_weight_kg: Option<f64>,
}

/// Taxi pacing, per airframe: the pace on a straight taxiway, and the caps the
/// schedule falls to for an ordinary bend and a sharp one.
///
/// These were one set of numbers for every aircraft (10/6/4), tuned on the 172,
/// where 10 knots is a THROTTLE setting. On a jet it is a brake setting: an
/// MD-82 at idle thrust rolls past 10 knots on flat pavement on its own, so the
/// governor rides the brakes the whole taxi, and airline practice is nearer
/// 15-20 knots down a long taxiway with 10 through the turns. Anything omitted
/// falls back to the GA figures, so an airframe that says nothing taxis exactly
/// as it did before this existed.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TaxiProfile {
    /// Pace on a straight leg, knots — the target when the pilot names none.
    #[serde(default)]
    pub kts: Option<f64>,
    /// Cap through an ordinary bend (>= 30 degrees).
    #[serde(default)]
    pub turn_kts: Option<f64>,
    /// Cap through a sharp corner (>= 100 degrees) and a turn-around join.
    #[serde(default)]
    pub sharp_kts: Option<f64>,
    /// Heading error (degrees) at which the steering commands FULL deflection.
    /// Bigger = gentler. `None` keeps the shared default, which was tuned on the
    /// 172.
    #[serde(default)]
    pub steer_full_deg: Option<f64>,
    /// Seconds of yaw-rate LEAD mixed into the steering error — the damping
    /// term. `None` keeps the shared default.
    ///
    /// This is the knob a CASTORING nosewheel needs, and the reason it is
    /// per-airframe at all. A linked nosewheel (172, MD-82) is self-damped and
    /// its taxi yaw rates are small, so the term is nearly inert there; a
    /// castoring one steers through the rudder and differential braking, which
    /// is a laggier plant that the same gain drives into a weave. Measured over
    /// the blackbox at 4-12 kt, yaw response per unit of steer runs 44 deg/s on
    /// the Twin Otter and 12.8 on the 172, but only 6.9 on the SF50 and 2.3 on
    /// the SR22 — the two castoring airframes are not weaker loops by accident,
    /// they are laggier ones, and lag is fixed with phase lead, not gain.
    #[serde(default)]
    pub steer_lead_s: Option<f64>,
    /// Cap on how fast the steering command may travel, per SECOND. `None` = no
    /// limit, which is every airframe that has never needed one.
    ///
    /// Deliberately generous where it is set: a slew limit is phase lag, and a
    /// TIGHT one causes the very oscillation it looks like it should cure (the
    /// auto-takeoff roll's 0.25-per-tick limiter swept only 0.5/s and produced a
    /// textbook slew-induced limit cycle, 2026-07-21). Set it above the rate the
    /// airframe actually uses so it only ever clips the violent transient.
    #[serde(default)]
    pub steer_slew_per_s: Option<f64>,
    /// Pure-pursuit aim time for the taxi steering, seconds. `None` keeps the
    /// shared 5 s. A slow-turning plant needs a LONG aim — see the autotaxi
    /// lookahead law; the SF50 carries 9.
    #[serde(default)]
    pub lookahead_s: Option<f64>,
    /// Steer with DIFFERENTIAL BRAKES as well as the yoke-heading channel.
    ///
    /// For airframes where that channel has nothing behind it at taxi speed:
    /// a castoring nosewheel takes no steering input, and on a JET there is no
    /// prop wash over the rudder either, so between walking pace and
    /// rudder-effective speed the aircraft simply does not answer. The SF50 is
    /// the proving case: thirty seconds of full-scale steer command while the
    /// cross-track error GREW to 13.8 m and the aircraft taxied down the grass
    /// beside taxiway Alpha (Luton, 2026-07-28). Real castoring-wheel aircraft
    /// are steered exactly this way — a touch of inside brake.
    #[serde(default)]
    pub brake_steer: bool,
    /// The taxi hold's speed control below idle: the deepest share of the
    /// BETA range (0..1, of X-Plane's -1..0) the lever may go, before any
    /// brake is used on a rolling taxi. For a turboprop whose idle is a
    /// taxi too fast: "Don't ride the brakes in taxi. Ease the throttle into
    /// beta instead" (Lancair Evolution POH). `None`: the brakes, as before.
    /// A hypothesis until a blackboxed taxi measures it.
    #[serde(default)]
    pub beta_max: Option<f64>,
    /// The yaw rate a taxi corner may ask of this airframe, degrees per
    /// second: a corner of radius R is taken no faster than this times R.
    /// `None` keeps the angle-only bend caps. The 737 turns at about 20 deg/s
    /// at full lock under 7 kt (LEMG and LIBP, 2026-09-26), and entered an
    /// 80-degree corner drawn at a 9 m radius at 11.5 kt, where the route asked
    /// 38 deg/s: it swung 5.8 m wide.
    #[serde(default)]
    pub max_yaw_dps: Option<f64>,
    /// A full-length backtrack's pace down the strip, knots. `None` keeps the
    /// straight pace floored at 14 (a GA backtrack at 6 kt timed full auto
    /// out at LXGB). The Evolution's 14 ran it to 16 kt on LILB's 15 m strip,
    /// 3.6 m off, and 9.5 m past its stop (2026-09-30, #196).
    #[serde(default)]
    pub backtrack_kts: Option<f64>,
    /// The nosewheel TILLER, where the pedals do not turn the aircraft
    /// ([`TillerProfile`]). `None`: the pedal channel alone, as before.
    #[serde(default)]
    pub tiller: Option<TillerProfile>,
}

/// An airliner's large-angle ground steering. The pedal channel is the rudder
/// pedals, and a 737's pedals steer the nose about 7 degrees: auto taxi held
/// full pedal on the Zibo for many seconds with 152 degrees to turn and rolled
/// 90 m off EGHL's route (2026-10-07; probed parked, 0.5 on the pedals gave
/// 3.5 degrees of nosewheel). The tiller turns it 78. The steer law's command
/// goes to the plugin's `xcopilot/tiller` channel as well as the pedals (the
/// 737 sums the two), faded out between [`TillerProfile::full_below_kts`] and
/// [`TillerProfile::hand_off_kts`] (`crate::tiller::share`): a crew hands the
/// tiller off and steers on the pedals for the takeoff roll and the fast part
/// of the rollout.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TillerProfile {
    /// What the plugin drives with the channel ([`TillerMethod`]).
    pub method: TillerMethod,
    /// Full tiller deflection, nosewheel degrees: needed by the stock method
    /// (the channel becomes `tire_steer_command_deg`).
    #[serde(default)]
    pub max_deg: Option<f64>,
    /// Groundspeed at and above which the tiller is let go, knots. Default 30.
    #[serde(default)]
    pub hand_off_kts: Option<f64>,
    /// Groundspeed below which the tiller takes the whole command, knots.
    /// Default 20.
    #[serde(default)]
    pub full_below_kts: Option<f64>,
}

/// Which input the plugin's tiller channel drives (`xcopilot/tiller_method`).
/// The codes are the plugin's (`plugin/src/tiller.cpp`); the names it writes
/// are listed by [`TillerMethod::references`] so the swap-time probe says a
/// missing one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TillerMethod {
    /// The Zibo's NOSEWHEEL AXIS option held at VR while steering and
    /// `laminar/B738/nosewheel_steer_ratio` written; the pilot's option put
    /// back at release.
    ZiboVr,
    /// The Zibo's NOSEWHEEL AXIS option held at YAW: the tiller value rides
    /// the pedal channel's yaw ratio.
    ZiboYaw,
    /// `laminar/B738/nosewheel_steer_override` and the ratio.
    ZiboOverride,
    /// `override_wheel_steer` and `tire_steer_command_deg[0]`, any airframe.
    Stock,
}

impl TillerMethod {
    /// The plugin's code for the method.
    pub fn code(self) -> i32 {
        match self {
            TillerMethod::ZiboVr => 1,
            TillerMethod::ZiboYaw => 2,
            TillerMethod::ZiboOverride => 3,
            TillerMethod::Stock => 4,
        }
    }

    /// The airframe and sim names the plugin writes for this method.
    pub fn references(self) -> &'static [&'static str] {
        match self {
            TillerMethod::ZiboVr => &[
                "laminar/B738/effects/nosewheel",
                "laminar/B738/nosewheel_steer_ratio",
            ],
            TillerMethod::ZiboYaw => &["laminar/B738/effects/nosewheel"],
            TillerMethod::ZiboOverride => &[
                "laminar/B738/nosewheel_steer_override",
                "laminar/B738/nosewheel_steer_ratio",
            ],
            TillerMethod::Stock => &[
                "sim/operation/override/override_wheel_steer",
                "sim/flightmodel2/gear/tire_steer_command_deg",
            ],
        }
    }
}

/// The input the plugin's yoke channels drive (`xcopilot/yoke_method`).
/// The codes are the plugin's (`plugin/src/drefs.cpp`, `kYoke*`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum YokeInput {
    /// The Zibo 737's external input: `laminar/B738/flt_ctrls/joy_override`
    /// and its `joy_pitch`, `joy_roll`, `joy_heading`. zibomod rewrites the
    /// stock yoke ratios every frame, so a stock write is a silent no-op.
    ZiboJoy,
}

impl YokeInput {
    /// The plugin's code; 0 is the stock path.
    pub fn code(input: Option<YokeInput>) -> i32 {
        match input {
            None => 0,
            Some(YokeInput::ZiboJoy) => 1,
        }
    }

    /// The airframe names the plugin writes on this route.
    pub fn references(self) -> &'static [&'static str] {
        match self {
            YokeInput::ZiboJoy => &[
                "laminar/B738/flt_ctrls/joy_override",
                "laminar/B738/flt_ctrls/joy_pitch",
                "laminar/B738/flt_ctrls/joy_roll",
                "laminar/B738/flt_ctrls/joy_heading",
            ],
        }
    }
}

/// The live airframe's yoke input, `None` for the stock yoke ratios.
pub fn yoke_input() -> Option<YokeInput> {
    live_profile().and_then(|p| p.yoke_input)
}

/// The live airframe's tiller, when it declares one.
pub fn tiller() -> Option<TillerProfile> {
    live_profile().and_then(|p| p.taxi.as_ref().and_then(|t| t.tiller.clone()))
}

/// A switch whose only control is a TOGGLE command, paired with the dataref
/// that reads its state.
///
/// Firing a toggle as "on" turns the switch OFF whenever it was already on, so
/// a toggle can only be driven safely by reading first and acting on the
/// difference. Whole airframes are built this way: every light and the avionics
/// master on the Citation X are toggle-only, and its xlua datarefs are
/// READ-ONLY over the Web API (`PATCH … → 403 dataref_is_readonly`, verified
/// live 2026-07-28), so commands are not merely preferred there — they are the
/// only way to actuate anything at all.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToggleSwitch {
    /// The toggle command to fire when the state doesn't match.
    pub command: String,
    /// The 0/1 dataref that reads this switch's actual position.
    pub dataref: String,
}

/// One switchable light.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LightProfile {
    /// Canonical token the LLM selects by (e.g. `"landing"`).
    pub key: String,
    /// Spoken form for the readback (e.g. `"landing light"`).
    pub spoken: String,
    /// The 0/1 switch dataref — read back to confirm the actual state.
    pub dataref: String,
    /// Commands that drive the panel switch on/off. Preferred over a raw
    /// `dataref` write: a write can set the value without animating the 3D
    /// switch (as the avionics master did). When present the tool fires these;
    /// `dataref` is still used for the readback. Falls back to a write if unset.
    #[serde(default)]
    pub command_on: Option<String>,
    #[serde(default)]
    pub command_off: Option<String>,
    /// Further switch datarefs driven to the SAME state as `dataref`. One
    /// "light" on the panel is sometimes two switches: the Citation X has
    /// independent left and right landing lights, and switching only one leaves
    /// the aircraft half lit while the readback happily confirms the half it
    /// looked at. Written, never commanded — the extras exist precisely because
    /// the airframe has no single command for the pair.
    #[serde(default)]
    pub also_datarefs: Vec<String>,
    /// Toggle-only switches making up this light. When present these are
    /// driven INSTEAD of the command/dataref above: each is read and toggled
    /// only if it doesn't already match. One "light" can be several — the
    /// Citation X's landing lights are an independent left and right.
    #[serde(default)]
    pub toggles: Vec<ToggleSwitch>,
    /// The copilot switches this light on by its own hand once the aircraft is
    /// powered and the sun is down, and off again at daybreak (see
    /// [`crate::cockpit_auto`]). For the cockpit lighting a crew member would
    /// simply reach up and set — the Baron's roof floods — never for an
    /// EXTERIOR light, where switching on the pilot's behalf changes what other
    /// traffic sees.
    ///
    /// One-shot per spell, never re-asserted: the pilot switching it back off
    /// is the pilot's decision and the copilot does not argue. Off is only ever
    /// commanded for a light the copilot itself turned on.
    #[serde(default)]
    pub auto_at_night: bool,
}

/// One electrical system the `set_electrical` tool can switch. Driven by the
/// profile so airframes with different wiring (the 172's single battery + split
/// master; the SR22's dual battery/alternator) need no code — just config.
/// Switching fires `on_commands`/`off_commands` (commands animate the 3D switch;
/// a raw dataref write may not); `readback` datarefs (any may be `name[i]`) are
/// read after to confirm — all must match the target, else the mixed state is
/// reported.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectricalSystem {
    /// Token the LLM selects by (e.g. `"battery"`, `"battery1"`, `"avionics"`).
    pub key: String,
    /// Spoken form for the readback (e.g. `"battery one"`, `"both batteries"`).
    pub spoken: String,
    pub on_commands: Vec<String>,
    pub off_commands: Vec<String>,
    /// Switch datarefs written to 1/0 directly, for systems the sim lets us
    /// write but that have no on/off command. Fired after any commands above.
    /// NOT a way to drive an xlua add-on: those datarefs are read-only over the
    /// Web API (403), which is what `toggles` exists for.
    #[serde(default)]
    pub write_datarefs: Vec<String>,
    /// Toggle-only switches in this system, each read first and toggled only if
    /// it doesn't already match the target (see [`ToggleSwitch`]). The Citation
    /// X's avionics master is toggle-only AND its dataref is unwritable, so
    /// this is the only way to switch it.
    #[serde(default)]
    pub toggles: Vec<ToggleSwitch>,
    /// Guard covers over this system's switch, each a TOGGLE command and the
    /// dataref that reads its position (0 closed, 1 open). A guarded switch
    /// is held ON by its closed guard: the stock 737's xlua forces
    /// `battery_on[0]` to 1 every frame while `guard_cover_pos[0]` is under
    /// 0.5, so `battery_1_off` with the guard shut is undone the next frame
    /// and the mission's shutdown said "The battery didn't move" (17:07Z,
    /// 2026-09-26). Switching OFF opens each guard first and waits for it to
    /// read open; switching ON closes it after the switch, as a crew does.
    #[serde(default)]
    pub guards: Vec<ToggleSwitch>,
    #[serde(default)]
    pub readback: Vec<String>,
}

/// The add-on's own transponder mode SELECTOR, for airframes where the stock
/// `transponder_mode` actuator is downstream of the knob rather than upstream
/// of the box.
///
/// The RWDesigns Twin Otter is the case that found this: its KT73 is a SASL
/// instrument, and `master.lua`'s `KT73()` runs every frame setting
/// `sim/cockpit/radios/transponder_mode` FROM `TO/radios/KT73SelKnob`. Writing
/// the stock actuator therefore does nothing that survives a frame — the
/// cockpit knob never moves and the mode snaps back — so the copilot's
/// standby/altitude switching was silently a no-op on that airframe
/// (2026-07-27). Same shape as [`AircraftProfile::fuel_pump_commands`]: the
/// universal write is not wrong, it is simply not connected to anything here.
///
/// The values are the KNOB's positions, which need not match the stock
/// encoding at all (the Twotter's ALT is knob 5 and drives stock mode 2, not
/// 3), which is exactly why the readback re-reads the KNOB.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransponderModeKnob {
    /// The add-on's selector dataref ("TO/radios/KT73SelKnob").
    pub dataref: String,
    pub off: i64,
    pub standby: i64,
    pub on: i64,
    pub altitude: i64,
    /// Commands that turn the knob one position up / down, for a knob whose
    /// dataref is READ-ONLY. The A330's `auto_on_knob_pos` is: probed live it
    /// reports `is_writable: false`, so every mode write on that airframe was
    /// a silent no-op and the flight went out in standby to "negative radar
    /// contact, recycle transponder" (EGPH, 2026-09-20). With both declared
    /// the knob is STEPPED, read before each step, and the dataref is only
    /// ever read.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub up_command: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub down_command: Option<String>,
    /// Other knobs the box needs before ON or ALTITUDE transmits anything.
    /// The A330 has three: with the mode knob in AUTO and the TCAS knob left
    /// in STBY its script writes stock mode 1, so the takeoff blackboxed
    /// `transponder_alt ok: true` from the one knob it looked at and the
    /// flight squawked standby round Glasgow's zone (2026-09-21). Turned
    /// after the mode knob, never for OFF or STANDBY, and the answer is then
    /// the STOCK mode, which the add-on computes from all of them.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub also: Vec<CompanionKnob>,
}

/// Whether the stock load sheet writes (`weight/m_fuel`, `weight/m_fixed`)
/// are known to take on an airframe (see [`AircraftProfile::load_sheet`]).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LoadSheet {
    /// Stock tanks and payload: written and read back (the 737, probed live
    /// on the EGPH ramp 2026-09-21; the 172).
    #[default]
    Stock,
    /// Never written and read back on this airframe.
    Unprobed,
    /// The aircraft's scripts own the tanks: a stock write is undone.
    OwnFuelSystem,
}

/// An add-on's own seat belt sign switch: a read-only position stepped by
/// its commands. `off` is where the crew leaves it in cruise (AUTO on an
/// Airbus: the signs come back with the flaps or the gear by themselves).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SeatBeltSigns {
    pub position: String,
    pub on: i64,
    pub off: i64,
    pub up_command: String,
    pub down_command: String,
}

/// The configuration items an airliner's crew owns and a trainer has not got.
/// Stock datarefs on purpose (`speedbrake_ratio`, `auto_brake_level`): the
/// profile declares THAT the airframe has them, never a name.
/// The takeoff power the MANUAL allows, as engine torque, until a speed:
/// the Lancair Evolution takes off at 1,250 ft-lb of its 2,230 "since more
/// power would over-power the rudders' and ailerons' ability to counter the
/// engine torque" (N844X POH), more only as the speed builds past 150 kt
/// (POM). The takeoff put in full power 2.5 s into the roll and the aircraft
/// left LILB 03's centreline by 129 m with the steering pinned against it
/// (2026-09-25). Held on the MEASURED torque (`ENGN_TRQ`), never a guessed
/// lever position.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TakeoffPowerProfile {
    pub torque_ftlb: f64,
    pub until_kias: f64,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TakeoffConfigProfile {
    /// Ground spoilers armed for landing, and for takeoff unless
    /// `takeoff_speedbrake_down`. Also declares the speedbrake lever itself.
    #[serde(default)]
    pub arm_spoilers: bool,
    /// Boeing practice: the lever sits in the DOWN detent for takeoff (armed
    /// is a takeoff configuration warning on the 737, and its own Before
    /// Takeoff list says so); a reject raises it by hand. Armed only at the
    /// gear. An Airbus leaves this false and arms for the roll.
    #[serde(default)]
    pub takeoff_speedbrake_down: bool,
    /// Autobrake to RTO / MAX before the roll (stock level 0).
    #[serde(default)]
    pub autobrake_rto: bool,
    /// The stock `auto_brake_level` autoland selects at the gear. The A330's
    /// script lights LO at 3 and MED at 4. `None` leaves it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub landing_autobrake_level: Option<i64>,
    /// Flight directors ON before the roll (stock `flight_director_mode`,
    /// raised by `sim/autopilot/fdir_on`, read back).
    #[serde(default)]
    pub flight_directors: bool,
    /// Autothrottle ARMED and not engaged before the roll (stock
    /// `autothrottle_enabled` 0: armed by `autothrottle_arm` from hard off,
    /// an engaged mode taken out by `autothrottle_off`, because the takeoff
    /// owns the thrust levers). An airframe whose script arms it from the
    /// lever detents (the A330) leaves this false.
    #[serde(default)]
    pub autothrottle_armed: bool,
    /// V2, KIAS, set in the MCP speed window before the roll (stock
    /// `airspeed_dial_kts_mach`, read back). The reference card's figure.
    /// `None` leaves the window.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub v2_kts: Option<f64>,
    /// Thrust reduction: above this height AGL the takeoff pulls the levers
    /// back to `climb_thrust_ratio`. Both or neither; `None` keeps takeoff
    /// thrust until whoever takes the aircraft next sets it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thrust_reduction_agl_ft: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub climb_thrust_ratio: Option<f64>,
}

impl AutoflightProfile {
    /// The stock reads and commands the panel's owner (`autoflight`) flies a
    /// declared panel through beside its own buttons, for the profile probe
    /// ([`profile_references`]). Kept equal to `autoflight::stock_references`
    /// by its test (this crate's lib cannot see that module).
    pub const STOCK: [&'static str; 11] = [
        "sim/cockpit/autopilot/autopilot_state",
        "sim/cockpit2/autopilot/autothrottle_on",
        "sim/cockpit2/autopilot/autothrottle_enabled",
        "sim/cockpit2/autopilot/servos_on",
        "sim/cockpit2/autopilot/heading_status",
        "sim/cockpit2/autopilot/vvi_status",
        "sim/cockpit2/autopilot/altitude_hold_status",
        "sim/autopilot/autothrottle_off",
        "sim/cockpit2/autopilot/altitude_dial_ft",
        "sim/cockpit/autopilot/vertical_velocity",
        "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot",
    ];
}

impl TakeoffConfigProfile {
    pub const SPEEDBRAKE: &'static str = "sim/cockpit2/controls/speedbrake_ratio";
    /// 0 RTO, 1 off, 2..5 the landing levels.
    pub const AUTOBRAKE: &'static str = "sim/cockpit2/switches/auto_brake_level";
    /// 0 off, 1 on, 2 on with the servos.
    pub const FD_MODE: &'static str = "sim/cockpit2/autopilot/flight_director_mode";
    pub const CMD_FD_ON: &'static str = "sim/autopilot/fdir_on";
    /// -1 hard off, 0 ARMED (servos declutched; the PFD's "ARM"), 1 speed,
    /// 2 N1, 3 retard.
    pub const AT_MODE: &'static str = "sim/cockpit2/autopilot/autothrottle_enabled";
    pub const CMD_AT_ARM: &'static str = "sim/autopilot/autothrottle_arm";
    /// "All modes off, remain armed."
    pub const CMD_AT_OFF: &'static str = "sim/autopilot/autothrottle_off";
    /// The MCP speed window (the 737's glareshield script reads this one).
    pub const SPEED_WINDOW: &'static str = "sim/cockpit2/autopilot/airspeed_dial_kts_mach";

    /// The stock names the declarations rely on, for the profile probe
    /// ([`profile_references`]): the profile says THAT the airframe has an
    /// item, the names live here and in `takeoff_config`, which uses these.
    pub fn references(&self) -> Vec<&'static str> {
        let mut out = Vec::new();
        if self.arm_spoilers || self.takeoff_speedbrake_down {
            out.push(Self::SPEEDBRAKE);
        }
        if self.autobrake_rto || self.landing_autobrake_level.is_some() {
            out.push(Self::AUTOBRAKE);
        }
        if self.flight_directors {
            out.extend([Self::FD_MODE, Self::CMD_FD_ON]);
        }
        if self.autothrottle_armed {
            out.extend([Self::AT_MODE, Self::CMD_AT_ARM, Self::CMD_AT_OFF]);
        }
        if self.v2_kts.is_some() {
            out.push(Self::SPEED_WINDOW);
        }
        out
    }
}

/// One more knob a transponder needs set (see [`TransponderModeKnob::also`]):
/// read-only, stepped by its commands to `want`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompanionKnob {
    pub dataref: String,
    pub want: i64,
    pub up_command: String,
    pub down_command: String,
}

/// Custom on/off commands for the electric boost pumps, for airframes where the
/// universal `fuel_pump_on` array is a no-op (see
/// [`AircraftProfile::fuel_pump_commands`]). All `on_commands` fire for "pump
/// on" and all `off_commands` for "pump off" — a multi-pump aircraft (the Twin
/// Otter's FWD + AFT) switches them together, matching how the checklist says
/// "boost pumps ON". `readback` datarefs (each may be `name[i]`) confirm.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PumpCommands {
    pub on_commands: Vec<String>,
    pub off_commands: Vec<String>,
    /// Switch datarefs written to 1/0 directly, for systems whose only control
    /// is a TOGGLE command. Firing a toggle as "on" turns the system off
    /// whenever it was already on, which is worse than having no tool at all —
    /// the Citation X's avionics master is the case (`cmd_avionics_toggle` is
    /// the only command its scripts create). Written after any commands above.
    #[serde(default)]
    pub write_datarefs: Vec<String>,
    #[serde(default)]
    pub readback: Vec<String>,
}

/// An add-on's own read of a checklist state (`checklist_states:`): every
/// dataref (`name` or `name[i]`) must hold the value.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StateRead {
    pub datarefs: Vec<String>,
    /// Value -> a switch position, or an inclusive `[lo, hi]`.
    #[serde(flatten)]
    pub values: std::collections::BTreeMap<String, StateValue>,
}

/// One value of a [`StateRead`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum StateValue {
    At(f64),
    Range([f64; 2]),
}
