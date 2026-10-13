//! Profiles loaded and matched, and the one snapshot of the airframe's facts.

use super::*;

/// Load a profile by id: prefer `profiles/<id>.yaml`, else the built-in.
pub fn load(id: &str) -> Option<AircraftProfile> {
    read_profile(id, true)
}

/// [`load`], with or without the "loaded from YAML" log line (the matcher
/// sweep reads every profile and would print one per file).
pub(super) fn read_profile(id: &str, log: bool) -> Option<AircraftProfile> {
    let path = format!("profiles/{id}.yaml");
    match std::fs::read_to_string(&path) {
        Ok(text) => match serde_yaml::from_str::<AircraftProfile>(&text) {
            Ok(profile) => {
                if log {
                    tracing::info!(path = %path, "loaded aircraft profile from YAML");
                }
                Some(profile)
            }
            Err(e) => {
                tracing::error!("failed to parse {path}: {e:#}; falling back to built-in");
                builtin(id)
            }
        },
        Err(_) => builtin(id), // no YAML file — use the built-in definition
    }
}

/// What a profile says it is FOR (the `match:` block). Every key given must
/// hit; `icao` is required, and a profile with nothing else is a type-wide
/// catch-all. Each list hits when ANY of its entries does, compared
/// case-insensitively, the path with backslashes as slashes.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ProfileMatch {
    /// `sim/aircraft/view/acf_ICAO` values.
    #[serde(default)]
    pub icao: Vec<String>,
    /// Substrings of `sim/aircraft/view/acf_relative_path` (the model's
    /// folder and file, relative to the install).
    #[serde(default)]
    pub acf: Vec<String>,
    /// Substrings of `sim/aircraft/view/acf_author`.
    #[serde(default)]
    pub author: Vec<String>,
    /// Substrings of `sim/aircraft/view/acf_descrip`.
    #[serde(default)]
    pub description: Vec<String>,
}

impl ProfileMatch {
    /// Does this profile claim the ICAO type at all?
    pub fn claims_type(&self, icao: &str) -> bool {
        let icao = icao.trim();
        !icao.is_empty()
            && self
                .icao
                .iter()
                .any(|t| t.trim().eq_ignore_ascii_case(icao))
    }
}

/// Every profile's `match:` block, as `(id, matcher)` sorted by id: the
/// YAMLs in `profiles/` (read the way [`load`] reads them), and a built-in
/// whose YAML is missing. A profile without a block is left out.
pub fn profile_matchers() -> Vec<(String, ProfileMatch)> {
    let mut ids: Vec<String> = std::fs::read_dir("profiles")
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let path = e.path();
            (path.extension().is_some_and(|x| x == "yaml"))
                .then(|| path.file_stem()?.to_str().map(str::to_string))?
        })
        .collect();
    for b in ["cessna_172sp", "cirrus_sr22"] {
        if !ids.iter().any(|i| i == b) {
            ids.push(b.to_string());
        }
    }
    ids.sort();
    ids.into_iter()
        .filter_map(|id| {
            let p = read_profile(&id, false)?;
            Some((p.id, p.matcher?))
        })
        .collect()
}

/// The profile for a TYPE, not a model: a flight plan's ICAO (SimBrief
/// `aircraft/icaocode`) names no model, so any profile whose `match.icao`
/// holds it answers (the first by id). For type-level questions only (jet
/// or helicopter, from a plan); the loaded aircraft is resolved by
/// `detect::resolve`, never by this.
pub fn profile_for_type(icao: &str) -> Option<AircraftProfile> {
    profile_matchers()
        .into_iter()
        .find(|(_, m)| m.claims_type(icao))
        .and_then(|(id, _)| read_profile(&id, true))
}

/// The universal thrust-lever dataref: what every airframe used until one
/// didn't.
pub const STOCK_THROTTLE: &str = "sim/cockpit2/engine/actuators/throttle_ratio_all";

/// Everything the crew reads per AIRFRAME, as one immutable snapshot
/// (refactor audit 2026-10-04, phase 3 item 3). Each field was a static of
/// its own, set piecemeal on a swap, so a reader between two of those writes
/// saw half of each airframe, and a field a swap forgot to write outlived its
/// airframe (`hover::ROTOR`, the S-76's rotor read on the next aeroplane).
/// Now `flight_state::publish_airframe` builds the whole struct from the
/// profile with a literal naming EVERY field (one forgotten does not
/// compile) and [`install_airframe_facts`] swaps it in at once. The `.acf`
/// facts the detector reads later (the castoring nose, the gear warning)
/// are copy-on-write updates of the same snapshot ([`update_airframe_facts`]).
///
/// Every accessor keeps its name and reads its field here (`speed_floor_kts`,
/// `flap_limits`, `procedure::climb_fpm`, `hover::collective_dref`...), so
/// callers did not change. The fields' types live in this module because it
/// compiles into the lib as well as the bin.
#[derive(Debug, Clone)]
pub struct AirframeFacts {
    /// The whole profile, for the consumers that need the tool surface.
    pub profile: Option<std::sync::Arc<AircraftProfile>>,
    /// (kg per gallon, spoken unit) ([`fuel_unit_for`]).
    pub fuel_unit: (f64, FuelSpoken),
    /// The thrust lever THIS airframe obeys, `None` the stock one: a stale
    /// override would aim the next aircraft's power at a dataref it does
    /// not use, the same shape as the transponder knob and the radio-power
    /// latch.
    pub throttle_dref: Option<String>,
    /// The `.acf`'s castoring nose (the detector's), false until read.
    pub nose_castors: bool,
    /// The `.acf`'s free castoring nose (the detector's), false until read.
    pub nose_free: bool,
    /// Vlo.
    pub gear_limit_kts: Option<f64>,
    /// Each flap detent's `(ratio, Vfe)` ([`flap_limits_for`]).
    pub flap_limits: Vec<(f64, Option<f64>)>,
    /// Nominal bus volts, `None` the 24 volt default.
    pub bus_volts_nominal: Option<f64>,
    /// 1.2 Vs clean ([`speed_floor_for`]).
    pub speed_floor_kts: Option<f64>,
    /// Vne and the high Vmo ([`SpeedLimits::new`]).
    pub speed_limits: SpeedLimits,
    /// 1.2 Vso, else the clean floor ([`approach_floor_for`]).
    pub approach_floor_kts: Option<f64>,
    /// The per-notch stall table ([`stall_curve_for`]).
    pub stall_curve: Option<StallCurve>,
    /// The approach thrust floor ([`approach_thrust_min_for`]).
    pub approach_thrust_min: Option<f64>,
    /// The roll's rudder authority, `None` the default roll law.
    pub roll_rudder: Option<RollRudderProfile>,
    /// The book landing distance, metres ([`landing_distance_m_for`]).
    pub landing_distance_m: Option<f64>,
    /// The enroute vertical rates (`procedure::vs_limits_for`); `None` is
    /// the GA figure, for a profile that declares none.
    pub vs_limits: Option<VsLimits>,
    /// The gear-warning envelope, the profile's or the `.acf`'s (the
    /// detector's, over a silent profile).
    pub gear_warning: Option<GearWarning>,
    /// A jet's declared climb-out ([`ClimbOut::from_profile`]).
    pub climb_out: Option<ClimbOut>,
    /// The manual takeoff power limit.
    pub takeoff_power: Option<TakeoffPowerProfile>,
    /// `takeoff_config:`.
    pub takeoff_config: Option<TakeoffConfigProfile>,
    /// `ap_altitude_capture`.
    pub ap_altitude_capture: bool,
    /// A helicopter's rotor: (rpm dataref, governed rpm, collective
    /// dataref) (`hover::rotor_facts`); `None` on anything else.
    pub rotor: Option<(String, f64, String)>,
    /// The seat belt switch ([`SignsSwitch::for_profile`]).
    pub signs: Option<SignsSwitch>,
    /// `cabin_air:`.
    pub cabin_air: Option<CabinAirProfile>,
    /// The transponder's add-on mode knob: never merely set when an
    /// airframe happens to have one (a published-once flag that outlived an
    /// airframe change is how a cold-and-dark 172 silently killed a Twin
    /// Otter's radio for a whole session, 2026-07-27).
    pub mode_knob: Option<TransponderModeKnob>,
    /// The landing and taxi lights ([`Exterior::from_lights`]).
    pub exterior: Exterior,
    /// The altimeter fast paths, `None` the stock knob alone.
    pub altimeter: Option<AltimeterProfile>,
    /// The `.acf`'s model (the detector's): its ground geometry, its
    /// engines and what an identification flight learned flying it
    /// ([`learned::AcfModel`]). `None` until read: a new airframe starts
    /// unmeasured.
    pub acf_model: Option<learned::AcfModel>,
}

impl Default for AirframeFacts {
    /// Before any airframe: every field what a silent profile gives.
    fn default() -> Self {
        AirframeFacts {
            profile: None,
            fuel_unit: (AVGAS_KG_PER_GAL, FuelSpoken::Gallons),
            throttle_dref: None,
            nose_castors: false,
            nose_free: false,
            gear_limit_kts: None,
            flap_limits: Vec::new(),
            bus_volts_nominal: None,
            speed_floor_kts: None,
            speed_limits: SpeedLimits::new(None, None),
            approach_floor_kts: None,
            stall_curve: None,
            approach_thrust_min: None,
            roll_rudder: None,
            landing_distance_m: None,
            vs_limits: None,
            gear_warning: None,
            climb_out: None,
            takeoff_power: None,
            takeoff_config: None,
            ap_altitude_capture: false,
            rotor: None,
            signs: None,
            cabin_air: None,
            mode_knob: None,
            exterior: Exterior::default(),
            altimeter: None,
            acf_model: None,
        }
    }
}

/// The live airframe's facts: replaced whole on a swap, copied on write for
/// a late `.acf` fact. Readers clone the `Arc` and drop the lock.
pub(super) static AIRFRAME_FACTS: std::sync::LazyLock<std::sync::RwLock<std::sync::Arc<AirframeFacts>>> =
    std::sync::LazyLock::new(|| {
        std::sync::RwLock::new(std::sync::Arc::new(AirframeFacts::default()))
    });

/// The live airframe's facts.
pub fn airframe_facts() -> std::sync::Arc<AirframeFacts> {
    AIRFRAME_FACTS
        .read()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
}

/// Install a new airframe's facts, all at once (`flight_state::publish_airframe`).
pub fn install_airframe_facts(f: AirframeFacts) {
    #[cfg(test)]
    THROTTLE_DREF_T.with(|t| *t.borrow_mut() = f.throttle_dref.clone());
    *AIRFRAME_FACTS.write().unwrap_or_else(|e| e.into_inner()) = std::sync::Arc::new(f);
}

/// Change some of the live airframe's facts: the detector's `.acf` reads,
/// and the tests' single-field publishes. Copy on write under the lock, so
/// two updates of different fields never lose one another.
pub fn update_airframe_facts(change: impl FnOnce(&mut AirframeFacts)) {
    let mut g = AIRFRAME_FACTS.write().unwrap_or_else(|e| e.into_inner());
    change(std::sync::Arc::make_mut(&mut g));
}

/// An airframe's enroute vertical rates, fpm (`procedure::vs_limits_for`
/// derives them; `procedure::climb_fpm` and the rest read them).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VsLimits {
    pub climb_fpm: f64,
    pub descent_fpm: f64,
    pub best_climb_fpm: Option<f64>,
}

/// A jet's climb-out, declared by the profile (`acceleration_height_agl_ft`
/// and `best_climb_fpm`) and republished on every swap; flown by
/// `autotakeoff`.
///
/// EGPH 24, 2026-09-21, the 737: every airframe climbed out on one capped
/// 1,500 fpm dial at takeoff thrust. That is a 172's whole climb and a fifth
/// of a 737's excess power, so the rest went into speed: 185 kt at 340 ft,
/// the flaps running up on speed alone at 400, 500 and 740 ft, 240 kt through
/// 1,000 ft. A crew holds V2 + 20 to the acceleration height by PITCH, then
/// lowers the nose and cleans up on the schedule.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClimbOut {
    pub accel_agl_ft: f64,
    /// The hardest the speed hold may climb, fpm.
    pub max_fpm: f64,
}

impl ClimbOut {
    pub fn from_profile(p: &AircraftProfile) -> Option<Self> {
        let r = p.reference.as_ref()?;
        let accel_agl_ft = r
            .acceleration_height_agl_ft
            .filter(|h| h.is_finite() && *h > 0.0)?;
        Some(ClimbOut {
            accel_agl_ft,
            max_fpm: r.best_climb_fpm.filter(|f| f.is_finite()).unwrap_or(0.0),
        })
    }
}

/// How the live airframe's seat belt signs are worked (`tools::signs`): the
/// stock switch, or an add-on's own declared one.
#[derive(Debug, Clone)]
pub enum SignsSwitch {
    Stock,
    Own(SeatBeltSigns),
}

impl SignsSwitch {
    /// The profile's declared switch, else the stock one on an airframe
    /// with a cabin, else none. Pure.
    pub fn for_profile(own: Option<SeatBeltSigns>, has_cabin: bool) -> Option<Self> {
        match own {
            Some(s) => Some(SignsSwitch::Own(s)),
            None if has_cabin => Some(SignsSwitch::Stock),
            None => None,
        }
    }
}

/// The active airframe's landing and taxi lights, the two the ground and
/// climb stages switch (`tools::lights`). Republished on EVERY profile swap,
/// including to an empty set, never merely set when an airframe happens to
/// have them (the transponder knob's lesson, 2026-07-27).
#[derive(Clone, Debug, Default)]
pub struct Exterior {
    pub landing: Option<LightProfile>,
    pub taxi: Option<LightProfile>,
}

impl Exterior {
    /// The landing and taxi lights from a profile's light list. Pure.
    pub fn from_lights(lights: &[LightProfile]) -> Self {
        let pick = |key: &str| {
            lights
                .iter()
                .find(|l| l.key.eq_ignore_ascii_case(key))
                .cloned()
        };
        Exterior {
            landing: pick("landing"),
            taxi: pick("taxi"),
        }
    }
}
