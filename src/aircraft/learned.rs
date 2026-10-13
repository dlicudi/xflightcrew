//! What the airframe's own flight MEASURED (`xfc identify`, 2026-10-13): how
//! each axis answers its control and what the throttle buys, stored per
//! airframe where the pilot controller (`src/pilot`) reads it. And what its
//! `.acf` says about its geometry and engines.
//!
//! **Why a file, and why this is not a state dump.** CLAUDE.md: "Only
//! settings persist; never a state dump: persisted facts outlive their
//! context". A learned response is not the state of a flight: it is a
//! property of the airframe's flight MODEL, the same kind of fact a profile
//! line declares (and replaces lines that were typed into profiles: the
//! flare's elevator limits, its ground pull, its power schedule). Its
//! context is the `.acf`, so the file is KEYED to it and cannot outlive it:
//! - named after the `.acf`'s path ([`Learned::path_for`]),
//! - fingerprinted on the `.acf`'s whole text: an aircraft update, or a
//!   Plane Maker edit, changes the fingerprint and the file is not believed
//!   (said, and the fleet's priors fly until it is measured again),
//! - versioned ([`VERSION`]): a file from another schema is not read,
//! - written only by an identification flight, never by a flight's loop,
//!   and kept under `profiles/learned/` beside the profiles, reviewed and
//!   committed like them. Written to the data folder's
//!   (`sim::paths::learned_write_dir`), read from it first and then from
//!   the shipped ones (`learned_read_dirs`): one folder in a checkout.
//!
//! The online refinement while flying (`pilot::bank::RollResponse`) is per
//! segment and never written back.

use serde::{Deserialize, Serialize};

/// The schema this build reads and writes.
pub const VERSION: u32 = 1;

/// One axis' response, at the airspeeds it was measured at: the rate `x`
/// answers the control `u`, `delay_s` late, as `x' = b u - damping x -
/// stiffness s`, where `s` is the airflow angle the axis turns (alpha for
/// the pitch, X-Plane's beta for the yaw, none for the roll) and the path
/// follows it back as `s' = x - path s` (`pilot::response`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Response {
    /// Angular acceleration per unit of control, deg/s^2.
    pub b: f64,
    /// The rate's own decay, 1/s.
    pub damping: f64,
    /// The moment against the airflow angle, deg/s^2 per degree (positive
    /// restoring: the pitch's alpha spring, the yaw's weathercock).
    #[serde(default)]
    pub stiffness: f64,
    /// How fast the flight path follows that angle, 1/s.
    #[serde(default)]
    pub path: f64,
    /// The moment the OTHER axis' airflow angle gives this one, deg/s^2 per
    /// degree: the roll's dihedral against X-Plane's beta (a crosswind's
    /// wing-low slip leans on it). Zero where not measured.
    #[serde(default)]
    pub cross: f64,
    /// The control's transport delay, s.
    pub delay_s: f64,
    /// The airspeeds it was measured at, kt.
    pub ias_kts: f64,
    pub tas_kts: f64,
}

impl Response {
    /// The same airframe at another airspeed: moments go with dynamic
    /// pressure (IAS^2), the damping and the path's answer with density
    /// times speed (IAS^2 / TAS), so the rate a unit holds goes with TAS
    /// (ArduPlane's SCALING_SPEED / airspeed on the output is this law).
    /// Unread speeds keep the measurement.
    pub fn at(&self, ias_kts: f64, tas_kts: f64) -> Response {
        if !(ias_kts.is_finite() && tas_kts.is_finite() && ias_kts > 20.0 && tas_kts > 20.0) {
            return *self;
        }
        let q = (ias_kts / self.ias_kts).powi(2);
        let rho_v = q * self.tas_kts / tas_kts;
        Response {
            b: self.b * q,
            damping: self.damping * rho_v,
            stiffness: self.stiffness * q,
            path: self.path * rho_v,
            cross: self.cross * q,
            ias_kts,
            tas_kts,
            ..*self
        }
    }

    /// The rate a unit of control holds once the axis has settled, deg/s
    /// per unit: the effectiveness over the damping and, where the axis
    /// turns the airflow, the spring the path leaves standing (`stiffness /
    /// path`). The 172's pitch fit holds 20 deg/s; its roll, 18.
    pub fn rate_gain(&self) -> f64 {
        let spring = if self.path > 1e-6 { self.stiffness / self.path } else { 0.0 };
        self.b / (self.damping + spring).max(0.05)
    }

    /// Whether every number is one a flying aircraft could have.
    pub fn sane(&self) -> bool {
        self.b.is_finite()
            && self.b > 0.5
            && self.b < 2_000.0
            && self.damping.is_finite()
            && (0.02..=20.0).contains(&self.damping)
            && self.stiffness.is_finite()
            && (-5.0..=200.0).contains(&self.stiffness)
            && self.path.is_finite()
            && (0.0..=10.0).contains(&self.path)
            && self.cross.is_finite()
            && (-50.0..=50.0).contains(&self.cross)
            && (0.0..=1.0).contains(&self.delay_s)
            && self.ias_kts > 20.0
            && self.tas_kts > 20.0
    }
}

/// One axis' identification and how well it explained the flight.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Fit {
    pub response: Response,
    /// The share of the rate's derivative the model explained (median).
    pub r2: f64,
    /// Inputs fitted and believed.
    pub segments: usize,
}

/// What the throttle buys in energy, measured.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EnergyFit {
    /// Energy rate per unit of lever, fpm (TECS's `K_thr2STE`).
    pub fpm_per_unit: f64,
    /// The engine's lag behind the lever, s.
    pub spool_s: f64,
    pub ias_kts: f64,
    pub r2: f64,
}

/// The conditions an identification was flown in.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Conditions {
    pub ias_kts: f64,
    pub alt_ft: f64,
    pub flap: f64,
}

/// One airframe's measurement flight, as stored.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Learned {
    pub version: u32,
    /// The `.acf`, relative to the X-Plane install.
    pub acf: String,
    /// [`fingerprint`] of the `.acf`'s text when it was flown.
    pub acf_fingerprint: String,
    /// The day it was flown (UTC, YYYY-MM-DD).
    pub measured_on: String,
    pub conditions: Conditions,
    #[serde(default)]
    pub pitch: Option<Fit>,
    #[serde(default)]
    pub roll: Option<Fit>,
    #[serde(default)]
    pub yaw: Option<Fit>,
    #[serde(default)]
    pub energy: Option<EnergyFit>,
}

/// FNV-1a 64 over the `.acf`'s text: any change to the airframe's model
/// retires what was learned on it.
pub fn fingerprint(acf_text: &str) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in acf_text.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    format!("fnv64:{h:016x}")
}

/// Why a learned file was not believed.
#[derive(Debug, Clone, PartialEq)]
pub enum Refused {
    /// None was flown for this `.acf`.
    Missing,
    /// It does not parse, or is another schema's.
    Unreadable(String),
    /// The `.acf` has changed since.
    Stale,
}

impl Learned {
    /// The file's name for an `.acf` path: its letters and digits, the
    /// rest `_`.
    pub fn file_name_for(acf_rel: &str) -> String {
        let slug: String = acf_rel
            .trim()
            .to_lowercase()
            .chars()
            .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
            .collect();
        format!("{}.json", slug.trim_matches('_'))
    }

    /// Where `xfc identify` writes the file for an `.acf` path.
    pub fn path_for(acf_rel: &str) -> std::path::PathBuf {
        sim::paths::learned_write_dir().join(Self::file_name_for(acf_rel))
    }

    /// The learned file's text checked against this `.acf`. Pure.
    pub fn parse_for(text: &str, acf_text: &str) -> Result<Learned, Refused> {
        let l: Learned = serde_json::from_str(text).map_err(|e| Refused::Unreadable(e.to_string()))?;
        if l.version != VERSION {
            return Err(Refused::Unreadable(format!("version {} (this build reads {VERSION})", l.version)));
        }
        if l.acf_fingerprint != fingerprint(acf_text) {
            return Err(Refused::Stale);
        }
        Ok(l)
    }

    /// What was learned for the `.acf` at `acf_rel`, whose text is
    /// `acf_text`: the user's file first, then the shipped one.
    pub fn load_for(acf_rel: &str, acf_text: &str) -> Result<Learned, Refused> {
        let name = Self::file_name_for(acf_rel);
        sim::paths::learned_read_dirs()
            .iter()
            .find_map(|dir| std::fs::read_to_string(dir.join(&name)).ok())
            .map_or(Err(Refused::Missing), |text| {
                Self::parse_for(&text, acf_text)
            })
    }

    /// Written whole, then renamed into place.
    pub fn save(&self) -> std::io::Result<std::path::PathBuf> {
        let path = Self::path_for(&self.acf);
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let tmp = path.with_extension("json.tmp");
        std::fs::write(&tmp, serde_json::to_string_pretty(self).map_err(std::io::Error::other)? + "\n")?;
        std::fs::rename(&tmp, &path)?;
        Ok(path)
    }
}

/// The live airframe's `.acf` model, read by the detector once it has the
/// file: the geometry, the engines and what was learned.
#[derive(Debug, Clone, PartialEq)]
pub struct AcfModel {
    pub geometry: GroundGeometry,
    pub engine: Option<EngineClass>,
    /// What an identification learned on THIS `.acf`; `Err` says why not.
    pub learned: Result<Learned, Refused>,
}

impl AcfModel {
    /// Everything the `.acf` at `acf_rel` (text `acf_text`) says, and what
    /// was learned flying it.
    pub fn read(acf_rel: &str, acf_text: &str) -> AcfModel {
        AcfModel {
            geometry: GroundGeometry::from_acf(acf_text),
            engine: EngineClass::from_acf(acf_text),
            learned: Learned::load_for(acf_rel, acf_text),
        }
    }
}

/// The detector's `.acf` model for the live airframe (a copy-on-write
/// update of its facts; `flight_state::publish_airframe` resets it to
/// `None` on every swap).
pub fn publish_acf_model(m: Option<AcfModel>) {
    super::update_airframe_facts(|a| a.acf_model = m);
}

/// The live airframe's `.acf` model, `None` until the detector read it.
pub fn acf_model() -> Option<AcfModel> {
    super::airframe_facts().acf_model.clone()
}

/// What kind of engine answers the lever, by how fast it spools down from
/// approach power: a piston at once, a turboprop's governor in about a
/// second, a turbofan's spools in two or three.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineClass {
    Piston,
    Turboprop,
    Jet,
}

impl EngineClass {
    /// Seconds from approach power to idle thrust.
    pub fn spool_s(self) -> f64 {
        match self {
            EngineClass::Piston => 0.3,
            EngineClass::Turboprop => 1.0,
            EngineClass::Jet => 2.5,
        }
    }

    /// From the `.acf`'s first engine (`_engn/0/_type`): RCP_* a piston,
    /// TRB_* a turboprop, anything else a jet. `None` when unread.
    pub fn from_acf(acf_text: &str) -> Option<EngineClass> {
        let t = acf_value(acf_text, "_engn/0/_type")?;
        Some(if t.starts_with("RCP") {
            EngineClass::Piston
        } else if t.starts_with("TRB") {
            EngineClass::Turboprop
        } else {
            EngineClass::Jet
        })
    }
}

/// The attitudes the airframe's geometry allows on the runway, degrees,
/// from its `.acf`: the gear's contact points on extended struts and the
/// fuselage's skin; and how far the nose reaches ahead of the point the
/// sim positions the aircraft by.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundGeometry {
    /// Pitch at which the nosewheel and the mains touch together: below
    /// it the nosewheel lands first. `None` on a tailwheel.
    pub level_deg: Option<f64>,
    /// Pitch at which the fuselage's lowest aft point meets the runway
    /// with the mains on it: the tail strike.
    pub tail_strike_deg: Option<f64>,
    /// The fuselage's foremost point ahead of the aircraft's origin,
    /// metres. X-Plane places an aircraft by its default centre of gravity
    /// (`acf/_cgZ`, feet aft of the `.acf`'s reference point), not by its
    /// middle: the stock 737-800's nose is 18.1 m ahead of it, the A330's
    /// 29.6 m, the 172's 1.9 m. `None` without a fuselage or a CG.
    pub nose_m: Option<f64>,
}

/// Margins kept inside the geometry, degrees: the mains first by a degree,
/// and two short of the tail (struts compress, the runway is not flat).
pub const MAINS_FIRST_MARGIN_DEG: f64 = 1.0;
pub const TAIL_MARGIN_DEG: f64 = 2.0;

fn acf_value<'a>(acf_text: &'a str, key: &str) -> Option<&'a str> {
    acf_text.lines().find_map(|l| {
        let mut it = l.split_whitespace();
        (it.next()? == "P" && it.next()? == key).then(|| it.next()).flatten()
    })
}

impl GroundGeometry {
    /// Read from the `.acf` (`P key value` lines; lengths in feet, z aft
    /// positive, y up): each gear's contact point is its attachment, the
    /// strut's extended length down its angles and the tyre's radius. The
    /// mains are the aft-most gear; a nosewheel is a gear ahead of them; the
    /// tail strike is the smallest pivot angle about the mains of any point
    /// of the fuselage (`_body/0`) aft of them. Pure.
    pub fn from_acf(acf_text: &str) -> GroundGeometry {
        let mut vals: std::collections::HashMap<&str, f64> = std::collections::HashMap::new();
        for l in acf_text.lines() {
            let mut it = l.split_whitespace();
            if it.next() != Some("P") {
                continue;
            }
            let (Some(k), Some(v)) = (it.next(), it.next()) else { continue };
            if k.starts_with("_gear/") || k.starts_with("_body/0/") || k == "acf/_cgZ" {
                if let Ok(v) = v.parse::<f64>() {
                    vals.insert(k, v);
                }
            }
        }
        // The nose: the fuselage's foremost section point, ahead of the CG
        // the sim places the aircraft by.
        let nose_z = vals
            .iter()
            .filter(|(k, _)| k.starts_with("_body/0/_geo_xyz/") && k.ends_with(",2"))
            .map(|(_, z)| z + vals.get("_body/0/_part_z").copied().unwrap_or(0.0))
            .min_by(f64::total_cmp);
        let nose_m = vals
            .get("acf/_cgZ")
            .zip(nose_z)
            .map(|(cg, nz)| (cg - nz) * 0.3048)
            .filter(|n| *n > 0.0);
        // Gear contacts: (z, y, braked).
        let mut gears: Vec<(f64, f64, bool)> = Vec::new();
        for i in 0..16 {
            let g = |f: &str| vals.get(format!("_gear/{i}/{f}").as_str()).copied();
            let (Some(y), Some(z), Some(len)) = (g("_gear_y"), g("_gear_z"), g("_leg_len")) else { continue };
            if len <= 0.0 {
                continue;
            }
            let lon = g("_lonE").unwrap_or(0.0).to_radians();
            let lat = g("_latE").unwrap_or(0.0).to_radians();
            let r = g("_tire_radius").unwrap_or(0.0);
            let braked = g("_gear_brakes").unwrap_or(0.0) > 0.5;
            gears.push((z + len * lon.sin(), y - len * lon.cos() * lat.cos() - r, braked));
        }
        // The mains are the braked gear (else the aft-most); a gear behind
        // them is a tailwheel, which has no nosewheel to protect.
        let mains = gears.iter().filter(|g| g.2).max_by(|a, b| a.0.total_cmp(&b.0));
        let Some(&(zm, ym, _)) = mains.or_else(|| gears.iter().max_by(|a, b| a.0.total_cmp(&b.0))) else {
            return GroundGeometry { level_deg: None, tail_strike_deg: None, nose_m };
        };
        let tailwheel = gears.iter().any(|g| g.0 > zm + 2.0);
        let nose = gears.iter().filter(|g| g.0 < zm - 2.0).min_by(|a, b| a.0.total_cmp(&b.0));
        let level_deg = nose
            .filter(|_| !tailwheel)
            .map(|&(zn, yn, _)| ((yn - ym) / (zn - zm)).atan().to_degrees());
        // The fuselage: its sections' points, offset by the part's place.
        let (py, pz) = (
            vals.get("_body/0/_part_y").copied().unwrap_or(0.0),
            vals.get("_body/0/_part_z").copied().unwrap_or(0.0),
        );
        let mut strike: Option<f64> = None;
        for (k, v) in &vals {
            let Some(rest) = k.strip_prefix("_body/0/_geo_xyz/") else { continue };
            let idx: Vec<&str> = rest.split(',').collect();
            if idx.len() != 3 || idx[2] != "1" {
                continue;
            }
            let zk = format!("_body/0/_geo_xyz/{},{},2", idx[0], idx[1]);
            let Some(z) = vals.get(zk.as_str()) else { continue };
            let (y, z) = (v + py, z + pz);
            if z > zm + 1.0 {
                let a = ((y - ym) / (z - zm)).atan().to_degrees();
                if a > 0.0 {
                    strike = Some(strike.map_or(a, |s: f64| s.min(a)));
                }
            }
        }
        GroundGeometry { level_deg, tail_strike_deg: strike.filter(|_| !tailwheel), nose_m }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_learned_file_is_keyed_to_its_acf() {
        let acf = "P acf/_name x\n";
        let l = Learned {
            version: VERSION,
            acf: "Aircraft/Laminar Research/Boeing 737-800/b738.acf".into(),
            acf_fingerprint: fingerprint(acf),
            measured_on: "2026-10-13".into(),
            conditions: Conditions { ias_kts: 160.0, alt_ft: 5000.0, flap: 0.625 },
            pitch: None,
            roll: Some(Fit {
                response: Response { b: 47.0, damping: 1.9, stiffness: 0.0, path: 0.0, cross: 6.7, delay_s: 0.0, ias_kts: 167.0, tas_kts: 180.0 },
                r2: 0.9,
                segments: 2,
            }),
            yaw: None,
            energy: None,
        };
        let text = serde_json::to_string(&l).unwrap();
        assert_eq!(Learned::parse_for(&text, acf), Ok(l.clone()));
        assert_eq!(Learned::parse_for(&text, "P acf/_name y\n"), Err(Refused::Stale), "an edited .acf retires it");
        let old = text.replace("\"version\":1", "\"version\":0");
        assert!(matches!(Learned::parse_for(&old, acf), Err(Refused::Unreadable(_))));
        assert_eq!(
            Learned::path_for("Aircraft/Laminar Research/Boeing 737-800/b738.acf"),
            sim::paths::learned_write_dir()
                .join("aircraft_laminar_research_boeing_737_800_b738_acf.json")
        );
    }

    /// The stock 172 and 737 `.acf` gear and fuselage, as the files carry
    /// them (2026-10-13): the 172 sits about level on its gear and the
    /// 737-800's tail meets the runway near 11 degrees (Boeing's figure for
    /// the -800 on extended struts).
    #[test]
    fn the_geometry_comes_from_the_gear_and_the_skin() {
        let c172 = "P _gear/0/_gear_y -1.42\nP _gear/0/_gear_z -0.6\nP _gear/0/_leg_len 2.2\nP _gear/0/_lonE 14\nP _gear/0/_tire_radius 0.62\n\
                    P _gear/1/_gear_y -1.85\nP _gear/1/_gear_z 4.6\nP _gear/1/_leg_len 3.0\nP _gear/1/_latE -57\nP _gear/1/_lonE 5\nP _gear/1/_tire_radius 0.62\n";
        let g = GroundGeometry::from_acf(c172);
        let level = g.level_deg.expect("a nosewheel");
        assert!((-3.0..4.0).contains(&level), "{g:?}");
        let b738 = "P _gear/0/_gear_y -5.57\nP _gear/0/_gear_z 13.59\nP _gear/0/_leg_len 4.5\nP _gear/0/_tire_radius 1.14\n\
                    P _gear/1/_gear_y -2.44\nP _gear/1/_gear_z 64.08\nP _gear/1/_leg_len 7.6\nP _gear/1/_lonE -4\nP _gear/1/_tire_radius 1.85\n\
                    P _body/0/_part_y 0\nP _body/0/_part_z 0\n\
                    P _body/0/_geo_xyz/18,9,1 -1.5\nP _body/0/_geo_xyz/18,9,2 112.0\n\
                    P _body/0/_geo_xyz/19,9,1 1.5\nP _body/0/_geo_xyz/19,9,2 125.0\n";
        let g = GroundGeometry::from_acf(b738);
        let tail = g.tail_strike_deg.expect("a tail");
        assert!((9.0..13.0).contains(&tail), "{g:?}");
        assert_eq!(GroundGeometry::from_acf("").tail_strike_deg, None);
    }

    /// The nose reaches ahead of the CG the sim places the aircraft by: the
    /// stock 737-800's fuselage runs from z 0 to 124.8 ft with its CG at
    /// 59.5 ft (the installed `.acf`, 2026-10-13), so the nose is 18.1 m
    /// ahead; without a CG there is no reading, never a guess.
    #[test]
    fn the_nose_is_measured_from_the_cg() {
        let b738 = "P acf/_cgZ 59.5\nP _body/0/_part_z 0\n\
                    P _body/0/_geo_xyz/0,0,1 0.0\nP _body/0/_geo_xyz/0,0,2 -0.017\n\
                    P _body/0/_geo_xyz/19,9,1 1.5\nP _body/0/_geo_xyz/19,9,2 124.84\n";
        let nose = GroundGeometry::from_acf(b738).nose_m.expect("a nose");
        assert!((nose - 18.14).abs() < 0.05, "{nose}");
        // A fuselage part placed aft of the reference point moves its nose.
        let moved = b738.replace("_part_z 0", "_part_z 10");
        let nose = GroundGeometry::from_acf(&moved).nose_m.expect("a nose");
        assert!((nose - 15.09).abs() < 0.05, "{nose}");
        assert_eq!(GroundGeometry::from_acf(&b738.replace("P acf/_cgZ 59.5\n", "")).nose_m, None);
    }
}
