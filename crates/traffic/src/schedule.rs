//! Schedules: who flies when. Generated from the field and a seed
//! ([`generate`]), or read from the pilot's file ([`parse_file`],
//! [`from_file`]); never fetched. A schedule is a list of [`Movement`]s;
//! the bin spawns each at its time through [`Movement::spawn`].
//!
//! Despawn rules live with the phase machine (`aircraft.rs`): a departure
//! is gone [`DEPART_GONE_NM`] out, an arrival [`PARKED_LINGER_S`] after it
//! parks, a circuit aircraft once its laps are flown and it has parked.

use serde::Deserialize;

use crate::aircraft::TrafficAircraft;
pub use crate::aircraft::{DEPART_GONE_NM, PARKED_LINGER_S};
use crate::field::{stand_taken, FieldModel, RunwayEnd};
use crate::kin::{dist_m, project_m, LatLon};
use crate::perf::{class_for_type, Perf, JET_HEAVY, JET_MED, PISTON_SINGLE, TURBOPROP};
use crate::phase::Plan;
use crate::rng::XorShift64;

/// How busy the generated sky is (`XCOPILOT_TRAFFIC_DENSITY`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Density {
    Off,
    Light,
    Normal,
}

impl Density {
    /// `off`, `light` or `normal` (any case); anything else is `None`.
    pub fn parse(s: &str) -> Option<Density> {
        match s.trim().to_ascii_lowercase().as_str() {
            "off" => Some(Density::Off),
            "light" => Some(Density::Light),
            "normal" => Some(Density::Normal),
            _ => None,
        }
    }

    /// Minutes between IFR movements of each kind.
    fn ifr_interval_s(self) -> f64 {
        match self {
            Density::Off => f64::INFINITY,
            Density::Light => 20.0 * 60.0,
            Density::Normal => 10.0 * 60.0,
        }
    }
}

/// A generated schedule covers this much time; the bin generates again.
pub const HORIZON_S: f64 = 3600.0;

/// Nothing spawns in the air this close to our own aircraft.
pub const OWN_CLEAR_NM: f64 = 3.0;

/// An arrival starts this far out.
pub const ENTRY_NM: f64 = 10.0;

/// The shortest runway a jet is scheduled on, and a turboprop.
pub const JET_MIN_RUNWAY_M: f64 = 1800.0;
pub const TURBOPROP_MIN_RUNWAY_M: f64 = 1100.0;

/// What kind of movement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MovementKind {
    Circuit,
    Departure,
    Arrival,
}

/// One scheduled flight.
#[derive(Debug, Clone, PartialEq)]
pub struct Movement {
    /// Seconds after the schedule's start.
    pub t_s: f64,
    pub callsign: String,
    pub spoken: String,
    pub icao_type: String,
    pub kind: MovementKind,
    pub runway: String,
    /// The stand a circuit or a departure starts on.
    pub stand: Option<String>,
    /// Where an arrival starts.
    pub entry: Option<LatLon>,
    pub straight_in: bool,
    pub laps: u32,
    pub sid: Option<String>,
    pub star: Option<String>,
    /// A departure's way out, degrees true.
    pub exit_bearing: f64,
    pub seed: u64,
}

/// What the world looks like at generation or spawn time.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SpawnCtx {
    /// The tower's runway in use; the field's longest when `None`.
    pub runway: Option<String>,
    /// Our own aircraft: no airborne spawn within [`OWN_CLEAR_NM`], and
    /// its stand is occupied.
    pub own: Option<LatLon>,
    /// Everyone else on the ground (stands they stand on are taken).
    pub occupied: Vec<LatLon>,
}

impl SpawnCtx {
    fn taken(&self) -> Vec<LatLon> {
        self.occupied.iter().copied().chain(self.own).collect()
    }
}

/// Why a movement could not spawn now (the bin tries again later).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    /// The field has no such runway.
    NoRunway,
    /// No free stand to start on.
    NoStand,
    /// The entry point is within [`OWN_CLEAR_NM`] of us.
    NearOwn,
}

impl Movement {
    /// The aircraft, as traffic id `id`, or why not now. A circuit or a
    /// departure takes its stand only when free, else the nearest free
    /// one; an arrival never starts within [`OWN_CLEAR_NM`] of us.
    pub fn spawn(
        &self,
        id: u32,
        field: &FieldModel,
        ctx: &SpawnCtx,
    ) -> Result<TrafficAircraft, Refusal> {
        let end = field.end(&self.runway).ok_or(Refusal::NoRunway)?;
        let plan = match self.kind {
            MovementKind::Circuit => Plan::Circuit {
                runway: end.ident.clone(),
                laps: self.laps.max(1),
                left: end.left,
            },
            MovementKind::Departure => Plan::Departure {
                runway: end.ident.clone(),
                sid: self.sid.clone(),
                exit_bearing: self.exit_bearing,
            },
            MovementKind::Arrival => Plan::Arrival {
                entry: self.entry.unwrap_or_else(|| default_entry(&end)),
                star: self.star.clone(),
                runway: end.ident.clone(),
                straight_in: self.straight_in,
            },
        };
        if let Plan::Arrival { entry, .. } = &plan {
            if ctx
                .own
                .is_some_and(|o| dist_m(o, *entry) < OWN_CLEAR_NM * 1852.0)
            {
                return Err(Refusal::NearOwn);
            }
            return TrafficAircraft::at_entry(
                id,
                &self.callsign,
                &self.spoken,
                &self.icao_type,
                plan,
                field,
                self.seed,
            )
            .ok_or(Refusal::NoRunway);
        }
        let taken = ctx.taken();
        let (perf, _) = class_for_type(&self.icao_type);
        let wanted = self
            .stand
            .as_deref()
            .and_then(|n| field.stand(n))
            .filter(|s| !stand_taken(s, &taken));
        let stand = match wanted {
            Some(s) => s.name.clone(),
            None => free_stands(field, perf, &taken)
                .into_iter()
                .next()
                .ok_or(Refusal::NoStand)?,
        };
        TrafficAircraft::at_stand(
            id,
            &self.callsign,
            &self.spoken,
            &self.icao_type,
            plan,
            field,
            &stand,
            self.seed,
        )
        .ok_or(Refusal::NoStand)
    }
}

/// Ten miles out on the extended centreline.
fn default_entry(end: &RunwayEnd) -> LatLon {
    project_m(end.thr, end.course_deg + 180.0, ENTRY_NM * 1852.0)
}

/// The field's default runway: the longest assignable strip's first end,
/// preferring an end with an approach.
pub fn default_runway(field: &FieldModel) -> Option<String> {
    let strip = nav::assignable_runways(&field.airport.runways)
        .max_by(|a, b| a.length_m().total_cmp(&b.length_m()))?;
    [&strip.ident_a, &strip.ident_b]
        .into_iter()
        .find(|id| field.procs.approach_for(id).is_some())
        .or(Some(&strip.ident_a))
        .cloned()
}

/// Stand names a `perf`-class aircraft may use, free of `taken`, in file
/// order. A stand that names its categories must name this class's.
fn free_stands(field: &FieldModel, perf: &Perf, taken: &[LatLon]) -> Vec<String> {
    let cat = category(perf);
    field
        .net
        .stands
        .iter()
        .filter(|s| s.is_suitable())
        .filter(|s| {
            s.accepts.is_empty()
                || s.accepts
                    .iter()
                    .any(|a| a == cat || (cat == "jets" && a == "heavy"))
        })
        .filter(|s| !stand_taken(s, taken))
        .map(|s| s.name.clone())
        .collect()
}

/// apt.dat's stand category for a class.
fn category(perf: &Perf) -> &'static str {
    // By name: a `const` table's address is not one address.
    if perf.name == JET_MED.name || perf.name == JET_HEAVY.name {
        "jets"
    } else if perf.name == TURBOPROP.name {
        "turboprops"
    } else {
        "props"
    }
}

/// A seed for the field: the same field and seed, the same schedule.
fn field_seed(icao: &str, seed: u64) -> u64 {
    icao.bytes().fold(seed ^ 0xCBF2_9CE4_8422_2325, |h, b| {
        (h ^ b as u64).wrapping_mul(0x0100_0000_01B3)
    })
}

/// Generate an hour ([`HORIZON_S`]) of traffic for `field`:
/// - 1 (light) or 1 to 3 (normal) circuit aircraft where the field allows
///   circuits (`allows_circuits != Some(false)`) and has stands;
/// - IFR departures by SID where the field publishes SIDs, and IFR
///   arrivals (straight in where the runway has an approach, else to the
///   downwind) where it publishes approaches or STARs, every 20 (light) or
///   10 (normal) minutes, on a runway long enough for the type;
/// - never on an occupied stand (ours, anyone's in `ctx`, or one already
///   handed out), never an entry within [`OWN_CLEAR_NM`] of us.
pub fn generate(field: &FieldModel, density: Density, seed: u64, ctx: &SpawnCtx) -> Vec<Movement> {
    let mut out = Vec::new();
    if density == Density::Off {
        return out;
    }
    let Some(runway) = ctx.runway.clone().or_else(|| default_runway(field)) else {
        return out;
    };
    let Some(end) = field.end(&runway) else {
        return out;
    };
    let mut rng = XorShift64::new(field_seed(&field.icao, seed));
    let mut taken = ctx.taken();
    let take_stand =
        |perf: &Perf, rng: &mut XorShift64, taken: &mut Vec<LatLon>| -> Option<String> {
            let free = free_stands(field, perf, taken);
            let name = free.get(rng.below(free.len() as u64) as usize)?.clone();
            let s = field.stand(&name)?;
            taken.push((s.lat, s.lon));
            Some(name)
        };
    let prefix = registration_prefix(&field.icao);

    if field.airport.allows_circuits != Some(false) {
        let n = match density {
            Density::Normal => 1 + rng.below(3) as u32,
            _ => 1,
        };
        let mut t = rng.range_f64(30.0, 180.0);
        for _ in 0..n {
            let icao_type = ["C172", "C152", "PA28", "SR22"][rng.below(4) as usize];
            let Some(stand) = take_stand(&PISTON_SINGLE, &mut rng, &mut taken) else {
                break;
            };
            let (callsign, spoken) = registration(prefix, icao_type, &mut rng);
            out.push(Movement {
                t_s: t,
                callsign,
                spoken,
                icao_type: icao_type.into(),
                kind: MovementKind::Circuit,
                runway: end.ident.clone(),
                stand: Some(stand),
                entry: None,
                straight_in: false,
                laps: 2 + rng.below(3) as u32,
                sid: None,
                star: None,
                exit_bearing: 0.0,
                seed: rng.next_u64(),
            });
            t += rng.range_f64(120.0, 420.0);
        }
    }

    let ifr_type = |rng: &mut XorShift64| -> Option<&'static str> {
        if end.length_m >= JET_MIN_RUNWAY_M {
            Some(["A320", "B738", "A319", "AT76"][rng.below(4) as usize])
        } else if end.length_m >= TURBOPROP_MIN_RUNWAY_M {
            Some("AT76")
        } else {
            None
        }
    };
    let every = density.ifr_interval_s();
    let has_stands = field.net.stands.iter().any(|s| s.is_suitable());

    if !field.procs.sids.is_empty() && has_stands {
        let mut t = rng.range_f64(60.0, every);
        while t < HORIZON_S {
            let Some(icao_type) = ifr_type(&mut rng) else {
                break;
            };
            let (perf, _) = class_for_type(icao_type);
            let Some(stand) = take_stand(perf, &mut rng, &mut taken) else {
                break;
            };
            let sid = field.procs.sids[rng.below(field.procs.sids.len() as u64) as usize].clone();
            let (callsign, spoken) = airline(&field.icao, &mut rng);
            out.push(Movement {
                t_s: t,
                callsign,
                spoken,
                icao_type: icao_type.into(),
                kind: MovementKind::Departure,
                runway: end.ident.clone(),
                stand: Some(stand),
                entry: None,
                straight_in: false,
                laps: 0,
                sid: Some(sid),
                star: None,
                exit_bearing: rng.range_f64(0.0, 360.0),
                seed: rng.next_u64(),
            });
            t += every * rng.range_f64(0.8, 1.2);
        }
    }

    let arrivals = !field.procs.approaches.is_empty() || !field.procs.stars.is_empty();
    if arrivals && has_stands {
        let straight_in = field.procs.approach_for(&end.ident).is_some();
        let mut t = rng.range_f64(every * 0.3, every);
        while t < HORIZON_S {
            let Some(icao_type) = ifr_type(&mut rng) else {
                break;
            };
            let Some(entry) = entry_clear_of_own(&end, ctx.own, &mut rng) else {
                t += every;
                continue;
            };
            let star = (!field.procs.stars.is_empty()).then(|| {
                field.procs.stars[rng.below(field.procs.stars.len() as u64) as usize].clone()
            });
            let (callsign, spoken) = airline(&field.icao, &mut rng);
            out.push(Movement {
                t_s: t,
                callsign,
                spoken,
                icao_type: icao_type.into(),
                kind: MovementKind::Arrival,
                runway: end.ident.clone(),
                stand: None,
                entry: Some(entry),
                straight_in,
                laps: 0,
                sid: None,
                star,
                exit_bearing: 0.0,
                seed: rng.next_u64(),
            });
            t += every * rng.range_f64(0.8, 1.2);
        }
    }
    out.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    out
}

/// An entry [`ENTRY_NM`] out, within 50 degrees of the extended
/// centreline, clear of us by [`OWN_CLEAR_NM`]; turned further round when
/// we are in the way, `None` if nowhere is clear.
fn entry_clear_of_own(
    end: &RunwayEnd,
    own: Option<LatLon>,
    rng: &mut XorShift64,
) -> Option<LatLon> {
    let base = end.course_deg + 180.0 + rng.range_f64(-50.0, 50.0);
    (0..8)
        .map(|k| base + k as f64 * 45.0)
        .map(|b| project_m(end.thr, b, ENTRY_NM * 1852.0))
        .find(|p| own.is_none_or(|o| dist_m(o, *p) >= OWN_CLEAR_NM * 1852.0))
}

/// The registration prefix by the field's ICAO region.
fn registration_prefix(icao: &str) -> &'static str {
    match icao.get(..2).unwrap_or("") {
        "EG" => "G",
        "LE" | "GC" => "EC",
        "LF" => "F",
        "ED" | "ET" => "D",
        "LI" => "I",
        "EH" => "PH",
        "EB" => "OO",
        "LS" => "HB",
        "LO" => "OE",
        _ => "N",
    }
}

const PHONETIC: [&str; 26] = [
    "Alpha", "Bravo", "Charlie", "Delta", "Echo", "Foxtrot", "Golf", "Hotel", "India", "Juliett",
    "Kilo", "Lima", "Mike", "November", "Oscar", "Papa", "Quebec", "Romeo", "Sierra", "Tango",
    "Uniform", "Victor", "Whiskey", "X-ray", "Yankee", "Zulu",
];

/// A light aircraft's registration and how it is spoken: "N345SP",
/// "Cessna three four five Sierra Papa"; "G-ABCD", "Cessna Golf Alpha
/// Bravo Charlie Delta".
fn registration(prefix: &str, icao_type: &str, rng: &mut XorShift64) -> (String, String) {
    let maker = match icao_type {
        "PA28" => "Piper",
        "SR22" => "Cirrus",
        _ => "Cessna",
    };
    let letter = |rng: &mut XorShift64| (b'A' + rng.below(26) as u8) as char;
    if prefix == "N" {
        let first = char::from(b'1' + rng.below(9) as u8);
        let digits: String = std::iter::once(first)
            .chain((0..2).map(|_| char::from(b'0' + rng.below(10) as u8)))
            .collect();
        let tail: String = (0..2).map(|_| letter(rng)).collect();
        let spoken = format!(
            "{maker} {} {}",
            digits.chars().map(digit_word).collect::<Vec<_>>().join(" "),
            tail.chars().map(phonetic).collect::<Vec<_>>().join(" ")
        );
        (format!("N{digits}{tail}"), spoken)
    } else {
        let n = 5usize.saturating_sub(prefix.len()).max(3);
        let tail: String = (0..n).map(|_| letter(rng)).collect();
        let spoken = format!(
            "{maker} {}",
            prefix
                .chars()
                .chain(tail.chars())
                .map(phonetic)
                .collect::<Vec<_>>()
                .join(" ")
        );
        (format!("{prefix}{tail}"), spoken)
    }
}

/// An airline callsign for the region, and its telephony.
fn airline(icao: &str, rng: &mut XorShift64) -> (String, String) {
    let carriers: &[(&str, &str)] = match icao.get(..2).unwrap_or("") {
        "EG" => &[("BAW", "Speedbird"), ("EZY", "Easy"), ("RYR", "Ryanair")],
        "LE" | "GC" => &[("IBE", "Iberia"), ("VLG", "Vueling"), ("RYR", "Ryanair")],
        "LF" => &[
            ("AFR", "Air France"),
            ("EZY", "Easy"),
            ("TVF", "France Soleil"),
        ],
        "ED" | "ET" => &[
            ("DLH", "Lufthansa"),
            ("EWG", "Eurowings"),
            ("CFG", "Condor"),
        ],
        "LI" => &[("ITY", "Itarrow"), ("RYR", "Ryanair"), ("EZY", "Easy")],
        _ => &[("RYR", "Ryanair"), ("EZY", "Easy"), ("DLH", "Lufthansa")],
    };
    let (code, telephony) = carriers[rng.below(carriers.len() as u64) as usize];
    let number = 100 + rng.below(9800);
    let words = number
        .to_string()
        .chars()
        .map(digit_word)
        .collect::<Vec<_>>()
        .join(" ");
    (format!("{code}{number}"), format!("{telephony} {words}"))
}

fn digit_word(c: char) -> &'static str {
    match c {
        '0' => "zero",
        '1' => "one",
        '2' => "two",
        '3' => "three",
        '4' => "four",
        '5' => "five",
        '6' => "six",
        '7' => "seven",
        '8' => "eight",
        '9' => "niner",
        _ => "",
    }
}

fn phonetic(c: char) -> &'static str {
    match c.to_ascii_uppercase() {
        c @ 'A'..='Z' => PHONETIC[(c as u8 - b'A') as usize],
        c => digit_word(c),
    }
}

/// One row of the pilot's traffic file (`XCOPILOT_TRAFFIC_FILE`, YAML, a
/// list of these). The bin reads the variable and the file; this crate
/// only parses the text.
///
/// ```yaml
/// - t_s: 0
///   callsign: N345SP
///   type: C172
///   kind: circuit        # circuit | departure | arrival
///   runway: "13"         # optional: the tower's runway in use
///   stand: W4            # optional: a free stand
///   laps: 3              # optional, circuits (default 3)
///   spoken: Cessna three four five   # optional
/// ```
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FileEntry {
    pub t_s: f64,
    pub callsign: String,
    #[serde(rename = "type")]
    pub icao_type: String,
    pub kind: MovementKind,
    #[serde(default)]
    pub runway: Option<String>,
    #[serde(default)]
    pub stand: Option<String>,
    #[serde(default)]
    pub laps: Option<u32>,
    #[serde(default)]
    pub spoken: Option<String>,
}

/// Parse a traffic file. Refused whole on any bad row, with the row named:
/// a half-read schedule is a schedule nobody wrote.
pub fn parse_file(text: &str) -> anyhow::Result<Vec<FileEntry>> {
    let rows: Vec<FileEntry> = serde_yaml::from_str(text)?;
    for (i, r) in rows.iter().enumerate() {
        anyhow::ensure!(
            r.t_s.is_finite() && r.t_s >= 0.0,
            "row {}: t_s must be 0 or more",
            i + 1
        );
        anyhow::ensure!(!r.callsign.trim().is_empty(), "row {}: no callsign", i + 1);
        anyhow::ensure!(!r.icao_type.trim().is_empty(), "row {}: no type", i + 1);
    }
    Ok(rows)
}

/// The file's rows as movements at `field`: the runway defaults to the
/// tower's (or the field's), an arrival enters ten miles out (straight in
/// where the runway has an approach), a departure leaves on a seeded
/// bearing. A row naming a runway the field lacks is dropped.
pub fn from_file(
    rows: &[FileEntry],
    field: &FieldModel,
    seed: u64,
    ctx: &SpawnCtx,
) -> Vec<Movement> {
    let mut rng = XorShift64::new(field_seed(&field.icao, seed));
    let fallback = ctx.runway.clone().or_else(|| default_runway(field));
    let mut out: Vec<Movement> = rows
        .iter()
        .filter_map(|r| {
            let runway = r.runway.clone().or_else(|| fallback.clone())?;
            let end = field.end(&runway)?;
            let entry = (r.kind == MovementKind::Arrival)
                .then(|| entry_clear_of_own(&end, ctx.own, &mut rng))
                .flatten();
            Some(Movement {
                t_s: r.t_s,
                callsign: r.callsign.trim().to_string(),
                spoken: r
                    .spoken
                    .clone()
                    .unwrap_or_else(|| r.callsign.trim().to_string()),
                icao_type: r.icao_type.trim().to_ascii_uppercase(),
                kind: r.kind,
                runway: end.ident.clone(),
                stand: r.stand.clone(),
                entry,
                straight_in: field.procs.approach_for(&end.ident).is_some(),
                laps: r.laps.unwrap_or(3),
                sid: None,
                star: None,
                exit_bearing: rng.range_f64(0.0, 360.0),
                seed: rng.next_u64(),
            })
        })
        .collect();
    out.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    out
}

#[cfg(test)]
mod tests;
