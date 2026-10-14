//! Who flies from a field, and in what: a small weighted table of carriers
//! (realism review, 2026-10-09). Every field drew the same three carriers
//! of its country and four types: Ryanair at Heathrow, 27% ATRs at Gatwick
//! (one airline stand in five went regional), nine Air Nostrum ATRs of nine
//! at Gibraltar, where easyJet and BA fly A320s. Now a field we fly has its
//! own mix ([`field_mix`]), else its country's ([`country_mix`]), else the
//! old three ([`FALLBACK`]); each carrier names its ICAO code, its
//! telephony (the station's table has every one, pinned by the bin's
//! `every_traffic_carrier_has_its_telephony_at_the_station`), its share of
//! the field, the types it flies there with their weights, and the flight
//! numbers it uses.
//!
//! The weights are shares of movements, judged from the airlines' own
//! timetables (summer 2025, rounded): easyJet is near half of Gatwick, BA
//! half of Heathrow, Ryanair a third of Malaga. A type the field cannot
//! take (no stand wide enough, a runway too short, [`Pick`]) is dropped and
//! the rest renormalised; a field whose own mix leaves nothing falls to its
//! country's, then to the fallback, so a turboprop-only field keeps its
//! regional carrier.

use crate::rng::XorShift64;

/// One airline as it flies at a field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Carrier {
    /// The ICAO airline designator ("EZY"): the callsign's letters, and the
    /// livery the plugin asks the CSL for.
    pub code: &'static str,
    /// The radio telephony ("Easy").
    pub telephony: &'static str,
    /// The types it flies, each with its weight. A weight of 0 is an
    /// understudy: flown only where none of the weighted types fits (the
    /// Saab where no stand takes the ATR, UNSS and ULKK, 2026-10-09).
    pub types: &'static [(&'static str, u32)],
    /// Its flight numbers, inclusive: a carrier's own range, never one
    /// range for all (every callsign was 100 to 9899).
    pub numbers: (u32, u32),
}

/// A carrier's share at a field.
pub type Share = (Carrier, u32);

// The carriers. A carrier flying different fleets or numbers at different
// fields has one entry per use (BA's Gatwick 2xxx flights and long-haul
// 777s are not its Heathrow ones).
pub(crate) const fn c(
    code: &'static str,
    telephony: &'static str,
    types: &'static [(&'static str, u32)],
    numbers: (u32, u32),
) -> Carrier {
    Carrier {
        code,
        telephony,
        types,
        numbers,
    }
}

const NARROW_A: &[(&str, u32)] = &[("A320", 45), ("A321", 20), ("A319", 15), ("A20N", 20)];
pub(crate) const EZY: Carrier = c(
    "EZY",
    "Easy",
    &[("A320", 45), ("A319", 15), ("A20N", 25), ("A21N", 15)],
    (11, 8999),
);
pub(crate) const EZY_GATWICK: Carrier = c("EZY", "Easy", EZY.types, (8000, 8999));
pub(crate) const BAW_EUROPE: Carrier = c(
    "BAW",
    "Speedbird",
    &[("A320", 55), ("A319", 20), ("A321", 15), ("A20N", 10)],
    (300, 999),
);
pub(crate) const BAW_GATWICK: Carrier = c(
    "BAW",
    "Speedbird",
    &[
        ("A320", 50),
        ("A321", 15),
        ("A20N", 15),
        ("B772", 15),
        ("B77W", 5),
    ],
    (2000, 2799),
);
pub(crate) const BAW_HEATHROW: Carrier = c(
    "BAW",
    "Speedbird",
    &[
        ("A320", 28),
        ("A319", 10),
        ("A321", 14),
        ("A20N", 8),
        ("A21N", 6),
        ("B772", 8),
        ("B77W", 7),
        ("B788", 4),
        ("B789", 6),
        ("A359", 5),
        ("A388", 4),
    ],
    (1, 999),
);
pub(crate) const BAW_LONG: Carrier = c(
    "BAW",
    "Speedbird",
    &[("B772", 30), ("B77W", 30), ("B789", 20), ("A388", 20)],
    (1, 299),
);
pub(crate) const RYR: Carrier = c(
    "RYR",
    "Ryanair",
    &[("B738", 75), ("B38M", 25)],
    (1000, 9999),
);
pub(crate) const WZZ: Carrier = c(
    "WZZ",
    "Wizz Air",
    &[("A321", 50), ("A21N", 35), ("A320", 15)],
    (1000, 9999),
);
pub(crate) const VLG: Carrier = c(
    "VLG",
    "Vueling",
    &[
        ("A320", 50),
        ("A321", 15),
        ("A319", 10),
        ("A20N", 20),
        ("A21N", 5),
    ],
    (1000, 8999),
);
pub(crate) const TOM: Carrier = c(
    "TOM",
    "Tomjet",
    &[("B738", 60), ("B38M", 25), ("B788", 10), ("B789", 5)],
    (1, 999),
);
pub(crate) const EXS: Carrier = c(
    "EXS",
    "Channex",
    &[("B738", 60), ("A321", 25), ("A21N", 15)],
    (100, 999),
);
pub(crate) const UAE: Carrier = c("UAE", "Emirates", &[("A388", 60), ("B77W", 40)], (1, 99));
pub(crate) const VIR: Carrier = c(
    "VIR",
    "Virgin",
    &[("A333", 20), ("A339", 30), ("B789", 35), ("A35K", 15)],
    (1, 499),
);
pub(crate) const EIN: Carrier = c(
    "EIN",
    "Shamrock",
    &[("A320", 60), ("A21N", 30), ("A321", 10)],
    (100, 999),
);
pub(crate) const DLH: Carrier = c(
    "DLH",
    "Lufthansa",
    &[
        ("A320", 40),
        ("A321", 25),
        ("A20N", 20),
        ("A21N", 10),
        ("A319", 5),
    ],
    (1, 2999),
);
pub(crate) const DLH_FRANKFURT: Carrier = c(
    "DLH",
    "Lufthansa",
    &[
        ("A320", 25),
        ("A321", 15),
        ("A20N", 10),
        ("A21N", 5),
        ("CRJ9", 6),
        ("E190", 6),
        ("A333", 6),
        ("A346", 4),
        ("A359", 8),
        ("B748", 5),
        ("B744", 3),
        ("B789", 4),
        ("A388", 3),
    ],
    (1, 2999),
);
pub(crate) const AAL_LONG: Carrier = c(
    "AAL",
    "American",
    &[("B772", 45), ("B77W", 30), ("B788", 10), ("B789", 15)],
    (1, 299),
);
pub(crate) const AAL: Carrier = c(
    "AAL",
    "American",
    &[
        ("A321", 30),
        ("B738", 30),
        ("A319", 10),
        ("B772", 10),
        ("B77W", 8),
        ("B788", 6),
        ("B789", 6),
    ],
    (1, 2999),
);
pub(crate) const UAL_LONG: Carrier = c(
    "UAL",
    "United",
    &[("B772", 45), ("B763", 20), ("B789", 25), ("B788", 10)],
    (1, 999),
);
pub(crate) const UAL: Carrier = c(
    "UAL",
    "United",
    &[
        ("B738", 15),
        ("B739", 15),
        ("B38M", 5),
        ("A320", 10),
        ("A319", 5),
        ("B772", 15),
        ("B77W", 10),
        ("B789", 15),
        ("B78X", 5),
        ("B763", 5),
    ],
    (1, 2999),
);
pub(crate) const DAL_LONG: Carrier = c(
    "DAL",
    "Delta",
    &[("A333", 40), ("A339", 30), ("B763", 30)],
    (1, 499),
);
pub(crate) const DAL: Carrier = c(
    "DAL",
    "Delta",
    &[
        ("A321", 20),
        ("B739", 15),
        ("A21N", 10),
        ("B752", 15),
        ("A333", 10),
        ("A339", 10),
        ("B763", 10),
        ("A359", 10),
    ],
    (1, 2999),
);
pub(crate) const JBU: Carrier = c(
    "JBU",
    "JetBlue",
    &[("A320", 40), ("A321", 40), ("A21N", 10), ("BCS3", 10)],
    (1, 2999),
);
pub(crate) const SWA: Carrier = c(
    "SWA",
    "Southwest",
    &[("B737", 40), ("B738", 40), ("B38M", 20)],
    (100, 4999),
);
pub(crate) const ASA: Carrier = c(
    "ASA",
    "Alaska",
    &[("B739", 50), ("B738", 40), ("A320", 10)],
    (1, 1999),
);
pub(crate) const QTR: Carrier = c(
    "QTR",
    "Qatari",
    &[("A359", 40), ("B77W", 30), ("A388", 20), ("A35K", 10)],
    (1, 99),
);
pub(crate) const TAP: Carrier = c(
    "TAP",
    "Air Portugal",
    &[("A320", 40), ("A321", 30), ("A20N", 15), ("A21N", 15)],
    (1300, 1399),
);
pub(crate) const THY: Carrier = c(
    "THY",
    "Turkish",
    &[("A321", 50), ("B738", 25), ("A333", 25)],
    (1960, 1999),
);
pub(crate) const ACA: Carrier = c(
    "ACA",
    "Air Canada",
    &[("B789", 50), ("B77W", 30), ("B788", 20)],
    (800, 899),
);
pub(crate) const SWR: Carrier = c(
    "SWR",
    "Swiss",
    &[("A320", 40), ("A321", 20), ("A20N", 20), ("BCS3", 20)],
    (300, 499),
);
pub(crate) const AFR: Carrier = c("AFR", "Air France", NARROW_A, (1000, 1999));
pub(crate) const AFR_PARIS: Carrier = c(
    "AFR",
    "Air France",
    &[
        ("A320", 25),
        ("A321", 15),
        ("A319", 10),
        ("BCS3", 10),
        ("B77W", 10),
        ("B772", 5),
        ("A359", 8),
        ("B789", 5),
    ],
    (1, 1999),
);
pub(crate) const AFR_LONG: Carrier = c("AFR", "Air France", &[("B77W", 50), ("A359", 50)], (1, 99));
pub(crate) const KLM: Carrier = c(
    "KLM",
    "KLM",
    &[("B738", 50), ("B739", 20), ("E190", 30)],
    (1000, 1999),
);
pub(crate) const KLM_AMSTERDAM: Carrier = c(
    "KLM",
    "KLM",
    &[
        ("B738", 25),
        ("B739", 10),
        ("E190", 15),
        ("A21N", 5),
        ("B772", 5),
        ("B77W", 6),
        ("B789", 6),
        ("B78X", 4),
        ("A333", 4),
    ],
    (1, 1999),
);
pub(crate) const IBE: Carrier = c(
    "IBE",
    "Iberia",
    &[("A320", 40), ("A321", 30), ("A20N", 20), ("A21N", 10)],
    (3000, 3999),
);
pub(crate) const IBE_MADRID: Carrier = c(
    "IBE",
    "Iberia",
    &[
        ("A320", 25),
        ("A321", 20),
        ("A20N", 15),
        ("A21N", 10),
        ("A333", 10),
        ("A359", 15),
        ("A346", 5),
    ],
    (1, 3999),
);
pub(crate) const IBS: Carrier = c(
    "IBS",
    "Iberexpres",
    &[("A320", 60), ("A321", 25), ("A20N", 15)],
    (3800, 3999),
);
pub(crate) const AEA: Carrier = c(
    "AEA",
    "Air Europa",
    &[("B738", 60), ("B38M", 25), ("B789", 15)],
    (1000, 9999),
);
pub(crate) const NAX: Carrier = c(
    "NAX",
    "Nor Shuttle",
    &[("B738", 70), ("B38M", 30)],
    (1000, 7999),
);
pub(crate) const TRA: Carrier = c(
    "TRA",
    "Transavia",
    &[("B738", 75), ("A21N", 25)],
    (5000, 6999),
);
pub(crate) const TVF: Carrier = c(
    "TVF",
    "France Soleil",
    &[("B738", 80), ("A20N", 20)],
    (3000, 4999),
);
pub(crate) const EWG: Carrier = c(
    "EWG",
    "Eurowings",
    &[("A320", 50), ("A319", 30), ("A20N", 20)],
    (1000, 9999),
);
pub(crate) const CFG: Carrier = c(
    "CFG",
    "Condor",
    &[("A320", 40), ("A321", 30), ("A21N", 15), ("A339", 15)],
    (1000, 2999),
);
pub(crate) const ITY: Carrier = c(
    "ITY",
    "Itarrow",
    &[("A320", 50), ("A20N", 30), ("A21N", 20)],
    (1, 2999),
);
pub(crate) const ANE: Carrier = c(
    "ANE",
    "Air Nostrum",
    &[("AT76", 70), ("CRJ9", 30), ("SF34", 0)],
    (8000, 8999),
);
pub(crate) const LOG: Carrier = c("LOG", "Logan", &[("AT76", 50), ("SF34", 50)], (100, 999));
pub(crate) const HOP: Carrier = c(
    "HOP",
    "Air Hop",
    &[("AT76", 50), ("CRJ9", 25), ("E190", 25), ("SF34", 0)],
    (1000, 7999),
);

// The carriers only the route tables fly (`crate::routes`, 2026-10-13):
// Malaga's scheduled mix past the leisure carriers. Each one's telephony is
// the station's (`atc::callsign::TELEPHONY`).
pub(crate) const BEL: Carrier = c(
    "BEL",
    "Bee-line",
    &[("A320", 60), ("A319", 40)],
    (3000, 3999),
);
pub(crate) const AUA: Carrier = c(
    "AUA",
    "Austrian",
    &[("A320", 60), ("A321", 25), ("A20N", 15)],
    (100, 999),
);
pub(crate) const SAS: Carrier = c(
    "SAS",
    "Scandinavian",
    &[("A20N", 70), ("A320", 20), ("A21N", 10)],
    (1000, 4999),
);
pub(crate) const FIN: Carrier = c(
    "FIN",
    "Finnair",
    &[("A321", 50), ("A320", 30), ("A319", 20)],
    (1000, 1999),
);
pub(crate) const ICE: Carrier = c("ICE", "Iceair", &[("B38M", 100)], (500, 599));
pub(crate) const AEE: Carrier = c("AEE", "Aegean", &[("A20N", 60), ("A21N", 40)], (600, 699));
pub(crate) const LOT: Carrier = c("LOT", "LOT", &[("B38M", 100)], (400, 499));
pub(crate) const ETD: Carrier = c("ETD", "Etihad", &[("B789", 60), ("A321", 40)], (1, 199));

// Gatwick's route table's carriers (2026-10-14). Each telephony is the
// ICAO one, read 2026-10-14: Wikipedia's "List of airline codes" (SXS
// SUNEXPRESS, CAI CORENDON, CES CHINA EASTERN, SEH AIR CRETE, CCA AIR
// CHINA, AUR AYLINE), airplanes.live and airhex.com for UBT LONGBOAT (Norse
// Atlantic UK; NBT LONGSHIP is the Norwegian AOC), 123atc.com and
// airhex.com for KMM SKYKNIGHT (KM Malta Airlines). Aurigny flies its
// E195 to Gatwick (its ATRs serve the islands).
pub(crate) const SXS: Carrier = c(
    "SXS",
    "Sunexpress",
    &[("B738", 70), ("B38M", 30)],
    (1600, 1699),
);
pub(crate) const CAI: Carrier = c(
    "CAI",
    "Corendon",
    &[("B738", 70), ("B38M", 30)],
    (1600, 1699),
);
pub(crate) const KMM: Carrier = c(
    "KMM",
    "Skyknight",
    &[("A20N", 70), ("A21N", 30)],
    (100, 499),
);
pub(crate) const SEH: Carrier = c(
    "SEH",
    "Air Crete",
    &[("A20N", 60), ("A21N", 40)],
    (100, 999),
);
pub(crate) const UBT: Carrier = c("UBT", "Longboat", &[("B789", 100)], (1, 999));
pub(crate) const CES: Carrier = c(
    "CES",
    "China Eastern",
    &[("B77W", 60), ("A359", 40)],
    (500, 599),
);
pub(crate) const CCA: Carrier = c(
    "CCA",
    "Air China",
    &[("A333", 60), ("B789", 40)],
    (800, 899),
);
pub(crate) const AUR: Carrier = c("AUR", "Ayline", &[("E195", 100)], (600, 699));

/// Every carrier constant by its Rust name, written once so the name and
/// the value can never part: the route tables' data files
/// (`crates/traffic/routes/*.yaml`) name their carriers this way.
macro_rules! named_carriers {
    ($($name:ident),* $(,)?) => {
        /// The carrier constant called `name` ("EZY_GATWICK"), `None` for a
        /// name no constant carries.
        pub fn carrier_by_name(name: &str) -> Option<Carrier> {
            match name {
                $(stringify!($name) => Some($name),)*
                _ => None,
            }
        }

        /// Every name [`carrier_by_name`] knows.
        #[cfg(test)]
        pub(crate) const CARRIER_NAMES: &[&str] = &[$(stringify!($name)),*];
    };
}

named_carriers!(
    EZY,
    EZY_GATWICK,
    BAW_EUROPE,
    BAW_GATWICK,
    BAW_HEATHROW,
    BAW_LONG,
    RYR,
    WZZ,
    VLG,
    TOM,
    EXS,
    UAE,
    VIR,
    EIN,
    DLH,
    DLH_FRANKFURT,
    AAL_LONG,
    AAL,
    UAL_LONG,
    UAL,
    DAL_LONG,
    DAL,
    JBU,
    SWA,
    ASA,
    QTR,
    TAP,
    THY,
    ACA,
    SWR,
    AFR,
    AFR_PARIS,
    AFR_LONG,
    KLM,
    KLM_AMSTERDAM,
    IBE,
    IBE_MADRID,
    IBS,
    AEA,
    NAX,
    TRA,
    TVF,
    EWG,
    CFG,
    ITY,
    ANE,
    LOG,
    HOP,
    BEL,
    AUA,
    SAS,
    FIN,
    ICE,
    AEE,
    LOT,
    ETD,
    SXS,
    CAI,
    KMM,
    SEH,
    UBT,
    CES,
    CCA,
    AUR,
);

/// The fields we fly, each its own mix.
pub fn field_mix(icao: &str) -> Option<&'static [Share]> {
    Some(match icao {
        // Gatwick: easyJet's base, BA's 2xxx and its 777s, the leisure
        // carriers and Emirates' A380 a few a day.
        "EGKK" => &[
            (EZY_GATWICK, 45),
            (BAW_GATWICK, 15),
            (WZZ, 8),
            (TOM, 8),
            (VLG, 6),
            (EXS, 5),
            (TAP, 3),
            (NAX, 2),
            (RYR, 2),
            (THY, 2),
            (QTR, 1),
            (UAE, 1),
        ],
        // Heathrow: BA half of it, long-haul a third.
        "EGLL" => &[
            (BAW_HEATHROW, 50),
            (VIR, 8),
            (EIN, 4),
            (DLH, 4),
            (AAL_LONG, 4),
            (UAL_LONG, 3),
            (UAE, 3),
            (QTR, 3),
            (AFR, 2),
            (KLM, 2),
            (IBE, 2),
            (SWR, 2),
            (DAL_LONG, 2),
            (ACA, 2),
        ],
        "LEMG" => &[
            (RYR, 35),
            (EZY, 15),
            (VLG, 12),
            (EXS, 10),
            (TOM, 8),
            (NAX, 8),
            (IBS, 5),
            (ANE, 4),
            (WZZ, 3),
            (TRA, 3),
            (EWG, 2),
        ],
        // Gibraltar: easyJet and BA, A320s on its 1,680 m.
        "LXGB" => &[(EZY, 55), (BAW_EUROPE, 45)],
        "LEJR" => &[(RYR, 30), (ANE, 30), (VLG, 20), (EWG, 10), (CFG, 10)],
        "LEBL" => &[
            (VLG, 40),
            (RYR, 15),
            (IBE, 8),
            (EZY, 6),
            (ANE, 6),
            (AEA, 4),
            (DLH, 4),
            (AFR, 3),
            (KLM, 3),
            (BAW_EUROPE, 3),
            (UAE, 1),
            (QTR, 1),
            (AAL_LONG, 1),
        ],
        "LEMD" => &[
            (IBE_MADRID, 35),
            (AEA, 15),
            (IBS, 12),
            (RYR, 10),
            (VLG, 8),
            (ANE, 5),
            (AFR, 2),
            (DLH, 2),
            (BAW_EUROPE, 2),
            (KLM, 2),
            (AAL_LONG, 2),
            (UAL_LONG, 1),
            (DAL_LONG, 1),
            (UAE, 1),
            (QTR, 1),
        ],
        "EDDF" => &[
            (DLH_FRANKFURT, 60),
            (CFG, 8),
            (EWG, 4),
            (RYR, 3),
            (UAE, 2),
            (UAL_LONG, 2),
            (DAL_LONG, 1),
            (QTR, 1),
            (AFR, 2),
            (KLM, 2),
            (BAW_EUROPE, 2),
        ],
        "LFPG" => &[
            (AFR_PARIS, 55),
            (EZY, 10),
            (VLG, 5),
            (DLH, 3),
            (KLM, 3),
            (BAW_EUROPE, 3),
            (DAL_LONG, 3),
            (UAL_LONG, 2),
            (AAL_LONG, 2),
            (UAE, 2),
            (IBE, 2),
        ],
        "EHAM" => &[
            (KLM_AMSTERDAM, 55),
            (TRA, 12),
            (EZY, 8),
            (BAW_EUROPE, 3),
            (DLH, 3),
            (AFR, 3),
            (VLG, 3),
            (DAL_LONG, 3),
            (UAL_LONG, 2),
            (UAE, 2),
        ],
        "KJFK" => &[
            (DAL, 30),
            (JBU, 30),
            (AAL, 20),
            (UAL, 5),
            (BAW_LONG, 3),
            (VIR, 3),
            (AFR_LONG, 2),
            (UAE, 2),
            (DLH, 2),
        ],
        "KSFO" => &[
            (UAL, 45),
            (ASA, 15),
            (SWA, 10),
            (AAL, 10),
            (DAL, 10),
            (JBU, 4),
            (BAW_LONG, 2),
            (UAE, 1),
        ],
        "KLAX" => &[
            (AAL, 22),
            (DAL, 20),
            (UAL, 18),
            (SWA, 15),
            (ASA, 10),
            (JBU, 3),
            (BAW_LONG, 2),
            (UAE, 2),
            (VIR, 1),
        ],
        _ => return None,
    })
}

/// A country's carriers, by the field's ICAO prefix; each with its
/// regional so a field only turboprops can use still has its traffic.
pub fn country_mix(icao: &str) -> Option<&'static [Share]> {
    if icao.starts_with('K') && icao.len() == 4 {
        return Some(&[
            (AAL, 25),
            (DAL, 25),
            (UAL, 25),
            (SWA, 15),
            (JBU, 5),
            (ASA, 5),
        ]);
    }
    Some(match icao.get(..2).unwrap_or("") {
        "EG" => &[
            (EZY, 30),
            (BAW_EUROPE, 20),
            (RYR, 20),
            (EXS, 10),
            (TOM, 8),
            (LOG, 7),
            (WZZ, 5),
        ],
        "LE" | "GC" => &[
            (VLG, 25),
            (RYR, 25),
            (ANE, 15),
            (IBE, 10),
            (AEA, 10),
            (EZY, 10),
            (IBS, 5),
        ],
        "LF" => &[
            (AFR, 30),
            (EZY, 25),
            (TVF, 15),
            (HOP, 15),
            (RYR, 10),
            (VLG, 5),
        ],
        "ED" | "ET" => &[
            (DLH, 30),
            (EWG, 25),
            (CFG, 10),
            (RYR, 10),
            (EZY, 5),
            (ANE, 5),
        ],
        "LI" => &[
            (RYR, 35),
            (ITY, 30),
            (EZY, 20),
            (WZZ, 10),
            (VLG, 5),
            (ANE, 5),
        ],
        "EH" => &[(KLM, 40), (TRA, 30), (EZY, 15), (RYR, 5), (ANE, 5)],
        _ => return None,
    })
}

/// Anywhere else: the three carriers every field drew before, and the
/// regional that flies where they cannot.
pub const FALLBACK: &[Share] = &[
    (c("RYR", "Ryanair", &[("B738", 1)], (1000, 9999)), 35),
    (
        c("EZY", "Easy", &[("A320", 70), ("A319", 30)], (11, 8999)),
        30,
    ),
    (DLH, 25),
    (ANE, 10),
];

/// Every carrier any table names, for the telephony check.
pub fn all_carriers() -> Vec<Carrier> {
    const FIELDS: &[&str] = &[
        "EGKK", "EGLL", "LEMG", "LXGB", "LEJR", "LEBL", "LEMD", "EDDF", "LFPG", "EHAM", "KJFK",
        "KSFO", "KLAX",
    ];
    const COUNTRIES: &[&str] = &["EGXX", "LEXX", "LFXX", "EDXX", "LIXX", "EHXX", "KXXX"];
    FIELDS
        .iter()
        .filter_map(|f| field_mix(f))
        .chain(COUNTRIES.iter().filter_map(|f| country_mix(f)))
        .chain(std::iter::once(FALLBACK))
        .flat_map(|t| t.iter().map(|(c, _)| *c))
        .chain(crate::routes::all_carriers())
        .collect()
}

/// The carrier a callsign belongs to at `icao` ("EZY8123" is easyJet's):
/// the field's table first, then its country's, then the fallback, then
/// any table. `None` for a registration or a carrier no table names.
pub fn carrier_of(icao: &str, callsign: &str) -> Option<Carrier> {
    let code = callsign.get(..3)?;
    if !code.bytes().all(|b| b.is_ascii_uppercase()) {
        return None;
    }
    let find = |t: &[Share]| t.iter().map(|(c, _)| *c).find(|c| c.code == code);
    field_mix(icao)
        .and_then(find)
        .or_else(|| country_mix(icao).and_then(find))
        .or_else(|| find(FALLBACK))
        .or_else(|| all_carriers().into_iter().find(|c| c.code == code))
}

/// The carrier and type pairs a field can fly: (carrier, type, weight),
/// the carrier's share split over the types `fits` keeps by their own
/// weights (an understudy, weight 0, only where none of its carrier's
/// weighted types fits). The field's table, else its country's, else the
/// fallback, the first that leaves anything.
pub fn candidates(icao: &str, fits: &dyn Fn(&str) -> bool) -> Vec<(Carrier, &'static str, f64)> {
    let from = |table: &[Share]| -> Vec<(Carrier, &'static str, f64)> {
        let mut out = Vec::new();
        for (carrier, share) in table {
            let weighted: Vec<(&'static str, u32)> = carrier
                .types
                .iter()
                .copied()
                .filter(|(t, w)| *w > 0 && fits(t))
                .collect();
            let kept = if weighted.is_empty() {
                carrier
                    .types
                    .iter()
                    .copied()
                    .filter(|(t, w)| *w == 0 && fits(t))
                    .take(1)
                    .map(|(t, _)| (t, 1))
                    .collect()
            } else {
                weighted
            };
            let sum: u32 = kept.iter().map(|(_, w)| w).sum();
            for (t, w) in kept {
                out.push((
                    *carrier,
                    t,
                    f64::from(*share) * f64::from(w) / f64::from(sum),
                ));
            }
        }
        out
    };
    [field_mix(icao), country_mix(icao), Some(FALLBACK)]
        .into_iter()
        .flatten()
        .map(from)
        .find(|c| !c.is_empty())
        .unwrap_or_default()
}

/// One of `pool` drawn by its weight; `None` from an empty pool.
pub fn draw<'a>(
    pool: &'a [(Carrier, &'static str, f64)],
    rng: &mut XorShift64,
) -> Option<&'a (Carrier, &'static str, f64)> {
    let total: f64 = pool.iter().map(|p| p.2).sum();
    if pool.is_empty() || total <= 0.0 {
        return None;
    }
    let mut x = rng.range_f64(0.0, total);
    for p in pool {
        if x < p.2 {
            return Some(p);
        }
        x -= p.2;
    }
    pool.last()
}

/// A flight number of `carrier`'s, as its callsign and how it is spoken:
/// "EZY8123", "Easy eight one two three".
pub fn flight(carrier: &Carrier, rng: &mut XorShift64) -> (String, String) {
    let (lo, hi) = carrier.numbers;
    let number = lo + rng.below(u64::from(hi.saturating_sub(lo)) + 1) as u32;
    let words = number
        .to_string()
        .chars()
        .map(crate::schedule::digit_word)
        .collect::<Vec<_>>()
        .join(" ");
    (
        format!("{}{number}", carrier.code),
        format!("{} {words}", carrier.telephony),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every carrier's types are ones the performance and span tables know
    /// (an unknown type would fly as a 172), each carrier has a weighted
    /// type, and its numbers are a range.
    #[test]
    fn every_carrier_flies_types_the_tables_know() {
        for c in all_carriers() {
            assert!(c.types.iter().any(|(_, w)| *w > 0), "{}", c.code);
            assert!(c.numbers.0 >= 1 && c.numbers.0 <= c.numbers.1, "{}", c.code);
            assert_eq!(c.code.len(), 3, "{}", c.code);
            for (t, _) in c.types {
                assert_ne!(
                    crate::perf::class_for_type(t).0.name,
                    crate::perf::PISTON_SINGLE.name,
                    "{} {t}: not in the class table",
                    c.code
                );
            }
        }
    }

    /// A carrier is one telephony everywhere: two entries for BA are one
    /// "Speedbird".
    #[test]
    fn a_code_has_one_telephony() {
        let all = all_carriers();
        for a in &all {
            for b in &all {
                if a.code == b.code {
                    assert_eq!(a.telephony, b.telephony, "{}", a.code);
                }
            }
        }
    }

    /// Gatwick is easyJet's (about 45%) with BA second; Heathrow BA's
    /// (about half); Gibraltar easyJet and BA only; Ryanair never at
    /// Heathrow; Gatwick flies no ATR.
    #[test]
    fn the_big_fields_fly_their_own_carriers() {
        let all = |_: &str| true;
        let share = |icao: &str, code: &str| {
            let c = candidates(icao, &all);
            let total: f64 = c.iter().map(|p| p.2).sum();
            c.iter()
                .filter(|p| p.0.code == code)
                .map(|p| p.2)
                .sum::<f64>()
                / total
        };
        assert!((share("EGKK", "EZY") - 0.45).abs() < 0.05);
        assert!(share("EGKK", "BAW") > share("EGKK", "WZZ"));
        assert!((share("EGLL", "BAW") - 0.5).abs() < 0.05);
        assert_eq!(share("EGLL", "RYR"), 0.0);
        assert!((share("LXGB", "EZY") + share("LXGB", "BAW") - 1.0).abs() < 1e-9);
        assert!(share("LEMG", "RYR") > 0.3);
        assert!(candidates("EGKK", &all).iter().all(|p| p.1 != "AT76"));
        // A field nobody listed flies its country's.
        assert!(share("EGNX", "EZY") > 0.0 && share("EGNX", "LOG") > 0.0);
        assert!(share("ZZZZ", "RYR") > 0.0);
    }

    /// A type the field cannot take is dropped, and a field whose own mix
    /// leaves nothing falls to its country's, then the fallback: a
    /// turboprop-only Gatwick would still fly the country's regional.
    #[test]
    fn what_does_not_fit_falls_through_to_what_does() {
        let narrow = |t: &str| crate::perf::wingspan_code(t) <= 'C';
        let c = candidates("EGLL", &narrow);
        assert!(!c.is_empty() && c.iter().all(|p| narrow(p.1)));
        assert!(
            c.iter().all(|p| p.0.code != "UAE"),
            "Emirates flies no narrowbody"
        );
        let props = |t: &str| crate::perf::class_for_type(t).0.name == crate::perf::TURBOPROP.name;
        let c = candidates("EGKK", &props);
        assert!(
            !c.is_empty() && c.iter().all(|p| p.0.code == "LOG"),
            "{c:?}"
        );
        // The Saab is Air Nostrum's understudy: only where the ATR and the
        // CRJ do not fit.
        let b = |t: &str| crate::perf::wingspan_code(t) <= 'B';
        let c = candidates("ZZZZ", &b);
        assert_eq!(c.len(), 1);
        assert_eq!((c[0].0.code, c[0].1), ("ANE", "SF34"));
        let wide = |t: &str| crate::perf::wingspan_code(t) <= 'C';
        assert!(candidates("ZZZZ", &wide).iter().all(|p| p.1 != "SF34"));
    }

    /// Flight numbers are the carrier's own: Emirates two figures at most,
    /// easyJet at Gatwick its 8xxx, Ryanair four figures.
    #[test]
    fn flight_numbers_are_the_carriers_own() {
        let mut r = XorShift64::new(5);
        for _ in 0..200 {
            let (cs, spoken) = flight(&UAE, &mut r);
            let n: u32 = cs[3..].parse().unwrap();
            assert!((1..=99).contains(&n), "{cs}");
            assert!(spoken.starts_with("Emirates "), "{spoken}");
            let (cs, _) = flight(&EZY_GATWICK, &mut r);
            assert!(cs.starts_with("EZY8") && cs.len() == 7, "{cs}");
            let (cs, _) = flight(&RYR, &mut r);
            assert_eq!(cs.len(), 7, "{cs}");
        }
        assert_eq!(
            carrier_of("EGKK", "EZY8123").map(|c| c.numbers),
            Some((8000, 8999))
        );
        assert_eq!(
            carrier_of("LEMG", "EZY8123").map(|c| c.numbers),
            Some((11, 8999))
        );
        assert_eq!(carrier_of("LEMG", "G-ABCD"), None);
        assert_eq!(carrier_of("EGKK", "N345SP"), None);
    }

    /// The draw follows the weights.
    #[test]
    fn the_draw_follows_the_weights() {
        let pool = candidates("EGKK", &|_| true);
        let mut r = XorShift64::new(9);
        let n = 20_000;
        let ezy = (0..n)
            .filter(|_| draw(&pool, &mut r).unwrap().0.code == "EZY")
            .count();
        let share = ezy as f64 / n as f64;
        assert!((share - 0.45).abs() < 0.02, "{share}");
        assert!(draw(&[], &mut r).is_none());
    }

    /// Every carrier constant in this file is reachable by its name, so a
    /// route file can name any of them; the source is the list, so a new
    /// constant missing from `named_carriers!` fails here.
    #[test]
    fn every_carrier_const_is_reachable_by_name() {
        let src = include_str!("airlines.rs");
        let mut declared = Vec::new();
        for line in src.lines() {
            let Some(rest) = line.strip_prefix("pub(crate) const ") else {
                continue;
            };
            if let Some((name, ty)) = rest.split_once(':') {
                if ty.trim_start().starts_with("Carrier ") {
                    declared.push(name.trim().to_string());
                }
            }
        }
        assert!(declared.len() > 40, "{declared:?}");
        for name in &declared {
            assert!(carrier_by_name(name).is_some(), "{name} has no name");
        }
        assert_eq!(declared.len(), CARRIER_NAMES.len());
        assert_eq!(carrier_by_name("EZY_GATWICK"), Some(EZY_GATWICK));
        assert_eq!(carrier_by_name("BAW_EUROPE"), Some(BAW_EUROPE));
        assert_eq!(carrier_by_name("ezy"), None);
        assert_eq!(carrier_by_name("EZY GATWICK"), None);
    }
}
