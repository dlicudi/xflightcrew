//! Who actually flies where from a field, and how often: small static route
//! tables for the fields we fly first (Gibraltar and Malaga, the pilot's
//! approval of 2026-10-13), from which [`crate::rotation`] builds the day's
//! ROTATIONS. A field with no table keeps the hour generator
//! ([`crate::schedule::generate`]).
//!
//! A Rust table, not a data file: every row names a carrier constant of
//! [`crate::airlines`] (its telephony, its flight number range, its fleet),
//! so a carrier the station cannot say is a compile error away and a typo
//! in a type is a test failure, never a silent 172; the pilot's own file
//! (`XCOPILOT_TRAFFIC_FILE`) stays the YAML way in. Nothing here is read at
//! runtime from the network: the sim flies the timetable as written.
//!
//! Sources (read 2026-10-13):
//! - Gibraltar: Wikipedia, "Gibraltar International Airport", Airlines and
//!   destinations (British Airways to Heathrow; easyJet to Birmingham,
//!   Bristol, Gatwick and Manchester) and its 2024 route table (Heathrow
//!   193,714 passengers, Gatwick 130,689, Manchester 68,615, Bristol 31,038;
//!   3,628 movements in the year, about ten a day). BA492 Heathrow to
//!   Gibraltar and BA493 back, off the blocks 15:10 local
//!   (flightradar24.com/data/flights/ba493, planemapper.com/flights/BA492).
//! - Malaga: Wikipedia, "Malaga Airport", Airlines and destinations and the
//!   2025 route tables (Gatwick 1.24 M, Amsterdam 1.05 M, Barcelona 1.04 M,
//!   Dublin 853 k, ... down to San Sebastian 19 k); 186,990 movements in
//!   2025, about 512 a day, the summer near 600.
//!
//! A weekly frequency is the route's passengers over a full narrowbody each
//! way (about 150 seated, 85% full) and the 52 weeks, rounded, split
//! between the carriers the article names on it. Flight numbers are
//! illustrative inside each carrier's own range (only BA's are the real
//! ones). Malaga's times are the carriers' LINES OF FLYING ([`LINES`]):
//! each aircraft's day built from the route's block time and turnarounds
//! ([`crate::rotation`]); Gibraltar's rows keep the published slots.

use crate::airlines::{self, Carrier};

/// One route at a field: a carrier's rotation to and from `other`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteRow {
    pub carrier: Carrier,
    /// The INBOUND flight number of the day's first rotation (from
    /// `other` to the field); its outbound is the next number, the day's
    /// later rotations count on in twos (BA492 in, BA493 out).
    pub number: u32,
    /// The other end, its ICAO code and the name a board shows.
    pub other: &'static str,
    pub city: &'static str,
    /// Rotations a week in the IATA summer season, and in the winter.
    pub summer: u8,
    pub winter: u8,
    /// The local windows the arrivals fall in, hours: the day's rotations
    /// are spread evenly across them ([`crate::rotation`]). Empty
    /// ([`LINES`]): the carrier's lines of flying decide.
    pub windows: &'static [(f64, f64)],
    /// The first rotation of the day NIGHT-STOPS: its aircraft came in the
    /// evening before and leaves in the morning wave (a based carrier).
    pub based: bool,
    /// The types flown on the route, by weight; the carrier's own when
    /// empty.
    pub types: &'static [(&'static str, u32)],
}

/// A field's table.
#[derive(Debug, Clone, Copy)]
pub struct FieldRoutes {
    pub icao: &'static str,
    /// Standard time's offset from UTC, hours (Spain and Gibraltar keep
    /// Central European Time, with the EU's summer time on top).
    pub utc_offset_h: f64,
    pub rows: &'static [RouteRow],
}

#[allow(clippy::too_many_arguments)]
const fn r(
    carrier: Carrier,
    number: u32,
    other: &'static str,
    city: &'static str,
    summer: u8,
    winter: u8,
    windows: &'static [(f64, f64)],
    based: bool,
) -> RouteRow {
    RouteRow {
        carrier,
        number,
        other,
        city,
        summer,
        winter,
        windows,
        based,
        types: &[],
    }
}

/// [`r`] with the route's own types.
#[allow(clippy::too_many_arguments)]
const fn rt(
    carrier: Carrier,
    number: u32,
    other: &'static str,
    city: &'static str,
    summer: u8,
    winter: u8,
    windows: &'static [(f64, f64)],
    types: &'static [(&'static str, u32)],
) -> RouteRow {
    RouteRow {
        carrier,
        number,
        other,
        city,
        summer,
        winter,
        windows,
        based: false,
        types,
    }
}

use airlines::{
    AEA, AEE, AFR, ANE, AUA, BAW_EUROPE, BAW_GATWICK, BEL, DLH, EIN, ETD, EWG, EXS, EZY, FIN, IBE,
    ICE, ITY, KLM, LOT, NAX, QTR, RYR, SAS, SWR, TAP, THY, TOM, TRA, UAL_LONG, VLG, WZZ,
};

/// No windows: the route is flown on its carrier's LINES OF FLYING
/// ([`crate::rotation`]), each aircraft's day from its base's morning wave
/// on, one rotation a block time, a turnaround and a block time back after
/// the last. Until 2026-10-13 Malaga's rows named three windows (the
/// low-cost banks 09:00 to 12:00, 13:30 to 17:00 and 19:00 to 23:00, a hub
/// day 08:30 to 22:30, a shuttle 07:30 to 23:00): live at 08:00 the board
/// showed no arrival from 08:06 to 09:03 and no departure from 08:47 to
/// 09:45, the banks' edges, where the real field is busy all morning.
pub const LINES: &[(f64, f64)] = &[];

/// Gibraltar (2024 pattern): BA twice a day to Heathrow in summer, once in
/// winter at its 14:20 slot; easyJet to Gatwick most days and Manchester,
/// Bristol and Birmingham a few days a week, midday into the afternoon
/// (the runway shares the frontier road, so nothing night-stops).
const LXGB: &[RouteRow] = &[
    r(
        BAW_EUROPE,
        492,
        "EGLL",
        "London Heathrow",
        7,
        7,
        &[(14.0, 14.5)],
        false,
    ),
    r(
        BAW_EUROPE,
        490,
        "EGLL",
        "London Heathrow",
        7,
        4,
        &[(10.25, 10.75)],
        false,
    ),
    r(
        EZY,
        8901,
        "EGKK",
        "London Gatwick",
        9,
        6,
        &[(11.5, 12.5), (17.5, 18.5)],
        false,
    ),
    r(
        EZY,
        1911,
        "EGCC",
        "Manchester",
        5,
        3,
        &[(13.0, 13.5)],
        false,
    ),
    r(EZY, 6191, "EGGD", "Bristol", 3, 2, &[(15.5, 16.0)], false),
    r(
        EZY,
        2101,
        "EGBB",
        "Birmingham",
        2,
        0,
        &[(16.5, 17.0)],
        false,
    ),
];

/// Malaga (2025 pattern), its busiest routes first.
const LEMG: &[RouteRow] = &[
    // The United Kingdom and Ireland.
    r(EZY, 8061, "EGKK", "London Gatwick", 28, 21, LINES, true),
    rt(
        BAW_GATWICK,
        2614,
        "EGKK",
        "London Gatwick",
        14,
        7,
        LINES,
        &[("A320", 70), ("A20N", 30)],
    ),
    r(VLG, 8812, "EGKK", "London Gatwick", 7, 7, LINES, false),
    r(WZZ, 4471, "EGKK", "London Gatwick", 7, 4, LINES, false),
    r(EXS, 617, "EGKK", "London Gatwick", 7, 3, LINES, false),
    r(TOM, 614, "EGKK", "London Gatwick", 5, 2, LINES, false),
    r(
        BAW_EUROPE,
        470,
        "EGLL",
        "London Heathrow",
        7,
        7,
        LINES,
        false,
    ),
    r(RYR, 7062, "EIDW", "Dublin", 35, 28, LINES, true),
    r(EIN, 584, "EIDW", "Dublin", 21, 14, LINES, false),
    r(RYR, 1702, "EGCC", "Manchester", 14, 10, LINES, true),
    r(EZY, 1925, "EGCC", "Manchester", 14, 10, LINES, true),
    r(EXS, 563, "EGCC", "Manchester", 7, 5, LINES, false),
    r(TOM, 652, "EGCC", "Manchester", 5, 2, LINES, false),
    r(RYR, 8171, "EGSS", "London Stansted", 21, 14, LINES, true),
    r(EXS, 675, "EGSS", "London Stansted", 7, 4, LINES, false),
    r(EZY, 6127, "EGGD", "Bristol", 21, 14, LINES, true),
    r(RYR, 5313, "EGGD", "Bristol", 4, 3, LINES, false),
    r(RYR, 2291, "EGBB", "Birmingham", 10, 7, LINES, true),
    r(EXS, 695, "EGBB", "Birmingham", 10, 5, LINES, false),
    r(EZY, 3367, "EGBB", "Birmingham", 5, 3, LINES, false),
    r(EZY, 2041, "EGGW", "London Luton", 14, 10, LINES, false),
    r(RYR, 3027, "EGGW", "London Luton", 7, 5, LINES, true),
    r(WZZ, 4625, "EGGW", "London Luton", 3, 2, LINES, false),
    r(EZY, 2263, "EGGP", "Liverpool", 14, 10, LINES, false),
    r(RYR, 1987, "EGGP", "Liverpool", 7, 5, LINES, true),
    r(EZY, 6811, "EGPH", "Edinburgh", 7, 5, LINES, false),
    r(EXS, 751, "EGPH", "Edinburgh", 7, 4, LINES, false),
    r(RYR, 2615, "EGPH", "Edinburgh", 7, 4, LINES, true),
    r(EZY, 6957, "EGPF", "Glasgow", 7, 4, LINES, false),
    r(EXS, 787, "EGPF", "Glasgow", 7, 4, LINES, false),
    r(EZY, 6497, "EGNT", "Newcastle", 5, 3, LINES, false),
    r(EXS, 527, "EGNT", "Newcastle", 7, 4, LINES, false),
    r(RYR, 3631, "EGNX", "East Midlands", 7, 5, LINES, true),
    r(EXS, 643, "EGNX", "East Midlands", 7, 4, LINES, false),
    r(TOM, 286, "EGNX", "East Midlands", 3, 0, LINES, false),
    r(EXS, 729, "EGNM", "Leeds Bradford", 7, 4, LINES, false),
    r(RYR, 4417, "EGNM", "Leeds Bradford", 4, 2, LINES, false),
    r(EZY, 6587, "EGAA", "Belfast", 5, 3, LINES, false),
    r(RYR, 8935, "EGAA", "Belfast", 4, 2, LINES, false),
    r(TOM, 318, "EGFF", "Cardiff", 3, 0, LINES, false),
    r(RYR, 7181, "EICK", "Cork", 7, 4, LINES, false),
    r(EIN, 774, "EICK", "Cork", 4, 2, LINES, false),
    // The Benelux, France and the Nordics.
    r(KLM, 1043, "EHAM", "Amsterdam", 21, 14, LINES, false),
    r(TRA, 5091, "EHAM", "Amsterdam", 21, 14, LINES, false),
    r(EZY, 7951, "EHAM", "Amsterdam", 14, 7, LINES, false),
    r(VLG, 8411, "EHAM", "Amsterdam", 7, 7, LINES, false),
    r(TRA, 5341, "EHEH", "Eindhoven", 14, 10, LINES, false),
    r(RYR, 3913, "EHEH", "Eindhoven", 10, 7, LINES, true),
    r(BEL, 3713, "EBBR", "Brussels", 21, 14, LINES, false),
    r(TRA, 5221, "EBBR", "Brussels", 7, 4, LINES, false),
    r(VLG, 8961, "EBBR", "Brussels", 7, 7, LINES, false),
    r(RYR, 3561, "EBCI", "Brussels Charleroi", 14, 10, LINES, true),
    r(
        AFR,
        1312,
        "LFPG",
        "Paris Charles de Gaulle",
        14,
        14,
        LINES,
        false,
    ),
    r(TRA, 6112, "LFPO", "Paris Orly", 21, 14, LINES, false),
    r(VLG, 8013, "LFPO", "Paris Orly", 14, 14, LINES, true),
    r(EZY, 4417, "LFLL", "Lyon", 4, 3, LINES, false),
    r(EZY, 4581, "LFMN", "Nice", 3, 2, LINES, false),
    r(EZY, 5145, "LFSB", "Basel", 4, 3, LINES, false),
    r(NAX, 3586, "EKCH", "Copenhagen", 21, 14, LINES, true),
    r(SAS, 2581, "EKCH", "Copenhagen", 14, 10, LINES, false),
    r(NAX, 4171, "ESSA", "Stockholm", 21, 14, LINES, true),
    r(SAS, 1527, "ESSA", "Stockholm", 7, 7, LINES, false),
    r(RYR, 9161, "ESSA", "Stockholm", 4, 3, LINES, true),
    r(NAX, 1585, "ENGM", "Oslo", 14, 10, LINES, false),
    r(SAS, 4651, "ENGM", "Oslo", 7, 5, LINES, false),
    r(NAX, 5641, "EFHK", "Helsinki", 10, 7, LINES, false),
    r(FIN, 1653, "EFHK", "Helsinki", 14, 10, LINES, false),
    r(NAX, 3731, "ESGG", "Gothenburg", 4, 3, LINES, false),
    r(NAX, 2643, "ENBR", "Bergen", 3, 2, LINES, false),
    r(ICE, 532, "BIKF", "Reykjavik", 2, 2, &[(14.5, 15.5)], false),
    // Central Europe, Italy and the east.
    r(DLH, 1150, "EDDF", "Frankfurt", 14, 14, LINES, false),
    r(DLH, 1802, "EDDM", "Munich", 14, 10, LINES, false),
    r(EWG, 1580, "EDDL", "Dusseldorf", 10, 7, LINES, false),
    r(EWG, 7520, "EDDH", "Hamburg", 7, 4, LINES, false),
    r(EWG, 5820, "EDDK", "Cologne", 7, 5, LINES, false),
    r(EWG, 2654, "EDDS", "Stuttgart", 5, 3, LINES, false),
    r(EZY, 5531, "EDDB", "Berlin", 7, 4, LINES, false),
    r(EWG, 8710, "EDDB", "Berlin", 5, 3, LINES, false),
    r(SWR, 437, "LSZH", "Zurich", 14, 10, LINES, false),
    r(EZY, 1503, "LSZH", "Zurich", 7, 4, LINES, false),
    r(SWR, 469, "LSGG", "Geneva", 4, 4, LINES, false),
    r(EZY, 1427, "LSGG", "Geneva", 4, 3, LINES, false),
    r(AUA, 157, "LOWW", "Vienna", 10, 7, LINES, false),
    r(RYR, 6081, "LOWW", "Vienna", 7, 5, LINES, false),
    r(RYR, 4893, "LIRF", "Rome", 14, 10, LINES, true),
    r(ITY, 1286, "LIRF", "Rome", 7, 4, LINES, false),
    r(WZZ, 6201, "LIRF", "Rome", 7, 4, LINES, false),
    r(EZY, 2861, "LIMC", "Milan Malpensa", 14, 10, LINES, false),
    r(RYR, 2843, "LIMC", "Milan Malpensa", 7, 5, LINES, true),
    r(WZZ, 1347, "LIMC", "Milan Malpensa", 5, 3, LINES, false),
    r(RYR, 7317, "LIME", "Bergamo", 7, 5, LINES, false),
    r(RYR, 6941, "LIRN", "Naples", 4, 2, LINES, false),
    r(LOT, 431, "EPWA", "Warsaw", 5, 3, LINES, false),
    r(RYR, 1149, "EPKK", "Krakow", 5, 3, LINES, false),
    r(WZZ, 3331, "LHBP", "Budapest", 7, 4, LINES, false),
    r(WZZ, 3417, "LROP", "Bucharest", 4, 3, LINES, false),
    r(RYR, 4361, "LKPR", "Prague", 3, 2, LINES, false),
    r(AEE, 652, "LGAV", "Athens", 4, 0, LINES, false),
    r(TAP, 1361, "LPPT", "Lisbon", 14, 14, LINES, false),
    // Spain: the Barcelona and Madrid shuttles, the islands, Melilla.
    r(VLG, 2201, "LEBL", "Barcelona", 49, 42, LINES, true),
    r(RYR, 5021, "LEBL", "Barcelona", 14, 10, LINES, true),
    r(IBE, 3700, "LEMD", "Madrid", 21, 21, LINES, false),
    r(AEA, 1041, "LEMD", "Madrid", 14, 14, LINES, false),
    rt(ANE, 8810, "LEMD", "Madrid", 7, 7, LINES, &[("CRJ9", 100)]),
    r(VLG, 3911, "LEPA", "Palma de Mallorca", 14, 10, LINES, false),
    r(RYR, 6311, "LEPA", "Palma de Mallorca", 7, 4, LINES, false),
    r(AEA, 6083, "LEPA", "Palma de Mallorca", 5, 0, LINES, false),
    r(VLG, 1471, "LEBB", "Bilbao", 21, 14, LINES, true),
    r(VLG, 3021, "GCLP", "Gran Canaria", 14, 14, LINES, false),
    r(VLG, 3061, "GCXO", "Tenerife North", 14, 14, LINES, false),
    r(RYR, 7671, "GCRR", "Lanzarote", 7, 7, LINES, false),
    rt(
        ANE,
        8650,
        "GEML",
        "Melilla",
        28,
        28,
        LINES,
        &[("AT76", 100)],
    ),
    rt(
        ANE,
        8870,
        "LEVC",
        "Valencia",
        10,
        10,
        LINES,
        &[("CRJ9", 100)],
    ),
    r(VLG, 1281, "LEST", "Santiago", 12, 10, LINES, false),
    r(VLG, 3519, "LEIB", "Ibiza", 7, 2, LINES, false),
    r(RYR, 8261, "LEIB", "Ibiza", 7, 0, LINES, false),
    // The long tail (Wikipedia's destinations past the busiest-route
    // tables), each a few days a week: about a third of the airport's
    // passengers fly on routes too small to be named in those tables.
    r(RYR, 9301, "EKAH", "Aarhus", 5, 3, LINES, false),
    r(RYR, 9305, "LIPE", "Bologna", 7, 4, LINES, false),
    r(RYR, 9309, "EGHH", "Bournemouth", 4, 2, LINES, false),
    r(RYR, 9313, "LZIB", "Bratislava", 5, 3, LINES, false),
    r(RYR, 9317, "EDDW", "Bremen", 4, 3, LINES, false),
    r(RYR, 9321, "LKTB", "Brno", 3, 2, LINES, false),
    r(RYR, 9325, "LROP", "Bucharest", 5, 3, LINES, false),
    r(RYR, 9329, "LHBP", "Budapest", 6, 4, LINES, false),
    r(RYR, 9333, "EDDK", "Cologne", 6, 4, LINES, false),
    r(RYR, 9337, "EGTE", "Exeter", 3, 2, LINES, false),
    r(RYR, 9341, "GMFF", "Fez", 4, 4, LINES, false),
    r(RYR, 9345, "EDFH", "Frankfurt Hahn", 4, 3, LINES, false),
    r(RYR, 9349, "EPGD", "Gdansk", 5, 3, LINES, false),
    r(RYR, 9353, "EGPK", "Glasgow Prestwick", 4, 2, LINES, false),
    r(RYR, 9357, "ESGG", "Gothenburg", 4, 3, LINES, false),
    r(RYR, 9361, "EDDH", "Hamburg", 5, 3, LINES, false),
    r(RYR, 9365, "EDSB", "Karlsruhe", 6, 4, LINES, false),
    r(RYR, 9369, "EPKT", "Katowice", 5, 3, LINES, false),
    r(RYR, 9373, "EYKA", "Kaunas", 3, 3, LINES, false),
    r(RYR, 9377, "EIKN", "Knock", 3, 2, LINES, false),
    r(RYR, 9381, "LZKZ", "Kosice", 3, 2, LINES, false),
    r(RYR, 9385, "LPPT", "Lisbon", 7, 5, LINES, false),
    r(RYR, 9389, "EPLL", "Lodz", 3, 2, LINES, false),
    r(RYR, 9393, "EDHL", "Lubeck", 3, 2, LINES, false),
    r(RYR, 9397, "GMMX", "Marrakesh", 7, 7, LINES, false),
    r(RYR, 9401, "LFML", "Marseille", 5, 3, LINES, false),
    r(RYR, 9405, "EDJA", "Memmingen", 6, 4, LINES, false),
    r(RYR, 9409, "EDDG", "Munster", 3, 2, LINES, false),
    r(RYR, 9413, "EGHQ", "Newquay", 3, 0, LINES, false),
    r(RYR, 9417, "EDDN", "Nuremberg", 5, 3, LINES, false),
    r(RYR, 9421, "LKMT", "Ostrava", 3, 2, LINES, false),
    r(RYR, 9425, "LIRP", "Pisa", 5, 3, LINES, false),
    r(RYR, 9429, "LPPR", "Porto", 7, 5, LINES, false),
    r(RYR, 9433, "EPPO", "Poznan", 4, 3, LINES, false),
    r(RYR, 9437, "GMME", "Rabat", 5, 5, LINES, false),
    r(RYR, 9441, "EVRA", "Riga", 5, 3, LINES, false),
    r(RYR, 9445, "ENTO", "Sandefjord", 4, 3, LINES, false),
    r(RYR, 9449, "EINN", "Shannon", 5, 3, LINES, false),
    r(RYR, 9453, "LBSF", "Sofia", 5, 3, LINES, false),
    r(RYR, 9457, "GMTT", "Tangier", 7, 7, LINES, false),
    r(RYR, 9461, "EGNV", "Teesside", 3, 2, LINES, false),
    r(RYR, 9465, "GMTN", "Tetouan", 4, 4, LINES, false),
    r(RYR, 9469, "LIPH", "Treviso", 5, 3, LINES, false),
    r(RYR, 9473, "LIMF", "Turin", 5, 3, LINES, false),
    r(RYR, 9477, "EDLV", "Weeze", 6, 4, LINES, false),
    r(RYR, 9481, "EPWR", "Wroclaw", 5, 3, LINES, false),
    r(RYR, 9485, "LDZA", "Zagreb", 3, 2, LINES, false),
    r(RYR, 9489, "EKCH", "Copenhagen", 6, 4, LINES, false),
    r(RYR, 9493, "LEVC", "Valencia", 6, 4, LINES, false),
    r(RYR, 9497, "EHAM", "Amsterdam", 6, 4, LINES, false),
    r(RYR, 9501, "EDDB", "Berlin", 6, 4, LINES, false),
    r(EZY, 7101, "LFRS", "Nantes", 4, 3, LINES, false),
    r(EZY, 7105, "LFBD", "Bordeaux", 3, 3, LINES, false),
    r(EZY, 7109, "EGMC", "London Southend", 4, 2, LINES, false),
    r(EZY, 7113, "LGAV", "Athens", 3, 0, LINES, false),
    r(EZY, 7117, "GMMX", "Marrakesh", 4, 3, LINES, false),
    r(EZY, 7121, "EGNM", "Leeds Bradford", 4, 0, LINES, false),
    r(EXS, 801, "EGGD", "Bristol", 4, 0, LINES, false),
    r(EXS, 805, "EGGP", "Liverpool", 4, 0, LINES, false),
    r(EXS, 809, "EGAA", "Belfast", 5, 2, LINES, false),
    r(EXS, 813, "EGGW", "London Luton", 3, 0, LINES, false),
    r(TOM, 900, "EGNT", "Newcastle", 3, 0, LINES, false),
    r(TOM, 904, "EGBB", "Birmingham", 4, 0, LINES, false),
    r(TOM, 908, "EGGD", "Bristol", 3, 0, LINES, false),
    r(WZZ, 7001, "LZIB", "Bratislava", 3, 2, LINES, false),
    r(WZZ, 7005, "LRCL", "Cluj", 3, 3, LINES, false),
    r(WZZ, 7009, "EPGD", "Gdansk", 3, 2, LINES, false),
    r(WZZ, 7013, "EPKT", "Katowice", 4, 3, LINES, false),
    r(WZZ, 7017, "EPKK", "Krakow", 3, 2, LINES, false),
    r(WZZ, 7021, "LBSF", "Sofia", 4, 3, LINES, false),
    r(WZZ, 7025, "LATI", "Tirana", 3, 3, LINES, false),
    r(WZZ, 7029, "LIPZ", "Venice", 4, 3, LINES, false),
    r(WZZ, 7033, "EYVI", "Vilnius", 3, 3, LINES, false),
    r(WZZ, 7037, "EPWA", "Warsaw", 4, 3, LINES, false),
    r(WZZ, 7041, "EPWR", "Wroclaw", 3, 2, LINES, false),
    r(NAX, 6101, "EKYT", "Aalborg", 4, 3, LINES, false),
    r(NAX, 6105, "EKBI", "Billund", 4, 3, LINES, false),
    r(NAX, 6109, "ENZV", "Stavanger", 4, 3, LINES, false),
    r(NAX, 6113, "ENTO", "Sandefjord", 3, 2, LINES, false),
    r(NAX, 6117, "ESKN", "Stockholm Skavsta", 3, 2, LINES, false),
    r(NAX, 6121, "ENVA", "Trondheim", 3, 0, LINES, false),
    r(NAX, 6125, "EKAH", "Aarhus", 3, 0, LINES, false),
    r(NAX, 6129, "ENHD", "Haugesund", 2, 0, LINES, false),
    r(NAX, 6133, "EDDM", "Munich", 3, 0, LINES, false),
    r(VLG, 4101, "EGFF", "Cardiff", 3, 0, LINES, false),
    r(VLG, 4105, "GCFV", "Fuerteventura", 4, 4, LINES, false),
    r(VLG, 4109, "GCRR", "Lanzarote", 4, 4, LINES, false),
    r(TRA, 6501, "EHRD", "Rotterdam", 6, 4, LINES, false),
    r(TRA, 6505, "LFLL", "Lyon", 3, 0, LINES, false),
    r(TRA, 6509, "LFMT", "Montpellier", 3, 0, LINES, false),
    r(EWG, 9101, "LKPR", "Prague", 3, 0, LINES, false),
    r(EWG, 9105, "EDDV", "Hannover", 4, 3, LINES, false),
    // Long haul and the Gulf.
    rt(
        UAL_LONG,
        120,
        "KEWR",
        "Newark",
        7,
        0,
        &[(8.0, 8.5)],
        &[("B763", 100)],
    ),
    rt(
        QTR,
        52,
        "OTHH",
        "Doha",
        7,
        7,
        &[(13.0, 13.5)],
        &[("B788", 60), ("A359", 40)],
    ),
    rt(
        THY,
        1966,
        "LTFM",
        "Istanbul",
        7,
        7,
        &[(11.5, 12.0)],
        &[("A321", 100)],
    ),
    rt(
        ETD,
        131,
        "OMAA",
        "Abu Dhabi",
        3,
        0,
        &[(7.5, 8.0)],
        &[("A321", 100)],
    ),
];

/// The fields with a table.
const FIELDS: &[FieldRoutes] = &[
    FieldRoutes {
        icao: "LXGB",
        utc_offset_h: 1.0,
        rows: LXGB,
    },
    FieldRoutes {
        icao: "LEMG",
        utc_offset_h: 1.0,
        rows: LEMG,
    },
];

/// Every field with a route table.
pub fn fields() -> &'static [FieldRoutes] {
    FIELDS
}

/// The route table of `icao`, if we fly one there.
pub fn field_routes(icao: &str) -> Option<&'static FieldRoutes> {
    FIELDS.iter().find(|f| f.icao.eq_ignore_ascii_case(icao))
}

/// Every carrier a route table names, for the telephony check
/// ([`crate::airlines::all_carriers`]).
pub fn all_carriers() -> Vec<Carrier> {
    FIELDS
        .iter()
        .flat_map(|f| f.rows.iter().map(|r| r.carrier))
        .collect()
}

/// An airline's name as a board shows it, and its IATA designator (the
/// board's flight numbers: "U2 8901"). The ICAO code itself for one not
/// listed.
pub fn airline_name(code: &str) -> (&'static str, &'static str) {
    match code {
        "BAW" => ("British Airways", "BA"),
        "EZY" => ("easyJet", "U2"),
        "RYR" => ("Ryanair", "FR"),
        "VLG" => ("Vueling", "VY"),
        "EXS" => ("Jet2", "LS"),
        "TOM" => ("TUI", "BY"),
        "NAX" => ("Norwegian", "DY"),
        "KLM" => ("KLM", "KL"),
        "TRA" => ("Transavia", "HV"),
        "EIN" => ("Aer Lingus", "EI"),
        "BEL" => ("Brussels Airlines", "SN"),
        "SAS" => ("SAS", "SK"),
        "FIN" => ("Finnair", "AY"),
        "AUA" => ("Austrian", "OS"),
        "DLH" => ("Lufthansa", "LH"),
        "EWG" => ("Eurowings", "EW"),
        "SWR" => ("Swiss", "LX"),
        "AFR" => ("Air France", "AF"),
        "IBE" => ("Iberia", "IB"),
        "IBS" => ("Iberia Express", "I2"),
        "AEA" => ("Air Europa", "UX"),
        "ANE" => ("Air Nostrum", "YW"),
        "TAP" => ("TAP Air Portugal", "TP"),
        "THY" => ("Turkish Airlines", "TK"),
        "QTR" => ("Qatar Airways", "QR"),
        "UAL" => ("United", "UA"),
        "ETD" => ("Etihad", "EY"),
        "LOT" => ("LOT", "LO"),
        "ICE" => ("Icelandair", "FI"),
        "AEE" => ("Aegean", "A3"),
        "WZZ" => ("Wizz Air", "W6"),
        "ITY" => ("ITA Airways", "AZ"),
        _ => ("", ""),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every row flies types the performance tables know, inside its
    /// carrier's own number range, to a four-letter ICAO, in windows that
    /// are a day's hours (or on its lines of flying, to a place we know),
    /// and never a frequency of more than six a day.
    #[test]
    fn every_route_row_is_well_formed() {
        for f in FIELDS {
            for row in f.rows {
                let types = if row.types.is_empty() {
                    row.carrier.types
                } else {
                    row.types
                };
                assert!(types.iter().any(|(_, w)| *w > 0), "{}", row.number);
                for (t, _) in types {
                    assert_ne!(
                        crate::perf::class_for_type(t).0.name,
                        crate::perf::PISTON_SINGLE.name,
                        "{} {}: {t}",
                        row.carrier.code,
                        row.number
                    );
                }
                let (lo, hi) = row.carrier.numbers;
                assert!(
                    (lo..=hi).contains(&row.number),
                    "{}{} outside {lo}..{hi}",
                    row.carrier.code,
                    row.number
                );
                assert_eq!(row.other.len(), 4, "{}", row.other);
                assert!(
                    !row.windows.is_empty() || crate::places::position(row.other).is_some(),
                    "{}: lines of flying need the other end's place",
                    row.other
                );
                assert!(row
                    .windows
                    .iter()
                    .all(|(a, b)| 0.0 <= *a && a <= b && *b <= 24.0));
                assert!(row.summer <= 42 + 7 && row.winter <= 42 + 7);
                assert!(
                    !airline_name(row.carrier.code).0.is_empty(),
                    "{}",
                    row.carrier.code
                );
            }
        }
    }

    /// No two rows of a field share a flight number: two rotations would
    /// fly under one callsign.
    #[test]
    fn flight_numbers_never_collide_at_a_field() {
        for f in FIELDS {
            let mut seen = std::collections::HashSet::new();
            for row in f.rows {
                let daily = (row.summer.max(row.winter) as u32).div_ceil(7);
                for k in 0..daily {
                    for n in [row.number + 2 * k, row.number + 2 * k + 1] {
                        assert!(
                            seen.insert(format!("{}{n}", row.carrier.code)),
                            "{} {}{n} twice",
                            f.icao,
                            row.carrier.code
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn gibraltar_is_british_airways_and_easyjet() {
        let t = field_routes("lxgb").unwrap();
        assert!(t
            .rows
            .iter()
            .all(|r| ["BAW", "EZY"].contains(&r.carrier.code)));
        assert!(field_routes("EGKK").is_none());
    }
}
