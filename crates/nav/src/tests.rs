/// LIME (2026-09-25): the plan's LEGL1P departs 28; with runway 10 in
/// use the crew picks a runway-10 SID to the same LEGLO. The real file's
/// lines.
#[test]
fn a_star_is_chosen_for_the_runway_expected_and_the_route() {
    // LIPE: SimBrief planned BAME2S for 30; the wind says 12.
    let lipe = "\
STAR:010,2,BAME2S,RW30,BAMEK,LI,E,A,E C , ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   ,BOA,LI,D, , , , , ;
STAR:020,2,BAME2S,RW30,D352R,LI,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:030,2,BAME2S,RW30, , , , ,    , ,   ,CI, , , , , ,      ,    ,    ,1930,    , ,     ,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:040,2,BAME2S,RW30,D108O,LI,P,C,E   ,R,   ,AF, ,BOA,LI,D, ,      ,1080,0150,3490,    , ,     ,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:050,2,BAME2S,RW30,D114M,LI,P,C,E   , ,   ,CF, ,BOA,LI,D, ,      ,1144,0130,2940,0020, ,     ,     ,     ,-,210,    ,   , , , , , , , , ;
STAR:060,2,BAME2S,RW30,ABLEX,LI,P,C,EECH, ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:010,5,BAME5M,RW12,BAMEK,LI,E,A,E   , ,010,IF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   ,LIPE,LI,P,A,B, , , ;
STAR:020,5,BAME5M,RW12,KREVA,LI,E,A,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,05000,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:030,5,BAME5M,RW12,PE730,LI,P,C,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,04000,     ,     ,-,230,    ,   , , , , , , , , ;
STAR:040,5,BAME5M,RW12,PE725,LI,P,C,E   ,R,010,TF, , , , , ,      ,    ,    ,    ,    ,+,04000,     ,     ,-,230,    ,   , , , , , , , , ;
STAR:050,5,BAME5M,RW12,PE721,LI,P,C,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,04000,     ,     ,-,230,    ,   , , , , , , , , ;
STAR:060,5,BAME5M,RW12,PE515,LI,P,C,E   ,L,010,TF,Y, , , , ,      ,    ,    ,    ,    ,+,03500,     ,     ,-,210,    ,   , , , , , , , , ;
STAR:070,5,BAME5M,RW12,EMDUD,LI,P,C,EE H, ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,02500,     ,     ,-,200,    ,   , , , , , , , , ;";
    let route: Vec<String> = ["LUMAV", "BAMEK", "D352R", "D108O", "D114M", "ABLEX"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    let c = stars_for_route_in(lipe.lines(), "12", &route);
    assert_eq!(c.len(), 1, "{c:?}");
    assert_eq!((c[0].ident.as_str(), c[0].entry.as_str(), c[0].entry_index), ("BAME5M", "BAMEK", 1));
    let fixes: Vec<&str> = c[0].legs.iter().map(|l| l.fix_ident.as_str()).collect();
    assert_eq!(fixes, ["BAMEK", "KREVA", "PE730", "PE725", "PE721", "PE515", "EMDUD"]);
    let c = stars_for_route_in(lipe.lines(), "30", &route);
    assert_eq!(c[0].ident, "BAME2S");
    assert!(stars_for_route_in(lipe.lines(), "12", &["XXXXX".to_string()]).is_empty());
}

#[test]
fn a_star_enters_by_its_transition_and_ends_on_the_runway_part() {
    let ksfo = "\
STAR:010,4,ALWYS3,INYOE,INYOE,K2,E,A,E  H, ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,18000, ,   ,    ,   , , , , , , , , ;
STAR:020,4,ALWYS3,INYOE,DYAMD,K2,P,C,EE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,FL270,     ,     , ,270,    ,   , , , , , , , , ;
STAR:020,4,ALWYS3,RUSME,DYAMD,K2,P,C,EE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,FL300,     ,     , ,270,    ,   , , , , , , , , ;
STAR:010,4,ALWYS3,RUSME,RUSME,K2,E,A,E  H, ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,18000, ,   ,    ,   , , , , , , , , ;
STAR:010,5,ALWYS3, ,DYAMD,K2,P,C,E   , ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:020,5,ALWYS3, ,LAANE,K2,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,B,FL260,FL220,     , ,260,    ,   , , , , , , , , ;
STAR:010,6,ALWYS3,RW10B,LAANE,K2,P,C,E   , ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:020,6,ALWYS3,RW10B,ARRTU,K2,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,10000,     ,     , ,240,    ,   , , , , , , , , ;
STAR:010,6,ALWYS3,RW28B,LAANE,K2,P,C,E   , ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:020,6,ALWYS3,RW28B,SERFR,K2,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,12000,     ,     , ,250,    ,   , , , , , , , , ;";
    let route: Vec<String> = ["OAL", "RUSME", "DYAMD"].iter().map(|s| s.to_string()).collect();
    let c = stars_for_route_in(ksfo.lines(), "28L", &route);
    let fixes: Vec<&str> = c[0].legs.iter().map(|l| l.fix_ident.as_str()).collect();
    assert_eq!(c[0].entry, "RUSME");
    assert_eq!(fixes, ["RUSME", "DYAMD", "LAANE", "SERFR"]);
    let c = stars_for_route_in(ksfo.lines(), "10R", &route);
    assert_eq!(c[0].legs.last().map(|l| l.fix_ident.as_str()), Some("ARRTU"));
}

#[test]
fn a_sid_is_chosen_for_the_runway_in_use_and_the_route() {
    let text = r#"SID:005,2,BRL2Y,RW10, , , , ,    , ,   ,CD, ,BRL,LI,D, ,      ,    ,    ,1020,0032, ,     ,     ,06000, ,   ,    ,   ,BRL,LI,D, , , , , ;
SID:010,2,BRL2Y,RW10, , , , ,    , ,   ,CA, , , , , ,      ,    ,    ,1020,    ,+,01500,     ,     , ,   ,    ,   , , , , , , , , ;
SID:020,2,BRL2Y,RW10, , , , ,    ,R,   ,CI, , , , , ,      ,    ,    ,2300,    , ,     ,     ,     ,-,220,    ,   , , , , , , , , ;
SID:030,2,BRL2Y,RW10,D185L,LI,P,C,EY  , ,   ,CF, ,BRL,LI,D, ,      ,1848,0120,1850,0040,+,06000,     ,     , ,   ,    ,   , , , , , , , , ;
SID:040,2,BRL2Y,RW10,BRL,LI,D, ,VE  ,R,   ,CF,Y,BRL,LI,D, ,      ,0000,0000,0230,0120,+,FL120,     ,     ,-,220,    ,   , , , , , , , , ;
SID:010,5,LEGL1P,RW28, , , , ,    , ,010,CA, , , , , ,      ,    ,    ,2820,    ,+,01180,     ,06000,-,180,    ,   ,LIME,LI,P,A, , , , ;
SID:040,5,LEGL1P,RW28,ME609,LI,P,C,E   , ,010,CF, ,BRL,LI,D, ,      ,2262,0050,2220,0040,+,04000,     ,     ,-,180,    ,   , , , , , , , , ;
SID:050,5,LEGL1P,RW28,ME611,LI,P,C,EY  , ,010,CF, ,BRL,LI,D, ,      ,2250,0070,2220,0020,+,05000,     ,     ,-,200,    ,   , , , , , , , , ;
SID:060,5,LEGL1P,RW28,LEGLO,LI,E,A,EE  ,L,010,DF, , , , , ,      ,    ,    ,    ,    ,+,FL120,     ,     ,-,200,    ,   , , , , , , , , ;
SID:010,5,LEGL1R,RW10, , , , ,    , ,010,CA, , , , , ,      ,    ,    ,1020,    ,+,01500,     ,06000,-,200,    ,   ,LIME,LI,P,A, , , , ;
SID:020,5,LEGL1R,RW10,ME684,LI,P,C,E   ,R,010,CF, ,BRL,LI,D, ,      ,1848,0080,2300,0050, ,     ,     ,     ,-,220,    ,   , , , , , , , , ;
SID:040,5,LEGL1R,RW10,ME685,LI,P,C,EY  , ,010,CF, ,BRL,LI,D, ,      ,1848,0120,1850,0040,+,06000,     ,     ,-,220,    ,   , , , , , , , , ;
SID:050,5,LEGL1R,RW10,ME606,LI,P,C,E   ,L,010,DF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     ,-,220,    ,   , , , , , , , , ;
SID:060,5,LEGL1R,RW10,LEGLO,LI,E,A,EE  , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,FL120,     ,     ,-,220,    ,   , , , , , , , , ;
SID:010,5,LEGL1W,RW10, , , , ,    , ,010,CA, , , , , ,      ,    ,    ,1020,    ,+,01500,     ,06000,-,230,    ,   ,LIME,LI,P,A, , , , ;
SID:020,5,LEGL1W,RW10,ME686,LI,P,C,E   , ,010,DF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     ,-,230,    ,   , , , , , , , , ;
SID:040,5,LEGL1W,RW10,ME687,LI,P,C,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,03500,     ,     ,-,230,    ,   , , , , , , , , ;
SID:050,5,LEGL1W,RW10,ME606,LI,P,C,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    ,+,06000,     ,     ,-,230,    ,   , , , , , , , , ;
SID:060,5,LEGL1W,RW10,ME685,LI,P,C,E   , ,010,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     ,-,230,    ,   , , , , , , , , ;
SID:070,5,LEGL1W,RW10,LEGLO,LI,E,A,EE  ,R,010,TF, , , , , ,      ,    ,    ,    ,    ,+,FL120,     ,     ,-,230,    ,   , , , , , , , , ;"#;
    let route: Vec<String> = ["ME609", "ME611", "LEGLO", "SRN", "IXORA", "ARLES"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    let ten = sids_for_route_in(text.lines(), "10", &route);
    assert!(!ten.is_empty(), "a runway-10 SID to LEGLO exists");
    assert!(ten.iter().all(|c| c.join == "LEGLO"), "{ten:?}");
    assert_eq!(ten[0].ident, "LEGL1R", "fewest fixes first: {ten:?}");
    assert!(!ten.iter().any(|c| c.ident == "LEGL1P"), "28's SID never serves 10");
    assert!(!ten.iter().any(|c| c.ident == "BRL2Y"), "a SID off the route never joins it");
    let twenty_eight = sids_for_route_in(text.lines(), "28", &route);
    assert_eq!(twenty_eight[0].ident, "LEGL1P");
}

/// The EGKK route of sortie 1790601822 (2026-09-28, #152), its navlog
/// from the OFP: filed LAM6M off 26L, LAM L610 HEMEL.
fn egkk_egbb_route() -> Vec<String> {
    [
        "D257B", "DET31", "DET29", "ACORN", "D258K", "TOC", "D156O", "D156J", "LAM", "HEMEL",
        "BUZAD", "WELIN", "TOD", "PUFAX", "HON", "OSKOT", "GROVE",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect()
}

/// EGKK 08R, the real CIFP (X-Plane's own EGKK SIDs, trimmed to the four
/// that matter): KENE3P serves 08R but ends at KENET, off the filed
/// route; LAM5P and LAM1Z both end at LAM, and LAM5P is the shorter. The
/// tower cleared KENE3P (its exit nearest the bearing to Birmingham) and
/// the crew flew LAM5P; the one rule for both sides picks LAM5P. Off 26L
/// the filed LAM6M stands.
#[test]
fn the_sid_for_the_filed_route_at_egkk_is_the_one_that_reaches_lam() {
    let text = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/cifp/EGKK_sids.dat"));
    let route = egkk_egbb_route();
    let serving: Vec<String> =
        sid_transitions_in(text.lines(), "08R").into_iter().map(|(i, _)| i).collect();
    assert!(serving.contains(&"KENE3P".to_string()), "KENE3P serves 08R: {serving:?}");
    let all = sids_for_route_in(text.lines(), "08R", &route);
    assert!(!all.iter().any(|c| c.ident == "KENE3P"), "KENE3P never joins: {all:?}");
    let pick = sid_for_filed_route_in(text.lines(), "08R", &route, Some("LAM6M")).unwrap();
    assert_eq!((pick.ident.as_str(), pick.join.as_str()), ("LAM5P", "LAM"));
    // The filed SID, when it serves the runway, is the one.
    let pick = sid_for_filed_route_in(text.lines(), "26L", &route, Some("LAM6M")).unwrap();
    assert_eq!(pick.ident, "LAM6M");
    // The filed SID is preferred over a shorter one that joins as far.
    let pick = sid_for_filed_route_in(text.lines(), "08R", &route, Some("LAM1Z")).unwrap();
    assert_eq!(pick.ident, "LAM1Z");
    // A route no SID reaches: none, never a SID that leads elsewhere.
    let elsewhere: Vec<String> = vec!["SAM".into(), "GWC".into()];
    assert_eq!(sid_for_filed_route_in(text.lines(), "08R", &elsewhere, None), None);
}

use super::*;
use std::io::Write;

#[test]
fn runway_number_parses_designators() {
    assert_eq!(runway_number("28"), Some(28));
    assert_eq!(runway_number("16L"), Some(16));
    assert_eq!(runway_number("07R"), Some(7));
    assert_eq!(runway_number("K1"), None);
}

#[test]
fn finds_nearest_matching_runway_and_decodes_fields() {
    // Two runway-28 ILS records; one ~0nm away, one far. Expect the near one.
    let dir = std::env::temp_dir();
    let path = dir.join("xc-nav-test.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(f, "I").unwrap();
    writeln!(f, "1200 Version - test").unwrap();
    // near (lat 36.71, lon -4.55): freq 10950 -> 109.50, bearing field
    // magnetic x 360 + true: 281 x 360 + 281.9 = 101441.9
    writeln!(
        f,
        " 4  36.710000000   -4.550000000  100 10950 18 101441.900 ITST LXXX LX 28 ILS-cat-I"
    )
    .unwrap();
    // far (KSEA-ish)
    writeln!(
        f,
        " 4  47.466600000 -122.317800000  359 11075 18 123840.337 IFAR KSEA K1 28 ILS-cat-I"
    )
    .unwrap();
    f.flush().unwrap();

    let ils = find_ils(path.to_str().unwrap(), "28", 36.71, -4.55, 60.0)
        .unwrap()
        .expect("should find the near runway-28 ILS");
    assert_eq!(ils.airport, "LXXX");
    assert!(
        (ils.freq_mhz() - 109.50).abs() < 0.001,
        "freq {}",
        ils.freq_mhz()
    );
    assert!(
        (ils.course_mag - 281.0).abs() < 0.01 && (ils.course_true - 281.9).abs() < 0.01,
        "course {}",
        ils.course_mag
    );
}

#[test]
fn parses_leal_runways_and_picks_into_the_wind() {
    // Verbatim LEAL rows from the Global Airports apt.dat, plus a following
    // airport to prove the block scan stops.
    let path = std::env::temp_dir().join("xc-apt-test.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(f, "1    142 0 0 LEAL Alicante").unwrap();
    writeln!(f, "100 45.33 27 727 0.10 1 3 0  10 38.2845188 -0.5750054 0 0 3 3 0 2 28 38.2797989 -0.5412058 0 0 3 3 0 2").unwrap();
    // Comm rows: an 8.33-style row (kHz) and a legacy row (10 kHz units).
    writeln!(f, "1054 118250 Tower Freq").unwrap();
    writeln!(f, "50 12210 ATIS").unwrap();
    writeln!(f, "1   1027 0 0 0MU1 Sunderland").unwrap();
    writeln!(f, "100 22.86 3 0 0.25 0 0 0  09 37.2471001 -94.1770610 0 0 1 0 0 0 27 37.2469784 -94.1700162 0 0 1 0 0 0").unwrap();
    writeln!(f, "1054 122800 Sunderland Tower").unwrap();
    f.flush().unwrap();

    // Field elevation and name come off the header row (LEAL sits at 142 ft).
    let apt = find_airport(path.to_str().unwrap(), "LEAL")
        .unwrap()
        .expect("found");
    assert_eq!(apt.elev_ft, 142.0);
    assert_eq!(apt.name, "Alicante");
    // Frequencies: only LEAL's, decoded to kHz whatever the row era.
    assert_eq!(apt.freqs.len(), 2, "LEAL's two comm rows, not Sunderland's");
    let twr = apt.freq(FREQ_TOWER).expect("tower row");
    assert_eq!((twr.khz, twr.name.as_str()), (118250, "Tower Freq"));
    assert_eq!(
        apt.freq(FREQ_ATIS).expect("atis row").khz,
        122100,
        "legacy 10 kHz units"
    );
    assert!(apt.freq(FREQ_GROUND).is_none());
    let rw = find_runways(path.to_str().unwrap(), "LEAL").unwrap();
    assert_eq!(rw.len(), 1, "one physical runway, not Sunderland's");
    assert_eq!(
        (rw[0].ident_a.as_str(), rw[0].ident_b.as_str()),
        ("10", "28")
    );
    // Geometry sanity: the 10 end points roughly east-southeast.
    assert!(
        (rw[0].heading_a_deg() - 100.0).abs() < 8.0,
        "{}",
        rw[0].heading_a_deg()
    );

    // The hallucination case: wind 157 at 6 (today's LEAL METAR). The LLM
    // said "runway 18"; the field has 10 and 28, and 10 is more into wind.
    let (ident, _, headwind) = best_runway_for_wind(&rw, 157.0, 6.0).expect("a pick");
    assert_eq!(ident, "10");
    assert!(headwind > 0.0, "into wind, not tailwind: {headwind}");
    // Westerly wind flips it.
    assert_eq!(best_runway_for_wind(&rw, 280.0, 10.0).unwrap().0, "28");
    // Unknown airport → empty, and the chooser refuses to invent.
    let none = find_runways(path.to_str().unwrap(), "LEMD").unwrap();
    assert!(none.is_empty());
    assert!(best_runway_for_wind(&none, 157.0, 6.0).is_none());
}

#[test]
fn a_stand_is_entered_along_its_parking_line() {
    let stand = Stand {
        lat: 51.610610,
        lon: 8.617635,
        heading: 237.5,
        name: "7C".into(),
        kind: "tie_down".into(),
        accepts: vec![],
    };
    let mpl = 111_320.0 * stand.lat.to_radians().cos();
    let h = 237.5_f64.to_radians();
    // A point `along` metres back up the parking line and `side` metres
    // to its right.
    let at = |along: f64, side: f64| {
        let (n, e) = (-along * h.cos() - side * h.sin(), -along * h.sin() + side * h.cos());
        (stand.lat + n / 111_320.0, stand.lon + e / mpl)
    };
    // Arriving from behind the line and off to one side: entered along
    // it through the back point, arriving on 237.5.
    let pts = stand_lead_in(at(70.0, 60.0), &stand, mpl);
    assert_eq!(pts.len(), 3, "via the parking line");
    let last = bearing_deg(pts[1].0, pts[1].1, pts[2].0, pts[2].1);
    assert!(sim::angles::abs_diff_deg(last, 237.5) < 1.0, "arrives on 237.5: {last:.1}");
    let back = haversine_nm(pts[1].0, pts[1].1, pts[2].0, pts[2].1) * 1852.0;
    assert!((back - STAND_ALIGN_M).abs() < 0.5, "{back:.1} m back");
    // EDLP 7C (2026-09-24): the route ends WEST of the stand, so the back
    // point needs a 158 degree reversal; LOWI circled on exactly that.
    // Straight in instead.
    assert_eq!(stand_lead_in((51.610600, 8.616800), &stand, mpl).len(), 2);
    // Already on the line: straight.
    assert_eq!(stand_lead_in(at(60.0, 0.0), &stand, mpl).len(), 2);
}

/// LOWI (2026-09-24): a circling procedure (no runway fix), an RF arc
/// before the threshold, and a straight-in whose missed approach must
/// not be flown as the approach.
#[test]
fn only_a_straight_in_to_this_runway_is_an_approach() {
    let leg = |t: &str, f: &str| ProcLeg {
        seq: 0,
        transition: String::new(),
        leg_type: t.into(),
        fix_ident: f.into(),
        fix_region: String::new(),
        fix: None,
        course_mag: None,
        dist_nm: None,
        turn: None,
        alt_desc: ' ',
        alt1_ft: None,
        alt2_ft: None,
        is_faf: false,
        is_map: false,
        vpa_deg: None,
        navaid_ident: String::new(),
        navaid_region: String::new(),
        navaid: None,
        path: Default::default(),
    };
    let circling = vec![leg("IF", "WI813"), leg("TF", "WI814"), leg("TF", "WI006"), leg("TF", "RTT")];
    assert!(straight_in_legs(circling, "08").is_none());
    let arc = vec![leg("IF", "WI749"), leg("RF", "WI752"), leg("TF", "RW08"), leg("TF", "RTT")];
    assert!(straight_in_legs(arc, "08").is_none());
    let z = vec![leg("IF", "WI002"), leg("TF", "WI103"), leg("TF", "RW26"), leg("TF", "WI005"), leg("HM", "RTT")];
    let got = straight_in_legs(z.clone(), "26").expect("straight-in");
    assert_eq!(got.last().unwrap().fix_ident, "RW26", "missed approach dropped");
    assert_eq!(got.len(), 3);
    // A "26" approach is not the grass 26R's.
    assert!(straight_in_legs(z, "26R").is_none());
}

#[test]
fn an_airports_msa_is_its_highest_sector() {
    let file = " 1  LSZS LS LSZS M 261 155 25 045 135 25 000 000  0
 1  LOWI LO LOWI M 180 107 25 270 135 25 015 143 25 100 113 25 000 000  0
 2   RTT LO LOWI M 270 142 25 045 116 25 120 103 25 000 000  0
 2   MNW ED EDDM M 180 037 25 000 000  0";
    assert_eq!(airport_msa_in(file.lines(), "LSZS"), Some(15_500.0));
    assert_eq!(airport_msa_in(file.lines(), "LOWI"), Some(14_300.0), "its own record");
    assert_eq!(airport_msa_in(file.lines(), "EDDM"), Some(3_700.0), "a fix's record");
    assert_eq!(airport_msa_in(file.lines(), "EGLL"), None);
}

/// LOWI MOGT3H (AIRAC 2609): its RW26 transition's minimums, in order.
#[test]
fn a_sids_minimums_are_read_for_its_runway() {
    let file = "\
SID:010,5,MOGT3H,RW26,WI505,LO,P,C,E   , ,   ,CF, ,OEV,LO,D, ,      ,2570,0030,2570,0030,+,04000,     ,18000,-,210,    ,   , , , , , , , , ;
SID:020,5,MOGT3H,RW26,WI506,LO,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,07850,     ,18000,-,210,    ,   , , , , , , , , ;
SID:030,5,MOGT3H,RW26,WI507,LO,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,08400,     ,18000, ,   ,    ,   , , , , , , , , ;
SID:010,5,MOGT3H,RW08,WI900,LO,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,09999,     ,18000, ,   ,    ,   , , , , , , , , ;
SID:100,5,MOGT3H,RW26,MOGTI,LO,E,A,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,-,13000,     ,18000, ,   ,    ,   , , , , , , , , ;";
    let m = sid_minimums_in(file.lines(), "MOGT3H", "26");
    let got: Vec<(&str, f64)> = m.iter().map(|m| (m.fix.as_str(), m.min_ft)).collect();
    assert_eq!(got, vec![("WI505", 4000.0), ("WI506", 7850.0), ("WI507", 8400.0)]);
    assert!(sid_minimums_in(file.lines(), "OTHER1A", "26").is_empty());
}

/// Rows from the installed `earth_mora.dat` (cycle 2406).
#[test]
fn the_mora_grid_is_read_by_the_cells_south_west_corner() {
    let file = "I\n1150 Version - data cycle 2406\n\n\
+27 +060 095 113 112 097 087 099 108 101 022 020 024 025 025 026 038 048 039 026 019 019 019 029 083 098 101 167 311 310 305 263
+47 +000 035 035 035 044 044 044 078 085 112 114 138 140 150 136 110 100 070 040 052 059 059 040 049 068 096 094 083 029 033 029
+55 -030 010 010 010 010 010 010 010 010 010 010 010 010 010 010 010 010 010 010 010 010 013 038 038 040 042 041 041 041 037 014";
    let g = MoraGrid::parse(file.lines());
    assert_eq!(g.at(27.99, 86.93), Some(31_100.0), "Everest");
    assert_eq!(g.at(47.26, 11.35), Some(14_000.0), "Innsbruck");
    assert_eq!(g.at(55.95, -3.36), Some(4_100.0), "Edinburgh");
    assert_eq!(g.at(10.0, 10.0), None);
    assert_eq!(mora_ground_ft(31_100.0), 29_100.0);
    assert_eq!(mora_ground_ft(4_100.0), 3_100.0);
    assert_eq!(mora_ground_ft(1_000.0), 0.0);
}

/// EGPH AGPE1E and KSFO ALWYS3 (AIRAC 2609): an "at", a speed limit, a
/// window, a runway transition, and an enroute transition not flown.
#[test]
fn a_stars_restrictions_are_read_for_the_fixes_on_the_route() {
    let egph = "\
STAR:010,5,AGPE1E,ALL,AGPED,EG,E,A,E   , ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,FL260,     ,     , ,   ,    ,   , , , , , , , , ;
STAR:020,5,AGPE1E,ALL,HAVEN,EG,E,A,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     ,-,250,    ,   , , , , , , , , ;
STAR:030,5,AGPE1E,ALL,TARTN,EG,P,C,EE H, ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,FL070,     ,     ,-,230,    ,   , , , , , , , , ;";
    let c = star_constraints_in(egph.lines(), "AGPE1E", None, &["AGPED", "HAVEN", "TARTN"]);
    assert_eq!(c.len(), 3);
    assert_eq!((c[0].min_ft, c[0].max_ft, c[0].max_kts), (Some(26_000.0), Some(26_000.0), None));
    assert_eq!((c[1].min_ft, c[1].max_ft, c[1].max_kts), (None, None, Some(250.0)));
    assert_eq!((c[2].max_ft, c[2].max_kts), (Some(7_000.0), Some(230.0)));
    assert_eq!(c[2].clamp_ft(9_500.0), 7_000.0);
    let partial = star_constraints_in(egph.lines(), "AGPE1E", None, &["TARTN"]);
    assert_eq!(partial.len(), 1, "fixes the route does not fly are not constraints");

    let ksfo = "\
STAR:010,4,ALWYS3,INYOE,INYOE,K2,E,A,E  H, ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,18000, ,   ,    ,   , , , , , , , , ;
STAR:020,4,ALWYS3,INYOE,DYAMD,K2,P,C,EE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,FL270,     ,     , ,270,    ,   , , , , , , , , ;
STAR:020,4,ALWYS3,RUSME,DYAMD,K2,P,C,EE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,FL300,     ,     , ,270,    ,   , , , , , , , , ;
STAR:020,5,ALWYS3, ,LAANE,K2,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,B,FL260,FL220,     , ,260,    ,   , , , , , , , , ;
STAR:030,6,ALWYS3,RW10B,ARRTU,K2,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,10000,     ,     , ,240,    ,   , , , , , , , , ;
STAR:030,6,ALWYS3,RW28B,ARRTU,K2,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,12000,     ,     , ,250,    ,   , , , , , , , , ;";
    let route = ["INYOE", "DYAMD", "LAANE", "ARRTU"];
    let c = star_constraints_in(ksfo.lines(), "ALWYS3", Some("28L"), &route);
    let dyamd = c.iter().find(|c| c.fix == "DYAMD").unwrap();
    assert_eq!(dyamd.min_ft, Some(27_000.0), "the INYOE transition, not RUSME's");
    let laane = c.iter().find(|c| c.fix == "LAANE").unwrap();
    assert_eq!((laane.min_ft, laane.max_ft), (Some(22_000.0), Some(26_000.0)));
    assert_eq!(laane.clamp_ft(30_000.0), 26_000.0);
    assert_eq!(laane.clamp_ft(15_000.0), 22_000.0);
    let arrtu = c.iter().find(|c| c.fix == "ARRTU").unwrap();
    assert_eq!((arrtu.max_ft, arrtu.max_kts), (Some(12_000.0), Some(250.0)), "28L's, not 10's");
    let unknown = star_constraints_in(ksfo.lines(), "ALWYS3", None, &route);
    assert!(unknown.iter().all(|c| c.fix != "ARRTU"), "no runway, no runway transition");
}

/// LSZS RNAV 03 (AIRAC 2609): a final to a MAP two miles short, then a
/// visual segment; RNAV 21's MAP is 0.7 nm out at 5.3 degrees, too steep.
#[test]
fn a_map_approach_is_judged_on_its_visual_segment() {
    let leg = |id: &str, lat: f64, lon: f64, map: bool, alt: Option<f64>| ProcLeg {
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
        alt1_ft: alt,
        alt2_ft: None,
        is_faf: false,
        is_map: map,
        vpa_deg: None,
        navaid_ident: String::new(),
        navaid_region: String::new(),
        navaid: None,
        path: Default::default(),
    };
    let raw = vec![
        leg("ZS701", 46.3265, 9.575944, false, Some(12400.0)),
        leg("ZS702", 46.357417, 9.687639, false, Some(11800.0)),
        leg("ZS705", 46.5, 9.850667, true, Some(6598.0)),
        leg("ZS706", 46.567778, 9.915667, false, None),
    ];
    let legs = map_final_legs(raw.clone()).expect("ends at a MAP");
    assert_eq!(legs.last().unwrap().fix_ident, "ZS705", "missed approach cut");
    let seg = visual_segment(&legs, (46.5270195, 9.8780890), 28.9, 5602.0).unwrap();
    assert!((seg.dist_nm - 1.98).abs() < 0.05, "{seg:?}");
    assert!(seg.lateral_nm < 0.25 && seg.course_diff_deg < 12.0, "{seg:?}");
    assert!((seg.angle_deg - 4.74).abs() < 0.05, "{seg:?}");
    assert!(seg.flyable());
    // Too steep: RNAV 21's MAP, 394 ft up at 0.7 nm.
    let steep = VisualSegment { dist_nm: 0.7, lateral_nm: 0.05, course_diff_deg: 15.0, angle_deg: 5.3 };
    assert!(!steep.flyable());
    // A runway fix first is a straight-in, not a MAP approach.
    let mut rw = raw;
    rw.insert(2, leg("RW03", 46.527, 9.878, false, Some(5602.0)));
    assert!(map_final_legs(rw).is_none());
}

/// EGKK's two strips from the real apt.dat rows. Their headings differ
/// by 0.005 degrees (257.629 and 257.624 true), so at 175/7.7 (sortie
/// 1790524628, 2026-09-27) 26R faced the wind by 0.0007 kt more and won:
/// the IFR clearance named the standby. The wind picks the direction;
/// the longest strip facing it is the runway, in any wind.
#[test]
fn the_wind_picks_a_direction_and_the_main_parallel_takes_it() {
    let rw = vec![
        Runway {
            ident_a: "08R".into(),
            ident_b: "26L".into(),
            lat_a: 51.1450993,
            lon_a: -0.2123748,
            lat_b: 51.1514994,
            lon_b: -0.1659296,
            width_m: 46.0,
            surface: 26,
        },
        Runway {
            ident_a: "08L".into(),
            ident_b: "26R".into(),
            lat_a: 51.1468901,
            lon_a: -0.2126029,
            lat_b: 51.1518321,
            lon_b: -0.1767408,
            width_m: 46.0,
            surface: 23,
        },
    ];
    assert!(rw[0].length_m() > 3300.0 && rw[1].length_m() < 2600.0);
    let (id, hdg, hw) = best_runway_for_wind(&rw, 174.7, 7.7).unwrap();
    assert_eq!(id, "26L");
    assert!((hdg - 257.63).abs() < 0.01, "{hdg}");
    assert!(
        (hw - 0.948).abs() < 0.001,
        "the headwind is 26L's own: {hw}"
    );
    for from in (0..360).step_by(5) {
        let id = best_runway_for_wind(&rw, from as f64, 12.0).unwrap().0;
        assert!(id == "26L" || id == "08R", "wind {from}: {id}");
    }
}

#[test]
fn the_wind_pick_hands_out_the_main_runway_not_the_grass_strip() {
    // ETND's own two rows (2026-09-24): 08L/26R grass, 446 m; 08/26
    // asphalt, 1,240 m. At 260/6 the grass strip faces the wind better.
    let grass = parse_runway_row(
        "50.00 3 0 0.00 0 0 0 08L 52.5862935 8.3393953 0 0 1 0 0 0 26R 52.5866670 8.3459906 0 0 1 0 0 0"
            .split_whitespace(),
    )
    .unwrap();
    let paved = parse_runway_row(
        "45.11 1 0 0.25 0 2 0 08 52.5850000 8.3318780 12 0 1 0 0 0 26 52.5859870 8.3501930 0 0 1 3 0 0"
            .split_whitespace(),
    )
    .unwrap();
    assert!(!grass.is_hard() && paved.is_hard());
    let rw = vec![grass.clone(), paved];
    assert_eq!(best_runway_for_wind(&rw, 260.0, 6.0).unwrap().0, "26");
    assert_eq!(best_runway_for_wind(&rw, 80.0, 6.0).unwrap().0, "08");
    // A grass field still gets its grass runway.
    assert_eq!(best_runway_for_wind(&[grass], 260.0, 6.0).unwrap().0, "26R");
}

#[test]
fn taxi_net_parses_routes_and_names_the_crossings() {
    // A toy field: runway 09/27 east-west, a stand north of it, taxiway
    // A south from the stand, a short runway-flagged segment (the 14/32
    // crossing), then taxiway B west to the 09 holding point.
    //
    //   stand(1)
    //     | A
    //    (2)
    //     | 14/32  <- crossing
    //    (3)
    //     \___ B ___ (4)   [near the 09 threshold]
    //
    let path = std::env::temp_dir().join("xc-taxi-test.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(f, "1    50 0 0 LTST Testfield").unwrap();
    writeln!(f, "100 45.33 27 727 0.10 1 3 0  09 36.0000000 -5.0100000 0 0 3 3 0 2 27 36.0000000 -5.0000000 0 0 3 3 0 2").unwrap();
    writeln!(f, "1054 118250 Testfield Tower").unwrap();
    writeln!(f, "1201 36.0030000 -5.0050000 both 1 stand").unwrap();
    writeln!(f, "1201 36.0020000 -5.0050000 both 2 A").unwrap();
    writeln!(f, "1201 36.0010000 -5.0050000 both 3 junction").unwrap();
    writeln!(f, "1201 36.0004000 -5.0095000 both 4 B").unwrap();
    // A tail the way real Gateway networks build it: taxiway C on to the
    // node where it meets the runway CENTRELINE (7), which also anchors
    // a runway edge — the hold-short trim case.
    writeln!(f, "1201 36.0004000 -5.0140000 both 7 hp").unwrap();
    writeln!(f, "1201 36.0010000 -5.0140000 both 8 rwy").unwrap();
    writeln!(f, "1202 1 2 twoway taxiway A").unwrap();
    writeln!(f, "1202 2 3 twoway runway 14/32").unwrap();
    writeln!(f, "1202 3 4 twoway taxiway B").unwrap();
    // An `ils` zone row on B: the LVP hold, which the parser must SKIP —
    // were it collected, the zone tail below would swallow B too.
    writeln!(f, "1204 ils 09,27").unwrap();
    writeln!(f, "1202 4 7 twoway taxiway C").unwrap();
    // C lies inside 09/27's active zone: node 4 is the authored holding
    // point, the way EGLL marks every runway entry.
    writeln!(f, "1204 departure 09,27").unwrap();
    writeln!(f, "1204 arrival 09,27").unwrap();
    writeln!(f, "1202 7 8 twoway runway 09/27").unwrap();
    // Parking: a gate right by the stand node, and a hangar further out
    // that an arrival must NOT be sent to.
    writeln!(
        f,
        "1300 36.0031000 -5.0050000 180.0 gate turboprops|props Stand 1"
    )
    .unwrap();
    writeln!(f, "1300 36.0060000 -5.0050000 90.0 hangar jets Maintenance").unwrap();
    writeln!(f, "1 1027 0 0 0MU1 Sunderland").unwrap();
    writeln!(f, "1202 5 6 twoway taxiway Z").unwrap();
    f.flush().unwrap();

    let net = find_taxi_net(path.to_str().unwrap(), "LTST")
        .unwrap()
        .expect("network found");
    assert_eq!(net.nodes.len(), 6);
    assert_eq!(net.edges.len(), 5, "only LTST's edges, not Sunderland's");

    // Parking: both stands parse; the nearest SUITABLE one (from a point
    // near the gate) is the gate, never the hangar.
    assert_eq!(net.stands.len(), 2, "gate + hangar");
    let near = net.nearest_stand(36.0031, -5.0050).expect("a stand");
    assert_eq!(near.name, "Stand 1");
    assert!(near.is_suitable());
    assert!(net
        .stands
        .iter()
        .any(|s| s.kind == "hangar" && !s.is_suitable()));
    assert_eq!(
        near.accepts,
        vec!["turboprops".to_string(), "props".to_string()]
    );

    // From the stand to the 09 threshold: via A and B, crossing 14/32,
    // holding point named for the last taxiway before the runway.
    let plan = net
        .plan(
            36.0031,
            -5.0050,
            36.0000,
            -5.0100,
            HOLD_SHORT_STANDOFF_M,
            &[],
        )
        .expect("a route exists");
    assert_eq!(plan.via, vec!["A".to_string(), "B".to_string()]);
    assert_eq!(plan.crossings, vec!["14/32".to_string()]);
    assert_eq!(plan.holding_point, "B");
    assert!(plan.path.len() >= 4, "polyline for the compliance watch");
    // The polyline starts AT the aircraft — the apron leg from the stand
    // to the snapped node is part of the cleared route.
    assert_eq!(plan.path[0], (36.0031, -5.0050));
    assert_eq!(plan.seg.len(), plan.path.len() - 1, "seg stays aligned");
    assert_eq!(plan.seg[0], (String::new(), false));

    // To a threshold whose route ends at the runway-junction node (7):
    // the tail is pulled back by the hold-short standoff, so the route
    // ends where an aircraft STOPS, never on the runway (an untrimmed
    // route parked the auto taxi's nose across the hold line,
    // 2026-07-20). Node 4's route above stays untrimmed — it never
    // touches a runway edge.
    let plan2 = net
        .plan(
            36.0031,
            -5.0050,
            36.0004,
            -5.0141,
            HOLD_SHORT_STANDOFF_M,
            &[],
        )
        .expect("a route exists");
    assert_eq!(plan2.holding_point, "C");
    let &(llat, llon) = plan2.path.last().unwrap();
    let dx = (llon - (-5.0140)) * 111_320.0 * 36.0_f64.to_radians().cos();
    let dy = (llat - 36.0004) * 111_320.0;
    let d = (dx * dx + dy * dy).sqrt();
    assert!(
        (35.0..=45.0).contains(&d),
        "stops ~40 m short of the junction: {d}"
    );
    assert_eq!(
        plan2.seg.len(),
        plan2.path.len() - 1,
        "seg stays aligned after the trim"
    );
    // The trimmed tail SURVIVES as the lead-on: from the hold-short point
    // to the runway-junction node — the painted line the line-up drives.
    assert!(!plan2.lead_on.is_empty(), "geometric trim keeps its tail");
    assert_eq!(
        *plan2.lead_on.first().unwrap(),
        *plan2.path.last().unwrap(),
        "starts at the hold"
    );
    assert_eq!(
        *plan2.lead_on.last().unwrap(),
        (36.0004, -5.0140),
        "ends at the junction"
    );
    // An untrimmed route (node 4, never touches a runway edge) has none.
    assert!(plan.lead_on.is_empty(), "no trim, no lead-on");
    // An instrument-scale standoff trims further back along the same
    // route (Heathrow's CAT holds sit ~100 m out; 40 m parked a taxi on
    // the entry stub past the line, 2026-07-20).
    let cat = net
        .plan(36.0031, -5.0050, 36.0004, -5.0141, 100.0, &[])
        .expect("a route exists");
    let &(clat, clon) = cat.path.last().unwrap();
    let dx = (clon - (-5.0140)) * 111_320.0 * 36.0_f64.to_radians().cos();
    let dy = (clat - 36.0004) * 111_320.0;
    let d = (dx * dx + dy * dy).sqrt();
    assert!(
        (95.0..=105.0).contains(&d),
        "instrument standoff honoured: {d}"
    );
    // AUTHORED active zones beat both guesses: with the destination
    // runway named, the route ends at the zone boundary node — the
    // per-stub holding point as the airport author placed it. The `ils`
    // row on B must NOT extend the zone (LVP holds are future work).
    let zone: Vec<String> = vec!["09".into(), "27".into()];
    let zoned = net
        .plan(
            36.0031,
            -5.0050,
            36.0004,
            -5.0141,
            HOLD_SHORT_STANDOFF_M,
            &zone,
        )
        .expect("a route exists");
    assert_eq!(
        *zoned.path.last().unwrap(),
        (36.0004, -5.0095),
        "ends at node 4, the zone edge"
    );
    assert_eq!(
        zoned.seg.len(),
        zoned.path.len() - 1,
        "seg aligned after the zone trim"
    );

    // Nowhere near the network: no snap, no plan — never a wild route.
    assert!(net
        .plan(37.0, -6.0, 36.0, -5.01, HOLD_SHORT_STANDOFF_M, &[])
        .is_none());
    // A field without 1201/1202 rows has no network at all.
    assert!(find_taxi_net(path.to_str().unwrap(), "0MU1")
        .unwrap()
        .is_some());
    assert!(find_taxi_net(path.to_str().unwrap(), "LXXX")
        .unwrap()
        .is_none());
}

/// Southampton's shape, which made a clearance to nowhere (2026-07-30).
///
/// EGHI's parallel taxiway runs 170 m abeam the strip and its north end —
/// the A5 stub — is a DEAD END, connected to no runway edge. Being the
/// taxiway node nearest the runway 20 threshold (631 m, against 1047 m for
/// the nearest real entrance) it won the plain snap, so the tower cleared
/// us to "holding point Alpha five runway two zero" and the drive stopped
/// 172 m abeam a runway it could not reach. A runway destination has to
/// snap to a node the aircraft can get ONTO the runway from.
#[test]
fn a_runway_route_ends_at_an_entrance_not_the_nearest_dead_end() {
    // Runway 09/27 along lat 36.0000; the 27 threshold is the east end.
    // A parallel taxiway 111 m south, its EAST end (22) a dead-end stub
    // that is much the nearest taxiway node to that threshold. The only
    // real entrance is the stub 20->10, out at the west end.
    //
    //         09 thr                                      27 thr
    //   ---------(10)=========(11)------------------------- runway
    //             |                                (10,11 centreline)
    //            (20)-------(21)-------(22)   <- dead end, nearest
    //             |
    //            (30) stand
    let path = std::env::temp_dir().join("xc-taxi-deadend.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(f, "1    50 0 0 LTDE Deadend").unwrap();
    writeln!(f, "100 45.33 27 727 0.10 1 3 0  09 36.0000000 -5.0100000 0 0 3 3 0 2 27 36.0000000 -5.0000000 0 0 3 3 0 2").unwrap();
    writeln!(f, "1201 36.0000000 -5.0080000 both 10 rwy_w").unwrap();
    writeln!(f, "1201 36.0000000 -5.0030000 both 11 rwy_e").unwrap();
    writeln!(f, "1201 35.9990000 -5.0080000 both 20 hp").unwrap();
    writeln!(f, "1201 35.9990000 -5.0040000 both 21 par").unwrap();
    writeln!(f, "1201 35.9990000 -5.0015000 both 22 stub").unwrap();
    writeln!(f, "1201 35.9985000 -5.0075000 both 30 stand").unwrap();
    writeln!(f, "1202 10 11 twoway runway 09/27").unwrap();
    writeln!(f, "1202 20 10 twoway taxiway A1").unwrap();
    writeln!(f, "1202 20 21 twoway taxiway A").unwrap();
    writeln!(f, "1202 21 22 twoway taxiway A5").unwrap();
    writeln!(f, "1202 30 20 twoway taxiway A1").unwrap();
    drop(f);
    let net = find_taxi_net(path.to_str().unwrap(), "LTDE")
        .unwrap()
        .expect("a network");

    let m = |a: (f64, f64), b: (f64, f64)| {
        let dx = (a.1 - b.1) * 111_320.0 * 36.0_f64.to_radians().cos();
        let dy = (a.0 - b.0) * 111_320.0;
        (dx * dx + dy * dy).sqrt()
    };
    let thr27 = (36.0000, -5.0000);
    // The trap: the dead-end stub really is nearest, by a wide margin.
    assert!(
        m((35.9990, -5.0015), thr27) < m((36.0000, -5.0080), thr27),
        "the stub is the nearest taxiway node — that is the whole trap"
    );

    // Routing to the runway ends at the entrance, hold-short trimmed off
    // the junction, with the drawn lead-on kept for the line-up.
    let plan = net
        .plan_to_runway(35.9985, -5.0075, thr27.0, thr27.1, 40.0, &[], None)
        .expect("a route exists");
    let end = *plan.path.last().unwrap();
    assert!(
        m(end, (36.0000, -5.0080)) < 50.0,
        "ends near the entrance stub, not the dead end: {end:?}"
    );
    assert!(
        m(end, (35.9990, -5.0015)) > 300.0,
        "and nowhere near the A5 dead end"
    );
    assert_eq!(plan.holding_point, "A1");
    assert!(
        !plan.lead_on.is_empty(),
        "the entrance touches a runway edge, so the trim keeps a lead-on \
             for the line-up to drive — the dead end had none to keep"
    );
    assert_eq!(plan.seg.len(), plan.path.len() - 1, "seg stays aligned");

    // A POINT destination keeps the plain nearest-node snap: the off-route
    // replan aims at the standing route's already-trimmed holding point,
    // and must not be dragged back onto the junction the trim removed.
    let point = net
        .plan(35.9985, -5.0075, 35.9990, -5.0016, 40.0, &[])
        .expect("a route exists");
    assert!(
        m(*point.path.last().unwrap(), (35.9990, -5.0015)) < 20.0,
        "a point goal still snaps to the node nearest it"
    );

    // A network that models no runway at all falls back rather than
    // refusing a route it would have flown before.
    let path2 = std::env::temp_dir().join("xc-taxi-norwy.dat");
    let mut f2 = std::fs::File::create(&path2).unwrap();
    writeln!(f2, "1    50 0 0 LTNR Norunway").unwrap();
    writeln!(f2, "1201 35.9990000 -5.0080000 both 20 hp").unwrap();
    writeln!(f2, "1201 35.9990000 -5.0015000 both 22 stub").unwrap();
    writeln!(f2, "1202 20 22 twoway taxiway A").unwrap();
    drop(f2);
    let bare = find_taxi_net(path2.to_str().unwrap(), "LTNR")
        .unwrap()
        .expect("a network");
    assert!(
        bare.plan_to_runway(35.9990, -5.0081, thr27.0, thr27.1, 40.0, &[], None)
            .is_some(),
        "no entrance in the data is not a reason to refuse to taxi"
    );
}

/// Liege's shape: two parallels with a link between them, and the apron
/// on one side only (2026-09-22).
///
/// A 737 landed 04R and the vacate took the link toward the parallel 04L
/// because its junction was 35 m nearer than the apron exit's, and
/// distance was the only test. It ended on the wrong side of the strip,
/// inside the other runway's authored zone, with every stand behind it -
/// so the taxi-in clearance came back across the runway and the drive was
/// asked for a 180 on a taxiway.
#[test]
fn a_vacate_turns_toward_the_apron_not_the_parallel() {
    // Two east-west strips 400 m apart, a link between them, and an apron
    // exit on the SOUTH side. Landing east on the south strip.
    //
    //        ======================== 09L/27R (north)
    //                  |
    //                 (20) link, 1 m NEARER than the apron exit
    //                  |
    //   ===(10)=======(11)=====(12)== 09R/27L (south, we land here)
    //                                  |
    //                                 (30) apron exit
    //                                  |
    //                                 (31) stand
    let path = std::env::temp_dir().join("xc-taxi-parallel.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(f, "1    50 0 0 LTPA Parallel").unwrap();
    writeln!(f, "100 45.00 1 0 0.25 0 3 0  09R 36.0000000 -5.0100000 0 0 3 3 0 1 27L 36.0000000 -4.9900000 0 0 3 3 0 1").unwrap();
    writeln!(f, "100 45.00 1 0 0.25 0 3 0  09L 36.0036000 -5.0100000 0 0 3 3 0 1 27R 36.0036000 -4.9900000 0 0 3 3 0 1").unwrap();
    writeln!(f, "1201 36.0000000 -5.0060000 both 10 rwyw").unwrap();
    writeln!(f, "1201 36.0000000 -5.0010000 both 11 link").unwrap();
    writeln!(f, "1201 36.0000000 -5.0004000 both 12 apron").unwrap();
    writeln!(f, "1201 36.0018000 -5.0010000 both 19 linkmid").unwrap();
    writeln!(f, "1201 36.0036000 -5.0010000 both 20 rwyn").unwrap();
    writeln!(f, "1201 35.9982000 -5.0004000 both 30 s1").unwrap();
    writeln!(f, "1201 35.9973000 -5.0004000 both 31 s2").unwrap();
    writeln!(f, "1202 10 11 twoway runway 09R/27L").unwrap();
    writeln!(f, "1202 11 12 twoway runway 09R/27L").unwrap();
    // The link's FIRST hop is inside our own runway's zone, exactly as
    // Liege's C3 is - so the "never onto another runway's pavement" rule
    // cannot see the trap from the junction. Only the far half carries
    // the parallel's zone, and by then the aircraft is committed.
    writeln!(f, "1202 11 19 twoway taxiway A").unwrap();
    writeln!(f, "1204 departure 09R,27L").unwrap();
    writeln!(f, "1204 arrival 09R,27L").unwrap();
    writeln!(f, "1202 19 20 twoway taxiway A").unwrap();
    writeln!(f, "1204 departure 09L,27R").unwrap();
    writeln!(f, "1204 arrival 09L,27R").unwrap();
    writeln!(f, "1202 12 30 twoway taxiway S").unwrap();
    writeln!(f, "1202 30 31 twoway taxiway S").unwrap();
    writeln!(
        f,
        "1300 35.9971000 -5.0004000 0.0 gate jets|turboprops|props Stand 1"
    )
    .unwrap();
    drop(f);
    let net = find_taxi_net(path.to_str().unwrap(), "LTPA")
        .unwrap()
        .expect("a network");
    let zone: Vec<String> = vec!["09R".into(), "27L".into()];
    // Rolling east down 09R, west of both turn-offs.
    let plan = net
        .plan_vacate(36.0000, -5.0050, 90.0, &zone, VACATE_CLEAR_M)
        .expect("a vacate plan");
    let end = *plan.path.last().unwrap();
    assert!(
        end.0 < 36.0000,
        "south, toward the apron - not north onto the parallel's link: {end:?}"
    );
    assert_eq!(plan.exit_name, "S");
    // ...and with no stands at all in the data, the old distance-only
    // rule stands: thin data degrades, it does not refuse.
    let bare_path = std::env::temp_dir().join("xc-taxi-parallel-nostand.dat");
    let text = std::fs::read_to_string(&path)
        .unwrap()
        .lines()
        .filter(|l| !l.starts_with("1300 "))
        .collect::<Vec<_>>()
        .join("\n");
    std::fs::write(&bare_path, text).unwrap();
    let bare = find_taxi_net(bare_path.to_str().unwrap(), "LTPA")
        .unwrap()
        .expect("a network");
    assert!(
        bare.plan_vacate(36.0000, -5.0050, 90.0, &zone, VACATE_CLEAR_M)
            .is_some(),
        "no stands is not a reason to refuse to vacate"
    );
}

/// Two strips that START next to each other and diverge — Le Bourget's
/// shape, which sent a departure to the wrong runway's holding point
/// (2026-09-22).
///
/// LFPB's 07 and 09 thresholds are 18 m apart. Node 201, the stub that
/// enters runway 09, sits 43 m from the 07 threshold; node 134, the stub
/// that enters 07, sits 67 m. Nearest-entrance-anywhere therefore picked
/// 09's. The tower cleared "holding point Alpha runway zero seven", the
/// active-zone trim cut the route's last three edges, and the drive
/// stopped 169 m off runway 07's centreline with a lead-on that ended
/// 34 m off it — outside both of the line-up's gates, so full auto said
/// "I couldn't get us onto the runway" about an aircraft standing exactly
/// where it had been sent. An entrance has to be an entrance to the strip
/// that was asked for.
#[test]
fn a_runway_entrance_belongs_to_the_runway_asked_for() {
    // Two strips from a common west end, diverging east. Each has its own
    // stub off the same taxiway; the WRONG strip's stub is nearer the
    // threshold we are routing to — the whole trap.
    //
    //          (11)===== 05/23 (diverges NE)
    //   05 thr /
    //   09 thr (10)===================== 09/27
    //           |          |
    //          (21)       (20)        <- 21 enters 05, 20 enters 09
    //           |          |
    //          (31)-------(30) taxiway A -- (40) stand
    let path = std::env::temp_dir().join("xc-taxi-diverge.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(f, "1    50 0 0 LTDV Diverge").unwrap();
    // 09/27 due east along lat 36.0000.
    writeln!(f, "100 45.00 1 0 0.25 0 3 0  09 36.0000000 -5.0100000 0 0 3 3 0 1 27 36.0000000 -5.0000000 0 0 3 3 0 1").unwrap();
    // 05/23 from 18 m north of the same west end, climbing NE.
    writeln!(f, "100 45.00 1 0 0.25 0 3 0  05 36.0001600 -5.0100000 0 0 3 3 0 1 23 36.0060000 -5.0030000 0 0 3 3 0 1").unwrap();
    writeln!(f, "1201 36.0000000 -5.0095000 both 10 rwy09").unwrap();
    writeln!(f, "1201 36.0005000 -5.0089000 both 11 rwy05").unwrap();
    writeln!(f, "1201 35.9996500 -5.0095000 both 20 hp09").unwrap();
    writeln!(f, "1201 35.9996500 -5.0090000 both 21 hp05").unwrap();
    writeln!(f, "1201 35.9993000 -5.0095000 both 30 a09").unwrap();
    writeln!(f, "1201 35.9993000 -5.0090000 both 31 a05").unwrap();
    writeln!(f, "1201 35.9993000 -5.0080000 both 40 stand").unwrap();
    writeln!(f, "1202 10 60 twoway runway 09/27").unwrap();
    writeln!(f, "1201 36.0000000 -5.0040000 both 60 rwy09e").unwrap();
    writeln!(f, "1202 11 61 twoway runway 05/23").unwrap();
    writeln!(f, "1201 36.0040000 -5.0055000 both 61 rwy05e").unwrap();
    writeln!(f, "1202 20 10 twoway taxiway A1").unwrap();
    writeln!(f, "1202 21 11 twoway taxiway A2").unwrap();
    writeln!(f, "1202 30 20 twoway taxiway A").unwrap();
    writeln!(f, "1202 31 21 twoway taxiway A").unwrap();
    writeln!(f, "1202 30 31 twoway taxiway A").unwrap();
    writeln!(f, "1202 40 30 twoway taxiway A").unwrap();
    drop(f);
    let net = find_taxi_net(path.to_str().unwrap(), "LTDV")
        .unwrap()
        .expect("a network");
    let runways = find_runways(path.to_str().unwrap(), "LTDV").unwrap();
    let five = runways
        .iter()
        .find(|r| r.ident_a == "05")
        .expect("the 05/23 strip");
    let m = |a: (f64, f64), b: (f64, f64)| {
        let dx = (a.1 - b.1) * 111_320.0 * 36.0_f64.to_radians().cos();
        let dy = (a.0 - b.0) * 111_320.0;
        dx.hypot(dy)
    };
    let thr05 = (36.00016, -5.0100);
    let (hp05, hp09) = ((35.99965, -5.0090), (35.99965, -5.0095));
    // The trap: 09's stub really is the nearer node to 05's threshold.
    assert!(
        m(hp09, thr05) < m(hp05, thr05),
        "the wrong strip's entrance is nearest — that is the whole trap"
    );

    // Without the strip, the old rule: 09's holding point for a 05
    // departure.
    let wrong = net
        .plan_to_runway(35.9993, -5.0080, thr05.0, thr05.1, 40.0, &[], None)
        .expect("a route");
    assert_eq!(
        wrong.holding_point, "A1",
        "nearest-anywhere picks 09's stub: {:?}",
        wrong.via
    );

    // With it, 05's own.
    let right = net
        .plan_to_runway(35.9993, -5.0080, thr05.0, thr05.1, 40.0, &[], Some(five))
        .expect("a route");
    assert_eq!(
        right.holding_point, "A2",
        "the strip we were asked for owns the entrance: {:?}",
        right.via
    );
    // ...and the lead-on the line-up will drive ends on 05's centreline,
    // not 34 m off somebody else's.
    let last = *right.lead_on.last().expect("a lead-on to drive");
    assert!(
        m(last, (36.0005, -5.0089)) < 5.0,
        "the lead-on ends at 05's junction node: {last:?}"
    );
}

/// A field near 36°N, so the paint tests read as a taxi diagram in
/// metres: `ll(east, north)` from the airport reference point.
const LAT0: f64 = 36.0;
const LON0: f64 = -5.0;
fn ll(east_m: f64, north_m: f64) -> (f64, f64) {
    (
        LAT0 + north_m / 111_320.0,
        LON0 + east_m / (111_320.0 * LAT0.to_radians().cos()),
    )
}

/// A network leg that CHORDS a drawn curve rides the curve, not a zigzag
/// between the two.
///
/// LEMG 13's lead-on (2026-09-26, sortie 1790441293): the network went
/// from the taxiway straight to the runway junction while the paint swept
/// round a curve up to 6.5 m out of the chord; the middle samples were
/// past the snap distance and stayed on the chord, their neighbours sat
/// on the curve, and the ridden lead-on turned -62, +49, +43 and -63
/// degrees at consecutive vertices. The line-up's projection jumped 6 and
/// 11 m in single ticks and its heading error reached -43 degrees. Here
/// the curve is a 50 degree arc of 100 m radius, whose chord sits 9.4 m
/// inside it at the middle.
#[test]
fn a_leg_that_chords_a_drawn_curve_rides_the_curve() {
    // South 60 m, a 50 degree left arc (radius 100, centre 100 m east of
    // its start), then 60 m straight on 130 degrees.
    let mut drawn: Vec<(f64, f64)> = vec![(0.0, 0.0), (0.0, -60.0)];
    for k in 1..=10 {
        let a = (50.0_f64 * k as f64 / 10.0).to_radians();
        drawn.push((100.0 - 100.0 * a.cos(), -60.0 - 100.0 * a.sin()));
    }
    let c = *drawn.last().unwrap();
    let out = 130.0_f64.to_radians();
    let d = (c.0 + 60.0 * out.sin(), c.1 + 60.0 * out.cos());
    drawn.push(d);
    let paint = vec![drawn.iter().map(|&(e, n)| ll(e, n)).collect::<Vec<_>>()];
    // The network: straight to the start of the arc (1.5 m off the
    // paint, as authored), a chord to its end, and on down the straight.
    let path = vec![ll(1.5, 0.0), ll(1.5, -60.0), ll(c.0, c.1), ll(d.0, d.1)];
    let seg = vec![
        ("A".to_string(), false),
        ("HN1".to_string(), false),
        ("HN1".to_string(), false),
    ];
    let (rode, rode_seg) = ride_paint(&paint, &path, &seg);
    assert_eq!(rode_seg.len(), rode.len() - 1, "seg stays aligned with path");
    let xy = |p: (f64, f64)| {
        (
            (p.1 - LON0) * 111_320.0 * LAT0.to_radians().cos(),
            (p.0 - LAT0) * 111_320.0,
        )
    };
    let pts: Vec<(f64, f64)> = rode.iter().map(|&p| xy(p)).collect();
    // Every point ON the drawn line.
    let on_paint = |p: (f64, f64)| {
        drawn.windows(2).any(|w| {
            let (ax, ay, bx, by) = (w[0].0, w[0].1, w[1].0, w[1].1);
            let (dx, dy) = (bx - ax, by - ay);
            let t = (((p.0 - ax) * dx + (p.1 - ay) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
            ((ax + t * dx - p.0).powi(2) + (ay + t * dy - p.1).powi(2)).sqrt() < 1.0
        })
    };
    for p in &pts {
        assert!(on_paint(*p), "a point off the drawn line: {p:?}");
    }
    // And it runs one way: no vertex turns more than the arc's own
    // sampling does.
    let mut prev: Option<f64> = None;
    for w in pts.windows(2) {
        let (dx, dy) = (w[1].0 - w[0].0, w[1].1 - w[0].1);
        if dx.hypot(dy) < 0.5 {
            continue;
        }
        let brg = dx.atan2(dy).to_degrees().rem_euclid(360.0);
        if let Some(p) = prev {
            let turn = sim::angles::signed_diff_deg(brg, p);
            assert!(turn.abs() < 12.0, "a {turn:.0} degree vertex at {w:?}");
        }
        prev = Some(brg);
    }
    // The names still change where the network said.
    assert_eq!(rode_seg.first().unwrap().0, "A");
    assert_eq!(rode_seg.last().unwrap().0, "HN1");
}

/// The vacate leads, the stand route follows from its end: one point where
/// they meet, legs aligned, the exit named first once, crossings the
/// stand route's own (EDLP 24, 2026-09-24).
#[test]
fn a_stand_route_from_the_runway_starts_with_the_vacate() {
    let vacate = VacatePlan {
        path: vec![ll(0.0, 0.0), ll(100.0, 0.0), ll(100.0, -60.0)],
        seg: vec![("06/24".into(), true), ("C".into(), false)],
        exit_name: "C".into(),
    };
    let rest = TaxiRoutePlan {
        via: vec!["C".into(), "D".into()],
        crossings: vec![],
        holding_point: "7C".into(),
        path: vec![ll(100.0, -60.0), ll(100.0, -120.0), ll(200.0, -120.0)],
        seg: vec![("C".into(), false), ("D".into(), false)],
        lead_on: vec![],
    };
    let p = after_vacate(vacate.clone(), rest.clone());
    assert_eq!(p.path.len(), 5, "the meeting point once: {:?}", p.path);
    assert_eq!(p.seg.len() + 1, p.path.len());
    assert_eq!(p.path[0], ll(0.0, 0.0), "starts at the aircraft");
    assert_eq!(p.via, vec!["C".to_string(), "D".to_string()]);
    assert!(p.seg[0].1, "the rollout leg is the runway's");
    assert_eq!(p.holding_point, "7C");
    // A stand route that starts elsewhere is joined by a plain leg.
    let mut apart = rest;
    apart.path[0] = ll(100.0, -70.0);
    apart.via = vec!["D".into()];
    let q = after_vacate(vacate, apart);
    assert_eq!(q.path.len(), 6);
    assert_eq!(q.seg.len() + 1, q.path.len());
    assert_eq!(q.via, vec!["C".to_string(), "D".to_string()]);
}

/// The whole point, in one test: a network drawn beside its paint, and a
/// route that comes back ON the paint.
///
/// The geometry is Malaga's, reduced (2026-08-05). The drawn line runs
/// due east; the 1201 nodes that name it were authored 2.5 m north of it,
/// which is inside the tolerance of anyone eyeballing a WED window and
/// twenty times what the drive's own tracking error is.
#[test]
fn a_route_rides_the_drawn_line_the_network_only_approximates() {
    let paint = vec![vec![ll(0.0, 0.0), ll(400.0, 0.0)]];
    let path = vec![ll(0.0, 2.5), ll(200.0, 2.5), ll(400.0, 2.5)];
    let seg = vec![("A".to_string(), false); 2];
    let (rode, rode_seg) = ride_paint(&paint, &path, &seg);
    assert_eq!(rode_seg.len(), rode.len() - 1, "seg stays aligned with path");
    assert!(rode_seg.iter().all(|s| s.0 == "A"), "names are untouched");
    let off = |p: (f64, f64)| (p.0 - LAT0) * 111_320.0;
    for p in &rode {
        assert!(
            off(*p).abs() < 0.05,
            "every point should sit on the drawn line, not 2.5 m north of it: {:.2} m",
            off(*p)
        );
    }
    // Along-track is preserved: this moves a route sideways, never
    // shortens or re-routes it.
    let east = |p: (f64, f64)| (p.1 - LON0) * 111_320.0 * LAT0.to_radians().cos();
    assert!(east(rode[0]).abs() < 0.5 && (east(*rode.last().unwrap()) - 400.0).abs() < 0.5);
    assert!(
        rode.windows(2).all(|w| east(w[1]) > east(w[0])),
        "progress along the route must stay monotonic"
    );

    // A line that merely passes nearby is not this leg's line. Same
    // geometry, but the paint crosses at right angles: nothing snaps,
    // and the route is handed back as it was rather than dragged onto
    // the taxiway it is crossing.
    let across = vec![vec![ll(200.0, -50.0), ll(200.0, 50.0)]];
    let (unmoved, _) = ride_paint(&across, &path, &seg);
    assert!(unmoved.iter().all(|p| (off(*p) - 2.5).abs() < 0.05));

    // Too far away to be the same line: a taxiway width out is a
    // different taxiway, and a route is never invented onto one.
    let far = vec![vec![ll(0.0, -20.0), ll(400.0, -20.0)]];
    let (kept, _) = ride_paint(&far, &path, &seg);
    assert!(kept.iter().all(|p| (off(*p) - 2.5).abs() < 0.05));

    // No paint at all (thin data, or a net built without it) is not a
    // refusal — the route comes back exactly as planned.
    let (bare, bare_seg) = ride_paint(&[], &path, &seg);
    assert_eq!(bare, path);
    assert_eq!(bare_seg, seg);
}

/// An arrival is driven to the STAND, not to the taxiway node nearest it.
///
/// The network models the taxiway and stops; the apron lead-in usually
/// isn't in it at all. At Malaga that left the aircraft parked 66 m short
/// of the stand the tower had named, across a live taxiway, twice
/// (2026-08-05).
#[test]
fn an_arrival_is_driven_onto_the_stand_the_tower_named() {
    let dir = std::env::temp_dir().join(format!("xvl_stand_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("apt.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(f, "I").unwrap();
    writeln!(f, "1000 Generated").unwrap();
    writeln!(f, "1 10 0 0 TEST Test Field").unwrap();
    // A taxiway east-west; the stand sits 60 m north of its east end,
    // with nothing in the network to reach it.
    let a = ll(0.0, 0.0);
    let b = ll(200.0, 0.0);
    let stand_p = ll(200.0, 60.0);
    writeln!(f, "1201 {:.7} {:.7} both 1 A", a.0, a.1).unwrap();
    writeln!(f, "1201 {:.7} {:.7} both 2 A", b.0, b.1).unwrap();
    writeln!(f, "1202 1 2 twoway taxiway A").unwrap();
    writeln!(
        f,
        "1300 {:.7} {:.7} 0.0 gate jets|props Stand 1",
        stand_p.0, stand_p.1
    )
    .unwrap();
    drop(f);
    let net = find_taxi_net(path.to_str().unwrap(), "TEST")
        .unwrap()
        .expect("a network");
    let stand = net.stands[0].clone();
    let plan = net.plan_to_stand(a.0, a.1, &stand).expect("a route");
    let end = *plan.path.last().unwrap();
    let off = |p: (f64, f64), q: (f64, f64)| {
        let dx = (p.1 - q.1) * 111_320.0 * LAT0.to_radians().cos();
        let dy = (p.0 - q.0) * 111_320.0;
        (dx * dx + dy * dy).sqrt()
    };
    assert!(
        off(end, stand_p) < 1.0,
        "the drive must finish at the stand, not {:.0} m short on the taxiway",
        off(end, stand_p)
    );
    assert_eq!(
        plan.seg.len(),
        plan.path.len() - 1,
        "the lead-in keeps seg aligned"
    );
    assert!(
        plan.seg.last().is_some_and(|s| s.0.is_empty() && !s.1),
        "an apron lead-in is not a named taxiway: {:?}",
        plan.seg.last()
    );

    // Too far to be this route's stand: the old stop-at-the-node
    // behaviour, because driving blind across an apron is worse than
    // stopping short of it.
    let far = Stand {
        lat: ll(200.0, 300.0).0,
        lon: ll(200.0, 300.0).1,
        ..stand.clone()
    };
    let plan = net.plan_to_stand(a.0, a.1, &far).expect("a route");
    assert!(
        off(*plan.path.last().unwrap(), (far.lat, far.lon)) > 100.0,
        "a stand out of lead-in range is not driven to"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

/// A runway leg is left alone, and with it the point a vacate starts
/// from. The crossing runs square across every taxi line near it, the
/// strip's own markings are not linear features, and the aircraft's own
/// position is the one point in any route that must never move.
#[test]
fn the_snap_never_moves_a_runway_leg_or_the_start_of_a_vacate() {
    let paint = vec![vec![ll(-30.0, 0.0), ll(400.0, 0.0)]];
    let path = vec![ll(0.0, 4.0), ll(60.0, 4.0), ll(200.0, 4.0)];
    let seg = vec![("".to_string(), true), ("B".to_string(), false)];
    let (rode, _) = ride_paint(&paint, &path, &seg);
    assert_eq!(rode[0], path[0], "the runway leg keeps its start exactly");
    let off = |p: (f64, f64)| (p.0 - LAT0) * 111_320.0;
    assert!(
        off(*rode.last().unwrap()).abs() < 0.05,
        "the taxiway leg after it still rides the line"
    );
}

/// The parse: line type on the node the segment LEAVES, beziers sampled,
/// and anything that is not a centreline left out of the paint.
#[test]
fn drawn_centrelines_parse_out_of_a_linear_feature() {
    let dir = std::env::temp_dir().join(format!("xvl_paint_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("apt.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(f, "I").unwrap();
    writeln!(f, "1000 Generated").unwrap();
    writeln!(f, "1 10 0 0 TEST Test Field").unwrap();
    writeln!(f, "1201 36.0000000 -5.0000000 both 1 A").unwrap();
    writeln!(f, "1201 36.0000000 -5.0010000 both 2 A").unwrap();
    writeln!(f, "1202 1 2 twoway taxiway A").unwrap();
    // A centreline with a bezier in the middle, then a stretch of some
    // other marking, then the terminator.
    writeln!(f, "120 taxiway A centre").unwrap();
    writeln!(f, "111 36.0000000 -5.0000000 51").unwrap();
    writeln!(f, "112 36.0000000 -5.0005000 36.0000500 -5.0005000 51").unwrap();
    writeln!(f, "111 36.0000000 -5.0010000 4").unwrap();
    writeln!(f, "115 36.0000000 -5.0015000").unwrap();
    // A pavement boundary drawn with the same rows is NOT paint to drive.
    writeln!(f, "110 1 0.0 0.0 apron").unwrap();
    writeln!(f, "111 36.0100000 -5.0000000 51").unwrap();
    writeln!(f, "113 36.0100000 -5.0010000 51").unwrap();
    drop(f);
    let net = find_taxi_net(path.to_str().unwrap(), "TEST")
        .unwrap()
        .expect("a network");
    assert_eq!(net.paint.len(), 1, "one centreline run: {:?}", net.paint);
    let line = &net.paint[0];
    assert!(
        line.len() > BEZIER_STEPS,
        "the curve is sampled, not a straight chord: {}",
        line.len()
    );
    // The bezier bulges north of the chord, which is the whole reason to
    // sample it rather than join the nodes.
    let bulge = line
        .iter()
        .map(|p| (p.0 - 36.0) * 111_320.0)
        .fold(0.0_f64, f64::max);
    assert!(bulge > 0.5, "bezier ignored: {bulge:.2} m");
    // The type-4 leg (a hold bar) is not a line to taxi down, so the run
    // ends at the node where the centreline type stops — the segment
    // beyond it is drawn, but it is not a line to drive.
    assert!(
        line.iter().all(|p| p.1 >= -5.0010 - 1e-9),
        "the run must stop where the centreline type stops: {:?}",
        line.last()
    );
    assert!(
        (line.last().unwrap().1 + 5.0010).abs() < 1e-9,
        "and it must reach it: {:?}",
        line.last()
    );
    let _ = std::fs::remove_dir_all(&dir);
}

/// The sortie this came from, measured against the real file.
/// `cargo test -p atc -- --ignored live_lemg_route_rides --nocapture`
///
/// Malaga, 2026-08-05: the Baron taxied 1,214 m from its GA stand to
/// runway 13, tracking the planned route to 0.12 m and the drawn line to
/// nothing better than 1.7 m. The assertion is on the route, not on the
/// aircraft — where the route runs is the only part of that we own.
/// LIPR gate 210 to Alpha for 30 (2026-09-26): the route ended on a
/// repeated point and the stop swerved right toward "due north".
/// The real install's EGKK, the numbers of sortie 1790524628: the wind
/// pick is 26L, 26R stands for 26L, the CIFP's SID toward Birmingham off
/// 26L is a Mike departure, and the taxi from where the rejected 737
/// stopped (1790525500, 51.147885 N 0.192010 W, facing 261) reaches 26L's
/// holding point. `cargo test -p atc -- --ignored real_install_egkk --nocapture`
/// The transition levels the tower speaks flight levels from (issue
/// #97), read from the real apt.dat: EGKK 7,000, EGBB 7,500, LEBL 8,000,
/// KJFK 18,000. `cargo test -p atc -- --ignored real_install_transition_levels`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_transition_levels() {
    let apt =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    if !std::path::Path::new(apt).exists() {
        return;
    }
    for (icao, want) in [("EGKK", 7000.0), ("EGBB", 7500.0), ("LEBL", 8000.0), ("KJFK", 18000.0)] {
        let a = find_airport(apt, icao).unwrap().expect(icao);
        assert_eq!(a.transition_level_ft, Some(want), "{icao}");
    }
}

/// #182: `1302 allows_circuits` and the `1101` pattern side per runway,
/// the rows as LEMG writes them (2026-09-29 apt.dat): three flows, 13
/// left twice and 31 left; a runway whose flows disagree has no side.
#[test]
fn circuits_flag_and_pattern_sides_are_read_from_the_flows() {
    let path = std::env::temp_dir().join(format!("xc-apt-1101-{}.dat", std::process::id()));
    std::fs::write(
        &path,
        "I\n1000 Version\n\n\
             1 52 0 0 LEMG Malaga\n\
             100 45.00 1 0 0.25 0 2 1 13 36.6738 -4.4992 0 0 2 0 0 0 31 36.6624 -4.4827 0 0 2 0 0 0\n\
             1302 allows_circuits 0\n\
             1302 transition_level 7000\n\
             1000 12/13 Rwy ATC Flow\n\
             1101 13 left\n\
             1110 13 121025 departures heavy|jets 000000 000000\n\
             1000 Rwy 31 ATC Flow\n\
             1101 31 left\n\
             1101 13 left\n\
             1101 9 right\n\
             1101 09 left\n\
             1 11 0 0 LXGB Gibraltar\n\
             1302 allows_circuits 1\n\
             1101 09 left\n",
    )
    .unwrap();
    let p = path.to_str().unwrap();
    let lemg = find_airport(p, "LEMG").unwrap().unwrap();
    assert_eq!(lemg.allows_circuits, Some(false));
    assert_eq!(lemg.transition_level_ft, Some(7000.0));
    assert_eq!(lemg.pattern_left("13"), Some(true));
    assert_eq!(lemg.pattern_left("31"), Some(true));
    assert_eq!(lemg.pattern_left("09"), None, "the flows disagree");
    assert_eq!(lemg.pattern_left("12"), None, "unpublished");
    assert_eq!(lemg.freqs.len(), 0, "no row read as a frequency");
    let mut seen = Vec::new();
    scan_airports(p, |icao, a| {
        seen.push((icao.to_string(), a.allows_circuits, a.pattern_left("09")))
    })
    .unwrap();
    assert_eq!(
        seen,
        vec![
            ("LEMG".to_string(), Some(false), None),
            ("LXGB".to_string(), Some(true), Some(true)),
        ]
    );
    let _ = std::fs::remove_file(&path);
    assert_eq!(parse_allows_circuits("1"), Some(true));
    assert_eq!(parse_allows_circuits("yes"), None);
    assert!(same_runway_end("09", "9") && same_runway_end("08r", "08R"));
    assert!(!same_runway_end("08R", "08L"));
}

/// The real LEMG and EGKK: no circuits, 13/31 and 08R/26L left.
/// `cargo test -p atc -- --ignored real_install_circuits_flags`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_circuits_flags() {
    let apt =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    if !std::path::Path::new(apt).exists() {
        return;
    }
    let lemg = find_airport(apt, "LEMG").unwrap().expect("LEMG");
    assert_eq!(lemg.allows_circuits, Some(false));
    assert_eq!(lemg.pattern_left("13"), Some(true));
    assert_eq!(lemg.pattern_left("31"), Some(true));
    let egkk = find_airport(apt, "EGKK").unwrap().expect("EGKK");
    assert_eq!(egkk.allows_circuits, Some(false));
    assert_eq!(egkk.pattern_left("26L"), Some(true));
}

#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_egkk_one_runway() {
    let apt =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let (Ok(Some(egkk)), Ok(Some(egbb))) =
        (find_airport(apt, "EGKK"), find_airport(apt, "EGBB"))
    else {
        return;
    };
    let (id, _, _) = best_runway_for_wind(&egkk.runways, 174.7, 7.7).unwrap();
    assert_eq!(id, "26L");
    assert_eq!(close_parallel_main(&egkk.runways, "26R"), "26L");
    let first = |a: &str, b: &str| {
        if std::path::Path::new(a).exists() {
            a.to_string()
        } else {
            b.to_string()
        }
    };
    let cifp = first(
        &crate::test_xplane_path("Custom Data/CIFP"),
        &crate::test_xplane_path("Resources/default data/CIFP"),
    );
    let fix = first(
        &crate::test_xplane_path("Custom Data/earth_fix.dat"),
        &crate::test_xplane_path("Resources/default data/earth_fix.dat"),
    );
    let navd = first(
        &crate::test_xplane_path("Custom Data/earth_nav.dat"),
        &crate::test_xplane_path("Resources/default data/earth_nav.dat"),
    );
    let exits = sid_exits(&cifp, &fix, &navd, &egkk, "EGKK");
    let sid = sid_toward(
        &exits,
        "26L",
        egkk.position().unwrap(),
        egbb.position().unwrap(),
    );
    println!("EGKK 26L toward EGBB: {sid:?}");
    let sid = sid.expect("a SID off 26L").0;
    assert!(sid.ends_with('M') || sid.ends_with('X'), "a 26L SID: {sid}");
    let net = find_taxi_net(apt, "EGKK")
        .unwrap()
        .expect("EGKK taxi network");
    let r = egkk.runways.iter().find(|r| r.ident_b == "26L").unwrap();
    let zone = [r.ident_a.clone(), r.ident_b.clone()];
    let plan = net.plan_to_runway(
        51.147885,
        -0.192010,
        r.lat_b,
        r.lon_b,
        100.0,
        &zone,
        Some(r),
    );
    println!(
        "from the reject stop: {:?}",
        plan.as_ref()
            .map(|p| (&p.holding_point, &p.via, p.path.len()))
    );
    assert!(
        plan.is_some_and(|p| p.path.len() >= 2),
        "a taxi back from the runway"
    );
}

/// #152 on the installed data (sortie 1790601822, 2026-09-28): off 08R
/// the exit nearest Birmingham's bearing is KENE3P's, the SID the tower
/// cleared; it never joins the filed LAM L610 HEMEL, and the filed
/// route's SID off 08R is LAM5P, the one the crew flew.
/// `cargo test -p atc -- --ignored real_install_egkk_08r_filed_route --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_egkk_08r_filed_route() {
    let apt =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let (Ok(Some(egkk)), Ok(Some(egbb))) =
        (find_airport(apt, "EGKK"), find_airport(apt, "EGBB"))
    else {
        return;
    };
    let first = |a: &str, b: &str| {
        if std::path::Path::new(a).exists() {
            a.to_string()
        } else {
            b.to_string()
        }
    };
    let cifp = first(
        &crate::test_xplane_path("Custom Data/CIFP"),
        &crate::test_xplane_path("Resources/default data/CIFP"),
    );
    let fix = first(
        &crate::test_xplane_path("Custom Data/earth_fix.dat"),
        &crate::test_xplane_path("Resources/default data/earth_fix.dat"),
    );
    let navd = first(
        &crate::test_xplane_path("Custom Data/earth_nav.dat"),
        &crate::test_xplane_path("Resources/default data/earth_nav.dat"),
    );
    let exits = sid_exits(&cifp, &fix, &navd, &egkk, "EGKK");
    let toward = sid_toward(&exits, "08R", egkk.position().unwrap(), egbb.position().unwrap());
    println!("EGKK 08R toward EGBB by bearing: {toward:?}");
    assert_eq!(toward.map(|t| t.0).as_deref(), Some("KENE3P"), "the old pick");
    let serving = sid_transitions(&cifp, "EGKK", "08R");
    assert!(serving.iter().any(|(i, t)| i == "KENE3P" && t == "RW08R"), "{serving:?}");
    let route = egkk_egbb_route();
    let all = sids_for_route(&cifp, "EGKK", "08R", &route);
    println!("EGKK 08R SIDs joining the route: {all:?}");
    assert!(!all.iter().any(|c| c.ident == "KENE3P"));
    let pick = sid_for_filed_route(&cifp, "EGKK", "08R", &route, Some("LAM6M")).unwrap();
    assert_eq!((pick.ident.as_str(), pick.join.as_str()), ("LAM5P", "LAM"));
}

#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_lipr_route_has_no_zero_length_legs() {
    let p = &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let (Ok(Some(net)), Ok(Some(apt))) = (find_taxi_net(p, "LIPR"), find_airport(p, "LIPR")) else {
        return;
    };
    let r = apt.runways.iter().find(|r| r.ident_b == "30" || r.ident_a == "30").unwrap();
    let (tlat, tlon) = if r.ident_b == "30" { (r.lat_b, r.lon_b) } else { (r.lat_a, r.lon_a) };
    let zone = [r.ident_a.clone(), r.ident_b.clone()];
    let plan = net
        .plan_to_runway(44.02164715953246, 12.619642408739669, tlat, tlon, 100.0, &zone, Some(r))
        .expect("a route");
    let k = 111_320.0 * 44.0f64.to_radians().cos();
    for w in plan.path.windows(2) {
        let d = (((w[1].1 - w[0].1) * k).powi(2) + ((w[1].0 - w[0].0) * 111_320.0).powi(2)).sqrt();
        assert!(d >= 0.5, "a {d:.2} m leg");
    }
}

/// LIPK 2026-09-26: the tug's pattern left a 172 at the second point, on
/// the grass; the first is the stand it was pushed off.
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_lipk_pavement_knows_the_grass() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = find_taxi_net(path, "LIPK") else {
        return;
    };
    assert!(!net.pavement.is_empty(), "LIPK draws pavement");
    assert_eq!(net.on_pavement(44.19580628655603, 12.075011968589756), Some(true));
    assert_eq!(net.on_pavement(44.19488911712411, 12.074242855216026), Some(false));
}

/// How often a stand's lane is found, how long and how much it turns.
/// `cargo test -p atc -- --ignored live_push_lane_survey --nocapture`
#[test]
#[ignore]
fn live_push_lane_survey() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    for icao in ["LIBP", "LIPK", "LEMG", "EGLL", "EGPH", "EGSS", "EDLP", "LFPB"] {
        let Ok(Some(net)) = find_taxi_net(path, icao) else { continue };
        let (mut found, mut lens, mut turns) = (0, Vec::new(), Vec::new());
        for st in &net.stands {
            if let Some(l) = net.push_lane(st.lat, st.lon, st.heading) {
                found += 1;
                let len: f64 = l.windows(2).map(|w| haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0).sum();
                let n = l.len();
                let end = bearing_deg(l[n - 2].0, l[n - 2].1, l[n - 1].0, l[n - 1].1);
                let nose = (end + 180.0).rem_euclid(360.0);
                lens.push(len as i64);
                turns.push(sim::angles::signed_diff_deg(nose, st.heading) as i64);
            }
        }
        lens.sort();
        println!("{icao}: {found}/{} stands; lengths {:?}; turns {:?}", net.stands.len(),
                 lens.iter().step_by((lens.len() / 8).max(1)).collect::<Vec<_>>(),
                 turns.iter().take(12).collect::<Vec<_>>());
    }
}

/// LIBP stand 1 (2026-09-26): its painted lead-in, read backwards, is the
/// way out; the tug's own pattern swung the 737 into the buildings.
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_libp_stand_has_a_push_lane() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = find_taxi_net(path, "LIBP") else {
        return;
    };
    println!("stand lines: {}", net.stand_lines.len());
    let (la, lo): (f64, f64) = (42.431905167561986, 14.188689582813048);
    let k0 = 111_320.0 * la.to_radians().cos();
    for l in net.stand_lines.iter() {
        for e in [0, l.len() - 1] {
            let p = l[e];
            let d = (((p.1 - lo) * k0).powi(2) + ((p.0 - la) * 111_320.0).powi(2)).sqrt();
            if d < 40.0 {
                let q = if e == 0 { l[l.len().min(4) - 1] } else { l[l.len().saturating_sub(4)] };
                println!("end {e} of {} pts, {d:.1} m away, into brg {:.1}", l.len(), bearing_deg(q.0, q.1, p.0, p.1));
            }
        }
    }
    let lane = net.push_lane(42.431905167561986, 14.188689582813048, 171.8).expect("a lane");
    let k = 111_320.0 * 42.43f64.to_radians().cos();
    let mut along = 0.0;
    for w in lane.windows(2) {
        let (dx, dy) = ((w[1].1 - w[0].1) * k, (w[1].0 - w[0].0) * 111_320.0);
        along += (dx * dx + dy * dy).sqrt();
        println!("{:7.1} m  {:.6} {:.6}  brg {:5.1}", along, w[1].0, w[1].1, bearing_deg(w[0].0, w[0].1, w[1].0, w[1].1));
    }
    assert!(along > 20.0);
}

/// LIBP stand (2026-09-26): drawn, and nothing drawn 10 m behind it,
/// where the tug's straight leg ran a 737 into the buildings.
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_libp_stand_has_no_pavement_behind_it() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = find_taxi_net(path, "LIBP") else {
        return;
    };
    assert_eq!(net.on_pavement(42.43211496311211, 14.188648939301292), Some(true));
    assert_eq!(net.on_pavement(42.43224833216946, 14.188622900327418), Some(false));
}

/// LIPR Alpha (2026-09-26): a route ending on a repeated point. No leg
/// of a painted route is ever zero length.
#[test]
fn a_painted_route_has_no_zero_length_legs() {
    let paint = vec![vec![(44.0, 12.0), (44.0, 12.01)]];
    let path = vec![(44.0, 12.0), (44.0, 12.005), (44.0, 12.005), (44.0, 12.009), (44.0, 12.009)];
    let seg: Vec<(String, bool)> = (0..path.len() - 1).map(|_| ("A".to_string(), false)).collect();
    let (out, out_seg) = ride_paint(&paint, &path, &seg);
    assert_eq!(out_seg.len(), out.len() - 1, "one seg per leg");
    let k = 111_320.0 * 44.0f64.to_radians().cos();
    for w in out.windows(2) {
        let d = (((w[1].1 - w[0].1) * k).powi(2) + ((w[1].0 - w[0].0) * 111_320.0).powi(2)).sqrt();
        assert!(d >= 0.5, "a {d:.2} m leg in {out:?}");
    }
}

#[test]
fn pavement_holes_are_not_pavement_and_no_data_is_no_verdict() {
    let mut net = TaxiNet::default();
    assert_eq!(net.on_pavement(0.5, 0.5), None);
    let sq = |a: f64, b: f64| vec![(a, a), (a, b), (b, b), (b, a), (a, a)];
    net.pavement.push(vec![sq(0.0, 1.0), sq(0.4, 0.6)]);
    assert_eq!(net.on_pavement(0.2, 0.2), Some(true));
    assert_eq!(net.on_pavement(0.5, 0.5), Some(false), "a hole");
    assert_eq!(net.on_pavement(1.5, 0.5), Some(false));
}

#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_lemg_route_rides_the_paint() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = find_taxi_net(path, "LEMG") else {
        return; // no install, no verdict
    };
    assert!(!net.paint.is_empty(), "LEMG draws its taxi centrelines");
    let (slat, slon) = (36.682214, -4.502304); // the stand it pushed off
    let (tlat, tlon) = (36.6620, -4.5100); // runway 13 threshold, near enough
    let zone: Vec<String> = vec!["13".into(), "31".into()];
    let plan = net
        .plan_to_runway(slat, slon, tlat, tlon, 100.0, &zone, None)
        .expect("a route");
    // Distance from each route point to the nearest drawn centreline,
    // whatever direction it runs — a plain "how far off the paint is it".
    let m_per_deg_lon = 111_320.0 * slat.to_radians().cos();
    let to_paint = |p: (f64, f64)| -> f64 {
        net.paint
            .iter()
            .flat_map(|l| l.windows(2))
            .map(|w| {
                let (ax, ay) = ((w[0].1 - p.1) * m_per_deg_lon, (w[0].0 - p.0) * 111_320.0);
                let (bx, by) = ((w[1].1 - p.1) * m_per_deg_lon, (w[1].0 - p.0) * 111_320.0);
                let (dx, dy) = (bx - ax, by - ay);
                let l2 = dx * dx + dy * dy;
                let t = if l2 > 1e-6 {
                    (-(ax * dx + ay * dy) / l2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                ((ax + t * dx).powi(2) + (ay + t * dy).powi(2)).sqrt()
            })
            .fold(f64::MAX, f64::min)
    };
    let mut offs: Vec<f64> = plan.path.iter().skip(1).map(|&p| to_paint(p)).collect();
    offs.sort_by(f64::total_cmp);
    let median = offs[offs.len() / 2];
    let p90 = offs[offs.len() * 9 / 10];
    println!(
        "{} points ({} riding the line) over {:.0} m: median {:.2} m off the paint, \
             p90 {:.2} m, worst {:.2} m",
        offs.len(),
        offs.iter().filter(|d| **d < 0.5).count(),
        plan.path
            .windows(2)
            .map(|w| {
                let (dx, dy) = (
                    (w[0].1 - w[1].1) * m_per_deg_lon,
                    (w[0].0 - w[1].0) * 111_320.0,
                );
                (dx * dx + dy * dy).sqrt()
            })
            .sum::<f64>(),
        median,
        p90,
        offs.last().unwrap()
    );
    assert!(
        median < 0.5,
        "the route should ride the drawn line, median {median:.2} m off"
    );

    // And the arrival mirror, from where that sortie's taxi-in stopped:
    // the route must finish ON the stand, not at the taxiway node the
    // aircraft actually parked on (66 m short of W5, twice).
    let (plat, plon) = (36.680758, -4.502952);
    let stand = net.nearest_stand(plat, plon).expect("LEMG has stands").clone();
    let plan = net
        .plan_to_stand(plat, plon, &stand)
        .expect("a route to the stand");
    let end = *plan.path.last().unwrap();
    let short = {
        let (dx, dy) = (
            (stand.lon - end.1) * m_per_deg_lon,
            (stand.lat - end.0) * 111_320.0,
        );
        (dx * dx + dy * dy).sqrt()
    };
    println!("stand {} : route finishes {short:.1} m from it", stand.name);
    assert!(short < 5.0, "parks {short:.0} m short of stand {}", stand.name);
}

/// The real Southampton network behind the fix above.
/// `cargo test -p atc -- --ignored live_eghi_runway_20 --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_eghi_runway_20_holds_at_an_entrance() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = find_taxi_net(path, "EGHI") else {
        return; // no install, no verdict
    };
    // The GA apron stand the SF50 departed from, and the 20 threshold.
    let (slat, slon) = (50.953095, -1.358671);
    let (tlat, tlon) = (50.9589489, -1.3519495);
    let zone: Vec<String> = vec!["02".into(), "20".into()];
    let plan = net
        .plan_to_runway(slat, slon, tlat, tlon, 100.0, &zone, None)
        .expect("a route");
    let end = *plan.path.last().unwrap();
    let m = |a: (f64, f64), b: (f64, f64)| {
        let dx = (a.1 - b.1) * 111_320.0 * 50.95_f64.to_radians().cos();
        let dy = (a.0 - b.0) * 111_320.0;
        (dx * dx + dy * dy).sqrt()
    };
    println!(
        "holding point {} via {:?}, {:.0} m of route, ends {:.0} m from thr20",
        plan.holding_point,
        plan.via,
        plan.path.windows(2).map(|w| m(w[0], w[1])).sum::<f64>(),
        m(end, (tlat, tlon))
    );
    // The A5 dead end is at 50.954136,-1.357265 — 172 m abeam the strip
    // with no connection to it. Anywhere but there.
    assert!(
        m(end, (50.954136, -1.357265)) > 100.0,
        "must not end at the A5 dead-end stub: {end:?}"
    );
    assert!(
        !plan.lead_on.is_empty(),
        "a real entrance leaves the line-up a drawn lead-on"
    );
}

/// The real Liege network behind the vacate-toward-the-apron fix.
/// `cargo test -p atc -- --ignored live_eblg_vacate --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_eblg_vacate_04r_turns_toward_the_apron() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = find_taxi_net(path, "EBLG") else {
        return; // no install, no verdict
    };
    // Runway 04R, and the rolling handoff from the blackbox.
    let thr = (50.6271539, 5.4278755);
    let far = (50.6505719, 5.4650074);
    let course = bearing_deg(thr.0, thr.1, far.0, far.1);
    let ct = |p: (f64, f64)| {
        let d = haversine_nm(thr.0, thr.1, p.0, p.1) * 1852.0;
        let b = bearing_deg(thr.0, thr.1, p.0, p.1);
        d * sim::angles::signed_diff_deg(b, course).to_radians().sin()
    };
    let zone = vec!["04R".to_string(), "22L".to_string()];
    let plan = net
        .plan_vacate(50.6357615, 5.4415500, 45.2, &zone, VACATE_CLEAR_M)
        .expect("a vacate plan");
    let end = *plan.path.last().unwrap();
    println!(
        "exit {} ends {:+.0} m from 04R (stand 35 is at {:+.0})",
        plan.exit_name,
        ct(end),
        ct((50.637141, 5.449387))
    );
    // Every stand in the 13-35 group is on the positive side; C3, which
    // the old nearest-junction rule took, is the link to the parallel
    // 04L and ends 149 m out on the other one, inside 04L's own
    // departure and arrival zones.
    assert!(
        ct(end) > 0.0,
        "vacated onto the apron's side of the strip, not across to 04L: {:+.0} m",
        ct(end)
    );
    assert_ne!(plan.exit_name, "C3", "C3 goes to the other runway");
}

/// The real Le Bourget network behind the diverging-strips fix.
/// `cargo test -p atc -- --ignored live_lfpb_runway_07 --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_lfpb_runway_07_holds_on_its_own_strip() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = find_taxi_net(path, "LFPB") else {
        return; // no install, no verdict
    };
    let runways = find_runways(path, "LFPB").unwrap();
    let strip = runways
        .iter()
        .find(|r| r.ident_a == "07")
        .expect("LFPB 07/25");
    // The stand the sortie pushed off, and 07's threshold.
    let (slat, slon) = (48.95620, 2.43880);
    let (tlat, tlon) = (strip.lat_a, strip.lon_a);
    let zone: Vec<String> = vec!["07".into(), "25".into()];
    let plan = net
        .plan_to_runway(slat, slon, tlat, tlon, 100.0, &zone, Some(strip))
        .expect("a route");
    // Cross-track of a point from 07's centreline, metres.
    let course = strip.heading_a_deg();
    let ct = |p: (f64, f64)| {
        let d = haversine_nm(tlat, tlon, p.0, p.1) * 1852.0;
        let b = bearing_deg(tlat, tlon, p.0, p.1);
        d * sim::angles::signed_diff_deg(b, course).to_radians().sin()
    };
    let last = *plan.lead_on.last().expect("a lead-on to drive");
    println!(
        "holding point {} via {:?}, lead-on {} pts ending {:.1} m off 07",
        plan.holding_point,
        plan.via,
        plan.lead_on.len(),
        ct(last)
    );
    // Node 201 (the runway 09 stub, 34 m off 07) is what the old rule
    // picked; node 134 (2.5 m off 07) is 07's own. The line-up's
    // `line_up_path_from_lead_on` needs the lead-on to END on the
    // centreline it is lining up on, within 20 m.
    assert!(
        ct(last).abs() < 20.0,
        "the lead-on ends on runway 07's centreline, not runway 09's \
             stub: {:.1} m off",
        ct(last)
    );
}

/// The LIBP 04 shape, drawn small: the stub's first node beside the
/// threshold (68 m off the centreline, 71 m from the numbers) is nearer
/// than the node where the stub meets the runway (on the centreline,
/// 100 m up it). The route must still run on to the runway node, so the
/// lead-on the zone trim leaves reaches the centreline.
#[test]
fn a_runway_entrance_one_hop_off_the_strip_leads_on_to_the_centreline() {
    use std::io::Write as _;
    let path = std::env::temp_dir().join("xc-taxi-hop-entrance.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(f, "1    50 0 0 LTHP Hop").unwrap();
    writeln!(f, "100 45.00 1 0 0.10 1 3 0  09 36.0000000 -5.0100000 0 0 3 3 0 2 27 36.0000000 -4.9800000 0 0 3 3 0 2").unwrap();
    writeln!(f, "1201 36.0000000 -5.0088900 both 18 rwy_d").unwrap();
    writeln!(f, "1201 36.0000000 -4.9900000 both 19 rwy_far").unwrap();
    writeln!(f, "1201 35.9993900 -5.0097600 both 8 d_stub").unwrap();
    writeln!(f, "1201 35.9993900 -5.0080000 both 9 d_hold").unwrap();
    writeln!(f, "1201 35.9985000 -5.0060000 both 30 stand").unwrap();
    writeln!(f, "1202 18 19 twoway runway 09/27").unwrap();
    writeln!(f, "1202 18 8 twoway taxiway D").unwrap();
    writeln!(f, "1204 departure 09,27").unwrap();
    writeln!(f, "1202 8 9 twoway taxiway D").unwrap();
    writeln!(f, "1204 departure 09,27").unwrap();
    writeln!(f, "1202 9 30 twoway taxiway E").unwrap();
    drop(f);
    let net = find_taxi_net(path.to_str().unwrap(), "LTHP").unwrap().expect("a network");
    let runways = find_runways(path.to_str().unwrap(), "LTHP").unwrap();
    let strip = runways.iter().find(|r| r.ident_a == "09").expect("09/27");
    let (tlat, tlon) = (strip.lat_a, strip.lon_a);
    let m = |a: (f64, f64), b: (f64, f64)| {
        let dx = (a.1 - b.1) * 111_320.0 * 36.0_f64.to_radians().cos();
        let dy = (a.0 - b.0) * 111_320.0;
        (dx * dx + dy * dy).sqrt()
    };
    // The trap: the stub's hop is nearer the numbers than the junction.
    assert!(m((35.99939, -5.00976), (tlat, tlon)) < m((36.0, -5.00889), (tlat, tlon)));
    let zone: Vec<String> = vec!["09".into(), "27".into()];
    let plan = net
        .plan_to_runway(35.9985, -5.0060, tlat, tlon, 40.0, &zone, Some(strip))
        .expect("a route");
    let last = *plan.lead_on.last().expect("a lead-on");
    // Off the centreline is the latitude difference here.
    let off = (last.0 - 36.0).abs() * 111_320.0;
    assert!(off < 5.0, "the lead-on ends {off:.0} m off the centreline");
    assert!(m(*plan.path.last().unwrap(), (35.99939, -5.0080)) < 5.0, "the route holds at D");
    assert_eq!(plan.holding_point, "D");
}

/// LIBP 04 (2026-09-26, two 737 sorties): the line-up refused the drawn
/// lead-on as `junction_off_the_runway` at 67.9 m and drove the
/// synthetic path, a 135-degree corner off taxiway D, 6.7 and 7.5 m past
/// the centreline. `cargo test -p atc -- --ignored live_libp_runway_04 --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_libp_runway_04_lead_on_ends_on_the_centreline() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = find_taxi_net(path, "LIBP") else {
        return;
    };
    let runways = find_runways(path, "LIBP").unwrap();
    let strip = runways.iter().find(|r| r.ident_a == "04").expect("LIBP 04/22");
    let (slat, slon) = (42.430744, 14.185867);
    let (tlat, tlon) = (strip.lat_a, strip.lon_a);
    let zone: Vec<String> = vec!["04".into(), "22".into()];
    let course = strip.heading_a_deg();
    let ct = |p: (f64, f64)| {
        let d = haversine_nm(tlat, tlon, p.0, p.1) * 1852.0;
        let b = bearing_deg(tlat, tlon, p.0, p.1);
        d * sim::angles::signed_diff_deg(b, course).to_radians().sin()
    };
    for hold in [40.0, 60.0, 100.0] {
        let plan = net
            .plan_to_runway(slat, slon, tlat, tlon, hold, &zone, Some(strip))
            .expect("a route");
        let offs: Vec<String> = plan.lead_on.iter().map(|p| format!("{:.0}", ct(*p))).collect();
        println!(
            "hold {hold}: holding point {} via {:?}, route end {:.0} m off, lead-on {:?}",
            plan.holding_point,
            plan.via,
            plan.path.last().map_or(f64::NAN, |p| ct(*p)),
            offs
        );
        let last = *plan.lead_on.last().expect("a lead-on to drive");
        assert!(ct(last).abs() < 20.0, "lead-on ends {:.1} m off 04", ct(last));
    }
}

#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_heathrow_taxi_route_plans() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let t = std::time::Instant::now();
    let net = find_taxi_net(path, "EGLL")
        .unwrap()
        .expect("EGLL has a network");
    eprintln!(
        "EGLL taxi net: {} nodes, {} edges in {:?}",
        net.nodes.len(),
        net.edges.len(),
        t.elapsed()
    );
    assert!(net.nodes.len() > 500 && net.edges.len() > 300);
    // From T2 stands (51.4700, -0.4520) to the 27R threshold
    // (51.4775, -0.4325): a real route with named taxiways.
    let t = std::time::Instant::now();
    let plan = net
        .plan(
            51.4700,
            -0.4520,
            51.4775,
            -0.4325,
            HOLD_SHORT_STANDOFF_M,
            &[],
        )
        .expect("a route");
    eprintln!(
        "plan in {:?}: via {:?}, cross {:?}, hp {:?}",
        t.elapsed(),
        plan.via,
        plan.crossings,
        plan.holding_point
    );
    assert!(!plan.via.is_empty(), "named taxiways expected at Heathrow");
    assert!(
        t.elapsed().as_millis() < 200,
        "planning must not stall a reply"
    );
}

#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn live_apt_dat_scan_is_fast_enough_to_speak() {
    let path =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let t = std::time::Instant::now();
    let rw = find_runways(path, "LEAL").unwrap();
    let elapsed = t.elapsed();
    assert_eq!(rw.len(), 1);
    assert_eq!(
        (rw[0].ident_a.as_str(), rw[0].ident_b.as_str()),
        ("10", "28")
    );
    // A voice answer stalls past a few seconds — even a debug build must fit.
    assert!(elapsed.as_secs_f64() < 5.0, "scan took {elapsed:?}");
    eprintln!("LEAL scan: {elapsed:?}");
}

#[test]
fn airport_ils_reads_locs_and_merges_glideslopes() {
    // Real EGLL rows (Custom Data cycle): loc row 4, GS row 6, plus a
    // different airport's loc that must not leak in.
    let path = std::env::temp_dir().join("xc-nav-ils.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(f, "I").unwrap();
    writeln!(f, "1150 Version - data cycle").unwrap();
    writeln!(f, " 4  51.477697222   -0.426961111       79    11030    18  31769.709  IAA EGLL EG 09L ILS-cat-III").unwrap();
    writeln!(f, " 6  51.478750000   -0.480675000       79    11030    18 300089.709  IAA EGLL EG 09L GS").unwrap();
    writeln!(f, " 4  51.464969444   -0.428230556       75    10950    18  31769.717  IBB EGLL EG 09R ILS-cat-III").unwrap();
    writeln!(f, " 4  40.472222222   -3.560833333     2000    11010    18  50400.000  MAA LEMD LE 32L ILS-cat-I").unwrap();
    writeln!(f, "99").unwrap();
    f.flush().unwrap();
    let ils = airport_ils(path.to_str().unwrap(), "EGLL").unwrap();
    assert_eq!(ils.len(), 2, "{ils:?}");
    let l = ils.iter().find(|i| i.runway == "09L").expect("09L");
    assert_eq!(l.freq_10khz, 11030);
    assert!(
        (l.course_mag - 88.0).abs() < 0.01,
        "magnetic 088 (x 360 + true 89.709): {}",
        l.course_mag
    );
    assert_eq!(l.gs_deg, Some(3.0), "row-6 encoding: angle×100000 + course");
    let r = ils.iter().find(|i| i.runway == "09R").expect("09R");
    assert_eq!(r.gs_deg, None, "no GS row for 09R in this sample");
}

#[test]
fn cifp_platform_altitude_reads_the_faf_crossing() {
    // Real EGLL ILS 27R final legs: IF at 3000 (GS intercept 2500), FAF
    // crossing 2500, runway fix, and a missed-approach climb that must
    // NOT win. Expected platform: the FAF's 2500.
    let dir = std::env::temp_dir().join("xc-cifp-test");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(
        dir.join("EGLL.dat"),
        "APPCH:010,I,I27R, ,CF27R,EG,P,C,E  I, ,   ,IF, ,IRR,EG,P,I,      ,0887,0123,    ,    ,I,03000,02500,     , ,   ,    ,   , , , , , ,0,D,S;\n\
             APPCH:020,I,I27R, ,FF27R,EG,P,C,E  F, ,   ,CF, ,IRR,EG,P,I,      ,0887,0098,2680,0025,G,02500,02500,     , ,   ,-300,   ,EGLL,EG,P,A, ,0,D,S;\n\
             APPCH:030,I,I27R, ,RW27R,EG,P,G,G  M, ,   ,CF, ,IRR,EG,P,I,      ,0887,0023,2680,0075, ,00136,     ,     , ,   ,-300,   , , , , , ,0,D,S;\n\
             APPCH:050,I,I27R, , , , , , E  , ,   ,CA, , , , , ,      ,    ,    ,3150,    ,+,03000,     ,     , ,   ,    ,   , , , , , ,0,D,S;\n\
             APPCH:010,A,I27R,BIG,BIG,EG,D, ,V  C, ,   ,IF, , , , , ,      ,    ,    ,    ,    ,-,220,    ,   , , , , , ,0,D,S;\n",
    )
    .unwrap();
    let dir = dir.to_str().unwrap().to_string();
    assert_eq!(ils_platform_alt_ft(&dir, "EGLL", "27R"), Some(2500.0));
    // No such approach / airport: honest None, callers fall back.
    assert_eq!(ils_platform_alt_ft(&dir, "EGLL", "09L"), None);
    assert_eq!(ils_platform_alt_ft(&dir, "EGKK", "26L"), None);
    // Variant-suffixed idents (Jerez has I20-Y/I20-Z, no bare I20): the
    // suffixed approach's FAF must still resolve. Real LEJR rows.
    std::fs::write(
        std::path::Path::new(&dir).join("LEJR.dat"),
        "APPCH:010,I,I20-Y, ,CF20Y,LE,P,C,E  I, ,   ,IF, ,IJR,LE,P,I,      ,0224,0104,    ,    ,J,02600,01800,     , ,   ,    ,   , , , , , ,0,D,S;\n\
             APPCH:020,I,I20-Y, ,JER,LE,D,B,E  F, ,   ,CF, ,IJR,LE,P,I,      ,0224,0066,2020,0038,H,01800,01800,     ,-,160,-300,   ,JER,LE,D,B, ,0,D,S;\n\
             APPCH:030,I,I20-Y, ,RW20,LE,P,G,G  M, ,   ,CF, ,IJR,LE,P,I,      ,0224,0014,2020,0052, ,00145,     ,     , ,   ,-300,   , , , , , ,0,D,S;\n",
    )
    .unwrap();
    assert_eq!(ils_platform_alt_ft(&dir, "LEJR", "20"), Some(1800.0));
}

/// Real-install smoke test (needs the sim's nav data + CIFP on disk):
/// `cargo test -- --ignored real_install_ils`. Heathrow must resolve all
/// four ILS runways and the 27R platform from the actual AIRAC files.
#[test]
#[ignore]
fn real_install_ils_chart_data_reads() {
    // Same Custom-Data-first rule as Config::from_env (which lives with
    // the crew crate): a Navigraph cycle outranks the stock files.
    fn env_or_first_existing(var: &str, custom: &str, stock: &str) -> String {
        std::env::var(var).unwrap_or_else(|_| {
            if std::path::Path::new(custom).exists() {
                custom
            } else {
                stock
            }
            .to_string()
        })
    }
    let nav_data_path = env_or_first_existing(
        "XPLANE_NAV_DATA",
        &crate::test_xplane_path("Custom Data/earth_nav.dat"),
        &crate::test_xplane_path("Resources/default data/earth_nav.dat"),
    );
    let cifp_dir = env_or_first_existing(
        "XPLANE_CIFP_DIR",
        &crate::test_xplane_path("Custom Data/CIFP"),
        &crate::test_xplane_path("Resources/default data/CIFP"),
    );
    let ils = airport_ils(&nav_data_path, "EGLL").expect("nav scan");
    for rwy in ["09L", "09R", "27L", "27R"] {
        let hit = ils.iter().find(|i| i.runway == rwy);
        assert!(hit.is_some(), "EGLL {rwy} ILS missing: {ils:?}");
        assert_eq!(hit.unwrap().gs_deg, Some(3.0), "{rwy}");
    }
    assert_eq!(
        ils_platform_alt_ft(&cifp_dir, "EGLL", "27R"),
        Some(2500.0),
        "27R platform from the CIFP FAF"
    );
}

// --- Terminal procedure parsing (real EGLL/LEMG CIFP records) ---

// The ILS 27R final segment at Heathrow: initial fix, FAF, runway, missed.
const EGLL_I27R: &str = "\
APPCH:010,I,I27R, ,CF27R,EG,P,C,E  I, ,   ,IF, ,IRR,EG,P,I,      ,0887,0123,    ,    ,I,03000,02500,     , ,   ,    ,   , , , , , ,0,D,S;
APPCH:020,I,I27R, ,FF27R,EG,P,C,E  F, ,   ,CF, ,IRR,EG,P,I,      ,0887,0098,2680,0025,G,02500,02500,     , ,   ,-300,   ,EGLL,EG,P,A, ,0,D,S;
APPCH:030,I,I27R, ,RW27R,EG,P,G,G  M, ,   ,CF, ,IRR,EG,P,I,      ,0887,0023,2680,0075, ,00136,     ,     , ,   ,-300,   , , , , , ,0,D,S;
APPCH:050,I,I27R, , , , , , E  , ,   ,CA, , , , , ,      ,    ,    ,3150,    ,+,03000,     ,     , ,   ,    ,   , , , , , ,0,D,S;
APPCH:010,A,I27R,BIG,BIG,EG,D, ,V  C, ,   ,IF, , , , , ,      ,    ,    ,    ,    ,-,220,    ,   , , , , , ,0,D,S;";

#[test]
fn appch_final_segment_parses_the_legs_in_order() {
    // transition=None keeps only the common/final portion (blank field 3),
    // NOT the BIG transition.
    let legs = parse_proc_legs(EGLL_I27R.lines(), ProcKind::Appch, "I27R", None);
    let types: Vec<&str> = legs.iter().map(|l| l.leg_type.as_str()).collect();
    assert_eq!(
        types,
        vec!["IF", "CF", "CF", "CA"],
        "final segment, BIG excluded"
    );

    // The FAF leg: FF27R, flagged, magnetic course 268, platform 2500.
    let faf = &legs[1];
    assert_eq!(faf.fix_ident, "FF27R");
    assert!(faf.is_faf);
    assert_eq!(faf.course_mag, Some(268.0));
    assert_eq!(faf.alt1_ft, Some(2500.0));

    // The runway leg: threshold at 136 ft.
    assert_eq!(legs[2].fix_ident, "RW27R");
    assert_eq!(legs[2].alt1_ft, Some(136.0));

    // The missed leg climbs on a heading to 3000 with no fix.
    let miss = &legs[3];
    assert_eq!(miss.leg_type, "CA");
    assert!(miss.fix_ident.is_empty());
    assert_eq!(miss.alt_desc, '+');
    assert_eq!(miss.alt1_ft, Some(3000.0));
    assert_eq!(miss.course_mag, Some(315.0));
}

#[test]
fn the_ils_chart_gives_the_threshold_and_the_missed_approach_altitude() {
    let legs = parse_proc_legs(EGLL_I27R.lines(), ProcKind::Appch, "I27R", None);
    let c = ils_chart_from_legs(&legs, "27R");
    assert_eq!(c.threshold_ft, Some(136.0));
    assert_eq!(c.missed_alt_ft, Some(3000.0), "the CA leg after RW27R");
    assert_eq!(c.decision_alt_ft(), Some(336.0));
    // The platform before the runway fix is never the missed approach.
    assert_eq!(ils_chart_from_legs(&legs[..3], "27R").missed_alt_ft, None);
}

#[test]
fn a_localizer_bearing_is_magnetic_times_360_plus_true() {
    let (m, t) = loc_courses(86278.824);
    assert_eq!(m, 239.0);
    assert!((t - 238.824).abs() < 1e-6);
    assert_eq!(loc_courses(268.0), (268.0, 268.0));
}

#[test]
fn appch_transition_is_included_when_named() {
    let legs = parse_proc_legs(EGLL_I27R.lines(), ProcKind::Appch, "I27R", Some("BIG"));
    assert!(
        legs.iter().any(|l| l.fix_ident == "BIG"),
        "BIG transition kept"
    );
    assert!(
        legs.iter().any(|l| l.fix_ident == "RW27R"),
        "common part still kept"
    );
}

// A Malaga SID off runway 30 (BLN4L): course-to-fix then track-to-fix legs.
const LEMG_BLN4L: &str = "\
SID:005,5,BLN4L,RW30,MG930,LE,P,C,E   , ,   ,CF, ,MLG,LE,D, ,      ,3172,0061,3120,0050, ,     ,     ,13000,-,210,    ,   , , , , , , , , ;
SID:010,5,BLN4L,RW30,MG901,LE,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     ,-,210,    ,   , , , , , , , , ;
SID:020,5,BLN4L,RW30,MG902,LE,P,C,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,07500,     ,     ,-,230,    ,   , , , , , , , , ;
SID:030,5,BLN4L,RW30,BLN,LE,D, ,VE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    ,+,10000,     ,     , ,   ,    ,   , , , , , , , , ;";

#[test]
fn sid_runway_transition_parses() {
    let legs = parse_proc_legs(LEMG_BLN4L.lines(), ProcKind::Sid, "BLN4L", Some("RW30"));
    let fixes: Vec<&str> = legs.iter().map(|l| l.fix_ident.as_str()).collect();
    assert_eq!(fixes, vec!["MG930", "MG901", "MG902", "BLN"]);
    // First leg is course-to-fix 312 heading; last leg climbs to at-or-above
    // 10,000.
    assert_eq!(legs[0].leg_type, "CF");
    assert_eq!(legs[0].course_mag, Some(312.0));
    assert_eq!(legs[3].alt_desc, '+');
    assert_eq!(legs[3].alt1_ft, Some(10000.0));
}

#[test]
fn approach_runway_strips_type_and_variant() {
    assert_eq!(approach_runway("I27R").as_deref(), Some("27R"));
    assert_eq!(approach_runway("R27R-Z").as_deref(), Some("27R"));
    assert_eq!(approach_runway("I20-Y").as_deref(), Some("20"));
    assert_eq!(approach_runway("D07").as_deref(), Some("07"));
    assert_eq!(approach_runway("RNAV"), None); // no runway digits at all
}

#[test]
fn reconstructs_the_gibraltar_dme_approach_from_fixes() {
    // LXGB: no CIFP approach, but earth_fix.dat carries the runway-27 DME
    // fixes. Reconstruct the straight-in from them (the real Gibraltar case).
    let apt = std::env::temp_dir().join("xc-lxgb-apt.dat");
    let mut f = std::fs::File::create(&apt).unwrap();
    writeln!(f, "1 11 0 0 LXGB Gibraltar").unwrap();
    // Runway 09/27: 27 threshold at the east end (~-5.341), 09 at the west.
    writeln!(f, "100 46 0 0 0 0 0 0 09 36.1509000 -5.3581000 0 0 0 0 0 0 27 36.1515000 -5.3412000 0 0 0 0 0 0").unwrap();
    f.flush().unwrap();
    let fix = std::env::temp_dir().join("xc-lxgb-fix.dat");
    let mut f = std::fs::File::create(&fix).unwrap();
    // The real LXGB fixes (earth_fix.dat), plus an unrelated field's fix.
    writeln!(
        f,
        "36.157361111 -5.240083333 27D05 LXGB LX 5906505 RWY27 D05"
    )
    .unwrap();
    writeln!(
        f,
        "36.158305556 -5.219500000 27D06 LXGB LX 5906505 RWY27 D06"
    )
    .unwrap();
    writeln!(
        f,
        "36.162027778 -5.137250000 27D10 LXGB LX 5906505 RWY27 D10"
    )
    .unwrap();
    writeln!(
        f,
        "36.104666667 -5.405333333 09D05 LXGB LX 5906505 RWY09 D05"
    )
    .unwrap();
    writeln!(
        f,
        "36.088000000 -5.404666667 09D06 LXGB LX 5906505 RWY09 D06"
    )
    .unwrap();
    // A synthetic 09 fix that IS on the 09 centreline, 4 nm out.
    writeln!(
        f,
        "36.148200000 -5.440600000 09D04 LXGB LX 5906505 RWY09 D04"
    )
    .unwrap();
    writeln!(f, "40.000000000 -3.000000000 27D05 LEMD LE 0 other field").unwrap();
    f.flush().unwrap();

    let airport = find_airport(apt.to_str().unwrap(), "LXGB")
        .unwrap()
        .unwrap();
    let legs = reconstruct_approach(fix.to_str().unwrap(), &airport, "LXGB", "27");
    // Three DME fixes (27D10/06/05) + the threshold, farthest first.
    let idents: Vec<&str> = legs.iter().map(|l| l.fix_ident.as_str()).collect();
    assert_eq!(idents, vec!["27D10", "27D06", "27D05", "RW27"]);
    // Descending altitude on a 3° path — outer fix highest.
    assert!(legs[0].alt1_ft.unwrap() > legs[2].alt1_ft.unwrap());
    assert!(
        legs[0].alt1_ft.unwrap() > 2500.0,
        "10 DME ~ 3000 ft: {:?}",
        legs[0].alt1_ft
    );
    // The innermost DME fix (27D05) is the FAF; the threshold is not.
    assert!(legs[2].is_faf, "27D05 is the FAF");
    assert!(!legs[3].is_faf);
    // The 09 fixes stay out of the 27 reconstruction.
    assert!(!idents.contains(&"09D05"));
    // A runway with no DME fixes → nothing to reconstruct.
    assert!(reconstruct_approach(fix.to_str().unwrap(), &airport, "LXGB", "18").is_empty());

    // Runway 09: the real 09D05/09D06 sit on the VOR/DME approach's offset
    // track, ~50 degrees south of the centreline. They are NOT straight-in
    // fixes (flying them as such crashed a Baron into the North Front,
    // 2026-09-11); only the on-course 09D04 survives, and it is the FAF.
    let legs = reconstruct_approach(fix.to_str().unwrap(), &airport, "LXGB", "09");
    let idents: Vec<&str> = legs.iter().map(|l| l.fix_ident.as_str()).collect();
    assert_eq!(idents, vec!["09D04", "RW09"], "off-course DME fixes dropped");
    assert!(legs[0].is_faf);
    let (xtk_m, along_m) = runway_frame_m(
        36.1509, -5.3581, runway_course_deg(&airport, "09").unwrap(), 36.1482, -5.4406,
    );
    assert!(xtk_m.abs() < 100.0, "on the centreline: xtk {xtk_m:.0} m");
    assert!((along_m - 4.0 * 1852.0).abs() < 200.0, "4 nm out: {along_m:.0} m");
    // And the offset fix, in the same frame: abeam, not ahead.
    let (xtk_m, along_m) = runway_frame_m(
        36.1509, -5.3581, runway_course_deg(&airport, "09").unwrap(), 36.104666667, -5.405333333,
    );
    assert!(xtk_m.abs() > 4000.0, "09D05 is {xtk_m:.0} m off the centreline");
    assert!(along_m < 5000.0, "with only {along_m:.0} m to run");
}

#[test]
fn no_on_course_dme_fixes_means_no_straight_in() {
    let apt = std::env::temp_dir().join("xc-lxgb-apt-nofix.dat");
    let mut f = std::fs::File::create(&apt).unwrap();
    writeln!(f, "1 11 0 0 LXGB Gibraltar").unwrap();
    writeln!(f, "100 46 0 0 0 0 0 0 09 36.1509000 -5.3581000 0 0 0 0 0 0 27 36.1515000 -5.3412000 0 0 0 0 0 0").unwrap();
    f.flush().unwrap();
    let fix = std::env::temp_dir().join("xc-lxgb-fix-nofix.dat");
    let mut f = std::fs::File::create(&fix).unwrap();
    writeln!(f, "36.104666667 -5.405333333 09D05 LXGB LX 5906505 RWY09 D05").unwrap();
    writeln!(f, "36.088000000 -5.404666667 09D06 LXGB LX 5906505 RWY09 D06").unwrap();
    f.flush().unwrap();
    let airport = find_airport(apt.to_str().unwrap(), "LXGB").unwrap().unwrap();
    // The live Gibraltar case: every 09 fix is off course, so there is no
    // reconstruction at all and the arrival falls back to a visual landing.
    assert!(reconstruct_approach(fix.to_str().unwrap(), &airport, "LXGB", "09").is_empty());
}

#[test]
fn resolver_fills_runway_and_navaid_fixes() {
    // A tiny airport with runway 30/12, plus a nav-data file holding the
    // MG930 waypoint and the BLN VOR — the resolver must place both.
    let apt = std::env::temp_dir().join("xc-proc-apt.dat");
    let mut f = std::fs::File::create(&apt).unwrap();
    writeln!(f, "1 16 0 0 LEMG Malaga").unwrap();
    // Runway 30/12: end A "30" near (36.67,-4.50), end B "12".
    writeln!(f, "100 45 0 0 0 0 0 0 30 36.6700000 -4.5000000 0 0 0 0 0 0 12 36.6600000 -4.4800000 0 0 0 0 0 0").unwrap();
    f.flush().unwrap();
    let fix = std::env::temp_dir().join("xc-proc-fix.dat");
    let mut f = std::fs::File::create(&fix).unwrap();
    writeln!(f, "36.7535555 -4.5927222 MG930 LEMG LE 4464727 MG930").unwrap();
    f.flush().unwrap();
    let nav = std::env::temp_dir().join("xc-proc-nav.dat");
    let mut f = std::fs::File::create(&nav).unwrap();
    writeln!(
        f,
        "3 38.1525555 -3.6247222 1772 11620 130 1.000 BLN ENRT LE BAILEN VOR/DME"
    )
    .unwrap();
    f.flush().unwrap();

    let airport = find_airport(apt.to_str().unwrap(), "LEMG")
        .unwrap()
        .unwrap();
    let mut legs = parse_proc_legs(
        "SID:005,5,X,RW30,MG930,LE,P,C,E   , ,   ,CF, ,MLG,LE,D, ,      ,0,0,3120,0050, ,     ,     ,     , ,   ,    ,   ;\n\
             SID:010,5,X,RW30,BLN,LE,D, ,VE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     ,     ,     ;\n\
             SID:020,5,X,RW30,RW30,LE,P,G,G  M, ,   ,CF, , , , , ,      ,0,0,3120,0050, ,     ,     ,     ;"
            .lines(),
        ProcKind::Sid,
        "X",
        Some("RW30"),
    );
    resolve_proc_fixes(
        fix.to_str().unwrap(),
        nav.to_str().unwrap(),
        &airport,
        "LEMG",
        &mut legs,
        36.67,
        -4.5,
    );
    // MG930 waypoint from earth_fix.dat.
    assert_eq!(legs[0].fix.as_ref().unwrap().ident, "MG930");
    assert!((legs[0].fix.as_ref().unwrap().lat - 36.7535555).abs() < 1e-4);
    // BLN VOR from earth_nav.dat.
    assert!((legs[1].fix.as_ref().unwrap().lat - 38.1525555).abs() < 1e-4);
    // RW30 threshold from apt.dat.
    assert!((legs[2].fix.as_ref().unwrap().lat - 36.67).abs() < 1e-6);
}

/// Real-install smoke test: Malaga publishes procedures; parse + resolve the
/// ILS to a runway and one of its SIDs end to end.
/// `cargo test -p atc -- --ignored real_install_procedures`
#[test]
#[ignore]
fn real_install_procedures() {
    fn first_existing(custom: &str, stock: &str) -> String {
        if std::path::Path::new(custom).exists() {
            custom
        } else {
            stock
        }
        .to_string()
    }
    let cifp = first_existing(
        &crate::test_xplane_path("Custom Data/CIFP"),
        &crate::test_xplane_path("Resources/default data/CIFP"),
    );
    let fix_path = first_existing(
        &crate::test_xplane_path("Custom Data/earth_fix.dat"),
        &crate::test_xplane_path("Resources/default data/earth_fix.dat"),
    );
    let nav_path = first_existing(
        &crate::test_xplane_path("Custom Data/earth_nav.dat"),
        &crate::test_xplane_path("Resources/default data/earth_nav.dat"),
    );
    let apt =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let airport = find_airport(apt, "LEMG").unwrap().expect("LEMG in apt.dat");
    let (near_lat, near_lon) = airport.position().unwrap();

    // Pick the ILS approach for 12 or 30 and resolve every leg's fix.
    for rwy in ["12", "30", "13", "31"] {
        if let Some(appch) = best_approach(&cifp, "LEMG", rwy) {
            let mut legs = load_proc_legs(&cifp, "LEMG", ProcKind::Appch, &appch, None);
            assert!(!legs.is_empty(), "{appch} has legs");
            resolve_proc_fixes(
                &fix_path, &nav_path, &airport, "LEMG", &mut legs, near_lat, near_lon,
            );
            let named: Vec<_> = legs.iter().filter(|l| !l.fix_ident.is_empty()).collect();
            let resolved = named.iter().filter(|l| l.fix.is_some()).count();
            assert!(
                resolved >= named.len().saturating_sub(1),
                "{appch}: resolved {resolved}/{} fixes",
                named.len()
            );
            println!("LEMG {rwy}: approach {appch}, {} legs", legs.len());
        }
    }
    // At least one SID resolves too.
    let sid =
        sid_for_runway(&cifp, "LEMG", "30").or_else(|| sid_for_runway(&cifp, "LEMG", "12"));
    let sid = sid.expect("LEMG has a SID for 30/12");
    let mut legs = load_proc_legs(&cifp, "LEMG", ProcKind::Sid, &sid, Some("RW30"));
    if legs.is_empty() {
        legs = load_proc_legs(&cifp, "LEMG", ProcKind::Sid, &sid, Some("RW12"));
    }
    resolve_proc_fixes(
        &fix_path, &nav_path, &airport, "LEMG", &mut legs, near_lat, near_lon,
    );
    assert!(
        legs.iter().any(|l| l.fix.is_some()),
        "SID {sid} resolved a fix"
    );

    // Gibraltar: no coded approach, but the DME fixes reconstruct the
    // straight-in (the whole reason this exists). Runway 27's fixes must
    // chain farthest-first to the threshold, over the water.
    let apt =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    if let Ok(Some(lxgb)) = find_airport(apt, "LXGB") {
        assert!(
            list_procedures(&cifp, "LXGB", ProcKind::Appch).is_empty(),
            "LXGB has no coded approach"
        );
        let recon = reconstruct_approach(&fix_path, &lxgb, "LXGB", "27");
        assert!(
            recon.len() >= 3,
            "reconstructed 27 straight-in from DME fixes: {recon:?}"
        );
        assert!(
            recon.last().unwrap().fix_ident.starts_with("RW"),
            "ends at the threshold"
        );
        assert!(recon.iter().any(|l| l.is_faf), "an FAF is flagged");
    }
}

#[test]
fn none_when_no_runway_nearby() {
    let path = std::env::temp_dir().join("xc-nav-test2.dat");
    let mut f = std::fs::File::create(&path).unwrap();
    writeln!(
        f,
        " 4  47.466600000 -122.317800000  359 11075 18 123840.337 IFAR KSEA K1 28 ILS-cat-I"
    )
    .unwrap();
    f.flush().unwrap();
    // Aircraft far from the only runway-28 → none within 60nm.
    let r = find_ils(path.to_str().unwrap(), "28", 36.71, -4.55, 60.0).unwrap();
    assert!(r.is_none());
}

/// A close-in stand must still get a route.
///
/// EGHI 2026-07-30: after landing, the aircraft sat 224 m from taxiway
/// node 44 and the nearest suitable stand was another 69 m past the same
/// node — so both ends snapped to node 44, `start == goal`, and the
/// planner returned None. The tower then answered "taxi to parking, this
/// frequency" with no route attached, and the crew's taxi-in waited out
/// its full 60 s before giving up with "no taxi-in clearance came back".
/// The clearance HAD come; the route had not.
#[test]
fn a_stand_on_the_same_node_still_gets_a_route() {
    // Two nodes joined by one taxiway; the stand sits just past node 1,
    // so a nearby aircraft and the stand both snap to it.
    let mut net = TaxiNet::default();
    net.nodes.insert(1, (50.9500, -1.3600));
    net.nodes.insert(2, (50.9520, -1.3600));
    net.edges.push(TaxiEdge {
        a: 1,
        b: 2,
        name: "Alpha".into(),
        runway: false,
        oneway: false,
        zone_of: Vec::new(),
    });
    let (stand_lat, stand_lon) = (50.94995, -1.35985);
    let (ac_lat, ac_lon) = (50.95005, -1.36005);
    let plan = net
        .plan(ac_lat, ac_lon, stand_lat, stand_lon, 0.0, &[])
        .expect("a close stand is a short taxi, not an impossible one");
    assert!(
        plan.path.len() >= 2,
        "the publication drops anything shorter: {:?}",
        plan.path
    );
    // It must START AT THE AIRCRAFT. Starting at the snapped node put the
    // route 224 m from a stopped aircraft at EGHI, so the drive's arrival
    // test was true on tick one and it "secured" in two seconds without
    // moving.
    let first = plan.path[0];
    assert!(
        (first.0 - ac_lat).abs() < 1e-6 && (first.1 - ac_lon).abs() < 1e-6,
        "route must begin where the aircraft is, got {first:?}"
    );
    let end = *plan.path.last().unwrap();
    assert!(
        (end.0 - stand_lat).abs() < 1e-6 && (end.1 - stand_lon).abs() < 1e-6,
        "the route must END at the stand, got {end:?}"
    );
    assert_eq!(plan.seg.len(), plan.path.len() - 1, "seg tracks path");

    // A RUNWAY destination we are already at is still nothing to do. This
    // was written as `.is_none() || true`, which asserts nothing and hid
    // behind a "runway behaviour unchanged" message — probed, the call
    // really does return None here, so assert it.
    assert!(
        net.plan_to_runway(50.95005, -1.36005, 50.94995, -1.35985, 0.0, &[], None)
            .is_none(),
        "a runway destination on the node we are already at is no taxi"
    );
    // ...and a runway destination that IS somewhere else still plans, even
    // though this net has no runway edges for `runway_entrance_near` to
    // find. No authored entrance degrades to the plain snap; it is never a
    // refusal, because "no entrance in the data" is not a reason to refuse
    // to taxi (776 runway ends worldwide have none).
    let far = net
        .plan_to_runway(50.95005, -1.36005, 50.9520, -1.3600, 0.0, &[], None)
        .expect("no runway edges means the plain snap, not a refusal");
    let plain = net
        .plan(50.95005, -1.36005, 50.9520, -1.3600, 0.0, &[])
        .expect("the plain plan is the yardstick");
    assert_eq!(
        far.path, plain.path,
        "with no entrance authored, they agree"
    );
}

/// LILB's own network (apt.dat 1201/1202 as authored) and the Lancair
/// Evolution's stand: 45.771275 9.164289, facing 28.9 (1790712974,
/// 1790720549, 1790721973; 2026-09-29 and 30). The nearest node (5) is
/// 16.7 m away on a bearing of 98 and the edge beyond it runs 335: the
/// route ran to it and hairpinned 124 degrees, and every taxi failed
/// 14.2 to 16.3 m off the line. Now it joins taxiway A ahead of the
/// aircraft: no turn anywhere on the route past 90 degrees, the first
/// leg within 30 degrees of the nose, and node 5 not on the route.
#[test]
fn a_route_joins_its_first_taxiway_ahead_never_back_at_the_node() {
    let mut net = TaxiNet::default();
    for (id, p) in [
        (0, (45.771927, 9.164043)),
        (1, (45.772005, 9.163885)),
        (2, (45.767882, 9.161007)),
        (3, (45.772092, 9.163689)),
        (4, (45.772017, 9.163642)),
        (5, (45.771252, 9.164503)),
        (6, (45.772075, 9.164124)),
    ] {
        net.nodes.insert(id, p);
    }
    let e = |a, b, name: &str, runway, zone: &[&str]| TaxiEdge {
        a,
        b,
        name: name.into(),
        runway,
        oneway: false,
        zone_of: zone.iter().map(|s| s.to_string()).collect(),
    };
    net.edges.push(e(0, 5, "taxiway_A", false, &[]));
    net.edges.push(e(1, 0, "taxiway_A", false, &[]));
    net.edges.push(e(1, 6, "taxiway_A", false, &[]));
    net.edges.push(e(4, 1, "taxiway_A", false, &["03", "21"]));
    net.edges.push(e(3, 4, "runway", true, &["03", "21"]));
    net.edges.push(e(4, 2, "runway", true, &["03", "21"]));
    let (lat, lon, psi) = (45.771275, 9.164289, 28.9);
    let zone = ["03".to_string(), "21".to_string()];
    let plan = net
        .plan_to_runway(lat, lon, 45.7671573, 9.1605458, 40.0, &zone, None)
        .expect("a route to 03");
    let turn = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
        let d = bearing_deg(b.0, b.1, c.0, c.1) - bearing_deg(a.0, a.1, b.0, b.1);
        sim::angles::wrap180(d).abs()
    };
    // As planned before: out to node 5 and back up the taxiway.
    let node5 = net.nodes[&5];
    let node0 = net.nodes[&0];
    assert!(turn((lat, lon), node5, node0) > 120.0, "the recorded hairpin");
    assert_eq!(plan.path[0], (lat, lon), "starts at the aircraft");
    assert!(!plan.path.contains(&node5), "node 5 is behind us: {:?}", plan.path);
    assert_eq!(plan.seg.len(), plan.path.len() - 1);
    let first = bearing_deg(lat, lon, plan.path[1].0, plan.path[1].1);
    let off_nose = sim::angles::abs_diff_deg(first, psi);
    assert!(off_nose < 30.0, "first leg {first:.0} vs nose {psi}");
    for w in plan.path.windows(3) {
        let t = turn(w[0], w[1], w[2]);
        assert!(t <= 90.0, "a {t:.0} degree turn at {:?}", w[1]);
    }
    // The route still ends at the 21_hold edge's hold (node 1), and is
    // shorter by the leg back to the node.
    assert_eq!(*plan.path.last().unwrap(), net.nodes[&1]);
    let total: f64 = plan
        .path
        .windows(2)
        .map(|w| haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0)
        .sum();
    assert!(total < 110.0, "{total:.1} m (was 115.1)");
}

/// The join is pure geometry: abeam an edge's start it changes nothing,
/// a runway leg is never moved, and an edge wholly behind is dropped.
#[test]
fn join_first_edge_ahead_leaves_a_start_abeam_its_node_alone() {
    let here = (51.0, -1.0);
    let north = |m: f64| (51.0 + m / 111_320.0, -1.0);
    // Standing on the start node: unchanged.
    let mut path = vec![north(0.0), north(100.0), north(200.0)];
    let mut seg = vec![(String::new(), false); 2];
    join_first_edge_ahead(&mut path, &mut seg, here);
    assert_eq!(path[0], north(0.0));
    // 40 m past the start node on the centreline: the join is 5 m ahead.
    let mut path = vec![north(-40.0), north(100.0), north(200.0)];
    join_first_edge_ahead(&mut path, &mut seg, here);
    assert!((path[0].0 - north(5.0).0).abs() * 111_320.0 < 0.01, "{:?}", path[0]);
    // Past the first edge entirely: its far node is the join.
    let mut path = vec![north(-100.0), north(-2.0), north(200.0)];
    let mut seg = vec![(String::new(), false); 2];
    join_first_edge_ahead(&mut path, &mut seg, here);
    assert_eq!(path, vec![north(-2.0), north(200.0)]);
    assert_eq!(seg.len(), 1);
    // A runway leg is the runway's.
    let mut path = vec![north(-40.0), north(100.0)];
    let mut seg = vec![(String::new(), true)];
    join_first_edge_ahead(&mut path, &mut seg, here);
    assert_eq!(path[0], north(-40.0));
}

/// A strip running north with a taxiway parallel to it, joined abeam each
/// runway node. Nodes 1..3 are the runway, 11..13 the taxiway.
fn vacate_net() -> TaxiNet {
    let mut net = TaxiNet::default();
    // Runway 36/18, three nodes up the centreline (0, 400, 800 m north).
    net.nodes.insert(1, (51.0000, -1.0000));
    net.nodes.insert(2, (51.0036, -1.0000));
    net.nodes.insert(3, (51.0072, -1.0000));
    // Parallel taxiway ~150 m east, abeam each.
    net.nodes.insert(11, (51.0000, -0.99786));
    net.nodes.insert(12, (51.0036, -0.99786));
    net.nodes.insert(13, (51.0072, -0.99786));
    let e = |a, b, name: &str, runway, zone: &[&str]| TaxiEdge {
        a,
        b,
        name: name.into(),
        runway,
        oneway: false,
        zone_of: zone.iter().map(|s| s.to_string()).collect(),
    };
    net.edges.push(e(1, 2, "36/18", true, &["36", "18"]));
    net.edges.push(e(2, 3, "36/18", true, &["36", "18"]));
    // The links onto the taxiway sit INSIDE the runway's active zone...
    net.edges.push(e(1, 11, "A1", false, &["36", "18"]));
    net.edges.push(e(2, 12, "A2", false, &["36", "18"]));
    net.edges.push(e(3, 13, "A3", false, &["36", "18"]));
    // ...and the parallel taxiway itself is outside it.
    net.edges.push(e(11, 12, "Alpha", false, &[]));
    net.edges.push(e(12, 13, "Alpha", false, &[]));
    net
}

/// Vacating is the completion of the landing clearance, not a new
/// instruction — so it plans from where autoland stopped us without
/// asking anybody, and it must not stop with the tail still on the paint.
#[test]
fn a_vacate_leaves_the_runway_forwards_and_gets_fully_clear() {
    let net = vacate_net();
    let zone = ["36".to_string(), "18".to_string()];
    // Stopped just short of node 2, rolled out northbound on 36.
    let (lat, lon) = (51.0034, -1.0000);
    let plan = net
        .plan_vacate(lat, lon, 0.0, &zone, VACATE_CLEAR_M)
        .expect("there is an exit ahead");

    // Starts at the aircraft — a route that begins at the snapped node
    // reads as already-arrived on the drive's first tick (the EGHI stand
    // lesson, one module over).
    assert!(
        (plan.path[0].0 - lat).abs() < 1e-9 && (plan.path[0].1 - lon).abs() < 1e-9,
        "must start where the aircraft is, got {:?}",
        plan.path[0]
    );
    assert_eq!(plan.seg.len(), plan.path.len() - 1, "seg tracks path");

    // It does NOT stop on the link: that edge is inside the active zone,
    // so the walk carries on down Alpha until it is past the boundary.
    assert!(plan.path.len() >= 3, "must not stop on the link: {plan:?}");
    assert_eq!(plan.exit_name, "Alpha");
    let end = *plan.path.last().unwrap();
    let east_m = (end.1 - lon) * 111_320.0 * lat.to_radians().cos();
    assert!(east_m > 140.0, "must end off the strip, {east_m} m east");

    // Never BACKWARDS down the runway. Landing the other way from the same
    // spot, the exit taken must be the one that is now ahead.
    let south = net
        .plan_vacate(lat, lon, 180.0, &zone, VACATE_CLEAR_M)
        .expect("there is an exit ahead that way too");
    assert!(
        south.path[1].0 < lat,
        "a southbound rollout must exit south, got {:?}",
        south.path[1]
    );
}

/// EGLL 27L, 2026-09-17. The nearest junction ahead was 84 m away and its
/// only link, N4W, ran back east-north-east — a rapid exit built for 09R
/// traffic, 169 degrees off our rollout. The vacate took it: up the
/// runway, a turn on the strip, and back east past the touchdown point,
/// 624 m of route and three minutes of occupancy with traffic inside a
/// mile. The turn-off that works, N5W at 54 degrees, was 220 m further
/// on. A junction is only an exit if you can turn onto what leaves it.
#[test]
fn a_vacate_refuses_an_exit_it_would_have_to_turn_back_to_take() {
    let mut net = TaxiNet::default();
    // Runway 09R/27L along a parallel of latitude; we roll WEST on 27L.
    let m = |east_m: f64| -1.0 + east_m / (111_320.0 * 51.0_f64.to_radians().cos());
    net.nodes.insert(1, (51.0, m(0.0)));
    net.nodes.insert(2, (51.0, m(-300.0)));
    net.nodes.insert(3, (51.0, m(-700.0)));
    // Node 2's only turn-off is angled BACK east (the 09R rapid exit).
    net.nodes.insert(12, (51.0009, m(-210.0)));
    // Node 3 offers a proper forward turn-off, north-west.
    net.nodes.insert(13, (51.0009, m(-790.0)));
    net.nodes.insert(23, (51.0018, m(-790.0)));
    let e = |a, b, name: &str, runway, zone: &[&str]| TaxiEdge {
        a,
        b,
        name: name.into(),
        runway,
        oneway: false,
        zone_of: zone.iter().map(|s| s.to_string()).collect(),
    };
    net.edges.push(e(1, 2, "09R/27L", true, &["09R", "27L"]));
    net.edges.push(e(2, 3, "09R/27L", true, &["09R", "27L"]));
    net.edges.push(e(2, 12, "N4W", false, &["09R", "27L"]));
    net.edges.push(e(3, 13, "N5W", false, &["09R", "27L"]));
    net.edges.push(e(13, 23, "Block 81", false, &[]));
    let zone = ["27L".to_string(), "09R".to_string()];
    // Stopped on the centreline, heading west, N4W's junction 300 m ahead
    // and N5W's 700 m ahead.
    let plan = net
        .plan_vacate(51.0, m(0.0), 270.0, &zone, VACATE_CLEAR_M)
        .expect("the further exit is still an exit");
    assert!(
        plan.seg.iter().all(|(n, _)| n != "N4W"),
        "the backwards rapid exit must not be taken: {:?}",
        plan.seg
    );
    assert_eq!(plan.exit_name, "Block 81", "walks out through N5W");
    // And every step of it keeps going the way we were rolling: no point
    // on the route may lie east of where the aircraft stopped.
    let east_of_start = plan.path.iter().any(|p| p.1 > m(0.0) + 1e-9);
    assert!(!east_of_start, "the route doubles back: {:?}", plan.path);

    // A field where the ONLY turn-off is the backwards one yields no plan
    // at all, and the caller taxis in from the runway — the same
    // degradation the dead end makes, and better than a U-turn on it.
    net.edges
        .retain(|e| e.name != "N5W" && e.name != "Block 81");
    assert!(
        net.plan_vacate(51.0, m(0.0), 270.0, &zone, VACATE_CLEAR_M)
            .is_none(),
        "no exit is better than a reversal"
    );
}

/// EDLP 24 (2026-09-24): B leaves the runway 108 degrees off the SF50's
/// rollout, 400 m ahead; C is 74 degrees off, 970 m ahead. At a 100
/// degree limit B was refused and the vacate drove 1,217 m at turn pace.
#[test]
fn a_vacate_takes_a_turn_off_a_little_past_abeam() {
    let mut net = TaxiNet::default();
    let m = |east_m: f64| -1.0 + east_m / (111_320.0 * 51.0_f64.to_radians().cos());
    let north = |n_m: f64| 51.0 + n_m / 111_320.0;
    net.nodes.insert(1, (51.0, m(0.0)));
    net.nodes.insert(2, (51.0, m(-400.0)));
    net.nodes.insert(3, (51.0, m(-970.0)));
    // B: from node 2, 108 degrees off a westbound (270) rollout, i.e. on
    // 162: south and a little east.
    let b = 162.0_f64.to_radians();
    net.nodes.insert(12, (north(80.0 * b.cos()), m(-400.0 + 80.0 * b.sin())));
    net.nodes.insert(22, (north(160.0 * b.cos()), m(-400.0 + 160.0 * b.sin())));
    // C: from node 3, 74 degrees off, on 196.
    let c = 196.0_f64.to_radians();
    net.nodes.insert(13, (north(80.0 * c.cos()), m(-970.0 + 80.0 * c.sin())));
    net.nodes.insert(23, (north(160.0 * c.cos()), m(-970.0 + 160.0 * c.sin())));
    let e = |a, b, name: &str, runway, zone: &[&str]| TaxiEdge {
        a,
        b,
        name: name.into(),
        runway,
        oneway: false,
        zone_of: zone.iter().map(|s| s.to_string()).collect(),
    };
    net.edges.push(e(1, 2, "06/24", true, &["06", "24"]));
    net.edges.push(e(2, 3, "06/24", true, &["06", "24"]));
    net.edges.push(e(2, 12, "B", false, &["06", "24"]));
    net.edges.push(e(12, 22, "B", false, &[]));
    net.edges.push(e(3, 13, "C", false, &["06", "24"]));
    net.edges.push(e(13, 23, "C", false, &[]));
    let zone = ["24".to_string(), "06".to_string()];
    let plan = net
        .plan_vacate(51.0, m(0.0), 270.0, &zone, VACATE_CLEAR_M)
        .expect("an exit");
    assert_eq!(plan.exit_name, "B", "{:?}", plan.seg);
}

/// The two ways the data can be thin. Neither is a reason to sit on a
/// runway, so neither is a refusal — the same rule `plan_to_runway`
/// follows for the 776 runway ends with no authored entrance.
#[test]
fn a_vacate_degrades_rather_than_refusing() {
    let zone = ["36".to_string(), "18".to_string()];

    // No authored ACTIVE ZONES: nothing marks the boundary, so the walk
    // just runs its clearance distance out from the exit. Still a plan.
    let mut no_zone = vacate_net();
    for e in &mut no_zone.edges {
        e.zone_of.clear();
    }
    let p = no_zone
        .plan_vacate(51.0034, -1.0000, 0.0, &zone, VACATE_CLEAR_M)
        .expect("no zones is not a refusal");
    assert!(p.path.len() >= 2);

    // No authored RUNWAY edges: nothing to be adjacent to, so every
    // taxiway node is a candidate and the nearest ahead wins.
    let mut no_rwy = vacate_net();
    no_rwy.edges.retain(|e| !e.runway);
    let p = no_rwy
        .plan_vacate(51.0034, -1.0000, 0.0, &zone, VACATE_CLEAR_M)
        .expect("no runway edges is not a refusal either");
    assert!(p.path.len() >= 2);

    // No network at all is the one honest None: there is nothing to drive
    // along, and the caller keeps its old behaviour.
    assert!(TaxiNet::default()
        .plan_vacate(51.0034, -1.0000, 0.0, &zone, VACATE_CLEAR_M)
        .is_none());
}

/// A junction whose only taxiway link is authored one-way TOWARD the
/// runway (standard ground-flow authoring) is a dead end for the outward
/// walk. Before the audit fix this returned a one-leg "plan" from the
/// aircraft to a node ON the centreline — a drive down the runway that
/// taxi_in then reported as a vacate. No plan is the honest answer.
#[test]
fn a_dead_end_junction_is_a_refusal_not_a_runway_plan() {
    let mut net = TaxiNet::default();
    net.nodes.insert(1, (51.0000, -1.0000));
    net.nodes.insert(2, (51.0036, -1.0000));
    net.nodes.insert(3, (51.0072, -1.0000));
    net.nodes.insert(12, (51.0036, -0.99786));
    let e = |a: i64, b: i64, name: &str, runway: bool, oneway: bool, z: &[&str]| TaxiEdge {
        a,
        b,
        name: name.into(),
        runway,
        oneway,
        zone_of: z.iter().map(|s| s.to_string()).collect(),
    };
    net.edges.push(e(1, 2, "36/18", true, false, &["36", "18"]));
    net.edges.push(e(2, 3, "36/18", true, false, &["36", "18"]));
    // The ONLY link at the junction, one-way toward the runway.
    net.edges.push(e(12, 2, "A2", false, true, &["36", "18"]));
    let zone = ["36".to_string(), "18".to_string()];
    assert!(
        net.plan_vacate(51.0000, -1.0000, 0.0, &zone, VACATE_CLEAR_M)
            .is_none(),
        "an untraversable exit must refuse, never plan down the strip"
    );
}

/// A vacate is planned FROM the strip. From the parallel taxiway — the
/// give-up-then-retry case, where the landing clearance still stands —
/// the nearest-junction-ahead snap would aim the aircraft back AT the
/// runway, so where runway edges exist to judge by, off-strip refuses.
#[test]
fn a_vacate_from_the_taxiway_is_a_refusal() {
    let net = vacate_net();
    let zone = ["36".to_string(), "18".to_string()];
    // On Alpha, 150 m east of the centreline, pointing up the taxiway.
    assert!(
        net.plan_vacate(51.0034, -0.99786, 0.0, &zone, VACATE_CLEAR_M)
            .is_none(),
        "already off the strip: nothing to vacate"
    );
    // ...and thin data still cannot judge it, so it keeps the old
    // behaviour rather than refusing (the 776-ends rule).
    let mut no_rwy = vacate_net();
    no_rwy.edges.retain(|e| !e.runway);
    assert!(no_rwy
        .plan_vacate(51.0034, -0.99786, 0.0, &zone, VACATE_CLEAR_M)
        .is_some());
}

/// At an intersecting-runway field the way off our strip must never be
/// ACROSS the other one. `zone` names only the runway we landed on, so a
/// plain in-our-zone test read a link inside the crossing runway's zone
/// as clean pavement and preferred it over our own exit stub.
#[test]
fn the_walk_prefers_our_exit_stub_over_a_crossing_runways_zone() {
    let mut net = vacate_net();
    // A link west from the same junction, inside the CROSSING runway's
    // active zone (drawn as a plain taxiway edge, the EGLL-N7 pattern).
    net.nodes.insert(22, (51.0036, -1.00214));
    net.edges.push(TaxiEdge {
        a: 2,
        b: 22,
        name: "B".into(),
        runway: false,
        oneway: false,
        zone_of: vec!["09".to_string(), "27".to_string()],
    });
    let zone = ["36".to_string(), "18".to_string()];
    let plan = net
        .plan_vacate(51.0034, -1.0000, 0.0, &zone, VACATE_CLEAR_M)
        .expect("still plans");
    // The first step off the junction is our own stub EAST onto Alpha,
    // never the westbound link across the other strip.
    let step = plan.path[2];
    assert!(
        step.1 > -1.0000,
        "walked across the crossing runway's zone: {plan:?}"
    );
}
/// #36 over the real apt.dat: every field whose runway ends converge
/// (an end within 300 m of another strip's end on a course 10 degrees or
/// more away, the LFPB 07/09 shape), a taxi from its stands to each such
/// end planned with `plan_to_runway` and the strip, as the station
/// plans it. The route must end at the cleared strip's own entrance:
/// the lead-on ends on ITS centreline (the line-up's 20 m), never on the
/// neighbour's. The same plan without the strip (the old
/// nearest-entrance-anywhere rule) is counted beside it.
/// Fields are taken longest runway first; `XVL_SWEEP_FIELDS` sets how
/// many (default 120).
/// `cargo test --release -p atc -- --ignored real_install_converging_thresholds --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_converging_thresholds_taxi_to_their_own_strip() {
    let apt =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    if !std::path::Path::new(apt).exists() {
        return;
    }
    let limit: usize = std::env::var("XVL_SWEEP_FIELDS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(120);
    // (icao, runways, [(strip index, end is b, other strip index)]).
    type Conv = (String, Vec<Runway>, Vec<(usize, bool, usize)>);
    let mut fields: Vec<Conv> = Vec::new();
    let (mut ends_total, mut ends_conv) = (0usize, 0usize);
    scan_airports(apt, |icao, a| {
        let rw = a.runways;
        let mut conv = Vec::new();
        for (i, r) in rw.iter().enumerate() {
            for b in [false, true] {
                ends_total += 1;
                let (lat, lon, hdg) = if b {
                    (r.lat_b, r.lon_b, r.heading_b_deg())
                } else {
                    (r.lat_a, r.lon_a, r.heading_a_deg())
                };
                let other = rw.iter().enumerate().filter(|(j, _)| *j != i).find(|(_, o)| {
                    [(o.lat_a, o.lon_a, o.heading_a_deg()), (o.lat_b, o.lon_b, o.heading_b_deg())]
                        .iter()
                        .any(|&(la, lo, h)| {
                            haversine_nm(lat, lon, la, lo) * 1852.0 <= 300.0
                                && sim::angles::abs_diff_deg(h, hdg) >= 10.0
                        })
                });
                if let Some((j, _)) = other {
                    ends_conv += 1;
                    conv.push((i, b, j));
                }
            }
        }
        if !conv.is_empty() {
            fields.push((icao.to_string(), rw, conv));
        }
    })
    .unwrap();
    println!(
        "{ends_conv} of {ends_total} runway ends converge, at {} fields",
        fields.len()
    );
    fields.sort_by(|a, b| {
        let l = |f: &Conv| f.1.iter().map(Runway::length_m).fold(0.0, f64::max);
        l(b).total_cmp(&l(a))
    });
    // Cross- and along-track of `p` from strip `r`'s centreline, from end A.
    let track = |r: &Runway, p: (f64, f64)| {
        let d = haversine_nm(r.lat_a, r.lon_a, p.0, p.1) * 1852.0;
        let ang = sim::angles::signed_diff_deg(bearing_deg(r.lat_a, r.lon_a, p.0, p.1), r.heading_a_deg())
            .to_radians();
        ((d * ang.sin()).abs(), d * ang.cos())
    };
    let on_strip = |r: &Runway, p: (f64, f64)| {
        let (xt, along) = track(r, p);
        xt <= r.width_m / 2.0 + 15.0 && along >= -15.0 && along <= r.length_m() + 15.0
    };
    // Where the drive ends: the lead-on's last point, else the holding
    // point. Some(true) = our strip's entrance.
    let judge = |s: &Runway, other: &Runway, plan: &TaxiRoutePlan| -> Option<bool> {
        let last = plan.lead_on.last().or(plan.path.last()).copied()?;
        if !plan.lead_on.is_empty() {
            let ours = track(s, last).0 <= 20.0;
            let theirs = track(other, last).0 <= 20.0 && on_strip(other, last);
            return Some(ours || !theirs);
        }
        // Untrimmed: the holding point, nearer ours than theirs.
        Some(track(s, last).0 <= track(other, last).0 || on_strip(s, last))
    };
    let (mut routes, mut none, mut ok, mut untrimmed, mut old_wrong) = (0, 0, 0, 0, 0);
    let (mut with_net, mut wrong, mut thin) = (0usize, Vec::new(), Vec::new());
    for (icao, rw, conv) in fields.iter().take(limit) {
        let Ok(Some(net)) = find_taxi_net(apt, icao) else {
            continue;
        };
        if net.stands.is_empty() {
            continue;
        }
        with_net += 1;
        let n = net.stands.len();
        let starts: Vec<&Stand> = [0, n / 2, n - 1].iter().map(|&k| &net.stands[k]).collect();
        for &(i, b, j) in conv {
            let (s, other) = (&rw[i], &rw[j]);
            let (end, tlat, tlon) = if b {
                (&s.ident_b, s.lat_b, s.lon_b)
            } else {
                (&s.ident_a, s.lat_a, s.lon_a)
            };
            let zone = [s.ident_a.clone(), s.ident_b.clone()];
            let standoff = (s.width_m / 2.0 + 30.0).max(HOLD_SHORT_STANDOFF_M);
            for st in &starts {
                let old = net.plan_to_runway(st.lat, st.lon, tlat, tlon, standoff, &zone, None);
                if old.as_ref().and_then(|p| judge(s, other, p)) == Some(false) {
                    old_wrong += 1;
                }
                let Some(plan) =
                    net.plan_to_runway(st.lat, st.lon, tlat, tlon, standoff, &zone, Some(s))
                else {
                    none += 1;
                    continue;
                };
                routes += 1;
                if judge(s, other, &plan) == Some(true) {
                    ok += 1;
                    if plan.lead_on.is_empty() {
                        untrimmed += 1;
                    }
                    continue;
                }
                let last = plan.lead_on.last().or(plan.path.last()).copied();
                // How much of OUR strip the network draws: nodes of
                // runway edges along it, and how many a taxiway reaches.
                // None reached is thin data: nothing but another
                // runway's pavement leads onto it.
                let aligned = |e: &&TaxiEdge| {
                    let (Some(&pa), Some(&pb)) = (net.nodes.get(&e.a), net.nodes.get(&e.b))
                    else {
                        return false;
                    };
                    let off = sim::angles::signed_diff_deg(bearing_deg(pa.0, pa.1, pb.0, pb.1), s.heading_a_deg())
                        .abs();
                    off.min(180.0 - off) <= STRIP_EDGE_ALIGN_DEG
                };
                let mid_in_rect = |e: &&TaxiEdge| {
                    let (Some(&pa), Some(&pb)) = (net.nodes.get(&e.a), net.nodes.get(&e.b))
                    else {
                        return false;
                    };
                    let (xt, along) = track(s, ((pa.0 + pb.0) / 2.0, (pa.1 + pb.1) / 2.0));
                    xt <= (s.width_m / 2.0).max(15.0) + STRIP_NODE_SHOULDER_M
                        && along >= -STRIP_NODE_SHOULDER_M
                        && along <= s.length_m() + STRIP_NODE_SHOULDER_M
                };
                let own: std::collections::HashSet<i64> = net
                    .edges
                    .iter()
                    .filter(|e| e.runway)
                    .filter(mid_in_rect)
                    .filter(aligned)
                    .flat_map(|e| [e.a, e.b])
                    .collect();
                let rwy_nodes = own.len();
                let any_nodes = own
                    .iter()
                    .filter(|n| net.edges.iter().any(|e| !e.runway && (e.a == **n || e.b == **n)))
                    .count();
                let list = if any_nodes == 0 { &mut thin } else { &mut wrong };
                list.push(format!(
                    "{icao} {end} [our strip: {rwy_nodes} runway nodes, {any_nodes} on a taxiway] (beside {}/{}) from {}: hold {} lead-on {} pts, end {:?} {:.0} m off ours, {:.0} m off theirs",
                    other.ident_a,
                    other.ident_b,
                    st.name,
                    plan.holding_point,
                    plan.lead_on.len(),
                    last,
                    last.map_or(f64::NAN, |p| track(s, p).0),
                    last.map_or(f64::NAN, |p| track(other, p).0),
                ));
            }
        }
    }
    println!(
        "{with_net} fields with a network and stands: {routes} routes, {ok} on their own strip \
             ({untrimmed} untrimmed holds nearer their own), {} on the neighbour's, {} on the \
             neighbour's where no taxiway reaches ours (thin data degrades), \
             {none} unplanned; the old nearest-anywhere rule: {old_wrong} on the neighbour's",
        wrong.len(),
        thin.len()
    );
    for w in wrong.iter().chain(&thin) {
        println!("  {w}");
    }
    assert!(routes > 0, "the sweep planned nothing");
    assert!(wrong.is_empty(), "{} routes end at the neighbour's entrance", wrong.len());
}

/// #190, LEMG (iMac 2026-09-29, sortie 1790712526, a Citation X at the
/// gate): today's planner, the route-start join (`join_first_edge_ahead`)
/// included, still starts the cleared route down the stand's lane (155
/// behind the 327.7 nose) and then the taxiway on 222: the join ahead
/// does not reach a start 81 m off its first edge, from the gate or from
/// where the tug let go. The painted lane read backwards ends facing 26.
/// `cargo test -p atc --release -- --ignored live_lemg_190 --nocapture`
#[test]
#[ignore] // needs the real X-Plane install
fn live_lemg_190_route_from_the_gate_starts_down_the_lane() {
    let apt = &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let Ok(Some(net)) = find_taxi_net(apt, "LEMG") else {
        return;
    };
    let rws = find_runways(apt, "LEMG").unwrap();
    let r = rws.iter().find(|r| r.ident_a == "12" || r.ident_b == "12").unwrap().clone();
    let (tlat, tlon) = if r.ident_a == "12" { (r.lat_a, r.lon_a) } else { (r.lat_b, r.lon_b) };
    let zone = [r.ident_a.clone(), r.ident_b.clone()];
    for (lat, lon) in [(36.67335705897913, -4.489441674810065), (36.6729659291503, -4.489131668721906)] {
        let plan = net.plan_to_runway(lat, lon, tlat, tlon, 100.0, &zone, Some(&r)).expect("a route");
        let first = bearing_deg(lat, lon, plan.path[1].0, plan.path[1].1);
        let then = bearing_deg(plan.path[1].0, plan.path[1].1, plan.path[2].0, plan.path[2].1);
        println!("from {lat},{lon}: {first:.1} then {then:.1}");
        assert!(sim::angles::abs_diff_deg(first, 327.7) > 150.0, "the way out is behind: {first}");
        assert!((then - 222.2).abs() < 1.0, "{then}");
    }
    let lane = net.push_lane(36.67335705897913, -4.489441674810065, 327.7).expect("a painted lane");
    let n = lane.len();
    let end = (bearing_deg(lane[n - 3].0, lane[n - 3].1, lane[n - 1].0, lane[n - 1].1) + 180.0) % 360.0;
    println!("lane {n} pts, ends facing {end:.1}");
    assert!((end - 26.0).abs() < 8.0, "{end}");
}

/// A T-junction as the network and the paint each draw it: Quebec north
/// to a node ON Delta's east-west centreline 50 m up, Quebec's paint
/// stopping 30 m short, a 30 m fillet each way onto Delta.
fn t_junction(fillets: bool) -> Vec<Vec<(f64, f64)>> {
    let arc = |sign: f64| -> Vec<(f64, f64)> {
        (0..=12)
            .map(|k| {
                let a = (90.0_f64 * k as f64 / 12.0).to_radians();
                ll(sign * (30.0 - 30.0 * a.cos()), 20.0 + 30.0 * a.sin())
            })
            .collect()
    };
    let mut paint = vec![
        vec![ll(0.0, -150.0), ll(0.0, if fillets { 20.0 } else { 50.0 })],
        vec![ll(-200.0, 50.0), ll(200.0, 50.0)],
    ];
    if fillets {
        paint.push(arc(-1.0));
        paint.push(arc(1.0));
    }
    paint
}

fn xy(p: (f64, f64)) -> (f64, f64) {
    ((p.1 - LON0) * 111_320.0 * LAT0.to_radians().cos(), (p.0 - LAT0) * 111_320.0)
}

fn sharpest(path: &[(f64, f64)]) -> f64 {
    vertex_turns(path, 111_320.0 * LAT0.to_radians().cos())
        .iter()
        .fold(0.0_f64, |a, t| a.max(t.abs()))
}

/// #194 (LEMG Quebec onto Delta, Citation X, 2026-09-30): a corner the
/// network cut across a drawn fillet turns on the fillet it turns AWAY
/// on, never on the one the other way, and the samples the snap threw
/// between the two go with it.
#[test]
fn a_corner_rides_the_fillet_it_turns_on() {
    let paint = t_junction(true);
    let seg = vec![("Q".to_string(), false), ("D".to_string(), false)];
    for (sign, name) in [(-1.0, "left"), (1.0, "right")] {
        let path = vec![ll(0.0, -150.0), ll(0.0, 50.0), ll(sign * 200.0, 50.0)];
        let (old, _) = ride_paint_with(&paint, &path, &seg, false);
        let (new, new_seg) = ride_paint(&paint, &path, &seg);
        assert_eq!(new_seg.len() + 1, new.len(), "{name}");
        assert_eq!(old.last(), new.last(), "{name}: the route ends where it did");
        // The snap alone turns at the node or saws between the fillets.
        assert!(sharpest(&old) > 40.0, "{name}: {:.0}", sharpest(&old));
        assert!(sharpest(&new) <= 15.0, "{name}: {:.0}", sharpest(&new));
        for p in new.iter().map(|&p| xy(p)) {
            assert!(p.0 * sign >= -0.5, "{name}: never on the other fillet: {p:?}");
            assert!(p.1 <= 50.5, "{name}: never past Delta: {p:?}");
        }
        // Through the fillet's middle, 30 (1 - cos 45) off each line.
        let mid = (sign * (30.0 - 30.0 * 45.0_f64.to_radians().cos()), 20.0 + 30.0 * 45.0_f64.to_radians().sin());
        let nearest = new.iter().map(|&p| {
            let q = xy(p);
            ((q.0 - mid.0).powi(2) + (q.1 - mid.1).powi(2)).sqrt()
        });
        assert!(nearest.fold(f64::INFINITY, f64::min) < 3.0, "{name}: on the fillet");
        // The names change where the node was: Quebec then Delta.
        assert_eq!(new_seg.first().map(|s| s.0.as_str()), Some("Q"));
        assert_eq!(new_seg.last().map(|s| s.0.as_str()), Some("D"));
    }
}

/// A junction drawn square has no fillet to ride, a runway leg is never
/// ridden, and a gentle bend is the snap's: all as the snap left them.
#[test]
fn a_corner_without_a_fillet_is_left_as_the_snap_made_it() {
    let seg = vec![("Q".to_string(), false), ("D".to_string(), false)];
    let path = vec![ll(0.0, -150.0), ll(0.0, 50.0), ll(-200.0, 50.0)];
    let square = t_junction(false);
    assert_eq!(ride_paint(&square, &path, &seg), ride_paint_with(&square, &path, &seg, false));
    let filleted = t_junction(true);
    let runway = vec![("Q".to_string(), false), ("12".to_string(), true)];
    assert_eq!(ride_paint(&filleted, &path, &runway), ride_paint_with(&filleted, &path, &runway, false));
    let gentle = vec![ll(0.0, -150.0), ll(0.0, 50.0), ll(-40.0, 200.0)];
    assert_eq!(ride_paint(&filleted, &gentle, &seg), ride_paint_with(&filleted, &gentle, &seg, false));
}

/// #194 as drawn at LEMG (from the install): Quebec's paint ends 47 m
/// short of Delta's centreline and the node sits on it. Before, the ride
/// turned 49, 45, -112 and -83 degrees in 30 m; now it rides lines 158
/// and 157 (the fillet left, through their joint) and turns under 15
/// degrees at any vertex.
#[test]
fn lemg_quebec_onto_delta_rides_the_left_fillet() {
    let raw: [(f64, f64); 3] = [(36.683599, -4.506806), (36.687126, -4.502939), (36.688841, -4.506956)];
    #[rustfmt::skip]
    let paint: Vec<Vec<(f64, f64)>> = vec![
        vec![(36.68680750, -4.50327940), (36.68681270, -4.50327312), (36.68682681, -4.50325461), (36.68684765, -4.50322436), (36.68687299, -4.50318286), (36.68690063, -4.50313061), (36.68692837, -4.50306808), (36.68695399, -4.50299578), (36.68697530, -4.50291420), (36.68698764, -4.50282780), (36.68698962, -4.50274279), (36.68698394, -4.50266229), (36.68697326, -4.50258944), (36.68696029, -4.50252734), (36.68694770, -4.50247912), (36.68693817, -4.50244790), (36.68693440, -4.50243680)],
        vec![(36.68734910, -4.50342280), (36.68734506, -4.50341464), (36.68733313, -4.50339276), (36.68731358, -4.50336104), (36.68728668, -4.50332335), (36.68725269, -4.50328358), (36.68721191, -4.50324561), (36.68716458, -4.50321332), (36.68711100, -4.50319060)],
        vec![(36.68711100, -4.50319060), (36.68704602, -4.50318040), (36.68698726, -4.50318389), (36.68693547, -4.50319725), (36.68689141, -4.50321670), (36.68685583, -4.50323844), (36.68682948, -4.50325866), (36.68681312, -4.50327358), (36.68680750, -4.50327940)],
        vec![(36.68332550, -4.50710330), (36.68386960, -4.50650580), (36.68680750, -4.50327940)],
        vec![(36.68320300, -4.49356340), (36.68693440, -4.50243680), (36.68734910, -4.50342280), (36.68877120, -4.50680470), (36.68932580, -4.50812360), (36.68958210, -4.50873320)],
    ];
    let seg = vec![("Q".to_string(), false), ("D".to_string(), false)];
    let m_lon = 111_320.0 * raw[1].0.to_radians().cos();
    let (old, _) = ride_paint_with(&paint, &raw, &seg, false);
    let (new, _) = ride_paint(&paint, &raw, &seg);
    let worst = |p: &[(f64, f64)]| vertex_turns(p, m_lon).iter().fold(0.0_f64, |a, t| a.max(t.abs()));
    assert!(worst(&old) > 100.0, "the recorded jog: {:.0}", worst(&old));
    assert!(worst(&new) < 15.0, "{:.0}", worst(&new));
    assert_eq!(old.last(), new.last());
    let d = |a: (f64, f64), b: (f64, f64)| (((a.1 - b.1) * m_lon).powi(2) + ((a.0 - b.0) * 111_320.0).powi(2)).sqrt();
    let joint = (36.68711100, -4.50319060);
    assert!(new.iter().any(|&p| d(p, joint) < 1.0), "through the fillet's joint");
    // Nothing east of the right fillet's start: the other fillet is not ridden.
    let right_mid = (36.68697530, -4.50291420);
    assert!(new.iter().all(|&p| d(p, right_mid) > 10.0));
}

/// #194: the fillet pass over real fields, old against new, stands to
/// every runway end. Prints what changed and fails on a new route that
/// is sharper than the old one or strays from it.
/// `cargo test --release -p atc -- --ignored real_install_fillet_sweep --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_fillet_sweep() {
    let apt =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    if !std::path::Path::new(apt).exists() {
        return;
    }
    let fields = std::env::var("FILLET_FIELDS").unwrap_or_else(|_| {
        "LEMG EGKK EGBB EGLL EGPH LFPB LFPG EDDF EHAM LEBL LIRF LSZH EBLG LIPR LIME LILB LSZS EDLP LFBD EIDW KJFK KSFO LIEA LSGG".into()
    });
    let turns = |pts: &[(f64, f64)]| -> Vec<f64> {
        let mut out = Vec::new();
        let mut prev: Option<f64> = None;
        let mut last = pts[0];
        for &q in &pts[1..] {
            let m = 111_320.0 * last.0.to_radians().cos();
            let d = (((q.1 - last.1) * m).powi(2) + ((q.0 - last.0) * 111_320.0).powi(2)).sqrt();
            if d < 1.0 {
                continue;
            }
            let b = bearing_deg(last.0, last.1, q.0, q.1);
            if let Some(p) = prev {
                out.push(sim::angles::signed_diff_deg(b, p));
            }
            prev = Some(b);
            last = q;
        }
        out
    };
    let off = |q: (f64, f64), line: &[(f64, f64)]| -> f64 {
        let m = 111_320.0 * q.0.to_radians().cos();
        line.windows(2)
            .map(|w| {
                let (ax, ay) = ((w[0].1 - q.1) * m, (w[0].0 - q.0) * 111_320.0);
                let (bx, by) = ((w[1].1 - q.1) * m, (w[1].0 - q.0) * 111_320.0);
                let (dx, dy) = (bx - ax, by - ay);
                let l2 = dx * dx + dy * dy;
                let t = if l2 < 1e-9 { 0.0 } else { (-(ax * dx + ay * dy) / l2).clamp(0.0, 1.0) };
                ((ax + t * dx).powi(2) + (ay + t * dy).powi(2)).sqrt()
            })
            .fold(f64::INFINITY, f64::min)
    };
    let (mut routes, mut changed, mut worse, mut smoother) = (0, 0, 0, 0);
    let mut worst_off = 0.0_f64;
    for icao in fields.split_whitespace() {
        let Ok(Some(net)) = find_taxi_net(apt, icao) else { continue };
        let Ok(runways) = find_runways(apt, icao) else { continue };
        let mut bare = net.clone();
        bare.paint.clear();
        let step = (net.stands.len() / 12).max(1);
        for st in net.stands.iter().step_by(step) {
            for r in &runways {
                for (id, tlat, tlon) in [(&r.ident_a, r.lat_a, r.lon_a), (&r.ident_b, r.lat_b, r.lon_b)] {
                    let zone = [r.ident_a.clone(), r.ident_b.clone()];
                    let Some(raw) = bare.plan_to_runway(st.lat, st.lon, tlat, tlon, 60.0, &zone, Some(r)) else {
                        continue;
                    };
                    if raw.path.len() < 2 {
                        continue;
                    }
                    routes += 1;
                    let (old, _) = ride_paint_with(&net.paint, &raw.path, &raw.seg, false);
                    let (new, _) = ride_paint_with(&net.paint, &raw.path, &raw.seg, true);
                    if old == new {
                        continue;
                    }
                    changed += 1;
                    let o = turns(&old).iter().fold(0.0_f64, |a, t| a.max(t.abs()));
                    let n = turns(&new).iter().fold(0.0_f64, |a, t| a.max(t.abs()));
                    let stray = new.iter().map(|&q| off(q, &old)).fold(0.0_f64, f64::max);
                    worst_off = worst_off.max(stray);
                    if n + 1.0 < o {
                        smoother += 1;
                    }
                    assert_eq!(old.last(), new.last(), "{icao}: the route ends where it ended");
                    if n > o + 1.0 || stray > 30.0 {
                        worse += 1;
                        println!("{icao} {} -> {id}: max vertex {o:.0} -> {n:.0}, strays {stray:.1} m", st.name);
                    }
                }
            }
        }
    }
    println!(
        "{routes} routes, {changed} changed, {smoother} smoother, {worse} sharper or astray, furthest {worst_off:.1} m from the old"
    );
    assert_eq!(worse, 0);
}

/// A strip whose only ways on at the 18 end are authored oneway OFF it
/// (exits), and a twoway stub at the 36 end: the LEMD 18R shape
/// (2026-10-05). The route to 18 used to ride the runway from the 36 end
/// to the numbers, the zone trim left the hold at the 36 end with that ride
/// as its lead-on, and the ride's runway edges were read as a crossing of
/// our own strip. Now the 36 end's stub is a BACKTRACK entrance: the hold
/// is there, the lead-on only joins the strip, nothing is crossed, and the
/// end is reported as having no way on at its threshold's half.
#[test]
fn an_exit_only_threshold_is_a_backtrack_never_a_runway_ride() {
    let mut net = TaxiNet::default();
    for (id, lat, lon) in [
        (1, 51.0300, -1.0000),
        (2, 51.0200, -1.0000),
        (3, 51.0100, -1.0000),
        (4, 51.0000, -1.0000),
        (11, 51.0290, -0.9980),
        (14, 51.0010, -0.9980),
        (21, 51.0290, -0.9960),
        (22, 51.0010, -0.9960),
        (23, 51.0150, -0.9960),
    ] {
        net.nodes.insert(id, (lat, lon));
    }
    let e = |a: i64, b: i64, name: &str, runway: bool, oneway: bool, z: &[&str]| TaxiEdge {
        a,
        b,
        name: name.into(),
        runway,
        oneway,
        zone_of: z.iter().map(|s| s.to_string()).collect(),
    };
    let zone = ["18", "36"];
    net.edges.push(e(1, 2, "18/36", true, false, &zone));
    net.edges.push(e(2, 3, "18/36", true, false, &zone));
    net.edges.push(e(3, 4, "18/36", true, false, &zone));
    // The 18 end's stub: an exit, oneway off the runway.
    net.edges.push(e(1, 11, "N", false, true, &zone));
    net.edges.push(e(11, 21, "N", false, false, &[]));
    net.edges.push(e(21, 23, "P", false, false, &[]));
    net.edges.push(e(23, 22, "P", false, false, &[]));
    net.edges.push(e(22, 14, "S", false, false, &[]));
    net.edges.push(e(14, 4, "S", false, false, &zone));
    let r = Runway {
        ident_a: "18".into(),
        ident_b: "36".into(),
        lat_a: 51.0300,
        lon_a: -1.0000,
        lat_b: 51.0000,
        lon_b: -1.0000,
        width_m: 45.0,
        surface: 1,
    };
    assert!(!net.departure_end_enterable(&r, "18"));
    assert!(net.departure_end_enterable(&r, "36"));
    let zone = ["18".to_string(), "36".to_string()];
    let p = net
        .plan_to_runway(51.0150, -0.9950, r.lat_a, r.lon_a, 60.0, &zone, Some(&r))
        .expect("the backtrack's route");
    assert_eq!(p.holding_point, "S", "{p:?}");
    assert!(p.crossings.is_empty(), "our own strip read as a crossing: {p:?}");
    let ride: f64 = p
        .lead_on
        .windows(2)
        .map(|w| haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0)
        .sum();
    assert!(ride < 300.0, "a {ride:.0} m lead-on rides the runway: {p:?}");
}

/// LEMD from the 737's stand on 2026-10-05: 18R, 32L and 32R have no way on
/// at their threshold's half (their stubs there are authored oneway off
/// the runway), so none gets a hold at its far end with a runway ride for a
/// lead-on; the other ends keep the holds they always had.
/// `cargo test --release -p atc -- --ignored real_install_lemd_18r_departure --nocapture`
#[test]
#[ignore] // needs the real X-Plane install; run with --ignored
fn real_install_lemd_18r_departure() {
    let apt =
        &crate::test_xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    if !std::path::Path::new(apt).exists() {
        return;
    }
    let a = find_airport(apt, "LEMD").unwrap().expect("LEMD");
    let net = find_taxi_net(apt, "LEMD").unwrap().expect("LEMD's network");
    for (id, hold, enterable) in [
        ("18R", "Z6", false),
        ("32L", "LF", false),
        ("32R", "K3", false),
        ("36L", "Z1", true),
        ("36R", "Y1", true),
        ("14L", "K1", true),
        ("14R", "LC", true),
        ("18L", "Y7", true),
    ] {
        let r = a.runways.iter().find(|r| r.ident_a == id || r.ident_b == id).unwrap();
        let (lat, lon) = if r.ident_a == id { (r.lat_a, r.lon_a) } else { (r.lat_b, r.lon_b) };
        let zone = [r.ident_a.clone(), r.ident_b.clone()];
        let p = net
            .plan_to_runway(40.49909, -3.56863, lat, lon, 100.0, &zone, Some(r))
            .expect("a route");
        let ride: f64 = p
            .lead_on
            .windows(2)
            .map(|w| haversine_nm(w[0].0, w[0].1, w[1].0, w[1].1) * 1852.0)
            .sum();
        println!("{id}: hold {} crossings {:?} lead-on {ride:.0} m", p.holding_point, p.crossings);
        assert_eq!(p.holding_point, hold, "{id}");
        assert_eq!(net.departure_end_enterable(r, id), enterable, "{id}");
        assert!(ride < 500.0, "{id}: a {ride:.0} m lead-on rides the runway");
        let own = format!("{}/{}", r.ident_a, r.ident_b);
        // 18L's route crosses 18L/36R near its end today, unchanged.
        assert!(enterable || !p.crossings.contains(&own), "{id}: crosses its own strip");
    }
}
