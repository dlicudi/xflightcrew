//! The recovery planner's pure decisions, and the LEMG 2026-10-05 requests
//! replayed against the plan's real fixes (earth_fix.dat, CIFP VULP1G).

use super::*;

fn wp(ident: &str, lat: f64, lon: f64, alt: f64) -> Waypoint {
    Waypoint {
        ident: ident.into(),
        lat,
        lon,
        alt,
    }
}

/// LFRG-LEMG's tail: VULPE and the VULP1G arrival, then the field. The
/// positions are the navigation data's; the navlog altitudes are a
/// descent's, near enough (the joins here are decided on geometry).
fn lemg_plan() -> Vec<Waypoint> {
    vec![
        wp("LFRG", 49.365, 0.154, 479.0),
        wp("MOCEQ", 38.309555556, -4.686638889, 21_000.0),
        wp("VULPE", 37.761222222, -4.798444444, 15_000.0),
        wp("MG549", 37.382388889, -4.666388889, 12_000.0),
        wp("PALOH", 37.267222222, -4.581777778, 11_000.0),
        wp("USOME", 37.066944444, -4.580166667, 9_000.0),
        wp("MG531", 36.933444444, -4.579083333, 8_000.0),
        wp("FOFUS", 36.932777778, -4.695500000, 7_000.0),
        wp("MG529", 36.999527778, -4.696138889, 7_000.0),
        wp("MG533", 37.066277778, -4.696777778, 6_000.0),
        wp("MG537", 37.133027778, -4.697444444, 6_000.0),
        wp("MG541", 37.199805556, -4.698083333, 6_000.0),
        wp("MG437", 37.199111111, -4.802388889, 6_000.0),
        wp("MG433", 37.132333333, -4.801666667, 5_000.0),
        wp("LENHI", 37.065583333, -4.800916667, 5_000.0),
        wp("LEMG", 36.674900000, -4.499100000, 53.0),
    ]
}

fn lemg_legs(w: &[Waypoint]) -> Vec<ProcLeg> {
    let mut legs = crate::procedure::enroute_legs(w, "LFRG", "LEMG", 35_000.0);
    crate::procedure::extend_to_arrival_gateway(&mut legs, w.last().unwrap(), Some(53.0));
    legs
}

fn join_at(lat: f64, lon: f64, track: f64, purpose: Purpose) -> Join {
    let w = lemg_plan();
    let legs = lemg_legs(&w);
    choose_join(
        &legs,
        &w,
        "LEMG",
        lat,
        lon,
        Some(-1.0),
        None,
        Some(track),
        purpose,
    )
    .unwrap()
}

/// 12:43:09 and 12:43:54: "Climb to 9,000 feet first ... outside the 3
/// mile rejoin corridor" both times, 14 and 16 nm from Malaga. The way in is
/// the arrival.
#[test]
fn the_refusals_at_12_43_are_the_arrival() {
    let j = join_at(36.7563502870743, -4.759707010167053, 308.8, Purpose::Resume);
    assert!(
        matches!(j, Join::Arrival { dist_nm } if (12.0..15.0).contains(&dist_nm)),
        "{j:?}"
    );
    let j = join_at(
        36.78159052255375,
        -4.799125431616119,
        308.5,
        Purpose::Resume,
    );
    assert!(
        matches!(j, Join::Arrival { dist_nm } if (14.0..18.0).contains(&dist_nm)),
        "{j:?}"
    );
}

/// 12:52:22: "land the aircraft", 8,000 ft, 25 nm south-west of Malaga with
/// Gibraltar 26 nm away: the destination is Malaga, inside autoland's reach.
#[test]
fn the_landing_request_at_12_52_is_malaga() {
    let (lat, lon) = (36.343807280612126, -4.8796170270074315);
    let j = join_at(lat, lon, 178.5, Purpose::Land);
    assert!(
        matches!(j, Join::Arrival { dist_nm } if (24.0..30.0).contains(&dist_nm)),
        "{j:?}"
    );
    // As a resume it is past the arrival radius: direct to the best fix,
    // never a refusal.
    let j = join_at(lat, lon, 178.5, Purpose::Resume);
    assert!(
        matches!(j, Join::Direct { .. } | Join::Arrival { .. }),
        "{j:?}"
    );
}

/// 12:55:56: "not abeam any leg of the route ... LEMG is 41 miles away", at
/// 9,300 ft south-west of Malaga heading away. Direct to the fix with the
/// shortest way in: the arrival's gateway north-west of the field.
#[test]
fn the_refusal_at_12_55_is_a_direct_join() {
    let (lat, lon) = (36.15014184698853, -5.0519005103250585);
    let j = join_at(lat, lon, 224.5, Purpose::Resume);
    let Join::Direct {
        index,
        dist_nm,
        via_nm,
    } = j
    else {
        panic!("{j:?}")
    };
    let w = lemg_plan();
    let legs = lemg_legs(&w);
    let fix = legs[index].fix.as_ref().unwrap();
    assert!(
        dist_nm > 35.0 && dist_nm < 60.0,
        "{dist_nm} to {}",
        fix.ident
    );
    assert!(via_nm < 75.0, "{via_nm}");
    // Never a fix behind the arrival: the way through it is the shortest.
    for (i, l) in legs.iter().enumerate() {
        if let (Some(f), true) = (l.fix.as_ref(), i != index) {
            let other =
                crate::nav::haversine_nm(lat, lon, f.lat, f.lon) + remaining_nm(&legs, i, w.last());
            assert!(
                other + 1e-6 >= via_nm - 6.0,
                "{} {other} vs {via_nm}",
                f.ident
            );
        }
    }
    eprintln!(
        "12:55:56: direct {} ({}), {dist_nm:.0} nm, {via_nm:.0} nm to LEMG",
        fix.ident, legs[index].fix_ident
    );
}

#[test]
fn on_the_route_the_leg_is_joined_and_near_the_origin_the_first_fix() {
    let w = lemg_plan();
    let legs = lemg_legs(&w);
    // On the MG541 to MG437 leg.
    let j = join_at(37.20, -4.75, 270.0, Purpose::Resume);
    assert!(
        matches!(j, Join::Abeam(i) if legs[i].fix_ident == "MG437"),
        "{j:?}"
    );
    // Climbing out of Rouen, abeam nothing yet.
    let j = choose_join(
        &legs,
        &w,
        "LEMG",
        49.30,
        0.10,
        None,
        None,
        Some(200.0),
        Purpose::Resume,
    );
    assert!(matches!(j, Some(Join::Departure | Join::Abeam(_))), "{j:?}");
}

#[test]
fn a_direct_fix_behind_costs_its_turn() {
    // Two fixes, equally far to go: the one ahead wins.
    let w = vec![
        wp("A", 0.0, 0.0, 0.0),
        wp("B", 0.0, 1.0, 0.0),
        wp("C", 0.0, 2.0, 0.0),
        wp("D", 0.0, 3.0, 0.0),
    ];
    let legs = crate::procedure::enroute_legs(&w, "A", "D", 10_000.0);
    let ahead = best_direct(&legs, w.last(), 0.5, 1.5, Some(90.0)).unwrap();
    let Join::Direct { index, .. } = ahead else {
        panic!()
    };
    assert_eq!(legs[index].fix_ident, "C");
}

#[test]
fn below_the_way_s_floor_the_first_leg_is_a_climb() {
    assert_eq!(climb_first(2_930.0, 4_730.0), Some(4_800.0));
    assert_eq!(climb_first(4_650.0, 4_730.0), None, "inside the detent");
    assert_eq!(climb_first(9_000.0, 4_730.0), None);
    let leg = climb_leg(-54.0, 4_800.0);
    assert_eq!(leg.leg_type, "VA");
    assert_eq!(leg.course_mag, Some(306.0));
    assert!(leg.fix.is_none());
    // The follower's own test of a to-altitude leg: done at the altitude.
    assert!(!crate::procedure::leg_captured(&leg, None, 4_500.0, 0.5));
    assert!(crate::procedure::leg_captured(&leg, None, 4_790.0, 0.5));
}

#[test]
fn only_the_physically_impossible_is_refused_and_it_names_its_number() {
    assert_eq!(
        impossible(
            "LEMG",
            9_000.0,
            41_000.0,
            Some(60.0),
            41.0,
            250.0,
            Some(("13", 3_200.0, 1_634.0))
        ),
        None
    );
    let c = impossible("LSZS", 15_500.0, 14_000.0, None, 20.0, 120.0, None).unwrap();
    assert!(c.contains("15,500") && c.contains("14,000"), "{c}");
    let f = impossible("LEMG", 5_000.0, 41_000.0, Some(15.0), 41.0, 250.0, None).unwrap();
    assert!(
        f.contains("15 minutes") && f.contains("10 minutes away"),
        "{f}"
    );
    let r = impossible(
        "LXGB",
        5_000.0,
        41_000.0,
        None,
        20.0,
        250.0,
        Some(("09", 1_763.0, 1_900.0)),
    )
    .unwrap();
    assert!(r.contains("1763") && r.contains("1900"), "{r}");
    for line in [c, f, r] {
        assert!(
            !line.contains('\u{2014}') && !line.contains('\u{2013}'),
            "{line}"
        );
    }
}

/// A slow aircraft's time to go is its own groundspeed down to the planners'
/// 60 kt floor: 30 nm at 70 kt is 26 minutes, so 30 minutes of fuel is short
/// of it plus the approach's 10 (at the old 100 kt floor it was 18 and flew).
#[test]
fn a_slow_aircraft_is_timed_at_its_own_groundspeed() {
    let f = impossible("LEMG", 3_000.0, 14_000.0, Some(30.0), 30.0, 70.0, None).unwrap();
    assert!(f.contains("26 minutes away"), "{f}");
    let crawl = impossible("LEMG", 3_000.0, 14_000.0, Some(30.0), 30.0, 20.0, None).unwrap();
    assert!(crawl.contains("30 minutes away"), "floored at 60 kt: {crawl}");
}

#[test]
fn the_path_is_probed_every_quarter_mile_to_forty() {
    let p = path_points(36.0, -5.0, 36.0, -4.0);
    let last = p.last().unwrap();
    let d = crate::nav::haversine_nm(36.0, -5.0, last.0, last.1);
    assert!((d - 40.0).abs() < 0.1, "{d}");
    assert_eq!(p.len(), 160);
    let short = path_points(36.0, -5.0, 36.0, -4.99);
    assert!(short.len() >= 1);
}

/// EGKK-EGPH, 737 at FL380, 2026-10-16 (sortie 1792158539-full_auto): "can
/// you resume the flight plan?" 13 nm west of the RIBEL-INPIP leg with the
/// FMS on INPIP. The shortest way in is the synthetic gateway, which no FMS
/// holds; the box's own INPIP is barely longer, so the join goes there and
/// the STAR is flown. A fix the route lacks, an unread FMS or a fix far
/// behind keeps the join.
#[test]
fn a_direct_join_goes_to_the_fms_active_fix_when_it_is_no_detour() {
    let w = vec![
        wp("EGKK", 51.148, -0.190, 202.0),
        wp("NELSA", 53.863333, -2.184797, 31_900.0),
        wp("RIBEL", 54.016111, -2.289656, 30_100.0),
        wp("INPIP", 54.876539, -2.896139, 26_000.0),
        wp("INREV", 55.149289, -3.094294, 20_000.0),
        wp("ESKDO", 55.298889, -3.204239, 17_000.0),
        wp("TARTN", 55.717192, -3.138536, 7_000.0),
        wp("EGPH", 55.950145, -3.372288, 136.0),
    ];
    let mut legs = crate::procedure::enroute_legs(&w, "EGKK", "EGPH", 38_000.0);
    crate::procedure::extend_to_arrival_gateway(&mut legs, w.last().unwrap(), Some(136.0));
    let (lat, lon, trk) = (54.40, -2.96, 7.7);
    let join = choose_join(
        &legs,
        &w,
        "EGPH",
        lat,
        lon,
        None,
        None,
        Some(trk),
        Purpose::Resume,
    )
    .unwrap();
    let ident = |j: &Join| legs[j.leg_index().unwrap()].fix_ident.clone();
    assert_eq!(ident(&join), "EGPH arrival", "{join:?}");
    let to = |fms: Option<&str>| {
        toward_active_fix(join.clone(), &legs, w.last(), lat, lon, Some(trk), fms)
    };
    let j = to(Some("INPIP"));
    assert!(matches!(j, Join::Direct { .. }), "{j:?}");
    assert_eq!(ident(&j), "INPIP");
    assert_eq!(to(Some("D257B")), join, "not in the route");
    assert_eq!(to(Some("")), join, "an empty FMS");
    assert_eq!(to(None), join, "unread");
    assert_eq!(to(Some("RIBEL")), join, "behind: a detour");
    // Only a direct join is redirected.
    let abeam = Join::Abeam(3);
    assert_eq!(
        toward_active_fix(
            abeam.clone(),
            &legs,
            w.last(),
            lat,
            lon,
            Some(trk),
            Some("INPIP")
        ),
        abeam
    );
}
