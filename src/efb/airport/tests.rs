use super::*;
use crate::nav::{Runway, Stand, TaxiEdge};

/// LEMG's 13/31 and 12/30 as apt.dat's 100 rows have them.
fn lemg() -> AirportInfo {
    let rwy = |a: &str, la: f64, oa: f64, b: &str, lb: f64, ob: f64| Runway {
        ident_a: a.into(),
        ident_b: b.into(),
        lat_a: la,
        lon_a: oa,
        lat_b: lb,
        lon_b: ob,
        width_m: 45.11,
        surface: 1,
    };
    AirportInfo {
        name: "Malaga".into(),
        elev_ft: 52.0,
        runways: vec![
            rwy("12", 36.6910753, -4.5078270, "30", 36.6795648, -4.4805125),
            rwy("13", 36.6845573, -4.5126253, "31", 36.6654027, -4.4858140),
        ],
        freqs: Vec::new(),
        transition_level_ft: None,
        allows_circuits: None,
        pattern_sides: Vec::new(),
    }
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn a_runway_end_lands_at_its_metres() {
    // From runway 13's threshold, 31's lies 2,391 m east and 2,130 m south.
    let f = Frame::new(36.6845573, -4.5126253);
    let (x, y) = f.xy(36.6654027, -4.4858140);
    assert!(close(x, 2390.8, 0.5) && close(y, -2129.9, 0.5), "{x} {y}");
    assert_eq!(f.xy(36.6845573, -4.5126253), (0.0, 0.0));
}

#[test]
fn the_diagram_keeps_each_runway_its_length_and_heading() {
    let apt = lemg();
    let d = build_diagram("lemg", &apt, None).expect("runways");
    assert_eq!(d.icao, "LEMG");
    assert_eq!(d.runways.len(), 2);
    for (geo, r) in d.runways.iter().zip(&apt.runways) {
        let (dx, dy) = (geo.b.x - geo.a.x, geo.b.y - geo.a.y);
        let len = dx.hypot(dy);
        assert!(
            close(len, r.length_m(), 2.0),
            "{} {len} vs {}",
            geo.a.id,
            r.length_m()
        );
        let brg = dx.atan2(dy).to_degrees().rem_euclid(360.0);
        assert!(
            close(brg, r.heading_a_deg(), 0.1),
            "{} {brg} vs {}",
            geo.a.id,
            r.heading_a_deg()
        );
        assert_eq!(geo.width_m, 45.1);
        assert!(geo.hard);
    }
    // The reference point is the middle of the runways' extent.
    let [x0, y0, x1, y1] = d.bounds;
    assert!(
        close(x0 + x1, 0.0, 0.2) && close(y0 + y1, 0.0, 0.2),
        "{:?}",
        d.bounds
    );
    assert!(d.sources.pavement.starts_with("none"));
    assert_eq!(d.version.len(), 16);
    // The same input is the same version; a different field is not.
    assert_eq!(
        build_diagram("LEMG", &apt, None).unwrap().version,
        d.version
    );
    let mut other = apt.clone();
    other.runways.pop();
    assert_ne!(
        build_diagram("LEMG", &other, None).unwrap().version,
        d.version
    );
}

/// A tiny field: runway 09/27 along y = 0, a taxiway A parallel to the
/// south joined by a link B whose north half is inside 09/27's zone.
fn net(frame: &Frame) -> TaxiNet {
    let at = |x: f64, y: f64| {
        (
            frame.lat0 + y / M_PER_DEG,
            frame.lon0 + x / (M_PER_DEG * frame.lat0.to_radians().cos()),
        )
    };
    let mut n = TaxiNet::default();
    for (id, x, y) in [
        (1, -500.0, 0.0),
        (2, 500.0, 0.0),
        (3, 0.0, 0.0),
        (4, 0.0, -90.0),
        (5, 0.0, -180.0),
        (6, -400.0, -180.0),
        (7, 400.0, -180.0),
    ] {
        n.nodes.insert(id, at(x, y));
    }
    let e = |a: i64, b: i64, runway: bool, name: &str, zone: &[&str]| TaxiEdge {
        a,
        b,
        oneway: false,
        runway,
        name: name.into(),
        zone_of: zone.iter().map(|s| s.to_string()).collect(),
    };
    n.edges = vec![
        e(1, 3, true, "09/27", &["09", "27"]),
        e(3, 2, true, "09/27", &["09", "27"]),
        e(3, 4, false, "B", &["27", "09"]),
        e(4, 5, false, "B", &[]),
        e(6, 5, false, "A", &[]),
        e(5, 7, false, "A", &[]),
    ];
    let (sl, so) = at(300.0, -260.0);
    n.stands.push(Stand {
        lat: sl,
        lon: so,
        heading: 360.0,
        name: "Stand 1".into(),
        kind: "gate".into(),
        accepts: Vec::new(),
    });
    n
}

#[test]
fn a_hold_is_where_the_zone_meets_the_taxiway() {
    let f = Frame::new(51.0, 0.0);
    let h = holds(&net(&f), &f);
    assert_eq!(h.len(), 1, "{h:?}");
    assert!(
        close(h[0].x, 0.0, 0.1) && close(h[0].y, -90.0, 0.1),
        "{h:?}"
    );
    assert_eq!(h[0].rwy, "09/27", "sorted by number");
}

#[test]
fn every_taxiway_is_labelled_once_on_its_longest_edge() {
    let f = Frame::new(51.0, 0.0);
    let l = taxiway_labels(&net(&f), &f);
    let names: Vec<&str> = l.iter().map(|l| l.name.as_str()).collect();
    assert!(names.contains(&"A") && names.contains(&"B"), "{names:?}");
    assert!(
        !names.iter().any(|n| n.contains('/')),
        "runways are not taxiways"
    );
    let a: Vec<&Label> = l.iter().filter(|l| l.name == "A").collect();
    assert_eq!(a.len(), 2, "two A edges 400 m apart: {a:?}");
    assert!(a.iter().all(|l| close(l.brg, 90.0, 0.5)));
}

#[test]
fn a_field_without_pavement_draws_its_network_and_says_so() {
    let apt = lemg();
    let f = Frame::for_airport(&apt).unwrap();
    let n = net(&f);
    let d = build_diagram("LEMG", &apt, Some(&n)).unwrap();
    assert!(d.pavement.is_empty());
    assert_eq!(d.network.len(), 4, "taxiway edges only");
    assert!(d.sources.pavement.contains("1202"));
    assert_eq!(d.stands.len(), 1);
    assert_eq!(d.stands[0].hdg, 0.0);
    assert!(close(d.stands[0].x, 300.0, 0.2) && close(d.stands[0].y, -260.0, 0.2));
}

#[test]
fn a_straight_run_of_points_simplifies_to_its_ends() {
    let pts: Vec<(f64, f64)> = (0..50).map(|i| (i as f64, 0.01 * (i % 2) as f64)).collect();
    assert_eq!(simplify(&pts, 0.4), vec![(0.0, 0.0), (49.0, 0.01)]);
    let corner = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)];
    assert_eq!(simplify(&corner, 0.4), corner);
}

#[test]
fn the_route_splits_where_the_aircraft_is() {
    let pts = vec![(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)];
    let (d, r, rem) = split_route(&pts, Some((60.0, 3.0)));
    assert_eq!(d, vec![(0.0, 0.0), (60.0, 0.0)]);
    assert_eq!(r, vec![(60.0, 0.0), (100.0, 0.0), (100.0, 100.0)]);
    assert!(close(rem, 140.0, 1e-9));
    // Round the corner: the second leg.
    let (d, r, rem) = split_route(&pts, Some((102.0, 30.0)));
    assert_eq!(d.last(), Some(&(100.0, 30.0)));
    assert_eq!(r.first(), Some(&(100.0, 30.0)));
    assert!(close(rem, 70.0, 1e-9));
    // No aircraft: nothing driven, all of it to go.
    let (d, r, rem) = split_route(&pts, None);
    assert!(d.is_empty());
    assert_eq!(r, pts);
    assert!(close(rem, 200.0, 1e-9));
}

fn route(f: &Frame, to_stand: bool) -> RouteFacts {
    let at = |x: f64, y: f64| {
        (
            f.lat0 + y / M_PER_DEG,
            f.lon0 + x / (M_PER_DEG * f.lat0.to_radians().cos()),
        )
    };
    RouteFacts {
        source: "cleared",
        // South on A, across runway 09/27 (a runway leg), on to the hold at C1.
        path: vec![
            at(0.0, -300.0),
            at(0.0, -100.0),
            at(0.0, 100.0),
            at(0.0, 300.0),
        ],
        seg: vec![
            ("A".into(), false),
            ("09/27".into(), true),
            ("C".into(), false),
        ],
        via: "Alpha, Charlie".into(),
        holding_point: if to_stand {
            "Stand 4".into()
        } else {
            "C1".into()
        },
        to_stand,
    }
}

fn tick_at(f: &Frame, x: f64, y: f64, on_ground: bool) -> Tick {
    Tick {
        lat: f.lat0 + y / M_PER_DEG,
        lon: f.lon0 + x / (M_PER_DEG * f.lat0.to_radians().cos()),
        on_ground,
        gs_kts: 12.0,
        track_deg: Some(1.0),
        heading_deg: 359.0,
        ..Default::default()
    }
}

fn diagram_ref(icao: &str, f: &Frame) -> DiagramRef {
    DiagramRef {
        icao: icao.into(),
        name: "Test".into(),
        version: "0123456789abcdef".into(),
        lat0: f.lat0,
        lon0: f.lon0,
        runways: Vec::new(),
    }
}

#[test]
fn the_route_is_drawn_in_the_frame_with_its_crossing_and_its_end() {
    let f = Frame::new(51.0, -0.2);
    let a = AirportFacts {
        station: Some(("EGKK".into(), 51.0, -0.2)),
        diagram: Some(diagram_ref("EGKK", &f)),
        route: Some(route(&f, false)),
        card: Some(CardFacts {
            kind: 'C',
            dist_m: 118.0,
            label: "cross 09/27".into(),
            aim_true_deg: 360.4,
            xtk_m: -2.04,
            turn_deg: 0.0,
            remaining_m: 498.0,
        }),
        cleared_runway: Some("26L".into()),
        ..Default::default()
    };
    let t = tick_at(&f, 2.0, -200.0, true);
    let p = page(&a, Some(&t), None, false);
    assert_eq!(p.status, "ready");
    assert_eq!(p.icao.as_deref(), Some("EGKK"));
    assert_eq!(p.why, Some("here"));
    let own = p.own.expect("own");
    assert!(
        close(own.x, 2.0, 0.1) && close(own.y, -200.0, 0.1),
        "{own:?}"
    );
    assert_eq!(own.heading, 359.0);
    let r = p.route.expect("route");
    assert_eq!(r.driven.len(), 4, "start and the foot: {:?}", r.driven);
    assert!(close(r.driven[3], -200.0, 0.1));
    assert_eq!(r.remaining.len(), 8);
    assert!(close(r.remaining_m, 500.0, 0.5));
    assert_eq!(r.holds.len(), 1);
    assert_eq!(r.holds[0].label, "09/27");
    assert!(close(r.holds[0].y, -100.0, 0.1));
    let dest = r.dest.expect("dest");
    assert_eq!((dest.kind, dest.label.as_str()), ("hold", "C1"));
    assert!(close(dest.y, 300.0, 0.1));
    let c = p.card.expect("card");
    assert_eq!(c.text, "Cross 09/27 in 120 m");
    assert_eq!(c.aim_true_deg, 0.4);
    assert_eq!(c.xtk_m, -2.0, "signed, left of the route");
    let rwy = p.runway.expect("cleared runway");
    assert_eq!((rwy.ident.as_str(), rwy.why), ("26L", "cleared"));

    // To a stand the end is the stand.
    let a2 = AirportFacts {
        route: Some(route(&f, true)),
        ..a.clone()
    };
    let d = page(&a2, Some(&t), None, false)
        .route
        .unwrap()
        .dest
        .unwrap();
    assert_eq!((d.kind, d.label.as_str()), ("stand", "Stand 4"));

    // A route held at a field 50 km away is not drawn here, nor its card.
    let far = Frame::new(51.45, -0.2);
    let a3 = AirportFacts {
        route: Some(route(&far, false)),
        ..a.clone()
    };
    let p3 = page(&a3, Some(&t), None, false);
    assert!(p3.route.is_none() && p3.card.is_none());

    // No route: no card.
    let a4 = AirportFacts { route: None, ..a };
    assert!(page(&a4, Some(&t), None, false).card.is_none());
}

fn plan(origin: &str, o: (f64, f64), dest: &str, d: (f64, f64)) -> PlanFacts {
    PlanFacts {
        origin: origin.into(),
        destination: dest.into(),
        origin_rwy: Some("13".into()),
        dest_rwy: Some("26L".into()),
        waypoints: vec![
            (origin.into(), o.0, o.1, 0.0),
            ("NORMI".into(), 45.0, -1.0, 35_000.0),
            (dest.into(), d.0, d.1, 0.0),
        ],
        ..Default::default()
    }
}

#[test]
fn the_field_is_the_departure_on_the_ground_and_the_destination_in_the_air() {
    let lemg = (36.675, -4.499);
    let egkk = (51.148, -0.190);
    let p = plan("LEMG", lemg, "EGKK", egkk);
    let a = AirportFacts::default();
    let ground = |pos: (f64, f64)| Tick {
        lat: pos.0,
        lon: pos.1,
        on_ground: true,
        ..Default::default()
    };
    // At Malaga on the stand.
    let c = wanted_field(&a, Some(&ground((36.68, -4.49))), Some(&p), false).unwrap();
    assert_eq!((c.icao.as_str(), c.why), ("LEMG", "departure"));
    // In the air anywhere: the destination.
    let mut air = ground((45.0, -1.0));
    air.on_ground = false;
    let c = wanted_field(&a, Some(&air), Some(&p), true).unwrap();
    assert_eq!((c.icao.as_str(), c.why), ("EGKK", "destination"));
    // Landed at Gatwick: the field we are on.
    let c = wanted_field(&a, Some(&ground((51.15, -0.18))), Some(&p), false).unwrap();
    assert_eq!((c.icao.as_str(), c.why), ("EGKK", "destination"));
    // On the ground at a third field the station serves: that field.
    let a2 = AirportFacts {
        station: Some(("LEZL".into(), 37.418, -5.893)),
        ..Default::default()
    };
    let c = wanted_field(&a2, Some(&ground((37.42, -5.89))), Some(&p), false).unwrap();
    assert_eq!((c.icao.as_str(), c.why), ("LEZL", "here"));
    // The station at the plan's own origin keeps the plan's word for it.
    let a3 = AirportFacts {
        station: Some(("LEMG".into(), 36.68, -4.49)),
        ..Default::default()
    };
    let c = wanted_field(&a3, Some(&ground((36.68, -4.49))), Some(&p), false).unwrap();
    assert_eq!((c.icao.as_str(), c.why), ("LEMG", "departure"));
    let mut no_pos = p.clone();
    no_pos.waypoints.remove(0);
    let c = wanted_field(&a3, Some(&ground((36.68, -4.49))), Some(&no_pos), false).unwrap();
    assert_eq!(
        (c.icao.as_str(), c.why),
        ("LEMG", "departure"),
        "the station's position stands in"
    );
    // Nowhere near anything known: the departure still.
    let c = wanted_field(&a2, Some(&ground((40.0, 0.0))), Some(&p), false).unwrap();
    assert_eq!(c.icao, "LEMG");
    // No plan: the station's field, on the ground or in the air.
    let c = wanted_field(&a2, Some(&air), None, true).unwrap();
    assert_eq!((c.icao.as_str(), c.why), ("LEZL", "here"));
    // Nothing known: no field.
    assert!(wanted_field(&a, None, None, false).is_none());
}

#[test]
fn the_page_says_loading_until_the_wanted_field_is_built() {
    let f = Frame::new(51.148, -0.190);
    let p = plan("LEMG", (36.675, -4.499), "EGKK", (51.148, -0.190));
    let mut air = tick_at(&f, 0.0, -20_000.0, false);
    air.alt_ft = 3000.0;
    // Built for the departure, flying to the destination: loading.
    let a = AirportFacts {
        diagram: Some(diagram_ref("LEMG", &f)),
        arrival_runway: Some("08R".into()),
        ..Default::default()
    };
    let out = page(&a, Some(&air), Some(&p), true);
    assert_eq!(out.icao.as_deref(), Some("EGKK"));
    assert_eq!(out.status, "loading");
    assert!(out.version.is_none() && out.own.is_none());
    // Built: the aircraft on final, the arrival runway marked.
    let a = AirportFacts {
        diagram: Some(diagram_ref("EGKK", &f)),
        ..a
    };
    let out = page(&a, Some(&air), Some(&p), true);
    assert_eq!(out.status, "ready");
    assert_eq!(out.version.as_deref(), Some("0123456789abcdef"));
    assert!(out
        .own
        .is_some_and(|o| !o.on_ground && close(o.y, -20_000.0, 0.1)));
    let r = out.runway.unwrap();
    assert_eq!((r.ident.as_str(), r.why), ("08R", "arrival"));
    // Without full auto's runway, the plan's.
    let a = AirportFacts {
        arrival_runway: None,
        ..a
    };
    let r = page(&a, Some(&air), Some(&p), true).runway.unwrap();
    assert_eq!((r.ident.as_str(), r.why), ("26L", "the plan's"));
    // A field that could not be built says so.
    let a = AirportFacts {
        diagram: None,
        unavailable: true,
        ..a
    };
    assert_eq!(page(&a, Some(&air), Some(&p), true).status, "unavailable");
}

#[test]
fn a_card_reads_like_the_panel() {
    let c = |kind: char, d: f64, label: &str| {
        card_text(&CardFacts {
            kind,
            dist_m: d,
            label: label.into(),
            aim_true_deg: 0.0,
            xtk_m: 0.0,
            turn_deg: 0.0,
            remaining_m: 0.0,
        })
    };
    assert_eq!(c('T', 123.0, "left onto B"), "Left onto B in 120 m");
    assert_eq!(c('T', 8.0, "right onto A1"), "Right onto A1 now");
    assert_eq!(c('H', 1460.0, "hold short 26L"), "Hold short 26L in 1.5 km");
    assert_eq!(c('X', 0.0, "off route"), "Off the route");
}

/// The real install's LEMG and EGKK, written as the page fetches them, for
/// the headless look: `EFB_DIAGRAM_OUT=target/efb cargo test -- --ignored
/// real_install_diagrams --nocapture`.
#[test]
#[ignore] // needs the real X-Plane install
fn real_install_diagrams() {
    let apt = crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let out = std::env::var("EFB_DIAGRAM_OUT").ok();
    for icao in ["LEMG", "EGKK"] {
        let t = Instant::now();
        let d = load(&apt, icao).expect("the field builds");
        let json = serde_json::to_string(&d).unwrap();
        println!(
            "{icao}: {} runways, {} pavement polygons, {} lines, {} labels, {} holds, {} stands, {} kB, {:?}; {}",
            d.runways.len(),
            d.pavement.len(),
            d.lines.len(),
            d.taxiways.len(),
            d.holds.len(),
            d.stands.len(),
            json.len() / 1024,
            t.elapsed(),
            d.sources.pavement,
        );
        assert!(!d.pavement.is_empty(), "{icao} draws its pavement");
        assert!(!d.holds.is_empty() && !d.stands.is_empty());
        // Each runway keeps its length through the projection.
        let info = crate::nav::find_airport(&apt, icao).unwrap().unwrap();
        for (g, r) in d.runways.iter().zip(&info.runways) {
            let len = (g.b.x - g.a.x).hypot(g.b.y - g.a.y);
            assert!(
                close(len, r.length_m(), 3.0),
                "{icao} {}: {len} vs {}",
                g.a.id,
                r.length_m()
            );
        }
        if let Some(dir) = &out {
            std::fs::create_dir_all(dir).unwrap();
            std::fs::write(format!("{dir}/{icao}.json"), &json).unwrap();
        }
    }
}

/// Page states over the real LEMG and EGKK, for the headless look: a 737 a
/// third of the way down a planned taxi from a stand to LEMG 13, and one
/// taxiing in to a Gatwick stand off 26L. Written beside the diagrams:
/// `EFB_DIAGRAM_OUT=target/efb cargo test -- --ignored real_install_page_states`.
#[test]
#[ignore] // needs the real X-Plane install
fn real_install_page_states() {
    use crate::efb::state::{build, tests::cruise};
    use crate::flight::{Control, Phase};
    let apt = crate::config::xplane_path("Global Scenery/Global Airports/Earth nav data/apt.dat");
    let dir = std::env::var("EFB_DIAGRAM_OUT").expect("EFB_DIAGRAM_OUT");
    std::fs::create_dir_all(&dir).unwrap();
    for (icao, rwy, outbound) in [("LEMG", "13", true), ("EGKK", "26L", false)] {
        let d = load(&apt, icao).expect("diagram");
        let info = crate::nav::find_airport(&apt, icao).unwrap().unwrap();
        let net = crate::nav::find_taxi_net(&apt, icao).unwrap().unwrap();
        std::fs::write(
            format!("{dir}/{icao}.json"),
            serde_json::to_string(&d).unwrap(),
        )
        .unwrap();
        let strip = info
            .runways
            .iter()
            .find(|r| r.ident_a == rwy || r.ident_b == rwy)
            .unwrap();
        let (tla, tlo) = if strip.ident_a == rwy {
            (strip.lat_a, strip.lon_a)
        } else {
            (strip.lat_b, strip.lon_b)
        };
        // A stand about a mile from the threshold, so the route has turns.
        let stand = net
            .stands
            .iter()
            .filter(|s| s.is_suitable())
            .min_by(|a, b| {
                let da = crate::nav::haversine_nm(a.lat, a.lon, tla, tlo);
                let db = crate::nav::haversine_nm(b.lat, b.lon, tla, tlo);
                (da - 1.0).abs().total_cmp(&(db - 1.0).abs())
            })
            .unwrap()
            .clone();
        let plan = if outbound {
            net.plan_to_runway(
                stand.lat,
                stand.lon,
                tla,
                tlo,
                60.0,
                &[strip.ident_a.clone(), strip.ident_b.clone()],
                Some(strip),
            )
        } else {
            let mid = (
                (strip.lat_a + strip.lat_b) / 2.0,
                (strip.lon_a + strip.lon_b) / 2.0,
            );
            net.plan_to_stand(mid.0, mid.1, &stand)
        }
        .expect("a route");
        let watch = crate::party_line::TaxiWatch {
            path: plan.path.clone(),
            via: plan.via.join(", "),
            seg: plan.seg.clone(),
            holding_point: if outbound {
                plan.holding_point.clone()
            } else {
                stand.name.clone()
            },
            lead_on: plan.lead_on.clone(),
        };
        // A third of the way along, by points.
        let k = (watch.path.len() / 3).max(1);
        let (a, b) = (watch.path[k - 1], watch.path[k]);
        let (lat, lon) = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
        let hdg = crate::nav::bearing_deg(a.0, a.1, b.0, b.1);
        let card = crate::taxi_guide::Guide::build(&watch)
            .and_then(|g| g.turn_card(lat, lon))
            .map(|c| CardFacts::of(&c));
        let mut f = cruise();
        if let Some(s) = f.flight.as_mut() {
            s.phase = if outbound {
                Phase::TaxiOut
            } else {
                Phase::TaxiIn
            };
            s.control = Control::Automation("auto taxi");
            s.alt_ft = info.elev_ft;
            s.ias_kts = 0.0;
            s.gs_kts = 14.0;
        }
        f.owner = Some("auto taxi");
        f.mission = Some("full auto");
        if let Some(t) = f.tick.as_mut() {
            t.lat = lat;
            t.lon = lon;
            t.on_ground = true;
            t.gs_kts = 14.0;
            t.ias_kts = 0.0;
            t.alt_ft = info.elev_ft;
            t.agl_ft = 0.0;
            t.track_deg = Some(hdg);
            t.heading_deg = hdg;
        }
        if let Some(p) = f.plan.as_mut() {
            p.waypoints.insert(0, ("LEMG".into(), 36.675, -4.499, 0.0));
        }
        f.airport = AirportFacts {
            station: Some((icao.into(), d.ref_lat, d.ref_lon)),
            diagram: Some(DiagramRef {
                icao: d.icao.clone(),
                name: d.name.clone(),
                version: d.version.clone(),
                lat0: d.ref_lat,
                lon0: d.ref_lon,
                runways: runway_ends(&d),
            }),
            unavailable: false,
            route: Some(RouteFacts {
                source: "cleared",
                path: watch.path.clone(),
                seg: watch.seg.clone(),
                via: watch.via.clone(),
                holding_point: watch.holding_point.clone(),
                to_stand: !outbound,
            }),
            card,
            cleared_runway: outbound.then(|| rwy.to_string()),
            arrival_runway: None,
        };
        let s = build(&f);
        println!(
            "{icao}: {} via {}, card {:?}",
            s.airport.status,
            watch.via,
            s.airport.card.as_ref().map(|c| &c.text)
        );
        assert_eq!(s.airport.status, "ready");
        assert!(s.airport.route.is_some() && s.airport.own.is_some());
        std::fs::write(
            format!("{dir}/state_{icao}.json"),
            serde_json::to_string(&s).unwrap(),
        )
        .unwrap();
    }
}
