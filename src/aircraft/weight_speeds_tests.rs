use super::*;

fn zibo() -> AircraftProfile {
    load("zibo_738").expect("the Zibo profile loads")
}

fn row(w: &[f64], k: &[f64], kg: f64) -> Option<f64> {
    speed_by_weight(w, k, kg)
}

/// Straight between columns, held at both ends, nothing from a malformed
/// row or a weight that is not one.
#[test]
fn a_row_is_read_by_weight_and_held_at_its_ends() {
    let w = [40.0, 45.0, 50.0];
    let k = [108.0, 115.0, 122.0];
    assert_eq!(row(&w, &k, 45_000.0), Some(115.0));
    assert!((row(&w, &k, 47_500.0).unwrap() - 118.5).abs() < 1e-9);
    assert_eq!(row(&w, &k, 30_000.0), Some(108.0), "held at the light end");
    assert_eq!(row(&w, &k, 90_000.0), Some(122.0), "held at the heavy end");
    assert_eq!(row(&w, &k[..2], 45_000.0), None, "lengths differ");
    assert_eq!(
        row(&[40.0, 40.0, 50.0], &k, 45_000.0),
        None,
        "columns not rising"
    );
    assert_eq!(row(&w, &[108.0, f64::NAN, 122.0], 45_000.0), None);
    assert_eq!(row(&w, &k, f64::NAN), None);
    assert_eq!(row(&w, &k, 0.0), None);
    assert_eq!(row(&[], &[], 45_000.0), None);
}

/// sqrt(W / W_ref): a quarter of the weight halves the stall; nothing for
/// a missing or absurd weight, so the declared stalls fly.
#[test]
fn the_stall_scales_as_the_root_of_the_weight() {
    assert_eq!(
        weight_stall_scale(Some(64_000.0), Some(64_000.0)),
        Some(1.0)
    );
    assert!((weight_stall_scale(Some(64_000.0), Some(16_000.0)).unwrap() - 0.5).abs() < 1e-12);
    assert!((weight_stall_scale(Some(50_000.0), Some(72_000.0)).unwrap() - 1.2).abs() < 1e-12);
    assert_eq!(weight_stall_scale(None, Some(45_700.0)), None);
    assert_eq!(weight_stall_scale(Some(64_000.0), None), None);
    assert_eq!(weight_stall_scale(Some(64_000.0), Some(0.0)), None);
    assert_eq!(weight_stall_scale(Some(64_000.0), Some(f64::NAN)), None);
    assert_eq!(
        weight_stall_scale(Some(64_000.0), Some(1_000.0)),
        None,
        "not a weight"
    );
    assert_eq!(weight_stall_scale(Some(64_000.0), Some(900_000.0)), None);
}

/// LEMG 30, 2026-10-07, 45.7 t: the FCOM's Vref40 is 116 and the approach
/// 121, where the profile's fixed 145 flew the final flat and landed on
/// the nosewheel. Vref is the fullest flap's row, held at the card's ends.
#[test]
fn the_zibo_approach_is_vref40_plus_five_at_its_weight() {
    let p = zibo();
    let t = p.weight_speeds.as_ref().unwrap().vref.clone().unwrap();
    let vref = landing_vref_kts(&t, 45_700.0).unwrap();
    assert_eq!(vref.round(), 116.0);
    assert_eq!(approach_kts_at_weight(&t, 45_700.0), Some(121.0));
    // The card's own columns, and its ends.
    assert_eq!(approach_kts_at_weight(&t, 65_000.0), Some(146.0));
    assert_eq!(approach_kts_at_weight(&t, 30_000.0), Some(113.0));
    assert_eq!(approach_kts_at_weight(&t, 99_000.0), Some(165.0));
    // An additive declared is the additive flown.
    let mut t2 = t.clone();
    t2.additive_kts = Some(10.0);
    assert_eq!(approach_kts_at_weight(&t2, 45_700.0), Some(126.0));
    // A table whose rows do not match its flaps reads nothing.
    t2.flaps.pop();
    assert_eq!(approach_kts_at_weight(&t2, 45_700.0), None);
}

/// At 45.7 t the stalls ride the weight: Vso 114 at 64 t is 96, so the
/// landing floor is 116 under the approach's 121 (it was 137, over a Vref
/// of 116, and would have floored a correct approach back up). Clean, 141;
/// flaps 40, the landing floor. Without a weight, the declared floors.
#[test]
fn the_zibo_floors_follow_its_weight() {
    let p = zibo();
    let vs = p.vs_clean_kts.unwrap();
    let vs0 = p.reference.as_ref().and_then(|r| r.vs0_kts);
    assert_eq!(approach_floor_for(vs0, Some(vs)), Some(137.0), "declared");
    let s = profile_stall_scale(&p, Some(45_700.0)).unwrap();
    assert!((s - (45_700.0_f64 / 64_000.0).sqrt()).abs() < 1e-12);
    let landing = approach_floor_for(vs0.map(|v| v * s), Some(vs * s)).unwrap();
    assert_eq!(landing, 116.0);
    assert!(landing < 121.0, "the floor sits under the approach");
    assert_eq!(speed_floor_for(Some(vs * s)), Some(141.0));
    assert_eq!(profile_stall_scale(&p, None), None, "no weight, no scaling");
    // At its reference weight the floors are the declared ones.
    let one = profile_stall_scale(&p, Some(64_000.0)).unwrap();
    assert_eq!(
        approach_floor_for(vs0.map(|v| v * one), Some(vs * one)),
        Some(137.0)
    );
}

/// The takeoff card at 26K, flaps 5: at 45.7 t V1 110, Vr 111, V2 127
/// (the profile rotated at 145). Read only for the flap the takeoff sets.
#[test]
fn the_zibo_takeoff_speeds_come_off_its_card() {
    let p = zibo();
    let t = p.weight_speeds.as_ref().unwrap().takeoff.clone().unwrap();
    let flap = p.flaps.as_ref().and_then(|f| f.takeoff_ratio);
    assert_eq!(
        flap,
        Some(t.flap),
        "the card is for the flap the takeoff sets"
    );
    assert_eq!(
        takeoff_speeds_at_weight(&t, flap, 45_700.0),
        Some(TakeoffSpeeds {
            v1: 110.0,
            vr: 111.0,
            v2: 127.0
        })
    );
    assert_eq!(
        takeoff_speeds_at_weight(&t, flap, 70_000.0),
        Some(TakeoffSpeeds {
            v1: 141.0,
            vr: 143.0,
            v2: 152.0
        })
    );
    assert_eq!(
        takeoff_speeds_at_weight(&t, flap, 35_000.0),
        Some(TakeoffSpeeds {
            v1: 101.0,
            vr: 102.0,
            v2: 120.0
        }),
        "held at the light end"
    );
    assert_eq!(
        takeoff_speeds_at_weight(&t, Some(0.125), 45_700.0),
        None,
        "another flap"
    );
    assert_eq!(
        takeoff_speeds_at_weight(&t, None, 45_700.0),
        None,
        "the pilot's flap"
    );
    // A card out of order (V1 over Vr) is not flown.
    let mut bad = t.clone();
    bad.v1 = bad.vr.iter().map(|v| v + 5.0).collect();
    assert_eq!(takeoff_speeds_at_weight(&bad, flap, 45_700.0), None);
}

/// Only the 737-800s (the Zibo and the stock aircraft, one Boeing card)
/// declare speeds by weight: every other profile flies its fixed figures
/// and declared stalls, byte-identical.
#[test]
fn no_other_profile_weighs_its_speeds() {
    for entry in std::fs::read_dir("profiles").unwrap() {
        let path = entry.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) != Some("yaml") {
            continue;
        }
        let id = path.file_stem().unwrap().to_str().unwrap().to_string();
        let Some(p) = load(&id) else { continue };
        let declares = p.weight_speeds.is_some()
            || p.reference
                .as_ref()
                .is_some_and(|r| r.stall_ref_weight_kg.is_some());
        assert_eq!(declares, id == "zibo_738" || id == "boeing_737_800", "{id}");
    }
}
