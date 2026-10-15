//! An unknown type read from its `.acf`, the profile's references and their
//! probe, and the built-in lookup.

use super::*;

/// A profile for an airframe nobody has written one for, built from the
/// placards in its own `.acf` file: the speeds, the flap detents and their
/// limits, whether the gear retracts, whether the engine is carburetted. The
/// tool surface is the stock 172's (stock datarefs are the best guess for a
/// stock-systems airframe); only the NUMBERS change, and those are the ones
/// that hurt. The RV-10 flew on the 172's 65 kt approach speed — its own clean
/// stall speed — because the fallback was silent and total (2026-09-11).
///
/// `acf_text` is the file's contents (`P key value` lines). Pure, so it
/// battle-tests on a fixture. Approach speed is the larger of 1.3 Vso and
/// 1.2 Vs: the acf's Vso is what the author typed, and the model can stall
/// above it with partial flap (the RV-10 did, at 63 kt against a Vso of 50).
pub fn generic_from_acf(icao: &str, acf_text: &str) -> AircraftProfile {
    let vals: std::collections::HashMap<&str, &str> = acf_text
        .lines()
        .filter_map(|l| {
            let mut it = l.split_whitespace();
            (it.next()? == "P").then(|| (it.next(), it.next()))?.into()
        })
        .filter_map(|(k, v)| Some((k?, v?)))
        .collect();
    let num = |k: &str| vals.get(k).and_then(|v| v.parse::<f64>().ok());
    let kts = |k: &str| num(k).filter(|v| *v > 0.0 && *v < 9000.0);
    let mut p = cessna_172sp();
    p.id = format!("generic_{}", icao.to_lowercase());
    // Built on the 172's bones, but never detected as one.
    p.matcher = None;
    p.pressurised = None; // Unknown systems must use neutral cabin-altitude wording.
    let name = vals
        .get("acf/_name")
        .map(|s| s.to_string())
        .unwrap_or_else(|| icao.to_string());
    // `_name` is one token here (split on whitespace); good enough for a label.
    p.display_name = format!("{name} (no profile)");
    // The surveyed autopilot decides the coupled ILS (`ils::coupled_allowed`).
    p.ils_coupled = None;
    // Its checklist is found by id (a generic one has none); the 172's
    // switch reads are not this aircraft's either.
    p.checklist_states = Default::default();
    p.bus_volts_nominal = None;

    let vs = kts("acf/_Vs_kts");
    let vso = kts("acf/_Vso_kts");
    p.vs_clean_kts = vs.or(p.vs_clean_kts);
    p.vne_kts = kts("acf/_Vne_kts").or(p.vne_kts);
    let vle = kts("acf/_Vle_kts");
    p.retractable_gear = vle.is_some();
    p.vlo_kts = vle;
    p.gear_warning = gear_warning_from_acf(acf_text);
    let injected = vals
        .get("_engn/0/_type")
        .is_some_and(|t| t.contains("INJ") || !t.starts_with("RCP"));
    p.carb_heat = !injected;
    if vals
        .get("_engn/0/_type")
        .is_some_and(|t| !t.starts_with("RCP"))
    {
        // Not a piston: the magneto and mixture-side tools would be lies.
        p.magnetos = false;
        p.fuel_pump = false;
    }

    // Flap detents: `_flap_detents` notches at `_flap1_dn/i` degrees; ratio
    // is degrees over the last notch; Vfe1 for the first, Vfem for the last,
    // a straight line between.
    let detents = num("acf/_flap_detents").map(|d| d as usize).unwrap_or(0);
    let degs: Vec<f64> = (1..=detents)
        .filter_map(|i| num(&format!("acf/_flap1_dn/{i}")))
        .filter(|d| *d > 0.0)
        .collect();
    if let (Some(last), true) = (degs.last().copied(), !degs.is_empty()) {
        let vfe1 = kts("acf/_Vfe1_kts");
        let vfem = kts("acf/_Vfem_kts").or(vfe1);
        let n = degs.len();
        let mut list = vec![FlapDetent {
            label: "up".into(),
            spoken: "up".into(),
            ratio: 0.0,
            vfe_kts: None,
            // A generic airframe declares no manoeuvring speeds, so it keeps
            // the one-step clean-up on `initial_climb_kts`.
            retract_above_kts: None,
            vs_kts: None,
            idle_decel_kts_s: None,
        }];
        // A slatted wing's per-detent stall, from the same file; only when
        // every notch maps one to one (a slats-only notch at 0 degrees is
        // dropped from `degs`, and the stall line is then kept).
        let acf_stall = detent_stall_from_acf(acf_text).filter(|v| v.len() == n);
        for (i, d) in degs.iter().enumerate() {
            let full = i + 1 == n;
            let vfe = match (vfe1, vfem) {
                (Some(a), Some(b)) if n > 1 => Some(a + (b - a) * i as f64 / (n - 1) as f64),
                (a, b) => b.or(a),
            };
            list.push(FlapDetent {
                label: if full {
                    "full".into()
                } else {
                    format!("{d:.0}")
                },
                spoken: if full {
                    "full".into()
                } else {
                    format!("{d:.0} degrees")
                },
                ratio: (d / last * 10_000.0).round() / 10_000.0,
                vfe_kts: vfe.map(f64::round),
                retract_above_kts: None,
                vs_kts: acf_stall.as_ref().and_then(|v| v.get(i).copied()),
                idle_decel_kts_s: None,
            });
        }
        p.flaps = Some(FlapProfile {
            detents: list,
            takeoff_ratio: Some(0.0),
            by_command: None,
            speedbrake_max_ratio: None,
        });
    }

    let r = p.reference.get_or_insert_with(Default::default);
    r.vno_kts = kts("acf/_Vno_kts").or(r.vno_kts);
    r.va_kts = kts("acf/_Va_kts").or(r.va_kts);
    r.vx_kts = kts("acf/_Vx_kts").or(r.vx_kts);
    r.vy_kts = kts("acf/_Vy_kts").or(r.vy_kts);
    r.best_glide_kts = kts("acf/_Vbg_kts").or(r.best_glide_kts);
    r.vs0_kts = vso.or(r.vs0_kts);
    if let Some(vs) = vs {
        r.vr_kts = Some(fallback_vr_kts(vs));
        r.approach_kts = Some(fallback_approach_kts(vso, vs));
    }
    if let Some(vno) = r.vno_kts {
        r.cruise_ias_kts = Some((vno * 0.85).round());
    }
    r.cruise_kts = None;
    r.best_climb_fpm = None;
    r.max_gross_lb = num("acf/_m_max").or(r.max_gross_lb);
    r.empty_weight_lb = num("acf/_m_empty").or(r.empty_weight_lb);
    r.useful_load_lb = match (r.max_gross_lb, r.empty_weight_lb) {
        (Some(g), Some(e)) if g > e => Some(g - e),
        _ => None,
    };
    for f in [
        &mut r.service_ceiling_ft,
        &mut r.fuel_capacity_gal,
        &mut r.fuel_useable_gal,
        &mut r.endurance_hours,
        &mut r.takeoff_distance_ft,
        &mut r.landing_distance_ft,
        &mut r.landing_roll_ft,
    ] {
        *f = None; // not in the acf: not recited
    }
    p
}

/// Every dataref and command name a profile relies on, as strings, for the
/// probe that asks the sim whether each one exists. A walk over the profile's
/// own serialisation rather than a hand list of fields: a new field with a
/// dataref in it is probed the day it is added, not the day someone remembers.
/// Names are `a/b/...` paths without whitespace; keys that carry prose or
/// file paths are skipped, an `[index]` suffix is dropped.
pub fn profile_references(p: &AircraftProfile) -> Vec<String> {
    const SKIP_KEYS: [&str; 11] = [
        "match",
        "say",
        "spoken",
        "label",
        "key",
        "kind",
        "id",
        "display_name",
        "voice",
        "phases",
        "positions_spoken",
    ];
    fn walk(v: &serde_yaml::Value, key: &str, out: &mut Vec<String>) {
        // Skipped whole, not only as a string: `match:` is a mapping, and
        // its `acf` path was probed as a name ("B737-800X/", "Sparky/
        // 747-400/": no whitespace, so it passed the name test, 2026-10-06).
        if SKIP_KEYS.contains(&key) {
            return;
        }
        match v {
            serde_yaml::Value::String(s) => {
                let name = s.split('[').next().unwrap_or(s).trim();
                let last = name.rsplit('/').next().unwrap_or("");
                if name.contains('/')
                    && !name.contains(char::is_whitespace)
                    && !last.contains('.')
                    && !name.starts_with('/')
                    && !out.iter().any(|o| o == name)
                {
                    out.push(name.to_string());
                }
            }
            serde_yaml::Value::Sequence(items) => items.iter().for_each(|i| walk(i, key, out)),
            serde_yaml::Value::Mapping(m) => {
                for (k, v) in m {
                    let k = k.as_str().unwrap_or("");
                    walk(v, k, out);
                }
            }
            _ => {}
        }
    }
    let mut out = Vec::new();
    if let Ok(v) = serde_yaml::to_value(p) {
        walk(&v, "", &mut out);
    }
    // A declared panel is flown through the owner's stock names too.
    if p.autoflight.is_some() {
        for name in AutoflightProfile::STOCK {
            if !out.iter().any(|o| o == name) {
                out.push(name.to_string());
            }
        }
    }
    // The takeoff configuration's declarations carry no names either: the
    // stock ones behind each ([`TakeoffConfigProfile::references`]).
    if let Some(tc) = &p.takeoff_config {
        for name in tc.references() {
            if !out.iter().any(|o| o == name) {
                out.push(name.to_string());
            }
        }
    }
    // Nor does the yoke input: the plugin writes them.
    if let Some(y) = p.yoke_input {
        for name in y.references() {
            if !out.iter().any(|o| o == name) {
                out.push(name.to_string());
            }
        }
    }
    // The tiller's method names no dataref in the YAML: the plugin's.
    if let Some(t) = p.taxi.as_ref().and_then(|t| t.tiller.as_ref()) {
        for name in t.method.references() {
            if !out.iter().any(|o| o == name) {
                out.push(name.to_string());
            }
        }
    }
    // Fields that carry no name of their own, only indices into stock
    // arrays the tool reads and writes: the names they rely on.
    if !p.fuel_tank_pumps_off_when_empty.is_empty() {
        for name in [
            crate::datarefs::FUEL_TANK_PUMP_ON,
            crate::datarefs::FUEL_PER_TANK_KG,
            crate::datarefs::FUEL_TANK_LATERAL_M,
        ] {
            if !out.iter().any(|o| o == name) {
                out.push(name.to_string());
            }
        }
    }
    out
}

/// Which of a profile's names the sim does not have: neither a dataref nor
/// a command. Pure over the two indexes, so the probe's verdict tests
/// without a sim.
pub fn missing_references(
    names: &[String],
    datarefs: &std::collections::HashSet<String>,
    commands: &std::collections::HashSet<String>,
) -> Vec<String> {
    names
        .iter()
        .filter(|n| !datarefs.contains(*n) && !commands.contains(*n))
        .cloned()
        .collect()
}

/// The spoken result of a probe: nothing when every name exists, otherwise
/// the count and the first few, so the pilot knows which tools will fail
/// before asking for them.
pub fn probe_summary(missing: &[String], total: usize) -> Option<String> {
    if missing.is_empty() {
        return None;
    }
    let shown: Vec<String> = missing
        .iter()
        .take(3)
        .map(|m| m.rsplit('/').next().unwrap_or(m).replace('_', " "))
        .collect();
    let more = if missing.len() > 3 {
        format!(" and {} more", missing.len() - 3)
    } else {
        String::new()
    };
    Some(format!(
        "Profile check: {} of {total} names this profile relies on are not in this aircraft: {}{more}. The tools that use them will fail.",
        missing.len(),
        shown.join(", ")
    ))
}

/// Built-in profile by id (fallback when no YAML file exists).
pub fn builtin(id: &str) -> Option<AircraftProfile> {
    match id {
        "cessna_172sp" => Some(cessna_172sp()),
        "cirrus_sr22" => Some(cirrus_sr22()),
        _ => None,
    }
}
