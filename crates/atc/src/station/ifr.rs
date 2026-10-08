//! IFR clearance helpers: SID basis, publishing, destination and runway requests.

use super::*;

/// Why an IFR clearance names the SID it does (blackbox `ifr_clearance`
/// `sid_basis`, `sid_join`).
#[derive(Debug, Clone, PartialEq)]
pub(super) enum SidBasis {
    /// The SID off the runway that joins the filed route, at `join`.
    Route { join: String },
    /// A filed route no SID off the runway joins: runway heading.
    NoneJoins,
    /// No filed route: the exit nearest the destination's bearing.
    Bearing,
    /// No positioned exits: the first SID in the file for the runway.
    FirstInFile,
}

impl SidBasis {
    fn label(&self) -> &'static str {
        match self {
            SidBasis::Route { .. } => "route",
            SidBasis::NoneJoins => "none_joins",
            SidBasis::Bearing => "bearing",
            SidBasis::FirstInFile => "first_in_file",
        }
    }

    fn join(&self) -> Option<&str> {
        match self {
            SidBasis::Route { join } => Some(join),
            _ => None,
        }
    }
}

/// Publish an IFR clearance whose readback the tower accepted, for the
/// crew (`party_line::assigned_clearance`), and blackbox it item by item.
pub(super) fn publish_ifr_clearance(
    c: tower::IfrClearance,
    callsign: &str,
    bearings: Option<(f64, f64)>,
    basis: Option<&SidBasis>,
) {
    tracing::info!(dest = %c.dest, sid = ?c.sid, runway = %c.runway, alt = c.initial_alt_ft,
                   squawk = c.squawk, "IFR clearance read back and accepted");
    sim::blackbox::record(
        "atc",
        serde_json::json!({
            "event": "ifr_clearance", "side": "tower", "callsign": callsign,
            "dest": c.dest, "sid": c.sid, "runway": c.runway,
            "initial_alt_ft": c.initial_alt_ft, "squawk": c.squawk,
            "sid_exit_brg": bearings.map(|b| b.0.round()),
            "dest_brg": bearings.map(|b| b.1.round()),
            "sid_basis": basis.map(SidBasis::label),
            "sid_join": basis.and_then(SidBasis::join),
        }),
    );
    *OWN_STRIP.ifr_flight.lock().unwrap_or_else(|e| e.into_inner()) = Some(IfrFlight {
        code: c.squawk,
        callsign: callsign.to_string(),
        airborne: false,
    });
    *OWN_STRIP.ifr_clearance.lock().unwrap_or_else(|e| e.into_inner()) = Some(c);
}

/// An IFR destination as heard, resolved against the region index: a code
/// first (three letters also with the K a US pilot drops), then the name
/// words, every one a whole word of the field's name. Several fields by one
/// name: the one whose radio name IS the words, else none (a guess would
/// clear the flight somewhere it never filed). `(icao, apt.dat name)`;
/// `None` = unknown to the index (or the index not loaded).
pub(crate) fn resolve_destination(dest: &str) -> Option<(String, String, (f64, f64))> {
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    let dest = dest.trim();
    let single = !dest.contains(' ')
        && (3..=4).contains(&dest.len())
        && dest.chars().all(|c| c.is_ascii_alphanumeric());
    if single {
        let up = dest.to_ascii_uppercase();
        let mut codes = vec![up.clone()];
        if up.len() == 3 {
            codes.push(format!("K{up}"));
        }
        for code in &codes {
            if let Some(a) = region.iter().find(|a| a.icao.eq_ignore_ascii_case(code)) {
                return Some((a.icao.clone(), a.apt.name.clone(), (a.lat, a.lon)));
            }
        }
        // Beyond the region index: any land field by its code.
        let dir = FIELD_DIR.lock().unwrap_or_else(|e| e.into_inner());
        for code in &codes {
            if let Some((icao, name, pos)) = dir.iter().find(|(i, _, _)| i.eq_ignore_ascii_case(code)) {
                return Some((icao.clone(), name.clone(), *pos));
            }
        }
    }
    let words_of = |s: &str| -> Vec<String> {
        s.to_lowercase()
            .split(|c: char| !c.is_alphanumeric())
            .filter(|w| !w.is_empty())
            .map(str::to_string)
            .collect()
    };
    let query = words_of(dest);
    if query.is_empty() {
        return None;
    }
    let matches: Vec<&RegionApt> = region
        .iter()
        .filter(|a| {
            let name = words_of(&a.apt.name);
            query.iter().all(|q| name.contains(q))
        })
        .collect();
    let pick = match matches.as_slice() {
        [one] => Some(*one),
        [] => None,
        many => {
            let exact: Vec<&&RegionApt> = many
                .iter()
                .filter(|a| words_of(spoken_airport(&a.apt.name)) == query)
                .collect();
            match exact.as_slice() {
                [one] => Some(**one),
                _ => None,
            }
        }
    }?;
    Some((pick.icao.clone(), pick.apt.name.clone(), (pick.lat, pick.lon)))
}

/// A requested runway resolved against the field: `Ok(ident)` for a strip
/// end the tower hands out (`nav::assignable_runways`), else the spoken
/// (runway, reason) the refusal says. No runway in the words is refused
/// with an empty runway, which the tower answers with "say again": a
/// mishear never buys a runway.
pub(super) fn resolve_runway_request(
    runways: &[nav::Runway],
    heard: Option<&str>,
) -> Result<String, (String, String)> {
    let Some(canon) = heard else {
        return Err((String::new(), String::new()));
    };
    let spoken_canon = || {
        let digits: String = canon.chars().take_while(|c| c.is_ascii_digit()).collect();
        let padded = if digits.len() == 1 {
            format!("0{canon}")
        } else {
            canon.to_string()
        };
        atis::spoken_runway(&padded)
    };
    let Some(ident) = runways
        .iter()
        .flat_map(|r| [&r.ident_a, &r.ident_b])
        .find(|id| tower::runway_matches(id, canon))
        .cloned()
    else {
        return Err((spoken_canon(), "no such runway here".to_string()));
    };
    let assignable = nav::assignable_runways(runways)
        .any(|r| r.ident_a == ident || r.ident_b == ident);
    if !assignable {
        return Err((
            atis::spoken_runway(&ident),
            "that runway is not available".to_string(),
        ));
    }
    Ok(ident)
}
