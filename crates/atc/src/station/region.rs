//! The enroute region: field directory, region switches, reach, arrival pin.

use super::*;

/// The enroute region: every towered field from apt.dat, indexed once on a
/// background thread at startup, so the flight can outgrow its departure
/// airport. When another tower becomes decisively nearer, the current
/// controller hands the flight off by name and frequency, and the station
/// rebuilds itself around the new field — Gibraltar stops following you to
/// Malaga.
pub struct RegionApt {
    pub icao: String,
    pub apt: nav::AirportInfo,
    pub(super) lat: f64,
    pub(super) lon: f64,
}

pub(super) static REGION: Mutex<Vec<RegionApt>> = Mutex::new(Vec::new());

/// Every land field in apt.dat, towered or not, as (ICAO, name, position):
/// the IFR clearance's destination directory beyond the region index
/// (issue #96). An IFR flight files to any field with a runway, and the
/// region index holds only staffed towers, so "IFR clearance to" a
/// tower-less field (or one whose tower row is a Gateway placeholder) was
/// "say again destination". Built by the same scan as [`REGION`]; codes
/// only (a name across forty thousand fields is a guess).
pub(super) type FieldDirectoryEntry = (String, String, (f64, f64));
pub(super) static FIELD_DIR: Mutex<Vec<FieldDirectoryEntry>> = Mutex::new(Vec::new());

/// The active station's identity + position, for the switch decision. Set by
/// [`Station::from_airport`].
pub(super) static ACTIVE_FIELD: Mutex<Option<(String, f64, f64)>> = Mutex::new(None);

/// The installed station's ICAO, as its owner told it
/// ([`Station::set_procedures`]): what our own traffic loads its field by.
/// `None` with no station, or one whose owner never named it.
pub(super) static ACTIVE_ICAO: Mutex<Option<String>> = Mutex::new(None);

/// The installed field's phraseology, for the ticks that speak without a
/// `Station` in hand (the go-around order, the region handoff): its
/// dialect by prefix ([`dialect_for`]) and whether it speaks CAP 413
/// ([`speaks_cap413`]). No field installed is ICAO, not UK.
pub(super) fn installed_phraseology() -> (Dialect, bool) {
    match installed_icao() {
        Some(icao) => (dialect_for(&icao), speaks_cap413(&icao)),
        None => (Dialect::Icao, false),
    }
}

/// The field speaks CAP 413's own words inside the ICAO dialect ("circuit
/// height", "avoiding action"): the UK (EG), and Gibraltar (LXGB), whose
/// tower is the RAF's.
pub(crate) fn speaks_cap413(icao: &str) -> bool {
    atis::is_uk_field(icao) || icao.eq_ignore_ascii_case("LXGB")
}

/// The installed station's ICAO ([`ACTIVE_ICAO`]); unlike `active_icao`
/// it needs no region index.
pub fn installed_icao() -> Option<String> {
    ACTIVE_ICAO
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
}

pub(super) static LAST_REGION_SWITCH: Mutex<Option<Instant>> = Mutex::new(None);

/// The new field must be this much nearer than the active one before a
/// handoff — hysteresis, so the midpoint between two fields can't flap.
pub(super) const SWITCH_MARGIN_NM: f64 = 5.0;
/// The ground relocation's margin: the field whose pavement is nearer, by a
/// fraction of a mile, is the field you are at.
pub(super) const GROUND_MARGIN_NM: f64 = 0.2;
/// No handoff to a field further than this — enroute over nowhere keeps the
/// last controller (a lightweight "radar" continuity).
pub(super) const SWITCH_MAX_NM: f64 = 60.0;
/// Minimum quiet time between handoffs.
pub(super) const SWITCH_COOLDOWN_SECS: f64 = 180.0;

/// Index every towered field once (a full apt.dat pass, seconds — run it on a
/// background thread). Without it [`region_tick`] never fires and the station
/// stays single-airport, exactly the pre-region behavior.
/// Whether the enroute region index has been loaded — the owner checks this
/// before spawning another full apt.dat scan (a live mode re-enable must not
/// pay the boot cost twice).
pub fn region_loaded() -> bool {
    !REGION.lock().unwrap_or_else(|e| e.into_inner()).is_empty()
}

pub fn load_region(apt_path: &str) {
    let mut v: Vec<RegionApt> = Vec::new();
    let mut dir: Vec<(String, String, (f64, f64))> = Vec::new();
    let res = nav::scan_airports(apt_path, |icao, apt| {
        if let Some(pos) = apt.position() {
            dir.push((icao.to_string(), apt.name.clone(), pos));
        }
        // Staffed towers only: a Gateway "unnamed entity" TWR row (an
        // ultralight strip's air-to-air frequency, mistyped) must not become
        // a handoff target or "the nearest station" (LEMY, 2026-07-10).
        if apt.staffed_tower().is_some() {
            if let Some((lat, lon)) = apt.position() {
                v.push(RegionApt {
                    icao: icao.to_string(),
                    apt,
                    lat,
                    lon,
                });
            }
        }
    });
    match res {
        Ok(()) => {
            tracing::info!(towered_fields = v.len(), fields = dir.len(), "enroute region indexed");
            *REGION.lock().unwrap_or_else(|e| e.into_inner()) = v;
            *FIELD_DIR.lock().unwrap_or_else(|e| e.into_inner()) = dir;
        }
        Err(e) => tracing::warn!("region index failed ({e:#}); staying single-airport"),
    }
}

/// A due station switch: what to say and who says it. Airborne it's a real
/// controller handoff; on the ground it's the CREW noting the local tower
/// (you didn't fly here under radar — you loaded the flight at a new field,
/// and "radar services terminated" from a parked aircraft would be theatre).
pub struct RegionSwitch {
    pub line: String,
    /// True = the copilot's voice (ground relocation); false = the controller.
    pub crew: bool,
}

/// You're AT a field (parked/taxiing) when it's within this — the ground
/// relocation case ("I started at LERL and Gibraltar answered", 2026-07-09).
pub(super) const AT_FIELD_NM: f64 = 4.0;

/// One tick of the enroute watch: when another towered field is decisively
/// nearer than the active one, arm the station swap and return what to speak.
/// Airborne: the handoff ("radar services terminated, contact Malaga Tower on
/// ..."). On the ground: a silent relocation with a crew note, but only when
/// physically AT the new field. The main loop consumes [`PENDING_STATION`] on
/// its next pass; a cooldown keeps back-to-back fields from ping-ponging.
/// How much STATION a field is, for the enroute handoff choice: its distinct
/// published services beyond the tower itself. A primary airport scores 3-5;
/// a strip whose apt.dat rows just point at the primary's controllers scores
/// 0-1, so it never wins the sky it doesn't control.
pub(super) fn region_services(apt: &nav::AirportInfo) -> usize {
    [
        nav::FREQ_ATIS,
        nav::FREQ_CLEARANCE,
        nav::FREQ_GROUND,
        nav::FREQ_APPROACH,
        nav::FREQ_DEPARTURE,
    ]
    .iter()
    .filter(|k| apt.freq(**k).is_some())
    .count()
}

/// Nautical miles of pull each published service adds to a field in the
/// airborne handoff choice (see [`region_services`]).
pub(super) const SERVICE_BIAS_NM: f64 = 3.0;

pub fn region_tick(lat: f64, lon: f64, on_ground: bool, now: Instant) -> Option<RegionSwitch> {
    if uncontrolled_arrival() {
        return None;
    }
    if PENDING_STATION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some()
    {
        return None; // a switch is already waiting to be consumed
    }
    if LAST_REGION_SWITCH
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some_and(|t| now.duration_since(t).as_secs_f64() < SWITCH_COOLDOWN_SECS)
    {
        return None;
    }
    let (active_name, a_lat, a_lon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    // Airborne, "nearest" is service-weighted: a real airport (ATIS, ground,
    // its own approach) outranks a strip a few miles nearer that merely
    // BORROWED the big field's frequencies — Trebujena's apt.dat rows name
    // "Jerez Tower"/"Seville Approach", and picking it by raw distance had
    // approach faithfully vectoring the Jerez arrival onto Trebujena's 800 m
    // grass runway two six (flown 2026-07-11). On the ground, raw nearest by
    // FOOTPRINT (the nearest runway centreline, not the first threshold): a
    // relocation goes to the field whose pavement you are physically on,
    // however small. Measured from the first runway row, Anchorage's GA ramp
    // was "nearer" Lake Hood Strip than Anchorage itself, and the whole
    // sortie (2026-09-10) taxied 1.4 km across Anchorage's aprons on a
    // beeline to Lake Hood's network before the drawn route began.
    let distance = |a: &RegionApt| -> f64 {
        if on_ground {
            a.apt
                .ground_distance_nm(lat, lon)
                .unwrap_or_else(|| crate::nav::haversine_nm(lat, lon, a.lat, a.lon))
        } else {
            crate::nav::haversine_nm(lat, lon, a.lat, a.lon)
        }
    };
    let score = |d: f64, apt: &nav::AirportInfo| {
        if on_ground {
            d
        } else {
            d - SERVICE_BIAS_NM * region_services(apt) as f64
        }
    };
    // An IFR flight in the air is worked by radar units, never handed to a
    // tower that only owns its zone: EGBB-EGKK (2026-09-27) was told
    // "contact Oxford Kidlington Tower" at FL150. It goes to a field's
    // approach (or departure) position, and a tower-only field is passed
    // over, save the active one and the pinned arrival (the flight is
    // landing there; its tower is where the inbound call goes).
    let ifr = OWN_STRIP.ifr_flight.lock().unwrap_or_else(|e| e.into_inner()).clone();
    let ifr_airborne = !on_ground && ifr.is_some();
    let pin = arrival_pin_khz();
    let eligible = |a: &RegionApt| {
        !ifr_airborne
            || a.apt.name == active_name
            || radar_position(&a.apt).is_some()
            || a.apt.freq(nav::FREQ_TOWER).is_some_and(|f| Some(f.khz) == pin)
    };
    let (d_best, s_best, best) = region
        .iter()
        .filter(|a| eligible(a))
        .map(|a| {
            let d = distance(a);
            (d, score(d, &a.apt), a)
        })
        .min_by(|x, y| x.1.total_cmp(&y.1))?;
    if best.apt.name == active_name {
        return None;
    }
    // Ground: only a relocation TO a field you're actually at. Airborne: the
    // enroute handoff, bounded so cruising over nowhere keeps the last
    // controller.
    if (on_ground && d_best > AT_FIELD_NM) || (!on_ground && d_best > SWITCH_MAX_NM) {
        return None;
    }
    // Hysteresis on the same scale the pick was made on. Adjacent fields
    // (Anchorage and Lake Hood share a fence) are decided by a fraction of a
    // mile on the ground, so the enroute margin would pin the pilot to
    // whichever answered first; the cooldown and a parked aircraft's fixed
    // geometry keep it from ping-ponging.
    let s_active = region
        .iter()
        .find(|a| a.apt.name == active_name)
        .map(|a| score(distance(a), &a.apt))
        .unwrap_or_else(|| crate::nav::haversine_nm(lat, lon, a_lat, a_lon));
    let margin = if on_ground {
        GROUND_MARGIN_NM
    } else {
        SWITCH_MARGIN_NM
    };
    if s_best + margin > s_active {
        return None;
    }
    // A committed arrival owns the station (see [`pin_arrival_station`]): the
    // only swap the region may still make is the one TO the pinned field.
    // On the GROUND the pavement under the wheels outranks the pin — we
    // landed somewhere else, or the arrival is over and nobody released it —
    // so the relocation is taken and the pin dropped, never stranding the
    // station at a field we have left.
    if let Some(pin) = arrival_pin_khz() {
        if !best.apt.freq(nav::FREQ_TOWER).is_some_and(|f| f.khz == pin) {
            if !on_ground {
                return None;
            }
            *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
    }
    let khz = best.apt.freq(nav::FREQ_TOWER)?.khz;
    // Whether ANY controller of the active station has actually worked this
    // flight in this session: two-way contact publishes a clearance state,
    // a radar service arms the squawk watch. Without either there is no
    // service to hand off — a fresh mid-flight boot must not open with a
    // controller terminating a service it never provided (heard live
    // 2026-07-11: app start near Sevilla answered by "radar services
    // terminated" from a departure position never called).
    // An IFR flight in the air is always some unit's: `Station::install`
    // clears the clearance and the squawk watch, but never the IFR flight,
    // so without it every switch after the first airborne handoff fell to
    // the crew's note and the cruise went silent (LEMG-EGKK 2026-10-14:
    // Granada Departure the last word, five switches to Bilbao unspoken).
    let had_contact = ifr_airborne
        || OWN_STRIP.clearance
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some()
        || OWN_STRIP.squawk_watch
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_some();
    let sw = if on_ground {
        RegionSwitch {
            line: format!(
                "We're at {}, tower here is on {}.",
                spoken_airport(&best.apt.name),
                atis::spoken_mhz(khz)
            ),
            crew: true,
        }
    } else if !had_contact {
        // First fix of the session: the copilot orients, no radar theatre.
        // Also every switch AFTER a VFR flight's airborne handoff: its
        // service ended there, and `Station::install` cleared the clearance
        // and the squawk watch. The crew speaks the note only when it is
        // worth volunteering (`monitor::region_note_said`).
        RegionSwitch {
            line: format!(
                "Closest field is {}, tower is on {}.",
                spoken_airport(&best.apt.name),
                atis::spoken_mhz(khz)
            ),
            crew: true,
        }
    } else {
        // An IFR flight is handed unit to unit with its code: no service
        // ends and no squawk is ordered (LIEA-LEBL, 2026-09-27, an IFR 737
        // on 4501 was told "squawk seven thousand" at FL200).
        // A discrete code in effect goes back to conspicuity with the service
        // that assigned it.
        let squawk_vfr = ifr.is_none()
            && OWN_STRIP.squawk_watch
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .is_some();
        // Enroute, the IFR flight goes to the field's radar position (see
        // `eligible` above); the pinned arrival keeps its tower, where the
        // crew's inbound call goes.
        let pinned = best.apt.freq(nav::FREQ_TOWER).is_some_and(|f| Some(f.khz) == pin);
        let (position, khz) = match radar_position(&best.apt) {
            Some(r) if ifr_airborne && !pinned => r,
            _ => ("Tower", khz),
        };
        let to = format!("{} {position}", spoken_airport(&best.apt.name));
        // The instruction as data for the crew that has to obey it: the
        // spoken line alone had no consumer, so the flight kept the discrete
        // code and the old frequency until its own arrival stage retuned.
        *REGION_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = Some(RegionHandoff {
            to: to.clone(),
            khz,
            squawk_vfr,
            ifr_code: ifr.as_ref().map(|f| f.code),
        });
        RegionSwitch {
            line: region_handoff_line(ifr.as_ref().map(|f| f.callsign.as_str()), squawk_vfr, &to, khz, installed_phraseology().0),
            crew: false,
        }
    };
    tracing::info!(from = %active_name, to = %best.apt.name, ground = on_ground, "region switch");
    *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) =
        Some((best.icao.clone(), best.apt.clone()));
    *LAST_REGION_SWITCH.lock().unwrap_or_else(|e| e.into_inner()) = Some(now);
    Some(sw)
}

/// A field's radar position for an IFR handoff: its approach, else its
/// departure frequency, as spoken ("Approach") and in kHz. `None` = the field
/// has only a tower.
pub(super) fn radar_position(apt: &nav::AirportInfo) -> Option<(&'static str, i64)> {
    apt.freq(nav::FREQ_APPROACH)
        .map(|f| ("Approach", f.khz))
        .or_else(|| apt.freq(nav::FREQ_DEPARTURE).map(|f| ("Departure", f.khz)))
}

/// The enroute handoff as spoken. VFR: the radar service ends ("Radar
/// services terminated", seven thousand when a code was assigned). IFR
/// (`ifr_callsign`): a plain handoff to the next unit, the code kept, the
/// same "contact X on Y" the station's other handoffs use in both dialects.
pub(crate) fn region_handoff_line(
    ifr_callsign: Option<&str>,
    squawk_vfr: bool,
    to: &str,
    khz: i64,
    dialect: Dialect,
) -> String {
    let mhz = atis::spoken_mhz_in(khz, dialect);
    match ifr_callsign {
        Some(cs) if !cs.is_empty() => format!("{cs}, contact {to} on {mhz}."),
        Some(_) => format!("Contact {to} on {mhz}."),
        None => {
            let squawk = if squawk_vfr {
                " Squawk seven thousand."
            } else {
                ""
            };
            format!("Radar services terminated.{squawk} Contact {to} on {mhz}.")
        }
    }
}

/// The active station's controller frequencies (kHz → speaking position),
/// published by [`Station::install`] for the monitor's traffic advisories.
pub(super) static CONTROLLER_FREQS: Mutex<Vec<(i64, StationSpeaker)>> = Mutex::new(Vec::new());

/// The active station's positions by the names they key ("Gatwick
/// Ground"), published with [`CONTROLLER_FREQS`] for the ticks that speak
/// as one (the ground's handoff at the hold).
pub(super) static POSITION_NAMES: Mutex<Vec<(StationSpeaker, String)>> = Mutex::new(Vec::new());

/// The installed station's position `who`: its name as keyed and its
/// frequency (kHz). `None` when the field has no such position.
pub(crate) fn active_position(who: StationSpeaker) -> Option<(String, i64)> {
    let khz = CONTROLLER_FREQS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|(_, s)| *s == who)
        .map(|(k, _)| *k)?;
    let name = POSITION_NAMES
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|(s, _)| *s == who)
        .map(|(_, n)| n.clone())?;
    Some((name, khz))
}

/// The active tower's frequency (kHz), for the crew side — the full-auto
/// orchestrator tunes COM1 to it before requesting taxi. `None` when no station
/// is installed.
pub(crate) fn active_tower_khz() -> Option<i64> {
    CONTROLLER_FREQS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|(_, s)| matches!(s, StationSpeaker::Tower))
        .map(|(k, _)| *k)
}

/// The active station's APPROACH frequency, when its field publishes one
/// (apt.dat 1055): where an instrument arrival checks in before the tower.
pub(crate) fn active_approach_khz() -> Option<i64> {
    CONTROLLER_FREQS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|(_, s)| matches!(s, StationSpeaker::Approach))
        .map(|(k, _)| *k)
}

/// The active station's panel identity — name, field elevation, published
/// services — for the radio row and the ATC tab. Published by
/// [`Station::install`], like the rest of the side state.
pub(super) type ActivePanel = (String, f64, Vec<(String, i64)>);
pub(super) static ACTIVE_PANEL: Mutex<Option<ActivePanel>> = Mutex::new(None);

/// The installed station's phraseology dialect, for the lines spoken
/// outside a `Station` method (the coach's suggested calls, the off-route
/// correction): an FAA field's are FAA (#263). Published by
/// [`Station::install`] with the panel.
pub(super) static ACTIVE_DIALECT: Mutex<Dialect> = Mutex::new(Dialect::Icao);

/// The installed station's dialect ([`ACTIVE_DIALECT`]).
pub(super) fn active_dialect() -> Dialect {
    *ACTIVE_DIALECT.lock().unwrap_or_else(|e| e.into_inner())
}

/// A field's position by its code, from the field directory (any land
/// field) or the region index: the crew's EFB map draws the plan's origin,
/// destination and alternate by it. `None` until the index has loaded.
pub fn field_position(icao: &str) -> Option<(f64, f64)> {
    let code = icao.trim();
    if code.is_empty() {
        return None;
    }
    if let Some(p) = FIELD_DIR
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|(i, _, _)| i.eq_ignore_ascii_case(code))
        .map(|(_, _, p)| *p)
    {
        return Some(p);
    }
    REGION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|a| a.icao.eq_ignore_ascii_case(code))
        .map(|a| (a.lat, a.lon))
}

/// A field on a map: code, position, staffed tower.
#[derive(Clone, Debug, PartialEq)]
pub struct FieldDot {
    pub icao: String,
    pub lat: f64,
    pub lon: f64,
    pub towered: bool,
}

/// The fields inside a lat/lon box, for the crew's EFB map: the towered
/// ones (the region index) and, with `all`, every land field in the
/// directory, at most `limit`, towered first. Empty until the index loads.
pub fn fields_in_box(
    lat0: f64,
    lon0: f64,
    lat1: f64,
    lon1: f64,
    all: bool,
    limit: usize,
) -> Vec<FieldDot> {
    let inside = |la: f64, lo: f64| la >= lat0 && la <= lat1 && lo >= lon0 && lo <= lon1;
    let mut out: Vec<FieldDot> = REGION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .filter(|a| inside(a.lat, a.lon))
        .take(limit)
        .map(|a| FieldDot {
            icao: a.icao.clone(),
            lat: a.lat,
            lon: a.lon,
            towered: true,
        })
        .collect();
    if all && out.len() < limit {
        let dir = FIELD_DIR.lock().unwrap_or_else(|e| e.into_inner());
        for (icao, _, (la, lo)) in dir.iter() {
            if out.len() >= limit {
                break;
            }
            if inside(*la, *lo) && !out.iter().any(|f| f.icao == *icao) {
                out.push(FieldDot {
                    icao: icao.clone(),
                    lat: *la,
                    lon: *lo,
                    towered: false,
                });
            }
        }
    }
    out
}

/// A ground station's nominal antenna height. VHF range is line of sight, so
/// even on the ramp the mast buys a few miles.
pub(super) const ANTENNA_FT: f64 = 50.0;

/// VHF radio range: the line-of-sight horizon for a receiver `h` feet above
/// the station's elevation, plus the station's own mast. Terrain masking is
/// ignored — this drives what the panel *shows*, a planning aid, not a
/// propagation model.
pub fn vhf_range_nm(height_above_field_ft: f64) -> f64 {
    1.23 * (height_above_field_ft.max(0.0).sqrt() + ANTENNA_FT.sqrt())
}

/// One field inside VHF range, for the panel's ATC tab and the crew's
/// "nearest station" note.
pub struct RangedField {
    pub name: String,
    pub dist_nm: f64,
    /// This is the active station — the one that answers radio calls.
    pub active: bool,
    /// Published services in briefing order, from [`services`].
    pub services: Vec<(String, i64)>,
}

/// Every towered field within VHF line of sight of the aircraft, nearest
/// first, capped at `limit`. Reads the region index; with the region off (or
/// not indexed yet) it falls back to the active station alone, so the list
/// never knows less than the station does.
pub fn fields_in_range(lat: f64, lon: f64, alt_msl_ft: f64, limit: usize) -> Vec<RangedField> {
    let active_name = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(name, _, _)| name.clone());
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    let mut v: Vec<RangedField> = region
        .iter()
        .filter_map(|a| {
            let d = crate::nav::haversine_nm(lat, lon, a.lat, a.lon);
            (d <= vhf_range_nm(alt_msl_ft - a.apt.elev_ft)).then(|| RangedField {
                name: a.apt.name.clone(),
                dist_nm: d,
                active: active_name.as_deref() == Some(a.apt.name.as_str()),
                services: services(&a.apt),
            })
        })
        .collect();
    drop(region);
    if v.is_empty() {
        if let Some((name, svcs)) = active_station_in_range(lat, lon, alt_msl_ft) {
            let (_, alat, alon) = ACTIVE_FIELD
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
                .unwrap_or_default();
            v.push(RangedField {
                name,
                dist_nm: crate::nav::haversine_nm(lat, lon, alat, alon),
                active: true,
                services: svcs,
            });
        }
    }
    v.sort_by(|x, y| x.dist_nm.total_cmp(&y.dist_nm));
    v.truncate(limit);
    v
}

/// You call it, you get it: when a keyed transmission goes out on a frequency
/// the active station does NOT cover, but which belongs to another in-range
/// field, arm the station swap so THAT field answers the call about to be
/// routed. The ATC tab invites the pilot to tune any in-range station;
/// without this, calling Badajoz on its real tower frequency was answered by
/// dead air and a crew note about Gibraltar (2026-07-10). Unicom is excluded
/// (a shared party line, not a facility), and the nearest match wins when
/// fields share a frequency. Stamps the region cooldown so the automatic
/// handoff can't immediately yank the station away from an explicit tune.
/// Commit the station to the field we are arriving at, by its TOWER
/// frequency: while this is set the enroute region may hand the station TO
/// that field and never away from it, and no other field may take it.
///
/// The region pick is service-weighted DISTANCE, which is the right rule for
/// crossing a country and the wrong one for the last fifteen miles. Inbound
/// to Heathrow with the inbound call already made and answered, the tick
/// found London City eight miles off the nose against Heathrow's fifteen and
/// took the station; the final report and then the taxi-in request were
/// transmitted to a tower that was no longer listening, both `void`, and the
/// 737 landed on 27L with no clearance and sat on it for three minutes with
/// traffic inside a mile (LFLL-EGLL, 2026-09-17). `None` releases it.
pub fn pin_arrival_station(tower_khz: Option<i64>) {
    *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner()) = tower_khz;
}

/// End radar service for an arrival to a field with no staffed tower.
/// Idempotent: the enroute/arrival seam may be retried, but the retirement is
/// one transition and never an invented station for the destination.
pub fn begin_uncontrolled_arrival(destination: &str) {
    if OWN_STRIP.uncontrolled_arrival.swap(true, Ordering::SeqCst) {
        return;
    }
    *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *REGION_HANDOFF.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.departure_handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.clearance.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.assigned_squawk.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.ifr_clearance.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.ifr_flight.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.squawk_watch.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.squawk_nudge.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.vectors.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.landing_handoff.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.landing_hold.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.pending_landing_release
        .lock()
        .unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.taxi_route.lock().unwrap_or_else(|e| e.into_inner()) = None;
    *OWN_STRIP.taxi_to_stand.lock().unwrap_or_else(|e| e.into_inner()) = false;
    *OWN_STRIP.self_taxi.lock().unwrap_or_else(|e| e.into_inner()) = None;
    tracing::info!(%destination, "radar service ended for uncontrolled arrival");
    sim::blackbox::record(
        "atc",
        serde_json::json!({
            "event": "uncontrolled_arrival",
            "destination": destination,
            "why": "no_staffed_tower",
        }),
    );
}

pub(crate) fn uncontrolled_arrival() -> bool {
    OWN_STRIP.uncontrolled_arrival.load(Ordering::SeqCst)
}

/// Release the arrival-only service suppression. The next region tick may use
/// the installed station normally again; a new station install also clears it.
pub fn end_arrival() {
    OWN_STRIP.uncontrolled_arrival.store(false, Ordering::SeqCst);
}

/// The pinned arrival field's tower frequency, if the crew has committed to
/// one. Crate-internal: the crew sets the pin, the region reads it.
pub(super) fn arrival_pin_khz() -> Option<i64> {
    *ARRIVAL_PIN.lock().unwrap_or_else(|e| e.into_inner())
}

pub fn arm_station_for_freq(com1_khz: i64, lat: f64, lon: f64, alt_msl_ft: f64, now: Instant) {
    if PENDING_STATION
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some()
    {
        return; // a switch is already waiting to be consumed
    }
    let active_name = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|(name, _, _)| name.clone());
    let region = REGION.lock().unwrap_or_else(|e| e.into_inner());
    // The channel nearest the dial first, then the nearest field: the 5 kHz
    // slack is for a 25 kHz row read against an 8.33 dial, never a reason
    // for a nearer field to take the call. LXGB-LEJR 2026-10-09: Jerez
    // Tower tuned on 118.555, Trebujena's 118.550 16.9 nm away answered
    // "join final runway zero eight" for a field with only 02/20.
    let hit = region
        .iter()
        .filter(|a| active_name.as_deref() != Some(a.apt.name.as_str()))
        .filter_map(|a| {
            services(&a.apt)
                .iter()
                .filter(|(word, _)| word != "unicom")
                .map(|(_, khz)| (khz - com1_khz).abs())
                .filter(|gap| *gap <= 5)
                .min()
                .map(|gap| (gap, a))
        })
        .map(|(gap, a)| (gap, crate::nav::haversine_nm(lat, lon, a.lat, a.lon), a))
        .filter(|(_, d, a)| *d <= vhf_range_nm(alt_msl_ft - a.apt.elev_ft))
        .min_by(|x, y| x.0.cmp(&y.0).then(x.1.total_cmp(&y.1)));
    if let Some((_, d, a)) = hit {
        tracing::info!(
            to = %a.apt.name,
            khz = com1_khz,
            nm = format!("{d:.1}"),
            "station swap: transmission on another in-range field's frequency"
        );
        *PENDING_STATION.lock().unwrap_or_else(|e| e.into_inner()) =
            Some((a.icao.clone(), a.apt.clone()));
        *LAST_REGION_SWITCH.lock().unwrap_or_else(|e| e.into_inner()) = Some(now);
    }
}

pub fn active_station_range_nm(lat: f64, lon: f64) -> Option<f64> {
    let (_, alat, alon) = ACTIVE_FIELD
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()?;
    Some(crate::nav::haversine_nm(lat, lon, alat, alon))
}
