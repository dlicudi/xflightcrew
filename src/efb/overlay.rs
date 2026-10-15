//! The Map tab's weather and airspace for a view (GET /overlay?bbox=): the
//! flight category and wind of every field the sim's METAR file reports
//! (`weather`, the file the app already reads, never the sim), and the
//! no-go volumes the local planner flies round (`airspace`, the sim's own
//! OpenAir file). Both read off the runtime's thread and kept: the METARs
//! re-read only when the file changed, the airspace once. Bounded like the
//! basemap: at most [`WX_MAX`] dots and [`MAX_BYTES`] of outlines.

use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant, SystemTime};

use serde::Serialize;

use crate::airspace::Airspace;

/// A field's weather on the map.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct WxDot {
    pub(crate) icao: String,
    pub(crate) lat: f64,
    pub(crate) lon: f64,
    /// "VFR", "MVFR", "IFR" or "LIFR".
    pub(crate) cat: &'static str,
    /// "270/15", "270/15G25", "VRB/03" or "calm".
    pub(crate) wind: String,
}

/// A no-go volume's outline: [lat, lon, lat, lon, ...].
#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct SpaceOut {
    pub(crate) name: String,
    pub(crate) pts: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(crate) struct Overlay {
    pub(crate) wx: Vec<WxDot>,
    pub(crate) airspace: Vec<SpaceOut>,
}

/// At most this many weather dots in one answer.
pub(crate) const WX_MAX: usize = 400;
/// The outlines in one answer stop at about this many bytes.
pub(crate) const MAX_BYTES: usize = 200_000;
/// Airspace only for a view no taller than this (degrees of latitude).
pub(crate) const AIRSPACE_SPAN_DEG: f64 = crate::efb::basemap::TOWERED_SPAN_DEG;
/// The METAR file is looked at no oftener than this.
const WX_EVERY: Duration = Duration::from_secs(60);

/// The flight category of one METAR line (FAA: ceiling, the lowest broken,
/// overcast or obscured layer, and visibility): LIFR below 500 ft or 1 sm,
/// IFR below 1,000 ft or 3 sm, MVFR to 3,000 ft or 5 sm, else VFR. `None`
/// for a line that is no METAR. Pure.
pub(crate) fn category(line: &str) -> Option<&'static str> {
    let m = crate::weather::parse(line)?;
    let ceiling_ft = line
        .split_whitespace()
        .skip(2)
        .take_while(|t| !matches!(*t, "RMK" | "NOSIG" | "BECMG" | "TEMPO"))
        .filter_map(|t| {
            let rest = ["BKN", "OVC", "VV"]
                .iter()
                .find_map(|c| t.strip_prefix(c))?;
            let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
            (digits.len() == 3).then(|| digits.parse::<f64>().ok().map(|h| h * 100.0))?
        })
        .reduce(f64::min)
        .unwrap_or(f64::INFINITY);
    let sm = m.vis_m.map_or(f64::INFINITY, |v| v as f64 / 1609.344);
    Some(if ceiling_ft < 500.0 || sm < 1.0 {
        "LIFR"
    } else if ceiling_ft < 1000.0 || sm < 3.0 {
        "IFR"
    } else if ceiling_ft <= 3000.0 || sm <= 5.0 {
        "MVFR"
    } else {
        "VFR"
    })
}

/// The wind as the map writes it. Pure.
fn wind_words(m: &crate::weather::Metar) -> String {
    if m.wind_kts == 0 {
        return "calm".into();
    }
    let dir = m.wind_dir.map_or("VRB".to_string(), |d| format!("{d:03}"));
    let gust = m.gust_kts.map(|g| format!("G{g}")).unwrap_or_default();
    format!("{dir}/{:02}{gust}", m.wind_kts)
}

/// Every field of `fields` (ICAO, lat, lon) the METAR text reports, with
/// its category and wind. Pure.
pub(crate) fn dots_from(text: &str, fields: &[(String, f64, f64)]) -> Vec<WxDot> {
    let mut by_icao = std::collections::HashMap::new();
    for line in text.lines() {
        let line = line.trim();
        let Some(m) = crate::weather::parse(line) else {
            continue;
        };
        let Some(cat) = category(line) else { continue };
        by_icao
            .entry(m.station.clone())
            .or_insert((cat, wind_words(&m)));
    }
    fields
        .iter()
        .filter_map(|(icao, lat, lon)| {
            let (cat, wind) = by_icao.get(icao)?;
            Some(WxDot {
                icao: icao.clone(),
                lat: *lat,
                lon: *lon,
                cat,
                wind: wind.clone(),
            })
        })
        .collect()
}

fn inside(bbox: [f64; 4], lat: f64, lon: f64) -> bool {
    lat >= bbox[0] && lat <= bbox[2] && lon >= bbox[1] && lon <= bbox[3]
}

/// The answer for a box `[lat0, lon0, lat1, lon1]` from what is held: the
/// dots inside it (at most [`WX_MAX`]), and the volumes overlapping it while
/// the box is no taller than [`AIRSPACE_SPAN_DEG`], until [`MAX_BYTES`].
/// Pure.
pub(crate) fn answer_with(spaces: &[Airspace], wx: &[WxDot], bbox: [f64; 4]) -> Overlay {
    let wx = wx
        .iter()
        .filter(|d| inside(bbox, d.lat, d.lon))
        .take(WX_MAX)
        .cloned()
        .collect();
    let mut airspace = Vec::new();
    if bbox[2] - bbox[0] <= AIRSPACE_SPAN_DEG {
        let r4 = |v: f64| (v * 1e4).round() / 1e4;
        let mut bytes = 0;
        for a in spaces.iter().filter(|a| a.poly.len() >= 3) {
            let (mut la0, mut la1, mut lo0, mut lo1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for &(la, lo) in &a.poly {
                la0 = la0.min(la);
                la1 = la1.max(la);
                lo0 = lo0.min(lo);
                lo1 = lo1.max(lo);
            }
            if la0 > bbox[2] || la1 < bbox[0] || lo0 > bbox[3] || lo1 < bbox[1] {
                continue;
            }
            bytes += a.poly.len() * 20 + a.name.len() + 20;
            if bytes > MAX_BYTES {
                break;
            }
            airspace.push(SpaceOut {
                name: a.name.split_whitespace().collect::<Vec<_>>().join(" "),
                pts: a
                    .poly
                    .iter()
                    .flat_map(|&(la, lo)| [r4(la), r4(lo)])
                    .collect(),
            });
        }
    }
    Overlay { wx, airspace }
}

/// The no-go volumes, the whole file parsed once (memo, holds no flight
/// fact).
static AIRSPACE: OnceLock<Vec<Airspace>> = OnceLock::new();

struct WxHeld {
    checked: Option<Instant>,
    modified: Option<SystemTime>,
    dots: Vec<WxDot>,
}

/// The METAR file's dots, re-built when the file changed (memo of a file).
static METARS: Mutex<WxHeld> = Mutex::new(WxHeld {
    checked: None,
    modified: None,
    dots: Vec::new(),
});

/// The METAR file's dots, handed to `f`: looked at no oftener than
/// [`WX_EVERY`], rebuilt when the file changed or nothing placed yet (the
/// field index loads late).
fn with_wx_dots<R>(f: impl FnOnce(&[WxDot]) -> R) -> R {
    let mut h = METARS.lock().unwrap_or_else(|e| e.into_inner());
    if h.checked.is_some_and(|t| t.elapsed() < WX_EVERY) {
        return f(&h.dots);
    }
    h.checked = Some(Instant::now());
    let path = crate::weather::latest_file(&crate::config::xplane_root());
    let modified = path
        .as_ref()
        .and_then(|p| std::fs::metadata(p).ok())
        .and_then(|m| m.modified().ok());
    if modified == h.modified && !h.dots.is_empty() {
        return f(&h.dots);
    }
    let text = path
        .and_then(|p| std::fs::read_to_string(p).ok())
        .unwrap_or_default();
    let fields: Vec<(String, f64, f64)> =
        crate::party_line::fields_in_box(-90.0, -180.0, 90.0, 180.0, true, usize::MAX)
            .into_iter()
            .map(|f| (f.icao, f.lat, f.lon))
            .collect();
    h.modified = modified;
    h.dots = dots_from(&text, &fields);
    f(&h.dots)
}

/// The answer for a box, from the files (blocking: off the runtime's
/// thread).
pub(crate) fn answer(bbox: [f64; 4]) -> Overlay {
    let spaces = AIRSPACE.get_or_init(|| {
        let path = crate::settings::live().airspace_path.clone();
        crate::airspace::load_box(&path, -90.0, 90.0, -180.0, 180.0)
    });
    with_wx_dots(|wx| answer_with(spaces, wx, bbox))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_metar_has_its_flight_category() {
        let cat = |l: &str| category(l).unwrap();
        assert_eq!(cat("EGKK 151020Z 27015KT 9999 FEW040 15/09 Q1012"), "VFR");
        assert_eq!(cat("EGKK 151020Z 27015KT CAVOK 15/09 Q1012"), "VFR");
        assert_eq!(cat("EGKK 151020Z 27015KT 9999 BKN025 15/09 Q1012"), "MVFR");
        assert_eq!(
            cat("KJFK 151020Z 27015KT 4SM BR SCT040 15/09 A2992"),
            "MVFR"
        );
        assert_eq!(
            cat("EGKK 151020Z 27015KT 9999 SCT004 OVC008 15/09 Q1012"),
            "IFR"
        );
        assert_eq!(
            cat("EGKK 151020Z 27015KT 2500 BR FEW040 15/09 Q1012"),
            "IFR"
        );
        assert_eq!(
            cat("EGKK 151020Z 00000KT 0300 FG VV001 10/10 Q1012"),
            "LIFR"
        );
        // A trend group is no observation.
        assert_eq!(
            cat("EGKK 151020Z 27015KT 9999 FEW040 15/09 Q1012 TEMPO BKN005"),
            "VFR"
        );
        assert_eq!(category("not a metar"), None);
    }

    fn field(icao: &str, lat: f64, lon: f64) -> (String, f64, f64) {
        (icao.into(), lat, lon)
    }

    /// The file's stations placed at their fields, the wind written as the
    /// map shows it; a station with no field, and a field with no METAR,
    /// are not on the map.
    #[test]
    fn the_metar_file_places_dots_at_fields() {
        let text = "EGKK 151020Z 27015G25KT 9999 FEW040 15/09 Q1012\n\
                    LEMG 151030Z VRB03KT CAVOK 22/10 Q1018\n\
                    ZZZZ 151030Z 00000KT CAVOK 22/10 Q1018\n";
        let fields = [
            field("EGKK", 51.148, -0.19),
            field("LEMG", 36.675, -4.499),
            field("EGLL", 51.47, -0.46),
        ];
        let d = dots_from(text, &fields);
        assert_eq!(d.len(), 2);
        assert_eq!(d[0].icao, "EGKK");
        assert_eq!(d[0].wind, "270/15G25");
        assert_eq!(d[1].wind, "VRB/03");
        assert_eq!(d[1].cat, "VFR");
    }

    fn square(name: &str, lat: f64, lon: f64) -> Airspace {
        Airspace {
            class: 'R',
            name: name.into(),
            floor_ft: 0.0,
            ceiling_ft: 5000.0,
            poly: vec![
                (lat, lon),
                (lat + 0.1, lon),
                (lat + 0.1, lon + 0.1),
                (lat, lon + 0.1),
            ],
        }
    }

    /// A box's answer: the volume overlapping it outlined with its name,
    /// the one elsewhere not; a view too wide for outlines has the
    /// weather only.
    #[test]
    fn a_restricted_volume_is_in_its_box_answer() {
        let spaces = [
            square("R-164   (ALGECIRAS)", 36.1, -5.5),
            square("R-77", 45.0, 2.0),
        ];
        let wx = [WxDot {
            icao: "LXGB".into(),
            lat: 36.15,
            lon: -5.35,
            cat: "VFR",
            wind: "calm".into(),
        }];
        let a = answer_with(&spaces, &wx, [35.5, -6.0, 36.8, -4.5]);
        assert_eq!(a.airspace.len(), 1);
        assert_eq!(a.airspace[0].name, "R-164 (ALGECIRAS)");
        assert_eq!(a.airspace[0].pts.len(), 8);
        assert_eq!(a.wx.len(), 1);
        let wide = answer_with(&spaces, &wx, [30.0, -10.0, 50.0, 10.0]);
        assert!(wide.airspace.is_empty() && wide.wx.len() == 1);
        // The page asks with the same limit.
        let js = include_str!("map.js");
        assert!(js.contains(&format!("AIRSPACE_DEG = {};", AIRSPACE_SPAN_DEG)));
    }
}
