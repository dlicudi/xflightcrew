//! Weather away from the aircraft: X-Plane 12's real-weather download leaves
//! the METARs it fetched in `Output/real weather/metar-YYYY-MM-DD-HH.MM.txt`,
//! one station per line. That is the only honest source for "what weather can
//! we expect on arrival": the sim's own weather datarefs describe the air the
//! aircraft is IN, and answering an arrival question from them (as happened
//! live on 2026-09-12, 45 nm out of Malaga) is a wrong answer said
//! confidently. No network call: the file is read on demand, never polled.
//!
//! When real weather is off the newest file is stale; the answer then says
//! how old it is rather than pretending it is current.

use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

/// A METAR reduced to what a copilot would brief from it.
#[derive(Debug, Clone, PartialEq)]
pub struct Metar {
    pub station: String,
    /// `ddhhmm` observation time, spoken as "hhmm zulu".
    pub time: String,
    /// Wind direction (true) or `None` for variable/calm.
    pub wind_dir: Option<u32>,
    pub wind_kts: u32,
    pub gust_kts: Option<u32>,
    /// Visibility in metres; `9999`/CAVOK become `None` (10 km or more).
    pub vis_m: Option<u32>,
    pub cavok: bool,
    /// Lowest cloud layer as spoken ("few at 3500 feet").
    pub clouds: Vec<String>,
    /// Present weather, spoken ("light rain", "thunderstorm").
    pub wx: Vec<String>,
    pub temp_c: Option<i32>,
    pub dew_c: Option<i32>,
    /// QNH in hectopascals (an `A` group is converted).
    pub qnh_hpa: Option<u32>,
}

/// Newest `metar-*.txt` under the sim's real-weather output folder.
pub fn latest_file(xplane_root: &str) -> Option<PathBuf> {
    let dir = Path::new(xplane_root).join("Output").join("real weather");
    let mut best: Option<(SystemTime, PathBuf)> = None;
    for e in std::fs::read_dir(dir).ok()?.flatten() {
        let p = e.path();
        let name = p.file_name()?.to_string_lossy().to_string();
        if !(name.starts_with("metar") && name.ends_with(".txt")) {
            continue;
        }
        let m = e.metadata().and_then(|m| m.modified()).ok()?;
        if best.as_ref().is_none_or(|(t, _)| m > *t) {
            best = Some((m, p));
        }
    }
    best.map(|(_, p)| p)
}

/// The raw METAR line for `icao` in a file's text, if present.
pub fn line_for<'a>(text: &'a str, icao: &str) -> Option<&'a str> {
    let icao = icao.trim().to_ascii_uppercase();
    text.lines()
        .map(str::trim)
        .find(|l| l.split_whitespace().next() == Some(icao.as_str()))
}

/// Every station ident the file carries (for "is this token an airport?").
pub fn has_station(text: &str, icao: &str) -> bool {
    line_for(text, icao).is_some()
}

#[cfg(test)]
thread_local! {
    /// A test's METAR text in place of the sim's file: CI has no X-Plane.
    static TEST_METARS: std::cell::RefCell<Option<String>> = const { std::cell::RefCell::new(None) };
}

/// Stand in `text` for the sim's METAR file on this test's thread.
#[cfg(test)]
pub fn set_test_metars(text: Option<&str>) {
    TEST_METARS.with(|t| *t.borrow_mut() = text.map(str::to_string));
}

/// The newest METAR file's text under `xplane_root`, if there is one.
pub fn latest_text(xplane_root: &str) -> Option<String> {
    #[cfg(test)]
    if let Some(t) = TEST_METARS.with(|t| t.borrow().clone()) {
        return Some(t);
    }
    latest_file(xplane_root).and_then(|p| std::fs::read_to_string(p).ok())
}

fn num(s: &str) -> Option<u32> {
    (!s.is_empty() && s.bytes().all(|b| b.is_ascii_digit())).then(|| s.parse().ok())?
}

fn signed_temp(s: &str) -> Option<i32> {
    let (neg, digits) = match s.strip_prefix('M') {
        Some(d) => (true, d),
        None => (false, s),
    };
    let v = num(digits)? as i32;
    Some(if neg { -v } else { v })
}

fn cloud_word(code: &str) -> Option<&'static str> {
    Some(match code {
        "FEW" => "few",
        "SCT" => "scattered",
        "BKN" => "broken",
        "OVC" => "overcast",
        "VV" => "sky obscured, vertical visibility",
        _ => return None,
    })
}

fn wx_words(tok: &str) -> Option<String> {
    let (intensity, rest) = match tok.as_bytes().first()? {
        b'-' => ("light ", &tok[1..]),
        b'+' => ("heavy ", &tok[1..]),
        _ => ("", tok),
    };
    let (vicinity, rest) = match rest.strip_prefix("VC") {
        Some(r) => (true, r),
        None => (false, rest),
    };
    let mut parts: Vec<&str> = Vec::new();
    let mut s = rest;
    while s.len() >= 2 {
        let (code, tail) = s.split_at(2);
        parts.push(match code {
            "TS" => "thunderstorm",
            "SH" => "showers of",
            "FZ" => "freezing",
            "MI" => "shallow",
            "BC" => "patches of",
            "BL" => "blowing",
            "DR" => "drifting",
            "PR" => "partial",
            "RA" => "rain",
            "DZ" => "drizzle",
            "SN" => "snow",
            "SG" => "snow grains",
            "GR" => "hail",
            "GS" => "small hail",
            "PL" => "ice pellets",
            "IC" => "ice crystals",
            "UP" => "unknown precipitation",
            "BR" => "mist",
            "FG" => "fog",
            "HZ" => "haze",
            "FU" => "smoke",
            "DU" => "dust",
            "SA" => "sand",
            "VA" => "volcanic ash",
            "SQ" => "squalls",
            "FC" => "funnel cloud",
            "DS" => "dust storm",
            "SS" => "sand storm",
            "PO" => "dust whirls",
            _ => return None,
        });
        s = tail;
    }
    if parts.is_empty() || !s.is_empty() {
        return None;
    }
    let mut out = format!("{intensity}{}", parts.join(" "));
    if vicinity {
        out.push_str(" in the vicinity");
    }
    Some(out)
}

/// Parse one METAR line. Tolerant: anything unrecognised is skipped, and a
/// line whose ident or time is missing is not a METAR.
pub fn parse(line: &str) -> Option<Metar> {
    let mut toks = line.split_whitespace().peekable();
    let station = toks.next()?.to_string();
    if station.len() != 4 || !station.bytes().all(|b| b.is_ascii_uppercase()) {
        return None;
    }
    let time = toks
        .next()
        .and_then(|t| t.strip_suffix('Z'))
        .filter(|t| t.len() == 6 && t.bytes().all(|b| b.is_ascii_digit()))?
        .to_string();
    let mut m = Metar {
        station,
        time,
        wind_dir: None,
        wind_kts: 0,
        gust_kts: None,
        vis_m: None,
        cavok: false,
        clouds: Vec::new(),
        wx: Vec::new(),
        temp_c: None,
        dew_c: None,
        qnh_hpa: None,
    };
    let mut wind_seen = false;
    for tok in toks {
        // Anything after the remarks / trend groups is not the observation.
        if matches!(tok, "RMK" | "NOSIG" | "BECMG" | "TEMPO") {
            break;
        }
        if tok == "AUTO" || tok == "COR" {
            continue;
        }
        if tok == "CAVOK" {
            m.cavok = true;
            continue;
        }
        // Wind: dddffKT, dddffGggKT, VRBffKT, dddffMPS.
        if !wind_seen {
            if let Some(body) = tok.strip_suffix("KT").or_else(|| tok.strip_suffix("MPS")) {
                let mps = tok.ends_with("MPS");
                let (dir, rest) = body.split_at(body.len().min(3));
                let (spd, gust) = match rest.split_once('G') {
                    Some((s, g)) => (s, num(g)),
                    None => (rest, None),
                };
                if let Some(spd) = num(spd) {
                    let to_kts = |v: u32| {
                        if mps {
                            (v as f64 * 1.943_844).round() as u32
                        } else {
                            v
                        }
                    };
                    m.wind_dir = if dir == "VRB" { None } else { num(dir) };
                    m.wind_kts = to_kts(spd);
                    m.gust_kts = gust.map(to_kts);
                    wind_seen = true;
                    continue;
                }
            }
        }
        // Variable wind band (090V160): direction is already known; skip.
        if tok.len() == 7 && tok.as_bytes()[3] == b'V' && num(&tok[..3]).is_some() {
            continue;
        }
        // Visibility: 4-digit metres or statute miles ("10SM", "1/2SM").
        if tok.len() == 4 && num(tok).is_some() && m.vis_m.is_none() && wind_seen {
            let v = num(tok).unwrap();
            m.vis_m = (v < 9999).then_some(v);
            continue;
        }
        if let Some(sm) = tok.strip_suffix("SM") {
            let sm = sm.trim_start_matches('P').trim_start_matches('M');
            let miles = match sm.split_once('/') {
                Some((a, b)) => match (a.parse::<f64>(), b.parse::<f64>()) {
                    (Ok(a), Ok(b)) if b > 0.0 => a / b,
                    _ => continue,
                },
                None => match sm.parse::<f64>() {
                    Ok(v) => v,
                    Err(_) => continue,
                },
            };
            let metres = (miles * 1609.344).round() as u32;
            m.vis_m = (metres < 9999).then_some(metres);
            continue;
        }
        // Clouds: FEW035, SCT026CB, BKN010, OVC005, VV002.
        if let Some(code) = ["FEW", "SCT", "BKN", "OVC", "VV"]
            .iter()
            .find(|c| tok.starts_with(*c))
        {
            let rest = &tok[code.len()..];
            let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
            if let Some(hundreds) = num(&digits) {
                let word = cloud_word(code).unwrap_or("cloud");
                let suffix = if rest.ends_with("CB") {
                    ", cumulonimbus"
                } else if rest.ends_with("TCU") {
                    ", towering cumulus"
                } else {
                    ""
                };
                m.clouds
                    .push(format!("{word} at {} feet{suffix}", hundreds * 100));
            }
            continue;
        }
        if matches!(tok, "NSC" | "SKC" | "CLR" | "NCD") {
            continue;
        }
        // Temperature / dew point: 26/18, M02/M05, 26/.
        if let Some((t, d)) = tok.split_once('/') {
            if let Some(tc) = signed_temp(t) {
                if m.temp_c.is_none() {
                    m.temp_c = Some(tc);
                    m.dew_c = signed_temp(d);
                    continue;
                }
            }
        }
        // Pressure: Q1020 (hPa) or A2992 (inHg hundredths).
        if let Some(q) = tok.strip_prefix('Q').and_then(num) {
            m.qnh_hpa = Some(q);
            continue;
        }
        if let Some(a) = tok.strip_prefix('A').and_then(num) {
            if tok.len() == 5 {
                m.qnh_hpa = Some((a as f64 / 100.0 * 33.8639).round() as u32);
                continue;
            }
        }
        if let Some(w) = wx_words(tok) {
            m.wx.push(w);
        }
    }
    Some(m)
}

/// The briefing line a copilot would read from the parsed METAR.
pub fn spoken(m: &Metar, label: &str) -> String {
    let mut parts: Vec<String> = Vec::new();
    let hhmm = &m.time[2..];
    parts.push(format!("{label}, {hhmm} zulu"));
    let wind = match (m.wind_dir, m.wind_kts) {
        (_, 0) => "wind calm".to_string(),
        (None, k) => format!("wind variable at {k} knots"),
        (Some(d), k) => format!("wind {d:03} at {k} knots"),
    };
    parts.push(match m.gust_kts {
        Some(g) => format!("{wind} gusting {g}"),
        None => wind,
    });
    if m.cavok {
        parts.push("CAVOK".to_string());
    } else {
        parts.push(match m.vis_m {
            None => "visibility 10 kilometres or more".to_string(),
            Some(v) if v >= 5000 => format!("visibility {} kilometres", v / 1000),
            Some(v) => format!("visibility {v} metres"),
        });
        parts.extend(m.wx.iter().cloned());
        if m.clouds.is_empty() {
            parts.push("no significant cloud".to_string());
        } else {
            parts.extend(m.clouds.iter().take(3).cloned());
        }
    }
    if let Some(t) = m.temp_c {
        parts.push(match m.dew_c {
            Some(d) => format!("temperature {t}, dew point {d}"),
            None => format!("temperature {t}"),
        });
    }
    if let Some(q) = m.qnh_hpa {
        parts.push(format!("QNH {q}"));
    }
    format!("{}.", parts.join(", "))
}

/// How stale the file is, spoken only when it matters (real weather off, or
/// the sim has not refreshed in a while). `None` = fresh enough to say nothing.
pub fn staleness(modified: SystemTime, now: SystemTime) -> Option<String> {
    let age = now.duration_since(modified).unwrap_or(Duration::ZERO);
    if age < Duration::from_secs(2 * 3600) {
        return None;
    }
    let hours = age.as_secs() / 3600;
    Some(if hours < 48 {
        format!("That report is {hours} hours old, real weather may be off.")
    } else {
        format!(
            "That report is {} days old, real weather may be off.",
            hours / 24
        )
    })
}

/// A report's surface wind as (direction true, knots). A light variable
/// wind is CALM, `(0, 0)`: there is no runway to face, so the runway in use
/// stands. As `None` it sent the tower to the wind at the aircraft, and LEMG
/// at VRB01KT cleared an arrival on 13's final to 31 on the air at 1,700 ft
/// (#270, 2026-10-11). `None` only for a strong variable wind, where no
/// runway is right.
pub fn wind_of(m: &Metar) -> Option<(f64, f64)> {
    match m.wind_dir {
        Some(d) => Some((d as f64, m.wind_kts as f64)),
        None if m.wind_kts <= CALM_VRB_MAX_KTS => Some((0.0, 0.0)),
        None => None,
    }
}

/// A variable wind up to this many knots reads as calm.
const CALM_VRB_MAX_KTS: u32 = 5;

/// The station's surface wind from the newest METAR on file, or `None`: no
/// file, no line for the station, a variable wind, or a file older than
/// [`staleness`] lets pass (real weather off, the sim not refreshing). The
/// arrival's runway pick reads this before the aircraft's own wind: at 30 nm
/// and 7,000 ft the aircraft is in the winds aloft, and the tower on the
/// surface may be working the other end (LXGB 27 into 090 at 21, 2026-09-14).
pub fn surface_wind(xplane_root: &str, icao: &str) -> Option<(f64, f64)> {
    let path = latest_file(xplane_root)?;
    let fresh = std::fs::metadata(&path)
        .and_then(|md| md.modified())
        .ok()
        .is_none_or(|t| staleness(t, SystemTime::now()).is_none());
    if !fresh {
        return None;
    }
    let text = std::fs::read_to_string(&path).ok()?;
    wind_of(&parse(line_for(&text, icao)?)?)
}

/// The field's surface wind for the tower, `(from, kts)`: the sim's own
/// surface wind for the region (the lowest layer of the wind profile the
/// monitor already reads, `winds_aloft`), the weather the aircraft is
/// actually flown in and there without real weather; else the field's
/// METAR. Only for the field the station is at, near the aircraft: a
/// destination far away has its METAR alone (`surface_wind`). With neither
/// published the tower judged an arrival on the air at the aircraft (008 at
/// 5 kt at 1,700 ft against the field's 157 at 2) and cleared 31 where the
/// field and the real LEMG used 13 (#270, 2026-10-11).
pub fn field_wind(xplane_root: &str, icao: &str) -> Option<(f64, f64)> {
    WINDS_CACHE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .and_then(|(_, w)| w.layers.first().map(|&(_, from, kts)| (from, kts)))
        .or_else(|| surface_wind(xplane_root, icao))
}

/// The whole answer for one station: reads the newest file, parses the line,
/// speaks it, and flags a stale file. `Err` carries the spoken reason.
pub fn briefing(xplane_root: &str, icao: &str, label: &str) -> Result<String, String> {
    let Some(path) = latest_file(xplane_root) else {
        return Err(format!(
            "I have no METARs on file, so nothing for {label}. Real weather writes them."
        ));
    };
    let text = std::fs::read_to_string(&path)
        .map_err(|_| format!("Couldn't read the METAR file for {label}."))?;
    let Some(line) = line_for(&text, icao) else {
        return Err(format!("No METAR on file for {label}."));
    };
    let Some(m) = parse(line) else {
        return Err(format!("The METAR for {label} didn't parse."));
    };
    let mut out = spoken(&m, label);
    if let Some(stale) = std::fs::metadata(&path)
        .and_then(|md| md.modified())
        .ok()
        .and_then(|t| staleness(t, SystemTime::now()))
    {
        out.push(' ');
        out.push_str(&stale);
    }
    Ok(out)
}

/// The region's wind profile: the winds we could be in at another level, as
/// opposed to [`wind_of`]/[`surface_wind`], which are the wind somewhere
/// specific. Layers are `(altitude ft MSL, direction FROM degrees true,
/// speed knots)`, ascending.
///
/// This is what makes a cruise level a choice rather than a formula. On
/// EIDW-BIKF (2026-09-20) the jet stream core sat across FL300-FL390 — 93 kt
/// of headwind at the planned FL340 and 94 at FL390, but only 48 at FL236 —
/// and nothing in the plan had ever looked.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct WindsAloft {
    pub layers: Vec<(f64, f64, f64)>,
}

/// Meters per second to knots.
const MSC_TO_KTS: f64 = 1.943_844;

impl WindsAloft {
    /// Build from the sim's three parallel layer arrays. `None` unless all
    /// three agree on a length and at least two layers are usable — one layer
    /// is not a profile, and a profile is the only thing worth choosing from.
    pub fn from_layers(alt_m: &[f64], speed_msc: &[f64], dir_deg: &[f64]) -> Option<WindsAloft> {
        let n = alt_m.len();
        if n != speed_msc.len() || n != dir_deg.len() {
            return None;
        }
        let mut layers: Vec<(f64, f64, f64)> = alt_m
            .iter()
            .zip(speed_msc)
            .zip(dir_deg)
            .filter(|((a, s), d)| a.is_finite() && s.is_finite() && d.is_finite() && **s >= 0.0)
            .map(|((a, s), d)| (a * 3.280_84, d.rem_euclid(360.0), s * MSC_TO_KTS))
            .collect();
        layers.sort_by(|a, b| a.0.total_cmp(&b.0));
        (layers.len() >= 2).then_some(WindsAloft { layers })
    }

    /// The wind at `alt_ft` as `(east, north)` components in knots, linearly
    /// interpolated between the bracketing layers and held flat outside them.
    ///
    /// Interpolated as a VECTOR, never as a direction: averaging 350° and 010°
    /// as numbers gives 180°, a wind blowing the opposite way.
    fn uv_at(&self, alt_ft: f64) -> Option<(f64, f64)> {
        let uv = |&(_, d, s): &(f64, f64, f64)| {
            let r = d.to_radians();
            (-s * r.sin(), -s * r.cos())
        };
        let first = self.layers.first()?;
        let last = self.layers.last()?;
        if alt_ft <= first.0 {
            return Some(uv(first));
        }
        if alt_ft >= last.0 {
            return Some(uv(last));
        }
        let hi = self.layers.iter().position(|l| l.0 >= alt_ft)?;
        let (lo_l, hi_l) = (&self.layers[hi - 1], &self.layers[hi]);
        let span = hi_l.0 - lo_l.0;
        let f = if span > 0.0 {
            (alt_ft - lo_l.0) / span
        } else {
            0.0
        };
        let ((u0, v0), (u1, v1)) = (uv(lo_l), uv(hi_l));
        Some((u0 + (u1 - u0) * f, v0 + (v1 - v0) * f))
    }

    /// The wind at `alt_ft` as `(direction FROM degrees true, knots)`.
    pub fn at_ft(&self, alt_ft: f64) -> Option<(f64, f64)> {
        let (u, v) = self.uv_at(alt_ft)?;
        let kts = u.hypot(v);
        let from = (u.atan2(v).to_degrees() + 180.0).rem_euclid(360.0);
        Some((from, kts))
    }

    /// Headwind component on `track_deg` at `alt_ft`, knots. POSITIVE is a
    /// headwind; a tailwind is negative, because the sign is the whole
    /// question and a magnitude would hide which way the air is pushing.
    pub fn headwind_at(&self, alt_ft: f64, track_deg: f64) -> Option<f64> {
        let (u, v) = self.uv_at(alt_ft)?;
        let t = track_deg.to_radians();
        Some(-(u * t.sin() + v * t.cos()))
    }
}

/// How often the wind profile is actually fetched. The monitor asks every
/// tick at 3 Hz; the weather moves on its own clock, and three ARRAY reads
/// nine times a second is exactly the kind of volume the API-load histogram
/// exists to catch. Everything downstream tolerates a minute of staleness:
/// the plan is made once and the step-climb advisory sustains for two.
const WINDS_TTL: std::time::Duration = std::time::Duration::from_secs(60);
static WINDS_CACHE: std::sync::Mutex<Option<(std::time::Instant, WindsAloft)>> =
    std::sync::Mutex::new(None);

/// Read the region's wind profile from the sim, at most once per
/// [`WINDS_TTL`]. `None` whenever the three arrays can't all be had or don't
/// agree — a level chosen on half a wind profile would be worse than one
/// chosen on none, which is what the plan did before this existed.
///
/// A failed read keeps serving the last good profile until it expires: the
/// wind a minute ago is a far better answer than no wind at all, and the
/// alternative is an advisory that flickers with the API.
pub async fn winds_aloft(xp: &sim::xplane::XPlaneClient) -> Option<WindsAloft> {
    if let Some((t, w)) = WINDS_CACHE
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
    {
        if t.elapsed() < WINDS_TTL {
            return Some(w.clone());
        }
    }
    let (alt, spd, dir) = tokio::join!(
        xp.get_float_array(sim::datarefs::WINDS_ALOFT_ALT_M),
        xp.get_float_array(sim::datarefs::WINDS_ALOFT_SPEED_MSC),
        xp.get_float_array(sim::datarefs::WINDS_ALOFT_DIR_DEG_TRUE),
    );
    let fresh = (|| WindsAloft::from_layers(&alt.ok()?, &spd.ok()?, &dir.ok()?))();
    let mut cache = WINDS_CACHE.lock().unwrap_or_else(|e| e.into_inner());
    match fresh {
        Some(w) => {
            *cache = Some((std::time::Instant::now(), w.clone()));
            Some(w)
        }
        // Keep the stale one rather than going blind, but do not refresh its
        // stamp: the next tick tries again.
        None => cache.as_ref().map(|(_, w)| w.clone()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FILE: &str = "2026/09/12 12:00\nAGGH 121200Z 20002KT 9999 FEW015 SCT026 26/24 Q1011\nLEMG 121330Z 13010KT 090V160 9999 FEW035 26/18 Q1020 NOSIG\nLXGB 121320Z 10016KT 9999 FEW025 25/19 Q1019 NOSIG\n";

    #[test]
    fn the_surface_wind_is_the_reports_and_variable_is_none() {
        let m = parse(line_for(FILE, "LXGB").unwrap()).unwrap();
        assert_eq!(wind_of(&m), Some((100.0, 16.0)));
        // A light variable wind is calm (#270: LEMG VRB01KT); only a strong
        // one is no answer.
        let vrb = parse("LEMG 121330Z VRB03KT 9999 FEW035 26/18 Q1020").unwrap();
        assert_eq!(wind_of(&vrb), Some((0.0, 0.0)));
        let storm = parse("LEMG 121330Z VRB15G28KT 9999 FEW035 26/18 Q1020").unwrap();
        assert_eq!(wind_of(&storm), None);
        let calm = parse("LEMG 121330Z 00000KT 9999 FEW035 26/18 Q1020").unwrap();
        assert_eq!(wind_of(&calm), Some((0.0, 0.0)));
    }

    #[test]
    fn the_arrival_station_is_found_and_briefed() {
        let line = line_for(FILE, "lemg").unwrap();
        let m = parse(line).unwrap();
        assert_eq!(m.wind_dir, Some(130));
        assert_eq!(m.wind_kts, 10);
        assert_eq!(m.vis_m, None);
        assert_eq!(m.clouds, vec!["few at 3500 feet"]);
        assert_eq!((m.temp_c, m.dew_c), (Some(26), Some(18)));
        assert_eq!(m.qnh_hpa, Some(1020));
        assert_eq!(
            spoken(&m, "Malaga"),
            "Malaga, 1330 zulu, wind 130 at 10 knots, visibility 10 kilometres or more, few at 3500 feet, temperature 26, dew point 18, QNH 1020."
        );
        assert!(!has_station(FILE, "EGLL"));
    }

    #[test]
    fn gusts_weather_inches_and_miles_are_understood() {
        let m = parse("KJFK 121251Z 24015G25KT 2SM -RA BR BKN008 OVC015 M02/M04 A2992 RMK AO2")
            .unwrap();
        assert_eq!(m.gust_kts, Some(25));
        assert_eq!(m.vis_m, Some(3219));
        assert_eq!(m.wx, vec!["light rain", "mist"]);
        assert_eq!(
            m.clouds,
            vec!["broken at 800 feet", "overcast at 1500 feet"]
        );
        assert_eq!((m.temp_c, m.dew_c), (Some(-2), Some(-4)));
        assert_eq!(m.qnh_hpa, Some(1013));
        let s = spoken(&m, "Kennedy");
        assert!(s.contains("gusting 25"), "{s}");
        assert!(s.contains("visibility 3219 metres"), "{s}");
        assert!(s.contains("light rain, mist"), "{s}");
    }

    #[test]
    fn cavok_calm_and_variable_winds_read_plainly() {
        let m = parse("LEMD 121300Z 00000KT CAVOK 30/10 Q1018").unwrap();
        let s = spoken(&m, "Madrid");
        assert!(s.contains("wind calm, CAVOK"), "{s}");
        let m = parse("LEMD 121300Z VRB03KT 9999 NSC 30/10 Q1018").unwrap();
        assert!(spoken(&m, "Madrid").contains("wind variable at 3 knots"));
        assert!(spoken(&m, "Madrid").contains("no significant cloud"));
        // A thunderstorm with rain, cumulonimbus, in metres per second.
        let m = parse("UUEE 121300Z 22008MPS 4000 TSRA SCT020CB 18/16 Q1005").unwrap();
        assert_eq!(m.wind_kts, 16);
        assert_eq!(m.wx, vec!["thunderstorm rain"]);
        assert_eq!(m.clouds, vec!["scattered at 2000 feet, cumulonimbus"]);
    }

    #[test]
    fn a_stale_file_is_flagged_a_fresh_one_is_not() {
        let now = SystemTime::now();
        assert!(staleness(now - Duration::from_secs(1800), now).is_none());
        assert_eq!(
            staleness(now - Duration::from_secs(5 * 3600), now).as_deref(),
            Some("That report is 5 hours old, real weather may be off.")
        );
        assert!(staleness(now - Duration::from_secs(3 * 86400), now)
            .unwrap()
            .contains("3 days old"));
    }

    #[test]
    fn junk_is_not_a_metar() {
        assert!(parse("2026/09/12 12:00").is_none());
        assert!(parse("LEMG").is_none());
        assert!(line_for(FILE, "ZZZZ").is_none());
    }
}
