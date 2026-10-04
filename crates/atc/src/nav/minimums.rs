//! Minimum altitudes: MSA, SID climb minimums, grid MORA.

use super::*;

/// The highest minimum sector altitude published for `icao` (the airport's
/// own record in `earth_msa.dat`, else any record belonging to it), feet.
/// LSZS: sectors of 15,500 and 13,500 ft out to 25 nm. `None` without a record.
pub fn airport_msa_ft(msa_path: &str, icao: &str) -> Option<f64> {
    let text = std::fs::read_to_string(msa_path).ok()?;
    airport_msa_in(text.lines(), icao)
}

/// [`airport_msa_ft`] over the file's lines (pure, for tests).
pub fn airport_msa_in<'a>(lines: impl Iterator<Item = &'a str>, icao: &str) -> Option<f64> {
    let mut own: Option<f64> = None;
    let mut any: Option<f64> = None;
    for line in lines {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 8 || !f[3].eq_ignore_ascii_case(icao) {
            continue;
        }
        let mut max: Option<f64> = None;
        for t in f[5..].chunks(3) {
            if t.len() < 3 {
                break;
            }
            let (Ok(alt), Ok(radius)) = (t[1].parse::<f64>(), t[2].parse::<f64>()) else {
                break;
            };
            if alt <= 0.0 && radius <= 0.0 {
                break;
            }
            max = Some(max.unwrap_or(0.0).max(alt * 100.0));
        }
        let Some(m) = max else { continue };
        if f[1].eq_ignore_ascii_case(icao) {
            own = Some(own.unwrap_or(0.0).max(m));
        } else {
            any = Some(any.unwrap_or(0.0).max(m));
        }
    }
    own.or(any)
}

/// A minimum altitude a departure must cross a fix at or above.
#[derive(Debug, Clone, PartialEq)]
pub struct ClimbMinimum {
    pub fix: String,
    pub min_ft: f64,
}

/// The minimum altitudes of SID `sid` at `icao` for `runway`, in procedure
/// order: its runway transition (a parallel "RW26B" included), the common
/// part and any enroute transition. `+` and "at" constraints are minimums, a
/// `B` window's lower bound is; `-` constraints bound nothing from below.
/// LOWI's MOGT3H needs 7,850 ft at WI506 nine miles out, and the follower
/// flew SimBrief's predicted profile at cruise speed, about 2,200 ft under it
/// (2026-09-24).
pub fn sid_minimums(cifp_dir: &str, icao: &str, sid: &str, runway: &str) -> Vec<ClimbMinimum> {
    let path = std::path::Path::new(cifp_dir).join(format!("{icao}.dat"));
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    sid_minimums_in(text.lines(), sid, runway)
}

/// [`sid_minimums`] over the file's lines (pure, for tests).
pub fn sid_minimums_in<'a>(
    lines: impl Iterator<Item = &'a str>,
    sid: &str,
    runway: &str,
) -> Vec<ClimbMinimum> {
    let mut out: Vec<ClimbMinimum> = Vec::new();
    for line in lines {
        let Some(body) = line.strip_prefix("SID:") else {
            continue;
        };
        let f: Vec<&str> = body.split(',').collect();
        if !cifp_field(&f, 2).eq_ignore_ascii_case(sid) {
            continue;
        }
        let tr = cifp_field(&f, 3);
        let ours = match tr.strip_prefix("RW") {
            // "RW26B" serves both parallels: its number is enough.
            Some(r) if r.ends_with('B') => runway_number(r) == runway_number(runway),
            Some(r) => runway_matches(r, runway) && r.len() == runway.len(),
            None => true, // the common part, or an enroute transition
        };
        if !ours {
            continue;
        }
        let fix = cifp_field(&f, 4);
        if fix.is_empty() {
            continue;
        }
        let (a1, a2) = (arinc_alt_ft(cifp_field(&f, 23)), arinc_alt_ft(cifp_field(&f, 24)));
        let min = match cifp_field(&f, 22).chars().next().unwrap_or(' ') {
            '+' | ' ' => a1,
            'B' => a2,
            _ => None,
        };
        if let Some(min_ft) = min {
            match out.iter_mut().find(|m| m.fix == fix) {
                Some(m) => m.min_ft = m.min_ft.max(min_ft),
                None => out.push(ClimbMinimum {
                    fix: fix.to_string(),
                    min_ft,
                }),
            }
        }
    }
    out
}

/// X-Plane's grid MORA (`earth_mora.dat`): the minimum off-route altitude of
/// every one-degree cell, feet, the highest ground or obstacle in it plus
/// 1,000 ft (2,000 where that ground is above 5,000 ft). A line is "LAT LON"
/// then 30 cells eastward in hundreds of feet; LAT is the cell's SOUTH edge
/// (Everest, 27.99N 86.93E, is the 311 in row +27, column 86).
#[derive(Debug, Clone, Default)]
pub struct MoraGrid {
    cells: std::collections::HashMap<(i32, i32), f64>,
}

impl MoraGrid {
    /// Parse the file's lines. Unparseable lines are skipped.
    pub fn parse<'a>(lines: impl Iterator<Item = &'a str>) -> MoraGrid {
        let mut cells = std::collections::HashMap::new();
        for line in lines {
            let f: Vec<&str> = line.split_whitespace().collect();
            if f.len() < 3 {
                continue;
            }
            let (Ok(lat), Ok(lon0)) = (f[0].parse::<i32>(), f[1].parse::<i32>()) else {
                continue;
            };
            for (i, v) in f[2..].iter().enumerate() {
                if let Ok(h) = v.parse::<f64>() {
                    cells.insert((lat, lon0 + i as i32), h * 100.0);
                }
            }
        }
        MoraGrid { cells }
    }

    /// Load `earth_mora.dat`; `None` when it cannot be read.
    pub fn load(path: &str) -> Option<MoraGrid> {
        let text = std::fs::read_to_string(path).ok()?;
        let g = MoraGrid::parse(text.lines());
        (!g.cells.is_empty()).then_some(g)
    }

    /// The MORA of the cell holding this point, feet.
    pub fn at(&self, lat: f64, lon: f64) -> Option<f64> {
        self.cells
            .get(&(lat.floor() as i32, lon.floor() as i32))
            .copied()
    }
}

/// The highest ground a MORA stands for, feet: its 2,000 ft margin above
/// 7,000, its 1,000 below. Sea cells (1,000 or less) are sea level.
pub fn mora_ground_ft(mora_ft: f64) -> f64 {
    if mora_ft > 7_000.0 {
        mora_ft - 2_000.0
    } else {
        (mora_ft - 1_000.0).max(0.0)
    }
}
