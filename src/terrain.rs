//! Terrain elevation from the sim itself, through the plugin's probe channel.
//!
//! The generic answer to "how does the copilot know where the Rock is": it
//! asks X-Plane. The plugin registers a probe mailbox — the app writes a list
//! of lat/lons to `xcopilot/terrain/query` ("lat,lon|lat,lon|..."), the
//! plugin's flight loop probes each point with `XPLMProbeTerrainXYZ` and
//! publishes elevations (meters MSL) in `xcopilot/terrain/elev_m`, bumping
//! `xcopilot/terrain/gen`. Same offline, works at every airport, no external
//! data, no per-airport exceptions.
//!
//! Consumers ([`crate::circuit`]'s pattern side-pick and forward guard) treat
//! `None` as "no terrain data" and keep their old behavior — an old plugin
//! without the channel degrades gracefully instead of failing the flight.
//!
//! Also the shared terrain facts the copilot and the safety loops read
//! without naming an automation: the clearance kept over the ground ahead
//! ([`CLEARANCE_FT`]), the floor the follower probes and publishes
//! ([`floor_ahead_ft`]), the points along a track it probes
//! ([`look_ahead_points`]) and the grid MORA ([`mora_grid`]). Moved out of
//! `procedure` and `plan_audit` (pillars audit 2026-10-07, knots 1 and 2).

use crate::xplane::XPlaneClient;

static MAILBOX: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

const QUERY: &str = "xcopilot/terrain/query";
const ELEV_M: &str = "xcopilot/terrain/elev_m";
const VALID: &str = "xcopilot/terrain/valid";
const GEN: &str = "xcopilot/terrain/gen";
const M_TO_FT: f64 = 3.280_84;

/// The plugin answers at most this many points per query (its buffer is one
/// 511-byte TextRef; ~20 chars per point).
pub const MAX_POINTS: usize = 24;

/// Ground elevations (feet MSL) for `pts`, in order. `None` = the plugin
/// doesn't have the channel (old build) or didn't answer in time.
pub async fn sample_ft(xp: &XPlaneClient, pts: &[(f64, f64)]) -> Option<Vec<f64>> {
    if pts.is_empty() {
        return Some(Vec::new());
    }
    // Query/generation/result are one transaction shared by all flight loops.
    let _mailbox = MAILBOX.lock().await;
    let pts = &pts[..pts.len().min(MAX_POINTS)];
    let gen0 = xp.get_float(GEN).await.ok()?;
    let q: String = pts
        .iter()
        .map(|(la, lo)| format!("{la:.5},{lo:.5}"))
        .collect::<Vec<_>>()
        .join("|");
    xp.set_data(QUERY, q.as_bytes()).await.ok()?;
    // The probe runs in the next flight loop — one frame. Poll briefly.
    for _ in 0..20 {
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        if xp.get_float(GEN).await.ok()? > gen0 + 0.5 {
            let (v, valid) = tokio::join!(xp.get_float_array(ELEV_M), xp.get_float(VALID));
            return validated_heights(v.ok()?, valid.ok()?, pts.len());
        }
    }
    None
}

/// The highest ground elevation (feet MSL) under a polyline; `None` = no data.
/// Any number of points: queried in [`MAX_POINTS`] chunks (a polyline longer
/// than one query used to be silently truncated to its first 24 samples).
pub async fn max_elevation_ft(xp: &XPlaneClient, pts: &[(f64, f64)]) -> Option<f64> {
    let mut best: Option<f64> = None;
    for chunk in pts.chunks(MAX_POINTS) {
        let v = sample_ft(xp, chunk).await?;
        for e in v {
            best = Some(best.map_or(e, |b| b.max(e)));
        }
    }
    best
}

/// Clearance the follower keeps over the highest ground AHEAD, feet. The
/// non-mountainous IFR figure, and the least an instructor would accept from
/// an automation flying a leg it cannot see.
pub const CLEARANCE_FT: f64 = 1_000.0;

/// The lowest altitude the follower may command: the highest ground within
/// its look-ahead plus [`CLEARANCE_FT`]. `None` means no terrain answer (old
/// plugin, probe timed out) and the floor is not applied: the module's
/// standing contract, degrade rather than refuse.
///
/// This exists because a plan's altitude is only as good as the ground under
/// it. The LEMG-LEJR leg to JRZ carried 2,700 ft across the Sierra de
/// Grazalema, whose ridge probes at 3,861 ft; the follower flew it down to
/// 220 ft AGL (2026-09-13). The planner floors its own CRUISE altitude over
/// this same probe, but a constraint handed down from a plan was flown as
/// given. Nothing the follower commands is trusted over the ground again.
static FLOOR_AHEAD: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// The highest-ground floor the follower is currently holding itself to, feet
/// MSL. `None` outside a procedure or with no terrain answer. Read by the
/// autopilot tool so a PILOT's altitude is judged against the same ground the
/// follower can see ahead, not merely what is under the wheels.
pub fn floor_ahead_ft() -> Option<f64> {
    if !crate::flight_state::Owner::Procedure.active() {
        return None;
    }
    *FLOOR_AHEAD.lock().unwrap_or_else(|e| e.into_inner())
}

/// The follower's one write of [`floor_ahead_ft`]: the probed floor, or
/// `None` once it stops (its exit, a retired probe, an approach).
pub(crate) fn publish_floor_ahead(v: Option<f64>) {
    *FLOOR_AHEAD.lock().unwrap_or_else(|e| e.into_inner()) = v;
}

/// Points along the ground track from `lat`/`lon` on `brg_true`, out to
/// `look_nm`, for the terrain probe. Flat-earth stepping: over 15 nm the error
/// is metres, and the answer is a maximum over samples either way.
pub fn look_ahead_points(
    lat: f64,
    lon: f64,
    brg_true: f64,
    look_nm: f64,
    n: usize,
) -> Vec<(f64, f64)> {
    let n = n.max(1);
    let b = brg_true.to_radians();
    let coslat = lat.to_radians().cos().abs().max(0.01);
    (1..=n)
        .map(|i| {
            let d = look_nm * i as f64 / n as f64;
            (
                lat + b.cos() * d / 60.0,
                lon + b.sin() * d / (60.0 * coslat),
            )
        })
        .collect()
}

/// The grid MORA, loaded once per process and shared by every reader (the
/// audit, the safe hold, the recovery planner, the local planner's terrain):
/// `earth_mora.dat` beside the fix file (Custom Data when the fixes are
/// there), else the default data's. The safe hold and the planners each
/// loaded their own copy, two of them never the Custom Data one.
pub fn mora_grid(fix_data_path: &str) -> Option<&'static crate::nav::MoraGrid> {
    use crate::nav::MoraGrid;
    static GRID: std::sync::OnceLock<Option<MoraGrid>> = std::sync::OnceLock::new();
    GRID.get_or_init(|| {
        let beside = std::path::Path::new(fix_data_path)
            .with_file_name("earth_mora.dat")
            .to_string_lossy()
            .into_owned();
        MoraGrid::load(&beside).or_else(|| {
            MoraGrid::load(&format!(
                "{}/Resources/default data/earth_mora.dat",
                crate::config::xplane_root()
            ))
        })
    })
    .as_ref()
}

fn validated_heights(values: Vec<f64>, valid: f64, wanted: usize) -> Option<Vec<f64>> {
    if !valid.is_finite()
        || valid <= 0.5
        || values.len() != wanted
        || values.iter().any(|v| !v.is_finite())
    {
        return None;
    }
    Some(values.into_iter().map(|m| m * M_TO_FT).collect())
}

#[cfg(test)]
mod tests {
    #[test]
    fn a_failed_probe_cannot_impersonate_measured_sea_level() {
        assert_eq!(super::validated_heights(vec![0.0], 1.0, 1), Some(vec![0.0]));
        assert_eq!(super::validated_heights(vec![0.0], 0.0, 1), None);
        assert_eq!(super::validated_heights(vec![0.0], 1.0, 2), None);
        assert_eq!(super::validated_heights(vec![f64::NAN], 1.0, 1), None);
    }
}
