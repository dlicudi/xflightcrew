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

use crate::xplane::XPlaneClient;

const QUERY: &str = "xcopilot/terrain/query";
const ELEV_M: &str = "xcopilot/terrain/elev_m";
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
            let v = xp.get_float_array(ELEV_M).await.ok()?;
            if v.len() < pts.len() {
                return None;
            }
            return Some(v.into_iter().take(pts.len()).map(|m| m * M_TO_FT).collect());
        }
    }
    None
}

/// The highest ground elevation (feet MSL) under a polyline; `None` = no data.
pub async fn max_elevation_ft(xp: &XPlaneClient, pts: &[(f64, f64)]) -> Option<f64> {
    let v = sample_ft(xp, pts).await?;
    v.into_iter().max_by(|a, b| a.total_cmp(b))
}
