//! Great-circle helpers.

/// Initial great-circle bearing from point 1 to point 2, degrees true 0..360.
pub fn bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dl = (lon2 - lon1).to_radians();
    let y = dl.sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

/// Great-circle distance in nautical miles between two lat/lon points (haversine).
pub fn haversine_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R_NM: f64 = 3440.065;
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dphi = (lat2 - lat1).to_radians();
    let dlam = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dlam / 2.0).sin().powi(2);
    2.0 * R_NM * a.sqrt().asin()
}

/// A point `dist_nm` from (lat, lon) along true `bearing` (flat-earth good enough
/// at approach ranges).
pub fn project_latlon(lat: f64, lon: f64, bearing: f64, dist_nm: f64) -> (f64, f64) {
    let d = dist_nm / 60.0; // nm → degrees of latitude
    let b = bearing.to_radians();
    let dlat = d * b.cos();
    let dlon = d * b.sin() / lat.to_radians().cos().max(1e-6);
    (lat + dlat, lon + dlon)
}
