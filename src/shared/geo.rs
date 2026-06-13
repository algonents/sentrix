//! Great-circle geometry helpers, in nautical miles and degrees.

const EARTH_RADIUS_NM: f64 = 3440.065;

/// Great-circle distance in nautical miles
pub fn haversine_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (phi1, phi2) = (lat1.to_radians(), lat2.to_radians());
    let dphi = (lat2 - lat1).to_radians();
    let dlambda = (lon2 - lon1).to_radians();
    let a = (dphi / 2.0).sin().powi(2) + phi1.cos() * phi2.cos() * (dlambda / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_NM * a.sqrt().asin()
}

/// Initial great-circle bearing from point 1 to point 2, degrees [0, 360)
pub fn initial_bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (phi1, phi2) = (lat1.to_radians(), lat2.to_radians());
    let dlambda = (lon2 - lon1).to_radians();
    let y = dlambda.sin() * phi2.cos();
    let x = phi1.cos() * phi2.sin() - phi1.sin() * phi2.cos() * dlambda.cos();
    (y.atan2(x).to_degrees() + 360.0) % 360.0
}

/// Destination point reached from `(lat, lon)` flying `bearing_deg` for
/// `dist_nm` along a great circle. The forward geodesic — used by the agent to
/// integrate position each step.
pub fn destination_point(lat: f64, lon: f64, bearing_deg: f64, dist_nm: f64) -> (f64, f64) {
    let delta = dist_nm / EARTH_RADIUS_NM; // angular distance
    let theta = bearing_deg.to_radians();
    let phi1 = lat.to_radians();
    let lambda1 = lon.to_radians();
    let phi2 = (phi1.sin() * delta.cos() + phi1.cos() * delta.sin() * theta.cos()).asin();
    let lambda2 = lambda1
        + (theta.sin() * delta.sin() * phi1.cos()).atan2(delta.cos() - phi1.sin() * phi2.sin());
    (phi2.to_degrees(), lambda2.to_degrees())
}

/// Signed smallest angle from `from_deg` to `to_deg`, in (-180, 180].
/// Positive = turn right (clockwise).
pub fn angle_diff_deg(from_deg: f64, to_deg: f64) -> f64 {
    let d = (to_deg - from_deg).rem_euclid(360.0);
    if d > 180.0 { d - 360.0 } else { d }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_destination_roundtrips_with_distance_and_bearing() {
        // From LSGG, fly 045° for 10 nm; distance and bearing back out.
        let (lat, lon) = (46.2383, 6.11);
        let (lat2, lon2) = destination_point(lat, lon, 45.0, 10.0);
        assert!((haversine_nm(lat, lon, lat2, lon2) - 10.0).abs() < 0.01);
        assert!((initial_bearing_deg(lat, lon, lat2, lon2) - 45.0).abs() < 0.1);
    }

    #[test]
    fn test_angle_diff_sign_and_wrap() {
        assert!((angle_diff_deg(10.0, 40.0) - 30.0).abs() < 1e-9); // right
        assert!((angle_diff_deg(40.0, 10.0) + 30.0).abs() < 1e-9); // left
        assert!((angle_diff_deg(350.0, 10.0) - 20.0).abs() < 1e-9); // wrap right
        assert!((angle_diff_deg(10.0, 350.0) + 20.0).abs() < 1e-9); // wrap left
    }
}
