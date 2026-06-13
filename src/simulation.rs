//! Flight path replay
//!
//! Turns the waypoint list from a SimBrief flight log into a time-indexed
//! path and interpolates aircraft state along it.
//!
//! The timeline is derived from leg distance / average ground speed rather
//! than the log's TTLT column: TTLT is minute-resolution, so consecutive
//! waypoints can share the same timestamp, which would create zero-duration
//! segments.

use anyhow::{bail, Result};

use crate::lido::{LidoBulletin, Waypoint};

const EARTH_RADIUS_NM: f64 = 3440.065;

/// ATC speed limit below FL100, used as the acceleration/deceleration target
/// near the airports
const MAX_LOW_ALT_SPEED_KTS: f64 = 250.0;
/// Altitude per track mile on final approach (~3 degree glide path)
const APPROACH_GLIDE_FT_PER_NM: f64 = 318.0;
/// Distance over which the departure accelerates from V2 towards 250 kt
const DEPARTURE_ACCEL_DIST_NM: f64 = 2.0;
/// Deceleration stages on final: (distance to destination nm, target GS kts);
/// the VREF placeholder is filled in at runtime
const ARRIVAL_DECEL_DIST_NM: [f64; 3] = [15.0, 10.0, 4.0];

/// A waypoint with all values resolved and a position on the timeline
#[derive(Debug, Clone)]
pub struct PathPoint {
    pub ident: String,
    pub lat: f64,
    pub lon: f64,
    pub altitude_ft: f64,
    pub gs_kts: f64,
    pub tas_kts: f64,
    /// Seconds from departure
    pub time_s: f64,
    /// Great-circle bearing towards the next waypoint, degrees
    pub track_deg: f64,
}

/// Interpolated aircraft state at a point in time
#[derive(Debug, Clone)]
pub struct SimulatedState {
    pub lat: f64,
    pub lon: f64,
    pub altitude_ft: f64,
    pub gs_kts: f64,
    pub tas_kts: f64,
    pub track_deg: f64,
    /// Next waypoint ahead (None once the destination is reached)
    pub next_ident: Option<String>,
    /// True once the destination has been reached
    pub ended: bool,
}

#[derive(Debug)]
pub struct FlightPath {
    points: Vec<PathPoint>,
}

impl FlightPath {
    /// Build a path from a parsed bulletin, applying the takeoff/approach
    /// speed profile when the bulletin provides V2/VREF. Geometry is never
    /// altered — synthetic points lie on the existing legs.
    pub fn from_bulletin(bulletin: &LidoBulletin) -> Result<Self> {
        let mut waypoints = bulletin.waypoints.clone();
        apply_departure_profile(&mut waypoints, bulletin.v2_kts);
        apply_arrival_profile(&mut waypoints, bulletin.vref_kts);
        Self::from_waypoints(waypoints)
    }

    pub fn from_waypoints(waypoints: Vec<Waypoint>) -> Result<Self> {
        if waypoints.len() < 2 {
            bail!("flight path needs at least 2 waypoints, got {}", waypoints.len());
        }

        let mut gs: Vec<Option<f64>> = waypoints.iter().map(|w| w.gs_kts).collect();
        fill_gaps(&mut gs);
        if gs[0].is_none() {
            bail!("flight log contains no ground speeds - cannot build a timeline");
        }

        // Airports at the route ends carry no FL in the log; treat them as
        // 0 ft so the profile climbs from / descends to the surface.
        let mut alt: Vec<Option<f64>> = waypoints.iter().map(|w| w.altitude_ft).collect();
        if alt[0].is_none() {
            alt[0] = Some(0.0);
        }
        if alt.last().unwrap().is_none() {
            *alt.last_mut().unwrap() = Some(0.0);
        }
        fill_gaps(&mut alt);

        let mut points: Vec<PathPoint> = Vec::with_capacity(waypoints.len());
        let mut time_s = 0.0;
        for (i, w) in waypoints.iter().enumerate() {
            let gs_kts = gs[i].unwrap();

            if i > 0 {
                let prev = &waypoints[i - 1];
                let dist_nm = haversine_nm(prev.lat, prev.lon, w.lat, w.lon);
                let avg_gs = (gs[i - 1].unwrap() + gs_kts) / 2.0;
                if avg_gs > 0.0 {
                    time_s += dist_nm / avg_gs * 3600.0;
                }
            }

            let track_deg = if i + 1 < waypoints.len() {
                let next = &waypoints[i + 1];
                initial_bearing_deg(w.lat, w.lon, next.lat, next.lon)
            } else {
                points.last().map(|p| p.track_deg).unwrap_or(0.0)
            };

            points.push(PathPoint {
                ident: w.ident.clone(),
                lat: w.lat,
                lon: w.lon,
                altitude_ft: alt[i].unwrap(),
                gs_kts,
                // TAS is only printed on cruise rows in the log; elsewhere,
                // estimate it as GS minus the wind component (GS itself as
                // the last resort).
                tas_kts: w
                    .tas_kts
                    .unwrap_or(gs_kts - w.wind_comp_kts.unwrap_or(0.0)),
                time_s,
                track_deg,
            });
        }

        Ok(Self { points })
    }

    pub fn points(&self) -> &[PathPoint] {
        &self.points
    }

    /// Total flight duration in seconds
    pub fn total_duration_s(&self) -> f64 {
        self.points.last().map(|p| p.time_s).unwrap_or(0.0)
    }

    /// Total great-circle route length in nautical miles
    pub fn total_distance_nm(&self) -> f64 {
        self.points
            .windows(2)
            .map(|w| haversine_nm(w[0].lat, w[0].lon, w[1].lat, w[1].lon))
            .sum()
    }

    /// Interpolated state at `elapsed_s` seconds after departure.
    ///
    /// Past the destination, returns the final position with zero speed.
    pub fn sample(&self, elapsed_s: f64) -> SimulatedState {
        let pts = &self.points;
        let last = pts.last().unwrap();
        let t = elapsed_s.max(0.0);

        if t >= last.time_s {
            return SimulatedState {
                lat: last.lat,
                lon: last.lon,
                altitude_ft: last.altitude_ft,
                gs_kts: 0.0,
                tas_kts: 0.0,
                track_deg: last.track_deg,
                next_ident: None,
                ended: true,
            };
        }

        // Find the segment containing t (paths are ~20 points, linear is fine)
        let mut i = 0;
        while i + 1 < pts.len() && pts[i + 1].time_s <= t {
            i += 1;
        }
        let (a, b) = (&pts[i], &pts[i + 1]);

        let duration = b.time_s - a.time_s;
        let f = if duration > 0.0 { (t - a.time_s) / duration } else { 1.0 };
        let lerp = |x: f64, y: f64| x + (y - x) * f;

        SimulatedState {
            lat: lerp(a.lat, b.lat),
            lon: lerp(a.lon, b.lon),
            altitude_ft: lerp(a.altitude_ft, b.altitude_ft),
            gs_kts: lerp(a.gs_kts, b.gs_kts),
            tas_kts: lerp(a.tas_kts, b.tas_kts),
            track_deg: a.track_deg,
            next_ident: Some(b.ident.clone()),
            ended: false,
        }
    }
}

/// Departure speed profile: lift off at V2 and accelerate towards the
/// low-altitude limit over the first miles, instead of inheriting the first
/// waypoint's climb GS at the runway.
fn apply_departure_profile(waypoints: &mut Vec<Waypoint>, v2_kts: Option<f64>) {
    let Some(v2) = v2_kts else { return };
    if waypoints.len() < 2 {
        return;
    }
    let (dep, next) = (waypoints[0].clone(), waypoints[1].clone());
    waypoints[0].gs_kts = Some(v2);

    let leg_nm = haversine_nm(dep.lat, dep.lon, next.lat, next.lon);
    if leg_nm <= DEPARTURE_ACCEL_DIST_NM * 1.5 {
        return;
    }
    let f = DEPARTURE_ACCEL_DIST_NM / leg_nm;
    let next_gs = next.gs_kts.unwrap_or(MAX_LOW_ALT_SPEED_KTS);
    waypoints.insert(
        1,
        Waypoint {
            ident: "CLIMB".to_string(),
            lat: dep.lat + (next.lat - dep.lat) * f,
            lon: dep.lon + (next.lon - dep.lon) * f,
            altitude_ft: Some(next.altitude_ft.unwrap_or(0.0) * f),
            tas_kts: None,
            gs_kts: Some(next_gs.min(MAX_LOW_ALT_SPEED_KTS)),
            wind_comp_kts: None,
            cum_time_min: None,
        },
    );
}

/// Arrival speed profile: decelerate on the final leg down to VREF at the
/// destination, with altitude capped to a ~3 degree glide near the runway.
fn apply_arrival_profile(waypoints: &mut Vec<Waypoint>, vref_kts: Option<f64>) {
    let Some(vref) = vref_kts else { return };
    let n = waypoints.len();
    if n < 2 {
        return;
    }
    let (prev, dest) = (waypoints[n - 2].clone(), waypoints[n - 1].clone());
    waypoints[n - 1].gs_kts = Some(vref);
    if waypoints[n - 1].altitude_ft.is_none() {
        waypoints[n - 1].altitude_ft = Some(0.0);
    }

    let leg_nm = haversine_nm(prev.lat, prev.lon, dest.lat, dest.lon);
    let prev_gs = prev.gs_kts.unwrap_or(MAX_LOW_ALT_SPEED_KTS);
    let prev_alt = prev.altitude_ft.unwrap_or(0.0);
    let stage_gs = [
        prev_gs.min(MAX_LOW_ALT_SPEED_KTS),
        200.0_f64.min(prev_gs),
        (vref + 15.0).min(prev_gs),
    ];

    let mut insert_at = n - 1;
    for (&dist_nm, gs) in ARRIVAL_DECEL_DIST_NM.iter().zip(stage_gs) {
        if dist_nm >= leg_nm * 0.9 {
            continue; // final leg too short for this stage
        }
        let f = (leg_nm - dist_nm) / leg_nm;
        let alt_linear = prev_alt * dist_nm / leg_nm;
        waypoints.insert(
            insert_at,
            Waypoint {
                ident: format!("FIN{:02.0}", dist_nm),
                lat: prev.lat + (dest.lat - prev.lat) * f,
                lon: prev.lon + (dest.lon - prev.lon) * f,
                altitude_ft: Some(alt_linear.min(APPROACH_GLIDE_FT_PER_NM * dist_nm)),
                tas_kts: None,
                gs_kts: Some(gs),
                wind_comp_kts: None,
                cum_time_min: None,
            },
        );
        insert_at += 1;
    }
}

/// Forward-fill then back-fill missing values from their neighbours
fn fill_gaps(values: &mut [Option<f64>]) {
    let mut last = None;
    for v in values.iter_mut() {
        if v.is_some() {
            last = *v;
        } else {
            *v = last;
        }
    }
    let mut next = None;
    for v in values.iter_mut().rev() {
        if v.is_some() {
            next = *v;
        } else {
            *v = next;
        }
    }
}

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lido::{parse_bulletin, parse_flight_log};

    fn wp(
        ident: &str,
        lat: f64,
        lon: f64,
        alt: Option<f64>,
        tas: Option<f64>,
        gs: Option<f64>,
    ) -> Waypoint {
        Waypoint {
            ident: ident.to_string(),
            lat,
            lon,
            altitude_ft: alt,
            tas_kts: tas,
            gs_kts: gs,
            wind_comp_kts: None,
            cum_time_min: None,
        }
    }

    #[test]
    fn test_straight_leg_interpolation() {
        // 60 nm due north at a constant 360 kt -> 600 s
        let path = FlightPath::from_waypoints(vec![
            wp("A", 46.0, 6.0, Some(10000.0), Some(400.0), Some(360.0)),
            wp("B", 47.0, 6.0, Some(20000.0), Some(400.0), Some(360.0)),
        ])
        .unwrap();

        assert!((path.total_duration_s() - 600.0).abs() < 5.0);

        let mid = path.sample(path.total_duration_s() / 2.0);
        assert!((mid.lat - 46.5).abs() < 0.01);
        assert!((mid.lon - 6.0).abs() < 1e-9);
        assert!((mid.altitude_ft - 15000.0).abs() < 100.0);
        assert!((mid.gs_kts - 360.0).abs() < 1e-9);
        assert!(mid.track_deg < 1.0 || mid.track_deg > 359.0); // due north
        assert_eq!(mid.next_ident.as_deref(), Some("B"));
        assert!(!mid.ended);
    }

    #[test]
    fn test_holds_last_position_after_arrival() {
        let path = FlightPath::from_waypoints(vec![
            wp("A", 46.0, 6.0, Some(5000.0), None, Some(300.0)),
            wp("B", 46.5, 6.0, Some(0.0), None, Some(300.0)),
        ])
        .unwrap();

        let end = path.sample(path.total_duration_s() + 100.0);
        assert!(end.ended);
        assert_eq!(end.gs_kts, 0.0);
        assert!((end.lat - 46.5).abs() < 1e-9);
        assert_eq!(end.next_ident, None);
    }

    #[test]
    fn test_fills_missing_values() {
        // Middle waypoint (e.g. a FIR boundary) has no altitude/speeds
        let path = FlightPath::from_waypoints(vec![
            wp("A", 46.0, 6.0, Some(10000.0), Some(420.0), Some(400.0)),
            wp("FIR", 46.5, 6.0, None, None, None),
            wp("B", 47.0, 6.0, Some(10000.0), Some(420.0), Some(400.0)),
        ])
        .unwrap();

        let fir = &path.points()[1];
        assert_eq!(fir.gs_kts, 400.0);
        assert_eq!(fir.altitude_ft, 10000.0);
        assert_eq!(fir.tas_kts, 400.0); // missing TAS falls back to GS
    }

    #[test]
    fn test_real_flight_log() {
        let wps = parse_flight_log(include_str!("../simulations/lsgg_lfpg.txt")).unwrap();
        let path = FlightPath::from_waypoints(wps).unwrap();

        // Log says 238 nm and 45 min block-to-block; distance/GS timing
        // should land in the same ballpark.
        assert!((path.total_distance_nm() - 238.0).abs() < 10.0);
        let mins = path.total_duration_s() / 60.0;
        assert!((30.0..60.0).contains(&mins), "unexpected duration: {mins} min");

        // Starts at LSGG, ends at LFPG
        let start = path.sample(0.0);
        assert!((start.lat - 46.2383).abs() < 0.01);
        assert!((start.lon - 6.11).abs() < 0.01);

        let end = path.sample(path.total_duration_s());
        assert!(end.ended);
        assert!((end.lat - 49.01).abs() < 0.01);
        assert!((end.lon - 2.5483).abs() < 0.01);

        // Mid-flight the aircraft is at cruise FL300 heading roughly north-west
        let mid = path.sample(path.total_duration_s() * 0.5);
        assert!((mid.altitude_ft - 30000.0).abs() < 1.0);
        assert!((270.0..360.0).contains(&mid.track_deg));
    }

    #[test]
    fn test_tas_estimated_from_wind_component() {
        let mut a = wp("A", 46.0, 6.0, Some(17600.0), None, Some(365.0));
        a.wind_comp_kts = Some(-38.0); // 38 kt headwind -> TAS = GS + 38
        let path = FlightPath::from_waypoints(vec![
            a,
            wp("B", 47.0, 6.0, Some(17600.0), None, Some(365.0)),
        ])
        .unwrap();
        assert_eq!(path.points()[0].tas_kts, 403.0);
    }

    #[test]
    fn test_bulletin_speed_profile() {
        let b = parse_bulletin(include_str!("../simulations/lsgg_lfpg.txt")).unwrap();
        let path = FlightPath::from_bulletin(&b).unwrap();

        // Departure: lifts off at V2, not at the first waypoint's climb GS
        let start = path.sample(0.0);
        assert!((start.gs_kts - 154.0).abs() < 1.0, "gs = {}", start.gs_kts);

        // Short final: below 200 kt and on the glide, decelerating to VREF
        let end_t = path.total_duration_s();
        let short_final = path.sample(end_t - 30.0);
        assert!(short_final.gs_kts < 200.0, "gs = {}", short_final.gs_kts);
        assert!(short_final.altitude_ft < 2000.0, "alt = {}", short_final.altitude_ft);

        // With the profile, total duration approaches the plan's 45 min ETE
        let mins = end_t / 60.0;
        assert!((40.0..50.0).contains(&mins), "duration: {mins} min");

        // Without V2/VREF the path is unchanged (and faster)
        let plain = FlightPath::from_waypoints(b.waypoints.clone()).unwrap();
        assert!(plain.total_duration_s() < end_t);
    }
}
