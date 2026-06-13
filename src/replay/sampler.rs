//! Replay execution: interpolate a `FlightPlan` as a pure function of elapsed
//! time. This is replay's whole execution model — given the plan and a clock,
//! look up where the aircraft is. The plan itself is built in `shared::plan`.

use crate::shared::plan::FlightPlan;

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

/// Interpolated state at `elapsed_s` seconds after departure.
///
/// Past the destination, returns the final position with zero speed.
pub fn sample(plan: &FlightPlan, elapsed_s: f64) -> SimulatedState {
    let pts = plan.points();
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shared::lido::{parse_briefing, parse_flight_log, Waypoint};

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
        let plan = FlightPlan::from_waypoints(vec![
            wp("A", 46.0, 6.0, Some(10000.0), Some(400.0), Some(360.0)),
            wp("B", 47.0, 6.0, Some(20000.0), Some(400.0), Some(360.0)),
        ])
        .unwrap();

        assert!((plan.total_duration_s() - 600.0).abs() < 5.0);

        let mid = sample(&plan, plan.total_duration_s() / 2.0);
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
        let plan = FlightPlan::from_waypoints(vec![
            wp("A", 46.0, 6.0, Some(5000.0), None, Some(300.0)),
            wp("B", 46.5, 6.0, Some(0.0), None, Some(300.0)),
        ])
        .unwrap();

        let end = sample(&plan, plan.total_duration_s() + 100.0);
        assert!(end.ended);
        assert_eq!(end.gs_kts, 0.0);
        assert!((end.lat - 46.5).abs() < 1e-9);
        assert_eq!(end.next_ident, None);
    }

    #[test]
    fn test_real_flight_log() {
        let wps = parse_flight_log(include_str!("../../briefs/lsgg_lfpg.txt")).unwrap();
        let plan = FlightPlan::from_waypoints(wps).unwrap();

        // Starts at LSGG, ends at LFPG
        let start = sample(&plan, 0.0);
        assert!((start.lat - 46.2383).abs() < 0.01);
        assert!((start.lon - 6.11).abs() < 0.01);

        let end = sample(&plan, plan.total_duration_s());
        assert!(end.ended);
        assert!((end.lat - 49.01).abs() < 0.01);
        assert!((end.lon - 2.5483).abs() < 0.01);

        // Mid-flight the aircraft is at cruise FL300 heading roughly north-west
        let mid = sample(&plan, plan.total_duration_s() * 0.5);
        assert!((mid.altitude_ft - 30000.0).abs() < 1.0);
        assert!((270.0..360.0).contains(&mid.track_deg));
    }

    #[test]
    fn test_briefing_speed_profile() {
        let b = parse_briefing(include_str!("../../briefs/lsgg_lfpg.txt")).unwrap();
        let plan = FlightPlan::from_briefing(&b).unwrap();

        // Departure: lifts off at V2, not at the first waypoint's climb GS
        let start = sample(&plan, 0.0);
        assert!((start.gs_kts - 154.0).abs() < 1.0, "gs = {}", start.gs_kts);

        // Short final: below 200 kt and on the glide, decelerating to VREF
        let end_t = plan.total_duration_s();
        let short_final = sample(&plan, end_t - 30.0);
        assert!(short_final.gs_kts < 200.0, "gs = {}", short_final.gs_kts);
        assert!(short_final.altitude_ft < 2000.0, "alt = {}", short_final.altitude_ft);

        // With the profile, total duration approaches the plan's 45 min ETE
        let mins = end_t / 60.0;
        assert!((40.0..50.0).contains(&mins), "duration: {mins} min");

        // Without V2/VREF the plan is unchanged (and faster)
        let plain = FlightPlan::from_waypoints(b.waypoints.clone()).unwrap();
        assert!(plain.total_duration_s() < end_t);
    }
}
