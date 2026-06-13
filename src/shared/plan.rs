//! Flight plan: the route built from a briefing.
//!
//! A briefing's waypoint list resolved into per-leg targets (GS, altitude),
//! great-circle bearings, and a distance÷GS timeline. This is mode-agnostic
//! input infrastructure: **replay** samples the plan as a function of time, the
//! **agent** executes it via `step(dt)`. Construction lives here so both modes
//! fly the exact same route (and the agent-vs-replay parity test is meaningful).
//!
//! The timeline (`time_s`) is derived from leg distance / average ground speed
//! rather than the log's TTLT column: TTLT is minute-resolution, so consecutive
//! waypoints can share the same timestamp, which would create zero-duration
//! segments.

use anyhow::{bail, Result};

use crate::shared::geo::{haversine_nm, initial_bearing_deg};
use crate::shared::lido::{LidoBriefing, Waypoint};

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
pub struct PlanPoint {
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

/// The route an aircraft flies: resolved waypoints with per-leg targets.
#[derive(Debug)]
pub struct FlightPlan {
    points: Vec<PlanPoint>,
}

impl FlightPlan {
    /// Build a plan from a parsed briefing, applying the takeoff/approach
    /// speed profile when the briefing provides V2/VREF. Geometry is never
    /// altered — synthetic points lie on the existing legs.
    pub fn from_briefing(briefing: &LidoBriefing) -> Result<Self> {
        let mut waypoints = briefing.waypoints.clone();
        apply_departure_profile(&mut waypoints, briefing.v2_kts);
        apply_arrival_profile(&mut waypoints, briefing.vref_kts);
        Self::from_waypoints(waypoints)
    }

    pub fn from_waypoints(waypoints: Vec<Waypoint>) -> Result<Self> {
        if waypoints.len() < 2 {
            bail!("flight plan needs at least 2 waypoints, got {}", waypoints.len());
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

        let mut points: Vec<PlanPoint> = Vec::with_capacity(waypoints.len());
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

            points.push(PlanPoint {
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

    pub fn points(&self) -> &[PlanPoint] {
        &self.points
    }

    /// Total flight duration in seconds (from the distance÷GS timeline)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shared::lido::parse_flight_log;

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
    fn test_fills_missing_values() {
        // Middle waypoint (e.g. a FIR boundary) has no altitude/speeds
        let plan = FlightPlan::from_waypoints(vec![
            wp("A", 46.0, 6.0, Some(10000.0), Some(420.0), Some(400.0)),
            wp("FIR", 46.5, 6.0, None, None, None),
            wp("B", 47.0, 6.0, Some(10000.0), Some(420.0), Some(400.0)),
        ])
        .unwrap();

        let fir = &plan.points()[1];
        assert_eq!(fir.gs_kts, 400.0);
        assert_eq!(fir.altitude_ft, 10000.0);
        assert_eq!(fir.tas_kts, 400.0); // missing TAS falls back to GS
    }

    #[test]
    fn test_tas_estimated_from_wind_component() {
        let mut a = wp("A", 46.0, 6.0, Some(17600.0), None, Some(365.0));
        a.wind_comp_kts = Some(-38.0); // 38 kt headwind -> TAS = GS + 38
        let plan = FlightPlan::from_waypoints(vec![
            a,
            wp("B", 47.0, 6.0, Some(17600.0), None, Some(365.0)),
        ])
        .unwrap();
        assert_eq!(plan.points()[0].tas_kts, 403.0);
    }

    #[test]
    fn test_real_brief_geometry() {
        let wps = parse_flight_log(include_str!("../../briefs/lsgg_lfpg.txt")).unwrap();
        let plan = FlightPlan::from_waypoints(wps).unwrap();

        // Log says 238 nm and 45 min block-to-block; distance/GS timing
        // should land in the same ballpark.
        assert!((plan.total_distance_nm() - 238.0).abs() < 10.0);
        let mins = plan.total_duration_s() / 60.0;
        assert!((30.0..60.0).contains(&mins), "unexpected duration: {mins} min");
    }
}
