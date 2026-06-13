//! The kinematic agent: an aircraft that integrates its own state toward
//! per-leg targets each `step(dt)`, following its flight plan (LNAV).
//!
//! Physics is ground-speed based (no BADA): four rate limiters cap how fast
//! track, ground speed, and altitude change, and position integrates forward
//! along the current track at the current GS. Targets (GS, altitude, the
//! lateral aim point) come from the active leg of the `FlightPlan`.

use crate::shared::geo::{angle_diff_deg, destination_point, haversine_nm, initial_bearing_deg};
use crate::shared::plan::FlightPlan;

/// Standard rate turn (degrees per second).
const TURN_RATE_DEG_S: f64 = 3.0;
/// Maximum vertical rate (feet per minute) toward the target altitude.
const MAX_VS_FPM: f64 = 2000.0;
/// Ground-speed change cap (knots per second).
const GS_ACCEL_KT_S: f64 = 0.7;
/// Sequence to the next leg once within this distance of the active waypoint.
const WAYPOINT_CAPTURE_NM: f64 = 1.0;
/// Overshoot guard: once we've approached within this and start receding, the
/// waypoint is behind us — sequence even if we never hit the capture radius.
const CLOSE_APPROACH_NM: f64 = 3.0;

/// A single simulated aircraft flying its plan.
pub struct Aircraft {
    pub callsign: String,
    pub icao_address: String,
    // live state
    pub lat: f64,
    pub lon: f64,
    pub altitude_ft: f64,
    pub gs_kts: f64,
    pub track_deg: f64,
    // plan + progress
    plan: FlightPlan,
    /// Index of the active (target) waypoint in `plan.points()`.
    leg: usize,
    /// Closest approach to the active waypoint seen this leg (overshoot guard).
    closest_nm: f64,
    pub ended: bool,
}

impl Aircraft {
    /// Spawn at the plan's first waypoint, pointed at the second, at the plan's
    /// initial GS (V2 once the departure profile is applied) and altitude.
    pub fn new(callsign: String, icao_address: String, plan: FlightPlan) -> Self {
        let p0 = &plan.points()[0];
        let (lat, lon, altitude_ft, gs_kts, track_deg) =
            (p0.lat, p0.lon, p0.altitude_ft, p0.gs_kts, p0.track_deg);
        Aircraft {
            callsign,
            icao_address,
            lat,
            lon,
            altitude_ft,
            gs_kts,
            track_deg,
            plan,
            leg: 1,
            closest_nm: f64::INFINITY,
            ended: false,
        }
    }

    /// Ident of the waypoint currently being flown toward (None once arrived).
    pub fn target_ident(&self) -> Option<&str> {
        if self.ended {
            None
        } else {
            Some(self.plan.points()[self.leg].ident.as_str())
        }
    }

    /// Ident of the final waypoint (destination).
    pub fn arrival_ident(&self) -> &str {
        self.plan.points().last().unwrap().ident.as_str()
    }

    /// Advance the aircraft by `dt` seconds: aim at the active waypoint, apply
    /// the four rate limiters, integrate position, and sequence the plan.
    pub fn step(&mut self, dt: f64) {
        if self.ended {
            self.gs_kts = 0.0;
            return;
        }

        // Copy what we need from the active leg, releasing the borrow on `plan`.
        let n_points = self.plan.points().len();
        let (awp_lat, awp_lon, target_gs, target_alt) = {
            let a = &self.plan.points()[self.leg];
            (a.lat, a.lon, a.gs_kts, a.altitude_ft)
        };

        // 1. Track toward the aim point, capped at standard rate.
        let desired_track = initial_bearing_deg(self.lat, self.lon, awp_lat, awp_lon);
        let turn = angle_diff_deg(self.track_deg, desired_track);
        let max_turn = TURN_RATE_DEG_S * dt;
        self.track_deg = (self.track_deg + turn.clamp(-max_turn, max_turn)).rem_euclid(360.0);

        // 2. Ground speed toward target.
        let max_dgs = GS_ACCEL_KT_S * dt;
        self.gs_kts += (target_gs - self.gs_kts).clamp(-max_dgs, max_dgs);

        // 3. Altitude toward target, capped at the vertical-rate limit.
        let max_dalt = MAX_VS_FPM / 60.0 * dt;
        self.altitude_ft += (target_alt - self.altitude_ft).clamp(-max_dalt, max_dalt);

        // 4. Integrate position forward along the (new) track at GS.
        let dist_nm = self.gs_kts * dt / 3600.0;
        let (lat, lon) = destination_point(self.lat, self.lon, self.track_deg, dist_nm);
        self.lat = lat;
        self.lon = lon;

        // LNAV: sequence to the next leg on capture or overshoot.
        let d = haversine_nm(self.lat, self.lon, awp_lat, awp_lon);
        self.closest_nm = self.closest_nm.min(d);
        let captured = d < WAYPOINT_CAPTURE_NM;
        let passed = self.closest_nm < CLOSE_APPROACH_NM && d > self.closest_nm + 0.3;
        if captured || passed {
            self.leg += 1;
            self.closest_nm = f64::INFINITY;
            if self.leg >= n_points {
                self.ended = true;
                self.gs_kts = 0.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shared::lido::{parse_briefing, Waypoint};

    fn wp(ident: &str, lat: f64, lon: f64, alt: f64, gs: f64) -> Waypoint {
        Waypoint {
            ident: ident.to_string(),
            lat,
            lon,
            altitude_ft: Some(alt),
            tas_kts: None,
            gs_kts: Some(gs),
            wind_comp_kts: None,
            cum_time_min: None,
        }
    }

    #[test]
    fn test_flies_straight_leg_and_arrives() {
        // ~60 nm due north at 360 kt; integrate to arrival.
        let plan = FlightPlan::from_waypoints(vec![
            wp("A", 46.0, 6.0, 10000.0, 360.0),
            wp("B", 47.0, 6.0, 10000.0, 360.0),
        ])
        .unwrap();
        let mut ac = Aircraft::new("T1".into(), "4b1234".into(), plan);

        // Heading roughly north at spawn.
        assert!(ac.track_deg < 1.0 || ac.track_deg > 359.0);

        for _ in 0..1200 {
            // 1 s steps, well past the ~600 s leg
            ac.step(1.0);
            if ac.ended {
                break;
            }
        }
        assert!(ac.ended);
        assert!((ac.lat - 47.0).abs() < 0.05, "lat = {}", ac.lat);
        assert!((ac.lon - 6.0).abs() < 0.05, "lon = {}", ac.lon);
    }

    #[test]
    fn test_real_brief_flies_to_destination() {
        // Fly the whole LSGG->LFPG route (21 waypoints, turns, climb, descent)
        // and confirm LNAV sequences every leg and arrives near LFPG without
        // getting stuck.
        let b = parse_briefing(include_str!("../../briefs/lsgg_lfpg.txt")).unwrap();
        let plan = FlightPlan::from_briefing(&b).unwrap();
        let mut ac = Aircraft::new("ALU".into(), "1349".into(), plan);

        for _ in 0..5400 {
            // 1 s steps, up to 90 min — well past the ~45 min flight
            ac.step(1.0);
            if ac.ended {
                break;
            }
        }
        assert!(ac.ended, "agent never arrived (LNAV stuck?)");
        assert!((ac.lat - 49.01).abs() < 0.1, "lat = {}", ac.lat);
        assert!((ac.lon - 2.5483).abs() < 0.1, "lon = {}", ac.lon);
    }

    #[test]
    fn test_climbs_toward_target_within_rate() {
        let plan = FlightPlan::from_waypoints(vec![
            wp("A", 46.0, 6.0, 0.0, 250.0),
            wp("B", 47.5, 6.0, 30000.0, 250.0),
        ])
        .unwrap();
        let mut ac = Aircraft::new("T1".into(), "4b1234".into(), plan);

        let before = ac.altitude_ft;
        ac.step(60.0); // one minute
        let climb = ac.altitude_ft - before;
        // capped at MAX_VS_FPM per minute
        assert!(climb <= MAX_VS_FPM + 1.0, "climbed {climb} ft in 60 s");
        assert!(climb > 0.0);
    }
}
