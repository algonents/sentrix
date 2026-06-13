//! The kinematic agent: an aircraft that integrates its own state toward
//! per-leg targets each `step(dt)`, following its flight plan (LNAV).
//!
//! Physics is ground-speed based (no BADA): four rate limiters cap how fast
//! track, ground speed, and altitude change, and position integrates forward
//! along the current track at the current GS. Targets (GS, altitude, the
//! lateral aim point) come from the active leg of the `FlightPlan`.

use crate::agent::performance::VerticalLimits;
use crate::shared::geo::{angle_diff_deg, destination_point, haversine_nm, initial_bearing_deg};
use crate::shared::plan::FlightPlan;

/// Standard rate turn (degrees per second).
const TURN_RATE_DEG_S: f64 = 3.0;
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
    /// Per-type climb/descent rate limits (the physical ceiling on VNAV).
    limits: VerticalLimits,
    pub ended: bool,
}

impl Aircraft {
    /// Spawn at the plan's first waypoint, pointed at the second, at the plan's
    /// initial GS (V2 once the departure profile is applied) and altitude.
    pub fn new(
        callsign: String,
        icao_address: String,
        plan: FlightPlan,
        limits: VerticalLimits,
    ) -> Self {
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
            limits,
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

        // 3. Altitude: pace toward the active fix's planned altitude (VNAV),
        //    bounded by the type's climb/descent limit. The required rate to
        //    arrive on target is `Δalt / (dist_to_fix / gs)`; we move at
        //    min(required, limit), so when the plan asks for more than the type
        //    can do, the cap binds and we fall short honestly.
        let dalt = target_alt - self.altitude_ft;
        let dist_to_fix_nm = haversine_nm(self.lat, self.lon, awp_lat, awp_lon);
        let time_to_fix_min = if self.gs_kts > 1.0 {
            dist_to_fix_nm / self.gs_kts * 60.0
        } else {
            f64::INFINITY
        };
        let required_fpm = if time_to_fix_min > 1e-6 {
            dalt.abs() / time_to_fix_min
        } else {
            f64::INFINITY
        };
        let limit_fpm = if dalt >= 0.0 {
            self.limits.climb_limit_fpm(self.altitude_ft)
        } else {
            self.limits.descent_limit_fpm(self.altitude_ft)
        };
        let max_dalt = required_fpm.min(limit_fpm) / 60.0 * dt;
        self.altitude_ft += dalt.clamp(-max_dalt, max_dalt);

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
        let mut ac = Aircraft::new("T1".into(), "4b1234".into(), plan, VerticalLimits::flat(2000.0));

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
        let mut ac = Aircraft::new("ALU".into(), "1349".into(), plan, VerticalLimits::flat(2000.0));

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
    fn test_climb_capped_by_limit() {
        // Big climb over a short leg: required rate >> the 2000 fpm limit, so
        // the agent climbs at the cap and falls short.
        let plan = FlightPlan::from_waypoints(vec![
            wp("A", 46.0, 6.0, 0.0, 250.0),
            wp("B", 46.17, 6.0, 30000.0, 250.0), // ~10 nm, +30000 ft
        ])
        .unwrap();
        let mut ac = Aircraft::new("T1".into(), "4b1234".into(), plan, VerticalLimits::flat(2000.0));

        let before = ac.altitude_ft;
        ac.step(60.0); // one minute
        let climb = ac.altitude_ft - before;
        assert!(climb <= 2000.0 + 1.0, "climbed {climb} ft in 60 s");
        assert!(climb > 1900.0, "should pin to the cap, got {climb}");
    }

    // VNAV verification at DJL: fly the real LSGG->LFPG brief and report the
    // altitude when the agent sequences past DJL (plan FL288 = 28,800 ft), with
    // the default flat cap vs real OpenAP A320 WRAP limits. Ignored (needs a
    // local OpenAP checkout; no GPL data is shipped).
    #[test]
    #[ignore = "requires a local OpenAP checkout at ~/Repos/openap"]
    fn test_djl_crossing_default_vs_wrap() {
        use crate::agent::performance::{PerformanceModel, WrapPerformance};

        let home = std::env::var("HOME").unwrap();
        let wrap =
            WrapPerformance::load(format!("{home}/Repos/openap/openap/data/wrap"), 2000.0).unwrap();
        let b = parse_briefing(include_str!("../../briefs/lsgg_lfpg.txt")).unwrap();
        let ac_type = b.aircraft_type.clone().unwrap_or_default();

        let alt_at_djl = |limits: VerticalLimits| -> f64 {
            let plan = FlightPlan::from_briefing(&b).unwrap();
            let mut ac = Aircraft::new("ALU".into(), "1349".into(), plan, limits);
            let mut was_djl = false;
            for _ in 0..6000 {
                let at_djl = ac.target_ident() == Some("DJL");
                if was_djl && !at_djl {
                    return ac.altitude_ft; // just sequenced past DJL
                }
                was_djl |= at_djl;
                ac.step(1.0);
                if ac.ended {
                    break;
                }
            }
            ac.altitude_ft
        };

        let default = alt_at_djl(VerticalLimits::flat(2000.0));
        let wrap_a320 = alt_at_djl(wrap.vertical_limits(&ac_type));
        println!(
            "DJL altitude (plan 28800 ft) — default 2000 fpm: {default:.0} ft, WRAP {ac_type}: {wrap_a320:.0} ft"
        );
    }

    #[test]
    fn test_climb_paces_to_meet_target() {
        // Gentle climb over a long leg: required rate is below the limit, so the
        // agent paces (does not slam to the cap).
        let plan = FlightPlan::from_waypoints(vec![
            wp("A", 46.0, 6.0, 10000.0, 300.0),
            wp("B", 47.0, 6.0, 12000.0, 300.0), // 60 nm, +2000 ft
        ])
        .unwrap();
        let mut ac = Aircraft::new("T1".into(), "4b1234".into(), plan, VerticalLimits::flat(3000.0));

        // 60 nm at 300 kt = 12 min; 2000 ft / 12 min = ~167 fpm, well under cap.
        let before = ac.altitude_ft;
        ac.step(60.0);
        let climb = ac.altitude_ft - before;
        assert!(climb > 100.0 && climb < 300.0, "should pace ~167 ft, got {climb}");
    }
}
