//! Common CAT-062 building blocks shared across modes: time-of-day, unit
//! conversion, and the simulated-flight identity helpers (fallback identities
//! and 12-bit track-number collision remapping). The actual state →
//! `Cat062Record` conversion stays in each mode's driver, since that depends on
//! the mode's native state type.

use anyhow::Result;
use chrono::{Timelike, Utc};
use libasterix::asterix::cat062::icao_to_track_number;

/// Knots → metres per second, for encoding velocity as Cartesian vx/vy.
pub const KNOTS_TO_MPS: f64 = 0.514444;

/// Base for fallback Mode-S addresses, used for simulated flights without an
/// ICAO flight plan section (and no config overrides). Addresses are allocated
/// sequentially per flight index so concurrent flights stay unique.
const DEFAULT_SIM_ICAO_ADDRESS_BASE: u32 = 0x4b1234;

/// Current time as seconds since midnight UTC (CAT062 I062/070 convention)
pub fn seconds_since_midnight_utc() -> f64 {
    let now = Utc::now();
    now.num_seconds_from_midnight() as f64 + (now.nanosecond() as f64 / 1_000_000_000.0)
}

/// Fallback callsign for the flight at `index`: SIM001, SIM002, ...
pub fn default_sim_callsign(index: usize) -> String {
    format!("SIM{:03}", index + 1)
}

/// Fallback Mode-S address for the flight at `index`
pub fn default_sim_icao_address(index: usize) -> String {
    format!("{:06x}", DEFAULT_SIM_ICAO_ADDRESS_BASE + index as u32)
}

/// Replacement addresses so every flight publishes a distinct 12-bit track
/// number - a shared one would corrupt downstream tracker correlation.
/// Bulletins generated from the same SimBrief airframe share a Mode-S CODE,
/// so collisions are the common case, not the exception; colliding flights
/// after the first are remapped onto the fallback address range.
///
/// Returns `(flight index, replacement icao_address)` per collision.
pub fn remap_track_collisions(icao_addresses: &[&str]) -> Result<Vec<(usize, String)>> {
    let mut used: Vec<u16> = Vec::with_capacity(icao_addresses.len());
    let mut remaps = Vec::new();
    let mut next_fallback = 0usize;

    for (i, addr) in icao_addresses.iter().enumerate() {
        let mut track = icao_to_track_number(addr);
        if used.contains(&track) {
            loop {
                // The fallback range spans all 4096 track numbers, so with
                // fewer flights than that a free one always exists.
                anyhow::ensure!(
                    next_fallback < 4096,
                    "no free 12-bit track numbers left for {}",
                    addr
                );
                let candidate = default_sim_icao_address(next_fallback);
                next_fallback += 1;
                track = icao_to_track_number(&candidate);
                if !used.contains(&track) {
                    remaps.push((i, candidate));
                    break;
                }
            }
        }
        used.push(track);
    }
    Ok(remaps)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_sim_identity_is_sequential() {
        assert_eq!(default_sim_callsign(0), "SIM001");
        assert_eq!(default_sim_callsign(1), "SIM002");
        assert_eq!(default_sim_icao_address(0), "4b1234");
        assert_eq!(default_sim_icao_address(1), "4b1235");
    }

    #[test]
    fn test_remap_track_collisions_no_collision() {
        assert!(remap_track_collisions(&[]).unwrap().is_empty());
        assert!(remap_track_collisions(&["4b1234", "4b1235"]).unwrap().is_empty());
    }

    #[test]
    fn test_remap_track_collisions_rewrites_later_flight() {
        // Identical addresses (same SimBrief airframe) and distinct addresses
        // sharing the low 12 bits both collide; the later flight is remapped
        for dup in ["4b1234", "4c1234"] {
            let remaps = remap_track_collisions(&["4b1234", dup]).unwrap();
            assert_eq!(remaps.len(), 1);
            let (i, replacement) = &remaps[0];
            assert_eq!(*i, 1);
            assert_ne!(
                icao_to_track_number(replacement),
                icao_to_track_number("4b1234")
            );
        }
    }

    #[test]
    fn test_remap_avoids_already_used_fallbacks() {
        // The first two flights already occupy the first two fallback
        // addresses; the colliding third flight must skip past both
        let remaps = remap_track_collisions(&["4b1234", "4b1235", "4b1234"]).unwrap();
        assert_eq!(remaps.len(), 1);
        let (i, replacement) = &remaps[0];
        assert_eq!(*i, 2);
        let tracks: Vec<u16> = ["4b1234", "4b1235", replacement]
            .iter()
            .map(|a| icao_to_track_number(a))
            .collect();
        assert_eq!(tracks[2], icao_to_track_number("4b1236"));
        assert_ne!(tracks[2], tracks[0]);
        assert_ne!(tracks[2], tracks[1]);
    }
}
