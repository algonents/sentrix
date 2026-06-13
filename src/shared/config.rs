//! Configuration module for Sentrix
//!
//! Loads settings from TOML configuration file.

use anyhow::{Context, Result};
use serde::Deserialize;
use std::path::Path;

/// Geographic bounding box for filtering aircraft. Defined here (not in the
/// live/OpenSky client) so the shared config layer owns no mode-specific
/// dependency; the live client imports it from here for its query shape.
#[derive(Debug, Clone, Deserialize)]
pub struct BoundingBox {
    pub min_lat: f64,
    pub max_lat: f64,
    pub min_lon: f64,
    pub max_lon: f64,
}

/// Main configuration structure
#[derive(Debug, Deserialize)]
pub struct Config {
    /// Polling interval in seconds
    #[serde(default = "default_poll_interval")]
    pub poll_interval_secs: u64,

    /// Bounding box for geographic filtering
    pub bounding_box: BoundingBox,

    /// ASTERIX data source identifier
    pub asterix: AsterixConfig,

    /// UDP output configuration
    pub udp: UdpConfig,

    /// Simulation mode settings (used with --simulate)
    #[serde(default)]
    pub simulation: SimulationConfig,
}

/// ASTERIX-specific configuration
#[derive(Debug, Deserialize)]
pub struct AsterixConfig {
    /// System Area Code
    pub sac: u8,
    /// System Identification Code
    pub sic: u8,
}

/// UDP output configuration
#[derive(Debug, Deserialize)]
pub struct UdpConfig {
    /// Destination address (IP:port)
    pub destination: String,
}

/// Identity overrides for the simulated aircraft in simulation mode.
///
/// When unset, the values from the OFP briefing are used (ATC callsign and
/// Mode-S CODE from the ICAO flight plan), with hardcoded fallbacks for
/// briefings that lack a flight plan section.
#[derive(Debug, Default, Deserialize)]
pub struct SimulationConfig {
    /// Callsign published in the CAT062 target identification
    #[serde(default)]
    pub callsign: Option<String>,
    /// 24-bit Mode-S address as a hex string (also seeds the track number)
    #[serde(default)]
    pub icao_address: Option<String>,
}

fn default_poll_interval() -> u64 {
    10
}

impl Config {
    /// Load configuration from a TOML file
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let contents = std::fs::read_to_string(path)
            .with_context(|| format!("Failed to read config file: {}", path.display()))?;

        toml::from_str(&contents)
            .with_context(|| format!("Failed to parse config file: {}", path.display()))
    }

    /// Load configuration from the default location (conf/sentrix.toml)
    pub fn load() -> Result<Self> {
        Self::from_file("conf/sentrix.toml")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_config() {
        let toml = r#"
poll_interval_secs = 15

[bounding_box]
min_lat = 45.0
max_lat = 55.0
min_lon = -5.0
max_lon = 15.0

[asterix]
sac = 1
sic = 2

[udp]
destination = "127.0.0.1:4000"
"#;

        let config: Config = toml::from_str(toml).unwrap();
        assert_eq!(config.poll_interval_secs, 15);
        assert_eq!(config.asterix.sac, 1);
        assert_eq!(config.asterix.sic, 2);
        assert_eq!(config.udp.destination, "127.0.0.1:4000");
        // [simulation] section is optional; unset values defer to the briefing
        assert_eq!(config.simulation.callsign, None);
        assert_eq!(config.simulation.icao_address, None);
    }

    #[test]
    fn test_parse_simulation_config() {
        let toml = r#"
[bounding_box]
min_lat = 45.0
max_lat = 55.0
min_lon = -5.0
max_lon = 15.0

[asterix]
sac = 1
sic = 2

[udp]
destination = "127.0.0.1:4000"

[simulation]
callsign = "SWR123"
icao_address = "4b17e5"
"#;

        let config: Config = toml::from_str(toml).unwrap();
        assert_eq!(config.simulation.callsign.as_deref(), Some("SWR123"));
        assert_eq!(config.simulation.icao_address.as_deref(), Some("4b17e5"));
    }
}
