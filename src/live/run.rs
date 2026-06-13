//! Live-mode driver: poll OpenSky on `poll_interval_secs`, convert each state
//! vector to a CAT-062 record, and publish the batch over UDP. On a 429 it
//! backs off for the server-provided `retry-after` (or 30 s) before the next
//! poll.

use anyhow::{Context, Result};
use chrono::Utc;
use libasterix::asterix::cat062::{
    encode_cat062_block, icao_to_track_number, parse_icao_address, velocity_to_cartesian,
    Cat062Record,
};

use crate::live::opensky::{fetch_states, Credentials, StateVector, TokenManager};
use crate::shared::cat062::seconds_since_midnight_utc;
use crate::shared::config::Config;
use crate::shared::publisher::Publisher;

/// Convert an OpenSky `StateVector` to a CAT-062 record. Returns `None` (and is
/// filtered out) when the state carries no position.
fn state_to_cat062(state: &StateVector, sac: u8, sic: u8) -> Option<Cat062Record> {
    // Skip if no position data
    let lat = state.latitude()?;
    let lon = state.longitude()?;

    let mut record = Cat062Record::new(sac, sic);

    // Track number from ICAO address (hashed to 12-bit)
    record.track_number = icao_to_track_number(state.icao_address());

    // Time of day (seconds since midnight UTC)
    record.time_of_day = if let Some(tp) = state.time_position() {
        // Convert Unix timestamp to seconds since midnight UTC
        (tp % 86400) as f64
    } else {
        seconds_since_midnight_utc()
    };

    // Position
    record.latitude = lat;
    record.longitude = lon;

    // Altitude
    record.altitude_ft = state.altitude_ft();

    // Velocity (convert polar to cartesian if available)
    if let (Some(speed), Some(heading)) = (state.velocity_mps(), state.true_track()) {
        let (vx, vy) = velocity_to_cartesian(speed, heading);
        record.vx = Some(vx);
        record.vy = Some(vy);
    }

    // Target identification
    record.icao_address = parse_icao_address(state.icao_address());
    record.callsign = state.callsign().map(|s| s.to_string());

    // Track status (basic: confirmed track)
    record.track_status = 0x00;

    Some(record)
}

/// Live mode: poll OpenSky and republish as CAT062
pub async fn run_live(config: &Config, publisher: &Publisher) -> Result<()> {
    println!(
        "Bounding box: lat [{}, {}], lon [{}, {}]",
        config.bounding_box.min_lat,
        config.bounding_box.max_lat,
        config.bounding_box.min_lon,
        config.bounding_box.max_lon
    );

    // Load credentials
    let (credentials, source) = Credentials::load()
        .context("Failed to load OpenSky credentials. Set OPENSKY_CLIENT_ID and OPENSKY_CLIENT_SECRET or create conf/credentials.json")?;
    println!("Credentials loaded from {}", source);

    // Initialize token manager
    let token_manager = TokenManager::new(credentials);

    // Initialize HTTP client
    let http_client = reqwest::Client::builder()
        .user_agent("sentrix/0.1.0")
        .build()
        .context("Failed to create HTTP client")?;

    println!("\nStarting main loop...\n");

    // Main polling loop
    let poll_interval = std::time::Duration::from_secs(config.poll_interval_secs);

    loop {
        match fetch_states(&http_client, &config.bounding_box, Some(&token_manager)).await {
            Ok(states) => {
                let record_count = states.len();

                // Convert to CAT062 records
                let records: Vec<Cat062Record> = states
                    .iter()
                    .filter_map(|s| state_to_cat062(s, config.asterix.sac, config.asterix.sic))
                    .collect();

                if !records.is_empty() {
                    // Encode and send
                    let block = encode_cat062_block(&records);
                    match publisher.send(&block) {
                        Ok(bytes_sent) => {
                            println!(
                                "[{}] Sent {} records ({} bytes) from {} states",
                                Utc::now().format("%H:%M:%S"),
                                records.len(),
                                bytes_sent,
                                record_count
                            );
                        }
                        Err(e) => {
                            eprintln!("Failed to send UDP: {}", e);
                        }
                    }
                } else {
                    println!(
                        "[{}] No valid records (fetched {} states)",
                        Utc::now().format("%H:%M:%S"),
                        record_count
                    );
                }
            }
            Err(e) => {
                eprintln!("Failed to fetch states: {}", e);
                if e.is_rate_limited() {
                    let backoff = e.retry_after_secs().unwrap_or(30);
                    eprintln!("Rate limited, backing off for {}s", backoff);
                    tokio::time::sleep(std::time::Duration::from_secs(backoff)).await;
                }
            }
        }

        tokio::time::sleep(poll_interval).await;
    }
}
