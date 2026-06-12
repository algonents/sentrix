//! Sentrix - OpenSky to ASTERIX CAT062 converter
//!
//! Fetches aircraft data from OpenSky Network and publishes it as ASTERIX CAT062 over UDP.
//! With `--simulate <flight log>`, replays a SimBrief flight log instead of live data.

mod config;
mod lido;
mod opensky;
mod publisher;
mod simulation;

use anyhow::{Context, Result};
use chrono::{Timelike, Utc};
use libasterix::asterix::cat062::{
    encode_cat062_block, icao_to_track_number, parse_icao_address, velocity_to_cartesian,
    Cat062Record,
};

use crate::config::Config;
use crate::opensky::{fetch_states, Credentials, StateVector, TokenManager};
use crate::publisher::Publisher;
use crate::simulation::FlightPath;

const KNOTS_TO_MS: f64 = 0.514444;

/// Simulated-aircraft identity fallbacks for bulletins without an ICAO
/// flight plan section (and no [simulation] config overrides)
const DEFAULT_SIM_CALLSIGN: &str = "SIM001";
const DEFAULT_SIM_ICAO24: &str = "4b1234";

/// Current time as seconds since midnight UTC (CAT062 I062/070 convention)
fn seconds_since_midnight_utc() -> f64 {
    let now = Utc::now();
    now.num_seconds_from_midnight() as f64 + (now.nanosecond() as f64 / 1_000_000_000.0)
}

/// Convert OpenSky StateVector to CAT062 record
fn state_to_cat062(state: &StateVector, sac: u8, sic: u8) -> Option<Cat062Record> {
    // Skip if no position data
    let lat = state.latitude()?;
    let lon = state.longitude()?;

    let mut record = Cat062Record::new(sac, sic);

    // Track number from ICAO address (hashed to 12-bit)
    record.track_number = icao_to_track_number(state.icao24());

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
    record.altitude_ft = state.altitude_feet();

    // Velocity (convert polar to cartesian if available)
    if let (Some(speed), Some(heading)) = (state.velocity_ms(), state.true_track()) {
        let (vx, vy) = velocity_to_cartesian(speed, heading);
        record.vx = Some(vx);
        record.vy = Some(vy);
    }

    // Target identification
    record.icao_address = parse_icao_address(state.icao24());
    record.callsign = state.callsign().map(|s| s.to_string());

    // Track status (basic: confirmed track)
    record.track_status = 0x00;

    Some(record)
}

/// Extract the flight log path from a `--simulate <path>` argument, if present
fn parse_simulate_arg() -> Result<Option<String>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.iter().position(|a| a == "--simulate") {
        Some(i) => {
            let path = args
                .get(i + 1)
                .filter(|p| !p.starts_with("--"))
                .cloned()
                .context("--simulate requires a flight log path, e.g. --simulate simulations/lsgg_lfpg.txt")?;
            Ok(Some(path))
        }
        None => Ok(None),
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    println!("Sentrix - OpenSky to ASTERIX CAT062 converter");
    println!("============================================");

    let sim_log = parse_simulate_arg()?;

    // Load configuration
    let config = Config::load().context("Failed to load configuration")?;
    println!(
        "Configuration loaded: poll every {}s, SAC={} SIC={}",
        config.poll_interval_secs, config.asterix.sac, config.asterix.sic
    );

    // Initialize UDP publisher
    let publisher = Publisher::new(&config.udp.destination)?;
    println!("UDP publisher ready: -> {}", config.udp.destination);

    match sim_log {
        Some(path) => run_simulation(&config, &path, &publisher).await,
        None => run_live(&config, &publisher).await,
    }
}

/// Live mode: poll OpenSky and republish as CAT062
async fn run_live(config: &Config, publisher: &Publisher) -> Result<()> {
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

/// Simulation mode: replay a SimBrief LIDO OFP bulletin as CAT062
async fn run_simulation(config: &Config, log_path: &str, publisher: &Publisher) -> Result<()> {
    let text = std::fs::read_to_string(log_path)
        .with_context(|| format!("Failed to read flight log: {}", log_path))?;
    let bulletin = lido::parse_bulletin(&text)
        .with_context(|| format!("Failed to parse OFP bulletin: {}", log_path))?;
    let log_ete_min = bulletin.waypoints.last().and_then(|w| w.cum_time_min);
    let path = FlightPath::from_bulletin(&bulletin)?;

    // Identity: config override -> bulletin (FPL callsign / Mode-S CODE) -> fallback
    let sim = &config.simulation;
    let callsign = sim
        .callsign
        .clone()
        .or(bulletin.callsign)
        .unwrap_or_else(|| DEFAULT_SIM_CALLSIGN.to_string());
    let icao24 = sim
        .icao24
        .clone()
        .or(bulletin.mode_s_code)
        .unwrap_or_else(|| DEFAULT_SIM_ICAO24.to_string());

    let first = path.points().first().unwrap();
    let last = path.points().last().unwrap();
    println!(
        "Simulation: {} -> {} | {} waypoints, {:.0} nm, estimated {:.0} min (log ETE: {})",
        bulletin.dep_runway.as_deref().unwrap_or(&first.ident),
        bulletin.arr_runway.as_deref().unwrap_or(&last.ident),
        path.points().len(),
        path.total_distance_nm(),
        path.total_duration_s() / 60.0,
        log_ete_min.map_or("n/a".to_string(), |m| format!("{} min", m))
    );
    println!(
        "Aircraft: {} ({}, {}) icao24 {}",
        callsign,
        bulletin.aircraft_type.as_deref().unwrap_or("type n/a"),
        bulletin.registration.as_deref().unwrap_or("reg n/a"),
        icao24
    );
    match (bulletin.v2_kts, bulletin.vref_kts) {
        (Some(v2), Some(vref)) => {
            println!("Speed profile: V2 {:.0} kt, VREF {:.0} kt", v2, vref)
        }
        _ => println!("Speed profile: n/a (no runway analysis in bulletin)"),
    }

    println!("\nStarting simulation loop...\n");

    let interval = std::time::Duration::from_secs(config.poll_interval_secs);
    let start = std::time::Instant::now();
    let mut holding_announced = false;

    loop {
        let state = path.sample(start.elapsed().as_secs_f64());

        let mut record = Cat062Record::new(config.asterix.sac, config.asterix.sic);
        record.track_number = icao_to_track_number(&icao24);
        record.time_of_day = seconds_since_midnight_utc();
        record.latitude = state.lat;
        record.longitude = state.lon;
        record.altitude_ft = Some(state.alt_ft.round() as i32);
        let (vx, vy) = velocity_to_cartesian(state.gs_kts * KNOTS_TO_MS, state.track_deg);
        record.vx = Some(vx);
        record.vy = Some(vy);
        // Mode-S address from the FPL CODE/ item - downstream systems use this
        // (with the callsign) for flight plan correlation
        record.icao_address = parse_icao_address(&icao24);
        record.callsign = Some(callsign.clone());
        record.track_status = 0x00;

        let block = encode_cat062_block(&[record]);
        match publisher.send(&block) {
            Ok(bytes_sent) => {
                if state.ended {
                    if !holding_announced {
                        println!(
                            "[{}] {} arrived at {} - holding last position",
                            Utc::now().format("%H:%M:%S"),
                            callsign,
                            last.ident
                        );
                        holding_announced = true;
                    }
                } else {
                    println!(
                        "[{}] {} {:8.4} {:9.4} {:5.0} ft GS {:3.0} kt TAS {:3.0} kt trk {:03.0} -> {} ({} bytes)",
                        Utc::now().format("%H:%M:%S"),
                        callsign,
                        state.lat,
                        state.lon,
                        state.alt_ft,
                        state.gs_kts,
                        state.tas_kts,
                        state.track_deg,
                        state.next_ident.as_deref().unwrap_or("-"),
                        bytes_sent
                    );
                }
            }
            Err(e) => eprintln!("Failed to send UDP: {}", e),
        }

        tokio::time::sleep(interval).await;
    }
}
