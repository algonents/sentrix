//! Sentrix - OpenSky to ASTERIX CAT062 converter
//!
//! Fetches aircraft data from OpenSky Network and publishes it as ASTERIX CAT062 over UDP.
//! With `--simulate <flight log>...`, replays one or more SimBrief flight logs
//! concurrently instead of live data.

mod config;
mod lido;
mod opensky;
mod publisher;
mod simulation;

use anyhow::{bail, Context, Result};
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

/// Base for fallback Mode-S addresses, used for bulletins without an ICAO
/// flight plan section (and no [simulation] config overrides). Addresses are
/// allocated sequentially per flight index so concurrent flights stay unique.
const DEFAULT_SIM_ICAO24_BASE: u32 = 0x4b1234;

/// Fallback callsign for the flight at `index`: SIM001, SIM002, ...
fn default_sim_callsign(index: usize) -> String {
    format!("SIM{:03}", index + 1)
}

/// Fallback Mode-S address for the flight at `index`
fn default_sim_icao24(index: usize) -> String {
    format!("{:06x}", DEFAULT_SIM_ICAO24_BASE + index as u32)
}

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

/// Extract the flight log paths from a `--simulate <path>...` argument, if present
fn parse_simulate_arg() -> Result<Option<Vec<String>>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.iter().position(|a| a == "--simulate") {
        Some(i) => {
            let paths: Vec<String> = args[i + 1..]
                .iter()
                .take_while(|p| !p.starts_with("--"))
                .cloned()
                .collect();
            if paths.is_empty() {
                bail!("--simulate requires at least one flight log path, e.g. --simulate simulations/lsgg_lfpg.txt");
            }
            Ok(Some(paths))
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
        Some(paths) => run_simulation(&config, &paths, &publisher).await,
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

/// One replayed flight: a precomputed path plus the identity published in
/// its CAT062 records
struct SimFlight {
    path: FlightPath,
    callsign: String,
    icao24: String,
    /// Destination ident, for the arrival announcement
    arrival: String,
    holding_announced: bool,
}

/// Replacement addresses so every flight publishes a distinct 12-bit track
/// number - a shared one would corrupt downstream tracker correlation.
/// Bulletins generated from the same SimBrief airframe share a Mode-S CODE,
/// so collisions are the common case, not the exception; colliding flights
/// after the first are remapped onto the fallback address range.
///
/// Returns `(flight index, replacement icao24)` per collision.
fn remap_track_collisions(icao24s: &[&str]) -> Result<Vec<(usize, String)>> {
    let mut used: Vec<u16> = Vec::with_capacity(icao24s.len());
    let mut remaps = Vec::new();
    let mut next_fallback = 0usize;

    for (i, addr) in icao24s.iter().enumerate() {
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
                let candidate = default_sim_icao24(next_fallback);
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

/// Load one bulletin into a `SimFlight`, printing its summary.
///
/// Identity precedence: config override (single-flight mode only) -> bulletin
/// (FPL callsign / Mode-S CODE) -> per-index fallback.
fn load_sim_flight(
    config: &Config,
    log_path: &str,
    index: usize,
    single: bool,
) -> Result<SimFlight> {
    let text = std::fs::read_to_string(log_path)
        .with_context(|| format!("Failed to read flight log: {}", log_path))?;
    let bulletin = lido::parse_bulletin(&text)
        .with_context(|| format!("Failed to parse OFP bulletin: {}", log_path))?;
    let log_ete_min = bulletin.waypoints.last().and_then(|w| w.cum_time_min);
    let path = FlightPath::from_bulletin(&bulletin)
        .with_context(|| format!("Failed to build flight path: {}", log_path))?;

    let sim = &config.simulation;
    let callsign = single
        .then(|| sim.callsign.clone())
        .flatten()
        .or(bulletin.callsign)
        .unwrap_or_else(|| default_sim_callsign(index));
    let icao24 = single
        .then(|| sim.icao24.clone())
        .flatten()
        .or(bulletin.mode_s_code)
        .unwrap_or_else(|| default_sim_icao24(index));

    let first = path.points().first().unwrap();
    let last = path.points().last().unwrap();
    println!(
        "Flight {}: {} -> {} | {} waypoints, {:.0} nm, estimated {:.0} min (log ETE: {})",
        callsign,
        bulletin.dep_runway.as_deref().unwrap_or(&first.ident),
        bulletin.arr_runway.as_deref().unwrap_or(&last.ident),
        path.points().len(),
        path.total_distance_nm(),
        path.total_duration_s() / 60.0,
        log_ete_min.map_or("n/a".to_string(), |m| format!("{} min", m))
    );
    println!(
        "  Aircraft: {} ({}, {}) icao24 {}",
        callsign,
        bulletin.aircraft_type.as_deref().unwrap_or("type n/a"),
        bulletin.registration.as_deref().unwrap_or("reg n/a"),
        icao24
    );
    match (bulletin.v2_kts, bulletin.vref_kts) {
        (Some(v2), Some(vref)) => {
            println!("  Speed profile: V2 {:.0} kt, VREF {:.0} kt", v2, vref)
        }
        _ => println!("  Speed profile: n/a (no runway analysis in bulletin)"),
    }

    let arrival = last.ident.clone();
    Ok(SimFlight {
        path,
        callsign,
        icao24,
        arrival,
        holding_announced: false,
    })
}

/// Simulation mode: replay one or more SimBrief LIDO OFP bulletins as CAT062.
/// All flights share one timeline and their records are batched into a single
/// CAT062 block per tick.
async fn run_simulation(config: &Config, log_paths: &[String], publisher: &Publisher) -> Result<()> {
    let single = log_paths.len() == 1;
    let sim = &config.simulation;
    if !single && (sim.callsign.is_some() || sim.icao24.is_some()) {
        eprintln!(
            "Warning: [simulation] identity overrides apply to single-flight mode only - ignored for {} flight logs",
            log_paths.len()
        );
    }

    let mut flights = Vec::with_capacity(log_paths.len());
    for (i, log_path) in log_paths.iter().enumerate() {
        flights.push(load_sim_flight(config, log_path, i, single)?);
    }

    let addresses: Vec<&str> = flights.iter().map(|f| f.icao24.as_str()).collect();
    for (i, replacement) in remap_track_collisions(&addresses)? {
        eprintln!(
            "Warning: {} icao24 {} shares a 12-bit track number with an earlier flight - using {} instead",
            flights[i].callsign, flights[i].icao24, replacement
        );
        flights[i].icao24 = replacement;
    }

    println!(
        "\nStarting simulation loop ({} flight{})...\n",
        flights.len(),
        if flights.len() == 1 { "" } else { "s" }
    );

    let interval = std::time::Duration::from_secs(config.poll_interval_secs);
    let start = std::time::Instant::now();

    loop {
        let elapsed = start.elapsed().as_secs_f64();
        let mut records = Vec::with_capacity(flights.len());

        for flight in &mut flights {
            let state = flight.path.sample(elapsed);

            let mut record = Cat062Record::new(config.asterix.sac, config.asterix.sic);
            record.track_number = icao_to_track_number(&flight.icao24);
            record.time_of_day = seconds_since_midnight_utc();
            record.latitude = state.lat;
            record.longitude = state.lon;
            record.altitude_ft = Some(state.alt_ft.round() as i32);
            let (vx, vy) = velocity_to_cartesian(state.gs_kts * KNOTS_TO_MS, state.track_deg);
            record.vx = Some(vx);
            record.vy = Some(vy);
            // Mode-S address from the FPL CODE/ item - downstream systems use this
            // (with the callsign) for flight plan correlation
            record.icao_address = parse_icao_address(&flight.icao24);
            record.callsign = Some(flight.callsign.clone());
            record.track_status = 0x00;
            records.push(record);

            if state.ended {
                if !flight.holding_announced {
                    println!(
                        "[{}] {} arrived at {} - holding last position",
                        Utc::now().format("%H:%M:%S"),
                        flight.callsign,
                        flight.arrival
                    );
                    flight.holding_announced = true;
                }
            } else {
                println!(
                    "[{}] {} {:8.4} {:9.4} {:5.0} ft GS {:3.0} kt TAS {:3.0} kt trk {:03.0} -> {}",
                    Utc::now().format("%H:%M:%S"),
                    flight.callsign,
                    state.lat,
                    state.lon,
                    state.alt_ft,
                    state.gs_kts,
                    state.tas_kts,
                    state.track_deg,
                    state.next_ident.as_deref().unwrap_or("-")
                );
            }
        }

        let block = encode_cat062_block(&records);
        if let Err(e) = publisher.send(&block) {
            eprintln!("Failed to send UDP: {}", e);
        }

        tokio::time::sleep(interval).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_sim_identity_is_sequential() {
        assert_eq!(default_sim_callsign(0), "SIM001");
        assert_eq!(default_sim_callsign(1), "SIM002");
        assert_eq!(default_sim_icao24(0), "4b1234");
        assert_eq!(default_sim_icao24(1), "4b1235");
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
