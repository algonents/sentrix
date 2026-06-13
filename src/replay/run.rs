//! Replay-mode driver: load each bulletin into a `SimFlight`, remap any
//! 12-bit track-number collisions, then on every tick sample each flight's
//! `FlightPath`, build one CAT-062 record per flight, and publish the batch as
//! a single block.

use anyhow::{Context, Result};
use chrono::Utc;
use libasterix::asterix::cat062::{
    encode_cat062_block, icao_to_track_number, parse_icao_address, velocity_to_cartesian,
    Cat062Record,
};

use crate::replay::flight_path::FlightPath;
use crate::shared::cat062::{
    default_sim_callsign, default_sim_icao_address, remap_track_collisions,
    seconds_since_midnight_utc, KNOTS_TO_MPS,
};
use crate::shared::config::Config;
use crate::shared::lido;
use crate::shared::publisher::Publisher;

/// One replayed flight: a precomputed path plus the identity published in
/// its CAT062 records
struct SimFlight {
    path: FlightPath,
    callsign: String,
    icao_address: String,
    /// Destination ident, for the arrival announcement
    arrival: String,
    holding_announced: bool,
}

/// Load one bulletin into a `SimFlight`, printing its summary.
///
/// Identity precedence: config override (single-flight mode only) -> bulletin
/// (FPL callsign / Mode-S CODE) -> per-index fallback.
fn load_sim_flight(
    config: &Config,
    bulletin_path: &str,
    index: usize,
    single: bool,
) -> Result<SimFlight> {
    let text = std::fs::read_to_string(bulletin_path)
        .with_context(|| format!("Failed to read bulletin: {}", bulletin_path))?;
    let bulletin = lido::parse_bulletin(&text)
        .with_context(|| format!("Failed to parse OFP bulletin: {}", bulletin_path))?;
    let log_ete_min = bulletin.waypoints.last().and_then(|w| w.cum_time_min);
    let path = FlightPath::from_bulletin(&bulletin)
        .with_context(|| format!("Failed to build flight path: {}", bulletin_path))?;

    let sim = &config.simulation;
    let callsign = single
        .then(|| sim.callsign.clone())
        .flatten()
        .or(bulletin.callsign)
        .unwrap_or_else(|| default_sim_callsign(index));
    let icao_address = single
        .then(|| sim.icao_address.clone())
        .flatten()
        .or(bulletin.icao_address)
        .unwrap_or_else(|| default_sim_icao_address(index));

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
        icao_address
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
        icao_address,
        arrival,
        holding_announced: false,
    })
}

/// Replay mode: replay one or more SimBrief LIDO OFP bulletins as CAT062.
/// All flights share one timeline and their records are batched into a single
/// CAT062 block per tick.
pub async fn run_replay(
    config: &Config,
    bulletin_paths: &[String],
    publisher: &Publisher,
) -> Result<()> {
    let single = bulletin_paths.len() == 1;
    let sim = &config.simulation;
    if !single && (sim.callsign.is_some() || sim.icao_address.is_some()) {
        eprintln!(
            "Warning: [simulation] identity overrides apply to single-flight mode only - ignored for {} bulletins",
            bulletin_paths.len()
        );
    }

    let mut flights = Vec::with_capacity(bulletin_paths.len());
    for (i, bulletin_path) in bulletin_paths.iter().enumerate() {
        flights.push(load_sim_flight(config, bulletin_path, i, single)?);
    }

    let addresses: Vec<&str> = flights.iter().map(|f| f.icao_address.as_str()).collect();
    for (i, replacement) in remap_track_collisions(&addresses)? {
        eprintln!(
            "Warning: {} icao24 {} shares a 12-bit track number with an earlier flight - using {} instead",
            flights[i].callsign, flights[i].icao_address, replacement
        );
        flights[i].icao_address = replacement;
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
            record.track_number = icao_to_track_number(&flight.icao_address);
            record.time_of_day = seconds_since_midnight_utc();
            record.latitude = state.lat;
            record.longitude = state.lon;
            record.altitude_ft = Some(state.altitude_ft.round() as i32);
            let (vx, vy) = velocity_to_cartesian(state.gs_kts * KNOTS_TO_MPS, state.track_deg);
            record.vx = Some(vx);
            record.vy = Some(vy);
            // Mode-S address from the FPL CODE/ item - downstream systems use this
            // (with the callsign) for flight plan correlation
            record.icao_address = parse_icao_address(&flight.icao_address);
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
                    state.altitude_ft,
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
