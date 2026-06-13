//! Agent-mode driver: load each brief into an `Aircraft`, remap any 12-bit
//! track-number collisions, then on every publish tick integrate the agents in
//! ~1 s sub-steps and publish one CAT-062 block for all of them.
//!
//! Independent of replay: it shares only `shared/` (plan construction, geo,
//! CAT-062 helpers, the publisher) and never the replay execution loop.

use anyhow::{Context, Result};
use chrono::Utc;
use libasterix::asterix::cat062::encode_cat062_block;

use crate::agent::aircraft::Aircraft;
use crate::agent::performance::{
    DefaultPerformance, PerformanceModel, WrapPerformance, DEFAULT_VERTICAL_RATE_FPM,
};
use crate::shared::cat062::{
    default_sim_callsign, default_sim_icao_address, flight_record, remap_track_collisions,
};
use crate::shared::config::Config;
use crate::shared::lido;
use crate::shared::plan::FlightPlan;
use crate::shared::publisher::Publisher;

/// Internal integration step. Published positions are coarser (every
/// `poll_interval_secs`), but the agent integrates finely so turns and
/// climbs stay smooth.
const SUBSTEP_S: f64 = 1.0;

/// Load one brief into an `Aircraft`, printing its summary.
///
/// Identity precedence mirrors replay: config override (single-flight mode
/// only) -> briefing (FPL callsign / Mode-S CODE) -> per-index fallback.
fn load_aircraft(
    config: &Config,
    briefing_path: &str,
    index: usize,
    single: bool,
    perf: &dyn PerformanceModel,
) -> Result<Aircraft> {
    let text = std::fs::read_to_string(briefing_path)
        .with_context(|| format!("Failed to read briefing: {}", briefing_path))?;
    let briefing = lido::parse_briefing(&text)
        .with_context(|| format!("Failed to parse OFP briefing: {}", briefing_path))?;
    let plan = FlightPlan::from_briefing(&briefing)
        .with_context(|| format!("Failed to build flight plan: {}", briefing_path))?;

    // Resolve the aircraft type's vertical-rate limits from the performance
    // model (defaults to flat constants when no WRAP data is loaded).
    let limits = perf.vertical_limits(briefing.aircraft_type.as_deref().unwrap_or(""));

    let sim = &config.simulation;
    let callsign = single
        .then(|| sim.callsign.clone())
        .flatten()
        .or(briefing.callsign)
        .unwrap_or_else(|| default_sim_callsign(index));
    let icao_address = single
        .then(|| sim.icao_address.clone())
        .flatten()
        .or(briefing.icao_address)
        .unwrap_or_else(|| default_sim_icao_address(index));

    let first = plan.points().first().unwrap();
    let last = plan.points().last().unwrap();
    println!(
        "Agent {}: {} -> {} | {} waypoints, {:.0} nm",
        callsign,
        briefing.dep_runway.as_deref().unwrap_or(&first.ident),
        briefing.arr_runway.as_deref().unwrap_or(&last.ident),
        plan.points().len(),
        plan.total_distance_nm(),
    );
    println!(
        "  Aircraft: {} ({}, {}) icao24 {}",
        callsign,
        briefing.aircraft_type.as_deref().unwrap_or("type n/a"),
        briefing.registration.as_deref().unwrap_or("reg n/a"),
        icao_address
    );

    Ok(Aircraft::new(callsign, icao_address, plan, limits))
}

/// Agent mode: fly one or more briefs as stateful kinematic agents, integrating
/// each per tick and publishing a single CAT-062 block. `performance_dir`, if
/// given, loads OpenAP WRAP per-type rate limits from that directory.
pub async fn run_agent(
    config: &Config,
    briefing_paths: &[String],
    publisher: &Publisher,
    performance_dir: Option<&str>,
) -> Result<()> {
    let perf: Box<dyn PerformanceModel> = match performance_dir {
        Some(dir) => {
            let wrap = WrapPerformance::load(dir, DEFAULT_VERTICAL_RATE_FPM)
                .with_context(|| format!("Failed to load performance data from {}", dir))?;
            println!("Performance: OpenAP WRAP from {}", dir);
            Box::new(wrap)
        }
        None => {
            println!(
                "Performance: built-in defaults ({:.0} fpm). Use --performance <dir> for OpenAP WRAP.",
                DEFAULT_VERTICAL_RATE_FPM
            );
            Box::new(DefaultPerformance::new(DEFAULT_VERTICAL_RATE_FPM))
        }
    };

    let single = briefing_paths.len() == 1;
    let sim = &config.simulation;
    if !single && (sim.callsign.is_some() || sim.icao_address.is_some()) {
        eprintln!(
            "Warning: [simulation] identity overrides apply to single-flight mode only - ignored for {} briefings",
            briefing_paths.len()
        );
    }

    let mut fleet = Vec::with_capacity(briefing_paths.len());
    for (i, briefing_path) in briefing_paths.iter().enumerate() {
        fleet.push(load_aircraft(config, briefing_path, i, single, perf.as_ref())?);
    }

    let addresses: Vec<&str> = fleet.iter().map(|a| a.icao_address.as_str()).collect();
    for (i, replacement) in remap_track_collisions(&addresses)? {
        eprintln!(
            "Warning: {} icao24 {} shares a 12-bit track number with an earlier agent - using {} instead",
            fleet[i].callsign, fleet[i].icao_address, replacement
        );
        fleet[i].icao_address = replacement;
    }

    println!(
        "\nStarting agent loop ({} aircraft)...\n",
        fleet.len(),
    );

    let interval = std::time::Duration::from_secs(config.poll_interval_secs);
    let substeps = ((config.poll_interval_secs as f64 / SUBSTEP_S).round() as u32).max(1);
    let mut announced = vec![false; fleet.len()];

    loop {
        // Publish the current state of every agent as one block.
        let records: Vec<_> = fleet
            .iter()
            .map(|ac| {
                flight_record(
                    config.asterix.sac,
                    config.asterix.sic,
                    &ac.callsign,
                    &ac.icao_address,
                    ac.lat,
                    ac.lon,
                    ac.altitude_ft,
                    ac.gs_kts,
                    ac.track_deg,
                )
            })
            .collect();
        let block = encode_cat062_block(&records);
        if let Err(e) = publisher.send(&block) {
            eprintln!("Failed to send UDP: {}", e);
        }

        for (i, ac) in fleet.iter().enumerate() {
            if ac.ended {
                if !announced[i] {
                    println!(
                        "[{}] {} arrived at {} - holding last position",
                        Utc::now().format("%H:%M:%S"),
                        ac.callsign,
                        ac.arrival_ident()
                    );
                    announced[i] = true;
                }
            } else {
                println!(
                    "[{}] {} {:8.4} {:9.4} {:5.0} ft GS {:3.0} kt trk {:03.0} -> {}",
                    Utc::now().format("%H:%M:%S"),
                    ac.callsign,
                    ac.lat,
                    ac.lon,
                    ac.altitude_ft,
                    ac.gs_kts,
                    ac.track_deg,
                    ac.target_ident().unwrap_or("-")
                );
            }
        }

        // Advance one publish interval of sim-time in fine sub-steps.
        for _ in 0..substeps {
            for ac in &mut fleet {
                ac.step(SUBSTEP_S);
            }
        }

        tokio::time::sleep(interval).await;
    }
}
