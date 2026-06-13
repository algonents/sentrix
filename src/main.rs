//! Sentrix - OpenSky to ASTERIX CAT062 converter
//!
//! Publishes aircraft state as ASTERIX CAT062 over UDP. Two independent
//! sources share only `shared`: **live** mode polls the OpenSky Network, and
//! **replay** mode (`--simulate <bulletin>...`) replays one or more SimBrief
//! OFP bulletins concurrently. A future **agent** mode will live alongside them.

mod agent;
mod live;
mod replay;
mod shared;

use anyhow::{bail, Context, Result};

use crate::live::run::run_live;
use crate::replay::run::run_replay;
use crate::shared::config::Config;
use crate::shared::publisher::Publisher;

/// Extract the bulletin paths from a `--simulate <path>...` argument, if present
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
                bail!("--simulate requires at least one bulletin path, e.g. --simulate simulations/lsgg_lfpg.txt");
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

    let bulletin_paths = parse_simulate_arg()?;

    // Load configuration
    let config = Config::load().context("Failed to load configuration")?;
    println!(
        "Configuration loaded: poll every {}s, SAC={} SIC={}",
        config.poll_interval_secs, config.asterix.sac, config.asterix.sic
    );

    // Initialize UDP publisher
    let publisher = Publisher::new(&config.udp.destination)?;
    println!("UDP publisher ready: -> {}", config.udp.destination);

    match bulletin_paths {
        Some(paths) => run_replay(&config, &paths, &publisher).await,
        None => run_live(&config, &publisher).await,
    }
}
