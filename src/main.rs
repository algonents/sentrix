//! Sentrix - OpenSky to ASTERIX CAT062 converter
//!
//! Publishes aircraft state as ASTERIX CAT062 over UDP. Independent sources
//! share only `shared`: **live** mode polls the OpenSky Network, **replay**
//! mode (`--simulate <brief>...`) replays SimBrief briefs as a precomputed
//! timeline, and **agent** mode (`--agent <brief>...`) flies them as stateful
//! kinematic agents.

mod agent;
mod live;
mod replay;
mod shared;

use anyhow::{bail, Context, Result};

use crate::agent::run::run_agent;
use crate::live::run::run_live;
use crate::replay::run::run_replay;
use crate::shared::config::Config;
use crate::shared::publisher::Publisher;

/// Extract the brief paths following `flag` (e.g. `--simulate`), if present.
fn parse_brief_paths(flag: &str) -> Result<Option<Vec<String>>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.iter().position(|a| a == flag) {
        Some(i) => {
            let paths: Vec<String> = args[i + 1..]
                .iter()
                .take_while(|p| !p.starts_with("--"))
                .cloned()
                .collect();
            if paths.is_empty() {
                bail!("{flag} requires at least one brief path, e.g. {flag} briefs/lsgg_lfpg.txt");
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

    let agent_paths = parse_brief_paths("--agent")?;
    let replay_paths = parse_brief_paths("--simulate")?;

    // Load configuration
    let config = Config::load().context("Failed to load configuration")?;
    println!(
        "Configuration loaded: poll every {}s, SAC={} SIC={}",
        config.poll_interval_secs, config.asterix.sac, config.asterix.sic
    );

    // Initialize UDP publisher
    let publisher = Publisher::new(&config.udp.destination)?;
    println!("UDP publisher ready: -> {}", config.udp.destination);

    match (agent_paths, replay_paths) {
        (Some(paths), _) => run_agent(&config, &paths, &publisher).await,
        (None, Some(paths)) => run_replay(&config, &paths, &publisher).await,
        (None, None) => run_live(&config, &publisher).await,
    }
}
