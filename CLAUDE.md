# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
cargo run                    # Live mode (reads conf/sentrix.toml + conf/credentials.json)
cargo run -- --simulate briefs/lsgg_lfpg.txt  # Replay mode: deterministic timeline (no credentials)
cargo run -- --simulate briefs/lsgg_lfpg.txt briefs/lsgg_lszh.txt  # Multiple concurrent flights
cargo run -- --agent briefs/lsgg_lfpg.txt  # Agent mode: stateful kinematic sim (step(dt))
cargo run -- --agent briefs/lsgg_lfpg.txt --performance ~/Repos/openap/openap/data/wrap  # + OpenAP WRAP per-type performance (user-supplied, not shipped)
cargo build --release        # Optimized build
cargo test -- --ignored      # Run ignored cross-checks (need a local OpenAP checkout)
cargo test                   # Run all tests
cargo test test_parse_config # Run a single test by name
cargo check                  # Type-check without building
cargo clippy                 # Lint
```

The binary requires `conf/sentrix.toml` and OpenSky credentials at runtime. Credentials are loaded with this precedence: `conf/credentials.json` first, then `OPENSKY_CLIENT_ID` / `OPENSKY_CLIENT_SECRET` env vars.

Rust edition is **2024** (Cargo.toml) — use a recent toolchain.

## Architecture

Sentrix is a single-binary Tokio async loop with three CAT-062 sources sharing one output path:

```
live:   OpenSky REST (JSON)  --fetch_states-->  StateVector   --state_to_cat062-->        Cat062Record --encode_cat062_block--> UDP
replay: LIDO OFP briefing  --parse_briefing--> LidoBriefing  --FlightPlan::from_briefing / sampler::sample(t)--> Cat062Record --> UDP
agent:  LIDO OFP briefing  --parse_briefing--> LidoBriefing  --FlightPlan::from_briefing / Aircraft::step(dt)--> Cat062Record --> UDP
```

The binary is **pure dispatch** (`main.rs`); each CAT-062 source is its own module, and mode-agnostic code lives in `shared/`. Replay and agent modes are deliberately **independent** — they share only `shared/` (including the `FlightPlan` they both consume), never an execution loop (see `docs/SIMULATION.md`).

```
src/
  main.rs            arg parsing + dispatch (--agent / --simulate / else live)
  shared/            mode-agnostic infrastructure; depends on no mode
    config.rs        TOML config loader; also defines BoundingBox (moved here from opensky so shared owns no mode dependency)
    publisher.rs     thin UdpSocket wrapper
    lido.rs          SimBrief LIDO OFP briefing parser
    geo.rs           haversine_nm / initial_bearing_deg / destination_point / angle_diff_deg
    plan.rs          FlightPlan: briefing -> resolved route (per-leg GS/alt targets, V2/VREF profile, bearings, timeline). Consumed by replay AND agent
    cat062.rs        common CAT-062 helpers: flight_record builder, time-of-day, KNOTS_TO_MPS, track-collision remap, sim identity fallbacks
  live/              OpenSky live mode
    opensky.rs       REST client + OAuth2 client-credentials flow
    run.rs           run_live polling loop + state_to_cat062
  replay/            deterministic timeline playback of a FlightPlan
    sampler.rs       sample(&FlightPlan, elapsed) -> interpolated SimulatedState
    run.rs           run_replay loop + SimFlight + load_sim_flight
  agent/             stateful kinematic agent (independent of replay)
    aircraft.rs      Aircraft: state + plan progress + step(dt) — LNAV, GS limiter, VNAV
    performance.rs   pluggable PerformanceModel: DefaultPerformance + WrapPerformance (OpenAP WRAP loader)
    run.rs           run_agent loop + load_aircraft + 1 s sub-step integration
```

Load-bearing details:

- **`main.rs`** dispatches `--agent` / `--simulate` / live. `state_to_cat062` (live) lives in `live/run.rs`; replay and agent build records via the shared `shared::cat062::flight_record` helper. The state→`Cat062Record` conversion depends on `libasterix`, so it is kept out of `opensky.rs` / `sampler.rs` / `aircraft.rs`.
- **`live/run.rs`** — polls on `poll_interval_secs`, and on `OpenSkyError::RateLimited` sleeps for the server-provided `retry-after` (or 30s fallback) *in addition to* the normal poll interval.
- **`replay/run.rs`** — replays one or more briefings on the same interval, batching one record per flight per tick into a single CAT-062 block; flights whose Mode-S codes share a 12-bit track number are remapped onto fallback addresses with a warning (common case: briefings generated from the same SimBrief airframe). `[simulation]` config identity overrides apply in single-flight mode only.
- **`agent/aircraft.rs`** — `Aircraft::step(dt)` integrates state toward per-leg targets: LNAV (≤3°/s turn, 1 nm waypoint capture), GS limiter (≤0.7 kt/s), and **VNAV** — paces altitude at `min(required_rate, performance_limit)` so it tracks the plan where the type can and falls short honestly when the limit binds. `run_agent` integrates ~1 s sub-steps and publishes every `poll_interval_secs`.
- **`agent/performance.rs`** — climb/descent rate limits come through a `PerformanceModel` (default = flat 2000 fpm). `--performance <dir>` loads OpenAP WRAP limits from a **user-supplied** directory; OpenAP's data is **GPL-3.0** and is never vendored (sentrix is MIT, heading to AGPL+Commercial). The loader reimplements `kinematic.py::WRAP` — it is not a transliteration, and no data is shipped.
- **`shared/lido.rs`** — the FLIGHT LOG section is the only mandatory one; waypoint blocks are located by fixed-width column slices — column positions are load-bearing.
- **`live/opensky.rs`** — `TokenManager` caches the access token behind an `Arc<RwLock<...>>` and refreshes 60 s before expiry. `StateVector` has a **custom `Deserialize`** because OpenSky returns state vectors as positional JSON arrays (17 or 18 elements); the field-to-index mapping in the struct comments is load-bearing — do not reorder. `OpenSkyError` distinguishes rate limits from other failures so the live loop can back off specifically on 429.
- **`shared/publisher.rs`** — binds to `0.0.0.0:0` (ephemeral local port) and `send_to`s each ASTERIX block as one UDP datagram. No fragmentation or framing is added — the receiver parses ASTERIX block boundaries itself.

## Simulation engine: current model and direction

`replay/` is a **timeline replay**: the whole flight is precomputed and `sampler::sample(&FlightPlan, elapsed)` is a pure function of time. `agent/` is a **separate, independent stateful model** that integrates `Aircraft::step(dt)` — physics stays simple, ground-speed based, **no BADA**. Both consume the same `shared::plan::FlightPlan`.

Built so far: agent core (LNAV + GS limiter + **VNAV**) and **OpenAP Slice 1** — per-type vertical performance via the pluggable `PerformanceModel` (user-supplied WRAP data, never shipped). Still planned: turn anticipation, the agent-vs-replay parity test, **Slice 2** (CAS/Mach speed schedule), then scenarios (Phase 3) and the clearance channel (Phase 4); replay's own remaining feature is time control (Phase 1). See `docs/SIMULATION.md` for the full description, phase plan, and decisions log. Discipline: don't add features *inside* `sample(t)` — replay is frozen; new capability goes in the agent or the shared plan.

## External dependency: libasterix

CAT-062 encoding is **not** implemented in this repo — it comes from the `libasterix` crate (published separately at https://github.com/algonents/libasterix; local checkout at `~/Repos/libasterix`). When changing the conversion, the public surface used is: `Cat062Record`, `encode_cat062_block`, `icao_to_track_number`, `parse_icao_address`, `velocity_to_cartesian`. Check that crate's docs/source before assuming a field exists on `Cat062Record`.

**Known encoder gap (deferred by decision, 2026-06-12)**: `encode_cat062_record` in libasterix 0.1.0 never writes `Cat062Record.icao_address` to the wire — I062/245 carries STI + callsign only, and the 24-bit Mode-S address belongs in I062/380 subfield ADR, which the crate decodes but does not encode. All modes set `icao_address` anyway (live in `state_to_cat062`, replay/agent via `shared::cat062::flight_record`), so the address transmits as soon as libasterix gains ADR encoding. Until then, correlation runs on callsign + the 12-bit track number (low bits of the Mode-S address).

## Conventions worth knowing

- Unit conversions happen at the OpenSky boundary: `StateVector::altitude_ft()` converts metres→feet, `velocity_to_cartesian` (from libasterix) converts polar→Cartesian. Downstream code works in ASTERIX-native units.
- `track_number` is derived by hashing the 24-bit ICAO address to 12 bits (`icao_to_track_number`). Collisions are possible but accepted — this is a simulator, not an operational system.
- Records with no lat/lon are silently dropped in `state_to_cat062` (`live/run.rs`) — returns `None`, filtered by `filter_map`. A fetch of N states commonly yields fewer than N records.
