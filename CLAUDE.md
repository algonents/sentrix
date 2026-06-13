# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
cargo run                    # Live mode (reads conf/sentrix.toml + conf/credentials.json)
cargo run -- --simulate simulations/lsgg_lfpg.txt  # Simulation mode (no credentials needed)
cargo run -- --simulate simulations/lsgg_lfpg.txt simulations/lsgg_lszh.txt  # Multiple concurrent flights
cargo build --release        # Optimized build
cargo test                   # Run all tests
cargo test test_parse_config # Run a single test by name
cargo check                  # Type-check without building
cargo clippy                 # Lint
```

The binary requires `conf/sentrix.toml` and OpenSky credentials at runtime. Credentials are loaded with this precedence: `conf/credentials.json` first, then `OPENSKY_CLIENT_ID` / `OPENSKY_CLIENT_SECRET` env vars.

Rust edition is **2024** (Cargo.toml) — use a recent toolchain.

## Architecture

Sentrix is a single-binary Tokio async loop with two modes sharing one output path:

```
live:  OpenSky REST (JSON)  --fetch_states-->  StateVector  --state_to_cat062-->  Cat062Record  --encode_cat062_block-->  UDP bytes
replay: LIDO OFP bulletin  --parse_bulletin-->  LidoBulletin  --FlightPath::from_bulletin / sample-->  Cat062Record  --encode_cat062_block-->  UDP bytes
```

The binary is **pure dispatch** (`main.rs`); each CAT-062 source is its own module, and mode-agnostic code lives in `shared/`. Replay mode and the future agent mode are deliberately **independent** — they share only `shared/`, never an execution loop (see `docs/SIMULATION.md`).

```
src/
  main.rs            arg parsing + dispatch (run_replay on --simulate, else run_live)
  shared/            mode-agnostic infrastructure; depends on no mode
    config.rs        TOML config loader; also defines BoundingBox (moved here from opensky so shared owns no mode dependency)
    publisher.rs     thin UdpSocket wrapper
    lido.rs          SimBrief LIDO OFP bulletin parser
    geo.rs           haversine_nm / initial_bearing_deg
    cat062.rs        common CAT-062 helpers: seconds_since_midnight_utc, KNOTS_TO_MPS, track-collision remap, sim identity fallbacks
  live/              OpenSky live mode
    opensky.rs       REST client + OAuth2 client-credentials flow
    run.rs           run_live polling loop + state_to_cat062
  replay/            deterministic bulletin playback
    flight_path.rs   FlightPath + sample(elapsed)
    run.rs           run_replay loop + SimFlight + load_sim_flight
  agent/             placeholder for the future stateful agent engine (empty)
```

Load-bearing details:

- **`main.rs`** dispatches to `run_replay` (`--simulate <bulletin>...`) or `run_live`. Both `state_to_cat062` (live) and the replay record-building stay in their mode's `run.rs` because they depend on `libasterix` types; keeping them out of `opensky.rs` / `flight_path.rs` avoids coupling those to ASTERIX concepts.
- **`live/run.rs`** — polls on `poll_interval_secs`, and on `OpenSkyError::RateLimited` sleeps for the server-provided `retry-after` (or 30s fallback) *in addition to* the normal poll interval.
- **`replay/run.rs`** — replays one or more bulletins on the same interval, batching one record per flight per tick into a single CAT-062 block; flights whose Mode-S codes share a 12-bit track number are remapped onto fallback addresses with a warning (common case: bulletins generated from the same SimBrief airframe). `[simulation]` config identity overrides apply in single-flight mode only.
- **`shared/lido.rs`** — the FLIGHT LOG section is the only mandatory one; waypoint blocks are located by fixed-width column slices — column positions are load-bearing.
- **`live/opensky.rs`** — `TokenManager` caches the access token behind an `Arc<RwLock<...>>` and refreshes 60 s before expiry. `StateVector` has a **custom `Deserialize`** because OpenSky returns state vectors as positional JSON arrays (17 or 18 elements); the field-to-index mapping in the struct comments is load-bearing — do not reorder. `OpenSkyError` distinguishes rate limits from other failures so the live loop can back off specifically on 429.
- **`shared/publisher.rs`** — binds to `0.0.0.0:0` (ephemeral local port) and `send_to`s each ASTERIX block as one UDP datagram. No fragmentation or framing is added — the receiver parses ASTERIX block boundaries itself.

## Simulation engine: current model and direction

`replay/` is a **timeline replay**: the whole flight is precomputed at startup and `sample(elapsed)` is a pure function of time. The agreed direction (2026-06-13) keeps replay a bounded, deterministic playback engine (multi-bulletin replay + time control) and builds the stateful, agent-based model as a **separate, independent mode** in `agent/` — its own execution (`step(dt)`), agent-executed scenarios, and a clearance feedback loop (CFL/HDG/DCT/SPD). The two modes share only `shared/`. Physics stays simple and ground-speed based — no BADA. See `docs/SIMULATION.md` for the full description, the phase plan, and the decisions log. Replay's only remaining feature is time control; do not add anything else *inside* `sample(t)`.

## External dependency: libasterix

CAT-062 encoding is **not** implemented in this repo — it comes from the `libasterix` crate (published separately at https://github.com/algonents/libasterix; local checkout at `~/Repos/libasterix`). When changing the conversion, the public surface used is: `Cat062Record`, `encode_cat062_block`, `icao_to_track_number`, `parse_icao_address`, `velocity_to_cartesian`. Check that crate's docs/source before assuming a field exists on `Cat062Record`.

**Known encoder gap (deferred by decision, 2026-06-12)**: `encode_cat062_record` in libasterix 0.1.0 never writes `Cat062Record.icao_address` to the wire — I062/245 carries STI + callsign only, and the 24-bit Mode-S address belongs in I062/380 subfield ADR, which the crate decodes but does not encode. Both live and simulation mode set `icao_address` anyway, so the address transmits as soon as libasterix gains ADR encoding. Until then, correlation runs on callsign + the 12-bit track number (low bits of the Mode-S address).

## Conventions worth knowing

- Unit conversions happen at the OpenSky boundary: `StateVector::altitude_ft()` converts metres→feet, `velocity_to_cartesian` (from libasterix) converts polar→Cartesian. Downstream code works in ASTERIX-native units.
- `track_number` is derived by hashing the 24-bit ICAO address to 12 bits (`icao_to_track_number`). Collisions are possible but accepted — this is a simulator, not an operational system.
- Records with no lat/lon are silently dropped in `state_to_cat062` (`live/run.rs`) — returns `None`, filtered by `filter_map`. A fetch of N states commonly yields fewer than N records.
