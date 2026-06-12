# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
cargo run                    # Live mode (reads conf/sentrix.toml + conf/credentials.json)
cargo run -- --simulate simulations/lsgg_lfpg.txt  # Simulation mode (no credentials needed)
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
sim:   LIDO OFP bulletin  --parse_bulletin-->  LidoBulletin  --FlightPath::from_bulletin / sample-->  Cat062Record  --encode_cat062_block-->  UDP bytes
```

Six modules, each a thin layer with one responsibility:

- **`main.rs`** — orchestrates both loops (`run_live` / `run_simulation`, selected by the `--simulate <file>` arg). The conversion function `state_to_cat062` lives here (not in `opensky.rs`) because it depends on `libasterix` types; keeping it in `main` avoids coupling the OpenSky client to ASTERIX concepts. The live loop polls on `poll_interval_secs`, and on `OpenSkyError::RateLimited` sleeps for the server-provided `retry-after` (or 30s fallback) *in addition to* the normal poll interval. The sim loop publishes one record per tick using the same interval.
- **`lido.rs`** — SimBrief LIDO OFP bulletin parser. The FLIGHT LOG section is the only mandatory one; waypoint blocks are located by their LAT/LON column patterns (fixed-width slices — column positions are load-bearing). Optional sections: FPL identity (callsign/REG/CODE), routing runways, V2/VREF from the runway analysis tables, winds aloft. A flight-log-only extract parses with all optionals `None`.
- **`simulation.rs`** — turns waypoints into a time-indexed `FlightPath` (timeline = leg distance ÷ average GS, **not** the minute-resolution TTLT column) and interpolates state with `sample(elapsed)`. `from_bulletin` additionally synthesizes V2 takeoff and VREF approach profile points *on the existing legs* — path geometry is intentionally never altered (decision: no fabricated runway-aligned approaches; en-route fidelity is the priority).
- **`opensky.rs`** — OpenSky REST client + OAuth2 client-credentials flow. `TokenManager` caches the access token behind an `Arc<RwLock<...>>` and refreshes 60 s before expiry. `StateVector` has a **custom `Deserialize`** because OpenSky returns state vectors as positional JSON arrays (17 or 18 elements); the field-to-index mapping in the struct comments is load-bearing — do not reorder. `OpenSkyError` distinguishes rate limits from other failures so the main loop can back off specifically on 429.
- **`config.rs`** — TOML config loader. `BoundingBox` is re-exported from `opensky.rs` (not defined here) so the API client owns its own query-shape type.
- **`publisher.rs`** — thin `UdpSocket` wrapper. Binds to `0.0.0.0:0` (ephemeral local port) and `send_to`s each ASTERIX block as one UDP datagram. No fragmentation or framing is added — the receiver is expected to parse ASTERIX block boundaries itself.

## External dependency: libasterix

CAT-062 encoding is **not** implemented in this repo — it comes from the `libasterix` crate (published separately at https://github.com/algonents/libasterix; local checkout at `~/Repos/libasterix`). When changing the conversion, the public surface used is: `Cat062Record`, `encode_cat062_block`, `icao_to_track_number`, `parse_icao_address`, `velocity_to_cartesian`. Check that crate's docs/source before assuming a field exists on `Cat062Record`.

**Known encoder gap (deferred by decision, 2026-06-12)**: `encode_cat062_record` in libasterix 0.1.0 never writes `Cat062Record.icao_address` to the wire — I062/245 carries STI + callsign only, and the 24-bit Mode-S address belongs in I062/380 subfield ADR, which the crate decodes but does not encode. Both live and simulation mode set `icao_address` anyway, so the address transmits as soon as libasterix gains ADR encoding. Until then, correlation runs on callsign + the 12-bit track number (low bits of the Mode-S address).

## Conventions worth knowing

- Unit conversions happen at the OpenSky boundary: `StateVector::altitude_feet()` converts metres→feet, `velocity_to_cartesian` (from libasterix) converts polar→Cartesian. Downstream code works in ASTERIX-native units.
- `track_number` is derived by hashing the 24-bit ICAO address to 12 bits (`icao_to_track_number`). Collisions are possible but accepted — this is a simulator, not an operational system.
- Records with no lat/lon are silently dropped in `state_to_cat062` (returns `None`, filtered by `filter_map`). A fetch of N states commonly yields fewer than N records.
