# sentrix

An OpenSky Network to ASTERIX CAT-062 converter. Polls live aircraft state vectors from the [OpenSky Network](https://opensky-network.org/) REST API and publishes them as binary ASTERIX CAT-062 messages over UDP.

Useful for simulating real ASTERIX data sources during ATM (Air Traffic Management) software development.

## How It Works

```
OpenSky REST API  -->  sentrix  -->  ASTERIX CAT-062 over UDP
   (poll)                              (publish)
```

Sentrix periodically fetches aircraft positions within a configurable geographic bounding box, converts each state vector into an ASTERIX CAT-062 record using [libasterix](https://github.com/algonents/libasterix), and sends the encoded block to a UDP destination.

## Setup

### Credentials

Sentrix requires OpenSky Network API credentials. Provide them via environment variables:

```bash
export OPENSKY_CLIENT_ID="your_client_id"
export OPENSKY_CLIENT_SECRET="your_client_secret"
```

Or create a `conf/credentials.json`:

```json
{
  "client_id": "your_client_id",
  "client_secret": "your_client_secret"
}
```

### Configuration

Edit `conf/sentrix.toml` to configure polling, bounding box, and output:

```toml
poll_interval_secs = 5

[bounding_box]
min_lat = 45.8
max_lat = 47.8
min_lon = 5.9
max_lon = 10.5

[asterix]
sac = 1
sic = 1

[udp]
destination = "127.0.0.1:4000"

# Optional overrides; by default identity comes from the OFP bulletin
[simulation]
#callsign = "SIM001"
#icao24 = "4b1234"
```

## Usage

```bash
cargo run
```

Output:

```
Sentrix - OpenSky to ASTERIX CAT062 converter
============================================
Configuration loaded: poll every 5s, SAC=1 SIC=1
Bounding box: lat [45.8, 47.8], lon [5.9, 10.5]
UDP publisher ready: -> 127.0.0.1:4000

[14:32:05] Sent 47 records (2856 bytes) from 52 states
[14:32:10] Sent 48 records (2904 bytes) from 53 states
```

## Simulation Mode

Instead of live OpenSky data, Sentrix can replay a [SimBrief](https://www.simbrief.com/) LIDO-layout OFP bulletin, publishing a single simulated aircraft as CAT-062 in real time:

```bash
cargo run -- --simulate simulations/lsgg_lfpg.txt
```

No OpenSky credentials are needed in this mode. Sentrix parses the FLIGHT LOG waypoints (position, flight level, TAS, GS), builds a timeline from leg distance and ground speed, and interpolates position, altitude and speeds between waypoints on every tick (`poll_interval_secs`). On arrival, the aircraft holds its final position with zero velocity.

When the input is a full bulletin (not just a flight-log extract), the other sections refine the simulation:

- **Identity**: the ATC callsign (ICAO flight plan item 7) is published in the CAT-062 target identification (I062/245) and the Mode-S address (`CODE/` item) seeds the 12-bit track number, for flight-plan correlation. `[simulation]` config values override them. *Known limitation: the full 24-bit Mode-S address belongs in I062/380 (ADR), which libasterix 0.1.0 does not yet encode — `icao_address` is populated on the record and will be transmitted once the crate supports it.*
- **Speed profile**: V2 (takeoff runway analysis) and VREF (landing distance table, interpolated at the planned landing weight) drive realistic acceleration after takeoff and deceleration to the threshold on final, with a ~3° glide — bringing the replay duration in line with the plan's ETE.
- **Winds aloft** are parsed and reserved for future climb/descent modelling.

A flight-log-only extract still works; the refinements simply switch off.

Output:

```
Simulation: LSGG/22 -> LFPG/27R | 22 waypoints, 236 nm, estimated 42 min (log ETE: 45 min)
Aircraft: ALU (A320, N320SB) icao24 1349
Speed profile: V2 154 kt, VREF 133 kt

[08:19:59] ALU  46.2383    6.1100     0 ft GS 154 kt TAS 154 kt trk 225 -> CLIMB (35 bytes)
[08:20:04] ALU  46.2369    6.1080    93 ft GS 168 kt TAS 168 kt trk 225 -> CLIMB (35 bytes)
```

Notes:
- CAT-062 has no TAS field in the encoded record — the published velocity (I062/185) is derived from ground speed and track; TAS is interpolated for the console output only (estimated as GS minus the wind component where the log omits it).
- The flight log omits altitude at the airports; they default to 0 ft.
- The path geometry is always the published route (airways + STAR) exactly as logged — synthetic profile points lie on the existing legs, and no runway-aligned approach geometry is fabricated.

## License

MIT
