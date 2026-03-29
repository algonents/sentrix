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

## License

MIT
