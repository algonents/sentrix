//! OpenSky Network API client
//!
//! Fetches real-time ADS-B aircraft state vectors from the OpenSky Network.
//! API documentation: https://openskynetwork.github.io/opensky-api/
//!
//! Authentication uses OAuth2 client credentials flow.
//! Rate limits depend on account type and contribution status.

use serde::Deserialize;
use std::sync::Arc;
use tokio::sync::RwLock;

const OPENSKY_API_URL: &str = "https://opensky-network.org/api/states/all";
const OPENSKY_TOKEN_URL: &str = "https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token";

/// OAuth2 token response from OpenSky
#[derive(Debug, Deserialize)]
struct TokenResponse {
    access_token: String,
    expires_in: u64,
}

/// OpenSky Network OAuth2 credentials
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Credentials {
    pub client_id: String,
    pub client_secret: String,
}

impl Credentials {
    pub fn new(client_id: impl Into<String>, client_secret: impl Into<String>) -> Self {
        Self {
            client_id: client_id.into(),
            client_secret: client_secret.into(),
        }
    }

    /// Load credentials from environment variables OPENSKY_CLIENT_ID and OPENSKY_CLIENT_SECRET
    pub fn from_env() -> Option<Self> {
        let client_id = std::env::var("OPENSKY_CLIENT_ID").ok()?;
        let client_secret = std::env::var("OPENSKY_CLIENT_SECRET").ok()?;
        Some(Self { client_id, client_secret })
    }

    /// Load credentials from a JSON file (e.g., credentials.json downloaded from OpenSky)
    pub fn from_file(path: impl AsRef<std::path::Path>) -> Option<Self> {
        let contents = std::fs::read_to_string(path).ok()?;
        serde_json::from_str(&contents).ok()
    }

    /// Load credentials, trying file first, then environment variables
    /// Returns the credentials and the source they were loaded from
    pub fn load() -> Option<(Self, &'static str)> {
        if let Some(creds) = Self::from_file("conf/credentials.json") {
            return Some((creds, "conf/credentials.json"));
        }
        if let Some(creds) = Self::from_env() {
            return Some((creds, "environment variables"));
        }
        None
    }
}

/// Cached access token with expiry tracking
#[derive(Debug)]
struct CachedToken {
    access_token: String,
    expires_at: std::time::Instant,
}

/// OAuth2 token manager that handles fetching and refreshing tokens
#[derive(Debug)]
pub struct TokenManager {
    credentials: Credentials,
    cached_token: Arc<RwLock<Option<CachedToken>>>,
}

impl TokenManager {
    pub fn new(credentials: Credentials) -> Self {
        Self {
            credentials,
            cached_token: Arc::new(RwLock::new(None)),
        }
    }

    /// Get a valid access token, refreshing if necessary
    pub async fn get_token(&self, client: &reqwest::Client) -> Result<String, reqwest::Error> {
        // Check if we have a valid cached token
        {
            let cached = self.cached_token.read().await;
            if let Some(ref token) = *cached {
                // Refresh 60 seconds before expiry
                if token.expires_at > std::time::Instant::now() + std::time::Duration::from_secs(60) {
                    return Ok(token.access_token.clone());
                }
            }
        }

        // Fetch new token
        let response: TokenResponse = client
            .post(OPENSKY_TOKEN_URL)
            .form(&[
                ("grant_type", "client_credentials"),
                ("client_id", &self.credentials.client_id),
                ("client_secret", &self.credentials.client_secret),
            ])
            .send()
            .await?
            .json()
            .await?;

        let expires_at = std::time::Instant::now() + std::time::Duration::from_secs(response.expires_in);
        let access_token = response.access_token.clone();

        // Cache the token
        {
            let mut cached = self.cached_token.write().await;
            *cached = Some(CachedToken {
                access_token: response.access_token,
                expires_at,
            });
        }

        Ok(access_token)
    }
}

/// Bounding box for geographic filtering
#[derive(Debug, Clone, Deserialize)]
pub struct BoundingBox {
    pub min_lat: f64,
    pub max_lat: f64,
    pub min_lon: f64,
    pub max_lon: f64,
}

impl BoundingBox {
    pub fn new(min_lat: f64, max_lat: f64, min_lon: f64, max_lon: f64) -> Self {
        Self { min_lat, max_lat, min_lon, max_lon }
    }

    /// London/UK area
    pub fn london() -> Self {
        Self::new(50.0, 53.0, -2.0, 2.0)
    }

    /// Switzerland
    pub fn switzerland() -> Self {
        Self::new(45.8, 47.8, 5.9, 10.5)
    }

    /// UK + Switzerland (Western/Central Europe)
    pub fn uk_switzerland() -> Self {
        Self::new(45.8, 53.0, -2.0, 10.5)
    }
}

/// OpenSky API response
#[derive(Debug, Deserialize)]
pub struct OpenSkyResponse {
    pub time: i64,
    pub states: Option<Vec<StateVector>>,
}

/// Aircraft state vector from OpenSky
/// Fields: https://openskynetwork.github.io/opensky-api/rest.html#all-state-vectors
///
/// Deserialized from a JSON array with 17 or 18 elements (category field is optional).
#[derive(Debug)]
pub struct StateVector {
    pub icao24: String,              // 0
    pub callsign: Option<String>,    // 1
    pub time_position: Option<i64>,  // 3
    pub longitude: Option<f64>,      //wha 5
    pub latitude: Option<f64>,       // 6
    pub baro_altitude: Option<f64>,  // 7 (meters)
    pub on_ground: bool,             // 8
    pub velocity: Option<f64>,       // 9 (m/s)
    pub true_track: Option<f64>,     // 10 (degrees)
    pub squawk: Option<String>,      // 14
    pub position_source: i64,        // 16
    pub category: Option<i64>,       // 17 (optional)
}

impl<'de> serde::Deserialize<'de> for StateVector {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let arr: Vec<serde_json::Value> = Vec::deserialize(deserializer)?;
        if arr.len() < 17 {
            return Err(serde::de::Error::custom(format!(
                "expected at least 17 elements, got {}",
                arr.len()
            )));
        }

        let str_field = |v: &serde_json::Value| v.as_str().map(|s| s.to_string());
        let f64_field = |v: &serde_json::Value| v.as_f64();
        let i64_field = |v: &serde_json::Value| v.as_i64();
        let bool_field = |v: &serde_json::Value| v.as_bool().unwrap_or(false);

        Ok(StateVector {
            icao24: str_field(&arr[0]).unwrap_or_default(),
            callsign: str_field(&arr[1]),
            time_position: i64_field(&arr[3]),
            longitude: f64_field(&arr[5]),
            latitude: f64_field(&arr[6]),
            baro_altitude: f64_field(&arr[7]),
            on_ground: bool_field(&arr[8]),
            velocity: f64_field(&arr[9]),
            true_track: f64_field(&arr[10]),
            squawk: str_field(&arr[14]),
            position_source: i64_field(&arr[16]).unwrap_or(0),
            category: arr.get(17).and_then(|v| i64_field(v)),
        })
    }
}

impl StateVector {
    pub fn icao24(&self) -> &str {
        &self.icao24
    }

    pub fn time_position(&self) -> Option<i64> {
        self.time_position
    }

    pub fn callsign(&self) -> Option<&str> {
        self.callsign.as_ref().map(|s| s.trim())
    }

    pub fn longitude(&self) -> Option<f64> {
        self.longitude
    }

    pub fn latitude(&self) -> Option<f64> {
        self.latitude
    }

    /// Altitude in feet (converted from meters)
    pub fn altitude_feet(&self) -> Option<i32> {
        self.baro_altitude.map(|m| (m * 3.28084) as i32)
    }

    /// Velocity in m/s
    pub fn velocity_ms(&self) -> Option<f64> {
        self.velocity
    }

    /// True track (heading) in degrees
    pub fn true_track(&self) -> Option<f64> {
        self.true_track
    }

    /// Heading in degrees (integer)
    pub fn heading(&self) -> Option<i32> {
        self.true_track.map(|h| h as i32)
    }
}

/// Fetch aircraft states from OpenSky Network
///
/// If a token manager is provided, uses OAuth2 Bearer token for authenticated requests.
pub async fn fetch_states(
    client: &reqwest::Client,
    bbox: &BoundingBox,
    token_manager: Option<&TokenManager>,
) -> Result<Vec<StateVector>, OpenSkyError> {
    let url = format!(
        "{}?lamin={}&lomin={}&lamax={}&lomax={}",
        OPENSKY_API_URL,
        bbox.min_lat,
        bbox.min_lon,
        bbox.max_lat,
        bbox.max_lon
    );

    let mut request = client.get(&url);

    if let Some(tm) = token_manager {
        let token = tm.get_token(client).await?;
        request = request.bearer_auth(token);
    }

    let response = request.send().await?;
    let status = response.status();

    if status == reqwest::StatusCode::TOO_MANY_REQUESTS {
        let retry_after = response
            .headers()
            .get("x-rate-limit-retry-after-seconds")
            .or_else(|| response.headers().get("retry-after"))
            .and_then(|v| v.to_str().ok())
            .and_then(|v| v.parse::<u64>().ok());
        return Err(OpenSkyError::RateLimited { retry_after });
    }

    let text = response.text().await?;

    if !status.is_success() {
        return Err(OpenSkyError::HttpStatus { status, body: text });
    }

    let parsed: OpenSkyResponse = match serde_json::from_str(&text) {
        Ok(r) => r,
        Err(e) => {
            let preview = if text.len() > 500 { &text[..500] } else { &text };
            eprintln!("Deserialization error: {}\nResponse preview: {}", e, preview);
            return Err(OpenSkyError::Serde(e));
        }
    };

    Ok(parsed.states.unwrap_or_default())
}

/// Error type for OpenSky API operations
#[derive(Debug)]
pub enum OpenSkyError {
    Request(reqwest::Error),
    Serde(serde_json::Error),
    RateLimited { retry_after: Option<u64> },
    HttpStatus { status: reqwest::StatusCode, body: String },
}

impl OpenSkyError {
    pub fn is_rate_limited(&self) -> bool {
        matches!(self, OpenSkyError::RateLimited { .. })
    }

    pub fn retry_after_secs(&self) -> Option<u64> {
        match self {
            OpenSkyError::RateLimited { retry_after } => *retry_after,
            _ => None,
        }
    }
}

impl std::fmt::Display for OpenSkyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OpenSkyError::Request(e) => write!(f, "OpenSky request error: {}", e),
            OpenSkyError::Serde(e) => write!(f, "OpenSky deserialization error: {}", e),
            OpenSkyError::RateLimited { retry_after } => match retry_after {
                Some(secs) => write!(f, "OpenSky rate limited (retry after {}s)", secs),
                None => write!(f, "OpenSky rate limited"),
            },
            OpenSkyError::HttpStatus { status, body } => {
                write!(f, "OpenSky HTTP {}: {}", status, body)
            }
        }
    }
}

impl std::error::Error for OpenSkyError {}

impl From<reqwest::Error> for OpenSkyError {
    fn from(err: reqwest::Error) -> Self {
        OpenSkyError::Request(err)
    }
}
