//! Pluggable aircraft-performance provider.
//!
//! The agent reads its climb/descent rate *limits* through a `PerformanceModel`
//! so the numbers can come from anywhere. sentrix ships only this code — a
//! built-in `DefaultPerformance` (flat constants, today's behaviour) and a
//! loader for the OpenAP WRAP format — and **never the OpenAP data**, which is
//! GPL-3.0. A user points `WrapPerformance` at their own OpenAP `data/wrap/`
//! directory at runtime.
//!
//! The WRAP loader reimplements the lookup in `openap/kinematic.py::WRAP` (the
//! behavioural reference); it is not a transliteration. Units in the WRAP files
//! are SI (m/s, km); we convert to ours (fpm, ft) on the way in.

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result};

/// Default vertical rate when no performance data is loaded (today's behaviour).
pub const DEFAULT_VERTICAL_RATE_FPM: f64 = 2000.0;

const MPS_PER_FPM: f64 = 0.00508; // ft/min -> m/s (OpenAP aero.fpm)
const FT_PER_KM: f64 = 1000.0 / 0.3048; // km -> ft

fn mps_to_fpm(mps: f64) -> f64 {
    mps / MPS_PER_FPM
}

fn km_to_ft(km: f64) -> f64 {
    km * FT_PER_KM
}

/// Per-aircraft climb/descent rate limits (fpm, positive) as a function of
/// altitude. Climb and descent each split into three bands at the WRAP
/// crossover altitudes (constant-CAS and constant-Mach segments).
#[derive(Debug, Clone)]
pub struct VerticalLimits {
    climb_low: f64,  // below climb_cas_alt_ft
    climb_mid: f64,  // climb_cas_alt_ft .. climb_mach_alt_ft
    climb_high: f64, // above climb_mach_alt_ft
    climb_cas_alt_ft: f64,
    climb_mach_alt_ft: f64,
    descent_high: f64, // above descent_mach_alt_ft
    descent_mid: f64,  // descent_cas_alt_ft .. descent_mach_alt_ft
    descent_low: f64,  // below descent_cas_alt_ft
    descent_mach_alt_ft: f64,
    descent_cas_alt_ft: f64,
}

impl VerticalLimits {
    /// A constant rate at every altitude (the default model).
    pub fn flat(fpm: f64) -> Self {
        Self {
            climb_low: fpm,
            climb_mid: fpm,
            climb_high: fpm,
            climb_cas_alt_ft: 0.0,
            climb_mach_alt_ft: 0.0,
            descent_high: fpm,
            descent_mid: fpm,
            descent_low: fpm,
            descent_mach_alt_ft: 0.0,
            descent_cas_alt_ft: 0.0,
        }
    }

    /// Max climb rate (fpm, positive) at `altitude_ft`.
    pub fn climb_limit_fpm(&self, altitude_ft: f64) -> f64 {
        if altitude_ft < self.climb_cas_alt_ft {
            self.climb_low
        } else if altitude_ft < self.climb_mach_alt_ft {
            self.climb_mid
        } else {
            self.climb_high
        }
    }

    /// Max descent rate (fpm, positive) at `altitude_ft`.
    pub fn descent_limit_fpm(&self, altitude_ft: f64) -> f64 {
        if altitude_ft >= self.descent_mach_alt_ft {
            self.descent_high
        } else if altitude_ft >= self.descent_cas_alt_ft {
            self.descent_mid
        } else {
            self.descent_low
        }
    }
}

/// A source of per-aircraft performance limits.
pub trait PerformanceModel {
    /// Vertical-rate limits for an ICAO aircraft type (e.g. "A320"). Unknown
    /// types fall back to the default constant limits.
    fn vertical_limits(&self, icao_type: &str) -> VerticalLimits;
}

/// Flat constant limits for every type — today's behaviour, no data needed.
pub struct DefaultPerformance {
    rate_fpm: f64,
}

impl DefaultPerformance {
    pub fn new(rate_fpm: f64) -> Self {
        Self { rate_fpm }
    }
}

impl PerformanceModel for DefaultPerformance {
    fn vertical_limits(&self, _icao_type: &str) -> VerticalLimits {
        VerticalLimits::flat(self.rate_fpm)
    }
}

/// Per-type limits loaded from a directory of OpenAP WRAP `.txt` files (the
/// user's own `openap/.../data/wrap/`). Never shipped with sentrix.
pub struct WrapPerformance {
    by_type: HashMap<String, VerticalLimits>,
    /// ICAO type (lowercased) -> the WRAP key that covers it (`_synonym.csv`).
    synonyms: HashMap<String, String>,
    default_rate_fpm: f64,
}

impl WrapPerformance {
    /// Load every `*.txt` WRAP file in `dir` (plus `_synonym.csv` if present).
    pub fn load(dir: impl AsRef<Path>, default_rate_fpm: f64) -> Result<Self> {
        let dir = dir.as_ref();
        let mut by_type = HashMap::new();

        let entries = std::fs::read_dir(dir)
            .with_context(|| format!("Failed to read WRAP directory: {}", dir.display()))?;
        for entry in entries {
            let path = entry?.path();
            if path.extension().and_then(|e| e.to_str()) != Some("txt") {
                continue;
            }
            let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else {
                continue;
            };
            if stem.starts_with('_') {
                continue; // e.g. _synonym.csv has no .txt, but guard anyway
            }
            let content = std::fs::read_to_string(&path)
                .with_context(|| format!("Failed to read WRAP file: {}", path.display()))?;
            if let Some(limits) = parse_vertical_limits(&content) {
                by_type.insert(stem.to_ascii_lowercase(), limits);
            }
        }

        if by_type.is_empty() {
            anyhow::bail!("no WRAP `*.txt` files parsed from {}", dir.display());
        }

        Ok(Self {
            by_type,
            synonyms: load_synonyms(dir),
            default_rate_fpm,
        })
    }
}

impl PerformanceModel for WrapPerformance {
    fn vertical_limits(&self, icao_type: &str) -> VerticalLimits {
        let key = icao_type.to_ascii_lowercase();
        if let Some(limits) = self.by_type.get(&key) {
            return limits.clone();
        }
        if let Some(syn) = self.synonyms.get(&key)
            && let Some(limits) = self.by_type.get(syn)
        {
            return limits.clone();
        }
        VerticalLimits::flat(self.default_rate_fpm)
    }
}

/// Read `_synonym.csv` (`orig,new` rows) into a lowercase `orig -> new` map.
fn load_synonyms(dir: &Path) -> HashMap<String, String> {
    let mut map = HashMap::new();
    let Ok(content) = std::fs::read_to_string(dir.join("_synonym.csv")) else {
        return map;
    };
    for line in content.lines().skip(1) {
        let mut cols = line.split(',');
        if let (Some(orig), Some(new)) = (cols.next(), cols.next()) {
            map.insert(
                orig.trim().to_ascii_lowercase(),
                new.trim().to_ascii_lowercase(),
            );
        }
    }
    map
}

/// The `default`/opt value of a WRAP variable. The fixed-width table has a
/// multi-word `name` column, so we read from the right: each row ends
/// `... opt min max model params`, so `opt = tokens[len-5]`, `variable =
/// tokens[0]`. Robust to spaces/commas in the name.
fn wrap_opt(content: &str, var: &str) -> Option<f64> {
    content.lines().find_map(|line| {
        let tokens: Vec<&str> = line.split_whitespace().collect();
        if tokens.len() >= 5 && tokens[0] == var {
            tokens[tokens.len() - 5].parse().ok()
        } else {
            None
        }
    })
}

/// Parse one type's WRAP file into `VerticalLimits` (None if any required
/// variable is missing). Descent rates are stored as positive magnitudes.
fn parse_vertical_limits(content: &str) -> Option<VerticalLimits> {
    let v = |var: &str| wrap_opt(content, var);
    Some(VerticalLimits {
        climb_low: mps_to_fpm(v("cl_vs_avg_pre_cas")?),
        climb_mid: mps_to_fpm(v("cl_vs_avg_cas_const")?),
        climb_high: mps_to_fpm(v("cl_vs_avg_mach_const")?),
        climb_cas_alt_ft: km_to_ft(v("cl_h_cas_const")?),
        climb_mach_alt_ft: km_to_ft(v("cl_h_mach_const")?),
        descent_high: mps_to_fpm(v("de_vs_avg_mach_const")?.abs()),
        descent_mid: mps_to_fpm(v("de_vs_avg_cas_const")?.abs()),
        descent_low: mps_to_fpm(v("de_vs_avg_after_cas")?.abs()),
        descent_mach_alt_ft: km_to_ft(v("de_h_mach_const")?),
        descent_cas_alt_ft: km_to_ft(v("de_h_cas_const")?),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // A synthetic fixture in the WRAP column layout (made-up numbers — NOT
    // OpenAP data). Only the variables the loader reads are present.
    const FIXTURE: &str = "\
variable              flight phase    name               opt    min    max    model  parameters
cl_vs_avg_pre_cas     climb           Pre CAS rate       10.0   8.0    12.0   norm   10.0|1.0
cl_vs_avg_cas_const   climb           CAS rate           8.0    6.0    10.0   norm   8.0|1.0
cl_vs_avg_mach_const  climb           Mach rate          5.0    3.0    7.0    norm   5.0|1.0
cl_h_cas_const        climb           CAS alt            3.0    2.0    4.0    norm   3.0|0.5
cl_h_mach_const       climb           Mach alt           9.0    8.0    10.0   norm   9.0|0.5
de_vs_avg_mach_const  descent         Mach desc rate    -6.0   -8.0   -4.0    norm  -6.0|1.0
de_vs_avg_cas_const   descent         CAS desc rate     -10.0  -12.0  -8.0    norm  -10.0|1.0
de_vs_avg_after_cas   descent         After CAS rate    -6.0   -8.0   -4.0    norm  -6.0|1.0
de_h_mach_const       descent         Mach desc alt      9.0    8.0    10.0   norm   9.0|0.5
de_h_cas_const        descent         CAS desc alt       6.0    5.0    7.0    norm   6.0|0.5
";

    #[test]
    fn test_wrap_opt_reads_from_the_right() {
        // multi-word name must not confuse the column read
        assert_eq!(wrap_opt(FIXTURE, "cl_vs_avg_cas_const"), Some(8.0));
        assert_eq!(wrap_opt(FIXTURE, "de_vs_avg_cas_const"), Some(-10.0));
        assert_eq!(wrap_opt(FIXTURE, "nonexistent"), None);
    }

    #[test]
    fn test_parse_and_band_selection() {
        let limits = parse_vertical_limits(FIXTURE).unwrap();

        // climb bands: 10/8/5 m/s -> fpm, split at 3 km (9842 ft) and 9 km (29528 ft)
        assert!((limits.climb_limit_fpm(5_000.0) - mps_to_fpm(10.0)).abs() < 1.0);
        assert!((limits.climb_limit_fpm(15_000.0) - mps_to_fpm(8.0)).abs() < 1.0);
        assert!((limits.climb_limit_fpm(35_000.0) - mps_to_fpm(5.0)).abs() < 1.0);

        // descent bands: split at 9 km (29528 ft) and 6 km (19685 ft)
        assert!((limits.descent_limit_fpm(35_000.0) - mps_to_fpm(6.0)).abs() < 1.0);
        assert!((limits.descent_limit_fpm(25_000.0) - mps_to_fpm(10.0)).abs() < 1.0);
        assert!((limits.descent_limit_fpm(10_000.0) - mps_to_fpm(6.0)).abs() < 1.0);
    }

    #[test]
    fn test_default_is_flat() {
        let perf = DefaultPerformance::new(2000.0);
        let limits = perf.vertical_limits("A320");
        assert_eq!(limits.climb_limit_fpm(1_000.0), 2000.0);
        assert_eq!(limits.climb_limit_fpm(38_000.0), 2000.0);
        assert_eq!(limits.descent_limit_fpm(38_000.0), 2000.0);
    }

    // Cross-check our parse against the real OpenAP A320 data, if the user has
    // a local checkout. Ignored by default (no GPL data is shipped with sentrix).
    #[test]
    #[ignore = "requires a local OpenAP checkout at ~/Repos/openap"]
    fn test_matches_openap_a320() {
        let home = std::env::var("HOME").unwrap();
        let path = format!("{home}/Repos/openap/openap/data/wrap/a320.txt");
        let content = std::fs::read_to_string(&path).expect("OpenAP a320.txt");
        let limits = parse_vertical_limits(&content).unwrap();
        // cl_vs_avg_cas_const = 8.43 m/s -> ~1659 fpm
        assert!((limits.climb_limit_fpm(20_000.0) - mps_to_fpm(8.43)).abs() < 1.0);
    }
}
