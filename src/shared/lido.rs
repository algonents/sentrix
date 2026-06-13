//! SimBrief LIDO operational flight plan (OFP) briefing parser
//!
//! Parses the LIDO-layout OFP that SimBrief generates (see `briefs/*.txt`).
//! The FLIGHT LOG section is mandatory; every other section (ICAO flight plan,
//! routing, runway analysis, wind information) is optional, so a flight-log-only
//! extract still parses — missing sections simply yield `None` / empty values.
//!
//! Waypoints come from the fixed-width FLIGHT LOG, a block of three columnar
//! lines per waypoint, described by the header repeated in the log:
//!
//! ```text
//! AWY                           FL   IMT   MN    WIND  OAT  EFOB  PBRN
//! POSITION    LAT      EET ETO MORA  ITT  TAS    COMP  TDV
//! IDENT       LONG    TTLT ATO DIS  RDIS   GS     SHR  TRP  AFOB  ABRN
//! ```
//!
//! Blocks are located by the LAT/LONG column patterns rather than by section
//! structure, so page-break header repeats, FREQ lines and FIR-boundary rows
//! are handled naturally — and the scan is unaffected by the other briefing
//! sections. All per-waypoint values other than the coordinates are optional:
//! airports omit FL, climb/descent rows omit TAS, FIR crossings carry
//! coordinates only.

use anyhow::{bail, Result};

/// Everything extracted from a LIDO briefing
#[derive(Debug)]
pub struct LidoBriefing {
    /// Flight log waypoints (the only mandatory section)
    pub waypoints: Vec<Waypoint>,
    /// ATC callsign — ICAO flight plan item 7 (what radar labels display)
    pub callsign: Option<String>,
    /// Aircraft registration (FPL item 18, REG/)
    pub registration: Option<String>,
    /// Aircraft type (FPL item 9, e.g. "A320")
    pub aircraft_type: Option<String>,
    /// 24-bit Mode-S address as hex (FPL item 18, CODE/) — transmitted in
    /// I062/245 for track correlation
    pub icao_address: Option<String>,
    /// Departure airport/runway from the ROUTING line, e.g. "LSGG/22"
    pub dep_runway: Option<String>,
    /// Arrival airport/runway, e.g. "LFPG/27R"
    pub arr_runway: Option<String>,
    /// Takeoff safety speed for the planned runway (knots)
    pub v2_kts: Option<f64>,
    /// Landing reference speed at the planned landing weight (knots)
    pub vref_kts: Option<f64>,
    /// Winds/temperature aloft by flight level at points along the route.
    /// Not consumed by the simulation yet (future climb/descent modelling).
    #[allow(dead_code)]
    pub wind_profiles: Vec<WindProfile>,
}

/// Winds aloft at one route point (CLIMB, T O C, ..., DESCENT).
///
/// Not consumed by the simulation yet — parsed for future climb/descent
/// speed-schedule modelling (IAS/Mach -> TAS -> GS needs wind + temperature).
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct WindProfile {
    pub name: String,
    pub levels: Vec<WindLevel>,
}

/// One flight level of a wind profile
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct WindLevel {
    pub fl: u32,
    pub dir_deg: f64,
    pub speed_kts: f64,
    pub oat_c: f64,
}

/// A waypoint extracted from the flight log
#[derive(Debug, Clone)]
pub struct Waypoint {
    pub ident: String,
    /// Latitude in degrees (north positive)
    pub lat: f64,
    /// Longitude in degrees (east positive)
    pub lon: f64,
    /// Planned altitude in feet (from the FL column)
    pub altitude_ft: Option<f64>,
    /// True airspeed in knots
    pub tas_kts: Option<f64>,
    /// Ground speed in knots
    pub gs_kts: Option<f64>,
    /// Wind component in knots (COMP column; tailwind positive, headwind negative)
    pub wind_comp_kts: Option<f64>,
    /// Cumulative elapsed time from departure in minutes (TTLT column)
    pub cum_time_min: Option<u32>,
}

/// Parse a full LIDO briefing (or a flight-log-only extract of one).
pub fn parse_briefing(text: &str) -> Result<LidoBriefing> {
    let lines: Vec<&str> = text.lines().collect();
    let (dep_runway, arr_runway) = parse_routing(&lines);
    Ok(LidoBriefing {
        waypoints: parse_flight_log(text)?,
        callsign: parse_fpl_callsign(&lines),
        registration: parse_fpl_token(&lines, "REG/"),
        aircraft_type: parse_fpl_aircraft_type(&lines),
        icao_address: parse_fpl_token(&lines, "CODE/"),
        dep_runway,
        arr_runway,
        v2_kts: parse_v2(&lines),
        vref_kts: parse_vref(&lines),
        wind_profiles: parse_wind_information(&lines),
    })
}

/// Parse the waypoint list out of the FLIGHT LOG section.
pub fn parse_flight_log(text: &str) -> Result<Vec<Waypoint>> {
    let lines: Vec<&str> = text.lines().collect();
    let mut waypoints = Vec::new();

    let mut i = 0;
    while i < lines.len() {
        let lat = field(lines[i], 12, 19).and_then(parse_lat);
        let lon = lines
            .get(i + 1)
            .and_then(|l| field(l, 11, 19))
            .and_then(parse_lon);

        let (Some(lat), Some(lon)) = (lat, lon) else {
            i += 1;
            continue;
        };

        let lat_line = lines[i];
        let lon_line = lines[i + 1];
        let awy_line = if i > 0 { lines[i - 1] } else { "" };

        // T O C / T O D and FIR boundaries have no ident on the LONG line;
        // fall back to the position name on the LAT line.
        let ident = field(lon_line, 0, 11)
            .or_else(|| field(lat_line, 0, 12))
            .unwrap_or("?")
            .to_string();

        // FIR/UIR boundary rows ("-LFMM", "-LSAS", ...) are airspace
        // annotations, not route fixes, and their printed coordinates are
        // unreliable: SimBrief's own DIS/RDIS columns can place the crossing
        // on the far side of the adjacent fix, so keeping the row folds the
        // path back on itself. The crossing lies on the leg between the real
        // fixes by definition - nothing is lost by dropping it.
        if ident.starts_with('-') {
            i += 2;
            continue;
        }

        waypoints.push(Waypoint {
            ident,
            lat,
            lon,
            altitude_ft: field(awy_line, 29, 33)
                .and_then(|s| s.parse::<f64>().ok())
                .map(|fl| fl * 100.0),
            tas_kts: field(lat_line, 38, 43).and_then(|s| s.parse().ok()),
            gs_kts: field(lon_line, 38, 43).and_then(|s| s.parse().ok()),
            wind_comp_kts: field(lat_line, 47, 52).and_then(parse_wind_comp),
            cum_time_min: field(lon_line, 20, 24).and_then(parse_hhmm),
        });

        i += 2;
    }

    if waypoints.len() < 2 {
        bail!(
            "flight log contains only {} waypoint(s) - not a valid SimBrief flight log?",
            waypoints.len()
        );
    }

    Ok(waypoints)
}

/// ATC callsign from the ICAO flight plan: "(FPL-ALU-IS" -> "ALU"
fn parse_fpl_callsign(lines: &[&str]) -> Option<String> {
    let line = lines.iter().find(|l| l.contains("(FPL-"))?;
    let rest = &line[line.find("(FPL-")? + 5..];
    let callsign = rest.split('-').next()?.trim();
    (!callsign.is_empty()
        && callsign.len() <= 7
        && callsign.chars().all(|c| c.is_ascii_alphanumeric()))
    .then(|| callsign.to_string())
}

/// Aircraft type from FPL item 9: the line after "(FPL-...", e.g.
/// "-A320/M-SDE3FGHIRWY/LB1" -> "A320"
fn parse_fpl_aircraft_type(lines: &[&str]) -> Option<String> {
    let idx = lines.iter().position(|l| l.contains("(FPL-"))?;
    let line = lines.get(idx + 1)?.trim();
    let rest = line.strip_prefix('-')?;
    let typ = rest.split('/').next()?;
    (typ.len() >= 2 && typ.len() <= 4 && typ.chars().all(|c| c.is_ascii_alphanumeric()))
        .then(|| typ.to_string())
}

/// A "KEY/value" token from the FPL block (e.g. "REG/N320SB", "CODE/1349")
fn parse_fpl_token(lines: &[&str], prefix: &str) -> Option<String> {
    let start = lines.iter().position(|l| l.contains("(FPL-"))?;
    for line in &lines[start..] {
        for token in line.split_whitespace() {
            if let Some(value) = token.strip_prefix(prefix) {
                let value = value.trim_end_matches(')');
                if !value.is_empty() {
                    return Some(value.to_string());
                }
            }
        }
        if line.contains(')') {
            break;
        }
    }
    None
}

/// Departure and arrival airport/runway from the ROUTING section line:
/// "LSGG/22 DIPIR1A ... TINIL9W LFPG/27R"
fn parse_routing(lines: &[&str]) -> (Option<String>, Option<String>) {
    let Some(start) = lines.iter().position(|l| l.trim() == "ROUTING:") else {
        return (None, None);
    };
    for line in lines.iter().skip(start + 1).take(8) {
        let tokens: Vec<&str> = line.split_whitespace().collect();
        let (Some(first), Some(last)) = (tokens.first(), tokens.last()) else {
            continue;
        };
        if is_airport_runway(first) {
            let arr = is_airport_runway(last).then(|| last.to_string());
            return (Some(first.to_string()), arr);
        }
    }
    (None, None)
}

/// "LSGG/22" / "LFPG/27R" style token
fn is_airport_runway(token: &str) -> bool {
    match token.split_once('/') {
        Some((apt, rwy)) => {
            apt.len() == 4
                && apt.chars().all(|c| c.is_ascii_uppercase())
                && !rwy.is_empty()
                && rwy.len() <= 3
                && rwy.chars().all(|c| c.is_ascii_alphanumeric())
        }
        None => false,
    }
}

/// V2 for the planned takeoff runway.
///
/// The planned-data row in "/// TAKEOFF DATA ///" prints truncated V-speeds,
/// so the runway is read there and the speeds from the "DRY RWY - PTOW -
/// CALM WIND" table, whose rows end in "... FLP V1 VR V2 LIMIT".
fn parse_v2(lines: &[&str]) -> Option<f64> {
    let takeoff = lines
        .iter()
        .position(|l| l.contains("/// TAKEOFF DATA ///"))?;
    let prwy = planned_runway(&lines[takeoff..])?;

    let table = lines
        .iter()
        .position(|l| l.contains("DRY RWY - PTOW - CALM WIND"))?;
    for line in lines.iter().skip(table + 1) {
        if line.starts_with("----") {
            break; // next table
        }
        let tokens: Vec<&str> = line.split_whitespace().collect();
        if tokens.first() == Some(&prwy.as_str()) && tokens.len() >= 3 {
            return tokens[tokens.len() - 2]
                .parse::<f64>()
                .ok()
                .filter(|v| (80.0..250.0).contains(v));
        }
    }
    None
}

/// VREF interpolated at the planned landing weight.
///
/// PLDW comes from the planned row in "/// LANDING DATA ///"; the
/// "LANDING DISTANCE" table provides (LDW, VREF) rows to interpolate in.
fn parse_vref(lines: &[&str]) -> Option<f64> {
    let landing = lines
        .iter()
        .position(|l| l.contains("/// LANDING DATA ///"))?;
    // "LFPG  27R  18.0  259M13  1025  6600  FULL  6319  AFM" -> PLDW (2nd-to-last)
    let pldw = planned_row(&lines[landing..])
        .and_then(|tokens| tokens.get(tokens.len().checked_sub(2)?)?.parse::<f64>().ok())?;

    let table = lines.iter().position(|l| l.contains("LANDING DISTANCE"))?;
    let mut rows: Vec<(f64, f64)> = Vec::new();
    for line in lines.iter().skip(table + 1).take(20) {
        // The planned-weight bracket row is marked with a leading "/"
        let tokens: Vec<&str> = line.trim_start_matches('/').split_whitespace().collect();
        if tokens.len() < 2 {
            continue;
        }
        let (Ok(ldw), Ok(vref)) = (tokens[0].parse::<f64>(), tokens[1].parse::<f64>()) else {
            continue;
        };
        if ldw >= 1000.0 && (80.0..250.0).contains(&vref) {
            rows.push((ldw, vref));
        }
    }
    interpolate_table(&rows, pldw)
}

/// The planned-data row of a takeoff/landing report: the first row after the
/// "APT  PRWY ..." header whose first token is a 4-letter airport code.
fn planned_row<'a>(section: &[&'a str]) -> Option<Vec<&'a str>> {
    let header = section
        .iter()
        .position(|l| l.trim_start().starts_with("APT"))?;
    for line in section.iter().skip(header + 1).take(4) {
        let tokens: Vec<&str> = line.split_whitespace().collect();
        if let Some(first) = tokens.first()
            && first.len() == 4
            && first.chars().all(|c| c.is_ascii_uppercase())
        {
            return Some(tokens);
        }
    }
    None
}

/// The planned runway from a takeoff/landing report (second token of the planned row)
fn planned_runway(section: &[&str]) -> Option<String> {
    planned_row(section)?.get(1).map(|s| s.to_string())
}

/// Linear interpolation in a sorted (x, y) table, clamped to the table ends
fn interpolate_table(rows: &[(f64, f64)], x: f64) -> Option<f64> {
    let (first, last) = (rows.first()?, rows.last()?);
    if x <= first.0 {
        return Some(first.1);
    }
    if x >= last.0 {
        return Some(last.1);
    }
    for pair in rows.windows(2) {
        let ((x0, y0), (x1, y1)) = (pair[0], pair[1]);
        if x >= x0 && x <= x1 && x1 > x0 {
            return Some(y0 + (y1 - y0) * (x - x0) / (x1 - x0));
        }
    }
    None
}

/// Wind/temperature aloft tables from the WIND INFORMATION section.
///
/// Layout: a name header line ("CLIMB            T O C ..."), then data lines
/// in 17-character columns, one cell per profile: "350 314/037 -51"
/// (FL, direction/speed, OAT).
fn parse_wind_information(lines: &[&str]) -> Vec<WindProfile> {
    let mut profiles: Vec<WindProfile> = Vec::new();
    let Some(start) = lines.iter().position(|l| l.trim() == "WIND INFORMATION") else {
        return profiles;
    };

    // Indices into `profiles` for the block currently being filled
    let mut current_block: Vec<usize> = Vec::new();

    for line in lines.iter().skip(start + 1) {
        let trimmed = line.trim();
        if trimmed.starts_with('[') || line.starts_with("----") {
            break; // end of section
        }
        if trimmed.is_empty() || trimmed.chars().all(|c| c == '-' || c == ' ') {
            continue;
        }

        // Data lines start with a flight level ("350 314/037 -51  ...");
        // header lines start with a profile name ("CLIMB ... FF302")
        if trimmed.chars().next().is_some_and(|c| c.is_ascii_digit()) {
            // Data line: one cell per profile of the current block
            for (col, &profile_idx) in current_block.iter().enumerate() {
                if let Some(level) = parse_wind_cell(line, col) {
                    profiles[profile_idx].levels.push(level);
                }
            }
        } else {
            // Header line: starts a new block of profiles
            current_block.clear();
            for col in 0..4 {
                let Some(name) = field(line, col * 17, col * 17 + 17) else {
                    break;
                };
                current_block.push(profiles.len());
                profiles.push(WindProfile {
                    name: name.to_string(),
                    levels: Vec::new(),
                });
            }
        }
    }

    profiles
}

/// One 17-character wind cell: "350 314/037 -51" -> (FL, dir, speed, OAT)
fn parse_wind_cell(line: &str, col: usize) -> Option<WindLevel> {
    let cell = field(line, col * 17, col * 17 + 17)?;
    let mut tokens = cell.split_whitespace();
    let fl: u32 = tokens.next()?.parse().ok()?;
    let (dir, spd) = tokens.next()?.split_once('/')?;
    let oat: f64 = tokens.next()?.parse().ok()?;
    Some(WindLevel {
        fl,
        dir_deg: dir.parse().ok()?,
        speed_kts: spd.parse().ok()?,
        oat_c: oat,
    })
}

/// Extract a fixed-width column from a line, returning None when the line is
/// too short or the column is blank.
fn field(line: &str, start: usize, end: usize) -> Option<&str> {
    let end = end.min(line.len());
    if start >= end {
        return None;
    }
    let s = line.get(start..end)?.trim();
    (!s.is_empty()).then_some(s)
}

/// Parse a latitude like "N4614.3" (degrees + decimal minutes)
fn parse_lat(s: &str) -> Option<f64> {
    parse_coord(s, 'N', 'S', 2).filter(|v| v.abs() <= 90.0)
}

/// Parse a longitude like "E00606.6" (degrees + decimal minutes)
fn parse_lon(s: &str) -> Option<f64> {
    parse_coord(s, 'E', 'W', 3).filter(|v| v.abs() <= 180.0)
}

fn parse_coord(s: &str, pos: char, neg: char, deg_digits: usize) -> Option<f64> {
    if !s.is_ascii() {
        return None;
    }
    let hemi = s.chars().next()?;
    let sign = if hemi == pos {
        1.0
    } else if hemi == neg {
        -1.0
    } else {
        return None;
    };

    // Expect DDMM.M (lat) or DDDMM.M (lon) after the hemisphere letter
    let rest = &s[1..];
    if rest.len() != deg_digits + 4 {
        return None;
    }
    let deg: f64 = rest[..deg_digits].parse().ok()?;
    let min: f64 = rest[deg_digits..].parse().ok()?;
    if min >= 60.0 {
        return None;
    }

    Some(sign * (deg + min / 60.0))
}

/// Wind component like "P004" (tailwind +4 kt) or "M038" (headwind -38 kt)
fn parse_wind_comp(s: &str) -> Option<f64> {
    let sign = match s.chars().next()? {
        'P' => 1.0,
        'M' => -1.0,
        _ => return None,
    };
    s[1..].parse::<f64>().ok().map(|v| sign * v)
}

/// Parse an HHMM elapsed time ("0045" -> 45 minutes)
fn parse_hhmm(s: &str) -> Option<u32> {
    let v: u32 = s.parse().ok()?;
    Some((v / 100) * 60 + v % 100)
}

#[cfg(test)]
mod tests {
    use super::*;

    const BRIEFING: &str = include_str!("../../briefs/lsgg_lfpg.txt");

    /// A flight-log-only extract (no FPL, runway analysis or wind sections)
    const EXTRACT: &str = "\
AWY                           FL   IMT   MN    WIND  OAT  EFOB  PBRN
POSITION    LAT      EET ETO MORA  ITT  TAS    COMP  TDV
IDENT       LONG    TTLT ATO DIS  RDIS   GS     SHR  TRP  AFOB  ABRN
--------------------------------------------------------------------
                                   222                     5.2   0.2
GENEVA      N4614.3      ...  74   225         P004
LSGG       E00606.6 0000 ...       238  276               ....  ....

DIPIR1A                      050   328  .41 005/005   10   4.9   0.5
PASSEIRY    N4609.8 0002 ...  78   331         P004  P05
PAS        E00600.0 0002 ...   6   232  276          436  ....  ....
";

    #[test]
    fn test_parse_coordinates() {
        assert!((parse_lat("N4614.3").unwrap() - 46.238333).abs() < 1e-6);
        assert!((parse_lat("S4614.3").unwrap() + 46.238333).abs() < 1e-6);
        assert!((parse_lon("E00606.6").unwrap() - 6.11).abs() < 1e-6);
        assert!((parse_lon("W00232.9").unwrap() + 2.548333).abs() < 1e-6);
        assert_eq!(parse_lat("LAT"), None);
        assert_eq!(parse_lat("N4675.0"), None); // minutes >= 60
        assert_eq!(parse_lon("E00606"), None); // too short
    }

    #[test]
    fn test_parse_full_log() {
        // The waypoint scan must not be confused by the other briefing sections
        let wps = parse_flight_log(BRIEFING).unwrap();
        assert_eq!(wps.len(), 17);
        assert_eq!(wps.first().unwrap().ident, "LSGG");
        assert_eq!(wps.last().unwrap().ident, "LFPG");
    }

    #[test]
    fn test_parse_waypoint_fields() {
        let wps = parse_flight_log(BRIEFING).unwrap();

        // PASSEIRY: full climb waypoint (no TAS printed during climb)
        let pas = &wps[1];
        assert_eq!(pas.ident, "PAS");
        assert!((pas.lat - 46.163333).abs() < 1e-6);
        assert!((pas.lon - 6.0).abs() < 1e-6);
        assert_eq!(pas.altitude_ft, Some(5000.0));
        assert_eq!(pas.gs_kts, Some(276.0));
        assert_eq!(pas.tas_kts, None);
        assert_eq!(pas.wind_comp_kts, Some(4.0));
        assert_eq!(pas.cum_time_min, Some(2));

        // Top of climb: ident falls back to the position name
        let toc = wps.iter().find(|w| w.ident == "T O C").unwrap();
        assert_eq!(toc.altitude_ft, Some(30000.0));
        assert_eq!(toc.tas_kts, Some(465.0));
        assert_eq!(toc.gs_kts, Some(441.0));

        // KELUK: headwind component
        let keluk = wps.iter().find(|w| w.ident == "KELUK").unwrap();
        assert_eq!(keluk.wind_comp_kts, Some(-38.0));

        // FIR boundary rows are skipped - their printed coordinates are
        // unreliable and can fold the path back on itself
        assert!(wps.iter().all(|w| !w.ident.starts_with('-')));

        // Destination: cumulative time is total flight time
        assert_eq!(wps.last().unwrap().cum_time_min, Some(45));
    }

    #[test]
    fn test_parse_full_briefing() {
        let b = parse_briefing(BRIEFING).unwrap();

        assert_eq!(b.waypoints.len(), 17);
        assert_eq!(b.callsign.as_deref(), Some("ALU"));
        assert_eq!(b.registration.as_deref(), Some("N320SB"));
        assert_eq!(b.aircraft_type.as_deref(), Some("A320"));
        assert_eq!(b.icao_address.as_deref(), Some("1349"));
        assert_eq!(b.dep_runway.as_deref(), Some("LSGG/22"));
        assert_eq!(b.arr_runway.as_deref(), Some("LFPG/27R"));

        // V2 from the DRY RWY - PTOW - CALM WIND table, runway 22
        assert_eq!(b.v2_kts, Some(154.0));

        // VREF interpolated at PLDW 6319 between (6300, 133) and (6400, 134)
        let vref = b.vref_kts.unwrap();
        assert!((vref - 133.19).abs() < 0.01, "vref = {vref}");
    }

    #[test]
    fn test_parse_wind_information() {
        let b = parse_briefing(BRIEFING).unwrap();

        let names: Vec<&str> = b.wind_profiles.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(
            names,
            ["CLIMB", "T O C", "TINIL", "FF302", "NANOP", "T O D", "DESCENT"]
        );

        let climb = &b.wind_profiles[0];
        assert_eq!(climb.levels.len(), 5);
        let top = &climb.levels[0];
        assert_eq!(top.fl, 350);
        assert_eq!(top.dir_deg, 314.0);
        assert_eq!(top.speed_kts, 37.0);
        assert_eq!(top.oat_c, -51.0);

        let descent = &b.wind_profiles[6];
        assert_eq!(descent.levels.len(), 5);
        assert_eq!(descent.levels[4].oat_c, 4.0); // "100 306/037 +04"
    }

    #[test]
    fn test_extract_only_degrades_gracefully() {
        let b = parse_briefing(EXTRACT).unwrap();
        assert_eq!(b.waypoints.len(), 2);
        assert_eq!(b.callsign, None);
        assert_eq!(b.registration, None);
        assert_eq!(b.aircraft_type, None);
        assert_eq!(b.icao_address, None);
        assert_eq!(b.dep_runway, None);
        assert_eq!(b.arr_runway, None);
        assert_eq!(b.v2_kts, None);
        assert_eq!(b.vref_kts, None);
        assert!(b.wind_profiles.is_empty());
    }
}
