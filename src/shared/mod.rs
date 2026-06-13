//! Mode-agnostic infrastructure shared by every CAT-062 source.
//!
//! Nothing here knows *how* aircraft state is produced (replay sampling, live
//! polling, agent integration) — only how to parse OFP briefings, do geometry,
//! build the common parts of a CAT-062 record, load config, and publish. The
//! per-mode drivers live in their own modules and depend on this one; this one
//! depends on no mode.

pub mod cat062;
pub mod config;
pub mod geo;
pub mod lido;
pub mod plan;
pub mod publisher;
