//! Replay mode: deterministic playback of SimBrief OFP briefings as CAT-062.
//!
//! A bounded, standalone capability — parse briefings into time-indexed flight
//! paths and sample them on a tick. It has no scenario or clearance concept;
//! those belong to the (independent) agent mode.

pub mod run;
pub mod sampler;
