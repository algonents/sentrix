//! Agent mode: stateful, kinematic simulation. Each aircraft integrates its
//! own state toward per-leg targets via `Aircraft::step(dt)`, following its
//! flight plan (LNAV). Developed independently of replay mode (see
//! docs/SIMULATION.md); the two share only `crate::shared`, never an execution
//! loop.
//!
//! Phase 2 scope: uncleared agents flying their plans. Scenarios (Phase 3) and
//! the clearance channel (Phase 4) build on top of this.

pub mod aircraft;
pub mod performance;
pub mod run;
