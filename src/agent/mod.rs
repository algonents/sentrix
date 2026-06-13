//! Agent mode: stateful, kinematic simulation — future home of the
//! `Aircraft::step(dt)` engine, agent-executed scenarios, and the clearance
//! channel. Developed independently of replay mode (see docs/SIMULATION.md);
//! the two share only `crate::shared`, never an execution loop.
//!
//! Intentionally empty for now — a signpost for where the agent engine lands.
