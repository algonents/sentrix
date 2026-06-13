# Sentrix Simulation Engine

This document describes how the simulation engine works today, the agreed
direction (scenario generation including conflicts, with a feedback loop
through which a controller submits clearances), the migration analysis showing
what survives each step, and the phased plan. It is the context-recovery
artifact for this effort: when returning to the problem, read this first.

Last updated: 2026-06-13.

## Current model: timeline replay

Simulation mode (`cargo run -- --simulate <bulletin>`) is a clock-driven
**replay** of a precomputed timeline. The entire flight is determined at
startup; the loop only looks up where the plan says the aircraft is at each
moment. The simulation accepts **no runtime inputs** — the only input is the
wall clock. There is no way to change ground speed, altitude, or heading once
the replay has started.

### Startup (once)

1. `run_simulation` (`main.rs`) reads the SimBrief LIDO OFP bulletin and
   parses it with `lido::parse_bulletin` into a `LidoBulletin`: the waypoint
   list from the FLIGHT LOG section (ident, lat/lon, FL, TAS, GS, wind
   component per row) plus optionals — callsign, Mode-S code, runways,
   V2/VREF, winds aloft. FLIGHT LOG is the only mandatory section; a
   flight-log-only extract parses with all optionals `None`. Waypoint blocks
   are located by their LAT/LON column patterns (fixed-width slices — column
   positions are load-bearing).
2. `FlightPath::from_bulletin` (`simulation.rs`) builds the timeline:
   - V2/VREF profile points are synthesized onto the first and last legs:
     lift off at V2 and accelerate towards 250 kt; decelerate through
     ~250/200 kt stages down to VREF on final, with altitude capped to a
     ~3° glide near the runway. Path geometry is never altered — synthetic
     points lie on the existing legs.
   - Missing GS/altitude values are forward/back-filled from neighbours;
     the endpoint airports (no FL in the log) are treated as 0 ft.
   - A single pass accumulates time: each leg's duration = haversine
     distance ÷ average of the two endpoint ground speeds. This is
     deliberately **not** the bulletin's TTLT column, which is
     minute-resolution and would create zero-duration segments. Every
     waypoint becomes a `PathPoint` with an absolute `time_s` from departure
     and its great-circle bearing to the next point.

### The publish loop (every `poll_interval_secs`)

3. The loop captures `Instant::now()` at start; each tick calls
   `path.sample(start.elapsed())`.
4. `sample` finds the segment containing that elapsed time and linearly
   interpolates lat, lon, altitude, GS and TAS between the segment's
   endpoints. Track is held constant per segment — the aircraft flies
   straight lines between fixes and snaps to a new heading at each one.
5. The state is converted to a `Cat062Record` (track number hashed from the
   icao_address, GS+track converted to Cartesian vx/vy, callsign and Mode-S
   address attached), encoded with `encode_cat062_block`, and sent as one
   UDP datagram. One record, one block, one datagram per tick.
6. Past the last point, `sample` returns the destination with zero speed and
   `ended = true`; the loop publishes that frozen position indefinitely.

### Properties and limitations

- **Time is the only input.** The simulation has no state besides the start
  instant — at a given elapsed time the output is always identical. This is
  what makes it a replay, and why a mid-flight clearance is impossible in
  this model: there is no current state to act on, only a lookup into a
  fixed table.
- **Real time only.** Wall-clock drives the sample; a 45-minute flight takes
  45 minutes. No acceleration or jump-to-time yet (cheap to add — see
  phase 1).
- **One aircraft.** One path, one identity, one record per datagram.
- **Minimal CAT-062 records.** Only position, altitude, vx/vy, callsign,
  track number and time-of-day are populated; `track_status` is hardwired
  to `0x00`. No ROCD (I062/220), no mode-of-flight, no coasting/terminated
  flags.
- **Acceleration is implicit and slightly inconsistent.** On a segment whose
  endpoint speeds differ (e.g. 150→250 kt), three mechanisms disagree:
  the leg *duration* uses the average speed (correct for uniform
  acceleration); the *reported* GS is lerped linearly in time (which
  describes uniform acceleration); but the *position* is also lerped
  linearly in time, i.e. the aircraft actually moves at the constant
  average speed for the whole segment (true acceleration would make
  position quadratic in time). Reported velocity therefore deviates from
  the positional derivative by up to **half the per-leg speed change**
  (±50 kt in the example), peaking at segment boundaries. Invisible in
  cruise (near-constant GS) and bounded on the short synthesized
  departure/arrival legs, but a tracker or smoothing filter comparing
  vx/vy against successive positions will see a small systematic bias on
  accelerating segments. Deliberately not fixed in replay (it would be
  investment inside `sample(t)`): the phase 3 agent erases it by
  construction, since `step(dt)` integrates position *from* the current
  GS, making reported speed and positional derivative identical by
  definition, with acceleration an explicit rate limiter.

## Direction

### Goal

Evolve sentrix from a single-flight replayer into a **scenario generator for
testing surveillance/ATC systems**: many controllable flights, specific
scenarios such as conflicts, and a feedback loop where a controller issues
clearances (climb/descend, headings, direct-to, speed) that the simulated
aircraft execute. The near-term driver is **debugging complex visualization
scenarios** (CWP-style consumers of the CAT-062 feed); the controller loop is
the long-term capability.

### SimBrief OFPs as templates

We do not build our own flight-profile generator and we do not generate route
geometry. SimBrief profiles are built from real aircraft performance, real
routes and real forecast winds; anything we synthesized would be a less
faithful imitation. Producing an OFP is however a manual per-flight workflow,
so OFPs become **templates**: one bulletin can be instantiated many times with
variations.

Variations that preserve realism (they transform profile columns, never
invent geometry):

- **Identity** — callsign / icao_address / registration per instance.
- **Start offset** — staggered departures; negative offsets spawn a flight
  already en route via `sample(offset + elapsed)`.
- **Speed scaling** — scale the GS/TAS columns by a factor; the timeline
  recomputes automatically because time is derived from distance ÷ GS.
  Design rule: apply the factor to the waypoint GS columns **before**
  V2/VREF profile synthesis and leave V2/VREF untouched, so takeoff and
  landing speeds stay honest while the en-route portion scales.
- **Cruise level shift** — shift the cruise plateau (the max-altitude run of
  waypoints) ±1000/2000 ft.
- **Lateral offset** — small parallel offsets (SLOP-like, 1–2 nm).

Out of scope by decision: reversing routes, splicing templates, waypoint
perturbation — that is route generation, which stays external (it would
require a navigation database for airways/SIDs/STARs).

A scenario is a config file listing template instances, roughly:

```toml
[[flight]]
template = "simulations/lsgg_lfpg.txt"
callsign = "SWR11A"
icao_address = "4b17e1"

[[flight]]
template = "simulations/lsgg_lfpg.txt"
callsign = "SWR22B"
icao_address = "4b17e2"
start_offset_s = 300     # departs 5 min behind...
speed_factor = 1.20      # ...20% faster GS: catches up on SWR11A
cruise_shift_ft = 1000   # offset level, else the overtake is a collision
```

Caveats:

- `icao_to_track_number` hashes icao_address to 12 bits; scenario load must detect
  track-number collisions and reject or remap — two flights sharing a track
  number would corrupt downstream tracker tests.
- Two instances of the same template share the exact lateral path: an
  overtake without `cruise_shift_ft` or a lateral offset is a scripted
  collision. Sometimes that is the intent — it just has to be a knob, not an
  accident.

### Criteria-based generation: the scenario solver

Replay timelines are **closed-form functions of time** — every flight's
position(t) is known before anything runs. This makes criteria-based scenario
generation a solver problem, not a simulation problem:

- *"Where/when does separation between these two flights drop below 5 nm?"*
  — walk both timelines, report closest approach. No simulation run needed.
- *"Make the overtake happen at waypoint PAS"* — solve for the
  `start_offset_s` that co-locates both aircraft at PAS.
- *"Crossing conflict at fix X at the same FL, 600 s into the scenario"* —
  solve each flight's offset so both reach X at t=600; `cruise_shift_ft`
  puts them at the same level.

Catch-up/in-trail scenarios (same path, one aircraft faster) are pure
configuration; the solver turns hand-tuned offsets into declarations. Build
the solver **on the replay model** — in the agent model trajectories are no
longer closed-form and the same questions get much harder. Solver outputs
(offsets, shifts) remain valid as agent initial conditions.

### What replay supports well — and what it cannot

**Phase 1 (multi-flight replay) covers ~80% of visualization debugging:**

- **Determinism is the killer feature**: a scenario reproduces identically
  every run, so a rendering bug ("label overlap when SWR12K crosses AFR332
  near PAS") becomes a repeatable test case. Live OpenSky data can never do
  this, and even the agent model is less deterministic once interactive
  clearances enter.
- Density/clutter (label anti-overlap, track tables, render performance at
  50–200 targets), geometric edge cases on demand (crossings, convergence,
  pop-in via negative offsets), full climb/descent profiles.

Known gaps for visualization debugging, all cheap, all carrying over to the
agent model (they are loop-level, not `sample`-level, so they do not violate
the replay freeze):

1. **Time control** — accelerate / jump-to-time by scaling or offsetting the
   elapsed clock in the loop. Multiplies the value of everything else
   (debugging an event 25 min into a scenario must not cost 25 min per
   iteration).
2. **CAT-062 field audit** — whatever fields the consumer renders beyond our
   minimal set go untested (ROCD, mode-of-flight, coasting flags). Audit the
   consumer's reads against our writes. Remember the libasterix gap:
   `icao_address` never reaches the wire (see CLAUDE.md).
3. **Track lifecycle** — flights currently never end (hold last position
   forever); add a terminate-after-arrival option so track drop/coast
   rendering is exercised.
4. **Heading snaps at waypoints** — instantaneous track changes make speed
   vectors jump. Cosmetic; fixed for free by the agent's turn dynamics, not
   worth fixing in replay.

**What replay structurally cannot do: controller in the loop.** Everything
above operates *before t=0* — it authors a fixed future and plays it. A
clearance requires the future to be revisable at any t. Rejected hack: faking
clearances by rebuilding the remaining timeline from the current interpolated
position — it almost works for altitude/speed changes but collapses for
heading vectors (an assigned heading has no waypoint geometry to build a
timeline from), and amounts to a worse intent model inside replay code. The
division of labor is: phase 1 + solver *create* the situation; the agent +
clearance channel let someone *fix* it.

### Clearance feedback loop: replay → agent

The execution core changes from a time-indexed lookup to a **stateful
integrating agent**: each aircraft holds current state (position, altitude,
GS, track) plus *intent* (cleared level, route or assigned heading, target
speed), and a `step(dt)` advances it each tick.

The physics stays simple and **ground-speed based** — four rate limiters, no
mass/thrust/drag, no BADA (deliberately rejected as over-complicated for
cruise flight):

- position integrates along the current track at GS
- altitude moves toward the cleared level at a capped rate (~1,500–2,500 fpm)
- track turns toward the target at standard rate (3°/s)
- GS moves toward the target speed at ~0.5–1 kt/s

The subtle parts are LNAV (waypoint sequencing and turn anticipation so the
aircraft cuts corners like an FMS) and rejoin-route behaviour after a heading
vector.

`FlightPath` is promoted from *executor* to *flight plan*: the route the
agent follows by default and the source of its GS/altitude targets per phase
(speed targets come from the template's own GS column, not a performance
model). Uncleared, an agent flying its plan should produce nearly the same
output as the replay — which doubles as the regression test for the
migration.

Clearance types: `CFL` (cleared flight level), `HDG` (assigned heading),
`DCT` (direct to waypoint), `SPD` (assigned speed). Each writes a target into
the agent's intent.

The command channel rides on the existing Tokio loop: `tokio::select!` over
the tick timer and a command source (UDP or TCP line protocol), with a simple
text protocol such as `SWR123 CFL 360`. Anything — a script, a human with
netcat, a controller working position — can act as the controller.

### Migration durability: what survives, what is replaced

The total planned write-off across all phases is **zero** (originally
bounded to `FlightPath::sample()`, ~50 lines, until the coexistence
decision below retained it as a permanent mode). Layer by layer:

| Layer | Contents | Fate in agent migration |
|---|---|---|
| Input | `lido.rs` (~600 lines, fixed-width parsing — the most fragile/expensive code in the repo) | Untouched; agents need flight plans |
| Plan | `FlightPath` construction: gap-filling, V2/VREF synthesis, distance÷GS timeline, bearings | Survives with a role change: from *answer* ("where am I at t") to *intent* (route + GS/alt targets). `time_s` stays useful for the solver and ETAs |
| Execution | `FlightPath::sample()` interpolation | Joined by `Aircraft::step(dt)` as a second, coexisting mode (see below); replay stays frozen |
| Output | `Cat062Record` conversion, `encode_cat062_block`, `publisher.rs` | Untouched; indifferent to interpolation vs integration |
| Scenario | TOML config, variations, multi-flight loop, solver | Untouched; describes initial conditions + flight plans, exactly what agents consume. Loop changes one call: `path.sample(t)` → `aircraft.step(dt)` |
| Loop extras | Time control, command channel, track lifecycle | Loop-level; carry over |

The discipline that keeps this true: **no new features inside `sample(t)`**.
New capability attaches to the plan, the scenario, the loop, or the output —
never to the interpolation.

### Mode coexistence (decided 2026-06-13)

Replay and agent execution **coexist permanently**; replay is not retired
after the agent lands (this amends the original phase 3 plan, which deleted
`sample()` after the parity test). Rationale:

- Replay is the **deterministic gold standard**: identical output every run,
  a property agent mode cannot fully promise once interactive clearances
  exist. Visualization regression testing keeps relying on it.
- Permanent parity checking: any future agent change can be validated
  against replay on the same plan, not just once during migration.
- Cost is near zero precisely because of the `sample(t)` feature freeze —
  replay stays ~50 frozen lines.

Shape: both modes consume the same plan (`FlightPath`) and emit the same
per-tick state, so the loop needs one seam:

```rust
enum FlightSim {
    Replay(FlightPath),   // state = path.sample(elapsed)
    Agent(Aircraft),      // state = aircraft.step(dt)
}
```

Mode is selected per flight (scenario config field; CLI default), which
enables **mixed scenarios** — e.g. 20 replayed flights as deterministic
background traffic plus one agent-mode aircraft under control: the natural
controller-in-the-loop demo. Boundary semantics: a clearance addressed to a
replay-mode flight is rejected with an explicit error, never silently
ignored.

### Phasing

Ordering rationale: each phase consumes the previous one's output and nothing
is rework. Alternative ordering is legitimate if the controller demo becomes
the priority: do phase 3+4 for a single aircraft first, then phase 1 — the
phasing minimizes rework but is not sacred.

#### Phase 1 — Multi-flight scenario player (replay-based; days)

- [ ] Scenario TOML (`--scenario <file>`, keeping `--simulate <bulletin>` as the
  single-flight shortcut): per-flight `template`, `callsign`, `icao_address`,
  `start_offset_s`, `speed_factor`, `cruise_shift_ft`, `lateral_offset_nm`.
- [ ] Variations implemented as transforms on the waypoint columns before
  `FlightPath` construction; `speed_factor` applied before V2/VREF synthesis.
- [x] Multi-flight loop: a `Vec` of (path, identity, offset) instances, sample
  each per tick, batch all records into **one** `encode_cat062_block` per
  tick (live mode already proves multi-record blocks downstream).
  *(via `--simulate <path>...`; the `offset` field awaits the scenario TOML.)*
- [x] icao_address → 12-bit track-number collision detection at scenario load
  (reject or remap).
- [ ] For visualization debugging, include: time control (scale + start offset),
  terminate-after-arrival option, and the CAT-062 field audit against the
  actual consumer.

#### Phase 2 — Scenario solver (replay timelines are closed-form; days)

- [ ] Closest-approach report between any two flights (where/when separation
  drops below a threshold).
- [ ] Offset solving: co-locate two flights at a chosen fix / chosen time
  (overtake at PAS, crossing conflict at X at t=600).
- [ ] Build on replay before the agent migration; outputs remain valid as agent
  initial conditions.

#### Phase 3 — Kinematic agent (the structural change; up to a week)

- [ ] `Aircraft` with state (lat, lon, alt_ft, gs_kts, track_deg) + intent
  (cleared FL, LNAV-route-or-assigned-heading, target speed) + `step(dt)`
  with the four rate limiters.
- [ ] Optional fidelity upgrade: source the rate limits per aircraft type from
  **OpenAP/WRAP** (TU Delft, LGPL-3.0, https://github.com/TUDelft-CNS-ATM/openap)
  instead of hardcoded constants. WRAP is a purely kinematic model (speed,
  altitude, vertical rate per flight phase, derived from ADS-B data) — the
  same modeling philosophy as the agent, so it does not reopen the no-BADA
  decision. The bulletin already provides the aircraft type; the WRAP data
  tables are portable to Rust static tables (mind LGPL attribution if
  embedded). This matters most for clearances: a mid-flight "climb FL360"
  has no SimBrief profile to follow, so per-type rates are what make the
  maneuver realistic. Assessed 2026-06-12 as not worth doing earlier than
  this phase — in replay, intra-leg precision gains are invisible to the
  consumer. Related prior art: **BlueSky** (same TU Delft group), an open
  ATM simulator whose command stack (ALT/HDG/SPD/DCT) closely matches our
  phase 4 protocol — a good reference for clearance semantics and LNAV.
- [ ] LNAV: waypoint sequencing, turn anticipation, rejoin-route after vector.
- [ ] `FlightPath` becomes the plan; targets derived from it per phase.
- [ ] **Regression test**: uncleared agent flying its plan matches replay output
  within tolerance. Replay is **not** deleted afterwards — it remains a
  permanent coexisting mode (see "Mode coexistence" above); the parity test
  becomes a standing check rather than a one-off migration gate.
- [ ] Tick-rate detail: at 3°/s a 5 s publish tick is a 15° heading jump —
  integrate internally at ~1 s sub-steps, publish every
  `poll_interval_secs`.

#### Phase 4 — Clearance channel (days)

- [ ] `tokio::select!` over tick timer + command source (UDP or TCP lines).
- [ ] Text protocol: `<CALLSIGN> CFL <fl> | HDG <deg> | DCT <wpt> | SPD <kts>`,
  with simple ack/error replies.
- [ ] This phase completes the controller-in-the-loop story: solver authors the
  conflict → agents fly it → CWP displays it → controller sends
  `SWR22B SPD 250` → the agent's intent updates and the situation resolves.

### Decisions log

- 2026-06-12 — No own flight-profile engine; SimBrief OFPs remain the source
  of route geometry and nominal profiles (used as templates).
- 2026-06-12 — Physics stays GS-based kinematics; BADA-style point-mass
  models rejected.
- 2026-06-12 — Variations transform profile columns only; route geometry is
  never invented.
- 2026-06-12 — `sample(t)`-based features are frozen: the agent model
  (phase 3) retires interpolation as the execution model. Loop-level
  features (time control, track lifecycle, command channel) are exempt —
  they carry over.
- 2026-06-12 — Rejected: faking clearances in replay by rebuilding the
  remaining timeline from the current position (collapses for heading
  vectors; duplicates the intent model badly).
- 2026-06-12 — Scenario solver is built on replay timelines (closed-form)
  before the agent migration; in the agent model the same questions are
  much harder.
- 2026-06-12 — `speed_factor` applies to waypoint GS columns before V2/VREF
  profile synthesis; V2/VREF are never scaled.
- 2026-06-12 — Migration write-off is bounded to `FlightPath::sample()`
  (~50 lines); everything else (parser, plan construction, output path,
  scenario layer) carries over to the agent model.
- 2026-06-13 — FIR/UIR boundary rows (`-`-prefixed idents) are skipped by
  the parser: they are airspace annotations with unreliable printed
  coordinates (can fold the path back on itself, observed as a course
  deviation blip on LSGG→LSZH).
- 2026-06-13 — **Mode coexistence**: replay and agent execution coexist
  permanently, selectable per flight; mixed scenarios (replayed background
  traffic + agent-mode controlled aircraft) are a supported configuration.
  Amends the 2026-06-12 phase 3 plan to delete `sample()` after parity —
  the migration write-off drops to zero and the parity test becomes a
  standing check. Clearances addressed to replay-mode flights are rejected
  explicitly.
