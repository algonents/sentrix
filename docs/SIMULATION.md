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
  Phase 1).
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
  investment inside `sample(t)`): the agent phase erases it by
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

The architecture splits cleanly in two. **Replay** stays a bounded,
deterministic playback engine — play given bulletins concurrently, control time
(accelerate, jump-to-start), nothing more. **Controllable flights, scenarios and
clearances are agent-based**: a scenario is something *agents execute*, not
something replay authors. The sections below describe the agent era; replay's
own roadmap is just time control (Phase 1).

### SimBrief OFPs as templates

*(Agent era — Phase 3. Replay has no scenario concept; this is the scenario
model the agents consume.)*

We do not build our own flight-profile generator and we do not generate route
geometry. SimBrief profiles are built from real aircraft performance, real
routes and real forecast winds; anything we synthesized would be a less
faithful imitation. Producing an OFP is however a manual per-flight workflow,
so OFPs become **templates**: one bulletin can be instantiated many times with
variations.

Variations preserve realism — they set each agent's **initial intent** at
spawn, never invent geometry:

- **Identity** — callsign / icao_address / registration per instance.
- **Start offset** — staggered departures; a flight can also spawn already en
  route by initialising the agent partway along its plan.
- **Speed scaling** — a different initial target speed (in replay terms, the
  en-route GS the agent aims for). V2/VREF stay untouched, so takeoff and
  landing speeds remain honest while the en-route portion scales.
- **Cruise level shift** — a different initial cleared FL for the cruise plateau
  (±1000/2000 ft).
- **Lateral offset** — a small parallel-route offset (SLOP-like, 1–2 nm).

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

*(Part of Phase 3.)* A Phase 3 scenario is **open-loop**: with no clearances,
every agent's trajectory is determined by its plan before anything runs —
effectively a closed-form function of time. That makes criteria-based scenario
generation a solver problem, not a simulation problem:

- *"Where/when does separation between these two flights drop below 5 nm?"*
  — walk both trajectories, report closest approach. No run needed.
- *"Make the overtake happen at waypoint PAS"* — solve for the start offset
  that co-locates both aircraft at PAS.
- *"Crossing conflict at fix X at the same FL, 600 s into the scenario"* —
  solve each flight's offset so both reach X at t=600; an initial cleared-FL
  shift puts them at the same level.

Catch-up/in-trail scenarios (same route, one aircraft faster) are pure
configuration; the solver turns hand-tuned offsets into declarations. The
tractability holds **only while the scenario is open-loop** — once the
clearance channel (Phase 4) perturbs a running scenario, trajectories are no
longer predetermined and the same questions get much harder. Solver outputs
(offsets, shifts) are the scenario's initial conditions.

### What replay supports well — and what it cannot

**Deterministic playback covers much of visualization debugging:**

- **Determinism is the killer feature**: a given set of bulletins reproduces
  identically every run. Constructing a *specific* geometry ("label overlap
  when SWR12K crosses AFR332 near PAS") is a Phase 3 scenario — but once
  authored it too replays bit-identically, because open-loop execution is
  deterministic. Live OpenSky data can never do this, and even the agent model
  loses determinism once interactive clearances enter.
- Pure replay already exercises density/clutter (label anti-overlap, track
  tables, render performance at 50–200 targets from real OFPs) and full
  climb/descent profiles; *constructed* edge cases (crossings, convergence)
  come with Phase 3 scenarios.

Remaining loop-level items (all cheap, all carrying over to the agent model —
they are loop-level, not `sample`-level, so they do not violate the replay
freeze):

1. **Time control** — accelerate / jump-to-time by scaling or offsetting the
   elapsed clock in the loop. Multiplies the value of everything else
   (debugging an event 25 min into a scenario must not cost 25 min per
   iteration). **This is Phase 1** (see Phasing).
2. **CAT-062 field audit** — whatever fields the consumer renders beyond our
   minimal set go untested (ROCD, mode-of-flight, coasting flags). Audit the
   consumer's reads against our writes. Remember the libasterix gap:
   `icao_address` never reaches the wire (see CLAUDE.md).
3. **Track lifecycle** — by decision, an arrived flight just holds its last
   (ground) position indefinitely; no track-termination or coast handling is
   added (not worth the complexity for a visualization feed).
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
division of labor is: scenarios (Phase 3) *create* the situation; the clearance
channel (Phase 4) lets someone *change* it.

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
| Multi-flight & scenarios | Multi-flight loop (shipped); scenario authoring + solver (Phase 3) | The loop is execution-agnostic — it batches one record per flight whether `sample(t)` or `step(dt)` produced it. Authoring + solver are built in the agent era and consume agent intent directly, so there is nothing to write off |
| Loop extras | Time control (Phase 1), clearance command channel (Phase 4) | Loop-level; mode-agnostic, carry over |

The discipline that keeps this true: **no new features inside `sample(t)`**.
New capability attaches to the plan, the scenario, the loop, or the output —
never to the interpolation.

### Mode coexistence (decided 2026-06-13)

Replay and agent execution **coexist permanently**; replay is not retired
after the agent lands (this amends the original agent-phase plan, which deleted
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

Replay itself has **no scenario concept**: it plays one or more bulletins
concurrently (shipped) with time control on top (Phase 1), and nothing more.
All multi-flight situation authoring lives in the agent era — a conflict is
only meaningful once something can *act* on it.

Ordering follows two principles — do what is *unblocked* first, and add
*non-determinism last*:

- **Time control (Phase 1)** needs only the current replay loop and multiplies
  the value of every later phase, so it comes first.
- **The agent (Phase 2)** is the structural change everything situational
  depends on, so it precedes scenarios.
- **Scenarios (Phase 3)** feed many agents a *given* situation they execute
  open-loop — no intervention, so it reproduces identically every run.
- **The clearance channel (Phase 4)** adds the feedback loop that perturbs a
  *running* scenario. It is the only phase that introduces non-determinism, so
  it lands last.

The phasing minimizes rework but is not sacred; doing Phase 2 for a single
aircraft and jumping to a minimal clearance demo is legitimate if the
controller story becomes the priority.

#### Already shipped — replay engine

- [x] Multi-flight loop: a `Vec` of (path, identity) instances, sample each per
  tick, batch all records into **one** `encode_cat062_block` per tick
  (`--simulate <bulletin>...`).
- [x] `icao_address` → 12-bit track-number collision detection at load (reject
  or remap).

#### Phase 1 — Time control (loop-level; ~a day)

- [ ] `--speed <factor>` time-scale multiplier: the sim loop advances the
  elapsed clock by `tick × factor` instead of real seconds, so a 45-min flight
  replays in 4.5 min at `--speed 10`. Default `1.0` is today's real-time
  behaviour; live mode ignores it (OpenSky is inherently real-time).
- [ ] `--start-at <hms|secs>` jump-to-time: seed the elapsed clock with a
  **global** offset so the replay opens at, e.g., t=25 min — debug a late event
  without waiting (or scrubbing) to it. One clock for the whole replay, not
  per-flight.

Notes (not tasks):

- Both flags are loop-level (scale/offset the elapsed clock fed to `sample`);
  they touch neither `sample(t)` nor the agent migration, so they carry over
  verbatim once execution becomes agent-based.
- Granularity caveat: the publish tick stays `poll_interval_secs`, so at high
  `--speed` successive published positions are far apart (≈7 NM at jet speed for
  a 5 s tick × 10). Fine for visualization; drop `poll_interval_secs` if a
  smoother track is wanted.
- Track lifecycle stays minimal by decision: an arrived flight holds its last
  (ground) position indefinitely — no track-termination or coast handling.

#### Phase 2 — Kinematic agent (the structural change; up to a week)

- [ ] `Aircraft` with state (lat, lon, `altitude_ft`, `gs_kts`, `track_deg`) +
  intent (cleared FL, LNAV-route-or-assigned-heading, target speed) + `step(dt)`
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
  maneuver realistic. Related prior art: **BlueSky** (same TU Delft group), an
  open ATM simulator whose command stack (ALT/HDG/SPD/DCT) closely matches our
  clearance-channel protocol — a good reference for clearance semantics and LNAV.
- [ ] LNAV: waypoint sequencing, turn anticipation, rejoin-route after vector.
- [ ] `FlightPath` becomes the plan; targets derived from it per phase.
- [ ] **Regression test**: an uncleared agent flying its plan matches replay
  output within tolerance. Replay is **not** deleted afterwards — it remains a
  permanent coexisting mode (see "Mode coexistence" above); the parity test
  becomes a standing check rather than a one-off migration gate.
- [ ] Tick-rate detail: at 3°/s a 5 s publish tick is a 15° heading jump —
  integrate internally at ~1 s sub-steps, publish every `poll_interval_secs`.

#### Phase 3 — Scenarios: agents execute a given situation (open-loop; days)

- [ ] Scenario file (`--scenario <file>`): a list of flights, each = a SimBrief
  OFP `template` + identity + **initial intent** (start offset, initial cleared
  FL, initial target speed, route/lateral offset). The agents execute it
  autonomously — no clearances — so a scenario reproduces identically every run.
- [ ] Variations expressed as agent **intent** at spawn, not timeline
  transforms: cruise-level shift → initial cleared FL, speed scaling → initial
  target speed, lateral offset → parallel-route offset. Geometry is never
  invented (no route synthesis).
- [ ] Conflict authoring + solver: an uncleared agent's trajectory is still
  determined by its plan before anything runs (effectively closed-form), so
  criteria-based generation stays a *solver* problem — closest-approach report
  between two flights, and offset solving (co-locate at a fix / at a time:
  overtake at PAS, crossing at X at t=600).
- [ ] `icao_address` collision remap (shipped) applies at scenario load.

#### Phase 4 — Clearance channel: close the loop (days)

- [ ] `tokio::select!` over the tick timer + a command source (UDP or TCP lines).
- [ ] Text protocol: `<CALLSIGN> CFL <fl> | HDG <deg> | DCT <wpt> | SPD <kts>`,
  with simple ack/error replies. Each clearance writes a target into the
  addressed agent's intent, perturbing the running scenario.
- [ ] Completes the controller-in-the-loop story: Phase 3 authors the conflict →
  agents fly it open-loop → CWP displays it → controller sends `SWR22B SPD 250`
  → the agent's intent updates and the situation resolves. A clearance addressed
  to a replay-mode flight is rejected explicitly.

### Decisions log

- 2026-06-12 — No own flight-profile engine; SimBrief OFPs remain the source
  of route geometry and nominal profiles (used as templates).
- 2026-06-12 — Physics stays GS-based kinematics; BADA-style point-mass
  models rejected.
- 2026-06-12 — Variations never invent route geometry (only identity, speed,
  level, lateral offset). *Updated 2026-06-13:* expressed as each agent's
  **initial intent** at spawn, not as replay profile-column transforms, since
  scenarios are agent-era.
- 2026-06-12 — `sample(t)`-based features are frozen: the agent model
  retires interpolation as the execution model. Loop-level
  features (time control, command channel) are exempt — they carry over.
- 2026-06-12 — Rejected: faking clearances in replay by rebuilding the
  remaining timeline from the current position (collapses for heading
  vectors; duplicates the intent model badly).
- 2026-06-12 — Criteria-based scenario generation is a solver problem
  (closest-approach, offset solving) that exploits closed-form trajectories.
  *Updated 2026-06-13:* the solver is part of Phase 3 (agent-era); its
  tractability comes from scenarios being **open-loop** (uncleared agents are
  deterministic) and ends once clearances perturb a running scenario.
- 2026-06-12 — `speed_factor` scales en-route speed only; V2/VREF (takeoff/
  landing) are never scaled, so those stay honest. *Updated 2026-06-13:* in the
  agent era this is an initial target-speed setting, not a GS-column transform.
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
  Amends the 2026-06-12 agent-phase plan to delete `sample()` after parity —
  the migration write-off drops to zero and the parity test becomes a
  standing check. Clearances addressed to replay-mode flights are rejected
  explicitly.
- 2026-06-13 — **Time control is its own phase (Phase 1)**: a `--speed`
  time-scale multiplier + a global `--start-at` jump-to-time, both CLI flags,
  default `--speed 1.0`. Loop-level (scale/offset the elapsed clock, not
  `sample(t)`), so it carries over to the agent model. Excludes pause/step.
- 2026-06-13 — **Scenarios removed from replay.** Replay is a bounded,
  deterministic playback engine — concurrent multi-bulletin replay (shipped) +
  time control (Phase 1), nothing more. All situation authoring (templates,
  variations, solver, conflicts) is agent-era (Phase 3), because a conflict is
  only meaningful once something can *act* on it.
- 2026-06-13 — **Open-loop before closed-loop.** Phase order: time control (1)
  → kinematic agent (2) → scenarios the agents execute autonomously (3,
  deterministic) → clearance channel (4, the feedback loop that perturbs a
  running scenario). Non-determinism lands last.
- 2026-06-13 — **No track termination.** An arrived flight holds its last
  (ground) position indefinitely; no coast/drop handling — unnecessary
  complexity for a visualization feed.
