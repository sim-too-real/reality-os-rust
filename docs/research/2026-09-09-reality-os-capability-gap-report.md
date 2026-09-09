# Reality OS: Capability, Architecture, and Critical-Gap Report

**Research cutoff:** 2026-09-09  
**Scope:** source commit `c5c2c7b99804dcc2eb08cad5670f982d72969b50` on `main`; source files were clean and the generated report/Graphify artifacts were untracked at verification time. ROS 2 + MuJoCo are initial integration defaults; Unitree H1 and a generic 6-DoF arm are validation fixtures, not architectural assumptions.  
**Audience:** technical founders, robotics architects, controls/perception engineers, and safety reviewers.  
**Status:** research-backed architecture recommendation for approval. It is not a functional-safety certificate, a claim of metal readiness, or the final task-by-task implementation plan.

## Executive verdict

The codebase has a strong *thesis* and a useful safety-oriented skeleton, but it is not yet a general robotics platform. Its strongest idea is the separation between learned proposal, physical certification, a typed `CertifiedCommand`, a runtime governor, and a plant write boundary. The code is unusually honest about simulation versus hardware, names missing fieldbus and safety elements as holes, and has a clean, small Rust workspace that is easy to reason about.

That conceptual strength should be preserved. The present implementation, however, is still a compact simulation prototype: it has no complete robot dynamics model, estimator, geometric world model, motion planner, whole-body controller, production sensor pipeline, real-time executor, hardware driver, independent safety path, or closed-loop failure recovery. Several public APIs also allow the claimed last gate to be bypassed. A passing unit-test suite proves that the current contracts behave as tested; it does not prove physical correctness, deadline behavior, hardware exclusivity, fault containment, or safety.

Two ratings are therefore appropriate:

- **Safety-kernel research concept: 2.8/5.** The authority boundaries, typed decisions, honesty rules, and evidence vocabulary are a credible foundation.
- **Production-capable, general robot runtime: 1.0/5.** Most of the body, world, estimation, planning, control, hardware, and assurance stack is absent or represented by screens/stubs.

The recommended strategy is **a sovereign Rust kernel with replaceable robotics backends**. Reality OS should own the canonical body/world/scenario contracts, authority and lease model, evidence graph, runtime assurance, command arbitration, fault semantics, and final safe-write protocol. It should integrate—not recreate—ROS 2, `ros2_control`, Pinocchio, MoveIt/Tesseract, GTSAM, Crocoddyl, MuJoCo/Gazebo/Isaac Lab, and accelerator-specific perception. Learned models should propose goals, skills, subgoals, or short bounded references; deterministic components must validate, execute, monitor, recover, and stop.

The central first-principles correction is this:

> Robot-, environment-, floor-, and scenario-agnostic cannot mean “one default works everywhere.” It must mean that every assumption is explicit, versioned, measurable, and replaceable; every capability is discovered; uncertainty propagates; and an unsupported or out-of-distribution condition reduces authority to `PROBE`, `DEGRADE`, `REFUSE`, or `ABORT`.

## What was examined

The review combined direct source inspection, build/test verification, graph analysis, and current primary-source research.

- The workspace contains 14 Rust packages spanning kernel, physics, data, plant, governor, session, ROS schemas, embodiment, vision, rate scheduling, agent proposals, a gauntlet, and a CLI app.
- With `rustc 1.95.0` and `cargo 1.95.0`, `cargo test --workspace --all-targets` passes all 57 tests; `cargo clippy --workspace --all-targets -- -D warnings` and `cargo fmt --all -- --check` also pass. These commands were re-run on 2026-09-09.
- Graphify was **already installed**; it was not newly installed. It analyzed 88 supported files (about 25,321 words) and produced 1,202 nodes, 1,911 edges, and 77 communities. Its most connected abstractions are `CertifiedCommand`, `CommandLedger`, `EventStore`, `RuntimeIdentity`, `SimPlant`, and the write/governor path. The generated graph is at `graphify-out/graph.html`, with JSON and an audit report beside it.
- Primary sources were favored for current software status, papers, official model cards, standards scopes, and safety architecture. Vendor performance claims are treated as vendor-reported, not independent guarantees.

## Maturity scorecard

Scale: **0 absent**, **1 stub/screen**, **2 useful prototype**, **3 integrated engineering system**, **4 production-qualified**, **5 safety-assured for a declared application and operating domain**.

The scores are engineering judgments, not an arithmetic quality index. Each considers whether a layer is represented, implemented end to end, quantitatively measured on target hardware, fault-tested, and supported by assurance evidence. The concept score emphasizes the authority/evidence design; the production score weights the missing physical stack and validation much more heavily.

| Layer | Score | Present strength | Blocking gap |
|---|---:|---|---|
| Architectural doctrine | 3.0 | Clear separation of proposer, certifier, governor, and plant; explicit SIM ≠ metal | Claims exceed enforcement in several public APIs |
| Typed command authority | 2.0 | Certificates, command IDs, expiry, identity fields, ledger, narrowing vocabulary | Public bypass, mutable signed object, incomplete signature coverage, no hardware-rooted authority |
| Runtime safety | 1.0 | Freshness checks, latches, envelopes, refusal paths | No active independent watchdog, safe-drive function, FTTI analysis, or safety state machine |
| Robot embodiment | 1.0 | Four manifests and per-joint scalar limits | No link/frame tree, axes, inertias, geometry, transmissions, contacts, tools, sensors, uncertainty, or import fidelity |
| Environment / floor model | 0.5 | Gravity, friction, air-density examples | A few global scalars cannot represent surfaces, terrain, topology, deformability, traction, weather, dynamic agents, or local uncertainty |
| Physics and dynamics | 1.0 | Useful closed-form screens | No multibody/contact model, derivatives, parameter identification, or solver backend |
| State estimation | 0.0 | Sensor freshness/hash vocabulary | No time synchronization, calibration pipeline, fusion, base state, contact state, localization, health estimation, or factor graph |
| Vision and spatial intelligence | 0.5 | Minimal pixel-to-blob/unprojection and see-before-act concept | Caller-supplied booleans can spoof evidence; no camera validation, tracking, depth, segmentation, scene graph, SLAM, or uncertainty calibration |
| Task and motion planning | 0.5 | Domain plugin registry and tiny intent/plan types | No frames/units/targets/timing, collision planning, trajectory generation, TAMP, behavior execution, or resource locking |
| Whole-body control | 0.25 | Component-wise bounded-trust clamp | No inverse dynamics, QP hierarchy, contact/friction/stability constraints, MPC, feasibility handling, or recovery controller |
| Agentic intelligence | 1.0 | LLM output is intentionally proposal-only and forbidden fields are stripped | Fixed verb filter, no typed skill contracts, world grounding, independent success predicates, memory governance, or recovery loop |
| Real-time performance | 0.5 | Rate vocabulary and explicit SIM labeling | `work_us` is injected; no RT clock/executor, WCET measurement, bounded allocation, CPU isolation, overload policy, or deadline-triggered fallback |
| Hardware integration | 0.0 | `HardwareDriverPort` boundary and simulated port | Fieldbus is a named hole; no ROS node/controller plugin, drive state machine, acknowledgement protocol, brakes, STO/SS1, or HIL |
| Fault tolerance | 0.75 | Refusal, abort/estop latches, journal-continuity idea | No fault-containment zones, redundancy model, active health manager, minimal-risk controller, or verified restart semantics |
| Observability / replay | 1.5 | Typed debug events, traces, command/event stores | Not deterministic or durable enough; no causal timeline, clock mapping, exact config/model/build capture, MCAP, tracing, or fleet diagnostics |
| Validation / assurance | 1.0 | Unit tests, gauntlet vocabulary, honest claims | No quantitative robotics benchmarks, differential simulation, fuzz/property campaign, HIL/fault injection, hazard traceability, or field evidence |

## What is genuinely strong today

### 1. A valuable authority model

The repository recognizes a distinction many robotics stacks blur: a model that proposes motion is not the component that may authorize motor power. `RealityOs::decide`, `CertifiedCommand`, `RuntimeGovernor`, `CommandLedger`, and `Plant` establish the beginnings of an authority chain. The provided Grok admission path strips selected authority claims, which is the right direction, but authority classification still relies partly on caller-controlled source strings and is not yet a type- or provenance-enforced trust boundary.

### 2. Fail-closed vocabulary and epistemic honesty

Typed statuses and violations make refusal reasons observable. The docs consistently label formula checks, PFL values, rates, and hardware paths as simulated. Naming the fieldbus and metal bridge as explicit holes is far better than hiding them behind a nominal interface.

### 3. Small, modular, testable Rust surface

The workspace is compact enough for full-context review. Modules have recognizable responsibilities, compilation is warning-clean, and the tests exercise identity, refusal, replay, certificates, sensor binding, and simulated dispatch. Rust is well suited to the kernel and critical protocol layers because ownership and strong types can eliminate classes of memory and aliasing failures—provided the runtime design also controls allocation, blocking, scheduling, and unsafe FFI.

### 4. A useful integration seam

`Plant`, `HardwareDriverPort`, domain plugins, and session composition are the beginnings of the correct seams. They should evolve into capability-negotiated contracts rather than be discarded. The key is to prevent arbitrary implementations from bypassing the authority path and to make semantics—units, frames, time, identity, acknowledgement, failure—far richer.

## Critical gaps in the current code

### P0: must be closed before any hazardous hardware can be energized

#### P0.1 — The software “last gate” is bypassable

`CertifiedWriteGuard::enter` and `with_certified_write` are public (`crates/plant/src/write_guard.rs:18-35`), `RuntimeGovernor.plant`, `.config`, `.envelope`, and `.signing_key` are public (`crates/governor/src/governor.rs:57-68`), and `HardwareBackedPlant.port` is public (`crates/plant/src/backed.rs:9-10`). `Plant` and `HardwareDriverPort` are also public, unsealed traits (`crates/plant/src/traits.rs:5-35`), so an arbitrary implementation can ignore the intended uncertified-write guard. A caller in the same process can enter the guard or reach/replace the port without following the session/governor path. The current uniqueness property is therefore unenforceable for the public abstraction, contradicting the README’s “SIM last-gate uniqueness” claim.

**Required correction:** make the actual actuator capability unforgeable and physically singular. Seal constructors and write tokens inside a private crate boundary; move the production gate to a separate process or safety controller; ensure the autonomy host has no alternate bus credentials, debug route, maintenance endpoint, or direct drive handle. Authority must come from topology, not call convention.

#### P0.2 — There is no active watchdog or autonomous safe transition

Heartbeat and sensor age are checked only when another write is attempted (`crates/governor/src/governor.rs:173-203`). If the process freezes while the actuator continues its previous command, no timer independently executes a stop. `engage_estop` calls a plant method but does not establish independent acknowledgement of the final safety element.

**Required correction:** add an independently clocked, windowed or challenge-response watchdog that controls a safety enable and supervises real progress. Define hazard-specific reactions—controlled stop, SS1, safe brake control, STO—not merely a zero command. Hazardous deployments need justified independence and fault containment derived from their hazard analysis; a separate safety PLC/MCU or safety-rated controller/drive path is a common realization, but the exact topology is product-specific. NASA’s Runtime Assurance/Simplex pattern similarly places a monitor and simpler backup controller around an untrusted advanced controller.[^1]

#### P0.3 — Unknown intent silently becomes actuator authority

The comment says unknown kinds refuse, but `DomainRegistry::infer_kind` maps the default case to `actuator_envelope` (`crates/reality-os/src/domains/mod.rs:94-109`), and that planner invents half of every declared torque limit (`crates/reality-os/src/domains/actuator.rs:14-20`). An unrecognized verb can therefore become a motor-space proposal.

**Required correction:** unknown intent must return an explicit unsupported result. No planner may synthesize a nonzero action from limits alone. Plans must specify a target, frame, units, mode, timing, provenance, preconditions, and completion predicate.

#### P0.4 — Numeric, dimensional, and time validation is incomplete

There are useful finite checks in newer paths, but limits and timestamps are not validated consistently. The actuator certifier reuses the last limit for extra action dimensions (`crates/reality-os/src/domains/actuator.rs:32-34`) instead of requiring exact dimensionality. Observation uncertainty can be absent or non-finite (`crates/reality-os/src/see.rs:46-55`). Governor freshness arithmetic accepts caller-provided floating time without a monotonic clock or rollback/future-time policy (`crates/governor/src/governor.rs:122-200`). `CertifiedCommand::issue` permits negative TTL and does not validate overflow in `now + ttl` (`crates/reality-os/src/command.rs:63-79`). The signing helper converts a non-finite time to `0.0`, while expiry uses a floating comparison that is false for `NaN` (`crates/plant/src/signing.rs:86-90`; `crates/plant/src/ledger.rs:233-235`). Public mutation can therefore create an effectively non-expiring malformed command. Similar malformed-limit behavior exists across scalar domain screens.

**Required correction:** introduce unit- and frame-typed finite scalars; exact-size state vectors tied to a model hash; validated positive bounds; a monotonic deadline type distinct from simulation and wall time; explicit time domains and clock mappings; and property/fuzz tests over NaN, infinities, signed zero, extreme magnitude, dimension mismatch, rollback, jumps, and wraparound.

#### P0.5 — “See before act” is attestable only by caller assertion

The decision path accepts `pixels_present` and `scene_compiled` booleans in `WorldView` (`crates/reality-os/src/domains/mod.rs:30-52`) while the core does not depend on the vision crate. A caller can set those flags without binding a frame, camera calibration, model version, image digest, scene epoch, covariance, or transform chain. Camera intrinsics/extrinsics and focal values are not comprehensively validated in `crates/vision/src/lib.rs`. The current sensor hash covers values but not timestamp, frame, sequence, sensor identity, or calibration; sensor ingest also fails to reject every non-finite sample/timestamp (`crates/session/src/session.rs:174-193`). `NaN` can bypass the staleness comparison, and valid values may be replayed or substituted across frames.

**Required correction:** replace booleans with a signed/hashed `ObservationEvidence` reference carrying sensor identity, calibration hash, capture and monotonic receive times, image/depth digests, transform-tree epoch, perception model hash, objects/geometry, covariance, quality/OOD scores, and expiry. Required geometric predicates must be recomputed or independently verified by the certifier.

#### P0.6 — Command integrity is partial and mutable

The HMAC payload binds IDs, time, issuer status/reason/action, and snapshot IDs, but omits live certificate state, live allowed action, actuator IDs, waypoints, release/as-built/calibration identity, sensor packet hash, parameters, units, frames, mode, and policy/config hashes (`crates/plant/src/signing.rs:14-25`). All `CertifiedCommand` fields are public (`crates/reality-os/src/command.rs:9-47`), and execution checks the live certificate (`crates/plant/src/execute.rs:42-45`) without requiring equality to the signed issuer status. A signed `REFUSE` can therefore be mutated to a live `ALLOW` without invalidating the current signature. The envelope also permits post-sign narrowing using an absolute-magnitude test that allows sign reversal (`crates/plant/src/signing.rs:39-49`).

**Required correction:** use immutable command/envelope types and a canonical binary serialization. Bind issuer, robot, actuator set, session/boot epoch, sequence, monotonic validity interval, mode, schema, units/frames, full proposal hash, model/calibration/policy/config hashes, evidence references, and allowed transformation rules. Represent narrowing as a separately signed derived command linked to its parent; use component/geometry-aware partial orders, never absolute magnitude alone. HMAC can authenticate a local trust domain, but production device identity, rotation, revocation, and non-repudiation requirements may require asymmetric keys and a hardware root.

#### P0.7 — The ledger cannot yet prove exactly-once physical action

The ledger consumes a command before calling the plant (`crates/plant/src/execute.rs:62-84`). A drive may accept motion and then return an error, producing an ambiguous “refused” result. The default ledger is in-memory; session start neither configures a journal nor applies continuity (`crates/session/src/session.rs:130-154`). Journal loading trusts stored chain hashes instead of recomputing every record and `prev_hash` link, accepts a missing file as an empty history, and has no explicit `fsync`, single-writer lock, transaction protocol, or robust partial-tail recovery (`crates/plant/src/ledger.rs:107-196`). Restored estop state can update the software latch without proving the hardware stop is engaged. Hash-chain-shaped data alone does not prove continuity or motor outcome.

**Required correction:** define an idempotent command protocol with prepare/commit/acknowledge semantics, drive sequence numbers, observed actuator state, and an explicit `UnknownOutcome` state. Persist anti-replay state and safety latches through an atomic journal/WAL with bounded recovery. Never auto-resend an ambiguous motion command.

### P1: required for an integrated engineering robot

#### P1.1 — The body model is a limit table, not an embodiment model

`RobotModel` stores ID, kind, source, and per-joint position/velocity/torque limits (`crates/embodiment/src/lib.rs:18-33`). It lacks the data required for kinematics or whole-body dynamics: a link/frame graph, joint types and axes, transforms, mass/inertia/center of mass, collision/visual geometry, transmissions and gear efficiency, motor/current/thermal models, sensors, tools, contact candidates, support polygons, payloads, uncertainty, and calibration lineage.

#### P1.2 — The environment model assumes one global “floor” physics tuple

`Environment` has global gravity, friction, air density, and a note (`embodiment/src/lib.rs:72-124`). Real floors and terrains are local, anisotropic, uncertain, deformable, discontinuous, and dynamic. A wheeled robot needs slope, roughness, step/ditch geometry, sinkage, slip and traction; a legged robot needs contact patches and compliance; an arm may have no floor relevance at all.

#### P1.3 — There is no estimator or calibrated belief state

The platform hashes samples and records freshness, but it does not synchronize sensors, estimate floating-base state, fuse IMU/encoder/vision/force data, infer contacts, localize, calibrate online, detect drift, estimate terrain/body parameters, or represent correlations. GTSAM provides a mature factor-graph substrate for sensor fusion and SLAM, including IMU preintegration, but leaves front-end association and domain modeling to the application.[^2]

#### P1.4 — There is no spatial world model

A capable robot needs simultaneous metric and semantic representations: transform graph, occupancy/voxel/ESDF/TSDF, meshes, objects and articulated parts, support and affordance surfaces, topology/routes, dynamic agents and predictions, traversability, no-go volumes, and provenance/uncertainty. NVIDIA nvblox and Open3D are useful backends for GPU and portable dense geometry respectively; neither should become the canonical world contract.[^3]

#### P1.5 — `PhysicalPlan` cannot express a physical plan

It currently contains only `kind`, `Vec<f64>`, and rationale (`crates/reality-os/src/plan.rs:3-22`). It cannot encode target/body frames, trajectory samples or splines, derivatives, timing, control mode, contacts, task constraints, collision margins, resources, terminal sets, tolerances, or contingencies.

#### P1.6 — “QP interception” is a component clamp, not whole-body control

The bounded-trust layer clamps components. It does not solve rigid-body dynamics or respect coupled constraints. A whole-body controller must reason over

\[
M(q)\dot v + h(q,v) = S^T\tau + J_c(q)^T\lambda,
\]

subject to contact consistency, friction cones, unilateral forces, torque/current/thermal/power limits, joint and Cartesian bounds, self/environment collision, balance/stability, task priorities, and actuator bandwidth. Pinocchio supplies fast rigid-body algorithms and derivatives; Crocoddyl supplies contact-aware optimal control; IHMC, TSID, `mc_rtc`, and PAL WBC provide useful whole-body architectural references.[^4]

#### P1.7 — Self-correction is a status, not a closed loop

`PROBE` can be returned, but no component selects an information-gathering action, predicts its value, executes it, updates belief, retries, replans, or verifies recovery. A robust loop is:

`Observe → Time-align → Estimate → Predict → Propose → Verify → Execute → Monitor → Diagnose → Probe/Replan/Degrade/Stop`.

Every transition needs independent predicates, deadlines, bounded retries, evidence, and an escape to a minimal-risk state.

#### P1.8 — The current “1 kHz” path is declarative

The rate crate compares a caller-injected `work_us` with a budget and explicitly labels itself simulation-only (`crates/rate/src/lib.rs:90-111`). The critical path allocates `Vec` and `String`, serializes JSON, hashes, appends files, and stores unbounded traces. ROS 2’s own real-time guidance requires preallocation, locked memory, predictable scheduling, avoidance of page faults and blocking operations, and measured missed-deadline behavior; the standard executors are not automatically deterministic.[^5]

#### P1.9 — ROS 2 and hardware are schemas, not integrations

The ROS crate defines codecs/chain state but no middleware node; the CLI reports the fieldbus as a named hole. There is no lifecycle-managed hardware adapter, `ros2_control` plugin, DDS security configuration, transform listener, sensor QoS path, drive state machine, or bus interface. This honesty is good, but it means the current code cannot control H1 or the arm.

### P2: required for learning, fleet scale, and durable generality

- **Agentic execution:** replace the fixed JSON verb list with a versioned `SkillIR` whose skills declare typed inputs/outputs, preconditions, invariants, effects, resource/authority needs, perception requirements, failure modes, recovery edges, observability, and qualification evidence.
- **Learning system:** add standardized synchronized episodes, demonstrations including failures/recoveries, policy registry, dataset lineage, offline evaluation, shadow execution, canary policies, rollback, and per-embodiment adaptation. LeRobot and Open X-Embodiment offer reusable schemas and policy baselines, not universal deployment semantics.[^6]
- **Security:** add secure/measured boot, hardware-protected device keys, signed manifests, least privilege, zones/conduits, key rotation/revocation, anti-rollback, A/B updates, and recovery. IEC 62443 and NIST SP 800-193 provide relevant lifecycle and firmware-resilience frames.[^7]
- **Debuggability:** record causal IDs, all clock domains, model/config/build hashes, exact inputs and derived beliefs, solver iteration/residual/feasibility, command/ack/observed motion, state transitions, health and dropped-data counters. Keep a bounded nonblocking RT ring; persist/visualize outside the critical path.
- **Validation:** replace Cartesian scenario multiplication with risk-driven coverage, falsification, randomized dynamics/sensors, metamorphic and cross-simulator tests, target-hardware timing, HIL, fault injection, restrained physical tests, and requirement-to-evidence traceability.

## What “agnostic” must mean

### Robot agnosticism

The kernel never special-cases H1, an arm, a quadruped, or wheels. It consumes a versioned `EmbodimentGraph` and dynamically discovers capabilities. A body adapter maps native representations (URDF/SRDF, SDF, MJCF, USD, vendor calibration) into canonical links, joints, frames, geometry, inertial properties, transmissions, actuators, sensors, tools, contacts, limits, timing, and uncertainty. Unsupported source features remain explicit diagnostics; lossy conversion must never disappear silently.

Each behavior asks for capabilities—such as `floating_base`, `differential_drive`, `cartesian_impedance`, `force_torque_sensing`, `bimanual`, `flight`, or `safe_torque_off`—rather than robot names. Unitree H1 and the 6-DoF arm become conformance suites that prove the same contracts work for radically different morphologies. Later fixtures should include a wheeled AMR, quadruped, and aerial system.

### Environment and floor agnosticism

There is no singleton “floor.” The world contains surfaces and volumes with spatially varying properties and evidence:

- geometry, normals, slope, curvature, roughness, steps/gaps and topology;
- friction and traction distributions, compliance, damping, deformability and sinkage;
- load-bearing capacity, wet/ice/loose labels, temperature, wind and illumination;
- dynamic agents, predicted occupancy and social/operational rules;
- covariance, provenance, last-observed time, validity domain, and conservative bounds.

If friction is unknown, the controller estimates conservatively, performs a bounded traction probe when safe, or reduces speed/force. It never substitutes `mu = 0.6` without recording that assumption and its authority level.

### Scenario agnosticism

A scenario is data plus contracts, not an `if` branch. `ScenarioSpec` should define initial belief, assets, embodiments, environment fields, task graph, operating-design-domain predicates, clocks, sensor/noise/fault models, human zones, invariants, success/failure predicates, evidence requirements, and reproducibility hashes. Scenario plugins can add semantics but cannot weaken kernel invariants.

### The honest boundary of generality

No robot can be proven safe or capable in literally every scenario. General intelligence is an expanding competence frontier. The platform can be general in *representation, composition, learning, and graceful handling of novelty* while every deployed release remains qualified for a finite body/configuration/ODD/task envelope. Outside that envelope, the correct intelligence is to notice, gather evidence, ask, degrade, or stop.

## Three viable platform strategies

### Approach A — Pure-Rust robotics stack built from the ground up

Reality OS would implement its own middleware, model parsers, rigid-body algorithms, estimator, collision engine, planners, trajectory optimizer, WBC/MPC solvers, simulators, visualization, and drivers.

**Advantages:** maximum control over types, memory, failure semantics, determinism, and supply chain; one language; potentially a very small trusted base after years of work.

**Costs and risks:** it recreates several decades of robotics infrastructure; numerical corner cases and model-format fidelity are easy to underestimate; hardware and planner ecosystems would lag; achieving credible safety evidence would take longer because every primitive is new. A small team would spend years reaching today’s ecosystem baseline before differentiating on intelligence.

**Use selectively:** write the sovereign contracts and the small trusted/safety subset in Rust, not the whole robotics ecosystem.

### Approach B — Sovereign Rust kernel with replaceable mature backends **(recommended)**

Reality OS owns canonical semantics and authority. ROS 2, `ros2_control`, Pinocchio, MoveIt/Tesseract/Nav2, GTSAM, Crocoddyl, simulators, perception accelerators, and learned policies live behind versioned adapters or isolated services. Every result returns through a typed validator and runtime-assurance boundary.

**Advantages:** preserves the unique safety/evidence thesis; gains mature algorithms and hardware compatibility quickly; permits cross-validation and backend replacement; keeps NVIDIA, ROS, or a specific VLA from becoming the product’s identity; supports mixed Rust/C++/Python without putting dynamic runtimes in the trusted loop.

**Costs and risks:** integration and ABI/version governance are substantial; multiple representations can drift; IPC adds latency; deterministic behavior must be measured across boundaries. These risks are manageable with a single canonical model, conformance tests, process isolation, pinned artifacts, and strict ownership of authoritative state.

### Approach C — Learning-first end-to-end platform with a safety shield

Build primarily around GR00T, OpenVLA/OFT, π-family policies, Gemini Robotics, or future world-action models; add a constraint filter around their actions.

**Advantages:** fastest route to impressive semantic and manipulation demos; benefits from rapidly improving models, datasets, and GPU infrastructure; agentic behavior feels native.

**Costs and risks:** cross-embodiment transfer still needs calibration, normalization, demonstrations, and qualification; action chunks are partly open-loop; latency and uncertainty are not deterministic; a thin safety filter cannot repair bad state estimation, misunderstood geometry, infeasible recovery, or unknown dynamics. Frontier model providers themselves recommend deterministic low-level guardrails.[^8]

**Use as a capability plane, not the spine.** The platform should make learned models hot-swappable and progressively grant authority only when evidence supports it.

## Recommended target architecture

```text
Human / fleet / API / mission
              |
              v
  Agentic deliberation plane (untrusted, non-real-time)
  language grounding, task DAG, skill choice, explanation, memory query
              |
              | typed GoalIR / SkillIR proposal
              v
  Deterministic autonomy plane (bounded, observable)
  capability resolution -> task executive -> TAMP/motion planning
  -> independent feasibility and evidence checks
              |
              | time-bounded TrajectoryReference + fallback
              v
  Estimation and control plane (real-time, no cloud dependency)
  synchronized belief -> prediction -> WBC/MPC -> command arbitration
  -> dynamic safety filter -> execution monitor
              |
              | immutable expiring command envelope
              v
  Independent safety plane / runtime-assurance island
  state machine, watchdog, hard envelopes, reachability/stopping checks,
  safe controller, drive feedback, safe communication, SS1/STO/SBC
              |
              | sole physical actuation path
              v
  Drives / brakes / contactors / motors / physical body

Cross-cutting: canonical body+world+scenario models; evidence graph;
MCAP/event log; clocks; security identity; simulation/HIL; conformance tests.
```

### Plane 1 — Independent safety and actuation

This is the smallest trusted computing base and the only layer permitted to energize hazardous motion. Production authority should live on an independently supervised safety MCU/PLC or an equivalently justified partition, not in an ordinary ROS node. It owns:

- boot/disabled/ready/enabled/degraded/protective-stop/emergency-stop/fault-latched/recovery transitions;
- actuator identity, safety mode, command sequence, expiry, and anti-replay state;
- hard position, velocity, acceleration, jerk, force, momentum, energy, workspace and separation envelopes allocated by the hazard analysis;
- end-to-end watchdogs, actuator feedback, bus health, brake/contactor state, and safe-drive functions;
- a simple verified fallback or minimal-risk controller;
- a bounded RT event ring and safety-case evidence hooks.

The main computer may request a transition or action; it cannot force one. Reset never resumes motion. Network, GPU, ROS, agent, disk, visualization, and fleet failures cannot disable local protective functions.

### Plane 2 — Deterministic estimation and control

This plane maintains the best current body/environment belief and turns validated references into physically coherent whole-body commands. It should use multiple rates rather than forcing every function into “1 kHz”:

| Loop | Indicative range | Rule |
|---|---:|---|
| Drive current/commutation | 5–40 kHz, normally vendor drive | Treat as a device capability, not a platform constant |
| Safety/watchdog/gate | 500 Hz–2 kHz | Derived from hazard FTTI and bus/drive timing |
| WBC / inverse dynamics | 200 Hz–2 kHz | Body and solver dependent; bounded iterations and fallback |
| State/contact estimation | 100 Hz–2 kHz | Sensor-dependent, time-aligned, covariance-bearing |
| Local MPC / trajectory tracking | 50–500 Hz | Deadline and horizon capability-negotiated |
| Local perception | 15–120 Hz | May use GPU; stale results rejected |
| Global planning / task executive | 0.5–30 Hz | Asynchronous and cancellable |
| Semantic agent | 0.05–5 Hz | Never in a deadline-critical dependency chain |

These are ranges, not guarantees. Each adapter declares supported rates, jitter, latency, control modes, queue bounds, and timeout behavior; commissioning chooses values. Acceptance is based on measured end-to-end response and missed-deadline behavior on production hardware.

### Plane 3 — Probabilistic perception and learning

Vision, learned representations, policies, and world models run here. Recommended capability ladder:

1. calibrated camera/IMU/encoder/force acquisition and hardware timestamping;
2. image quality, occlusion, blur, exposure, depth validity, and OOD checks;
3. depth/stereo/lidar geometry, segmentation, detection, tracking and object pose;
4. dense local reconstruction plus semantic scene graph and dynamic-agent prediction;
5. learned affordances, grasp proposals, visual subgoals and short action references;
6. learned world/action models for candidate ranking, anomaly prediction and offline data generation.

Every output carries covariance or calibrated confidence, source sensor and transform epochs, model/build hash, validity region, capture-to-result latency, and evidence links. A learned world model is a prediction source, not the ground truth or a safety proof.

### Plane 4 — Agentic deliberation

The agent’s role is to interpret intent, query state, compose qualified skills, request observations, explain failures, and choose recovery branches. It must not invent new physical verbs or tool authority at runtime.

`SkillIR` should include:

- exact version and signer;
- typed parameters, frames, units, resources, and required capabilities;
- preconditions, invariants, effects, success and failure predicates;
- observation/evidence requirements and maximum staleness;
- admissible planners/controllers and authority ceiling;
- timeout, retry budget, recovery graph, cancellation semantics, and safe terminal set;
- qualification matrix by embodiment, tool, environment class, scenario family, and software/model version.

BehaviorTree.CPP is a strong reusable task-execution engine because it supports runtime-loaded trees, asynchronous actions, logging, replay, and ROS integration; MoveIt Task Constructor provides staged manipulation composition.[^9] Reality OS should own the typed skill contract and authority checks, while either engine can execute an adapter representation.

## Canonical models the kernel must own

### `EmbodimentGraph`

Minimum entities and invariants:

- immutable body/configuration identity and source artifacts;
- tree or closed-chain links/joints with typed frames and transforms;
- mass, center of mass, inertia tensor, geometry, material/contact candidates;
- actuators, transmissions, gear ratios, current/torque/speed/thermal/power curves;
- sensors, rates, delays, noise, time domains, calibration and transform lineage;
- tools/end effectors, grasp frames, payload/cable/fluid constraints;
- position/velocity/acceleration/jerk/effort/current/energy limits with evidence tier;
- support regions, foot/wheel/contact geometry and controllability capabilities;
- parameter covariance, validity temperature/load ranges, and last identification time.

The importer preserves the original URDF/SDF/MJCF/USD/vendor files and emits a canonical hash plus diagnostics. It must reject physically impossible inertia, invalid transforms, duplicate frames, unit ambiguity, unsupported transmissions, and inconsistent DoF. Pinocchio is the preferred compact math backend; Drake is valuable as an offline/reference oracle. Neither owns the canonical source of truth.[^10]

### `WorldBelief`

The world is a time-varying probabilistic graph with multiple synchronized views:

- transform/frame graph;
- metric geometry (occupancy, voxels, mesh, TSDF/ESDF);
- objects, parts, articulation, support/containment relations and affordances;
- surfaces/terrain fields and traversability by locomotion mode;
- dynamic agents, velocity/intent hypotheses and predicted occupancy tubes;
- maps/topology/routes and operational zones;
- uncertainty, provenance, freshness, conflicts and invalidation dependencies.

There must be exactly one authority for each state class. If MoveIt or Tesseract is selected as the planning-scene backend, the other receives a read-only mirror; two mutable scenes are guaranteed drift.

### `BeliefState`

Represent the estimator’s posterior, not just a point estimate:

\[
b_t = p(x_t,\theta, e_t \mid z_{1:t}, u_{1:t-1}),
\]

where `x` contains joint/floating-base/actuator/thermal state, `theta` contains body and contact parameters, and `e` contains environment properties. The state exposes innovations, residuals, covariance/credible bounds, sensor health, observability warnings, and competing hypotheses. Safety uses conservative projections rather than trusting a semantic confidence score.

### `ScenarioSpec` and `OperatingEnvelope`

`ScenarioSpec` is reproducible input. `OperatingEnvelope` is the proven/qualified subset currently allowed. The latter combines body build, payload/tool, environment bounds, task/skill set, human proximity, speed/force limits, sensor health, software/model versions, and required safeguards. Novelty is measured as distance from this envelope and causes an explicit authority transition.

### `GoalIR`, `SkillIR`, `TrajectoryReference`, and `CommandEnvelope`

Each stage reduces ambiguity and authority:

```text
free-form intent
  -> GoalIR              semantic desired outcome; no motor authority
  -> SkillIR DAG         qualified behaviors and recovery edges
  -> TrajectoryReference frames, time, contacts, tolerances, fallback
  -> ControlProposal     bounded short horizon, solver evidence
  -> CommandEnvelope     exact actuators/mode/sequence/expiry/policy hash
  -> DriveFrame          bus-specific, emitted only by safety authority
```

No downward stage may widen a parent’s feasible set. Every transformation records parent hash, code/model/config hash, timing, and the evidence used.

## Whole-body control and high-speed computation

### Control stack

Use a hierarchy rather than one universal solver:

1. **Device controller:** current/torque/velocity/position loop in the drive.
2. **State/contact estimator:** base, joints, actuator health, contacts and terrain parameters.
3. **Trajectory/reference manager:** interpolates a validated trajectory and maintains a safe fallback.
4. **Whole-body QP:** tracks tasks while enforcing dynamics, contact/friction, limits, collision and stability.
5. **MPC/OCP:** predicts contacts and nonlinear behavior over a horizon; returns a trajectory plus feedback policy.
6. **Runtime-assurance filter:** checks recoverability and can switch to the verified backup.
7. **Independent safety controller:** enforces final hazard-derived limits and power-state transitions.

Control Barrier Function QPs can make sets forward invariant when model, state, disturbance, feasibility and timing assumptions hold; that conditional guarantee is useful but does not replace safety-rated stops or lifecycle evidence.[^11]

### Performance topology

- Keep the RT control/safety processes isolated from agent, logging, UI, filesystem, discovery, and GPU-failure domains.
- Preallocate and prefault memory; use fixed-capacity messages/queues; avoid unbounded locks and priority inversion; pin critical threads/IRQs; lock memory; control CPU frequency and thermal behavior.
- Use a monotonic clock for deadlines/freshness and synchronized PTP/TAI only for cross-machine correlation. Simulation/ROS time may pause or jump and must never drive a physical watchdog.[^12]
- Use shared memory/loaned messages for high-volume local perception where beneficial. `iceoryx2` is a promising Rust-native zero-copy data plane, but its open-source lock-free design and safety-certification roadmap are not themselves certification.[^13]
- Use ROS 2/DDS for ecosystem interoperability and lifecycle, not inside the irreducible drive-safety path. Cyclone DDS is a practical default local/LAN RMW; native Zenoh is useful for WAN/fleet/queryable data.
- Wrap C++ numerical libraries through a narrow C ABI or isolated process. Pin versions, warm caches/solvers, bound iterations, apply cancellation deadlines, validate all outputs, and keep a previous feasible solution plus fallback.
- Measure sensor-to-actuator latency, age of information, jitter, deadline miss rate, solver convergence/feasibility, CPU/GPU/memory pressure, bus latency, and physical stopping—not just loop frequency.

## Self-correction and failure tolerance

Self-correction must be externalized into observable control logic. The model saying “I corrected myself” is not evidence.

### Failure-detection channels

- model prediction versus sensed motion/contact;
- controller tracking residuals and saturation;
- estimator innovation, covariance growth and observability;
- vision quality/OOD/disagreement and scene contradictions;
- planner/controller feasibility and terminal-containment checks;
- actuator current/temperature/encoder/brake/drive-state diagnostics;
- time, queue, watchdog, network, process and resource health;
- independent task success predicates.

### Recovery ladder

1. continue within tolerance;
2. reduce speed/force/horizon and increase separation;
3. pause/hold while maintaining stability;
4. execute a bounded information-gathering action;
5. re-estimate/relocalize/recalibrate a permitted parameter;
6. replan with an alternate grasp/path/contact/skill;
7. switch backend/controller/sensor where redundancy is qualified;
8. retreat to a known safe set or minimal-risk condition;
9. protective/emergency stop and request human intervention.

Each level has a bounded retry budget and hysteresis to prevent oscillation. Online adaptation may update a tightly bounded estimator parameter when its validity checks pass. It must not change hard limits, safety logic, trusted keys, production policy weights, or the safety controller without a controlled release process.

Long-term memory is split from immediate evidence. A lossless, short-horizon event buffer supports control and reconstruction; a semantic episodic store links every summary to original observations, body/calibration/model versions, confidence and expiry. Contradictory memory is retained and resolved, not silently overwritten. Remembered prose never becomes motion directly.

## Competitors, adjacent platforms, and what to reuse

No single competitor covers the full target. The practical advantage comes from composing their strongest layers behind Reality OS contracts.

| System | Strongest reusable capability | What it does not solve for Reality OS | Decision |
|---|---|---|---|
| **ROS 2 Lyrical/Jazzy** | Transport graph, QoS, actions/services, lifecycle, time, bags, broad device ecosystem | Canonical body/world semantics, deterministic safety, physical command authority | Adopt at adapter boundary. Mainline current long-lived Lyrical; keep Jazzy compatibility for NVIDIA tooling.[^14] |
| **`rclrs`** | Native Rust ROS publishers/subscribers/services/actions, QoS, loaned messages | API stability and hard-real-time guarantees | Use behind an internal adapter; pin toolchain/generated interfaces and retain a C++ sidecar option.[^15] |
| **`ros2_control`** | Standard hardware interfaces, controller manager, common controllers, simulator integrations | Safety-rated final gate or universal dynamics/control | Reuse controller/hardware ecosystem; place Reality OS arbitration and independent safe-write after controller output.[^16] |
| **MoveIt 2 / MoveIt Pro** | Collision-aware manipulation, plugin planners, Servo, planning scene, task constructor; Pro adds hardened commercial workflows | Whole-body/contact control and safety certification | Default manipulation service; validate trajectories independently. Consider Pro only when support/workflow value justifies proprietary dependency.[^17] |
| **Tesseract** | ROS-agnostic environment, continuous swept collision, TrajOpt/Descartes/process planning, task-composer DAG | Universal runtime or certified controller | Optional industrial planning backend; choose one authoritative planning scene.[^18] |
| **Nav2** | Mature mobile planning/control/route/BT ecosystem and a useful collision monitor | 3D whole-body world model; safety-rated collision protection | Adopt for wheeled/mobile 2D navigation. Its collision monitor is explicitly an additional non-certified layer.[^19] |
| **Pinocchio** | Fast rigid-body kinematics/dynamics, centroidal quantities and analytical derivatives | Planning, state estimation, safety and bounded WCET | Preferred multibody backend behind a narrow ABI; cross-check the small safety subset.[^20] |
| **Crocoddyl / OCS2** | Contact-aware optimal control and MPC architectures | Guaranteed solver completion or safety | Prefer Crocoddyl for optional WBC/contact OCP. Keep OCS2 experimental because its main release/ROS 2 posture is less current.[^21] |
| **Drake** | Rich multibody, optimization, IK, planning and formal-analysis/reference tools | Stable embedded ABI or hard-RT deployment | Use as a pinned offline/reference oracle and differential validator.[^22] |
| **MuJoCo** | Fast preallocated articulated/contact dynamics; clean C API; MJCF/URDF | Photorealistic sensor validation and product safety | Default fast dynamics/control/RL lane and first validation fixture.[^23] |
| **Gazebo** | Broad ROS/system simulation, SDF worlds, sensors, plugin architecture | Determinism by default | Primary ROS integration/fault-injection lane; native plugins for controlled stepping.[^24] |
| **Isaac Sim / Isaac Lab** | GPU-parallel environments, RTX sensors, synthetic data, domain randomization, RL/IL | Runtime portability, exact reproducibility across stacks, safety | Optional accelerated training/perception lane; export versioned policy artifacts through a neutral ABI.[^25] |
| **Isaac ROS / NITROS / nvblox / cuMotion** | GPU-resident perception and mapping, accelerated collision/motion | Hardware/vendor independence and safety certification | Isolate as an optional NVIDIA process with one standard boundary and a portable fallback.[^26] |
| **GTSAM** | Factor-graph estimation, smoothing, SLAM/SfM, IMU preintegration | Sensor front ends and canonical robot/world contracts | Default high-level estimation backend after a simpler deterministic EKF lane.[^2] |
| **Open3D / FoundationPose** | Portable 3D processing and strong object-pose baseline | End-to-end perception confidence/safety | Use as replaceable perception plugins with calibration/OOD validation.[^27] |
| **BehaviorTree.CPP / MoveIt Task Constructor** | Observable asynchronous task orchestration and staged manipulation | Physical authority and semantic verification | Reuse execution machinery; keep `SkillIR` and transition guards sovereign.[^9] |
| **Viam** | Uniform component APIs, registry, reconfiguration, deployment and fleet monitoring | Deep dynamics, WBC and safety | Borrow its developer/operations ergonomics and component discovery, not its abstraction as the control core.[^28] |
| **Intrinsic / Flowstate** | Digital twins, skills, perception and workcell orchestration | Open, robot-general safety/control substrate | Competitive product benchmark for workflow and skill packaging; closed commercial dependency.[^29] |
| **GR00T / Gemini Robotics / OpenVLA-OFT / π0.5 / Octo** | Semantic transfer, visual reasoning, action chunks, cross-dataset learning | Deterministic timing, arbitrary-body transfer, calibrated physical constraints, safety | Build a portfolio adapter. Learned output stops at goals, skills, subgoals, or bounded short reference chunks.[^30] |
| **LeRobot / Open X-Embodiment** | Dataset schemas, synchronized episodes, policy baselines, cross-embodiment mixtures | Calibration, action semantics, commissioning, and safety | Reuse for data/policy interchange while retaining Reality OS embodiment and evidence manifests.[^6] |
| **MCAP / Rerun / Foxglove / `ros2_tracing`** | Durable neutral pub/sub recording, multimodal visualization, operator UI, execution traces | Deterministic replay by themselves | MCAP is canonical raw archive; Rerun for engineering; Foxglove for operations; attach exact configuration, clock mapping and execution order.[^31] |

### Current learned-system reality

The best disclosed VLA pattern combines a large vision-language backbone with a continuous-action expert, heterogeneous pretraining, chunked actions, and embodiment-specific post-training. OpenVLA/OFT, π0.5, GR00T, Octo and Gemini Robotics demonstrate meaningful semantic and dexterous transfer. None removes calibration or commissioning. Action chunks improve throughput but are partly open-loop between policy updates. Google’s 2026 Gemini Robotics safety report documents material human-distance classification error and explicitly excludes certified hardware, redundancy, and real-time guarantees; it recommends deterministic low-level safeguards.[^8]

This leads to a robust policy hierarchy:

- semantic model: propose goals, task graph, observation requests, explanations;
- VLA/specialist policy: propose bounded short-horizon Cartesian or joint references;
- deterministic planning/control: enforce kinematics, collision, dynamics, timing and terminal containment;
- runtime assurance: preserve a feasible fallback and switch before leaving the recoverable set;
- independent safety: enforce hazard-derived limits and control motive power.

The platform should benchmark a portfolio. Large VLAs must beat small ACT/Diffusion Policy specialists on success, recovery, compute, latency and data cost before receiving authority. Model novelty never widens the safety envelope.

## Implementation program and acceptance gates

The program is intentionally staged. “Implement all gaps” cannot safely be one undifferentiated coding pass: body semantics precede dynamics; dynamics and clocks precede WBC; estimation and world evidence precede autonomous recovery; the independent safety topology precedes hazardous hardware. Each phase leaves a usable, testable platform increment.

Durations are rough calendar ranges for a focused multi-disciplinary team; they are not commitments. Several research/backend tracks can run in parallel, but safety gates remain sequential.

### Phase 0 — Contain current P0 defects (2–4 weeks)

**Build**

- Seal certified-write capability and remove public plant/port mutation paths.
- Make unknown intent refuse; forbid nonzero plan synthesis from limits.
- Add exact dimensionality, finite/positive bound, timestamp/time-domain and sequence validation everywhere.
- Replace magnitude-only narrowing with typed per-mode containment; make commands immutable after signing.
- Expand the canonical signed payload and add parent/derived-envelope linkage.
- Add `UnknownOutcome` and an idempotent prepare/execute/ack protocol in simulation.
- Replace see-before-act booleans with a minimal observation evidence handle.
- Correct `DispatchResult.realized` and propagate all audit-write errors according to mode.

**Verify**

- Compile-fail/API tests prove ordinary crates cannot construct a write token or reach a production port.
- Property/fuzz corpus covers arbitrary JSON, NaN/Inf, dimensionality, clock rollback/future stamps, mutation, replay, sequence and signature attacks.
- Kill/power-loss simulation covers every point around ledger prepare/write/ack and never duplicates ambiguous motion.
- Existing 57 tests remain green; add explicit regression tests for every P0 finding.

**Exit gate:** software-only actuation authority is structurally singular under the declared process threat model. This is still not permission to energize hazardous hardware.

### Phase 1 — Canonical contracts, units, time and plugin ABI (4–8 weeks)

**Build**

- New core crates/modules for units/frames, monotonic and synchronized time, model identity, evidence references, health/fault taxonomy and fixed-capacity RT messages.
- `EmbodimentGraph`, `CapabilityManifest`, `WorldBelief`, `ScenarioSpec`, `OperatingEnvelope`, `GoalIR`, `SkillIR`, `TrajectoryReference`, and immutable `CommandEnvelope` schemas.
- Versioned adapter protocol with handshake, capability negotiation, health, cancellation, deadline and deterministic error semantics.
- URDF/SRDF + MJCF import first; SDF/USD later. Preserve originals and emit unsupported-feature diagnostics.
- Manifest signing, schema migration, semantic version compatibility and golden fixtures for H1 and the 6-DoF arm.

**Verify**

- Schema round-trip/golden tests and compatibility matrix.
- Physical model validation: frames, units, inertia definiteness, limit consistency, transform closure and DoF exactness.
- Same capability queries and task schemas work without robot-name branches on both fixtures.

**Exit gate:** no control/planning API depends on H1, arm, Earth, indoor floor, MuJoCo, or ROS-specific types.

### Phase 2 — Runtime, ROS 2 and simulation substrate (6–10 weeks)

**Build**

- ROS 2 adapter using pinned `rclrs` where practical and a C++ sidecar where ecosystem/ABI maturity requires it.
- `ros2_control` hardware/controller adapter and lifecycle/error mapping, initially simulation-only.
- Direct MuJoCo backend for fast deterministic stepping; Gazebo lane for ROS/system integration.
- Explicit executor topology, bounded channels, monotonic scheduler, cancellation/deadline propagation and overload policy.
- MCAP raw evidence writer, bounded RT telemetry ring, Rerun visualization adapter, ROS tracing hooks and deterministic replay manifest.

**Verify**

- Cross-language and adapter conformance tests.
- Sensor-to-command age and latency measured under CPU/memory/I/O/network stress.
- Replays reproduce the same discrete decisions and explain tolerated numerical divergence.
- Simulator differential tests compare kinematics, gravity, limits, contacts and stopping across backends.

**Exit gate:** both reference bodies execute bounded simulated trajectories through the exact same canonical and authority paths, with reproducible evidence.

### Phase 3 — Body intelligence and state estimation (8–14 weeks)

**Build**

- Pinocchio backend for FK/Jacobians/RNEA/CRBA/ABA/centroidal dynamics and derivatives.
- Deterministic EKF/error-state baseline for joint/base/IMU state; GTSAM smoothing/factor-graph plugin for richer fusion and offline refinement.
- Contact, slip, actuator/thermal and sensor-health estimation.
- Calibration service for time offsets, cameras, IMU, joint zeros, tool frames and force sensors.
- Bounded online system identification for payload, friction and actuator parameters with evidence tier and rollback.

**Verify**

- Analytical-versus-finite-difference derivatives; Pinocchio-versus-MuJoCo/Drake oracle comparisons.
- Estimator consistency (NEES/NIS where assumptions permit), innovation/OOD tests, observability detection, dropout/bias/time-offset fault injection.
- H1 floating-base/contact and arm fixed-base/tool fixtures meet declared error/latency budgets in simulation and recorded datasets.

**Exit gate:** every controller consumes a time-aligned belief with uncertainty and health, never raw unqualified samples.

### Phase 4 — Vision and spatial world intelligence (8–16 weeks)

**Build**

- Calibrated RGB/depth/stereo/lidar pipelines with quality and latency diagnostics.
- Object detection/segmentation/tracking/pose plugins; geometric collision representation; TSDF/ESDF/occupancy; semantic scene graph.
- Terrain/surface belief fields, traversability by locomotion capability, dynamic-agent tracking and predicted occupancy.
- Active-perception skills such as scan, viewpoint change, touch/force probe and traction probe.
- Observation evidence compiler binds source pixels/depth, transforms, calibration, model, covariance, OOD and scene epoch.

**Verify**

- Calibration and transform perturbation tests; occlusion/blur/lighting/material/weather/domain-shift suites.
- Object/scene metrics plus downstream collision/grasp/navigation error—not only mAP.
- Stale, contradictory or OOD perception reduces authority; spoofed boolean/stamp cannot satisfy the evidence gate.

**Exit gate:** see-before-act is cryptographically and geometrically traceable from raw observation to decision.

### Phase 5 — Planning, WBC, MPC and runtime assurance (12–24 weeks)

**Build**

- MoveIt 2 default manipulation service; Nav2 mobile adapter; optional Tesseract industrial lane.
- Collision-aware IK, path planning, time parameterization and trajectory validation.
- Hierarchical whole-body QP with inverse dynamics, contacts, friction, balance, joint/actuator/thermal/power and collision constraints.
- Crocoddyl contact/MPC backend with warm start, bounded iteration/deadline and prior-feasible fallback.
- CBF/reachability/stopping filters under explicit assumptions.
- Runtime Assurance switch with a verified hold/brake/retreat/minimal-risk controller and recoverable-set monitor.

**Verify**

- Constraint residual and infeasibility suites; singularity/contact-switch/slip/payload/saturation tests.
- Worst-case solver deadlines under contention; missed deadline always selects a tested fallback.
- Perturbed model/state/disturbance sweeps establish the declared robust envelope.
- Whole-body tasks on H1 and collision-aware force/position tasks on the arm share the same task/reference/control contracts.

**Exit gate:** the learned/agentic plane can be removed or frozen and the robot still tracks, rejects, degrades and reaches a safe condition deterministically.

### Phase 6 — Agentic skills, memory and learned-policy portfolio (8–20 weeks)

**Build**

- Signed skill registry and capability-grounded task DAG compiler.
- Behavior-tree/state-machine executor with independent precondition/effect/completion verification.
- Bounded recovery, active perception, clarification and human-handoff paths.
- Evidence-linked short-term and episodic memory with expiry/conflict handling.
- Adapters for an open VLA baseline (OpenVLA-OFT or GR00T depending hardware), a compact Octo/ACT/Diffusion specialist, and a remote semantic model.
- LeRobot/Open-X-style dataset export, failure/recovery demonstrations, policy/model registry, shadow/canary modes and rollback.

**Verify**

- Prompt injection and tool-authority tests; free text never reaches motion.
- Long-horizon task metrics separate planning, perception, execution, recovery and human-intervention failures.
- Policy outputs are adversarially perturbed; validator/fallback behavior remains invariant.
- A new policy/agent can be swapped without changing the body, world, control, or safety contracts.

**Exit gate:** agentic intelligence expands task competence without becoming a single point of physical authority.

### Phase 7 — Independent safety spine and hardware qualification (parallel design; gated integration, 6–18+ months)

**Build**

- Hazard analysis, safety functions, safe states, FTTI and PLr/SIL allocation for a specific initial product/ODD.
- Separate safety MCU/PLC, independent watchdog, safe I/O or safety fieldbus, drive feedback, SS1/STO/SBC/brake/contactor control.
- Secure/measured boot, hardware-backed identity, signed and anti-rollback configuration/update bundles.
- Vendor driver adapters (for example EtherCAT/CAN/CiA 402 where applicable), but ordinary transport is never mistaken for a safety protocol.
- HIL rigs with real controllers/drives, load/brake emulation and safe fault injection.

**Verify**

- Main-compute freeze/power loss, bus loss/corruption, sensor drift/freeze, clock faults, resource exhaustion, brownout, drive/brake/contact faults and restart-at-every-boundary.
- Measure detection time, transition time, stopping distance/time, final element state and diagnostic coverage against every safety requirement.
- Trace hazards → safety requirements → architecture/code/config → test procedure/result → release baseline.
- Independent assessor review and product/domain-specific standards work. ISO 10218-1/-2 covers industrial robot and integration scopes; ISO 3691-4 covers driverless industrial trucks; ISO 13849 or IEC 62061 may structure safety-related control evidence, but none certifies an agnostic software platform in the abstract.[^32]

**Exit gate:** only the declared hardware/application/ODD release—not “Reality OS everywhere”—may make a production safety claim.

### Phase 8 — Generality expansion and continuous assurance (ongoing)

Add embodiments in deliberately different families: wheeled AMR, quadruped, bimanual humanoid, aerial robot, deformable or articulated tool. Add indoor/outdoor, slopes/stairs/rubble, ice/loose/deformable terrain, dynamic human environments, adverse sensing and degraded power. Promote a capability only when it passes the same conformance, differential simulation, replay, HIL and field gates.

## Quantitative capability and release metrics

“Incredibly capable” needs measurable contracts. Maintain scorecards by body × environment class × scenario family × fault × software/model/config version.

| Dimension | Representative metrics |
|---|---|
| Safety | unsafe command escape count; safety-function coverage; detection/transition/stopping latency; minimum margin; false/missed protective stops; proof-test coverage |
| Real time | p50/p99/p99.99/max latency; age of information; deadline misses; scheduling and page-fault counts; queue high-water marks; thermal/load sensitivity |
| Estimation | pose/velocity/contact error; NEES/NIS; covariance calibration; relocalization time; drift; fault isolation precision/recall |
| Perception/world | geometry error; pose/tracking error; collision false negative; OOD calibration; scene freshness; downstream task success |
| Control | tracking error; constraint violation; solver feasibility/time; energy; slip/fall/drop rate; recovery-set retention |
| Planning | success, time, path/trajectory quality, clearance, replan/cancel latency, fallback availability |
| Agent/skills | grounded-plan validity; unsupported skill rejection; long-horizon completion; recoveries; human interventions; hallucinated capability attempts |
| Generality | zero/few-shot performance separated from post-training; commissioning effort; adapter code size; conformance pass rate across bodies/backends |
| Debug | time-to-root-cause; percent incidents deterministically reproducible; evidence completeness; dropped telemetry; causal-chain coverage |

Use RobotPerf and ROS tracing methods as inputs, but build system-level benchmarks around the canonical contracts. REP-2014 is informative rather than an accepted standard; its useful principle is quantitative grey-box measurement of realistic graphs.[^33]

## Testing and assurance pyramid

1. **Types and construction invariants:** impossible units, frames, model IDs, time domains and authority states should be unrepresentable.
2. **Unit/property/fuzz:** formulas, serialization, parser/importer, monotonic state transitions, containment, cryptography and hostile values.
3. **Model conformance:** body/environment/scenario adapters, golden files and source round trips.
4. **Deterministic SIL:** seeded scenarios, accelerated time, exact event decisions, fault injection and state-machine coverage.
5. **Differential SIL:** MuJoCo/Gazebo/Drake or independent algorithms compare invariant quantities within declared tolerance.
6. **Performance-in-the-loop:** production compute under CPU/GPU/network/storage/thermal contention with tracing enabled and disabled.
7. **HIL:** real buses, controllers, drives, clocks, watchdogs and final-element feedback with bounded energy.
8. **Restrained physical tests:** conservative envelopes and independent emergency protection.
9. **Limited ODD pilots:** shadow/canary updates, telemetry, anomaly review and rapid rollback.
10. **Assurance case:** hazards, assumptions, evidence and residual risk tied to one release and application.

UL 4600’s goal-based safety-case approach is relevant to autonomous behavior because it expects combined analysis, simulation, closed-course/field testing and update evidence rather than simulation alone.[^34]

## Proposed repository evolution

This is a logical ownership map, not yet the approved file-by-file plan:

```text
crates/
  units/              dimensioned quantities, finite/bounded constructors
  time/               monotonic/synchronized/sim clocks and mappings
  model/              embodiment graph, capabilities, import diagnostics
  evidence/           observation/model/config provenance and hashes
  world/              belief graph and geometry/semantic interfaces
  estimation/         estimator traits, health and belief contracts
  trajectory/         frame/time/contact-aware references and containment
  control/            controller/WBC/MPC traits and solver evidence
  skills/             GoalIR, SkillIR, registry and execution contracts
  runtime/            scheduling, leases, arbitration, fault/state machine
  safety-protocol/    immutable command envelopes and safety-island protocol
  adapters/
    ros2/ ros2_control/ mujoco/ gazebo/ pinocchio/ gtsam/ moveit/ nav2/
  observability/      RT ring, MCAP, replay manifests and trace correlation
  assurance/          scenario/fault specs, evidence traceability and gates

hardware/             safety MCU/PLC protocol, HIL fixtures (separate lifecycle)
models/               source artifacts + signed normalized manifests
scenarios/            reusable environment/task/fault specifications
qualification/        release/ODD matrices and generated evidence indexes
```

Keep the current crates where they already express a clean concept, but progressively move raw vectors, floats, public mutable fields and stringly typed violations into these canonical types. Avoid a large rewrite: introduce adapters around existing behavior, migrate one vertical path at a time, and keep tests executable throughout.

## Key risks and explicit non-goals

- **False universality:** solve with explicit ODD/capability/evidence contracts and refusal outside them.
- **Integration sprawl:** one authoritative model/state per concern, a small supported backend matrix, version pinning and adapter conformance.
- **Safety theater:** physical topology, hazard-derived requirements and independent validation matter more than naming a Rust function “governor.”
- **Realtime theater:** loop frequency, average latency and “zero-copy” do not establish WCET or correct overload behavior.
- **Simulation overconfidence:** every simulator shares modeling blind spots; require cross-backend, HIL and restrained metal evidence.
- **Learned-model monoculture:** maintain specialist and deterministic baselines; make vendor/model adapters removable.
- **Online-learning hazard:** production may learn observations and bounded estimated parameters; it may not self-modify safety authority.
- **Scope explosion:** do not implement every planner, solver, simulator, middleware and viewer. Own contracts and differentiating authority; reuse proven engines.
- **Certification claim:** the generic platform can be developed for assurance and produce evidence, but certification/conformity attaches to an exact product, configuration, application, ODD and lifecycle.

## Decision requested

Approve **Approach B: sovereign Rust kernel with replaceable mature backends**, with these binding principles:

1. H1 and the 6-DoF arm are first conformance fixtures only.
2. ROS 2 and MuJoCo are first adapters only.
3. The independent safety path and deterministic control/evidence spine outrank any learned model.
4. Unknown or unqualified conditions reduce authority; no hidden defaults.
5. The implementation proceeds through gated vertical slices, beginning with Phase 0 containment and Phase 1 contracts.

After approval, the next artifact should be the formal design specification required by the brainstorming workflow, followed by a task-by-task implementation plan with exact files, tests, checkpoints, dependency pins and rollback points. Only then should a persistent GPT-5.6 Terra implementation task be created.

## Sources

[^1]: NASA, [Runtime Assurance Architecture](https://ntrs.nasa.gov/api/citations/20140016536/downloads/20140016536.pdf); NASA, [Formal Verification of a Runtime Assurance Architecture](https://ntrs.nasa.gov/citations/20230017350).
[^2]: Georgia Tech Borg Lab, [GTSAM documentation](https://gtsam.org/docs/) and [IMU preintegration notes](https://borglab.github.io/gtsam/imufactor/).
[^3]: NVIDIA Isaac ROS, [nvblox documentation](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/index.html); Open3D, [official documentation](https://www.open3d.org/docs/latest/).
[^4]: Pinocchio, [features and documentation](https://stack-of-tasks.github.io/pinocchio/index.html); Crocoddyl, [official repository](https://github.com/loco-3d/crocoddyl); IHMC, [Controller Core documentation](https://ihmcroboticsdocs.github.io/ihmc-open-robotics-software/docs/01-controllercore.html); Stack of Tasks, [TSID repository](https://github.com/stack-of-tasks/TSID).
[^5]: ROS 2, [Real-time programming proposal](https://design.ros2.org/articles/realtime_proposal.html) and [Executors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html).
[^6]: Hugging Face, [LeRobot documentation](https://huggingface.co/docs/lerobot/index); Google DeepMind et al., [Open X-Embodiment paper](https://arxiv.org/abs/2310.08864) and [repository](https://github.com/google-deepmind/open_x_embodiment).
[^7]: IEC, [IEC 62443-4-1](https://webstore.iec.ch/en/publication/33615) and [IEC 62443-3-3](https://webstore.iec.ch/en/publication/7033); NIST, [SP 800-193 Platform Firmware Resiliency Guidelines](https://csrc.nist.gov/pubs/sp/800/193/final); Uptane, [Standard 2.0](https://uptane.org/docs/2.0.0/standard/uptane-standard).
[^8]: Google DeepMind, [Gemini Robotics 2 Safety Report](https://storage.googleapis.com/deepmind-media/gemini-robotics/Gemini-Robotics-2-Safety.pdf), [Gemini Robotics 2 announcement](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/), and [ER 2 model card](https://deepmind.google/models/model-cards/gemini-robotics-er-2/).
[^9]: BehaviorTree.CPP, [official documentation](https://www.behaviortree.dev/); MoveIt, [Task Constructor concepts](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html).
[^10]: Pinocchio, [official project](https://stack-of-tasks.github.io/pinocchio/index.html); Drake, [official project](https://drake.mit.edu/) and [Inverse Kinematics API](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_inverse_kinematics.html).
[^11]: Ames et al., [Control Barrier Function Based Quadratic Programs](https://arxiv.org/abs/1609.06408).
[^12]: The Open Group, [POSIX monotonic clock rationale](https://pubs.opengroup.org/onlinepubs/9699919799/xrat/V4_xsh_chap02.html); ROS 2, [Clock and Time design](https://design.ros2.org/articles/clock_and_time.html).
[^13]: Eclipse iceoryx2, [Zero-copy data plane overview](https://ekxide.github.io/iceoryx2-book/main/overview/what-is-iceoryx2.html) and [shared-memory design](https://ekxide.github.io/iceoryx2-book/main/fundamentals/shared-memory.html).
[^14]: ROS 2, [Lyrical Luth release information](https://docs.ros.org/en/kilted/Releases/Release-Lyrical-Luth.html) and [distribution support table](https://docs.ros.org/en/humble/Releases.html).
[^15]: ROS 2 Rust, [`rclrs` repository and limitations](https://github.com/ros2-rust/ros2_rust) and [0.7 changelog](https://github.com/ros2-rust/ros2_rust/blob/main/rclrs/CHANGELOG.md).
[^16]: `ros2_control`, [documentation](https://control.ros.org/master/doc/ros2_control/doc/index.html), [different update rates](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/different_update_rates_userdoc.html), and [asynchronous controllers](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/running_controllers_asynchronously.html).
[^17]: MoveIt, [concepts](https://moveit.ai/documentation/concepts/), [Hybrid Planning](https://moveit.picknik.ai/main/doc/examples/hybrid_planning/hybrid_planning_tutorial.html), and [MoveIt Pro technical specifications](https://docs.picknik.ai/technical_specifications/).
[^18]: Tesseract Robotics, [official repository](https://github.com/tesseract-robotics/tesseract) and [collision documentation](https://tesseract-robotics.github.io/tesseract/collision.html).
[^19]: Nav2, [Behavior Trees](https://docs.nav2.org/rolling/getting_started/navigation_concepts/behavior_trees/) and [Collision Monitor safety limitations](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/).
[^20]: Pinocchio, [dynamic algorithms](https://docs.ros.org/en/ros2_packages/rolling/api/pinocchio/doc/a-features/g-dynamic.html).
[^21]: Crocoddyl, [overview](https://docs.ros.org/en/rolling/p/crocoddyl/doc/Overview.html); OCS2, [repository](https://github.com/leggedrobotics/ocs2) and [getting started/performance caveat](https://leggedrobotics.github.io/ocs2/getting-started.html).
[^22]: Drake, [systems framework](https://drake.mit.edu/doxygen_cxx/group__systems.html), [planning APIs](https://drake.mit.edu/doxygen_cxx/group__planning.html), and [release policy](https://drake.mit.edu/stable.html).
[^23]: MuJoCo, [official overview](https://mujoco.readthedocs.io/en/stable/overview.html).
[^24]: Gazebo, [architecture](https://gazebosim.org/docs/harmonic/architecture/) and [ROS 2 integration](https://gazebosim.org/docs/harmonic/ros2_integration/).
[^25]: NVIDIA, [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/v2.2.1/index.html) and [reference architecture](https://isaac-sim.github.io/IsaacLab/main/source/refs/reference_architecture/index.html).
[^26]: NVIDIA Isaac ROS, [release notes](https://nvidia-isaac-ros.github.io/releases/index.html), [NITROS](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html), and [cuMotion documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/index.html).
[^27]: NVIDIA, [FoundationPose paper](https://arxiv.org/abs/2312.08344); Open3D, [documentation](https://www.open3d.org/docs/latest/).
[^28]: Viam, [platform overview](https://docs.viam.com/what-is-viam/) and [hardware component model](https://docs.viam.com/hardware/configure-hardware/).
[^29]: Intrinsic, [Capabilities](https://www.intrinsic.ai/capabilities) and [Flowstate](https://www.intrinsic.ai/flowstate).
[^30]: NVIDIA, [GR00T](https://developer.nvidia.com/isaac/gr00t); OpenVLA, [project](https://openvla.github.io/) and [repository](https://github.com/openvla/openvla); Physical Intelligence, [π0.5 paper](https://arxiv.org/abs/2504.16054); Octo, [paper](https://arxiv.org/abs/2405.12213).
[^31]: MCAP, [format specification](https://mcap.dev/spec); Rerun, [data-layer overview](https://rerun.io/docs/overview/what-is-rerun); ROS 2, [`ros2_tracing`](https://github.com/ros2/ros2_tracing/blob/rolling/README.md).
[^32]: ISO, [ISO 10218-1:2025](https://www.iso.org/standard/73933.html), [ISO 10218-2:2025](https://www.iso.org/standard/73934.html), [ISO 3691-4:2023](https://www.iso.org/standard/83545.html), and [ISO 13849-1:2023](https://www.iso.org/standard/73481.html); IEC, [IEC 62061](https://webstore.iec.ch/en/publication/92835).
[^33]: ROS, [REP-2014: Benchmarking performance in ROS 2](https://ros.org/reps/rep-2014.html); RobotPerf, [benchmark suite](https://github.com/robotperf/benchmarks).
[^34]: UL Solutions, [UL 4600 Edition 3 update](https://www.ul.com/news/ul-4600-edition-3-updates-incorporate-autonomous-trucking).
