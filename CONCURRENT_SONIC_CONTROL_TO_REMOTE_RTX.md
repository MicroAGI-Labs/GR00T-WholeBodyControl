# Concurrent SONIC Control to a Remote RTX Simulator

## Goal

Run multiple independent G1 robots in one vectorized Isaac Sim process on the
remote RTX PRO 6000, with one SONIC controller per robot on the DGX Spark.
Commands, state, reset/re-arm, evaluation, logs, and operator input must always
map to exactly one robot.

This extends the single-robot setup in
[`RTX_SIM_GUIDE.md`](RTX_SIM_GUIDE.md). It keeps the same SONIC binary and
Unitree message types as the physical robot. Simulation routing is selected by
configuration; physical-robot defaults stay unchanged.

The MVP deliberately uses:

- one vectorized Isaac process;
- one DDS domain on each host;
- one Zenoh bridge on each host;
- one supervised SSH tunnel;
- namespaced DDS topics for robot isolation;
- in-process state and command records inside Isaac.

Do not add DDS sidecars, cross-process IPC, one bridge per robot, or one tunnel
port per robot for the MVP.

---

## 1. Milestones and success criteria

### MVP: two pairs at RTF 0.1

The first milestone is complete when:

1. One Isaac process contains two independently simulated G1s.
2. Two SONIC processes run on the Spark.
3. Each controller receives only its robot's state and only its command reaches
   that robot.
4. Both robots hold stationary IDLE balance concurrently for 60 simulated
   seconds.
5. Robot 0 can remain in IDLE while robot 1 WALKS and returns to IDLE.
6. Resetting or re-arming one robot does not reset, pin, or release the other.
7. Restarting one SONIC process affects only its robot.
8. Restarting the shared tunnel or either bridge may briefly interrupt both
   robots, but both enter safe recovery and reconnect without a full restart.
9. Logs, ZMQ endpoints, diagnostics, and evaluation identify the robot and do
   not collide.
10. `SIM_ROBOT_COUNT=1` remains the default and reproduces the current
    single-robot balance result.

### Second milestone: four pairs

After the two-pair MVP passes, increase to four pairs and repeat the cross-talk,
selective reset, mixed-mode, restart, and 60-second balance tests. Four pairs
are the first supported concurrent configuration.

### Deferred

- Eight or more simulated robots.
- Twelve- or sixteen-controller capacity claims.
- A separate DDS domain, bridge, or SSH port per robot.
- DDS sidecars and a versioned cross-process shared-memory protocol.
- Failure isolation for shared Isaac, bridge, or tunnel processes.
- Per-robot cameras or multiple VLA stacks; use one overview viewport.
- RTF 1.0 over the current WARP/SSH route.
- Training or modifying SONIC.

If later requirements demand unmodified topic names for every simulated robot
or independent bridge failure domains, reconsider DDS sidecars after four
robots are working and measured.

---

## 2. Validated baseline

Measurements and live inspection on 2026-07-21:

| Resource or signal | Measurement |
|---|---:|
| Spark CPU | 20 ARM cores |
| Spark memory | 119 GiB total, about 114 GiB available |
| One SONIC at RTF 0.1 | about 8% of one CPU core, 0.54 GiB RSS, 24 threads |
| Four SONIC processes | about 0.32 CPU cores total, 2.15 GiB RSS |
| RTX CPU/memory | 48 vCPUs, 176 GiB RAM |
| Current Isaac process | about 5.9 GiB RSS and 0.6 CPU core |
| Current flat-sim internal step work | about 29 ms |
| Current slow-motion wall period | about 200 ms |
| Achieved RTF | 0.100 |
| Pod-side `rt/lowstate` | 99.3 Hz, 12.8 ms maximum gap, no >100 ms stalls |

The live RTX stack has one Isaac process, one in-process DDS manager, one Zenoh
bridge, one secondary-IMU adapter, and one camera publisher. DDS currently runs
in threads inside Isaac. The JSON shared-memory mailboxes are also used within
that process; they are not an existing sidecar API.

The timing suggests headroom for two to four vectorized robots at RTF 0.1, but
only an actual multi-environment run establishes capacity.

Before multiple controllers:

- Remove the unconditional CPU-0 pin in `G1Deploy::SetThreadPriority()`.
- Replace fractional `sleep(0.02)` calls with
  `std::this_thread::sleep_for(...)`; POSIX `sleep` accepts whole seconds.

Do not use `--output-type none` or `--output-type log` for active control. The
current deploy treats both as log-only and disables motor command publication.
Use `--output-type all` until that behavior is separately redesigned.

---

## 3. Current multi-robot blockers

Isaac Lab supports vectorized environments and the flat scene already uses
environment spacing and replicated physics. The custom path remains
single-robot:

- `sim_main.py` passes `num_envs=1`.
- The flat scene config declares one environment.
- State tensors are batched but only row zero is published.
- The action provider stores one command and returns one row.
- Gain, velocity, torque, latency, and warmup buffers assume one row.
- Base hold, joint hold, reset/re-arm, disturbance, and evaluation are global.
- Simulator body, Dex3, reset, evaluation, and secondary-IMU topics are fixed.
- SONIC body, Dex3, secondary-IMU, and motion-switcher topics are fixed.
- Controller ZMQ ports and telemetry paths collide by default.
- `stack_singleton.py` assumes one bridge, adapter, camera publisher, and sim.

Simply increasing `num_envs` would broadcast one action to all robots and
publish only one robot's state. Never use that as an intermediate setup.

---

## 4. MVP architecture

```text
DGX Spark                                         RTX PRO 6000 pod

SONIC 0 -- rt/sim/g1/0/** --\
                              one Spark     one SSH     one RTX
SONIC 1 -- rt/sim/g1/1/** ---- DDS/Zenoh == tunnel == DDS/Zenoh ---- Isaac envs 0..N-1
                              bridge        :7447       bridge          |
SONIC N -- rt/sim/g1/N/** --/                         in-process robot records
```

All SONIC processes use Spark DDS domain 0 because their topic names differ.
One Spark bridge forwards the configured topics through the existing `:7447`
tunnel to one RTX bridge on DDS domain 1.

### Topic naming

Add `SONIC_TOPIC_PREFIX`. Empty is the physical-robot default and must preserve
every existing topic exactly. Controller `i` uses:

```text
SONIC_TOPIC_PREFIX=rt/sim/g1/i
```

| Signal | Empty/default | Prefix `rt/sim/g1/i` |
|---|---|---|
| body state | `rt/lowstate` | `rt/sim/g1/i/lowstate` |
| body command | `rt/lowcmd` | `rt/sim/g1/i/lowcmd` |
| secondary IMU | `rt/secondary_imu` | `rt/sim/g1/i/secondary_imu` |
| reset/re-arm | `rt/reset_pose/cmd` | `rt/sim/g1/i/reset_pose/cmd` |
| evaluation | `rt/eval` | `rt/sim/g1/i/eval` |
| left Dex3 command | `rt/dex3/left/cmd` | `rt/sim/g1/i/dex3/left/cmd` |
| left Dex3 state | `rt/dex3/left/state` | `rt/sim/g1/i/dex3/left/state` |
| right Dex3 command | `rt/dex3/right/cmd` | `rt/sim/g1/i/dex3/right/cmd` |
| right Dex3 state | `rt/dex3/right/state` | `rt/sim/g1/i/dex3/right/state` |

Use one topic helper in each language. SONIC, simulator endpoints, diagnostics,
and reset tools must share the rule.

The sim has no Unitree motion-switcher service. Add an explicit option such as
`SONIC_SKIP_MOTION_SWITCHER=1` rather than namespacing that service for the MVP.
Physical-robot behavior remains the default.

### In-process state and command boundary

Create one record per environment containing:

- latest 29-joint state and pelvis IMU;
- latest 29-joint command, gains, velocity target, and feed-forward torque;
- state and command sequence counters and monotonic timestamps;
- reset/re-arm request and sequence;
- reset epoch or minimum accepted command sequence;
- evaluator state and result.

DDS callbacks update their record under a small lock. Isaac snapshots all
commands into batched tensors and updates all state records after physics. This
avoids cross-process shared memory, seqlocks, resource-tracker behavior, and
sidecar lifecycle.

Freshness and reset epochs are required. Release must never apply a command
cached before that robot's latest re-arm.

### Secondary IMU

Publish each secondary IMU from the same simulator endpoint as its LowState.
This removes the separate IMU adapter from the concurrent path. Retain the
adapter for the old single-robot path until that path is migrated and retested.

### Failure model

Per-robot components are SONIC, ZMQ endpoints, logs, topics, and command/state
records. Restarting one must not affect another robot.

Isaac, both bridges, the tunnel, and overview viewport are shared. Their restart
may interrupt all robots. The requirement is safe damping and automatic
recovery, not zero interruption. Per-robot bridge isolation is not an MVP claim.

---

## 5. Configuration contract

```text
SIM_ROBOT_COUNT=2
SIM_TOPIC_PREFIX_BASE=rt/sim/g1
ZMQ_INPUT_PORT_BASE=5556
ZMQ_OUTPUT_PORT_BASE=5657
```

For robot `i`:

```text
SONIC_INSTANCE_ID=i
SONIC_TOPIC_PREFIX=${SIM_TOPIC_PREFIX_BASE}/i
ZMQ_INPUT_PORT=${ZMQ_INPUT_PORT_BASE}+i
ZMQ_OUTPUT_PORT=${ZMQ_OUTPUT_PORT_BASE}+i
```

Rules:

- Validate robot count and reject duplicate prefixes, ports, and output paths.
- Put `robot_id` in process labels, logs, telemetry, eval, and diagnostics.
- Never treat a missing robot ID as broadcast.
- Never use `pkill -f` for per-controller lifecycle operations.
- Keep Spark DDS domain 0, RTX DDS domain 1, and Zenoh port `7447`.
- `SIM_ROBOT_COUNT=1` with empty prefix is the compatibility configuration.

---

## 6. Implementation plan

### Phase 0 — Freeze the known-good baseline

- [ ] Commit the observed-pose, exact-hold, current-command caching,
  configurable IDLE reference, and leg-blend work.
- [ ] Run joint-mapping tests and the C++ build.
- [ ] Run the 60-simulated-second balance test at RTF 0.1.
- [ ] Record launch variables, build, posture, and evaluator result.

Exit: after a complete restart, one robot balances for 60 simulated seconds.

### Phase 1 — Make SONIC instance-clean

- [ ] Fix fractional sleeps and unconditional CPU-0 affinity.
- [ ] Add the shared topic helper and `SONIC_TOPIC_PREFIX`.
- [ ] Prefix LowState, LowCmd, secondary IMU, and Dex3.
- [ ] Add `SONIC_SKIP_MOTION_SWITCHER=1` for simulation.
- [ ] Add `SONIC_INSTANCE_ID` for labels.
- [ ] Require unique ZMQ ports and telemetry/log paths.
- [ ] Guard active launches against output types `none` and `log`.

Tests:

- [ ] Empty prefix exactly preserves physical-robot topics.
- [ ] Two SONIC processes use distinct prefixes on one DDS domain.
- [ ] Restart controller 1 while controller 0 remains alive.
- [ ] Confirm neither process busy-spins and record loop timing.

Exit: two SONIC processes run against synthetic namespaced feeds without topic
or port collisions.

### Phase 2 — Vectorize Isaac state and control

- [ ] Add validated `SIM_ROBOT_COUNT`/`--num_envs`.
- [ ] Configure scene count and spacing.
- [ ] Make action, gain, velocity, torque, warmup, and latency buffers batched.
- [ ] Read one command record per environment and apply only to that row.
- [ ] Publish every state row in canonical Unitree 29-joint order.
- [ ] Create one in-process namespaced DDS endpoint per environment.
- [ ] Publish secondary IMU directly from each endpoint.
- [ ] Keep Dex3 optional initially; if enabled, make it per-environment.
- [ ] Preserve exact behavior for `N=1`.

Tests:

- [ ] Batched mapping round-trip for `N=1,2,4`.
- [ ] Distinct sentinel commands prove exact action-row isolation.
- [ ] Distinct state rows reach only their matching subscribers.
- [ ] Two pinned robots run ten simulated minutes without contact.

Exit: two independently addressable robots step locally in one Isaac process.

### Phase 3 — Make lifecycle state per robot

- [ ] Replace global hold, warmup, release, and re-arm state with per-env masks.
- [ ] Apply pose, velocity, joint, wrench, and disturbance writes by env ID.
- [ ] Route each reset/re-arm topic to exactly one environment.
- [ ] Give every environment an evaluator, start pose, timer, and result topic.
- [ ] Invalidate the old command epoch on re-arm.
- [ ] Release only after a fresh command arrives for that robot.
- [ ] Verify one fall or completed eval does not mutate another robot.

Exit: reset, re-arm, release, and evaluation work independently for two robots.

### Phase 4 — Use the existing remote transport

- [ ] Generate one explicit allow-list for all configured namespaced topics.
- [ ] Keep one RTX bridge on DDS domain 1 and port `7447`.
- [ ] Keep one Spark bridge on DDS domain 0.
- [ ] Keep the existing supervised single-port tunnel.
- [ ] Add `--robot-id` or `--topic-prefix` to diagnostics.
- [ ] Report state/command rates, age, sequence, and reconnect count per robot.

Tests:

- [ ] Observe both remote state streams simultaneously.
- [ ] Send opposite sentinel commands and prove end-to-end isolation.
- [ ] Restart one SONIC process while the other remains controlled.
- [ ] Restart each shared bridge and the tunnel; both controllers damp and
  recover after the route returns.

Exit: two remote pairs communicate without cross-talk and recover from shared
transport interruption.

### Phase 5 — Demonstrate two, then scale to four

- [ ] Start two pinned robots and two SONIC controllers.
- [ ] Release both and hold IDLE for 60 simulated seconds.
- [ ] Keep robot 0 IDLE while robot 1 WALKS, then returns to IDLE.
- [ ] Re-arm/release robot 1 while robot 0 keeps balancing.
- [ ] Restart controller 1 and verify only robot 1 recovers.
- [ ] Record RTF, step timing, CPU/RSS, GPU use, message age, and balance.
- [ ] Add small checked-in launch/status/stop scripts after manual success.
- [ ] Repeat the entire suite with `SIM_ROBOT_COUNT=4`.

Exit: four robots pass without cross-talk and maintain RTF at or above 0.095.

---

## 7. Required tests and measurements

Essential unit tests:

- Empty prefix preserves exact default topics.
- IDs 0 through 3 generate unique complete topic sets.
- Batched 29-joint mappings round-trip for one, two, and four rows.
- A command for robot `i` changes only action row `i`.
- State row `i` is published only on prefix `i`.
- Reset/re-arm changes only the selected environment.
- Commands from a previous reset epoch are rejected.
- Duplicate prefixes, ports, and paths are rejected.

MVP integration tests:

1. Opposite sentinel commands with exact routing assertions.
2. Two concurrent 60-second IDLE balance runs.
3. Robot 0 IDLE while robot 1 WALKS and returns to IDLE.
4. Selective reset/re-arm of robot 1.
5. Restart controller 1 while robot 0 continues.
6. Restart shared transport and verify safe all-robot recovery.
7. Repeat with four robots.

At robot counts 1, 2, and 4, record:

- achieved RTF and Isaac step p50/p95/p99/max;
- RTX CPU/RSS and GPU utilization/memory;
- Spark per-controller CPU/RSS and loop p50/p95/p99/max;
- state/command rate, age, missing sequences, and reconnects;
- per-robot displacement, height, tilt, termination, and score.

Stop increasing concurrency if:

- any cross-talk or non-selective reset occurs;
- RTF falls below 0.095 for the 0.1 target;
- controllers miss deadlines or repeatedly enter feed-loss recovery;
- message age exceeds its safe threshold;
- GPU or host memory exceeds 80%;
- the known-good single-robot test stops reproducing.

Do not benchmark 8, 12, or 16 controllers until four real pairs pass.

---

## 8. Minimal rollout order

1. Commit and reproduce one robot.
2. Fix SONIC sleep/affinity and add topic prefixes.
3. Test two SONIC processes with synthetic namespaced feeds.
4. Vectorize Isaac to two environments.
5. Prove local DDS isolation and selective reset.
6. Add namespaced topics to the existing bridge pair.
7. Run the remote two-robot balance and mixed-mode demonstration.
8. Add minimal launch/status/stop scripts.
9. Increase to four and repeat.
10. Decide from measurements whether further transport isolation is worthwhile.

---

## 9. Expected code changes

Likely existing files:

- `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp`
- `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/robot_parameters.hpp`
- `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/dex3_hands.hpp`
- `rtx_pod/unitree_sim_isaaclab/sim_main.py`
- `rtx_pod/unitree_sim_isaaclab/action_provider/action_provider_lowcmd29.py`
- `rtx_pod/unitree_sim_isaaclab/tasks/common_observations/g1_29dof_state.py`
- `rtx_pod/unitree_sim_isaaclab/dds/g1_robot_dds.py`
- deployed upstream simulator Dex3/reset DDS files
- `rtx_pod/unitree_sim_isaaclab/tools/stand_eval.py`
- flat task environment configuration
- `rtx_pod/start_flat.sh`
- `rtx_pod/zenoh-sim-bridge.json5`
- `rtx_pod/g1_dds_diag.py`
- Spark bridge configuration in `gear_sonic_deploy/deploy.sh`

Small new components may include:

- one topic-name helper per language;
- one in-process multi-robot record class;
- `rtx_pod/start_concurrent_flat.sh`;
- `start_concurrent_sonic.sh`;
- focused batched-routing and cross-talk tests.

Avoid a manifest framework or systemd integration until the manual two-robot
path works. Validated shell launchers are sufficient for the MVP.

---

## 10. Operational invariants

- Every state, command, reset, result, endpoint, and log maps to one robot.
- Empty topic prefix preserves the physical-robot protocol exactly.
- No command or reset is broadcast implicitly.
- No robot consumes a command from before its current reset epoch.
- Restarting one SONIC process does not interrupt another robot.
- Shared transport loss triggers safe recovery for every controller.
- The single-robot workflow remains the default.
- Capacity claims require simultaneous independent feeds and measured timing.

---

## 11. Current status and next action

The single-robot simulator is live and stable at RTF 0.1. Parts of the state
path already use batched Isaac tensors, but transport, actions, lifecycle state,
evaluation, and controller topics remain single-robot.

No concurrent control path has been implemented.

Next: complete Phase 0, then the small SONIC process/topic cleanup in Phase 1
before changing Isaac's environment count.
