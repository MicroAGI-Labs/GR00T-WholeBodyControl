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

## 2. Validated baseline and two-robot bring-up

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
| Two-env Spark-side state, robot 0 | 102.0 Hz, 60.6 ms maximum gap, no >100 ms stalls |
| Two-env Spark-side state, robot 1 | 101.9 Hz, 56.6 ms maximum gap, no >100 ms stalls |
| Two-env 60 s free IDLE result | both success; 0.00 m displacement; 2.0° / 1.9° final tilt |

The two-robot path was brought up on 2026-07-21. The live RTX stack has one
two-environment Isaac process, one in-process multi-robot DDS manager, one
Zenoh bridge, no secondary-IMU adapter, and one camera publisher. Two SONIC
processes on the Spark receive distinct state streams and publish distinct
commands. Both robots have been independently re-armed and released and are
standing under SONIC control at RTF 0.1. Both evaluators reached 60.02 simulated
seconds with zero reported displacement and no fall. Robot 0 finished at
0.683 m pelvis height and 2.0° tilt with result +95.34; robot 1 finished at
0.684 m and 1.9° with result +95.59.

The two-environment run establishes capacity for two at RTF 0.1. Four remains
an unmeasured follow-on.

Before multiple controllers:

- Remove the unconditional CPU-0 pin in `G1Deploy::SetThreadPriority()`.
- Replace fractional `sleep(0.02)` calls with
  `std::this_thread::sleep_for(...)`; POSIX `sleep` accepts whole seconds.

Do not use `--output-type none` or `--output-type log` for active control. The
current deploy treats both as log-only and disables motor command publication.
Use `--output-type all` until that behavior is separately redesigned.

---

## 3. Implemented state and remaining MVP gaps

Implemented in the current working tree:

- configurable SONIC topic prefixes with exact empty-prefix compatibility;
- per-instance controller identity, ZMQ endpoints, logs, and telemetry;
- simulator-only motion-switcher bypass and corrected fractional sleeps/CPU
  affinity behavior;
- `--num_envs`/`SIM_ROBOT_COUNT`, batched actions and states, one command/state
  record per environment, and one namespaced DDS endpoint per environment;
- per-environment hold, reset epoch, fresh-command gate, evaluator, re-arm, and
  release;
- in-process namespaced secondary-IMU publication for the concurrent path;
- namespaced diagnostics and lifecycle publisher;
- one shared tunnel and bridge per host with explicit interface selection;
- `forward_discovery: false` local routing, which avoids remote DDS endpoint
  replicas and allows either bridge or controller to start first;
- explicit localhost discovery on the Spark bridge because `lo` has no
  multicast discovery.

The remaining MVP work is validation and operator packaging, not another
transport layer:

- run opposite-command and mixed IDLE/WALK cross-talk tests;
- verify selective reset while the other robot remains free-standing;
- restart one controller and each shared transport component during control;
- retest the `SIM_ROBOT_COUNT=1` compatibility path;
- add a minimal checked-in two-controller launch/status/stop wrapper after the
  manual sequence is stable;
- route per-environment Dex3 in a later milestone; hands are explicitly
  disabled for the concurrent body-control MVP.

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

Both bridges use local DDS route mode (`forward_discovery: false`). The RTX
bridge binds CycloneDDS to the pod's default-route interface (`eth0` in the
validated run). The Spark bridge binds to `lo` and explicitly scans localhost
participant indices 0 through 99; loopback is not multicast-capable, so leaving
discovery implicit causes controllers to wait forever even when Zenoh is
connected. Keep shared memory disabled because the bridge peers are on separate
hosts across TCP/SSH.

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
```

Use these exact per-controller values for the two-robot MVP:

| Setting | Robot 0 | Robot 1 |
|---|---:|---:|
| `SONIC_INSTANCE_ID` | `0` | `1` |
| `SONIC_TOPIC_PREFIX` | `rt/sim/g1/0` | `rt/sim/g1/1` |
| `ZMQ_INPUT_PORT` | `5556` | `5558` |
| `ZMQ_OUTPUT_PORT` | `5657` | `5658` |
| `IDLE_TELEMETRY_LOGFILE` | `/tmp/sonic0_idle.csv` | `/tmp/sonic1_idle.csv` |

Rules:

- Validate robot count and reject duplicate prefixes, ports, and output paths.
- Put `robot_id` in process labels, logs, telemetry, eval, and diagnostics.
- Never treat a missing robot ID as broadcast.
- Never use `pkill -f` for per-controller lifecycle operations.
- Keep Spark DDS domain 0, RTX DDS domain 1, and Zenoh port `7447`.
- `SIM_ROBOT_COUNT=1` with empty prefix is the compatibility configuration.

---

## 6. Implementation plan

### Phase 0 — Freeze the known-good baseline (complete)

- [x] Commit the observed-pose, exact-hold, current-command caching,
  configurable IDLE reference, and leg-blend work.
- [x] Run joint-mapping tests and the C++ build.
- [x] Run the single-robot 60-simulated-second balance test at RTF 0.1.
- [x] Record launch variables, build, posture, and evaluator result.

Exit: after a complete restart, one robot balances for 60 simulated seconds.

### Phase 1 — Make SONIC instance-clean (implementation complete)

- [x] Fix fractional sleeps and unconditional CPU-0 affinity.
- [x] Add the shared topic helper and `SONIC_TOPIC_PREFIX`.
- [x] Prefix LowState, LowCmd, secondary IMU, and Dex3.
- [x] Add `SONIC_SKIP_MOTION_SWITCHER=1` for simulation.
- [x] Add `SONIC_INSTANCE_ID` for labels.
- [x] Use unique ZMQ ports and telemetry/log paths in concurrent launches.
- [ ] Guard active launches against output types `none` and `log`.

Tests:

- [x] Empty prefix exactly preserves physical-robot topics.
- [x] Two SONIC processes use distinct prefixes on one DDS domain.
- [ ] Restart controller 1 while controller 0 remains alive.
- [ ] Confirm neither process busy-spins and record loop timing.

Exit: two SONIC processes run against synthetic namespaced feeds without topic
or port collisions.

### Phase 2 — Vectorize Isaac state and control (body path complete)

- [x] Add validated `SIM_ROBOT_COUNT`/`--num_envs`.
- [x] Configure scene count and spacing.
- [x] Make action, gain, velocity, torque, warmup, and latency buffers batched.
- [x] Read one command record per environment and apply only to that row.
- [x] Publish every state row in canonical Unitree 29-joint order.
- [x] Create one in-process namespaced DDS endpoint per environment.
- [x] Publish secondary IMU directly from each endpoint.
- [x] Keep Dex3 disabled in concurrent mode until it is per-environment.
- [x] Keep the existing single-robot code path as the `N=1` default.

Tests:

- [ ] Batched mapping round-trip for `N=1,2,4` (single-row joint mapping is covered).
- [ ] Distinct sentinel commands prove exact action-row isolation.
- [x] Distinct state rows reach only their matching subscribers in the live run.
- [ ] Two pinned robots run ten simulated minutes without contact.

Exit: two independently addressable robots step locally in one Isaac process.

### Phase 3 — Make lifecycle state per robot (implementation complete)

- [x] Replace global hold, warmup, release, and re-arm state with per-env masks.
- [x] Apply lifecycle pose, velocity, and joint writes by env ID.
- [x] Route each reset/re-arm topic to exactly one environment.
- [x] Give every environment an evaluator, start pose, timer, and result topic.
- [x] Invalidate the old command epoch on re-arm.
- [x] Release only after a fresh command arrives for that robot.
- [ ] Verify one fall or completed eval does not mutate another robot.

Exit: reset, re-arm, release, and evaluation work independently for two robots.

### Phase 4 — Use the existing remote transport (data path complete)

- [x] Generate one explicit allow-list for all configured namespaced topics.
- [x] Keep one RTX bridge on DDS domain 1 and port `7447`.
- [x] Keep one Spark bridge on DDS domain 0.
- [x] Keep the existing supervised single-port tunnel.
- [x] Add `--topic-prefix` to diagnostics.
- [ ] Report state/command rates, age, sequence, and reconnect count per robot.

Tests:

- [x] Observe both remote state streams simultaneously.
- [ ] Send opposite sentinel commands and prove end-to-end isolation.
- [ ] Restart one SONIC process while the other remains controlled.
- [ ] Restart each shared bridge and the tunnel while free-standing; both controllers damp and
  recover after the route returns.

The Spark bridge has been restarted while both controllers were waiting. Both
controllers recovered to `Init Done` without being restarted, which validates
startup-order independence for the state path. The safety behavior during a
free-standing transport interruption remains to be tested.

Exit: two remote pairs communicate without cross-talk and recover from shared
transport interruption.

### Phase 5 — Demonstrate two, then scale to four (in progress)

- [x] Start two pinned robots and two SONIC controllers.
- [x] Release both and hold IDLE for 60 simulated seconds.
- [ ] Keep robot 0 IDLE while robot 1 WALKS, then returns to IDLE.
- [ ] Re-arm/release robot 1 while robot 0 keeps balancing.
- [ ] Restart controller 1 and verify only robot 1 recovers.
- [ ] Record RTF, step timing, CPU/RSS, GPU use, message age, and balance.
- [ ] Add small checked-in launch/status/stop scripts after manual success.
- [ ] Repeat the entire suite with `SIM_ROBOT_COUNT=4`.

Acceptance result: both robots reached 60.02 simulated seconds concurrently
with zero displacement and no fall.

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

1. [x] Commit and reproduce one robot.
2. [x] Fix SONIC sleep/affinity and add topic prefixes.
3. [x] Start two SONIC processes with isolated topics and endpoints.
4. [x] Vectorize Isaac to two environments.
5. [x] Route two state/command pairs through the existing bridge and tunnel.
6. [x] Re-arm and release each environment through its namespaced lifecycle topic.
7. [x] Finish the 60-simulated-second concurrent IDLE run.
8. [ ] Run opposite-command, mixed-mode, selective-reset, and restart tests.
9. [ ] Add minimal launch/status/stop scripts and retest `N=1`.
10. [ ] Increase to four only after the two-robot acceptance suite passes.

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

New components in the current implementation:

- one topic-name helper per language;
- `rtx_pod/unitree_sim_isaaclab/dds/g1_multi_robot_dds.py` for per-environment
  records and DDS endpoints;
- focused topic and joint-mapping tests.

Still justified after manual acceptance:

- a small `start_concurrent_sonic.sh` plus matching status/stop helpers;
- focused batched row-isolation and lifecycle-epoch tests.

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

The concurrent body-control path is live at RTF 0.1:

- RTX: one Isaac process with two environments, one bridge, one camera
  publisher, and no separate IMU adapter;
- Spark: two SONIC processes, one bridge, and one autossh tunnel;
- robot 0 and robot 1 each receive about 102 Hz namespaced state and publish an
  isolated LowCmd;
- the Spark bridge was restarted after both SONIC processes had started; both
  recovered without a controller restart;
- namespaced re-arm and release requests were acknowledged independently by the
  simulator;
- both robots completed the concurrent 60.02-simulated-second IDLE evaluation
  with zero displacement, no fall, and final tilt of 2.0° and 1.9°.

Next, run the mixed IDLE/WALK, selective reset, one-controller restart, and
shared-transport recovery checks. Do not add sidecars, more bridges, or a
manifest framework before these tests expose a concrete need.
