# RTX6000 Sim Latency — why SONIC balance is RTF-capped, and how far we pushed it

**TL;DR** — Running the SONIC low-level controller on the Spark against the Isaac
sim on the remote RTX6000 pod, the G1 balances **only when the sim runs slower than
real time**. With the compressed one-tunnel architecture, eight robots completed
the full 60.02 simulated-second evaluator at every tested point through **RTF
0.20**. Use 0.20 for supervised throughput and 0.175 as the conservative
fallback. At **RTF 0.225**, two robots fell at about 29 simulated seconds. A later
eight-robot RTF 0.20 run fell after 492.3 simulated seconds during a synchronized
state-delivery discontinuity while Isaac remained at RTF 0.198; that is a shared
transport-path incident, not a lower measured simulator-throughput ceiling.
Going faster reliably needs either a lower-latency network path (infra) or a
delay-robust controller (retrain SONIC). First-order state prediction and
gain-softening were implemented and tested — neither moved the earlier ceiling.

Related: [`RTX_SIM_GUIDE.md`](RTX_SIM_GUIDE.md) (bring-up),
[`SIM_RESILIENCE_PLAN.md`](SIM_RESILIENCE_PLAN.md) (auto-recovery + the working balance recipe),
memory `sonic-rtf-delay-tolerance-levers`, `sim-resilience-implementation`,
`sonic-imu-bug-and-concurrent-harness`.

> **Update 2026-07-10.** Balance reproduced, and two refinements to the numbers below:
> (1) With the current `autossh` tunnel the RTT is higher, so a clean stand needed RTF
> **0.10–0.125** (`sim_slowmo` 8–10), not 0.333 — same law (`stable RTF ≈ margin / RTT`),
> just a bigger RTT. (2) A clean release requires the right **warmup geometry**: RIGID hold
> (`SIM_BASE_SOFT=0`) + `SIM_WARMUP_JOINTS=1` holding the policy's default stance (knee 0.669)
> at init z **0.793** (feet-on-ground), from a **fresh** deploy. First send cat-4 to re-arm
> the upright hold, then release with cat-3. z=0.8
> jams the knees to ~1.9 and topples on release; SOFT hold flails. See `SIM_RESILIENCE_PLAN.md`
> §8 and memory `sim-resilience-implementation`.

> **Update 2026-07-21.** The same RTF 0.1 transport now carries two independent
> namespaced SONIC control loops through one tunnel and one bridge per host. One
> vectorized Isaac process publishes both 100 Hz state streams in-process. The
> Spark measured 102.0 Hz and 101.9 Hz with no gaps above 100 ms during the probe;
> Isaac held RTF 0.100. Both robots were released under separate SONIC processes
> and both completed 60.02 simulated seconds with zero displacement and no fall;
> final tilt was 2.0° and 1.9°. See
> [`CONCURRENT_SONIC_CONTROL_TO_REMOTE_RTX.md`](CONCURRENT_SONIC_CONTROL_TO_REMOTE_RTX.md).

> **RTF sweep 2026-07-21.** A clean-reset, two-controller sweep with
> `CONTROL_WALL_SCALE` matched to `1 / sim_slowmo` supersedes the earlier RTF 0.333
> headline for the current remote concurrent setup. Both robots passed 60.02 s at
> RTF 0.20, although with substantial transient translation; both fell near 29 s
> at RTF 0.225. Results above 0.225 were intermittent and non-monotonic because
> transport/controller timing jitter affects when the instability is excited, but
> every long-enough tested point above 0.20 produced at least one fall.

> **Concurrency sweep 2026-07-21.** At the deliberately conservative RTF 0.15,
> four robots/controllers completed 60.02 simulated seconds together with 0.09 m
> displacement and 1.9–2.0° tilt each. Five failed at 14.06 s in a clean repeat;
> six was intermittent, and seven and eight failed repeatedly. Twenty-four
> pinned controllers could initialize, but reduced state delivery to 24–26 Hz
> and pulled cumulative Isaac RTF toward 0.12, so 24 is a startup-capacity result,
> not a supported control count.

> **RTF 0.10 transport follow-up 2026-07-21.** An uncompressed eight-robot run
> failed at 36.84 s (an earlier repeat failed at 10.50 s). Direct pod probes were
> healthy at 99.6 Hz state / 50.0 Hz command, while Spark saw only 63.1–63.5 Hz /
> 29.8–30.0 Hz, command p99 gaps of 142–150 ms, and maxima up to 0.70 s. Enabling
> SSH compression restored 97.5–97.7 Hz / 49.0–49.4 Hz, reduced command p99 to
> 39–46 ms, and all eight completed 60.02 s at 0.09 m displacement and 1.8–2.2°
> tilt. This established eight at RTF 0.10; the later compressed-tunnel sweep
> superseded the old four-at-0.15 boundary and passed eight through RTF 0.20.

> **Long-horizon monitoring update 2026-07-22.** An eight-robot RTF 0.20 run
> passed 60 seconds, remained upright for 492.3 simulated seconds (about 41 wall
> minutes), then all eight crossed 45° within 0.1 simulated seconds. Their Spark
> telemetry repeated the last upright state and then jumped directly to 87–89°;
> Isaac's RTF stayed at 0.198 and no process or SSH child restarted. This rules
> out gradual simulator slowdown and points to a shared state-delivery gap, but
> does not identify the exact network/bridge layer. Continuous evaluators and
> compact monitors now run on both sides of the bridge for exact attribution.

---

## 1. Setup and constraint

| Component | Where | Notes |
|---|---|---|
| SONIC deploy (`g1_deploy_onnx_ref`) | **Spark** (aarch64) | reads `rt/lowstate`, writes `rt/lowcmd` at 50 Hz over DDS |
| Isaac sim (`sim_main.py`) | **RTX6000 pod** (remote k8s) | `--device cuda`, 100 Hz step, publishes `rt/lowstate` |
| Transport | zenoh-bridge-dds ↔ `ssh -L :7447` | Spark DDS domain 0 ↔ pod domain 1 |

Hard requirements (unchanged by this work): **controller runs on the Spark**, **sim
runs on the RTX6000**, and the **DDS interface stays identical to the real G1** so the
same deploy binary drives sim or hardware.

The RTX6000 pod is reachable **only** via Cloudflare WARP at `spark@10.5.7.178`,
**port 22 only, UDP blocked**. So all DDS traffic is forced through a single
`ssh -L 7447` tunnel — TCP-over-TCP. There is no LAN, tailscale, or WireGuard path.

---

## 2. Loop profile — the network dominates

Per control cycle at RTF 1.0 (wall-clock):

| Segment | Latency | Reducible from Spark? |
|---|---:|---|
| Sim publishes lowstate → Spark (one-way) | ~19 ms | No (WARP) |
| Arrival → deploy consumes (async gap) | 2–5 ms | Partly |
| Deploy compute (obs + policy + motor cmd) | ~1.2 ms | No (already tiny) |
| Spark publishes lowcmd → pod (one-way) | ~19 ms | No (WARP) |
| lowcmd arrival → sim applies | 0–20 ms | Partly |
| **Round-trip dead time** | **~40–60 ms** | — |

Measurements taken this session:
- Warm established-socket RTT through the real ssh path: **p50 38 ms, p90 50 ms**
  (cold `connect()` SYN→SYN-ACK min 42 ms — warm ≈ cold, so 38 ms is a genuine floor,
  not tunable overhead).
- Live DDS `rt/lowstate` inter-arrival at the Spark at RTF 1.0: **p50 9 ms, p99 23 ms,
  worst gap 69 ms** (zenoh coalesces stale samples, so real jitter is far milder than
  raw TCP-over-TCP would suggest).

> **Caveat on the deploy's own "LowState age" readout (2–5 ms):** it is stamped
> *locally on arrival* (`utils.hpp` `DataBuffer::SetData`), so it does **not** include
> the ~19 ms one-way transit. The controller is structurally blind to how stale its
> state actually is.

---

## 3. Why it falls — the governing relationship

RTF only changes wall-clock pacing; **the sim math per step is byte-identical** regardless
of RTF. So the *only* thing that changes between "balances" and "falls" is the control-loop
delay measured in **sim-time**:

```
sim-time loop delay  ≈  wall_delay (~40 ms) × RTF   +   async slop
```

Current measured sweep (one vectorized two-G1 Isaac process, two Spark SONIC
processes, fresh controller startup and robot reset at every point,
`CONTROL_WALL_SCALE` matched to RTF):

| RTF (= 1 / sim_slowmo) | Observation horizon | Result |
|---:|---:|---|
| 0.125 | 16.58 s | Both upright; short screen only |
| 0.1667 | 18.02 s | Both upright; short screen only |
| **0.20** | **60.02 s** | **Both succeeded; 0.55 m / 1.84 m displacement in the full repeat** |
| **0.225** | ~29 s | **Both fell: 29.52 s / 28.98 s** |
| 0.25 | 21.62 s | One robot fell at 19.40 s; the other was recovering |
| 0.2625 | 50.42 s | One fell at 44.40 s; the other remained upright but drifted 1.70 m |
| 0.275 | ~23 s | Both fell at 23.64 s / 22.74 s |
| 0.30 | 4.22 s | One fell at 3.42 s |
| 0.333 | 14–22 s | Intermittent: one run fell at 14.48 s; repeat survived 21.5 s with degraded control |

The non-monotonic fall times do not make the higher points safe. They show that
the exact time-to-fall depends on asynchronous transport and controller phase.
Use the pass/fail bracket, not a single short survival, when choosing an operating
point. In particular, the first RTF 0.25 screen was still upright at 18.5 s, but
its clean repeat fell at 19.4 s.

**Is 38 ms a physics limit? No.** A standing G1 is an inverted pendulum pivoting at the
ankles: CoM height L ≈ 0.6 m → unstable pole ωᵤ = √(g/L) ≈ 4 rad/s → the fundamental
delay bound for stabilizability is ωᵤ·T ≲ 1, i.e. **~250 ms**. At 38 ms, ωᵤ·T ≈ 0.15 —
seven times inside the physics budget.

The real limiter is the **controller**: SONIC is a stiff, high-bandwidth position-PD
policy (kp up to 99) that was effectively trained at zero latency. Delay eats phase
margin as ω_c·T, and the current concurrent sweep shows the combined loop becomes
unreliable between RTF 0.20 and 0.225 — long before the pendulum's own 250 ms limit
matters. The older single-robot/direct-path experiments tolerated RTF 0.333, but that
number is not a safe ceiling for the present autossh, two-controller architecture.

So:

```
stable RTF  ≈  controller delay margin / effective wall-clock loop delay

current measured bracket:  0.20 succeeds for 60 s; 0.225 falls near 29 s
```

---

## 4. Levers tried (all feature-flagged in the deploy; default = real-robot unchanged)

### 4a. Forward state prediction — **ineffective, mildly harmful**
On receipt, extrapolate the lowstate to cancel the loop delay: joint `q += dq·T`, and
integrate the IMU orientation quaternion by its gyro (`PredictLowStateForward` /
`PredictImuForward`, applied in `LowStateHandler` + `imuTorsoHandler`).

- Env `PRED_HORIZON_S`; live-tunable via `/tmp/pred_horizon` (`MaybeRefreshPredHorizon`).
- Result: no improvement at any RTF; larger horizons *increased* peak tilt. First-order
  extrapolation cannot catch a **slow-growing** delay instability — velocities stay small
  until divergence is already underway, so the prediction adds nothing useful (and injects
  wrong lead at the fast dynamics).

### 4b. Commanded-gain soften / extra damping — **marginal**
Scale the commanded `kp`/`kd` at the 500 Hz writer choke point to lower the effective
control bandwidth (wider delay margin).

- Live-tunable via `/tmp/gain_scale` = `"<kp_scale> <kd_scale>"` (`MaybeRefreshGainScale`).
- Result: reduces peak tilt (e.g. 100° → 69°) but does **not** prevent the fall at
  RTF ≥ 0.4; too soft (kp×0.35) falls *faster* because it can't hold the stance.

### 4c. What was ruled out earlier
Contact model, armature, joint friction, PhysX solver iterations, foot geometry, mass,
control rate — none explain the RTF dependence, and there is **no** PhysX/MuJoCo dynamics
gap (that earlier framing was wrong). The dependence is purely loop delay.

---

## 5. Result and operating point

**RTF 0.20 is the current supervised operating point with the compressed autossh
tunnel; use 0.175 for additional margin.** The earlier maximum-margin recipe was:

- Deploy: `CONTROL_WALL_SCALE=0.1` (must equal sim RTF).
- Pod sim: `/tmp/sim_slowmo=10` (RTF = 1 / slowmo).
- Stationary IDLE: `SONIC_IDLE_HOLD_REFERENCE=1`,
  `SONIC_IDLE_PITCH_BIAS_DEG=-8`, and `SIM_WARMUP_POSE=sonic`.
- Confirmed: 60.02 s unaided stand; final displacement 0.10 m, tilt 0.7°, no fall.
- Revalidated with synchronized reference/state/command telemetry: 60.02 s unaided
  stand, final displacement 0.03 m, height 0.707 m, tilt 1.3°, result +71.56.

That RTF 0.10 recipe remains useful for maximum delay tolerance. The current
eight-robot throughput setup instead uses `CONTROL_WALL_SCALE=0.20` and
`/tmp/sim_slowmo=5`.

For the two-robot bring-up, both controllers still used
`CONTROL_WALL_SCALE=0.1`, but the warmup and frozen IDLE reference were matched
to the observed posture: `SIM_WARMUP_POSE=observed`,
`SONIC_IDLE_PITCH_BIAS_DEG=0`, and `SONIC_IDLE_LEG_BLEND=0`. Both concurrent
evaluators reached 60.02 simulated seconds successfully; this is now the
recommended two-robot IDLE configuration.

The two-robot RTF sweep also completed 60.02 s at RTF 0.20, but with 0.55 m and
1.84 m displacement in that repeat. At RTF 0.225 both robots fell near 29 s.
Therefore:

- use **RTF 0.10** for the largest established latency margin;
- with the default compressed `sim_tunnel.sh`, eight concurrent robots passed
  60.02 s at RTF 0.10, 0.125, 0.15, 0.175, and **0.20** when controller timing
  matched each RTF;
- the eight-robot RTF 0.20 pass ended with every robot at 0.09 m displacement,
  0.682–0.685 m pelvis height, and 1.8–2.2° tilt;
- use **RTF 0.20** as the current supervised operating point and 0.175 as the
  conservative fallback until 0.20 has repeat qualification;
- do not use **RTF 0.225 or above** for unattended balance on the current route.

### Concurrency limit at RTF 0.15

This count sweep used the uncompressed-tunnel baseline. It held the rest of the architecture fixed: one Isaac process, one bridge per
host, one autossh tunnel, and one SONIC process per robot. Four is the highest
count that completed the full evaluator in that baseline; the later compressed
tunnel follow-up passed all eight at RTF 0.15 and supersedes this as the current
capacity result:

| Count | Longest relevant result |
|---:|---|
| **4** | **PASS 60.02 s; all at 0.09 m displacement and 1.9–2.0° tilt** |
| **5** | **FAIL 14.06 s; one fall, no tunnel reconnect or controller feed-loss event** |
| 6 | Intermittent: >20 s short pass, but two fell by 5.30 s in the long attempt |
| 7 | Two failures, first fall at 1.70 s and 4.76 s |
| 8 | Two failures, first fall at roughly 1.4–4.6 s |

One separate five-robot run ended with all five falling at about 23 simulated
seconds. It is excluded from the controller-count boundary because the autossh
child timed out and restarted at the same instant and every controller logged a
shared LowState outage. The bridges and controllers recovered automatically,
but restart resilience cannot preserve balance through a multi-second WAN gap.

At 24 pinned robots, all feeds existed and all 24 controllers initialized. The
Spark processes consumed 12.59 GiB RSS and about 7.9 CPU cores, while feed rate
fell to 24–26 Hz and Isaac no longer sustained RTF 0.15. This shows why process
initialization alone is not a useful concurrency claim.

### Concurrency limit at RTF 0.10

The lower rate plus SSH compression provides enough delay margin for eight
independent controllers:

| Count | Result |
|---:|---|
| **7** | **PASS 60.02 s; all at 0.09 m displacement and 1.8–2.2° tilt** |
| 8, uncompressed tunnel | Intermittent FAIL: 10.50 s in one repeat; 36.84 s in the instrumented repeat |
| **8, compressed tunnel** | **PASS 60.02 s; all at 0.09 m displacement and 1.8–2.2° tilt** |

The instrumented uncompressed run separated the bottleneck cleanly. At the pod,
all eight state streams were 99.6 Hz with 12–14 ms p99 gaps and all command
streams were 50.0 Hz with 65–69 ms p99 gaps. Across the shared tunnel, Spark saw
only 63.1–63.5 Hz state and 29.8–30.0 Hz command, with 142–150 ms command p99 and
up to 0.70 s maximum gaps. Isaac still held RTF 0.100, Spark GPU utilization was
about 4%, and policy inference was normally 0.4–0.6 ms: compute was not the
limit.

`sim_tunnel.sh` now enables SSH compression by default. The repeated DDS samples
compress well: in a 10 s sample the bridge carried 14.25 Mbit/s state and 3.34
Mbit/s command payload internally, while the outer SSH connection carried only
0.79 Mbit/s and 0.24 Mbit/s respectively. The post-pass Spark probe measured
97.5–97.7 Hz state and 49.0–49.4 Hz command; command p99 was 39–46 ms. Set
`SSH_COMPRESSION=no` only for an explicit comparison, not for concurrent balance.

### Eight-robot compressed-tunnel RTF sweep

Keeping the now-default SSH compression enabled removed the earlier shared-link
bottleneck. Eight robots completed the 60.02-second evaluator at every tested
point through RTF 0.20:

| RTF | Result |
|---:|---|
| 0.125 | PASS; all 0.09 m, 1.8–2.2° tilt |
| 0.15 | PASS; all 0.09 m, 2.0–2.1° tilt |
| 0.175 | PASS; 0.01–0.03 m, 1.8–2.3° tilt |
| **0.20** | **PASS; all 0.09 m, 1.8–2.2° tilt, 0.682–0.685 m height** |

The RTF 0.20 qualification used `SIM_SLOWMO=5` and
`CONTROL_WALL_SCALE=0.20` on every controller. Treat it as the current
supervised setting; RTF 0.175 retains more measured timing margin.

### Long-horizon failure attribution

The 2026-07-22 incident is qualitatively different from gradual loss of balance:

- stable duration: 492.3 simulated seconds, approximately 41 wall minutes;
- failure spread: less than 0.1 simulated seconds across all eight robots;
- last delivered state: 0.8–3.9° tilt, repeated for six controller samples;
- next delivered state: 87–89° tilt;
- Isaac: normal 0.198 RTF through the corresponding sim-time interval;
- transport processes: no SSH, bridge, or sim restart.

The synchronized discontinuity implicates the common return-data path. It does
not prove a physical-link outage because a short WARP/SSH, Zenoh, or Spark DDS
stall can leave the SSH process connected. Use paired instances of
`g1_concurrent_diag.py monitor`: domain 0/`lo` on Spark and domain 1/`eth0` on
RTX. A clean RTX stream with a Spark gap localizes the event after the simulator;
matching RTX state/RTF degradation instead implicates Isaac. The monitor logs
five-second snapshots plus threshold crossings and first-fall transitions;
unattended runs should not use the multi-gigabyte full controller CSV.

### Real-robot path is preserved
All new deploy code is off by default:
- `CONTROL_WALL_SCALE` defaults to 1.0 (real-time),
- `PRED_HORIZON_S` / `/tmp/pred_horizon` default 0 (no-op),
- `/tmp/gain_scale` defaults to `1.0 1.0` (no-op).
- `SONIC_IDLE_HOLD_REFERENCE` defaults off in the binary (the RTX sim launch helper
  enables it only when `CONTROL_WALL_SCALE` is not 1.0).

Launch the *same* binary for hardware with no knobs set → identical behavior to before.

---

## 6. Reproduce

```bash
# --- pod (RTX6000) ---
echo 10 > /tmp/sim_slowmo                    # RTF 0.10 (sim re-reads live)

# --- Spark ---
echo "1.0 1.0" > /tmp/gain_scale             # gain scaling off
echo "0.0"     > /tmp/pred_horizon           # prediction off
CONTROL_WALL_SCALE=0.1 ./run_deploy_direct.sh     # also enables measured IDLE + -8° trim
#   then in the deploy: ']' start, ENTER enable planner, '1' standing set

# release the sim base-hold so SONIC balances unaided (pod, DDS domain 1):
#   ~/live-sim/venv/bin/python ~/live-sim/fire_reset.py 0 4   # cat4 = re-arm upright hold
#   ~/live-sim/venv/bin/python ~/live-sim/fire_reset.py 0 3   # cat3 = release after planner warmup
#   (LD_LIBRARY_PATH=~/live-sim/cyclonedds/install/lib, env -u CYCLONEDDS_URI)

# observe tilt (Spark, domain 0):
.venv_sim/bin/python io_capture.py 0 check 20
```

For the **real G1**: run `./run_deploy_direct.sh` with no env overrides
(`CONTROL_WALL_SCALE=1.0`, prediction/gain off) — unchanged interface.

For two concurrent robots, use the namespaced launch and lifecycle commands in
[`RTX_SIM_GUIDE.md`](RTX_SIM_GUIDE.md#4a-concurrent-two-g1s-controlled-by-two-spark-sonic-processes).

---

## 7. To go faster than RTF 0.20 reliably

Only two levers remain, both larger than a code change:

1. **Lower-latency path to the RTX6000** — co-locate the Spark in the pod's region, or a
   direct/peered link. `stable RTF ≈ margin / effective loop delay`, so cutting the
   ~38 ms RTT should raise the ceiling materially. The exact LAN ceiling must be measured;
   it should not be inferred from the remote sweep. This is an infrastructure decision
   (the pod is only WARP-reachable today).

2. **Retrain SONIC with latency/jitter domain-randomization** so the policy is delay-robust
   (wider delay margin). This is the only software route to real-time balance over the
   fixed network, but it requires the training pipeline.

Not worth pursuing: further prediction/gain tuning (ceiling is delay-margin-bound, §4),
or network transport tweaks (38 ms is the WARP floor, not overhead, §2).
