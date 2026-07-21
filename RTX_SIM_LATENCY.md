# RTX6000 Sim Latency — why SONIC balance is RTF-capped, and how far we pushed it

**TL;DR** — Running the SONIC low-level controller on the Spark against the Isaac
sim on the remote RTX6000 pod, the G1 balances **only when the sim runs slower than
real time**. The cause is the **fixed ~38 ms network round-trip** between Spark and
pod, against SONIC's **~18–20 ms delay margin**. We measured the stable ceiling at
**RTF ≈ 0.333** (1.67× faster than the previous RTF 0.2 operating point) and confirmed
a stable 30 s+ unaided stand there. Going faster needs either a lower-latency network
path (infra) or a delay-robust controller (retrain SONIC). First-order state prediction
and gain-softening were implemented and tested — neither moves the ceiling.

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

Measured cliff (fresh, clean DDS, CONTROL_WALL_SCALE matched to RTF):

| RTF (= 1 / sim_slowmo) | sim-time loop delay | Result |
|---:|---:|---|
| 0.20 | ~8 ms + async | ✅ stable |
| **0.333** | ~13 ms + async | ✅ **stable (30 s+, tilt → sub-1°)** |
| 0.40 | ~16 ms + async | ❌ falls hard (>40° within ~1 s) |
| 0.50 | ~20 ms + async | ❌ falls ~2 s |
| 1.00 | ~40 ms + async | ❌ falls ~1 s |

**Is 38 ms a physics limit? No.** A standing G1 is an inverted pendulum pivoting at the
ankles: CoM height L ≈ 0.6 m → unstable pole ωᵤ = √(g/L) ≈ 4 rad/s → the fundamental
delay bound for stabilizability is ωᵤ·T ≲ 1, i.e. **~250 ms**. At 38 ms, ωᵤ·T ≈ 0.15 —
seven times inside the physics budget.

The real limiter is the **controller**: SONIC is a stiff, high-bandwidth position-PD
policy (kp up to 99) that was effectively trained at zero latency. Its **delay margin is
only ~18–20 ms of sim-time** (from the cliff above). Delay eats phase margin as ω_c·T,
and at SONIC's bandwidth even ~20 ms is enough to destabilize the marginal stand — long
before the pendulum's own 250 ms limit matters.

So:

```
stable RTF  ≈  SONIC delay margin (~18–20 ms)  /  wall round-trip (~38–55 ms)  ≈  0.33
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

**RTF 0.10 is the verified operating point with the current autossh tunnel:**
- Deploy: `CONTROL_WALL_SCALE=0.1` (must equal sim RTF).
- Pod sim: `/tmp/sim_slowmo=10` (RTF = 1 / slowmo).
- Stationary IDLE: `SONIC_IDLE_HOLD_REFERENCE=1`,
  `SONIC_IDLE_PITCH_BIAS_DEG=-8`, and `SIM_WARMUP_POSE=sonic`.
- Confirmed: 60.02 s unaided stand; final displacement 0.10 m, tilt 0.7°, no fall.
- Revalidated with synchronized reference/state/command telemetry: 60.02 s unaided
  stand, final displacement 0.03 m, height 0.707 m, tilt 1.3°, result +71.56.

This keeps the G1 upright under SONIC control on the Spark despite the current tunnel
latency.

For the two-robot bring-up, both controllers still used
`CONTROL_WALL_SCALE=0.1`, but the warmup and frozen IDLE reference were matched
to the observed posture: `SIM_WARMUP_POSE=observed`,
`SONIC_IDLE_PITCH_BIAS_DEG=0`, and `SONIC_IDLE_LEG_BLEND=0`. Both concurrent
evaluators reached 60.02 simulated seconds successfully; this is now the
recommended two-robot IDLE configuration.

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

## 7. To go faster than RTF 0.333

Only two levers remain, both larger than a code change:

1. **Lower-latency path to the RTX6000** — co-locate the Spark in the pod's region, or a
   direct/peered link. `stable RTF ≈ margin / RTT`, so cutting the ~38 ms RTT scales the
   ceiling linearly; at LAN latency (<1 ms) SONIC would balance well past RTF 1.0. This is
   an infrastructure decision (the pod is only WARP-reachable today).

2. **Retrain SONIC with latency/jitter domain-randomization** so the policy is delay-robust
   (wider delay margin). This is the only software route to real-time balance over the
   fixed network, but it requires the training pipeline.

Not worth pursuing: further prediction/gain tuning (ceiling is delay-margin-bound, §4),
or network transport tweaks (38 ms is the WARP floor, not overhead, §2).
