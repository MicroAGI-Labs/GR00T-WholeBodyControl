# SIM Resilience Plan — continuous, self-recovering Spark ↔ RTX-sim operation

**TL;DR** — Today the SONIC-controller-on-Spark ↔ Isaac-sim-on-RTX6000 loop is brittle:
a WARP hiccup, a bounced sidecar, or a pod-side shm glitch silently stops `rt/lowstate`
and the G1 falls, and nothing comes back without hands-on restarts. This plan makes the
loop **fail-then-auto-recover**: when the link drops, the controller goes to damping; when
it returns, the controller **re-arms and resumes on its own, with no operator input**. We
get there with three self-healing layers — a supervised tunnel (`autossh`), an auto-recovery
state in the deploy, and self-healing shm consumers — while keeping the deploy's real↔sim
interface byte-identical.

Related: [`RTX_SIM_GUIDE.md`](RTX_SIM_GUIDE.md) (bring-up), [`RTX_SIM_LATENCY.md`](RTX_SIM_LATENCY.md)
(the RTF/latency ceiling), [`ELIXIR_SUPERVISOR_GOAL.md`](ELIXIR_SUPERVISOR_GOAL.md) (the
deferred control-plane / RigPilot design).

---

## 1. Goal

**Continuous, reliable operation with automatic recovery from temporary disruptions.**

Concretely, the loop must survive — without a human — a network flap (WARP/tunnel drop),
a sidecar restart, or a transient pod-side shm glitch:

- On loss of the state feed, the controller drops to **damping** (a transient safe state,
  not a terminal stop).
- When the feed returns, the controller **automatically re-arms, exits damping, and resumes**
  the mode it was in — **no keypresses, no relaunch**.
- The transport re-establishes itself; the shm consumers re-attach themselves.

Non-goal clarification (per direction): this is **not** about sim fail-safety for its own
sake — there is no real robot to protect on the sim path. Damping-on-loss exists to make
resumption clean (no snap/jump on re-engage), not to "make the sim safe."

## 2. Constraints (unchanged)

- Controller runs on the **Spark**; sim runs on the **RTX6000** pod.
- The pod is reachable **only** over Cloudflare WARP on **`:22`** (UDP blocked), so DDS
  rides `ssh -L` — TCP-over-TCP. This is fixed and outside our control.
- The deploy's **real↔sim interface stays identical**: the same binary drives the real G1
  and the sim. Every deploy change here must be behaviorally identical on the real robot
  (auto-recovery is a strict improvement there too, not a sim-only fork).

## 3. Deferred (explicitly out of scope for this pass)

- **Control plane / RigPilot** (`ELIXIR_SUPERVISOR_GOAL.md`) — deferred by direction.
- **Removing Docker** from the Spark bridge — Docker is fine, keep it.
- **Removing a Zenoh bridge / CycloneDDS-native TCP** — both bridges stay (both endpoints
  are DDS).
- **Heavy shm hardening** (cross-process mutex, shared-ring redesign) — beyond the minimal
  self-healing + optional seqlock below.

---

## 4. The system today (what breaks, and why it doesn't come back)

Transport chain, sim path:

```
Isaac sim (domain 1, /dev/shm isaac_*)
  → pod: zenoh-bridge-dds  (peer, listen tcp/0.0.0.0:7447)
    → ssh -N -L 7447 -L 5555   (ONE ssh conn, WARP, :22 only, TCP-over-TCP)   ◄── SPOF, no auto-restart
      → Spark: zenoh-spark-bridge (docker, connect tcp/127.0.0.1:7447)
        → DDS on lo
          → deploy  (--disable-crc-check)
```

| Failure | What happens now | Why it doesn't self-recover |
|---|---|---|
| WARP / tunnel drop | `ssh -N` exits; `rt/lowstate` stops | Nothing restarts the tunnel. The Spark bridge connects to **loopback** `127.0.0.1:7447`, which never "fails", so Zenoh's own reconnect never fires. |
| Controller sees stale state | Keeps commanding on stale state → G1 falls | The absent→damping guard (`CheckSafety`, `g1_deploy_onnx_ref.cpp:2894`, `LOW_STATE_ABSENT_THRESHOLD=500 ms`) is **gated `&& !disable_crc_check_`** → **off** on the sim path. |
| Controller absent-guard *does* fire (real path) | Sets `operator_state.stop=true` → `Stop()` tears down threads + damping | **Terminal.** Recovery = relaunch the process by hand. No path back to CONTROL. |
| Sidecar (camera pub / DDS pub) restarts | Segments get unlinked out from under the sim → `cams=NONE` / `lowstate=0` | `resource_tracker` **unlinks on consumer exit** (`shared_memory_utils.py`, `sharedmemorymanager.py`); no `unregister`. |
| Sim recreates a segment | Consumers read a dead handle | Handle is **cached forever**; consumers only handle "never opened", never re-open (`sharedmemorymanager.py:21-33`, `shared_memory_utils.py:205-209`). |
| Any shm read error | `dds_publisher` skips publish silently (`g1_robot_dds.py:75-77`); 1 Hz timestamp | Silent 0-Hz `lowstate`, no signal, no re-attach. |

Key relevant facts already in the code:
- `CreateDampingCommand()` exists (`:2825`): kp=0, kd=8 — the damping pose we'll reuse.
- State machine `ProgramState { INIT, WAIT_FOR_CONTROL, CONTROL }` (`:170`); `Control()` at
  `:3921`. `InitControl()` (`:2847`) already does a **soft ramp from measured pose** to the
  default stand — the mechanism we need for jump-free re-engage.
- Thresholds: `LOW_STATE_LATE_THRESHOLD=50 ms` (`:300`), `LOW_STATE_ABSENT_THRESHOLD=500 ms`
  (`:301`), `STREAMING_DATA_ABSENT_THRESHOLD=150 ms` (`:292`) — currently the "late" signal
  only drives an **audio beep**, not a control action.

---

## 5. Architecture decisions

### D1 — Fail-then-auto-recover, not fail-safe-and-stop
Loss of the state feed puts the controller into a **transient damping state**, not a
terminal `Stop()`. Recovery is automatic: on return of a fresh feed the controller re-arms
and resumes. The realtime threads keep running throughout — we never tear down and rebuild
the process to recover.

### D2 — Detect at the edges, heal at each layer
Each layer owns its own recovery, so a failure anywhere heals locally:
- **Transport** self-heals via `autossh` (supervised tunnel with backoff).
- **Controller** self-heals via a new auto-recovery state driven by a state-feed watchdog.
- **shm consumers** self-heal in code (re-attach), instead of a restart-the-stack ritual.

### D3 — A pod-side shm glitch looks like a network flap, and that's fine
Both surface to the Spark as "`rt/lowstate` stopped." So the **same** controller
auto-recovery handles both — provided the feed actually comes back. `autossh` guarantees
that for the tunnel; shm self-healing (D5) guarantees it for the sidecars.

### D4 — Damping is a state, re-engage is jump-free
On resume, the first engaged command must be continuous with the robot's **measured** pose
(reuse the `InitControl` soft-ramp), and the controller must **resume the mode it was in**
(planner vs pose). No snap-to-target on re-engage.

### D5 — Fix the shm consumers in code; don't restart the stack
Make the consumers self-healing rather than relying on `start_all`:
1. **Never unlink a segment you didn't create** — `resource_tracker.unregister(...)` at every
   open-by-name site. Makes segments long-lived; makes bouncing a sidecar safe.
2. **Re-attach on staleness/error** — close and re-open the handle by name when reads fail or
   the timestamp is stale past a threshold. Consumers survive a sim restart on their own.
3. **Surface staleness** — millisecond timestamps (cameras already have them; bump the state
   path off its 1 s resolution) so #2 has a real signal and "frozen" isn't masked as "live".
4. *(optional)* **Seqlock** — write a counter before+after the payload; reader retries on
   mismatch — closes the torn-read window.

The only failure that still needs a restart is the **sim process itself** dying (it owns the
physics). Even then, the sidecars auto-recover — so it's "restart the sim", not "restart the
whole stack". Automating that is the optional pod-side watchdog (D6). `rm /dev/shm/*` will
always break it — that's "don't do that", not a resilience gap.

### D6 — Optional pod-side sim watchdog (the pod analog of autossh)
A ~15-line loop on the pod: if `rt/lowstate` Hz == 0 for N s, `pkill -9` the sim stack and
re-run `start_all.sh` (never `rm /dev/shm`). Closes the one gap D5 can't (the sim dying)
without reintroducing the restart ritual for everything else.

### D7 — Vendored pod edits are acceptable
D5 lands in the pod's local `unitree_sim_isaaclab` checkout (vendored), same as the existing
quaternion-order patch. Keep the diffs minimal and documented so they survive a pod refresh.

### D8 — Interface parity is a hard invariant
All deploy changes are behavior-preserving on the real robot: auto-recovery replaces the
current terminal-stop with a strictly better resume, and uses the existing damping + soft-ramp
primitives. No sim-only code fork in the deploy.

---

## 6. Target flow (control feed drops, then returns)

```
CONTROL  ──(lowstate absent > T_absent)──►  RECOVER_DAMPING
   ▲                                            │  emit CreateDampingCommand() every tick
   │                                            │  keep threads alive; watch for fresh lowstate
   │                                            ▼
   └──(mode restored, ramp done)◄── RE_ARM ◄──(lowstate fresh & advancing for T_stable)
                                     soft-ramp from measured pose (InitControl-style)
```

- `T_absent`: state-feed age beyond which we damp (tunable; independent of the old real-robot
  500 ms terminal guard).
- `T_stable`: how long the feed must be fresh **and advancing** (tick/seq monotonic, not just
  present) before re-arming — debounced so a flapping link doesn't oscillate.
- `RE_ARM` restores planner/pose mode and soft-ramps → no jump. Then back to `CONTROL`.

---

## 7. TODO

### Workstream A — Transport (autossh)  ✅ approved
- [ ] Replace the bare `ssh -N -o ExitOnForwardFailure=yes -L 7447 -L 5555` with **`autossh`**
      (monitored, auto-reconnect with backoff), forwarding both `:7447` and `:5555` on one conn.
- [ ] Single source of truth for the pod endpoint (`~/.ssh/config` Host alias or one env file)
      so a pod-IP change after restart is a one-line edit, not multi-place.
- [ ] Tune Zenoh session keepalive/timeout low so a stale session re-scouts quickly once the
      byte pipe heals underneath.
- [ ] Verify recovery: kill the tunnel mid-run → confirm it re-establishes and `rt/lowstate`
      resumes with no manual action.

### Workstream B — Controller auto-recovery (deploy)  ✅ approved
- [ ] Add a **state-feed watchdog** usable in sim mode (decouple absent-detection from
      `disable_crc_check_`); detect both *absent* (age > `T_absent`) and *frozen* (tick/seq
      not advancing).
- [ ] Add a transient **RECOVER_DAMPING** state: emit `CreateDampingCommand()` each tick,
      keep realtime threads alive, **do not** set `operator_state.stop`.
- [ ] Add **RE_ARM**: on feed fresh+advancing for `T_stable`, soft-ramp from measured pose
      (reuse `InitControl` mechanism) and restore prior mode (planner/pose) → back to CONTROL,
      no operator input.
- [ ] Make `T_absent` / `T_stable` tunable (env or `/tmp` knob, consistent with existing
      `PRED_HORIZON_S` / `/tmp/*` pattern); defaults preserve real-robot behavior.
- [ ] Confirm parity: on the real robot the new path is a strict improvement (auto-resume
      instead of terminal stop) with identical steady-state behavior.
- [ ] Rebuild `target/release/g1_deploy_onnx_ref`; validate loss→damping→auto-resume against
      the sim (kill tunnel, restart sidecar) with no keypresses.

### Workstream C — Self-healing shm consumers (pod, vendored)  ✅ approved
- [ ] Identify the exact create/open sites and owners for `isaac_robot_state`, `dds_robot_cmd`,
      and `isaac_{head,left,right}_image_shm` (who creates, who opens).
- [ ] Add `resource_tracker.unregister(...)` at every **open-by-name** site in
      `dds/sharedmemorymanager.py` and `tools/shared_memory_utils.py` (readers must never
      unlink segments they didn't create).
- [ ] Add **re-attach on staleness/error**: close + re-open the handle by name when reads
      fail or the timestamp is stale past a threshold.
- [ ] Move the state-path timestamp to **milliseconds** (`sharedmemorymanager.py:54`) so
      staleness is detectable sub-second (cameras already ms).
- [ ] *(optional)* Add a **seqlock** guard (counter before+after payload; reader retries on
      mismatch) to close the torn-read window.
- [ ] Document the vendored diff (survive a pod refresh); verify: bounce the camera pub alone
      and the DDS publisher alone → both re-attach, `lowstate`/cameras resume, **no** stack
      restart, **no** `rm /dev/shm`.

### Workstream D — Pod-side sim watchdog  ⭘ optional
- [ ] Add a small pod loop: if `rt/lowstate` Hz == 0 for N s, `pkill -9` sim stack +
      re-run `start_all.sh` (never `rm /dev/shm`). Closes the "sim process died" gap.

### Validation (end-to-end)
- [ ] **Network flap:** drop WARP/tunnel for ~10 s mid-stand → deploy damps → tunnel
      re-establishes → deploy auto-resumes upright, no input.
- [ ] **Sidecar bounce:** restart camera pub, then DDS publisher → feeds re-attach, deploy
      rides through (damps only if `lowstate` actually gapped), no stack restart.
- [ ] **Sim restart (with D6):** kill the sim → watchdog relaunches → sidecars re-attach →
      deploy auto-resumes.
- [ ] **Real-robot parity smoke test:** confirm steady-state behavior unchanged and that a
      transient feed gap now auto-resumes instead of terminal-stopping.

---

## 8. Status

**Implemented & verified 2026-07-10; hardened 2026-07-11.** All workstreams done: A (autossh, `sim_tunnel.sh` +
`rtx-pod` ssh alias), B (deploy auto-recovery — `RECOVER_DAMPING` state, **1 s** trigger,
auto re-arm), C (self-healing shm — `resource_tracker` opt-out + reopen, proven), D (pod
watchdog `sim_watchdog.sh` — detection verified), plus **G** (single-instance guard
`stack_singleton.py` + `flock` in `start_flat.sh`, added after a duplicate-publisher scare).

The pod bridge is also supervised by `bridge_supervisor.sh`. `start_flat.sh` uses
PID-file-based bridge shutdown rather than `pkill -f` (which can self-match the
bring-up shell), so a pod bridge restart, Spark bridge restart, or tunnel restart
may occur in any order: Zenoh reconnects the peer and the supervisor respawns a
dead pod bridge.

Forced-disruption tests all auto-recovered with **zero operator input**: tunnel flap
(autossh healed → deploy damped → resumed), camera-pub bounce (shm re-attached, `cams=[…]`
not `NONE`), and full sim restart (deploy damped → auto-resumed).

**Balancing achieved:** SONIC balances the G1 unaided in Isaac for **60 s** at RTF 0.10
(final tilt ≈2.5°, no fall). Working recipe: RIGID hold + `SIM_WARMUP_JOINTS=1` + init z
0.793 (clean default stance), **fresh** deploy, `sim_slowmo=10` with matched
`CONTROL_WALL_SCALE=0.1`, then cat-4 re-arm and cat-3 release. The whole-body flat task
must accept cat-3/cat-4 even with `--enable_wholebody_dds`; do not gate those handlers to
the non-whole-body path. See memory `sim-resilience-implementation` and
`RTX_SIM_LATENCY.md`.

Deferred (§3): control plane, Docker removal, bridge simplification. The D watchdog's
auto-restart path should get flock/backoff refinement before being relied on.
