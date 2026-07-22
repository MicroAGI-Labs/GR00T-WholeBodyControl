# RTX6000 pod code (reference copies)

The Isaac sim runs on the **RTX6000 k8s pod**, not on the Spark. Its code lives on the
pod's shared storage at `spark@10.5.7.178:/home/spark/live-sim` (mounted on the Spark via
sshfs at `~/pod-live-sim`) — a **separate checkout**, not part of this repo. These are
**reference copies** of the files we authored or modified there, so the pod-side work is
version-controlled and reviewable.

> ⚠️ **These are copies, not the live files.** Edits made here do NOT reach the pod, and
> pod edits do NOT auto-sync here. After changing either side, copy across (`cp` over the
> `~/pod-live-sim` sshfs mount) and re-commit. The full upstream `unitree_sim_isaaclab`
> checkout (Unitree's repo, multi-GB) is intentionally NOT vendored — only our additions
> and the individual files we patched.

## Our helper scripts → `~/live-sim/<file>`

| File | Role |
|---|---|
| `start_flat.sh` | One-command bring-up of the floating-base balance stack (`Isaac-Flat-G129-Dex3`, `dds_lowcmd29`). Single-instance: `flock` guard + robust clean-slate + post-launch `stack_singleton assert`. |
| `bridge_supervisor.sh` | Keeps the pod `zenoh-bridge-dds` process alive. The bridge may start before or after the Spark bridge/tunnel; Zenoh reconnects peers and this supervisor only respawns a dead local binary. |
| `start_all.sh` | Bring-up of the fixed-base manipulation stack (`PickPlace-Cylinder-…-Joint`). |
| `start_sim.sh` | Low-level sim launcher (pkill + venv + `LD_LIBRARY_PATH` + pidfile + log). |
| `stack_singleton.py` | Robust single-instance control — `kill` / `count` / `assert` by process `comm` (collapses parent+child, never self-matches like `pgrep -f`). |
| `sim_watchdog.sh` | Pod-side liveness watchdog: restart the sim via `start_flat.sh` if `rt/lowstate` dies (never wipes `/dev/shm`). |
| `secondary_imu_adapter.py` | Mirrors `rt/lowstate.imu_state` → `rt/secondary_imu` (the deploy requires it). |
| `gear_sonic_camera_pub.py` | Publishes sim cameras on ZMQ `:5555` in the Orin wire format (keys `ego_view`/`left_wrist`/`right_wrist`). |
| `zenoh-sim-bridge.json5` | Pod zenoh-bridge-dds config (domain 1, listen `:7447`). |
| `fire_reset.py` | Publish one lifecycle command: cat-2/cat-4 re-arm and cat-3 release. Its expanded DDS participant range also works when eight controllers occupy the SDK's default range. |
| `set_idle_reference.py` | Atomically change one running namespaced SONIC controller's stationary-IDLE target: `ROBOT_ID PITCH_DEG LEG_BLEND`. |
| `g1_concurrent_diag.py` | One-participant multi-robot lifecycle, probe, evaluator, and compact long-run monitor. `monitor` records LowState/LowCmd arrival gaps, DDS tick deltas, IMU tilt, and late falls; run it on both Spark (domain 0/`lo`) and RTX (domain 1/`eth0`) to localize transport failures. |
| `g1_dds_diag.py` | Balance-stack DDS diagnostics CLI over `rt/lowstate`/`rt/lowcmd`/`rt/eval`. Subcommands: `watch` (live tilt/knee/\|gyro\| stand check), `eval` (print the in-sim balance eval stream — termination + result score), `capture` (29-joint measured+commanded + IMU → CSV), `probe` (lowstate inter-arrival gaps, diagnoses 'Lost LowState'), `warm` (hold the zenoh↔DDS route warm). `--domain 0` = Spark/deploy side, `1` = pod sim. |
| `unitree_sim_isaaclab/tools/stand_eval.py` | **Deterministic in-sim balance eval** → publishes `rt/eval` (std_msgs/String JSON). `termination` ramps 0→1 over the 60 s qualification, but pose monitoring continues afterward so a late fall replaces success with a latched failure. `result` spans −100…100 (100 = perfect upright at start; −100 = fall). Armed on hold-release and disarmed on cat-4 re-arm; env-tunable (`EVAL_WINDOW_S`, `EVAL_DIST_MAX_M`, `EVAL_FALL_TILT_DEG`, …). |

## Our edits to the vendored `unitree_sim_isaaclab/` → same relative path on the pod

| File | Change |
|---|---|
| `dds/sharedmemorymanager.py` | Self-healing shm: `resource_tracker.unregister` on attach + reopen-on-error + ms timestamps (SIM_RESILIENCE_PLAN WS-C). |
| `tools/shared_memory_utils.py` | Same `resource_tracker` opt-out + drop-and-reattach for the camera reader. |
| `tasks/g1_tasks/flat_g1_29dof_dex3/flat_g1_29dof_dex3_env_cfg.py` | Floating-base preset (`g1_29dof_dex3_wholebody`, reverted from a `base_fix` regression); init z `0.793` (feet-on-ground for the SONIC default stance). |
| `tasks/common_observations/g1_29dof_state.py` | Main `rt/lowstate.imu_state` = PELVIS IMU (`use_torso_imu=False`), matching MuJoCo/training. |
| `dds/g1_robot_dds.py` | IMU quaternion order `[w,x,y,z]` to match the real Unitree LowState. |
| `dds/g1_joint_mapping.py` | Single 29-joint Unitree ordering contract plus name-based Isaac↔Unitree gather/scatter helpers. |
| `sim_main.py` | (a) Hooks for `tools/stand_eval.py`: instantiate `StandEval` before the loop (`EVAL=1`), `arm()` on hold-release (cat-2/cat-3), `disarm()` on re-arm (cat-4), `update()` each control step. (b) File-triggered external-force disturbance for eval testing: `echo "fx fy fz [dur_s]" > $SIM_PUSH_FILE` (default `/tmp/sim_push`) applies a one-shot global wrench on the pelvis. (c) Multi-env reset/hold poses add Isaac's environment origins rather than stacking every robot at world zero, and the WebRTC viewport frames the complete grid. |

> **`rt/eval` must be in BOTH zenoh allow-lists** to reach the Spark: the pod
> `zenoh-sim-bridge.json5` (domain 1) **and** the Spark-side `zenoh-spark-config.json5`
> (domain 0, bind-mounted into the `zenoh-spark-bridge` container). Restart both bridges
> after editing, or just re-run `start_flat.sh` + `docker restart zenoh-spark-bridge`.

See `../SIM_RESILIENCE_PLAN.md`, `../RTX_SIM_GUIDE.md`, and memory
`sim-resilience-implementation` for how these fit together and the balance recipe.

Run the dependency-free joint mapping tests from the repository root:

```bash
python -m unittest discover -s rtx_pod/tests -p 'test_*.py' -v
```
