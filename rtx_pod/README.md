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
| `start_all.sh` | Bring-up of the fixed-base manipulation stack (`PickPlace-Cylinder-…-Joint`). |
| `start_sim.sh` | Low-level sim launcher (pkill + venv + `LD_LIBRARY_PATH` + pidfile + log). |
| `stack_singleton.py` | Robust single-instance control — `kill` / `count` / `assert` by process `comm` (collapses parent+child, never self-matches like `pgrep -f`). |
| `sim_watchdog.sh` | Pod-side liveness watchdog: restart the sim via `start_flat.sh` if `rt/lowstate` dies (never wipes `/dev/shm`). |
| `secondary_imu_adapter.py` | Mirrors `rt/lowstate.imu_state` → `rt/secondary_imu` (the deploy requires it). |
| `gear_sonic_camera_pub.py` | Publishes sim cameras on ZMQ `:5555` in the Orin wire format (keys `ego_view`/`left_wrist`/`right_wrist`). |
| `zenoh-sim-bridge.json5` | Pod zenoh-bridge-dds config (domain 1, listen `:7447`). |
| `fire_reset.py` | Publish `rt/reset_pose/cmd` (domain 1): cat-2 teleport-upright+release, cat-3 release-in-place, cat-4 re-arm hold. |

## Our edits to the vendored `unitree_sim_isaaclab/` → same relative path on the pod

| File | Change |
|---|---|
| `dds/sharedmemorymanager.py` | Self-healing shm: `resource_tracker.unregister` on attach + reopen-on-error + ms timestamps (SIM_RESILIENCE_PLAN WS-C). |
| `tools/shared_memory_utils.py` | Same `resource_tracker` opt-out + drop-and-reattach for the camera reader. |
| `tasks/g1_tasks/flat_g1_29dof_dex3/flat_g1_29dof_dex3_env_cfg.py` | Floating-base preset (`g1_29dof_dex3_wholebody`, reverted from a `base_fix` regression); init z `0.793` (feet-on-ground for the SONIC default stance). |
| `tasks/common_observations/g1_29dof_state.py` | Main `rt/lowstate.imu_state` = PELVIS IMU (`use_torso_imu=False`), matching MuJoCo/training. |
| `dds/g1_robot_dds.py` | IMU quaternion order `[w,x,y,z]` to match the real Unitree LowState. |

See `../SIM_RESILIENCE_PLAN.md`, `../RTX_SIM_GUIDE.md`, and memory
`sim-resilience-implementation` for how these fit together and the balance recipe.
