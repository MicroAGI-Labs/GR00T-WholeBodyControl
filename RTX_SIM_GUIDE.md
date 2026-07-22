# Running the G1 in Isaac Sim as a Real-Robot Stand-In (RTX box + DGX Spark)

This guide sets up a **hardware-in-the-loop Isaac Sim testbench** for the Unitree
G1: one or more G1s simulated in Isaac Sim on an RTX PRO 6000 box, driven by the
**same SONIC binary and Unitree DDS message types** used for the real robot.
Simulation-only routing and timing are selected with environment variables;
empty topic prefixes and default timing preserve the physical-robot path.

The core design principle:

> **The sim impersonates the real robot on both of its interfaces** — the Unitree
> **DDS** control bus (`rt/lowstate` / `rt/lowcmd`) and the **camera server**
> (gear_sonic ZMQ `:5555`). A concurrent simulation uses namespaced DDS topics
> such as `rt/sim/g1/0/lowstate`; the unprefixed hardware topics remain the
> default. **Do not maintain a separate simulation controller binary.**

```
                 DGX Spark (GB10)                       RTX PRO 6000 pod (k8s)
   ┌───────────────────────────────────┐        ┌──────────────────────────────────┐
   │ GR00T N1.7 policy server (:5550)   │        │ Isaac Sim G1 (CPU physics)         │
   │ SONIC deploy(s) ─ lowcmd ────────┐ │  DDS   │  ├─ DDS domain 1 (lowstate…)       │
   │ VLA client (run_vla_inference)   │ │◄─Zenoh─┤  ├─ shared-mem cameras             │
   │   --camera-host 127.0.0.1 ───────┘ │  :7447 │ zenoh-bridge-dds (:7447)           │
   │                                     │  :5555 │ in-process per-robot DDS/IMU (N>1)  │
   │  (ssh -L tunnels for :7447, :5555)  │◄───────┤ gear_sonic_camera_pub (:5555)      │
   └───────────────────────────────────┘        │  GPU: rendering only                │
                                                  └──────────────────────────────────┘
```

The pick/place stack can run near real time, but remote SONIC balance is
latency-limited. With the compressed SSH tunnel, **eight concurrent robots have
completed 60.02 simulated seconds through RTF 0.20**. Use RTF 0.20 for supervised
throughput runs and 0.175 when more transport margin matters. A later RTF 0.20
run stayed upright for 492.3 simulated seconds before all eight failed together
during a shared state-delivery discontinuity; Isaac held RTF 0.198 throughout,
so that incident is not evidence of simulator slowdown. At RTF 0.225, two robots
fell near 29 seconds. See [`RTX_SIM_LATENCY.md`](RTX_SIM_LATENCY.md) and **§7
Gotchas**.

---

## 0. Hosts & access

| Host | Role | Access |
|------|------|--------|
| **RTX box** | Isaac Sim physics + rendering | `ssh spark@<SERVICE_IP>` (graphics-capable k8s pod) |
| **DGX Spark** | SONIC deploy + GR00T VLA + policy server | local (`~/repos/GR00T-WholeBodyControl`, `~/repos/Isaac-GR00T`) |

The RTX pod's `$HOME` (`~/live-sim`) is on **shared network storage** (`/research`),
so the installed environment survives pod restarts and follows to a new pod.

> **Two IPs, and they change on every pod restart.** The examples below use
> `<SERVICE_IP>` and `<POD_IP>` — fill in the current values after each restart:
>
> | Name | What it is | How to get it | Reachable ports |
> |------|-----------|---------------|-----------------|
> | `<SERVICE_IP>` | per-instance k8s **ClusterIP Service** (e.g. `10.5.x.x`) | shown on the instance's page | **`:22` only** (SSH). Stable-ish name for SSH across restarts. |
> | `<POD_IP>` | the **pod's own IP** on `10.4.0.0/16` (e.g. `10.4.8.23`) | `hostname -i` **on the pod** | **all ports, TCP + UDP** — from a WARP-enrolled machine |
>
> **Access is over Cloudflare WARP**, which advertises the `10.4.0.0/16` pod CIDR into
> the split-tunnel. So the pod IP is directly reachable (no NAT, no port-forward) from a
> WARP laptop — this is how the **WebRTC viewport** connects (§4a-bis). **Caveat, measured:**
> the **DGX Spark** only reaches the pod on `:22` (its WARP profile/Gateway policy blocks
> the rest), so the **Spark→sim data path still needs SSH tunnels** for `:7447`/`:5555`
> (§4a). Don't advertise `<SERVICE_IP>` as a WebRTC/ICE candidate — it only exposes `:22`
> and signaling will fail.
>
> After a restart, re-run the one-shot bring-up on the pod: **`~/live-sim/start_all.sh`**
> (sim + zenoh bridge + IMU adapter + camera pub, in order, idempotent — see §3).

> **The pod MUST be graphics-capable** (Vulkan working), not compute-only. A
> compute-only GPU pod fails at `vkCreateInstance` before ever touching the GPU and
> Isaac Sim silently falls back to CPU `llvmpipe`. Verify with `vulkaninfo --summary`
> showing the RTX PRO 6000 (not `llvmpipe`). This is a **pod-spec / `NVIDIA_DRIVER_CAPABILITIES`**
> issue — must include `graphics` — not fixable inside the pod.

---

## 1. One-time install on the RTX box (uv-based, no conda)

`$HOME=~` is shared storage, so this is done once. Everything lives under `~/live-sim`.

### 1a. System packages (pod is a minimized image)

```bash
ssh spark@10.5.7.178
sudo apt-get update
sudo apt-get install -y git curl tmux iproute2 \
    libxt6 libvulkan1 vulkan-tools libglu1-mesa libegl1 libgles2 libopengl0 libglx0 libgl1
# Make the injected NVIDIA driver libs resolvable (needed for Vulkan):
echo -e "/usr/local/nvidia/lib\n/usr/local/nvidia/lib64" | sudo tee /etc/ld.so.conf.d/zz-nvidia-injected.conf
sudo ldconfig
vulkaninfo --summary | grep deviceName   # must show "NVIDIA RTX PRO 6000", NOT llvmpipe
```

### 1b. uv + repos + environment

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
mkdir -p ~/live-sim && cd ~/live-sim
git clone --depth 1 https://github.com/unitreerobotics/unitree_sim_isaaclab.git
git clone https://github.com/isaac-sim/IsaacLab.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
git clone https://github.com/unitreerobotics/unitree_sdk2_python
```

Build CycloneDDS (C++):

```bash
cd ~/live-sim/cyclonedds && mkdir -p build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install && cmake --build . --target install
export CYCLONEDDS_HOME=~/live-sim/cyclonedds/install
```

Create the venv and install Isaac Sim 5.1 + Isaac Lab (Python 3.11, Blackwell CUDA 12.8):

```bash
cd ~/live-sim
uv venv --python 3.11 --seed venv
source venv/bin/activate
uv pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu128
uv pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
cd ~/live-sim/IsaacLab && ./isaaclab.sh --install
cd ~/live-sim/unitree_sdk2_python && uv pip install -e .
cd ~/live-sim/unitree_sim_isaaclab
uv pip install -r requirements.txt
cd teleimager && uv pip install -e . && cd ..
bash fetch_assets.sh          # downloads the Unitree USD scene/robot assets
```

> Isaac Sim's first `isaacsim` import prompts to accept the Omniverse EULA — answer
> `Yes`. Shader cache (`~/.cache/ov`, ~1.6 GB) persists on shared storage, so first-
> render compilation is a one-time cost.

### 1c. Real-time physics config

Edit the **Joint** task config
`tasks/g1_tasks/pick_place_cylinder_g1_29dof_dex3/pickplace_cylinder_g1_29dof_dex3_joint_env_cfg.py`,
in `__post_init__`:

```python
self.decimation = 2
self.sim.dt = 0.008      # 125 Hz physics → ~0.95x real-time on CPU physics
```

`dt` is the real-time knob (RTF scales ~linearly with it): `0.005→0.59x`,
`0.008→0.95x`, `0.006→~0.75x`. Larger `dt` = coarser contact fidelity; validate
grasping and drop toward `0.006` if grasps jitter.

---

## 2. Helper scripts on the RTX box (`~/live-sim/`)

Create these three once. They keep the stack reproducible and connection-drop-safe.

### `start_sim.sh` — reliable sim launcher (PID file, unbuffered log)

```bash
#!/bin/bash
cd ~/live-sim/unitree_sim_isaaclab
source ~/live-sim/venv/bin/activate
export LD_LIBRARY_PATH=~/live-sim/cyclonedds/install/lib:$LD_LIBRARY_PATH
pkill -9 -f "sim_main.py --device" 2>/dev/null; sleep 2
nohup python -u sim_main.py "$@" > ~/live-sim/sim_run.log 2>&1 &
echo $! > ~/live-sim/sim.pid
sleep 3
kill -0 $(cat ~/live-sim/sim.pid) && echo "STARTED pid=$(cat ~/live-sim/sim.pid)" || { echo FAILED; tail -5 ~/live-sim/sim_run.log; }
```

### `zenoh-sim-bridge.json5` — Zenoh↔DDS bridge config

```json5
{
  mode: "peer",
  listen: { endpoints: ["tcp/0.0.0.0:7447"] },
  scouting: { multicast: { enabled: false } },
  plugins: { dds: { domain: 1, allow: ["rt/lowstate", "rt/lowcmd", "rt/secondary_imu"] } }
}
```

Download the matching bridge binary (**must equal the Spark container version, v1.9.0**):

```bash
cd ~/live-sim
curl -sL -o zenoh-dds.zip https://github.com/eclipse-zenoh/zenoh-plugin-dds/releases/download/1.9.0/zenoh-plugin-dds-1.9.0-x86_64-unknown-linux-gnu-standalone.zip
unzip -o zenoh-dds.zip     # -> ./zenoh-bridge-dds
```

### `secondary_imu_adapter.py` — republish LowState IMU as `rt/secondary_imu`

The sim only fills `LowState.imu_state`; the SONIC deploy also requires
`rt/secondary_imu` (torso IMU) or it aborts. This mirrors the pelvis IMU.

```python
import time
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, IMUState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__IMUState_

ChannelFactoryInitialize(1)                       # sim is on DDS domain 1
pub = ChannelPublisher("rt/secondary_imu", IMUState_); pub.Init()

def cb(msg):
    imu = unitree_hg_msg_dds__IMUState_()
    s = msg.imu_state
    imu.quaternion = s.quaternion; imu.gyroscope = s.gyroscope
    imu.accelerometer = s.accelerometer; imu.rpy = s.rpy; imu.temperature = s.temperature
    pub.Write(imu)

sub = ChannelSubscriber("rt/lowstate", LowState_); sub.Init(cb, 10)
print("[secondary_imu_adapter] running", flush=True)
while True: time.sleep(5)
```

### `gear_sonic_camera_pub.py` — publish sim cameras in gear_sonic `:5555` format

Taps the sim's shared memory (`tools.shared_memory_utils.MultiImageReader`,
keys `head`/`left`/`right`) and republishes on ZMQ PUB `:5555` in the **exact**
gear_sonic/Orin wire format (`msgpack.packb({"timestamps":…, "images":{k: jpeg}}, use_bin_type=True)`),
mapping **`head→ego_view`**, `left→left_wrist`, `right→right_wrist`. Decoupled from the
physics loop (reads shared memory at its own ~30 fps, zero per-step cost). Full
script: **`sim_gear_sonic_camera_pub.py`** in this repo (also reproduced in the
Appendix) — copy it to `~/live-sim/gear_sonic_camera_pub.py` on the RTX box.

> **The head camera MUST be keyed `ego_view`, not `head`** (`KEY_MAP` in the script).
> The VLA client reads `camera_msg["images"]["ego_view"]` directly (see
> `prepare_observation_from_sensors` in `run_vla_inference.py`) — `ComposedCameraClientSensor`
> does **not** remap keys, so a `head` key raises `KeyError: 'ego_view'` and inference
> never produces an observation. The client emits video keys `ego_view` / `left_wrist` /
> `wrist_view` (the last from `right_wrist`). This is the Orin's real wire format —
> parity, not a sim choice.

---

## 3. Bring-up on the RTX box (tmux session `sim`)

> **Shortcut:** all four steps below are automated by **`~/live-sim/start_all.sh`** — it
> launches the sim (with livestream), waits until it is *stepping*, then starts the zenoh
> bridge, IMU adapter, and camera pub, and prints a status table + listening ports. Use it
> after every restart; the manual steps below document what it does and the exact flags.
>
> **Do not restart the camera pub by itself while the sim keeps running** — see §7 gotcha 11.

```bash
ssh spark@<SERVICE_IP>
tmux new-session -d -s sim

# 1. Sim (real-time: CPU physics, headless offscreen cameras at ~10 Hz)
#    --livestream_type 2 --public_ip $(hostname -i) enables the WebRTC 3rd-person
#    viewport (see §4a-bis). Omit them if you don't need to watch it.
tmux new-window -t sim -n main
tmux send-keys -t sim:main '~/live-sim/start_sim.sh --device cpu --headless --enable_cameras \
   --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint --robot_type g129 --enable_dex3_dds \
   --render_interval 12 --livestream_type 2 --public_ip $(hostname -i)' Enter

# 2. Zenoh bridge  (env -u CYCLONEDDS_URI so it uses default DDS matching the sim)
tmux new-window -t sim -n bridge
tmux send-keys -t sim:bridge 'env -u CYCLONEDDS_URI UHLC_MAX_DELTA_MS=2000 \
   ~/live-sim/zenoh-bridge-dds --config ~/live-sim/zenoh-sim-bridge.json5' Enter

# 3. Secondary-IMU adapter
tmux new-window -t sim -n imu
tmux send-keys -t sim:imu 'source ~/live-sim/venv/bin/activate && \
   LD_LIBRARY_PATH=~/live-sim/cyclonedds/install/lib python ~/live-sim/secondary_imu_adapter.py' Enter

# 4. Camera publisher  (needs repo root on PYTHONPATH for `tools`)
tmux new-window -t sim -n campub
tmux send-keys -t sim:campub 'source ~/live-sim/venv/bin/activate && \
   PYTHONPATH=~/live-sim/unitree_sim_isaaclab python -u ~/live-sim/gear_sonic_camera_pub.py --port 5555 --fps 30' Enter
```

Verify (from the RTX box, sim venv):

```bash
# lowstate flowing on domain 1
LD_LIBRARY_PATH=~/live-sim/cyclonedds/install/lib python -c "
from unitree_sdk2py.core.channel import *; from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
import time; ChannelFactoryInitialize(1); n=[0]
s=ChannelSubscriber('rt/lowstate',LowState_); s.Init(lambda m:n.__setitem__(0,n[0]+1),10); time.sleep(3); print('lowstate Hz≈', n[0]/3)"
# camera publisher log shows: published … cams=['ego_view','left_wrist','right_wrist']
tail -3 ~/live-sim/campub.log
```

---

## 4. Bring-up on the Spark

### 4a. Tunnels (Spark reaches the pod only on :22; forward DDS + camera)

The Spark's WARP profile only lets it reach the pod on `:22`, so tunnel DDS + camera
over SSH (one connection, both forwards). **Use the supervised `autossh` tunnel** so a WARP
hiccup or pod flap self-heals instead of silently killing `rt/lowstate` (a bare `ssh -N`
does not come back — see [`SIM_RESILIENCE_PLAN.md`](SIM_RESILIENCE_PLAN.md)):

```bash
./sim_tunnel.sh            # autossh -M0, forwards :7447 + :5555, auto-reconnects
# endpoint = the `rtx-pod` Host alias in ~/.ssh/config (edit its HostName once per pod restart)
# verify: nc -z 127.0.0.1 7447 && nc -z 127.0.0.1 5555
```

`localhost` in the forward target is the pod's own loopback, where the zenoh bridge (`:7447`)
and camera pub (`:5555`) both listen on `0.0.0.0`. (Plain fallback:
`ssh -N -o ExitOnForwardFailure=yes -L 7447:localhost:7447 -L 5555:localhost:5555 rtx-pod` —
but it won't auto-reconnect.)

> **Resilience & auto-recovery:** the deploy now damps on LowState loss and auto-resumes when
> it returns (`AUTO_RECOVER`, 1 s trigger); the pod shm consumers self-heal a sidecar bounce;
> and `start_flat.sh` is single-instance (flock + assert). Full design + the SONIC balance
> recipe (RIGID warmup + `SIM_WARMUP_JOINTS=1` + init z 0.793, RTF 0.10–0.125, cat-3 release):
> `SIM_RESILIENCE_PLAN.md`.

### 4a-flat. Flat-world SONIC balance (verified RTF 0.10)

Use `start_flat.sh` for the free-base, flat-world task. It intentionally uses GPU physics
(`--device cuda`); the CPU-physics recommendation in §6 applies to the single-env
pick/place task, not this balance recipe.

```bash
# RTX pod: start from a clean flat stack and hold SONIC's policy-native stance.
echo 10 > /tmp/sim_slowmo
SIM_SLOWMO=10 SIM_BASE_HOLD_S=9999 SIM_BASE_SOFT=0 SIM_WARMUP_JOINTS=1 \
  SIM_WARMUP_POSE=sonic \
  bash ~/live-sim/start_flat.sh

# Spark: launch a fresh sim deploy with matching timing and stationary-IDLE trim.
ZENOH_JETSON_ENDPOINT=tcp/127.0.0.1:7447 UHLC_MAX_DELTA_MS=2000 \
  CONTROL_WALL_SCALE=0.1 SONIC_IDLE_HOLD_REFERENCE=1 \
  SONIC_IDLE_PITCH_BIAS_DEG=-8 DEPLOY_YES=1 \
  ./deploy.sh --input-type zmq_manager --output-type all g1zenoh
```

Start the controller, send cat-4 to teleport/re-arm the upright hold, allow the planner to
initialize, then send cat-3 to release. In `sim_main.py`, cat-3 and cat-4 **must not** be
gated by `not args_cli.enable_wholebody_dds`; otherwise this workflow is silently ignored by
the flat task.

The stationary-IDLE reference is captured once when the IDLE clip is accepted: all 29
measured joint positions are repeated with zero target velocity, and the measured pelvis
orientation receives a -8 degree pitch trim. The trim cancels the forward-biased recovery
cycle without changing the policy or its measured-state inputs. It is sim-opt-in; real-robot
launches retain the original planner reference unless these variables are explicitly set.

To record what the policy is actually tracking, set `IDLE_TELEMETRY_LOGFILE` on the
deploy. Each 50 Hz control row contains the reference pose, the exact `LowState` snapshot
used for inference, and the resulting motor command in canonical Unitree hardware order:

```bash
IDLE_TELEMETRY_LOGFILE=/tmp/sonic_idle.csv \
  CONTROL_WALL_SCALE=0.1 ./run_deploy_direct.sh
python3 rtx_pod/analyze_idle_telemetry.py /tmp/sonic_idle.csv --discard 12
```

`run_deploy_direct.sh` and `run_deploy_clean.sh` select the interface on the active IPv4
default route; set `DDS_INTERFACE=<name>` to override it. On the pod,
`g1_dds_diag.py capture` also records the latest absolute root position/quaternion from
`rt/eval`, alongside measured and commanded joints.

### 4a-concurrent. Multiple G1s controlled by Spark SONIC processes

The concurrent MVP keeps one Isaac process, one bridge per host, and one SSH
tunnel. Isolation comes from namespaced topics, not extra DDS domains or
sidecars. The tested two-robot setup is documented in full in
[`CONCURRENT_SONIC_CONTROL_TO_REMOTE_RTX.md`](CONCURRENT_SONIC_CONTROL_TO_REMOTE_RTX.md).

Start two environments on the RTX pod:

```bash
SIM_ROBOT_COUNT=2 SIM_SLOWMO=10 SIM_BASE_HOLD_S=9999 \
  SIM_BASE_SOFT=0 SIM_WARMUP_JOINTS=1 SIM_WARMUP_POSE=observed \
  bash ~/live-sim/start_flat.sh
```

This launches exactly one Isaac process, one RTX Zenoh bridge, one camera
publisher, and no secondary-IMU adapter. The in-process multi-robot DDS endpoint
publishes each robot's LowState and secondary IMU directly. Concurrent Dex3 is
disabled for this body-control MVP; `start_flat.sh` retains `--enable_dex3_dds`
only for the legacy single-robot path until hand state and commands are routed
per environment.

The Spark bridge is still created by the normal `g1zenoh` deployment. It uses
DDS domain 0 on `lo`, explicit localhost discovery, and
`forward_discovery: false`. Once the `zenoh-spark-bridge` container exists, it
can be restarted independently and controllers may start before or after it:

```bash
docker restart zenoh-spark-bridge
docker logs --tail 30 zenoh-spark-bridge
```

The bridge allow rule is a regex covering IDs `0..N-1`; it does not need to be
regenerated as the count changes. Namespaced launches through
`run_deploy_clean.sh` also select `gear_sonic_deploy/dds_sim_spark.json`
automatically. That file raises CycloneDDS's auto participant-index ceiling;
without it, eight controllers plus the local bridge exhaust the default `0..9`
range. The physical-robot launch path remains unchanged.

Launch the controllers in clean tmux sessions. The explicit zero pitch bias and
zero leg blend make the IDLE reference match the observed warmup posture used
by this concurrent run:

```bash
ROOT=/home/microagi/repos/GR00T-WholeBodyControl

tmux new-session -d -s sonic0 \
  "env DDS_INTERFACE=lo CONTROL_WALL_SCALE=0.1 \
   SONIC_INSTANCE_ID=0 SONIC_TOPIC_PREFIX=rt/sim/g1/0 \
   SONIC_SKIP_MOTION_SWITCHER=1 SONIC_IDLE_HOLD_REFERENCE=1 \
   SONIC_IDLE_PITCH_BIAS_DEG=0 SONIC_IDLE_LEG_BLEND=0 \
   ZMQ_INPUT_PORT=5556 ZMQ_OUTPUT_PORT=5657 \
   IDLE_TELEMETRY_LOGFILE=/tmp/sonic0_idle.csv \
   $ROOT/run_deploy_clean.sh"

tmux new-session -d -s sonic1 \
  "env DDS_INTERFACE=lo CONTROL_WALL_SCALE=0.1 \
   SONIC_INSTANCE_ID=1 SONIC_TOPIC_PREFIX=rt/sim/g1/1 \
   SONIC_SKIP_MOTION_SWITCHER=1 SONIC_IDLE_HOLD_REFERENCE=1 \
   SONIC_IDLE_PITCH_BIAS_DEG=0 SONIC_IDLE_LEG_BLEND=0 \
   ZMQ_INPUT_PORT=5558 ZMQ_OUTPUT_PORT=5658 \
   IDLE_TELEMETRY_LOGFILE=/tmp/sonic1_idle.csv \
   $ROOT/run_deploy_clean.sh"
```

Wait for `Init Done` in both sessions. Send `]`, then Enter, then `1` to each,
waiting for planner initialization between keys. Re-arm and release each robot
through its own lifecycle topic:

Namespaced launches also enable a per-instance live IDLE reference file at
`/tmp/sonic_idle_reference_<id>.txt`. Change a running controller without a
restart using:

```bash
# pitch trim in degrees [-20,20], leg blend [0,1]
# 0 = measured handoff legs; 1 = SONIC native default leg angles
python3 rtx_pod/set_idle_reference.py 0 -4 0.25
python3 rtx_pod/set_idle_reference.py 1  4 0.50
```

The controller polls the file every 500 ms and slews the new values onto its
frozen measured-pose reference. It only edits an active stationary-IDLE clip;
WALK and other planner motions are not modified. Writes are atomic, malformed
files retain the last valid target, pitch is clamped to ±20°, and leg blend to
`0..1`. Set `SONIC_IDLE_REFERENCE_FILE` explicitly to use a different path.

For a visually distinct eight-robot comparison while preserving one-variable
interpretability, use:

```bash
python3 rtx_pod/set_idle_reference.py 0 -6 0
python3 rtx_pod/set_idle_reference.py 1 -3 0
python3 rtx_pod/set_idle_reference.py 2  0 0
python3 rtx_pod/set_idle_reference.py 3  3 0
python3 rtx_pod/set_idle_reference.py 4  6 0
python3 rtx_pod/set_idle_reference.py 5  0 0.35
python3 rtx_pod/set_idle_reference.py 6  0 0.70
python3 rtx_pod/set_idle_reference.py 7  0 1.0
```

In the short 2026-07-22 visual screen all eight initially remained upright after
the slew. Pelvis heights spanned 0.661–0.729 m and measured base tilt spanned
1.1–10.9°. The long monitor then showed that this matrix is **not a qualified
hold set**:

| Robot | Target `(pitch°, leg blend)` | Fall time since release (sim s) |
|---:|---:|---:|
| 7 | `(0, 1.00)` | 112.84 |
| 6 | `(0, 0.70)` | 115.80 |
| 4 | `(+6, 0)` | 117.94 |
| 3 | `(+3, 0)` | 198.64 |
| 2 | `(0, 0)` | 199.50 |
| 0 | `(-6, 0)` | 209.30 |
| 5 | `(0, 0.35)` | 275.96 |
| 1 | `(-3, 0)` | 276.20 |

The targets were introduced late in the run, after the baseline had already
passed its 60-second qualification, so these are not exposure durations. The
ordering suggests the full native-leg blends and largest positive pitch consumed
balance margin first, but the trial is confounded by transport disturbances: a
shared 0.58–0.61 s RTX state-publication pause preceded robots 6/4, roughly 1 s
RTX command gaps preceded robots 3/2, and a 27.5 s command-delivery gap surrounded
the final pair. Use the matrix for supervised visualization only; reset to
`(0,0)` and requalify before treating any target as stable.

```bash
cd "$ROOT"
for i in 0 1; do
  DDS_DOMAIN=0 DDS_INTERFACE=lo python3 rtx_pod/fire_reset.py 3 4 rt/sim/g1/$i
done
sleep 5
for i in 0 1; do
  DDS_DOMAIN=0 DDS_INTERFACE=lo python3 rtx_pod/fire_reset.py 3 3 rt/sim/g1/$i
done
```

Verify both feeds and evaluators independently:

```bash
python3 rtx_pod/g1_dds_diag.py --domain 0 --interface lo \
  --topic-prefix rt/sim/g1/0 probe --dur 5
python3 rtx_pod/g1_dds_diag.py --domain 0 --interface lo \
  --topic-prefix rt/sim/g1/1 probe --dur 5
python3 rtx_pod/g1_dds_diag.py --domain 0 --interface lo \
  --topic-prefix rt/sim/g1/0 eval 10
python3 rtx_pod/g1_dds_diag.py --domain 0 --interface lo \
  --topic-prefix rt/sim/g1/1 eval 10
```

The 2026-07-21 run measured about 102 Hz for each Spark-side state stream and
RTF 0.100. Both robots completed 60.02 simulated seconds with zero displacement
and no fall; final tilt was 2.0° and 1.9°. Both controllers also recovered to
`Init Done` after the Spark bridge was restarted before release without
restarting the controllers. Mixed IDLE/WALK isolation and free-standing
transport-restart tests remain open.

#### Concurrent RTF sweep (2026-07-21)

The follow-up sweep used the same two-environment Isaac process and two SONIC
processes. Each point started with fresh controllers and a robot re-arm/release;
`CONTROL_WALL_SCALE` was set equal to RTF and `/tmp/sim_slowmo` to its reciprocal.
Short screens are labelled as such and must not be interpreted as 60-second
validation:

| RTF | Duration observed | Outcome |
|---:|---:|---|
| 0.125 | 16.58 s | Both upright; short screen |
| 0.1667 | 18.02 s | Both upright; short screen |
| **0.20** | **60.02 s** | **Both succeeded; final tilt 2.1° / 2.4°, displacement 0.55 m / 1.84 m** |
| **0.225** | ~29 s | **Both fell at 29.52 s / 28.98 s** |
| 0.25 | 21.62 s | One fell at 19.40 s |
| 0.2625 | 50.42 s | One fell at 44.40 s; survivor had drifted 1.70 m |
| 0.275 | ~23 s | Both fell at 23.64 s / 22.74 s |
| 0.30 | 4.22 s | One fell at 3.42 s |
| 0.333 | 14–22 s | Intermittent: one fall at 14.48 s; repeat survived 21.5 s with degraded control |

For this two-robot sweep, the measured long-horizon failure onset is bracketed
to **RTF 0.20–0.225**. RTF 0.20 was the highest demonstrated 60-second no-fall
setting, but it was not consistently stationary. The later compressed
eight-robot sweep also passed RTF 0.20 and made it the current supervised point;
use 0.175 for extra margin. Do not infer stability from an 18–20 second screen:
the RTF 0.225 and 0.25 failures occurred just after or well beyond that window.

#### RTF 0.15 concurrency sweep (2026-07-21)

The count sweep used one vectorized Isaac process, one bridge on each host, one
autossh tunnel, and one SONIC process per robot. Every controller used
`CONTROL_WALL_SCALE=0.15`; Isaac used `SIM_SLOWMO=6.666666667`. Each qualifying
run used the observed warmup pose, a cat-4 re-arm, a 10 s wall-clock settle, and
a cat-3 release. A short pass is not counted as stable: the accepted count had
to complete the 60.02-simulated-second evaluator.

| Count | Outcome at RTF 0.15 |
|---:|---|
| **4** | **PASS, 60.02 s:** all four at 0.09 m displacement, 1.9–2.0° tilt, 0.682–0.684 m pelvis height |
| **5** | **FAIL:** one fell at 14.06 s in the clean repeat; an earlier all-robot fall at ~23 s coincided with an autossh reconnect and was discarded |
| 6 | Intermittent: passed short 8 s and >20 s runs, but two fell by 5.30 s in the long qualification |
| 7 | FAIL in both attempts; first fall by 1.70 s and by 4.76 s in the repeat |
| 8 | FAIL in both attempts; first fall by roughly 1.4–4.6 s |
| 24 pinned | All 24 controllers initialized, but this was not released after the lower-count failures; see capacity result below |

The verified concurrency limit is therefore **four balancing robots at RTF
0.15**. In the accepted run, all four state feeds remained at 100.0–100.1 Hz
with 68–76 ms maximum observed gaps, Isaac held RTF 0.150, and the four Spark
processes used 2.13 GiB aggregate RSS and about 27.5% of one CPU core averaged
over their lifetimes. No controller entered feed-loss recovery.

The 24-process capacity check separates "starts" from "controls stably." All 24
SONIC processes initialized (12.59 GiB aggregate RSS, about 7.9 CPU cores), but
state delivery fell from about 90–93 Hz before controller startup to 24–26 Hz
afterward, with roughly 125–235 ms gaps, and cumulative Isaac RTF fell toward
0.12. Since five was already unstable with healthy transport, 24 was left
pinned rather than released. Do not describe 24 as a supported control count.

Use the one-participant diagnostic helper for count-wide lifecycle, feed, and
evaluation checks; it avoids consuming one DDS participant per robot:

```bash
python3 rtx_pod/g1_concurrent_diag.py --count 4 probe --duration 8
python3 rtx_pod/g1_concurrent_diag.py --count 4 lifecycle 4 --delay 2
python3 rtx_pod/g1_concurrent_diag.py --count 4 lifecycle 3 --delay 1
python3 rtx_pod/g1_concurrent_diag.py --count 4 eval \
  --sim-seconds 60 --wall-timeout 460 --interval 10

# Leave this running for long-duration failure attribution. It writes one compact
# snapshot every 5 s plus state/command gaps >=150 ms and the first fall transition.
tmux new-session -d -s sonic_monitor \
  "cd $ROOT && exec python3 rtx_pod/g1_concurrent_diag.py --count 8 monitor \
   --interval 5 --gap-ms 150 --out /tmp/g1_concurrent_monitor.jsonl"

# Run the same observer inside the RTX pod, after copying the helper into
# ~/live-sim. Domain 1 / eth0 is the simulator side of the bridge.
tmux new-session -d -s g1_monitor_rtx \
  "cd ~/live-sim && exec venv/bin/python g1_concurrent_diag.py --count 8 \
   --domain 1 --interface eth0 monitor --interval 5 --gap-ms 150 \
   --out /tmp/g1_concurrent_monitor_rtx.jsonl"
```

The in-sim evaluator continues checking pose after the 60-s qualification
boundary. `termination=1`/`passed=true` means the qualification window was
completed; it is not a terminal snapshot. A later fall replaces it with a
latched `fallen=true` result. The monitor independently derives tilt from each
LowState IMU, records DDS tick deltas across state gaps, and watches LowCmd in
the reverse direction. Comparing the Spark and RTX logs localizes a gap: clean
RTX state plus missing Spark state is the return transport path; clean Spark
commands plus missing RTX commands is the outbound path; matching RTX state
and RTF degradation points at Isaac. Do not use the full IDLE telemetry CSV for
unattended monitoring: eight instances produce tens of gigabytes per day.

An autossh interruption is a shared failure domain. During the discarded
five-robot run, the SSH child exited and restarted, all five controllers entered
feed-loss damping together, the bridges recovered without manual restart, and
the robots nevertheless fell. Startup-order/restart resilience does not imply
that an actively balancing robot can survive a multi-second WAN outage.

#### RTF 0.10 concurrency follow-up (2026-07-21)

Reducing RTF from 0.15 to 0.10 and compressing the shared tunnel raises the
verified count from four to **eight**:

| Count | Outcome at RTF 0.10 |
|---:|---|
| **7** | **PASS, 60.02 s:** all seven at 0.09 m displacement, 1.8–2.2° tilt, 0.683–0.684 m pelvis height |
| 8, uncompressed SSH | Intermittent FAIL: 10.50 s in one repeat and 36.84 s in the instrumented repeat |
| **8, compressed SSH** | **PASS, 60.02 s:** all eight at 0.09 m displacement, 1.8–2.2° tilt, 0.680–0.685 m pelvis height |

The instrumented uncompressed eight-robot run proved the loss was in the shared
transport rather than Isaac or SONIC compute. Pod-side state/command remained at
99.6/50.0 Hz, but Spark received only 63.1–63.5/29.8–30.0 Hz; command p99 gaps
were 142–150 ms and maxima reached 0.70 s. Spark GPU utilization was 4% and
policy inference normally took 0.4–0.6 ms.

`sim_tunnel.sh` now enables SSH compression by default. With it, Spark measured
97.5–97.7 Hz state and 49.0–49.4 Hz commands after the passing run; command p99
was 39–46 ms. The highly repetitive payload compressed from 14.25 to 0.79
Mbit/s in the state direction and from 3.34 to 0.24 Mbit/s in the command
direction in a 10 s sample. Use `SSH_COMPRESSION=no ./sim_tunnel.sh` only to
reproduce the old bottleneck.

That count sweep predated SSH compression. The compressed-tunnel follow-up kept
all eight robots and increased RTF with matched `CONTROL_WALL_SCALE`:

| RTF | Eight-robot compressed-tunnel result |
|---:|---|
| 0.125 | PASS 60.02 s; 0.09 m displacement, 1.8–2.2° tilt |
| 0.15 | PASS 60.02 s; 0.09 m displacement, 2.0–2.1° tilt |
| 0.175 | PASS 60.02 s; 0.01–0.03 m displacement, 1.8–2.3° tilt |
| **0.20** | **PASS 60.02 s; 0.09 m displacement, 1.8–2.2° tilt for all eight** |

RTF 0.20 is therefore the current demonstrated eight-robot operating point on
the compressed route. It has one full-duration qualification rather than the
repeat evidence available at 0.10, so keep 0.175 as the conservative fallback
if later 0.20 runs show intermittent transport-sensitive failures.

#### RTF 0.20 long-horizon transport incident (2026-07-22)

One eight-robot run passed its 60-second evaluator and then remained upright
until 492.3 simulated seconds after release (about 41 wall-clock minutes). All
eight crossed 45° tilt within less than 0.1 simulated seconds of each other.
The controller telemetry held the last upright quaternion for six samples and
then jumped from 0.8–3.9° to 87–89° in one sample. That discontinuity is not a
physically resolved fall trajectory: the controllers missed the intervening
state.

Isaac continued at an instantaneous and cumulative RTF of approximately 0.198
across the event. The SSH child, both bridges, and sim process did not restart.
The evidence therefore rules out accumulated simulator slowdown and independent
pose instability. It is consistent with a shared state-delivery stall somewhere
between the RTX DDS publisher and the eight Spark controllers; it does not by
itself identify Wi-Fi, WARP, SSH, Zenoh, or Spark DDS as the exact layer.

The original evaluator hid this late fall because it latched its 60-second
success payload. `StandEval` now continues pose checks after qualification, and
the `monitor` command above runs on both sides of the bridge. Compare their
timestamped gap/tick records with `sim_run.log` on the next incident. The
controller processes also demonstrated restart-order resilience in the next
full sim restart: all eight entered feed-loss damping, detected the restored
LowState stream, and soft-rearmed without controller restarts.

### 4a-bis. Watch it: WebRTC 3rd-person viewport (from a WARP laptop, NO tunnel)

The Omniverse viewport livestream (enabled by `--livestream_type 2 --public_ip <POD_IP>`
in §3) serves on the pod at **TCP `8011`** (signaling) + **TCP `49100`** + dynamic UDP media.
A WARP-enrolled laptop reaches the pod IP directly, so:

1. Get `<POD_IP>` on the pod: `hostname -i` (e.g. `10.4.8.23`).
2. Open NVIDIA's **Isaac Sim WebRTC Streaming Client** desktop app → connect to **`<POD_IP>`**.
   Do **not** use `<SERVICE_IP>` (SSH-only) — signaling will hang.

The camera image-server WebRTC (ports `55555-7` ZMQ + `60001-3`) is a *separate* stream
(the wrist/head camera feeds), not the interactive viewport.

For multi-environment runs, `sim_main.py` computes an elevated viewport pose
from `scene.env_origins`, so the stream frames the entire robot grid. Reset and
base-hold paths add those same origins to Isaac's environment-local default root
state; omitting that conversion stacks every robot at world `(0,0)` and makes
eight environments look like one robot.

### 4b. Clock skew note

The pod clock can differ from the Spark by >0.5 s and we lack `CAP_SYS_TIME` to
fix it, so Zenoh's HLC would reject messages. `deploy.sh` passes
`UHLC_MAX_DELTA_MS` into the Spark-side bridge container — set it generously:

```bash
export UHLC_MAX_DELTA_MS=2000
```

### 4c. GR00T policy server (Spark GB10)

```bash
cd ~/repos/Isaac-GR00T && source .venv/bin/activate && source scripts/activate_spark.sh
python -u gr00t/eval/run_gr00t_server.py \
    --model-path <checkpoint> --embodiment-tag UNITREE_G1_SONIC --device cuda:0 --port 5550
# ready when it prints: "✓ Server ready — listening on 0.0.0.0:5550"
```

Checkpoints on the Spark HF cache (`~/.cache/huggingface/hub/`) — pick by task:

| Checkpoint (`--model-path` ends in) | Trained on | Use for |
|---|---|---|
| `models--LucaFrat--groot-bs256/**/checkpoint-12000` | `dataset` (`g1_finetune` run, bs 256) | **manipulation / pick-place** (e.g. "put the bar into the crate") |
| `models--LucaFrat--groot-wbc-32/**/checkpoint-5000` | `dataset_wbc_train` | whole-body control |
| `models--LucaFrat--groot-wbc-16/**/checkpoint-8000` | `dataset_wbc_train` | whole-body control |

The GB10's unified memory fits several servers at once (~6 GB each) — run a second one on
`--port 5551` to compare checkpoints without killing the first.

### 4d. SONIC deploy (points at the tunnel instead of the Orin)

```bash
cd ~/repos/GR00T-WholeBodyControl/gear_sonic_deploy
ZENOH_JETSON_ENDPOINT=tcp/127.0.0.1:7447 UHLC_MAX_DELTA_MS=2000 DEPLOY_YES=1 \
  ./deploy.sh --input-type zmq_manager --output-type all g1zenoh
# wait for "Init Done"
```

`g1zenoh` starts a Spark-side zenoh bridge in Docker (`--network host`, so its
`tcp/127.0.0.1:7447` = the host tunnel from §4a). Against the sim it reaches "Init Done"
fine — the deploy's MotionSwitcher `ReleaseMode` does **not** block even though the sim
has no motion_switcher topics. Requires working Docker on the Spark + the
`eclipse/zenoh-bridge-dds:latest` image.

### 4e. VLA client + keyboard (what actually works for sim)

⚠️ **Do not use `launch_inference.py` for the sim path.** Its pane-0 deploy neither sets
`ZENOH_JETSON_ENDPOINT` (so it points at the Orin, not the tunnel) nor inherits it from
your shell (tmux panes get the tmux server's env), and it would start a *second* deploy
conflicting with §4d. Run the VLA client and a keyboard publisher directly instead:

```bash
# VLA client (point --port at the server from §4c; camera via the tunnel)
cd ~/repos/GR00T-WholeBodyControl && source .venv_inference/bin/activate
python gear_sonic/scripts/run_vla_inference.py \
    --host localhost --port 5550 --embodiment-tag unitree_g1_sonic \
    --prompt "put the bar into the crate" \
    --action-publish-rate 50 --action-horizon 40 \
    --camera-host 127.0.0.1 --camera-port 5555
```

The keyboard publisher **binds** `tcp://localhost:5580` (the client's `ZMQKeyboardSubscriber`
*connects*). Minimal version:

```python
import zmq, time
pub = zmq.Context().socket(zmq.PUB); pub.bind("tcp://localhost:5580"); time.sleep(0.5)
while True:
    k = input()
    pub.send_string(("prompt:" + k[2:]) if k.startswith("t ") else k)
```

### 4f. Start sequence (order matters)

In the keyboard publisher, type — **one key per line**, waiting a beat between:

1. **`k`** — start the C++ control loop (comes up in PLANNER mode). The deploy begins
   publishing robot state; the client stops printing "waiting for state msg".
2. **`i`** — send initial pose and switch **PLANNER → POSE** mode (manipulation). Deploy
   logs "Temporary motion completed. Reset to frame 0".
3. **`p`** — resume the policy loop (`pause_loop` starts `True`; without this it just prints
   "Pausing..." and never sends actions). After this you'll see
   `ZMQ: Sent latent action - frame: … token shape: (64,)` and the deploy
   `Received 64D token (latent action)` — the arms start moving.

Runtime keys: `t <text>` change prompt, `p` pause/resume, `i` re-init pose, `k` stop loop.

The **only** differences vs the real robot are `ZENOH_JETSON_ENDPOINT` and `--camera-host`
pointing at `127.0.0.1` (the tunnels) instead of the Orin's IP.

---

## 5. Shutdown

```bash
# Spark: stop the VLA client, deploy, keyboard pub, policy server, and the Spark bridge:
tmux kill-session -t vla; tmux kill-session -t deploy; tmux kill-session -t kbd
docker rm -f zenoh-spark-bridge            # the deploy's Spark-side bridge container
pkill -f "ssh -N .*7447:localhost:7447"    # the SSH tunnels
# (leave the GR00T policy server up if you want to reuse it — it just holds GPU mem.)

# RTX box:
ssh spark@<SERVICE_IP> 'pkill -9 -f sim_main.py; pkill -f zenoh-bridge-dds; pkill -f secondary_imu_adapter; pkill -f gear_sonic_camera_pub'
```

---

## 6. Performance & the real-time tradeoff

Authoritative RTF (measured via `env.sim.current_time` vs wall-clock), single env:

| Config | RTF | Notes |
|--------|-----|-------|
| GPU physics (`cuda:0`), cameras | 0.30 | GPU PhysX is **slower** for 1 env |
| CPU physics, cameras, `dt=0.005` (200 Hz) | 0.59 | better contacts, sub-real-time |
| **CPU physics, cameras, `dt=0.008` (125 Hz)** | **0.95** | **real-time; default** |

- **Use CPU physics.** GPU PhysX parallelizes across *environments* (thousands); a
  single G1 has far too few bodies to fill the GPU, so per-step CUDA sync dominates
  and CPU's serial loop is ~2× faster. The GPU's job here is **rendering only**.
- **Cameras are ~free** at 10 Hz offscreen — the physics `env.step` is the whole budget.
- **The VLA runs on the Spark GB10**, so the RTX GPU stays free for rendering.
- If you need higher-fidelity contacts than `dt=0.008` allows, lower `dt` and pair
  with **lockstep pacing** (deploy stepped to sim-time) so sub-real-time doesn't
  destabilize the controller — more work, not yet implemented.

---

## 7. Gotchas (each of these cost real time — read before debugging)

1. **`--headless` is mandatory.** Without it the sim prints *"Please left-click on
   the Sim window to activate rendering"* and hangs forever (no window in k8s).
   `--headless` → offscreen render. This looks exactly like a performance hang.
2. **CPU physics, not GPU** for a single env (see §6). `--device cpu`.
3. **Joint task, not Wholebody.** `Wholebody` tasks run Unitree's own ONNX
   locomotion policy every step (~40 % of step time) *and* fight SONIC's leg
   commands. Use `…-Joint` tasks = "G1 on a stand", external DDS drives arms+hands.
4. **DDS domain 1.** The sim uses `ChannelFactoryInitialize(1)`. Subscribers/adapters
   must match.
5. **The sim ignores `CYCLONEDDS_URI`** (unitree_sdk2py hardcodes default config:
   eth0 + multicast, which works pod-internally). Run the Zenoh bridge with
   `env -u CYCLONEDDS_URI` so it uses the same default and actually discovers the
   sim's topics. (No `CAP_NET_ADMIN` to enable loopback multicast, so don't rely on `lo`.)
6. **Clock skew + Zenoh HLC.** Pod clock may be >0.5 s off and unfixable (no
   `CAP_SYS_TIME`); pass `UHLC_MAX_DELTA_MS=2000` to both bridges (deploy.sh forwards it).
7. **`rt/secondary_imu` is required** by the deploy — run the IMU adapter (§2).
8. **RTF measurement:** use `env.sim.current_time`, **not** `LowState.tick` (a
   message counter, not ms) and **not** `loop_freq × decimation × dt` (the assumed
   `dt` was wrong by 2×). Read the *moving* average / long runs, not warmup-
   contaminated overall averages.
9. **`docker` is broken on the pod** — use the standalone Zenoh bridge binary, not
   the container.
10. **Wire parity is verifiable:** the camera message's `images` dict must have keys
    `ego_view` / `left_wrist` / `right_wrist` (480×640×3 uint8). The VLA client reads
    `images["ego_view"]` directly — a `head` key gives `KeyError: 'ego_view'` (see §2).
11. **Shared memory is fragile — the sim owns it; don't touch `/dev/shm`.** In the legacy
    single-robot path, both the camera feed and DDS state path use Python
    `multiprocessing.shared_memory` segments (named `/dev/shm/psm_*`). The concurrent
    path keeps body command/state records inside the Isaac process, but the camera path
    still uses shared memory. Two ways to break legacy/shared-memory consumers, both seen
    in practice:
    - **On deployments without the resilience patch, restarting the camera pub alone** while
      the sim runs: `MultiImageReader`'s
      `resource_tracker` *unlinks* the segments on exit ("N leaked shared_memory objects to
      clean up at shutdown"), so the new pub reads **0 frames / `cams=NONE`**.
    - **`rm -f /dev/shm/psm_*`**: this also nukes the **DDS input shm**, so
      `g1_robot_dds.dds_publisher()` reads `input_shm` as `None` and **silently skips the
      publish** — `rt/lowstate` drops to **0 msgs with no error in the log**, the sim keeps
      stepping, and `ls /dev/shm | grep -c psm` shows `0`. Camera may still work (its pub
      holds an open fd), which makes this very confusing. **Never `rm /dev/shm/psm_*`.**

    With the implemented self-healing reader patch, a sidecar bounce re-attaches safely; this
    must be verified by observing resumed cameras and `rt/lowstate`. Recovery for either
    failure on an unpatched deployment: restart the **whole sim stack together** — `pkill -9 -f sim_main.py;
    pkill -9 -f gear_sonic_camera_pub; pkill -9 -f zenoh-bridge-dds; pkill -9 -f
    secondary_imu_adapter; sleep 3; bash ~/live-sim/start_all.sh` (no `rm`; let the fresh sim
    recreate its own segments). Verify with `ls /dev/shm | grep -c psm` (should be >0) and a
    `rt/lowstate` subscriber (should be ~100 Hz) before bringing up the deploy.
12. **The viewport livestream is now ON by default** in `sim_main.py` (was gated on
    `--no_render`, which also freezes rendering via `render_interval → 1e6`, so you could
    never have both). It's decoupled and default-on: `LIVESTREAM=livestream_type` (default 2)
    unless you pass **`--no_livestream`**. `--public_ip` defaults to **`auto`** (this host's
    primary IP via a UDP-socket probe), so the advertised ICE candidate is the pod IP with no
    flag needed. The old belief that the viewport streamed on `55555-7` was wrong — those are
    the camera image-server; the interactive viewport is `8011`/`49100` (§4a-bis).
13. **The sim's base-orientation quaternion was wrong-order (cost the whole "flailing"
    debug).** `dds/g1_robot_dds.py` published `imu_state.quaternion` as `[x,y,z,w]`
    (scalar-last), but the real Unitree `LowState.imu_state.quaternion` — and everything
    downstream (`compute_projected_gravity`, the training data) — is `[w,x,y,z]`
    (scalar-first). Effect: the policy's `projected_gravity` read `[0.02, −0.99, 0.13]` (robot
    "tipped 97°") vs. training's `[−0.03, 0.02, −1.0]` (upright), driving constant recovery
    motion. Fix (sim-side, preserves parity): `imu_state.quaternion[:] = imu_array[[3,4,5,6]]`
    (was `[[4,5,6,3]]`). Sanity-check any sim IMU/orientation field by confirming
    `compute_projected_gravity(base_quat) ≈ [0,0,-1]` when the robot is upright.

---

## 8. Open items

- ✅ **Concurrent IDLE scaling passed** (2026-07-21): four robots completed
  60.02 simulated seconds at RTF 0.15, and eight completed 60.02 seconds at RTF
  0.10; every accepted robot finished at 0.09 m displacement without a fall.
  Five is the first unreliable count at RTF 0.15; the next RTF 0.10 boundary
  above eight is not yet measured. Complete the mixed-mode, selective-reset, and
  one-controller-restart acceptance tests before expanding either supported
  boundary.
- ✅ **Full VLA loop closed** (2026-07-08): GR00T `bs256/checkpoint-12000` on `:5551` →
  deploy `g1zenoh` (POSE mode) → sim G1, prompt `"put the bar into the crate"`. Confirmed
  the arms move under policy control (arm-joint Δ ≈ 3.9 rad / 3 s; deploy consuming 64-D
  latent tokens at `LowState age ~10 ms`). *Task-completion quality* (does it actually seat
  the bar in the crate) still depends on how well the checkpoint matches this scene.
- **Validate dex3 grasp fidelity at `dt=0.008`.** If grasps jitter, drop to
  `dt=0.006` (~0.75×) or implement lockstep pacing for full-fidelity + stability.
- **Optional:** switch the task's cameras from `CameraCfg` to `TiledCameraCfg` for
  cheaper rendering headroom (not needed at current rates).
