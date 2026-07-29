# Project Diary: Unitree G1 + GR00T-WholeBodyControl on DGX Spark

## 2026-05-18 — Architecture Decision
- Decided against upgrading the G1 (JetPack 5.1.1 / TensorRT 8.5.2). Too risky for existing low-level control.
- New target: DGX Spark runs modern stack (TensorRT 10.16, SONIC/GEAR-SONIC, PICO service). G1 stays frozen and only executes low-level commands over ZMQ/Ethernet.
- Spark = brain + teleop server. G1 = body.

## 2026-05-18 — G1 Dual-Homed Networking (Layer 1)
- WiFi (Andrews-iPhone-G1) set as primary for internet (metric 100).
- Ethernet kept as reliable local fallback (metric 20100) for motion controller and SSH.
- Careful use of `never-default`, `route-metric`, and clearing stale routes. Autoconnect + Cloudflare DNS (1.1.1.3) enabled.
- Lesson: Always verify with `ip route get 8.8.8.8` and `ip route get 192.168.123.1` after every change.

## 2026-05-18 — GR00T-WholeBodyControl Repo
- Cloned full repo + LFS on the Spark (4.6 GB total).
- Key directories: `gear_sonic_deploy/`, `gear_sonic/`, `motionbricks/`.
- Confirmed G1-specific assets (meshes, USD models, Unitree SDK libs, `roboticsservice_1.0.0.0_arm64.deb`) are present.

## 2026-05-18 — TensorRT 10.16.1 on DGX Spark
- Installed via official Ubuntu 24.04 arm64 local repo (10.16.1 + CUDA 13.2).
- `trtexec` works. C++ libraries and dev packages are solid.
- Python bindings (`python3-libnvinfer`) land in system Python 3.12 only.
- Lesson: Never rely on conda `python3` for system CUDA/TensorRT packages.

## 2026-05-18 — XRoboToolkit-PC-Service (roboticsservice)
- Installed vendored `roboticsservice_1.0.0.0_arm64.deb`.
- Created user systemd service (`~/.config/systemd/user/roboticsservice.service`).
- Hit `libicuuc.so.70` missing → fixed by manually installing `libicu70` from Jammy.
- Service now reaches "release mode".

## 2026-05-18 — PICO Client Setup (XRoboToolkit APK)
- Downloaded XRoboToolkit APK from https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/
- Installed onto PICO headset using `adb install`

## 2026-05-18 — Python Environment Strategy for GR00T
- GR00T uses `uv` + managed Python (mostly 3.10) for `.venv_inference`, `.venv_teleop`, etc.
- On Spark we created `.venv_inference` using **system Python 3.12 + `--system-site-packages`** so TensorRT is visible.
- This is the pattern that works when you have system-installed NVIDIA stacks.

## 2026-05-20 — MuJoCo Sim-to-Sim Bring-up
- Created `.venv_sim` (uv, Python 3.10) via `install_scripts/install_mujoco_sim.sh`.
- `unitree_sdk2_python` needs CycloneDDS; the bundled libs lack cmake config + `idlc`.
  Built CycloneDDS 0.10.x from source → `~/cyclonedds/install`, then
  `CYCLONEDDS_HOME=~/cyclonedds/install uv pip install -e external_dependencies/unitree_sdk2_python`.
- Runtime needs `LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib`.
- DDS on **loopback** doesn't work out of the box: `lo` has no MULTICAST flag and no
  multicast route. Fixed with `sudo ip link set lo multicast on` +
  `sudo ip route add 239.255.0.0/16 dev lo`. Without it, sim and the C++ deploy never
  discover each other (sim log: "lo is not multicast-capable: disabling multicast").
- MuJoCo viewer: robot hangs off the floor via a virtual **elastic band**. Key **9**
  toggles it off (real physics), **7/8** lower/raise. `backspace` resets pose.

## 2026-05-20 — PICO Body Tracking
- `is_body_data_available()` stayed False while head+controllers streamed fine →
  body tracking is a separate feature. Needs the **2 ankle motion trackers** paired +
  calibrated and XRoboToolKit set to **"Full body"**. After that, body data flowed.
- `pico_manager_thread_server.py` bugs fixed: `enable_vis_rerun` wasn't passed in the
  `--manager` path; `get_g1_key_frame_poses` import was gated behind `--vis_vr3pt/--vis_smpl`
  but calibration needs it always; added a mode-independent Rerun update.

## 2026-05-20 — Real G1 Networking + the Zenoh DDS Bridge (the hard one)
- Spark↔G1 over a direct Ethernet cable on `enP7s7` = static **192.168.123.222/24**,
  `never-default` (internet stays on WiFi). Robot: **.161** = motion control board
  (publishes `rt/lowstate`/reads `rt/lowcmd`), **.164** = Jetson (`ssh unitree@…`, key-based;
  sudo pw `123`; hostname `ubuntu`; runs `master_service` + DDS on `eth0` domain 0).
- **Problem:** could ping the robot but received **zero DDS** from it. Diagnosed as a
  one-way multicast issue — the robot's internal switch does IGMP snooping and Linux
  defaults to IGMPv3, so the robot's inbound multicast (`rt/lowstate`) never reaches us,
  while our outbound multicast does reach the robot. Proof: an SPDP listener **on the
  Jetson** saw `.161` (122 pkts), `.164`, and `.222` (us); the same listener on the Spark
  saw nothing from the robot.
- **Solution:** `eclipse/zenoh-bridge-dds` on both ends, tunneling DDS over a Zenoh TCP
  link on the reliable wired path (no multicast needed across the switch).
  - Jetson (`~/zenoh-dds/start_bridge.sh`): `--network host`, `CYCLONEDDS_URI` pinning
    **eth0**, zenoh `mode peer` + `listen tcp/0.0.0.0:7447`, `allow [rt/lowstate, rt/lowcmd, rt/secondary_imu]`.
  - Spark (`~/cyclonedds_spark.xml` + `~/zenoh-spark-config.json5`, auto-started by deploy.sh):
    `CYCLONEDDS_URI` pinning **lo**, zenoh `connect tcp/192.168.123.164:7447`, same allow list.
- Bridge gotchas (cost us hours):
  - Config paths must **exist as files before `docker run`** — otherwise Docker creates
    them as **directories** and the bridge crash-loops ("Is a directory").
  - Default zenoh `peer` mode listens on an **ephemeral** port; must set an explicit
    `listen tcp/0.0.0.0:7447` so the other side can connect.
  - Disable zenoh **multicast scouting** and use explicit `connect` — scouting won't
    cross the robot switch either.
  - `plugins.dds.general.network_interface` is **not** a valid field; pin the interface
    via `CYCLONEDDS_URI` instead.
  - Spark-local DDS must be on **`lo`** (the multicast-route fix supports it); `enP7s7`
    local discovery is diverted by the `239.255.0.0/16 dev lo` route.

## 2026-05-20 — Jetson WiFi → STARLINK-MUC
- Switched G1 Jetson WiFi to STARLINK-MUC via `nmcli`, keeping `eth0`/`unitree1` untouched
  (it's the robot link + our SSH). New WiFi profile: `ipv4.never-default no`,
  `route-metric 100`; set `unitree1` `route-metric 10000` (modify only, never bounce).
- PSK gotchas: a multi-line backslash-continued `nmcli modify` **corrupted the psk**
  (tripled it); and the supplied password had a typo (extra `k`). Verify with
  `nmcli -s -g 802-11-wireless-security.psk connection show <name>` and set it on a single line.

## 2026-05-20 — deploy.sh + input-type lesson
- `deploy.sh` changes: `DEPLOY_YES=1` to skip the `read` prompt (ble.sh breaks it);
  new **`g1zenoh`** target (DDS on `lo`, real-robot behavior, `--disable-crc-check`
  because LowState is round-tripped via Zenoh); auto-start the Spark zenoh bridge
  (writing its config files first).
- **`--input-type manager` vs `zmq_manager`:** `manager` is a meta-switcher
  (Shift+1/2/3/4 = keyboard/gamepad/zmq) whose zmq sub-endpoint only consumes the
  **`pose`** topic and is toggled by **ENTER** (safety-disabled on switch). `zmq_manager`
  is the full PICO state machine and consumes **`pose` + `planner` + `manager_state`**.
  Symptom that wasted time: robot didn't move because the PICO was in **PLANNER** mode
  (publishing the `planner` topic) while the deploy was subscribed to `pose`. Use
  `zmq_manager` for real teleop; PICO **A+X** = POSE, stick-click = VR_3PT, A+B/X+Y cycle locomotion.
- Verified end-to-end control: `lowcmd` published ~486 Hz with real gains,
  `mode_machine` matched lowstate, motor `tau_est` non-zero and `q` tracking the command.
- **Teleop confirmed: the G1 balanced, walked, and hopped well under PICO teleop.**

## 2026-05-20 — Performance
- Teleop felt laggy. Bottleneck was **not** GPU (GB10 ~22%) or network (robot ping 0.13 ms)
  — it was **CPU**: the **ghostty terminal rendering the deploy's stdout consumed ~10–13
  of 20 cores**, starving the 50 Hz control loop. Fix: redirect the deploy's output to a
  file/`/dev/null` (stdin/keyboard still works) and run `pico_manager` headless
  (drop `--vis_rerun`; Rerun was +23 GB RAM + a core).

## 2026-05-21 — Fork + Branch
- Installed `gh` user-local (`~/.local/bin`, no sudo). Fork already existed at
  **MicroAGI-Labs/GR00T-WholeBodyControl**; pushed this session's work to branch
  **`andy-dgx-teleop`** (remote `microagi`, HTTPS via gh token).
- Committed the Zenoh bridge setup into the repo: `gear_sonic_deploy/zenoh/`
  (`start_bridge_orin.sh` for the robot Orin, Spark reference configs, README).

## 2026-05-21 — WiFi → RobotNet (UniFi Express 7)
- Replaced the ad-hoc WiFi (STARLINK-MUC / iPhone hotspot) with a dedicated **RobotNet**
  SSID on a **UniFi Express 7** router for the whole rig (Spark + G1 management/internet).
- The wired `192.168.123.x` control link (Spark `enP7s7` ↔ G1 `eth0`) is unchanged — Zenoh
  bridge + teleop continue over Ethernet, independent of WiFi.
- G1 Orin WiFi connected to RobotNet via `nmcli` over the wired SSH (so SSH never drops):
  `eth0`/`unitree1` untouched (route-metric bumped to backup, no bounce), new WiFi profile
  `ipv4.never-default no` + `route-metric 100`. Single-line psk (multi-line `nmcli modify`
  corrupts the key). Result: wlan0 = 192.168.1.229 (default route via WiFi), robot subnet
  still on eth0. Spark↔robot wired ping 0.25ms unaffected.
- RobotNet is **WPA2/WPA3 mixed mode**; the modern Spark uses `sae` (WPA3), but the older
  Jetson connects fine via `wpa-psk` (WPA2 path) — more compatible with JetPack 5's stack.
  PSK was read off the Spark's already-working RobotNet profile
  (`nmcli -s -g 802-11-wireless-security.psk con show RobotNet`).
- Pruned stale WiFi profiles on the Orin — removed `STARLINK-MUC`, `OnePlus12`,
  `TP-LINK_0B20_5G`; only `RobotNet` remains. Their PSKs are saved locally on the Spark
  at `~/robot-wifi-credentials.txt` (`chmod 600`, **not committed** — DIARY is a public fork).
- Did the Orin SSH + nmcli edits through a persistent **tmux** session (`tmux-mcp`),
  raw-mode interactive SSH, so the work survives disconnects.

## 2026-05-21 — Full restart test + untethered WiFi teleop
- Ran a clean shutdown → confirm → cold start of the whole stack. Software cold-starts
  cleanly: Orin bridge → Spark bridge → roboticsservice/pico_manager → deploy.
- **Untethered whole-body teleop over WiFi confirmed.** Routed the zenoh Spark↔Orin hop over
  RobotNet — Spark bridge connects to the Orin's wlan0 IP via
  `ZENOH_JETSON_ENDPOINT=tcp/<orin-wifi-ip>:7447 ./deploy.sh ... g1zenoh`; the Orin bridge
  still reads the robot's DDS on its internal `eth0`. Motors execute over WiFi
  (`motorstate=0`, joints track command) — WiFi is viable for low-level control. lowstate
  ~1000 Hz with occasional ~110ms jitter spikes (didn't break teleop; wired is steadier).
- **`0x40000` motor fault = robot-state, not WiFi:** after a power-cycle all 29 motors fault
  `0x40000` and ignore `lowcmd` (cmd_q not tracked, tiny tau). Cleared by a fresh robot
  restart + re-entering dev/low-level mode on the remote. (An earlier "WiFi jitter trips the
  motor watchdog" hypothesis was wrong.) Verify cleared via lowstate `motorstate==0`.
- **`body.timeStampNs` is a PICO-app bug** — always frozen/0 even when tracking is live;
  use pose-change / per-joint IMU timestamps as the liveness signal. pico_manager now prints
  a `[PICO]` health line every 2s (pose_chg/joint_ts_adv/body_ts_adv/connected/miss),
  replacing the old RAW POSE DEBUG spam.
- New tooling: `gear_sonic/scripts/pico_watchdog.sh` (auto-restart dead/wedged pico_manager),
  `gear_sonic/scripts/pico_link_telemetry.py` (standalone PICO link diagnostic).

## 2026-05-21 — Static IPs (UniFi Express 7 DHCP reservations)
- Reserved fixed RobotNet (192.168.1.x) DHCP leases on the UniFi Express 7 for every device,
  so IPs stop drifting — the drift repeatedly broke the PICO↔Spark link and the zenoh endpoint:
  - Spark (DGX) `wlP9s9` MAC `50:2e:91:5b:34:52` → **192.168.1.178**
  - G1 Orin `wlan0` MAC `4c:bb:47:ab:f8:d8` → **192.168.1.229**
  - PICO headset → reserved (its RobotNet lease, per UniFi)
- Result: the PICO PC-Service IP (Spark `192.168.1.178`) and `ZENOH_JETSON_ENDPOINT`
  (Orin `192.168.1.229:7447`) are now stable across reboots. Resolves the IP-drift issue.

## 2026-07-09 — SONIC balance in RTX6000 Isaac sim is latency-capped (RTF 0.333)
- Goal: run the RTX6000 Isaac sim **faster** while SONIC (on the Spark) keeps the G1
  upright, controller-on-Spark + sim-on-RTX6000 + swappable real/sim interface intact.
  Full write-up in [`RTX_SIM_LATENCY.md`](RTX_SIM_LATENCY.md).
- **Root cause = fixed network latency, not sim dynamics.** The RTX6000 is a remote k8s
  pod reachable **only via Cloudflare WARP** (`spark@10.5.7.178`, port 22 only, UDP
  blocked) → all DDS is forced through one `ssh -L 7447` tunnel (TCP-over-TCP). Warm RTT
  floor **~38 ms** (cold `connect()` ≈ warm, so it's the path, not overhead). Live DDS
  `rt/lowstate` jitter at RTF 1.0: p50 9 ms / p99 23 ms / worst 69 ms (zenoh coalesces
  stale — much milder than raw TCP-over-TCP). The deploy's printed "LowState age"
  (2–5 ms) is stamped **locally on arrival** (`utils.hpp`), so it's blind to the ~19 ms
  transit.
- **RTF is the only variable** — sim math per step is byte-identical regardless of pacing,
  so the sole difference between "balances" and "falls" is loop delay in *sim-time*
  (≈ 38 ms × RTF + async). Measured cliff: RTF 0.20 ✅, **0.333 ✅ (30 s+ stable, tilt →
  sub-1°)**, 0.40 ❌ hard, 0.50/1.0 ❌. ⇒ SONIC's delay margin ≈ **18–20 ms sim-time**.
- **38 ms is NOT a physics limit.** Standing G1 ≈ inverted pendulum, ωᵤ ≈ 4 rad/s → delay
  budget ~250 ms (ωᵤ·T ≲ 1); at 38 ms ωᵤ·T ≈ 0.15. The limiter is SONIC being a stiff,
  high-bandwidth policy effectively trained at zero latency — phase margin dies at ω_c·T.
- **Levers tested (all feature-flagged in the deploy, default-off → real robot unchanged):**
  - Forward state prediction (`PRED_HORIZON_S` + `/tmp/pred_horizon`; `q += dq·T`, quat
    integrated by gyro): **ineffective / mildly harmful** — first-order can't catch the
    slow-growing delay instability.
  - Commanded-gain soften/damp (`/tmp/gain_scale`): **marginal** — lowers peak tilt but
    doesn't prevent the fall; too soft can't hold the stance.
  - Neither moves the ceiling → it's delay-margin-bound.
- **Delivered: sim now runs 1.67× faster (RTF 0.333) with a stable unaided stand** vs the
  prior RTF 0.2 point. Persisted in `run_deploy_direct.sh`
  (`CONTROL_WALL_SCALE=0.333`, must equal sim RTF = 1/`sim_slowmo`; pod `/tmp/sim_slowmo=3`).
  Real-robot defaults (`CONTROL_WALL_SCALE=1.0`, prediction/gain off) reproduce the
  original behavior exactly.

## 2026-07-10 — Connection resilience + SONIC balancing reproduced (no dynamics gap)
- Goal: implement [`SIM_RESILIENCE_PLAN.md`](SIM_RESILIENCE_PLAN.md) so the Spark↔RTX6000-sim
  loop **auto-recovers** from network flaps / process bounces, and confirm SONIC balances the
  G1 in Isaac under forced disruption. Full plan + status in that doc; details in memory
  `sim-resilience-implementation`.
- **Resilience implemented & demonstrated (zero operator input):**
  - **A — autossh tunnel** (`sim_tunnel.sh` + a `rtx-pod` `~/.ssh/config` alias = single source
    of truth for the pod endpoint). Replaces the bare `ssh -N -L`; kill the child ssh → it
    re-establishes `:7447`/`:5555` in ~1 s.
  - **B — deploy auto-recovery** (`g1_deploy_onnx_ref.cpp`, env-flagged `AUTO_RECOVER=on`): new
    `RECOVER_DAMPING` state + `FeedHealthy()` (arrival-age **and** `tick`-advance). On LowState
    loss → damping (threads stay alive, no terminal stop); on return fresh+advancing → soft
    re-arm via the INIT ramp → auto-resume CONTROL, mode preserved. Damping trigger
    `AUTO_RECOVER_ABSENT_MS` set to **1 s** (rides out jitter; was 200 ms). Same binary, strictly
    better on the real robot.
  - **C — self-healing shm** (pod `dds/sharedmemorymanager.py`, `tools/shared_memory_utils.py`):
    `resource_tracker.unregister` at every attach-by-name site (a consumer must never unlink a
    segment it didn't create — CPython unlinks tracked segs on exit) + reopen-on-error + ms
    timestamps. Proven with a create/attach/exit test; bouncing the camera pub alone now
    re-reads `cams=[ego_view,left_wrist,right_wrist]` instead of `cams=NONE`.
  - **D — pod watchdog** (`sim_watchdog.sh`): restarts the sim if `rt/lowstate` dies. Detection
    verified; the auto-restart path needs flock/backoff refinement before being relied on.
  - **G — single-instance guard** (added after a duplicate-publisher scare that turned out to be
    a miscount): `stack_singleton.py` (kill/count/assert by process `comm`, collapses
    parent+child, never self-matches) + `flock` + post-launch assertion in `start_flat.sh`.
  - **Forced-disruption tests all auto-recovered:** tunnel flap, camera-pub bounce, and a full
    sim restart → the deploy damped then resumed on its own each time.
- **SONIC balancing in Isaac — reproduced (~30 s unaided stand, sway 2–12° with active
  recovery).** It is a **latency** effect, **not** a PhysX/MuJoCo dynamics gap — the earlier
  "dynamics gap / doesn't transfer" writeups were wrong; deleted the two hallucinated memories
  and corrected the rest. Working recipe (order matters):
  1. **Warmup geometry**: RIGID hold (`SIM_BASE_SOFT=0`) + `SIM_WARMUP_JOINTS=1` (forces the
     default stance — knee **0.669**, a shallow stand) + init z **0.793** (feet-on-ground for
     that stance; z=0.8/0.85 jammed the knees to ~1.9 and toppled on release). SOFT hold flails.
  2. **Fresh deploy** started after the sim is up (don't drag it through sim-restart
     auto-recoveries), planner + standing (`]`, ENTER, `1`), warm up held, then **cat-3 release**.
  3. **Slower than the old 0.333**: the current autossh tunnel needs RTF **0.10–0.125**
     (`/tmp/sim_slowmo`=8–10, matched `CONTROL_WALL_SCALE`) — higher tunnel RTT now, so more
     delay margin. Feed was smooth (0 gaps >100 ms) and auto-recovery never misfired during the
     stand. The ~30 s ceiling is marginal stability; indefinite needs a lower actual RTT.
- **Real bugs found & fixed along the way:**
  - The flat task had silently regressed to a **fixed-base** robot preset
    (`g1_29dof_dex3_base_fix` → reverted to `g1_29dof_dex3_wholebody`) — a welded base can't balance.
  - **The `pkill -f` self-kill footgun** (cost hours): `pkill -9 -f "g1_deploy_onnx_ref"` matches
    the *shell running it* (the pattern is in that shell's own cmdline) and SIGKILLs it, so the
    command dies before doing anything — this was behind most "sessions won't launch". Kill via
    `ps -eo pid,args | awk '/[t]arget…/{print $1}' | xargs -r kill -9` (the `[t]` self-exclusion
    trick); on the pod use `stack_singleton.py`.
  - Deploy launch under the harness: the `sonic_deploy` ble.sh pane mangles/eats send-keys of
    long commands. Reliable path: run the deploy as a tmux pane's **direct command** (no login
    shell → no ble.sh), stdout→logfile, stdin=PTY (the keyboard handler uses termios raw mode,
    so it needs a PTY, not a FIFO). The binary needs no conda env.

## Open TODOs
- [ ] **Push RTX6000 sim RTF past 0.333** — the main lever is a lower-latency path to the
  pod (co-locate the Spark / direct link): `stable RTF ≈ margin / RTT`, so cutting the
  ~38 ms WARP RTT scales the ceiling linearly. See `RTX_SIM_LATENCY.md` §7.
- [ ] **Make cold-start hands-off:** (a) run `roboticsservice` as a persistent systemd
  service so pico_manager doesn't bounce it (avoids the PICO needing a manual reconnect),
  (b) document/automate the robot motor-enable (clear `0x40000` after power-cycle).
- [ ] **Verify LowCmd CRC across the Zenoh round-trip.** `g1zenoh` mode passes
  `--disable-crc-check` on the assumption the bridge re-serialization may alter bytes.
  Teleop works with it disabled — confirm whether the robot accepts the round-tripped
  `rt/lowcmd` CRC and whether the incoming check can be safely re-enabled (safety
  hardening). Tracked in `gear_sonic_deploy/zenoh/README.md`.
- [ ] **Indefinite (vs ~30 s) Isaac balance** — the stand is marginally stable at the current
  tunnel latency (needs RTF 0.10–0.125). The real fix is a lower actual RTT to the pod
  (co-locate / faster link); `stable RTF ≈ margin / RTT`. See `RTX_SIM_LATENCY.md` §7.
- [ ] **Refine the pod watchdog auto-restart** (`sim_watchdog.sh`): detection works, but the
  restart contends on the WS-G `flock` when it retries — add single-in-flight + backoff so a
  dead sim reliably comes back (the deploy already auto-recovers once lowstate returns).

## Key Lessons So Far
- External high-power compute (Spark) + frozen robot (G1) is the right pragmatic split.
- Vendored `.deb`s from PICO/ByteDance are brittle on newer Ubuntu — expect missing libs (icu, etc.).
- Always create project venvs with `--system-site-packages` when depending on system TensorRT/CUDA.
- Network metrics and route verification must be obsessive on dual-homed robots.
- `uv` + system-site-packages is the current winning combo for this hardware mix.
- `adb install` is reliable for getting the XRoboToolkit client onto the PICO.
- The G1's internal switch breaks external DDS multicast (IGMP snooping + IGMPv3) — bridge
  DDS over Zenoh on the wired link instead of fighting multicast.
- For any DDS-bridge container: create the config files **before** `docker run`, pin the NIC
  via `CYCLONEDDS_URI`, listen on a fixed port, and use explicit unicast `connect`.
- Spark-local DDS lives on `lo` (with the multicast flag+route); the deploy reaches the real
  robot through the bridge, not a direct robot-subnet interface.
- Match the deploy `--input-type` to the PICO stream: `zmq_manager` for real teleop, and the
  PICO must be in **POSE** mode (topic `pose`) for full-body tracking.
- On a shared desktop, terminal output volume is a real-time hazard — redirect deploy logs.
- **Closed-loop balance over a remote link is latency-capped, and the cap is the
  *controller's* delay margin, not physics.** A stiff, high-bandwidth policy (SONIC) tolerates
  only ~18–20 ms of loop delay; over a fixed ~38 ms round-trip that pins the stable sim RTF
  near 0.33. Measure warm RTT (not the deploy's locally-stamped "age") to know the real
  budget, and match `CONTROL_WALL_SCALE` to the sim RTF. Prediction/gain-softening don't
  rescue a slow-growing delay instability — cut the delay (network) instead.
- **There is no SONIC/Isaac "dynamics gap".** SONIC balances the G1 in Isaac fine when the sim
  runs slow enough for the round-trip latency; the earlier "MuJoCo balances, PhysX doesn't /
  doesn't transfer" conclusion was a wrong intermediate hypothesis (it had ruled out
  contact/armature/friction/solver/mass — all irrelevant because latency dominated). Deleted
  those hallucinated writeups. What *does* matter for a clean stand is the **warmup geometry**:
  hold the robot RIGIDLY in the policy's default stance (`SIM_BASE_SOFT=0` + `SIM_WARMUP_JOINTS=1`)
  at the feet-on-ground base height (init z 0.793), release with cat-3, from a *fresh* deploy.
- **Design for fail-then-auto-recover, not fail-safe-and-stop.** A remote-driven controller
  should treat a feed gap as a transient: drop to damping, keep the realtime threads alive, and
  auto-re-arm (soft-ramp from measured pose) when the feed returns — no operator input, no
  process restart. Pair it with a self-healing transport (autossh) and self-healing shm
  (opt out of `resource_tracker` so a consumer never unlinks the producer's segment on exit).
- **`pkill -f <pattern>` self-matches and SIGKILLs its own shell** when the pattern appears in
  that shell's command line — it silently aborts the whole command (before e.g. `tmux
  new-session`). Kill via `ps | awk '/[p]attern/'` (bracket self-exclusion) or a `comm`-based
  helper; same class of bug as `pgrep -f` inflating process counts by matching itself.