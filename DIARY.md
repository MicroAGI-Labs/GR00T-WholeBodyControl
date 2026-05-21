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

## Open TODOs
- [ ] **Make cold-start hands-off:** (a) run `roboticsservice` as a persistent systemd
  service so pico_manager doesn't bounce it (avoids the PICO needing a manual reconnect),
  (b) document/automate the robot motor-enable (clear `0x40000` after power-cycle).
- [ ] **Verify LowCmd CRC across the Zenoh round-trip.** `g1zenoh` mode passes
  `--disable-crc-check` on the assumption the bridge re-serialization may alter bytes.
  Teleop works with it disabled — confirm whether the robot accepts the round-tripped
  `rt/lowcmd` CRC and whether the incoming check can be safely re-enabled (safety
  hardening). Tracked in `gear_sonic_deploy/zenoh/README.md`.

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