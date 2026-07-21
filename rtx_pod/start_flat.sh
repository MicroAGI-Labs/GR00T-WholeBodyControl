#!/bin/bash
# start_flat.sh — bring up the SIMPLE WHOLE-BODY G1 stack (floating base, flat plane).
#
# Same as start_all.sh but launches the Isaac-Flat-G129-Dex3 task driven by the
# whole-body 29-joint DDS provider (--action_source dds_lowcmd29), so the external
# SONIC controller balances a FREE-base G1 — the correct "sim impersonates the robot"
# setup. All 29 body joints + dex3 fingers come from rt/lowcmd. No tables/objects.
#
# Access is identical to start_all.sh (POD IP; DDS :7447, camera :5555, WebRTC viewport).
set -uo pipefail
LS="$HOME/live-sim"
cd "$LS"

# The policy-native crouch is the validated handoff pose for stationary IDLE.
# A caller can still select the straighter visual pose with
# SIM_WARMUP_POSE=vertical, or the measured balance equilibrium with
# SIM_WARMUP_POSE=observed, for A/B tests.
export SIM_WARMUP_POSE="${SIM_WARMUP_POSE:-sonic}"

# ---- Single-instance guard (SIM_RESILIENCE_PLAN.md Workstream G) -------------
# Two overlapping bring-ups used to race past the pkill guards into DUPLICATE
# rt/lowstate / rt/secondary_imu publishers, which silently corrupt the DDS state
# (the robot flails, no error anywhere). Serialize bring-up under an exclusive
# lock, then wipe any prior stack robustly (by interpreter, never a self-matching
# pkill -f), so we always start from a clean, single, consistent stack.
LOCK="$LS/.stack_bringup.lock"
if [ "${_STACK_LOCKED:-}" != "1" ] && command -v flock >/dev/null 2>&1; then
  # re-exec via absolute `bash <script>` (NOT "$0", which may be a bare relative
  # name that isn't on PATH after exec -> silent failure/no sim).
  # --close prevents long-lived sim/bridge children from inheriting the lock
  # descriptor and blocking every later restart after this script exits.
  exec env _STACK_LOCKED=1 flock --close -w 60 "$LOCK" bash "$LS/start_flat.sh" "$@"
fi
echo "==> [pre] robust clean slate (kill any prior stack instances)"
# Stop the bridge supervisor before killing the bridge.  Otherwise it races the
# clean-slate phase by immediately respawning Zenoh, leaving a stale bridge PID
# and making a full-stack restart depend on process ordering.
if [ -r "$LS/bridge_supervisor.pid" ]; then
  supervisor_pid="$(cat "$LS/bridge_supervisor.pid" 2>/dev/null || true)"
  [ -n "$supervisor_pid" ] && kill "$supervisor_pid" 2>/dev/null || true
  rm -f "$LS/bridge_supervisor.pid"
fi
python3 "$LS/stack_singleton.py" kill || true

POD_IP="$(hostname -i | awk '{print $1}')"
SIM_ARGS="--device cuda --headless --enable_cameras --task Isaac-Flat-G129-Dex3 --robot_type g129 --enable_dex3_dds --action_source dds_lowcmd29 --render_interval 12"

alive() { local p; p="$(cat "$1" 2>/dev/null)"; [ -n "$p" ] && kill -0 "$p" 2>/dev/null; }

# Stop only the PID we started.  Do not use `pkill -f` here: the bridge's name
# appears in this shell's command line, so that pattern can kill the bring-up
# shell itself and strand the stack with no Zenoh listener.
stop_pidfile() {
  local pidfile="$1" pid
  pid="$(cat "$pidfile" 2>/dev/null || true)"
  [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
  rm -f "$pidfile"
}

start_zenoh_bridge() {
  env -u CYCLONEDDS_URI \
      LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
      UHLC_MAX_DELTA_MS=2000 \
      setsid "$LS/zenoh-bridge-dds" --config "$LS/zenoh-sim-bridge.json5" \
      > "$LS/zenoh_bridge.log" 2>&1 < /dev/null &
  echo $! > "$LS/zenoh.pid"
}

# ---- Kill the camera pub BEFORE (re)launching the sim (shm-race guard) -------
echo "==> [0/4] clearing any stale camera pub (shm-race guard)"
tmux kill-session -t campub 2>/dev/null || true
pkill -9 -f "gear_sonic_camera_pub.py" 2>/dev/null || true
sleep 1

echo "==> [1/4] Isaac Sim (Isaac-Flat-G129-Dex3, whole-body dds_lowcmd29)"
"$LS/start_sim.sh" $SIM_ARGS

echo "==> waiting for sim to start stepping (timeout 300s)..."
ready=0
for _ in $(seq 1 300); do
  if grep -qiE "average loop time|\[Performance\]" "$LS/sim_run.log" 2>/dev/null; then
    echo "    sim is stepping."; ready=1; break
  fi
  if ! alive "$LS/sim.pid"; then
    echo "    ERROR: sim exited during startup. Last log lines:"; tail -25 "$LS/sim_run.log"; exit 1
  fi
  sleep 1
done
[ "$ready" = 1 ] || { echo "    ERROR: sim did not reach stepping within timeout."; tail -25 "$LS/sim_run.log"; exit 1; }

echo "==> [2/4] Zenoh DDS bridge"
stop_pidfile "$LS/zenoh.pid"; sleep 1
start_zenoh_bridge

# The peer may start before or after the Spark tunnel/bridge.  Zenoh reconnects
# on its own, and this lightweight supervisor also respawns the local bridge if
# the binary itself exits; neither restart order requires a manual bridge restart.
if ! alive "$LS/bridge_supervisor.pid"; then
  setsid bash "$LS/bridge_supervisor.sh" > "$LS/bridge_supervisor.log" 2>&1 < /dev/null &
  echo $! > "$LS/bridge_supervisor.pid"
fi

echo "==> [3/4] Secondary IMU adapter"
pkill -9 -f "secondary_imu_adapter.py" 2>/dev/null; sleep 1
(
  source "$LS/venv/bin/activate"
  export LD_LIBRARY_PATH="$LS/cyclonedds/install/lib:/usr/local/nvidia/lib64"
  env -u CYCLONEDDS_URI nohup python "$LS/secondary_imu_adapter.py" \
      > "$LS/imu_adapter.log" 2>&1 &
  echo $! > "$LS/imu.pid"
)

echo "==> [4/4] gear_sonic camera publisher"
pkill -9 -f "gear_sonic_camera_pub.py" 2>/dev/null; sleep 1
(
  cd "$LS/unitree_sim_isaaclab"
  source "$LS/venv/bin/activate"
  PYTHONPATH="$LS/unitree_sim_isaaclab" nohup python -u "$LS/gear_sonic_camera_pub.py" --port 5555 --fps 30 \
      > "$LS/campub.log" 2>&1 &
  echo $! > "$LS/campub.pid"
)

sleep 4
echo
echo "===== status ====="
for entry in "Isaac Sim:sim.pid" "Zenoh bridge:zenoh.pid" "IMU adapter:imu.pid" "Camera pub:campub.pid"; do
  name="${entry%%:*}"; pidf="${entry##*:}"
  if alive "$LS/$pidf"; then printf "  OK    %-14s (pid %s)\n" "$name" "$(cat "$LS/$pidf")"
  else printf "  DEAD  %-14s -> see %s\n" "$name" "$LS/${pidf%.pid}*.log"; fi
done
echo
echo "===== single-instance assertion (WS-G) ====="
python3 "$LS/stack_singleton.py" assert || {
  echo "  !! DUPLICATE stack instances detected AFTER bring-up — the stack is in an"
  echo "     inconsistent state (multiple DDS publishers). Run: python3 $LS/stack_singleton.py kill"
  echo "     then re-run this script."; }
echo
echo "Whole-body flat sim up. Drive it from the Spark exactly like start_all.sh:"
echo "  deploy (g1zenoh) -> keyboard -> VLA -> k/i/p.  POD IP: $POD_IP  (DDS :7447 camera :5555)"
