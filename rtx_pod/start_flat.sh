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
SIM_ROBOT_COUNT="${SIM_ROBOT_COUNT:-1}"
case "$SIM_ROBOT_COUNT" in
  ''|*[!0-9]*) echo "SIM_ROBOT_COUNT must be an integer" >&2; exit 2 ;;
esac
if [ "$SIM_ROBOT_COUNT" -lt 1 ] || [ "$SIM_ROBOT_COUNT" -gt 24 ]; then
  echo "SIM_ROBOT_COUNT must be between 1 and 24" >&2
  exit 2
fi
export SIM_ROBOT_COUNT

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
DDS_INTERFACE="${SIM_DDS_INTERFACE:-$(ip -4 route show default | awk 'NR==1 {print $5}')}"
DDS_INTERFACE="${DDS_INTERFACE:-eth0}"
DDS_CONFIG="$LS/cyclonedds_rtx.xml"
cat > "$DDS_CONFIG" <<XML
<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="$DDS_INTERFACE" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>
XML
echo "    RTX DDS bridge interface: $DDS_INTERFACE"
SIM_ARGS="--device cuda --headless --enable_cameras --task Isaac-Flat-G129-Dex3 --robot_type g129 --action_source dds_lowcmd29 --render_interval 12 --num_envs $SIM_ROBOT_COUNT"
if [ "$SIM_ROBOT_COUNT" -eq 1 ]; then
  # The legacy single-robot workflow has a matching Dex3 DDS endpoint.  Hands
  # are deliberately disabled in the concurrent body-control MVP until they
  # have per-environment state and command routing.
  SIM_ARGS="$SIM_ARGS --enable_dex3_dds"
fi

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
  env CYCLONEDDS_URI="$DDS_CONFIG" \
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
rm -f "$LS/imu.pid"
if [ "$SIM_ROBOT_COUNT" -eq 1 ]; then
  (
    source "$LS/venv/bin/activate"
    export LD_LIBRARY_PATH="$LS/cyclonedds/install/lib:/usr/local/nvidia/lib64"
    env -u CYCLONEDDS_URI nohup python "$LS/secondary_imu_adapter.py" \
        > "$LS/imu_adapter.log" 2>&1 &
    echo $! > "$LS/imu.pid"
  )
else
  echo "    skipped: multi-robot DDS publishes namespaced secondary IMUs directly"
fi

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
status_entries=("Isaac Sim:sim.pid" "Zenoh bridge:zenoh.pid" "Camera pub:campub.pid")
[ "$SIM_ROBOT_COUNT" -eq 1 ] && status_entries+=("IMU adapter:imu.pid")
for entry in "${status_entries[@]}"; do
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
echo "Whole-body flat sim up with $SIM_ROBOT_COUNT robot(s)."
echo "  deploy (g1zenoh) -> keyboard -> VLA -> k/i/p.  POD IP: $POD_IP  (DDS :7447 camera :5555)"
