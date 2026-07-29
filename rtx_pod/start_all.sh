#!/bin/bash
# start_all.sh — one-shot bring-up of the live Isaac Sim G1 stack on the RTX pod.
#
# Order:  sim  ->  (wait until it is stepping)  ->  zenoh DDS bridge  ->  IMU adapter  ->  camera publisher
# Idempotent: kills any prior instance of each before starting. PID files + logs live under ~/live-sim.
# Everything here is home-relative, so it works unchanged after an ephemeral pod restart.
#
# ACCESS (no tunnels needed): this pod's own IP (hostname -i, e.g. 10.4.8.23) is on the
# 10.4.0.0/16 pod CIDR, which is advertised into the Cloudflare WARP split-tunnel. Any
# WARP-enrolled machine reaches it directly on ALL ports (TCP+UDP). Connect to the POD IP,
# NOT the per-instance service IP (10.5.x.x) which only exposes :22 for SSH.
#   - DDS (deploy):    <pod-ip>:7447
#   - camera:          <pod-ip>:5555
#   - WebRTC viewport: point the "Isaac Sim WebRTC Streaming Client" at <pod-ip>

set -uo pipefail
LS="$HOME/live-sim"
cd "$LS"

# ---- Sim launch args --------------------------------------------------------
# The WebRTC viewport livestream is ON by default in sim_main.py and auto-advertises
# this host's primary IP as the ICE candidate (--public_ip auto), so no livestream
# flags are needed here. Pass --no_livestream to sim_main.py to turn it off.
# Connect the Isaac Sim WebRTC Streaming Client to the pod IP (hostname -i).
# NOTE: --render_interval 12 gives smooth physics but a choppy (~5-8 Hz) WebRTC
# viewport. For a smooth live view, change it to  --render_interval 1  (the GPU
# is otherwise idle). Physics/RTF are unaffected.
POD_IP="$(hostname -i | awk '{print $1}')"   # for the status footer / where to point the client
SIM_ARGS="--device cpu --headless --enable_cameras --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint --robot_type g129 --enable_dex3_dds --render_interval 12"

alive() { local p; p="$(cat "$1" 2>/dev/null)"; [ -n "$p" ] && kill -0 "$p" 2>/dev/null; }

# ---- Kill the camera pub BEFORE (re)launching the sim -----------------------
# The pub's MultiImageReader attaches to the sim's shared-memory segments; if an
# OLD pub is still alive when the new sim creates fresh shm, killing it later
# (step 4) makes its resource_tracker UNLINK the new sim's segments -> the new
# pub then reads 0 frames / cams=NONE. Killing it here, before the sim exists,
# avoids that race. (Safe: this script's cmdline does not contain the pattern.)
echo "==> [0/4] clearing any stale camera pub (shm-race guard)"
tmux kill-session -t campub 2>/dev/null || true
pkill -9 -f "gear_sonic_camera_pub.py" 2>/dev/null || true
sleep 1

echo "==> [1/4] Isaac Sim"
"$LS/start_sim.sh" $SIM_ARGS      # handles pkill + venv + LD_LIBRARY_PATH + sim.pid + sim_run.log

echo "==> waiting for sim to start stepping (timeout 300s)..."
ready=0
for _ in $(seq 1 300); do
  if grep -qiE "average loop time|\[Performance\]" "$LS/sim_run.log" 2>/dev/null; then
    echo "    sim is stepping."; ready=1; break
  fi
  if ! alive "$LS/sim.pid"; then
    echo "    ERROR: sim process exited during startup. Last log lines:"; tail -25 "$LS/sim_run.log"; exit 1
  fi
  sleep 1
done
[ "$ready" = 1 ] || { echo "    ERROR: sim did not reach stepping within timeout."; tail -25 "$LS/sim_run.log"; exit 1; }

echo "==> [2/4] Zenoh DDS bridge"
pkill -9 -f "zenoh-bridge-dds" 2>/dev/null; sleep 1
env -u CYCLONEDDS_URI \
    LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
    UHLC_MAX_DELTA_MS=2000 \
    nohup "$LS/zenoh-bridge-dds" --config "$LS/zenoh-sim-bridge.json5" \
    > "$LS/zenoh_bridge.log" 2>&1 &
echo $! > "$LS/zenoh.pid"

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
echo "Listening ports (expect: 8011+49100 livestream viewport, 7447 zenoh, 5555 camera, 55555-7 + 60001-3 cam-webrtc):"
ss -ltn 2>/dev/null | grep -E ':(8011|49100|7447|5555|55555|55556|55557|60001|60002|60003)\b' | awk '{print "  "$4}' | sort -t: -k2 -n || echo "  (give it a few seconds and re-check with: ss -ltn)"
echo
echo "Done. Access from any WARP machine at the POD IP:  $POD_IP"
echo "  - WebRTC viewport: Isaac Sim WebRTC Streaming Client -> $POD_IP"
echo "  - DDS $POD_IP:7447   camera $POD_IP:5555   (NOT the 10.5.x service IP; that is SSH-only)"
