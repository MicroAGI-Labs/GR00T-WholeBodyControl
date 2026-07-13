#!/bin/bash
# Keep the pod Zenoh DDS bridge present across Spark/tunnel/bridge restarts.
# Zenoh handles peer reconnection; this only restarts a dead local binary.
set -uo pipefail
LS="${LIVE_SIM_DIR:-$HOME/live-sim}"
POLL_S="${BRIDGE_SUPERVISOR_POLL_S:-2}"

alive() {
  local pid
  pid="$(cat "$LS/zenoh.pid" 2>/dev/null || true)"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

start_bridge() {
  echo "$(date -Is) [bridge-supervisor] starting zenoh-bridge-dds"
  env -u CYCLONEDDS_URI \
      LD_LIBRARY_PATH=/usr/local/nvidia/lib64 \
      UHLC_MAX_DELTA_MS=2000 \
      setsid "$LS/zenoh-bridge-dds" --config "$LS/zenoh-sim-bridge.json5" \
      >> "$LS/zenoh_bridge.log" 2>&1 < /dev/null &
  echo $! > "$LS/zenoh.pid"
}

echo "$(date -Is) [bridge-supervisor] started (poll=${POLL_S}s)"
while true; do
  alive || start_bridge
  sleep "$POLL_S"
done
