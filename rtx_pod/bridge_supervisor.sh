#!/bin/bash
# Keep the pod Zenoh DDS bridge present across Spark/tunnel/bridge restarts.
# Zenoh handles peer reconnection; this only restarts a dead local binary.
set -uo pipefail
LS="${LIVE_SIM_DIR:-$HOME/live-sim}"
POLL_S="${BRIDGE_SUPERVISOR_POLL_S:-2}"
DDS_INTERFACE="${SIM_DDS_INTERFACE:-$(ip -4 route show default | awk 'NR==1 {print $5}')}"
DDS_INTERFACE="${DDS_INTERFACE:-eth0}"
DDS_CONFIG="$LS/cyclonedds_rtx.xml"

alive() {
  local pid
  pid="$(cat "$LS/zenoh.pid" 2>/dev/null || true)"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

start_bridge() {
  echo "$(date -Is) [bridge-supervisor] starting zenoh-bridge-dds"
  if [ ! -s "$DDS_CONFIG" ]; then
    printf '%s\n' "<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$DDS_INTERFACE\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>" > "$DDS_CONFIG"
  fi
  env CYCLONEDDS_URI="$DDS_CONFIG" \
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
