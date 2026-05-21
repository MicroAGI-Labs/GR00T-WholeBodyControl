#!/usr/bin/env bash
# ============================================================================
# Zenoh <-> DDS bridge — ROBOT (Orin/Jetson) side.
#
# Run this ON the G1's onboard Orin (eth0 = robot network, 192.168.123.x).
# It bridges the robot's DDS topics over a Zenoh TCP link so the off-board
# Spark can reach them. This is required because the G1's internal switch does
# IGMP snooping and the robot's inbound multicast (rt/lowstate) never reaches an
# external host — see ./README.md.
#
# Pairs with the Spark side started by gear_sonic_deploy/deploy.sh (g1zenoh mode).
#
# Usage (on the Orin):   ./start_bridge_orin.sh
# Or from the Spark:     ssh g1 'bash -s' < gear_sonic_deploy/zenoh/start_bridge_orin.sh
# ============================================================================
set -e

ROBOT_IFACE="${ROBOT_IFACE:-eth0}"          # robot network interface on the Orin
ZENOH_PORT="${ZENOH_PORT:-7447}"            # port the Spark connects to
CFG_DIR="${CFG_DIR:-$HOME/zenoh-dds}"

mkdir -p "$CFG_DIR"

# Zenoh config: fixed listen port, no multicast scouting (won't cross the robot
# switch), bridge exactly the topics the deploy uses.
cat > "$CFG_DIR/config.json5" <<JSON5
{
  mode: "peer",
  listen: { endpoints: ["tcp/0.0.0.0:${ZENOH_PORT}"] },
  scouting: { multicast: { enabled: false } },
  plugins: {
    dds: {
      domain: 0,
      allow: ["rt/lowstate", "rt/lowcmd", "rt/secondary_imu"]
    }
  }
}
JSON5

# CycloneDDS: pin the robot interface (NOT wlan0). The plugin reads the NIC from
# CYCLONEDDS_URI; plugins.dds.general.network_interface is NOT a valid field.
cat > "$CFG_DIR/cyclonedds.xml" <<XML
<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="${ROBOT_IFACE}" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>
XML

# NOTE: the config files MUST exist before `docker run`, otherwise Docker creates
# the bind-mount source paths as directories and the bridge crash-loops
# ("Is a directory"). That's why we write them above first.
docker rm -f zenoh-dds-bridge 2>/dev/null || true
docker run -d --restart unless-stopped \
  --name zenoh-dds-bridge \
  --network host \
  -v "$CFG_DIR/config.json5:/config.json5:ro" \
  -v "$CFG_DIR/cyclonedds.xml:/cyclonedds.xml:ro" \
  -e CYCLONEDDS_URI=/cyclonedds.xml \
  eclipse/zenoh-bridge-dds:latest \
  --config /config.json5

echo "[start_bridge_orin] launched on '${ROBOT_IFACE}', listening tcp/:${ZENOH_PORT}. Verifying..."
sleep 5
docker ps --filter name=zenoh-dds-bridge --format "  {{.Names}}: {{.Status}}"
docker logs --tail 30 zenoh-dds-bridge 2>&1 | sed -E 's/\x1b\[[0-9;]*m//g' | grep -i "Route DDS->Zenoh" | head -3
ss -tln 2>/dev/null | grep -q ":${ZENOH_PORT} " \
  && echo "[start_bridge_orin] listening on ${ZENOH_PORT} OK" \
  || echo "[start_bridge_orin] WARN: not listening on ${ZENOH_PORT}"
