# Zenoh DDS bridge: off-board Spark ↔ G1

Lets an off-board machine (the DGX Spark) run the GR00T deploy and drive the real
G1 over DDS, when the robot's DDS is **not directly reachable by multicast**.

## Why this exists

The G1 has an internal Ethernet switch connecting its control board
(`192.168.123.161`, publishes `rt/lowstate` / reads `rt/lowcmd`), the onboard Orin
(`192.168.123.164`), and the external port you plug into. That switch does **IGMP
snooping**, and Linux defaults to **IGMPv3**, so:

- Spark → robot multicast works (our packets reach the robot).
- robot → Spark multicast is **dropped** (we never receive `rt/lowstate`).

DDS discovery needs both directions, so a direct `./deploy.sh real` over the cable
sees nothing. Diagnostic proof: an SPDP listener **on the Orin** sees `.161`, `.164`
and the Spark; the same listener on the Spark sees nothing from the robot.

## The fix

Run `eclipse/zenoh-bridge-dds` on **both** ends and tunnel DDS over a **Zenoh TCP**
link on the reliable wired path (no multicast across the switch):

```
 robot .161  --DDS(eth0)-->  Orin bridge  --Zenoh/TCP 7447-->  Spark bridge  --DDS(lo)-->  deploy
            <----------------------------- rt/lowcmd ------------------------------------
```

Topics bridged: `rt/lowstate`, `rt/lowcmd`, `rt/secondary_imu`.

## Run it

**Orin side (on the robot):**
```bash
./start_bridge_orin.sh          # writes config + starts the container (eth0, listen :7447)
```
Auto-restarts on reboot (`docker run --restart unless-stopped`).

**Spark side:** started automatically by the deploy's `g1zenoh` mode —
```bash
DEPLOY_YES=1 ./deploy.sh --input-type zmq_manager --output-type log g1zenoh
```
`g1zenoh` runs the deploy's DDS on `lo` (real-robot behavior) and launches the Spark
bridge connecting to `tcp/192.168.123.164:7447`. The Spark configs here
(`spark_cyclonedds.xml`, `spark_config.json5`) are reference copies of what deploy.sh
generates in `$HOME`.

## One-time Spark host setup

DDS on loopback needs multicast enabled on `lo`:
```bash
sudo ip link set lo multicast on
sudo ip route add 239.255.0.0/16 dev lo
```

## Gotchas (learned the hard way)

- Config files must **exist before `docker run`** or Docker creates them as
  directories → crash loop (`Is a directory`). The scripts write them first.
- Default zenoh `peer` mode listens on an **ephemeral** port — set an explicit
  `listen tcp/0.0.0.0:7447`.
- Disable zenoh **multicast scouting**; use explicit `connect` (scouting won't cross
  the robot switch either).
- Pin the NIC via `CYCLONEDDS_URI`; `plugins.dds.general.network_interface` is invalid.

## TODO

- [ ] **Verify LowCmd CRC across the Zenoh round-trip.** `g1zenoh` currently passes
      `--disable-crc-check` (disables the deploy's *incoming* LowState CRC check) on the
      assumption the Zenoh re-serialization may alter bytes. Confirm whether the robot
      accepts the round-tripped `rt/lowcmd` CRC, and whether the incoming check can be
      safely re-enabled. (Teleop works with it disabled; this is a safety hardening item.)
