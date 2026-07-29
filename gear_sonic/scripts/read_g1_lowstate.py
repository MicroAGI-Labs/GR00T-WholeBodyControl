#!/usr/bin/env python3
"""
READ-ONLY pre-flight check: subscribe to the G1's LowState over DDS and print it.

Commands NO motion. Purely passive — proves the full DDS comms path to the robot
before running the actuating deploy.

Usage:
    python gear_sonic/scripts/read_g1_lowstate.py [interface]
    (interface defaults to enP7s7)
"""
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

iface = sys.argv[1] if len(sys.argv) > 1 else "enP7s7"
print(f"[pre-flight] Initializing DDS on interface '{iface}' (domain 0)...")
ChannelFactoryInitialize(0, iface)

latest = {"msg": None, "count": 0}


def on_low_state(msg: LowState_):
    latest["msg"] = msg
    latest["count"] += 1


sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(on_low_state, 10)
print("[pre-flight] Subscribed to rt/lowstate. Waiting up to 10s for robot data...")

t0 = time.time()
while latest["msg"] is None and time.time() - t0 < 10.0:
    time.sleep(0.1)

if latest["msg"] is None:
    print("\n[pre-flight] ✗ NO LowState received in 10s.")
    print("  - Is the robot powered on with its low-level service running?")
    print("  - Is the DDS domain correct (default 0)?")
    print(f"  - Is '{iface}' the cable to the robot?")
    sys.exit(1)

# Got data — print a few samples to confirm it's live and changing
print("\n[pre-flight] ✓ Receiving LowState from robot! Sampling 3x over 1.5s:\n")
for s in range(3):
    m = latest["msg"]
    imu = m.imu_state
    # First 6 joint positions (left + right hip/knee region) as a sanity sample
    q = [round(m.motor_state[i].q, 3) for i in range(6)]
    rpy = [round(v, 3) for v in imu.rpy]
    print(f"  sample {s}: msgs={latest['count']}  imu_rpy={rpy}  joints[0:6]={q}")
    time.sleep(0.5)

print("\n[pre-flight] ✓ DDS comms with the G1 are working (read-only). Safe to proceed.")
