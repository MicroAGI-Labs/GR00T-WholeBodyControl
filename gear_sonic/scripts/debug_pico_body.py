#!/usr/bin/env python3
"""
Quick diagnostic: is the Pico body tracking data actually changing?

Run:  python gear_sonic/scripts/debug_pico_body.py

Checks:
  1. Can we connect to the XRoboToolkit service?
  2. Is body data available?
  3. Are timestamps advancing?
  4. Are body joint values changing between frames?
  5. What does the full body_poses array look like?
"""
import subprocess
import time
import numpy as np

# Start the robotics service
print("[1/5] Starting robotics service...")
subprocess.Popen(["bash", "/opt/apps/roboticsservice/runService.sh"])
time.sleep(2)

import xrobotoolkit_sdk as xrt

print("[2/5] Initializing XRoboToolkit SDK...")
xrt.init()

print("[3/5] Waiting for body data...")
for i in range(180):
    if xrt.is_body_data_available():
        print(f"  Body data available after {i+1}s")
        break
    if (i + 1) % 10 == 0:
        print(f"  Still waiting... {i+1}s")
    time.sleep(1)
else:
    print("  ERROR: No body data after 180s. Is the Pico headset connected and body tracking enabled?")
    exit(1)

print("[4/5] Reading frames to check if data changes...")
prev_poses = None
prev_stamp = None
frames_same = 0
frames_diff = 0

for i in range(100):
    stamp_ns = xrt.get_time_stamp_ns()
    body_poses = np.array(xrt.get_body_joints_pose())

    if i == 0:
        print(f"\n  body_poses shape: {body_poses.shape}")
        print(f"  body_poses dtype: {body_poses.dtype}")
        print(f"  First timestamp: {stamp_ns}")
        print(f"\n  Full joint dump (frame 0):")
        joint_names = [
            "Root/Hips", "SpineLower", "SpineMiddle", "SpineUpper",  # 0-3
            "Chest", "Neck", "Head", "HeadTip",                      # 4-7 (approx)
            "L-Shoulder", "L-UpperArm", "L-Forearm", "L-Hand",       # 8-11
            "R-Shoulder", "R-UpperArm", "R-Forearm", "R-Hand",       # 12-15 (approx)
            "L-UpperLeg", "L-LowerLeg", "L-Foot", "L-Toes",         # 16-19
            "L-Wrist", "R-Wrist", "R-UpperLeg", "R-LowerLeg",       # 20-23
            "R-Foot", "R-Toes",                                       # 24-25
        ]
        for j in range(min(len(body_poses), 26)):
            name = joint_names[j] if j < len(joint_names) else f"Joint-{j}"
            print(f"    [{j:2d}] {name:20s}: {np.round(body_poses[j], 4)}")
        if len(body_poses) > 26:
            print(f"    ... + {len(body_poses) - 26} more joints")

    if prev_poses is not None:
        if np.allclose(body_poses, prev_poses, atol=1e-6):
            frames_same += 1
        else:
            frames_diff += 1
            if frames_diff <= 3:
                # Show what changed
                diff = np.abs(body_poses - prev_poses)
                max_diff_joint = np.unravel_index(diff.argmax(), diff.shape)
                print(f"\n  Frame {i}: DATA CHANGED! Max delta={diff.max():.6f} at joint {max_diff_joint[0]}")

    if prev_stamp is not None and stamp_ns == prev_stamp:
        pass  # timestamp didn't advance this iteration

    prev_poses = body_poses.copy()
    prev_stamp = stamp_ns
    time.sleep(0.05)  # 20 Hz sampling

print(f"\n[5/5] Summary over 100 frames (~5s):")
print(f"  Frames with SAME data as previous: {frames_same}")
print(f"  Frames with DIFFERENT data:        {frames_diff}")

if frames_diff == 0:
    print("\n  DIAGNOSIS: Body tracking data is completely frozen.")
    print("  Possible causes:")
    print("    - Pico headset not being worn (needs to be on someone's head)")
    print("    - Body tracking not enabled in Pico settings")
    print("    - Pico is in sleep/standby mode")
    print("    - Pico cameras obstructed or in too dark an environment")
    print("    - Streaming app on Pico not running or not connected")
    print("    - SDK returning stale/cached data")
elif frames_diff < 10:
    print("\n  DIAGNOSIS: Data is mostly frozen with occasional updates — tracking may be intermittent.")
else:
    print("\n  DIAGNOSIS: Body tracking is working! Data is updating normally.")
