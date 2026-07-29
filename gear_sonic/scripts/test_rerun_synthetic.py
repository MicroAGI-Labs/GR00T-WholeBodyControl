#!/usr/bin/env python3
"""
Minimal standalone synthetic test for the Rerun 3-point visualizer.

This script deliberately avoids importing any modules that pull in Pinocchio
or other NumPy-1.x compiled packages, so it works even if the environment
has NumPy 2 + Rerun.

Usage:
    python gear_sonic/scripts/test_rerun_synthetic.py
"""

import math
import time
import numpy as np

from gear_sonic.utils.teleop.vis.rerun_3pt_visualizer import Rerun3PtVisualizer


def generate_fake_3pt_pose(t: float) -> np.ndarray:
    """Generate plausible moving 3-point pose (L-Wrist, R-Wrist, Neck)."""
    base_l = np.array([-0.25, -0.35, 0.95])
    base_r = np.array([-0.25,  0.35, 0.95])
    base_n = np.array([0.0, 0.0, 1.55])

    amp = 0.22
    freq = 0.75

    lx = base_l[0] + amp * math.sin(freq * t * 1.05)
    ly = base_l[1] + amp * 0.55 * math.sin(freq * t * 0.65 + 1.1)
    lz = base_l[2] + amp * 0.35 * math.sin(freq * t * 1.2 + 0.4)

    rx = base_r[0] + amp * math.sin(freq * t * 0.95 + 2.0)
    ry = base_r[1] + amp * 0.55 * math.sin(freq * t * 1.35 + 0.7)
    rz = base_r[2] + amp * 0.35 * math.sin(freq * t * 1.05 + 1.5)

    nx = base_n[0] + 0.07 * math.sin(freq * t * 0.55)
    ny = base_n[1] + 0.10 * math.sin(freq * t * 0.48 + 0.25)
    nz = base_n[2] + 0.04 * math.sin(freq * t * 0.72)

    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

    return np.array([
        [lx, ly, lz, qw, qx, qy, qz],   # L-Wrist
        [rx, ry, rz, qw, qx, qy, qz],   # R-Wrist
        [nx, ny, nz, qw, qx, qy, qz],   # Neck
    ])


def main():
    print("=" * 70)
    print("FAST SYNTHETIC TEST — Rerun 3-Point Visualizer")
    print("=" * 70)
    print("This mirrors the real usage inside pico_manager_thread_server.py")
    print("when --vis_rerun is passed.")
    print("=" * 70)

    # === Same pattern the real manager will use ===
    enable_vis_rerun = True   # simulates the --vis_rerun flag

    rerun_visualizer = None
    if enable_vis_rerun:
        rerun_visualizer = Rerun3PtVisualizer(
            title="GEAR-SONIC 3Pt Teleop (PICO) — Synthetic",
            spawn=True,
        )

    t = 0.0
    dt = 1.0 / 30.0

    try:
        while True:
            vr_3pt_pose = generate_fake_3pt_pose(t)

            # === Same guarded update the real code does in process_smpl_pose ===
            if rerun_visualizer is not None:
                rerun_visualizer.update(vr_3pt_pose)

            t += dt
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nTest finished.")


if __name__ == "__main__":
    main()