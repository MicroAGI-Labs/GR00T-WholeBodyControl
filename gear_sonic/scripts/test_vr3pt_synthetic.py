#!/usr/bin/env python3
"""
Synthetic / isolated test for VR3PtPoseVisualizer.

This script drives the visualizer with fake moving 3-point poses (L-Wrist, R-Wrist, Neck)
so we can debug PyVista rendering / update issues without needing a PICO, roboticsservice,
or the full manager loop.

Usage:
    python gear_sonic/scripts/test_vr3pt_synthetic.py

    # With G1 robot model (default)
    python gear_sonic/scripts/test_vr3pt_synthetic.py --with-g1

    # Just the 3-point markers, no robot
    python gear_sonic/scripts/test_vr3pt_synthetic.py --no-g1

    # Slower / faster animation
    python gear_sonic/scripts/test_vr3pt_synthetic.py --hz 30
"""

import argparse
import math
import time

import numpy as np

from gear_sonic.utils.teleop.vis.vr3pt_pose_visualizer import VR3PtPoseVisualizer
from gear_sonic.utils.teleop.vis.open3d_3pt_visualizer import Open3D3PtVisualizer
from gear_sonic.utils.teleop.vis.rerun_3pt_visualizer import Rerun3PtVisualizer


def generate_synthetic_3pt_pose(t: float) -> np.ndarray:
    """
    Generate a plausible moving 3-point pose (L-Wrist, R-Wrist, Neck).
    Returns shape (3, 7) with [x, y, z, qw, qx, qy, qz]
    """
    # Base positions (roughly in front of a standing person, meters)
    base_l = np.array([-0.25, -0.35, 0.95])
    base_r = np.array([-0.25,  0.35, 0.95])
    base_n = np.array([ 0.0 ,  0.0 , 1.55])

    # Simple swinging motion
    amp = 0.25
    freq = 0.8

    lx = base_l[0] + amp * math.sin(freq * t * 1.1)
    ly = base_l[1] + amp * 0.6 * math.sin(freq * t * 0.7 + 1.2)
    lz = base_l[2] + amp * 0.4 * math.sin(freq * t * 1.3 + 0.5)

    rx = base_r[0] + amp * math.sin(freq * t * 0.9 + 2.1)
    ry = base_r[1] + amp * 0.6 * math.sin(freq * t * 1.4 + 0.8)
    rz = base_r[2] + amp * 0.4 * math.sin(freq * t * 1.1 + 1.6)

    # Neck moves a little (head turning / leaning)
    nx = base_n[0] + 0.08 * math.sin(freq * t * 0.6)
    ny = base_n[1] + 0.12 * math.sin(freq * t * 0.5 + 0.3)
    nz = base_n[2] + 0.05 * math.sin(freq * t * 0.8)

    # Simple identity orientation for all three (we're mostly testing position + G1 FK)
    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

    pose = np.array([
        [lx, ly, lz, qw, qx, qy, qz],   # L-Wrist
        [rx, ry, rz, qw, qx, qy, qz],   # R-Wrist
        [nx, ny, nz, qw, qx, qy, qz],   # Neck
    ])
    return pose


def main():
    parser = argparse.ArgumentParser(description="Synthetic test for VR3PtPoseVisualizer")
    parser.add_argument("--with-g1", action="store_true", default=True,
                        help="Show G1 robot model (default: True)")
    parser.add_argument("--no-g1", dest="with_g1", action="store_false",
                        help="Disable G1 robot model")
    parser.add_argument("--hz", type=float, default=40.0,
                        help="Target update rate in Hz (default 40)")
    parser.add_argument("--duration", type=float, default=120.0,
                        help="How long to run the test (seconds)")
    parser.add_argument("--backend", choices=["pyvista", "open3d", "rerun"], default="rerun",
                        help="Which visualizer backend to use (default: rerun - recommended)")
    args = parser.parse_args()

    print("=" * 70)
    print("SYNTHETIC VR3PT VISUALIZER TEST (no PICO required)")
    print("=" * 70)
    print(f"  with_g1_robot : {args.with_g1}")
    print(f"  target rate   : {args.hz} Hz")
    print(f"  duration      : {args.duration} s")
    print()
    print("You should see the G1 robot + colored wrist/neck markers moving.")
    print("Close the window or press Ctrl-C to exit.")
    print("=" * 70)

    if args.backend == "rerun":
        print("Using Rerun backend (recommended - best interaction + timeline)")
        visualizer = Rerun3PtVisualizer(title="GEAR-SONIC 3Pt Teleop (Rerun)")
        # Rerun runs its own viewer; we just feed data in the loop below
    elif args.backend == "open3d":
        print("Using Open3D backend")
        visualizer = Open3D3PtVisualizer(with_g1_robot=args.with_g1)
        visualizer.create()
        visualizer._synthetic_mode = True
        print("Open3D now owns the window. Close it or Ctrl-C to exit.")
        visualizer.vis.run()
        return
    else:
        print("Using PyVista backend")
        visualizer = VR3PtPoseVisualizer(
            axis_length=0.08,
            ball_radius=0.015,
            with_g1_robot=args.with_g1,
        )
        visualizer.create_realtime_plotter(interactive=True)

    period = 1.0 / args.hz
    start = time.time()
    t = 0.0
    frame = 0

    try:
        while (time.time() - start) < args.duration:
            frame_start = time.time()

            vr_3pt_pose = generate_synthetic_3pt_pose(t)

            if args.backend == "rerun":
                visualizer.update(vr_3pt_pose)
            elif args.backend == "open3d":
                visualizer.update(vr_3pt_pose)
            else:
                visualizer.update_from_vr_pose(vr_3pt_pose)
                visualizer.render()

            # Extra aggressive redraw attempts (common fixes for frozen PyVista on Linux/aarch64)
            try:
                if visualizer.plotter is not None:
                    # Direct VTK render window render (bypasses some PyVista layers)
                    if hasattr(visualizer.plotter, "iren") and visualizer.plotter.iren is not None:
                        rw = visualizer.plotter.iren.GetRenderWindow()
                        if rw is not None:
                            rw.Render()
                    # Also try the interactor render if available
                    if hasattr(visualizer.plotter, "iren") and visualizer.plotter.iren is not None:
                        visualizer.plotter.iren.Render()
            except Exception as e:
                pass  # don't spam if iren not ready

            t += period
            frame += 1

            # simple console heartbeat every 2 seconds
            if frame % int(args.hz * 2) == 0:
                print(f"[{frame:5d}] t={t:6.2f}s  Lwrist=({vr_3pt_pose[0,0]:+.3f},{vr_3pt_pose[0,1]:+.3f},{vr_3pt_pose[0,2]:+.3f})")

            elapsed = time.time() - frame_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    print("Test finished. Close the visualization window if it is still open.")


if __name__ == "__main__":
    main()
