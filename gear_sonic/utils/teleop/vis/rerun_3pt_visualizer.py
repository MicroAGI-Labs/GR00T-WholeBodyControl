#!/usr/bin/env python3
"""
Rerun-based 3-point pose visualizer for GEAR-SONIC / GR00T teleoperation.

This is the recommended debug visualizer going forward.

Usage (synthetic test):
    python gear_sonic/scripts/test_vr3pt_synthetic.py --backend rerun

Or programmatically:
    from gear_sonic.utils.teleop.vis.rerun_3pt_visualizer import Rerun3PtVisualizer

    vis = Rerun3PtVisualizer(title="GEAR-SONIC Debug", spawn=True)
    vis.update(vr_3pt_pose)   # (3, 7) array [L-Wrist, R-Wrist, Neck]
"""

from typing import Optional
import numpy as np

try:
    import rerun as rr
except ImportError:
    rr = None


class Rerun3PtVisualizer:
    """
    Clean, modern 3D visualizer using Rerun.

    Logs:
      - Three colored coordinate frames (L-Wrist, R-Wrist, Neck)
      - Three origin spheres for easy tracking
      - A simple torso reference frame
    """

    def __init__(
        self,
        title: str = "GEAR-SONIC 3Pt Teleop",
        spawn: bool = True,
        recording_id: Optional[str] = None,
    ):
        if rr is None:
            raise ImportError(
                "Rerun is not installed. Run: uv pip install rerun-sdk"
            )

        print("[Rerun3PtVisualizer] __init__ called")
        self.title = title
        print("[Rerun3PtVisualizer] About to call rr.init(spawn=True)")
        # Simplest possible call — matches what made the synthetic "just work" earlier
        rr.init(title, recording_id=recording_id, spawn=True)
        print("[Rerun3PtVisualizer] rr.init(spawn=True) returned")

        # Log a world coordinate frame once
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

        # Simple ground plane for orientation
        rr.log(
            "world/ground",
            rr.Boxes3D(
                centers=[[0.0, 0.0, -0.01]],
                half_sizes=[[2.0, 2.0, 0.01]],
            ),
            static=True,
        )

        print(f"[Rerun3PtVisualizer] Initialized.")
        print("  - If the Rerun window didn't appear, use the command printed above.")
        print("  - Camera control and timeline should work well once connected.")

    def update(self, vr_3pt_pose: np.ndarray) -> None:
        """
        Update the three points.

        Args:
            vr_3pt_pose: shape (3, 7)
                Row 0 = L-Wrist  [x, y, z, qw, qx, qy, qz]
                Row 1 = R-Wrist
                Row 2 = Neck
        """
        assert vr_3pt_pose.shape == (3, 7), "Expected (3, 7) array"

        labels = ["L-Wrist", "R-Wrist", "Neck"]
        colors = [[0, 255, 80], [80, 160, 255], [255, 140, 0]]  # green, blue, orange

        for i, (label, color) in enumerate(zip(labels, colors)):
            pos = vr_3pt_pose[i, :3]
            quat_wxyz = vr_3pt_pose[i, 3:]  # Rerun expects wxyz scalar-first

            # Log the transform (position + orientation)
            rr.log(
                f"world/{label}",
                rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=[quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]),
                ),
            )

            # Log a small set of axes at this transform (very useful for orientation)
            rr.log(
                f"world/{label}/axes",
                rr.Arrows3D(
                    origins=[[0, 0, 0]],
                    vectors=[[0.15, 0, 0], [0, 0.15, 0], [0, 0, 0.15]],
                    colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                ),
            )

            # Log a sphere at the origin of the transform for easy visual tracking
            rr.log(
                f"world/{label}/origin",
                rr.Points3D(
                    positions=[[0, 0, 0]],
                    radii=[0.04],
                    colors=[color],
                ),
            )

        # Optional: log a simple torso reference that follows the neck height/orientation
        # (rough approximation — can be improved later with proper FK)
        neck_pos = vr_3pt_pose[2, :3]
        rr.log(
            "world/torso",
            rr.Transform3D(
                translation=[neck_pos[0], neck_pos[1], neck_pos[2] - 0.55],
                rotation=rr.Quaternion(xyzw=[0, 0, 0, 1]),  # identity for now
            ),
        )
        rr.log(
            "world/torso/box",
            rr.Boxes3D(
                centers=[[0, 0, 0]],
                half_sizes=[[0.2, 0.15, 0.25]],
                colors=[[120, 120, 130]],
            ),
        )


# ---------------------------------------------------------------------------
# Convenience for the synthetic test
# ---------------------------------------------------------------------------

def create_rerun_3pt_visualizer(title: str = "GEAR-SONIC 3Pt Teleop (Rerun)") -> Rerun3PtVisualizer:
    return Rerun3PtVisualizer(title=title, spawn=True)


if __name__ == "__main__":
    # Quick manual test
    import time
    import math

    print("Running standalone Rerun 3-point visualizer test...")
    vis = create_rerun_3pt_visualizer()

    t = 0.0
    try:
        while True:
            pose = np.array([
                [-0.3 + 0.25*math.sin(t*1.1), -0.4, 1.0 + 0.1*math.sin(t*1.3), 1,0,0,0],
                [-0.3 + 0.25*math.sin(t*0.9),  0.4, 1.0 + 0.1*math.sin(t*1.1), 1,0,0,0],
                [ 0.0 + 0.05*math.sin(t*0.6),  0.0, 1.55 + 0.05*math.sin(t*0.8), 1,0,0,0],
            ])
            vis.update(pose)
            t += 0.033
            time.sleep(0.033)
    except KeyboardInterrupt:
        print("Closed.")