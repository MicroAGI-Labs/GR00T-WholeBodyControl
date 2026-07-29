#!/usr/bin/env python3
"""
Open3D-based real-time visualizer for the 3-point VR pose (L-Wrist, R-Wrist, Neck)
used in GEAR-SONIC / GR00T teleoperation.

This is intended as a more reliable alternative to the PyVista version on
certain Linux/aarch64 platforms (e.g. DGX Spark) where live PyVista updates
can be problematic.

Usage (synthetic test):
    python gear_sonic/scripts/test_vr3pt_synthetic.py --backend open3d

Or directly:
    from gear_sonic.utils.teleop.vis.open3d_3pt_visualizer import Open3D3PtVisualizer
    vis = Open3D3PtVisualizer(with_g1_robot=True)
    vis.create()
    ...
    vis.update(vr_3pt_pose)   # (3, 7) array
    vis.spin_once()
"""

from pathlib import Path
from typing import Optional
import time

import numpy as np
import open3d as o3d


class Open3D3PtVisualizer:
    """
    Real-time visualizer using Open3D.

    Shows:
      - 3 RGB coordinate frames for L-Wrist, R-Wrist, Neck
      - Optional simple G1 reference frame (torso + base)
      - Ground plane + axis

    The update() method is designed to be called at 30-90 Hz from an external loop.
    """

    def __init__(
        self,
        with_g1_robot: bool = True,
        window_name: str = "GEAR-SONIC 3Pt Teleop (Open3D)",
        window_width: int = 1400,
        window_height: int = 900,
    ):
        self.with_g1_robot = with_g1_robot
        self.window_name = window_name
        self.window_width = window_width
        self.window_height = window_height

        self.vis: Optional[o3d.visualization.Visualizer] = None
        self.created = False

        # The three moving frames
        self.lwrist_frame: Optional[o3d.geometry.TriangleMesh] = None
        self.rwrist_frame: Optional[o3d.geometry.TriangleMesh] = None
        self.neck_frame: Optional[o3d.geometry.TriangleMesh] = None

        # Keep original vertex data so we can reset every frame (avoids transform accumulation)
        self._lwrist_orig: Optional[np.ndarray] = None
        self._rwrist_orig: Optional[np.ndarray] = None
        self._neck_orig: Optional[np.ndarray] = None

        # Simple G1 reference geometry
        self.g1_base_frame: Optional[o3d.geometry.TriangleMesh] = None
        self.g1_torso_frame: Optional[o3d.geometry.TriangleMesh] = None

        self._last_poses: Optional[np.ndarray] = None
        self.latest_pose = None
        self._synthetic_mode = False

    def _make_colored_axis(self, pose, size=0.18, color=None):
        """Create a fresh coordinate frame at the given pose."""
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
        if color is not None:
            # Tint the axes
            frame.paint_uniform_color(color)
        self._apply_pose_to_mesh(frame, pose)
        return frame

    def _make_colored_sphere(self, pose, radius=0.04, color=None):
        """Create a fresh sphere at the given [x,y,z]."""
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        if color is not None:
            sphere.paint_uniform_color(color)
        self._apply_pose_to_mesh(sphere, pose, apply_rotation=False)
        return sphere

    def _apply_pose_to_mesh(self, mesh, pose, apply_rotation=True):
        """Apply position (+ optional rotation) to a mesh that was created at origin."""
        x, y, z, qw, qx, qy, qz = pose
        if apply_rotation:
            quat_xyzw = np.array([qx, qy, qz, qw])
            R = o3d.geometry.get_rotation_matrix_from_quaternion(quat_xyzw)
            mesh.rotate(R, center=(0, 0, 0))
        mesh.translate([x, y, z])

    def _apply_pose_to_current_geometry(self, pose):
        """Apply the latest pose to the already-created geometry objects."""
        if self.lwrist_frame is not None:
            self._set_frame_pose(self.lwrist_frame, pose[0], orig_vertices=self._lwrist_orig)
            self.vis.update_geometry(self.lwrist_frame)
        if self.rwrist_frame is not None:
            self._set_frame_pose(self.rwrist_frame, pose[1], orig_vertices=self._rwrist_orig)
            self.vis.update_geometry(self.rwrist_frame)
        if self.neck_frame is not None:
            self._set_frame_pose(self.neck_frame, pose[2], orig_vertices=self._neck_orig)
            self.vis.update_geometry(self.neck_frame)

        if self.lwrist_sphere is not None:
            self._set_sphere_position(self.lwrist_sphere, pose[0], self._lwrist_sphere_orig)
            self.vis.update_geometry(self.lwrist_sphere)
        if self.rwrist_sphere is not None:
            self._set_sphere_position(self.rwrist_sphere, pose[1], self._rwrist_sphere_orig)
            self.vis.update_geometry(self.rwrist_sphere)
        if self.neck_sphere is not None:
            self._set_sphere_position(self.neck_sphere, pose[2], self._neck_sphere_orig)
            self.vis.update_geometry(self.neck_sphere)

    def create(self) -> None:
        """Create the Open3D window and add static + dynamic geometry."""
        if self.created:
            return

        # Use VisualizerWithKeyCallback so we can have animation + full mouse interaction
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(
            window_name=self.window_name,
            width=self.window_width,
            height=self.window_height,
        )

        # Set a nice dark background
        opt = self.vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.12])
        opt.point_size = 3.0

        # Add ground grid (simple large plane)
        ground = o3d.geometry.TriangleMesh.create_box(width=4, height=4, depth=0.01)
        ground.translate([-2.0, -2.0, -0.01])
        ground.paint_uniform_color([0.15, 0.15, 0.18])
        self.vis.add_geometry(ground)

        # World coordinate frame at origin
        world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4)
        self.vis.add_geometry(world_frame)

        # Create the three moving RGB frames (will be transformed every update)
        self.lwrist_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.18)
        self.rwrist_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.18)
        self.neck_frame   = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.18)

        # Store original vertices for fast reset every frame
        self._lwrist_orig = np.asarray(self.lwrist_frame.vertices)
        self._rwrist_orig = np.asarray(self.rwrist_frame.vertices)
        self._neck_orig   = np.asarray(self.neck_frame.vertices)

        self.vis.add_geometry(self.lwrist_frame)
        self.vis.add_geometry(self.rwrist_frame)
        self.vis.add_geometry(self.neck_frame)

        # Big obvious colored spheres at the three points (much easier to track)
        self.lwrist_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.10)
        self.lwrist_sphere.paint_uniform_color([0.0, 1.0, 0.3])   # bright green
        self.rwrist_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.10)
        self.rwrist_sphere.paint_uniform_color([0.3, 0.6, 1.0])   # bright blue
        self.neck_sphere   = o3d.geometry.TriangleMesh.create_sphere(radius=0.10)
        self.neck_sphere.paint_uniform_color([1.0, 0.5, 0.0])     # bright orange

        # Start the spheres in a visible area near the pelvis box so they're obvious from frame 1
        self.lwrist_sphere.translate([-0.3, -0.3, 0.9])
        self.rwrist_sphere.translate([-0.3,  0.3, 0.9])
        self.neck_sphere.translate([0.0, 0.0, 1.4])

        self.vis.add_geometry(self.lwrist_sphere)
        self.vis.add_geometry(self.rwrist_sphere)
        self.vis.add_geometry(self.neck_sphere)

        # Store original sphere vertices (after we moved them to a visible starting spot)
        self._lwrist_sphere_orig = np.asarray(self.lwrist_sphere.vertices)
        self._rwrist_sphere_orig = np.asarray(self.rwrist_sphere.vertices)
        self._neck_sphere_orig   = np.asarray(self.neck_sphere.vertices)

        # Simple G1 reference (base + torso)
        if self.with_g1_robot:
            self.g1_base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25)
            self.g1_torso_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.20)

            # Move torso frame up a bit (rough)
            self.g1_torso_frame.translate([0, 0, 0.9])

            self.vis.add_geometry(self.g1_base_frame)
            self.vis.add_geometry(self.g1_torso_frame)

            # Add a simple "pelvis box" so it looks more like a robot
            pelvis = o3d.geometry.TriangleMesh.create_box(width=0.25, height=0.18, depth=0.12)
            pelvis.translate([-0.125, -0.09, 0.75])
            pelvis.paint_uniform_color([0.3, 0.3, 0.35])
            self.vis.add_geometry(pelvis)

        # Initial camera position (similar to the PyVista one)
        self.vis.get_view_control().set_front([1.2, -1.0, 0.8])
        self.vis.get_view_control().set_lookat([0.0, 0.0, 1.0])
        self.vis.get_view_control().set_up([0, 0, 1])

        self.created = True

        # Register animation callback — this is the proper Open3D way
        # to have live updates while keeping full mouse/keyboard control.
        self.vis.register_animation_callback(self._animate)

        print("[Open3D3PtVisualizer] Window created. Use mouse to rotate/pan.")

    def update(self, vr_3pt_pose: np.ndarray) -> None:
        """
        Update the positions/orientations of the three points + spheres.
        This version mutates the existing geometry objects (much friendlier to mouse interaction).
        """
        if not self.created:
            self.create()

        assert vr_3pt_pose.shape == (3, 7), "Expected shape (3, 7)"

        # Update the three moving coordinate frames
        self._set_frame_pose(self.lwrist_frame, vr_3pt_pose[0], orig_vertices=self._lwrist_orig)
        self._set_frame_pose(self.rwrist_frame, vr_3pt_pose[1], orig_vertices=self._rwrist_orig)
        self._set_frame_pose(self.neck_frame,   vr_3pt_pose[2], orig_vertices=self._neck_orig)

        # Update the colored spheres at the origins
        self._set_sphere_position(self.lwrist_sphere, vr_3pt_pose[0], self._lwrist_sphere_orig)
        self._set_sphere_position(self.rwrist_sphere, vr_3pt_pose[1], self._rwrist_sphere_orig)
        self._set_sphere_position(self.neck_sphere,   vr_3pt_pose[2], self._neck_sphere_orig)

        # Tell Open3D these objects changed
        self.vis.update_geometry(self.lwrist_frame)
        self.vis.update_geometry(self.rwrist_frame)
        self.vis.update_geometry(self.neck_frame)
        self.vis.update_geometry(self.lwrist_sphere)
        self.vis.update_geometry(self.rwrist_sphere)
        self.vis.update_geometry(self.neck_sphere)

        # We no longer call poll_events() + update_renderer() here.
        # The animation callback (registered in create) will do it at the right time,
        # which preserves mouse interaction.

    def _set_frame_pose(self, mesh: o3d.geometry.TriangleMesh, pose: np.ndarray, scale: float = 1.0, orig_vertices: Optional[np.ndarray] = None):
        """Reset + apply new position + orientation to a coordinate frame."""
        x, y, z, qw, qx, qy, qz = pose

        quat_xyzw = np.array([qx, qy, qz, qw])
        R = o3d.geometry.get_rotation_matrix_from_quaternion(quat_xyzw)

        if orig_vertices is None:
            orig_vertices = np.asarray(mesh.vertices)

        # Apply rotation + scale + translation directly to vertices (reliable, no accumulation)
        new_verts = (orig_vertices @ R.T) * scale + np.array([x, y, z])
        mesh.vertices = o3d.utility.Vector3dVector(new_verts)
        mesh.compute_vertex_normals()  # required for proper visual update in many cases

    def _set_sphere_position(self, sphere: o3d.geometry.TriangleMesh, pose: np.ndarray, orig_vertices: np.ndarray):
        """Move a sphere to the given [x,y,z] position (ignore rotation)."""
        x, y, z = pose[:3]
        new_verts = orig_vertices + np.array([x, y, z])
        sphere.vertices = o3d.utility.Vector3dVector(new_verts)
        sphere.compute_vertex_normals()

        # Better approach: store original and always start from identity
        # For simplicity in first version we reset scale/rotation each time
        # (we recreate the transform every call)

    def spin_once(self, timeout: float = 0.001) -> bool:
        """
        Process events and redraw. Call this every frame.
        Returns False if the window was closed.
        """
        if self.vis is None:
            return False

        self.vis.poll_events()
        self.vis.update_renderer()
        return True

    def _animate(self, vis):
        """
        Animation callback — Open3D calls this regularly.
        For the synthetic test we generate nice waving motion here
        by recreating the moving objects every frame (reliable update).
        """
        if hasattr(self, "_synthetic_mode") and self._synthetic_mode:
            import time
            t = time.time()
            pose = np.array([
                [-0.3 + 0.25*np.sin(t*1.1), -0.4 + 0.15*np.sin(t*0.7), 1.0 + 0.1*np.sin(t*1.3), 1,0,0,0],
                [-0.3 + 0.25*np.sin(t*0.9 + 1),  0.4 + 0.15*np.sin(t*1.2), 1.0 + 0.1*np.sin(t*0.8), 1,0,0,0],
                [0.0 + 0.08*np.sin(t*0.6), 0.0 + 0.1*np.sin(t*0.5), 1.55 + 0.05*np.sin(t*0.9), 1,0,0,0],
            ])

            # Remove old moving objects (this path made the spheres visible)
            for name in ['lwrist_frame','rwrist_frame','neck_frame',
                         'lwrist_sphere','rwrist_sphere','neck_sphere']:
                g = getattr(self, name, None)
                if g is not None:
                    try: vis.remove_geometry(g)
                    except: pass

            # Recreate fresh every frame (visible spheres)
            self.lwrist_frame = self._make_colored_axis(pose[0], size=0.18, color=[0.0,1.0,0.3])
            self.rwrist_frame = self._make_colored_axis(pose[1], size=0.18, color=[0.3,0.6,1.0])
            self.neck_frame   = self._make_colored_axis(pose[2], size=0.18, color=[1.0,0.5,0.0])

            self.lwrist_sphere = self._make_colored_sphere(pose[0], radius=0.10, color=[0.0,1.0,0.3])
            self.rwrist_sphere = self._make_colored_sphere(pose[1], radius=0.10, color=[0.3,0.6,1.0])
            self.neck_sphere   = self._make_colored_sphere(pose[2], radius=0.10, color=[1.0,0.5,0.0])

            vis.add_geometry(self.lwrist_frame)
            vis.add_geometry(self.rwrist_frame)
            vis.add_geometry(self.neck_frame)
            vis.add_geometry(self.lwrist_sphere)
            vis.add_geometry(self.rwrist_sphere)
            vis.add_geometry(self.neck_sphere)
        else:
            if self.latest_pose is not None:
                self._apply_pose_to_current_geometry(self.latest_pose)

        vis.poll_events()
        vis.update_renderer()
        return True

    def set_pose(self, vr_3pt_pose: np.ndarray):
        """Thread-safe-ish way for external code to feed poses (used by real manager)."""
        self.latest_pose = vr_3pt_pose.copy()

    def close(self):
        if self.vis is not None:
            self.vis.destroy_window()
            self.vis = None
            self.created = False


# ---------------------------------------------------------------------------
# Convenience function for the synthetic test
# ---------------------------------------------------------------------------

def create_open3d_3pt_visualizer(with_g1: bool = True) -> Open3D3PtVisualizer:
    vis = Open3D3PtVisualizer(with_g1_robot=with_g1)
    vis.create()
    return vis


if __name__ == "__main__":
    # Quick manual test
    print("Running standalone Open3D 3-point visualizer test...")
    import time

    vis = create_open3d_3pt_visualizer(with_g1=True)

    t = 0.0
    try:
        while True:
            # Fake moving pose
            pose = np.array([
                [-0.3 + 0.2*np.sin(t*1.1), -0.4, 1.0 + 0.1*np.sin(t*1.3), 1, 0, 0, 0],  # L
                [-0.3 + 0.2*np.sin(t*0.9),  0.4, 1.0 + 0.1*np.sin(t*1.1), 1, 0, 0, 0],  # R
                [ 0.0 + 0.05*np.sin(t*0.6), 0.0, 1.55 + 0.05*np.sin(t*0.8), 1, 0, 0, 0], # Neck
            ])
            vis.update(pose)
            vis.spin_once()
            t += 0.033
            time.sleep(0.033)
    except KeyboardInterrupt:
        vis.close()
        print("Closed.")