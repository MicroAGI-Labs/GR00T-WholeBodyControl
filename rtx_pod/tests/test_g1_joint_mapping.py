from __future__ import annotations

import sys
import unittest
from pathlib import Path


POD_SOURCE = Path(__file__).resolve().parents[1] / "unitree_sim_isaaclab"
sys.path.insert(0, str(POD_SOURCE))

from dds.g1_joint_mapping import (  # noqa: E402
    UNITREE_FROM_ISAAC_INDICES,
    UNITREE_G1_29_DEFAULT_POSITIONS,
    UNITREE_G1_29_EFFORT_LIMITS,
    UNITREE_G1_EFFORT_LIMIT_BY_NAME,
    UNITREE_G1_29_JOINT_NAMES,
    isaac_to_unitree,
    unitree_indices_in,
    unitree_to_isaac,
)


def isaac_joint_names_with_hands() -> list[str]:
    """Build the flat-task articulation order plus 14 non-body hand joints."""

    names = [""] * len(UNITREE_G1_29_JOINT_NAMES)
    for unitree_index, isaac_index in enumerate(UNITREE_FROM_ISAAC_INDICES):
        names[isaac_index] = UNITREE_G1_29_JOINT_NAMES[unitree_index]
    names.extend(f"hand_joint_{index}" for index in range(14))
    return names


class TestG1JointMapping(unittest.TestCase):
    def setUp(self) -> None:
        self.isaac_names = isaac_joint_names_with_hands()
        self.unitree_values = [1000.0 + index for index in range(29)]

    def test_flat_task_permutation_is_complete_and_non_identity(self) -> None:
        self.assertEqual(sorted(UNITREE_FROM_ISAAC_INDICES), list(range(29)))
        self.assertNotEqual(UNITREE_FROM_ISAAC_INDICES, tuple(range(29)))
        self.assertEqual(
            unitree_indices_in(self.isaac_names), UNITREE_FROM_ISAAC_INDICES
        )

    def test_sonic_default_pose_covers_every_hardware_joint(self) -> None:
        self.assertEqual(len(UNITREE_G1_29_DEFAULT_POSITIONS), 29)
        pose = dict(zip(UNITREE_G1_29_JOINT_NAMES, UNITREE_G1_29_DEFAULT_POSITIONS))
        self.assertEqual(pose["left_knee_joint"], 0.669)
        self.assertEqual(pose["right_ankle_pitch_joint"], -0.363)

    def test_sonic_effort_contract_covers_every_hardware_joint(self) -> None:
        self.assertEqual(len(UNITREE_G1_29_EFFORT_LIMITS), 29)
        self.assertEqual(set(UNITREE_G1_EFFORT_LIMIT_BY_NAME), set(UNITREE_G1_29_JOINT_NAMES))
        self.assertEqual(UNITREE_G1_EFFORT_LIMIT_BY_NAME["left_hip_pitch_joint"], 139.0)
        self.assertEqual(UNITREE_G1_EFFORT_LIMIT_BY_NAME["right_hip_pitch_joint"], 139.0)
        self.assertEqual(UNITREE_G1_EFFORT_LIMIT_BY_NAME["left_hip_yaw_joint"], 88.0)
        self.assertEqual(UNITREE_G1_EFFORT_LIMIT_BY_NAME["left_ankle_pitch_joint"], 25.0)

    def test_control_maps_every_lowcmd_motor_to_named_isaac_joint(self) -> None:
        existing = [-5000.0 - index for index in range(len(self.isaac_names))]
        mapped = unitree_to_isaac(
            self.unitree_values, self.isaac_names, existing_isaac_values=existing
        )

        for unitree_index, isaac_index in enumerate(UNITREE_FROM_ISAAC_INDICES):
            self.assertEqual(mapped[isaac_index], self.unitree_values[unitree_index])
        self.assertEqual(mapped[29:], existing[29:], "hand joints must be preserved")

    def test_state_maps_every_isaac_joint_to_lowstate_motor_order(self) -> None:
        isaac_values = [-1.0] * len(self.isaac_names)
        for unitree_index, isaac_index in enumerate(UNITREE_FROM_ISAAC_INDICES):
            isaac_values[isaac_index] = self.unitree_values[unitree_index]

        self.assertEqual(
            isaac_to_unitree(isaac_values, self.isaac_names), self.unitree_values
        )

    def test_bidirectional_round_trip_for_all_lowcmd_and_lowstate_fields(self) -> None:
        fields = {
            "position": [0.01 * index for index in range(29)],
            "velocity": [10.0 + index for index in range(29)],
            "torque": [20.0 + index for index in range(29)],
            "kp": [30.0 + index for index in range(29)],
            "kd": [40.0 + index for index in range(29)],
        }

        for field_name, unitree_values in fields.items():
            with self.subTest(field=field_name):
                isaac_values = unitree_to_isaac(
                    unitree_values,
                    self.isaac_names,
                    existing_isaac_values=[-1.0] * len(self.isaac_names),
                )
                self.assertEqual(
                    isaac_to_unitree(isaac_values, self.isaac_names), unitree_values
                )

    def test_raw_isaac_prefix_is_not_valid_lowstate_order(self) -> None:
        isaac_values = list(range(len(self.isaac_names)))
        correctly_mapped = isaac_to_unitree(isaac_values, self.isaac_names)

        self.assertNotEqual(isaac_values[:29], correctly_mapped)
        self.assertEqual(correctly_mapped, list(UNITREE_FROM_ISAAC_INDICES))

    def test_missing_or_duplicate_joint_names_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "missing G1 body joints"):
            unitree_indices_in(self.isaac_names[:-15])

        duplicate_names = list(self.isaac_names)
        duplicate_names[-1] = duplicate_names[0]
        with self.assertRaisesRegex(ValueError, "duplicate joint names"):
            unitree_indices_in(duplicate_names)

    def test_wrong_vector_lengths_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "Expected 29"):
            unitree_to_isaac(self.unitree_values[:-1], self.isaac_names)
        with self.assertRaisesRegex(ValueError, "length mismatch"):
            isaac_to_unitree([0.0] * 29, self.isaac_names)


if __name__ == "__main__":
    unittest.main()
