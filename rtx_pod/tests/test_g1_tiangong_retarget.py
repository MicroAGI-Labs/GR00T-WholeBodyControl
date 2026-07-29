from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


POD_SOURCE = Path(__file__).resolve().parents[1] / "unitree_sim_isaaclab"
sys.path.insert(0, str(POD_SOURCE))

from dds.g1_tiangong_retarget import (  # noqa: E402
    G1_CANONICAL_SIGNS,
    TIANGONG_ACTIVE_JOINT_NAMES,
    TIANGONG_HEAD_JOINT_NAMES,
    g1_to_tiangong,
    indices_in,
    is_tiangong_robot,
    tiangong_to_g1,
)


class G1TiangongRetargetTests(unittest.TestCase):
    def test_maps_exactly_29_joints_and_freezes_two_heads(self):
        self.assertEqual(len(TIANGONG_ACTIVE_JOINT_NAMES), 29)
        self.assertEqual(TIANGONG_HEAD_JOINT_NAMES, ("head_yaw_joint", "head_pitch_joint"))
        self.assertTrue(set(TIANGONG_ACTIVE_JOINT_NAMES).isdisjoint(TIANGONG_HEAD_JOINT_NAMES))

    def test_only_elbow_flexion_changes_sign(self):
        self.assertEqual([i for i, sign in enumerate(G1_CANONICAL_SIGNS) if sign < 0], [18, 25])

    def test_round_trip(self):
        values = [float(index) + 0.25 for index in range(29)]
        self.assertEqual(tiangong_to_g1(g1_to_tiangong(values)), values)

    def test_name_mapping_is_not_positional(self):
        names = ["head_yaw_joint", *reversed(TIANGONG_ACTIVE_JOINT_NAMES), "head_pitch_joint"]
        indices = indices_in(names)
        self.assertEqual(indices[0], names.index("hip_pitch_l_joint"))
        self.assertEqual(len(set(indices)), 29)

    def test_odd_ids_are_tiangong_only_when_enabled(self):
        with patch.dict(os.environ, {"SIM_MIXED_TIANGONG": "1"}):
            self.assertEqual([is_tiangong_robot(i) for i in range(8)],
                             [False, True, False, True, False, True, False, True])
        with patch.dict(os.environ, {"SIM_MIXED_TIANGONG": "0"}):
            self.assertFalse(any(is_tiangong_robot(i) for i in range(8)))


if __name__ == "__main__":
    unittest.main()
