import os
import sys
import unittest


SIM_ROOT = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "unitree_sim_isaaclab"
)
if SIM_ROOT not in sys.path:
    sys.path.insert(0, SIM_ROOT)

from dds.g1_multi_robot_dds import robot_topic


class MultiRobotTopicTests(unittest.TestCase):
    def test_topics_are_unique_and_deterministic(self) -> None:
        topics = {
            robot_topic("rt/sim/g1", robot_id, suffix)
            for robot_id in range(4)
            for suffix in ("lowstate", "lowcmd", "secondary_imu", "reset_pose/cmd")
        }
        self.assertEqual(len(topics), 16)

    def test_unitree_root_is_removed_from_suffix(self) -> None:
        self.assertEqual(
            robot_topic("rt/sim/g1/", 2, "rt/dex3/left/state"),
            "rt/sim/g1/2/dex3/left/state",
        )

    def test_invalid_identity_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            robot_topic("", 0, "lowstate")
        with self.assertRaises(ValueError):
            robot_topic("rt/sim/g1", -1, "lowstate")


if __name__ == "__main__":
    unittest.main()
