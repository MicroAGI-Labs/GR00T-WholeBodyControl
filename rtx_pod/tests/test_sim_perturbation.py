import importlib.util
import json
from pathlib import Path
import unittest


MODULE_PATH = (Path(__file__).parents[1] / "unitree_sim_isaaclab" / "tools" /
               "sim_perturbation.py")
SPEC = importlib.util.spec_from_file_location("sim_perturbation", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ParsePerturbationTest(unittest.TestCase):
    def parse(self, **overrides):
        value = {
            "id": "test-1", "robots": [0], "force_n": [40, 0, 0],
            "torque_nm": [0, 0, 0], "duration_s": 0.15,
        }
        value.update(overrides)
        return MODULE.parse_perturbation(json.dumps(value), robot_count=8)

    def test_targeted_force(self):
        command = self.parse(robots=[2, 5, 2], force_n=[-40, 1, 0])
        self.assertEqual(command.robot_ids, (2, 5))
        self.assertEqual(command.force_n, (-40.0, 1.0, 0.0))

    def test_all_robots(self):
        self.assertEqual(self.parse(robots="all").robot_ids, tuple(range(8)))

    def test_rejects_out_of_range_robot(self):
        with self.assertRaisesRegex(ValueError, "valid range"):
            self.parse(robots=[8])

    def test_rejects_excessive_force(self):
        with self.assertRaisesRegex(ValueError, "exceeds"):
            self.parse(force_n=[501, 0, 0])

    def test_rejects_excessive_duration(self):
        with self.assertRaisesRegex(ValueError, "exceeds"):
            self.parse(duration_s=2.01)

    def test_rejects_zero_wrench(self):
        with self.assertRaisesRegex(ValueError, "both be zero"):
            self.parse(force_n=[0, 0, 0])


if __name__ == "__main__":
    unittest.main()
