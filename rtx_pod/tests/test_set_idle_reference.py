import importlib.util
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = Path(__file__).parents[1] / "set_idle_reference.py"
SPEC = importlib.util.spec_from_file_location("set_idle_reference", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class SetIdleReferenceTests(unittest.TestCase):
    def test_reference_path_is_per_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(
                MODULE.reference_path(7, root),
                root / "sonic_idle_reference_7.txt",
            )

    def test_write_reference_replaces_complete_value(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = MODULE.reference_path(2, root)
            MODULE.write_reference(path, -4.5, 0.375)
            self.assertEqual(path.read_text(), "-4.5 0.375\n")
            MODULE.write_reference(path, 8.0, 1.0)
            self.assertEqual(path.read_text(), "8 1\n")
            self.assertFalse(list(root.glob(f".{path.name}.*")))


if __name__ == "__main__":
    unittest.main()
