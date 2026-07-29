#!/usr/bin/env python3
"""Set one running SONIC controller's live stationary-IDLE reference.

The matching controller must be launched with, for example,
``SONIC_IDLE_REFERENCE_FILE=/tmp/sonic_idle_reference_3.txt``.
"""

import argparse
import os
import tempfile
from pathlib import Path


def reference_path(robot_id: int, directory: Path) -> Path:
    return directory / f"sonic_idle_reference_{robot_id}.txt"


def write_reference(path: Path, pitch_degrees: float, leg_blend: float) -> None:
    """Atomically publish a complete reference so the controller never reads half a write."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            output.write(f"{pitch_degrees:.9g} {leg_blend:.9g}\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_id", type=int, choices=range(24), metavar="ROBOT_ID")
    parser.add_argument("pitch_degrees", type=float, metavar="PITCH_DEG")
    parser.add_argument("leg_blend", type=float, metavar="LEG_BLEND")
    parser.add_argument("--directory", type=Path, default=Path("/tmp"))
    args = parser.parse_args()
    if not -20.0 <= args.pitch_degrees <= 20.0:
        parser.error("PITCH_DEG must be between -20 and 20")
    if not 0.0 <= args.leg_blend <= 1.0:
        parser.error("LEG_BLEND must be between 0 and 1")

    path = reference_path(args.robot_id, args.directory)
    write_reference(path, args.pitch_degrees, args.leg_blend)
    print(
        f"robot {args.robot_id}: live IDLE target pitch={args.pitch_degrees:g} deg "
        f"leg_blend={args.leg_blend:g} -> {path}"
    )


if __name__ == "__main__":
    main()
