#!/usr/bin/env python3
"""Atomically request one targeted pelvis perturbation from the running sim."""

import argparse
import json
import os
from pathlib import Path
import tempfile
import time


def parse_args():
    parser = argparse.ArgumentParser(
        description="Apply a bounded force/torque to selected simulated G1 pelvises"
    )
    targets = parser.add_mutually_exclusive_group(required=True)
    targets.add_argument("--robot", type=int, action="append", dest="robots",
                         help="robot ID; repeat to target several robots")
    targets.add_argument("--all", action="store_true", help="target every robot")
    parser.add_argument("--force", nargs=3, type=float, required=True,
                        metavar=("FX", "FY", "FZ"), help="world-frame force in N")
    parser.add_argument("--torque", nargs=3, type=float, default=(0.0, 0.0, 0.0),
                        metavar=("TX", "TY", "TZ"), help="world-frame torque in Nm")
    parser.add_argument("--duration", type=float, default=0.15,
                        help="simulated seconds to apply the wrench (default: 0.15)")
    parser.add_argument("--file", default=os.environ.get(
        "SIM_PERTURB_FILE", "/tmp/sim_perturbation.json"))
    return parser.parse_args()


def main():
    args = parse_args()
    payload = {
        "id": str(time.time_ns()),
        "robots": "all" if args.all else args.robots,
        "force_n": args.force,
        "torque_nm": args.torque,
        "duration_s": args.duration,
    }
    destination = Path(args.file)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{destination.name}.",
                                      dir=str(destination.parent), text=True)
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(payload, stream, separators=(",", ":"))
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    print(json.dumps(payload, separators=(",", ":")))


if __name__ == "__main__":
    main()
