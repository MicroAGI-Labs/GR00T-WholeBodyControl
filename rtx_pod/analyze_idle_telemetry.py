#!/usr/bin/env python3
"""Summarize synchronized SONIC IDLE reference/state/command telemetry."""

import argparse
import csv
import math
import statistics


JOINT_NAMES = [
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_ank_pitch", "L_ank_roll",
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_ank_pitch", "R_ank_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "L_sh_pitch", "L_sh_roll", "L_sh_yaw", "L_elbow", "L_wr_roll", "L_wr_pitch", "L_wr_yaw",
    "R_sh_pitch", "R_sh_roll", "R_sh_yaw", "R_elbow", "R_wr_roll", "R_wr_pitch", "R_wr_yaw",
]


def quat_rpy_deg(row, prefix):
    """Return intrinsic roll/pitch/yaw in degrees for a wxyz CSV quaternion."""
    w, x, y, z = (float(row[f"{prefix}_{c}"]) for c in ("qw", "qx", "qy", "qz"))
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def mean(rows, column):
    return statistics.fmean(float(row[column]) for row in rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="CSV produced by --idle-telemetry-logfile")
    parser.add_argument("--discard", type=float, default=2.0,
                        help="discard this many controller seconds after the first IDLE row")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="analyze at most this many controller seconds (0=all remaining)")
    args = parser.parse_args()

    with open(args.csv, newline="") as f:
        # The controller may still be appending while this report is read.  A
        # final row without its newline can therefore be incomplete; ignore it
        # instead of turning a live-safe diagnostic into a parse failure.
        rows = [
            row for row in csv.DictReader(f)
            if None not in row.values() and int(row["movement_mode"]) == 0
        ]
    if not rows:
        raise SystemExit("no IDLE rows found")

    t0 = float(rows[0]["controller_time_s"])
    window_start = t0 + args.discard
    window_end = window_start + args.duration if args.duration > 0 else math.inf
    rows = [
        row for row in rows
        if window_start <= float(row["controller_time_s"]) <= window_end
    ]
    if not rows:
        raise SystemExit("no rows remain after --discard")

    ref_rpy = [quat_rpy_deg(row, "ref") for row in rows]
    meas_rpy = [quat_rpy_deg(row, "meas") for row in rows]
    gyro = [math.sqrt(sum(float(row[f"meas_w{a}"]) ** 2 for a in "xyz")) for row in rows]
    print(f"rows={len(rows)} controller_span={float(rows[-1]['controller_time_s'])-float(rows[0]['controller_time_s']):.3f}s")
    print("pelvis_rpy_deg "
          f"ref=({statistics.fmean(v[0] for v in ref_rpy):+.3f},"
          f"{statistics.fmean(v[1] for v in ref_rpy):+.3f},"
          f"{statistics.fmean(v[2] for v in ref_rpy):+.3f}) "
          f"measured=({statistics.fmean(v[0] for v in meas_rpy):+.3f},"
          f"{statistics.fmean(v[1] for v in meas_rpy):+.3f},"
          f"{statistics.fmean(v[2] for v in meas_rpy):+.3f}) "
          f"gyro_norm_mean={statistics.fmean(gyro):.4f}rad/s")
    print("joint,ref_mean,measured_mean,measured_minus_ref,command_mean,command_minus_measured,measured_sd")
    for i, name in enumerate(JOINT_NAMES):
        ref = mean(rows, f"ref_q{i}")
        measured = mean(rows, f"meas_q{i}")
        command = mean(rows, f"cmd_q{i}")
        measured_values = [float(row[f"meas_q{i}"]) for row in rows]
        print(f"{name},{ref:+.5f},{measured:+.5f},{measured-ref:+.5f},"
              f"{command:+.5f},{command-measured:+.5f},"
              f"{statistics.pstdev(measured_values):.5f}")


if __name__ == "__main__":
    main()
