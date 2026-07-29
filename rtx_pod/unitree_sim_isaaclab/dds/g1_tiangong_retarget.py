"""Deliberately simple G1-command to TienKung 2dex simulation retarget.

This is an experiment adapter, not a robot-agnostic policy claim.  The 29 G1
body coordinates map to TienKung's 12 leg, 3 waist, and 14 arm coordinates;
TienKung's two head joints are held separately at zero.
"""

import os


TIANGONG_ACTIVE_JOINT_NAMES = (
    "hip_pitch_l_joint", "hip_roll_l_joint", "hip_yaw_l_joint",
    "knee_pitch_l_joint", "ankle_pitch_l_joint", "ankle_roll_l_joint",
    "hip_pitch_r_joint", "hip_roll_r_joint", "hip_yaw_r_joint",
    "knee_pitch_r_joint", "ankle_pitch_r_joint", "ankle_roll_r_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint",
    "wrist_roll_l_joint",
    "shoulder_pitch_r_joint", "shoulder_roll_r_joint", "shoulder_yaw_r_joint",
    "elbow_pitch_r_joint", "elbow_yaw_r_joint", "wrist_pitch_r_joint",
    "wrist_roll_r_joint",
)

TIANGONG_HEAD_JOINT_NAMES = ("head_yaw_joint", "head_pitch_joint")

# TienKung elbow flexion is negative while the G1 elbow coordinate is positive.
G1_CANONICAL_SIGNS = tuple(
    -1.0 if index in (18, 25) else 1.0 for index in range(29)
)


def mixed_tiangong_enabled():
    return os.environ.get("SIM_MIXED_TIANGONG", "0") == "1"


def is_tiangong_robot(robot_id):
    """The MVP alternates bodies: IDs 1,3,5,7 are TienKung."""
    return mixed_tiangong_enabled() and int(robot_id) % 2 == 1


def tiangong_ids(robot_count):
    return tuple(index for index in range(robot_count) if is_tiangong_robot(index))


def indices_in(joint_names, wanted=TIANGONG_ACTIVE_JOINT_NAMES):
    index_by_name = {name: index for index, name in enumerate(joint_names)}
    missing = [name for name in wanted if name not in index_by_name]
    if missing:
        raise ValueError(f"TienKung articulation is missing joints: {missing}")
    return tuple(index_by_name[name] for name in wanted)


def g1_to_tiangong(values):
    if len(values) != 29:
        raise ValueError(f"expected 29 G1 values, got {len(values)}")
    return [float(value) * sign for value, sign in zip(values, G1_CANONICAL_SIGNS)]


def tiangong_to_g1(values):
    # The sign transform is its own inverse.
    return g1_to_tiangong(values)
