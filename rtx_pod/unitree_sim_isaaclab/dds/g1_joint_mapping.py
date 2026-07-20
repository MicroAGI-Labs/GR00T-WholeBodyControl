"""Canonical joint-order conversions for the Unitree G1 DDS boundary.

Unitree ``LowState.motor_state`` and ``LowCmd.motor_cmd`` use the 29-joint order
declared below.  Isaac articulations may expose those joints in a different order
and may include additional hand joints.  All state and command paths must use
these conversions instead of copying an array prefix.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TypeVar


T = TypeVar("T")


UNITREE_G1_29_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


# SONIC's hardware-order standing pose from policy_parameters.hpp.  This is
# also the pose used for sim reset and joint warmup, so the controller never
# starts against a different hand-tuned posture.
UNITREE_G1_29_DEFAULT_POSITIONS = (
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    0.0, 0.0, 0.0,
    0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
    0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
)


# Per-joint effort contract used by the SONIC deploy. These values mirror
# policy_parameters.hpp and must also be the Isaac actuator limits: the policy's
# action scale is computed from them. In particular, current SONIC uses the
# 7520_22 hip-pitch motor (139 Nm); the upstream Isaac preset still carries the
# older 7520_14 value (88 Nm), which clips more than half of standing updates.
UNITREE_G1_29_EFFORT_LIMITS = (
    139.0, 139.0, 88.0, 139.0, 25.0, 25.0,
    139.0, 139.0, 88.0, 139.0, 25.0, 25.0,
    88.0, 25.0, 25.0,
    25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0,
    25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0,
)

UNITREE_G1_EFFORT_LIMIT_BY_NAME = dict(
    zip(UNITREE_G1_29_JOINT_NAMES, UNITREE_G1_29_EFFORT_LIMITS)
)


# Unitree index -> Isaac articulation index for the G1 articulation used by the
# flat task.  This is the permutation previously embedded in
# get_robot_boy_joint_states().
UNITREE_FROM_ISAAC_INDICES = (
    0,
    3,
    6,
    9,
    13,
    17,
    1,
    4,
    7,
    10,
    14,
    18,
    2,
    5,
    8,
    11,
    15,
    19,
    21,
    23,
    25,
    27,
    12,
    16,
    20,
    22,
    24,
    26,
    28,
)


def unitree_indices_in(isaac_joint_names: Sequence[str]) -> tuple[int, ...]:
    """Return Isaac indices in canonical Unitree order.

    Name-based lookup is used at runtime so the conversion remains correct if
    Isaac changes articulation ordering or appends hand joints.
    """

    if len(set(isaac_joint_names)) != len(isaac_joint_names):
        raise ValueError("Isaac articulation contains duplicate joint names")

    index_by_name = {name: index for index, name in enumerate(isaac_joint_names)}
    missing = [name for name in UNITREE_G1_29_JOINT_NAMES if name not in index_by_name]
    if missing:
        raise ValueError(f"Isaac articulation is missing G1 body joints: {missing}")
    return tuple(index_by_name[name] for name in UNITREE_G1_29_JOINT_NAMES)


def isaac_to_unitree(
    isaac_values: Sequence[T], isaac_joint_names: Sequence[str]
) -> list[T]:
    """Gather an Isaac-order vector into canonical Unitree 29-joint order."""

    if len(isaac_values) != len(isaac_joint_names):
        raise ValueError(
            f"Isaac value/name length mismatch: {len(isaac_values)} != {len(isaac_joint_names)}"
        )
    return [isaac_values[index] for index in unitree_indices_in(isaac_joint_names)]


def unitree_to_isaac(
    unitree_values: Sequence[T],
    isaac_joint_names: Sequence[str],
    existing_isaac_values: Sequence[T] | None = None,
) -> list[T]:
    """Scatter canonical Unitree values into an Isaac-order articulation vector.

    ``existing_isaac_values`` preserves joints not represented by Unitree's
    29-joint body protocol, such as dexterous hand joints.
    """

    if len(unitree_values) != len(UNITREE_G1_29_JOINT_NAMES):
        raise ValueError(
            f"Expected 29 Unitree body values, received {len(unitree_values)}"
        )
    if existing_isaac_values is None:
        result: list[T | None] = [None] * len(isaac_joint_names)
    else:
        if len(existing_isaac_values) != len(isaac_joint_names):
            raise ValueError(
                "Existing Isaac value/name length mismatch: "
                f"{len(existing_isaac_values)} != {len(isaac_joint_names)}"
            )
        result = list(existing_isaac_values)

    for unitree_index, isaac_index in enumerate(unitree_indices_in(isaac_joint_names)):
        result[isaac_index] = unitree_values[unitree_index]
    return result  # type: ignore[return-value]
