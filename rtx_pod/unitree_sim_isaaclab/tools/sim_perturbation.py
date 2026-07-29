"""Validation helpers for file-triggered Isaac G1 perturbations.

This module deliberately has no Isaac or torch dependency so the command
contract can be unit-tested outside the simulator.
"""

from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class Perturbation:
    command_id: str
    robot_ids: tuple
    force_n: tuple
    torque_nm: tuple
    duration_s: float


def _vector(value, name):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must be a three-element JSON array")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{name} must contain only finite numbers")
    return result


def parse_perturbation(text, robot_count, max_force_n=500.0,
                       max_torque_nm=200.0, max_duration_s=2.0):
    """Parse and bound one perturbation command.

    Format::

      {"id":"unique", "robots":[0], "force_n":[50,0,0],
       "torque_nm":[0,0,0], "duration_s":0.15}

    ``robots`` may also be the string ``"all"``. Limits are checked rather
    than silently clamped so a typo cannot turn into an unexpectedly hard hit.
    """
    try:
        value = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid perturbation JSON: {exc.msg}") from exc
    if not isinstance(value, dict):
        raise ValueError("perturbation command must be a JSON object")

    command_id = str(value.get("id", "")).strip()
    if not command_id:
        raise ValueError("id must be a non-empty unique command identifier")

    robots = value.get("robots")
    if robots == "all":
        robot_ids = tuple(range(robot_count))
    elif isinstance(robots, list) and robots:
        if any(isinstance(robot_id, bool) or not isinstance(robot_id, int)
               for robot_id in robots):
            raise ValueError("robots must contain integer robot IDs")
        robot_ids = tuple(dict.fromkeys(robots))
    else:
        raise ValueError('robots must be a non-empty list or "all"')
    if any(robot_id < 0 or robot_id >= robot_count for robot_id in robot_ids):
        raise ValueError(f"robot ID outside valid range 0..{robot_count - 1}")

    force_n = _vector(value.get("force_n"), "force_n")
    torque_nm = _vector(value.get("torque_nm", [0, 0, 0]), "torque_nm")
    force_mag = math.sqrt(sum(component * component for component in force_n))
    torque_mag = math.sqrt(sum(component * component for component in torque_nm))
    if force_mag > max_force_n:
        raise ValueError(f"force magnitude {force_mag:.1f} N exceeds {max_force_n:.1f} N")
    if torque_mag > max_torque_nm:
        raise ValueError(
            f"torque magnitude {torque_mag:.1f} Nm exceeds {max_torque_nm:.1f} Nm"
        )

    try:
        duration_s = float(value.get("duration_s"))
    except (TypeError, ValueError) as exc:
        raise ValueError("duration_s must be a finite positive number") from exc
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be a finite positive number")
    if duration_s > max_duration_s:
        raise ValueError(
            f"duration_s {duration_s:.3f} exceeds {max_duration_s:.3f} s"
        )
    if force_mag == 0.0 and torque_mag == 0.0:
        raise ValueError("force_n and torque_nm cannot both be zero")

    return Perturbation(
        command_id=command_id,
        robot_ids=robot_ids,
        force_n=force_n,
        torque_nm=torque_nm,
        duration_s=duration_s,
    )
