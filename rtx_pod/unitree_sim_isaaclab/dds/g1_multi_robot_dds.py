"""Namespaced in-process DDS transport for vectorized G1 simulation.

The Unitree ChannelFactory remains process-wide on DDS domain 1.  Robot
isolation therefore comes from distinct topics, not additional DDS domains or
sidecar processes.  Isaac owns one record per environment; DDS callbacks update
commands and reset requests while the simulation loop updates measured state.
"""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
import threading
import time
from typing import Any

import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__IMUState_,
    unitree_hg_msg_dds__LowState_,
)
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_, LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


def robot_topic(prefix_base: str, robot_id: int, suffix: str) -> str:
    """Return the canonical namespaced topic for one simulated robot."""

    base = prefix_base.rstrip("/")
    suffix = suffix.removeprefix("rt/").lstrip("/")
    if not base:
        raise ValueError("multi-robot topic prefix base cannot be empty")
    if robot_id < 0:
        raise ValueError("robot_id must be non-negative")
    return f"{base}/{robot_id}/{suffix}"


@dataclass
class _RobotRecord:
    lock: threading.Lock = field(default_factory=threading.Lock)
    state: dict[str, np.ndarray] | None = None
    state_seq: int = 0
    state_time_ns: int = 0
    command: dict[str, Any] | None = None
    command_seq: int = 0
    command_time_ns: int = 0
    valid_after_command_seq: int = 0
    reset_seq: int = 0
    reset_requests: deque[tuple[int, str]] = field(default_factory=deque)


class G1MultiRobotDDS:
    """DDS endpoints and latest-sample records for ``robot_count`` sim rows."""

    def __init__(
        self,
        robot_count: int,
        prefix_base: str = "rt/sim/g1",
        publish_hz: float = 100.0,
    ) -> None:
        if robot_count < 2:
            raise ValueError("G1MultiRobotDDS is only for robot_count >= 2")
        if publish_hz <= 0:
            raise ValueError("publish_hz must be positive")

        self.robot_count = robot_count
        self.prefix_base = prefix_base.rstrip("/")
        self.publish_period = 1.0 / publish_hz
        self.records = [_RobotRecord() for _ in range(robot_count)]
        self.crcs = [CRC() for _ in range(robot_count)]
        self.lowstates = [unitree_hg_msg_dds__LowState_() for _ in range(robot_count)]
        self.secondary_imus = [unitree_hg_msg_dds__IMUState_() for _ in range(robot_count)]
        self.state_publishers = []
        self.imu_publishers = []
        self.command_subscribers = []
        self.reset_subscribers = []
        self._running = False
        self._thread: threading.Thread | None = None

        for robot_id in range(robot_count):
            state_pub = ChannelPublisher(self.topic(robot_id, "lowstate"), LowState_)
            state_pub.Init()
            imu_pub = ChannelPublisher(self.topic(robot_id, "secondary_imu"), IMUState_)
            imu_pub.Init()

            command_sub = ChannelSubscriber(self.topic(robot_id, "lowcmd"), LowCmd_)
            command_sub.Init(
                lambda msg, rid=robot_id: self._on_command(rid, msg), 32
            )
            reset_sub = ChannelSubscriber(self.topic(robot_id, "reset_pose/cmd"), String_)
            reset_sub.Init(lambda msg, rid=robot_id: self._on_reset(rid, msg), 8)

            self.state_publishers.append(state_pub)
            self.imu_publishers.append(imu_pub)
            self.command_subscribers.append(command_sub)
            self.reset_subscribers.append(reset_sub)

        print(
            f"[g1_multi_dds] initialized {robot_count} robots under "
            f"{self.prefix_base}/<id>",
            flush=True,
        )

    def topic(self, robot_id: int, suffix: str) -> str:
        return robot_topic(self.prefix_base, robot_id, suffix)

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._publish_loop, name="g1-multi-dds", daemon=True
        )
        self._thread.start()

    def stop_all_communication(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def stop_communication(self) -> None:
        self.stop_all_communication()

    def update_states(self, positions, velocities, torques, imu_data) -> None:
        arrays = [
            np.asarray(value, dtype=np.float32)
            for value in (positions, velocities, torques, imu_data)
        ]
        if any(value.shape[0] != self.robot_count for value in arrays):
            raise ValueError("state batch does not match robot_count")

        now_ns = time.monotonic_ns()
        for robot_id, record in enumerate(self.records):
            with record.lock:
                record.state = {
                    "joint_positions": arrays[0][robot_id].copy(),
                    "joint_velocities": arrays[1][robot_id].copy(),
                    "joint_torques": arrays[2][robot_id].copy(),
                    "imu_data": arrays[3][robot_id].copy(),
                }
                record.state_seq += 1
                record.state_time_ns = now_ns

    def snapshot_commands(self) -> list[dict[str, Any] | None]:
        result = []
        for record in self.records:
            with record.lock:
                if (
                    record.command is None
                    or record.command_seq <= record.valid_after_command_seq
                ):
                    result.append(None)
                else:
                    command = deepcopy(record.command)
                    command["_seq"] = record.command_seq
                    command["_time_ns"] = record.command_time_ns
                    result.append(command)
        return result

    def invalidate_commands(self, robot_ids) -> None:
        for robot_id in robot_ids:
            record = self.records[int(robot_id)]
            with record.lock:
                record.valid_after_command_seq = record.command_seq

    def has_fresh_command(self, robot_id: int) -> bool:
        record = self.records[robot_id]
        with record.lock:
            return (
                record.command is not None
                and record.command_seq > record.valid_after_command_seq
            )

    def pop_reset_requests(self) -> list[tuple[int, int, str]]:
        requests = []
        for robot_id, record in enumerate(self.records):
            with record.lock:
                while record.reset_requests:
                    seq, category = record.reset_requests.popleft()
                    requests.append((robot_id, seq, category))
        return requests

    def _on_command(self, robot_id: int, msg: LowCmd_) -> None:
        if self.crcs[robot_id].Crc(msg) != msg.crc:
            print(f"[g1_multi_dds:{robot_id}] rejected LowCmd with bad CRC", flush=True)
            return
        motor_count = len(msg.motor_cmd)
        command = {
            "mode_pr": int(msg.mode_pr),
            "mode_machine": int(msg.mode_machine),
            "motor_cmd": {
                "positions": [float(msg.motor_cmd[i].q) for i in range(motor_count)],
                "velocities": [float(msg.motor_cmd[i].dq) for i in range(motor_count)],
                "torques": [float(msg.motor_cmd[i].tau) for i in range(motor_count)],
                "kp": [float(msg.motor_cmd[i].kp) for i in range(motor_count)],
                "kd": [float(msg.motor_cmd[i].kd) for i in range(motor_count)],
            },
        }
        record = self.records[robot_id]
        with record.lock:
            record.command = command
            record.command_seq += 1
            record.command_time_ns = time.monotonic_ns()

    def _on_reset(self, robot_id: int, msg: String_) -> None:
        record = self.records[robot_id]
        with record.lock:
            record.reset_seq += 1
            record.reset_requests.append((record.reset_seq, str(msg.data)))

    def _publish_loop(self) -> None:
        next_publish = time.monotonic()
        while self._running:
            now = time.monotonic()
            if now < next_publish:
                time.sleep(next_publish - now)
            else:
                next_publish = now
            for robot_id in range(self.robot_count):
                self._publish_robot(robot_id)
            next_publish += self.publish_period

    def _publish_robot(self, robot_id: int) -> None:
        record = self.records[robot_id]
        with record.lock:
            if record.state is None:
                return
            state = {key: value.copy() for key, value in record.state.items()}

        lowstate = self.lowstates[robot_id]
        position = state["joint_positions"]
        velocity = state["joint_velocities"]
        torque = state["joint_torques"]
        count = min(len(lowstate.motor_state), len(position), len(velocity), len(torque))
        for index in range(count):
            motor = lowstate.motor_state[index]
            motor.q = float(position[index])
            motor.dq = float(velocity[index])
            motor.tau_est = float(torque[index])

        imu_data = state["imu_data"]
        if len(imu_data) >= 13:
            lowstate.imu_state.quaternion[:] = imu_data[3:7]
            lowstate.imu_state.accelerometer[:] = imu_data[7:10]
            lowstate.imu_state.gyroscope[:] = imu_data[10:13]

        lowstate.tick += 1
        lowstate.crc = self.crcs[robot_id].Crc(lowstate)
        self.state_publishers[robot_id].Write(lowstate)

        secondary = self.secondary_imus[robot_id]
        secondary.quaternion = list(lowstate.imu_state.quaternion)
        secondary.gyroscope = list(lowstate.imu_state.gyroscope)
        secondary.accelerometer = list(lowstate.imu_state.accelerometer)
        secondary.rpy = list(lowstate.imu_state.rpy)
        secondary.temperature = int(lowstate.imu_state.temperature)
        self.imu_publishers[robot_id].Write(secondary)
