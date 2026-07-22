# Copyright (c) 2025 MicroAGI. License: Apache License, Version 2.0
#
# Whole-body DDS action provider.
#
# Unlike DDSActionProvider (which only maps the 14 arm joints and leaves legs/waist
# at default) and DDSRLActionProvider (which runs an ONBOARD RL locomotion policy for
# the legs), this provider maps ALL 29 body joints straight from the external Unitree
# ``rt/lowcmd`` stream, plus the dex3 fingers. There is NO onboard policy: the external
# SONIC low-level controller does the balancing, exactly as on the real robot. This is
# the correct provider for a floating-base "sim impersonates the real robot" setup.
#
# Modeled on action_provider_dds.DDSActionProvider (returns full_action and lets
# sim_main/RobotController step the env).
from action_provider.action_base import ActionProvider
from typing import Optional
import time
import torch
from dds.dds_master import dds_manager
from dds.g1_tiangong_retarget import (
    G1_CANONICAL_SIGNS,
    TIANGONG_HEAD_JOINT_NAMES,
    indices_in as tiangong_indices_in,
    is_tiangong_robot,
)
from tasks.common_observations.g1_29dof_state import get_robot_boy_joint_names


class DDSLowCmd29ActionProvider(ActionProvider):
    """Whole-body DDS action provider: all 29 body joints + dex3 fingers from rt/lowcmd."""

    def __init__(self, env, args_cli):
        super().__init__("DDSLowCmd29ActionProvider")
        self.enable_robot = args_cli.robot_type
        self.enable_dex3 = args_cli.enable_dex3_dds
        self.env = env
        self.num_envs = int(env.num_envs)
        self.multi_robot = getattr(env, "_multi_robot_dds", None)
        self.robot_dds = None
        self.dex3_dds = None
        self._setup_dds()
        self._setup_joint_mapping()

    def _setup_dds(self):
        try:
            if self.multi_robot is not None:
                self.robot_dds = self.multi_robot
                # Dex3 vectorization is deliberately deferred until body
                # control is proven. Body/arm commands still cover all 29 G1
                # joints; fingers stay at their environment defaults.
                self.dex3_dds = None
                print(f"[{self.name}] namespaced multi-robot DDS initialized "
                      f"({self.num_envs} robots, Dex3 held at default)")
                return
            if self.enable_robot in ("g129", "h1_2"):
                self.robot_dds = dds_manager.get_object("g129")
            if self.enable_dex3:
                self.dex3_dds = dds_manager.get_object("dex3")
            print(f"[{self.name}] DDS communication initialized "
                  f"(robot={self.robot_dds is not None}, dex3={self.dex3_dds is not None})")
        except Exception as e:
            print(f"[{self.name}] DDS initialization failed: {e}")

    def _setup_joint_mapping(self):
        device = self.env.device
        self.all_joint_names = self.env.scene["robot"].data.joint_names
        self.joint_to_index = {name: i for i, name in enumerate(self.all_joint_names)}

        # Canonical Unitree 29-joint order — identical for rt/lowcmd and rt/lowstate.
        # positions[i] (Unitree order) -> Isaac articulation slot joint_to_index[name].
        body_names = get_robot_boy_joint_names()
        assert len(body_names) == 29, f"expected 29 body joints, got {len(body_names)}"
        missing = [n for n in body_names if n not in self.joint_to_index]
        assert not missing, f"joints missing from articulation: {missing}"
        self._body_target_indices = [self.joint_to_index[n] for n in body_names]
        self._body_source_indices = list(range(29))
        self._body_target_idx_t = torch.tensor(self._body_target_indices, dtype=torch.long, device=device)
        self._body_source_idx_t = torch.tensor(self._body_source_indices, dtype=torch.long, device=device)

        if self.enable_dex3:
            # dex3 finger order (per hand) matches DDSActionProvider.
            self.left_hand_joint_mapping = {
                "left_hand_thumb_0_joint": 0, "left_hand_thumb_1_joint": 1, "left_hand_thumb_2_joint": 2,
                "left_hand_middle_0_joint": 3, "left_hand_middle_1_joint": 4,
                "left_hand_index_0_joint": 5, "left_hand_index_1_joint": 6}
            self.right_hand_joint_mapping = {
                "right_hand_thumb_0_joint": 0, "right_hand_thumb_1_joint": 1, "right_hand_thumb_2_joint": 2,
                "right_hand_middle_0_joint": 3, "right_hand_middle_1_joint": 4,
                "right_hand_index_0_joint": 5, "right_hand_index_1_joint": 6}
            self._left_hand_target_indices = [self.joint_to_index[n] for n in self.left_hand_joint_mapping]
            self._left_hand_source_indices = list(self.left_hand_joint_mapping.values())
            self._right_hand_target_indices = [self.joint_to_index[n] for n in self.right_hand_joint_mapping]
            self._right_hand_source_indices = list(self.right_hand_joint_mapping.values())
            self._left_hand_target_idx_t = torch.tensor(self._left_hand_target_indices, dtype=torch.long, device=device)
            self._left_hand_source_idx_t = torch.tensor(self._left_hand_source_indices, dtype=torch.long, device=device)
            self._right_hand_target_idx_t = torch.tensor(self._right_hand_target_indices, dtype=torch.long, device=device)
            self._right_hand_source_idx_t = torch.tensor(self._right_hand_source_indices, dtype=torch.long, device=device)
            self._left_hand_buf = torch.empty(len(self._left_hand_source_indices), device=device, dtype=torch.float32)
            self._right_hand_buf = torch.empty(len(self._right_hand_source_indices), device=device, dtype=torch.float32)

        # Hold the default standing pose until commands arrive (ActionsCfg uses
        # use_default_offset=False, so the raw action IS the absolute joint target).
        default_q = self.env.scene["robot"].data.default_joint_pos.clone()
        self._full_action_buf = default_q.to(device=device, dtype=torch.float32)
        self._default_full = default_q.to(device=device, dtype=torch.float32).clone()
        # The environment default is the measured free-standing equilibrium.
        # Keep reset and rigid-hold warmup on that same pose.  SONIC's internal
        # default-angle offsets are a policy coordinate convention, not a
        # physically self-consistent pinned pose in Isaac.
        self._warmup_full = self._default_full.clone()
        self._positions_buf = torch.empty(
            (self.num_envs, 29), device=device, dtype=torch.float32
        )

        # Dumb mixed-body experiment: the same 29-value G1 command is mapped
        # by joint meaning onto TienKung. Its two extra head joints stay at the
        # articulation default (zero). Elbow flexion has the opposite sign.
        self.tiangong = self.env.scene.articulations.get("tiangong_robot")
        self._tiangong_mask = torch.tensor(
            [is_tiangong_robot(i) for i in range(self.num_envs)],
            dtype=torch.bool, device=device,
        )
        if self.tiangong is not None:
            self._tg_target_indices = list(
                tiangong_indices_in(self.tiangong.data.joint_names)
            )
            self._tg_head_indices = [
                self.tiangong.data.joint_names.index(name)
                for name in TIANGONG_HEAD_JOINT_NAMES
            ]
            self._tg_signs = torch.tensor(
                G1_CANONICAL_SIGNS, device=device, dtype=torch.float32
            )
            self._tg_full_action = self.tiangong.data.default_joint_pos.clone().to(
                device=device, dtype=torch.float32
            )
            self._tg_kp_buf = torch.zeros(
                (self.num_envs, 29), device=device, dtype=torch.float32
            )
            self._tg_kd_buf = torch.zeros_like(self._tg_kp_buf)
            self._tg_dq_buf = torch.zeros_like(self._tg_kp_buf)
            self._tg_tau_buf = torch.zeros_like(self._tg_kp_buf)
            self._tg_gains_written = False
            self._tg_q_lo = self.tiangong.data.soft_joint_pos_limits[..., 0].to(
                device=device, dtype=torch.float32
            )
            self._tg_q_hi = self.tiangong.data.soft_joint_pos_limits[..., 1].to(
                device=device, dtype=torch.float32
            )
            print(
                f"[{self.name}] TienKung retarget rows="
                f"{torch.nonzero(self._tiangong_mask).squeeze(-1).tolist()} "
                f"mapped_joints={self._tg_target_indices} heads={self._tg_head_indices}",
                flush=True,
            )

        # Action-latency model (SIM_ACT_LATENCY = N control steps, 50Hz -> 20ms each).
        # SONIC balances the real G1, which has ~tens of ms of actuation delay; Isaac's
        # implicit PD is ~instantaneous, so the policy's corrections land too early and it
        # goes marginally unstable (holds ~1s then diverges). Delaying the applied target
        # by N steps matches the trained/real latency.
        import os as _os_al
        self._act_latency = int(_os_al.environ.get("SIM_ACT_LATENCY", "0"))
        self._act_buf = []
        if self._act_latency > 0:
            print(f"[{self.name}] ACTION LATENCY = {self._act_latency} steps "
                  f"({self._act_latency * 20} ms)", flush=True)

        # Joint-position limits, to CLAMP commanded targets. When the controller is
        # out-of-distribution (e.g. robot tipping) it emits targets past the joint range
        # (waist_yaw 6.06 rad vs ±2.618 limit); those blow the PD torque up and explode
        # PhysX ("Illegal BroadPhaseUpdateData"). The real robot physically can't exceed
        # its limits, so clamping is faithful and prevents the sim from diverging to nan.
        try:
            _lim = self.env.scene["robot"].data.soft_joint_pos_limits
            self._q_lo = _lim[..., 0].to(device=device, dtype=torch.float32)
            self._q_hi = _lim[..., 1].to(device=device, dtype=torch.float32)
        except Exception as e:
            print(f"[{self.name}] joint limit fetch failed ({e}); clamping disabled")
            self._q_lo = self._q_hi = None

        # Buffers for the FULL per-command PD protocol (kp/kd/dq/tau), shape [N, 29],
        # column i corresponds to Isaac joint self._body_target_indices[i] (Unitree order).
        self._kp_buf = torch.zeros((self.num_envs, 29), device=device, dtype=torch.float32)
        self._kd_buf = torch.zeros((self.num_envs, 29), device=device, dtype=torch.float32)
        self._gains_written = False  # cache: only push kp/kd to sim when they change
        self._dq_buf = torch.zeros((self.num_envs, 29), device=device, dtype=torch.float32)
        self._tau_buf = torch.zeros((self.num_envs, 29), device=device, dtype=torch.float32)

        # --- ROOT-CAUSE TEST: match MuJoCo training torso inertial (gated) ---
        import os as _os_torso
        _tm = _os_torso.environ.get("SIM_TORSO_MASS", "")
        if _tm:
            try:
                _rob = self.env.scene["robot"]; _view = _rob.root_physx_view
                _bi = list(_rob.data.body_names).index("torso_link")
                _m = _view.get_masses().clone(); _m[:, _bi] = float(_tm)
                _view.set_masses(_m, torch.arange(_m.shape[0]))
                _inr = _view.get_inertias().clone()
                _nb = _m.shape[1]
                _flat = _inr.view(_inr.shape[0], _nb, -1)
                _flat[:, _bi, 0] = 0.12407; _flat[:, _bi, 4] = 0.111951; _flat[:, _bi, 8] = 0.0325382
                _view.set_inertias(_flat.view(_inr.shape), torch.arange(_inr.shape[0]))
                print(f"[{self.name}] TORSO OVERRIDE mass->{_tm} inertia->MuJoCo diag (body {_bi})", flush=True)
            except Exception as _e:
                print(f"[{self.name}] torso override FAILED: {_e}", flush=True)


    def get_action(self, env) -> Optional[torch.Tensor]:
        try:
            # During the exact joint warmup, continue ingesting LowCmd into the
            # persistent buffers even though the applied action remains the held
            # pose.  Returning before this read left a command from the preceding
            # trial queued for the first released step, making identical re-arm
            # tests non-deterministic.
            warmup_mask = getattr(self.env, "_warmup_joint_mask", None)
            if warmup_mask is None:
                warmup_active = time.time() < getattr(
                    self.env, "_warmup_joint_until", 0.0
                )
                warmup_mask = torch.full(
                    (self.num_envs,), warmup_active,
                    dtype=torch.bool, device=self.env.device,
                )
            else:
                warmup_mask = warmup_mask.to(
                    device=self.env.device, dtype=torch.bool
                )

            full_action = self._full_action_buf  # persists last command / default pose
            robot = self.env.scene["robot"]
            if self.robot_dds is not None:
                if self.multi_robot is not None:
                    commands = self.robot_dds.snapshot_commands()
                else:
                    commands = [self.robot_dds.get_robot_command()]

                gains_changed = False
                tg_gains_changed = False
                for robot_id, cmd in enumerate(commands):
                    if not cmd or 'motor_cmd' not in cmd:
                        continue
                    mc = cmd['motor_cmd']
                    positions = mc['positions']
                    if len(positions) >= 29:
                        dev = self.env.device
                        self._positions_buf[robot_id].copy_(
                            torch.tensor(positions[:29], dtype=torch.float32, device=dev))
                        if self.tiangong is not None and bool(self._tiangong_mask[robot_id]):
                            body_vals = self._positions_buf[robot_id] * self._tg_signs
                            self._tg_full_action[robot_id, self._tg_target_indices] = body_vals
                            self._tg_full_action[robot_id, self._tg_head_indices] = 0.0
                        else:
                            body_vals = self._positions_buf[robot_id].index_select(
                                0, self._body_source_idx_t
                            )
                            full_action[robot_id].index_copy_(
                                0, self._body_target_idx_t, body_vals
                            )

                        # --- Faithful lowcmd PD protocol -------------------------------
                        # Push the COMMANDED per-joint gains + dq/tau into the implicit
                        # actuator so PhysX runs tau = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff
                        # with the real robot's gains, instead of the sim's fixed cfg gains.
                        # Values are in Unitree order == column order of _body_target_indices.
                        kp = mc.get('kp'); kd = mc.get('kd')
                        dq = mc.get('velocities'); tau = mc.get('torques')
                        jids = self._body_target_indices
                        if kp is not None and kd is not None and len(kp) >= 29 and len(kd) >= 29:
                            new_kp = torch.tensor(kp[:29], dtype=torch.float32, device=dev)
                            new_kd = torch.tensor(kd[:29], dtype=torch.float32, device=dev)
                            # SONIC sends CONSTANT gains; write_joint_stiffness/damping_to_sim
                            # are (relatively) expensive USD writes on the CPU pipeline. Only
                            # push them when they actually change, not every 50 Hz tick.
                            if self.tiangong is not None and bool(self._tiangong_mask[robot_id]):
                                if (not self._tg_gains_written
                                        or not torch.equal(new_kp, self._tg_kp_buf[robot_id])
                                        or not torch.equal(new_kd, self._tg_kd_buf[robot_id])):
                                    self._tg_kp_buf[robot_id].copy_(new_kp)
                                    self._tg_kd_buf[robot_id].copy_(new_kd)
                                    tg_gains_changed = True
                            elif (not self._gains_written
                                  or not torch.equal(new_kp, self._kp_buf[robot_id])
                                  or not torch.equal(new_kd, self._kd_buf[robot_id])):
                                self._kp_buf[robot_id].copy_(new_kp)
                                self._kd_buf[robot_id].copy_(new_kd)
                                gains_changed = True
                                # DIAGNOSTIC: read back the ACTUAL sim gains to confirm the
                                # SONIC gains stuck (vs the cfg gains 150-200/ankle 20).
                                if not getattr(self, "_gains_logged", False):
                                    try:
                                        st = robot.data.joint_stiffness[0]
                                        dp = robot.data.joint_damping[0]
                                        ap = self.joint_to_index["left_ankle_pitch_joint"]
                                        kn = self.joint_to_index["left_knee_joint"]
                                        hp = self.joint_to_index["left_hip_pitch_joint"]
                                        print(f"[{self.name}] APPLIED sim gains: hip_pitch kp={st[hp]:.1f}(SONIC~99) "
                                              f"knee kp={st[kn]:.1f}(~99) ankle_pitch kp={st[ap]:.1f}(~28.5) "
                                              f"ankle kd={dp[ap]:.2f}(~1.8) | cmd kp[0]={float(new_kp[0]):.1f}", flush=True)
                                    except Exception as _e:
                                        print(f"[{self.name}] gain readback failed: {_e}", flush=True)
                                    self._gains_logged = True
                        if dq is not None and len(dq) >= 29:
                            _dq = torch.tensor(dq[:29], dtype=torch.float32, device=dev)
                            if self.tiangong is not None and bool(self._tiangong_mask[robot_id]):
                                self._tg_dq_buf[robot_id].copy_(_dq * self._tg_signs)
                            else:
                                self._dq_buf[robot_id].copy_(_dq)
                        if tau is not None and len(tau) >= 29:
                            _tau = torch.tensor(tau[:29], dtype=torch.float32, device=dev)
                            if self.tiangong is not None and bool(self._tiangong_mask[robot_id]):
                                self._tg_tau_buf[robot_id].copy_(_tau * self._tg_signs)
                            else:
                                self._tau_buf[robot_id].copy_(_tau)

                if gains_changed:
                    robot.write_joint_stiffness_to_sim(
                        self._kp_buf, joint_ids=self._body_target_indices
                    )
                    robot.write_joint_damping_to_sim(
                        self._kd_buf, joint_ids=self._body_target_indices
                    )
                    self._gains_written = True
                if self.tiangong is not None and tg_gains_changed:
                    self.tiangong.write_joint_stiffness_to_sim(
                        self._tg_kp_buf, joint_ids=self._tg_target_indices
                    )
                    self.tiangong.write_joint_damping_to_sim(
                        self._tg_kd_buf, joint_ids=self._tg_target_indices
                    )
                    self._tg_gains_written = True
                robot.set_joint_velocity_target(
                    self._dq_buf, joint_ids=self._body_target_indices
                )
                robot.set_joint_effort_target(
                    self._tau_buf, joint_ids=self._body_target_indices
                )
                if self.tiangong is not None:
                    self._tg_full_action = torch.clamp(
                        self._tg_full_action, self._tg_q_lo, self._tg_q_hi
                    )
                    if torch.any(warmup_mask & self._tiangong_mask):
                        _held = warmup_mask & self._tiangong_mask
                        self._tg_full_action[_held] = (
                            self.tiangong.data.default_joint_pos[_held]
                        )
                    self.tiangong.set_joint_position_target(self._tg_full_action)
                    self.tiangong.set_joint_velocity_target(
                        self._tg_dq_buf, joint_ids=self._tg_target_indices
                    )
                    self.tiangong.set_joint_effort_target(
                        self._tg_tau_buf, joint_ids=self._tg_target_indices
                    )
            if self.dex3_dds is not None:
                hand_cmds = self.dex3_dds.get_hand_commands()
                if hand_cmds:
                    left_cmd = hand_cmds.get('left_hand_cmd', {})
                    right_cmd = hand_cmds.get('right_hand_cmd', {})
                    if left_cmd and right_cmd:
                        lp = left_cmd.get('positions', [])
                        rp = right_cmd.get('positions', [])
                        if len(lp) >= len(self._left_hand_buf) and len(rp) >= len(self._right_hand_buf):
                            self._left_hand_buf.copy_(
                                torch.tensor(lp[:len(self._left_hand_buf)], dtype=torch.float32, device=self.env.device))
                            self._right_hand_buf.copy_(
                                torch.tensor(rp[:len(self._right_hand_buf)], dtype=torch.float32, device=self.env.device))
                            full_action[0].index_copy_(0, self._left_hand_target_idx_t,
                                                       self._left_hand_buf.index_select(0, self._left_hand_source_idx_t))
                            full_action[0].index_copy_(0, self._right_hand_target_idx_t,
                                                       self._right_hand_buf.index_select(0, self._right_hand_source_idx_t))
            if self._q_lo is not None:
                full_action = torch.clamp(full_action, self._q_lo, self._q_hi)
            if torch.any(warmup_mask):
                full_action = full_action.clone()
                full_action[warmup_mask] = self._warmup_full[warmup_mask]
            # apply the position target from N control steps ago (action latency)
            if self._act_latency > 0:
                self._act_buf.append(full_action.clone())
                if len(self._act_buf) > self._act_latency + 1:
                    self._act_buf.pop(0)
                full_action = self._act_buf[0]
            return full_action
        except Exception as e:
            print(f"[{self.name}] Get DDS action failed: {e}")
            return None

    def cleanup(self):
        try:
            if self.robot_dds is not None:
                self.robot_dds.stop_communication()
            if self.dex3_dds is not None:
                self.dex3_dds.stop_communication()
        except Exception as e:
            print(f"[{self.name}] Clean up DDS resources failed: {e}")
