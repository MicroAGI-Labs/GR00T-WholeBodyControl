# Copyright (c) 2025 MicroAGI. License: Apache License, Version 2.0
#
# Simple WHOLE-BODY G1 sim: a floating-base 29-DOF Unitree G1 (dex3 hands) standing on
# a flat ground plane. Driven entirely by the external Unitree ``rt/lowcmd`` stream over
# DDS (all 29 body joints + dex3 fingers) so the SONIC low-level controller does the
# balancing — "sim impersonates the real robot". No tables / objects / onboard policy.
#
# Reuses the proven floating-base articulation (G129_CFG_WITH_DEX3_WHOLEBODY, real-robot
# PD gains) and the g1_29dof_state lowstate/IMU publishing from the -Wholebody task; only
# the scene (flat) and the action source (see action_provider_lowcmd29 / --action_source
# dds_lowcmd29) differ. The task id deliberately omits "Wholebody" so sim_main's
# auto-route to the onboard-RL provider does not fire.
import torch

import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as base_mdp
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg
from isaaclab.utils import configclass

# reuse the shared mdp (obs funcs + JointPositionActionCfg) from the wholebody task
from tasks.g1_tasks.move_cylinder_g1_29dof_dex3_wholebody import mdp
from tasks.common_config import G1RobotPresets, CameraPresets  # isort: skip
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager


##
# Scene: floating G1 on a flat ground plane
##
@configclass
class FlatG1SceneCfg(InteractiveSceneCfg):
    """Minimal scene: ground plane, dome light, floating G1, robot-mounted cameras."""

    # flat ground
    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        spawn=GroundPlaneCfg(),
    )

    # floating-base 29-DOF G1, standing at 0.8 m. Use the DEX1 (simple-gripper) whole-body
    # variant for the BALANCE bring-up: it carries far fewer actuated finger DOF than dex3,
    # which cuts the fixed per-step USD/articulation sync cost on the single-env CPU pipeline
    # and lets RTF reach ~1.0 (the balance policy needs sim-time == wall-clock). Hands are
    # irrelevant to balancing. Launch WITHOUT --enable_dex3_dds. Swap back to
    # g1_29dof_dex3_wholebody for the full VLA/manipulation run.
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex3_wholebody(
        init_pos=(0.0, 0.0, 0.793),
        init_rot=(1.0, 0.0, 0.0, 0.0),
    )

    # lighting
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    # robot-mounted cameras (ego + wrists) — same presets the VLA/camera-pub expect.
    # DISABLED for the balance bring-up: vision is irrelevant to balancing and camera
    # rendering is the dominant fixed per-step cost on the CPU pipeline (it wrecks RTF).
    # Re-enable (and restore --enable_cameras + the camera_image obs term) for the VLA run.
    # front_camera = CameraPresets.g1_front_camera()
    # left_wrist_camera = CameraPresets.left_dex3_wrist_camera()
    # right_wrist_camera = CameraPresets.right_dex3_wrist_camera()


##
# MDP
##
@configclass
class ActionsCfg:
    """Absolute joint-position targets on ALL joints.

    use_default_offset=False: the raw action IS the absolute joint angle, matching the
    SONIC deploy which sends absolute q_target = default + residual over rt/lowcmd.
    """
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[".*"], scale=1.0, use_default_offset=False
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        robot_gipper_state = ObsTerm(func=mdp.get_robot_dex3_joint_states)
        # camera_image disabled together with the scene cameras (see FlatG1SceneCfg).
        # camera_image = ObsTerm(func=mdp.get_camera_image)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    """No task terminations — this is a persistent balancing sandbox."""
    pass


@configclass
class RewardsCfg:
    """No rewards — DDS-driven, not an RL task. (Must be an empty configclass, not
    None: IsaacLab's manager reads cfg.__dict__.)"""
    pass


@configclass
class EventCfg:
    """No IsaacLab event terms (reset handled by the SimpleEventManager below)."""
    pass


@configclass
class FlatG1Dex3EnvCfg(ManagerBasedRLEnvCfg):
    scene: FlatG1SceneCfg = FlatG1SceneCfg(num_envs=1, env_spacing=2.5, replicate_physics=True)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    commands = None
    curriculum = None

    def __post_init__(self):
        # Reset the robot into the SONIC controller's ACTUAL standing pose. Stream logging
        # (rt/lowcmd vs rt/lowstate in the held state) showed the controller commands
        # knee~0.30 / ankle~-0.20 while the robot was held at knee 0.669 / ankle -0.363 ->
        # a ~0.37 rad leg mismatch that made it lurch and topple on release. The proven
        # stance is the config DEFAULT_DOF_ANGLES (hip -0.1, knee 0.3, ankle -0.2) with the
        # arms at loco_upper_body_dof_pos (shoulder_roll +-0.3, elbow 1.0) that the policy
        # tracks. z lowered accordingly (test controller-free that the pin holds ~0 deg).
        # This is the ACTUAL pose the SONIC controller balances at, captured from the
        # working MuJoCo run (g1_debug body_q_measured, where measured==target == the
        # controller's equilibrium): hip~-0.05, knee~0.47, ANKLE~0 (not -0.2!), arms at
        # shoulder_roll +-0.3 / shoulder_yaw -+0.65 / elbow 0.76. Every earlier guess had
        # the wrong knee AND a far-too-plantarflexed ankle, so on release the controller
        # yanked the legs to this pose and toppled. z set for feet-on-ground at this stance.
        self.scene.robot.init_state.pos = (0.0, 0.0, 0.80)
        self.scene.robot.init_state.joint_pos = {
            "left_hip_pitch_joint": -0.05, "left_knee_joint": 0.47, "left_ankle_pitch_joint": 0.0,
            "right_hip_pitch_joint": -0.05, "right_knee_joint": 0.47, "right_ankle_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.3, "left_shoulder_yaw_joint": -0.65, "left_elbow_joint": 0.76,
            "right_shoulder_roll_joint": -0.3, "right_shoulder_yaw_joint": 0.65, "right_elbow_joint": 0.76,
        }
        # 100 Hz physics, 50 Hz control (control period 0.02 s == deploy Control thread).
        # NOTE: single-env CPU PhysX at 200 Hz (decim=4) only reaches RTF~0.69 -> the
        # deploy's wall-clock 50 Hz loop over-samples a slow-motion sim and the balance
        # policy topples. Halving to 2 substeps drops the loop under the 20 ms budget so
        # the real-time throttle in sim_main pins RTF~=1.0 (correct timing beats 200 Hz
        # fidelity). GPU physics is *worse* here (single-env kernel-launch overhead).
        # 100 Hz physics, 50 Hz control. decim=1 (50 Hz physics) is too coarse -> foot
        # contact explodes to nan; decim=2 is the stable floor. The per-step cost is a
        # ~21 ms FIXED overhead (USD/articulation sync for the DOF count on CPU), not
        # substeps, so RTF is capped ~0.85 until the DOF count itself is reduced.
        # Physics rate override: MuJoCo/SONIC-training run 200 Hz physics (dt=0.005),
        # Isaac defaults to 100 Hz (dt=0.01, decim=2) here for RTF. The controller balances
        # only marginally in Isaac (holds ~1s then diverges); doubling the physics rate to
        # match training can restore the stability margin. SIM_DT / SIM_DECIM override
        # (keep dt*decim = 0.02 s so control stays 50 Hz == the deploy loop).
        import os as _os_solver
        self.decimation = int(_os_solver.environ.get("SIM_DECIM", "2"))
        self.episode_length_s = 1.0e9  # effectively never time out
        self.sim.dt = float(_os_solver.environ.get("SIM_DT", "0.01"))
        # Trim the contact solver to get the CPU loop under the 20 ms budget so the
        # throttle can pin RTF~=1.0 (was 4 -> ~22 ms/loop -> RTF 0.875). 2 position
        # iterations still resolves a flat-ground stand cleanly.
        import os as _os_solver
        # Optional: swap in a robot USD whose FEET are a flat box sole (matching MuJoCo)
        # instead of the stock 4 rigid point-spheres. Point contacts under the rigid PhysX
        # solver can't smoothly modulate center-of-pressure -> SONIC's ankle can't arrest
        # pitch and the robot free-falls despite correct commands. See g1..._footbox.usd
        # (built by /tmp/fix_feet.py: de-instanced, 4 corner spheres deactivated, box
        # half-extents 0.085,0.03,0.005 @ (0.035,0,-0.03) added to each ankle_roll_link).
        _footbox = _os_solver.environ.get("SIM_FOOT_BOX_USD", "")
        if _footbox:
            self.scene.robot.spawn.usd_path = _footbox
            print(f"[sim] FOOT-BOX USD override: {_footbox}", flush=True)
        self.scene.robot.spawn.articulation_props.solver_position_iteration_count = int(
            _os_solver.environ.get("SIM_SOLVER_POS", "2"))
        self.scene.robot.spawn.articulation_props.solver_velocity_iteration_count = int(
            _os_solver.environ.get("SIM_SOLVER_VEL", "1"))

        # MATCH MuJoCo joint passive params. The MuJoCo model SONIC balances on
        # (g1_29dof_with_hand.xml <default>) gives EVERY joint armature=0.01,
        # frictionloss=0.2 (wrist/finger 0.1), damping=0.05. Isaac's ImplicitActuatorCfg
        # left armature=None (->0) and no joint friction, so the same low-kd (1.8-6.3)
        # SONIC PD is underdamped/under-resisted in Isaac -> topples on release. armature
        # (reflected rotor inertia) and friction (joint dry friction) persist (SONIC does
        # NOT overwrite them; it only writes kp/kd). Passive damping 0.05 can't be added
        # via the actuator (its `damping` == the PD kd, overwritten each step) -- negligible
        # vs kd~6 anyway. Sweep/disable via SIM_ARMATURE / SIM_JOINT_FRICTION (-1 = leave).
        _arm = float(_os_solver.environ.get("SIM_ARMATURE", "0.01"))
        _fric = float(_os_solver.environ.get("SIM_JOINT_FRICTION", "0.2"))
        for _ac in self.scene.robot.actuators.values():
            if _arm >= 0.0:
                _ac.armature = _arm
            if _fric >= 0.0:
                _ac.friction = _fric
        # Contact-report generation for every body is pure overhead here (balance reads
        # contact via lowstate, not Isaac contact sensors) and adds to the per-step cost.
        self.scene.robot.spawn.activate_contact_sensors = False
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
        self.sim.physics_material.static_friction = 1.0
        self.sim.physics_material.dynamic_friction = 1.0
        self.sim.physics_material.friction_combine_mode = "max"
        self.sim.physics_material.restitution_combine_mode = "max"
        # COMPLIANT (soft) ground contact to approximate MuJoCo's soft solref/solimp
        # contact model. PhysX defaults to RIGID contact; the MuJoCo-trained SONIC policy
        # is tuned to soft foot contact, and rigid contact is the leading suspect for why
        # it under-reacts and topples in Isaac despite matched gains/obs/timing. Lower
        # stiffness = softer. Tunable via SIM_CONTACT_K / SIM_CONTACT_D.
        import os as _os
        _ck = float(_os.environ.get("SIM_CONTACT_K", "0"))
        if _ck > 0.0:
            self.sim.physics_material.compliant_contact_stiffness = _ck
            self.sim.physics_material.compliant_contact_damping = float(_os.environ.get("SIM_CONTACT_D", "2000"))

        # a "reset everything to default" event (no object in this scene)
        self.event_manager = SimpleEventManager()
        self.event_manager.register("reset_all_self", SimpleEvent(
            func=lambda env: base_mdp.reset_scene_to_default(
                env, torch.arange(env.num_envs, device=env.device))
        ))
