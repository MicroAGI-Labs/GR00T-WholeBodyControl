"""Minimal TienKung 2dex articulation for the mixed-body sim experiment."""

import os

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
import isaaclab.sim as sim_utils


def tiangong2dex_cfg():
    urdf = os.environ.get(
        "SIM_TIANGONG_URDF",
        os.path.expanduser("~/live-sim/assets/tiangong2dex/tiangong2dex.urdf"),
    )
    return ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/TiangongRobot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=urdf,
            fix_base=False,
            replace_cylinders_with_capsules=True,
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=2,
                solver_velocity_iteration_count=1,
            ),
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                    stiffness=0.0, damping=0.0
                )
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.97),
            joint_pos={
                ".*hip_pitch.*joint": -0.15,
                ".*knee_pitch.*joint": 0.30,
                ".*ankle_pitch.*joint": -0.15,
                ".*shoulder_pitch.*joint": 0.20,
                "shoulder_roll_l_joint": 0.10,
                "shoulder_roll_r_joint": -0.10,
                ".*elbow_pitch.*joint": -0.50,
                "head_yaw_joint": 0.0,
                "head_pitch_joint": 0.0,
            },
            joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=0.9,
        actuators={
            # The G1 LowCmd kp/kd values replace these for the 29 mapped joints.
            # A single generous effort cap keeps this first experiment small;
            # the URDF still supplies its physical joint limits.
            "all": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                effort_limit_sim=400.0,
                velocity_limit_sim=20.0,
                stiffness=20.0,
                damping=1.0,
                armature=0.01,
                friction=0.1,
            )
        },
    )
