"""TienKung 2dex (Open-X-Humanoid) robot config for SONIC.

31 actuated DOF, 32 bodies (pelvis + 31 DOF links). Mirrors the structure of
`h2.py` (the in-repo 31-DOF reference embodiment). Sources:

- URDF / meshes : Open-X-Humanoid/TienKung_URDF, tiangong2dex_urdf/
                  (commit 5c221783fb92fcc4af891ef1dc0502963caf2266)
- MJCF          : same repo, tiangong2dex_torq.xml (fixed sensor/tcp bodies
                  stripped so it is 32 bodies; see mjcf/tiangong2dex.xml header)
- Actuator gains: legs/feet/waist/shoulder/elbow_pitch stiffness+damping from
                  Open-X-Humanoid/TienKung-Lab `dev`
                  (commit 164f615002b7ce7ae393a25f9397d27955c9dd6d),
                  legged_lab/assets/EVT2/tiangong.py DEX_V3_CFG (23-DOF
                  locomotion config for this exact URDF). elbow_yaw + wrist
                  gains are VENDOR-VERIFIED from Open-X-Humanoid/Deploy_Tienkung
                  `3.0` mimic-policy deploy configs (beyondzero.yaml /
                  BeyondMimic.yaml, identical arm kps/kds). Effort/velocity
                  limits are taken from the URDF (NOT DEX_V3_CFG, whose effort
                  table diverges).
- Armature      : VENDOR-VERIFIED per-joint rotor inertia from
                  Open-X-Humanoid/xSIM_MUJOCO resources/evt2/urdf/evt2.xml
                  <default> motor classes (head has no vendor value — small
                  guess kept). Joint friction 0.1 from evt2.xml frictionloss.

Naming differs from Unitree: side is a SUFFIX (`hip_pitch_l_joint`), there is
no `torso_link` (its equivalent is `waist_pitch_link`), the elbow is split into
`elbow_pitch`+`elbow_yaw`, and each arm chain ends at `wrist_roll_{l,r}_link`.
"""

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
import isaaclab.sim as sim_utils

ASSET_DIR = "gear_sonic/data/assets"

# ---------------------------------------------------------------------------
# Joint / body ordering and IsaacLab <-> MuJoCo index mappings
# ---------------------------------------------------------------------------
# TIANGONG2DEX_ISAACLAB_JOINTS is the *body* list (root pelvis + 31 DOF links)
# in Isaac Lab traversal order. It was produced by a breadth-first traversal of
# the URDF kinematic tree, enqueuing each body's children in URDF joint
# *document* order and skipping fixed-joint child bodies (Isaac Lab merges fixed
# joints into their parent). That exact rule was FIRST validated by
# reproducing H2's committed `H2_ISAACLAB_JOINTS` from h2.urdf byte-for-byte,
# then applied unchanged to tiangong2dex.urdf.
#
# The MuJoCo order is the document order of the <body>/<joint> elements in the
# MJCF worldbody (mjcf/tiangong2dex.xml), excluding the fixed sensor bodies.
#
# NOTE: because TienKung 2dex and H2 are kinematically isomorphic (2x6-DOF legs,
# a 3-DOF waist chain, a 2-DOF head, 2x7-DOF arms, listed in the same order in
# both MJCFs), the four index arrays below come out numerically IDENTICAL to
# H2's committed arrays. That is expected and is corroborating evidence, not a
# copy-paste — they were generated independently from the tiangong files.
#
# !!! CLUSTER VERIFICATION REQUIRED BEFORE ANY TRAINING !!!
# The Isaac Lab side of the ordering is inferred, not observed. Load
# tiangong2dex.urdf in Isaac Lab, print `robot.body_names` / `robot.joint_names`,
# and confirm they equal TIANGONG2DEX_ISAACLAB_JOINTS (bodies) and its
# non-root joints (DOF). Isaac Lab's URDF importer must also merge the 7 fixed
# sensor/tcp bodies (as it does for H2's 2 rubber-hand bodies) for the 32-body
# count to hold. If either fails, these arrays are WRONG and must be regenerated.

TIANGONG2DEX_ISAACLAB_TO_MUJOCO_MAPPING = {}

TIANGONG2DEX_ISAACLAB_JOINTS = [
    "pelvis",
    "hip_pitch_l_link",
    "hip_pitch_r_link",
    "waist_yaw_link",
    "hip_roll_l_link",
    "hip_roll_r_link",
    "waist_roll_link",
    "hip_yaw_l_link",
    "hip_yaw_r_link",
    "waist_pitch_link",
    "knee_pitch_l_link",
    "knee_pitch_r_link",
    "head_yaw_link",
    "shoulder_pitch_l_link",
    "shoulder_pitch_r_link",
    "ankle_pitch_l_link",
    "ankle_pitch_r_link",
    "head_pitch_link",
    "shoulder_roll_l_link",
    "shoulder_roll_r_link",
    "ankle_roll_l_link",
    "ankle_roll_r_link",
    "shoulder_yaw_l_link",
    "shoulder_yaw_r_link",
    "elbow_pitch_l_link",
    "elbow_pitch_r_link",
    "elbow_yaw_l_link",
    "elbow_yaw_r_link",
    "wrist_pitch_l_link",
    "wrist_pitch_r_link",
    "wrist_roll_l_link",
    "wrist_roll_r_link",
]
# Index arrays: to produce MuJoCo-ordered data from IsaacLab-ordered data,
# out = data[MAPPING]; i.e. MAPPING[mujoco_pos] = isaaclab_index. (Same
# convention as h2.py / order_converter.convert.)
TIANGONG2DEX_ISAACLAB_TO_MUJOCO_DOF = [
    0,
    3,
    6,
    9,
    14,
    19,
    1,
    4,
    7,
    10,
    15,
    20,
    2,
    5,
    8,
    11,
    16,
    12,
    17,
    21,
    23,
    25,
    27,
    29,
    13,
    18,
    22,
    24,
    26,
    28,
    30,
]
TIANGONG2DEX_MUJOCO_TO_ISAACLAB_DOF = [
    0,
    6,
    12,
    1,
    7,
    13,
    2,
    8,
    14,
    3,
    9,
    15,
    17,
    24,
    4,
    10,
    16,
    18,
    25,
    5,
    11,
    19,
    26,
    20,
    27,
    21,
    28,
    22,
    29,
    23,
    30,
]
TIANGONG2DEX_ISAACLAB_TO_MUJOCO_BODY = [
    0,
    1,
    4,
    7,
    10,
    15,
    20,
    2,
    5,
    8,
    11,
    16,
    21,
    3,
    6,
    9,
    12,
    17,
    13,
    18,
    22,
    24,
    26,
    28,
    30,
    14,
    19,
    23,
    25,
    27,
    29,
    31,
]
TIANGONG2DEX_MUJOCO_TO_ISAACLAB_BODY = [
    0,
    1,
    7,
    13,
    2,
    8,
    14,
    3,
    9,
    15,
    4,
    10,
    16,
    18,
    25,
    5,
    11,
    17,
    19,
    26,
    6,
    12,
    20,
    27,
    21,
    28,
    22,
    29,
    23,
    30,
    24,
    31,
]

TIANGONG2DEX_ISAACLAB_TO_MUJOCO_MAPPING = {
    "isaaclab_joints": TIANGONG2DEX_ISAACLAB_JOINTS,
    "isaaclab_to_mujoco_dof": TIANGONG2DEX_ISAACLAB_TO_MUJOCO_DOF,
    "mujoco_to_isaaclab_dof": TIANGONG2DEX_MUJOCO_TO_ISAACLAB_DOF,
    "isaaclab_to_mujoco_body": TIANGONG2DEX_ISAACLAB_TO_MUJOCO_BODY,
    "mujoco_to_isaaclab_body": TIANGONG2DEX_MUJOCO_TO_ISAACLAB_BODY,
}

# ---------------------------------------------------------------------------
# Armature (rotor inertia) — VENDOR VALUES
# ---------------------------------------------------------------------------
# Source: Open-X-Humanoid/xSIM_MUJOCO resources/evt2/urdf/evt2.xml <default>
#   joint classes:
#   https://github.com/Open-X-Humanoid/xSIM_MUJOCO/blob/main/resources/evt2/urdf/evt2.xml
# Each MJCF <default class="*_motor"> sets a per-class `armature`. Values below
# are keyed by that motor class and applied to the joints the evt2 body tree
# assigns to each class (verified firsthand from every joint's `class=` attr).
#
# evt2 arm-chain class map (verified from evt2.xml joint `class=`, BOTH arms):
#   shoulder_pitch/roll/yaw + elbow_pitch + elbow_yaw -> arm_motor (0.1);
#   ONLY wrist_pitch/wrist_roll -> wrist_motor (0.0236).
#   >>> elbow_yaw is arm_motor, NOT wrist_motor. It shares the wrist's 25 Nm
#   >>> effort cap but a larger rotor. (The prior audit/task expected 0.0236
#   >>> here — evt2.xml says 0.1; firsthand verification wins.)
#
# NOTE: this CONTRADICTS TienKung-Lab, which defines no armature and instead
# randomizes joint armature over abs (0.002, 0.060) at reset (walk_cfg.py). The
# vendor MuJoCo model is authoritative for the physical rotor inertia, so it is
# the nominal here; the TienKung-Lab band is a training-time DR choice.
ARM_HIP_PITCH = 0.24  # evt2 hip_pitch_motor
ARM_HIP_ROLL = 0.24  # evt2 hip_roll_motor
ARM_HIP_YAW = 0.18  # evt2 hip_yaw_motor
ARM_KNEE = 0.37  # evt2 knee_motor
ARM_ANKLE = 0.032  # evt2 ankle_motor (ankle_pitch / ankle_roll)
ARM_WAIST = 0.17  # evt2 waist_motor (waist_yaw / roll / pitch)
ARM_ARM = 0.1  # evt2 arm_motor (shoulder_pitch/roll/yaw, elbow_pitch, elbow_yaw)
ARM_WRIST = 0.0236  # evt2 wrist_motor (wrist_pitch / wrist_roll)
# Head has NO vendor value anywhere (evt2 models no head joint). Keep a small
# guess; TienKung-Lab's (0.002, 0.060) armature DR band is the only nearby
# reference and head sits at its floor.
ARM_HEAD = 0.002  # GUESS — no vendor value exists for head_yaw / head_pitch

# ---------------------------------------------------------------------------
# Articulation config
# ---------------------------------------------------------------------------
# Stiffness/damping: legs/feet/waist/shoulder/elbow_pitch from TienKung-Lab
#   DEX_V3_CFG (starting point, NOT ground truth; its waist gains are
#   placeholders). elbow_yaw + wrist_pitch + wrist_roll are now VENDOR-VERIFIED
#   from Deploy_Tienkung 3.0 mimic-policy deploy configs (beyondzero.yaml /
#   BeyondMimic.yaml, identical arm kps/kds):
#   https://github.com/Open-X-Humanoid/Deploy_Tienkung/blob/3.0/policy/beyond_mimic/config/BeyondMimic.yaml
# Armature: VENDOR-VERIFIED from xSIM_MUJOCO evt2.xml (see ARM_* block above).
# Joint friction: 0.1 on every group, from evt2.xml frictionloss default.
# Effort/velocity limits: parsed from the vendored URDF.
# head gains (20/1) and head armature (0.002) remain guesses (no vendor head).
TIANGONG2DEX_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        replace_cylinders_with_capsules=True,
        asset_path=f"{ASSET_DIR}/robot_description/urdf/tiangong2dex/tiangong2dex.urdf",
        activate_contact_sensors=True,
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
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # DEX_V3_CFG default standing pose (z=0.97). rest defaults to 0.0.
        pos=(0.0, 0.0, 0.97),
        joint_pos={
            ".*hip_pitch.*joint": -0.15,
            ".*knee_pitch.*joint": 0.3,
            ".*ankle_pitch.*joint": -0.15,
            ".*shoulder_pitch.*joint": 0.2,
            "shoulder_roll_l_joint": 0.1,
            "shoulder_roll_r_joint": -0.1,
            ".*elbow_pitch.*joint": -0.5,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*hip_yaw.*joint",
                ".*hip_roll.*joint",
                ".*hip_pitch.*joint",
                ".*knee_pitch.*joint",
            ],
            effort_limit_sim={  # URDF
                ".*hip_yaw.*joint": 150.0,
                ".*hip_roll.*joint": 235.0,
                ".*hip_pitch.*joint": 235.0,
                ".*knee_pitch.*joint": 400.0,
            },
            velocity_limit_sim={  # URDF
                ".*hip_yaw.*joint": 13.823,
                ".*hip_roll.*joint": 16.755,
                ".*hip_pitch.*joint": 16.755,
                ".*knee_pitch.*joint": 11.100,
            },
            stiffness={  # DEX_V3_CFG
                ".*hip_yaw.*joint": 150.0,
                ".*hip_roll.*joint": 300.0,
                ".*hip_pitch.*joint": 300.0,
                ".*knee_pitch.*joint": 330.0,
            },
            damping={  # DEX_V3_CFG
                ".*hip_yaw.*joint": 5.0,
                ".*hip_roll.*joint": 10.0,
                ".*hip_pitch.*joint": 10.0,
                ".*knee_pitch.*joint": 10.0,
            },
            armature={  # evt2.xml motor classes
                ".*hip_yaw.*joint": ARM_HIP_YAW,
                ".*hip_roll.*joint": ARM_HIP_ROLL,
                ".*hip_pitch.*joint": ARM_HIP_PITCH,
                ".*knee_pitch.*joint": ARM_KNEE,
            },
            friction=0.1,  # evt2.xml frictionloss default
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*ankle_pitch.*joint", ".*ankle_roll.*joint"],
            effort_limit_sim={  # URDF
                ".*ankle_pitch.*joint": 55.0,
                ".*ankle_roll.*joint": 55.0,
            },
            velocity_limit_sim={  # URDF
                ".*ankle_pitch.*joint": 14.137,
                ".*ankle_roll.*joint": 14.137,
            },
            stiffness={  # DEX_V3_CFG
                ".*ankle_pitch.*joint": 30.0,
                ".*ankle_roll.*joint": 16.8,
            },
            damping={  # DEX_V3_CFG
                ".*ankle_pitch.*joint": 2.5,
                ".*ankle_roll.*joint": 1.4,
            },
            armature={  # evt2.xml ankle_motor
                ".*ankle_pitch.*joint": ARM_ANKLE,
                ".*ankle_roll.*joint": ARM_ANKLE,
            },
            friction=0.1,  # evt2.xml frictionloss default
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"],
            effort_limit_sim={  # URDF
                "waist_yaw_joint": 91.0,
                "waist_roll_joint": 150.0,
                "waist_pitch_joint": 150.0,
            },
            velocity_limit_sim={  # URDF
                "waist_yaw_joint": 9.425,
                "waist_roll_joint": 13.823,
                "waist_pitch_joint": 13.823,
            },
            stiffness={  # DEX_V3_CFG (marked placeholder in TienKung-Lab)
                "waist_yaw_joint": 400.0,
                "waist_roll_joint": 400.0,
                "waist_pitch_joint": 400.0,
            },
            damping={  # DEX_V3_CFG (marked placeholder in TienKung-Lab)
                "waist_yaw_joint": 5.0,
                "waist_roll_joint": 10.0,
                "waist_pitch_joint": 10.0,
            },
            armature={  # evt2.xml waist_motor (uniform 0.17)
                "waist_yaw_joint": ARM_WAIST,
                "waist_roll_joint": ARM_WAIST,
                "waist_pitch_joint": ARM_WAIST,
            },
            friction=0.1,  # evt2.xml frictionloss default
        ),
        "head": ImplicitActuatorCfg(
            # GUESS: DEX_V3_CFG has no head (head fixed there) and evt2.xml models
            # no head joint. Small gains + armature, effort/velocity from URDF.
            joint_names_expr=["head_yaw_joint", "head_pitch_joint"],
            effort_limit_sim=6.3,  # URDF
            velocity_limit_sim=7.645,  # URDF
            stiffness=20.0,  # GUESS
            damping=1.0,  # GUESS
            armature=ARM_HEAD,  # GUESS (no vendor head armature)
            friction=0.1,  # evt2.xml frictionloss default (extrapolated to head)
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*shoulder_pitch.*joint",
                ".*shoulder_roll.*joint",
                ".*shoulder_yaw.*joint",
                ".*elbow_pitch.*joint",
                ".*elbow_yaw.*joint",
                ".*wrist_pitch.*joint",
                ".*wrist_roll.*joint",
            ],
            effort_limit_sim={  # URDF
                ".*shoulder_pitch.*joint": 90.0,
                ".*shoulder_roll.*joint": 90.0,
                ".*shoulder_yaw.*joint": 50.0,
                ".*elbow_pitch.*joint": 50.0,
                ".*elbow_yaw.*joint": 25.0,
                ".*wrist_pitch.*joint": 25.0,
                ".*wrist_roll.*joint": 25.0,
            },
            velocity_limit_sim={  # URDF
                ".*shoulder_pitch.*joint": 7.218,
                ".*shoulder_roll.*joint": 7.218,
                ".*shoulder_yaw.*joint": 11.455,
                ".*elbow_pitch.*joint": 11.455,
                ".*elbow_yaw.*joint": 12.218,
                ".*wrist_pitch.*joint": 12.218,
                ".*wrist_roll.*joint": 12.218,
            },
            # stiffness/damping: shoulder + elbow_pitch from TienKung-Lab DEX_V3_CFG;
            # elbow_yaw + wrist_pitch + wrist_roll VENDOR-VERIFIED from
            # Deploy_Tienkung 3.0 mimic-policy deploy configs (beyondzero.yaml /
            # BeyondMimic.yaml, identical arm kps/kds):
            # https://github.com/Open-X-Humanoid/Deploy_Tienkung/blob/3.0/policy/beyond_mimic/config/BeyondMimic.yaml
            stiffness={
                ".*shoulder_pitch.*joint": 150.0,
                ".*shoulder_roll.*joint": 50.0,
                ".*shoulder_yaw.*joint": 50.0,
                ".*elbow_pitch.*joint": 150.0,
                ".*elbow_yaw.*joint": 150.0,  # Deploy_Tienkung 3.0 mimic (was 50.0 GUESS)
                ".*wrist_pitch.*joint": 200.0,  # Deploy_Tienkung 3.0 mimic (was 20.0 GUESS)
                ".*wrist_roll.*joint": 200.0,  # Deploy_Tienkung 3.0 mimic (was 20.0 GUESS)
            },
            damping={
                ".*shoulder_pitch.*joint": 5.0,
                ".*shoulder_roll.*joint": 2.5,
                ".*shoulder_yaw.*joint": 2.5,
                ".*elbow_pitch.*joint": 5.0,
                ".*elbow_yaw.*joint": 5.0,  # Deploy_Tienkung 3.0 mimic (matches prior value)
                ".*wrist_pitch.*joint": 2.0,  # Deploy_Tienkung 3.0 mimic (matches prior value)
                ".*wrist_roll.*joint": 2.0,  # Deploy_Tienkung 3.0 mimic (matches prior value)
            },
            armature={  # evt2.xml: arm_motor (0.1) for shoulders+elbow_pitch+elbow_yaw; wrist_motor (0.0236) for wrist_pitch/roll
                ".*shoulder_pitch.*joint": ARM_ARM,
                ".*shoulder_roll.*joint": ARM_ARM,
                ".*shoulder_yaw.*joint": ARM_ARM,
                ".*elbow_pitch.*joint": ARM_ARM,
                ".*elbow_yaw.*joint": ARM_ARM,  # arm_motor 0.1, NOT wrist_motor (verified evt2.xml)
                ".*wrist_pitch.*joint": ARM_WRIST,
                ".*wrist_roll.*joint": ARM_WRIST,
            },
            friction=0.1,  # evt2.xml frictionloss default
        ),
    },
)

# TienKung 2dex Action Scale (same derivation as H2: 0.25 * effort / stiffness).
TIANGONG2DEX_ACTION_SCALE = {}
for a in TIANGONG2DEX_CFG.actuators.values():
    e = a.effort_limit_sim
    s = a.stiffness
    names = a.joint_names_expr
    if not isinstance(e, dict):
        e = dict.fromkeys(names, e)
    if not isinstance(s, dict):
        s = dict.fromkeys(names, s)
    for n in names:
        if n in e and n in s and s[n]:
            TIANGONG2DEX_ACTION_SCALE[n] = 0.25 * e[n] / s[n]
