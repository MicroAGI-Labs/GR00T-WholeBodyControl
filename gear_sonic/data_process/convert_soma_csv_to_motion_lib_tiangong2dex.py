#!/usr/bin/env python3  # noqa: EXE001
# ruff: noqa: T201, DOC
"""Convert SOMA-retargeter CSV/PKL data to motion_lib format for TienKung 2dex SONIC.

31-DOF sibling of ``convert_soma_csv_to_motion_lib.py`` (which is G1 29-DOF and is
left untouched). Same structure / CLI / output schema; the only differences are
the robot-specific constants (DOF_AXIS, DOF/body counts, IsaacLab<->MuJoCo
reorder, joint/body name lists). Produces the motion_lib PKL fields SONIC training
expects: root_trans_offset, pose_aa, dof, root_rot, smpl_joints, fps.

Supported input modes (identical to the G1 script):
  1. Single motion dir with CSVs (joint_pos.csv, body_pos.csv, body_quat.csv)
  2. Parent dir of such motion subdirs
  3. Deploy PKL (joblib dict: joint_pos, body_pos_w, body_quat_w per sequence)
  4. Directory of flat SOMA-retargeter CSVs (one CSV per motion, degrees+cm)
  5. Parent dir of session dirs containing flat CSVs

The flat-CSV (mode 4/5) is what the current NVIDIA soma-retargeter actually emits
via ``save_csv`` (Frame, root_translate{X,Y,Z} cm, root_rotate{X,Y,Z} euler-deg,
then the DOF columns in deg, MuJoCo/actuator order). See ``load_bones_csv``.

=============================================================================
QUATERNION CONVENTION VERDICT (resolved by reading the motion_lib loader):
=============================================================================
The motion_lib loader drives forward kinematics from ``pose_aa`` and
``root_trans_offset`` ONLY. It NEVER reads the stored ``root_rot`` field:
  - gear_sonic/utils/motion_lib/motion_lib_base.py:1768-1769
        trans    = curr_file["root_trans_offset"]
        pose_aa  = curr_file["pose_aa"]
  - motion_lib_base.py:1801-1804  root orientation is handled as an axis-angle
        ROTVEC taken from pose_aa[:, :3]  (transform.Rotation.from_rotvec(...))
  - motion_lib_base.py:1873        fk_batch(pose_aa, trans)   # no root_rot passed
  - torch_humanoid_batch.py:390    pose_quat = axis_angle_to_quaternion(pose)
        (root at body index 0; converted quat->matrix for FK)
  A repo-wide grep for a stored-``root_rot`` reader finds only this family of
  converters' own ``downsample_sequence``; nothing in gear_sonic consumes it.

Consequences, and what THIS script emits (identical to the G1 script):
  - The load-time INPUT that matters is ``body_quat_w[:, 0]``, which MUST be in
    WXYZ order (this is what the new-embodiments guide means by "wxyz"). We
    reindex WXYZ -> XYZW via [1,2,3,0] and feed scipy ``Rotation.from_quat``
    (which requires XYZW) to build the root rotvec pose_aa[:, 0]. That rotvec is
    the only thing FK uses, so it is convention-correct as long as the input is
    genuinely wxyz.
  - The OUTPUT ``root_rot`` field is written in XYZW (scipy) for parity with the
    G1 script and any external reader, but it is FUNCTIONALLY VESTIGIAL for
    training (the loader ignores it). Do not "fix" it to wxyz to match the guide:
    the guide's wxyz refers to the INPUT body quats, not this stored field.
=============================================================================

Usage (mirrors the G1 script):
    python gear_sonic/data_process/convert_soma_csv_to_motion_lib_tiangong2dex.py \
        --input  data/soma_retarget/<motion_or_dir> \
        --output data/tiangong2dex_test.pkl --fps 30 --fps_source 120

Self-test (no external retarget data needed):
    python gear_sonic/data_process/convert_soma_csv_to_motion_lib_tiangong2dex.py --selftest
"""

import argparse
import os
import sys

import joblib
import numpy as np
from scipy.spatial import transform

# ---------------------------------------------------------------------------
# Robot constants (TienKung 2dex, 31 DOF / 32 bodies).
#
# These are kept CONSISTENT with (and cross-checked against, see --selftest)
# gear_sonic/envs/manager_env/robots/tiangong2dex.py. They are hardcoded here --
# rather than imported -- because that robot module imports isaaclab at top level,
# which is unavailable in a plain-python conversion context; the G1 script hardcodes
# its axes for the same "avoid heavy deps" reason. The selftest imports the robot
# module when isaaclab IS available and asserts these copies match it exactly.
# ---------------------------------------------------------------------------

# MJ_TO_IL[mj] = il: for MuJoCo DOF slot mj, gives the IsaacLab DOF index.
# Used as ``dof_mj = joint_pos_il[:, MJ_TO_IL]`` to turn IsaacLab-ordered joint
# angles into MuJoCo order. NOTE ON NAMING: despite the "MJ_TO_IL" name (kept for
# parity with the G1 script), this array is IDENTICAL to the robot module's
# ``TIANGONG2DEX_ISAACLAB_TO_MUJOCO_DOF`` -- the robot module names it from the
# "data[arr] transforms IL->MJ" viewpoint, while the G1 script names it from the
# "the values ARE il indices" viewpoint. Same gather array, opposite mental model.
# Verified in --selftest by deriving the gather from the MJCF joint order vs the
# IsaacLab body order.
MJ_TO_IL = np.array(
    [0, 3, 6, 9, 14, 19, 1, 4, 7, 10, 15, 20, 2, 5, 8, 11, 16, 12, 17, 21, 23, 25,
     27, 29, 13, 18, 22, 24, 26, 28, 30],
    dtype=np.int32,
)

NUM_DOF = 31
NUM_BODIES = 32  # pelvis + 31 actuated links

# DOF_AXIS: rotation axis of each DOF in MuJoCo joint (document/actuator) order.
# Machine-derived by parsing the vendored URDF <axis> entries and ordering them by
# the MJCF hinge-joint document order (URDF axes == MJCF axes, verified). Re-derived
# and asserted against the vendored MJCF in --selftest, so this is not hand-typed.
DOF_AXIS = np.array(
    [
        [0, 1, 0],  # 0  hip_pitch_l
        [1, 0, 0],  # 1  hip_roll_l
        [0, 0, 1],  # 2  hip_yaw_l
        [0, 1, 0],  # 3  knee_pitch_l
        [0, 1, 0],  # 4  ankle_pitch_l
        [1, 0, 0],  # 5  ankle_roll_l
        [0, 1, 0],  # 6  hip_pitch_r
        [1, 0, 0],  # 7  hip_roll_r
        [0, 0, 1],  # 8  hip_yaw_r
        [0, 1, 0],  # 9  knee_pitch_r
        [0, 1, 0],  # 10 ankle_pitch_r
        [1, 0, 0],  # 11 ankle_roll_r
        [0, 0, 1],  # 12 waist_yaw
        [1, 0, 0],  # 13 waist_roll
        [0, 1, 0],  # 14 waist_pitch
        [0, 0, 1],  # 15 head_yaw
        [0, 1, 0],  # 16 head_pitch
        [0, 1, 0],  # 17 shoulder_pitch_l
        [1, 0, 0],  # 18 shoulder_roll_l
        [0, 0, 1],  # 19 shoulder_yaw_l
        [0, 1, 0],  # 20 elbow_pitch_l
        [0, 0, 1],  # 21 elbow_yaw_l
        [0, 1, 0],  # 22 wrist_pitch_l
        [1, 0, 0],  # 23 wrist_roll_l
        [0, 1, 0],  # 24 shoulder_pitch_r
        [1, 0, 0],  # 25 shoulder_roll_r
        [0, 0, 1],  # 26 shoulder_yaw_r
        [0, 1, 0],  # 27 elbow_pitch_r
        [0, 0, 1],  # 28 elbow_yaw_r
        [0, 1, 0],  # 29 wrist_pitch_r
        [1, 0, 0],  # 30 wrist_roll_r
    ],
    dtype=np.float32,
)

# Joint names in MuJoCo (MJCF actuator / document) order -- the order the flat SOMA
# CSV DOF columns are written in. Consistent with the MJCF and the robot module.
MUJOCO_JOINT_NAMES = [
    "hip_pitch_l_joint", "hip_roll_l_joint", "hip_yaw_l_joint", "knee_pitch_l_joint",
    "ankle_pitch_l_joint", "ankle_roll_l_joint", "hip_pitch_r_joint", "hip_roll_r_joint",
    "hip_yaw_r_joint", "knee_pitch_r_joint", "ankle_pitch_r_joint", "ankle_roll_r_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", "head_yaw_joint",
    "head_pitch_joint", "shoulder_pitch_l_joint", "shoulder_roll_l_joint",
    "shoulder_yaw_l_joint", "elbow_pitch_l_joint", "elbow_yaw_l_joint",
    "wrist_pitch_l_joint", "wrist_roll_l_joint", "shoulder_pitch_r_joint",
    "shoulder_roll_r_joint", "shoulder_yaw_r_joint", "elbow_pitch_r_joint",
    "elbow_yaw_r_joint", "wrist_pitch_r_joint", "wrist_roll_r_joint",
]

# Body names in MuJoCo (MJCF worldbody document) order (pelvis + 31 DOF links).
MUJOCO_BODY_NAMES = [
    "pelvis", "hip_pitch_l_link", "hip_roll_l_link", "hip_yaw_l_link", "knee_pitch_l_link",
    "ankle_pitch_l_link", "ankle_roll_l_link", "hip_pitch_r_link", "hip_roll_r_link",
    "hip_yaw_r_link", "knee_pitch_r_link", "ankle_pitch_r_link", "ankle_roll_r_link",
    "waist_yaw_link", "waist_roll_link", "waist_pitch_link", "head_yaw_link",
    "head_pitch_link", "shoulder_pitch_l_link", "shoulder_roll_l_link", "shoulder_yaw_l_link",
    "elbow_pitch_l_link", "elbow_yaw_l_link", "wrist_pitch_l_link", "wrist_roll_l_link",
    "shoulder_pitch_r_link", "shoulder_roll_r_link", "shoulder_yaw_r_link",
    "elbow_pitch_r_link", "elbow_yaw_r_link", "wrist_pitch_r_link", "wrist_roll_r_link",
]

# IsaacLab body order (pelvis + 31 links); kept consistent with the robot module's
# TIANGONG2DEX_ISAACLAB_JOINTS. Used by --selftest to re-derive MJ_TO_IL from names.
_ISAACLAB_BODY_ORDER = [
    "pelvis", "hip_pitch_l_link", "hip_pitch_r_link", "waist_yaw_link", "hip_roll_l_link",
    "hip_roll_r_link", "waist_roll_link", "hip_yaw_l_link", "hip_yaw_r_link", "waist_pitch_link",
    "knee_pitch_l_link", "knee_pitch_r_link", "head_yaw_link", "shoulder_pitch_l_link",
    "shoulder_pitch_r_link", "ankle_pitch_l_link", "ankle_pitch_r_link", "head_pitch_link",
    "shoulder_roll_l_link", "shoulder_roll_r_link", "ankle_roll_l_link", "ankle_roll_r_link",
    "shoulder_yaw_l_link", "shoulder_yaw_r_link", "elbow_pitch_l_link", "elbow_pitch_r_link",
    "elbow_yaw_l_link", "elbow_yaw_r_link", "wrist_pitch_l_link", "wrist_pitch_r_link",
    "wrist_roll_l_link", "wrist_roll_r_link",
]

# Flat-CSV DOF column names (MuJoCo order). Documents the expected column order for
# the SOMA-retargeter output; load_bones_csv reads DOF columns POSITIONALLY (see
# note there) so it is robust to header naming, but this is the intended header.
BONES_CSV_JOINT_NAMES = [n + "_dof" for n in MUJOCO_JOINT_NAMES]


def load_bones_csv(csv_path: str) -> dict:
    """Load a single flat SOMA-retargeter CSV motion file.

    Format (from soma_retargeter save_csv): Frame, root_translate{X,Y,Z} (cm),
    root_rotate{X,Y,Z} (euler xyz intrinsic, deg), then NUM_DOF DOF columns (deg),
    in MuJoCo/actuator order.

    NOTE: columns are read POSITIONALLY (Frame + 3 trans + 3 rot + NUM_DOF dof),
    NOT by header name. The current soma-retargeter ships only a 29-DOF G1 CSV
    header (UnitreeG129DOF_CSVConfig), so a 31-DOF run emits correct DATA rows but
    a G1-shaped/short HEADER. Positional extraction is robust to that; it also
    consumes a properly-headed 31-DOF CSV identically. (A dedicated 31-DOF CSV
    config for the retargeter is a separate follow-up -- see retarget/README.md.)
    """
    import pandas as pd

    data = pd.read_csv(csv_path)
    arr = data.values.astype(np.float64)
    T = arr.shape[0]
    ncols = arr.shape[1]
    expected = 1 + 6 + NUM_DOF
    if ncols < expected:
        raise ValueError(
            f"{csv_path}: expected >= {expected} columns (Frame + 6 root + {NUM_DOF} dof), got {ncols}"
        )

    # Root position: cm -> meters
    root_pos = (arr[:, 1:4] / 100.0).astype(np.float32)

    # Root rotation: euler xyz (intrinsic) deg -> quat (xyzw scipy) -> wxyz
    euler_deg = arr[:, 4:7]
    root_quat_xyzw = (
        transform.Rotation.from_euler("xyz", euler_deg, degrees=True).as_quat().astype(np.float32)
    )
    root_quat_wxyz = root_quat_xyzw[:, [3, 0, 1, 2]]

    # Joint DOFs: deg -> rad, already in MuJoCo order (positional slice)
    joint_pos_mj = np.deg2rad(arr[:, 7 : 7 + NUM_DOF]).astype(np.float32)  # (T, NUM_DOF)

    # Dummy body arrays: only body 0 (root) is read by convert_sequence.
    body_pos_w = np.zeros((T, NUM_BODIES, 3), dtype=np.float32)
    body_pos_w[:, 0, :] = root_pos
    body_quat_w = np.zeros((T, NUM_BODIES, 4), dtype=np.float32)
    body_quat_w[:, :, 0] = 1.0  # identity quaternion wxyz for non-root bodies
    body_quat_w[:, 0, :] = root_quat_wxyz

    return {
        "joint_pos": joint_pos_mj,  # (T, NUM_DOF) MuJoCo order, radians
        "body_pos_w": body_pos_w,  # (T, NUM_BODIES, 3)
        "body_quat_w": body_quat_w,  # (T, NUM_BODIES, 4) wxyz
        "joint_order": "mj",  # already MuJoCo order, skip IL->MJ reorder
    }


def load_csv_motion(motion_dir: str) -> dict:
    """Load a single motion from a directory of CSV files (joint_pos/body_pos/body_quat)."""
    joint_pos_f = os.path.join(motion_dir, "joint_pos.csv")
    body_pos_f = os.path.join(motion_dir, "body_pos.csv")
    body_quat_f = os.path.join(motion_dir, "body_quat.csv")

    if not os.path.exists(joint_pos_f):
        return None

    joint_pos = np.loadtxt(joint_pos_f, delimiter=",", skiprows=1, dtype=np.float32)
    body_pos = np.loadtxt(body_pos_f, delimiter=",", skiprows=1, dtype=np.float32)
    body_quat = np.loadtxt(body_quat_f, delimiter=",", skiprows=1, dtype=np.float32)

    T = joint_pos.shape[0]
    body_pos = body_pos.reshape(T, -1, 3)
    body_quat = body_quat.reshape(T, -1, 4)

    return {
        "joint_pos": joint_pos,  # (T, NUM_DOF) IsaacLab order
        "body_pos_w": body_pos,  # (T, NB, 3) world frame
        "body_quat_w": body_quat,  # (T, NB, 4) wxyz
    }


def convert_sequence(seq_data: dict, fps: int, humanoid_fk=None) -> dict:  # noqa: ARG001
    """Convert a single sequence to motion_lib format.

    seq_data: dict with joint_pos (T, NUM_DOF), body_pos_w (T, NB, 3),
              body_quat_w (T, NB, 4 wxyz), optional joint_order ("il"|"mj").
    Returns a motion_lib entry: root_trans_offset, pose_aa, dof, root_rot,
    smpl_joints, fps. Math is identical to the G1 script (only the constants differ).
    """
    joint_pos = seq_data["joint_pos"]  # (T, NUM_DOF)
    body_pos_w = seq_data["body_pos_w"]  # (T, NB, 3)
    body_quat_w = seq_data["body_quat_w"]  # (T, NB, 4) wxyz
    joint_order = seq_data.get("joint_order", "il")  # "il" or "mj"

    T = joint_pos.shape[0]

    # 1. Root position = pelvis (body 0) position
    root_trans_offset = body_pos_w[:, 0, :].copy()  # (T, 3)

    # 2. Root quaternion: body 0, wxyz -> xyzw (scipy). See VERDICT block above:
    #    the INPUT body_quat_w[:,0] must be wxyz.
    root_quat_wxyz = body_quat_w[:, 0, :]  # (T, 4) [w, x, y, z]
    root_quat_xyzw = root_quat_wxyz[:, [1, 2, 3, 0]]  # (T, 4) [x, y, z, w]

    # 3. Reorder DOFs to MuJoCo order if the input is IsaacLab-ordered
    if joint_order == "il":
        dof_mj = joint_pos[:, MJ_TO_IL]  # (T, NUM_DOF)
    else:
        dof_mj = joint_pos  # already MuJoCo order

    # 4. DOF -> pose_aa (axis-angle) using the hardcoded tiangong2dex axes
    dof = dof_mj[:, :NUM_DOF]
    pose_aa = np.zeros((T, NUM_BODIES, 3), dtype=np.float32)
    # body idx = dof idx + 1 (body 0 = pelvis/root)
    pose_aa[:, 1:NUM_BODIES, :] = DOF_AXIS[None, :, :] * dof[:, :, None]
    # Root rotation as axis-angle (the ONLY root-orientation term FK consumes)
    pose_aa[:, 0, :] = transform.Rotation.from_quat(root_quat_xyzw).as_rotvec()

    return {
        "root_trans_offset": root_trans_offset.astype(np.float32),
        "pose_aa": pose_aa.astype(np.float32),
        "dof": dof.astype(np.float32),
        "root_rot": root_quat_xyzw.astype(np.float32),  # xyzw; VESTIGIAL (see VERDICT)
        "smpl_joints": np.zeros((T, 24, 3), dtype=np.float32),  # placeholder
        "fps": fps,
    }


def downsample_sequence(entry: dict, fps_source: int, fps_target: int) -> dict:
    """Stride-based downsample of a motion_lib entry (matches G1 script)."""
    if fps_source == fps_target:
        return entry
    jump = int(fps_source / fps_target)
    if jump <= 1:
        return entry
    return {
        "root_trans_offset": entry["root_trans_offset"][::jump],
        "pose_aa": entry["pose_aa"][::jump],
        "dof": entry["dof"][::jump],
        "root_rot": entry["root_rot"][::jump],
        "smpl_joints": entry["smpl_joints"][::jump],
        "fps": fps_target,
    }


def init_humanoid_fk():
    """Initialize Humanoid_Batch from the tiangong2dex MJCF (parity with G1 script).

    Unused by the flat-CSV path (which uses the hardcoded DOF_AXIS). Kept for
    non-bones inputs / structural parity; requires the SONIC deps to be importable.
    """
    import omegaconf

    motion_cfg = omegaconf.OmegaConf.create(
        {
            "asset": {
                "assetRoot": "gear_sonic/data/assets/robot_description/mjcf/",
                "assetFileName": "tiangong2dex.xml",
                "urdfFileName": "",
            },
            "extend_config": [],
        }
    )
    from gear_sonic.utils.motion_lib import torch_humanoid_batch

    return torch_humanoid_batch.Humanoid_Batch(motion_cfg)


def process_session_csvs(args_tuple):
    """Process all flat CSVs in one session dir. Used by multiprocessing (--individual)."""
    session_dir, session_name, out_dir, fps, fps_source = args_tuple
    import warnings

    warnings.filterwarnings("ignore")

    csv_files = sorted([f for f in os.listdir(session_dir) if f.endswith(".csv")])
    session_out = os.path.join(out_dir, session_name)
    os.makedirs(session_out, exist_ok=True)

    converted = 0
    failed = 0
    for csv_f in csv_files:
        name = os.path.splitext(csv_f)[0]
        out_path = os.path.join(session_out, name + ".pkl")
        if os.path.exists(out_path):
            converted += 1  # skip existing
            continue
        try:
            seq = load_bones_csv(os.path.join(session_dir, csv_f))
            fps_for_convert = fps_source if fps_source else fps
            entry = convert_sequence(seq, fps_for_convert)
            if fps_source and fps_source != fps:
                entry = downsample_sequence(entry, fps_source, fps)
            joblib.dump({name: entry}, out_path, compress=True)
            converted += 1
        except Exception:  # noqa: BLE001
            failed += 1
    return session_name, converted, failed, len(csv_files)


def run_selftest() -> int:
    """Fabricate tiny synthetic inputs, run the conversion, assert shapes / orders /
    quat convention against the robot constants (and the vendored MJCF if present).

    numpy + scipy required (pandas only for the bones-CSV sub-check). No retarget
    data, no isaaclab, no torch needed.
    """
    print("[selftest] tiangong2dex converter")
    ok = True

    def check(cond, msg):
        nonlocal ok
        status = "PASS" if cond else "FAIL"
        if not cond:
            ok = False
        print(f"  [{status}] {msg}")

    # --- constant consistency ---
    check(NUM_DOF == 31 and NUM_BODIES == 32, f"NUM_DOF={NUM_DOF}, NUM_BODIES={NUM_BODIES}")
    check(DOF_AXIS.shape == (NUM_DOF, 3), f"DOF_AXIS shape {DOF_AXIS.shape}")
    check(len(MUJOCO_JOINT_NAMES) == NUM_DOF, f"MUJOCO_JOINT_NAMES len {len(MUJOCO_JOINT_NAMES)}")
    check(len(MUJOCO_BODY_NAMES) == NUM_BODIES, f"MUJOCO_BODY_NAMES len {len(MUJOCO_BODY_NAMES)}")
    check(len(BONES_CSV_JOINT_NAMES) == NUM_DOF, f"BONES_CSV_JOINT_NAMES len {len(BONES_CSV_JOINT_NAMES)}")
    check(sorted(MJ_TO_IL.tolist()) == list(range(NUM_DOF)), "MJ_TO_IL is a permutation of 0..30")

    # --- MJ_TO_IL re-derived from names: gather[mj] = il_index(mujoco_joint[mj]) ---
    il_dof = [b[:-5] + "_joint" for b in _ISAACLAB_BODY_ORDER[1:]]  # link->joint
    gather = np.array([il_dof.index(j) for j in MUJOCO_JOINT_NAMES], dtype=np.int32)
    check(np.array_equal(gather, MJ_TO_IL), "MJ_TO_IL == gather derived from IsaacLab/MuJoCo joint names")

    # --- cross-check DOF_AXIS + names against the vendored MJCF if reachable ---
    here = os.path.dirname(os.path.abspath(__file__))
    mjcf = os.path.normpath(
        os.path.join(here, "..", "data", "assets", "robot_description", "mjcf", "tiangong2dex.xml")
    )
    if os.path.exists(mjcf):
        import xml.etree.ElementTree as ET

        root = ET.parse(mjcf).getroot()
        wb = root.find("worldbody")
        mj_bodies, mj_joints, mj_axis = [], [], []

        def walk(elem):
            for b in elem.findall("body"):
                mj_bodies.append(b.get("name"))
                for j in b.findall("joint"):
                    if j.get("type") == "hinge":
                        mj_joints.append(j.get("name"))
                        mj_axis.append([int(round(float(x))) for x in j.get("axis").split()])
                walk(b)

        walk(wb)
        check(mj_bodies == MUJOCO_BODY_NAMES, "MJCF body order == MUJOCO_BODY_NAMES")
        check(mj_joints == MUJOCO_JOINT_NAMES, "MJCF hinge order == MUJOCO_JOINT_NAMES")
        check(np.array_equal(np.array(mj_axis, dtype=np.float32), DOF_AXIS), "MJCF axes == DOF_AXIS")
    else:
        print(f"  [SKIP] MJCF not found at {mjcf} (run on the pod/WBC clone to exercise this)")

    # --- try importing the robot module (only works where isaaclab is installed) ---
    try:
        from gear_sonic.envs.manager_env.robots.tiangong2dex import (
            TIANGONG2DEX_ISAACLAB_JOINTS,
            TIANGONG2DEX_ISAACLAB_TO_MUJOCO_DOF,
        )

        check(list(TIANGONG2DEX_ISAACLAB_JOINTS) == _ISAACLAB_BODY_ORDER,
              "robot module ISAACLAB_JOINTS == _ISAACLAB_BODY_ORDER")
        check(np.array_equal(np.array(TIANGONG2DEX_ISAACLAB_TO_MUJOCO_DOF, dtype=np.int32), MJ_TO_IL),
              "robot module ISAACLAB_TO_MUJOCO_DOF == MJ_TO_IL")
    except Exception as e:  # noqa: BLE001
        print(f"  [SKIP] robot module import (needs isaaclab): {type(e).__name__}")

    # --- synthetic conversion: joint_order='mj' (no reorder), known root quat ---
    Tn = 4
    rng = np.random.default_rng(0)
    joint_pos_mj = rng.standard_normal((Tn, NUM_DOF)).astype(np.float32) * 0.2
    body_pos = np.zeros((Tn, NUM_BODIES, 3), dtype=np.float32)
    body_pos[:, 0, :] = rng.standard_normal((Tn, 3)).astype(np.float32)
    # a known non-identity root quat, expressed wxyz
    qx = transform.Rotation.from_euler("xyz", [10, 20, 30], degrees=True).as_quat().astype(np.float32)  # xyzw
    q_wxyz = qx[[3, 0, 1, 2]]
    body_quat = np.zeros((Tn, NUM_BODIES, 4), dtype=np.float32)
    body_quat[:, :, 0] = 1.0
    body_quat[:, 0, :] = q_wxyz

    entry = convert_sequence(
        {"joint_pos": joint_pos_mj, "body_pos_w": body_pos, "body_quat_w": body_quat, "joint_order": "mj"},
        fps=30,
    )
    check(entry["pose_aa"].shape == (Tn, NUM_BODIES, 3), f"pose_aa shape {entry['pose_aa'].shape}")
    check(entry["dof"].shape == (Tn, NUM_DOF), f"dof shape {entry['dof'].shape}")
    check(entry["root_trans_offset"].shape == (Tn, 3), "root_trans_offset shape")
    check(entry["root_rot"].shape == (Tn, 4), "root_rot shape")
    check(entry["smpl_joints"].shape == (Tn, 24, 3), "smpl_joints shape")
    check(np.allclose(entry["dof"], joint_pos_mj, atol=1e-6), "joint_order='mj' leaves dof unreordered")
    # pose_aa[:,1:] == DOF_AXIS * dof
    expect_aa = DOF_AXIS[None, :, :] * entry["dof"][:, :, None]
    check(np.allclose(entry["pose_aa"][:, 1:, :], expect_aa, atol=1e-6), "pose_aa[1:] == DOF_AXIS * dof")
    # root: root_rot is xyzw of the input; pose_aa[:,0] is its rotvec
    check(np.allclose(entry["root_rot"], qx[None, :], atol=1e-5), "root_rot == input xyzw (wxyz->xyzw)")
    expect_rv = transform.Rotation.from_quat(qx).as_rotvec().astype(np.float32)
    check(np.allclose(entry["pose_aa"][:, 0, :], expect_rv[None, :], atol=1e-5), "pose_aa[0] == rotvec(root xyzw)")

    # --- synthetic conversion: joint_order='il' reorders by MJ_TO_IL ---
    joint_pos_il = rng.standard_normal((Tn, NUM_DOF)).astype(np.float32)
    entry_il = convert_sequence(
        {"joint_pos": joint_pos_il, "body_pos_w": body_pos, "body_quat_w": body_quat, "joint_order": "il"},
        fps=30,
    )
    check(np.allclose(entry_il["dof"], joint_pos_il[:, MJ_TO_IL], atol=1e-6),
          "joint_order='il' reorders dof by MJ_TO_IL")

    # --- bones-CSV round trip (positional read), pandas required ---
    try:
        import tempfile

        import pandas as pd

        deg = np.rad2deg(joint_pos_mj[0]).tolist()
        header = ["Frame", "root_translateX", "root_translateY", "root_translateZ",
                  "root_rotateX", "root_rotateY", "root_rotateZ"] + BONES_CSV_JOINT_NAMES
        rows = []
        for t in range(Tn):
            rows.append([t, 12.0, -3.0, 94.0, 10.0, 20.0, 30.0] + np.rad2deg(joint_pos_mj[t]).tolist())
        with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="") as fh:
            pd.DataFrame(rows, columns=header).to_csv(fh.name, index=False)
            tmp = fh.name
        seq = load_bones_csv(tmp)
        os.unlink(tmp)
        check(seq["joint_order"] == "mj", "bones-CSV tagged joint_order='mj'")
        check(seq["joint_pos"].shape == (Tn, NUM_DOF), f"bones-CSV joint_pos {seq['joint_pos'].shape}")
        check(np.allclose(seq["joint_pos"], joint_pos_mj, atol=1e-4), "bones-CSV deg->rad round trip")
        check(np.allclose(seq["body_pos_w"][:, 0, :], np.array([0.12, -0.03, 0.94]), atol=1e-5),
              "bones-CSV root pos cm->m")
        bentry = convert_sequence(seq, fps=30)
        check(bentry["pose_aa"].shape == (Tn, NUM_BODIES, 3), "bones-CSV entry pose_aa shape")
    except ImportError:
        print("  [SKIP] bones-CSV sub-check (pandas not installed)")

    print("[selftest]", "ALL PASS" if ok else "FAILURES PRESENT")
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description="Convert SOMA CSV/PKL to motion_lib (TienKung 2dex, 31 DOF)")
    parser.add_argument("--input", help="CSV dir, parent dir of CSV dirs, or deploy PKL")
    parser.add_argument("--output", help="Output path (PKL file or directory for individual PKLs)")
    parser.add_argument("--fps", type=int, default=30, help="Target output FPS (default: 30)")
    parser.add_argument(
        "--fps_source",
        type=int,
        default=None,
        help="Source data FPS; if set and != --fps, data is downsampled (SOMA CSVs are often 120fps).",
    )
    parser.add_argument(
        "--individual", action="store_true", help="Write individual PKLs per motion (preserves session dirs)"
    )
    parser.add_argument("--num_workers", type=int, default=8, help="Parallel workers for --individual mode")
    parser.add_argument("--selftest", action="store_true", help="Run offline self-test and exit")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(run_selftest())

    if not args.input or not args.output:
        parser.error("--input and --output are required (unless --selftest)")

    print(f"TienKung 2dex {NUM_DOF} DOFs, {NUM_BODIES} bodies (hardcoded axes)")

    if args.individual:
        if not os.path.isdir(args.input):
            print("ERROR: --individual requires a directory input")
            sys.exit(1)

        subdirs = sorted(
            [d for d in os.listdir(args.input) if os.path.isdir(os.path.join(args.input, d))]
        )
        has_csvs = any(f.endswith(".csv") for f in os.listdir(args.input))
        has_session_subdirs = (
            any(
                any(f.endswith(".csv") for f in os.listdir(os.path.join(args.input, d)))
                for d in subdirs[:3]
            )
            if subdirs
            else False
        )

        session_dirs = []
        if has_session_subdirs:
            for d in subdirs:
                subdir = os.path.join(args.input, d)
                if any(f.endswith(".csv") for f in os.listdir(subdir)):
                    session_dirs.append((subdir, d, args.output, args.fps, args.fps_source))
        elif has_csvs:
            session_name = os.path.basename(args.input.rstrip("/"))
            session_dirs.append((args.input, session_name, args.output, args.fps, args.fps_source))

        print(f"\nBatch converting {len(session_dirs)} sessions with {args.num_workers} workers")
        print(f"Output: {args.output}")
        os.makedirs(args.output, exist_ok=True)

        import multiprocessing

        total_converted = total_failed = total_csvs = 0
        with multiprocessing.Pool(processes=args.num_workers) as pool:
            for session_name, converted, failed, n_csvs in pool.imap_unordered(
                process_session_csvs, session_dirs
            ):
                total_converted += converted
                total_failed += failed
                total_csvs += n_csvs
                print(
                    f"  {session_name}: {converted}/{n_csvs} converted"
                    + (f" ({failed} failed)" if failed else "")
                )

        print(f"\nDone: {total_converted} motions converted, {total_failed} failed, {total_csvs} total CSVs")
        return

    # Combined PKL output path
    sequences = {}

    if args.input.endswith(".pkl"):
        print(f"Loading deploy PKL: {args.input}")
        data = joblib.load(args.input)
        for name, seq in data.items():
            sequences[name] = seq
        print(f"  Found {len(sequences)} sequences")

    elif os.path.isfile(os.path.join(args.input, "joint_pos.csv")):
        name = os.path.basename(args.input)
        print(f"Loading single CSV motion: {name}")
        seq = load_csv_motion(args.input)
        if seq is None:
            print("ERROR: joint_pos.csv not found")
            sys.exit(1)
        sequences[name] = seq
        print(f"  {seq['joint_pos'].shape[0]} frames")

    elif os.path.isdir(args.input):
        csv_files = sorted([f for f in os.listdir(args.input) if f.endswith(".csv")])
        subdirs = sorted(
            [d for d in os.listdir(args.input) if os.path.isdir(os.path.join(args.input, d))]
        )

        if csv_files and not any(
            os.path.exists(os.path.join(args.input, d, "joint_pos.csv")) for d in subdirs[:5]
        ):
            print(f"Scanning directory for flat SOMA CSVs: {args.input}")
            for csv_f in csv_files:
                name = os.path.splitext(csv_f)[0]
                try:
                    sequences[name] = load_bones_csv(os.path.join(args.input, csv_f))
                except Exception as e:  # noqa: BLE001
                    print(f"  WARNING: Failed to load {csv_f}: {e}")
            print(f"  Found {len(sequences)} flat-CSV motions")
        elif subdirs:
            has_session_csvs = False
            for dname in subdirs[:3]:
                subdir = os.path.join(args.input, dname)
                sub_csvs = [f for f in os.listdir(subdir) if f.endswith(".csv")]
                if sub_csvs and not os.path.exists(os.path.join(subdir, "joint_pos.csv")):
                    has_session_csvs = True
                    break

            if has_session_csvs:
                print(f"Scanning session dirs for flat SOMA CSVs: {args.input}")
                for dname in sorted(subdirs):
                    subdir = os.path.join(args.input, dname)
                    sub_csvs = sorted([f for f in os.listdir(subdir) if f.endswith(".csv")])
                    for csv_f in sub_csvs:
                        name = os.path.splitext(csv_f)[0]
                        try:
                            sequences[name] = load_bones_csv(os.path.join(subdir, csv_f))
                        except Exception as e:  # noqa: BLE001
                            print(f"  WARNING: Failed to load {dname}/{csv_f}: {e}")
                    if sub_csvs:
                        print(f"  Session {dname}: {len(sub_csvs)} CSVs")
                print(f"  Found {len(sequences)} total flat-CSV motions")
            else:
                print(f"Scanning directory: {args.input}")
                for dname in sorted(subdirs):
                    seq = load_csv_motion(os.path.join(args.input, dname))
                    if seq is not None:
                        sequences[dname] = seq
                print(f"  Found {len(sequences)} motion directories with CSVs")
    else:
        print(f"ERROR: {args.input} is not a valid input")
        sys.exit(1)

    if not sequences:
        print("ERROR: No sequences found")
        sys.exit(1)

    motion_lib_dict = {}
    for name, seq_data in sequences.items():
        T = seq_data["joint_pos"].shape[0]
        print(f"  Converting {name}: {T} frames @ {args.fps} fps")
        fps_for_convert = args.fps_source if args.fps_source else args.fps
        entry = convert_sequence(seq_data, fps_for_convert)
        if args.fps_source and args.fps_source != args.fps:
            entry = downsample_sequence(entry, args.fps_source, args.fps)
        motion_lib_dict[name] = entry

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    print(f"\nSaving motion_lib PKL: {args.output}")
    joblib.dump(motion_lib_dict, args.output, compress=True)
    print(f"Done: {len(motion_lib_dict)} sequences saved")


if __name__ == "__main__":
    main()
