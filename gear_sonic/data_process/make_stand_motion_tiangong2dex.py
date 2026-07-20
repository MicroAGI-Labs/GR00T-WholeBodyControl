"""Generate a synthetic standing motion PKL for TienKung 2dex.

Smoke-test vehicle for the tiangong2dex embodiment (guide step 5/7): a
static default-stance clip in the motion-library PKL format, so the
training env can construct before real retargeted data exists. Joint and
body ordering is read from the vendored MJCF (document order == MuJoCo
order), the same file the motion library loads for FK.

    /isaac-sim/python.sh gear_sonic/data_process/make_stand_motion_tiangong2dex.py \
        --output ~/tiangong2dex-data/stand_smoke --seconds 3
"""

import argparse
import os
import xml.etree.ElementTree as ET

import joblib
import numpy as np

MJCF = os.path.join(
    os.path.dirname(__file__),
    "..",
    "data/assets/robot_description/mjcf/tiangong2dex.xml",
)

# Default stance, keyed by joint name (matches TIANGONG2DEX_CFG init_state).
STANCE = {
    "hip_pitch_l_joint": -0.15,
    "hip_pitch_r_joint": -0.15,
    "knee_pitch_l_joint": 0.3,
    "knee_pitch_r_joint": 0.3,
    "ankle_pitch_l_joint": -0.15,
    "ankle_pitch_r_joint": -0.15,
    "shoulder_pitch_l_joint": 0.2,
    "shoulder_pitch_r_joint": 0.2,
    "shoulder_roll_l_joint": 0.1,
    "shoulder_roll_r_joint": -0.1,
    "elbow_pitch_l_joint": -0.5,
    "elbow_pitch_r_joint": -0.5,
}
ROOT_Z = 0.97

parser = argparse.ArgumentParser()
parser.add_argument("--output", required=True, help="output dir for the PKL")
parser.add_argument("--seconds", type=float, default=3.0)
parser.add_argument("--fps", type=int, default=30)
args = parser.parse_args()

worldbody = ET.parse(MJCF).getroot().find("worldbody")
joints = [
    j.get("name") for j in worldbody.iter("joint") if j.get("name") is not None
]
bodies = [b.get("name") for b in worldbody.iter("body")]
assert len(joints) == 31, f"expected 31 joints, got {len(joints)}"
assert len(bodies) == 32, f"expected 32 bodies, got {len(bodies)}"
unknown = set(STANCE) - set(joints)
assert not unknown, f"stance joints not in MJCF: {unknown}"

T = int(args.seconds * args.fps)
dof_frame = np.array([STANCE.get(j, 0.0) for j in joints], dtype=np.float32)

motion = {
    "root_trans_offset": np.tile([0.0, 0.0, ROOT_Z], (T, 1)).astype(np.float32),
    "pose_aa": np.zeros((T, len(bodies), 3), dtype=np.float32),
    "dof": np.tile(dof_frame, (T, 1)),
    "root_rot": np.tile([1.0, 0.0, 0.0, 0.0], (T, 1)).astype(np.float32),  # wxyz identity
    "smpl_joints": np.zeros((T, 24, 3), dtype=np.float32),
    "fps": args.fps,
}

os.makedirs(args.output, exist_ok=True)
out = os.path.join(args.output, "stand_default.pkl")
joblib.dump({"stand_default": motion}, out)
print(f"wrote {out}: {T} frames, dof {motion['dof'].shape}, "
      f"pose_aa {motion['pose_aa'].shape}", flush=True)
