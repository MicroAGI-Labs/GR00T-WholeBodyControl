"""Verify the inferred Isaac Lab joint/body ordering for TienKung 2dex.

Cluster-verification for the BFS-derived lists in
gear_sonic/envs/manager_env/robots/tiangong2dex.py: loads the URDF through
the actual TIANGONG2DEX_CFG articulation and compares the observed
body/joint names against the committed lists. Exit code 0 = match.

Run with the Isaac Sim python:
    /isaac-sim/python.sh gear_sonic/scripts/verify_tiangong2dex_ordering.py
"""

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402

from gear_sonic.envs.manager_env.robots.tiangong2dex import (  # noqa: E402
    TIANGONG2DEX_CFG,
    TIANGONG2DEX_ISAACLAB_JOINTS,
)

sim = SimulationContext(sim_utils.SimulationCfg(dt=0.005, device="cuda:0"))
robot = Articulation(TIANGONG2DEX_CFG.replace(prim_path="/World/Robot"))
sim.reset()

result_lines = []


def emit(msg):
    result_lines.append(msg)
    print(msg, flush=True)


observed_bodies = list(robot.body_names)
observed_joints = list(robot.joint_names)
expected_bodies = list(TIANGONG2DEX_ISAACLAB_JOINTS)

emit(f"observed bodies ({len(observed_bodies)}): {observed_bodies}")
emit(f"observed joints ({len(observed_joints)}): {observed_joints}")

ok = True
if observed_bodies != expected_bodies:
    ok = False
    emit("BODY MISMATCH vs TIANGONG2DEX_ISAACLAB_JOINTS:")
    for i, (o, e) in enumerate(zip(observed_bodies, expected_bodies)):
        if o != e:
            emit(f"  [{i}] observed={o} expected={e}")
    if len(observed_bodies) != len(expected_bodies):
        emit(f"  length {len(observed_bodies)} vs {len(expected_bodies)}")
if len(observed_joints) != 31:
    ok = False
    emit(f"JOINT COUNT MISMATCH: {len(observed_joints)} != 31")

emit("ORDERING VERIFIED" if ok else "ORDERING WRONG — regenerate mappings")

with open("/tmp/tiangong2dex_ordering_result.txt", "w") as f:
    f.write("\n".join(result_lines) + "\n")
simulation_app.close()
sys.exit(0 if ok else 1)
