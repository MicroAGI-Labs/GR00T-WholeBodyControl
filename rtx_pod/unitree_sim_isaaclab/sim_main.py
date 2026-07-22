
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
#!/usr/bin/env python3
# main.py
import os

project_root = os.path.dirname(os.path.abspath(__file__))
os.environ["PROJECT_ROOT"] = project_root

import argparse
import contextlib
import time
import sys
import signal
import torch
import gymnasium as gym
from pathlib import Path

# Isaac Lab AppLauncher
from isaaclab.app import AppLauncher

from teleimager.image_server import run_isaacsim_server
from dds.dds_create import create_dds_objects,create_dds_objects_replay
from dds.dds_master import dds_manager
# add command line arguments
parser = argparse.ArgumentParser(description="Unitree Simulation")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G129-Head-Waist-Fix", help="task name")
parser.add_argument("--action_source", type=str, default="dds", 
                   choices=["dds", "file", "trajectory", "policy", "replay","dds_wholebody","dds_lowcmd29"],
                   help="Action source")


parser.add_argument("--robot_type", type=str, default="g129", help="robot type")
parser.add_argument("--enable_dex1_dds", action="store_true", help="enable gripper DDS")
parser.add_argument("--enable_dex3_dds", action="store_true", help="enable dexterous hand DDS")
parser.add_argument("--enable_inspire_dds", action="store_true", help="enable inspire hand DDS")
parser.add_argument("--stats_interval", type=float, default=10.0, help="statistics print interval (seconds)")

parser.add_argument("--file_path", type=str, default="/home/unitree/Code/xr_teleoperate/teleop/utils/data", help="file path (when action_source=file)")
parser.add_argument("--generate_data_dir", type=str, default="./data", help="save data dir")
parser.add_argument("--generate_data", action="store_true", default=False, help="generate data")
parser.add_argument("--rerun_log", action="store_true", default=False, help="rerun log")
parser.add_argument("--replay_data",  action="store_true", default=False, help="replay data")

parser.add_argument("--modify_light",  action="store_true", default=False, help="modify light")
parser.add_argument("--modify_camera",  action="store_true", default=False,    help="modify camera")

# performance analysis parameters
parser.add_argument("--step_hz", type=int, default=100, help="control frequency")
parser.add_argument("--enable_profiling", action="store_true", default=True, help="enable performance analysis")
parser.add_argument("--profile_interval", type=int, default=500, help="performance analysis report interval (steps)")

parser.add_argument("--model_path", type=str, default="assets/model/policy.onnx", help="model path")
parser.add_argument("--reward_interval", type=int, default=10, help="step interval for reward calculation")
parser.add_argument("--enable_wholebody_dds", action="store_true", default=False, help="enable wh dds")

parser.add_argument("--physics_dt", type=float, default=None, help="physics time step, e.g., 0.005")
parser.add_argument("--render_interval", type=int, default=None, help="render interval steps (>=1)")
parser.add_argument("--camera_write_interval", type=int, default=None, help="camera write interval steps (>=1)")


parser.add_argument("--no_render",action="store_true",default=False,help="disable rendering updates entirely (overrides render interval)",)
parser.add_argument("--no_livestream", action="store_true", default=False, help="disable the Omniverse WebRTC viewport livestream (livestream is ON by default)")
parser.add_argument("--public_ip",type=str,default="auto",help="public IP advertised as the WebRTC ICE candidate; 'auto' = this host's primary IP (correct for a WARP-reachable pod)")
parser.add_argument("--livestream_type", type=int, default=2, help="livestream type when enabled (1: WebRTC public network, 2: WebRTC private network)")

parser.add_argument("--solver_iterations", type=int, default=None, help="physx solver iteration count (e.g., 4)")
parser.add_argument("--gravity_z", type=float, default=None, help="override gravity z (e.g., -9.8)")
parser.add_argument("--skip_cvtcolor", action="store_true", default=False, help="skip cv2.cvtColor if upstream already BGR")

parser.add_argument("--camera_jpeg", action="store_true", default=True, help="enable JPEG compression for camera frames")
parser.add_argument("--camera_jpeg_quality", type=int, default=85, help="JPEG quality (1-100)")

parser.add_argument("--physx_substeps", type=int, default=None, help="physx substeps per step")
parser.add_argument("--camera_include", type=str, default="front_camera,left_wrist_camera,right_wrist_camera", help="comma-separated camera names to enable")
parser.add_argument("--camera_exclude", type=str, default="world_camera", help="comma-separated camera names to disable")

parser.add_argument("--env_reward_interval", type=int, default=5, help="environment reward compute interval (steps)")
parser.add_argument("--seed", type=int, default=42, help="environment seed")
parser.add_argument(
    "--num_envs",
    type=int,
    default=int(os.environ.get("SIM_ROBOT_COUNT", "1")),
    help="number of vectorized G1 robots (default: SIM_ROBOT_COUNT or 1)",
)
# add AppLauncher parameters
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.num_envs < 1 or args_cli.num_envs > 24:
    parser.error("--num_envs must be between 1 and 24")
# Omniverse WebRTC viewport livestream is ON by default; pass --no_livestream to disable.
# (Kept independent of --no_render: that flag freezes the viewport via render_interval->1e6,
# so the two must be decoupled to stream the live 3rd-person view while rendering normally.)
if args_cli.no_livestream or int(args_cli.livestream_type) <= 0:
    os.environ["LIVESTREAM"] = "0"
    print("[sim] WebRTC livestream DISABLED.")
else:
    def _primary_ip():
        # The address a remote (WARP-reachable) client uses to reach this host — the
        # pod's own IP, not 127.0.0.1 — so the advertised ICE candidate is reachable.
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"
    public_ip = _primary_ip() if args_cli.public_ip in ("", "auto") else args_cli.public_ip
    os.environ["LIVESTREAM"] = str(args_cli.livestream_type)
    os.environ["PUBLIC_IP"] = public_ip
    print(f"[sim] WebRTC livestream ON (type {args_cli.livestream_type}); advertising PUBLIC_IP={public_ip}. Pass --no_livestream to disable.")

if args_cli.enable_dex3_dds and args_cli.enable_dex1_dds and args_cli.enable_inspire_dds:
    print("Error: enable_dex3_dds and enable_dex1_dds and enable_inspire_dds cannot be enabled at the same time")
    print("Please select one of the options")
    sys.exit(1)


import pinocchio                 
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from layeredcontrol.robot_control_system import (
    RobotController, 
    ControlConfig,
)

from dds.reset_pose_dds import *
import tasks
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

from tools.augmentation_utils import (
    update_light,
    batch_augment_cameras_by_name,
)

from tools.data_json_load import sim_state_to_json
from dds.sim_state_dds import *
from action_provider.create_action_provider import create_action_provider
from tools.get_stiffness import get_robot_stiffness_from_env
from tools.get_reward import get_step_reward_value,get_current_rewards

def setup_signal_handlers(controller,dds_manager=None,image_server=None):
    """set signal handlers"""
    def signal_handler(signum, frame):
        print(f"\nreceived signal {signum}, stopping controller...")
        try:
            controller.stop()
        except Exception as e:
            print(f"Failed to stop controller: {e}")
        try:
            if dds_manager is not None:
                dds_manager.stop_all_communication()
        except Exception as e:
            print(f"Failed to stop DDS: {e}")
        try:
            if image_server is not None:
                image_server.stop()
        except Exception as e:
            print(f"Failed to stop image server: {e}")
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)



def main():
    """main function"""
    # import cProfile
    # import pstats
    # import io
    # profiler = cProfile.Profile()
    # profiler.enable()
    import os
    import atexit
    try:
        os.setpgrp()
        current_pgid = os.getpgrp()
        print(f"Setting process group: {current_pgid}")
        
        def cleanup_process_group():
            try:
                print(f"Cleaning up process group: {current_pgid}")
                import signal
                os.killpg(current_pgid, signal.SIGTERM)
            except Exception as e:
                print(f"Failed to clean up process group: {e}")
        
        atexit.register(cleanup_process_group)
        
    except Exception as e:
        print(f"Failed to set process group: {e}")
    print("=" * 60)
    print("robot control system started")
    print(f"Task: {args_cli.task}")
    print(f"Action source: {args_cli.action_source}")
    print("=" * 60)

    # parse environment configuration
    try:
        env_cfg = parse_env_cfg(
            args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs
        )
        env_cfg.env_name = args_cli.task
    except Exception as e:
        print(f"Failed to parse environment configuration: {e}")
        return
    
    # create environment
    print("\ncreate environment...")
    try:
        env_cfg.seed = args_cli.seed
        env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
        env.seed(args_cli.seed)
        try:
            sensors_dict = getattr(env.scene, "sensors", {})
            if sensors_dict:
                print("Sensors in the environment:")
                for name, sensor in sensors_dict.items():
                    print(name, sensor)
                print("="*60)
        except Exception as e:
            print(f"[sim] failed to list sensors: {e}")
        print(f"\ncreate environment success ...")
        try:
            env._reward_interval = max(1, int(args_cli.env_reward_interval))
            env._reward_counter = 0
            env._reward_last = None
            print(f"[env] reward compute interval set to {env._reward_interval} steps")
        except Exception as e:
            print(f"[env] failed to set reward interval: {e}")
        if args_cli.physics_dt is not None:
            try:
                env.sim.set_substep_time(args_cli.physics_dt)
                print(f"[sim] physics dt set to {args_cli.physics_dt}")
            except Exception:
                try:
                    env.sim.dt = args_cli.physics_dt
                    print(f"[sim] physics dt assigned to env.sim.dt={args_cli.physics_dt}")
                except Exception as e:
                    print(f"[sim] failed to set physics dt: {e}")
        headless_mode = bool(getattr(args_cli, "headless", False))
        render_interval = None
        if args_cli.render_interval is not None:
            try:
                render_interval = max(1, int(args_cli.render_interval))
            except Exception as e:
                print(f"[sim] invalid render_interval value {args_cli.render_interval}: {e}")
        try:
            if args_cli.no_render:
                env.sim.render_interval = 1_000_000
                env.sim.render_mode = "offscreen"
                print("[sim] rendering disabled via --no_render")
            elif headless_mode:
                env.sim.render_mode = "offscreen"
                env.sim.render_interval = render_interval or 1
                print(f"[sim] headless offscreen rendering every {env.sim.render_interval} steps")
            elif render_interval is not None:
                env.sim.render_interval = render_interval
                print(f"[sim] render_interval set to {env.sim.render_interval}")
        except Exception as e:
            print(f"[sim] failed to configure rendering: {e}")
        if args_cli.camera_write_interval is not None:
            try:
                import tasks.common_observations.camera_state as cam_state
                cam_state._camera_cache['write_interval_steps'] = max(1, int(args_cli.camera_write_interval))
                print(f"[camera] write interval steps set to {cam_state._camera_cache['write_interval_steps']}")
            except Exception as e:
                print(f"[camera] failed to set write interval: {e}")

        try:
            if args_cli.solver_iterations is not None:
                env.sim.physx.solver_iteration_count = int(args_cli.solver_iterations)
                print(f"[sim] solver_iteration_count={env.sim.physx.solver_iteration_count}")
            if args_cli.physx_substeps is not None:
                try:
                    env.sim.physx.substeps = int(args_cli.physx_substeps)
                except Exception:
                    try:
                        env.sim.set_substeps(int(args_cli.physx_substeps))
                    except Exception:
                        pass
                print(f"[sim] physx_substeps set to {args_cli.physx_substeps}")
            if args_cli.gravity_z is not None:
                g = float(args_cli.gravity_z)
                env.sim.physx.gravity = (0.0, 0.0, g)
                print(f"[sim] gravity set to {env.sim.physx.gravity}")
        except Exception as e:
            print(f"[sim] failed to set physx params: {e}")
        if args_cli.skip_cvtcolor:
            os.environ["CAMERA_SKIP_CVTCOLOR"] = "1"
        try:
            import tasks.common_observations.camera_state as cam_state
            enable_jpeg = bool(args_cli.camera_jpeg) or (os.getenv("CAMERA_JPEG") == "1")
            jpeg_quality = int(args_cli.camera_jpeg_quality if args_cli.camera_jpeg else os.getenv("CAMERA_JPEG_QUALITY", args_cli.camera_jpeg_quality))
            cam_state.set_writer_options(enable_jpeg=enable_jpeg, jpeg_quality=jpeg_quality, skip_cvtcolor=args_cli.skip_cvtcolor)
            include = [n.strip() for n in (args_cli.camera_include or "").split(',') if n.strip()]
            exclude = [n.strip() for n in (args_cli.camera_exclude or "").split(',') if n.strip()]
            try:
                cam_state.set_camera_allowlist(include)
            except Exception:
                pass
            try:
                sensors_dict = getattr(env.scene, "sensors", {})
                for name, sensor in sensors_dict.items():
                    lname = name.lower()
                    if "camera" not in lname:
                        continue
                    if exclude and name in exclude:
                        for attr_name, value in [("enabled", False), ("is_enabled", False)]:
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, value)
                                except Exception:
                                    pass
                        for meth in ("set_active", "disable", "pause"):
                            if hasattr(sensor, meth):
                                try:
                                    getattr(sensor, meth)(False)
                                except Exception:
                                    pass
                        for attr_name in ("update_period", "_update_period"):
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, 1e6)
                                except Exception:
                                    pass
                    elif include and name not in include:
                        for attr_name in ("update_period", "_update_period"):
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, 1e6)
                                except Exception:
                                    pass
            except Exception as e:
                print(f"[camera] failed to tune sensors: {e}")
        except Exception as e:
            print(f"[camera] failed to apply writer options: {e}")
    except Exception as e:
        print(f"\nFailed to create environment: {e}")
        return
    
    # get robot stiffness and damping parameters from runtime environment
    print("\n" + "="*60)
    print("🔍 Getting robot stiffness and damping parameters from runtime environment")
    print("="*60)
    
    try:
        stiffness_data = get_robot_stiffness_from_env(env)
        if stiffness_data:
            print("✅ Successfully got robot parameters!")
        else:
            print("⚠️ Failed to get robot parameters, will try again after environment reset")
    except Exception as e:
        print(f"⚠️ Error getting robot parameters: {e}")
    
    print("="*60)
    
    if not getattr(args_cli, "headless", False) and not args_cli.no_render:
        print("\n")
        print("***  Please left-click on the Sim window to activate rendering. ***")
        print("\n")
    else:
        print("\n")
        print("***  Running without GUI; rendering handled offscreen. ***")
        print("\n")
    # reset environment
    if args_cli.modify_light:
        update_light(
            prim_path="/World/light",
            color=(0.75, 0.75, 0.75),
            intensity=500.0,
            # position=(1.0, 2.0, 3.0),
            radius=0.1,
            enabled=True,
            cast_shadows=True
        )
    if args_cli.modify_camera:
        batch_augment_cameras_by_name(
            names=["front_cam"],
            focal_length=3.0,
            horizontal_aperture=22.0,
            vertical_aperture=16.0,
            exposure=0.8,                
            focus_distance=1.2
        )
    env.sim.reset()
    env.reset()

    def default_root_state_world(robot, env_ids=None):
        """Return Isaac Lab's environment-local default root state in world space."""
        if env_ids is None:
            root = robot.data.default_root_state.clone()
            origins = env.scene.env_origins
        else:
            root = robot.data.default_root_state[env_ids].clone()
            origins = env.scene.env_origins[env_ids]
        root[:, :3] += origins
        return root

    # The default viewer pose is intended for one environment.  Center an
    # elevated overview on the cloned environment grid so a WebRTC client sees
    # every robot without needing to navigate the viewport manually.
    if env.num_envs > 1 and not args_cli.no_render:
        try:
            origins = env.scene.env_origins.detach().cpu()
            grid_min = origins.min(dim=0).values
            grid_max = origins.max(dim=0).values
            center = (grid_min + grid_max) * 0.5
            span = max(
                float(grid_max[0] - grid_min[0]),
                float(grid_max[1] - grid_min[1]),
                2.5,
            )
            distance = max(7.5, 1.75 * span)
            eye = (
                float(center[0]) + distance,
                float(center[1]) + distance,
                max(6.0, 1.3 * span),
            )
            target = (float(center[0]), float(center[1]), 0.75)
            env.sim.set_camera_view(eye=eye, target=target)
            print(
                f"[sim] multi-robot viewport: eye={eye}, target={target}, "
                f"grid_min={grid_min.tolist()}, grid_max={grid_max.tolist()}",
                flush=True,
            )
        except Exception as e:
            print(f"[sim] failed to configure multi-robot viewport: {e}", flush=True)
    
    # create simplified control configuration
    try:    
        control_config = ControlConfig(
            step_hz=args_cli.step_hz,
            replay_mode=args_cli.replay_data
        )
    except Exception as e:
        print(f"Failed to create control configuration: {e}")
        return
    
    # create controller

    if not args_cli.replay_data:
        print("========= create image server =========")
        try:
            image_server = run_isaacsim_server()
        except Exception as e:
            print(f"Failed to create image server: {e}")
            return
        print("========= create image server success =========")
        print("========= create dds =========")
        try:
            multi_robot_dds = None
            if args_cli.num_envs > 1:
                from dds.g1_multi_robot_dds import G1MultiRobotDDS
                multi_robot_dds = G1MultiRobotDDS(
                    args_cli.num_envs,
                    prefix_base=os.environ.get(
                        "SIM_TOPIC_PREFIX_BASE", "rt/sim/g1"
                    ),
                )
                env._multi_robot_dds = multi_robot_dds
                multi_robot_dds.start()
                reset_pose_dds = None
                sim_state_dds = None
                dds_lifecycle = multi_robot_dds
            else:
                reset_pose_dds,sim_state_dds,dds_manager = create_dds_objects(args_cli,env)
                dds_lifecycle = dds_manager
        except Exception as e:
            print(f"Failed to create dds: {e}")
            return
        print("========= create dds success =========")
    else:
        print("========= create dds =========")
        try:
            create_dds_objects_replay(args_cli,env)
        except Exception as e:
            print(f"Failed to create dds: {e}")
            return
        print("========= create dds success =========")
        from tools.data_json_load import get_data_json_list
        print("========= get data json list =========")
        data_idx=0
        data_json_list = get_data_json_list(args_cli.file_path)
        if args_cli.action_source != "replay":
            args_cli.action_source = "replay"
        print("========= get data json list success =========")
    # create action provider
    
    print(f"\ncreate action provider: {args_cli.action_source}...")
    try:
        print(f"args_cli.task: {args_cli.task}")
        if not args_cli.replay_data and ("Wholebody" in args_cli.task or args_cli.enable_wholebody_dds):
            args_cli.action_source = "dds_wholebody"
            args_cli.enable_wholebody_dds = True
            control_config.use_rl_action_mode = True
        action_provider = create_action_provider(env,args_cli)
        if action_provider is None:
            print("action provider creation failed, exiting")
            return
    except Exception as e:
        print(f"Failed to create action provider: {e}")
        return
    
    # set action provider
    print("========= create controller =========")
    controller = RobotController(env, control_config)
    controller.set_action_provider(action_provider)
    print("========= create controller success =========")
    
    # configure performance analysis
    if args_cli.enable_profiling:
        controller.set_profiling(True, args_cli.profile_interval)
        print(f"performance analysis enabled, report every {args_cli.profile_interval} steps")
    else:
        controller.set_profiling(False)
        print("performance analysis disabled")


    # set signal handlers
    if not args_cli.replay_data:
        setup_signal_handlers(controller,dds_lifecycle,image_server)
    else:
        setup_signal_handlers(controller)
        
    print("Note: The DDS in Sim transmits messages on channel 1. Please ensure that other DDS instances use the same channel for message exchange by setting: ChannelFactoryInitialize(1).")
    try:
        # start controller - start asynchronous components
        print("========= start controller =========")
        controller.start()
        print("========= start controller success =========")
        
        # main loop - execute in main thread to support rendering
        last_stats_time = time.time()
        loop_start_time = time.time()
        _rtf_sim_t0 = float(env.sim.current_time)  # RTF instrumentation
        loop_count = 0
        last_loop_time = time.time()
        recent_loop_times = []  # for calculating moving average frequency
        
        
        reward_interval = max(1, args_cli.reward_interval)

        # --- real-time throttle -------------------------------------------------
        # The external SONIC deploy free-runs at 50 Hz wall-clock and its balance
        # policy is only stable when sim-time advances at wall-clock rate (RTF~=1).
        # Without this the loop free-runs: CPU -> slower than real-time (RTF<1),
        # GPU -> faster than real-time (RTF>1). Both under/over-sample the control
        # loop in sim-time and topple the robot. Pin each loop to one control
        # period (decimation * physics_dt). Sleep only when we're ahead; if a loop
        # overruns it just runs slow (no-op), so this is safe on CPU too.
        # Disable with SIM_REALTIME=0.
        _rt_on = os.environ.get("SIM_REALTIME", "1") != "0"
        try:
            _rt_period = float(env.cfg.decimation) * float(env.sim.get_physics_dt())
        except Exception:
            _rt_period = 0.02
        if not _rt_on:
            _rt_period = 0.0
        # SLOW-MOTION: stretch the wall-clock target period by SIM_SLOWMO (>1 = slower
        # than real-time). Physics still advances one control period (decim*dt) of
        # SIM-TIME per loop, but the loop now spans SIM_SLOWMO*period of WALL time, so
        # RTF = 1/SIM_SLOWMO. Point: the FIXED wall-clock round-trip latency (DDS
        # transport + deploy compute + lowstate age) becomes SIM_SLOWMO x smaller as a
        # fraction of sim-time, shrinking the effective control latency the balance loop
        # sees -- WITHOUT changing control frequency (keep the deploy rate-matched:
        # CONTROL_WALL_SCALE = RTF = 1/SIM_SLOWMO, so it still runs 50 Hz in sim-time and
        # the policy's obs-history timebase is unchanged). Opposite of adding latency.
        _rt_base = _rt_period                       # one control period (pre-slowmo)
        _slowmo = float(os.environ.get("SIM_SLOWMO", "1"))
        # DYNAMIC slow-mo: the loop re-reads SIM_SLOWMO_FILE each iteration, so the sim
        # speed can be changed live (echo a number into the file) with NO Isaac restart.
        _slowmo_file = os.environ.get("SIM_SLOWMO_FILE", "/tmp/sim_slowmo")
        _slowmo_next_check = 0.0
        if _rt_base > 0.0 and _slowmo > 0.0:
            _rt_period = _rt_base * _slowmo
        print(f"[sim] real-time throttle: {'ON' if _rt_on else 'OFF'} target period={_rt_period*1000:.2f} ms "
              f"(SIM_SLOWMO={_slowmo}, live via {_slowmo_file})", flush=True)

        # --- virtual elastic band (startup base-hold) ---------------------------
        # The external SONIC controller uses a 4-step observation HISTORY. A mid-run
        # reset snaps the robot upright but leaves fallen-state data in that history,
        # so the policy sacks the freshly-reset robot. MuJoCo avoided this with an
        # elastic band that held the base steady while the controller warmed up, then
        # released. Replicate it: for SIM_BASE_HOLD_S seconds after each reset-all, pin
        # the base to its init pose (zero velocity) every step while the deploy's PD
        # holds the joints and its history fills with clean standing data; then release.
        _base_hold_s = float(os.environ.get("SIM_BASE_HOLD_S", "0"))
        # Hold from STARTUP until an explicit release (a reset-all command), not just after
        # a reset. The deploy must never control a fallen robot (its OOD commands explode
        # PhysX); by pinning the base upright + holding default joints from t=0, the deploy
        # always sees a clean standing robot while it inits and its obs history fills. A
        # reset-all (rt/reset_pose/cmd cat 2) then RELEASES the hold so the controller balances.
        _HOLD_FOREVER = 1.0e18
        _base_hold_until = _HOLD_FOREVER if _base_hold_s > 0.0 else 0.0
        _multi_mode = args_cli.num_envs > 1
        if _multi_mode:
            env._base_hold_mask = torch.full(
                (env.num_envs,), _base_hold_s > 0.0,
                dtype=torch.bool, device=env.device,
            )
            env._warmup_joint_mask = torch.full(
                (env.num_envs,),
                _base_hold_s > 0.0
                and os.environ.get("SIM_WARMUP_JOINTS", "0") == "1",
                dtype=torch.bool, device=env.device,
            )
        # By default only the BASE is pinned and the controller drives the legs while held.
        # SIM_WARMUP_JOINTS=1 is the explicit static-pose experiment: joint positions and
        # velocities are pinned to the environment default as well.  Merely sending that
        # pose through the implicit PD actuator is not a freeze (gravity deflects it), and
        # would make a measured-pose IDLE reference differ from the requested hold pose.
        if _base_hold_s > 0.0 and os.environ.get("SIM_WARMUP_JOINTS", "0") == "1":
            try:
                env._warmup_joint_until = _HOLD_FOREVER
            except Exception:
                pass
        _bh_was_active = False
        _bh_kp_pos = float(os.environ.get("SIM_BH_KP_POS", "10000"))
        _bh_kd_pos = float(os.environ.get("SIM_BH_KD_POS", "1000"))
        _bh_kp_ang = float(os.environ.get("SIM_BH_KP_ANG", "1000"))
        _bh_kd_ang = float(os.environ.get("SIM_BH_KD_ANG", "50"))
        _bh_f_max = float(os.environ.get("SIM_BH_F_MAX", "600"))   # N, clamp to avoid PhysX blowup
        _bh_tq_max = float(os.environ.get("SIM_BH_TQ_MAX", "300"))  # N·m
        print(f"[sim] virtual base-hold: {_base_hold_s:.1f} s after reset (SIM_BASE_HOLD_S), "
              f"kp_pos={_bh_kp_pos} kd_pos={_bh_kd_pos} kp_ang={_bh_kp_ang} kd_ang={_bh_kd_ang}", flush=True)

        # --- SOFT base-hold (external wrench) vs RIGID pose-pin -------------------
        # RIGID pin (write_root_pose_to_sim) force-sets the base upright every step, so
        # the controller's balance actions have NO effect on the base during warmup ->
        # it receives out-of-distribution proprioception (fake-zero pelvis vel) and drifts
        # to a folded crouch, then topples on release. SOFT mode instead applies a clamped
        # PD wrench on the pelvis (like MuJoCo's compliant elastic band): the base can move
        # and RESPOND to the controller, so its obs stays in-distribution and it settles
        # onto its feet at a real Isaac equilibrium before release. Toggle: SIM_BASE_SOFT=1.
        _bh_soft = os.environ.get("SIM_BASE_SOFT", "0") == "1"
        if _multi_mode and _bh_soft:
            raise ValueError("multi-robot MVP supports only SIM_BASE_SOFT=0")
        _bh_soft_applied = False
        try:
            _bh_base_bid = env.scene["robot"].data.body_names.index("pelvis")
        except Exception:
            _bh_base_bid = 0
        if _bh_soft:
            import logging as _logging
            for _ln in ("isaaclab", "isaaclab.assets", "isaaclab.assets.articulation.articulation"):
                _logging.getLogger(_ln).setLevel(_logging.ERROR)
            print(f"[sim] BASE HOLD MODE = SOFT external-wrench on body {_bh_base_bid} "
                  f"(f_max={_bh_f_max}N tq_max={_bh_tq_max}Nm)", flush=True)
        else:
            print("[sim] BASE HOLD MODE = RIGID pose-pin", flush=True)

        # --- deterministic in-sim balance eval (publishes rt/eval) --------------
        # Scores the flat-stand: termination 0..1 (1 == a full SUCCESS_WINDOW stand
        # of sim-time, or a fall) and result -100..100 (100 = perfect upright at the
        # start spot, -100 = fall). Armed on hold-release (controller takes over),
        # disarmed on a re-arm-hold. Never raises into the loop. Disable EVAL=0.
        stand_eval = None
        if os.environ.get("EVAL", "1") != "0":
            try:
                from tools.stand_eval import StandEval
                if _multi_mode:
                    stand_eval = [
                        StandEval(
                            env.scene["robot"],
                            lambda: float(env.sim.current_time),
                            env_id=robot_id,
                            topic=multi_robot_dds.topic(robot_id, "eval"),
                        )
                        for robot_id in range(env.num_envs)
                    ]
                else:
                    stand_eval = StandEval(
                        env.scene["robot"], lambda: float(env.sim.current_time)
                    )
            except Exception as _e:
                print(f"[sim] stand_eval init failed (continuing without eval): {_e}", flush=True)

        # --- targeted external-wrench disturbance (balance robustness testing) ---
        # perturb_sim.py atomically writes a uniquely identified JSON command.  A
        # command can select one, several, or all vectorized environments.  Forces
        # and torques are world-frame pelvis wrenches and use simulated duration.
        from tools.sim_perturbation import parse_perturbation
        _perturb_file = os.environ.get(
            "SIM_PERTURB_FILE", "/tmp/sim_perturbation.json"
        )
        _perturb_max_force = float(os.environ.get("SIM_PERTURB_MAX_FORCE_N", "500"))
        _perturb_max_torque = float(os.environ.get("SIM_PERTURB_MAX_TORQUE_NM", "200"))
        _perturb_max_duration = float(os.environ.get("SIM_PERTURB_MAX_DURATION_S", "2"))
        _perturb_command = None
        _perturb_until = 0.0
        _perturb_last_id = None
        _perturb_seen_text = None
        _perturb_cleared = True

        # use torch.inference_mode() and exception suppression
        with contextlib.suppress(KeyboardInterrupt), torch.inference_mode():
            while simulation_app.is_running() and controller.is_running:
                current_time = time.time()
                loop_count += 1
                if not args_cli.replay_data:
                    if sim_state_dds is not None:
                        try:
                            env_state = env.scene.get_state()
                            env_state_json = sim_state_to_json(env_state)
                            sim_state = {
                                "init_state": env_state_json,
                                "task_name": args_cli.task,
                            }
                            sim_state_dds.write_sim_state_data(sim_state)
                        except Exception as e:
                            print(f"Failed to write sim state: {e}")
                            raise e

                    if _multi_mode:
                        reset_requests = multi_robot_dds.pop_reset_requests()
                        reset_pose_cmd = None
                    else:
                        reset_requests = []
                        try:
                            reset_pose_cmd = reset_pose_dds.get_reset_pose_command()
                        except Exception as e:
                            print(f"Failed to get reset pose command: {e}")
                            raise e
                    # Compute current reward values manually if needed for debugging
                    try:
                        if (loop_count % reward_interval) == 0:
                            pass
                            # current_reward = get_step_reward_value(env)
                    except Exception as e:
                        print(f"奖励计算失败: {e}")
                        pass
                    
                    for robot_id, reset_seq, reset_category in reset_requests:
                        try:
                            env_id = torch.tensor(
                                [robot_id], dtype=torch.long, device=env.device
                            )
                            if reset_category in ("2", "4"):
                                robot = env.scene["robot"]
                                root = default_root_state_world(robot, env_id)
                                joint_q = robot.data.default_joint_pos[env_id].clone()
                                joint_dq = torch.zeros_like(
                                    robot.data.default_joint_vel[env_id]
                                )
                                robot.write_root_pose_to_sim(
                                    root[:, :7], env_ids=env_id
                                )
                                robot.write_root_velocity_to_sim(
                                    root[:, 7:], env_ids=env_id
                                )
                                robot.write_joint_state_to_sim(
                                    joint_q, joint_dq, env_ids=env_id
                                )
                                multi_robot_dds.invalidate_commands([robot_id])
                                env._base_hold_mask[robot_id] = True
                                if os.environ.get("SIM_WARMUP_JOINTS", "0") == "1":
                                    env._warmup_joint_mask[robot_id] = True
                                if stand_eval is not None:
                                    stand_eval[robot_id].disarm()
                                print(
                                    f"[sim:{robot_id}] re-arm hold "
                                    f"(reset seq {reset_seq})",
                                    flush=True,
                                )
                            elif reset_category == "3":
                                if not multi_robot_dds.has_fresh_command(robot_id):
                                    print(
                                        f"[sim:{robot_id}] release deferred: "
                                        "waiting for a post-rearm LowCmd",
                                        flush=True,
                                    )
                                    continue
                                env._base_hold_mask[robot_id] = False
                                env._warmup_joint_mask[robot_id] = False
                                if stand_eval is not None:
                                    stand_eval[robot_id].arm()
                                print(
                                    f"[sim:{robot_id}] release-only -> controller balances",
                                    flush=True,
                                )
                        except Exception as e:
                            print(
                                f"[sim:{robot_id}] reset request failed: {e}",
                                flush=True,
                            )
                            raise

                    if reset_pose_cmd is not None:
                        try:
                            reset_category = reset_pose_cmd.get("reset_category")
                            if (args_cli.enable_wholebody_dds and (reset_category == '1' or reset_category == '2')) or (not args_cli.enable_wholebody_dds and reset_category == '1'):
                                print("reset object")
                                env_cfg.event_manager.trigger("reset_object_self", env)
                                reset_pose_dds.write_reset_pose_command(-1)
                            elif reset_category == '2' and not args_cli.enable_wholebody_dds:
                                print("reset all")
                                env_cfg.event_manager.trigger("reset_all_self", env)
                                reset_pose_dds.write_reset_pose_command(-1)
                                # One-time snap the floating base UPRIGHT to the init pose
                                # (reset_scene_to_default leaves the base orientation fallen).
                                # No continuous pin: the SONIC controller (already active) then
                                # balances from this clean upright stance, exactly like the
                                # MuJoCo band-release. Requires the reset pose == the pose the
                                # controller balances at (captured from MuJoCo).
                                try:
                                    _rb = env.scene["robot"]
                                    _r0 = default_root_state_world(_rb)
                                    _rb.write_root_pose_to_sim(_r0[:, :7])
                                    _rb.write_root_velocity_to_sim(_r0[:, 7:])
                                except Exception as _e:
                                    print(f"[sim] reset root-upright failed: {_e}", flush=True)
                                # release any startup hold -> controller takes over
                                _base_hold_until = 0.0
                                try:
                                    env._warmup_joint_until = 0.0
                                except Exception:
                                    pass
                                if stand_eval is not None:
                                    stand_eval.arm()
                                print("[sim] reset -> upright; controller balances", flush=True)
                            elif reset_category == '3':
                                # RELEASE-ONLY: drop the base hold in place, NO teleport.
                                # For the SOFT-hold protocol the robot has already settled onto
                                # its feet at a real Isaac equilibrium; just let go so the
                                # controller continues from that in-distribution stance.
                                reset_pose_dds.write_reset_pose_command(-1)
                                _base_hold_until = 0.0
                                try:
                                    env._warmup_joint_until = 0.0
                                except Exception:
                                    pass
                                if stand_eval is not None:
                                    stand_eval.arm()
                                print("[sim] release-only (cat3) -> controller balances in place", flush=True)
                            elif reset_category == '4':
                                # RE-ARM HOLD: teleport the base back upright to init pose and
                                # re-enable the base-hold + joint warmup, WITHOUT restarting
                                # Isaac. Lets us re-run the hold->release test at a new sim
                                # speed (or after a fall) live. Pair with cat-3 to release.
                                try:
                                    _rb = env.scene["robot"]
                                    env_cfg.event_manager.trigger("reset_all_self", env)
                                    _r0 = default_root_state_world(_rb)
                                    _rb.write_root_pose_to_sim(_r0[:, :7])
                                    _rb.write_root_velocity_to_sim(_r0[:, 7:])
                                except Exception as _e:
                                    print(f"[sim] cat4 re-arm teleport failed: {_e}", flush=True)
                                reset_pose_dds.write_reset_pose_command(-1)
                                _base_hold_until = _HOLD_FOREVER
                                if os.environ.get("SIM_WARMUP_JOINTS", "0") == "1":
                                    try:
                                        env._warmup_joint_until = _HOLD_FOREVER
                                    except Exception:
                                        pass
                                if stand_eval is not None:
                                    stand_eval.disarm()
                                print("[sim] re-arm hold (cat4) -> base pinned upright again", flush=True)
                        except Exception as e:
                            print(f"Failed to write reset pose command: {e}")
                            raise e
                else:
                    if action_provider.get_start_loop() and data_idx<len(data_json_list):
                        print(f"data_idx: {data_idx}")
                        try:
                            sim_state,task_name = action_provider.load_data(data_json_list[data_idx])
                            if task_name!=args_cli.task:
                                raise ValueError(f" The {task_name} in the dataset is different from the {args_cli.task} being executed .")
                        except Exception as e:
                            print(f"Failed to load data: {e}")
                            raise e
                        try:
                            env.reset_to(sim_state, torch.tensor([0], device=env.device), is_relative=True)
                            env.sim.reset()
                            time.sleep(1)
                            action_provider.start_replay()
                            data_idx+=1
                        except Exception as e:
                            print(f"Failed to start replay: {e}")
                            raise e
                # print(f"env_state: {env_state}")
                # calculate instantaneous loop time
                loop_dt = current_time - last_loop_time
                last_loop_time = current_time
                recent_loop_times.append(loop_dt)
                
                # keep recent 100 loop times
                if len(recent_loop_times) > 100:
                    recent_loop_times.pop(0)

                # The rigid hold is applied after controller.step(), so the raw
                # PhysX velocity at the observation point describes motion that
                # is discarded by the post-step pose/velocity pin below.  Tell
                # the DDS observation path when that sampled velocity is not an
                # observable inter-step velocity.  Position and orientation stay
                # measured; only velocity channels are made consistent with the
                # rigidly held pose seen by the external controller.
                if _multi_mode:
                    env._rigid_hold_active = False
                    env._joint_warmup_active = False
                else:
                    env._rigid_hold_active = bool(
                        _base_hold_until > current_time and not _bh_soft
                    )
                    env._joint_warmup_active = bool(
                        time.time() < getattr(env, "_warmup_joint_until", 0.0)
                    )
                
                # execute control step (in main thread, support rendering)
                controller.step()

                # Exact joint freeze for the static-pose experiment.  Apply this after the
                # physics/control step, like the rigid base pin below, so the next LowState
                # contains precisely the requested default posture with zero joint speed.
                # On release the write stops and the already-running controller takes over.
                if _multi_mode and torch.any(env._warmup_joint_mask):
                    try:
                        _robot = env.scene["robot"]
                        _ids = torch.nonzero(
                            env._warmup_joint_mask, as_tuple=False
                        ).squeeze(-1)
                        _q0 = _robot.data.default_joint_pos[_ids]
                        _dq0 = torch.zeros_like(_robot.data.default_joint_vel[_ids])
                        _robot.write_joint_state_to_sim(
                            _q0, _dq0, env_ids=_ids
                        )
                    except Exception as _e:
                        print(f"[sim] multi joint-warmup pin failed: {_e}", flush=True)
                elif env._joint_warmup_active:
                    try:
                        _robot = env.scene["robot"]
                        _q0 = _robot.data.default_joint_pos
                        _dq0 = torch.zeros_like(_robot.data.default_joint_vel)
                        _robot.write_joint_state_to_sim(_q0, _dq0)
                    except Exception as _e:
                        print(f"[sim] joint-warmup pin failed: {_e}", flush=True)

                # virtual elastic band: apply a SOFT PD force+torque on the base while
                # the hold window is active (mirrors MuJoCo's band). A soft, continuously
                # applied wrench resists the base being flung by the warming-up controller
                # during the physics step -> the IMU stays clean and the policy's 4-step
                # history refills with upright data. A hard post-hoc pose write does NOT
                # (the base flies free during the step, corrupting base_ang_vel).
                if _multi_mode:
                    if torch.any(env._base_hold_mask):
                        try:
                            _robot = env.scene["robot"]
                            _ids = torch.nonzero(
                                env._base_hold_mask, as_tuple=False
                            ).squeeze(-1)
                            _cur = _robot.data.root_state_w[_ids].clone()
                            _r0 = default_root_state_world(_robot, _ids)
                            _pose = _cur[:, :7].clone()
                            _pose[:, 0:2] = _r0[:, 0:2]
                            _pose[:, 3:7] = _r0[:, 3:7]
                            _robot.write_root_pose_to_sim(_pose, env_ids=_ids)
                            _vel = _cur[:, 7:].clone()
                            _vel[:, 0:2] = 0.0
                            _vel[:, 3:6] = 0.0
                            _vel[:, 2] = torch.minimum(
                                _vel[:, 2], torch.zeros_like(_vel[:, 2])
                            )
                            _robot.write_root_velocity_to_sim(_vel, env_ids=_ids)
                        except Exception as _e:
                            print(f"[sim] multi base-hold pin failed: {_e}", flush=True)
                elif _base_hold_until > current_time and _bh_soft:
                    # SOFT compliant hold: clamped PD wrench on the pelvis in the WORLD frame.
                    # Applied via set_external_force_and_torque -> written to sim on the NEXT
                    # env.step (one-step lag, ~20ms). The base stays free to move & respond to
                    # the controller, keeping obs in-distribution (unlike the rigid pose-pin).
                    try:
                        _robot = env.scene["robot"]
                        _cur = _robot.data.root_state_w             # [N,13] world
                        _r0 = default_root_state_world(_robot)
                        if not torch.isfinite(_cur).all():
                            raise ValueError("non-finite root_state (robot exploded); skip wrench")
                        _pos = _cur[:, 0:3]; _quat = _cur[:, 3:7]
                        _lin = _cur[:, 7:10]; _ang = _cur[:, 10:13]
                        # horizontal position hold (leave vertical to the feet)
                        _perr = (_r0[:, 0:3] - _pos).clone()
                        _perr[:, 2] = 0.0
                        _force = _bh_kp_pos * _perr - _bh_kd_pos * _lin
                        _force = _force.clone(); _force[:, 2] = 0.0
                        # upright-orientation error via quaternion (wxyz): q_err = q_tgt * conj(q_cur)
                        _qw, _qx, _qy, _qz = _quat[:, 0], _quat[:, 1], _quat[:, 2], _quat[:, 3]
                        _tw, _tx, _ty, _tz = _r0[:, 3], _r0[:, 4], _r0[:, 5], _r0[:, 6]
                        _cw, _cx, _cy, _cz = _qw, -_qx, -_qy, -_qz
                        _ew = _tw*_cw - _tx*_cx - _ty*_cy - _tz*_cz
                        _ex = _tw*_cx + _tx*_cw + _ty*_cz - _tz*_cy
                        _ey = _tw*_cy - _tx*_cz + _ty*_cw + _tz*_cx
                        _ez = _tw*_cz + _tx*_cy - _ty*_cx + _tz*_cw
                        _sgn = torch.sign(_ew)
                        _sgn = torch.where(_sgn == 0, torch.ones_like(_sgn), _sgn)
                        _evec = 2.0 * _sgn.unsqueeze(-1) * torch.stack([_ex, _ey, _ez], dim=-1)
                        # NOTE: empirically the restoring term needs -evec (the framework's
                        # is_global torque / quaternion-error convention flips it); overdamped
                        # +evec still exploded (anti-restoring), -evec is stable.
                        _torque = -_bh_kp_ang * _evec - _bh_kd_ang * _ang
                        # clamp magnitudes to avoid PhysX blowup
                        _fm = torch.norm(_force, dim=-1, keepdim=True).clamp(min=1e-6)
                        _force = _force * (_fm.clamp(max=_bh_f_max) / _fm)
                        _tm = torch.norm(_torque, dim=-1, keepdim=True).clamp(min=1e-6)
                        _torque = _torque * (_tm.clamp(max=_bh_tq_max) / _tm)
                        _force = torch.nan_to_num(_force, nan=0.0, posinf=0.0, neginf=0.0)
                        _torque = torch.nan_to_num(_torque, nan=0.0, posinf=0.0, neginf=0.0)
                        _robot.set_external_force_and_torque(
                            _force.unsqueeze(1), _torque.unsqueeze(1),
                            body_ids=[_bh_base_bid], is_global=True)
                        _bh_soft_applied = True
                    except Exception as _e:
                        print(f"[sim] soft base-hold failed: {_e}", flush=True)
                elif _base_hold_until > current_time:
                    # RIGID orientation-only pin (legacy): force base upright, kill drift,
                    # leave vertical height free so the feet settle at natural height.
                    try:
                        _robot = env.scene["robot"]
                        _cur = _robot.data.root_state_w.clone()   # [N,13]
                        _r0 = default_root_state_world(_robot)
                        _pose = _cur[:, :7].clone()
                        _pose[:, 0:2] = _r0[:, 0:2]               # hold x,y at spawn
                        _pose[:, 3:7] = _r0[:, 3:7]               # force upright orientation
                        _robot.write_root_pose_to_sim(_pose)
                        _vel = _cur[:, 7:].clone()
                        _vel[:, 0:2] = 0.0                        # no horizontal drift
                        _vel[:, 3:6] = 0.0                        # no angular drift
                        _vel[:, 2] = min(float(_vel[0, 2]), 0.0)  # allow settling down only
                        _robot.write_root_velocity_to_sim(_vel)
                    except Exception as _e:
                        print(f"[sim] base-hold pin failed: {_e}", flush=True)
                elif _bh_soft_applied:
                    # release: disable the external wrench exactly once
                    try:
                        _robot = env.scene["robot"]
                        _z = torch.zeros((env.num_envs, 1, 3), device=env.device)
                        _robot.set_external_force_and_torque(
                            _z, _z, body_ids=[_bh_base_bid], is_global=True)
                    except Exception:
                        pass
                    _bh_soft_applied = False
                _bh_was_active = (
                    bool(torch.any(env._base_hold_mask))
                    if _multi_mode else _base_hold_until > current_time
                )

                # File-triggered targeted disturbance. Commands aimed at held
                # environments are rejected rather than queued and unexpectedly
                # applied after release.
                _sim_now = float(env.sim.current_time)
                try:
                    if os.path.exists(_perturb_file):
                        with open(_perturb_file, encoding="utf-8") as _stream:
                            _txt = _stream.read().strip()
                        if _txt and _txt != _perturb_seen_text:
                            # Mark it seen before parsing so a malformed manual
                            # write logs once rather than every physics tick.
                            _perturb_seen_text = _txt
                            _candidate = parse_perturbation(
                                _txt, env.num_envs,
                                max_force_n=_perturb_max_force,
                                max_torque_nm=_perturb_max_torque,
                                max_duration_s=_perturb_max_duration,
                            )
                            if _candidate.command_id != _perturb_last_id:
                                _perturb_last_id = _candidate.command_id
                                if _multi_mode:
                                    _eligible = tuple(
                                        robot_id for robot_id in _candidate.robot_ids
                                        if not bool(env._base_hold_mask[robot_id])
                                    )
                                else:
                                    _eligible = (() if _base_hold_until > current_time
                                                 else _candidate.robot_ids)
                                if not _eligible:
                                    print(
                                        f"[sim] PERTURBATION rejected id={_candidate.command_id}: "
                                        "all requested robots are held",
                                        flush=True,
                                    )
                                else:
                                    _perturb_command = (_candidate, _eligible)
                                    _perturb_until = _sim_now + _candidate.duration_s
                                    _perturb_cleared = False
                                    print(
                                        f"[sim] PERTURBATION start id={_candidate.command_id} "
                                        f"robots={list(_eligible)} force_n={_candidate.force_n} "
                                        f"torque_nm={_candidate.torque_nm} "
                                        f"duration_sim_s={_candidate.duration_s:.3f}",
                                        flush=True,
                                    )
                except Exception as _e:
                    print(f"[sim] perturbation command rejected: {_e}", flush=True)

                if _perturb_command is not None and _sim_now < _perturb_until:
                    try:
                        _candidate, _eligible = _perturb_command
                        _f = torch.zeros((env.num_envs, 1, 3), device=env.device)
                        _tq = torch.zeros_like(_f)
                        _ids = torch.tensor(_eligible, device=env.device, dtype=torch.long)
                        _f[_ids, 0, :] = torch.tensor(
                            _candidate.force_n, device=env.device, dtype=torch.float32
                        )
                        _tq[_ids, 0, :] = torch.tensor(
                            _candidate.torque_nm, device=env.device, dtype=torch.float32
                        )
                        env.scene["robot"].set_external_force_and_torque(
                            _f, _tq, body_ids=[_bh_base_bid], is_global=True
                        )
                    except Exception as _e:
                        print(f"[sim] perturbation apply failed: {_e}", flush=True)
                elif _perturb_command is not None and not _perturb_cleared:
                    try:
                        _z = torch.zeros((env.num_envs, 1, 3), device=env.device)
                        env.scene["robot"].set_external_force_and_torque(
                            _z, _z, body_ids=[_bh_base_bid], is_global=True
                        )
                    except Exception:
                        pass
                    _candidate, _eligible = _perturb_command
                    print(
                        f"[sim] PERTURBATION end id={_candidate.command_id} "
                        f"robots={list(_eligible)}",
                        flush=True,
                    )
                    _perturb_cleared = True
                    _perturb_command = None

                # deterministic balance eval -> rt/eval (sim-time throttled, never raises)
                if stand_eval is not None:
                    if _multi_mode:
                        for evaluator in stand_eval:
                            evaluator.update()
                    else:
                        stand_eval.update()

                # print statistics and loop frequency periodically
                if current_time - last_stats_time >= args_cli.stats_interval:
                    # calculate while loop execution frequency
                    elapsed_time = current_time - loop_start_time
                    loop_frequency = loop_count / elapsed_time if elapsed_time > 0 else 0
                    
                    # calculate moving average frequency (based on recent loop times)
                    if recent_loop_times:
                        avg_loop_time = sum(recent_loop_times) / len(recent_loop_times)
                        moving_avg_frequency = 1.0 / avg_loop_time if avg_loop_time > 0 else 0
                        min_loop_time = min(recent_loop_times)
                        max_loop_time = max(recent_loop_times)
                        max_freq = 1.0 / min_loop_time if min_loop_time > 0 else 0
                        min_freq = 1.0 / max_loop_time if max_loop_time > 0 else 0
                    else:
                        moving_avg_frequency = 0
                        min_freq = max_freq = 0
                    
                    print(f"\n=== While loop execution frequency statistics ===")
                    print(f"loop execution count: {loop_count}")
                    print(f"running time: {elapsed_time:.2f} seconds")
                    print(f"overall average frequency: {loop_frequency:.2f} Hz")
                    _rtf_sim = float(env.sim.current_time) - _rtf_sim_t0
                    print(f"[RTF] sim_time={_rtf_sim:.2f}s wall={elapsed_time:.2f}s RTF={(_rtf_sim/elapsed_time) if elapsed_time>0 else 0:.3f}")
                    print(f"moving average frequency: {moving_avg_frequency:.2f} Hz (last {len(recent_loop_times)} times)")
                    print(f"frequency range: {min_freq:.2f} - {max_freq:.2f} Hz")
                    print(f"average loop time: {(elapsed_time/loop_count*1000):.2f} ms")
                    if recent_loop_times:
                        print(f"recent loop time: {(avg_loop_time*1000):.2f} ms")
                    print(f"=============================")
                    
                    # print_stats(controller)
                    last_stats_time = current_time
       
                # check environment state
                if env.sim.is_stopped():
                    print("\nenvironment stopped")
                    break
                # DYNAMIC slow-mo: re-read the slowmo file ~2x/sec and update the target
                # period live (no restart). Format: a single float (e.g. "3.0").
                if _rt_base > 0.0 and current_time >= _slowmo_next_check:
                    _slowmo_next_check = current_time + 0.5
                    try:
                        with open(_slowmo_file) as _sf:
                            _new_slowmo = float(_sf.read().strip())
                        if _new_slowmo > 0.0 and abs(_new_slowmo - _slowmo) > 1e-6:
                            _slowmo = _new_slowmo
                            _rt_period = _rt_base * _slowmo
                            print(f"[sim] SLOWMO -> {_slowmo} (target period {_rt_period*1000:.1f} ms)", flush=True)
                    except (FileNotFoundError, ValueError):
                        pass
                # real-time throttle: sleep so this loop spans one control period
                if _rt_period > 0.0:
                    _rt_elapsed = time.time() - current_time
                    _rt_sleep = _rt_period - _rt_elapsed
                    if _rt_sleep > 0.0:
                        time.sleep(_rt_sleep)
    except KeyboardInterrupt:
        print("\nuser interrupted program")
    
    except Exception as e:
        print(f"\nprogram exception: {e}")
    
    finally:
        # clean up resources
        print("\nclean up resources...")
        controller.cleanup()
        image_server.stop()
        env.close()
        print("cleanup completed")
    # profiler.disable()
    # s = io.StringIO()
    # ps = pstats.Stats(profiler, stream=s).strip_dirs().sort_stats("time")
    # ps.print_stats(30)  

    # print(s.getvalue())

if __name__ == "__main__":
    try:
        main()
    finally:
        print("Performing final cleanup...")
        
        # Get current process information
        import os
        import subprocess
        import signal
        import time
        
        current_pid = os.getpid()
        print(f"Current main process PID: {current_pid}")
        
        try:
            # Find all related Python processes
            result = subprocess.run(['pgrep', '-f', 'sim_main.py'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                print(f"Found related processes: {pids}")
                
                for pid in pids:
                    if pid and pid != str(current_pid):
                        try:
                            print(f"Terminating child process: {pid}")
                            os.kill(int(pid), signal.SIGTERM)
                        except ProcessLookupError:
                            print(f"Process {pid} does not exist")
                        except Exception as e:
                            print(f"Failed to terminate process {pid}: {e}")
                
                # Wait for processes to exit
                time.sleep(2)
                
                # Check if there are any remaining processes, force kill them
                result2 = subprocess.run(['pgrep', '-f', 'sim_main.py'], 
                                       capture_output=True, text=True)
                if result2.returncode == 0:
                    remaining_pids = result2.stdout.strip().split('\n')
                    for pid in remaining_pids:
                        if pid and pid != str(current_pid):
                            try:
                                print(f"Force killing process: {pid}")
                                os.kill(int(pid), signal.SIGKILL)
                            except Exception as e:
                                print(f"Failed to force kill process {pid}: {e}")
                                
        except Exception as e:
            print(f"Error during process cleanup: {e}")
        
        try:
            simulation_app.close()
        except Exception as e:
            print(f"Failed to close simulation application: {e}")
            
        print("Program exit completed")
        
        # Force exit
        os._exit(0)

# python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-Cylinder-G129-Dex1-Joint   --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-G129-Inspire-Joint    --enable_inspire_dds --robot_type g129

# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex1-Joint     --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-G129-Dex3-Joint    --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task  Isaac-PickPlace-RedBlock-G129-Inspire-Joint    --enable_inspire_dds --robot_type g129


# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex1-Joint     --enable_dex1_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Dex3-Joint     --enable_dex3_dds --robot_type g129
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-G129-Inspire-Joint     --enable_inspire_dds --robot_type g129




# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Move-Cylinder-G129-Dex1-Wholebody  --robot_type g129 --enable_dex1_dds 
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Move-Cylinder-G129-Dex3-Wholebody  --robot_type g129 --enable_dex3_dds 
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Move-Cylinder-G129-Inspire-Wholebody  --robot_type g129 --enable_inspire_dds 


# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint  --enable_inspire_dds --robot_type h1_2
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-PickPlace-RedBlock-H12-27dof-Inspire-Joint  --enable_inspire_dds --robot_type h1_2
# python sim_main.py --device cpu  --enable_cameras  --task Isaac-Stack-RgyBlock-H12-27dof-Inspire-Joint --enable_inspire_dds --robot_type h1_2
