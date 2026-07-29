#!/usr/bin/env python3
"""Publish Isaac Sim camera frames in the gear_sonic camera-server wire format.

Taps the sim's shared memory (tools.shared_memory_utils.MultiImageReader, keys
head/left/right) and republishes on a ZMQ PUB socket in EXACTLY the format the
real G1 Orin camera server uses (gear_sonic/camera/sensor_server.py):

    msgpack.packb({"timestamps": {k: float}, "images": {k: <jpeg bytes>}},
                  use_bin_type=True)   # top-level plain msgpack; images are JPEG

So the Spark VLA client (run_vla_inference.py --camera-host <this box>) is
byte-identical whether it talks to the sim or the real robot. Decoupled from the
physics loop — reads shared memory at its own rate, zero per-step sim cost.

Run from the unitree_sim_isaaclab repo root, in the sim venv, while the sim runs.
"""
import argparse
import time

import cv2
import msgpack
import numpy as np
import zmq

from tools.shared_memory_utils import MultiImageReader

# sim shared-memory name -> gear_sonic camera key (matches CameraMountPosition)
KEY_MAP = {"head": "ego_view", "left": "left_wrist", "right": "right_wrist"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--jpeg-quality", type=int, default=80)  # gear_sonic default
    ap.add_argument("--rgb-input", action="store_true", default=True,
                    help="Isaac frames are RGB; convert to BGR before JPEG so the "
                         "gear_sonic client's BGR->RGB flip restores RGB (Orin parity).")
    args = ap.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 20)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind(f"tcp://*:{args.port}")

    reader = MultiImageReader()
    period = 1.0 / args.fps
    enc = [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality]
    print(f"[gear_sonic_cam_pub] PUB tcp://*:{args.port} @ {args.fps} FPS, keys={list(KEY_MAP.values())}", flush=True)

    n = 0
    last_log = time.time()
    while True:
        t0 = time.time()
        images = reader.read_images()  # {'head','left','right': HxWxC uint8} or None
        if images:
            ts = time.time()
            out_images = {}
            out_ts = {}
            for shm_key, arr in images.items():
                gk = KEY_MAP.get(shm_key)
                if gk is None or arr is None:
                    continue
                frame = arr
                if args.rgb_input and frame.ndim == 3 and frame.shape[2] == 3:
                    frame = frame[..., ::-1]  # RGB -> BGR (cv2/gear_sonic convention)
                ok, buf = cv2.imencode(".jpg", frame, enc)
                if not ok:
                    continue
                out_images[gk] = buf.tobytes()
                out_ts[gk] = ts
            if out_images:
                payload = {"timestamps": out_ts, "images": out_images}
                try:
                    sock.send(msgpack.packb(payload, use_bin_type=True), flags=zmq.NOBLOCK)
                    n += 1
                except zmq.Again:
                    pass
        if time.time() - last_log > 5.0:
            print(f"[gear_sonic_cam_pub] published {n} frames ({n/(time.time()-last_log):.1f}/s), "
                  f"cams={list(out_images.keys()) if images else 'NONE'}", flush=True)
            n = 0
            last_log = time.time()
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)


if __name__ == "__main__":
    main()
