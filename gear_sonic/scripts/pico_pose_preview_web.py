#!/usr/bin/env python3
"""Read-only browser preview for SONIC Pico pose targets."""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import threading
import time

import msgpack
import numpy as np
import zmq


TOPIC = b"pose_preview"
HEADER_SIZE = 1280
DTYPES = {
    "f32": np.dtype("<f4"), "f64": np.dtype("<f8"),
    "i32": np.dtype("<i4"), "i64": np.dtype("<i8"),
    "u8": np.dtype("u1"), "bool": np.dtype("?"),
}


def unpack(raw: bytes, topic: bytes = TOPIC) -> dict[str, np.ndarray]:
    if not raw.startswith(topic):
        raise ValueError("unexpected topic")
    start = len(topic)
    header = raw[start : start + HEADER_SIZE].split(b"\0", 1)[0]
    fields = json.loads(header.decode())["fields"]
    cursor = start + HEADER_SIZE
    result = {}
    for field in fields:
        dtype = DTYPES[field["dtype"]]
        shape = tuple(field["shape"])
        size = math.prod(shape) * dtype.itemsize
        result[field["name"]] = np.frombuffer(raw[cursor : cursor + size], dtype=dtype).reshape(shape)
        cursor += size
    return result


class PreviewState:
    def __init__(self, zmq_host: str, zmq_port: int):
        self.lock = threading.Lock()
        self.value = {"connected": False, "reason": "Waiting for pose_preview"}
        self.status = {
            "manager_mode": "UNKNOWN", "manager_connected": False,
            "sonic_process": False, "sonic_active": False,
        }
        self.zmq_host = zmq_host
        self.zmq_port = zmq_port
        self.last_positions = None

    def run(self):
        context = zmq.Context.instance()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, TOPIC)
        socket.setsockopt(zmq.CONFLATE, 1)
        socket.connect(f"tcp://{self.zmq_host}:{self.zmq_port}")
        while True:
            if not socket.poll(250):
                with self.lock:
                    age = time.monotonic() - self.value.get("received_monotonic", 0)
                    if age > 0.5:
                        self.value["connected"] = False
                        self.value["ready"] = False
                        self.value["reason"] = "Preview stale"
                continue
            try:
                data = unpack(socket.recv())
                joints = np.asarray(data["smpl_joints"][-1], dtype=float)
                calibrated_targets = np.asarray(data["vr_position"], dtype=float).reshape(3, 3)
                device_poses = np.asarray(data.get("device_poses", np.empty((0, 7))), dtype=float)
                # The rendered SMPL joints are root-local model coordinates. Raw
                # XR device poses are Unity-world coordinates and must not be
                # overlaid on them. Anchor display markers to the corresponding
                # SMPL joints: left wrist, right wrist, neck.
                targets = joints[[22, 23, 12]]
                device_targets = (
                    device_poses[:, :3] if device_poses.shape == (3, 7) else np.empty((0, 3))
                )
                fps = float(np.asarray(data.get("pico_fps", [0])).flat[0])
                calibrated = bool(np.asarray(data.get("calibrated", [False])).flat[0])
                finite = bool(np.isfinite(joints).all() and np.isfinite(targets).all())
                jump = 0.0
                if self.last_positions is not None and finite:
                    jump = float(np.linalg.norm(targets - self.last_positions, axis=1).max())
                if finite:
                    self.last_positions = targets.copy()
                checks = {
                    "calibrated": calibrated,
                    "tracking_fps": fps >= 60.0,
                    "finite": finite,
                    "target_bounds": finite and bool(np.abs(targets).max() < 2.0),
                    "frame_jump": finite and jump < 0.20,
                }
                ready = all(checks.values())
                value = {
                    "connected": True,
                    "ready": ready,
                    "reason": "Preview healthy" if ready else "Check failed",
                    "fps": round(fps, 1),
                    "jump_m": round(jump, 4),
                    "checks": checks,
                    "joints": joints.tolist(),
                    "targets": targets.tolist(),
                    "device_targets": device_targets.tolist(),
                    "calibrated_targets": calibrated_targets.tolist(),
                    "target_source": "SMPL left wrist / right wrist / neck",
                    "received_monotonic": time.monotonic(),
                }
                with self.lock:
                    self.value = value
            except Exception as exc:
                with self.lock:
                    self.value = {"connected": False, "reason": f"Decode error: {exc}"}

    def snapshot(self):
        with self.lock:
            result = dict(self.value)
            result.update(self.status)
        result.pop("received_monotonic", None)
        mode = result.get("manager_mode", "UNKNOWN")
        running = result.get("sonic_process", False)
        active = result.get("sonic_active", False)
        if not result.get("manager_connected"):
            action = "Reconnect Pico or restart the manager; manager state is stale."
            severity = "stop"
        elif not running and mode == "OFF":
            action = "Start SONIC and wait for Init Done. Keep the manager OFF."
            severity = "wait"
        elif not running:
            action = "Reset the manager to OFF before starting SONIC."
            severity = "stop"
        elif not active and mode == "OFF":
            action = "Stand neutral, then A+B+X+Y once to calibrate and start PLANNER."
            severity = "ready"
        elif not active:
            action = "SONIC is waiting but manager is already active. Reset manager to OFF, then recalibrate/start."
            severity = "stop"
        elif mode == "PLANNER":
            action = "PLANNER active. If preview is healthy, A+X once enters full-body POSE."
            severity = "ready"
        elif mode == "POSE":
            action = "Full-body teleoperation ACTIVE. A+X returns to PLANNER; A+B+X+Y stops."
            severity = "active"
        elif mode == "POSE_PAUSE":
            action = "Pose paused while menu is held. Release menu to resume, or A+B+X+Y to stop."
            severity = "wait"
        else:
            action = f"{mode} active. Use the documented transition or A+B+X+Y to stop."
            severity = "active"
        result["next_action"] = action
        result["action_severity"] = severity
        result["sonic_state"] = "ACTIVE CONTROL" if active else ("INIT DONE / WAITING" if running else "NOT RUNNING")
        return result

    @staticmethod
    def _sonic_process_running() -> bool:
        for entry in os.scandir("/proc"):
            if not entry.name.isdigit():
                continue
            try:
                cmdline = open(f"/proc/{entry.name}/cmdline", "rb").read()
            except (OSError, PermissionError):
                continue
            argv0 = cmdline.split(b"\0", 1)[0]
            if os.path.basename(os.fsdecode(argv0)) == "g1_deploy_onnx_ref":
                return True
        return False

    def run_status(self):
        context = zmq.Context.instance()
        manager = context.socket(zmq.SUB)
        manager.setsockopt(zmq.SUBSCRIBE, b"manager_state")
        manager.setsockopt(zmq.CONFLATE, 1)
        manager.connect(f"tcp://{self.zmq_host}:{self.zmq_port}")
        debug = context.socket(zmq.SUB)
        debug.setsockopt(zmq.SUBSCRIBE, b"g1_debug")
        debug.setsockopt(zmq.CONFLATE, 1)
        debug.connect(f"tcp://{self.zmq_host}:5557")
        poller = zmq.Poller()
        poller.register(manager, zmq.POLLIN)
        poller.register(debug, zmq.POLLIN)
        modes = {0: "OFF", 1: "POSE", 2: "PLANNER", 3: "PLANNER_FROZEN", 4: "POSE_PAUSE", 5: "PLANNER_VR_3PT"}
        last_manager = 0.0
        last_debug = 0.0
        while True:
            events = dict(poller.poll(200))
            if manager in events:
                try:
                    data = unpack(manager.recv(), b"manager_state")
                    mode = int(data["stream_mode"].flat[0])
                    last_manager = time.monotonic()
                    with self.lock:
                        self.status["manager_mode"] = modes.get(mode, f"MODE_{mode}")
                except Exception:
                    pass
            if debug in events:
                try:
                    raw = debug.recv()
                    msgpack.unpackb(raw[len(b"g1_debug"):], raw=False)
                    last_debug = time.monotonic()
                except Exception:
                    pass
            now = time.monotonic()
            with self.lock:
                self.status["manager_connected"] = now - last_manager < 0.75
                self.status["sonic_process"] = self._sonic_process_running()
                self.status["sonic_active"] = now - last_debug < 0.50


HTML = r'''<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>SONIC Pico Pose Preview</title><style>
:root{color-scheme:dark}body{margin:0;background:#0b1018;color:#e7edf6;font:15px system-ui,sans-serif}
main{max-width:1200px;margin:auto;padding:22px}.top{display:flex;gap:14px;align-items:center;flex-wrap:wrap}
h1{font-size:23px;margin:0 auto 0 0}.badge{padding:8px 13px;border-radius:999px;background:#7f1d1d;font-weight:700}
.ok{background:#14532d}.cards{display:grid;grid-template-columns:2fr 1fr;gap:16px;margin-top:18px}
.panel{background:#121a26;border:1px solid #273449;border-radius:12px;padding:15px}canvas{width:100%;height:610px;background:#080c12;border-radius:8px}
.states{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-top:16px}.state{background:#121a26;border:1px solid #273449;border-radius:10px;padding:12px}.state b{display:block;font-size:18px;margin-top:5px}
.action{margin-top:12px;padding:14px;border-radius:10px;background:#3b2f13;border:1px solid #a16207;font-weight:650}.action.stop{background:#3b171b;border-color:#b91c1c}.action.ready{background:#143523;border-color:#15803d}.action.active{background:#12314a;border-color:#0284c7}
.checks{display:grid;gap:9px}.check{padding:10px;border-radius:8px;background:#35171b}.check.ok{background:#143523}
.metric{font-size:28px;font-variant-numeric:tabular-nums}.note{color:#aebbd0;line-height:1.5}.warn{color:#fbbf24;font-weight:650}
@media(max-width:800px){.cards{grid-template-columns:1fr}.states{grid-template-columns:1fr}canvas{height:450px}}
</style></head><body><main><div class="top"><h1>SONIC Pico pose preview</h1><span id="status" class="badge">WAITING</span></div>
<p class="note">Read-only preview of <code>pose_preview</code>. This page has no robot-control endpoint. Inspect motion here while SONIC remains in planner mode.</p>
<div class="states"><div class="state">Pico manager<b id="manager">UNKNOWN</b></div><div class="state">SONIC controller<b id="sonic">UNKNOWN</b></div><div class="state">Calibration<b id="calibration">UNKNOWN</b></div></div><div id="action" class="action">Waiting for state…</div>
<div class="cards"><div class="panel"><canvas id="view"></canvas></div><div class="panel">
<div class="metric"><span id="fps">—</span> FPS</div><p>Targets: <b id="source">—</b></p><p>Largest target step: <b id="jump">—</b> m</p>
<div id="checks" class="checks"></div><p class="warn">“Preview healthy” is an inspection aid, not authorization to enable motion. Keep the robot supported and e-stop ready.</p>
</div></div></main><script>
const canvas=document.querySelector('#view'),ctx=canvas.getContext('2d');
const edges=[[0,1],[1,4],[4,7],[7,10],[0,2],[2,5],[5,8],[8,11],[0,3],[3,6],[6,9],[9,12],[12,15],[12,13],[13,16],[16,18],[18,20],[12,14],[14,17],[17,19],[19,21]];
function resize(){const d=devicePixelRatio||1,r=canvas.getBoundingClientRect();canvas.width=r.width*d;canvas.height=r.height*d;ctx.setTransform(d,0,0,d,0,0)}addEventListener('resize',resize);resize();
function project(p,side,w,h){const scale=Math.min(w,h)*.52;return side?[w*.75+p[0]*scale,h*.68-p[2]*scale]:[w*.25-p[1]*scale,h*.68-p[2]*scale]}
function draw(data){const r=canvas.getBoundingClientRect(),w=r.width,h=r.height;ctx.clearRect(0,0,w,h);ctx.font='14px system-ui';ctx.fillStyle='#9fb0c8';ctx.fillText('Front',w*.25-18,24);ctx.fillText('Side',w*.75-15,24);
 if(!data.joints)return;const root=data.joints[0];const local=data.joints.map(p=>[p[0]-root[0],p[1]-root[1],p[2]-root[2]]);const localTargets=(data.targets||[]).map(p=>[p[0]-root[0],p[1]-root[1],p[2]-root[2]]);for(const side of [false,true]){const pts=local.map(p=>project(p,side,w,h));ctx.strokeStyle='#64748b';ctx.lineWidth=3;for(const [a,b] of edges){if(a>=pts.length||b>=pts.length)continue;ctx.beginPath();ctx.moveTo(...pts[a]);ctx.lineTo(...pts[b]);ctx.stroke()}ctx.fillStyle='#d5deeb';for(const p of pts){ctx.beginPath();ctx.arc(p[0],p[1],3,0,7);ctx.fill()}
 const colors=['#38bdf8','#fb7185','#fbbf24'];localTargets.forEach((p,i)=>{const q=project(p,side,w,h);ctx.fillStyle=colors[i];ctx.beginPath();ctx.arc(q[0],q[1],8,0,7);ctx.fill()})}}
async function update(){try{const d=await fetch('/api/state',{cache:'no-store'}).then(r=>r.json());draw(d);document.querySelector('#fps').textContent=d.fps??'—';document.querySelector('#source').textContent=d.target_source??'—';document.querySelector('#jump').textContent=d.jump_m??'—';document.querySelector('#manager').textContent=d.manager_connected?d.manager_mode:'STALE';document.querySelector('#sonic').textContent=d.sonic_state;document.querySelector('#calibration').textContent=d.checks?.calibrated?'CAPTURED':'NOT CAPTURED';const a=document.querySelector('#action');a.textContent='Next: '+d.next_action;a.className='action '+d.action_severity;const s=document.querySelector('#status');s.textContent=d.ready?'PREVIEW HEALTHY':(d.reason||'WAITING').toUpperCase();s.className='badge '+(d.ready?'ok':'');const labels={calibrated:'Calibration captured',tracking_fps:'Tracking ≥ 60 FPS',finite:'Finite coordinates',target_bounds:'Targets within 2 m',frame_jump:'Target step < 20 cm'};document.querySelector('#checks').innerHTML=Object.entries(labels).map(([k,v])=>`<div class="check ${d.checks?.[k]?'ok':''}">${d.checks?.[k]?'✓':'✕'} ${v}</div>`).join('')}catch(e){}}setInterval(update,100);update();
</script></body></html>'''


def make_handler(state: PreviewState):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/api/state":
                body = json.dumps(state.snapshot(), allow_nan=False).encode()
                content_type = "application/json"
            elif self.path in ("/", "/index.html"):
                body = HTML.encode()
                content_type = "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format, *_args):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--zmq-host", default="127.0.0.1")
    parser.add_argument("--zmq-port", type=int, default=5556)
    args = parser.parse_args()
    state = PreviewState(args.zmq_host, args.zmq_port)
    threading.Thread(target=state.run, daemon=True).start()
    threading.Thread(target=state.run_status, daemon=True).start()
    server = ThreadingHTTPServer((args.bind, args.port), make_handler(state))
    print(f"Pico pose preview: http://{args.bind}:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
