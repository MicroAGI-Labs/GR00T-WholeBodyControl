#!/usr/bin/env python3
"""g1_dds_diag.py — G1 SONIC balance-stack DDS diagnostics.

One CLI over the stack's DDS traffic.  With no prefix it observes the legacy
``rt/lowstate`` / ``rt/lowcmd`` topics.  ``--topic-prefix rt/sim/g1/0``
selects one robot in a concurrent simulation.

Subcommands
-----------
  watch     live base tilt / knee / |gyro| — the quick "is it standing?" check
  eval      print the in-sim deterministic balance eval (rt/eval: termination + score)
  capture   record measured+commanded 29-joint state + IMU to a CSV
  probe     rt/lowstate inter-arrival gaps — diagnose the deploy's 'Lost LowState'
  warm      hold the zenoh<->DDS route warm (persistent subscriber)

Domains
-------
The SONIC deploy and the Spark-side subscribers run on DDS **domain 0** (the
``lo`` interface); the pod Isaac-sim DDS is **domain 1**. Pass ``--domain`` to
match where you're listening. Default 0 (Spark/deploy side).

Examples
--------
  ./g1_dds_diag.py watch                 # legacy topics, domain 0
  ./g1_dds_diag.py --topic-prefix rt/sim/g1/0 watch 10
  ./g1_dds_diag.py --domain 0 --topic-prefix rt/sim/g1/1 capture 16 robot1
  ./g1_dds_diag.py --domain 1 warm       # keep pod-side route warm
"""
import argparse
import json
import math
import os
import sys
import time

# unitree_sdk2py is normally on the default path; fall back to the known
# Spark checkout if it isn't (keeps the tool runnable in a bare shell).
try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
except ModuleNotFoundError:
    sys.path.insert(0, os.path.expanduser("~/unitree_sdk2_python"))
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

N = 29  # G1 29-DoF

# Canonical Unitree 29-joint order: L-leg[0:6] R-leg[6:12] waist[12:15] arms[15:29].
JOINT_NAMES = [
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_ank_pitch", "L_ank_roll",
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_ank_pitch", "R_ank_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "L_sh_pitch", "L_sh_roll", "L_sh_yaw", "L_elbow", "L_wr_roll", "L_wr_pitch", "L_wr_yaw",
    "R_sh_pitch", "R_sh_roll", "R_sh_yaw", "R_elbow", "R_wr_roll", "R_wr_pitch", "R_wr_yaw",
]
# Handy leg-joint indices for the terse live readouts.
KNEE = (3, 9)
HIP_PITCH = (0, 6)
ANKLE_PITCH = (4, 10)


def tilt_deg(quat):
    """Base tilt from upright (deg) given an (w, x, y, z) quaternion."""
    _, x, y, _ = quat
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def gyro_norm(gyro):
    return math.sqrt(sum(v * v for v in gyro))


def _init(domain, interface=None):
    # Let CycloneDDS select its configured/default interface unless the caller
    # explicitly asks for one.  Hard-coding loopback made ``--domain 1`` unable
    # to observe the pod simulator, whose DDS participant is on eth0.
    if interface:
        ChannelFactoryInitialize(domain, interface)
    else:
        ChannelFactoryInitialize(domain)


def _topic(args, legacy_topic):
    """Return a legacy topic or its per-robot namespaced equivalent."""
    prefix = args.topic_prefix.strip().rstrip("/")
    if not prefix:
        return legacy_topic
    suffix = legacy_topic[3:] if legacy_topic.startswith("rt/") else legacy_topic
    return f"{prefix}/{suffix}"


# --------------------------------------------------------------------------- watch
def cmd_watch(args):
    """Live one-line tilt / knee / |gyro| — quick free-standing sanity check."""
    _init(args.domain, args.interface)
    st = {"quat": None, "gyro": None, "knee": None}

    def cb(m):
        st["quat"] = list(m.imu_state.quaternion)
        st["gyro"] = list(m.imu_state.gyroscope)
        st["knee"] = (m.motor_state[KNEE[0]].q, m.motor_state[KNEE[1]].q)

    state_topic = _topic(args, "rt/lowstate")
    ChannelSubscriber(state_topic, LowState_).Init(cb, 10)
    print(f"[watch] domain={args.domain} waiting for {state_topic}...", flush=True)
    t0 = time.time()
    while args.dur <= 0 or time.time() - t0 < args.dur:
        if st["quat"]:
            knee = (st["knee"][0] + st["knee"][1]) / 2
            print(f"  t={time.time()-t0:5.1f}s  tilt={tilt_deg(st['quat']):5.1f}deg  "
                  f"knee={knee:5.3f}  |gyro|={gyro_norm(st['gyro']):4.2f}", flush=True)
        time.sleep(args.interval)


# ---------------------------------------------------------------------------- eval
def cmd_eval(args):
    """Print the in-sim balance eval stream (rt/eval): termination + result score."""
    import json
    _init(args.domain, args.interface)
    st = {"last": None}
    eval_topic = _topic(args, "rt/eval")
    ChannelSubscriber(eval_topic, String_).Init(lambda m: st.__setitem__("last", m.data), 10)
    print(f"[eval] domain={args.domain} waiting for {eval_topic}...", flush=True)
    t0 = time.time()
    seen = None
    while args.dur <= 0 or time.time() - t0 < args.dur:
        raw = st["last"]
        if raw and raw != seen:
            seen = raw
            try:
                d = json.loads(raw)
            except ValueError:
                print(f"  (unparseable) {raw}", flush=True)
                time.sleep(args.interval)
                continue
            flag = "FALL" if d.get("fallen") else ("SUCCESS" if d.get("termination", 0) >= 1
                                                   else ("stand" if d.get("standing") else "hold"))
            print(f"  t={d.get('t', 0):6.2f}s  term={d.get('termination', 0):5.3f}  "
                  f"result={d.get('result', 0):+7.2f}  [{flag:7s}] "
                  f"dist={d.get('dist', 0):5.2f}m z={d.get('height', 0):5.3f} "
                  f"tilt={d.get('tilt', 0):5.1f}deg  {d.get('reason', '')}", flush=True)
        time.sleep(args.interval)


# ------------------------------------------------------------------------- capture
def cmd_capture(args):
    """Record measured/commanded joints, IMU, and latest absolute sim root pose."""
    _init(args.domain, args.interface)
    st = {"mq": None, "mdq": None, "mtau": None, "cq": None, "ckp": None, "ckd": None,
          "quat": None, "gyro": None, "acc": None, "eval": {}}

    def scb(m):
        st["mq"] = [m.motor_state[i].q for i in range(N)]
        st["mdq"] = [m.motor_state[i].dq for i in range(N)]
        st["mtau"] = [m.motor_state[i].tau_est for i in range(N)]
        st["quat"] = list(m.imu_state.quaternion)
        st["gyro"] = list(m.imu_state.gyroscope)
        st["acc"] = list(m.imu_state.accelerometer)

    def ccb(m):
        st["cq"] = [m.motor_cmd[i].q for i in range(N)]
        st["ckp"] = [m.motor_cmd[i].kp for i in range(N)]
        st["ckd"] = [m.motor_cmd[i].kd for i in range(N)]

    def ecb(m):
        try:
            st["eval"] = json.loads(m.data)
        except (TypeError, ValueError):
            pass

    state_topic = _topic(args, "rt/lowstate")
    command_topic = _topic(args, "rt/lowcmd")
    eval_topic = _topic(args, "rt/eval")
    ChannelSubscriber(state_topic, LowState_).Init(scb, 10)
    ChannelSubscriber(command_topic, LowCmd_).Init(ccb, 10)
    ChannelSubscriber(eval_topic, String_).Init(ecb, 10)

    out = args.out or f"/tmp/cap_{args.label}.csv"
    cols = (["t", "tilt", "wnorm"]
            + [f"mq{i}" for i in range(N)] + [f"mdq{i}" for i in range(N)]
            + [f"mtau{i}" for i in range(N)] + [f"cq{i}" for i in range(N)]
            + [f"ckp{i}" for i in range(N)] + [f"ckd{i}" for i in range(N)]
            + ["accx", "accy", "accz", "eval_t", "root_x", "root_y", "root_z",
               "root_qw", "root_qx", "root_qy", "root_qz", "eval_dist",
               "eval_tilt", "eval_result", "eval_termination"])
    print(f"[capture] domain={args.domain} prefix={args.topic_prefix or '<legacy>'} "
          f"label={args.label} dur={args.dur}s -> {out}", flush=True)
    time.sleep(1.0)  # let both topics arrive
    n = 0
    with open(out, "w") as f:
        f.write(",".join(cols) + "\n")
        t0 = time.time()
        while time.time() - t0 < args.dur:
            if st["mq"] and st["cq"]:
                ev = st["eval"]
                pos = ev.get("position") or [float("nan")] * 3
                quat = ev.get("quaternion") or [float("nan")] * 4
                row = ([f"{time.time()-t0:.3f}", f"{tilt_deg(st['quat']):.3f}",
                        f"{gyro_norm(st['gyro']):.4f}"]
                       + [f"{v:.4f}" for v in st["mq"]] + [f"{v:.4f}" for v in st["mdq"]]
                       + [f"{v:.4f}" for v in st["mtau"]] + [f"{v:.4f}" for v in st["cq"]]
                       + [f"{v:.2f}" for v in st["ckp"]] + [f"{v:.2f}" for v in st["ckd"]]
                       + [f"{v:.4f}" for v in st["acc"]]
                       + [f"{ev.get('t', float('nan')):.4f}"]
                       + [f"{v:.6f}" for v in pos] + [f"{v:.7f}" for v in quat]
                       + [f"{ev.get(k, float('nan')):.4f}" for k in
                          ("dist", "tilt", "result", "termination")])
                f.write(",".join(row) + "\n")
                n += 1
            time.sleep(args.period)
    print(f"[capture] wrote {n} rows to {out}", flush=True)


# --------------------------------------------------------------------------- probe
def cmd_probe(args):
    """Report rt/lowstate inter-arrival gaps — steady ~20ms vs burst-then-stall."""
    _init(args.domain, args.interface)
    state = {"n": 0, "last": None, "gaps": [], "first": None}

    def cb(m):
        now = time.time()
        state["n"] += 1
        if state["first"] is None:
            state["first"] = now
        if state["last"] is not None:
            state["gaps"].append(now - state["last"])
        state["last"] = now

    state_topic = _topic(args, "rt/lowstate")
    ChannelSubscriber(state_topic, LowState_).Init(cb, 10)
    print(f"[probe] domain={args.domain} listening to {state_topic} for {args.dur}s...", flush=True)
    t0 = time.time()
    while time.time() - t0 < args.dur:
        time.sleep(0.2)
    g = state["gaps"]
    if not g:
        print(f"[probe] got {state['n']} msgs — NO inter-arrival data (route down?).")
        return
    span = (state["last"] - state["first"]) if state["first"] else 0.0
    print(f"[probe] msgs={state['n']}  span={span:.2f}s  rate={state['n']/max(span,1e-9):.1f}Hz")
    print(f"[probe] gap ms: min={min(g)*1e3:.1f}  mean={sum(g)/len(g)*1e3:.1f}  "
          f"max={max(g)*1e3:.1f}  (>{args.stall_ms}ms stalls: "
          f"{sum(1 for x in g if x*1e3 > args.stall_ms)})")


# ---------------------------------------------------------------------------- warm
def cmd_warm(args):
    """Persistent subscriber that keeps the zenoh<->DDS route warm; prints a heartbeat."""
    _init(args.domain, args.interface)
    n = [0]
    state_topic = _topic(args, "rt/lowstate")
    ChannelSubscriber(state_topic, LowState_).Init(lambda m: n.__setitem__(0, n[0] + 1), 10)
    print(f"[warm] domain={args.domain} holding {state_topic} route warm", flush=True)
    last = 0
    while True:
        time.sleep(2.0)
        print(f"[warm d{args.domain}] total={n[0]} (+{n[0]-last} in 2s)", flush=True)
        last = n[0]


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--domain", type=int, default=0,
                   help="DDS domain (0=Spark/deploy side, 1=pod sim). Default 0.")
    p.add_argument("--interface", default=None,
                   help="DDS network interface (default: CycloneDDS auto-selection).")
    p.add_argument("--topic-prefix", default="",
                   help="robot topic prefix, e.g. rt/sim/g1/0 (default: legacy rt/* topics)")
    sub = p.add_subparsers(dest="cmd", required=True)

    w = sub.add_parser("watch", help="live tilt/knee/|gyro| stand check")
    w.add_argument("dur", nargs="?", type=float, default=0.0, help="seconds (0=forever)")
    w.add_argument("--interval", type=float, default=0.5)
    w.set_defaults(func=cmd_watch)

    ev = sub.add_parser("eval", help="print the in-sim balance eval stream (rt/eval)")
    ev.add_argument("dur", nargs="?", type=float, default=0.0, help="seconds (0=forever)")
    ev.add_argument("--interval", type=float, default=0.2)
    ev.set_defaults(func=cmd_eval)

    c = sub.add_parser("capture", help="record 29-joint measured+commanded + IMU to CSV")
    c.add_argument("dur", nargs="?", type=float, default=16.0)
    c.add_argument("label", nargs="?", default="cap")
    c.add_argument("out", nargs="?", default=None, help="CSV path (default /tmp/cap_<label>.csv)")
    c.add_argument("--period", type=float, default=0.02, help="sample period s (~50Hz)")
    c.set_defaults(func=cmd_capture)

    pr = sub.add_parser("probe", help="rt/lowstate inter-arrival gap continuity")
    pr.add_argument("--dur", type=float, default=8.0)
    pr.add_argument("--stall-ms", type=float, default=100.0)
    pr.set_defaults(func=cmd_probe)

    wm = sub.add_parser("warm", help="hold the zenoh<->DDS route warm")
    wm.set_defaults(func=cmd_warm)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
