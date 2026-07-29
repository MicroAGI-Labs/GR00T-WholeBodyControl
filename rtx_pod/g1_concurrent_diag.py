#!/usr/bin/env python3
"""One-participant lifecycle and diagnostics for concurrent simulated G1s.

Creating one diagnostic process per robot exhausts CycloneDDS's default
participant-index range before the controller itself reaches useful scale.  This
tool creates all per-robot publishers/subscribers under one participant.
"""

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
import sys
import threading
import time

try:
    import unitree_sdk2py.core.channel as channel_module
except ModuleNotFoundError:
    sys.path.insert(0, os.path.expanduser("~/unitree_sdk2_python"))
    import unitree_sdk2py.core.channel as channel_module

from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_


def configure_participant_range(max_index):
    """Extend the SDK's generated CycloneDDS XML before factory init."""
    discovery = (
        "<Discovery><ParticipantIndex>auto</ParticipantIndex>"
        f"<MaxAutoParticipantIndex>{max_index}</MaxAutoParticipantIndex>"
        "<Peers><Peer Address=\"127.0.0.1\" /></Peers></Discovery>"
    )
    for name in ("ChannelConfigHasInterface", "ChannelConfigAutoDetermine"):
        template = getattr(channel_module, name)
        if "MaxAutoParticipantIndex" not in template:
            setattr(channel_module, name, template.replace("</Domain>", discovery + "</Domain>"))


def init_dds(args):
    configure_participant_range(args.max_participant_index)
    channel_module.ChannelFactoryInitialize(args.domain, args.interface)


def topic(args, robot_id, suffix):
    return f"{args.prefix.rstrip('/')}/{robot_id}/{suffix}"


def lifecycle(args):
    init_dds(args)
    pubs = []
    for robot_id in range(args.count):
        pub = channel_module.ChannelPublisher(
            topic(args, robot_id, "reset_pose/cmd"), String_
        )
        pub.Init()
        pubs.append(pub)
    time.sleep(args.delay)
    message = std_msgs_msg_dds__String_()
    message.data = str(args.category)
    for pub in pubs:
        pub.Write(message)
    print(
        f"category {args.category} sent to {args.count} robots "
        f"after {args.delay:g}s discovery",
        flush=True,
    )
    time.sleep(1)


def probe(args):
    init_dds(args)
    state_arrivals = [[] for _ in range(args.count)]
    command_arrivals = [[] for _ in range(args.count)]
    subscribers = []

    def handler(arrivals, robot_id):
        return lambda _message: arrivals[robot_id].append(time.monotonic())

    def percentile(values, fraction):
        ordered = sorted(values)
        return ordered[min(round((len(ordered) - 1) * fraction), len(ordered) - 1)]

    def report(label, arrivals):
        missing = 0
        for robot_id, stamps in enumerate(arrivals):
            if len(stamps) < 2:
                missing += 1
                print(f"{label} r{robot_id:02d} msgs={len(stamps)} NO_RATE")
                continue
            gaps = [(b - a) * 1000.0 for a, b in zip(stamps, stamps[1:])]
            rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
            print(
                f"{label} r{robot_id:02d} msgs={len(stamps)} rate={rate:6.1f}Hz "
                f"gap_p50={percentile(gaps, 0.50):6.1f}ms "
                f"p95={percentile(gaps, 0.95):6.1f}ms "
                f"p99={percentile(gaps, 0.99):6.1f}ms "
                f"max={max(gaps):6.1f}ms"
            )
        return missing

    for robot_id in range(args.count):
        sub = channel_module.ChannelSubscriber(
            topic(args, robot_id, "lowstate"), LowState_
        )
        sub.Init(handler(state_arrivals, robot_id), 10)
        subscribers.append(sub)
        if args.include_commands:
            sub = channel_module.ChannelSubscriber(
                topic(args, robot_id, "lowcmd"), LowCmd_
            )
            sub.Init(handler(command_arrivals, robot_id), 10)
            subscribers.append(sub)
    time.sleep(args.duration)

    missing = report("state", state_arrivals)
    if args.include_commands:
        missing += report("cmd  ", command_arrivals)
    expected = args.count * (2 if args.include_commands else 1)
    print(f"SUMMARY received={expected - missing}/{expected}")
    return 1 if missing else 0


def eval_all(args):
    init_dds(args)
    latest = [None] * args.count
    subscribers = []

    def handler(robot_id):
        def receive(message):
            try:
                latest[robot_id] = json.loads(message.data)
            except (TypeError, ValueError):
                pass
        return receive

    for robot_id in range(args.count):
        sub = channel_module.ChannelSubscriber(topic(args, robot_id, "eval"), String_)
        sub.Init(handler(robot_id), 10)
        subscribers.append(sub)

    deadline = time.monotonic() + args.wall_timeout
    next_report = time.monotonic() + args.interval
    outcome = "timeout"
    while time.monotonic() < deadline:
        received = [item for item in latest if item is not None]
        if any(item.get("fallen") for item in received):
            outcome = "fall"
            break
        if len(received) == args.count and min(item.get("t", 0.0) for item in received) >= args.sim_seconds:
            outcome = "pass"
            break
        if time.monotonic() >= next_report:
            min_t = min((item.get("t", 0.0) for item in received), default=0.0)
            print(f"progress received={len(received)}/{args.count} min_t={min_t:.2f}s", flush=True)
            next_report += args.interval
        time.sleep(0.1)

    fallen = 0
    for robot_id, item in enumerate(latest):
        if item is None:
            print(f"r{robot_id:02d} NO_EVAL")
            continue
        is_fallen = bool(item.get("fallen"))
        fallen += int(is_fallen)
        flag = "FALL" if is_fallen else "stand"
        print(
            f"r{robot_id:02d} t={item.get('t', 0.0):6.2f}s {flag:5s} "
            f"dist={item.get('dist', 0.0):5.2f}m "
            f"z={item.get('height', 0.0):.3f} "
            f"tilt={item.get('tilt', 0.0):5.1f}deg "
            f"result={item.get('result', 0.0):+7.2f}"
        )
    received_count = sum(item is not None for item in latest)
    min_t = min((item.get("t", 0.0) for item in latest if item is not None), default=0.0)
    print(
        f"SUMMARY outcome={outcome} received={received_count}/{args.count} "
        f"fallen={fallen} min_t={min_t:.2f}s"
    )
    return 0 if outcome == "pass" and not fallen else 1


def monitor(args):
    """Continuously record compact transport timing and in-sim fall telemetry."""
    init_dds(args)
    lock = threading.Lock()
    events = deque()
    streams = {
        "state": [dict(last=None, count=0, max_gap_ms=0.0, tick=None, fall_latched=False) for _ in range(args.count)],
        "cmd": [dict(last=None, count=0, max_gap_ms=0.0, tick=None) for _ in range(args.count)],
        "eval": [dict(last=None, count=0, max_gap_ms=0.0, value=None, fall_latched=False) for _ in range(args.count)],
    }
    subscribers = []

    def wall_time():
        return datetime.now(timezone.utc).isoformat(timespec="milliseconds")

    def arrival(stream, robot_id, value=None, tick=None):
        now = time.monotonic()
        with lock:
            item = streams[stream][robot_id]
            previous = item["last"]
            previous_tick = item.get("tick")
            gap_ms = 0.0 if previous is None else (now - previous) * 1000.0
            item["last"] = now
            item["count"] += 1
            item["max_gap_ms"] = max(item["max_gap_ms"], gap_ms)
            if tick is not None:
                item["tick"] = int(tick)
            if value is not None:
                item["value"] = value
            if stream != "eval" and previous is not None and gap_ms >= args.gap_ms:
                events.append({
                    "type": "gap", "wall_time": wall_time(),
                    "stream": stream, "robot_id": robot_id,
                    "gap_ms": round(gap_ms, 3),
                    "tick_before": previous_tick, "tick_after": item.get("tick"),
                    "tick_delta": (
                        None if previous_tick is None or item.get("tick") is None
                        else (item["tick"] - previous_tick) % (2 ** 32)
                    ),
                })
            if (stream in ("state", "eval") and value and value.get("fallen")
                    and not item.get("fall_latched", False)):
                item["fall_latched"] = True
                events.append({
                    "type": "fall" if stream == "eval" else "state_fall",
                    "wall_time": wall_time(), "robot_id": robot_id,
                    stream: value,
                })

    def state_handler(robot_id):
        def receive(message):
            quat = list(message.imu_state.quaternion)
            _, x, y, _ = quat
            cosine = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
            tilt = math.degrees(math.acos(cosine))
            arrival(
                "state", robot_id,
                value={"tilt": round(tilt, 3), "fallen": tilt > 50.0, "quaternion": quat},
                tick=getattr(message, "tick", None),
            )
        return receive

    def cmd_handler(robot_id):
        return lambda _message: arrival("cmd", robot_id)

    def eval_handler(robot_id):
        def receive(message):
            try:
                value = json.loads(message.data)
            except (TypeError, ValueError):
                return
            arrival("eval", robot_id, value=value)
        return receive

    for robot_id in range(args.count):
        for suffix, message_type, handler in (
            ("lowstate", LowState_, state_handler(robot_id)),
            ("lowcmd", LowCmd_, cmd_handler(robot_id)),
            ("eval", String_, eval_handler(robot_id)),
        ):
            sub = channel_module.ChannelSubscriber(topic(args, robot_id, suffix), message_type)
            sub.Init(handler, 10)
            subscribers.append(sub)

    output = open(args.out, "a", buffering=1)
    start = time.monotonic()
    deadline = start + args.duration if args.duration > 0 else float("inf")
    next_report = start
    fall_seen = set()
    output.write(json.dumps({
        "type": "start", "wall_time": wall_time(), "count": args.count,
        "gap_threshold_ms": args.gap_ms, "prefix": args.prefix,
    }, separators=(",", ":")) + "\n")
    print(f"monitoring {args.count} robots -> {args.out} (Ctrl-C to stop)", flush=True)

    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_report:
                time.sleep(min(0.1, next_report - now))
                continue
            with lock:
                pending = list(events)
                events.clear()
                robots = []
                for robot_id in range(args.count):
                    record = {"robot_id": robot_id}
                    for stream in ("state", "cmd", "eval"):
                        item = streams[stream][robot_id]
                        record[f"{stream}_age_ms"] = (
                            None if item["last"] is None else round((now - item["last"]) * 1000.0, 3)
                        )
                        record[f"{stream}_count"] = item["count"]
                        record[f"{stream}_max_gap_ms"] = round(item["max_gap_ms"], 3)
                    record["state_tick"] = streams["state"][robot_id].get("tick")
                    state_value = streams["state"][robot_id].get("value")
                    if state_value:
                        record["state_tilt"] = state_value["tilt"]
                        if state_value["fallen"]:
                            fall_seen.add(robot_id)
                    value = streams["eval"][robot_id].get("value")
                    if value:
                        record["eval"] = {
                            key: value.get(key) for key in
                            ("t", "fallen", "passed", "dist", "height", "tilt", "reason")
                        }
                        if value.get("fallen"):
                            fall_seen.add(robot_id)
                    robots.append(record)
            for event in pending:
                output.write(json.dumps(event, separators=(",", ":")) + "\n")
            snapshot = {
                "type": "snapshot", "wall_time": wall_time(),
                "elapsed_wall_s": round(now - start, 3), "robots": robots,
            }
            output.write(json.dumps(snapshot, separators=(",", ":")) + "\n")
            state_ages = [r["state_age_ms"] for r in robots if r["state_age_ms"] is not None]
            cmd_ages = [r["cmd_age_ms"] for r in robots if r["cmd_age_ms"] is not None]
            eval_times = [r["eval"]["t"] for r in robots if r.get("eval") and r["eval"]["t"] is not None]
            print(
                f"wall={now-start:7.1f}s eval_min={min(eval_times, default=0):7.2f}s "
                f"state_age_max={max(state_ages, default=float('nan')):6.1f}ms "
                f"cmd_age_max={max(cmd_ages, default=float('nan')):6.1f}ms "
                f"fallen={len(fall_seen)}/{args.count}",
                flush=True,
            )
            if args.stop_on_fall and fall_seen:
                break
            next_report += args.interval
    except KeyboardInterrupt:
        pass
    finally:
        output.write(json.dumps({
            "type": "stop", "wall_time": wall_time(),
            "elapsed_wall_s": round(time.monotonic() - start, 3),
            "fallen": sorted(fall_seen),
        }, separators=(",", ":")) + "\n")
        output.close()
    return 1 if fall_seen else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--prefix", default="rt/sim/g1")
    parser.add_argument("--max-participant-index", type=int, default=99)
    commands = parser.add_subparsers(dest="command", required=True)

    lifecycle_parser = commands.add_parser("lifecycle")
    lifecycle_parser.add_argument("category", type=int, choices=(1, 2, 3, 4))
    lifecycle_parser.add_argument("--delay", type=float, default=2.0)
    lifecycle_parser.set_defaults(run=lifecycle)

    probe_parser = commands.add_parser("probe")
    probe_parser.add_argument("--duration", type=float, default=5.0)
    probe_parser.add_argument(
        "--include-commands", action="store_true",
        help="also report per-robot lowcmd timing in the reverse direction",
    )
    probe_parser.set_defaults(run=probe)

    eval_parser = commands.add_parser("eval")
    eval_parser.add_argument("--sim-seconds", type=float, default=10.0)
    eval_parser.add_argument("--wall-timeout", type=float, default=120.0)
    eval_parser.add_argument("--interval", type=float, default=10.0)
    eval_parser.set_defaults(run=eval_all)

    monitor_parser = commands.add_parser(
        "monitor", help="log continuous state/command gaps and eval/fall status"
    )
    monitor_parser.add_argument("--duration", type=float, default=0.0,
                                help="wall seconds; 0 runs until interrupted")
    monitor_parser.add_argument("--interval", type=float, default=5.0)
    monitor_parser.add_argument("--gap-ms", type=float, default=100.0)
    monitor_parser.add_argument("--out", default="/tmp/g1_concurrent_monitor.jsonl")
    monitor_parser.add_argument("--stop-on-fall", action="store_true")
    monitor_parser.set_defaults(run=monitor)

    args = parser.parse_args()
    if args.count < 1 or args.count > 24:
        parser.error("--count must be between 1 and 24")
    raise SystemExit(args.run(args))


if __name__ == "__main__":
    main()
