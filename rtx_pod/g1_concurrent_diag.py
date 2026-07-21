#!/usr/bin/env python3
"""One-participant lifecycle and diagnostics for concurrent simulated G1s.

Creating one diagnostic process per robot exhausts CycloneDDS's default
participant-index range before the controller itself reaches useful scale.  This
tool creates all per-robot publishers/subscribers under one participant.
"""

import argparse
import json
import os
import sys
import time

try:
    import unitree_sdk2py.core.channel as channel_module
except ModuleNotFoundError:
    sys.path.insert(0, os.path.expanduser("~/unitree_sdk2_python"))
    import unitree_sdk2py.core.channel as channel_module

from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


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
    arrivals = [[] for _ in range(args.count)]
    subscribers = []

    def handler(robot_id):
        return lambda _message: arrivals[robot_id].append(time.monotonic())

    for robot_id in range(args.count):
        sub = channel_module.ChannelSubscriber(
            topic(args, robot_id, "lowstate"), LowState_
        )
        sub.Init(handler(robot_id), 10)
        subscribers.append(sub)
    time.sleep(args.duration)

    missing = 0
    for robot_id, stamps in enumerate(arrivals):
        if len(stamps) < 2:
            missing += 1
            print(f"r{robot_id:02d} msgs={len(stamps)} NO_RATE")
            continue
        gaps = [(b - a) * 1000.0 for a, b in zip(stamps, stamps[1:])]
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        print(
            f"r{robot_id:02d} msgs={len(stamps)} rate={rate:6.1f}Hz "
            f"max_gap={max(gaps):6.1f}ms"
        )
    print(f"SUMMARY received={args.count - missing}/{args.count}")
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
    probe_parser.set_defaults(run=probe)

    eval_parser = commands.add_parser("eval")
    eval_parser.add_argument("--sim-seconds", type=float, default=10.0)
    eval_parser.add_argument("--wall-timeout", type=float, default=120.0)
    eval_parser.add_argument("--interval", type=float, default=10.0)
    eval_parser.set_defaults(run=eval_all)

    args = parser.parse_args()
    if args.count < 1 or args.count > 24:
        parser.error("--count must be between 1 and 24")
    raise SystemExit(args.run(args))


if __name__ == "__main__":
    main()
