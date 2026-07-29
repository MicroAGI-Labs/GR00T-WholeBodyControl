#!/usr/bin/env python3
"""Robust single-instance control for the live Isaac-sim stack.

Why this exists (SIM_RESILIENCE_PLAN.md Workstream G): the bring-up scripts used
`pkill -9 -f "<pattern>"`, which (a) self-matches the shell running it, (b) can
partially kill (sim dies, sidecars orphan), and (c) races when two bring-ups
overlap -> duplicate rt/lowstate / rt/secondary_imu publishers silently corrupt
the DDS state (robot flails) with no error anywhere.

This identifies stack processes by their real interpreter/binary (`comm` starts
with "python" or "zenoh"), NOT by grepping full command lines, so it can never
match a bash/ssh/pgrep wrapper. Commands:

    kill    - SIGTERM then SIGKILL every stack process (clean slate)
    count   - print how many real instances of each component are running
    assert  - exit 0 iff each component has <= 1 instance, else 1 (+ detail)
"""
import glob, os, signal, sys, time

# component -> substring that uniquely appears in its cmdline
COMPONENTS = {
    "sim":   "sim_main.py",
    "zenoh": "zenoh-bridge-dds",
    "imu":   "secondary_imu_adapter",
    "cam":   "gear_sonic_camera_pub",
}


def _procs():
    """Yield (pid, comm, cmdline) for real stack processes only.

    A real stack process runs as the venv python or the zenoh binary; a bash -c
    wrapper (or this script, comm=python3) is excluded so we never self-match."""
    me = os.getpid()
    for d in glob.glob("/proc/[0-9]*"):
        pid = int(d.rsplit("/", 1)[1])
        if pid == me:
            continue
        try:
            comm = open(d + "/comm").read().strip()
            cmd = open(d + "/cmdline").read().replace("\x00", " ")
        except OSError:
            continue
        # Only the actual interpreters/binaries; excludes bash/sh/ssh/pgrep/grep
        # and this controller itself (comm=python3). The sim/imu/cam run under the
        # venv as comm="python"; the bridge as comm="zenoh-bridge-dd".
        if not (comm.startswith("python") and comm != "python3") and not comm.startswith("zenoh"):
            continue
        yield pid, comm, cmd


def _ppid(pid):
    """Parent PID from /proc/<pid>/stat (parse after the last ')' so a comm with
    spaces/parens can't shift the fields)."""
    try:
        data = open(f"/proc/{pid}/stat").read()
        return int(data[data.rfind(")") + 1:].split()[1])
    except Exception:
        return 0


def _by_component():
    raw = {k: [] for k in COMPONENTS}
    for pid, comm, cmd in _procs():
        for name, sig in COMPONENTS.items():
            if sig in cmd:
                raw[name].append((pid, comm, cmd[:70]))
    # Collapse parent+child trees into one instance: Isaac forks a worker child
    # with an IDENTICAL cmdline, so a single sim shows as 2 python procs. An
    # instance == a matched proc whose parent is NOT itself a matched proc of the
    # same component (i.e. the root of each tree). This makes the count reflect
    # real independent instances, not process trees.
    out = {}
    for name, hits in raw.items():
        pids = {pid for pid, _, _ in hits}
        out[name] = [(pid, comm, cmd) for (pid, comm, cmd) in hits if _ppid(pid) not in pids]
    return out


def cmd_count():
    for name, hits in _by_component().items():
        print(f"{name}: {len(hits)}")
        for pid, comm, cmd in hits:
            print(f"   pid={pid} comm={comm} :: {cmd}")


def cmd_kill():
    pids = sorted({pid for hits in _by_component().values() for pid, _, _ in hits})
    if not pids:
        print("kill: nothing running")
        return
    for pid in pids:
        try: os.kill(pid, signal.SIGTERM)
        except ProcessLookupError: pass
    time.sleep(2)
    for pid in pids:
        try: os.kill(pid, signal.SIGKILL)
        except ProcessLookupError: pass
    time.sleep(1)
    remaining = sum(len(h) for h in _by_component().values())
    print(f"kill: signalled {pids}; remaining={remaining}")


def cmd_assert():
    bad = {k: v for k, v in _by_component().items() if len(v) > 1}
    if bad:
        print("ASSERT FAILED — duplicate stack instances:")
        for name, hits in bad.items():
            print(f"  {name}: {len(hits)}")
            for pid, comm, cmd in hits:
                print(f"     pid={pid} :: {cmd}")
        sys.exit(1)
    counts = {k: len(v) for k, v in _by_component().items()}
    print(f"ASSERT OK — singletons: {counts}")
    sys.exit(0)


if __name__ == "__main__":
    action = sys.argv[1] if len(sys.argv) > 1 else "count"
    {"kill": cmd_kill, "count": cmd_count, "assert": cmd_assert}.get(action, cmd_count)()
