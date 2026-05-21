#!/usr/bin/env python3
"""PICO link telemetry / freeze diagnostic for the Unitree G1 teleop rig.

Purpose
-------
The PICO body-tracking feed periodically FREEZES ("a few frames then stops").
This script tells us *definitively* which failure mode is happening:

  (A) Network / device disconnect
      The SDK fires a PXREADeviceMissing event (PICO dropped its connection to
      the RoboticsService). In that case ``get_connection_status()`` shows
      ``connected == False`` and ``missing_count`` increments.

  (B) Frozen-but-connected
      The device stays "connected" (no DeviceMissing) but the body timestamp
      (``get_body_timestamp_ns()``) stops advancing -> headset / tracker
      tracking stalled while still connected.

This distinction tells us whether the WiFi/network is at fault (mode A) or the
headset tracking is at fault (mode B).

It calls ``xrt.init()`` and then prints, once per second for ~30 s, a
timestamped line with:
  - ``is_body_data_available()``
  - the body timestamp and whether it advanced since the last poll
  - ``get_connection_status()`` (connected, find/missing counts, seconds-since-*)

It clearly flags transitions:
  - "DEVICE MISSING fired"  (mode A)
  - "body ts FROZEN (connected but not advancing)"  (mode B)
  - "body ts advancing OK"

IMPORTANT
---------
Only ONE SDK client can use the roboticsservice at a time. The live teleop
stack runs ``pico_manager_thread_server.py``, which holds the SDK client.
DO NOT run this script while ``pico_manager`` is running -- it will conflict.
Stop pico_manager first, then run this, then restart pico_manager.

Usage
-----
    /home/microagi/repos/GR00T-WholeBodyControl/.venv_teleop/bin/python \\
        /home/microagi/repos/GR00T-WholeBodyControl/gear_sonic/scripts/pico_link_telemetry.py

Optional args:
    --duration SECONDS   how long to monitor (default 30)
    --rate HZ            poll/print rate in Hz (default 1.0)
"""

import argparse
import time
from datetime import datetime

import xrobotoolkit_sdk as xrt


def _ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def main() -> None:
    parser = argparse.ArgumentParser(description="PICO link telemetry / freeze diagnostic")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="how long to monitor, in seconds (default 30)")
    parser.add_argument("--rate", type=float, default=1.0,
                        help="poll/print rate in Hz (default 1.0)")
    args = parser.parse_args()

    period = 1.0 / args.rate if args.rate > 0 else 1.0

    print(f"[{_ts()}] init()  (make sure pico_manager is STOPPED -- only one SDK client allowed)")
    xrt.init()

    # Give the SDK a moment to connect and start delivering frames.
    print(f"[{_ts()}] waiting up to 5s for first body data ...")
    wait_deadline = time.time() + 5.0
    while not xrt.is_body_data_available() and time.time() < wait_deadline:
        time.sleep(0.05)

    prev_body_ts = None          # last body timestamp (ns) seen
    prev_missing_count = None     # detect new DeviceMissing events
    prev_find_count = None        # detect new DeviceFind events

    start = time.time()
    next_tick = start
    print(f"[{_ts()}] monitoring for {args.duration:.0f}s at {args.rate:g} Hz")
    print("-" * 100)

    try:
        while time.time() - start < args.duration:
            now = time.time()
            if now < next_tick:
                time.sleep(min(0.02, next_tick - now))
                continue
            next_tick += period

            available = bool(xrt.is_body_data_available())
            body_ts = int(xrt.get_body_timestamp_ns())
            status = xrt.get_connection_status()

            connected = bool(status.get("connected", False))
            find_count = int(status.get("find_count", 0))
            missing_count = int(status.get("missing_count", 0))
            s_find = float(status.get("seconds_since_last_find", -1.0))
            s_missing = float(status.get("seconds_since_last_missing", -1.0))
            s_state = float(status.get("seconds_since_last_state_update", -1.0))

            # --- transition / mode detection ------------------------------------
            flags = []

            if prev_missing_count is not None and missing_count > prev_missing_count:
                flags.append(">>> DEVICE MISSING fired (mode A: network/device disconnect)")
            if prev_find_count is not None and find_count > prev_find_count:
                flags.append(">>> DEVICE FIND fired (reconnected)")

            if prev_body_ts is None:
                ts_state = "first sample"
                advanced = None
            else:
                advanced = body_ts != prev_body_ts
                if advanced:
                    ts_state = "body ts advancing OK"
                elif connected:
                    ts_state = "body ts FROZEN (connected but not advancing) (mode B: tracking stall)"
                    flags.append(">>> " + ts_state)
                else:
                    ts_state = "body ts not advancing (disconnected)"

            delta_ts_ms = (body_ts - prev_body_ts) / 1e6 if prev_body_ts is not None else 0.0

            line = (
                f"[{_ts()}] "
                f"avail={int(available)} "
                f"connected={int(connected)} "
                f"body_ts={body_ts} (d={delta_ts_ms:+.1f}ms) "
                f"find={find_count} missing={missing_count} "
                f"s_since_find={s_find:.1f} s_since_missing={s_missing:.1f} "
                f"s_since_state={s_state:.2f} "
                f"| {ts_state}"
            )
            print(line)
            for f in flags:
                print(f"          {f}")

            prev_body_ts = body_ts
            prev_missing_count = missing_count
            prev_find_count = find_count

    except KeyboardInterrupt:
        print(f"\n[{_ts()}] interrupted by user")
    finally:
        print("-" * 100)
        # Final summary to make the diagnosis explicit.
        try:
            status = xrt.get_connection_status()
            print(f"[{_ts()}] SUMMARY: connected={bool(status.get('connected'))} "
                  f"total_find={int(status.get('find_count', 0))} "
                  f"total_missing={int(status.get('missing_count', 0))} "
                  f"s_since_last_state_update={float(status.get('seconds_since_last_state_update', -1.0)):.2f}")
            if int(status.get("missing_count", 0)) > 0:
                print(f"[{_ts()}] => At least one DEVICE MISSING occurred -> mode (A) "
                      f"network/device disconnect is in play.")
            else:
                print(f"[{_ts()}] => No DEVICE MISSING events. If you saw 'body ts FROZEN' "
                      f"above, this is mode (B) tracking stall while connected.")
        except Exception as exc:  # noqa: BLE001
            print(f"[{_ts()}] could not read final status: {exc}")
        try:
            xrt.close()
        except Exception:  # noqa: BLE001
            pass
        print(f"[{_ts()}] closed SDK. Remember to restart pico_manager.")


if __name__ == "__main__":
    main()
