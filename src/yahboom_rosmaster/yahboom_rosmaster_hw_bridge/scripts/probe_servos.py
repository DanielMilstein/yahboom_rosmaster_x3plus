#!/usr/bin/env python3
"""Manual servo-ID discovery sweep for the Yahboom ROSMaster X3 Plus.

Run this ON THE ORIN, not in a ROS launch. It does NOT need ROS — just
Rosmaster_Lib and a powered-on robot with the arm clear of obstacles.

Sweeps each candidate servo ID 1..7 with a short two-step motion
(70° → 110° → 90° park) and waits 1.2 s between each. Watch which
physical joint moves on each ID and write the mapping into
`config/servo_map.yaml`.

After running, also note the direction convention: when the script
sweeps from 70° to 110°, does the joint move in the same direction
that the URDF treats as positive rotation? If not, set `invert: true`
for that joint in `servo_map.yaml`.

Usage:
    python3 src/yahboom_rosmaster/yahboom_rosmaster_hw_bridge/scripts/probe_servos.py

Optional:
    --start N      first ID to sweep (default 1)
    --end N        last ID to sweep, inclusive (default 7)
    --low DEG      low angle of the sweep (default 70)
    --high DEG     high angle of the sweep (default 110)
    --hold-ms MS   per-segment hold time in ms (default 1200)
"""
from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=7)
    parser.add_argument("--low", type=int, default=70)
    parser.add_argument("--high", type=int, default=110)
    parser.add_argument("--hold-ms", type=int, default=1200, dest="hold_ms")
    args = parser.parse_args()

    try:
        from Rosmaster_Lib import Rosmaster
    except ImportError as exc:
        print(f"Rosmaster_Lib not importable: {exc}", file=sys.stderr)
        print("This script must be run on the Orin where the stock", file=sys.stderr)
        print("Yahboom driver is preinstalled.", file=sys.stderr)
        return 1

    bot = Rosmaster()
    bot.create_receive_threading()
    time.sleep(0.3)
    fw = bot.get_version()
    print(f"STM32 firmware version: {fw}")
    if fw is None or fw <= 0:
        print("WARN: firmware did not handshake. Carrying on; commands may not", file=sys.stderr)
        print("      reach the board. Check power and /dev/myserial.", file=sys.stderr)

    hold_s = args.hold_ms / 1000.0
    for sid in range(args.start, args.end + 1):
        print(f"\n=== probing servo id={sid} ===")
        try:
            cur = bot.get_uart_servo_angle(sid)
        except Exception as exc:  # noqa: BLE001
            cur = None
            print(f"  (could not read current angle: {exc})")
        print(f"  current readback: {cur}")
        try:
            print(f"  -> {args.low}°")
            bot.set_uart_servo_angle(sid, args.low, args.hold_ms)
            time.sleep(hold_s)
            print(f"  -> {args.high}°")
            bot.set_uart_servo_angle(sid, args.high, args.hold_ms)
            time.sleep(hold_s)
            print("  -> 90° (park)")
            bot.set_uart_servo_angle(sid, 90, args.hold_ms)
            time.sleep(hold_s)
        except Exception as exc:  # noqa: BLE001
            print(f"  error during sweep: {exc}")
        print(f"  observed: ____________ (note which joint moved + direction)")

    print("\nSweep done. Fill in src/yahboom_rosmaster/yahboom_rosmaster_hw_bridge/")
    print("config/servo_map.yaml with the recorded ids + zero_offset_deg, then")
    print("set invert: true on any joint that moved opposite the URDF sign convention.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
