#!/usr/bin/env python3
"""Find the correct Yahboom car_type enum for this chassis.

The STM32 uses the enum's wheel geometry BOTH to convert commanded m/s
into wheel speeds AND to convert encoder counts back into the velocity
that get_motion_data() reports. A wrong enum therefore makes commands run
hot/cold AND corrupts odometry feedback (scale and even sign) — observed
on this robot as a 0.08 m/s command driving ~0.8 m/s while odometry
integrated NEGATIVE x.

Run on the Orin with the ROS stack DOWN (single serial owner) and the
robot on the floor with >= 1 m of clear runway:

    python3 probe_car_type.py [--port /dev/ttyUSB2]

For each enum it waits for ENTER, drives forward at a commanded 0.1 m/s
for 1 s while printing get_motion_data(), then stops. Record per enum:

  - physical speed: should be a slow ~0.1 m/s creep (~10 cm covered)
  - direction: straight forward
  - reported vx: should read ~ +0.10 during motion

The WINNING enum satisfies all three. Use it as car_type:= in the
launches (and tell Claude so the defaults get flipped).
"""
import argparse
import sys
import time

try:
    from Rosmaster_Lib import Rosmaster
except ImportError:
    sys.exit("Rosmaster_Lib not found — run this on the robot.")

CANDIDATES = (1, 2, 4, 5, 6)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB2")
    parser.add_argument("--speed", type=float, default=0.1)
    args = parser.parse_args()

    bot = Rosmaster(com=args.port)
    bot.create_receive_threading()
    time.sleep(0.5)
    print(f"firmware: {bot.get_version()}  (must not be -1)")

    for ct in CANDIDATES:
        bot.set_car_type(ct)
        time.sleep(0.3)
        try:
            input(
                f"\n=== car_type={ct}: ENTER to drive fwd at {args.speed} m/s "
                "for 1 s (Ctrl-C to abort) === "
            )
        except KeyboardInterrupt:
            print("\naborted")
            break
        bot.set_car_motion(args.speed, 0.0, 0.0)
        for _ in range(5):
            time.sleep(0.2)
            print(f"  motion_data: {bot.get_motion_data()}")
        bot.set_car_motion(0.0, 0.0, 0.0)
        time.sleep(0.5)
        print(f"  car_type={ct}: note physical speed/direction vs reported vx above")

    bot.set_car_motion(0.0, 0.0, 0.0)
    print("\nDone. Winner = gentle ~0.1 m/s forward AND reported vx ~ +0.10.")


if __name__ == "__main__":
    main()
