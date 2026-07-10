#!/usr/bin/env python3
"""Top-down viewer for what the lidar actually sees.

Grabs /scan, prints per-sector statistics, and renders a bird's-eye PNG
with the same range gates the executor's scan-match audit uses — so you
can check whether the walls/printer are being picked up, and which points
the audit is actually matching on.

Usage (on the Jetson, with the driver running):
    ros2 run gemini_pick_place_executor lidar_debug.py               # one shot -> /tmp/lidar_view.png
    ros2 run gemini_pick_place_executor lidar_debug.py --watch       # refresh every 2 s
    ros2 run gemini_pick_place_executor lidar_debug.py --out ~/scan.png

Reading the plot: robot at origin, +X forward (red arrow). Blue points are
inside the audit's gates [min_range, max_range]; grey points are seen by
the lidar but IGNORED by the audit. Straight dense lines at the sides =
walls; the blob ahead = printer. If a wall you can see with your eyes has
no points, the lidar isn't returning it (height, material, or config).
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def sector_name(angle):
    if abs(angle) <= 0.52:
        return "front"
    if 0.52 < angle <= 2.09:
        return "left"
    if -2.09 <= angle < -0.52:
        return "right"
    return "rear"


class LidarDebug(Node):
    def __init__(self):
        super().__init__("lidar_debug")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.scan = None
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, qos)

    def cb(self, msg):
        self.scan = msg


def report(scan, gate_min, gate_max, out_path):
    n = len(scan.ranges)
    angles = [scan.angle_min + scan.angle_increment * i for i in range(n)]
    finite = 0
    gated_pts = []
    ignored_pts = []
    sectors = {"front": [], "left": [], "right": [], "rear": []}
    for a, r in zip(angles, scan.ranges):
        if not math.isfinite(r) or r <= 0.0:
            continue
        finite += 1
        x, y = r * math.cos(a), r * math.sin(a)
        if gate_min <= r <= gate_max:
            gated_pts.append((x, y))
            sectors[sector_name(a)].append(r)
        else:
            ignored_pts.append((x, y))

    print(f"frame_id={scan.header.frame_id}  rays={n}  finite={finite} "
          f"({100.0 * finite / max(1, n):.0f}%)  "
          f"in-gate[{gate_min},{gate_max}]={len(gated_pts)}")
    for name in ("front", "left", "right", "rear"):
        rs = sorted(sectors[name])
        if rs:
            print(f"  {name:5s}: {len(rs):4d} pts  min={rs[0]:.2f}  "
                  f"median={rs[len(rs) // 2]:.2f}  max={rs[-1]:.2f}")
        else:
            print(f"  {name:5s}:    0 pts  <-- nothing in gate! "
                  "(wall too far/near, too low/high, or absorbing material)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available — stats only "
              "(sudo apt install python3-matplotlib for the plot)")
        return

    fig, ax = plt.subplots(figsize=(8, 8))
    if ignored_pts:
        ax.scatter(*zip(*ignored_pts), s=3, c="lightgrey",
                   label="seen, outside audit gate")
    if gated_pts:
        ax.scatter(*zip(*gated_pts), s=5, c="tab:blue",
                   label="used by scan-match audit")
    for radius, style in ((gate_min, ":"), (gate_max, "--")):
        circ = plt.Circle((0, 0), radius, fill=False, ls=style, color="k", lw=0.7)
        ax.add_patch(circ)
    ax.arrow(0, 0, 0.15, 0, head_width=0.04, color="red")
    ax.plot(0, 0, "ks", markersize=8)
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_xlabel("x forward [m]")
    ax.set_ylabel("y left [m]")
    ax.set_title(f"lidar top-down view ({len(gated_pts)} gated / "
                 f"{finite} finite points)")
    ax.legend(loc="upper right", fontsize=8)
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"plot written to {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/tmp/lidar_view.png")
    parser.add_argument("--watch", action="store_true",
                        help="refresh every 2 s until Ctrl-C")
    parser.add_argument("--gate-min", type=float, default=0.25,
                        help="audit min range (matches lidar_min_range_m)")
    parser.add_argument("--gate-max", type=float, default=2.5,
                        help="audit max range (matches lidar_max_range_m)")
    args = parser.parse_args()

    rclpy.init()
    node = LidarDebug()
    try:
        while rclpy.ok():
            node.scan = None
            deadline = node.get_clock().now().nanoseconds + int(5e9)
            while node.scan is None:
                rclpy.spin_once(node, timeout_sec=0.2)
                if node.get_clock().now().nanoseconds > deadline:
                    print("no scan received on /scan within 5 s — "
                          "is the driver running (use_lidar:=true)?")
                    return 1
            report(node.scan, args.gate_min, args.gate_max, args.out)
            if not args.watch:
                return 0
            import time
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
