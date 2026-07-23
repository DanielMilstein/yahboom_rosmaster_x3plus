#!/usr/bin/env python3
"""Interactive arm-pose teaching tool ("teach pendant").

Jog the arm joint by joint while watching the camera feed (e.g.
`ros2 run rqt_image_view rqt_image_view`), then print the captured pose
in ready-to-paste form — as the executor's verify_show_joints_rad launch
arg and as an SRDF <group_state> snippet.

Run it with the hardware bringup up (arm controller active), the
executor NOT running:

    ros2 run gemini_pick_place_executor teach_pose.py

Commands (then Enter):
    2 -0.1        jog arm_joint2 by -0.1 rad
    2 = 0.9       set arm_joint2 to 0.9 rad (absolute)
    all 0 1.05 -0.17 -1.57 0    move all five joints (absolute)
    p             print current pose (launch arg + SRDF snippet)
    r             read current joint positions
    q             quit

Moves are DIRECT (no planning, no collision checking) — jog in small
steps and keep an eye on the arm. Values clamp to +-1.5708 (servo
range). Joints: 1=base yaw, 2=shoulder, 3=elbow, 4=wrist pitch,
5=gripper roll.
"""

import math
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM_JOINTS = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
LIMIT = 1.5708


class TeachPose(Node):
    def __init__(self):
        super().__init__("teach_pose")
        self.declare_parameter(
            "arm_action_topic", "/arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter("move_duration_s", 1.5)
        self._lock = threading.Lock()
        self._positions = {}
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("arm_action_topic").value),
        )

    def _js_cb(self, msg):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._positions[name] = float(pos)

    def joints(self):
        with self._lock:
            if not all(j in self._positions for j in ARM_JOINTS):
                return None
            return [self._positions[j] for j in ARM_JOINTS]

    def send(self, values):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in values]
        dur = float(self.get_parameter("move_duration_s").value)
        pt.time_from_start = Duration(
            sec=int(dur), nanosec=int((dur % 1.0) * 1e9)
        )
        traj.points = [pt]
        goal.trajectory = traj
        if not self._client.wait_for_server(timeout_sec=3.0):
            print("!! arm action server unavailable — is the bringup running?")
            return False
        fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            print("!! trajectory goal rejected")
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=dur + 5.0)
        return True


def fmt(values):
    return ", ".join(f"{v:+.4f}" for v in values)


def print_pose(values):
    vals = ", ".join(f"{v:.4f}" for v in values)
    print(f"\ncurrent pose (actual servo readback):")
    print(f"  {fmt(values)}\n")
    print("executor launch arg:")
    print(f'  verify_show_joints_rad:="[{vals}]"\n')
    print("SRDF group_state snippet (x3plus_moveit_config/config/"
          "rosmaster_x3_plus.srdf):")
    print('  <group_state name="show" group="arm_group">')
    for name, v in zip(ARM_JOINTS, values):
        print(f'      <joint name="{name}" value="{v:.4f}"/>')
    print("  </group_state>\n")


def main():
    rclpy.init()
    node = TeachPose()
    spinner = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spinner.start()

    print(__doc__)
    print("waiting for /joint_states ...")
    import time

    deadline = time.time() + 5.0
    while node.joints() is None and time.time() < deadline:
        time.sleep(0.1)
    cur = node.joints()
    if cur is None:
        print("!! no /joint_states after 5 s — is the bringup running?")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print(f"current: {fmt(cur)}")

    try:
        while True:
            try:
                line = input("teach> ").strip()
            except EOFError:
                break
            if not line:
                continue
            tok = line.split()
            cmd = tok[0].lower()
            if cmd == "q":
                break
            if cmd == "r":
                cur = node.joints()
                print(f"current: {fmt(cur)}")
                continue
            if cmd == "p":
                print_pose(node.joints())
                continue
            if cmd == "all" and len(tok) == 6:
                try:
                    target = [
                        max(-LIMIT, min(LIMIT, float(v))) for v in tok[1:]
                    ]
                except ValueError:
                    print("?? five numbers expected")
                    continue
                print(f"moving to: {fmt(target)}")
                node.send(target)
                continue
            # "<joint> <delta>" or "<joint> = <abs>"
            if cmd in ("1", "2", "3", "4", "5"):
                idx = int(cmd) - 1
                cur = node.joints()
                try:
                    if len(tok) == 3 and tok[1] == "=":
                        val = float(tok[2])
                    elif len(tok) == 2 and tok[1].startswith("="):
                        val = float(tok[1][1:])
                    elif len(tok) == 2:
                        val = cur[idx] + float(tok[1])
                    else:
                        raise ValueError
                except ValueError:
                    print("?? e.g. '2 -0.1' (jog) or '2 = 0.9' (absolute)")
                    continue
                target = list(cur)
                target[idx] = max(-LIMIT, min(LIMIT, val))
                print(
                    f"{ARM_JOINTS[idx]}: {cur[idx]:+.4f} -> {target[idx]:+.4f}"
                )
                node.send(target)
                continue
            print("?? commands: '2 -0.1', '2 = 0.9', "
                  "'all j1 j2 j3 j4 j5', 'p', 'r', 'q'")
    except KeyboardInterrupt:
        pass
    finally:
        cur = node.joints()
        if cur is not None:
            print_pose(cur)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
