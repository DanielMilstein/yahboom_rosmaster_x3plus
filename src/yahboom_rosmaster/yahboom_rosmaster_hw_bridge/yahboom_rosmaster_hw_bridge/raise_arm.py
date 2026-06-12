"""Raise the arm to the SRDF `up` pose (all joints zero) via the bridge.

The arm physically rests folded against the chassis when powered down, and
MoveIt refuses to plan from that in-collision state. This helper sends one
raw FollowJointTrajectory goal straight to the bridge — no collision
checking — to pull the arm up before a pick-place run.

    ros2 run yahboom_rosmaster_hw_bridge raise_arm [--duration 5.0]

Requires the bridge up with enable_arm_execution=true (the goal is
rejected otherwise). WATCH THE ARM: this moves all five joints blindly.
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]


class RaiseArm(Node):
    def __init__(self) -> None:
        super().__init__("raise_arm")

    def run(self, action_name: str, duration_s: float) -> int:
        client = ActionClient(self, FollowJointTrajectory, action_name)
        self.get_logger().info(f"Waiting for action server '{action_name}' ...")
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"No action server on '{action_name}'. Is the bridge up "
                "(hw_bridge.launch.py or hardware_moveit.launch.py)?"
            )
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(ARM_JOINTS)
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int((duration_s % 1.0) * 1e9)
        goal.trajectory.points = [point]

        self.get_logger().warn(
            f"Raising arm to 'up' (all joints 0.0) over {duration_s:.1f}s — WATCH THE ARM."
        )
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "Goal rejected. Most likely enable_arm_execution=false on the "
                "bridge, or the bridge is in stub mode (no serial)."
            )
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_s + 5.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error("Timed out waiting for the trajectory result.")
            return 1
        code = result.result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"Trajectory failed with error_code={code}.")
            return 1
        self.get_logger().info("Arm raised to 'up'. MoveIt can plan from here.")
        return 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5.0, help="motion time in seconds")
    parser.add_argument(
        "--action-name",
        default="/arm_controller/follow_joint_trajectory",
        help="FollowJointTrajectory action server name",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RaiseArm()
    try:
        code = node.run(args.action_name, max(1.0, args.duration))
    except KeyboardInterrupt:
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
