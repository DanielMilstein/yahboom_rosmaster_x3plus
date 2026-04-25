#!/usr/bin/env python3
"""Run a short ROSMASTER X3 Plus arm and gripper demo."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = [
    "arm_joint1",
    "arm_joint2",
    "arm_joint3",
    "arm_joint4",
    "arm_joint5",
]


class X3PlusServoDance(Node):
    """Publish a fixed, conservative arm and gripper motion sequence."""

    def __init__(self):
        super().__init__("x3_plus_servo_dance")
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            "/gripper_controller/commands",
            10,
        )

    def move_arm(self, positions, duration_sec=2.0):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        msg.points = [point]

        self.arm_pub.publish(msg)
        self.get_logger().info(f"Arm target: {positions}")
        time.sleep(duration_sec + 0.3)

    def move_gripper(self, position, wait_sec=0.8):
        msg = Float64MultiArray()
        msg.data = [position]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Gripper target: {position:.2f}")
        time.sleep(wait_sec)

    def run(self):
        self.get_logger().info("Waiting for controller topic discovery...")
        time.sleep(2.0)

        neutral = [0.0, 0.0, 0.0, 0.0, 0.0]
        half_open = -0.7
        open_gripper = -1.2
        closed_gripper = 0.0

        self.move_arm(neutral, 1.5)
        self.move_gripper(half_open)

        self.move_arm([0.65, 0.25, -0.35, 0.25, 0.0], 1.8)
        self.move_arm([-0.65, 0.25, -0.35, 0.25, 0.0], 1.8)
        self.move_arm([0.0, 0.55, -0.75, 0.55, 0.45], 1.8)
        self.move_arm([0.0, 0.25, -0.25, -0.45, -0.35], 1.8)
        self.move_arm([0.45, 0.35, -0.55, 0.35, 0.75], 1.8)
        self.move_arm([-0.45, 0.35, -0.55, 0.35, -0.45], 1.8)

        self.move_gripper(open_gripper)
        self.move_gripper(closed_gripper)
        self.move_gripper(open_gripper)

        self.move_arm(neutral, 2.0)
        self.move_gripper(half_open)
        self.get_logger().info("Servo dance complete.")


def main():
    rclpy.init()
    node = X3PlusServoDance()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
