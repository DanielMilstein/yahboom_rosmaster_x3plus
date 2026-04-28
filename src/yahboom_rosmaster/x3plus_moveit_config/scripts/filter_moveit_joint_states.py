#!/usr/bin/env python3

from sensor_msgs.msg import JointState

import rclpy
from rclpy.node import Node


MOVEIT_JOINTS = {
    "arm_joint1",
    "arm_joint2",
    "arm_joint3",
    "arm_joint4",
    "arm_joint5",
    "grip_joint",
}


class MoveItJointStateFilter(Node):
    def __init__(self):
        super().__init__("moveit_joint_state_filter")
        self.publisher = self.create_publisher(JointState, "/moveit_joint_states", 10)
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.filter_joint_states, 10
        )

    def filter_joint_states(self, msg):
        filtered = JointState()
        filtered.header = msg.header

        for index, name in enumerate(msg.name):
            if name not in MOVEIT_JOINTS:
                continue

            filtered.name.append(name)
            if index < len(msg.position):
                filtered.position.append(msg.position[index])
            if index < len(msg.velocity):
                filtered.velocity.append(msg.velocity[index])
            if index < len(msg.effort):
                filtered.effort.append(msg.effort[index])

        if filtered.name:
            self.publisher.publish(filtered)


def main():
    rclpy.init()
    node = MoveItJointStateFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
