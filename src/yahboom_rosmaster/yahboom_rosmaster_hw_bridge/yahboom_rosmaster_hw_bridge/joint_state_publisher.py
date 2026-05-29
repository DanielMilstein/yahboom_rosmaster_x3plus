"""15 Hz /joint_states publisher polled from Rosmaster_Lib.

Phase 0 stub: publishes zero-position joint states for arm + gripper so
that downstream consumers (rviz, perception bridge tf lookups) can start
up without errors during integration testing. Phase 1/2 swap the zeros
for actual servo readbacks.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ._rosmaster_lib import acquire_driver


DEFAULT_JOINTS = (
    "arm_joint1",
    "arm_joint2",
    "arm_joint3",
    "arm_joint4",
    "arm_joint5",
    "grip_joint",
)


class JointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("hw_joint_state_publisher")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("joint_names", list(DEFAULT_JOINTS))
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("serial_baud", 115200)

        self._driver = acquire_driver(
            self.get_parameter("serial_port").value,
            int(self.get_parameter("serial_baud").value),
        )
        self._joint_names = list(self.get_parameter("joint_names").value)
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self._timer = self.create_timer(period, self._tick)

        mode = "stub" if self._driver is None else "live"
        self.get_logger().info(
            f"[joint_state] up: {len(self._joint_names)} joints @ "
            f"{self.get_parameter('rate_hz').value} Hz ({mode})"
        )

    def _tick(self) -> None:
        # Phase 1: replace zeros with self._driver.get_uart_servo_angle(id) per joint.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [0.0] * len(self._joint_names)
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
