"""15 Hz /joint_states publisher.

Phase 1 stub: publishes zero-position joint states for arm + gripper so
that downstream consumers (rviz, perception bridge tf lookups) can start
up without errors. Stays in stub mode through Phase 1 because base_driver
holds /dev/myserial open and Rosmaster_Lib can't be safely opened a
second time. Phase 2 consolidates everything into a single bridge node
that shares one serial connection; at that point this becomes a real
servo readback at 15 Hz.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


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
        # serial_port / serial_baud accepted for parity with other bridge
        # nodes' launch params; unused in Phase 1.
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("serial_baud", 115200)

        self._joint_names = list(self.get_parameter("joint_names").value)
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"[joint_state] up: {len(self._joint_names)} joints @ "
            f"{self.get_parameter('rate_hz').value} Hz (stub — Phase 1)"
        )

    def _tick(self) -> None:
        # Phase 2: replace zeros with self._driver.get_uart_servo_angle(id) per joint,
        # via the consolidated yahboom_bridge_node that owns serial.
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
