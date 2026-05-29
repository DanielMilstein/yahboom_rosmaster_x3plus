"""Mecanum base driver: /cmd_vel -> Rosmaster_Lib.set_car_motion(),
/odom and /imu/data_raw published from get_motion_data() + get_imu_attitude().

Phase 0 stub: subscribes /cmd_vel and logs at debug level; publishes
zero odometry and identity-orientation IMU at 30 Hz so downstream tf
listeners don't time out during early integration. Phase 1 wires the
serial calls.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from ._rosmaster_lib import acquire_driver


class BaseDriver(Node):
    def __init__(self) -> None:
        super().__init__("base_driver")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")

        self._driver = acquire_driver(
            self.get_parameter("serial_port").value,
            int(self.get_parameter("serial_baud").value),
        )

        self._cmd_sub = self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self._on_cmd_vel, 10
        )
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 10
        )
        self._imu_pub = self.create_publisher(
            Imu, self.get_parameter("imu_topic").value, 10
        )
        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(period, self._publish_state)

        mode = "stub" if self._driver is None else "live"
        self.get_logger().info(f"[base] up in {mode} mode")

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Phase 1: self._driver.set_car_motion(msg.linear.x, msg.linear.y, msg.angular.z)
        if self._driver is None:
            self.get_logger().debug(
                f"[base] stub cmd_vel: lin=({msg.linear.x:.2f},{msg.linear.y:.2f}) "
                f"ang={msg.angular.z:.2f}"
            )

    def _publish_state(self) -> None:
        # Phase 1: read self._driver.get_motion_data() and get_imu_attitude(); fill messages.
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.get_parameter("odom_frame").value
        odom.child_frame_id = self.get_parameter("base_frame").value
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.get_parameter("imu_frame").value
        imu.orientation.w = 1.0
        self._imu_pub.publish(imu)


def main() -> None:
    rclpy.init()
    node = BaseDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
