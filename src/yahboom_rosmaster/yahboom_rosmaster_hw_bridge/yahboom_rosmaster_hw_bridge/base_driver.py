"""Mecanum base driver for the Yahboom ROSMaster X3 Plus.

Phase 1 (live serial). Owns /dev/myserial on the host — the other
bridge nodes (joint_state_publisher, arm/gripper action servers)
remain in stub mode through Phase 1 because only one process can
hold the port open at a time. Phase 2 will consolidate everything
into a single bridge node.

Behavior on real hardware:
- /cmd_vel (Twist) → Rosmaster_Lib.set_car_motion(vx, vy, vz_angular)
- /odom (Odometry) ← get_motion_data() integrated to (x, y, theta)
- /imu/data_raw (Imu) ← get_imu_attitude_data() + accel + gyro
- odom → base_footprint TF (optional, on by default for hardware)

Cleanup: on shutdown, send set_car_motion(0, 0, 0) so the chassis
doesn't roll away if a controlling node dies.
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from ._rosmaster_lib import acquire_driver


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class BaseDriver(Node):
    def __init__(self) -> None:
        super().__init__("base_driver")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("car_type", 1)
        self.declare_parameter("serial_delay", 0.002)
        self.declare_parameter("serial_debug", False)
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")

        self._driver = acquire_driver(
            port=self.get_parameter("serial_port").value,
            car_type=int(self.get_parameter("car_type").value),
            delay=float(self.get_parameter("serial_delay").value),
            debug=bool(self.get_parameter("serial_debug").value),
        )

        if self._driver is not None:
            try:
                fw = self._driver.get_version()
                self.get_logger().info(f"[base] STM32 firmware version: {fw}")
                if fw is None or fw <= 0:
                    self.get_logger().warn(
                        "[base] firmware version not yet reported (<=0); "
                        "receive thread may still be syncing"
                    )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"[base] get_version raised: {exc}")

        # Pose integration state (world-frame, accumulated from body-frame velocity).
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_publish_time = self.get_clock().now()

        self._cmd_sub = self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self._on_cmd_vel, 10
        )
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 10
        )
        self._imu_pub = self.create_publisher(
            Imu, self.get_parameter("imu_topic").value, 10
        )
        self._tf_broadcaster: Optional[TransformBroadcaster] = (
            TransformBroadcaster(self)
            if bool(self.get_parameter("publish_odom_tf").value)
            else None
        )
        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(period, self._publish_state)

        mode = "live" if self._driver is not None else "stub"
        self.get_logger().info(f"[base] up in {mode} mode")

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._driver is None:
            self.get_logger().debug(
                f"[base] stub cmd_vel: lin=({msg.linear.x:.2f},{msg.linear.y:.2f}) "
                f"ang={msg.angular.z:.2f}"
            )
            return
        try:
            self._driver.set_car_motion(
                float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)
            )
        except Exception as exc:  # noqa: BLE001 — third-party lib may raise anything
            self.get_logger().error(f"[base] set_car_motion failed: {exc}")

    def _publish_state(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_publish_time).nanoseconds * 1e-9
        self._last_publish_time = now
        if dt <= 0.0 or dt > 1.0:
            # Skip the first tick (dt=0) and absorb pathological hiccups (>1 s)
            # so a single stalled callback doesn't fling the pose integrator.
            dt = 0.0

        if self._driver is None:
            vx = vy = vth = 0.0
            roll = pitch = yaw = 0.0
            ax = ay = az = 0.0
            gx = gy = gz = 0.0
        else:
            try:
                vx, vy, vth = self._driver.get_motion_data()
                roll, pitch, yaw = self._driver.get_imu_attitude_data(ToAngle=False)
                ax, ay, az = self._driver.get_accelerometer_data()
                gx, gy, gz = self._driver.get_gyroscope_data()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"[base] sensor read failed: {exc}")
                return

        # Integrate body-frame velocity into world frame.
        # Use mid-step heading (theta + 0.5 * vth * dt) for a slight accuracy bump
        # over plain Euler when the robot is rotating.
        if dt > 0.0:
            mid_theta = self._theta + 0.5 * vth * dt
            cos_t = math.cos(mid_theta)
            sin_t = math.sin(mid_theta)
            self._x += (vx * cos_t - vy * sin_t) * dt
            self._y += (vx * sin_t + vy * cos_t) * dt
            self._theta += vth * dt

        stamp = now.to_msg()
        odom_frame = self.get_parameter("odom_frame").value
        base_frame = self.get_parameter("base_frame").value
        imu_frame = self.get_parameter("imu_frame").value

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = _rpy_to_quaternion(0.0, 0.0, self._theta)
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.angular.z = float(vth)
        self._odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = imu_frame
        imu.orientation = _rpy_to_quaternion(float(roll), float(pitch), float(yaw))
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        self._imu_pub.publish(imu)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = odom_frame
            tf.child_frame_id = base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation = _rpy_to_quaternion(0.0, 0.0, self._theta)
            self._tf_broadcaster.sendTransform(tf)

    def stop_motors(self) -> None:
        """Best-effort stop. Safe to call multiple times."""
        if self._driver is None:
            return
        try:
            self._driver.set_car_motion(0.0, 0.0, 0.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"[base] stop_motors failed: {exc}")

    def destroy_node(self) -> bool:  # type: ignore[override]
        self.stop_motors()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
