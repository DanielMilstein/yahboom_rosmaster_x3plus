#!/usr/bin/env python3
"""Drive the base back to the odometry origin (x=0, y=0, yaw=0).

Convenience tool for aborted hardware runs: kill the executor, leave
yahboom_bridge_node RUNNING (it owns the odom frame — restarting it
re-zeros odometry at the robot's current spot, making this a no-op),
then:

    ros2 run gemini_pick_place_executor return_to_origin.py

Defaults target the hardware bridge topics (/cmd_vel_stamped, /odom).
Any pose works, not just the origin:

    ros2 run gemini_pick_place_executor return_to_origin.py \
        --ros-args -p goal_x:=0.1 -p goal_y:=-0.2 -p goal_yaw:=0.0

Control mirrors the executor's proven drive loops: closed-loop P on
odometry, world-frame error rotated into the live base frame (correct
at any heading), divergence abort if odom feedback is lying, then an
in-place yaw alignment with the mecanum static-friction floor.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


def quat_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class ReturnToOrigin(Node):
    def __init__(self):
        super().__init__("return_to_origin")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_stamped")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("drive_kp", 1.5)
        self.declare_parameter("drive_max_lin_speed_mps", 0.10)
        self.declare_parameter("drive_max_ang_speed_rps", 0.3)
        self.declare_parameter("drive_position_tol_m", 0.01)
        self.declare_parameter("drive_yaw_tol_rad", 0.02)
        self.declare_parameter("drive_timeout_sec", 60.0)
        self.declare_parameter("drive_abort_divergence_m", 0.10)
        self.declare_parameter("odom_wait_sec", 3.0)

        self.latest_odom = None
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_cb,
            10,
        )

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def _p(self, name):
        return float(self.get_parameter(name).value)

    def pose(self):
        msg = self.latest_odom
        if msg is None:
            return None
        p = msg.pose.pose.position
        return (
            float(p.x),
            float(p.y),
            quat_yaw(msg.pose.pose.orientation),
        )

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.cmd_vel_pub.publish(msg)

    def _twist(self, vx=0.0, vy=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    def wait_for_odom(self):
        deadline = time.time() + self._p("odom_wait_sec")
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_odom is not None:
                return True
        return False

    def drive_to_goal_position(self, goal_x, goal_y):
        kp = self._p("drive_kp")
        max_speed = self._p("drive_max_lin_speed_mps")
        tol = self._p("drive_position_tol_m")
        timeout = self._p("drive_timeout_sec")
        divergence = self._p("drive_abort_divergence_m")

        start = self.pose()
        min_err = math.hypot(goal_x - start[0], goal_y - start[1])
        self.get_logger().info(
            f"driving ({start[0]:+.3f},{start[1]:+.3f}) -> "
            f"({goal_x:+.3f},{goal_y:+.3f}), {min_err:.3f} m to go"
        )
        deadline = time.time() + timeout
        while rclpy.ok():
            if time.time() > deadline:
                self.stop()
                self.get_logger().error("position drive: timeout")
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
            cur = self.pose()
            if cur is None:
                continue
            cx, cy, cyaw = cur
            err_x_w = goal_x - cx
            err_y_w = goal_y - cy
            err = math.hypot(err_x_w, err_y_w)
            min_err = min(min_err, err)
            if divergence > 0.0 and err > min_err + divergence:
                self.stop()
                self.get_logger().error(
                    f"position drive: error diverging ({err:.3f} m, best "
                    f"{min_err:.3f} m) — odom feedback inverted/mis-scaled? "
                    "Stopping."
                )
                return False
            if err < tol:
                self.stop()
                self.get_logger().info(f"position: arrived (err={err:.4f} m)")
                return True
            # World-frame error into the current base frame.
            cc = math.cos(cyaw)
            ss = math.sin(cyaw)
            vx = max(-max_speed, min(max_speed, kp * (cc * err_x_w + ss * err_y_w)))
            vy = max(-max_speed, min(max_speed, kp * (-ss * err_x_w + cc * err_y_w)))
            self._twist(vx=vx, vy=vy)
        self.stop()
        return False

    def rotate_to_goal_yaw(self, goal_yaw):
        kp = self._p("drive_kp")
        max_w = self._p("drive_max_ang_speed_rps")
        tol = self._p("drive_yaw_tol_rad")
        timeout = self._p("drive_timeout_sec")

        deadline = time.time() + timeout
        while rclpy.ok():
            if time.time() > deadline:
                self.stop()
                self.get_logger().error("yaw align: timeout")
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
            cur = self.pose()
            if cur is None:
                continue
            err = math.atan2(
                math.sin(goal_yaw - cur[2]), math.cos(goal_yaw - cur[2])
            )
            if abs(err) <= tol:
                self.stop()
                self.get_logger().info(
                    f"yaw: arrived (err={math.degrees(err):+.2f}°)"
                )
                return True
            w = max(-max_w, min(max_w, kp * err))
            # Mecanum static friction stalls tiny angular commands; floor
            # the magnitude so the last fraction of a degree still moves.
            if abs(w) < 0.08:
                w = math.copysign(0.08, w)
            self._twist(wz=w)
        self.stop()
        return False


def main():
    rclpy.init()
    node = ReturnToOrigin()
    ok = False
    try:
        if not node.wait_for_odom():
            node.get_logger().error(
                "no odometry received — is yahboom_bridge_node running? "
                "(NOTE: restarting the bridge re-zeros odom at the robot's "
                "current position, so origin = wherever it restarted)"
            )
        else:
            goal_x = float(node.get_parameter("goal_x").value)
            goal_y = float(node.get_parameter("goal_y").value)
            goal_yaw = float(node.get_parameter("goal_yaw").value)
            ok = node.drive_to_goal_position(goal_x, goal_y)
            # Align heading even after a position timeout — a straight
            # robot in the wrong spot beats a crooked one.
            ok = node.rotate_to_goal_yaw(goal_yaw) and ok
            final = node.pose()
            if final is not None:
                node.get_logger().info(
                    f"final odom pose: x={final[0]:+.3f} y={final[1]:+.3f} "
                    f"yaw={math.degrees(final[2]):+.1f}°"
                )
    except KeyboardInterrupt:
        pass
    finally:
        # Never leave the base coasting.
        try:
            node.stop()
            time.sleep(0.1)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
