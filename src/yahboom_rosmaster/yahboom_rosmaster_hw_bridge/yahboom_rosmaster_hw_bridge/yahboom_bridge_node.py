"""Consolidated single-process bridge for the Yahboom ROSMaster X3 Plus.

Holds the one Rosmaster_Lib serial connection on /dev/myserial and
exposes:

- Base teleop:        /cmd_vel (Twist) + /cmd_vel_stamped (TwistStamped) -> set_car_motion()
- Odom + IMU:         /odom, /imu/data_raw, optional odom->base_footprint TF
- Joint states:       /joint_states at 15 Hz (get_uart_servo_angle per joint)
- Arm trajectory:     /arm_controller/follow_joint_trajectory action server
- Gripper trajectory: /gripper_controller/follow_joint_trajectory action server

Arm + gripper execution are gated by enable_arm_execution /
enable_gripper_execution launch params (default false) so the node
can be brought up safely before servo_map.yaml is calibrated.

Replaces the four Phase 1 stub nodes — Rosmaster_Lib can only have
one process holding the serial port at a time, so consolidation is
the only correct shape.
"""
from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from ._rosmaster_lib import acquire_driver


# ----------------------- helpers -----------------------


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


class JointMap:
    """Loads servo_map.yaml and maps between URDF joint radians and servo degrees.

    Schema (per joint): {id: int, open_rad: float, close_rad: float, invert: bool}.
    Linear map: joint=open_rad at servo 0 deg, joint=close_rad at servo 180 deg.
    Invert flips the direction so an "open" servo angle maps to the high joint value.
    """

    def __init__(self, path: str) -> None:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        self._joints: Dict[str, dict] = data.get("joints", {})
        self.arm_joints: List[str] = [j for j in self._joints if j.startswith("arm_")]
        self.gripper_joints: List[str] = [j for j in self._joints if not j.startswith("arm_")]
        self.all_joints: List[str] = list(self._joints.keys())

    def servo_id(self, joint_name: str) -> int:
        return int(self._joints[joint_name]["id"])

    def _endpoints(self, joint_name: str) -> tuple[float, float]:
        j = self._joints[joint_name]
        lo, hi = float(j["open_rad"]), float(j["close_rad"])
        if j.get("invert", False):
            lo, hi = hi, lo
        return lo, hi

    def rad_to_deg(self, joint_name: str, rad: float) -> float:
        lo, hi = self._endpoints(joint_name)
        if hi == lo:
            return 90.0
        deg = (rad - lo) / (hi - lo) * 180.0
        return max(0.0, min(180.0, deg))

    def deg_to_rad(self, joint_name: str, deg: float) -> float:
        lo, hi = self._endpoints(joint_name)
        deg = max(0.0, min(180.0, deg))
        return lo + (hi - lo) * (deg / 180.0)


# ----------------------- bridge node -----------------------


class YahboomBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("yahboom_bridge_node")

        # Serial / driver params.
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("car_type", 1)
        self.declare_parameter("serial_delay", 0.002)
        self.declare_parameter("serial_debug", False)

        # Base. Plain Twist on cmd_vel_topic (teleop_twist_keyboard et al.);
        # TwistStamped on cmd_vel_stamped_topic (the pick-place executor).
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_stamped_topic", "/cmd_vel_stamped")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("base_publish_rate_hz", 30.0)
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")

        # Joints.
        self.declare_parameter("servo_map_path", "")
        self.declare_parameter("joint_state_rate_hz", 15.0)
        self.declare_parameter("joint_state_topic", "/joint_states")

        # Arm + gripper safety gates.
        self.declare_parameter("enable_arm_execution", False)
        self.declare_parameter("enable_gripper_execution", False)
        self.declare_parameter("arm_action_name", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter(
            "gripper_action_name", "/gripper_controller/follow_joint_trajectory"
        )
        self.declare_parameter("trajectory_min_run_ms", 50)

        # Load joint map.
        servo_map_path = str(self.get_parameter("servo_map_path").value)
        if not servo_map_path:
            self.get_logger().error(
                "servo_map_path parameter is empty; pass via launch arg"
            )
            self._joint_map = JointMap.__new__(JointMap)
            self._joint_map._joints = {}
            self._joint_map.arm_joints = []
            self._joint_map.gripper_joints = []
            self._joint_map.all_joints = []
        else:
            self._joint_map = JointMap(servo_map_path)
            self.get_logger().info(
                f"[joint_map] loaded {len(self._joint_map.all_joints)} joints "
                f"from {servo_map_path}"
            )

        # Acquire serial driver.
        self._driver = acquire_driver(
            port=self.get_parameter("serial_port").value,
            car_type=int(self.get_parameter("car_type").value),
            delay=float(self.get_parameter("serial_delay").value),
            debug=bool(self.get_parameter("serial_debug").value),
        )
        self._serial_lock = threading.Lock()
        if self._driver is not None:
            try:
                fw = self._driver.get_version()
                self.get_logger().info(f"[bridge] STM32 firmware version: {fw}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"[bridge] get_version raised: {exc}")

        # Callback groups: base + joint state on one MutEx, action servers reentrant
        # so they can be preempted / run alongside the readback timer.
        self._cb_base = MutuallyExclusiveCallbackGroup()
        self._cb_actions = ReentrantCallbackGroup()

        self._setup_base()
        self._setup_joint_state()
        self._setup_arm_action()
        self._setup_gripper_action()

        mode = "live" if self._driver is not None else "stub"
        self.get_logger().info(f"[bridge] up in {mode} mode")

    # ---------- base subsystem ----------

    def _setup_base(self) -> None:
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_base_time = self.get_clock().now()

        self._cmd_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self._on_cmd_vel,
            10,
            callback_group=self._cb_base,
        )
        self._cmd_stamped_sub = self.create_subscription(
            TwistStamped,
            self.get_parameter("cmd_vel_stamped_topic").value,
            self._on_cmd_vel_stamped,
            10,
            callback_group=self._cb_base,
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
        period = 1.0 / float(self.get_parameter("base_publish_rate_hz").value)
        self._base_timer = self.create_timer(
            period, self._publish_base_state, callback_group=self._cb_base
        )

    def _on_cmd_vel_stamped(self, msg: TwistStamped) -> None:
        self._on_cmd_vel(msg.twist)

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._driver is None:
            return
        try:
            with self._serial_lock:
                self._driver.set_car_motion(
                    float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[base] set_car_motion failed: {exc}")

    def _publish_base_state(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_base_time).nanoseconds * 1e-9
        self._last_base_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0

        if self._driver is None:
            vx = vy = vth = 0.0
            roll = pitch = yaw = 0.0
            ax = ay = az = 0.0
            gx = gy = gz = 0.0
        else:
            try:
                with self._serial_lock:
                    vx, vy, vth = self._driver.get_motion_data()
                    roll, pitch, yaw = self._driver.get_imu_attitude_data(ToAngle=False)
                    ax, ay, az = self._driver.get_accelerometer_data()
                    gx, gy, gz = self._driver.get_gyroscope_data()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"[base] sensor read failed: {exc}")
                return

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
        if self._driver is None:
            return
        try:
            with self._serial_lock:
                self._driver.set_car_motion(0.0, 0.0, 0.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"[base] stop_motors failed: {exc}")

    # ---------- joint state subsystem ----------

    def _setup_joint_state(self) -> None:
        self._joint_state_pub = self.create_publisher(
            JointState, self.get_parameter("joint_state_topic").value, 10
        )
        # Initialize last-known positions so the publisher has something
        # to send while serial reads are still warming up.
        self._last_positions: Dict[str, float] = {
            j: 0.0 for j in self._joint_map.all_joints
        }
        rate = float(self.get_parameter("joint_state_rate_hz").value)
        if rate > 0.0:
            period = 1.0 / rate
            self._joint_state_timer = self.create_timer(
                period, self._publish_joint_states, callback_group=self._cb_base
            )

    def _publish_joint_states(self) -> None:
        if not self._joint_map.all_joints:
            return
        if self._driver is not None:
            for jn in self._joint_map.all_joints:
                sid = self._joint_map.servo_id(jn)
                try:
                    with self._serial_lock:
                        deg = self._driver.get_uart_servo_angle(sid)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().debug(
                        f"[joint_state] read servo id={sid} failed: {exc}"
                    )
                    continue
                # Rosmaster_Lib returns -1 / -2 / -3 on read failure; only
                # accept plausibly-valid degree values (0..180 with a touch
                # of slack for calibration jitter).
                if deg is None or deg < -5 or deg > 185:
                    continue
                self._last_positions[jn] = self._joint_map.deg_to_rad(jn, float(deg))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_map.all_joints)
        msg.position = [self._last_positions[j] for j in msg.name]
        self._joint_state_pub.publish(msg)

    # ---------- arm trajectory action ----------

    def _setup_arm_action(self) -> None:
        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.get_parameter("arm_action_name").value,
            execute_callback=self._arm_execute,
            goal_callback=self._arm_goal_callback,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb_actions,
        )
        self.get_logger().info(
            f"[arm] action server on '{self.get_parameter('arm_action_name').value}' "
            f"(enable_arm_execution={bool(self.get_parameter('enable_arm_execution').value)})"
        )

    def _arm_goal_callback(self, _goal_request) -> GoalResponse:
        if not bool(self.get_parameter("enable_arm_execution").value):
            self.get_logger().warn("[arm] rejecting goal: enable_arm_execution=false")
            return GoalResponse.REJECT
        if self._driver is None:
            self.get_logger().warn("[arm] rejecting goal: stub mode")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _arm_execute(self, goal_handle):
        return self._execute_trajectory(
            goal_handle, allowed_joints=self._joint_map.arm_joints, label="arm"
        )

    # ---------- gripper trajectory action ----------

    def _setup_gripper_action(self) -> None:
        self._gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.get_parameter("gripper_action_name").value,
            execute_callback=self._gripper_execute,
            goal_callback=self._gripper_goal_callback,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb_actions,
        )
        self.get_logger().info(
            f"[gripper] action server on '{self.get_parameter('gripper_action_name').value}' "
            f"(enable_gripper_execution={bool(self.get_parameter('enable_gripper_execution').value)})"
        )

    def _gripper_goal_callback(self, _goal_request) -> GoalResponse:
        if not bool(self.get_parameter("enable_gripper_execution").value):
            self.get_logger().warn("[gripper] rejecting goal: enable_gripper_execution=false")
            return GoalResponse.REJECT
        if self._driver is None:
            self.get_logger().warn("[gripper] rejecting goal: stub mode")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _gripper_execute(self, goal_handle):
        return self._execute_trajectory(
            goal_handle, allowed_joints=self._joint_map.gripper_joints, label="gripper"
        )

    # ---------- shared trajectory executor ----------

    def _execute_trajectory(self, goal_handle, allowed_joints: List[str], label: str):
        traj = goal_handle.request.trajectory
        joint_names = list(traj.joint_names)
        result = FollowJointTrajectory.Result()

        # Validate joints are all in our map and belong to this subsystem.
        for jn in joint_names:
            if jn not in allowed_joints:
                self.get_logger().error(
                    f"[{label}] goal contains unknown joint '{jn}'; allowed={allowed_joints}"
                )
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                return result

        sids = [self._joint_map.servo_id(jn) for jn in joint_names]
        min_run_ms = int(self.get_parameter("trajectory_min_run_ms").value)
        last_t = 0.0

        for idx, point in enumerate(traj.points):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f"[{label}] goal canceled at point {idx}")
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            t_target = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            dt = max(min_run_ms / 1000.0, t_target - last_t)
            run_ms = max(min_run_ms, int(dt * 1000))

            with self._serial_lock:
                for sid, jn, rad in zip(sids, joint_names, point.positions):
                    deg = int(round(self._joint_map.rad_to_deg(jn, float(rad))))
                    try:
                        self._driver.set_uart_servo_angle(sid, deg, run_ms)
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().error(
                            f"[{label}] set_uart_servo_angle(id={sid}, "
                            f"deg={deg}, ms={run_ms}) failed: {exc}"
                        )
                        goal_handle.abort()
                        result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        return result

            time.sleep(dt)
            last_t = t_target

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # ---------- shutdown ----------

    def destroy_node(self) -> bool:  # type: ignore[override]
        self.stop_motors()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = YahboomBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
