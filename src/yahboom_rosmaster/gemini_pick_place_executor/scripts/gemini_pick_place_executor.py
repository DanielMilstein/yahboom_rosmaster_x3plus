#!/usr/bin/env python3

import json
import math
import threading
import time
from copy import deepcopy

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from yahboom_rosmaster_msgs.srv import GeminiPickPlace, GeminiVerifyPick


def normalized_point_to_pixel(point, width, height):
    y = float(point[0])
    x = float(point[1])
    u = int(round(x * (width - 1)))
    v = int(round(y * (height - 1)))
    u = max(0, min(width - 1, u))
    v = max(0, min(height - 1, v))
    return u, v


def normalized_box_point(box, y_fraction, x_fraction):
    ymin, xmin, ymax, xmax = [float(value) for value in box]
    y = ymin + (ymax - ymin) * float(y_fraction)
    x = xmin + (xmax - xmin) * float(x_fraction)
    y = max(0.0, min(1.0, y))
    x = max(0.0, min(1.0, x))
    return [y, x]


def make_color(red, green, blue, alpha=1.0):
    color = ColorRGBA()
    color.r = red
    color.g = green
    color.b = blue
    color.a = alpha
    return color


def top_down_quaternion(yaw=0.0):
    """`R_x(π)` + yaw via the axis-tilt parameterization. Empirically this is
    what makes the gripper physically point DOWN at the can on this URDF — the
    arm_link5 FK at joints-zero looks identity, but the gripper assembly
    downstream of `grip_joint` (rpy=(0,-π/2,0) + the rlink/llink chain)
    effectively flips the gripper's "down" axis. Returns (qx, qy, qz, qw)."""
    half = 0.5 * float(yaw)
    return (math.cos(half), math.sin(half), 0.0, 0.0)


def rotate_vector_by_quat(v, qx, qy, qz, qw):
    """Apply rotation R(Q) to a 3-vector v. Returns (rx, ry, rz)."""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    rx = (1.0 - 2.0 * (qy * qy + qz * qz)) * x \
        + 2.0 * (qx * qy - qz * qw) * y \
        + 2.0 * (qx * qz + qy * qw) * z
    ry = 2.0 * (qx * qy + qz * qw) * x \
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * y \
        + 2.0 * (qy * qz - qx * qw) * z
    rz = 2.0 * (qx * qz - qy * qw) * x \
        + 2.0 * (qy * qz + qx * qw) * y \
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * z
    return rx, ry, rz


# Gripper calibration: object width -> servo angle (per hardware datasheet).
# The Gazebo/URDF grip_joint runs from 0 rad (closed, servo=180 deg) to
# -1.54 rad (max open, servo=0 deg); we assume a linear servo-to-grip_joint
# mapping (TODO: verify against hardware interface if behaviour mismatches).
SERVO_DEG_AT_CLOSED = 180.0
GRIP_JOINT_AT_OPEN = -1.54


def servo_deg_to_grip_joint(servo_deg):
    return GRIP_JOINT_AT_OPEN * (SERVO_DEG_AT_CLOSED - float(servo_deg)) / SERVO_DEG_AT_CLOSED


# (object_width_m, servo_deg) — measured calibration.
GRIP_CALIBRATION_M_DEG = [
    (0.000, 180.0),
    (0.005, 176.0),
    (0.010, 168.0),
    (0.015, 160.0),
    (0.020, 152.0),
    (0.025, 143.0),
    (0.030, 134.0),
    (0.035, 125.0),
    (0.040, 115.0),
    (0.045, 105.0),
    (0.050, 95.0),
    (0.055, 80.0),
    (0.060, 57.0),
]


def width_to_grip_joint_rad(width_m):
    table = GRIP_CALIBRATION_M_DEG
    if width_m <= table[0][0]:
        return servo_deg_to_grip_joint(table[0][1])
    if width_m >= table[-1][0]:
        return servo_deg_to_grip_joint(table[-1][1])
    for i in range(1, len(table)):
        w0, s0 = table[i - 1]
        w1, s1 = table[i]
        if w0 <= width_m <= w1:
            t = (width_m - w0) / (w1 - w0)
            servo = s0 + t * (s1 - s0)
            return servo_deg_to_grip_joint(servo)
    return servo_deg_to_grip_joint(table[-1][1])


class GeminiPickPlaceExecutor(Node):
    def __init__(self):
        super().__init__("gemini_pick_place_executor")

        self.declare_parameter("task", "put the red can in the blue bin")
        self.declare_parameter("image_topic", "/perception_bridge/debug_image")
        self.declare_parameter("gemini_service", "/gemini_pick_place")
        self.declare_parameter("pixel_topic", "/perception_bridge/pixel")
        self.declare_parameter("base_point_topic", "/perception_bridge/selected_point_base")
        self.declare_parameter("marker_topic", "/gemini_pick_place/debug_markers")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("project_timeout_sec", 3.0)
        self.declare_parameter("service_timeout_sec", 10.0)
        self.declare_parameter("pick_lift_m", 0.06)
        self.declare_parameter("place_lift_m", 0.06)
        # Named SRDF pose to retreat to before driving with payload. Default "up"
        # is the all-zeros pose (arm straight up) which keeps the gripper safely
        # above any nearby obstacle while the chassis translates.
        self.declare_parameter("carry_pose_named", "up")
        self.declare_parameter("destination_point_source", "box_bias")
        self.declare_parameter("destination_box_y_fraction", 0.5)
        self.declare_parameter("destination_box_x_fraction", 0.5)
        self.declare_parameter("destination_z_source", "target")
        self.declare_parameter("destination_z_max", 0.25)
        self.declare_parameter("destination_z_fixed", 0.20)
        self.declare_parameter("execute", False)
        self.declare_parameter("arm_group_name", "arm_group")
        self.declare_parameter("gripper_group_name", "grip_group")
        self.declare_parameter("end_effector_link", "arm_link5")
        self.declare_parameter("home_named", "up")
        self.declare_parameter("gripper_open_named", "open")
        self.declare_parameter("gripper_closed_named", "close")
        self.declare_parameter("gripper_tcp_offset_z", 0.02)
        # Fingertip position in arm_link5's local frame. For the Yahboom X3 Plus
        # arm, grip_joint origin in arm_link5 frame is (-0.0035, -0.0126, -0.0685),
        # so the gripper extends along arm_link5's -Z. Fingertip is roughly
        # 12 cm along -Z (gripper + finger length). For each candidate orientation
        # Q, the wrist IK target is computed as
        # `fingertip_target - R(Q) * gripper_tip_offset_xyz`, so the fingertip
        # lands on the perceived point regardless of orientation.
        self.declare_parameter("gripper_tip_offset_xyz", [0.0, 0.0, -0.09])
        self.declare_parameter("use_orientation_constraint", True)
        self.declare_parameter("top_down_yaw", 0.0)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("velocity_scale", 0.3)
        self.declare_parameter("accel_scale", 0.3)
        self.declare_parameter("grasp_clearance_m", 0.005)
        self.declare_parameter("min_grasp_width_m", 0.005)
        self.declare_parameter("max_grasp_width_m", 0.060)
        self.declare_parameter("default_grasp_width_m", 0.045)
        # Vertical extent of the grasped object. Measured from the bbox when possible
        # (project top-edge-center and bottom-edge-center pixels to 3D); falls back
        # to the parameter below. `grasp_z_fraction_from_top` chooses how far down
        # the can the fingertip descends (0.0 = top, 0.5 = mid, 1.0 = bottom).
        self.declare_parameter("object_height_fallback_m", 0.10)
        self.declare_parameter("grasp_z_fraction_from_top", 0.5)
        # Table-height safety floor. The pick fingertip is clamped so it never
        # descends below `table_z + pick_z_safety_m`. With table_z_source set
        # to "perception", we use the z_bottom from measure_object_extent
        # (projection of the bbox-bottom-center pixel — typically the table
        # level at the can's base). With "param", we use table_z_m directly.
        self.declare_parameter("table_z_source", "perception")
        self.declare_parameter("table_z_m", 0.14)
        # Generous default: 1-2 cm for perception z_bottom over-reading the
        # actual table when the bbox is small, ~6 cm for the tip-offset
        # modeling error on this gripper, ~2 cm true clearance.
        self.declare_parameter("pick_z_safety_m", 0.10)
        # Gemini grasp verification after the pick + lift.
        self.declare_parameter("verify_pick_with_gemini", True)
        self.declare_parameter("verify_pick_required", True)
        self.declare_parameter("verify_pick_service", "/gemini_verify_pick")
        # Named arm pose used to "show" the gripper (with whatever's in it) to
        # the camera before verification — see the "show" group_state in the
        # SRDF. Override if you want a different framing.
        self.declare_parameter("verify_show_pose_named", "show")
        # Closed-loop gripper close. Step the commanded grip_joint position
        # toward closed in small increments, read back actual position from
        # /joint_states, and stop on stall (commanded keeps advancing, actual
        # stops following). On contact, apply a tiny extra clamp and hold.
        # If no contact is detected before reaching the close limit or the
        # timeout, return failure → the pick retry loop kicks in.
        self.declare_parameter("close_grip_step_size_rad", 0.05)
        self.declare_parameter("close_grip_settle_time_s", 0.10)
        self.declare_parameter("close_grip_position_error_threshold_rad", 0.05)
        self.declare_parameter("close_grip_movement_threshold_rad", 0.01)
        self.declare_parameter("close_grip_extra_grip_step_rad", 0.06)
        self.declare_parameter("close_grip_hold_position_offset_rad", 0.0)
        self.declare_parameter("close_grip_timeout_s", 30.0)
        self.declare_parameter("joint_states_topic", "/joint_states")
        # On verification failure (or any pick-phase failure), reset the gripper,
        # restow, re-perceive, and try the pick again — up to this many times.
        self.declare_parameter("max_pick_attempts", 3)
        self.declare_parameter("position_tolerance_m", 0.01)
        self.declare_parameter("orientation_xy_tol_rad", 0.1)
        self.declare_parameter("orientation_z_tol_rad", 3.14)
        self.declare_parameter("ik_timeout_sec", 4.0)
        self.declare_parameter("approach_pitch_below_rad", 1.0472)  # ~60 deg below horizontal
        self.declare_parameter("enable_base_drive", True)
        self.declare_parameter("cmd_vel_topic", "mecanum_drive_controller/cmd_vel")
        self.declare_parameter("odom_topic", "mecanum_drive_controller/odom")
        self.declare_parameter("sweet_x", 0.18)
        self.declare_parameter("sweet_y", 0.0)
        self.declare_parameter("reach_window_x_min", 0.10)
        self.declare_parameter("reach_window_x_max", 0.25)
        self.declare_parameter("reach_window_y_half", 0.05)
        self.declare_parameter("drive_axes", "y_only")
        # Default forward bound is 0.0 to avoid driving into a forward obstacle
        # (e.g., the table). Override via launch if the scene allows it.
        self.declare_parameter("base_search_dx_range_m", [-0.30, 0.0])
        self.declare_parameter("base_search_dy_range_m", [-0.30, 0.30])
        self.declare_parameter("base_search_step_m", 0.03)
        self.declare_parameter("ik_search_timeout_sec", 0.3)
        self.declare_parameter("drive_max_lin_speed_mps", 0.10)
        self.declare_parameter("drive_kp", 1.5)
        self.declare_parameter("drive_position_tol_m", 0.01)
        self.declare_parameter("drive_timeout_sec", 15.0)
        self.declare_parameter("drive_settle_sec", 0.3)
        self.declare_parameter("return_after_place", True)
        # drive_mode: "auto" tries closed-loop, falls back to open-loop if no odom; "closed_loop" or "open_loop" force.
        self.declare_parameter("drive_mode", "auto")
        self.declare_parameter("drive_odom_wait_sec", 1.0)
        self.declare_parameter("stow_joint_values", [-1.5708, 1.0, -0.5, 0.0, 0.0])
        self.declare_parameter("stow_for_perception", True)
        self.declare_parameter("restow_after_place", True)
        self.declare_parameter("stow_settle_sec", 0.3)

        self.latest_image = None
        self.latest_base_point = None
        self.base_point_event = threading.Event()
        self.worker_started = False
        self.worker_lock = threading.Lock()
        self.latest_odom = None
        self.odom_event = threading.Event()
        self.latest_joint_state = None

        image_topic = self.get_parameter("image_topic").value
        pixel_topic = self.get_parameter("pixel_topic").value
        base_point_topic = self.get_parameter("base_point_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.base_point_sub = self.create_subscription(
            PointStamped, base_point_topic, self.base_point_callback, 10
        )
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self.joint_state_callback, 10
        )
        self.pixel_pub = self.create_publisher(PointStamped, pixel_topic, 10)
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, marker_qos)
        self.gemini_client = self.create_client(
            GeminiPickPlace, self.get_parameter("gemini_service").value
        )
        self.verify_client = self.create_client(
            GeminiVerifyPick, self.get_parameter("verify_pick_service").value
        )
        self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.start_timer = self.create_timer(0.5, self.maybe_start)

        self.moveit = None
        self.arm_component = None
        self.gripper_component = None
        execute_param = bool(self.get_parameter("execute").value)
        stow_param = bool(self.get_parameter("stow_for_perception").value)
        if execute_param or stow_param:
            self.init_moveit()

        self.get_logger().info(
            f"Waiting for image on {image_topic}; markers will publish on {marker_topic}"
        )

    def init_moveit(self):
        try:
            from moveit.planning import MoveItPy  # noqa: F401
        except ImportError as exc:
            self.get_logger().error(
                f"execute:=true requires moveit_py to be installed: {exc}"
            )
            raise

        from moveit.planning import MoveItPy

        arm_name = str(self.get_parameter("arm_group_name").value)
        gripper_name = str(self.get_parameter("gripper_group_name").value)
        self.get_logger().info(
            f"Initializing MoveItPy (arm='{arm_name}', gripper='{gripper_name}')"
        )
        self.moveit = MoveItPy(node_name="gemini_pick_place_executor")
        self.arm_component = self.moveit.get_planning_component(arm_name)
        self.gripper_component = self.moveit.get_planning_component(gripper_name)

    def image_callback(self, msg):
        self.latest_image = msg

    def base_point_callback(self, msg):
        self.latest_base_point = msg
        self.base_point_event.set()

    def odom_callback(self, msg):
        self.latest_odom = msg
        self.odom_event.set()

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def _get_joint_position(self, name):
        msg = self.latest_joint_state
        if msg is None:
            return None
        try:
            idx = list(msg.name).index(name)
        except ValueError:
            return None
        if idx >= len(msg.position):
            return None
        return float(msg.position[idx])

    def maybe_start(self):
        if not bool(self.get_parameter("auto_start").value):
            return
        if self.worker_started or self.latest_image is None:
            return
        with self.worker_lock:
            if self.worker_started:
                return
            self.worker_started = True
        threading.Thread(target=self.run_once, daemon=True).start()

    def run_once(self):
        execute = bool(self.get_parameter("execute").value)
        drive_enabled = bool(self.get_parameter("enable_base_drive").value)
        stow_for_perception = bool(self.get_parameter("stow_for_perception").value)

        if stow_for_perception and self.moveit is not None:
            if not self.plan_and_execute_stow("00_stow_for_perception"):
                self.get_logger().error("could not stow arm; aborting")
                return

        perceived = self.perceive_targets()
        if perceived is None:
            return
        image, plan, target_point, destination_point = perceived
        self.sanitize_destination_z(target_point, destination_point)

        initial_odom = None
        if execute and drive_enabled:
            pick_lift = float(self.get_parameter("pick_lift_m").value)
            # Test pre-pick (target.z + pick_lift) and the pick height
            # (target.z, no descent below the bias point) so the chosen base
            # offset works for both.
            initial_pick_lifts = [pick_lift, 0.0]
            initial_odom = self.snapshot_odom()  # may be None in open-loop mode
            drive_result = self.drive_to_feasible(
                target_point, initial_pick_lifts, "drive_to_reach_target"
            )
            if not drive_result:
                self.get_logger().error("base drive failed; aborting")
                return
            applied_dx, applied_dy = drive_result
            # Dead-reckon the destination through the same drive delta; re-projecting
            # a 2D bbox from the new vantage gives noisy readings (the bin's centroid
            # can land on the chassis rim), but the rigid base move is exact.
            destination_point.point.x = float(destination_point.point.x) - applied_dx
            destination_point.point.y = float(destination_point.point.y) - applied_dy
            self.get_logger().info(
                f"destination dead-reckoned through drive: "
                f"({destination_point.point.x:.3f},"
                f"{destination_point.point.y:.3f},"
                f"{destination_point.point.z:.3f})"
            )
            perceived = self.perceive_targets()
            if perceived is None:
                return
            # Use refreshed image/plan/target, but DISCARD the re-perceived destination.
            image, plan, target_point, _re_destination = perceived
            self.sanitize_destination_z(target_point, destination_point)

        # Promote target_point.z to the top of the object so pre-pick lift gives
        # genuine clearance above it, and capture the object height for the pick
        # descent. Without this, the perceived z lands somewhere on the can side
        # and the gripper crashes down on top of it.
        z_top, measured_height, measured_z_bottom = self.measure_object_extent(
            plan, image
        )
        if z_top is not None:
            target_point.point.z = z_top
        object_height = (
            measured_height
            if measured_height is not None and measured_height > 0.0
            else float(self.get_parameter("object_height_fallback_m").value)
        )
        # Determine the table_z used as the pick-fingertip safety floor.
        table_z_source = str(self.get_parameter("table_z_source").value).lower()
        if table_z_source == "perception" and measured_z_bottom is not None:
            table_z = float(measured_z_bottom)
        else:
            table_z = float(self.get_parameter("table_z_m").value)
            if table_z_source == "perception":
                self.get_logger().info(
                    f"table_z source=perception unavailable; falling back to "
                    f"param table_z_m={table_z:.3f}"
                )

        # The first drive_to_feasible chose its offset against the *pre-correction*
        # target (raw perception, no z_top fixup). Re-perception shifted xy by a
        # cm or two, and the height correction can raise z by 1-2 cm — enough to
        # flip a barely-feasible IK into infeasible at the arm's reach boundary.
        # Re-verify reachability and nudge the base again if needed.
        if execute and drive_enabled:
            pick_lift = float(self.get_parameter("pick_lift_m").value)
            # Pick at target.z (no descent below it); pre-pick at target.z + lift.
            corrected_pick_lifts = [pick_lift, 0.0]
            drive_result2 = self.drive_to_feasible(
                target_point,
                corrected_pick_lifts,
                "drive_to_reach_target_corrected",
            )
            if not drive_result2:
                self.get_logger().error(
                    "secondary base drive failed after target correction; aborting"
                )
                return
            applied_dx2, applied_dy2 = drive_result2
            if applied_dx2 != 0.0 or applied_dy2 != 0.0:
                destination_point.point.x = float(destination_point.point.x) - applied_dx2
                destination_point.point.y = float(destination_point.point.y) - applied_dy2
                self.get_logger().info(
                    f"destination dead-reckoned through correction drive: "
                    f"({destination_point.point.x:.3f},"
                    f"{destination_point.point.y:.3f},"
                    f"{destination_point.point.z:.3f})"
                )

        self.publish_debug_markers(target_point, destination_point)
        self.log_candidate_summary(plan, target_point, destination_point)

        if execute:
            grasp_width = self.measure_grasp_width(plan, image)
            target_label = str(
                plan.get("target_object", {}).get("label", "object")
            )
            success = self.execute_pick_place(
                target_point,
                destination_point,
                grasp_width,
                object_height,
                table_z,
                target_label,
            )
            if success:
                self.get_logger().info("Pick-and-place sequence completed")
            else:
                self.get_logger().error("Pick-and-place sequence aborted")
            if success and initial_odom is not None and bool(
                self.get_parameter("return_after_place").value
            ):
                self.drive_back_to(initial_odom)
            if success and bool(self.get_parameter("restow_after_place").value):
                self.plan_and_execute_stow("end_stow")

    def perceive_targets(self):
        image = deepcopy(self.latest_image)
        if image is None:
            self.get_logger().warn("No image available")
            return None

        task = self.get_parameter("task").value
        result = self.call_gemini(task, image)
        if result is None:
            return None

        if not result["response"].accepted:
            self.get_logger().warn(
                "Gemini response was valid but not accepted: "
                f"{result['response'].error_message}"
            )
            return None

        try:
            plan = json.loads(result["response"].result_json)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Could not parse accepted Gemini JSON: {exc}")
            return None

        target_pixel = normalized_point_to_pixel(
            plan["target_object"]["point"], image.width, image.height
        )
        destination_point_2d, destination_reason = self.destination_point_2d(plan)
        destination_pixel = normalized_point_to_pixel(
            destination_point_2d, image.width, image.height
        )

        self.get_logger().info(
            f"Projecting target pixel {target_pixel} and destination pixel {destination_pixel} "
            f"({destination_reason})"
        )
        target_point = self.project_pixel("target", target_pixel, image.header.frame_id)
        if target_point is None:
            return None
        destination_point = self.project_pixel(
            "destination", destination_pixel, image.header.frame_id
        )
        if destination_point is None:
            return None

        return image, plan, target_point, destination_point

    def sanitize_destination_z(self, target_point, destination_point):
        src = str(self.get_parameter("destination_z_source").value).lower()
        z_max = float(self.get_parameter("destination_z_max").value)
        original = float(destination_point.point.z)
        if src == "target":
            chosen = float(target_point.point.z)
        elif src == "fixed":
            chosen = float(self.get_parameter("destination_z_fixed").value)
        else:
            chosen = original
        if chosen > z_max:
            self.get_logger().warn(
                f"destination z {chosen:.3f} clamped to {z_max:.3f}"
            )
            chosen = z_max
        if abs(chosen - original) > 1e-4:
            self.get_logger().info(
                f"sanitize_destination_z: {original:.3f} -> {chosen:.3f} (source={src})"
            )
        destination_point.point.z = chosen

    def measure_grasp_width(self, plan, image):
        default_w = float(self.get_parameter("default_grasp_width_m").value)
        box = plan.get("target_object", {}).get("box")
        if not box or len(box) != 4:
            self.get_logger().warn(
                f"No target_object.box from Gemini; using default grasp width {default_w:.3f} m"
            )
            return default_w

        ymin, xmin, ymax, xmax = [float(v) for v in box]
        y_mid = 0.5 * (ymin + ymax)
        left_pixel = normalized_point_to_pixel([y_mid, xmin], image.width, image.height)
        right_pixel = normalized_point_to_pixel([y_mid, xmax], image.width, image.height)
        self.get_logger().info(
            f"Projecting box edges to measure width: left={left_pixel} right={right_pixel}"
        )
        left_pt = self.project_pixel("box_left", left_pixel, image.header.frame_id)
        right_pt = self.project_pixel("box_right", right_pixel, image.header.frame_id)
        if left_pt is None or right_pt is None:
            self.get_logger().warn(
                f"Could not project box edges; using default grasp width {default_w:.3f} m"
            )
            return default_w

        dx = left_pt.point.x - right_pt.point.x
        dy = left_pt.point.y - right_pt.point.y
        dz = left_pt.point.z - right_pt.point.z
        width = math.sqrt(dx * dx + dy * dy + dz * dz)
        self.get_logger().info(f"Measured object grasp width: {width:.3f} m")
        return width

    def measure_object_extent(self, plan, image):
        """Return (z_top, height, z_bottom) by projecting two pixels along
        the bbox vertical centerline. `z_bottom` is the projected z at the
        bbox-bottom pixel — for an object resting on a flat surface this
        approximates the table/surface level at the object's base. Inset 8% inside the bbox so the rays land on the
        object rather than the background — the bbox is often slightly loose,
        and projecting from the very edge can hit the table far behind/below
        the can, giving wildly wrong depths.

        Returns (None, None) if the bbox is missing, projection fails, or the
        result fails sanity (z_top should be above z_bottom by at least 2 cm,
        height in [0.02, 0.30] m, and the two projections close in xy — a can
        is roughly vertical, so they should land at similar x, y).
        """
        box = plan.get("target_object", {}).get("box")
        if not box or len(box) != 4:
            return None, None, None
        ymin, xmin, ymax, xmax = [float(v) for v in box]
        yspan = ymax - ymin
        if yspan <= 0:
            return None, None, None
        inset = 0.08 * yspan
        top_y = ymin + inset
        bottom_y = ymax - inset
        x_mid = 0.5 * (xmin + xmax)
        top_pixel = normalized_point_to_pixel(
            [top_y, x_mid], image.width, image.height
        )
        bottom_pixel = normalized_point_to_pixel(
            [bottom_y, x_mid], image.width, image.height
        )
        top_pt = self.project_pixel("box_top", top_pixel, image.header.frame_id)
        bottom_pt = self.project_pixel("box_bottom", bottom_pixel, image.header.frame_id)
        if top_pt is None or bottom_pt is None:
            return None, None, None
        z_top = float(top_pt.point.z)
        z_bottom = float(bottom_pt.point.z)
        height = z_top - z_bottom
        xy_spread = math.hypot(
            float(top_pt.point.x) - float(bottom_pt.point.x),
            float(top_pt.point.y) - float(bottom_pt.point.y),
        )
        if height < 0.02 or height > 0.30 or xy_spread > 0.08:
            self.get_logger().warn(
                f"measure_object_extent rejected: z_top={z_top:.3f} "
                f"z_bottom={z_bottom:.3f} height={height:.3f}m "
                f"xy_spread={xy_spread:.3f}m; falling back"
            )
            return None, None, None
        self.get_logger().info(
            f"Measured object extent: z_top={z_top:.3f} z_bottom={z_bottom:.3f} "
            f"height={height:.3f}m"
        )
        return z_top, height, z_bottom

    def destination_point_2d(self, plan):
        destination = plan["destination"]
        source = str(self.get_parameter("destination_point_source").value)
        if source == "box_bias" and "box" in destination:
            point = normalized_box_point(
                destination["box"],
                float(self.get_parameter("destination_box_y_fraction").value),
                float(self.get_parameter("destination_box_x_fraction").value),
            )
            return point, (
                "destination box bias "
                f"y={self.get_parameter('destination_box_y_fraction').value} "
                f"x={self.get_parameter('destination_box_x_fraction').value}"
            )
        if source not in ("point", "box_bias"):
            self.get_logger().warn(
                f"Unknown destination_point_source={source!r}; using Gemini destination point"
            )
        return destination["point"], "Gemini destination point"

    def call_gemini(self, task, image):
        timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        if not self.gemini_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("Timed out waiting for /gemini_pick_place")
            return None

        request = GeminiPickPlace.Request()
        request.task = task
        request.image = image
        future = self.gemini_client.call_async(request)

        while rclpy.ok() and not future.done():
            time.sleep(0.05)

        if not future.done() or future.result() is None:
            self.get_logger().error("Gemini service call did not return a response")
            return None

        response = future.result()
        self.get_logger().info(
            "Gemini result: "
            f"success={response.success} accepted={response.accepted} "
            f"confidence={response.confidence:.3f} log_path={response.log_path}"
        )
        if not response.success:
            self.get_logger().error(response.error_message)
            return None
        return {"response": response}

    def call_verify_pick(self, target_label, image):
        timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        if not self.verify_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"Timed out waiting for {self.get_parameter('verify_pick_service').value}"
            )
            return None

        request = GeminiVerifyPick.Request()
        request.target_label = target_label
        request.image = image
        future = self.verify_client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            self.get_logger().error("Verify service call did not return a response")
            return None
        return future.result()

    def run_verify_pick_step(self, target_label):
        if not bool(self.get_parameter("verify_pick_with_gemini").value):
            self.get_logger().info("verify_pick disabled by parameter; skipping")
            return True
        if self.latest_image is None:
            self.get_logger().warn(
                "verify_pick: no image available; skipping verification"
            )
            return True
        image = deepcopy(self.latest_image)
        result = self.call_verify_pick(target_label, image)
        required = bool(self.get_parameter("verify_pick_required").value)
        if result is None or not result.success:
            err = (result.error_message if result is not None else "no response")
            self.get_logger().error(f"verify_pick: service failure ({err})")
            return not required
        self.get_logger().info(
            f"verify_pick: picked_up={result.picked_up} "
            f"confidence={result.confidence:.3f} reason={result.reason!r} "
            f"log_path={result.log_path}"
        )
        if result.picked_up:
            return True
        if required:
            self.get_logger().error(
                f"Pick verification failed for '{target_label}': {result.reason}"
            )
            return False
        self.get_logger().warn(
            f"Pick verification failed for '{target_label}' but "
            "verify_pick_required=false; continuing"
        )
        return True

    def project_pixel(self, name, pixel, frame_id):
        self.base_point_event.clear()
        self.latest_base_point = None

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x = float(pixel[0])
        msg.point.y = float(pixel[1])
        msg.point.z = 0.0
        self.pixel_pub.publish(msg)

        timeout_sec = float(self.get_parameter("project_timeout_sec").value)
        if not self.base_point_event.wait(timeout=timeout_sec):
            self.get_logger().error(
                f"Timed out waiting for projected {name} point from perception bridge"
            )
            return None

        point = deepcopy(self.latest_base_point)
        self.get_logger().info(
            f"{name} base point: frame={point.header.frame_id} "
            f"x={point.point.x:.3f} y={point.point.y:.3f} z={point.point.z:.3f}"
        )
        return point

    def publish_debug_markers(self, target_point, destination_point):
        markers = MarkerArray()
        markers.markers.extend(
            [
                self.make_sphere_marker(1, "target", target_point, make_color(1.0, 0.05, 0.05)),
                self.make_sphere_marker(
                    2, "destination", destination_point, make_color(0.05, 0.35, 1.0)
                ),
                self.make_line_marker(3, target_point, destination_point),
                self.make_lift_marker(
                    4,
                    "target_lift",
                    target_point,
                    float(self.get_parameter("pick_lift_m").value),
                    make_color(1.0, 0.75, 0.05),
                ),
                self.make_lift_marker(
                    5,
                    "place_lift",
                    destination_point,
                    float(self.get_parameter("place_lift_m").value),
                    make_color(0.0, 0.9, 0.7),
                ),
            ]
        )
        self.marker_pub.publish(markers)

    def top_down_pose(self, point, lift_z):
        """Build the desired *fingertip* PoseStamped at the perception point + lift.

        The wrist IK target is computed per-orientation in plan_and_execute_pose
        using gripper_tip_offset_xyz, so we no longer need to add a wrist-z bias
        here. Orientation is a placeholder (top-down yaw) — plan_and_execute_pose
        iterates over candidates.
        """
        yaw = float(self.get_parameter("top_down_yaw").value)
        qx, qy, qz, qw = top_down_quaternion(yaw)
        pose = PoseStamped()
        pose.header.frame_id = point.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point.point.x)
        pose.pose.position.y = float(point.point.y)
        pose.pose.position.z = float(point.point.z) + float(lift_z)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def plan_and_execute(self, component, group_name, label):
        if component is None or self.moveit is None:
            self.get_logger().error(f"[{label}] MoveItPy not initialized")
            return False
        try:
            component.set_workspace(-1.0, -1.0, -0.1, 1.0, 1.0, 2.0)
        except Exception:
            pass
        plan_result = component.plan()
        if not plan_result:
            self.get_logger().error(f"[{label}] planning failed")
            return False
        try:
            trajectory = plan_result.trajectory
        except AttributeError:
            self.get_logger().error(f"[{label}] plan result has no trajectory")
            return False
        self.get_logger().info(f"[{label}] plan ok, executing on '{group_name}'")
        status = self.moveit.execute(group_name, trajectory)
        # moveit_py's execute() is asynchronous and frequently returns with
        # status RUNNING before the controller actually finishes, especially
        # for the longer arm trajectories. The default ExecutionStatus repr
        # also doesn't include a useful keyword. So: be permissive — only
        # return False when the status text explicitly says the trajectory
        # was rejected or aborted by the trajectory-execution-manager (the
        # case we care about — "Invalid Trajectory: start point deviates…"
        # rejections show up as ABORTED). RUNNING / SUCCEEDED / UNKNOWN /
        # default-repr all pass through as success.
        status_text = ""
        try:
            attr = getattr(status, "status", None)
            if attr is not None:
                status_text = str(attr)
            else:
                status_text = str(status)
        except Exception:
            status_text = ""
        ok = True
        for fail_word in ("ABORTED", "FAILED", "REJECT", "INVALID"):
            if fail_word in status_text.upper():
                ok = False
                break
        self.get_logger().info(f"[{label}] execution status: {status}")
        return ok

    def candidate_orientations(self, target_x, target_y):
        """Generate fallback orientations from top-down to tilted, all yawed to face target.
        Baseline (candidate 0) is RPY=(π, 0, yaw) — the empirically-correct
        gripper-points-down on this URDF. Tilts are RPY=(π, -delta, yaw).
        """
        yaw = math.atan2(target_y, target_x)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)

        results = []
        # Original: top-down RPY=(pi, 0, 0) yawed by atan2(y,x)
        # quat = (cos(yaw/2), sin(yaw/2), 0, 0) for fixed RPY=(pi,0,yaw) — same as top_down_quaternion(yaw).
        results.append((cy, sy, 0.0, 0.0))

        # Tilted candidates: gripper Z tilted "below horizontal" by various angles, yawed to face target.
        # Build directly from RPY=(pi, -delta, yaw) in fixed XYZ convention.
        # qw = cos(R/2)cos(P/2)cos(Y/2) + sin(R/2)sin(P/2)sin(Y/2)
        # With R=pi: cos(R/2)=0, sin(R/2)=1
        # qw = sin(P/2)sin(Y/2) = (-sd)*sy = -sd*sy
        # qx = sin(R/2)cos(P/2)cos(Y/2) - cos(R/2)sin(P/2)sin(Y/2) = cd*cy
        # qy = cos(R/2)sin(P/2)cos(Y/2) + sin(R/2)cos(P/2)sin(Y/2) = cd*sy
        # qz = cos(R/2)cos(P/2)sin(Y/2) - sin(R/2)sin(P/2)cos(Y/2) = -(-sd)*cy = sd*cy
        # (with P = -delta, so sin(P/2) = -sd, cos(P/2) = cd)
        for delta_rad in (0.4, 0.8, 1.2, 1.4):  # ~23, 46, 69, 80 deg from top-down
            cd = math.cos(delta_rad / 2.0)
            sd = math.sin(delta_rad / 2.0)
            qw = -sd * sy
            qx = cd * cy
            qy = cd * sy
            qz = sd * cy
            n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if n > 1e-9:
                results.append((qx / n, qy / n, qz / n, qw / n))
        return results

    def state_is_collision_free(self, state):
        """Check `state` against the current planning scene. Returns False if
        the IK plugin (KDL) returned a kinematically-valid but self-colliding
        configuration, which OMPL would later reject as an invalid goal state.
        """
        if self.moveit is None:
            return True
        try:
            psm = self.moveit.get_planning_scene_monitor()
            arm_name = str(self.get_parameter("arm_group_name").value)
            with psm.read_only() as scene:
                colliding = scene.is_state_colliding(
                    robot_state=state,
                    joint_model_group_name=arm_name,
                    verbose=False,
                )
            return not colliding
        except Exception as exc:
            # If the planning scene API isn't available for some reason, fail
            # open: trust the IK and let OMPL reject if needed.
            self.get_logger().warn(
                f"state_is_collision_free: could not query planning scene ({exc}); "
                "assuming collision-free"
            )
            return True

    def plan_and_execute_pose(self, pose_stamped, label):
        if self.arm_component is None or self.moveit is None:
            self.get_logger().error(f"[{label}] MoveItPy not initialized")
            return False
        from moveit.core.robot_state import RobotState
        from geometry_msgs.msg import Pose

        arm_name = str(self.get_parameter("arm_group_name").value)
        ee_link = str(self.get_parameter("end_effector_link").value)
        timeout = float(self.get_parameter("ik_timeout_sec").value)
        robot_model = self.moveit.get_robot_model()

        # KDL only IKs to arm_link5 (Available tip frames: [arm_link5]). So we
        # IK the wrist, but compute the wrist target per orientation so the
        # fingertip lands on the perception point. Wrist target =
        # fingertip - R(Q) * gripper_tip_offset_xyz.
        fx = float(pose_stamped.pose.position.x)
        fy = float(pose_stamped.pose.position.y)
        fz = float(pose_stamped.pose.position.z)
        tip_offset_param = list(self.get_parameter("gripper_tip_offset_xyz").value)
        try:
            tip_offset = [float(v) for v in tip_offset_param]
            if len(tip_offset) != 3:
                raise ValueError(f"expected 3 elements, got {len(tip_offset)}")
        except Exception as exc:
            self.get_logger().warn(
                f"[{label}] invalid gripper_tip_offset_xyz ({exc}); using [0,0,-0.12]"
            )
            tip_offset = [0.0, 0.0, -0.12]

        self.get_logger().info(
            f"[{label}] fingertip target=({fx:.3f},{fy:.3f},{fz:.3f}) "
            f"frame={pose_stamped.header.frame_id} tip_offset_local={tip_offset}"
        )

        candidates = self.candidate_orientations(fx, fy)
        for idx, (qx, qy, qz, qw) in enumerate(candidates):
            ox, oy, oz = rotate_vector_by_quat(tip_offset, qx, qy, qz, qw)
            wx = fx - ox
            wy = fy - oy
            wz = fz - oz
            attempt_pose = Pose()
            attempt_pose.position.x = wx
            attempt_pose.position.y = wy
            attempt_pose.position.z = wz
            attempt_pose.orientation.x = qx
            attempt_pose.orientation.y = qy
            attempt_pose.orientation.z = qz
            attempt_pose.orientation.w = qw

            state = RobotState(robot_model)
            state.update()
            ok = state.set_from_ik(arm_name, attempt_pose, ee_link, timeout)
            if not ok:
                self.get_logger().warn(f"[{label}] IK candidate #{idx} failed")
                continue
            if not self.state_is_collision_free(state):
                self.get_logger().warn(
                    f"[{label}] IK candidate #{idx} self-collides; skipping"
                )
                continue
            self.get_logger().info(
                f"[{label}] IK ok with orientation #{idx} "
                f"quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})"
            )
            self.arm_component.set_start_state_to_current_state()
            self.arm_component.set_goal_state(robot_state=state)
            return self.plan_and_execute(self.arm_component, arm_name, label)

        self.get_logger().warn(
            f"[{label}] all {len(candidates)} candidate orientations failed; "
            "trying position-only fallback"
        )
        position_tol = float(self.get_parameter("position_tolerance_m").value)
        fallback = Constraints()
        fallback.name = "position_only_goal"
        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = ee_link
        pc.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tol]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose_stamped.pose)
        fallback.position_constraints.append(pc)
        self.arm_component.set_start_state_to_current_state()
        self.arm_component.set_goal_state(motion_plan_constraints=[fallback])
        if self.plan_and_execute(self.arm_component, arm_name, label):
            self.get_logger().info(f"[{label}] position-only fallback succeeded")
            return True
        self.get_logger().error(
            f"[{label}] IK + position-only fallback both failed"
        )
        return False

    def build_pose_constraints(self, pose_stamped, ee_link):
        position_tol = float(self.get_parameter("position_tolerance_m").value)
        xy_tol = float(self.get_parameter("orientation_xy_tol_rad").value)
        z_tol = float(self.get_parameter("orientation_z_tol_rad").value)

        constraints = Constraints()
        constraints.name = "gemini_pose_goal"

        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = ee_link
        pc.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tol]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose_stamped.pose)
        constraints.position_constraints.append(pc)

        if bool(self.get_parameter("use_orientation_constraint").value):
            oc = OrientationConstraint()
            oc.header = pose_stamped.header
            oc.link_name = ee_link
            oc.orientation = pose_stamped.pose.orientation
            oc.absolute_x_axis_tolerance = xy_tol
            oc.absolute_y_axis_tolerance = xy_tol
            oc.absolute_z_axis_tolerance = z_tol
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)

        return constraints

    def plan_and_execute_named_arm(self, name, label):
        if self.arm_component is None:
            return False
        arm_name = str(self.get_parameter("arm_group_name").value)
        self.arm_component.set_start_state_to_current_state()
        self.arm_component.set_goal_state(configuration_name=str(name))
        return self.plan_and_execute(self.arm_component, arm_name, label)

    def plan_and_execute_named_gripper(self, name, label):
        if self.gripper_component is None:
            return False
        gripper_name = str(self.get_parameter("gripper_group_name").value)
        self.gripper_component.set_start_state_to_current_state()
        self.gripper_component.set_goal_state(configuration_name=str(name))
        return self.plan_and_execute(self.gripper_component, gripper_name, label)

    def plan_and_execute_stow(self, label):
        if self.arm_component is None or self.moveit is None:
            self.get_logger().warn(f"[{label}] MoveItPy not initialized; cannot stow")
            return False
        from moveit.core.robot_state import RobotState

        values_param = self.get_parameter("stow_joint_values").value
        try:
            values = [float(v) for v in values_param]
        except Exception as exc:
            self.get_logger().error(f"[{label}] invalid stow_joint_values: {exc}")
            return False
        if len(values) != 5:
            self.get_logger().error(
                f"[{label}] stow_joint_values must have 5 entries, got {len(values)}"
            )
            return False
        arm_name = str(self.get_parameter("arm_group_name").value)
        robot_model = self.moveit.get_robot_model()
        state = RobotState(robot_model)
        state.set_joint_group_positions(arm_name, values)
        self.arm_component.set_start_state_to_current_state()
        self.arm_component.set_goal_state(robot_state=state)
        ok = self.plan_and_execute(self.arm_component, arm_name, label)
        if ok:
            settle = float(self.get_parameter("stow_settle_sec").value)
            if settle > 0.0:
                time.sleep(settle)
        return ok

    def target_outside_reach_window(self, point):
        x = float(point.point.x)
        y = float(point.point.y)
        x_min = float(self.get_parameter("reach_window_x_min").value)
        x_max = float(self.get_parameter("reach_window_x_max").value)
        y_half = float(self.get_parameter("reach_window_y_half").value)
        mode = str(self.get_parameter("drive_axes").value).lower()
        check_x = mode != "y_only"
        check_y = mode != "x_only"
        outside = False
        if check_x and (x < x_min or x > x_max):
            outside = True
        if check_y and abs(y) > y_half:
            outside = True
        self.get_logger().info(
            f"reach window check (mode={mode}): target=({x:.3f},{y:.3f}) "
            f"window x=[{x_min:.3f},{x_max:.3f}] |y|<={y_half:.3f} -> "
            f"{'outside' if outside else 'inside'}"
        )
        return outside

    def snapshot_odom(self, wait_sec=None):
        timeout = float(self.get_parameter("drive_odom_wait_sec").value) if wait_sec is None else float(wait_sec)
        if not self.odom_event.wait(timeout=timeout):
            return None
        msg = self.latest_odom
        if msg is None:
            return None
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return {"x": x, "y": y, "yaw": yaw}

    def publish_zero_velocity(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.cmd_vel_pub.publish(msg)

    def find_feasible_drive_for_point(self, point, lift_zs):
        """Search candidate base displacements (dx, dy) for one where arm IK is
        feasible AND collision-free at every fingertip target
        (point.x, point.y, point.z + lift) for each lift in `lift_zs`.

        Accepts a scalar or iterable for backwards compatibility. The same
        orientation index must work at *all* requested lifts, so a single
        approach path can be planned through them.

        Returns (dx, dy, orientation_idx) of the smallest-norm displacement
        that succeeds, or None if no candidate in the search range works.
        """
        try:
            lifts = [float(v) for v in lift_zs]
        except TypeError:
            lifts = [float(lift_zs)]
        if not lifts:
            lifts = [0.0]
        if self.arm_component is None or self.moveit is None:
            return None
        from moveit.core.robot_state import RobotState
        from geometry_msgs.msg import Pose

        arm_name = str(self.get_parameter("arm_group_name").value)
        ee_link = str(self.get_parameter("end_effector_link").value)
        timeout = float(self.get_parameter("ik_search_timeout_sec").value)
        robot_model = self.moveit.get_robot_model()

        tip_offset_param = list(self.get_parameter("gripper_tip_offset_xyz").value)
        try:
            tip_offset = [float(v) for v in tip_offset_param]
            if len(tip_offset) != 3:
                raise ValueError(f"expected 3 elements, got {len(tip_offset)}")
        except Exception as exc:
            self.get_logger().warn(
                f"find_feasible_drive: invalid gripper_tip_offset_xyz ({exc}); "
                "using [0,0,-0.12]"
            )
            tip_offset = [0.0, 0.0, -0.12]

        step = float(self.get_parameter("base_search_step_m").value)
        dx_range = list(self.get_parameter("base_search_dx_range_m").value)
        dy_range = list(self.get_parameter("base_search_dy_range_m").value)
        if len(dx_range) != 2 or len(dy_range) != 2 or step <= 0.0:
            self.get_logger().error(
                "find_feasible_drive: invalid search ranges or step; "
                f"dx_range={dx_range} dy_range={dy_range} step={step}"
            )
            return None

        axes_mode = str(self.get_parameter("drive_axes").value).lower()

        # Build candidate dx, dy lists honoring drive_axes.
        def make_range(lo, hi, step):
            if hi < lo:
                return [0.0]
            n = int(math.floor((hi - lo) / step)) + 1
            return [lo + i * step for i in range(n)]

        if axes_mode == "y_only":
            dx_values = [0.0]
            dy_values = make_range(dy_range[0], dy_range[1], step)
        elif axes_mode == "x_only":
            dx_values = make_range(dx_range[0], dx_range[1], step)
            dy_values = [0.0]
        else:
            dx_values = make_range(dx_range[0], dx_range[1], step)
            dy_values = make_range(dy_range[0], dy_range[1], step)

        # Order candidates by ascending distance from (0, 0).
        candidates = sorted(
            ((dx, dy) for dx in dx_values for dy in dy_values),
            key=lambda d: d[0] * d[0] + d[1] * d[1],
        )

        # Pre-compute orientation list once per (dx, dy) since it depends on (fx_hypo, fy_hypo).
        fx_world = float(point.point.x)
        fy_world = float(point.point.y)
        z_world = float(point.point.z)

        for cand_idx, (dx, dy) in enumerate(candidates):
            fx = fx_world - dx
            fy = fy_world - dy
            orientations = self.candidate_orientations(fx, fy)
            for orient_idx, (qx, qy, qz, qw) in enumerate(orientations):
                ox, oy, oz = rotate_vector_by_quat(tip_offset, qx, qy, qz, qw)
                # Require this orientation to be valid at every requested lift.
                all_lifts_ok = True
                for lift in lifts:
                    fz = z_world + lift
                    wx = fx - ox
                    wy = fy - oy
                    wz = fz - oz
                    attempt_pose = Pose()
                    attempt_pose.position.x = wx
                    attempt_pose.position.y = wy
                    attempt_pose.position.z = wz
                    attempt_pose.orientation.x = qx
                    attempt_pose.orientation.y = qy
                    attempt_pose.orientation.z = qz
                    attempt_pose.orientation.w = qw

                    state = RobotState(robot_model)
                    state.update()
                    if not state.set_from_ik(
                        arm_name, attempt_pose, ee_link, timeout
                    ):
                        all_lifts_ok = False
                        break
                    if not self.state_is_collision_free(state):
                        all_lifts_ok = False
                        break
                if all_lifts_ok:
                    self.get_logger().info(
                        f"find_feasible_drive: feasible at dx={dx:.3f} dy={dy:.3f} "
                        f"orient #{orient_idx} after {cand_idx + 1} candidates "
                        f"(lifts={[round(l, 3) for l in lifts]})"
                    )
                    return dx, dy, orient_idx
        self.get_logger().warn(
            f"find_feasible_drive: no feasible offset in {len(candidates)} candidates "
            f"(point=({fx_world:.3f},{fy_world:.3f},{z_world:.3f}), "
            f"lifts={[round(l, 3) for l in lifts]})"
        )
        return None

    def drive_to_feasible(self, point, lift_z, label):
        # Accept a scalar or an iterable of lifts; the search requires all
        # requested lifts to be feasible & collision-free at the same orientation.
        result = self.find_feasible_drive_for_point(point, lift_z)
        if result is None:
            self.get_logger().error(
                f"[{label}] no feasible base offset found in search range"
            )
            return None
        dx, dy, orient_idx = result
        self.get_logger().info(
            f"[{label}] feasible base offset dx={dx:.3f} dy={dy:.3f} "
            f"(orientation #{orient_idx}); driving"
        )
        if not self.drive_relative_base(dx, dy):
            return None
        # Reflect the base move in the point's coordinates (now in new base frame).
        axes_mode = str(self.get_parameter("drive_axes").value).lower()
        applied_dx = dx if axes_mode in ("xy", "x_only") else 0.0
        applied_dy = dy if axes_mode in ("xy", "y_only") else 0.0
        if applied_dx != 0.0:
            point.point.x = float(point.point.x) - applied_dx
        if applied_dy != 0.0:
            point.point.y = float(point.point.y) - applied_dy
        return applied_dx, applied_dy

    def drive_to_reach(self, target_point):
        sweet_x = float(self.get_parameter("sweet_x").value)
        sweet_y = float(self.get_parameter("sweet_y").value)
        dx = float(target_point.point.x) - sweet_x
        dy = float(target_point.point.y) - sweet_y
        self.get_logger().info(
            f"drive_to_reach: target=({target_point.point.x:.3f},{target_point.point.y:.3f}) "
            f"sweet=({sweet_x:.3f},{sweet_y:.3f}) -> dx={dx:.3f} dy={dy:.3f}"
        )
        return self.drive_relative_base(dx, dy)

    def drive_to_reach_point(self, point, label):
        """Drive base so `point` lands at (sweet_x, sweet_y) in base frame.

        Mutates `point.point.x`/`point.point.y` in place after a successful drive
        so callers see the post-drive coordinates. Other tracked PointStamped
        objects in the same base frame need to be updated by the caller (via
        offset_point_by_drive) if they're still needed downstream.
        """
        if not self.target_outside_reach_window(point):
            self.get_logger().info(f"[{label}] already in reach; skipping drive")
            return True
        sweet_x = float(self.get_parameter("sweet_x").value)
        sweet_y = float(self.get_parameter("sweet_y").value)
        dx = float(point.point.x) - sweet_x
        dy = float(point.point.y) - sweet_y
        self.get_logger().info(
            f"[{label}] drive to reach point=({point.point.x:.3f},{point.point.y:.3f}) "
            f"sweet=({sweet_x:.3f},{sweet_y:.3f}) -> dx={dx:.3f} dy={dy:.3f}"
        )
        ok = self.drive_relative_base(dx, dy)
        if ok:
            axes = str(self.get_parameter("drive_axes").value).lower()
            if axes in ("xy", "x_only"):
                point.point.x = sweet_x
            if axes in ("xy", "y_only"):
                point.point.y = sweet_y
        return ok

    def drive_relative_base(self, dx_base, dy_base):
        axes = str(self.get_parameter("drive_axes").value).lower()
        if axes == "y_only":
            if abs(dx_base) > 1e-6:
                self.get_logger().info(
                    f"drive_relative_base: drive_axes=y_only; dropping dx={dx_base:.3f}"
                )
            dx_base = 0.0
        elif axes == "x_only":
            if abs(dy_base) > 1e-6:
                self.get_logger().info(
                    f"drive_relative_base: drive_axes=x_only; dropping dy={dy_base:.3f}"
                )
            dy_base = 0.0
        mode = str(self.get_parameter("drive_mode").value).lower()
        odom0 = None
        if mode in ("auto", "closed_loop"):
            odom0 = self.snapshot_odom()
            if odom0 is None and mode == "closed_loop":
                self.get_logger().error("drive_relative_base: closed_loop requested but no odom available")
                return False
            if odom0 is None:
                self.get_logger().warn(
                    "drive_relative_base: no odometry, falling back to open-loop timed drive"
                )
                return self.drive_relative_base_open_loop(dx_base, dy_base)
        else:
            return self.drive_relative_base_open_loop(dx_base, dy_base)

        x0 = odom0["x"]
        y0 = odom0["y"]
        yaw0 = odom0["yaw"]

        # Convert base-frame displacement to world-frame goal.
        c0 = math.cos(yaw0)
        s0 = math.sin(yaw0)
        goal_x_w = x0 + (dx_base * c0 - dy_base * s0)
        goal_y_w = y0 + (dx_base * s0 + dy_base * c0)

        kp = float(self.get_parameter("drive_kp").value)
        max_speed = float(self.get_parameter("drive_max_lin_speed_mps").value)
        tol = float(self.get_parameter("drive_position_tol_m").value)
        timeout = float(self.get_parameter("drive_timeout_sec").value)
        period = 0.05  # 20 Hz

        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout
        while rclpy.ok():
            now = self.get_clock().now().nanoseconds / 1e9
            if now > deadline:
                self.get_logger().error("drive_relative_base: timeout")
                self.publish_zero_velocity()
                return False
            cur = self.latest_odom
            if cur is None:
                time.sleep(period)
                continue
            cx = float(cur.pose.pose.position.x)
            cy = float(cur.pose.pose.position.y)
            cqx = float(cur.pose.pose.orientation.x)
            cqy = float(cur.pose.pose.orientation.y)
            cqz = float(cur.pose.pose.orientation.z)
            cqw = float(cur.pose.pose.orientation.w)
            cyaw = math.atan2(
                2.0 * (cqw * cqz + cqx * cqy),
                1.0 - 2.0 * (cqy * cqy + cqz * cqz),
            )

            err_x_w = goal_x_w - cx
            err_y_w = goal_y_w - cy
            err_norm = math.sqrt(err_x_w * err_x_w + err_y_w * err_y_w)
            if err_norm < tol:
                self.publish_zero_velocity()
                settle = float(self.get_parameter("drive_settle_sec").value)
                if settle > 0.0:
                    time.sleep(settle)
                self.get_logger().info(
                    f"drive_relative_base: arrived (err={err_norm:.4f} m)"
                )
                return True

            # Rotate world-frame error into the current base frame.
            cc = math.cos(cyaw)
            ss = math.sin(cyaw)
            err_x_base = cc * err_x_w + ss * err_y_w
            err_y_base = -ss * err_x_w + cc * err_y_w

            vx = max(-max_speed, min(max_speed, kp * err_x_base))
            vy = max(-max_speed, min(max_speed, kp * err_y_base))

            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "base_link"
            twist.twist.linear.x = vx
            twist.twist.linear.y = vy
            self.cmd_vel_pub.publish(twist)
            time.sleep(period)
        self.publish_zero_velocity()
        return False

    def drive_relative_base_open_loop(self, dx_base, dy_base):
        max_speed = float(self.get_parameter("drive_max_lin_speed_mps").value)
        settle = float(self.get_parameter("drive_settle_sec").value)
        dist = math.sqrt(dx_base * dx_base + dy_base * dy_base)
        if dist < 1e-6:
            self.publish_zero_velocity()
            return True
        duration = dist / max_speed
        vx = max_speed * dx_base / dist
        vy = max_speed * dy_base / dist
        self.get_logger().info(
            f"open-loop drive: dx={dx_base:.3f} dy={dy_base:.3f} -> vx={vx:.3f} vy={vy:.3f} for {duration:.2f}s"
        )
        period = 0.05  # 20 Hz
        end_time = self.get_clock().now().nanoseconds / 1e9 + duration
        while rclpy.ok():
            now = self.get_clock().now().nanoseconds / 1e9
            if now >= end_time:
                break
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "base_link"
            twist.twist.linear.x = vx
            twist.twist.linear.y = vy
            self.cmd_vel_pub.publish(twist)
            time.sleep(period)
        self.publish_zero_velocity()
        if settle > 0.0:
            time.sleep(settle)
        self.get_logger().info("open-loop drive: done")
        return True

    def drive_back_to(self, initial_odom):
        if initial_odom is None:
            # Open-loop fallback: we don't know how far we drove, so reverse the most-recent
            # commanded delta is not tracked; just no-op rather than misposition.
            self.get_logger().warn(
                "drive_back_to: no initial odometry recorded; skipping return"
            )
            return False
        cur = self.snapshot_odom()
        if cur is None:
            self.get_logger().warn("drive_back_to: no current odometry; skipping return")
            return False
        # World-frame error is initial - current; convert to base-frame for drive_relative_base.
        err_x_w = initial_odom["x"] - cur["x"]
        err_y_w = initial_odom["y"] - cur["y"]
        cc = math.cos(cur["yaw"])
        ss = math.sin(cur["yaw"])
        dx_base = cc * err_x_w + ss * err_y_w
        dy_base = -ss * err_x_w + cc * err_y_w
        self.get_logger().info(
            f"drive_back_to: returning by base-frame ({dx_base:.3f},{dy_base:.3f})"
        )
        return self.drive_relative_base(dx_base, dy_base)

    def plan_and_execute_gripper_value(self, grip_joint_rad, label):
        if self.gripper_component is None or self.moveit is None:
            return False
        from moveit.core.robot_state import RobotState

        gripper_name = str(self.get_parameter("gripper_group_name").value)
        robot_model = self.moveit.get_robot_model()
        state = RobotState(robot_model)
        target_value = float(grip_joint_rad)

        # grip_group is declared in the SRDF by links, which pulls in grip_joint
        # plus its 5 mimic joints (6 variables total). set_joint_group_positions
        # asserts the input vector matches that count; we want to set the single
        # active joint and let mimics propagate via state.update().
        try:
            state.set_joint_group_active_positions(gripper_name, [target_value])
        except (AttributeError, TypeError) as exc:
            self.get_logger().info(
                f"[{label}] gripper using set_variable_position fallback ({exc})"
            )
            try:
                state.set_variable_position("grip_joint", target_value)
            except Exception as inner:
                self.get_logger().error(
                    f"[{label}] could not set grip_joint via fallback: {inner}"
                )
                return False
        try:
            state.update()
        except Exception:
            pass

        # moveit_py's PlanningSceneMonitor lags badly during back-to-back
        # gripper trajectories — by step 5 of the closed-loop close the PSM
        # was 3 steps behind /joint_states, so set_start_state_to_current_state
        # picked up a stale position and the trajectory-execution-manager
        # rejected the plan with "start point deviates ...". Build the start
        # state directly from our own /joint_states subscription so the plan
        # starts where the joint actually is *right now*.
        js = self.latest_joint_state
        used_fresh_start = False
        if js is not None and js.name:
            start_state = RobotState(robot_model)
            for jname, jpos in zip(js.name, js.position):
                try:
                    start_state.set_variable_position(jname, float(jpos))
                except Exception:
                    pass
            try:
                start_state.update()
            except Exception:
                pass
            try:
                self.gripper_component.set_start_state(robot_state=start_state)
                used_fresh_start = True
            except Exception as exc:
                self.get_logger().warn(
                    f"[{label}] set_start_state(robot_state=...) failed ({exc}); "
                    "falling back to PSM start state"
                )
        if not used_fresh_start:
            self.gripper_component.set_start_state_to_current_state()
        self.gripper_component.set_goal_state(robot_state=state)
        return self.plan_and_execute(self.gripper_component, gripper_name, label)

    def _close_gripper_until_contact(self, label):
        """Step the grip_joint command toward the SRDF "close" limit (0.0 rad)
        in small increments. After each step, read back the actual grip_joint
        position from /joint_states. Stop when actual lags command (the fingers
        hit something) and apply a small extra clamp to firm the hold. Returns
        True on contact, False if no contact found before fully closing or
        timing out — which lets the pick retry loop kick in.

        Stall is detected when EITHER:
          * actual joint delta between steps < movement_threshold (after the
            first step), OR
          * |commanded - actual| > position_error_threshold.
        """
        step_size = float(self.get_parameter("close_grip_step_size_rad").value)
        settle = float(self.get_parameter("close_grip_settle_time_s").value)
        err_thresh = float(
            self.get_parameter("close_grip_position_error_threshold_rad").value
        )
        move_thresh = float(
            self.get_parameter("close_grip_movement_threshold_rad").value
        )
        extra_grip = float(self.get_parameter("close_grip_extra_grip_step_rad").value)
        hold_offset = float(
            self.get_parameter("close_grip_hold_position_offset_rad").value
        )
        timeout = float(self.get_parameter("close_grip_timeout_s").value)
        # SRDF "close" = 0.0; "open" = -1.54. Closing = increasing toward 0.
        close_limit = 0.0

        if step_size <= 0.0:
            self.get_logger().error(
                f"[{label}] close_grip_step_size_rad must be > 0; got {step_size}"
            )
            return False

        start_time = time.time()
        current = self._get_joint_position("grip_joint")
        if current is None:
            self.get_logger().error(
                f"[{label}] no grip_joint position from /joint_states; "
                "cannot run closed-loop close"
            )
            return False

        commanded = current
        prev_actual = current
        contact = False
        stop_reason = "timeout"
        step_count = 0

        while time.time() - start_time < timeout:
            step_count += 1
            commanded = min(close_limit, commanded + step_size)
            if not self.plan_and_execute_gripper_value(
                commanded, f"{label}_step{step_count}"
            ):
                self.get_logger().error(
                    f"[{label}] step {step_count} plan/execute failed at cmd={commanded:.3f}"
                )
                return False
            if settle > 0.0:
                time.sleep(settle)
            actual = self._get_joint_position("grip_joint")
            if actual is None:
                self.get_logger().warn(
                    f"[{label}] no joint state after step {step_count}; "
                    "skipping stall check"
                )
                continue
            movement = abs(actual - prev_actual)
            position_error = commanded - actual  # positive = actual lags command
            self.get_logger().info(
                f"[{label}] step {step_count}: cmd={commanded:.3f} "
                f"actual={actual:.3f} delta={movement:.3f} err={position_error:.3f} "
                f"contact=False"
            )
            stalled_by_movement = step_count > 1 and movement < move_thresh
            stalled_by_error = position_error > err_thresh
            if stalled_by_movement or stalled_by_error:
                contact = True
                stop_reason = (
                    "movement < threshold"
                    if stalled_by_movement
                    else "position error > threshold"
                )
                self.get_logger().info(
                    f"[{label}] contact detected after step {step_count} "
                    f"(reason: {stop_reason})"
                )
                break
            prev_actual = actual
            if commanded >= close_limit - 1e-6:
                stop_reason = "fully closed, no contact"
                break

        if not contact:
            self.get_logger().error(
                f"[{label}] no contact detected (stop_reason={stop_reason!r}, "
                f"steps={step_count}); gripper closed without finding the object"
            )
            return False

        hold = max(commanded - 1.0, min(close_limit, commanded + extra_grip + hold_offset))
        self.get_logger().info(
            f"[{label}] commanding hold position {hold:.3f} "
            f"(commanded={commanded:.3f} + extra_grip={extra_grip:.3f} "
            f"+ hold_offset={hold_offset:.3f})"
        )
        if not self.plan_and_execute_gripper_value(hold, f"{label}_hold"):
            self.get_logger().error(
                f"[{label}] could not apply hold position {hold:.3f}"
            )
            return False
        final_actual = self._get_joint_position("grip_joint")
        self.get_logger().info(
            f"[{label}] final hold position={hold:.3f} "
            f"(actual={final_actual if final_actual is None else f'{final_actual:.3f}'}), "
            f"stop_reason={stop_reason!r}"
        )
        return True

    def grasp_grip_joint(self, measured_width_m):
        clearance = float(self.get_parameter("grasp_clearance_m").value)
        min_w = float(self.get_parameter("min_grasp_width_m").value)
        max_w = float(self.get_parameter("max_grasp_width_m").value)
        target_w = max(min_w, min(max_w, measured_width_m - clearance))
        joint_value = width_to_grip_joint_rad(target_w)
        self.get_logger().info(
            f"Grasp width target {target_w:.3f} m (measured {measured_width_m:.3f} m, "
            f"clearance {clearance:.3f} m) -> grip_joint {joint_value:.3f} rad"
        )
        return joint_value

    def _run_step_sequence(self, steps):
        for name, action in steps:
            self.get_logger().info(f"step '{name}' starting")
            if not action():
                self.get_logger().error(f"step '{name}' failed; aborting sequence")
                return False
        return True

    def _clamp_grasp_descent(self, target_point, object_height_m, table_z):
        """Compute the pick offset relative to target.z. Default is 0 — grasp
        at target.z (Gemini's bias-pixel projection, typically on the can's
        visible surface). The floor clamp can still raise this when target.z
        itself dips below table_z + safety_margin. Returns the descent value
        (positive = above target.z); logs the clamp when triggered.
        """
        grasp_descent = 0.0
        safety = float(self.get_parameter("pick_z_safety_m").value)
        pick_z_min = float(table_z) + safety
        pick_z_unclamped = float(target_point.point.z) + grasp_descent
        if pick_z_unclamped < pick_z_min:
            new_descent = pick_z_min - float(target_point.point.z)
            self.get_logger().info(
                f"pick clamped to table floor: pick_z={pick_z_min:.3f} "
                f"(was {pick_z_unclamped:.3f}, table_z={table_z:.3f}, "
                f"safety={safety:.3f}); descent {grasp_descent:.3f} -> "
                f"{new_descent:.3f}"
            )
            grasp_descent = new_descent
        self.get_logger().info(
            f"grasp at target.z + {grasp_descent:.3f}m "
            f"(object_height={object_height_m:.3f}m)"
        )
        return grasp_descent

    def _run_pick_phase(
        self, target_point, grasp_width_m, object_height_m, table_z, target_label
    ):
        home = str(self.get_parameter("home_named").value)
        open_name = str(self.get_parameter("gripper_open_named").value)
        pick_lift = float(self.get_parameter("pick_lift_m").value)
        verify_show_pose = str(self.get_parameter("verify_show_pose_named").value)
        grasp_descent = self._clamp_grasp_descent(
            target_point, object_height_m, table_z
        )
        grip_value = self.grasp_grip_joint(grasp_width_m)

        steps = [
            ("01_home", lambda: self.plan_and_execute_named_arm(home, "01_home")),
            ("02_open_gripper",
             lambda: self.plan_and_execute_named_gripper(open_name, "02_open_gripper")),
            ("03_pre_pick",
             lambda: self.plan_and_execute_pose(
                 self.top_down_pose(target_point, pick_lift), "03_pre_pick")),
            ("04_pick",
             lambda: self.plan_and_execute_pose(
                 self.top_down_pose(target_point, grasp_descent), "04_pick")),
            ("05_close_gripper",
             lambda: self._close_gripper_until_contact("05_close_gripper")),
            ("06_lift",
             lambda: self.plan_and_execute_pose(
                 self.top_down_pose(target_point, pick_lift), "06_lift")),
            ("06b_verify_show",
             lambda: self.plan_and_execute_named_arm(
                 verify_show_pose, "06b_verify_show")),
            ("06_verify_pick",
             lambda: self.run_verify_pick_step(target_label)),
        ]
        return self._run_step_sequence(steps)

    def _run_place_phase(self, destination_point):
        open_name = str(self.get_parameter("gripper_open_named").value)
        place_lift = float(self.get_parameter("place_lift_m").value)

        steps = [
            ("06a_tuck_for_drive",
             lambda: self.plan_and_execute_named_arm(
                 str(self.get_parameter("carry_pose_named").value),
                 "06a_tuck_for_drive")),
            ("06b_drive_to_destination",
             lambda: self.drive_to_feasible(
                 destination_point,
                 [place_lift, 0.0],
                 "06b_drive_to_destination",
             )),
            ("07_pre_place",
             lambda: self.plan_and_execute_pose(
                 self.top_down_pose(destination_point, place_lift), "07_pre_place")),
            ("08_place",
             lambda: self.plan_and_execute_pose(
                 self.top_down_pose(destination_point, 0.0), "08_place")),
            ("09_open_gripper",
             lambda: self.plan_and_execute_named_gripper(open_name, "09_open_gripper")),
            ("10_retreat",
             lambda: self.plan_and_execute_pose(
                 self.top_down_pose(destination_point, place_lift), "10_retreat")),
        ]
        return self._run_step_sequence(steps)

    def _prepare_for_pick_retry(self, destination_point):
        """Reset state before another pick attempt: open the gripper, stow the
        arm, re-perceive, re-measure the object, and (if drive is enabled)
        nudge the base for the new target. Returns a dict with the refreshed
        (target_point, grasp_width_m, object_height_m, table_z) or None on
        failure. `destination_point` is dead-reckoned through any extra drive.
        """
        open_name = str(self.get_parameter("gripper_open_named").value)
        if not self.plan_and_execute_named_gripper(open_name, "retry_open_gripper"):
            return None
        if not self.plan_and_execute_stow("retry_stow_for_reperception"):
            return None
        perceived = self.perceive_targets()
        if perceived is None:
            return None
        image, plan, target_point, _re_destination = perceived
        # Destination has already been established once and dead-reckoned through
        # the initial drive — keep that, don't trust re-perception of it.
        self.sanitize_destination_z(target_point, destination_point)

        z_top, measured_height, measured_z_bottom = self.measure_object_extent(
            plan, image
        )
        if z_top is not None:
            target_point.point.z = z_top
        object_height = (
            measured_height
            if measured_height is not None and measured_height > 0.0
            else float(self.get_parameter("object_height_fallback_m").value)
        )
        table_z_source = str(self.get_parameter("table_z_source").value).lower()
        if table_z_source == "perception" and measured_z_bottom is not None:
            table_z = float(measured_z_bottom)
        else:
            table_z = float(self.get_parameter("table_z_m").value)

        # Re-verify reachability and nudge base if the new target xy/z slipped
        # outside the previously-blessed feasibility window.
        drive_enabled = bool(self.get_parameter("enable_base_drive").value)
        if drive_enabled:
            pick_lift = float(self.get_parameter("pick_lift_m").value)
            corrected_lifts = [pick_lift, 0.0]
            drive_result = self.drive_to_feasible(
                target_point, corrected_lifts, "retry_drive_correction"
            )
            if not drive_result:
                self.get_logger().error(
                    "retry: drive_to_feasible could not place target in reach"
                )
                return None
            applied_dx, applied_dy = drive_result
            if applied_dx != 0.0 or applied_dy != 0.0:
                destination_point.point.x = (
                    float(destination_point.point.x) - applied_dx
                )
                destination_point.point.y = (
                    float(destination_point.point.y) - applied_dy
                )
                self.get_logger().info(
                    f"retry: destination dead-reckoned through correction drive: "
                    f"({destination_point.point.x:.3f},"
                    f"{destination_point.point.y:.3f},"
                    f"{destination_point.point.z:.3f})"
                )
        grasp_width = self.measure_grasp_width(plan, image)
        target_label = str(plan.get("target_object", {}).get("label", "object"))
        return {
            "target_point": target_point,
            "grasp_width_m": grasp_width,
            "object_height_m": object_height,
            "table_z": table_z,
            "target_label": target_label,
        }

    def execute_pick_place(
        self,
        target_point,
        destination_point,
        grasp_width_m,
        object_height_m,
        table_z,
        target_label,
    ):
        max_attempts = max(1, int(self.get_parameter("max_pick_attempts").value))
        pick_state = {
            "target_point": target_point,
            "grasp_width_m": grasp_width_m,
            "object_height_m": object_height_m,
            "table_z": table_z,
            "target_label": target_label,
        }
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"Pick attempt {attempt}/{max_attempts}")
            ok = self._run_pick_phase(
                pick_state["target_point"],
                pick_state["grasp_width_m"],
                pick_state["object_height_m"],
                pick_state["table_z"],
                pick_state["target_label"],
            )
            if ok:
                break
            if attempt >= max_attempts:
                self.get_logger().error(
                    f"All {max_attempts} pick attempts failed; aborting sequence"
                )
                return False
            self.get_logger().warn(
                f"Pick attempt {attempt}/{max_attempts} failed; "
                "resetting and retrying"
            )
            refreshed = self._prepare_for_pick_retry(destination_point)
            if refreshed is None:
                self.get_logger().error(
                    "Could not prepare for pick retry; aborting sequence"
                )
                return False
            pick_state = refreshed

        return self._run_place_phase(destination_point)

    def make_sphere_marker(self, marker_id, namespace, point, color):
        marker = Marker()
        marker.header.frame_id = point.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color = color
        return marker

    def make_line_marker(self, marker_id, target_point, destination_point):
        marker = Marker()
        marker.header.frame_id = target_point.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place_line"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color = make_color(0.9, 0.9, 0.9)
        marker.points.append(target_point.point)
        marker.points.append(destination_point.point)
        return marker

    def make_lift_marker(self, marker_id, namespace, point, lift_m, color):
        marker = Marker()
        marker.header.frame_id = point.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.scale.y = 0.035
        marker.scale.z = 0.05
        marker.color = color

        start = deepcopy(point.point)
        end = deepcopy(point.point)
        end.z += lift_m
        marker.points.append(start)
        marker.points.append(end)
        return marker

    def log_candidate_summary(self, plan, target_point, destination_point):
        pick_lift = float(self.get_parameter("pick_lift_m").value)
        place_lift = float(self.get_parameter("place_lift_m").value)
        self.get_logger().info(
            "Debug pick/place candidates only. "
            f"target={plan['target_object']['label']} "
            f"at ({target_point.point.x:.3f}, {target_point.point.y:.3f}, {target_point.point.z:.3f}), "
            f"destination={plan['destination']['label']} "
            f"at ({destination_point.point.x:.3f}, {destination_point.point.y:.3f}, {destination_point.point.z:.3f}), "
            f"pick_lift={pick_lift:.3f}m place_lift={place_lift:.3f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GeminiPickPlaceExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
