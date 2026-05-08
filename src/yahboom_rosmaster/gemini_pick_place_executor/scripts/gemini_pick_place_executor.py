#!/usr/bin/env python3

import json
import math
import threading
import time
from copy import deepcopy

from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from sensor_msgs.msg import Image
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from yahboom_rosmaster_msgs.srv import GeminiPickPlace


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
    half = 0.5 * float(yaw)
    return (math.cos(half), math.sin(half), 0.0, 0.0)


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
        self.declare_parameter("pick_lift_m", 0.04)
        self.declare_parameter("place_lift_m", 0.04)
        self.declare_parameter("destination_point_source", "box_bias")
        self.declare_parameter("destination_box_y_fraction", 0.5)
        self.declare_parameter("destination_box_x_fraction", 0.5)
        self.declare_parameter("execute", False)
        self.declare_parameter("arm_group_name", "arm_group")
        self.declare_parameter("gripper_group_name", "grip_group")
        self.declare_parameter("end_effector_link", "arm_link5")
        self.declare_parameter("home_named", "up")
        self.declare_parameter("gripper_open_named", "open")
        self.declare_parameter("gripper_closed_named", "close")
        self.declare_parameter("gripper_tcp_offset_z", 0.04)
        self.declare_parameter("use_orientation_constraint", True)
        self.declare_parameter("top_down_yaw", 0.0)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("velocity_scale", 0.3)
        self.declare_parameter("accel_scale", 0.3)
        self.declare_parameter("grasp_clearance_m", 0.005)
        self.declare_parameter("min_grasp_width_m", 0.005)
        self.declare_parameter("max_grasp_width_m", 0.060)
        self.declare_parameter("default_grasp_width_m", 0.045)
        self.declare_parameter("position_tolerance_m", 0.01)
        self.declare_parameter("orientation_xy_tol_rad", 0.1)
        self.declare_parameter("orientation_z_tol_rad", 3.14)
        self.declare_parameter("ik_timeout_sec", 2.0)
        self.declare_parameter("approach_pitch_below_rad", 1.0472)  # ~60 deg below horizontal

        self.latest_image = None
        self.latest_base_point = None
        self.base_point_event = threading.Event()
        self.worker_started = False
        self.worker_lock = threading.Lock()

        image_topic = self.get_parameter("image_topic").value
        pixel_topic = self.get_parameter("pixel_topic").value
        base_point_topic = self.get_parameter("base_point_topic").value
        marker_topic = self.get_parameter("marker_topic").value

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.base_point_sub = self.create_subscription(
            PointStamped, base_point_topic, self.base_point_callback, 10
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
        self.start_timer = self.create_timer(0.5, self.maybe_start)

        self.moveit = None
        self.arm_component = None
        self.gripper_component = None
        if bool(self.get_parameter("execute").value):
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
        image = deepcopy(self.latest_image)
        if image is None:
            self.get_logger().warn("No image available")
            return

        task = self.get_parameter("task").value
        result = self.call_gemini(task, image)
        if result is None:
            return

        if not result["response"].accepted:
            self.get_logger().warn(
                "Gemini response was valid but not accepted: "
                f"{result['response'].error_message}"
            )
            return

        try:
            plan = json.loads(result["response"].result_json)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Could not parse accepted Gemini JSON: {exc}")
            return

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
            return
        destination_point = self.project_pixel(
            "destination", destination_pixel, image.header.frame_id
        )
        if destination_point is None:
            return

        self.publish_debug_markers(target_point, destination_point)
        self.log_candidate_summary(plan, target_point, destination_point)

        if bool(self.get_parameter("execute").value):
            grasp_width = self.measure_grasp_width(plan, image)
            success = self.execute_pick_place(target_point, destination_point, grasp_width)
            if success:
                self.get_logger().info("Pick-and-place sequence completed")
            else:
                self.get_logger().error("Pick-and-place sequence aborted")

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
        offset = float(self.get_parameter("gripper_tcp_offset_z").value)
        yaw = float(self.get_parameter("top_down_yaw").value)
        qx, qy, qz, qw = top_down_quaternion(yaw)
        pose = PoseStamped()
        pose.header.frame_id = point.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point.point.x)
        pose.pose.position.y = float(point.point.y)
        pose.pose.position.z = float(point.point.z) + float(lift_z) + offset
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
        self.get_logger().info(f"[{label}] execution status: {status}")
        return True

    def candidate_orientations(self, target_x, target_y):
        """Generate fallback orientations from top-down to tilted, all yawed to face target."""
        yaw = math.atan2(target_y, target_x)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)

        results = []
        # Original: top-down RPY=(pi, 0, 0) yawed by atan2(y,x)
        # quat = (cos(yaw/2), sin(yaw/2), 0, 0) for fixed RPY=(pi,0,yaw) — same as top_down_quaternion(yaw).
        results.append((cy, sy, 0.0, 0.0))

        # Tilted candidates: gripper Z tilted "below horizontal" by various angles, yawed to face target.
        # Construction: q_yaw_around_Z * q_roll_pi_around_X_in_yawed_frame * q_pitch_around_Y
        # We approximate with a sequential body rotation from RPY(pi, -delta, yaw)
        # where delta in (0, pi/2) moves from top-down toward forward-pointing.
        for delta_rad in (0.4, 0.8, 1.2, 1.4):  # ~23, 46, 69, 80 deg from top-down
            cd = math.cos(delta_rad / 2.0)
            sd = math.sin(delta_rad / 2.0)
            # q_top_down = (cy, sy, 0, 0) (after applying roll=pi yaw=yaw)
            # q_pitch around body Y by -delta = (cos(-d/2), 0, sin(-d/2), 0) = (cd, 0, -sd, 0) but
            # acting in the yawed/rolled frame -> need composition.
            # Easier: build directly from RPY=(pi, -delta, yaw) in fixed XYZ convention.
            # qw = cos(R/2)cos(P/2)cos(Y/2) + sin(R/2)sin(P/2)sin(Y/2)
            # With R=pi: cos(R/2)=0, sin(R/2)=1
            # qw = sin(P/2)sin(Y/2) = (-sd)*sy = -sd*sy
            # qx = sin(R/2)cos(P/2)cos(Y/2) - cos(R/2)sin(P/2)sin(Y/2) = cd*cy
            # qy = cos(R/2)sin(P/2)cos(Y/2) + sin(R/2)cos(P/2)sin(Y/2) = cd*sy
            # qz = cos(R/2)cos(P/2)sin(Y/2) - sin(R/2)sin(P/2)cos(Y/2) = -(-sd)*cy = sd*cy
            # but P = -delta means sin(P/2) = -sd, cos(P/2) = cd
            qw = -sd * sy
            qx = cd * cy
            qy = cd * sy
            qz = sd * cy
            # Normalize
            n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if n > 1e-9:
                results.append((qx / n, qy / n, qz / n, qw / n))
        return results

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

        px = float(pose_stamped.pose.position.x)
        py = float(pose_stamped.pose.position.y)
        pz = float(pose_stamped.pose.position.z)
        self.get_logger().info(
            f"[{label}] IK target pos=({px:.3f},{py:.3f},{pz:.3f}) frame={pose_stamped.header.frame_id}"
        )

        candidates = self.candidate_orientations(px, py)
        for idx, (qx, qy, qz, qw) in enumerate(candidates):
            attempt_pose = Pose()
            attempt_pose.position.x = px
            attempt_pose.position.y = py
            attempt_pose.position.z = pz
            attempt_pose.orientation.x = qx
            attempt_pose.orientation.y = qy
            attempt_pose.orientation.z = qz
            attempt_pose.orientation.w = qw

            state = RobotState(robot_model)
            state.update()
            ok = state.set_from_ik(arm_name, attempt_pose, ee_link, timeout)
            if ok:
                self.get_logger().info(
                    f"[{label}] IK ok with orientation #{idx} "
                    f"quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})"
                )
                self.arm_component.set_start_state_to_current_state()
                self.arm_component.set_goal_state(robot_state=state)
                return self.plan_and_execute(self.arm_component, arm_name, label)
            self.get_logger().warn(f"[{label}] IK candidate #{idx} failed")

        self.get_logger().error(
            f"[{label}] IK failed for all {len(candidates)} candidate orientations"
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

    def plan_and_execute_gripper_value(self, grip_joint_rad, label):
        if self.gripper_component is None or self.moveit is None:
            return False
        from moveit.core.robot_state import RobotState

        gripper_name = str(self.get_parameter("gripper_group_name").value)
        robot_model = self.moveit.get_robot_model()
        state = RobotState(robot_model)
        state.set_joint_group_positions(gripper_name, [float(grip_joint_rad)])
        self.gripper_component.set_start_state_to_current_state()
        self.gripper_component.set_goal_state(robot_state=state)
        return self.plan_and_execute(self.gripper_component, gripper_name, label)

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

    def execute_pick_place(self, target_point, destination_point, grasp_width_m):
        home = str(self.get_parameter("home_named").value)
        open_name = str(self.get_parameter("gripper_open_named").value)
        pick_lift = float(self.get_parameter("pick_lift_m").value)
        place_lift = float(self.get_parameter("place_lift_m").value)
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
                 self.top_down_pose(target_point, 0.0), "04_pick")),
            ("05_close_gripper",
             lambda: self.plan_and_execute_gripper_value(grip_value, "05_close_gripper")),
            ("06_lift",
             lambda: self.plan_and_execute_pose(
                 self.top_down_pose(target_point, pick_lift), "06_lift")),
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

        for name, action in steps:
            self.get_logger().info(f"step '{name}' starting")
            if not action():
                self.get_logger().error(f"step '{name}' failed; aborting sequence")
                return False
        return True

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
