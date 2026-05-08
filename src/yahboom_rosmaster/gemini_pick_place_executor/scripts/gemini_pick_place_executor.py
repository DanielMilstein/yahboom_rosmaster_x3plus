#!/usr/bin/env python3

import json
import threading
import time
from copy import deepcopy
from types import SimpleNamespace

from geometry_msgs.msg import Point, PointStamped, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from yahboom_rosmaster_msgs.action import PickPlaceManipulation
from yahboom_rosmaster_msgs.srv import GeminiPickPlace
from yahboom_rosmaster_msgs.srv import ProjectDetectionBox


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


class GeminiPickPlaceExecutor(Node):
    def __init__(self):
        super().__init__("gemini_pick_place_executor")

        self.declare_parameter("task", "put the red cube in the blue bin")
        self.declare_parameter("image_topic", "/perception_bridge/debug_image")
        self.declare_parameter("gemini_service", "/gemini_pick_place")
        self.declare_parameter("box_projection_service", "/perception_bridge/project_detection_box")
        self.declare_parameter("manipulation_action", "pick_place_manipulation")
        self.declare_parameter("pixel_topic", "/perception_bridge/pixel")
        self.declare_parameter("base_point_topic", "/perception_bridge/selected_point_base")
        self.declare_parameter("marker_topic", "/gemini_pick_place/debug_markers")
        self.declare_parameter("marker_frame", "base_footprint")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("project_timeout_sec", 3.0)
        self.declare_parameter("service_timeout_sec", 10.0)
        self.declare_parameter("action_timeout_sec", 180.0)
        self.declare_parameter("pick_lift_m", 0.08)
        self.declare_parameter("place_lift_m", 0.08)
        self.declare_parameter("destination_point_source", "box_bias")
        self.declare_parameter("destination_box_y_fraction", 0.5)
        self.declare_parameter("destination_box_x_fraction", 0.5)
        self.declare_parameter("use_sim_task_geometry", False)
        self.declare_parameter("sim_target_x", 0.24)
        self.declare_parameter("sim_target_y", 0.10)
        self.declare_parameter("sim_target_z", 0.20)
        self.declare_parameter("sim_target_size_x", 0.03)
        self.declare_parameter("sim_target_size_y", 0.03)
        self.declare_parameter("sim_target_size_z", 0.03)
        self.declare_parameter("sim_destination_x", 0.39)
        self.declare_parameter("sim_destination_y", -0.07)
        self.declare_parameter("sim_destination_z", 0.20)
        self.declare_parameter("sim_destination_size_x", 0.12)
        self.declare_parameter("sim_destination_size_y", 0.12)
        self.declare_parameter("sim_destination_size_z", 0.08)
        self.declare_parameter("execute", False)

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
        self.box_projection_client = self.create_client(
            ProjectDetectionBox, self.get_parameter("box_projection_service").value
        )
        self.manipulation_client = ActionClient(
            self, PickPlaceManipulation, self.get_parameter("manipulation_action").value
        )
        self.start_timer = self.create_timer(0.5, self.maybe_start)

        self.get_logger().info(
            f"Waiting for image on {image_topic}; markers will publish on {marker_topic}"
        )

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

        if "box" not in plan["target_object"]:
            self.get_logger().error("Gemini target_object is missing required box")
            return
        if "box" not in plan["destination"]:
            self.get_logger().error("Gemini destination is missing required box")
            return

        destination_point_2d, destination_reason = self.destination_point_2d(plan)

        if bool(self.get_parameter("use_sim_task_geometry").value):
            target_projection = self.sim_projection("target")
            destination_projection = self.sim_projection("destination")
            self.get_logger().warn(
                "Using known Gazebo task geometry instead of Gemini/depth projection"
            )
        else:
            target_projection = self.project_detection(
                "target",
                plan["target_object"]["label"],
                plan["target_object"]["point"],
                plan["target_object"]["box"],
            )
            if target_projection is None:
                return
            destination_projection = self.project_detection(
                "destination",
                plan["destination"]["label"],
                destination_point_2d,
                plan["destination"]["box"],
            )
            if destination_projection is None:
                return

        self.get_logger().info(f"Destination point source: {destination_reason}")
        manipulation_result = self.call_manipulation(plan, target_projection, destination_projection)
        self.publish_debug_markers(
            target_projection.center,
            destination_projection.center,
            manipulation_result.poses if manipulation_result is not None else [],
            manipulation_result.stage_names if manipulation_result is not None else [],
        )
        self.log_candidate_summary(plan, target_projection, destination_projection, manipulation_result)

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

    def project_detection(self, name, label, point, box):
        timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        if not self.box_projection_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("Timed out waiting for /perception_bridge/project_detection_box")
            return None

        request = ProjectDetectionBox.Request()
        request.label = str(label)
        request.point = [float(point[0]), float(point[1])]
        request.box = [float(value) for value in box]
        future = self.box_projection_client.call_async(request)

        while rclpy.ok() and not future.done():
            time.sleep(0.05)

        if not future.done() or future.result() is None:
            self.get_logger().error(f"Projection service did not return a {name} response")
            return None

        response = future.result()
        if not response.success:
            self.get_logger().error(f"Could not project {name}: {response.error_message}")
            return None

        self.get_logger().info(
            f"{name} projection: center=({response.center.x:.3f}, "
            f"{response.center.y:.3f}, {response.center.z:.3f}) "
            f"dims=({response.dimensions.x:.3f}, {response.dimensions.y:.3f}, "
            f"{response.dimensions.z:.3f}) samples={response.valid_depth_samples}"
        )
        return response

    def sim_projection(self, name):
        prefix = "sim_target" if name == "target" else "sim_destination"
        center = Point()
        center.x = float(self.get_parameter(f"{prefix}_x").value)
        center.y = float(self.get_parameter(f"{prefix}_y").value)
        center.z = float(self.get_parameter(f"{prefix}_z").value)

        dimensions = Vector3()
        dimensions.x = float(self.get_parameter(f"{prefix}_size_x").value)
        dimensions.y = float(self.get_parameter(f"{prefix}_size_y").value)
        dimensions.z = float(self.get_parameter(f"{prefix}_size_z").value)

        self.get_logger().info(
            f"{name} sim geometry: center=({center.x:.3f}, {center.y:.3f}, {center.z:.3f}) "
            f"dims=({dimensions.x:.3f}, {dimensions.y:.3f}, {dimensions.z:.3f})"
        )
        return SimpleNamespace(center=center, dimensions=dimensions, valid_depth_samples=0)

    def call_manipulation(self, plan, target_projection, destination_projection):
        timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        if not self.manipulation_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("Timed out waiting for pick_place_manipulation action server")
            return None

        goal = PickPlaceManipulation.Goal()
        goal.target_label = str(plan["target_object"]["label"])
        goal.target_center = target_projection.center
        goal.target_dimensions = target_projection.dimensions
        goal.destination_label = str(plan["destination"]["label"])
        goal.destination_center = destination_projection.center
        goal.destination_dimensions = destination_projection.dimensions
        goal.execute = bool(self.get_parameter("execute").value)

        send_future = self.manipulation_client.send_goal_async(
            goal, feedback_callback=self.manipulation_feedback_callback
        )
        while rclpy.ok() and not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Manipulation action goal was rejected")
            return None

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + float(self.get_parameter("action_timeout_sec").value)
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not result_future.done() or result_future.result() is None:
            self.get_logger().error("Timed out waiting for manipulation action result")
            return None

        result = result_future.result().result
        if result.success:
            self.get_logger().info("Manipulation planning/execution succeeded")
        else:
            self.get_logger().error(
                f"Manipulation failed at {result.failed_stage}: {result.error_message}"
            )
        return result

    def manipulation_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Manipulation feedback: {feedback.stage_name} {feedback.state}"
        )

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

    def publish_debug_markers(self, target_point, destination_point, poses=None, stage_names=None):
        poses = poses or []
        stage_names = stage_names or []
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
        for index, pose in enumerate(poses):
            name = stage_names[index] if index < len(stage_names) else f"stage_{index}"
            markers.markers.append(
                self.make_pose_marker(10 + index, name, pose, make_color(0.9, 0.9, 0.1))
            )
        self.marker_pub.publish(markers)

    def make_sphere_marker(self, marker_id, namespace, point, color):
        marker = Marker()
        marker.header.frame_id = self.get_parameter("marker_frame").value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color = color
        return marker

    def make_line_marker(self, marker_id, target_point, destination_point):
        marker = Marker()
        marker.header.frame_id = self.get_parameter("marker_frame").value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place_line"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color = make_color(0.9, 0.9, 0.9)
        marker.points.append(target_point)
        marker.points.append(destination_point)
        return marker

    def make_lift_marker(self, marker_id, namespace, point, lift_m, color):
        marker = Marker()
        marker.header.frame_id = self.get_parameter("marker_frame").value
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

        start = deepcopy(point)
        end = deepcopy(point)
        end.z += lift_m
        marker.points.append(start)
        marker.points.append(end)
        return marker

    def make_pose_marker(self, marker_id, namespace, pose, color):
        marker = Marker()
        marker.header.frame_id = self.get_parameter("marker_frame").value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.06
        marker.scale.y = 0.012
        marker.scale.z = 0.012
        marker.color = color
        return marker

    def log_candidate_summary(self, plan, target_projection, destination_projection, manipulation_result):
        pick_lift = float(self.get_parameter("pick_lift_m").value)
        place_lift = float(self.get_parameter("place_lift_m").value)
        target_point = target_projection.center
        destination_point = destination_projection.center
        status = "not requested" if manipulation_result is None else str(manipulation_result.success)
        self.get_logger().info(
            "Pick/place candidate summary. "
            f"target={plan['target_object']['label']} "
            f"at ({target_point.x:.3f}, {target_point.y:.3f}, {target_point.z:.3f}), "
            f"destination={plan['destination']['label']} "
            f"at ({destination_point.x:.3f}, {destination_point.y:.3f}, {destination_point.z:.3f}), "
            f"pick_lift={pick_lift:.3f}m place_lift={place_lift:.3f}m "
            f"manipulation_success={status}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GeminiPickPlaceExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
