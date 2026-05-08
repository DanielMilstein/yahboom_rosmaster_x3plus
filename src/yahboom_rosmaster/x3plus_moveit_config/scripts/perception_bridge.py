#!/usr/bin/env python3

import math
import struct

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Vector3
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker
from yahboom_rosmaster_msgs.srv import ProjectDetectionBox

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros


def quaternion_rotate_vector(qx, qy, qz, qw, vector):
    x, y, z = vector
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


class PerceptionBridge(Node):
    def __init__(self):
        super().__init__("x3plus_perception_bridge")

        self.declare_parameter("rgb_topic", "/cam_1/color/image_raw")
        self.declare_parameter("depth_topic", "/cam_1/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/cam_1/color/camera_info")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("sync_slop", 0.08)
        self.declare_parameter("debug_pixel_u", -1)
        self.declare_parameter("debug_pixel_v", -1)
        self.declare_parameter("depth_search_radius", 10)
        self.declare_parameter("dimension_box_scale", 0.65)
        self.declare_parameter("dimension_sample_count", 7)
        self.declare_parameter("dimension_trim_fraction", 0.2)
        self.declare_parameter("override_intrinsics", False)
        self.declare_parameter("override_fx", 0.0)
        self.declare_parameter("override_fy", 0.0)
        self.declare_parameter("override_cx", 0.0)
        self.declare_parameter("override_cy", 0.0)
        self.declare_parameter("expected_horizontal_fov", -1.0)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        sync_slop = float(self.get_parameter("sync_slop").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.latest_synced_stamp = None
        self.last_debug_pixel = None
        self.last_depth_pixel = None
        self.warned_camera_info = False
        self.warned_intrinsics_override = False
        self.logged_projection_intrinsics = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rgb_sub = Subscriber(self, Image, self.rgb_topic, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=sync_slop
        )
        self.sync.registerCallback(self.synced_image_callback)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, qos
        )
        self.pixel_sub = self.create_subscription(
            PointStamped, "/perception_bridge/pixel", self.pixel_callback, 10
        )
        self.box_service = self.create_service(
            ProjectDetectionBox,
            "/perception_bridge/project_detection_box",
            self.project_detection_box_callback,
        )

        self.camera_point_pub = self.create_publisher(
            PointStamped, "/perception_bridge/selected_point_camera", 10
        )
        self.base_point_pub = self.create_publisher(
            PointStamped, "/perception_bridge/selected_point_base", 10
        )
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.marker_pub = self.create_publisher(
            Marker, "/perception_bridge/debug_marker", marker_qos
        )
        self.debug_image_pub = self.create_publisher(
            Image, "/perception_bridge/debug_image", 10
        )

        self.last_marker = None
        self.debug_timer = self.create_timer(1.0, self.publish_debug_pixel)
        self.marker_timer = self.create_timer(0.5, self.republish_last_marker)
        self.get_logger().info(
            f"Perception bridge listening to RGB={self.rgb_topic}, "
            f"depth={self.depth_topic}, camera_info={self.camera_info_topic}"
        )
        self.get_logger().info(
            "Publish geometry_msgs/PointStamped to /perception_bridge/pixel "
            "with point.x=u and point.y=v to project a pixel."
        )
        self.get_logger().info(
            "Call /perception_bridge/project_detection_box with normalized Gemini "
            "[y, x] point and [ymin, xmin, ymax, xmax] box to estimate 3D center/dimensions."
        )

    def synced_image_callback(self, rgb_msg, depth_msg):
        self.latest_rgb = rgb_msg
        self.latest_depth = depth_msg
        self.latest_synced_stamp = depth_msg.header.stamp
        self.publish_debug_image()

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg
        self.warn_if_camera_info_suspicious(msg)

    def capture_latest_rgb_frame(self):
        return self.latest_rgb

    def capture_latest_depth_frame(self):
        return self.latest_depth

    def capture_camera_intrinsics(self):
        return self.latest_camera_info

    def warn_if_camera_info_suspicious(self, camera_info):
        if self.warned_camera_info:
            return

        warnings = []
        expected_cx = camera_info.width * 0.5
        expected_cy = camera_info.height * 0.5
        cx = camera_info.k[2]
        cy = camera_info.k[5]
        fx = camera_info.k[0]

        if abs(cx - expected_cx) > max(2.0, camera_info.width * 0.05):
            warnings.append(
                f"cx={cx:.3f} is far from image center {expected_cx:.3f}"
            )
        if abs(cy - expected_cy) > max(2.0, camera_info.height * 0.05):
            warnings.append(
                f"cy={cy:.3f} is far from image center {expected_cy:.3f}"
            )

        expected_horizontal_fov = float(
            self.get_parameter("expected_horizontal_fov").value
        )
        if expected_horizontal_fov > 0.0 and camera_info.width > 0:
            expected_fx = camera_info.width / (
                2.0 * math.tan(expected_horizontal_fov * 0.5)
            )
            if expected_fx > 0.0 and abs(fx - expected_fx) / expected_fx > 0.05:
                warnings.append(
                    f"fx={fx:.3f} differs from FOV-derived fx={expected_fx:.3f}"
                )

        if warnings:
            self.warned_camera_info = True
            self.get_logger().warn(
                "CameraInfo may not match the image geometry: " + "; ".join(warnings)
            )

    def get_projection_intrinsics(self, camera_info):
        if bool(self.get_parameter("override_intrinsics").value):
            fx = float(self.get_parameter("override_fx").value)
            fy = float(self.get_parameter("override_fy").value)
            cx = float(self.get_parameter("override_cx").value)
            cy = float(self.get_parameter("override_cy").value)
            if fx <= 0.0 or fy <= 0.0:
                raise RuntimeError(
                    "override_intrinsics requires positive override_fx and override_fy"
                )
            if not self.warned_intrinsics_override:
                self.warned_intrinsics_override = True
                self.get_logger().warn(
                    "Using explicit projection intrinsics "
                    f"fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}"
                )
            return fx, fy, cx, cy

        fx, fy, cx, cy = camera_info.k[0], camera_info.k[4], camera_info.k[2], camera_info.k[5]
        if not self.logged_projection_intrinsics:
            self.logged_projection_intrinsics = True
            self.get_logger().info(
                "Using CameraInfo projection intrinsics "
                f"size={camera_info.width}x{camera_info.height} "
                f"fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}"
            )
        return fx, fy, cx, cy

    def pixel_callback(self, msg):
        u = int(round(msg.point.x))
        v = int(round(msg.point.y))
        self.last_debug_pixel = (u, v)
        self.publish_debug_image()
        self.project_and_publish(u, v)

    def publish_debug_pixel(self):
        u = int(self.get_parameter("debug_pixel_u").value)
        v = int(self.get_parameter("debug_pixel_v").value)
        if u >= 0 and v >= 0:
            self.last_debug_pixel = (u, v)
            self.publish_debug_image()
            self.project_and_publish(u, v, throttle_errors=True)

    def project_and_publish(self, u, v, throttle_errors=False):
        try:
            camera_point = self.project_2d_pixel_to_3d_point(u, v)
            base_point = self.transform_camera_point_to_base(camera_point)
        except Exception as exc:
            if throttle_errors:
                self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            else:
                self.get_logger().warn(str(exc))
            return None

        self.camera_point_pub.publish(camera_point)
        self.base_point_pub.publish(base_point)
        self.publish_marker(base_point)
        self.get_logger().info(
            f"pixel=({u}, {v}) depth_pixel={self.last_depth_pixel} "
            f"camera=({camera_point.point.x:.3f}, "
            f"{camera_point.point.y:.3f}, {camera_point.point.z:.3f}) "
            f"{self.base_frame}=({base_point.point.x:.3f}, "
            f"{base_point.point.y:.3f}, {base_point.point.z:.3f})"
        )
        return base_point

    def project_detection_box_callback(self, request, response):
        try:
            center_u, center_v = self.normalized_point_to_pixel(request.point)
            min_u, min_v, max_u, max_v = self.normalized_box_to_pixel_bounds(request.box)
            min_u, min_v, max_u, max_v = self.shrink_pixel_bounds(
                min_u,
                min_v,
                max_u,
                max_v,
                float(self.get_parameter("dimension_box_scale").value),
            )
            center_point = self.project_and_publish(center_u, center_v)
            if center_point is None:
                raise RuntimeError("Could not project detection center")

            samples = self.sample_box_points(min_u, min_v, max_u, max_v)
            if len(samples) < 4:
                raise RuntimeError(
                    f"Only {len(samples)} valid depth samples inside detection box"
                )

            xs = [point.point.x for point in samples]
            ys = [point.point.y for point in samples]
            zs = [point.point.z for point in samples]

            response.success = True
            response.error_message = ""
            response.center = center_point.point
            response.dimensions = Vector3(
                x=self.trimmed_range(xs),
                y=self.trimmed_range(ys),
                z=self.trimmed_range(zs),
            )
            response.valid_depth_samples = len(samples)
            self.get_logger().info(
                f"box label={request.label!r} center=({response.center.x:.3f}, "
                f"{response.center.y:.3f}, {response.center.z:.3f}) "
                f"dims=({response.dimensions.x:.3f}, {response.dimensions.y:.3f}, "
                f"{response.dimensions.z:.3f}) samples={response.valid_depth_samples}"
            )
        except Exception as exc:
            response.success = False
            response.error_message = str(exc)
            response.valid_depth_samples = 0
            self.get_logger().warn(f"Could not project detection box: {exc}")
        return response

    def normalized_point_to_pixel(self, point):
        rgb_msg = self.capture_latest_rgb_frame()
        if rgb_msg is None:
            raise RuntimeError("No RGB frame received yet")
        if len(point) != 2:
            raise RuntimeError("Detection point must be [y, x]")
        y = max(0.0, min(1.0, float(point[0])))
        x = max(0.0, min(1.0, float(point[1])))
        u = int(round(x * (rgb_msg.width - 1)))
        v = int(round(y * (rgb_msg.height - 1)))
        return u, v

    def normalized_box_to_pixel_bounds(self, box):
        rgb_msg = self.capture_latest_rgb_frame()
        if rgb_msg is None:
            raise RuntimeError("No RGB frame received yet")
        if len(box) != 4:
            raise RuntimeError("Detection box must be [ymin, xmin, ymax, xmax]")

        ymin, xmin, ymax, xmax = [max(0.0, min(1.0, float(value))) for value in box]
        if ymin > ymax or xmin > xmax:
            raise RuntimeError("Detection box min coordinates must not exceed max coordinates")

        min_u = int(round(xmin * (rgb_msg.width - 1)))
        max_u = int(round(xmax * (rgb_msg.width - 1)))
        min_v = int(round(ymin * (rgb_msg.height - 1)))
        max_v = int(round(ymax * (rgb_msg.height - 1)))
        return min_u, min_v, max_u, max_v

    def shrink_pixel_bounds(self, min_u, min_v, max_u, max_v, scale):
        scale = max(0.1, min(1.0, scale))
        center_u = (min_u + max_u) * 0.5
        center_v = (min_v + max_v) * 0.5
        half_width = max(1.0, (max_u - min_u) * 0.5 * scale)
        half_height = max(1.0, (max_v - min_v) * 0.5 * scale)
        return (
            int(round(center_u - half_width)),
            int(round(center_v - half_height)),
            int(round(center_u + half_width)),
            int(round(center_v + half_height)),
        )

    def trimmed_range(self, values):
        if not values:
            return 0.0
        sorted_values = sorted(values)
        trim_fraction = max(
            0.0,
            min(0.45, float(self.get_parameter("dimension_trim_fraction").value)),
        )
        trim_count = int(len(sorted_values) * trim_fraction)
        if trim_count > 0 and trim_count * 2 < len(sorted_values):
            sorted_values = sorted_values[trim_count:-trim_count]
        return max(sorted_values) - min(sorted_values)

    def sample_box_points(self, min_u, min_v, max_u, max_v):
        depth_msg = self.capture_latest_depth_frame()
        if depth_msg is None:
            raise RuntimeError("No synchronized depth frame received yet")

        min_u = max(0, min(depth_msg.width - 1, min_u))
        max_u = max(0, min(depth_msg.width - 1, max_u))
        min_v = max(0, min(depth_msg.height - 1, min_v))
        max_v = max(0, min(depth_msg.height - 1, max_v))
        if min_u > max_u or min_v > max_v:
            raise RuntimeError("Detection box is outside the depth image")

        sample_count = max(3, int(self.get_parameter("dimension_sample_count").value))
        points = []
        for row in range(sample_count):
            if sample_count == 1:
                v = min_v
            else:
                v = int(round(min_v + (max_v - min_v) * row / (sample_count - 1)))
            for col in range(sample_count):
                if sample_count == 1:
                    u = min_u
                else:
                    u = int(round(min_u + (max_u - min_u) * col / (sample_count - 1)))
                try:
                    camera_point = self.project_2d_pixel_to_3d_point(u, v)
                    points.append(self.transform_camera_point_to_base(camera_point))
                except Exception:
                    continue
        return points

    def project_2d_pixel_to_3d_point(self, u, v):
        depth_msg = self.capture_latest_depth_frame()
        camera_info = self.capture_camera_intrinsics()
        if depth_msg is None:
            raise RuntimeError("No synchronized depth frame received yet")
        if camera_info is None:
            raise RuntimeError("No camera_info received yet")
        if u < 0 or v < 0 or u >= depth_msg.width or v >= depth_msg.height:
            raise RuntimeError(
                f"Pixel ({u}, {v}) is outside depth image {depth_msg.width}x{depth_msg.height}"
            )

        depth_m, depth_u, depth_v = self.depth_near_pixel(depth_msg, u, v)
        self.last_depth_pixel = (depth_u, depth_v)
        self.publish_debug_image()
        if not math.isfinite(depth_m) or depth_m <= 0.0:
            raise RuntimeError(f"Invalid depth {depth_m} at pixel ({u}, {v})")

        fx, fy, cx, cy = self.get_projection_intrinsics(camera_info)
        if fx == 0.0 or fy == 0.0:
            raise RuntimeError("camera_info has invalid focal length")

        point = PointStamped()
        point.header.stamp = depth_msg.header.stamp
        point.header.frame_id = depth_msg.header.frame_id or camera_info.header.frame_id
        point.point.x = (float(depth_u) - cx) * depth_m / fx
        point.point.y = (float(depth_v) - cy) * depth_m / fy
        point.point.z = depth_m
        return point

    def depth_near_pixel(self, depth_msg, u, v):
        depth_m = self.depth_at_pixel(depth_msg, u, v)
        if math.isfinite(depth_m) and depth_m > 0.0:
            return depth_m, u, v

        radius = int(self.get_parameter("depth_search_radius").value)
        best = None
        for dy in range(-radius, radius + 1):
            sample_v = v + dy
            if sample_v < 0 or sample_v >= depth_msg.height:
                continue
            for dx in range(-radius, radius + 1):
                sample_u = u + dx
                if sample_u < 0 or sample_u >= depth_msg.width:
                    continue
                sample_depth = self.depth_at_pixel(depth_msg, sample_u, sample_v)
                if not math.isfinite(sample_depth) or sample_depth <= 0.0:
                    continue
                distance_sq = dx * dx + dy * dy
                if best is None or distance_sq < best[0]:
                    best = (distance_sq, sample_depth, sample_u, sample_v)

        if best is None:
            return depth_m, u, v

        _, sample_depth, sample_u, sample_v = best
        self.get_logger().info(
            f"Using nearest valid depth pixel ({sample_u}, {sample_v}) for requested pixel ({u}, {v})"
        )
        return sample_depth, sample_u, sample_v

    def depth_at_pixel(self, depth_msg, u, v):
        index = v * depth_msg.width + u
        encoding = depth_msg.encoding.upper()

        if encoding in ("32FC1", "TYPE_32FC1"):
            byte_index = index * 4
            return struct.unpack_from("<f", depth_msg.data, byte_index)[0]
        if encoding in ("16UC1", "MONO16", "TYPE_16UC1"):
            byte_index = index * 2
            return struct.unpack_from("<H", depth_msg.data, byte_index)[0] * 0.001

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        value = float(depth_image[v, u])
        if "16U" in encoding or encoding == "MONO16":
            value *= 0.001
        return value

    def transform_camera_point_to_base(self, camera_point):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                camera_point.header.frame_id,
                rclpy.time.Time.from_msg(camera_point.header.stamp),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                camera_point.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )

        t = transform.transform.translation
        q = transform.transform.rotation
        rx, ry, rz = quaternion_rotate_vector(
            q.x,
            q.y,
            q.z,
            q.w,
            (camera_point.point.x, camera_point.point.y, camera_point.point.z),
        )

        base_point = PointStamped()
        base_point.header.stamp = camera_point.header.stamp
        base_point.header.frame_id = self.base_frame
        base_point.point.x = rx + t.x
        base_point.point.y = ry + t.y
        base_point.point.z = rz + t.z
        return base_point

    def publish_marker(self, point):
        marker = Marker()
        marker.header.frame_id = point.header.frame_id
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = "perception_bridge"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.05
        marker.color.a = 1.0
        self.last_marker = marker
        self.marker_pub.publish(marker)

    def republish_last_marker(self):
        if self.last_marker is not None:
            self.marker_pub.publish(self.last_marker)

    def publish_debug_image(self):
        rgb_msg = self.capture_latest_rgb_frame()
        if rgb_msg is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Could not convert RGB image for debug overlay: {exc}")
            return

        if self.last_debug_pixel is not None:
            u, v = self.last_debug_pixel
            height, width = image.shape[:2]
            if 0 <= u < width and 0 <= v < height:
                color = (0, 255, 0)
                half_size = 10
                cv2.rectangle(
                    image,
                    (max(0, u - half_size), max(0, v - half_size)),
                    (min(width - 1, u + half_size), min(height - 1, v + half_size)),
                    color,
                    2,
                )
                cv2.line(image, (max(0, u - 16), v), (min(width - 1, u + 16), v), color, 1)
                cv2.line(image, (u, max(0, v - 16)), (u, min(height - 1, v + 16)), color, 1)

        if self.last_depth_pixel is not None:
            u, v = self.last_depth_pixel
            height, width = image.shape[:2]
            if 0 <= u < width and 0 <= v < height:
                color = (0, 255, 255)
                cv2.circle(image, (u, v), 5, color, 2)

        debug_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug_msg.header = rgb_msg.header
        self.debug_image_pub.publish(debug_msg)


def main():
    rclpy.init()
    node = PerceptionBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
