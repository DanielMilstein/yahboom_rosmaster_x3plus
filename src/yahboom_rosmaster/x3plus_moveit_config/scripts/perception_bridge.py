#!/usr/bin/env python3

import math
import struct

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker

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

    def synced_image_callback(self, rgb_msg, depth_msg):
        self.latest_rgb = rgb_msg
        self.latest_depth = depth_msg
        self.latest_synced_stamp = depth_msg.header.stamp
        self.publish_debug_image()

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def capture_latest_rgb_frame(self):
        return self.latest_rgb

    def capture_latest_depth_frame(self):
        return self.latest_depth

    def capture_camera_intrinsics(self):
        return self.latest_camera_info

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
            f"pixel=({u}, {v}) camera=({camera_point.point.x:.3f}, "
            f"{camera_point.point.y:.3f}, {camera_point.point.z:.3f}) "
            f"{self.base_frame}=({base_point.point.x:.3f}, "
            f"{base_point.point.y:.3f}, {base_point.point.z:.3f})"
        )
        return base_point

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

        depth_m = self.depth_at_pixel(depth_msg, u, v)
        if not math.isfinite(depth_m) or depth_m <= 0.0:
            raise RuntimeError(f"Invalid depth {depth_m} at pixel ({u}, {v})")

        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]
        if fx == 0.0 or fy == 0.0:
            raise RuntimeError("camera_info has invalid focal length")

        point = PointStamped()
        point.header.stamp = depth_msg.header.stamp
        point.header.frame_id = depth_msg.header.frame_id or camera_info.header.frame_id
        point.point.x = (float(u) - cx) * depth_m / fx
        point.point.y = (float(v) - cy) * depth_m / fy
        point.point.z = depth_m
        return point

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
