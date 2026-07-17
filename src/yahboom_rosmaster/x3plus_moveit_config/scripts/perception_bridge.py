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
        # Spiral-search radius (px) for a valid depth around the requested
        # pixel. 20 px: at the Astra's ~0.6 m minimum range the dropout
        # patch around an object can exceed the old 10 px.
        self.declare_parameter("depth_search_radius", 20)
        self.declare_parameter("override_intrinsics", False)
        self.declare_parameter("override_fx", 0.0)
        self.declare_parameter("override_fy", 0.0)
        self.declare_parameter("override_cx", 0.0)
        self.declare_parameter("override_cy", 0.0)
        self.declare_parameter("expected_horizontal_fov", -1.0)
        # Constant correction added to every projected point in base_frame
        # [x, y, z] (metres). Compensates a small camera extrinsic bias (the
        # Astra's mount pose in the URDF is slightly off, biasing localization
        # laterally at the ~0.45 m grasp range). Default [0,0,0] = no change /
        # sim behavior. Relative measurements (object height, grasp width)
        # cancel the offset; only absolute target points shift. Tune from a
        # no-drive perceive-only run: offset = (measured_pos - perceived_pos).
        # NOTE: the +0.125 x offset that briefly lived here was fitted to a
        # wrong-surface depth artifact (spiral search sampling platform/wall
        # pixels when the white cube's depth drops out) — it chased a moving
        # target and poisons the plane-ranging path. x belongs at 0; the y
        # trim is a real lateral extrinsic bias and stays.
        self.declare_parameter("correction_offset_xyz", [0.0, 0.037, 0.0])
        # Camera pitch trim (rad), applied to the optical-frame point before
        # the TF to base. Corrects a physically mis-pitched camera mount: the
        # signature is perception reading LOW and SHORT, with both errors
        # growing linearly with range. POSITIVE tips the perceived scene UP
        # (fixes low+short); magnitude ~= z_error / ray_length. Unlike
        # correction_offset_xyz (constant, valid at one range only), a pitch
        # trim is correct at every range. Runtime-tunable:
        #   ros2 param set /x3plus_perception_bridge pitch_correction_rad 0.12
        self.declare_parameter("pitch_correction_rad", 0.0)

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

        return camera_info.k[0], camera_info.k[4], camera_info.k[2], camera_info.k[5]

    def pixel_callback(self, msg):
        u = int(round(msg.point.x))
        v = int(round(msg.point.y))
        # point.z doubles as the mode switch: z > 0 requests plane ranging
        # (intersect the pixel ray with the horizontal base-frame plane at
        # that height, no depth involved); z <= 0 keeps the depth path.
        plane_z = float(msg.point.z)
        self.last_debug_pixel = (u, v)
        self.publish_debug_image()
        self.project_and_publish(
            u, v, plane_z=plane_z if plane_z > 0.0 else None
        )

    def publish_debug_pixel(self):
        u = int(self.get_parameter("debug_pixel_u").value)
        v = int(self.get_parameter("debug_pixel_v").value)
        if u >= 0 and v >= 0:
            self.last_debug_pixel = (u, v)
            self.publish_debug_image()
            self.project_and_publish(u, v, throttle_errors=True)

    def project_and_publish(self, u, v, throttle_errors=False, plane_z=None):
        try:
            if plane_z is not None:
                camera_point, base_point = self.project_pixel_plane_to_base(
                    u, v, plane_z
                )
            else:
                camera_point = self.project_2d_pixel_to_3d_point(u, v)
                base_point = self.transform_camera_point_to_base(camera_point)
        except Exception as exc:
            if throttle_errors:
                self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            else:
                self.get_logger().warn(str(exc))
            # Always answer the requester: a NaN sentinel unblocks the
            # executor immediately (instead of a silent 3 s timeout) and
            # tells it the failure was bridge-side (reason logged above).
            if not throttle_errors:
                failure = PointStamped()
                failure.header.stamp = self.get_clock().now().to_msg()
                failure.header.frame_id = self.base_frame
                failure.point.x = float("nan")
                failure.point.y = float("nan")
                failure.point.z = float("nan")
                self.base_point_pub.publish(failure)
            return None

        self.camera_point_pub.publish(camera_point)
        self.base_point_pub.publish(base_point)
        self.publish_marker(base_point)
        mode = f"plane@{plane_z:.3f}" if plane_z is not None else "depth"
        self.get_logger().info(
            f"pixel=({u}, {v}) mode={mode} depth_pixel={self.last_depth_pixel} "
            f"camera=({camera_point.point.x:.3f}, "
            f"{camera_point.point.y:.3f}, {camera_point.point.z:.3f}) "
            f"{self.base_frame}=({base_point.point.x:.3f}, "
            f"{base_point.point.y:.3f}, {base_point.point.z:.3f})"
        )
        return base_point

    def project_pixel_plane_to_base(self, u, v, plane_z):
        """Project (u, v) by intersecting its viewing ray with the
        horizontal base-frame plane z=plane_z. Depth is never consulted:
        this is the ranging path for objects whose depth pixels drop out
        (the 30mm white cube at 0.6-1.0 m), where the depth spiral would
        silently sample a NEIGHBORING surface and corrupt x by a scene-
        dependent amount. The ray geometry is calibrated (camera pitch
        verified against multi-range z data) and the plane height is
        tape-measured, so x/y land within ~2 cm with no depth dependency —
        and it keeps working inside the Astra's 0.6 m minimum depth range.
        Returns (camera_point, base_point); raises on degenerate rays.
        """
        camera_info = self.capture_camera_intrinsics()
        if camera_info is None:
            raise RuntimeError("No camera_info received yet")
        if u < 0 or v < 0 or u >= camera_info.width or v >= camera_info.height:
            raise RuntimeError(
                f"Pixel ({u}, {v}) is outside image "
                f"{camera_info.width}x{camera_info.height}"
            )
        fx, fy, cx, cy = self.get_projection_intrinsics(camera_info)
        if fx == 0.0 or fy == 0.0:
            raise RuntimeError("camera_info has invalid focal length")

        # Optical-frame ray through the pixel (x right, y down, z forward),
        # pitch-trimmed exactly like the depth path.
        dx = (float(u) - cx) / fx
        dy = (float(v) - cy) / fy
        dz = 1.0
        pitch = float(self.get_parameter("pitch_correction_rad").value)
        if pitch != 0.0:
            c, s = math.cos(pitch), math.sin(pitch)
            dy, dz = c * dy - s * dz, s * dy + c * dz

        stamp = self.latest_synced_stamp or self.get_clock().now().to_msg()
        frame = camera_info.header.frame_id
        origin = PointStamped()
        origin.header.stamp = stamp
        origin.header.frame_id = frame
        tip = PointStamped()
        tip.header.stamp = stamp
        tip.header.frame_id = frame
        tip.point.x, tip.point.y, tip.point.z = dx, dy, dz
        # Both endpoints go through the standard transform (incl. the
        # extrinsic correction offset): the offset cancels in the direction
        # and correctly shifts the ray origin.
        origin_b = self.transform_camera_point_to_base(origin)
        tip_b = self.transform_camera_point_to_base(tip)
        rdx = tip_b.point.x - origin_b.point.x
        rdy = tip_b.point.y - origin_b.point.y
        rdz = tip_b.point.z - origin_b.point.z
        norm = math.sqrt(rdx * rdx + rdy * rdy + rdz * rdz)
        if norm < 1e-6:
            raise RuntimeError("plane ranging: degenerate ray")
        if rdz / norm > -0.087:
            # Ray must descend toward the plane by at least ~5 deg or the
            # intersection amplifies pixel noise into meters.
            raise RuntimeError(
                f"plane ranging: ray too grazing (dz/|d|={rdz / norm:+.3f})"
            )
        s_par = (float(plane_z) - origin_b.point.z) / rdz
        if s_par <= 0.0 or s_par * norm > 5.0:
            raise RuntimeError(
                f"plane ranging: implausible intersection (range "
                f"{s_par * norm:.2f} m)"
            )

        base_point = PointStamped()
        base_point.header.stamp = stamp
        base_point.header.frame_id = self.base_frame
        base_point.point.x = origin_b.point.x + s_par * rdx
        base_point.point.y = origin_b.point.y + s_par * rdy
        base_point.point.z = origin_b.point.z + s_par * rdz
        camera_point = PointStamped()
        camera_point.header.stamp = stamp
        camera_point.header.frame_id = frame
        camera_point.point.x = dx * s_par
        camera_point.point.y = dy * s_par
        camera_point.point.z = dz * s_par
        return camera_point, base_point

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
        pitch = float(self.get_parameter("pitch_correction_rad").value)
        if pitch != 0.0:
            # Rotate about the optical x-axis (x right, y down, z forward):
            # positive pitch moves the point toward -y (up) — range preserved.
            c, s = math.cos(pitch), math.sin(pitch)
            y, z = point.point.y, point.point.z
            point.point.y = c * y - s * z
            point.point.z = s * y + c * z
        return point

    def depth_near_pixel(self, depth_msg, u, v):
        depth_m = self.depth_at_pixel(depth_msg, u, v)
        if math.isfinite(depth_m) and depth_m > 0.0:
            return depth_m, u, v

        radius = int(self.get_parameter("depth_search_radius").value)
        samples = []
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
                samples.append((sample_depth, sample_u, sample_v))

        if not samples:
            return depth_m, u, v

        # MEDIAN of the valid neighborhood, not the nearest single pixel:
        # around a dropped-out object the nearest valid sample is usually a
        # DIFFERENT surface (platform in front/behind, wall edge), which
        # silently poisons the range by a scene-dependent amount — the root
        # cause of the wandering x error this bridge chased with offsets.
        samples.sort(key=lambda s: s[0])
        sample_depth, sample_u, sample_v = samples[len(samples) // 2]
        dist = math.hypot(sample_u - u, sample_v - v)
        if dist > 8.0:
            self.get_logger().warn(
                f"depth fallback: sampled ({sample_u}, {sample_v}), "
                f"{dist:.0f}px from requested ({u}, {v}) — likely a "
                "different surface; treat this range as suspect",
                throttle_duration_sec=2.0,
            )
        else:
            self.get_logger().info(
                f"Using median neighborhood depth pixel ({sample_u}, "
                f"{sample_v}) for requested pixel ({u}, {v})"
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

        off = list(self.get_parameter("correction_offset_xyz").value)
        if len(off) != 3:
            off = [0.0, 0.0, 0.0]

        base_point = PointStamped()
        base_point.header.stamp = camera_point.header.stamp
        base_point.header.frame_id = self.base_frame
        base_point.point.x = rx + t.x + float(off[0])
        base_point.point.y = ry + t.y + float(off[1])
        base_point.point.z = rz + t.z + float(off[2])
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
