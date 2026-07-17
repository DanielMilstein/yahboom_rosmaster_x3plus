#!/usr/bin/env python3

import faulthandler
import json
import math
import threading
import time
from copy import deepcopy

# moveit_py is a C++ binding; if it segfaults we want the Python-level
# stack on stderr instead of a silent exit code -11.
faulthandler.enable(all_threads=True)

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState, LaserScan
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.action import ActionClient
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
        # Diagnostic mode: instead of the pick-place flow, scan fx (fy=0) at
        # ik_probe_z against all orientation candidates and log which solve
        # IK. Maps the 5-DOF arm's KDL feasibility boundary on real config.
        self.declare_parameter("ik_probe", False)
        self.declare_parameter("ik_probe_z", 0.166)
        # x of arm_joint1 (the arm's yaw column) in base_footprint, from the
        # URDF (rosmaster_x3_plus_arm.urdf.xacro arm_joint1 origin). Used to
        # put candidate grasp yaws on the 5-DOF arm's reachable manifold.
        self.declare_parameter("arm_base_offset_x_m", 0.09825)
        # Abort a closed-loop drive if the position error grows this much
        # past its best value — catches inverted/mis-scaled odometry
        # (positive-feedback runaway) within centimeters. 0 disables.
        self.declare_parameter("drive_abort_divergence_m", 0.10)
        # Re-run perception after the approach drive (sim default: refines
        # the target from the closer vantage). On hardware set false: the
        # drive puts the target inside the camera's ~0.6 m minimum range,
        # so re-perception always fails — dead-reckon through odom instead.
        self.declare_parameter("reperceive_after_drive", True)
        self.declare_parameter("project_timeout_sec", 3.0)
        # Per-pixel projection retries (no Gemini re-call): covers a bad
        # depth read / TF hiccup at the bridge, which answers with a NaN
        # sentinel, and a lost request/response (timeout).
        self.declare_parameter("project_attempts", 4)
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
        # Fingertip position in arm_link5's local frame: the gripper extends
        # ~9 cm along arm_link5's +Z (magnitude tape-measured wrist->fingertip;
        # direction confirmed on hardware 2026-07-02 by comparing FK of the
        # actual joints against photos — with -0.09 the model placed the
        # fingertip 9 cm BEHIND the flange, so the wrist was commanded 9 cm
        # past the object and the real fingers overshot it). For each candidate
        # orientation Q, the wrist IK target is computed as
        # `fingertip_target - R(Q) * gripper_tip_offset_xyz`, so the fingertip
        # lands on the perceived point regardless of orientation.
        self.declare_parameter("gripper_tip_offset_xyz", [0.0, 0.0, 0.09])
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
        # Descend this far BELOW the perceived target.z to grip the object
        # body (positive = lower). Gemini's target point projects to the
        # object's visible TOP surface; for a short object whose only
        # perceived point is the top, the fingers must close around the body
        # a centimetre or two beneath it. Default 0.0 preserves sim behavior
        # (grasp at target.z). The table-floor clamp still applies on top.
        self.declare_parameter("grasp_z_offset_m", 0.0)
        # Roll applied to the wrist (arm_joint5) of every IK-solved grasp pose
        # to seat the jaws in the grasping plane. IK leaves the wrist roll
        # free and often picks one-up/one-down; ~+/-1.5708 corrects it.
        # Default 0 preserves sim behavior.
        self.declare_parameter("grasp_roll_offset_rad", 0.0)
        # When True, candidate_orientations yields the near-horizontal tilts
        # FIRST (top-down last). At the arm's forward reach limit only the
        # near-horizontal side-grasps are kinematically feasible (verified by
        # IK probe and by hand); the default top-down-first order otherwise
        # picks a too-vertical orientation that can't get its jaws around a
        # short object's body. Default False preserves sim (close, top-down)
        # behavior.
        self.declare_parameter("grasp_tilt_first", False)
        # Table-height safety floor. The pick fingertip is clamped so it never
        # descends below `table_z + pick_z_safety_m`. With table_z_source set
        # to "perception", we use the z_bottom from measure_object_extent
        # (projection of the bbox-bottom-center pixel — typically the table
        # level at the can's base). With "param", we use table_z_m directly.
        self.declare_parameter("table_z_source", "perception")
        self.declare_parameter("table_z_m", 0.14)
        # Base-search candidate ordering: "min_reach" (default) drives so the
        # target ends closest to the arm column (grasp mid-envelope, least
        # servo droop); "min_drive" is the legacy smallest-base-motion-first.
        self.declare_parameter("base_search_order", "min_reach")
        # Horizontal distance from the arm column where the grasp is most
        # comfortable (mid-envelope). min_reach ordering aims the drive here.
        self.declare_parameter("base_search_ideal_reach_m", 0.33)
        # When reperceive_after_drive is set, cap the FIRST drive so the
        # target stays at least this far ahead (x, base frame) — outside the
        # depth camera's ~0.6 m minimum-range blind zone — so the post-drive
        # re-perception can actually see it. The corrected drive then closes
        # the remaining distance with fresh, close-range perception.
        # 0.62, not 0.55: the close perception used to park the cube right
        # inside the Astra's ~0.6 m minimum depth range, maximizing dropouts.
        self.declare_parameter("reperceive_min_target_x_m", 0.62)
        # Ground-plane ranging for the target's x/y: intersect the bbox
        # bottom-center pixel ray with the tape-measured platform plane
        # (table_z_m) instead of trusting depth. Depth dropouts on the white
        # cube made the bridge's spiral sample NEIGHBORING surfaces, moving
        # x by a scene-dependent amount — no constant offset can fix that.
        # z stays on the (calibrated, accurate) depth path.
        self.declare_parameter("plane_ranging", False)
        # Lidar front-wall x reference: the gap (x_target - x_wall) is
        # invariant to robot pose, so it is logged on every perception for
        # calibration. Set wall_to_target_x_m to the taped wall-face ->
        # cube-near-face distance to GATE the vision x against the lidar
        # (warn beyond wall_ref_tol_m); wall_ref_override additionally
        # replaces the vision x with wall_x + wall_to_target_x_m — only
        # valid when the cube is placed at the taped spot (fixed demo).
        self.declare_parameter("wall_to_target_x_m", -1.0)
        self.declare_parameter("wall_ref_tol_m", 0.06)
        self.declare_parameter("wall_ref_override", False)
        # Lidar scan-match drive audit. Before/after each base drive the
        # full scan (front + sides — no flat-landmark assumption; the thing
        # ahead is a 3D printer) is ICP-matched to measure the TRUE planar
        # motion (dx, dy, dyaw) and log the disagreement with wheel-odometry
        # dead reckoning. Observe-only unless lidar_drive_correction is set.
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("lidar_audit", True)
        self.declare_parameter("lidar_min_range_m", 0.25)
        self.declare_parameter("lidar_max_range_m", 2.5)
        self.declare_parameter("lidar_match_gate_m", 0.08)
        # Phase B: apply one follow-up drive covering the measured shortfall
        # when the mismatch exceeds the tolerance. Enable only after audited
        # runs show ~5 mm repeatability. Corrections are capped, and skipped
        # when the match is low-confidence or reports a large rotation.
        self.declare_parameter("lidar_drive_correction", False)
        self.declare_parameter("lidar_correction_tol_m", 0.01)
        self.declare_parameter("lidar_correction_max_m", 0.05)
        # Heading correction from the same scan match. The ICP has always
        # measured dyaw, but only translation was ever driven out — small
        # per-drive twists accumulated until the robot returned to its start
        # spot visibly rotated. Rotate out any confident dyaw beyond tol
        # (and below max, above which the match is suspect).
        self.declare_parameter("lidar_yaw_correction", True)
        self.declare_parameter("lidar_yaw_tol_rad", 0.015)
        self.declare_parameter("lidar_yaw_max_rad", 0.12)
        self.declare_parameter("drive_max_ang_speed_rps", 0.3)
        self.declare_parameter("drive_yaw_tol_rad", 0.01)
        # Side-grasp engagement depth. Perception (Gemini pixel + depth)
        # returns a point on the object's NEAR face; a horizontal gripper
        # whose fingertips stop there leaves the whole object beyond the
        # tips and the jaws close on air (the grasp region runs from the
        # tips back toward the palm). Advance the 04_pick fingertip target
        # this far along the horizontal approach direction so the object
        # body lands between the fingers. Applied only when
        # grasp_tilt_first is set (side grasps); a top-down grasp centers
        # via the bbox instead. ~object depth is a good value.
        self.declare_parameter("grasp_engage_depth_m", 0.03)
        # After an arm pose executes, the joint readback + FK measure where
        # the fingertip actually ended up (servo droop under load shows here).
        # If the error exceeds the tolerance, re-target once with the error
        # subtracted, up to this many iterations. 0 disables. Two iterations:
        # sagging servos deliver only a fraction of each correction (observed
        # +5 mm of a +27 mm request), so one pass leaves a residual.
        self.declare_parameter("pose_correction_iters", 2)
        self.declare_parameter("pose_correction_tol_m", 0.012)
        # Generous default: 1-2 cm for perception z_bottom over-reading the
        # actual table when the bbox is small, ~6 cm for the tip-offset
        # modeling error on this gripper, ~2 cm true clearance.
        self.declare_parameter("pick_z_safety_m", 0.10)
        # Gemini grasp verification after the pick + lift.
        # Empty-grasp gate: after close-until-contact, if the jaws ended up
        # this much more closed (rad) than the expected object stop computed
        # from the measured width, the grasp caught air — fail the pick
        # immediately (no lift/verify). Catches the failure mode where the
        # visual verifier hallucinates success on an empty closed gripper.
        self.declare_parameter("grasp_empty_tol_rad", 0.15)
        # Publish the lidar-detected FRONT WALL (the arena barrier ahead of
        # the robot) as a MoveIt collision object before each pick attempt,
        # so IK/planning keep the arm off it. Wall pose comes straight from
        # the scan (median x of the front-sector points), so it stays
        # correct as the base drives.
        self.declare_parameter("lidar_wall_collision", True)
        self.declare_parameter("lidar_wall_height_m", 0.13)
        # The arena walls stand ON the drive surface (base_footprint z=0);
        # a raised base puts the box exactly in the altitude band the arm
        # crosses when reaching over the wall, vetoing every pick plan.
        self.declare_parameter("lidar_wall_base_z_m", 0.0)
        # base_link -> laser_link x offset from the URDF (laser_joint);
        # scan points must be shifted by it to be true base-frame x.
        self.declare_parameter("lidar_offset_x_m", 0.10478)
        # Hold the arm at a failed (empty) grasp for this long so the
        # scene can be tape-measured against the logged FK fingertip.
        self.declare_parameter("empty_grasp_freeze_sec", 0.0)
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
        # Calibrated against observed close-loop telemetry. Free-motion `err`
        # has been seen between 0.003 and 0.014 depending on Gazebo load and
        # controller jitter; real contact onset reads ~0.018-0.019. 0.018 sits
        # just above the observed quiet-run noise ceiling with a ~1.5x margin
        # and catches the typical onset. Step 1 is exempt (rest-state
        # acceleration transient). The mid-loop step-action-timeout backstop
        # (see step loop below) catches any gradual contact where `err`
        # plateaus below this threshold without ever crossing it.
        self.declare_parameter("close_grip_position_error_threshold_rad", 0.018)
        self.declare_parameter("close_grip_movement_threshold_rad", 0.040)
        self.declare_parameter("close_grip_extra_grip_step_rad", 0.06)
        self.declare_parameter("close_grip_hold_position_offset_rad", 0.0)
        # Each step takes ~1s in the direct-action path; full close from
        # -1.54 to 0.0 at step_size=0.05 is ~31 steps.
        self.declare_parameter("close_grip_timeout_s", 35.0)
        self.declare_parameter("joint_states_topic", "/joint_states")
        # Direct-controller action for per-step gripper close. moveit_py's
        # PlanningSceneMonitor lags badly during back-to-back gripper trajectories
        # (the staleness blew up to 0.78 rad of phantom "start state" mismatch
        # within a few steps), so the close loop talks to the controller directly.
        self.declare_parameter(
            "gripper_action_topic",
            "/gripper_controller/follow_joint_trajectory",
        )
        # Direct-controller action for the start-collision recovery move.
        self.declare_parameter(
            "arm_action_topic",
            "/arm_controller/follow_joint_trajectory",
        )
        # If the initial stow can't plan (typically: the arm woke up parked
        # in a model self-collision, e.g. wrist against the lidar housing —
        # OMPL then rejects every start state), recover by sending a direct
        # joint trajectory to the safe 'up' pose (all zeros) and re-planning
        # once. Same motion as the established manual recovery.
        self.declare_parameter("start_collision_recovery", True)
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
        # Full perception passes (Gemini + pixel projections) are retried on
        # transient failures — depth holes at a projected pixel flicker frame
        # to frame. Each attempt re-calls Gemini, so keep this small.
        self.declare_parameter("perception_attempts", 3)
        self.declare_parameter("perception_retry_delay_sec", 1.0)
        # Auto-reset after a failed pick-and-place: open gripper, drive base
        # back to the initial odom snapshot (= run-start position), park the
        # arm at the named SRDF pose. Lets the user re-run without manually
        # moving the robot back. Set to False for debugging-in-place.
        self.declare_parameter("reset_on_failure", True)
        self.declare_parameter("failure_reset_pose_named", "up")

        self.latest_image = None
        self.latest_base_point = None
        self.base_point_event = threading.Event()
        self.worker_started = False
        self.worker_lock = threading.Lock()
        self.latest_odom = None
        self.odom_event = threading.Event()
        self.latest_joint_state = None
        self._gripper_action_client = None
        self._arm_action_client = None
        self._stow_recovery_attempted = False

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
        self.latest_scan = None
        self._lidar_warned = False
        # Lidar drivers publish /scan best-effort (sensor-data QoS); a
        # default reliable subscription is QoS-incompatible and receives
        # nothing.
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            scan_qos,
        )
        self.pixel_pub = self.create_publisher(PointStamped, pixel_topic, 10)
        # PlanningSceneMonitor listens on 'collision_object' — used to keep
        # the lidar-detected front wall in the planning scene.
        self.collision_pub = self.create_publisher(
            CollisionObject, "collision_object", 10
        )
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

    def scan_callback(self, msg):
        self.latest_scan = msg

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
        if bool(self.get_parameter("ik_probe").value):
            if self.worker_started or self.moveit is None:
                return
            with self.worker_lock:
                if self.worker_started:
                    return
                self.worker_started = True
            threading.Thread(target=self.run_ik_probe, daemon=True).start()
            return
        if not bool(self.get_parameter("auto_start").value):
            return
        if self.worker_started or self.latest_image is None:
            return
        with self.worker_lock:
            if self.worker_started:
                return
            self.worker_started = True
        threading.Thread(target=self.run_once, daemon=True).start()

    def run_ik_probe(self):
        """Scan fx (fy=0, z=ik_probe_z) against every orientation candidate and
        log which combinations solve IK — the arm's real feasibility map."""
        from moveit.core.robot_state import RobotState
        from geometry_msgs.msg import Pose

        arm_name = str(self.get_parameter("arm_group_name").value)
        ee_link = str(self.get_parameter("end_effector_link").value)
        timeout = float(self.get_parameter("ik_search_timeout_sec").value)
        robot_model = self.moveit.get_robot_model()
        tip_offset = [float(v) for v in self.get_parameter("gripper_tip_offset_xyz").value]
        z = float(self.get_parameter("ik_probe_z").value)

        self.get_logger().info(
            f"IK probe: fy=0 z={z:.3f} tip_offset={tip_offset} "
            "orient 0=top-down, 1..5=tilt 0.4/0.8/1.2/1.4/1.57 rad "
            "('!col' = IK ok but in collision)"
        )
        fx = 0.14
        while fx <= 0.421:
            solved = []
            for orient_idx, (qx, qy, qz, qw) in enumerate(self.candidate_orientations(fx, 0.0)):
                ox, oy, oz = rotate_vector_by_quat(tip_offset, qx, qy, qz, qw)
                pose = Pose()
                pose.position.x = fx - ox
                pose.position.y = -oy
                pose.position.z = z - oz
                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw
                state = RobotState(robot_model)
                state.update()
                if state.set_from_ik(arm_name, pose, ee_link, timeout):
                    tag = "" if self.state_is_collision_free(state) else "!col"
                    solved.append(f"{orient_idx}{tag}")
            self.get_logger().info(f"IK probe fx={fx:.2f}: solved=[{','.join(solved) or 'none'}]")
            fx += 0.02
        self.get_logger().info("IK probe complete (no motion was commanded).")

    def run_once(self):
        execute = bool(self.get_parameter("execute").value)
        drive_enabled = bool(self.get_parameter("enable_base_drive").value)
        stow_for_perception = bool(self.get_parameter("stow_for_perception").value)

        if stow_for_perception and self.moveit is not None:
            if not self.plan_and_execute_stow("00_stow_for_perception"):
                self.get_logger().error("could not stow arm; aborting")
                return

        # Anchor scan at the start pose: the failure-reset return refines
        # against this to land back on the exact start spot (the odometry
        # return alone accumulates the whole excursion's error).
        self._start_scan_anchor = self._lidar_scan_points()

        perceived = self.perceive_targets_with_retries()
        if perceived is None:
            self.get_logger().error(
                "initial perception failed; sequence NOT started (node stays "
                "idle — fix the scene/bridge and relaunch)"
            )
            return
        image, plan, target_point, destination_point = perceived
        self.sanitize_destination_z(target_point, destination_point)

        initial_odom = None
        reperceive = bool(self.get_parameter("reperceive_after_drive").value)
        extent = None
        if execute and drive_enabled:
            pick_lift = float(self.get_parameter("pick_lift_m").value)
            initial_odom = self.snapshot_odom()  # may be None in open-loop mode
            if not reperceive:
                # Hardware: the approach drive puts the target inside the
                # camera's ~0.6 m minimum-range blind zone, so neither a
                # re-perception nor a post-drive extent measurement can see
                # it. Measure the object NOW, from the valid pre-drive
                # vantage, and dead-reckon positions through the drive.
                extent = self.measure_object_extent(plan, image)
            # Validate the base offset at BOTH the pre-pick height
            # (target.z + pick_lift) and the actual deepest pick point
            # (target.z + nominal descent). Validating only target.z used to
            # let the base drive to a spot where the real, lower grasp was
            # kinematically unreachable — the pick then failed all IK after
            # a committed drive. The table-floor clamp only raises the pick,
            # so the nominal descent point is the conservative lowest target.
            height_guess = (
                extent[1]
                if extent is not None and extent[1] is not None and extent[1] > 0.0
                else float(self.get_parameter("object_height_fallback_m").value)
            )
            initial_pick_lifts = [
                pick_lift,
                self._grasp_descent_nominal(height_guess),
            ]
            # With re-perception enabled, the first drive is a pure STAGING
            # move: stop while the target is still visible to the depth
            # camera (outside its ~0.6 m min-range blind zone) and centered
            # laterally. Crucially, NO arm-IK feasibility is required at the
            # staging stop — the target is intentionally still out of reach
            # there; the corrected drive after re-perceiving owns
            # reachability. (Requiring IK here made every candidate fail by
            # construction and burned minutes of search.)
            if reperceive:
                min_tx = float(
                    self.get_parameter("reperceive_min_target_x_m").value
                )
                dx_range = list(
                    self.get_parameter("base_search_dx_range_m").value
                )
                dy_range = list(
                    self.get_parameter("base_search_dy_range_m").value
                )
                stage_dx = max(
                    0.0,
                    min(
                        float(target_point.point.x) - min_tx,
                        float(dx_range[1]),
                    ),
                )
                stage_dy = max(
                    float(dy_range[0]),
                    min(float(dy_range[1]), float(target_point.point.y)),
                )
                drive_result = self.drive_staging(
                    target_point, stage_dx, stage_dy, "drive_to_reperceive"
                )
            else:
                drive_result = self.drive_to_feasible(
                    target_point,
                    initial_pick_lifts,
                    "drive_to_reach_target",
                    engage_last_lift=True,
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
            if reperceive:
                perceived = self.perceive_targets_with_retries(
                    require_destination=False
                )
                if perceived is None:
                    return
                # Use refreshed image/plan/target, but DISCARD the re-perceived destination.
                image, plan, target_point, _re_destination = perceived
                self.sanitize_destination_z(target_point, destination_point)
            else:
                # drive_to_feasible already reflected the base move in
                # target_point's coordinates — do NOT subtract again.
                self.get_logger().info(
                    f"target dead-reckoned through drive: "
                    f"({target_point.point.x:.3f},"
                    f"{target_point.point.y:.3f},"
                    f"{target_point.point.z:.3f})"
                )

        # Promote target_point.z to the top of the object so pre-pick lift gives
        # genuine clearance above it, and capture the object height for the pick
        # descent. Without this, the perceived z lands somewhere on the can side
        # and the gripper crashes down on top of it.
        z_top, measured_height, measured_z_bottom = (
            extent
            if extent is not None
            else self.measure_object_extent(plan, image)
        )
        if z_top is not None:
            target_point.point.z = z_top
        object_height = (
            measured_height
            if measured_height is not None and measured_height > 0.0
            else float(self.get_parameter("object_height_fallback_m").value)
        )
        # Determine the table_z used as the pick-fingertip safety floor.
        table_z = self._resolve_table_z(measured_z_bottom)

        # The first drive_to_feasible chose its offset against the *pre-correction*
        # target (raw perception, no z_top fixup). Re-perception shifted xy by a
        # cm or two, and the height correction can raise z by 1-2 cm — enough to
        # flip a barely-feasible IK into infeasible at the arm's reach boundary.
        # Re-verify reachability and nudge the base again if needed.
        if execute and drive_enabled:
            pick_lift = float(self.get_parameter("pick_lift_m").value)
            # Pick at the true deepest point (target.z + nominal descent);
            # pre-pick at target.z + lift.
            corrected_pick_lifts = [
                pick_lift,
                self._grasp_descent_nominal(object_height),
            ]
            drive_result2 = self.drive_to_feasible(
                target_point,
                corrected_pick_lifts,
                "drive_to_reach_target_corrected",
                engage_last_lift=True,
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

            # Failure cleanup: leave the robot in a known state so the user
            # can rerun without manually moving anything. Each step is
            # wrapped so a single reset failure doesn't skip the rest.
            if (
                not success
                and bool(self.get_parameter("reset_on_failure").value)
            ):
                self.get_logger().info(
                    "reset_on_failure enabled; returning robot to known state"
                )
                open_name = str(self.get_parameter("gripper_open_named").value)
                failure_pose = str(
                    self.get_parameter("failure_reset_pose_named").value
                )
                try:
                    self.plan_and_execute_named_gripper(
                        open_name, "failure_reset_open"
                    )
                except Exception as exc:
                    self.get_logger().warn(f"failure_reset_open: {exc}")
                # The wall box is fixed in base_footprint; after the
                # drive-back below it would sit inside the robot and could
                # veto the reset pose plan.
                try:
                    self._remove_front_wall_collision(
                        "failure_reset", "base about to drive back"
                    )
                except Exception as exc:
                    self.get_logger().warn(f"failure_reset_wall_remove: {exc}")
                if initial_odom is not None:
                    try:
                        self.drive_back_to(initial_odom)
                    except Exception as exc:
                        self.get_logger().warn(f"failure_reset_drive_back: {exc}")
                try:
                    self.plan_and_execute_named_arm(
                        failure_pose, "failure_reset_to_up"
                    )
                except Exception as exc:
                    self.get_logger().warn(f"failure_reset_to_up: {exc}")

    def perceive_targets_with_retries(self, require_destination=True):
        """perceive_targets, retried. A single perception pass dies on any
        transient: a depth hole at the target/destination pixel (the bridge
        then never answers the projection and we time out), a rejected
        Gemini response, a dropped service call. Depth holes flicker frame
        to frame, so a fresh attempt usually succeeds. Each attempt re-calls
        Gemini, so attempts are bounded (perception_attempts)."""
        attempts = max(1, int(self.get_parameter("perception_attempts").value))
        delay = float(self.get_parameter("perception_retry_delay_sec").value)
        for i in range(attempts):
            if i > 0:
                self.get_logger().warn(
                    f"perception attempt {i + 1}/{attempts} "
                    f"(previous attempt failed)"
                )
                if delay > 0.0:
                    time.sleep(delay)
            perceived = self.perceive_targets(
                require_destination=require_destination
            )
            if perceived is not None:
                return perceived
        self.get_logger().error(
            f"perception failed after {attempts} attempts"
        )
        return None

    def perceive_targets(self, require_destination=True):
        """Full Gemini + projection pass. require_destination=False tolerates
        a failed/timed-out destination projection (returned as None) — the
        re-perception paths discard the re-perceived destination anyway and
        keep the dead-reckoned one, so its projection failing must not abort
        the retry."""
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
        if bool(self.get_parameter("plane_ranging").value):
            refined = self._plane_range_target(plan, image, target_point)
            if refined is not None:
                target_point = refined
        self._wall_reference_check(target_point)
        destination_point = self.project_pixel(
            "destination", destination_pixel, image.header.frame_id
        )
        if destination_point is None and require_destination:
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
        x_mid = 0.5 * (xmin + xmax)
        bottom_y = ymax - 0.08 * yspan
        bottom_pixel = normalized_point_to_pixel(
            [bottom_y, x_mid], image.width, image.height
        )
        bottom_pt = self.project_pixel(
            "box_bottom", bottom_pixel, image.header.frame_id
        )
        if bottom_pt is None:
            return None, None, None
        z_bottom = float(bottom_pt.point.z)
        # Sample the top at increasing insets: at a shallow viewing angle the
        # 8%-inset ray can graze the object's top edge and land on the table
        # BEHIND it (z_top < z_bottom, huge xy_spread). A deeper inset lands
        # on the object face; height is then slightly under-read, which the
        # grasp math tolerates far better than the full fallback.
        for top_inset in (0.08, 0.25, 0.40):
            top_y = ymin + top_inset * yspan
            top_pixel = normalized_point_to_pixel(
                [top_y, x_mid], image.width, image.height
            )
            top_pt = self.project_pixel(
                "box_top", top_pixel, image.header.frame_id
            )
            if top_pt is None:
                continue
            z_top = float(top_pt.point.z)
            height = z_top - z_bottom
            xy_spread = math.hypot(
                float(top_pt.point.x) - float(bottom_pt.point.x),
                float(top_pt.point.y) - float(bottom_pt.point.y),
            )
            if height < 0.02 or height > 0.30 or xy_spread > 0.08:
                self.get_logger().warn(
                    f"measure_object_extent rejected at top_inset="
                    f"{top_inset:.2f}: z_top={z_top:.3f} "
                    f"z_bottom={z_bottom:.3f} height={height:.3f}m "
                    f"xy_spread={xy_spread:.3f}m"
                )
                continue
            self.get_logger().info(
                f"Measured object extent (top_inset={top_inset:.2f}): "
                f"z_top={z_top:.3f} z_bottom={z_bottom:.3f} "
                f"height={height:.3f}m"
            )
            return z_top, height, z_bottom
        self.get_logger().warn(
            "measure_object_extent: all top insets rejected; falling back"
        )
        return None, None, None

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

    def _verify_show_step(self, verify_show_pose):
        """Strike the 'show' pose before Gemini pick verification. Skipped
        when verification is off; NON-FATAL when the pose can't be planned
        (the SRDF 'show' state trips a base_link<->arm_link3 collision in
        the model on hardware) — verification then just uses the current
        (post-lift) view instead of aborting an already-lifted pick."""
        if not bool(self.get_parameter("verify_pick_with_gemini").value):
            return True
        if not self.plan_and_execute_named_arm(verify_show_pose, "06b_verify_show"):
            self.get_logger().warn(
                "06b_verify_show: 'show' pose unplannable; verifying from "
                "the current pose instead"
            )
        return True

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

    def project_pixel(self, name, pixel, frame_id, plane_z=None):
        """Ask the perception bridge to project one pixel to base frame.
        Retried in place (fresh depth frames arrive continuously, so a bad
        depth read or TF hiccup usually clears within a frame or two) —
        much cheaper than failing the whole perception pass and re-calling
        Gemini. The bridge answers failures with a NaN sentinel (reason on
        the bridge's own log); a timeout means the request/response itself
        was lost."""
        attempts = max(1, int(self.get_parameter("project_attempts").value))
        timeout_sec = float(self.get_parameter("project_timeout_sec").value)
        for i in range(attempts):
            if i > 0:
                time.sleep(0.3)
            self.base_point_event.clear()
            self.latest_base_point = None

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.point.x = float(pixel[0])
            msg.point.y = float(pixel[1])
            # point.z is the bridge's mode switch: > 0 = plane-ranging
            # height in base frame, <= 0 = depth projection.
            msg.point.z = float(plane_z) if plane_z is not None else 0.0
            self.pixel_pub.publish(msg)

            if not self.base_point_event.wait(timeout=timeout_sec):
                self.get_logger().warn(
                    f"Timed out waiting for projected {name} point "
                    f"(attempt {i + 1}/{attempts})"
                )
                continue
            point = deepcopy(self.latest_base_point)
            if math.isnan(float(point.point.x)):
                self.get_logger().warn(
                    f"bridge could not project {name} pixel {pixel} "
                    f"(attempt {i + 1}/{attempts}; reason on the bridge log "
                    "— usually invalid depth at that pixel or a TF hiccup)"
                )
                continue
            self.get_logger().info(
                f"{name} base point: frame={point.header.frame_id} "
                f"x={point.point.x:.3f} y={point.point.y:.3f} "
                f"z={point.point.z:.3f}"
            )
            return point
        self.get_logger().error(
            f"projection of {name} pixel {pixel} failed after "
            f"{attempts} attempts"
        )
        return None

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
        self._last_fk_fingertip = None
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
        if ok:
            self._log_joint_readback(label, trajectory, group_name)
        return ok

    def _log_joint_readback(self, label, trajectory, group_name):
        """Diagnostic for physical grasp misses: once the servos settle after
        an arm execution, log the trajectory's final commanded joint positions
        against the /joint_states readback (per-joint error = servo tracking /
        clamping), then FK the ACTUAL joints to log where the flange and the
        modeled fingertip really are per the URDF. Splits a miss between arm
        tracking error (joint deltas), tool-model error (FK fingertip vs the
        commanded fingertip target), and perception error (FK fingertip on
        target but off the real object)."""
        arm_name = str(self.get_parameter("arm_group_name").value)
        if group_name != arm_name or self.moveit is None:
            return
        try:
            msg = trajectory.get_robot_trajectory_msg()
            names = list(msg.joint_trajectory.joint_names)
            commanded = [
                float(p) for p in msg.joint_trajectory.points[-1].positions
            ]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"[{label}] joint readback: no trajectory msg ({exc})"
            )
            return
        # execute() is async — poll /joint_states until the arm stops moving
        # (3 consecutive stable reads) or the deadline passes. A servo pinned
        # at a clamp reads stable but far from commanded; the deadline covers
        # a readback that never stabilizes.
        deadline = time.monotonic() + 8.0
        time.sleep(0.5)
        prev = None
        stable = 0
        actual = None
        while time.monotonic() < deadline:
            cur = [self._get_joint_position(n) for n in names]
            if any(v is None for v in cur):
                time.sleep(0.25)
                continue
            if prev is not None and max(
                abs(a - b) for a, b in zip(cur, prev)
            ) < 0.004:
                stable += 1
                if stable >= 3:
                    actual = cur
                    break
            else:
                stable = 0
            prev = cur
            time.sleep(0.25)
        if actual is None:
            actual = prev
        if actual is None:
            self.get_logger().warn(
                f"[{label}] joint readback: no /joint_states for {names}"
            )
            return
        detail = ", ".join(
            f"{n}: cmd={c:+.3f} act={a:+.3f} err={a - c:+.3f}"
            for n, c, a in zip(names, commanded, actual)
        )
        self.get_logger().info(f"[{label}] joint readback: {detail}")
        try:
            from moveit.core.robot_state import RobotState

            ee_link = str(self.get_parameter("end_effector_link").value)
            state = RobotState(self.moveit.get_robot_model())
            state.set_joint_group_positions(arm_name, list(actual))
            state.update()
            tf = state.get_global_link_transform(ee_link)
            px, py, pz = float(tf[0, 3]), float(tf[1, 3]), float(tf[2, 3])
            tip = [
                float(v)
                for v in self.get_parameter("gripper_tip_offset_xyz").value
            ]
            fx = px + float(
                tf[0, 0] * tip[0] + tf[0, 1] * tip[1] + tf[0, 2] * tip[2]
            )
            fy = py + float(
                tf[1, 0] * tip[0] + tf[1, 1] * tip[1] + tf[1, 2] * tip[2]
            )
            fz = pz + float(
                tf[2, 0] * tip[0] + tf[2, 1] * tip[1] + tf[2, 2] * tip[2]
            )
            self.get_logger().info(
                f"[{label}] FK(actual joints): flange=({px:.3f},{py:.3f},{pz:.3f}) "
                f"fingertip=({fx:.3f},{fy:.3f},{fz:.3f}) (URDF root frame)"
            )
            self._last_fk_fingertip = (fx, fy, fz)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"[{label}] joint readback FK failed: {exc}")

    def candidate_orientations(self, target_x, target_y):
        """Generate fallback orientations from top-down to tilted, all yawed to face target.
        Baseline (candidate 0) is RPY=(π, 0, yaw) — the empirically-correct
        gripper-points-down on this URDF. Tilts are RPY=(π, -delta, yaw).

        Yaw is measured about the ARM's yaw column (arm_joint1, mounted
        0.09825 m forward of base_footprint), not the base origin. A 5-DOF
        arm can only realize gripper azimuths through its own column; KDL
        solves exact 6D poses, so a yaw off that manifold by even a fraction
        of a degree makes IK fail for every off-axis target.
        """
        arm_x = float(self.get_parameter("arm_base_offset_x_m").value)
        yaw = math.atan2(target_y, target_x - arm_x)
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
        # 1.57 (fully horizontal) is the reach-extending pose for far targets:
        # a 5-DOF arm at full forward extension often has no exact IK solution
        # for near-vertical gripper poses, but does for a horizontal approach.
        for delta_rad in (0.4, 0.8, 1.2, 1.4, 1.57):  # ~23, 46, 69, 80, 90 deg from top-down
            cd = math.cos(delta_rad / 2.0)
            sd = math.sin(delta_rad / 2.0)
            qw = -sd * sy
            qx = cd * cy
            qy = cd * sy
            qz = sd * cy
            n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if n > 1e-9:
                results.append((qx / n, qy / n, qz / n, qw / n))

        if bool(self.get_parameter("grasp_tilt_first").value):
            # Prefer the near-horizontal side-grasp: reverse so the steepest
            # tilt (~1.57 rad, horizontal) is tried first and top-down is the
            # last resort. find_feasible and plan_and_execute share this order,
            # so the validated and executed orientations stay consistent.
            # (Reverse BEFORE the roll expansion below so each orientation's
            # preferred roll twin stays first within its pair.)
            results = results[::-1]

        # Roll every candidate about the gripper's approach axis (local z) to
        # seat the jaws in the grasping plane. The fingertip offset lies along
        # this axis, so the roll leaves the fingertip position invariant (IK
        # re-solves the wrist) — unlike rolling joint5 directly, which swings
        # the offset fingertip through an arc.
        #
        # The wrist realizes these candidates at j5 ~= roll - pi (measured:
        # roll 3.1416 -> j5 0.000, roll 1.5708 -> j5 -1.571, roll 0.0 ->
        # j5 +-pi = OUTSIDE the +-1.5708 servo limit, which failed IK on
        # every candidate at every base offset: 864/864 rejections). A
        # parallel-jaw gripper grasps identically under a 180-deg roll (the
        # fingers swap), so emit BOTH roll and roll+pi for each candidate,
        # trying the twin with the more central predicted wrist angle first.
        # This makes every roll value IK-viable; the roll still selects the
        # physical jaw plane (1.5708 = horizontal jaws for the side grasp).
        roll = float(self.get_parameter("grasp_roll_offset_rad").value)
        variants = []
        for extra in (0.0, math.pi):
            v = roll + extra
            j5_pred = math.atan2(math.sin(v - math.pi), math.cos(v - math.pi))
            variants.append((abs(j5_pred), v))
        variants.sort(key=lambda t: t[0])
        rolled = []
        for (qx, qy, qz, qw) in results:
            for _, v in variants:
                rz = math.sin(v / 2.0)
                rw = math.cos(v / 2.0)
                rolled.append((
                    qx * rw + qy * rz,
                    qy * rw - qx * rz,
                    qw * rz + qz * rw,
                    qw * rw - qz * rz,
                ))
        return rolled

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

    # Forward-reaching IK seeds (arm_joint1..5). KDL returns a single solution
    # near its seed; seeded from the default 'up' (all-zero) pose it fails to
    # find the far-forward, near-horizontal grasp branches that physically
    # exist (verified by hand). Forward seeds first so the common case is fast.
    _IK_SEEDS = (
        (0.0, 0.70, -0.60, -1.30, 0.0),
        (0.0, 1.00, -1.00, -1.20, 0.0),
        (0.0, 0.60, -1.10, -0.80, 1.5),
        (0.0, 0.90, -0.50, -1.50, -1.5),
        (0.0, 0.00, 0.00, 0.00, 0.0),
    )

    def _ik_solve(self, robot_model, arm_name, pose, ee_link, timeout,
                  check_collision=True):
        """IK from several forward-reaching seeds; returns the first
        collision-free RobotState or None. Multiple seeds let KDL find the
        far-forward grasp branch it misses when seeded only from 'up'.

        check_collision=False skips the planning-scene query (state_is_collision_free).
        The base search calls it that way: it only needs reachability, the
        per-candidate collision query is the main source of moveit_py's
        planning-scene-monitor concurrency segfault, and plan_and_execute
        re-checks collision at the actual grasp poses. (On hardware the
        tabletop/octomap are disabled, so the search-time check only caught
        self-collisions anyway.)"""
        from moveit.core.robot_state import RobotState
        # Joint solutions cached from the last feasibility search solve the
        # executed pick/pre-pick poses (same targets) in one fast seeded
        # call instead of grinding the generic seeds x full timeout.
        seeds = tuple(getattr(self, "_search_seed_joints", ())) + self._IK_SEEDS
        for seed in seeds:
            state = RobotState(robot_model)
            try:
                state.set_joint_group_positions(arm_name, list(seed))
            except Exception:  # noqa: BLE001
                pass
            state.update()
            if state.set_from_ik(arm_name, pose, ee_link, timeout):
                if not check_collision or self.state_is_collision_free(state):
                    return state
        return None

    def plan_and_execute_pose(self, pose_stamped, label):
        """Execute a fingertip pose, then close the loop on servo droop: the
        joint readback + FK report where the fingertip actually ended up; if
        it's off by more than pose_correction_tol_m, re-target once with the
        measured error subtracted (biasing the command high/forward so the
        sagged pose lands on the true target)."""
        ok = self._plan_and_execute_pose_once(pose_stamped, label)
        if not ok:
            return False
        iters = int(self.get_parameter("pose_correction_iters").value)
        tol = float(self.get_parameter("pose_correction_tol_m").value)
        tx = float(pose_stamped.pose.position.x)
        ty = float(pose_stamped.pose.position.y)
        tz = float(pose_stamped.pose.position.z)
        for i in range(max(0, iters)):
            fk = getattr(self, "_last_fk_fingertip", None)
            if fk is None:
                break
            ex, ey, ez = fk[0] - tx, fk[1] - ty, fk[2] - tz
            err = math.sqrt(ex * ex + ey * ey + ez * ez)
            if err <= tol:
                break
            corrected = deepcopy(pose_stamped)
            corrected.pose.position.x = tx - ex
            corrected.pose.position.y = ty - ey
            corrected.pose.position.z = tz - ez
            self.get_logger().info(
                f"[{label}] droop correction #{i + 1}: fingertip error "
                f"({ex:+.3f},{ey:+.3f},{ez:+.3f}) |{err:.3f}|m > {tol:.3f}m; "
                f"re-targeting at ({corrected.pose.position.x:.3f},"
                f"{corrected.pose.position.y:.3f},{corrected.pose.position.z:.3f})"
            )
            if not self._plan_and_execute_pose_once(
                corrected, f"{label}_corr{i + 1}"
            ):
                self.get_logger().warn(
                    f"[{label}] droop correction failed to plan/execute; "
                    "keeping the uncorrected pose"
                )
                break
        return True

    def _plan_and_execute_pose_once(self, pose_stamped, label):
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
                f"[{label}] invalid gripper_tip_offset_xyz ({exc}); using [0,0,0.09]"
            )
            tip_offset = [0.0, 0.0, 0.09]

        self.get_logger().info(
            f"[{label}] fingertip target=({fx:.3f},{fy:.3f},{fz:.3f}) "
            f"frame={pose_stamped.header.frame_id} tip_offset_local={tip_offset}"
        )

        candidates = self.candidate_orientations(fx, fy)
        # Try the orientation the base search already validated first —
        # every failed candidate here costs ik_timeout_sec x len(_IK_SEEDS)
        # (~20 s), and the search's winner almost always solves.
        ordered = list(enumerate(candidates))
        preferred = getattr(self, "_preferred_orient_idx", None)
        if preferred is not None and 0 <= preferred < len(ordered):
            ordered.insert(0, ordered.pop(preferred))
        for idx, (qx, qy, qz, qw) in ordered:
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

            state = self._ik_solve(
                robot_model, arm_name, attempt_pose, ee_link, timeout
            )
            if state is None:
                self.get_logger().warn(
                    f"[{label}] IK candidate #{idx} failed (all seeds)"
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
        if (
            not ok
            and not self._stow_recovery_attempted
            and bool(self.get_parameter("start_collision_recovery").value)
        ):
            # A stow that can't plan usually means the arm woke up parked in
            # a model self-collision (e.g. wrist against the lidar housing)
            # and OMPL rejects every start state. Escape with a direct joint
            # move to the safe 'up' pose — the same motion as the manual
            # recovery — then re-plan the stow once.
            self._stow_recovery_attempted = True
            self.get_logger().warn(
                f"[{label}] stow could not plan — start state likely in "
                "collision; recovering with a direct joint move to 'up' "
                "and re-planning once"
            )
            j2_before = self._get_joint_position("arm_joint2")
            if self._send_arm_joints_direct(
                [0.0, 0.0, 0.0, 0.0, 0.0], f"{label}_recovery"
            ):
                time.sleep(1.0)
                j2_after = self._get_joint_position("arm_joint2")
                if (
                    j2_before is not None
                    and j2_after is not None
                    and abs(j2_before) > 0.10
                    and abs(j2_after - j2_before) < 0.03
                ):
                    # The controller reported success but the joint never
                    # moved: the bridge is likely not talking to the servos
                    # at all (seen when serial_port points at the wrong
                    # device — the bridge log then shows
                    # 'STM32 firmware version: -1').
                    self.get_logger().error(
                        f"[{label}] recovery commanded but arm_joint2 did "
                        f"not move ({j2_before:+.3f} -> {j2_after:+.3f}) — "
                        "the servo bridge looks dead. Check the bridge log "
                        "for 'STM32 firmware version: -1' and verify "
                        "serial_port (expansion board = /dev/myserial)."
                    )
                self.arm_component.set_start_state_to_current_state()
                self.arm_component.set_goal_state(robot_state=state)
                ok = self.plan_and_execute(self.arm_component, arm_name, label)
        if ok:
            settle = float(self.get_parameter("stow_settle_sec").value)
            if settle > 0.0:
                time.sleep(settle)
        return ok

    def _send_arm_joints_direct(self, values, label, duration_s=3.5):
        """Send a single-waypoint trajectory for arm_joint1..5 straight to the
        arm controller's FollowJointTrajectory action, bypassing MoveIt. Used
        only for start-collision recovery, where the planner refuses to work
        from the current state at all. Returns True when the action succeeds.
        """
        if self._arm_action_client is None:
            topic = str(self.get_parameter("arm_action_topic").value)
            self._arm_action_client = ActionClient(
                self, FollowJointTrajectory, topic
            )
        if not self._arm_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f"[{label}] arm action server "
                f"{self.get_parameter('arm_action_topic').value!r} unavailable"
            )
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5",
        ]
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in values]
        sec = int(duration_s)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((duration_s - sec) * 1e9)
        goal.trajectory.points.append(point)

        send_future = self._arm_action_client.send_goal_async(goal)
        deadline = time.time() + 5.0
        while rclpy.ok() and not send_future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not send_future.done():
            self.get_logger().error(f"[{label}] send_goal_async timed out")
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"[{label}] arm recovery goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        deadline = time.time() + duration_s + 3.0
        while rclpy.ok() and not result_future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not result_future.done():
            self.get_logger().warn(
                f"[{label}] arm recovery result wait timed out"
            )
            return False
        return True

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

    def rotate_relative_base(self, dyaw_rad):
        """Closed-loop in-place rotation by dyaw_rad on odometry yaw,
        mirroring drive_relative_base's structure. Positive = CCW."""
        if abs(dyaw_rad) < 1e-4:
            return True
        odom0 = self.snapshot_odom()
        if odom0 is None:
            self.get_logger().warn(
                "rotate_relative_base: no odometry; skipping rotation"
            )
            return False
        goal = odom0["yaw"] + float(dyaw_rad)
        kp = float(self.get_parameter("drive_kp").value)
        max_w = float(self.get_parameter("drive_max_ang_speed_rps").value)
        tol = float(self.get_parameter("drive_yaw_tol_rad").value)
        timeout = float(self.get_parameter("drive_timeout_sec").value)
        period = 0.05  # 20 Hz
        t0 = time.time()
        while time.time() - t0 < timeout:
            od = self.snapshot_odom(wait_sec=0.2)
            if od is None:
                break
            err = math.atan2(
                math.sin(goal - od["yaw"]), math.cos(goal - od["yaw"])
            )
            if abs(err) <= tol:
                self.publish_zero_velocity()
                self.get_logger().info(
                    f"rotate_relative_base: arrived "
                    f"(err={math.degrees(err):+.2f}°)"
                )
                return True
            w = max(-max_w, min(max_w, kp * err))
            # Mecanum static friction stalls tiny angular commands; floor
            # the magnitude so the last fraction of a degree still moves.
            if abs(w) < 0.08:
                w = math.copysign(0.08, w)
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "base_link"
            twist.twist.angular.z = w
            self.cmd_vel_pub.publish(twist)
            time.sleep(period)
        self.publish_zero_velocity()
        self.get_logger().warn(
            "rotate_relative_base: stopped before reaching the yaw goal "
            "(timeout or odometry dropout)"
        )
        return False

    def find_feasible_drive_for_point(
        self, point, lift_zs, max_dx=None, engage_last_lift=False
    ):
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
                "using [0,0,0.09]"
            )
            tip_offset = [0.0, 0.0, 0.09]

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
            vals = [lo + i * step for i in range(n)]
            # Always include the range endpoint: with a tight drive envelope
            # the last few cm are often exactly the ones that make IK feasible.
            if vals[-1] < hi - 1e-9:
                vals.append(hi)
            return vals

        if axes_mode == "y_only":
            dx_values = [0.0]
            dy_values = make_range(dy_range[0], dy_range[1], step)
        elif axes_mode == "x_only":
            dx_values = make_range(dx_range[0], dx_range[1], step)
            dy_values = [0.0]
        else:
            dx_values = make_range(dx_range[0], dx_range[1], step)
            dy_values = make_range(dy_range[0], dy_range[1], step)

        # Optional cap on the forward drive (e.g. keep the target outside
        # the camera's blind zone so a post-drive re-perception can see it).
        if max_dx is not None:
            dx_values = [v for v in dx_values if v <= max_dx + 1e-9]
            if not dx_values:
                dx_values = [max(0.0, float(max_dx))]
        # Candidate ordering. "min_drive" (legacy) tries the smallest base
        # motion first. "min_reach" (default) prefers the drive that leaves
        # the target closest to the arm's IDEAL reach (mid-envelope: least
        # droop, best margin) — NOT closest to the column: with a long drive
        # budget, aiming for zero distance parks the target at unreachable
        # candidates first and the search grinds through the whole grid.
        order = str(self.get_parameter("base_search_order").value).lower()
        arm_x = float(self.get_parameter("arm_base_offset_x_m").value)
        ideal = float(self.get_parameter("base_search_ideal_reach_m").value)
        tx = float(point.point.x)
        ty = float(point.point.y)
        if order == "min_drive":
            key = lambda d: d[0] * d[0] + d[1] * d[1]  # noqa: E731
        else:
            # Decomposed reach: aim the FORWARD component at the ideal and
            # keep the target laterally CENTERED. Scoring the scalar
            # (hypot) distance is wrong: when the forward term is already
            # below ideal, the hypotenuse can only reach the ideal by
            # padding with LATERAL offset — the search then strafes the
            # base away from the target (observed on hardware: the robot
            # kept driving right of a left-lying cube, retry after retry).
            key = lambda d: (  # noqa: E731
                abs((tx - d[0] - arm_x) - ideal)
                + 1.5 * abs(ty - d[1]),
                d[0] * d[0] + d[1] * d[1],
            )
        candidates = sorted(
            ((dx, dy) for dx in dx_values for dy in dy_values), key=key
        )

        # Pre-compute orientation list once per (dx, dy) since it depends on (fx_hypo, fy_hypo).
        fx_world = float(point.point.x)
        fy_world = float(point.point.y)
        z_world = float(point.point.z)

        # The executed pick advances the fingertip grasp_engage_depth_m past
        # the perceived near face along the horizontal approach (side
        # grasp), and pre-pick/lift hover directly above that engaged point
        # (vertical descent). ALL lifts must therefore be validated at the
        # engaged point: 3 cm at the reach boundary is the difference
        # between the search blessing a spot and the pick then failing
        # every orientation there (observed: 12/12 IK failures at a
        # search-approved offset).
        engage = 0.0
        if engage_last_lift and bool(self.get_parameter("grasp_tilt_first").value):
            engage = float(self.get_parameter("grasp_engage_depth_m").value)

        ik_fails = 0
        collision_fails = 0
        total_candidates = len(candidates)
        progress_every = max(1, total_candidates // 20)  # ~20 updates total
        for cand_idx, (dx, dy) in enumerate(candidates):
            if cand_idx % progress_every == 0:
                pct = int(100 * cand_idx / total_candidates) if total_candidates else 100
                bar = "#" * (pct // 5) + "-" * (20 - pct // 5)
                self.get_logger().info(
                    f"find_feasible_drive: [{bar}] {pct:3d}% "
                    f"({cand_idx}/{total_candidates}, dx={dx:.3f} dy={dy:.3f})"
                )
            fx = fx_world - dx
            fy = fy_world - dy
            orientations = self.candidate_orientations(fx, fy)
            for orient_idx, (qx, qy, qz, qw) in enumerate(orientations):
                ox, oy, oz = rotate_vector_by_quat(tip_offset, qx, qy, qz, qw)
                # Require this orientation to be valid at every requested lift.
                all_lifts_ok = True
                solutions = []
                for lift_idx, lift in enumerate(lifts):
                    exx, eyy = fx, fy
                    if engage > 0.0:
                        # Engagement advances along the approach yaw, which
                        # is identical for the engaged point (same ray from
                        # the arm column), so the orientations still apply.
                        yaw = math.atan2(fy, fx - arm_x)
                        exx = fx + engage * math.cos(yaw)
                        eyy = fy + engage * math.sin(yaw)
                    fz = z_world + lift
                    wx = exx - ox
                    wy = eyy - oy
                    wz = fz - oz
                    attempt_pose = Pose()
                    attempt_pose.position.x = wx
                    attempt_pose.position.y = wy
                    attempt_pose.position.z = wz
                    attempt_pose.orientation.x = qx
                    attempt_pose.orientation.y = qy
                    attempt_pose.orientation.z = qz
                    attempt_pose.orientation.w = qw

                    state = self._ik_solve(
                        robot_model, arm_name, attempt_pose, ee_link, timeout,
                        check_collision=False,
                    )
                    if state is None:
                        ik_fails += 1
                        all_lifts_ok = False
                        break
                    try:
                        solutions.append(tuple(
                            float(v) for v in
                            state.get_joint_group_positions(arm_name)
                        ))
                    except Exception:  # noqa: BLE001
                        pass
                if all_lifts_ok:
                    self.get_logger().info(
                        f"find_feasible_drive: feasible at dx={dx:.3f} dy={dy:.3f} "
                        f"orient #{orient_idx} after {cand_idx + 1} candidates "
                        f"(lifts={[round(l, 3) for l in lifts]}, "
                        f"engage_last={engage:.3f})"
                    )
                    # Remember which orientation the search validated so the
                    # pick-time candidate loop tries it FIRST — each failed
                    # candidate there costs ik_timeout_sec x len(_IK_SEEDS).
                    self._preferred_orient_idx = orient_idx
                    # And cache the joint solutions: the pick/pre-pick
                    # targets are these exact poses (post-drive), so seeding
                    # IK with them solves in one fast call instead of
                    # re-deriving what the search already proved.
                    self._search_seed_joints = tuple(solutions)
                    return dx, dy, orient_idx
        self.get_logger().warn(
            f"find_feasible_drive: no feasible offset in {len(candidates)} candidates "
            f"(point=({fx_world:.3f},{fy_world:.3f},{z_world:.3f}), "
            f"lifts={[round(l, 3) for l in lifts]}; "
            f"rejections: ik={ik_fails} collision={collision_fails})"
        )
        return None

    def _lidar_scan_points(self):
        """Latest lidar scan as base-frame xy points (polar->xy plus the
        URDF's base_link->laser_link forward offset). Returns an (N,2)
        numpy array or None (no scan / audit disabled)."""
        if not bool(self.get_parameter("lidar_audit").value):
            return None
        scan = self.latest_scan
        if scan is None:
            if not self._lidar_warned:
                self.get_logger().warn(
                    "lidar_audit enabled but no scan received on "
                    f"{self.get_parameter('scan_topic').value} — drive "
                    "audit disabled for this run (is the driver up? "
                    "use_lidar:=true in hardware_moveit.launch.py)"
                )
                self._lidar_warned = True
            return None
        import numpy as np

        rmin = float(self.get_parameter("lidar_min_range_m").value)
        rmax = float(self.get_parameter("lidar_max_range_m").value)
        r = np.asarray(scan.ranges, dtype=float)
        th = scan.angle_min + scan.angle_increment * np.arange(len(r))
        good = np.isfinite(r) & (r >= rmin) & (r <= rmax)
        r, th = r[good], th[good]
        if len(r) < 30:
            return None
        pts = np.stack([r * np.cos(th), r * np.sin(th)], axis=1)
        # laser_link sits ahead of base_link (URDF laser_joint x); shift so
        # the points are true base-frame x. Differential uses (the ICP
        # audit) don't care, but absolute ones (the front-wall fit) do.
        pts[:, 0] += float(self.get_parameter("lidar_offset_x_m").value)
        if len(pts) > 400:
            pts = pts[:: len(pts) // 400 + 1]
        return pts

    def _scan_match(self, pts_before, pts_after, guess_dx, guess_dy):
        """Estimate the robot's true planar motion between two scans via a
        small point-to-point ICP seeded with the odometry delta. A static
        point seen at p_before appears after a motion (R, t) at
        p_after = R^-1 (p_before - t), so aligning R·p_after + t onto
        pts_before recovers (t=translation, R=yaw). Returns
        (dx, dy, dyaw, rms, n_pairs) or None when the match is unreliable
        (too few gated correspondences)."""
        import numpy as np

        gate = float(self.get_parameter("lidar_match_gate_m").value)
        A = np.asarray(pts_after, dtype=float)
        B = np.asarray(pts_before, dtype=float)
        if len(A) < 30 or len(B) < 30:
            return None
        R = np.eye(2)
        t = np.array([float(guess_dx), float(guess_dy)])
        rms = float("inf")
        n_pairs = 0
        for _ in range(12):
            P = A @ R.T + t
            d2 = ((P[:, None, :] - B[None, :, :]) ** 2).sum(axis=2)
            j = d2.argmin(axis=1)
            dmin = np.sqrt(d2[np.arange(len(P)), j])
            mask = dmin < gate
            n_pairs = int(mask.sum())
            if n_pairs < 30:
                return None
            src = A[mask]
            dst = B[j[mask]]
            cs, cd = src.mean(axis=0), dst.mean(axis=0)
            H = (src - cs).T @ (dst - cd)
            U, _, Vt = np.linalg.svd(H)
            Rn = Vt.T @ U.T
            if np.linalg.det(Rn) < 0.0:
                Vt[1, :] *= -1.0
                Rn = Vt.T @ U.T
            tn = cd - Rn @ cs
            converged = np.allclose(Rn, R, atol=1e-7) and np.allclose(
                tn, t, atol=1e-7
            )
            R, t = Rn, tn
            resid = dst - (src @ R.T + t)
            rms = float(np.sqrt((resid**2).sum(axis=1).mean()))
            if converged:
                break
        dyaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
        return float(t[0]), float(t[1]), dyaw, rms, n_pairs

    def _lidar_audit_drive(self, pts_before, applied_dx, applied_dy, label):
        """Compare the just-executed drive's dead-reckoned displacement with
        the lidar scan match; log the mismatch, and (Phase B, gated) issue a
        follow-up drive covering the measured shortfall."""
        if pts_before is None:
            return
        time.sleep(0.3)  # let a fresh post-drive scan arrive
        pts_after = self._lidar_scan_points()
        if pts_after is None:
            return
        match = self._scan_match(pts_before, pts_after, applied_dx, applied_dy)
        if match is None:
            self.get_logger().warn(
                f"[{label}] lidar audit: scan match unreliable "
                "(too few correspondences); trusting odometry"
            )
            return
        mdx, mdy, dyaw, rms, n_pairs = match
        ex, ey = mdx - applied_dx, mdy - applied_dy
        self.get_logger().info(
            f"[{label}] lidar audit: odom (dx={applied_dx:+.3f}, "
            f"dy={applied_dy:+.3f}) | scan-match (dx={mdx:+.3f}, "
            f"dy={mdy:+.3f}, dyaw={math.degrees(dyaw):+.1f}°) | "
            f"mismatch ({ex:+.3f}, {ey:+.3f}) rms={rms:.3f} pairs={n_pairs}"
        )
        if not bool(self.get_parameter("lidar_drive_correction").value):
            return
        confident = rms <= 0.035 and n_pairs >= 100
        # Heading first: the drive intended zero yaw change, so any measured
        # dyaw is real base twist. Rotating it out here keeps per-drive
        # twists from accumulating into a visibly rotated robot (and keeps
        # the dead-reckoned target frames honest — the stored coordinates
        # were computed assuming the heading never changed).
        if confident and bool(self.get_parameter("lidar_yaw_correction").value):
            yaw_tol = float(self.get_parameter("lidar_yaw_tol_rad").value)
            yaw_max = float(self.get_parameter("lidar_yaw_max_rad").value)
            if yaw_tol < abs(dyaw) <= yaw_max:
                self.get_logger().info(
                    f"[{label}] lidar yaw correction: rotating "
                    f"{math.degrees(-dyaw):+.1f}° to restore heading"
                )
                self.rotate_relative_base(-dyaw)
        tol = float(self.get_parameter("lidar_correction_tol_m").value)
        cap = float(self.get_parameter("lidar_correction_max_m").value)
        err = math.hypot(ex, ey)
        if err <= tol:
            return
        # Real-scene match quality: first hardware audits showed rms
        # 0.018-0.022 with ~280 pairs while measuring a consistent,
        # tape-plausible 1 cm odometry shortfall — so the gate sits above
        # that, not at the synthetic-scene ideal.
        if rms > 0.035 or n_pairs < 100 or abs(dyaw) > 0.05:
            self.get_logger().warn(
                f"[{label}] lidar correction skipped: low-confidence match "
                f"(rms={rms:.3f} pairs={n_pairs} "
                f"dyaw={math.degrees(dyaw):+.1f}°)"
            )
            return
        cx = max(-cap, min(cap, -ex))
        cy = max(-cap, min(cap, -ey))
        self.get_logger().info(
            f"[{label}] lidar correction: driving ({cx:+.3f}, {cy:+.3f}) "
            "to cover the measured shortfall"
        )
        self.drive_relative_base(cx, cy)

    def drive_staging(self, point, dx, dy, label):
        """Drive a fixed base displacement with the same bookkeeping as
        drive_to_feasible (axes filtering, point dead-reckoning, lidar
        audit) but WITHOUT any arm-IK feasibility requirement. Used for the
        re-perception staging move, where the target is intentionally left
        outside arm reach (but inside camera view)."""
        axes_mode = str(self.get_parameter("drive_axes").value).lower()
        dx = dx if axes_mode in ("xy", "x_only") else 0.0
        dy = dy if axes_mode in ("xy", "y_only") else 0.0
        self.get_logger().info(
            f"[{label}] staging drive dx={dx:.3f} dy={dy:.3f} "
            "(no IK requirement; re-perception follows)"
        )
        if dx == 0.0 and dy == 0.0:
            return 0.0, 0.0
        lidar_pts_before = self._lidar_scan_points()
        if not self.drive_relative_base(dx, dy):
            return None
        point.point.x = float(point.point.x) - dx
        point.point.y = float(point.point.y) - dy
        self._lidar_audit_drive(lidar_pts_before, dx, dy, label)
        return dx, dy

    def drive_to_feasible(
        self, point, lift_z, label, max_dx=None, engage_last_lift=False
    ):
        # Accept a scalar or an iterable of lifts; the search requires all
        # requested lifts to be feasible & collision-free at the same orientation.
        result = self.find_feasible_drive_for_point(
            point, lift_z, max_dx=max_dx, engage_last_lift=engage_last_lift
        )
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
        lidar_pts_before = self._lidar_scan_points()
        if not self.drive_relative_base(dx, dy):
            return None
        # Reflect the base move in the point's coordinates (now in new base frame).
        axes_mode = str(self.get_parameter("drive_axes").value).lower()
        applied_dx = dx if axes_mode in ("xy", "x_only") else 0.0
        applied_dy = dy if axes_mode in ("xy", "y_only") else 0.0
        self._lidar_audit_drive(lidar_pts_before, applied_dx, applied_dy, label)
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
        divergence = float(self.get_parameter("drive_abort_divergence_m").value)
        period = 0.05  # 20 Hz

        initial_err = math.sqrt(
            (goal_x_w - x0) ** 2 + (goal_y_w - y0) ** 2
        )
        min_err = initial_err
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
            # Divergence abort: if the error GROWS while we drive, the
            # odometry feedback is lying (wrong sign/scale, e.g. bad
            # car_type) and the loop is positive-feedback — stop NOW
            # instead of accelerating into furniture until the timeout.
            min_err = min(min_err, err_norm)
            if divergence > 0.0 and err_norm > min_err + divergence:
                self.get_logger().error(
                    f"drive_relative_base: error diverging ({err_norm:.3f} m, "
                    f"best was {min_err:.3f} m) — odometry feedback is likely "
                    "inverted or mis-scaled (check car_type). Stopping."
                )
                self.publish_zero_velocity()
                return False
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
        ok = self.drive_relative_base(dx_base, dy_base)
        if ok:
            self._lidar_return_to_anchor()
        return ok

    def _lidar_return_to_anchor(self):
        """Absolute return-to-start refinement. The odometry-based return
        accumulates the whole excursion's error — including the motion the
        per-drive lidar corrections added physically, which odometry never
        recorded — so the robot lands off its start spot. Scan-matching the
        current view against the anchor scan captured at run start measures
        the absolute offset from the start pose directly; drive it out (up
        to two refinement passes)."""
        anchor = getattr(self, "_start_scan_anchor", None)
        if anchor is None or not bool(
            self.get_parameter("lidar_drive_correction").value
        ):
            return
        for i in range(2):
            time.sleep(0.3)
            pts_now = self._lidar_scan_points()
            if pts_now is None:
                return
            match = self._scan_match(anchor, pts_now, 0.0, 0.0)
            if match is None:
                self.get_logger().warn(
                    "return-to-anchor: scan match unreliable; leaving the "
                    "odometry-based return as-is"
                )
                return
            dx, dy, dyaw, rms, n_pairs = match
            err = math.hypot(dx, dy)
            self.get_logger().info(
                f"return-to-anchor pass {i + 1}: offset from start "
                f"({dx:+.3f}, {dy:+.3f}, dyaw={math.degrees(dyaw):+.1f}°) "
                f"rms={rms:.3f} pairs={n_pairs}"
            )
            yaw_corr = bool(self.get_parameter("lidar_yaw_correction").value)
            yaw_tol = float(self.get_parameter("lidar_yaw_tol_rad").value)
            yaw_max = float(self.get_parameter("lidar_yaw_max_rad").value)
            yaw_ok = (not yaw_corr) or abs(dyaw) <= yaw_tol
            # The old code returned as soon as POSITION converged, so a
            # well-placed but twisted robot never got its heading fixed.
            if err <= 0.015 and yaw_ok:
                return
            if rms > 0.035 or n_pairs < 100 or err > 0.15 or abs(dyaw) > yaw_max:
                self.get_logger().warn(
                    "return-to-anchor: low-confidence or oversized offset; "
                    "not correcting"
                )
                return
            if not yaw_ok:
                self.get_logger().info(
                    f"return-to-anchor: rotating {math.degrees(-dyaw):+.1f}° "
                    "to restore the start heading"
                )
                self.rotate_relative_base(-dyaw)
            if err > 0.015:
                self.drive_relative_base(-dx, -dy)

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

        self.gripper_component.set_start_state_to_current_state()
        self.gripper_component.set_goal_state(robot_state=state)
        return self.plan_and_execute(self.gripper_component, gripper_name, label)

    def _send_gripper_position_direct(self, target_value, label, duration_s=0.5):
        """Send a single-waypoint trajectory to the gripper controller's
        FollowJointTrajectory action, bypassing MoveIt entirely. The controller
        validates against its own /joint_states reading (no PSM staleness),
        which is what we need for the rapid back-to-back close steps.

        Returns True if the action succeeded, False otherwise.
        """
        if self._gripper_action_client is None:
            topic = str(self.get_parameter("gripper_action_topic").value)
            self._gripper_action_client = ActionClient(
                self, FollowJointTrajectory, topic
            )
        if not self._gripper_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f"[{label}] gripper action server "
                f"{self.get_parameter('gripper_action_topic').value!r} unavailable"
            )
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["grip_joint"]
        point = JointTrajectoryPoint()
        point.positions = [float(target_value)]
        sec = int(duration_s)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((duration_s - sec) * 1e9)
        goal.trajectory.points.append(point)

        send_future = self._gripper_action_client.send_goal_async(goal)
        deadline = time.time() + 5.0
        while rclpy.ok() and not send_future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not send_future.done():
            self.get_logger().error(f"[{label}] send_goal_async timed out")
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f"[{label}] gripper action goal rejected (cmd={target_value:.3f})"
            )
            return False

        result_future = goal_handle.get_result_async()
        deadline = time.time() + duration_s + 2.0
        while rclpy.ok() and not result_future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not result_future.done():
            self.get_logger().warn(
                f"[{label}] gripper action result wait timed out (cmd={target_value:.3f})"
            )
            return False
        return True

    def _close_gripper_until_contact(self, label, expected_grip=None):
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
            step_ok = self._send_gripper_position_direct(
                commanded, f"{label}_step{step_count}", duration_s=0.4
            )
            if not step_ok:
                # The controller timed out trying to reach `commanded`. Mid-range
                # and past step 1, the only reason that happens is the gripper is
                # physically blocked — i.e., contact. Treat the same as a stall
                # detection: declare contact, fall through to hold logic.
                if step_count <= 1:
                    self.get_logger().error(
                        f"[{label}] step {step_count} action failed at cmd={commanded:.3f} "
                        f"during warm-up; treating as fatal (controller not ready)"
                    )
                    return False
                contact = True
                stop_reason = "step action timed out (controller cannot reach commanded)"
                self.get_logger().info(
                    f"[{label}] contact detected after step {step_count} "
                    f"(reason: {stop_reason}); cmd={commanded:.3f}"
                )
                break
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
            # Both stall checks skip step 1 — the joint is accelerating from
            # a standstill at the open limit, so the first step has a larger
            # transient lag (delta low / err high) than any later free-motion
            # step. Without this guard the loop reads first-step settling as
            # contact and quits before getting anywhere near the object.
            stalled_by_movement = step_count > 1 and movement <= move_thresh
            stalled_by_error = step_count > 1 and position_error >= err_thresh
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
        hold_ok = self._send_gripper_position_direct(
            hold, f"{label}_hold", duration_s=0.4
        )
        if not hold_ok:
            # When the joint is in contact with the object, the controller
            # can't physically reach `hold` (which is `commanded + extra_grip`
            # past the contact position), so the FollowJointTrajectory action
            # result never returns within our wait window. The controller is
            # still pushing into the object the whole time — that IS the
            # clamp force we wanted. Treat this as soft success and proceed
            # to lift; the next gripper command (the place-step open) will
            # preempt this still-active hold action cleanly.
            self.get_logger().warn(
                f"[{label}] hold action did not return within timeout — "
                f"controller is likely still pushing into the contact, "
                f"treating as successful grip (cmd={hold:.3f})"
            )
        final_actual = self._get_joint_position("grip_joint")
        self.get_logger().info(
            f"[{label}] final hold position={hold:.3f} "
            f"(actual={final_actual if final_actual is None else f'{final_actual:.3f}'}), "
            f"stop_reason={stop_reason!r}"
        )
        # Empty-grasp gate: a real object stops the jaws near the expected
        # grip position computed from the measured width. If the fingers
        # ended up far MORE closed than that (grip_joint: -1.54 open ->
        # 0.0 closed), they closed on air — the "contact" was the jaws
        # meeting each other. Fail the pick NOW, before lift/verify: the
        # visual verifier has hallucinated success on exactly this state.
        if expected_grip is not None and final_actual is not None:
            tol = float(self.get_parameter("grasp_empty_tol_rad").value)
            if float(final_actual) >= float(expected_grip) + tol:
                self.get_logger().error(
                    f"[{label}] EMPTY GRASP: jaws closed to "
                    f"{final_actual:.3f} rad, past the expected object stop "
                    f"({expected_grip:.3f} + tol {tol:.2f}); nothing between "
                    "the fingers — failing the pick without lift/verify"
                )
                freeze = float(
                    self.get_parameter("empty_grasp_freeze_sec").value
                )
                if freeze > 0.0:
                    fk = self._last_fk_fingertip
                    fk_txt = (
                        f"FK fingertip=({fk[0]:.3f},{fk[1]:.3f},{fk[2]:.3f})"
                        if fk is not None else "FK fingertip unavailable"
                    )
                    self.get_logger().warn(
                        f"[{label}] FREEZING at the failed grasp for "
                        f"{freeze:.0f}s — TAPE-MEASURE NOW: (1) fingertip "
                        "height above the surface the cube sits on, "
                        "(2) horizontal gap fingertip -> cube near face. "
                        f"Model says {fk_txt} (base_footprint; drive "
                        "surface = z 0)."
                    )
                    time.sleep(freeze)
                return False
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

    def _fit_front_wall_x(self):
        """Fit the front wall's base-frame x from the latest scan (narrow
        forward corridor, tight-cluster re-fit). Returns (wall_x, n_points)
        or None. Shared by the collision publisher and the wall-referenced
        x sanity gate. The corridor half-width must stay below the arena's
        side wall distance (~0.21 m) or the side walls pollute the median."""
        pts = self._lidar_scan_points()
        if pts is None:
            return None
        import numpy as np

        sel = pts[
            (np.abs(pts[:, 1]) < 0.15) & (pts[:, 0] > 0.15) & (pts[:, 0] < 1.2)
        ]
        if len(sel) < 15:
            return None
        wall_x = float(np.median(sel[:, 0]))
        # A real wall is a tight x-cluster; re-fit on the points near the
        # median so stray returns (cube edge, corner spill) don't skew it.
        near = sel[np.abs(sel[:, 0] - wall_x) < 0.06]
        if len(near) < 15:
            return None
        return float(np.median(near[:, 0])), len(near)

    def _plane_range_target(self, plan, image, depth_point):
        """Ground-plane ranging for the target's x/y: intersect the bbox
        bottom-center pixel ray with the tape-measured platform plane
        (table_z_m). Depth never enters, so the white cube's depth dropouts
        (which made the bridge's spiral sample neighboring surfaces and
        move x by a scene-dependent amount) cannot corrupt x. z stays on
        the depth path, which is calibrated and accurate. Returns a
        corrected copy of depth_point, or None to keep it unchanged."""
        box = plan.get("target_object", {}).get("box")
        if not box or len(box) != 4:
            self.get_logger().warn(
                "plane ranging: no target bbox from Gemini; keeping depth x/y"
            )
            return None
        table_z = float(self.get_parameter("table_z_m").value)
        ymin, xmin, ymax, xmax = [float(v) for v in box]
        x_mid = 0.5 * (xmin + xmax)
        bottom_pixel = normalized_point_to_pixel(
            [ymax, x_mid], image.width, image.height
        )
        center_pixel = normalized_point_to_pixel(
            [0.5 * (ymin + ymax), x_mid], image.width, image.height
        )
        bottom_pt = self.project_pixel(
            "plane_bottom", bottom_pixel, image.header.frame_id,
            plane_z=table_z,
        )
        if bottom_pt is None:
            self.get_logger().warn(
                "plane ranging: bottom-pixel plane projection failed; "
                "keeping depth x/y"
            )
            return None
        # Cross-check variant: the bbox CENTER sits at ~cube mid-height;
        # the two estimates should agree to ~1 cm when the pitch and plane
        # height are right.
        center_pt = self.project_pixel(
            "plane_center", center_pixel, image.header.frame_id,
            plane_z=table_z + 0.015,
        )
        center_txt = (
            f"{center_pt.point.x:.3f}" if center_pt is not None else "n/a"
        )
        self.get_logger().info(
            f"plane ranging: x_plane_bottom={bottom_pt.point.x:.3f} "
            f"x_plane_center={center_txt} x_depth={depth_point.point.x:.3f} "
            f"(y plane-depth delta {bottom_pt.point.y - depth_point.point.y:+.3f})"
        )
        if abs(bottom_pt.point.x - depth_point.point.x) > 0.05:
            self.get_logger().warn(
                f"plane vs depth x disagree by "
                f"{bottom_pt.point.x - depth_point.point.x:+.3f} m — depth "
                "likely sampled a different surface; trusting the plane"
            )
        refined = deepcopy(depth_point)
        refined.point.x = float(bottom_pt.point.x)
        refined.point.y = float(bottom_pt.point.y)
        return refined

    def _wall_reference_check(self, target_point):
        """Cross-check (and optionally override) the vision x against the
        lidar-fitted front wall. The gap (x_target - x_wall) is invariant to
        robot pose, so it is logged every perception for calibration; with
        wall_to_target_x_m taped in, a gap disagreement beyond
        wall_ref_tol_m warns, and wall_ref_override replaces the vision x
        with the lidar-referenced value (fixed demo placements only)."""
        fit = self._fit_front_wall_x()
        if fit is None:
            return
        wall_x, n_fit = fit
        gap = float(target_point.point.x) - wall_x
        taped = float(self.get_parameter("wall_to_target_x_m").value)
        self.get_logger().info(
            f"wall reference: wall_x={wall_x:.3f} ({n_fit} pts), "
            f"measured target-wall gap={gap:+.3f}"
            + (f", taped gap={taped:+.3f}" if taped >= 0.0 else "")
        )
        if taped < 0.0:
            return
        tol = float(self.get_parameter("wall_ref_tol_m").value)
        x_ref = wall_x + taped
        if abs(gap - taped) > tol:
            self.get_logger().warn(
                f"wall reference: vision x={target_point.point.x:.3f} is "
                f"{gap - taped:+.3f} m off the lidar-referenced "
                f"x={x_ref:.3f} (tol {tol:.3f})"
            )
        if bool(self.get_parameter("wall_ref_override").value):
            self.get_logger().info(
                f"wall reference override: x {target_point.point.x:.3f} -> "
                f"{x_ref:.3f}"
            )
            target_point.point.x = x_ref

    def _remove_front_wall_collision(self, label, reason):
        """Clear any previously published front-wall box. Without this a
        stale wall (fitted before a drive, or mis-fitted) stays in the
        planning scene forever and vetoes every subsequent pick plan."""
        obj = CollisionObject()
        obj.header.frame_id = "base_footprint"
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = "lidar_front_wall"
        obj.operation = CollisionObject.REMOVE
        self.collision_pub.publish(obj)
        self.get_logger().warn(
            f"[{label}] front wall collision object removed: {reason}"
        )

    def _publish_front_wall_collision(self, label):
        """Fit the arena's front wall from the latest lidar scan (median x
        of the points in a narrow forward corridor) and publish it as a
        thin collision box so IK/planning keep the arm and gripper off it.
        In this scene the wall legitimately stands BETWEEN the robot and
        the pick target (it guards the printer screen; the arm reaches
        over it), so a fit in front of the target is expected, not a
        mis-fit. Re-fit per pick attempt, so it tracks the base as it
        drives; when no trustworthy fit exists the previous box is
        REMOVED, never left stale. The corridor half-width must stay
        below the arena's side wall distance (~0.21 m) or the side walls
        pollute the median."""
        if not bool(self.get_parameter("lidar_wall_collision").value):
            return
        fit = self._fit_front_wall_x()
        if fit is None:
            self._remove_front_wall_collision(
                label, "front wall not visible in scan (or fit too scattered)"
            )
            return
        wall_x, n_fit = fit
        if wall_x < 0.24:
            # After deep approach drives the wall can end up hugging the
            # chassis; a box there overlaps the robot's own collision body
            # and puts every start state in collision.
            self._remove_front_wall_collision(
                label, f"wall at x={wall_x:.3f} too close to the chassis "
                "to model as a collision box"
            )
            return
        wall_h = float(self.get_parameter("lidar_wall_height_m").value)
        wall_z0 = float(self.get_parameter("lidar_wall_base_z_m").value)

        obj = CollisionObject()
        obj.header.frame_id = "base_footprint"
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = "lidar_front_wall"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # Width covers the arena interior (~0.42 m); much wider just creates
        # spurious finger-vs-box conflicts when the gripper works beside it.
        box.dimensions = [0.02, 0.5, wall_h]
        from geometry_msgs.msg import Pose as _Pose

        pose = _Pose()
        pose.position.x = wall_x + 0.01
        pose.position.y = 0.0
        pose.position.z = wall_z0 + wall_h / 2.0
        pose.orientation.w = 1.0
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD  # same id -> replaces in scene
        self.collision_pub.publish(obj)
        self.get_logger().info(
            f"[{label}] front wall collision object at x={wall_x:.3f} "
            f"({n_fit} scan points)"
        )

    def _resolve_table_z(self, measured_z_bottom):
        """Table height used as the pick-fingertip safety floor. With
        table_z_source=perception the measured object bottom refines the
        param by a cm or two — but a reading far off the tape-measured
        table_z_m is a depth failure, not a new table, and trusting it
        would command the fingertip below (or way above) the physical
        surface. Such readings fall back to the param."""
        table_z_source = str(self.get_parameter("table_z_source").value).lower()
        table_z_param = float(self.get_parameter("table_z_m").value)
        if table_z_source != "perception":
            return table_z_param
        if measured_z_bottom is None:
            self.get_logger().info(
                f"table_z source=perception unavailable; falling back to "
                f"param table_z_m={table_z_param:.3f}"
            )
            return table_z_param
        table_z = float(measured_z_bottom)
        if abs(table_z - table_z_param) > 0.03:
            self.get_logger().warn(
                f"perceived z_bottom={table_z:.3f} disagrees with "
                f"table_z_m={table_z_param:.3f} by more than 3cm; "
                "distrusting perception and using the param floor"
            )
            return table_z_param
        return table_z

    def _grasp_descent_nominal(self, object_height_m):
        """Nominal (unclamped) pick descent added to target.z: the fingertip
        descends grasp_z_fraction_from_top * object_height below the perceived
        top so the jaws close around the body, plus the manual grasp_z_offset_m.
        Negative = below the perceived surface. The table-floor clamp in
        _clamp_grasp_descent can still raise the final pick above this.
        """
        fraction = float(self.get_parameter("grasp_z_fraction_from_top").value)
        manual = float(self.get_parameter("grasp_z_offset_m").value)
        descent = -manual
        if object_height_m is not None and object_height_m > 0.0 and fraction > 0.0:
            descent -= float(object_height_m) * fraction
        return descent

    def _clamp_grasp_descent(self, target_point, object_height_m, table_z):
        """Compute the pick offset relative to target.z (the object's
        perceived top): the height-scaled auto-descent plus grasp_z_offset_m,
        from _grasp_descent_nominal. The floor clamp can still raise this when
        the resulting pick_z dips below table_z + safety_margin. Returns the
        descent value (added to target.z; negative = below the perceived
        surface); logs the clamp when triggered.
        """
        grasp_descent = self._grasp_descent_nominal(object_height_m)
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
        fraction = float(self.get_parameter("grasp_z_fraction_from_top").value)
        self.get_logger().info(
            f"grasp at target.z + {grasp_descent:.3f}m: "
            f"z_top={float(target_point.point.z):.3f}, "
            f"object_height={object_height_m:.3f}, fraction={fraction:.2f}, "
            f"fingertip_z={float(target_point.point.z) + grasp_descent:.3f}, "
            f"floor={pick_z_min:.3f}"
        )
        return grasp_descent

    def _run_pick_phase(
        self, target_point, grasp_width_m, object_height_m, table_z, target_label
    ):
        home = str(self.get_parameter("home_named").value)
        open_name = str(self.get_parameter("gripper_open_named").value)
        pick_lift = float(self.get_parameter("pick_lift_m").value)
        verify_show_pose = str(self.get_parameter("verify_show_pose_named").value)
        self._publish_front_wall_collision("pick_prep")
        grasp_descent = self._clamp_grasp_descent(
            target_point, object_height_m, table_z
        )
        grip_value = self.grasp_grip_joint(grasp_width_m)

        # Perception hits the object's near face; for a side grasp, push the
        # pick fingertip target past it along the horizontal approach so the
        # body sits between the fingers (see grasp_engage_depth_m). Pre-pick
        # and lift hover directly above this ENGAGED point: the descent is a
        # pure vertical drop with the open jaws straddling the object.
        pick_target = target_point
        engage = float(self.get_parameter("grasp_engage_depth_m").value)
        if engage > 0.0 and bool(self.get_parameter("grasp_tilt_first").value):
            arm_x = float(self.get_parameter("arm_base_offset_x_m").value)
            yaw = math.atan2(
                float(target_point.point.y),
                float(target_point.point.x) - arm_x,
            )
            pick_target = deepcopy(target_point)
            pick_target.point.x += engage * math.cos(yaw)
            pick_target.point.y += engage * math.sin(yaw)
            self.get_logger().info(
                f"side-grasp engagement: pick target advanced {engage:.3f}m "
                f"along approach to ({pick_target.point.x:.3f},"
                f"{pick_target.point.y:.3f})"
            )

        steps = [
            ("01_home", lambda: self.plan_and_execute_named_arm(home, "01_home")),
            ("02_open_gripper",
             lambda: self.plan_and_execute_named_gripper(open_name, "02_open_gripper")),
            # Pre-pick hovers directly ABOVE the engaged pick point so the
            # final 04 move is a pure VERTICAL drop (open jaws straddle the
            # cube on the way down). The old behind-above pre-pick made the
            # descent diagonal, which at the arm's reach boundary couples
            # the axes: the reachable envelope is an arc through the cube,
            # so on-target x came out high and on-target z came out short —
            # never both. A vertical last move locks x in at height (where
            # reach is easy) and spends the descent purely on z.
            ("03_pre_pick",
             lambda: self.plan_and_execute_pose(
                 self.top_down_pose(pick_target, pick_lift), "03_pre_pick")),
            ("04_pick",
             lambda: self.plan_and_execute_pose(
                 self.top_down_pose(pick_target, grasp_descent), "04_pick")),
            ("05_close_gripper",
             lambda: self._close_gripper_until_contact(
                 "05_close_gripper", expected_grip=grip_value)),
            # Lift straight up with the object for the same reason.
            ("06_lift",
             lambda: self.plan_and_execute_pose(
                 self.top_down_pose(pick_target, pick_lift), "06_lift")),
            # Only strike the 'show' pose when Gemini verification will
            # actually look at it. Best-effort either way: the SRDF 'show'
            # state fails planning on hardware (model finds a
            # base_link<->arm_link3 collision), and aborting a pick that
            # already lifted the object just because the presentation pose
            # is unplannable throws away a success — verify from the lift
            # pose instead.
            ("06b_verify_show",
             lambda: self._verify_show_step(verify_show_pose)),
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
            # The wall box is base_footprint-fixed; drop it before the base
            # drives or it goes stale (and can land inside the robot).
            ("06b_drive_to_destination",
             lambda: (
                 self._remove_front_wall_collision(
                     "06b_drive_to_destination", "base about to drive"
                 ),
                 self.drive_to_feasible(
                     destination_point,
                     [place_lift, 0.0],
                     "06b_drive_to_destination",
                 ),
             )[1]),
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

    def _prepare_for_pick_retry(self, destination_point, prev_state=None):
        """Reset state before another pick attempt: open the gripper, stow the
        arm, re-perceive, re-measure the object, and (if drive is enabled)
        nudge the base for the new target. Returns a dict with the refreshed
        (target_point, grasp_width_m, object_height_m, table_z) or None on
        failure. `destination_point` is dead-reckoned through any extra drive.
        When re-perception fails but `prev_state` is available, the retry
        falls back to the last known target position (dead-reckoned and
        lidar-audited through every drive) instead of aborting the run —
        the empty-grasp gate catches the attempt if the object moved.
        """
        last_target_point = prev_state["target_point"] if prev_state else None
        open_name = str(self.get_parameter("gripper_open_named").value)
        if not self.plan_and_execute_named_gripper(open_name, "retry_open_gripper"):
            # Opening at the failed-pick pose can be vetoed by the wall
            # collision box (the finger links sweep near it when the arm
            # hovers over the wall). Pull the arm to stow first — that plan
            # is wall-aware — and open from there. Costs the drop-in-place
            # behavior for a held object, but this path only runs on a
            # FAILED pick, so the jaws are almost certainly empty.
            self.get_logger().warn(
                "retry: open-gripper vetoed at the pick pose (wall box?); "
                "stowing first, then opening"
            )
            if not self.plan_and_execute_stow("retry_stow_for_reperception"):
                return None
            if not self.plan_and_execute_named_gripper(
                open_name, "retry_open_gripper_stowed"
            ):
                return None
        elif not self.plan_and_execute_stow("retry_stow_for_reperception"):
            return None
        # The wall box is fixed in base_footprint; the drives below would
        # leave it stale (eventually inside the robot). The next pick
        # attempt re-fits it from a fresh scan at pick_prep.
        self._remove_front_wall_collision(
            "retry", "base about to drive; wall will be re-fit at pick_prep"
        )
        # The approach drives usually leave the target inside the Astra's
        # ~0.6 m near blind zone; re-perceiving from there yields not_found
        # on a frame where the cube is invisible and kills the whole run.
        # Back off until the last known target sits at re-perception range.
        drive_enabled = bool(self.get_parameter("enable_base_drive").value)
        min_x = float(self.get_parameter("reperceive_min_target_x_m").value)
        if (
            drive_enabled
            and last_target_point is not None
            and float(last_target_point.point.x) < min_x
        ):
            back_dx = float(last_target_point.point.x) - min_x
            applied = self.drive_staging(
                destination_point, back_dx, 0.0, "retry_back_off"
            )
            if applied is None:
                return None
            # Keep the last known target valid in the new base frame too —
            # it is the fallback pick point if re-perception fails below.
            last_target_point.point.x = (
                float(last_target_point.point.x) - applied[0]
            )
            last_target_point.point.y = (
                float(last_target_point.point.y) - applied[1]
            )
        perceived = self.perceive_targets_with_retries(require_destination=False)
        image = plan = None
        if perceived is not None:
            image, plan, target_point, _re_destination = perceived
            # Destination has already been established once and dead-reckoned
            # through the initial drive — keep that, don't trust re-perception
            # of it.
            self.sanitize_destination_z(target_point, destination_point)

            z_top, measured_height, measured_z_bottom = (
                self.measure_object_extent(plan, image)
            )
            if z_top is not None:
                target_point.point.z = z_top
            object_height = (
                measured_height
                if measured_height is not None and measured_height > 0.0
                else float(self.get_parameter("object_height_fallback_m").value)
            )
            table_z = self._resolve_table_z(measured_z_bottom)
        elif prev_state is not None:
            target_point = last_target_point
            object_height = prev_state["object_height_m"]
            table_z = prev_state["table_z"]
            self.get_logger().warn(
                "retry re-perception failed; falling back to the last known "
                f"target position ({target_point.point.x:.3f},"
                f"{target_point.point.y:.3f},{target_point.point.z:.3f}) "
                "dead-reckoned through all drives"
            )
        else:
            return None

        # Re-verify reachability and nudge base if the new target xy/z slipped
        # outside the previously-blessed feasibility window.
        if drive_enabled:
            pick_lift = float(self.get_parameter("pick_lift_m").value)
            corrected_lifts = [
                pick_lift,
                self._grasp_descent_nominal(object_height),
            ]
            drive_result = self.drive_to_feasible(
                target_point,
                corrected_lifts,
                "retry_drive_correction",
                engage_last_lift=True,
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
        if plan is not None:
            grasp_width = self.measure_grasp_width(plan, image)
            target_label = str(plan.get("target_object", {}).get("label", "object"))
        else:
            grasp_width = prev_state["grasp_width_m"]
            target_label = prev_state["target_label"]
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
            refreshed = self._prepare_for_pick_retry(destination_point, pick_state)
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
