"""Bring up the consolidated yahboom_bridge_node.

One process owns /dev/myserial and hosts:
- /cmd_vel → base, /odom + /imu/data_raw publishing.
- /joint_states from servo readbacks at 15 Hz.
- /arm_controller/follow_joint_trajectory action server.
- /gripper_controller/follow_joint_trajectory action server.

Safety: enable_arm_execution and enable_gripper_execution default to
false. Bring up first to verify joint_states, then flip the relevant
gate via launch arg (or `ros2 param set ...`) once servo_map.yaml is
calibrated and the robot is clear.

`car_type` defaults to 1 (Yahboom enum: X3). For the X3 PLUS relaunch
with car_type:=2 (or 5) if Mecanum teleop drives wrong.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("yahboom_rosmaster_hw_bridge")
    default_servo_map = os.path.join(pkg_share, "config", "servo_map.yaml")

    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    car_type = LaunchConfiguration("car_type")
    serial_debug = LaunchConfiguration("serial_debug")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    servo_map_path = LaunchConfiguration("servo_map_path")
    joint_state_rate_hz = LaunchConfiguration("joint_state_rate_hz")
    enable_arm_execution = LaunchConfiguration("enable_arm_execution")
    enable_gripper_execution = LaunchConfiguration("enable_gripper_execution")

    declares = [
        DeclareLaunchArgument("serial_port", default_value="/dev/myserial"),
        DeclareLaunchArgument("serial_baud", default_value="115200"),
        DeclareLaunchArgument(
            "car_type",
            default_value="1",
            description="Yahboom kinematics enum. 1=X3, try 2 or 5 for X3 Plus.",
        ),
        DeclareLaunchArgument("serial_debug", default_value="false"),
        DeclareLaunchArgument("publish_odom_tf", default_value="true"),
        DeclareLaunchArgument(
            "servo_map_path",
            default_value=default_servo_map,
            description="Path to servo_map.yaml (joint name → servo id + rad↔deg map).",
        ),
        DeclareLaunchArgument(
            "joint_state_rate_hz",
            default_value="15.0",
            description="Servo readback rate for /joint_states; drop to 10 Hz if serial is slow.",
        ),
        DeclareLaunchArgument(
            "enable_arm_execution",
            default_value="false",
            description="Gate arm trajectory execution. Keep false until servo_map is calibrated.",
        ),
        DeclareLaunchArgument(
            "enable_gripper_execution",
            default_value="false",
            description="Gate gripper trajectory execution. Keep false until grip_joint is calibrated.",
        ),
    ]

    bridge_node = Node(
        package="yahboom_rosmaster_hw_bridge",
        executable="yahboom_bridge_node",
        name="yahboom_bridge_node",
        output="screen",
        parameters=[
            {
                "serial_port": serial_port,
                "serial_baud": serial_baud,
                "car_type": car_type,
                "serial_debug": serial_debug,
                "publish_odom_tf": publish_odom_tf,
                "servo_map_path": servo_map_path,
                "joint_state_rate_hz": joint_state_rate_hz,
                "enable_arm_execution": enable_arm_execution,
                "enable_gripper_execution": enable_gripper_execution,
            }
        ],
    )

    return LaunchDescription(declares + [bridge_node])
