"""Bring up the four hw_bridge nodes that replace the Gazebo plugin path.

On the Orin:  ros2 launch yahboom_rosmaster_hw_bridge hw_bridge.launch.py
On a dev box: same command — all four nodes come up in stub mode (no
Rosmaster_Lib required), useful for verifying topic / action wiring
before touching hardware.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")

    common_params = {
        "serial_port": serial_port,
        "serial_baud": serial_baud,
    }

    declares = [
        DeclareLaunchArgument("serial_port", default_value="/dev/myserial"),
        DeclareLaunchArgument("serial_baud", default_value="115200"),
    ]

    nodes = [
        Node(
            package="yahboom_rosmaster_hw_bridge",
            executable="base_driver",
            name="base_driver",
            output="screen",
            parameters=[common_params],
        ),
        Node(
            package="yahboom_rosmaster_hw_bridge",
            executable="joint_state_publisher",
            name="hw_joint_state_publisher",
            output="screen",
            parameters=[common_params],
        ),
        Node(
            package="yahboom_rosmaster_hw_bridge",
            executable="arm_action_server",
            name="arm_action_server",
            output="screen",
            parameters=[common_params],
        ),
        Node(
            package="yahboom_rosmaster_hw_bridge",
            executable="gripper_action_server",
            name="gripper_action_server",
            output="screen",
            parameters=[common_params],
        ),
    ]

    return LaunchDescription(declares + nodes)
