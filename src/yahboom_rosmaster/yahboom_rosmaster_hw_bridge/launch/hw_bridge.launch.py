"""Bring up the four hw_bridge nodes.

On the Orin:
  ros2 launch yahboom_rosmaster_hw_bridge hw_bridge.launch.py

The base_driver opens /dev/myserial in live mode (Phase 1). The other
three nodes stay in stub mode until Phase 2 consolidates them into
the same process — only one process can hold the serial port open.

`car_type` defaults to 1 (Yahboom enum: X3). For the X3 PLUS the
correct value may be 2 or 5 depending on firmware rev — relaunch
with car_type:=2 (or 5) if Mecanum teleop drives the wrong direction
or rotates in place when commanded forward.

Other tunables exposed as launch args: serial_port, serial_baud,
serial_debug, publish_odom_tf.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    car_type = LaunchConfiguration("car_type")
    serial_debug = LaunchConfiguration("serial_debug")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")

    common_params = {
        "serial_port": serial_port,
        "serial_baud": serial_baud,
    }
    base_params = {
        **common_params,
        "car_type": car_type,
        "serial_debug": serial_debug,
        "publish_odom_tf": publish_odom_tf,
    }

    declares = [
        DeclareLaunchArgument("serial_port", default_value="/dev/myserial"),
        DeclareLaunchArgument("serial_baud", default_value="115200"),
        DeclareLaunchArgument(
            "car_type",
            default_value="1",
            description="Yahboom kinematics enum. 1=X3, try 2 or 5 for X3 Plus.",
        ),
        DeclareLaunchArgument(
            "serial_debug",
            default_value="false",
            description="Enable Rosmaster_Lib debug=True to dump raw TX/RX bytes.",
        ),
        DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="true",
            description="Publish odom -> base_footprint TF from the base driver.",
        ),
    ]

    nodes = [
        Node(
            package="yahboom_rosmaster_hw_bridge",
            executable="base_driver",
            name="base_driver",
            output="screen",
            parameters=[base_params],
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
