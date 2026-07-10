#!/usr/bin/env python3
"""Start the onboard lidar driver for hardware runs.

The physical unit is a YDLidar (brand confirmed on the robot 2026-07-09);
the YDLidar SDK is pre-installed system-wide on the Jetson and Yahboom's
udev rule provides the /dev/ydlidar port symlink. The ROS2 wrapper must be
built into the workspace once:
    git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git src/ydlidar_ros2_driver
    colcon build --packages-select ydlidar_ros2_driver

TODO: set `lidar_baudrate` (and any model-specific driver params) once the
exact YDLidar model is known — X4=128000, T-mini Pro=230400, TG=512000;
`sudo ydlidar_test` usually auto-detects it.

The driver MUST publish with frame_id `laser_frame` (the URDF lidar link,
mounted at base_link xy=(0,0), z=0.0825) so scans are usable in base frame
without extra TF work.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    pkg = LaunchConfiguration("lidar_pkg").perform(context)
    exe = LaunchConfiguration("lidar_exec").perform(context)
    node = Node(
        package=pkg,
        executable=exe,
        name="lidar_driver",
        output="screen",
        parameters=[
            {
                # ydlidar_ros2_driver param names ("port"); "serial_port"
                # kept for other drivers — unknown params are ignored.
                "port": LaunchConfiguration("lidar_serial_port"),
                "serial_port": LaunchConfiguration("lidar_serial_port"),
                "frame_id": LaunchConfiguration("lidar_frame_id"),
                "baudrate": ParameterValue(
                    LaunchConfiguration("lidar_baudrate"), value_type=int
                ),
            }
        ],
        remappings=[("scan", LaunchConfiguration("scan_topic"))],
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_pkg",
                default_value="ydlidar_ros2_driver",
                description="Driver package (physical unit is a YDLidar; "
                "build ydlidar_ros2_driver into the workspace — see module "
                "docstring).",
            ),
            DeclareLaunchArgument(
                "lidar_exec",
                default_value="ydlidar_ros2_driver_node",
                description="Driver executable within lidar_pkg.",
            ),
            DeclareLaunchArgument(
                "lidar_serial_port",
                default_value="/dev/ydlidar",
                description="Lidar serial device (Yahboom's udev rule "
                "provides the /dev/ydlidar symlink).",
            ),
            DeclareLaunchArgument(
                "lidar_baudrate",
                default_value="230400",
                description="TODO: set per YDLidar model (X4=128000, "
                "T-mini Pro=230400, TG=512000) — `sudo ydlidar_test` "
                "auto-detects it.",
            ),
            DeclareLaunchArgument("lidar_frame_id", default_value="laser_frame"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
