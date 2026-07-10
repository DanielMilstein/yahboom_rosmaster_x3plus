#!/usr/bin/env python3
"""Start the onboard lidar driver for hardware runs.

TODO(Step 0, see plan): the exact driver package/executable depends on the
physical unit shipped with this X3 Plus (URDF models an RPLIDAR S2, but
Yahboom units commonly carry an Oradar MS200 or a YDLidar instead — check
the sticker and `ros2 pkg list | grep -iE 'lidar|sllidar|ydlidar|oradar'`
on the Jetson). Once identified, set `lidar_pkg` / `lidar_exec` /
`lidar_serial_port` defaults below. Until then this launch stays generic:
every knob is a launch argument.

The driver MUST publish with frame_id `laser_frame` (the URDF lidar link,
mounted at base_link xy=(0,0), z=0.0825) so scans are usable in base frame
without extra TF work.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
                # Common param names across sllidar/ydlidar/oradar drivers;
                # unknown params are ignored by drivers that don't use them.
                "serial_port": LaunchConfiguration("lidar_serial_port"),
                "frame_id": LaunchConfiguration("lidar_frame_id"),
                "port_name": LaunchConfiguration("lidar_serial_port"),
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
                default_value="sllidar_ros2",
                description="Driver package (TODO: confirm against the "
                "physical unit — sllidar_ros2 / ydlidar_ros2_driver / "
                "oradar_lidar).",
            ),
            DeclareLaunchArgument(
                "lidar_exec",
                default_value="sllidar_node",
                description="Driver executable within lidar_pkg.",
            ),
            DeclareLaunchArgument(
                "lidar_serial_port",
                default_value="/dev/ttyUSB0",
                description="Lidar serial device. One of the CP2102/CH340 "
                "peripherals in /dev/serial/by-id — must NOT be the STM32's "
                "/dev/myserial.",
            ),
            DeclareLaunchArgument("lidar_frame_id", default_value="laser_frame"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
