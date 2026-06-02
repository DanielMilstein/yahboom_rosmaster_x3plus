"""Bring up the Orbbec Astra Pro+ on real hardware.

Includes OrbbecSDK_ROS2's `astra_pro_plus.launch.py` with our overrides:
- depth_registration=true so the depth image is aligned to color (required
  for perception_bridge.py's pixel-indexed depth lookup with Gemini's
  RGB-frame coordinates).
- align_mode=HW for hardware-side alignment (only mode this rev supports).

Topics land under /camera/* (the SDK default namespace).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    depth_registration = LaunchConfiguration("depth_registration")
    align_mode = LaunchConfiguration("align_mode")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    camera_name = LaunchConfiguration("camera_name")

    orbbec_share = FindPackageShare("orbbec_camera")
    astra_pro_plus_launch = PathJoinSubstitution(
        [orbbec_share, "launch", "astra_pro_plus.launch.py"]
    )

    declares = [
        DeclareLaunchArgument("depth_registration", default_value="true"),
        DeclareLaunchArgument("align_mode", default_value="HW"),
        DeclareLaunchArgument("enable_point_cloud", default_value="true"),
        DeclareLaunchArgument("camera_name", default_value="camera"),
    ]

    orbbec = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(astra_pro_plus_launch),
        launch_arguments=[
            ("depth_registration", depth_registration),
            ("align_mode", align_mode),
            ("enable_point_cloud", enable_point_cloud),
            ("camera_name", camera_name),
        ],
    )

    return LaunchDescription(declares + [orbbec])
