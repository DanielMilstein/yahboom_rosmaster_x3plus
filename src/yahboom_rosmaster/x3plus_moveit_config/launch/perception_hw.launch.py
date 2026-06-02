"""Perception-only bringup on real hardware.

Stands up:
- robot_state_publisher driven by the URDF with use_gazebo=false, so the
  chassis static TF (base_footprint -> camera_link, etc.) is in /tf.
- The Orbbec Astra Pro+ driver (via camera.launch.py).
- perception_bridge.py wired to /camera/* with override_intrinsics=false
  so projection uses the camera's own published camera_info.

No MoveIt, no executor, no Gazebo. Sibling to gazebo_moveit.launch.py
but only the perception slice — Phase 2 adds a full hardware_moveit
launch on top of this once the arm + gripper bridge is consolidated.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder("rosmaster_x3_plus", package_name="x3plus_moveit_config")
        .robot_description(
            mappings={
                "robot_name": "rosmaster_x3_plus",
                "use_gazebo": "false",
            }
        )
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("yahboom_rosmaster_hw_bridge"), "launch", "camera.launch.py"]
            )
        ),
    )

    perception_bridge_node = Node(
        package="x3plus_moveit_config",
        executable="perception_bridge.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "rgb_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/depth/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "base_frame": "base_footprint",
                # Real camera publishes calibrated intrinsics in camera_info;
                # no manual override here (unlike the Gazebo path).
                "override_intrinsics": False,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            robot_state_publisher,
            camera_launch,
            perception_bridge_node,
        ]
    )
