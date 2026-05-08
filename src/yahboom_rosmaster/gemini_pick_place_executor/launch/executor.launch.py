import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


FORWARDED_PARAMS = [
    ("execute", "false"),
    ("task", "put the red can in the blue bin"),
    ("pick_lift_m", "0.04"),
    ("place_lift_m", "0.04"),
    ("gripper_tcp_offset_z", "0.04"),
    ("position_tolerance_m", "0.01"),
    ("orientation_xy_tol_rad", "0.3"),
    ("orientation_z_tol_rad", "3.14"),
    ("use_orientation_constraint", "true"),
    ("ik_timeout_sec", "2.0"),
    ("enable_base_drive", "true"),
    ("sweet_x", "0.18"),
    ("sweet_y", "0.0"),
    ("reach_window_x_min", "0.10"),
    ("reach_window_x_max", "0.25"),
    ("reach_window_y_half", "0.05"),
    ("drive_max_lin_speed_mps", "0.10"),
    ("drive_position_tol_m", "0.01"),
    ("drive_timeout_sec", "15.0"),
    ("return_after_place", "true"),
    ("drive_mode", "auto"),
    ("stow_for_perception", "true"),
    ("restow_after_place", "true"),
    ("top_down_yaw", "0.0"),
    ("planning_time", "5.0"),
    ("velocity_scale", "0.3"),
    ("accel_scale", "0.3"),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_cpp_yaml = os.path.join(
        get_package_share_directory("gemini_pick_place_executor"),
        "config",
        "moveit_cpp.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder("rosmaster_x3_plus", package_name="x3plus_moveit_config")
        .robot_description(
            mappings={
                "robot_name": "rosmaster_x3_plus",
                "use_gazebo": "true",
            }
        )
        .moveit_cpp(file_path=moveit_cpp_yaml)
        .to_moveit_configs()
    )

    forwarded = {name: LaunchConfiguration(name) for name, _ in FORWARDED_PARAMS}

    executor_node = Node(
        package="gemini_pick_place_executor",
        executable="gemini_pick_place_executor.py",
        name="gemini_pick_place_executor",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time, **forwarded},
        ],
    )

    declares = [DeclareLaunchArgument("use_sim_time", default_value="true")]
    declares += [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in FORWARDED_PARAMS
    ]

    return LaunchDescription(declares + [executor_node])
