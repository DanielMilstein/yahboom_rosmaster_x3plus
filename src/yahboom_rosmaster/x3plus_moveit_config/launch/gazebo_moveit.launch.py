from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_rviz = LaunchConfiguration("moveit_rviz")
    headless = LaunchConfiguration("headless")
    world_file = LaunchConfiguration("world_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_pkg = FindPackageShare("yahboom_rosmaster_gazebo")
    moveit_pkg = FindPackageShare("x3plus_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("rosmaster_x3_plus", package_name="x3plus_moveit_config")
        .robot_description(
            mappings={
                "robot_name": "rosmaster_x3_plus",
                "use_gazebo": "true",
            }
        )
        .to_moveit_configs()
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [gazebo_pkg, "launch", "rosmaster_x3_plus.gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "headless": headless,
            "use_rviz": TextSubstitution(text="false"),
            "use_sim_time": use_sim_time,
            "world_file": world_file,
            "load_controllers": "true",
        }.items(),
    )

    joint_state_filter_node = Node(
        package="x3plus_moveit_config",
        executable="filter_moveit_joint_states.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        remappings=[("/joint_states", "/moveit_joint_states")],
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )

    tabletop_scene_node = Node(
        package="x3plus_moveit_config",
        executable="publish_tabletop_scene.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    perception_bridge_node = Node(
        package="x3plus_moveit_config",
        executable="perception_bridge.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "rgb_topic": "/cam_1/color/image_raw",
                "depth_topic": "/cam_1/depth/image_raw",
                "camera_info_topic": "/cam_1/color/camera_info",
                "base_frame": "base_footprint",
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([moveit_pkg, "config", "moveit.rviz"]),
        ],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/joint_states", "/moveit_joint_states")],
        condition=IfCondition(moveit_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("moveit_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_file", default_value="tabletop_can_bin.world"),
            gazebo_launch,
            joint_state_filter_node,
            TimerAction(period=2.0, actions=[rviz_node]),
            TimerAction(period=5.0, actions=[move_group_node]),
            TimerAction(period=8.0, actions=[tabletop_scene_node]),
            TimerAction(period=8.0, actions=[perception_bridge_node]),
        ]
    )
