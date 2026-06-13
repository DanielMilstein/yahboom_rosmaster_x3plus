"""Full MoveIt + perception + bridge bringup on real hardware.

Hardware analog of gazebo_moveit.launch.py. Brings up:

- robot_state_publisher from the URDF with use_gazebo=false.
- yahboom_rosmaster_hw_bridge bridge node (cmd_vel, odom, imu,
  /joint_states, arm + gripper action servers).
- Orbbec Astra Pro+ driver via camera.launch.py.
- filter_moveit_joint_states bridging /joint_states -> /moveit_joint_states.
- move_group with the moveit config.
- perception_bridge with hardware topics, override_intrinsics=false.
- rviz with the moveit panel (optional via moveit_rviz arg).

Safety: enable_arm_execution / enable_gripper_execution default to false,
matching hw_bridge.launch.py. Pass them as true once servo_map.yaml is
calibrated and the workspace is clear.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    moveit_rviz = LaunchConfiguration("moveit_rviz")
    tabletop_scene = LaunchConfiguration("tabletop_scene")
    enable_arm_execution = LaunchConfiguration("enable_arm_execution")
    enable_gripper_execution = LaunchConfiguration("enable_gripper_execution")
    car_type = LaunchConfiguration("car_type")
    serial_port = LaunchConfiguration("serial_port")

    moveit_pkg = FindPackageShare("x3plus_moveit_config")
    hw_bridge_pkg = FindPackageShare("yahboom_rosmaster_hw_bridge")

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

    hw_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([hw_bridge_pkg, "launch", "hw_bridge.launch.py"])
        ),
        launch_arguments=[
            ("enable_arm_execution", enable_arm_execution),
            ("enable_gripper_execution", enable_gripper_execution),
            ("car_type", car_type),
            ("serial_port", serial_port),
        ],
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([hw_bridge_pkg, "launch", "camera.launch.py"])
        ),
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
                "trajectory_execution.allowed_start_tolerance": 0.05,
            },
        ],
    )

    # publish_tabletop_scene.py publishes the SIM scene (table slab, can,
    # bin walls) as collision objects pinned to base_footprint. On real
    # hardware those props are fiction and collision-reject valid grasps,
    # so the node is gated off by default here.
    tabletop_scene_node = Node(
        package="x3plus_moveit_config",
        executable="publish_tabletop_scene.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(tabletop_scene),
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
                "override_intrinsics": False,
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
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("moveit_rviz", default_value="true"),
            DeclareLaunchArgument(
                "tabletop_scene",
                default_value="false",
                description="Publish the sim-scene collision props (table/can/bin). "
                "Off by default on hardware — they don't match the real room.",
            ),
            DeclareLaunchArgument(
                "enable_arm_execution",
                default_value="false",
                description="Gate arm execution. Flip to true after servo_map calibration.",
            ),
            DeclareLaunchArgument(
                "enable_gripper_execution",
                default_value="false",
                description="Gate gripper execution. Flip to true after grip_joint calibration.",
            ),
            DeclareLaunchArgument(
                "car_type",
                default_value="2",
                description="Yahboom kinematics enum. 2 = X3 Plus (sweep-verified); "
                "wrong values corrupt command scale AND odometry sign.",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/myserial",
                description="STM32 serial device. Override (e.g. /dev/ttyUSB2) if the "
                "myserial udev symlink drifts after a replug.",
            ),
            robot_state_publisher,
            hw_bridge_launch,
            camera_launch,
            joint_state_filter_node,
            TimerAction(period=2.0, actions=[rviz_node]),
            TimerAction(period=5.0, actions=[move_group_node]),
            TimerAction(period=8.0, actions=[tabletop_scene_node]),
            TimerAction(period=8.0, actions=[perception_bridge_node]),
        ]
    )
