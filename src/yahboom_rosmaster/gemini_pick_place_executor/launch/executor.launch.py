from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    execute = LaunchConfiguration("execute")
    use_sim_time = LaunchConfiguration("use_sim_time")
    task = LaunchConfiguration("task")

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

    executor_node = Node(
        package="gemini_pick_place_executor",
        executable="gemini_pick_place_executor.py",
        name="gemini_pick_place_executor",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "execute": execute,
                "task": task,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "task", default_value="put the red can in the blue bin"
            ),
            executor_node,
        ]
    )
