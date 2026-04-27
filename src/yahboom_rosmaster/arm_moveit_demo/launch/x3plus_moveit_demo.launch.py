from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    db = LaunchConfiguration("db")
    debug = LaunchConfiguration("debug")

    x3plus_moveit_config_share = FindPackageShare("x3plus_moveit_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("db", default_value="false"),
            DeclareLaunchArgument("debug", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [x3plus_moveit_config_share, "launch", "demo.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "db": db,
                    "debug": debug,
                }.items(),
            ),
        ]
    )
