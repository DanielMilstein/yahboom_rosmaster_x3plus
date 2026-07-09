"""Per-job launch for a print-removal run.

Includes the pick-and-place executor AND the Gemini bridge node so that both
run inside the gateway's subprocess: the bridge reads GEMINI_API_KEY(_2,_3)
from the environment at node start, which is how API keys configured in the
Autoprint web app reach it (the gateway injects them per job).

Hardware defaults: use_gazebo:=false use_sim_time:=false. Every launch
argument given on the command line (task, execute, any FORWARDED_PARAMS name)
is passed through to the executor's own launch file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    executor_launch = os.path.join(
        get_package_share_directory('gemini_pick_place_executor'),
        'launch',
        'executor.launch.py',
    )
    # Forward every launch configuration set so far (CLI args + our declared
    # defaults) into the executor's launch file.
    forwarded = list(context.launch_configurations.items())

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(executor_launch),
            launch_arguments=forwarded,
        ),
        Node(
            package='gemini_robotics_bridge',
            executable='gemini_robotics_bridge.py',
            name='gemini_robotics_bridge',
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_gazebo', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
