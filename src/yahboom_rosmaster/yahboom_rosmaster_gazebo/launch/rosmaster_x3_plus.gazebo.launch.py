#!/usr/bin/env python3
"""Launch the ROSMASTER X3 Plus Gazebo simulation."""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_pkg = FindPackageShare(package='yahboom_rosmaster_gazebo').find(
        'yahboom_rosmaster_gazebo')
    description_pkg = FindPackageShare(package='yahboom_rosmaster_description')

    urdf_model = PathJoinSubstitution([
        description_pkg,
        'urdf',
        'robots',
        'rosmaster_x3_plus.urdf.xacro'
    ])
    rviz_config_file = PathJoinSubstitution([
        gazebo_pkg,
        'rviz',
        'rosmaster_x3_plus_gazebo_sim.rviz'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'yahboom_rosmaster.gazebo.launch.py')),
            launch_arguments={
                'robot_name': 'rosmaster_x3_plus',
                'rviz_config_file': rviz_config_file,
                'urdf_model': urdf_model,
            }.items()
        )
    ])
