#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the mecanum wheel robot.

This script creates a launch description that starts the necessary controllers
for operating the mecanum wheel robot in a specific sequence.

Launched Controllers:
    1. Joint State Broadcaster: Publishes joint states to /joint_states
    2. Mecanum Drive Controller: Controls the robot's mecanum drive movements via ~/cmd_vel

:author: Addison Sears-Collins
:date: November 20, 2024
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """Generate a launch description for sequentially starting robot controllers.

    The function creates a launch sequence that ensures controllers are started
    in the correct order.

    Returns:
        LaunchDescription: Launch description containing sequenced controller starts
    """
    robot_name = LaunchConfiguration('robot_name')

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='rosmaster_x3',
        description='Name of the robot/controller configuration to load')

    start_x3_controllers_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            'mecanum_drive_controller',
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '30',
            '--switch-timeout', '30',
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", robot_name, "' != 'rosmaster_x3_plus'"]))
    )

    start_x3_plus_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'joint_state_broadcaster',
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '90',
            '--switch-timeout', '90',
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", robot_name, "' == 'rosmaster_x3_plus'"]))
    )

    start_x3_plus_mecanum_controller_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'mecanum_drive_controller',
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '90',
            '--switch-timeout', '90',
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", robot_name, "' == 'rosmaster_x3_plus'"]))
    )

    start_x3_plus_arm_controller_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'arm_controller',
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '90',
            '--switch-timeout', '90',
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", robot_name, "' == 'rosmaster_x3_plus'"]))
    )

    start_x3_plus_gripper_controller_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner',
            'gripper_controller',
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '90',
            '--switch-timeout', '90',
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", robot_name, "' == 'rosmaster_x3_plus'"]))
    )

    delayed_start = TimerAction(
        period=25.0,
        actions=[start_x3_controllers_cmd, start_x3_plus_joint_state_broadcaster_cmd]
    )
    delayed_mecanum_start = TimerAction(
        period=35.0,
        actions=[start_x3_plus_mecanum_controller_cmd]
    )
    delayed_arm_start = TimerAction(
        period=45.0,
        actions=[start_x3_plus_arm_controller_cmd]
    )
    delayed_gripper_start = TimerAction(
        period=65.0,
        actions=[start_x3_plus_gripper_controller_cmd]
    )

    # Create and populate the launch description
    ld = LaunchDescription()

    # Add the actions to the launch description in sequence
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(delayed_start)
    ld.add_action(delayed_mecanum_start)
    ld.add_action(delayed_arm_start)
    ld.add_action(delayed_gripper_start)

    return ld
