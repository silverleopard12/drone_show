#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for headway_planner"""

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('headway_planner'),
            'config',
            'default.yaml'
        ]),
        description='Path to configuration file'
    )

    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='10',
        description='Number of drones'
    )

    scheduler_type_arg = DeclareLaunchArgument(
        'scheduler_type',
        default_value='greedy',
        description='Scheduler type: greedy or milp'
    )

    enable_slotting_arg = DeclareLaunchArgument(
        'enable_slotting',
        default_value='true',
        description='Enable altitude slotting'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/headway_planner_output',
        description='Output directory for .plan files'
    )

    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='',
        description='Path to mission YAML file (if mission_source==yaml)'
    )

    # Node definition
    headway_planner_node = Node(
        package='headway_planner',
        executable='headway_planner_node',
        name='headway_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'formation.num_drones': LaunchConfiguration('num_drones'),
                'scheduler_type': LaunchConfiguration('scheduler_type'),
                'enable_slotting': LaunchConfiguration('enable_slotting'),
                'output_dir': LaunchConfiguration('output_dir'),
                'mission_file': LaunchConfiguration('mission_file'),
            }
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        config_file_arg,
        num_drones_arg,
        scheduler_type_arg,
        enable_slotting_arg,
        output_dir_arg,
        mission_file_arg,
        headway_planner_node,
    ])
