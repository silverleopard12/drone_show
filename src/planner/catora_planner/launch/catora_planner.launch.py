#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('catora_planner')

    # Declare launch arguments
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='2.0',
        description='Maximum velocity for trajectory generation (m/s)'
    )

    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='2.0',
        description='Maximum acceleration for trajectory generation (m/s^2)'
    )

    trajectory_dt_arg = DeclareLaunchArgument(
        'trajectory_dt',
        default_value='0.2',
        description='Time step for trajectory discretization (s)'
    )

    # Create node
    catora_planner_node = Node(
        package='catora_planner',
        executable='catora_planner_node',
        name='catora_planner',
        output='screen',
        parameters=[{
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'trajectory_dt': LaunchConfiguration('trajectory_dt'),
        }],
        remappings=[
            ('~/get_assignment', '/catora_planner/get_assignment'),
            ('~/get_reshaping_trajectories', '/catora_planner/get_reshaping_trajectories'),
        ]
    )

    return LaunchDescription([
        max_velocity_arg,
        max_acceleration_arg,
        trajectory_dt_arg,
        catora_planner_node
    ])
