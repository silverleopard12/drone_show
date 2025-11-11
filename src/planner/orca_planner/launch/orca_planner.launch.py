#!/usr/bin/env python3
"""
Launch file for ORCA-based trajectory planner
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for ORCA planner"""

    # Package directory
    orca_planner_dir = get_package_share_directory('orca_planner')

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('orca_planner'),
            'config',
            'default.yaml'
        ]),
        description='Path to configuration file'
    )

    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='10',
        description='Number of drones in the swarm'
    )

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='point_to_point',
        description='Mission type: point_to_point, formation, waypoints'
    )

    output_path_arg = DeclareLaunchArgument(
        'output_path',
        default_value='/tmp/orca_trajectories.traj',
        description='Output path for generated trajectories'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # ORCA Planner node
    orca_planner_node = Node(
        package='orca_planner',
        executable='orca_planner_node',
        name='orca_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'num_drones': LaunchConfiguration('num_drones'),
                'mission_type': LaunchConfiguration('mission_type'),
                'output_path': LaunchConfiguration('output_path'),
            }
        ],
        remappings=[
            ('/orca_planner/trajectories', '/trajectories'),
            ('/orca_planner/status', '/planner_status'),
        ]
    )

    # RViz node (conditional)
    def launch_rviz(context, *args, **kwargs):
        rviz_enabled = LaunchConfiguration('rviz').perform(context)

        nodes = []
        if rviz_enabled.lower() == 'true':
            rviz_config_file = os.path.join(
                orca_planner_dir,
                'config',
                'orca_planner.rviz'
            )

            # Check if config exists
            if not os.path.exists(rviz_config_file):
                rviz_config_file = ''

            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file] if rviz_config_file else [],
                output='screen'
            )
            nodes.append(rviz_node)

        return nodes

    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        num_drones_arg,
        mission_type_arg,
        output_path_arg,
        rviz_arg,

        # Nodes
        orca_planner_node,
        OpaqueFunction(function=launch_rviz),
    ])
