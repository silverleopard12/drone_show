#!/usr/bin/env python3
"""
Launch file for DL-based trajectory planner
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for DL planner"""

    # Package directories
    dl_planner_dir = get_package_share_directory('dl_planner')

    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('dl_planner'),
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

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('dl_planner'),
            'models',
            'collision_net.pt'
        ]),
        description='Path to neural network model'
    )

    output_path_arg = DeclareLaunchArgument(
        'output_path',
        default_value='/tmp/drone_show_trajectories.traj',
        description='Output path for generated trajectories'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # DL Planner node
    dl_planner_node = Node(
        package='dl_planner',
        executable='dl_planner_node',
        name='dl_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'num_drones': LaunchConfiguration('num_drones'),
                'model_path': LaunchConfiguration('model_path'),
                'output_path': LaunchConfiguration('output_path'),
            }
        ],
        remappings=[
            # Add remappings if needed
        ]
    )

    # RViz node (conditional)
    def launch_rviz(context, *args, **kwargs):
        rviz_enabled = LaunchConfiguration('rviz').perform(context)

        nodes = []
        if rviz_enabled.lower() == 'true':
            rviz_config_file = os.path.join(
                dl_planner_dir,
                'config',
                'dl_planner.rviz'
            )

            # Check if config exists, otherwise use default
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
        model_path_arg,
        output_path_arg,
        rviz_arg,

        # Nodes
        dl_planner_node,
        OpaqueFunction(function=launch_rviz),
    ])
