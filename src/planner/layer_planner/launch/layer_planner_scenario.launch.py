#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('layer_planner')
    config_file = os.path.join(pkg_dir, 'config', 'layer_planner_params.yaml')

    # Default scenario directory (scenarios_FHA in workspace root)
    # This assumes the workspace structure: /home/pjh/ego_swarm/ego-planner-swarm/
    default_scenario_dir = '/home/pjh/ego_swarm/ego-planner-swarm/scenarios_FHA_25'

    # Declare launch arguments
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='10',
        description='Number of drones in the swarm'
    )

    scenario_dir_arg = DeclareLaunchArgument(
        'scenario_dir',
        default_value=default_scenario_dir,
        description='Directory containing scenario node_*.txt files'
    )

    # Layer planner node
    layer_planner_node = Node(
        package='layer_planner',
        executable='layer_planner_node',
        name='layer_planner_node',
        output='screen',
        parameters=[
            config_file,
            {
                'num_drones': LaunchConfiguration('num_drones'),
                'scenario_dir': LaunchConfiguration('scenario_dir')
            }
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        num_drones_arg,
        scenario_dir_arg,
        layer_planner_node
    ])
