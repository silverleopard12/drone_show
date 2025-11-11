#!/usr/bin/env python3
"""
Scenario launch file for RL Planner with Ego-Swarm integration
Follows scenario_swarm.launch.py pattern

Note: RL Planner requires trained policy. Train first using:
  python3 src/planner/rl_planner/scripts/train_ppo.py
  or
  python3 src/planner/rl_planner/scripts/train_mappo.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Map parameters
    map_size_x = LaunchConfiguration('map_size_x', default=10.0)
    map_size_y = LaunchConfiguration('map_size_y', default=100.0)
    map_size_z = LaunchConfiguration('map_size_z', default=50.0)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x)
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y)
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z)
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic)

    use_mockamap = LaunchConfiguration('use_mockamap', default=False)
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap)

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic)

    # RL parameters
    policy_path = LaunchConfiguration('policy_path', default='/tmp/rl_policy.pth')
    policy_path_cmd = DeclareLaunchArgument('policy_path', default_value=policy_path,
                                             description='Path to trained RL policy')

    policy_type = LaunchConfiguration('policy_type', default='ppo')
    policy_type_cmd = DeclareLaunchArgument('policy_type', default_value=policy_type,
                                             description='Policy type: ppo or mappo')

    # Map generator node (optional)
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 10.0},
            {'map/y_size': 100.0},
            {'map/z_size': 50.0},
            {'map/resolution': 0.1},
            {'map/obs_num': 0},
            {'map/circle_num': 0},
        ],
        condition=UnlessCondition(use_mockamap)
    )

    # Drone configurations - 10 drones
    # RL planner uses learned policy for navigation
    drone_configs = [
        {'drone_id': 0, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 85.0, 'target_z': 10.0},
        {'drone_id': 1, 'init_x': 3.0, 'init_y': 13.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 77.0, 'target_z': 10.0},
        {'drone_id': 2, 'init_x': 3.0, 'init_y': 21.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 69.0, 'target_z': 10.0},
        {'drone_id': 3, 'init_x': 3.0, 'init_y': 29.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 61.0, 'target_z': 10.0},
        {'drone_id': 4, 'init_x': 3.0, 'init_y': 37.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 53.0, 'target_z': 10.0},
        {'drone_id': 5, 'init_x': 3.0, 'init_y': 45.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 45.0, 'target_z': 10.0},
        {'drone_id': 6, 'init_x': 3.0, 'init_y': 53.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 37.0, 'target_z': 10.0},
        {'drone_id': 7, 'init_x': 3.0, 'init_y': 61.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 29.0, 'target_z': 10.0},
        {'drone_id': 8, 'init_x': 3.0, 'init_y': 69.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 21.0, 'target_z': 10.0},
        {'drone_id': 9, 'init_x': 3.0, 'init_y': 77.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 13.0, 'target_z': 10.0},
    ]

    # Drone nodes - Each runs RL policy + simulator
    drone_nodes = []

    for config in drone_configs:
        # Note: RL planner uses Python nodes (not C++)
        # Each drone runs inference with the trained policy
        drone_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rl_planner'), 'launch', 'run_in_sim_rl.launch.py')),
            launch_arguments={
                'drone_id': str(config['drone_id']),
                'init_x': str(config['init_x']),
                'init_y': str(config['init_y']),
                'init_z': str(config['init_z']),
                'target_x': str(config['target_x']),
                'target_y': str(config['target_y']),
                'target_z': str(config['target_z']),
                'map_size_x': map_size_x,
                'map_size_y': map_size_y,
                'map_size_z': map_size_z,
                'odom_topic': odom_topic,
                'use_dynamic': use_dynamic,
                'policy_path': policy_path,
                'policy_type': policy_type,
            }.items()
        )
        drone_nodes.append(drone_launch)

    # Swarm Synchronizer
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        get_package_share_directory('ego_planner')))))  # rl_planner might not be built
    swarm_sync_script = os.path.join(project_root, 'scripts', 'swarm_synchronizer.py')

    swarm_synchronizer = ExecuteProcess(
        cmd=['python3', swarm_sync_script, '--ros-args', '-p', f'num_drones:={len(drone_configs)}', '-p', 'wait_time:=5.0'],
        output='screen'
    )

    # Mission Timer
    mission_timer_script = os.path.join(project_root, 'scripts', 'mission_timer.py')

    timer_cmd = ['python3', mission_timer_script, '--ros-args', '-p', f'num_drones:={len(drone_configs)}', '-p', 'arrival_threshold:=0.5']
    for config in drone_configs:
        drone_id = config['drone_id']
        timer_cmd.extend([
            '-p', f'drone_{drone_id}_target_x:={config["target_x"]}',
            '-p', f'drone_{drone_id}_target_y:={config["target_y"]}',
            '-p', f'drone_{drone_id}_target_z:={config["target_z"]}'
        ])

    mission_timer = ExecuteProcess(
        cmd=timer_cmd,
        output='screen'
    )

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(use_mockamap_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(policy_path_cmd)
    ld.add_action(policy_type_cmd)

    # Add map (optional)
    # ld.add_action(map_generator_node)

    # Add synchronizer and timer
    ld.add_action(swarm_synchronizer)
    ld.add_action(mission_timer)

    # Add drone nodes
    for drone in drone_nodes:
        ld.add_action(drone)

    return ld
