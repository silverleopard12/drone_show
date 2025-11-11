#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Map parameters
    map_size_x = LaunchConfiguration('map_size_x', default=10.0)
    map_size_y = LaunchConfiguration('map_size_y', default=100.0)
    map_size_z = LaunchConfiguration('map_size_z', default=50.0)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    use_mockamap = LaunchConfiguration('use_mockamap', default=False)
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap,
                                              description='Choose map type, map_generator or mockamap')

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic,
                                            description='Use Drone Simulation Considering Dynamics or Not')

    # Headway planner parameters
    scheduler_type = LaunchConfiguration('scheduler_type', default='greedy')
    scheduler_type_cmd = DeclareLaunchArgument('scheduler_type', default_value=scheduler_type,
                                               description='Scheduler type: greedy or milp')

    enable_slotting = LaunchConfiguration('enable_slotting', default='true')
    enable_slotting_cmd = DeclareLaunchArgument('enable_slotting', default_value=enable_slotting,
                                                description='Enable altitude slotting')

    output_dir = LaunchConfiguration('output_dir', default='/tmp/headway_planner_output')
    output_dir_cmd = DeclareLaunchArgument('output_dir', default_value=output_dir,
                                           description='Output directory for plan files')

    # Map generator node (no obstacles for clean swarm)
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
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 0},  # No obstacles
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 0},  # No circular obstacles
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'sensing/radius': 5.0},
            {'sensing/rate': 1.0},
            {'min_distance': 1.2}
        ],
        condition=UnlessCondition(use_mockamap)
    )

    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[
            {'seed': 127},
            {'update_freq': 0.5},
            {'resolution': 0.1},
            {'x_length': 10},
            {'y_length': 100},
            {'z_length': 50},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1}
        ],
        condition=IfCondition(use_mockamap)
    )

    # Drone configurations - 10 drones in line formation
    # Following Ego-Swarm DRONE_CONFIGS pattern
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

    # Headway Coordinator Node - centralized planning
    config_file = PathJoinSubstitution([
        FindPackageShare('headway_planner'),
        'config',
        'default.yaml'
    ])

    headway_coordinator_node = Node(
        package='headway_planner',
        executable='headway_planner_node',
        name='headway_coordinator',
        output='screen',
        parameters=[
            config_file,
            {
                'formation.num_drones': len(drone_configs),
                'scheduler_type': scheduler_type,
                'enable_slotting': enable_slotting,
                'output_dir': output_dir,
                'mission_source': 'formation',  # Use formation-based mission
            }
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Drone nodes - Each runs headway_executor + simulator
    drone_nodes = []

    for config in drone_configs:
        drone_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('headway_planner'), 'launch', 'run_in_sim_headway.launch.py')),
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
                'plan_file': [output_dir, '/drone_', str(config['drone_id']), '.plan']
            }.items()
        )
        drone_nodes.append(drone_launch)

    # Swarm Synchronizer - waits for all drones to spawn, then triggers start
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        get_package_share_directory('headway_planner')))))
    swarm_sync_script = os.path.join(project_root, 'scripts', 'swarm_synchronizer.py')

    swarm_synchronizer = ExecuteProcess(
        cmd=['python3', swarm_sync_script, '--ros-args', '-p', f'num_drones:={len(drone_configs)}', '-p', 'wait_time:=5.0'],
        output='screen'
    )

    # Mission Timer - tracks start time and arrival times
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
    ld.add_action(scheduler_type_cmd)
    ld.add_action(enable_slotting_cmd)
    ld.add_action(output_dir_cmd)

    # Add map nodes (disabled to reduce memory - trajectory visualization only)
    # ld.add_action(map_generator_node)
    # ld.add_action(mockamap_node)

    # Add headway coordinator (centralized planning)
    ld.add_action(headway_coordinator_node)

    # Add swarm synchronizer and mission timer
    ld.add_action(swarm_synchronizer)
    ld.add_action(mission_timer)

    # Add drone nodes
    for drone in drone_nodes:
        ld.add_action(drone)

    return ld
