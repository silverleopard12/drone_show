#!/usr/bin/env python3
"""
Per-drone launch file for DL Planner with simulator integration
Follows run_in_sim.launch.py pattern from Ego-Swarm
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # LaunchConfiguration definitions
    map_size_x = LaunchConfiguration('map_size_x', default=10.0)
    map_size_y = LaunchConfiguration('map_size_y', default=100.0)
    map_size_z = LaunchConfiguration('map_size_z', default=50.0)
    init_x = LaunchConfiguration('init_x', default=0.0)
    init_y = LaunchConfiguration('init_y', default=0.0)
    init_z = LaunchConfiguration('init_z', default=0.0)
    target_x = LaunchConfiguration('target_x', default=20.0)
    target_y = LaunchConfiguration('target_y', default=20.0)
    target_z = LaunchConfiguration('target_z', default=10.0)
    drone_id = LaunchConfiguration('drone_id', default=0)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')

    # DL parameters
    model_path = LaunchConfiguration('model_path', default='/tmp/dl_collision_net.pt')

    # DeclareLaunchArgument definitions
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x)
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y)
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z)
    init_x_cmd = DeclareLaunchArgument('init_x', default_value=init_x)
    init_y_cmd = DeclareLaunchArgument('init_y', default_value=init_y)
    init_z_cmd = DeclareLaunchArgument('init_z', default_value=init_z)
    target_x_cmd = DeclareLaunchArgument('target_x', default_value=target_x)
    target_y_cmd = DeclareLaunchArgument('target_y', default_value=target_y)
    target_z_cmd = DeclareLaunchArgument('target_z', default_value=target_z)
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id)
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic)
    model_path_cmd = DeclareLaunchArgument('model_path', default_value=model_path)

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic)

    use_pcl_render = LaunchConfiguration('use_pcl_render', default=False)
    use_pcl_render_cmd = DeclareLaunchArgument('use_pcl_render', default_value=use_pcl_render)

    use_odom_vis = LaunchConfiguration('use_odom_vis', default=True)
    use_odom_vis_cmd = DeclareLaunchArgument('use_odom_vis', default_value=use_odom_vis)

    # DL Planner Node
    # Uses neural network for collision prediction
    dl_planner_node = Node(
        package='dl_planner',
        executable='dl_planner_node',
        name='dl_planner',
        namespace=['drone_', drone_id],
        output='screen',
        parameters=[
            {
                'drone_id': drone_id,
                'target_x': target_x,
                'target_y': target_y,
                'target_z': target_z,
                'model_path': model_path,
                'planning_rate': 10.0,  # Hz
                'lookahead_time': 3.0,  # seconds
            }
        ],
        remappings=[
            ('odom', ['drone_', drone_id, '_', odom_topic]),
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            # Subscribe to other drones for collision checking
            ('/swarm/states', '/swarm/agent_states'),
        ]
    )

    # Include simulator (from ego_planner package)
    ego_planner_share = get_package_share_directory('ego_planner')

    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ego_planner_share, 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'use_pcl_render': use_pcl_render,
            'use_odom_vis': use_odom_vis,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': init_x,
            'init_y_': init_y,
            'init_z_': init_z,
            'odometry_topic': odom_topic
        }.items()
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(init_x_cmd)
    ld.add_action(init_y_cmd)
    ld.add_action(init_z_cmd)
    ld.add_action(target_x_cmd)
    ld.add_action(target_y_cmd)
    ld.add_action(target_z_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(model_path_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_pcl_render_cmd)
    ld.add_action(use_odom_vis_cmd)

    # Add nodes
    ld.add_action(dl_planner_node)
    ld.add_action(simulator_include)

    return ld
