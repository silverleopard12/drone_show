#!/usr/bin/env python3
"""
Per-drone launch file for RL Planner with simulator integration
Follows run_in_sim.launch.py pattern from Ego-Swarm

Note: This launches a Python-based RL policy inference node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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

    # RL parameters
    policy_path = LaunchConfiguration('policy_path', default='/tmp/rl_policy.pth')
    policy_type = LaunchConfiguration('policy_type', default='ppo')

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
    policy_path_cmd = DeclareLaunchArgument('policy_path', default_value=policy_path)
    policy_type_cmd = DeclareLaunchArgument('policy_type', default_value=policy_type)

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic)

    use_pcl_render = LaunchConfiguration('use_pcl_render', default=False)
    use_pcl_render_cmd = DeclareLaunchArgument('use_pcl_render', default_value=use_pcl_render)

    use_odom_vis = LaunchConfiguration('use_odom_vis', default=True)
    use_odom_vis_cmd = DeclareLaunchArgument('use_odom_vis', default_value=use_odom_vis)

    # RL Policy Inference Node (Python-based)
    # This would be a Python script that loads the policy and publishes commands
    rl_planner_dir = get_package_share_directory('ego_planner')  # Fallback to ego_planner if rl_planner not built
    try:
        rl_planner_dir = get_package_share_directory('rl_planner')
    except:
        pass

    rl_inference_script = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(rl_planner_dir)))),
        'src', 'planner', 'rl_planner', 'scripts', 'inference.py'
    )

    # Launch RL inference as Python process
    rl_inference_node = ExecuteProcess(
        cmd=[
            'python3', rl_inference_script,
            '--drone-id', drone_id,
            '--policy-path', policy_path,
            '--policy-type', policy_type,
            '--target-x', target_x,
            '--target-y', target_y,
            '--target-z', target_z,
            '--odom-topic', ['/drone_', drone_id, '_', odom_topic],
            '--cmd-topic', ['/drone_', drone_id, '_planning/pos_cmd'],
        ],
        output='screen'
    )

    # Alternative: Use ROS2 Node if rl_planner provides one
    # rl_planner_node = Node(
    #     package='rl_planner',
    #     executable='rl_policy_node',
    #     name='rl_policy',
    #     namespace=['drone_', drone_id],
    #     output='screen',
    #     parameters=[{
    #         'drone_id': drone_id,
    #         'policy_path': policy_path,
    #         'policy_type': policy_type,
    #         'target_x': target_x,
    #         'target_y': target_y,
    #         'target_z': target_z,
    #     }],
    #     remappings=[
    #         ('odom', ['drone_', drone_id, '_', odom_topic]),
    #         ('cmd', ['drone_', drone_id, '_planning/pos_cmd']),
    #     ]
    # )

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
    ld.add_action(policy_path_cmd)
    ld.add_action(policy_type_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_pcl_render_cmd)
    ld.add_action(use_odom_vis_cmd)

    # Add nodes
    # Note: rl_inference_node requires trained policy and inference.py script
    # Uncomment when ready:
    # ld.add_action(rl_inference_node)

    ld.add_action(simulator_include)

    return ld
