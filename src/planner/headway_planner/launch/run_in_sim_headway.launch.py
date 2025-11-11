#!/usr/bin/env python3

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
    plan_file = LaunchConfiguration('plan_file', default='/tmp/headway_planner_output/drone_0.plan')

    # DeclareLaunchArgument definitions
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    init_x_cmd = DeclareLaunchArgument('init_x', default_value=init_x, description='Initial x position of the drone')
    init_y_cmd = DeclareLaunchArgument('init_y', default_value=init_y, description='Initial y position of the drone')
    init_z_cmd = DeclareLaunchArgument('init_z', default_value=init_z, description='Initial z position of the drone')
    target_x_cmd = DeclareLaunchArgument('target_x', default_value=target_x, description='Target x position')
    target_y_cmd = DeclareLaunchArgument('target_y', default_value=target_y, description='Target y position')
    target_z_cmd = DeclareLaunchArgument('target_z', default_value=target_z, description='Target z position')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='ID of the drone')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')
    plan_file_cmd = DeclareLaunchArgument('plan_file', default_value=plan_file, description='Path to .plan file from coordinator')

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic,
                                            description='Use Drone Simulation Considering Dynamics or Not')

    use_pcl_render = LaunchConfiguration('use_pcl_render', default=False)
    use_pcl_render_cmd = DeclareLaunchArgument('use_pcl_render', default_value=use_pcl_render,
                                               description='Use PCL Render Node for Sensing Simulation')

    use_odom_vis = LaunchConfiguration('use_odom_vis', default=True)
    use_odom_vis_cmd = DeclareLaunchArgument('use_odom_vis', default_value=use_odom_vis,
                                             description='Use Odometry Visualization for RViz')

    # Headway Trajectory Executor Node
    # This node will:
    # 1. Wait for coordinator to generate .plan file
    # 2. Parse PX4 .plan file
    # 3. Convert waypoints to trajectory commands
    # 4. Publish to /drone_{id}_planning/pos_cmd
    #
    # TODO: Implement headway_executor_node
    # For now, we use a placeholder that publishes target position
    # Note: Using namespace instead of node name prefix
    # This allows proper topic remapping with drone_id prefix
    headway_executor_node = Node(
        package='headway_planner',
        executable='headway_executor_node',
        name='headway_executor',
        namespace=['drone_', drone_id],
        output='screen',
        parameters=[
            {
                'drone_id': drone_id,
            }
        ],
        remappings=[
            ('odom', ['drone_', drone_id, '_', odom_topic]),
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
        ]
    )

    # Trajectory server node (converts trajectory to pos_cmd)
    # Using ego_planner's traj_server
    ego_planner_share = get_package_share_directory('ego_planner')

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )

    # Include simulator (from ego_planner package)
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

    # Create LaunchDescription and add actions
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
    ld.add_action(plan_file_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_pcl_render_cmd)
    ld.add_action(use_odom_vis_cmd)

    # Add nodes and includes
    ld.add_action(headway_executor_node)
    # Note: traj_server not needed - executor publishes velocity commands directly
    # ld.add_action(traj_server_node)
    ld.add_action(simulator_include)

    return ld
