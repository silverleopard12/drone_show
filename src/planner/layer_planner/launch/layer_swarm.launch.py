import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition


# Hardcoded drone configurations for 36 drones (FORMATION_36_A -> FORMATION_36_B)
# Init: 6x6 Square Grid, Target: Triangle Formation with Fair Hungarian Allocation
DRONE_CONFIGS = [
    {'drone_id': 0, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 26.1, 'target_z': 16.1},
    {'drone_id': 1, 'init_x': 3.0, 'init_y': 7.5, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 10.1},
    {'drone_id': 2, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 32.1, 'target_z': 12.1},
    {'drone_id': 3, 'init_x': 3.0, 'init_y': 12.5, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 34.1, 'target_z': 16.1},
    {'drone_id': 4, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 36.1, 'target_z': 18.1},
    {'drone_id': 5, 'init_x': 3.0, 'init_y': 17.5, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 38.1, 'target_z': 18.1},
    {'drone_id': 6, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 12.5, 'target_x': 3.0, 'target_y': 24.1, 'target_z': 18.1},
    {'drone_id': 7, 'init_x': 3.0, 'init_y': 7.5, 'init_z': 12.5, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 12.1},
    {'drone_id': 8, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 12.5, 'target_x': 3.0, 'target_y': 32.1, 'target_z': 16.1},
    {'drone_id': 9, 'init_x': 3.0, 'init_y': 12.5, 'init_z': 12.5, 'target_x': 3.0, 'target_y': 34.1, 'target_z': 18.1},
    {'drone_id': 10, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 12.5, 'target_x': 3.0, 'target_y': 36.1, 'target_z': 16.1},
    {'drone_id': 11, 'init_x': 3.0, 'init_y': 17.5, 'init_z': 12.5, 'target_x': 3.0, 'target_y': 36.1, 'target_z': 20.1},
    {'drone_id': 12, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 26.1, 'target_z': 18.1},
    {'drone_id': 13, 'init_x': 3.0, 'init_y': 7.5, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 14.1},
    {'drone_id': 14, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 32.1, 'target_z': 20.1},
    {'drone_id': 15, 'init_x': 3.0, 'init_y': 12.5, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 34.1, 'target_z': 14.1},
    {'drone_id': 16, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 32.1, 'target_z': 14.1},
    {'drone_id': 17, 'init_x': 3.0, 'init_y': 17.5, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 38.1, 'target_z': 20.1},
    {'drone_id': 18, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 17.5, 'target_x': 3.0, 'target_y': 22.1, 'target_z': 18.1},
    {'drone_id': 19, 'init_x': 3.0, 'init_y': 7.5, 'init_z': 17.5, 'target_x': 3.0, 'target_y': 28.1, 'target_z': 12.1},
    {'drone_id': 20, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 17.5, 'target_x': 3.0, 'target_y': 28.1, 'target_z': 14.1},
    {'drone_id': 21, 'init_x': 3.0, 'init_y': 12.5, 'init_z': 17.5, 'target_x': 3.0, 'target_y': 28.1, 'target_z': 18.1},
    {'drone_id': 22, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 17.5, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 18.1},
    {'drone_id': 23, 'init_x': 3.0, 'init_y': 17.5, 'init_z': 17.5, 'target_x': 3.0, 'target_y': 32.1, 'target_z': 18.1},
    {'drone_id': 24, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 26.1, 'target_z': 14.1},
    {'drone_id': 25, 'init_x': 3.0, 'init_y': 7.5, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 20.1, 'target_z': 20.1},
    {'drone_id': 26, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 22.1, 'target_z': 20.1},
    {'drone_id': 27, 'init_x': 3.0, 'init_y': 12.5, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 24.1, 'target_z': 20.1},
    {'drone_id': 28, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 26.1, 'target_z': 20.1},
    {'drone_id': 29, 'init_x': 3.0, 'init_y': 17.5, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 28.1, 'target_z': 20.1},
    {'drone_id': 30, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 22.5, 'target_x': 3.0, 'target_y': 24.1, 'target_z': 16.1},
    {'drone_id': 31, 'init_x': 3.0, 'init_y': 7.5, 'init_z': 22.5, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 20.1},
    {'drone_id': 32, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 22.5, 'target_x': 3.0, 'target_y': 28.1, 'target_z': 16.1},
    {'drone_id': 33, 'init_x': 3.0, 'init_y': 12.5, 'init_z': 22.5, 'target_x': 3.0, 'target_y': 34.1, 'target_z': 20.1},
    {'drone_id': 34, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 22.5, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 16.1},
    {'drone_id': 35, 'init_x': 3.0, 'init_y': 17.5, 'init_z': 22.5, 'target_x': 3.0, 'target_y': 40.1, 'target_z': 20.1},
]

def generate_launch_description():
    # Map parameters
    map_size_x = LaunchConfiguration('map_size_x', default=10.0)
    map_size_y = LaunchConfiguration('map_size_y', default=80.0)
    map_size_z = LaunchConfiguration('map_size_z', default=40.0)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    drone_nodes = []

    # Create simulator and traj_server for each drone
    for config in DRONE_CONFIGS:
        drone_id_val = str(config['drone_id'])

        # Simulator for this drone
        simulator_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')
            ),
            launch_arguments={
                'use_dynamic_cmd': use_dynamic,
                'use_pcl_render': 'False',
                'use_odom_vis': 'True',
                'drone_id': drone_id_val,
                'map_size_x_': map_size_x,
                'map_size_y_': map_size_y,
                'map_size_z_': map_size_z,
                'init_x_': str(config['init_x']),
                'init_y_': str(config['init_y']),
                'init_z_': str(config['init_z']),
                'odometry_topic': odom_topic
            }.items()
        )
        drone_nodes.append(simulator_launch)

        # Trajectory server for this drone
        traj_server_node = Node(
            package='ego_planner',
            executable='traj_server',
            name=f'drone_{drone_id_val}_traj_server',
            output='screen',
            remappings=[
                ('position_cmd', f'drone_{drone_id_val}_planning/pos_cmd'),
                ('planning/bspline', f'drone_{drone_id_val}_planning/bspline')
            ],
            parameters=[
                {'traj_server/time_forward': 1.0}
            ]
        )
        drone_nodes.append(traj_server_node)

    # Layer planner node (single instance for all drones)
    layer_planner_config = os.path.join(
        get_package_share_directory('layer_planner'),
        'config',
        'layer_planner_params.yaml'
    )

    # Build target parameters from DRONE_CONFIGS
    target_params = {
        'num_drones': 36,
        'scenario_dir': ''  # Empty string to use hardcoded targets from parameters
    }

    for config in DRONE_CONFIGS:
        drone_id = config['drone_id']
        target_params[f'drone_{drone_id}_target_x'] = config['target_x']
        target_params[f'drone_{drone_id}_target_y'] = config['target_y']
        target_params[f'drone_{drone_id}_target_z'] = config['target_z']

    layer_planner_node = Node(
        package='layer_planner',
        executable='layer_planner_node',
        name='layer_planner_node',
        output='screen',
        parameters=[
            layer_planner_config,
            target_params
        ],
        emulate_tty=True
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(use_dynamic_cmd)

    # Add layer planner FIRST (so it's ready before traj_servers start)
    ld.add_action(layer_planner_node)

    # Add all drone nodes (simulators + traj_servers) after layer_planner
    for drone_node in drone_nodes:
        ld.add_action(drone_node)

    return ld
