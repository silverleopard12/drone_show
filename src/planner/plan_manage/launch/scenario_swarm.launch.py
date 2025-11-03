import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Map parameters - increased for expanded 36-drone formation (Y: 5~46, Z: 10~35)
    map_size_x = LaunchConfiguration('map_size_x', default = 10.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 100.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 50.0)
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    # Scenario parameters
    # scenario_dir = LaunchConfiguration('scenario_dir', default='scenarios')
    # num_drones = LaunchConfiguration('num_drones', default=10)

    # scenario_dir_cmd = DeclareLaunchArgument('scenario_dir', default_value=scenario_dir, description='Directory containing scenario files')
    # num_drones_cmd = DeclareLaunchArgument('num_drones', default_value=num_drones, description='Number of drones')

    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    # Map generator node
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
        condition = UnlessCondition(use_mockamap)
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
        condition = IfCondition(use_mockamap)
    )

    # Scenario publisher node - DISABLED for hardcoded preset target test
    # No need for scenario_publisher when using PRESET_TARGET mode with hardcoded assignments
    # from launch.actions import ExecuteProcess
    # scenario_publisher_node = ExecuteProcess(...)

    # Drone configurations - 36 drones
    # Init: 6x6 Grid formation (FORMATION_36_A) - Y spacing 8.0m, Z spacing 5.0m for better collision avoidance
    # Target: Triangle formation (FORMATION_36_B) with Fair Hungarian Allocation (3.2m spacing)
    drone_configs = [
        {'drone_id': 0, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 20.5, 'target_z': 19.7},
        {'drone_id': 1, 'init_x': 3.0, 'init_y': 13.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 26.9, 'target_z': 19.7},
        {'drone_id': 2, 'init_x': 3.0, 'init_y': 21.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 13.3},
        {'drone_id': 3, 'init_x': 3.0, 'init_y': 29.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 10.1},
        {'drone_id': 4, 'init_x': 3.0, 'init_y': 37.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 33.3, 'target_z': 13.3},
        {'drone_id': 5, 'init_x': 3.0, 'init_y': 45.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 36.5, 'target_z': 16.5},
        {'drone_id': 6, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 23.7, 'target_z': 16.5},
        {'drone_id': 7, 'init_x': 3.0, 'init_y': 13.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 26.9, 'target_z': 13.3},
        {'drone_id': 8, 'init_x': 3.0, 'init_y': 21.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 33.3, 'target_z': 19.7},
        {'drone_id': 9, 'init_x': 3.0, 'init_y': 29.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 16.5},
        {'drone_id': 10, 'init_x': 3.0, 'init_y': 37.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 33.3, 'target_z': 16.5},
        {'drone_id': 11, 'init_x': 3.0, 'init_y': 45.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 39.7, 'target_z': 19.7},
        {'drone_id': 12, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 20.5, 'target_z': 22.9},
        {'drone_id': 13, 'init_x': 3.0, 'init_y': 13.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 26.9, 'target_z': 16.5},
        {'drone_id': 14, 'init_x': 3.0, 'init_y': 21.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 23.7, 'target_z': 19.7},
        {'drone_id': 15, 'init_x': 3.0, 'init_y': 29.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 19.7},
        {'drone_id': 16, 'init_x': 3.0, 'init_y': 37.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 36.5, 'target_z': 19.7},
        {'drone_id': 17, 'init_x': 3.0, 'init_y': 45.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 39.7, 'target_z': 22.9},
        {'drone_id': 18, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 17.3, 'target_z': 22.9},
        {'drone_id': 19, 'init_x': 3.0, 'init_y': 13.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 23.7, 'target_z': 22.9},
        {'drone_id': 20, 'init_x': 3.0, 'init_y': 21.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 26.9, 'target_z': 22.9},
        {'drone_id': 21, 'init_x': 3.0, 'init_y': 29.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 22.9},
        {'drone_id': 22, 'init_x': 3.0, 'init_y': 37.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 36.5, 'target_z': 22.9},
        {'drone_id': 23, 'init_x': 3.0, 'init_y': 45.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 42.9, 'target_z': 22.9},
        {'drone_id': 24, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 14.1, 'target_z': 26.1},
        {'drone_id': 25, 'init_x': 3.0, 'init_y': 13.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 20.5, 'target_z': 26.1},
        {'drone_id': 26, 'init_x': 3.0, 'init_y': 21.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 26.9, 'target_z': 26.1},
        {'drone_id': 27, 'init_x': 3.0, 'init_y': 29.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 33.3, 'target_z': 22.9},
        {'drone_id': 28, 'init_x': 3.0, 'init_y': 37.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 36.5, 'target_z': 26.1},
        {'drone_id': 29, 'init_x': 3.0, 'init_y': 45.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 46.1, 'target_z': 26.1},
        {'drone_id': 30, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 17.3, 'target_z': 26.1},
        {'drone_id': 31, 'init_x': 3.0, 'init_y': 13.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 23.7, 'target_z': 26.1},
        {'drone_id': 32, 'init_x': 3.0, 'init_y': 21.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 30.1, 'target_z': 26.1},
        {'drone_id': 33, 'init_x': 3.0, 'init_y': 29.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 33.3, 'target_z': 26.1},
        {'drone_id': 34, 'init_x': 3.0, 'init_y': 37.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 39.7, 'target_z': 26.1},
        {'drone_id': 35, 'init_x': 3.0, 'init_y': 45.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 42.9, 'target_z': 26.1},
    ]

    drone_nodes = []

    for config in drone_configs:
        drone_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ego_planner'), 'launch', 'run_in_sim_minimal.launch.py')),
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
                'odom_topic': odom_topic
                # Using minimal launch file - no visualization, no dynamics
            }.items()
        )
        drone_nodes.append(drone_launch)

    # Swarm Synchronizer - waits for all drones to spawn, then triggers start
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        get_package_share_directory('ego_planner')))))
    swarm_sync_script = os.path.join(project_root, 'scripts', 'swarm_synchronizer.py')

    swarm_synchronizer = ExecuteProcess(
        cmd=['python3', swarm_sync_script, '--ros-args', '-p', 'num_drones:=36', '-p', 'wait_time:=5.0'],
        output='screen'
    )

    # Mission Timer - tracks start time and arrival times
    mission_timer_script = os.path.join(project_root, 'scripts', 'mission_timer.py')

    # Build parameter list for mission timer
    timer_cmd = ['python3', mission_timer_script, '--ros-args', '-p', 'num_drones:=36', '-p', 'arrival_threshold:=0.5']
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
    # ld.add_action(scenario_dir_cmd)
    # ld.add_action(num_drones_cmd)
    ld.add_action(use_mockamap_cmd)
    ld.add_action(use_dynamic_cmd)

    # Add map nodes (disabled to reduce memory usage - trajectory visualization only)
    # ld.add_action(map_generator_node)
    # ld.add_action(mockamap_node)

    # Add swarm synchronizer (will exit after trigger) and mission timer
    ld.add_action(swarm_synchronizer)
    ld.add_action(mission_timer)

    # Add drone nodes
    for drone in drone_nodes:
        ld.add_action(drone)

    # Scenario publisher removed - using PRESET_TARGET mode with hardcoded assignments
    # ld.add_action(scenario_publisher_node)

    return ld
