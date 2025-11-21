import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Map parameters - adjusted for 36-drone formation
    map_size_x = LaunchConfiguration('map_size_x', default = 10.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 80.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 40.0)
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    use_mockamap = LaunchConfiguration('use_mockamap', default=False)
    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    # CAT-ORA parameters
    catora_max_vel = LaunchConfiguration('catora_max_vel', default=2.0)
    catora_max_acc = LaunchConfiguration('catora_max_acc', default=2.0)
    catora_traj_dt = LaunchConfiguration('catora_traj_dt', default=0.2)

    catora_max_vel_cmd = DeclareLaunchArgument('catora_max_vel', default_value=catora_max_vel, description='CAT-ORA max velocity')
    catora_max_acc_cmd = DeclareLaunchArgument('catora_max_acc', default_value=catora_max_acc, description='CAT-ORA max acceleration')
    catora_traj_dt_cmd = DeclareLaunchArgument('catora_traj_dt', default_value=catora_traj_dt, description='CAT-ORA trajectory dt')

    # Map generator node
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 10.0},
            {'map/y_size': 80.0},
            {'map/z_size': 40.0},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 0},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 0},
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
            {'y_length': 80},
            {'z_length': 40},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1}
        ],
        condition = IfCondition(use_mockamap)
    )

    # CAT-ORA Formation Node
    catora_node = Node(
        package='catora_planner',
        executable='catora_planner_node',
        name='catora_planner_node',
        output='screen',
        parameters=[{
            'max_velocity': catora_max_vel,
            'max_acceleration': catora_max_acc,
            'trajectory_dt': catora_traj_dt,
        }],
        remappings=[
            ('~/get_assignment', '/catora_planner/get_assignment'),
            ('~/get_reshaping_trajectories', '/catora_planner/get_reshaping_trajectories'),
        ]
    )

    # Layer separation configuration for 36 drones
    # 36 drones = 6 layers (6 drones per layer)
    # Symmetric distribution: -1.25m, -0.75m, -0.25m, +0.25m, +0.75m, +1.25m
    LAYER_COUNT_36 = 6
    LAYER_OFFSET_36 = 0.5
    LAYER_CENTER_OFFSET_36 = -1.25

    # Drone configurations - 36 drones
    # Init: 6x6 Grid formation (FORMATION_36_A) - Y: 5~30m, Z: 10~35m
    # Target: Triangle formation (FORMATION_36_B) - Y: 15~65m, Z: 10~35m
    # Generated by CAT-ORA Standalone Algorithm - Max distance: 35.00m, Avg: 23.13m
    # Collision-aware assignment with bottleneck optimization
    drone_configs = [
        {'drone_id': 0, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 35.0, 'target_z': 20.0},  # Distance: 31.62m
        {'drone_id': 1, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 35.0, 'target_z': 15.0},  # Distance: 25.50m
        {'drone_id': 2, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 45.0, 'target_z': 20.0},  # Distance: 31.62m
        {'drone_id': 3, 'init_x': 3.0, 'init_y': 20.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 40.0, 'target_z': 15.0},  # Distance: 20.62m
        {'drone_id': 4, 'init_x': 3.0, 'init_y': 25.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 45.0, 'target_z': 15.0},  # Distance: 20.62m
        {'drone_id': 5, 'init_x': 3.0, 'init_y': 30.0, 'init_z': 10.0, 'target_x': 3.0, 'target_y': 40.0, 'target_z': 10.0},  # Distance: 10.00m
        {'drone_id': 6, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 35.0, 'target_z': 25.0},  # Distance: 31.62m
        {'drone_id': 7, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 30.0, 'target_z': 20.0},  # Distance: 20.62m
        {'drone_id': 8, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 40.0, 'target_z': 20.0},  # Distance: 25.50m
        {'drone_id': 9, 'init_x': 3.0, 'init_y': 20.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 50.0, 'target_z': 25.0},  # Distance: 31.62m
        {'drone_id': 10, 'init_x': 3.0, 'init_y': 25.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 55.0, 'target_z': 25.0},  # Distance: 31.62m
        {'drone_id': 11, 'init_x': 3.0, 'init_y': 30.0, 'init_z': 15.0, 'target_x': 3.0, 'target_y': 50.0, 'target_z': 20.0},  # Distance: 20.62m
        {'drone_id': 12, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 25.0, 'target_z': 25.0},  # Distance: 20.62m
        {'drone_id': 13, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 30.0, 'target_z': 25.0},  # Distance: 20.62m
        {'drone_id': 14, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 40.0, 'target_z': 25.0},  # Distance: 25.50m
        {'drone_id': 15, 'init_x': 3.0, 'init_y': 20.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 45.0, 'target_z': 25.0},  # Distance: 25.50m
        {'drone_id': 16, 'init_x': 3.0, 'init_y': 25.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 55.0, 'target_z': 30.0},  # Distance: 31.62m
        {'drone_id': 17, 'init_x': 3.0, 'init_y': 30.0, 'init_z': 20.0, 'target_x': 3.0, 'target_y': 60.0, 'target_z': 30.0},  # Distance: 31.62m
        {'drone_id': 18, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 25.0, 'target_z': 30.0},  # Distance: 20.62m
        {'drone_id': 19, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 30.0, 'target_z': 30.0},  # Distance: 20.62m
        {'drone_id': 20, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 35.0, 'target_z': 30.0},  # Distance: 20.62m
        {'drone_id': 21, 'init_x': 3.0, 'init_y': 20.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 40.0, 'target_z': 30.0},  # Distance: 20.62m
        {'drone_id': 22, 'init_x': 3.0, 'init_y': 25.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 45.0, 'target_z': 30.0},  # Distance: 20.62m
        {'drone_id': 23, 'init_x': 3.0, 'init_y': 30.0, 'init_z': 25.0, 'target_x': 3.0, 'target_y': 50.0, 'target_z': 30.0},  # Distance: 20.62m
        {'drone_id': 24, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 35.0, 'target_z': 35.0},  # Distance: 30.41m
        {'drone_id': 25, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 40.0, 'target_z': 35.0},  # Distance: 30.41m
        {'drone_id': 26, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 20.0, 'target_z': 30.0},  # Distance: 5.00m
        {'drone_id': 27, 'init_x': 3.0, 'init_y': 20.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 50.0, 'target_z': 35.0},  # Distance: 30.41m
        {'drone_id': 28, 'init_x': 3.0, 'init_y': 25.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 55.0, 'target_z': 35.0},  # Distance: 30.41m
        {'drone_id': 29, 'init_x': 3.0, 'init_y': 30.0, 'init_z': 30.0, 'target_x': 3.0, 'target_y': 60.0, 'target_z': 35.0},  # Distance: 30.41m
        {'drone_id': 30, 'init_x': 3.0, 'init_y': 5.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 15.0, 'target_z': 35.0},  # Distance: 10.00m
        {'drone_id': 31, 'init_x': 3.0, 'init_y': 10.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 20.0, 'target_z': 35.0},  # Distance: 10.00m
        {'drone_id': 32, 'init_x': 3.0, 'init_y': 15.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 25.0, 'target_z': 35.0},  # Distance: 10.00m
        {'drone_id': 33, 'init_x': 3.0, 'init_y': 20.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 30.0, 'target_z': 35.0},  # Distance: 10.00m
        {'drone_id': 34, 'init_x': 3.0, 'init_y': 25.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 45.0, 'target_z': 35.0},  # Distance: 20.00m
        {'drone_id': 35, 'init_x': 3.0, 'init_y': 30.0, 'init_z': 35.0, 'target_x': 3.0, 'target_y': 65.0, 'target_z': 35.0},  # Distance: 35.00m
    ]


    drone_nodes = []

    for config in drone_configs:
        drone_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ego_planner'), 'launch', 'run_in_sim.launch.py')),
            launch_arguments={
                'drone_id': str(config['drone_id']),
                'init_x': str(config['init_x']),
                'init_y': str(config['init_y']),
                'init_z': str(config['init_z'] + LAYER_CENTER_OFFSET_36 + (config['drone_id'] % LAYER_COUNT_36) * LAYER_OFFSET_36),
                'target_x': str(config['target_x']),
                'target_y': str(config['target_y']),
                'target_z': str(config['target_z']),
                'map_size_x': map_size_x,
                'map_size_y': map_size_y,
                'map_size_z': map_size_z,
                'odom_topic': odom_topic,
                'planning_horizon': '15.0'
            }.items()
        )
        drone_nodes.append(drone_launch)

    # Swarm Synchronizer
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        get_package_share_directory('ego_planner')))))
    swarm_sync_script = os.path.join(project_root, 'scripts', 'swarm_synchronizer.py')

    swarm_synchronizer = ExecuteProcess(
        cmd=['python3', swarm_sync_script, '--ros-args', '-p', 'num_drones:=36', '-p', 'wait_time:=5.0'],
        output='screen'
    )

    # Mission Timer
    mission_timer_script = os.path.join(project_root, 'scripts', 'mission_timer.py')

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

    # Trajectory Recorder - automatically records all drone trajectories
    trajectory_recorder_node = Node(
        package='traj_recorder',
        executable='traj_recorder_node',
        name='trajectory_recorder',
        output='screen',
        parameters=[{
            'num_drones': 36,
            'output_folder': 'trajectories_catora_36_drones',
            'sampling_dt': 0.02,  # 50Hz sampling
            'default_rgb': [255, 255, 255],
        }]
    )

    ld = LaunchDescription()

    # Add arguments
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(use_mockamap_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(catora_max_vel_cmd)
    ld.add_action(catora_max_acc_cmd)
    ld.add_action(catora_traj_dt_cmd)

    # Add CAT-ORA node
    ld.add_action(catora_node)

    # Add swarm synchronizer and mission timer
    ld.add_action(swarm_synchronizer)
    ld.add_action(mission_timer)

    # Add trajectory recorder
    ld.add_action(trajectory_recorder_node)

    # Add drone nodes
    for drone in drone_nodes:
        ld.add_action(drone)

    return ld
