import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    """
    Dynamics-enabled version for 36-drone swarm
    - ego_planner_node (path planning)
    - traj_server (trajectory execution)
    - so3_quadrotor_simulator (physics simulation with dynamics)
    - so3_control (SO3 controller)
    - odom_visualization (RViz visualization)

    NO sensing simulation (pcl_render removed for performance)
    More realistic than minimal version with physics simulation
    """

    # LaunchConfiguration definitions
    map_size_x = LaunchConfiguration('map_size_x', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z', default=5.0)
    init_x = LaunchConfiguration('init_x', default=0.0)
    init_y = LaunchConfiguration('init_y', default=0.0)
    init_z = LaunchConfiguration('init_z', default=0.0)
    target_x = LaunchConfiguration('target_x', default=20.0)
    target_y = LaunchConfiguration('target_y', default=20.0)
    target_z = LaunchConfiguration('target_z', default=1.0)
    drone_id = LaunchConfiguration('drone_id', default=0)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')

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

    # Include advanced parameters (ego_planner_node)
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'realworld_experiment': 'True',  # Wait for trigger for synchronized start
            'obj_num_set': str(0),  # No moving objects
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'pcl_render_node/cloud',
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(1.0),  # Reduced from 1.5 to 1.0 to avoid stationary drone collisions
            'max_acc': str(2.0),  # Reduced from 2.5 to 2.0 for safer deceleration
            'planning_horizon': str(10.0),  # Increased from 7.5 to 10.0 to see further ahead
            'use_distinctive_trajs': 'True',
            'flight_type': str(2),
            'point_num': str(1),
            'point0_x': target_x,
            'point0_y': target_y,
            'point0_z': target_z,
            'point1_x': str(0.0),
            'point1_y': str(15.0),
            'point1_z': str(1.0),
            'point2_x': str(15.0),
            'point2_y': str(0.0),
            'point2_z': str(1.0),
            'point3_x': str(0.0),
            'point3_y': str(-15.0),
            'point3_z': str(1.0),
            'point4_x': str(-15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items()
    )

    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='log',  # Reduce terminal spam
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )

    # SO3 Quadrotor Dynamics Simulator
    so3_quadrotor_simulator = launch_ros.actions.Node(
        package='so3_quadrotor_simulator',
        executable='so3_quadrotor_simulator',
        output='log',  # Reduce terminal spam
        name=['drone_', drone_id, '_quadrotor_simulator_so3'],
        parameters=[
            {'rate/odom': 100.0},
            {'simulator/init_state_x': init_x},
            {'simulator/init_state_y': init_y},
            {'simulator/init_state_z': init_z}
        ],
        remappings=[
            ('odom', ['drone_', drone_id, '_visual_slam/odom']),
            ('cmd', ['drone_', drone_id, '_so3_cmd']),
            ('force_disturbance', ['drone_', drone_id, '_force_disturbance']),
            ('moment_disturbance', ['drone_', drone_id, '_moment_disturbance'])
        ]
    )

    # SO3 Control - Load config files
    gains_file = os.path.join(
        get_package_share_directory('so3_control'),
        'config',
        'gains_hummingbird.yaml'
    )
    corrections_file = os.path.join(
        get_package_share_directory('so3_control'),
        'config',
        'corrections_hummingbird.yaml'
    )

    so3_control_component = launch_ros.descriptions.ComposableNode(
        package='so3_control',
        plugin='SO3ControlComponent',
        name=['drone_', drone_id, '_so3_control_component'],
        parameters=[
            {'so3_control/init_state_x': init_x},
            {'so3_control/init_state_y': init_y},
            {'so3_control/init_state_z': init_z},
            {'mass': 0.98},
            {'use_angle_corrections': False},
            {'use_external_yaw': False},
            {'gains/rot/z': 1.0},
            {'gains/ang/z': 0.1},
            gains_file,
            corrections_file
        ],
        remappings=[
            ('odom', ['drone_', drone_id, '_visual_slam/odom']),
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('motors', ['drone_', drone_id, '_motors']),
            ('corrections', ['drone_', drone_id, '_corrections']),
            ('so3_cmd', ['drone_', drone_id, '_so3_cmd'])
        ]
    )

    so3_control_container = ComposableNodeContainer(
        name=['drone_', drone_id, '_so3_control_container'],
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            so3_control_component
        ],
        output='log'  # Reduce terminal spam
    )

    # Drone visualization for RViz
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=['drone_', drone_id, '_odom_visualization'],
        output='log',
        remappings=[
            ('odom', ['drone_', drone_id, '_', odom_topic]),
            ('robot', ['drone_', drone_id, '_vis/robot']),
            ('path', ['drone_', drone_id, '_vis/path']),
            ('time_gap', ['drone_', drone_id, '_vis/time_gap']),
        ],
        parameters=[
            {'color/a': 1.0},
            {'color/r': 0.0},
            {'color/g': 0.0},
            {'color/b': 0.0},
            {'covariance_scale': 100.0},
            {'robot_scale': 1.0},
            {'tf45': False},
            {'drone_id': drone_id}
        ]
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

    # Add nodes with dynamics simulation
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(so3_quadrotor_simulator)
    ld.add_action(so3_control_container)
    ld.add_action(odom_visualization_node)

    return ld
