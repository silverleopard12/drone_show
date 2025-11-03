from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'num_drones',
            default_value='1',
            description='Number of drones to record trajectories from'
        ),
        DeclareLaunchArgument(
            'output_folder',
            default_value='trajectories',
            description='Base folder name for trajectory output (will append datetime)'
        ),
        DeclareLaunchArgument(
            'sampling_dt',
            default_value='0.05',
            description='Sampling time interval in seconds (default: 0.05 = 50ms)'
        ),

        # Multi-drone trajectory recorder node
        Node(
            package='traj_recorder',
            executable='traj_recorder_node',
            name='trajectory_recorder',
            output='screen',
            parameters=[{
                'num_drones': LaunchConfiguration('num_drones'),
                'output_folder': LaunchConfiguration('output_folder'),
                'sampling_dt': LaunchConfiguration('sampling_dt'),
                'default_rgb': [255, 255, 255],
                'record_height': 3.0,
            }]
        ),
    ])
