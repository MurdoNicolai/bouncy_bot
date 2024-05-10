from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            description='Path to the YAML file with node parameters'
        ),

        Node(
            package='my_cartpole_training',
            executable='start_training.py',
            name='cartpole_gym',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])
