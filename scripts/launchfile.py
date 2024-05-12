import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # Get Package Directory
    package_directory = get_package_share_directory("bouncy_bot")
    print("Launch1")

    # Load URDF File
    urdf_file = os.path.join(package_directory, "urdf", "bouncy.xacro")
    print("Launch5")

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, {'robot_description': Command(['xacro ', urdf_file])}]
    )
    print("Launch6")

    # Generate Random Spawn Coordinates
    randx = random.uniform(-0.5, 0.5)
    randy = random.uniform(-0.5, 0.5)

    # Declare Spawn Arguments
    declare_spawn_x = DeclareLaunchArgument('x', default_value=str(randx), description='Model Spawn X Axis Value')
    declare_spawn_y = DeclareLaunchArgument('y', default_value=str(randy), description='Model Spawn Y Axis Value')
    declare_spawn_z = DeclareLaunchArgument('z', default_value='15', description='Model Spawn Z Axis Value')

    # Spawn Robot Node
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='gz_spawn_entity',
        arguments=[
            '-name', 'bouncy',
            '-allow_renaming', 'true',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen',
        respawn=False  # Enable respawn
    )

    # ROS-Gazebo Bridge Node
    ign_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/jump_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
            '/turn_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
            '/rot1_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
            '/rot2_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/bouncy/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/laser/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        output='screen'
    )

    # Launch Description
    ld = LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        robot_state_publisher_node,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        gz_spawn_entity,
        ign_bridge,
    ])

    return ld
