import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, LaunchConfiguration)
from launch_ros.actions import (Node, SetParameter)
import random


# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Get Package Description and Directory #
    package_description = "bouncy_bot"
    package_directory = get_package_share_directory(package_description)

    # Load URDF File #
    urdf_file = 'bouncy.xacro'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    print("URDF Loaded !")

    # Robot State Publisher (RSP) #
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': True},
                    {'robot_description': Command(['xacro ', robot_desc_path])}]
    )
    # Generate a random float between for poition X and Y spawns
    randx = (random.random() - 0.5)
    randy = (random.random() - 0.5)
    # Spawn the Robot #
    declare_spawn_x = DeclareLaunchArgument("x", default_value=str(randx),
                                            description="Model Spawn X Axis Value")
    declare_spawn_y = DeclareLaunchArgument("y", default_value=str(randy),
                                            description="Model Spawn Y Axis Value")
    declare_spawn_z = DeclareLaunchArgument("z", default_value="15",
                                            description="Model Spawn Z Axis Value")
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name", "bouncy",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )
        # ROS-Gazebo Bridge #
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
            "/jump_cmd" + "@std_msgs/msg/Float64" + "@ignition.msgs.Double",
            "/turn_cmd" + "@std_msgs/msg/Float64" + "@ignition.msgs.Double",
            "/rot1_cmd" + "@std_msgs/msg/Float64" + "@ignition.msgs.Double",
            "/rot2_cmd" + "@std_msgs/msg/Float64" + "@ignition.msgs.Double",
            "/model/bouncy/pose" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            "/odom" + "@nav_msgs/msg/Odometry" + "[ignition.msgs.Odometry",
            "/laser/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
            "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
        ],
        remappings=[
        ],
        output="screen",
    )
    ld = LaunchDescription(
        [
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo) #
            SetParameter(name="use_sim_time", value=True),
            robot_state_publisher_node,
            declare_spawn_x,
            declare_spawn_y,
            declare_spawn_z,
            gz_spawn_entity,
            ign_bridge,
        ])
    print(Command(['xacro ', robot_desc_path]))

    # Create and Return the Launch Description Object #
    return ld
