import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('diffbot_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    urdf_path = os.path.join(pkg_share, 'urdf', 'diffbot.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'diffbot', '-topic', 'robot_description', '-z', '0.2'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    return LaunchDescription([gz_sim, robot_state_publisher, spawn, bridge])