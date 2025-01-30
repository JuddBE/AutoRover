# ROS2 and Gazebo Launch File for rover

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import subprocess
import xacro
import os

def generate_launch_description():
    # Paths to necessary files
    pkg_share = get_package_share_directory('rover')
    world_path = os.path.join(pkg_share, 'worlds', 'rover.sdf')
    xacro_path = os.path.join(pkg_share, 'models', 'my_robot.urdf.xacro')
    urdf_content = xacro.process_file(xacro_path).toxml()

    urdf_path = os.path.join(pkg_share, 'models', 'my_robot.urdf')
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(urdf_content)

    # Set the QT_QPA_PLATFORM environment variable
    qt_env = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': f'-v 4 -r {world_path}'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content}]
    )

    # Spawn URDF model
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_robot', '-file', urdf_path],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    rviz_config_path = os.path.join(pkg_share, 'config', 'rover_config.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name="rviz2",
        output='log',
        arguments=['-d', rviz_config_path]
    )

    # Example ROS 2 node
    # example_node = Node(
    #     package='your_package_name',
    #     executable='example_node_executable',
    #     name='example_node',
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_path, description='Path to the SDF world file'),
        qt_env,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz
    ])
