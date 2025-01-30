# ROS2 and Gazebo Launch File for rover

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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

    urdf_temp_path = os.path.join(pkg_share, 'models', 'my_robot.urdf')
    with open(urdf_temp_path, 'w') as urdf_file:
        urdf_file.write(urdf_content)

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Spawn URDF model
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_robot', '-file', urdf_file],
        output='screen'
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
        gazebo_launch,
        spawn_robot
    ])
