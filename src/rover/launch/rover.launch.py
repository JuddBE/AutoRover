"""
Launch File for rover.
"""

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
    world_path = os.path.join(pkg_share, 'worlds', 'rover.world')
    xacro_path = os.path.join(pkg_share, 'models', 'my_robot.urdf.xacro')
    urdf_content = xacro.process_file(xacro_path).toxml()

    urdf_path = os.path.join(pkg_share, 'models', 'my_robot.urdf')
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(urdf_content)

    # Set the QT_QPA_PLATFORM environment variable, USE THIS IF USING A VM
    # qt_env = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Run RSP to publish robot states from simulation
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content, 'use_sim_time': use_sim_time}]
    )

    # Spawn URDF model
    # spawn_robot = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=['-file', urdf_path,
    #                 # '-x', '0.0',
    #                 # '-y', '0.0',
    #                 # '-z', '7.0',
    #                 ],
    #     output='screen'
    # )

    # Bridge ros and gazebo's topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name="ros_gz_bridge",
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[
        ('cloud/points', 'cloud'),  # Gazebo topic -> ROS topic
        ],
        arguments=['cloud/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    # Rviz running for visualization
    rviz_config_path = os.path.join(pkg_share, 'config', 'rover_config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name="rviz2",
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_path]
    )

    # # # EKF node (Use wheel odometry and IMU to find rover position, replaced by custom-made localization node below which performs much better)
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[os.path.join(pkg_share, 'config', 'ekf_config.yaml')],
    #     remappings=[
    #         ('/odometry/filtered', '/position')
    #     ]
    # )

    # Localization node (Use odometry and IMU to find rover position in map)
    localization = Node(
        package='rover',
        executable='localization',
        name='localization',
        output='screen'
    )

    # Map generation node (Get point cloud from lidar, convert to traversability map)
    mapping = Node(
        package='rover',
        executable='mapping',
        name='mapping',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Path planning node (Use traversability map to find optimal path)
    planning = Node(
        package='rover',
        executable='path_planning',
        name='path_planning',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Path folowing node (Use waypoints to pursue path)
    tracking = Node(
        package='rover',
        executable='path_tracking',
        name='path_tracking',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Rover control node (Follow optimal path)
    control = Node(
        package='rover',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        # qt_env,  # USE THIS IF USING A VM
        gazebo_launch,
        robot_state_publisher,
        # spawn_robot,
        bridge,
        rviz,
        localization,
        mapping,
        planning,
        tracking,
        control
    ])
