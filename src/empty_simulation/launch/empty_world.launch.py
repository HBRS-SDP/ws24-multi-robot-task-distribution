import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    mrtd_pkg_dir = get_package_share_directory('empty_simulation')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')


    # Include Gazebo empty world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_dir, 'launch', 'gazebo.launch.py'])),
        launch_arguments={
            'world_name': PathJoinSubstitution([mrtd_pkg_dir, 'worlds', 'warehouse.world']),
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )

    # Define robot description
    robot_description_param = {
        'robot_description': Command([
            'xacro ',
            PathJoinSubstitution([
                turtlebot3_description_dir, 'urdf', 'turtlebot3_burger.urdf'
            ])
        ])
    }

    # Spawn the robot in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Launch description
    return LaunchDescription([
        gazebo_launch,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description_param],
            output='screen'
        ),
        spawn_robot_node
    ])
