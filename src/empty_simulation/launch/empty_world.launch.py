import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    mrtd_pkg_dir = get_package_share_directory('empty_simulation')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Launch arguments
    declare_model_arg = DeclareLaunchArgument(
        'model', default_value=os.getenv('TURTLEBOT3_MODEL', 'burger'),
        description='Model type [burger, waffle, waffle_pi]')
    declare_x_pos_arg = DeclareLaunchArgument(
        'x_pos', default_value='0.0', description='Initial x position of the robot')
    declare_y_pos_arg = DeclareLaunchArgument(
        'y_pos', default_value='0.0', description='Initial y position of the robot')
    declare_z_pos_arg = DeclareLaunchArgument(
        'z_pos', default_value='0.0', description='Initial z position of the robot')

    # Include Gazebo empty world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'empty_world.launch.py')),
        launch_arguments={
            'world_name': os.path.join(mrtd_pkg_dir, 'worlds', 'warehouse.world'),
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
            os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_', LaunchConfiguration('model'), '.urdf.xacro'),
            ' --inorder'
        ])
    }

    # Spawn the robot in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        arguments=[
            '-entity', 'turtlebot3_' + LaunchConfiguration('model'),
            '-x', LaunchConfiguration('x_pos'),
            '-y', LaunchConfiguration('y_pos'),
            '-z', LaunchConfiguration('z_pos'),
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Launch description
    return LaunchDescription([
        declare_model_arg,
        declare_x_pos_arg,
        declare_y_pos_arg,
        declare_z_pos_arg,
        gazebo_launch,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description_param],
            output='screen'
        ),
        spawn_robot_node
    ])

