import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    mrtd_pkg_dir = get_package_share_directory('empty_simulation')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

    # Set the world path (use warehouse world from the first launch file)
    world = PathJoinSubstitution([mrtd_pkg_dir, 'worlds', 'warehouse.world'])

    # Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        )
    )

    # Get robot URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindPackageShare("turtlebot3_description")]),
            "/urdf/",
            "turtlebot3_burger.urdf"
        ]
    )

    # Robot state publisher
    robot_state_pub_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }],
    )

    # Joint state publisher
    joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # Spawn the robot in Gazebo
    spawn_robot_gazebo_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'turtlebot3_burger',
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', z_pose],
        output='screen'
    )

    # RViz setup
    rviz_cmd = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([mrtd_pkg_dir, 'rviz', 'turtlebot3_navigation.rviz'])],
    )

    # Static transform publisher for base_link to base_footprint
    static_transform_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"]
    )

    # Return the launch description
    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_use_sim_time_argument,
      #  declare_use_rviz_cmd,
        gzserver_cmd,
        gzclient_cmd,
      #  robot_state_pub_cmd,
      #  joint_state_publisher_cmd,
        spawn_robot_gazebo_cmd,
      #  rviz_cmd,
      #  static_transform_cmd
    ])
