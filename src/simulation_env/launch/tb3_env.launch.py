#!/usr/bin/env python3
# Authors: Deebul Nair

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
    # lc = LaunchContext()
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

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

    # x_pose = LaunchConfiguration('x_pose', default='0.0')
    # y_pose = LaunchConfiguration('y_pose', default='0.0')
    x_pose = LaunchConfiguration("x_pose", default="-9.85")
    y_pose = LaunchConfiguration("y_pose", default="-6.07")
    #yaw_angle = LaunchConfiguration("yaw_angle", default="0.0")
    '''
    world = os.path.join(
        get_package_share_directory('robile_gazebo'),
        'worlds',
        'potential_field_gazebo.world'
    )
    '''

    world = os.path.join(
        get_package_share_directory('simulation_env'),
        'worlds',
        'amr_test_world.world'
        )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # # Get URDF via xacro
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("robile_description"),
    #                 "gazebo",
    #                 "gazebo_robile_laserscanner_camera.xacro"
    #             ]
    #         ),
    #         " ",
    #         "platform_config:=4_wheel_config",
    #         " ",
    #         "movable_joints:=False",
    #     ]
    # )

    # robot_state_pub_cmd = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[{
    #             'use_sim_time': use_sim_time,
    #             'robot_description': ParameterValue(robot_description_content, value_type=str)
    #     }],
    # )

    joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # spawn_robot_gazebo_cmd = Node(package='gazebo_ros',
    #                               executable='spawn_entity.py',
    #                               arguments=['-topic', 'robot_description',
    #                                          '-entity', 'robile',
    #                                          '-x', x_pose,
    #                                          '-y', y_pose,
    #                                          ],
    #                               output='screen')

    rviz_cmd = Node(package='rviz2',
                    namespace='',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    )

    static_transform_cmd = Node(package="tf2_ros",
                                executable="static_transform_publisher",
                                output="screen",
                                arguments=["0", "0", "0", "0", "0",
                                           "0", "base_footprint", "base_link"]
                                )

    sdp_nav_dir = get_package_share_directory("simulation_env")
    map_name = "map_layout_4"
    map_file = os.path.join(sdp_nav_dir, "maps", map_name + ".yaml")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"yaml_filename": map_file},
            {"topic_name": "map"},
            {"frame_id": "map"},
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapper",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    tf2_ros_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0.0", "0.0", "0", "0", "0", "0", "map", "odom"],
    )

    '''
        tf2_ros_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0.0", "0.0", "0", "0", "0", "0", "map", "odom"],
    )
    '''



    nodes = [
        rviz_cmd,
        gzserver_cmd,
        gzclient_cmd,
        # robot_state_pub_cmd,
        # spawn_robot_gazebo_cmd,
        static_transform_cmd,
        joint_state_publisher_cmd,
        #Below things were  added for Astar assignment
        map_server,
        lifecycle_manager,
        tf2_ros_map_to_odom,
        spawn_turtlebot_cmd
    ]

    return LaunchDescription(nodes)

    # urdf_path = os.path.join(
    #    get_package_share_directory('robile_description'),
    #    'robots',
    #    urdf_file_name)

    # with open(urdf_path, 'r') as infp:
    #    robot_desc = infp.read()

    # return LaunchDescription([
    #    DeclareLaunchArgument(
    #        'use_sim_time',
    #        default_value='false',
    #        description='Use simulation (Gazebo) clock if true'),

    #    Node(
    #        package='robot_state_publisher',
    #        executable='robot_state_publisher',
    #        name='robot_state_publisher',
    #        output='screen',
    #        parameters=[{
    #            'use_sim_time': use_sim_time,
    #            'robot_description': robot_desc
    #        }],
    #    ),
    # ])