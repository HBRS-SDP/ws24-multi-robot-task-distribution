#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    world = os.path.join(
        get_package_share_directory("simulation_env"), "worlds", "sdp_world_2.world"
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("simulation_env"), "rviz", "sdp_wh.rviz"
            )
        ],
    )

    # Get the directory for the simulation environment package
    map_dir = get_package_share_directory("simulation_env")
    map_name = "wh_world_2"
    map_file = os.path.join(map_dir, "maps", map_name + ".yaml")

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

    return LaunchDescription(
        [
            gzclient_cmd,
            gzserver_cmd,
            spawn_turtlebot_cmd,
            robot_state_publisher_cmd,
            map_server,
            lifecycle_manager,
            tf2_ros_map_to_odom,
            rviz_launch_cmd,
            # localization_node,
            # navigation_node,
        ]
    )
