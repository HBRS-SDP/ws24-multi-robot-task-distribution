#!/usr/bin/env python3

"""
Launch file for simulating multiple TurtleBot3 robots with navigation in Gazebo.

This launch file performs the following tasks:
1. Declares launch arguments for configuring the simulation:
   - `number_of_robots`: Number of robots to spawn (default: 2).
   - `use_sim_time`: Whether to use simulation time (default: false).
   - `enable_drive`: Whether to enable the robot drive node (default: false).
   - `enable_rviz`: Whether to enable RViz visualization (default: true).
   - `rviz_config_file`: Path to the RViz configuration file.
   - `nav_params_file`: Path to the navigation parameters file.

2. Sets up the Gazebo simulation environment:
   - Includes Gazebo server and client launch files.
   - Loads the simulation world file.

3. Spawns multiple TurtleBot3 robots in Gazebo:
   - Generates robot namespaces and positions dynamically based on the number of robots.
   - Spawns each robot with its URDF model and state publisher.
   - Ensures sequential spawning of robots using event handlers.

4. Configures navigation for each robot:
   - Launches the navigation stack for each robot with its namespace.
   - Publishes initial pose topics for each robot.

5. Optionally launches RViz and drive nodes for each robot:
   - RViz is launched if `enable_rviz` is true.
   - Drive nodes are launched if `enable_drive` is true.

6. Handles remapping of `/tf` and `/tf_static` topics to avoid conflicts.

Dependencies:
- `turtlebot3_multi_robot`: Provides URDF models and navigation configurations.
- `simulation_env`: Provides the simulation world and RViz configuration.
- `robot_navigation`: Provides navigation parameters and behavior trees.
- `gazebo_ros`: Provides Gazebo integration with ROS2.

Returns:
    LaunchDescription: The complete launch description for the simulation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging


def generate_launch_description():
    ld = LaunchDescription()

    TURTLEBOT3_MODEL = "burger"

    # Namespace and number of robots
    namespace = "robot"

    number_of_robots = LaunchConfiguration("number_of_robots", default="2")
    declare_number_of_robots = DeclareLaunchArgument(
        name="number_of_robots",
        default_value=number_of_robots,
        description="Number of robots",
    )

    # Create a LaunchContext to evaluate LaunchConfiguration
    context = LaunchContext()
    number_of_robots_value = number_of_robots.perform(context)
    num_robots = int(number_of_robots_value)

    # Generate robots list
    robots = [
        {
            "name": f"{namespace}_{i+1}",
            "x_pose": str((i - 1) * 0.75),
            "y_pose": "-0.2",
            "z_pose": "0.01",
        }
        for i in range(num_robots)
    ]

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value=use_sim_time,
        description="Use simulator time",
    )

    enable_drive = LaunchConfiguration("enable_drive", default="false")
    declare_enable_drive = DeclareLaunchArgument(
        name="enable_drive",
        default_value=enable_drive,
        description="Enable robot drive node",
    )

    enable_rviz = LaunchConfiguration("enable_rviz", default="false")
    declare_enable_rviz = DeclareLaunchArgument(
        name="enable_rviz", default_value=enable_rviz, description="Enable rviz launch"
    )

    turtlebot3_multi_robot = get_package_share_directory("turtlebot3_multi_robot")
    package_dir = get_package_share_directory("simulation_env")
    nav_package_dir = get_package_share_directory("robot_navigation")
    nav_launch_dir = os.path.join(turtlebot3_multi_robot, "launch", "nav2_bringup")

    rviz_config_file = LaunchConfiguration("rviz_config_file")
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(package_dir, "rviz", "robot_config.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    urdf = os.path.join(
        turtlebot3_multi_robot, "urdf", "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"
    )

    world = os.path.join(
        package_dir, "worlds", "sdp_world_2.world"
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzserver.launch.py",
            )
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py",
            )
        ),
    )

    params_file = LaunchConfiguration("nav_params_file")
    declare_params_file_cmd = DeclareLaunchArgument(
        "nav_params_file",
        default_value=os.path.join(nav_package_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    ld.add_action(declare_number_of_robots)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": os.path.join(
                    get_package_share_directory("simulation_env"),
                    "maps",
                    "wh_world_2.yaml",
                )
            },
        ],
        remappings=remappings,
    )

    map_server_lifecyle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)

    ######################

    # Remapping is required for state publisher otherwise /tf and /tf_static
    # will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        namespace = ["/" + robot["name"]]

        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package="robot_state_publisher",
            namespace=namespace,
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "publish_frequency": 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )

        # Create spawn call
        spawn_turtlebot3_burger = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-file",
                os.path.join(
                    turtlebot3_multi_robot,
                    "models",
                    "turtlebot3_" + TURTLEBOT3_MODEL,
                    "model.sdf",
                ),
                "-entity",
                robot["name"],
                "-robot_namespace",
                namespace,
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                "0.01",
                "-Y",
                "0.0",
                "-unpause",
            ],
            output="screen",
        )

        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, "bringup_launch.py")
            ),
            launch_arguments={
                "slam": "False",
                "namespace": namespace,
                "use_namespace": "True",
                "map": "",
                "map_server": "False",
                "params_file": params_file,
                "default_bt_xml_filename": os.path.join(
                    get_package_share_directory("nav2_bt_navigator"),
                    "behavior_trees",
                    "navigate_to_pose_w_replanning_and_recovery.xml",
                ),
                "autostart": "true",
                "use_sim_time": use_sim_time,
                "log_level": "warn",
            }.items(),
        )

        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)
            ld.add_action(bringup_cmd)

        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
                        spawn_turtlebot3_burger,
                        turtlebot_state_publisher,
                        bringup_cmd,
                    ],
                )
            )

            ld.add_action(spawn_turtlebot3_event)

        # Save last instance for next RegisterEventHandler
        last_action = spawn_turtlebot3_burger
    ######################

    ######################
    # Start rviz nodes and drive nodes after the last robot is spawned
    for robot in robots:

        namespace = ["/" + robot["name"]]

        # Create a initial pose topic publish call
        message = (
            "{header: {frame_id: map}, pose: {pose: {position: {x: "
            + robot["x_pose"]
            + ", y: "
            + robot["y_pose"]
            + ", z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}"
        )

        initial_pose_cmd = ExecuteProcess(
            cmd=[
                "ros2",
                "topic",
                "pub",
                "-t",
                "3",
                "--qos-reliability",
                "reliable",
                namespace + ["/initialpose"],
                "geometry_msgs/PoseWithCovarianceStamped",
                message,
            ],
            output="screen",
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, "rviz_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "namespace": namespace,
                "use_namespace": "True",
                "rviz_config": rviz_config_file,
                "log_level": "warn",
            }.items(),
            condition=IfCondition(enable_rviz),
        )

        drive_turtlebot3_burger = Node(
            package="turtlebot3_gazebo",
            executable="turtlebot3_drive",
            namespace=namespace,
            output="screen",
            condition=IfCondition(enable_drive),
        )

        # Use RegisterEventHandler to ensure next robot rviz launch happens
        # only after all robots are spawned
        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, rviz_cmd, drive_turtlebot3_burger],
            )
        )

        # Perform next rviz and other node instantiation after the previous intialpose request done
        last_action = initial_pose_cmd

        ld.add_action(post_spawn_event)
        ld.add_action(declare_params_file_cmd)
    ######################

    return ld
