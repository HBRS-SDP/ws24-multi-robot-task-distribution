from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the num_of_robots argument
    num_of_robots_arg = DeclareLaunchArgument(
        'num_of_robots',
        default_value='2',
        description='Number of robots to spawn'
    )

    # Get the num_of_robots value from the launch configuration
    num_of_robots = LaunchConfiguration('num_of_robots')

    # Path to the sim_with_nav.launch.py file
    sim_with_nav_launch_path = os.path.join(
        get_package_share_directory('simulation_env'),
        'launch',
        'sim_with_nav.launch.py'
    )

    # Dynamically get the path to the Server.py script
    package_share_directory = get_package_share_directory('web_server')
    server_script_path = os.path.join(package_share_directory, 'web_server', 'Server.py')

    return LaunchDescription([
        # Declare the num_of_robots argument
        num_of_robots_arg,

        # Include the sim_with_nav.launch.py file with the num_of_robots argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_with_nav_launch_path),
            launch_arguments={
                'enable_rviz': 'false',
                'number_of_robots': num_of_robots
            }.items()
        ),

        # Launch the logger node
        Node(
            package='logger',
            executable='logger',
            name='logger_node',
            output='screen'
        ),

        # Launch the fleet_manager node with the num_of_robots parameter
        Node(
            package='fleet_manager',
            executable='fleet_manager',
            name='fleet_manager_node',
            output='screen',
            parameters=[{'num_robots': num_of_robots}]
        ),

        # Launch the shared_memory_node
        Node(
            package='shared_memory_node',
            executable='shared_memory_node',
            name='shared_memory_node',
            output='screen'
        ),

        # Launch the task_manager_node
        Node(
            package='task_management',
            executable='task_manager_node',
            name='task_manager_node',
            output='screen'
        ),

        # Execute the Server.py script using python3
        ExecuteProcess(
            cmd=['python3', server_script_path],
            output='screen'
        ),
    ])