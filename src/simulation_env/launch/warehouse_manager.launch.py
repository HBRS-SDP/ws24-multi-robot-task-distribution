"""
Launch file for managing a warehouse simulation with multiple robots.

This launch file performs the following tasks:
1. Declares a launch argument:
   - `num_of_robots`: Specifies the number of robots to spawn in the simulation (default: 3).

2. Launches the following nodes:
   - `logger_node`: Logs simulation events and data.
   - `fleet_manager_node`: Manages the fleet of robots, with the number of robots specified by the `num_of_robots` parameter.
   - `shared_memory_node`: Handles shared memory for inter-process communication.
   - `task_manager_node`: Manages tasks and distributes them among the robots.

Dependencies:
- `logger`: Provides the `logger` node for logging.
- `fleet_manager`: Provides the `fleet_manager` node for robot fleet management.
- `shared_memory_node`: Provides the `shared_memory_node` for shared memory operations.
- `task_management`: Provides the `task_manager_node` for task distribution.

Returns:
    LaunchDescription: The complete launch description for the warehouse manager.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the num_of_robots argument
    num_of_robots_arg = DeclareLaunchArgument(
        'num_of_robots',
        default_value='2',
        description='Number of robots to spawn'
    )

    # Get the num_of_robots value from the launch configuration
    num_of_robots = LaunchConfiguration('num_of_robots')

    return LaunchDescription([
        # Declare the num_of_robots argument
        num_of_robots_arg,

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
    ])