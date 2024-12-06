from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with an empty world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/usr/share/gazebo-11/worlds/empty.world'],
            output='screen'
        )
    ])

