from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='session_09',
            executable='lap_timer_client',
            name='lap_timer_client',
        ),
        Node(
            package='session_09',
            executable='lap_timer_server',
            name='lap_timer_server',
        ),
        Node(
            package='session_09',
            executable='driver',
            name='driver',
        ),
        Node(
            package='session_09',
            executable='wall_finder_service',
            name='wall_finder_service',
        ),
    ])