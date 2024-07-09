from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_twist_mux',
            executable='twist_mux',
            name='custom_twist_mux',
            parameters=[{
                'sources': ['cmd_vel', 'cmd_vel_joy'],
                'priorities': [2, 1]
            }]
        )
    ])