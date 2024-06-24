import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leap_hand',
            executable='leaphand_node.py',
            name='leaphand_node',
            output='screen',
            parameters=[
                {'kP': 800.0},
                {'kI': 0.0},
                {'kD': 200.0},
                {'curr_lim': 500.0}
            ]
        )
    ])
