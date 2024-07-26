import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('leap_ros2'),
        'config',
        'default_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='leap_ros2',
            executable='leaphand_node.py',
            name='leaphand_node',
            parameters=[config]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
