import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('leap_ros2'),
        'config',
        'default_params.yaml'
    )

    # Print the path to the YAML file
    print(f"Loading configuration from: {config}")
    
    # Read and parse the YAML file
    with open(config, 'r') as file:
        config_params = yaml.safe_load(file)

    # Print the content of the YAML file
    print(config_params)

    return LaunchDescription([
        Node(
            package='leap_ros2',
            executable='leaphand_node.py',
            name='leaphand_node',
            parameters=[config_params]  # Pass the parsed parameters here
        ),
        Node(
            package='leap_ros2',
            executable='demo_repeat_joint_data.py',
            name='demo_repeat_joint_data',
            parameters=[config_params]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
