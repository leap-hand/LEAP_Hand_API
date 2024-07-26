import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the YAML file
    config_file_path = os.path.join(
        get_package_share_directory('leap_ros2'),
        'config',
        'default_params.yaml'
    )

    # Print the path to the YAML file
    print(f"Loading configuration from: {config_file_path}")

    # Read and parse the YAML file
    with open(config_file_path, 'r') as file:
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
    ])

if __name__ == '__main__':
    generate_launch_description()
