import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def load_yaml_file(file_path):
    # Print the path to the YAML file
    print(f"Loading configuration from: {file_path}")

    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
    

def launch_setup(context, *args, **kwargs):
    # Retrieve the path to the YAML file from the config_file argument
    config_file_path = LaunchConfiguration('config_file').perform(context)

    # Load parameters from the YAML file
    config_params = load_yaml_file(config_file_path)

    return [
        Node(
            package='leap_ros2',
            executable='leaphand_node.py',
            name='leaphand_node',
            parameters=[config_params]  # Pass the loaded parameters here
        ),
    ]


def generate_launch_description():
    # Declare the YAML file path as a launch argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(get_package_share_directory('leap_ros2'), 'config', 'default_params.yaml'),
        description='Path to the YAML config file'
    )

    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=launch_setup)  # Ensure the config_file argument is processed at runtime
    ])


if __name__ == '__main__':
    generate_launch_description()