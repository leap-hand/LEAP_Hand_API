from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leap_hand',
            executable='leaphand_node.py',
            name='leaphand_node',
            parameters=[
                {'baudrate': 3000000},
                {'devicename': '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NQ6SW-if00-port0'},
                {'max_publish_frequency': 60.0},
                {'pub_pos': True},
                {'pub_vel': False},
                {'pub_current': False},
                {'kP': 400},
                {'kI': 0},
                {'kD': 200},
                {'curr_lim': 2000},
                {'curr_pos': [0]*16}
            ]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()