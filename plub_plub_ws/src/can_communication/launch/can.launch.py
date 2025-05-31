from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_communication',
            executable='can_node',
            name='can_node',
            output='screen',
            parameters=[{
                'can_interface': 'can1',
                'tx_can_id': 0x10,
                'rx_can_id': 0x00000001,
                'bitrate': 500000,
                'filter_incoming': True
            }]
        )
    ])
