from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_drive',
            executable='mecanum_drive_controller',
            name='controller',
            output='screen'
        ),
        Node(
            package='mecanum_drive',
            executable='mecanum_drive_odometry',
            name='odom_publisher',
            output='screen'
        ),
    ])
