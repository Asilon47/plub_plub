<<<<<<< HEAD
"""Composite launch file that brings up the OAK-D camera, lidar, mecanum
control, CAN, and RTAB-Map SLAM.
"""

import os

=======
import os
>>>>>>> 3c11c9864d18f234fb81bf437f364af2d23b27f3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

<<<<<<< HEAD

def generate_launch_description():
    # Locate the DepthAI driver package to reuse its default parameter file
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    # --- Launch-time arguments ------------------------------------------------
    name_arg = DeclareLaunchArgument(
        "name", default_value="oak", description="Namespace for the camera nodes"
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(depthai_prefix, "config", "rgbd.yaml"),
        description="DepthAI driver parameters YAML",
    )

    # Retrieve argument values once the description is processed
    name = LaunchConfiguration("name")
    params_file = LaunchConfiguration("params_file")

    ld = LaunchDescription([name_arg, params_arg])

    # --- Static transforms ----------------------------------------------------
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_cam_to_base",
            arguments=[
                "0.20",
                "0.00",
                "0.23",
                "0",
                "0",
                "0",
                "base_link",
                "oak-d-base-frame",
            ],
            output="screen",
        )
    )

    # --- Camera driver include -----------------------------------------------
    camera_launch = os.path.join(depthai_prefix, "launch", "camera.launch.py")
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={"name": name, "params_file": params_file}.items(),
        )
    )

    # --- Other subsystem launch files ----------------------------------------
    other_launches = [
        ("mecanum_drive", "mecanum_launch.py"),
        ("can_communication", "can.launch.py"),
        ("ldlidar_stl_ros2", "ld19.launch.py"),
        ("rtab", "rtab.launch.py"),
    ]
    for pkg, lf in other_launches:
        pkg_share = get_package_share_directory(pkg)
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", lf))
            )
        )

    # Static transform for the lidar
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_lidar_to_base",
            arguments=[
                "0.145",
                "0.00",
                "0.255",
                "0",
                "0",
                "0",
                "base_link",
                "base_laser",
            ],
            output="screen",
        )
    )

    return ld
=======
def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    # 1) Declare the same arguments that camera.launch.py expects:
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='oak',
        description='Top-level namespace for the camera nodes'
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(depthai_prefix, 'config', 'rgbd.yaml'),
        description='Path to the DepthAI ROS2 driver parameters YAML'
    )

    # 2) Grab them via LaunchConfiguration
    name = LaunchConfiguration('name')
    params_file = LaunchConfiguration('params_file')

    ld = LaunchDescription([
        name_arg,
        params_arg,
    ])

    # your existing static transform publishers…
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_cam_to_base',
            arguments=[
                '0.20', '0.00', '0.23',
                '0', '0', '0',
                'base_link', 'oak-d-base-frame'
            ],
            output='screen',
        )
    )


    # 3) When including camera.launch.py, pass those arguments along:
    camera_launch = os.path.join(
        depthai_prefix,
        'launch',
        'camera.launch.py'
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                'name': name,
                'params_file': params_file
            }.items()
        )
    )

    # the rest of your other_includes…
    other_launches = [
        ('mecanum_drive',     'mecanum_launch.py'),
        ('can_communication', 'can.launch.py'),
        ('ldlidar_stl_ros2',      'ld19.launch.py'),
        ('rtab',              'rtab.launch.py'),
    ]
    for pkg, lf in other_launches:
        pkg_share = get_package_share_directory(pkg)
        full_path = os.path.join(pkg_share, 'launch', lf)
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(full_path)
            )
        )
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_lidar_to_base',
            arguments=[
                '0.145', '0.00', '0.255',
                '0', '0', '0',
                'base_link', 'base_laser'
            ],
            output='screen',
        )
    )
    return ld

>>>>>>> 3c11c9864d18f234fb81bf437f364af2d23b27f3
