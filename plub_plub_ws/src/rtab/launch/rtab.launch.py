import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    name = "oak"
    parameters = [
        {
            "frame_id": "base_link",
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_scan": True,
            "subscribe_odom_info": False,
            "subscribe_imu": False,
            "sync_queue_size" : 10,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
            "approx_sync_max_interval":"0.001",
            "visual_odometry" : False,
            "delete_db_on_start" : True,
            "FAST/gpu" : "true",
            "Grid/Sensor" : "0",
            "RGBD/NeighborLinkRefining" : "true",
            "RGBD/ProximityBySpace" : "true",
            "RGBD/AngularUpdate" : "0.05",
            "RGBD/LinearUpdate" : "0.05",
            "Grid/RangeMax" : "11.5",
            "Grid/RangeMin" : "0.5",
            "Reg/Force3DoF" : "true",
            "Reg/Strategy" : "1",    
            'RGBD/ProximityByTime':      'false',   
            'RGBD/ProximityPathMaxNeighbors': '10', 
            'Vis/MinInliers':            '10',       
            'RGBD/OptimizeFromGraphEnd': 'false',  
            'RGBD/OptimizeMaxError':     '4',       
            'Reg/Force3DoF':             'true',   
            'Grid/FromDepth':            'false',      
            'RGBD/LocalRadius':          '5',      
            'Icp/CorrespondenceRatio':   '0.2',    
            'Icp/PM':                    'false',
            'Icp/PointToPlane':          'false',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/VoxelSize':             '0.05'

        }
    ]

    remappings = [
        ("rgb/image", name + "/rgb/image_rect"),
        ("rgb/camera_info", name + "/rgb/camera_info"),
        ("depth/image", name + "/stereo/image_raw"),
    ]

    return [
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=parameters,
            remappings=remappings,
        ),
        
    ]



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="oak"),
        OpaqueFunction(function=launch_setup),
    ])
