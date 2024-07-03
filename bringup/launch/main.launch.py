from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    
    config = PathJoinSubstitution([
                    FindPackageShare('bringup'),
                    'config',
                    'camera_info.yaml'
                ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('camera_images'),
                    'launch',
                    'camera.launch.py'
                ])
            ]),
            launch_arguments={
                'image_topic': '/camera/image',
                'camera_url': 'http://192.168.0.30:4747/video?320x240',
                'camera_info_url': config,
                'camera_info_topic': '/camera/camera_info'
            }.items()
        )
    ])