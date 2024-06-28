from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    
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
                # droidCam has max rate of 15 Hz    
                # 'rate': '15',
                'image_topic': '/camera_image',
                # 'image_topic': '/image',
                'camera_url': 'http://192.168.0.30:4747/video?320x240'
            }.items()
        )
    ])