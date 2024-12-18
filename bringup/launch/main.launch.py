from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    camera_ns = 'camera'

    yolo_detector = Node(
        package='yolo',
        executable='yolo_detector',
        name='yoloDetector',
        namespace = camera_ns,
        remappings=[
            ('image', 'image_rect_color')
        ]
    )

    show_detected_images = Node(
        package='image_view',
        executable='image_view',
        name='showImage',
        namespace = camera_ns,
        remappings=[
            ('image', 'detected_image')
        ]
    )
    
    return LaunchDescription([
        # launch alexa api and action servers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('voice_commands'),
                    'launch',
                    'alexa.launch.py'
                ])
            ])            
        ),

        # launch ros2mqtt bridge
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('mqtt_communication'),
        #             'launch',
        #             'mqtt.launch.py'
        #         ])
        #     ])            
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('bringup'),
        #             'launch',
        #             'camera.launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'camera_ns': camera_ns,
        #         'camera_url': 'http://192.168.0.30:4747/video?320x240',
        #         'camera_info_url': 'package://bringup/config/camera_info.yaml'
        #     }.items()
        # ),

        # yolo_detector,
        # show_detected_images
    ])