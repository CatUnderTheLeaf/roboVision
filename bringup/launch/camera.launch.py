from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    rate_launch_arg = DeclareLaunchArgument(
        'rate',
        default_value='30'
    )
    image_topic_launch_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image'
    )
    camera_url_launch_arg = DeclareLaunchArgument(
        'camera_url',
        default_value='0'
    )

    return LaunchDescription([
        rate_launch_arg,
        image_topic_launch_arg,
        camera_url_launch_arg,
        Node(
            package='camera_images',
            executable='webcam_publisher',
            name='webCam',
            parameters=[{
                'rate': LaunchConfiguration('rate'),
                'camera_url': LaunchConfiguration('camera_url'),
            }],
            remappings=[
                ('/image', LaunchConfiguration('image_topic'))
            ]
        ),        
    ])