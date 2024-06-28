from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    image_topic_launch_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image'
    )
    camera_url_launch_arg = DeclareLaunchArgument(
        'camera_url',
        default_value='0'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        image_topic_launch_arg,
        camera_url_launch_arg,
        Node(
            package='image_tools',
            executable='showimage',
            name='showImage',             
            remappings=[
                ('/image', LaunchConfiguration('image_topic'))
            ]
        ),         
        Node(
            package='image_publisher', 
            executable='image_publisher_node',
            name='webCam',
            output='screen',
            arguments=[LaunchConfiguration('camera_url')],
            parameters=[{'use_sim_time': use_sim_time,
                         'publish_rate': 30.0}],
            remappings=[('image_raw', LaunchConfiguration('image_topic')),
                        ('camera_info', '/camera/camera_info')]),     
    ])