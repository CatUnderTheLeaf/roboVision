from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    
    image_topic_launch_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image'
    )
    camera_url_launch_arg = DeclareLaunchArgument(
        'camera_url',
        default_value='0'
    )
    camera_info_launch_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value=''
    )
    camera_info_topic_launch_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera_info'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  
    config = DeclareLaunchArgument(
        'camera_info_url',
        default_value=''
    )


    return LaunchDescription([
        image_topic_launch_arg,
        camera_url_launch_arg,
        camera_info_launch_arg,
        camera_info_topic_launch_arg,
        config,
        Node(
            package='camera_images',
            executable='camera_info_publisher',
            name='cameraInfoPublisher',
            output='screen',
            parameters=[{'camera_info_url': LaunchConfiguration('camera_info_url'),
                         }],
            remappings=[
                ('/camera_info', LaunchConfiguration('camera_info_topic'))
            ]
        ),
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
                         'publish_rate': 30.0,
                         }],
            remappings=[('image_raw', LaunchConfiguration('image_topic')),
                        ('camera_info', 'false_camera_info')]),
        
    ])