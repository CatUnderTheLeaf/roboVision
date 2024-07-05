from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    
    arg_namespace = DeclareLaunchArgument(
        name='camera_ns', default_value='my_camera',
        description=('namespace for all components loaded')
    )

    camera_url_launch_arg = DeclareLaunchArgument(
        'camera_url',
        default_value='0'
    )

    camera_info_url = DeclareLaunchArgument(
        'camera_info_url',
        default_value=''
    )

    camera_ns = LaunchConfiguration('camera_ns')

    # the bug with camera_info_url was fixed on Jun 10, 2024
    # but it is not yet in the package repository,
    # so I downloaded the last version and just changed the name
    image_publisher = Node(
        package='im_publisher', 
        executable='im_publisher_node',
        name='webCam',
        output='screen',
        namespace = camera_ns,
        arguments=[LaunchConfiguration('camera_url')],                      
        parameters=[{'publish_rate': 30.0,
                        'camera_info_url': LaunchConfiguration('camera_info_url')
                    }],
    )

    image_rectifier = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('camera_images'),
                'launch',
                'image_proc.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': 'camera'
        }.items()
    )
    
    show_images = Node(
        package='image_view',
        executable='image_view',
        name='showImage',
        namespace = camera_ns,
        remappings=[
            ('image', 'image_rect_color')
        ]
    )

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
        arg_namespace,
        camera_info_url,
        camera_url_launch_arg,
        image_publisher,
        image_rectifier,
        show_images,
        yolo_detector,
        show_detected_images
    ])
