from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    alexa_client = Node(
        package='voice_commands',
        executable='alexa_client',
        name='alexa_client',
    )

    
    return LaunchDescription([
       alexa_client
    ])