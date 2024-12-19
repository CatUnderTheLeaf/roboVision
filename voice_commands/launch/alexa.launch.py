import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    filename = os.path.join(get_package_share_directory('voice_commands'), 'config',
                            'alexa-skill-config.yaml')

    alexa_client = Node(
        package='voice_commands',
        executable='alexa_client',
        # executable='alexa_interface.py',
        name='alexa_client',
        # arguments=[filename],
        parameters=[{
            'config': filename,
        }],
        # Launch the node with root access (GPIO) in a shell
        # because I want to launch flask app on the port 80
        # prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
        shell=True,

    )

    action_client = Node(
        package='voice_commands',
        executable='action_client',
        name='action_client',
        shell=True,

    )

    action_server = Node(
        package='voice_commands',
        executable='action_server',
        name='action_server',
        shell=True,

    )
    
    return LaunchDescription([
       alexa_client,
    #    action_server,
    #    action_client
    ])