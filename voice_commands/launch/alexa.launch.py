import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    # maybe in the future i will look into this
    # it sends this path as an argument
    # but python doesn't see it in sys.argv

    # filename = os.path.join(get_package_share_directory('voice_commands'), 'config',
    #                         'alexa-skill-config.yaml')

    alexa_client = Node(
        package='voice_commands',
        executable='alexa_client',
        name='alexa_client',
        # arguments=[filename],
        # Launch the node with root access (GPIO) in a shell
        # because I want to launch flask app on the port 80
        prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
        shell=True,

    )

    
    return LaunchDescription([
       alexa_client
    ])