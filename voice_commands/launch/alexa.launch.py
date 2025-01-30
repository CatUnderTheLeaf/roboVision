from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    alexa_client = Node(
        package='voice_commands',
        executable='alexa_client',
        name='alexa_client',
        shell=True,

    )

    # test action client 
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
       action_server,
    #    action_client
    ])