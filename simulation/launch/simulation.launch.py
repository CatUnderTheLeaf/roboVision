from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, AppendEnvironmentVariable, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.conditions import IfCondition 
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_project_simulation = FindPackageShare('simulation')

    world = PathJoinSubstitution([
                pkg_project_simulation,
                'worlds',
                LaunchConfiguration('world')
            ])

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([
                pkg_project_simulation,
                'models'
            ])
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    return LaunchDescription([
        set_env_vars_resources,
        DeclareLaunchArgument(
            'world', default_value='small_house.world',
            description='Specify world for the robot'),                
        gzserver_cmd,
        gzclient_cmd,        
    ])

