from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, AppendEnvironmentVariable, LogInfo, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    pkg_project_description = FindPackageShare('description')
    pkg_project_bringup = FindPackageShare('bringup')

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[
           PathJoinSubstitution([pkg_project_bringup, 'config/ekf.yaml']), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    return LaunchDescription([        
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='Flag to open RViz.'),
        DeclareLaunchArgument(
            'sim_legobot', default_value='true',
            description='flag to launch the simulated in RVIZ legobot with ros2_control'),
        
        # Launch simulated with ros2_control legobot and rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_project_description,
                    'launch',
                    'legobot.launch.py'
                ])
            ),
            launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'model': 'legobot/legobot_with_controllers.xacro',
                    'controllers_config': 'legobot_controllers.yaml',
                    'rviz': LaunchConfiguration('rviz'),
                    'rvizconfig': 'legobot.rviz',
                    # !!!!! IMPORTANT !!!!!!
                    # If you work on a real robot and don’t have a simulator running, 
                    # it is often faster to use the mock_components/GenericSystem hardware component 
                    # instead of writing a custom one. Stop the launch file and start it again 
                    # with 'use_mock_hardware:=True'
                    'use_mock_hardware': "false",
                }.items(),
            condition=IfCondition(LaunchConfiguration('sim_legobot'))
        ),

        # Launch real legobot and rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_project_description,
                    'launch',
                    'real_legobot.launch.py'
                ])
            ),
            launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'model': 'legobot/legobot.xacro',
                    'rviz': LaunchConfiguration('rviz'),
                    'rvizconfig': 'legobot.rviz',
                }.items(),
            condition=UnlessCondition(LaunchConfiguration('sim_legobot'))
        ),

        # robot_localization_node

    ])