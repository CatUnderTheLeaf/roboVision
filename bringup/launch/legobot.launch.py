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
            'gui', default_value='False',
            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'model', default_value='legobot/legobot_with_controllers.xacro',
            description='Path to robot urdf file in the models dir'),
        DeclareLaunchArgument(
            'controllers_config', default_value='legobot_controllers.yaml',
            description='Path to robot controllers config'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='Flag to open RViz.'),
        DeclareLaunchArgument(
            'rvizconfig', default_value='legobot.rviz',
            description='Absolute path to rviz config file'),
        # !!!!! IMPORTANT !!!!!!
        # If you work on a real robot and don’t have a simulator running, 
        # it is often faster to use the mock_components/GenericSystem hardware component 
        # instead of writing a custom one. Stop the launch file and start it again 
        # with 'use_mock_hardware:=True'
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        ),
        
        # Launch legobot and rviz
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
                    'model': LaunchConfiguration('model'),
                    'gui': LaunchConfiguration('gui'),
                    'controllers_config': LaunchConfiguration('controllers_config'),
                    'rviz': LaunchConfiguration('rviz'),
                    'rvizconfig': LaunchConfiguration('rvizconfig'),
                    'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
                }.items()
        ),
        robot_localization_node

    ])