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
    
    default_model_path  =  PathJoinSubstitution([pkg_project_description, 'models', LaunchConfiguration('model')])
    default_rviz_config_path = PathJoinSubstitution([pkg_project_description, 'config', LaunchConfiguration('rvizconfig')])
    default_controllers_config_path = PathJoinSubstitution([pkg_project_description, 'config', LaunchConfiguration('controllers_config')])

    robot_description = Command(['xacro ', default_model_path, ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware')]) 

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[default_controllers_config_path],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/legobot_diff_controller/cmd_vel", "/cmd_vel"),
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_description}
        ],
        output='both'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["legobot_diff_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz'))
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
        
        control_node,
        robot_state_publisher_node,
        robot_controller_spawner,        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_controller_spawner,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
            )
        ),
    ])