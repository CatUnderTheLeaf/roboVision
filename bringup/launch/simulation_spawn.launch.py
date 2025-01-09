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
    
    pkg_project_bringup = FindPackageShare('bringup')
    ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_project_description = FindPackageShare('description')
    pkg_project_simulation = FindPackageShare('simulation')

    # default_model_path  =  PathJoinSubstitution([pkg_project_description, 'models', LaunchConfiguration('model')])
    # default_rviz_config_path = PathJoinSubstitution([pkg_project_bringup, 'config', LaunchConfiguration('rvizconfig')])

    # Launch context if conversion from substitution to string is needed
    launch_context = LaunchContext()

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([
                pkg_project_description,
                'models'
            ])
    )


    
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'lego_bot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.01'
        ],
        output='screen',
    )

    bridge_params = PathJoinSubstitution([
        pkg_project_bringup,
        'config',
        'gazebo_bridge.yaml'
            ])

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # arguments=[
        #     '--ros-args',
        #     '-p',
        #     'config_file:='+bridge_params.perform(launch_context),
        # ],
        parameters=[{
            'config_file': bridge_params.perform(launch_context),
            # 'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'legobot_diff_controller'],
        output='screen'
    )


    # add camera image bridge
    # start_gazebo_ros_image_bridge_cmd = Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=['/camera/image_raw'],
    #     output='screen',
    # )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[
           PathJoinSubstitution([pkg_project_bringup, 'config/ekf.yaml']), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        # TODO
        # model path can be in another package
        DeclareLaunchArgument(
            'model', default_value='legobot/legobot.xacro',
            description='Path to robot urdf file in the models dir'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='Flag to open RViz.'),
        # Launch legobot and rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_project_bringup,
                    'launch',
                    'legobot.launch.py'
                ])
            ),
            launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'model': LaunchConfiguration('model'),
                    'rviz': LaunchConfiguration('rviz'),
                }.items()
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=start_gazebo_ros_spawner_cmd,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),

        
        set_env_vars_resources,

                        
        # Declare spawn arguments
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Specify namespace of the robot'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Specify namespace of the robot'),

        start_gazebo_ros_spawner_cmd,           
                    
        
        start_gazebo_ros_bridge_cmd,    
                       
        # robot_localization_node,
        
        
    ])

