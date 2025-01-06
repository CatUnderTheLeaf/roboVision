from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_project_bringup = get_package_share_directory('bringup')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = PathJoinSubstitution([
                FindPackageShare('description'),
                'worlds',
                'small_house.world'
            ])


    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([
                get_package_share_directory('description'),
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
    
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
    
    declare_model = DeclareLaunchArgument(
        'model', default_value='model.xacro',
        description='Absolute path to robot urdf file')

    pkg_project_description = get_package_share_directory('description')
    default_model_path  =  PathJoinSubstitution([pkg_project_description, 'models', 'diff_drive', LaunchConfiguration('model')])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'lego_bot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[
           PathJoinSubstitution([pkg_project_bringup, 'config/ekf.yaml']), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )



    return LaunchDescription([

        set_env_vars_resources,
        gzserver_cmd,
        gzclient_cmd,
        declare_x_position_cmd,
        declare_y_position_cmd,
        declare_model,
        robot_state_publisher_node,
        start_gazebo_ros_spawner_cmd

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('bringup'),
        #             'launch',
        #             'diff_drive.launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'model': arg_model_name,
        #         'rviz': 'False'
        #     }.items()
        # ),

        # robot_localization_node,

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('bringup'),
        #             'launch',
        #             'rviz.launch.py'
        #         ])
        #     ]),            
        # ),
    ])