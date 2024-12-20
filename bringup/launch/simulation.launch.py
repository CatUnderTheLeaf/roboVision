from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    arg_model_name = 'gazebo.xacro'
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

  
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='spawn_entity.py',
        arguments=['-entity', 'lego_bot', '-topic', 'robot_description'],
        output='screen'
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
        gzclient_cmd



        # DeclareLaunchArgument(name='use_sim_time', default_value='True',
        #                     description='Flag to enable use_sim_time'),
        # ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world], output='screen'),
        
        # spawn_entity,

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