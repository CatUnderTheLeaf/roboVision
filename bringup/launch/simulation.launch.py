from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    arg_model_name = 'gazebo.xacro'
    pkg_project_bringup = get_package_share_directory('bringup')
  
    spawn_entity = Node(
        package='gazebo_ros',
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
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                            description='Flag to enable use_sim_time'),
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        spawn_entity,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('bringup'),
                    'launch',
                    'diff_drive.launch.py'
                ])
            ]),
            launch_arguments={
                'model': arg_model_name,
                'rviz': 'False'
            }.items()
        ),

        robot_localization_node,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('bringup'),
                    'launch',
                    'rviz.launch.py'
                ])
            ]),            
        ),
    ])