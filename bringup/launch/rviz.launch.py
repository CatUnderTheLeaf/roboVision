import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import launch_ros

def generate_launch_description():
        
    pkg_project_bringup = get_package_share_directory('bringup')
    default_rviz_config_path = PathJoinSubstitution([pkg_project_bringup, 'config', 'diff_drive.rviz'])
  
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        rviz_node
    ])