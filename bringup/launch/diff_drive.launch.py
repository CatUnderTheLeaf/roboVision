import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
        
    pkg_project_description = get_package_share_directory('description')
    default_model_path  =  PathJoinSubstitution([pkg_project_description, 'models', 'diff_drive', LaunchConfiguration('model')])
  
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        # parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )
    
    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
        #                                     description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value='model.xacro',
                                            description='Absolute path to robot urdf file'),

        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
    ])