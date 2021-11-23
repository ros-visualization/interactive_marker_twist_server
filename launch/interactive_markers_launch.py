from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get config type (i.e. linear, planar, or aerial) as argument
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='linear'
    )

    # Append '.yaml' to config type argument to create file name (i.e. 'linear.yaml', 'planar.yaml', or 'aerial.yaml')
    config_file = SetLaunchConfiguration(
        'config_file',
        [LaunchConfiguration('config'), '.yaml']
    )
        
    # Generate path to config file
    config_file_path = PathJoinSubstitution(
        [FindPackageShare('interactive_marker_twist_server'),
        'config',
        LaunchConfiguration('config_file')]
    )
    
    node_interactive_marker_twist_server = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        parameters=[config_file_path],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(config_arg)
    ld.add_action(config_file)
    ld.add_action(node_interactive_marker_twist_server)

    return ld
