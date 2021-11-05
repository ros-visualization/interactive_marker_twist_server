from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("interactive_marker_twist_server"),
        "config",
        "aerial.yaml"],
    )
    
    node_interactive_marker_twist_server = Node(
        package="interactive_marker_twist_server",
        executable="marker_server",
        name="twist_server_node",
        parameters=[config],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(node_interactive_marker_twist_server)

    return ld