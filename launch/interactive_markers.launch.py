from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():    
    node_interactive_marker_twist_server = Node(
        package="my_package",
        executable="my_node",
        name="twist_marker_server",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(node_interactive_marker_twist_server)

    return ld