# Copyright 2011 Willow Garage, Inc.
# Copyright 2013 Mike Purvis
# Copyright 2021 Clearpath Robotics Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#   * Neither the name of the {copyright_holder} nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get config type (i.e. linear, planar, or aerial) as argument
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='linear'
    )

    # Append '.yaml' to config type argument to create file name (i.e. 'linear.yaml', 'planar
    # yaml', or 'aerial.yaml')
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
