# Copyright 2022 Perception for Physical Interaction Laboratory at Poznan University of Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition


def if_equals_condition(arg_name: str, value: str):
    return IfCondition(PythonExpression([
        '"', LaunchConfiguration(arg_name), '" == "', value, '"'
    ]))


def generate_launch_description():
    """Generate launch description with a single component."""

    ainstein_radar_pkg_prefix = get_package_share_directory('ainstein_radar_ros2')
    ros2_socketcan_prefix = get_package_share_directory('ros2_socketcan')

    ainstein_radar_param_file = os.path.join(ainstein_radar_pkg_prefix, 'param/K77.param.yaml')

    # Load the parameters to ComposableNode
    with open(ainstein_radar_param_file, 'r') as file:
        configParams = yaml.safe_load(file)['/**']['ros__parameters']

    can_sender_interface = LaunchConfiguration('sender_interface', default='can0')
    can_sender_receiver = LaunchConfiguration('receiver_interface', default='can0')
    ros2_socketcan_condition = DeclareLaunchArgument(
        'start_socketcan',
        default_value='True',
        description='Launch ros2_socketcan node'
    )

    ainstein_radar_container = ComposableNodeContainer(
        name='ainstein_radar_container',
        namespace=TextSubstitution(text=''),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ainstein_radar_ros2',
                plugin='ainstein_radar::AinsteinRadarNodeK77',
                name='ainstein_radar_ros2_node',
                parameters=[configParams],
            ),
        ],
        output='screen',
    )

    #TODO ad remapping
    ros2_socketcan_sender_launch = GroupAction([IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [ros2_socketcan_prefix, '/launch/socket_can_sender.launch.py']),
        launch_arguments={
            # 'remappings': [('to_can_bus, to_can_bus_1')],
            'interface': can_sender_interface
        }.items(),
        condition=if_equals_condition("start_socketcan", "True")

    )])

    #TODO ad remapping
    ros2_socketcan_receiver_launch = GroupAction([IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [ros2_socketcan_prefix, '/launch/socket_can_receiver.launch.py']),
        launch_arguments={
            'namespace': 'radar',
            # 'remappings': [('from_can_bus, from_can_bus_1')],
            'interface': can_sender_receiver
        }.items(),
        condition=if_equals_condition("start_socketcan", "True")
    )])

    return launch.LaunchDescription([ros2_socketcan_condition,
                                     ros2_socketcan_sender_launch,
                                     ros2_socketcan_receiver_launch,
                                     ainstein_radar_container])
