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

import os.path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ainstein_pkg_prefix = get_package_share_directory('ainstein_radar_ros2')

    ainstein_param_file = os.path.join(ainstein_pkg_prefix, 'param/defaults.param.yaml')

    ainstein_param = DeclareLaunchArgument(
        'ainstein_radar_driver_node_param_file',
        default_value=ainstein_param_file,
        description='Path to config file to Ainstein Radar Driver'
    )

    # Node
    ainstein_radar_driver_node = Node(
        package='ainstein_radar_ros2',
        executable='ainstein_radar_ros2_exe',
        name='ainstein_radar_driver',
        namespace='radar',
        output='screen',
        parameters=[LaunchConfiguration('ainstein_radar_driver_node_param_file')]
    )

    return LaunchDescription([
        ainstein_param,
        ainstein_radar_driver_node,
    ])
