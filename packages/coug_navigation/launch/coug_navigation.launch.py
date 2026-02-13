# Copyright (c) 2026 BYU FROST Lab
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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    auv_ns = LaunchConfiguration("auv_ns")

    fleet_params = os.path.join(
        os.path.expanduser("~"), "config", "fleet", "coug_navigation_params.yaml"
    )
    auv_params = PythonExpression(
        [
            "os.path.join(os.path.expanduser('~'), 'config', '",
            auv_ns,
            "' + '_params.yaml')",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            Node(
                package="coug_navigation",
                executable="waypoint_follower",
                name="waypoint_follower_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
