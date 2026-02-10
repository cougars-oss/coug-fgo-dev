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
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    multiagent_viz = LaunchConfiguration("multiagent_viz").perform(context)
    auv_ns = LaunchConfiguration("auv_ns").perform(context)

    pkg_share = get_package_share_directory("coug_rviz")

    if multiagent_viz == "true":
        rviz_config_file = os.path.join(pkg_share, "rviz", "multi_rviz_config.rviz")
    else:
        template_path = os.path.join(pkg_share, "rviz", "rviz_config.rviz.template")
        with open(template_path, "r") as f:
            template_content = f.read()

        config_content = template_content.replace("AUV_NS", auv_ns)

        temp_config = tempfile.NamedTemporaryFile(
            mode="w", delete=False, suffix=".rviz"
        )
        temp_config.write(config_content)
        temp_config.close()

        rviz_config_file = temp_config.name

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="coug_rviz",
            executable="odom_to_path",
            name="odom_to_path_node",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "multiagent_viz",
                default_value="false",
                description="Use multi-agent visualization config if true",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
