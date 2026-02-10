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

    pkg_share = get_package_share_directory("coug_mapviz")

    if multiagent_viz == "true":
        mapviz_config_file = os.path.join(
            pkg_share, "mapviz", "multi_mapviz_config.mvc"
        )
    else:
        template_path = os.path.join(pkg_share, "mapviz", "mapviz_config.mvc.template")
        with open(template_path, "r") as f:
            template_content = f.read()

        config_content = template_content.replace("AUV_NS", auv_ns)

        temp_config = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".mvc")
        temp_config.write(config_content)
        temp_config.close()

        mapviz_config_file = temp_config.name

    return [
        Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[
                {"config": mapviz_config_file},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("fix", "/origin"),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_origin_transform",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--yaw",
                "0",
                "--pitch",
                "0",
                "--roll",
                "0",
                "--frame-id",
                "map",
                "--child-frame-id",
                "origin",
            ],
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
