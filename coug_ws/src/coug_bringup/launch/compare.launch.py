# Copyright (c) 2026 BYU FRoSt Lab
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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration("use_sim_time")
    play_bag_path = LaunchConfiguration("play_bag_path")
    record_bag_path = LaunchConfiguration("record_bag_path")

    play_bag_str = context.perform_substitution(play_bag_path)
    record_bag_str = context.perform_substitution(record_bag_path)

    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_fgo_dir = get_package_share_directory("coug_fgo")
    fgo_params_file = os.path.join(coug_fgo_dir, "config", "fgo_params.yaml")

    actions = []

    if record_bag_str:
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-a", "-o", record_bag_str],
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(coug_bringup_dir, "launch", "base.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "multiagent_viz": "false",
            }.items(),
        )
    )

    actions.append(
        Node(
            package="coug_fgo",
            executable="factor_graph",
            name="factor_graph_node",
            namespace="auv0",
            parameters=[
                fgo_params_file,
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": "map",
                    "odom_frame": "auv0/odom",
                    "base_frame": "auv0/base_link",
                },
            ],
        )
    )

    if play_bag_str:
        actions.append(
            ExecuteProcess(
                cmd=["ros2", "bag", "play", play_bag_str],
            )
        )

    return actions


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (HoloOcean) clock if true",
            ),
            DeclareLaunchArgument(
                "play_bag_path",
                default_value="",
                description="Path to rosbag to play (if empty, no playing)",
            ),
            DeclareLaunchArgument(
                "record_bag_path",
                default_value="",
                description="Path to record rosbag (if empty, no recording)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
